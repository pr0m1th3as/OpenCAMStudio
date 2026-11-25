//! MCP transports run on workers; desktop commands execute on the iced event loop.
use super::*;
use crate::mcp::desktop_link::{authenticate, new_token, Endpoint};
use crate::mcp::Command;
use crate::mcp::{self as api, *};
use serde_json::{json, Value};
use std::io::{self, BufReader};
use std::net::{Ipv4Addr, Shutdown, TcpListener, TcpStream};
use std::sync::mpsc::{self, Receiver, SyncSender, TryRecvError};
use std::thread::{self, JoinHandle};

pub(super) struct Request {
    name: String,
    arguments: Value,
    reply: SyncSender<CommandResult>,
}

pub(super) enum DesktopMcp {
    Stdio(Receiver<Request>),
    Local(LocalServer),
}

pub(super) fn start() -> DesktopMcp {
    let (send, receive) = mpsc::sync_channel(1);
    thread::spawn(move || {
        let result = serve_desktop(io::stdin().lock(), io::stdout().lock(), &send);
        if let Err(error) = result {
            eprintln!("OpenCAMStudio MCP: {error}");
        }
    });
    DesktopMcp::Stdio(receive)
}

fn serve_desktop(
    reader: impl io::BufRead,
    writer: impl io::Write,
    send: &SyncSender<Request>,
) -> io::Result<()> {
    let tools = api::tools()
        .into_iter()
        .chain(catalog().iter().map(Command::tool))
        .collect();
    api::serve(reader, writer, tools, |name, arguments| {
        let (reply, result) = mpsc::sync_channel(1);
        send.send(Request {
            name: name.into(),
            arguments,
            reply,
        })
        .map_err(|_| "desktop session closed".to_string())?;
        result
            .recv()
            .map_err(|_| "desktop session closed".to_string())?
    })
}

pub(super) struct LocalServer {
    listener: TcpListener,
    port: u16,
    token: String,
    send: SyncSender<Request>,
    receive: Receiver<Request>,
    connection: Option<(TcpStream, JoinHandle<()>)>,
    _endpoint_file: tempfile::NamedTempFile,
}

impl LocalServer {
    fn start(directory: &Path) -> io::Result<Self> {
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0))?;
        listener.set_nonblocking(true)?;
        let port = listener.local_addr()?.port();
        let (send, receive) = mpsc::sync_channel(1);
        let endpoint = Endpoint {
            port,
            token: new_token()?,
        };
        let endpoint_file = endpoint.publish(directory)?;
        Ok(Self {
            listener,
            port,
            token: endpoint.token,
            send,
            receive,
            connection: None,
            _endpoint_file: endpoint_file,
        })
    }

    fn accept(&mut self) -> io::Result<()> {
        if self
            .connection
            .as_ref()
            .is_some_and(|(_, worker)| !worker.is_finished())
        {
            return Ok(());
        }
        self.connection = None;
        let stream = match self.listener.accept() {
            Ok((stream, _)) => stream,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::Interrupted
                        | io::ErrorKind::ConnectionAborted
                ) =>
            {
                return Ok(())
            }
            Err(error) => return Err(error),
        };
        stream.set_nonblocking(false)?;
        let socket = stream.try_clone()?;
        let send = self.send.clone();
        let token = self.token.clone();
        let worker = thread::Builder::new()
            .name("mcp-desktop".into())
            .spawn(move || {
                if authenticate(&stream, &token).is_ok() {
                    let _ = serve_desktop(BufReader::new(&stream), &stream, &send);
                }
                // Wake the client even while the GUI still owns its socket handle.
                let _ = stream.shutdown(Shutdown::Both);
            })?;
        self.connection = Some((socket, worker));
        Ok(())
    }

    pub(super) fn port(&self) -> u16 {
        self.port
    }

    fn configuration(&self) -> io::Result<String> {
        let executable = std::env::current_exe()?
            .with_file_name(format!("opencamstudio-mcp{}", std::env::consts::EXE_SUFFIX));
        #[cfg(target_os = "linux")]
        let (executable, environment) = {
            if let Some(image) = std::env::var_os("APPIMAGE")
                .map(PathBuf::from)
                .filter(|path| path.is_file())
            {
                // Launch the AppImage itself. Its mount path changes each time.
                (image, None)
            } else {
                let environment = executable
                    .ancestors()
                    .nth(3)
                    .filter(|path| path.join("AppRun").is_file())
                    .and_then(|path| {
                        path.join("usr/lib")
                            .to_str()
                            .map(|p| json!({"LD_LIBRARY_PATH": p}))
                    });
                (executable, environment)
            }
        };
        let command = executable.to_str().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                "the executable path is not UTF-8",
            )
        })?;
        let server = json!({"command": command, "args": ["--mcp-connect"]});
        #[cfg(target_os = "linux")]
        let server = {
            let mut server = server;
            if let Some(environment) = environment {
                server["env"] = environment;
            }
            server
        };
        Ok(serde_json::to_string_pretty(
            &json!({"mcpServers": {"opencamstudio": server}}),
        )?)
    }
}

impl Drop for LocalServer {
    fn drop(&mut self) {
        if let Some((stream, _)) = &self.connection {
            let _ = stream.shutdown(Shutdown::Both);
        }
    }
}

impl App {
    pub(super) fn restore_local_mcp(&mut self) {
        if !self.settings.mcp_enabled {
            return;
        }
        match settings_path().and_then(|path| self.restore_local_mcp_from(&path)) {
            Ok(()) => {}
            Err(error) => {
                let message = format!("Could not start MCP: {error}");
                self.status = message.clone();
                self.mcp_feedback = Some(message);
                self.sync_output();
            }
        }
    }

    fn restore_local_mcp_from(&mut self, path: &Path) -> io::Result<()> {
        if self.settings.mcp_enabled && self.mcp.is_none() {
            self.mcp = Some(DesktopMcp::Local(LocalServer::start(config_directory(
                path,
            )?)?));
        }
        Ok(())
    }

    pub(super) fn save_preferences(&mut self, settings: crate::Settings) -> io::Result<()> {
        self.save_preferences_to(settings, &settings_path()?)
    }

    pub(super) fn save_preferences_to(&mut self, settings: crate::Settings, path: &Path) -> io::Result<()> {
        let directory = config_directory(path)?;
        std::fs::create_dir_all(directory)?;
        let pending = tempfile::NamedTempFile::new_in(directory)?;
        settings.save_to(pending.path())?;
        let mcp_changed = settings.mcp_enabled != self.settings.mcp_enabled;
        let server = if mcp_changed && settings.mcp_enabled && self.mcp.is_none() {
            Some(LocalServer::start(directory)?)
        } else {
            None
        };
        pending.persist(path)?;
        self.settings = settings;
        self.controller.set_block_export_on_verification_errors(
            self.settings.block_export_on_verification_errors,
        );
        if mcp_changed {
            self.mcp_feedback = None;
            if let Some(server) = server {
                self.mcp = Some(DesktopMcp::Local(server));
            } else if !self.settings.mcp_enabled && matches!(self.mcp, Some(DesktopMcp::Local(_))) {
                self.mcp = None;
            }
        }
        Ok(())
    }

    pub(super) fn copy_local_mcp_configuration(&mut self) -> iced::Task<Message> {
        if let Some(DesktopMcp::Local(server)) = &self.mcp {
            match server.configuration() {
                Ok(config) => {
                    self.mcp_feedback = Some("Client configuration copied.".into());
                    return iced::clipboard::write(config);
                }
                Err(error) => {
                    self.mcp_feedback =
                        Some(format!("Could not create client configuration: {error}"));
                }
            }
        }
        iced::Task::none()
    }

    pub(super) fn poll_mcp(&mut self) -> iced::Task<Message> {
        let request = match &mut self.mcp {
            Some(DesktopMcp::Stdio(receive)) => match receive.try_recv() {
                Err(TryRecvError::Disconnected) => return iced::exit(),
                result => result,
            },
            Some(DesktopMcp::Local(server)) => {
                if let Err(error) = server.accept() {
                    let message = format!("Local MCP stopped: {error}");
                    self.status = message.clone();
                    self.mcp_feedback = Some(message);
                    self.mcp = None;
                    return iced::Task::none();
                }
                server.receive.try_recv()
            }
            None => return iced::Task::none(),
        };
        let Ok(request) = request else {
            return iced::Task::none();
        };
        let result = self.mcp_call(&request.name, request.arguments);
        if let Err(error) = &result {
            self.status = format!("MCP {}: {error}", request.name);
        }
        let _ = request.reply.send(result);
        self.sync_output();
        self.sync_inspector_visibility();
        std::mem::take(&mut self.mcp_task)
    }

    fn mcp_call(&mut self, name: &str, args: Value) -> CommandResult {
        if let Some(command) = catalog().into_iter().find(|c| c.name == name) {
            return command.invoke(self, args);
        }
        let shop = api::shop_tools().iter().any(|t| t["name"] == name);
        let read_only = if shop {
            api::shop_tools()
                .iter()
                .any(|t| t["name"] == name && t["annotations"]["readOnlyHint"] == true)
        } else {
            api::is_read_only(name)
        };
        if !read_only
            && !matches!(name, "cancel_run" | "cancel_cad_import" | "save_shop")
            && (self.setup_draft_dirty() || self.inspector_dirty() || self.pattern_draft.is_some())
        {
            return Err("Apply or cancel desktop editor changes before issuing an MCP edit. Use desktop_command with cancel_editor to discard them.".into());
        }
        let revision = self.controller.revision();
        let active = self.controller.active_setup_id();
        let selection = self.controller.selection();
        let result = if shop {
            api::call_shop(
                &mut api::Shop {
                    controller: &mut self.controller,
                    library: &mut self.library,
                    machines: &mut self.machines,
                    settings: &mut self.settings,
                },
                name,
                args,
            )
        } else {
            api::call(&mut self.controller, name, args)
        };
        if result.is_ok() {
            if matches!(
                name,
                "new_project" | "open_project" | "commit_cad_preview" | "import_cad" | "open_dxf"
            ) {
                self.reset_model_browser();
                self.view = ViewControls::default();
            }
            if matches!(name, "update_preferences" | "restore_preferences") {
                self.apply_mcp_preferences();
            }
            if shop && !read_only {
                self.reload_tool_edit();
                if let Some(player) = &mut self.simulation {
                    player.sync_force_material(&self.library);
                }
            }
            if self.controller.revision() != revision
                || self.controller.active_setup_id() != active
                || self.controller.selection() != selection
            {
                self.operation_draft = None;
                self.operation_geometry = None;
                self.operation_binding_preview = None;
                self.stock_to_leave_draft = None;
                self.rebase_setup_draft();
                self.height_draft_id = None;
                self.operation_tools.clear();
                if let Selection::Operation(id) = self.controller.selection() {
                    if self.controller.pending_op().is_none() {
                        self.begin_operation_draft(id);
                    } else {
                        self.refresh_fields();
                    }
                } else {
                    self.refresh_fields();
                }
                self.viewport_cache.replace(None);
                self.cad_scene_cache.replace(None);
                self.pending_simulation = None;
                self.pending_toolpath_export = None;
                self.pending_processing_time = None;
                self.processing_time_dialog = None;
            }
            if self
                .simulation
                .as_ref()
                .is_some_and(|player| !player.current(&self.controller))
            {
                self.simulation = None;
            }
            if matches!(
                name,
                "cancel_run" | "start_run" | "start_toolpath_run" | "start_pattern_toolpath_run"
            ) {
                self.pending_simulation = None;
                self.pending_toolpath_export = None;
                self.pending_processing_time = None;
                self.processing_time_dialog = None;
            }
            if matches!(
                name,
                "activate_machine"
                    | "set_machine"
                    | "update_machine"
                    | "open_project"
                    | "update_library_machine"
                    | "delete_machine"
                    | "set_post_kind"
                    | "set_estlcam_options"
            ) {
                self.active_machine = self.controller.machine().name.clone();
                self.machine_name_edit = None;
                self.machine_post_edit = None;
                self.machine_estlcam_edit = None;
                self.refresh_fields();
            }
            self.status = format!("MCP: {name}");
        }
        result
    }

    fn apply_mcp_preferences(&mut self) {
        self.sync_workspace_preferences();
        self.apply_fixed_layout();
        if let Some(player) = &mut self.simulation {
            player.configure(&self.settings.simulation, self.settings.view.stock_color);
        }
    }
}

fn settings_path() -> io::Result<PathBuf> {
    crate::settings_path().ok_or_else(|| io::Error::other("settings path unavailable"))
}

fn config_directory(settings: &Path) -> io::Result<&Path> {
    settings
        .parent()
        .ok_or_else(|| io::Error::other("settings directory unavailable"))
}

macro_rules! desktop_catalog {
    ($($name:ident [$read:literal] $description:literal {$($key:literal: $schema:expr),* $(,)?} |$c:ident, $a:ident| $body:block)*) => {
        fn catalog() -> Vec<Command<App>> {
            vec![$(Command {name:stringify!($name),description:$description,read_only:$read,
                properties:json!({$($key:$schema),*}),run:|$c,$a| $body}),*]
        }
    }
}
fn desktop_state(c: &App) -> Value {
    json!({"status":c.status,"ribbon":format!("{:?}",c.active_tab),
        "view":{"yaw":c.view.yaw,"pitch":c.view.pitch,"zoom":c.view.zoom,"pan":c.view.pan},
        "dirty_editor":c.setup_draft_dirty() || c.inspector_dirty() || c.pattern_draft.is_some(),
        "simulation":c.simulation.as_ref().map(|p|json!({"position_seconds":p.position,"playing":p.playing,"speed":p.speed,"options":p.options})),
        "panes": ([Pane::Project,Pane::Library,Pane::Viewport,Pane::Inspector,Pane::Output].iter()
            .map(|pane|json!({"name":pane.name(),"visible":c.pane_handle(*pane).is_some()})).collect::<Vec<_>>())})
}
fn pane(value: &str) -> Result<Pane, String> {
    match value {
        "Project" => Ok(Pane::Project),
        "Library" => Ok(Pane::Library),
        "Viewport" => Ok(Pane::Viewport),
        "Inspector" => Ok(Pane::Inspector),
        "Output" => Ok(Pane::Output),
        _ => Err("unknown pane".into()),
    }
}

desktop_catalog! {
    get_desktop_state [true] "Inspect the visible desktop, editor state, camera, panes and simulation playback." {} |c,_a| {Ok(desktop_state(c))}
    desktop_set_view [false] "Set camera yaw/pitch in degrees, zoom (-4..6) and pan [x,y,z]; omitted fields are retained." {"yaw":optional(number()),"pitch":optional(number()),"zoom":optional(number()),"pan":optional(vector(3))} |c,a| {
        if let Some(value)=a.optional("yaw")? {c.view.yaw=value;}
        if let Some(value)=a.optional("pitch")? {c.view.pitch=value;}
        if let Some(value)=a.optional::<f32>("zoom")? {c.view.zoom=value.clamp(-4.,6.);}
        if let Some(value)=a.optional("pan")? {c.view.pan=value;}
        Ok(desktop_state(c))
    }
    desktop_set_pane [false] "Show or hide a desktop pane." {"pane":choice(&["Project","Library","Viewport","Inspector","Output"]),"visible":boolean()} |c,a| {
        let _=c.update(Message::SetPaneVisible(pane(&a.get::<String>("pane")?)?,a.get("visible")?)); Ok(desktop_state(c))
    }
    desktop_select_ribbon [false] "Select a desktop ribbon tab." {"tab":choice(&["Home","Machining","Tools","Machinery","View"])} |c,a| {
        let tab=match a.get::<String>("tab")?.as_str() {"Home"=>RibbonTab::Home,"Machining"=>RibbonTab::Machining,
            "Tools"=>RibbonTab::Tools,"Machinery"=>RibbonTab::Machinery,_=>RibbonTab::View};
        let _=c.update(Message::SelectRibbonTab(tab)); Ok(desktop_state(c))
    }
    desktop_set_visibility [false] "Show or hide a CAD asset/instance, setup, or operation by stable ID." {"target":choice(&["asset","instance","setup","operation"]),"id":id(),"visible":boolean()} |c,a| {
        let id:u64=a.get("id")?; let visible=a.get("visible")?;
        let message=match a.get::<String>("target")?.as_str() {
            "asset" if c.controller.geometry_state().asset(id).is_some()=>Message::SetAssetVisible(id,visible),
            "instance" if c.controller.geometry_state().instance(id).is_some()=>Message::SetInstanceVisible(id,visible),
            "setup" if c.controller.setups().iter().any(|s|s.id==id)=>Message::SetSetupVisible(id,visible),
            "operation" if u32::try_from(id).ok().is_some_and(|id|c.controller.operation(id).is_some())=>Message::SetOperationVisible(id as u32,visible),
            _=>return Err("target not found".into()),
        };
        let _=c.update(message); Ok(desktop_state(c))
    }
    desktop_isolate_instance [false] "Isolate a CAD instance in the viewport." {"id":id()} |c,a| {
        let id=a.get("id")?; if c.controller.geometry_state().instance(id).is_none(){return Err("instance not found".into());}
        let _=c.update(Message::CadIsolateInstance(id)); Ok(desktop_state(c))
    }
    desktop_open_editor [false] "Open a setup, operation, pattern, or shop tool editor." {"target":choice(&["setup","operation","pattern","library_tool"]),"id":id()} |c,a| {
        let id:u64=a.get("id")?;
        let message=match a.get::<String>("target")?.as_str() {
            "setup" if c.controller.setups().iter().any(|s|s.id==id)=>Message::EditSetup(id),
            "operation" if u32::try_from(id).ok().is_some_and(|id|c.controller.operation(id).is_some())=>Message::Select(Selection::Operation(id as u32)),
            "pattern" if c.controller.inspection_setup().patterns.iter().any(|p|p.id==id)=>Message::EditPattern(id),
            "library_tool" if (id as usize)<c.library.tools.len()=>Message::SelectLibraryTool(id as usize),
            _=>return Err("editor target not found".into()),
        };
        let _=c.update(message); Ok(desktop_state(c))
    }
    desktop_command [false] "Invoke a desktop presentation or editor command. Most project edits use the shared application commands instead." {"command":choice(&[
        "reset_view","toggle_stock","toggle_transparent_stock","toggle_origin","toggle_envelope","toggle_gizmo","toggle_tooltips",
        "toggle_snapping","toggle_feature_highlights","show_all_models","show_preferences","close_preferences","show_license","close_license",
        "clear_output","apply_editor","cancel_editor"
    ])} |c,a| {
        let message=match a.get::<String>("command")?.as_str() {
            "reset_view"=>Message::ResetView,"toggle_stock"=>Message::ToggleStock,
            "toggle_transparent_stock"=>Message::ToggleTransparentStock,"toggle_origin"=>Message::ToggleShowOrigin,
            "toggle_envelope"=>Message::ToggleEnvelope,"toggle_gizmo"=>Message::ToggleGizmo,"toggle_tooltips"=>Message::ToggleTooltips,
            "toggle_snapping"=>Message::ToggleSnapping,"toggle_feature_highlights"=>Message::ToggleFeatureHighlights,
            "show_all_models"=>Message::CadShowAll,"show_preferences"=>Message::ShowPrefs,"close_preferences"=>Message::Preferences(preferences::Edit::Cancel),
            "show_license"=>Message::ShowLicense,"close_license"=>Message::CloseLicense,"clear_output"=>Message::OutputClear,
            "apply_editor"=>if c.pattern_draft.is_some(){Message::PatternCommit}else if c.setup_draft.is_some(){Message::SetupCommit}else{Message::Apply},
            "cancel_editor"=>{
                c.end_setup_draft(); c.pattern_draft=None; c.operation_draft=None;
                c.operation_geometry=None; c.operation_binding_preview=None; c.stock_to_leave_draft=None;
                c.cad_pending_kind=None; c.cad_editing_op=None; c.profile_entry_editor=None;
                c.machine_name_edit=None; c.machine_post_edit=None; c.machine_estlcam_edit=None;
                c.controller.cancel_operation(); c.reload_tool_edit(); c.height_draft_id=None;
                if let Selection::Operation(id)=c.controller.selection(){c.begin_operation_draft(id);}else{c.refresh_fields();}
                c.status="Editor changes cancelled by MCP.".into(); return Ok(desktop_state(c));
            },
            _=>return Err("unknown desktop command".into()),
        };
        let window_command = matches!(message, Message::ShowPrefs | Message::Preferences(_));
        let task = c.update(message);
        if window_command {
            c.mcp_task = task;
        }
        Ok(desktop_state(c))
    }
    desktop_simulation [false] "Control stock-removal playback. Open after a completed run; seek uses seconds and speed is 0.25..10." {
        "action":choice(&["open","close","play","pause","start","end","previous_point","next_point","previous_operation","next_operation","seek","speed"]),
        "value":optional(number()),"operation":optional(id())
    } |c,a| {
        let action:String=a.get("action")?;
        if action=="open" {
            c.controller.poll_run();
            let mut player=SimulationPlayer::new_for_operation(&c.controller,a.optional("operation")?)?;
            player.sync_force_material(&c.library);
            player.configure(&c.settings.simulation,c.settings.view.stock_color); c.simulation=Some(player);
        } else if action=="close" {c.simulation=None; c.pending_simulation=None;}
        else {
            let player=c.simulation.as_mut().ok_or("open simulation first")?;
            match action.as_str() {
                "play"=>{if !player.playing{player.toggle();}},"pause"=>{if player.playing{player.toggle();}},
                "start"=>player.seek_end(false),"end"=>player.seek_end(true),
                "previous_point"=>player.seek_point(false),"next_point"=>player.seek_point(true),
                "previous_operation"=>player.seek_operation(false),"next_operation"=>player.seek_operation(true),
                "seek"=>player.seek(a.get("value")?),"speed"=>player.speed=a.get::<f64>("value")?.clamp(0.25,10.),
                _=>return Err("unknown simulation action".into()),
            }
        }
        Ok(desktop_state(c))
    }
    get_desktop_output [true] "Read the desktop output/diagnostics console." {} |c,_a| {serialized(&c.output_snapshot)}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn desktop_mcp_edits_the_visible_controller_and_preserves_undo() {
        let (mut app, _) = App::new();
        app.controller.add_setup();
        let original = app.controller.inspection_setup().name.clone();
        app.mcp_call("update_setup", json!({"patch":{"name":"MCP setup"}}))
            .unwrap();
        assert_eq!(app.controller.inspection_setup().name, "MCP setup");
        app.mcp_call("undo", json!({})).unwrap();
        assert_eq!(app.controller.inspection_setup().name, original);
        app.mcp_call(
            "desktop_set_view",
            json!({"yaw":45.,"pitch":30.,"zoom":2.,"pan":[1.,2.,3.]}),
        )
        .unwrap();
        assert_eq!(app.view.yaw, 45.);
        assert_eq!(app.view.pan, [1., 2., 3.]);
    }

    #[test]
    fn desktop_mcp_keeps_drafts_until_explicitly_cancelled() {
        let (mut app, _) = App::new();
        app.controller.add_setup();
        app.begin_setup_draft();
        app.setup_draft.as_mut().unwrap().name = "Uncommitted".into();
        let before = app.controller.document().clone();
        assert!(app.mcp_call("add_setup", json!({})).is_err());
        assert_eq!(app.controller.document(), &before);
        app.mcp_call("get_state", json!({})).unwrap();
        app.mcp_call("desktop_command", json!({"command":"cancel_editor"}))
            .unwrap();
        app.mcp_call("add_setup", json!({})).unwrap();
        assert_eq!(app.controller.setups().len(), 2);
        assert!(!app
            .controller
            .setups()
            .iter()
            .any(|s| s.name == "Uncommitted"));
    }

    #[test]
    fn remote_operation_edits_rebase_the_visible_height_and_stock_editors() {
        let (mut app, _) = App::new();
        app.mcp_call("open_dxf", json!({"text":SAMPLE_DXF,"name":"MCP sample"}))
            .unwrap();
        let id = app.controller.inspection_setup().operations[0].id();
        app.begin_operation_draft(id);
        app.mcp_call(
            "update_operation",
            json!({"id":id,"patch":{"Profile":{"feed":225.}},
            "heights":{"clearance":15.,"retract":5.,"top_of_stock":0.}}),
        )
        .unwrap();
        assert!(!app.inspector_dirty());
        app.mcp_call(
            "update_operation",
            json!({"id":id,"patch":{"Profile":{"feed":250.}}}),
        )
        .unwrap();
        assert!(!app.inspector_dirty());
        app.height_offsets
            .insert(cam_model::HeightPlaneKind::Clearance, "17".into());
        app.mcp_call("desktop_command", json!({"command":"cancel_editor"}))
            .unwrap();
        assert!(!app.inspector_dirty());
    }

    #[test]
    fn desktop_queue_returns_the_result_of_the_same_session_edit() {
        let (mut app, _) = App::new();
        let (send, receive) = mpsc::sync_channel(1);
        let (reply, result) = mpsc::sync_channel(1);
        app.mcp = Some(DesktopMcp::Stdio(receive));
        send.send(Request {
            name: "add_setup".into(),
            arguments: json!({}),
            reply,
        })
        .unwrap();
        let _ = app.poll_mcp();
        assert!(result.recv().unwrap().is_ok());
        assert_eq!(app.controller.setups().len(), 1);
    }

    #[test]
    fn local_mcp_edits_the_open_project_and_restores_the_saved_preference() {
        use std::io::{BufRead, Read, Write};
        use std::time::{Duration, Instant};

        let scratch = crate::config::tests::Scratch::new("mcp-preference");
        let settings_path = scratch.file("settings.json");
        let (mut app, _) = App::new();
        app.controller.add_setup();
        let original = app.controller.inspection_setup().name.clone();
        assert!(app.mcp.is_none());
        app.settings.mcp_enabled = false;
        let settings = crate::Settings {
            mcp_enabled: true,
            ..app.settings.clone()
        };
        app.save_preferences_to(settings, &settings_path).unwrap();
        let Some(DesktopMcp::Local(server)) = &app.mcp else {
            panic!("could not enable MCP: {:?}", app.mcp_feedback);
        };
        let address = (Ipv4Addr::LOCALHOST, server.port());
        let token = server.token.clone();
        let client_config = server.configuration().unwrap();
        let endpoint = Endpoint::discover(&scratch.0).unwrap();
        assert_eq!(endpoint.port, address.1);
        assert!(endpoint.token == token);
        let timeout = Duration::from_secs(5);

        let mut rejected = TcpStream::connect(address).unwrap();
        rejected.set_read_timeout(Some(timeout)).unwrap();
        writeln!(rejected, "{}", "x".repeat(token.len())).unwrap();
        let _ = app.update(Message::PollMcp);
        assert_eq!(rejected.read(&mut [0]).unwrap(), 0);

        let mut client = TcpStream::connect(address).unwrap();
        client.set_read_timeout(Some(timeout)).unwrap();
        writeln!(client, "{token}").unwrap();
        for message in [
            json!({"jsonrpc":"2.0","id":1,"method":"initialize","params":{
                "protocolVersion":"2025-11-25","capabilities":{},"clientInfo":{}
            }}),
            json!({"jsonrpc":"2.0","method":"notifications/initialized"}),
            json!({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{
                "name":"update_setup","arguments":{"patch":{"name":"Attached client"}}
            }}),
        ] {
            writeln!(client, "{message}").unwrap();
        }
        let deadline = Instant::now() + timeout;
        while app.controller.inspection_setup().name != "Attached client" {
            assert!(
                Instant::now() < deadline,
                "the MCP edit never reached the desktop"
            );
            let _ = app.update(Message::PollMcp);
            thread::sleep(Duration::from_millis(1));
        }

        let mut reader = BufReader::new(&client);
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();
        assert_eq!(line, "OK\n");
        line.clear();
        reader.read_line(&mut line).unwrap();
        assert_eq!(serde_json::from_str::<Value>(&line).unwrap()["id"], 1);
        line.clear();
        reader.read_line(&mut line).unwrap();
        assert_eq!(
            serde_json::from_str::<Value>(&line).unwrap()["result"]["isError"],
            false
        );

        app.mcp_call("undo", json!({})).unwrap();
        assert_eq!(app.controller.inspection_setup().name, original);
        drop(app);
        assert_eq!(reader.read(&mut [0]).unwrap(), 0);
        assert!(TcpStream::connect(address).is_err());

        let (mut app, _) = App::new();
        (app.settings, _) = crate::load_settings_from(&settings_path);
        assert!(app.settings.mcp_enabled);
        app.restore_local_mcp_from(&settings_path).unwrap();
        let Some(DesktopMcp::Local(server)) = &app.mcp else {
            panic!("MCP did not start from the saved setting");
        };
        assert_eq!(server.configuration().unwrap(), client_config);
        assert_ne!(server.token, token);
        let endpoint = Endpoint::discover(&scratch.0).unwrap();
        let address = (Ipv4Addr::LOCALHOST, endpoint.port);
        let mut client = TcpStream::connect(address).unwrap();
        client.set_read_timeout(Some(timeout)).unwrap();
        writeln!(client, "{}", endpoint.token).unwrap();
        let _ = app.update(Message::PollMcp);
        let mut reply = [0; 3];
        client.read_exact(&mut reply).unwrap();
        assert_eq!(&reply, b"OK\n");

        let settings = crate::Settings {
            mcp_enabled: false,
            ..app.settings.clone()
        };
        app.save_preferences_to(settings, &settings_path).unwrap();
        assert!(app.mcp.is_none());
        assert_eq!(client.read(&mut [0]).unwrap(), 0);
        assert!(TcpStream::connect(address).is_err());
        assert!(Endpoint::discover(&scratch.0).is_err());
        (app.settings, _) = crate::load_settings_from(&settings_path);
        assert!(!app.settings.mcp_enabled);
        app.restore_local_mcp_from(&settings_path).unwrap();
        assert!(app.mcp.is_none());
    }

    #[test]
    fn desktop_and_shared_tool_names_are_unique() {
        let mut names = std::collections::BTreeSet::new();
        for tool in api::tools()
            .into_iter()
            .chain(catalog().iter().map(Command::tool))
        {
            assert!(names.insert(tool["name"].as_str().unwrap().to_owned()));
        }
    }
}
