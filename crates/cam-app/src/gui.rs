//! The iced desktop shell — a thin view over [`crate::AppController`].
//!
//! A professional-CAM layout built on iced's [`pane_grid`]: a tabbed **ribbon**
//! (Home / Machining / Tools / Machinery / View) across the top, then four
//! docked, resizable panes — a **Project** tree
//! (select a node), the **Viewport** (backplot + simulated stock), an
//! **Inspector** (edit the selected node), and an **Output** console
//! (diagnostics + status). Any pane can be shown/hidden from the **Windows**
//! ribbon tab (hiding one keeps the others' sizes). All behaviour is delegated to
//! the controller; this module only translates messages and draws. Only compiled
//! with the `gui` feature.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::path::{Path, PathBuf};
use std::sync::Arc;

#[path = "gui_preferences.rs"]
mod preferences;

#[path = "gui_mcp.rs"]
mod mcp;

use iced::widget::pane_grid::{self, PaneGrid};
use iced::widget::{
    button, canvas, checkbox, column, container, mouse_area, pick_list, row, scrollable, shader,
    slider, text, text_input, tooltip, Space,
};
use iced::{Alignment, Background, Border, Color, Element, Length, Padding};

use cam_model::{
    Axis, CarveClearing, ClearParams, CutDir, EntityKind, EntityRef, Hand, InstanceId, Lead,
    Operation, Plunge, Side, ToolKind,
};
use cam_post::PostKind;

use crate::project::OcamFile;
// `ToolKindPick`/`families_for` are plain data (an operation's tool families), so they
// live with the library rather than in this shell -- that is what lets the headless
// tests assert the starter library can start every operation.
use crate::tool_library::{families_for, ToolKindPick, ToolLibrary};

#[path = "gui_inspector_style.rs"]
mod inspector_style;
use inspector_style::*;
#[path = "gui_height_editor.rs"]
mod height_editor;
#[path = "gui_height_planes.rs"]
mod height_planes;
#[path = "gui_height_reference.rs"]
mod height_reference;
#[path = "gui_icon_picker.rs"]
mod icon_picker;
#[path = "gui_inspector.rs"]
mod inspector_view;
#[path = "gui_jobs.rs"]
mod jobs_view;
#[path = "gui_output.rs"]
mod output_view;
#[path = "gui_processing_time.rs"]
mod processing_time_view;
use jobs_view::ToolpathAction;
#[path = "gui_backplot_preview.rs"]
mod backplot_preview_view;
#[path = "gui_simulation.rs"]
mod simulation_view;
use simulation_view::{SimulationOption, SimulationPlayer, SimulationTab};
#[path = "gui_operation_linking.rs"]
mod operation_linking;
#[path = "gui_operation_metadata.rs"]
mod operation_metadata;
#[path = "gui_operation_sections.rs"]
mod operation_sections;
#[path = "gui_operation_tabs.rs"]
mod operation_tabs;
#[path = "gui_operation_tool_selector.rs"]
mod operation_tool_selector;
#[path = "gui_pattern_editor.rs"]
mod pattern_editor_view;
#[path = "gui_rotary.rs"]
mod rotary_view;
#[path = "gui_setup_editor.rs"]
mod setup_editor_view;
#[path = "gui_thread_parameters.rs"]
mod thread_parameters;
use operation_tabs::OperationTab;
#[path = "gui_move.rs"]
mod move_view;
#[path = "gui_project_tree.rs"]
mod project_tree_view;
use project_tree_view::{TreeDrag, TreeMessage, TreeNode};
#[path = "gui_workspace.rs"]
mod workspace_view;
use workspace_view::Workspace;
#[path = "gui_geometry_selection.rs"]
mod geometry_selection;
#[path = "gui_operation_geometry.rs"]
mod operation_geometry;
use geometry_selection::{
    cad_geometry_labels, drawing_geometry_labels, operation_geometry_labels, ProfileSelectionMode,
};
use operation_geometry::GeometryMessage;
#[path = "gui_creation.rs"]
mod creation;
#[path = "gui_cutting_data.rs"]
mod cutting_data;
#[path = "gui_cutting_forces.rs"]
mod cutting_forces;
#[cfg(test)]
#[path = "gui_performance.rs"]
mod performance;
#[path = "gui_tool_drawing.rs"]
mod tool_drawing;
use tool_drawing::ToolCanvas;
#[path = "gui_tool_presets.rs"]
mod tool_presets;
use tool_presets::{MaterialChoice, PresetChoice, PresetField};
#[path = "gui_grid.rs"]
mod grid_view;
#[path = "gui_help.rs"]
mod illustrated_help;
#[path = "gui_operation_hints.rs"]
mod operation_hints;
#[path = "gui_operation_positions.rs"]
mod operation_positions;
#[path = "gui_operation_preview.rs"]
mod operation_preview;
#[path = "gui_parameter_defaults.rs"]
mod parameter_defaults;
#[path = "gui_parameter_expressions.rs"]
mod parameter_expressions;
#[path = "gui_parameter_options.rs"]
mod parameter_options;
#[path = "gui_picking.rs"]
mod picking_view;
#[path = "gui_pocket_parameters.rs"]
mod pocket_parameters;
#[path = "gui_profile_entries.rs"]
mod profile_entries_view;
#[path = "gui_profile_parameters.rs"]
mod profile_parameters;
#[path = "gui_profile_sides.rs"]
mod profile_sides;
#[path = "gui_profile_tabs.rs"]
mod profile_tabs;
#[path = "gui_wcs.rs"]
mod wcs_view;
use picking_view::{SetupFeature, SetupHover, SetupPickGeometry};
use wcs_view::{SetupOrientation, SetupOriginMode};
#[path = "gui_notices.rs"]
mod notices;
use notices::Notice;
#[path = "gui_unsaved.rs"]
mod unsaved;
use unsaved::DiscardAction;
#[path = "gui_status.rs"]
mod status_view;
use operation_metadata::operation_fields;
use status_view::{StatusMessages, StatusTarget};

/// Ribbon palette and metrics, adopted from **OpenCADStudio**'s ribbon
/// (`HakanSeven12/OpenCADStudio`, GPL-3.0) so users moving CAD → CAM meet a
/// familiar surface. Values are its own `TOPBAR_BG`/`RIBBON_BG`/… constants; the
/// widget structure here is ours (a simpler five-tab band, no density modes).
mod palette {
    use iced::Color;

    const fn rgb(r: f32, g: f32, b: f32) -> Color {
        Color { r, g, b, a: 1.0 }
    }

    /// Tab-strip background.
    pub const TOPBAR_BG: Color = rgb(0.17, 0.17, 0.17);
    /// Ribbon-band background.
    pub const RIBBON_BG: Color = rgb(0.22, 0.22, 0.22);
    /// The band's dark framing border.
    pub const BORDER_DARK: Color = rgb(0.12, 0.12, 0.12);
    /// Active-tab accent.
    pub const ACCENT_BLUE: Color = rgb(0.20, 0.55, 0.90);
    /// Normal control text.
    pub const LABEL_COLOR: Color = rgb(0.82, 0.82, 0.82);
    /// Ribbon text and group captions on the dark ribbon background.
    pub const RIBBON_TEXT: Color = rgb(0.92, 0.92, 0.92);
    pub const RIBBON_CAPTION: Color = rgb(0.75, 0.75, 0.75);
    pub const RIBBON_DISABLED: Color = rgb(0.65, 0.65, 0.65);
    /// Muted labels and disabled controls elsewhere.
    pub const GROUP_LABEL: Color = rgb(0.50, 0.50, 0.50);
    /// Hover fill for tabs.
    pub const TOOL_HOVER: Color = rgb(0.32, 0.32, 0.32);
    /// Hover fill for command rows.
    pub const ROW_HOVER: Color = rgb(0.24, 0.24, 0.24);
    /// Pane content background.
    pub const PANE_BG: Color = rgb(0.15, 0.15, 0.15);
    /// Visible divider line framing each pane.
    pub const SEPARATOR: Color = rgb(0.34, 0.34, 0.34);
    /// Selected project-tree row highlight (a muted fill, not the loud button blue).
    pub const SELECT_BG: Color = rgb(0.20, 0.28, 0.38);
    /// Warning accent (amber) — e.g. the exact-duplicate operation marker. Paired
    /// with a ⚠ glyph so the meaning survives colour-vision deficiency.
    pub const WARN: Color = rgb(0.95, 0.62, 0.10);
    /// Error accent (red) for an invalid inspector field. A thickened border carries the
    /// same signal by weight, so it does not rely on hue alone.
    pub const ERROR: Color = rgb(0.92, 0.30, 0.28);
}

/// The smallest tip **radius** a ground pointed tool can physically have, in mm.
///
/// Neither a chamfer mill nor a V-bit has a true `r = 0` point — it cannot be ground
/// and would not survive contact. 50 µm is the practical floor for a ground tip; below
/// it the number is fiction and the geometry degenerates (a V-bit's tip arc collapses,
/// and a chamfer mill's non-cutting flat — the very thing that distinguishes it from a
/// V-bit — disappears).
///
/// Applied as a **radius** to both, so the two are held to the same physical size: a
/// V-bit's tip radius directly, a chamfer mill's tip *diameter* at twice this.
pub const MIN_TIP_RADIUS_MM: f64 = 0.05;

/// Ribbon command icons. Each variant is a small embedded SVG (see
/// `assets/icons/CREDITS.md` — most reused from OpenCADStudio, GPL-3.0; the
/// CAM-specific ones drawn to match). The SVGs carry their own colours, so we
/// render them untinted.
#[derive(Clone, Copy, Debug)]
enum Icon {
    New,
    Open,
    Save,
    Import,
    Export,
    Undo,
    Redo,
    Run,
    Toolpath,
    ToolpathTime,
    Pattern,
    CircularPattern,
    Profile,
    Pocket,
    Drill,
    Thread,
    Bore,
    Chamfer,
    Engrave,
    Carve,
    Adaptive,
    Parallel,
    Spiral,
    Face,
    NewTool,
    Materials,
    Renumber,
    ImportLibrary,
    ExportLibrary,
    Duplicate,
    Edit,
    Suppress,
    DeleteOperation,
    Delete,
    #[allow(dead_code)] // Retained view command for the M1a workspace shell.
    ShowStock,
    ResetView,
    #[allow(dead_code)] // Retained view command for the M1a workspace shell.
    ShowCube,
    SetOrigin,
    Info,
    Machine,
    License,
}

impl Icon {
    /// Every icon, so tests can sweep the whole bundled set. Kept beside the enum:
    /// a variant added without a line here fails `every_icon_is_listed_in_all`.
    #[cfg(test)]
    const ALL: [Icon; 41] = [
        Icon::New,
        Icon::Open,
        Icon::Save,
        Icon::Import,
        Icon::Export,
        Icon::Undo,
        Icon::Redo,
        Icon::Run,
        Icon::Toolpath,
        Icon::ToolpathTime,
        Icon::Pattern,
        Icon::CircularPattern,
        Icon::Profile,
        Icon::Pocket,
        Icon::Drill,
        Icon::Thread,
        Icon::Bore,
        Icon::Chamfer,
        Icon::Engrave,
        Icon::Carve,
        Icon::Adaptive,
        Icon::Parallel,
        Icon::Spiral,
        Icon::Face,
        Icon::NewTool,
        Icon::Materials,
        Icon::Renumber,
        Icon::ImportLibrary,
        Icon::ExportLibrary,
        Icon::Duplicate,
        Icon::Edit,
        Icon::Suppress,
        Icon::DeleteOperation,
        Icon::Delete,
        Icon::ShowStock,
        Icon::ResetView,
        Icon::ShowCube,
        Icon::SetOrigin,
        Icon::Info,
        Icon::Machine,
        Icon::License,
    ];
}

impl Icon {
    fn bytes(self) -> &'static [u8] {
        match self {
            Icon::New => include_bytes!("../assets/icons/new.svg"),
            Icon::Open => include_bytes!("../assets/icons/open.svg"),
            Icon::Save => include_bytes!("../assets/icons/save.svg"),
            Icon::Import => include_bytes!("../assets/icons/cui_import.svg"),
            Icon::Export => include_bytes!("../assets/icons/cui_export.svg"),
            Icon::Undo => include_bytes!("../assets/icons/undo.svg"),
            Icon::Redo => include_bytes!("../assets/icons/redo.svg"),
            Icon::Run => include_bytes!("../assets/icons/run.svg"),
            Icon::Toolpath => include_bytes!("../assets/icons/toolpath.svg"),
            Icon::ToolpathTime => include_bytes!("../assets/icons/toolpath_time.svg"),
            Icon::Pattern => include_bytes!("../assets/icons/pattern.svg"),
            Icon::CircularPattern => include_bytes!("../assets/icons/pattern_circular.svg"),
            Icon::Profile => include_bytes!("../assets/icons/profile.svg"),
            Icon::Pocket => include_bytes!("../assets/icons/pocket.svg"),
            Icon::Drill => include_bytes!("../assets/icons/drill.svg"),
            Icon::Bore => include_bytes!("../assets/icons/bore.svg"),
            Icon::Thread => include_bytes!("../assets/icons/thread.svg"),
            Icon::Chamfer => include_bytes!("../assets/icons/chamfer.svg"),
            Icon::Engrave => include_bytes!("../assets/icons/engrave.svg"),
            Icon::Carve => include_bytes!("../assets/icons/carve.svg"),
            Icon::Spiral => include_bytes!("../assets/icons/spiral.svg"),
            Icon::Parallel => include_bytes!("../assets/icons/parallel.svg"),
            Icon::Adaptive => include_bytes!("../assets/icons/adaptive.svg"),
            Icon::Face => include_bytes!("../assets/icons/face.svg"),
            Icon::Materials => include_bytes!("../assets/icons/materials.svg"),
            Icon::NewTool => include_bytes!("../assets/icons/endmill.svg"),
            Icon::Renumber => include_bytes!("../assets/icons/renumber.svg"),
            Icon::ImportLibrary => include_bytes!("../assets/icons/import_library.svg"),
            Icon::ExportLibrary => include_bytes!("../assets/icons/export_library.svg"),
            Icon::Duplicate => include_bytes!("../assets/icons/copy.svg"),
            Icon::Edit => include_bytes!("../assets/icons/edit.svg"),
            Icon::Suppress => include_bytes!("../assets/icons/suppress.svg"),
            Icon::DeleteOperation => include_bytes!("../assets/icons/delete_operation.svg"),
            Icon::Delete => include_bytes!("../assets/icons/erase.svg"),
            Icon::ShowStock => include_bytes!("../assets/icons/box3d.svg"),
            Icon::ResetView => include_bytes!("../assets/icons/zoom_ext.svg"),
            Icon::ShowCube => include_bytes!("../assets/icons/viewcube.svg"),
            Icon::SetOrigin => include_bytes!("../assets/icons/origin.svg"),
            Icon::Info => include_bytes!("../assets/icons/info.svg"),
            Icon::Machine => include_bytes!("../assets/icons/machine.svg"),
            Icon::License => include_bytes!("../assets/icons/license.svg"),
        }
    }

    /// A one-line description of the command this icon stands for, shown as a hover
    /// tooltip on its ribbon button (the label under the icon is terser, and vanishes
    /// entirely at Compact density — the tooltip is where the icon explains itself).
    fn help(self) -> &'static str {
        match self {
            Icon::Materials => "Manage the shared materials used by stock and cutting presets.",
            Icon::New => "Start a new, empty project.",
            Icon::Open => "Open a saved .ocam project from disk.",
            Icon::Save => "Save the current project to disk.",
            Icon::Import => "Import a STEP/IGES model or DXF/DWG drawing.",
            Icon::Export => "Export the generated toolpaths as G-code (NC) for your machine.",
            Icon::Undo => "Undo the last change.",
            Icon::Redo => "Redo the change just undone.",
            Icon::Run => "Generate toolpaths for every operation and simulate the result.",
            Icon::Toolpath => "Recompute the selected toolpaths.",
            Icon::ToolpathTime => "Calculate the selected toolpath processing time.",
            Icon::Pattern => "Repeat source operations in a rectangular or circular pattern.",
            Icon::CircularPattern => "Repeat source operations around a circular pattern.",
            Icon::Profile => {
                "Add a Profile operation — cut along a chain, inside/outside/on the line."
            }
            Icon::Pocket => "Add a Pocket operation — clear the area inside a closed boundary.",
            Icon::Drill => "Add a Drill operation — peck/drill the selected hole circles.",
            Icon::Bore => "Add a Bore operation to helically mill a hole or boss.",
            Icon::Thread => "Add a Thread-milling operation on a bore or boss.",
            Icon::Chamfer => "Add a Chamfer operation — break an edge with a V/chamfer tool.",
            Icon::Engrave => {
                "Add an Engrave operation — plough a V-groove along a path with a V-bit. \
                              Groove width follows from the depth and the bit's angle; a chamfer mill \
                              will not do (its tip does not cut)."
            }
            Icon::Carve => {
                "Add a Carve operation — carve out the AREA a closed boundary encloses \
                            with a V-bit. Unlike engraving, the tool never touches the boundary: its \
                            flanks land on it, and the depth follows from the shape's own width."
            }
            Icon::Spiral => "Add a 3D Spiral finishing operation for selected CAD bodies or faces.",
            Icon::Parallel => "Add a 3D Parallel or sampled Scallop finishing operation for selected CAD bodies or faces.",
            Icon::Adaptive => "Add a 3D Adaptive Clearing operation for selected CAD bodies or faces.",
            Icon::Face => "Add a Face operation — skim the top of the stock flat.",
            Icon::NewTool => "Add a new tool to the library.",
            Icon::Renumber => {
                "Renumber the whole library sequentially in the current tab order \
                 (asks for confirmation — it rewrites every tool number)."
            }
            Icon::ImportLibrary => {
                "Import a .ocam library or select tools from a Fusion .tools database."
            }
            Icon::ExportLibrary => "Export the working tool library to a .ocam file to share.",
            Icon::Duplicate => "Duplicate the selected operation.",
            Icon::Edit => "Edit the selected setup or operation.",
            Icon::Suppress => "Exclude the operation from machining.",
            Icon::DeleteOperation => "Delete the selected operation or setup.",
            Icon::Delete => "Delete the selected item.",
            Icon::ShowStock => "Show or hide the simulated stock surface in the viewport.",
            Icon::ResetView => "Reset the camera to frame the whole part.",
            Icon::ShowCube => "Show or hide the orientation cube.",
            Icon::SetOrigin => "Show or hide the workpiece-origin datum marker.",
            Icon::Info => "Show or hide hover tooltips on inspector fields and ribbon icons.",
            Icon::Machine => {
                "Set up the machine — working travel and the post-processor (G-code dialect, \
                 e.g. grbl / FluidNC / Fanuc) the exported NC is written for."
            }
            Icon::License => {
                "Licence and credits — the GNU GPL v3 this program is released under, \
                 and the third-party work it builds on."
            }
        }
    }

    /// An iced SVG handle for this icon. The handle id is derived from the bytes,
    /// so iced caches the rasterisation across frames.
    fn handle(self) -> iced::widget::svg::Handle {
        iced::widget::svg::Handle::from_memory(self.bytes())
    }
}

/// An SVG icon widget sized to a square `size` px.
fn icon_svg(icon: Icon, size: f32) -> Element<'static, Message> {
    iced::widget::svg(icon.handle())
        .width(Length::Fixed(size))
        .height(Length::Fixed(size))
        .into()
}
use cam_cldata::{Program, Step};
use cam_render::{
    CadScene, MeshVertex, OrbitCamera, PickCandidate, Scene, Vertex, ENVELOPE, ENVELOPE_OVER, PART,
};
use cam_toolpath::{JobCompletion, Severity};

use crate::{
    op_accepts_open_paths, op_selects_circles, op_takes_islands, AppController, CuttingData,
    LoopRef, OpKind, PickResult, Selection, SnapHit, SnapKind,
};

/// A small sample part (rectangle + circular hole) so the app is useful without
/// a file dialog on first run.
const SAMPLE_DXF: &str = "\
0\nSECTION\n2\nENTITIES\n\
0\nLINE\n10\n10.0\n20\n10.0\n11\n70.0\n21\n10.0\n\
0\nLINE\n10\n70.0\n20\n10.0\n11\n70.0\n21\n50.0\n\
0\nLINE\n10\n70.0\n20\n50.0\n11\n10.0\n21\n50.0\n\
0\nLINE\n10\n10.0\n20\n50.0\n11\n10.0\n21\n10.0\n\
0\nCIRCLE\n10\n40.0\n20\n30.0\n40\n5.0\n\
0\nENDSEC\n0\nEOF\n";

/// Launch the desktop application.
pub fn run() -> iced::Result {
    run_app(|| {
        let (mut app, task) = App::new();
        app.restore_local_mcp();
        (app, task)
    })
}

/// Open a saved setup/project before launching the desktop application.
/// Load errors are returned before creating a window so automation can detect them.
pub fn run_with_setup(path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let (mut app, task) = App::new_with_setup(path)
        .map_err(|error| format!("Could not load setup {}: {error}", path.display()))?;
    app.restore_local_mcp();
    let initial = std::cell::RefCell::new(Some((app, task)));
    run_app(move || initial.borrow_mut().take().expect("application boots once"))?;
    Ok(())
}

/// Launch a desktop session controlled by the MCP client through stdio.
pub fn run_with_mcp(path: Option<&Path>) -> Result<(), Box<dyn std::error::Error>> {
    let (mut app, task) = match path {
        Some(path) => App::new_with_setup(path)?,
        None => App::new(),
    };
    app.mcp = Some(mcp::start());
    let initial = std::cell::RefCell::new(Some((app, task)));
    run_app(move || initial.borrow_mut().take().expect("application boots once"))?;
    Ok(())
}

fn run_app(boot: impl iced::application::BootFn<App, Message> + 'static) -> iced::Result {
    iced::daemon(
        move || {
            let (mut app, task) = boot.boot();
            let (id, open) = iced::window::open(iced::window::Settings {
                exit_on_close_request: false,
                ..Default::default()
            });
            app.main_window = Some(id);
            (app, open.discard().chain(task))
        },
        App::update,
        App::window_view,
    )
    .title(App::title_for_window)
    .theme(|app: &App, id| {
        if app.is_preferences_window(id) {
            preferences::theme()
        } else {
            iced::Theme::Dark
        }
    })
    .subscription(App::subscription)
    .run()
}

fn window_events(
    window: iced::window::Id,
    handle: fn(iced::Event, iced::event::Status) -> Option<Message>,
) -> iced::Subscription<Message> {
    iced::event::listen_with(|event, status, id| Some((id, event, status)))
        .with((window, handle))
        .filter_map(|((window, handle), (id, event, status))| {
            if window == id {
                handle(event, status)
            } else {
                None
            }
        })
}

/// Prompt (native dialog) for an existing file matching `exts`, returning its path.
async fn pick_open(name: &'static str, exts: &'static [&'static str]) -> Option<PathBuf> {
    rfd::AsyncFileDialog::new()
        .add_filter(name, exts)
        .pick_file()
        .await
        .map(|h| h.path().to_path_buf())
}

/// Prompt (native dialog) for a save location, seeded with `default_name`.
async fn pick_save(
    name: &'static str,
    default_name: &'static str,
    exts: &'static [&'static str],
) -> Option<PathBuf> {
    rfd::AsyncFileDialog::new()
        .add_filter(name, exts)
        .set_file_name(default_name)
        .save_file()
        .await
        .map(|h| h.path().to_path_buf())
}

/// A short human-readable summary of the exact-duplicate operation groups, for
/// the export confirm dialog. Each group is rendered as `#a = #b (= #c…)`.
fn describe_duplicates(groups: &[Vec<u32>]) -> String {
    let list = groups
        .iter()
        .map(|g| {
            g.iter()
                .map(|id| format!("#{id}"))
                .collect::<Vec<_>>()
                .join(" = ")
        })
        .collect::<Vec<_>>()
        .join(";  ");
    format!(
        "These operations are exact duplicates and would each post the same \
         toolpath — the machine would cut it more than once:\n\n    {list}\n\n\
         Export all of them anyway?"
    )
}

/// Native Yes/No warning dialog gating an export that contains exact-duplicate
/// operations. Returns `true` only if the user chose to proceed.
async fn confirm_export_duplicates(detail: String) -> bool {
    rfd::AsyncMessageDialog::new()
        .set_level(rfd::MessageLevel::Warning)
        .set_title("Duplicate operations")
        .set_description(detail)
        .set_buttons(rfd::MessageButtons::YesNo)
        .show()
        .await
        == rfd::MessageDialogResult::Yes
}

async fn report_export_verification_warnings(count: usize) {
    let detail = format!(
        "Verification detected collisions with stock or fixtures ({count} total). Review the setup or operation status indicators for details."
    );
    rfd::AsyncMessageDialog::new()
        .set_level(rfd::MessageLevel::Warning)
        .set_title("G-code export has verification warnings")
        .set_description(detail)
        .set_buttons(rfd::MessageButtons::Ok)
        .show()
        .await;
}

/// Native error dialog for an export that cannot happen.
///
/// A dialog rather than the status line, because the status line lives in the **Output**
/// pane and the operator can hide it — at which point a refusal would be entirely
/// silent. A blocked export is exactly the case that must not be.
async fn report_export_blocked(detail: String) {
    rfd::AsyncMessageDialog::new()
        .set_level(rfd::MessageLevel::Error)
        .set_title("Cannot export")
        .set_description(detail)
        .set_buttons(rfd::MessageButtons::Ok)
        .show()
        .await;
}

/// Native Yes/No warning gating the **bulk renumber** (it rewrites every tool's number).
async fn confirm_renumber(detail: String) -> bool {
    rfd::AsyncMessageDialog::new()
        .set_level(rfd::MessageLevel::Warning)
        .set_title("Renumber tools")
        .set_description(detail)
        .set_buttons(rfd::MessageButtons::YesNo)
        .show()
        .await
        == rfd::MessageDialogResult::Yes
}

async fn confirm_delete_library(name: String, last: bool) -> (String, bool) {
    let detail = if last {
        format!("Delete the library ‘{name}’ and all its tools? A fresh default library will be created.")
    } else {
        format!("Delete the library ‘{name}’ and all its tools? Tools already used in projects are retained.")
    };
    let confirmed = rfd::AsyncMessageDialog::new()
        .set_level(rfd::MessageLevel::Warning)
        .set_title("Delete tool library")
        .set_description(detail)
        .set_buttons(rfd::MessageButtons::YesNo)
        .show()
        .await
        == rfd::MessageDialogResult::Yes;
    (name, confirmed)
}

/// The docked panes. Any of them can be shown or hidden from the Windows ribbon
/// tab (except the Viewport, which is always present as the main view).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pane {
    Project,
    Library,
    Viewport,
    Inspector,
    Output,
}

impl Pane {
    fn name(self) -> &'static str {
        match self {
            Pane::Project => "Project",
            Pane::Library => "Tool Library",
            Pane::Viewport => "Viewport",
            Pane::Inspector => "Inspector",
            Pane::Output => "Output",
        }
    }

    /// The short label this pane uses in the ribbon's **Panes** group. Only the tool
    /// library differs: it compacts to "Tools" so the group stays narrow in a band
    /// that already carries five commands and the cube slider. The pane's own title
    /// bar keeps the full name from [`name`](Self::name).
    fn ribbon_label(self) -> &'static str {
        match self {
            Pane::Library => "Tools",
            other => other.name(),
        }
    }

    /// This pane's minimum size (px) along whichever axis it is split — enforced
    /// individually while resizing (see `App::clamp_resize`).
    /// The size this pane may not shrink past, from the user's preferences.
    ///
    /// Was a hard-coded match. The shipped defaults still fit real content — the
    /// Project pane's Duplicate/Delete row, the Inspector's field rows — but they were
    /// chosen on one monitor, and a value picked on one display is a bug on another
    /// (a 240 px Inspector plus a 200 px Project eats a third of a 1366-wide screen).
    fn min_size(self, prefs: &crate::PanePrefs) -> f32 {
        match self {
            Pane::Project => prefs.min_project_px,
            Pane::Library => prefs.min_library_px,
            Pane::Viewport => prefs.min_viewport_px,
            Pane::Inspector => prefs.min_inspector_px,
            Pane::Output => prefs.min_output_px,
        }
    }

    /// The edge this pane docks to when re-shown. The Viewport has no fixed edge
    /// — it simply takes the space where it is split back in.
    fn dock_edge(self) -> Option<pane_grid::Edge> {
        match self {
            // Library docks Left, like Project — it substitutes for it on the
            // Tools tab, and can also sit alongside it.
            Pane::Project | Pane::Library => Some(pane_grid::Edge::Left),
            Pane::Inspector => Some(pane_grid::Edge::Right),
            Pane::Output => Some(pane_grid::Edge::Bottom),
            Pane::Viewport => None,
        }
    }
}

/// Every pane, in Windows-menu order.
const ALL_PANES: [Pane; 5] = [
    Pane::Project,
    Pane::Library,
    Pane::Viewport,
    Pane::Inspector,
    Pane::Output,
];

/// The Library-pane right-click menu state. Starts as a "Set number…" item; clicking it
/// morphs the same popup into a number input (`input = Some(buffer)`).
#[derive(Clone, Debug)]
struct LibMenu {
    index: usize,
    input: Option<String>,
}

/// How the Tool Library pane lists its tools — the pane's two internal tabs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LibraryView {
    /// Ordered by tool number (T1, T2, …).
    Ordered,
    /// Grouped by tool family (all end mills, then drills, …), sorted by diameter.
    Grouped,
}

/// A tab in the top-bar ribbon. Each tab shows a band of grouped commands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RibbonTab {
    Home,
    Machining,
    Tools,
    /// The shop's machines, kept beside the installation's tool library.
    Machinery,
    View,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum SetupEditorTab {
    #[default]
    Setup,
    Stock,
    Postprocess,
}

impl SetupEditorTab {
    const ALL: [Self; 3] = [Self::Setup, Self::Stock, Self::Postprocess];
}

impl std::fmt::Display for SetupEditorTab {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Setup => "Setup",
            Self::Stock => "Stock",
            Self::Postprocess => "Postprocess",
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(clippy::enum_variant_names)]
enum StockEditorKind {
    FixedBox,
    RelativeBox,
    FixedCylinder,
    RelativeCylinder,
    FixedTube,
    RelativeTube,
    FromSolid,
    LegacyBox,
    PreviousSetup,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct StockSolidChoice {
    id: u64,
    name: String,
}
impl std::fmt::Display for StockSolidChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.name)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BoxAnchor {
    Min,
    Center,
    Max,
}

impl BoxAnchor {
    const ALL: [Self; 3] = [Self::Min, Self::Center, Self::Max];
    fn value(self) -> f64 {
        match self {
            Self::Min => 0.0,
            Self::Center => 0.5,
            Self::Max => 1.0,
        }
    }
    fn of(value: f64) -> Self {
        if value < 0.25 {
            Self::Min
        } else if value > 0.75 {
            Self::Max
        } else {
            Self::Center
        }
    }
}
impl std::fmt::Display for BoxAnchor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Min => "Min",
            Self::Center => "Center",
            Self::Max => "Max",
        })
    }
}

impl StockEditorKind {
    const ALL: [Self; 8] = [
        Self::FixedBox,
        Self::RelativeBox,
        Self::FixedCylinder,
        Self::RelativeCylinder,
        Self::FixedTube,
        Self::RelativeTube,
        Self::FromSolid,
        Self::PreviousSetup,
    ];
}

impl StockEditorKind {
    fn icon(self) -> &'static [u8] {
        match self {
            Self::FixedBox => include_bytes!("../assets/icons/stock/fixed_box.svg"),
            Self::RelativeBox | Self::LegacyBox => {
                include_bytes!("../assets/icons/stock/relative_box.svg")
            }
            Self::FixedCylinder => include_bytes!("../assets/icons/stock/fixed_cylinder.svg"),
            Self::RelativeCylinder => include_bytes!("../assets/icons/stock/relative_cylinder.svg"),
            Self::FixedTube => include_bytes!("../assets/icons/stock/fixed_tube.svg"),
            Self::RelativeTube => include_bytes!("../assets/icons/stock/relative_tube.svg"),
            Self::FromSolid | Self::PreviousSetup => {
                include_bytes!("../assets/icons/stock/solid.svg")
            }
        }
    }
}

impl std::fmt::Display for StockEditorKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::FixedBox => "Fixed size box",
            Self::RelativeBox => "Relative size box",
            Self::FixedCylinder => "Fixed size cylinder",
            Self::RelativeCylinder => "Relative size cylinder",
            Self::FixedTube => "Fixed size tube",
            Self::RelativeTube => "Relative size tube",
            Self::FromSolid => "From solid",
            Self::LegacyBox => "Legacy box (existing project)",
            Self::PreviousSetup => "From previous setup",
        })
    }
}

impl RibbonTab {
    /// The tabs shown in the strip, left to right.
    const ALL: [RibbonTab; 5] = [
        RibbonTab::Home,
        RibbonTab::Machining,
        RibbonTab::Tools,
        RibbonTab::Machinery,
        RibbonTab::View,
    ];

    fn label(self) -> &'static str {
        match self {
            RibbonTab::Home => "Home",
            RibbonTab::Machining => "Machining",
            RibbonTab::Tools => "Tools",
            RibbonTab::Machinery => "Machinery",
            RibbonTab::View => "View",
        }
    }
}

/// The kind of a [`Lead`], for the inspector picker (params come from fields).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum LeadKind {
    None,
    Linear,
    Arc,
}

impl LeadKind {
    const ALL: [LeadKind; 3] = [LeadKind::None, LeadKind::Linear, LeadKind::Arc];

    fn of(lead: Lead) -> Self {
        match lead {
            Lead::None => LeadKind::None,
            Lead::Linear { .. } => LeadKind::Linear,
            Lead::Arc { .. } => LeadKind::Arc,
            Lead::ArcSweep { .. } => LeadKind::Arc,
        }
    }

    /// A `Lead` of this kind, carrying over the previous size where sensible.
    fn to_lead(self, prev: Lead) -> Lead {
        let size = match prev {
            Lead::None => 0.6,
            Lead::Linear { length } => length,
            Lead::Arc { radius } | Lead::ArcSweep { radius, .. } => radius,
        };
        match self {
            LeadKind::None => Lead::None,
            LeadKind::Linear => Lead::Linear { length: size },
            LeadKind::Arc => {
                if let Lead::ArcSweep { angle_deg, .. } = prev {
                    Lead::ArcSweep {
                        radius: size,
                        angle_deg,
                    }
                } else {
                    Lead::Arc { radius: size }
                }
            }
        }
    }
}

impl std::fmt::Display for LeadKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            LeadKind::None => "None",
            LeadKind::Linear => "Linear",
            LeadKind::Arc => "Arc",
        })
    }
}

/// The kind of a [`Plunge`], for the inspector picker.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum PlungeKind {
    Straight,
    Ramp,
    Helix,
    ZigZag,
}

impl PlungeKind {
    const ALL: [PlungeKind; 4] = [
        PlungeKind::Straight,
        PlungeKind::Ramp,
        PlungeKind::Helix,
        PlungeKind::ZigZag,
    ];

    fn of(plunge: Plunge) -> Self {
        match plunge {
            Plunge::Straight => PlungeKind::Straight,
            Plunge::Ramp { .. } => PlungeKind::Ramp,
            Plunge::Helix { .. } => PlungeKind::Helix,
            Plunge::ZigZag { .. } => PlungeKind::ZigZag,
        }
    }

    /// A `Plunge` of this kind with sensible default parameters.
    fn to_plunge(self) -> Plunge {
        match self {
            PlungeKind::Straight => Plunge::Straight,
            PlungeKind::Ramp => Plunge::Ramp { angle_deg: 5.0 },
            PlungeKind::Helix => Plunge::Helix {
                radius: 2.0,
                pitch: 1.0,
            },
            PlungeKind::ZigZag => Plunge::ZigZag {
                length: 5.0,
                angle_deg: 5.0,
            },
        }
    }
}

impl std::fmt::Display for PlungeKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            PlungeKind::Straight => "Straight",
            PlungeKind::Ramp => "Ramp along path",
            PlungeKind::Helix => "Helix",
            PlungeKind::ZigZag => "Zig-zag",
        })
    }
}

/// Whether a thread is cut into a bore or onto a boss, for the inspector picker
/// (a friendlier face on `ThreadOp::internal`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum Bore {
    Internal,
    External,
}

impl Bore {
    const ALL: [Bore; 2] = [Bore::Internal, Bore::External];

    fn of(internal: bool) -> Self {
        if internal {
            Bore::Internal
        } else {
            Bore::External
        }
    }
}

impl std::fmt::Display for Bore {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Bore::Internal => "Internal",
            Bore::External => "External",
        })
    }
}

/// Climb vs. conventional milling, for the inspector picker (a friendlier face on
/// `ThreadOp::climb`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum CutStyle {
    Climb,
    Conventional,
}

impl CutStyle {
    const ALL: [CutStyle; 2] = [CutStyle::Climb, CutStyle::Conventional];

    fn of(climb: bool) -> Self {
        if climb {
            CutStyle::Climb
        } else {
            CutStyle::Conventional
        }
    }
}

impl std::fmt::Display for CutStyle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            CutStyle::Climb => "Climb",
            CutStyle::Conventional => "Conventional",
        })
    }
}

/// A thread mill's cutting form — the single-tooth vs multiple-teeth toggle. Maps onto
/// `ThreadMill { pitch }`: single-point ⇔ `None` (one tooth, cuts any pitch by its
/// helical lead), full-form ⇔ `Some(pitch)` (a stack of teeth at a fixed pitch).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ThreadForm {
    SinglePoint,
    FullForm,
}

impl ThreadForm {
    const ALL: [ThreadForm; 2] = [ThreadForm::SinglePoint, ThreadForm::FullForm];
}

impl std::fmt::Display for ThreadForm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            ThreadForm::SinglePoint => "Single-point",
            ThreadForm::FullForm => "Full-form",
        })
    }
}

/// The kind-specific inspector fields for a tool of `kind` (empty for kinds fully
/// described by diameter).
fn tool_kind_fields(kind: ToolKind) -> Vec<Field> {
    match kind {
        ToolKind::BullNose { .. } => vec![Field::CornerRadius],
        ToolKind::RadiusMill { .. } => vec![Field::CornerRadius, Field::TipLength],
        ToolKind::ChamferMill { .. } => vec![Field::ChamferAngle, Field::TipDiameter],
        ToolKind::Drill { .. } => vec![Field::PointAngle],
        ToolKind::ThreadMill { .. } => vec![Field::ToolThreadPitch],
        ToolKind::VBit { .. } => vec![Field::PointAngle, Field::TipRadius],
        ToolKind::EndMill | ToolKind::RoughingMill | ToolKind::BallMill | ToolKind::FaceMill => {
            Vec::new()
        }
    }
}

/// Read a tool's kind-specific parameter for a field, if it applies.
fn tool_kind_field(kind: ToolKind, field: Field) -> Option<f64> {
    match (kind, field) {
        (
            ToolKind::BullNose { corner_radius } | ToolKind::RadiusMill { corner_radius, .. },
            Field::CornerRadius,
        ) => Some(corner_radius),
        (ToolKind::RadiusMill { tip_length, .. }, Field::TipLength) => Some(tip_length),
        (
            ToolKind::ChamferMill {
                included_angle_deg, ..
            },
            Field::ChamferAngle,
        ) => Some(included_angle_deg),
        (ToolKind::ChamferMill { tip_diameter, .. }, Field::TipDiameter) => Some(tip_diameter),
        (ToolKind::Drill { point_angle_deg }, Field::PointAngle) => Some(point_angle_deg),
        // Single-form (None) shows as 0 — "any pitch".
        (ToolKind::ThreadMill { pitch }, Field::ToolThreadPitch) => Some(pitch.unwrap_or(0.0)),
        (
            ToolKind::VBit {
                included_angle_deg, ..
            },
            Field::PointAngle,
        ) => Some(included_angle_deg),
        (ToolKind::VBit { tip_radius, .. }, Field::TipRadius) => Some(tip_radius),
        _ => None,
    }
}

/// Write the parsed inspector fields onto a tool's kind-specific parameters.
/// Write the parsed common tool dimensions onto `t`, enforcing the length constraint
/// **overall = flute + shank**: editing flute or shank recomputes overall; editing
/// overall recomputes the shank (flute held fixed), with shank floored at 0. The
/// effective flute (`flute_len()`) is materialised into `flute_length` on any edit so
/// the three stay consistent. Kind-specific parameters are handled separately by
/// [`apply_tool_kind_fields`].
fn apply_tool_dims(t: &mut cam_model::Tool, parsed: &BTreeMap<Field, f64>) {
    if let Some(&v) = parsed.get(&Field::ToolDiameter) {
        t.diameter = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::ShankDiameter) {
        t.shank_diameter = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::NeckLength) {
        t.neck_length = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::NeckDiameter) {
        t.neck_diameter = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::Flutes) {
        t.flutes = v.round().max(1.0) as u32;
    }
    // Nominal cutting data (library defaults). Clamped non-negative; 0 = unset.
    if let Some(&v) = parsed.get(&Field::NominalRpm) {
        t.nominal_rpm = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::NominalFeed) {
        t.nominal_feed = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::NominalPlungeFeed) {
        t.nominal_plunge_feed = v.max(0.0);
    }

    // overall = flute + shank. Snapshot the effective flute/shank *before* the edit.
    let old_flute = t.flute_len();
    let old_shank = (t.length - old_flute).max(0.0);
    let new_flute = parsed
        .get(&Field::FluteLength)
        .map(|&v| v.max(0.0))
        .unwrap_or(old_flute);
    t.flute_length = new_flute; // materialise (drops the 0-sentinel)
    let new_length = if let Some(&sh) = parsed.get(&Field::ShankLength) {
        new_flute + sh.max(0.0) // shank edited ⇒ overall follows
    } else if let Some(&ov) = parsed.get(&Field::ToolLength) {
        ov // overall edited ⇒ shank absorbs (derived below)
    } else {
        new_flute + old_shank // flute may have changed; preserve the shank
    };
    t.length = new_length.max(new_flute); // shank ≥ 0
}

fn apply_tool_kind_fields(kind: &mut ToolKind, parsed: &BTreeMap<Field, f64>) {
    let get = |f: Field| parsed.get(&f).copied();
    match kind {
        ToolKind::BullNose { corner_radius } => {
            if let Some(v) = get(Field::CornerRadius) {
                *corner_radius = v;
            }
        }
        ToolKind::RadiusMill {
            corner_radius,
            tip_length,
        } => {
            if let Some(v) = get(Field::CornerRadius) {
                *corner_radius = v;
            }
            if let Some(v) = get(Field::TipLength) {
                *tip_length = v;
            }
        }
        ToolKind::ChamferMill {
            included_angle_deg,
            tip_diameter,
        } => {
            if let Some(v) = get(Field::ChamferAngle) {
                *included_angle_deg = v;
            }
            if let Some(v) = get(Field::TipDiameter) {
                *tip_diameter = v;
            }
        }
        ToolKind::Drill { point_angle_deg } => {
            if let Some(v) = get(Field::PointAngle) {
                *point_angle_deg = v;
            }
        }
        ToolKind::ThreadMill { pitch } => {
            if let Some(v) = get(Field::ToolThreadPitch) {
                // The single-point/full-form choice is the toggle's job; the pitch field
                // only sets the value, and only when already full-form.
                if pitch.is_some() && v > 0.0 {
                    *pitch = Some(v);
                }
            }
        }
        ToolKind::VBit {
            included_angle_deg,
            tip_radius,
        } => {
            if let Some(v) = get(Field::PointAngle) {
                *included_angle_deg = v;
            }
            if let Some(v) = get(Field::TipRadius) {
                *tip_radius = v.max(0.0);
            }
        }
        ToolKind::EndMill | ToolKind::RoughingMill | ToolKind::BallMill | ToolKind::FaceMill => {}
    }
}

/// An editable inspector field, keyed independently of which node owns it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Field {
    Wrap(rotary_view::Number),
    LiftHeight,
    OperationTop,
    StartX,
    StartY,
    Clearance,
    Retract,
    TopOfStock,
    /// Setup: the Z the planner lifts to before a tool change or reorientation.
    ToolChangeHeight,
    /// Stock: material added on both X sides of the part's bounding box (mm).
    StockXOffset,
    /// Stock: material added on both Y sides of the part's bounding box (mm).
    StockYOffset,
    /// Stock: absolute Z of the top surface (mm).
    StockTop,
    /// Stock: block thickness below the top (mm); bottom = top − thickness.
    StockThickness,
    /// Machine travel (working volume) on X / Y / Z, mm.
    MachineTravelX,
    MachineTravelY,
    MachineTravelZ,
    MachineMaxZ,
    MachineRapidRate,
    MachineMinSpindleRpm,
    MachineMaxSpindleRpm,
    MachineMaxFeed,
    MachineToolChangeSeconds,
    /// Workpiece origin (datum) X / Y / Z, part-space mm.
    OriginX,
    OriginY,
    OriginZ,
    /// The active origin's machine work-coordinate index (`G54`-`G59`, or
    /// `G15 H<n>` on Okuma — see `PostKind::datum_label`).
    OriginIndex,
    ToolDiameter,
    ToolLength,
    /// Length of cut (flute length) from the tip, mm. Also reused, kind-aware, as a face
    /// mill's body height and a thread mill's thread length.
    FluteLength,
    /// Shank diameter, mm (a face mill's arbor diameter, kind-aware).
    ShankDiameter,
    /// Shank length, mm — a *derived* field: overall = flute + shank (editing it moves
    /// overall length; editing overall moves it back).
    ShankLength,
    /// Reduced-neck length from the cutting end, mm; 0 = no reduced neck.
    NeckLength,
    /// Reduced-neck diameter, mm; 0 = same as the cutting diameter.
    NeckDiameter,
    Flutes,
    /// Tool nominal spindle speed (rpm) — the library default that seeds a new
    /// operation's own RPM. `0` = unset.
    NominalRpm,
    /// Tool nominal cutting feed (mm/min) — seeds a new operation's feed. `0` = unset.
    NominalFeed,
    /// Tool nominal plunge feed (mm/min) — seeds a new operation's plunge feed.
    /// `0` = unset.
    NominalPlungeFeed,
    /// Bull-nose corner radius (mm).
    CornerRadius,
    TipLength,
    /// Chamfer/V mill included point angle (deg).
    ChamferAngle,
    /// Chamfer/V mill flat-tip diameter (mm).
    TipDiameter,
    /// V-bit rounded-tip radius (mm).
    TipRadius,
    /// Drill point angle (deg).
    PointAngle,
    /// Thread mill's ground pitch (mm); 0 means single-form (any pitch).
    ToolThreadPitch,
    BoreDiameter,
    BoreTaper,
    BorePitch,
    BoreAngle,
    BoreStepovers,
    BoreStepover,
    BoreFinishing,
    BoreRadialStock,
    BoreAxialStock,
    BoreTolerance,
    BoreSafeDistance,
    BoreLeadRadius,

    Depth,
    /// Drill start plane, as a distance below the stock top (mm); 0 = at the top.
    DrillStartOffset,
    /// Drill peck increment (mm); 0 = drill straight to depth (no peck).
    Peck,
    /// Drill dwell at the hole bottom (seconds); 0 = no dwell.
    Dwell,
    Stepdown,
    SpiralAxialOffset,
    SpiralStepdowns,
    SpiralStepover,
    SpiralInnerLimit,
    SpiralOuterLimit,
    SpiralFromSlope,
    SpiralToSlope,
    SpiralStayDownDistance,
    ParallelStepover,
    ParallelAngle,
    ScallopSafeDistance,
    ScallopSmoothingTolerance,
    AdaptiveTolerance,
    AdaptiveOptimalLoad,
    AdaptiveFineStepdown,
    AdaptiveOffset,
    StockLeaveRadial,
    StockLeaveAxial,
    Stepover,
    /// Profile finishing allowance left on the wall (mm).
    ProfileOffset,
    Profile(profile_parameters::Number),
    PocketFinish(pocket_parameters::Number),
    TabWidth,
    TabHeight,
    TabSpacing,
    /// Operation spindle speed (rpm) — `M3 S<rpm>`. Seeded from the tool's nominal
    /// RPM when the operation is created; `0` falls back to the job default.
    SpindleRpm,
    CuttingSpeed,
    Feed,
    FeedPerTooth,
    PlungeFeed,
    RampFeed,
    LeadInFeed,
    LeadOutFeed,
    /// Thread major diameter (mm).
    MajorDia,
    /// Thread pitch (mm).
    Pitch,
    /// Absolute Z of the top of a threaded length (mm).
    ThreadTop,
    /// Absolute Z of the bottom of a threaded length (mm).
    ThreadBottom,
    /// Thread radial infeed passes (count, ≥1).
    ThreadPasses,
    /// Thread spring passes at full depth (count, 0 = none).
    ThreadSpringPasses,
    /// Thread blind-hole drill clearance below the thread bottom (mm); 0 = through hole.
    ThreadDrillClearance,
    /// Thread blind-hole required allowance (mm); 0 = auto (one pitch).
    ThreadBlindAllowance,
    /// Chamfer: absolute Z of the top edge being chamfered (mm). Seeded from the
    /// stock top, editable because the edge is not always there.
    ChamferTop,
    /// Chamfer width (mm).
    ChamferWidth,
    /// Chamfer tip depth below the top edge (mm); selects the flank section.
    ChamferDepth,
    /// Additional tip clearance below the chamfer.
    ChamferTipOffset,
    /// Chamfer width increment per pass (mm); 0 cuts in one pass.
    ChamferStep,
    ContourRoughStepover,
    ContourRoughSmoothing,
    ContourRoughCount,
    /// Lead-in size (length for Linear, radius for Arc).
    LeadInSize,
    /// Lead-out size.
    LeadOutSize,
    /// Closure overlap: distance kept cutting past the start to erase the
    /// entry/exit witness (profile, pocket, chamfer).
    LeadOverlap,
    /// Face: top cutting plane above the drawing reference (mm).
    FaceStartOffset,
    /// Face: pass overlap as a percentage of the tool diameter.
    Overlap,
    /// Maximum horizontal spacing between concentric pocket passes.
    PocketStepover,
    FaceOverlap,
    /// Face: overshoot past the stock edge before the turnaround (mm).
    FaceOvershoot,
    /// Adaptive-clearing stepover: the radial width of cut along a *straight* wall
    /// (mm); 0 disables adaptive clearing (pocket, profile outside-roughing). Not a
    /// hard ceiling — see the tooltip on the geometric floor.
    Engagement,
    /// Plunge parameter A: ramp/zig-zag angle, or helix radius.
    PlungeA,
    /// Plunge parameter B: zig-zag length, or helix pitch.
    PlungeB,
    /// Carve: radial spacing of the carve rings (mm); 0 = auto.
    RingStep,
    /// Carve clearing pass: max depth per pass (mm); 0 = full depth in one.
    ClearStepdown,
    /// Carve clearing pass: ring overlap as a percentage of the tool diameter.
    ClearOverlap,
    /// Carve clearing pass: adaptive stepover along a straight wall (mm); 0 = plain
    /// concentric.
    ClearEngagement,
    /// Carve clearing pass: cutting feed (mm/min); 0 inherits the carve's.
    ClearFeed,
    /// Carve clearing pass: plunge feed (mm/min); 0 inherits the carve's.
    ClearPlungeFeed,
    /// Carve clearing pass: closure overlap past the start (mm).
    ClearLeadOverlap,
    /// Carve clearing pass: lead-in size (mm).
    ClearLeadInSize,
    /// Carve clearing pass: lead-out size (mm).
    ClearLeadOutSize,
    /// Carve clearing pass: plunge parameter A (ramp/zig-zag angle, or helix radius).
    ClearPlungeA,
    /// Carve clearing pass: plunge parameter B (zig-zag length, or helix pitch).
    ClearPlungeB,
    /// Carve clearing pass: allowance left off the carved surface for the V-bit (mm).
    ClearOffset,
    /// Carve: target ridge height left on the flat floor (mm); 0 = auto.
    Scallop,
}

fn set_machine_z_limits(envelope: &mut cam_model::Envelope, travel: f64, max_z: f64) {
    envelope.max.z = max_z;
    envelope.min.z = max_z - travel;
}

impl Field {
    fn label(self) -> &'static str {
        if let Self::Wrap(number) = self {
            return number.label();
        }
        match self {
            Self::Wrap(_) => unreachable!(),
            Field::OperationTop => "Cutting top Z (mm)",
            Field::StartX => "Start X (mm)",
            Field::StartY => "Start Y (mm)",
            Field::Clearance => "Clearance (mm)",
            Field::Retract => "Retract (mm)",
            Field::ToolChangeHeight => "Tool Change Height (mm)",
            Field::TopOfStock => "Top of stock (mm)",
            Field::StockXOffset => "X offset (mm)",
            Field::StockYOffset => "Y offset (mm)",
            Field::StockTop => "Stock top (mm)",
            Field::StockThickness => "Thickness (mm)",
            Field::MachineTravelX => "X travel (mm)",
            Field::MachineTravelY => "Y travel (mm)",
            Field::MachineTravelZ => "Z travel (mm)",
            Field::MachineMaxZ => "Maximum Z from base WCS (mm)",
            Field::MachineRapidRate => "Rapid rate (mm/min)",
            Field::MachineMinSpindleRpm => "Minimum spindle speed (rpm)",
            Field::MachineMaxSpindleRpm => "Maximum spindle speed (rpm)",
            Field::MachineMaxFeed => "Maximum cutting feed (mm/min)",
            Field::MachineToolChangeSeconds => "Tool change time (s)",
            Field::OriginX => "Origin X (mm)",
            Field::OriginY => "Origin Y (mm)",
            Field::OriginZ => "Origin Z (mm)",
            Field::OriginIndex => "H index",
            Field::ToolDiameter => "Flute diameter (mm)",
            Field::ToolLength => "Overall length (mm)",
            Field::FluteLength => "Flute length (mm)",
            Field::ShankDiameter => "Shank diameter (mm)",
            Field::ShankLength => "Shank length (mm)",
            Field::NeckLength => "Neck length (mm, 0=none)",
            Field::NeckDiameter => "Neck ⌀ (mm, 0=cut ⌀)",
            Field::Flutes => "Flutes",
            Field::NominalRpm => "Nominal RPM",
            Field::NominalFeed => "Nominal feed (mm/min)",
            Field::NominalPlungeFeed => "Nominal plunge (mm/min)",
            Field::CornerRadius => "Corner radius (mm)",
            Field::TipLength => "Tip length (mm)",
            Field::ChamferAngle => "Point angle (deg)",
            Field::TipDiameter => "Tip ⌀ (mm)",
            Field::TipRadius => "Tip radius (mm)",
            Field::PointAngle => "Point angle (deg)",
            Field::BoreDiameter => "Diameter (mm)",
            Field::BoreTaper => "Taper angle (deg)",
            Field::BorePitch => "Pitch (mm/rev)",
            Field::BoreAngle => "Ramp angle (deg)",
            Field::BoreStepovers => "Roughing stepovers",
            Field::BoreStepover => "Radial stepover (mm)",
            Field::BoreFinishing => "Finishing stepover (mm, 0=off)",
            Field::BoreRadialStock => "Radial stock to leave (mm)",
            Field::BoreAxialStock => "Axial stock to leave (mm)",
            Field::BoreTolerance => "Tolerance (mm)",
            Field::BoreSafeDistance => "Safe distance (mm)",
            Field::LiftHeight => "Lift height (mm)",
            Field::BoreLeadRadius => "Horizontal lead radius (mm)",
            Field::ToolThreadPitch => "Pitch (mm)",
            Field::Depth => "Depth (mm)",
            Field::DrillStartOffset => "Start offset (mm)",
            Field::Peck => "Peck (mm, 0=off)",
            Field::Dwell => "Dwell (s, 0=off)",
            Field::TabWidth => "Tab width (mm)",
            Field::TabHeight => "Tab height (mm)",
            Field::TabSpacing => "Tab spacing (mm)",
            Field::ProfileOffset => "Wall allowance (mm)",
            Field::Profile(n) => n.label(),
            Field::PocketFinish(n) => n.label(),
            Field::Stepdown => "Stepdown (mm)",
            Field::SpiralAxialOffset => "Axial offset (mm)",
            Field::SpiralStepdowns => "Number of stepdowns",
            Field::SpiralStepover => "Stepover (mm)",
            Field::SpiralInnerLimit => "Inner limit (mm)",
            Field::SpiralOuterLimit => "Outer limit (mm)",
            Field::SpiralFromSlope => "From slope (deg)",
            Field::SpiralToSlope => "To slope (deg)",
            Field::SpiralStayDownDistance => "Maximum stay-down distance (mm)",
            Field::ParallelStepover => "Stepover (mm)",
            Field::ParallelAngle => "Pass angle (deg)",
            Field::ScallopSafeDistance => "Safe distance (mm)",
            Field::ScallopSmoothingTolerance => "Smoothing tolerance (mm)",
            Field::AdaptiveTolerance => "Tolerance (mm)",
            Field::AdaptiveOptimalLoad => "Optimal load (mm)",
            Field::AdaptiveFineStepdown => "Fine stepdown (mm)",
            Field::AdaptiveOffset => "Additional offset (mm)",
            Field::StockLeaveRadial => "Radial stock to leave (mm)",
            Field::StockLeaveAxial => "Axial stock to leave (mm)",
            Field::Stepover => "Stepover (mm)",
            Field::SpindleRpm => "Spindle (rpm)",
            Field::CuttingSpeed => "Cutting speed (m/min)",
            Field::Feed => "Feed (mm/min)",
            Field::FeedPerTooth => "Feed per tooth (mm/tooth)",
            Field::PlungeFeed => "Plunge feed (mm/min)",
            Field::RampFeed => "Ramp feed (mm/min)",
            Field::LeadInFeed => "Lead-in feed (mm/min)",
            Field::LeadOutFeed => "Lead-out feed (mm/min)",
            Field::MajorDia => "Major ⌀ (mm)",
            Field::Pitch => "Pitch (mm)",
            Field::ThreadTop => "Thread top (mm)",
            Field::ThreadBottom => "Thread bottom (mm)",
            Field::ThreadPasses => "Passes",
            Field::ThreadSpringPasses => "Spring passes",
            Field::ThreadDrillClearance => "Drill clearance (mm, 0=through)",
            Field::ThreadBlindAllowance => "Blind allowance (mm, 0=auto)",
            Field::ChamferTop => "Top edge Z (mm)",
            Field::ChamferWidth => "Chamfer width (mm)",
            Field::ChamferDepth => "Tip depth (mm, 0=tip)",
            Field::ChamferTipOffset => "Chamfer tip offset (mm)",
            Field::ChamferStep => "Step (mm, 0=one pass)",
            Field::ContourRoughStepover => "Maximum stepover (mm)",
            Field::PocketStepover => "Maximum Stepover (mm)",
            Field::ContourRoughCount => "Number of stepovers",
            Field::ContourRoughSmoothing => "Smoothing deviation (mm)",
            Field::LeadInSize => "Lead-in size (mm)",
            Field::LeadOutSize => "Lead-out size (mm)",
            Field::LeadOverlap => "Lead overlap (mm)",
            Field::FaceStartOffset => "Start offset (mm)",
            Field::Overlap => "Overlap (%)",
            Field::FaceOverlap => "Overlap (mm)",
            Field::FaceOvershoot => "Overshoot (mm)",
            Field::Engagement => "Adaptive stepover (mm, 0=off)",
            Field::RingStep => "Ring step (mm, 0=auto)",
            Field::ClearStepdown => "Clear stepdown (mm, 0=full)",
            Field::ClearOverlap => "Clear overlap (%)",
            Field::ClearEngagement => "Clear adaptive stepover (0=off)",
            Field::ClearFeed => "Clear feed (mm/min, 0=same)",
            Field::ClearPlungeFeed => "Clear plunge feed (0=same)",
            Field::ClearLeadOverlap => "Clear lead overlap (mm)",
            Field::ClearLeadInSize => "Clear lead-in size (mm)",
            Field::ClearLeadOutSize => "Clear lead-out size (mm)",
            Field::ClearPlungeA => "Clearing plunge parameter 1",
            Field::ClearPlungeB => "Clearing plunge parameter 2",
            Field::ClearOffset => "Clear allowance (mm)",
            Field::Scallop => "Floor scallop (mm, 0=auto)",
            // Fallbacks only: `plunge_label` names these per style (a ramp's is an
            // angle, a helix's a radius — never both).
            Field::PlungeA => "Plunge parameter 1",
            Field::PlungeB => "Plunge parameter 2",
        }
    }

    /// A one- or two-sentence, new-user-friendly explanation of what the field does,
    /// shown as a hover tooltip on its label in the inspector.
    fn help(self) -> &'static str {
        match self {
            Field::Wrap(_) => "Cylindrical wrap in millimetres. Radius plus offset defines the development circumference. Surface Z is the flat drawing's radial reference; the seam sets rotary zero. Cylinder selection derives radius and axis centre from CAD.",
            Field::LiftHeight => "Distance above the cutting level for local linking moves, in millimetres. Adaptive links that withdraw from walls lift at least 0.2 mm. Other zero-height links remain at depth. Lifts are limited to clearance; distant links still retract fully.",
            Field::BoreDiameter => "Finished feature diameter at the top height.",
            Field::BoreTaper => "Positive narrows downward; negative widens downward. Undercuts are rejected.",
            Field::BorePitch => "Axial travel per revolution of the bore helix.",
            Field::BoreAngle => "Maximum ramp angle, evaluated at the smallest tool-centre orbit.",
            Field::BoreStepovers => "Additional radial passes before the final wall pass.",
            Field::BoreStepover => "Radial distance between successive helical passes.",
            Field::BoreFinishing => "Radial allowance removed by an additional finishing helix.",
            Field::BoreRadialStock => "Positive leaves wall stock; negative removes material beyond the nominal wall.",
            Field::BoreAxialStock => "Raises the cut bottom by this amount. Negative extends below the bottom.",
            Field::BoreTolerance => "Maximum chord error when linearizing a tapered helix.",
            Field::BoreSafeDistance => "Minimum standoff above the feature at the retract height and outside boss leads.",
            Field::BoreLeadRadius => "Radius of the horizontal tangent lead. For holes it is limited to half the tool-centre orbit; for bosses the safe distance can increase it.",

            Field::OperationTop => "Absolute cutting top in setup Z, in millimetres.",
            Field::SpiralAxialOffset => "Thickness above the final surface to remove across multiple passes.",
            Field::SpiralStepdowns => "Enter zero to calculate the pass count from the maximum stepdown.",
            Field::SpiralStepover => "Horizontal radial spacing between turns or concentric passes.",
            Field::SpiralInnerLimit => "Minimum radius measured from the spiral center.",
            Field::SpiralOuterLimit => "Maximum radius measured from the spiral center. Zero uses the machining boundary.",
            Field::SpiralFromSlope => "Minimum selected surface inclination from horizontal.",
            Field::SpiralToSlope => "Maximum selected surface inclination from horizontal.",
            Field::SpiralStayDownDistance => "Maximum horizontal length for a checked low link. Zero uses clearance for separated passes.",
            Field::ParallelStepover => "Target distance between finishing passes, in millimetres. Scallop uses approximate surface distance.",
            Field::ParallelAngle => "Direction of the cutting rows measured counterclockwise from setup X, in degrees.",
            Field::ScallopSafeDistance => "Vertical clearance above the model for minimum retraction. Unsafe links return to full clearance.",
            Field::ScallopSmoothingTolerance => "Maximum additional filtering deviation from the generated path. This is added to the machining tolerance.",
            Field::AdaptiveTolerance => "Machining tolerance in millimetres. Scallop smoothing, when enabled, adds its separate filtering tolerance.",
            Field::AdaptiveOptimalLoad => "Target radial engagement for adaptive clearing, in millimetres.",
            Field::AdaptiveFineStepdown => "Axial spacing for intermediate terrace refinement, in millimetres. Fine passes are added only where shelves or slopes expose additional material; unchanged vertical walls use the primary stepdown.",
            Field::AdaptiveOffset => "Signed shift of the selected adaptive boundary. Positive values move it outward, except Inside containment moves it inward.",
            Field::StockLeaveRadial => "Material left on vertical walls for a later finishing operation, in millimetres.",
            Field::StockLeaveAxial => "Material left on horizontal floors for a later finishing operation, in millimetres.",
            Field::StartX => "Preferred starting X in setup millimetres; the strategy chooses an eligible point near it.",
            Field::StartY => "Preferred starting Y in setup millimetres; the strategy chooses an eligible point near it.",
            Field::Clearance => {
                "Safe height for rapid (non-cutting) moves above the part. The tool \
                 traverses at this Z between features so it never drags through stock."
            }
            Field::Retract => {
                "Height the tool pulls back to between passes or plunges — lower than \
                 Clearance for speed, but still above any uncut material."
            }
            Field::ToolChangeHeight => {
                "Z relative to the base workpiece origin that the tool lifts to before \
                 a tool change (M6) or a manual reorientation \
                 (M00) — above the Clearance plane, so the changer or your hands have room. \
                 Defaults to the top of the machine's Z travel; set it lower only if you \
                 have a reason."
            }
            Field::TopOfStock => {
                "Absolute Z of the top surface of the raw material. Cutting depths are \
                 measured downward from here."
            }
            Field::StockXOffset => {
                "Extra material left on both +X and −X sides of the part's bounding box \
                 — the raw block is this much wider than the part in X."
            }
            Field::StockYOffset => {
                "Extra material left on both +Y and −Y sides of the part's bounding box \
                 — the raw block is this much wider than the part in Y."
            }
            Field::StockTop => "Absolute Z of the stock's top surface (mm).",
            Field::StockThickness => {
                "Height of the raw block below its top: the bottom sits at (top − \
                 thickness)."
            }
            Field::MachineTravelX => {
                "Working-envelope size in X. Toolpaths that exceed it are flagged so a \
                 program can't command the machine past its limits."
            }
            Field::MachineTravelY => "Working-envelope size in Y (see X travel).",
            Field::MachineTravelZ => "Working-envelope size in Z (see X travel).",
            Field::MachineMaxZ => "Assumed highest reachable Z relative to the setup's base work origin. Used for automatic tool-change height; measure it for the workholding and setup. Changing it preserves Z travel.",
            Field::MachineRapidRate => "Rapid traverse rate used for time estimates.",
            Field::MachineMinSpindleRpm => "Lowest running spindle speed allowed during G-code export. Zero disables the additional minimum.",
            Field::MachineMaxSpindleRpm => "Highest spindle speed allowed during G-code export.",
            Field::MachineMaxFeed => "Highest cutting, plunge, or drilling feed allowed during G-code export.",
            Field::MachineToolChangeSeconds => "Estimated seconds spent at each tool change.",
            Field::OriginX => {
                "Part-space X of the workpiece datum — the (0,0,0) the G-code is written \
                 about. Set it to the corner or feature you'll touch off on the machine."
            }
            Field::OriginY => "Part-space Y of the workpiece datum (see Origin X).",
            Field::OriginZ => "Part-space Z of the workpiece datum (see Origin X).",
            Field::OriginIndex => {
                "The machine work-coordinate index this origin selects — the fixture \
                 offset you've taught on the control. How it is written depends on the \
                 post: G54-G59 on ISO controls (so only six exist), G15 H<n> on Okuma. \
                 Choosing an index another origin already uses swaps the two."
            }
            Field::ToolDiameter => "The tool's cutting diameter (mm).",
            Field::ToolLength => {
                "Overall length of the tool — the stickout below the holder. Used for \
                 reach checks and the backplot; does not change the cutting geometry."
            }
            Field::FluteLength => {
                "Length of the cutting edge (length of cut) measured from the tip. The \
                 rest of the tool, up to the overall length, is the non-cutting shank; \
                 the split is used for gouge/collision checks."
            }
            Field::ShankDiameter => {
                "Diameter of the non-cutting shank above the cutting portion (typically \
                 equal to the cutting diameter)."
            }
            Field::ShankLength => {
                "Length of the non-cutting shank above the cutting portion. Overall length \
                 = cutting length + shank length; editing the shank moves the overall \
                 length, and editing the overall length moves the shank."
            }
            Field::NeckLength => {
                "Length of a reduced-diameter neck above the flutes (reach/stub tools). \
                 0 = no reduced neck."
            }
            Field::NeckDiameter => {
                "Diameter of the reduced neck. 0 = the same as the cutting diameter."
            }
            Field::Flutes => {
                "Number of cutting edges (or inserts). Informational for now — \
                 feed-per-tooth math comes later; it does not change the toolpath."
            }
            Field::NominalRpm => {
                "Default spindle speed for this tool (rpm). Seeds a new operation's RPM \
                 when the tool is chosen; the operation can override it. 0 = unset."
            }
            Field::NominalFeed => {
                "Default cutting feed for this tool (mm/min). Seeds a new operation's feed; \
                 the operation can override it. A per-tool feed is only a starting point — \
                 it depends on the cut too. 0 = unset (uses the job default)."
            }
            Field::NominalPlungeFeed => {
                "Default plunge (Z-entry) feed for this tool (mm/min). Seeds a new \
                 operation's plunge feed; the operation can override it. 0 = unset."
            }
            Field::TipLength => "Length of the cylindrical tip below the rounded cutting edge.",
            Field::CornerRadius => {
                "Corner radius of the rounded-edge (bull-nose) end mill — the fillet \
                 between the flat bottom and the side. Must be smaller than the tool \
                 radius (⌀/2)."
            }
            Field::ChamferAngle => {
                "Included angle of the chamfer mill's cone (the full point angle, e.g. \
                 90° for a 45° chamfer). Sets how cut depth maps to chamfer width."
            }
            Field::TipDiameter => {
                "Diameter of the flat, non-cutting tip of a chamfer mill (min 0.10 mm — \
                 a chamfer mill is always ground with a flat, and one cannot be ground \
                 sharper). Only the angled flank cuts, which is why a chamfer mill \
                 cannot engrave: use a V-bit."
            }
            Field::TipRadius => {
                "Rounded-tip radius of a carving V-bit (min 0.05 mm — a point is always \
                 ground to a radius, and one cannot be ground sharper). Unlike a \
                 chamfer mill's flat, this rounded tip cuts, which is what lets a V-bit \
                 engrave."
            }
            Field::PointAngle => {
                "Included angle of the tool's point (the full cone angle) — e.g. a \
                 drill's 118°/135° point, or a V-bit's 60°/90° included angle."
            }
            Field::ToolThreadPitch => {
                "Ground pitch of a full-form thread mill (mm) — the axial spacing of its \
                 stacked teeth. (Single-point mills, set via the toggle, have no fixed \
                 pitch.)"
            }
            Field::Depth => {
                "Total cut depth below the top of stock (a positive distance). The \
                 feature's floor sits this far down."
            }
            Field::DrillStartOffset => {
                "Where the hole begins, as a height above the stock top (mm) — the same \
                 convention as the facing start offset. 0 starts at the stock top; a \
                 positive value starts above it (a proud boss); negative starts below \
                 (a recessed or faced surface). Depth is measured down from here."
            }
            Field::Peck => {
                "Peck-drilling increment (mm): the drill cuts this deep, fully retracts \
                 to clear chips, then returns and repeats until it reaches depth. \
                 0 = drill straight to depth in one plunge (no pecking)."
            }
            Field::Dwell => {
                "Dwell at the bottom of the hole (seconds): the tool pauses with the \
                 spindle turning before retracting, cleaning up the hole bottom. \
                 0 = no dwell."
            }
            Field::TabWidth => "Retained tab width along the contour. Tool diameter is added to the protected travel length.",
            Field::TabHeight => "Tab height above the final profile floor; must be less than the cut depth.",
            Field::TabSpacing => "Maximum distance between tab centres. Tabs are distributed evenly along each toolpath contour.",
            Field::Profile(n) => n.help(),
            Field::PocketFinish(n) => n.help(),
            Field::ProfileOffset => {
                "Finishing allowance left on the wall (mm): the roughing pass stops this \
                 far from the final profile so a finish pass can clean it up. 0 = cut to \
                 size."
            }
            Field::Stepdown => {
                "Maximum depth removed per Z level. The cut is split into passes no \
                 deeper than this; smaller is gentler on the tool."
            }
            Field::Stepover => {
                "Radial width of cut between adjacent passes (mm). For outside roughing \
                 it sets the concentric-pass spacing."
            }
            Field::SpindleRpm => {
                "Spindle speed for this operation (rpm), emitted as M3 S<rpm>. Seeded from \
                 the tool's nominal RPM when the operation is created; edit it for this cut. \
                 0 falls back to the job default."
            }
            Field::CuttingSpeed => "Surface speed at the cutter diameter: π × diameter × RPM / 1000 (m/min). Editing this value updates spindle RPM.",
            Field::FeedPerTooth => "Feed per cutting tooth: cutting feed / (RPM × flute count), in mm/tooth. Editing this value updates cutting feed. RPM changes recalculate this value while retaining cutting feed.",
            Field::Feed => "Cutting feed rate — how fast the tool advances while cutting (mm/min).",
            Field::RampFeed => "Ramp, helix and zigzag entry feed in mm/min. Zero inherits the cutting feed.",
            Field::LeadInFeed => "Lead-in feed in mm/min. Zero inherits the cutting feed. Used when a lead-in is generated.",
            Field::LeadOutFeed => "Lead-out feed in mm/min. Zero inherits the cutting feed. Used when a lead-out is generated.",
            Field::PlungeFeed => {
                "Feed rate for downward (Z) entry moves — usually slower than the \
                 cutting feed, since end mills cut poorly straight down (mm/min)."
            }
            Field::MajorDia => {
                "Major (nominal) diameter of the thread — the crest diameter for an \
                 external thread, the bore's tapped size for internal (mm). CAD selection \
                 seeds this from the cylinder diameter; set the required thread size \
                 explicitly. An existing internal bore must be smaller than the major \
                 diameter for the thread mill to engage material."
            }
            Field::Pitch => {
                "Thread pitch: the axial distance advanced per revolution (mm). One turn \
                 of the helix climbs by this much."
            }
            Field::ThreadTop => "Absolute Z of the top of the threaded length (mm).",
            Field::ThreadBottom => "Absolute Z of the bottom of the threaded length (mm).",
            Field::ThreadPasses => {
                "Radial infeed passes: cut the thread to full depth in this many equal \
                 radial steps (each a full helix). More passes lighten the cut for hard \
                 material. 1 = single full-depth pass."
            }
            Field::ThreadSpringPasses => {
                "Extra spring passes at full depth to clean up elastic spring-back after \
                 the last cutting pass. 0 = none."
            }
            Field::ThreadDrillClearance => {
                "For a blind hole: how far the pre-drilled hole extends below the thread \
                 bottom (mm). Must be at least the blind allowance. 0 = through hole (no \
                 check)."
            }
            Field::ThreadBlindAllowance => {
                "Required clearance between the last thread and the bottom of a blind hole \
                 (mm) — the tool cannot thread flush to a blind bottom. 0 = auto (one pitch)."
            }
            Field::ChamferTop => {
                "Absolute Z of the edge being chamfered (mm) — where the bevel starts, \
                 and what Tip depth is measured down from. Seeded from the top of \
                 stock, which is right for an edge on the raw surface; set it lower to \
                 chamfer the rim of a pocket or a step that an earlier operation cut."
            }
            Field::ChamferWidth => {
                "Horizontal width of the chamfer from the selected edge. For a radius mill, the tool corner radius determines the roundover width instead."
            }
            Field::ChamferDepth => {
                "How far below the top edge the tool tip rides (mm), which picks where \
                 on the tool's flank the cut lands. 0 = cut at the tip."
            }
            Field::ChamferTipOffset => "Additional distance below the chamfer edge. The tool moves outward by the corresponding amount to keep the cutting flank on the same bevel. Radius mills require zero tip offset.",
            Field::ContourRoughStepover => "Distance between adjacent contour roughing passes in the XY plane.",
            Field::ContourRoughSmoothing => "Maximum arc-fitting deviation for roughing contours; must not exceed the stepover.",
            Field::ContourRoughCount => "Additional passes on the air side before the final contour.",
            Field::ChamferStep => {
                "Chamfer width added per pass (mm) for a multi-pass chamfer. 0 = cut the \
                 whole chamfer in a single pass."
            }
            Field::LeadInSize => {
                "Size of the lead-in that eases the tool onto the wall — arc radius for \
                 an arc lead, tangent length for a linear one (mm)."
            }
            Field::LeadOutSize => {
                "Size of the lead-out that eases the tool off the wall (see lead-in)."
            }
            Field::LeadOverlap => {
                "How far the tool keeps cutting past the start point before leaving, to \
                 erase the entry/exit witness mark (mm). Adjoining adaptive cuts repeat this \
                 distance from their shared seam; 0 uses one exact junction."
            }
            Field::FaceStartOffset => {
                "Height of the first facing cut above the drawing reference (mm) — start \
                 above the true top to skim scale, or at 0 to cut to size."
            }
            Field::PocketStepover => "Maximum horizontal distance between adjacent concentric roughing passes. Enter a value up to the tool diameter.",
            Field::FaceOverlap => "Distance shared by neighboring facing passes, in millimetres.",
            Field::Overlap => {
                "How much each facing pass overlaps the previous, as a percent of tool \
                 diameter. Higher = smoother floor, more passes."
            }
            Field::FaceOvershoot => {
                "How far the tool runs past the stock edge before turning around (mm), \
                 so the cutter fully clears the edge."
            }
            Field::Engagement => {
                "Adaptive-clearing stepover: the radial width of cut taken along a \
                 STRAIGHT wall (mm). Keeps tool load roughly constant for high-speed \
                 clearing. 0 = plain concentric clearing (off). Climb only.\n\n\
                 It is not a hard maximum. Where the path curves tightly — inner \
                 corners, the loops near a pocket's centre — the geometry forces more \
                 than you ask for, up to roughly 1.4x this value, and no spiral clearer \
                 can do better. What adaptive clearing removes is the full-diameter \
                 SLOT, which is the tool-breaking hazard; the residual rise on tight \
                 loops is a feed-rate matter, so slow the feed if the tool complains."
            }
            // Reached only with no operation selected — with one, `plunge_help` names the
            // quantity the chosen style actually uses. Deliberately does not enumerate
            // "angle or radius": that positional wording is what the kind-aware labels and
            // tooltips exist to replace.
            Field::PlungeA | Field::PlungeB => {
                "A parameter of the plunge style chosen below. Pick a style and this box \
                 is labelled with the quantity it takes."
            }
            Field::RingStep => {
                "How far apart the WALL rings step inward (mm) -- a roughing control, not \
                 a finish one. The finished wall is cut by the deepest ring alone, whose \
                 flank spans from the boundary to its tip; the shallower rings limit how \
                 much one pass takes and reach into corners. Coarser costs tool load, not \
                 surface quality. 0 = choose automatically."
            }
            Field::Scallop => {
                "The ridge you will accept on the flat FLOOR (mm) -- the real finish \
                 control. A cone cannot leave a flat floor, so adjacent passes leave a \
                 ridge between them; asking for a height rather than a spacing lets the \
                 spacing open up where the tool's geometry allows. 0 = a fine default."
            }
            Field::ClearStepdown => {
                "Maximum depth the clearing tool removes per pass (mm). 0 takes the whole \
                 flat-area depth in one pass."
            }
            Field::ClearOverlap => {
                "How much each clearing ring overlaps the previous, as a percent of the \
                 clearing tool's diameter. Higher = smoother floor, more passes."
            }
            Field::ClearEngagement => {
                "Adaptive-clearing stepover for the clearing pass: the radial width of \
                 cut it takes along a STRAIGHT wall (mm). 0 = plain concentric \
                 clearing. Climb only. As above, tight loops geometrically exceed it by \
                 up to about 1.4x — that is a feed-rate matter, not a slot."
            }
            Field::ClearFeed => {
                "Cutting feed for the clearing pass (mm/min). 0 uses the carve's own \
                 feed -- but an end mill and a V-bit rarely want the same number."
            }
            Field::ClearPlungeFeed => {
                "Plunge feed for the clearing pass (mm/min). 0 uses the carve's own."
            }
            Field::ClearLeadOverlap => {
                "How far each clearing ring keeps cutting past its start before leaving \
                 (mm), to erase the entry/exit witness mark. Adjoining adaptive cuts repeat \
                 this distance from their shared seam; 0 uses one exact junction."
            }
            Field::ClearLeadInSize => {
                "Size of the clearing pass's lead-in onto the flat area's edge -- arc \
                 radius for an arc lead, tangent length for a linear one (mm)."
            }
            Field::ClearLeadOutSize => "Size of the clearing pass's lead-out (see lead-in).",
            // As for `PlungeA`/`PlungeB`: with a clearing pass selected, `plunge_help`
            // names the quantity the chosen style takes.
            Field::ClearPlungeA | Field::ClearPlungeB => {
                "A parameter of the clearing pass's plunge style. Pick a style and this \
                 box is labelled with the quantity it takes."
            }
            Field::ClearOffset => {
                "How far the end mill stays off the carved surface (mm), leaving that skin \
                 for the V-bit -- which finishes it better, with the flank of its cone \
                 rather than the corner of a cylinder. Nothing is abandoned: the V-bit's \
                 own passes are computed from what the end mill actually swept."
            }
        }
    }
}

/// Require a deliberate hover before showing help on any control.
fn delayed_tooltip<'a, M: 'a>(
    content: impl Into<Element<'a, M>>,
    help: impl Into<Element<'a, M>>,
    position: tooltip::Position,
) -> tooltip::Tooltip<'a, M> {
    tooltip(content, help, position).delay(std::time::Duration::from_millis(600))
}

/// An inspector label wrapped in a hover tooltip carrying its help text — the shared
/// way every parameter (field, picker, checkbox) explains itself to new users. Keeps
/// the fixed 135-px label column so rows stay aligned.
/// Wrap any element in a hover tooltip carrying `help` (appearing to its left, toward
/// the viewport), when `show`. The shared primitive behind every inspector tooltip.
fn help_wrap<'a>(
    content: impl Into<Element<'a, Message>>,
    help: &'static str,
    show: bool,
) -> Element<'a, Message> {
    let content = content.into();
    if !show {
        return content;
    }
    delayed_tooltip(
        content,
        illustrated_help::content(help),
        tooltip::Position::Left,
    )
    .into()
}

/// A flexible inspector label with hover help. The value column retains its width
/// while labels wrap to fit the available dock width.
fn label_help<'a>(
    label: impl iced::widget::text::IntoFragment<'a>,
    help: &'static str,
    show: bool,
) -> Element<'a, Message> {
    help_wrap(
        text(label)
            .width(Length::Fill)
            .size(12)
            .color(INSPECTOR_TEXT),
        help,
        show,
    )
}

/// Hover-help text for the inspector's non-numeric controls (pickers, checkboxes).
/// Numeric fields carry their own via [`Field::help`].
mod help {
    pub const SIDE: &str =
        "Which side of the chain the tool runs. Outside retains the enclosed part. Inside finishes the wall and can leave an uncut core: clear the pocket separately. On centers the tool on the line. Open left/right sides follow the selected chain direction.";
    pub const LEAD_IN: &str =
        "None starts on the contour. Linear approaches along a tangent line; Arc approaches \
         along a tangent arc. Adaptive re-entries descend from the linking height along the lead.";
    pub const LEAD_OUT: &str =
        "None ends on the contour. Linear departs along a tangent line; Arc departs \
         along a tangent arc. Adaptive departures lift to the linking height along the lead.";
    pub const PLUNGE: &str =
        "How the tool enters downward at each level. Straight drops vertically (needs a \
         centre-cutting tool or pre-drilled hole); Ramp descends along the toolpath itself, \
         arriving on the contour at full depth; Zig-zag oscillates in place, for a slot too \
         narrow to ramp along; Helix spirals down — all gentler than a straight plunge.";
    pub const CLIMB: &str = "Climb (down milling) starts with a thick chip that becomes thinner. Conventional (up milling) starts thin and becomes thicker. Profile travel follows this choice, the spindle rotation and cutter side. On-contour slots engage both sides; here the setting reverses travel. Conventional clearing uses the concentric strategy.";
    pub const CARVE_CLEAR: &str =
        "Clear the flat areas the depth cap leaves with a second, flat-bottomed tool \
         before the V-bit runs. A cone cannot leave a flat floor, so without this those \
         areas come out ridged. The end mill also takes the bulk material, sparing the \
         carving tip.";
    pub const CARVE_PLUNGE: &str =
        "How the V-bit enters downward at each ring. A V-bit always has a cutting tip, so \
         a straight drop is safe -- but a carve enters hundreds of times, and ramping \
         along the ring is kinder to the tip and to the finish. This is the V-bit's own \
         entry; the clearing tool has its own picker below.";
    pub const CARVE_CLEAR_PLUNGE: &str =
        "How the clearing tool enters downward. The flat area is entered in solid stock, \
         so a tool that is not centre-cutting -- or a deep carve in hard material -- \
         wants a ramp or helix rather than a straight drop.";
    pub const CARVE_STAY_DOWN: &str =
        "Link the carve rings without lifting where it is safe to do so, instead of \
         retracting to clearance for each. A carve can run to hundreds of rings, so this \
         saves most of the cycle time; each link is checked individually, and any that \
         would gouge still lifts.";
    pub const FACE_DIRECTION: &str =
        "The axis the facing passes sweep along. Defaults to the boundary edge you picked; \
         otherwise the longest edge.";
    pub const CHAMFER_GRADUAL: &str =
        "Spread the chamfer over its passes so each removes an equal amount of material, \
         rather than full-width passes stepping down. Only matters for a multi-pass chamfer.";
    pub const THREAD_GRADUAL: &str =
        "Size the radial infeed passes so each removes an equal amount of material, \
         instead of stepping out by an equal radius each time. A thread form is a V, so \
         the groove widens as it deepens and equal radial steps make the LAST pass the \
         heaviest — the pass that cuts the finished flank. Gradual reverses that, \
         leaving the finishing pass the lightest. Only matters with more than one pass.";
    pub const BORE: &str =
        "Whether the thread is cut into a hole (Internal) or onto a boss/stud (External). \
         Sets which side of the pitch line the tool orbits.";
    pub const HAND: &str =
        "Thread hand: Right-hand advances when turned clockwise (the common case), \
         Left-hand the opposite. Sets the helix direction.";
    pub const THREAD_CUT: &str =
        "Climb vs conventional milling for the threading orbit — climb usually finishes \
         cleaner.";
    pub const ACTIVE_MACHINE: &str =
        "Which machine you are working on. The fields below edit it, and an export is \
         checked against its travel — so this is the one thing that decides whether a \
         job can be cut here. Machines are local to this installation: opening a project \
         built elsewhere never changes it.";
    pub const POST: &str =
        "The machine controller/dialect the exported G-code is written for (grbl, Fanuc, \
         …). Pick the one your control speaks.";
    pub const TOOL_TYPE: &str =
        "The cutter geometry class. Changing it reveals the geometry fields that class \
         needs (corner radius, point angle, tip diameter, …) and how the toolpath treats \
         the tip.";
    pub const MACHINE_NAME: &str =
        "A free-text label to tell your machines apart. It does not affect the toolpath \
         or output.";
}

/// The size (length/radius) carried by a lead, or 0 for `None`.
fn lead_size(lead: Lead) -> f64 {
    match lead {
        Lead::None => 0.0,
        Lead::Linear { length } => length,
        Lead::Arc { radius } | Lead::ArcSweep { radius, .. } => radius,
    }
}

/// Plunge parameters as `(a, b)` for the inspector: `(angle, —)` for ramp,
/// `(radius, pitch)` for helix, `(angle, length)` for zig-zag, `(0, 0)` straight.
fn plunge_params(plunge: Plunge) -> (f64, f64) {
    match plunge {
        Plunge::Straight => (0.0, 0.0),
        Plunge::Ramp { angle_deg } => (angle_deg, 0.0),
        Plunge::Helix { radius, pitch } => (radius, pitch),
        Plunge::ZigZag { length, angle_deg } => (angle_deg, length),
    }
}

/// Set a lead's size (length/radius), keeping its kind.
fn set_lead_size(lead: Lead, size: f64) -> Lead {
    match lead {
        Lead::None => Lead::None,
        Lead::Linear { .. } => Lead::Linear { length: size },
        Lead::Arc { .. } => Lead::Arc { radius: size },
        Lead::ArcSweep { angle_deg, .. } => Lead::ArcSweep {
            radius: size,
            angle_deg,
        },
    }
}

/// Set a plunge's parameters from the inspector `(a, b)`, keeping its kind.
fn set_plunge_params(plunge: Plunge, a: f64, b: f64) -> Plunge {
    match plunge {
        Plunge::Straight => Plunge::Straight,
        Plunge::Ramp { .. } => Plunge::Ramp { angle_deg: a },
        Plunge::Helix { .. } => Plunge::Helix {
            radius: a,
            pitch: b,
        },
        Plunge::ZigZag { .. } => Plunge::ZigZag {
            angle_deg: a,
            length: b,
        },
    }
}

/// The inspector **label** for a plunge parameter — `first` selects the A/B slot,
/// `clearing` the carve clearing pass's copy of the pair.
///
/// These were positional — "Plunge angle/radius", "Plunge length/pitch" — one slot
/// standing for two physically distinct quantities depending on the style, and the label
/// had to say so because it could not name them. **An angle and a radius are not the same
/// kind of thing** (Andreas, 2026-08-01): a ramp is always an angle in degrees, never a
/// radius; a helix is always a radius and a pitch, both in millimetres.
///
/// A free function, and paired with [`plunge_param_help`], so the two can be checked
/// against each other headlessly — the methods on the app need a selection, which is what
/// let the help stay positional for four days after the labels stopped being so.
fn plunge_param_label(plunge: Plunge, first: bool, clearing: bool) -> Option<&'static str> {
    Some(match (plunge, first, clearing) {
        (Plunge::Ramp { .. }, true, false) => "Ramp angle (°)",
        (Plunge::Ramp { .. }, true, true) => "Clearing ramp angle (°)",
        (Plunge::Helix { .. }, true, false) => "Helix radius (mm)",
        (Plunge::Helix { .. }, false, false) => "Helix pitch (mm)",
        (Plunge::Helix { .. }, true, true) => "Clearing helix radius (mm)",
        (Plunge::Helix { .. }, false, true) => "Clearing helix pitch (mm)",
        (Plunge::ZigZag { .. }, true, false) => "Zig-zag angle (°)",
        (Plunge::ZigZag { .. }, false, false) => "Zig-zag length (mm)",
        (Plunge::ZigZag { .. }, true, true) => "Clearing zig-zag angle (°)",
        (Plunge::ZigZag { .. }, false, true) => "Clearing zig-zag length (mm)",
        // `Straight` takes no parameters, and `Ramp`'s second slot is unused: neither has
        // a box to label. Pinned by `every_labelled_plunge_parameter_explains_itself`.
        _ => return None,
    })
}

/// The inspector **tooltip** for a plunge parameter, describing the quantity that style
/// actually takes. Pairs with [`plunge_param_label`], whose `None` cases it mirrors.
///
/// **Deliberately says nothing about which tool.** A helix radius is a helix radius
/// whether the V-bit or a carve's clearing end mill is descending on it; the label already
/// carries the "Clearing" prefix, and the style *picker's* tooltip is where the
/// tool-specific warning belongs (an end mill may not be centre-cutting, a V-bit always
/// is). Hence no `clearing` parameter here — the physics does not have one.
fn plunge_param_help(plunge: Plunge, first: bool) -> Option<&'static str> {
    Some(match (plunge, first) {
        (Plunge::Ramp { .. }, true) => {
            "Angle from horizontal the tool descends at while travelling ALONG the \
             toolpath, arriving at full depth where the pass begins. Shallower is gentler \
             and travels further -- 0.5° needs 115 mm of travel per millimetre of descent \
             -- so on a small loop the ramp wraps around it repeatedly. Past 32 laps it \
             steepens to fit rather than growing without bound."
        }
        (Plunge::Helix { .. }, true) => {
            "Radius of the helix the tool spirals down on, measured from the entry point. \
             The circle it travels is twice this across, plus the tool's own diameter, and \
             all of it has to fit inside the material being entered."
        }
        (Plunge::Helix { .. }, false) => {
            "How far the helix descends per full turn (mm). Together with the radius this \
             already fixes the helix angle, which is why there is no separate angle \
             control -- a third knob would overconstrain it."
        }
        (Plunge::ZigZag { .. }, true) => {
            "Angle from horizontal for the back-and-forth ramp. Use this style only where \
             there is no room to ramp along the path -- a slot no wider than the tool -- \
             since the oscillation re-cuts the same ground on every stroke."
        }
        (Plunge::ZigZag { .. }, false) => {
            "How far the oscillation reaches along the toolpath before reversing (mm). The \
             number of out-and-back strokes is chosen so each stays within this reach and \
             the angle still holds, so a shorter reach means MORE strokes, not a steeper \
             descent."
        }
        _ => return None,
    })
}

/// A library tool as offered in the wizard's tool picker (carries its library index
/// so a pick embeds the right entry).
#[derive(Clone, Copy, PartialEq)]
struct ToolChoice {
    index: usize,
    number: u32,
    diameter: f64,
    kind: ToolKind,
}

impl std::fmt::Display for ToolChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "T{} ⌀{} {}",
            self.number,
            fmt_num(self.diameter),
            self.kind
        )
    }
}

#[derive(Clone, PartialEq)]
struct PatternDraft {
    setup_id: u64,
    pattern_id: Option<u64>,
    name: String,
    members: Vec<u32>,
    circular: bool,
    order: cam_model::PatternOrder,
    clockwise: bool,
    options: cam_model::PatternOptions,
    picking: Option<usize>,
    highlighted: Option<usize>,
    values: [String; 4],
}

struct App {
    controller: AppController,
    pending_discard: Option<DiscardAction>,
    discarding_confirmed: bool,
    mcp: Option<mcp::DesktopMcp>,
    mcp_feedback: Option<String>,
    mcp_task: iced::Task<Message>,
    viewport_cache: std::cell::RefCell<Option<(workspace_view::ViewportKey, Viewport)>>,
    profile_contour_cache: std::cell::RefCell<profile_sides::ProfileContourCache>,
    cad_scene_cache: std::cell::RefCell<Option<(workspace_view::CadSceneKey, Arc<CadScene>)>>,
    notice: Option<Notice>,
    notice_queue: VecDeque<Notice>,
    notice_filters: [bool; 3],
    status_messages: StatusMessages,
    /// Shared transactional editor for setup, stock and postprocess settings. The
    /// draft survives tab changes; only the footer can commit or discard it.
    setup_draft: Option<cam_model::Setup>,
    move_draft: Option<move_view::MoveDraft>,
    pattern_draft: Option<PatternDraft>,
    pattern_tab: pattern_editor_view::PatternTab,
    setup_model_selection: Option<InstanceId>,
    setup_fixture_selection: Option<InstanceId>,
    setup_fixture_pick: bool,
    setup_model_pick: bool,
    setup_values: BTreeMap<usize, String>,
    operation_draft: Option<Operation>,
    stock_to_leave_draft: Option<cam_model::StockToLeave>,
    stock_to_leave_initialized: bool,
    rotary_pick: Option<rotary_view::Pick>,
    rotary_wrap_draft: Option<cam_model::RotaryWrap>,
    rotary_index_draft: Option<cam_model::RotaryIndex>,
    rotary_index_mode: rotary_view::IndexMode,
    rotary_index_values: [String; 4],
    creation_edited_fields: BTreeSet<Field>,
    thread_parameters: thread_parameters::ThreadParameters,
    creation_geometry_height_defaults: Option<cam_model::OperationHeightSettings>,
    operation_geometry: Option<Vec<EntityRef>>,
    geometry_row_selection: Option<usize>,
    geometry_additive: bool,
    geometry_auto_pick: bool,
    geometry_editor: Option<operation_geometry::GeometryEditor>,
    profile_tab_picking: bool,
    operation_position_pick: Option<operation_positions::PositionKind>,
    operation_position_markers: [Option<[f64; 3]>; 2],
    profile_tab_selection: Option<usize>,
    profile_contour_selection: usize,
    profile_side_overrides: Vec<cam_model::ProfileContour>,
    profile_entry_editor: Option<profile_entries_view::EntryEditor>,
    operation_binding_preview: Option<cam_model::GeometryBinding>,
    drawing_editing_op: Option<u32>,
    operation_tab: OperationTab,
    collapsed_operation_sections: BTreeSet<operation_sections::Section>,
    focused_height: Option<String>,
    parameter_editor: parameter_expressions::Editor,
    parameter_menu: Option<(parameter_defaults::Target, iced::Point)>,
    height_draft_id: Option<u32>,
    height_draft: Option<cam_model::OperationHeightSettings>,
    height_disabled_feed_reference: Option<cam_model::HeightReference>,
    height_offsets: BTreeMap<cam_model::HeightPlaneKind, String>,
    height_reference_pick: Option<cam_model::HeightPlaneKind>,
    collapsed_heights: BTreeSet<cam_model::HeightPlaneKind>,
    workspace: Workspace,
    machining_tab: RibbonTab,
    show_project_browser: bool,
    planar_selection: Vec<LoopRef>,
    tree_drag: Option<TreeDrag>,
    operation_rename: Option<(u32, String)>,
    tree_hover: Option<(TreeNode, bool)>,
    tree_menu: Option<TreeNode>,
    pattern_menu_y: Option<f32>,
    /// Tools selected by an operation draft but not yet published to the setup.
    operation_tools: Vec<cam_model::Tool>,
    setup_tab: SetupEditorTab,
    /// An armed edge-reference field. Viewport picks are routed here until cleared.
    setup_axis_pick: Option<usize>,
    setup_orientation: Option<SetupOrientation>,
    setup_origin_mode: Option<SetupOriginMode>,
    expanded_setups: BTreeSet<u64>,
    collapsed_assets: BTreeSet<u64>,
    expanded_operations: BTreeSet<u32>,
    expanded_patterns: BTreeSet<u64>,
    tools_expanded: bool,
    hidden_assets: BTreeSet<u64>,
    /// None scopes display and picking to the active setup.
    model_view: Option<TreeNode>,
    browser_view: TreeNode,
    models_collapsed: bool,
    setups_collapsed: bool,
    hidden_setups: BTreeSet<u64>,
    hidden_ops: BTreeSet<u32>,
    panes: pane_grid::State<Pane>,
    /// Explicit editor focus, separate from the controller's fallback setup selection.
    inspector_active: bool,
    inspector_enabled: bool,
    /// Current window size, tracked so pane resizes can be clamped to per-pane
    /// pixel minimums (pane_grid works in ratios, not pixels).
    window: iced::Size,
    /// Fixed pixel sizes for the non-Viewport panes. Held constant across window
    /// resizes (only the Viewport absorbs the change); updated when the user drags a
    /// divider. Project / Inspector are widths, Output is a height.
    project_px: f32,
    inspector_px: f32,
    output_px: f32,
    main_window: Option<iced::window::Id>,
    preferences: Option<preferences::Preferences>,
    /// Every machine this installation knows about. The **active** one is mirrored into
    /// the controller (which gates exports against it); this is the set to pick from.
    machines: crate::MachineLibrary,
    /// The active machine's name — the key the selection is remembered by.
    active_machine: String,
    /// Uncommitted machine edits, awaiting **Apply** — the same promise the numeric
    /// fields make. A rename that took effect per keystroke had no way to be abandoned,
    /// wrote both config files on every letter, and gave the operator no signal that
    /// anything was pending (Andreas, 2026-08-01). Only the *Active* picker stays
    /// instant: switching machines is a navigation, not an edit.
    machine_name_edit: Option<String>,
    machine_post_edit: Option<PostKind>,
    machine_estlcam_edit: Option<cam_post::EstlcamOptions>,
    /// The user's preferences, as loaded at startup and written back on change.
    ///
    /// The view/snap/pane fields below stay the live state — the settings copy is
    /// refreshed through `remember`, which is the single place that knows the mapping.
    settings: crate::Settings,
    /// The active ribbon tab.
    active_tab: RibbonTab,
    /// The index (within the active tab's groups) of the open collapse-popup, if any.
    open_group: Option<usize>,
    /// Edit buffers for the inspector fields of the current selection.
    fields: BTreeMap<Field, String>,
    /// Whether the viewport overlays the simulated stock surface.
    show_stock: bool,
    /// Whether the orientation-cube gizmo is shown (toggleable).
    show_gizmo: bool,
    /// Whether hover tooltips (inspector parameters + ribbon icons) are shown. On by
    /// default to help new users; toggled off from the View tab once they're fluent.
    tooltips: bool,
    /// Whether the licence-and-credits overlay is open (View tab -> About -> Licence).
    show_license: bool,
    /// On-screen edge length of the orientation cube, in **logical pixels** —
    /// fixed (not scaled with the window), adjustable at runtime via the View tab.
    gizmo_size: f32,
    /// The orbit-camera orientation, owned here; clicking a cube face or dragging
    /// the viewport reports changes back as messages.
    view: ViewControls,
    /// Last known cursor position over the viewport (window coords), for drawing
    /// the pickbox while a geometry pick is pending.
    cursor: Option<iced::Point>,
    /// Cross-project tool library (loaded from the config dir), picked from during
    /// op setup and managed from the Tools tab.
    library: ToolLibrary,
    libraries: crate::tool_library::LibraryCollection,
    library_name_edit: String,
    library_renaming: bool,
    library_context_menu: Option<iced::Point>,
    library_sidebar_drag: Option<(f32, f32)>,
    /// The library tool selected for editing in the Tools-tab library editor.
    lib_sel: usize,
    fusion_import: Option<Vec<crate::fusion_tools::ImportEntry>>,
    /// A live **working copy** of the selected library tool while editing in the Tools
    /// tab. Field edits apply here first (so the viewport previews them instantly without
    /// an Apply); the committed library entry is only overwritten on Apply. `None` outside
    /// Tools mode. Dirty ⇔ this differs from the committed entry.
    tool_edit: Option<cam_model::Tool>,
    tool_drafts: BTreeMap<usize, tool_presets::LibraryToolDraft>,
    library_preset: Option<u64>,
    preset_values: BTreeMap<PresetField, String>,
    edited_preset_input: Option<PresetField>,
    edited_tool_input: Option<Field>,
    selected_tool_parameter: Option<Field>,
    tool_info: Option<cam_model::Tool>,
    operation_tool_selector: Option<operation_tool_selector::ToolSelector>,
    cad_selected_project_tool: Option<cam_model::Tool>,
    preset_buffers: BTreeMap<u64, BTreeMap<PresetField, String>>,
    new_material_name: String,
    selected_material: Option<u64>,
    editing_material: Option<u64>,
    creating_material: bool,
    material_force_inputs: BTreeMap<u64, String>,
    /// Session-only what-if engagement; None identifies an unsaved creation draft.
    cutting_force_inputs: BTreeMap<(u64, Option<u32>), [String; 3]>,
    show_materials: bool,
    show_preset_editor: bool,
    preset_editor_snapshot: Option<tool_presets::LibraryToolDraft>,
    /// Which internal tab the Tool Library pane shows (Serial / Family).
    library_view: LibraryView,
    tool_table_columns: [bool; 8],
    tool_table_widths: [f32; 8],
    tool_table_order: [usize; 8],
    tool_table_drag: Option<workspace_view::ToolColumnDrag>,
    tool_table_menu: Option<(usize, bool)>,
    tool_table_menu_pos: iced::Point,
    tool_table_menu_search: String,
    tool_table_filter_draft: BTreeSet<String>,
    tool_table_search: String,
    tool_table_filters: [Option<BTreeSet<String>>; 8],
    /// The operation whose right-click context menu is open, and where to anchor it
    /// (window-absolute coords, captured from `window_cursor` at right-click time).
    open_op_menu: Option<u32>,
    /// Export only the exact context-menu job that requested a save dialog.
    pending_toolpath_export: Option<crate::JobStamp>,
    pending_simulation: Option<crate::JobStamp>,
    pending_processing_time: Option<processing_time_view::PendingProcessingTime>,
    processing_time_dialog: Option<processing_time_view::ProcessingTimeDialog>,
    simulation: Option<SimulationPlayer>,
    operation_preview: backplot_preview_view::OperationPreview,
    op_menu_pos: iced::Point,
    /// The project tool (by number) whose right-click "Add to library" menu is open,
    /// and where to anchor it. `None` unless a menu is open.
    open_tool_menu: Option<u32>,
    tool_menu_pos: iced::Point,
    /// The Library-pane right-click menu: which library index, whether it has morphed
    /// into the "set number" input (and its buffer), and where to anchor it.
    lib_menu: Option<LibMenu>,
    lib_menu_pos: iced::Point,
    /// Last known cursor position in **window-absolute** coords (from a global
    /// event subscription). Overlays are placed from the window origin, so this —
    /// not a widget-local position — is what anchors the op context menu exactly
    /// under the cursor.
    window_cursor: iced::Point,
    /// Selected operations whose toolpaths are visible. Empty hides all toolpaths.
    /// Editor focus is separate: selection alone never opens the inspector.
    focus_ops: BTreeSet<u32>,
    /// Live keyboard modifiers (from a global subscription), so an op click can
    /// tell a plain click (single focus) from ⌘/Ctrl-click (add/remove from set).
    modifiers: iced::keyboard::Modifiers,
    profile_selection_mode: ProfileSelectionMode,
    profile_coplanar: bool,
    profile_chain_edges: bool,
    cad_filter: CadFilter,
    cad_filter_explicit: bool,
    cad_hover: Option<EntityRef>,
    cad_candidates: Vec<PickCandidate>,
    cad_hidden: BTreeSet<InstanceId>,
    cad_select_through: bool,
    cad_pending_kind: Option<OpKind>,
    cad_tool_pick: Option<u32>,
    cad_inspecting: bool,
    cad_editing_op: Option<u32>,
    cad_override_top: String,
    cad_override_bottom: String,
    /// Enabled viewport object-snaps for operation picking (End/Mid/…); the order
    /// is irrelevant, membership is what counts.
    snaps: Vec<SnapKind>,
    /// The object-snap currently under the cursor while a pick is pending (drawn
    /// as a marker and used as the start when clicked). `None` when nothing snaps.
    snap_hover: Option<SnapHit>,
    setup_hover: SetupHover,
    /// The world-mm pickbox aperture from the last hover, sizing the snap marker
    /// (roughly constant on screen).
    snap_aperture: f64,
    /// The loop under the cursor while a pick is pending — highlighted so it is
    /// obvious which loop a click will select (vital for concentric circles).
    hover_loop: Option<LoopRef>,
    /// "Set origin" pick mode: a viewport click drops the workpiece datum (using
    /// object snaps for corners/centres, or a free point elsewhere).
    setting_origin: bool,
    /// Two-point origin mode: X from the 1st pick, Y from the 2nd, Z the midpoint.
    setting_origin_2pt: bool,
    /// The first point captured in two-point origin mode (awaiting the second).
    origin_first: Option<[f64; 3]>,
    /// Whether the workpiece-origin datum marker is drawn (View toggle).
    show_origin: bool,
    /// Draw the active machine's travel as a box around the job.
    show_envelope: bool,
    status: String,
    output_history: VecDeque<String>,
    output_content: iced::widget::text_editor::Content,
    output_snapshot: String,
    output_last_geometry_error: Option<String>,
    output_cleared: bool,
}

// The pickbox aperture, the object-snap catch distance derived from it, and the snap
// marker's size are all preferences now (`Settings::snapping`) — see `SnapPrefs`.
// The catch distance is deliberately not its own control: it stays a fixed multiple
// of the pickbox, because two absolute knobs would let a user set a catch distance
// smaller than the pickbox feeding it.

/// Whether an operation kind uses a start/lead-in point, and so honours object
/// snaps. Face/Drill/Thread have no start, so snaps are inert (and hidden) there.
fn op_uses_snaps(kind: OpKind) -> bool {
    matches!(
        kind,
        OpKind::Profile | OpKind::Chamfer | OpKind::Pocket | OpKind::Engrave | OpKind::Carve
    )
}

/// Whether any of `visible`'s edit buffers has diverged from the model.
///
/// Free-standing, and taking the model read as a closure, so the rule can be tested
/// without standing up a GUI — `inspector_dirty` is otherwise unreachable from a
/// headless test, and an always-on Apply button is exactly the kind of thing that
/// goes unnoticed for months.
///
/// A buffer that will not parse counts as **dirty**: the operator has typed
/// something, even if it is not yet a number. Apply stays disabled in that case
/// anyway, via `any_field_invalid`. A field with no buffer at all counts as clean —
/// nothing has been typed into it.
fn fields_are_dirty(
    visible: &[Field],
    buffers: &BTreeMap<Field, String>,
    committed: impl Fn(Field) -> Option<f64>,
) -> bool {
    visible.iter().any(|&f| {
        let Some(model) = committed(f) else {
            return false;
        };
        match buffers.get(&f) {
            // `fmt_num` round-trips an f64 exactly, so an untouched buffer parses
            // back to precisely the value it was seeded with.
            Some(text) => text.parse::<f64>().map(|v| v != model).unwrap_or(true),
            None => false,
        }
    })
}

/// Whether a field belongs to a carve's **clearing pass** rather than to the carve
/// itself. These render in their own section under the clearing tool, not mixed in with
/// the V-bit's own numbers — two tools, two blocks.
fn is_clear_field(field: Field) -> bool {
    matches!(
        field,
        Field::ClearStepdown
            | Field::ClearOverlap
            | Field::ClearOffset
            | Field::ClearEngagement
            | Field::ClearFeed
            | Field::ClearPlungeFeed
            | Field::ClearLeadOverlap
            | Field::ClearLeadInSize
            | Field::ClearLeadOutSize
            | Field::ClearPlungeA
            | Field::ClearPlungeB
    )
}

/// Orientation-cube on-screen size (logical px): the slider's range. The *default*
/// now lives in `Settings` alone — the session reads it from there, so a constant
/// here would be a second copy to keep in step.
const GIZMO_SIZE_MIN: f32 = 60.0;
const GIZMO_SIZE_MAX: f32 = 220.0;
/// Inset of the cube from the viewport's top-right corner (logical px).
const GIZMO_MARGIN: f32 = 8.0;

/// Highlight colour for a picked boundary loop (accent blue).
const PICK_BOUNDARY: [f32; 4] = [0.20, 0.55, 0.90, 1.0];
/// Highlight colour for an excluded island loop (gold).
const PICK_ISLAND: [f32; 4] = [0.90, 0.65, 0.10, 1.0];
/// Highlight colour for the loop under the cursor during a pick (bright yellow) —
/// shows which loop a click will select, e.g. among concentric circles.
const PICK_HOVER: [f32; 4] = [0.95, 0.92, 0.25, 1.0];
/// Object-snap marker colour (bright cyan — reads over part and stock alike).
const SNAP_MARK: [f32; 4] = [0.20, 0.95, 0.95, 1.0];
/// Workpiece-origin datum marker colour (magenta — distinct from the snaps/loops
/// and legible under red-green colour deficiency).
const ORIGIN_MARK: [f32; 4] = [0.95, 0.30, 0.85, 1.0];

/// Draw the workpiece-origin datum: a ringed crosshair at `origin` (part XY at
/// `z`), sized `r` (mm). Always shown, so the datum the G-code is referenced to
/// is visible in the viewport.
fn add_origin_marker(scene: &mut Scene, origin: [f64; 3], z: f32, r: f32) {
    let (cx, cy) = (origin[0] as f32, origin[1] as f32);
    // Cross.
    scene.add_strip(vec![[cx - r, cy, z], [cx + r, cy, z]], ORIGIN_MARK);
    scene.add_strip(vec![[cx, cy - r, z], [cx, cy + r, z]], ORIGIN_MARK);
    // Ring (octagon at 0.6·r).
    let rr = r * 0.6;
    let ring: Vec<[f32; 3]> = (0..=8)
        .map(|i| {
            let a = i as f32 / 8.0 * std::f32::consts::TAU;
            [cx + rr * a.cos(), cy + rr * a.sin(), z]
        })
        .collect();
    scene.add_strip(ring, ORIGIN_MARK);
}

/// Draw the object-snap marker at `hit.point` as a glyph specific to the snap
/// kind (AutoCAD idiom: square = End, triangle = Mid, diamond = Quadrant,
/// hourglass = Nearest), sized by the pickbox `aperture` so it reads at any zoom.
fn add_snap_marker(scene: &mut Scene, hit: SnapHit, aperture: f64, scale: f32, z: f32) {
    let (cx, cy) = (hit.point[0] as f32, hit.point[1] as f32);
    // A touch larger than the (already doubled) snap aperture, so the engaged
    // marker reads clearly in place of the pickbox.
    let h = (aperture as f32 * scale).max(0.01);
    let p = |dx: f32, dy: f32| [cx + dx * h, cy + dy * h, z];
    let strip: Vec<[f32; 3]> = match hit.kind {
        // Square.
        SnapKind::End => vec![
            p(-1.0, -1.0),
            p(1.0, -1.0),
            p(1.0, 1.0),
            p(-1.0, 1.0),
            p(-1.0, -1.0),
        ],
        // Upward triangle.
        SnapKind::Mid => vec![p(-1.0, -0.8), p(1.0, -0.8), p(0.0, 1.0), p(-1.0, -0.8)],
        // Diamond.
        SnapKind::Quadrant => vec![
            p(0.0, -1.2),
            p(1.2, 0.0),
            p(0.0, 1.2),
            p(-1.2, 0.0),
            p(0.0, -1.2),
        ],
        // Hourglass (two triangles sharing the centre) drawn as one open path.
        SnapKind::Nearest => vec![
            p(-1.0, 1.0),
            p(1.0, 1.0),
            p(-1.0, -1.0),
            p(1.0, -1.0),
            p(-1.0, 1.0),
        ],
    };
    scene.add_strip(strip, SNAP_MARK);
}

/// Highlight a picked path. `closed` closes it back to its start; an **open**
/// imported stroke must not be closed, or the highlight would show a segment the
/// toolpath will never cut.
fn add_path_highlight(scene: &mut Scene, pts: &[cam_geo::Point], closed: bool, color: [f32; 4]) {
    let mut strip: Vec<[f32; 3]> = pts.iter().map(|p| [p.x as f32, p.y as f32, 0.0]).collect();
    if closed {
        if let Some(&first) = strip.first() {
            strip.push(first);
        }
    }
    scene.add_highlight(strip, color);
}

/// Colour of the drilling annotations — a light blue, distinct from the cyan snap
/// and magenta origin markers and legible under red-green colour deficiency.
const DRILL_MARK: [f32; 4] = [0.40, 0.72, 1.0, 1.0];
/// Radius of a peck ring / half-length of a dwell bar, in **screen pixels** (so the
/// mark never scales with the model — it is sized against `world_per_pixel` at draw).
const DRILL_MARK_PX: f32 = 4.4;
/// Facets in a peck ring (a billboarded circle drawn as a line loop).
const DRILL_RING_SEGMENTS: usize = 16;
/// Safety cap on rings per hole, so a pathologically tiny peck can't flood the buffer.
const DRILL_MAX_RINGS: u32 = 200;

/// A drilling annotation anchored in world space: a **ring** at a peck retract
/// depth, or a horizontal **bar** at the hole bottom when it dwells. The pixel
/// sizing/billboarding happens at draw time (see [`Viewport::draw`]).
#[derive(Clone, Copy, Debug, PartialEq)]
struct DrillMark {
    at: [f32; 3],
    kind: DrillMarkKind,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum DrillMarkKind {
    /// A peck retract level (locates the pecking).
    PeckRing,
    /// The exact bottom where a dwelling drill stops.
    DwellBar,
}

/// Collect the drilling annotation anchors from a backplot program: a ring at each
/// **intermediate** peck retract (strictly between the hole top and bottom — the
/// bottom is the plunge-line end, not a peck), and a bar at the bottom of every
/// hole that dwells. Pure; the screen sizing is applied later, per frame.
fn drill_marks_of(program: &Program) -> Vec<DrillMark> {
    let mut marks = Vec::new();
    for step in program.steps() {
        let Step::Drill(c) = step else { continue };
        for &[x, y] in &c.points {
            let (x, y) = (x as f32, y as f32);
            if let Some(peck) = c.peck.filter(|p| *p > 0.0) {
                let mut k = 1u32;
                loop {
                    let z = c.z_top - peck * k as f64;
                    if z <= c.depth + 1e-6 || k > DRILL_MAX_RINGS {
                        break; // reached (or passed) the bottom — that's not a peck
                    }
                    marks.push(DrillMark {
                        at: [x, y, z as f32],
                        kind: DrillMarkKind::PeckRing,
                    });
                    k += 1;
                }
            }
            if c.dwell.is_some() {
                marks.push(DrillMark {
                    at: [x, y, c.depth as f32],
                    kind: DrillMarkKind::DwellBar,
                });
            }
        }
    }
    marks
}

/// Append `mark`, sized to `px_world` (world mm for the desired pixel size) and
/// billboarded into the camera's `right`/`up` plane, to a `LineList` vertex buffer.
fn push_drill_mark(
    out: &mut Vec<Vertex>,
    mark: DrillMark,
    px_world: f32,
    right: [f32; 3],
    up: [f32; 3],
) {
    let at = mark.at;
    let point = |du: f32, dv: f32| Vertex {
        position: [
            at[0] + du * right[0] + dv * up[0],
            at[1] + du * right[1] + dv * up[1],
            at[2] + du * right[2] + dv * up[2],
        ],
        color: DRILL_MARK,
        width: 1.0,
    };
    match mark.kind {
        DrillMarkKind::PeckRing => {
            let r = px_world;
            let n = DRILL_RING_SEGMENTS;
            for i in 0..n {
                let a0 = i as f32 / n as f32 * std::f32::consts::TAU;
                let a1 = (i + 1) as f32 / n as f32 * std::f32::consts::TAU;
                out.push(point(r * a0.cos(), r * a0.sin()));
                out.push(point(r * a1.cos(), r * a1.sin()));
            }
        }
        DrillMarkKind::DwellBar => {
            // A little wider than a ring so the exact stop reads as a crisp line.
            let h = px_world * 1.4;
            out.push(point(-h, 0.0));
            out.push(point(h, 0.0));
        }
    }
}

/// The `(yaw, pitch)` that views a cube face (given by its outward normal)
/// straight on, **the right way up** — the orientation a click on that gizmo face
/// snaps to. Side and front/back views use `pitch = −90°` so world +Z is screen
/// up (not −Z, which reads upside down).
fn face_view(normal: [f32; 3]) -> (f32, f32) {
    use std::f32::consts::{FRAC_PI_2, PI};
    if normal[2] > 0.5 {
        (0.0, 0.0) // +Z top → +Y up
    } else if normal[2] < -0.5 {
        (0.0, PI) // −Z bottom
    } else if normal[1] > 0.5 {
        (PI, -FRAC_PI_2) // +Y back → +Z up
    } else if normal[1] < -0.5 {
        (0.0, -FRAC_PI_2) // −Y front → +Z up
    } else if normal[0] > 0.5 {
        (-FRAC_PI_2, -FRAC_PI_2) // +X right → +Z up
    } else {
        (FRAC_PI_2, -FRAC_PI_2) // −X left → +Z up
    }
}

#[derive(Clone, Copy, Debug)]
enum BoreOption {
    Internal(bool),
    Climb(bool),
    Angle(bool),
    Repeat(bool),
    Center(bool),
    Reverse(bool),
    Order(cam_model::BoreOrder),
}

#[derive(Debug, Clone)]
enum Message {
    MoveOpen(TreeNode),
    MoveValue(usize, String),
    MoveHandleStart(usize, [f64; 9], [f64; 3]),
    MoveHandleDelta(f64),
    MoveHandleInput(String),
    MoveCancel,
    MoveApply,
    Rotary(rotary_view::Action),
    PollMcp,
    TraverseFocus(bool),
    OutputAction(iced::widget::text_editor::Action),
    OutputCopy,
    OutputClear,
    OpenSample,
    CancelToolDraft,
    RunJob,
    OperationTabChanged(OperationTab),
    OperationSectionToggled(operation_sections::Section),
    ProfileCompChanged(cam_model::Comp),
    ProfilePassToggle(profile_parameters::Toggle, bool),
    PocketFinishToggle(pocket_parameters::Toggle, bool),
    PocketDepthToggle(pocket_parameters::DepthToggle, bool),
    PocketAdaptiveToggled(bool),
    ProfileTabShape(cam_model::TabShape),
    ProfileCorner(cam_model::ProfileCorner),
    ProfileStockBoundary(profile_parameters::StockChoice),
    ProfileTabsEnabled(bool),
    ProfileTabPositioning(profile_tabs::Positioning),
    ProfileTabPickToggle,
    ProfileTabPicked([f64; 2]),
    ProfileTabRemove(usize),
    ProfileTabSelected(usize),
    ProfileTabsClear,
    EngraveClosedChanged(bool),
    StockToLeaveToggled(bool),
    OperationPositionPick(operation_positions::PositionKind),
    OperationPositionClear(operation_positions::PositionKind),
    OperationPositionPicked([f64; 3]),
    OperationToolPicked(usize),
    OpenOperationToolSelector,
    CloseOperationToolSelector,
    OperationProjectToolPicked(cam_model::Tool),
    EditDrawingGeometry(u32),
    UseDrawingGeometry,
    CancelDrawingGeometry,
    Tree(TreeMessage),
    ShowStatus(StatusTarget),
    #[allow(dead_code)] // Retained legacy view command; M1a reorganizes its UI.
    Select(Selection),
    SelectSetup(u64),
    EditSetup(u64),
    SetupTabChanged(SetupEditorTab),
    SetupNameChanged(String),
    SetupProgramChanged(String),
    SetupOrientationChanged(SetupOrientation),
    SetupOriginModeChanged(SetupOriginMode),
    SetupUseModelFrame(u64),
    SetupUseModelOrigin(u64),
    SetupOriginChanged(usize, String),
    SetupWorkOffsetChanged(String),
    SetupToolChangeAutomatic(bool),
    SetupToolChangeHeightChanged(String),
    SetupModelPick,
    SetupFixturePick,
    SetupGeometryClear(bool),
    SetupFixtureToggled(InstanceId, bool),
    SetupFixtureClearanceChanged(usize, String),
    SetupModelToggled(u64, bool),
    SetupStockKindChanged(StockEditorKind),
    SetupStockNumberChanged(usize, String),
    SetupStockSolidChanged(StockSolidChoice),
    SetupStockAxisClear,
    SetupStockOriginAxis(usize),
    SetupStockPositionChanged(usize, BoxAnchor),
    SetupAxisPick(usize),
    BeginPattern(bool),
    PatternTabChanged(pattern_editor_view::PatternTab),
    EditPattern(u64),
    PatternName(String),
    PatternValue(usize, String),
    PatternMember(u32, bool),
    PatternOrderChanged(cam_model::PatternOrder),
    PatternTypeChanged(bool),
    PatternSpacingChanged(cam_model::PatternSpacing),
    PatternSecondDirection(bool),
    PatternKeepOriginal(bool),
    PatternReverse(bool),
    PatternAxisReversed(usize, bool),
    PatternSymmetric(usize, bool),
    PatternPick(usize),
    PatternHighlight(usize),
    PatternClear(usize),
    PatternDirectionChanged(bool),
    TogglePatternExpanded(u64),
    PatternSuppression(u64, bool),
    PatternDissolve(u64),
    PatternDelete(u64),
    PatternCommit,
    PatternCancel,
    SetupAxisFlip(usize, bool),
    SetupAxisToggle(usize),
    SetupAxisClear(usize),
    SetupCommit,
    SetupCancel,
    CancelOperationDraft,
    SetupNew,
    SetupDuplicate(u64),
    SetupDelete(u64),
    ToggleSetupExpanded(u64),
    ToggleAssetExpanded(u64),
    ToggleOperationExpanded(u32),
    ToggleToolsExpanded,
    SetAssetVisible(u64, bool),
    SetSetupVisible(u64, bool),
    SetOperationVisible(u32, bool),
    FieldChanged(Field, String),
    ParameterInputState(Field, bool, usize),
    OpenParameterMenu(Field),
    OpenParameterOptionMenu(parameter_options::OptionMenu),
    CloseParameterMenu,
    ParameterMenuAction(parameter_defaults::Action),
    ParameterComplete(Field, &'static str, usize),
    ParameterSuggestionMove(Field, i32),
    HeightReferenceChanged(cam_model::HeightPlaneKind, cam_model::HeightReference),
    HeightOffsetChanged(cam_model::HeightPlaneKind, String),
    HeightSourceToggled(cam_model::HeightPlaneKind, cam_model::HeightReference, bool),
    HeightSectionToggle(cam_model::HeightPlaneKind),
    HeightFeedEnabled(bool),
    HeightPickToggle(cam_model::HeightPlaneKind),
    HeightSelectionClear(cam_model::HeightPlaneKind),
    HeightEntityRemove(cam_model::HeightPlaneKind, usize),
    HeightDefaults,
    CheckHeightFocus,
    CheckOperationRenameFocus,
    OperationRenameFocus(u32, bool),
    HeightFocus(String, bool),
    /// Rename the machine.
    MachineNameChanged(String),
    /// Choose the post/controller dialect for export.
    PostKindChanged(PostKind),
    EstlcamOutputSelectionChanged(u8),
    /// Commit the inspector fields (one undo step) and recompute the toolpath.
    Apply,
    Undo,
    Redo,
    // --- File I/O (native dialogs via rfd) ---
    /// Start a fresh, empty project.
    NewProject,
    CloseRequested(iced::window::Id),
    DiscardConfirmed,
    DiscardCancelled,
    /// Prompt for and open a `.ocam` project.
    OpenProject,
    /// The chosen project to open (`None` = cancelled).
    ProjectToOpen(Option<PathBuf>),
    /// Save to the current path, or prompt if there is none.
    SaveProject,
    /// Always prompt for a save location.
    SaveProjectAs,
    /// The chosen path to save to (`None` = cancelled).
    ProjectToSave(Option<PathBuf>),
    /// Prompt for a `.ocam` to **export** the working tool library to.
    ExportLibrary,
    /// The chosen path to write the library to (`None` = cancelled).
    LibraryToExport(Option<PathBuf>),
    /// Prompt for a `.ocam` or Fusion tool library to import.
    ImportLibrary,
    SelectToolLibrary(String),
    NewToolLibrary,
    LibraryNameChanged(String),
    RenameToolLibrary,
    BeginLibraryRename,
    CancelLibraryRename,
    OpenLibraryContextMenu(String),
    CloseLibraryContextMenu,
    ToggleLibrarySidebar,
    LibrarySidebarResizeStart,
    DeleteToolLibrary,
    DeleteToolLibraryConfirmed((String, bool)),
    /// The chosen library file to import (`None` = cancelled).
    LibraryToImport(Option<PathBuf>),
    FusionImportToggle(usize, bool),
    FusionImportSelectAll(bool),
    FusionImportConfirm,
    FusionImportCancel,
    /// Prompt for and import a `.dxf`/`.dwg` file.
    ImportCad,
    PollCadImport,
    PollRun,
    CancelRun,
    CancelOperationPreview,
    CancelCadImport,
    NoticeDismissed,
    NoticeBackdrop,
    NoticeFilterChanged(usize, bool),
    ReplaceCadAsset(u64),
    CadReplacementChosen(u64, Option<PathBuf>),
    RemoveCadAsset(u64),
    CadFilterChanged(CadFilter),
    GeometryFilterChanged(CadFilter),
    CadHover(Option<EntityRef>),
    CadPicked(Vec<PickCandidate>),
    ProfileSelectionModeChanged(ProfileSelectionMode),
    ProfileCoplanarChanged(bool),
    ProfileChainEdgesChanged(bool),
    GeometryPick,
    OperationGeometry(GeometryMessage),
    GeometryRowSelected(usize),
    GeometryRemove(usize),
    GeometryClear,
    CadSelect(EntityRef),
    CadIsolateInstance(InstanceId),
    CadShowAll,
    SetInstanceVisible(InstanceId, bool),
    CadEscape,
    ConfirmCadOperation,
    EditCadGeometry(u32),
    #[allow(dead_code)] // Legacy absolute-height compatibility command.
    CadOverrideTopChanged(String),
    #[allow(dead_code)] // Legacy absolute-height compatibility command.
    CadOverrideBottomChanged(String),
    /// The chosen CAD file to import (`None` = cancelled).
    CadToImport(Option<PathBuf>),
    /// Prompt for a `.nc` export location.
    ExportNc,
    /// The operator declined to export exact duplicate operations.
    ExportDuplicatesDeclined,
    /// The chosen `.nc` path (`None` = cancelled).
    NcToExport(String, Option<PathBuf>),
    // --- Operation-creation wizard ---
    /// Begin creating an operation of `kind` (enter geometry-pick mode).
    BeginOp(OpKind),
    /// Cancel the pending operation.
    CancelOp,
    /// Confirm a pending operation (finalise a pocket boundary + islands).
    ConfirmOp,
    /// Change the selected profile's cut side.
    SideChanged(Side),
    ProfileContourSideCycle(usize),
    ProfileEntry(profile_entries_view::EntryMessage),
    /// Change the selected thread between internal (bore) and external (boss).
    ThreadInternalChanged(bool),
    BoreOption(BoreOption),
    /// Change the selected thread's hand.
    ThreadHandChanged(Hand),
    ThreadParameters(thread_parameters::ThreadMessage),
    /// Change the selected thread between climb and conventional milling.
    ThreadClimbChanged(bool),
    /// Toggle whether the selected carve links its rings without lifting.
    CarveStayDownToggled(bool),
    /// Turn the selected carve's flat-area clearing pass on or off.
    CarveClearToggled(bool),
    /// Choose the library tool (by index) that clears the selected carve's flat areas.
    CarveClearToolChanged(usize),
    /// Change how the selected carve's clearing tool enters in Z.
    CarveClearPlungeChanged(PlungeKind),
    /// Climb vs conventional for the selected carve's clearing pass.
    CarveClearClimbToggled(bool),
    /// Wall lead-in kind for the selected carve's clearing pass.
    CarveClearLeadInChanged(LeadKind),
    /// Wall lead-out kind for the selected carve's clearing pass.
    CarveClearLeadOutChanged(LeadKind),
    /// A world `(x, y)` picked in the viewport plus the pickbox aperture in world
    /// mm (completes a pending operation).
    PickWorld([f32; 2], f32),
    /// While a pick is pending: the cursor's screen point (for the pickbox), its
    /// world `(x, y)`, and the pickbox aperture (world mm) — resolves and previews
    /// the object-snap under the cursor.
    HoverWorld(iced::Point, [f32; 2], f32),
    /// The cursor moved over the viewport (window coords) while a pick is pending,
    /// but no world point was available (draws the pickbox, clears any snap).
    ViewportCursor(iced::Point),
    /// Toggle a viewport object-snap on/off during operation picking.
    ToggleSnap(SnapKind),
    ToggleSnapping,
    ToggleFeatureHighlights,
    SetupHover(iced::Point, SetupHover),
    SetupStockEdgePicked([f64; 3]),
    /// Focus a workpiece origin — makes it active + selects it.
    #[allow(dead_code)] // Retained legacy view command; M1a reorganizes its UI.
    SelectOrigin(u32),
    /// Add a new workpiece origin (a reorientation group).
    #[allow(dead_code)] // Retained legacy view command; M1a reorganizes its UI.
    AddOrigin,
    /// Delete the workpiece origin with the given index (extras only).
    #[allow(dead_code)] // Retained legacy view command; M1a reorganizes its UI.
    DeleteOrigin(u32),
    /// Freeze/unfreeze an origin's operations (drop them from the run/viewport).
    #[allow(dead_code)] // Retained legacy view command; M1a reorganizes its UI.
    ToggleOriginDisabled(u32, bool),
    /// Enter/leave single-point "set workpiece origin" pick mode (Edit tab).
    ToggleSetOrigin,
    /// Enter/leave two-point origin pick mode: X from the 1st pick, Y from the
    /// 2nd, Z the midpoint of both.
    ToggleSetOrigin2pt,
    /// Show or hide the workpiece-origin datum marker (View tab).
    #[allow(dead_code)] // Retained legacy view command; M1a reorganizes its UI.
    ToggleShowOrigin,
    /// Show / hide the machine-travel envelope around the job.
    #[allow(dead_code)] // Retained legacy view command; M1a reorganizes its UI.
    ToggleEnvelope,
    /// A viewport click while setting the origin (either mode): world `(x,y)` +
    /// aperture, resolved to a snapped or free point.
    OriginPointPicked([f64; 3]),
    /// Structural edits to the selected operation.
    DuplicateOp,
    DeleteOp,
    /// Restart the creation wizard for the context-menu operation, replacing it in
    /// place (same kind, same position in the order).
    ReinitOp,
    /// Include (`false`) or exclude (`true`) an operation from toolpath output.
    SetOpExcluded(u32, bool),
    #[allow(dead_code)] // Retained legacy view command; M1a reorganizes its UI.
    ToggleStock,
    ToggleTransparentStock,
    /// Relative camera changes reported by dragging in the viewport.
    OrbitBy(f32, f32),
    PanBy([f32; 3]),
    ZoomBy(f32),
    /// Snap to a `(yaw, pitch)` orientation (a clicked cube face), re-framing.
    SetView(f32, f32),
    /// Reset to the framed top view.
    ResetView,
    /// Show or hide the orientation cube.
    #[allow(dead_code)] // Retained legacy view command; M1a reorganizes its UI.
    ToggleGizmo,
    /// Show or hide hover tooltips (inspector + ribbon icons).
    ToggleTooltips,
    /// Open the licence-and-credits overlay.
    ShowLicense,
    /// Dismiss the licence-and-credits overlay.
    CloseLicense,
    /// Set the orientation cube's on-screen size (logical px).
    SetGizmoSize(f32),
    /// A continuous control has settled (slider released) — write preferences now
    /// rather than on every intermediate value.
    SettingsSettled,
    /// Make this machine the active one — and adopt its control with it.
    ActiveMachineChanged(String),
    /// Add a machine to the library: a copy of the active one, ready to edit.
    NewMachine,
    /// Remove the active machine (never the last).
    DeleteMachine,
    ShowPrefs,
    Preferences(preferences::Edit),
    ShortcutKey(Option<char>, iced::keyboard::Modifiers),
    SetSimulationAccuracy(f32),
    PaneResized(pane_grid::ResizeEvent),
    PaneDragged(pane_grid::DragEvent),
    /// The window was resized (tracked for pixel-accurate pane minimums).
    WindowResized(iced::Size),
    /// Change the selected tool's geometry kind (committed immediately).
    ToolKindChanged(ToolKind),
    /// Change the selected tool's cutting direction (down-cut / up-cut).
    ToolCuttingDirChanged(CutDir),
    /// Toggle a thread mill between single-point (one tooth) and full-form (teeth at a
    /// fixed pitch).
    ThreadFormChanged(ThreadForm),
    /// Change the selected profile's lead-in / lead-out / plunge kind (committed
    /// immediately with default parameters; sizes are then edited as fields).
    LeadInKindChanged(LeadKind),
    LeadOutKindChanged(LeadKind),
    ChamferExitEnabled(bool),
    ChamferEntryEnabled(bool),
    ChamferSameLeadOut(bool),
    PlungeKindChanged(PlungeKind),
    /// Change the selected face op's pass direction (committed immediately).
    FaceDirectionChanged(Axis),
    /// Toggle climb (true) vs conventional (false) clearing on the selected
    /// pocket / profile-roughing op.
    ClearingClimbToggled(bool),
    ParallelBothWaysToggled(bool),
    ParallelScallopToggled(bool),
    ParallelTransitionChanged(cam_model::ParallelTransition),
    ScallopSmoothingToggled(bool),
    ScallopRetractionChanged(cam_model::ScallopRetraction),
    ScallopOrderChanged(cam_model::ScallopOrder),
    SpiralModeChanged(cam_model::SpiralMode),
    SpiralOrderChanged(cam_model::SpiralOrder),
    SpiralMillingChanged(cam_model::SpiralMilling),
    SpiralCutDirectionChanged(cam_model::SpiralCutDirection),
    SpiralBoundaryChanged(cam_model::SpiralBoundary),
    SpiralContactOnlyToggled(bool),
    SpiralDepthsToggled(bool),
    SpiralDepthOrderToggled(bool),
    SpiralLinkInsideOutToggled(bool),
    SpiralClockwiseToggled(bool),
    PocketKeepToolDownToggled(bool),
    AdaptiveClimbToggled(bool),
    AdaptiveContainmentChanged(cam_model::AdaptiveContainment),
    /// Toggle the selected chamfer's gradual (equal-material) stepping.
    ChamferGradualToggled(bool),
    ContourRoughingToggled(bool),
    /// Toggle the selected thread's gradual (equal-material) radial infeed.
    ThreadGradualToggled(bool),
    /// Create a new default tool and select it.
    NewTool,
    LibraryPresetSelected(u64),
    EditLibraryPreset(u64),
    AddPreset,
    DuplicatePreset,
    EditPreset,
    ClosePresetEditor,
    CancelPresetEditor,
    OpenMaterials,
    Noop,
    CloseMaterials,
    ToolParameterFocus(Field, bool),
    ToolInfoOpen(cam_model::Tool),
    ToolInfoClose,
    ToolDetailChanged(tool_presets::ToolDetail, String),
    DeletePreset,
    PresetNameChanged(String),
    ToolCoolingChanged(cam_model::Cooling),
    PresetCoolingChanged(cam_model::Cooling),
    OperationCoolingChanged(cam_model::Cooling),
    PresetMaterialChanged(MaterialChoice),
    PresetValueChanged(PresetField, String),
    OperationPresetPicked(PresetChoice),
    SetupMaterialPicked(MaterialChoice),
    SelectMaterial(u64),
    NewMaterial,
    EditMaterial,
    EditMaterialRow(u64),
    NewMaterialNameChanged(String),
    AddMaterial,
    AddStarterMaterials,
    MaterialRenamed(u64, String),
    MaterialCuttingForceChanged(u64, String),
    CuttingForceCustomToggled(u32, bool),
    CuttingForceInputChanged(u32, usize, String),
    DeleteMaterial(u64),
    /// Delete the selected tool.
    DeleteTool,
    /// Switch the Tool Library pane's internal tab (Serial / Family).
    SetLibraryView(LibraryView),
    ToolColumnResizeStart(usize),
    ToolColumnReorderStart(usize),
    ToolColumnHover(usize, bool),
    ToolColumnLeave(usize),
    ToolColumnRelease,
    OpenToolTableMenu(usize, bool),
    CloseToolTableMenu,
    ToolTableMenuSearch(String),
    ToolTableFilterValue(String, bool),
    ToolTableFilterAll(bool),
    ApplyToolTableFilter,
    ClearToolTableColumnFilter,
    SetToolTableColumn(usize, bool),
    ToolTableSearch(String),
    ClearToolTableFilters,
    /// Open the Library-pane right-click menu for the library tool at this index.
    LibToolMenu(usize),
    /// Morph the Library menu into the "set number" input.
    LibMenuSetNumber,
    /// Update the "set number" input buffer.
    LibNumberInput(String),
    /// Commit the "set number" input (swaps on collision).
    LibNumberCommit,
    /// Dismiss the Library-pane menu / number input.
    CloseLibMenu,
    /// Bulk-renumber the library (opens a confirm dialog first).
    RenumberLibrary,
    /// The confirm-dialog result for the bulk renumber.
    RenumberConfirmed(bool),
    /// Open the project-tree right-click menu for the tool numbered `u32` (only wired
    /// for tools not in the library).
    ToolMenu(u32),
    /// Dismiss the tool context menu.
    CloseToolMenu,
    /// Promote the project tool numbered `u32` into the shop library (§6.3).
    AddToolToLibrary(u32),
    /// Switch the active ribbon tab.
    SelectRibbonTab(RibbonTab),
    /// Open/close the collapse-popup for a collapsed group (index in the active tab).
    ToggleRibbonGroup(usize),
    /// Close any open collapse-popup.
    CloseRibbonPopup,
    /// Show (`true`) or hide (`false`) a pane.
    SetPaneVisible(Pane, bool),
    /// Stage a library tool (by index) for the new operation.
    SetPendingLibraryTool(usize),
    /// Select a library tool for editing in the Tools-tab library editor.
    SelectLibraryTool(usize),
    /// Open the right-click context menu for operation `id` (anchored under the
    /// cursor), and select that operation.
    OpMenu(u32),
    ToolpathCommand(TreeNode, ToolpathAction),
    ProcessingTimeFeedOverride(String),
    ProcessingTimeClose,
    SimulationPlayPause,
    SimulationToolpathStart,
    SimulationPreviousOperation,
    SimulationPreviousPoint,
    SimulationNextPoint,
    SimulationNextOperation,
    SimulationToolpathEnd,
    SimulationSpeed(f64),
    SimulationSeek(f64),
    SimulationTick,
    SimulationClose,
    SimulationOption(SimulationOption),
    SimulationTab(SimulationTab),
    SimulationSection(usize),
    SimulationAccuracySettled,
    SimulationSaveSettings,
    /// Move the context-menu's operation to the workpiece origin with this index —
    /// i.e. into that origin's group in the project tree, and so under its work offset
    /// at post time. Until this existed the only way across was delete-and-recreate.
    MoveOpToOrigin(u32),
    TransferOpToSetup(u64, bool),
    /// Dismiss the operation context menu.
    CloseOpMenu,
    /// Track the window-absolute cursor position (global subscription) so overlays
    /// can be placed exactly under the cursor.
    WindowCursor(iced::Point),
    /// A left-click on operation `id`'s tree row. Plain click focuses it alone;
    /// ⌘/Ctrl-click toggles it in the viewport highlight set (multi-select);
    /// plain-clicking the sole focused op clears the highlight.
    ClickOp(u32),
    /// Track live keyboard modifiers (global subscription) for click semantics.
    ModifiersChanged(iced::keyboard::Modifiers),
}

/// Grab leeway (px) for the pane_grid resize dividers. Shared with the viewport
/// so it can ignore presses in the same band (see [`in_resize_band`]).
const PANE_RESIZE_LEEWAY: f32 = 8.0;

/// pane_grid's own (global) floor; the real per-pane minimums are applied in
/// [`App::clamp_resize`].
const PANE_MIN_PX: f32 = 40.0;
/// Gap between panes.
const PANE_SPACING: f32 = 4.0;
/// Approximate menu-bar height, subtracted from the window to get the grid area.
const MENU_BAR_H: f32 = 44.0;

/// The initial layout: Project | (Viewport | Inspector), with Output below.
fn initial_panes() -> pane_grid::State<Pane> {
    use pane_grid::{Axis, Configuration};
    pane_grid::State::with_configuration(Configuration::Split {
        axis: Axis::Horizontal,
        ratio: 0.78,
        a: Box::new(Configuration::Split {
            axis: Axis::Vertical,
            ratio: 0.19,
            a: Box::new(Configuration::Pane(Pane::Project)),
            b: Box::new(Configuration::Split {
                axis: Axis::Vertical,
                ratio: 0.74,
                a: Box::new(Configuration::Pane(Pane::Viewport)),
                b: Box::new(Configuration::Pane(Pane::Inspector)),
            }),
        }),
        b: Box::new(Configuration::Pane(Pane::Output)),
    })
}

impl App {
    fn toggle_setup_body_pick(&mut self, instance: InstanceId) -> iced::Task<Message> {
        let included = self.setup_draft.as_ref().is_some_and(|draft| {
            if self.setup_fixture_pick {
                draft.fixture_instances.contains(&instance)
            } else {
                draft.model_instances.contains(&instance)
            }
        });
        self.update(if self.setup_fixture_pick {
            Message::SetupFixtureToggled(instance, !included)
        } else {
            Message::SetupModelToggled(instance, !included)
        })
    }

    fn begin_setup_draft(&mut self) {
        self.model_view = None;
        self.inspector_active = true;
        self.inspector_enabled = true;
        self.cancel_setup_picks();
        self.setup_tab = SetupEditorTab::Setup;
        self.setup_orientation = None;
        self.setup_origin_mode = None;
        self.setup_draft = self.controller.document().active_setup().cloned();
        self.setup_values.clear();
        self.setup_model_selection = None;
        self.setup_fixture_selection = None;
        self.operation_draft = None;
        self.operation_tools.clear();
        self.operation_geometry = None;
        self.operation_binding_preview = None;
    }

    fn end_setup_draft(&mut self) {
        self.setup_draft = None;
        self.setup_values.clear();
        self.setup_model_selection = None;
        self.setup_fixture_selection = None;
        self.cancel_setup_picks();
        self.inspector_active = false;
    }

    /// Refresh an existing edit session after a committed document change.
    /// Background operation updates must never start a setup edit session.
    fn rebase_setup_draft(&mut self) {
        if self.setup_draft.is_none() {
            return;
        }
        let id = self.controller.active_setup_id();
        self.setup_draft = self
            .controller
            .setups()
            .iter()
            .find(|s| s.id == id)
            .cloned();
        self.setup_values.clear();
        self.setup_model_selection = None;
        self.setup_fixture_selection = None;
        self.setup_orientation = None;
        self.setup_origin_mode = None;
    }

    fn setup_draft_dirty(&self) -> bool {
        let Some(draft) = &self.setup_draft else {
            return false;
        };
        if !self.setup_values.is_empty() {
            return true;
        }
        self.controller
            .setups()
            .iter()
            .find(|s| s.id == draft.id)
            .is_some_and(|saved| saved != draft)
    }

    fn setup_value(&self, key: usize, model: String) -> String {
        self.setup_values.get(&key).cloned().unwrap_or(model)
    }

    fn clear_setup_origin_buffers(&mut self) {
        for key in 0..3 {
            self.setup_values.remove(&key);
        }
    }

    fn cancel_setup_picks(&mut self) {
        if self.setup_model_pick {
            self.cad_hover = None;
            self.cad_candidates.clear();
        }
        self.setup_model_pick = false;
        self.setup_fixture_pick = false;
        self.setup_hover = SetupHover::default();
        self.setup_axis_pick = None;
        self.setting_origin = false;
        self.setting_origin_2pt = false;
        self.origin_first = None;
        self.snap_hover = None;
        self.hover_loop = None;
    }

    fn setup_frame_issue(&self, setup: &cam_model::Setup) -> Option<String> {
        if let Err(error) = setup.frame.validate() {
            return Some(error);
        }
        let refs = &setup.frame.references;
        let resolved = if refs.x.is_some()
            || refs.y.is_some()
            || refs.z.is_some()
            || refs.stock_axes.iter().any(Option::is_some)
        {
            Some(
                setup
                    .frame
                    .resolve_references(self.controller.geometry_state(), refs.clone()),
            )
        } else {
            refs.model_instance.map(|id| {
                self.controller
                    .pending_frame_from_model(id)
                    .map(|(frame, _)| frame)
            })
        };
        if let Some(Err(error)) = resolved {
            return Some(error);
        }
        if let Some(Ok(frame)) = resolved {
            for i in 0..3 {
                for j in 0..3 {
                    if (frame.world_from_setup.rotation[i][j]
                        - setup.frame.world_from_setup.rotation[i][j])
                        .abs()
                        > 1e-6
                    {
                        return Some(
                            "WCS references no longer match the selected setup axes.".into(),
                        );
                    }
                }
            }
        }
        None
    }

    fn begin_operation_draft(&mut self, id: u32) {
        self.rotary_pick = None;
        self.rotary_wrap_draft = None;
        self.seed_rotary_index_draft(id);
        self.thread_parameters = Default::default();
        self.geometry_editor = None;
        self.profile_contour_selection = 0;
        self.profile_side_overrides.clear();
        self.profile_entry_editor = None;
        self.operation_position_pick = None;
        self.operation_position_markers = [None; 2];
        self.profile_tab_picking = false;
        self.profile_tab_selection = None;
        self.model_view = None;
        self.end_setup_draft();
        self.inspector_active = true;
        self.controller.select(Selection::Operation(id));
        self.operation_geometry = None;
        self.operation_binding_preview = None;
        self.drawing_editing_op = None;
        self.operation_draft = self.controller.operation(id).cloned();
        self.stock_to_leave_draft = Some(self.controller.inspection_setup().stock_to_leave_for(id));
        self.stock_to_leave_initialized = self.stock_to_leave_value().enabled;
        if matches!(self.operation_draft, Some(Operation::Profile(_))) {
            let first = self
                .controller
                .geometry_state()
                .bindings
                .get(&id)
                .and_then(|b| b.selected.first());
            self.profile_selection_mode = match first.map(|r| r.kind) {
                Some(EntityKind::Face) => ProfileSelectionMode::FaceContours,
                Some(EntityKind::Silhouette) => ProfileSelectionMode::Silhouette,
                _ => ProfileSelectionMode::Chain,
            };
            self.profile_chain_edges = first.is_some_and(|r| r.kind == EntityKind::Edge);
        }
        self.seed_height_editor(id);
        self.operation_tools.clear();
        self.fields.clear();
        self.refresh_fields();
    }

    fn seed_rotary_index_draft(&mut self, id: u32) {
        self.rotary_index_draft = self.controller.document().rotary_indexes.get(&id).cloned();
        self.rotary_index_mode = if self
            .rotary_index_draft
            .as_ref()
            .is_some_and(|i| i.face.is_some())
        {
            rotary_view::IndexMode::Face
        } else {
            rotary_view::IndexMode::Manual
        };
        self.seed_rotary_index_values();
    }

    fn stage_operation_tool(&mut self, mut tool: cam_model::Tool) -> u32 {
        if let Some(number) = self
            .controller
            .inspection_setup()
            .tools
            .iter()
            .find(|saved| saved.identity() == tool.identity())
            .map(|t| t.number)
        {
            tool.number = number;
            if let Some(staged) = self.operation_tools.iter_mut().find(|t| t.number == number) {
                *staged = tool;
            } else {
                self.operation_tools.push(tool);
            }
            return number;
        }
        if let Some(existing) = self
            .operation_tools
            .iter_mut()
            .find(|saved| saved.identity() == tool.identity())
        {
            tool.number = existing.number;
            let number = tool.number;
            *existing = tool;
            return number;
        }
        let number = self
            .controller
            .inspection_setup()
            .tools
            .iter()
            .chain(self.operation_tools.iter())
            .map(|saved| saved.number)
            .max()
            .unwrap_or(0)
            .saturating_add(1)
            .max(1);
        tool.number = number;
        self.operation_tools.push(tool);
        number
    }

    fn operation_draft_dirty(&self) -> bool {
        let Selection::Operation(id) = self.controller.selection() else {
            return false;
        };
        if self.rotary_index_draft.as_ref() != self.controller.document().rotary_indexes.get(&id) {
            return true;
        }
        if self.height_rules_dirty() {
            return true;
        }
        if self.operation_geometry.as_ref().is_some_and(|selected| {
            self.controller
                .geometry_state()
                .bindings
                .get(&id)
                .is_some_and(|binding| binding.selected != *selected)
        }) {
            return true;
        }
        self.operation_draft.as_ref().is_some_and(|draft| {
            self.controller
                .operation(id)
                .is_some_and(|saved| saved != draft)
        })
    }

    fn current_operation(&self, id: u32) -> Option<&Operation> {
        if self.creating_operation() {
            return self.operation_draft.as_ref().filter(|op| op.id() == id);
        }
        self.operation_draft
            .as_ref()
            .filter(|op| op.id() == id)
            .or_else(|| self.controller.operation(id))
    }

    fn edit_operation_draft(&mut self, edit: impl FnOnce(&mut Operation)) {
        // Tree selection and Apply can leave an operation selected without a
        // draft. Start one on the first edit without resetting numeric buffers
        // or the cutter that the picker has just staged.
        if !self.creating_operation() {
            if let Selection::Operation(id) = self.controller.selection() {
                if self.operation_draft.as_ref().map(Operation::id) != Some(id) {
                    self.operation_draft = self.controller.operation(id).cloned();
                }
            }
        }
        if let Some(operation) = &mut self.operation_draft {
            edit(operation);
        }
    }

    fn refresh_operation_fields(&mut self) {
        let previous = std::mem::take(&mut self.fields);
        let overrides = (
            self.cad_override_top.clone(),
            self.cad_override_bottom.clone(),
        );
        self.refresh_fields();
        (self.cad_override_top, self.cad_override_bottom) = overrides;
        let supported = self.inspector_fields();
        for (&field, value) in &previous {
            if supported.contains(&field)
                || (value.trim().parse::<f64>().is_err()
                    && parameter_expressions::parameter(field).is_some_and(|p| {
                        self.operation_draft
                            .as_ref()
                            .is_some_and(|op| p.value(op).is_some())
                    }))
            {
                self.fields.insert(field, value.clone());
            }
        }
        self.seed_user_parameter_defaults(|field| !previous.contains_key(&field));
    }

    fn set_round_stock_axis(&mut self, entity: EntityRef) -> bool {
        let Some(draft) = &self.setup_draft else {
            return false;
        };
        if entity.kind != EntityKind::Edge {
            self.status = "Select a straight model edge or an origin axis for the stock.".into();
            return false;
        }
        let mut candidate = draft.clone();
        match &mut candidate.stock {
            cam_model::Stock::FixedRound { axis, .. }
            | cam_model::Stock::RelativeRound { axis, .. } => {
                *axis = Some(cam_model::StockAxis::Edge(
                    cam_model::StockAxisReference::from(&entity),
                ))
            }
            _ => return false,
        }
        if let Err(error) = self.controller.document().round_stock_geometry(&candidate) {
            self.status = error;
            return false;
        }
        self.setup_draft = Some(candidate);
        self.status = format!("Stock axis set from edge {}.", entity.id);
        true
    }

    fn set_setup_axis_reference(&mut self, axis: usize, entity: Option<EntityRef>) {
        let Some(draft) = &mut self.setup_draft else {
            return;
        };
        if axis >= 3 {
            return;
        }
        let refs = &mut draft.frame.references;
        let target = match axis {
            0 => &mut refs.x,
            1 => &mut refs.y,
            _ => &mut refs.z,
        };
        let flipped = target.as_ref().is_some_and(|r| r.flipped);
        *target = entity.map(|entity| cam_model::FrameAxisReference { entity, flipped });
        refs.stock_axes[axis] = None;
        refs.stock_flipped[axis] = false;
        refs.model_instance = None;
        self.resolve_setup_axes();
    }
    fn set_setup_stock_axis(&mut self, axis: usize, direction: [f64; 3]) {
        let Some(draft) = &mut self.setup_draft else {
            return;
        };
        if axis >= 3 {
            return;
        }
        let refs = &mut draft.frame.references;
        match axis {
            0 => refs.x = None,
            1 => refs.y = None,
            _ => refs.z = None,
        }
        refs.stock_axes[axis] = Some(direction);
        refs.stock_flipped[axis] = false;
        refs.model_instance = None;
        self.resolve_setup_axes();
    }
    fn set_setup_axis_flip(&mut self, axis: usize, flipped: bool) {
        let Some(draft) = &mut self.setup_draft else {
            return;
        };
        if axis >= 3 {
            return;
        }
        let refs = &draft.frame.references;
        let reference = [&refs.x, &refs.y, &refs.z][axis];
        let current = reference
            .as_ref()
            .map_or(refs.stock_flipped[axis], |r| r.flipped);
        if current == flipped {
            return;
        }
        if self.setup_orientation == Some(SetupOrientation::Model) {
            self.setup_orientation = Some(SetupOrientation::SelectedAxes);
        }
        // A sign change on one axis alone mirrors the frame. Rotate by 180°
        // instead: keep Z when flipping X/Y, and keep X when flipping Z.
        let companion = if axis == 1 { 0 } else { 1 };
        let world_origin = draft.frame.to_world_vector(draft.origin);
        for index in [axis, companion] {
            for row in &mut draft.frame.world_from_setup.rotation {
                row[index] = -row[index];
            }
            let refs = &mut draft.frame.references;
            if let Some(reference) = match index {
                0 => &mut refs.x,
                1 => &mut refs.y,
                _ => &mut refs.z,
            } {
                reference.flipped = !reference.flipped;
            } else if refs.stock_axes[index].is_some() {
                refs.stock_flipped[index] = !refs.stock_flipped[index];
            }
            refs.model_instance = None;
        }
        draft.origin = draft.frame.to_setup_vector(world_origin);
        draft.frame.references.origin =
            draft.frame.references.origin.with_coordinates(draft.origin);
        self.resolve_setup_axes();
    }

    fn resolve_setup_axes(&mut self) {
        let Some(draft) = &mut self.setup_draft else {
            return;
        };
        let refs = &draft.frame.references;
        match draft
            .frame
            .resolve_references(self.controller.geometry_state(), refs.clone())
        {
            Ok(mut frame) => {
                let origin = draft.frame.to_world_vector(draft.origin);
                draft.origin = frame.to_setup_vector(origin);
                frame.references.origin = frame.references.origin.with_coordinates(draft.origin);
                draft.frame = frame;
                self.setup_values.retain(|key, _| *key > 2);
                self.status = "WCS edge reference updated in the setup draft.".into();
            }
            Err(error) => self.report_error(error),
        }
    }

    fn new_with_setup(path: &Path) -> Result<(Self, iced::Task<Message>), crate::ProjectError> {
        let (mut app, task) = Self::new();
        app.open_project(path)?;
        Ok((app, task))
    }

    fn save_tool_libraries(&mut self) {
        self.libraries
            .libraries
            .insert(self.libraries.active.clone(), self.library.clone());
        crate::config::save(&self.libraries);
    }

    fn switch_tool_library(&mut self, name: String) {
        if name == self.libraries.active {
            return;
        }
        if self.library_mode() && self.inspector_dirty() {
            self.status =
                "Apply or discard the current tool edits before switching libraries.".into();
            return;
        }
        let Some(next) = self.libraries.libraries.get(&name).cloned() else {
            return;
        };
        self.libraries
            .libraries
            .insert(self.libraries.active.clone(), self.library.clone());
        self.library_name_edit = name.clone();
        self.library_renaming = false;
        self.libraries.active = name;
        self.library = next;
        self.lib_sel = 0;
        self.tool_drafts.clear();
        self.reload_tool_edit();
        if let Some(player) = &mut self.simulation {
            player.sync_force_material(&self.library);
        }
        self.save_tool_libraries();
    }

    fn library_picker(&self) -> Element<'_, Message> {
        pick_list(
            self.libraries.libraries.keys().cloned().collect::<Vec<_>>(),
            Some(self.libraries.active.clone()),
            Message::SelectToolLibrary,
        )
        .text_size(12)
        .padding([7, 9])
        .style(inspector_picker)
        .into()
    }

    fn new() -> (Self, iced::Task<Message>) {
        // Destructured rather than discarded: a library that failed to load is the one
        // thing at startup the operator must be *told* about, or they meet the stock
        // tools and assume theirs were lost. They are not — the file is untouched, with
        // a `.bak` beside it — but only if nobody edits a tool first.
        let (legacy, library_load) = ToolLibrary::load();
        let (loaded_libraries, collection_load) =
            crate::config::load::<crate::tool_library::LibraryCollection>();
        let libraries = if matches!(collection_load, crate::config::ConfigLoad::Loaded) {
            loaded_libraries
        } else {
            crate::tool_library::LibraryCollection::from_legacy(legacy)
        };
        let library = libraries.libraries[&libraries.active].clone();
        // Preferences seed the session. A rejected settings file is deliberately not
        // reported the way a rejected *library* is: defaults here cost the user a
        // layout, not their tools.
        let (settings, _) = crate::load_settings();
        let (machines, machine_load) = crate::load_machines();
        // Resolve where you left off. A name that no longer resolves lands on the first
        // machine — never on nothing — and is *reported*, because silently authoring
        // against a different machine's travel is the failure this whole area exists to
        // prevent.
        let remembered = settings.session.machine.clone();
        let stale = remembered.is_some() && !machines.resolves(remembered.as_deref());
        let active = machines
            .resolve(remembered.as_deref())
            .cloned()
            .unwrap_or_else(|| crate::MachineEntry::new(crate::default_machine()));
        let active_machine = active.name().to_string();
        let mut app = Self {
            controller: AppController::new(active.machine.clone()),
            pending_discard: None,
            discarding_confirmed: false,
            mcp: None,
            mcp_feedback: None,
            mcp_task: iced::Task::none(),
            viewport_cache: Default::default(),
            profile_contour_cache: Default::default(),
            cad_scene_cache: Default::default(),
            notice: None,
            notice_queue: VecDeque::new(),
            notice_filters: [true; 3],
            status_messages: StatusMessages::default(),
            move_draft: None,
            setup_draft: None,
            pattern_draft: None,
            pattern_tab: pattern_editor_view::PatternTab::Placement,
            setup_model_selection: None,
            setup_fixture_selection: None,
            setup_fixture_pick: false,
            setup_model_pick: false,
            setup_values: BTreeMap::new(),
            operation_draft: None,
            stock_to_leave_draft: None,
            stock_to_leave_initialized: false,
            rotary_pick: None,
            rotary_wrap_draft: None,
            rotary_index_draft: None,
            rotary_index_mode: rotary_view::IndexMode::Manual,
            rotary_index_values: Default::default(),
            creation_edited_fields: BTreeSet::new(),
            thread_parameters: Default::default(),
            creation_geometry_height_defaults: None,
            operation_geometry: None,
            geometry_row_selection: None,
            geometry_additive: false,
            geometry_auto_pick: false,
            geometry_editor: None,
            profile_tab_picking: false,
            operation_position_pick: None,
            operation_position_markers: [None; 2],
            profile_tab_selection: None,
            profile_contour_selection: 0,
            profile_side_overrides: Vec::new(),
            profile_entry_editor: None,
            operation_binding_preview: None,
            drawing_editing_op: None,
            operation_tab: OperationTab::default(),
            collapsed_operation_sections: BTreeSet::new(),
            focused_height: None,
            parameter_editor: Default::default(),
            parameter_menu: None,
            height_draft_id: None,
            height_draft: None,
            height_disabled_feed_reference: None,
            height_offsets: BTreeMap::new(),
            height_reference_pick: None,
            collapsed_heights: BTreeSet::new(),
            workspace: Workspace::Machining,
            machining_tab: RibbonTab::Home,
            show_project_browser: true,
            planar_selection: Vec::new(),
            tree_drag: None,
            operation_rename: None,
            tree_hover: None,
            tree_menu: None,
            pattern_menu_y: None,
            operation_tools: Vec::new(),
            setup_tab: SetupEditorTab::Setup,
            setup_axis_pick: None,
            setup_orientation: None,
            setup_origin_mode: None,
            expanded_setups: BTreeSet::from([1]),
            collapsed_assets: BTreeSet::new(),
            expanded_operations: BTreeSet::new(),
            expanded_patterns: BTreeSet::new(),
            tools_expanded: false,
            hidden_assets: BTreeSet::new(),
            model_view: Some(TreeNode::Models),
            browser_view: TreeNode::Models,
            models_collapsed: true,
            setups_collapsed: false,
            hidden_setups: BTreeSet::new(),
            hidden_ops: BTreeSet::new(),
            panes: initial_panes(),
            inspector_active: false,
            inspector_enabled: true,
            show_license: false,
            main_window: None,
            preferences: None,
            window: iced::Size::new(1280.0, 800.0),
            project_px: settings.panes.project_px,
            inspector_px: settings.panes.inspector_px,
            output_px: settings.panes.output_px,
            active_tab: RibbonTab::Home,
            open_group: None,
            fields: BTreeMap::new(),
            show_stock: settings.view.show_stock,
            show_gizmo: settings.view.show_gizmo,
            tooltips: settings.view.tooltips,
            gizmo_size: settings.view.gizmo_size,
            view: ViewControls::default(),
            cursor: None,
            library,
            library_name_edit: libraries.active.clone(),
            library_renaming: false,
            library_context_menu: None,
            library_sidebar_drag: None,
            libraries,
            machines,
            active_machine,
            machine_name_edit: None,
            machine_post_edit: None,
            machine_estlcam_edit: None,
            lib_sel: 0,
            fusion_import: None,
            tool_edit: None,
            tool_drafts: BTreeMap::new(),
            library_preset: None,
            preset_values: BTreeMap::new(),
            edited_preset_input: None,
            edited_tool_input: None,
            selected_tool_parameter: None,
            tool_info: None,
            operation_tool_selector: None,
            cad_selected_project_tool: None,
            preset_buffers: BTreeMap::new(),
            new_material_name: String::new(),
            selected_material: None,
            editing_material: None,
            creating_material: false,
            material_force_inputs: BTreeMap::new(),
            cutting_force_inputs: BTreeMap::new(),
            show_materials: false,
            show_preset_editor: false,
            preset_editor_snapshot: None,
            library_view: if settings.tool_table.grouped {
                LibraryView::Grouped
            } else {
                LibraryView::Ordered
            },
            tool_table_columns: settings.tool_table.visible,
            tool_table_widths: settings.tool_table.widths,
            tool_table_order: settings.tool_table.order,
            tool_table_drag: None,
            tool_table_menu: None,
            tool_table_menu_pos: iced::Point::ORIGIN,
            tool_table_menu_search: String::new(),
            tool_table_filter_draft: BTreeSet::new(),
            tool_table_search: settings.tool_table.search.clone(),
            tool_table_filters: settings.tool_table.filters.clone(),
            open_op_menu: None,
            pending_toolpath_export: None,
            pending_simulation: None,
            pending_processing_time: None,
            processing_time_dialog: None,
            simulation: None,
            operation_preview: Default::default(),
            op_menu_pos: iced::Point::ORIGIN,
            open_tool_menu: None,
            tool_menu_pos: iced::Point::ORIGIN,
            lib_menu: None,
            lib_menu_pos: iced::Point::ORIGIN,
            window_cursor: iced::Point::ORIGIN,
            focus_ops: BTreeSet::new(),
            modifiers: iced::keyboard::Modifiers::default(),
            profile_selection_mode: ProfileSelectionMode::Chain,
            profile_coplanar: false,
            profile_chain_edges: false,
            cad_filter: CadFilter::Face,
            cad_filter_explicit: false,
            cad_hover: None,
            cad_candidates: vec![],
            cad_hidden: BTreeSet::new(),
            cad_select_through: false,
            cad_pending_kind: None,
            cad_tool_pick: None,
            cad_inspecting: false,
            cad_editing_op: None,
            cad_override_top: String::new(),
            cad_override_bottom: String::new(),
            snaps: settings.snapping.default_snaps.clone(),
            snap_hover: None,
            setup_hover: SetupHover::default(),
            snap_aperture: 1.0,
            hover_loop: None,
            setting_origin: false,
            setting_origin_2pt: false,
            origin_first: None,
            show_origin: settings.view.show_origin,
            show_envelope: settings.view.show_envelope,
            status: "Open the sample part to begin.".to_string(),
            output_history: VecDeque::new(),
            output_content: iced::widget::text_editor::Content::new(),
            output_snapshot: String::new(),
            output_last_geometry_error: None,
            output_cleared: false,
            // Last: every field above reads from it.
            settings,
        };
        // The active machine carries its control, so resolving the machine above chose
        // the post too — one source of truth, which is why `session.post` was retired.
        app.controller.set_post_kind(active.post);
        app.controller.set_estlcam_options(active.estlcam);
        if stale {
            app.status = format!(
                "The machine you last used is no longer in the library; using \"{}\" \
                 instead. Check it before cutting.",
                app.active_machine
            );
        } else if let crate::MachineLoad::Rejected(why) = &machine_load {
            app.status = format!(
                "Machine library {why}. Using the default machine; your file is untouched \
                 (a copy is beside it as machines.json.bak)."
            );
        }
        if let crate::config::ConfigLoad::Rejected(why) = &collection_load {
            app.status = format!("Tool libraries {why}. Your file was preserved with a backup.");
        }
        if let crate::LibraryLoad::Rejected(why) = &library_load {
            app.status = format!(
                "Tool library {why}. Using the starter tools; your file is untouched \
                 (a copy is beside it as tools.json.bak)."
            );
        }
        if let Some(project) = app.pane_handle(Pane::Project) {
            app.panes.close(project);
        }
        app.controller
            .set_simulation_accuracy(app.settings.simulation.accuracy);
        app.controller.set_block_export_on_verification_errors(
            app.settings.block_export_on_verification_errors,
        );
        app.controller.select(Selection::Setup);
        app.restore_pane_preferences();
        // Always open on Home, regardless of the workspace last used.
        app.workspace = Workspace::Machining;
        app.active_tab = RibbonTab::Home;
        app.machining_tab = RibbonTab::Home;
        app.refresh_fields();
        app.sync_inspector_visibility();
        // Seed the fixed layout against the assumed window size; the first real
        // resize event re-applies it against the true size.
        app.apply_fixed_layout();
        app.sync_output();
        (app, iced::Task::none())
    }

    fn update(&mut self, message: Message) -> iced::Task<Message> {
        match message {
            Message::Preferences(edit) => return self.edit_preferences(edit),
            Message::CloseRequested(id) if self.is_preferences_window(id) => {
                return self.close_preferences();
            }
            _ => {}
        }
        if matches!(message, Message::CadEscape) && self.operation_rename.is_some() {
            self.operation_rename = None;
            return iced::Task::none();
        }
        if let Message::DiscardConfirmed = message {
            if let Some(action) = self.pending_discard.take() {
                self.discarding_confirmed = true;
                let task = self.update(action.message());
                self.discarding_confirmed = false;
                return task;
            }
            return iced::Task::none();
        }
        if matches!(message, Message::DiscardCancelled)
            || (self.pending_discard.is_some() && matches!(message, Message::CadEscape))
        {
            self.pending_discard = None;
            return iced::Task::none();
        }
        if self.pending_discard.is_some()
            && !matches!(
                message,
                Message::WindowResized(_) | Message::WindowCursor(_)
            )
        {
            return iced::Task::none();
        }
        // Handle window-manager close requests even when another editor/dialog is open.
        if let Message::CloseRequested(id) = message {
            if self.main_window.is_some_and(|main| id != main) {
                return iced::Task::none();
            }
            if !self.discarding_confirmed && self.has_changes_to_discard() {
                self.pending_discard = Some(DiscardAction::Close(id));
                return iced::Task::none();
            }
            return iced::exit();
        }
        if matches!(message, Message::PollMcp) {
            return self.poll_mcp();
        }
        if self.move_draft.is_some()
            && matches!(
                &message,
                Message::Select(_)
                    | Message::SelectSetup(_)
                    | Message::EditSetup(_)
                    | Message::ClickOp(_)
                    | Message::BeginOp(_)
                    | Message::Undo
                    | Message::Redo
                    | Message::RunJob
                    | Message::ExportNc
                    | Message::ImportCad
                    | Message::ReplaceCadAsset(_)
                    | Message::RemoveCadAsset(_)
                    | Message::MoveOpen(_)
            )
        {
            self.status = "Apply or cancel the move first.".into();
            return iced::Task::none();
        }
        if self.pattern_draft.is_some()
            && matches!(
                &message,
                Message::Select(_)
                    | Message::SelectSetup(_)
                    | Message::EditSetup(_)
                    | Message::ClickOp(_)
                    | Message::BeginOp(_)
                    | Message::SelectOrigin(_)
                    | Message::AddOrigin
                    | Message::SetupNew
                    | Message::SetupDuplicate(_)
                    | Message::SetupDelete(_)
                    | Message::DuplicateOp
                    | Message::DeleteOp
                    | Message::ReinitOp
                    | Message::MoveOpToOrigin(_)
                    | Message::TransferOpToSetup(_, _)
                    | Message::SetOpExcluded(_, _)
                    | Message::Undo
                    | Message::Redo
                    | Message::RunJob
                    | Message::ExportNc
                    | Message::ToolpathCommand(_, _)
                    | Message::OpenSample
                    | Message::NewProject
                    | Message::OpenProject
            )
        {
            self.status = "Apply or cancel the pattern draft before changing selection or processing toolpaths.".into();
            return iced::Task::none();
        }
        let validate_geometry = match &message {
            Message::ProfileEntry(
                profile_entries_view::EntryMessage::Name(_)
                | profile_entries_view::EntryMessage::Tolerance(_),
            ) => false,
            Message::CadSelect(_)
            | Message::CadPicked(_)
            | Message::BeginOp(_)
            | Message::ProfileEntry(_)
            | Message::ConfirmCadOperation
            | Message::GeometryRemove(_)
            | Message::GeometryClear
            | Message::ProfileSelectionModeChanged(_)
            | Message::ProfileChainEdgesChanged(_) => true,
            _ => false,
        };
        let sync_thread = matches!(
            &message,
            Message::ThreadParameters(_)
                | Message::ThreadInternalChanged(_)
                | Message::CadSelect(_)
                | Message::CadPicked(_)
                | Message::PickWorld(_, _)
                | Message::OperationGeometry(_)
                | Message::GeometryRemove(_)
                | Message::GeometryClear
                | Message::ConfirmCadOperation
                | Message::UseDrawingGeometry
                | Message::BeginOp(_)
                | Message::OperationToolPicked(_)
                | Message::OperationProjectToolPicked(_)
        );
        let previous_status = self.status.clone();
        let previous_result = self.controller.outcome().and_then(|o| o.stamp.clone());
        let context = self.output_context();
        let task = self.update_message(message);
        if self
            .controller
            .outcome()
            .and_then(|o| o.stamp.as_ref())
            .is_some_and(|stamp| stamp.request_id == 0 && Some(stamp) != previous_result.as_ref())
        {
            self.record_status_messages();
        }
        if sync_thread {
            self.sync_thread_standard();
        }
        if self.status != previous_status {
            self.append_output(&context, &self.status.clone());
        }
        if validate_geometry {
            self.log_geometry_validation();
        }
        self.sync_output();
        if self
            .simulation
            .as_ref()
            .is_some_and(|player| !player.current(&self.controller))
        {
            self.simulation = None;
        }
        self.sync_inspector_visibility();
        self.sync_operation_preview();
        task
    }

    fn update_message(&mut self, message: Message) -> iced::Task<Message> {
        if matches!(message, Message::CadEscape) && self.parameter_menu.take().is_some() {
            return iced::Task::none();
        }
        if matches!(message, Message::CadEscape) && self.library_context_menu.is_some() {
            self.library_context_menu = None;
            return iced::Task::none();
        }
        if matches!(message, Message::CadEscape) && self.library_renaming {
            self.library_renaming = false;
            self.library_name_edit = self.libraries.active.clone();
            return iced::Task::none();
        }
        if self.operation_tool_selector.is_some() && matches!(message, Message::CadEscape) {
            if self.tool_table_menu.is_some() {
                self.tool_table_menu = None;
            } else {
                self.close_operation_tool_selector();
            }
            return iced::Task::none();
        }
        if self.operation_tool_selector.is_some()
            && !operation_tool_selector::selector_message(&message)
        {
            return iced::Task::none();
        }
        if self.tool_info.is_some() && matches!(message, Message::CadEscape) {
            self.tool_info = None;
            return iced::Task::none();
        }
        if self.move_draft.is_some() && matches!(message, Message::CadEscape) {
            self.cancel_move();
            return iced::Task::none();
        }
        if self.tool_info.is_some()
            && !matches!(
                &message,
                Message::ToolInfoClose
                    | Message::PollRun
                    | Message::PollCadImport
                    | Message::WindowResized(_)
                    | Message::WindowCursor(_)
                    | Message::ModifiersChanged(_)
                    | Message::NoticeDismissed
                    | Message::NoticeFilterChanged(..)
            )
        {
            return iced::Task::none();
        }
        if self.processing_time_dialog.is_some()
            && self.notice.is_none()
            && matches!(message, Message::CadEscape)
        {
            self.processing_time_dialog = None;
            return iced::Task::none();
        }
        if self.processing_time_dialog.is_some()
            && !matches!(
                &message,
                Message::ProcessingTimeClose
                    | Message::ProcessingTimeFeedOverride(_)
                    | Message::CadEscape
                    | Message::PollRun
                    | Message::PollCadImport
                    | Message::NoticeDismissed
                    | Message::WindowResized(_)
                    | Message::WindowCursor(_)
                    | Message::ModifiersChanged(_)
            )
        {
            return iced::Task::none();
        }
        if (self.show_materials || self.show_preset_editor)
            && matches!(message, Message::CadEscape | Message::Apply)
        {
            let close = if self.show_materials {
                Message::CloseMaterials
            } else if matches!(message, Message::CadEscape) {
                Message::CancelPresetEditor
            } else {
                Message::ClosePresetEditor
            };
            return self.update_message(close);
        }
        if (self.show_materials || self.show_preset_editor)
            && matches!(
                message,
                Message::ToggleStock | Message::CheckHeightFocus | Message::Tree(_)
            )
        {
            return iced::Task::none();
        }
        match &message {
            Message::OutputAction(action) => {
                self.output_action(action.clone());
                return iced::Task::none();
            }
            Message::OutputCopy => {
                return iced::clipboard::write(
                    self.output_content
                        .selection()
                        .unwrap_or_else(|| self.output_content.text()),
                )
            }
            Message::OutputClear => {
                self.output_history.clear();
                self.output_cleared = true;
                self.output_snapshot.clear();
                self.output_content = iced::widget::text_editor::Content::new();
                return iced::Task::none();
            }
            _ => {}
        }
        if self.geometry_editor.is_some() && matches!(message, Message::Apply | Message::ConfirmOp)
        {
            self.operation_geometry_message(GeometryMessage::Confirm);
            return iced::Task::none();
        }
        if self.profile_entry_editor.is_some()
            && matches!(message, Message::Apply | Message::ConfirmOp)
        {
            self.status = "Finish or cancel the geometry entry settings first.".into();
            return iced::Task::none();
        }
        if matches!(message, Message::CancelOp | Message::CancelOperationDraft) {
            self.profile_entry_editor = None;
        }
        if self.notice.is_some() {
            if matches!(&message, Message::CadEscape | Message::Apply) {
                return self.update(Message::NoticeDismissed);
            }
            if !matches!(
                &message,
                Message::NoticeDismissed
                    | Message::NoticeBackdrop
                    | Message::NoticeFilterChanged(..)
                    | Message::PollCadImport
                    | Message::PollRun
                    | Message::WindowResized(_)
                    | Message::WindowCursor(_)
                    | Message::ModifiersChanged(_)
            ) {
                return iced::Task::none();
            }
        }
        if self.library_mode()
            && (self.inspector_dirty() || !self.tool_drafts.is_empty())
            && matches!(
                &message,
                Message::NewTool
                    | Message::DeleteTool
                    | Message::ImportLibrary
                    | Message::NewToolLibrary
                    | Message::DeleteToolLibrary
                    | Message::DeleteToolLibraryConfirmed((_, true))
                    | Message::SelectToolLibrary(_)
                    | Message::LibMenuSetNumber
                    | Message::RenumberLibrary
                    | Message::RenumberConfirmed(true)
                    | Message::LibraryToImport(Some(_))
            )
        {
            self.status = "OK or Cancel tool changes before changing the library.".into();
            return iced::Task::none();
        }
        if (self.controller.pending_op().is_some()
            || self.cad_pending_kind.is_some()
            || self.cad_editing_op.is_some())
            && matches!(
                &message,
                Message::Select(_)
                    | Message::SelectSetup(_)
                    | Message::EditSetup(_)
                    | Message::ClickOp(_)
                    | Message::BeginOp(_)
                    | Message::SelectOrigin(_)
                    | Message::SetupNew
                    | Message::SetupDuplicate(_)
                    | Message::SetupDelete(_)
                    | Message::Undo
                    | Message::Redo
                    | Message::NewProject
                    | Message::OpenProject
                    | Message::OpenSample
                    | Message::ImportCad
            )
        {
            self.status =
                "Confirm or Cancel geometry selection before changing the editing context.".into();
            return iced::Task::none();
        }
        let operation_settings_dirty = self.inspector_active
            && matches!(self.controller.selection(), Selection::Operation(_))
            && (self.inspector_dirty()
                || self.any_field_invalid()
                || !self.operation_height_overrides_valid());
        if (self.setup_draft_dirty() || operation_settings_dirty)
            && matches!(
                &message,
                Message::OpenSample
                    | Message::Undo
                    | Message::Redo
                    | Message::SelectRibbonTab(RibbonTab::Tools | RibbonTab::Machinery)
                    | Message::Select(_)
                    | Message::SelectSetup(_)
                    | Message::EditSetup(_)
                    | Message::ClickOp(_)
                    | Message::BeginOp(_)
                    | Message::SelectOrigin(_)
                    | Message::AddOrigin
                    | Message::NewProject
                    | Message::OpenProject
                    | Message::ImportCad
                    | Message::SetupNew
                    | Message::SetupDuplicate(_)
                    | Message::SetupDelete(_)
                    | Message::DuplicateOp
                    | Message::DeleteOp
                    | Message::ReinitOp
                    | Message::MoveOpToOrigin(_)
                    | Message::TransferOpToSetup(_, _)
                    | Message::SetOpExcluded(_, _)
                    | Message::OpMenu(_)
            )
        {
            self.status = if operation_settings_dirty { "Apply or cancel operation changes before changing selection or restructuring operations." } else { "Apply or cancel setup changes before changing selection or starting another task." }.into();
            return iced::Task::none();
        }
        if !self.discarding_confirmed {
            let action = match &message {
                Message::NewProject => Some(DiscardAction::New),
                Message::OpenSample => Some(DiscardAction::Sample),
                Message::ProjectToOpen(Some(path)) => Some(DiscardAction::Open(path.clone())),
                _ => None,
            };
            if let Some(action) = action.filter(|_| self.has_changes_to_discard()) {
                self.pending_discard = Some(action);
                return iced::Task::none();
            }
        }
        if matches!(
            &message,
            Message::Select(_)
                | Message::SelectSetup(_)
                | Message::EditSetup(_)
                | Message::ClickOp(_)
                | Message::SelectOrigin(_)
                | Message::AddOrigin
                | Message::SetupNew
                | Message::SetupDuplicate(_)
                | Message::SetupDelete(_)
                | Message::SetupCommit
                | Message::SetupCancel
                | Message::OpenSample
                | Message::NewProject
                | Message::OpenProject
        ) {
            self.cancel_setup_picks();
        }
        if matches!(
            &message,
            Message::SelectSetup(_)
                | Message::EditSetup(_)
                | Message::SetupNew
                | Message::SetupDuplicate(_)
                | Message::SetupDelete(_)
        ) {
            self.cad_inspecting = false;
            self.cad_editing_op = None;
            self.cad_hover = None;
            self.cad_candidates.clear();
        }
        if matches!(
            &message,
            Message::SelectSetup(_)
                | Message::EditSetup(_)
                | Message::BeginOp(_)
                | Message::SetupDuplicate(_)
                | Message::SetupDelete(_)
        ) {
            self.model_view = None;
            self.pending_simulation = None;
            self.pending_processing_time = None;
        }
        // Any action other than opening a group-popup closes the open one (so a
        // command picked from a popup, or a click elsewhere, dismisses it).
        if !matches!(message, Message::ToggleRibbonGroup(_)) {
            self.open_group = None;
        }
        if matches!(
            &message,
            Message::BeginOp(_)
                | Message::CancelOp
                | Message::ConfirmOp
                | Message::CancelOperationDraft
                | Message::CadEscape
                | Message::ConfirmCadOperation
                | Message::UseDrawingGeometry
                | Message::CancelDrawingGeometry
                | Message::Select(_)
                | Message::ClickOp(_)
                | Message::NewProject
                | Message::OpenSample
        ) {
            self.geometry_additive = false;
            if matches!(
                message,
                Message::CadEscape | Message::CancelOperationDraft | Message::CancelOp
            ) {
                self.geometry_auto_pick = false;
            }
            self.geometry_row_selection = None;
            if self.operation_position_pick.take().is_some()
                && matches!(message, Message::CadEscape)
            {
                self.status = "Position selection cancelled.".into();
                return iced::Task::none();
            }
            if matches!(message, Message::CadEscape) && self.profile_tab_picking {
                self.profile_tab_picking = false;
                return iced::Task::none();
            }
            self.profile_tab_picking = false;
            self.profile_tab_selection = None;
        }
        match message {
            Message::TraverseFocus(backward) => {
                return if backward {
                    iced::widget::operation::focus_previous()
                } else {
                    iced::widget::operation::focus_next()
                };
            }
            Message::PollMcp => return self.poll_mcp(),
            Message::SimulationOption(option) => {
                let continuous = matches!(option, SimulationOption::Tolerance(_));
                option.apply(&mut self.settings.simulation);
                if let Some(player) = &mut self.simulation {
                    player.configure(&self.settings.simulation, self.settings.view.stock_color);
                }
                if !continuous {
                    self.settings.save();
                }
            }
            Message::SimulationTab(tab) => {
                if let Some(player) = &mut self.simulation {
                    player.tab = tab;
                }
            }
            Message::SimulationSection(index) => {
                if let Some(section) = self
                    .simulation
                    .as_mut()
                    .and_then(|p| p.sections.get_mut(index))
                {
                    *section = !*section;
                }
            }
            Message::SimulationAccuracySettled => {
                if let Some(player) = &mut self.simulation {
                    player.configure(&self.settings.simulation, self.settings.view.stock_color);
                }
                self.settings.save();
            }
            Message::SimulationSaveSettings => self.settings.save(),
            Message::SimulationPlayPause => {
                if let Some(player) = &mut self.simulation {
                    player.toggle();
                }
            }
            Message::SimulationToolpathStart => {
                if let Some(player) = &mut self.simulation {
                    player.seek_end(false);
                }
            }
            Message::SimulationPreviousOperation => {
                if let Some(player) = &mut self.simulation {
                    player.seek_operation(false);
                }
            }
            Message::SimulationPreviousPoint => {
                if let Some(player) = &mut self.simulation {
                    player.seek_point(false);
                }
            }
            Message::SimulationNextPoint => {
                if let Some(player) = &mut self.simulation {
                    player.seek_point(true);
                }
            }
            Message::SimulationNextOperation => {
                if let Some(player) = &mut self.simulation {
                    player.seek_operation(true);
                }
            }
            Message::SimulationToolpathEnd => {
                if let Some(player) = &mut self.simulation {
                    player.seek_end(true);
                }
            }
            Message::SimulationSpeed(speed) => {
                if let Some(player) = &mut self.simulation {
                    player.speed = speed.clamp(0.25, 10.);
                }
            }
            Message::SimulationSeek(seconds) => {
                if let Some(player) = &mut self.simulation {
                    player.seek(seconds);
                }
            }
            Message::SimulationTick => {
                if let Some(player) = &mut self.simulation {
                    player.sync_force_material(&self.library);
                    player.tick();
                }
            }
            Message::SimulationClose => {
                self.simulation = None;
                self.pending_simulation = None;
            }
            Message::ToolpathCommand(node, action) => return self.toolpath_command(node, action),
            Message::ProcessingTimeFeedOverride(value) => {
                self.processing_time_feed_override_changed(value);
            }
            Message::ProcessingTimeClose => {
                self.processing_time_dialog = None;
            }
            Message::RunJob => {
                if self.setup_draft_dirty()
                    || self.inspector_dirty()
                    || self.controller.pending_op().is_some()
                    || self.cad_pending_kind.is_some()
                    || self.cad_editing_op.is_some()
                {
                    self.status =
                        "OK or Cancel inspector changes and operation selection before running."
                            .into();
                } else {
                    self.rerun_all();
                }
            }
            Message::CancelToolDraft => {
                self.reload_tool_edit();
                self.status = "Tool changes cancelled.".into();
            }
            Message::OperationSectionToggled(section) => {
                if !self.collapsed_operation_sections.remove(&section) {
                    self.collapsed_operation_sections.insert(section);
                    if section.tab == OperationTab::Linking && section.label == "Positions" {
                        self.operation_position_pick = None;
                    }
                    if section.tab == OperationTab::Geometry && section.label == "Geometry" {
                        self.geometry_auto_pick = false;
                        self.geometry_additive = false;
                    }
                    if section.tab == OperationTab::Geometry && section.label == "Holding tabs" {
                        self.profile_tab_picking = false;
                    }
                }
            }
            Message::OperationTabChanged(tab) => {
                self.operation_tab = tab;
                self.operation_position_pick = None;
                self.profile_tab_picking = false;
                self.height_reference_pick = None;
                self.focused_height = None;
                self.geometry_auto_pick = tab == OperationTab::Geometry
                    && self.operation_uses_cad_geometry()
                    && self.cad_pending_kind.is_none()
                    && self.cad_editing_op.is_none();
                if tab == OperationTab::Geometry && self.operation_uses_cad_geometry() {
                    self.geometry_additive =
                        self.cad_pending_kind.is_some() || self.cad_editing_op.is_some();
                    self.cad_filter_explicit = false;
                } else if tab != OperationTab::Geometry {
                    self.geometry_additive = false;
                }
            }
            Message::ProfileTabsEnabled(enabled) => {
                self.profile_tab_picking = false;
                self.profile_tab_selection = None;
                let values = [Field::TabWidth, Field::TabHeight, Field::TabSpacing]
                    .into_iter()
                    .filter_map(|field| {
                        self.fields
                            .get(&field)
                            .and_then(|s| s.parse::<f64>().ok())
                            .filter(|v| v.is_finite())
                            .map(|value| (field, value))
                    })
                    .collect();
                self.edit_operation_draft(|op| {
                    apply_op_fields(op, &values);
                    if let Operation::Profile(profile) = op {
                        profile.tabs.enabled = enabled;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ProfileTabPositioning(mode) => {
                self.profile_tab_picking = false;
                self.profile_tab_selection = None;
                let spacing = self
                    .fields
                    .get(&Field::TabSpacing)
                    .and_then(|s| s.parse::<f64>().ok())
                    .filter(|v| v.is_finite());
                self.edit_operation_draft(|op| {
                    if let Operation::Profile(p) = op {
                        p.tabs.manual = mode == profile_tabs::Positioning::Manual;
                        p.tabs.count = if mode == profile_tabs::Positioning::ByCount {
                            Some(p.tabs.count.unwrap_or(4))
                        } else {
                            None
                        };
                        if let Some(spacing) = spacing {
                            p.tabs.spacing = spacing;
                        }
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ProfileTabPickToggle => {
                if self
                    .profile_tab_operation()
                    .is_some_and(|p| p.tabs.enabled && p.tabs.manual)
                {
                    self.profile_tab_picking = !self.profile_tab_picking;
                    self.geometry_additive = false;
                    self.height_reference_pick = None;
                }
            }
            Message::ProfileTabPicked(point) => {
                if self.profile_tab_picking && point.iter().all(|v| v.is_finite()) {
                    self.edit_operation_draft(|op| {
                        if let Operation::Profile(p) = op {
                            if p.tabs.enabled && p.tabs.manual && !p.tabs.positions.contains(&point)
                            {
                                p.tabs.positions.push(point);
                            }
                        }
                    });
                }
            }
            Message::ProfileTabSelected(index) => {
                self.profile_tab_selection =
                    (index < self.profile_tab_locations().len()).then_some(index);
                if self.profile_tab_selection.is_some() {
                    self.operation_tab = OperationTab::Geometry;
                    self.height_reference_pick = None;
                    self.focused_height = None;
                    return iced::widget::operation::scroll_to(
                        "profile-tab-locations",
                        iced::widget::operation::AbsoluteOffset {
                            x: None,
                            y: Some((index as f32 * 35. - 48.).max(0.)),
                        },
                    );
                }
            }
            Message::ProfileTabRemove(index) => {
                self.profile_tab_selection = self.profile_tab_selection.and_then(|selected| {
                    if selected == index {
                        None
                    } else {
                        Some(if selected > index {
                            selected - 1
                        } else {
                            selected
                        })
                    }
                });
                let mut positions = self
                    .profile_tab_operation()
                    .map(|op| {
                        if op.tabs.manual {
                            op.tabs.positions
                        } else {
                            // Keep the precise footprints around offset corners when
                            // converting automatic placements to editable manual tabs.
                            self.profile_tab_diameter(&op)
                                .map(|diameter| {
                                    cam_toolpath::profile_tab_preview(
                                        &op,
                                        diameter,
                                        cam_cldata::SpindleDir::Cw,
                                    )
                                    .into_iter()
                                    .map(|tab| tab.center)
                                    .collect()
                                })
                                .unwrap_or_default()
                        }
                    })
                    .unwrap_or_default();
                if index < positions.len() {
                    positions.remove(index);
                    self.edit_operation_draft(|op| {
                        if let Operation::Profile(p) = op {
                            p.tabs.manual = true;
                            p.tabs.positions = positions;
                        }
                    });
                    self.refresh_operation_fields();
                }
            }
            Message::ProfileTabsClear => {
                self.profile_tab_selection = None;
                self.edit_operation_draft(|op| {
                    if let Operation::Profile(p) = op {
                        p.tabs.positions.clear();
                    }
                });
            }
            Message::PocketAdaptiveToggled(enabled) => {
                let parsed = self
                    .fields
                    .iter()
                    .filter(|(&field, _)| field != Field::PocketStepover)
                    .filter_map(|(f, s)| {
                        s.parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .map(|v| (*f, v))
                    })
                    .collect();
                let mut seed = false;
                self.edit_operation_draft(|op| {
                    apply_op_fields(op, &parsed);
                    if let Operation::Pocket(p) = op {
                        seed = enabled
                            && p.clear.clearing.engagement <= 0.0
                            && !p
                                .parameter_expressions
                                .contains_key(&cam_model::OperationParameter::Engagement);
                        p.clear.adaptive_enabled = Some(enabled);
                    }
                    if seed {
                        cam_model::OperationParameter::Engagement
                            .set_source(op, creation::DEFAULT_POCKET_ENGAGEMENT);
                    }
                });
                self.refresh_operation_fields();
                if seed {
                    self.fields.insert(
                        Field::Engagement,
                        creation::DEFAULT_POCKET_ENGAGEMENT.into(),
                    );
                }
            }
            Message::PocketDepthToggle(toggle, value) => {
                let parsed = self
                    .fields
                    .iter()
                    .filter(|(&field, _)| field != Field::PocketStepover)
                    .filter_map(|(f, s)| {
                        s.parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .map(|v| (*f, v))
                    })
                    .collect();
                self.edit_operation_draft(|op| {
                    apply_op_fields(op, &parsed);
                    match op {
                        Operation::Pocket(p) => toggle.set(&mut p.clear.multiple_depths, value),
                        Operation::Face(f) => toggle.set(&mut f.multiple_depths, value),
                        _ => {}
                    }
                });
                self.refresh_operation_fields();
            }
            Message::PocketFinishToggle(toggle, value) => {
                let parsed = self
                    .fields
                    .iter()
                    .filter(|(&field, _)| field != Field::PocketStepover)
                    .filter_map(|(f, s)| {
                        s.parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .map(|v| (*f, v))
                    })
                    .collect();
                self.edit_operation_draft(|op| {
                    apply_op_fields(op, &parsed);
                    if let Operation::Pocket(p) = op {
                        toggle.set(&mut p.finishing, value);
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ContourRoughingToggled(enabled) => {
                let parsed = self
                    .fields
                    .iter()
                    .filter_map(|(f, s)| {
                        s.parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .map(|v| (*f, v))
                    })
                    .collect();
                self.edit_operation_draft(|op| {
                    apply_op_fields(op, &parsed);
                    match op {
                        Operation::Profile(p) => p.passes.contour_roughing.enabled = enabled,
                        Operation::Chamfer(c) => c.contour_roughing.enabled = enabled,
                        _ => {}
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ProfilePassToggle(toggle, value) => {
                let parsed = self
                    .fields
                    .iter()
                    .filter_map(|(f, s)| {
                        s.parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .map(|v| (*f, v))
                    })
                    .collect();
                self.edit_operation_draft(|op| {
                    apply_op_fields(op, &parsed);
                    if let Operation::Profile(p) = op {
                        toggle.set(p, value);
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ProfileStockBoundary(choice) => {
                let boundary = match choice {
                    profile_parameters::StockChoice::Setup => None,
                    profile_parameters::StockChoice::Drawing(index) => self
                        .controller
                        .regions()
                        .get(index)
                        .map(|p| p.outer().clone()),
                    profile_parameters::StockChoice::Saved => return iced::Task::none(),
                };
                self.edit_operation_draft(|op| {
                    if let Operation::Profile(p) = op {
                        p.passes.stock_boundary = boundary;
                    }
                });
            }
            Message::ProfileCorner(corner) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Profile(p) = op {
                        p.passes.outer_corner = corner;
                    }
                });
            }
            Message::ProfileTabShape(shape) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Profile(p) = op {
                        p.tabs.shape = shape;
                    }
                });
            }
            Message::ProfileCompChanged(comp) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Profile(profile) = op {
                        profile.comp = comp;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::EngraveClosedChanged(closed) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Engrave(engrave) = op {
                        engrave.closed = closed;
                        if !closed {
                            engrave.start = None;
                            engrave.end = None;
                        }
                    }
                });
                self.refresh_operation_fields();
            }
            Message::Rotary(action) => self.rotary_action(action),
            Message::StockToLeaveToggled(enabled) => {
                let mut stock = self.stock_to_leave_with_fields();
                if enabled
                    && (!self.stock_to_leave_initialized
                        || (self.stock_to_leave_draft.is_none() && !stock.enabled))
                {
                    stock.radial = if matches!(self.operation_draft, Some(Operation::Face(_))) {
                        0.0
                    } else {
                        0.5
                    };
                    stock.axial = 0.5;
                    self.stock_to_leave_initialized = true;
                }
                stock.enabled = enabled;
                self.stock_to_leave_draft = Some(stock);
                self.refresh_operation_fields();
            }
            Message::OperationPositionPick(kind) => self.begin_position_pick(kind),
            Message::OperationPositionClear(kind) => self.clear_operation_position(kind),
            Message::OperationPositionPicked(point) => self.pick_operation_surface_position(point),
            Message::OpenOperationToolSelector => self.open_operation_tool_selector(),
            Message::CloseOperationToolSelector => self.close_operation_tool_selector(),
            Message::OperationProjectToolPicked(tool) => {
                return self.choose_operation_tool(tool, true);
            }
            Message::OperationToolPicked(index) => {
                if let Some(tool) = self.library.tools.get(index).cloned() {
                    return self.choose_operation_tool(tool, false);
                }
            }
            Message::EditDrawingGeometry(id) => {
                if self.current_operation(id).is_some()
                    && self.controller.reinitialize_operation(id)
                {
                    let number = self.current_operation(id).unwrap().tool();
                    self.controller.set_pending_tool(number);
                    self.drawing_editing_op = Some(id);
                    self.status = "Pick the drawing geometry. Ctrl/Command adds compatible features. Use geometry stages it for OK.".into();
                }
            }
            Message::UseDrawingGeometry => {
                if let Some(id) = self.drawing_editing_op {
                    if let Some(operation) = self.current_operation(id) {
                        match self.controller.operation_with_drawing_geometry(
                            operation,
                            &self.controller.pending_operation_geometry(),
                        ) {
                            Ok(draft) => {
                                self.operation_draft = Some(draft);
                                self.controller.cancel_operation();
                                self.drawing_editing_op = None;
                                self.refresh_operation_fields();
                                self.operation_tab = OperationTab::Geometry;
                                self.status = "Drawing geometry staged. OK commits all tabs; Cancel discards them.".into();
                            }
                            Err(error) => self.report_error(error),
                        }
                    }
                }
            }
            Message::CancelDrawingGeometry => {
                self.controller.cancel_operation();
                self.drawing_editing_op = None;
                self.status = "Geometry selection cancelled; other draft edits retained.".into();
            }
            Message::MoveOpen(node) => self.open_move(node),
            Message::MoveValue(index, value) => {
                if let Some(draft) = &mut self.move_draft {
                    draft.values[index] = value;
                    draft.clear_handle();
                }
            }
            Message::MoveHandleStart(index, base, direction) => {
                self.start_move_handle(index, base, direction)
            }
            Message::MoveHandleDelta(value) => self.set_move_handle(value, format!("{value:.2}")),
            Message::MoveHandleInput(value) => {
                if let Ok(number) = value.trim().parse::<f64>() {
                    if number.is_finite() {
                        self.set_move_handle(number, value.clone());
                    }
                }
                self.set_move_handle_text(value);
            }
            Message::MoveCancel => self.cancel_move(),
            Message::MoveApply => self.apply_move(),
            Message::Tree(message) => return self.update_tree(message),
            Message::ShowStatus(target) => {
                let fingerprints = self.controller.current_toolpath_fingerprints();
                if let Some(notice) = self.status_notice(target, &fingerprints) {
                    return self.queue_notice(notice);
                }
            }
            Message::OpenSample => {
                self.pending_processing_time = None;
                match self.controller.open_dxf(SAMPLE_DXF, "sample.dxf") {
                    Ok(n) => {
                        self.status_messages = StatusMessages::default();
                        self.reset_model_browser();
                        self.planar_selection.clear();
                        self.focus_ops.clear();
                        self.controller.select(Selection::Setup);
                        self.end_setup_draft();
                        self.refresh_fields();
                        self.status = format!("Imported {n} region(s).");
                        self.rerun();
                    }
                    Err(e) => self.status = format!("Import failed: {e}"),
                }
            }
            Message::SelectSetup(id) => {
                if self.pattern_draft.is_some() {
                    self.status =
                        "Apply or cancel the pattern draft before switching setups.".into();
                    return iced::Task::none();
                }
                if self.setup_draft_dirty() {
                    self.status = "Apply or cancel setup changes before switching setups.".into();
                } else if let Err(error) = self.controller.activate_setup(id) {
                    self.status = error;
                } else {
                    self.expanded_setups.insert(id);
                    self.cad_inspecting = false;
                    self.cad_editing_op = None;
                    self.controller.select(Selection::Setup);
                    self.operation_draft = None;
                    self.end_setup_draft();
                    self.focus_ops.clear();
                    self.refresh_fields();
                }
            }
            Message::EditSetup(id) => {
                let _ = self.update(Message::SelectSetup(id));
                if self.controller.active_setup_id() == id {
                    self.begin_setup_draft();
                }
            }
            Message::ToggleAssetExpanded(id) => {
                if !self.collapsed_assets.remove(&id) {
                    self.collapsed_assets.insert(id);
                }
            }
            Message::ToggleOperationExpanded(id) => {
                if !self.expanded_operations.remove(&id) {
                    self.expanded_operations.insert(id);
                }
            }
            Message::ToggleToolsExpanded => self.tools_expanded = !self.tools_expanded,
            Message::SetAssetVisible(id, visible) => {
                if visible {
                    self.hidden_assets.remove(&id);
                } else {
                    self.hidden_assets.insert(id);
                }
                self.cad_hover = None;
                self.cad_candidates.clear();
            }
            Message::ToggleSetupExpanded(id) => {
                if !self.expanded_setups.remove(&id) {
                    self.expanded_setups.insert(id);
                }
            }
            Message::SetSetupVisible(id, visible) => {
                if visible {
                    self.hidden_setups.remove(&id);
                } else {
                    self.hidden_setups.insert(id);
                }
            }
            Message::SetOperationVisible(id, visible) => {
                if visible {
                    self.hidden_ops.remove(&id);
                } else {
                    self.hidden_ops.insert(id);
                }
            }
            Message::SetupTabChanged(tab) => {
                if tab != SetupEditorTab::Setup {
                    self.cancel_setup_picks();
                }
                self.setup_tab = tab;
            }
            Message::SetupNameChanged(value) => {
                if let Some(draft) = &mut self.setup_draft {
                    draft.name = value;
                }
            }
            Message::SetupProgramChanged(value) => {
                if let Some(draft) = &mut self.setup_draft {
                    draft.program_name = value;
                }
            }
            Message::SetupOrientationChanged(mode) => self.change_setup_orientation(mode),
            Message::SetupOriginModeChanged(mode) => self.change_setup_origin_mode(mode),
            Message::SetupUseModelFrame(instance) => {
                match self.controller.pending_frame_from_model(instance) {
                    Ok((mut frame, model_origin)) => {
                        let mode = self.setup_origin_mode();
                        self.setup_orientation = Some(SetupOrientation::Model);
                        self.clear_setup_origin_buffers();
                        self.cancel_setup_picks();
                        if let Some(draft) = &mut self.setup_draft {
                            let origin = if mode == SetupOriginMode::Model {
                                model_origin
                            } else {
                                frame.to_setup_vector(draft.frame.to_world_vector(draft.origin))
                            };
                            frame.references.origin = if mode == SetupOriginMode::Model {
                                cam_model::FrameOrigin::ModelOrigin
                            } else if matches!(
                                draft.frame.references.origin,
                                cam_model::FrameOrigin::ModelOrigin
                            ) {
                                cam_model::FrameOrigin::Explicit(origin)
                            } else {
                                draft.frame.references.origin.with_coordinates(origin)
                            };
                            draft.frame = frame;
                            draft.origin = origin;
                        }
                    }
                    Err(error) => self.report_error(error),
                }
            }
            Message::SetupUseModelOrigin(instance) => {
                let model_origin = self
                    .controller
                    .geometry_state()
                    .instance(instance)
                    .map(|model| model.placement.translation);
                if let Some(model_origin) = model_origin {
                    self.clear_setup_origin_buffers();
                    self.cancel_setup_picks();
                    if let Some(draft) = &mut self.setup_draft {
                        draft.origin = draft.frame.to_setup_vector(model_origin);
                        draft.frame.references.origin = cam_model::FrameOrigin::ModelOrigin;
                    }
                }
            }
            Message::SetupOriginChanged(index, value) => {
                self.setup_origin_mode = Some(SetupOriginMode::Coordinates);
                self.setup_values.insert(index, value.clone());
                if let (Some(draft), Ok(v)) = (&mut self.setup_draft, value.parse::<f64>()) {
                    if index < 3 {
                        draft.origin[index] = v;
                        draft.frame.references.origin =
                            cam_model::FrameOrigin::Explicit(draft.origin);
                    }
                }
            }
            Message::SetupWorkOffsetChanged(value) => {
                self.setup_values.insert(40, value.clone());
                if let (Some(draft), Ok(v)) = (&mut self.setup_draft, value.parse::<u32>()) {
                    draft.origin_index = v;
                }
            }
            Message::SetupToolChangeAutomatic(automatic) => {
                self.setup_values.remove(&41);
                if let Some(draft) = &mut self.setup_draft {
                    draft.tool_change_height =
                        if automatic {
                            None
                        } else {
                            Some(draft.resolved_tool_change_height(
                                self.controller.machine().envelope.max.z,
                            ))
                        };
                }
            }
            Message::SetupToolChangeHeightChanged(value) => {
                self.setup_values.insert(41, value.clone());
                if let (Some(draft), Ok(v)) = (&mut self.setup_draft, value.trim().parse::<f64>()) {
                    if v.is_finite() {
                        draft.tool_change_height = Some(v + draft.origin[2]);
                    }
                }
            }
            Message::SetupModelPick => {
                if self.setup_draft.is_some() {
                    let picking = !self.setup_model_pick || self.setup_fixture_pick;
                    self.cancel_setup_picks();
                    self.setup_model_pick = picking;
                    self.cad_hover = None;
                    self.cad_candidates.clear();
                    self.status = if picking {
                        "Click bodies to select or deselect setup models. Click the selection button when finished."
                    } else { "Setup body selection finished." }.into();
                }
            }
            Message::SetupFixturePick => {
                if self.setup_draft.is_some() {
                    let picking = !(self.setup_model_pick && self.setup_fixture_pick);
                    self.cancel_setup_picks();
                    self.setup_model_pick = picking;
                    self.setup_fixture_pick = picking;
                    self.cad_hover = None;
                    self.cad_candidates.clear();
                    self.status = if picking {
                        "Click bodies to select or deselect fixtures. Click the selection button when finished."
                    } else {
                        "Fixture selection finished."
                    }
                    .into();
                }
            }
            Message::SetupGeometryClear(fixture) => {
                if let Some(draft) = &mut self.setup_draft {
                    if fixture {
                        draft.fixture_instances.clear();
                        self.setup_fixture_selection = None;
                    } else {
                        draft.model_instances.clear();
                        self.setup_model_selection = None;
                    }
                }
            }
            Message::SetupFixtureToggled(instance, included) => {
                if included
                    && self
                        .controller
                        .geometry_state()
                        .instance(instance)
                        .is_none()
                {
                    return iced::Task::none();
                }
                if let Some(draft) = &mut self.setup_draft {
                    if included {
                        if !draft.fixture_instances.contains(&instance) {
                            draft.fixture_instances.push(instance);
                        }
                        draft.model_instances.retain(|id| *id != instance);
                        if self.setup_model_selection == Some(instance) {
                            self.setup_model_selection = None;
                        }
                        self.setup_fixture_selection = Some(instance);
                        if let Some(body) = self.controller.geometry_state().instance(instance) {
                            self.collapsed_assets.remove(&body.asset_id);
                        }
                    } else {
                        draft.fixture_instances.retain(|id| *id != instance);
                        if self.setup_fixture_selection == Some(instance) {
                            self.setup_fixture_selection = draft.fixture_instances.last().copied();
                        }
                    }
                }
            }
            Message::SetupFixtureClearanceChanged(axis, value) => {
                if axis < 2 {
                    self.setup_values.insert(42 + axis, value.clone());
                    if let (Some(draft), Ok(v)) =
                        (&mut self.setup_draft, value.trim().parse::<f64>())
                    {
                        if v.is_finite() {
                            if axis == 0 {
                                draft.fixture_clearance.radial = v;
                            } else {
                                draft.fixture_clearance.axial = v;
                            }
                        }
                    }
                }
            }
            Message::SetupModelToggled(instance, included) => {
                if self
                    .controller
                    .geometry_state()
                    .instance(instance)
                    .is_none()
                {
                    return iced::Task::none();
                }
                if let Some(draft) = &mut self.setup_draft {
                    if included {
                        if !draft.model_instances.contains(&instance) {
                            draft.model_instances.push(instance);
                        }
                        self.setup_model_selection = Some(instance);
                        draft.fixture_instances.retain(|id| *id != instance);
                        if self.setup_fixture_selection == Some(instance) {
                            self.setup_fixture_selection = None;
                        }
                        if let Some(body) = self.controller.geometry_state().instance(instance) {
                            self.collapsed_assets.remove(&body.asset_id);
                        }
                    } else {
                        draft.model_instances.retain(|id| *id != instance);
                        if self.setup_model_selection == Some(instance) {
                            self.setup_model_selection = draft.model_instances.last().copied();
                        }
                    }
                }
            }
            Message::SetupStockKindChanged(kind) => {
                if self.setup_axis_pick == Some(3) {
                    self.setup_axis_pick = None;
                    self.cad_candidates.clear();
                }
                if let Some(draft) = &mut self.setup_draft {
                    let stock_axis = match draft.stock {
                        cam_model::Stock::FixedRound { axis, .. }
                        | cam_model::Stock::RelativeRound { axis, .. } => axis,
                        _ => None,
                    };
                    let stock = match kind {
                        StockEditorKind::PreviousSetup => cam_model::Stock::PreviousSetup,
                        StockEditorKind::FixedBox => match draft.stock {
                            cam_model::Stock::FixedBox { .. } => draft.stock,
                            _ => cam_model::Stock::FixedBox {
                                size: [1.0; 3],
                                model_position: [0.5; 3],
                            },
                        },
                        StockEditorKind::RelativeBox => match draft.stock {
                            cam_model::Stock::RelativeBox { .. } => draft.stock,
                            _ => cam_model::Stock::RelativeBox {
                                offsets: [[1.0; 3]; 2],
                            },
                        },
                        StockEditorKind::FromSolid => match draft.stock {
                            cam_model::Stock::FromSolid { .. } => draft.stock,
                            _ => cam_model::Stock::FromSolid { instance: None },
                        },
                        StockEditorKind::FixedCylinder | StockEditorKind::FixedTube => {
                            let tube = kind == StockEditorKind::FixedTube;
                            if let cam_model::Stock::FixedRound { tube: existing, .. } = draft.stock
                            {
                                if existing == tube {
                                    return iced::Task::none();
                                }
                            }
                            let mut oriented = draft.clone();
                            let round = self
                                .controller
                                .document()
                                .round_stock_geometry(draft)
                                .ok()
                                .flatten();
                            if let Some((frame, _, _)) = &round {
                                oriented.frame = frame.clone();
                            }
                            let (min, max) = self.controller.pending_setup_model_bounds(&oriented);
                            let diameter = round
                                .as_ref()
                                .map(|(_, lo, hi)| hi[0] - lo[0])
                                .unwrap_or_else(|| {
                                    (max[0] - min[0]).hypot(max[1] - min[1]).max(1.) + 2.
                                });
                            let length = round
                                .as_ref()
                                .map(|(_, lo, hi)| hi[2] - lo[2])
                                .unwrap_or_else(|| (max[2] - min[2]).max(1.) + 2.);
                            cam_model::Stock::FixedRound {
                                diameter,
                                inner_diameter: if tube { diameter / 2. } else { 0. },
                                length,
                                position: 0.5,
                                tube,
                                axis: stock_axis,
                            }
                        }
                        StockEditorKind::RelativeCylinder | StockEditorKind::RelativeTube => {
                            let tube = kind == StockEditorKind::RelativeTube;
                            if let cam_model::Stock::RelativeRound { tube: existing, .. } =
                                draft.stock
                            {
                                if existing == tube {
                                    return iced::Task::none();
                                }
                            }
                            cam_model::Stock::RelativeRound {
                                radial: 1.,
                                inner_diameter: if tube { 1. } else { 0. },
                                bottom: 1.,
                                top: 1.,
                                tube,
                                axis: stock_axis,
                            }
                        }
                        StockEditorKind::LegacyBox => match draft.stock {
                            cam_model::Stock::BoundingBox { .. } => draft.stock,
                            _ => draft.stock,
                        },
                    };
                    let changed = draft.stock != stock;
                    draft.stock = stock;
                    if changed {
                        self.setup_values
                            .retain(|key, _| !((20..=25).contains(key) || (70..=73).contains(key)));
                    }
                }
            }
            Message::SetupStockOriginAxis(index) => {
                if let (Some(draft), Some(origin_axis)) = (
                    &mut self.setup_draft,
                    cam_model::StockOriginAxis::from_index(index),
                ) {
                    match &mut draft.stock {
                        cam_model::Stock::FixedRound { axis, .. }
                        | cam_model::Stock::RelativeRound { axis, .. } => {
                            *axis = Some(cam_model::StockAxis::Origin { origin_axis });
                            self.status =
                                format!("Stock axis set to origin {}.", ["X", "Y", "Z"][index]);
                        }
                        _ => {}
                    }
                }
                self.setup_axis_pick = None;
                self.cad_candidates.clear();
            }
            Message::SetupStockAxisClear => {
                if let Some(draft) = &mut self.setup_draft {
                    match &mut draft.stock {
                        cam_model::Stock::FixedRound { axis, .. }
                        | cam_model::Stock::RelativeRound { axis, .. } => *axis = None,
                        _ => {}
                    }
                }
                if self.setup_axis_pick == Some(3) {
                    self.setup_axis_pick = None;
                }
                self.cad_candidates.clear();
            }
            Message::SetupStockSolidChanged(choice) => {
                if let Some(draft) = &mut self.setup_draft {
                    draft.stock = cam_model::Stock::FromSolid {
                        instance: Some(choice.id),
                    };
                }
            }
            Message::SetupStockNumberChanged(index, value) => {
                self.setup_values.insert(20 + index, value.clone());
                if let (Some(draft), Ok(v)) = (&mut self.setup_draft, value.parse::<f64>()) {
                    match &mut draft.stock {
                        cam_model::Stock::PreviousSetup | cam_model::Stock::FromSolid { .. } => {}
                        cam_model::Stock::FixedBox {
                            size,
                            model_position,
                        } => {
                            if index < 3 {
                                size[index] = v;
                            } else if index < 4 {
                                model_position[index - 3] = v;
                            }
                        }
                        cam_model::Stock::RelativeBox { offsets } => {
                            if index < 6 {
                                offsets[index / 3][index % 3] = v;
                            }
                        }
                        cam_model::Stock::FixedRound {
                            diameter,
                            inner_diameter,
                            length,
                            position,
                            ..
                        } => match index {
                            0 => *diameter = v,
                            1 => *inner_diameter = v,
                            2 => *length = v,
                            3 => *position = v,
                            _ => {}
                        },
                        cam_model::Stock::RelativeRound {
                            radial,
                            inner_diameter,
                            bottom,
                            top,
                            ..
                        } => match index {
                            0 => *radial = v,
                            1 => *inner_diameter = v,
                            2 => *bottom = v,
                            3 => *top = v,
                            _ => {}
                        },
                        cam_model::Stock::BoundingBox {
                            x_offset,
                            y_offset,
                            top,
                            thickness,
                        } => match index {
                            50 => *x_offset = v,
                            51 => *y_offset = v,
                            52 => *top = v,
                            53 => *thickness = v,
                            _ => {}
                        },
                    }
                }
            }
            Message::SetupStockPositionChanged(axis, anchor) => {
                if let Some(cam_model::Stock::FixedBox { model_position, .. }) =
                    self.setup_draft.as_mut().map(|draft| &mut draft.stock)
                {
                    if axis < 3 {
                        model_position[axis] = anchor.value();
                    }
                }
            }
            Message::SetupAxisPick(axis) => {
                self.setting_origin = false;
                self.setting_origin_2pt = false;
                self.origin_first = None;
                self.setup_hover = SetupHover::default();
                self.setup_model_pick = false;
                self.setup_axis_pick = Some(axis);
                self.cad_candidates.clear();
                self.cad_filter = CadFilter::Edge;
                self.status = if axis == 3 {
                    "Select a straight model edge or an origin axis for the stock.".into()
                } else {
                    format!(
                        "Select a straight model or stock edge for {}.",
                        ["X", "Y", "Z"][axis]
                    )
                };
            }
            Message::PatternTabChanged(tab) => self.pattern_tab = tab,
            Message::BeginPattern(circular) => {
                if self.setup_draft_dirty() {
                    self.status = "Apply or cancel setup changes before editing a pattern.".into();
                    return iced::Task::none();
                }
                self.end_setup_draft();
                self.simulation = None;
                self.pattern_tab = pattern_editor_view::PatternTab::Placement;
                let Some(setup) = self
                    .controller
                    .setups()
                    .iter()
                    .find(|s| s.id == self.controller.active_setup_id())
                else {
                    return iced::Task::none();
                };
                let members: Vec<u32> = if self.focus_ops.is_empty() {
                    Vec::new()
                } else {
                    self.focus_ops.iter().copied().collect()
                };
                let origin = setup.origin_position(self.controller.active_origin());
                self.pattern_draft = Some(PatternDraft {
                    setup_id: setup.id,
                    pattern_id: None,
                    name: "Pattern".into(),
                    members,
                    circular,
                    order: cam_model::PatternOrder::ByInstance,
                    clockwise: false,
                    options: cam_model::PatternOptions {
                        spacing: if circular {
                            cam_model::PatternSpacing::Extent
                        } else {
                            cam_model::PatternSpacing::Spacing
                        },
                        second_direction: false,
                        ..Default::default()
                    },
                    picking: None,
                    highlighted: None,
                    values: if circular {
                        [
                            "4".into(),
                            origin[0].to_string(),
                            origin[1].to_string(),
                            "360".into(),
                        ]
                    } else {
                        ["2".into(), "2".into(), "20".into(), "20".into()]
                    },
                });
                self.controller.select(Selection::Setup);
                self.inspector_active = true;
                self.inspector_enabled = true;
                self.status = "Choose source operations and pattern placement, then create or cancel the draft.".into();
            }
            Message::EditPattern(id) => {
                if self.setup_draft_dirty() {
                    self.status = "Apply or cancel setup changes before editing a pattern.".into();
                    return iced::Task::none();
                }
                self.end_setup_draft();
                self.simulation = None;
                self.pattern_tab = pattern_editor_view::PatternTab::Placement;
                let Some((setup_id, pattern)) = self.controller.setups().iter().find_map(|setup| {
                    setup
                        .patterns
                        .iter()
                        .find(|p| p.id == id)
                        .map(|p| (setup.id, p.clone()))
                }) else {
                    self.status = "Pattern no longer exists.".into();
                    return iced::Task::none();
                };
                let (circular, clockwise, values) = match pattern.kind {
                    cam_model::PatternKind::Rectangular { nx, ny, dx, dy } => (
                        false,
                        false,
                        [
                            nx.to_string(),
                            ny.to_string(),
                            dx.to_string(),
                            dy.to_string(),
                        ],
                    ),
                    cam_model::PatternKind::Circular {
                        center_xy,
                        count,
                        distribution,
                        direction,
                    } => {
                        let sweep = match distribution {
                            cam_model::CircularDistribution::Full => 360.0,
                            cam_model::CircularDistribution::Partial { sweep_degrees } => {
                                sweep_degrees
                            }
                            cam_model::CircularDistribution::Spacing { angle_degrees } => {
                                angle_degrees
                            }
                        };
                        (
                            true,
                            direction == cam_model::PatternDirection::Cw,
                            [
                                count.to_string(),
                                center_xy[0].to_string(),
                                center_xy[1].to_string(),
                                sweep.to_string(),
                            ],
                        )
                    }
                };
                let mut options = pattern.options;
                if let cam_model::PatternKind::Circular { distribution, .. } = pattern.kind {
                    options.spacing = if matches!(
                        distribution,
                        cam_model::CircularDistribution::Spacing { .. }
                    ) {
                        cam_model::PatternSpacing::Spacing
                    } else {
                        cam_model::PatternSpacing::Extent
                    };
                }
                self.pattern_draft = Some(PatternDraft {
                    setup_id,
                    pattern_id: Some(id),
                    name: pattern.name,
                    members: pattern.members,
                    circular,
                    order: pattern.order,
                    clockwise,
                    options,
                    picking: None,
                    highlighted: None,
                    values,
                });
                self.controller.activate_setup(setup_id).ok();
                self.controller.select(Selection::Setup);
                self.inspector_active = true;
                self.inspector_enabled = true;
            }
            Message::PatternName(value) => {
                if let Some(draft) = self.pattern_draft.as_mut() {
                    draft.name = value;
                }
            }
            Message::PatternValue(index, value) => {
                if let Some(draft) = self.pattern_draft.as_mut() {
                    if index < 4 {
                        draft.values[index] = value;
                        if draft.circular && (index == 1 || index == 2) {
                            draft.options.axis = None;
                        }
                    }
                }
            }
            Message::PatternMember(id, checked) => {
                if let Some(draft) = self.pattern_draft.as_mut() {
                    if checked {
                        if !draft.members.contains(&id) {
                            draft.members.push(id);
                        }
                    } else {
                        draft.members.retain(|member| *member != id);
                    }
                }
            }
            Message::PatternOrderChanged(order) => {
                if let Some(draft) = self.pattern_draft.as_mut() {
                    draft.order = order;
                }
            }
            Message::PatternTypeChanged(circular) => self.change_pattern_type(circular),
            Message::PatternSpacingChanged(spacing) => {
                if let Some(draft) = &mut self.pattern_draft {
                    draft.options.spacing = spacing;
                }
            }
            Message::PatternSecondDirection(enabled) => {
                if let Some(draft) = &mut self.pattern_draft {
                    draft.options.second_direction = enabled;
                }
            }
            Message::PatternKeepOriginal(enabled) => {
                if let Some(draft) = &mut self.pattern_draft {
                    draft.options.keep_original = enabled;
                }
            }
            Message::PatternReverse(enabled) => {
                if let Some(draft) = &mut self.pattern_draft {
                    draft.options.reverse = enabled;
                }
            }
            Message::PatternAxisReversed(axis, enabled) => {
                if let Some(draft) = &mut self.pattern_draft {
                    if axis < 2 {
                        draft.options.directions[axis].reversed = enabled;
                    }
                }
            }
            Message::PatternSymmetric(axis, enabled) => {
                if let Some(draft) = &mut self.pattern_draft {
                    if axis < 2 {
                        draft.options.directions[axis].symmetric = enabled;
                    }
                }
            }
            Message::PatternHighlight(axis) => {
                if let Some(draft) = &mut self.pattern_draft {
                    draft.picking = None;
                    draft.highlighted = Some(axis);
                }
                self.cad_hover = None;
                self.cad_candidates.clear();
                self.status = "Selected pattern edge highlighted.".into();
            }
            Message::PatternPick(axis) => {
                self.cancel_setup_picks();
                if let Some(draft) = &mut self.pattern_draft {
                    draft.picking = Some(axis);
                }
                self.cad_filter = CadFilter::Edge;
                self.cad_candidates.clear();
                self.status = if axis == 2 {
                    "Select a circular edge or a straight axis parallel to setup Z."
                } else {
                    "Select a straight edge in setup XY."
                }
                .into();
            }
            Message::PatternClear(axis) => {
                if let Some(draft) = &mut self.pattern_draft {
                    draft.picking = None;
                    if draft.highlighted == Some(axis) {
                        draft.highlighted = None;
                    }
                    if axis < 2 {
                        draft.options.directions[axis].reference = None;
                        draft.options.directions[axis].angle_degrees =
                            if axis == 0 { 0.0 } else { 90.0 };
                    } else {
                        draft.options.axis = None;
                    }
                }
            }
            Message::PatternDirectionChanged(clockwise) => {
                if let Some(draft) = self.pattern_draft.as_mut() {
                    draft.clockwise = clockwise;
                }
            }
            Message::TogglePatternExpanded(id) => {
                if !self.expanded_patterns.remove(&id) {
                    self.expanded_patterns.insert(id);
                }
            }
            Message::PatternCancel => {
                self.pattern_draft = None;
                self.status = "Pattern draft cancelled.".into();
            }
            Message::PatternCommit => {
                if let Some(draft) = self.pattern_draft.clone() {
                    let parsed = pattern_editor_view::configuration_from_draft(&draft);
                    match parsed.and_then(|(kind, options)| {
                        if let Some(id) = draft.pattern_id {
                            self.controller.edit_pattern_with_options(
                                draft.setup_id,
                                id,
                                &draft.members,
                                draft.name.clone(),
                                kind,
                                draft.order,
                                options,
                            )?;
                        } else {
                            self.controller.create_pattern_with_options(
                                draft.setup_id,
                                &draft.members,
                                draft.name.clone(),
                                kind,
                                draft.order,
                                options,
                            )?;
                        }
                        Ok(())
                    }) {
                        Ok(_) => {
                            self.pattern_draft = None;
                            self.inspector_active = false;
                            self.status = "Pattern created. Seed operations remain editable inside the group.".into();
                        }
                        Err(reason) => self.status = reason,
                    }
                }
            }
            Message::PatternSuppression(id, suppressed) => {
                if let Some(setup_id) = self
                    .controller
                    .setups()
                    .iter()
                    .find(|s| s.patterns.iter().any(|p| p.id == id))
                    .map(|s| s.id)
                {
                    self.controller
                        .set_pattern_suppressed(setup_id, id, suppressed);
                }
            }
            Message::PatternDissolve(id) => {
                if let Some(setup_id) = self
                    .controller
                    .setups()
                    .iter()
                    .find(|s| s.patterns.iter().any(|p| p.id == id))
                    .map(|s| s.id)
                {
                    self.controller.dissolve_pattern(setup_id, id);
                    self.status = "Pattern dissolved; source operations remain in place.".into();
                }
            }
            Message::PatternDelete(id) => {
                if let Some((setup_id, count)) = self.controller.setups().iter().find_map(|s| {
                    s.patterns
                        .iter()
                        .find(|p| p.id == id)
                        .map(|p| (s.id, p.members.len()))
                }) {
                    self.controller.delete_pattern_and_operations(setup_id, id);
                    self.status = format!("Deleted pattern and {count} source operation(s).");
                }
            }
            Message::SetupAxisFlip(axis, flipped) => self.set_setup_axis_flip(axis, flipped),
            Message::SetupAxisToggle(axis) => {
                if self.setup_axis_pick == Some(3) {
                    return self.update(Message::SetupStockOriginAxis(axis));
                }
                if let Some(draft) = &self.setup_draft {
                    if axis < 3 {
                        let refs = &draft.frame.references;
                        let flipped = [&refs.x, &refs.y, &refs.z][axis]
                            .as_ref()
                            .map_or(refs.stock_flipped[axis], |r| r.flipped);
                        self.set_setup_axis_flip(axis, !flipped);
                        self.cancel_setup_picks();
                    }
                }
            }
            Message::SetupAxisClear(axis) => self.set_setup_axis_reference(axis, None),
            Message::SetupCommit => {
                if self
                    .setup_values
                    .values()
                    .any(|v| !v.trim().parse::<f64>().is_ok_and(|n| n.is_finite()))
                    || self
                        .setup_values
                        .get(&40)
                        .is_some_and(|v| v.trim().parse::<u32>().is_err())
                {
                    self.status = "Correct the invalid setup value before applying.".into();
                    return iced::Task::none();
                }
                if let Some(mut draft) = self.setup_draft.clone() {
                    if let Some(value) = self.setup_values.get(&41) {
                        // Interpret the edited WCS Z against the final draft datum.
                        draft.tool_change_height =
                            Some(value.trim().parse::<f64>().unwrap() + draft.origin[2]);
                    }
                    match self.controller.apply_setup_settings(draft) {
                        Ok(()) => {
                            self.end_setup_draft();
                            self.refresh_fields();
                            self.rerun();
                            self.status = "Setup settings applied.".into();
                        }
                        Err(error) => self.report_error(error),
                    }
                }
            }
            Message::SetupCancel => {
                self.end_setup_draft();
                self.refresh_fields();
                self.status = "Setup changes cancelled.".into();
            }
            Message::SetupNew => {
                if self.setup_draft_dirty() {
                    self.status = "Apply or cancel setup changes before adding a setup.".into();
                } else {
                    let models = self.selected_setup_models();
                    let id = self.controller.add_setup_with_models(models);
                    self.workspace = Workspace::Machining;
                    self.active_tab = RibbonTab::Machining;
                    self.machining_tab = RibbonTab::Machining;
                    self.expanded_setups.insert(id);
                    self.controller.activate_setup(id).ok();
                    self.controller.select(Selection::Setup);
                    self.begin_setup_draft();
                    self.refresh_fields();
                }
            }
            Message::SetupDuplicate(id) => {
                if self.setup_draft_dirty() {
                    self.status = "Apply or cancel setup changes before duplicating.".into();
                } else {
                    match self.controller.duplicate_setup(id) {
                        Ok(new_id) => {
                            self.expanded_setups.insert(new_id);
                            self.controller.activate_setup(new_id).ok();
                            self.controller.select(Selection::Setup);
                            self.end_setup_draft();
                            self.refresh_fields();
                        }
                        Err(error) => self.report_error(error),
                    }
                }
            }
            Message::SetupDelete(id) => {
                if self.setup_draft_dirty() {
                    self.status = "Apply or cancel setup changes before deleting a setup.".into();
                } else if let Err(error) = self.controller.delete_setup(id) {
                    self.status = error;
                } else {
                    self.end_setup_draft();
                    self.refresh_fields();
                }
            }
            Message::Select(selection) => {
                if self.pattern_draft.is_some() {
                    self.status =
                        "Apply or cancel the pattern draft before changing selection.".into();
                    return iced::Task::none();
                }
                if self.setup_draft_dirty() {
                    self.status = "Apply or cancel setup changes before changing selection.".into();
                    return iced::Task::none();
                }
                self.cad_inspecting = false;
                self.cad_editing_op = None;
                self.end_setup_draft();
                self.controller.select(selection);
                self.inspector_active = matches!(selection, Selection::Operation(_));
                if let Selection::Operation(id) = selection {
                    self.inspector_enabled = true;
                    self.begin_operation_draft(id);
                } else {
                    self.operation_draft = None;
                    self.stock_to_leave_draft = None;
                }
                // Explicit Edit commands select the operation and open its editor.
                self.focus_ops.clear();
                if let Selection::Operation(id) = selection {
                    self.focus_ops.insert(id);
                }
                self.refresh_fields();
            }
            Message::ClickOp(id) => {
                if self.pattern_draft.is_some() {
                    self.status =
                        "Apply or cancel the pattern draft before changing selection.".into();
                    return iced::Task::none();
                }
                if self.setup_draft_dirty() {
                    self.status = "Apply or cancel setup changes before changing selection.".into();
                    return iced::Task::none();
                }
                self.cad_inspecting = false;
                self.cad_editing_op = None;
                if self.modifiers.command() {
                    // ⌘/Ctrl-click toggles this op in the highlight set.
                    if !self.focus_ops.remove(&id) {
                        self.focus_ops.insert(id);
                    }
                } else if self.focus_ops.len() == 1 && self.focus_ops.contains(&id) {
                    // Plain-clicking the sole focused op clears the highlight.
                    self.focus_ops.clear();
                } else {
                    // Plain click focuses just this op.
                    self.focus_ops.clear();
                    self.focus_ops.insert(id);
                }
                // Keep the primary selection in the visible set. The controller's
                // setup fallback represents an empty browser selection below.
                let sel = if self.focus_ops.contains(&id) {
                    Selection::Operation(id)
                } else if let Some(&other) = self.focus_ops.iter().next_back() {
                    Selection::Operation(other)
                } else {
                    Selection::Setup
                };
                self.controller.select(sel);
                self.inspector_active = false;
                self.end_setup_draft();
                self.operation_draft = None;
                self.operation_position_markers = [None; 2];
                // Browser selection reads the selected operation's saved allowance.
                // An editor draft from the previous operation must not follow it.
                self.stock_to_leave_draft = None;
                self.operation_tools.clear();
                self.operation_geometry = None;
                self.operation_binding_preview = None;
                if self.focus_ops.is_empty() {
                    self.restore_browser_view();
                } else {
                    // Selecting an operation leaves the model browser and returns
                    // to its machining backplot. Keeping Models active hides every
                    // operation, including the one the user just focused.
                    self.model_view = None;
                }
                self.refresh_fields();
            }
            Message::ModifiersChanged(m) => self.modifiers = m,
            // Field edits only touch the local buffer; nothing is applied or
            // recomputed until Apply, so undo has one step per real change.
            Message::HeightReferenceChanged(plane, reference) => {
                self.height_reference_selected(plane, reference)
            }
            Message::HeightFeedEnabled(enabled) => {
                use cam_model::{HeightPlaneKind as Plane, HeightReference as Reference};
                let reference = if enabled {
                    self.height_disabled_feed_reference
                        .take()
                        .unwrap_or(Reference::Plane(Plane::Top))
                } else {
                    Reference::Disabled
                };
                self.height_reference_selected(Plane::Feed, reference);
                self.collapsed_heights.remove(&Plane::Feed);
            }
            Message::HeightOffsetChanged(plane, value) => {
                self.height_offsets.insert(plane, value);
                self.focused_height = Some(height_editor::plane_input_id(plane));
            }
            Message::HeightSourceToggled(plane, reference, on) => {
                if let Some(settings) = &mut self.height_draft {
                    let sources = &mut settings.definition_mut(plane).sources;
                    sources.retain(|r| *r != reference);
                    if on {
                        sources.push(reference);
                    }
                }
                self.focused_height = Some(height_editor::plane_input_id(plane));
            }
            Message::HeightDefaults => {
                let settings = cam_model::OperationHeightSettings::default();
                self.height_offsets = cam_model::HeightPlaneKind::ALL
                    .into_iter()
                    .map(|p| (p, fmt_num(settings.definition(p).offset)))
                    .collect();
                self.height_draft = Some(settings);
                self.height_disabled_feed_reference = None;
                self.height_reference_pick = None;
            }
            Message::HeightSectionToggle(plane) => {
                if !self.collapsed_heights.remove(&plane) {
                    self.collapsed_heights.insert(plane);
                    if self.height_reference_pick == Some(plane) {
                        self.height_reference_pick = None;
                    }
                    self.focused_height = None;
                }
            }
            Message::HeightPickToggle(plane) => {
                self.height_reference_pick = if self.height_reference_pick == Some(plane) {
                    None
                } else {
                    Some(plane)
                };
                self.cad_filter = if self.height_draft.as_ref().is_some_and(|s| {
                    matches!(
                        s.definition(plane).reference,
                        cam_model::HeightReference::FixtureTop
                            | cam_model::HeightReference::FixtureBottom
                    )
                }) {
                    CadFilter::Body
                } else {
                    CadFilter::Face
                };
                self.cad_inspecting = false;
                self.focused_height = Some(height_editor::plane_input_id(plane));
            }
            Message::HeightEntityRemove(plane, index) => {
                if let Some(settings) = &mut self.height_draft {
                    let entities = &mut settings.definition_mut(plane).entities;
                    if index < entities.len() {
                        entities.remove(index);
                    } else {
                        let index = index - entities.len();
                        let drawings = &mut settings.definition_mut(plane).drawings;
                        if index < drawings.len() {
                            drawings.remove(index);
                        }
                    }
                }
            }
            Message::HeightSelectionClear(plane) => {
                if let Some(settings) = &mut self.height_draft {
                    let d = settings.definition_mut(plane);
                    d.entities.clear();
                    d.drawings.clear();
                }
            }
            Message::ToolInfoOpen(tool) => self.tool_info = Some(tool),
            Message::ToolInfoClose => self.tool_info = None,
            Message::ToolParameterFocus(field, focused) => {
                if focused && self.library_mode() {
                    self.selected_tool_parameter = Some(field);
                }
            }
            Message::CheckOperationRenameFocus => {
                if let Some((id, _)) = &self.operation_rename {
                    let id = *id;
                    return iced::widget::operation::is_focused("operation-rename")
                        .map(move |focused| Message::OperationRenameFocus(id, focused));
                }
            }
            Message::OperationRenameFocus(id, focused) => {
                if !focused
                    && self
                        .operation_rename
                        .as_ref()
                        .is_some_and(|(editing, _)| *editing == id)
                {
                    if let Some((id, name)) = self.operation_rename.take() {
                        self.controller.rename_operation(id, name);
                    }
                }
            }
            Message::CheckHeightFocus => {
                if self.library_mode() {
                    return iced::Task::batch(self.inspector_fields().into_iter().map(|field| {
                        iced::widget::operation::is_focused(tool_drawing::parameter_id(field))
                            .map(move |focused| Message::ToolParameterFocus(field, focused))
                    }));
                }
                if self.operation_tab == OperationTab::Heights
                    && matches!(self.controller.selection(), Selection::Operation(_))
                {
                    return iced::Task::batch(self.height_input_ids().into_iter().map(|id| {
                        iced::widget::operation::is_focused(id.clone())
                            .map(move |focused| Message::HeightFocus(id.clone(), focused))
                    }));
                }
                self.focused_height = None;
            }
            Message::HeightFocus(id, focused) => {
                if focused {
                    self.focused_height = Some(id);
                } else if self.focused_height.as_ref() == Some(&id) {
                    self.focused_height = None;
                }
            }
            Message::ParameterInputState(field, focused, cursor) => {
                self.parameter_input_state(field, focused, cursor);
            }
            Message::OpenParameterMenu(field) => return self.open_parameter_menu(field),
            Message::OpenParameterOptionMenu(option) => {
                return self.open_parameter_option_menu(option);
            }
            Message::CloseParameterMenu => self.parameter_menu = None,
            Message::ParameterMenuAction(action) => return self.parameter_menu_action(action),
            Message::ParameterComplete(field, name, cursor) => {
                return self.complete_parameter(field, name, cursor);
            }
            Message::ParameterSuggestionMove(field, delta) => {
                self.move_parameter_suggestion(field, delta);
            }
            Message::FieldChanged(field, value) => {
                if self.library_mode() {
                    self.edited_tool_input = Some(field);
                    self.selected_tool_parameter = Some(field);
                }
                if matches!(field, Field::MajorDia | Field::Pitch) {
                    self.thread_parameters.mode = thread_parameters::ThreadMatchMode::Manual;
                }
                if matches!(field, Field::TabSpacing | Field::StartX | Field::StartY) {
                    self.profile_tab_selection = None;
                }
                if self.creating_operation() {
                    self.creation_edited_fields.insert(field);
                }
                let before_chamfer = self
                    .operation_draft
                    .as_ref()
                    .filter(|op| matches!(op, Operation::Chamfer(_)))
                    .map(|op| {
                        let mut op = op.clone();
                        let values = self
                            .fields
                            .iter()
                            .filter_map(|(f, v)| v.parse::<f64>().ok().map(|v| (*f, v)))
                            .collect();
                        apply_op_fields(&mut op, &values);
                        op
                    });
                self.fields.insert(field, value);
                self.edit_cutting_field(field);
                if matches!(field, Field::ChamferWidth | Field::ChamferDepth) {
                    if self.creating_operation() {
                        self.sync_creation_geometry_heights();
                    } else if let Some(before) = before_chamfer {
                        if let Some(value) = self
                            .fields
                            .get(&field)
                            .and_then(|v| v.parse::<f64>().ok())
                            .filter(|v| v.is_finite())
                        {
                            let mut after = before.clone();
                            apply_op_fields(&mut after, &BTreeMap::from([(field, value)]));
                            self.sync_chamfer_height_defaults(&before, &after);
                        }
                    }
                }
                // In the Tools editor, every field edit flows into the working copy so
                // the viewport previews it instantly (no Apply needed); the length trio
                // also refreshes its derived buffers. Apply then commits the working copy.
                if self.library_mode() {
                    self.live_edit(field);
                }
            }
            Message::MachineNameChanged(name) => {
                // Buffered, not committed: Apply lights up, exactly as it does for the
                // travel fields beside it.
                self.machine_name_edit = Some(name);
            }
            Message::PostKindChanged(kind) => {
                // Also buffered. The picker sits in the Machine inspector, so it edits
                // *this machine's* control — and an edit waits for Apply.
                self.machine_post_edit = Some(kind);
            }
            Message::EstlcamOutputSelectionChanged(selection) => {
                let mut options = self
                    .machine_estlcam_edit
                    .unwrap_or(self.controller.estlcam_options());
                (options.output_1, options.output_2) = match selection {
                    1 => (true, false),
                    2 => (false, true),
                    3 => (true, true),
                    _ => (false, false),
                };
                self.machine_estlcam_edit = Some(options);
            }
            Message::Apply => {
                if self.cad_editing_op.is_some() || self.drawing_editing_op.is_some() {
                    self.status =
                        "Use geometry or cancel selection before applying operation changes."
                            .into();
                    return iced::Task::none();
                }
                if self.creating_operation() {
                    return self.update(Message::ConfirmOp);
                }
                // Blocked while any field is invalid or there is nothing to commit (also
                // catches Enter-to-apply).
                if !self.any_field_invalid()
                    && self.operation_height_overrides_valid()
                    && self.inspector_dirty()
                {
                    self.apply_inspector();
                    // A committed edit ends the inspector session. Failed applies
                    // leave the draft dirty, so the inspector stays open for fixes.
                    if !self.inspector_dirty() {
                        self.inspector_active = false;
                    }
                }
            }
            Message::CancelOperationDraft => {
                self.rotary_pick = None;
                self.drawing_editing_op = None;
                self.controller.cancel_operation();
                if let Selection::Operation(id) = self.controller.selection() {
                    let refs = self
                        .controller
                        .geometry_state()
                        .bindings
                        .get(&id)
                        .map(|binding| binding.selected.clone())
                        .unwrap_or_default();
                    self.controller.set_cad_selection(refs);
                }
                self.operation_geometry = None;
                self.operation_binding_preview = None;
                self.operation_draft = None;
                self.operation_tools.clear();
                self.stock_to_leave_draft = None;
                self.cad_editing_op = None;
                self.cad_inspecting = false;
                self.height_draft_id = None;
                self.refresh_fields();
                self.inspector_active = false;
                self.status = "Operation changes cancelled.".into();
            }
            Message::NewProject => {
                self.profile_entry_editor = None;
                // The post is *not* touched: a new project does not move you to a
                // different machine or control, so there is nothing to re-target.
                self.planar_selection.clear();
                self.pending_processing_time = None;
                self.processing_time_dialog = None;
                self.controller.new_project();
                self.status_messages = StatusMessages::default();
                self.reset_model_browser();
                self.focus_ops.clear();
                self.controller.select(Selection::Setup);
                self.end_setup_draft();
                self.refresh_fields();
                self.status = "New project.".to_string();
            }
            Message::OpenProject => {
                return iced::Task::perform(
                    pick_open("OpenCAMStudio project", &["ocam"]),
                    Message::ProjectToOpen,
                );
            }
            Message::ProjectToOpen(Some(path)) => {
                if let Err(error) = self.open_project(&path) {
                    self.status = format!("Open failed: {error}");
                }
            }
            Message::SaveProject => match self.controller.current_path().map(PathBuf::from) {
                Some(path) => self.save_to(&path),
                None => {
                    return iced::Task::perform(
                        pick_save("OpenCAMStudio project", "project.ocam", &["ocam"]),
                        Message::ProjectToSave,
                    );
                }
            },
            Message::SaveProjectAs => {
                return iced::Task::perform(
                    pick_save("OpenCAMStudio project", "project.ocam", &["ocam"]),
                    Message::ProjectToSave,
                );
            }
            Message::ProjectToSave(Some(path)) => self.save_to(&path),
            Message::ExportLibrary => {
                self.library_context_menu = None;
                return iced::Task::perform(
                    pick_save("OpenCAMStudio tool library", "tools.ocam", &["ocam"]),
                    Message::LibraryToExport,
                );
            }
            Message::LibraryToExport(Some(path)) => {
                let file = OcamFile::Library(self.library.clone());
                self.status = match file.to_json() {
                    Ok(json) => match std::fs::write(&path, json) {
                        Ok(()) => format!("Exported tool library to {}.", path.display()),
                        Err(e) => format!("Library export failed: {e}"),
                    },
                    Err(e) => format!("Library export failed: {e}"),
                };
            }
            Message::ToggleLibrarySidebar => {
                self.settings.tool_table.library_sidebar_collapsed =
                    !self.settings.tool_table.library_sidebar_collapsed;
                self.library_sidebar_drag = None;
                self.library_context_menu = None;
                self.settings.save();
            }
            Message::LibrarySidebarResizeStart => {
                self.library_sidebar_drag = Some((
                    self.window_cursor.x,
                    self.settings.tool_table.library_sidebar_width,
                ));
            }
            Message::SelectToolLibrary(name) => self.switch_tool_library(name),
            Message::OpenLibraryContextMenu(name) => {
                if self.library_mode()
                    && (self.inspector_dirty() || !self.tool_drafts.is_empty())
                    && name != self.libraries.active
                {
                    self.status = "OK or Cancel tool changes before changing the library.".into();
                } else {
                    self.switch_tool_library(name);
                    self.library_context_menu = Some(self.window_cursor);
                }
            }
            Message::CloseLibraryContextMenu => self.library_context_menu = None,
            Message::BeginLibraryRename => {
                self.library_context_menu = None;
                self.library_name_edit = self.libraries.active.clone();
                self.library_renaming = true;
            }
            Message::CancelLibraryRename => {
                self.library_renaming = false;
                self.library_name_edit = self.libraries.active.clone();
            }
            Message::LibraryNameChanged(name) => self.library_name_edit = name,
            Message::RenameToolLibrary => {
                let name = self.library_name_edit.trim().to_string();
                if name.is_empty()
                    || (name != self.libraries.active
                        && self.libraries.libraries.contains_key(&name))
                {
                    self.status = "Enter a unique, non-empty library name.".into();
                } else if name != self.libraries.active {
                    self.libraries.libraries.remove(&self.libraries.active);
                    self.libraries.active = name.clone();
                    self.library_name_edit = name;
                    self.save_tool_libraries();
                    self.library_renaming = false;
                } else {
                    self.library_renaming = false;
                }
            }
            Message::NewToolLibrary => {
                self.library_context_menu = None;
                if self.library_mode() && self.inspector_dirty() {
                    self.status =
                        "Apply or discard the current tool edits before creating a library.".into();
                    return iced::Task::none();
                }
                let name = self.libraries.unique_name("New library");
                self.libraries
                    .libraries
                    .insert(name.clone(), ToolLibrary::default());
                self.switch_tool_library(name);
            }
            Message::DeleteToolLibrary => {
                self.library_context_menu = None;
                return iced::Task::perform(
                    confirm_delete_library(
                        self.libraries.active.clone(),
                        self.libraries.libraries.len() == 1,
                    ),
                    Message::DeleteToolLibraryConfirmed,
                );
            }
            Message::DeleteToolLibraryConfirmed((name, confirmed)) => {
                if confirmed && name == self.libraries.active {
                    self.libraries.libraries.remove(&name);
                    if self.libraries.libraries.is_empty() {
                        self.libraries
                            .libraries
                            .insert("Default tools".into(), ToolLibrary::defaults());
                    }
                    let next = self.libraries.libraries.keys().next().cloned().unwrap();
                    self.libraries.active = next.clone();
                    self.library_name_edit = next.clone();
                    self.library = self.libraries.libraries[&next].clone();
                    self.lib_sel = 0;
                    self.tool_drafts.clear();
                    self.reload_tool_edit();
                    if let Some(player) = &mut self.simulation {
                        player.sync_force_material(&self.library);
                    }
                    self.save_tool_libraries();
                    self.status = format!("Deleted tool library ‘{name}’.");
                }
            }
            Message::ImportLibrary => {
                self.library_context_menu = None;
                if self.library_mode() && self.inspector_dirty() {
                    self.status =
                        "Apply or discard the current tool edits before importing a library."
                            .into();
                    return iced::Task::none();
                }
                return iced::Task::perform(
                    pick_open("Tool library (.ocam or Fusion .tools)", &["ocam", "tools"]),
                    Message::LibraryToImport,
                );
            }
            Message::LibraryToImport(Some(path)) => {
                if path
                    .extension()
                    .is_some_and(|ext| ext.eq_ignore_ascii_case("tools"))
                {
                    match crate::fusion_tools::read(&path) {
                        Ok(entries) if entries.is_empty() => {
                            self.status = "The tool database is empty.".into()
                        }
                        Ok(entries) => self.fusion_import = Some(entries),
                        Err(e) => self.status = format!("Tool database import failed: {e}"),
                    }
                    return iced::Task::none();
                }
                self.status = match OcamFile::read(&path) {
                    Ok(OcamFile::Library(lib)) => {
                        let n = lib.tools.len();
                        let base = path
                            .file_stem()
                            .and_then(|s| s.to_str())
                            .unwrap_or("Imported library");
                        let name = self.libraries.unique_name(base);
                        self.libraries.libraries.insert(name.clone(), lib);
                        self.switch_tool_library(name);
                        format!("Imported {n} tool(s) from {}.", path.display())
                    }
                    Ok(OcamFile::Project(_)) => {
                        "That .ocam is a project, not a tool library.".to_string()
                    }
                    Err(e) => format!("Library import failed: {e}"),
                };
            }
            Message::FusionImportToggle(index, selected) => {
                if let Some(entry) = self
                    .fusion_import
                    .as_mut()
                    .and_then(|items| items.get_mut(index))
                {
                    if entry.tool.is_some() {
                        entry.selected = selected;
                    }
                }
            }
            Message::FusionImportSelectAll(selected) => {
                if let Some(entries) = &mut self.fusion_import {
                    for entry in entries {
                        entry.selected = selected && entry.tool.is_some();
                    }
                }
            }
            Message::FusionImportCancel => self.fusion_import = None,
            Message::FusionImportConfirm => {
                if let Some(entries) = self.fusion_import.take() {
                    let first = self.library.tools.len();
                    for entry in entries.into_iter().filter(|entry| entry.selected) {
                        if let Some(tool) = entry.tool {
                            self.library.append_imported_tool(tool);
                        }
                    }
                    let count = self.library.tools.len() - first;
                    if count > 0 {
                        self.save_tool_libraries();
                        self.lib_sel = first;
                        self.reload_tool_edit();
                    }
                    self.status = format!("Imported {count} tool(s) into the library.");
                }
            }
            Message::ImportCad => {
                return iced::Task::perform(
                    pick_open(
                        "CAD model / drawing",
                        &["step", "stp", "iges", "igs", "dxf", "dwg"],
                    ),
                    Message::CadToImport,
                );
            }
            Message::CadToImport(Some(path)) => {
                self.controller.start_cad_import(path);
                self.status = "Importing CAD: translating, validating and tessellating…".into();
            }
            Message::PollCadImport => {
                if let Some(result) = self.controller.poll_cad_import() {
                    return self.complete_import(result);
                }
            }
            Message::ReplaceCadAsset(id) => {
                return iced::Task::perform(
                    pick_open("Replace CAD model", &["step", "stp", "iges", "igs"]),
                    move |path| Message::CadReplacementChosen(id, path),
                );
            }
            Message::CadReplacementChosen(id, Some(path)) => {
                match self.controller.start_cad_replacement(id, path) {
                    Ok(_) => self.status = "Importing replacement model…".into(),
                    Err(error) => {
                        self.status = error.clone();
                        return self.queue_notice(Notice {
                            title: "Cannot replace model".into(),
                            entries: vec![notices::NoticeEntry {
                                severity: Severity::Error,
                                message: error.clone(),
                                source: None,
                                verification_collision: false,
                            }],
                            detail: error,
                            error: true,
                        });
                    }
                }
            }
            Message::CadReplacementChosen(_, None) => {}
            Message::RemoveCadAsset(id) => {
                if self.controller.remove_cad_asset(id) {
                    self.model_view = Some(TreeNode::Models);
                    self.hidden_assets.remove(&id);
                    self.collapsed_assets.remove(&id);
                    self.cad_hover = None;
                    self.cad_candidates.clear();
                    self.refresh_fields();
                    self.status =
                        "Model removed. Dependent operations need reselection; Undo restores it."
                            .into();
                }
            }
            Message::NoticeFilterChanged(index, value) => {
                if let Some(filter) = self.notice_filters.get_mut(index) {
                    *filter = value;
                }
            }
            Message::NoticeBackdrop => {}
            Message::NoticeDismissed => {
                self.notice = None;
                return self.show_next_notice();
            }
            Message::PollRun => {
                self.poll_operation_preview();
                if self.controller.poll_run() {
                    self.record_status_messages();
                    self.report_run();
                    if let Some(pending) = self.pending_processing_time.take() {
                        if self.controller.outcome().is_some_and(|outcome| {
                            outcome.stamp.as_ref() == Some(&pending.stamp)
                                && outcome.completion == JobCompletion::Complete
                        }) {
                            self.open_processing_time_dialog(pending);
                        }
                    }
                    if self.pending_simulation.take().is_some_and(|stamp| {
                        self.controller.outcome().is_some_and(|outcome| {
                            outcome.stamp.as_ref() == Some(&stamp)
                                && outcome.completion == JobCompletion::Complete
                        })
                    }) {
                        let operation = (self.focus_ops.len() == 1)
                            .then(|| self.focus_ops.iter().next().copied())
                            .flatten();
                        match SimulationPlayer::new_for_operation(&self.controller, operation) {
                            Ok(mut player) => {
                                player.sync_force_material(&self.library);
                                player.configure(
                                    &self.settings.simulation,
                                    self.settings.view.stock_color,
                                );
                                self.simulation = Some(player);
                                self.status =
                                    "Simulation ready. Press Play or drag the timeline.".into();
                            }
                            Err(error) => self.report_error(error),
                        }
                    }
                    let export = self.pending_toolpath_export.take().is_some_and(|stamp| {
                        self.controller.outcome().is_some_and(|outcome| {
                            outcome.stamp.as_ref() == Some(&stamp)
                                && outcome.completion == JobCompletion::Complete
                        })
                    });
                    if export {
                        return self.update(Message::ExportNc);
                    }
                }
            }
            Message::CancelOperationPreview => {
                self.cancel_operation_preview();
                self.status = "Operation preview cancelled.".into();
            }
            Message::CancelRun => {
                self.pending_toolpath_export = None;
                self.pending_simulation = None;
                self.pending_processing_time = None;
                self.processing_time_dialog = None;
                self.controller.cancel_run();
                self.cancel_operation_preview();
                self.status = "Run cancelled. Run again before exporting.".into();
            }
            Message::CancelCadImport => {
                self.controller.cancel_cad_import();
                self.status = "CAD import cancelled.".into();
            }
            Message::GeometryFilterChanged(filter) => {
                if self.stock_contour_editor() {
                    if self.cad_filter != filter {
                        self.controller.set_cad_selection(vec![]);
                    }
                    self.cad_filter = filter;
                    self.cad_filter_explicit = true;
                    self.cad_hover = None;
                    self.cad_candidates.clear();
                    return iced::Task::none();
                }
                let _ = self.pick_operation_geometry();
                if self.cad_filter != filter
                    && !matches!(
                        self.geometry_operation_kind(),
                        Some(OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral)
                    )
                {
                    self.remove_operation_geometry(None);
                }
                self.cad_filter = filter;
                self.cad_filter_explicit = true;
                self.cad_hover = None;
                self.cad_candidates.clear();
            }
            Message::CadFilterChanged(filter) => {
                self.cad_filter = filter;
                self.cad_filter_explicit = true;
                self.cad_hover = None;
                self.cad_candidates.clear();
            }
            Message::CadHover(entity) => self.cad_hover = entity,
            Message::CadPicked(candidates) => {
                if self.rotary_pick.is_some() {
                    if let Some(candidate) = candidates.first() {
                        self.rotary_cad_pick(&candidate.entity);
                    }
                    return iced::Task::none();
                }
                if self
                    .pattern_draft
                    .as_ref()
                    .is_some_and(|draft| draft.picking.is_some())
                {
                    if let Some(candidate) = candidates.first() {
                        self.pattern_entity_picked(candidate.entity.clone());
                    }
                    return iced::Task::none();
                }
                if self
                    .profile_entry_editor
                    .as_ref()
                    .is_some_and(|e| e.picking)
                {
                    if let Some(candidate) = candidates.first() {
                        self.profile_entry_cad_pick(candidate.entity.clone());
                    }
                    return iced::Task::none();
                }
                if self.setup_model_pick {
                    if let Some(candidate) = candidates.first() {
                        return self.toggle_setup_body_pick(candidate.entity.instance_id);
                    }
                    return iced::Task::none();
                }
                if self.height_reference_pick.is_some() {
                    if let Some(candidate) = candidates.first() {
                        self.height_geometry_selected(candidate.entity.clone());
                    }
                    return iced::Task::none();
                }
                if let (Some(axis), [candidate]) = (self.setup_axis_pick, candidates.as_slice()) {
                    let selected = if axis == 3 {
                        self.set_round_stock_axis(candidate.entity.clone())
                    } else {
                        self.set_setup_axis_reference(axis, Some(candidate.entity.clone()));
                        true
                    };
                    if selected {
                        self.setup_axis_pick = None;
                        self.cad_candidates.clear();
                    }
                } else if self.setup_axis_pick.is_some() && candidates.len() > 1 {
                    self.cad_candidates = candidates.clone();
                    self.status = "Choose the intended overlapping edge in the right panel.".into();
                } else if self.setup_axis_pick.is_some() {
                    self.status = "No edge was selected. The axis picker is still armed.".into();
                } else if let Some(candidate) = candidates.first() {
                    self.select_cad_entity(candidate.entity.clone());
                } else if !self.modifiers.command()
                    && !self.modifiers.control()
                    && !self.modifiers.shift()
                {
                    self.controller.set_cad_selection(vec![]);
                    if !self.creating_operation()
                        && self.cad_editing_op.is_none()
                        && !self.setup_draft_dirty()
                        && !self.inspector_dirty()
                    {
                        self.clear_browser_selection();
                    }
                }
                if self.setup_axis_pick.is_none() {
                    self.cad_candidates = candidates;
                }
            }
            Message::ProfileEntry(message) => self.profile_entry_message(message),
            Message::ProfileSelectionModeChanged(mode) => {
                if self.profile_selection_mode != mode {
                    let _ = self.pick_operation_geometry();
                    self.remove_operation_geometry(None);
                    self.profile_selection_mode = mode;
                    self.cad_filter_explicit = true;
                    self.status = mode.hint().into();
                }
            }
            Message::ProfileCoplanarChanged(enabled) => self.profile_coplanar = enabled,
            Message::ProfileChainEdgesChanged(enabled) => {
                if self.profile_chain_edges != enabled {
                    let _ = self.pick_operation_geometry();
                    self.remove_operation_geometry(None);
                    self.profile_chain_edges = enabled;
                    self.cad_filter_explicit = true;
                }
            }
            Message::GeometryPick => return self.pick_operation_geometry(),
            Message::OperationGeometry(message) => self.operation_geometry_message(message),
            Message::GeometryRowSelected(index) => self.geometry_row_selection = Some(index),
            Message::GeometryRemove(index) => self.remove_operation_geometry(Some(index)),
            Message::GeometryClear => self.remove_operation_geometry(None),
            Message::CadSelect(entity) => {
                if self
                    .pattern_draft
                    .as_ref()
                    .is_some_and(|draft| draft.picking.is_some())
                {
                    self.pattern_entity_picked(entity);
                    return iced::Task::none();
                }
                if self.setup_model_pick {
                    return self.toggle_setup_body_pick(entity.instance_id);
                }
                if self.height_reference_pick.is_some() {
                    self.height_geometry_selected(entity);
                    return iced::Task::none();
                }
                if let Some(axis) = self.setup_axis_pick {
                    let selected = if axis == 3 {
                        self.set_round_stock_axis(entity)
                    } else {
                        self.set_setup_axis_reference(axis, Some(entity));
                        true
                    };
                    if selected {
                        self.setup_axis_pick = None;
                        self.cad_candidates.clear();
                    }
                } else {
                    self.select_cad_entity(entity);
                }
            }
            Message::CadIsolateInstance(id) => {
                self.cad_hidden = self
                    .controller
                    .geometry_state()
                    .instances
                    .iter()
                    .filter(|instance| instance.id != id)
                    .map(|instance| instance.id)
                    .collect();
            }
            Message::CadShowAll => {
                self.cad_hidden.clear();
                self.hidden_assets.clear();
            }
            Message::SetInstanceVisible(instance, visible) => {
                self.cad_hover = None;
                self.cad_candidates.clear();
                if visible {
                    self.cad_hidden.remove(&instance);
                } else {
                    self.cad_hidden.insert(instance);
                }
            }
            Message::CadEscape => {
                if self.rotary_pick.take().is_some() {
                    self.status = "Rotary geometry selection finished.".into();
                    return iced::Task::none();
                }
                if self
                    .pattern_draft
                    .as_mut()
                    .is_some_and(|draft| draft.picking.take().is_some())
                {
                    self.cad_candidates.clear();
                    self.status = "Pattern selection cancelled.".into();
                    return iced::Task::none();
                }
                if self.simulation.take().is_some() {
                    self.pending_simulation = None;
                }
                if self.geometry_editor.is_some() {
                    self.operation_geometry_message(GeometryMessage::Cancel);
                    return iced::Task::none();
                }
                if self.setup_model_pick || self.setup_axis_pick.is_some() || self.in_origin_pick()
                {
                    self.cancel_setup_picks();
                    self.status = "Setup selection cancelled.".into();
                    return iced::Task::none();
                }
                if self.height_reference_pick.take().is_some() {
                    return iced::Task::none();
                }
                if self.creating_operation() {
                    return self.update(Message::CancelOp);
                }
                self.cancel_tree_drag();
                self.drawing_editing_op = None;
                self.planar_selection.clear();
                self.cad_inspecting = false;
                self.cad_editing_op = None;
                self.cad_pending_kind = None;
                self.cad_filter_explicit = false;
                self.geometry_additive = false;
                self.geometry_auto_pick = false;
                self.cad_hover = None;
                self.cad_candidates.clear();
                self.controller.set_cad_selection(vec![]);
                self.controller.cancel_operation();
                if self.setup_draft.is_none() && !self.inspector_active {
                    self.clear_browser_selection();
                }
            }
            Message::ConfirmCadOperation => {
                if self.creating_operation() {
                    return self.update(Message::ConfirmOp);
                }
                if let Some(id) = self.cad_editing_op {
                    let selected = self.controller.cad_selection().to_vec();
                    let profile_geometry = self.profile_selection_geometry().map(|p| p.0);
                    match self.preview_geometry_draft(id, selected.clone()) {
                        Ok(binding) => {
                            let previous_operation = self.operation_draft.clone();
                            let geometry_changed = self
                                .operation_binding_preview
                                .as_ref()
                                .or_else(|| self.controller.geometry_state().bindings.get(&id))
                                .is_some_and(|previous| previous.selected != selected);
                            if let Some(profile) = profile_geometry {
                                self.edit_operation_draft(|operation| {
                                    if let Operation::Profile(draft) = operation {
                                        draft.chain = profile.chain;
                                        draft.closed = profile.closed;
                                        draft.side = profile.side;
                                        draft.additional_chains = profile.additional_chains;
                                    }
                                });
                            }
                            if let Ok(input) = crate::cad_operations::geometry_for_setup(
                                self.controller.document(),
                            )
                            .and_then(|geometry| {
                                cam_geometry3d::resolve_binding_advisory(&geometry, &binding)
                            }) {
                                self.edit_operation_draft(|operation| match operation {
                                    Operation::Chamfer(op) => {
                                        op.faces = input.chamfers.clone();
                                        if let Some(face) = op.faces.first() {
                                            op.top = face.top;
                                            op.depth = face.top - face.bottom;
                                            op.width = face.width();
                                        }
                                        if let Some(path) = input.profiles.first() {
                                            if op.chain != path.chain {
                                                op.side = path.side;
                                            }
                                            op.chain = path.chain.clone();
                                            op.closed = path.closed;
                                            if !op.closed {
                                                op.start = None;
                                                op.lead_overlap = 0.;
                                            }
                                        }
                                    }
                                    Operation::Engrave(op) => {
                                        if let Some(path) = input.profiles.first() {
                                            op.chain = path.chain.clone();
                                            op.closed = path.closed;
                                            if !op.closed {
                                                op.start = None;
                                            }
                                        }
                                    }
                                    Operation::Pocket(cam_model::PocketOp {
                                        boundary,
                                        islands,
                                        additional_regions,
                                        ..
                                    }) => {
                                        if let Some(region) = input.regions.first() {
                                            *boundary = region.outer().clone();
                                            *islands = region.holes().to_vec();
                                            *additional_regions =
                                                input.regions.iter().skip(1).cloned().collect();
                                        }
                                    }
                                    Operation::Carve(op) => {
                                        if let Some(region) = input.regions.first() {
                                            op.boundary = region.outer().clone();
                                            op.islands = region.holes().to_vec();
                                        }
                                    }
                                    Operation::Bore(op) => {
                                        op.points = input
                                            .holes
                                            .iter()
                                            .map(|h| [h.center.x, h.center.y])
                                            .collect();
                                        if let Some(hole) = input.holes.first() {
                                            op.diameter = hole.diameter;
                                        }
                                        op.z_top = input.heights.top;
                                        op.z_bottom = input.heights.bottom;
                                    }
                                    Operation::Thread(op) => {
                                        op.points = input
                                            .holes
                                            .iter()
                                            .map(|h| [h.center.x, h.center.y])
                                            .collect()
                                    }
                                    _ => {}
                                });
                            }
                            self.sync_pocket_geometry_height_defaults(&binding);
                            self.sync_chamfer_face_height_defaults(previous_operation.as_ref());
                            if geometry_changed
                                && !matches!(
                                    previous_operation.as_ref(),
                                    Some(Operation::Pocket(_) | Operation::Chamfer(_))
                                )
                            {
                                self.refresh_geometry_height_defaults(previous_operation.as_ref());
                            }
                            self.sync_chamfer_geometry_fields();
                            self.operation_binding_preview = Some(binding);
                            self.operation_geometry = Some(selected);
                            self.cad_editing_op = None;
                            self.cad_inspecting = false;
                            self.cad_pending_kind = None;
                            self.operation_tab = OperationTab::Geometry;
                            self.status = "Geometry staged. OK commits every operation tab; Cancel discards the changes.".into();
                        }
                        Err(error) => {
                            self.report_error(format!("Geometry selection rejected: {error}"))
                        }
                    }
                    return iced::Task::none();
                }
            }
            Message::EditCadGeometry(id) => {
                if let Some(binding) = self.controller.geometry_state().bindings.get(&id) {
                    self.cad_filter_explicit = false;
                    let refs = binding.selected.clone();
                    if matches!(self.current_operation(id), Some(Operation::Profile(_))) {
                        self.profile_selection_mode = match refs.first().map(|r| r.kind) {
                            Some(EntityKind::Face) => ProfileSelectionMode::FaceContours,
                            Some(EntityKind::Silhouette) => ProfileSelectionMode::Silhouette,
                            _ => ProfileSelectionMode::Chain,
                        };
                        self.profile_chain_edges =
                            refs.first().is_some_and(|r| r.kind == EntityKind::Edge);
                    }
                    self.cad_filter = match binding.recipe {
                        cam_model::ExtractionRecipe::ProfileLoop { .. }
                        | cam_model::ExtractionRecipe::ChamferLoop
                        | cam_model::ExtractionRecipe::ChamferFaces
                        | cam_model::ExtractionRecipe::EngravePath
                        | cam_model::ExtractionRecipe::CarveRegion
                        | cam_model::ExtractionRecipe::PocketBoundary => {
                            match refs.first().map(|r| r.kind) {
                                Some(EntityKind::Edge) => CadFilter::Edge,
                                Some(EntityKind::Face) => CadFilter::Face,
                                _ => CadFilter::Loop,
                            }
                        }
                        cam_model::ExtractionRecipe::AdaptiveClearing
                        | cam_model::ExtractionRecipe::ParallelFinishing
                        | cam_model::ExtractionRecipe::SpiralFinishing => {
                            match refs.first().map(|r| r.kind) {
                                Some(EntityKind::Face) => CadFilter::Face,
                                _ => CadFilter::Body,
                            }
                        }
                        _ => CadFilter::Face,
                    };
                    let refs = self.operation_geometry.clone().unwrap_or(refs);
                    self.controller.set_cad_selection(refs);
                    self.cad_inspecting = true;
                    self.cad_editing_op = Some(id);
                    self.status = "Select replacement geometry, then Use geometry to stage it. The operation OK/Cancel footer commits or discards all tabs.".into();
                }
            }
            Message::CadOverrideTopChanged(value) => self.cad_override_top = value,
            Message::CadOverrideBottomChanged(value) => self.cad_override_bottom = value,
            Message::ExportNc => {
                if self.setup_draft_dirty()
                    || self.inspector_dirty()
                    || self.controller.pending_op().is_some()
                    || self.cad_pending_kind.is_some()
                    || self.cad_editing_op.is_some()
                {
                    self.status = "Export blocked: OK or Cancel pending inspector changes and geometry selection first.".into();
                    return iced::Task::none();
                }
                // **Post before opening the dialog.** The refusal used to come *after*
                // the operator had picked a folder and a filename, which reads as "saved"
                // — they close the dialog believing the job exported and go looking for a
                // file that was never written. Doing the work first means the only dialog
                // they ever see is one that will produce a file.
                let nc = match self.controller.export_nc() {
                    Ok(nc) => nc.to_owned(),
                    Err(e) => {
                        self.status = format!("Export blocked: {e}");
                        return iced::Task::perform(report_export_blocked(format!("{e}")), |()| {
                            Message::CloseRibbonPopup
                        });
                    }
                };
                // Guardrail: if any included operations are exact duplicates, they
                // would post the same toolpath twice. Confirm before the machine
                // sees it — but don't block (a spring/finishing pass is legitimate).
                let included: BTreeSet<_> = self
                    .controller
                    .outcome()
                    .into_iter()
                    .flat_map(|outcome| &outcome.program.steps)
                    .filter_map(|step| match step {
                        cam_cldata::Step::Rapid { tag, .. }
                        | cam_cldata::Step::Linear { tag, .. }
                        | cam_cldata::Step::Arc { tag, .. } => Some(tag.op_id),
                        cam_cldata::Step::Drill(cycle) => Some(cycle.tag.op_id),
                        _ => None,
                    })
                    .collect();
                let groups: Vec<_> = self
                    .controller
                    .duplicate_operation_groups()
                    .into_iter()
                    .map(|group| {
                        group
                            .into_iter()
                            .filter(|id| included.contains(id))
                            .collect::<Vec<_>>()
                    })
                    .filter(|group| group.len() > 1)
                    .collect();
                let duplicates = (!groups.is_empty()).then(|| describe_duplicates(&groups));
                let verification_warnings = self
                    .controller
                    .outcome()
                    .map_or(0, |outcome| outcome.collisions.len());
                let kind = self.controller.post_kind();
                return iced::Task::perform(
                    async move {
                        if let Some(detail) = duplicates {
                            if !confirm_export_duplicates(detail).await {
                                return Message::ExportDuplicatesDeclined;
                            }
                        }
                        if verification_warnings > 0 {
                            report_export_verification_warnings(verification_warnings).await;
                        }
                        let path =
                            pick_save("G-code", kind.default_file_name(), kind.file_extensions())
                                .await;
                        Message::NcToExport(nc, path)
                    },
                    std::convert::identity,
                );
            }
            Message::ExportDuplicatesDeclined => {
                self.status =
                    "Export cancelled — exclude or edit the duplicate operation(s).".to_string();
            }
            Message::NcToExport(nc, Some(path)) => {
                // The post already succeeded (checked before the dialog opened), so a
                // failure here is the *write* — a read-only folder, a full disk. Still
                // worth a dialog: the operator has just been told a file was coming.
                match std::fs::write(&path, nc) {
                    Ok(()) => {
                        self.status = format!("Exported G-code to {}.", path.display());
                    }
                    Err(e) => {
                        self.status = format!("Export failed: {e}");
                        return iced::Task::perform(
                            report_export_blocked(format!(
                                "Could not write {}.\n\n{e}",
                                path.display()
                            )),
                            |()| Message::CloseRibbonPopup,
                        );
                    }
                }
            }
            // Cancelled dialogs — nothing to do.
            Message::ProjectToOpen(None)
            | Message::ProjectToSave(None)
            | Message::LibraryToExport(None)
            | Message::LibraryToImport(None)
            | Message::CadToImport(None)
            | Message::NcToExport(_, None) => {}
            Message::Undo => {
                if self.controller.undo() {
                    self.rebase_setup_draft();
                    if let Selection::Operation(id) = self.controller.selection() {
                        self.begin_operation_draft(id);
                    }
                    self.refresh_fields();
                    self.rerun();
                    self.status = format!("Undid change. {}", self.status);
                } else {
                    self.status = "Nothing to undo.".to_string();
                }
            }
            Message::Redo => {
                if self.controller.redo() {
                    self.rebase_setup_draft();
                    if let Selection::Operation(id) = self.controller.selection() {
                        self.begin_operation_draft(id);
                    }
                    self.refresh_fields();
                    self.rerun();
                    self.status = format!("Redid change. {}", self.status);
                } else {
                    self.status = "Nothing to redo.".to_string();
                }
            }
            Message::SelectOrigin(index) => {
                self.end_setup_draft();
                self.controller.select_origin(index);
                self.refresh_fields();
            }
            Message::AddOrigin => {
                self.end_setup_draft();
                self.controller.add_origin();
                self.refresh_fields();
            }
            Message::DeleteOrigin(index) => {
                self.controller.delete_origin(index);
                self.refresh_fields();
                self.rerun();
            }
            Message::ToggleOriginDisabled(index, disabled) => {
                self.controller.set_origin_disabled(index, disabled);
                self.rerun();
            }
            Message::ToggleShowOrigin => {
                self.show_origin = !self.show_origin;
                self.remember();
            }
            Message::ToggleEnvelope => {
                self.show_envelope = !self.show_envelope;
                self.remember();
                self.status = if self.show_envelope {
                    format!(
                        "Showing {}'s travel around the job — a size check, not its \
                         position on the machine.",
                        self.controller.machine().name
                    )
                } else {
                    "Hiding the machine envelope.".to_string()
                };
            }
            Message::ToggleSetOrigin => {
                self.setup_model_pick = false;
                self.setup_axis_pick = None;
                self.setup_hover = SetupHover::default();
                let on = !self.setting_origin;
                self.setting_origin = on;
                self.setting_origin_2pt = false;
                self.origin_first = None;
                self.snap_hover = None;
                self.hover_loop = None;
                if on {
                    self.controller.cancel_operation();
                    self.status =
                        "Set draft origin: click a corner/centre (snaps) or any point.".to_string();
                }
            }
            Message::ToggleSetOrigin2pt => {
                self.setup_model_pick = false;
                self.setup_axis_pick = None;
                self.setup_hover = SetupHover::default();
                let on = !self.setting_origin_2pt;
                self.setting_origin_2pt = on;
                self.setting_origin = false;
                self.origin_first = None;
                self.snap_hover = None;
                self.hover_loop = None;
                if on {
                    self.controller.cancel_operation();
                    self.status = "Set draft origin: click point 1 (sets X).".to_string();
                }
            }
            Message::OriginPointPicked(world) => {
                let p = self
                    .setup_draft
                    .as_ref()
                    .map_or(world, |setup| setup.frame.to_setup_vector(world));
                self.setup_values.retain(|key, _| *key > 2);
                self.setup_hover = SetupHover::default();
                self.snap_hover = None;
                self.hover_loop = None;
                if self.setting_origin {
                    let mode = self.setup_origin_mode();
                    if let Some(draft) = &mut self.setup_draft {
                        draft.origin = p;
                        draft.frame.references.origin = match mode {
                            SetupOriginMode::StockBox => cam_model::FrameOrigin::StockBoxPoint(p),
                            SetupOriginMode::ModelBox => cam_model::FrameOrigin::ModelBoxPoint(p),
                            _ => cam_model::FrameOrigin::SelectedPoint(p),
                        };
                    }
                    self.setting_origin = false;
                    self.status = format!(
                        "Draft origin set to X{:.3} Y{:.3} Z{:.3}.",
                        p[0], p[1], p[2]
                    );
                } else if let Some(first) = self.origin_first.take() {
                    let origin = [first[0], p[1], (first[2] + p[2]) / 2.0];
                    if let Some(draft) = &mut self.setup_draft {
                        draft.origin = origin;
                        draft.frame.references.origin = cam_model::FrameOrigin::TwoPoints(origin);
                    }
                    self.setting_origin_2pt = false;
                    self.status = format!(
                        "Draft origin set to X{:.3} Y{:.3} Z{:.3}.",
                        origin[0], origin[1], origin[2]
                    );
                } else {
                    self.origin_first = Some(p);
                    self.status = "Set draft origin: click point 2 (sets Y).".to_string();
                }
            }
            Message::BeginOp(kind) => {
                self.cad_selected_project_tool = None;
                if self.controller.setups().is_empty() {
                    self.status =
                        "Select a model or body and create a setup before adding an operation."
                            .into();
                    return iced::Task::none();
                }
                self.end_setup_draft();
                self.cad_filter_explicit = false;
                self.operation_tools.clear();
                self.operation_tab = OperationTab::Tool;
                let surface = matches!(kind, OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral);
                if surface && self.controller.geometry_state().instances.is_empty() {
                    self.status =
                        "Import a solid CAD model before creating a 3D surface operation.".into();
                    return iced::Task::none();
                }
                if !self.controller.geometry_state().instances.is_empty()
                    && (surface
                        || self.controller.regions().is_empty()
                        || !self.controller.cad_selection().is_empty())
                {
                    self.cancel_setup_picks();
                    self.cad_pending_kind = Some(kind);
                    if kind == OpKind::Profile {
                        self.profile_selection_mode =
                            match self.controller.cad_selection().first().map(|r| r.kind) {
                                Some(EntityKind::Face) => ProfileSelectionMode::FaceContours,
                                Some(EntityKind::Silhouette) => ProfileSelectionMode::Silhouette,
                                _ => ProfileSelectionMode::Chain,
                            };
                        self.profile_chain_edges = self
                            .controller
                            .cad_selection()
                            .first()
                            .is_some_and(|r| r.kind == EntityKind::Edge);
                    }
                    let tool = self.library.default_tool_for(kind);
                    self.cad_tool_pick = tool.map(|tool| tool.number);
                    self.cad_inspecting = false;
                    self.cad_editing_op = None;
                    self.cad_filter =
                        if matches!(kind, OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral) {
                            match self.controller.cad_selection().first().map(|r| r.kind) {
                                Some(EntityKind::Face) => CadFilter::Face,
                                _ => CadFilter::Body,
                            }
                        } else if matches!(
                            kind,
                            OpKind::Profile | OpKind::Chamfer | OpKind::Engrave
                        ) {
                            CadFilter::Loop
                        } else {
                            CadFilter::Face
                        };
                    self.controller.cancel_operation();
                    self.seed_creation_editor(kind);
                    self.status = "Choose a tool, then select geometry on the Geometry tab.".into();
                    return iced::Task::none();
                }
                self.setting_origin = false;
                self.setting_origin_2pt = false;
                self.origin_first = None;
                if self.planar_selection.is_empty() {
                    self.controller.begin_operation(kind);
                } else if let Err(error) = self
                    .controller
                    .begin_operation_from_selection(kind, &self.planar_selection)
                {
                    self.status = error;
                    return iced::Task::none();
                }
                // Start with a supported cutter; Face prefers the largest flat mill.
                // Every parameter remains editable in the shared operation tabs.
                if self.controller.pending_op().is_some() {
                    if let Some(tool) = self.library.default_tool_for(kind) {
                        let family = ToolKindPick::of(tool.kind);
                        if families_for(kind).contains(&family) {
                            let number = self.stage_operation_tool(tool);
                            self.controller.set_pending_tool(number);
                        }
                    }
                }
                if self.creating_operation() {
                    self.seed_creation_editor(kind);
                } else {
                    self.refresh_fields();
                }
                self.status = if self.controller.pending_op().is_some() {
                    if op_accepts_open_paths(kind) && !self.controller.open_paths().is_empty() {
                        "Choose a tool and click a boundary or an open stroke — in \
                         either order, then Confirm."
                            .to_string()
                    } else {
                        "Choose a tool and click the geometry — in either order, then Confirm."
                            .to_string()
                    }
                } else {
                    "Open a part first.".to_string()
                };
            }
            Message::SetPendingLibraryTool(i) => {
                self.cad_selected_project_tool = None;
                if let Some(tool) = self.library.tools.get(i).cloned() {
                    if let Some(op) = self.operation_draft.as_ref() {
                        if !families_for(AppController::operation_kind(op))
                            .contains(&ToolKindPick::of(tool.kind))
                        {
                            self.status =
                                "This cutter kind is not supported by the operation.".into();
                            return iced::Task::none();
                        }
                    }
                    if self.cad_pending_kind.is_some() {
                        self.cad_tool_pick = Some(tool.number);
                    } else {
                        let number = self.stage_operation_tool(tool);
                        self.controller.set_pending_tool(number);
                    }
                    self.sync_creation_tool();
                    self.refresh_operation_fields();
                    self.seed_user_parameter_defaults(|_| true);
                }
            }
            Message::CancelOp => {
                self.rotary_pick = None;
                self.cad_selected_project_tool = None;
                self.cutting_force_inputs
                    .remove(&(self.controller.inspection_setup().id, None));
                self.geometry_editor = None;
                self.cad_pending_kind = None;
                self.cad_filter_explicit = false;
                self.geometry_additive = false;
                self.geometry_auto_pick = false;
                self.cad_tool_pick = None;
                self.cad_inspecting = false;
                self.controller.set_cad_selection(vec![]);
                self.controller.cancel_operation();
                self.operation_draft = None;
                self.operation_tools.clear();
                self.height_draft_id = None;
                self.focus_selected_op();
                self.refresh_fields();
                self.status = "Cancelled operation creation.".to_string();
            }
            Message::ConfirmOp => {
                if self.creating_operation() {
                    let result = self.creation_operation().and_then(|(operation, binding)| {
                        self.validate_rotary_index_draft()?;
                        let settings = self.edited_height_settings()?;
                        self.controller.commit_operation_creation_with_index(
                            operation,
                            binding,
                            settings,
                            self.operation_tools.clone(),
                            self.stock_to_leave_with_fields(),
                            self.rotary_index_draft.clone(),
                        )
                    });
                    match result {
                        Ok(id) => {
                            let setup_id = self.controller.inspection_setup().id;
                            if let Some(inputs) =
                                self.cutting_force_inputs.remove(&(setup_id, None))
                            {
                                self.cutting_force_inputs
                                    .insert((setup_id, Some(id)), inputs);
                            }
                            // The creation draft was used to stage the allowance, but
                            // the committed setup is now the source of truth for the
                            // selected operation. Keeping it would mask the value just
                            // saved (including fractional allowances) in the inspector.
                            self.stock_to_leave_draft = None;
                            self.cad_pending_kind = None;
                            self.cad_filter_explicit = false;
                            self.geometry_additive = false;
                            self.geometry_auto_pick = false;
                            self.cad_tool_pick = None;
                            self.cad_inspecting = false;
                            self.controller.set_cad_selection(vec![]);
                            self.planar_selection.clear();
                            self.cursor = None;
                            self.rebase_setup_draft();
                            self.begin_operation_draft(id);
                            self.inspector_active = false;
                            self.focus_ops.clear();
                            self.focus_ops.insert(id);
                            self.rerun();
                            self.status = "Operation created.".into();
                        }
                        Err(reason) => self.report_error(reason),
                    }
                }
            }
            Message::PickWorld(w, aperture) => {
                if self.profile_entry_drawing_pick(w, aperture) {
                    return iced::Task::none();
                }
                if let Some(plane) = self.height_reference_pick {
                    if let Some((reference, _)) = self
                        .controller
                        .nearest_drawing_feature(w.map(f64::from), aperture as f64)
                    {
                        if let Some((points, closed)) = self.controller.loop_points(reference) {
                            let selection = cam_model::DrawingHeightSelection {
                                points: points.iter().map(|p| [p.x, p.y]).collect(),
                                closed,
                            };
                            if let Some(settings) = &mut self.height_draft {
                                let drawings = &mut settings.definition_mut(plane).drawings;
                                if let Some(index) = drawings.iter().position(|d| d == &selection) {
                                    drawings.remove(index);
                                } else {
                                    drawings.push(selection);
                                }
                            }
                        }
                    }
                    return iced::Task::none();
                }
                if self.controller.pending_op().is_none() {
                    if self.setup_draft_dirty()
                        || (matches!(self.controller.selection(), Selection::Operation(_))
                            && self.inspector_dirty())
                    {
                        self.status =
                            "OK or Cancel inspector changes before selecting drawing features."
                                .into();
                        return iced::Task::none();
                    }
                    self.clear_browser_selection();
                    let Some((feature, _)) = self
                        .controller
                        .nearest_drawing_feature(w.map(f64::from), aperture as f64)
                    else {
                        if !self.modifiers.command()
                            && !self.modifiers.control()
                            && !self.modifiers.shift()
                        {
                            self.planar_selection.clear();
                        }
                        return iced::Task::none();
                    };
                    let additive = self.modifiers.command()
                        || self.modifiers.control()
                        || self.modifiers.shift();
                    let mut selected = if additive {
                        self.planar_selection.clone()
                    } else {
                        Vec::new()
                    };
                    if selected
                        .first()
                        .is_some_and(|first| first.is_open() != feature.is_open())
                    {
                        self.status =
                            "Open paths and closed boundaries must be selected separately.".into();
                        return iced::Task::none();
                    }
                    if let Some(index) = selected.iter().position(|old| *old == feature) {
                        selected.remove(index);
                    } else {
                        selected.push(feature);
                    }
                    self.planar_selection = selected;
                    self.status = format!(
                        "{} drawing feature(s) selected. Choose an operation.",
                        self.planar_selection.len()
                    );
                    return iced::Task::none();
                }
                let multiple = self.controller.pending_op().is_some_and(|pending| {
                    op_takes_islands(pending.kind)
                        || op_selects_circles(pending.kind)
                        || pending.kind == OpKind::Profile
                });
                if (self.geometry_additive && multiple)
                    || self.modifiers.command()
                    || self.modifiers.control()
                    || self.modifiers.shift()
                {
                    if let Some((feature, point)) = self
                        .controller
                        .nearest_drawing_feature(w.map(f64::from), aperture as f64)
                    {
                        let mut selected = self.controller.pending_operation_geometry();
                        if let Some(index) = selected.iter().position(|old| *old == feature) {
                            selected.remove(index);
                        } else {
                            selected.push(feature);
                        }
                        match self
                            .controller
                            .set_pending_operation_geometry(&selected, Some(point))
                        {
                            Ok(()) => {
                                self.status = format!(
                                "{} compatible feature(s) selected; Confirm creates the operation.",
                                selected.len()
                            )
                            }
                            Err(error) => self.report_error(error),
                        }
                    }
                    return iced::Task::none();
                }

                // Snaps only apply to op kinds with a start; others select by the
                // nearest point on the loop.
                let snaps: &[SnapKind] = if self
                    .controller
                    .pending_op()
                    .is_some_and(|p| op_uses_snaps(p.kind))
                {
                    if self.settings.snapping.enabled {
                        &self.snaps
                    } else {
                        &[]
                    }
                } else {
                    &[]
                };
                match self.controller.pick_operation_geometry(
                    [w[0] as f64, w[1] as f64],
                    aperture as f64,
                    snaps,
                ) {
                    PickResult::Selecting => {
                        let pending = self.controller.pending_op();
                        let has_tool = pending.as_ref().is_some_and(|p| p.tool.is_some());
                        let islands = pending.as_ref().is_some_and(|p| op_takes_islands(p.kind));
                        let n = pending.map_or(0, |p| p.islands.len());
                        self.status = if islands {
                            format!("Boundary set — click areas to exclude ({n}), then Confirm.")
                        } else if has_tool {
                            "Geometry set — Confirm to create the operation.".to_string()
                        } else {
                            "Geometry set — now choose a tool, then Confirm.".to_string()
                        };
                    }
                    PickResult::Missed => {
                        // Word the miss to match what is actually pickable for this
                        // op — a circular edge for drill/thread, else any boundary.
                        let circles = self
                            .controller
                            .pending_op()
                            .is_some_and(|p| op_selects_circles(p.kind));
                        self.status = if circles {
                            "No arc there — click on a circular edge.".to_string()
                        } else {
                            "No line there — click a boundary edge.".to_string()
                        };
                    }
                }
            }
            Message::ViewportCursor(p) => {
                self.cursor = Some(p);
                self.snap_hover = None;
                self.hover_loop = None;
            }
            Message::HoverWorld(screen, w, aperture) => {
                self.cursor = Some(screen);
                self.snap_aperture = aperture as f64;
                // Preview the object-snap under the cursor (drawn as a marker).
                // Active for set-origin, and for op kinds that use a start (inert
                // for Face/Drill/Thread).
                let use_snaps = self.in_origin_pick()
                    || self
                        .controller
                        .pending_op()
                        .is_some_and(|p| op_uses_snaps(p.kind));
                self.snap_hover = if use_snaps && self.settings.snapping.enabled {
                    self.controller.snap_at(
                        [w[0] as f64, w[1] as f64],
                        aperture as f64,
                        &self.snaps,
                    )
                } else {
                    None
                };
                // Highlight the loop a click would select (op picks only —
                // disambiguates concentric circles; drill/thread → circles only).
                self.hover_loop = if self.in_origin_pick() {
                    None
                } else {
                    let circles = self
                        .controller
                        .pending_op()
                        .is_some_and(|p| op_selects_circles(p.kind));
                    self.snap_hover.map(|h| h.loop_ref).or_else(|| {
                        self.controller
                            .nearest_loop_point(
                                [w[0] as f64, w[1] as f64],
                                aperture as f64,
                                circles,
                            )
                            .map(|(l, _)| l)
                    })
                };
            }
            Message::SetupHover(cursor, hover) => {
                self.cursor = Some(cursor);
                self.cad_hover = match hover
                    .snap
                    .as_ref()
                    .map(|h| &h.source)
                    .or(hover.edge.as_ref())
                {
                    Some(SetupFeature::Model(entity)) => Some(entity.clone()),
                    _ => None,
                };
                self.setup_hover = hover;
            }
            Message::SetupStockEdgePicked(direction) => {
                if let Some(axis) = self
                    .pattern_draft
                    .as_ref()
                    .and_then(|draft| draft.picking)
                    .filter(|axis| *axis < 2)
                {
                    self.pattern_direction_picked(axis, direction, None);
                    return iced::Task::none();
                }
                if let Some(axis) = self.setup_axis_pick.take() {
                    if axis == 3 {
                        self.setup_axis_pick = Some(3);
                        self.status =
                            "Select a straight model edge or an origin axis for the stock.".into();
                    } else {
                        self.set_setup_stock_axis(axis, direction);
                    }
                }
                self.setup_hover = SetupHover::default();
                self.cad_candidates.clear();
            }
            Message::ToggleSnapping => {
                self.settings.snapping.enabled = !self.settings.snapping.enabled;
                self.snap_hover = None;
                self.setup_hover = SetupHover::default();
                self.remember();
            }
            Message::ToggleFeatureHighlights => {
                self.settings.snapping.highlight_features =
                    !self.settings.snapping.highlight_features;
                self.remember();
            }
            Message::ToggleSnap(kind) => {
                self.snap_hover = None;
                self.setup_hover = SetupHover::default();
                if let Some(pos) = self.snaps.iter().position(|k| *k == kind) {
                    self.snaps.remove(pos);
                } else {
                    self.snaps.push(kind);
                }
                self.remember();
            }
            Message::ReinitOp => {
                self.operation_tools.clear();
                let id = self.open_op_menu.take();
                if let Some(id) = id {
                    if self.controller.reinitialize_operation(id) {
                        // Reinitialization uses the same tabbed creation editor.
                        if let Some(kind) = self.controller.pending_op().map(|p| p.kind) {
                            if let Some(tool) = self.library.default_tool_for(kind) {
                                let family = ToolKindPick::of(tool.kind);
                                if families_for(kind).contains(&family) {
                                    let number = self.stage_operation_tool(tool);
                                    self.controller.set_pending_tool(number);
                                }
                            }
                        }
                        if let Some(pending) = self.controller.pending_op() {
                            self.seed_creation_editor(pending.kind);
                        }
                        self.status = "Reinitialising: pick a tool, then the geometry \
                                       (the operation keeps its place)."
                            .to_string();
                    } else {
                        self.status = "Open a part first.".to_string();
                    }
                }
            }
            Message::DuplicateOp => {
                self.open_op_menu = None;
                self.controller.duplicate_selected_operation();
                self.focus_selected_op();
                self.refresh_fields();
                self.rerun();
            }
            Message::SetOpExcluded(id, excluded) => {
                self.controller.set_operation_excluded(id, excluded);
                self.rerun();
            }
            Message::DeleteOp => {
                self.open_op_menu = None;
                self.controller.delete_selected_operation();
                // A tool no longer used by any op drops out of the setup.
                self.controller.prune_unused_tools();
                self.focus_selected_op();
                self.refresh_fields();
                self.rerun();
            }
            Message::MoveOpToOrigin(origin) => {
                if let Some(id) = self.open_op_menu.take() {
                    self.controller.set_operation_origin(id, origin);
                    // The op keeps its identity and moves group; re-focus it so the
                    // tree scrolls/highlights where it landed rather than leaving the
                    // eye at the row it left.
                    self.focus_selected_op();
                    self.refresh_fields();
                    self.rerun();
                    let name = self.origin_menu_label(origin);
                    self.status = format!("Operation {id} moved to {name}.");
                }
            }
            Message::TransferOpToSetup(destination, copy) => {
                if let Some(id) = self.open_op_menu.take() {
                    match self
                        .controller
                        .transfer_operation_to_setup(id, destination, copy)
                    {
                        Ok(new_id) => {
                            self.controller.select(Selection::Operation(new_id));
                            self.focus_ops.clear();
                            self.focus_ops.insert(new_id);
                            self.begin_operation_draft(new_id);
                            self.refresh_fields();
                            self.rerun();
                            let name = self
                                .controller
                                .setups()
                                .iter()
                                .find(|s| s.id == destination)
                                .map(|s| s.name.as_str())
                                .unwrap_or("setup");
                            self.status = format!(
                                "Operation {new_id} {} setup {name}.",
                                if copy { "copied to" } else { "moved to" }
                            );
                        }
                        Err(error) => self.report_error(error),
                    }
                }
            }
            Message::ToggleStock => {
                if let Some(player) = &self.simulation {
                    let show = !player.options.show_stock;
                    return self
                        .update_message(Message::SimulationOption(SimulationOption::Stock(show)));
                }
                self.show_stock = !self.show_stock;
                self.status = if self.show_stock {
                    "Showing stock material.".to_string()
                } else {
                    "Hiding stock material.".to_string()
                };
                self.remember();
            }
            Message::ToggleTransparentStock => {
                if let Some(player) = &self.simulation {
                    let transparent = !player.options.transparent_stock;
                    return self.update_message(Message::SimulationOption(
                        SimulationOption::TransparentStock(transparent),
                    ));
                }
                self.settings.view.transparent_stock = !self.settings.view.transparent_stock;
                self.status = if self.settings.view.transparent_stock {
                    "Stock material is transparent."
                } else {
                    "Stock material is opaque."
                }
                .into();
                self.remember();
            }
            Message::OrbitBy(dyaw, dpitch) => {
                // Turntable: horizontal drag spins about world up, vertical tilts.
                // Unclamped, so pitch can go all the way round to the underside.
                self.view.yaw += dyaw;
                self.view.pitch += dpitch;
            }
            Message::PanBy(delta) => {
                for (p, d) in self.view.pan.iter_mut().zip(delta) {
                    *p += d;
                }
            }
            Message::ZoomBy(dz) => {
                self.view.zoom = (self.view.zoom + dz).clamp(-4.0, 6.0);
            }
            Message::SetView(yaw, pitch) => {
                self.view = ViewControls {
                    yaw,
                    pitch,
                    zoom: 0.0,
                    pan: [0.0; 3],
                };
            }
            Message::ResetView => self.view = ViewControls::default(),
            Message::ToggleGizmo => {
                self.show_gizmo = !self.show_gizmo;
                self.remember();
            }
            Message::ShowLicense => self.show_license = true,
            Message::CloseLicense => self.show_license = false,
            Message::ToggleTooltips => {
                self.tooltips = !self.tooltips;
                self.status = if self.tooltips {
                    "Tooltips on.".to_string()
                } else {
                    "Tooltips off.".to_string()
                };
                self.remember();
            }
            Message::SetGizmoSize(v) => {
                // Not persisted here: the slider emits continuously while dragged, and
                // writing the file on every tick would be a hundred writes for one
                // decision. `SettingsSettled` fires on release.
                self.gizmo_size = v.clamp(GIZMO_SIZE_MIN, GIZMO_SIZE_MAX)
            }
            Message::SettingsSettled => {
                if self
                    .controller
                    .set_simulation_accuracy(self.settings.simulation.accuracy)
                {
                    self.rerun();
                }
                self.remember();
            }
            Message::ActiveMachineChanged(name) => {
                if let Some(entry) = self.machines.by_name(&name).cloned() {
                    // A pending edit belongs to the machine it was typed on; carrying it
                    // across would apply it to the wrong one.
                    self.machine_name_edit = None;
                    self.machine_post_edit = None;
                    self.machine_estlcam_edit = None;
                    self.active_machine = name;
                    self.controller.set_machine(entry.machine);
                    // The machine carries its control: picking the machine picks the
                    // post. This is the error the library itself would otherwise create
                    // — right machine, wrong control, because the last job used another.
                    self.controller.set_post_kind(entry.post);
                    self.controller.set_estlcam_options(entry.estlcam);
                    self.remember();
                    self.rerun();
                    self.status = format!("Machine: {} ({}).", self.active_machine, entry.post);
                }
            }
            Message::NewMachine => {
                self.machine_estlcam_edit = None;
                // A copy of the active one: machines in a shop differ from each other in
                // one or two numbers, so duplicating beats starting from nothing.
                let mut entry = self
                    .machines
                    .by_name(&self.active_machine)
                    .cloned()
                    .unwrap_or_else(|| crate::MachineEntry::new(crate::default_machine()));
                entry.machine.name = format!("{} copy", entry.name());
                self.active_machine = self.machines.add(entry);
                self.machines.save();
                self.sync_active_machine();
                self.status = format!("Added machine \"{}\".", self.active_machine);
            }
            Message::DeleteMachine => {
                if self.machines.remove(&self.active_machine) {
                    self.machine_estlcam_edit = None;
                    self.machines.save();
                    // `remove` refuses the last one, so there is always something here.
                    let entry = self.machines.machines[0].clone();
                    self.active_machine = entry.name().to_string();
                    self.controller.set_machine(entry.machine);
                    self.controller.set_post_kind(entry.post);
                    self.controller.set_estlcam_options(entry.estlcam);
                    self.remember();
                    self.rerun();
                    self.status = format!("Machine: {}.", self.active_machine);
                }
            }
            Message::ShowPrefs => return self.open_preferences(),
            Message::Preferences(_) => unreachable!(),
            Message::ShortcutKey(key, modifiers) => {
                if let Some(binding) = shortcut_binding(key, modifiers) {
                    if let Some(id) = shortcut_for_binding(&self.settings, &binding) {
                        if let Some(action) = shortcut_action(id) {
                            if !matches!(action, Message::BeginOp(_))
                                || (self.controller.has_geometry()
                                    && !self.controller.setups().is_empty()
                                    && !self.library.tools.is_empty())
                            {
                                return self.update(action);
                            }
                        }
                    }
                }
            }
            Message::SetSimulationAccuracy(value) => {
                let (lo, hi) = crate::SIMULATION_ACCURACY_RANGE;
                self.settings.simulation.accuracy = value.clamp(lo, hi);
                if let Some(player) = &mut self.simulation {
                    player.options.accuracy = self.settings.simulation.accuracy;
                }
            }
            Message::PaneResized(pane_grid::ResizeEvent { split, ratio }) => {
                let ratio = self.clamp_resize(split, ratio);
                self.panes.resize(split, ratio);
                // Persist the dragged size so a later window resize keeps it —
                // and, now, so the next run opens with the layout the user chose.
                self.capture_side_px(split, ratio);
                self.remember();
            }
            Message::PaneDragged(pane_grid::DragEvent::Dropped { pane, target }) => {
                self.panes.drop(pane, target);
            }
            Message::PaneDragged(_) => {}
            Message::WindowResized(size) => {
                self.window = size;
                // Hold the side panes + Output at their fixed pixel sizes; the
                // Viewport absorbs the change.
                self.apply_fixed_layout();
            }
            Message::ToolKindChanged(kind) => {
                // Route to whichever tool the inspector is editing: the library
                // entry (Tools tab) or the selected setup tool. Switching kind
                // resets its parameters to the new kind's defaults.
                // Straight flutes apply to flat end mills; changing to any other
                // kind falls back to a down-cut.
                let fix_dir = |t: &mut cam_model::Tool| {
                    t.kind = kind;
                    if !matches!(kind, ToolKind::EndMill | ToolKind::RoughingMill)
                        && t.cutting_direction == CutDir::Straight
                    {
                        t.cutting_direction = CutDir::Down;
                    }
                };
                if self.library_mode() {
                    // Edit the working copy (a pending change committed on Apply), not the
                    // library entry — keeps any in-progress numeric edits from being lost.
                    if let Some(t) = self.tool_edit.as_mut() {
                        // Crossing tool families resets the dimensions to the new kind's
                        // defaults (a face mill shouldn't inherit an end mill's ⌀6);
                        // staying within the end-mill family keeps the dimensions and just
                        // swaps the cutting end.
                        if end_mill_family(t.kind) && end_mill_family(kind) {
                            fix_dir(t);
                        } else {
                            *t = cam_model::Tool {
                                description: t.description.clone(),
                                manufacturer: t.manufacturer.clone(),
                                product_id: t.product_id.clone(),
                                product_link: t.product_link.clone(),
                                ..crate::tool_library::default_tool(t.number, kind)
                            };
                        }
                    }
                    // The kind-specific fields depend on the kind — repopulate them from
                    // the working copy (refresh_fields does *not* reset the baseline).
                    self.refresh_fields();
                } else if let Selection::Tool(i) = self.controller.selection() {
                    self.controller.edit_tool(i, fix_dir);
                    self.rerun();
                    self.refresh_fields();
                }
            }
            Message::ToolCuttingDirChanged(dir) => {
                if self.library_mode() {
                    if let Some(t) = self.tool_edit.as_mut() {
                        t.cutting_direction = dir;
                    }
                    self.refresh_operation_fields();
                } else if let Selection::Tool(i) = self.controller.selection() {
                    self.controller.edit_tool(i, |t| t.cutting_direction = dir);
                    self.rerun();
                    self.refresh_fields();
                }
            }
            Message::ThreadFormChanged(form) => {
                // Single-point ⇒ pitch None + a reduced neck (the single-tooth head);
                // full-form ⇒ Some(pitch), a stacked-teeth band (neck ⌀ unused).
                let set_form = |t: &mut cam_model::Tool| {
                    if let ToolKind::ThreadMill { pitch } = &mut t.kind {
                        match form {
                            ThreadForm::SinglePoint => {
                                *pitch = None;
                                if t.neck_diameter <= 0.0 || t.neck_diameter >= t.diameter {
                                    t.neck_diameter = (t.diameter * 0.7).max(0.1);
                                }
                            }
                            ThreadForm::FullForm => *pitch = Some(pitch.unwrap_or(1.0)),
                        }
                    }
                };
                if self.library_mode() {
                    if let Some(t) = self.tool_edit.as_mut() {
                        set_form(t);
                    }
                    self.refresh_operation_fields();
                } else if let Selection::Tool(i) = self.controller.selection() {
                    self.controller.edit_tool(i, set_form);
                    self.rerun();
                    self.refresh_fields();
                }
            }
            Message::LeadInKindChanged(kind) => {
                self.edit_operation_draft(|op| match op {
                    Operation::Profile(p) => p.lead_in = kind.to_lead(p.lead_in),
                    Operation::Chamfer(c) => c.lead_in = kind.to_lead(c.lead_in),
                    Operation::Pocket(p) => p.clear.lead_in = kind.to_lead(p.clear.lead_in),
                    _ => {}
                });
                self.refresh_operation_fields();
            }
            Message::LeadOutKindChanged(kind) => {
                self.edit_operation_draft(|op| match op {
                    Operation::Profile(p) => p.lead_out = kind.to_lead(p.lead_out),
                    Operation::Chamfer(c) => c.lead_out = kind.to_lead(c.lead_out),
                    Operation::Pocket(p) => p.clear.lead_out = kind.to_lead(p.clear.lead_out),
                    _ => {}
                });
                self.refresh_operation_fields();
            }
            Message::ChamferExitEnabled(enabled) => {
                let parsed = self
                    .fields
                    .iter()
                    .filter_map(|(f, s)| {
                        s.parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .map(|v| (*f, v))
                    })
                    .collect();
                self.edit_operation_draft(|op| {
                    apply_op_fields(op, &parsed);
                    if let Operation::Chamfer(c) = op {
                        c.lead_out_enabled = Some(enabled);
                        if enabled && c.lead_out == Lead::None {
                            c.lead_out = Lead::Arc { radius: 0.6 };
                        }
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ChamferEntryEnabled(enabled) => {
                let parsed = self
                    .fields
                    .iter()
                    .filter_map(|(f, s)| {
                        s.parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .map(|v| (*f, v))
                    })
                    .collect();
                self.edit_operation_draft(|op| {
                    apply_op_fields(op, &parsed);
                    if let Operation::Chamfer(c) = op {
                        c.lead_in_enabled = Some(enabled);
                        if !enabled && c.same_lead_out && c.lead_out == Lead::None {
                            c.lead_out = if c.lead_in == Lead::None {
                                Lead::Arc { radius: 0.6 }
                            } else {
                                c.lead_in
                            };
                        }
                        if enabled && c.lead_in == Lead::None {
                            c.lead_in = Lead::Arc { radius: 0.6 };
                        }
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ChamferSameLeadOut(same) => {
                let parsed = self
                    .fields
                    .iter()
                    .filter_map(|(f, s)| {
                        s.parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .map(|v| (*f, v))
                    })
                    .collect();
                self.edit_operation_draft(|op| {
                    apply_op_fields(op, &parsed);
                    if let Operation::Chamfer(c) = op {
                        let entry_enabled =
                            c.lead_in != Lead::None && c.lead_in_enabled.unwrap_or(true);
                        let exit_enabled = c.lead_out_enabled.unwrap_or(true)
                            && (c.lead_out != Lead::None || (c.same_lead_out && entry_enabled));
                        if !same && c.same_lead_out && exit_enabled && c.lead_out == Lead::None {
                            c.lead_out = c.lead_in;
                        }
                        c.same_lead_out = same;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::PlungeKindChanged(kind) => {
                self.edit_operation_draft(|op| match op {
                    Operation::Profile(p) => p.plunge = kind.to_plunge(),
                    Operation::Pocket(p) => p.clear.plunge = kind.to_plunge(),
                    // The V-bit's own entry — `CarveClearPlungeChanged` is the clearing
                    // pass's, and the two must not be crossed.
                    Operation::Carve(c) => c.plunge = kind.to_plunge(),
                    _ => {}
                });
                self.refresh_operation_fields();
            }
            Message::FaceDirectionChanged(axis) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Face(f) = op {
                        f.direction = axis;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ChamferGradualToggled(on) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Chamfer(c) = op {
                        c.gradual = on;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ThreadGradualToggled(on) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Thread(t) = op {
                        t.gradual = on;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ClearingClimbToggled(on) => {
                self.edit_operation_draft(|op| match op {
                    Operation::Pocket(p) => p.clear.clearing.climb = on,
                    Operation::Profile(p) => {
                        p.clearing.climb = on;
                    }
                    _ => {}
                });
                self.refresh_operation_fields();
            }
            Message::ParallelBothWaysToggled(on) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Parallel(p) = op {
                        p.params.both_ways = on;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ParallelScallopToggled(on) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Parallel(p) = op {
                        p.params.scallop = on;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ParallelTransitionChanged(transition) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Parallel(p) = op {
                        p.params.transition = transition;
                    }
                });
            }
            Message::ScallopSmoothingToggled(on) => {
                self.edit_operation_draft(|op| match op {
                    Operation::Parallel(p) => p.params.scallop_smoothing = on,
                    Operation::Spiral(p) => p.params.smoothing = on,
                    _ => {}
                });
                self.refresh_operation_fields();
            }
            Message::ScallopRetractionChanged(policy) => {
                self.edit_operation_draft(|op| match op {
                    Operation::Parallel(p) if p.params.scallop => p.params.scallop_retraction = policy,
                    Operation::Parallel(p) => p.params.retraction = policy,
                    Operation::Spiral(p) => p.params.retraction = policy,
                    _ => {}
                });
                self.refresh_operation_fields();
            }
            Message::ScallopOrderChanged(order) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Parallel(p) = op {
                        p.params.scallop_order = order;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::SpiralModeChanged(value) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Spiral(p) = op {
                        p.params.mode = value;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::SpiralOrderChanged(value) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Spiral(p) = op {
                        p.params.order = value;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::SpiralMillingChanged(value) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Spiral(p) = op {
                        p.params.milling = value;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::SpiralCutDirectionChanged(value) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Spiral(p) = op {
                        p.params.cut_direction = value;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::SpiralBoundaryChanged(value) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Spiral(p) = op {
                        p.params.boundary = value;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::SpiralContactOnlyToggled(value) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Spiral(p) = op {
                        p.params.contact_only = value;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::SpiralDepthsToggled(value) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Spiral(p) = op {
                        p.params.multiple_depths = value;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::SpiralDepthOrderToggled(value) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Spiral(p) = op {
                        p.params.order_by_depth = value;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::SpiralLinkInsideOutToggled(value) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Spiral(p) = op {
                        p.params.link_inside_out = value;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::SpiralClockwiseToggled(value) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Spiral(p) = op {
                        p.params.clockwise = value;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::PocketKeepToolDownToggled(on) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Pocket(p) = op {
                        p.clear.keep_tool_down = on;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::AdaptiveClimbToggled(on) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Adaptive(adaptive) = op {
                        adaptive.params.climb = on;
                    }
                });
                self.refresh_operation_fields();
            }
            Message::AdaptiveContainmentChanged(value) => {
                self.edit_operation_draft(|op| match op {
                    Operation::Adaptive(p) => p.params.containment = value,
                    Operation::Parallel(p) => p.params.containment = value,
                    Operation::Spiral(p) => p.params.containment = value,
                    _ => {}
                });
                self.refresh_operation_fields();
            }
            Message::SideChanged(side) => {
                if self.profile_geometry_active() {
                    self.set_profile_contour_side(self.profile_contour_selection, side);
                } else {
                    self.edit_operation_draft(|op| {
                        if let Operation::Chamfer(c) = op {
                            c.side = side;
                        }
                    });
                    self.refresh_operation_fields();
                }
            }
            Message::ProfileContourSideCycle(index) => {
                if let Some((profile, _)) = self.profile_selection_geometry() {
                    if let Some(contour) = profile.contours().nth(index) {
                        let next = match contour.side {
                            Side::Outside => Side::Inside,
                            Side::Inside => Side::On,
                            Side::On => Side::Outside,
                        };
                        self.set_profile_contour_side(index, next);
                    }
                }
            }
            Message::BoreOption(option) => {
                // Preserve edits before a toggle replaces the visible pitch/angle
                // or lead-radius field. Seed newly revealed fields from the draft.
                let values = self
                    .fields
                    .iter()
                    .filter_map(|(field, text)| {
                        text.parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .map(|v| (*field, v))
                    })
                    .collect();
                self.edit_operation_draft(|op| {
                    apply_op_fields(op, &values);
                    if let Operation::Bore(b) = op {
                        match option {
                            BoreOption::Internal(v) => b.internal = v,
                            BoreOption::Climb(v) => b.climb = v,
                            BoreOption::Angle(v) => b.ramp_angle = v.then_some(3.),
                            BoreOption::Repeat(v) => b.repeat_pass = v,
                            BoreOption::Center(v) => b.lead_to_center = v,
                            BoreOption::Reverse(v) => b.reverse_order = v,
                            BoreOption::Order(v) => b.order = v,
                        }
                    }
                });
                self.refresh_operation_fields();
            }
            Message::ThreadInternalChanged(internal) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Thread(t) = op {
                        t.internal = internal;
                    }
                });
            }
            Message::ThreadHandChanged(hand) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Thread(t) = op {
                        t.hand = hand;
                    }
                });
            }
            Message::ThreadParameters(message) => self.thread_parameters_message(message),
            Message::ThreadClimbChanged(climb) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Thread(t) = op {
                        t.climb = climb;
                    }
                });
            }
            Message::CarveStayDownToggled(on) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Carve(c) = op {
                        c.stay_down = on;
                    }
                });
            }
            Message::CarveClearToggled(on) => {
                // Turning it on seeds the first flat-bottomed tool in the library, so
                // the operator gets a working clearing pass from one click and refines
                // it after. Turning it off leaves the carve V-bit-only (and ridged).
                let seed = on
                    .then(|| {
                        self.library
                            .tools
                            .iter()
                            .find(|t| {
                                matches!(
                                    t.kind,
                                    ToolKind::EndMill
                                        | ToolKind::RoughingMill
                                        | ToolKind::BullNose { .. }
                                )
                            })
                            .cloned()
                    })
                    .flatten()
                    .map(|t| self.stage_operation_tool(t));
                self.edit_operation_draft(|op| {
                    if let Operation::Carve(c) = op {
                        c.clear = match (on, seed) {
                            // Seed the feeds from the carve's own, so the pass is
                            // runnable from one click; they are editable straight after.
                            (true, Some(tool)) => Some(CarveClearing {
                                tool,
                                params: ClearParams {
                                    multiple_depths: Default::default(),
                                    adaptive_enabled: None,
                                    pocket_overlap: None,
                                    feed: c.feed,
                                    plunge_feed: c.plunge_feed,
                                    ..Default::default()
                                },
                            }),
                            _ => None,
                        };
                    }
                });
                // The visible field set can change with this choice, so reseed the
                // buffers -- a newly revealed field would otherwise render blank.
                self.refresh_operation_fields();
                if on {
                    self.seed_user_clearing_options();
                }
            }
            Message::CarveClearToolChanged(index) => {
                let Some(tool) = self.library.tools.get(index).cloned() else {
                    return iced::Task::none();
                };
                let number = self.stage_operation_tool(tool);
                self.edit_operation_draft(|op| {
                    if let Operation::Carve(c) = op {
                        // Keep the parameters; only the cutter changes.
                        match &mut c.clear {
                            Some(cl) => cl.tool = number,
                            none => {
                                *none = Some(CarveClearing {
                                    tool: number,
                                    params: ClearParams::default(),
                                })
                            }
                        }
                    }
                });
            }
            Message::CarveClearPlungeChanged(kind) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Carve(c) = op {
                        if let Some(cl) = &mut c.clear {
                            cl.params.plunge = kind.to_plunge();
                        }
                    }
                });
                // The visible field set can change with this choice, so reseed the
                // buffers -- a newly revealed field would otherwise render blank.
                self.refresh_operation_fields();
            }
            Message::CarveClearClimbToggled(on) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Carve(c) = op {
                        if let Some(cl) = &mut c.clear {
                            cl.params.clearing.climb = on;
                        }
                    }
                });
            }
            Message::CarveClearLeadInChanged(kind) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Carve(c) = op {
                        if let Some(cl) = &mut c.clear {
                            cl.params.lead_in = kind.to_lead(cl.params.lead_in);
                        }
                    }
                });
                // The visible field set can change with this choice, so reseed the
                // buffers -- a newly revealed field would otherwise render blank.
                self.refresh_operation_fields();
            }
            Message::CarveClearLeadOutChanged(kind) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Carve(c) = op {
                        if let Some(cl) = &mut c.clear {
                            cl.params.lead_out = kind.to_lead(cl.params.lead_out);
                        }
                    }
                });
                // The visible field set can change with this choice, so reseed the
                // buffers -- a newly revealed field would otherwise render blank.
                self.refresh_operation_fields();
            }
            Message::LibraryPresetSelected(id) => self.select_library_preset(Some(id)),
            Message::EditLibraryPreset(id) => {
                self.select_library_preset(Some(id));
                self.begin_preset_edit();
            }
            Message::AddPreset | Message::DuplicatePreset => {
                let duplicate = matches!(message, Message::DuplicatePreset);
                self.begin_preset_edit();
                self.add_library_preset(duplicate);
                self.show_preset_editor = self.library_preset.is_some();
            }
            Message::EditPreset => self.begin_preset_edit(),
            Message::ClosePresetEditor => {
                if !self.preset_edit_invalid() {
                    self.show_preset_editor = false;
                    self.edited_preset_input = None;
                    self.preset_editor_snapshot = None;
                }
            }
            Message::CancelPresetEditor => self.cancel_preset_edit(),
            Message::Noop => {}
            Message::OpenMaterials => self.show_materials = true,
            Message::CloseMaterials => {
                // Valid entries are saved immediately; abandon incomplete input on close.
                self.material_force_inputs.clear();
                self.editing_material = None;
                self.creating_material = false;
                self.show_materials = false;
            }
            Message::ToolDetailChanged(field, value) => {
                if let Some(tool) = &mut self.tool_edit {
                    match field {
                        tool_presets::ToolDetail::Description => tool.description = value,
                        tool_presets::ToolDetail::Manufacturer => tool.manufacturer = value,
                        tool_presets::ToolDetail::ProductId => tool.product_id = value,
                        tool_presets::ToolDetail::ProductLink => tool.product_link = value,
                    }
                }
            }
            Message::DeletePreset => self.delete_library_preset(),
            Message::ToolCoolingChanged(cooling) => {
                if let Some(tool) = &mut self.tool_edit {
                    tool.cooling = cooling;
                }
            }
            Message::PresetCoolingChanged(cooling) => {
                if let Some(preset) = self.library_preset_mut() {
                    preset.cooling = cooling;
                }
            }
            Message::OperationCoolingChanged(cooling) => {
                self.edit_operation_draft(|op| op.set_cooling(cooling));
            }
            Message::PresetNameChanged(name) => {
                if let Some(preset) = self.library_preset_mut() {
                    preset.name = name;
                }
            }
            Message::PresetMaterialChanged(material) => {
                if let Some(preset) = self.library_preset_mut() {
                    preset.material_id = material.id;
                }
            }
            Message::PresetValueChanged(field, value) => {
                self.edit_preset_value(field, value);
            }
            Message::OperationPresetPicked(choice) => self.pick_operation_preset(choice.id),
            Message::SetupMaterialPicked(choice) => {
                if let Some(draft) = &mut self.setup_draft {
                    draft.material_id = choice.id;
                    draft.material = if choice.id.is_some() {
                        choice.name
                    } else {
                        String::new()
                    };
                }
            }
            Message::SelectMaterial(id) => {
                self.selected_material = Some(id);
                self.editing_material = None;
                self.creating_material = false;
            }
            Message::NewMaterial => {
                self.creating_material = true;
                self.editing_material = None;
                self.new_material_name.clear();
            }
            Message::EditMaterialRow(id) => {
                self.selected_material = Some(id);
                self.editing_material = Some(id);
                self.creating_material = false;
            }
            Message::EditMaterial => {
                self.editing_material = self.selected_material;
                self.creating_material = false;
            }
            Message::NewMaterialNameChanged(name) => self.new_material_name = name,
            Message::AddMaterial => self.add_managed_material(),
            Message::AddStarterMaterials => {
                let count = self.library.add_starter_materials();
                if count > 0 {
                    self.save_tool_libraries();
                }
                self.status = format!("Added {count} starter material(s).");
            }
            Message::MaterialRenamed(id, name) => self.rename_managed_material(id, name),
            Message::MaterialCuttingForceChanged(id, value) => {
                self.edit_material_cutting_force(id, value)
            }
            Message::CuttingForceCustomToggled(id, enabled) => {
                self.toggle_custom_cutting_force(id, enabled)
            }
            Message::CuttingForceInputChanged(id, index, value) => {
                let key = self.cutting_force_key(id);
                if let Some(inputs) = self.cutting_force_inputs.get_mut(&key) {
                    if let Some(input) = inputs.get_mut(index) {
                        *input = value;
                    }
                }
            }
            Message::DeleteMaterial(id) => self.delete_managed_material(id),
            Message::NewTool => {
                // Add a tool to the library and select it for editing. Its **type**
                // seeds from the currently-selected tool (a chamfer mill begets a
                // chamfer mill), defaulting to an end mill when nothing is selected. If
                // an op wizard is active, also embed it and pick it for the pending op.
                let seed_kind = self
                    .library
                    .tools
                    .get(self.lib_sel)
                    .map(|t| t.kind)
                    .unwrap_or(ToolKind::EndMill);
                self.lib_sel = self.library.add_of_kind(seed_kind);
                self.save_tool_libraries();
                if self.controller.pending_op().is_some() {
                    if let Some(tool) = self.library.tools.get(self.lib_sel).cloned() {
                        let number = self.controller.use_tool(tool);
                        self.controller.set_pending_tool(number);
                    }
                }
                self.reload_tool_edit();
            }
            Message::DeleteTool => {
                if self.lib_sel < self.library.tools.len() && self.library.tools.len() > 1 {
                    self.library.tools.remove(self.lib_sel);
                    self.lib_sel = self.lib_sel.min(self.library.tools.len() - 1);
                    self.save_tool_libraries();
                    self.reload_tool_edit();
                }
            }
            Message::ToolMenu(number) => {
                self.open_tool_menu = Some(number);
                self.tool_menu_pos = self.window_cursor;
            }
            Message::CloseToolMenu => self.open_tool_menu = None,
            Message::AddToolToLibrary(number) => {
                // Promote the project tool numbered `number` into the shop library, then
                // reconcile so it adopts the shop number (§6.3). Idempotent by geometry.
                self.open_tool_menu = None;
                let tool = self
                    .controller
                    .inspection_setup()
                    .tools
                    .iter()
                    .find(|t| t.number == number)
                    .cloned();
                if let Some(tool) = tool {
                    self.library.add_tool(tool);
                    self.save_tool_libraries();
                    let report = self.controller.reconcile_tools(&self.library.tools);
                    self.refresh_fields();
                    self.status = match report.summary() {
                        Some(s) => format!("Added tool to library. {s}."),
                        None => "Added tool to library.".to_string(),
                    };
                }
            }
            Message::SelectLibraryTool(i) => self.select_library_tool(i),
            Message::SetLibraryView(view) => {
                self.library_view = view;
                self.remember_tool_table();
            }
            Message::ToolColumnResizeStart(column) => {
                self.tool_table_drag = Some(workspace_view::ToolColumnDrag::Resize {
                    column,
                    start_x: self.window_cursor.x,
                    width: self.tool_table_widths[column],
                });
            }
            Message::ToolColumnReorderStart(column) => {
                self.tool_table_drag = Some(workspace_view::ToolColumnDrag::Reorder {
                    column,
                    start: self.window_cursor,
                    target: None,
                });
            }
            Message::ToolColumnHover(column, after) => {
                if let Some(workspace_view::ToolColumnDrag::Reorder { target, .. }) =
                    &mut self.tool_table_drag
                {
                    *target = Some((column, after));
                }
            }
            Message::ToolColumnLeave(column) => {
                if let Some(workspace_view::ToolColumnDrag::Reorder { target, .. }) =
                    &mut self.tool_table_drag
                {
                    if target.is_some_and(|(hover, _)| hover == column) {
                        *target = None;
                    }
                }
            }
            Message::ToolColumnRelease => {
                if self.library_sidebar_drag.take().is_some() {
                    self.settings.save();
                }
                let had_drag = self.tool_table_drag.is_some();
                if let Some(workspace_view::ToolColumnDrag::Reorder {
                    column,
                    start,
                    target: Some((target, after)),
                }) = self.tool_table_drag.take()
                {
                    if (self.window_cursor.x - start.x).abs() >= 5.0 {
                        workspace_view::reorder_tool_column(
                            &mut self.tool_table_order,
                            column,
                            target,
                            after,
                        );
                    }
                }
                if had_drag {
                    self.remember_tool_table();
                }
            }
            Message::OpenToolTableMenu(index, visibility) => {
                if index < 8 || (visibility && index == 8) {
                    self.tool_table_menu = Some((index, visibility));
                    self.tool_table_menu_pos = self.window_cursor;
                    self.tool_table_menu_search.clear();
                    self.tool_table_filter_draft = if index < 8 {
                        self.tool_table_filters[index]
                            .clone()
                            .unwrap_or_else(|| self.tool_table_values(index))
                    } else {
                        BTreeSet::new()
                    };
                }
            }
            Message::CloseToolTableMenu => self.tool_table_menu = None,
            Message::ToolTableMenuSearch(value) => self.tool_table_menu_search = value,
            Message::ToolTableFilterValue(value, checked) => {
                if checked {
                    self.tool_table_filter_draft.insert(value);
                } else {
                    self.tool_table_filter_draft.remove(&value);
                }
            }
            Message::ToolTableFilterAll(checked) => {
                if let Some((index, _)) = self.tool_table_menu {
                    let search = self.tool_table_menu_search.trim().to_lowercase();
                    for value in self.tool_table_values(index) {
                        if value.to_lowercase().contains(&search) {
                            if checked {
                                self.tool_table_filter_draft.insert(value);
                            } else {
                                self.tool_table_filter_draft.remove(&value);
                            }
                        }
                    }
                }
            }
            Message::ApplyToolTableFilter => {
                if let Some((index, _)) = self.tool_table_menu {
                    let all = self.tool_table_values(index);
                    self.tool_table_filters[index] = if all == self.tool_table_filter_draft {
                        None
                    } else {
                        Some(self.tool_table_filter_draft.clone())
                    };
                }
                self.tool_table_menu = None;
                self.remember_tool_table();
            }
            Message::ClearToolTableColumnFilter => {
                if let Some((index, _)) = self.tool_table_menu {
                    self.tool_table_filters[index] = None;
                }
                self.tool_table_menu = None;
                self.remember_tool_table();
            }
            Message::SetToolTableColumn(index, visible) => {
                if let Some(column) = self.tool_table_columns.get_mut(index) {
                    *column = visible;
                    self.remember_tool_table();
                }
            }
            Message::ToolTableSearch(value) => {
                self.tool_table_search = value;
                self.remember_tool_table();
            }
            Message::ClearToolTableFilters => {
                self.tool_table_search.clear();
                self.tool_table_filters = Default::default();
                self.remember_tool_table();
            }
            Message::LibToolMenu(i) => {
                self.select_library_tool(i);
                self.lib_menu = Some(LibMenu {
                    index: i,
                    input: None,
                });
                self.lib_menu_pos = self.window_cursor;
            }
            Message::LibMenuSetNumber => {
                if let Some(menu) = &mut self.lib_menu {
                    let cur = self
                        .library
                        .tools
                        .get(menu.index)
                        .map(|t| t.number.to_string())
                        .unwrap_or_default();
                    menu.input = Some(cur);
                }
            }
            Message::LibNumberInput(s) => {
                if let Some(menu) = &mut self.lib_menu {
                    menu.input = Some(s);
                }
            }
            Message::LibNumberCommit => {
                if let Some(menu) = &self.lib_menu {
                    if let Some(n) = menu
                        .input
                        .as_ref()
                        .and_then(|s| s.trim().parse::<u32>().ok())
                    {
                        self.library.set_number(menu.index, n);
                        self.save_tool_libraries();
                        self.reload_tool_edit();
                    }
                }
                self.lib_menu = None;
            }
            Message::CloseLibMenu => self.lib_menu = None,
            Message::RenumberLibrary => {
                let n = self.library.tools.len();
                let by = match self.library_view {
                    LibraryView::Ordered => "current order (compacting any gaps)",
                    LibraryView::Grouped => "family, then diameter",
                };
                let detail = format!(
                    "Renumber all {n} library tools sequentially (T1…T{n}) by {by}?\n\n\
                     This changes every tool's number and cannot be undone. Open projects \
                     re-align to the new numbering when next opened.",
                );
                return iced::Task::perform(confirm_renumber(detail), Message::RenumberConfirmed);
            }
            Message::RenumberConfirmed(true) => {
                let order = self.renumber_order();
                self.library.set_numbers_in_order(&order);
                self.save_tool_libraries();
                self.reload_tool_edit();
                self.status = "Renumbered the tool library.".to_string();
            }
            Message::RenumberConfirmed(false) => {}
            Message::OpMenu(id) => {
                self.inspector_active = false;
                self.pending_simulation = None;
                self.pending_processing_time = None;
                // Context-clicking a member of the current highlight set keeps the
                // whole set available to group commands; a fresh row establishes a
                // one-operation selection.
                if !self.focus_ops.contains(&id) {
                    self.controller.select(Selection::Operation(id));
                    self.focus_ops.clear();
                    self.focus_ops.insert(id);
                } else {
                    self.controller.select(Selection::Operation(id));
                }
                self.end_setup_draft();
                self.operation_draft = None;
                self.operation_tools.clear();
                self.operation_geometry = None;
                self.operation_binding_preview = None;
                self.model_view = None;
                self.refresh_fields();
                self.open_op_menu = Some(id);
                self.op_menu_pos = self.window_cursor;
            }
            Message::CloseOpMenu => {
                self.open_op_menu = None;
                self.tree_menu = None;
            }
            Message::WindowCursor(p) => {
                self.window_cursor = p;
                self.tree_pointer_moved(p);
                if let Some((start_x, width)) = self.library_sidebar_drag {
                    self.settings.tool_table.library_sidebar_width =
                        (width + p.x - start_x).clamp(180.0, 480.0);
                }
                if let Some(workspace_view::ToolColumnDrag::Resize {
                    column,
                    start_x,
                    width,
                }) = self.tool_table_drag
                {
                    self.tool_table_widths[column] = (width + p.x - start_x).clamp(55.0, 600.0);
                }
            }
            Message::SelectRibbonTab(tab) => {
                let target = if tab == RibbonTab::Tools {
                    Workspace::Tools
                } else {
                    Workspace::Machining
                };
                self.switch_workspace(target);
                if self.workspace != target {
                    return iced::Task::none();
                }
                self.active_tab = tab;
                if tab == RibbonTab::Machinery {
                    self.controller.select(Selection::Machine);
                    self.refresh_fields();
                }
            }
            Message::ToggleRibbonGroup(i) => {
                self.open_group = if self.open_group == Some(i) {
                    None
                } else {
                    Some(i)
                };
            }
            // The preamble already cleared it; the arm keeps the match exhaustive.
            Message::CloseRibbonPopup => {}
            Message::CloseRequested(_) | Message::DiscardConfirmed | Message::DiscardCancelled => {
                unreachable!()
            }
            Message::OutputAction(_) | Message::OutputCopy | Message::OutputClear => unreachable!(),
            Message::SetPaneVisible(pane, show) => {
                if pane == Pane::Inspector {
                    self.inspector_enabled = show;
                } else {
                    self.set_pane_visible(pane, show);
                }
                self.remember();
            }
        }
        iced::Task::none()
    }

    fn subscription(&self) -> iced::Subscription<Message> {
        iced::Subscription::batch([
            iced::window::close_requests().map(Message::CloseRequested),
            if self.mcp.is_some() {
                iced::time::every(std::time::Duration::from_millis(25)).map(|_| Message::PollMcp)
            } else {
                iced::Subscription::none()
            },
            if self
                .simulation
                .as_ref()
                .is_some_and(SimulationPlayer::needs_tick)
            {
                iced::time::every(std::time::Duration::from_millis(33))
                    .map(|_| Message::SimulationTick)
            } else {
                iced::Subscription::none()
            },
            if self.controller.run_needs_poll() || self.operation_preview.needs_poll() {
                iced::time::every(std::time::Duration::from_millis(50)).map(|_| Message::PollRun)
            } else {
                iced::Subscription::none()
            },
            if self.controller.cad_import_pending() {
                iced::time::every(std::time::Duration::from_millis(100))
                    .map(|_| Message::PollCadImport)
            } else {
                iced::Subscription::none()
            },
            self.preferences
                .as_ref()
                .map_or_else(iced::Subscription::none, |prefs| {
                    window_events(prefs.id, preferences::keyboard_event)
                }),
            self.main_window
                .map_or_else(iced::Subscription::none, |id| {
                    iced::Subscription::batch([
                        window_events(id, Self::main_window_event),
                        window_events(id, |event, _| match event {
                            iced::Event::Mouse(iced::mouse::Event::ButtonReleased(
                                iced::mouse::Button::Left,
                            ))
                            | iced::Event::Keyboard(iced::keyboard::Event::KeyReleased {
                                ..
                            }) => Some(Message::CheckHeightFocus),
                            _ => None,
                        }),
                        window_events(id, |event, _| match event {
                            iced::Event::Mouse(iced::mouse::Event::ButtonReleased(
                                iced::mouse::Button::Left,
                            )) => Some(Message::CheckOperationRenameFocus),
                            _ => None,
                        }),
                        window_events(id, |event, _| match event {
                            iced::Event::Mouse(iced::mouse::Event::ButtonReleased(
                                iced::mouse::Button::Left,
                            )) => Some(Message::ToolColumnRelease),
                            _ => None,
                        }),
                    ])
                }),
        ])
    }

    fn main_window_event(event: iced::Event, status: iced::event::Status) -> Option<Message> {
        match event {
            iced::Event::Window(iced::window::Event::Resized(size)) => {
                Some(Message::WindowResized(size))
            }
            // Track the window-absolute cursor so overlays anchor under the pointer.
            iced::Event::Mouse(iced::mouse::Event::CursorMoved { position }) => {
                Some(Message::WindowCursor(position))
            }
            iced::Event::Mouse(iced::mouse::Event::ButtonReleased(iced::mouse::Button::Left)) => {
                Some(Message::Tree(TreeMessage::Release))
            }
            iced::Event::Keyboard(iced::keyboard::Event::KeyPressed {
                key: iced::keyboard::Key::Character(key),
                modifiers,
                repeat: false,
                ..
            }) if status == iced::event::Status::Ignored => {
                Some(Message::ShortcutKey(key.chars().next(), modifiers))
            }
            iced::Event::Keyboard(iced::keyboard::Event::KeyPressed {
                key: iced::keyboard::Key::Named(iced::keyboard::key::Named::Tab),
                modifiers,
                ..
            }) if status == iced::event::Status::Ignored
                && !modifiers.control()
                && !modifiers.alt()
                && !modifiers.logo() =>
            {
                Some(Message::TraverseFocus(modifiers.shift()))
            }
            iced::Event::Keyboard(iced::keyboard::Event::KeyPressed {
                key: iced::keyboard::Key::Named(iced::keyboard::key::Named::F8),
                ..
            }) => Some(Message::ToggleStock),
            iced::Event::Keyboard(iced::keyboard::Event::KeyPressed {
                key: iced::keyboard::Key::Named(iced::keyboard::key::Named::Escape),
                ..
            }) => Some(Message::CadEscape),
            // Live modifiers, so an op-row click can tell plain from ⌘/Ctrl.
            iced::Event::Keyboard(iced::keyboard::Event::ModifiersChanged(m)) => {
                Some(Message::ModifiersChanged(m))
            }
            _ => None,
        }
    }

    /// Clamp a split's new `ratio` so neither side falls below its panes' pixel
    /// minimums (pane_grid only offers one global floor, so we do it per pane).
    fn clamp_resize(&self, split: pane_grid::Split, ratio: f32) -> f32 {
        let bounds = iced::Size::new(
            self.window.width,
            (self.window.height - MENU_BAR_H).max(1.0),
        );
        let regions = self
            .panes
            .layout()
            .split_regions(PANE_SPACING, PANE_MIN_PX, bounds);
        let Some(&(axis, rect, _)) = regions.get(&split) else {
            return ratio;
        };
        let dim = match axis {
            pane_grid::Axis::Vertical => rect.width,
            pane_grid::Axis::Horizontal => rect.height,
        };
        let Some((a, b)) = find_split(self.panes.layout(), split) else {
            return ratio;
        };
        if dim <= 1.0 {
            return ratio;
        }
        let mins = &self.settings.panes;
        let lo = (subtree_min(a, &self.panes, axis, mins) / dim).clamp(0.0, 0.95);
        let hi = (1.0 - subtree_min(b, &self.panes, axis, mins) / dim).clamp(lo, 1.0);
        ratio.clamp(lo, hi)
    }

    /// The grid handle of `pane`, if it's currently shown.
    fn pane_handle(&self, pane: Pane) -> Option<pane_grid::Pane> {
        self.panes
            .iter()
            .find(|(_, p)| **p == pane)
            .map(|(h, _)| *h)
    }

    /// Close the pane itself so an idle inspector gives its space to the viewport.
    /// Creation and geometry picking belong to the active operation editor.
    fn sync_inspector_visibility(&mut self) {
        let editing = self.move_draft.is_some()
            || self.simulation.is_some()
            || self.pattern_draft.is_some()
            || self.library_mode()
            || (self.active_tab == RibbonTab::Machinery
                && self.controller.selection() == Selection::Machine)
            || self.creating_operation()
            || self.cad_editing_op.is_some()
            || self.drawing_editing_op.is_some()
            || (self.controller.selection() == Selection::Setup && self.setup_draft.is_some())
            || (self.inspector_active
                && matches!(self.controller.selection(), Selection::Operation(_)));
        let show = self.inspector_enabled && editing;
        if show != self.pane_handle(Pane::Inspector).is_some() {
            self.set_pane_visible(Pane::Inspector, show);
        }
    }

    /// Restore user choices independently of the inspector's temporary visibility.
    fn restore_pane_preferences(&mut self) {
        self.show_project_browser = self.settings.panes.show_project_browser;
        self.inspector_enabled = self.settings.panes.inspector_enabled;
        self.set_pane_visible(Pane::Output, self.settings.panes.show_output);
        let workspace = if self.settings.panes.show_library {
            Workspace::Tools
        } else {
            Workspace::Machining
        };
        if self.workspace != workspace {
            self.workspace = workspace;
            self.active_tab = if workspace == Workspace::Tools {
                RibbonTab::Tools
            } else {
                self.machining_tab
            };
            self.reload_tool_edit();
        }
    }

    /// Show or hide a pane. The Viewport is always visible (no-op). Hiding closes a
    /// pane; showing splits it back off an existing pane and docks it to its fixed
    /// edge. Either way the fixed layout is re-derived so the remaining side panes
    /// keep their sizes and the Viewport takes the rest.
    fn set_pane_visible(&mut self, pane: Pane, show: bool) {
        if pane == Pane::Project {
            self.show_project_browser = show;
            return;
        }
        if pane == Pane::Library {
            self.switch_workspace(if show {
                Workspace::Tools
            } else {
                Workspace::Machining
            });
            return;
        }
        if pane == Pane::Viewport {
            return;
        }
        match (show, self.pane_handle(pane)) {
            (false, Some(handle)) => {
                self.panes.close(handle);
            }
            (true, None) => {
                let anchor = self.panes.iter().next().map(|(h, _)| *h);
                if let Some(anchor) = anchor {
                    if let Some((new_pane, _)) =
                        self.panes.split(pane_grid::Axis::Vertical, anchor, pane)
                    {
                        if let Some(edge) = pane.dock_edge() {
                            self.panes.move_to_edge(new_pane, edge);
                        }
                    }
                }
            }
            _ => {}
        }
        self.apply_fixed_layout();
    }

    /// Resize the split that bounds `pane` so `pane` occupies `px` pixels along its
    /// split axis, handing the remainder to its sibling (the Viewport side). The
    /// pixel math mirrors `maximize_pane`; `px` is floored at the pane's minimum.
    fn set_pane_px(&mut self, pane: Pane, px: f32) {
        let Some(handle) = self.pane_handle(pane) else {
            return;
        };
        let Some(path) = splits_to_pane(self.panes.layout(), handle) else {
            return;
        };
        // The pane's own boundary is the deepest split on its path.
        let Some(&(split, in_a)) = path.last() else {
            return;
        };
        let bounds = iced::Size::new(
            self.window.width,
            (self.window.height - MENU_BAR_H).max(1.0),
        );
        let regions = self
            .panes
            .layout()
            .split_regions(PANE_SPACING, PANE_MIN_PX, bounds);
        let Some(&(axis, rect, _)) = regions.get(&split) else {
            return;
        };
        let dim = match axis {
            pane_grid::Axis::Vertical => rect.width,
            pane_grid::Axis::Horizontal => rect.height,
        };
        if dim <= 1.0 {
            return;
        }
        let frac = (px.max(pane.min_size(&self.settings.panes)) / dim).clamp(0.0, 1.0);
        let ratio = if in_a { frac } else { 1.0 - frac };
        let ratio = self.clamp_resize(split, ratio);
        self.panes.resize(split, ratio);
    }

    /// Write the controller's machine and post back into the library entry, following a
    /// rename, and persist.
    ///
    /// The Machine inspector edits the *active* machine in place, and the name is the
    /// handle the selection is remembered by — so a rename has to move the entry and the
    /// remembered selection together, or the next launch resolves nothing.
    fn sync_active_machine(&mut self) {
        let entry = crate::MachineEntry {
            machine: self.controller.machine().clone(),
            post: self.controller.post_kind(),
            estlcam: self.controller.estlcam_options(),
        };
        let settled = self.machines.replace(&self.active_machine, entry);
        // `replace` keeps names unique, so a rename onto a name already taken comes back
        // disambiguated. Push that back, or the inspector would show one name while the
        // library held another — and the selection is remembered by the library's.
        if settled != self.controller.machine().name {
            self.controller.edit_machine(|m| m.name = settled.clone());
        }
        self.active_machine = settled;
        self.machines.save();
        self.remember();
    }

    fn restore_tool_table_preferences(&mut self) {
        let prefs = &self.settings.tool_table;
        self.tool_table_columns = prefs.visible;
        self.tool_table_widths = prefs.widths;
        self.tool_table_order = prefs.order;
        self.library_view = if prefs.grouped {
            LibraryView::Grouped
        } else {
            LibraryView::Ordered
        };
        self.tool_table_search = prefs.search.clone();
        self.tool_table_filters = prefs.filters.clone();
        self.tool_table_drag = None;
        self.tool_table_menu = None;
    }

    fn remember_tool_table(&mut self) {
        if self.operation_tool_selector.is_some() {
            return;
        }
        let prefs = &mut self.settings.tool_table;
        prefs.visible = self.tool_table_columns;
        prefs.widths = self.tool_table_widths;
        prefs.order = self.tool_table_order;
        prefs.grouped = self.library_view == LibraryView::Grouped;
        prefs.search = self.tool_table_search.clone();
        prefs.filters = self.tool_table_filters.clone();
        self.settings.save();
    }

    /// Copy the live view/snap/pane state into the preferences and write them out.
    ///
    /// Called from every handler that changes one of them. The mapping itself lives in
    /// `Settings::remember_session` so it can be tested headlessly; this is only the
    /// gathering.
    fn remember(&mut self) {
        self.settings.remember_session(
            crate::ViewPrefs {
                show_stock: self.show_stock,
                transparent_stock: self.settings.view.transparent_stock,
                stock_color: self.settings.view.stock_color,
                stock_transparency: self.settings.view.stock_transparency,
                show_gizmo: self.show_gizmo,
                show_origin: self.show_origin,
                show_envelope: self.show_envelope,
                tooltips: self.tooltips,
                gizmo_size: self.gizmo_size,
                // Not mirrored on `App` — the panel writes it straight into `settings`,
                // so carry the current value through rather than a default.
                origin_marker_scale: self.settings.view.origin_marker_scale,
                extra: Default::default(),
            },
            self.snaps.clone(),
            crate::SessionState {
                machine: Some(self.active_machine.clone()),
                extra: Default::default(),
            },
            crate::PanePrefs {
                show_project_browser: self.show_project_browser,
                show_library: self.workspace == Workspace::Tools,
                inspector_enabled: self.inspector_enabled,
                show_output: self.pane_handle(Pane::Output).is_some(),
                project_px: self.project_px,
                inspector_px: self.inspector_px,
                output_px: self.output_px,
                // The minimums are not session state — they are set in preferences,
                // and remembering a layout must not reset them.
                ..self.settings.panes.clone()
            },
        );
        self.settings.save();
    }

    /// Hold the non-Viewport panes at their fixed pixel sizes, letting the Viewport
    /// absorb the rest. Outermost split first so a parent's dimension is settled
    /// before its children read it: Output (a height) and Project (a width) are
    /// independent, while Inspector's region width depends on Project — so it is set
    /// last. Hidden panes are simply skipped (`set_pane_px` early-returns).
    fn apply_fixed_layout(&mut self) {
        self.set_pane_px(Pane::Output, self.output_px);
        self.set_pane_px(Pane::Project, self.project_px);
        // The Library pane shares the left slot's width with Project.
        self.set_pane_px(Pane::Library, self.project_px);
        self.set_pane_px(Pane::Inspector, self.inspector_px);
    }

    /// After a manual divider drag, store the affected pane's new pixel size so a
    /// later window resize preserves it. `split`/`ratio` are the just-applied resize.
    fn capture_side_px(&mut self, split: pane_grid::Split, ratio: f32) {
        let bounds = iced::Size::new(
            self.window.width,
            (self.window.height - MENU_BAR_H).max(1.0),
        );
        let regions = self
            .panes
            .layout()
            .split_regions(PANE_SPACING, PANE_MIN_PX, bounds);
        let Some(&(axis, rect, _)) = regions.get(&split) else {
            return;
        };
        let dim = match axis {
            pane_grid::Axis::Vertical => rect.width,
            pane_grid::Axis::Horizontal => rect.height,
        };
        if dim <= 1.0 {
            return;
        }
        for pane in [Pane::Project, Pane::Inspector, Pane::Output] {
            let Some(handle) = self.pane_handle(pane) else {
                continue;
            };
            let Some(path) = splits_to_pane(self.panes.layout(), handle) else {
                continue;
            };
            let Some(&(pane_split, in_a)) = path.last() else {
                continue;
            };
            if pane_split != split {
                continue;
            }
            let px = if in_a {
                ratio * dim
            } else {
                (1.0 - ratio) * dim
            };
            match pane {
                Pane::Project | Pane::Library => self.project_px = px,
                Pane::Inspector => self.inspector_px = px,
                Pane::Output => self.output_px = px,
                Pane::Viewport => {}
            }
        }
    }

    /// Reload the inspector edit buffers from the model for the current
    /// selection.
    fn refresh_fields(&mut self) {
        self.parameter_editor = Default::default();
        self.parameter_menu = None;
        self.edited_tool_input = None;
        if self.creating_operation() {
            self.focused_height = None;
            self.fields.clear();
            for field in self.inspector_fields() {
                if let Some(source) = self.saved_field_source(field) {
                    self.fields.insert(field, source);
                }
            }
            return;
        }
        if let Selection::Operation(id) = self.controller.selection() {
            if self.height_draft_id != Some(id) {
                self.seed_height_editor(id);
            }
        } else {
            self.height_draft_id = None;
            self.height_draft = None;
            self.height_offsets.clear();
            self.height_reference_pick = None;
        }
        self.focused_height = None;
        self.fields.clear();
        if let Selection::Operation(id) = self.controller.selection() {
            if let Some(binding) = self.controller.geometry_state().bindings.get(&id) {
                self.cad_override_top = binding.overrides.top.map(fmt_num).unwrap_or_default();
                self.cad_override_bottom =
                    binding.overrides.bottom.map(fmt_num).unwrap_or_default();
            }
        }
        for field in self.inspector_fields() {
            if let Some(source) = self.saved_field_source(field) {
                self.fields.insert(field, source);
            }
        }
    }

    /// Seed the wizard's editable cutting-data buffers (RPM / feed / plunge) from the
    /// pending operation's chosen tool nominals. A no-op unless a tool is set.
    /// Reset the Tools working copy to the committed library tool — the clean baseline
    /// the dirty check (and thus the Apply button) compares against — and reload the
    /// field buffers from it. Called at every "load" point (tool selection, New, Delete,
    /// tab switch, post-Apply), but *not* on an in-place field/picker edit.
    fn reload_tool_edit(&mut self) {
        self.selected_tool_parameter = None;
        self.show_preset_editor = false;
        self.preset_editor_snapshot = None;
        self.library_preset = None;
        self.preset_buffers.clear();
        self.tool_edit = if self.library_mode() {
            self.library.tools.get(self.lib_sel).cloned()
        } else {
            None
        };
        self.select_library_preset(
            self.tool_edit
                .as_ref()
                .and_then(|t| t.presets.first().map(|p| p.id)),
        );
        self.refresh_fields();
    }

    /// Whether either origin-pick mode (single or two-point) is active.
    fn in_origin_pick(&self) -> bool {
        self.setting_origin || self.setting_origin_2pt
    }

    /// Reset the viewport highlight to the controller's current single selection
    /// (an operation, or nothing). Used after structural edits (duplicate/delete)
    /// that move the selection, so the highlight set never keeps stale ids.
    fn focus_selected_op(&mut self) {
        self.focus_ops.clear();
        if let Selection::Operation(id) = self.controller.selection() {
            self.focus_ops.insert(id);
            if self
                .operation_draft
                .as_ref()
                .is_none_or(|draft| draft.id() != id)
            {
                self.begin_operation_draft(id);
            }
        } else {
            self.operation_draft = None;
            self.operation_tools.clear();
        }
    }

    /// Whether the Inspector is in tool-library editing mode (driven by the Tools
    /// ribbon tab, independent of the project selection).
    fn library_mode(&self) -> bool {
        self.workspace == Workspace::Tools
    }

    /// The library indices in the order the bulk renumber will assign 1..N — the same
    /// order the current Library-pane tab displays (Ordered by number, Grouped by
    /// family then diameter).
    fn renumber_order(&self) -> Vec<usize> {
        let mut idx: Vec<usize> = (0..self.library.tools.len()).collect();
        match self.library_view {
            LibraryView::Ordered => idx.sort_by_key(|&i| self.library.tools[i].number),
            LibraryView::Grouped => idx.sort_by(|&a, &b| {
                let (ta, tb) = (&self.library.tools[a], &self.library.tools[b]);
                kind_order(ta.kind)
                    .cmp(&kind_order(tb.kind))
                    .then(ta.diameter.total_cmp(&tb.diameter))
                    .then(ta.number.cmp(&tb.number))
            }),
        }
        idx
    }

    /// The plunge style an operation enters with, and (separately) the one its clearing
    /// pass uses — the two are independent settings on a carve.
    fn op_plunge_styles(op: &Operation) -> (Option<Plunge>, Option<Plunge>) {
        match op {
            Operation::Profile(p) => (Some(p.plunge), None),
            Operation::Pocket(p) => (Some(p.clear.plunge), None),
            Operation::Carve(c) => (Some(c.plunge), c.clear.as_ref().map(|cl| cl.params.plunge)),
            _ => (None, None),
        }
    }

    /// The label for a plunge parameter under the current selection's style — see
    /// [`plunge_param_label`], which holds the wording.
    fn plunge_label(&self, field: Field) -> Option<&'static str> {
        let (plunge, first, clearing) = self.plunge_param(field)?;
        plunge_param_label(plunge, first, clearing)
    }

    /// Which plunge parameter `field` is, for the current selection: the style it belongs
    /// to, whether it is the **first** of the pair, and whether it is the **clearing**
    /// pass's rather than the operation's own. `None` when it is not a plunge parameter,
    /// or when nothing that has one is selected.
    ///
    /// The one place the `Field` → (style, slot) mapping lives, so the label and the help
    /// cannot disagree about which parameter they are describing.
    fn plunge_param(&self, field: Field) -> Option<(Plunge, bool, bool)> {
        let op = if self.creating_operation() {
            self.operation_draft.as_ref()?
        } else {
            let Selection::Operation(id) = self.controller.selection() else {
                return None;
            };
            self.current_operation(id)?
        };
        let (own, clear) = Self::op_plunge_styles(op);
        let (plunge, clearing) = match field {
            Field::PlungeA | Field::PlungeB => (own?, false),
            Field::ClearPlungeA | Field::ClearPlungeB => (clear?, true),
            _ => return None,
        };
        Some((
            plunge,
            matches!(field, Field::PlungeA | Field::ClearPlungeA),
            clearing,
        ))
    }

    /// The tooltip for a plunge parameter under the current selection's style — see
    /// [`plunge_param_help`]. `None` falls through to the generic wording, which is only
    /// reachable with nothing selected.
    ///
    /// The labels became kind-aware on 2026-08-01 and these did not, so a box labelled
    /// "Ramp angle (°)" still explained itself as "the ramp/zig-zag angle in degrees, or
    /// the helix radius in mm" — the very positional wording the labels exist to replace.
    fn plunge_help(&self, field: Field) -> Option<&'static str> {
        let (plunge, first, _) = self.plunge_param(field)?;
        plunge_param_help(plunge, first)
    }

    /// The kind of the tool the inspector is editing (library entry or project tool).
    fn inspected_tool_kind(&self) -> Option<ToolKind> {
        if self.library_mode() {
            // Prefer the working copy so a Type change (which edits it, not the committed
            // entry) immediately drives the kind-specific field set, labels and validation.
            return self
                .tool_edit
                .as_ref()
                .map(|t| t.kind)
                .or_else(|| self.library.tools.get(self.lib_sel).map(|t| t.kind));
        }
        if let Selection::Tool(i) = self.controller.selection() {
            self.controller
                .inspection_setup()
                .tools
                .get(i)
                .map(|t| t.kind)
        } else {
            None
        }
    }

    /// The inspector label for a field, kind-aware. A V-bit's transverse measurement is
    /// its shaft, not a (variable) flute diameter, so `ToolDiameter` reads "Shank
    /// diameter" there.
    fn field_label(&self, field: Field) -> &'static str {
        if field == Field::Engagement && matches!(self.operation_draft, Some(Operation::Pocket(_)))
        {
            return "Adaptive stepover (mm)";
        }
        if field == Field::Stepdown
            && matches!(
                self.operation_draft,
                Some(Operation::Pocket(_) | Operation::Face(_) | Operation::Profile(_))
            )
        {
            return "Maximum Roughing Stepdown (mm)";
        }
        match (self.inspected_tool_kind(), field) {
            (Some(ToolKind::RadiusMill { .. }), Field::ToolDiameter) => "Tip diameter (mm)",
            // V-bit and chamfer mill: the single transverse ⌀ is the shaft they flare to.
            (Some(ToolKind::VBit { .. } | ToolKind::ChamferMill { .. }), Field::ToolDiameter) => {
                "Shank diameter (mm)"
            }
            // Face mill: cutting-⌀ disc, and the flute/shank fields describe the shell-mill
            // body and its arbor.
            (Some(ToolKind::FaceMill), Field::ToolDiameter) => "Cutting diameter (mm)",
            (Some(ToolKind::FaceMill), Field::FluteLength) => "Body height (mm)",
            (Some(ToolKind::FaceMill), Field::ShankDiameter) => "Arbor diameter (mm)",
            // Thread mill: a single-point mill's ⌀ is the *minimum* cutting ⌀ (smallest
            // hole); FluteLength is its length of cut (reach) and it exposes the reduced
            // neck. A full-form mill uses the plain cutting ⌀ and thread length.
            (Some(ToolKind::ThreadMill { pitch: None }), Field::ToolDiameter) => {
                "Min cutting diameter (mm)"
            }
            (Some(ToolKind::ThreadMill { pitch: Some(_) }), Field::ToolDiameter) => {
                "Cutting diameter (mm)"
            }
            (Some(ToolKind::ThreadMill { pitch: None }), Field::FluteLength) => {
                "Length of cut (mm)"
            }
            (Some(ToolKind::ThreadMill { pitch: Some(_) }), Field::FluteLength) => {
                "Thread length (mm)"
            }
            (Some(ToolKind::ThreadMill { .. }), Field::NeckDiameter) => "Neck diameter (mm)",
            _ => self.plunge_label(field).unwrap_or_else(|| field.label()),
        }
    }

    /// Kind-aware tooltip for a Tools-inspector field — the same field means different
    /// things across tool kinds (a V-bit's `ToolDiameter` is its shaft, a face mill's is
    /// the cutting disc, …), so the help matches the (kind-aware) label. Non-tool fields,
    /// and kinds where the generic wording already fits, fall back to `field.help()`.
    fn field_help(&self, field: Field) -> &'static str {
        if field == Field::Engagement && matches!(self.operation_draft, Some(Operation::Pocket(_)))
        {
            return "Target radial width of cut along a straight wall. Tight curves can exceed this value; adaptive clearing reduces full-width slotting.";
        }
        if field == Field::ProfileOffset && self.geometry_operation_kind() == Some(OpKind::Profile)
        {
            return profile_parameters::WALL_ALLOWANCE_HELP;
        }
        use ToolKind::*;
        match (self.inspected_tool_kind(), field) {
            (Some(RadiusMill { .. }), Field::ToolDiameter) => "Diameter of the cylindrical tip. The maximum cutting diameter is this value plus twice the corner radius.",
            (Some(RadiusMill { .. }), Field::CornerRadius) => "Radius of the concave cutting edge, measured from the tip to the outer cutting diameter.",
            // Cutting/flute diameter — the meaning turns on the kind.
            (Some(EndMill | RoughingMill | BallMill | BullNose { .. }), Field::ToolDiameter) => {
                "Diameter across the flutes — the end mill's cutting diameter."
            }
            (Some(Drill { .. }), Field::ToolDiameter) => {
                "The drill's cutting diameter — the ⌀ of the hole it produces."
            }
            (Some(VBit { .. } | ChamferMill { .. }), Field::ToolDiameter) => {
                "Shaft diameter — the cone flares up to this, and the shaft continues \
                 above it. (These tools have no single flute ⌀: the cutting ⌀ varies \
                 continuously along the cone.)"
            }
            (Some(FaceMill), Field::ToolDiameter) => {
                "Cutting diameter — the ⌀ of the disc the inserts sweep; it drives the \
                 facing stepover."
            }
            (Some(ThreadMill { pitch: None }), Field::ToolDiameter) => {
                "Minimum cutting diameter — the tooth-crest ⌀, i.e. the smallest hole the \
                 single-point mill can enter and thread."
            }
            (Some(ThreadMill { pitch: Some(_) }), Field::ToolDiameter) => {
                "Cutting diameter of the thread mill — it must clear the thread's minor \
                 diameter to enter the hole."
            }
            (Some(ThreadMill { .. }), Field::NeckDiameter) => {
                "Neck diameter — the reduced undercut behind the tooth. It sets the maximum \
                 thread depth (and so the coarsest pitch): (min cutting ⌀ − neck ⌀) / 2. A \
                 smaller neck clears deeper forms."
            }
            // Cutting-length family.
            (Some(Drill { .. }), Field::FluteLength) => {
                "Flute length — the fluted (cutting) portion from the tip; the plain shank \
                 continues above it. Must be at least the drill-point cone height."
            }
            (Some(FaceMill), Field::FluteLength) => {
                "Body height — the axial length of the wide cutting body; the narrower \
                 arbor continues above it up to the overall length."
            }
            (Some(ThreadMill { pitch: None }), Field::FluteLength) => {
                "Length of cut — how far down from the tip the single tooth can thread \
                 (the reduced-neck reach); it sets the maximum threaded depth."
            }
            (Some(ThreadMill { pitch: Some(_) }), Field::FluteLength) => {
                "Thread length — the length of the threaded cutting band, which bounds the \
                 thread depth reachable in one helical pass."
            }
            // Shank / arbor.
            (Some(FaceMill), Field::ShankDiameter) => {
                "Arbor diameter — the mounting stub above the cutting body (narrower than \
                 the cutting ⌀)."
            }
            // Point angle serves both the drill and the V-bit.
            (Some(Drill { .. }), Field::PointAngle) => {
                "Included angle of the drill point (e.g. 118° or 135°), placed so the full \
                 diameter reaches the intended depth. Bounded to 90°–135°."
            }
            (Some(VBit { .. }), Field::PointAngle) => {
                "Included angle of the V-bit's cone (e.g. 60° or 90°) — the point angle \
                 that sets how cut depth maps to cut width."
            }
            // Same fallback shape as `field_label`: a plunge parameter's help follows the
            // selected entry style, exactly as its label does.
            _ => self.plunge_help(field).unwrap_or_else(|| field.help()),
        }
    }

    /// Whether a field's current buffer value is invalid (flagged red). Rules:
    /// - **Corner radius** ≤ flute radius (⌀/2) — rounded-edge end mill.
    /// - **Flute length** > 0 everywhere; ≥ corner radius (rounded-edge); ≥ flute radius
    ///   ⌀/2 (ball nose) — the cutting-end feature must fit inside the flute.
    fn field_invalid(&self, field: Field) -> bool {
        let Ok(value) = self.parsed_field_value(field) else {
            return true;
        };
        if self.creating_operation() {
            return self
                .operation_draft
                .as_ref()
                .is_some_and(|op| operation_tabs::operation_field_invalid(op, field, value));
        }
        if !self.library_mode() {
            if let Selection::Operation(id) = self.controller.selection() {
                if let Some(operation) = self.current_operation(id) {
                    if matches!(field, Field::Clearance | Field::Retract | Field::TopOfStock)
                        && self.preview_safety_heights(id).validate().is_err()
                    {
                        return true;
                    }
                    if operation_tabs::operation_field_invalid(operation, field, value) {
                        return true;
                    }
                }
            }
        }
        let buf = |f: Field| self.fields.get(&f).and_then(|s| s.parse::<f64>().ok());
        match field {
            Field::MachineTravelX | Field::MachineTravelY | Field::MachineTravelZ => value <= 0.0,
            Field::MachineMaxZ => buf(Field::MachineTravelZ)
                .is_some_and(|travel| !(value - travel).is_finite() || value - travel >= value),
            Field::MachineMinSpindleRpm => {
                value < 0.0 || buf(Field::MachineMaxSpindleRpm).is_some_and(|max| value > max)
            }
            Field::MachineRapidRate | Field::MachineMaxSpindleRpm | Field::MachineMaxFeed => {
                value <= 0.0
                    || (field == Field::MachineMaxSpindleRpm
                        && buf(Field::MachineMinSpindleRpm).is_some_and(|min| min > value))
            }
            Field::MachineToolChangeSeconds => value < 0.0,
            Field::TipLength => {
                value < 0.0
                    || matches!((buf(Field::FluteLength), buf(Field::CornerRadius)), (Some(fl), Some(cr)) if value + cr > fl + 1e-9)
            }
            Field::CornerRadius => {
                if matches!(
                    self.inspected_tool_kind(),
                    Some(ToolKind::RadiusMill { .. })
                ) {
                    return value <= 0.0
                        || matches!((buf(Field::FluteLength), buf(Field::TipLength)), (Some(fl), Some(tip)) if value + tip > fl + 1e-9);
                }
                matches!((buf(Field::CornerRadius), buf(Field::ToolDiameter)),
                    (Some(cr), Some(d)) if cr > d * 0.5 + 1e-9)
            }
            Field::PointAngle => {
                let Some(a) = buf(Field::PointAngle) else {
                    return true;
                };
                match self.inspected_tool_kind() {
                    // Drill point angle is bounded to [90°, 135°].
                    Some(ToolKind::Drill { .. }) => !(90.0..=135.0).contains(&a),
                    // A V-bit's included angle just has to be a valid cone.
                    Some(ToolKind::VBit { .. }) => !(0.0 < a && a < 180.0),
                    _ => false,
                }
            }
            Field::TipRadius => {
                // Physically a V-bit's point is always *rounded* — a true r=0 edge
                // cannot be ground and would not survive contact. The rounded tip is
                // also what makes a V-bit cut at its centre (and so engrave), which is
                // exactly what distinguishes it from a chamfer mill's flat. So the
                // radius must be positive, and cannot exceed the shaft radius (⌀/2).
                match (buf(Field::TipRadius), buf(Field::ToolDiameter)) {
                    (Some(tr), _) if tr < MIN_TIP_RADIUS_MM - 1e-9 => true,
                    (Some(tr), Some(d)) => tr > d * 0.5 + 1e-9,
                    _ => false,
                }
            }
            Field::ChamferAngle => {
                // A chamfer mill's included angle just has to be a valid cone.
                let Some(a) = buf(Field::ChamferAngle) else {
                    return true;
                };
                !(0.0 < a && a < 180.0)
            }
            Field::TipDiameter => {
                // Physically a chamfer mill always has a *flat* at its point — it is
                // ground that way, and that flat is precisely why it does not cut at
                // its centre (and so cannot engrave). A zero flat would make it a
                // V-bit, not a chamfer mill, so it must be positive; and it must stay
                // narrower than the shaft ⌀, else there is no cone at all.
                match (buf(Field::TipDiameter), buf(Field::ToolDiameter)) {
                    // Held to the same physical radius as a V-bit's tip, hence 2×.
                    (Some(tip), _) if tip < 2.0 * MIN_TIP_RADIUS_MM - 1e-9 => true,
                    (Some(tip), Some(d)) => tip >= d - 1e-9,
                    _ => false,
                }
            }
            Field::ShankDiameter => {
                // A face mill's arbor cannot be wider than its cutting body (⌀), else the
                // silhouette flares out above the body. Only enforced for face mills.
                matches!(self.inspected_tool_kind(), Some(ToolKind::FaceMill))
                    && matches!((buf(Field::ShankDiameter), buf(Field::ToolDiameter)),
                        (Some(arbor), Some(d)) if arbor > d + 1e-9)
            }
            Field::ToolThreadPitch => {
                // Shown only for a full-form thread mill, where the pitch must be positive.
                !matches!(buf(Field::ToolThreadPitch), Some(p) if p > 0.0)
            }
            Field::NeckDiameter => {
                // A single-point thread mill's neck must be strictly reduced (smaller than
                // the min cutting ⌀), else there is no undercut / thread-depth clearance.
                matches!(
                    self.inspected_tool_kind(),
                    Some(ToolKind::ThreadMill { .. })
                ) && matches!((buf(Field::NeckDiameter), buf(Field::ToolDiameter)),
                        (Some(neck), Some(d)) if neck >= d - 1e-9)
            }
            Field::FluteLength => {
                let Some(fl) = buf(Field::FluteLength) else {
                    return true; // empty / unparseable
                };
                if fl <= 1e-9 {
                    return true; // flute length cannot be 0 (anywhere)
                }
                match self.inspected_tool_kind() {
                    Some(ToolKind::RadiusMill { .. }) => {
                        matches!((buf(Field::CornerRadius), buf(Field::TipLength)), (Some(cr), Some(tip)) if fl < cr + tip - 1e-9)
                    }
                    Some(ToolKind::BullNose { .. }) => {
                        matches!(buf(Field::CornerRadius), Some(cr) if fl < cr - 1e-9)
                    }
                    Some(ToolKind::BallMill) => {
                        matches!(buf(Field::ToolDiameter), Some(d) if fl < d * 0.5 - 1e-9)
                    }
                    Some(ToolKind::Drill { .. }) => {
                        // Must be at least the point cone height = (⌀/2) / tan(angle/2).
                        matches!((buf(Field::ToolDiameter), buf(Field::PointAngle)),
                            (Some(d), Some(pa)) if pa > 0.0
                                && fl < (d * 0.5) / (pa * 0.5).to_radians().tan() - 1e-9)
                    }
                    _ => false,
                }
            }
            _ => false,
        }
    }

    /// Whether any inspector field is currently invalid (Apply is disabled if so).
    fn any_field_invalid(&self) -> bool {
        if let Some(op) = self
            .operation_draft
            .as_ref()
            .filter(|op| op.rotary_wrap().is_some())
        {
            let mut op = op.clone();
            let values = self
                .fields
                .iter()
                .filter_map(|(f, v)| v.parse::<f64>().ok().map(|v| (*f, v)))
                .collect();
            apply_op_fields(&mut op, &values);
            if op
                .rotary_wrap()
                .is_some_and(|wrap| wrap.validate().is_err())
            {
                return true;
            }
        }
        if self.profile_parameter_error().is_some() {
            return true;
        }
        if self.library_mode() && self.preset_edit_invalid() {
            return true;
        }
        if let Selection::Operation(id) = self.controller.selection() {
            if !self.library_mode()
                && !self.creating_operation()
                && (self.height_rules_dirty()
                    || self
                        .controller
                        .inspection_setup()
                        .height_settings
                        .contains_key(&id))
                && self.resolve_height_editor(id).is_err()
            {
                return true;
            }
        }
        if !self.library_mode() && !self.creating_operation() {
            if let Selection::Operation(id) = self.controller.selection() {
                if matches!(self.current_operation(id), Some(Operation::Thread(_))) {
                    let value = |field| self.fields.get(&field).and_then(|s| s.parse::<f64>().ok());
                    if !matches!((value(Field::ThreadTop), value(Field::ThreadBottom)), (Some(top), Some(bottom)) if top > bottom)
                    {
                        return true;
                    }
                }
            }
        }
        self.inspector_fields()
            .iter()
            .any(|&f| self.field_invalid(f))
    }

    /// Apply the flute/shank/overall length constraint to the selected library tool
    /// **live** from the `edited` field's current buffer, then refresh the *other* two
    /// length buffers so the derived value shows immediately. Only fires on a valid
    /// number; the edited field's own buffer is left untouched (mid-typing).
    fn live_edit(&mut self, edited: Field) {
        let Some(v) = self.fields.get(&edited).and_then(|s| s.parse::<f64>().ok()) else {
            return; // empty / unparseable: hold the last valid working copy
        };
        if v < 0.0 || (edited == Field::FluteLength && v <= 1e-9) {
            return; // never write a zero/negative flute (keeps the last valid state)
        }
        let mut parsed: BTreeMap<Field, f64> = BTreeMap::new();
        parsed.insert(edited, v);
        // Apply just the edited field to the working copy (single-field semantics keep
        // the overall = flute + shank coupling correct), then read back the derived
        // length trio to refresh the *other* two buffers for display.
        let derived = {
            let Some(t) = self.tool_edit.as_mut() else {
                return;
            };
            apply_tool_dims(t, &parsed);
            apply_tool_kind_fields(&mut t.kind, &parsed);
            [
                (Field::FluteLength, t.flute_len()),
                (Field::ShankLength, (t.length - t.flute_len()).max(0.0)),
                (Field::ToolLength, t.length),
            ]
        };
        for (f, val) in derived {
            if f != edited {
                self.fields.insert(f, fmt_num(val));
            }
        }
        if matches!(edited, Field::ToolDiameter | Field::Flutes) {
            self.refresh_preset_cutting_fields(None);
        }
    }

    /// The tool whose cross-section the viewport should preview (Phase 5): the selected
    /// library tool while the Tools tab is active, else a selected project tool. `None`
    /// ⇒ show the normal 3D backplot.
    fn preview_tool(&self) -> Option<cam_model::Tool> {
        if self.library_mode() {
            // The live working copy (which carries the in-progress edits); fall back to
            // the committed tool before the first edit / working copy is set.
            return self
                .tool_edit
                .clone()
                .or(self.library.tools.get(self.lib_sel).cloned());
        }
        match self.controller.selection() {
            Selection::Tool(i) => self.controller.inspection_setup().tools.get(i).cloned(),
            _ => None,
        }
    }

    /// Whether the Tools inspector has uncommitted edits — the working copy differs
    /// from the committed library entry. Outside Tools mode there is no working copy,
    /// so the Apply button keeps its usual (validity-only) gating.
    fn inspector_dirty(&self) -> bool {
        if self.machine_edit_dirty() {
            return true;
        }
        if !self.library_mode() {
            if self.stock_to_leave_dirty() {
                return true;
            }
            if self.operation_draft_dirty() {
                return true;
            }
            if let Selection::Operation(id) = self.controller.selection() {
                if let Some(binding) = self.controller.geometry_state().bindings.get(&id) {
                    let parsed = |value: &str| {
                        if value.trim().is_empty() {
                            None
                        } else {
                            value.trim().parse::<f64>().ok()
                        }
                    };
                    if !self.operation_height_overrides_valid()
                        || parsed(&self.cad_override_top) != binding.overrides.top
                        || parsed(&self.cad_override_bottom) != binding.overrides.bottom
                    {
                        return true;
                    }
                }
            }
            // Same promise the Tools editor makes, everywhere else: Apply is live only
            // when something is actually pending. The buffers are seeded from the model
            // by `refresh_fields`, so an untouched inspector compares equal all the way
            // down and the button stays grey — which is what tells the operator, at a
            // glance, whether an edit has been committed or is still sitting in the box.
            return self.parameter_fields_dirty()
                || fields_are_dirty(
                    &self
                        .inspector_fields()
                        .into_iter()
                        .filter(|f| parameter_expressions::parameter(*f).is_none())
                        .collect::<Vec<_>>(),
                    &self.fields,
                    |f| self.field_value(f),
                );
        }
        match (self.tool_edit.clone(), self.library.tools.get(self.lib_sel)) {
            (Some(edit), Some(saved)) => {
                edit != *saved
                    || fields_are_dirty(&self.inspector_fields(), &self.fields, |field| {
                        self.field_value(field)
                    })
            }
            _ => false,
        }
    }

    fn stock_to_leave_dirty(&self) -> bool {
        let draft = self.stock_to_leave_value();
        if self.creating_operation() {
            return draft != cam_model::StockToLeave::default();
        }
        match self.controller.selection() {
            Selection::Operation(id) => {
                draft != self.controller.inspection_setup().stock_to_leave_for(id)
            }
            _ => false,
        }
    }

    fn stock_to_leave_with_fields(&self) -> cam_model::StockToLeave {
        let mut stock = self.stock_to_leave_value();
        if let Some(value) = self
            .fields
            .get(&Field::StockLeaveRadial)
            .and_then(|s| s.trim().parse().ok())
        {
            stock.radial = value;
        }
        if let Some(value) = self
            .fields
            .get(&Field::StockLeaveAxial)
            .and_then(|s| s.trim().parse().ok())
        {
            stock.axial = value;
        }
        stock
    }

    /// Which fields the inspector shows for the current selection.
    fn inspector_fields(&self) -> Vec<Field> {
        // The field set is **kind-specific** — the end mill has its own characterisation
        // (Andreas): flute ⌀, flute length, shank ⌀, shank length, overall length,
        // flutes — no neck. Other kinds keep the generic set until characterised in turn.
        // The Cutting-direction control is rendered separately (it's a picker, not a
        // numeric field), as is the Type picker.
        let tool_fields = |kind: Option<ToolKind>| {
            let mut fields = match kind {
                // Square & Ball Nose end mills share the exact end-mill field set.
                Some(ToolKind::EndMill | ToolKind::RoughingMill | ToolKind::BallMill) => vec![
                    Field::ToolDiameter, // "Flute ⌀"
                    Field::FluteLength,
                    Field::ShankDiameter,
                    Field::ShankLength,
                    Field::ToolLength, // "Overall length"
                    Field::Flutes,
                ],
                // Rounded-Edge (bull-nose) end mill: the same, plus a corner radius.
                Some(ToolKind::BullNose { .. }) => vec![
                    Field::ToolDiameter,
                    Field::CornerRadius,
                    Field::FluteLength,
                    Field::ShankDiameter,
                    Field::ShankLength,
                    Field::ToolLength,
                    Field::Flutes,
                ],
                Some(ToolKind::RadiusMill { .. }) => vec![
                    Field::ToolDiameter,
                    Field::CornerRadius,
                    Field::TipLength,
                    Field::FluteLength,
                    Field::ShankDiameter,
                    Field::ShankLength,
                    Field::ToolLength,
                    Field::Flutes,
                ],
                // Drill bit: flute/shank/overall + point angle. No flutes count, no direction.
                Some(ToolKind::Drill { .. }) => vec![
                    Field::ToolDiameter,
                    Field::FluteLength,
                    Field::ShankDiameter,
                    Field::ShankLength,
                    Field::ToolLength,
                    Field::PointAngle,
                ],
                // V-bit: a single shaft ⌀ (rendered via ToolDiameter, relabelled "Shank
                // diameter" — see `field_label`), overall length, point angle, tip radius.
                // A V-bit has *no* flute diameter: the cutting ⌀ varies along the cone, so
                // the only fixed transverse measurement is the shaft it flares up to.
                Some(ToolKind::VBit { .. }) => vec![
                    Field::ToolDiameter,
                    Field::ToolLength,
                    Field::PointAngle,
                    Field::TipRadius,
                ],
                // Chamfer mill: like a V-bit but its tip is a flat (non-cutting) instead of a
                // rounded one — a single shaft ⌀, overall length, point angle, and a flat tip
                // ⌀ in place of the V-bit's tip radius.
                Some(ToolKind::ChamferMill { .. }) => vec![
                    Field::ToolDiameter,
                    Field::ToolLength,
                    Field::ChamferAngle,
                    Field::TipDiameter,
                ],
                // Face mill (shell mill): a wide cutting body on a narrower arbor. FluteLength
                // is relabelled "Body height" and ShankDiameter "Arbor diameter" (see
                // `field_label`). No shank-length / neck — the arbor length is overall − body.
                Some(ToolKind::FaceMill) => vec![
                    Field::ToolDiameter,
                    Field::FluteLength,
                    Field::ShankDiameter,
                    Field::ToolLength,
                    Field::Flutes,
                ],
                // Thread mill (single-point/full-form is a toggle, rendered separately):
                // - single-point (single profile): one 60° tooth + a reduced neck. Min cutting
                //   ⌀ (smallest hole), neck ⌀ (sets max thread depth), length of cut (reach),
                //   shank ⌀/length, overall. The blind-hole allowance is derived per operation.
                // - full-form: a long threaded band at a fixed pitch (cutting ⌀, thread length,
                //   shank ⌀, shank length, overall, pitch). No neck.
                Some(ToolKind::ThreadMill { pitch: None }) => vec![
                    Field::ToolDiameter,
                    Field::NeckDiameter,
                    Field::FluteLength,
                    Field::ShankDiameter,
                    Field::ShankLength,
                    Field::ToolLength,
                    Field::Flutes,
                ],
                Some(ToolKind::ThreadMill { pitch: Some(_) }) => vec![
                    Field::ToolDiameter,
                    Field::FluteLength,
                    Field::ShankDiameter,
                    Field::ShankLength,
                    Field::ToolLength,
                    Field::ToolThreadPitch,
                    Field::Flutes,
                ],
                other => {
                    let mut f = vec![
                        Field::ToolDiameter,
                        Field::ToolLength,
                        Field::FluteLength,
                        Field::ShankDiameter,
                        Field::NeckLength,
                        Field::NeckDiameter,
                        Field::Flutes,
                    ];
                    if let Some(k) = other {
                        f.extend(tool_kind_fields(k));
                    }
                    f
                }
            };
            // Nominal cutting data is common to every tool kind — the library defaults
            // that seed a new operation's RPM/feed/plunge.
            fields.extend([
                Field::NominalRpm,
                Field::NominalFeed,
                Field::NominalPlungeFeed,
            ]);
            fields
        };
        if self.library_mode() {
            // Use the working-copy kind (via inspected_tool_kind) so a Type change swaps
            // in the new field set immediately, without waiting for Apply.
            return tool_fields(self.inspected_tool_kind());
        }
        if self.creating_operation() {
            return self
                .operation_draft
                .as_ref()
                .map(|operation| self.geometry_operation_fields(operation))
                .unwrap_or_else(|| vec![Field::SpindleRpm, Field::Feed, Field::PlungeFeed]);
        }
        match self.controller.selection() {
            // Safety planes are edited on the operation; tool-change height is
            // still shared by the setup.
            Selection::Setup => vec![Field::ToolChangeHeight],
            Selection::Machine => vec![
                Field::MachineTravelX,
                Field::MachineTravelY,
                Field::MachineTravelZ,
                Field::MachineMaxZ,
                Field::MachineRapidRate,
                Field::MachineMinSpindleRpm,
                Field::MachineMaxSpindleRpm,
                Field::MachineMaxFeed,
                Field::MachineToolChangeSeconds,
            ],
            Selection::Origin => vec![
                Field::OriginIndex,
                Field::OriginX,
                Field::OriginY,
                Field::OriginZ,
            ],
            Selection::Tool(i) => tool_fields(
                self.controller
                    .inspection_setup()
                    .tools
                    .get(i)
                    .map(|t| t.kind),
            ),
            Selection::Stock => vec![
                Field::StockXOffset,
                Field::StockYOffset,
                Field::StockTop,
                Field::StockThickness,
            ],
            Selection::Operation(id) => self
                .current_operation(id)
                .map(|op| {
                    let mut fields = self.geometry_operation_fields(op);
                    fields.extend([Field::Clearance, Field::Retract, Field::TopOfStock]);
                    fields
                })
                .unwrap_or_default(),
        }
    }
    /// The model value backing a field for the current selection, if any.
    fn field_value(&self, field: Field) -> Option<f64> {
        if !self.library_mode() {
            if let Some(parameter) = parameter_expressions::parameter(field) {
                let op = self.operation_draft.as_ref().or_else(|| {
                    if let Selection::Operation(id) = self.controller.selection() {
                        self.controller.operation(id)
                    } else {
                        None
                    }
                });
                if let Some(op) = op {
                    if matches!(field, Field::CuttingSpeed | Field::FeedPerTooth)
                        && ![
                            cam_model::OperationParameter::CuttingSpeed,
                            cam_model::OperationParameter::FeedPerTooth,
                            cam_model::OperationParameter::SpindleRpm,
                            cam_model::OperationParameter::Feed,
                        ]
                        .iter()
                        .any(|p| p.expression(op).is_some())
                    {
                        return self.operation_cutting_value(op, field);
                    }
                    let tool = self
                        .operation_tools
                        .iter()
                        .chain(self.controller.inspection_setup().tools.iter())
                        .find(|t| t.number == op.tool());
                    if !op.has_parameter_expressions() {
                        if let Some(value) = op_field(op, field) {
                            return Some(value);
                        }
                    }
                    if let Ok(values) = op.evaluate_parameter_sources(tool, &BTreeMap::new()) {
                        if let Some(value) = values.get(&parameter) {
                            return Some(*value);
                        }
                    }
                }
            }
        }
        if matches!(field, Field::CuttingSpeed | Field::FeedPerTooth) {
            let operation = self.operation_draft.as_ref().or_else(|| {
                if let Selection::Operation(id) = self.controller.selection() {
                    self.controller.operation(id)
                } else {
                    None
                }
            })?;
            return self.operation_cutting_value(operation, field);
        }
        if matches!(field, Field::StockLeaveRadial | Field::StockLeaveAxial) {
            let stock = self.stock_to_leave_value();
            return Some(match field {
                Field::StockLeaveRadial => stock.radial,
                _ => stock.axial,
            });
        }
        if self.creating_operation() {
            return self
                .operation_draft
                .as_ref()
                .and_then(|op| op_field(op, field));
        }
        if self.library_mode() {
            // Prefer the live working copy so kind/picker edits repopulate from it.
            let t = self
                .tool_edit
                .as_ref()
                .or_else(|| self.library.tools.get(self.lib_sel))?;
            return match field {
                Field::ToolDiameter => Some(t.diameter),
                Field::ToolLength => Some(t.length),
                // Effective flute (resolves the 0-sentinel), so shank = overall − flute.
                Field::FluteLength => Some(t.flute_len()),
                Field::ShankDiameter => Some(t.shank_dia()),
                Field::ShankLength => Some((t.length - t.flute_len()).max(0.0)),
                Field::NeckLength => Some(t.neck_length),
                Field::NeckDiameter => Some(t.neck_diameter),
                Field::Flutes => Some(t.flutes as f64),
                Field::NominalRpm => Some(t.nominal_rpm),
                Field::NominalFeed => Some(t.nominal_feed),
                Field::NominalPlungeFeed => Some(t.nominal_plunge_feed),
                _ => tool_kind_field(t.kind, field),
            };
        }
        if let Field::MachineTravelX | Field::MachineTravelY | Field::MachineTravelZ = field {
            let (x, y, z) = self.controller.machine().envelope.extent();
            return Some(match field {
                Field::MachineTravelX => x,
                Field::MachineTravelY => y,
                _ => z,
            });
        }
        if field == Field::MachineMaxZ {
            return Some(self.controller.machine().envelope.max.z);
        }
        if field == Field::MachineRapidRate {
            return Some(self.controller.machine().rapid_rate);
        }
        if field == Field::MachineMinSpindleRpm {
            return Some(self.controller.machine().min_spindle_rpm);
        }
        if field == Field::MachineMaxSpindleRpm {
            return Some(self.controller.machine().max_spindle_rpm);
        }
        if field == Field::MachineMaxFeed {
            return Some(self.controller.machine().max_feed);
        }
        if field == Field::MachineToolChangeSeconds {
            return Some(self.controller.machine().tool_change_seconds);
        }
        let setup = self.controller.inspection_setup();
        let heights = match self.controller.selection() {
            Selection::Operation(id) => setup.heights_for(id),
            _ => setup.heights,
        };
        match field {
            Field::Clearance => Some(heights.clearance),
            Field::Retract => Some(heights.retract),
            Field::TopOfStock => Some(heights.top_of_stock),
            // Shows the *effective* height: the explicit value if set, else the
            // machine-Z-travel default the planner would resolve.
            Field::ToolChangeHeight => Some(
                setup.resolved_tool_change_height(self.controller.machine().envelope.max.z)
                    - setup.origin[2],
            ),
            Field::OriginX => Some(
                self.controller
                    .origin_position(self.controller.active_origin())[0],
            ),
            Field::OriginY => Some(
                self.controller
                    .origin_position(self.controller.active_origin())[1],
            ),
            Field::OriginZ => Some(
                self.controller
                    .origin_position(self.controller.active_origin())[2],
            ),
            Field::OriginIndex => Some(self.controller.active_origin() as f64),
            Field::StockXOffset | Field::StockYOffset | Field::StockTop | Field::StockThickness => {
                match setup.stock {
                    cam_model::Stock::BoundingBox {
                        x_offset,
                        y_offset,
                        top,
                        thickness,
                    } => Some(match field {
                        Field::StockXOffset => x_offset,
                        Field::StockYOffset => y_offset,
                        Field::StockTop => top,
                        _ => thickness,
                    }),
                    _ => None,
                }
            }
            Field::ToolDiameter => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.diameter),
                _ => None,
            },
            Field::ToolLength => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.length),
                _ => None,
            },
            Field::FluteLength => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.flute_len()),
                _ => None,
            },
            Field::ShankDiameter => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.shank_dia()),
                _ => None,
            },
            Field::ShankLength => match self.controller.selection() {
                Selection::Tool(i) => setup
                    .tools
                    .get(i)
                    .map(|t| (t.length - t.flute_len()).max(0.0)),
                _ => None,
            },
            Field::Flutes => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.flutes as f64),
                _ => None,
            },
            Field::NominalRpm => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.nominal_rpm),
                _ => None,
            },
            Field::NominalFeed => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.nominal_feed),
                _ => None,
            },
            Field::NominalPlungeFeed => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.nominal_plunge_feed),
                _ => None,
            },
            _ => match self.controller.selection() {
                // Kind-specific tool parameters (corner radius, chamfer angle, …).
                Selection::Tool(i) => setup
                    .tools
                    .get(i)
                    .and_then(|t| tool_kind_field(t.kind, field)),
                Selection::Operation(id) => self
                    .current_operation(id)
                    .and_then(|op| op_field(op, field)),
                _ => None,
            },
        }
    }

    /// Parse the inspector buffers and commit them to the selected node as one
    /// undoable change, then recompute.
    /// Whether a machine name or post is typed but not yet applied.
    fn machine_edit_dirty(&self) -> bool {
        let name = self
            .machine_name_edit
            .as_ref()
            .is_some_and(|n| n != &self.controller.machine().name);
        let post = self
            .machine_post_edit
            .is_some_and(|p| p != self.controller.post_kind());
        let estlcam = self
            .machine_estlcam_edit
            .is_some_and(|o| o != self.controller.estlcam_options());
        name || post || estlcam
    }

    /// Commit any pending machine name / post, writing them into the library.
    fn apply_machine_edit(&mut self) {
        if !self.machine_edit_dirty() {
            return;
        }
        if let Some(name) = self.machine_name_edit.clone() {
            self.controller.edit_machine(|m| m.name = name);
        }
        if let Some(post) = self.machine_post_edit {
            self.controller.set_post_kind(post);
        }
        if let Some(options) = self.machine_estlcam_edit {
            self.controller.set_estlcam_options(options);
        }
        self.sync_active_machine();
        // Re-seed from what was actually stored — `sync_active_machine` may have
        // disambiguated a rename onto a name already taken, and the box must show what
        // the library holds, not what was typed.
        self.machine_name_edit = None;
        self.machine_post_edit = None;
        self.machine_estlcam_edit = None;
        self.status = format!(
            "Machine: {} ({}).",
            self.active_machine,
            self.controller.post_kind()
        );
    }

    fn apply_inspector(&mut self) {
        self.apply_machine_edit();
        let mut parsed: BTreeMap<Field, f64> = BTreeMap::new();
        let supported = self.inspector_fields();
        for &field in self.fields.keys() {
            if !supported.contains(&field) {
                continue;
            }
            match self.parsed_field_value(field) {
                Ok(v) => {
                    parsed.insert(field, v);
                }
                Err(_) => {
                    self.status = format!("{} is not a valid number.", field.label());
                    return;
                }
            }
        }

        // Library-tool editing writes to the library file, not the project — an
        // embedded copy in a project is a snapshot and is left untouched. The working
        // copy already carries every live edit, so committing is a copy-through; refresh
        // then resets it as the new clean baseline.
        if self.library_mode() {
            if self.preset_edit_invalid() {
                self.status = "Correct invalid preset values before applying.".into();
                return;
            }
            if let (Some(edit), Some(slot)) = (
                self.tool_edit.clone(),
                self.library.tools.get_mut(self.lib_sel),
            ) {
                *slot = edit;
            }
            self.save_tool_libraries();
            self.reload_tool_edit();
            return;
        }

        if let Selection::Operation(id) = self.controller.selection() {
            let Some(mut operation) = self
                .operation_draft
                .clone()
                .filter(|draft| draft.id() == id)
                .or_else(|| self.controller.operation(id).cloned())
            else {
                self.status = "The selected operation no longer exists.".into();
                return;
            };
            apply_op_fields(&mut operation, &parsed);
            if let Err(error) = self.apply_parameter_sources(&mut operation) {
                self.report_error(error);
                return;
            }
            let parse_override = |value: &str| -> Result<Option<f64>, String> {
                if value.trim().is_empty() {
                    Ok(None)
                } else {
                    value
                        .trim()
                        .parse::<f64>()
                        .ok()
                        .filter(|v| v.is_finite())
                        .map(Some)
                        .ok_or_else(|| "Linked heights must be finite numbers or blank.".into())
                }
            };
            let overrides = if self.controller.geometry_state().bindings.contains_key(&id) {
                match parse_override(&self.cad_override_top).and_then(|top| {
                    parse_override(&self.cad_override_bottom)
                        .map(|bottom| cam_model::GeometryOverrides { top, bottom })
                }) {
                    Ok(value) => Some(value),
                    Err(error) => {
                        self.status = error;
                        return;
                    }
                }
            } else {
                None
            };
            let used_tools = operation.tools();
            if let Err(reason) = self.validate_rotary_index_draft() {
                self.report_error(reason);
                return;
            }
            let tools = self
                .operation_tools
                .iter()
                .filter(|tool| used_tools.contains(&tool.number))
                .cloned()
                .collect();
            let heights = self.preview_safety_heights(id);
            let result = if self.height_rules_dirty()
                || self
                    .controller
                    .inspection_setup()
                    .height_settings
                    .contains_key(&id)
            {
                match self.edited_height_settings() {
                    Ok(settings) => self
                        .controller
                        .apply_operation_height_settings_and_stock_and_index(
                            operation,
                            overrides,
                            tools,
                            self.operation_geometry.clone(),
                            settings,
                            self.stock_to_leave_with_fields(),
                            self.rotary_index_draft.clone(),
                        ),
                    Err(reason) => Err(reason),
                }
            } else {
                self.controller
                    .apply_operation_settings_with_heights_and_stock_and_index(
                        operation,
                        overrides,
                        tools,
                        self.operation_geometry.clone(),
                        Some(heights),
                        self.stock_to_leave_with_fields(),
                        self.rotary_index_draft.clone(),
                    )
            };
            match result {
                Ok(()) => {
                    // Drop the staged copy after committing so refresh_fields reads
                    // the newly saved setup value instead of the stale pre-Apply draft.
                    self.stock_to_leave_draft = None;
                    self.operation_draft = None;
                    self.operation_tools.clear();
                    self.rebase_setup_draft();
                    self.seed_height_editor(id);
                    self.refresh_fields();
                    self.rerun();
                    self.operation_geometry = None;
                    self.operation_binding_preview = None;
                    self.status = format!("Operation {id} settings applied.");
                }
                Err(error) => self.report_error(error),
            }
            return;
        }

        match self.controller.selection() {
            Selection::Setup => self.controller.edit_setup(|s| {
                if let Some(&v) = parsed.get(&Field::ToolChangeHeight) {
                    s.tool_change_height = Some(v + s.origin[2]);
                }
            }),
            Selection::Machine => self.controller.edit_machine(|m| {
                // Travel is the working-volume extent. Z is anchored at its
                // separately editable maximum relative to the base work origin.
                let e = &mut m.envelope;
                if let Some(&v) = parsed.get(&Field::MachineTravelX) {
                    e.max.x = e.min.x + v.max(0.0);
                }
                if let Some(&v) = parsed.get(&Field::MachineTravelY) {
                    e.max.y = e.min.y + v.max(0.0);
                }
                let z_travel = parsed
                    .get(&Field::MachineTravelZ)
                    .copied()
                    .unwrap_or(e.max.z - e.min.z);
                let max_z = parsed.get(&Field::MachineMaxZ).copied().unwrap_or(e.max.z);
                set_machine_z_limits(e, z_travel, max_z);
                if let Some(&v) = parsed.get(&Field::MachineRapidRate) {
                    m.rapid_rate = v;
                }
                if let Some(&v) = parsed.get(&Field::MachineMinSpindleRpm) {
                    m.min_spindle_rpm = v;
                }
                if let Some(&v) = parsed.get(&Field::MachineMaxSpindleRpm) {
                    m.max_spindle_rpm = v;
                }
                if let Some(&v) = parsed.get(&Field::MachineMaxFeed) {
                    m.max_feed = v;
                }
                if let Some(&v) = parsed.get(&Field::MachineToolChangeSeconds) {
                    m.tool_change_seconds = v;
                }
            }),
            Selection::Origin => {
                self.controller.edit_active_origin(|o| {
                    if let Some(&v) = parsed.get(&Field::OriginX) {
                        o[0] = v;
                    }
                    if let Some(&v) = parsed.get(&Field::OriginY) {
                        o[1] = v;
                    }
                    if let Some(&v) = parsed.get(&Field::OriginZ) {
                        o[2] = v;
                    }
                });
                // The H index applies on Apply (like every field). A round to the
                // nearest positive integer; changing it may swap with another origin.
                if let Some(&v) = parsed.get(&Field::OriginIndex) {
                    let idx = v.round().max(1.0) as u32;
                    if idx != self.controller.active_origin() {
                        self.controller.set_active_origin_index(idx);
                    }
                }
            }
            Selection::Tool(i) => self.controller.edit_tool(i, |t| {
                apply_tool_dims(t, &parsed);
                apply_tool_kind_fields(&mut t.kind, &parsed);
            }),
            Selection::Operation(_) => {}
            Selection::Stock => self.controller.edit_stock(|stock| {
                if let cam_model::Stock::BoundingBox {
                    x_offset,
                    y_offset,
                    top,
                    thickness,
                } = stock
                {
                    if let Some(&v) = parsed.get(&Field::StockXOffset) {
                        *x_offset = v.max(0.0);
                    }
                    if let Some(&v) = parsed.get(&Field::StockYOffset) {
                        *y_offset = v.max(0.0);
                    }
                    if let Some(&v) = parsed.get(&Field::StockTop) {
                        *top = v;
                    }
                    if let Some(&v) = parsed.get(&Field::StockThickness) {
                        *thickness = v.max(0.0);
                    }
                }
            }),
        }
        // The machine library snapshot was synced before numeric fields were applied.
        // Sync once more now so the numeric machine settings persist with the edit.
        if self.controller.selection() == Selection::Machine {
            self.sync_active_machine();
        }
        self.refresh_fields();
        self.rerun();
    }

    /// Shared by startup loading and the native Open dialog.
    fn open_project(&mut self, path: &Path) -> Result<(), crate::ProjectError> {
        self.pending_processing_time = None;
        self.processing_time_dialog = None;
        self.controller.open_project(path)?;
        self.workspace = Workspace::Machining;
        self.active_tab = RibbonTab::Machining;
        self.machining_tab = RibbonTab::Machining;
        self.status_messages = StatusMessages::default();
        self.reset_model_browser();
        self.planar_selection.clear();
        // Matched project tools adopt the shop library's numbering.
        let report = self.controller.reconcile_tools(&self.library.tools);
        self.focus_ops.clear();
        self.end_setup_draft();
        self.restore_project_view();
        self.operation_draft = None;
        if let Selection::Operation(id) = self.controller.selection() {
            self.seed_rotary_index_draft(id);
        } else {
            self.rotary_index_draft = None;
        }
        // Establish a baseline for legacy files too, using their default opening view.
        self.controller
            .set_project_view(Some(self.capture_project_view()));
        self.refresh_fields();
        // Rebuild the preview from saved cutting fragments only when current.
        // Opening a stale/legacy project must not initiate a recalculation.
        let statuses = self.controller.toolpath_statuses();
        let document = self.controller.document();
        if self.controller.outcome().is_none()
            && document.active_setup().is_some_and(|setup| {
                setup
                    .operations
                    .iter()
                    .filter(|op| document.operation_is_included(document.setup().id, op.id()))
                    .all(|op| statuses.get(&op.id()).copied().unwrap_or(false))
            })
        {
            self.rerun();
        } else if self.controller.outcome().is_some() {
            self.record_status_messages();
        }
        self.status = match report.summary() {
            Some(summary) => format!("Opened {}. Tools: {summary}.", path.display()),
            None => format!("Opened {}.", path.display()),
        };
        Ok(())
    }

    /// Save the project to `path` and report the result.
    fn save_to(&mut self, path: &Path) {
        let previous_view = self.controller.project_view().cloned();
        self.controller
            .set_project_view(Some(self.capture_project_view()));
        self.status = match self.controller.save_project(path) {
            Ok(()) => format!("Saved {}.", path.display()),
            Err(e) => {
                self.controller.set_project_view(previous_view);
                format!("Save failed: {e}")
            }
        };
    }

    fn title_for_window(&self, id: iced::window::Id) -> String {
        if self.is_preferences_window(id) {
            "Preferences".into()
        } else {
            self.window_title()
        }
    }

    fn window_view(&self, id: iced::window::Id) -> Element<'_, Message> {
        if let Some(prefs) = self.preferences.as_ref().filter(|prefs| prefs.id == id) {
            prefs.view(self).map(Message::Preferences)
        } else if self.main_window == Some(id) {
            self.view()
        } else {
            Space::new().into()
        }
    }

    fn window_title(&self) -> String {
        let project_name = self
            .controller
            .current_path()
            .and_then(Path::file_stem)
            .map(|name| name.to_string_lossy())
            .unwrap_or_else(|| "New Project".into());
        let unsaved_marker = if self.controller.has_unsaved_changes() || self.project_view_changed()
        {
            " *"
        } else {
            ""
        };

        format!("Open CAM Studio - {project_name}{unsaved_marker}")
    }

    fn view(&self) -> Element<'_, Message> {
        let grid = PaneGrid::new(&self.panes, |_id, pane, _is_max| {
            pane_grid::Content::new(self.pane_content(*pane)).title_bar(
                pane_grid::TitleBar::new(
                    text(if *pane == Pane::Viewport {
                        self.workspace.label()
                    } else {
                        pane.name()
                    })
                    .size(13),
                )
                .padding(4),
            )
        })
        .spacing(4)
        .min_size(PANE_MIN_PX)
        .on_resize(PANE_RESIZE_LEEWAY, Message::PaneResized)
        .on_drag(Message::PaneDragged)
        .width(Length::Fill)
        .height(Length::Fill);

        let mut content = column![self.ribbon(), grid];
        if let Some(progress) = self.calculation_progress() {
            content = content.push(progress);
        }
        let mut layers = iced::widget::stack![content];
        if let Some(popup) = self.ribbon_popup() {
            // A full-window catcher under the popup so a click off it dismisses.
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseRibbonPopup);
            layers = layers.push(catcher).push(popup);
        }
        if let Some(menu) = self.tree_menu_overlay() {
            // A full-window catcher under the menu so a click off it dismisses.
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::Tree(TreeMessage::Close));
            layers = layers.push(catcher).push(menu);
        }
        if let Some(menu) = self.tool_menu_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseToolMenu);
            layers = layers.push(catcher).push(menu);
        }
        if let Some(menu) = self.library_context_menu_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseLibraryContextMenu);
            layers = layers.push(catcher).push(menu);
        }
        if let Some(menu) = self.lib_menu_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseLibMenu);
            layers = layers.push(catcher).push(menu);
        }
        if let Some(menu) = self.parameter_menu_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseParameterMenu)
                .on_right_press(Message::CloseParameterMenu)
                .on_scroll(|_| Message::CloseParameterMenu);
            layers = layers.push(catcher).push(menu);
        }
        if let Some(dialog) = self.operation_tool_selector_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseOperationToolSelector);
            layers = layers.push(catcher).push(dialog);
        }
        if let Some(menu) = self.tool_table_menu_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseToolTableMenu);
            layers = layers.push(catcher).push(menu);
        }
        if let Some(pickbox) = self.pickbox_overlay() {
            layers = layers.push(pickbox);
        }
        if let Some(card) = self.license_overlay() {
            // Same catcher pattern as the menus: a click anywhere off the card closes.
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseLicense);
            layers = layers.push(catcher).push(card);
        }
        if let Some(dialog) = self.fusion_import_overlay() {
            layers = layers.push(dialog);
        }
        if let Some(dialog) = self.tool_management_overlay() {
            layers = layers.push(dialog);
        }
        if let Some(dialog) = self.processing_time_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::ProcessingTimeClose);
            layers = layers.push(catcher).push(dialog);
        }
        if let Some(dialog) = self.tool_info_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::ToolInfoClose);
            layers = layers.push(catcher).push(dialog);
        }
        if let Some(dialog) = self.notice_overlay() {
            layers = layers.push(dialog);
        }
        if let Some(dialog) = self.unsaved_overlay() {
            layers = layers.push(dialog);
        }
        layers.into()
    }

    fn fusion_import_overlay(&self) -> Option<Element<'_, Message>> {
        let entries = self.fusion_import.as_ref()?;
        let selected = entries.iter().filter(|entry| entry.selected).count();
        let mut list = column![].spacing(8);
        for (index, entry) in entries.iter().enumerate() {
            let mut item = column![].spacing(3);
            let label = if let Some(tool) = &entry.tool {
                format!(
                    "{}  —  {} · ⌀{:.2} mm",
                    entry.label, tool.kind, tool.diameter
                )
            } else {
                entry.label.clone()
            };
            let check = checkbox(entry.selected)
                .label(label)
                .size(16)
                .text_size(13)
                .on_toggle_maybe(
                    entry
                        .tool
                        .as_ref()
                        .map(|_| move |value| Message::FusionImportToggle(index, value)),
                );
            item = item.push(check);
            if !entry.detail.is_empty() {
                item = item.push(text(&entry.detail).size(11).color(INSPECTOR_MUTED));
            }
            list = list.push(item);
        }
        let body = column![
            text("Import Fusion tools").size(18),
            text("Choose tools to add to your library. Existing tools are kept; imported tools receive free numbers.").size(12),
            text("Check direction and cutting data after import. Fusion coolant and preset step settings are not imported.").size(11).color(INSPECTOR_MUTED),
            row![inspector_button("Select all").on_press(Message::FusionImportSelectAll(true)),
                 inspector_button("Select none").on_press(Message::FusionImportSelectAll(false))].spacing(8),
            scrollable(list).height(Length::Fill),
            row![text(format!("{selected} selected")).size(12), Space::new().width(Length::Fill),
                 inspector_button("Cancel").on_press(Message::FusionImportCancel),
                 inspector_button("Import").on_press_maybe((selected > 0).then_some(Message::FusionImportConfirm))].spacing(8)
        ].spacing(12);
        let card = container(body)
            .width(650)
            .height((self.window.height - 60.).clamp(200., 700.))
            .padding(16)
            .style(|theme: &iced::Theme| container::Style {
                background: Some(Background::Color(
                    theme.extended_palette().background.weak.color,
                )),
                border: Border {
                    color: palette::SEPARATOR,
                    width: 1.,
                    radius: 6.0.into(),
                },
                ..Default::default()
            });
        let backdrop = mouse_area(
            container(Space::new().width(Length::Fill).height(Length::Fill)).style(|_| {
                container::Style {
                    background: Some(Background::Color(Color::from_rgba(0., 0., 0., 0.45))),
                    ..Default::default()
                }
            }),
        )
        .on_press(Message::Noop);
        Some(
            iced::widget::stack![
                backdrop,
                container(iced::widget::opaque(card))
                    .width(Length::Fill)
                    .height(Length::Fill)
                    .align_x(Alignment::Center)
                    .align_y(Alignment::Center)
            ]
            .into(),
        )
    }

    /// The licence-and-credits overlay (View tab -> About -> Licence). `None` unless
    /// open.
    ///
    /// The **full GPL text is embedded**, not merely referenced: GPL-3.0 section 4
    /// asks that a copy of the licence accompany the program, and a shipped binary --
    /// an AppImage, a .dmg, an MSI -- travels without the repository's LICENSE file
    /// beside it. `include_str!` makes the copy part of the executable, so it is
    /// there wherever the program is.
    ///
    /// The credits below are the same facts as `assets/icons/CREDITS.md`, restated
    /// here because a user who has only the binary cannot read that file. They are
    /// obligations, not courtesies: the ribbon icons are GPL-3.0 from OpenCADStudio,
    /// and the application icon is the project's one CC BY-SA 4.0 exception.
    fn license_overlay(&self) -> Option<Element<'_, Message>> {
        if !self.show_license {
            return None;
        }
        const GPL: &str = include_str!("../../../LICENSE");

        let heading = |t: &'static str| text(t).size(14);
        let body = |t: String| text(t).size(12);

        let credits = column![
            heading("Third-party work"),
            body(
                "Ribbon icons — several are taken unmodified from OpenCADStudio \
                 (github.com/HakanSeven12/OpenCADStudio), © its contributors, GPL-3.0, \
                 reused here under the same licence. The CAM-specific icons are original."
                    .to_string()
            ),
            body(
                "Application icon — © Andreas Bertsatos, CC BY-SA 4.0 International. \
                 This is a deliberate exception to this program's GPL-3.0-only licence, \
                 so the mark may be used and attributed on Wikimedia. Its letterforms \
                 are outlines of DejaVu Sans Bold (Bitstream Vera Fonts Copyright)."
                    .to_string()
            ),
            body(
                "DXF/DWG import — acadrust, MPL-2.0. Geometry, toolpaths, posts and the \
                 simulator are original to this project."
                    .to_string()
            ),
        ]
        .spacing(6);
        #[cfg(feature = "occt")]
        let credits = credits.push(body(
            "STEP/IGES import uses Open CASCADE Technology (OCCT), \
             Copyright © Open CASCADE SAS and contributors. OCCT is licensed under \
             LGPL 2.1 with the Open CASCADE exception. The licence and exception are \
             included in the package's licences directory and are available at \
             https://occt3d.com/dev/doc/overview/html/occt_public_license.html."
                .to_string(),
        ));

        let card = container(
            column![
                row![
                    iced::widget::svg(Icon::License.handle())
                        .width(30)
                        .height(30),
                    text(format!("Open CAM Studio {}", crate::version_string())).size(20),
                ]
                .spacing(10)
                .align_y(Alignment::Center),
                body("A CAM application for CNC toolpath generation.".to_string()),
                body("Copyright © 2026 Andreas Bertsatos.".to_string()),
                heading("Licence"),
                body(
                    "This program is free software: you can redistribute it and/or modify \
                     it under the terms of the GNU General Public License, version 3 only, \
                     as published by the Free Software Foundation.\n\n\
                     This program is distributed in the hope that it will be useful, but \
                     WITHOUT ANY WARRANTY; without even the implied warranty of \
                     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU \
                     General Public License below for more details."
                        .to_string()
                ),
                credits,
                heading("GNU General Public License, version 3"),
                container(scrollable(text(GPL).size(11).font(iced::Font::MONOSPACE)).height(260))
                    .padding(8)
                    .style(|theme: &iced::Theme| container::Style {
                        background: Some(Background::Color(theme.palette().background)),
                        border: Border {
                            color: theme.extended_palette().background.strong.color,
                            width: 1.0,
                            radius: 3.0.into(),
                        },
                        ..Default::default()
                    }),
                row![
                    Space::new().width(Length::Fill),
                    // Explicit padding and a 1.0 line height, matching the other
                    // buttons in the app. Left to iced's defaults the label sits high
                    // in the button: the default line box for a 13pt text is taller
                    // than the glyphs, and uniform default padding centres the BOX,
                    // not the ink inside it.
                    button(
                        text("Close")
                            .size(13)
                            .line_height(iced::widget::text::LineHeight::Relative(1.0))
                            .align_y(Alignment::Center)
                    )
                    .padding(Padding::from([5.0, 14.0]))
                    .on_press(Message::CloseLicense),
                ]
                .align_y(Alignment::Center),
            ]
            .spacing(10)
            .padding(4),
        )
        .width(620)
        .padding(16)
        .style(|theme: &iced::Theme| container::Style {
            background: Some(Background::Color(
                theme.extended_palette().background.weak.color,
            )),
            border: Border {
                color: theme.extended_palette().background.strong.color,
                width: 1.0,
                radius: 6.0.into(),
            },
            ..Default::default()
        });

        Some(
            container(card)
                .width(Length::Fill)
                .height(Length::Fill)
                .align_x(Alignment::Center)
                .align_y(Alignment::Center)
                .into(),
        )
    }

    /// The origins operation `id` could be moved to. Thin wrapper over
    /// [`origin_move_targets`], which holds the rule and can be tested without an app.
    fn origin_move_targets_for(&self, id: u32) -> Vec<u32> {
        let current = self
            .controller
            .inspection_setup()
            .operations
            .iter()
            .find(|op| op.id() == id)
            .map(|op| op.work_offset());
        origin_move_targets(
            &self.controller.origin_indices(),
            current,
            self.controller.base_origin_index(),
        )
    }

    /// How an origin reads wherever it is named — `"Origin 2 · G55"`, or plain
    /// `"Origin 7"` when the selected post has no word for that datum (the case the tree
    /// marks with a ⚠). The vocabulary comes from the post, never from a `match` here.
    ///
    /// Used by the tree header, the move-to menu and the status line, so a job cannot
    /// call the same origin two different things depending on where you read it.
    fn origin_menu_label(&self, index: u32) -> String {
        match self.controller.datum_label(index) {
            Some(word) => format!("Origin {index} · {word}"),
            None => format!("Origin {index}"),
        }
    }

    /// The operation right-click context menu (Delete / Duplicate), its top-left
    /// anchored exactly under the cursor. `None` unless a menu is open. Reuses the
    /// ribbon-popup overlay pattern: positioned in the top-level view stack over a
    /// click-off catcher.
    fn tool_menu_overlay(&self) -> Option<Element<'_, Message>> {
        let number = self.open_tool_menu?;
        let menu = container(
            button(
                row![
                    icon_svg(Icon::ImportLibrary, 14.0),
                    text("Add to library").size(13)
                ]
                .spacing(6)
                .align_y(Alignment::Center),
            )
            .width(Length::Fixed(150.0))
            .padding(Padding::from([4.0, 8.0]))
            .on_press(Message::AddToolToLibrary(number))
            .style(|_theme, status| command_button_style(status)),
        )
        .padding(4)
        .style(|_theme| container::Style {
            background: Some(Background::Color(palette::RIBBON_BG)),
            border: Border {
                color: palette::BORDER_DARK,
                width: 1.0,
                radius: 3.0.into(),
            },
            ..container::Style::default()
        });
        let positioned = container(menu)
            .width(Length::Fill)
            .height(Length::Fill)
            .align_x(iced::alignment::Horizontal::Left)
            .align_y(iced::alignment::Vertical::Top)
            .padding(Padding {
                top: self.tool_menu_pos.y,
                left: self.tool_menu_pos.x,
                right: 0.0,
                bottom: 0.0,
            });
        Some(positioned.into())
    }

    fn library_context_menu_overlay(&self) -> Option<Element<'_, Message>> {
        let pos = self.library_context_menu?;
        let item = |label: &'static str, message| {
            button(text(label).size(13))
                .width(Length::Fill)
                .padding([6, 10])
                .on_press(message)
                .style(|_, status| command_button_style(status))
        };
        let popup = container(column![
            item("New library", Message::NewToolLibrary),
            item("Import library…", Message::ImportLibrary),
            item("Rename", Message::BeginLibraryRename),
            item("Export library…", Message::ExportLibrary),
            item("Delete", Message::DeleteToolLibrary),
        ])
        .width(190)
        .padding(4)
        .style(|_| container::Style {
            background: Some(Background::Color(palette::RIBBON_BG)),
            border: Border {
                color: palette::BORDER_DARK,
                width: 1.0,
                radius: 3.0.into(),
            },
            ..container::Style::default()
        });
        Some(
            container(popup)
                .width(Length::Fill)
                .height(Length::Fill)
                .align_x(iced::alignment::Horizontal::Left)
                .align_y(iced::alignment::Vertical::Top)
                .padding(Padding {
                    top: pos.y.min((self.window.height - 190.0).max(0.0)),
                    left: pos.x.min((self.window.width - 200.0).max(0.0)),
                    right: 0.0,
                    bottom: 0.0,
                })
                .into(),
        )
    }

    /// The Library-pane right-click menu, anchored under the cursor. Shows a "Set number…"
    /// item that morphs into a small number input (Enter or Set commits; swaps on clash).
    fn lib_menu_overlay(&self) -> Option<Element<'_, Message>> {
        let menu = self.lib_menu.as_ref()?;
        let content: Element<'_, Message> = match &menu.input {
            None => button(text("Set number…").size(13))
                .width(Length::Fixed(140.0))
                .padding(Padding::from([4.0, 8.0]))
                .on_press(Message::LibMenuSetNumber)
                .style(|_theme, status| command_button_style(status))
                .into(),
            Some(buf) => row![
                inspector_input("T#", buf)
                    .on_input(Message::LibNumberInput)
                    .on_submit(Message::LibNumberCommit)
                    .width(Length::Fixed(64.0)),
                button(text("Set").size(13)).on_press(Message::LibNumberCommit),
            ]
            .spacing(6)
            .align_y(Alignment::Center)
            .into(),
        };
        let popup = container(content)
            .padding(4)
            .style(|_theme| container::Style {
                background: Some(Background::Color(palette::RIBBON_BG)),
                border: Border {
                    color: palette::BORDER_DARK,
                    width: 1.0,
                    radius: 3.0.into(),
                },
                ..container::Style::default()
            });
        let positioned = container(popup)
            .width(Length::Fill)
            .height(Length::Fill)
            .align_x(iced::alignment::Horizontal::Left)
            .align_y(iced::alignment::Vertical::Top)
            .padding(Padding {
                top: self.lib_menu_pos.y,
                left: self.lib_menu_pos.x,
                right: 0.0,
                bottom: 0.0,
            });
        Some(positioned.into())
    }

    /// The pickbox: a small accent square drawn over the cursor while a geometry
    /// pick is pending, its half-size the vertex-snap aperture. Purely visual — it
    /// does not intercept clicks (they fall through to the viewport).
    fn pickbox_overlay(&self) -> Option<Element<'_, Message>> {
        if self.controller.pending_op().is_none() && !self.in_origin_pick() {
            return None;
        }
        // Once a snap engages, its (bolder) in-scene marker stands in for the
        // pickbox — so the blue aperture square shows only while nothing snaps.
        if self.snap_hover.is_some() || self.setup_hover.snap.is_some() {
            return None;
        }
        let c = self.cursor?;
        let px = self.settings.snapping.pickbox_px;
        let half = px / 2.0;
        let square = container(Space::new())
            .width(Length::Fixed(px))
            .height(Length::Fixed(px))
            .style(|_theme| container::Style {
                border: Border {
                    color: palette::ACCENT_BLUE,
                    width: 1.5,
                    radius: 0.0.into(),
                },
                ..container::Style::default()
            });
        let positioned = container(square)
            .width(Length::Fill)
            .height(Length::Fill)
            .padding(Padding {
                top: (c.y - half).max(0.0),
                left: (c.x - half).max(0.0),
                right: 0.0,
                bottom: 0.0,
            });
        Some(positioned.into())
    }

    /// The top-bar ribbon: a tab strip (darker bar, active tab in accent) over a
    /// framed band showing the active tab's grouped commands. Styled after
    /// OpenCADStudio's ribbon (see the [`palette`] module).
    fn ribbon(&self) -> Element<'_, Message> {
        let tab_btn = |tab: RibbonTab| {
            let active = tab == self.active_tab;
            button(text(tab.label()).size(12))
                .padding(Padding::from([5.0, 14.0]))
                .on_press(Message::SelectRibbonTab(tab))
                .style(move |_theme, status| tab_button_style(active, status))
        };
        let mut tabs = row![].spacing(6);
        for tab in RibbonTab::ALL {
            tabs = tabs.push(tab_btn(tab));
        }
        // `About` is NOT a tab: it opens the licence-and-credits popup instead of
        // switching the band below. It sits immediately after View, where a user
        // looks for it, rather than exiled to the window's right edge. A tab strip
        // does promise that every entry changes the ribbon under it, so a hairline
        // divider marks the boundary -- enough to say "different kind of thing"
        // without separating it from the group it belongs with.
        tabs = tabs
            .push(
                // The colour goes on the 1px inner container; the padding that spaces
                // it from its neighbours goes on the outer one. A container paints its
                // background across its padding as well, so styling the padded
                // container directly would draw a 15px filled block, not a hairline.
                container(container(Space::new().width(1).height(16)).style(|_theme| {
                    container::Style {
                        background: Some(Background::Color(palette::BORDER_DARK)),
                        ..container::Style::default()
                    }
                }))
                .padding(Padding::from([0.0, 7.0])),
            )
            .push(
                button(text("Preferences").size(12))
                    .padding(Padding::from([5.0, 14.0]))
                    .on_press(Message::ShowPrefs)
                    .style(|_theme, status| tab_button_style(false, status)),
            )
            .push(
                button(text("About").size(12))
                    .padding(Padding::from([5.0, 14.0]))
                    .on_press(Message::ShowLicense)
                    .style(|_theme, status| tab_button_style(false, status)),
            );
        let strip = container(tabs.padding(2).align_y(Alignment::Center))
            .width(Length::Fill)
            .style(|_theme| container::Style {
                background: Some(Background::Color(palette::TOPBAR_BG)),
                ..container::Style::default()
            });

        let band = container(self.ribbon_body())
            .padding(6)
            .width(Length::Fill)
            .style(|_theme| container::Style {
                background: Some(Background::Color(palette::RIBBON_BG)),
                border: Border {
                    color: palette::BORDER_DARK,
                    width: 1.0,
                    radius: 0.0.into(),
                },
                ..container::Style::default()
            });

        column![strip, band].into()
    }

    /// The icon-command groups for the active tab. Always non-empty — the pane
    /// toggles and the cube slider are appended separately by [`Self::ribbon_body`],
    /// since neither is an icon command the density solver can size.
    fn ribbon_specs(&self) -> Vec<GroupSpec> {
        let has_geo = self.controller.has_geometry();
        // A new op needs geometry to pick; its tool is drawn from the library, so the
        // library must have at least one tool (it seeds defaults, so it always does).
        let can_create =
            has_geo && !self.controller.setups().is_empty() && !self.library.tools.is_empty();
        let operation = |icon, label, kind, shortcut| Command {
            operation_hint: Some((
                kind,
                display_shortcut(shortcut_value(&self.settings, shortcut)),
            )),
            ..cmd(icon, label, can_create.then_some(Message::BeginOp(kind)))
        };
        match self.active_tab {
            RibbonTab::Home => vec![
                GroupSpec {
                    title: "Project",
                    commands: vec![
                        cmd_help(
                            Icon::New,
                            "New",
                            Some(Message::NewProject),
                            format!(
                                "{} ({})",
                                Icon::New.help(),
                                display_shortcut(shortcut_value(&self.settings, "new"))
                            ),
                        ),
                        cmd_help(
                            Icon::Open,
                            "Open",
                            Some(Message::OpenProject),
                            format!(
                                "{} ({})",
                                Icon::Open.help(),
                                display_shortcut(shortcut_value(&self.settings, "open"))
                            ),
                        ),
                        cmd_help(
                            Icon::Save,
                            "Save",
                            Some(Message::SaveProject),
                            format!(
                                "{} ({})",
                                Icon::Save.help(),
                                display_shortcut(shortcut_value(&self.settings, "save"))
                            ),
                        ),
                        cmd_help(
                            Icon::Save,
                            "Save As",
                            Some(Message::SaveProjectAs),
                            format!(
                                "Save the project under a new name. ({})",
                                display_shortcut(shortcut_value(&self.settings, "save_as"))
                            ),
                        ),
                    ],
                },
                GroupSpec {
                    title: "Data",
                    commands: vec![
                        cmd(Icon::Import, "Import", Some(Message::ImportCad)),
                        cmd(Icon::Export, "Export", Some(Message::ExportNc)),
                        cmd(Icon::Open, "Sample", Some(Message::OpenSample)),
                    ],
                },
                GroupSpec {
                    title: "Edit",
                    commands: vec![
                        cmd_help(
                            Icon::Undo,
                            "Undo",
                            Some(Message::Undo),
                            format!(
                                "{} ({})",
                                Icon::Undo.help(),
                                display_shortcut(shortcut_value(&self.settings, "undo"))
                            ),
                        ),
                        cmd_help(
                            Icon::Redo,
                            "Redo",
                            Some(Message::Redo),
                            format!(
                                "{} ({})",
                                Icon::Redo.help(),
                                display_shortcut(shortcut_value(&self.settings, "redo"))
                            ),
                        ),
                        if self.controller.run_pending() {
                            cmd(Icon::Run, "Cancel run", Some(Message::CancelRun))
                        } else {
                            cmd(Icon::Run, "Run", Some(Message::RunJob))
                        },
                    ],
                },
            ],
            RibbonTab::Machining => vec![
                GroupSpec {
                    title: "2D",
                    commands: vec![
                        operation(Icon::Face, "Face", OpKind::Face, "face"),
                        operation(Icon::Profile, "Profile", OpKind::Profile, "profile"),
                        operation(Icon::Pocket, "Pocket", OpKind::Pocket, "pocket"),
                        operation(Icon::Thread, "Thread", OpKind::Thread, "thread"),
                        operation(Icon::Bore, "Bore", OpKind::Bore, "bore"),
                        operation(Icon::Chamfer, "Chamfer", OpKind::Chamfer, "chamfer"),
                        operation(Icon::Engrave, "Engrave", OpKind::Engrave, "engrave"),
                        operation(Icon::Carve, "Carve", OpKind::Carve, "carve"),
                    ],
                },
                GroupSpec {
                    title: "3D",
                    commands: vec![
                        operation(Icon::Adaptive, "3D Adaptive", OpKind::Adaptive, "adaptive"),
                        operation(Icon::Parallel, "3D Parallel", OpKind::Parallel, "parallel"),
                        operation(Icon::Spiral, "3D Spiral", OpKind::Spiral, "spiral"),
                    ],
                },
                GroupSpec {
                    title: "Drilling",
                    commands: vec![operation(Icon::Drill, "Drill", OpKind::Drill, "drill")],
                },
            ],
            RibbonTab::Machinery => vec![GroupSpec {
                title: "Machines",
                commands: vec![
                    // Its own icon, not the tool's — borrowing one is what made "Add
                    // machine" read "Add a new tool to the library".
                    cmd_help(
                        Icon::Machine,
                        "New",
                        Some(Message::NewMachine),
                        "Add a machine to this installation, copied from the active one. \
                         Machines are local: a project records the machine it was built \
                         for but never sets yours, because an export is checked against \
                         the active machine's travel.",
                    ),
                    cmd_help(
                        Icon::Delete,
                        "Delete",
                        // Never the last: an export has to be checked against something.
                        (self.machines.machines.len() > 1).then_some(Message::DeleteMachine),
                        "Remove the active machine. The last one cannot be removed — an \
                         export has to be checked against something.",
                    ),
                ],
            }],
            RibbonTab::Tools => vec![
                GroupSpec {
                    title: "Library",
                    commands: vec![
                        cmd(Icon::NewTool, "New", Some(Message::NewTool)),
                        cmd(Icon::Materials, "Materials", Some(Message::OpenMaterials)),
                        cmd(
                            Icon::Delete,
                            "Delete",
                            // Keep at least one tool in the library.
                            (self.library.tools.len() > 1).then_some(Message::DeleteTool),
                        ),
                        // Bulk renumber (guarded by a confirm dialog).
                        cmd(Icon::Renumber, "Renumber", Some(Message::RenumberLibrary)),
                        // ("Add to library" lives on the project-tree tool right-click menu,
                        // shown only for a project tool that isn't already in the library.)
                    ],
                },
                GroupSpec {
                    title: "Library file",
                    commands: vec![
                        cmd(Icon::ImportLibrary, "Import", Some(Message::ImportLibrary)),
                        cmd(Icon::ExportLibrary, "Export", Some(Message::ExportLibrary)),
                    ],
                },
            ],
            RibbonTab::View => vec![GroupSpec {
                title: "View",
                commands: vec![
                    cmd(Icon::ResetView, "Reset", Some(Message::ResetView)),
                    toggle_cmd(Icon::Info, "Tips", self.tooltips, Message::ToggleTooltips),
                ],
            }],
        }
    }

    /// The densities the active tab's groups should render at, given the window
    /// width.
    fn ribbon_densities(&self, specs: &[GroupSpec]) -> Vec<Density> {
        let counts: Vec<usize> = specs.iter().map(|g| g.commands.len()).collect();
        let available = (self.window.width - RIBBON_CHROME).max(0.0);
        solve_densities(&counts, available)
    }

    /// The groups shown for the active ribbon tab, each at its solved density.
    fn ribbon_body(&self) -> Element<'_, Message> {
        let specs = self.ribbon_specs();
        let densities = self.ribbon_densities(&specs);
        let mut band = row![].align_y(Alignment::Start);
        for (i, (spec, &density)) in specs.iter().zip(&densities).enumerate() {
            if i > 0 {
                band = band.push(ribbon_separator());
            }
            band = band.push(render_group(spec, density, i, self.tooltips));
        }
        // The View tab gets a live orientation-cube size control (a slider has no
        // place in the icon-command band, so it is appended as its own group).
        // The View tab gets the cube-size slider, then the pane-visibility toggles —
        // both sit outside the density solver, which sizes icon-command groups only.
        if self.active_tab == RibbonTab::View {
            band = band.push(ribbon_separator()).push(self.cube_size_group());
            band = band.push(ribbon_separator()).push(self.panes_group());
        }
        band.into()
    }

    /// A labelled slider setting the orientation cube's on-screen size, appended
    /// to the View tab. Disabled (greyed) while the cube is hidden.
    fn cube_size_group(&self) -> Element<'_, Message> {
        let control: Element<'_, Message> = if self.show_gizmo {
            slider(
                GIZMO_SIZE_MIN..=GIZMO_SIZE_MAX,
                self.gizmo_size,
                Message::SetGizmoSize,
            )
            .step(1.0_f32)
            .on_release(Message::SettingsSettled)
            .width(Length::Fixed(140.0))
            .into()
        } else {
            // Keep the footprint stable when the cube is off: an inert placeholder.
            container(
                Space::new()
                    .width(Length::Fixed(140.0))
                    .height(Length::Fixed(16.0)),
            )
            .into()
        };
        let value = text(format!("{} px", self.gizmo_size as i32))
            .size(11)
            .color(palette::RIBBON_CAPTION);
        ribbon_group(
            "Cube size",
            column![control, value]
                .spacing(4)
                .align_x(Alignment::Center),
        )
    }

    /// The Windows tab: a checkbox per pane (naturally narrow, no collapse).
    /// The pane-visibility toggles, appended to the **View** tab as a compact block
    /// captioned "Panes". Laid out in two balanced columns: the ribbon band is only
    /// ~70 px tall, so a single stacked column of four would overflow it.
    ///
    /// The Viewport is always visible and has no toggle. Labels come from
    /// [`Pane::ribbon_label`], so the tool library reads "Tools" here while its own
    /// title bar keeps the full name.
    fn panes_group(&self) -> Element<'_, Message> {
        let toggle = |pane: Pane| -> Element<'_, Message> {
            let shown = match pane {
                Pane::Project => self.show_project_browser,
                Pane::Library => self.workspace == Workspace::Tools,
                Pane::Inspector => self.inspector_enabled,
                _ => self.pane_handle(pane).is_some(),
            };
            row![
                checkbox(shown)
                    .size(15)
                    .on_toggle(move |v| Message::SetPaneVisible(pane, v)),
                text(pane.ribbon_label()).size(12),
            ]
            .spacing(5)
            .align_y(Alignment::Center)
            .into()
        };
        // Derived from ALL_PANES rather than listed, so a pane added later appears
        // here automatically instead of silently going missing.
        let toggles: Vec<Pane> = ALL_PANES
            .into_iter()
            .filter(|p| *p != Pane::Viewport)
            .collect();
        let split = toggles.len().div_ceil(2);
        let mut grid = row![].spacing(12).align_y(Alignment::Start);
        for chunk in toggles.chunks(split) {
            let mut col = column![].spacing(4);
            for &pane in chunk {
                col = col.push(toggle(pane));
            }
            grid = grid.push(col);
        }
        ribbon_group("Panes", grid)
    }

    /// The floating panel for an open collapsed-group popup, positioned under its
    /// button. `None` unless a group is open *and* actually collapsed at the
    /// current width. Its x-offset is the exact sum of the preceding groups' drawn
    /// widths — the analytic layout doubles as popup positioning.
    fn ribbon_popup(&self) -> Option<Element<'_, Message>> {
        let index = self.open_group?;
        let specs = self.ribbon_specs();
        let spec = specs.get(index)?;
        let densities = self.ribbon_densities(&specs);
        if !densities.get(index)?.is_popup() {
            return None;
        }
        let x = 6.0
            + densities[..index]
                .iter()
                .zip(&specs[..index])
                .map(|(&d, s)| group_width(s.commands.len(), d) + GROUP_GAP)
                .sum::<f32>();

        let mut commands = row![].spacing(CMD_GAP);
        for command in &spec.commands {
            commands = commands.push(render_command(command, false, self.tooltips));
        }
        let panel = container(commands)
            .padding(6)
            .style(|_theme| container::Style {
                background: Some(Background::Color(palette::RIBBON_BG)),
                border: Border {
                    color: palette::BORDER_DARK,
                    width: 1.0,
                    radius: 3.0.into(),
                },
                ..container::Style::default()
            });
        // Drop the panel just below the ribbon, offset to the group's column.
        let positioned = container(panel)
            .width(Length::Fill)
            .height(Length::Fill)
            .align_x(iced::alignment::Horizontal::Left)
            .align_y(iced::alignment::Vertical::Top)
            .padding(Padding {
                top: RIBBON_H,
                left: x,
                right: 0.0,
                bottom: 0.0,
            });
        Some(positioned.into())
    }

    fn pane_content(&self, pane: Pane) -> Element<'_, Message> {
        let inner: Element<'_, Message> = match pane {
            Pane::Project => self.project_tree(),
            Pane::Library => self.library_pane(),
            Pane::Viewport => self.workspace_main(),
            Pane::Inspector => {
                if let Some(player) = &self.simulation {
                    player.menu()
                } else if self.library_mode() {
                    self.tools_inspector()
                } else {
                    self.inspector()
                }
            }
            Pane::Output => self.output(),
        };
        // A bordered panel so the gaps between panes read as visible separators.
        container(inner)
            .width(Length::Fill)
            .height(Length::Fill)
            .style(|_theme| container::Style {
                background: Some(Background::Color(palette::PANE_BG)),
                border: Border {
                    color: palette::SEPARATOR,
                    width: 1.0,
                    radius: 0.0.into(),
                },
                ..container::Style::default()
            })
            .into()
    }

    /// The project tree: a structure view of the setup, stock, the tools in use
    /// (read-only), and the operations. Rows are plain selectable text (the selected
    /// row is highlighted, not a button). Each operation row carries inline controls
    /// (include checkbox + reorder arrows) and a right-click menu (Duplicate/Delete).
    /// A workpiece-origin group header in the operation list: `Origin <n> · <datum>`
    /// with its position, a freeze checkbox (drops the group's ops from the run/
    /// viewport), and a delete button (extras only — the base origin is not
    /// removable). Clicking the row makes the origin active so new ops join it and the
    /// inspector edits it.
    ///
    /// The index is the identity — it is what each operation stores and what the
    /// reorder arrows move — and the datum word beside it is the *selected post's*
    /// name for it (`G55` on Fanuc, `H2` on Okuma), so switching post relabels every
    /// row. A post with no word for this datum (a seventh fixture on an ISO control,
    /// which carries only `G54`-`G59`) is marked ⚠ here rather than left to fail at
    /// export, after the job has been built.
    #[allow(dead_code)] // Legacy origin-group view retained for M1a context commands.
    fn origin_header_row(&self, index: u32) -> Element<'_, Message> {
        let pos = self.controller.origin_position(index);
        let disabled = self.controller.is_origin_disabled(index);
        let active = self.controller.selection() == Selection::Origin
            && self.controller.active_origin() == index;
        let freeze = checkbox(!disabled)
            .size(15)
            .on_toggle(move |on| Message::ToggleOriginDisabled(index, !on));
        // The datum word joins the index when the post has one; when it does not, the
        // ⚠ below says so rather than the row saying it twice.
        let datum = self.controller.datum_label(index);
        let named = self.origin_menu_label(index);
        let label = text(format!(
            "{named} — X{} Y{} Z{}",
            fmt_num(pos[0]),
            fmt_num(pos[1]),
            fmt_num(pos[2])
        ))
        .size(13)
        .width(Length::Fill)
        .color(if active {
            palette::LABEL_COLOR
        } else {
            palette::GROUP_LABEL
        });
        let mut controls = row![freeze, label].spacing(6).align_y(Alignment::Center);
        if datum.is_none() {
            // Glyph, not colour — the meaning has to survive colour-vision deficiency.
            controls = controls.push(
                text(format!("⚠ no {} work offset", self.controller.post_kind()))
                    .size(11)
                    .color(palette::WARN),
            );
        }
        if index != self.controller.base_origin_index() {
            controls =
                controls.push(button(text("✕").size(12)).on_press(Message::DeleteOrigin(index)));
        }
        let inner = container(controls.padding(Padding::from([2.0, 6.0])))
            .width(Length::Fill)
            .style(move |_theme| container::Style {
                background: active.then_some(Background::Color(palette::SELECT_BG)),
                border: Border {
                    radius: 3.0.into(),
                    ..Border::default()
                },
                ..container::Style::default()
            });
        mouse_area(inner)
            .on_press(Message::SelectOrigin(index))
            .into()
    }

    fn select_cad_entity(&mut self, entity: EntityRef) {
        if self.rotary_cad_pick(&entity) {
            return;
        }
        let wrapped_pocket =
            matches!(self.operation_draft.as_ref(), Some(Operation::Pocket(p)) if p.wrap.is_some());
        if wrapped_pocket
            && self.operation_tab == OperationTab::Geometry
            && self.geometry_editor.is_none()
            && (self.geometry_auto_pick || self.cad_pending_kind.is_some())
        {
            let filter = match entity.kind {
                EntityKind::Face => Some(CadFilter::Face),
                EntityKind::Loop => Some(CadFilter::Loop),
                EntityKind::Edge => Some(CadFilter::Edge),
                _ => None,
            };
            if let Some(filter) = filter {
                self.operation_geometry_message(GeometryMessage::Add(filter));
            }
        }
        if self.wrapped_pocket_geometry_editor() {
            let additive = (self.geometry_additive && entity.kind == EntityKind::Edge)
                || (self.modifiers.command() && !self.modifiers.control())
                || self.modifiers.shift();
            match self.controller.toggle_cad_selection(entity, additive) {
                Ok(()) => {
                    self.geometry_row_selection = None;
                    self.status =
                        "Select geometry in the viewport, then use OK to stage it.".into();
                }
                Err(error) => self.report_error(error),
            }
            return;
        }
        if self.stock_contour_editor() {
            let additive = entity.kind != EntityKind::Silhouette
                && !self
                    .controller
                    .cad_selection()
                    .iter()
                    .any(|r| r.kind == EntityKind::Silhouette);
            match self.controller.toggle_cad_selection(entity, additive) {
                Ok(()) => self.status = "Stock contour selected; close the chain, then OK.".into(),
                Err(reason) => self.status = reason,
            }
            return;
        }
        if self.profile_entry_cad_pick(entity.clone()) {
            return;
        }
        let auto_edit_id = if self.geometry_auto_pick
            && self.operation_tab == OperationTab::Geometry
            && self.cad_editing_op.is_none()
            && self.cad_pending_kind.is_none()
        {
            match self.controller.selection() {
                Selection::Operation(id)
                    if self.controller.geometry_state().bindings.contains_key(&id) =>
                {
                    Some(id)
                }
                _ => None,
            }
        } else {
            None
        };
        if let Some(id) = auto_edit_id {
            if self.setup_draft_dirty() {
                self.status = "OK or Cancel setup changes before selecting new geometry.".into();
                return;
            }
            if matches!(&self.operation_draft, Some(Operation::Profile(profile)) if profile.selections.is_some())
            {
                let entry_kind = match entity.kind {
                    EntityKind::Face => Some(cam_model::ProfileGeometryKind::FaceContours),
                    EntityKind::Edge | EntityKind::Loop => {
                        Some(cam_model::ProfileGeometryKind::Chain)
                    }
                    EntityKind::Silhouette => Some(cam_model::ProfileGeometryKind::Silhouette),
                    _ => None,
                };
                if let Some(kind) = entry_kind {
                    self.profile_entry_message(profile_entries_view::EntryMessage::Add(kind));
                    self.profile_entry_cad_pick(entity);
                    return;
                }
            }
            let _ = self.update(Message::EditCadGeometry(id));
            self.geometry_additive = true;
            self.geometry_auto_pick = false;
        }
        if self.cad_editing_op.is_none()
            && self.cad_pending_kind.is_none()
            && (self.setup_draft_dirty()
                || (matches!(self.controller.selection(), Selection::Operation(_))
                    && self.inspector_dirty()))
        {
            self.status = "OK or Cancel inspector changes before selecting new geometry.".into();
            return;
        }
        self.adopt_automatic_geometry_pick(&entity);
        self.cad_inspecting = self.cad_editing_op.is_some();
        let profile = self.profile_geometry_active();
        if profile
            && self.profile_selection_mode == ProfileSelectionMode::FaceContours
            && self.profile_coplanar
            && entity.kind == EntityKind::Face
        {
            let result = self
                .indexed_draft_geometry()
                .and_then(|geometry| cam_geometry3d::coplanar_profile_faces(&geometry, &entity));
            match result {
                Ok(references) => {
                    self.controller.set_cad_selection(references);
                    self.geometry_row_selection = None;
                    self.status = "Selected all contours of coplanar faces in this body.".into();
                }
                Err(error) => self.report_error(format!(
                    "{}: {error}",
                    crate::cad_operations::describe_entities(
                        self.controller.geometry_state(),
                        std::slice::from_ref(&entity)
                    )
                )),
            }
            return;
        }
        let pocket_edges = entity.kind == EntityKind::Edge
            && self.geometry_operation_kind() == Some(OpKind::Pocket);
        let multiple = pocket_edges
            || (profile && entity.kind != EntityKind::Silhouette)
            || matches!(
                self.cad_pending_kind,
                Some(OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral)
            )
            || self
                .cad_editing_op
                .and_then(|id| self.current_operation(id))
                .is_some_and(|op| {
                    matches!(
                        op,
                        Operation::Adaptive(_) | Operation::Parallel(_) | Operation::Spiral(_)
                    )
                })
            || matches!(
                self.cad_pending_kind,
                Some(
                    OpKind::Drill
                        | OpKind::Bore
                        | OpKind::Thread
                        | OpKind::Engrave
                        | OpKind::Chamfer
                )
            )
            || self
                .cad_editing_op
                .and_then(|id| self.current_operation(id))
                .is_some_and(|op| {
                    matches!(
                        op,
                        Operation::Drill(_)
                            | Operation::Bore(_)
                            | Operation::Thread(_)
                            | Operation::Engrave(_)
                            | Operation::Chamfer(_)
                    )
                });
        let additive =
            (self.geometry_additive && multiple && entity.kind != EntityKind::Silhouette)
                || (self.modifiers.command() && !self.modifiers.control())
                || self.modifiers.shift();
        self.geometry_row_selection = None;
        let adaptive = matches!(
            self.cad_pending_kind,
            Some(OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral)
        ) || self
            .cad_editing_op
            .and_then(|id| self.current_operation(id))
            .is_some_and(|operation| {
                matches!(
                    operation,
                    Operation::Adaptive(_) | Operation::Parallel(_) | Operation::Spiral(_)
                )
            });
        let selection_result = if adaptive {
            self.controller
                .toggle_adaptive_cad_selection(entity.clone(), additive)
        } else {
            self.controller
                .toggle_cad_selection(entity.clone(), additive)
        };
        match selection_result {
            Ok(()) => {
                if !self.creating_operation() && self.cad_editing_op.is_none() {
                    let selected = self.controller.cad_selection().to_vec();
                    self.clear_browser_selection();
                    self.controller.set_cad_selection(selected);
                }
                self.sync_chamfer_geometry_fields();
                self.reveal_selected_bodies();
                self.sync_creation_geometry_heights();
                self.focus_ops.clear();
                self.status = format!(
                    "{} CAD feature(s) selected.{}",
                    self.controller.cad_selection().len(),
                    if self
                        .controller
                        .cad_selection()
                        .iter()
                        .any(|r| r.kind == EntityKind::Silhouette)
                    {
                        " XY silhouette: whole-body outline projected along setup Z."
                    } else {
                        ""
                    }
                );
            }
            Err(error) => self.report_error(format!(
                "{}: {error}",
                crate::cad_operations::describe_entities(
                    self.controller.geometry_state(),
                    std::slice::from_ref(&entity)
                )
            )),
        }
    }

    fn adopt_automatic_geometry_pick(&mut self, entity: &EntityRef) {
        if self.cad_filter_explicit
            || !(self.geometry_additive || self.geometry_auto_pick)
            || self.operation_tab != OperationTab::Geometry
            || self.setup_model_pick
            || self.setup_axis_pick.is_some()
        {
            return;
        }
        let kind = self
            .cad_pending_kind
            .or_else(|| {
                self.cad_editing_op
                    .and_then(|id| self.current_operation(id))
                    .map(AppController::operation_kind)
            })
            .or_else(|| {
                (self.geometry_auto_pick && self.operation_tab == OperationTab::Geometry)
                    .then(|| self.geometry_operation_kind())
                    .flatten()
            });
        let Some(kind) = kind else {
            return;
        };
        if kind == OpKind::Profile {
            match entity.kind {
                EntityKind::Face => {
                    self.profile_selection_mode = ProfileSelectionMode::FaceContours;
                    self.profile_chain_edges = false;
                    self.cad_filter = CadFilter::Face;
                }
                EntityKind::Edge => {
                    self.profile_selection_mode = ProfileSelectionMode::Chain;
                    self.profile_chain_edges = true;
                    self.cad_filter = CadFilter::Edge;
                }
                EntityKind::Loop => {
                    self.profile_selection_mode = ProfileSelectionMode::Chain;
                    self.profile_chain_edges = false;
                    self.cad_filter = CadFilter::Loop;
                }
                EntityKind::Silhouette => {
                    self.profile_selection_mode = ProfileSelectionMode::Silhouette;
                    self.cad_filter = CadFilter::Body;
                }
                _ => {}
            }
            return;
        }
        self.cad_filter = match (kind, entity.kind) {
            (OpKind::Chamfer, EntityKind::Face) => CadFilter::Face,
            (OpKind::Chamfer, EntityKind::Edge) => CadFilter::Edge,
            (
                OpKind::Pocket | OpKind::Chamfer | OpKind::Engrave | OpKind::Carve,
                EntityKind::Loop,
            ) => CadFilter::Loop,
            (OpKind::Pocket | OpKind::Engrave, EntityKind::Edge) => CadFilter::Edge,
            (OpKind::Pocket | OpKind::Carve, EntityKind::Face) => CadFilter::Face,
            (OpKind::Thread | OpKind::Drill, EntityKind::Face) => CadFilter::Face,
            (OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral, EntityKind::Face) => {
                CadFilter::Face
            }
            (OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral, EntityKind::Body) => {
                CadFilter::Body
            }
            (OpKind::Face, EntityKind::Silhouette) => CadFilter::Silhouette,
            _ => self.cad_filter,
        };
    }

    /// The object-snap toggle row shown during a pick. Enabled snaps read as
    /// filled; Nearest is the opt-in fallback. (Quadrant arrives in Phase 2.)
    fn snap_toolbar(&self) -> Element<'_, Message> {
        let toggle = |kind: SnapKind, label: &'static str| {
            let on = self.snaps.contains(&kind);
            button(text(label).size(12))
                .padding(Padding::from([3.0, 8.0]))
                .on_press(Message::ToggleSnap(kind))
                .style(move |_theme, status| snap_toggle_style(on, status))
        };
        column![
            text("Object snaps").size(12).color(palette::GROUP_LABEL),
            row![
                toggle(SnapKind::End, "End"),
                toggle(SnapKind::Mid, "Mid"),
                toggle(SnapKind::Quadrant, "Quad"),
                toggle(SnapKind::Nearest, "Nearest"),
            ]
            .spacing(6),
        ]
        .spacing(4)
        .into()
    }

    /// The **Tool Library pane** — a selectable list of every library tool, in one of
    /// two internal tabs: **Serial** (by number) or **Family** (grouped by kind, sorted
    /// by diameter). Substitutes for the Project pane on the Tools tab; the selected
    /// tool's *fields* live in the Inspector (kept clean). New / Delete are on the
    /// Tools ribbon.
    fn library_pane(&self) -> Element<'_, Message> {
        let tab = |label: &str, view: LibraryView| {
            let active = self.library_view == view;
            button(text(label.to_string()).size(12))
                .padding(Padding::from([3.0, 12.0]))
                .on_press(Message::SetLibraryView(view))
                .style(move |_theme, status| snap_toggle_style(active, status))
        };
        let mut list = column![row![
            tab("Ordered", LibraryView::Ordered),
            tab("Grouped", LibraryView::Grouped),
        ]
        .spacing(6)]
        .spacing(4)
        .padding(6);

        if self.library.tools.is_empty() {
            list = list.push(tree_note("empty — add a tool with New"));
        }

        // (index, tool) pairs so a selection keeps the real library index whatever the
        // display order.
        let mut items: Vec<(usize, &cam_model::Tool)> =
            self.library.tools.iter().enumerate().collect();
        match self.library_view {
            LibraryView::Ordered => {
                // Number **first** in the library (the number is the tool's identity
                // here) — the reverse of the Project pane, e.g. "T1: ⌀6 End mill (2 flutes)".
                items.sort_by_key(|(_, t)| t.number);
                for (i, t) in items {
                    let base = format!("T{}: ⌀{} {}", t.number, fmt_num(t.diameter), t.kind);
                    let label = match library_extra(t) {
                        Some(extra) => format!("{base} ({extra})"),
                        None => base,
                    };
                    list = list.push(
                        mouse_area(select_row(
                            label,
                            i == self.lib_sel,
                            Message::SelectLibraryTool(i),
                        ))
                        .on_right_press(Message::LibToolMenu(i)),
                    );
                }
            }
            LibraryView::Grouped => {
                items.sort_by(|(_, a), (_, b)| {
                    kind_order(a.kind)
                        .cmp(&kind_order(b.kind))
                        .then(a.diameter.total_cmp(&b.diameter))
                        .then(a.number.cmp(&b.number))
                });
                let mut current: Option<String> = None;
                for (i, t) in items {
                    let fam = t.kind.to_string();
                    if current.as_deref() != Some(fam.as_str()) {
                        // Category headers are plural ("Square End Mills", "Drills", …).
                        list = list.push(tree_header(&format!("{fam}s")));
                        current = Some(fam);
                    }
                    // Kind is the group header, so the row omits it: "T2: ⌀6 (4 flutes)".
                    let base = format!("T{}: ⌀{}", t.number, fmt_num(t.diameter));
                    let label = match library_extra(t) {
                        Some(extra) => format!("{base} ({extra})"),
                        None => base,
                    };
                    list = list.push(
                        mouse_area(select_row(
                            label,
                            i == self.lib_sel,
                            Message::SelectLibraryTool(i),
                        ))
                        .on_right_press(Message::LibToolMenu(i)),
                    );
                }
            }
        }

        scrollable(list)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    /// The Tools-tab tool-library editor in the **Inspector**: just the selected
    /// tool's editable fields (the list lives in the Tool Library pane). New / Delete
    /// live on the Tools ribbon tab.
    fn library_editor(&self) -> Element<'_, Message> {
        // Read the header from the working copy so ⌀/kind track the live edits, matching
        // the viewport preview (the committed list/pane updates on Apply).
        let name = self
            .preview_tool()
            .map(|t| {
                format!(
                    "T{} — ⌀{} {}",
                    t.number,
                    self.settings.format_number(t.diameter),
                    t.kind
                )
            })
            .unwrap_or_else(|| "Tool Library".to_string());
        let mut list = column![inspector_heading(name)].spacing(6).padding(14);

        if let Some(t) = self.preview_tool() {
            // Keep the label flexible so long tool types fit a narrow inspector.
            list = list.push(
                row![
                    help_wrap(
                        text("Type").width(Length::Fill).size(12),
                        help::TOOL_TYPE,
                        self.tooltips,
                    ),
                    pick_list(
                        &ToolKindPick::ALL[..],
                        Some(ToolKindPick::of(t.kind)),
                        |p| { Message::ToolKindChanged(p.to_kind()) }
                    )
                    .text_size(12)
                    .padding([7, 9])
                    .style(inspector_picker)
                    .width(Length::Fixed(177.0)),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
        }

        if self
            .preview_tool()
            .is_some_and(|tool| matches!(tool.kind, ToolKind::RoughingMill))
        {
            list = list.push(text("Roughing serrations are not modeled; simulation uses the nominal flat-end envelope.")
                .size(11).color(INSPECTOR_MUTED));
        }

        if let Some(tool) = &self.tool_edit {
            for (label, field, value) in [
                (
                    "Description",
                    tool_presets::ToolDetail::Description,
                    &tool.description,
                ),
                (
                    "Manufacturer",
                    tool_presets::ToolDetail::Manufacturer,
                    &tool.manufacturer,
                ),
                (
                    "Product ID",
                    tool_presets::ToolDetail::ProductId,
                    &tool.product_id,
                ),
                (
                    "Product link",
                    tool_presets::ToolDetail::ProductLink,
                    &tool.product_link,
                ),
            ] {
                list = list.push(
                    row![
                        text(label).size(12).width(Length::Fill),
                        inspector_input("", value)
                            .on_input(move |value| Message::ToolDetailChanged(field, value))
                            .width(177)
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            }
        }

        for field in self.inspector_fields() {
            let raw = self
                .fields
                .get(&field)
                .map(String::as_str)
                .unwrap_or_default();
            let value = self.settings.format_numeric_input(
                raw,
                self.edited_tool_input == Some(field) || field == Field::Flutes,
            );
            list = list.push(self.parameter_field_row(
                field,
                self.field_label(field),
                self.field_help(field),
                &value,
            ));
        }
        if let Some(tool) = &self.tool_edit {
            list = list.push(
                row![
                    text("Cooling").size(12).width(Length::Fill),
                    pick_list(
                        &cam_model::Cooling::ALL[..],
                        Some(tool.cooling),
                        Message::ToolCoolingChanged
                    )
                    .text_size(12)
                    .padding([7, 9])
                    .style(inspector_picker)
                    .width(177)
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
        }
        if let Some(t) = self.preview_tool() {
            // Read from the working copy so the pickers (cutting direction, Type, thread
            // form) track live edits, matching the fields and viewport.
            // Cutting direction (a picker, not a numeric field) applies to the end-mill
            // family only; "Straight flute" is offered for flat end mills.
            let is_end_mill = matches!(
                t.kind,
                ToolKind::EndMill
                    | ToolKind::RoughingMill
                    | ToolKind::BallMill
                    | ToolKind::BullNose { .. }
            );
            if is_end_mill {
                let dir_opts = if matches!(t.kind, ToolKind::EndMill | ToolKind::RoughingMill) {
                    vec![CutDir::Down, CutDir::Up, CutDir::Straight]
                } else {
                    vec![CutDir::Down, CutDir::Up]
                };
                list = list.push(
                    row![
                        help_wrap(
                            text("Cutting direction").width(Length::Fill).size(13),
                            "Down-cut vs up-cut (helix direction); a physical property of the \
                             tool, like the flute count. Flat end mills also allow a straight \
                             (axial) flute.",
                            self.tooltips,
                        ),
                        pick_list(
                            dir_opts,
                            Some(t.cutting_direction),
                            Message::ToolCuttingDirChanged
                        )
                        .text_size(12)
                        .padding([7, 9])
                        .style(inspector_picker)
                        .width(Length::Fixed(113.0)),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            }
            // Thread form (single-point vs full-form) — a thread-mill-only toggle.
            if let ToolKind::ThreadMill { pitch } = t.kind {
                let form = if pitch.is_some() {
                    ThreadForm::FullForm
                } else {
                    ThreadForm::SinglePoint
                };
                list = list.push(
                    row![
                        help_wrap(
                            text("Thread form").width(Length::Fill).size(13),
                            "Single-point mills carry one tooth and cut any pitch by their \
                             helical lead; full-form mills carry a stack of teeth at a fixed \
                             ground pitch.",
                            self.tooltips,
                        ),
                        pick_list(&ThreadForm::ALL[..], Some(form), Message::ThreadFormChanged)
                            .text_size(12)
                            .padding([7, 9])
                            .style(inspector_picker)
                            .width(Length::Fixed(113.0)),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            }
        }

        scrollable(list)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    fn cad_inspector(&self) -> Element<'_, Message> {
        let selected = self.controller.cad_selection();
        let mut list = column![
            inspector_heading("CAD geometry selection"),
            text("Click selects; Ctrl-click selects the XY silhouette. Shift-click adds/removes. Drag to orbit. Escape cancels.")
                .size(12)
        ]
        .spacing(10)
        .padding(14);
        list = list.push(self.geometry_list(cad_geometry_labels(selected)));
        for entity in selected {
            list = list.push(
                text(format!(
                    "{:?} {} · body {} · instance {} · revision {}",
                    entity.kind, entity.id, entity.body_id, entity.instance_id, entity.revision
                ))
                .size(12),
            );
            if let Some(asset) = self.controller.geometry_state().asset(entity.asset_id) {
                if entity.kind == EntityKind::Silhouette {
                    list = list
                        .push(text("Whole-body outline projected onto setup XY (Z = 0).").size(12));
                    if let Some(projection) = &asset.topology.projection {
                        list = list.push(
                            text(format!(
                                "Projection deflection {:.4} mm; outward allowance {:.4} mm.",
                                projection.chord_tolerance,
                                projection.chord_tolerance + cam_geo::GRID_MM * 2.
                            ))
                            .size(11),
                        );
                    } else {
                        list = list
                            .push(text("Reimport the CAD file to enable its silhouette.").size(12));
                    }
                }
                let radius = match entity.kind {
                    EntityKind::Edge => asset
                        .topology
                        .edges
                        .iter()
                        .find(|e| e.id == entity.id)
                        .and_then(|e| match e.curve {
                            cam_model::CadCurve::CircleArc { radius, .. } => Some(radius),
                            _ => None,
                        }),
                    EntityKind::Face => asset
                        .topology
                        .faces
                        .iter()
                        .find(|f| f.id == entity.id)
                        .and_then(|f| match f.surface {
                            cam_model::CadSurface::Cylinder { radius, .. } => Some(radius),
                            cam_model::CadSurface::Torus { minor_radius, .. } => Some(minor_radius),
                            _ => None,
                        }),
                    _ => None,
                };
                if let Some(radius) = radius {
                    list = list.push(text(format!("Radius {radius:.3} mm")).size(12));
                }
                for diagnostic in &asset.import_diagnostics {
                    list = list.push(text(diagnostic).size(11));
                }
                if let Some(body) = asset
                    .topology
                    .bodies
                    .iter()
                    .find(|b| b.id == entity.body_id)
                {
                    list = list.push(
                        text(format!(
                            "{} · {:?} → mm · {}\nBody size {:.3} × {:.3} × {:.3} mm",
                            asset.name,
                            asset.source_units,
                            asset.backend,
                            body.bounds[1][0] - body.bounds[0][0],
                            body.bounds[1][1] - body.bounds[0][1],
                            body.bounds[1][2] - body.bounds[0][2]
                        ))
                        .size(11),
                    );
                }
            }
        }
        if self.cad_candidates.len() > 1 {
            list = list.push(text("Overlapping candidates").size(13));
            for candidate in &self.cad_candidates {
                list = list.push(
                    button(
                        text(format!(
                            "{:?} {} · instance {}{}",
                            candidate.entity.kind,
                            candidate.entity.id,
                            candidate.entity.instance_id,
                            if candidate.occluded {
                                " (behind solid)"
                            } else {
                                ""
                            }
                        ))
                        .size(12),
                    )
                    .on_press(Message::CadSelect(candidate.entity.clone())),
                );
            }
        }
        if let Some(id) = self.cad_editing_op {
            list = list.push(text(format!("Repair geometry for operation {id}; its tool and manual overrides are preserved.")).size(12));
            let preview = self.preview_geometry_draft(id, selected.to_vec());
            list = list.push(
                inspector_button("Use geometry")
                    .style(inspector_primary)
                    .on_press_maybe(preview.is_ok().then_some(Message::ConfirmCadOperation)),
            );
            if let Err(reason) = preview {
                list = list.push(
                    text(reason)
                        .size(12)
                        .color(Color::from_rgb(0.95, 0.55, 0.45)),
                );
            }
        } else {
            list = list.push(text("Create operation from selection").size(13));
            list = list.push(
                row![
                    inspector_button("Pocket").on_press(Message::BeginOp(OpKind::Pocket)),
                    inspector_button("Drill").on_press(Message::BeginOp(OpKind::Drill))
                ]
                .spacing(4),
            );
            list = list.push(
                row![
                    inspector_button("Profile").on_press(Message::BeginOp(OpKind::Profile)),
                    inspector_button("Face").on_press(Message::BeginOp(OpKind::Face))
                ]
                .spacing(4),
            );
        }
        let assets: BTreeSet<_> = selected.iter().map(|r| r.asset_id).collect();
        if assets.len() == 1 {
            let id = *assets.iter().next().unwrap();
            list = list.push(
                row![
                    inspector_button("Replace model…").on_press(Message::ReplaceCadAsset(id)),
                    inspector_button("Remove model").on_press(Message::RemoveCadAsset(id))
                ]
                .spacing(4),
            );
        }
        list = list.push(inspector_button("Cancel selection").on_press(Message::CadEscape));
        scrollable(list).into()
    }
}

/// The `a` and `b` subtrees of the split with the given id, if found.
fn find_split(
    node: &pane_grid::Node,
    split: pane_grid::Split,
) -> Option<(&pane_grid::Node, &pane_grid::Node)> {
    match node {
        pane_grid::Node::Split { id, a, b, .. } => {
            if *id == split {
                Some((a, b))
            } else {
                find_split(a, split).or_else(|| find_split(b, split))
            }
        }
        pane_grid::Node::Pane(_) => None,
    }
}

/// The splits on the path from the root to `pane`, outermost first. Each is
/// paired with whether `pane` lies in that split's `a` subtree (`true`) or its
/// `b` subtree (`false`). `None` if the pane isn't in the tree.
fn splits_to_pane(
    node: &pane_grid::Node,
    pane: pane_grid::Pane,
) -> Option<Vec<(pane_grid::Split, bool)>> {
    match node {
        pane_grid::Node::Pane(p) => (*p == pane).then(Vec::new),
        pane_grid::Node::Split { id, a, b, .. } => {
            if let Some(mut rest) = splits_to_pane(a, pane) {
                rest.insert(0, (*id, true));
                Some(rest)
            } else if let Some(mut rest) = splits_to_pane(b, pane) {
                rest.insert(0, (*id, false));
                Some(rest)
            } else {
                None
            }
        }
    }
}

/// The minimum size (px) a subtree needs along `axis`, **honouring the current
/// internal split ratios**. This matters because dragging one boundary only
/// changes that one split's ratio — every descendant split keeps its ratio — so
/// the space handed to a subtree is apportioned by fixed fractions, not freely.
/// A child split along `axis` with ratio `r` gives region `a` about `r·(W−gap)`,
/// so to keep both children above their mins the subtree needs
/// `gap + max(min_a/r, min_b/(1−r))`; perpendicular splits take the larger child.
/// (A plain sum-of-mins would under-count and let one pane starve while its
/// sibling stays wide.)
fn subtree_min(
    node: &pane_grid::Node,
    panes: &pane_grid::State<Pane>,
    axis: pane_grid::Axis,
    mins: &crate::PanePrefs,
) -> f32 {
    match node {
        pane_grid::Node::Pane(p) => panes.get(*p).map_or(0.0, |pane| pane.min_size(mins)),
        pane_grid::Node::Split {
            axis: sub_axis,
            a,
            b,
            ratio,
            ..
        } => {
            let (ma, mb) = (
                subtree_min(a, panes, axis, mins),
                subtree_min(b, panes, axis, mins),
            );
            if *sub_axis == axis {
                let r = ratio.clamp(0.05, 0.95);
                PANE_SPACING + (ma / r).max(mb / (1.0 - r))
            } else {
                ma.max(mb)
            }
        }
    }
}

/// A divider occupying the existing gap between ribbon groups.
fn ribbon_separator() -> Element<'static, Message> {
    container(
        container(Space::new().width(1).height(58)).style(|_theme| container::Style {
            background: Some(Background::Color(palette::SEPARATOR)),
            ..container::Style::default()
        }),
    )
    .width(Length::Fixed(GROUP_GAP))
    .center_x(Length::Fixed(GROUP_GAP))
    .into()
}

/// A labelled ribbon group: its command widgets stacked over a small caption in
/// muted grey (OpenCADStudio places the group label at the bottom).
fn ribbon_group<'a>(
    title: &'a str,
    content: impl Into<Element<'a, Message>>,
) -> Element<'a, Message> {
    let caption = text(title).size(10).color(palette::RIBBON_CAPTION);
    container(
        column![content.into(), caption]
            .spacing(4)
            .align_x(Alignment::Center),
    )
        .padding(Padding::from([3.0, 4.0]))
        .into()
}

/// Fixed width of a large (Full-density) ribbon command button, px.
const COMMAND_W: f32 = 64.0;

/// Tab-strip button: transparent normally, accent-filled when active, a subtle
/// fill on hover.
fn tab_button_style(active: bool, status: button::Status) -> button::Style {
    let background = if active {
        Some(Background::Color(palette::ACCENT_BLUE))
    } else if matches!(status, button::Status::Hovered | button::Status::Pressed) {
        Some(Background::Color(palette::TOOL_HOVER))
    } else {
        None
    };
    button::Style {
        background,
        text_color: if active {
            Color::WHITE
        } else {
            palette::RIBBON_TEXT
        },
        border: Border {
            radius: 3.0.into(),
            ..Border::default()
        },
        ..button::Style::default()
    }
}

/// Object-snap toggle: filled accent when enabled, outlined when off.
fn snap_toggle_style(on: bool, status: button::Status) -> button::Style {
    let hovered = matches!(status, button::Status::Hovered | button::Status::Pressed);
    let background = if on {
        Some(Background::Color(palette::ACCENT_BLUE))
    } else if hovered {
        Some(Background::Color(palette::TOOL_HOVER))
    } else {
        None
    };
    button::Style {
        background,
        text_color: if on {
            Color::WHITE
        } else {
            palette::RIBBON_TEXT
        },
        border: Border {
            color: palette::SEPARATOR,
            width: 1.0,
            radius: 3.0.into(),
        },
        ..button::Style::default()
    }
}

/// Ribbon command button: flat, muted-grey when disabled, a row-hover fill
/// otherwise.
fn command_button_style(status: button::Status) -> button::Style {
    let background = match status {
        button::Status::Hovered | button::Status::Pressed => {
            Some(Background::Color(palette::ROW_HOVER))
        }
        _ => None,
    };
    let text_color = if matches!(status, button::Status::Disabled) {
        palette::RIBBON_DISABLED
    } else {
        palette::RIBBON_TEXT
    };
    button::Style {
        background,
        text_color,
        border: Border {
            radius: 3.0.into(),
            ..Border::default()
        },
        ..button::Style::default()
    }
}

/// Ribbon toggle button: accent-tinted while `on`, otherwise a normal command
/// button.
fn command_toggle_style(on: bool, status: button::Status) -> button::Style {
    if !on {
        return command_button_style(status);
    }
    let mut accent = palette::ACCENT_BLUE;
    // A muted accent fill so the icon/label stay readable; brighter on hover.
    accent.a = if matches!(status, button::Status::Hovered | button::Status::Pressed) {
        0.55
    } else {
        0.38
    };
    button::Style {
        background: Some(Background::Color(accent)),
        text_color: palette::RIBBON_TEXT,
        border: Border {
            radius: 3.0.into(),
            ..Border::default()
        },
        ..button::Style::default()
    }
}

// --- Responsive ribbon collapse ------------------------------------------------
//
// As the window narrows, each group degrades through four densities so the ribbon
// never overflows — modelled on OpenCADStudio's ribbon. The layout maths is a pure
// function of the per-group command counts and the available width, so the
// degradation order is unit-tested without any GUI.

/// How densely a ribbon group is drawn. Ordered loosest → tightest.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Density {
    /// Large icon-over-label buttons in a row.
    Full,
    /// Icon-only buttons in a row (labels dropped).
    Compact,
    /// One representative button (icon + caption) that opens a popup of the group.
    Collapsed,
    /// One representative button (icon only) that opens a popup.
    Tight,
}

impl Density {
    /// A collapsed/tight group is a single popup-opening button, not its commands.
    fn is_popup(self) -> bool {
        matches!(self, Density::Collapsed | Density::Tight)
    }
}

const CMD_GAP: f32 = 2.0; // between command buttons within a group
const GROUP_GAP: f32 = 8.0; // between groups
const GROUP_PAD: f32 = 4.0; // per side, inside a group container
const COMPACT_W: f32 = 34.0; // icon-only command button
const COLLAPSED_W: f32 = 64.0; // single group button, with caption
const TIGHT_W: f32 = 34.0; // single group button, icon only
/// Ribbon chrome outside the group row (band padding + a little slack).
const RIBBON_CHROME: f32 = 28.0;
/// Approximate full ribbon height (tab strip + band), for dropping popups just
/// below it. Eyeballed; a few px off only shifts the popup slightly.
const RIBBON_H: f32 = 96.0;

/// The drawn width of a group of `n` commands at a given density.
fn group_width(n: usize, density: Density) -> f32 {
    let (btn_w, buttons) = match density {
        Density::Full => (COMMAND_W, n),
        Density::Compact => (COMPACT_W, n),
        Density::Collapsed => (COLLAPSED_W, 1),
        Density::Tight => (TIGHT_W, 1),
    };
    let inner = buttons as f32 * btn_w + buttons.saturating_sub(1) as f32 * CMD_GAP;
    // Full/Compact groups wrap in `ribbon_group`, a container padded `GROUP_PAD`
    // per side; Collapsed/Tight render as a bare fixed-width button with no such
    // wrapper. Counting the padding for the popup densities made this width — and
    // thus the popup x-offset that sums it over preceding groups — drift right by
    // `2*GROUP_PAD` per collapsed group.
    if density.is_popup() {
        inner
    } else {
        inner + GROUP_PAD * 2.0
    }
}

/// The total drawn width of the group row at the given densities (with gaps).
fn row_width(counts: &[usize], densities: &[Density]) -> f32 {
    let groups: f32 = counts
        .iter()
        .zip(densities)
        .map(|(&n, &d)| group_width(n, d))
        .sum();
    let gaps = counts.len().saturating_sub(1) as f32 * GROUP_GAP;
    groups + gaps
}

/// Assign each group a density so the row fits `available` px. Groups degrade
/// **right-to-left**, one density level at a time (all groups drop to Compact from
/// the right before any drop to Collapsed, etc.) — OpenCADStudio's behaviour. If
/// everything is Tight and it still overflows, the row is left tight (iced clips).
fn solve_densities(counts: &[usize], available: f32) -> Vec<Density> {
    let mut densities = vec![Density::Full; counts.len()];
    for level in [Density::Compact, Density::Collapsed, Density::Tight] {
        if row_width(counts, &densities) <= available {
            break;
        }
        for i in (0..densities.len()).rev() {
            if densities[i] < level {
                densities[i] = level;
            }
            if row_width(counts, &densities) <= available {
                break;
            }
        }
    }
    densities
}

// Stored as portable Cmd combinations: Cmd means ⌘ on macOS and Ctrl elsewhere.
const SHORTCUTS: &[(&str, &str, &str)] = &[
    ("new", "New project", "Cmd+N"),
    ("open", "Open project", "Cmd+O"),
    ("save", "Save project", "Cmd+S"),
    ("save_as", "Save as", "Cmd+Shift+S"),
    ("undo", "Undo", "Cmd+Z"),
    ("redo", "Redo", "Cmd+Shift+Z"),
    ("face", "Face", "Cmd+Shift+F"),
    ("profile", "Profile", "Cmd+Shift+P"),
    ("pocket", "Pocket", "Cmd+Shift+K"),
    ("drill", "Drill", "Cmd+Shift+D"),
    ("thread", "Thread", "Cmd+Shift+T"),
    ("bore", "Bore", "Cmd+Shift+B"),
    ("chamfer", "Chamfer", "Cmd+Shift+C"),
    ("engrave", "Engrave", "Cmd+Shift+E"),
    ("carve", "Carve", "Cmd+Shift+V"),
    ("adaptive", "3D Adaptive Clearing", "Cmd+Shift+A"),
    ("parallel", "Parallel", "Cmd+Shift+L"),
    ("spiral", "3D Spiral", "Cmd+Shift+I"),
];

fn shortcut_value<'a>(settings: &'a crate::Settings, id: &str) -> &'a str {
    settings
        .shortcuts
        .get(id)
        .map(String::as_str)
        .unwrap_or_else(|| {
            SHORTCUTS
                .iter()
                .find(|(name, _, _)| *name == id)
                .map_or("", |(_, _, default)| *default)
        })
}

fn shortcut_binding(key: Option<char>, modifiers: iced::keyboard::Modifiers) -> Option<String> {
    use iced::keyboard::Modifiers;
    if modifiers != Modifiers::COMMAND && modifiers != (Modifiers::COMMAND | Modifiers::SHIFT) {
        return None;
    }
    let key = key?.to_ascii_uppercase();
    if !key.is_ascii_alphanumeric() {
        return None;
    }
    Some(format!(
        "Cmd+{}{key}",
        if modifiers.shift() { "Shift+" } else { "" }
    ))
}

fn shortcut_for_binding(settings: &crate::Settings, binding: &str) -> Option<&'static str> {
    SHORTCUTS
        .iter()
        .find(|(id, _, _)| shortcut_value(settings, id) == binding)
        .map(|(id, _, _)| *id)
        .or_else(|| {
            (!cfg!(target_os = "macos")
                && binding == "Cmd+Y"
                && !settings.shortcuts.contains_key("redo"))
            .then_some("redo")
        })
}

fn shortcut_action(id: &str) -> Option<Message> {
    Some(match id {
        "new" => Message::NewProject,
        "open" => Message::OpenProject,
        "save" => Message::SaveProject,
        "save_as" => Message::SaveProjectAs,
        "undo" => Message::Undo,
        "redo" => Message::Redo,
        "face" => Message::BeginOp(OpKind::Face),
        "profile" => Message::BeginOp(OpKind::Profile),
        "pocket" => Message::BeginOp(OpKind::Pocket),
        "drill" => Message::BeginOp(OpKind::Drill),
        "thread" => Message::BeginOp(OpKind::Thread),
        "bore" => Message::BeginOp(OpKind::Bore),
        "chamfer" => Message::BeginOp(OpKind::Chamfer),
        "engrave" => Message::BeginOp(OpKind::Engrave),
        "carve" => Message::BeginOp(OpKind::Carve),
        "adaptive" => Message::BeginOp(OpKind::Adaptive),
        "parallel" => Message::BeginOp(OpKind::Parallel),
        "spiral" => Message::BeginOp(OpKind::Spiral),
        _ => return None,
    })
}

fn display_shortcut(binding: &str) -> String {
    if binding.is_empty() {
        return "Unassigned".into();
    }
    binding.replace(
        "Cmd",
        if cfg!(target_os = "macos") {
            "⌘"
        } else {
            "Ctrl"
        },
    )
}

/// One ribbon command in a group's data model — enough to render it at any
/// density (icon+label, icon-only, or inside a popup).
struct Command {
    icon: Icon,
    label: &'static str,
    /// `None` disables the button (greyed, unclickable).
    action: Option<Message>,
    /// `Some(on)` renders a toggle (accent-tinted while on); `None` is a plain action.
    toggle: Option<bool>,
    /// Tooltip for *this* command, when the icon's own words do not fit.
    ///
    /// Help used to be keyed by icon alone, which is right while an icon means one
    /// thing. The moment one is reused — the tool icon standing in for "add a machine" —
    /// it silently describes the wrong action, which is worse than no tooltip.
    help: Option<String>,
    /// Kept separately from the action so disabled operations still explain themselves.
    operation_hint: Option<(OpKind, String)>,
}

/// A ribbon group's data model.
struct GroupSpec {
    title: &'static str,
    commands: Vec<Command>,
}

impl GroupSpec {
    /// The icon shown when the group collapses to a single button.
    fn rep_icon(&self) -> Icon {
        self.commands.first().map_or(Icon::Run, |c| c.icon)
    }
}

fn cmd(icon: Icon, label: &'static str, action: Option<Message>) -> Command {
    Command {
        icon,
        label,
        action,
        toggle: None,
        help: None,
        operation_hint: None,
    }
}

/// A command whose tooltip is its own, because it borrows an icon that means something
/// else elsewhere.
///
/// Help keyed by icon alone silently describes the wrong action the moment an icon is
/// reused — which is how "Add machine" came to read "Add a new tool to the library".
fn cmd_help(
    icon: Icon,
    label: &'static str,
    action: Option<Message>,
    help: impl Into<String>,
) -> Command {
    Command {
        help: Some(help.into()),
        ..cmd(icon, label, action)
    }
}

fn toggle_cmd(icon: Icon, label: &'static str, on: bool, msg: Message) -> Command {
    Command {
        icon,
        label,
        action: Some(msg),
        help: None,
        operation_hint: None,
        toggle: Some(on),
    }
}

/// Render a single command at Full (`compact = false`, icon over label) or Compact
/// (`compact = true`, icon only) density. When `show`, the button carries a hover
/// tooltip describing the command — the icon's only self-explanation at Compact.
fn render_command(command: &Command, compact: bool, show: bool) -> Element<'static, Message> {
    let (icon_px, width) = if compact {
        (22.0, COMPACT_W)
    } else {
        (26.0, COMMAND_W)
    };
    let content: Element<'static, Message> = if compact {
        icon_svg(command.icon, icon_px)
    } else {
        column![
            icon_svg(command.icon, icon_px),
            text(command.label).size(10)
        ]
        .spacing(3)
        .align_x(Alignment::Center)
        .into()
    };
    let base = button(container(content).center_x(Length::Fill))
        .width(Length::Fixed(width))
        .padding(Padding::from([4.0, 2.0]));
    let btn: Element<'static, Message> = match command.toggle {
        Some(on) => base
            .on_press_maybe(command.action.clone())
            .style(move |_theme, status| command_toggle_style(on, status))
            .into(),
        None => base
            .on_press_maybe(command.action.clone())
            .style(|_theme, status| command_button_style(status))
            .into(),
    };
    if !show {
        return btn;
    }
    if let Some((kind, shortcut)) = &command.operation_hint {
        return delayed_tooltip(
            btn,
            operation_hints::content(*kind, shortcut),
            tooltip::Position::Bottom,
        )
        .padding(0)
        .gap(5)
        .into();
    }
    delayed_tooltip(
        btn,
        container(
            text(
                command
                    .help
                    .clone()
                    .unwrap_or_else(|| command.icon.help().to_string()),
            )
            .size(12),
        )
        .padding(8)
        .max_width(300.0)
        .style(container::rounded_box),
        tooltip::Position::Bottom,
    )
    .into()
}

/// Render a whole group at its assigned density. Collapsed/Tight groups become a
/// single button that toggles the group's popup (`index` in the active tab).
fn render_group(
    spec: &GroupSpec,
    density: Density,
    index: usize,
    show: bool,
) -> Element<'static, Message> {
    if density.is_popup() {
        let (icon_px, width) = if density == Density::Tight {
            (22.0, TIGHT_W)
        } else {
            (26.0, COLLAPSED_W)
        };
        let content: Element<'static, Message> = if density == Density::Collapsed {
            column![
                icon_svg(spec.rep_icon(), icon_px),
                text(spec.title).size(10)
            ]
            .spacing(3)
            .align_x(Alignment::Center)
            .into()
        } else {
            icon_svg(spec.rep_icon(), icon_px)
        };
        return button(container(content).center_x(Length::Fill))
            .width(Length::Fixed(width))
            .padding(Padding::from([4.0, 2.0]))
            .on_press(Message::ToggleRibbonGroup(index))
            .style(|_theme, status| command_button_style(status))
            .into();
    }
    let compact = density == Density::Compact;
    let mut commands = row![].spacing(CMD_GAP);
    for command in &spec.commands {
        commands = commands.push(render_command(command, compact, show));
    }
    ribbon_group(spec.title, commands)
}

/// A short label for an operation's kind.
fn op_kind(op: &Operation) -> &'static str {
    match op {
        Operation::Profile(_) => "Profile",
        Operation::Drill(_) => "Drill",
        Operation::Pocket(_) => "Pocket",
        Operation::Face(_) => "Face",
        Operation::Chamfer(_) => "Chamfer",
        Operation::Bore(_) => "Bore",
        Operation::Thread(_) => "Thread",
        Operation::Engrave(_) => "Engrave",
        Operation::Carve(_) => "Carve",
        Operation::Adaptive(_) => "3D Adaptive Clearing",
        Operation::Parallel(o) if o.params.scallop => "3D Scallop",
        Operation::Parallel(_) => "3D Parallel",
        Operation::Spiral(_) => "3D Spiral",
    }
}

/// Read an operation's value for a given field, if the op has it.
fn op_field(op: &Operation, field: Field) -> Option<f64> {
    if let Field::Wrap(number) = field {
        return op.rotary_wrap().map(|w| number.get(w));
    }
    match (op, field) {
        (Operation::Pocket(o), Field::LiftHeight) => Some(o.clear.lift_height),
        (Operation::Adaptive(o), Field::LeadInSize) => Some(o.params.lead_in_radius),
        (Operation::Adaptive(o), Field::LeadOutSize) => Some(o.params.lead_out_radius),
        (Operation::Adaptive(o), Field::LiftHeight) => Some(o.params.lift_height),
        (op, Field::StartX) => operation_tabs::operation_start(op).map(|point| point[0]),
        (op, Field::StartY) => operation_tabs::operation_start(op).map(|point| point[1]),
        (Operation::Engrave(op), Field::OperationTop) => Some(op.top),
        (Operation::Carve(op), Field::OperationTop) => Some(op.top),
        // Spindle speed is common to every operation kind.
        (op, Field::SpindleRpm) => Some(op.spindle_rpm()),
        (op, Field::RampFeed) => Some(op.entry_feeds().ramp.unwrap_or(0.)),
        (op, Field::LeadInFeed) => Some(op.entry_feeds().lead_in.unwrap_or(0.)),
        (op, Field::LeadOutFeed) => Some(op.entry_feeds().lead_out.unwrap_or(0.)),
        (Operation::Profile(o), Field::ContourRoughStepover) => {
            Some(o.passes.contour_roughing.stepover)
        }
        (Operation::Profile(o), Field::ContourRoughSmoothing) => {
            Some(o.passes.contour_roughing.smoothing_deviation)
        }
        (Operation::Chamfer(o), Field::ContourRoughSmoothing) => {
            Some(o.contour_roughing.smoothing_deviation)
        }
        (Operation::Profile(o), Field::ContourRoughCount) => {
            Some(o.passes.contour_roughing.stepovers as f64)
        }
        (Operation::Chamfer(o), Field::ContourRoughStepover) => Some(o.contour_roughing.stepover),
        (Operation::Chamfer(o), Field::ContourRoughCount) => {
            Some(o.contour_roughing.stepovers as f64)
        }
        (Operation::Profile(o), Field::Depth) => Some(o.depth),
        (Operation::Profile(o), Field::Profile(n)) => Some(n.get(o)),
        (Operation::Chamfer(o), Field::Profile(profile_parameters::Number::LeadInSweep)) => {
            Some(match o.lead_in {
                Lead::ArcSweep { angle_deg, .. } => angle_deg,
                _ => 90.,
            })
        }
        (Operation::Chamfer(o), Field::Profile(profile_parameters::Number::LeadOutSweep)) => {
            Some(match o.lead_out {
                Lead::ArcSweep { angle_deg, .. } => angle_deg,
                _ => 90.,
            })
        }
        (Operation::Profile(o), Field::Stepover) => Some(o.stepover),
        (Operation::Profile(o), Field::TabWidth) => Some(o.tabs.width),
        (Operation::Profile(o), Field::TabHeight) => Some(o.tabs.height),
        (Operation::Profile(o), Field::TabSpacing) => Some(o.tabs.spacing),
        (Operation::Profile(o), Field::ProfileOffset) => Some(o.offset),
        (Operation::Profile(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Profile(o), Field::Feed) => Some(o.feed),
        (Operation::Profile(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Profile(o), Field::LeadInSize) => Some(lead_size(o.lead_in)),
        (Operation::Profile(o), Field::LeadOutSize) => Some(lead_size(o.lead_out)),
        (Operation::Profile(o), Field::LeadOverlap) => Some(o.lead_overlap),
        (Operation::Carve(o), Field::Depth) => Some(o.depth),
        (Operation::Carve(o), Field::ProfileOffset) => Some(o.offset),
        (Operation::Carve(o), Field::RingStep) => Some(o.ring_step),
        (Operation::Carve(o), Field::Scallop) => Some(o.scallop),
        (Operation::Carve(o), Field::Feed) => Some(o.feed),
        (Operation::Carve(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Carve(o), Field::PlungeA) => Some(plunge_params(o.plunge).0),
        (Operation::Carve(o), Field::PlungeB) => Some(plunge_params(o.plunge).1),
        (Operation::Carve(o), Field::ClearStepdown) => Some(o.clear?.params.stepdown),
        (Operation::Carve(o), Field::ClearOverlap) => Some(o.clear?.params.overlap * 100.0),
        (Operation::Carve(o), Field::ClearOffset) => Some(o.clear?.params.offset),
        (Operation::Carve(o), Field::ClearEngagement) => Some(o.clear?.params.clearing.engagement),
        (Operation::Carve(o), Field::ClearFeed) => Some(o.clear?.params.feed),
        (Operation::Carve(o), Field::ClearPlungeFeed) => Some(o.clear?.params.plunge_feed),
        (Operation::Carve(o), Field::ClearLeadOverlap) => Some(o.clear?.params.lead_overlap),
        (Operation::Carve(o), Field::ClearLeadInSize) => Some(lead_size(o.clear?.params.lead_in)),
        (Operation::Carve(o), Field::ClearLeadOutSize) => Some(lead_size(o.clear?.params.lead_out)),
        (Operation::Carve(o), Field::ClearPlungeA) => Some(plunge_params(o.clear?.params.plunge).0),
        (Operation::Carve(o), Field::ClearPlungeB) => Some(plunge_params(o.clear?.params.plunge).1),
        (Operation::Profile(o), Field::Engagement) => Some(o.clearing.engagement),
        (Operation::Profile(o), Field::PlungeA) => Some(plunge_params(o.plunge).0),
        (Operation::Profile(o), Field::PlungeB) => Some(plunge_params(o.plunge).1),
        (Operation::Pocket(o), Field::PocketFinish(n)) => Some(n.get(&o.finishing)),
        (Operation::Pocket(o), Field::Depth) => Some(o.depth),
        (Operation::Pocket(o), Field::Stepdown) => Some(o.clear.stepdown),
        (Operation::Pocket(o), Field::Overlap) => Some(o.clear.overlap * 100.0),
        (Operation::Pocket(o), Field::PocketStepover) => Some(o.maximum_stepover.unwrap_or(0.0)),
        (Operation::Pocket(o), Field::ProfileOffset) => Some(o.clear.offset),
        (Operation::Pocket(o), Field::LeadInSize) => Some(lead_size(o.clear.lead_in)),
        (Operation::Pocket(o), Field::LeadOutSize) => Some(lead_size(o.clear.lead_out)),
        (Operation::Pocket(o), Field::Feed) => Some(o.clear.feed),
        (Operation::Pocket(o), Field::PlungeFeed) => Some(o.clear.plunge_feed),
        (Operation::Pocket(o), Field::LeadOverlap) => Some(o.clear.lead_overlap),
        (Operation::Pocket(o), Field::Engagement) => Some(o.clear.clearing.engagement),
        (Operation::Pocket(o), Field::PlungeA) => Some(plunge_params(o.clear.plunge).0),
        (Operation::Pocket(o), Field::PlungeB) => Some(plunge_params(o.clear.plunge).1),
        (Operation::Face(o), Field::FaceStartOffset) => Some(o.start_offset),
        (Operation::Face(o), Field::Depth) => Some(o.depth),
        (Operation::Face(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Face(o), Field::Overlap) => Some(o.overlap * 100.0),
        (Operation::Face(o), Field::FaceOverlap) => Some(o.absolute_overlap.unwrap_or(0.0)),
        (Operation::Face(o), Field::FaceOvershoot) => Some(o.overshoot),
        (Operation::Face(o), Field::Feed) => Some(o.feed),
        (Operation::Face(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Drill(o), Field::Depth) => Some(o.depth),
        (Operation::Drill(o), Field::DrillStartOffset) => Some(o.start_offset),
        (Operation::Drill(o), Field::Peck) => Some(o.peck.unwrap_or(0.0)),
        (Operation::Drill(o), Field::Dwell) => Some(o.dwell.unwrap_or(0.0)),
        (Operation::Drill(o), Field::Feed) => Some(o.feed),
        (Operation::Engrave(o), Field::Depth) => Some(o.depth),
        (Operation::Engrave(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Engrave(o), Field::Feed) => Some(o.feed),
        (Operation::Engrave(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Chamfer(o), Field::ChamferTop) => Some(o.top),
        (Operation::Chamfer(o), Field::ChamferWidth) => Some(o.width),
        (Operation::Chamfer(o), Field::ChamferDepth) => Some(o.depth),
        (Operation::Chamfer(o), Field::ChamferTipOffset) => Some(o.tip_offset),
        (Operation::Chamfer(o), Field::ChamferStep) => Some(o.step),
        (Operation::Chamfer(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Chamfer(o), Field::Feed) => Some(o.feed),
        (Operation::Chamfer(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Chamfer(o), Field::LeadInSize) => Some(lead_size(o.lead_in)),
        (Operation::Chamfer(o), Field::LeadOutSize) => Some(lead_size(o.lead_out)),
        (Operation::Chamfer(o), Field::LeadOverlap) => Some(o.lead_overlap),
        (Operation::Bore(o), Field::BoreDiameter) => Some(o.diameter),
        (Operation::Bore(o), Field::BoreTaper) => Some(o.taper_angle),
        (Operation::Bore(o), Field::BorePitch) => Some(o.pitch),
        (Operation::Bore(o), Field::BoreAngle) => Some(o.ramp_angle.unwrap_or(3.)),
        (Operation::Bore(o), Field::BoreStepovers) => Some(o.stepovers as f64),
        (Operation::Bore(o), Field::BoreStepover) => Some(o.stepover),
        (Operation::Bore(o), Field::BoreFinishing) => Some(o.finishing_stepover),
        (Operation::Bore(o), Field::BoreRadialStock) => Some(o.radial_stock),
        (Operation::Bore(o), Field::BoreAxialStock) => Some(o.axial_stock),
        (Operation::Bore(o), Field::BoreTolerance) => Some(o.tolerance),
        (Operation::Bore(o), Field::BoreSafeDistance) => Some(o.safe_distance),
        (Operation::Bore(o), Field::BoreLeadRadius) => Some(o.lead_radius),
        (Operation::Bore(o), Field::Feed) => Some(o.feed),
        (Operation::Bore(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Thread(o), Field::MajorDia) => Some(o.major_dia),
        (Operation::Thread(o), Field::Pitch) => Some(o.pitch),
        (Operation::Thread(o), Field::ThreadTop) => Some(o.z_top),
        (Operation::Thread(o), Field::ThreadBottom) => Some(o.z_bottom),
        (Operation::Thread(o), Field::ThreadPasses) => Some(o.passes as f64),
        (Operation::Thread(o), Field::ThreadSpringPasses) => Some(o.spring_passes as f64),
        (Operation::Thread(o), Field::ThreadDrillClearance) => Some(o.drill_clearance),
        (Operation::Thread(o), Field::ThreadBlindAllowance) => Some(o.blind_allowance),
        (Operation::Thread(o), Field::Feed) => Some(o.feed),
        (Operation::Thread(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (op @ Operation::Spiral(_), field) => parameter_expressions::parameter(field)?.value(op),
        (Operation::Parallel(o), Field::ParallelStepover) => Some(o.params.stepover),
        (Operation::Parallel(o), Field::ParallelAngle) => Some(o.params.angle),
        (Operation::Parallel(o), Field::ScallopSafeDistance) => {
            Some(o.params.safe_distance)
        }
        (Operation::Parallel(o), Field::ScallopSmoothingTolerance) => {
            Some(o.params.scallop_smoothing_tolerance)
        }
        (Operation::Parallel(o), Field::AdaptiveTolerance) => Some(o.params.tolerance),
        (Operation::Parallel(o), Field::AdaptiveOffset) => Some(o.params.additional_offset),
        (Operation::Parallel(o), Field::Feed) => Some(o.params.feed),
        (Operation::Parallel(o), Field::PlungeFeed) => Some(o.params.plunge_feed),
        (Operation::Adaptive(o), Field::AdaptiveTolerance) => Some(o.params.tolerance),
        (Operation::Adaptive(o), Field::AdaptiveOptimalLoad) => Some(o.params.optimal_load),
        (Operation::Adaptive(o), Field::Stepdown) => Some(o.params.stepdown),
        (Operation::Adaptive(o), Field::AdaptiveFineStepdown) => Some(o.params.fine_stepdown),
        (Operation::Adaptive(o), Field::AdaptiveOffset) => Some(o.params.additional_offset),
        (Operation::Adaptive(o), Field::Feed) => Some(o.params.feed),
        (Operation::Adaptive(o), Field::PlungeFeed) => Some(o.params.plunge_feed),
        _ => None,
    }
}

/// Write the parsed inspector fields onto an operation.
fn apply_op_fields(op: &mut Operation, parsed: &BTreeMap<Field, f64>) {
    if let Some(mut wrap) = op.rotary_wrap().cloned() {
        for (field, value) in parsed {
            if let Field::Wrap(number) = field {
                if number.cylinder_dimension() && (number.get(&wrap) - value).abs() > 1e-9 {
                    wrap.cylinder = None;
                }
                number.set(&mut wrap, *value);
            }
        }
        op.set_rotary_wrap(Some(wrap));
    }
    let get = |f: Field| parsed.get(&f).copied();
    if let Some(value) = get(Field::LiftHeight) {
        match op {
            Operation::Pocket(o) => o.clear.lift_height = value,
            Operation::Adaptive(o) => o.params.lift_height = value,
            _ => {}
        }
    }
    if let Some(value) = get(Field::RampFeed) {
        op.entry_feeds_mut().ramp = (value > 0.).then_some(value);
    }
    if let Some(value) = get(Field::LeadInFeed) {
        op.entry_feeds_mut().lead_in = (value > 0.).then_some(value);
    }
    if let Some(value) = get(Field::LeadOutFeed) {
        op.entry_feeds_mut().lead_out = (value > 0.).then_some(value);
    }
    // Spindle speed is common to every operation kind.
    if let Some(v) = get(Field::SpindleRpm) {
        op.set_spindle_rpm(v.max(0.0));
    }
    if let Some(&value) = parsed.get(&Field::StartX) {
        operation_tabs::set_operation_start_coordinate(op, 0, value);
    }
    if let Some(&value) = parsed.get(&Field::StartY) {
        operation_tabs::set_operation_start_coordinate(op, 1, value);
    }
    if let Some(&value) = parsed.get(&Field::OperationTop) {
        match op {
            Operation::Engrave(op) => op.top = value,
            Operation::Carve(op) => op.top = value,
            _ => {}
        }
    }
    let rough = match op {
        Operation::Profile(o) => Some(&mut o.passes.contour_roughing),
        Operation::Chamfer(o) => Some(&mut o.contour_roughing),
        _ => None,
    };
    if let Some(rough) = rough {
        if let Some(v) = get(Field::ContourRoughStepover).filter(|v| v.is_finite() && *v > 0.) {
            rough.stepover = v;
        }
        if let Some(v) = get(Field::ContourRoughSmoothing).filter(|v| v.is_finite() && *v > 0.) {
            rough.smoothing_deviation = v;
        }
        if let Some(v) = get(Field::ContourRoughCount)
            .filter(|v| v.is_finite() && *v >= 1. && *v <= 1000. && v.fract() == 0.)
        {
            rough.stepovers = v as u32;
        }
    }
    match op {
        Operation::Profile(o) => {
            for (&field, &value) in parsed {
                if let Field::Profile(n) = field {
                    if !n.invalid(value) {
                        n.set(o, value);
                    }
                }
            }
            if let Some(v) = get(Field::TabWidth) {
                o.tabs.width = v;
            }
            if let Some(v) = get(Field::TabHeight) {
                o.tabs.height = v;
            }
            if let Some(v) = get(Field::TabSpacing) {
                o.tabs.spacing = v;
            }
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v;
            }
            if let Some(v) = get(Field::Stepover) {
                o.stepover = v.max(0.0);
            }
            if let Some(v) = get(Field::Engagement) {
                o.clearing.engagement = v.max(0.0);
            }
            if let Some(v) = get(Field::ProfileOffset) {
                o.offset = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
            if let Some(v) = get(Field::LeadInSize) {
                o.lead_in = set_lead_size(o.lead_in, v);
            }
            if let Some(v) = get(Field::LeadOutSize) {
                o.lead_out = set_lead_size(o.lead_out, v);
            }
            if let Some(v) = get(Field::LeadOverlap) {
                o.lead_overlap = v.max(0.0);
            }
            let (a, b) = plunge_params(o.plunge);
            let a = get(Field::PlungeA).unwrap_or(a);
            let b = get(Field::PlungeB).unwrap_or(b);
            o.plunge = set_plunge_params(o.plunge, a, b);
        }
        Operation::Pocket(o) => {
            for (&field, &value) in parsed {
                if let Field::PocketFinish(n) = field {
                    if !n.invalid(value) {
                        n.set(&mut o.finishing, value);
                    }
                }
            }
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.clear.stepdown = v;
            }
            if let Some(v) = get(Field::PocketStepover) {
                o.maximum_stepover = Some(v);
            }
            if let Some(v) = get(Field::Engagement) {
                o.clear.clearing.engagement = v.max(0.0);
            }
            if let Some(v) = get(Field::ProfileOffset) {
                o.clear.offset = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.clear.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.clear.plunge_feed = v;
            }
            if let Some(v) = get(Field::LeadOverlap) {
                o.clear.lead_overlap = v.max(0.0);
            }
            if let Some(v) = get(Field::LeadInSize) {
                o.clear.lead_in = set_lead_size(o.clear.lead_in, v);
            }
            if let Some(v) = get(Field::LeadOutSize) {
                o.clear.lead_out = set_lead_size(o.clear.lead_out, v);
            }
            let (a, b) = plunge_params(o.clear.plunge);
            let a = get(Field::PlungeA).unwrap_or(a);
            let b = get(Field::PlungeB).unwrap_or(b);
            o.clear.plunge = set_plunge_params(o.clear.plunge, a, b);
        }
        Operation::Face(o) => {
            if let Some(v) = get(Field::FaceStartOffset) {
                o.start_offset = v.max(0.0);
            }
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v;
            }
            if let Some(v) = get(Field::FaceOverlap) {
                o.absolute_overlap = Some(v);
            }
            if let Some(v) = get(Field::FaceOvershoot) {
                // Negative is allowed — it plunges into the stock (the strategy warns).
                o.overshoot = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
        }
        Operation::Drill(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::DrillStartOffset) {
                o.start_offset = v;
            }
            if let Some(v) = get(Field::Peck) {
                // 0 (or negative) clears pecking; the toolpath requires peck > 0.
                o.peck = (v > 0.0).then_some(v);
            }
            if let Some(v) = get(Field::Dwell) {
                o.dwell = (v > 0.0).then_some(v);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
        }
        Operation::Chamfer(o) => {
            for (field, lead) in [
                (
                    Field::Profile(profile_parameters::Number::LeadInSweep),
                    &mut o.lead_in,
                ),
                (
                    Field::Profile(profile_parameters::Number::LeadOutSweep),
                    &mut o.lead_out,
                ),
            ] {
                if let Some(angle_deg) =
                    get(field).filter(|v| v.is_finite() && *v > 0. && *v <= 170.)
                {
                    if let Lead::Arc { radius } | Lead::ArcSweep { radius, .. } = *lead {
                        *lead = Lead::ArcSweep { radius, angle_deg };
                    }
                }
            }
            // Deliberately unclamped: an edge below the part datum is an ordinary
            // thing to chamfer, so a negative Z is a value, not a mistake.
            if let Some(v) = get(Field::ChamferTop) {
                o.top = v;
            }
            if let Some(v) = get(Field::ChamferWidth) {
                o.width = v;
            }
            if let Some(v) = get(Field::ChamferDepth) {
                o.depth = v.max(0.0);
            }
            if let Some(v) = get(Field::ChamferTipOffset) {
                o.tip_offset = v;
            }
            if let Some(v) = get(Field::ChamferStep) {
                o.step = v.max(0.0);
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
            if let Some(v) = get(Field::LeadInSize) {
                o.lead_in = set_lead_size(o.lead_in, v);
            }
            if let Some(v) = get(Field::LeadOutSize) {
                o.lead_out = set_lead_size(o.lead_out, v);
            }
            if let Some(v) = get(Field::LeadOverlap) {
                o.lead_overlap = v.max(0.0);
            }
        }
        Operation::Bore(o) => {
            if let Some(v) = get(Field::BoreDiameter) {
                o.diameter = v;
            }
            if let Some(v) = get(Field::BoreTaper) {
                o.taper_angle = v;
            }
            if let Some(v) = get(Field::BorePitch) {
                o.pitch = v;
            }
            if o.ramp_angle.is_some() {
                if let Some(v) = get(Field::BoreAngle) {
                    o.ramp_angle = Some(v);
                }
            }
            if let Some(v) = get(Field::BoreStepovers) {
                o.stepovers = v as u32;
            }
            if let Some(v) = get(Field::BoreStepover) {
                o.stepover = v;
            }
            if let Some(v) = get(Field::BoreFinishing) {
                o.finishing_stepover = v;
            }
            if let Some(v) = get(Field::BoreRadialStock) {
                o.radial_stock = v;
            }
            if let Some(v) = get(Field::BoreAxialStock) {
                o.axial_stock = v;
            }
            if let Some(v) = get(Field::BoreTolerance) {
                o.tolerance = v;
            }
            if let Some(v) = get(Field::BoreSafeDistance) {
                o.safe_distance = v;
            }
            if let Some(v) = get(Field::BoreLeadRadius) {
                o.lead_radius = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
        }
        Operation::Thread(o) => {
            if let Some(v) = get(Field::MajorDia) {
                o.major_dia = v;
            }
            if let Some(v) = get(Field::Pitch) {
                o.pitch = v;
            }
            if let Some(v) = get(Field::ThreadTop) {
                o.z_top = v;
            }
            if let Some(v) = get(Field::ThreadBottom) {
                o.z_bottom = v;
            }
            if let Some(v) = get(Field::ThreadPasses) {
                o.passes = (v.round() as u32).max(1);
            }
            if let Some(v) = get(Field::ThreadSpringPasses) {
                o.spring_passes = v.round().max(0.0) as u32;
            }
            if let Some(v) = get(Field::ThreadDrillClearance) {
                o.drill_clearance = v.max(0.0);
            }
            if let Some(v) = get(Field::ThreadBlindAllowance) {
                o.blind_allowance = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
        }
        Operation::Spiral(o) => {
            if let Some(v) = get(Field::SpiralAxialOffset) {
                o.params.axial_offset = v;
            }
            if let Some(v) = get(Field::SpiralStepdowns) {
                o.params.stepdowns = v as u32;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.params.max_stepdown = v;
            }
            if let Some(v) = get(Field::SpiralStepover) {
                o.params.stepover = v;
            }
            if let Some(v) = get(Field::SpiralInnerLimit) {
                o.params.inner_limit = v;
            }
            if let Some(v) = get(Field::SpiralOuterLimit) {
                o.params.outer_limit = v;
            }
            if let Some(v) = get(Field::SpiralFromSlope) {
                o.params.from_slope = v;
            }
            if let Some(v) = get(Field::SpiralToSlope) {
                o.params.to_slope = v;
            }
            if let Some(v) = get(Field::SpiralStayDownDistance) {
                o.params.max_stay_down_distance = v;
            }
            if let Some(v) = get(Field::AdaptiveTolerance) {
                o.params.tolerance = v;
            }
            if let Some(v) = get(Field::AdaptiveOffset) {
                o.params.additional_offset = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.params.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.params.plunge_feed = v;
            }
            if let Some(v) = get(Field::ScallopSafeDistance) {
                o.params.safe_distance = v;
            }
            if let Some(v) = get(Field::ScallopSmoothingTolerance) {
                o.params.smoothing_tolerance = v;
            }
        }
        Operation::Parallel(o) => {
            if let Some(v) = get(Field::ParallelStepover) {
                o.params.stepover = v;
            }
            if let Some(v) = get(Field::ParallelAngle) {
                o.params.angle = v;
            }
            if let Some(v) = get(Field::ScallopSafeDistance) {
                o.params.safe_distance = v;
            }
            if let Some(v) = get(Field::ScallopSmoothingTolerance) {
                o.params.scallop_smoothing_tolerance = v;
            }
            if let Some(v) = get(Field::AdaptiveTolerance) {
                o.params.tolerance = v;
            }
            if let Some(v) = get(Field::AdaptiveOffset) {
                o.params.additional_offset = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.params.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.params.plunge_feed = v;
            }
        }
        Operation::Adaptive(o) => {
            if let Some(v) = get(Field::LeadInSize) {
                o.params.lead_in_radius = v;
            }
            if let Some(v) = get(Field::LeadOutSize) {
                o.params.lead_out_radius = v;
            }
            if let Some(v) = get(Field::AdaptiveTolerance) {
                o.params.tolerance = v;
            }
            if let Some(v) = get(Field::AdaptiveOptimalLoad) {
                o.params.optimal_load = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.params.stepdown = v;
            }
            if let Some(v) = get(Field::AdaptiveFineStepdown) {
                o.params.fine_stepdown = v;
            }
            if let Some(v) = get(Field::AdaptiveOffset) {
                o.params.additional_offset = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.params.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.params.plunge_feed = v;
            }
        }
        Operation::Engrave(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
        }
        Operation::Carve(o) => {
            if let Some(cl) = &mut o.clear {
                let p = &mut cl.params;
                if let Some(v) = get(Field::ClearStepdown) {
                    p.stepdown = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearOverlap) {
                    p.overlap = (v / 100.0).clamp(0.0, 0.99);
                }
                if let Some(v) = get(Field::ClearOffset) {
                    p.offset = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearEngagement) {
                    p.clearing.engagement = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearFeed) {
                    p.feed = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearPlungeFeed) {
                    p.plunge_feed = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearLeadOverlap) {
                    p.lead_overlap = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearLeadInSize) {
                    p.lead_in = set_lead_size(p.lead_in, v);
                }
                if let Some(v) = get(Field::ClearLeadOutSize) {
                    p.lead_out = set_lead_size(p.lead_out, v);
                }
                let (a, b) = plunge_params(p.plunge);
                let a = get(Field::ClearPlungeA).unwrap_or(a);
                let b = get(Field::ClearPlungeB).unwrap_or(b);
                p.plunge = set_plunge_params(p.plunge, a, b);
            }
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::ProfileOffset) {
                o.offset = v;
            }
            if let Some(v) = get(Field::RingStep) {
                o.ring_step = v.max(0.0);
            }
            if let Some(v) = get(Field::Scallop) {
                o.scallop = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
            let (a, b) = plunge_params(o.plunge);
            let a = get(Field::PlungeA).unwrap_or(a);
            let b = get(Field::PlungeB).unwrap_or(b);
            o.plunge = set_plunge_params(o.plunge, a, b);
        }
    }
}

/// Format a number for display (Rust's shortest round-trippable form).
fn fmt_num(v: f64) -> String {
    format!("{v}")
}

/// A selectable project-tree / list row: plain text that highlights when active
/// (a muted fill, not a button). Left-click sends `on_press`.
fn select_row<'a>(label: String, active: bool, on_press: Message) -> Element<'a, Message> {
    let content = container(text(label).size(10).color(palette::LABEL_COLOR))
        .width(Length::Fill)
        .padding(Padding::from([3.0, 6.0]))
        .style(move |_theme| container::Style {
            background: active.then_some(Background::Color(palette::SELECT_BG)),
            border: Border {
                radius: 3.0.into(),
                ..Border::default()
            },
            ..container::Style::default()
        });
    mouse_area(content).on_press(on_press).into()
}

/// A section header row in the project tree.
/// A tool's flute count as a short label, e.g. `"2 flutes"` / `"1 flute"`.
fn flute_count(flutes: u32) -> String {
    format!("{flutes} flute{}", if flutes == 1 { "" } else { "s" })
}

/// The end-mill family — square, roughing, ball-nose and rounded-edge (bull-nose) end mills —
/// which share a field set and dimensions, so switching between them preserves the
/// tool's measurements (a Type change *across* families resets to defaults instead).
fn end_mill_family(kind: ToolKind) -> bool {
    matches!(
        kind,
        ToolKind::EndMill | ToolKind::RoughingMill | ToolKind::BallMill | ToolKind::BullNose { .. }
    )
}

/// The **Tool Library** parenthetical — the extra, kind-specific descriptor shown only
/// in the Library pane (never the Project pane, which carries just number + compact
/// form). Radii use a shared `r…` form (`r0.5`, `r0`) for consistency across kinds, and
/// cutting direction a single letter (D/U/S). `None` = no parenthetical for this kind.
///
/// - End mills (square / ball): `(2 flutes, D)`
/// - Rounded-edge end mills: `(2 flutes, D, r1)` — trailing corner radius
/// - V-bits: `(60°, r0.25)` — point angle + tip radius
/// - Face / chamfer / thread mills: `(2 flutes)`
/// - Drill bits: none
fn library_extra(t: &cam_model::Tool) -> Option<String> {
    let dir = |d: CutDir| match d {
        CutDir::Down => "D",
        CutDir::Up => "U",
        CutDir::Straight => "S",
    };
    match t.kind {
        ToolKind::EndMill | ToolKind::RoughingMill | ToolKind::BallMill => Some(format!(
            "{}, {}",
            flute_count(t.flutes),
            dir(t.cutting_direction)
        )),
        ToolKind::RadiusMill { corner_radius, .. } => Some(format!(
            "({}, D{}, R{})",
            flute_count(t.flutes),
            t.diameter,
            corner_radius
        )),
        ToolKind::BullNose { corner_radius } => Some(format!(
            "{}, {}, r{}",
            flute_count(t.flutes),
            dir(t.cutting_direction),
            fmt_num(corner_radius)
        )),
        ToolKind::VBit {
            included_angle_deg,
            tip_radius,
        } => Some(format!(
            "{}°, r{}",
            fmt_num(included_angle_deg),
            fmt_num(tip_radius)
        )),
        // Chamfer mill mirrors the V-bit, but its tip is a flat ⌀ (non-cutting), not a
        // rounded radius — hence `⌀…` rather than `r…`.
        ToolKind::ChamferMill {
            included_angle_deg,
            tip_diameter,
        } => Some(format!(
            "{}°, ⌀{}",
            fmt_num(included_angle_deg),
            fmt_num(tip_diameter)
        )),
        // Thread mill: pitch first (its distinguishing trait), then flute count.
        ToolKind::ThreadMill { pitch } => Some(format!(
            "{}, {}",
            match pitch {
                Some(p) => format!("P{}", fmt_num(p)),
                None => "any pitch".to_string(),
            },
            flute_count(t.flutes)
        )),
        ToolKind::FaceMill => Some(flute_count(t.flutes)),
        ToolKind::Drill { .. } => None,
    }
}

/// A stable family sort/order key for the Tool Library pane's Family view.
fn kind_order(kind: ToolKind) -> u8 {
    match kind {
        ToolKind::EndMill => 0,
        ToolKind::RoughingMill => 1,
        ToolKind::BallMill => 2,
        ToolKind::BullNose { .. } => 3,
        ToolKind::RadiusMill { .. } => 9,
        ToolKind::FaceMill => 4,
        ToolKind::ChamferMill { .. } => 5,
        ToolKind::VBit { .. } => 6,
        ToolKind::Drill { .. } => 7,
        ToolKind::ThreadMill { .. } => 8,
    }
}

fn tree_header<'a>(label: &str) -> Element<'a, Message> {
    container(text(label.to_string()).size(11).color(palette::GROUP_LABEL))
        .padding(Padding::from([6.0, 4.0]))
        .into()
}

/// A muted placeholder note in the project tree (e.g. an empty section).
fn tree_note<'a>(label: &str) -> Element<'a, Message> {
    container(
        text(format!("({label})"))
            .size(11)
            .color(palette::GROUP_LABEL),
    )
    .padding(Padding::from([1.0, 12.0]))
    .into()
}

/// Width of an inspector's editable control, logical px.
///
/// Numeric inputs and pickers share it so their right edges line up down the column —
/// with a carve's two tools stacked in one inspector, a ragged edge reads as disorder.
const INSPECTOR_INPUT_W: f32 = 90.0;

/// Width for a picker whose options do not fit [`INSPECTOR_INPUT_W`] — "Conventional",
/// "Right-hand", a post's name. This is the width every picker had before the column was
/// introduced, kept unchanged for the ones that were already right.
const INSPECTOR_PICKER_W: f32 = 120.0;

/// An inspector field row: an explicit label + tooltip and a numeric text input bound to
/// `field`, drawn **invalid** (red border + value) when `invalid` — the thicker border
/// carries the signal by weight, not hue alone. The explicit label lets a field be
/// renamed and re-explained per tool kind (e.g. a V-bit's `ToolDiameter` reads "Shank
/// diameter" with matching help).
fn field_row_labeled<'a>(
    field: Field,
    label: &'a str,
    help: &'static str,
    value: &str,
    show: bool,
    invalid: bool,
) -> Element<'a, Message> {
    let mut input = inspector_input("", value)
        .align_x(iced::alignment::Horizontal::Right)
        .on_input(move |v| Message::FieldChanged(field, v))
        .on_submit(Message::Apply)
        .width(Length::Fixed(INSPECTOR_INPUT_W));
    if tool_drawing::is_parameter(field) {
        input = input.id(tool_drawing::parameter_id(field));
    } else if operation_tabs::field_tab(field) == OperationTab::Heights {
        input = input.id(height_planes::field_id(field));
    }
    if invalid {
        input = input.style(|theme, status| {
            let mut s = inspector_input_style(theme, status);
            s.border.color = palette::ERROR;
            s.border.width = 1.5;
            s.value = palette::ERROR;
            s
        });
    }
    row![label_help(label, help, show), input]
        .spacing(8)
        .align_y(Alignment::Center)
        .into()
}

/// A labelled strategy picker row (lead / plunge kind, side, …) for the inspector,
/// with a hover tooltip (`help`) explaining the choice when `show`.
fn profile_picker<T>(
    label: &str,
    help: &'static str,
    show: bool,
    selected: T,
    options: &'static [T],
    on_select: impl Fn(T) -> Message + 'static,
) -> Element<'static, Message>
where
    T: ToString + PartialEq + Clone + 'static,
{
    profile_picker_w(
        label,
        help,
        show,
        selected,
        options,
        on_select,
        INSPECTOR_INPUT_W,
    )
}

/// [`profile_picker`] at an explicit width, for the few whose option text does not fit
/// the shared column.
#[allow(clippy::too_many_arguments)]
fn profile_picker_w<T>(
    label: &str,
    help: &'static str,
    show: bool,
    selected: T,
    options: &'static [T],
    on_select: impl Fn(T) -> Message + 'static,
    width: f32,
) -> Element<'static, Message>
where
    T: ToString + PartialEq + Clone + 'static,
{
    let control = pick_list(options, Some(selected), on_select)
        .text_size(12)
        .padding([7, 9])
        .style(inspector_picker)
        .width(Length::Fixed(width));
    profile_picker_row(label, help, show, control)
}

fn profile_picker_row(
    label: &str,
    help: &'static str,
    show: bool,
    control: impl Into<Element<'static, Message>>,
) -> Element<'static, Message> {
    let whole_row_help = [
        help::LEAD_IN,
        help::LEAD_OUT,
        profile_parameters::CUT_ORDER_HELP,
        profile_parameters::CORNER_HELP,
    ]
    .contains(&help);
    let row = row![
        label_help(label.to_string(), help, show && !whole_row_help),
        control.into(),
    ]
    .spacing(8)
    .align_y(Alignment::Center);
    if whole_row_help {
        help_wrap(row, help, show)
    } else {
        row.into()
    }
}

// ---------------------------------------------------------------------------
// The wgpu viewport, hosted in iced's shader widget — a 3D orbit camera.
// ---------------------------------------------------------------------------

/// Persistent orbit state — an *unclamped turntable*: `yaw` spins about the
/// world up axis, `pitch` tilts. Both are top view at `0`, and neither is
/// clamped, so pitch can carry all the way round to the underside. A turntable
/// (rather than a free trackball) keeps the horizon level and every drag
/// predictable.
#[derive(Clone, Copy, Debug, Default)]
struct ViewControls {
    yaw: f32,
    pitch: f32,
    zoom: f32,
    pan: [f32; 3],
}

/// Which button is dragging the view.
#[derive(Clone, Copy, Debug)]
enum DragMode {
    Orbit,
    Pan,
}

/// The shader widget's transient state: the active drag plus a pending cube
/// snap. The camera lives in [`App`] so the view-cube buttons can drive it; drags
/// report *relative* deltas back as messages (loss-free even across a burst of
/// events).
#[derive(Default)]
struct ViewportState {
    drag: Option<DragMode>,
    last: Option<iced::Point>,
    /// A gizmo-face snap `(yaw, pitch)` armed on press, fired on release only if
    /// the press stays a click. Cleared the moment the press becomes a drag, so a
    /// rotation that merely *begins* over the cube orbits instead of snapping.
    gizmo_arm: Option<(f32, f32)>,
    cad_arm: Option<Vec<PickCandidate>>,
    planar_arm: Option<([f32; 2], f32)>,
    /// Where the (armed) press began, to measure click-vs-drag travel.
    press: Option<iced::Point>,
}

/// Orbit sensitivity (radians per pixel).
const ORBIT_SENS: f32 = 0.008;
/// Cursor travel (logical px) past which an armed cube press becomes an orbit
/// drag rather than a snap-to-view click.
const GIZMO_CLICK_SLOP: f32 = 4.0;
/// Zoom sensitivity (exponent per wheel line).
const ZOOM_SENS: f32 = 0.15;

/// Half-width (px) of the band along the viewport edges reserved for the
/// pane_grid resize dividers, whose grab zone (`PANE_RESIZE_LEEWAY`) extends a
/// little into the pane. Slightly larger than the leeway for safety.
const RESIZE_BAND: f32 = PANE_RESIZE_LEEWAY + 2.0;

/// Whether `pos` (absolute) is within [`RESIZE_BAND`] of any edge of `bounds` —
/// i.e. where a press likely means "drag the pane divider", not "orbit".
fn browser_region_contains(
    point: iced::Point,
    bounds: iced::Rectangle,
    width: f32,
    height: f32,
) -> bool {
    browser_contains(point, bounds, width)
        && point.y <= bounds.y + 8. + height.min(bounds.height - 16.)
}

fn browser_contains(point: iced::Point, bounds: iced::Rectangle, width: f32) -> bool {
    width > 0.
        && point.x >= bounds.x + 8.
        && point.x <= bounds.x + width + 8.
        && point.y >= bounds.y + 8.
        && point.y <= bounds.y + bounds.height - 8.
}

fn in_resize_band(pos: iced::Point, bounds: iced::Rectangle) -> bool {
    pos.x - bounds.x < RESIZE_BAND
        || bounds.x + bounds.width - pos.x < RESIZE_BAND
        || pos.y - bounds.y < RESIZE_BAND
        || bounds.y + bounds.height - pos.y < RESIZE_BAND
}

/// The origins an operation could be moved to: every origin in `indices` except the one
/// the operation already sits under, keeping `indices`' order so the menu matches the
/// tree. Empty on a single-origin job, which is what hides the menu section entirely.
///
/// `current` is the operation's stored `work_offset`, which need not name a live origin
/// — deleting an origin leaves its operations pointing at an index that is gone. The
/// tree groups such an operation under `base`, so this treats it as being there. Reading
/// the stale index literally would offer the base as a destination for a row already
/// drawn under the base, and moving it there would look like nothing happened.
///
/// A free function, not a method: it is the whole rule, and this way it can be tested
/// without standing up an application.
fn origin_move_targets(indices: &[u32], current: Option<u32>, base: u32) -> Vec<u32> {
    let here = current.filter(|wo| indices.contains(wo)).unwrap_or(base);
    indices.iter().copied().filter(|i| *i != here).collect()
}

/// The orientation cube's square rectangle, anchored to the top-right of a
/// `w × h` viewport. `size` and `margin` are given in the same units as `w,h`
/// (logical px for the click hit-test, physical px for drawing), so the cube is a
/// **fixed** on-screen size rather than a fraction of the window. Clamped so it
/// always fits within the viewport with its margin.
fn gizmo_rect(w: f32, h: f32, size: f32, margin: f32) -> (f32, f32, f32) {
    let size = size.clamp(1.0, (w.min(h) - 2.0 * margin).max(1.0));
    (w - size - margin, margin, size)
}

fn union_bounds(
    a: Option<([f32; 3], [f32; 3])>,
    b: Option<([f32; 3], [f32; 3])>,
) -> Option<([f32; 3], [f32; 3])> {
    match (a, b) {
        (Some((alo, ahi)), Some((blo, bhi))) => Some((
            std::array::from_fn(|i| alo[i].min(blo[i])),
            std::array::from_fn(|i| ahi[i].max(bhi[i])),
        )),
        (a, b) => a.or(b),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CadFilter {
    Body,
    Face,
    Edge,
    Loop,
    Vertex,
    Silhouette,
}
impl CadFilter {
    const ALL: [Self; 6] = [
        Self::Body,
        Self::Face,
        Self::Edge,
        Self::Loop,
        Self::Vertex,
        Self::Silhouette,
    ];
    fn kind(self) -> EntityKind {
        match self {
            Self::Body => EntityKind::Body,
            Self::Face => EntityKind::Face,
            Self::Edge => EntityKind::Edge,
            Self::Loop => EntityKind::Loop,
            Self::Vertex => EntityKind::Vertex,
            Self::Silhouette => EntityKind::Silhouette,
        }
    }
}
impl std::fmt::Display for CadFilter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

#[derive(Clone)]
struct Viewport {
    stock_contour_picking: bool,
    stock_contour_setup_selected: bool,
    setup_geometry: Arc<SetupPickGeometry>,
    setup_axis: bool,
    stock_axis_model_only: bool,
    operation_position_picking: bool,
    operation_position_color: [f32; 4],
    setup_hover: SetupHover,
    setup_snaps: Vec<SnapKind>,
    highlight_features: bool,
    snap_marker_scale: f32,
    browser_overlay_width: f32,
    browser_overlay_height: f32,
    planar_features: Vec<LoopRef>,
    planar_outlines: Arc<Vec<(LoopRef, Vec<Vertex>)>>,
    vertices: Arc<Vec<Vertex>>,
    path_cones: Arc<Vec<cam_render::PathCone>>,
    mesh_vertices: Arc<Vec<MeshVertex>>,
    mesh_indices: Arc<Vec<u32>>,
    transparent_stock_vertices: Arc<Vec<MeshVertex>>,
    transparent_stock_indices: Arc<Vec<u32>>,
    transparent_stock_convex: bool,
    show_stock: bool,
    cad_edges: Arc<Vec<Vertex>>,
    cad_scene: Arc<CadScene>,
    cad_filter: EntityKind,
    cad_filter_explicit: bool,
    geometry_pick_active: bool,
    cad_operation_kind: Option<OpKind>,
    cad_silhouette: bool,
    cad_select_through: bool,
    /// Scene extent (backplot + overlays) — used to *size* markers to the view.
    bounds: Option<([f32; 3], [f32; 3])>,
    /// The box the camera frames on: the **stock**, which is stable across
    /// operation-parameter edits. Framing on this (rather than the toolpath) keeps
    /// the part a constant on-screen size — tweaking an offset/stepover must not
    /// appear to resize the design.
    frame_bounds: Option<([f32; 3], [f32; 3])>,
    controls: ViewControls,
    grid_size: [f32; 3],
    grid_frame: cam_model::RigidTransform,
    show_gizmo: bool,
    /// On-screen cube size, logical px (fixed; independent of window size).
    gizmo_size: f32,
    /// The object-snap catch aperture, logical px. Carried per frame like `gizmo_size`
    /// rather than read from a constant, because it is a user preference now.
    ///
    /// The marker *scale* is not held here: the marker is built into the scene during
    /// [`Viewport::new`], where the preferences are still in scope, so storing it
    /// would be a copy with no reader.
    snap_catch_px: f32,
    /// Geometry-pick mode (a new-operation wizard is awaiting a region click).
    picking: bool,
    /// "Set origin" pick mode — a click drops the workpiece datum.
    set_origin: bool,
    /// An object-snap is engaged under the cursor — its marker replaces the
    /// crosshair/pickbox.
    snap_engaged: bool,
    /// The world Z of the plane clicks are projected onto (top of stock).
    pick_z: f32,
    /// Drilling annotations (peck rings / dwell bars) anchored in world space;
    /// sized to constant screen pixels and billboarded in [`Self::draw`].
    drill_marks: Arc<Vec<DrillMark>>,
}

impl Viewport {
    #[allow(clippy::too_many_arguments)] // cohesive per-frame view inputs
    fn new(
        controller: &AppController,
        preview: Option<&crate::RunOutcome>,
        operation_preview: Option<&crate::RunOutcome>,
        cad_filter: EntityKind,
        cad_hover: Option<&EntityRef>,
        cad_hidden: &BTreeSet<InstanceId>,
        cad_select_through: bool,
        show_stock: bool,
        controls: ViewControls,
        show_gizmo: bool,
        gizmo_size: f32,
        prefs: &crate::Settings,
        focus_ops: &[u32],
        hidden_ops: &[u32],
        snap: Option<(SnapHit, f64)>,
        hover_loop: Option<LoopRef>,
        set_origin: bool,
        show_origin: bool,
        origin_first: Option<[f64; 3]>,
        // The active machine's travel per axis, when the envelope is shown.
        envelope: Option<[f32; 3]>,
        editor_geometry: Option<&[EntityRef]>,
        silhouette_bottom: Option<f64>,
        cached_cad_scene: Option<Arc<CadScene>>,
        stock_draft: Option<&cam_model::Setup>,
        tree_body: Option<InstanceId>,
    ) -> Self {
        let cad_hover = cad_hover.filter(|reference| {
            preview.is_none()
                && !cad_hidden.contains(&reference.instance_id)
                && controller
                    .geometry_state()
                    .instance(reference.instance_id)
                    .is_some_and(|instance| instance.visible)
        });
        let picking = controller.pending_op().is_some();
        let pick_z = controller.inspection_setup().heights.top_of_stock as f32;
        // After a run, show the full backplot; before it, at least show the
        // imported part outlines so opening a file is visibly reflected.
        let outcome = if editor_geometry.is_some() {
            None
        } else {
            preview
                .or_else(|| controller.run_preview())
                .or_else(|| {
                    controller.outcome().filter(|verified| {
                        !focus_ops.is_empty()
                            && focus_ops.iter().all(|id| {
                                verified
                                    .scene
                                    .strips
                                    .iter()
                                    .any(|strip| strip.op == Some(*id))
                            })
                    })
                })
                .or(operation_preview)
                .or_else(|| controller.outcome())
        };
        let mut scene = match outcome {
            Some(outcome) => outcome.scene.clone(),
            None => {
                let mut scene = Scene::new();
                for region in controller.regions() {
                    scene.add_region(region, PART);
                }
                // Open imported strokes too, or an engravable path is invisible until
                // (and unless) a run happens to include it.
                for path in controller.open_paths() {
                    scene.add_open_path(path.points(), PART);
                }
                scene
            }
        };
        let show_run_preview = focus_ops.is_empty()
            && outcome
                .zip(controller.run_preview())
                .is_some_and(|(shown, running)| std::ptr::eq(shown, running));
        let operation_visible =
            |id| !hidden_ops.contains(&id) && (focus_ops.contains(&id) || show_run_preview);
        scene
            .strips
            .retain(|strip| preview.is_some() || strip.op.is_none_or(operation_visible));
        scene
            .path_cones
            .retain(|cone| preview.is_some() || operation_visible(cone.op));
        // Drilling annotations from the backplot (peck rings + dwell bars), once a
        // run has produced a program. Anchors only — sized to the screen at draw.
        let drill_marks = outcome
            .map(|o| {
                let frame = &controller.inspection_setup().frame;
                let visible_drills = Program {
                    steps: o.program.steps.iter().filter(|step| {
                        matches!(step, Step::Drill(c) if focus_ops.contains(&c.tag.op_id) && !hidden_ops.contains(&c.tag.op_id))
                    }).cloned().collect(),
                };
                drill_marks_of(&visible_drills)
                    .into_iter()
                    .map(|mut mark| {
                        mark.at = frame
                            .to_world_vector(mark.at.map(f64::from))
                            .map(|v| v as f32);
                        mark
                    })
                    .collect()
            })
            .unwrap_or_default();
        // Frame the camera on the *stable* part/backplot only — capture bounds
        // before the transient pick overlays (hover highlight, snap marker), or
        // the whole view would re-fit and drift as the cursor moves.
        let cad_scene = cached_cad_scene.unwrap_or_else(|| {
            Arc::new(CadScene::from_geometry_in_setup(
                controller.geometry_state(),
                cad_hidden,
                &controller.inspection_setup().frame,
            ))
        });
        let bounds = union_bounds(
            controller
                .outcome()
                .and_then(|outcome| outcome.scene.bounds())
                .or_else(|| scene.bounds()),
            cad_scene.bounds(),
        );
        // Playback keeps the machining selection intact, but draws the model
        // without selection colors or feature outlines over the simulated cut.
        let mut cad_selected = if preview.is_some() {
            Vec::new()
        } else if let Some(selected) = editor_geometry {
            selected.to_vec()
        } else {
            let mut selected = controller.cad_selection().to_vec();
            if let Some(instance) =
                tree_body.and_then(|id| controller.geometry_state().instance(id))
            {
                if let Some(asset) = controller.geometry_state().asset(instance.asset_id) {
                    selected.push(EntityRef {
                        asset_id: asset.id,
                        revision: asset.revision,
                        instance_id: instance.id,
                        body_id: instance.body_id,
                        kind: EntityKind::Body,
                        id: instance.body_id,
                    });
                }
            }
            for id in focus_ops {
                if let Some(binding) = controller.geometry_state().bindings.get(id) {
                    selected.extend(binding.selected.iter().cloned());
                }
            }
            selected
        };
        cad_selected.retain(|reference| {
            !cad_hidden.contains(&reference.instance_id)
                && controller
                    .geometry_state()
                    .instance(reference.instance_id)
                    .is_some_and(|instance| instance.visible)
        });
        let mut cad_lines = cad_scene.edges(&cad_selected, cad_hover);
        // CAD selection strips use the shared highlight weight. Move them to
        // the same through-model renderer used by silhouette selection; keep
        // ordinary model edges depth tested.
        let (highlights, edges): (Vec<_>, Vec<_>) = cad_lines
            .strips
            .into_iter()
            .partition(|strip| strip.width > 1.0);
        scene.strips.extend(highlights);
        cad_lines.strips = edges;
        let silhouette_refs: BTreeSet<_> = cad_selected
            .iter()
            .chain(cad_hover)
            .filter(|r| r.kind == EntityKind::Silhouette)
            .cloned()
            .collect();
        if !silhouette_refs.is_empty() {
            let frame = &controller.inspection_setup().frame;
            if let Ok(geometry) = controller.geometry_state().geometry_in_setup(frame) {
                for reference in silhouette_refs {
                    if let Ok(region) = cam_geometry3d::silhouette(&geometry, &reference) {
                        let z = silhouette_bottom
                            .filter(|_| cad_selected.contains(&reference))
                            .or_else(|| {
                                controller
                                    .document()
                                    .height_selection_bounds(std::slice::from_ref(&reference))
                                    .ok()
                                    .map(|(min, _)| min[2])
                            })
                            .unwrap_or(0.);
                        let mut points: Vec<_> = region
                            .outer()
                            .points()
                            .iter()
                            .map(|p| frame.to_world_vector([p.x, p.y, z]).map(|v| v as f32))
                            .collect();
                        if let Some(first) = points.first().copied() {
                            points.push(first);
                        }
                        let color = if cad_selected.contains(&reference) {
                            PICK_BOUNDARY
                        } else {
                            PICK_HOVER
                        };
                        // The projected outline must stay visible through
                        // the model/stock, like other selection overlays.
                        scene.add_highlight(points, color);
                    }
                }
            }
        }
        let cad_edges = Arc::new(cad_lines.line_vertices());
        // While the pick wizard is active, highlight the chosen boundary (accent)
        // and any excluded islands (gold) so the user sees what they picked.
        if let Some(pending) = controller.pending_op() {
            // The loop under the cursor (what a click selects), drawn first so the
            // boundary/island highlights paint over it once chosen.
            if let Some((pts, closed)) = hover_loop
                .filter(|l| Some(*l) != pending.boundary)
                .and_then(|l| controller.loop_points(l))
            {
                add_path_highlight(&mut scene, &pts, closed, PICK_HOVER);
            }
            if let Some((pts, closed)) = pending.boundary.and_then(|b| controller.loop_points(b)) {
                add_path_highlight(&mut scene, &pts, closed, PICK_BOUNDARY);
            }
            for island in pending
                .islands
                .iter()
                .chain(pending.additional_holes.iter())
            {
                if let Some((pts, closed)) = controller.loop_points(*island) {
                    add_path_highlight(&mut scene, &pts, closed, PICK_ISLAND);
                }
            }
        }
        // The workpiece-origin datum marker (View toggle), only once geometry is
        // loaded — never on an empty document at startup — and sized to the scene.
        if show_origin && controller.has_geometry() {
            if let Some((mn, mx)) = bounds {
                let scale = prefs.view.origin_marker_scale;
                let base_r = ((mx[0] - mn[0]).max(mx[1] - mn[1]) * 0.06 * scale).max(1.0);
                let active = controller.active_origin();
                // One marker per origin so every datum is visible; the active origin
                // (the one the pick flow and inspector edit) is drawn larger, so it
                // reads without relying on colour.
                for idx in controller.origin_indices() {
                    let pos = controller
                        .inspection_setup()
                        .frame
                        .to_world_vector(controller.origin_position(idx));
                    let r = if idx == active { base_r } else { base_r * 0.65 };
                    add_origin_marker(&mut scene, pos, pick_z, r);
                }
            }
        }
        // The first point captured in two-point origin mode (awaiting the second).
        if let Some(first) = origin_first {
            let r = bounds
                .map(|(mn, mx)| {
                    ((mx[0] - mn[0]).max(mx[1] - mn[1]) * 0.04 * prefs.view.origin_marker_scale)
                        .max(1.0)
                })
                .unwrap_or(3.0);
            add_origin_marker(&mut scene, first, first[2] as f32, r);
        }
        // The machine's travel, around the job. Added *after* `bounds` was captured, so a
        // large machine cannot re-frame the camera and shrink the part — the box is
        // context, not content.
        //
        // Red when the job does not fit: `check_travel` compares the program's span
        // against travel per axis, so this shows the same comparison before the export
        // refuses it rather than after.
        if let Some(travel) = envelope {
            if let Some((lo, hi)) = bounds {
                let over = (hi[0] - lo[0]) > travel[0]
                    || (hi[1] - lo[1]) > travel[1]
                    || (hi[2] - lo[2]) > travel[2];
                let colour = if over { ENVELOPE_OVER } else { ENVELOPE };
                scene.add_envelope(travel, (lo, hi), colour);
            }
        }
        // The object-snap marker under the cursor (op pick *or* set-origin).
        if let Some((hit, aperture)) = snap {
            add_snap_marker(
                &mut scene,
                hit,
                aperture,
                prefs.snapping.marker_scale,
                pick_z,
            );
        }
        // Browser stock ends at the last selected operation in execution order.
        // Never flash a later operation's removal while its preview is pending.
        // Transparent stock is drawn separately after the opaque CAD surfaces.
        let stock_outcome = if preview.is_some() {
            outcome
        } else if focus_ops.is_empty() || editor_geometry.is_some() {
            None
        } else {
            operation_preview
                .filter(|result| !result.stock_indices.is_empty())
                .or_else(|| {
                    let operations = &controller.inspection_setup().operations;
                    let through = operations
                        .iter()
                        .rposition(|op| focus_ops.contains(&op.id()))?;
                    let expected: BTreeSet<_> = operations[..=through]
                        .iter()
                        .filter(|op| {
                            controller
                                .document()
                                .operation_is_included(controller.active_setup_id(), op.id())
                        })
                        .map(Operation::id)
                        .collect();
                    controller.outcome().filter(|result| {
                        let actual: BTreeSet<_> = result
                            .program
                            .steps()
                            .iter()
                            .filter_map(Step::tag)
                            .map(|tag| tag.op_id)
                            .collect();
                        actual == expected
                    })
                })
        };
        let (mut mesh_vertices, mut mesh_indices) = match (show_stock, stock_draft, stock_outcome) {
            (true, Some(draft), _) => workspace_view::uncut_stock_mesh_for_setup(controller, draft),
            (true, None, Some(outcome))
                if outcome.completion == JobCompletion::Complete
                    || !outcome.stock_indices.is_empty() =>
            {
                (
                    outcome.stock_vertices.clone(),
                    outcome.stock_indices.clone(),
                )
            }
            (true, None, _) => workspace_view::uncut_stock_mesh(controller),
            _ => (Vec::new(), Vec::new()),
        };
        // The initial box or solid cylinder is convex. Its front and rear
        // triangles need separate ordering when stock is translucent; a
        // simulated or hollow blank can be non-convex and uses depth sorting.
        let transparent_stock_convex = preview.is_none()
            && (stock_draft.is_some() || stock_outcome.is_none())
            && matches!(
                stock_draft
                    .unwrap_or_else(|| controller.inspection_setup())
                    .stock,
                cam_model::Stock::BoundingBox { .. }
                    | cam_model::Stock::FixedBox { .. }
                    | cam_model::Stock::RelativeBox { .. }
                    | cam_model::Stock::FixedRound { tube: false, .. }
                    | cam_model::Stock::RelativeRound { tube: false, .. }
            );
        let [r, g, b] = prefs.view.stock_color;
        for (index, vertex) in mesh_vertices.iter_mut().enumerate() {
            if preview.is_none()
                && stock_outcome
                    .and_then(|outcome| outcome.stock_operations.get(index).copied().flatten())
                    .is_some_and(|op| focus_ops.contains(&op) && !hidden_ops.contains(&op))
            {
                vertex.color = [r, g, b, 1.0];
            }
        }
        let (transparent_stock_vertices, transparent_stock_indices) =
            if prefs.view.transparent_stock {
                for vertex in &mut mesh_vertices {
                    vertex.color[3] = 1.0 - prefs.view.stock_transparency;
                }
                (
                    std::mem::take(&mut mesh_vertices),
                    std::mem::take(&mut mesh_indices),
                )
            } else {
                (Vec::new(), Vec::new())
            };
        // Opaque stock views retain only CAD reference edges to avoid hiding
        // gouges or fighting coincident cut faces. Transparent stock needs the
        // solid model behind it for comparison.
        if !show_stock
            || prefs.view.transparent_stock
            || (mesh_indices.is_empty() && transparent_stock_indices.is_empty())
        {
            let (cad_vertices, cad_indices) = cad_scene.mesh(&cad_selected, cad_hover);
            let offset = mesh_vertices.len() as u32;
            mesh_vertices.extend(cad_vertices);
            mesh_indices.extend(cad_indices.into_iter().map(|i| i + offset));
        }
        // Frame on the stock (stable across parameter edits) so the part keeps a
        // constant on-screen size; fall back to the scene extent before any
        // geometry is loaded.
        let frame_bounds = if controller.has_geometry() {
            union_bounds(
                stock_draft
                    .and_then(|setup| {
                        workspace_view::stock_world_bounds_for_setup(controller, setup)
                    })
                    .or_else(|| workspace_view::stock_world_bounds(controller)),
                cad_scene.bounds(),
            )
        } else {
            Some(([-50., -50., 0.], [50., 50., 0.]))
        };
        Self {
            stock_contour_picking: false,
            stock_contour_setup_selected: false,
            setup_geometry: Arc::new(SetupPickGeometry::new(
                controller,
                controller.inspection_setup(),
                cad_hidden,
            )),
            setup_axis: false,
            stock_axis_model_only: false,
            operation_position_picking: false,
            operation_position_color: operation_positions::PositionKind::Entry.color(),
            setup_hover: SetupHover::default(),
            setup_snaps: if prefs.snapping.enabled {
                prefs.snapping.default_snaps.clone()
            } else {
                vec![]
            },
            highlight_features: prefs.snapping.highlight_features,
            snap_marker_scale: prefs.snapping.marker_scale,
            browser_overlay_width: 0.,
            browser_overlay_height: f32::INFINITY,
            planar_features: Vec::new(),
            planar_outlines: Arc::new(
                controller
                    .regions()
                    .iter()
                    .enumerate()
                    .flat_map(|(region, polygon)| {
                        std::iter::once(LoopRef::outer(region)).chain(
                            (0..polygon.holes().len()).map(move |hole| LoopRef::hole(region, hole)),
                        )
                    })
                    .chain((0..controller.open_paths().len()).map(LoopRef::open))
                    .filter_map(|reference| {
                        let (points, closed) = controller.loop_points(reference)?;
                        let mut highlight = Scene::new();
                        add_path_highlight(&mut highlight, &points, closed, PICK_BOUNDARY);
                        Some((reference, highlight.line_vertices()))
                    })
                    .collect(),
            ),
            vertices: Arc::new(scene.line_vertices()),
            path_cones: Arc::new(scene.path_cones),
            mesh_vertices: Arc::new(mesh_vertices),
            mesh_indices: Arc::new(mesh_indices),
            transparent_stock_vertices: Arc::new(transparent_stock_vertices),
            transparent_stock_indices: Arc::new(transparent_stock_indices),
            transparent_stock_convex,
            show_stock,
            cad_edges,
            cad_scene,
            cad_filter,
            cad_filter_explicit: false,
            geometry_pick_active: false,
            cad_operation_kind: None,
            cad_silhouette: false,
            cad_select_through,
            bounds,
            frame_bounds,
            controls,
            grid_size: {
                let (x, y, z) = controller.machine().envelope.extent();
                [x as f32, y as f32, z as f32]
            },
            grid_frame: Default::default(),
            show_gizmo,
            gizmo_size,
            snap_catch_px: prefs.snapping.pickbox_px * crate::SNAP_CATCH_MULTIPLE,
            picking,
            set_origin,
            snap_engaged: snap.is_some(),
            pick_z,
            drill_marks: Arc::new(drill_marks),
        }
    }

    fn setup_hover_at(&self, pos: iced::Point, bounds: iced::Rectangle) -> SetupHover {
        if bounds.width <= 0. || bounds.height <= 0. {
            return SetupHover::default();
        }
        self.setup_geometry.query(
            self.camera(),
            bounds,
            pos,
            &self.setup_snaps,
            self.setup_axis,
            self.stock_axis_model_only,
            &self.cad_scene,
            self.cad_select_through,
            0.5 * self.snap_catch_px,
        )
    }
    fn operation_position_at(&self, pos: iced::Point, bounds: iced::Rectangle) -> Option<[f64; 3]> {
        if bounds.width <= 0.0 || bounds.height <= 0.0 {
            return None;
        }
        let camera = self.camera();
        let ray = camera.pick_ray(
            2.0 * (pos.x - bounds.x) / bounds.width - 1.0,
            1.0 - 2.0 * (pos.y - bounds.y) / bounds.height,
            bounds.width / bounds.height,
        );
        let world = self
            .setup_hover_at(pos, bounds)
            .snap
            .map(|hit| hit.point)
            .or_else(|| {
                self.cad_scene
                    .pick_candidates(ray, EntityKind::Face, 0.0, false)
                    .first()
                    .map(|hit| {
                        std::array::from_fn(|i| ray.origin[i] + hit.depth * ray.direction[i])
                    })
            })
            .or_else(|| self.setup_geometry.fallback(ray))?;
        Some(self.setup_geometry.to_setup(world))
    }

    fn origin_point_at(&self, pos: iced::Point, bounds: iced::Rectangle) -> Option<[f64; 3]> {
        if let Some(hit) = self.setup_hover_at(pos, bounds).snap {
            return Some(hit.point);
        }
        let u = 2. * (pos.x - bounds.x) / bounds.width - 1.;
        let v = 1. - 2. * (pos.y - bounds.y) / bounds.height;
        self.setup_geometry.fallback(self.camera().pick_ray(
            u,
            v,
            bounds.width / bounds.height.max(1.),
        ))
    }

    fn cad_candidates_at(&self, pos: iced::Point, bounds: iced::Rectangle) -> Vec<PickCandidate> {
        if bounds.width <= 0.0 || bounds.height <= 0.0 {
            return vec![];
        }
        let aspect = bounds.width / bounds.height;
        let u = 2.0 * (pos.x - bounds.x) / bounds.width - 1.0;
        let v = 1.0 - 2.0 * (pos.y - bounds.y) / bounds.height;
        let camera = self.camera();
        // Portrait framing grows the vertical half extent; use the actual pixel scale.
        let aperture =
            0.5 * self.snap_catch_px * camera.world_per_pixel(bounds.height) / aspect.min(1.0);
        let ray = camera.pick_ray(u, v, aspect);
        if self.cad_silhouette && !self.setup_axis {
            return self.cad_scene.pick_silhouette_candidates(
                ray,
                aperture as f64,
                self.cad_select_through,
            );
        }
        if self.stock_contour_picking && self.cad_filter == EntityKind::Loop {
            // Stock loops must be planar in setup XY. A cylindrical wall loop
            // includes its axial seam and must not win over the adjacent rim.
            return self.cad_scene.pick_pocket_loop_candidates(
                ray,
                aperture as f64,
                self.cad_select_through,
            );
        }
        let operation_kind = if self.setup_axis {
            None
        } else {
            self.cad_operation_kind
        };
        if !self.setup_axis {
            if let Some(kind) = operation_kind {
                if self.geometry_pick_active && !self.cad_filter_explicit {
                    return self.operation_candidates(kind, ray, aperture as f64);
                }
            }
            if self.cad_operation_kind.is_none() && !self.cad_filter_explicit {
                return self.cad_scene.pick_any_candidates(
                    ray,
                    &[
                        EntityKind::Face,
                        EntityKind::Edge,
                        EntityKind::Loop,
                        EntityKind::Vertex,
                        EntityKind::Body,
                    ],
                    aperture as f64,
                    self.cad_select_through,
                );
            }
        }
        match if self.setup_axis {
            None
        } else {
            self.cad_operation_kind
        } {
            Some(OpKind::Pocket | OpKind::Profile | OpKind::Chamfer | OpKind::Engrave)
                if matches!(self.cad_filter, EntityKind::Face | EntityKind::Edge) =>
            {
                self.cad_scene.pick_candidates(
                    ray,
                    self.cad_filter,
                    aperture as f64,
                    self.cad_select_through,
                )
            }
            Some(OpKind::Pocket) if self.cad_filter == EntityKind::Loop => self
                .cad_scene
                .pick_pocket_loop_candidates(ray, aperture as f64, self.cad_select_through),
            Some(OpKind::Chamfer | OpKind::Engrave | OpKind::Carve)
                if self.cad_filter == EntityKind::Loop =>
            {
                self.cad_scene.pick_surface_loop_candidates(
                    ray,
                    aperture as f64,
                    self.cad_select_through,
                )
            }
            Some(OpKind::Profile) => self.cad_scene.pick_profile_candidates(
                ray,
                aperture as f64,
                self.cad_select_through,
            ),
            Some(OpKind::Drill | OpKind::Bore | OpKind::Thread) => self
                .cad_scene
                .pick_drill_candidates(ray, aperture as f64, self.cad_select_through),
            _ => self.cad_scene.pick_candidates(
                ray,
                if self.setup_axis {
                    EntityKind::Edge
                } else {
                    self.cad_filter
                },
                aperture as f64,
                self.cad_select_through,
            ),
        }
    }

    fn operation_candidates(
        &self,
        kind: OpKind,
        ray: cam_render::Ray,
        aperture: f64,
    ) -> Vec<PickCandidate> {
        let through = self.cad_select_through;
        let mut candidates = match kind {
            OpKind::Profile => {
                let mut candidates = self
                    .cad_scene
                    .pick_profile_candidates(ray, aperture, through);
                candidates.extend(self.cad_scene.pick_any_candidates(
                    ray,
                    &[EntityKind::Face, EntityKind::Edge, EntityKind::Silhouette],
                    aperture,
                    through,
                ));
                candidates
            }
            OpKind::Chamfer => {
                let mut candidates = self.cad_scene.pick_any_candidates(
                    ray,
                    &[EntityKind::Face, EntityKind::Edge],
                    aperture,
                    through,
                );
                candidates.extend(
                    self.cad_scene
                        .pick_surface_loop_candidates(ray, aperture, through),
                );
                candidates
            }
            OpKind::Engrave => self.cad_scene.pick_any_candidates(
                ray,
                &[EntityKind::Edge, EntityKind::Loop],
                aperture,
                through,
            ),
            OpKind::Pocket | OpKind::Carve => {
                let mut candidates =
                    self.cad_scene
                        .pick_any_candidates(ray, &[EntityKind::Face], aperture, through);
                candidates.extend(if kind == OpKind::Pocket {
                    self.cad_scene
                        .pick_pocket_loop_candidates(ray, aperture, through)
                } else {
                    self.cad_scene
                        .pick_surface_loop_candidates(ray, aperture, through)
                });
                candidates
            }
            OpKind::Drill | OpKind::Bore | OpKind::Thread => {
                self.cad_scene.pick_drill_candidates(ray, aperture, through)
            }
            OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral => {
                self.cad_scene.pick_any_candidates(
                    ray,
                    &[EntityKind::Face, EntityKind::Body],
                    aperture,
                    through,
                )
            }
            OpKind::Face => self.cad_scene.pick_any_candidates(
                ray,
                &[EntityKind::Face, EntityKind::Silhouette],
                aperture,
                through,
            ),
        };
        candidates.sort_by(|a, b| {
            a.depth
                .total_cmp(&b.depth)
                .then_with(|| {
                    let rank = |entity: &EntityRef| match kind {
                        OpKind::Profile => match entity.kind {
                            EntityKind::Loop => 0,
                            EntityKind::Edge => 1,
                            EntityKind::Face => 2,
                            EntityKind::Silhouette => 3,
                            _ => 4,
                        },
                        OpKind::Chamfer => match entity.kind {
                            EntityKind::Loop => 0,
                            EntityKind::Face => 1,
                            EntityKind::Edge => 2,
                            _ => 3,
                        },
                        OpKind::Engrave => match entity.kind {
                            EntityKind::Loop => 0,
                            EntityKind::Edge => 1,
                            _ => 2,
                        },
                        OpKind::Pocket | OpKind::Carve => match entity.kind {
                            EntityKind::Face => 0,
                            EntityKind::Loop => 1,
                            _ => 2,
                        },
                        OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral => match entity.kind {
                            EntityKind::Face => 0,
                            EntityKind::Body => 1,
                            _ => 2,
                        },
                        OpKind::Face => match entity.kind {
                            EntityKind::Face => 0,
                            EntityKind::Silhouette => 1,
                            _ => 2,
                        },
                        OpKind::Drill | OpKind::Bore | OpKind::Thread => 0,
                    };
                    rank(&a.entity).cmp(&rank(&b.entity))
                })
                .then_with(|| a.entity.cmp(&b.entity))
        });
        let mut seen = BTreeSet::new();
        candidates.retain(|candidate| seen.insert(candidate.entity.clone()));
        candidates
    }

    /// The orbit camera framed on the stable stock box, with the current controls.
    fn camera(&self) -> OrbitCamera {
        let (min, max) = self
            .frame_bounds
            .or(self.bounds)
            .unwrap_or(([0.0, 0.0, 0.0], [1.0, 1.0, 0.0]));
        let mut cam = OrbitCamera::framed(min, max);
        // Keep the lateral framing on the stock (stable on-screen size), but stretch the
        // depth range to cover the whole toolpath — tall tool-change lifts reach well
        // above the stock, and would otherwise clip on rotate at any zoom.
        if let Some((smin, smax)) = self.bounds {
            cam.cover_depth(smin, smax);
        }
        cam.orient = cam_render::orientation(self.controls.yaw, self.controls.pitch);
        cam.zoom = self.controls.zoom;
        cam.pan = self.controls.pan;
        cam
    }

    /// The orientation cube's camera: the same rotation as the part, framed on
    /// the unit cube (no zoom / pan).
    fn gizmo_camera(&self) -> OrbitCamera {
        let mut cam = OrbitCamera::framed([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
        cam.orient = cam_render::orientation(self.controls.yaw, self.controls.pitch);
        cam
    }

    /// If `pos` (absolute) lands on a gizmo face, the `(yaw, pitch)` to snap to.
    fn gizmo_pick(&self, pos: iced::Point, bounds: iced::Rectangle) -> Option<(f32, f32)> {
        if !self.show_gizmo {
            return None;
        }
        let (gx, gy, size) = gizmo_rect(bounds.width, bounds.height, self.gizmo_size, GIZMO_MARGIN);
        let (gxa, gya) = (bounds.x + gx, bounds.y + gy);
        if pos.x < gxa || pos.x > gxa + size || pos.y < gya || pos.y > gya + size {
            return None;
        }
        let u = 2.0 * (pos.x - gxa) / size - 1.0;
        let v = 1.0 - 2.0 * (pos.y - gya) / size; // screen-y down → NDC-y up
        let cam = self.gizmo_camera();
        let normal = cam_render::pick_face(cam.orient, cam.half_height(), u, v)?;
        Some(face_view(normal))
    }
}

impl shader::Program<Message> for Viewport {
    type State = ViewportState;
    type Primitive = ScenePrimitive;

    fn update(
        &self,
        state: &mut Self::State,
        event: &iced::Event,
        bounds: iced::Rectangle,
        cursor: iced::mouse::Cursor,
    ) -> Option<shader::Action<Message>> {
        let iced::Event::Mouse(mouse_event) = event else {
            return None;
        };
        use iced::mouse::{Button, Event as Mouse, ScrollDelta};
        let position = match mouse_event {
            Mouse::CursorMoved { position } => Some(*position),
            _ => cursor.position(),
        };
        if position.is_some_and(|position| {
            browser_region_contains(
                position,
                bounds,
                self.browser_overlay_width,
                self.browser_overlay_height,
            )
        }) {
            state.drag = None;
            state.last = None;
            state.press = None;
            state.cad_arm = None;
            state.planar_arm = None;
            state.gizmo_arm = None;
            return Some(shader::Action::capture());
        }
        match mouse_event {
            Mouse::ButtonPressed(button) => {
                let pos = cursor.position_over(bounds)?;
                // A left-press on a gizmo face *arms* a snap-to-view: it fires on
                // release only if the press stays a click (see ButtonReleased). We
                // still begin an orbit drag, so if the user drags — a rotation that
                // merely started over the cube — it orbits and the snap is dropped.
                if matches!(button, Button::Left) {
                    if let Some(view) = self.gizmo_pick(pos, bounds) {
                        state.gizmo_arm = Some(view);
                        state.drag = Some(DragMode::Orbit);
                        state.last = Some(pos);
                        state.press = Some(pos);
                        return Some(shader::Action::capture());
                    }
                }
                if matches!(button, Button::Left) && self.operation_position_picking {
                    let point = self.operation_position_at(pos, bounds);
                    return Some(match point {
                        Some(point) => {
                            shader::Action::publish(Message::OperationPositionPicked(point))
                                .and_capture()
                        }
                        None => shader::Action::capture(),
                    });
                }
                if matches!(button, Button::Left)
                    && self.stock_contour_picking
                    && self.show_stock
                    && matches!(
                        self.setup_hover_at(pos, bounds).edge,
                        Some(SetupFeature::Stock(_))
                    )
                {
                    return Some(
                        shader::Action::publish(Message::OperationGeometry(
                            GeometryMessage::ResetStock,
                        ))
                        .and_capture(),
                    );
                }
                if matches!(button, Button::Left) && self.set_origin {
                    return Some(match self.origin_point_at(pos, bounds) {
                        Some(point) => {
                            shader::Action::publish(Message::OriginPointPicked(point)).and_capture()
                        }
                        None => shader::Action::capture(),
                    });
                }
                if matches!(button, Button::Left) && self.setup_axis {
                    let hover = self.setup_hover_at(pos, bounds);
                    let message = match hover.edge {
                        Some(source @ SetupFeature::Stock(_)) => self
                            .setup_geometry
                            .direction(&source)
                            .map(Message::SetupStockEdgePicked),
                        Some(SetupFeature::Model(_)) => {
                            Some(Message::CadPicked(self.cad_candidates_at(pos, bounds)))
                        }
                        _ => Some(Message::CadPicked(vec![])),
                    };
                    return Some(match message {
                        Some(m) => shader::Action::publish(m).and_capture(),
                        None => shader::Action::capture(),
                    });
                }
                // In geometry-pick mode a left-click selects geometry (projected
                // onto the stock-top plane) instead of orbiting; the pickbox
                // half-size becomes the world-space snap aperture.
                if (self.picking || self.set_origin) && matches!(button, Button::Left) {
                    let aspect = if bounds.height > 0.0 {
                        bounds.width / bounds.height
                    } else {
                        1.0
                    };
                    let u = 2.0 * (pos.x - bounds.x) / bounds.width - 1.0;
                    let v = 1.0 - 2.0 * (pos.y - bounds.y) / bounds.height;
                    let cam = self.camera();
                    if let Some(w) = cam.pick_plane(u, v, aspect, self.pick_z) {
                        let aperture =
                            0.5 * self.snap_catch_px * cam.world_per_pixel(bounds.height);
                        let msg = Message::PickWorld(w, aperture);
                        return Some(shader::Action::publish(msg).and_capture());
                    }
                    return Some(shader::Action::capture());
                }
                // Leave presses in the pane-resize band along the edges to the
                // pane_grid divider — its grab zone bleeds a few px into the pane,
                // and capturing them here would orbit while the user resizes.
                if in_resize_band(pos, bounds) {
                    return None;
                }
                if matches!(button, Button::Left)
                    && !self.set_origin
                    && self.cad_scene.bounds().is_some()
                {
                    state.cad_arm = Some(self.cad_candidates_at(pos, bounds));
                    state.drag = Some(DragMode::Orbit);
                    state.last = Some(pos);
                    state.press = Some(pos);
                    return Some(shader::Action::capture());
                }
                if matches!(button, Button::Left)
                    && self.cad_scene.bounds().is_none()
                    && !self.set_origin
                    && !self.picking
                {
                    let aspect = bounds.width / bounds.height.max(1.);
                    let u = 2. * (pos.x - bounds.x) / bounds.width - 1.;
                    let v = 1. - 2. * (pos.y - bounds.y) / bounds.height;
                    if let Some(point) = self.camera().pick_plane(u, v, aspect, self.pick_z) {
                        state.planar_arm = Some((
                            point,
                            0.5 * self.snap_catch_px * self.camera().world_per_pixel(bounds.height),
                        ));
                        state.press = Some(pos);
                    }
                }
                let mode = match button {
                    Button::Left => DragMode::Orbit,
                    Button::Middle | Button::Right => DragMode::Pan,
                    _ => return None,
                };
                state.drag = Some(mode);
                state.last = Some(pos);
                return Some(shader::Action::capture());
            }
            Mouse::CursorMoved { position } => {
                if let (Some(mode), Some(last)) = (state.drag, state.last) {
                    // While a cube snap is armed, hold off orbiting: a small wiggle
                    // still counts as a click, but travel past the slop commits to
                    // an orbit and drops the snap.
                    if let Some(press) = state.press {
                        if state.gizmo_arm.is_some()
                            || state.cad_arm.is_some()
                            || state.planar_arm.is_some()
                        {
                            let moved = (position.x - press.x).hypot(position.y - press.y);
                            if moved < GIZMO_CLICK_SLOP {
                                return Some(shader::Action::capture());
                            }
                            state.gizmo_arm = None;
                            state.cad_arm = None;
                            state.planar_arm = None;
                            // Apply this movement too: a fast drag may have only one
                            // move event before release. The slop already excludes clicks.
                        }
                    }
                    let (dx, dy) = (position.x - last.x, position.y - last.y);
                    state.last = Some(*position);
                    // Report a relative change; App owns and accumulates it.
                    let message = match mode {
                        DragMode::Orbit => Message::OrbitBy(dx * ORBIT_SENS, dy * ORBIT_SENS),
                        DragMode::Pan => {
                            let cam = self.camera();
                            let wpp = cam.world_per_pixel(bounds.height);
                            let (r, u) = (cam.right(), cam.up());
                            // Drag moves the scene with the cursor.
                            Message::PanBy([
                                (-r[0] * dx + u[0] * dy) * wpp,
                                (-r[1] * dx + u[1] * dy) * wpp,
                                (-r[2] * dx + u[2] * dy) * wpp,
                            ])
                        }
                    };
                    return Some(shader::Action::publish(message).and_capture());
                }
                if self.set_origin || self.setup_axis || self.operation_position_picking {
                    let hover = if cursor.position_over(bounds).is_some() {
                        self.setup_hover_at(*position, bounds)
                    } else {
                        SetupHover::default()
                    };
                    return Some(shader::Action::publish(Message::SetupHover(
                        *position, hover,
                    )));
                }
                if !self.set_origin && self.cad_scene.bounds().is_some() {
                    let candidate = cursor.position_over(bounds).and_then(|p| {
                        self.cad_candidates_at(p, bounds)
                            .first()
                            .map(|c| c.entity.clone())
                    });
                    return Some(shader::Action::publish(Message::CadHover(candidate)));
                }
                // While a pick is pending (op or set-origin), track the cursor
                // (pickbox) and its world point (snap preview). Passive tracking.
                if (self.picking || self.set_origin) && cursor.position_over(bounds).is_some() {
                    let aspect = if bounds.height > 0.0 {
                        bounds.width / bounds.height
                    } else {
                        1.0
                    };
                    let u = 2.0 * (position.x - bounds.x) / bounds.width - 1.0;
                    let v = 1.0 - 2.0 * (position.y - bounds.y) / bounds.height;
                    let cam = self.camera();
                    let msg = match cam.pick_plane(u, v, aspect, self.pick_z) {
                        Some(w) => {
                            let aperture =
                                0.5 * self.snap_catch_px * cam.world_per_pixel(bounds.height);
                            Message::HoverWorld(*position, w, aperture)
                        }
                        None => Message::ViewportCursor(*position),
                    };
                    return Some(shader::Action::publish(msg));
                }
            }
            Mouse::ButtonReleased(_) => {
                if let Some((point, aperture)) = state.planar_arm.take() {
                    state.drag = None;
                    state.last = None;
                    state.press = None;
                    return Some(
                        shader::Action::publish(Message::PickWorld(point, aperture)).and_capture(),
                    );
                }
                if let Some(candidates) = state.cad_arm.take() {
                    state.drag = None;
                    state.last = None;
                    state.press = None;
                    return Some(
                        shader::Action::publish(Message::CadPicked(candidates)).and_capture(),
                    );
                }
                // A still-armed press never became a drag: it's a click — snap now.
                if let Some((yaw, pitch)) = state.gizmo_arm.take() {
                    state.drag = None;
                    state.last = None;
                    state.press = None;
                    return Some(
                        shader::Action::publish(Message::SetView(yaw, pitch)).and_capture(),
                    );
                }
                if state.drag.take().is_some() {
                    state.last = None;
                    state.press = None;
                    return Some(shader::Action::capture());
                }
            }
            Mouse::WheelScrolled { delta } if cursor.position_over(bounds).is_some() => {
                let lines = match delta {
                    ScrollDelta::Lines { y, .. } => *y,
                    ScrollDelta::Pixels { y, .. } => *y / 20.0,
                };
                return Some(
                    shader::Action::publish(Message::ZoomBy(lines * ZOOM_SENS)).and_capture(),
                );
            }
            _ => {}
        }
        None
    }

    fn draw(
        &self,
        _state: &Self::State,
        cursor: iced::mouse::Cursor,
        bounds: iced::Rectangle,
    ) -> Self::Primitive {
        let aspect = if bounds.height > 0.0 {
            bounds.width / bounds.height
        } else {
            1.0
        };
        // Drilling annotations are sized here, where the viewport's pixel height is
        // known, so they stay a constant screen size at any zoom (never scaling with
        // the model) and billboard to face the camera at any orbit angle.
        // Transient annotations have a separate buffer. Orbiting or hovering must
        // never copy the full backplot just to append a handful of overlay lines.
        let mut overlays = Vec::new();
        if !self.drill_marks.is_empty() {
            let cam = self.camera();
            let px_world = DRILL_MARK_PX * cam.world_per_pixel(bounds.height);
            for &mark in self.drill_marks.iter() {
                push_drill_mark(&mut overlays, mark, px_world, cam.right(), cam.up());
            }
        }
        for reference in &self.planar_features {
            if let Some((_, points)) = self.planar_outlines.iter().find(|(r, _)| r == reference) {
                overlays.extend_from_slice(points);
            }
        }
        let mut setup_scene = Scene::new();
        if self.show_stock {
            for (_, edge) in self.setup_geometry.stock_lines() {
                setup_scene.add_strip(
                    edge.iter().map(|p| p.map(|v| v as f32)).collect(),
                    if self.stock_contour_setup_selected {
                        PICK_BOUNDARY
                    } else {
                        [0.45, 0.49, 0.54, 1.]
                    },
                );
            }
        }
        if self.set_origin || self.setup_axis || self.operation_position_picking {
            self.setup_geometry.overlays(
                &mut setup_scene,
                self.camera(),
                bounds,
                &self.setup_hover,
                &self.setup_snaps,
                self.highlight_features,
                self.set_origin || self.operation_position_picking,
                self.snap_marker_scale,
                &self.cad_scene,
                self.cad_select_through,
            );
        }
        if self.operation_position_picking {
            if let Some(point) = cursor
                .position_over(bounds)
                .filter(|point| {
                    !browser_region_contains(
                        *point,
                        bounds,
                        self.browser_overlay_width,
                        self.browser_overlay_height,
                    )
                })
                .and_then(|point| self.operation_position_at(point, bounds))
            {
                let point = self.setup_geometry.to_world(point);
                let camera = self.camera();
                let size = 7.0 * self.snap_marker_scale * camera.world_per_pixel(bounds.height)
                    / aspect.min(1.0);
                for direction in [camera.right(), camera.up()] {
                    setup_scene.add_highlight(
                        [-1.0, 1.0]
                            .into_iter()
                            .map(|sign| {
                                std::array::from_fn(|i| {
                                    point[i] as f32 + sign * size * direction[i]
                                })
                            })
                            .collect(),
                        self.operation_position_color,
                    );
                }
            }
        }
        overlays.extend(setup_scene.line_vertices());
        let (cone_vertices, cone_indices) = cam_render::path_cone_mesh(
            &self.path_cones,
            12.0 * self.camera().world_per_pixel(bounds.height) / aspect.min(1.0),
        );
        ScenePrimitive {
            cone_vertices: Arc::new(cone_vertices),
            cone_indices: Arc::new(cone_indices),
            grid_vertices: Arc::new(
                grid_view::GridLayout::new(
                    self.camera(),
                    bounds.size(),
                    self.grid_size,
                    self.grid_frame,
                )
                .vertices(),
            ),
            vertices: self.vertices.clone(),
            overlay_vertices: Arc::new(overlays),
            mesh_vertices: self.mesh_vertices.clone(),
            mesh_indices: self.mesh_indices.clone(),
            transparent_stock_vertices: self.transparent_stock_vertices.clone(),
            transparent_stock_indices: self.transparent_stock_indices.clone(),
            transparent_stock_convex: self.transparent_stock_convex,
            cad_edges: self.cad_edges.clone(),
            view_proj: self.camera().view_proj(aspect),
            view_rotation: self.camera().orient,
            gizmo_view_proj: self.gizmo_camera().view_proj(1.0),
            show_gizmo: self.show_gizmo,
            gizmo_size: self.gizmo_size,
            logical_width: bounds.width,
        }
    }

    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: iced::Rectangle,
        cursor: iced::mouse::Cursor,
    ) -> iced::mouse::Interaction {
        let over = cursor.position_over(bounds).is_some();
        if (self.picking || self.set_origin || self.setup_axis || self.operation_position_picking)
            && over
        {
            // Aiming a pick: a crosshair, but a plain arrow once a snap engages so
            // its marker reads on its own. Never the orbit "hand" while picking.
            if self.snap_engaged || self.setup_hover.snap.is_some() {
                iced::mouse::Interaction::default()
            } else {
                iced::mouse::Interaction::Crosshair
            }
        } else if state.drag.is_some() {
            iced::mouse::Interaction::Grabbing
        } else if over {
            iced::mouse::Interaction::Grab
        } else {
            iced::mouse::Interaction::default()
        }
    }
}

/// A depth texture sized to the render target.
struct DepthTarget {
    view: wgpu::TextureView,
    size: (u32, u32),
    _texture: wgpu::Texture,
}

/// The shared GPU state for the viewport — iced constructs it once and hands it
/// back to us each frame. It owns both renderers (solid stock drawn first, the
/// backplot lines over it) and a depth buffer so the rotated solid occludes
/// itself correctly.
struct ViewportPipeline {
    grid: cam_render::LineRenderer,
    lines: cam_render::LineRenderer,
    overlays: cam_render::LineRenderer,
    uploaded: Option<ScenePrimitive>,
    mesh: cam_render::MeshRenderer,
    cones: cam_render::MeshRenderer,
    transparent_stock: cam_render::MeshRenderer,
    cad_edges: cam_render::LineRenderer,
    gizmo: cam_render::GizmoRenderer,
    depth: Option<DepthTarget>,
}

impl shader::Pipeline for ViewportPipeline {
    fn new(device: &wgpu::Device, queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        ViewportPipeline {
            lines: cam_render::LineRenderer::new(device, format),
            overlays: cam_render::LineRenderer::new(device, format),
            uploaded: None,
            mesh: cam_render::MeshRenderer::new(device, format),
            cones: cam_render::MeshRenderer::new_overlay(device, format),
            transparent_stock: cam_render::MeshRenderer::new_transparent(device, format),
            grid: cam_render::LineRenderer::new_depth_tested(device, format),
            cad_edges: cam_render::LineRenderer::new_depth_tested(device, format),
            gizmo: cam_render::GizmoRenderer::new(device, queue, format),
            depth: None,
        }
    }
}

impl ViewportPipeline {
    /// Ensure a depth texture matching the full render target (`size` pixels).
    fn ensure_depth(&mut self, device: &wgpu::Device, size: (u32, u32)) {
        if self.depth.as_ref().map(|d| d.size) == Some(size) {
            return;
        }
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("cam-render viewport depth"),
            size: wgpu::Extent3d {
                width: size.0,
                height: size.1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: cam_render::DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        self.depth = Some(DepthTarget {
            view,
            size,
            _texture: texture,
        });
    }
}

#[derive(Clone, Debug)]
struct ScenePrimitive {
    cone_vertices: Arc<Vec<MeshVertex>>,
    cone_indices: Arc<Vec<u32>>,
    grid_vertices: Arc<Vec<Vertex>>,
    overlay_vertices: Arc<Vec<Vertex>>,
    vertices: Arc<Vec<Vertex>>,
    mesh_vertices: Arc<Vec<MeshVertex>>,
    mesh_indices: Arc<Vec<u32>>,
    transparent_stock_vertices: Arc<Vec<MeshVertex>>,
    transparent_stock_indices: Arc<Vec<u32>>,
    transparent_stock_convex: bool,
    cad_edges: Arc<Vec<Vertex>>,
    view_proj: [[f32; 4]; 4],
    /// World-to-view rotation for camera-fixed illumination of the part and cube.
    view_rotation: [[f32; 4]; 4],
    /// The orientation cube's view-projection (same rotation, its own framing).
    gizmo_view_proj: [[f32; 4]; 4],
    /// Whether to draw the orientation cube.
    show_gizmo: bool,
    /// The cube's on-screen edge length, logical px (fixed; window-independent).
    gizmo_size: f32,
    /// The widget's logical width, to recover the physical-per-logical scale in
    /// `render` (which is handed physical `clip_bounds`) and size the cube in px.
    logical_width: f32,
}

impl shader::Primitive for ScenePrimitive {
    type Pipeline = ViewportPipeline;

    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bounds: &iced::Rectangle,
        viewport: &shader::Viewport,
    ) {
        // Retain the previous Arcs as identity tokens: keeping them alive also
        // prevents allocator address reuse from mistaking new geometry for old.
        let previous = pipeline.uploaded.as_ref();
        if previous.is_none_or(|p| p.grid_vertices != self.grid_vertices) {
            pipeline.grid.upload(device, &self.grid_vertices);
        }
        if previous.is_none_or(|p| !Arc::ptr_eq(&p.vertices, &self.vertices)) {
            pipeline.lines.upload(device, &self.vertices);
        }
        if previous.is_none_or(|p| p.overlay_vertices != self.overlay_vertices) {
            pipeline.overlays.upload(device, &self.overlay_vertices);
        }
        if previous.is_none_or(|p| {
            p.cone_vertices != self.cone_vertices || p.cone_indices != self.cone_indices
        }) {
            pipeline
                .cones
                .upload(device, &self.cone_vertices, &self.cone_indices);
        }
        if previous.is_none_or(|p| {
            !Arc::ptr_eq(&p.mesh_vertices, &self.mesh_vertices)
                || !Arc::ptr_eq(&p.mesh_indices, &self.mesh_indices)
        }) {
            pipeline
                .mesh
                .upload(device, &self.mesh_vertices, &self.mesh_indices);
        }
        if previous.is_none_or(|p| !Arc::ptr_eq(&p.cad_edges, &self.cad_edges)) {
            pipeline.cad_edges.upload(device, &self.cad_edges);
        }
        if previous.is_none_or(|p| {
            !Arc::ptr_eq(
                &p.transparent_stock_vertices,
                &self.transparent_stock_vertices,
            ) || !Arc::ptr_eq(
                &p.transparent_stock_indices,
                &self.transparent_stock_indices,
            ) || p.transparent_stock_convex != self.transparent_stock_convex
        }) {
            let upload = if self.transparent_stock_convex {
                cam_render::MeshRenderer::upload_transparent_convex
            } else {
                cam_render::MeshRenderer::upload_transparent
            };
            upload(
                &mut pipeline.transparent_stock,
                device,
                &self.transparent_stock_vertices,
                &self.transparent_stock_indices,
                self.view_proj,
            );
        } else if !self.transparent_stock_indices.is_empty()
            && previous.is_some_and(|p| p.view_proj != self.view_proj)
        {
            let resort = if self.transparent_stock_convex {
                cam_render::MeshRenderer::resort_transparent_convex
            } else {
                cam_render::MeshRenderer::resort_transparent
            };
            resort(
                &mut pipeline.transparent_stock,
                device,
                &self.transparent_stock_vertices,
                &self.transparent_stock_indices,
                self.view_proj,
            );
        }
        pipeline.uploaded = Some(self.clone());
        let logical_size = [bounds.width, bounds.height];
        pipeline
            .grid
            .set_camera(queue, self.view_proj, logical_size);
        pipeline
            .overlays
            .set_camera(queue, self.view_proj, logical_size);
        pipeline
            .transparent_stock
            .set_camera(queue, self.view_proj, self.view_rotation);
        pipeline
            .cad_edges
            .set_camera(queue, self.view_proj, logical_size);
        pipeline
            .lines
            .set_camera(queue, self.view_proj, logical_size);
        pipeline
            .mesh
            .set_camera(queue, self.view_proj, self.view_rotation);
        pipeline
            .cones
            .set_camera(queue, self.view_proj, self.view_rotation);
        pipeline
            .gizmo
            .set_camera(queue, self.gizmo_view_proj, self.view_rotation);
        let size = viewport.physical_size();
        pipeline.ensure_depth(device, (size.width.max(1), size.height.max(1)));
    }

    fn render(
        &self,
        pipeline: &Self::Pipeline,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        clip_bounds: &iced::Rectangle<u32>,
    ) {
        let Some(depth) = &pipeline.depth else {
            return;
        };
        if clip_bounds.width == 0 || clip_bounds.height == 0 {
            return;
        }
        // Our own pass: preserve iced's UI (Load), clear only our depth. We draw
        // the solid (depth-writing) then the backplot lines (always on top),
        // scissored to this widget's rectangle.
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("cam-render viewport pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &depth.view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });
        pass.set_viewport(
            clip_bounds.x as f32,
            clip_bounds.y as f32,
            clip_bounds.width as f32,
            clip_bounds.height as f32,
            0.0,
            1.0,
        );
        pass.set_scissor_rect(
            clip_bounds.x,
            clip_bounds.y,
            clip_bounds.width,
            clip_bounds.height,
        );
        pipeline.grid.draw(&mut pass);
        pipeline.mesh.draw(&mut pass);
        pipeline.cad_edges.draw(&mut pass);
        pipeline.transparent_stock.draw(&mut pass);
        pipeline.lines.draw(&mut pass);
        pipeline.cones.draw(&mut pass);
        pipeline.overlays.draw(&mut pass);
        drop(pass);

        // Second pass: the orientation cube in the top-right corner, with its own
        // depth cleared so the part never occludes it. Same rect as the click
        // hit-test (gizmo_rect), offset into the widget's clip region.
        if !self.show_gizmo {
            return;
        }
        // `clip_bounds` is physical px; the cube size is fixed in logical px.
        // Recover the physical-per-logical scale from the widget's logical width
        // so the cube stays the same on-screen size at any DPI / window size.
        let scale = if self.logical_width > 0.0 {
            clip_bounds.width as f32 / self.logical_width
        } else {
            1.0
        };
        let (lx, ly, size) = gizmo_rect(
            clip_bounds.width as f32,
            clip_bounds.height as f32,
            self.gizmo_size * scale,
            GIZMO_MARGIN * scale,
        );
        let size = size as u32;
        if size == 0 {
            return;
        }
        let gx = clip_bounds.x + lx as u32;
        let gy = clip_bounds.y + ly as u32;
        let mut gizmo_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("cam-render gizmo pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &depth.view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });
        gizmo_pass.set_viewport(gx as f32, gy as f32, size as f32, size as f32, 0.0, 1.0);
        gizmo_pass.set_scissor_rect(gx, gy, size, size);
        pipeline.gizmo.draw(&mut gizmo_pass);
    }
}

#[cfg(test)]
mod shortcut_tests {
    use super::*;
    use iced::keyboard::Modifiers;

    #[test]
    fn defaults_are_unique_and_cover_all_actions() {
        let settings = crate::Settings::default();
        let mut bindings = BTreeSet::new();
        for &(id, _, binding) in SHORTCUTS {
            assert!(bindings.insert(binding), "duplicate binding: {binding}");
            assert!(shortcut_action(id).is_some(), "missing action: {id}");
            assert_eq!(shortcut_for_binding(&settings, binding), Some(id));
        }
        assert_eq!(
            shortcut_binding(Some('p'), Modifiers::COMMAND | Modifiers::SHIFT).as_deref(),
            Some("Cmd+Shift+P")
        );
        assert!(shortcut_binding(Some('p'), Modifiers::COMMAND | Modifiers::ALT).is_none());
        assert!(shortcut_binding(Some('p'), Modifiers::NONE).is_none());
    }

    #[test]
    fn overrides_and_unassigned_bindings_survive_settings_round_trip() {
        let mut settings = crate::Settings::default();
        settings.shortcuts.insert("profile".into(), "Cmd+9".into());
        settings.shortcuts.insert("save".into(), String::new());
        let loaded: crate::Settings =
            serde_json::from_str(&serde_json::to_string(&settings).unwrap()).unwrap();
        assert_eq!(shortcut_for_binding(&loaded, "Cmd+9"), Some("profile"));
        assert_eq!(shortcut_for_binding(&loaded, "Cmd+Shift+P"), None);
        assert_eq!(shortcut_value(&loaded, "save"), "");
        assert_eq!(shortcut_for_binding(&loaded, "Cmd+S"), None);
        assert_eq!(shortcut_value(&loaded, "open"), "Cmd+O");
    }
}

#[cfg(test)]
mod startup_tests {
    use super::*;

    #[test]
    fn startup_loads_saved_geometry_setups_and_editor_state() {
        let path = std::env::temp_dir().join(format!(
            "startup setup with spaces-{}.ocam",
            std::process::id()
        ));
        let mut source = AppController::new(crate::default_machine());
        source.open_dxf(SAMPLE_DXF, "startup sample.dxf").unwrap();
        source.edit_setup(|setup| setup.name = "Top".into());
        source.add_setup();
        source.edit_setup(|setup| setup.name = "Bottom".into());
        source.save_project(&path).unwrap();

        let (app, _) = App::new_with_setup(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        assert_eq!(app.controller.current_path(), Some(path.as_path()));
        assert_eq!(app.controller.setups().len(), 2);
        assert_eq!(app.controller.setups()[0].name, "Top");
        assert_eq!(app.controller.setups()[1].name, "Bottom");
        assert_eq!(app.controller.regions().len(), source.regions().len());
        assert!(app.controller.has_geometry());
        assert_eq!(app.controller.selection(), Selection::Setup);
        assert!(app.setup_draft.is_none());
        assert!(app.status.starts_with("Opened "));
    }

    #[test]
    fn startup_returns_load_errors_before_opening_a_window() {
        let path =
            std::env::temp_dir().join(format!("invalid-startup-{}.ocam", std::process::id()));
        std::fs::write(&path, "not a project").unwrap();
        let invalid = run_with_setup(&path).unwrap_err().to_string();
        std::fs::remove_file(&path).unwrap();
        assert!(invalid.contains("project format error"), "{invalid}");
        assert!(invalid.contains(&path.display().to_string()), "{invalid}");
        let missing = run_with_setup(&path).unwrap_err().to_string();
        assert!(missing.contains("file error"), "{missing}");
    }
}

#[cfg(test)]
mod origin_move_tests {
    use super::origin_move_targets;

    #[test]
    fn a_single_origin_job_offers_nowhere_to_move() {
        // What keeps the menu section off an ordinary job: with one origin there is no
        // destination, and a "Move to Origin 1" row on the operation already in origin 1
        // would be a control that does nothing.
        assert!(origin_move_targets(&[1], Some(1), 1).is_empty());
    }

    #[test]
    fn every_other_origin_is_offered_in_tree_order() {
        // Order matters: the menu reads down the same sequence as the project tree, so
        // "the third one" means the same thing in both.
        assert_eq!(origin_move_targets(&[1, 2, 3], Some(2), 1), vec![1, 3]);
        assert_eq!(origin_move_targets(&[1, 2, 3], Some(1), 1), vec![2, 3]);
    }

    #[test]
    fn a_raised_base_is_not_assumed_to_be_origin_one() {
        // The base origin's `H<n>` is editable, so the indices need not start at 1 and
        // "the base" is whatever the setup says. Hard-coding 1 anywhere here would offer
        // an operation a move to the group it is already in.
        assert_eq!(origin_move_targets(&[4, 5], Some(4), 4), vec![5]);
        assert_eq!(origin_move_targets(&[4, 5], Some(5), 4), vec![4]);
    }

    #[test]
    fn an_operation_pointing_at_a_deleted_origin_counts_as_being_on_the_base() {
        // Deleting an origin leaves its operations carrying an index that no longer
        // exists; the tree draws them under the base. If that stale index were read
        // literally the base would be offered as a destination for a row already shown
        // under the base — a move that appears to do nothing.
        assert_eq!(origin_move_targets(&[1, 3], Some(2), 1), vec![3]);
        // Same for an operation with no origin recorded at all.
        assert_eq!(origin_move_targets(&[1, 3], None, 1), vec![3]);
    }
}

#[cfg(test)]
mod multi_setup_draft_tests {
    use super::*;

    fn send(app: &mut App, message: Message) {
        let _task = app.update(message);
    }

    #[test]
    fn saved_profile_selection_allows_switching_operations_without_escape() {
        let path = std::env::temp_dir().join(format!(
            "saved-profile-selection-{}.ocam",
            std::process::id()
        ));
        let mut source = AppController::new(crate::default_machine());
        source.open_dxf(SAMPLE_DXF, "profiles.dxf").unwrap();
        source.new_operation(OpKind::Profile);
        source.new_operation(OpKind::Profile);
        let second = source.document().setup().operations[1].id();
        source.edit_setup(|setup| {
            setup.stock_to_leave.insert(
                second,
                cam_model::StockToLeave {
                    enabled: true,
                    radial: 0.25,
                    axial: 0.1,
                },
            );
        });
        source.save_project(&path).unwrap();
        let (mut app, _) = App::new();
        let loaded = app.open_project(&path);
        std::fs::remove_file(&path).unwrap();
        loaded.unwrap();
        let operations = &app.controller.inspection_setup().operations;
        let first = operations[0].id();
        let second = operations[1].id();
        let before = app.controller.document().clone();
        send(&mut app, Message::ClickOp(second));
        send(&mut app, Message::ClickOp(first));
        assert_eq!(app.controller.selection(), Selection::Operation(first));
        send(&mut app, Message::Select(Selection::Operation(first)));
        send(&mut app, Message::CancelOperationDraft);
        send(
            &mut app,
            Message::Tree(TreeMessage::Press(TreeNode::Operation(second))),
        );
        send(&mut app, Message::Tree(TreeMessage::Release));
        assert_eq!(app.controller.selection(), Selection::Operation(second));
        assert!(
            !app.inspector_dirty(),
            "an untouched selection must be clean"
        );
        assert!(
            !app.any_field_invalid(),
            "{:?}",
            app.resolve_height_editor(second)
        );
        assert_eq!(
            app.stock_to_leave_value(),
            before.setup().stock_to_leave_for(second)
        );
        send(
            &mut app,
            Message::Tree(TreeMessage::Press(TreeNode::Operation(first))),
        );
        send(&mut app, Message::Tree(TreeMessage::Release));
        assert_eq!(
            app.controller.selection(),
            Selection::Operation(first),
            "{}",
            app.status
        );
        assert!(!app.inspector_dirty());
        assert_eq!(
            app.stock_to_leave_value(),
            before.setup().stock_to_leave_for(first)
        );
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn setup_selection_and_context_menu_do_not_start_editing() {
        let (mut app, _) = App::new();
        assert!(app.setup_draft.is_none());
        assert!(app.pane_handle(Pane::Inspector).is_none());
        send(&mut app, Message::OpenSample);
        let id = app.controller.active_setup_id();
        send(&mut app, Message::SelectSetup(id));
        send(
            &mut app,
            Message::Tree(TreeMessage::Open(TreeNode::Setup(id))),
        );
        assert_eq!(app.tree_menu, Some(TreeNode::Setup(id)));
        assert!(app.setup_draft.is_none());
        assert!(app.pane_handle(Pane::Inspector).is_none());

        send(
            &mut app,
            Message::Tree(TreeMessage::Command(Box::new(Message::EditSetup(id)))),
        );
        assert!(app.setup_draft.is_some());
        assert!(app.pane_handle(Pane::Inspector).is_some());
        send(&mut app, Message::SetupNameChanged("Explicit edit".into()));
        send(&mut app, Message::BeginOp(OpKind::Profile));
        assert!(app.controller.pending_op().is_none());
        assert!(app.setup_draft_dirty());
        assert!(app.pane_handle(Pane::Inspector).is_some());
        send(&mut app, Message::SetupCancel);
        assert!(app.setup_draft.is_none());
        assert!(!app.setup_draft_dirty());
        assert!(app.pane_handle(Pane::Inspector).is_none());

        send(&mut app, Message::EditSetup(id));
        send(&mut app, Message::SetupNameChanged("Applied edit".into()));
        send(&mut app, Message::SetupCommit);
        assert_eq!(app.controller.inspection_setup().name, "Applied edit");
        assert!(app.setup_draft.is_none());
        assert!(app.pane_handle(Pane::Inspector).is_none());
    }

    #[test]
    fn pattern_editor_create_apply_and_cancel_are_transactional() {
        let (mut app, _) = App::new();
        send(&mut app, Message::OpenSample);
        let operation = app.controller.inspection_setup().operations[0].id();
        app.focus_ops.insert(operation);
        send(&mut app, Message::BeginPattern(false));
        assert_eq!(app.pattern_draft.as_ref().unwrap().members, vec![operation]);
        assert!(app.pane_handle(Pane::Inspector).is_some());
        send(&mut app, Message::ClickOp(operation));
        assert!(
            app.pattern_draft.is_some(),
            "selection changes do not discard a draft"
        );
        send(&mut app, Message::RunJob);
        assert!(
            app.controller.outcome().is_none(),
            "generation is blocked until the draft is resolved"
        );
        send(&mut app, Message::PatternValue(0, "1".into()));
        send(&mut app, Message::PatternCommit);
        assert!(app.pattern_draft.is_some(), "invalid drafts remain open");
        assert!(app.controller.inspection_setup().patterns.is_empty());
        send(&mut app, Message::PatternValue(0, "2".into()));
        send(&mut app, Message::PatternCommit);
        let pattern = app.controller.inspection_setup().patterns[0].clone();
        assert_eq!(pattern.members, vec![operation]);
        assert_eq!(
            pattern.kind,
            cam_model::PatternKind::Rectangular {
                nx: 2,
                ny: 1,
                dx: 20.0,
                dy: 0.0
            }
        );
        assert!(app.pattern_draft.is_none());

        app.inspector_enabled = false;
        send(
            &mut app,
            Message::Tree(TreeMessage::DoubleClick(TreeNode::Pattern(pattern.id))),
        );
        assert!(app.pane_handle(Pane::Inspector).is_some());
        send(&mut app, Message::PatternValue(2, "35".into()));
        send(&mut app, Message::PatternCancel);
        assert_eq!(
            app.controller.inspection_setup().patterns[0].kind,
            pattern.kind
        );

        send(&mut app, Message::EditPattern(pattern.id));
        send(&mut app, Message::PatternValue(2, "35".into()));
        send(&mut app, Message::PatternCommit);
        assert_eq!(
            app.controller.inspection_setup().patterns[0].kind,
            cam_model::PatternKind::Rectangular {
                nx: 2,
                ny: 1,
                dx: 35.0,
                dy: 0.0
            }
        );
        assert!(app.controller.undo());
        assert_eq!(
            app.controller.inspection_setup().patterns[0].kind,
            pattern.kind
        );
    }

    #[test]
    fn circular_pattern_draft_defaults_to_the_active_origin_in_setup_xy() {
        let (mut app, _) = App::new();
        send(&mut app, Message::OpenSample);
        app.controller.add_origin();
        app.controller
            .edit_active_origin(|origin| *origin = [17.25, -8.5, 0.0]);
        let setup = app.controller.inspection_setup();
        let expected = setup.origin_position(app.controller.active_origin());
        send(&mut app, Message::BeginPattern(true));
        let draft = app.pattern_draft.as_ref().unwrap();
        assert_eq!(draft.values[1].parse::<f64>().unwrap(), expected[0]);
        assert_eq!(draft.values[2].parse::<f64>().unwrap(), expected[1]);
        send(&mut app, Message::PatternCancel);
    }

    #[test]
    fn cancelling_new_operations_never_leaves_a_hidden_setup_edit() {
        let (mut app, _) = App::new();
        send(&mut app, Message::OpenSample);
        let id = app.controller.active_setup_id();
        send(&mut app, Message::EditSetup(id));
        for cancel in [Message::CancelOp, Message::CadEscape] {
            send(&mut app, Message::BeginOp(OpKind::Profile));
            assert!(app.creating_operation(), "{}", app.status);
            assert!(app.setup_draft.is_none());
            send(&mut app, cancel);
            assert!(!app.creating_operation());
            assert!(app.setup_draft.is_none());
            assert!(!app.setup_draft_dirty());
            assert!(app.pane_handle(Pane::Inspector).is_none());
        }
        send(&mut app, Message::Tree(TreeMessage::Open(TreeNode::Setups)));
        send(
            &mut app,
            Message::Tree(TreeMessage::Command(Box::new(Message::SetupNew))),
        );
        assert_ne!(app.controller.active_setup_id(), id);
        assert_eq!(app.active_tab, RibbonTab::Machining);
        assert_eq!(app.machining_tab, RibbonTab::Machining);
        assert!(app.setup_draft.is_some());
        assert!(app.pane_handle(Pane::Inspector).is_some());
    }

    #[test]
    fn cancelling_cad_operations_does_not_reactivate_the_setup_editor() {
        let mut app = model_list_app();
        for cancel in [Message::CancelOp, Message::CadEscape] {
            send(&mut app, Message::BeginOp(OpKind::Face));
            assert_eq!(app.cad_pending_kind, Some(OpKind::Face), "{}", app.status);
            assert!(app.setup_draft.is_none());
            send(&mut app, cancel);
            assert!(!app.creating_operation());
            assert!(app.setup_draft.is_none());
            assert!(!app.setup_draft_dirty());
            assert!(app.pane_handle(Pane::Inspector).is_none());
        }
        let id = app.controller.active_setup_id();
        send(&mut app, Message::EditSetup(id));
        assert!(app.setup_draft.is_some(), "{}", app.status);
        assert!(app.pane_handle(Pane::Inspector).is_some());
    }

    fn blank_app() -> App {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.add_setup();
        app.begin_setup_draft();
        app
    }

    fn carve_app() -> (App, u32) {
        let mut app = blank_app();
        let vbit = app
            .library
            .tools
            .iter()
            .find(|tool| matches!(tool.kind, ToolKind::VBit { .. }))
            .cloned()
            .expect("starter library includes a V-bit");
        let tool = app.controller.use_tool(vbit);
        app.controller
            .add_operation(Operation::Carve(cam_model::CarveOp {
                parameter_expressions: Default::default(),
                id: 0,
                tool,
                clear: None,
                boundary: cam_geo::Contour::new(vec![
                    cam_geo::Point::new(0.0, 0.0),
                    cam_geo::Point::new(10.0, 0.0),
                    cam_geo::Point::new(10.0, 10.0),
                ]),
                islands: Vec::new(),
                top: 0.0,
                depth: 1.0,
                offset: 0.0,
                ring_step: 0.0,
                scallop: 0.0,
                spindle_rpm: 0.0,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                feed: 300.0,
                plunge_feed: 100.0,
                plunge: Plunge::Straight,
                stay_down: false,
                end: None,
                start: None,
            }));
        let id = app.controller.inspection_setup().operations[0].id();
        app.begin_operation_draft(id);
        (app, id)
    }

    fn stock_profile_app() -> (App, u32) {
        let mut app = blank_app();
        let operation = app.controller.operation_template(OpKind::Profile, 1);
        app.controller.add_operation(operation);
        let id = app.controller.inspection_setup().operations[0].id();
        app.begin_operation_draft(id);
        (app, id)
    }

    #[test]
    fn rotary_wrap_editor_stages_numbers_and_commits_all_tabs_in_one_undo_step() {
        let (mut app, id) = stock_profile_app();
        // The proposed radius plus offset must fit inside a physical blank.
        app.controller.edit_stock(|stock| {
            *stock = cam_model::Stock::FixedRound {
                diameter: 200.,
                inner_diameter: 0.,
                length: 60.,
                position: 0.5,
                tube: false,
                axis: Some(cam_model::StockAxis::Origin {
                    origin_axis: cam_model::StockOriginAxis::X,
                }),
            }
        });
        let before = app.controller.document().clone();
        send(&mut app, Message::Rotary(rotary_view::Action::Enable(true)));
        let field = Field::Wrap(rotary_view::Number::Offset);
        assert!(app.inspector_fields().contains(&field));
        send(&mut app, Message::FieldChanged(field, "40".into()));
        assert!(app
            .controller
            .operation(id)
            .unwrap()
            .rotary_wrap()
            .is_none());
        assert!(!app.any_field_invalid(), "{}", app.status);
        send(&mut app, Message::Apply);
        assert_eq!(
            app.controller
                .operation(id)
                .unwrap()
                .rotary_wrap()
                .unwrap()
                .offset,
            40.,
            "{}",
            app.status
        );
        assert!(app.controller.undo());
        assert_eq!(*app.controller.document(), before);
        app.begin_operation_draft(id);
        send(&mut app, Message::Rotary(rotary_view::Action::Enable(true)));
        send(&mut app, Message::CancelOperationDraft);
        assert_eq!(*app.controller.document(), before);
    }

    #[test]
    fn rotary_mode_switch_preserves_wrap_and_existing_tab_settings() {
        let (mut app, _) = stock_profile_app();
        let heights = app.height_draft.clone();
        let offsets = app.height_offsets.clone();
        app.edit_operation_draft(|op| {
            op.set_rotary_wrap(Some(cam_model::RotaryWrap {
                radius: 100.,
                unwrap_geometry: false,
                ..Default::default()
            }))
        });
        send(
            &mut app,
            Message::Rotary(rotary_view::Action::Enable(false)),
        );
        assert!(app
            .operation_draft
            .as_ref()
            .unwrap()
            .rotary_wrap()
            .is_none());
        send(&mut app, Message::Rotary(rotary_view::Action::Enable(true)));
        let wrap = app.operation_draft.as_ref().unwrap().rotary_wrap().unwrap();
        assert_eq!(wrap.radius, 100.);
        assert!(!wrap.unwrap_geometry);
        assert_eq!(app.height_draft, heights);
        assert_eq!(app.height_offsets, offsets);
    }

    #[test]
    fn rotary_wrap_creation_requires_real_boundaries_and_rejects_invalid_offset() {
        let (mut app, _) = stock_profile_app();
        send(&mut app, Message::Rotary(rotary_view::Action::Enable(true)));
        send(
            &mut app,
            Message::FieldChanged(Field::Wrap(rotary_view::Number::Offset), "-25".into()),
        );
        assert!(app.any_field_invalid());
        app.cad_pending_kind = Some(OpKind::Pocket);
        app.seed_creation_editor(OpKind::Pocket);
        send(&mut app, Message::Rotary(rotary_view::Action::Enable(true)));
        assert!(!app.creation_ready());
    }

    #[test]
    fn parallel_editor_updates_scan_controls_and_preserves_draft_cancel() {
        let mut app = blank_app();
        let operation = app.controller.operation_template(OpKind::Parallel, 1);
        let id = operation.id();
        assert_eq!(
            families_for(OpKind::Parallel),
            &[crate::ToolKindPick::BallMill]
        );
        for field in [
            Field::ParallelStepover,
            Field::ParallelAngle,
            Field::AdaptiveTolerance,
        ] {
            assert!(operation_fields(&operation).contains(&field));
            assert_eq!(operation_tabs::field_tab(field), OperationTab::Passes);
        }
        assert!(operation_tabs::operation_field_invalid(
            &operation,
            Field::ParallelStepover,
            0.
        ));
        assert!(!operation_tabs::operation_field_invalid(
            &operation,
            Field::ParallelAngle,
            -45.
        ));
        app.controller.add_operation(operation);
        app.begin_operation_draft(id);
        send(
            &mut app,
            Message::FieldChanged(Field::ParallelStepover, "0.3".into()),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::ParallelAngle, "35".into()),
        );
        send(&mut app, Message::ParallelBothWaysToggled(false));
        assert!(
            matches!(app.current_operation(id), Some(Operation::Parallel(p)) if !p.params.both_ways)
        );
        send(&mut app, Message::CancelOperationDraft);
        assert!(
            matches!(app.controller.operation(id), Some(Operation::Parallel(p)) if p.params.both_ways && p.params.angle == 0.)
        );
    }

    #[test]
    fn spiral_editor_picks_center_and_applies_or_cancels_pass_settings() {
        use cam_model::{ScallopRetraction, SpiralMode, SpiralOrder};
        let mut app = blank_app();
        let operation = app.controller.operation_template(OpKind::Spiral, 1);
        let id = operation.id();
        assert_eq!(
            families_for(OpKind::Spiral),
            &[crate::ToolKindPick::BallMill]
        );
        assert_eq!(
            operation_tabs::field_tab(Field::SpiralInnerLimit),
            OperationTab::Passes
        );
        assert_eq!(
            operation_tabs::field_tab(Field::SpiralFromSlope),
            OperationTab::Geometry
        );
        assert_eq!(
            operation_tabs::field_tab(Field::SpiralStayDownDistance),
            OperationTab::Linking
        );
        app.controller.add_operation(operation);
        app.begin_operation_draft(id);
        send(
            &mut app,
            Message::OperationPositionPick(operation_positions::PositionKind::Entry),
        );
        send(&mut app, Message::OperationPositionPicked([6., 7., 2.]));
        send(
            &mut app,
            Message::SpiralModeChanged(SpiralMode::SpiralWithCircles),
        );
        send(
            &mut app,
            Message::SpiralOrderChanged(SpiralOrder::OutsideIn),
        );
        send(&mut app, Message::SpiralClockwiseToggled(true));
        send(
            &mut app,
            Message::ScallopRetractionChanged(ScallopRetraction::Shortest),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::SpiralInnerLimit, "1.5".into()),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::SpiralOuterLimit, "8".into()),
        );
        send(&mut app, Message::Apply);
        let Some(Operation::Spiral(saved)) = app.controller.operation(id) else {
            panic!("Spiral operation is missing");
        };
        assert_eq!(saved.params.center, Some([6., 7.]));
        assert_eq!(saved.params.mode, SpiralMode::SpiralWithCircles);
        assert_eq!(saved.params.order, SpiralOrder::OutsideIn);
        assert_eq!(saved.params.retraction, ScallopRetraction::Shortest);
        assert_eq!(
            [saved.params.inner_limit, saved.params.outer_limit],
            [1.5, 8.]
        );
        assert!(saved.params.clockwise);
        for tab in [
            OperationTab::Tool,
            OperationTab::Geometry,
            OperationTab::Heights,
            OperationTab::Passes,
            OperationTab::Linking,
        ] {
            app.operation_tab = tab;
            let _view = app.operation_inspector(id);
        }
        app.begin_operation_draft(id);
        send(
            &mut app,
            Message::OperationPositionClear(operation_positions::PositionKind::Entry),
        );
        send(&mut app, Message::CancelOperationDraft);
        assert_eq!(
            app.controller
                .operation(id)
                .and_then(operation_tabs::operation_start),
            Some([6., 7.])
        );
        assert!(app.controller.undo());
        assert_eq!(
            app.controller
                .operation(id)
                .and_then(operation_tabs::operation_start),
            None
        );
    }

    #[test]
    fn scallop_mode_is_a_saved_and_cancelable_finishing_choice() {
        let mut app = blank_app();
        let operation = app.controller.operation_template(OpKind::Parallel, 1);
        let id = operation.id();
        app.controller.add_operation(operation);
        app.begin_operation_draft(id);
        send(&mut app, Message::ParallelScallopToggled(true));
        let draft = app.current_operation(id).unwrap();
        assert!(matches!(draft, Operation::Parallel(p) if p.params.scallop));
        assert!(!operation_fields(draft).contains(&Field::ParallelAngle));
        assert!(!operation_fields(draft).contains(&Field::LiftHeight));
        send(&mut app, Message::CancelOperationDraft);
        assert!(
            matches!(app.controller.operation(id), Some(Operation::Parallel(p)) if !p.params.scallop)
        );
    }

    #[test]
    fn scallop_linking_and_smoothing_controls_apply_and_undo() {
        let mut app = blank_app();
        let mut operation = app.controller.operation_template(OpKind::Parallel, 1);
        if let Operation::Parallel(p) = &mut operation {
            p.params.scallop = true;
        }
        let id = operation.id();
        app.controller.add_operation(operation);
        app.begin_operation_draft(id);
        send(&mut app, Message::ScallopSmoothingToggled(true));
        send(
            &mut app,
            Message::ScallopOrderChanged(cam_model::ScallopOrder::InsideOut),
        );
        send(
            &mut app,
            Message::ScallopRetractionChanged(cam_model::ScallopRetraction::Shortest),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::ScallopSmoothingTolerance, "0.07".into()),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::ScallopSafeDistance, "0.8".into()),
        );
        send(
            &mut app,
            Message::OperationPositionPick(operation_positions::PositionKind::Entry),
        );
        send(&mut app, Message::OperationPositionPicked([3., 4., 2.]));
        let draft = app.current_operation(id).unwrap();
        assert!(operation_fields(draft).contains(&Field::ScallopSmoothingTolerance));
        assert_eq!(operation_tabs::operation_start(draft), Some([3., 4.]));
        send(&mut app, Message::Apply);
        assert!(
            matches!(app.controller.operation(id), Some(Operation::Parallel(p))
            if p.params.scallop_smoothing && p.params.scallop_smoothing_tolerance == 0.07
                && p.params.safe_distance == 0.8 && p.params.scallop_start == Some([3., 4.])
                && p.params.scallop_order == cam_model::ScallopOrder::InsideOut
                && p.params.scallop_retraction == cam_model::ScallopRetraction::Shortest),
            "{}",
            app.status
        );
        assert!(app.controller.undo());
        assert!(
            matches!(app.controller.operation(id), Some(Operation::Parallel(p))
            if !p.params.scallop_smoothing && p.params.scallop_start.is_none()
                && p.params.scallop_order == cam_model::ScallopOrder::OutsideIn
                && p.params.scallop_retraction == cam_model::ScallopRetraction::Full)
        );
    }

    #[test]
    fn parallel_transition_type_applies_cancels_and_undoes() {
        use cam_model::ParallelTransition;
        let mut app = blank_app();
        let operation = app.controller.operation_template(OpKind::Parallel, 1);
        let id = operation.id();
        app.controller.add_operation(operation);
        let mode = |op: &Operation| match op {
            Operation::Parallel(p) => p.params.transition,
            _ => unreachable!(),
        };
        app.begin_operation_draft(id);
        app.operation_tab = OperationTab::Linking;
        let _ = app.operation_inspector(id);
        send(&mut app, Message::ParallelTransitionChanged(ParallelTransition::NoContact));
        assert_eq!(mode(app.current_operation(id).unwrap()), ParallelTransition::NoContact);
        send(&mut app, Message::CancelOperationDraft);
        assert_eq!(mode(app.controller.operation(id).unwrap()), ParallelTransition::StraightLine);
        app.begin_operation_draft(id);
        send(&mut app, Message::ParallelTransitionChanged(ParallelTransition::Smooth));
        send(&mut app, Message::Apply);
        assert_eq!(mode(app.controller.operation(id).unwrap()), ParallelTransition::Smooth);
        assert!(app.controller.undo());
        assert_eq!(mode(app.controller.operation(id).unwrap()), ParallelTransition::StraightLine);
    }

    #[test]
    fn parallel_retraction_policy_supports_apply_cancel_and_undo() {
        use cam_model::ScallopRetraction;
        let mut app = blank_app();
        let operation = app.controller.operation_template(OpKind::Parallel, 1);
        let id = operation.id();
        assert!(!operation_fields(&operation).contains(&Field::LiftHeight));
        app.controller.add_operation(operation);
        let minimum = |op: &Operation| {
            matches!(op, Operation::Parallel(p) if p.params.retraction == ScallopRetraction::Minimum)
        };
        app.begin_operation_draft(id);
        send(&mut app, Message::ScallopRetractionChanged(ScallopRetraction::Minimum));
        assert!(minimum(app.current_operation(id).unwrap()));
        assert!(operation_fields(app.current_operation(id).unwrap()).contains(&Field::ScallopSafeDistance));
        send(&mut app, Message::CancelOperationDraft);
        assert!(!minimum(app.controller.operation(id).unwrap()));
        app.begin_operation_draft(id);
        send(&mut app, Message::ScallopRetractionChanged(ScallopRetraction::Minimum));
        send(&mut app, Message::Apply);
        assert!(minimum(app.controller.operation(id).unwrap()), "{}", app.status);
        assert!(app.controller.undo());
        assert!(!minimum(app.controller.operation(id).unwrap()));
    }

    #[test]
    fn adaptive_arc_sizes_are_linking_fields_and_can_be_edited_independently() {
        let mut app = blank_app();
        let mut operation = app.controller.operation_template(OpKind::Adaptive, 1);
        let Operation::Adaptive(op) = &mut operation else {
            unreachable!()
        };
        op.params.feed = 300.;
        op.params.plunge_feed = 100.;
        let id = operation.id();
        for field in [Field::LeadInSize, Field::LeadOutSize] {
            assert!(operation_fields(&operation).contains(&field));
            assert_eq!(operation_tabs::field_tab(field), OperationTab::Linking);
            assert_eq!(op_field(&operation, field), Some(0.6));
            assert!(!operation_tabs::operation_field_invalid(&operation, field, 0.));
            assert!(operation_tabs::operation_field_invalid(&operation, field, -1.));
        }
        app.controller.add_operation(operation);
        app.begin_operation_draft(id);
        send(&mut app, Message::FieldChanged(Field::LeadInSize, "0.9".into()));
        send(&mut app, Message::FieldChanged(Field::LeadOutSize, "0".into()));
        send(&mut app, Message::Apply);
        let applied = app.controller.operation(id).unwrap();
        assert_eq!(op_field(applied, Field::LeadInSize), Some(0.9));
        assert_eq!(op_field(applied, Field::LeadOutSize), Some(0.));
    }

    #[test]
    fn lift_height_editor_supports_validation_apply_cancel_and_undo() {
        for kind in [OpKind::Pocket, OpKind::Adaptive] {
            let mut app = blank_app();
            let mut operation = app.controller.operation_template(kind, 1);
            if let Operation::Adaptive(op) = &mut operation {
                op.params.feed = 300.;
                op.params.plunge_feed = 100.;
            }
            let id = operation.id();
            let field = Field::LiftHeight;
            let initial = 0.2;
            assert!(operation_fields(&operation).contains(&field));
            assert_eq!(operation_tabs::field_tab(field), OperationTab::Linking);
            assert_eq!(op_field(&operation, field), Some(initial));
            for value in [-1., f64::NAN, f64::INFINITY] {
                assert!(operation_tabs::operation_field_invalid(
                    &operation, field, value
                ));
            }
            assert!(!operation_tabs::operation_field_invalid(
                &operation, field, 0.
            ));
            assert!(!operation_tabs::operation_field_invalid(
                &operation, field, 2.
            ));
            app.controller.add_operation(operation);
            app.begin_operation_draft(id);
            send(&mut app, Message::FieldChanged(field, "2".into()));
            assert_eq!(
                op_field(app.controller.operation(id).unwrap(), field),
                Some(initial)
            );
            send(&mut app, Message::CancelOperationDraft);
            app.begin_operation_draft(id);
            assert_eq!(
                app.fields.get(&field).map(String::as_str),
                Some("0.2")
            );
            send(&mut app, Message::FieldChanged(field, "2".into()));
            send(&mut app, Message::Apply);
            assert_eq!(
                op_field(app.controller.operation(id).unwrap(), field),
                Some(2.),
                "{}",
                app.status
            );
            assert!(app.controller.undo());
            assert_eq!(
                op_field(app.controller.operation(id).unwrap(), field),
                Some(initial)
            );
            assert!(app.controller.redo());
            assert_eq!(
                op_field(app.controller.operation(id).unwrap(), field),
                Some(2.)
            );
        }
    }

    #[test]
    fn adaptive_tolerance_is_editable_in_passes_and_supports_apply_cancel_and_undo() {
        let mut app = blank_app();
        let mut operation = app.controller.operation_template(OpKind::Adaptive, 1);
        if let Operation::Adaptive(op) = &mut operation {
            op.params.feed = 300.;
            op.params.plunge_feed = 100.;
        }
        let id = operation.id();
        let field = Field::AdaptiveTolerance;
        assert!(operation_fields(&operation).contains(&field));
        assert_eq!(operation_tabs::field_tab(field), OperationTab::Passes);
        assert_eq!(op_field(&operation, field), Some(0.1));
        assert!(operation_tabs::operation_field_invalid(
            &operation, field, 0.0
        ));
        assert!(operation_tabs::operation_field_invalid(
            &operation, field, -0.001
        ));
        app.controller.add_operation(operation);
        app.begin_operation_draft(id);
        send(&mut app, Message::FieldChanged(field, "0.025".into()));
        assert_eq!(
            op_field(app.controller.operation(id).unwrap(), field),
            Some(0.1)
        );
        send(&mut app, Message::CancelOperationDraft);
        assert_eq!(
            op_field(app.controller.operation(id).unwrap(), field),
            Some(0.1)
        );
        app.begin_operation_draft(id);
        send(&mut app, Message::FieldChanged(field, "0.025".into()));
        send(&mut app, Message::Apply);
        assert_eq!(
            op_field(app.controller.operation(id).unwrap(), field),
            Some(0.025),
            "{}",
            app.status
        );
        assert!(app.controller.undo());
        assert_eq!(
            op_field(app.controller.operation(id).unwrap(), field),
            Some(0.1)
        );
        assert!(app.controller.redo());
        assert_eq!(
            op_field(app.controller.operation(id).unwrap(), field),
            Some(0.025)
        );
    }

    #[test]
    fn stock_to_leave_edit_is_staged_until_apply_and_cancel_discards_it() {
        let (mut app, id) = stock_profile_app();
        let before = app.controller.inspection_setup().clone();
        send(&mut app, Message::StockToLeaveToggled(true));
        assert_eq!(
            app.stock_to_leave_with_fields(),
            cam_model::StockToLeave {
                enabled: true,
                radial: 0.5,
                axial: 0.5,
            }
        );
        send(
            &mut app,
            Message::FieldChanged(Field::StockLeaveRadial, "0.3".into()),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::StockLeaveAxial, "0.2".into()),
        );
        send(&mut app, Message::StockToLeaveToggled(false));
        send(&mut app, Message::StockToLeaveToggled(true));
        assert_eq!(app.stock_to_leave_with_fields().radial, 0.3);
        assert_eq!(app.stock_to_leave_with_fields().axial, 0.2);
        assert_eq!(app.controller.inspection_setup(), &before);
        assert!(app.inspector_dirty());

        send(&mut app, Message::CancelOperationDraft);
        assert_eq!(app.controller.inspection_setup(), &before);
        assert_eq!(
            app.controller.inspection_setup().stock_to_leave_for(id),
            Default::default()
        );

        send(&mut app, Message::StockToLeaveToggled(true));
        send(
            &mut app,
            Message::FieldChanged(Field::StockLeaveRadial, "0.5".into()),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::StockLeaveAxial, "0.2".into()),
        );
        send(&mut app, Message::Apply);
        assert_eq!(
            app.controller.inspection_setup().stock_to_leave_for(id),
            cam_model::StockToLeave {
                enabled: true,
                radial: 0.5,
                axial: 0.2
            }
        );
        assert_eq!(app.stock_to_leave_value().radial, 0.5);
        assert!(app.controller.undo());
        assert_eq!(
            app.controller.inspection_setup().stock_to_leave_for(id),
            Default::default()
        );
    }

    #[test]
    fn face_stock_to_leave_defaults_to_axial_only() {
        let mut app = blank_app();
        let operation = app.controller.operation_template(OpKind::Face, 1);
        app.controller.add_operation(operation);
        let id = app.controller.inspection_setup().operations[0].id();
        app.begin_operation_draft(id);
        send(&mut app, Message::StockToLeaveToggled(true));
        assert_eq!(
            app.stock_to_leave_with_fields(),
            cam_model::StockToLeave {
                enabled: true,
                radial: 0.0,
                axial: 0.5,
            }
        );
    }

    #[test]
    fn operation_stock_allowance_follows_duplicates_and_is_removed_on_delete() {
        let (mut app, id) = stock_profile_app();
        let allowance = cam_model::StockToLeave {
            enabled: true,
            radial: 0.3,
            axial: 0.2,
        };
        app.controller.edit_setup(|setup| {
            setup.stock_to_leave.insert(id, allowance);
        });
        app.controller.duplicate_selected_operation();
        let Selection::Operation(copy_id) = app.controller.selection() else {
            panic!("duplicate should be selected");
        };
        assert_eq!(
            app.controller
                .inspection_setup()
                .stock_to_leave_for(copy_id),
            allowance
        );
        app.controller.delete_selected_operation();
        assert_eq!(
            app.controller
                .inspection_setup()
                .stock_to_leave_for(copy_id),
            Default::default()
        );
        assert_eq!(
            app.controller.inspection_setup().stock_to_leave_for(id),
            allowance
        );

        let setup_copy = app
            .controller
            .duplicate_setup(app.controller.active_setup_id())
            .unwrap();
        let duplicate_setup = app
            .controller
            .setups()
            .iter()
            .find(|s| s.id == setup_copy)
            .unwrap();
        let copied_id = duplicate_setup.operations[0].id();
        assert_eq!(duplicate_setup.stock_to_leave_for(copied_id), allowance);
    }

    fn model_list_app() -> App {
        use cam_model::*;
        let mut app = blank_app();
        app.controller
            .add_cad_asset(
                GeometryAsset {
                    id: 1,
                    revision: 1,
                    name: "Model".into(),
                    source_format: CadFormat::Step,
                    source_data: vec![].into(),
                    import_options: None,
                    import_diagnostics: vec![],
                    source_fingerprint: String::new(),
                    source_units: SourceUnits::Millimetres,
                    backend: "analytic test fixture".into(),
                    topology: TopologySnapshot {
                        bodies: (1..=3)
                            .map(|id| CadBody {
                                id,
                                name: format!("Body {id}"),
                                valid_solid: true,
                                bounds: [[0.0, 0.0, 0.0], [10.0, 10.0, 10.0]],
                            })
                            .collect(),
                        ..Default::default()
                    }
                    .into(),
                },
                (1..=3)
                    .map(|id| GeometryInstance {
                        id,
                        asset_id: 1,
                        body_id: id,
                        name: format!("Body {id}"),
                        placement: RigidTransform {
                            translation: [(id - 1) as f64 * 20., 0., 0.],
                            ..Default::default()
                        },
                        visible: true,
                    })
                    .collect(),
            )
            .unwrap();
        app.begin_setup_draft();
        app
    }

    #[test]
    fn fixture_body_picking_clearances_and_role_changes_share_the_setup_draft() {
        let mut app = model_list_app();
        send(&mut app, Message::SetupModelToggled(1, true));
        send(&mut app, Message::SetupCommit);
        app.begin_setup_draft();
        let before = app.controller.document().clone();
        send(&mut app, Message::SetupFixturePick);
        assert!(app.setup_fixture_pick && app.setup_model_pick);
        assert_eq!(app.machining_viewport().cad_filter, EntityKind::Body);
        for instance in [1, 2, 2, 2] {
            send(
                &mut app,
                Message::CadPicked(vec![PickCandidate {
                    entity: EntityRef {
                        asset_id: 1,
                        revision: 1,
                        instance_id: instance,
                        body_id: instance,
                        kind: EntityKind::Body,
                        id: instance,
                    },
                    depth: 0.,
                    occluded: false,
                }]),
            );
        }
        let draft = app.setup_draft.as_ref().unwrap();
        assert_eq!(draft.fixture_instances, vec![1, 2]);
        assert!(draft.model_instances.is_empty());
        send(
            &mut app,
            Message::SetupFixtureClearanceChanged(0, "2.5".into()),
        );
        send(
            &mut app,
            Message::SetupFixtureClearanceChanged(1, "0".into()),
        );
        send(&mut app, Message::CadEscape);
        assert!(!app.setup_fixture_pick && !app.setup_model_pick);
        assert_eq!(app.controller.document(), &before);
        send(&mut app, Message::SetupCancel);
        assert_eq!(app.controller.document(), &before);
        app.begin_setup_draft();
        send(&mut app, Message::SetupFixtureToggled(2, true));
        send(
            &mut app,
            Message::SetupFixtureClearanceChanged(0, "2.5".into()),
        );
        send(
            &mut app,
            Message::SetupFixtureClearanceChanged(1, "0".into()),
        );
        send(&mut app, Message::SetupCommit);
        assert_eq!(app.controller.inspection_setup().fixture_instances, vec![2]);
        assert_eq!(
            app.controller.inspection_setup().fixture_clearance.radial,
            2.5
        );
        assert_eq!(
            app.controller.inspection_setup().fixture_clearance.axial,
            0.
        );
        assert!(!app.viewport_hidden_instances().contains(&2));
        assert!(app.controller.undo());
        assert_eq!(app.controller.document(), &before);
        assert!(app.controller.redo());
        app.begin_setup_draft();
        send(&mut app, Message::SetupModelToggled(2, true));
        assert!(app
            .setup_draft
            .as_ref()
            .unwrap()
            .fixture_instances
            .is_empty());
    }

    #[test]
    fn invalid_fixture_clearance_keeps_draft_open_and_missing_fixtures_can_be_removed() {
        let mut app = model_list_app();
        let before = app.controller.document().clone();
        send(&mut app, Message::SetupFixtureToggled(2, true));
        for invalid in ["-1", "NaN", "", "-"] {
            send(
                &mut app,
                Message::SetupFixtureClearanceChanged(0, invalid.into()),
            );
            send(&mut app, Message::SetupCommit);
            assert!(app.setup_draft.is_some());
            assert_eq!(app.controller.document(), &before);
        }
        send(
            &mut app,
            Message::SetupFixtureClearanceChanged(0, "0".into()),
        );
        send(&mut app, Message::SetupCommit);
        assert!(app.setup_draft.is_none());
        app.begin_setup_draft();
        app.setup_draft
            .as_mut()
            .unwrap()
            .fixture_instances
            .push(999);

        let _ = app.setup_editor();
        send(&mut app, Message::SetupFixtureToggled(999, false));
        assert_eq!(app.setup_draft.as_ref().unwrap().fixture_instances, vec![2]);
    }

    #[test]
    fn relative_stock_defaults_to_one_and_all_sides_are_editable() {
        let mut app = model_list_app();
        let _ = app.update(Message::SetupStockKindChanged(StockEditorKind::FixedBox));
        let _ = app.update(Message::SetupStockKindChanged(StockEditorKind::RelativeBox));
        assert_eq!(
            app.setup_draft.as_ref().unwrap().stock,
            cam_model::Stock::RelativeBox {
                offsets: [[1.0; 3]; 2]
            }
        );
        for index in 0..6 {
            let _ = app.update(Message::SetupStockNumberChanged(
                index,
                (index + 2).to_string(),
            ));
        }
        assert_eq!(
            app.setup_draft.as_ref().unwrap().stock,
            cam_model::Stock::RelativeBox {
                offsets: [[2., 3., 4.], [5., 6., 7.]]
            }
        );
    }

    #[test]
    fn setup_geometry_clear_removes_only_the_chosen_role_in_the_draft() {
        let mut app = model_list_app();
        send(&mut app, Message::SetupModelToggled(1, true));
        send(&mut app, Message::SetupFixtureToggled(2, true));
        let committed = app.controller.document().clone();
        send(&mut app, Message::SetupGeometryClear(true));
        let draft = app.setup_draft.as_ref().unwrap();
        assert_eq!(draft.model_instances, vec![1]);
        assert!(draft.fixture_instances.is_empty());
        send(&mut app, Message::SetupGeometryClear(false));
        assert!(app.setup_draft.as_ref().unwrap().model_instances.is_empty());
        assert_eq!(app.controller.document(), &committed);
    }

    #[test]
    fn model_list_add_remove_and_cancel_are_staged_in_one_setup_draft() {
        let mut app = model_list_app();
        let before = app.controller.document().clone();
        assert!(app.setup_draft.as_ref().unwrap().model_instances.is_empty());
        send(&mut app, Message::SetupModelPick);
        assert!(app.setup_model_pick);
        assert_eq!(app.machining_viewport().cad_filter, EntityKind::Body);
        for instance in [1, 3, 2, 2, 2] {
            send(
                &mut app,
                Message::CadPicked(vec![PickCandidate {
                    entity: EntityRef {
                        asset_id: 1,
                        revision: 1,
                        instance_id: instance,
                        body_id: instance,
                        kind: EntityKind::Body,
                        id: instance,
                    },
                    depth: 0.0,
                    occluded: false,
                }]),
            );
        }
        assert_eq!(
            app.setup_draft.as_ref().unwrap().model_instances,
            vec![1, 3, 2]
        );
        assert!(
            app.setup_model_pick,
            "multiple bodies can be picked in one session"
        );
        assert_eq!(app.controller.document(), &before);
        send(&mut app, Message::CadEscape);
        assert!(!app.setup_model_pick);
        assert!(
            app.setup_draft_dirty(),
            "Escape finishes picking without losing draft edits"
        );

        send(&mut app, Message::SetupModelToggled(2, false));
        assert_eq!(
            app.setup_draft.as_ref().unwrap().model_instances,
            vec![1, 3]
        );
        send(&mut app, Message::SetupCancel);
        assert_eq!(app.controller.document(), &before);

        assert!(!app.setup_draft_dirty());
    }

    #[test]
    fn model_list_rejects_unknown_ids_and_allows_removing_the_last_model() {
        let mut app = model_list_app();
        send(&mut app, Message::SetupModelToggled(1, true));
        send(&mut app, Message::SetupModelToggled(99, true));

        assert_eq!(app.setup_draft.as_ref().unwrap().model_instances, vec![1]);
        let (min, max) = app
            .controller
            .pending_setup_model_bounds(app.setup_draft.as_ref().unwrap());
        assert_eq!((min, max), ([0.; 3], [10.; 3]));
        send(&mut app, Message::SetupCommit);
        assert_eq!(app.controller.inspection_setup().model_instances, vec![1]);
        app.begin_setup_draft();
        send(&mut app, Message::SetupModelToggled(1, false));
        assert!(app.setup_draft.as_ref().unwrap().model_instances.is_empty());

        send(&mut app, Message::SetupCommit);
        assert!(app.controller.inspection_setup().model_instances.is_empty());
        assert!(app.controller.undo());
        assert_eq!(app.controller.inspection_setup().model_instances, vec![1]);
    }

    #[test]
    fn body_membership_controls_bounds_and_survives_save_and_reopen() {
        let mut app = model_list_app();
        assert!(app
            .controller
            .setup_geometry_state()
            .unwrap()
            .instances
            .is_empty());
        send(&mut app, Message::SetupModelToggled(2, true));
        send(&mut app, Message::SetupCommit);
        assert_eq!(
            app.controller.setup_model_bounds(),
            ([20., 0., 0.], [30., 10., 10.])
        );
        let geometry = app.controller.setup_geometry_state().unwrap();
        assert_eq!(geometry.instances.len(), 1);
        assert_eq!(geometry.instances[0].body_id, 2);
        let path =
            std::env::temp_dir().join(format!("setup-body-membership-{}.ocam", std::process::id()));
        let first = app.controller.active_setup_id();
        let selected_bounds = app.controller.setup_model_bounds();
        let empty_setup = app.controller.add_setup();
        app.controller.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        reopened.activate_setup(empty_setup).unwrap();
        assert!(reopened.document().setup().model_instances.is_empty());
        assert!(reopened
            .setup_geometry_state()
            .unwrap()
            .instances
            .is_empty());
        reopened.activate_setup(first).unwrap();
        assert_eq!(reopened.document().setup().model_instances, vec![2]);
        assert_eq!(reopened.setup_model_bounds(), selected_bounds);
    }

    #[test]
    fn new_setup_has_no_models_but_duplicate_preserves_its_selection() {
        let mut app = model_list_app();
        send(&mut app, Message::SetupModelToggled(2, true));
        send(&mut app, Message::SetupCommit);
        let first = app.controller.active_setup_id();
        app.controller.duplicate_setup(first).unwrap();
        assert_eq!(app.controller.inspection_setup().model_instances, vec![2]);
        send(&mut app, Message::SetupNew);
        assert!(app.setup_draft.as_ref().unwrap().model_instances.is_empty());
    }

    #[test]
    fn setup_tabs_share_one_draft_and_cancel_restores_every_tab() {
        let mut app = blank_app();
        let committed = app.controller.inspection_setup().clone();
        send(&mut app, Message::SetupNameChanged("Top fixture".into()));
        send(
            &mut app,
            Message::SetupMaterialPicked(MaterialChoice {
                id: Some(1),
                name: "Aluminum".into(),
            }),
        );
        send(&mut app, Message::SetupProgramChanged("TOP-01".into()));
        send(&mut app, Message::SetupTabChanged(SetupEditorTab::Stock));
        send(
            &mut app,
            Message::SetupTabChanged(SetupEditorTab::Postprocess),
        );
        assert_eq!(app.setup_draft.as_ref().unwrap().name, "Top fixture");
        assert_eq!(app.setup_draft.as_ref().unwrap().material, "Aluminum");
        assert_eq!(app.setup_draft.as_ref().unwrap().program_name, "TOP-01");
        assert!(app.setup_draft_dirty());

        send(&mut app, Message::SetupCancel);
        assert_eq!(app.controller.inspection_setup(), &committed);
        assert!(!app.setup_draft_dirty());
    }

    #[test]
    fn invalid_numeric_and_stale_edge_references_block_setup_commit() {
        let mut app = blank_app();
        send(
            &mut app,
            Message::SetupNameChanged("Should not commit".into()),
        );
        app.setup_values.insert(0, "-".into());
        let entity = EntityRef {
            asset_id: 99,
            revision: 0,
            instance_id: 1,
            body_id: 1,
            kind: EntityKind::Edge,
            id: 99,
        };
        app.setup_draft.as_mut().unwrap().frame.references.x =
            Some(cam_model::FrameAxisReference {
                entity,
                flipped: false,
            });
        let issue = app.setup_frame_issue(app.setup_draft.as_ref().unwrap());
        assert!(
            issue.is_some(),
            "a stale edge reference should be visible as a field issue"
        );
        let committed_name = app.controller.inspection_setup().name.clone();
        send(&mut app, Message::SetupCommit);
        assert_eq!(app.controller.inspection_setup().name, committed_name);
        assert!(app.setup_draft_dirty());
    }

    #[test]
    fn setup_tool_change_editor_applies_wcs_height_and_restores_automatic_default() {
        let mut app = blank_app();
        let original = app.controller.inspection_setup().clone();
        send(&mut app, Message::SetupToolChangeAutomatic(false));
        send(&mut app, Message::SetupToolChangeHeightChanged("60".into()));
        send(&mut app, Message::SetupOriginChanged(2, "12".into()));
        assert_eq!(app.controller.inspection_setup(), &original);
        send(&mut app, Message::SetupCommit);
        assert!(app.setup_draft.is_none(), "{}", app.status);
        assert_eq!(
            app.controller.inspection_setup().tool_change_height,
            Some(72.)
        );
        assert!(app.controller.undo());
        assert_eq!(app.controller.inspection_setup(), &original);
        assert!(app.controller.redo());

        app.begin_setup_draft();
        send(
            &mut app,
            Message::SetupToolChangeHeightChanged("NaN".into()),
        );
        send(&mut app, Message::SetupCommit);
        assert!(app.setup_draft.is_some());
        assert_eq!(
            app.controller.inspection_setup().tool_change_height,
            Some(72.)
        );
        send(&mut app, Message::SetupToolChangeAutomatic(true));
        assert!(!app.setup_values.contains_key(&41));
        send(&mut app, Message::SetupCancel);
        assert_eq!(
            app.controller.inspection_setup().tool_change_height,
            Some(72.)
        );

        app.begin_setup_draft();
        send(&mut app, Message::SetupToolChangeAutomatic(true));
        send(&mut app, Message::SetupCommit);
        let setup = app.controller.inspection_setup();
        assert_eq!(setup.tool_change_height, None);
        assert_eq!(setup.resolved_tool_change_height(50.), 62.);
    }

    #[test]
    fn setup_origin_edit_is_explicit_and_stock_mode_change_discards_old_buffers() {
        let mut app = blank_app();
        send(&mut app, Message::SetupOriginChanged(0, "12.5".into()));
        assert_eq!(
            app.setup_draft.as_ref().unwrap().frame.references.origin,
            cam_model::FrameOrigin::Explicit([12.5, 0.0, 0.0])
        );

        app.setup_values.insert(20, "unfinished".into());
        send(
            &mut app,
            Message::SetupStockKindChanged(StockEditorKind::FixedBox),
        );
        assert!(!app.setup_values.contains_key(&20));
        assert!(matches!(
            app.setup_draft.as_ref().unwrap().stock,
            cam_model::Stock::FixedBox { .. }
        ));
    }

    #[test]
    fn setup_cancel_clears_armed_viewport_picks() {
        let mut app = blank_app();
        app.setting_origin_2pt = true;
        app.origin_first = Some([1.0, 2.0, 3.0]);
        app.setup_axis_pick = Some(1);
        send(&mut app, Message::SetupCancel);
        assert!(!app.setting_origin);
        assert!(!app.setting_origin_2pt);
        assert!(app.origin_first.is_none());
        assert!(app.setup_axis_pick.is_none());
    }

    #[test]
    fn undo_redo_and_open_sample_cannot_abandon_a_dirty_setup_draft() {
        let mut app = blank_app();
        let original = app.controller.inspection_setup().name.clone();
        send(&mut app, Message::SetupNameChanged("Uncommitted".into()));
        send(&mut app, Message::Undo);
        send(&mut app, Message::Redo);
        send(&mut app, Message::OpenSample);
        assert_eq!(app.controller.inspection_setup().name, original);
        assert_eq!(app.setup_draft.as_ref().unwrap().name, "Uncommitted");
        assert!(app.setup_draft_dirty());
    }

    #[test]
    fn operation_cancel_discards_enum_numeric_and_staged_tool_edits() {
        let (mut app, id) = carve_app();
        let committed = app.controller.inspection_setup().clone();
        send(&mut app, Message::CarveStayDownToggled(true));
        send(&mut app, Message::CarveClearToggled(true));
        send(&mut app, Message::FieldChanged(Field::Depth, "2.5".into()));
        assert!(app.operation_draft_dirty());
        assert!(!app.operation_tools.is_empty());

        send(&mut app, Message::CancelOperationDraft);
        assert_eq!(app.controller.inspection_setup(), &committed);
        assert!(!app.operation_draft_dirty());
        assert!(
            matches!(app.controller.operation(id), Some(Operation::Carve(op)) if !op.stay_down && op.clear.is_none() && op.depth == 1.0)
        );
    }

    #[test]
    fn operation_ok_commits_enum_numeric_and_staged_tool_in_one_undo_step() {
        let (mut app, id) = carve_app();
        let before_commit = app.controller.inspection_setup().clone();
        send(&mut app, Message::CarveStayDownToggled(true));
        send(&mut app, Message::CarveClearToggled(true));
        send(&mut app, Message::FieldChanged(Field::Depth, "2.5".into()));
        send(&mut app, Message::Apply);

        assert!(
            matches!(app.controller.operation(id), Some(Operation::Carve(op)) if op.stay_down && op.clear.is_some() && op.depth == 2.5)
        );
        assert_eq!(
            app.controller.inspection_setup().tools.len(),
            before_commit.tools.len() + 1
        );
        assert!(
            app.controller.undo(),
            "the operation and its staged cutter share one history edit"
        );
        assert_eq!(app.controller.inspection_setup(), &before_commit);
    }
}

/// What is left of the migration's scaffolding.
///
/// This module existed to assert that `Settings`'s defaults equalled the constants the
/// GUI actually used, while the two coexisted. They no longer do: `PICKBOX_PX`,
/// `SNAP_PICK_PX`, `SNAP_MARK_SCALE`, `GIZMO_SIZE_DEFAULT` and `Pane::min_size`'s
/// hard-coded match are all gone, and `Settings` is the only place those values are
/// written. **A cross-check deleted because the duplication went away is the good
/// outcome** — the remaining one guards a value that genuinely still lives in two
/// places, because the slider needs literal bounds.
#[cfg(test)]
mod settings_agree_with_constants {
    use super::*;

    #[test]
    fn pane_choices_restore_without_treating_an_idle_inspector_as_disabled() {
        let (mut app, _) = App::new();
        app.settings.panes = crate::PanePrefs {
            show_project_browser: false,
            show_library: true,
            inspector_enabled: false,
            show_output: false,
            ..Default::default()
        };
        app.restore_pane_preferences();
        app.sync_inspector_visibility();
        assert!(!app.show_project_browser);
        assert_eq!(app.workspace, Workspace::Tools);
        assert_eq!(app.active_tab, RibbonTab::Tools);
        assert!(app.pane_handle(Pane::Output).is_none());
        assert!(app.pane_handle(Pane::Inspector).is_none());

        // A project saved with its browser open must respect the user's pane choice.
        let mut view = app.capture_project_view();
        view.show_project_browser = true;
        app.controller.set_project_view(Some(view));
        app.restore_project_view();
        assert!(!app.show_project_browser);

        app.settings.panes = crate::PanePrefs::default();
        app.restore_pane_preferences();
        app.sync_inspector_visibility();
        assert!(app.show_project_browser);
        assert_eq!(app.workspace, Workspace::Machining);
        assert!(app.pane_handle(Pane::Output).is_some());
        assert!(app.inspector_enabled);
        assert!(app.pane_handle(Pane::Inspector).is_none());

        app.controller.open_dxf(SAMPLE_DXF, "sample").unwrap();
        app.controller.select(Selection::Setup);
        app.begin_setup_draft();
        app.sync_inspector_visibility();
        assert!(app.pane_handle(Pane::Inspector).is_some());
    }

    #[test]
    fn the_gizmo_slider_range_is_the_preferences_range() {
        assert_eq!(
            crate::GIZMO_SIZE_RANGE,
            (GIZMO_SIZE_MIN, GIZMO_SIZE_MAX),
            "the preference's range must be the slider's range"
        );
    }

    /// Every pane must take its minimum from preferences — a `min_size` arm that
    /// ignored `prefs` and returned a constant would silently pin one pane, and the
    /// symptom (one divider that will not go past a size) reads as a layout quirk
    /// rather than a bug.
    #[test]
    fn every_pane_reads_its_minimum_from_the_preferences() {
        let mut prefs = crate::PanePrefs {
            min_project_px: 111.0,
            min_library_px: 122.0,
            min_viewport_px: 133.0,
            min_inspector_px: 144.0,
            min_output_px: 155.0,
            ..Default::default()
        };
        for (pane, want) in [
            (Pane::Project, 111.0),
            (Pane::Library, 122.0),
            (Pane::Viewport, 133.0),
            (Pane::Inspector, 144.0),
            (Pane::Output, 155.0),
        ] {
            assert_eq!(
                pane.min_size(&prefs),
                want,
                "{pane:?} ignored the preference"
            );
        }
        // And they must not be reading a *shared* field either.
        prefs.min_inspector_px = 999.0;
        assert_eq!(Pane::Project.min_size(&prefs), 111.0);
        assert_eq!(Pane::Inspector.min_size(&prefs), 999.0);
    }
}

#[cfg(test)]
mod inspector_field_tests {
    use super::*;
    use cam_model::{DrillOp, Tool};

    #[test]
    fn new_pointed_tools_seed_a_physically_real_tip() {
        // Ground truth: neither tool has a true r=0 point — one cannot be ground and
        // would not survive contact. The kinds differ by what the tip *is*: a chamfer
        // mill's flat does not cut (so it cannot engrave), a V-bit's rounded tip does.
        // A zero on either would collapse them into the same geometry.
        match ToolKindPick::ChamferMill.to_kind() {
            ToolKind::ChamferMill { tip_diameter, .. } => {
                assert!(
                    tip_diameter >= 2.0 * MIN_TIP_RADIUS_MM,
                    "a chamfer mill is ground with a flat, at the physical floor"
                );
            }
            other => panic!("{other:?}"),
        }
        match ToolKindPick::VBit.to_kind() {
            ToolKind::VBit { tip_radius, .. } => {
                assert!(
                    tip_radius >= MIN_TIP_RADIUS_MM,
                    "a V-bit's point is ground to a radius, at the physical floor"
                );
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_seeded_chamfer_mill_cannot_engrave_but_a_seeded_vbit_can() {
        // The seeds must land on the right side of the engraving guard: this is the
        // whole practical consequence of flat-vs-rounded.
        let cham = Tool {
            number: 1,
            diameter: 6.0,
            length: 40.0,
            flutes: 2,
            kind: ToolKindPick::ChamferMill.to_kind(),
            ..Default::default()
        };
        let vbit = Tool {
            kind: ToolKindPick::VBit.to_kind(),
            ..cham.clone()
        };
        assert!(!cham.profile().has_cutting_tip(), "flat tip must not cut");
        assert!(vbit.profile().has_cutting_tip(), "rounded tip must cut");
    }

    // --- the Apply button's dirty rule ---

    #[test]
    fn an_untouched_inspector_is_not_dirty() {
        // The whole point: Apply greys out until something is actually pending, so the
        // operator can see at a glance what has been committed and what has not.
        let visible = [Field::Depth, Field::Feed, Field::Stepdown];
        let model = |f: Field| match f {
            Field::Depth => Some(4.0),
            Field::Feed => Some(300.0),
            Field::Stepdown => Some(2.0),
            _ => None,
        };
        let mut buffers = BTreeMap::new();
        for f in visible {
            buffers.insert(f, fmt_num(model(f).unwrap()));
        }
        assert!(!fields_are_dirty(&visible, &buffers, model));

        // Retyping the same number in a different but equal form is still not a change.
        buffers.insert(Field::Depth, "4.000".to_string());
        assert!(!fields_are_dirty(&visible, &buffers, model));
    }

    #[test]
    fn a_changed_field_makes_the_inspector_dirty() {
        let visible = [Field::Depth, Field::Feed];
        let model = |f: Field| match f {
            Field::Depth => Some(4.0),
            Field::Feed => Some(300.0),
            _ => None,
        };
        let mut buffers = BTreeMap::new();
        buffers.insert(Field::Depth, "4".to_string());
        buffers.insert(Field::Feed, "300".to_string());
        assert!(!fields_are_dirty(&visible, &buffers, model));
        buffers.insert(Field::Depth, "4.5".to_string());
        assert!(fields_are_dirty(&visible, &buffers, model));
    }

    #[test]
    fn half_typed_counts_as_dirty_but_an_unseeded_field_does_not() {
        let visible = [Field::Depth, Field::Feed];
        let model = |f: Field| match f {
            Field::Depth => Some(4.0),
            Field::Feed => Some(300.0),
            _ => None,
        };
        // Mid-typing something that is not yet a number ("-", "") is a pending edit.
        // Apply stays disabled regardless, via `any_field_invalid`.
        let mut buffers = BTreeMap::new();
        buffers.insert(Field::Depth, "-".to_string());
        assert!(fields_are_dirty(&visible, &buffers, model));

        // But "4." is not half-typed, it is four: Rust parses it, and the comparison is
        // numeric, so it correctly reads as no change at all.
        buffers.insert(Field::Depth, "4.".to_string());
        assert!(!fields_are_dirty(&visible, &buffers, model));

        // A field the model exposes but that has no buffer yet -- a control just
        // revealed by a picker -- is not an edit. Nothing has been typed into it.
        let buffers = BTreeMap::new();
        assert!(!fields_are_dirty(&visible, &buffers, model));
    }

    #[test]
    fn a_field_the_model_has_no_value_for_is_ignored() {
        // A field the model returns `None` for (e.g. not applicable to the current
        // selection) with a stale buffer must not light Apply up forever.
        let visible = [Field::OriginX];
        let mut buffers = BTreeMap::new();
        buffers.insert(Field::OriginX, "12".to_string());
        assert!(!fields_are_dirty(&visible, &buffers, |_| None));
    }

    fn drill(peck: Option<f64>, dwell: Option<f64>) -> Operation {
        Operation::Drill(DrillOp {
            parameter_expressions: Default::default(),
            spindle_rpm: 0.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            id: 1,
            tool: 1,
            points: vec![[0.0, 0.0]],
            depth: 10.0,
            start_offset: 0.0,
            peck,
            dwell,
            feed: 100.0,
        })
    }

    #[test]
    fn drill_exposes_peck_and_dwell_fields() {
        // This used to hand-build the list it then asserted on, so it could only ever
        // agree with itself. It now asks the inspector's own rule.
        let fields = operation_fields(&drill(None, None));
        assert!(fields.contains(&Field::Peck));
        assert!(fields.contains(&Field::Dwell));
    }

    fn chamfer(top: f64) -> Operation {
        Operation::Chamfer(cam_model::ChamferOp {
            parameter_expressions: Default::default(),
            contour_roughing: Default::default(),
            tip_offset: 0.0,
            lead_in_enabled: None,
            same_lead_out: false,
            lead_out_enabled: None,
            closed: true,
            faces: Vec::new(),
            stepdown: 0.,
            id: 1,
            tool: 1,
            chain: cam_geo::Contour::new(vec![
                cam_geo::Point::new(0.0, 0.0),
                cam_geo::Point::new(10.0, 0.0),
                cam_geo::Point::new(10.0, 10.0),
                cam_geo::Point::new(0.0, 10.0),
            ]),
            side: Side::Outside,
            width: 1.0,
            top,
            depth: 0.0,
            step: 0.0,
            gradual: false,
            spindle_rpm: 0.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            feed: 200.0,
            plunge_feed: 100.0,
            end: None,
            start: None,
            lead_in: Lead::None,
            lead_out: Lead::None,
            lead_overlap: 0.0,
        })
    }

    #[test]
    fn a_chamfers_top_edge_is_editable_and_may_sit_below_the_datum() {
        // `top` was seeded from the stock top and then unreachable, which is only ever
        // right for an edge on the raw surface. The rim of a pocket, or a step an
        // earlier operation cut, is the ordinary case that had no way to be stated.
        let mut op = chamfer(3.0);
        assert!(
            operation_fields(&op).contains(&Field::ChamferTop),
            "the inspector must show the top edge"
        );
        assert_eq!(op_field(&op, Field::ChamferTop), Some(3.0));

        // Deliberately unclamped: an edge below the part datum is a value, not a slip.
        let mut parsed = BTreeMap::new();
        parsed.insert(Field::ChamferTop, -4.5);
        apply_op_fields(&mut op, &parsed);
        assert_eq!(op_field(&op, Field::ChamferTop), Some(-4.5));
        let Operation::Chamfer(c) = &op else {
            unreachable!()
        };
        assert_eq!(c.top, -4.5, "a negative top must survive to the model");
    }

    /// One operation of every kind, built the way the app builds them.
    fn one_of_every_kind() -> Vec<Operation> {
        let mut app = AppController::new(crate::default_machine());
        app.open_dxf(SAMPLE_DXF, "sample.dxf")
            .expect("sample loads");
        for kind in [
            OpKind::Profile,
            OpKind::Pocket,
            OpKind::Face,
            OpKind::Drill,
            OpKind::Thread,
            OpKind::Chamfer,
            OpKind::Engrave,
            OpKind::Carve,
        ] {
            app.new_operation(kind);
        }
        app.document().setup().operations.clone()
    }

    #[test]
    fn every_field_the_inspector_shows_can_be_written_back() {
        // The gap this closes: a `Field` can be declared, labelled, given a tooltip and
        // listed by `operation_fields`, and still have no arm in `op_field` or
        // `apply_op_fields` — in which case the box renders, accepts typing, and springs
        // back on Apply. Nothing else in the crate calls either function, so only this
        // catches it. Asserted per operation kind, since the arms are per kind.
        let ops = one_of_every_kind();
        assert!(
            ops.len() >= 6,
            "the sample part must yield operations to check"
        );
        for op in &ops {
            for field in operation_fields(op) {
                let Some(before) = op_field(op, field) else {
                    panic!("{}: {field:?} is shown but cannot be read", op_kind(op));
                };
                // Exercise a valid edit; bottom-referenced taper permits only
                // nonpositive slopes for the supported cylindrical tools.
                let want = if field == Field::Profile(profile_parameters::Number::Taper) {
                    before - 1.
                } else {
                    before + 1.
                };
                let mut edited = op.clone();
                let mut parsed = BTreeMap::new();
                parsed.insert(field, want);
                apply_op_fields(&mut edited, &parsed);
                assert_eq!(
                    op_field(&edited, field),
                    Some(want),
                    "{}: {field:?} is shown but does not accept an edit ({before} -> {want})",
                    op_kind(op)
                );
            }
        }
    }

    #[test]
    fn every_labelled_plunge_parameter_explains_itself() {
        // The defect this exists to prevent, which really happened: the labels were made
        // kind-aware and the tooltips were not, so a box reading "Ramp angle (°)" still
        // explained itself as "the ramp/zig-zag angle in degrees, or the helix radius in
        // mm" — offering the reader two physically different quantities for the one number
        // in front of them, which is exactly what naming the label was meant to end.
        //
        // The property is the pairing, not the wording: a slot has a label if and only if
        // it has help. That is what drifted, and it can only be checked because both are
        // free functions — the methods that wrap them need a live selection.
        for plunge in PlungeKind::ALL.map(PlungeKind::to_plunge) {
            for first in [true, false] {
                let help = plunge_param_help(plunge, first);
                for clearing in [true, false] {
                    let label = plunge_param_label(plunge, first, clearing);
                    assert_eq!(
                        label.is_some(),
                        help.is_some(),
                        "{plunge:?} slot {} (clearing={clearing}): label {:?} but help {:?}",
                        if first { "A" } else { "B" },
                        label.map(|_| "some"),
                        help.map(|_| "some"),
                    );
                }
                // And where there is help, it must not fall back to the positional
                // phrasing the labels replaced.
                if let Some(h) = help {
                    assert!(
                        !h.contains(" or the helix") && !h.contains("plunge parameter"),
                        "{plunge:?} slot {} still explains itself positionally: {h}",
                        if first { "A" } else { "B" }
                    );
                }
            }
        }
        // Straight takes no parameters at all, and a ramp's second slot is unused — both
        // must stay unlabelled, or the inspector would show a box with nothing behind it.
        assert!(plunge_param_label(Plunge::Straight, true, false).is_none());
        assert!(plunge_param_label(Plunge::Ramp { angle_deg: 5.0 }, false, false).is_none());
    }

    #[test]
    fn a_carves_two_plunges_are_edited_separately() {
        // A carve is the only operation with **two** plunge styles — the V-bit's own
        // (v13) and the clearing end mill's — and they share one pair of parameter slots
        // split by prefix (`PlungeA` vs `ClearPlungeA`). Crossing them is a one-word
        // mistake, both sides are a `Plunge`, so nothing else would notice: the field
        // would read, write and round-trip perfectly, on the wrong tool.
        //
        // Not covered by `every_field_the_inspector_shows_can_be_written_back` either,
        // which only sees fields a *default* operation shows — and the default is
        // `Straight`, whose parameter boxes do not appear at all.
        let mut op = one_of_every_kind()
            .into_iter()
            .find(|o| matches!(o, Operation::Carve(_)))
            .expect("the sample part yields a carve");
        let Operation::Carve(c) = &mut op else {
            unreachable!()
        };
        c.plunge = Plunge::Ramp { angle_deg: 5.0 };
        c.clear = Some(CarveClearing {
            tool: 2,
            params: ClearParams {
                multiple_depths: Default::default(),
                adaptive_enabled: None,
                pocket_overlap: None,
                plunge: Plunge::Helix {
                    radius: 1.0,
                    pitch: 0.5,
                },
                ..Default::default()
            },
        });

        let fields = operation_fields(&op);
        assert!(
            fields.contains(&Field::PlungeA),
            "the V-bit's ramp angle is not shown"
        );
        assert!(
            fields.contains(&Field::ClearPlungeA),
            "the clearing helix radius is not shown"
        );
        assert_eq!(
            op_field(&op, Field::PlungeA),
            Some(5.0),
            "the V-bit's own angle"
        );
        assert_eq!(
            op_field(&op, Field::ClearPlungeA),
            Some(1.0),
            "the clearing pass's own radius"
        );

        // Editing one must leave the other exactly as it was.
        let mut parsed = BTreeMap::new();
        parsed.insert(Field::PlungeA, 12.0);
        apply_op_fields(&mut op, &parsed);
        let Operation::Carve(c) = &op else {
            unreachable!()
        };
        assert_eq!(c.plunge, Plunge::Ramp { angle_deg: 12.0 });
        assert_eq!(
            c.clear.expect("the clearing pass").params.plunge,
            Plunge::Helix {
                radius: 1.0,
                pitch: 0.5
            },
            "editing the V-bit's plunge moved the clearing pass's"
        );
    }

    #[test]
    fn peck_and_dwell_round_trip_with_a_zero_off_sentinel() {
        // Set: an on-value writes Some; then the same read shows it back.
        let mut op = drill(None, None);
        let mut parsed = BTreeMap::new();
        parsed.insert(Field::Peck, 2.5);
        parsed.insert(Field::Dwell, 0.75);
        apply_op_fields(&mut op, &parsed);
        assert_eq!(op_field(&op, Field::Peck), Some(2.5));
        assert_eq!(op_field(&op, Field::Dwell), Some(0.75));
        if let Operation::Drill(o) = &op {
            assert_eq!(o.peck, Some(2.5));
            assert_eq!(o.dwell, Some(0.75));
        }

        // 0 clears both back to None (off), and a disabled field reads as 0.
        let mut off = BTreeMap::new();
        off.insert(Field::Peck, 0.0);
        off.insert(Field::Dwell, 0.0);
        apply_op_fields(&mut op, &off);
        assert_eq!(op_field(&op, Field::Peck), Some(0.0));
        assert_eq!(op_field(&op, Field::Dwell), Some(0.0));
        if let Operation::Drill(o) = &op {
            assert_eq!(
                o.peck, None,
                "0 must clear peck to None (toolpath needs peck>0)"
            );
            assert_eq!(o.dwell, None);
        }
    }

    fn drilled(peck: Option<f64>, dwell: Option<f64>) -> Program {
        use cam_cldata::{DrillCycle, MoveKind, Tag};
        let mut prog = Program::new();
        prog.push(Step::Drill(DrillCycle {
            points: vec![[0.0, 0.0], [5.0, 0.0]],
            z_top: 0.0,
            depth: -6.0,
            retract: 2.0,
            peck,
            dwell,
            feed: 100.0,
            tag: Tag::new(1, MoveKind::Plunge),
        }));
        prog
    }

    #[test]
    fn peck_rings_land_on_intermediate_depths_only_never_the_bottom() {
        // depth 6, peck 2 ⇒ retracts at -2 and -4; the -6 bottom is not a peck.
        let marks = drill_marks_of(&drilled(Some(2.0), None));
        let rings: Vec<f32> = marks
            .iter()
            .filter(|m| m.kind == DrillMarkKind::PeckRing && m.at[0] == 0.0)
            .map(|m| m.at[2])
            .collect();
        assert_eq!(rings, vec![-2.0, -4.0], "one ring per intermediate peck");
        assert!(
            !marks.iter().any(|m| m.at[2] <= -6.0),
            "no mark sits at or below the bottom for a non-dwelling hole"
        );
        // Two holes ⇒ the rings are mirrored at the second point.
        assert_eq!(
            marks
                .iter()
                .filter(|m| m.kind == DrillMarkKind::PeckRing)
                .count(),
            4
        );
    }

    #[test]
    fn a_dwelling_hole_gets_one_bar_at_the_exact_bottom_per_point() {
        let marks = drill_marks_of(&drilled(None, Some(0.5)));
        let bars: Vec<[f32; 3]> = marks
            .iter()
            .filter(|m| m.kind == DrillMarkKind::DwellBar)
            .map(|m| m.at)
            .collect();
        assert_eq!(bars, vec![[0.0, 0.0, -6.0], [5.0, 0.0, -6.0]]);
        assert!(!marks.iter().any(|m| m.kind == DrillMarkKind::PeckRing));
    }

    #[test]
    fn a_plain_hole_has_no_annotations() {
        assert!(drill_marks_of(&drilled(None, None)).is_empty());
    }

    #[test]
    fn a_peck_bigger_than_the_hole_rings_nothing() {
        // First peck already reaches the bottom ⇒ no intermediate retract to mark.
        let marks = drill_marks_of(&drilled(Some(20.0), None));
        assert!(marks.is_empty());
    }

    #[test]
    fn a_negative_peck_is_treated_as_off_not_stored() {
        let mut op = drill(Some(1.0), None);
        let mut parsed = BTreeMap::new();
        parsed.insert(Field::Peck, -3.0);
        apply_op_fields(&mut op, &parsed);
        if let Operation::Drill(o) = &op {
            assert_eq!(o.peck, None, "a negative peck must not reach the toolpath");
        }
    }
}

#[cfg(test)]
mod ribbon_tests {
    use super::*;

    #[test]
    fn families_allow_deliberate_cutter_substitutions() {
        use ToolKindPick as F;
        for kind in [
            OpKind::Profile,
            OpKind::Pocket,
            OpKind::Bore,
            OpKind::Face,
            OpKind::Drill,
            OpKind::Engrave,
            OpKind::Adaptive,
        ] {
            let expected: Vec<_> = F::ALL
                .into_iter()
                .filter(|family| *family != F::RadiusMill)
                .collect();
            assert_eq!(families_for(kind), expected.as_slice(), "{kind:?}");
        }
        assert_eq!(families_for(OpKind::Thread), &[F::ThreadMill]);
        assert_eq!(
            families_for(OpKind::Chamfer),
            &[F::ChamferMill, F::VBit, F::RadiusMill]
        );
        assert_eq!(families_for(OpKind::Carve), &[F::VBit]);
    }

    #[test]
    fn every_bundled_icon_is_well_formed_svg() {
        // carve.svg once shipped with "--" inside an XML comment, which XML forbids.
        // The renderer does not complain: it simply draws nothing, so the button came
        // out blank and no test, build or clippy run had a word to say about it.
        // Parse every icon so a malformed one fails loudly instead of silently.
        for icon in Icon::ALL {
            let bytes = icon.bytes();
            let text = std::str::from_utf8(bytes).expect("an icon must be UTF-8");
            let doc = roxmltree::Document::parse(text)
                .unwrap_or_else(|e| panic!("{icon:?} is not well-formed SVG: {e}"));
            assert_eq!(
                doc.root_element().tag_name().name(),
                "svg",
                "{icon:?} is not an SVG"
            );
        }
    }

    #[test]
    fn every_svg_in_the_assets_tree_is_well_formed() {
        // The sweep above only covers what `Icon::ALL` names. The application icon is
        // not a ribbon icon, so it is in no enum and nothing in the code references
        // it -- it is read by the packaging workflows instead, where a malformed file
        // would surface as a blank icon on a shipped AppImage rather than as a test
        // failure. So walk the directory rather than the enum: an asset is checked
        // because it exists, not because some code happens to mention it.
        //
        // NB this proves well-formedness and nothing more. Parsing would not have
        // caught either of the other two ways this icon's SVG went wrong (rsvg
        // ignoring `textLength`, and `dominant-baseline` honoured by rsvg but not by
        // desktop viewers) -- both parse perfectly and simply render differently.
        fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for entry in std::fs::read_dir(dir).expect("assets dir must be readable") {
                let path = entry.expect("readable entry").path();
                if path.is_dir() {
                    walk(&path, out);
                } else if path.extension().is_some_and(|e| e == "svg") {
                    out.push(path);
                }
            }
        }
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("assets");
        let mut svgs = Vec::new();
        walk(&root, &mut svgs);
        assert!(
            svgs.len() > Icon::ALL.len(),
            "expected the ribbon icons plus at least the application icon, found {}",
            svgs.len()
        );
        for path in &svgs {
            let text = std::fs::read_to_string(path)
                .unwrap_or_else(|e| panic!("{}: unreadable: {e}", path.display()));
            let doc = roxmltree::Document::parse(&text)
                .unwrap_or_else(|e| panic!("{}: not well-formed SVG: {e}", path.display()));
            assert_eq!(
                doc.root_element().tag_name().name(),
                "svg",
                "{}: root element is not <svg>",
                path.display()
            );
        }
    }

    #[test]
    fn the_legacy_application_icon_carries_its_author_and_licence() {
        // The legacy icon is the project's deliberate departure from GPL-3.0-only
        // (CC BY-SA 4.0, so the mark can live on Wikimedia). That exception is only
        // worth anything if the claim actually travels with the file, and the file
        // goes through editors -- Inkscape rewrote it once already -- so pin it.
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("assets")
            .join("opencamstudio.svg");
        let text = std::fs::read_to_string(&path).expect("the application icon must exist");
        let doc = roxmltree::Document::parse(&text).expect("well-formed");
        assert!(
            text.contains("Andreas Bertsatos"),
            "the application icon must name its author"
        );
        assert!(
            text.contains("creativecommons.org/licenses/by-sa/4.0/"),
            "the application icon must declare CC BY-SA 4.0"
        );
        // Outlines, not live text: a <text> element would make the rendered icon
        // depend on whichever font the packaging runner happens to have installed,
        // and Linux, macOS and Windows would each resolve it differently.
        assert!(
            !doc.descendants().any(|n| n.has_tag_name("text")),
            "the application icon must contain no live <text>; convert glyphs to paths"
        );
    }

    #[test]
    fn every_icon_is_listed_in_all() {
        // A cheap guard on the sweep above: if a variant is added without extending
        // ALL, the icon it names goes unchecked.
        let mut seen: Vec<&[u8]> = Icon::ALL.iter().map(|i| i.bytes()).collect();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), Icon::ALL.len(), "two variants share one asset");
    }

    #[test]
    fn geometric_cutter_requirements_are_separate_from_suitability() {
        use ToolKindPick as F;
        assert!(families_for(OpKind::Engrave).contains(&F::ChamferMill));
        assert!(families_for(OpKind::Bore).contains(&F::BallMill));
        assert!(families_for(OpKind::Bore).contains(&F::VBit));
        assert_eq!(families_for(OpKind::Thread), &[F::ThreadMill]);
        assert_eq!(
            families_for(OpKind::Chamfer),
            &[F::ChamferMill, F::VBit, F::RadiusMill]
        );
        assert_eq!(families_for(OpKind::Carve), &[F::VBit]);
    }

    #[test]
    fn every_operation_offers_at_least_one_family() {
        for op in [
            OpKind::Profile,
            OpKind::Pocket,
            OpKind::Face,
            OpKind::Drill,
            OpKind::Thread,
            OpKind::Chamfer,
            OpKind::Engrave,
            OpKind::Carve,
        ] {
            assert!(!families_for(op).is_empty(), "{op:?} offers nothing");
        }
    }

    #[test]
    fn tabs_read_left_to_right_in_workflow_order() {
        // One tab per destination; workspace navigation is handled by these tabs.
        let labels: Vec<&str> = RibbonTab::ALL.iter().map(|t| t.label()).collect();
        assert_eq!(
            labels,
            vec!["Home", "Machining", "Tools", "Machinery", "View"]
        );
    }

    #[test]
    fn the_tool_library_compacts_to_tools_in_the_ribbon_only() {
        // The ribbon band is tight, so the label is shortened there — but the pane's
        // own identity (its title bar, and anything keyed on the name) is unchanged.
        assert_eq!(Pane::Library.ribbon_label(), "Tools");
        assert_eq!(Pane::Library.name(), "Tool Library");
        // Every other pane reads the same in both places.
        for pane in ALL_PANES.into_iter().filter(|p| *p != Pane::Library) {
            assert_eq!(pane.ribbon_label(), pane.name(), "{pane:?}");
        }
    }

    #[test]
    fn the_panes_group_covers_every_pane_except_the_viewport() {
        // The viewport is always visible and deliberately has no toggle; everything
        // else must be reachable, so a pane added later cannot go missing.
        let toggled: Vec<Pane> = ALL_PANES
            .into_iter()
            .filter(|p| *p != Pane::Viewport)
            .collect();
        assert_eq!(toggled.len(), ALL_PANES.len() - 1);
        assert!(!toggled.contains(&Pane::Viewport));
        // Two balanced columns, so the block stays inside the ~70 px band.
        let split = toggled.len().div_ceil(2);
        assert_eq!(toggled.chunks(split).count(), 2, "must be two columns");
    }

    // Home tab shape: File(2) + Edit(3).
    const HOME: [usize; 2] = [2, 3];

    #[test]
    fn wide_window_keeps_all_groups_full() {
        let d = solve_densities(&HOME, 2000.0);
        assert_eq!(d, vec![Density::Full, Density::Full]);
    }

    #[test]
    fn narrowing_degrades_right_group_first() {
        // A width that fits Full+Compact but not Full+Full: the rightmost (Edit)
        // group must be the one that drops.
        let full_full = row_width(&HOME, &[Density::Full, Density::Full]);
        let full_compact = row_width(&HOME, &[Density::Full, Density::Compact]);
        let w = 0.5 * (full_full + full_compact); // between the two
        let d = solve_densities(&HOME, w);
        assert_eq!(d, vec![Density::Full, Density::Compact]);
    }

    #[test]
    fn very_narrow_collapses_everything_to_tight() {
        let d = solve_densities(&HOME, 1.0);
        assert_eq!(d, vec![Density::Tight, Density::Tight]);
    }

    #[test]
    fn degradation_is_monotonic_in_width() {
        // As available width shrinks, total tightness never decreases.
        let tightness = |d: &[Density]| d.iter().map(|&x| x as u8 as u32).sum::<u32>();
        let mut prev = 0;
        for w in (0..1200).step_by(20).map(|x| x as f32) {
            let t = tightness(&solve_densities(&HOME, w));
            // Wider (later, larger w) must be <= tighter earlier — iterate ascending
            // width and assert non-increasing tightness relative to the previous
            // (narrower) width.
            assert!(t <= prev || prev == 0, "tightness rose as width grew");
            prev = t;
        }
    }
}

#[cfg(test)]
mod indexed_project_restore_tests {
    use super::*;

    #[test]
    fn restored_indexed_operation_is_clean_and_can_simulate() {
        let path =
            std::env::temp_dir().join(format!("indexed-restore-{}.ocam", std::process::id()));
        let (mut app, _) = App::new();
        app.controller.open_dxf(SAMPLE_DXF, "profiles.dxf").unwrap();
        app.controller.new_operation(OpKind::Pocket);
        let id = app.controller.inspection_setup().operations[0].id();
        let index = cam_model::RotaryIndex {
            angle: -90.0,
            ..Default::default()
        };
        app.controller
            .set_operation_rotary_index(id, Some(index.clone()))
            .unwrap();
        app.controller.select(Selection::Operation(id));
        app.controller
            .set_project_view(Some(app.capture_project_view()));
        app.controller.save_project(&path).unwrap();
        app.open_project(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        assert_eq!(app.rotary_index_draft.as_ref(), Some(&index));
        assert!(!app.inspector_dirty());
        let _ = app.toolpath_command(TreeNode::Operation(id), ToolpathAction::Simulate);
        assert!(!app.status.contains("Apply or cancel inspector changes"));
        assert_eq!(app.rotary_index_draft.as_ref(), Some(&index));
        app.rotary_index_draft.as_mut().unwrap().angle = 45.0;
        assert!(app.inspector_dirty());
        let _ = app.toolpath_command(TreeNode::Operation(id), ToolpathAction::Simulate);
        assert!(app.status.contains("Apply or cancel inspector changes"));
    }
}
