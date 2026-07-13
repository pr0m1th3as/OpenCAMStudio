//! The iced desktop shell — a thin view over [`crate::AppController`].
//!
//! A professional-CAM layout built on iced's resizable [`pane_grid`]: a toolbar
//! across the top, then four docked panes — a **Project** tree (select a node),
//! the **Viewport** (backplot + simulated stock), an **Inspector** (edit the
//! selected node), and an **Output** dock (diagnostics + status). All behaviour
//! is delegated to the controller; this module only translates messages and
//! draws. Only compiled with the `gui` feature.

use std::collections::BTreeMap;
use std::sync::Arc;

use iced::widget::pane_grid::{self, PaneGrid};
use iced::widget::{button, column, container, row, scrollable, shader, text, text_input};
use iced::{Alignment, Element, Length};

use cam_model::{Envelope, Machine, Operation, Point3};
use cam_render::{MeshVertex, OrbitCamera, Scene, Vertex, PART};
use cam_toolpath::{CancelToken, Severity};

use crate::{AppController, Selection};

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
    iced::application(App::new, App::update, App::view)
        .title("OpenCAMStudio")
        .theme(theme)
        .run()
}

fn theme(_state: &App) -> iced::Theme {
    iced::Theme::Dark
}

/// The docked regions of the shell.
#[derive(Clone, Copy, Debug)]
enum Pane {
    Project,
    Viewport,
    Inspector,
    Output,
}

/// An editable inspector field, keyed independently of which node owns it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Field {
    Clearance,
    Retract,
    TopOfStock,
    ToolDiameter,
    Depth,
    Stepdown,
    Stepover,
    Feed,
    PlungeFeed,
}

impl Field {
    fn label(self) -> &'static str {
        match self {
            Field::Clearance => "Clearance (mm)",
            Field::Retract => "Retract (mm)",
            Field::TopOfStock => "Top of stock (mm)",
            Field::ToolDiameter => "Tool ⌀ (mm)",
            Field::Depth => "Depth (mm)",
            Field::Stepdown => "Stepdown (mm)",
            Field::Stepover => "Stepover (mm)",
            Field::Feed => "Feed (mm/min)",
            Field::PlungeFeed => "Plunge feed (mm/min)",
        }
    }
}

struct App {
    controller: AppController,
    panes: pane_grid::State<Pane>,
    /// Edit buffers for the inspector fields of the current selection.
    fields: BTreeMap<Field, String>,
    /// Whether the viewport overlays the simulated stock surface.
    show_stock: bool,
    /// Whether the orientation-cube gizmo is shown (toggleable).
    show_gizmo: bool,
    /// The orbit-camera orientation, owned here; clicking a cube face or dragging
    /// the viewport reports changes back as messages.
    view: ViewControls,
    status: String,
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

#[derive(Debug, Clone)]
enum Message {
    OpenSample,
    Select(Selection),
    FieldChanged(Field, String),
    /// Commit the inspector fields (one undo step) and recompute the toolpath.
    Apply,
    Export,
    Undo,
    Redo,
    ToggleStock,
    /// Relative camera changes reported by dragging in the viewport.
    OrbitBy(f32, f32),
    PanBy([f32; 3]),
    ZoomBy(f32),
    /// Snap to a `(yaw, pitch)` orientation (a clicked cube face), re-framing.
    SetView(f32, f32),
    /// Reset to the framed top view.
    ResetView,
    /// Show or hide the orientation cube.
    ToggleGizmo,
    PaneResized(pane_grid::ResizeEvent),
    PaneDragged(pane_grid::DragEvent),
}

fn default_machine() -> Machine {
    Machine {
        name: "desktop".into(),
        rapid_rate: 2000.0,
        max_spindle_rpm: 10_000.0,
        max_feed: 800.0,
        envelope: Envelope::new(
            Point3::new(0.0, 0.0, -50.0),
            Point3::new(300.0, 180.0, 50.0),
        ),
        safe_z: 5.0,
        tool_change_pos: None,
    }
}

/// The initial four-pane layout: Project | (Viewport | Inspector), Output below.
fn initial_panes() -> pane_grid::State<Pane> {
    use pane_grid::{Axis, Configuration};
    pane_grid::State::with_configuration(Configuration::Split {
        axis: Axis::Horizontal,
        ratio: 0.76,
        a: Box::new(Configuration::Split {
            axis: Axis::Vertical,
            ratio: 0.19,
            a: Box::new(Configuration::Pane(Pane::Project)),
            b: Box::new(Configuration::Split {
                axis: Axis::Vertical,
                ratio: 0.72,
                a: Box::new(Configuration::Pane(Pane::Viewport)),
                b: Box::new(Configuration::Pane(Pane::Inspector)),
            }),
        }),
        b: Box::new(Configuration::Pane(Pane::Output)),
    })
}

impl App {
    fn new() -> (Self, iced::Task<Message>) {
        let mut app = Self {
            controller: AppController::new(default_machine()),
            panes: initial_panes(),
            fields: BTreeMap::new(),
            show_stock: false,
            show_gizmo: true,
            view: ViewControls::default(),
            status: "Open the sample part to begin.".to_string(),
        };
        app.refresh_fields();
        (app, iced::Task::none())
    }

    fn update(&mut self, message: Message) -> iced::Task<Message> {
        match message {
            Message::OpenSample => match self.controller.open_dxf(SAMPLE_DXF, "sample.dxf") {
                Ok(n) => {
                    self.refresh_fields();
                    self.status = format!("Imported {n} region(s).");
                    self.rerun();
                }
                Err(e) => self.status = format!("Import failed: {e}"),
            },
            Message::Select(selection) => {
                self.controller.select(selection);
                self.refresh_fields();
            }
            // Field edits only touch the local buffer; nothing is applied or
            // recomputed until Apply, so undo has one step per real change.
            Message::FieldChanged(field, value) => {
                self.fields.insert(field, value);
            }
            Message::Apply => self.apply_inspector(),
            Message::Export => {
                self.status = match self.controller.export_nc() {
                    Ok(nc) => format!("Exported {} lines of G-code.", nc.lines().count()),
                    Err(crate::ExportError::RapidThroughStock(n)) => {
                        format!("Export blocked: {n} rapid(s) through stock — see Output.")
                    }
                    Err(e) => format!("Export blocked: {e:?}"),
                };
            }
            Message::Undo => {
                if self.controller.undo() {
                    self.refresh_fields();
                    self.rerun();
                    self.status = format!("Undid change. {}", self.status);
                } else {
                    self.status = "Nothing to undo.".to_string();
                }
            }
            Message::Redo => {
                if self.controller.redo() {
                    self.refresh_fields();
                    self.rerun();
                    self.status = format!("Redid change. {}", self.status);
                } else {
                    self.status = "Nothing to redo.".to_string();
                }
            }
            Message::ToggleStock => {
                self.show_stock = !self.show_stock;
                self.status = if self.show_stock {
                    "Showing simulated stock.".to_string()
                } else {
                    "Hiding simulated stock.".to_string()
                };
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
            Message::ToggleGizmo => self.show_gizmo = !self.show_gizmo,
            Message::PaneResized(pane_grid::ResizeEvent { split, ratio }) => {
                self.panes.resize(split, ratio);
            }
            Message::PaneDragged(pane_grid::DragEvent::Dropped { pane, target }) => {
                self.panes.drop(pane, target);
            }
            Message::PaneDragged(_) => {}
        }
        iced::Task::none()
    }

    /// Reload the inspector edit buffers from the model for the current
    /// selection.
    fn refresh_fields(&mut self) {
        self.fields.clear();
        for field in self.inspector_fields() {
            if let Some(value) = self.field_value(field) {
                self.fields.insert(field, fmt_num(value));
            }
        }
    }

    /// Which fields the inspector shows for the current selection.
    fn inspector_fields(&self) -> Vec<Field> {
        match self.controller.selection() {
            Selection::Setup => vec![Field::Clearance, Field::Retract, Field::TopOfStock],
            Selection::Tool(_) => vec![Field::ToolDiameter],
            Selection::Stock => Vec::new(),
            Selection::Operation(id) => match self.controller.operation(id) {
                Some(Operation::Profile(_)) => {
                    vec![
                        Field::Depth,
                        Field::Stepdown,
                        Field::Feed,
                        Field::PlungeFeed,
                    ]
                }
                Some(Operation::Pocket(_)) => vec![
                    Field::Depth,
                    Field::Stepdown,
                    Field::Stepover,
                    Field::Feed,
                    Field::PlungeFeed,
                ],
                Some(Operation::Face(_)) => vec![Field::Depth, Field::Stepdown, Field::Feed],
                Some(Operation::Drill(_)) => vec![Field::Depth, Field::Feed],
                None => Vec::new(),
            },
        }
    }

    /// The model value backing a field for the current selection, if any.
    fn field_value(&self, field: Field) -> Option<f64> {
        let setup = &self.controller.document().setup;
        match field {
            Field::Clearance => Some(setup.heights.clearance),
            Field::Retract => Some(setup.heights.retract),
            Field::TopOfStock => Some(setup.heights.top_of_stock),
            Field::ToolDiameter => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.diameter),
                _ => None,
            },
            _ => self
                .controller
                .selected_operation()
                .and_then(|op| op_field(op, field)),
        }
    }

    /// Parse the inspector buffers and commit them to the selected node as one
    /// undoable change, then recompute.
    fn apply_inspector(&mut self) {
        let mut parsed: BTreeMap<Field, f64> = BTreeMap::new();
        for (&field, text) in &self.fields {
            match text.trim().parse::<f64>() {
                Ok(v) => {
                    parsed.insert(field, v);
                }
                Err(_) => {
                    self.status = format!("{} is not a valid number.", field.label());
                    return;
                }
            }
        }

        match self.controller.selection() {
            Selection::Setup => self.controller.edit_heights(|h| {
                if let Some(&v) = parsed.get(&Field::Clearance) {
                    h.clearance = v;
                }
                if let Some(&v) = parsed.get(&Field::Retract) {
                    h.retract = v;
                }
                if let Some(&v) = parsed.get(&Field::TopOfStock) {
                    h.top_of_stock = v;
                }
            }),
            Selection::Tool(i) => self.controller.edit_tool(i, |t| {
                if let Some(&v) = parsed.get(&Field::ToolDiameter) {
                    t.diameter = v;
                }
            }),
            Selection::Operation(_) => self
                .controller
                .edit_selected_operation(|op| apply_op_fields(op, &parsed)),
            Selection::Stock => {}
        }
        self.refresh_fields();
        self.rerun();
    }

    /// Recompute the toolpath for the current document and report the result.
    fn rerun(&mut self) {
        if self.controller.document().setup.operations.is_empty() {
            self.status = "Open a part first.".to_string();
            return;
        }
        let outcome = self.controller.run(&CancelToken::new());
        let errors = outcome
            .diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Error)
            .count();
        self.status = if errors > 0 {
            format!("{errors} error(s) — see Output.")
        } else {
            "Toolpath ready. Export when you like.".to_string()
        };
    }

    fn view(&self) -> Element<'_, Message> {
        let toolbar = row![
            text("OpenCAMStudio").size(18),
            button("Open sample").on_press(Message::OpenSample),
            button("Run").on_press(Message::Apply),
            button("Export .nc").on_press(Message::Export),
            button("Undo").on_press(Message::Undo),
            button("Redo").on_press(Message::Redo),
            button(if self.show_stock {
                "Hide stock"
            } else {
                "Show stock"
            })
            .on_press(Message::ToggleStock),
            button("Reset view").on_press(Message::ResetView),
            button(if self.show_gizmo {
                "Cube: on"
            } else {
                "Cube: off"
            })
            .on_press(Message::ToggleGizmo),
        ]
        .spacing(8)
        .padding(8)
        .align_y(Alignment::Center);

        let grid = PaneGrid::new(&self.panes, |_id, pane, _is_maximized| {
            let title = match pane {
                Pane::Project => "Project",
                Pane::Viewport => "Viewport",
                Pane::Inspector => "Inspector",
                Pane::Output => "Output",
            };
            pane_grid::Content::new(self.pane_content(*pane))
                .title_bar(pane_grid::TitleBar::new(text(title).size(13)).padding(4))
        })
        .spacing(4)
        .on_resize(8, Message::PaneResized)
        .on_drag(Message::PaneDragged)
        .width(Length::Fill)
        .height(Length::Fill);

        column![toolbar, grid].into()
    }

    fn pane_content(&self, pane: Pane) -> Element<'_, Message> {
        match pane {
            Pane::Project => self.project_tree(),
            Pane::Viewport => container(
                shader(Viewport::new(
                    &self.controller,
                    self.show_stock,
                    self.view,
                    self.show_gizmo,
                ))
                .width(Length::Fill)
                .height(Length::Fill),
            )
            .width(Length::Fill)
            .height(Length::Fill)
            .into(),
            Pane::Inspector => self.inspector(),
            Pane::Output => self.output(),
        }
    }

    /// The project tree: setup, stock, tools, and operations as selectable rows.
    fn project_tree(&self) -> Element<'_, Message> {
        let setup = &self.controller.document().setup;
        let sel = self.controller.selection();
        let node = |label: String, target: Selection, active: bool| {
            let mark = if active { "▸ " } else { "  " };
            button(text(format!("{mark}{label}")).size(13))
                .on_press(Message::Select(target))
                .width(Length::Fill)
        };

        let mut list = column![
            node(
                format!("Setup — {}", setup.name),
                Selection::Setup,
                sel == Selection::Setup
            ),
            node(
                "Stock".to_string(),
                Selection::Stock,
                sel == Selection::Stock
            ),
            text("  Tools").size(12),
        ]
        .spacing(2);
        for (i, tool) in setup.tools.iter().enumerate() {
            list = list.push(node(
                format!("  T{} ⌀{}", tool.number, fmt_num(tool.diameter)),
                Selection::Tool(i),
                sel == Selection::Tool(i),
            ));
        }
        list = list.push(text("  Operations").size(12));
        if setup.operations.is_empty() {
            list = list.push(text("    (none — open a part)").size(11));
        }
        for op in &setup.operations {
            let id = op.id();
            list = list.push(node(
                format!("  {}: {}", id, op_kind(op)),
                Selection::Operation(id),
                sel == Selection::Operation(id),
            ));
        }

        scrollable(list.padding(6))
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    /// The inspector: editable fields for the selected node.
    fn inspector(&self) -> Element<'_, Message> {
        let heading = match self.controller.selection() {
            Selection::Setup => "Setup".to_string(),
            Selection::Stock => "Stock".to_string(),
            Selection::Tool(i) => format!("Tool {}", i + 1),
            Selection::Operation(id) => match self.controller.operation(id) {
                Some(op) => format!("Operation {id} — {}", op_kind(op)),
                None => "Operation".to_string(),
            },
        };

        let mut list = column![text(heading).size(15)].spacing(8).padding(8);

        if let Selection::Stock = self.controller.selection() {
            let cam_model::Stock::Box { min, max } = self.controller.document().setup.stock;
            list = list.push(
                text(format!(
                    "Box  X {:.1}…{:.1}\n     Y {:.1}…{:.1}\n     Z {:.1}…{:.1}",
                    min[0], max[0], min[1], max[1], min[2], max[2]
                ))
                .size(12),
            );
            return scrollable(list)
                .width(Length::Fill)
                .height(Length::Fill)
                .into();
        }

        let ordered = self.inspector_fields();
        if ordered.is_empty() {
            list = list.push(text("Nothing to edit here yet.").size(12));
        }
        for field in ordered {
            let value = self.fields.get(&field).cloned().unwrap_or_default();
            list = list.push(
                row![
                    text(field.label()).width(Length::Fixed(150.0)).size(13),
                    text_input("", &value)
                        .on_input(move |v| Message::FieldChanged(field, v))
                        .on_submit(Message::Apply)
                        .width(Length::Fixed(90.0)),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
        }
        list = list.push(button("Apply").on_press(Message::Apply));

        scrollable(list)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    /// The output dock: run diagnostics and the status line.
    fn output(&self) -> Element<'_, Message> {
        let mut list = column![text(self.status.clone()).size(13)]
            .spacing(4)
            .padding(8);
        if let Some(outcome) = self.controller.outcome() {
            if outcome.diagnostics.is_empty() {
                list = list.push(text("No diagnostics.").size(12));
            } else {
                list = list.push(text("Diagnostics:").size(13));
                for d in &outcome.diagnostics {
                    list = list.push(text(format!("• {:?}: {}", d.severity, d.message)).size(12));
                }
            }
            // Material-removal simulation results: collisions a backplot can't show.
            if outcome.collisions.is_empty() {
                list = list.push(text("Simulation: no collisions.").size(12));
            } else {
                list =
                    list.push(text(format!("Collisions ({}):", outcome.collisions.len())).size(13));
                for c in &outcome.collisions {
                    list = list.push(text(format!("⚠ {:?}: {}", c.kind, c.message)).size(12));
                }
            }
        }
        scrollable(list)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }
}

/// A short label for an operation's kind.
fn op_kind(op: &Operation) -> &'static str {
    match op {
        Operation::Profile(_) => "Profile",
        Operation::Drill(_) => "Drill",
        Operation::Pocket(_) => "Pocket",
        Operation::Face(_) => "Face",
    }
}

/// Read an operation's value for a given field, if the op has it.
fn op_field(op: &Operation, field: Field) -> Option<f64> {
    match (op, field) {
        (Operation::Profile(o), Field::Depth) => Some(o.depth),
        (Operation::Profile(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Profile(o), Field::Feed) => Some(o.feed),
        (Operation::Profile(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Pocket(o), Field::Depth) => Some(o.depth),
        (Operation::Pocket(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Pocket(o), Field::Stepover) => Some(o.stepover),
        (Operation::Pocket(o), Field::Feed) => Some(o.feed),
        (Operation::Pocket(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Face(o), Field::Depth) => Some(o.depth),
        (Operation::Face(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Face(o), Field::Feed) => Some(o.feed),
        (Operation::Drill(o), Field::Depth) => Some(o.depth),
        (Operation::Drill(o), Field::Feed) => Some(o.feed),
        _ => None,
    }
}

/// Write the parsed inspector fields onto an operation.
fn apply_op_fields(op: &mut Operation, parsed: &BTreeMap<Field, f64>) {
    let get = |f: Field| parsed.get(&f).copied();
    match op {
        Operation::Profile(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
        }
        Operation::Pocket(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v;
            }
            if let Some(v) = get(Field::Stepover) {
                o.stepover = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
        }
        Operation::Face(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
        }
        Operation::Drill(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
        }
    }
}

/// Format a number for display (Rust's shortest round-trippable form).
fn fmt_num(v: f64) -> String {
    format!("{v}")
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

/// The shader widget's transient state: only the active drag. The camera lives
/// in [`App`] so the view-cube buttons can drive it; drags report *relative*
/// deltas back as messages (loss-free even across a burst of events).
#[derive(Default)]
struct ViewportState {
    drag: Option<DragMode>,
    last: Option<iced::Point>,
}

/// Orbit sensitivity (radians per pixel).
const ORBIT_SENS: f32 = 0.008;
/// Zoom sensitivity (exponent per wheel line).
const ZOOM_SENS: f32 = 0.15;

/// The orientation cube's square rectangle in the top-right of a `w × h`
/// viewport (widget-local coordinates). Fractional, so it is the same in logical
/// and physical pixels — the click hit-test and the drawn cube stay aligned.
fn gizmo_rect(w: f32, h: f32) -> (f32, f32, f32) {
    let size = (w.min(h) * 0.26).max(1.0);
    let margin = 8.0;
    (w - size - margin, margin, size)
}

struct Viewport {
    vertices: Arc<Vec<Vertex>>,
    mesh_vertices: Arc<Vec<MeshVertex>>,
    mesh_indices: Arc<Vec<u32>>,
    bounds: Option<([f32; 3], [f32; 3])>,
    controls: ViewControls,
    show_gizmo: bool,
}

impl Viewport {
    fn new(
        controller: &AppController,
        show_stock: bool,
        controls: ViewControls,
        show_gizmo: bool,
    ) -> Self {
        // After a run, show the full backplot; before it, at least show the
        // imported part outlines so opening a file is visibly reflected.
        let scene = match controller.outcome() {
            Some(outcome) => outcome.scene.clone(),
            None => {
                let mut scene = Scene::new();
                for region in controller.regions() {
                    scene.add_region(region, PART);
                }
                scene
            }
        };
        // The simulated stock is drawn under the backplot, only when toggled on
        // and available (a run has produced it).
        let (mesh_vertices, mesh_indices) = match controller.outcome() {
            Some(outcome) if show_stock => (
                outcome.stock_vertices.clone(),
                outcome.stock_indices.clone(),
            ),
            _ => (Vec::new(), Vec::new()),
        };
        Self {
            vertices: Arc::new(scene.line_vertices()),
            mesh_vertices: Arc::new(mesh_vertices),
            mesh_indices: Arc::new(mesh_indices),
            bounds: scene.bounds(),
            controls,
            show_gizmo,
        }
    }

    /// The orbit camera framed on the current scene, with the current controls.
    fn camera(&self) -> OrbitCamera {
        let (min, max) = self.bounds.unwrap_or(([0.0, 0.0, 0.0], [1.0, 1.0, 0.0]));
        let mut cam = OrbitCamera::framed(min, max);
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
        let (gx, gy, size) = gizmo_rect(bounds.width, bounds.height);
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
        match mouse_event {
            Mouse::ButtonPressed(button) => {
                let pos = cursor.position_over(bounds)?;
                // A left-click on a gizmo face snaps the view to that side.
                if matches!(button, Button::Left) {
                    if let Some((yaw, pitch)) = self.gizmo_pick(pos, bounds) {
                        return Some(
                            shader::Action::publish(Message::SetView(yaw, pitch)).and_capture(),
                        );
                    }
                }
                let mode = match button {
                    Button::Left => DragMode::Orbit,
                    Button::Right => DragMode::Pan,
                    _ => return None,
                };
                state.drag = Some(mode);
                state.last = Some(pos);
                return Some(shader::Action::capture());
            }
            Mouse::CursorMoved { position } => {
                if let (Some(mode), Some(last)) = (state.drag, state.last) {
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
            }
            Mouse::ButtonReleased(_) => {
                if state.drag.take().is_some() {
                    state.last = None;
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
        _cursor: iced::mouse::Cursor,
        bounds: iced::Rectangle,
    ) -> Self::Primitive {
        let aspect = if bounds.height > 0.0 {
            bounds.width / bounds.height
        } else {
            1.0
        };
        ScenePrimitive {
            vertices: self.vertices.clone(),
            mesh_vertices: self.mesh_vertices.clone(),
            mesh_indices: self.mesh_indices.clone(),
            view_proj: self.camera().view_proj(aspect),
            gizmo_view_proj: self.gizmo_camera().view_proj(1.0),
            show_gizmo: self.show_gizmo,
        }
    }

    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: iced::Rectangle,
        cursor: iced::mouse::Cursor,
    ) -> iced::mouse::Interaction {
        if state.drag.is_some() {
            iced::mouse::Interaction::Grabbing
        } else if cursor.position_over(bounds).is_some() {
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
    lines: cam_render::LineRenderer,
    mesh: cam_render::MeshRenderer,
    gizmo: cam_render::GizmoRenderer,
    depth: Option<DepthTarget>,
}

impl shader::Pipeline for ViewportPipeline {
    fn new(device: &wgpu::Device, queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        ViewportPipeline {
            lines: cam_render::LineRenderer::new(device, format),
            mesh: cam_render::MeshRenderer::new(device, format),
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

#[derive(Debug)]
struct ScenePrimitive {
    vertices: Arc<Vec<Vertex>>,
    mesh_vertices: Arc<Vec<MeshVertex>>,
    mesh_indices: Arc<Vec<u32>>,
    view_proj: [[f32; 4]; 4],
    /// The orientation cube's view-projection (same rotation, its own framing).
    gizmo_view_proj: [[f32; 4]; 4],
    /// Whether to draw the orientation cube.
    show_gizmo: bool,
}

impl shader::Primitive for ScenePrimitive {
    type Pipeline = ViewportPipeline;

    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _bounds: &iced::Rectangle,
        viewport: &shader::Viewport,
    ) {
        pipeline.lines.upload(device, &self.vertices);
        pipeline
            .mesh
            .upload(device, &self.mesh_vertices, &self.mesh_indices);
        pipeline.lines.set_camera(queue, self.view_proj);
        pipeline.mesh.set_camera(queue, self.view_proj);
        pipeline.gizmo.set_camera(queue, self.gizmo_view_proj);
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
        pipeline.mesh.draw(&mut pass);
        pipeline.lines.draw(&mut pass);
        drop(pass);

        // Second pass: the orientation cube in the top-right corner, with its own
        // depth cleared so the part never occludes it. Same rect as the click
        // hit-test (gizmo_rect), offset into the widget's clip region.
        if !self.show_gizmo {
            return;
        }
        let (lx, ly, size) = gizmo_rect(clip_bounds.width as f32, clip_bounds.height as f32);
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
