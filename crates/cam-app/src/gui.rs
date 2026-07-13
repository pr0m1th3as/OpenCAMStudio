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
use cam_render::{MeshVertex, Scene, Vertex, PART};
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
    status: String,
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
            Message::Export => match self.controller.export_nc() {
                Ok(nc) => self.status = format!("Exported {} lines of G-code.", nc.lines().count()),
                Err(e) => self.status = format!("Export blocked: {e:?}"),
            },
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
                shader(Viewport::new(&self.controller, self.show_stock))
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
// The wgpu viewport, hosted in iced's shader widget.
// ---------------------------------------------------------------------------

struct Viewport {
    vertices: Arc<Vec<Vertex>>,
    mesh_vertices: Arc<Vec<MeshVertex>>,
    mesh_indices: Arc<Vec<u32>>,
    bounds: Option<([f32; 3], [f32; 3])>,
}

impl Viewport {
    fn new(controller: &AppController, show_stock: bool) -> Self {
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
        }
    }
}

impl<Message> shader::Program<Message> for Viewport {
    type State = ();
    type Primitive = ScenePrimitive;

    fn draw(
        &self,
        _state: &Self::State,
        _cursor: iced::mouse::Cursor,
        _bounds: iced::Rectangle,
    ) -> Self::Primitive {
        ScenePrimitive {
            vertices: self.vertices.clone(),
            mesh_vertices: self.mesh_vertices.clone(),
            mesh_indices: self.mesh_indices.clone(),
            bounds: self.bounds,
        }
    }
}

/// The shared GPU state for the viewport — iced constructs it once and hands it
/// back to us each frame. It owns both renderers: the solid stock (drawn first,
/// underneath) and the line backplot (drawn on top).
struct ViewportPipeline {
    lines: cam_render::LineRenderer,
    mesh: cam_render::MeshRenderer,
}

impl shader::Pipeline for ViewportPipeline {
    fn new(device: &wgpu::Device, _queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        ViewportPipeline {
            lines: cam_render::LineRenderer::new(device, format),
            mesh: cam_render::MeshRenderer::new(device, format),
        }
    }
}

#[derive(Debug)]
struct ScenePrimitive {
    vertices: Arc<Vec<Vertex>>,
    mesh_vertices: Arc<Vec<MeshVertex>>,
    mesh_indices: Arc<Vec<u32>>,
    bounds: Option<([f32; 3], [f32; 3])>,
}

impl shader::Primitive for ScenePrimitive {
    type Pipeline = ViewportPipeline;

    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bounds: &iced::Rectangle,
        _viewport: &shader::Viewport,
    ) {
        pipeline.lines.upload(device, &self.vertices);
        pipeline
            .mesh
            .upload(device, &self.mesh_vertices, &self.mesh_indices);
        let aspect = if bounds.height > 0.0 {
            bounds.width / bounds.height
        } else {
            1.0
        };
        let (min, max) = self.bounds.unwrap_or(([0.0, 0.0, 0.0], [1.0, 1.0, 0.0]));
        let view_proj = cam_render::top_view(min, max, aspect, 0.1);
        pipeline.lines.set_camera(queue, view_proj);
        pipeline.mesh.set_camera(queue, view_proj);
    }

    fn draw(&self, pipeline: &Self::Pipeline, render_pass: &mut wgpu::RenderPass<'_>) -> bool {
        // Solid stock first, then the backplot lines over it (no depth buffer —
        // correct for the orthographic top view; see MeshRenderer).
        pipeline.mesh.draw(render_pass);
        pipeline.lines.draw(render_pass);
        true
    }
}
