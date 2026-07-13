//! The iced desktop shell — a thin view over [`crate::AppController`].
//!
//! Controls on the left (open a sample part, edit parameters, run, undo/redo,
//! export); a `wgpu` viewport on the right (the [`cam_render`] backplot, drawn
//! through iced's shader widget). All behaviour is delegated to the controller;
//! this module only translates messages and draws.
//!
//! Only compiled with the `gui` feature.

use std::sync::Arc;

use iced::widget::{button, column, container, row, scrollable, shader, text, text_input, Space};
use iced::{Alignment, Element, Length};

use cam_model::{Envelope, Machine, Point3};
use cam_render::{MeshVertex, Scene, Vertex, PART};
use cam_toolpath::{CancelToken, Severity};

use crate::AppController;

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

/// A fixed vertical gap.
fn gap(height: f32) -> Space {
    Space::new().height(Length::Fixed(height))
}

struct App {
    controller: AppController,
    tool_diameter: String,
    depth: String,
    stepdown: String,
    status: String,
    /// Whether the viewport overlays the simulated stock surface.
    show_stock: bool,
}

#[derive(Debug, Clone)]
enum Message {
    OpenSample,
    ToolDiameter(String),
    Depth(String),
    Stepdown(String),
    /// Commit the edited fields (one undo step) and recompute the toolpath.
    Apply,
    Export,
    Undo,
    Redo,
    /// Show or hide the simulated stock in the viewport.
    ToggleStock,
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

impl App {
    fn new() -> (Self, iced::Task<Message>) {
        let controller = AppController::new(default_machine());
        let p = *controller.params();
        (
            Self {
                tool_diameter: p.tool_diameter.to_string(),
                depth: p.depth.to_string(),
                stepdown: p.stepdown.to_string(),
                status: "Open the sample part to begin.".to_string(),
                show_stock: false,
                controller,
            },
            iced::Task::none(),
        )
    }

    fn sync_fields(&mut self) {
        let p = *self.controller.params();
        self.tool_diameter = p.tool_diameter.to_string();
        self.depth = p.depth.to_string();
        self.stepdown = p.stepdown.to_string();
    }

    fn update(&mut self, message: Message) -> iced::Task<Message> {
        match message {
            Message::OpenSample => match self.controller.open_dxf(SAMPLE_DXF, "sample.dxf") {
                Ok(n) => {
                    self.status = format!("Imported {n} region(s).");
                    self.rerun();
                }
                Err(e) => self.status = format!("Import failed: {e}"),
            },
            // Field edits only touch the local text buffer; nothing is applied or
            // recomputed until Apply/Run so undo has one step per real change.
            Message::ToolDiameter(v) => self.tool_diameter = v,
            Message::Depth(v) => self.depth = v,
            Message::Stepdown(v) => self.stepdown = v,
            Message::Apply => self.apply_and_run(),
            Message::Export => match self.controller.export_nc() {
                Ok(nc) => self.status = format!("Exported {} lines of G-code.", nc.lines().count()),
                Err(e) => self.status = format!("Export blocked: {e:?}"),
            },
            Message::Undo => {
                if self.controller.undo() {
                    self.sync_fields();
                    self.rerun();
                    self.status = format!("Undid change. {}", self.status);
                } else {
                    self.status = "Nothing to undo.".to_string();
                }
            }
            Message::Redo => {
                if self.controller.redo() {
                    self.sync_fields();
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
        }
        iced::Task::none()
    }

    /// Parse the edited fields, commit them as one undoable change if they differ
    /// from the current parameters, and recompute.
    fn apply_and_run(&mut self) {
        let (Ok(tool), Ok(depth), Ok(stepdown)) = (
            self.tool_diameter.parse::<f64>(),
            self.depth.parse::<f64>(),
            self.stepdown.parse::<f64>(),
        ) else {
            self.status = "A parameter field is not a valid number.".to_string();
            return;
        };
        let mut target = *self.controller.params();
        target.tool_diameter = tool;
        target.depth = depth;
        target.stepdown = stepdown;
        if target != *self.controller.params() {
            self.controller.edit_params(|p| *p = target);
        }
        self.rerun();
    }

    /// Recompute the toolpath for the current parameters and report the result.
    fn rerun(&mut self) {
        if self.controller.regions().is_empty() {
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
            format!("{errors} error(s) — see diagnostics.")
        } else {
            "Toolpath ready. Export when you like.".to_string()
        };
    }

    fn view(&self) -> Element<'_, Message> {
        let field = |label: &str, value: &str, on_change: fn(String) -> Message| {
            row![
                text(label.to_string()).width(Length::Fixed(110.0)),
                text_input("", value)
                    .on_input(on_change)
                    .on_submit(Message::Apply)
                    .width(Length::Fixed(90.0)),
            ]
            .spacing(8)
            .align_y(Alignment::Center)
        };

        let controls = column![
            text("OpenCAMStudio").size(22),
            button("Open sample part").on_press(Message::OpenSample),
            gap(8.0),
            field("Tool ⌀ (mm)", &self.tool_diameter, Message::ToolDiameter),
            field("Depth (mm)", &self.depth, Message::Depth),
            field("Stepdown (mm)", &self.stepdown, Message::Stepdown),
            text("Press Enter or Run to apply.").size(11),
            gap(8.0),
            row![
                button("Undo").on_press(Message::Undo),
                button("Redo").on_press(Message::Redo),
            ]
            .spacing(8),
            row![
                button("Run").on_press(Message::Apply),
                button("Export .nc").on_press(Message::Export),
            ]
            .spacing(8),
            button(if self.show_stock {
                "Hide stock"
            } else {
                "Show stock"
            })
            .on_press(Message::ToggleStock),
            gap(12.0),
            text(self.status.clone()),
            diagnostics_view(&self.controller),
        ]
        .spacing(8)
        .padding(16)
        .width(Length::Fixed(320.0));

        let viewport = container(
            shader(Viewport::new(&self.controller, self.show_stock))
                .width(Length::Fill)
                .height(Length::Fill),
        )
        .width(Length::Fill)
        .height(Length::Fill);

        row![controls, viewport].into()
    }
}

fn diagnostics_view(controller: &AppController) -> Element<'_, Message> {
    let Some(outcome) = controller.outcome() else {
        return Space::new().into();
    };
    if outcome.diagnostics.is_empty() {
        return Space::new().into();
    }
    let mut list = column![text("Diagnostics:")].spacing(4);
    for d in &outcome.diagnostics {
        list = list.push(text(format!("• {:?}: {}", d.severity, d.message)).size(12));
    }
    scrollable(list).height(Length::Fixed(160.0)).into()
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
