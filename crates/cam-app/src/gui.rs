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
use cam_render::Vertex;
use cam_toolpath::{CancelToken, Severity};

use crate::{AppController, JobParams};

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
        .run()
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
}

#[derive(Debug, Clone)]
enum Message {
    OpenSample,
    ToolDiameter(String),
    Depth(String),
    Stepdown(String),
    Run,
    Export,
    Undo,
    Redo,
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
                Ok(n) => self.status = format!("Imported {n} region(s). Set parameters and Run."),
                Err(e) => self.status = format!("Import failed: {e}"),
            },
            Message::ToolDiameter(v) => {
                self.tool_diameter = v.clone();
                if let Ok(d) = v.parse::<f64>() {
                    self.controller
                        .edit_params(|p: &mut JobParams| p.tool_diameter = d);
                }
            }
            Message::Depth(v) => {
                self.depth = v.clone();
                if let Ok(d) = v.parse::<f64>() {
                    self.controller.edit_params(|p: &mut JobParams| p.depth = d);
                }
            }
            Message::Stepdown(v) => {
                self.stepdown = v.clone();
                if let Ok(d) = v.parse::<f64>() {
                    self.controller
                        .edit_params(|p: &mut JobParams| p.stepdown = d);
                }
            }
            Message::Run => {
                let outcome = self.controller.run(&CancelToken::new());
                let errors = outcome
                    .diagnostics
                    .iter()
                    .filter(|d| d.severity == Severity::Error)
                    .count();
                let strips = outcome.scene.strips.len();
                self.status = if errors > 0 {
                    format!("Run produced {errors} error(s) — see diagnostics.")
                } else {
                    format!("Ran OK: {strips} backplot/outline strips. Export when ready.")
                };
            }
            Message::Export => match self.controller.export_nc() {
                Ok(nc) => self.status = format!("Exported {} lines of G-code.", nc.lines().count()),
                Err(e) => self.status = format!("Export blocked: {e:?}"),
            },
            Message::Undo => {
                if self.controller.undo() {
                    self.sync_fields();
                    self.status = "Undid last change.".to_string();
                }
            }
            Message::Redo => {
                if self.controller.redo() {
                    self.sync_fields();
                    self.status = "Redid change.".to_string();
                }
            }
        }
        iced::Task::none()
    }

    fn view(&self) -> Element<'_, Message> {
        let field = |label: &str, value: &str, on_change: fn(String) -> Message| {
            row![
                text(label.to_string()).width(Length::Fixed(110.0)),
                text_input("", value)
                    .on_input(on_change)
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
            gap(8.0),
            row![
                button("Undo").on_press(Message::Undo),
                button("Redo").on_press(Message::Redo),
            ]
            .spacing(8),
            row![
                button("Run").on_press(Message::Run),
                button("Export .nc").on_press(Message::Export),
            ]
            .spacing(8),
            gap(12.0),
            text(self.status.clone()),
            diagnostics_view(&self.controller),
        ]
        .spacing(8)
        .padding(16)
        .width(Length::Fixed(320.0));

        let viewport = container(
            shader(Viewport::new(&self.controller))
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
    bounds: Option<([f32; 3], [f32; 3])>,
}

impl Viewport {
    fn new(controller: &AppController) -> Self {
        let scene = controller
            .outcome()
            .map(|o| o.scene.clone())
            .unwrap_or_default();
        Self {
            vertices: Arc::new(scene.line_vertices()),
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
            bounds: self.bounds,
        }
    }
}

/// The shared GPU state for the viewport — iced constructs it once and hands it
/// back to us each frame.
struct ViewportPipeline(cam_render::LineRenderer);

impl shader::Pipeline for ViewportPipeline {
    fn new(device: &wgpu::Device, _queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        ViewportPipeline(cam_render::LineRenderer::new(device, format))
    }
}

#[derive(Debug)]
struct ScenePrimitive {
    vertices: Arc<Vec<Vertex>>,
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
        pipeline.0.upload(device, &self.vertices);
        let aspect = if bounds.height > 0.0 {
            bounds.width / bounds.height
        } else {
            1.0
        };
        let (min, max) = self.bounds.unwrap_or(([0.0, 0.0, 0.0], [1.0, 1.0, 0.0]));
        pipeline
            .0
            .set_camera(queue, cam_render::top_view(min, max, aspect, 0.1));
    }

    fn draw(&self, pipeline: &Self::Pipeline, render_pass: &mut wgpu::RenderPass<'_>) -> bool {
        pipeline.0.draw(render_pass);
        true
    }
}
