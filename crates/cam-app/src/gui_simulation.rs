//! Video-style playback uses a separate, cancellable preview lane. The verified
//! machining outcome remains intact for diagnostics and post processing.
use super::*;
use crate::jobs::{JobInputs, JobService};
use crate::settings::{
    SimulationPrefs, SimulationRotaryView, SimulationStockColoring, SimulationToolDisplay,
    SimulationToolpathColoring, SimulationToolpathMode,
};
use cam_cldata::{ArcDir, Point3, Program, Step};
use iced::widget::column;
use std::time::Instant;
#[path = "gui_simulation_options.rs"]
mod options;
pub(super) use options::{SimulationOption, SimulationTab};
#[path = "gui_simulation_forces.rs"]
mod forces;

struct TimedStep {
    step: Step,
    from: Option<Point3>,
    rotary_from: f64,
    rotary_frame: Option<(cam_cldata::RotaryAxis, Point3)>,
    start: f64,
    end: f64,
}

struct Timeline {
    steps: Vec<TimedStep>,
    duration: f64,
}

fn distance(a: Point3, b: Point3) -> f64 {
    (a.x - b.x).hypot(a.y - b.y).hypot(a.z - b.z)
}

fn arc_sweep(a: Point3, end: Point3, center: Point3, dir: ArcDir) -> f64 {
    let start = (a.y - center.y).atan2(a.x - center.x);
    let finish = (end.y - center.y).atan2(end.x - center.x);
    let sweep = match dir {
        ArcDir::Ccw => (finish - start).rem_euclid(std::f64::consts::TAU),
        ArcDir::Cw => -(start - finish).rem_euclid(std::f64::consts::TAU),
    };
    if sweep.abs() < 1e-10 {
        if dir == ArcDir::Ccw {
            std::f64::consts::TAU
        } else {
            -std::f64::consts::TAU
        }
    } else {
        sweep
    }
}

impl Timeline {
    fn new(program: &Program, rapid_rate: f64) -> Result<Self, String> {
        let mut expanded = Vec::new();
        // Expand cycles so the timeline can stop inside a hole, including pecks
        // and dwells, instead of treating an entire drill cycle as one frame.
        for step in &program.steps {
            if let Step::Drill(c) = step {
                let pecks = c
                    .peck
                    .map_or(1., |peck| ((c.z_top - c.depth) / peck).ceil());
                if !pecks.is_finite() || pecks * c.points.len() as f64 > 100_000. {
                    return Err("Simulation playback exceeds 100000 drilling pecks.".into());
                }
                for &[x, y] in &c.points {
                    let rapid = |z, kind| Step::Rapid {
                        to: Point3::new(x, y, z),
                        tag: cam_cldata::Tag { kind, ..c.tag },
                    };
                    expanded.push(rapid(c.retract, cam_cldata::MoveKind::Link));
                    expanded.push(rapid(c.z_top, cam_cldata::MoveKind::Link));
                    let mut top = c.z_top;
                    loop {
                        let target = c.peck.map_or(c.depth, |peck| (top - peck).max(c.depth));
                        if target >= top {
                            return Err("Drilling peck cannot advance at this depth.".into());
                        }
                        if top < c.z_top {
                            expanded
                                .push(rapid((top + 0.5).min(c.z_top), cam_cldata::MoveKind::Link));
                        }
                        expanded.push(Step::Linear {
                            to: Point3::new(x, y, target),
                            feed: c.feed,
                            tag: cam_cldata::Tag {
                                kind: cam_cldata::MoveKind::Plunge,
                                ..c.tag
                            },
                        });
                        if target <= c.depth {
                            if let Some(seconds) = c.dwell {
                                expanded.push(Step::Dwell { seconds });
                            }
                        }
                        expanded.push(rapid(c.retract, cam_cldata::MoveKind::Retract));
                        top = target;
                        if target <= c.depth {
                            break;
                        }
                    }
                }
            } else {
                expanded.push(step.clone());
            }
        }
        let mut steps = Vec::with_capacity(expanded.len());
        let mut position = None;
        let mut duration = 0.;
        let mut rotary_angle = 0.;
        let mut rotary_frame = None;
        for step in expanded {
            if let Step::Rotary { axis, center, .. } = &step {
                rotary_frame = Some((*axis, *center));
            }
            let from = position;
            let rotary_from = rotary_angle;
            let seconds = match &step {
                Step::Rotary {
                    to,
                    angle,
                    length,
                    feed,
                    ..
                } => {
                    position = Some(*to);
                    rotary_angle = *angle;
                    length / feed.unwrap_or(rapid_rate).max(1e-6) * 60.
                }
                Step::Datum(_) => {
                    position = None;
                    0.
                }
                Step::Rapid { to, .. } => {
                    position = Some(*to);
                    from.map_or(0., |a| distance(a, *to) / rapid_rate.max(1.) * 60.)
                }
                Step::Linear { to, feed, .. } => {
                    position = Some(*to);
                    from.map_or(0., |a| distance(a, *to) / feed.max(1e-6) * 60.)
                }
                Step::Arc {
                    end,
                    center,
                    dir,
                    feed,
                    ..
                } => {
                    position = Some(*end);
                    from.map_or(0., |a| {
                        let radius = (a.x - center.x).hypot(a.y - center.y);
                        let length =
                            (radius * arc_sweep(a, *end, *center, *dir)).hypot(end.z - a.z);
                        length / feed.max(1e-6) * 60.
                    })
                }
                Step::Dwell { seconds } => *seconds,
                _ => 0.,
            };
            let start = duration;
            duration += seconds.max(0.);
            steps.push(TimedStep {
                step,
                from,
                rotary_from,
                rotary_frame,
                start,
                end: duration,
            });
        }
        Ok(Self { steps, duration })
    }

    fn prefix(&self, seconds: f64) -> Program {
        let seconds = seconds.clamp(0., self.duration);
        let count = self.steps.partition_point(|step| step.end <= seconds);
        let mut steps: Vec<_> = self.steps[..count].iter().map(|s| s.step.clone()).collect();
        if let Some(timed) = self.steps.get(count) {
            let t = ((seconds - timed.start) / (timed.end - timed.start)).clamp(0., 1.);
            if t > 0. {
                let mut step = timed.step.clone();
                if let Some(a) = timed.from {
                    match &mut step {
                        Step::Rotary {
                            to, angle, length, ..
                        } => {
                            *to = Point3::new(
                                a.x + (to.x - a.x) * t,
                                a.y + (to.y - a.y) * t,
                                a.z + (to.z - a.z) * t,
                            );
                            *angle = timed.rotary_from + (*angle - timed.rotary_from) * t;
                            *length *= t;
                        }
                        Step::Rapid { to, .. } | Step::Linear { to, .. } => {
                            *to = Point3::new(
                                a.x + (to.x - a.x) * t,
                                a.y + (to.y - a.y) * t,
                                a.z + (to.z - a.z) * t,
                            );
                        }
                        Step::Arc {
                            end, center, dir, ..
                        } => {
                            let angle = (a.y - center.y).atan2(a.x - center.x)
                                + arc_sweep(a, *end, *center, *dir) * t;
                            let radius = (a.x - center.x).hypot(a.y - center.y);
                            *end = Point3::new(
                                center.x + radius * angle.cos(),
                                center.y + radius * angle.sin(),
                                a.z + (end.z - a.z) * t,
                            );
                        }
                        Step::Dwell { seconds } => *seconds *= t,
                        _ => {}
                    }
                }
                steps.push(step);
            }
        }
        Program { steps }
    }
}

/// Geometry, tool position and comparison colors are published as one frame.
struct StockFrame {
    outcome: crate::RunOutcome,
    position: f64,
    stock_position: f64,
}

pub(super) struct SimulationPlayer {
    source: crate::JobStamp,
    inputs: JobInputs,
    timeline: Timeline,
    path_operation: Option<u32>,
    jobs: JobService,
    force_jobs: JobService,
    force_trace: Option<forces::ForceTrace>,
    force_error: Option<String>,
    force_material: Option<cam_model::Material>,
    pub preview: crate::RunOutcome,
    /// Identity of the displayed geometry, retained by the viewport cache.
    pub(super) display_token: Arc<()>,
    pub position: f64,
    requested: f64,
    rendered: f64,
    pub playing: bool,
    pub speed: f64,
    error: Option<String>,
    collision: Option<cam_sim::Collision>,
    acknowledged_step: Option<usize>,
    collision_floor: f64,
    last_tick: Instant,
    pub options: SimulationPrefs,
    pub tab: SimulationTab,
    pub sections: [bool; 4],
    stock_position: f64,
    target: Arc<CadScene>,
    comparison: crate::stock_comparison::StockComparison,
    comparison_classes: Vec<Option<cam_render::SurfaceClass>>,
    pending_frame: Option<StockFrame>,
    custom_stock_color: [f32; 3],
}

impl SimulationPlayer {
    pub fn new_for_operation(
        controller: &AppController,
        operation: Option<u32>,
    ) -> Result<Self, String> {
        let outcome = controller.outcome().ok_or("No toolpath to simulate.")?;
        let source = outcome
            .stamp
            .clone()
            .ok_or("Run the toolpath before playback.")?;
        let timeline = Timeline::new(&outcome.program, controller.machine().rapid_rate)?;
        let position = operation
            .and_then(|id| {
                timeline.steps.iter().find_map(|step| {
                    let tag = match &step.step {
                        Step::Rotary { tag, .. }
                        | Step::Rapid { tag, .. }
                        | Step::Linear { tag, .. }
                        | Step::Arc { tag, .. } => Some(tag),
                        _ => None,
                    }?;
                    (tag.op_id == id).then_some(step.start)
                })
            })
            .unwrap_or(0.);
        let preview = crate::RunOutcome {
            program: timeline.prefix(position),
            ..Default::default()
        };
        let mut inputs = controller.job_inputs();
        inputs.preview_resolution = Some(inputs.stock_resolution(&outcome.program));
        let mut geometry = controller.geometry_state().clone();
        let setup = controller.inspection_setup();
        for instance in &mut geometry.instances {
            instance.visible &= setup.model_instances.contains(&instance.id)
                && geometry
                    .assets
                    .iter()
                    .find(|asset| asset.id == instance.asset_id)
                    .is_some_and(|asset| {
                        asset
                            .topology
                            .bodies
                            .iter()
                            .any(|body| body.id == instance.body_id && body.valid_solid)
                    });
        }
        let target = Arc::new(CadScene::from_geometry(&geometry, &Default::default()));
        let mut player = Self {
            source,
            inputs,
            timeline,
            path_operation: operation,
            jobs: JobService::default(),
            force_jobs: JobService::default(),
            force_trace: None,
            force_error: None,
            force_material: None,
            preview,
            display_token: Arc::new(()),
            position,
            requested: position,
            rendered: position,
            playing: false,
            speed: 1.,
            error: None,
            collision: None,
            acknowledged_step: None,
            collision_floor: position,
            last_tick: Instant::now(),
            options: SimulationPrefs::default(),
            tab: SimulationTab::Display,
            sections: [true; 4],
            stock_position: position,
            target,
            comparison: Default::default(),
            comparison_classes: Vec::new(),
            pending_frame: None,
            custom_stock_color: [0.2, 0.65, 0.3],
        };
        player.request_frame();
        Ok(player)
    }
    pub fn current(&self, controller: &AppController) -> bool {
        controller.revision() == self.source.revision
            && controller.outcome().and_then(|o| o.stamp.as_ref()) == Some(&self.source)
    }
    fn request_frame(&mut self) {
        if self.pending_frame.take().is_some() {
            self.comparison.invalidate();
        }
        let mut inputs = self.inputs.clone();
        inputs.preview_program = Some(self.timeline.prefix(self.position));
        inputs.stock_force_surfaces =
            self.options.stock_coloring == SimulationStockColoring::CuttingForce;
        self.requested = self.position;
        self.jobs.start(
            self.source.revision,
            format!("playback:{}", self.position),
            inputs,
            Default::default(),
        );
    }
    pub fn seek(&mut self, seconds: f64) {
        self.error = None;
        self.collision = None;
        self.acknowledged_step = None;
        self.collision_floor = seconds.clamp(0., self.timeline.duration);
        self.playing = false;
        self.position = seconds.clamp(0., self.timeline.duration);
        self.last_tick = Instant::now();
        self.request_frame();
    }
    pub fn seek_end(&mut self, end: bool) {
        self.seek(if end { self.timeline.duration } else { 0. });
    }
    pub fn seek_point(&mut self, forward: bool) {
        let points: Vec<f64> = self
            .timeline
            .steps
            .iter()
            .filter(|step| {
                matches!(
                    &step.step,
                    Step::Rotary { .. }
                        | Step::Rapid { .. }
                        | Step::Linear { .. }
                        | Step::Arc { .. }
                )
            })
            .map(|step| if forward { step.end } else { step.start })
            .collect();
        let target = if forward {
            points
                .into_iter()
                .find(|time| *time > self.position + 1e-6)
                .unwrap_or(self.timeline.duration)
        } else {
            points
                .into_iter()
                .rev()
                .find(|time| *time < self.position - 1e-6)
                .unwrap_or(0.)
        };
        self.seek(target);
    }
    pub fn seek_operation(&mut self, forward: bool) {
        let mut operations: Vec<(u32, f64)> = Vec::new();
        for step in &self.timeline.steps {
            let op_id = match &step.step {
                Step::Rotary { tag, .. }
                | Step::Rapid { tag, .. }
                | Step::Linear { tag, .. }
                | Step::Arc { tag, .. } => Some(tag.op_id),
                _ => None,
            };
            if let Some(id) = op_id {
                if operations.last().map(|entry| entry.0) != Some(id) {
                    operations.push((id, step.start));
                }
            }
        }
        let target = if forward {
            operations
                .iter()
                .find(|(_, time)| *time > self.position + 1e-6)
                .map(|(_, time)| *time)
                .unwrap_or(self.timeline.duration)
        } else {
            operations
                .iter()
                .rev()
                .find(|(_, time)| *time < self.position - 1e-6)
                .map(|(_, time)| *time)
                .unwrap_or(0.)
        };
        self.seek(target);
    }
    pub fn toggle(&mut self) {
        if self.position >= self.timeline.duration {
            self.seek(0.);
        }
        if let Some(collision) = self.collision.take() {
            self.acknowledged_step = collision.step_index;
        }
        self.playing = !self.playing;
        self.last_tick = Instant::now();
    }
    pub fn tick(&mut self) {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_tick).as_secs_f64();
        self.last_tick = now;
        self.poll_forces();
        if let Some(result) = self.jobs.poll(self.source.revision) {
            if result.outcome.completion == JobCompletion::Complete {
                self.accept_frame(result.outcome);
            } else {
                self.playing = false;
                self.rendered = self.requested;
                let reason = result
                    .outcome
                    .diagnostics
                    .iter()
                    .find(|diagnostic| diagnostic.severity == cam_toolpath::Severity::Error)
                    .map(|diagnostic| diagnostic.message.as_str())
                    .unwrap_or("Unknown error");
                self.error = Some(format!(
                    "Simulation preview failed: {reason}. Seek again to retry."
                ));
            }
        }
        if let Some(classes) = self.comparison.poll() {
            self.comparison_classes = classes;
            if let Some(frame) = self.pending_frame.take() {
                self.publish_frame(frame);
            } else {
                self.refresh_display();
            }
        }
        if self.playing {
            self.position = (self.position + elapsed * self.speed).min(self.timeline.duration);
            if self.position >= self.timeline.duration {
                self.playing = false;
            }
        }
        if !self.jobs.pending()
            && self.pending_frame.is_none()
            && (self.position - self.rendered).abs() > 1e-6
        {
            self.request_frame();
        }
    }
    fn accept_frame(&mut self, outcome: crate::RunOutcome) {
        // The last motion may be only a fraction of the full timeline step.
        // Its collision fraction belongs to that requested prefix.
        let contact = outcome
            .collisions
            .iter()
            .filter_map(|collision| {
                let index = collision.step_index?;
                if !collision.kind.stops_playback()
                    || self.acknowledged_step.is_some_and(|ack| index <= ack)
                {
                    return None;
                }
                let step = self.timeline.steps.get(index)?;
                let end = step.end.min(self.requested);
                let time = step.start + collision.fraction * (end - step.start);
                (time + 1e-6 >= self.collision_floor).then_some((time, collision))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(time, collision)| (time, collision.clone()));
        if self.collision.is_none() && self.options.stop_on_collision {
            if let Some((time, collision)) = contact {
                self.collision = Some(collision.clone());
                self.playing = false;
                self.position = time;
                // Retain the earlier stock until verification of the contact
                // frame finishes; never publish material removed after contact.
                self.preview.program = self.timeline.prefix(time);
                self.display_token = Arc::new(());
                self.request_frame();
                return;
            }
        }
        let mut outcome = outcome;
        // The last prefix motion ends at the requested frame. Convert its local
        // fractions back to the full-program force trace before publishing.
        if let Some(index) = outcome.program.steps.len().checked_sub(1) {
            if let Some(step) = self.timeline.steps.get(index).filter(|s| s.end > s.start) {
                let scale = ((self.requested - step.start) / (step.end - step.start)).clamp(0., 1.);
                for source in outcome.stock_cut_sources.iter_mut().flatten() {
                    if source.step_index == index {
                        source.fraction *= scale;
                    }
                }
            }
        }
        if !self.options.stop_on_collision {
            self.collision = contact.map(|(_, collision)| collision.clone());
        }
        let stock_position =
            if !self.options.regenerate_on_rewind && self.requested < self.stock_position {
                outcome.stock_vertices = self.preview.stock_vertices.clone();
                outcome.stock_indices = self.preview.stock_indices.clone();
                outcome.stock_machined = self.preview.stock_machined.clone();
                outcome.stock_operations = self.preview.stock_operations.clone();
                outcome.stock_cut_sources = self.preview.stock_cut_sources.clone();
                self.stock_position
            } else {
                self.requested
            };
        self.prepare_frame(StockFrame {
            outcome,
            position: self.requested,
            stock_position,
        });
    }
    fn prepare_frame(&mut self, frame: StockFrame) {
        self.comparison.invalidate();
        self.comparison_classes.clear();
        if self.options.stock_coloring == SimulationStockColoring::Comparison {
            if let Some(classes) = self.comparison.request(
                &self.target,
                &frame.outcome.stock_vertices,
                f64::from(self.options.comparison_tolerance),
            ) {
                self.comparison_classes = classes;
            } else {
                // Keep the last complete frame visible. Publishing gray stock
                // here would flash on every asynchronous comparison update.
                self.pending_frame = Some(frame);
                return;
            }
        }
        self.publish_frame(frame);
    }
    fn publish_frame(&mut self, frame: StockFrame) {
        self.preview = frame.outcome;
        self.rendered = frame.position;
        self.stock_position = frame.stock_position;
        self.refresh_display();
    }
    pub(super) fn collision_active(&self) -> bool {
        self.collision.is_some()
    }
    pub fn needs_tick(&self) -> bool {
        self.playing
            || self.jobs.needs_poll()
            || self.force_jobs.needs_poll()
            || self.comparison.pending()
    }
    pub(super) fn rotary_pose(&self) -> Option<(cam_cldata::RotaryAxis, Point3, f64)> {
        self.preview
            .program
            .steps
            .iter()
            .rev()
            .find_map(|step| match step {
                Step::Rotary {
                    axis,
                    center,
                    angle,
                    ..
                } => Some((*axis, *center, *angle)),
                _ => None,
            })
    }

    /// Follow the rendered frame, including its active tool, when seeking in
    /// either direction. The tool and stock must describe the same instant.
    pub fn tool_mesh(&self) -> (Vec<MeshVertex>, Vec<u32>) {
        if !self.options.show_tool {
            return (vec![], vec![]);
        }
        let program = &self.preview.program;
        let tip = program.steps.iter().rev().find_map(|step| match step {
            Step::Rotary { to, .. } | Step::Rapid { to, .. } | Step::Linear { to, .. } => Some(*to),
            Step::Arc { end, .. } => Some(*end),
            _ => None,
        });
        let number = program.steps.iter().rev().find_map(|step| match step {
            Step::ToolChange { tool } => Some(*tool),
            _ => None,
        });
        let setup = self.inputs.document.setup();
        let rotary = (self.options.rotary_view == SimulationRotaryView::FixedStock)
            .then(|| self.rotary_pose())
            .flatten();
        let tool = setup.tools.iter().find(|tool| Some(tool.number) == number);
        let (Some(tip), Some(tool)) = (tip, tool) else {
            return (vec![], vec![]);
        };
        let mut profile = tool.profile();
        if self.options.tool_display == SimulationToolDisplay::Cutter {
            let last = profile.segs.iter().rposition(|segment| segment.cutting);
            if let Some(last) = last {
                profile.segs.truncate(last + 1);
                let top = profile.segs[last].end.y;
                profile.segs.push(cam_geo::ProfileSeg {
                    shape: cam_geo::SegShape::Line,
                    end: cam_geo::Point::new(0., top),
                    cutting: true,
                });
            }
        }
        let (mut vertices, mut indices) = cam_render::tool_mesh(&profile);
        if self.options.tool_display == SimulationToolDisplay::Holder {
            // The project has no holder dimensions yet. Show a schematic holder,
            // explicitly described in the panel, rather than invent collision data.
            let radius = tool.shank_diameter.max(tool.diameter) * 1.25;
            let z = tool.length;
            let holder = cam_geo::Profile2D {
                start: cam_geo::Point::new(0., z),
                segs: [
                    (radius * 0.65, z),
                    (radius, z + radius * 0.5),
                    (radius, z + radius * 2.),
                    (0., z + radius * 2.),
                ]
                .map(|(r, z)| cam_geo::ProfileSeg {
                    shape: cam_geo::SegShape::Line,
                    end: cam_geo::Point::new(r, z),
                    cutting: false,
                })
                .to_vec(),
            };
            let (mut holder_vertices, holder_indices) = cam_render::tool_mesh(&holder);
            for vertex in &mut holder_vertices {
                vertex.color = [0.15, 0.25, 0.32, 1.];
            }
            let offset = vertices.len() as u32;
            vertices.extend(holder_vertices);
            indices.extend(holder_indices.into_iter().map(|i| i + offset));
        }
        for vertex in &mut vertices {
            if self.collision.is_some() {
                vertex.color = [1., 0.05, 0.05, 1.];
            }
            if self.options.transparent_tool {
                vertex.color[3] = 0.3;
            }
            let mut point = Point3::new(
                tip.x + vertex.position[0] as f64,
                tip.y + vertex.position[1] as f64,
                tip.z + vertex.position[2] as f64,
            );
            let mut normal = Point3::new(
                vertex.normal[0] as f64,
                vertex.normal[1] as f64,
                vertex.normal[2] as f64,
            );
            if let Some((axis, center, angle)) = rotary {
                point = axis.material_point(point, center, angle);
                normal = axis.material_point(normal, Point3::new(0., 0., 0.), angle);
            }
            vertex.position = setup
                .frame
                .to_world_vector([point.x, point.y, point.z])
                .map(|v| v as f32);
            vertex.normal = setup
                .frame
                .to_world_vector([normal.x, normal.y, normal.z])
                .map(|v| v as f32);
        }
        (vertices, indices)
    }
    pub fn controls(&self) -> Element<'_, Message> {
        let duration = self.timeline.duration.max(0.001);
        let time = |s: f64| format!("{}:{:02}", s as u64 / 60, s as u64 % 60);
        let collision_status = self.collision.as_ref().map_or_else(String::new, |c| {
            format!(
                "{}: {} — X {:.3}, Y {:.3}, Z {:.3}",
                if self.options.stop_on_collision {
                    "Paused"
                } else {
                    "Collision"
                },
                c.message,
                c.at[0],
                c.at[1],
                c.at[2]
            )
        });
        container(
            column![
                row![
                    inspector_button("⏮").on_press(Message::SimulationToolpathStart),
                    inspector_button("◀| Op").on_press(Message::SimulationPreviousOperation),
                    inspector_button("◀ Point").on_press(Message::SimulationPreviousPoint),
                    inspector_button(if self.playing { "Pause" } else { "Play" })
                        .on_press(Message::SimulationPlayPause),
                    inspector_button("Point ▶").on_press(Message::SimulationNextPoint),
                    inspector_button("Op |▶").on_press(Message::SimulationNextOperation),
                    inspector_button("⏭").on_press(Message::SimulationToolpathEnd),
                ]
                .spacing(6)
                .align_y(Alignment::Center)
                .wrap(),
                row![
                    text(format!(
                        "{} / {}",
                        time(self.position),
                        time(self.timeline.duration)
                    ))
                    .size(12),
                    text(format!("{:.0}%", self.position / duration * 100.)).size(12),
                    text(self.error.as_deref().unwrap_or("")).size(12),
                    text(format!("Speed {:.2}×", self.speed)).size(12),
                    slider(0.25..=10., self.speed, Message::SimulationSpeed)
                        .step(0.25)
                        .width(110),
                    inspector_button("Close simulation").on_press(Message::SimulationClose),
                ]
                .spacing(10)
                .align_y(Alignment::Center)
                .wrap(),
                slider(0.0..=duration, self.position, Message::SimulationSeek)
                    .step((duration / 10000.).max(0.001)),
                text(collision_status)
                    .size(12)
                    .color(iced::Color::from_rgb(1., 0.2, 0.2)),
            ]
            .spacing(6),
        )
        .padding([8, 12])
        .width(Length::Fill)
        .into()
    }
}

impl App {
    pub(super) fn simulation_controls(&self) -> Element<'_, Message> {
        self.simulation
            .as_ref()
            .map_or_else(|| Space::new().height(0).into(), SimulationPlayer::controls)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_cldata::{MoveKind, Tag};
    use std::time::Duration;

    #[test]
    #[ignore = "Set OPENCAM_SIM_PROFILE_PROJECT to a local .ocam project path"]
    fn profile_local_playback() {
        let (mut app, _) = App::new();
        app.settings = crate::Settings::default();
        app.model_view = None;
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_project(std::env::var("OPENCAM_SIM_PROFILE_PROJECT").unwrap())
            .unwrap();
        let started = Instant::now();
        app.controller.start_run();
        while app.controller.run_needs_poll() {
            app.controller.poll_run();
            assert!(started.elapsed() < Duration::from_secs(180));
            std::thread::sleep(Duration::from_millis(1));
        }
        eprintln!(
            "initial job {:?}, steps {}",
            started.elapsed(),
            app.controller.outcome().unwrap().program.steps.len()
        );
        let mut player = SimulationPlayer::new_for_operation(&app.controller, None).unwrap();
        player.options.stop_on_collision = false;
        assert!(!player.forces_enabled());
        drain_player(&mut player);
        for fraction in [0.05, 0.5, 0.95] {
            player.seek(player.timeline.duration * fraction);
            drain_player(&mut player);
            let mut frames = Vec::new();
            for _ in 0..8 {
                let started = Instant::now();
                player.seek(player.position + 0.05);
                drain_player(&mut player);
                frames.push(started.elapsed());
            }
            frames.sort();
            eprintln!(
                "frames at {fraction}: median {:?}, max {:?}, vertices {}",
                frames[frames.len() / 2],
                frames.last().unwrap(),
                player.preview.stock_vertices.len()
            );
            app.simulation = Some(player);
            let started = Instant::now();
            for _ in 0..20 {
                std::hint::black_box(app.machining_viewport());
            }
            eprintln!("20 viewport builds at {fraction}: {:?}", started.elapsed());
            player = app.simulation.take().unwrap();
        }
    }

    fn tag() -> Tag {
        Tag::new(7, MoveKind::Cutting)
    }
    fn last_position(program: &Program) -> Point3 {
        program
            .steps
            .iter()
            .rev()
            .find_map(|step| match step {
                Step::Rapid { to, .. } | Step::Linear { to, .. } => Some(*to),
                Step::Arc { end, .. } => Some(*end),
                _ => None,
            })
            .unwrap()
    }
    #[test]
    fn timeline_interpolates_feed_moves_and_rewinds_without_future_steps() {
        let timeline = Timeline::new(
            &Program {
                steps: vec![
                    Step::Rapid {
                        to: Point3::new(0., 0., 0.),
                        tag: tag(),
                    },
                    Step::Linear {
                        to: Point3::new(10., 0., 0.),
                        feed: 60.,
                        tag: tag(),
                    },
                    Step::Dwell { seconds: 2. },
                    Step::SpindleOff,
                ],
            },
            2000.,
        )
        .unwrap();
        assert_eq!(timeline.duration, 12.);
        assert_eq!(last_position(&timeline.prefix(5.)).x, 5.);
        assert!(!timeline.prefix(11.).steps.contains(&Step::SpindleOff));
        assert!(timeline.prefix(12.).steps.contains(&Step::SpindleOff));
        assert_eq!(last_position(&timeline.prefix(2.)).x, 2.);
        assert_eq!(last_position(&timeline.prefix(0.)).x, 0.);
    }
    #[test]
    fn timeline_interpolates_full_circle_helices_in_both_directions() {
        for dir in [ArcDir::Cw, ArcDir::Ccw] {
            let timeline = Timeline::new(
                &Program {
                    steps: vec![
                        Step::Rapid {
                            to: Point3::new(1., 0., 0.),
                            tag: tag(),
                        },
                        Step::Arc {
                            end: Point3::new(1., 0., -2.),
                            center: Point3::new(0., 0., 0.),
                            dir,
                            feed: 60.,
                            tag: tag(),
                        },
                    ],
                },
                2000.,
            )
            .unwrap();
            let half = timeline.prefix(timeline.duration / 2.);
            assert!(half.validate().is_ok());
            let at = last_position(&half);
            assert!((at.x + 1.).abs() < 1e-9);
            assert!((at.z + 1.).abs() < 1e-9);
        }
    }
    #[test]
    fn drilling_playback_has_partial_pecks_and_dwell_time() {
        let timeline = Timeline::new(
            &Program {
                steps: vec![Step::Drill(cam_cldata::DrillCycle {
                    points: vec![[2., 3.]],
                    z_top: 0.,
                    depth: -2.,
                    retract: 1.,
                    peck: Some(1.),
                    dwell: Some(2.),
                    feed: 60.,
                    tag: tag(),
                })],
            },
            60.,
        )
        .unwrap();
        assert!(timeline
            .steps
            .iter()
            .all(|s| !matches!(s.step, Step::Drill(_))));
        assert!((last_position(&timeline.prefix(1.5)).z + 0.5).abs() < 1e-9);
        let full = timeline.prefix(timeline.duration);
        assert!(full.steps.contains(&Step::Dwell { seconds: 2. }));
        assert_eq!(last_position(&full).z, 1.);
        let scene = Scene::from_program(&full);
        assert!(scene
            .path_cones
            .iter()
            .any(|cone| cone.color == cam_render::PATH_IN));
        assert!(scene
            .path_cones
            .iter()
            .any(|cone| cone.color == cam_render::PATH_OUT));
        assert!(scene.path_cones.iter().all(|cone| cone.op == 7));
    }
    fn drain_player(player: &mut SimulationPlayer) {
        let deadline = Instant::now() + Duration::from_secs(15);
        while player.needs_tick() {
            player.tick();
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    pub(super) fn collision_player() -> SimulationPlayer {
        let mut controller = AppController::new(crate::default_machine());
        controller.edit_setup(|setup| {
            setup.tools = vec![cam_model::Tool {
                number: 2,
                diameter: 4.,
                neck_diameter: 2.,
                length: 30.,
                flute_length: 10.,
                kind: ToolKind::ThreadMill { pitch: None },
                ..Default::default()
            }];
        });
        let program = Program {
            steps: vec![
                Step::ToolChange { tool: 2 },
                Step::Rapid {
                    to: Point3::new(0., 0., 2.),
                    tag: tag(),
                },
                Step::Linear {
                    to: Point3::new(0., 0., -3.),
                    feed: 60.,
                    tag: Tag::new(7, MoveKind::Plunge),
                },
                Step::Linear {
                    to: Point3::new(3., 0., -3.),
                    feed: 60.,
                    tag: tag(),
                },
                Step::Linear {
                    to: Point3::new(3., 3., -3.),
                    feed: 60.,
                    tag: Tag::new(7, MoveKind::Link),
                },
            ],
        };
        let mut inputs = controller.job_inputs();
        inputs.min = [-5., -5., -5.];
        inputs.max = [5., 5., 0.];
        inputs.preview_resolution = Some(0.2);
        SimulationPlayer {
            source: crate::JobStamp {
                request_id: 1,
                revision: 0,
                input_fingerprint: String::new(),
            },
            inputs,
            timeline: Timeline::new(&program, 2000.).unwrap(),
            path_operation: None,
            jobs: JobService::default(),
            force_jobs: JobService::default(),
            force_trace: None,
            force_error: None,
            force_material: None,
            preview: Default::default(),
            display_token: Arc::new(()),
            position: 0.,
            requested: 0.,
            rendered: 0.,
            playing: false,
            speed: 10.,
            error: None,
            collision: None,
            acknowledged_step: None,
            collision_floor: 0.,
            last_tick: Instant::now(),
            options: SimulationPrefs::default(),
            tab: SimulationTab::Display,
            sections: [true; 4],
            stock_position: 0.,
            target: Default::default(),
            comparison: Default::default(),
            comparison_classes: Vec::new(),
            pending_frame: None,
            custom_stock_color: [0.2, 0.65, 0.3],
        }
    }
    #[test]
    fn thread_playback_exposes_and_colors_both_flanks_and_rewinds_them() {
        let mut player = collision_player();
        let setup = Arc::make_mut(&mut player.inputs.document).setup_mut();
        setup.tools[0].diameter = 2.;
        setup.tools[0].neck_diameter = 1.;
        setup.tools.push(cam_model::Tool {
            number: 1,
            diameter: 8.,
            length: 30.,
            flute_length: 10.,
            ..Default::default()
        });
        player.inputs.min = [-6., -6., 0.];
        player.inputs.max = [6., 6., 5.];
        player.inputs.preview_resolution = Some(0.125);
        let mut program = Program {
            steps: vec![
                Step::ToolChange { tool: 1 },
                Step::Rapid {
                    to: Point3::new(0., 0., 7.),
                    tag: Tag::new(1, MoveKind::Link),
                },
                Step::Linear {
                    to: Point3::new(0., 0., 0.),
                    feed: 600.,
                    tag: Tag::new(1, MoveKind::Plunge),
                },
                Step::Rapid {
                    to: Point3::new(0., 0., 7.),
                    tag: Tag::new(1, MoveKind::Retract),
                },
                Step::ToolChange { tool: 2 },
                Step::Linear {
                    to: Point3::new(0., 0., 0.),
                    feed: 600.,
                    tag: Tag::new(2, MoveKind::Plunge),
                },
                Step::Linear {
                    to: Point3::new(3.5, 0., 0.),
                    feed: 600.,
                    tag: Tag::new(2, MoveKind::LeadIn),
                },
            ],
        };
        for turn in 0..4 {
            program.steps.push(Step::Arc {
                end: Point3::new(3.5, 0., f64::from(turn + 1)),
                center: Point3::new(0., 0., f64::from(turn)),
                dir: ArcDir::Ccw,
                feed: 600.,
                tag: Tag::new(2, MoveKind::Cutting),
            });
        }
        player.timeline = Timeline::new(&program, 2000.).unwrap();
        player.seek_end(true);
        drain_player(&mut player);
        assert!(
            player.preview.collisions.is_empty(),
            "{:?}",
            player.preview.collisions
        );
        for coloring in [
            SimulationStockColoring::Operation,
            SimulationStockColoring::Tool,
        ] {
            player.options.stock_coloring = coloring;
            player.refresh_display();
            let color = player
                .preview
                .stock_vertices
                .iter()
                .zip(&player.preview.stock_operations)
                .find_map(|(vertex, owner)| (*owner == Some(2)).then_some(vertex.color))
                .unwrap();
            assert_ne!(
                color,
                player.options.material.color(player.custom_stock_color)
            );
            let mut up = 0;
            let mut down = 0;
            for ((vertex, owner), cut) in player
                .preview
                .stock_vertices
                .iter()
                .zip(&player.preview.stock_operations)
                .zip(&player.preview.stock_machined)
            {
                if *owner == Some(2) {
                    assert!(*cut);
                    assert_eq!(vertex.color, color);
                    up += usize::from(vertex.normal[2] > 0.5);
                    down += usize::from(vertex.normal[2] < -0.5);
                }
            }
            assert!(
                up > 100 && down > 100,
                "both thread flank colors must be visible"
            );
        }
        let finished = player.preview.stock_vertices.clone();
        player.seek(0.);
        drain_player(&mut player);
        assert!(player.preview.stock_operations.iter().all(Option::is_none));
        assert!(player.preview.stock_machined.iter().all(|cut| !cut));
        player.seek_end(true);
        drain_player(&mut player);
        assert_eq!(finished, player.preview.stock_vertices);
    }

    #[test]
    fn playback_stops_at_first_contact_and_colors_the_cutter_red_even_for_large_frames() {
        // A partial plunge and a frame spanning later cuts must stop at the
        // same first contact and show no stock removed by those later cuts.
        for requested in [3., 11.] {
            let mut player = collision_player();
            player.position = requested;
            player.request_frame();
            drain_player(&mut player);
            assert!(!player.playing);
            assert_eq!(player.collision.as_ref().unwrap().step_index, Some(2));
            assert!(
                (player.position - 2.001).abs() < 0.00002,
                "{}",
                player.position
            );
            assert!((last_position(&player.preview.program).z + 0.001).abs() < 0.00002);
            assert!(player.preview.stock_machined.iter().all(|&cut| !cut));
            let (vertices, _) = player.tool_mesh();
            assert!(!vertices.is_empty());
            assert!(vertices.iter().all(|v| v.color == [1., 0.05, 0.05, 1.]));

            player.toggle();
            assert!(player.collision.is_none());
            // Advance at high speed past a second, independent collision.
            player.last_tick = Instant::now() - Duration::from_secs(10);
            player.tick();
            drain_player(&mut player);
            assert!(!player.playing);
            assert_eq!(player.collision.as_ref().unwrap().step_index, Some(4));
            assert!(player.position > 8. && player.position < 11.);

            player.seek(0.);
            drain_player(&mut player);
            assert!(player.collision.is_none());
            assert!(player
                .tool_mesh()
                .0
                .iter()
                .any(|v| v.color != [1., 0.05, 0.05, 1.]));
            player.toggle();
            player.last_tick = Instant::now() - Duration::from_secs(10);
            player.tick();
            drain_player(&mut player);
            assert_eq!(player.collision.as_ref().unwrap().step_index, Some(2));
        }
    }
    #[test]
    fn seeking_rebuilds_stock_and_preserves_verified_export() {
        let mut controller = AppController::new(crate::default_machine());
        controller.edit_stock(|stock| {
            *stock = cam_model::Stock::FixedBox {
                size: [8., 8., 4.],
                model_position: [0.; 3],
            }
        });
        controller.add_operation(Operation::Drill(cam_model::DrillOp {
            cycle: Default::default(),
            tip_through_bottom: false,
            geometry: Default::default(),
            hole_heights: Vec::new(),
            parameter_expressions: Default::default(),
            id: 0,
            tool: 1,
            points: vec![[4., 4.]],
            depth: 2.,
            start_offset: 0.,
            peck: None,
            dwell: None,
            spindle_rpm: 1000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            feed: 100.,
        }));
        controller.start_run();
        let deadline = Instant::now() + Duration::from_secs(15);
        while controller.run_needs_poll() {
            controller.poll_run();
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        let nc = controller.export_nc().unwrap().to_owned();
        let mut player = SimulationPlayer::new_for_operation(&controller, None).unwrap();
        drain_player(&mut player);
        assert!(
            !player.preview.stock_vertices.is_empty(),
            "opening playback shows the initial stock"
        );
        assert!(player.preview.stock_machined.iter().all(|&cut| !cut));
        assert_eq!(
            player.inputs.preview_resolution,
            Some(
                player
                    .inputs
                    .stock_resolution(&controller.outcome().unwrap().program)
            )
        );
        player.seek(player.timeline.duration);
        drain_player(&mut player);
        let cut = player.preview.stock_vertices.clone();
        let cut_faces = player.preview.stock_machined.clone();
        let cut_operations = player.preview.stock_operations.clone();
        assert!(!cut.is_empty());
        assert!(cut_faces.iter().any(|&cut| cut));
        player.seek(0.);
        drain_player(&mut player);
        assert!(
            cut != player.preview.stock_vertices,
            "rewinding must restore uncut stock"
        );
        assert!(player.preview.stock_machined.iter().all(|&cut| !cut));
        assert!(player.preview.stock_operations.iter().all(Option::is_none));
        assert!(player.current(&controller));
        assert_eq!(controller.export_nc().unwrap(), nc);
        player.seek(player.timeline.duration);
        drain_player(&mut player);
        assert!(
            cut == player.preview.stock_vertices,
            "seeking forward must reproduce the same stock"
        );
        assert_eq!(cut_faces, player.preview.stock_machined);
        assert_eq!(cut_operations, player.preview.stock_operations);
        player.toggle();
        assert!(player.playing);
        assert_eq!(player.position, 0.);
        player.toggle();
        assert!(!player.playing);
        controller.edit_operation(0, |op| {
            if let Operation::Drill(op) = op {
                op.depth = 1.;
            }
        });
        assert!(!player.current(&controller));
    }

    #[test]
    fn solid_tool_follows_tool_changes_and_rewind_in_rotated_setup() {
        let mut controller = AppController::new(crate::default_machine());
        controller.edit_setup(|setup| {
            setup.frame = cam_model::SetupFrame::from_xy([0., 0., 1.], [1., 0., 0.]).unwrap();
            setup.tools = vec![
                cam_model::Tool {
                    number: 1,
                    diameter: 6.,
                    length: 30.,
                    ..Default::default()
                },
                cam_model::Tool {
                    number: 2,
                    diameter: 12.,
                    length: 40.,
                    ..Default::default()
                },
            ];
        });
        let timeline = Timeline::new(
            &Program {
                steps: vec![
                    Step::ToolChange { tool: 1 },
                    Step::Rapid {
                        to: Point3::new(0., 0., 0.),
                        tag: tag(),
                    },
                    Step::Linear {
                        to: Point3::new(10., 0., 0.),
                        feed: 60.,
                        tag: tag(),
                    },
                    Step::ToolChange { tool: 2 },
                    Step::Linear {
                        to: Point3::new(10., 10., 0.),
                        feed: 60.,
                        tag: tag(),
                    },
                ],
            },
            2000.,
        )
        .unwrap();
        let mut player = SimulationPlayer {
            source: crate::JobStamp {
                request_id: 1,
                revision: 0,
                input_fingerprint: String::new(),
            },
            inputs: controller.job_inputs(),
            timeline,
            path_operation: None,
            jobs: JobService::default(),
            force_jobs: JobService::default(),
            force_trace: None,
            force_error: None,
            force_material: None,
            preview: Default::default(),
            display_token: Arc::new(()),
            position: 0.,
            requested: 0.,
            rendered: 0.,
            playing: false,
            speed: 1.,
            error: None,
            collision: None,
            acknowledged_step: None,
            collision_floor: 0.,
            last_tick: Instant::now(),
            options: SimulationPrefs::default(),
            tab: SimulationTab::Display,
            sections: [true; 4],
            stock_position: 0.,
            target: Default::default(),
            comparison: Default::default(),
            comparison_classes: Vec::new(),
            pending_frame: None,
            custom_stock_color: [0.2, 0.65, 0.3],
        };
        let frame = &controller.inspection_setup().frame;
        for (time, tip, radius, length) in [
            (5., [5., 0., 0.], 3., 30.),
            (15., [10., 5., 0.], 6., 40.),
            (5., [5., 0., 0.], 3., 30.),
        ] {
            player.preview.program = player.timeline.prefix(time);
            let (vertices, indices) = player.tool_mesh();
            assert!(!indices.is_empty());
            let points: Vec<_> = vertices
                .iter()
                .map(|v| frame.to_setup_vector(v.position.map(f64::from)))
                .collect();
            let min: [f64; 3] =
                std::array::from_fn(|i| points.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min));
            let max: [f64; 3] = std::array::from_fn(|i| {
                points
                    .iter()
                    .map(|p| p[i])
                    .fold(f64::NEG_INFINITY, f64::max)
            });
            for axis in 0..3 {
                let expected_min = tip[axis] - if axis < 2 { radius } else { 0. };
                let expected_max = tip[axis] + if axis < 2 { radius } else { length };
                assert!((min[axis] - expected_min).abs() < 1e-5);
                assert!((max[axis] - expected_max).abs() < 1e-5);
            }
        }
    }
    fn display_player(time: f64) -> SimulationPlayer {
        let mut player = collision_player();
        player.timeline = Timeline::new(
            &Program {
                steps: vec![
                    Step::ToolChange { tool: 2 },
                    Step::Rapid {
                        to: Point3::new(0., 0., 0.),
                        tag: tag(),
                    },
                    Step::Linear {
                        to: Point3::new(40., 0., 0.),
                        feed: 60.,
                        tag: tag(),
                    },
                    Step::Linear {
                        to: Point3::new(40., 10., 0.),
                        feed: 60.,
                        tag: Tag::new(8, MoveKind::Cutting),
                    },
                    Step::Rapid {
                        to: Point3::new(50., 10., 0.),
                        tag: Tag::new(8, MoveKind::Link),
                    },
                    Step::Linear {
                        to: Point3::new(50., 10., -5.),
                        feed: 60.,
                        tag: Tag::new(8, MoveKind::Plunge),
                    },
                    Step::Rapid {
                        to: Point3::new(50., 10., 0.),
                        tag: Tag::new(8, MoveKind::Retract),
                    },
                ],
            },
            60.,
        )
        .unwrap();
        player.rendered = time;
        player.position = time;
        player.preview.program = player.timeline.prefix(time);
        player
    }
    fn displayed_bounds(player: &SimulationPlayer, axis: usize) -> (f32, f32) {
        player
            .preview
            .scene
            .strips
            .iter()
            .filter(|s| s.op.is_some())
            .flat_map(|s| s.points.iter().map(|p| p[axis]))
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v), hi.max(v))
            })
    }
    #[test]
    fn simulation_modes_clip_at_the_rendered_tip_and_show_complete_current_operation() {
        let mut player = display_player(30.);
        for (mode, expected) in [
            (SimulationToolpathMode::All, (0., 50.)),
            (SimulationToolpathMode::Trailing, (0., 30.)),
            (SimulationToolpathMode::Leading, (30., 50.)),
            (SimulationToolpathMode::Operation, (0., 40.)),
            (SimulationToolpathMode::Tail, (10., 30.)),
        ] {
            player.options.toolpath_mode = mode;
            player.refresh_display();
            assert_eq!(displayed_bounds(&player, 0), expected, "{mode}");
            if mode == SimulationToolpathMode::Operation {
                assert!(player.preview.scene.strips.iter().all(|s| s.op == Some(7)));
            }
        }
        player.rendered = 40.;
        player.options.toolpath_mode = SimulationToolpathMode::Operation;
        player.refresh_display();
        assert!(player.preview.scene.strips.iter().all(|s| s.op == Some(8)));
    }
    #[test]
    fn operation_scoped_simulation_hides_predecessor_paths_in_every_mode() {
        let mut player = display_player(45.);
        player.path_operation = Some(8);
        player.options.show_points = true;
        for &mode in SimulationToolpathMode::ALL {
            player.options.toolpath_mode = mode;
            player.refresh_display();
            assert!(!player.preview.scene.strips.is_empty(), "{mode}");
            assert!(
                player.preview.scene.strips.iter().all(|s| s.op == Some(8)),
                "{mode}"
            );
            assert!(player.preview.scene.path_cones.is_empty(), "{mode}");
        }
    }
    #[test]
    fn simulation_filters_keep_motion_classes_and_points_independent() {
        let mut player = display_player(45.);
        player.options.toolpath_mode = SimulationToolpathMode::All;
        player.options.show_links = false;
        player.options.show_leads = false;
        player.refresh_display();
        assert_eq!(displayed_bounds(&player, 0), (0., 40.));
        assert!(player.preview.scene.path_cones.is_empty());
        player.options.show_cutting = false;
        player.refresh_display();
        assert!(player.preview.scene.strips.is_empty());
        assert!(player.preview.scene.path_cones.is_empty());
        player.options.show_leads = true;
        player.refresh_display();
        assert_eq!(displayed_bounds(&player, 2), (-5., 0.));
        assert!(player.preview.scene.path_cones.is_empty());
        let count = player.preview.scene.strips.len();
        player.options.show_points = true;
        player.refresh_display();
        assert!(player.preview.scene.strips.len() > count);
        player.options.show_toolpath = false;
        player.refresh_display();
        assert!(player.preview.scene.strips.is_empty());
        assert!(player.preview.scene.path_cones.is_empty());
    }
    #[test]
    fn continuous_arc_playback_draws_motion_without_preview_arrows() {
        let tag = Tag::new(7, MoveKind::Cutting);
        let with_kind = |kind| Tag { kind, ..tag };
        let program = Program {
            steps: vec![
                Step::Rapid {
                    to: Point3::new(1., 0., 5.),
                    tag: with_kind(MoveKind::Link),
                },
                Step::Linear {
                    to: Point3::new(1., 0., 0.),
                    feed: 60.,
                    tag: with_kind(MoveKind::Plunge),
                },
                Step::Arc {
                    end: Point3::new(0., 1., 0.),
                    center: Point3::new(0., 0., 0.),
                    dir: ArcDir::Ccw,
                    feed: 60.,
                    tag,
                },
                Step::Arc {
                    end: Point3::new(-1., 0., 0.),
                    center: Point3::new(0., 0., 0.),
                    dir: ArcDir::Ccw,
                    feed: 60.,
                    tag,
                },
                Step::Arc {
                    end: Point3::new(0., -1., 0.),
                    center: Point3::new(0., 0., 0.),
                    dir: ArcDir::Ccw,
                    feed: 60.,
                    tag,
                },
                Step::Arc {
                    end: Point3::new(1., 0., 0.),
                    center: Point3::new(0., 0., 0.),
                    dir: ArcDir::Ccw,
                    feed: 60.,
                    tag,
                },
                Step::Rapid {
                    to: Point3::new(1., 0., 5.),
                    tag: with_kind(MoveKind::Retract),
                },
            ],
        };
        let mut player = display_player(0.);
        player.timeline = Timeline::new(&program, 60.).unwrap();
        player.rendered = player.timeline.duration;
        player.options.toolpath_mode = SimulationToolpathMode::All;
        player.options.show_links = true;
        player.options.show_leads = true;
        player.options.show_cutting = true;
        player.refresh_display();
        assert!(player.preview.scene.path_cones.is_empty());
        let preview = Scene::from_program(&program);
        assert_eq!(player.preview.scene.strips.len(), preview.strips.len());
        let cones = &preview.path_cones;
        assert_eq!(cones.len(), 2);
        assert_eq!(
            (cones[0].color, cones[1].color),
            (cam_render::PATH_IN, cam_render::PATH_OUT)
        );
        assert_eq!(cones[0].direction, [0., 0., -1.]);
        assert_eq!(cones[1].direction, [0., 0., 1.]);
    }

    #[test]
    fn leading_and_trailing_arc_paths_meet_at_the_helical_tip() {
        let mut player = display_player(0.);
        player.timeline = Timeline::new(
            &Program {
                steps: vec![
                    Step::Rapid {
                        to: Point3::new(1., 0., 0.),
                        tag: tag(),
                    },
                    Step::Arc {
                        end: Point3::new(1., 0., -2.),
                        center: Point3::new(0., 0., 0.),
                        dir: ArcDir::Ccw,
                        feed: 60.,
                        tag: tag(),
                    },
                ],
            },
            60.,
        )
        .unwrap();
        player.rendered = player.timeline.duration / 2.;
        let mut endpoints = Vec::new();
        for mode in [
            SimulationToolpathMode::Trailing,
            SimulationToolpathMode::Leading,
        ] {
            player.options.toolpath_mode = mode;
            player.refresh_display();
            let strips = &player.preview.scene.strips;
            let point = if mode == SimulationToolpathMode::Trailing {
                *strips.last().unwrap().points.last().unwrap()
            } else {
                strips.first().unwrap().points[0]
            };
            endpoints.push(point);
        }
        assert_eq!(endpoints[0], endpoints[1]);
        assert!((endpoints[0][0] + 1.).abs() < 1e-6);
        assert!((endpoints[0][2] + 1.).abs() < 1e-6);
    }
    #[test]
    fn tool_options_change_geometry_transparency_and_programmed_point() {
        let mut player = display_player(30.);
        player.options.tool_display = SimulationToolDisplay::Cutter;
        let cutter_top = player
            .tool_mesh()
            .0
            .iter()
            .map(|v| v.position[2])
            .fold(0., f32::max);
        assert!(cutter_top > 0. && cutter_top < 10.);
        player.options.tool_display = SimulationToolDisplay::Holder;
        assert!(player.tool_mesh().0.iter().any(|v| v.position[2] > 30.));
        player.options.transparent_tool = true;
        assert!(player.tool_mesh().0.iter().all(|v| v.color[3] == 0.3));
        player.options.show_toolpath = false;
        player.options.programmed_point = true;
        player.refresh_display();
        assert_eq!(player.preview.scene.strips.len(), 3);
        player.options.show_tool = false;
        assert!(player.tool_mesh().0.is_empty());
        player.refresh_display();
        assert!(player.preview.scene.strips.is_empty());
    }
    fn comparison_player() -> SimulationPlayer {
        let mut controller = AppController::new(crate::default_machine());
        controller
            .open_project(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../testfiles/thread.ocam"
            ))
            .unwrap();
        let mut player = collision_player();
        player.target = Arc::new(CadScene::from_geometry(
            controller.geometry_state(),
            &Default::default(),
        ));
        player.preview.program = player.timeline.prefix(0.);
        player.preview.stock_vertices = vec![
            MeshVertex {
                position: [0., 0., 20.2],
                normal: [0., 0., 1.],
                color: [0.; 4],
            };
            10_000
        ];
        player.options.stock_coloring = SimulationStockColoring::Comparison;
        player.refresh_display();
        assert!(
            player.comparison.pending(),
            "comparison is queued without blocking the UI"
        );
        assert!(player.comparison_classes.is_empty());
        drain_player(&mut player);
        assert!(player
            .preview
            .stock_vertices
            .iter()
            .all(|v| v.color == [0.2, 0.4, 1., 1.]));
        player
    }

    #[test]
    fn comparison_runs_in_the_background_and_reclassifies_when_tolerance_changes() {
        if !crate::test_fixture::available("thread.ocam") {
            return;
        }
        let mut player = comparison_player();
        let previous = player.preview.stock_vertices.clone();
        let mut prefs = player.options.clone();
        prefs.comparison_tolerance = 0.5;
        player.configure(&prefs, player.custom_stock_color);
        assert!(player.comparison.pending());
        assert_eq!(player.preview.stock_vertices, previous);
        drain_player(&mut player);
        assert!(player
            .preview
            .stock_vertices
            .iter()
            .all(|v| v.color == [0.15, 0.85, 0.2, 1.]));
    }

    fn next_comparison_frame(player: &mut SimulationPlayer, position: f64, z: f32) {
        let mut vertices = player.preview.stock_vertices.clone();
        for vertex in &mut vertices {
            vertex.position[2] = z;
            vertex.color = [0.; 4];
        }
        player.position = position;
        player.requested = position;
        player.accept_frame(crate::RunOutcome {
            program: player.timeline.prefix(position),
            stock_vertices: vertices,
            ..Default::default()
        });
    }

    #[test]
    fn comparison_publishes_stock_colors_and_tool_position_together_without_gray_frames() {
        if !crate::test_fixture::available("thread.ocam") {
            return;
        }
        let mut player = comparison_player();
        for (position, z, color) in [
            (1., 20., [0.15, 0.85, 0.2, 1.]),
            (2., 20.2, [0.2, 0.4, 1., 1.]),
            (3., 20., [0.15, 0.85, 0.2, 1.]),
        ] {
            let previous = player.preview.stock_vertices.clone();
            let previous_tip = last_position(&player.preview.program);
            let previous_time = player.rendered;
            let previous_stock_time = player.stock_position;
            next_comparison_frame(&mut player, position, z);
            assert!(player.pending_frame.is_some());
            player.refresh_display();
            assert_eq!(player.preview.stock_vertices, previous);
            assert_eq!(last_position(&player.preview.program), previous_tip);
            assert_eq!(player.rendered, previous_time);
            assert_eq!(player.stock_position, previous_stock_time);
            drain_player(&mut player);
            assert!(player.pending_frame.is_none());
            assert_eq!(player.rendered, position);
            assert_eq!(player.stock_position, position);
            assert_eq!(
                last_position(&player.preview.program),
                last_position(&player.timeline.prefix(position))
            );
            assert!(player
                .preview
                .stock_vertices
                .iter()
                .all(|vertex| { vertex.position[2] == z && vertex.color == color }));
        }
    }

    #[test]
    fn pending_comparison_tracks_tolerance_and_can_be_published_in_another_color_mode() {
        if !crate::test_fixture::available("thread.ocam") {
            return;
        }
        let mut player = comparison_player();
        let previous = player.preview.stock_vertices.clone();
        next_comparison_frame(&mut player, 1., 21.);
        let mut prefs = player.options.clone();
        prefs.comparison_tolerance = 1.1;
        player.configure(&prefs, player.custom_stock_color);
        assert!(player.pending_frame.is_some());
        assert_eq!(player.preview.stock_vertices, previous);
        drain_player(&mut player);
        assert!(player
            .preview
            .stock_vertices
            .iter()
            .all(|vertex| { vertex.position[2] == 21. && vertex.color == [0.15, 0.85, 0.2, 1.] }));

        next_comparison_frame(&mut player, 2., 22.);
        prefs.stock_coloring = SimulationStockColoring::Material;
        player.configure(&prefs, player.custom_stock_color);
        assert!(player.pending_frame.is_none());
        assert!(!player.comparison.pending());
        assert_eq!(player.rendered, 2.);
        let material = player.options.material.color(player.custom_stock_color);
        assert!(player
            .preview
            .stock_vertices
            .iter()
            .all(|vertex| { vertex.position[2] == 22. && vertex.color == material }));
    }

    #[test]
    fn seeking_discards_a_pending_comparison_frame() {
        if !crate::test_fixture::available("thread.ocam") {
            return;
        }
        let mut player = comparison_player();
        next_comparison_frame(&mut player, 1., 21.);
        assert!(player.pending_frame.is_some());
        player.seek(0.);
        assert!(player.pending_frame.is_none());
        drain_player(&mut player);
        assert_eq!(player.rendered, 0.);
        assert_eq!(player.stock_position, 0.);
        assert_eq!(
            player.comparison_classes.len(),
            player.preview.stock_vertices.len()
        );
        assert!(player
            .preview
            .stock_vertices
            .iter()
            .all(|vertex| vertex.position[2] <= 0.));
    }

    #[test]
    fn comparison_keeps_stock_visible_when_rewinding_without_regeneration() {
        if !crate::test_fixture::available("thread.ocam") {
            return;
        }
        let mut player = comparison_player();
        player.stock_position = 2.;
        player.rendered = 2.;
        player.options.regenerate_on_rewind = false;
        let previous = player.preview.stock_vertices.clone();
        next_comparison_frame(&mut player, 1., 21.);
        assert!(player.pending_frame.is_some());
        assert_eq!(player.preview.stock_vertices, previous);
        drain_player(&mut player);
        assert_eq!(player.preview.stock_vertices, previous);
        assert_eq!(player.rendered, 1.);
        assert_eq!(player.stock_position, 2.);
    }

    #[test]
    fn stock_coloring_tracks_operation_and_tool_identity_and_missing_comparison_target() {
        let mut player = display_player(45.);
        player.preview.stock_vertices = vec![
            MeshVertex {
                position: [0.; 3],
                normal: [0., 0., 1.],
                color: [0.; 4]
            };
            3
        ];
        player.preview.stock_operations = vec![Some(7), Some(8), None];
        player.options.stock_coloring = SimulationStockColoring::Operation;
        player.refresh_display();
        assert_ne!(
            player.preview.stock_vertices[0].color,
            player.preview.stock_vertices[1].color
        );
        assert_eq!(
            player.preview.stock_vertices[2].color,
            player.options.material.color(player.custom_stock_color)
        );
        player.options.stock_coloring = SimulationStockColoring::Tool;
        player.refresh_display();
        assert_eq!(
            player.preview.stock_vertices[0].color,
            player.preview.stock_vertices[1].color
        );
        player.options.stock_coloring = SimulationStockColoring::Comparison;
        player.refresh_display();
        assert!(player
            .preview
            .stock_vertices
            .iter()
            .all(|v| v.color == [0.5, 0.5, 0.5, 1.]));
    }
    #[test]
    fn collisions_can_be_passed_and_rewind_can_preserve_or_regenerate_stock() {
        let mut player = collision_player();
        player.options.stop_on_collision = false;
        player.options.regenerate_on_rewind = false;
        player.seek(11.);
        drain_player(&mut player);
        assert_eq!(player.position, 11.);
        assert!(!player.preview.collisions.is_empty());
        let cut = player.preview.stock_vertices.clone();
        assert!(player.preview.stock_machined.iter().any(|v| *v));
        player.seek(1.);
        drain_player(&mut player);
        assert_eq!(player.preview.stock_vertices, cut);
        assert_eq!(last_position(&player.preview.program).z, 1.);
        let mut prefs = player.options.clone();
        prefs.regenerate_on_rewind = true;
        player.configure(&prefs, player.custom_stock_color);
        drain_player(&mut player);
        assert!(player.preview.stock_machined.iter().all(|v| !v));
        assert_ne!(player.preview.stock_vertices, cut);
    }
    #[test]
    fn indexed_path_and_tool_use_the_selected_rotary_frame() {
        let tag = Tag::new(7, MoveKind::Cutting);
        let start = Point3::new(0., 0., 10.);
        let end = Point3::new(5., 0., 10.);
        let program = Program {
            steps: vec![
                Step::ToolChange { tool: 2 },
                Step::Rapid {
                    to: start,
                    tag: Tag {
                        kind: MoveKind::Traverse,
                        ..tag
                    },
                },
                Step::Rotary {
                    to: start,
                    angle: 90.,
                    axis: cam_cldata::RotaryAxis::A,
                    center: Point3::new(0., 0., 0.),
                    feed: None,
                    length: 15.,
                    tag: Tag {
                        kind: MoveKind::Traverse,
                        ..tag
                    },
                },
                Step::Rotary {
                    to: end,
                    angle: 90.,
                    axis: cam_cldata::RotaryAxis::A,
                    center: Point3::new(0., 0., 0.),
                    feed: Some(60.),
                    length: 5.,
                    tag,
                },
            ],
        };
        let mut player = collision_player();
        player.timeline = Timeline::new(&program, 60.).unwrap();
        player.rendered = player.timeline.duration;
        player.preview.program = player.timeline.prefix(player.rendered);
        player.options.show_links = false;
        player.options.toolpath_mode = SimulationToolpathMode::All;
        player.refresh_display();
        let cutting: Vec<_> = player
            .preview
            .scene
            .strips
            .iter()
            .filter(|s| s.op == Some(7))
            .collect();
        assert!(!cutting.is_empty());
        for point in cutting.iter().flat_map(|s| &s.points) {
            assert!((point[1] - 10.).abs() < 1e-4, "{point:?}");
            assert!(point[2].abs() < 1e-4, "{point:?}");
        }
        let (fixed_tool, _) = player.tool_mesh();
        assert!(!fixed_tool.is_empty());
        let fixed_tip = fixed_tool[0].position;
        player.options.rotary_view = SimulationRotaryView::RotatingStock;
        let (machine_tool, _) = player.tool_mesh();
        let machine_tip = machine_tool[0].position;
        let mapped = cam_cldata::RotaryAxis::A.material_point(
            Point3::new(
                fixed_tip[0] as f64,
                fixed_tip[1] as f64,
                fixed_tip[2] as f64,
            ),
            Point3::new(0., 0., 0.),
            -90.,
        );
        assert!((machine_tip[0] - mapped.x as f32).abs() < 1e-4);
        assert!((machine_tip[1] - mapped.y as f32).abs() < 1e-4);
        assert!((machine_tip[2] - mapped.z as f32).abs() < 1e-4);
    }

    #[test]
    fn indexed_operation_can_simulate_from_start_through_end() {
        let mut controller = AppController::new(crate::default_machine());
        controller.edit_stock(|stock| {
            *stock = cam_model::Stock::FixedBox {
                size: [8., 8., 4.],
                model_position: [0.; 3],
            }
        });
        controller.add_operation(Operation::Drill(cam_model::DrillOp {
            cycle: Default::default(),
            tip_through_bottom: false,
            geometry: Default::default(),
            hole_heights: Vec::new(),
            parameter_expressions: Default::default(),
            id: 0,
            tool: 1,
            points: vec![[2., 2.]],
            depth: 2.,
            start_offset: 0.,
            peck: None,
            dwell: None,
            spindle_rpm: 1000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            feed: 100.,
        }));
        controller
            .set_operation_rotary_index(
                0,
                Some(cam_model::RotaryIndex {
                    axis: cam_cldata::RotaryAxis::B,
                    angle: 90.,
                    center: [4., 4., 2.],
                    ..Default::default()
                }),
            )
            .unwrap();
        controller.start_run();
        let deadline = Instant::now() + std::time::Duration::from_secs(30);
        while controller.run_needs_poll() {
            controller.poll_run();
            assert!(Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        let outcome = controller.outcome().unwrap();
        assert_eq!(
            outcome.completion,
            JobCompletion::Complete,
            "{:?}",
            outcome.diagnostics
        );
        let mut player = SimulationPlayer::new_for_operation(&controller, Some(0)).unwrap();
        drain_player(&mut player);
        assert!(player.error.is_none(), "{:?}", player.error);
        player.options.stop_on_collision = false;
        player.seek_end(true);
        drain_player(&mut player);
        assert!(player.error.is_none(), "{:?}", player.error);
        assert!(player.preview.program.has_rotary_motion());
        assert!(!player.preview.stock_indices.is_empty());
        let (axis, center, angle) = player.rotary_pose().unwrap();
        let first = player.preview.stock_vertices[0].position;
        let expected = axis.material_point(
            Point3::new(first[0] as f64, first[1] as f64, first[2] as f64),
            center,
            -angle,
        );
        let (mut app, _) = App::new();
        app.settings = crate::Settings::default();
        app.controller = controller;
        app.end_setup_draft();
        app.model_view = None;
        app.settings.simulation = player.options.clone();
        app.simulation = Some(player);
        let fixed = app.machining_viewport();
        assert_eq!(fixed.mesh_vertices[0].position, first);
        let _ = app.update(Message::SimulationOption(SimulationOption::RotaryView(
            SimulationRotaryView::RotatingStock,
        )));
        let rotating = app.machining_viewport();
        for (actual, target) in rotating.mesh_vertices[0]
            .position
            .iter()
            .zip([expected.x, expected.y, expected.z])
        {
            assert!((f64::from(*actual) - target).abs() < 1e-4);
        }
    }

    #[test]
    fn rotary_playback_interpolates_the_angle_and_material_path_duration() {
        let tag = cam_cldata::Tag::new(0, cam_cldata::MoveKind::Cutting);
        let to = Point3::new(0., 0., 20.);
        let length = 10. * std::f64::consts::PI;
        let program = Program {
            steps: vec![
                Step::Rapid { to, tag },
                Step::Rotary {
                    to,
                    angle: 0.,
                    axis: cam_cldata::RotaryAxis::A,
                    center: Point3::new(0., 0., 0.),
                    feed: None,
                    length: 0.,
                    tag,
                },
                Step::Rotary {
                    to,
                    angle: 90.,
                    axis: cam_cldata::RotaryAxis::A,
                    center: Point3::new(0., 0., 0.),
                    feed: Some(60.),
                    length,
                    tag,
                },
            ],
        };
        let timeline = Timeline::new(&program, 1000.).unwrap();
        assert!((timeline.duration - length).abs() < 1e-9);
        let prefix = timeline.prefix(length * 0.5);
        assert!(
            matches!(prefix.steps.last(),Some(Step::Rotary{angle,length:d,..}) if (*angle-45.).abs()<1e-9 && (*d-length*0.5).abs()<1e-9)
        );
        let material = prefix.rotary_backplot();
        let Step::Linear { to, .. } = material.steps.last().unwrap() else {
            panic!()
        };
        assert!((to.y - 10. * 2_f64.sqrt()).abs() < 1e-9 && (to.z - to.y).abs() < 1e-9);
        let mut player = collision_player();
        player.timeline = timeline;
        player.options.toolpath_mode = SimulationToolpathMode::Trailing;
        player.rendered = length * 0.5;
        player.preview.program = prefix;
        player.refresh_display();
        let last = player
            .preview
            .scene
            .strips
            .iter()
            .flat_map(|strip| &strip.points)
            .last()
            .unwrap();
        assert!((f64::from(last[1]) - to.y).abs() < 1e-4);
        assert!((f64::from(last[2]) - to.z).abs() < 1e-4);
    }
}
