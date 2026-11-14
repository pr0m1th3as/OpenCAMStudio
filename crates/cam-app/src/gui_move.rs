//! Transactional model and body placement editor.
use super::*;
use iced::widget::column;

#[derive(Clone, Debug)]
pub(super) struct MoveDraft {
    ids: Vec<u64>,
    label: String,
    previous_model_view: Option<TreeNode>,
    pub(super) values: [String; 9],
    active: Option<HandleEdit>,
}

#[derive(Clone, Debug)]
struct HandleEdit {
    index: usize,
    base: [f64; 9],
    direction: [f64; 3],
    text: String,
}

impl MoveDraft {
    fn handle_is_valid(&self) -> bool {
        self.active
            .as_ref()
            .is_none_or(|edit| edit.text.trim().parse::<f64>().is_ok_and(f64::is_finite))
    }
    pub(super) fn clear_handle(&mut self) {
        self.active = None;
    }
    fn numbers(&self) -> Option<[f64; 9]> {
        let mut result = [0.; 9];
        for (out, value) in result.iter_mut().zip(&self.values) {
            *out = value.trim().parse::<f64>().ok()?;
            if !out.is_finite() {
                return None;
            }
        }
        Some(result)
    }
}

// World-axis rotations are applied X, then Y, then Z, about the chosen world pivot.
fn move_rotation(v: [f64; 9]) -> [[f64; 3]; 3] {
    let (sx, cx) = v[3].to_radians().sin_cos();
    let (sy, cy) = v[4].to_radians().sin_cos();
    let (sz, cz) = v[5].to_radians().sin_cos();
    [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ]
}

fn moved(old: cam_model::RigidTransform, v: [f64; 9]) -> cam_model::RigidTransform {
    let rotation = move_rotation(v);
    let pivot = [v[6], v[7], v[8]];
    let delta = cam_model::RigidTransform {
        rotation,
        translation: [0.; 3],
    };
    let offset = delta.transform_vector(std::array::from_fn(|i| old.translation[i] - pivot[i]));
    cam_model::RigidTransform {
        rotation: std::array::from_fn(|i| {
            std::array::from_fn(|j| (0..3).map(|k| rotation[i][k] * old.rotation[k][j]).sum())
        }),
        translation: std::array::from_fn(|i| pivot[i] + offset[i] + v[i]),
    }
}

impl App {
    pub(super) fn open_move(&mut self, node: TreeNode) {
        let geometry = self.controller.geometry_state();
        let (ids, label) = match node {
            TreeNode::Asset(id) => (
                geometry
                    .instances
                    .iter()
                    .filter(|i| i.asset_id == id)
                    .map(|i| i.id)
                    .collect::<Vec<_>>(),
                geometry.asset(id).map(|a| format!("Model: {}", a.name)),
            ),
            TreeNode::Model(id) => (
                vec![id],
                geometry.instance(id).map(|i| format!("Body: {}", i.name)),
            ),
            _ => return,
        };
        if let Some(label) = label.filter(|_| !ids.is_empty()) {
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for instance in geometry.instances.iter().filter(|i| ids.contains(&i.id)) {
                if let Some(body) = geometry
                    .asset(instance.asset_id)
                    .and_then(|a| a.topology.bodies.iter().find(|b| b.id == instance.body_id))
                {
                    for mask in 0..8 {
                        let p = instance.placement.transform_point(std::array::from_fn(|i| {
                            body.bounds[(mask >> i) & 1][i]
                        }));
                        for i in 0..3 {
                            lo[i] = lo[i].min(p[i]);
                            hi[i] = hi[i].max(p[i]);
                        }
                    }
                }
            }
            let mut values: [String; 9] = std::array::from_fn(|_| "0".into());
            for i in 0..3 {
                if lo[i].is_finite() && hi[i].is_finite() {
                    values[i + 6] = format!("{:.4}", (lo[i] + hi[i]) / 2.);
                }
            }
            let previous_model_view = self.model_view;
            self.model_view = Some(TreeNode::Models);
            self.inspector_enabled = true;
            self.move_draft = Some(MoveDraft {
                ids,
                label,
                previous_model_view,
                values,
                active: None,
            });
        }
    }
    pub(super) fn cancel_move(&mut self) {
        if let Some(draft) = self.move_draft.take() {
            self.model_view = draft.previous_model_view;
        }
    }
    pub(super) fn apply_move(&mut self) {
        let Some(draft) = &self.move_draft else {
            return;
        };
        let Some(values) = draft.numbers().filter(|_| draft.handle_is_valid()) else {
            self.status = "Enter finite numbers for the move.".into();
            return;
        };
        let placements: Option<Vec<_>> = draft
            .ids
            .iter()
            .map(|id| {
                self.controller
                    .geometry_state()
                    .instance(*id)
                    .map(|i| (*id, moved(i.placement, values)))
            })
            .collect();
        let result = placements
            .ok_or_else(|| "A selected body no longer exists".to_string())
            .and_then(|p| self.controller.place_cad_instances(p));
        match result {
            Ok(()) => {
                self.cancel_move();
                self.refresh_fields();
                self.status = "Move applied. Undo restores the previous placement.".into();
            }
            Err(error) => self.status = error,
        }
    }
    pub(super) fn start_move_handle(&mut self, index: usize, base: [f64; 9], direction: [f64; 3]) {
        if let Some(draft) = &mut self.move_draft {
            draft.active = Some(HandleEdit {
                index,
                base,
                direction,
                text: "0.00".into(),
            });
        }
    }
    pub(super) fn set_move_handle_text(&mut self, text: String) {
        if let Some(edit) = self
            .move_draft
            .as_mut()
            .and_then(|draft| draft.active.as_mut())
        {
            edit.text = text;
        }
    }
    pub(super) fn set_move_handle(&mut self, delta: f64, text: String) {
        let Some(draft) = &mut self.move_draft else {
            return;
        };
        let Some(edit) = &mut draft.active else {
            return;
        };
        let mut values = edit.base;
        if edit.index < 3 {
            for i in 0..3 {
                values[i] += edit.direction[i] * delta;
            }
        } else {
            let [x, y, z] = edit.direction;
            let (sin, cos) = delta.to_radians().sin_cos();
            let t = 1. - cos;
            let increment = [
                [t * x * x + cos, t * x * y - sin * z, t * x * z + sin * y],
                [t * x * y + sin * z, t * y * y + cos, t * y * z - sin * x],
                [t * x * z - sin * y, t * y * z + sin * x, t * z * z + cos],
            ];
            let base_rotation = move_rotation(edit.base);
            let rotation: [[f64; 3]; 3] = std::array::from_fn(|i| {
                std::array::from_fn(|j| (0..3).map(|k| increment[i][k] * base_rotation[k][j]).sum())
            });
            let y_angle = (-rotation[2][0]).clamp(-1., 1.).asin();
            let (x_angle, z_angle) = if y_angle.cos().abs() > 1e-9 {
                (
                    rotation[2][1].atan2(rotation[2][2]),
                    rotation[1][0].atan2(rotation[0][0]),
                )
            } else {
                (0., (-rotation[0][1]).atan2(rotation[1][1]))
            };
            values[3..6].copy_from_slice(&[
                x_angle.to_degrees(),
                y_angle.to_degrees(),
                z_angle.to_degrees(),
            ]);
        }
        for i in 0..6 {
            draft.values[i] = format!("{:.10}", values[i]);
        }
        edit.text = text;
    }
    pub(super) fn move_inspector(&self) -> Element<'_, Message> {
        let draft = self.move_draft.as_ref().expect("move draft");
        let mut fields = column![
            inspector_heading("Move"),
            text(&draft.label).size(13),
            text("Relative to the current placement · world axes")
                .size(12)
                .color(INSPECTOR_MUTED)
        ]
        .spacing(8)
        .padding(14);
        for (index, label) in [
            "X distance (mm)",
            "Y distance (mm)",
            "Z distance (mm)",
            "X angle (deg)",
            "Y angle (deg)",
            "Z angle (deg)",
            "Pivot X (mm)",
            "Pivot Y (mm)",
            "Pivot Z (mm)",
        ]
        .iter()
        .enumerate()
        {
            fields = fields.push(
                row![
                    text(*label).size(12).width(130),
                    text_input("0", &draft.values[index])
                        .on_input(move |v| Message::MoveValue(index, v))
                        .padding(5)
                        .size(13)
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
        }
        fields = fields.push(text("Drag an axis arrow or a rotation dot in the viewport. Rotations apply X, then Y, then Z about the pivot; translation follows.").size(11).color(INSPECTOR_MUTED));
        fields = fields.push(
            row![
                inspector_button("Cancel").on_press(Message::MoveCancel),
                inspector_button("Move").on_press_maybe(
                    draft
                        .numbers()
                        .filter(|_| draft.handle_is_valid())
                        .map(|_| Message::MoveApply)
                )
            ]
            .spacing(8),
        );
        scrollable(fields).height(Length::Fill).into()
    }
    pub(super) fn move_preview_scene(
        &self,
        hidden: &BTreeSet<InstanceId>,
    ) -> Option<Arc<CadScene>> {
        let draft = self.move_draft.as_ref()?;
        let values = draft.numbers()?;
        let mut geometry = self.controller.geometry_state().clone();
        for instance in &mut geometry.instances {
            if draft.ids.contains(&instance.id) {
                instance.placement = moved(instance.placement, values);
            }
        }
        Some(Arc::new(CadScene::from_geometry_in_setup(
            &geometry,
            hidden,
            &self.controller.inspection_setup().frame,
        )))
    }
    pub(super) fn move_handles(&self, camera: OrbitCamera) -> Option<MoveHandles> {
        let draft = self.move_draft.as_ref()?;
        let values = draft.numbers()?;
        let instance = self
            .controller
            .geometry_state()
            .instance(*draft.ids.first()?)?;
        let rotation = moved(instance.placement, values).rotation;
        let axes = std::array::from_fn(|axis| std::array::from_fn(|i| rotation[i][axis]));
        Some(MoveHandles {
            camera,
            axes,
            origin: std::array::from_fn(|i| values[6 + i] + values[i]),
            values,
        })
    }
    pub(super) fn move_handle_input(&self) -> Option<Element<'_, Message>> {
        let draft = self.move_draft.as_ref()?;
        let edit = draft.active.as_ref()?;
        let index = edit.index;
        Some(
            container(
                row![
                    text(format!(
                        "{} {}",
                        ["X", "Y", "Z"][index % 3],
                        if index < 3 { "distance" } else { "angle" }
                    ))
                    .size(12),
                    text_input("0", &edit.text)
                        .on_input(Message::MoveHandleInput)
                        .width(90)
                        .padding(5)
                        .size(14),
                    text(if index < 3 { "mm" } else { "deg" }).size(12)
                ]
                .spacing(6)
                .align_y(Alignment::Center),
            )
            .padding(8)
            .style(popup_surface)
            .into(),
        )
    }
}

#[derive(Clone)]
pub(super) struct MoveHandles {
    camera: OrbitCamera,
    pub(super) axes: [[f64; 3]; 3],
    origin: [f64; 3],
    values: [f64; 9],
}

#[derive(Default)]
pub(super) struct HandleState {
    drag: Option<HandleDrag>,
    locked_radii: Option<[f64; 3]>,
}
enum HandleDrag {
    Translation {
        start: iced::Point,
        direction: iced::Vector,
        scale: f64,
        step: f64,
    },
    Rotation {
        center: iced::Point,
        u: iced::Vector,
        v: iced::Vector,
        last_angle: f64,
        accumulated: f64,
    },
    // An edge-on ring has no unique projected angle; retain linear dragging there.
    EdgeOnRotation {
        start: iced::Point,
        direction: iced::Vector,
    },
}

fn rotation_cursor_angle(
    point: iced::Point,
    center: iced::Point,
    u: iced::Vector,
    v: iced::Vector,
) -> Option<f64> {
    let determinant = u.x * v.y - u.y * v.x;
    let area = u.x.hypot(u.y) * v.x.hypot(v.y);
    if area < 1. || determinant.abs() < area * 0.01 {
        return None;
    }
    let p = point - center;
    let x = (p.x * v.y - p.y * v.x) / determinant;
    let y = (u.x * p.y - u.y * p.x) / determinant;
    if x.hypot(y) < 0.05 {
        return None;
    }
    Some((y as f64).atan2(x as f64))
}

fn drag_step(world_per_pixel: f32) -> f64 {
    let target = world_per_pixel as f64 * 12.;
    if target <= 0.5 {
        0.5
    } else if target <= 1. {
        1.
    } else if target <= 5. {
        5.
    } else {
        10.
    }
}
impl MoveHandles {
    /// Keep rotation dots outside the axis cross, including in foreshortened views.
    /// Choose a larger radius when projection would otherwise overlap another handle.
    fn rotation_radii(&self, size: iced::Size, length: f64) -> Option<[f64; 3]> {
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, size);
        let project = |point| picking_view::project(self.camera, bounds, point).map(|v| v.0);
        let center = project(self.origin)?;
        let mut tips = [center; 3];
        for axis in 0..3 {
            tips[axis] = project(std::array::from_fn(|j| {
                self.origin[j] + length * self.axes[axis][j]
            }))?;
        }
        let mut dots = Vec::new();
        let mut radii = [length * 1.8; 3];
        for axis in 0..3 {
            let a = (axis + 1) % 3;
            let b = (axis + 2) % 3;
            let mut best = (0., radii[axis], center);
            for step in 0..=16 {
                let radius = length * (1.8 + 0.2 * step as f64);
                let point = project(std::array::from_fn(|j| {
                    self.origin[j]
                        + radius
                            * std::f64::consts::FRAC_1_SQRT_2
                            * (self.axes[a][j] + self.axes[b][j])
                }))?;
                let clearance = tips
                    .iter()
                    .chain(dots.iter())
                    .map(|other| point.distance(*other))
                    .fold(point.distance(center), f32::min);
                if clearance > best.0 {
                    best = (clearance, radius, point);
                }
                if clearance >= 28. {
                    break;
                }
            }
            radii[axis] = best.1;
            dots.push(best.2);
        }
        Some(radii)
    }
    fn rotation_basis(
        &self,
        axis: usize,
        size: iced::Size,
        length: f64,
    ) -> Option<(iced::Point, iced::Vector, iced::Vector)> {
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, size);
        let center = picking_view::project(self.camera, bounds, self.origin)?.0;
        let radius = self.rotation_radii(size, length)?[axis];
        let point = |i: usize| std::array::from_fn(|j| self.origin[j] + radius * self.axes[i][j]);
        let u = picking_view::project(self.camera, bounds, point((axis + 1) % 3))?.0 - center;
        let v = picking_view::project(self.camera, bounds, point((axis + 2) % 3))?.0 - center;
        Some((center, u, v))
    }
    fn projected(
        &self,
        size: iced::Size,
    ) -> Option<(
        iced::Point,
        [iced::Point; 3],
        [iced::Point; 3],
        [iced::Vector; 3],
        f64,
    )> {
        self.projected_with_radii(size, None)
    }
    fn projected_with_radii(
        &self,
        size: iced::Size,
        locked_radii: Option<[f64; 3]>,
    ) -> Option<(
        iced::Point,
        [iced::Point; 3],
        [iced::Point; 3],
        [iced::Vector; 3],
        f64,
    )> {
        if size.width <= 0. || size.height <= 0. {
            return None;
        }
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, size);
        let project = |p| picking_view::project(self.camera, bounds, p).map(|v| v.0);
        let origin = project(self.origin)?;
        let length = 75. * self.camera.world_per_pixel(size.height) as f64;
        let mut tips = [origin; 3];
        let mut dots = [origin; 3];
        let mut tangents = [iced::Vector::ZERO; 3];
        let radii = match locked_radii {
            Some(radii) => radii,
            None => self.rotation_radii(size, length)?,
        };
        for axis in 0..3 {
            let p = std::array::from_fn(|i| self.origin[i] + self.axes[axis][i] * length);
            tips[axis] = project(p)?;
            let a = (axis + 1) % 3;
            let b = (axis + 2) % 3;
            let radius = radii[axis];
            let angle = std::f64::consts::FRAC_PI_4;
            let mut p = self.origin;
            for i in 0..3 {
                p[i] += radius * (self.axes[a][i] * angle.cos() + self.axes[b][i] * angle.sin());
            }
            dots[axis] = project(p)?;
            let mut q = p;
            for i in 0..3 {
                q[i] += radius
                    * (-self.axes[a][i] * angle.sin() + self.axes[b][i] * angle.cos())
                    * 0.01;
            }
            tangents[axis] = (project(q)? - dots[axis]) / 0.01;
        }
        Some((origin, tips, dots, tangents, length))
    }
}
impl canvas::Program<Message> for MoveHandles {
    type State = HandleState;
    fn update(
        &self,
        state: &mut Self::State,
        event: &canvas::Event,
        bounds: iced::Rectangle,
        cursor: iced::mouse::Cursor,
    ) -> Option<canvas::Action<Message>> {
        use iced::mouse::{Button, Event};
        match event {
            canvas::Event::Mouse(Event::ButtonPressed(Button::Left)) => {
                let p = cursor.position_in(bounds)?;
                let (origin, tips, dots, tangents, length) = self.projected(bounds.size())?;
                let mut candidates = Vec::new();
                for axis in 0..3 {
                    let axis_vector = tips[axis] - origin;
                    if axis_vector.x.hypot(axis_vector.y) > 8. {
                        candidates.push((
                            axis,
                            p.distance(tips[axis]),
                            axis_vector,
                            length,
                            drag_step(self.camera.world_per_pixel(bounds.height)),
                        ));
                    }
                    if tangents[axis].x.hypot(tangents[axis].y) > 8. {
                        candidates.push((
                            axis + 3,
                            p.distance(dots[axis]),
                            tangents[axis],
                            180. / std::f64::consts::PI,
                            5.,
                        ));
                    }
                }
                let (index, _, direction, scale, step) = candidates
                    .into_iter()
                    .filter(|c| c.1 <= 12.)
                    .min_by(|a, b| a.1.total_cmp(&b.1))?;
                state.locked_radii = self.rotation_radii(bounds.size(), length);
                state.drag = Some(if index < 3 {
                    HandleDrag::Translation {
                        start: p,
                        direction,
                        scale,
                        step,
                    }
                } else if let Some((center, u, v)) =
                    self.rotation_basis(index - 3, bounds.size(), length)
                {
                    if let Some(last_angle) = rotation_cursor_angle(p, center, u, v) {
                        HandleDrag::Rotation {
                            center,
                            u,
                            v,
                            last_angle,
                            accumulated: 0.,
                        }
                    } else {
                        HandleDrag::EdgeOnRotation {
                            start: p,
                            direction,
                        }
                    }
                } else {
                    return None;
                });
                Some(
                    canvas::Action::publish(Message::MoveHandleStart(
                        index,
                        self.values,
                        self.axes[index % 3],
                    ))
                    .and_capture(),
                )
            }
            canvas::Event::Mouse(Event::CursorMoved { position }) => {
                let drag = state.drag.as_mut()?;
                let p = iced::Point::new(position.x - bounds.x, position.y - bounds.y);
                let value = match drag {
                    HandleDrag::Translation {
                        start,
                        direction,
                        scale,
                        step,
                    } => {
                        let d = p - *start;
                        let denom = direction.x * direction.x + direction.y * direction.y;
                        let raw = ((d.x * direction.x + d.y * direction.y) / denom) as f64 * *scale;
                        (raw / *step).round() * *step
                    }
                    HandleDrag::Rotation {
                        center,
                        u,
                        v,
                        last_angle,
                        accumulated,
                    } => {
                        if let Some(angle) = rotation_cursor_angle(p, *center, *u, *v) {
                            let change = (angle - *last_angle + std::f64::consts::PI)
                                .rem_euclid(std::f64::consts::TAU)
                                - std::f64::consts::PI;
                            *accumulated += change;
                            *last_angle = angle;
                        }
                        (accumulated.to_degrees() / 5.).round() * 5.
                    }
                    HandleDrag::EdgeOnRotation { start, direction } => {
                        let d = p - *start;
                        let denom = direction.x * direction.x + direction.y * direction.y;
                        let degrees = ((d.x * direction.x + d.y * direction.y) / denom) as f64
                            * 180.
                            / std::f64::consts::PI;
                        (degrees / 5.).round() * 5.
                    }
                };
                Some(canvas::Action::publish(Message::MoveHandleDelta(value)).and_capture())
            }
            canvas::Event::Mouse(Event::ButtonReleased(Button::Left))
                if state.drag.take().is_some() =>
            {
                state.locked_radii = None;
                Some(canvas::Action::capture())
            }
            _ => None,
        }
    }
    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: iced::Rectangle,
        cursor: iced::mouse::Cursor,
    ) -> iced::mouse::Interaction {
        if state.drag.is_some() {
            return iced::mouse::Interaction::Grabbing;
        }
        if let (Some(p), Some((origin, tips, dots, tangents, _))) =
            (cursor.position_in(bounds), self.projected(bounds.size()))
        {
            if (0..3).any(|axis| {
                ((tips[axis].x - origin.x).hypot(tips[axis].y - origin.y) > 8.
                    && p.distance(tips[axis]) <= 12.)
                    || (tangents[axis].x.hypot(tangents[axis].y) > 8.
                        && p.distance(dots[axis]) <= 12.)
            }) {
                return iced::mouse::Interaction::Grab;
            }
        }
        iced::mouse::Interaction::default()
    }
    fn draw(
        &self,
        state: &Self::State,
        renderer: &iced::Renderer,
        _: &iced::Theme,
        bounds: iced::Rectangle,
        _: iced::mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let mut frame = canvas::Frame::new(renderer, bounds.size());
        let Some((origin, tips, dots, _, length)) =
            self.projected_with_radii(bounds.size(), state.locked_radii)
        else {
            return vec![];
        };
        let colors = [
            Color::from_rgb(1., 0.3, 0.25),
            Color::from_rgb(0.25, 0.9, 0.35),
            Color::from_rgb(0.3, 0.65, 1.),
        ];
        let rect = iced::Rectangle::new(iced::Point::ORIGIN, bounds.size());
        let Some(radii) = state
            .locked_radii
            .or_else(|| self.rotation_radii(bounds.size(), length))
        else {
            return vec![];
        };
        for axis in 0..3 {
            let color = colors[axis];
            let stroke = canvas::Stroke::default().with_color(color).with_width(2.);
            let a = (axis + 1) % 3;
            let b = (axis + 2) % 3;
            let radius = radii[axis];
            let points: Vec<_> = (0..=32)
                .filter_map(|i| {
                    let angle = std::f64::consts::FRAC_PI_4
                        + (i as f64 / 32. - 0.5) * std::f64::consts::FRAC_PI_4;
                    let mut p = self.origin;
                    for j in 0..3 {
                        p[j] += radius
                            * (self.axes[a][j] * angle.cos() + self.axes[b][j] * angle.sin());
                    }
                    picking_view::project(self.camera, rect, p).map(|v| v.0)
                })
                .collect();
            for (i, pair) in points.windows(2).enumerate() {
                let x = (i as f32 + 0.5) / 32.;
                let alpha = (1. - (2. * x - 1.).abs()).min(0.5) * 2.;
                frame.stroke(
                    &canvas::Path::line(pair[0], pair[1]),
                    stroke.with_color(Color { a: alpha, ..color }),
                );
            }
            let v = tips[axis] - origin;
            let n = v.x.hypot(v.y);
            if n > 8. {
                let d = v / n;
                let side = iced::Vector::new(-d.y, d.x);
                frame.stroke(
                    &canvas::Path::line(origin, tips[axis]),
                    stroke.with_width(3.),
                );
                let arrow = canvas::Path::new(|p| {
                    p.move_to(tips[axis]);
                    p.line_to(tips[axis] - d * 12. + side * 5.);
                    p.line_to(tips[axis] - d * 12. - side * 5.);
                    p.close();
                });
                frame.fill(&arrow, color);
            }
            frame.fill(&canvas::Path::circle(dots[axis], 6.), Color::WHITE);
            frame.stroke(&canvas::Path::circle(dots[axis], 6.), stroke.with_width(3.));
            frame.fill_text(canvas::Text {
                content: ["X", "Y", "Z"][axis].into(),
                position: tips[axis] + iced::Vector::new(5., -15.),
                color,
                size: 13.0.into(),
                ..Default::default()
            });
        }
        vec![frame.into_geometry()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn handle_drag_publishes_snapped_axis_values() {
        use canvas::Program;
        use iced::mouse::{Button, Cursor, Event};
        let mut camera = OrbitCamera::framed([-50.; 3], [50.; 3]);
        camera.orient = cam_render::orientation(0.6, 0.7);
        let overlay = MoveHandles {
            camera,
            axes: [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            origin: [0.; 3],
            values: [0.; 9],
        };
        let bounds = iced::Rectangle::new(iced::Point::new(40., 60.), iced::Size::new(800., 600.));
        let (origin, tips, _, _, _) = overlay.projected(bounds.size()).unwrap();
        let tip = tips[0];
        let start = tip + iced::Vector::new(bounds.x, bounds.y);
        let mut state = HandleState::default();
        assert_eq!(
            overlay.mouse_interaction(&state, bounds, Cursor::Available(start)),
            iced::mouse::Interaction::Grab
        );
        let (message, _, _) = overlay
            .update(
                &mut state,
                &canvas::Event::Mouse(Event::ButtonPressed(Button::Left)),
                bounds,
                Cursor::Available(start),
            )
            .unwrap()
            .into_inner();
        assert!(matches!(message, Some(Message::MoveHandleStart(0, _, _))));
        assert_eq!(
            overlay.mouse_interaction(&state, bounds, Cursor::Available(start)),
            iced::mouse::Interaction::Grabbing
        );
        let end = start + (tip - origin) * 0.5;
        let (message, _, _) = overlay
            .update(
                &mut state,
                &canvas::Event::Mouse(Event::CursorMoved { position: end }),
                bounds,
                Cursor::Available(end),
            )
            .unwrap()
            .into_inner();
        match message {
            Some(Message::MoveHandleDelta(value)) => {
                let step = drag_step(camera.world_per_pixel(bounds.height));
                assert!(value > 0. && (value / step - (value / step).round()).abs() < 1e-8);
            }
            _ => panic!("expected X translation"),
        }
        assert_eq!(
            [
                drag_step(0.01),
                drag_step(0.06),
                drag_step(0.2),
                drag_step(0.7)
            ],
            [0.5, 1., 5., 10.]
        );
    }
    #[test]
    fn rotation_dot_starts_axis_rotation() {
        use canvas::Program;
        use iced::mouse::{Button, Cursor, Event};
        let mut camera = OrbitCamera::framed([-50.; 3], [50.; 3]);
        camera.orient = cam_render::orientation(0.6, 0.7);
        let overlay = MoveHandles {
            camera,
            axes: [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            origin: [0.; 3],
            values: [0.; 9],
        };
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(800., 600.));
        let (_, _, dots, tangents, _) = overlay.projected(bounds.size()).unwrap();
        let mut state = HandleState::default();
        let start = dots[2];
        let (message, _, _) = overlay
            .update(
                &mut state,
                &canvas::Event::Mouse(Event::ButtonPressed(Button::Left)),
                bounds,
                Cursor::Available(start),
            )
            .unwrap()
            .into_inner();
        assert!(matches!(message, Some(Message::MoveHandleStart(5, _, _))));
        let end = start + tangents[2] * 0.2;
        let (message, _, _) = overlay
            .update(
                &mut state,
                &canvas::Event::Mouse(Event::CursorMoved { position: end }),
                bounds,
                Cursor::Available(end),
            )
            .unwrap()
            .into_inner();
        assert!(matches!(message, Some(Message::MoveHandleDelta(value)) if value.abs() >= 5.));
    }
    #[test]
    fn rotation_handles_stay_clear_of_cross_and_each_other() {
        use canvas::Program;
        use iced::mouse::{Button, Cursor, Event};
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(1000., 800.));
        for (yaw, pitch) in [(0., 0.), (0.6, 0.7), (1.2, 0.35), (2.1, 1.1)] {
            let mut camera = OrbitCamera::framed([-50.; 3], [50.; 3]);
            camera.orient = cam_render::orientation(yaw, pitch);
            let overlay = MoveHandles {
                camera,
                axes: [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
                origin: [0.; 3],
                values: [0.; 9],
            };
            let (_, tips, dots, tangents, length) = overlay.projected(bounds.size()).unwrap();
            let radii = overlay.rotation_radii(bounds.size(), length).unwrap();
            assert!(radii.iter().all(|radius| *radius >= 1.8 * length));
            for axis in 0..3 {
                for tip in tips {
                    assert!(
                        dots[axis].distance(tip) >= 27.9,
                        "overlap at {yaw}, {pitch}"
                    );
                }
                for other in 0..axis {
                    assert!(dots[axis].distance(dots[other]) >= 27.9);
                }
                if tangents[axis].x.hypot(tangents[axis].y) > 8. {
                    let mut state = HandleState::default();
                    let (message, _, _) = overlay
                        .update(
                            &mut state,
                            &canvas::Event::Mouse(Event::ButtonPressed(Button::Left)),
                            bounds,
                            Cursor::Available(dots[axis]),
                        )
                        .unwrap()
                        .into_inner();
                    assert!(
                        matches!(message, Some(Message::MoveHandleStart(index, _, _)) if index == axis+3)
                    );
                }
            }
        }
    }
    #[test]
    fn rotation_drag_tracks_quadrants_and_wraps_in_both_directions() {
        use canvas::Program;
        use iced::mouse::{Button, Cursor, Event};
        let mut camera = OrbitCamera::framed([-50.; 3], [50.; 3]);
        camera.orient = cam_render::orientation(0.6, 0.7);
        let overlay = MoveHandles {
            camera,
            axes: [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            origin: [0.; 3],
            values: [0.; 9],
        };
        let bounds = iced::Rectangle::new(iced::Point::new(40., 60.), iced::Size::new(800., 600.));
        let (_, _, dots, _, length) = overlay.projected(bounds.size()).unwrap();
        let (center, u, v) = overlay.rotation_basis(2, bounds.size(), length).unwrap();
        let screen_offset = iced::Vector::new(bounds.x, bounds.y);
        for direction in [1., -1.] {
            let mut state = HandleState::default();
            let start = dots[2] + screen_offset;
            let (message, _, _) = overlay
                .update(
                    &mut state,
                    &canvas::Event::Mouse(Event::ButtonPressed(Button::Left)),
                    bounds,
                    Cursor::Available(start),
                )
                .unwrap()
                .into_inner();
            assert!(matches!(message, Some(Message::MoveHandleStart(5, _, _))));
            for step in 1..=16 {
                let degrees: f64 = direction * step as f64 * 30.;
                let angle = (45. + degrees).to_radians();
                let point =
                    center + u * angle.cos() as f32 + v * angle.sin() as f32 + screen_offset;
                let (message, _, _) = overlay
                    .update(
                        &mut state,
                        &canvas::Event::Mouse(Event::CursorMoved { position: point }),
                        bounds,
                        Cursor::Available(point),
                    )
                    .unwrap()
                    .into_inner();
                assert!(
                    matches!(message, Some(Message::MoveHandleDelta(value)) if (value-degrees).abs() < 0.01),
                    "step {step}, direction {direction}: {message:?}"
                );
            }
        }
    }
    #[test]
    fn move_rotates_about_pivot_then_translates() {
        let old = cam_model::RigidTransform {
            translation: [2., 0., 0.],
            ..Default::default()
        };
        let result = moved(old, [3., 0., 0., 0., 0., 90., 1., 0., 0.]);
        assert!((result.translation[0] - 4.).abs() < 1e-10);
        assert!((result.translation[1] - 1.).abs() < 1e-10);
        assert!(result.is_rigid());
    }
}
