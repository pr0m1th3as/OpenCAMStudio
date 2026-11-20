//! Dimensioned work-coordinate grid shared by the GPU line pass and canvas labels.
use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GridPlane {
    Xy,
    Xz,
    Yz,
}
impl GridPlane {
    fn for_camera(camera: OrbitCamera, frame: cam_model::RigidTransform) -> Self {
        // Only cardinal side views switch planes; even a small orbit restores XY.
        let world_normal = [
            camera.orient[0][2] as f64,
            camera.orient[1][2] as f64,
            camera.orient[2][2] as f64,
        ];
        let normal: [f32; 3] = std::array::from_fn(|i| {
            (0..3)
                .map(|j| frame.rotation[j][i] * world_normal[j])
                .sum::<f64>() as f32
        });
        const EPSILON: f32 = 1e-5;
        if normal[2].abs() < EPSILON && normal[0].abs() < EPSILON {
            Self::Xz
        } else if normal[2].abs() < EPSILON && normal[1].abs() < EPSILON {
            Self::Yz
        } else {
            Self::Xy
        }
    }
    fn axes(self) -> [usize; 2] {
        match self {
            Self::Xy => [0, 1],
            Self::Xz => [0, 2],
            Self::Yz => [1, 2],
        }
    }
    fn normal_axis(self) -> usize {
        match self {
            Self::Xy => 2,
            Self::Xz => 1,
            Self::Yz => 0,
        }
    }
    fn name(self) -> &'static str {
        match self {
            Self::Xy => "XY",
            Self::Xz => "XZ",
            Self::Yz => "YZ",
        }
    }
    fn point(self, coordinates: [f32; 2]) -> [f32; 3] {
        let mut point = [0.; 3];
        let [a, b] = self.axes();
        point[a] = coordinates[0];
        point[b] = coordinates[1];
        point
    }
    fn pick(
        self,
        camera: OrbitCamera,
        frame: cam_model::RigidTransform,
        u: f32,
        v: f32,
        aspect: f32,
    ) -> Option<[f32; 2]> {
        let ray = camera.pick_ray(u, v, aspect);
        let origin = frame.inverse_point(ray.origin);
        let ray_direction: [f64; 3] = std::array::from_fn(|i| {
            (0..3)
                .map(|j| frame.rotation[j][i] * ray.direction[j])
                .sum()
        });
        let normal = self.normal_axis();
        let direction = ray_direction[normal];
        if direction.abs() < 1e-8 {
            return None;
        }
        let t = -origin[normal] / direction;
        let [a, b] = self.axes();
        Some([
            (origin[a] + t * ray_direction[a]) as f32,
            (origin[b] + t * ray_direction[b]) as f32,
        ])
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct GridLayout {
    plane: GridPlane,
    frame: cam_model::RigidTransform,
    min: [f32; 2],
    max: [f32; 2],
    visible_min: [f32; 2],
    visible_max: [f32; 2],
    pub step: f32,
}
impl GridLayout {
    pub(super) fn new(
        camera: OrbitCamera,
        size: iced::Size,
        machine_size: [f32; 3],
        frame: cam_model::RigidTransform,
    ) -> Self {
        // The machine gives a travel, not an absolute work origin. Center its
        // footprint on the active work origin.
        let plane = GridPlane::for_camera(camera, frame);
        let axes = plane.axes();
        let half = axes.map(|axis| {
            let v = machine_size[axis];
            if v.is_finite() && v > 0. {
                v / 2.
            } else {
                50.
            }
        });
        let min = half.map(|v| -v);
        let max = half;
        let aspect = size.width / size.height.max(1.);
        let points: Vec<_> = [[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]]
            .into_iter()
            .filter_map(|[x, y]| plane.pick(camera, frame, x, y, aspect))
            .filter(|p| p[0].is_finite() && p[1].is_finite())
            .collect();
        let visible_min = std::array::from_fn(|axis| {
            points
                .iter()
                .map(|p| p[axis])
                .reduce(f32::min)
                .unwrap_or(min[axis])
                .max(min[axis])
        });
        let visible_max = std::array::from_fn(|axis| {
            points
                .iter()
                .map(|p| p[axis])
                .reduce(f32::max)
                .unwrap_or(max[axis])
                .min(max[axis])
        });
        let raw = camera.world_per_pixel(size.height.max(1.)) * 80.;
        let visible_span = (visible_max[0] - visible_min[0]).max(visible_max[1] - visible_min[1]);
        let step = nice_step(raw.max(visible_span / 100.));
        Self {
            plane,
            frame,
            min,
            max,
            visible_min,
            visible_max,
            step,
        }
    }
    fn point(&self, coordinates: [f32; 2]) -> [f32; 3] {
        self.frame
            .transform_point(self.plane.point(coordinates).map(f64::from))
            .map(|v| v as f32)
    }
    fn ticks(&self, axis: usize) -> Vec<f32> {
        let start = (self.visible_min[axis] / self.step).ceil();
        let count =
            ((self.visible_max[axis] / self.step).floor() - start + 1.).clamp(0., 101.) as usize;
        (0..count).map(|i| (start + i as f32) * self.step).collect()
    }
    pub(super) fn vertices(&self) -> Vec<Vertex> {
        let mut scene = Scene::new();
        for axis in 0..2 {
            for tick in self.ticks(axis) {
                let color = if tick.abs() < self.step * 0.01 {
                    [0.36, 0.43, 0.49, 1.]
                } else {
                    [0.20, 0.24, 0.28, 1.]
                };
                let mut a = self.min;
                let mut b = self.max;
                a[axis] = tick;
                b[axis] = tick;
                scene.add_strip(vec![self.point(a), self.point(b)], color);
            }
        }
        let [x, y] = self.max;
        scene.add_strip(
            [[-x, -y], [x, -y], [x, y], [-x, y], [-x, -y]]
                .into_iter()
                .map(|point| self.point(point))
                .collect(),
            [0.30, 0.34, 0.38, 1.],
        );
        scene.line_vertices()
    }
}
fn nice_step(raw: f32) -> f32 {
    if !raw.is_finite() || raw <= 0. {
        return 10.;
    }
    let power = 10_f32.powf(raw.log10().floor());
    let scaled = raw / power;
    let nice = if scaled <= 1. {
        1.
    } else if scaled <= 2. {
        2.
    } else if scaled <= 5. {
        5.
    } else {
        10.
    };
    (nice * power).max(0.000001)
}
fn tick_text(value: f32, step: f32) -> String {
    let decimals = (-step.log10().floor()).clamp(0., 6.) as usize;
    let value = if value.abs() < step * 0.001 {
        0.
    } else {
        value
    };
    format!("{value:.decimals$}")
}
fn project(camera: OrbitCamera, point: [f32; 3], size: iced::Size) -> iced::Point {
    let matrix = camera.view_proj(size.width / size.height.max(1.));
    let clip: [f32; 4] = std::array::from_fn(|row| {
        matrix[0][row] * point[0]
            + matrix[1][row] * point[1]
            + matrix[2][row] * point[2]
            + matrix[3][row]
    });
    iced::Point::new(
        (clip[0] / clip[3] + 1.) * size.width / 2.,
        (1. - clip[1] / clip[3]) * size.height / 2.,
    )
}
// Clip a projected grid line to the viewport so labels remain visible when
// zooming into the plane, even when its physical boundary is off screen.
fn clipped_line(
    a: iced::Point,
    b: iced::Point,
    size: iced::Size,
) -> Option<(iced::Point, iced::Point)> {
    if ![a.x, a.y, b.x, b.y].iter().all(|v| v.is_finite()) || size.width < 50. || size.height < 50.
    {
        return None;
    }
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let (mut lo, mut hi) = (0_f32, 1_f32);
    for (p, q) in [
        (-dx, a.x - 18.),
        (dx, size.width - 18. - a.x),
        (-dy, a.y - 18.),
        (dy, size.height - 30. - a.y),
    ] {
        if p.abs() < 1e-6 {
            if q < 0. {
                return None;
            }
        } else if p < 0. {
            lo = lo.max(q / p);
        } else {
            hi = hi.min(q / p);
        }
    }
    if lo > hi {
        return None;
    }
    Some((
        iced::Point::new(a.x + lo * dx, a.y + lo * dy),
        iced::Point::new(a.x + hi * dx, a.y + hi * dy),
    ))
}
#[derive(Clone)]
pub(super) struct GridLabels {
    pub camera: OrbitCamera,
    pub grid_size: [f32; 3],
    pub frame: cam_model::RigidTransform,
    pub measurement: Option<String>,
}
impl canvas::Program<Message> for GridLabels {
    type State = ();
    fn draw(
        &self,
        _: &(),
        renderer: &iced::Renderer,
        _: &iced::Theme,
        bounds: iced::Rectangle,
        _: iced::mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let size = bounds.size();
        let layout = GridLayout::new(self.camera, size, self.grid_size, self.frame);
        let mut frame = canvas::Frame::new(renderer, size);
        for axis in 0..2 {
            let mut previous: Option<iced::Point> = None;
            for tick in layout.ticks(axis) {
                let mut a = layout.min;
                let mut b = layout.max;
                a[axis] = tick;
                b[axis] = tick;
                let Some((a, b)) = clipped_line(
                    project(self.camera, layout.point(a), size),
                    project(self.camera, layout.point(b), size),
                    size,
                ) else {
                    continue;
                };
                let point = if a.y > b.y { a } else { b };
                // The text baseline follows the projected grid axis.
                let mut along = [0., 0., 0.];
                along[layout.plane.axes()[axis]] = layout.step;
                let origin = project(
                    self.camera,
                    layout.frame.translation.map(|v| v as f32),
                    size,
                );
                let direction = project(
                    self.camera,
                    layout
                        .frame
                        .transform_point(along.map(f64::from))
                        .map(|v| v as f32),
                    size,
                );
                let mut angle = (direction.y - origin.y).atan2(direction.x - origin.x);
                if angle > std::f32::consts::FRAC_PI_2 {
                    angle -= std::f32::consts::PI;
                }
                if angle < -std::f32::consts::FRAC_PI_2 {
                    angle += std::f32::consts::PI;
                }
                if previous.is_some_and(|p| (p.x - point.x).hypot(p.y - point.y) < 32.) {
                    continue;
                }
                previous = Some(point);
                frame.with_save(|frame| {
                    frame.translate(iced::Vector::new(point.x, point.y));
                    frame.rotate(angle);
                    frame.scale_nonuniform(iced::Vector::new(1., 0.8));
                    frame.fill_text(canvas::Text {
                        content: tick_text(tick, layout.step),
                        position: iced::Point::new(2., -12.),
                        color: Color::from_rgb(0.63, 0.68, 0.73),
                        size: 11.0.into(),
                        ..Default::default()
                    });
                });
            }
        }
        if let Some(measurement) = &self.measurement {
            frame.fill_text(canvas::Text {
                content: measurement.clone(),
                position: iced::Point::new(
                    (size.width - 165.).max(8.),
                    (size.height - 17.).max(8.),
                ),
                align_x: iced::alignment::Horizontal::Right.into(),
                color: Color::from_rgb(0.85, 0.88, 0.91),
                size: 11.0.into(),
                ..Default::default()
            });
        }
        frame.fill_text(canvas::Text {
            content: format!(
                "{} grid · {} mm",
                layout.plane.name(),
                tick_text(layout.step, layout.step)
            ),
            position: iced::Point::new((size.width - 155.).max(8.), (size.height - 17.).max(8.)),
            color: Color::from_rgb(0.63, 0.68, 0.73),
            size: 11.0.into(),
            ..Default::default()
        });
        vec![frame.into_geometry()]
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

// Closest distance between finite straight edges, including disjoint segments.
fn segment_distance(p: [f64; 3], q: [f64; 3], r: [f64; 3], s: [f64; 3]) -> f64 {
    let u = sub(q, p);
    let v = sub(s, r);
    let w = sub(p, r);
    let (a, b, c, d, e) = (dot(u, u), dot(u, v), dot(v, v), dot(u, w), dot(v, w));
    if a < 1e-20 && c < 1e-20 {
        return norm(w);
    }
    if a < 1e-20 {
        return norm(sub(w, v.map(|x| x * (e / c).clamp(0., 1.))));
    }
    if c < 1e-20 {
        return norm(std::array::from_fn(|i| {
            w[i] + u[i] * (-d / a).clamp(0., 1.)
        }));
    }
    let denom = a * c - b * b;
    let mut t = if denom > 1e-20 {
        ((b * e - c * d) / denom).clamp(0., 1.)
    } else {
        0.
    };
    let mut k = (b * t + e) / c;
    if k < 0. {
        k = 0.;
        t = (-d / a).clamp(0., 1.);
    } else if k > 1. {
        k = 1.;
        t = ((b - d) / a).clamp(0., 1.);
    }
    norm(std::array::from_fn(|i| w[i] + t * u[i] - k * v[i]))
}

pub(super) fn selection_measurement(
    state: &cam_model::GeometryState,
    selected: &[EntityRef],
) -> Option<String> {
    use cam_model::{CadCurve, CadSurface};
    if selected.is_empty() {
        return None;
    }
    if selected.len() > 2 {
        return Some(format!("{} features selected", selected.len()));
    }
    let resolve = |r: &EntityRef| {
        if !state.entity_exists(r) {
            return None;
        }
        Some((state.asset(r.asset_id)?, state.instance(r.instance_id)?))
    };
    if selected.len() == 1 {
        let r = &selected[0];
        let (asset, _) = resolve(r)?;
        let topology = &asset.topology;
        return Some(match r.kind {
            EntityKind::Edge => match &topology.edges.iter().find(|e| e.id == r.id)?.curve {
                CadCurve::Line { start, end } => {
                    format!("Line edge | Length: {:.2} mm", norm(sub(*end, *start)))
                }
                CadCurve::CircleArc { radius, .. } => {
                    format!("Circular edge | Radius: {:.2} mm", radius)
                }
                CadCurve::BezierSpans { .. } => "Spline edge".into(),
                CadCurve::Unsupported { .. } => "Edge".into(),
            },
            EntityKind::Face => {
                let face = topology.faces.iter().find(|f| f.id == r.id)?;
                let mesh = &topology.mesh;
                let area: f64 = mesh
                    .triangles
                    .iter()
                    .zip(&mesh.triangle_faces)
                    .filter(|(_, id)| **id == r.id)
                    .filter_map(|(tri, _)| {
                        let a = *mesh.vertices.get(tri[0] as usize)?;
                        let b = *mesh.vertices.get(tri[1] as usize)?;
                        let c = *mesh.vertices.get(tri[2] as usize)?;
                        Some(norm(cross(sub(b, a), sub(c, a))) * 0.5)
                    })
                    .sum();
                let kind = match &face.surface {
                    CadSurface::Plane { .. } => "Planar face",
                    CadSurface::Cylinder { .. } => "Cylindrical face",
                    CadSurface::Cone { .. } => "Conical face",
                    CadSurface::Sphere { .. } => "Spherical face",
                    CadSurface::Torus { .. } => "Toroidal face",
                    _ => "Face",
                };
                if area > 0. {
                    format!("{kind} | Area: ≈ {:.2} mm²", area)
                } else {
                    kind.into()
                }
            }
            EntityKind::Vertex => "Vertex".into(),
            EntityKind::Body => "Body".into(),
            EntityKind::Loop => "Loop".into(),
            EntityKind::Silhouette => "Silhouette".into(),
        });
    }
    let (a, ia) = resolve(&selected[0])?;
    let (b, ib) = resolve(&selected[1])?;
    if selected.iter().all(|r| r.kind == EntityKind::Edge) {
        let ea = a.topology.edges.iter().find(|e| e.id == selected[0].id)?;
        let eb = b.topology.edges.iter().find(|e| e.id == selected[1].id)?;
        if let (CadCurve::Line { start: p, end: q }, CadCurve::Line { start: r, end: s }) =
            (&ea.curve, &eb.curve)
        {
            let (p, q, r, s) = (
                ia.placement.transform_point(*p),
                ia.placement.transform_point(*q),
                ib.placement.transform_point(*r),
                ib.placement.transform_point(*s),
            );
            let (u, v) = (sub(q, p), sub(s, r));
            let denom = norm(u) * norm(v);
            if denom > 1e-12 {
                let cosine = (dot(u, v).abs() / denom).clamp(0., 1.);
                return Some(if cosine > 1. - 1e-9 {
                    format!(
                        "2 line edges | Min distance: {:.2} mm",
                        segment_distance(p, q, r, s)
                    )
                } else {
                    format!("2 line edges | Angle: {:.2}°", cosine.acos().to_degrees())
                });
            }
        }
    }
    if selected.iter().all(|r| r.kind == EntityKind::Face) {
        let fa = a.topology.faces.iter().find(|f| f.id == selected[0].id)?;
        let fb = b.topology.faces.iter().find(|f| f.id == selected[1].id)?;
        if let (
            CadSurface::Plane {
                origin: p,
                normal: u,
            },
            CadSurface::Plane {
                origin: q,
                normal: v,
            },
        ) = (&fa.surface, &fb.surface)
        {
            let (p, q, u, v) = (
                ia.placement.transform_point(*p),
                ib.placement.transform_point(*q),
                ia.placement.transform_vector(*u),
                ib.placement.transform_vector(*v),
            );
            let denom = norm(u) * norm(v);
            if denom > 1e-12 {
                let cosine = (dot(u, v).abs() / denom).clamp(0., 1.);
                return Some(if cosine > 1. - 1e-9 {
                    format!(
                        "2 planar faces | Plane separation: {:.2} mm",
                        dot(sub(q, p), u).abs() / norm(u)
                    )
                } else {
                    format!("2 planar faces | Angle: {:.2}°", cosine.acos().to_degrees())
                });
            }
        }
    }
    Some("2 features selected | Measurement unavailable".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn finite_segment_measurement_handles_disjoint_parallel_edges() {
        assert!(
            (segment_distance([0., 0., 0.], [1., 0., 0.], [2., 3., 0.], [3., 3., 0.])
                - 10_f64.sqrt())
            .abs()
                < 1e-10
        );
        assert!(
            (segment_distance([0., 0., 0.], [4., 0., 0.], [1., 3., 0.], [2., 3., 0.]) - 3.).abs()
                < 1e-10
        );
    }
    #[test]
    fn grid_uses_machine_travel_and_adapts_to_zoom() {
        let mut camera = OrbitCamera::framed([-50., -50., 0.], [50., 50., 0.]);
        let size = iced::Size::new(800., 600.);
        let grid = GridLayout::new(camera, size, [400., 200., 150.], Default::default());
        assert_eq!(grid.min, [-200., -100.]);
        assert_eq!(grid.max, [200., 100.]);
        assert!(grid.ticks(0).contains(&0.));
        assert!(grid.vertices().iter().all(|v| v.position[2] == 0.));
        camera.zoom = 3.;
        let zoomed = GridLayout::new(camera, size, [400., 200., 150.], Default::default());
        assert!(zoomed.step < grid.step);
        assert_eq!(zoomed.min, grid.min);
        assert!(zoomed.ticks(0).len() <= 101);
        camera.zoom = 100.;
        let close = GridLayout::new(camera, size, [400., 200., 150.], Default::default());
        assert!(close.step < zoomed.step);
        assert!(close.ticks(0).len() <= 101);
        assert_eq!(tick_text(-0.2, 0.1), "-0.2");
    }
    #[test]
    fn cardinal_side_views_use_the_matching_vertical_grid() {
        use std::f32::consts::{FRAC_PI_2, PI};
        let mut camera = OrbitCamera::framed([-50., -50., 0.], [50., 50., 0.]);
        let size = iced::Size::new(800., 600.);
        for (yaw, expected, normal, half) in [
            (0., GridPlane::Xz, 1, [200., 75.]),
            (PI, GridPlane::Xz, 1, [200., 75.]),
            (FRAC_PI_2, GridPlane::Yz, 0, [100., 75.]),
            (-FRAC_PI_2, GridPlane::Yz, 0, [100., 75.]),
        ] {
            camera.orient = cam_render::orientation(yaw, -FRAC_PI_2);
            let grid = GridLayout::new(camera, size, [400., 200., 150.], Default::default());
            assert_eq!(grid.plane, expected);
            assert_eq!(grid.max, half);
            assert!(grid.ticks(1).contains(&0.));
            assert!(grid.vertices().iter().all(|v| v.position[normal] == 0.));
            camera.zoom = 3.;
            assert!(
                GridLayout::new(camera, size, [400., 200., 150.], Default::default()).step
                    < grid.step
            );
            camera.zoom = 0.;
        }
        camera.orient = cam_render::orientation(0., -FRAC_PI_2 + 0.01);
        assert_eq!(
            GridPlane::for_camera(camera, Default::default()),
            GridPlane::Xy
        );
        camera.orient = cam_render::orientation(0.3, 0.8);
        assert_eq!(
            GridPlane::for_camera(camera, Default::default()),
            GridPlane::Xy
        );
        camera.orient = cam_render::orientation(0., 0.);
        assert_eq!(
            GridPlane::for_camera(camera, Default::default()),
            GridPlane::Xy
        );
        camera.orient = cam_render::orientation(0., PI);
        assert_eq!(
            GridPlane::for_camera(camera, Default::default()),
            GridPlane::Xy
        );
    }
    #[test]
    fn grid_geometry_and_picking_follow_work_frame() {
        let mut camera = OrbitCamera::framed([-50.; 3], [50.; 3]);
        camera.orient =
            cam_render::orientation(std::f32::consts::FRAC_PI_2, -std::f32::consts::FRAC_PI_2);
        let frame = cam_model::RigidTransform {
            rotation: [[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]],
            translation: [10., 20., 30.],
        };
        let size = iced::Size::new(800., 600.);
        let grid = GridLayout::new(camera, size, [400., 200., 150.], frame);
        assert_eq!(grid.plane, GridPlane::Xy);
        assert_eq!(grid.point([0., 0.]), [10., 20., 30.]);
        assert_eq!(grid.point([5., 7.]), [10., 27., 25.]);
        assert!(grid
            .vertices()
            .iter()
            .all(|v| (v.position[0] - 10.).abs() < 1e-5));
        let picked = grid
            .plane
            .pick(camera, frame, 0., 0., size.width / size.height)
            .unwrap();
        let world = frame.transform_point(grid.plane.point(picked).map(f64::from));
        assert!((world[0] - 10.).abs() < 1e-8);
        camera.orient = cam_render::orientation(0., 0.);
        assert_eq!(GridPlane::for_camera(camera, frame), GridPlane::Yz);
    }

    #[test]
    fn clipping_keeps_labels_inside_the_viewport() {
        let (a, b) = clipped_line(
            iced::Point::new(-50., 100.),
            iced::Point::new(900., 100.),
            iced::Size::new(800., 600.),
        )
        .unwrap();
        assert_eq!(a.x, 18.);
        assert_eq!(b.x, 782.);
    }
}
