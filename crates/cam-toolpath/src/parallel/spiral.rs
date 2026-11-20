//! Archimedean spirals and concentric circles projected onto the setup surface.
use super::surface_passes::{emit_path, PassSettings};
use super::*;
use cam_model::{
    SpiralBoundary, SpiralCutDirection, SpiralMilling, SpiralMode, SpiralOp, SpiralOrder,
    SpiralParams,
};
use std::f64::consts::TAU;

const MAX_POINTS: usize = 2_000_000;
const MAX_CURVES: usize = 100_000;

#[derive(Clone, Debug)]
pub struct SpiralStrategy {
    op: SpiralOp,
    stock: StockToLeave,
    geometry: ParallelGeometry,
}

impl SpiralStrategy {
    pub const DOMAIN_REVISION: u32 = 2;

    pub fn new(op: SpiralOp, stock: StockToLeave, geometry: ParallelGeometry) -> Self {
        Self {
            op,
            stock,
            geometry,
        }
    }
}

impl Strategy for SpiralStrategy {
    fn name(&self) -> &str {
        "spiral"
    }

    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        let mut operation = cam_model::Operation::Spiral(self.op.clone());
        if let Err(reason) = operation.resolve_parameter_expressions(env.tools) {
            return failed(reason);
        }
        let cam_model::Operation::Spiral(op) = operation else {
            unreachable!()
        };
        match self.compute_resolved(&op, env, cancel) {
            _ if cancel.is_cancelled() => StrategyResult {
                cancelled: true,
                ..Default::default()
            },
            Ok(result) => result,
            Err(reason) => failed(reason),
        }
    }
}

fn failed(reason: String) -> StrategyResult {
    StrategyResult {
        diagnostics: vec![Diagnostic::error(reason)],
        ..Default::default()
    }
}

impl SpiralStrategy {
    fn compute_resolved(
        &self,
        op: &SpiralOp,
        env: &JobEnv,
        cancel: &CancelToken,
    ) -> Result<StrategyResult, String> {
        let mut p = op.params;
        p.clockwise = clockwise(p, env);
        if cancel.is_cancelled() {
            return Err("Spiral cancelled".into());
        }
        validate(p)?;
        let offsets = depth_offsets(p)?;
        let projection = SurfaceProjection::new(
            "Spiral",
            &self.geometry,
            self.stock,
            env,
            SurfaceOptions {
                tool: op.tool,
                entry_feeds: op.entry_feeds,
                tolerance: p.tolerance,
                containment: AdaptiveContainment::Center,
                additional_offset: 0.,
                angle: 0.,
            },
            cancel,
        )?;
        let mut linker = projection.linker();
        let domain = Domain::new(p, &projection, env, cancel)?;
        let mut result = StrategyResult::default();
        let paths = project_paths(&domain, &linker, p, cancel)?;
        let pass = PassSettings {
            id: op.id,
            feed: p.feed,
            plunge_feed: p.plunge_feed,
            tolerance: p.tolerance,
            retraction: p.retraction,
            safe_distance: p.safe_distance,
            max_stay_down_distance: p.max_stay_down_distance,
            smoothing_tolerance: p.smoothing.then_some(p.smoothing_tolerance),
        };
        // Keep the immutable footprint separate from the mutable motion emitter.
        let selected = linker.selected;
        let check_chord = |from, to| domain.covers_segment(selected, from, to, cancel);
        let order = if p.order == SpiralOrder::Shortest && !p.link_inside_out {
            scallop::shortest_order(&paths, &vec![0; paths.len()], false, cancel)
                .ok_or("Spiral cancelled")?
                .into_iter()
                .map(|(index, _)| index)
                .collect()
        } else {
            (0..paths.len()).collect::<Vec<_>>()
        };
        if paths
            .iter()
            .map(Vec::len)
            .sum::<usize>()
            .saturating_mul(offsets.len())
            > MAX_POINTS
        {
            return Err(
                "Spiral depth passes exceed the motion budget; increase stepover or stepdown"
                    .into(),
            );
        }
        let base_axial = linker.axial;
        let base_bottom = linker.bottom;
        let top = linker.top + linker.error + p.tolerance + base_axial;
        let mut depth = None;
        let mut lifted = Vec::new();
        for index in 0..order.len() * offsets.len() {
            if cancel.is_cancelled() {
                return Err("Spiral cancelled".into());
            }
            let (path_index, layer) = if p.order_by_depth {
                (index % order.len(), index / order.len())
            } else {
                (index / offsets.len(), index % offsets.len())
            };
            let offset = offsets[layer];
            if depth != Some(layer) {
                linker.retract(&mut result, op.id);
                linker.axial = base_axial + offset;
                linker.bottom = (base_bottom + offset).min(top);
                depth = Some(layer);
            }
            let path = &paths[order[path_index]];
            let path = if offset > 0. {
                lifted.clear();
                lifted.extend(path.iter().map(|&[x, y, z]| [x, y, (z + offset).min(top)]));
                if lifted.iter().any(|q| q[2] >= linker.clearance) {
                    return Err("Spiral clearance must exceed every axial offset pass".into());
                }
                &lifted
            } else {
                path
            };
            emit_path(
                &mut result,
                path,
                &mut linker,
                pass,
                Some(&check_chord),
                cancel,
            );
            if result.program.steps().len() > MAX_POINTS {
                return Err(
                    "Spiral exceeds the two-million motion budget; increase stepover or tolerance"
                        .into(),
                );
            }
        }
        linker.retract(&mut result, op.id);
        if linker.passes == 0 {
            return Err(
                "Spiral produced no surface contact passes within the selected limits".into(),
            );
        }
        result.diagnostics.push(Diagnostic::info(format!(
            "3D Spiral: {} at {:.3} mm radial stepover",
            crate::passes_phrase(linker.passes),
            p.stepover
        )));
        Ok(result)
    }
}

fn clockwise(p: SpiralParams, env: &JobEnv) -> bool {
    match p.cut_direction {
        SpiralCutDirection::BothWays => p.clockwise,
        direction => {
            // The uncut strip is on the outside for outward passes and on
            // the inside for inward passes. Reverse the sense for M4.
            let outward = p.order != SpiralOrder::OutsideIn;
            let climb = direction == SpiralCutDirection::Climb;
            (outward == climb) != env.climb_follows_winding()
        }
    }
}

fn validate(p: SpiralParams) -> Result<(), String> {
    if [p.tolerance, p.stepover, p.feed, p.plunge_feed]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.)
        || !p.additional_offset.is_finite()
        || p.center
            .is_some_and(|point| !point.into_iter().all(f64::is_finite))
    {
        return Err("Spiral requires finite positive tolerance, stepover and feeds, and a finite center and offset".into());
    }
    if [p.inner_limit, p.outer_limit]
        .iter()
        .any(|v| !v.is_finite() || *v < 0.)
        || (p.outer_limit > 0. && p.outer_limit <= p.inner_limit)
    {
        return Err("Spiral limits must be nonnegative and the outer radius must exceed the inner radius when specified".into());
    }
    if !(0. ..=90.).contains(&p.from_slope) || !(p.from_slope..=90.).contains(&p.to_slope) {
        return Err("Spiral slope range must be ordered between 0° and 90°".into());
    }
    if [p.safe_distance, p.max_stay_down_distance]
        .iter()
        .any(|v| !v.is_finite() || *v < 0.)
        || (p.smoothing && (!p.smoothing_tolerance.is_finite() || p.smoothing_tolerance <= 0.))
    {
        return Err(
            "Spiral link distances and smoothing tolerance must be finite and valid".into(),
        );
    }
    Ok(())
}

fn depth_offsets(p: SpiralParams) -> Result<Vec<f64>, String> {
    if !p.multiple_depths {
        return Ok(vec![0.]);
    }
    if !p.axial_offset.is_finite()
        || p.axial_offset <= 0.
        || !p.max_stepdown.is_finite()
        || p.max_stepdown <= 0.
    {
        return Err("Spiral axial offset and maximum stepdown must be finite and positive".into());
    }
    let minimum = (p.axial_offset / p.max_stepdown).ceil().max(1.);
    let count = if p.stepdowns == 0 {
        minimum
    } else {
        f64::from(p.stepdowns)
    };
    if count < minimum || count > 1000. {
        return Err(
            "Spiral stepdowns must honor the maximum stepdown and stay within 1000 passes".into(),
        );
    }
    Ok((0..count as usize)
        .rev()
        .map(|index| p.axial_offset * (index as f64 / count))
        .collect())
}

struct Domain {
    bounds: [[f64; 2]; 2],
    silhouette: Vec<cam_geo::Polygon>,
    contact_only: bool,
    center: [f64; 2],
    inner: f64,
    outer: f64,
    complete_circle_radius: f64,
    chord_min_radius: f64,
    slope: Option<[f64; 2]>,
}

impl Domain {
    fn new(
        p: SpiralParams,
        projection: &SurfaceProjection,
        env: &JobEnv,
        cancel: &CancelToken,
    ) -> Result<Self, String> {
        let [lo, hi] = projection.bounds;
        let center = p
            .center
            .unwrap_or_else(|| [lo[0].midpoint(hi[0]), lo[1].midpoint(hi[1])]);
        let inset = containment_inset(p.containment, p.additional_offset, projection.radius);
        let mut silhouette = Vec::new();
        let [lo, hi] = match p.boundary {
            SpiralBoundary::BoundingBox => inset_bounds([lo, hi], inset),
            SpiralBoundary::StockBox => {
                let (lo, hi) = env
                    .stock
                    .ok_or("Spiral requires resolved stock bounds for this boundary")?;
                inset_bounds([lo, hi], inset)
            }
            SpiralBoundary::Silhouette => {
                let footprint = cam_geometry3d::projected_footprint(projection.selected.mesh)?;
                if cancel.is_cancelled() {
                    return Err("Spiral cancelled".into());
                }
                // Include the independent mesh error when the cutter must stay inside.
                let error = if p.containment == AdaptiveContainment::Inside {
                    projection.error
                } else {
                    0.
                };
                silhouette = if inset + error == 0. {
                    footprint
                } else {
                    cam_geo::offset(&footprint, -inset - error, cam_geo::JoinStyle::Round)
                        .map_err(|e| format!("Could not offset the Spiral silhouette: {e}"))?
                };
                silhouette
                    .iter()
                    .flat_map(|polygon| polygon.outer().points())
                    .fold(
                        [[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]],
                        |[lo, hi], point| {
                            [
                                [lo[0].min(point.x), lo[1].min(point.y)],
                                [hi[0].max(point.x), hi[1].max(point.y)],
                            ]
                        },
                    )
            }
        };
        if (0..2).any(|axis| !lo[axis].is_finite() || !hi[axis].is_finite() || lo[axis] >= hi[axis])
        {
            return Err("Spiral containment leaves no machining area".into());
        }
        let farthest = [0, 1].map(|axis| {
            (lo[axis] - center[axis])
                .abs()
                .max((hi[axis] - center[axis]).abs())
        });
        let outer = farthest[0].hypot(farthest[1]);
        let outer = if p.outer_limit > 0. {
            outer.min(p.outer_limit)
        } else {
            outer
        };
        if !outer.is_finite() || outer <= p.inner_limit {
            return Err("Spiral radii leave no machining area".into());
        }
        let complete_circle_radius = [0, 1]
            .map(|axis| (center[axis] - lo[axis]).min(hi[axis] - center[axis]))
            .into_iter()
            .fold(f64::INFINITY, f64::min);
        let complete_circle_radius = if silhouette.is_empty() {
            complete_circle_radius
        } else {
            let origin = cam_geo::Point::new(center[0], center[1]);
            silhouette
                .iter()
                .find(|polygon| polygon.contains(origin))
                .map_or(0., |polygon| {
                    boundary_edges(polygon)
                        .map(|(a, b)| distance_to_chord([a.x, a.y, 0.], [b.x, b.y, 0.], center))
                        .fold(complete_circle_radius, f64::min)
                })
        };
        Ok(Self {
            bounds: [lo, hi],
            silhouette,
            contact_only: p.contact_only,
            center,
            inner: p.inner_limit,
            outer,
            complete_circle_radius,
            // Circular chords may enter the nominal radius within the machining tolerance.
            chord_min_radius: (p.inner_limit - p.tolerance * 0.25).max(0.),
            slope: (p.from_slope != 0. || p.to_slope != 90.).then_some([p.from_slope, p.to_slope]),
        })
    }

    fn boundary_contains(&self, a: V, b: V) -> bool {
        if self.silhouette.is_empty() {
            return true;
        }
        let point = |v: V| cam_geo::Point::new(v[0], v[1]);
        let a = point(a);
        let b = point(b);
        self.silhouette.iter().any(|region| {
            if !region.contains(a) || (a != b && !region.contains(b)) {
                return false;
            }
            if a == b {
                return true;
            }
            let crosses = |c: cam_geo::Point, d: cam_geo::Point| {
                if a.x.min(b.x) > c.x.max(d.x)
                    || a.x.max(b.x) < c.x.min(d.x)
                    || a.y.min(b.y) > c.y.max(d.y)
                    || a.y.max(b.y) < c.y.min(d.y)
                {
                    return false;
                }
                let orient = |p: cam_geo::Point, q: cam_geo::Point, r: cam_geo::Point| {
                    (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)
                };
                orient(a, b, c) * orient(a, b, d) <= 0. && orient(c, d, a) * orient(c, d, b) <= 0.
            };
            !boundary_edges(region).any(|(c, d)| crosses(c, d))
        })
    }

    fn within_bounds(&self, q: V) -> bool {
        (0..2).all(|axis| (self.bounds[0][axis]..=self.bounds[1][axis]).contains(&q[axis]))
            && (q[0] - self.center[0]).hypot(q[1] - self.center[1]) <= self.outer + 1e-9
    }

    fn includes_slope(&self, triangle: &[V; 3]) -> bool {
        self.slope.is_none_or(|[min, max]| {
            let n = cross(sub(triangle[1], triangle[0]), sub(triangle[2], triangle[0]));
            let slope = n[0].hypot(n[1]).atan2(n[2].abs()).to_degrees();
            slope >= min - 1e-9 && slope <= max + 1e-9
        })
    }

    fn covers_segment(&self, selected: &Grid, a: V, b: V, cancel: &CancelToken) -> bool {
        distance_to_chord(a, b, self.center) >= self.chord_min_radius
            && self.within_bounds(a)
            && self.within_bounds(b)
            && self.boundary_contains(a, b)
            && ((!self.contact_only && self.slope.is_none())
                || selected.covers_segment_where(a, b, |t| self.includes_slope(t), cancel))
    }

    fn project(
        &self,
        mut q: V,
        linker: &PassLinker,
        tolerance: f64,
        cancel: &CancelToken,
    ) -> Option<V> {
        if !self.within_bounds(q) || !self.boundary_contains(q, q) {
            return None;
        }
        let contact = if self.slope.is_some() {
            linker
                .selected
                .covers_segment_where(q, q, |t| self.includes_slope(t), cancel)
        } else {
            !self.contact_only || linker.selected.contact(q)
        };
        if !contact {
            return None;
        }
        let reach = linker.radius + linker.error;
        let lift = linker.error + tolerance + linker.axial;
        let z = linker.grid.height(q, reach).unwrap_or(f64::NEG_INFINITY);
        let wall = linker
            .grid
            .indices(q, reach)
            .into_iter()
            .filter_map(|i| flat_height(q, &linker.grid.mesh[i], reach))
            .fold(f64::NEG_INFINITY, f64::max);
        q[2] = (z + lift)
            .max(wall - linker.radius + lift)
            .max(linker.bottom);
        (q[2] <= linker.top + lift && q[2] < linker.clearance).then_some(q)
    }
}

fn boundary_edges(
    polygon: &cam_geo::Polygon,
) -> impl Iterator<Item = (cam_geo::Point, cam_geo::Point)> + '_ {
    std::iter::once(polygon.outer())
        .chain(polygon.holes())
        .flat_map(|ring| {
            let points = ring.points();
            points
                .iter()
                .copied()
                .zip(points.iter().copied().cycle().skip(1))
        })
}

fn distance_to_chord(a: V, b: V, point: [f64; 2]) -> f64 {
    let d = sub(b, a);
    let length2 = d[0] * d[0] + d[1] * d[1];
    let closest = if length2 > 0. {
        let t = ((point[0] - a[0]) * d[0] + (point[1] - a[1]) * d[1]) / length2;
        add(a, mul(d, t.clamp(0., 1.)))
    } else {
        a
    };
    (closest[0] - point[0]).hypot(closest[1] - point[1])
}

fn inset_bounds([lo, hi]: [[f64; 2]; 2], inset: f64) -> [[f64; 2]; 2] {
    [lo.map(|v| v + inset), hi.map(|v| v - inset)]
}

#[derive(Clone, Copy)]
struct Curve {
    from: f64,
    to: f64,
    start_angle: f64,
    sweep: f64,
}

impl Curve {
    fn circle(radius: f64, start_angle: f64) -> Self {
        Self {
            from: radius,
            to: radius,
            start_angle,
            sweep: TAU,
        }
    }

    fn radius(self, angle: f64) -> f64 {
        self.from + (self.to - self.from) * (angle / self.sweep)
    }

    fn point(self, center: [f64; 2], angle: f64, sense: f64) -> V {
        let radius = self.radius(angle);
        // Close full circles at the exact first point for emitters and posts.
        let phi = if angle == self.sweep && self.from == self.to {
            self.start_angle
        } else {
            self.start_angle + sense * angle
        };
        let (s, c) = phi.sin_cos();
        [center[0] + radius * c, center[1] + radius * s, 0.]
    }
}

fn curves(domain: &Domain, p: SpiralParams, sense: f64) -> Result<Vec<Curve>, String> {
    let inward = p.order == SpiralOrder::OutsideIn;
    let inner = domain.inner;
    let outer = domain.outer;
    let turns = (outer - inner) / p.stepover;
    if !turns.is_finite() || turns > MAX_CURVES as f64 {
        return Err(
            "Spiral exceeds its turn budget; increase the stepover or reduce the outer limit"
                .into(),
        );
    }
    if p.mode == SpiralMode::ConcentricCircles {
        let first = if inner == 0. {
            (p.stepover * 0.5).min(outer)
        } else {
            inner
        };
        let intervals = ((outer - first) / p.stepover).ceil() as usize;
        let mut rings = (0..=intervals)
            .map(|i| Curve::circle((first + i as f64 * p.stepover).min(outer), 0.))
            .collect::<Vec<_>>();
        if inward {
            rings.reverse();
        }
        return Ok(rings);
    }
    let mut curves = Vec::with_capacity(3);
    let (from, to) = if inward {
        (outer, inner)
    } else {
        (inner, outer)
    };
    let sweep = turns * TAU;
    if sweep == 0. {
        return Err("Spiral sweep is too small; check the radius limits and stepover".into());
    }
    let spiral = Curve {
        from,
        to,
        start_angle: 0.,
        sweep,
    };
    let ring = |radius: f64| {
        p.mode == SpiralMode::SpiralWithCircles
            && radius > 0.
            && radius < domain.complete_circle_radius
            && (radius == inner || (p.outer_limit > 0. && radius == p.outer_limit))
    };
    if ring(from) {
        curves.push(Curve::circle(from, 0.));
    }
    curves.push(spiral);
    if ring(to) {
        curves.push(Curve::circle(to, (sense * sweep).rem_euclid(TAU)));
    }
    Ok(curves)
}

fn project_paths(
    domain: &Domain,
    linker: &PassLinker,
    p: SpiralParams,
    cancel: &CancelToken,
) -> Result<Vec<Vec<V>>, String> {
    let mut paths = Vec::new();
    let mut samples = 0;
    let mut vertices = 0;
    let mut refined = Vec::new();
    let sense = if p.clockwise { -1. } else { 1. };
    let spacing = (linker.radius * p.tolerance).sqrt().clamp(0.01, 0.5);
    let margin = linker.error + p.tolerance + linker.axial;
    for curve in curves(domain, p, sense)? {
        let mut path = Vec::new();
        let mut angle = 0.;
        let radial_rate = ((curve.to - curve.from) / curve.sweep).abs();
        loop {
            samples += 1;
            if samples > MAX_POINTS || vertices > MAX_POINTS || paths.len() > MAX_CURVES {
                return Err(
                    "Spiral exceeds its sampling budget; increase stepover or tolerance".into(),
                );
            }
            if samples % 128 == 0 && cancel.is_cancelled() {
                return Err("Spiral cancelled".into());
            }
            let xy = curve.point(domain.center, angle, sense);
            if let Some(point) = domain.project(xy, linker, p.tolerance, cancel) {
                if let Some(from) = path.last().copied() {
                    refined.clear();
                    if domain.covers_segment(linker.selected, from, point, cancel)
                        && refine(
                            linker.grid,
                            from,
                            point,
                            linker.radius,
                            linker.error,
                            margin,
                            p.tolerance,
                            linker.bottom,
                            0,
                            &mut refined,
                            cancel,
                        )
                        && refined
                            .iter()
                            .all(|q| q[2] <= linker.top + margin && q[2] < linker.clearance)
                    {
                        vertices += refined.len();
                        path.extend_from_slice(&refined);
                    } else {
                        finish_path(&mut paths, &mut path, p.milling);
                        path.push(point);
                    }
                } else {
                    path.push(point);
                }
            } else {
                finish_path(&mut paths, &mut path, p.milling);
            }
            if angle >= curve.sweep {
                break;
            }
            // Bound the spatial sampling and the chord error of the planar spiral.
            // For (r cos t, r sin t), |d²P/dt²| is at most r + 2|dr/dt|.
            let radius = curve
                .radius(angle)
                .max(curve.radius((angle + 0.15).min(curve.sweep)));
            let step = 0.15_f64
                .min(spacing / (radius + radial_rate))
                .min((2. * p.tolerance / (radius + 2. * radial_rate)).sqrt());
            let next = (angle + step).min(curve.sweep);
            if next <= angle {
                return Err("Spiral sampling cannot meet the requested tolerance".into());
            }
            angle = next;
        }
        finish_path(&mut paths, &mut path, p.milling);
    }
    if paths.len() > MAX_CURVES || vertices > MAX_POINTS {
        return Err("Spiral exceeds its sampling budget; increase stepover or tolerance".into());
    }
    Ok(paths)
}

fn finish_path(paths: &mut Vec<Vec<V>>, path: &mut Vec<V>, milling: SpiralMilling) {
    if path.len() < 2 {
        path.clear();
        return;
    }
    if milling == SpiralMilling::Both {
        paths.push(std::mem::take(path));
        return;
    }
    let mut from = 0;
    let mut rising = None;
    for (index, pair) in path.windows(2).enumerate() {
        let dz = pair[1][2] - pair[0][2];
        if dz.abs() <= 1e-9 {
            continue;
        }
        let next = dz > 0.;
        if rising.is_some_and(|previous| previous != next) {
            push_monotone(paths, &path[from..=index], rising, milling);
            from = index;
        }
        rising = Some(next);
    }
    push_monotone(paths, &path[from..], rising, milling);
    path.clear();
}

fn push_monotone(
    paths: &mut Vec<Vec<V>>,
    points: &[V],
    rising: Option<bool>,
    milling: SpiralMilling,
) {
    let mut path = points.to_vec();
    if rising.is_some_and(|up| up != (milling == SpiralMilling::Up)) {
        path.reverse();
    }
    paths.push(path);
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_cldata::{SpindleDir, Step};
    use cam_model::{CuttingHeights, Heights, ScallopRetraction, Tool};

    fn rectangle(x: [f64; 2], y: [f64; 2], z: impl Fn(f64, f64) -> f64) -> [[V; 3]; 2] {
        let [a, b, c, d] =
            [[x[0], y[0]], [x[1], y[0]], [x[1], y[1]], [x[0], y[1]]].map(|[x, y]| [x, y, z(x, y)]);
        [[a, b, c], [a, c, d]]
    }

    fn geometry(triangles: Vec<[V; 3]>) -> ParallelGeometry {
        ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 4.,
                bottom: -5.,
            },
        }
    }

    fn run(
        params: SpiralParams,
        geometry: ParallelGeometry,
        cancel: &CancelToken,
    ) -> StrategyResult {
        let tools = [Tool {
            kind: ToolKind::BallMill,
            diameter: 2.,
            number: 1,
            length: 20.,
            ..Default::default()
        }];
        let env = JobEnv {
            heights: Heights::new(10., 5., 4.),
            tools: &tools,
            stock: Some(([-10.; 2], [10.; 2])),
            spindle: SpindleDir::Cw,
        };
        let op = SpiralOp {
            parameter_expressions: Default::default(),
            id: 7,
            tool: 1,
            work_offset: 1,
            spindle_rpm: 1000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            params,
        };
        SpiralStrategy::new(op, StockToLeave::default(), geometry).compute(&env, cancel)
    }

    fn cuts(result: &StrategyResult) -> Vec<Vec<Point3>> {
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(result.program.validate().is_ok());
        let mut paths: Vec<Vec<_>> = Vec::new();
        for step in result.program.steps() {
            match step {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => {
                    paths.push(vec![*to])
                }
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                    paths.last_mut().unwrap().push(*to)
                }
                _ => {}
            }
        }
        paths
    }

    #[test]
    fn continuous_spiral_obeys_radial_pitch_order_and_winding() {
        let center = [2., -1.];
        let radial = |p: Point3| (p.x - center[0]).hypot(p.y - center[1]);
        for order in [
            SpiralOrder::InsideOut,
            SpiralOrder::OutsideIn,
            SpiralOrder::Shortest,
        ] {
            for clockwise in [false, true] {
                let result = run(
                    SpiralParams {
                        center: Some(center),
                        inner_limit: 1.,
                        outer_limit: 5.,
                        stepover: 1.,
                        order,
                        clockwise,
                        ..Default::default()
                    },
                    geometry(rectangle([-10., 10.], [-10., 10.], |_, _| 0.).to_vec()),
                    &CancelToken::new(),
                );
                let paths = cuts(&result);
                assert_eq!(paths.len(), 1);
                let points = &paths[0];
                let inward = order == SpiralOrder::OutsideIn;
                let mut angle = 0.;
                for pair in points.windows(2) {
                    let a = [pair[0].x - center[0], pair[0].y - center[1]];
                    let b = [pair[1].x - center[0], pair[1].y - center[1]];
                    let step = (a[0] * b[1] - a[1] * b[0]).atan2(a[0] * b[0] + a[1] * b[1]);
                    assert_eq!(step.is_sign_negative(), clockwise);
                    angle += step.abs();
                    let expected = if inward {
                        5. - angle / TAU
                    } else {
                        1. + angle / TAU
                    };
                    assert!((radial(pair[1]) - expected).abs() < 1e-8);
                }
                assert!((radial(points[0]) - if inward { 5. } else { 1. }).abs() < 1e-9);
                assert!(
                    (radial(*points.last().unwrap()) - if inward { 1. } else { 5. }).abs() < 1e-9
                );
            }
        }
    }

    #[test]
    fn ring_modes_follow_limits_and_smoothing_can_emit_arcs() {
        let flat = geometry(rectangle([-10., 10.], [-10., 10.], |_, _| 0.).to_vec());
        let p = SpiralParams {
            inner_limit: 2.,
            outer_limit: 5.,
            stepover: 2.,
            mode: SpiralMode::SpiralWithCircles,
            ..Default::default()
        };
        let cancel = CancelToken::new();
        let paths = cuts(&run(p, flat.clone(), &cancel));
        assert_eq!(paths.len(), 3);
        let radius = |p: &Point3| p.x.hypot(p.y);
        for (path, expected) in [(&paths[0], 2.), (&paths[2], 5.)] {
            assert_eq!(path.first(), path.last());
            assert!(path.iter().all(|pt| (radius(pt) - expected).abs() < 1e-9));
        }
        let p = SpiralParams {
            mode: SpiralMode::ConcentricCircles,
            ..p
        };
        let paths = cuts(&run(p, flat.clone(), &cancel));
        assert_eq!(paths.len(), 3);
        for (path, expected) in paths.iter().zip([2., 4., 5.]) {
            assert_eq!(path.first(), path.last());
            assert!(path.iter().all(|pt| (radius(pt) - expected).abs() < 1e-9));
        }
        let smoothed = run(
            SpiralParams {
                smoothing: true,
                ..p
            },
            flat,
            &cancel,
        );
        assert!(smoothed.program.validate().is_ok());
        assert!(smoothed
            .program
            .steps()
            .iter()
            .any(|step| matches!(step, Step::Arc { .. })));
    }

    #[test]
    fn disconnected_selection_retracts_and_sloped_cuts_can_be_oriented_uphill() {
        let surfaces = [
            rectangle([-8., -0.02], [-8., 8.], |x, _| x * 0.25),
            rectangle([0.02, 8.], [-8., 8.], |x, _| x * 0.25),
        ]
        .concat();
        let p = SpiralParams {
            stepover: 1.5,
            outer_limit: 6.,
            milling: SpiralMilling::Up,
            from_slope: 12.,
            to_slope: 15.,
            ..Default::default()
        };
        let flat = rectangle([-8., 8.], [-8., 8.], |_, _| -3.);
        let mut geo = geometry(surfaces);
        geo.triangles.extend(flat);
        let paths = cuts(&run(p, geo.clone(), &CancelToken::new()));
        assert!(paths.len() > 2);
        assert!(paths.iter().all(|path| {
            path.iter().all(|q| q.x.signum() == path[0].x.signum())
                && path.windows(2).all(|pair| pair[0].z <= pair[1].z + 1e-9)
        }));
        let result = run(
            SpiralParams {
                from_slope: 20.,
                to_slope: 30.,
                ..p
            },
            geo,
            &CancelToken::new(),
        );
        assert!(result.has_errors());
        assert!(result.program.steps().is_empty());
    }

    #[test]
    fn silhouette_offsets_limit_contact_and_multiple_depths_reach_the_final_surface() {
        let selected = [
            rectangle([-8., 8.], [-8., 0.], |_, _| 0.),
            rectangle([-8., 0.], [0., 8.], |_, _| 0.),
        ]
        .concat();
        let mut geo = geometry(rectangle([-10., 10.], [-10., 10.], |_, _| 0.).to_vec());
        geo.selected = selected;
        let p = SpiralParams {
            mode: SpiralMode::ConcentricCircles,
            center: Some([-3., -3.]),
            outer_limit: 8.,
            stepover: 2.,
            contact_only: false,
            containment: AdaptiveContainment::Inside,
            ..Default::default()
        };
        let cancel = CancelToken::new();
        let bounding = cuts(&run(p, geo.clone(), &cancel));
        assert!(bounding.iter().flatten().any(|q| q.x > 0. && q.y > 0.));
        let p = SpiralParams {
            boundary: SpiralBoundary::Silhouette,
            multiple_depths: true,
            axial_offset: 1.8,
            max_stepdown: 0.7,
            stepdowns: 3,
            ..p
        };
        let paths = cuts(&run(p, geo.clone(), &cancel));
        assert!(!paths.is_empty());
        assert!(paths.iter().flatten().all(|q| q.x < 0. || q.y < 0.));
        let elevations: Vec<_> = paths.iter().map(|p| p[0].z).collect();
        assert!(elevations.windows(2).all(|pair| pair[0] >= pair[1]));
        let z = *elevations.last().unwrap();
        assert!((elevations[0] - z - 1.2).abs() < 1e-9);
        assert!(elevations
            .iter()
            .any(|height| (*height - z - 0.6).abs() < 1e-9));
        let result = run(SpiralParams { stepdowns: 2, ..p }, geo, &cancel);
        assert!(result.has_errors() && result.program.steps().is_empty());
    }

    #[test]
    fn retraction_policies_control_links_between_spiral_passes() {
        let flat = geometry(rectangle([-10., 10.], [-10., 10.], |_, _| 0.).to_vec());
        let p = SpiralParams {
            mode: SpiralMode::ConcentricCircles,
            inner_limit: 2.,
            outer_limit: 5.,
            stepover: 1.,
            ..Default::default()
        };
        let cancel = CancelToken::new();
        let full = run(p, flat.clone(), &cancel);
        for (retraction, links, retracts, height) in [
            (ScallopRetraction::Full, 0, 4, 0.),
            (
                ScallopRetraction::Minimum,
                6,
                1,
                flat.error_bound + p.safe_distance,
            ),
            (
                ScallopRetraction::Shortest,
                3,
                1,
                flat.error_bound + p.tolerance,
            ),
        ] {
            let result = run(SpiralParams { retraction, ..p }, flat.clone(), &cancel);
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
            assert!(result.program.validate().is_ok());
            let steps = result.program.steps();
            assert!(matches!(steps.first(), Some(Step::Rapid { to, tag })
                if to.z == 10. && tag.kind == MoveKind::Link));
            assert!(matches!(steps.last(), Some(Step::Rapid { to, tag })
                if to.z == 10. && tag.kind == MoveKind::Retract));
            assert_eq!(
                steps
                    .iter()
                    .filter(|s| matches!(s,
                Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract))
                    .count(),
                retracts
            );
            let link_heights: Vec<_> = steps
                .iter()
                .filter_map(|s| match s {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Link => Some(to.z),
                    _ => None,
                })
                .collect();
            assert_eq!(link_heights.len(), links);
            assert!(link_heights.iter().all(|z| (z - height).abs() < 1e-9));
        }
        for fallback in [
            SpiralParams {
                retraction: ScallopRetraction::Shortest,
                max_stay_down_distance: 0.,
                ..p
            },
            SpiralParams {
                retraction: ScallopRetraction::Minimum,
                safe_distance: 10.,
                ..p
            },
        ] {
            assert_eq!(run(fallback, flat.clone(), &cancel).program, full.program);
        }
    }

    #[test]
    fn spiral_invalid_limits_and_cancellation_emit_no_motions() {
        let flat = geometry(rectangle([-10., 10.], [-10., 10.], |_, _| 0.).to_vec());
        let p = SpiralParams {
            inner_limit: 2.,
            outer_limit: 5.,
            ..Default::default()
        };
        let cancel = CancelToken::new();
        let result = run(
            SpiralParams {
                outer_limit: 1.,
                ..p
            },
            flat.clone(),
            &cancel,
        );
        assert!(result.has_errors() && result.program.steps().is_empty());
        cancel.cancel();
        let result = run(p, flat, &cancel);
        assert!(result.cancelled && result.program.steps().is_empty());
    }
}
