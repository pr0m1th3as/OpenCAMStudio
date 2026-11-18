//! Three-axis ball-end parallel finishing over the independent CAD projection.
use crate::{CancelToken, Diagnostic, JobEnv, Strategy, StrategyResult};
use cam_cldata::{MoveKind, Point3, Step, Tag};
use cam_geometry3d::ParallelGeometry;
use cam_model::{AdaptiveContainment, ParallelOp, StockToLeave, ToolKind};
use rayon::prelude::*;
use std::collections::HashMap;
use wide::{f64x4, CmpLe};
mod scallop;
mod spiral;
mod surface_passes;
pub use spiral::SpiralStrategy;
type V = [f64; 3];
fn sub(a: V, b: V) -> V {
    std::array::from_fn(|i| a[i] - b[i])
}
fn add(a: V, b: V) -> V {
    std::array::from_fn(|i| a[i] + b[i])
}
fn mul(a: V, s: f64) -> V {
    a.map(|x| x * s)
}
fn dot(a: V, b: V) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn inside(p: V, t: &[V; 3]) -> bool {
    let mut signs = [0.; 3];
    for i in 0..3 {
        let a = t[i];
        let b = t[(i + 1) % 3];
        signs[i] = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
    }
    signs.iter().all(|v| *v >= -1e-10) || signs.iter().all(|v| *v <= 1e-10)
}
/// Maximum triangle height minus the spherical cutter profile. Includes face,
/// edge and vertex contact (vertical triangles included).
fn drop_ball(p: V, t: &[V; 3], r: f64) -> Option<f64> {
    let mut best = f64::NEG_INFINITY;
    let mut candidate = |q: V| {
        let d = (q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2);
        if d <= r * r + 1e-10 {
            best = best.max(q[2] + (r * r - d).max(0.).sqrt() - r);
        }
    };
    for i in 0..3 {
        let a = t[i];
        let d = sub(t[(i + 1) % 3], a);
        let l = d[0].hypot(d[1]);
        if l < 1e-12 {
            candidate(a);
            candidate(t[(i + 1) % 3]);
            continue;
        }
        let along = ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l;
        let perp = ((p[0] - a[0]) * d[1] - (p[1] - a[1]) * d[0]) / l;
        if perp.abs() > r {
            continue;
        }
        let reach = (r * r - perp * perp).max(0.).sqrt();
        let slope = d[2] / l;
        let q = (along + reach * slope / (1. + slope * slope).sqrt()).clamp(0., l);
        candidate(add(a, mul(d, q / l)));
    }
    let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
    if n[2].abs() > 1e-12 {
        let gx = -n[0] / n[2];
        let gy = -n[1] / n[2];
        let den = (1. + gx * gx + gy * gy).sqrt();
        let mut q = [p[0] + r * gx / den, p[1] + r * gy / den, 0.];
        q[2] = t[0][2] + gx * (q[0] - t[0][0]) + gy * (q[1] - t[0][1]);
        if inside(q, t) {
            candidate(q);
        }
    }
    best.is_finite().then_some(best)
}
struct Grid<'a> {
    mesh: &'a [[V; 3]],
    cells: HashMap<(i32, i32), Vec<usize>>,
    bounds: Vec<Bounds4>,
    size: f64,
}
impl<'a> Grid<'a> {
    fn new(mesh: &'a [[V; 3]], size: f64, cancel: &CancelToken) -> Result<Self, &'static str> {
        let mut out = Self {
            mesh,
            cells: HashMap::new(),
            bounds: Vec::new(),
            size,
        };
        let mut bounds = Vec::with_capacity(mesh.len());
        let mut entries = 0u64;
        for (i, t) in mesh.iter().enumerate() {
            if i % 1024 == 0 && cancel.is_cancelled() {
                return Err("Parallel cancelled");
            }
            let lo = [0, 1].map(|a| t.iter().map(|p| p[a]).fold(f64::INFINITY, f64::min));
            let hi = [0, 1].map(|a| t.iter().map(|p| p[a]).fold(f64::NEG_INFINITY, f64::max));
            bounds.push((lo, hi));
            let width = (i64::from(out.key(hi[0])) - i64::from(out.key(lo[0])) + 1) as u64;
            let height = (i64::from(out.key(hi[1])) - i64::from(out.key(lo[1])) + 1) as u64;
            entries = entries.saturating_add(width.saturating_mul(height));
            if entries > 4_000_000 {
                return Err("Parallel CAD spatial index exceeds its memory budget; use a larger cutter or smaller model");
            }
            for x in out.key(lo[0])..=out.key(hi[0]) {
                for y in out.key(lo[1])..=out.key(hi[1]) {
                    out.cells.entry((x, y)).or_default().push(i);
                }
            }
        }
        out.bounds = pack_bounds(&bounds);
        Ok(out)
    }
    fn key(&self, x: f64) -> i32 {
        (x / self.size).floor() as i32
    }
    fn indices(&self, p: V, r: f64) -> Vec<usize> {
        let mut ids = Vec::new();
        for x in self.key(p[0] - r)..=self.key(p[0] + r) {
            for y in self.key(p[1] - r)..=self.key(p[1] + r) {
                if let Some(v) = self.cells.get(&(x, y)) {
                    ids.extend_from_slice(v);
                }
            }
        }
        ids.sort_unstable();
        ids.dedup();
        filter_bounds(&mut ids, &self.bounds, p, r);
        ids
    }
    fn height(&self, p: V, r: f64) -> Option<f64> {
        self.indices(p, r)
            .into_iter()
            .filter_map(|i| drop_ball(p, &self.mesh[i], r))
            .max_by(f64::total_cmp)
    }
    fn contact(&self, p: V) -> bool {
        self.indices(p, 0.).into_iter().any(|i| {
            let t = &self.mesh[i];
            let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
            n[2].abs() > 1e-12 && inside(p, t)
        })
    }

    /// Cover the projected segment with exact triangle intervals. Even a narrow
    /// gap between two sampled rows must break a local linking chain.
    fn covers_segment(&self, a: V, b: V, cancel: &CancelToken) -> bool {
        self.covers_segment_where(a, b, |_| true, cancel)
    }

    fn covers_segment_where(
        &self,
        a: V,
        b: V,
        allowed: impl Fn(&[V; 3]) -> bool,
        cancel: &CancelToken,
    ) -> bool {
        let mid = mul(add(a, b), 0.5);
        let half_xy = (a[0] - b[0]).hypot(a[1] - b[1]) * 0.5;
        let mut intervals = Vec::new();
        for id in self.indices(mid, half_xy) {
            if cancel.is_cancelled() {
                return false;
            }
            let triangle = &self.mesh[id];
            let normal = cross(sub(triangle[1], triangle[0]), sub(triangle[2], triangle[0]))[2];
            if normal.abs() < 1e-12 || !allowed(triangle) {
                continue;
            }
            let mut lo: f64 = 0.;
            let mut hi: f64 = 1.;
            for i in 0..3 {
                let p = triangle[i];
                let edge = sub(triangle[(i + 1) % 3], p);
                let side =
                    |q: V| normal.signum() * (edge[0] * (q[1] - p[1]) - edge[1] * (q[0] - p[0]));
                let (fa, fb) = (side(a), side(b));
                if fa < 0. && fb < 0. {
                    hi = -1.;
                    break;
                }
                if fa < 0. {
                    lo = lo.max(fa / (fa - fb));
                }
                if fb < 0. {
                    hi = hi.min(fa / (fa - fb));
                }
            }
            if lo <= hi {
                intervals.push((lo, hi));
            }
        }
        intervals.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
        let mut covered: f64 = 0.;
        for (lo, hi) in intervals {
            if lo > covered + 1e-10 {
                return false;
            }
            covered = covered.max(hi);
            if covered >= 1. - 1e-10 {
                return true;
            }
        }
        false
    }
}

/// Four independent XY bounding-box tests at a time, using double precision.
/// `wide` uses two NEON vectors on Apple Silicon and baseline SSE2 on x86-64;
/// other targets retain its portable fallback. Exact cutter tests still follow.
struct Bounds4 {
    lx: f64x4,
    ly: f64x4,
    hx: f64x4,
    hy: f64x4,
}
fn pack_bounds(bounds: &[([f64; 2], [f64; 2])]) -> Vec<Bounds4> {
    bounds
        .chunks(4)
        .map(|batch| {
            let lane = |i: usize| batch.get(i).copied().unwrap_or(([0.; 2], [0.; 2]));
            Bounds4 {
                lx: f64x4::new(std::array::from_fn(|i| lane(i).0[0])),
                ly: f64x4::new(std::array::from_fn(|i| lane(i).0[1])),
                hx: f64x4::new(std::array::from_fn(|i| lane(i).1[0])),
                hy: f64x4::new(std::array::from_fn(|i| lane(i).1[1])),
            }
        })
        .collect()
}
fn filter_bounds(ids: &mut Vec<usize>, bounds: &[Bounds4], p: V, r: f64) {
    let px = f64x4::splat(p[0]);
    let py = f64x4::splat(p[1]);
    let limit = f64x4::splat(r * r + 1e-10);
    let mut written = 0;
    let mut offset = 0;
    while offset < ids.len() {
        let first = offset;
        let block = ids[first] / 4;
        // Sorted candidate IDs commonly share a packed block. Load its bounds
        // once instead of gathering sixteen scalar coordinates for each query.
        let b = &bounds[block];
        let dx = (b.lx - px).max(px - b.hx).max(f64x4::ZERO);
        let dy = (b.ly - py).max(py - b.hy).max(f64x4::ZERO);
        let mask = (dx * dx + dy * dy).cmp_le(limit).move_mask();
        // The spatial index emits sorted, unique IDs. Most interior blocks are
        // complete, so avoid scanning their group boundaries a second time.
        if ids[first].is_multiple_of(4)
            && first + 4 <= ids.len()
            && ids[first + 3] == ids[first] + 3
        {
            for lane in 0..4 {
                if mask & (1 << lane) != 0 {
                    ids[written] = block * 4 + lane;
                    written += 1;
                }
            }
            offset += 4;
            continue;
        }
        while offset < ids.len() && ids[offset] / 4 == block {
            offset += 1;
        }
        for index in first..offset {
            let id = ids[index];
            if mask & (1 << (id % 4)) != 0 {
                ids[written] = id;
                written += 1;
            }
        }
    }
    ids.truncate(written);
}

fn split_row(fragments: &mut Vec<(Vec<V>, bool)>, path: &mut Vec<V>) {
    // Consecutive empty trims have the same linker effect. Keep the first so a
    // gap still retracts, without queuing millions of empty vectors over air.
    if !path.is_empty() || fragments.last().is_none_or(|(last, _)| !last.is_empty()) {
        fragments.push((std::mem::take(path), false));
    }
}
#[derive(Clone, Debug)]
pub struct ParallelStrategy {
    op: ParallelOp,
    stock: StockToLeave,
    geometry: ParallelGeometry,
}
impl ParallelStrategy {
    pub const DOMAIN_REVISION: u32 = 10;
    pub fn new(op: ParallelOp, stock: StockToLeave, geometry: ParallelGeometry) -> Self {
        Self {
            op,
            stock,
            geometry,
        }
    }
}
impl Strategy for ParallelStrategy {
    fn name(&self) -> &str {
        "parallel"
    }
    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        if self.op.parameter_expressions.is_empty() {
            return self.compute_resolved_parameters(env, cancel);
        }
        let mut operation = cam_model::Operation::Parallel(self.op.clone());
        if let Err(message) = operation.resolve_parameter_expressions(env.tools) {
            return StrategyResult {
                diagnostics: vec![crate::Diagnostic::error(message)],
                ..Default::default()
            };
        }
        let cam_model::Operation::Parallel(op) = operation else {
            unreachable!()
        };
        Self { op, ..self.clone() }.compute_resolved_parameters(env, cancel)
    }
}
impl ParallelStrategy {
    fn compute_resolved_parameters(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        let mut out = StrategyResult::default();
        let op = &self.op;
        let p = op.params;
        macro_rules! fail {
            ($s:expr) => {{
                out.program = Default::default();
                out.diagnostics.push(Diagnostic::error($s));
                return out;
            }};
        }
        if cancel.is_cancelled() {
            out.cancelled = true;
            return out;
        }
        if !p.lift_height.is_finite() || p.lift_height < 0. {
            fail!("Parallel lift height must be finite and nonnegative");
        }
        if [p.tolerance, p.stepover, p.feed, p.plunge_feed]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.)
            || !p.angle.is_finite()
            || !p.additional_offset.is_finite()
        {
            fail!("Parallel tolerance, stepover and feeds must be finite and positive; angle and offset must be finite");
        }
        let angle = if p.scallop { 0. } else { p.angle.to_radians() };
        let projection = match SurfaceProjection::new(
            "Parallel",
            &self.geometry,
            self.stock,
            env,
            SurfaceOptions {
                tool: op.tool,
                entry_feeds: op.entry_feeds,
                tolerance: p.tolerance,
                containment: p.containment,
                additional_offset: p.additional_offset,
                angle,
            },
            cancel,
        ) {
            Ok(projection) => projection,
            Err(_) if cancel.is_cancelled() => {
                out.cancelled = true;
                return out;
            }
            Err(reason) => fail!(reason),
        };
        let [lo, hi] = projection.bounds;
        let r = projection.radius;
        let error = projection.error;
        let contact_radius = r + error;
        let lift = error + p.tolerance + projection.axial;
        let step = (r * p.tolerance).sqrt().clamp(0.01, 0.5);
        let (s, c) = angle.sin_cos();
        let world = |u: f64, v: f64| [c * u - s * v, s * u + c * v, 0.];
        let rows = ((hi[1] - lo[1]) / p.stepover).ceil() as usize;
        let samples = ((hi[0] - lo[0]) / step).ceil() as usize;
        if !p.scallop
            && (rows > 100_000 || samples > 1_000_000 || rows.saturating_mul(samples) > 5_000_000)
        {
            fail!(
                "Parallel exceeds the five-million sample budget; increase stepover or tolerance"
            );
        }
        let mut linking = projection.linker();
        let grid = linking.grid;
        let selected = linking.selected;
        if p.scallop {
            return scallop::compute(grid, selected, &mut linking, op, lo, hi, cancel);
        }
        // Bound queued work by both row count and nominal samples. Indexed
        // collection keeps row order independent of worker scheduling.
        let batch_rows = (16_384 / (samples + 1)).clamp(1, 32);
        let sample_row = |row: usize| -> Option<Vec<(Vec<V>, bool)>> {
            if cancel.is_cancelled() {
                return None;
            }
            let bottom = self.geometry.heights.bottom;
            let top = self.geometry.heights.top;
            let mut fragments = Vec::new();
            let v = lo[1] + (row as f64 * p.stepover).min(hi[1] - lo[1]);
            let mut path: Vec<V> = Vec::new();
            let mut continuous = true;
            for j in 0..=samples {
                if j % 256 == 0 && cancel.is_cancelled() {
                    return None;
                }
                let k = if p.both_ways && row % 2 == 1 {
                    samples - j
                } else {
                    j
                };
                let mut q = world(lo[0] + (hi[0] - lo[0]) * k as f64 / samples as f64, v);
                // Contact-only trimming prevents cutting across openings or
                // separate selected faces. Every disconnected run retracts.
                if !selected.contact(q) && selected.height(q, r).is_none() {
                    split_row(&mut fragments, &mut path);
                    continuous = false;
                    continue;
                }
                let Some(z) = grid.height(q, contact_radius) else {
                    split_row(&mut fragments, &mut path);
                    continuous = false;
                    continue;
                };
                q[2] = z + lift;
                // Protect the cylindrical flute above the ball: no triangle
                // within its XY disk may reach above the sphere centre. Tall
                // walls are handled by lifting the cutter conservatively.
                let ids = grid.indices(q, contact_radius);
                let wall = ids
                    .iter()
                    .filter_map(|i| flat_height(q, &grid.mesh[*i], contact_radius))
                    .fold(f64::NEG_INFINITY, f64::max);
                q[2] = q[2].max(wall - r + lift).max(bottom);
                if q[2] > top + lift {
                    split_row(&mut fragments, &mut path);
                    continuous = false;
                    continue;
                }
                if let Some(prev) = path.last().copied() {
                    let previous_length = path.len();
                    if !refine(
                        grid,
                        prev,
                        q,
                        r,
                        error,
                        lift,
                        p.tolerance,
                        bottom,
                        0,
                        &mut path,
                        cancel,
                    ) {
                        if cancel.is_cancelled() {
                            return None;
                        }
                        // A vertical contact discontinuity cannot be followed
                        // continuously. Discard the attempted connector and
                        // restart from clearance rather than cutting a shortcut.
                        path.truncate(previous_length);
                        split_row(&mut fragments, &mut path);
                        continuous = false;
                        path.push(q);
                    }
                } else {
                    path.push(q);
                }
            }
            fragments.push((path, continuous));
            Some(fragments)
        };
        for first in (0..=rows).step_by(batch_rows) {
            let end = (first + batch_rows).min(rows + 1);
            let batch: Option<Vec<_>> = (first..end).into_par_iter().map(&sample_row).collect();
            let Some(batch) = batch else {
                out.cancelled = true;
                return out;
            };
            for fragments in batch {
                for (mut path, continuous) in fragments {
                    if cancel.is_cancelled() {
                        out.cancelled = true;
                        return out;
                    }
                    linking.emit(&mut out, &mut path, op, continuous, cancel);
                }
                if out.program.steps().len() > 2_000_000 {
                    fail!("Parallel exceeds the two-million motion budget; increase tolerance or stepover");
                }
            }
        }
        linking.retract(&mut out, op.id);
        if cancel.is_cancelled() {
            out.cancelled = true;
            return out;
        }
        let passes = linking.passes;
        out.diagnostics.push(Diagnostic::info(format!(
            "3D Parallel: {passes} passes at {:.3} mm stepover",
            p.stepover
        )));
        if passes == 0 {
            fail!("Parallel produced no surface contact passes");
        }
        out
    }
}
fn containment_inset(containment: AdaptiveContainment, offset: f64, radius: f64) -> f64 {
    match containment {
        AdaptiveContainment::Inside => radius + offset,
        AdaptiveContainment::Center => -offset,
        AdaptiveContainment::Outside => -radius - offset,
    }
}

struct SurfaceOptions {
    tool: u32,
    entry_feeds: cam_model::EntryFeeds,
    tolerance: f64,
    containment: AdaptiveContainment,
    additional_offset: f64,
    angle: f64,
}

struct SurfaceProjection<'mesh> {
    grid: Grid<'mesh>,
    selected: Grid<'mesh>,
    bounds: [[f64; 2]; 2],
    radius: f64,
    error: f64,
    axial: f64,
    clearance: f64,
    heights: cam_model::CuttingHeights,
}

impl<'mesh> SurfaceProjection<'mesh> {
    fn new(
        name: &str,
        geometry: &'mesh ParallelGeometry,
        stock: StockToLeave,
        env: &JobEnv,
        options: SurfaceOptions,
        cancel: &CancelToken,
    ) -> Result<Self, String> {
        let Some(tool) = env.tool(options.tool) else {
            return Err(format!("{name} tool is missing"));
        };
        if tool.kind != ToolKind::BallMill {
            return Err(format!("3D {name} currently requires a ball nose end mill"));
        }
        if !tool.diameter.is_finite() || tool.diameter <= 0. {
            return Err(format!("{name} tool diameter must be finite and positive"));
        }
        options.entry_feeds.validate()?;
        let (radial, axial) = stock.effective();
        if [radial, axial].iter().any(|v| !v.is_finite() || *v < 0.) {
            return Err(format!(
                "{name} stock allowances must be finite and nonnegative"
            ));
        }
        if geometry.selected.is_empty() || geometry.triangles.is_empty() {
            return Err(format!("{name} has no resolved CAD surface"));
        }
        if geometry
            .triangles
            .iter()
            .flatten()
            .flatten()
            .any(|v| !v.is_finite())
        {
            return Err(format!(
                "{name} CAD projection contains nonfinite coordinates"
            ));
        }
        geometry.heights.validate()?;
        if !geometry.error_bound.is_finite() || geometry.error_bound < 0.0 {
            return Err(format!(
                "{name} CAD error bound must be finite and nonnegative"
            ));
        }
        if [
            tool.length,
            tool.flute_len(),
            tool.shank_dia(),
            tool.neck_dia(),
        ]
        .iter()
        .any(|v| !v.is_finite() || *v < 0.)
            || tool.flute_len() < tool.radius()
        {
            return Err(format!(
                "{name} requires finite tool dimensions and flutes covering the ball nose"
            ));
        }
        let r = tool.radius() + radial;
        let error = geometry.error_bound;
        let lift = error + options.tolerance + axial;
        if !r.is_finite() || !lift.is_finite() {
            return Err(format!("{name} cutter envelope is not finite"));
        }
        let top = geometry
            .triangles
            .iter()
            .flatten()
            .map(|v| v[2])
            .fold(f64::NEG_INFINITY, f64::max)
            + lift;
        let clearance = env.heights.clearance;
        if !clearance.is_finite() || clearance <= top || clearance <= env.heights.top_of_stock {
            return Err(format!(
                "{name} clearance must be above the complete model, allowance and stock"
            ));
        }
        // The spherical tip is checked against every setup body. A larger shaft
        // would require a separate swept-holder model and is rejected explicitly.
        if tool.shank_dia() > tool.diameter + 1e-9 || tool.neck_dia() > tool.diameter + 1e-9 {
            return Err(format!(
                "{name} does not yet support shafts wider than the cutter"
            ));
        }
        let (s, c) = options.angle.sin_cos();
        let local = |v: V| [c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]];
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for v in geometry.selected.iter().flatten().copied().map(local) {
            for a in 0..2 {
                lo[a] = lo[a].min(v[a]);
                hi[a] = hi[a].max(v[a]);
            }
        }
        let inset = containment_inset(options.containment, options.additional_offset, r);
        for a in 0..2 {
            lo[a] += inset;
            hi[a] -= inset;
        }
        if (0..2).any(|a| !lo[a].is_finite() || !hi[a].is_finite() || lo[a] >= hi[a]) {
            return Err(format!("{name} containment leaves no machining area"));
        }
        let grid_size = r.max(0.1) * 2.;
        let build_grid =
            |mesh| Grid::new(mesh, grid_size, cancel).map_err(|e| e.replace("Parallel", name));
        Ok(Self {
            grid: build_grid(&geometry.triangles)?,
            selected: build_grid(&geometry.selected)?,
            bounds: [lo, hi],
            radius: r,
            error,
            axial,
            clearance,
            heights: geometry.heights,
        })
    }

    fn linker(&self) -> PassLinker<'_, 'mesh> {
        PassLinker {
            grid: &self.grid,
            selected: &self.selected,
            radius: self.radius,
            error: self.error,
            axial: self.axial,
            clearance: self.clearance,
            bottom: self.heights.bottom,
            top: self.heights.top,
            previous: None,
            passes: 0,
        }
    }
}

struct PassLinker<'a, 'mesh> {
    grid: &'a Grid<'mesh>,
    selected: &'a Grid<'mesh>,
    radius: f64,
    error: f64,
    axial: f64,
    clearance: f64,
    bottom: f64,
    top: f64,
    // Delay the retract until the next complete row can be checked.
    previous: Option<V>,
    passes: usize,
}
impl PassLinker<'_, '_> {
    /// Check the complete swept ball and cylindrical envelope, including the
    /// lift and descent. Bounding the whole XY segment also catches narrow
    /// obstacles between endpoints; sampling alone cannot certify a link.
    fn segment_clear(&self, a: V, b: V, cancel: &CancelToken) -> bool {
        self.segment_clear_extra(a, b, 0., cancel)
    }

    fn segment_clear_extra(&self, a: V, b: V, extra: f64, cancel: &CancelToken) -> bool {
        let mid = mul(add(a, b), 0.5);
        let reach = self.radius + self.error + extra;
        let half_xy = (a[0] - b[0]).hypot(a[1] - b[1]) * 0.5;
        // Shifting the cutter down by the axial allowance is equivalent to
        // raising the protected surface, preserving the requested floor skin.
        let ca = add(a, [0., 0., self.radius - self.axial]);
        let cb = add(b, [0., 0., self.radius - self.axial]);
        self.grid
            .indices(mid, reach + half_xy)
            .into_iter()
            .all(|i| {
                if cancel.is_cancelled() {
                    return false;
                }
                let triangle = &self.grid.mesh[i];
                segment_triangle(ca, cb, triangle) >= reach * reach
                    && !flat_height(mid, triangle, reach + half_xy)
                        .is_some_and(|z| z + self.error > ca[2].min(cb[2]))
            })
    }

    fn local_link(&self, from: V, to: V, op: &ParallelOp, cancel: &CancelToken) -> Option<Vec<V>> {
        let distance = (from[0] - to[0]).hypot(from[1] - to[1]);
        // Bound spatial queries even with an arbitrarily large user stepover.
        if distance > (1.5 * op.params.stepover).min(4. * self.radius) {
            return None;
        }
        if !self.selected.covers_segment(from, to, cancel) {
            return None;
        }
        if op.params.lift_height == 0. {
            // A zero-lift turnaround continues the finishing cut. Derive and
            // certify its Z against the surface just like the scan rows, so it
            // can machine the intervening scallop instead of demanding air.
            let margin = self.error + op.params.tolerance + self.axial;
            let mut path = Vec::new();
            if refine(
                self.grid,
                from,
                to,
                self.radius,
                self.error,
                margin,
                op.params.tolerance,
                self.bottom,
                0,
                &mut path,
                cancel,
            ) && path
                .iter()
                .all(|q| q[2] <= self.top + margin && q[2] < self.clearance)
            {
                return Some(path);
            }
            return None;
        }
        let z = (from[2].max(to[2]) + op.params.lift_height).min(self.clearance);
        // Once the requested lift reaches clearance, use the usual rapid link.
        if z >= self.clearance {
            return None;
        }
        let a = [from[0], from[1], z];
        let b = [to[0], to[1], z];
        (self.segment_clear(from, a, cancel)
            && self.segment_clear(a, b, cancel)
            && self.segment_clear(b, to, cancel))
        .then(|| vec![a, b])
    }

    fn retract(&mut self, out: &mut StrategyResult, id: u32) {
        if let Some(p) = self.previous.take() {
            out.program.push(Step::Rapid {
                to: Point3::new(p[0], p[1], self.clearance),
                tag: Tag::new(id, MoveKind::Retract),
            });
        }
    }

    fn emit(
        &mut self,
        out: &mut StrategyResult,
        path: &mut Vec<V>,
        op: &ParallelOp,
        continuous: bool,
        cancel: &CancelToken,
    ) {
        // A trimmed row can still meet the next row on one continuous face.
        // For surface-cutting turns, let the exact selection coverage and
        // surface checks decide continuity instead of retracting on any trim.
        let surface_turn = op.params.stay_down && op.params.lift_height == 0.;
        let continuous = continuous || surface_turn;
        if path.len() > 1 {
            self.passes += 1;
            let p = path[0];
            let local = self
                .previous
                .filter(|_| continuous && op.params.stay_down)
                .and_then(|from| self.local_link(from, p, op, cancel));
            let at_depth = local.is_some() && op.params.lift_height == 0.;
            if let Some(raised) = local {
                for q in raised {
                    if self.previous == Some(q) {
                        continue;
                    }
                    out.program.push(Step::Linear {
                        to: Point3::new(q[0], q[1], q[2]),
                        feed: op.params.feed,
                        tag: Tag::new(
                            op.id,
                            if at_depth {
                                MoveKind::Cutting
                            } else {
                                MoveKind::Link
                            },
                        ),
                    });
                }
                self.previous = None;
            } else {
                self.retract(out, op.id);
                out.program.push(Step::Rapid {
                    to: Point3::new(p[0], p[1], self.clearance),
                    tag: Tag::new(op.id, MoveKind::Link),
                });
            }
            if !at_depth {
                out.program.push(Step::Linear {
                    to: Point3::new(p[0], p[1], p[2]),
                    feed: op.params.plunge_feed,
                    tag: Tag::new(op.id, MoveKind::Plunge),
                });
            }
            for p in path.iter().skip(1) {
                out.program.push(Step::Linear {
                    to: Point3::new(p[0], p[1], p[2]),
                    feed: op.params.feed,
                    tag: Tag::new(op.id, MoveKind::Cutting),
                });
            }
            self.previous = path.last().copied();
            if !continuous || !op.params.stay_down {
                self.retract(out, op.id);
            }
        } else if !surface_turn {
            self.retract(out, op.id);
        }
        path.clear();
    }
}

// Maximum triangle Z inside a disk, for cylindrical flute protection.
fn flat_height(p: V, t: &[V; 3], r: f64) -> Option<f64> {
    let mut best = f64::NEG_INFINITY;
    for i in 0..3 {
        let a = t[i];
        let d = sub(t[(i + 1) % 3], a);
        let l = d[0] * d[0] + d[1] * d[1];
        if l < 1e-20 {
            if (a[0] - p[0]).hypot(a[1] - p[1]) <= r {
                best = best.max(a[2]);
            }
            continue;
        }
        let b = (a[0] - p[0]) * d[0] + (a[1] - p[1]) * d[1];
        let cc = (a[0] - p[0]).powi(2) + (a[1] - p[1]).powi(2) - r * r;
        let disc = b * b - l * cc;
        if disc < 0. {
            continue;
        }
        let lo = ((-b - disc.sqrt()) / l).max(0.);
        let hi = ((-b + disc.sqrt()) / l).min(1.);
        if lo <= hi {
            best = best.max(a[2] + d[2] * if d[2] >= 0. { hi } else { lo });
        }
    }
    let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
    if n[2].abs() > 1e-12 {
        let gx = -n[0] / n[2];
        let gy = -n[1] / n[2];
        let len = gx.hypot(gy);
        let mut q = p;
        if len > 1e-12 {
            q[0] += r * gx / len;
            q[1] += r * gy / len;
        }
        if inside(q, t) {
            best = best.max(t[0][2] + gx * (q[0] - t[0][0]) + gy * (q[1] - t[0][1]));
        }
    }
    best.is_finite().then_some(best)
}
// Keep the recursive segment's geometry and safety limits explicit at call sites.
#[allow(clippy::too_many_arguments)]
fn refine(
    grid: &Grid,
    a: V,
    b: V,
    r: f64,
    error: f64,
    lift: f64,
    tol: f64,
    bottom: f64,
    depth: u32,
    path: &mut Vec<V>,
    cancel: &CancelToken,
) -> bool {
    if cancel.is_cancelled() || path.len() > 100_000 {
        return false;
    }
    let mut mid = mul(add(a, b), 0.5);
    let z = grid.height(mid, r + error).unwrap_or(f64::NEG_INFINITY);
    let wall = grid
        .indices(mid, r + error)
        .into_iter()
        .filter_map(|i| flat_height(mid, &grid.mesh[i], r + error))
        .fold(f64::NEG_INFINITY, f64::max);
    let required = (z + lift).max(wall - r + lift).max(bottom);
    let ca = add(a, [0., 0., r]);
    let cb = add(b, [0., 0., r]);
    let half_xy = (a[0] - b[0]).hypot(a[1] - b[1]) * 0.5;
    let collision = grid.indices(mid, r + error + half_xy).into_iter().any(|i| {
        let t = &grid.mesh[i];
        segment_triangle(ca, cb, t) < (r + error).powi(2)
            || flat_height(mid, t, r + error + half_xy)
                .is_some_and(|z| z + error > ca[2].min(cb[2]))
    });
    let deviation = if z.is_finite() || wall.is_finite() {
        (required - mid[2]).abs()
    } else {
        0.
    };
    if deviation > tol * 0.25 || collision {
        if depth >= 20 {
            return false;
        }
        mid[2] = if z.is_finite() || wall.is_finite() {
            required
        } else {
            mid[2]
        };
        return refine(
            grid,
            a,
            mid,
            r,
            error,
            lift,
            tol,
            bottom,
            depth + 1,
            path,
            cancel,
        ) && refine(
            grid,
            mid,
            b,
            r,
            error,
            lift,
            tol,
            bottom,
            depth + 1,
            path,
            cancel,
        );
    }
    path.push(b);
    true
}

// Squared distance from a point to a solid triangle (Ericson's Voronoi regions).
fn point_triangle(p: V, t: &[V; 3]) -> f64 {
    let [a, b, c] = *t;
    let ab = sub(b, a);
    let ac = sub(c, a);
    let n = cross(ab, ac);
    if dot(n, n) < 1e-24 {
        return (0..3)
            .map(|i| segment_distance(p, p, t[i], t[(i + 1) % 3]))
            .fold(f64::INFINITY, f64::min);
    }
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0. && d2 <= 0. {
        return dot(ap, ap);
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0. && d4 <= d3 {
        return dot(bp, bp);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0. && d1 >= 0. && d3 <= 0. {
        let q = sub(p, add(a, mul(ab, d1 / (d1 - d3))));
        return dot(q, q);
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0. && d5 <= d6 {
        return dot(cp, cp);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0. && d2 >= 0. && d6 <= 0. {
        let q = sub(p, add(a, mul(ac, d2 / (d2 - d6))));
        return dot(q, q);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0. {
        let q = sub(
            p,
            add(b, mul(sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6)))),
        );
        return dot(q, q);
    }
    let den = va + vb + vc;
    if den.abs() < 1e-25 {
        return (0..3)
            .map(|i| segment_distance(p, p, t[i], t[(i + 1) % 3]))
            .fold(f64::INFINITY, f64::min);
    }
    let q = sub(p, add(a, add(mul(ab, vb / den), mul(ac, vc / den))));
    dot(q, q)
}
fn segment_distance(p: V, q: V, a: V, b: V) -> f64 {
    let d1 = sub(q, p);
    let d2 = sub(b, a);
    let r = sub(p, a);
    let aa = dot(d1, d1);
    let ee = dot(d2, d2);
    let f = dot(d2, r);
    let (mut s, mut t);
    if aa <= 1e-24 && ee <= 1e-24 {
        return dot(r, r);
    }
    if aa <= 1e-24 {
        s = 0.;
        t = (f / ee).clamp(0., 1.);
    } else {
        let c = dot(d1, r);
        if ee <= 1e-24 {
            t = 0.;
            s = (-c / aa).clamp(0., 1.);
        } else {
            let bb = dot(d1, d2);
            let den = aa * ee - bb * bb;
            s = if den.abs() > 1e-24 {
                ((bb * f - c * ee) / den).clamp(0., 1.)
            } else {
                0.
            };
            t = (bb * s + f) / ee;
            if t < 0. {
                t = 0.;
                s = (-c / aa).clamp(0., 1.);
            } else if t > 1. {
                t = 1.;
                s = ((bb - c) / aa).clamp(0., 1.);
            }
        }
    }
    let delta = sub(add(p, mul(d1, s)), add(a, mul(d2, t)));
    dot(delta, delta)
}
fn segment_triangle(a: V, b: V, t: &[V; 3]) -> f64 {
    let d = sub(b, a);
    let e1 = sub(t[1], t[0]);
    let e2 = sub(t[2], t[0]);
    let h = cross(d, e2);
    let det = dot(e1, h);
    if det.abs() > 1e-16 {
        let s = sub(a, t[0]);
        let u = dot(s, h) / det;
        let q = cross(s, e1);
        let v = dot(d, q) / det;
        let along = dot(e2, q) / det;
        if u >= 0. && v >= 0. && u + v <= 1. && (0.0..=1.0).contains(&along) {
            return 0.;
        }
    }
    let mut best = point_triangle(a, t).min(point_triangle(b, t));
    for i in 0..3 {
        best = best.min(segment_distance(a, b, t[i], t[(i + 1) % 3]));
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_cldata::SpindleDir;
    use cam_model::{CuttingHeights, Heights, ParallelParams, Tool};
    fn surface() -> ParallelGeometry {
        let triangles = vec![
            [[0., 0., 0.], [10., 0., 2.], [10., 10., 2.]],
            [[0., 0., 0.], [10., 10., 2.], [0., 10., 0.]],
        ];
        ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 3.,
                bottom: 0.,
            },
        }
    }
    fn op() -> ParallelOp {
        ParallelOp {
            parameter_expressions: Default::default(),
            id: 7,
            tool: 1,
            work_offset: 1,
            spindle_rpm: 1000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            params: ParallelParams {
                stepover: 1.,
                ..Default::default()
            },
        }
    }
    fn run(operation: ParallelOp, geometry: ParallelGeometry) -> StrategyResult {
        run_with_stock(operation, geometry, StockToLeave::default())
    }
    fn run_with_stock(
        operation: ParallelOp,
        geometry: ParallelGeometry,
        stock: StockToLeave,
    ) -> StrategyResult {
        let tools = [Tool {
            kind: ToolKind::BallMill,
            diameter: 2.,
            number: 1,
            length: 20.,
            ..Default::default()
        }];
        let env = JobEnv {
            heights: Heights::new(10., 5., 3.),
            tools: &tools,
            stock: None,
            spindle: SpindleDir::Cw,
        };
        ParallelStrategy::new(operation, stock, geometry).compute(&env, &CancelToken::new())
    }
    fn local_links(result: &StrategyResult) -> usize {
        result
            .program
            .steps()
            .iter()
            .filter(|s| {
                matches!(s,
            Step::Linear { tag, .. } if tag.kind == MoveKind::Link)
            })
            .count()
    }
    fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> [[V; 3]; 2] {
        [
            [[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.]],
            [[x0, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
        ]
    }

    #[test]
    fn simd_bounds_match_scalar_including_tails_and_grazing_contact() {
        let bounds: Vec<_> = (0..257)
            .map(|i| {
                let x = (i % 17) as f64 * 0.375 - 3.;
                let y = (i / 17) as f64 * 0.25 - 2.;
                ([x, y], [x + (i % 3) as f64, y + (i % 5) as f64 * 0.1])
            })
            .collect();
        let packed = pack_bounds(&bounds);
        for len in [0, 1, 3, 4, 5, 127, 256, 257] {
            for reverse in [false, true] {
                for stride in [1, 2, 3, 7] {
                    for p in [[0.; 3], [-3., -2., 0.], [1e10, -1e10, 0.]] {
                        for r in [0., 1e-8, 0.375, 1., 1e10] {
                            let mut actual: Vec<_> = (0..len).step_by(stride).collect();
                            if reverse {
                                actual.reverse();
                            }
                            let mut expected = actual.clone();
                            expected.retain(|&i| {
                                let (lo, hi) = bounds[i];
                                let dx = (lo[0] - p[0]).max(p[0] - hi[0]).max(0.);
                                let dy = (lo[1] - p[1]).max(p[1] - hi[1]).max(0.);
                                dx * dx + dy * dy <= r * r + 1e-10
                            });
                            filter_bounds(&mut actual, &packed, p, r);
                            assert_eq!(actual, expected, "len={len}, p={p:?}, r={r}");
                        }
                    }
                }
            }
        }
    }

    #[test]
    #[ignore = "manual SIMD throughput measurement"]
    fn bounds_filter_benchmark() {
        use std::{hint::black_box, time::Instant};
        let bounds: Vec<_> = (0..8192)
            .map(|i| {
                let x = (i % 128) as f64 * 0.1;
                let y = (i / 128) as f64 * 0.1;
                ([x, y], [x + 0.2, y + 0.2])
            })
            .collect();
        let packed = pack_bounds(&bounds);
        let source: Vec<_> = (0..bounds.len()).collect();
        for simd in [false, true] {
            let start = Instant::now();
            for i in 0..5000 {
                let p = black_box([(i % 128) as f64 * 0.1, 3., 0.]);
                let bounds = black_box(bounds.as_slice());
                let mut ids = source.clone();
                if simd {
                    filter_bounds(&mut ids, black_box(&packed), p, 2.);
                } else {
                    ids.retain(|&id| {
                        let (lo, hi) = bounds[id];
                        let dx = (lo[0] - p[0]).max(p[0] - hi[0]).max(0.);
                        let dy = (lo[1] - p[1]).max(p[1] - hi[1]).max(0.);
                        dx * dx + dy * dy <= 4. + 1e-10
                    });
                }
                black_box(ids);
            }
            println!(
                "{}: {:.2} ms",
                if simd { "SIMD" } else { "scalar" },
                start.elapsed().as_secs_f64() * 1000.
            );
        }
    }

    #[test]
    fn finishing_is_identical_with_one_or_four_workers() {
        let serial = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap();
        let parallel = rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .unwrap();
        for scallop in [false, true] {
            for stay_down in [false, true] {
                let mut operation = op();
                operation.params.scallop = scallop;
                operation.params.stay_down = stay_down;
                operation.params.containment = AdaptiveContainment::Center;
                let mut geometry = surface();
                // Separate selections exercise trims, empty fragments and retracts.
                geometry.selected = rectangle(0., 0., 4., 10.)
                    .into_iter()
                    .chain(rectangle(6., 0., 10., 10.))
                    .collect();
                let single = serial.install(|| run(operation.clone(), geometry.clone()));
                let multi = parallel.install(|| run(operation.clone(), geometry.clone()));
                assert!(!single.has_errors(), "{:?}", single.diagnostics);
                assert_eq!(single, multi);
            }
        }
    }

    #[test]
    fn scallop_generates_closed_surface_following_offsets() {
        let mut operation = op();
        operation.params.scallop = true;
        let result = run(operation, surface());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(result.program.validate().is_ok());
        let cuts: Vec<_> = result
            .program
            .steps()
            .iter()
            .filter_map(|step| match step {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(*to),
                _ => None,
            })
            .collect();
        assert!(cuts.len() > 20);
        let min_z = cuts.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
        let max_z = cuts.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_z - min_z > 0.5,
            "offsets must follow the sloping surface"
        );
        assert!(
            cuts.windows(2)
                .any(|w| (w[0].x - w[1].x).abs() > 0.01 && (w[0].y - w[1].y).abs() > 0.01),
            "contours are not parallel scan rows"
        );
    }

    #[test]
    fn scallop_offsets_a_flat_rectangle_inward_at_the_requested_spacing() {
        let triangles = rectangle(0., 0., 10., 10.).to_vec();
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -1.,
            },
        };
        let mut operation = op();
        operation.params.scallop = true;
        operation.params.containment = AdaptiveContainment::Center;
        let result = run(operation, geometry);
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let mut minima = Vec::new();
        let mut current = f64::INFINITY;
        for step in result.program.steps() {
            match step {
                Step::Linear { to, tag, .. }
                    if matches!(tag.kind, MoveKind::Plunge | MoveKind::Cutting) =>
                {
                    current = current.min(to.x)
                }
                Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract && current.is_finite() => {
                    minima.push(current);
                    current = f64::INFINITY;
                }
                _ => {}
            }
        }
        assert!(minima.len() >= 4, "{minima:?}");
        for (index, x) in minima.iter().take(4).enumerate() {
            assert!((*x - (index as f64 + 0.5)).abs() < 0.05, "{minima:?}");
        }
    }

    #[test]
    fn scallop_annulus_progresses_from_outer_edge_without_starting_at_the_hole() {
        let mut triangles = Vec::new();
        for (x0, y0, x1, y1) in [
            (0., 0., 20., 6.),
            (0., 14., 20., 20.),
            (0., 6., 6., 14.),
            (14., 6., 20., 14.),
        ] {
            triangles.extend(rectangle(x0, y0, x1, y1));
        }
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -1.,
            },
        };
        for order in [
            cam_model::ScallopOrder::OutsideIn,
            cam_model::ScallopOrder::Shortest,
        ] {
            let mut operation = op();
            operation.params.scallop = true;
            operation.params.scallop_order = order;
            operation.params.containment = AdaptiveContainment::Center;
            let result = run(operation, geometry.clone());
            assert!(!result.has_errors(), "{order:?}: {:?}", result.diagnostics);
            assert!(result.program.validate().is_ok());
            let mut contours = Vec::<Vec<Point3>>::new();
            for step in result.program.steps() {
                match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => {
                        contours.push(vec![*to])
                    }
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                        contours.last_mut().unwrap().push(*to)
                    }
                    _ => {}
                }
            }
            let offsets: Vec<f64> = contours
                .iter()
                .map(|points| {
                    points
                        .iter()
                        .map(|p| p.x.min(p.y).min(20. - p.x).min(20. - p.y))
                        .fold(f64::INFINITY, f64::min)
                })
                .collect();
            assert!(offsets.len() >= 5, "{order:?}: {offsets:?}");
            assert!(
                offsets[0] < 1.,
                "{order:?}: first pass must be at the outer edge: {offsets:?}"
            );
            for pair in offsets.windows(2) {
                assert!(
                    pair[1] + 0.1 >= pair[0],
                    "{order:?}: passes moved backwards across the ring: {offsets:?}"
                );
                assert!(
                    pair[1] - pair[0] <= 1.2,
                    "{order:?}: skipped an intervening pass: {offsets:?}"
                );
            }
        }
    }

    #[test]
    fn scallop_circular_sloped_ring_advances_towards_the_hole() {
        let mut triangles = Vec::new();
        for i in 0..96 {
            let point = |index: usize, radius: f64, z: f64| {
                let angle = index as f64 * std::f64::consts::TAU / 96.;
                [radius * angle.cos(), radius * angle.sin(), z]
            };
            let outer_a = point(i, 10., 0.);
            let outer_b = point(i + 1, 10., 0.);
            let inner_a = point(i, 4., -2.);
            let inner_b = point(i + 1, 4., -2.);
            triangles.extend([[outer_a, outer_b, inner_b], [outer_a, inner_b, inner_a]]);
        }
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -3.,
            },
        };
        for order in [
            cam_model::ScallopOrder::OutsideIn,
            cam_model::ScallopOrder::Shortest,
        ] {
            let mut operation = op();
            operation.params.scallop = true;
            operation.params.scallop_order = order;
            operation.params.containment = AdaptiveContainment::Center;
            let result = run(operation, geometry.clone());
            assert!(!result.has_errors(), "{order:?}: {:?}", result.diagnostics);
            assert!(result.program.validate().is_ok());
            let mut contours = Vec::<Vec<Point3>>::new();
            for step in result.program.steps() {
                match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => {
                        contours.push(vec![*to])
                    }
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                        contours.last_mut().unwrap().push(*to)
                    }
                    _ => {}
                }
            }
            let radii: Vec<f64> = contours
                .iter()
                .map(|path| path.iter().map(|p| p.x.hypot(p.y)).sum::<f64>() / path.len() as f64)
                .collect();
            assert!(radii.len() >= 5, "{order:?}: {radii:?}");
            assert!(
                radii[0] > 9.,
                "{order:?}: first pass should start outside: {radii:?}"
            );
            assert!(
                radii.last().unwrap() < &6.,
                "{order:?}: final passes should reach the inner edge: {radii:?}"
            );
            for pair in radii.windows(2) {
                assert!(
                    pair[1] <= pair[0] + 0.4,
                    "{order:?}: path returned toward the outside: {radii:?}"
                );
                assert!(
                    pair[0] - pair[1] < 1.5,
                    "{order:?}: path skipped a middle pass: {radii:?}"
                );
            }
        }
    }

    #[test]
    fn scallop_rounded_bore_does_not_finish_with_tiny_disconnected_fragments() {
        let angular = 96;
        let radial = 24;
        let mut selected = Vec::new();
        let point = |i: usize, j: usize| {
            let theta = i as f64 * std::f64::consts::TAU / angular as f64;
            let phi = j as f64 * std::f64::consts::FRAC_PI_2 / radial as f64;
            let radius = 19. - 4. * phi.sin();
            [
                radius * theta.cos(),
                radius * theta.sin(),
                16. + 4. * phi.cos(),
            ]
        };
        for i in 0..angular {
            for j in 0..radial {
                let a = point(i, j);
                let b = point(i + 1, j);
                let c = point(i + 1, j + 1);
                let d = point(i, j + 1);
                selected.extend([[a, b, c], [a, c, d]]);
            }
        }
        let geometry = ParallelGeometry {
            triangles: selected.clone(),
            selected,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 21.,
                bottom: 16.,
            },
        };
        let tools = [Tool {
            kind: ToolKind::BallMill,
            diameter: 6.,
            number: 1,
            length: 20.,
            ..Default::default()
        }];
        let env = JobEnv {
            heights: Heights::new(30., 25., 21.),
            tools: &tools,
            stock: None,
            spindle: SpindleDir::Cw,
        };
        let mut operation = op();
        operation.params.scallop = true;
        operation.params.stepover = 0.5;
        operation.params.tolerance = 0.1;
        operation.params.containment = AdaptiveContainment::Center;
        operation.params.scallop_smoothing = true;
        operation.params.scallop_smoothing_tolerance = 0.05;
        operation.params.scallop_retraction = cam_model::ScallopRetraction::Minimum;
        for order in [
            cam_model::ScallopOrder::OutsideIn,
            cam_model::ScallopOrder::Shortest,
            cam_model::ScallopOrder::InsideOut,
        ] {
            operation.params.scallop_order = order;
            let result =
                ParallelStrategy::new(operation.clone(), StockToLeave::default(), geometry.clone())
                    .compute(&env, &CancelToken::new());
            assert!(!result.has_errors(), "{order:?}: {:?}", result.diagnostics);
            assert!(result.program.validate().is_ok());
            let mut lengths = Vec::new();
            let mut last: Option<Point3> = None;
            for step in result.program.steps() {
                match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => {
                        lengths.push(0.);
                        last = Some(*to);
                    }
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                        let from = last.unwrap();
                        *lengths.last_mut().unwrap() += (to.x - from.x).hypot(to.y - from.y);
                        last = Some(*to);
                    }
                    Step::Arc {
                        end,
                        center,
                        dir,
                        tag,
                        ..
                    } if tag.kind == MoveKind::Cutting => {
                        let from = last.unwrap();
                        *lengths.last_mut().unwrap() += (from.x - center.x)
                            .hypot(from.y - center.y)
                            * cam_cldata::arc_sweep(from, *end, *center, *dir);
                        last = Some(*end);
                    }
                    _ => {}
                }
            }
            assert!(lengths.len() >= 5, "{order:?}: {lengths:?}");
            assert!(
                lengths.iter().all(|length| *length >= 1.),
                "{order:?}: isolated fragments at the bore edge: {lengths:?}"
            );
        }
    }

    #[test]
    fn scallop_preserves_offsets_in_a_wide_region_next_to_an_eccentric_hole() {
        let mut triangles = Vec::new();
        for (x0, y0, x1, y1) in [
            (0., 0., 20., 6.),
            (0., 14., 20., 20.),
            (0., 6., 3., 14.),
            (9., 6., 20., 14.),
        ] {
            triangles.extend(rectangle(x0, y0, x1, y1));
        }
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -1.,
            },
        };
        let mut operation = op();
        operation.params.scallop = true;
        operation.params.containment = AdaptiveContainment::Center;
        let result = run(operation, geometry);
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let max_offset = result
            .program
            .steps()
            .iter()
            .filter_map(|step| match step {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                    Some(to.x.min(to.y).min(20. - to.x).min(20. - to.y))
                }
                _ => None,
            })
            .fold(0., f64::max);
        assert!(
            max_offset > 4.5,
            "wide region lost its deeper offsets: {max_offset}"
        );
    }

    #[test]
    fn scallop_fixed_directions_reverse_contour_order_without_changing_cuts() {
        let triangles = rectangle(0., 0., 10., 10.).to_vec();
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -1.,
            },
        };
        let mut operation = op();
        operation.params.scallop = true;
        operation.params.containment = AdaptiveContainment::Center;
        let outside = run(operation.clone(), geometry.clone());
        operation.params.scallop_order = cam_model::ScallopOrder::InsideOut;
        let inside = run(operation, geometry);
        assert!(
            !outside.has_errors() && !inside.has_errors(),
            "{:?} {:?}",
            outside.diagnostics,
            inside.diagnostics
        );
        let contours = |result: &StrategyResult| {
            let mut all = Vec::<Vec<[f64; 3]>>::new();
            for step in result.program.steps() {
                match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => {
                        all.push(vec![[to.x, to.y, to.z]])
                    }
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                        all.last_mut().unwrap().push([to.x, to.y, to.z])
                    }
                    _ => {}
                }
            }
            all
        };
        let outer_contours = contours(&outside);
        let mut inner_contours = contours(&inside);
        assert!(outer_contours.len() >= 4);
        inner_contours.reverse();
        assert_eq!(
            outer_contours, inner_contours,
            "ordering must preserve the certified cuts"
        );
        assert!(inside.program.validate().is_ok());
    }

    #[test]
    fn scallop_shortest_order_reduces_repositioning_between_separate_regions() {
        let mut triangles = rectangle(0., 0., 4., 8.).to_vec();
        triangles.extend(rectangle(8., 0., 12., 8.));
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -1.,
            },
        };
        let mut operation = op();
        operation.params.scallop = true;
        operation.params.containment = AdaptiveContainment::Center;
        let outside = run(operation.clone(), geometry.clone());
        operation.params.scallop_order = cam_model::ScallopOrder::Shortest;
        let shortest = run(operation, geometry);
        assert!(
            !outside.has_errors() && !shortest.has_errors(),
            "{:?} {:?}",
            outside.diagnostics,
            shortest.diagnostics
        );
        let travel = |result: &StrategyResult| {
            let mut last_retract: Option<Point3> = None;
            let mut sum = 0.;
            for step in result.program.steps() {
                match step {
                    Step::Rapid { to, tag } if tag.kind == MoveKind::Retract => {
                        last_retract = Some(*to)
                    }
                    Step::Rapid { to, tag } if tag.kind == MoveKind::Link => {
                        if let Some(from) = last_retract.take() {
                            sum += (to.x - from.x).hypot(to.y - from.y);
                        }
                    }
                    _ => {}
                }
            }
            sum
        };
        assert!(
            travel(&shortest) < travel(&outside),
            "{} >= {}",
            travel(&shortest),
            travel(&outside)
        );
        assert!(shortest.program.validate().is_ok());
        let cutting_count = |result: &StrategyResult| {
            result.program.steps().iter()
            .filter(|step| matches!(step, Step::Linear { tag, .. } if tag.kind == MoveKind::Cutting)).count()
        };
        assert_eq!(cutting_count(&outside), cutting_count(&shortest));
        let segments = |result: &StrategyResult| {
            let mut previous: Option<[u64; 3]> = None;
            let mut cuts = Vec::new();
            for step in result.program.steps() {
                match step {
                    Step::Linear { to, tag, .. } | Step::Rapid { to, tag } => {
                        let point = [to.x.to_bits(), to.y.to_bits(), to.z.to_bits()];
                        if tag.kind == MoveKind::Cutting {
                            let from = previous.unwrap();
                            cuts.push(if from < point {
                                (from, point)
                            } else {
                                (point, from)
                            });
                        }
                        previous = Some(point);
                    }
                    _ => {}
                }
            }
            cuts.sort_unstable();
            cuts
        };
        assert_eq!(
            segments(&outside),
            segments(&shortest),
            "reordering must preserve every certified cutting segment"
        );
    }

    #[test]
    fn scallop_shortest_order_respects_preferred_entry_positions() {
        let mut triangles = rectangle(0., 0., 4., 8.).to_vec();
        triangles.extend(rectangle(8., 0., 12., 8.));
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -1.,
            },
        };
        let mut operation = op();
        operation.params.scallop = true;
        operation.params.containment = AdaptiveContainment::Center;
        operation.params.scallop_start = Some([2., 7.]);
        let outside = run(operation.clone(), geometry.clone());
        operation.params.scallop_order = cam_model::ScallopOrder::Shortest;
        let shortest = run(operation, geometry);
        assert!(!outside.has_errors() && !shortest.has_errors());
        let positions = |result: &StrategyResult| {
            let mut points: Vec<_> = result
                .program
                .steps()
                .iter()
                .filter_map(|step| match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => {
                        Some([to.x, to.y, to.z])
                    }
                    _ => None,
                })
                .collect();
            points.sort_by(|a, b| {
                a[0].total_cmp(&b[0])
                    .then(a[1].total_cmp(&b[1]))
                    .then(a[2].total_cmp(&b[2]))
            });
            points
        };
        assert_eq!(
            positions(&outside),
            positions(&shortest),
            "ordering must not override a preferred entry"
        );
    }

    #[test]
    fn scallop_retracts_between_disconnected_surface_regions() {
        let mut triangles = rectangle(0., 0., 4., 8.).to_vec();
        triangles.extend(rectangle(6., 0., 10., 8.));
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -1.,
            },
        };
        let mut operation = op();
        operation.params.scallop = true;
        operation.params.stay_down = true;
        operation.params.containment = AdaptiveContainment::Center;
        let result = run(operation, geometry);
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(result.program.validate().is_ok());
        let mut previous = None;
        for step in result.program.steps() {
            match step {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                    if let Some(x) = previous {
                        assert!(
                            !(x < 4. && to.x > 6.) && !(x > 6. && to.x < 4.),
                            "cut crossed the gap"
                        );
                    }
                    previous = Some(to.x);
                }
                Step::Rapid { .. } => previous = None,
                Step::Linear { to, .. } => previous = Some(to.x),
                _ => {}
            }
        }
    }

    #[test]
    fn scallop_checks_the_swept_cutter_against_an_unselected_boss() {
        let mut geometry = surface();
        let boss = [
            [[4., 4., 4.], [6., 4., 4.], [6., 6., 4.]],
            [[4., 4., 4.], [6., 6., 4.], [4., 6., 4.]],
        ];
        geometry.triangles.extend(boss);
        geometry.heights.top = 5.;
        let mut operation = op();
        operation.params.scallop = true;
        for order in [
            cam_model::ScallopOrder::OutsideIn,
            cam_model::ScallopOrder::InsideOut,
            cam_model::ScallopOrder::Shortest,
        ] {
            operation.params.scallop_order = order;
            operation.params.scallop_retraction = cam_model::ScallopRetraction::Shortest;
            let result = run(operation.clone(), geometry.clone());
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
            assert!(result.program.validate().is_ok());
            let mut previous: Option<Point3> = None;
            let mut cuts = 0;
            for step in result.program.steps() {
                match step {
                    Step::Linear { to, tag, .. } => {
                        if matches!(tag.kind, MoveKind::Cutting | MoveKind::Link) {
                            let from = previous.unwrap();
                            for triangle in &boss {
                                assert!(
                                    segment_triangle(
                                        [from.x, from.y, from.z + 1.],
                                        [to.x, to.y, to.z + 1.],
                                        triangle
                                    ) >= 1.0025_f64.powi(2),
                                    "{order:?} {:?}",
                                    tag.kind
                                );
                            }
                            if tag.kind == MoveKind::Cutting {
                                cuts += 1;
                            }
                        }
                        previous = Some(*to);
                    }
                    Step::Rapid { to, .. } => previous = Some(*to),
                    _ => {}
                }
            }
            assert!(cuts > 0);
        }
    }

    #[test]
    fn scallop_preferred_start_rotates_each_closed_contour() {
        let triangles = rectangle(0., 0., 10., 10.).to_vec();
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -1.,
            },
        };
        let mut operation = op();
        operation.params.scallop = true;
        operation.params.scallop_start = Some([9., 9.]);
        operation.params.containment = AdaptiveContainment::Center;
        let result = run(operation, geometry);
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let first = result
            .program
            .steps()
            .iter()
            .find_map(|step| match step {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => Some(*to),
                _ => None,
            })
            .unwrap();
        assert!(
            first.x > 8. || first.y > 8.,
            "entry did not move towards requested corner: {first:?}"
        );
        assert!(
            first.x > 7. && first.y > 7.,
            "entry must be near the requested corner: {first:?}"
        );
    }

    #[test]
    fn scallop_policies_generate_checked_feed_links_or_full_retractions() {
        let triangles = rectangle(0., 0., 10., 10.).to_vec();
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -1.,
            },
        };
        let mut operation = op();
        operation.params.scallop = true;
        operation.params.containment = AdaptiveContainment::Center;
        let full = run(operation.clone(), geometry.clone());
        assert!(!full.has_errors(), "{:?}", full.diagnostics);
        let count_links = |result: &StrategyResult| {
            result
                .program
                .steps()
                .iter()
                .filter(|s| matches!(s, Step::Linear {tag, ..} if tag.kind == MoveKind::Link))
                .count()
        };
        assert_eq!(count_links(&full), 0);
        for policy in [
            cam_model::ScallopRetraction::Minimum,
            cam_model::ScallopRetraction::Shortest,
        ] {
            operation.params.scallop_retraction = policy;
            let linked = run(operation.clone(), geometry.clone());
            assert!(!linked.has_errors(), "{:?}", linked.diagnostics);
            assert!(linked.program.validate().is_ok());
            assert!(count_links(&linked) > 0, "no links for {policy:?}");
            assert!(linked.program.steps().iter().filter(|s| matches!(s, Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract)).count()
                < full.program.steps().iter().filter(|s| matches!(s, Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract)).count());
        }
        operation.params.scallop_retraction = cam_model::ScallopRetraction::Minimum;
        operation.params.scallop_safe_distance = 20.;
        let fallback = run(operation, geometry);
        assert_eq!(
            count_links(&fallback),
            0,
            "clearance exceeding the safe plane must fall back to full retraction"
        );
    }

    #[test]
    fn scallop_smoothing_reduces_actual_cutting_motions() {
        let triangles = rectangle(0., 0., 10., 10.).to_vec();
        let geometry = ParallelGeometry {
            selected: triangles.clone(),
            triangles,
            error_bound: 0.0025,
            heights: CuttingHeights {
                top: 1.,
                bottom: -1.,
            },
        };
        let mut operation = op();
        operation.params.scallop = true;
        operation.params.containment = AdaptiveContainment::Center;
        let raw = run(operation.clone(), geometry.clone());
        operation.params.scallop_smoothing = true;
        operation.params.scallop_smoothing_tolerance = 0.05;
        let smooth = run(operation, geometry);
        assert!(!smooth.has_errors(), "{:?}", smooth.diagnostics);
        assert!(smooth.program.validate().is_ok());
        let cuts = |r: &StrategyResult| {
            r.program
                .steps()
                .iter()
                .filter(|step| {
                    matches!(step,
            Step::Linear { tag, .. } | Step::Arc { tag, .. } if tag.kind == MoveKind::Cutting)
                })
                .count()
        };
        assert!(
            cuts(&smooth) * 2 < cuts(&raw),
            "{} vs {}",
            cuts(&smooth),
            cuts(&raw)
        );
    }

    #[test]
    fn keep_tool_down_is_independent_of_lift_height_and_zero_does_not_lift() {
        for angle in [0., 37.] {
            let mut operation = op();
            operation.params.angle = angle;
            let baseline = run(operation.clone(), surface());
            operation.params.lift_height = 2.;
            let disabled = run(operation.clone(), surface());
            assert_eq!(
                disabled.program, baseline.program,
                "unchecked must retract regardless of lift height"
            );
            operation.params.stay_down = true;
            operation.params.lift_height = 0.;
            let zero = run(operation, surface());
            assert!(!zero.has_errors(), "{:?}", zero.diagnostics);
            assert!(zero.program.validate().is_ok());
            assert_eq!(
                local_links(&zero),
                0,
                "zero-lift transitions continue cutting the surface"
            );
            let cuts = |result: &StrategyResult| {
                result
                    .program
                    .steps()
                    .iter()
                    .filter_map(|s| match s {
                        Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(*to),
                        _ => None,
                    })
                    .collect::<Vec<_>>()
            };
            let zero_cuts = cuts(&zero);
            let mut next = 0;
            for point in cuts(&baseline) {
                next += zero_cuts[next..]
                    .iter()
                    .position(|p| *p == point)
                    .expect("original finishing cuts must be preserved")
                    + 1;
            }
            let mut transitions = 0;
            for pair in zero.program.steps().windows(2) {
                if let [Step::Linear { to: from, .. }, Step::Linear { to, tag, feed }] = pair {
                    if tag.kind == MoveKind::Cutting {
                        let (s, c) = angle.to_radians().sin_cos();
                        if (-s * (to.x - from.x) + c * (to.y - from.y)).abs() > 1e-8 {
                            assert_eq!(*feed, 500.);
                            if angle == 0. {
                                assert_eq!(from.z, to.z);
                            }
                            transitions += 1;
                        }
                    }
                }
            }
            assert!(transitions > 0);
            if angle == 0. {
                assert_eq!(
                    zero.program
                        .steps()
                        .iter()
                        .filter(|s| matches!(s, Step::Rapid { .. }))
                        .count(),
                    2,
                    "only the initial approach and final retract should use clearance"
                );
                assert_eq!(
                    zero.program
                        .steps()
                        .iter()
                        .filter(|s| matches!(s,
                    Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge))
                        .count(),
                    1
                );
                assert!(
                    zero.diagnostics
                        .iter()
                        .any(|d| d.message.contains("9 passes")),
                    "{:?}",
                    zero.diagnostics
                );
            }
        }
    }

    #[test]
    fn zero_lift_turnaround_follows_a_surface_crest_and_machines_the_remaining_stock() {
        let mut geometry = surface();
        geometry.triangles = rectangle(0., 0., 10., 10.).to_vec();
        for (y0, z0, y1, z1) in [(1., 0., 3., 0.4), (3., 0.4, 5., 0.)] {
            geometry.triangles.extend([
                [[0., y0, z0], [10., y0, z0], [10., y1, z1]],
                [[0., y0, z0], [10., y1, z1], [0., y1, z1]],
            ]);
        }
        geometry.selected = geometry.triangles.clone();
        let mut operation = op();
        operation.params.stay_down = true;
        operation.params.lift_height = 0.;
        operation.params.stepover = 4.;
        let result = run(operation, geometry.clone());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(result.program.validate().is_ok());
        assert_eq!(
            result
                .program
                .steps()
                .iter()
                .filter(|s| matches!(s, Step::Rapid { .. }))
                .count(),
            2
        );
        assert!(
            result.program.steps().iter().any(|s| matches!(s,
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting
                && to.x == 9. && (2.5..3.5).contains(&to.y) && to.z > 0.4)),
            "the connector must follow the crest instead of crossing it at endpoint Z"
        );
        // Independently check the emitted cutting points against the actual
        // drop-ball contact, including intermediate points along the turn.
        for step in result.program.steps() {
            if let Step::Linear { to, tag, .. } = step {
                if tag.kind != MoveKind::Cutting {
                    continue;
                }
                for triangle in &geometry.triangles {
                    if let Some(required) = drop_ball([to.x, to.y, to.z], triangle, 1.) {
                        assert!(to.z >= required);
                    }
                }
            }
        }
        let mut program = cam_cldata::Program::new();
        program.push(Step::ToolChange { tool: 1 });
        program.extend(result.program);
        let verified = cam_sim::simulate(
            &program,
            [-1., -1., -2.],
            [11., 11., 0.8],
            &cam_sim::SimOptions {
                resolution: 0.2,
                tool_radius: 1.,
            },
            &[cam_sim::SimTool {
                number: 1,
                profile: cam_sim::ToolProfile {
                    radius: 1.,
                    shape: cam_sim::ProfileShape::Ball,
                },
                noncutting_tip_radius: None,
            }],
        );
        assert!(verified.is_clean(), "{:?}", verified.collisions);
        assert!(verified.removed_volume > 0.);
    }

    #[test]
    fn lifted_parallel_links_preserve_surface_cuts_and_return_to_depth() {
        for (angle, stock) in [
            (0., StockToLeave::default()),
            (37., StockToLeave::default()),
            (
                0.,
                StockToLeave {
                    enabled: true,
                    radial: 0.2,
                    axial: 0.7,
                },
            ),
        ] {
            let mut operation = op();
            operation.params.angle = angle;
            let baseline = run_with_stock(operation.clone(), surface(), stock);
            operation.params.stay_down = true;
            operation.params.lift_height = 2.;
            let lifted = run_with_stock(operation.clone(), surface(), stock);
            assert!(!lifted.has_errors(), "{:?}", lifted.diagnostics);
            assert!(lifted.program.validate().is_ok());
            assert!(local_links(&lifted) > 0);
            let cuts = |result: &StrategyResult| {
                result
                    .program
                    .steps()
                    .iter()
                    .filter_map(|s| match s {
                        Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(*to),
                        _ => None,
                    })
                    .collect::<Vec<_>>()
            };
            assert_eq!(cuts(&lifted), cuts(&baseline));
            let retracts =
                |result: &StrategyResult| {
                    result.program.steps().iter().filter(|s|
                matches!(s, Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract)).count()
                };
            assert!(retracts(&lifted) < retracts(&baseline));
            for moves in lifted.program.steps().windows(5) {
                if let [Step::Linear { to: from, tag, .. }, Step::Linear { to: up, .. }, Step::Linear { to: across, .. }, Step::Linear {
                    to: down,
                    feed,
                    tag: entry,
                }, Step::Linear { to: cut, .. }] = moves
                {
                    if tag.kind == MoveKind::Cutting && entry.kind == MoveKind::Plunge {
                        assert_eq!(up.x, from.x);
                        assert_eq!(up.y, from.y);
                        assert_eq!(up.z, from.z.max(down.z) + 2.);
                        assert_eq!(across.z, up.z);
                        assert_eq!((across.x, across.y), (down.x, down.y));
                        assert_eq!(*feed, operation.params.plunge_feed);
                        let (s, c) = angle.to_radians().sin_cos();
                        assert!((-s * (cut.x - down.x) + c * (cut.y - down.y)).abs() < 1e-8);
                    }
                }
            }
            assert!(
                matches!(lifted.program.steps().last(), Some(Step::Rapid { to, tag })
                if to.z == 10. && tag.kind == MoveKind::Retract)
            );
        }
    }

    #[test]
    fn parallel_links_retract_for_excessive_lifts_long_returns_and_disconnected_rows() {
        let mut operation = op();
        operation.params.stay_down = true;
        operation.params.lift_height = 100.;
        let high = run(operation.clone(), surface());
        assert!(!high.has_errors());
        assert_eq!(local_links(&high), 0);
        operation.params.stay_down = true;
        operation.params.lift_height = 2.;
        operation.params.both_ways = false;
        let one_way = run(operation.clone(), surface());
        assert!(!one_way.has_errors());
        assert_eq!(local_links(&one_way), 0);
        operation.params.both_ways = true;
        for (index, selected) in [
            [rectangle(0., 0., 3., 10.), rectangle(7., 0., 10., 10.)].concat(),
            [rectangle(0., 0., 10., 3.), rectangle(0., 7., 10., 10.)].concat(),
        ]
        .into_iter()
        .enumerate()
        {
            let mut geometry = surface();
            geometry.selected = selected;
            let split = run(operation.clone(), geometry);
            assert!(!split.has_errors(), "{:?}", split.diagnostics);
            // Missing samples and empty rows must break the linking chain.
            let mut previous = None;
            for step in split.program.steps() {
                match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Link => {
                        if let Some(from) = previous {
                            let from: Point3 = from;
                            assert!((to.x - from.x).hypot(to.y - from.y) <= 1.5);
                        }
                        previous = Some(*to);
                    }
                    Step::Linear { to, .. } => previous = Some(*to),
                    Step::Rapid { .. } => previous = None,
                    _ => {}
                }
            }
            if index == 0 {
                assert_eq!(
                    local_links(&split),
                    0,
                    "rows split across their travel direction must retract"
                );
            }
        }
    }

    #[test]
    fn a_narrow_unselected_ridge_blocks_a_parallel_link_between_clear_endpoints() {
        for (height, stock) in [
            (5., StockToLeave::default()),
            (
                2.5,
                StockToLeave {
                    enabled: true,
                    radial: 0.,
                    axial: 2.,
                },
            ),
        ] {
            let mut geometry = surface();
            geometry.triangles = rectangle(0., 0., 10., 10.).to_vec();
            geometry.selected = geometry.triangles.clone();
            geometry
                .triangles
                .push([[8.9, 3., 0.], [9.1, 3., 0.], [9., 3., height]]);
            geometry.heights.top = 6.;
            let mut operation = op();
            operation.params.stepover = 4.;
            operation.params.stay_down = true;
            operation.params.lift_height = 2.;
            let result = run_with_stock(operation, geometry, stock);
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
            assert!(result.program.steps().iter().any(|s| matches!(s,
            Step::Rapid { to, tag } if tag.kind == MoveKind::Retract
                && to.x == 9. && to.y == 1. && to.z == 10.)));
            assert!(
                local_links(&result) > 0,
                "unblocked neighboring rows should still link locally"
            );
        }
    }

    #[test]
    fn a_selection_gap_between_rows_uses_clearance_even_when_the_model_is_flat() {
        let mut geometry = surface();
        geometry.triangles = rectangle(0., 0., 10., 10.).to_vec();
        geometry.selected = [rectangle(0., 0., 10., 1.4), rectangle(0., 1.6, 10., 10.)].concat();
        let mut operation = op();
        operation.params.stay_down = true;
        operation.params.lift_height = 0.;
        let result = run(operation, geometry);
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(result.program.steps().iter().any(|s| matches!(s,
            Step::Rapid { to, tag } if tag.kind == MoveKind::Retract
                && to.x == 9. && to.y == 1. && to.z == 10.)));
        assert_eq!(
            result
                .program
                .steps()
                .iter()
                .filter(|s| matches!(s,
            Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract))
                .count(),
            2,
            "only the real selection gap and the final exit need a retract"
        );
    }

    #[test]
    fn invalid_parallel_lift_heights_emit_no_motion() {
        for height in [-1., f64::NAN, f64::INFINITY] {
            let mut operation = op();
            operation.params.stay_down = true;
            operation.params.lift_height = height;
            let result = run(operation, surface());
            assert!(result.has_errors());
            assert!(result.program.steps().is_empty());
        }
    }
    #[test]
    fn ball_contact_accounts_for_face_edge_vertex_and_vertical_wall() {
        let t = [[0., 0., 0.], [10., 0., 2.], [0., 10., 0.]];
        let expected = 0.2 * 3. + (1.04_f64.sqrt() - 1.);
        assert!((drop_ball([3., 3., 0.], &t, 1.).unwrap() - expected).abs() < 1e-10);
        assert_eq!(drop_ball([-1., 0., 0.], &t, 1.), Some(-1.));
        assert!(drop_ball([-2., 0., 0.], &t, 1.).is_none());
        let wall = [[0., 0., 0.], [0., 0., 5.], [0., 5., 5.]];
        assert!(
            (drop_ball([0.5, 2., 0.], &wall, 1.).unwrap() - (4. + 0.75_f64.sqrt())).abs() < 1e-10
        );
    }
    #[test]
    fn swept_distance_detects_ridge_between_clear_endpoints() {
        let t = [[0., -1., 0.], [0., 1., 0.], [0., 0., 2.]];
        assert_eq!(segment_triangle([-1., 0., 1.], [1., 0., 1.], &t), 0.);
        assert!((segment_triangle([-1., 0., 3.], [1., 0., 3.], &t) - 1.).abs() < 1e-10);
    }
    #[test]
    fn sloped_surface_has_variable_z_parallel_rows_and_full_clearance_links() {
        let tool = Tool {
            kind: ToolKind::BallMill,
            diameter: 2.,
            number: 1,
            length: 20.,
            ..Default::default()
        };
        let env = JobEnv {
            heights: Heights {
                clearance: 10.,
                retract: 5.,
                top_of_stock: 3.,
            },
            tools: &[tool],
            stock: None,
            spindle: SpindleDir::Cw,
        };
        let result = ParallelStrategy::new(op(), StockToLeave::default(), surface())
            .compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let mut last: Option<f64> = None;
        let mut zmin = f64::INFINITY;
        let mut zmax = f64::NEG_INFINITY;
        for step in result.program.steps() {
            match step {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                    if let Some(prev) = last {
                        assert!((to.y - prev).abs() < 1e-8);
                    }
                    last = Some(to.y);
                    zmin = zmin.min(to.z);
                    zmax = zmax.max(to.z);
                }
                Step::Rapid { to, .. } => {
                    assert_eq!(to.z, 10.);
                    last = None;
                }
                _ => {}
            }
        }
        assert!(zmax - zmin > 1.);
    }
    #[test]
    fn invalid_inputs_and_cancellation_emit_no_motion() {
        let tool = Tool {
            number: 1,
            diameter: 2.,
            length: 20.,
            ..Default::default()
        };
        let env = JobEnv {
            heights: Heights {
                clearance: 10.,
                retract: 5.,
                top_of_stock: 3.,
            },
            tools: &[tool],
            stock: None,
            spindle: SpindleDir::Cw,
        };
        let strategy = ParallelStrategy::new(op(), StockToLeave::default(), surface());
        assert!(strategy.compute(&env, &CancelToken::new()).has_errors());
        let cancel = CancelToken::new();
        cancel.cancel();
        let r = strategy.compute(&env, &cancel);
        assert!(r.cancelled);
        assert!(r.program.steps().is_empty());
    }
    #[test]
    fn angled_one_way_rows_remain_parallel_and_protect_unselected_boss() {
        let tool = Tool {
            number: 1,
            diameter: 2.,
            length: 20.,
            kind: ToolKind::BallMill,
            ..Default::default()
        };
        let env = JobEnv {
            heights: Heights {
                clearance: 10.,
                retract: 5.,
                top_of_stock: 5.,
            },
            tools: &[tool],
            stock: None,
            spindle: SpindleDir::Cw,
        };
        let mut geometry = surface();
        geometry.triangles.extend([
            [[4., 4., 4.], [6., 4., 4.], [6., 6., 4.]],
            [[4., 4., 4.], [6., 6., 4.], [4., 6., 4.]],
        ]);
        geometry.heights.top = 5.;
        let mut operation = op();
        operation.params.angle = 37.;
        operation.params.both_ways = false;
        let r = ParallelStrategy::new(operation, StockToLeave::default(), geometry)
            .compute(&env, &CancelToken::new());
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        let (s, c) = 37_f64.to_radians().sin_cos();
        let mut last: Option<Point3> = None;
        let mut highest = f64::NEG_INFINITY;
        for step in r.program.steps() {
            match step {
                Step::Linear { to, tag, .. } => {
                    if tag.kind == MoveKind::Cutting {
                        let p = last.unwrap();
                        let dx = to.x - p.x;
                        let dy = to.y - p.y;
                        assert!((-s * dx + c * dy).abs() < 1e-8);
                        assert!(c * dx + s * dy >= -1e-8);
                        highest = highest.max(to.z);
                    }
                    last = Some(*to);
                }
                Step::Rapid { .. } => last = None,
                _ => {}
            }
        }
        assert!(highest >= 4., "the unselected boss must lift the cutter");
    }

    #[test]
    fn clearance_below_model_and_spatial_index_overflow_fail_closed() {
        let tool = Tool {
            number: 1,
            diameter: 2.,
            length: 20.,
            kind: ToolKind::BallMill,
            ..Default::default()
        };
        let env = JobEnv {
            heights: Heights {
                clearance: 1.,
                retract: 0.5,
                top_of_stock: 0.,
            },
            tools: &[tool],
            stock: None,
            spindle: SpindleDir::Cw,
        };
        let r = ParallelStrategy::new(op(), StockToLeave::default(), surface())
            .compute(&env, &CancelToken::new());
        assert!(r.has_errors());
        assert!(r.program.steps().is_empty());
        assert!(Grid::new(
            &[[[0., 0., 0.], [1e6, 0., 0.], [0., 1e6, 0.]]],
            1.,
            &CancelToken::new()
        )
        .is_err());
    }
}
