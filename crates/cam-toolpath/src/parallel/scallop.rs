//! Sampled surface-distance contours. The distance field is computed on the
//! accessible cutter-centre surface, then each contour is reprojected and
//! certified by the same swept-cutter checks as Parallel.
use super::surface_passes::{emit_path, PassSettings};
use super::*;
use cam_model::ScallopOrder;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

#[derive(Clone, Copy, PartialEq)]
struct QueueItem {
    distance: f64,
    index: usize,
}
impl Eq for QueueItem {}
impl Ord for QueueItem {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .distance
            .total_cmp(&self.distance)
            .then_with(|| self.index.cmp(&other.index))
    }
}
impl PartialOrd for QueueItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

type Edge = (usize, usize);
fn edge(a: usize, b: usize) -> Edge {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

fn neighbors(index: usize, nx: usize, ny: usize) -> [Option<usize>; 4] {
    let x = index % nx;
    let y = index / nx;
    [
        (x > 0).then(|| index - 1),
        (x + 1 < nx).then(|| index + 1),
        (y > 0).then(|| index - nx),
        (y + 1 < ny).then(|| index + nx),
    ]
}

/// Adjust only when the previous pass and the furthest sampled point both
/// remain within one stepover of the replacement pass.
fn adjusted_final_level(
    inner_edge: f64,
    max_distance: f64,
    spacing: f64,
    step: f64,
    levels: usize,
) -> Option<f64> {
    if levels <= 1 || !inner_edge.is_finite() {
        return None;
    }
    let nominal = (levels as f64 - 0.499_999) * step;
    let previous = nominal - step;
    let remaining = (step - (max_distance - inner_edge)).max(0.);
    let adjusted = inner_edge - spacing.min(remaining * 0.5);
    (nominal < max_distance
        && nominal > inner_edge
        && adjusted > previous + step * 0.05
        && max_distance - adjusted <= step)
        .then_some(adjusted)
}

/// Seed only the outer boundary of each connected selected region. Enclosed
/// holes are obstacles, not additional fronts moving toward the middle. Label
/// each background component, then use the background above a region's
/// topmost point as its exterior. This also handles a selected island inside
/// another region's hole without confusing it with that hole's inner edge.
fn outer_boundary_seeds(
    points: &[Option<V>],
    nx: usize,
    ny: usize,
    cancel: &CancelToken,
) -> Option<Vec<usize>> {
    let mut background = vec![usize::MAX; points.len()];
    let mut queue = VecDeque::new();
    for index in 0..points.len() {
        let x = index % nx;
        let y = index / nx;
        if (x == 0 || y == 0 || x + 1 == nx || y + 1 == ny) && points[index].is_none() {
            background[index] = 0;
            queue.push_back(index);
        }
    }
    let mut label = 0usize;
    let mut search_from = 0usize;
    loop {
        while let Some(index) = queue.pop_front() {
            if cancel.is_cancelled() {
                return None;
            }
            for next in neighbors(index, nx, ny).into_iter().flatten() {
                if points[next].is_none() && background[next] == usize::MAX {
                    background[next] = label;
                    queue.push_back(next);
                }
            }
        }
        while search_from < points.len()
            && (points[search_from].is_some() || background[search_from] != usize::MAX)
        {
            search_from += 1;
        }
        if search_from == points.len() {
            break;
        }
        label += 1;
        background[search_from] = label;
        queue.push_back(search_from);
    }
    let mut visited = vec![false; points.len()];
    let mut seeds = Vec::new();
    for first in 0..points.len() {
        if cancel.is_cancelled() {
            return None;
        }
        if points[first].is_none() || visited[first] {
            continue;
        }
        visited[first] = true;
        queue.push_back(first);
        let mut component = Vec::new();
        let mut topmost = first;
        while let Some(index) = queue.pop_front() {
            if cancel.is_cancelled() {
                return None;
            }
            component.push(index);
            if index / nx < topmost / nx {
                topmost = index;
            }
            for next in neighbors(index, nx, ny).into_iter().flatten() {
                if points[next].is_some() && !visited[next] {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
        }
        let exterior = if topmost / nx == 0 {
            0
        } else {
            background[topmost - nx]
        };
        for index in component {
            let x = index % nx;
            let y = index / nx;
            if x == 0
                || y == 0
                || x + 1 == nx
                || y + 1 == ny
                || neighbors(index, nx, ny)
                    .into_iter()
                    .flatten()
                    .any(|next| points[next].is_none() && background[next] == exterior)
            {
                seeds.push(index);
            }
        }
    }
    Some(seeds)
}

fn preferred_start(chain: &mut Vec<Edge>, positions: &HashMap<Edge, V>, start: Option<[f64; 2]>) {
    let Some(start) = start else {
        return;
    };
    let distance = |key: &Edge| {
        let p = positions[key];
        (p[0] - start[0]).powi(2) + (p[1] - start[1]).powi(2)
    };
    if chain.len() > 2 && chain.first() == chain.last() {
        chain.pop();
        if let Some((index, _)) = chain
            .iter()
            .enumerate()
            .min_by(|a, b| distance(a.1).total_cmp(&distance(b.1)))
        {
            chain.rotate_left(index);
        }
        chain.push(chain[0]);
    } else if chain.len() > 1 && distance(chain.last().unwrap()) < distance(&chain[0]) {
        chain.reverse();
    }
}

const MAX_QUEUED_POINTS: usize = 2_000_000;
const MAX_QUEUED_PATHS: usize = 100_000;
const ORDER_BATCH: usize = 64;
const ORDER_SAMPLES: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PathEntry {
    Original,
    Reverse,
    Rotate(usize),
}

fn xy_distance(a: V, b: V) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn closed(path: &[V]) -> bool {
    path.len() > 2 && path.first() == path.last()
}

/// Estimate a candidate's nearest entry using a bounded sample. Once selected,
/// scan the whole contour so its actual start is the nearest eligible vertex.
fn nearest_entry(
    path: &[V],
    from: V,
    allow_reorientation: bool,
    sampled: bool,
) -> (f64, PathEntry) {
    let mut best = (xy_distance(from, path[0]), PathEntry::Original);
    if !allow_reorientation {
        return best;
    }
    if closed(path) {
        let n = path.len() - 1;
        let stride = if sampled {
            n.div_ceil(ORDER_SAMPLES)
        } else {
            1
        };
        for index in (0..n).step_by(stride) {
            let distance = xy_distance(from, path[index]);
            if distance < best.0 {
                best = (distance, PathEntry::Rotate(index));
            }
        }
    } else {
        let distance = xy_distance(from, *path.last().unwrap());
        if distance < best.0 {
            best = (distance, PathEntry::Reverse);
        }
    }
    best
}

fn entry_end(path: &[V], entry: PathEntry) -> (V, V) {
    match entry {
        PathEntry::Original => (path[0], *path.last().unwrap()),
        PathEntry::Reverse => (*path.last().unwrap(), path[0]),
        PathEntry::Rotate(index) => (path[index], path[index]),
    }
}

/// Greedy selection within each stepover level, in bounded batches. Never
/// jump across levels to save travel: that can leave uncut stock between
/// passes. Retain the original order if the XY estimate does not improve it.
pub(super) fn shortest_order(
    paths: &[Vec<V>],
    levels: &[usize],
    allow_reorientation: bool,
    cancel: &CancelToken,
) -> Option<Vec<(usize, PathEntry)>> {
    let baseline: f64 = paths
        .windows(2)
        .map(|pair| xy_distance(*pair[0].last().unwrap(), pair[1][0]))
        .sum();
    let mut candidate = Vec::with_capacity(paths.len());
    let mut used = vec![false; paths.len()];
    let mut previous: Option<V> = None;
    let mut total = 0.;
    let mut batch_start = 0;
    while batch_start < paths.len() {
        let level = levels[batch_start];
        let mut batch_end = batch_start;
        while batch_end < paths.len()
            && batch_end - batch_start < ORDER_BATCH
            && levels[batch_end] == level
        {
            batch_end += 1;
        }
        for _ in batch_start..batch_end {
            if cancel.is_cancelled() {
                return None;
            }
            let (chosen, entry) = if let Some(from) = previous {
                let mut best: Option<(f64, usize)> = None;
                for index in batch_start..batch_end {
                    if used[index] {
                        continue;
                    }
                    let (cost, _) = nearest_entry(&paths[index], from, allow_reorientation, true);
                    if best.is_none_or(|(prior, prior_index)| {
                        cost < prior || (cost == prior && index < prior_index)
                    }) {
                        best = Some((cost, index));
                    }
                }
                let index = best.unwrap().1;
                (
                    index,
                    nearest_entry(&paths[index], from, allow_reorientation, false).1,
                )
            } else {
                (batch_start, PathEntry::Original)
            };
            used[chosen] = true;
            let (start, end) = entry_end(&paths[chosen], entry);
            if let Some(from) = previous {
                total += xy_distance(from, start);
            }
            previous = Some(end);
            candidate.push((chosen, entry));
        }
        batch_start = batch_end;
    }
    if total + 1e-9 < baseline {
        Some(candidate)
    } else {
        Some((0..paths.len()).map(|i| (i, PathEntry::Original)).collect())
    }
}

fn orient_path(path: &mut Vec<V>, entry: PathEntry) {
    match entry {
        PathEntry::Original => {}
        PathEntry::Reverse => path.reverse(),
        PathEntry::Rotate(index) => {
            path.pop();
            path.rotate_left(index);
            path.push(path[0]);
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn accept_path(
    out: &mut StrategyResult,
    path: &mut Vec<V>,
    pending: &mut Vec<Vec<V>>,
    pending_levels: &mut Vec<usize>,
    level: usize,
    queued_points: &mut usize,
    linker: &mut PassLinker,
    op: &ParallelOp,
    cancel: &CancelToken,
) -> bool {
    if op.params.scallop_order == ScallopOrder::Shortest && path.len() > 1 {
        *queued_points = queued_points.saturating_add(path.len());
        if *queued_points > MAX_QUEUED_POINTS || pending.len() >= MAX_QUEUED_PATHS {
            return false;
        }
        pending.push(std::mem::take(path));
        pending_levels.push(level);
    } else {
        emit_path(out, path, linker, pass_settings(op), None, cancel);
        path.clear();
    }
    true
}

fn pass_settings(op: &ParallelOp) -> PassSettings {
    let p = op.params;
    PassSettings {
        id: op.id,
        feed: p.feed,
        plunge_feed: p.plunge_feed,
        tolerance: p.tolerance,
        retraction: p.scallop_retraction,
        safe_distance: p.scallop_safe_distance,
        max_stay_down_distance: f64::INFINITY,
        smoothing_tolerance: p.scallop_smoothing.then_some(p.scallop_smoothing_tolerance),
    }
}

pub(super) fn compute(
    grid: &Grid,
    selected: &Grid,
    linker: &mut PassLinker,
    op: &ParallelOp,
    lo: [f64; 2],
    hi: [f64; 2],
    cancel: &CancelToken,
) -> StrategyResult {
    let mut out = StrategyResult::default();
    let p = op.params;
    macro_rules! fail {
        ($message:expr) => {{
            out.program = Default::default();
            out.diagnostics.push(Diagnostic::error($message));
            return out;
        }};
    }
    if p.scallop_start
        .is_some_and(|point| point.iter().any(|v| !v.is_finite()))
        || !p.scallop_safe_distance.is_finite()
        || p.scallop_safe_distance < 0.
        || (p.scallop_smoothing
            && (!p.scallop_smoothing_tolerance.is_finite() || p.scallop_smoothing_tolerance <= 0.))
    {
        fail!("Scallop entry, safe distance, and smoothing tolerance must be finite and valid");
    }
    // Four samples per stepover. The graph distance and contours are an
    // approximation, so this is deliberately bounded and documented.
    let spacing = (p.stepover / 4.).min(0.5);
    let counts = [0, 1].map(|axis| ((hi[axis] - lo[axis]) / spacing).ceil());
    if counts.iter().any(|n| !n.is_finite() || *n > 99_999.) {
        fail!("Scallop exceeds the one-million sample budget; increase stepover or reduce the machining area");
    }
    let nx = counts[0] as usize + 1;
    let ny = counts[1] as usize + 1;
    if nx < 2 || ny < 2 || nx > 100_000 || ny > 100_000 || nx.saturating_mul(ny) > 1_000_000 {
        fail!("Scallop exceeds the one-million sample budget; increase stepover or reduce the machining area");
    }
    let dx = (hi[0] - lo[0]) / (nx - 1) as f64;
    let dy = (hi[1] - lo[1]) / (ny - 1) as f64;
    let margin = linker.error + p.tolerance + linker.axial;
    let reach = linker.radius + linker.error;
    let mut points = vec![None; nx * ny];
    // Each sample is independent; retain f64 contact math and indexed storage.
    // The distance propagation and motion ordering below remain deterministic.
    points
        .par_iter_mut()
        .enumerate()
        .with_min_len(64)
        .for_each(|(index, point)| {
            if cancel.is_cancelled() {
                return;
            }
            let x = index % nx;
            let y = index / nx;
            let mut q = [lo[0] + x as f64 * dx, lo[1] + y as f64 * dy, 0.];
            if !selected.contact(q) {
                return;
            }
            let Some(z) = grid.height(q, reach) else {
                return;
            };
            let wall = grid
                .indices(q, reach)
                .iter()
                .filter_map(|i| flat_height(q, &grid.mesh[*i], reach))
                .fold(f64::NEG_INFINITY, f64::max);
            q[2] = (z + margin)
                .max(wall - linker.radius + margin)
                .max(linker.bottom);
            if q[2] <= linker.top + margin && q[2] < linker.clearance {
                *point = Some(q);
            }
        });
    if cancel.is_cancelled() {
        out.cancelled = true;
        return out;
    }
    let mut distance = vec![f64::INFINITY; points.len()];
    let mut queue = BinaryHeap::new();
    let Some(seeds) = outer_boundary_seeds(&points, nx, ny, cancel) else {
        out.cancelled = true;
        return out;
    };
    for index in seeds {
        distance[index] = 0.;
        queue.push(QueueItem {
            distance: 0.,
            index,
        });
    }
    while let Some(QueueItem {
        distance: d,
        index: i,
    }) = queue.pop()
    {
        if cancel.is_cancelled() {
            out.cancelled = true;
            return out;
        }
        if d > distance[i] {
            continue;
        }
        let x = i % nx;
        let y = i / nx;
        for (ox, oy) in [
            (-1isize, -1isize),
            (0, -1),
            (1, -1),
            (-1, 0),
            (1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        ] {
            let (xx, yy) = (x as isize + ox, y as isize + oy);
            if xx < 0 || yy < 0 || xx >= nx as isize || yy >= ny as isize {
                continue;
            }
            let j = yy as usize * nx + xx as usize;
            let (Some(a), Some(b)) = (points[i], points[j]) else {
                continue;
            };
            // Do not propagate diagonally across a gap or disconnected faces.
            if ox != 0
                && oy != 0
                && (points[y * nx + xx as usize].is_none()
                    || points[yy as usize * nx + x].is_none())
            {
                continue;
            }
            let candidate = d + dot(sub(a, b), sub(a, b)).sqrt();
            if candidate < distance[j] {
                distance[j] = candidate;
                queue.push(QueueItem {
                    distance: candidate,
                    index: j,
                });
            }
        }
    }
    let max_distance = distance
        .iter()
        .copied()
        .filter(|d| d.is_finite())
        .fold(0., f64::max);
    let levels = (max_distance / p.stepover).ceil() as usize;
    if levels > 100_000 || levels.saturating_mul(points.len()) > 100_000_000 {
        fail!("Scallop exceeds the contour budget; increase stepover or reduce the machining area");
    }
    // A nominal last level can graze an enclosed boundary at only a few grid
    // cells. Marching that level then produces dozens of tiny disconnected
    // paths even for a smooth circular bore. If a nearby continuous offset can
    // cover the complete sampled domain without a gap exceeding one stepover,
    // move the final level just before the first enclosed boundary. Otherwise
    // retain all original fragments: uneven regions must not be discarded.
    let mut final_level = None;
    if levels > 1 {
        let inner_edge = points
            .iter()
            .enumerate()
            .filter_map(|(i, p)| {
                (p.is_some()
                    && distance[i] > 0.
                    && distance[i].is_finite()
                    && neighbors(i, nx, ny)
                        .into_iter()
                        .flatten()
                        .any(|j| points[j].is_none()))
                .then_some(distance[i])
            })
            .fold(f64::INFINITY, f64::min);
        final_level = adjusted_final_level(inner_edge, max_distance, spacing, p.stepover, levels);
    }
    let mut pending = Vec::new();
    let mut pending_levels = Vec::new();
    let mut queued_points = 0usize;
    macro_rules! finish_path {
        ($path:expr, $level:expr) => {
            if !accept_path(&mut out, $path, &mut pending, &mut pending_levels, $level, &mut queued_points, linker, op, cancel) {
                fail!("Scallop shortest-distance ordering exceeds its path memory budget; increase stepover or choose a fixed order");
            }
        };
    }
    for ordinal in 0..levels {
        if cancel.is_cancelled() {
            out.cancelled = true;
            return out;
        }
        // A half-step begins near the boundary and covers the centre without
        // relying on a degenerate contour exactly on a sampled vertex.
        let level_index = if p.scallop_order == ScallopOrder::InsideOut {
            levels - 1 - ordinal
        } else {
            ordinal
        };
        let level = if level_index + 1 == levels {
            final_level.unwrap_or((level_index as f64 + 0.500_001) * p.stepover)
        } else {
            (level_index as f64 + 0.500_001) * p.stepover
        };
        if level >= max_distance {
            continue;
        }
        let mut positions = HashMap::<Edge, V>::new();
        let mut adjacency = HashMap::<Edge, Vec<Edge>>::new();
        for y in 0..ny - 1 {
            if y % 64 == 0 && cancel.is_cancelled() {
                out.cancelled = true;
                return out;
            }
            for x in 0..nx - 1 {
                let a = y * nx + x;
                let b = a + 1;
                let c = a + nx;
                let d = c + 1;
                for tri in [[a, b, d], [a, d, c]] {
                    if tri.iter().any(|i| points[*i].is_none()) {
                        continue;
                    }
                    let mut crossings = Vec::with_capacity(2);
                    for (u, v) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                        if (distance[u] < level) == (distance[v] < level) {
                            continue;
                        }
                        let key = edge(u, v);
                        let t = (level - distance[u]) / (distance[v] - distance[u]);
                        let point = add(
                            points[u].unwrap(),
                            mul(sub(points[v].unwrap(), points[u].unwrap()), t),
                        );
                        positions.entry(key).or_insert(point);
                        crossings.push(key);
                    }
                    if crossings.len() == 2 && crossings[0] != crossings[1] {
                        adjacency
                            .entry(crossings[0])
                            .or_default()
                            .push(crossings[1]);
                        adjacency
                            .entry(crossings[1])
                            .or_default()
                            .push(crossings[0]);
                    }
                }
            }
        }
        // Trace each contour once. Prefer open endpoints, then closed loops.
        while !adjacency.is_empty() {
            let start = adjacency
                .iter()
                .filter(|(_, v)| v.len() == 1)
                .map(|(k, _)| *k)
                .min()
                .or_else(|| adjacency.keys().copied().min())
                .unwrap();
            let mut chain = vec![start];
            let mut current = start;
            loop {
                let Some(next) = adjacency.get_mut(&current).and_then(|v| v.pop()) else {
                    adjacency.remove(&current);
                    break;
                };
                if adjacency.get(&current).is_some_and(|v| v.is_empty()) {
                    adjacency.remove(&current);
                }
                if let Some(back) = adjacency.get_mut(&next) {
                    if let Some(index) = back.iter().position(|v| *v == current) {
                        back.swap_remove(index);
                    }
                    if back.is_empty() {
                        adjacency.remove(&next);
                    }
                }
                chain.push(next);
                current = next;
                if current == start {
                    break;
                }
            }
            preferred_start(&mut chain, &positions, p.scallop_start);
            let mut path = Vec::new();
            for key in chain {
                let mut q = positions[&key];
                let Some(z) = grid.height(q, reach) else {
                    finish_path!(&mut path, level_index);
                    continue;
                };
                let wall = grid
                    .indices(q, reach)
                    .iter()
                    .filter_map(|i| flat_height(q, &grid.mesh[*i], reach))
                    .fold(f64::NEG_INFINITY, f64::max);
                q[2] = (z + margin)
                    .max(wall - linker.radius + margin)
                    .max(linker.bottom);
                if q[2] > linker.top + margin || q[2] >= linker.clearance {
                    finish_path!(&mut path, level_index);
                    continue;
                }
                if let Some(prev) = path.last().copied() {
                    let length = path.len();
                    if !selected.covers_segment(prev, q, cancel)
                        || !refine(
                            grid,
                            prev,
                            q,
                            linker.radius,
                            linker.error,
                            margin,
                            p.tolerance,
                            linker.bottom,
                            0,
                            &mut path,
                            cancel,
                        )
                    {
                        path.truncate(length);
                        finish_path!(&mut path, level_index);
                        path.push(q);
                    }
                } else {
                    path.push(q);
                }
            }
            finish_path!(&mut path, level_index);
            if out.program.steps().len() > 2_000_000 {
                fail!("Scallop exceeds the two-million motion budget");
            }
        }
    }
    if p.scallop_order == ScallopOrder::Shortest {
        let Some(order) =
            shortest_order(&pending, &pending_levels, p.scallop_start.is_none(), cancel)
        else {
            out.cancelled = true;
            return out;
        };
        let mut paths: Vec<_> = pending.into_iter().map(Some).collect();
        for (index, entry) in order {
            if cancel.is_cancelled() {
                out.cancelled = true;
                return out;
            }
            let mut path = paths[index].take().expect("each contour is scheduled once");
            orient_path(&mut path, entry);
            emit_path(&mut out, &path, linker, pass_settings(op), None, cancel);
            if out.program.steps().len() > 2_000_000 {
                fail!("Scallop exceeds the two-million motion budget");
            }
        }
    }
    linker.retract(&mut out, op.id);
    if cancel.is_cancelled() {
        out.cancelled = true;
        return out;
    }
    if linker.passes == 0 {
        fail!("Scallop produced no surface contact passes; the region may be too narrow for this stepover");
    }
    out.diagnostics.push(Diagnostic::info(format!(
        "3D Scallop: {} sampled surface-offset passes at {:.3} mm stepover",
        linker.passes, p.stepover
    )));
    if p.scallop_smoothing {
        let cutting = out
            .program
            .steps()
            .iter()
            .filter(|step| {
                matches!(step,
            Step::Linear { tag, .. } | Step::Arc { tag, .. } if tag.kind == MoveKind::Cutting)
            })
            .count();
        let arcs = out
            .program
            .steps()
            .iter()
            .filter(|step| {
                matches!(step,
            Step::Arc { tag, .. } if tag.kind == MoveKind::Cutting)
            })
            .count();
        out.diagnostics.push(Diagnostic::info(format!(
            "Scallop smoothing: {cutting} cutting motions including {arcs} arcs; filtering tolerance {:.4} mm",
            p.scallop_smoothing_tolerance
        )));
    }
    out.diagnostics.push(Diagnostic::warning(
        "Scallop stepover is approximate; vertical walls and narrow features may remain unfinished. Inspect the simulated result.",
    ));
    out
}

#[cfg(test)]
mod tests {
    use super::super::surface_passes::{fitted_arcs, simplify};
    use super::*;
    use cam_model::ScallopRetraction;

    #[test]
    fn final_offset_adjustment_keeps_both_coverage_and_prior_stepover_bounded() {
        let (inner_edge, furthest, spacing, step, levels) = (4.0725, 4.381, 0.125, 0.5, 9);
        let adjusted = adjusted_final_level(inner_edge, furthest, spacing, step, levels).unwrap();
        let previous = (levels as f64 - 1.499_999) * step;
        assert!(adjusted < inner_edge && adjusted > previous);
        assert!(adjusted - previous <= step);
        assert!(furthest - adjusted <= step);
        assert!(
            adjusted_final_level(3., 5.4, spacing, step, 11).is_none(),
            "an irregular wide region must retain its full offsets"
        );
        assert!(adjusted_final_level(f64::INFINITY, furthest, spacing, step, levels).is_none());
    }

    #[test]
    fn outer_seeds_ignore_holes_but_include_nested_selected_islands() {
        let (nx, ny) = (9, 9);
        let mut points = vec![None; nx * ny];
        for y in 1..=7 {
            for x in 1..=7 {
                if !(3..=5).contains(&x) || !(3..=5).contains(&y) || (x == 4 && y == 4) {
                    points[y * nx + x] = Some([x as f64, y as f64, 0.]);
                }
            }
        }
        let seeds = outer_boundary_seeds(&points, nx, ny, &CancelToken::new()).unwrap();
        assert!(
            seeds.contains(&(4 * nx + 1)),
            "outer ring must start at its exterior"
        );
        assert!(
            !seeds.contains(&(4 * nx + 2)),
            "the hole must not create an opposing front"
        );
        assert!(
            seeds.contains(&(4 * nx + 4)),
            "a separate island inside the hole has its own outer boundary"
        );
    }

    #[test]
    fn shortest_order_never_skips_to_a_later_stepover_level() {
        let paths: Vec<Vec<V>> = [0., 100., 1., 2.]
            .into_iter()
            .map(|x| vec![[x, 0., 0.], [x, 1., 0.]])
            .collect();
        let order = shortest_order(&paths, &[0, 1, 2, 3], true, &CancelToken::new()).unwrap();
        assert_eq!(
            order.iter().map(|(index, _)| *index).collect::<Vec<_>>(),
            vec![0, 1, 2, 3]
        );
    }

    #[test]
    fn smoothing_fits_only_certified_planar_arcs() {
        let triangle = [[-10., -10., 0.], [30., -10., 0.], [-10., 30., 0.]];
        let cancel = CancelToken::new();
        let selected_mesh = [triangle];
        let selected = Grid::new(&selected_mesh, 4., &cancel).unwrap();
        let grid = Grid::new(&selected_mesh, 4., &cancel).unwrap();
        let linker = PassLinker {
            grid: &grid,
            selected: &selected,
            radius: 1.,
            error: 0.0025,
            axial: 0.,
            clearance: 10.,
            bottom: -1.,
            top: 1.,
            previous: None,
            passes: 0,
        };
        let mut operation = ParallelOp {
            parameter_expressions: Default::default(),
            id: 1,
            tool: 1,
            spindle_rpm: 1000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            params: Default::default(),
        };
        operation.params.scallop_smoothing = true;
        operation.params.scallop_smoothing_tolerance = 0.05;
        let path: Vec<_> = (0..=100)
            .map(|i| {
                let angle = std::f64::consts::FRAC_PI_2 * i as f64 / 100.;
                [5. * angle.cos(), 5. * angle.sin(), 0.0225]
            })
            .collect();
        let fitted = fitted_arcs(&path, &linker, pass_settings(&operation), None, &cancel)
            .expect("safe planar circle should fit an arc");
        assert!(fitted
            .iter()
            .any(|s| matches!(s, cam_geo::PathSeg::Arc { .. })));
        let mut emitter = PassLinker {
            grid: &grid,
            selected: &selected,
            radius: 1.,
            error: 0.0025,
            axial: 0.,
            clearance: 10.,
            bottom: -1.,
            top: 1.,
            previous: None,
            passes: 0,
        };
        let mut emitted = StrategyResult::default();
        emit_path(
            &mut emitted,
            &mut path.clone(),
            &mut emitter,
            pass_settings(&operation),
            None,
            &cancel,
        );
        assert!(emitted
            .program
            .steps()
            .iter()
            .any(|step| matches!(step, Step::Arc { .. })));
        assert!(emitted.program.validate().is_ok());
        let mut nonplanar = path.clone();
        nonplanar[20][2] += 0.01;
        assert!(fitted_arcs(
            &nonplanar,
            &linker,
            pass_settings(&operation),
            None,
            &cancel
        )
        .is_none());
        let obstructed_mesh = [triangle, [[3., 2., 2.], [5., 2., 2.], [4., 4., 2.]]];
        let obstructed = Grid::new(&obstructed_mesh, 4., &cancel).unwrap();
        let blocked = PassLinker {
            grid: &obstructed,
            ..linker
        };
        assert!(
            fitted_arcs(&path, &blocked, pass_settings(&operation), None, &cancel).is_none(),
            "arc must not cross a protected obstacle"
        );
    }

    #[test]
    fn smoothing_respects_filter_distance_and_avoids_unsafe_shortcuts() {
        let mesh = [[[-10., -10., 0.], [30., -10., 0.], [-10., 30., 0.]]];
        let cancel = CancelToken::new();
        let grid = Grid::new(&mesh, 4., &cancel).unwrap();
        let linker = PassLinker {
            grid: &grid,
            selected: &grid,
            radius: 1.,
            error: 0.0025,
            axial: 0.,
            clearance: 10.,
            bottom: -1.,
            top: 1.,
            previous: None,
            passes: 0,
        };
        let mut operation = ParallelOp {
            parameter_expressions: Default::default(),
            id: 1,
            tool: 1,
            spindle_rpm: 1000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            params: Default::default(),
        };
        operation.params.scallop_smoothing = true;
        operation.params.scallop_smoothing_tolerance = 0.05;
        let path: Vec<_> = (0..=100)
            .map(|i| {
                let x = 1. + 8. * i as f64 / 100.;
                [x, 1. + 0.02 * x.sin(), 0.0225]
            })
            .collect();
        let reduced = simplify(&path, &linker, pass_settings(&operation), None, &cancel);
        assert!(reduced.len() < path.len() / 2);
        for point in &path {
            let d2 = reduced
                .windows(2)
                .map(|s| {
                    let v = sub(s[1], s[0]);
                    let t = (dot(sub(*point, s[0]), v) / dot(v, v)).clamp(0., 1.);
                    let d = sub(*point, add(s[0], mul(v, t)));
                    dot(d, d)
                })
                .fold(f64::INFINITY, f64::min);
            assert!(d2.sqrt() <= operation.params.scallop_smoothing_tolerance + 1e-9);
        }
    }

    #[test]
    fn shortest_link_rises_over_a_protected_obstacle() {
        let selected_mesh = [[[-10., -10., 0.], [30., -10., 0.], [-10., 30., 0.]]];
        let protected = [selected_mesh[0], [[4., 3., 4.], [6., 3., 4.], [5., 7., 4.]]];
        let cancel = CancelToken::new();
        let grid = Grid::new(&protected, 4., &cancel).unwrap();
        let selected = Grid::new(&selected_mesh, 4., &cancel).unwrap();
        let mut linker = PassLinker {
            grid: &grid,
            selected: &selected,
            radius: 1.,
            error: 0.0025,
            axial: 0.,
            clearance: 10.,
            bottom: -1.,
            top: 5.,
            previous: Some([2., 5., 0.0225]),
            passes: 0,
        };
        let mut operation = ParallelOp {
            parameter_expressions: Default::default(),
            id: 1,
            tool: 1,
            spindle_rpm: 1000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            params: Default::default(),
        };
        operation.params.scallop_retraction = ScallopRetraction::Shortest;
        operation.params.scallop_safe_distance = 1.;
        let path = vec![[8., 5., 0.0225], [8., 6., 0.0225]];
        let mut out = StrategyResult::default();
        emit_path(
            &mut out,
            &path,
            &mut linker,
            pass_settings(&operation),
            None,
            &cancel,
        );
        let links: Vec<_> = out
            .program
            .steps()
            .iter()
            .filter_map(|step| match step {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Link => Some(*to),
                _ => None,
            })
            .collect();
        assert_eq!(
            links.len(),
            2,
            "blocked direct path should use the checked raised route"
        );
        assert!(links.iter().all(|p| p.z >= 5.));
    }
}
