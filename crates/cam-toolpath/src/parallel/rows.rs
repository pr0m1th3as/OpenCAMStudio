//! Cutting chains across neighboring scan rows, with checked repositioning between chains.
use super::*;
use cam_cldata::ArcDir;
use cam_model::{ParallelTransition, ScallopRetraction};
use std::ops::Range;

const MOTION_LIMIT: usize = 2_000_000;

struct Pass {
    points: Vec<V>,
    incoming: bool,
    next: Option<(usize, Turn)>,
}

enum Turn {
    Lines(Vec<V>),
    Arcs(Vec<RoundArc>),
}

struct RoundArc {
    end: V,
    center: V,
    sweep: f64,
}

#[derive(Default)]
pub(super) struct ScanRows {
    passes: Vec<Pass>,
    previous_row: Range<usize>,
    points: usize,
}

impl ScanRows {
    pub(super) fn push(
        &mut self,
        fragments: Vec<Vec<V>>,
        linker: &PassLinker,
        op: &ParallelOp,
        axis: V,
        cancel: &CancelToken,
    ) -> bool {
        let first = self.passes.len();
        self.points += fragments.iter().map(Vec::len).sum::<usize>();
        if self.points > MOTION_LIMIT {
            return false;
        }
        self.passes.extend(fragments.into_iter().map(|points| Pass {
            points,
            incoming: false,
            next: None,
        }));
        let current = first..self.passes.len();
        let transition = op.params.transition;
        if op.params.both_ways && transition != ParallelTransition::NoContact {
            // Starts on the new row face the ends on the preceding row.
            // Examine the nearest available starts along the scan direction.
            let along = |point| dot(point, axis);
            let mut starts: Vec<_> = current
                .clone()
                .map(|index| (along(self.passes[index].points[0]), index))
                .collect();
            starts.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
            for from in self.previous_row.clone() {
                if cancel.is_cancelled() {
                    break;
                }
                let end = along(*self.passes[from].points.last().unwrap());
                let split = starts.partition_point(|start| start.0 < end);
                let mut neighbors = [
                    split.checked_sub(1).and_then(|i| starts.get(i)),
                    starts.get(split),
                ];
                neighbors.sort_by(|a, b| {
                    let distance = |entry: &Option<&(f64, usize)>| {
                        entry.map_or(f64::INFINITY, |(u, _)| (u - end).abs())
                    };
                    distance(a).total_cmp(&distance(b))
                });
                let mut chosen = None;
                let mut shortest = f64::INFINITY;
                for &(_, to) in neighbors.into_iter().flatten() {
                    if self.passes[to].incoming {
                        continue;
                    }
                    let from_points = &self.passes[from].points;
                    let to_points = &self.passes[to].points;
                    if transition == ParallelTransition::Smooth {
                        if let Some(turn) =
                            smooth_turn(from_points, to_points, linker, op, axis, cancel)
                        {
                            chosen = Some((to, turn));
                            break;
                        }
                    }
                    let Some(line) = cutting_turn(from_points, to_points, linker, op, axis, cancel)
                    else {
                        continue;
                    };
                    if transition != ParallelTransition::ShortestPath {
                        chosen = Some((to, Turn::Lines(line)));
                        break;
                    }
                    let length = path_length(*from_points.last().unwrap(), &line);
                    if length < shortest {
                        shortest = length;
                        chosen = Some((to, Turn::Lines(line)));
                    }
                }
                let Some((to, turn)) = chosen else {
                    continue;
                };
                self.points += match &turn {
                    Turn::Lines(line) => line.len(),
                    Turn::Arcs(arcs) => arcs.len(),
                };
                if self.points > MOTION_LIMIT {
                    return false;
                }
                self.passes[from].next = Some((to, turn));
                self.passes[to].incoming = true;
            }
        }
        self.previous_row = current;
        true
    }

    pub(super) fn emit(
        mut self,
        out: &mut StrategyResult,
        linker: &mut PassLinker,
        op: &ParallelOp,
        retract: f64,
        cancel: &CancelToken,
    ) -> bool {
        let mut roots: Vec<_> = (0..self.passes.len())
            .filter(|&index| !self.passes[index].incoming)
            .collect();
        for first in 0..roots.len() {
            if op.params.transition == ParallelTransition::ShortestPath {
                if let Some(from) = linker.previous {
                    // Bound the search while preferring nearby machining chains.
                    let distance = |index: usize| {
                        let delta = sub(self.passes[roots[index]].points[0], from);
                        dot(delta, delta)
                    };
                    let next = (first..(first + 32).min(roots.len()))
                        .min_by(|&a, &b| distance(a).total_cmp(&distance(b)))
                        .unwrap();
                    roots.swap(first, next);
                }
            }
            let mut index = roots[first];
            reposition(
                out,
                linker,
                op,
                self.passes[index].points[0],
                retract,
                cancel,
            );
            loop {
                if cancel.is_cancelled() {
                    return false;
                }
                let pass = &mut self.passes[index];
                for &q in &pass.points[1..] {
                    feed_move(out, op, q, op.params.feed, MoveKind::Cutting);
                }
                linker.passes += 1;
                linker.previous = pass.points.last().copied();
                let Some((next, turn)) = pass.next.take() else {
                    break;
                };
                match turn {
                    Turn::Lines(line) => {
                        for q in line {
                            if linker.previous != Some(q) {
                                feed_move(out, op, q, op.params.feed, MoveKind::Cutting);
                            }
                        }
                    }
                    Turn::Arcs(arcs) => {
                        for arc in arcs {
                            out.program.push(Step::Arc {
                                end: point(arc.end),
                                center: point(arc.center),
                                dir: if arc.sweep > 0. {
                                    ArcDir::Ccw
                                } else {
                                    ArcDir::Cw
                                },
                                feed: op.params.feed,
                                tag: Tag::new(op.id, MoveKind::Cutting),
                            });
                        }
                    }
                }
                index = next;
            }
            if out.program.steps().len() > MOTION_LIMIT {
                return false;
            }
        }
        true
    }
}

fn point([x, y, z]: V) -> Point3 {
    Point3::new(x, y, z)
}

fn path_length(start: V, path: &[V]) -> f64 {
    std::iter::once(&start)
        .chain(path.iter())
        .zip(path)
        .map(|(a, b)| {
            let delta = sub(*b, *a);
            dot(delta, delta).sqrt()
        })
        .sum()
}

fn smooth_turn(
    from: &[V],
    to: &[V],
    linker: &PassLinker,
    op: &ParallelOp,
    axis: V,
    cancel: &CancelToken,
) -> Option<Turn> {
    let a = *from.last()?;
    let b = to[0];
    let outward = mul(axis, dot(sub(a, from[0]), axis).signum());
    if dot(sub(*to.last()?, b), outward) >= 0. {
        return None;
    }
    let sideways = cross(outward, sub(b, a))[2];
    let width = sideways.abs();
    let stagger = dot(sub(b, a), outward);
    if width <= 1e-9 || stagger.abs() > 3. * width {
        return None;
    }
    // Only round neighboring ends on the same side of a continuous surface.
    if ![0., 0.25, 0.5, 1.].into_iter().any(|fraction| {
        let offset = mul(outward, -fraction * linker.radius);
        linker
            .selected
            .covers_segment(add(a, offset), add(b, offset), cancel)
    }) {
        return None;
    }
    let side = mul([-outward[1], outward[0], 0.], sideways.signum());
    let sign = sideways.signum();
    let arcs = if stagger.abs() < 1e-9 {
        vec![RoundArc {
            end: b,
            center: add(a, mul(side, width * 0.5)),
            sweep: sign * std::f64::consts::PI,
        }]
    } else {
        // Two tangent arcs accommodate staggered row ends while preserving
        // the outgoing scan tangent at both boundary points.
        let chord = width.hypot(stagger);
        let first_angle = width.atan2(stagger);
        let first_radius = chord * (chord + stagger) / (2. * width);
        let second_radius = chord * (chord - stagger) / (2. * width);
        let mut joint = add(
            a,
            add(
                mul(outward, first_radius * first_angle.sin()),
                mul(side, first_radius * (1. - first_angle.cos())),
            ),
        );
        joint[2] = a[2] + (b[2] - a[2]) * first_angle / std::f64::consts::PI;
        vec![
            RoundArc {
                end: joint,
                center: add(a, mul(side, first_radius)),
                sweep: sign * first_angle,
            },
            RoundArc {
                end: b,
                center: add(b, mul(side, -second_radius)),
                sweep: sign * (std::f64::consts::PI - first_angle),
            },
        ]
    };
    let level_tangents = (from[from.len() - 2][2] - a[2]).abs() <= 1e-9
        && (to[1][2] - b[2]).abs() <= 1e-9
        && (a[2] - b[2]).abs() <= 1e-9;
    if level_tangents
        && std::iter::once(a)
            .chain(arcs.iter().map(|arc| arc.end))
            .zip(&arcs)
            .all(|(start, arc)| {
                super::surface_passes::arc_sweep_clear(
                    [start, arc.end],
                    cam_geo::Point::new(arc.center[0], arc.center[1]),
                    arc.sweep > 0.,
                    linker,
                    op.params.tolerance,
                    cancel,
                )
            })
    {
        return Some(Turn::Arcs(arcs));
    }
    project_turn(a, &arcs, linker, op, cancel).map(Turn::Lines)
}

fn project_turn(
    start: V,
    arcs: &[RoundArc],
    linker: &PassLinker,
    op: &ParallelOp,
    cancel: &CancelToken,
) -> Option<Vec<V>> {
    let margin = linker.error + op.params.tolerance + linker.axial;
    let mut previous = start;
    let mut points = Vec::new();
    for (index, arc) in arcs.iter().enumerate() {
        let [dx, dy, _] = sub(previous, arc.center);
        let radius = dx.hypot(dy);
        let sag = op.params.tolerance * 0.125;
        let angle = (2. * (1. - (sag / radius).min(1.)).acos()).min(0.2);
        let count = (arc.sweep.abs() / angle).ceil() as usize;
        if !(1..=4096).contains(&count) {
            return None;
        }
        let initial_z = previous[2];
        for i in 1..=count {
            if cancel.is_cancelled() {
                return None;
            }
            let t = i as f64 / count as f64;
            let mut q = if i == count {
                arc.end
            } else {
                let (sin, cos) = (arc.sweep * t).sin_cos();
                [
                    arc.center[0] + cos * dx - sin * dy,
                    arc.center[1] + sin * dx + cos * dy,
                    0.,
                ]
            };
            // Keep the turn at or above the end heights interpolated along
            // it. A pocket or void outside the selected face must not pull
            // the cutter down during the turn.
            if index + 1 < arcs.len() || i < count {
                let floor = initial_z + (arc.end[2] - initial_z) * t;
                q[2] = floor.max(
                    row_tip_height(
                        linker.grid,
                        q,
                        linker.radius,
                        linker.error,
                        margin,
                        linker.bottom,
                    )
                    .unwrap_or(floor),
                );
            }
            if q[2] > linker.top + margin
                || q[2] >= linker.clearance
                || !sweep_clear(linker, previous, q, cancel)
            {
                return None;
            }
            points.push(q);
            previous = q;
        }
    }
    Some(points)
}

fn cutting_turn(
    from: &[V],
    to: &[V],
    linker: &PassLinker,
    op: &ParallelOp,
    axis: V,
    cancel: &CancelToken,
) -> Option<Vec<V>> {
    let a = *from.last()?;
    let b = to[0];
    let delta = sub(a, b);
    // Across an adjacent row the stepover is fixed. Allow the edge to shift
    // along the row as it curves, while keeping the turning cut local.
    if delta[0].hypot(delta[1]) > (1.5 * op.params.stepover).max(4. * linker.radius) {
        return None;
    }
    let back = dot(sub(from[0], a), axis);
    let forward = dot(sub(*to.last()?, b), axis);
    if back.signum() != forward.signum() {
        return None;
    }
    let prefer_shortest = op.params.transition == ParallelTransition::ShortestPath;
    if !prefer_shortest && op.params.containment == AdaptiveContainment::Center {
        let mut points = Vec::new();
        let outward = mul(axis, -back.signum());
        return follow_boundary([a, b], outward, linker, op, 0, &mut points, cancel)
            .then_some(points);
    }
    let chord_supported = linker.selected.covers_segment(a, b, cancel)
        || (op.params.containment != AdaptiveContainment::Center
            // In modes that allow overhang, the selected surface can lie
            // under the edge of the ball rather than under its centre.
            && [0.25, 0.5, 0.75, 1.].into_iter().any(|fraction| {
                let offset = mul(axis, back.signum() * linker.radius * fraction);
                linker.selected.covers_segment(add(a, offset), add(b, offset), cancel)
            }));
    if prefer_shortest && chord_supported && sweep_clear(linker, a, b, cancel) {
        return Some(vec![b]);
    }
    let mut shortest = None;
    if chord_supported {
        if let Some(turn) = follow_surface(a, b, linker, op, cancel) {
            if !prefer_shortest {
                return Some(turn);
            }
            shortest = Some(turn);
        }
    }
    if !prefer_shortest {
        return None;
    }

    // At a steep rim the endpoint chord can meet a wall between rows. Move a
    // short way back along the existing cut before stepping over. The return
    // paths have already passed the row sweep checks.
    let along = |point| dot(point, axis);
    for inset in [linker.radius, 2. * linker.radius, 3. * linker.radius] {
        let from_inset = from
            .iter()
            .rev()
            .position(|q| (along(*q) - along(a)).abs() >= inset);
        let to_inset = to
            .iter()
            .position(|q| (along(*q) - along(b)).abs() >= inset);
        if let (Some(back_steps), Some(to_index)) = (from_inset, to_inset) {
            let from_index = from.len() - back_steps - 1;
            let inside_a = from[from_index];
            let inside_b = to[to_index];
            if linker.selected.covers_segment(inside_a, inside_b, cancel) {
                if let Some(turn) = follow_surface(inside_a, inside_b, linker, op, cancel) {
                    let route: Vec<_> = from[from_index..from.len() - 1]
                        .iter()
                        .rev()
                        .copied()
                        .chain(turn)
                        .chain(to[..to_index].iter().rev().copied())
                        .collect();
                    if shortest
                        .as_ref()
                        .is_none_or(|best| path_length(a, &route) < path_length(a, best))
                    {
                        shortest = Some(route);
                    }
                }
            }
        }
    }
    shortest
}

fn follow_boundary(
    [a, b]: [V; 2],
    outward: V,
    linker: &PassLinker,
    op: &ParallelOp,
    depth: usize,
    points: &mut Vec<V>,
    cancel: &CancelToken,
) -> bool {
    if cancel.is_cancelled() {
        return false;
    }
    if linker.selected.covers_segment(a, b, cancel) {
        if let Some(path) = follow_surface(a, b, linker, op, cancel) {
            points.extend(path);
            return true;
        }
    }
    if depth == 10 {
        return false;
    }
    // Find the same edge in the cross-section halfway between these points.
    // A small inward allowance keeps the chords on the selected footprint
    // around a concave edge. The row endpoints themselves stay untouched.
    let mid = mul(add(a, b), 0.5);
    let reach = op.params.stepover.max(linker.radius);
    let start = add(mid, mul(outward, -reach));
    let end = add(mid, mul(outward, reach));
    let Some(spans) = linker
        .selected
        .covered_intervals(start, end, |_| true, cancel)
    else {
        return false;
    };
    let Some((lo, hi)) = spans
        .into_iter()
        .filter(|&(lo, hi)| hi < 1. && hi > lo)
        .min_by(|a, b| (a.1 - 0.5).abs().total_cmp(&(b.1 - 0.5).abs()))
    else {
        return false;
    };
    let inset = (op.params.tolerance * 0.125).min(0.005) / (2. * reach);
    let mut q = add(
        start,
        mul(sub(end, start), hi - inset.min((hi - lo) * 0.25)),
    );
    let margin = linker.error + op.params.tolerance + linker.axial;
    let Some(z) = row_tip_height(
        linker.grid,
        q,
        linker.radius,
        linker.error,
        margin,
        linker.bottom,
    ) else {
        return false;
    };
    q[2] = z;
    z <= linker.top + margin
        && z < linker.clearance
        && follow_boundary([a, q], outward, linker, op, depth + 1, points, cancel)
        && follow_boundary([q, b], outward, linker, op, depth + 1, points, cancel)
}

fn follow_surface(
    a: V,
    b: V,
    linker: &PassLinker,
    op: &ParallelOp,
    cancel: &CancelToken,
) -> Option<Vec<V>> {
    let margin = linker.error + op.params.tolerance + linker.axial;
    let mut path = Vec::new();
    if refine(
        linker.grid,
        a,
        b,
        linker.radius,
        linker.error,
        margin,
        op.params.tolerance,
        linker.bottom,
        0,
        &mut path,
        cancel,
    ) && path
        .iter()
        .all(|q| q[2] <= linker.top + margin && q[2] < linker.clearance)
    {
        Some(path)
    } else {
        // A safe chord can bridge shallow departures from the projected
        // surface where sampling a continuous contact curve is impossible.
        linker.segment_clear(a, b, cancel).then(|| vec![b])
    }
}

fn feed_move(out: &mut StrategyResult, op: &ParallelOp, q: V, feed: f64, kind: MoveKind) {
    out.program.push(Step::Linear {
        to: point(q),
        feed,
        tag: Tag::new(op.id, kind),
    });
}

fn rapid(out: &mut StrategyResult, op: &ParallelOp, q: V, kind: MoveKind) {
    out.program.push(Step::Rapid {
        to: point(q),
        tag: Tag::new(op.id, kind),
    });
}

fn reposition(
    out: &mut StrategyResult,
    linker: &mut PassLinker,
    op: &ParallelOp,
    to: V,
    retract: f64,
    cancel: &CancelToken,
) {
    let p = op.params;
    let no_contact = p.transition == ParallelTransition::NoContact;
    let plunge = |out: &mut StrategyResult| feed_move(out, op, to, p.plunge_feed, MoveKind::Plunge);
    if let Some(from) = linker.previous.take() {
        // Bound the work of the spatial queries for extremely distant jumps.
        let local = (from[0] - to[0]).hypot(from[1] - to[1]) <= 100. * linker.grid.size;
        if !no_contact
            && p.retraction == ScallopRetraction::Shortest
            && local
            && sweep_clear(linker, from, to, cancel)
        {
            // A linear feed is predictable on controllers with dogleg rapids.
            feed_move(out, op, to, p.feed, MoveKind::Link);
            return;
        }
        if p.retraction != ScallopRetraction::Full && local {
            let z = minimum_height(linker, from, to, p.safe_distance, cancel);
            if let Some(z) = z.filter(|&z| !no_contact || z > from[2].max(to[2]) + 1e-9) {
                if z > from[2] {
                    feed_move(out, op, [from[0], from[1], z], p.feed, MoveKind::Link);
                }
                feed_move(out, op, [to[0], to[1], z], p.feed, MoveKind::Link);
                if z > to[2] {
                    plunge(out);
                }
                return;
            }
        }
        rapid(out, op, [from[0], from[1], retract], MoveKind::Retract);
        rapid(out, op, [to[0], to[1], retract], MoveKind::Link);
    } else {
        rapid(out, op, [to[0], to[1], linker.clearance], MoveKind::Link);
    }
    plunge(out);
}

fn minimum_height(
    linker: &PassLinker,
    from: V,
    to: V,
    safe_distance: f64,
    cancel: &CancelToken,
) -> Option<f64> {
    let over = |z| sweep_clear(linker, [from[0], from[1], z], [to[0], to[1], z], cancel);
    let mut low = from[2].max(to[2]);
    let mut high = linker.clearance;
    if over(low) {
        high = low;
    } else {
        // The swept horizontal envelope clears monotonically as it rises.
        for _ in 0..32 {
            if cancel.is_cancelled() || high - low <= 0.001 {
                break;
            }
            let mid = (low + high) * 0.5;
            if over(mid) {
                high = mid;
            } else {
                low = mid;
            }
        }
    }
    let z = high + safe_distance;
    (z < linker.clearance
        && !cancel.is_cancelled()
        && over(z)
        && linker.segment_clear(from, [from[0], from[1], z], cancel)
        && linker.segment_clear([to[0], to[1], z], to, cancel))
    .then_some(z)
}

fn sweep_clear(linker: &PassLinker, a: V, b: V, cancel: &CancelToken) -> bool {
    let reach = linker.radius + linker.error;
    let middle = mul(add(a, b), 0.5);
    let half_distance = (a[0] - b[0]).hypot(a[1] - b[1]) * 0.5;
    let ca = add(a, [0., 0., linker.radius - linker.axial]);
    let cb = add(b, [0., 0., linker.radius - linker.axial]);
    let cylinder_plane = ca[2].min(cb[2]) - linker.error;
    let a_xy = [a[0], a[1], 0.];
    let b_xy = [b[0], b[1], 0.];
    linker
        .grid
        .indices(middle, reach + half_distance)
        .into_iter()
        .all(|index| {
            let triangle = &linker.grid.mesh[index];
            !cancel.is_cancelled()
                && segment_triangle(ca, cb, triangle) >= reach * reach
                && cylinder_clear(a_xy, b_xy, triangle, cylinder_plane, reach)
        })
}

fn cylinder_clear(a: V, b: V, triangle: &[V; 3], plane: f64, reach: f64) -> bool {
    // Clip the triangle above the ball centre, then check its XY projection
    // against the swept cylinder. At most four vertices survive the clip.
    let mut polygon = [[0.; 3]; 4];
    let mut len = 0;
    for i in 0..3 {
        let p = triangle[i];
        let q = triangle[(i + 1) % 3];
        if p[2] > plane {
            polygon[len] = [p[0], p[1], 0.];
            len += 1;
        }
        if (p[2] > plane) != (q[2] > plane) {
            let cut = add(p, mul(sub(q, p), (plane - p[2]) / (q[2] - p[2])));
            polygon[len] = [cut[0], cut[1], 0.];
            len += 1;
        }
    }
    (1..len.saturating_sub(1))
        .all(|i| segment_triangle(a, b, &[polygon[0], polygon[i], polygon[i + 1]]) > reach * reach)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shortest_transition_uses_a_direct_chord_when_the_surface_dips() {
        let strip = |y0, y1, z0, z1| {
            let [a, b, c, d] = [[-4., y0, z0], [1., y0, z0], [1., y1, z1], [-4., y1, z1]];
            [[a, b, c], [a, c, d]]
        };
        let surface = [strip(0., 2., 1., 0.), strip(2., 4., 0., 1.)].concat();
        let cancel = CancelToken::new();
        let grid = Grid::new(&surface, 2., &cancel).unwrap();
        let linker = PassLinker {
            grid: &grid,
            selected: &grid,
            radius: 1.,
            error: 0.0025,
            axial: 0.,
            clearance: 10.,
            bottom: -1.,
            top: 2.,
            previous: None,
            passes: 0,
        };
        let mut op = super::super::tests::op();
        op.params.stepover = 4.;
        let z = 1. + linker.error + op.params.tolerance;
        let from = [[-3., 0., z], [0., 0., z]];
        let to = [[0., 4., z], [-3., 4., z]];
        let straight = cutting_turn(&from, &to, &linker, &op, [1., 0., 0.], &cancel).unwrap();
        assert!(straight.iter().any(|p| p[2] < 0.8));
        op.params.transition = ParallelTransition::ShortestPath;
        let shortest = cutting_turn(&from, &to, &linker, &op, [1., 0., 0.], &cancel).unwrap();
        assert_eq!(shortest, [to[0]]);
        assert!(path_length(from[1], &shortest) < path_length(from[1], &straight));
    }

    #[test]
    fn cutting_turn_can_step_inward_to_bypass_a_blocked_rim() {
        let selected = [
            [[-4., -1., 0.], [1., -1., 0.], [1., 5., 0.]],
            [[-4., -1., 0.], [1., 5., 0.], [-4., 5., 0.]],
        ];
        let mut triangles = selected.to_vec();
        triangles.push([[-0.1, 2., 0.], [0.1, 2., 0.], [0., 2., 4.]]);
        let cancel = CancelToken::new();
        let grid = Grid::new(&triangles, 2., &cancel).unwrap();
        let selected = Grid::new(&selected, 2., &cancel).unwrap();
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
        let mut op = super::super::tests::op();
        op.params.stepover = 4.;
        let z = linker.error + op.params.tolerance;
        let from = [-3., -2., -1., 0.].map(|x| [x, 0., z]);
        let to = [0., -1., -2., -3.].map(|x| [x, 4., z]);
        assert!(follow_surface(from[3], to[0], &linker, &op, &cancel).is_none());
        assert!(cutting_turn(&from, &to, &linker, &op, [1., 0., 0.], &cancel).is_none());
        op.params.transition = ParallelTransition::ShortestPath;
        let turn = cutting_turn(&from, &to, &linker, &op, [1., 0., 0.], &cancel).unwrap();
        assert!(turn.iter().any(|p| p[0] <= -2. && p[1] == 4.));
        assert_eq!(turn.last(), Some(&to[0]));
        assert!(turn.iter().all(|p| p[2] == z));
    }

    #[test]
    fn straight_turn_follows_a_concave_boundary_without_retracing_a_row() {
        let lower = [[-4., 0., 0.], [1., 0., 0.], [0., 1., 0.], [-4., 1., 0.]];
        let upper = [[-4., 1., 0.], [0., 1., 0.], [1., 2., 0.], [-4., 2., 0.]];
        let triangles = [lower, upper]
            .map(|[a, b, c, d]| [[a, b, c], [a, c, d]])
            .concat();
        let cancel = CancelToken::new();
        let grid = Grid::new(&triangles, 2., &cancel).unwrap();
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
        let mut op = super::super::tests::op();
        op.params.containment = AdaptiveContainment::Center;
        op.params.stepover = 2.;
        let z = linker.error + op.params.tolerance;
        let from = [[-3., 0., z], [1., 0., z]];
        let to = [[1., 2., z], [-3., 2., z]];
        assert!(!grid.covers_segment(from[1], to[0], &cancel));
        let path = cutting_turn(&from, &to, &linker, &op, [1., 0., 0.], &cancel).unwrap();
        assert_eq!(path.last(), Some(&to[0]));
        assert!(path.len() > 1);
        for (a, b) in std::iter::once(&from[1]).chain(&path).zip(&path) {
            assert!(b[1] > a[1], "turn retraced a cutting row: {a:?} to {b:?}");
            assert!(grid.covers_segment(*a, *b, &cancel));
            assert!(((b[1] - 1.).abs() - b[0]).abs() < 0.003);
        }
    }

    #[test]
    fn smooth_turn_joins_staggered_row_ends_with_two_tangent_arcs() {
        let [a, b, c, d] = [
            [-4., -1., 0.],
            [-0.5, -1., 0.],
            [1.5, 3., 0.],
            [-4., 3., 0.],
        ];
        let triangles = [[a, b, c], [a, c, d]];
        let cancel = CancelToken::new();
        let grid = Grid::new(&triangles, 2., &cancel).unwrap();
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
        let mut op = super::super::tests::op();
        op.params.containment = AdaptiveContainment::Center;
        let z = linker.error + op.params.tolerance;
        let from = [[-3., 0., z], [0., 0., z]];
        let to = [[0.5, 1., z], [-3., 1., z]];
        let Turn::Arcs(arcs) =
            smooth_turn(&from, &to, &linker, &op, [1., 0., 0.], &cancel).unwrap()
        else {
            panic!("expected native arcs");
        };
        let [first, second] = arcs.as_slice() else {
            panic!("expected two arcs");
        };
        assert_eq!(first.center[0], from[1][0]);
        assert_eq!(second.center[0], to[0][0]);
        assert_eq!(second.end, to[0]);
        let joint = first.end;
        let radial_a = sub(joint, first.center);
        let radial_b = sub(joint, second.center);
        assert!(cross(radial_a, radial_b)[2].abs() < 1e-9 && dot(radial_a, radial_b) > 0.);
        assert!(
            joint[0] > 0.5 * joint[1],
            "the turn should be beyond the boundary"
        );
    }

    #[test]
    fn smooth_rejects_an_outward_turn_blocked_by_an_unselected_wall() {
        let [a, b, c, d] = [[-4., -1., 0.], [0., -1., 0.], [0., 2., 0.], [-4., 2., 0.]];
        let selected = [[a, b, c], [a, c, d]];
        let mut triangles = selected.to_vec();
        triangles.push([[0.45, 0.45, 0.], [0.45, 0.55, 0.], [0.45, 0.5, 4.]]);
        let cancel = CancelToken::new();
        let grid = Grid::new(&triangles, 0.2, &cancel).unwrap();
        let selected = Grid::new(&selected, 0.2, &cancel).unwrap();
        let linker = PassLinker {
            grid: &grid,
            selected: &selected,
            radius: 0.1,
            error: 0.0025,
            axial: 0.,
            clearance: 10.,
            bottom: -1.,
            top: 1.,
            previous: None,
            passes: 0,
        };
        let mut op = super::super::tests::op();
        op.params.containment = AdaptiveContainment::Center;
        op.params.transition = ParallelTransition::Smooth;
        let z = linker.error + op.params.tolerance;
        let from = [[-3., 0., z], [0., 0., z]];
        let to = [[0., 1., z], [-3., 1., z]];
        assert!(smooth_turn(&from, &to, &linker, &op, [1., 0., 0.], &cancel).is_none());
        let straight = cutting_turn(&from, &to, &linker, &op, [1., 0., 0.], &cancel).unwrap();
        assert_eq!(straight.last(), Some(&to[0]));
        assert!(straight.iter().all(|q| q[0] == 0. && q[2] == z));
    }

    #[test]
    fn shortest_requires_a_raise_over_a_ridge_and_minimum_ignores_off_path_geometry() {
        let obstacles = [
            [[0., -0.1, 0.], [0., 0.1, 0.], [0., 0., 2.5]],
            [[0., 2.5, 3.], [0.2, 2.5, 6.], [0., 3., 6.]],
        ];
        let cancel = CancelToken::new();
        let grid = Grid::new(&obstacles, 2., &cancel).unwrap();
        let linker = PassLinker {
            grid: &grid,
            selected: &grid,
            radius: 1.,
            error: 0.0025,
            axial: 0.,
            clearance: 8.,
            bottom: 0.,
            top: 6.,
            previous: None,
            passes: 0,
        };
        let from = [-4., 0., 0.1];
        let to = [4., 0., 0.1];
        assert!(!sweep_clear(&linker, from, to, &cancel));
        let z = minimum_height(&linker, from, to, 0.5, &cancel).unwrap();
        assert!((3.0025..3.004).contains(&z), "unexpected minimum lift {z}");
        assert!(minimum_height(&linker, from, to, 6., &cancel).is_none());
    }
}
