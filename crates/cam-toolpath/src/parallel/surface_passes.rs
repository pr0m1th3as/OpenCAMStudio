//! Checked links and filtering for projected finishing passes.
use super::*;
use cam_model::ScallopRetraction;

#[derive(Clone, Copy)]
pub(super) struct PassSettings {
    pub id: u32,
    pub feed: f64,
    pub plunge_feed: f64,
    pub tolerance: f64,
    pub retraction: ScallopRetraction,
    pub safe_distance: f64,
    pub max_stay_down_distance: f64,
    pub smoothing_tolerance: Option<f64>,
}

pub(super) type ChordCheck<'a> = Option<&'a dyn Fn(V, V) -> bool>;

// Remove vertices only when the entire old polyline stays within the filter
// tolerance of the replacement chord, remains ordered, and the chord passes
// the same selected-surface and swept-cutter checks as the original cut.
pub(super) fn simplify(
    path: &[V],
    linker: &PassLinker,
    settings: PassSettings,
    chord: ChordCheck,
    cancel: &CancelToken,
) -> Vec<V> {
    let Some(tolerance) = settings.smoothing_tolerance.filter(|_| path.len() >= 3) else {
        return path.to_vec();
    };
    let mut result = vec![path[0]];
    let mut first = 0;
    while first + 1 < path.len() && !cancel.is_cancelled() {
        let last = (first + 64).min(path.len() - 1);
        let mut chosen = first + 1;
        'candidate: for end in (first + 2..=last).rev() {
            let a = path[first];
            let b = path[end];
            let ab = sub(b, a);
            let length2 = dot(ab, ab);
            if length2 < 1e-20 {
                continue;
            }
            let mut previous_t = 0.;
            for point in &path[first + 1..end] {
                let t = dot(sub(*point, a), ab) / length2;
                if t < previous_t - 1e-10
                    || !(0.0..=1.0).contains(&t)
                    || dot(
                        sub(*point, add(a, mul(ab, t))),
                        sub(*point, add(a, mul(ab, t))),
                    ) > tolerance * tolerance
                {
                    continue 'candidate;
                }
                previous_t = t;
            }
            if chord.is_some_and(|check| !check(a, b))
                || !linker.selected.covers_segment(a, b, cancel)
            {
                continue;
            }
            if checked_line(a, b, linker, settings.tolerance, cancel) {
                chosen = end;
                break;
            }
        }
        result.push(path[chosen]);
        first = chosen;
    }
    result
}

fn checked_line(a: V, b: V, linker: &PassLinker, tolerance: f64, cancel: &CancelToken) -> bool {
    let mut checked = Vec::new();
    refine(
        linker.grid,
        a,
        b,
        linker.radius,
        linker.error,
        linker.error + tolerance + linker.axial,
        tolerance,
        linker.bottom,
        0,
        &mut checked,
        cancel,
    ) && checked.len() == 1
}

// Each short arc is enclosed by its endpoints and the intersection of their
// tangents. Require that whole triangle to lie in one selected CAD triangle;
// this certifies the arc cannot cross a small unselected gap between samples.
fn arc_clear(
    a: V,
    b: V,
    center: cam_geo::Point,
    ccw: bool,
    linker: &PassLinker,
    tolerance: f64,
    chord: ChordCheck,
    cancel: &CancelToken,
) -> bool {
    let radius = (a[0] - center.x).hypot(a[1] - center.y);
    if !radius.is_finite() || radius < 1e-9 {
        return false;
    }
    let start = (a[1] - center.y).atan2(a[0] - center.x);
    let end = (b[1] - center.y).atan2(b[0] - center.x);
    let sweep = if ccw {
        (end - start).rem_euclid(std::f64::consts::TAU)
    } else {
        (start - end).rem_euclid(std::f64::consts::TAU)
    };
    if sweep < 1e-9 || sweep > std::f64::consts::TAU - 1e-9 {
        return false;
    }
    // Keep the existing finite fallback when tolerance is NaN.
    #[allow(clippy::manual_clamp)]
    let max_sag = (tolerance * 0.125).min(0.005).max(1e-7);
    let max_angle = (2. * (1. - (max_sag / radius).min(1.)).acos()).min(0.1);
    let count = (sweep / max_angle).ceil() as usize;
    if count == 0 || count > 10_000 {
        return false;
    }
    let delta = sweep / count as f64;
    let sag = radius * (1. - (delta * 0.5).cos());
    let point = |angle: f64, r: f64| [center.x + r * angle.cos(), center.y + r * angle.sin(), a[2]];
    let sign = if ccw { 1. } else { -1. };
    for i in 0..count {
        if cancel.is_cancelled() {
            return false;
        }
        let aa = start + sign * delta * i as f64;
        let bb = aa + sign * delta;
        let p = if i == 0 { a } else { point(aa, radius) };
        let q = if i + 1 == count { b } else { point(bb, radius) };
        let tangent_intersection = point((aa + bb) * 0.5, radius / (delta * 0.5).cos());
        let mid = mul(add(p, q), 0.5);
        let search = (p[0] - q[0]).hypot(p[1] - q[1]) + sag + 1e-8;
        let candidates = linker.selected.indices(mid, search);
        let contained = candidates.iter().any(|&id| {
            let triangle = &linker.selected.mesh[id];
            cross(sub(triangle[1], triangle[0]), sub(triangle[2], triangle[0]))[2].abs() > 1e-12
                && inside(p, triangle)
                && inside(q, triangle)
                && inside(tangent_intersection, triangle)
        });
        let contained = contained
            || (chord.is_some()
                && covers_arc_hull(linker.selected, &[p, q, tangent_intersection], &candidates));
        if !contained
            || chord.is_some_and(|check| {
                !check(p, q) || !check(p, tangent_intersection) || !check(tangent_intersection, q)
            })
            || !linker.segment_clear_extra(p, q, sag + 1e-8, cancel)
        {
            return false;
        }
    }
    true
}

// A planar triangle seam can cross an arc even when the entire envelope is
// selected. Subtract the local triangle union before accepting that case.
fn covers_arc_hull(grid: &Grid, hull: &[V; 3], candidates: &[usize]) -> bool {
    let polygon = |triangle: &[V; 3]| {
        let ring = cam_geo::Contour::new(
            triangle
                .iter()
                .map(|v| cam_geo::Point::new(v[0], v[1]))
                .collect(),
        );
        cam_geo::Polygon::new(ring).ok()
    };
    let Some(hull) = polygon(hull) else {
        return false;
    };
    let clip: Vec<_> = candidates
        .iter()
        .filter_map(|&id| polygon(&grid.mesh[id]))
        .collect();
    !clip.is_empty() && cam_geo::difference(&[hull], &clip).is_ok_and(|missing| missing.is_empty())
}

pub(super) fn fitted_arcs(
    path: &[V],
    linker: &PassLinker,
    settings: PassSettings,
    chord: ChordCheck,
    cancel: &CancelToken,
) -> Option<Vec<cam_geo::PathSeg>> {
    let tolerance = settings.smoothing_tolerance?;
    if path.len() < 4 || path.iter().any(|p| (p[2] - path[0][2]).abs() > 1e-9) {
        return None;
    }
    let points: Vec<_> = path
        .iter()
        .map(|p| cam_geo::Point::new(p[0], p[1]))
        .collect();
    let fitted = cam_geo::fit_arcs(&points, tolerance * 0.25);
    let mut from = path[0];
    let mut arcs = 0;
    for segment in &fitted {
        match *segment {
            cam_geo::PathSeg::Line { end } => {
                let to = [end.x, end.y, from[2]];
                if chord.is_some_and(|check| {
                    !check(from, to) || !checked_line(from, to, linker, settings.tolerance, cancel)
                }) {
                    return None;
                }
                from = to;
            }
            cam_geo::PathSeg::Arc { end, center, ccw } => {
                let to = [end.x, end.y, from[2]];
                if !arc_clear(from, to, center, ccw, linker, tolerance, chord, cancel) {
                    return None;
                }
                from = to;
                arcs += 1;
            }
        }
    }
    (arcs > 0).then_some(fitted)
}

pub(super) fn emit_path(
    out: &mut StrategyResult,
    path: &[V],
    linker: &mut PassLinker,
    settings: PassSettings,
    chord: ChordCheck,
    cancel: &CancelToken,
) {
    if path.len() < 2 {
        return;
    }
    let arcs = fitted_arcs(path, linker, settings, chord, cancel);
    let simplified;
    let path = if arcs.is_some() || settings.smoothing_tolerance.is_none() {
        path
    } else {
        simplified = simplify(path, linker, settings, chord, cancel);
        &simplified
    };
    let to = path[0];
    let mut at_depth = false;
    if let Some(from) = linker.previous {
        let policy = settings.retraction;
        // Bound the spatial-index query; long or uncertain links use full clearance.
        let distance = (from[0] - to[0]).hypot(from[1] - to[1]);
        let local = distance <= settings.max_stay_down_distance.min(100. * linker.grid.size);
        if policy == ScallopRetraction::Shortest && local && linker.segment_clear(from, to, cancel)
        {
            out.program.push(Step::Linear {
                to: Point3::new(to[0], to[1], to[2]),
                feed: settings.feed,
                tag: Tag::new(settings.id, MoveKind::Link),
            });
            linker.previous = None;
            at_depth = true;
        } else if policy != ScallopRetraction::Full && local {
            let mid = mul(add(from, to), 0.5);
            let half = (from[0] - to[0]).hypot(from[1] - to[1]) * 0.5;
            let reach = linker.radius + linker.error + half;
            let highest = linker
                .grid
                .indices(mid, reach)
                .iter()
                .filter_map(|i| flat_height(mid, &linker.grid.mesh[*i], reach))
                .fold(f64::NEG_INFINITY, f64::max);
            let z = (highest + settings.safe_distance + linker.error + linker.axial)
                .max(from[2])
                .max(to[2]);
            let a = [from[0], from[1], z];
            let b = [to[0], to[1], z];
            if z.is_finite()
                && z < linker.clearance
                && linker.segment_clear(from, a, cancel)
                && linker.segment_clear(a, b, cancel)
                && linker.segment_clear(b, to, cancel)
            {
                for q in [a, b] {
                    out.program.push(Step::Linear {
                        to: Point3::new(q[0], q[1], q[2]),
                        feed: settings.feed,
                        tag: Tag::new(settings.id, MoveKind::Link),
                    });
                }
                linker.previous = None;
                at_depth = true;
                out.program.push(Step::Linear {
                    to: Point3::new(to[0], to[1], to[2]),
                    feed: settings.plunge_feed,
                    tag: Tag::new(settings.id, MoveKind::Plunge),
                });
            }
        }
    }
    if !at_depth {
        linker.retract(out, settings.id);
        out.program.push(Step::Rapid {
            to: Point3::new(to[0], to[1], linker.clearance),
            tag: Tag::new(settings.id, MoveKind::Link),
        });
        out.program.push(Step::Linear {
            to: Point3::new(to[0], to[1], to[2]),
            feed: settings.plunge_feed,
            tag: Tag::new(settings.id, MoveKind::Plunge),
        });
    }
    if let Some(segments) = arcs {
        for segment in segments {
            let tag = Tag::new(settings.id, MoveKind::Cutting);
            match segment {
                cam_geo::PathSeg::Line { end } => out.program.push(Step::Linear {
                    to: Point3::new(end.x, end.y, to[2]),
                    feed: settings.feed,
                    tag,
                }),
                cam_geo::PathSeg::Arc { end, center, ccw } => out.program.push(Step::Arc {
                    end: Point3::new(end.x, end.y, to[2]),
                    center: Point3::new(center.x, center.y, to[2]),
                    dir: if ccw {
                        cam_cldata::ArcDir::Ccw
                    } else {
                        cam_cldata::ArcDir::Cw
                    },
                    feed: settings.feed,
                    tag,
                }),
            }
        }
    } else {
        for q in path.iter().skip(1) {
            out.program.push(Step::Linear {
                to: Point3::new(q[0], q[1], q[2]),
                feed: settings.feed,
                tag: Tag::new(settings.id, MoveKind::Cutting),
            });
        }
    }
    linker.previous = path.last().copied();
    linker.passes += 1;
}
