//! Arc refitting — recognising runs of points that lie on a circle and
//! collapsing them into a single arc.
//!
//! Offsetting flattens round-join corners into many short segments. This walks a
//! polyline and, wherever a run of points lies on a common circle within `tol`,
//! replaces it with one [`PathSeg::Arc`]; everything else stays a
//! [`PathSeg::Line`]. It is deliberately conservative — an arc is only emitted
//! when the points genuinely fit — so the result never deviates from the input
//! by more than `tol`.

use crate::Point;

/// One reconstructed path segment, defined by its end point (the start is the
/// previous segment's end, or the polyline's first point).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PathSeg {
    /// A straight segment to `end`.
    Line { end: Point },
    /// A circular arc to `end` about the absolute `center`, `ccw` or not.
    Arc {
        end: Point,
        center: Point,
        ccw: bool,
    },
}

/// Radii larger than this are treated as straight lines (a near-collinear run
/// fits a huge circle that is not a meaningful arc).
const MAX_ARC_RADIUS: f64 = 1.0e5;

/// Fit arcs to a polyline, returning the reconstructed segments in order.
pub fn fit_arcs(points: &[Point], tol: f64) -> Vec<PathSeg> {
    let n = points.len();
    let mut segs = Vec::new();
    if n < 2 {
        return segs;
    }

    let mut i = 0;
    while i + 1 < n {
        if let Some((center, ccw, end_idx)) = grow_arc(points, i, tol) {
            segs.push(PathSeg::Arc {
                end: points[end_idx],
                center,
                ccw,
            });
            i = end_idx;
            continue;
        }
        segs.push(PathSeg::Line { end: points[i + 1] });
        i += 1;
    }
    segs
}

/// Try to grow an arc starting at index `i`. Returns the arc's centre,
/// direction, and the index of its last point (spanning at least three points)
/// if one fits, else `None`.
fn grow_arc(points: &[Point], i: usize, tol: f64) -> Option<(Point, bool, usize)> {
    if i + 2 >= points.len() {
        return None;
    }
    let (center, radius) = circumcircle(points[i], points[i + 1], points[i + 2])?;
    if radius > MAX_ARC_RADIUS {
        return None;
    }
    let ccw = orient(points[i], points[i + 1], points[i + 2]) > 0.0;

    let mut end = i + 2;
    while end + 1 < points.len() {
        let p = points[end + 1];
        if (center.distance(p) - radius).abs() > tol {
            break;
        }
        // The turn must keep the same sense, so the arc stays monotonic.
        if (orient(points[end - 1], points[end], p) > 0.0) != ccw {
            break;
        }
        end += 1;
    }
    // Require a genuine run (≥ 4 points).
    if end < i + 3 {
        return None;
    }
    // Reject near-straight runs: a real arc bulges from its start–end chord by
    // much more than the fit tolerance, whereas a straight edge meeting an arc
    // fits a huge, near-flat spurious circle that does not.
    let (a, b) = (points[i], points[end]);
    let bulge = points[i + 1..end]
        .iter()
        .map(|p| perp_distance(a, b, *p))
        .fold(0.0_f64, f64::max);
    if bulge <= 2.0 * tol {
        return None;
    }
    Some((center, ccw, end))
}

/// Perpendicular distance from `p` to the line through `a` and `b`.
fn perp_distance(a: Point, b: Point, p: Point) -> f64 {
    let len = a.distance(b);
    if len < 1e-12 {
        return a.distance(p);
    }
    (orient(a, b, p) / len).abs()
}

/// The circumcircle of three points, or `None` if they are collinear.
fn circumcircle(a: Point, b: Point, c: Point) -> Option<(Point, f64)> {
    let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if d.abs() < 1e-9 {
        return None;
    }
    let (a2, b2, c2) = (
        a.x * a.x + a.y * a.y,
        b.x * b.x + b.y * b.y,
        c.x * c.x + c.y * c.y,
    );
    let ux = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
    let uy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
    let center = Point::new(ux, uy);
    Some((center, center.distance(a)))
}

/// Twice the signed area of triangle `abc`: positive counter-clockwise.
fn orient(a: Point, b: Point, c: Point) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Arc;

    #[test]
    fn straight_points_stay_lines() {
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(3.0, 0.0),
        ];
        let segs = fit_arcs(&pts, 0.01);
        assert!(segs.iter().all(|s| matches!(s, PathSeg::Line { .. })));
        assert_eq!(segs.len(), 3);
    }

    #[test]
    fn a_flattened_quarter_arc_refits_to_one_arc() {
        // Flatten a quarter circle finely, then refit it.
        let arc = Arc::new(
            Point::new(0.0, 0.0),
            5.0,
            0.0,
            std::f64::consts::FRAC_PI_2,
            true,
        );
        let pts = arc.flatten(1e-3);
        let segs = fit_arcs(&pts, 0.05);
        assert_eq!(segs.len(), 1, "one arc, got {segs:?}");
        match segs[0] {
            PathSeg::Arc { center, ccw, end } => {
                assert!(center.distance(Point::new(0.0, 0.0)) < 0.05);
                assert!(ccw);
                assert!(end.distance(Point::new(0.0, 5.0)) < 0.05);
            }
            _ => panic!("expected an arc"),
        }
    }

    #[test]
    fn line_then_arc_is_split() {
        // A straight run into a quarter arc: the line stays a line, the arc refits.
        let mut pts = vec![Point::new(-10.0, 0.0), Point::new(0.0, 0.0)];
        let arc = Arc::new(
            Point::new(0.0, 5.0),
            5.0,
            -std::f64::consts::FRAC_PI_2,
            0.0,
            true,
        );
        pts.extend(arc.flatten(1e-3).into_iter().skip(1));
        let segs = fit_arcs(&pts, 0.05);
        assert!(matches!(segs[0], PathSeg::Line { .. }), "{segs:?}");
        assert!(
            segs.iter()
                .filter(|s| matches!(s, PathSeg::Arc { .. }))
                .count()
                == 1,
            "one arc: {segs:?}"
        );
    }
}
