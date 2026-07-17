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
    // The vertices lying on a common circle is not enough: the *straight*
    // segments between them are what the machine actually cuts, and they must
    // hug the circle too. If consecutive points are so sparse that the chord
    // between them departs from the circle by more than `tol` (the sagitta), the
    // run is a polygon whose corners merely happen to be concyclic — e.g. the
    // four corners of a square — not a sampled arc. Reject it, or we would turn
    // straight edges into a bulging arc. (A finely flattened arc has a sagitta of
    // microns and sails through.)
    if chord_sagitta(radius, points[i].distance(points[i + 1])) > tol
        || chord_sagitta(radius, points[i + 1].distance(points[i + 2])) > tol
    {
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
        // The chord to the next point must hug the circle (see above).
        if chord_sagitta(radius, points[end].distance(p)) > tol {
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
    // The circumcircle of the first three points is ill-conditioned: three closely
    // spaced, grid-quantised samples throw the centre off by microns, so the run's
    // far end lands up to `tol` off that circle and the emitted G2/G3 arc has
    // mismatched start/end radii — which a strict controller (grbl) rejects. Refit
    // the centre by least squares over the *whole* run, then pin it to the
    // perpendicular bisector of the start–end chord so the emitted endpoints are
    // exactly equidistant from it: a self-consistent, machine-valid arc.
    let refined = refine_center(&points[i..=end]).unwrap_or(center);
    Some((refined, ccw, end))
}

/// Refit an arc run's centre: least squares over every point, then projected onto
/// the perpendicular bisector of the start–end chord so the emitted start and end
/// are exactly equidistant from the returned centre (a self-consistent arc).
/// `None` only if the least-squares system is singular (collinear points).
fn refine_center(run: &[Point]) -> Option<Point> {
    let c = least_squares_circle(run)?;
    let (s, e) = (run[0], run[run.len() - 1]);
    let d = Point::new(e.x - s.x, e.y - s.y);
    let dd = d.x * d.x + d.y * d.y;
    if dd < 1e-18 {
        return Some(c); // start == end (a full loop): no chord to bisect
    }
    // Project c onto the bisector {p : |p−s| = |p−e|}, which passes through the
    // chord midpoint with normal `d`. The projected point is equidistant from s, e.
    let m = Point::new((s.x + e.x) * 0.5, (s.y + e.y) * 0.5);
    let t = ((c.x - m.x) * d.x + (c.y - m.y) * d.y) / dd;
    Some(Point::new(c.x - t * d.x, c.y - t * d.y))
}

/// Algebraic (Kåsa) least-squares circle centre through `pts`: solves the 3×3
/// normal equations for `x²+y²+A·x+B·y+C = 0` and returns `(−A/2, −B/2)`. Well
/// conditioned over a whole run where a 3-point circumcircle is not. `None` if the
/// system is singular (collinear points).
fn least_squares_circle(pts: &[Point]) -> Option<Point> {
    if pts.len() < 3 {
        return None;
    }
    let n = pts.len() as f64;
    let (mut sx, mut sy, mut sxx, mut syy, mut sxy, mut sxz, mut syz, mut sz) =
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for p in pts {
        let (x, y) = (p.x, p.y);
        let z = x * x + y * y;
        sx += x;
        sy += y;
        sxx += x * x;
        syy += y * y;
        sxy += x * y;
        sxz += x * z;
        syz += y * z;
        sz += z;
    }
    let sol = solve3([[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]], [-sxz, -syz, -sz])?;
    Some(Point::new(-0.5 * sol[0], -0.5 * sol[1]))
}

/// Solve a 3×3 linear system by Cramer's rule; `None` if singular.
fn solve3(m: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = det3(m);
    if det.abs() < 1e-12 {
        return None;
    }
    let mut out = [0.0; 3];
    for (k, o) in out.iter_mut().enumerate() {
        let mut mk = m;
        for r in 0..3 {
            mk[r][k] = b[r];
        }
        *o = det3(mk) / det;
    }
    Some(out)
}

/// Determinant of a 3×3 matrix.
fn det3(m: [[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// The sagitta of a chord of length `chord` on a circle of `radius`: how far the
/// straight chord bows away from the arc. Grows with the chord, so it measures
/// how sparsely an arc has been sampled.
fn chord_sagitta(radius: f64, chord: f64) -> f64 {
    let half = chord / 2.0;
    if half >= radius {
        return radius; // a half-turn or more between samples — hopelessly coarse
    }
    radius - (radius * radius - half * half).sqrt()
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
    fn a_square_stays_lines_not_a_circle() {
        // The four corners of a square are concyclic, but the edges between them
        // are straight. Refitting must keep them as lines — never collapse the
        // square into the circumscribed circle (a real bug an inward pocket
        // offset produced).
        let pts = [
            Point::new(8.0, 32.0),
            Point::new(8.0, 8.0),
            Point::new(32.0, 8.0),
            Point::new(32.0, 32.0),
            Point::new(8.0, 32.0), // closed
        ];
        let segs = fit_arcs(&pts, 0.01);
        assert!(
            segs.iter().all(|s| matches!(s, PathSeg::Line { .. })),
            "square must stay straight edges, got {segs:?}"
        );
    }

    #[test]
    fn refitted_arcs_are_self_consistent_and_centred() {
        // A refitted arc must be a *valid* G2/G3 arc: its start and end equidistant
        // from the emitted centre, or a strict controller (grbl) rejects it. This
        // offsets a 60×40 part outward by 6 (a ⌀6 tool at offset 3) with round joins
        // — the profile case that shipped I5.992 with a 0.007 mm radius mismatch —
        // and checks every refitted corner arc. Regression: the circumcircle-of-three
        // centre was ill-conditioned; the least-squares + chord-bisector refit is not.
        use crate::{offset, Contour, JoinStyle, Polygon};
        let part = Polygon::new(Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(60.0, 0.0),
            Point::new(60.0, 40.0),
            Point::new(0.0, 40.0),
        ]))
        .unwrap();
        let grown = offset(std::slice::from_ref(&part), 6.0, JoinStyle::Round).unwrap();
        let pts = grown[0].outer().points().to_vec();
        let segs = fit_arcs(&pts, 0.01);

        // The four true corner centres (part corners), and the true radius.
        let corners = [
            Point::new(0.0, 0.0),
            Point::new(60.0, 0.0),
            Point::new(60.0, 40.0),
            Point::new(0.0, 40.0),
        ];
        let mut arcs = 0;
        // fit_arcs' first segment starts at pts[0]; each subsequent segment starts
        // where the previous one ended.
        let mut start = pts[0];
        for s in &segs {
            match *s {
                PathSeg::Line { end } => start = end,
                PathSeg::Arc { end, center, .. } => {
                    arcs += 1;
                    let r_start = center.distance(start);
                    let r_end = center.distance(end);
                    assert!(
                        (r_start - r_end).abs() < 1e-6,
                        "arc start/end radii must match: {r_start:.6} vs {r_end:.6}"
                    );
                    // And the centre sits on the true corner (radius ≈ 6.0).
                    let nearest = corners
                        .iter()
                        .map(|c| center.distance(*c))
                        .fold(f64::MAX, f64::min);
                    assert!(nearest < 1e-3, "arc centre off the corner by {nearest:.5} mm");
                    assert!((r_start - 6.0).abs() < 1e-3, "arc radius {r_start:.5} ≠ 6.0");
                    start = end;
                }
            }
        }
        assert_eq!(arcs, 4, "the four rounded corners refit to four arcs, got {arcs}");
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

