//! Constant-engagement adaptive clearing — the generator.
//!
//! Front-advance peeling driven by the cleared-region model: seed the entry disc,
//! then repeatedly take the next tool-centre loop as the cleared region shrunk by
//! `radius − engagement` (clamped to the tool-centre-accessible region), which cuts
//! a fresh band of width ≈ `engagement` beyond the current front. The whole path is
//! then **certified** against the oracle ([`crate::clearsim::certify`]); it is
//! returned only if it holds engagement at the cap, covers the reachable target, and
//! never gouges. The caller falls back to concentric clearing on `None`.
//!
//! Scope today: a single simply-connected region with the entry inside it. Islands,
//! multi-lobe regions, and sharp corners whose engagement spikes are not certified
//! and fall back. Corner-loop insertion (to certify sharp corners) and islands are
//! the next phases — the oracle already guarantees every emitted path is correct
//! regardless, so this grows without ever shipping a bad toolpath.
// Consumed by the clearing pipeline once it certifies paths (spiral connection is
// the next step); the self-certification contract is exercised by tests now.
#![allow(dead_code)]

use cam_geo::{
    difference, intersection, offset, stroke_path, union, CapStyle, Contour, JoinStyle, Point,
    Polygon, Polyline,
};

use crate::clearsim::{certify, reachable};

/// Chord tolerance for flattening the entry disc (mm).
const FLAT_TOL: f64 = 0.05;
/// Angular samples per revolution when morphing loops into a spiral.
const ANGULAR_SAMPLES: usize = 32;
/// Cap on peel iterations (guards the front-advance loop).
const MAX_PASSES: usize = 400;
/// Cap on the generated path length. Certifying peak engagement is sequential
/// (O(n²) in the naïve cleared-region model), so a path longer than this bails to
/// the concentric fallback rather than spend the time. A raster engagement model
/// (next phase) lifts this so large pockets can go adaptive too.
const MAX_PATH: usize = 200;

/// Net area of a set of polygons.
fn area(polys: &[Polygon]) -> f64 {
    polys.iter().map(Polygon::area).sum()
}

/// The largest polygon by area (the main body of a boolean result).
fn largest(polys: Vec<Polygon>) -> Option<Polygon> {
    polys
        .into_iter()
        .max_by(|a, b| a.area().partial_cmp(&b.area()).unwrap_or(std::cmp::Ordering::Equal))
}

/// A filled disc of radius `r` at `c`.
fn disc(c: Point, r: f64) -> Option<Polygon> {
    let pts = cam_geo::Arc::circle(c, r).flatten(FLAT_TOL);
    Polygon::new(Contour::new(pts)).ok()
}

/// Centroid of a contour's vertices.
fn centroid(poly: &Polygon) -> Point {
    let pts = poly.outer().points();
    let n = pts.len().max(1) as f64;
    let (sx, sy) = pts.iter().fold((0.0, 0.0), |(sx, sy), p| (sx + p.x, sy + p.y));
    Point::new(sx / n, sy / n)
}

/// Generate a certified constant-engagement tool-centre path that clears `region`
/// (leaving `finish` skin on the walls) with a tool of radius `r` at engagement cap
/// `e`. `start` is the preferred entry (part XY). Returns `None` — meaning *fall back
/// to concentric* — whenever it cannot certify.
pub(crate) fn adaptive_path(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
) -> Option<Vec<Point>> {
    // Engagement must be a real cap below the full width of cut.
    if !(e > 0.0 && e < 2.0 * r) {
        return None;
    }
    // Islands are a later phase.
    if !region.holes().is_empty() {
        return None;
    }
    // Material to remove (skin left on the walls) and the tool-centre region.
    let to_clear = largest(offset(std::slice::from_ref(region), -finish, JoinStyle::Round).ok()?)?;
    if !to_clear.holes().is_empty() {
        return None;
    }
    let rc_polys = offset(std::slice::from_ref(region), -(r + finish), JoinStyle::Round).ok()?;
    let rc = largest(rc_polys)?;

    // Entry point: the operator's pick if it is inside the tool-centre region, else
    // the region centroid (fall back if even that is outside — a pinched region).
    let entry = start
        .map(|s| Point::new(s[0], s[1]))
        .filter(|p| rc.contains(*p))
        .unwrap_or_else(|| centroid(&rc));
    if !rc.contains(entry) {
        return None;
    }

    let reach = reachable(&to_clear, r);
    if reach.is_empty() {
        return None;
    }
    let cover_tol = 0.02 * area(&reach) + 1.0;

    // Seed the cleared region with the entry disc, then collect the successive
    // tool-centre loops (each cuts a band of ≈ e beyond the current front).
    let mut cleared: Vec<Polygon> = vec![disc(entry, r)?];
    let mut loops: Vec<Vec<Point>> = Vec::new();
    let mut last_uncut = f64::INFINITY;

    for _ in 0..MAX_PASSES {
        // Bail to the concentric fallback before the sequential certify gets costly.
        if loops.len() * ANGULAR_SAMPLES > MAX_PATH {
            return None;
        }
        let remaining = area(&difference(&reach, &cleared).ok()?);
        if remaining <= cover_tol {
            break;
        }
        // If a pass fails to make real progress, we are stuck (a pinch, a corner the
        // peel can't reach) — bail so the caller falls back rather than spin.
        if remaining >= last_uncut - 0.01 * area(&reach) {
            return None;
        }
        last_uncut = remaining;
        // Next tool-centre loop: the cleared region pulled in by (r − e) cuts a fresh
        // band of width ≈ e beyond the front; clamp to the tool-centre region so the
        // tool never crosses the wall.
        let front = offset(&cleared, -(r - e), JoinStyle::Round).ok()?;
        let clamped = intersection(&front, std::slice::from_ref(&rc)).ok()?;
        let loop_poly = largest(clamped)?;
        let pts = loop_poly.outer().points();
        if pts.len() < 3 {
            return None;
        }
        loops.push(pts.to_vec());

        // Grow the cleared region by this loop's sweep.
        let mut loop_path = pts.to_vec();
        loop_path.push(pts[0]);
        let swept = stroke_path(&Polyline::new(loop_path), r, CapStyle::Round, JoinStyle::Round).ok()?;
        if swept.is_empty() {
            return None;
        }
        cleared = union(&cleared, &swept).ok()?;
    }

    // Join the concentric loops into a continuous spiral so the radius grows a band
    // per revolution rather than jumping radially between loops — the radial links
    // were what spiked the engagement between passes.
    let path = spiral(&loops, entry)?;

    // The whole path must certify: engagement ≤ cap, reachable target covered, no gouge.
    let verdict = certify(&path, r, &to_clear);
    verdict.certified(e, cover_tol).then_some(path)
}

/// The farthest positive-`s` intersection of the ray `c + s·d` (unit `d`) with the
/// closed loop `pts`, as a point. `None` if the ray misses (a non-star-convex loop
/// seen from `c`, which is not spiral-morphable — the caller falls back).
fn ray_hit(c: Point, d: (f64, f64), pts: &[Point]) -> Option<Point> {
    let n = pts.len();
    let mut best_s = -1.0;
    let mut best = None;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let (ex, ey) = (b.x - a.x, b.y - a.y);
        let det = ex * d.1 - ey * d.0;
        if det.abs() < 1e-12 {
            continue;
        }
        let (rx, ry) = (a.x - c.x, a.y - c.y);
        let s = (ex * ry - ey * rx) / det;
        let u = (d.0 * ry - d.1 * rx) / det;
        if (0.0..=1.0).contains(&u) && s > 1e-9 && s > best_s {
            best_s = s;
            best = Some(Point::new(c.x + d.0 * s, c.y + d.1 * s));
        }
    }
    best
}

/// Resample a closed loop to `n` points at even angles about `center` (ray-cast).
fn resample_by_angle(pts: &[Point], center: Point, n: usize) -> Option<Vec<Point>> {
    (0..n)
        .map(|i| {
            let a = std::f64::consts::TAU * (i as f64) / (n as f64);
            ray_hit(center, (a.cos(), a.sin()), pts)
        })
        .collect()
}

/// Morph the concentric `loops` (inner-first) into one continuous spiral about
/// `center`: within each revolution the point blends from loop `k` to loop `k+1`,
/// so the radius grows a band per turn with no radial jump. Ends with a full lap of
/// the outermost loop to finish the wall. Returns `None` if any loop is not
/// star-convex from `center` (not spiral-morphable).
fn spiral(loops: &[Vec<Point>], center: Point) -> Option<Vec<Point>> {
    if loops.is_empty() {
        return None;
    }
    let n = ANGULAR_SAMPLES;
    let rings: Vec<Vec<Point>> = loops
        .iter()
        .map(|l| resample_by_angle(l, center, n))
        .collect::<Option<_>>()?;

    let mut path = vec![center];
    for pair in rings.windows(2) {
        let (inner, outer) = (&pair[0], &pair[1]);
        for i in 0..n {
            let t = i as f64 / n as f64;
            let (a, b) = (inner[i], outer[i]);
            path.push(Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
        }
    }
    // A closing lap of the outermost ring so the wall band is fully cut.
    let last = rings.last()?;
    for &p in last {
        path.push(p);
    }
    path.push(last[0]);
    Some(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clearsim::certify;

    /// A regular n-gon of radius `rad` centred at the origin (a stand-in circle).
    fn circle(rad: f64, n: usize) -> Polygon {
        let pts = (0..n)
            .map(|i| {
                let a = std::f64::consts::TAU * (i as f64) / (n as f64);
                Point::new(rad * a.cos(), rad * a.sin())
            })
            .collect();
        Polygon::new(Contour::new(pts)).unwrap()
    }

    #[test]
    fn adaptive_clears_a_round_pocket_as_a_bounded_engagement_spiral() {
        // A round pocket now certifies: the loops are joined into a spiral, so the
        // radius grows a band per revolution with no radial link spike. The oracle
        // independently confirms engagement at the cap, coverage, and no gouge.
        let region = circle(9.0, 40);
        let r = 3.0;
        let e = 2.0;
        let path = adaptive_path(&region, r, 0.0, e, Some([0.0, 0.0]))
            .expect("a round pocket should certify as a spiral");
        assert!(path.len() > 8, "expected a multi-turn spiral, got {}", path.len());
        let v = certify(&path, r, &region);
        assert!(
            v.max_engagement <= e * 1.05 + 1e-6,
            "peak engagement {} exceeds the cap {e}",
            v.max_engagement
        );
        assert!(v.uncut_area < 3.0, "the pocket is covered, uncut {}", v.uncut_area);
        assert!(v.gouge_area < 1.0, "no gouge, got {}", v.gouge_area);
    }

    #[test]
    fn the_generator_never_returns_an_uncertified_path() {
        // The core contract regardless of shape: a returned path always certifies.
        for (rad, e) in [(7.0, 3.0), (9.0, 4.0)] {
            let region = circle(rad, 28);
            let r = 3.0;
            if let Some(path) = adaptive_path(&region, r, 0.0, e, Some([0.0, 0.0])) {
                let v = certify(&path, r, &region);
                assert!(
                    v.max_engagement <= e * 1.05 + 1e-6,
                    "returned path must hold the cap, got {}",
                    v.max_engagement
                );
                assert!(v.gouge_area < 1.0, "returned path must not gouge, got {}", v.gouge_area);
            }
        }
    }

    #[test]
    fn tiny_pocket_that_cannot_fit_the_tool_falls_back() {
        // Tool too big to enter ⇒ no tool-centre region ⇒ fall back (None).
        let region = circle(2.0, 32);
        assert!(adaptive_path(&region, 3.0, 0.0, 2.0, None).is_none());
    }
}
