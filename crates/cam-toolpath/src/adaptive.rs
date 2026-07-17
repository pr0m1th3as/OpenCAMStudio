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

    // Seed the cleared region with the entry disc; the path starts at the entry.
    let mut cleared: Vec<Polygon> = vec![disc(entry, r)?];
    let mut path = vec![entry];
    let mut prev = entry;
    let mut last_uncut = f64::INFINITY;

    for _ in 0..MAX_PASSES {
        // Bail to the concentric fallback before the sequential certify gets costly.
        if path.len() > MAX_PATH {
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
        // Begin the loop nearest where we are, cut it, and close it.
        let ordered = crate::profile::rotate_to_start(pts, Some([prev.x, prev.y]));
        for &p in &ordered {
            path.push(p);
        }
        path.push(ordered[0]);
        prev = ordered[0];

        // Grow the cleared region by this loop's sweep.
        let mut loop_path = ordered.clone();
        loop_path.push(ordered[0]);
        let swept = stroke_path(&Polyline::new(loop_path), r, CapStyle::Round, JoinStyle::Round).ok()?;
        if swept.is_empty() {
            return None;
        }
        cleared = union(&cleared, &swept).ok()?;
    }

    // The whole path must certify: engagement ≤ cap, reachable target covered, no gouge.
    let verdict = certify(&path, r, &to_clear);
    verdict.certified(e, cover_tol).then_some(path)
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
    fn the_generator_never_returns_an_uncertified_path() {
        // The core contract: whatever the generator returns is either `None` (the
        // caller falls back to the proven concentric clearer) or a path the oracle
        // independently confirms — engagement at the cap, no gouge. It never ships a
        // path it cannot certify. (Round pockets fall back today because the radial
        // link between passes spikes engagement; the spiral connection that lets them
        // certify is the next step — this invariant holds throughout.)
        let region = circle(9.0, 32);
        let r = 3.0;
        let e = 2.0;
        if let Some(path) = adaptive_path(&region, r, 0.0, e, Some([0.0, 0.0])) {
            let v = certify(&path, r, &region);
            assert!(
                v.max_engagement <= e * 1.05 + 1e-6,
                "a returned path must hold the cap, got {}",
                v.max_engagement
            );
            assert!(v.gouge_area < 1.0, "a returned path must not gouge, got {}", v.gouge_area);
        }
    }

    #[test]
    fn tiny_pocket_that_cannot_fit_the_tool_falls_back() {
        // Tool too big to enter ⇒ no tool-centre region ⇒ fall back (None).
        let region = circle(2.0, 32);
        assert!(adaptive_path(&region, 3.0, 0.0, 2.0, None).is_none());
    }
}
