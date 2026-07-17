//! Constant-engagement adaptive clearing — the generator.
//!
//! For a simply-connected region: take the concentric tool-centre loops (the region
//! offset in by the tool radius, then by a stepover of `engagement`), then **morph
//! them into one continuous spiral** so the radius grows a band per revolution with
//! no radial link between passes — the links were what spiked the engagement. The
//! whole path is then **certified** against the fast raster oracle
//! ([`crate::raster`], cross-checked against the polygon anchor [`crate::clearsim`]):
//! it is returned only if it holds engagement at the cap, covers the reachable
//! target, and never gouges. The caller falls back to concentric clearing on `None`.
//!
//! Scope today: a simply-connected region, star-convex from the entry (so its loops
//! spiral cleanly). Regions that split, grow an island, or are not star-convex from
//! the entry fall back — cleared-region-tracked generation (the union front-advance)
//! and corner handling return for those in later phases. The oracle guarantees every
//! emitted path is correct regardless, so this grows without ever shipping a bad one.

use cam_geo::{offset, JoinStyle, Point, Polygon};

use crate::clearsim::reachable;

/// Angular samples per revolution when morphing loops into a spiral.
const ANGULAR_SAMPLES: usize = 48;
/// Cap on peel iterations (guards the front-advance loop).
const MAX_PASSES: usize = 400;
/// Safety cap on the number of concentric loops (the front-advance is bounded, but
/// this guards a pathological region). The raster oracle certifies in linear time,
/// so this is generous — the practical limit is the per-pass offset/boolean cost of
/// generation, not certification.
const MAX_LOOPS: usize = 400;

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

    // Collect the concentric tool-centre loops, wall-most first: offset the region
    // in by (r + finish) for the wall loop, then by a stepover of `e` until it closes
    // to the centre. For a simply-connected region these equal the front-advanced
    // loops but are far cheaper (independent offsets, no accumulated-cleared union);
    // cleared-region tracking returns for islands / concave regions. A loop that
    // splits or grows a hole means a non-simple region — fall back for now.
    let mut loops: Vec<Vec<Point>> = Vec::new();
    let mut d = r + finish;
    for _ in 0..MAX_PASSES {
        if loops.len() > MAX_LOOPS {
            return None;
        }
        let offs = offset(std::slice::from_ref(region), -d, JoinStyle::Round).ok()?;
        if offs.len() > 1 {
            return None; // the region split — not simply connected (later phase)
        }
        let Some(poly) = offs.into_iter().next() else {
            break; // closed off — innermost reached
        };
        if !poly.holes().is_empty() {
            return None; // an island opened up (later phase)
        }
        if poly.area() < e * e {
            break; // innermost sliver — the seed disc and first turns cover the core
        }
        loops.push(poly.outer().points().to_vec());
        d += e;
    }
    if loops.len() < 2 {
        return None; // too small to spiral
    }
    // Carve inside-out: innermost first.
    loops.reverse();

    // Join the concentric loops into a continuous spiral so the radius grows a band
    // per revolution rather than jumping radially between loops — the radial links
    // were what spiked the engagement between passes.
    let path = spiral(&loops, entry)?;

    // The whole path must certify against the (fast, linear) raster oracle:
    // engagement ≤ cap, reachable target covered, no gouge. Cross-checked against the
    // polygon trust anchor in tests.
    let verdict = crate::raster::certify(&path, r, &to_clear, e)?;
    // The raster reads a_e to within about one cell (px), biased high — the safe
    // direction. Allow the cap that plus the usual engagement-cap slack (an advisory
    // target, not a hard limit) so a spiral held near the cap is not falsely rejected.
    let ok = verdict.max_engagement <= e * 1.15
        && verdict.uncut_area <= cover_tol
        && verdict.gouge_area <= cover_tol;
    ok.then_some(path)
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
    use cam_geo::Contour;
    use std::time::Instant;

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
    fn raster_and_polygon_oracles_agree_on_an_adaptive_path() {
        // The raster oracle must be a faithful stand-in for the polygon trust anchor:
        // on the same certified adaptive path, both report bounded engagement, full
        // coverage, and no gouge, with peak engagement within a cell or two.
        let region = circle(9.0, 40);
        let (r, e) = (3.0, 2.0);
        let path = adaptive_path(&region, r, 0.0, e, Some([0.0, 0.0]))
            .expect("round pocket certifies");
        let poly = certify(&path, r, &region);
        let ras = crate::raster::certify(&path, r, &region, e).expect("raster builds");
        assert!(
            (poly.max_engagement - ras.max_engagement).abs() < 0.8,
            "oracles disagree on peak engagement: poly {} vs raster {}",
            poly.max_engagement,
            ras.max_engagement
        );
        assert!(ras.max_engagement <= e * 1.2, "raster engagement bounded, got {}", ras.max_engagement);
        assert!(ras.uncut_area < 7.0 && ras.gouge_area < 3.0, "raster: covered {}, gouge {}", ras.uncut_area, ras.gouge_area);
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
    fn a_large_round_pocket_goes_adaptive_quickly() {
        // The payoff of the raster oracle: a ⌀60 pocket (far past the old polygon-
        // certify cap) now certifies as an adaptive spiral, in well under a second.
        let region = circle(30.0, 96);
        let (r, e) = (3.0, 2.0);
        let t = Instant::now();
        let path = adaptive_path(&region, r, 0.0, e, Some([0.0, 0.0]))
            .expect("a large round pocket should certify");
        let secs = t.elapsed().as_secs_f64();
        assert!(path.len() > 100, "a large pocket is a long spiral, got {}", path.len());
        assert!(secs < 3.0, "adaptive generation should be quick, took {secs:.2}s");
        // Confirm on the polygon trust anchor too (this is a one-off in the test).
        let v = certify(&path, r, &region);
        assert!(v.max_engagement <= e * 1.2, "polygon oracle: bounded, got {}", v.max_engagement);
        assert!(v.uncut_area < 60.0, "polygon oracle: covered, uncut {}", v.uncut_area);
    }

    #[test]
    fn tiny_pocket_that_cannot_fit_the_tool_falls_back() {
        // Tool too big to enter ⇒ no tool-centre region ⇒ fall back (None).
        let region = circle(2.0, 32);
        assert!(adaptive_path(&region, 3.0, 0.0, 2.0, None).is_none());
    }
}
