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
//! Scope today: a simply-connected region whose inward offsets stay a single loop
//! (convex or gently concave, sharp corners included). Regions that split into lobes
//! or grow an island fall back — cleared-region-tracked generation (the union
//! front-advance) returns for those in a later phase. The oracle guarantees every
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

/// Point, unit tangent, and unit left-normal at arc-length `u` along the polyline
/// `pts` (treated as open: last vertex ends it).
#[allow(dead_code)] // used by the frame trochoidal entry (in progress) + tests
fn frame_at(pts: &[Point], u: f64) -> (Point, (f64, f64), (f64, f64)) {
    let mut acc = 0.0;
    let last = pts.len().saturating_sub(1);
    for k in 0..last {
        let seg = pts[k].distance(pts[k + 1]);
        if u <= acc + seg || k == last - 1 {
            let t = if seg > 1e-9 { ((u - acc) / seg).clamp(0.0, 1.0) } else { 0.0 };
            let p = Point::new(
                pts[k].x + (pts[k + 1].x - pts[k].x) * t,
                pts[k].y + (pts[k + 1].y - pts[k].y) * t,
            );
            let tan = {
                let (dx, dy) = (pts[k + 1].x - pts[k].x, pts[k + 1].y - pts[k].y);
                let l = dx.hypot(dy).max(1e-12);
                (dx / l, dy / l)
            };
            return (p, tan, (-tan.1, tan.0));
        }
        acc += seg;
    }
    (pts[0], (1.0, 0.0), (0.0, 1.0))
}

/// Open a cut along `guide` with a **trochoidal** path — small loops advancing along
/// the guide — so the tool bites only a stepover of fresh material at a time even
/// though the channel it opens is much wider than a peel. This is the entry that lets
/// a region with no spiral centre (a frame/annulus around an island) start clearing
/// without slotting its first loop.
///
/// One loop is completed per `e` of forward advance (the forward bite ⇒ engagement ≈
/// `e`); the loop radius `radius` sets the channel half-width and must exceed the
/// pitch so successive loops overlap into already-cut stock rather than re-slot.
#[allow(dead_code)] // the frame trochoidal entry (in progress) + tests
fn trochoidal_channel(guide: &[Point], e: f64, radius: f64) -> Vec<Point> {
    if guide.len() < 2 {
        return Vec::new();
    }
    let total: f64 = (0..guide.len() - 1).map(|k| guide[k].distance(guide[k + 1])).sum();
    if total < 1e-6 {
        return Vec::new();
    }
    let pitch = e; // forward advance per loop ⇒ ~e engagement
    let ds = (pitch / 10.0).clamp(0.05, 0.5);
    let steps = (total / ds).ceil() as usize;
    let mut path = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let u = (i as f64 * ds).min(total);
        let (g, tg, ng) = frame_at(guide, u);
        let phi = std::f64::consts::TAU * u / pitch;
        let (c, s) = (phi.cos(), phi.sin());
        // Loop about the guide point: behind (−T) → inward (N) → ahead (T) → out (−N).
        let dir = (-tg.0 * c + ng.0 * s, -tg.1 * c + ng.1 * s);
        path.push(Point::new(g.x + radius * dir.0, g.y + radius * dir.1));
    }
    path
}

/// Clear a region whose offsets stay nested loops but do **not** reduce to a spiral
/// centre — a frame/annulus around an island. Concentric loops honour the island; the
/// innermost (medial) loop is opened with a **trochoidal channel** (so its first cut
/// doesn't slot into solid), then the remaining loops peel off it, each adjacent to
/// cleared stock. Returns `(point, is_cut)` moves (a frame lifts between loop
/// families). `None` if there is nothing to clear.
/// Append a closed loop to a move path: reach its nearest point (a cut link if close
/// to cleared stock, else a rapid lift), then cut around and close it.
fn append_loop(path: &mut Vec<(Point, bool)>, loop_pts: &[Point], prev: &mut Option<Point>, threshold: f64) {
    if loop_pts.len() < 3 {
        return;
    }
    let ri = crate::profile::rotate_to_start(loop_pts, prev.map(|p| [p.x, p.y]));
    let start = ri[0];
    let link = prev.is_some_and(|pp| pp.distance(start) <= threshold);
    path.push((start, link));
    path.extend(ri[1..].iter().map(|&p| (p, true)));
    path.push((start, true));
    *prev = Some(start);
}

/// Clear a frame/annulus (a region around an island) at bounded engagement. The two
/// concentric loop families (outer contours offset in from the wall, hole contours
/// offset out from the island) are cut **contiguously** — medial→wall, then
/// medial→island — so each loop is adjacent to already-cleared stock, with a
/// trochoidal channel opening the medial so its first cut doesn't slot. Returns
/// `(point, is_cut)` moves (a frame lifts between families).
///
/// This covers frames with no gouge and keeps engagement well below a slot, but not
/// yet to the cap: the loops are cut directly, so the tool still over-engages a
/// bounded amount pivoting the sharp corners (same spike the pocket spiral solved
/// with arc-length morphing — the annular families need the equivalent, which is the
/// remaining piece). So this is not yet wired into `adaptive_path`.
#[allow(dead_code)]
fn frame_path(region: &Polygon, r: f64, finish: f64, e: f64) -> Option<Vec<(Point, bool)>> {
    // Separate the two loop families the concentric offsets produce, wall/island-most
    // first.
    let mut outer_family: Vec<Vec<Point>> = Vec::new();
    let mut island_family: Vec<Vec<Point>> = Vec::new();
    let mut d = r + finish;
    for _ in 0..MAX_LOOPS {
        let offs = offset(std::slice::from_ref(region), -d, JoinStyle::Round).ok()?;
        // A frame stays one annular polygon; splitting into lobes is a harder topology.
        if offs.len() != 1 {
            break;
        }
        let poly = offs.into_iter().next()?;
        outer_family.push(poly.outer().points().to_vec());
        for h in poly.holes() {
            island_family.push(h.points().to_vec());
        }
        d += e;
    }
    if outer_family.len() < 2 || island_family.is_empty() {
        return None;
    }

    let threshold = 1.5 * e;
    let mut path: Vec<(Point, bool)> = Vec::new();

    // Open a trochoidal channel around the medial (innermost) outer loop so the first
    // cut opens without slotting; both families then peel off it.
    let medial = outer_family.last()?;
    let mut guide = medial.clone();
    guide.push(medial[0]);
    let ch = trochoidal_channel(&guide, e, 1.5 * e);
    if ch.len() < 2 {
        return None;
    }
    path.push((ch[0], false)); // rapid to the channel start = the plunge
    path.extend(ch[1..].iter().map(|&p| (p, true)));
    let mut prev = ch.last().copied();

    // Peel each family contiguously (medial→wall, then medial→island); lift between.
    for loop_pts in outer_family.iter().rev().skip(1) {
        append_loop(&mut path, loop_pts, &mut prev, threshold);
    }
    for loop_pts in island_family.iter().rev() {
        append_loop(&mut path, loop_pts, &mut prev, threshold);
    }
    (path.len() > 4).then_some(path)
}

/// Resample a closed loop to `n` points at even **arc-length** intervals, starting at
/// the vertex whose direction from `center` is nearest +X (a phase alignment so the
/// same parameter on successive loops lands at roughly the same place around them).
///
/// Arc length — not angle — is used because concentric offset loops are *equidistant*
/// curves: the same arc-length fraction on adjacent loops stays ≈ a stepover apart
/// perpendicular, on straights *and* around corners. Sampling by angle instead balloons
/// the spacing at corners (a square corner is √2× farther out than its edge), which
/// spikes the engagement there.
fn resample_by_arclength(pts: &[Point], center: Point, n: usize) -> Option<Vec<Point>> {
    let m = pts.len();
    if m < 3 || n == 0 {
        return None;
    }
    // Phase-align: start at the vertex whose direction from `center` is nearest +X.
    let start = (0..m).min_by(|&a, &b| {
        let angle = |k: usize| (pts[k].y - center.y).atan2(pts[k].x - center.x).abs();
        angle(a).partial_cmp(&angle(b)).unwrap_or(std::cmp::Ordering::Equal)
    })?;
    let rot: Vec<Point> = (0..m).map(|k| pts[(start + k) % m]).collect();

    let perim: f64 = (0..m).map(|k| rot[k].distance(rot[(k + 1) % m])).sum();
    if perim < 1e-9 {
        return None;
    }
    let step = perim / n as f64;
    let mut out = Vec::with_capacity(n);
    out.push(rot[0]);
    let mut dist_along = 0.0;
    let mut next = step;
    for k in 0..m {
        let (a, b) = (rot[k], rot[(k + 1) % m]);
        let seg = a.distance(b);
        while out.len() < n && next <= dist_along + seg + 1e-9 {
            let t = if seg > 1e-12 { ((next - dist_along) / seg).clamp(0.0, 1.0) } else { 0.0 };
            out.push(Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
            next += step;
        }
        dist_along += seg;
    }
    while out.len() < n {
        out.push(rot[0]);
    }
    out.truncate(n);
    Some(out)
}

/// Morph the concentric `loops` (inner-first) into one continuous spiral about
/// `center`: within each revolution the point blends from loop `k` to loop `k+1`,
/// so the radius grows a band per turn with no radial jump. Ends with a full lap of
/// the outermost loop to finish the wall.
fn spiral(loops: &[Vec<Point>], center: Point) -> Option<Vec<Point>> {
    if loops.is_empty() {
        return None;
    }
    let n = ANGULAR_SAMPLES;
    let rings: Vec<Vec<Point>> = loops
        .iter()
        .map(|l| resample_by_arclength(l, center, n))
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
    fn a_square_pocket_goes_adaptive_with_bounded_corners() {
        // Sharp corners: arc-length resampling keeps the spiral turns a stepover apart
        // around the corners too (angle resampling would balloon to √2× and spike the
        // engagement there). The square certifies, covered, engagement at the cap.
        let region = Polygon::new(Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(40.0, 0.0),
            Point::new(40.0, 40.0),
            Point::new(0.0, 40.0),
        ]))
        .unwrap();
        let (r, e) = (3.0, 2.0);
        let path = adaptive_path(&region, r, 0.0, e, Some([20.0, 20.0]))
            .expect("a square pocket should certify");
        let v = certify(&path, r, &region);
        assert!(
            v.max_engagement <= e * 1.2,
            "corner engagement should be bounded, got {}",
            v.max_engagement
        );
        assert!(v.gouge_area < 2.0, "no gouge, got {}", v.gouge_area);
    }

    #[test]
    fn trochoidal_channel_opens_at_bounded_engagement() {
        // Open a channel along a straight guide in virgin stock: the trochoidal loops
        // bite only a stepover at a time, so the peak engagement stays near the cap
        // even though the channel is far wider than a peel. This is the entry that
        // lets a frame start clearing without slotting.
        let guide: Vec<Point> = (0..=40).map(|i| Point::new(i as f64, 0.0)).collect();
        let (r, e) = (3.0, 2.0);
        let path = trochoidal_channel(&guide, e, 2.0 * e);
        assert!(path.len() > 20, "a trochoidal channel is many small loops");
        // Big region so walls/coverage aren't a factor — we only assert engagement.
        let region = Polygon::new(Contour::new(vec![
            Point::new(-15.0, -15.0),
            Point::new(55.0, -15.0),
            Point::new(55.0, 15.0),
            Point::new(-15.0, 15.0),
        ]))
        .unwrap();
        let v = certify(&path, r, &region);
        assert!(
            v.max_engagement <= e * 1.3,
            "trochoidal channel engagement should stay near the cap, got {}",
            v.max_engagement
        );
    }

    #[test]
    fn a_frame_around_an_island_clears_without_slotting_or_gouging() {
        // 60×60 pocket with a 20×20 island (a roughing frame is the same shape). The
        // trochoidal medial channel avoids slotting and the contiguous family peel
        // covers the frame with no gouge; engagement stays well below a full slot
        // (diameter 6). Reaching the cap exactly still needs the annular-family morph
        // that bounds the sharp corners — the remaining piece — so this is not yet
        // wired into `adaptive_path`.
        let outer = Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(60.0, 0.0),
            Point::new(60.0, 60.0),
            Point::new(0.0, 60.0),
        ]);
        let island = Contour::new(vec![
            Point::new(20.0, 20.0),
            Point::new(40.0, 20.0),
            Point::new(40.0, 40.0),
            Point::new(20.0, 40.0),
        ]);
        let region = Polygon::with_holes(outer, vec![island]).unwrap();
        let (r, e) = (3.0, 2.0);
        let moves = frame_path(&region, r, 0.0, e).expect("a frame yields a path");
        let v = crate::raster::certify_moves(&moves, r, &region, e).expect("raster builds");
        let reach: f64 = crate::clearsim::reachable(&region, r).iter().map(|p| p.area()).sum();
        assert!(
            v.max_engagement <= 1.6 * e,
            "engagement well below a slot (no full-width cut), got {}",
            v.max_engagement
        );
        assert!(v.gouge_area < 1.0, "no gouge, got {}", v.gouge_area);
        assert!(
            v.uncut_area < 0.05 * reach + 2.0,
            "frame covered, uncut {} of {}",
            v.uncut_area,
            reach
        );
    }

    #[test]
    fn tiny_pocket_that_cannot_fit_the_tool_falls_back() {
        // Tool too big to enter ⇒ no tool-centre region ⇒ fall back (None).
        let region = circle(2.0, 32);
        assert!(adaptive_path(&region, 3.0, 0.0, 2.0, None).is_none());
    }
}
