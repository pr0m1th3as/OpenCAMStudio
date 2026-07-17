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
/// Append a cut segment (a bare tool-centre polyline) to a move path. The move onto
/// its start is a **cut link** if the previous point is within `threshold` of it
/// (adjacent to cleared stock), else a **rapid** reposition — which the oracle and
/// emitter read as a lift-and-replunge at the start. The rest of the segment is cut.
fn append_segment(path: &mut Vec<(Point, bool)>, seg: &[Point], prev: &mut Option<Point>, threshold: f64) {
    let Some((&start, rest)) = seg.split_first() else {
        return;
    };
    let link_cut = prev.is_some_and(|pp| pp.distance(start) <= threshold);
    path.push((start, link_cut));
    path.extend(rest.iter().map(|&p| (p, true)));
    *prev = seg.last().copied();
}

/// Append a single closed loop, cut exactly (not morphed): reach its nearest point
/// (a cut link if adjacent to cleared stock, else a rapid), cut around, and close it.
/// Used for the **island family**, whose corners are convex — the tool unwraps there,
/// so engagement drops rather than spikes and there is nothing to morph away; cutting
/// the exact offset loop keeps the tool off the rounded island corners (morphing
/// across radius-scaled corner arcs would chord inside them and nick the island).
fn append_plain_loop(path: &mut Vec<(Point, bool)>, loop_pts: &[Point], prev: &mut Option<Point>, threshold: f64) {
    if loop_pts.len() < 3 {
        return;
    }
    let ri = crate::profile::rotate_to_start(loop_pts, prev.map(|p| [p.x, p.y]));
    let start = ri[0];
    let link_cut = prev.is_some_and(|pp| pp.distance(start) <= threshold);
    path.push((start, link_cut));
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
/// The outer family (concave frame-wall corners, the pocket-style engagement spike)
/// is arc-length-morphed into a spiral so its sharp corners hold the cap; the island
/// family (convex corners that under-engage) is cut as exact concentric loops, which
/// also keeps the tool off the island's rounded corners. This certifies frames at the
/// cap; [`adaptive_frame`] is the certified entry the clearing engine calls.
fn frame_path(region: &Polygon, r: f64, finish: f64, e: f64) -> Option<Vec<(Point, bool)>> {
    let center = centroid(region);

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
    // cut opens without slotting; both spirals then peel off it.
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

    // The outer family has the frame's *concave* wall corners — the same engagement
    // spike a pocket has — so arc-length-morph it into a spiral (medial→wall); the
    // sharp corners stay a stepover apart, flattening the spike. Sample fine enough to
    // trace the walls faithfully.
    let step = (0.15 * r).clamp(0.05, e);
    let outer_loops: Vec<Vec<Point>> = outer_family.iter().rev().cloned().collect();
    let outer_sp = family_spiral(&outer_loops, center, step)?;
    append_segment(&mut path, &outer_sp, &mut prev, threshold);

    // The island family goes *around* a convex obstacle: engagement drops at those
    // corners rather than spiking, so there is nothing to morph — and morphing would
    // chord inside the radius-scaled rounded corner arcs and nick the island. Cut the
    // exact offset loops instead, peeling medial→island so each is adjacent to cleared
    // stock.
    for loop_pts in island_family.iter().rev() {
        append_plain_loop(&mut path, loop_pts, &mut prev, threshold);
    }

    (path.len() > 4).then_some(path)
}

/// Certified entry for clearing a frame/annulus (a region with island holes) at
/// constant engagement. Builds the [`frame_path`] and returns it **only if** it
/// certifies against the raster oracle — engagement at the cap, the reachable target
/// covered, no gouge — mirroring [`adaptive_path`]'s certified-or-fallback contract.
/// Returns `None` (⇒ fall back to plain concentric, which honours islands correctly)
/// whenever it cannot certify. The moves carry a cut/rapid flag: the frame lifts
/// between the outer spiral and the island loops.
pub(crate) fn adaptive_frame(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
) -> Option<Vec<(Point, bool)>> {
    if !(e > 0.0 && e < 2.0 * r) || region.holes().is_empty() {
        return None;
    }
    // Certify against the material actually removed (skin left on the walls).
    let to_clear = largest(offset(std::slice::from_ref(region), -finish, JoinStyle::Round).ok()?)?;
    let moves = frame_path(region, r, finish, e)?;

    let reach = reachable(&to_clear, r);
    if reach.is_empty() {
        return None;
    }
    let cover_tol = 0.02 * area(&reach) + 2.0;
    let verdict = crate::raster::certify_moves(&moves, r, &to_clear, e)?;
    let ok = verdict.max_engagement <= e * 1.2
        && verdict.uncut_area <= cover_tol
        && verdict.gouge_area <= cover_tol;
    ok.then_some(moves)
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

/// Morph a family of nested concentric `loops` (innermost first, **all wound the same
/// way**) into one continuous arc-length spiral about `center`. Unlike [`spiral`] it
/// seeds **no centre point**: the innermost element is a full loop (the medial ring of
/// a frame, or the tight loop around an island), not the region's centre — so the
/// family peels from its innermost loop outward, each turn a stepover from the last,
/// with the corners bounded by arc-length morphing (the same medicine that flattened
/// the pocket-spiral corner spike). Ends with a closing lap of the outermost loop to
/// finish that wall.
///
/// The two loop families of a frame wind oppositely (CCW outer contours, CW hole
/// contours — [`cam_geo`]'s convention), so each is morphed as its **own** spiral and
/// the caller links them at depth; blending across the winding seam would cross the
/// turns and gouge.
fn family_spiral(loops: &[Vec<Point>], center: Point, step: f64) -> Option<Vec<Point>> {
    match loops.len() {
        0 => return None,
        // A lone loop can't morph — emit it closed (its own wall lap).
        1 => {
            let mut v = loops[0].clone();
            v.push(*loops[0].first()?);
            return Some(v);
        }
        _ => {}
    }
    // Sample all loops at a common arc-length `step` fine enough to trace the tightest
    // rounded corner faithfully: a hole offset outward rounds its corners to radius ≈
    // the offset distance, and chording across such an arc dips the tool inside it —
    // gouging the island. `step` is set by the caller from the tool radius (not the
    // stepover) so the corner sagitta stays negligible. Radial spacing between turns
    // is the offset stepover regardless; this only sharpens along-loop fidelity.
    let max_perim = loops
        .iter()
        .map(|l| (0..l.len()).map(|k| l[k].distance(l[(k + 1) % l.len()])).sum::<f64>())
        .fold(0.0_f64, f64::max);
    let n = ((max_perim / step.max(1e-6)).ceil() as usize).clamp(ANGULAR_SAMPLES, 4096);

    let rings: Vec<Vec<Point>> = loops
        .iter()
        .map(|l| resample_by_arclength(l, center, n))
        .collect::<Option<_>>()?;

    let mut path = Vec::with_capacity(n * rings.len() + n);
    for pair in rings.windows(2) {
        let (inner, outer) = (&pair[0], &pair[1]);
        for i in 0..n {
            let t = i as f64 / n as f64;
            let (a, b) = (inner[i], outer[i]);
            path.push(Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
        }
    }
    // A closing lap of the outermost ring so its wall band is fully cut.
    let last = rings.last()?;
    path.extend_from_slice(last);
    path.push(last[0]);
    Some(path)
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
        // trochoidal medial channel avoids slotting; the outer family is arc-length-
        // morphed into a spiral so its concave wall corners hold the cap (the same
        // medicine as the pocket spiral), and the island family is cut as exact
        // concentric loops (its convex corners under-engage — nothing to morph, and
        // morphing across their radius-scaled corner arcs would nick the island). The
        // frame now clears covered, no gouge, engagement at the cap.
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
            v.max_engagement <= 1.2 * e,
            "engagement held at the cap by the morph (was ~1.5x before), got {}",
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
    fn adaptive_frame_certifies_and_falls_back_when_it_cannot() {
        // The certified entry the clearing engine calls: a real frame yields a
        // certified move path (engagement at the cap, covered, no gouge); a solid
        // pocket (no island) is not this entry's job and returns None so the caller
        // routes it through `adaptive_path` instead.
        let outer = Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(60.0, 0.0),
            Point::new(60.0, 60.0),
            Point::new(0.0, 60.0),
        ]);
        let island = Contour::new(vec![
            Point::new(25.0, 25.0),
            Point::new(35.0, 25.0),
            Point::new(35.0, 35.0),
            Point::new(25.0, 35.0),
        ]);
        let region = Polygon::with_holes(outer, vec![island]).unwrap();
        let (r, e) = (3.0, 2.0);
        let moves = adaptive_frame(&region, r, 0.0, e).expect("a frame certifies");
        assert!(moves.iter().any(|&(_, cut)| cut), "the frame has cutting moves");
        assert!(
            moves.iter().any(|&(_, cut)| !cut),
            "the frame lifts at least once (entry, and between families)"
        );
        // Re-certify independently: the wrapper's contract is that what it returns holds.
        let v = crate::raster::certify_moves(&moves, r, &region, e).expect("raster builds");
        assert!(v.max_engagement <= e * 1.2, "engagement at the cap, got {}", v.max_engagement);
        assert!(v.gouge_area < 1.0, "no gouge, got {}", v.gouge_area);

        // A solid pocket is not a frame — this entry declines it.
        assert!(adaptive_frame(&circle(9.0, 40), r, 0.0, e).is_none());
    }

    #[test]
    fn the_adaptive_spiral_winds_ccw_the_climb_sense() {
        // Adaptive clearing is climb-by-construction: the spiral inherits the inward-
        // offset winding, which for a pocket is CCW — the same sense the concentric
        // *climb* path cuts (it reverses to CW only for conventional). Confirm the
        // spiral's net winding is CCW (positive signed area) so the climb-only gate in
        // `clearing::clear` is cutting the direction it claims.
        let region = circle(9.0, 40);
        let path = adaptive_path(&region, 3.0, 0.0, 2.0, Some([0.0, 0.0])).expect("certifies");
        let signed2: f64 = path
            .windows(2)
            .map(|w| w[0].x * w[1].y - w[1].x * w[0].y)
            .sum();
        assert!(signed2 > 0.0, "spiral should wind CCW (climb), signed area·2 = {signed2}");
    }

    #[test]
    fn tiny_pocket_that_cannot_fit_the_tool_falls_back() {
        // Tool too big to enter ⇒ no tool-centre region ⇒ fall back (None).
        let region = circle(2.0, 32);
        assert!(adaptive_path(&region, 3.0, 0.0, 2.0, None).is_none());
    }
}
