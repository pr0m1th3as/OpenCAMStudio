//! Front-advance (cleared-region-tracking) adaptive clearing.
//!
//! Where the spiral-morph generator ([`crate::adaptive`]) blends pre-computed offset
//! rings and *hopes* they hold engagement — the exact oracle showed they slot at the
//! entry, at sharp corners, and at ring/handoff transitions — this advances the
//! **actual cleared region** outward by one stepover per pass.
//!
//! Each pass's tool centres follow `offset(cleared ⊕ e, −r)`, so the tool reaches
//! exactly `e` beyond what is already cleared and the *loops themselves* peel a stepover
//! by construction. That is the frontier's guarantee, and it holds: measured by the exact
//! oracle ([`crate::clearsim`]), the body of the path reads **1.4·e on a round pocket and
//! a square pocket alike** — the sameness being the evidence that the tracking is sound.
//!
//! The guarantee is about the loops, though, and a path is not only loops. What the
//! frontier does **not** hand you, and what is therefore built on top:
//!
//! - **connecting the loops** — cutting each loop then jumping a stepover outward to the
//!   next puts the tool's whole leading half into virgin stock, and the oracle reads the
//!   full **diameter**. This was the generator's dominant defect and it appeared on every
//!   shape, corners or not. [`connect_seam_spiral`] answers it: cut each loop at its own
//!   offset (never morphed off it, or the frontier's guarantee is discarded), and localise
//!   the hand-off to the +X seam.
//! - **the entry** — a radial move off the plunge point slots identically;
//!   [`core_spiral_to_seam`] eases out instead, landing on the first loop's seam.
//! - **concave corners** — uncut corner material wraps around the leading edge of a wall
//!   pass, so the engagement *angle* climbs even though the material is thin. **Still
//!   open** at ~2.2·e; wants corner-specific trochoidal peels (the classic HSM hard case).
//! - **concavities that split the frontier** into several loops — no single seam, so those
//!   fall back to the slotting links for now.
//!
//! Every emitted pass is verifiable against the exact engagement oracle; the caller keeps
//! the certified-or-fallback contract, so the open gaps above cost a fallback to
//! concentric clearing, never a bad path.
#![allow(dead_code)]

use cam_geo::{intersection, offset, union, Arc, Contour, JoinStyle, Point, Polygon};

/// Guard on the number of outward passes.
const MAX_PASSES: usize = 800;

/// Vertex-decimation tolerance (mm) for the cleared frontier — far below any stepover,
/// so the frontier shifts negligibly and engagement is unaffected.
const SIMPLIFY_EPS: f64 = 0.02;

/// Length of a loop-to-loop seam hand-off, in stepovers.
///
/// **Not a tuning knob** — the peak engagement is flat from 1 to 12 (measured), because
/// across the hand-off the tool only ever bites the one stepover it drifts outward,
/// however far that drift is spread. Set for a tidy point count; don't reach for it to
/// chase an engagement spike, as the spike will be the entry or a corner, not this.
const SEAM_ARC: f64 = 4.0;

fn total_area(polys: &[Polygon]) -> f64 {
    polys.iter().map(Polygon::area).sum()
}

/// The largest polygon by area (the body of a boolean/offset result).
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

/// A filled disc of radius `r` at `c` as a polygon (the material a plunge opens).
fn disc(c: Point, r: f64) -> Option<Polygon> {
    Polygon::new(Contour::new(Arc::circle(c, r).flatten(0.05))).ok()
}

/// Perpendicular distance from `p` to segment `a`→`b`.
fn seg_dist(p: Point, a: Point, b: Point) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    if len2 < 1e-12 {
        return p.distance(a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
    p.distance(Point::new(a.x + dx * t, a.y + dy * t))
}

/// Douglas–Peucker simplify of a closed ring at tolerance `eps` (iterative, so a
/// pathological ring can't blow the stack). Keeps the ring closed.
fn simplify_ring(ring: &[Point], eps: f64) -> Vec<Point> {
    let n = ring.len();
    if n < 8 {
        return ring.to_vec();
    }
    // Treat as a polyline 0..n with an appended closing vertex, so the seam vertex is
    // kept and the ring stays closed.
    let mut pts = ring.to_vec();
    pts.push(ring[0]);
    let m = pts.len();
    let mut keep = vec![false; m];
    keep[0] = true;
    keep[m - 1] = true;
    let mut stack = vec![(0usize, m - 1)];
    while let Some((a, b)) = stack.pop() {
        if b <= a + 1 {
            continue;
        }
        let (pa, pb) = (pts[a], pts[b]);
        let (mut maxd, mut idx) = (0.0_f64, 0usize);
        for (off, &p) in pts[a + 1..b].iter().enumerate() {
            let d = seg_dist(p, pa, pb);
            if d > maxd {
                maxd = d;
                idx = a + 1 + off;
            }
        }
        if maxd > eps {
            keep[idx] = true;
            stack.push((a, idx));
            stack.push((idx, b));
        }
    }
    // Collect kept vertices, dropping the appended closing duplicate.
    pts[..m - 1]
        .iter()
        .enumerate()
        .filter(|(i, _)| keep[*i])
        .map(|(_, &p)| p)
        .collect()
}

/// Simplify every contour of `polys` at tolerance `eps`, rebuilding the polygons.
/// `eps` is kept far below the stepover, so the cleared frontier shifts negligibly and
/// engagement is unaffected — this only caps the vertex count that repeated round
/// offsets would otherwise balloon.
fn simplify_polys(polys: &[Polygon], eps: f64) -> Vec<Polygon> {
    polys
        .iter()
        .filter_map(|p| {
            let outer = Contour::new(simplify_ring(p.outer().points(), eps));
            let holes: Vec<Contour> =
                p.holes().iter().map(|h| Contour::new(simplify_ring(h.points(), eps))).collect();
            if holes.is_empty() {
                Polygon::new(outer).ok()
            } else {
                Polygon::with_holes(outer, holes).ok()
            }
        })
        .collect()
}

/// Append a closed loop to `path`, entered at the point nearest `prev` to keep the
/// pass-to-pass link short. **This still slots** — the link is radial, so the tool's
/// whole leading half faces uncut stock (exact oracle: `a_e` = the diameter). It is the
/// fallback for passes the seam spiral cannot connect (a frontier that split into
/// several loops); [`connect_seam_spiral`] is the real connection.
fn append_loop(path: &mut Vec<Point>, loop_pts: &[Point], prev: &mut Point) {
    if loop_pts.len() < 3 {
        return;
    }
    let rot = crate::profile::rotate_to_start(loop_pts, Some([prev.x, prev.y]));
    path.extend_from_slice(&rot);
    path.push(rot[0]); // close the loop
    *prev = rot[0];
}

/// Angular samples per revolution of the core spiral.
const CORE_SAMPLES: usize = 48;
/// Samples across a loop-to-loop seam transition.
const SEAM_SAMPLES: usize = 24;

/// Where the **+X ray** from `from` first crosses the closed loop `pts`. This is the
/// *seam*: one consistent place on every frontier loop to hand off to the next, chosen
/// on a ray rather than by nearest-point so successive loops hand off at the same place
/// instead of wandering.
fn seam_crossing(pts: &[Point], from: Point) -> Option<Point> {
    let n = pts.len();
    let mut best: Option<f64> = None;
    for k in 0..n {
        let (a, b) = (pts[k], pts[(k + 1) % n]);
        let (da, db) = (a.y - from.y, b.y - from.y);
        if (da > 0.0) == (db > 0.0) {
            continue; // this edge does not straddle the ray
        }
        let x = a.x + (b.x - a.x) * (da / (da - db));
        if x > from.x && best.is_none_or(|bx| x < bx) {
            best = Some(x); // nearest crossing outward
        }
    }
    best.map(|x| Point::new(x, from.y))
}

/// Rotate a closed loop to start at its +X seam from `from`.
fn seam_rotate(pts: &[Point], from: Point) -> Option<Vec<Point>> {
    let s = seam_crossing(pts, from)?;
    let rot = crate::profile::rotate_to_start(pts, Some([s.x, s.y]));
    (rot.len() >= 3).then_some(rot)
}

/// Cumulative arc length along the **closed** loop `pts` (index `i` = length from
/// `pts[0]` to `pts[i]`; the final entry is the full perimeter, back to `pts[0]`).
fn cum_len(pts: &[Point]) -> Vec<f64> {
    let mut cum = Vec::with_capacity(pts.len() + 1);
    let mut acc = 0.0;
    cum.push(0.0);
    for k in 0..pts.len() {
        acc += pts[k].distance(pts[(k + 1) % pts.len()]);
        cum.push(acc);
    }
    cum
}

/// The point at arc length `u` along the closed loop `pts` (with `cum` from
/// [`cum_len`]), clamped to the loop.
fn at_len(pts: &[Point], cum: &[f64], u: f64) -> Point {
    let total = *cum.last().unwrap_or(&0.0);
    if total <= 1e-12 {
        return pts[0];
    }
    let u = u.clamp(0.0, total);
    let i = cum.partition_point(|&c| c <= u).saturating_sub(1).min(pts.len() - 1);
    let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
    let seg = cum[i + 1] - cum[i];
    if seg <= 1e-12 {
        return a;
    }
    let t = (u - cum[i]) / seg;
    Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

/// An Archimedean spiral from `center` out to `radius`, **ending on the +X seam** so it
/// hands straight over to the first frontier loop at that loop's seam with no radial
/// jump. Turn count is rounded up, so the realised pitch never exceeds `pitch` (each
/// turn bites at most a stepover of fresh material). The plunge disc bounds the first
/// turn. Without this the entry is a radial move from the plunge point into virgin
/// stock — a full slot.
fn core_spiral_to_seam(center: Point, radius: f64, pitch: f64) -> Vec<Point> {
    if radius <= 1e-6 || pitch <= 1e-6 {
        return vec![center];
    }
    let turns = (radius / pitch).ceil().max(1.0);
    let theta_max = std::f64::consts::TAU * turns;
    let steps = (turns * CORE_SAMPLES as f64).ceil() as usize;
    (0..=steps)
        .map(|k| {
            let th = theta_max * (k as f64) / (steps as f64);
            let rr = radius * th / theta_max;
            Point::new(center.x + rr * th.cos(), center.y + rr * th.sin())
        })
        .collect()
}

/// Stitch the frontier `loops` (innermost first, all wound the same way) into one
/// continuous path, entered by a core spiral from `entry`.
///
/// Each loop is cut **at its own offset, in full** — the frontier's engagement guarantee
/// comes from the loop lying exactly `e` outside the cleared region, so morphing a loop
/// away from that offset (as a global arc-length blend does) throws the guarantee away
/// and spikes wherever the two loops' arc-length parametrisations drift apart, i.e. at
/// corners. Instead the hand-off to the next loop is **localised to the +X seam**: the
/// last `delta` of arc length before the seam blends into the next loop's own last
/// `delta`, arriving at its seam travelling tangentially.
///
/// That keeps the radial drift spread over `delta` of mostly-tangential feed, so the
/// tool overlaps stock the current loop already cut and bites only the drift — instead
/// of a radial jump straight into virgin stock, which slots at the full diameter.
fn connect_seam_spiral(
    entry: Point,
    loops: &[Vec<Point>],
    e: f64,
    seam_arc: f64,
) -> Option<Vec<Point>> {
    let seamed: Vec<Vec<Point>> =
        loops.iter().map(|l| seam_rotate(l, entry)).collect::<Option<_>>()?;
    let cums: Vec<Vec<f64>> = seamed.iter().map(|l| cum_len(l)).collect();

    // Open the core out to the first loop's seam, landing on it.
    let mut path = core_spiral_to_seam(entry, seamed[0][0].distance(entry), e);

    for k in 0..seamed.len() {
        let (cur, ccur) = (&seamed[k], &cums[k]);
        let total = *ccur.last()?;
        let Some(nxt) = seamed.get(k + 1) else {
            // Outermost: a full closing lap finishes the wall.
            path.extend_from_slice(&cur[1..]);
            path.push(cur[0]);
            break;
        };
        let cnxt = &cums[k + 1];
        let tot_n = *cnxt.last()?;
        // Spread the hand-off over enough arc that the feed stays mostly tangential,
        // but never over so much of a small inner loop that it is all transition.
        let delta = (seam_arc * e).min(0.35 * total).min(0.35 * tot_n);
        if delta <= 1e-9 {
            return None;
        }
        // Cut this loop from its seam up to where the hand-off begins.
        for (i, p) in cur.iter().enumerate().skip(1) {
            if ccur[i] >= total - delta {
                break;
            }
            path.push(*p);
        }
        // Hand off: blend this loop's tail into the next loop's tail, so we arrive at
        // the next seam already travelling along it.
        for i in 0..=SEAM_SAMPLES {
            let t = i as f64 / SEAM_SAMPLES as f64;
            let a = at_len(cur, ccur, total - delta + t * delta);
            let b = at_len(nxt, cnxt, tot_n - delta + t * delta);
            path.push(Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
        }
    }
    Some(path)
}

/// Generate a constant-engagement tool-centre path that clears `region` (leaving
/// `finish` skin on the walls) with a tool of radius `r`, holding the radial width of
/// cut at or below `e` by advancing the cleared front. `start` is the preferred entry
/// (part XY). Returns `None` when it cannot build a path (degenerate, or — for now —
/// an island region, handled in a later increment).
pub(crate) fn front_advance_path(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
) -> Option<Vec<Point>> {
    front_advance_tuned(region, r, finish, e, start, SEAM_ARC)
}

/// [`front_advance_path`] with the seam hand-off length exposed, so it can be swept.
fn front_advance_tuned(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
    seam_arc: f64,
) -> Option<Vec<Point>> {
    if !(e > 0.0 && e < 2.0 * r) {
        return None;
    }
    if !region.holes().is_empty() {
        return None; // islands: a later increment (the frontier flows around holes)
    }
    let to_clear = largest(offset(std::slice::from_ref(region), -finish, JoinStyle::Round).ok()?)?;
    let tc = largest(offset(std::slice::from_ref(region), -(r + finish), JoinStyle::Round).ok()?)?;

    let entry = start
        .map(|s| Point::new(s[0], s[1]))
        .filter(|p| tc.contains(*p))
        .unwrap_or_else(|| centroid(&tc));
    if !tc.contains(entry) {
        return None;
    }

    let clear_slice = std::slice::from_ref(&to_clear);
    let to_clear_area = to_clear.area();
    // The frontier has reached every wall once `grown` fills the stock to within this.
    let covered_tol = 0.001 * to_clear_area + 0.5 * e * e;

    let mut cleared = vec![disc(entry, r)?];
    // The frontier loops, innermost first. Collected rather than emitted as we go: the
    // seam hand-off needs the *next* loop while cutting the current one.
    let mut loops: Vec<Vec<Point>> = Vec::new();
    // A pass whose frontier split into several loops (a concavity pinching the front in
    // two) has no single seam — those fall back to the slotting nearest-point links.
    let mut split = false;

    for _ in 0..MAX_PASSES {
        // Advance the frontier one stepover into fresh material, clipped to the stock.
        let grown = intersection(&offset(&cleared, e, JoinStyle::Round).ok()?, clear_slice).ok()?;
        if grown.is_empty() {
            break;
        }
        // Tool centres that realise that frontier: the grown region eroded by r.
        let pass = offset(&grown, -r, JoinStyle::Round).ok()?;
        if pass.is_empty() {
            break;
        }
        split |= pass.len() > 1;
        for poly in &pass {
            loops.push(poly.outer().points().to_vec());
        }
        // Done once the frontier fills the stock — stable, unlike an area-delta check
        // (which the decimation jitter would trip early or never).
        if to_clear_area - total_area(&grown) < covered_tol {
            break;
        }
        // Advance: the new cleared region is what the tool cut (opening of `grown`),
        // decimated so repeated round offsets don't balloon its vertex count (the
        // tolerance is far below the stepover — engagement unaffected).
        let opened = offset(&pass, r, JoinStyle::Round).ok()?;
        cleared = simplify_polys(&union(&cleared, &opened).ok().unwrap_or(opened), SIMPLIFY_EPS);
    }
    if loops.is_empty() {
        return None;
    }

    // Stitch the frontier into one continuous path. The seam spiral is the real
    // connection; a split frontier has no single seam, so it keeps the plain links
    // (which slot — the caller's certification rejects them).
    let path = match (!split).then(|| connect_seam_spiral(entry, &loops, e, seam_arc)).flatten() {
        Some(p) => p,
        None => {
            let mut p = vec![entry];
            let mut prev = entry;
            for l in &loops {
                append_loop(&mut p, l, &mut prev);
            }
            p
        }
    };

    (path.len() > 3).then_some(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reference tool radius and engagement cap for the measured tests.
    const R: f64 = 3.0;
    const E: f64 = 2.0;

    fn square(s: f64) -> Polygon {
        Polygon::new(Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(s, 0.0),
            Point::new(s, s),
            Point::new(0.0, s),
        ]))
        .unwrap()
    }

    fn circle(rad: f64, n: usize) -> Polygon {
        Polygon::new(Contour::new(
            (0..n)
                .map(|i| {
                    let a = std::f64::consts::TAU * (i as f64) / (n as f64);
                    Point::new(rad * a.cos(), rad * a.sin())
                })
                .collect(),
        ))
        .unwrap()
    }

    /// Split every segment of `path` to at most `step`, so each engagement reading
    /// localises to a point. Without this a reading cannot be attributed to a place:
    /// [`ClearedModel::engagement`] returns the peak *over a move*, so a 34 mm run down
    /// a wall reports one number for the whole wall and hides that it peaked in the
    /// corner. Measuring the un-densified path is what previously mis-attributed the
    /// generator's link slots to its corners.
    fn densify(path: &[Point], step: f64) -> Vec<Point> {
        let mut out = vec![path[0]];
        for w in path.windows(2) {
            let n = ((w[0].distance(w[1]) / step).ceil() as usize).max(1);
            for k in 1..=n {
                let t = k as f64 / n as f64;
                out.push(Point::new(
                    w[0].x + (w[1].x - w[0].x) * t,
                    w[0].y + (w[1].y - w[0].y) * t,
                ));
            }
        }
        out
    }

    /// Every `(a_e, where)` reading along the generated path, by the **exact** oracle.
    ///
    /// Each move is *measured* in short pieces so a reading is attributable to a place,
    /// but the model is *committed* one whole move at a time — exactly as [`certify`]
    /// does. Keeping those two granularities separate matters: committing each piece as
    /// it is measured would credit the tool with material its own move has not swept
    /// yet at that instant, which reads lower than production ever will. So the peak
    /// over these readings is the number `certify` sees, only localised.
    fn readings(region: &Polygon, r: f64, e: f64, ctr: [f64; 2]) -> Vec<(f64, Point)> {
        let raw = front_advance_path(region, r, 0.0, e, Some(ctr))
            .expect("front-advance produces a path");
        let mut m = crate::clearsim::ClearedModel::bounded(r, region.clone());
        m.seed_disc(raw[0]);
        let mut out = Vec::with_capacity(raw.len() * 2);
        for w in raw.windows(2) {
            for piece in densify(w, 1.0).windows(2) {
                out.push((m.engagement(piece[0], piece[1]), piece[0]));
            }
            m.commit(w[0], w[1]);
        }
        out
    }

    /// The two reference measurements, computed once — each is a few seconds of exact
    /// oracle, and every test below interrogates the same two paths.
    fn circle_readings() -> &'static [(f64, Point)] {
        static C: std::sync::OnceLock<Vec<(f64, Point)>> = std::sync::OnceLock::new();
        C.get_or_init(|| readings(&circle(30.0, 96), R, E, [0.0, 0.0]))
    }

    fn square_readings() -> &'static [(f64, Point)] {
        static C: std::sync::OnceLock<Vec<(f64, Point)>> = std::sync::OnceLock::new();
        C.get_or_init(|| readings(&square(40.0), R, E, [20.0, 20.0]))
    }

    /// Peak `a_e` over the readings taken where `keep` holds.
    fn peak(rs: &[(f64, Point)], keep: impl Fn(Point) -> bool) -> f64 {
        rs.iter().filter(|(_, p)| keep(*p)).fold(0.0f64, |m, (ae, _)| m.max(*ae))
    }

    /// Distance to the nearest corner of the `s`-square.
    fn d_corner(p: Point, s: f64) -> f64 {
        [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s)]
            .iter()
            .map(|c| p.distance(Point::new(c.0, c.1)))
            .fold(f64::MAX, f64::min)
    }

    /// Generation is quick and produces sane point counts — the perf milestone. The
    /// cleared frontier is decimated each pass, so repeated round offsets no longer
    /// balloon its vertex count, and a coverage-based termination (not an area-delta,
    /// which the decimation jitter tripped) keeps the pass count tight.
    #[test]
    fn front_advance_generation_is_quick() {
        let (r, e) = (3.0, 2.0);
        for (name, region, ctr) in [
            ("square40", square(40.0), [20.0, 20.0]),
            ("circle r9", circle(9.0, 40), [0.0, 0.0]),
            ("circle r30", circle(30.0, 96), [0.0, 0.0]),
        ] {
            let t = std::time::Instant::now();
            let path = front_advance_path(&region, r, 0.0, e, Some(ctr))
                .unwrap_or_else(|| panic!("{name}: should produce a path"));
            let gen = t.elapsed().as_secs_f64();
            assert!(gen < 3.0, "{name}: generation should be quick, took {gen:.2}s");
            assert!(path.len() < 4000, "{name}: sane point count, got {}", path.len());
        }
    }

    /// The seam-localised connection is load-bearing: it removes the **radial link
    /// slot**, which was the generator's dominant defect. Before it, the pass-to-pass
    /// links jumped a stepover straight outward into virgin stock, so the tool's whole
    /// leading half was engaged and the exact oracle read the full **diameter (5.78 of
    /// 6.0)** — right through the body of the path, on *every* shape, not just cornered
    /// ones. The body now sits at 2.79 (1.4·e).
    ///
    /// Measured on shapes large enough to actually contain the defect. That matters: the
    /// test this replaces measured circle r=9 — two or three loops, excluding everything
    /// within 2·e of the centre — so the shape could not exhibit the link slot at all,
    /// and it passed while asserting the links held at 1.7·e. They did not.
    #[test]
    fn the_seam_connection_removes_the_radial_link_slot() {
        for (name, rs, ctr) in [
            ("circle r30", circle_readings(), Point::new(0.0, 0.0)),
            ("square 40", square_readings(), Point::new(20.0, 20.0)),
        ] {
            // Away from the entry and from any concave corner — i.e. where only the
            // links used to spike. Was ≈ the diameter here; now near the cap.
            let body =
                peak(rs, |p| p.distance(ctr) > 3.0 * E && d_corner(p, 40.0) > 8.0 * E);
            assert!(body <= 1.5 * E, "{name}: link slot should be gone, body a_e {body}");
        }
    }

    /// The **body** of the path holds engagement near the cap **shape-independently**: a
    /// round pocket and a square pocket both read exactly 2.79 (1.4·e) away from the
    /// entry and the corners. That sameness is the evidence the frontier tracking itself
    /// is sound — what is left over is localised to the entry and the corners rather
    /// than smeared through the path.
    #[test]
    fn the_path_body_holds_engagement_near_the_cap_on_every_shape() {
        let body = peak(circle_readings(), |p| p.distance(Point::new(0.0, 0.0)) > 3.0 * E);
        assert!(body <= 1.5 * E, "round pocket body should sit near the cap, got {body}");

        // A corner's influence reaches ~8 stepovers along the walls into it (measured);
        // beyond that the square matches the circle exactly.
        let body = peak(square_readings(), |p| {
            p.distance(Point::new(20.0, 20.0)) > 3.0 * E && d_corner(p, 40.0) > 8.0 * E
        });
        assert!(body <= 1.5 * E, "square pocket body should sit near the cap, got {body}");
    }

    /// **The known remaining gap, pinned honestly: the concave corner.** With the links
    /// connected, a square pocket's *corners* are what is left — a wall pass running into
    /// a concave corner has the uncut corner material wrap around its leading edge, so
    /// the engagement angle (and thus `a_e`) climbs to ~2.2·e. This is the classic HSM
    /// hard case, answered with corner-specific trochoidal peels.
    ///
    /// The corner is now an **isolable** signal, which is the point: it reads 4.41 against
    /// a 2.79 body, so a corner treatment can be measured. Previously the radial link slot
    /// (5.78) sat right through the body and swamped it. Tighten this when the corner lands.
    ///
    /// **This is real, not an artifact of the arc formula** — checked, because the uncut
    /// material there is only 1.30 mm deep, which looks like the formula over-reporting.
    /// It is not: `engagement_area` (material removed per unit advance, sharing no machinery
    /// with the arc formula) peaks at **the same point** and reads **4.27 against 4.41**. The
    /// shell is thin *radially* but wraps ~100° of the tool, and the tool advances **laterally
    /// across** it — so each mm of advance sweeps a 6 mm crescent lying almost entirely inside
    /// the shell. Thin in the direction that is easy to measure; wide in the one that matters.
    #[test]
    fn the_concave_corner_is_the_dominant_remaining_defect() {
        let rs = square_readings();
        let near = peak(rs, |p| d_corner(p, 40.0) <= 8.0 * E);
        let body = peak(rs, |p| {
            p.distance(Point::new(20.0, 20.0)) > 3.0 * E && d_corner(p, 40.0) > 8.0 * E
        });
        assert!(near > 2.0 * E, "corner over-engages (known gap), got {near}");
        assert!(near > body + 0.5 * E, "corner is the isolable peak: {near} vs body {body}");
    }

    /// The **entry** is opened by an Archimedean core spiral rather than a radial move
    /// from the plunge point, which slotted at the full diameter. Each turn bites at most
    /// a stepover, and it lands on the first frontier loop's seam so there is no jump onto
    /// it. The residual 3.52 (1.76·e) is a tighter gap than the corner's, and next after it.
    #[test]
    fn the_core_spiral_entry_does_not_slot() {
        let entry = peak(circle_readings(), |p| p.distance(Point::new(0.0, 0.0)) <= 3.0 * E);
        assert!(entry <= 2.0 * E, "core-spiral entry should not slot, got {entry} (was 6.0)");
    }

    /// The seam hand-off cannot be tuned into or out of trouble: the tool only ever bites
    /// the stepover it drifts outward across the hand-off, however far that is spread. So
    /// the peak is insensitive to [`SEAM_ARC`] (measured flat from 1 to 12 stepovers) —
    /// which is *why* the residual peaks are attributable to the entry and the corners
    /// rather than to the connection. Guards against a future re-tune chasing the wrong
    /// knob.
    #[test]
    fn the_seam_hand_off_length_is_not_a_tuning_knob() {
        let (r, e) = (3.0, 2.0);
        let region = square(40.0);
        let peaks: Vec<f64> = [1.0, 12.0]
            .iter()
            .map(|&sa| {
                let raw =
                    front_advance_tuned(&region, r, 0.0, e, Some([20.0, 20.0]), sa).unwrap();
                let mut m = crate::clearsim::ClearedModel::bounded(r, region.clone());
                m.seed_disc(raw[0]);
                let mut pk = 0.0f64;
                for w in raw.windows(2) {
                    pk = pk.max(m.engagement(w[0], w[1]));
                    m.commit(w[0], w[1]);
                }
                pk
            })
            .collect();
        assert!(
            (peaks[0] - peaks[1]).abs() < 0.1 * e,
            "peak should not depend on the hand-off length, got {peaks:?}"
        );
    }
}
