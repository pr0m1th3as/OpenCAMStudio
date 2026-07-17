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
//! - **concave corners** — as the front closes on a sharp vertex the trapped corner
//!   material *wraps* the tool, and engagement climbs the nearer it gets (4.41 at 2.2·e).
//!   **Answered**, by two halves that only work together: the frontier **stands off** sharp
//!   corners ([`STANDOFF_RADII`]) so no pass drives into one, and [`corner_relief`] spends
//!   **travel** on the wedge it declined. The corner now reads **2.79 — the body's own
//!   geometric floor** — and total travel *fell* 10%.
//!
//!   Travel is the only lever, and that is measured, not assumed: the wedge is sized by the
//!   **tool radius, not the stepover** (a round tool always leaves `r²(1−π/4)` at a sharp
//!   vertex), so an 8× cut in `e` moves the corner only 5.23 → 2.90 while `a_e/e` *climbs*
//!   2.6 → 11.6, with the independent area oracle agreeing throughout. Cutting finer in a
//!   corner is a dead end.
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

/// Interior angle (radians) at or below which a tool-centre vertex counts as a **sharp
/// corner** needing trochoidal relief. A right-angled pocket corner is π/2; blunter than
/// this and the collar no longer wraps enough for the spike to appear.
const SHARP_CORNER: f64 = 2.6; // ≈ 150°

/// Angular samples per revolution of a corner-relief loop.
const RELIEF_SAMPLES: usize = 64;

/// How far out along the bisector the relief starts, as a fraction of the largest loop
/// that still reaches the vertex (`r/(1−sin(half))`). Swept against the oracle: below
/// ~0.5 the loops stop reaching the vertex and the relief's *own* peak sticks at 3.21,
/// while above ~0.75 the extra travel buys nothing.
const RELIEF_REACH: f64 = 0.75;

/// Inward progress per relief turn, in stepovers. This **is** a real knob, unlike
/// [`SEAM_ARC`]: the corner's load is bought down with travel, so the peak trades against
/// path length. Swept (reach 0.75): pitch 1.0 → 20 mm of travel and a 3.31 peak; 0.35 →
/// 58 mm and 2.69; 0.20 → 97 mm and 2.58. 0.35 is the knee — it puts the corner just
/// under the body's own geometric floor (2.79), past which tightening it chases a defect
/// that is no longer the dominant one.
const RELIEF_PITCH: f64 = 0.35;

/// How far the frontier stands off a sharp corner, in tool radii. The corners it declines
/// are handed to [`corner_relief`], which spends travel on them instead.
///
/// Swept end-to-end through the real generator (40 mm square, r=3, e=2) as standoff →
/// (corner `a_e`, reachable mm² left uncut, total travel):
///
/// ```text
///   0·r  → (4.50, 0.00, 1373)      1.5·r → (3.00, 0.66, 1361)
///   1·r  → (3.42, 0.40, 1365)      2·r   → (2.79, 1.01, 1231)
/// ```
///
/// **2·r is the natural stopping point, not a tuning preference:** it puts the corner at
/// 2.79 — *exactly* the body's own geometric floor `a_e(ρ) = e(ρ+r)/ρ − e²/(2ρ)`. Past
/// that the corner is no longer distinguishable from the rest of the path, and no standoff
/// can beat the floor, which is inherent to any spiral clearer and not a defect of this
/// one. Travel *falls* 10% on the way, because the laps stop chasing corner material a
/// trochoid clears with less tool in the cut.
///
/// The cost is `uncut`: 1.0 mm² on a 1600 mm² pocket (0.06%) of reachable material. It is
/// real, it is charged to `certify`, and it is the price of the standoff.
///
/// Do not raise it further hoping for more. The standoff is **not monotonic** — measured
/// with the alternative clip-to-opened-`tc` formulation, 3·r read 4.85 and 4·r read 5.82,
/// *worse* than no standoff at all, because [`corner_relief`]'s reach is fixed while the
/// wedge grows as ρ², so the relief starts over-engaging on a wedge it can no longer
/// finish. A bigger standoff needs a bigger relief, together or not at all.
const STANDOFF_RADII: f64 = 2.0;

/// Area difference (mm²) below which two consecutive frontier passes count as the same
/// lap. Real passes differ by a stepover's worth of area — orders of magnitude more —
/// so this only catches the frontier standing still.
const DUP_LAP_TOL: f64 = 0.5;

fn total_area(polys: &[Polygon]) -> f64 {
    polys.iter().map(Polygon::area).sum()
}

/// Unsigned area of a closed ring (shoelace).
fn ring_area(ring: &[Point]) -> f64 {
    let n = ring.len();
    if n < 3 {
        return 0.0;
    }
    let s: f64 =
        (0..n).map(|i| ring[i].x * ring[(i + 1) % n].y - ring[(i + 1) % n].x * ring[i].y).sum();
    0.5 * s.abs()
}

/// Morphological opening of `polys` by `rho`: erode then dilate, rounding every convex
/// corner of the result to radius ≥ `rho` so it pulls back from sharp corners. Returns the
/// input unchanged when the erosion empties it — a pocket too small to stand off inside is
/// better cleared without a standoff than not at all.
///
/// Applied **per pass**, which is a deliberate choice over opening the tool-centre region
/// once and clipping each pass to it. Both were measured on the real generator (40 mm
/// square, r=3, e=2, standoff 1.5·r): clipping to a once-opened `tc` bottoms out at a
/// corner of **3.83** and gets *worse* past 2·r, while opening each pass reaches **2.69**.
/// The difference is real geometry, not bookkeeping: a pass is `disc ∩ tc`, so it has
/// convex corners where the advancing front's arc meets a wall, and opening the pass
/// rounds those too — pulling the front back from the corner harder than clipping to a
/// rounded `tc` ever does.
///
/// The cost is a known, small artifact: both offsets re-tessellate their round joins, so
/// an opened pass is inscribed and shrinks by a sagitta everywhere, not only at corners.
/// On a round pocket — which has no sharp corner and gets no relief — that alone leaves
/// **0.1 mm² uncut** (0.004% of the pocket) and reads the body at 2.58 rather than 2.79.
/// That is under-cutting flattering the oracle, and it is accepted only because it is
/// far below any scallop tolerance and buys a corner improvement of a whole 1.14 mm.
/// Guarding against it by skipping passes whose area barely changes does **not** work: the
/// guard cannot tell "nothing to round off" from "a marginal corner", and lets a
/// transitional pass drive into the vertex — measured, that put the corner back to 3.10.
fn standoff_open(polys: &[Polygon], rho: f64) -> Vec<Polygon> {
    if rho <= 1e-9 {
        return polys.to_vec();
    }
    match offset(polys, -rho, JoinStyle::Round) {
        Ok(er) if !er.is_empty() => match offset(&er, rho, JoinStyle::Round) {
            Ok(op) if !op.is_empty() => op,
            _ => polys.to_vec(),
        },
        _ => polys.to_vec(),
    }
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
    reliefs: &[Vec<Point>],
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
            // Outermost: a full closing lap finishes the wall, then the corner reliefs
            // take the corners the standoff declined. They go **last** because that is
            // when the state they need exists: everything but the corners is cleared, so
            // each relief dips into its wedge and retreats through cut stock, and the
            // links reaching them cross cleared ground rather than virgin stock.
            path.extend_from_slice(&cur[1..]);
            path.push(cur[0]);
            for rel in reliefs {
                path.extend_from_slice(rel);
            }
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

/// A sharp convex vertex of the tool-centre region: the corner the tool must sweep out.
/// `v` is the vertex, `bisector` the inward unit bisector, `half` the half interior angle.
#[derive(Clone, Copy, Debug)]
struct Corner {
    v: Point,
    bisector: (f64, f64),
    half: f64,
}

/// The sharp convex vertices of the tool-centre region's outer contour.
///
/// These are the pocket's **concave** corners seen from the tool: the walls close around
/// the tool there, so the uncut collar wraps its leading edge instead of facing it
/// head-on. Blunt vertices are skipped — no wrap, no spike, no relief needed.
fn sharp_corners(tc: &Polygon) -> Vec<Corner> {
    let pts = tc.outer().points();
    let n = pts.len();
    if n < 3 {
        return Vec::new();
    }
    // Winding sign: a CCW contour has positive signed area, and its convex vertices turn
    // left. Comparing each vertex's turn against this makes the test winding-agnostic.
    let area2: f64 =
        (0..n).map(|i| pts[i].x * pts[(i + 1) % n].y - pts[(i + 1) % n].x * pts[i].y).sum();
    let ccw = area2 > 0.0;
    let mut out = Vec::new();
    for i in 0..n {
        let (p, v, q) = (pts[(i + n - 1) % n], pts[i], pts[(i + 1) % n]);
        let unit = |a: Point, b: Point| {
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let l = dx.hypot(dy);
            (l > 1e-9).then(|| (dx / l, dy / l))
        };
        let (Some(inb), Some(outb)) = (unit(p, v), unit(v, q)) else {
            continue;
        };
        // Convex iff the turn matches the winding.
        let cross = inb.0 * outb.1 - inb.1 * outb.0;
        if (cross > 0.0) != ccw || cross.abs() < 1e-9 {
            continue;
        }
        // Interior angle between the two edges meeting at `v`.
        let (ba, bc) = ((-inb.0, -inb.1), outb);
        let theta = (ba.0 * bc.0 + ba.1 * bc.1).clamp(-1.0, 1.0).acos();
        if theta > SHARP_CORNER {
            continue;
        }
        // Inward bisector: the two edge directions away from `v`, summed.
        let (bx, by) = (ba.0 + bc.0, ba.1 + bc.1);
        let bl = bx.hypot(by);
        if bl < 1e-9 {
            continue;
        }
        out.push(Corner { v, bisector: (bx / bl, by / bl), half: theta / 2.0 });
    }
    out
}

/// A **trochoidal relief** for one sharp corner: a shrinking spiral of maximal inscribed
/// tool-centre circles walking down the corner's bisector toward the vertex.
///
/// The loop centred at `v + s·bisector` has radius `s·sin(half)` — exactly its distance
/// to the two walls — so every loop is tangent to the walls and **cannot gouge**, at any
/// `s`. This is what escapes the fixed-radius trap: on an offset guide the distance to
/// `∂tc` is a *constant*, so "radius = distance to the wall" yields one radius that both
/// misses the vertex and overshoots the straight wall. Here the guide runs down the
/// bisector, so the same rule gives a radius that genuinely varies, and the medial-axis
/// transform guarantees the loops' union covers the corner wedge.
///
/// No inscribed loop ever reaches the vertex itself (that would need `sin(half) = 1`,
/// i.e. a straight wall), so this is a **pre-relief**: it takes the bulk of the wedge and
/// leaves the closing lap — which does pass through the vertex — to finish it. Coverage
/// is therefore still the frontier's job, and the relief cannot open a coverage gap.
///
/// The lever is **travel, not depth**. Measured: the corner load is set by the tool
/// radius, not the stepover — an 8× cut in `e` barely moves it, because the wedge a round
/// tool leaves at a sharp vertex is `r²(1−π/4)` however thinly you slice, and `a_e` is
/// material per unit *advance*. So the relief buys its reduction by spending path length
/// on the same wedge, which is exactly what `engagement_area` measures.
fn corner_relief(c: Corner, r: f64, e: f64) -> Vec<Point> {
    corner_relief_tuned(c, r, e, RELIEF_REACH, RELIEF_PITCH)
}

/// [`corner_relief`] with the reach factor and pitch exposed, so they can be swept.
fn corner_relief_tuned(c: Corner, r: f64, e: f64, reach: f64, pitch: f64) -> Vec<Point> {
    let sin_h = c.half.sin();
    if !(sin_h > 1e-6 && sin_h < 1.0 - 1e-9) || e <= 0.0 || r <= 0.0 {
        return Vec::new();
    }
    // Start far enough out that the loop's swept annulus still reaches past the vertex
    // (the tool spans `r` beyond its centre circle), and spiral **all the way in to the
    // vertex**. Running to zero is load-bearing, not tidiness: the loops are inscribed, so
    // a loop stopped at `s` leaves the tool centre `s(1−sin(half))` short of `v`, and the
    // crescent it never reaches is reachable material the frontier no longer revisits —
    // the standoff has removed the closing lap that used to pass through `v`. Measured
    // (r=3, e=2, standoff 4.5): stopping at `s_min` = 1.0 leaves 6.04 mm² uncut; at 0.20,
    // 1.57; at **0.0, 0.54** — better than the un-relieved generator's own 1.15 baseline,
    // and the last turns cost nothing (they run in stock the spiral just cleared).
    let s_max = reach * r / (1.0 - sin_h).max(0.05);
    let s_min = 0.0;
    if s_max <= 1e-9 {
        return Vec::new();
    }
    // One turn per `e` of inward progress along the bisector: the loop's reach toward the
    // vertex is `s − s·sin(half)`, so a step of `Δs` advances `Δs·(1 − sin(half))`.
    let advance = (1.0 - sin_h).max(1e-6);
    let turns = (((s_max - s_min) * advance / (e * pitch)).ceil()).max(1.0);
    let steps = (turns * RELIEF_SAMPLES as f64).ceil() as usize;
    // Local frame: `bisector` and its left-normal.
    let (bx, by) = c.bisector;
    let (nx, ny) = (-by, bx);
    (0..=steps)
        .map(|k| {
            let t = k as f64 / steps as f64;
            let s = s_max + (s_min - s_max) * t;
            let rho = s * sin_h;
            let phi = std::f64::consts::TAU * turns * t;
            // Start each turn pointing back down the bisector (into cleared stock) so the
            // loop dips toward the vertex and retreats through what it just cut.
            let (cs, sn) = ((phi + std::f64::consts::PI).cos(), (phi + std::f64::consts::PI).sin());
            let dir = (bx * cs + nx * sn, by * cs + ny * sn);
            Point::new(c.v.x + s * bx + rho * dir.0, c.v.y + s * by + rho * dir.1)
        })
        .collect()
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
    front_advance_tuned(region, r, finish, e, start, SEAM_ARC, STANDOFF_RADII * r)
}

/// [`front_advance_path`] with the seam hand-off length exposed, so it can be swept.
fn front_advance_tuned(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
    seam_arc: f64,
    standoff: f64,
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
    // Area and ring count of the previous pass, to notice the frontier standing still.
    let mut prev_pass: Option<(f64, usize)> = None;

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
        // **Corner standoff.** Opening the pass by `standoff` holds the frontier off the
        // sharp corners, which is where its engagement runs away: as the front closes on
        // a vertex the trapped corner material *wraps* the tool, and the load climbs the
        // nearer it gets (measured, per pass: 2.48 → 3.83 → 4.12 → 4.41 as the front's
        // closest approach falls 6.1 → 4.1 → 2.1 → 0.15 mm). The corners it declines are
        // handed to [`corner_relief`], which spends travel on them instead. Measured on a
        // 40 mm square (r=3, e=2): corner 4.41 → 2.69 at a standoff of 1.5·r, and total
        // travel *falls*, because the laps stop chasing material a trochoid clears better.
        //
        // This is **not** curvature capping. The front's turn radius at a corner is large
        // and *grows* (2 → 24 mm, measured); it is the standoff distance that matters.
        let pass = standoff_open(&pass, standoff);
        if pass.is_empty() {
            break;
        }
        split |= pass.len() > 1;
        // Stop once the frontier stops advancing. With a standoff it converges on the
        // stood-off region and then repeats it forever; without one it converges onto the
        // tool-centre boundary a hair before the coverage tolerance trips and emits the
        // closing lap twice. Both are the same defect — a lap that re-cuts what the last
        // one took — and it must hold however many loops the frontier is in, or a split
        // front never terminates and grinds on to `MAX_PASSES`.
        let area_now = total_area(&pass);
        if let Some((prev_area, prev_rings)) = prev_pass {
            if (area_now - prev_area).abs() < DUP_LAP_TOL {
                // Keep whichever lap encloses more — the later one reaches the wall exactly
                // where the earlier stopped a hair short.
                if area_now > prev_area {
                    loops.truncate(loops.len() - prev_rings);
                    for poly in &pass {
                        loops.push(simplify_ring(poly.outer().points(), SIMPLIFY_EPS));
                    }
                }
                break;
            }
        }
        prev_pass = Some((area_now, pass.len()));
        for poly in &pass {
            // Decimate as the frontier is: the standoff's offsets re-tessellate their round
            // joins, and an un-decimated ring carries that straight into the point count.
            loops.push(simplify_ring(poly.outer().points(), SIMPLIFY_EPS));
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
    // Trochoidal relief for each sharp corner of the tool-centre region. Every loop is a
    // maximal inscribed circle, so these cannot gouge however they are sequenced.
    let reliefs: Vec<Vec<Point>> =
        sharp_corners(&tc).into_iter().map(|c| corner_relief(c, r, e)).filter(|p| p.len() > 2).collect();
    let path = match (!split).then(|| connect_seam_spiral(entry, &loops, e, seam_arc, &reliefs)).flatten() {
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

    /// **The concave corner is answered — this test replaces the one that pinned it as the
    /// dominant defect.** A square pocket's corners now read **2.79 against a 2.79 body**:
    /// the corner is no longer distinguishable from the rest of the path, which is the
    /// strongest statement available, because 2.79 *is* the body's geometric floor
    /// `a_e(ρ) = e(ρ+r)/ρ − e²/(2ρ)`. Nothing can push a corner below the floor that the
    /// straight passes themselves sit on.
    ///
    /// It read **4.41** before, and the fix is two parts that only work together: the
    /// frontier **stands off** sharp corners ([`STANDOFF_RADII`]) so no pass drives into a
    /// vertex and lets the trapped material wrap the tool, and [`corner_relief`] then
    /// spends **travel** on the wedge the standoff declined. Travel *fell* 10% overall.
    ///
    /// Why travel is the only lever, measured and worth not re-litigating: the corner wedge
    /// is sized by the **tool radius, not the stepover** — a round tool always leaves
    /// `r²(1−π/4)` at a sharp vertex. An 8× cut in `e` moves the corner only 5.23 → 2.90
    /// while `a_e/e` *climbs* 2.6 → 11.6, and the independent area oracle agrees the whole
    /// way. Reaching for a smaller stepover in corners is a dead end.
    #[test]
    fn the_concave_corner_no_longer_over_engages() {
        let rs = square_readings();
        let near = peak(rs, |p| d_corner(p, 40.0) <= 8.0 * E);
        let body = peak(rs, |p| {
            p.distance(Point::new(20.0, 20.0)) > 3.0 * E && d_corner(p, 40.0) > 8.0 * E
        });
        assert!(near <= 1.5 * E, "corner should sit at the body's floor, got {near}");
        assert!(
            near <= body + 0.1 * E,
            "corner should be indistinguishable from the body: {near} vs {body}"
        );
    }

    /// The corner fix is a **pair**, and each half alone is worse than useless. Standoff
    /// without relief abandons the corners; relief without standoff finds them already
    /// eaten and cuts air (measured: it read 0.51 and bought nothing). Pinned by comparing
    /// against a standoff of zero, where the relief has nothing left to do.
    ///
    /// Also guards the **non-monotonicity**: the standoff cannot be raised on its own,
    /// because the relief's reach is fixed while the wedge grows as ρ².
    #[test]
    fn the_standoff_is_what_lets_the_relief_reach_the_corner() {
        let region = square(40.0);
        let bare =
            front_advance_tuned(&region, R, 0.0, E, Some([20.0, 20.0]), SEAM_ARC, 0.0).unwrap();
        let mut m = crate::clearsim::ClearedModel::bounded(R, region.clone());
        m.seed_disc(bare[0]);
        let mut worst = 0.0f64;
        for w in bare.windows(2) {
            for pc in densify(w, 1.0).windows(2) {
                if d_corner(pc[0], 40.0) <= 8.0 * E {
                    worst = worst.max(m.engagement(pc[0], pc[1]));
                }
            }
            m.commit(w[0], w[1]);
        }
        let with_standoff = peak(square_readings(), |p| d_corner(p, 40.0) <= 8.0 * E);
        assert!(
            worst > with_standoff + 0.5 * E,
            "without a standoff the relief cannot reach the corner: {worst} vs {with_standoff}"
        );
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
                    front_advance_tuned(&region, r, 0.0, e, Some([20.0, 20.0]), sa, STANDOFF_RADII * r)
                        .unwrap();
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


    /// A corner relief **cannot gouge**, at any pitch or reach, because every loop is a
    /// maximal inscribed circle of the tool-centre region: its centre sits at `s` along
    /// the bisector and its radius is exactly that centre's distance to the walls. This is
    /// the invariant that escapes the fixed-radius trap, so it is pinned directly rather
    /// than inferred from a certified path — the guard against a future re-tune quietly
    /// pushing a loop through a wall.
    #[test]
    fn a_corner_relief_never_leaves_the_tool_centre_region() {
        let r = 3.0;
        for (name, region) in [
            ("square 40", square(40.0)),
            ("square 24", square(24.0)),
        ] {
            let tc =
                largest(offset(std::slice::from_ref(&region), -r, JoinStyle::Round).unwrap())
                    .unwrap();
            let corners = sharp_corners(&tc);
            assert_eq!(corners.len(), 4, "{name}: a square pocket has four sharp corners");
            for c in &corners {
                for (reach, pitch) in [(0.75, 0.35), (1.0, 1.0), (0.5, 0.2)] {
                    for p in corner_relief_tuned(*c, r, E, reach, pitch) {
                        assert!(
                            tc.contains(p),
                            "{name}: relief left the tool-centre region at ({:.3},{:.3}) \
                             (reach {reach}, pitch {pitch}) — that is a gouge",
                            p.x,
                            p.y
                        );
                    }
                }
            }
        }
    }

    /// A **round** pocket has no sharp corners, so it gets no relief. Guards the detector
    /// against firing on the many blunt vertices of a flattened circle, which would spend
    /// travel relieving corners that do not exist.
    #[test]
    fn a_round_pocket_has_no_sharp_corners_to_relieve() {
        let tc = largest(
            offset(std::slice::from_ref(&circle(30.0, 96)), -R, JoinStyle::Round).unwrap(),
        )
        .unwrap();
        assert!(sharp_corners(&tc).is_empty(), "a circle has no sharp corner");
    }

    /// The frontier converges onto the tool-centre boundary a hair before the coverage
    /// tolerance trips, which made it emit the closing lap **twice** — the same wall cut
    /// over again for nothing. The duplicate is dropped, so no two consecutive loops
    /// enclose the same area.
    #[test]
    fn the_frontier_does_not_emit_the_same_closing_lap_twice() {
        let path = front_advance_path(&square(40.0), R, 0.0, E, Some([20.0, 20.0])).unwrap();
        // The closing lap is the exact tool-centre square; cutting it twice would put two
        // separate visits to the same corner in the path.
        let visits = path.iter().filter(|p| p.distance(Point::new(37.0, 37.0)) < 1e-6).count();
        assert!(visits <= 1, "the closing lap's corner should be visited once, got {visits}");
    }


}
