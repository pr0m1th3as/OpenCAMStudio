//! Front-advance (cleared-region-tracking) adaptive clearing.
//!
//! Where the retired spiral-morph generator (`adaptive.rs`, **deleted 2026-07-29**) blended
//! pre-computed offset rings and *hoped* they held engagement — the exact oracle showed they
//! slot at the entry, at sharp corners, and at ring/handoff transitions — this advances the
//! **actual cleared region** outward by one stepover per pass.
//!
//! That module was kept for a while on the theory that its frame/trochoidal pieces were the
//! starting point for islands here. Measured before deleting it, they were not: its frame
//! path read the **full diameter** (6.00 against a cap of 2.00) *and* gouged the outer wall
//! by ~111 mm², its trochoidal entry channel being struck at radius `1.5·e` along a guide
//! only `5 mm` inside the wall with nothing bounding the two against each other. Islands are
//! reached from *this* module instead, via split-frontier connection. Full measurements in
//! `ADAPTIVE_PLAN.md` §5.
//!
//! Each pass's tool centres follow `offset(cleared ⊕ a, −r)`, so the tool reaches exactly
//! `a` beyond what is already cleared and the *loops themselves* peel that much by
//! construction. That is the frontier's guarantee, and it holds: measured by the exact
//! oracle ([`crate::clearsim`]), the body of the path reads **1.4·e on a round pocket and
//! a square pocket alike** — the sameness being the evidence that the tracking is sound.
//!
//! The advance `a` is **not** the stepover `e`. Engagement is not a function of the
//! stepover alone: at tool-centre radius ρ the tool's outer edge sweeps `(ρ+r)/ρ` times as
//! far as its centre travels, so the same advance bites harder the tighter the loop — the
//! geometric floor `a_e = e(ρ+r)/ρ − e²/(2ρ)`. [`pitch_for_cap`] inverts it, so each pass
//! advances what its radius can carry, relaxing to the full stepover by ρ≈5 (r=3, e=2).
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
//!   **Answered** (3.52 → 2.69, the floor). The residual was never the spiral: it was the
//!   **geometric floor on the innermost frontier loops**, which advanced a flat stepover
//!   however tight they were. The advance is now radius-aware ([`pitch_for_cap`]).
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

/// Default vertex-decimation tolerance (mm). Adaptive operations can override
/// it; legacy 2D clearing retains this value.
pub(crate) const SIMPLIFY_EPS: f64 = 0.02;

/// Bound the polygon checks for optional cleared-stock links on each 3D region.
pub(crate) const MAX_3D_LINK_CHECK_VERTICES: usize = 150_000;

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
/// **Do not raise it: 2·r is a hard ceiling set by an unsolved problem elsewhere.** Above
/// it the *opened* pass region breaks into pieces, the frontier `split`s, and
/// [`connect_seam_spiral`] is skipped for the nearest-point links, which slot. Measured on
/// square 40 at `ENTRY_TARGET`=1.0: 2.5·r and 3·r both read **6.00 — the full diameter, on
/// the body as well as the corner**, which is the fallback links, not a corner effect.
/// Raising [`RELIEF_REACH`] alongside does not rescue it (1.78 … 6.00 at 2.5·r), because
/// the relief was never what was failing.
///
/// So the corner's last residual (2.58 against a 2.30 gate) is **blocked on split-frontier
/// connection** — the same piece islands need. Solve that once and both unlock.
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
    let s: f64 = (0..n)
        .map(|i| ring[i].x * ring[(i + 1) % n].y - ring[(i + 1) % n].x * ring[i].y)
        .sum();
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

/// The corner standoff, applied **only where it is a corner standoff**.
///
/// [`standoff_open`] is a *global* morphological opening, so it deletes every feature of the
/// frontier narrower than `2·rho` — and the annular band between an island and the wall is
/// exactly such a feature. The tool then reaches that band with nothing cleared beside it
/// and cuts it at the **full diameter**. Measured over the island set, `a_e` falls
/// monotonically as the standoff is reduced and then cliffs to 6.00 the moment the opening
/// starts eating the band:
///
/// ```text
///   standoff            0.5·r  0.75·r   1·r  1.25·r  1.5·r   2·r
///   square 40, no island 3.73   3.62   3.42   3.21   3.00   2.90   ← wants the big standoff
///   square 40, island 12 4.85   3.93   3.52   3.31   6.00   6.00   ← cliffs at 1.5·r
///   circle r20, island r6 4.76  3.83   3.62   3.31   6.00   6.00   ← cliffs at 1.5·r
///   square 60, 2 islands 4.93   3.93   6.00   6.00   6.00   6.00   ← cliffs at 1·r
/// ```
///
/// There is no global value that serves both: the hole-free pocket is best at `2·r` and the
/// two-island one has already collapsed by `1·r`. But the tension is an artefact, because
/// the standoff exists to hold the front off **sharp corners** and nothing else. So take
/// what the opening removed, split it into components, and **give back every component that
/// is not against a sharp corner** — the wide-open pocket keeps its full corner standoff
/// while a narrow passage keeps its frontier.
///
/// **Two rejected attempts at making the standoff island-safe**, recorded so they are not
/// tried again. The sweep below shows `a_e` falling monotonically as the standoff is reduced
/// and then cliffing to 6.00, and the cliff moves with the shape — so the standoff is the
/// dominant lever on an island region, and no single global value serves every shape:
///
/// ```text
///   standoff             0.5·r  0.75·r   1·r  1.25·r  1.5·r   2·r
///   square 40, no island  3.73   3.62   3.42   3.21   3.00   2.90  ← wants the big standoff
///   square 40, island 12  4.85   3.93   3.52   3.31   6.00   6.00  ← cliffs at 1.5·r
///   circle r20, island r6 4.76   3.83   3.62   3.31   6.00   6.00  ← cliffs at 1.5·r
///   square 60, 2 islands  4.93   3.93   6.00   6.00   6.00   6.00  ← cliffs at 1·r
/// ```
///
/// **1. Restrict it to the corners it is named for** — give back every removed component not
/// against a sharp vertex of `tc`. Made every case *worse*: the one region that certified
/// (square 60 / island 20, 2.90) collapsed to 6.00. The standoff is not only a corner
/// device — it also holds the front off the concavity that forms as it **closes around an
/// island**, which is nowhere near a sharp vertex of the outer contour.
///
/// **2. Shrink it per pass when it costs too much area** — a ladder stepping down while the
/// opening removes more than a set fraction of the pass. Changed **nothing**, and the trace
/// says why: on the two cases that slot the opening removes **0.0% of the area** (6.3% at
/// worst on the third). The band is not being eaten, so the mechanism inferred from the
/// sweep is wrong. Whatever `rho` does here it does by **reshaping** the frontier — an
/// opening rounds reflex corners at radius `rho` while barely touching area — not by
/// deleting it. That is where the next investigation should start.
/// The largest polygon by area (the body of a boolean/offset result).
fn largest(polys: Vec<Polygon>) -> Option<Polygon> {
    polys.into_iter().max_by(|a, b| {
        a.area()
            .partial_cmp(&b.area())
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

/// Centroid of a contour's vertices.
fn centroid(poly: &Polygon) -> Point {
    let pts = poly.outer().points();
    let n = pts.len().max(1) as f64;
    let (sx, sy) = pts
        .iter()
        .fold((0.0, 0.0), |(sx, sy), p| (sx + p.x, sy + p.y));
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

/// Distance from `p` to the nearest point of `poly`'s boundary — **including its holes**,
/// so an island counts as a wall the plunge must stand clear of.
pub(crate) fn boundary_dist(poly: &Polygon, p: Point) -> f64 {
    let rings =
        std::iter::once(poly.outer().points()).chain(poly.holes().iter().map(|h| h.points()));
    let mut best = f64::MAX;
    for ring in rings {
        let n = ring.len();
        for k in 0..n {
            best = best.min(seg_dist(p, ring[k], ring[(k + 1) % n]));
        }
    }
    best
}

/// Where to plunge in the tool-centre region `tc`.
///
/// The centroid when it lies inside — which is every hole-free region, so this changes no
/// existing measurement. For a region with an **island** the centroid can land in the hole
/// (dead centre of an annulus is the one place the tool cannot be), so fall back to the
/// deepest interior point: a coarse pole of inaccessibility, which is also the best plunge
/// on its own merits — the most room around the tool.
pub(crate) fn entry_point(tc: &Polygon) -> Option<Point> {
    let c = centroid(tc);
    if tc.contains(c) {
        return Some(c);
    }
    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    for p in tc.outer().points() {
        lo[0] = lo[0].min(p.x);
        lo[1] = lo[1].min(p.y);
        hi[0] = hi[0].max(p.x);
        hi[1] = hi[1].max(p.y);
    }
    let step = ((hi[0] - lo[0]).max(hi[1] - lo[1]) / 64.0).max(1e-3);
    let mut best: Option<(f64, Point)> = None;
    let mut y = lo[1];
    while y <= hi[1] {
        let mut x = lo[0];
        while x <= hi[0] {
            let p = Point::new(x, y);
            if tc.contains(p) {
                let d = boundary_dist(tc, p);
                if best.is_none_or(|(bd, _)| d > bd) {
                    best = Some((d, p));
                }
            }
            x += step;
        }
        y += step;
    }
    best.map(|(_, p)| p)
}

/// Douglas–Peucker simplify of a closed ring at tolerance `eps` (iterative, so a
/// pathological ring can't blow the stack). Keeps the ring closed.
fn simplify_ring(ring: &[Point], eps: f64) -> Vec<Point> {
    let n = ring.len();
    if n < 8 || eps <= 0.0 {
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
pub(crate) fn simplify_polys(polys: &[Polygon], eps: f64) -> Vec<Polygon> {
    polys
        .iter()
        .filter_map(|p| {
            let outer = Contour::new(simplify_ring(p.outer().points(), eps));
            let holes: Vec<Contour> = p
                .holes()
                .iter()
                .map(|h| Contour::new(simplify_ring(h.points(), eps)))
                .collect();
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

/// Guard on the core spiral's turn count. The pitch shrinks toward the centre, so an
/// unbounded march would crawl out of a tiny radius forever.
const MAX_CORE_TURNS: usize = 200;

/// What the core spiral aims its engagement at, in stepovers. Set to the body's own
/// geometric floor (~1.4·e): the entry hands off to the frontier loops, and there is no
/// value in it being quieter than the path it hands off to — nor any way for it to be, as
/// the floor binds the loops too. Swept: see [`core_spiral_to_seam`].
const ENTRY_TARGET: f64 = 1.4;

/// Floor on the core spiral's pitch, in stepovers. [`pitch_for_cap`] tends to zero at the
/// centre (`cap·ρ/r`), which would stall the spiral at ρ=0 and never leave. The innermost
/// band is not where the defect is anyway — measured, ρ<1.5 already reads 2.79.
const MIN_CORE_PITCH: f64 = 0.25;

/// Engagement allowance the certifier grants over the nominal stepover `e`, as a multiple
/// of `e`. **This is the geometric floor, not slack for a sloppy path.** The engagement
/// parameter is the *straight-wall* stepover; at a tool-centre radius ρ the band the tool
/// removes sits at `2π(ρ+r)` while the centre travels `2πρ`, so the material-per-advance
/// floor `a_e(ρ) = e(ρ+r)/ρ − e²/(2ρ)` exceeds `e` at every finite radius and reaches
/// ~1.4·e on the tight loops a Ø6 tool cuts near a pocket centre — unreachable by *any*
/// spiral clearer, concentric included (which simply is never certified). A full-diameter
/// slot is 3·e, so 1.5·e cleanly separates "held to the floor" from "slotting": front-
/// advance's measured whole-path peak is 2.69–2.90 on r=3/e=2 (1.35–1.45·e), which passes;
/// the retired spiral-morph and the raster-gated links read 6.00, which does not.
pub(crate) const CERT_ENGAGEMENT_SLACK: f64 = 1.75;

/// The floor of the bound, in tool radii — **the part `SLACK·e` cannot express.**
///
/// Measured at a fixed 0.5 mm cadence, the peak barely tracks `e` at all: on three shapes the
/// *absolute* peak stays in 2.07–3.31 mm across `e` = 1.0, 1.5, 2.0, while the *ratio* swings
/// from 1.04·e to 3.00·e. It is set by the tool and the turn floor, not by what the operator
/// asked for — which is what the geometric floor `a_e(ρ) = e(ρ+r)/ρ − e²/(2ρ)` says too, since
/// the `(ρ+r)/ρ` factor dominates as ρ tightens and the `e` dependence weakens.
///
/// So a pure multiple of `e` is the wrong shape of rule: no multiplier below 3 admits these
/// paths at `e` = 1.0, and a multiplier of 3 at `e` = 2.0 would wave through a full-diameter
/// slot. 1.15·r = 3.45 mm at Ø6 clears the measured 2.79–3.00 at `e` = 1.0 with a little room.
pub(crate) const CERT_ENGAGEMENT_FLOOR: f64 = 1.15;

/// The hard ceiling, as a fraction of the tool **diameter** — the property this gate exists for.
///
/// The original rationale read "a full-diameter slot is 3·e, so 1.5·e separates held-to-the-floor
/// from slotting", which is true only at `e` = 2, r = 3. A slot is `2r` whatever `e` is, so at
/// `e` = 3 a 1.75·e bound would be 5.25 against a 6.00 slot — 14% of margin, and the retired
/// spiral-morph read exactly 6.00. Expressed against the diameter it cannot drift with `e`.
pub(crate) const CERT_SLOT_FRACTION: f64 = 0.75;

/// The peak radial width of cut a path may reach and still certify, in mm.
///
/// Two terms because two different things can bound it — the operator's request scaled by the
/// slack, or the floor the tool geometry imposes no matter how light the request — and one
/// ceiling, because neither may be allowed to approach slotting. See each constant.
pub(crate) fn engagement_bound(e: f64, r: f64) -> f64 {
    (e * CERT_ENGAGEMENT_SLACK)
        .max(r * CERT_ENGAGEMENT_FLOOR)
        .min(2.0 * r * CERT_SLOT_FRACTION)
}
/// Samples across a loop-to-loop seam transition.
const SEAM_SAMPLES: usize = 24;

/// How far apart two consecutive frontier rings may sit, in stepovers, for the seam
/// hand-off to be a valid connection between them.
///
/// **This is a correctness gate, not a tuning knob.** The hand-off blends ring `k` into ring
/// `k+1` at matched arc-length-back-from-the-seam, and that blend is only meaningful while
/// the two rings are *adjacent* — a stepover of drift apart, which is what the frontier
/// guarantees for rings one pass apart. When the frontier changes topology (the front closes
/// around an island and the outer contour snaps from a horseshoe to a plain wall loop) or
/// splits into components, consecutive rings are **not** adjacent, and blending between them
/// interpolates a chord straight across the region — through uncut stock, and through the
/// island itself if one lies between. Measured on square 40 with a 12 mm island, that chord
/// is what read 6.00 and gouged 76 mm².
///
/// So the hand-off is *checked* rather than assumed, and a ring pair that fails is connected
/// by a **rapid** instead. 2·e leaves generous room over the frontier's own advance (`adv ≤
/// e`) while a topology jump is tens of millimetres — the two are not close.
const MAX_HANDOFF_GAP: f64 = 2.0;

/// Sample spacing (mm) when testing whether a segment stays inside the tool-centre region.
/// Far below the tool radius, so a segment that leaves the region is caught while the tool
/// is still a long way from actually being outside the part.
const SEGMENT_SAMPLE: f64 = 0.4;

/// How far outside the tool-centre region a cutting move may stray before it counts as a
/// gouge, in mm.
///
/// **A tolerance, not a boolean, and that distinction is the whole test.** Frontier rings are
/// contours of regions built by repeated offsets, so their vertices sit *on* `tc`'s boundary
/// to within tessellation — the round-join flattening, the [`SIMPLIFY_EPS`] decimation, the
/// integer grid the booleans run on. Sampling between two such vertices dips microns outside
/// constantly and means nothing. A plain `!tc.contains(..)` test therefore condemns almost
/// every move on a curved wall: measured on a circle with an island, it turned **1383 of
/// 2852 moves into rapids** while the real worst excursion was 0.019 mm. The quantity that
/// distinguishes noise from a gouge is the *depth*, and the two are three orders of
/// magnitude apart — 0.019 mm of tessellation against 7.593 mm of chord through an island.
/// 0.1 mm sits between them with room to spare either way.
const GOUGE_TOL: f64 = 0.1;

/// **A cutting move that leaves the tool-centre region is not a cut, it is a gouge.**
/// Rewrite each such move as a rapid, so the tool lifts at the last legal point and
/// re-plunges at the next one instead of drawing a line through whatever lies between.
///
/// This is the backstop that makes islands safe, and it is deliberately about *segments*
/// rather than vertices. Every vertex on a front-advance path is legal by construction —
/// frontier rings are contours of a region inside `tc`, and the seam blend interpolates
/// between two of them — so a vertex test reports these paths as clean. What is not
/// guaranteed is the straight line *between* two legal vertices: across a region with an
/// island, `tc` is an annulus, and a chord joining two points on opposite sides of it runs
/// straight over the island. Measured on square 40 with a 12 mm island, that chord was a
/// single 42 mm step through the island's centre — the whole of a 76 mm² gouge, invisible
/// to every vertex-based check.
///
/// Sampling, not exact: a segment could in principle duck outside `tc` and back between two
/// samples. [`SEGMENT_SAMPLE`] is an order of magnitude below the tool radius, so anything
/// it misses is far smaller than the certifier's own tolerance — and the certifier still
/// has the last word regardless.
fn break_gouging_segments(path: &mut [(Point, bool)], tc: &Polygon) {
    for i in 1..path.len() {
        if !path[i].1 {
            continue; // already a rapid
        }
        let (a, b) = (path[i - 1].0, path[i].0);
        let n = ((a.distance(b) / SEGMENT_SAMPLE).ceil() as usize).max(1);
        let leaves = (0..=n).any(|k| {
            let t = k as f64 / n as f64;
            let p = Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
            !tc.contains(p) && boundary_dist(tc, p) > GOUGE_TOL
        });
        if leaves {
            path[i].1 = false;
        }
    }
}

/// Distance from `p` to the closed ring `ring` as a whole.
fn ring_dist(p: Point, ring: &[Point]) -> f64 {
    let n = ring.len();
    (0..n)
        .map(|k| seg_dist(p, ring[k], ring[(k + 1) % n]))
        .fold(f64::MAX, f64::min)
}

/// The worst separation between the two rings across a seam hand-off — the quantity
/// [`MAX_HANDOFF_GAP`] bounds.
///
/// Measured as the distance from each hand-off sample to **the other ring as a whole**, not
/// to that ring's arc-length-matched point. The difference matters: matched-point distance
/// is phase-dependent, and the phase of two rings measured back from their own seams drifts
/// with the hand-off length whenever their perimeters differ — so that version grows on
/// perfectly nested rings as `SEAM_ARC` rises, and would condemn a hand-off that is fine
/// (measured: it fired on a hole-free square at `seam_arc = 12`, where the peak engagement
/// is known to be flat). Distance to the ring is phase-independent and is what adjacency
/// actually means.
fn handoff_gap(cur: &[Point], ccur: &[f64], nxt: &[Point], cnxt: &[f64], delta: f64) -> f64 {
    let (total, tot_n) = (*ccur.last().unwrap_or(&0.0), *cnxt.last().unwrap_or(&0.0));
    let mut worst = 0.0_f64;
    for i in 0..=SEAM_SAMPLES {
        let t = i as f64 / SEAM_SAMPLES as f64;
        let a = at_len(cur, ccur, total - delta + t * delta);
        let b = at_len(nxt, cnxt, tot_n - delta + t * delta);
        worst = worst.max(ring_dist(a, nxt)).max(ring_dist(b, cur));
    }
    worst
}

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
    let i = cum
        .partition_point(|&c| c <= u)
        .saturating_sub(1)
        .min(pts.len() - 1);
    let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
    let seg = cum[i + 1] - cum[i];
    if seg <= 1e-12 {
        return a;
    }
    let t = (u - cum[i]) / seg;
    Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

/// The radial advance per turn that holds a spiral turn at `cap`, at tool-centre radius
/// `rho`. This is the geometric floor `a_e = e(ρ+r)/ρ − e²/(2ρ)` **inverted** for `e`:
///
/// ```text
///   e(ρ) = (ρ+r) − √((ρ+r)² − 2ρ·cap)
/// ```
///
/// It tends to `cap` as ρ→∞ (a straight peel bites its stepover) and to `cap·ρ/r` as ρ→0
/// (a tight turn must bite less, because the tool's outer edge sweeps `(ρ+r)/ρ` times as
/// far as its centre travels). That ratio is the whole reason a fixed-pitch core spiral
/// over-engages at the entry and nowhere else.
fn pitch_for_cap(rho: f64, r: f64, cap: f64) -> f64 {
    let a = rho + r;
    let d = a * a - 2.0 * rho * cap;
    if d <= 0.0 {
        return cap; // cap unreachable at this radius; take the stepover and let the caller clamp
    }
    a - d.sqrt()
}

/// Integrate a variable-pitch spiral out from `center`, scaling every pitch by `lambda`.
/// Returns the points and the angle at which `radius` was reached.
fn spiral_march(
    center: Point,
    radius: f64,
    r: f64,
    cap: f64,
    e: f64,
    lambda: f64,
) -> (Vec<Point>, f64) {
    let dth = std::f64::consts::TAU / CORE_SAMPLES as f64;
    let mut rho = 0.0_f64;
    let mut th = 0.0_f64;
    let mut pts = vec![center];
    // Bounded so a pathological pitch cannot spin forever.
    for _ in 0..(CORE_SAMPLES * MAX_CORE_TURNS) {
        if rho >= radius {
            break;
        }
        let p = (lambda * pitch_for_cap(rho, r, cap)).clamp(MIN_CORE_PITCH * e, e);
        rho = (rho + p * dth / std::f64::consts::TAU).min(radius);
        th += dth;
        pts.push(Point::new(
            center.x + rho * th.cos(),
            center.y + rho * th.sin(),
        ));
    }
    (pts, th)
}

/// A spiral from `center` out to `radius`, **ending on the +X seam** so it hands straight
/// over to the first frontier loop at that loop's seam with no radial jump. Without this
/// the entry is a radial move from the plunge point into virgin stock — a full slot.
///
/// The pitch is **not** constant, and that is the point. A fixed-pitch Archimedean spiral
/// bites a stepover per turn everywhere, but engagement is not a function of the stepover
/// alone: at tool-centre radius ρ the tool's outer edge sweeps `(ρ+r)/ρ` times as far as
/// its centre travels, so the same pitch bites harder the tighter the turn. Measured on
/// the old fixed-pitch entry (r=3, e=2, circle r30), bucketed by ρ: **3.52 at ρ≈2, 3.42 at
/// ρ≈4, 3.00 at ρ≈5, then the body's 2.5** — the entry residual was the first two turns and
/// nothing else. So each turn takes the pitch [`pitch_for_cap`] says it can afford, which
/// is ~1.4 near the middle and relaxes to the full stepover by ρ≈5.
///
/// Landing on the seam still needs a whole number of turns, so the profile is scaled by a
/// `lambda ≤ 1` found by bisection. Scaling **down** is what makes that safe: it only ever
/// slows the spiral out, which lowers engagement — the turn count is rounded up, never
/// down, so no turn is asked to bite more than it was sized for.
fn core_spiral_to_seam(center: Point, radius: f64, e: f64, r: f64, cap: f64) -> Vec<Point> {
    if radius <= 1e-6 || e <= 1e-6 {
        return vec![center];
    }
    let (_, th_full) = spiral_march(center, radius, r, cap, e, 1.0);
    let turns = (th_full / std::f64::consts::TAU).ceil().max(1.0);
    let target = std::f64::consts::TAU * turns;
    // Bisect the scale so the spiral reaches `radius` exactly as it crosses the seam.
    let (mut lo, mut hi) = (1e-3_f64, 1.0_f64);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        let (_, th) = spiral_march(center, radius, r, cap, e, mid);
        if th > target {
            lo = mid; // too slow — it overshot the turn budget
        } else {
            hi = mid;
        }
    }
    spiral_march(center, radius, r, cap, e, hi).0
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
    r: f64,
    cap: f64,
) -> Option<Vec<(Point, bool)>> {
    // Seam each ring on the +X ray from the entry. A ring that does not straddle that ray —
    // a component of a split frontier sitting off to one side — has no such seam; start it
    // at its own nearest point instead, since the hand-off into it will be a rapid anyway.
    let seamed: Vec<Vec<Point>> = loops
        .iter()
        .map(|l| {
            seam_rotate(l, entry).or_else(|| {
                let rot = crate::profile::rotate_to_start(l, Some([entry.x, entry.y]));
                (rot.len() >= 3).then_some(rot)
            })
        })
        .collect::<Option<_>>()?;
    let cums: Vec<Vec<f64>> = seamed.iter().map(|l| cum_len(l)).collect();

    // Open the core out to the first loop's seam, landing on it. The first point is the
    // plunge; everything after it cuts until a hand-off says otherwise.
    let core = core_spiral_to_seam(entry, seamed[0][0].distance(entry), e, r, cap);
    let mut path: Vec<(Point, bool)> = Vec::with_capacity(core.len());
    for (i, p) in core.iter().enumerate() {
        path.push((*p, i > 0));
    }

    for k in 0..seamed.len() {
        let (cur, ccur) = (&seamed[k], &cums[k]);
        let total = *ccur.last()?;
        let Some(nxt) = seamed.get(k + 1) else {
            // Outermost: a full closing lap finishes the wall, then the corner reliefs
            // take the corners the standoff declined. They go **last** because that is
            // when the state they need exists: everything but the corners is cleared, so
            // each relief dips into its wedge and retreats through cut stock, and the
            // links reaching them cross cleared ground rather than virgin stock.
            //
            // **Staging them before the lap was measured and is much worse — do not.** The
            // standoff protects the corner *wedge*, but not the *wall band*, which is still
            // uncut until this lap cuts it. A relief's loops reach the wall by construction,
            // so run early they drive into virgin stock: measured on square 40, relief
            // 0.84 → 3.93 and the corner 2.58 → 4.93.
            path.extend(cur[1..].iter().map(|&p| (p, true)));
            path.push((cur[0], true));
            for rel in reliefs {
                path.extend(rel.iter().map(|&p| (p, true)));
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
        // **Is this hand-off a connection at all?** Only if the two rings are adjacent; see
        // [`MAX_HANDOFF_GAP`]. When they are not — the frontier changed topology or split —
        // finish this ring and *rapid* to the next, rather than blending a chord across
        // whatever lies between.
        if handoff_gap(cur, ccur, nxt, cnxt, delta) > MAX_HANDOFF_GAP * e {
            path.extend(cur[1..].iter().map(|&p| (p, true)));
            path.push((cur[0], true)); // close this ring
            path.push((nxt[0], false)); // lift, reposition, plunge on the next
            continue;
        }
        // Cut this loop from its seam up to where the hand-off begins.
        for (i, p) in cur.iter().enumerate().skip(1) {
            if ccur[i] >= total - delta {
                break;
            }
            path.push((*p, true));
        }
        // Hand off: blend this loop's tail into the next loop's tail, so we arrive at
        // the next seam already travelling along it.
        for i in 0..=SEAM_SAMPLES {
            let t = i as f64 / SEAM_SAMPLES as f64;
            let a = at_len(cur, ccur, total - delta + t * delta);
            let b = at_len(nxt, cnxt, tot_n - delta + t * delta);
            path.push((
                Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t),
                true,
            ));
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
fn sharp_corners(tc: &Polygon, simplification_tolerance: f64) -> Vec<Corner> {
    // CAD layer projections can leave microscopic zigzags on a smooth wall.
    // Measure corners on the same simplified outline used for frontier rings,
    // otherwise their short edges trigger large, unnecessary corner reliefs.
    // This only classifies corners; the original tc still guards all motion.
    let pts = simplify_ring(tc.outer().points(), simplification_tolerance);
    let n = pts.len();
    if n < 3 {
        return Vec::new();
    }
    // Winding sign: a CCW contour has positive signed area, and its convex vertices turn
    // left. Comparing each vertex's turn against this makes the test winding-agnostic.
    let area2: f64 = (0..n)
        .map(|i| pts[i].x * pts[(i + 1) % n].y - pts[(i + 1) % n].x * pts[i].y)
        .sum();
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
        out.push(Corner {
            v,
            bisector: (bx / bl, by / bl),
            half: theta / 2.0,
        });
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
            let (cs, sn) = (
                (phi + std::f64::consts::PI).cos(),
                (phi + std::f64::consts::PI).sin(),
            );
            let dir = (bx * cs + nx * sn, by * cs + ny * sn);
            Point::new(c.v.x + s * bx + rho * dir.0, c.v.y + s * by + rho * dir.1)
        })
        .collect()
}

/// Generate a constant-engagement tool-centre path that clears `region` (leaving
/// `finish` skin on the walls) with a tool of radius `r`, holding the radial width of
/// cut at or below `e` by advancing the cleared front. `start` is the preferred entry
/// (part XY). Returns `None` when it cannot build a path (degenerate).
///
/// Returns `(point, is_cut)` **moves**, not bare points: a frontier that changes topology —
/// which is what an island makes it do — cannot be joined into one continuous cut, and the
/// honest representation of that is a rapid rather than a chord drawn across the part.
/// Hole-free regions produce no rapids at all beyond the initial positioning move, so their
/// paths are unchanged.
pub(crate) fn front_advance_path(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
) -> Option<Vec<(Point, bool)>> {
    front_advance_tuned(region, r, finish, e, start, Tuning::new(r, e))
}

/// [`front_advance_path`], certified against the **exact** oracle ([`crate::clearsim`]).
///
/// This is the entry [`crate::clearing::clear`] dispatches: it returns the tool-centre
/// path only if that path holds engagement at the geometric-floor bound
/// ([`CERT_ENGAGEMENT_SLACK`]·`e`), covers the reachable target, and never gouges —
/// otherwise `None`, meaning *fall back to concentric*. The bare [`front_advance_path`] is
/// left uncertified for the module's measurement tests, which need the raw path to
/// instrument.
///
/// **The gate is the exact oracle, never the raster.** With its old formula the raster read
/// 0.80 against a true 6.00 on square 24, which is precisely why the previous,
/// raster-gated adaptive dispatch shipped full-diameter cuts and was retired. That formula
/// has since been repaired and the raster now tracks the exact oracle closely on the cases
/// measured — which changes nothing here: its bias is no longer *characterised* in either
/// direction, so it stays an instrument and never a gate. The exact
/// oracle is O(path × rays) but runs once per op (the path is reused across depth levels),
/// so the trade is a second against a broken cutter.
pub(crate) fn front_advance_certified(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
    simplification_tolerance: f64,
) -> Option<Vec<(Point, bool)>> {
    if !simplification_tolerance.is_finite() || simplification_tolerance < 0.0 {
        return None;
    }
    let mut path = front_advance_tuned(
        region,
        r,
        finish,
        e,
        start,
        Tuning {
            simplification_tolerance,
            ..Tuning::new(r, e)
        },
    )?;
    compact_repositions(&mut path);
    // The material to remove (skin left on the walls) — the same region the generator
    // clears, recomputed here so the certifier scores against the true target. `largest`
    // keeps the holes: an island is a hole of this polygon, and the oracle reads it as
    // material that must not be touched rather than as stock to remove.
    let to_clear = largest(offset(std::slice::from_ref(region), -finish, JoinStyle::Round).ok()?)?;
    let reach = crate::clearsim::reachable(&to_clear, r);
    if reach.is_empty() {
        return None;
    }
    // Coverage tolerance: the standoff leaves ~1 mm² of reachable corner material uncut by
    // construction (charged here, deliberately — see [`STANDOFF_RADII`]), plus a small
    // area-proportional term for tessellation slivers.
    let cover_tol = 0.02 * total_area(&reach) + 1.0;
    let trimmed = trim_cleared_travel(&path, r, &to_clear);
    for candidate in trimmed.into_iter().chain(std::iter::once(path)) {
        let verdict = crate::clearsim::certify_moves(&candidate, r, &to_clear);
        if verdict.max_engagement <= engagement_bound(e, r)
            && verdict.uncut_area <= cover_tol
            && verdict.gouge_area <= cover_tol
        {
            return Some(candidate);
        }
    }
    None
}

/// Open the middle as a spiral, then advance each end on its own. The
/// engagement and footprint checks let irregular shapes keep the full laps.
pub(crate) fn clear_pocket_ends(
    region: &Polygon,
    r: f64,
    e: f64,
    tolerance: f64,
    original: &[(Point, bool)],
    flat_floor: bool,
    cancel: &crate::CancelToken,
) -> Option<LinkedFrontier> {
    let outline = region.outer().points();
    // The perimeter's dominant direction gives a rotation-independent long axis.
    let (cos2, sin2) =
        outline
            .iter()
            .zip(outline.iter().cycle().skip(1))
            .fold((0., 0.), |(cx, sy), (a, b)| {
                let (x, y) = (b.x - a.x, b.y - a.y);
                let length = x.hypot(y).max(cam_geo::GRID_MM);
                (cx + (x * x - y * y) / length, sy + 2. * x * y / length)
            });
    let angle = 0.5 * sin2.atan2(cos2);
    let (s, c) = angle.sin_cos();
    let coordinates = |p: Point| (p.x * c + p.y * s, -p.x * s + p.y * c);
    let span = |points: &[Point], across: bool| {
        let (lo, hi) = points
            .iter()
            .map(|&p| {
                let (u, v) = coordinates(p);
                if across {
                    v
                } else {
                    u
                }
            })
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v), hi.max(v))
            });
        hi - lo
    };
    if span(outline, false) < 3. * span(outline, true) || cancel.is_cancelled() {
        return None;
    }
    let tuning = Tuning {
        simplification_tolerance: tolerance,
        ..Tuning::new(r, e)
    };
    let f = frontier_tuned(region, r, 0., e, None, tuning)?;
    if f.split {
        return None;
    }
    let width = span(f.tc.outer().points(), true);
    let rings: Vec<_> = f
        .passes
        .into_iter()
        .map(|mut pass| pass.remove(0))
        .collect();
    let core = rings
        .iter()
        .position(|ring| span(ring, true) >= width - 0.1 * e)?;
    if rings.len() < core + 5 {
        return None;
    }
    let reliefs: Vec<_> = sharp_corners(&f.tc, tolerance)
        .into_iter()
        .map(|corner| {
            (
                corner.v,
                corner_relief_tuned(corner, r, e, tuning.relief_reach, RELIEF_PITCH),
            )
        })
        .filter(|(_, path)| path.len() > 2)
        .collect();
    let opening = connect_seam_spiral(
        f.entry,
        &rings[..=core],
        e,
        tuning.seam_arc,
        &[],
        r,
        tuning.entry_target,
    )?;
    let entry_u = coordinates(f.entry).0;
    let last_u = coordinates(opening.last()?.0).0;
    let first_side = if last_u >= entry_u { 1. } else { -1. };
    let mut passes = Vec::new();
    for side in [first_side, -first_side] {
        let along = |p: Point| side * (coordinates(p).0 - entry_u);
        let mut previous = &rings[core];
        for ring in &rings[core + 1..] {
            if cancel.is_cancelled() {
                return None;
            }
            let previous_tip = previous.iter().map(|&p| along(p)).fold(0., f64::max);
            // Carry the next cap back onto the two already opened side walls.
            // That overlap lets the tool round onto the front from cleared stock.
            let start = (previous_tip - 0.5 * width - r - e).max(0.);
            passes.push(clip_end(ring, along, start)?);
            previous = ring;
        }
        for (corner, relief) in &reliefs {
            if along(*corner) > 0. || (along(*corner) == 0. && side == first_side) {
                passes.push(relief.clone());
            }
        }
    }
    let length = |moves: &[(Point, bool)]| {
        moves
            .windows(2)
            .map(|pair| pair[0].0.distance(pair[1].0))
            .sum::<f64>()
    };
    let original_length = length(original);
    let cover_tol = 0.02 * total_area(&crate::clearsim::reachable(&f.to_clear, r)) + 1.;
    // Reordering the laps changes their seams. Allow a small seam shift
    // within the path tolerance when comparing retained stock, and check
    // any extra cut outside the machining area against the actual footprint.
    let edge_tolerance = (2. * tolerance).min(0.05);
    // A two-dimensional footprint proves floor clearance for flat tools.
    // A shaped tip can leave a scallop underneath that same footprint.
    let options: &[(bool, bool)] = if flat_floor {
        &[(true, true), (false, true), (false, false)]
    } else {
        &[(false, false)]
    };
    for &(trim_overlap, lift_links) in options {
        let mut path = connect_pocket_passes(
            &opening,
            &passes,
            &f.to_clear,
            r,
            trim_overlap,
            lift_links,
            cancel,
        )?;
        if length(&path.moves) > 0.95 * original_length {
            continue;
        }
        break_gouging_segments(&mut path.moves, &f.tc);
        if path.moves.iter().skip(1).any(|(_, cut)| !cut) {
            continue;
        }
        let cuts = path.cut_moves();
        let verdict = crate::clearsim::certify_moves(&cuts, r, &f.to_clear);
        if verdict.max_engagement <= engagement_bound(e, r)
            && verdict.uncut_area <= cover_tol
            && verdict.gouge_area <= cover_tol
            && swept_material_change(original, &cuts, r, &f.to_clear, edge_tolerance)
                .is_some_and(|(lost, added)| lost <= 0.01 && added <= 0.005)
        {
            return Some(path);
        }
    }
    None
}

fn connect_pocket_passes(
    opening: &[(Point, bool)],
    passes: &[Vec<Point>],
    material: &Polygon,
    r: f64,
    trim_overlap: bool,
    lift_links: bool,
    cancel: &crate::CancelToken,
) -> Option<LinkedFrontier> {
    use crate::clearsim::ClearedModel;

    let mut path = LinkedFrontier {
        moves: opening.to_vec(),
        air: vec![false; opening.len()],
        feed_links: vec![false; opening.len()],
    };
    let mut model = ClearedModel::bounded(r, material.clone());
    model.seed_disc(opening.first()?.0);
    for pair in opening.windows(2) {
        if !pair[1].1 {
            return None;
        }
        model.commit(pair[0].0, pair[1].0);
    }
    let mut footprint = ClearedFootprint {
        remaining_check_vertices: Some(MAX_3D_LINK_CHECK_VERTICES),
        ..Default::default()
    };
    for pass in passes {
        if cancel.is_cancelled() {
            return None;
        }
        let clipped;
        let pass = if trim_overlap {
            clipped = trim_pocket_pass(pass, &model);
            &clipped
        } else {
            pass
        };
        let from = path.moves.last()?.0;
        let to = pass[0];
        let anchors = if lift_links {
            pocket_link_anchors(from, to, &model, &path, &mut footprint, r)
        } else {
            None
        };
        if let Some([a, b]) = anchors {
            model.commit(from, a);
            model.commit(b, to);
            path.push(a, false);
            path.push(b, true);
        } else {
            model.commit(from, to);
        }
        path.push(to, false);
        for pair in pass.windows(2) {
            model.commit(pair[0], pair[1]);
            path.push(pair[1], false);
        }
    }
    Some(path)
}

/// Propose an entry and exit near the edge of fresh stock. Retain half a
/// millimetre of overlap at both ends. The final footprint check must still
/// account for wall skims thinner than a cell of the stock grid.
fn trim_pocket_pass(points: &[Point], model: &crate::clearsim::ClearedModel) -> Vec<Point> {
    const OVERLAP: f64 = 0.5;
    let distances = cum_len(points);
    let length = distances[points.len() - 1];
    let at = |d| at_len(points, &distances, d);
    let n = (length / OVERLAP).ceil() as usize;
    let clear = |i: usize| {
        let d = length * i as f64 / n as f64;
        let next = length * (i + 1) as f64 / n as f64;
        model.sweeps_only_cleared(at(d), at(next), 0.15)
    };
    let Some(first_cut) = (0..n).find(|&i| !clear(i)) else {
        return points.to_vec();
    };
    let last_cut = (first_cut..n).rfind(|&i| !clear(i)).unwrap_or(first_cut);
    let start = length * first_cut.saturating_sub(1) as f64 / n as f64;
    let end = length * (last_cut + 2).min(n) as f64 / n as f64;
    let mut trimmed = vec![at(start)];
    trimmed.extend(
        points
            .iter()
            .zip(distances.iter().copied())
            .filter(|(_, d)| start < *d && *d < end)
            .map(|(&point, _)| point),
    );
    trimmed.push(at(end));
    trimmed
}

/// Leave the wall at feed, lift across verified cleared stock, then feed
/// back down and onto the next pass. Shorter withdrawals get first choice.
fn pocket_link_anchors(
    from: Point,
    to: Point,
    model: &crate::clearsim::ClearedModel,
    path: &LinkedFrontier,
    footprint: &mut ClearedFootprint,
    r: f64,
) -> Option<[Point; 2]> {
    if footprint.remaining_check_vertices == Some(0) {
        return None;
    }
    let length = from.distance(to);
    let point = |d: f64| {
        let t = d / length;
        Point::new(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t)
    };
    for approach in [0.6, 1.2, 2.4, r.max(2.4)] {
        if length < 2. * approach + 2. {
            break;
        }
        let [a, b] = [point(approach), point(length - approach)];
        if model.sweeps_only_cleared(a, b, 0.15)
            && footprint.covers_traverse(path, a, b, r).unwrap_or(false)
        {
            return Some([a, b]);
        }
    }
    None
}

/// Keep the single open arc on the advancing side of the cross-section.
fn clip_end(ring: &[Point], along: impl Fn(Point) -> f64, start: f64) -> Option<Vec<Point>> {
    let vertices = ring.len();
    let entry = (0..vertices)
        .find(|&i| along(ring[i]) < start && along(ring[(i + 1) % vertices]) >= start)?;
    let mut arc = Vec::new();
    let mut crossed = false;
    for j in 0..vertices {
        let a = ring[(entry + j) % vertices];
        let b = ring[(entry + j + 1) % vertices];
        let (u, v) = (along(a), along(b));
        if (u < start) != (v < start) {
            if crossed {
                return None;
            }
            let t = (start - u) / (v - u);
            arc.push(Point::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
            crossed = u >= start;
        }
        if v > start {
            arc.push(b);
        }
    }
    (crossed && arc.len() >= 3).then_some(arc)
}

/// Stop following a frontier once it has reached an already machined wall.
/// The grid only proposes redundant runs; polygon booleans below must confirm
/// that the replacement preserves the original swept material. The normal
/// engagement certificate still judges the resulting path before emission.
fn trim_cleared_travel(
    path: &[(Point, bool)],
    r: f64,
    material: &Polygon,
) -> Option<Vec<(Point, bool)>> {
    use crate::clearsim::ClearedModel;

    // Exceed the diagonal of the model's largest (0.1 mm) occupancy cell.
    const CLEAR_MARGIN: f64 = 0.15;
    let first = *path.first()?;
    let mut model = ClearedModel::bounded(r, material.clone());
    model.seed_disc(first.0);
    let mut result = vec![first];
    let mut pending = Vec::new();
    let mut saved = 0.0;
    // Flush a run through cleared stock. Shorten it only when the entire cutter
    // sweep of the direct feed link is clear. Retaining the endpoint preserves
    // the start and direction of the next actual cut, without adding re-entries.
    let flush = |result: &mut Vec<(Point, bool)>,
                 pending: &mut Vec<Point>,
                 model: &ClearedModel,
                 saved: &mut f64| {
        if pending.is_empty() {
            return;
        }
        let from = result.last().unwrap().0;
        let to = *pending.last().unwrap();
        let mut previous = from;
        let travel: f64 = pending
            .iter()
            .map(|&p| {
                let length = previous.distance(p);
                previous = p;
                length
            })
            .sum();
        if travel - from.distance(to) > r * 0.25
            && model.sweeps_only_cleared(from, to, CLEAR_MARGIN)
        {
            result.push((to, true));
            *saved += travel - from.distance(to);
        } else {
            result.extend(pending.iter().map(|&p| (p, true)));
        }
        pending.clear();
    };
    for pair in path.windows(2) {
        let (from, (to, cut)) = (pair[0].0, pair[1]);
        if !cut {
            flush(&mut result, &mut pending, &model, &mut saved);
            result.push((to, false));
            model.seed_disc(to);
            continue;
        }
        // Long wall segments can have fresh material only at their ends.
        let n = (from.distance(to) / 0.5).ceil().max(1.0) as usize;
        let mut a = from;
        for k in 1..=n {
            let t = k as f64 / n as f64;
            let b = Point::new(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t);
            if model.sweeps_only_cleared(a, b, CLEAR_MARGIN) {
                pending.push(b);
            } else {
                flush(&mut result, &mut pending, &model, &mut saved);
                result.push((b, true));
                model.commit(a, b);
            }
            a = b;
        }
    }
    // No reason to traverse cleared stock after the final useful cut.
    let mut previous = result.last()?.0;
    for p in pending {
        saved += previous.distance(p);
        previous = p;
    }
    if saved < r * 0.25 {
        return None;
    }
    compact_repositions(&mut result);

    preserves_swept_material(path, &result, r, material, 1e-3).then_some(result)
}

pub(crate) fn preserves_swept_material(
    original: &[(Point, bool)],
    replacement: &[(Point, bool)],
    r: f64,
    material: &Polygon,
    tolerance: f64,
) -> bool {
    swept_material_change(original, replacement, r, material, 0.)
        .is_some_and(|(lost, added)| lost <= tolerance && added <= tolerance)
}

fn swept_material_change(
    original: &[(Point, bool)],
    replacement: &[(Point, bool)],
    r: f64,
    material: &Polygon,
    edge_tolerance: f64,
) -> Option<(f64, f64)> {
    use cam_geo::{difference, stroke_path, CapStyle, Polyline};
    let footprint = |moves: &[(Point, bool)]| -> Option<Vec<Polygon>> {
        let mut polys = Vec::new();
        for run in crate::clearsim::cut_runs(moves) {
            polys.extend(
                stroke_path(&Polyline::new(run), r, CapStyle::Round, JoinStyle::Round).ok()?,
            );
        }
        Some(polys)
    };
    let before = footprint(original)?;
    let after = footprint(replacement)?;
    // A grid cell can hide a sliver. Limit loss of actual stock and additional
    // gouging to polygon tessellation/grid noise, independently
    // of the much larger coverage allowance used by the clearing certificate.
    let expanded;
    let comparison = if edge_tolerance > 0. {
        expanded = offset(&after, edge_tolerance, JoinStyle::Round).ok()?;
        &expanded
    } else {
        &after
    };
    let lost = total_area(
        &intersection(
            &difference(&before, comparison).ok()?,
            std::slice::from_ref(material),
        )
        .ok()?,
    );
    let added = total_area(
        &difference(
            &difference(&after, &before).ok()?,
            std::slice::from_ref(material),
        )
        .ok()?,
    );
    Some((lost, added))
}

pub(crate) struct LinkedFrontier {
    pub moves: Vec<(Point, bool)>,
    pub air: Vec<bool>,
    pub feed_links: Vec<bool>,
}

impl LinkedFrontier {
    fn push(&mut self, point: Point, air: bool) {
        self.moves.push((point, true));
        self.air.push(air);
        self.feed_links.push(false);
    }

    fn cut_moves(&self) -> Vec<(Point, bool)> {
        self.moves
            .iter()
            .enumerate()
            .map(|(i, &(point, cut))| (point, cut && !self.air[i] && !self.feed_links[i]))
            .collect()
    }
}

/// Peel away from a finished wall before rapid travel, then feed back onto the
/// next cut. A small Z lift alone would still leave the cutter rubbing the wall.
/// Every rapid corridor must fit inside the wall allowance and already-cleared
/// stock. Its entire cutter footprint is checked, not just the tool centre.
pub(crate) fn link_cleared_travel(
    path: &[(Point, bool)],
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    cancel: &crate::CancelToken,
    check_vertex_budget: Option<usize>,
) -> Option<LinkedFrontier> {
    const MARGIN: f64 = 0.6;
    let material = largest(offset(std::slice::from_ref(region), -finish, JoinStyle::Round).ok()?)?;
    let guards = offset(
        std::slice::from_ref(region),
        -(r + finish + MARGIN),
        JoinStyle::Round,
    )
    .ok()?;
    if guards.is_empty() {
        return None;
    }
    let inset = |p: Point| -> Option<Point> {
        if guards.iter().any(|g| g.contains(p)) {
            return Some(p);
        }
        guards
            .iter()
            .map(|g| crate::profile::rotate_to_start(g.outer().points(), Some([p.x, p.y]))[0])
            .min_by(|a, b| a.distance(p).total_cmp(&b.distance(p)))
    };
    let mut model = crate::clearsim::ClearedModel::bounded(r, material.clone());
    let first = *path.first()?;
    model.seed_disc(first.0);
    let mut result = LinkedFrontier {
        moves: vec![first],
        air: vec![false],
        feed_links: vec![false],
    };
    let mut pending = Vec::new();
    let mut cleared = ClearedFootprint {
        remaining_check_vertices: check_vertex_budget,
        ..Default::default()
    };
    let mut flush = |result: &mut LinkedFrontier,
                     pending: &mut Vec<Point>,
                     model: &crate::clearsim::ClearedModel|
     -> Option<()> {
        if pending.is_empty() {
            return Some(());
        }
        // Leave room at both ends of a cleared run: its end caps can still
        // skim fresh wall material even when the middle has already been cut.
        let mut head = 0;
        let mut distance = 0.;
        let mut previous = result.moves.last().unwrap().0;
        let tail = pending.len();
        let anchor = |p: Point| -> Option<Point> {
            let base = inset(p)?;
            let eligible = |q: Point| {
                guards.iter().any(|g| g.contains(q))
                    && model.sweeps_only_cleared(q, q, MARGIN)
                    && model.sweeps_only_cleared(p, q, 0.15)
            };
            if eligible(base) {
                return Some(base);
            }
            // Also withdraw from the current material frontier, not only the
            // finished wall: a rapid endpoint needs clearance on every side.
            for distance in [MARGIN, 2. * MARGIN, 3. * MARGIN] {
                let candidate = (0..16)
                    .map(|k| {
                        let angle = std::f64::consts::TAU * k as f64 / 16.;
                        Point::new(
                            base.x + distance * angle.cos(),
                            base.y + distance * angle.sin(),
                        )
                    })
                    .filter(|&q| eligible(q))
                    .min_by(|a, b| a.distance(p).total_cmp(&b.distance(p)));
                if candidate.is_some() {
                    return candidate;
                }
            }
            None
        };
        let mut linked = false;
        let mut last_head = None;
        for attempt in 0..=6 {
            while head < tail && distance < r * attempt as f64 / 6. {
                let p = pending[head];
                distance += previous.distance(p);
                previous = p;
                result.moves.push((p, true));
                result.air.push(false);
                result.feed_links.push(false);
                head += 1;
            }
            if head == tail {
                break;
            }
            if last_head.replace(head) == Some(head) {
                continue;
            }
            let from = result.moves.last().unwrap().0;
            let to = pending[tail - 1];
            let (Some(a), Some(b)) = (anchor(from), anchor(to)) else {
                continue;
            };
            // Short transitions cost more in retract/approach than they save.
            if a.distance(b) >= 2.
                && model.sweeps_only_cleared(from, a, 0.15)
                && model.sweeps_only_cleared(a, b, MARGIN)
                && model.sweeps_only_cleared(b, to, 0.15)
                && segment_in_regions(a, b, &guards)
                && cleared
                    .permits_link(result, &pending[head..tail], [a, b], r, &material)
                    .unwrap_or(false)
            {
                result.moves.extend([(a, true), (b, true), (to, true)]);
                result.air.extend([false, true, false]);
                result.feed_links.extend([true, false, true]);
                linked = true;
                break;
            }
            if cleared.remaining_check_vertices == Some(0) {
                return None;
            }
        }
        if !linked {
            result
                .moves
                .extend(pending[head..tail].iter().map(|&p| (p, true)));
            result.air.resize(result.moves.len(), false);
            result.feed_links.resize(result.moves.len(), false);
        }
        pending.clear();
        Some(())
    };
    for pair in path.windows(2) {
        if cancel.is_cancelled() {
            return None;
        }
        let (from, (to, cut)) = (pair[0].0, pair[1]);
        if !cut {
            flush(&mut result, &mut pending, &model)?;
            result.moves.push((to, false));
            result.air.push(false);
            result.feed_links.push(false);
            model.seed_disc(to);
            continue;
        }
        let n = (from.distance(to) / 0.5 - 1e-9).ceil().max(1.) as usize;
        let mut a = from;
        for k in 1..=n {
            let t = k as f64 / n as f64;
            let b = if k == n {
                to
            } else {
                Point::new(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t)
            };
            if model.sweeps_only_cleared(a, b, 0.15) {
                pending.push(b);
            } else {
                flush(&mut result, &mut pending, &model)?;
                result.moves.push((b, true));
                result.air.push(false);
                result.feed_links.push(false);
                model.commit(a, b);
            }
            a = b;
        }
    }
    // Keep the final approach: the original trimmer already removed the tail.
    result.moves.extend(pending.iter().map(|&p| (p, true)));
    result.air.resize(result.moves.len(), false);
    result.feed_links.resize(result.moves.len(), false);
    if !result.air.iter().any(|&air| air) {
        return None;
    }
    let actual_cuts = result.cut_moves();
    // Splitting a stroked polyline adds tessellated round caps. Each link is
    // checked against its actual cleared prefix above; allow only their small
    // aggregate polygon-grid discrepancy here (0.005 mm²).
    if !preserves_swept_material(path, &actual_cuts, r, &material, 5e-3) {
        return None;
    }
    let v = crate::clearsim::certify_moves(&actual_cuts, r, &material);
    let reach = crate::clearsim::reachable(&material, r);
    let tol = 0.02 * total_area(&reach) + 1.;
    (v.max_engagement <= engagement_bound(e, r) && v.uncut_area <= tol && v.gouge_area <= tol)
        .then_some(result)
}

fn segment_in_regions(a: Point, b: Point, regions: &[Polygon]) -> bool {
    let n = (a.distance(b) / SEGMENT_SAMPLE).ceil().max(1.) as usize;
    (0..=n).all(|k| {
        let t = k as f64 / n as f64;
        let p = Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
        regions
            .iter()
            .any(|g| g.contains(p) || boundary_dist(g, p) <= cam_geo::GRID_MM)
    })
}

/// Bounding boxes of the actual stroked polygons. Keep contact within the
/// clipping grid in the exact Boolean tests as well.
fn footprint_bounds(polygons: &[Polygon]) -> [f64; 4] {
    polygons.iter().flat_map(|p| p.outer().points()).fold(
        [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ],
        |[left, bottom, right, top], p| {
            [left.min(p.x), bottom.min(p.y), right.max(p.x), top.max(p.y)]
        },
    )
}

#[derive(Default)]
pub(crate) struct ClearedFootprint {
    scanned: usize,
    active_run: Vec<Point>,
    active_stroke: Vec<Polygon>,
    completed: Vec<([f64; 4], Polygon)>,
    remaining_check_vertices: Option<usize>,
}

impl ClearedFootprint {
    pub(crate) fn bounded() -> Self {
        Self {
            remaining_check_vertices: Some(MAX_3D_LINK_CHECK_VERTICES),
            ..Default::default()
        }
    }

    fn update(&mut self, prefix: &LinkedFrontier, radius: f64) -> Result<(), cam_geo::GeoError> {
        // The prefix only grows. Once a cut is interrupted, its stroke never
        // changes. Keep re-stroking the last run as a whole for identical joins.
        let mut dirty = false;
        for i in self.scanned.max(1)..prefix.moves.len() {
            let (point, cut) = prefix.moves[i];
            if cut && !prefix.air[i] && !prefix.feed_links[i] {
                if self.active_run.is_empty() {
                    self.active_run.push(prefix.moves[i - 1].0);
                }
                self.active_run.push(point);
                dirty = true;
            } else if !self.active_run.is_empty() {
                if dirty {
                    self.active_stroke = stroke_run(&self.active_run, radius)?;
                }
                self.completed.extend(
                    self.active_stroke
                        .drain(..)
                        .map(|polygon| (footprint_bounds(std::slice::from_ref(&polygon)), polygon)),
                );
                self.active_run.clear();
                dirty = false;
            }
        }
        if dirty {
            self.active_stroke = stroke_run(&self.active_run, radius)?;
        }
        self.scanned = prefix.moves.len();
        Ok(())
    }

    /// A wall's first skim can be thinner than a grid cell. Check both the
    /// material a shortcut would skip and the footprint it would traverse.
    fn permits_link(
        &mut self,
        prefix: &LinkedFrontier,
        pending: &[Point],
        [a, b]: [Point; 2],
        radius: f64,
        material: &Polygon,
    ) -> Option<bool> {
        let from = prefix.moves.last()?.0;
        let to = *pending.last()?;
        let mut run = vec![from];
        run.extend_from_slice(pending);
        let sweep = stroke_run(&run, radius).ok()?;
        let corridor = stroke_run(&[from, a, b, to], radius).ok()?;
        self.check_footprints(prefix, &sweep, &corridor, radius, Some(material))
    }

    fn covers_traverse(
        &mut self,
        prefix: &LinkedFrontier,
        a: Point,
        b: Point,
        radius: f64,
    ) -> Option<bool> {
        self.covers_route(prefix, &[a, b], radius)
    }

    pub(crate) fn covers_route(
        &mut self,
        prefix: &LinkedFrontier,
        points: &[Point],
        radius: f64,
    ) -> Option<bool> {
        if self.remaining_check_vertices == Some(0) {
            return None;
        }
        // The safety check is geometric and includes stock just outside the
        // cutting region. Give the rapid a margin from its previously cut wall.
        let corridor = stroke_run(points, radius + 0.05).ok()?;
        self.check_footprints(prefix, &[], &corridor, radius, None)
    }

    fn check_footprints(
        &mut self,
        prefix: &LinkedFrontier,
        sweep: &[Polygon],
        corridor: &[Polygon],
        radius: f64,
        material: Option<&Polygon>,
    ) -> Option<bool> {
        use cam_geo::difference;

        let [sx0, sy0, sx1, sy1] = footprint_bounds(sweep);
        let [cx0, cy0, cx1, cy1] = footprint_bounds(corridor);
        let [x0, y0, x1, y1] = [sx0.min(cx0), sy0.min(cy0), sx1.max(cx1), sy1.max(cy1)];
        let grid = 2.0 * cam_geo::GRID_MM;
        let overlaps = |[left, bottom, right, top]: [f64; 4]| {
            left <= x1 + grid && right >= x0 - grid && bottom <= y1 + grid && top >= y0 - grid
        };
        if self.update(prefix, radius).is_err() {
            *self = Self {
                remaining_check_vertices: self.remaining_check_vertices,
                ..Default::default()
            };
            return None;
        }
        let center = Point::new(x0 + (x1 - x0) * 0.5, y0 + (y1 - y0) * 0.5);
        let mut cleared = Vec::new();
        for polygon in self
            .completed
            .iter()
            .filter(|(bounds, _)| overlaps(*bounds))
            .map(|(_, polygon)| polygon)
            .chain(
                self.active_stroke
                    .iter()
                    .filter(|polygon| overlaps(footprint_bounds(std::slice::from_ref(*polygon)))),
            )
        {
            // Long strokes may wrap around the query. When no boundary edge
            // can reach its bounds, the entire query is inside or outside.
            let touches_query =
                std::iter::once(polygon.outer())
                    .chain(polygon.holes())
                    .any(|ring| {
                        let points = ring.points();
                        points
                            .iter()
                            .zip(points.iter().cycle().skip(1))
                            .any(|(a, b)| {
                                overlaps([a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)])
                            })
                    });
            if touches_query {
                cleared.push(polygon.clone());
            } else if polygon.contains(center) {
                return Some(true);
            }
        }
        // The overlay intersects the cleared polygons with one another as
        // well. Bound repeated work on dense 3D regions and keep the original
        // certified frontier if the optional rewrites exceed the budget.
        if let Some(remaining) = &mut self.remaining_check_vertices {
            let vertices: usize = cleared
                .iter()
                .chain(sweep)
                .chain(corridor)
                .flat_map(|polygon| std::iter::once(polygon.outer()).chain(polygon.holes()))
                .map(|ring| ring.points().len())
                .sum();
            *remaining = remaining.saturating_sub(vertices);
            if *remaining == 0 {
                return None;
            }
        }
        if let Some(material) = material {
            let new = intersection(
                &difference(sweep, &cleared).ok()?,
                std::slice::from_ref(material),
            )
            .ok()?;
            if total_area(&new) > 1e-6 {
                return Some(false);
            }
        }
        Some(total_area(&difference(corridor, &cleared).ok()?) <= 1e-3)
    }
}

fn stroke_run(points: &[Point], radius: f64) -> Result<Vec<Polygon>, cam_geo::GeoError> {
    cam_geo::stroke_path(
        &cam_geo::Polyline::new(points.to_vec()),
        radius,
        cam_geo::CapStyle::Round,
        JoinStyle::Round,
    )
}

/// A rejected segment needs a lift, but its endpoint only needs a re-entry
/// when a cutting segment follows it. Runs of rejected segments otherwise
/// become repeated plunge/retract cycles along an unmachinable loop.
/// Keep the start and end of every cutting segment unchanged; certification
/// still runs on the resulting path before it can be emitted.
fn compact_repositions(path: &mut Vec<(Point, bool)>) {
    let mut kept = 0;
    for i in 0..path.len() {
        if path[i].1 || path.get(i + 1).is_some_and(|next| next.1) {
            path[kept] = path[i];
            kept += 1;
        }
    }
    path.truncate(kept);
}

/// The generator's tuning, bundled so it can be swept as a unit. Shape-control
/// defaults come from the measured constants above; simplification is also
/// configurable per adaptive operation. Certification gates remain unchanged.
#[derive(Clone, Copy, Debug)]
struct Tuning {
    /// Frontier approximation and corner-detection tolerance, mm; 0 disables it.
    simplification_tolerance: f64,
    /// Loop-to-loop seam hand-off length, in mm. See [`SEAM_ARC`].
    seam_arc: f64,
    /// How far the frontier stands off a sharp corner, in mm. See [`STANDOFF_RADII`].
    standoff: f64,
    /// Engagement the radius-aware advance and core spiral aim at, in mm. See
    /// [`ENTRY_TARGET`].
    entry_target: f64,
    /// How far out the corner reliefs start. See [`RELIEF_REACH`].
    ///
    /// **Measured: this does not move the corner.** Swept at standoff 2·r on square 40,
    /// reach 0.75 / 1.00 / 1.30 all leave the corner at **2.58**; it only quiets the
    /// relief's own reading (0.84 → 0.57) and costs travel (1402 → 1834 mm). The corner's
    /// residual is not relief-limited, so do not reach for this knob to chase it.
    relief_reach: f64,
}

impl Tuning {
    /// The shipped tuning for a tool of radius `r` at stepover `e`.
    fn new(r: f64, e: f64) -> Self {
        Self {
            seam_arc: SEAM_ARC,
            simplification_tolerance: SIMPLIFY_EPS,
            standoff: STANDOFF_RADII * r,
            entry_target: ENTRY_TARGET * e,
            relief_reach: RELIEF_REACH,
        }
    }
}

/// The generated frontier, before it is stitched into a path. Separating generation from
/// connection is what makes a split frontier tractable: the connection needs to know which
/// rings belong to the *same pass* (far apart, no shared seam) and which are a pass apart
/// (adjacent, a stepover of drift between them), and a flattened ring list cannot say.
struct Frontier {
    /// Approximation shared by frontier generation and corner classification.
    simplification_tolerance: f64,
    /// Where the tool plunges.
    entry: Point,
    /// The tool-centre region (the region eroded by `r + finish`).
    tc: Polygon,
    /// The material to remove (the region less the finish skin).
    to_clear: Polygon,
    /// Frontier rings, innermost pass first, each pass holding its components.
    passes: Vec<Vec<Vec<Point>>>,
    /// Whether any pass split into more than one component. Diagnostic only now — the
    /// connection no longer branches on it, since the hand-off is checked pair by pair.
    split: bool,
}

/// Advance the cleared region outward one stepover per pass, collecting the frontier.
/// This is the half of the generator that carries the engagement guarantee; connecting
/// what it produces is [`connect_seam_spiral`]'s problem, and a much harder one.
fn frontier_tuned(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
    tuning: Tuning,
) -> Option<Frontier> {
    let Tuning {
        standoff,
        entry_target,
        simplification_tolerance,
        ..
    } = tuning;
    let to_clear = largest(offset(std::slice::from_ref(region), -finish, JoinStyle::Round).ok()?)?;
    let tc = largest(
        offset(
            std::slice::from_ref(region),
            -(r + finish),
            JoinStyle::Round,
        )
        .ok()?,
    )?;

    let entry = preferred_entry(&tc, start, e.min(r) * 0.5).or_else(|| entry_point(&tc))?;

    let clear_slice = std::slice::from_ref(&to_clear);
    let to_clear_area = to_clear.area();
    // The frontier has reached every wall once `grown` fills the stock to within this.
    let covered_tol = 0.001 * to_clear_area + 0.5 * e * e;

    let mut cleared = vec![disc(entry, r)?];
    // The frontier, innermost pass first, **keeping each pass's components together**.
    // Collected rather than emitted as we go: the seam hand-off needs the *next* loop while
    // cutting the current one. The nesting is load-bearing for a split frontier — a ring's
    // neighbour is the ring of the adjacent *pass*, not whatever happens to sit next in a
    // flattened list, which may be a far-away component of the same pass.
    let mut passes: Vec<Vec<Vec<Point>>> = Vec::new();
    // A pass whose frontier split into several loops (a concavity pinching the front in
    // two, or an island it has begun to flow around).
    let mut split = false;
    // Area of the previous pass, to notice the frontier standing still.
    let mut prev_area: Option<f64> = None;

    for _ in 0..MAX_PASSES {
        // **Radius-aware advance.** The frontier cannot afford a full stepover near the
        // entry. At tool-centre radius ρ the tool's outer edge sweeps `(ρ+r)/ρ` times as
        // far as its centre travels, so the same advance bites harder the tighter the
        // loop — the geometric floor `a_e = e(ρ+r)/ρ − e²/(2ρ)`, which blows up as ρ→0.
        // [`pitch_for_cap`] inverts it for the advance this radius can carry.
        //
        // The innermost loops are what the "entry residual" always was. It was **not** the
        // core spiral: measured on a circle r30, the spiral spans only ρ ∈ [0,2] (the first
        // frontier loop's radius), while the 3.42 peak sits at ρ≈3.94 — the loop at ρ=4,
        // where the floor predicts 3.0. Retuning the spiral's pitch moved it not at all,
        // across targets from 1.4·e to 0.5·e.
        //
        // ρ is estimated from the cleared region's equivalent-disc radius, less the tool:
        // near the entry the frontier really is a disc, which is exactly where this binds.
        let rho = (total_area(&cleared) / std::f64::consts::PI).sqrt() - r;
        let adv = pitch_for_cap(rho.max(0.0), r, entry_target).clamp(MIN_CORE_PITCH * e, e);
        // Advance the frontier into fresh material, clipped to the stock.
        let grown =
            intersection(&offset(&cleared, adv, JoinStyle::Round).ok()?, clear_slice).ok()?;
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
        // Decimate as the frontier is: the standoff's offsets re-tessellate their round
        // joins, and an un-decimated ring carries that straight into the point count.
        let rings: Vec<Vec<Point>> = pass
            .iter()
            .map(|poly| {
                simplify_ring(poly.outer().points(), simplification_tolerance)
                    .into_iter()
                    .map(|p| snap_to_wall(p, &tc, simplification_tolerance.min(0.02)))
                    .collect()
            })
            .collect();
        if let Some(prev) = prev_area {
            if (area_now - prev).abs() < DUP_LAP_TOL {
                // Keep whichever lap encloses more — the later one reaches the wall exactly
                // where the earlier stopped a hair short.
                if area_now > prev {
                    passes.pop();
                    passes.push(rings);
                }
                break;
            }
        }
        prev_area = Some(area_now);
        passes.push(rings);
        // Advance: the new cleared region is what the tool cut (opening of `grown`),
        // decimated at the requested tolerance to limit vertex growth from
        // repeated round offsets. The completed path still has to certify.
        let opened = offset(&pass, r, JoinStyle::Round).ok()?;
        cleared = simplify_polys(
            &union(&cleared, &opened).ok().unwrap_or(opened),
            simplification_tolerance,
        );
        // Done once the frontier fills the stock — stable, unlike an area-delta check
        // (which the decimation jitter would trip early or never).
        if to_clear_area - total_area(&grown) < covered_tol {
            break;
        }
    }
    if passes.is_empty() {
        return None;
    }
    Some(Frontier {
        entry,
        tc,
        to_clear,
        passes,
        split,
        simplification_tolerance,
    })
}

/// Round-offset tessellation can leave each frontier a few microns short of a
/// wall. Reach that wall once, within the requested approximation tolerance,
/// instead of repeatedly skimming the same finished wall as the front grows.
fn snap_to_wall(p: Point, tc: &Polygon, tolerance: f64) -> Point {
    if tolerance <= 0. {
        return p;
    }
    let mut nearest = p;
    let mut distance = tolerance;
    for contour in std::iter::once(tc.outer()).chain(tc.holes().iter()) {
        let points = contour.points();
        for i in 0..points.len() {
            let a = points[i];
            let b = points[(i + 1) % points.len()];
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let length2 = dx * dx + dy * dy;
            if length2 <= 1e-18 {
                continue;
            }
            let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / length2).clamp(0., 1.);
            let q = Point::new(a.x + t * dx, a.y + t * dy);
            if p.distance(q) <= distance {
                nearest = q;
                distance = p.distance(q);
            }
        }
    }
    nearest
}

/// Resolve a model-space preference onto an eligible cutter-centre region.
pub(crate) fn preferred_entry(tc: &Polygon, start: Option<[f64; 2]>, margin: f64) -> Option<Point> {
    let start = start.filter(|p| p.iter().all(|v| v.is_finite()))?;
    let point = Point::new(start[0], start[1]);
    if tc.contains(point) {
        return Some(point);
    }
    let inset = offset(std::slice::from_ref(tc), -margin, JoinStyle::Round).ok()?;
    inset
        .iter()
        .flat_map(|p| std::iter::once(p.outer()).chain(p.holes().iter()))
        .map(|contour| crate::profile::rotate_to_start(contour.points(), Some(start))[0])
        .filter(|p| tc.contains(*p))
        .min_by(|a, b| {
            ((a.x - point.x).powi(2) + (a.y - point.y).powi(2))
                .total_cmp(&((b.x - point.x).powi(2) + (b.y - point.y).powi(2)))
        })
}

#[cfg(test)]
fn frontier(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
    standoff: f64,
    entry_target: f64,
) -> Option<Frontier> {
    frontier_tuned(
        region,
        r,
        finish,
        e,
        start,
        Tuning {
            standoff,
            entry_target,
            ..Tuning::new(r, e)
        },
    )
}

/// [`front_advance_path`] with the tuning exposed, so it can be swept.
fn front_advance_tuned(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
    t: Tuning,
) -> Option<Vec<(Point, bool)>> {
    let Tuning {
        seam_arc,
        entry_target,
        relief_reach,
        ..
    } = t;
    if !(e > 0.0 && e < 2.0 * r) {
        return None;
    }
    let f = frontier_tuned(region, r, finish, e, start, t)?;
    let (path, _) = connect(&f, r, e, seam_arc, relief_reach, entry_target)?;
    (path.len() > 3).then_some(path)
}

/// Stitch a generated [`Frontier`] into one continuous tool-centre path. Returns the path
/// and whether the **seam spiral** carried it — `false` means it fell back to the plain
/// nearest-point links, which slot at the full diameter and which the caller's
/// certification is expected to reject.
fn connect(
    f: &Frontier,
    r: f64,
    e: f64,
    seam_arc: f64,
    relief_reach: f64,
    entry_target: f64,
) -> Option<(Vec<(Point, bool)>, bool)> {
    let loops: Vec<Vec<Point>> = f.passes.iter().flatten().cloned().collect();
    // Trochoidal relief for each sharp corner of the tool-centre region. Every loop is a
    // maximal inscribed circle, so these cannot gouge however they are sequenced.
    let reliefs: Vec<Vec<Point>> = sharp_corners(&f.tc, f.simplification_tolerance)
        .into_iter()
        .map(|c| corner_relief_tuned(c, r, e, relief_reach, RELIEF_PITCH))
        .filter(|p| p.len() > 2)
        .collect();
    // The seam spiral is attempted for **every** frontier now, split or not: the hand-off
    // gap check ([`MAX_HANDOFF_GAP`]) is what decides ring by ring whether a blend is a
    // connection or a chord across the region, and a rapid carries the pairs it refuses. A
    // split frontier is just the case where several of those checks fail in a row, so it no
    // longer needs its own (slotting) code path — `f.split` is now only a diagnostic.
    match connect_seam_spiral(f.entry, &loops, e, seam_arc, &reliefs, r, entry_target) {
        Some(mut p) => {
            // Whatever the hand-off check let through, no cutting move may leave the
            // tool-centre region.
            break_gouging_segments(&mut p, &f.tc);
            Some((p, true))
        }
        None => {
            // Last resort: the seam construction itself failed (a degenerate ring). The
            // plain links slot, and the caller's certification is expected to reject them.
            let mut p = vec![f.entry];
            let mut prev = f.entry;
            for l in &loops {
                append_loop(&mut p, l, &mut prev);
            }
            let mut moves: Vec<(Point, bool)> =
                p.iter().enumerate().map(|(i, &q)| (q, i > 0)).collect();
            break_gouging_segments(&mut moves, &f.tc);
            Some((moves, false))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejected_segments_only_reenter_before_a_cut() {
        let mut path = vec![
            (Point::new(4., 4.), false),
            (Point::new(8., 4.), true),
            (Point::new(12., 4.), false),
            (Point::new(16., 4.), false),
            (Point::new(20., 4.), false),
            (Point::new(24., 4.), true),
            (Point::new(28., 4.), false),
            (Point::new(32., 4.), false),
        ];
        let cutting_segments = |moves: &[(Point, bool)]| {
            moves
                .windows(2)
                .filter(|pair| pair[1].1)
                .map(|pair| (pair[0].0, pair[1].0))
                .collect::<Vec<_>>()
        };
        let original_segments = cutting_segments(&path);
        let before = crate::clearsim::certify_moves(&path, R, &square(40.));
        compact_repositions(&mut path);
        assert_eq!(path.len(), 4);
        assert_eq!(cutting_segments(&path), original_segments);
        assert!(path.windows(2).all(|pair| pair[0].1 || pair[1].1));
        assert!(path.last().unwrap().1, "no re-entry after the last cut");
        let after = crate::clearsim::certify_moves(&path, R, &square(40.));
        assert_eq!(after.max_engagement, before.max_engagement);
        assert_eq!(after.uncut_area, before.uncut_area);
        assert_eq!(after.gouge_area, before.gouge_area);
    }

    #[test]
    fn entirely_rejected_path_has_no_reentries() {
        let mut path = vec![(Point::new(4., 4.), false), (Point::new(8., 4.), false)];
        compact_repositions(&mut path);
        assert!(path.is_empty());
    }

    /// Reference tool radius and engagement cap for the measured tests.
    const R: f64 = 3.0;
    const E: f64 = 2.0;

    #[test]
    fn elongated_pocket_clears_each_end_in_turn() {
        let (s, c) = 0.37_f64.sin_cos();
        let blank = Polygon::new(Contour::new(
            [(-36., -4.), (36., -4.), (36., 4.), (-36., 4.)]
                .map(|(u, v)| Point::new(u * c - v * s, u * s + v * c))
                .to_vec(),
        ))
        .unwrap();
        let region = offset(&[blank], 7., JoinStyle::Round).unwrap().remove(0);
        let original = front_advance_certified(&region, R, 0., 2.4, None, 0.025).unwrap();
        let ends = clear_pocket_ends(
            &region,
            R,
            2.4,
            0.025,
            &original,
            true,
            &crate::CancelToken::new(),
        )
        .expect("a long, rounded pocket should use the shorter end passes");
        let length = |path: &[(Point, bool)]| {
            path.windows(2)
                .map(|w| w[0].0.distance(w[1].0))
                .sum::<f64>()
        };
        assert!(length(&ends.moves) < 0.95 * length(&original));
        assert!(ends.air.iter().any(|&lifted| lifted));
        let mut visited = Vec::new();
        for (p, _) in ends.moves {
            let u = p.x * c + p.y * s;
            if u.abs() > 22. && visited.last() != Some(&u.is_sign_positive()) {
                visited.push(u.is_sign_positive());
            }
        }
        assert_eq!(visited.len(), 2);
    }

    #[test]
    fn pocket_pass_trims_only_the_cleared_ends() {
        let p = Point::new;
        let mut model = crate::clearsim::ClearedModel::bounded(R, square(40.));
        for y in [10., 12., 14.] {
            model.commit(p(4., y), p(36., y));
        }
        let pass = [
            p(5., 12.),
            p(16., 12.),
            p(16., 20.),
            p(24., 20.),
            p(24., 12.),
            p(35., 12.),
        ];
        let trimmed = trim_pocket_pass(&pass, &model);
        assert!(trimmed.first().unwrap().x > 10.);
        assert!(trimmed.last().unwrap().x < 30.);
        assert!(trimmed.contains(&p(16., 20.)) && trimmed.contains(&p(24., 20.)));
    }

    #[test]
    fn an_air_run_cannot_license_a_shortcut_through_virgin_stock() {
        let p = |x, y| Point::new(x, y);
        let prefix = LinkedFrontier {
            moves: vec![
                (p(5., 5.), false),
                (p(35., 5.), true),
                (p(35., 35.), true),
                (p(5., 35.), true),
                (p(5., 5.), true),
            ],
            air: vec![false; 5],
            feed_links: vec![false; 5],
        };
        let material = square(40.);
        let along_wall = [p(35., 5.)];
        let mut limited = ClearedFootprint {
            remaining_check_vertices: Some(1),
            ..Default::default()
        };
        assert_eq!(
            limited.permits_link(&prefix, &along_wall, [p(5., 5.), p(35., 5.)], R, &material),
            None
        );
        assert_eq!(limited.remaining_check_vertices, Some(0));
        let mut cleared = ClearedFootprint::default();
        assert_eq!(
            cleared.permits_link(&prefix, &along_wall, [p(5., 5.), p(35., 5.)], R, &material),
            Some(true)
        );
        assert_eq!(
            cleared.permits_link(&prefix, &along_wall, [p(5., 5.), p(35., 35.)], R, &material),
            Some(false)
        );
        assert_eq!(
            cleared.permits_link(
                &prefix,
                &[p(35., 8.)],
                [p(5., 5.), p(35., 5.)],
                R,
                &material
            ),
            Some(false)
        );
    }

    #[test]
    fn swept_footprint_cache_matches_full_strokes_as_the_prefix_grows() {
        let p = Point::new;
        let mut prefix = LinkedFrontier {
            moves: vec![
                (p(1., 1.), false),
                (p(8., 1.), true),
                (p(8., 4.), true),
                (p(20., 10.), true), // Travel across uncleared material.
                (p(24., 10.), true),
                (p(24., 14.), true),
                (p(30., 20.), false),
                (p(35., 20.), true),
            ],
            air: vec![false, false, false, true, false, false, false, false],
            feed_links: vec![false, false, false, false, true, false, false, false],
        };
        let mut cache = ClearedFootprint::default();
        let moves = std::mem::take(&mut prefix.moves);
        for (i, motion) in moves.into_iter().enumerate() {
            prefix.moves.push(motion);
            cache.update(&prefix, R).unwrap();
            let cuts: Vec<_> = prefix.moves[..=i]
                .iter()
                .enumerate()
                .map(|(i, &(point, cut))| (point, cut && !prefix.air[i] && !prefix.feed_links[i]))
                .collect();
            let expected: Vec<_> = crate::clearsim::cut_runs(&cuts)
                .iter()
                .flat_map(|run| stroke_run(run, R).unwrap())
                .collect();
            let actual: Vec<_> = cache
                .completed
                .iter()
                .map(|(_, polygon)| polygon)
                .chain(&cache.active_stroke)
                .collect();
            assert_eq!(actual, expected.iter().collect::<Vec<_>>());
        }
    }

    #[test]
    fn rectangular_pocket_does_not_repeat_cleared_frontiers() {
        // cnc_test.ocam: self-contained geometry, no CAD kernel or local files.
        let region = Polygon::new(Contour::new(vec![
            Point::new(10., -10.),
            Point::new(40., -10.),
            Point::new(40., 10.),
            Point::new(10., 10.),
        ]))
        .unwrap();
        let travel = |moves: &[(Point, bool)]| -> f64 {
            moves.windows(2).map(|p| p[0].0.distance(p[1].0)).sum()
        };
        for e in [1.5, 2.4] {
            let raw = front_advance_path(&region, R, 0., e, None).unwrap();
            let trimmed = trim_cleared_travel(&raw, R, &region)
                .expect("redundant wall and relief passes must be removed");
            eprintln!(
                "e={e}: raw {}, trimmed {}, lifts {}",
                travel(&raw),
                travel(&trimmed),
                rapids(&trimmed)
            );
            assert!(travel(&trimmed) < 0.90 * travel(&raw));
            assert_eq!(rapids(&trimmed), rapids(&raw));
            let certified = front_advance_certified(&region, R, 0., e, None, SIMPLIFY_EPS)
                .expect("optimized pocket must remain certified");
            assert!(travel(&certified) < 0.90 * travel(&raw));
            let linked = link_cleared_travel(
                &certified,
                &region,
                R,
                0.,
                e,
                &crate::CancelToken::new(),
                None,
            )
            .expect("long cleared wall runs must become travel links");
            let rapid_travel: f64 = linked
                .moves
                .windows(2)
                .enumerate()
                .filter(|(i, _)| linked.air[i + 1])
                .map(|(_, pair)| pair[0].0.distance(pair[1].0))
                .sum();
            eprintln!(
                "e={e}: rapid travel {rapid_travel}, links {}",
                linked.air.iter().filter(|&&b| b).count()
            );
            assert!(rapid_travel > 40.);
        }
    }

    /// A move path as bare points, for the measurement tests that instrument geometry.
    ///
    /// Safe for those tests **because they are all hole-free**, and a hole-free frontier
    /// emits no rapid but the initial positioning move — so the points are the cut path and
    /// every number they report is unchanged by the move-path refactor. Do not reach for
    /// this on a holed region: it would silently charge the rapids as cuts.
    fn pts(moves: &[(Point, bool)]) -> Vec<Point> {
        moves.iter().map(|&(p, _)| p).collect()
    }

    /// How many real rapids a move path contains (the initial positioning move aside).
    fn rapids(moves: &[(Point, bool)]) -> usize {
        moves.iter().skip(1).filter(|(_, cut)| !cut).count()
    }

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

    fn ring_at(cx: f64, cy: f64, rad: f64, n: usize) -> Contour {
        Contour::new(
            (0..n)
                .map(|i| {
                    let a = std::f64::consts::TAU * (i as f64) / (n as f64);
                    Point::new(cx + rad * a.cos(), cy + rad * a.sin())
                })
                .collect(),
        )
    }

    fn rect_at(x0: f64, y0: f64, x1: f64, y1: f64) -> Contour {
        Contour::new(vec![
            Point::new(x0, y0),
            Point::new(x1, y0),
            Point::new(x1, y1),
            Point::new(x0, y1),
        ])
    }

    fn island_cases() -> Vec<(&'static str, Polygon)> {
        vec![
            (
                "square 40, island 12 centred",
                Polygon::with_holes(
                    rect_at(0.0, 0.0, 40.0, 40.0),
                    vec![rect_at(14.0, 14.0, 26.0, 26.0)],
                )
                .unwrap(),
            ),
            (
                "square 40, island 12 offset",
                Polygon::with_holes(
                    rect_at(0.0, 0.0, 40.0, 40.0),
                    vec![rect_at(22.0, 14.0, 34.0, 26.0)],
                )
                .unwrap(),
            ),
            (
                "circle r20, island r6",
                Polygon::with_holes(
                    ring_at(0.0, 0.0, 20.0, 64),
                    vec![ring_at(0.0, 0.0, 6.0, 32)],
                )
                .unwrap(),
            ),
            (
                "square 60, island 20 centred",
                Polygon::with_holes(
                    rect_at(0.0, 0.0, 60.0, 60.0),
                    vec![rect_at(20.0, 20.0, 40.0, 40.0)],
                )
                .unwrap(),
            ),
            (
                "square 60, two islands",
                Polygon::with_holes(
                    rect_at(0.0, 0.0, 60.0, 60.0),
                    vec![
                        rect_at(12.0, 12.0, 24.0, 24.0),
                        rect_at(36.0, 36.0, 48.0, 48.0),
                    ],
                )
                .unwrap(),
            ),
        ]
    }

    /// **The Route B measurement.** Front-advance over holed regions, scored by the exact
    /// oracle — engagement at the cap, coverage of the reachable target, gouge.
    #[test]
    #[ignore = "measurement harness for ADAPTIVE_PLAN.md §8"]
    fn island_oracle_table() {
        println!("\n| case | passes | split | a_e | uncut | reach | gouge | verdict |");
        println!("|---|---|---|---|---|---|---|---|");
        for (name, region) in island_cases() {
            let Some(f) = frontier(&region, R, 0.0, E, None, STANDOFF_RADII * R, ENTRY_TARGET)
            else {
                println!("| {name} | — | — | — | — | — | — | **no frontier** |");
                continue;
            };
            let (np, sp) = (f.passes.len(), f.split);
            let Some(path) = front_advance_path(&region, R, 0.0, E, None) else {
                println!("| {name} | {np} | {sp} | — | — | — | — | **no path** |");
                continue;
            };
            let v = crate::clearsim::certify_moves(&path, R, &region);
            let reach: f64 = crate::clearsim::reachable(&region, R)
                .iter()
                .map(|p| p.area())
                .sum();
            let ok = v.max_engagement <= engagement_bound(E, R)
                && v.uncut_area <= 0.02 * reach + 1.0
                && v.gouge_area <= 0.02 * reach + 1.0;
            println!(
                "| {name} | {np} | {sp} | {:.2} | {:.1} | {:.0} | {:.1} | {} |",
                v.max_engagement,
                v.uncut_area,
                reach,
                v.gouge_area,
                if ok { "PASS" } else { "**FAIL**" }
            );
        }
        println!();
    }

    /// **A hole-free pocket is still one continuous cut.** The gouge backstop rewrites a
    /// cutting move that leaves the tool-centre region into a rapid, and the failure mode
    /// that guards against is not it missing a gouge — the certifier catches that — but it
    /// firing on *legitimate* moves. It is a containment test on a region whose own boundary
    /// the path is supposed to hug, so a boolean version of it condemns nearly everything:
    /// measured, `!tc.contains(..)` with no tolerance turned **1383 of 2852 moves** on a
    /// round pocket with an island into lift-and-replunge, against a true worst excursion of
    /// 0.019 mm. This pins the cheap end of that: a pocket with no island must come out with
    /// no lift in it at all.
    #[test]
    fn a_hole_free_pocket_is_still_one_continuous_cut() {
        for (name, region, ctr) in [
            ("square 40", square(40.0), [20.0, 20.0]),
            ("circle r30", circle(30.0, 96), [0.0, 0.0]),
        ] {
            let path = front_advance_path(&region, R, 0.0, E, Some(ctr))
                .unwrap_or_else(|| panic!("{name}: should produce a path"));
            assert_eq!(
                rapids(&path),
                0,
                "{name}: a hole-free pocket needs no lift, got {} of {} moves",
                rapids(&path),
                path.len()
            );
        }
    }

    /// **No cutting move leaves the tool-centre region.** The property the whole island
    /// increment rests on: a tool centre inside `tc` is a tool inside the part, so a cutting
    /// segment that leaves `tc` is a gouge by definition. Checked on *segments*, densely —
    /// every vertex of a front-advance path is legal by construction, and the defect this
    /// found was a single 42 mm chord between two legal vertices, straight through the
    /// centre of an island.
    #[test]
    fn no_cutting_move_leaves_the_tool_centre_region() {
        for (name, region) in island_cases() {
            let Some(f) = frontier(&region, R, 0.0, E, None, STANDOFF_RADII * R, ENTRY_TARGET)
            else {
                continue;
            };
            let Some((path, _)) = connect(&f, R, E, SEAM_ARC, RELIEF_REACH, ENTRY_TARGET) else {
                continue;
            };
            let mut worst = 0.0_f64;
            for i in 1..path.len() {
                if !path[i].1 {
                    continue;
                }
                let (a, b) = (path[i - 1].0, path[i].0);
                let n = ((a.distance(b) / SEGMENT_SAMPLE).ceil() as usize).max(1);
                for k in 0..=n {
                    let t = k as f64 / n as f64;
                    let p = Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
                    if !f.tc.contains(p) {
                        worst = worst.max(boundary_dist(&f.tc, p));
                    }
                }
            }
            assert!(
                worst <= GOUGE_TOL,
                "{name}: a cutting move left the tool-centre region by {worst:.3} mm"
            );
        }
    }

    /// Diagnostic: where does the time go on an island region — generation, connection, or
    /// certification? Relevant because `clearing::clear` no longer refuses holed regions, so
    /// **every** island pocket now pays this cost even when it ends up falling back to
    /// concentric. Run under `--release`; the debug figure is not the shipping one.
    #[test]
    #[ignore = "diagnostic"]
    fn island_timing() {
        let mut cases = vec![
            ("CONTROL square 40 (no island)", square(40.0)),
            ("CONTROL circle r30 (no island)", circle(30.0, 96)),
        ];
        cases.extend(island_cases());
        for (name, region) in cases {
            let t0 = std::time::Instant::now();
            let Some(f) = frontier(&region, R, 0.0, E, None, STANDOFF_RADII * R, ENTRY_TARGET)
            else {
                println!("{name}: no frontier");
                continue;
            };
            let gen = t0.elapsed().as_secs_f64();
            let t1 = std::time::Instant::now();
            let Some((path, _)) = connect(&f, R, E, SEAM_ARC, RELIEF_REACH, ENTRY_TARGET) else {
                continue;
            };
            let con = t1.elapsed().as_secs_f64();
            let t2 = std::time::Instant::now();
            let _ = crate::clearsim::certify_moves(&path, R, &region);
            let cert = t2.elapsed().as_secs_f64();
            println!(
                "{name}: generate {gen:.2}s  connect {con:.2}s  certify {cert:.2}s  \
                 total {:.2}s  ({} passes, {} moves)",
                gen + con + cert,
                f.passes.len(),
                path.len()
            );
        }
    }

    /// **The standoff sweep.** Hypothesis: the corner standoff is a *global* morphological
    /// opening by `rho`, so it erases any part of the frontier narrower than `2·rho` — and
    /// the annular band between an island and the wall is exactly that. Band widths (tool
    /// centres): square 40/island 12 → 8 mm; circle r20/r6 → 8 mm; square 60/two islands →
    /// 6 mm between them; **square 60/island 20 → 14 mm, the only one wider than 2·rho = 12
    /// mm, and the only one that certifies.** If that is the mechanism, dropping the
    /// standoff should move the peak on the narrow cases and not on the wide one.
    #[test]
    #[ignore = "diagnostic"]
    fn standoff_sweep_over_islands() {
        let fracs = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0];
        println!(
            "\n| case | narrowest | {} |",
            fracs.map(|f| format!("{f}·r")).join(" | ")
        );
        println!("|---|---|{}", "---|".repeat(fracs.len()));
        let mut cases = vec![("CONTROL square 40 (no island)", square(40.0))];
        cases.extend(island_cases());
        for (name, region) in cases {
            // The narrowest passage of the tool-centre region, by the largest erosion that
            // still leaves something: the quantity a global opening is in tension with.
            let tc = largest(offset(std::slice::from_ref(&region), -R, JoinStyle::Round).unwrap())
                .unwrap();
            let mut narrow = 0.0_f64;
            for k in 1..40 {
                let d = k as f64 * 0.5;
                match offset(std::slice::from_ref(&tc), -d, JoinStyle::Round) {
                    Ok(v) if !v.is_empty() && total_area(&v) > 1.0 => narrow = 2.0 * d,
                    _ => break,
                }
            }
            let mut cells = vec![format!("{narrow:.0} mm")];
            for f in fracs {
                let standoff = f * R;
                let t = Tuning {
                    standoff,
                    ..Tuning::new(R, E)
                };
                match front_advance_tuned(&region, R, 0.0, E, None, t) {
                    Some(path) => {
                        let v = crate::clearsim::certify_moves(&path, R, &region);
                        cells.push(format!(
                            "a_e {:.2} / uncut {:.0}",
                            v.max_engagement, v.uncut_area
                        ));
                    }
                    None => cells.push("no path".into()),
                }
            }
            println!("| {name} | {} |", cells.join(" | "));
        }
        println!();
    }

    /// Diagnostic: **where** is the island path's engagement peak? A number without a place
    /// cannot be acted on — the whole point of the densified readings on the hole-free
    /// cases. Reports the worst few cutting moves by `a_e`, with their location, step
    /// length, and how far along the path they sit.
    #[test]
    #[ignore = "diagnostic"]
    fn locate_the_island_slot() {
        for (name, region) in island_cases().into_iter().take(3) {
            let Some(path) = front_advance_path(&region, R, 0.0, E, None) else {
                continue;
            };
            let mut m = crate::clearsim::ClearedModel::bounded(R, region.clone());
            let mut worst: Vec<(f64, Point, f64, usize)> = Vec::new();
            let mut prev_cut = false;
            for i in 1..path.len() {
                let (a, b) = (path[i - 1].0, path[i].0);
                if !path[i].1 {
                    prev_cut = false;
                    continue;
                }
                if !prev_cut {
                    m.seed_disc(a);
                }
                prev_cut = true;
                // Densify so a reading localises to a place rather than to a whole run.
                let n = ((a.distance(b) / 0.5).ceil() as usize).max(1);
                for k in 0..n {
                    let (t0, t1) = (k as f64 / n as f64, (k + 1) as f64 / n as f64);
                    let p0 = Point::new(a.x + (b.x - a.x) * t0, a.y + (b.y - a.y) * t0);
                    let p1 = Point::new(a.x + (b.x - a.x) * t1, a.y + (b.y - a.y) * t1);
                    worst.push((m.engagement(p0, p1), p0, a.distance(b), i));
                }
                m.commit(a, b);
            }
            worst.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap_or(std::cmp::Ordering::Equal));
            println!("\n{name} ({} moves):", path.len());
            for (ae, p, step, i) in worst.iter().take(5) {
                println!(
                    "  a_e {ae:.2} at ({:.1},{:.1})  step {step:.2} mm  move {i}",
                    p.x, p.y
                );
            }
        }
        println!();
    }

    /// Diagnostic: **where** does the path leave the tool-centre region? Densifies every
    /// segment, so it catches a chord whose endpoints are both legal but whose middle is
    /// not — the failure mode that a vertex-only check reports as clean. Reports the worst
    /// excursion, where it is, and the step length of the segment responsible, because a
    /// long step is the signature of a hand-off between rings that are not adjacent.
    #[test]
    #[ignore = "diagnostic"]
    fn locate_the_island_gouge() {
        for (name, region) in island_cases().into_iter().take(3) {
            let Some(f) = frontier(&region, R, 0.0, E, None, STANDOFF_RADII * R, ENTRY_TARGET)
            else {
                continue;
            };
            let Some((path, _)) = connect(&f, R, E, SEAM_ARC, RELIEF_REACH, ENTRY_TARGET) else {
                continue;
            };
            let mut worst = (0.0_f64, Point::new(0.0, 0.0), 0.0_f64, 0usize);
            let mut longest_step = (0.0_f64, 0usize);
            for i in 1..path.len() {
                let (a, b) = (path[i - 1].0, path[i].0);
                let step = a.distance(b);
                if step > longest_step.0 {
                    longest_step = (step, i);
                }
                if !path[i].1 {
                    continue; // a rapid removes nothing
                }
                let n = ((step / 0.2).ceil() as usize).max(1);
                for k in 0..=n {
                    let t = k as f64 / n as f64;
                    let p = Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
                    if !f.tc.contains(p) {
                        let d = boundary_dist(&f.tc, p);
                        if d > worst.0 {
                            worst = (d, p, step, i);
                        }
                    }
                }
            }
            println!(
                "{name}: worst excursion {:.3} mm at ({:.1},{:.1}) on a {:.2} mm step (move {}) | \
                 longest step {:.2} mm at move {} of {} | rapids {}",
                worst.0,
                worst.1.x,
                worst.1.y,
                worst.2,
                worst.3,
                longest_step.0,
                longest_step.1,
                path.len(),
                rapids(&path),
            );
        }
    }

    /// Diagnostic: is the long step *inside a frontier ring* rather than between two of
    /// them? A ring is a closed contour of a real region, so consecutive vertices should be
    /// millimetres apart at most; a 40 mm jump inside one means the contour itself is not
    /// what it is assumed to be.
    #[test]
    #[ignore = "diagnostic"]
    fn ring_internal_steps() {
        for (name, region) in island_cases().into_iter().take(2) {
            let Some(f) = frontier(&region, R, 0.0, E, None, STANDOFF_RADII * R, ENTRY_TARGET)
            else {
                continue;
            };
            let mut worst = (0.0_f64, 0usize, 0usize, 0usize);
            for (k, pass) in f.passes.iter().enumerate() {
                for (c, ring) in pass.iter().enumerate() {
                    let n = ring.len();
                    for i in 0..n {
                        let d = ring[i].distance(ring[(i + 1) % n]);
                        if d > worst.0 {
                            worst = (d, k, c, n);
                        }
                    }
                }
            }
            println!(
                "{name}: worst step inside a ring {:.2} mm (pass {}, component {}, {} pts)",
                worst.0, worst.1, worst.2, worst.3
            );
            // And how many components each pass really has, around the topology change.
            let counts: Vec<usize> = f.passes.iter().map(|p| p.len()).collect();
            println!("  components per pass: {counts:?}");
        }
    }

    /// Diagnostic: do the two derived regions keep the island at all? `to_clear` clips the
    /// frontier's growth and `tc` bounds the tool centres; if either silently loses the
    /// hole, the frontier runs straight over the island and every downstream measurement is
    /// meaningless.
    #[test]
    #[ignore = "diagnostic"]
    fn derived_regions_keep_their_holes() {
        for (name, region) in island_cases() {
            for finish in [0.0, 0.5] {
                let tc_polys = offset(
                    std::slice::from_ref(&region),
                    -(R + finish),
                    JoinStyle::Round,
                )
                .unwrap();
                let clear_polys =
                    offset(std::slice::from_ref(&region), -finish, JoinStyle::Round).unwrap();
                let to_clear = largest(clear_polys.clone()).unwrap();
                let tc = largest(tc_polys.clone()).unwrap();
                println!(
                    "{name} finish {finish}: region holes {} | to_clear {} polys, holes {} | tc {} polys, holes {}",
                    region.holes().len(),
                    clear_polys.len(),
                    to_clear.holes().len(),
                    tc_polys.len(),
                    tc.holes().len(),
                );
            }
        }
    }

    /// Diagnostic: is the island slot the *frontier* failing, or the **connection**? Cheap
    /// — no oracle. Reports whether the seam spiral carried each case, and whether any ring
    /// of the frontier leaves the tool-centre region (which is where a gouge would come
    /// from, since frontier rings are gouge-free by construction and the links are not).
    #[test]
    #[ignore = "diagnostic"]
    fn island_connection_provenance() {
        for (name, region) in island_cases() {
            let Some(f) = frontier(&region, R, 0.0, E, None, STANDOFF_RADII * R, ENTRY_TARGET)
            else {
                println!("{name}: no frontier");
                continue;
            };
            let Some((path, seamed)) = connect(&f, R, E, SEAM_ARC, RELIEF_REACH, ENTRY_TARGET)
            else {
                println!("{name}: no connection");
                continue;
            };
            // **How far** outside the tool-centre region does the path stray — not how many
            // points do, which cannot tell tessellation noise from driving over an island.
            let depth = |p: &Point| {
                if f.tc.contains(*p) {
                    0.0
                } else {
                    boundary_dist(&f.tc, *p)
                }
            };
            let ring_worst = f
                .passes
                .iter()
                .flatten()
                .flatten()
                .map(depth)
                .fold(0.0_f64, f64::max);
            let path_worst = path.iter().map(|(p, _)| depth(p)).fold(0.0_f64, f64::max);
            // And how far into an *island* specifically (the gouge that matters).
            let isl_worst = region
                .holes()
                .iter()
                .map(|h| {
                    let hp = Polygon::new(h.clone()).unwrap();
                    path.iter()
                        .map(|&(p, _)| {
                            if hp.contains(p) {
                                boundary_dist(&hp, p)
                            } else {
                                0.0
                            }
                        })
                        .fold(0.0_f64, f64::max)
                })
                .fold(0.0_f64, f64::max);
            println!(
                "{name}: seam-connected {seamed}, passes {}, rings {}, path {} pts | \
                 worst outside tc: rings {ring_worst:.3} mm, path {path_worst:.3} mm | \
                 deepest into an island: {isl_worst:.3} mm",
                f.passes.len(),
                f.passes.iter().flatten().count(),
                path.len(),
            );
        }
    }

    /// Diagnostic: what a frontier over a **holed** region actually looks like, pass by
    /// pass — whether it flows around an island at all, and where it splits when it does.
    #[test]
    #[ignore = "diagnostic"]
    fn frontier_shape_over_islands() {
        let cases: Vec<(&str, Polygon)> = vec![
            ("square 40 (control, no island)", square(40.0)),
            (
                "square 40, island 12 centred",
                Polygon::with_holes(
                    rect_at(0.0, 0.0, 40.0, 40.0),
                    vec![rect_at(14.0, 14.0, 26.0, 26.0)],
                )
                .unwrap(),
            ),
            (
                "circle r20, island r6",
                Polygon::with_holes(
                    ring_at(0.0, 0.0, 20.0, 64),
                    vec![ring_at(0.0, 0.0, 6.0, 32)],
                )
                .unwrap(),
            ),
            (
                "square 60, island 20 centred",
                Polygon::with_holes(
                    rect_at(0.0, 0.0, 60.0, 60.0),
                    vec![rect_at(20.0, 20.0, 40.0, 40.0)],
                )
                .unwrap(),
            ),
        ];
        for (name, region) in cases {
            println!("\n=== {name} ===");
            let Some(f) = frontier(&region, R, 0.0, E, None, STANDOFF_RADII * R, ENTRY_TARGET)
            else {
                println!("  frontier: None");
                continue;
            };
            println!(
                "  entry {:?}  passes {}  split {}",
                (f.entry.x, f.entry.y),
                f.passes.len(),
                f.split
            );
            for (k, pass) in f.passes.iter().enumerate() {
                let comps: Vec<String> = pass
                    .iter()
                    .map(|ring| {
                        let per: f64 = (0..ring.len())
                            .map(|i| ring[i].distance(ring[(i + 1) % ring.len()]))
                            .sum();
                        format!("{}pts/{:.0}mm", ring.len(), per)
                    })
                    .collect();
                println!("  pass {k}: {} comp(s)  [{}]", pass.len(), comps.join(", "));
            }
        }
        println!();
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
        let raw = pts(&front_advance_path(region, r, 0.0, e, Some(ctr))
            .expect("front-advance produces a path"));
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
        rs.iter()
            .filter(|(_, p)| keep(*p))
            .fold(0.0f64, |m, (ae, _)| m.max(*ae))
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
            // The wall-clock ceiling is deliberately generous: it guards against the
            // *catastrophic* regression this milestone retired — the 120 s pre-decimation
            // grind and the 37-minute split-front `MAX_PASSES` loop — not against micro-
            // timing. A tight bound is meaningless here: this is a debug build, and the
            // suite now runs the exact oracle in several tests concurrently (front-advance
            // is wired into `clearing::clear`), so wall-clock under a saturated runner
            // reflects contention, not the generator. The **point count** below is the
            // contention-independent guard that the frontier decimation is working.
            // Raised 20 s → 60 s on 2026-07-30, and the reason is worth keeping: the suite
            // grew heavier (island clearing now dispatches the steered generator, and its
            // tests run the exact oracle), the debug run reached **664 s**, and this
            // assertion started failing on contention alone — 7.34 s for the same case run
            // by itself. A guard that fires on how busy the machine is guards nothing.
            //
            // 60 s still catches everything it was written for by an order of magnitude: the
            // 120 s pre-decimation grind and the 37-minute split-front `MAX_PASSES` loop.
            assert!(
                gen < 60.0,
                "{name}: generation regressed to a grind, took {gen:.2}s"
            );
            assert!(
                path.len() < 4000,
                "{name}: sane point count, got {}",
                path.len()
            );
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
            let body = peak(rs, |p| {
                p.distance(ctr) > 3.0 * E && d_corner(p, 40.0) > 8.0 * E
            });
            assert!(
                body <= 1.5 * E,
                "{name}: link slot should be gone, body a_e {body}"
            );
        }
    }

    /// The **body** of the path holds engagement near the cap **shape-independently**: a
    /// round pocket and a square pocket both read exactly 2.79 (1.4·e) away from the
    /// entry and the corners. That sameness is the evidence the frontier tracking itself
    /// is sound — what is left over is localised to the entry and the corners rather
    /// than smeared through the path.
    #[test]
    fn the_path_body_holds_engagement_near_the_cap_on_every_shape() {
        let body = peak(circle_readings(), |p| {
            p.distance(Point::new(0.0, 0.0)) > 3.0 * E
        });
        assert!(
            body <= 1.5 * E,
            "round pocket body should sit near the cap, got {body}"
        );

        // A corner's influence reaches ~8 stepovers along the walls into it (measured);
        // beyond that the square matches the circle exactly.
        let body = peak(square_readings(), |p| {
            p.distance(Point::new(20.0, 20.0)) > 3.0 * E && d_corner(p, 40.0) > 8.0 * E
        });
        assert!(
            body <= 1.5 * E,
            "square pocket body should sit near the cap, got {body}"
        );
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
        assert!(
            near <= 1.5 * E,
            "corner should sit at the body's floor, got {near}"
        );
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
        let bare = front_advance_tuned(
            &region,
            R,
            0.0,
            E,
            Some([20.0, 20.0]),
            Tuning {
                standoff: 0.0,
                ..Tuning::new(R, E)
            },
        )
        .map(|m| pts(&m))
        .unwrap();
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

    /// The **entry no longer over-engages**: 3.52 → 2.69, which is the geometric floor and
    /// the same value the rest of the path holds. It is not the peak of anything any more.
    ///
    /// **What the entry residual actually was, after two wrong answers.** It was *not* the
    /// core spiral. On a circle r30 the first frontier loop sits at ρ=2, so the spiral spans
    /// only ρ ∈ [0,2] — while the 3.42 peak sat at ρ≈3.94, on the frontier **loop** at ρ=4.
    /// Retuning the spiral's pitch moved it not at all, across targets from 1.4·e to 0.5·e
    /// (measured: 3.42, every time). Nor was it an artifact of the arc formula, as the
    /// wrapped-groove story would have it: the independent area oracle agrees at the same
    /// move — **3.42 angle vs 3.53 area**.
    ///
    /// It was the **geometric floor on the innermost loops**: at tool-centre radius ρ the
    /// tool's outer edge sweeps `(ρ+r)/ρ` times as far as its centre travels, so a full
    /// stepover bites harder the tighter the loop — `floor(ρ=4)` = 3.0, `floor(ρ=2)` = 4.0.
    /// The frontier advanced a flat `e` regardless. The fix is the **radius-aware advance**
    /// in [`front_advance_tuned`]; see [`pitch_for_cap`].
    #[test]
    fn the_entry_holds_engagement_like_the_rest_of_the_path() {
        for (name, rs, c) in [
            ("circle r30", circle_readings(), Point::new(0.0, 0.0)),
            ("square 40", square_readings(), Point::new(20.0, 20.0)),
        ] {
            let entry = peak(rs, |p| p.distance(c) <= 3.0 * E);
            assert!(
                entry <= 1.5 * E,
                "{name}: entry should sit at the floor, got {entry} (was 3.52)"
            );
        }
    }

    /// The radius-aware advance is what fixed the entry, and it is **not** a free win — it
    /// is bought with passes. Pinned as a pair: forcing the advance to a flat stepover (by
    /// asking for a target the floor can always meet) puts the entry back over 3.4.
    ///
    /// Guards against "just lower the target": measured on a circle r30, target 1.4·e →
    /// entry 2.69 at +2.5% travel, because `pitch_for_cap` only bites below ρ≈5 and clamps
    /// to the full stepover beyond. 0.7·e → entry 2.07 but **+64% travel**, and 0.5·e →
    /// +119% — that is not a fix, it is cutting the whole pocket finer.
    #[test]
    fn the_radius_aware_advance_is_what_holds_the_entry() {
        let region = circle(30.0, 96);
        // A target of 4·e is above the floor at every radius here, so `pitch_for_cap`
        // never binds and the advance is a flat stepover — the old behaviour.
        let flat = front_advance_tuned(
            &region,
            R,
            0.0,
            E,
            Some([0.0, 0.0]),
            Tuning {
                entry_target: 4.0 * E,
                ..Tuning::new(R, E)
            },
        )
        .map(|m| pts(&m))
        .unwrap();
        let mut m = crate::clearsim::ClearedModel::bounded(R, region.clone());
        m.seed_disc(flat[0]);
        let mut worst = 0.0f64;
        for w in flat.windows(2) {
            for pc in densify(w, 1.0).windows(2) {
                if pc[0].distance(Point::new(0.0, 0.0)) <= 3.0 * E {
                    worst = worst.max(m.engagement(pc[0], pc[1]));
                }
            }
            m.commit(w[0], w[1]);
        }
        let tuned = peak(circle_readings(), |p| {
            p.distance(Point::new(0.0, 0.0)) <= 3.0 * E
        });
        assert!(
            worst > tuned + 0.3 * E,
            "a flat advance should over-engage the entry: {worst} vs {tuned}"
        );
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
                let raw = front_advance_tuned(
                    &region,
                    r,
                    0.0,
                    e,
                    Some([20.0, 20.0]),
                    Tuning {
                        seam_arc: sa,
                        ..Tuning::new(r, e)
                    },
                )
                .map(|mv| pts(&mv))
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
        for (name, region) in [("square 40", square(40.0)), ("square 24", square(24.0))] {
            let tc = largest(offset(std::slice::from_ref(&region), -r, JoinStyle::Round).unwrap())
                .unwrap();
            let corners = sharp_corners(&tc, SIMPLIFY_EPS);
            assert_eq!(
                corners.len(),
                4,
                "{name}: a square pocket has four sharp corners"
            );
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
            offset(
                std::slice::from_ref(&circle(30.0, 96)),
                -R,
                JoinStyle::Round,
            )
            .unwrap(),
        )
        .unwrap();
        assert!(
            sharp_corners(&tc, SIMPLIFY_EPS).is_empty(),
            "a circle has no sharp corner"
        );
    }

    #[test]
    fn cad_projection_zigzags_do_not_create_corners_on_a_round_wall() {
        let mut points = Vec::new();
        for i in 0..720 {
            let angle = std::f64::consts::TAU * i as f64 / 720.;
            for (a, radius) in [(angle, 12.), (angle + 0.0001, 12.0025)] {
                points.push(Point::new(-30. + radius * a.cos(), 10. + radius * a.sin()));
            }
        }
        for shift in [0, 1, 57] {
            let mut rotated = points.clone();
            rotated.rotate_left(shift);
            for reverse in [false, true] {
                let mut outline = rotated.clone();
                if reverse {
                    outline.reverse();
                }
                let tc = Polygon::new(Contour::new(outline)).unwrap();
                assert!(
                    !sharp_corners(&tc, 0.0).is_empty(),
                    "zero must disable noise simplification"
                );
                assert!(
                    sharp_corners(&tc, SIMPLIFY_EPS).is_empty(),
                    "projection noise must not request corner relief (shift {shift}, reverse {reverse})"
                );
            }
        }
    }

    #[test]
    fn simplification_tolerance_changes_the_certified_frontier() {
        let region = circle(12., 96);
        let fine = front_advance_certified(&region, R, 0., E, None, 0.005)
            .expect("fine approximation must certify");
        let coarse = front_advance_certified(&region, R, 0., E, None, 0.02)
            .expect("default approximation must certify");
        assert_ne!(fine, coarse, "the operation setting must affect the path");
        let ring = region.outer().points();
        assert_eq!(
            simplify_ring(ring, 0.),
            ring,
            "zero retains the original outline"
        );
    }

    #[test]
    fn simplifying_corner_detection_preserves_real_corners_on_a_dense_square() {
        let vertices = square(40.).outer().points().to_vec();
        let mut points = Vec::new();
        for i in 0..4 {
            let (a, b) = (vertices[i], vertices[(i + 1) % 4]);
            for j in 0..100 {
                let t = j as f64 / 100.;
                points.push(Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
            }
        }
        for reverse in [false, true] {
            let mut outline = points.clone();
            if reverse {
                outline.reverse();
            }
            let tc = Polygon::new(Contour::new(outline)).unwrap();
            let corners = sharp_corners(&tc, SIMPLIFY_EPS);
            assert_eq!(corners.len(), 4, "real corners still need relief");
            assert!(corners.iter().all(|corner| vertices.contains(&corner.v)));
        }
    }

    /// The frontier converges onto the tool-centre boundary a hair before the coverage
    /// tolerance trips, which made it emit the closing lap **twice** — the same wall cut
    /// over again for nothing. The duplicate is dropped, so no two consecutive loops
    /// enclose the same area.
    #[test]
    fn the_frontier_does_not_emit_the_same_closing_lap_twice() {
        let path = pts(&front_advance_path(&square(40.0), R, 0.0, E, Some([20.0, 20.0])).unwrap());
        // The closing lap is the exact tool-centre square; cutting it twice would put two
        // separate visits to the same corner in the path.
        let visits = path
            .iter()
            .filter(|p| p.distance(Point::new(37.0, 37.0)) < 1e-6)
            .count();
        assert!(
            visits <= 1,
            "the closing lap's corner should be visited once, got {visits}"
        );
    }

    /// **The raster gate never under-reads the exact oracle.** This is the safety property
    /// the whole certified-or-fallback contract rests on: the raster is what a gate can
    /// afford to run (~220× faster than the polygon oracle), so it must err *high*.
    ///
    /// It is pinned here, across a battery of real generated paths plus a deliberate slot,
    /// because the alternative is not hypothetical — the raster shipped **full-diameter
    /// slots** while reading 0.80, and the comment on it claimed it was "biased high, the
    /// safe direction". Nobody checked. This test is that check.
    ///
    /// The bound comes from [`CELL_MAX`]: the probe sits at `r − px`, so cells coarser than
    /// ~0.10 push it inside the tool and it starts missing the thin band at the perimeter.
    #[test]
    fn the_raster_never_under_reads_the_exact_oracle() {
        let (r, e) = (R, E);
        let mut cases: Vec<(String, Polygon, Vec<Point>)> = Vec::new();
        for (name, region, ctr) in [
            ("circle r30", circle(30.0, 96), [0.0, 0.0]),
            ("circle r12", circle(12.0, 64), [0.0, 0.0]),
            ("square 40", square(40.0), [20.0, 20.0]),
            ("square 24", square(24.0), [12.0, 12.0]),
        ] {
            let path = pts(&front_advance_path(&region, r, 0.0, e, Some(ctr))
                .unwrap_or_else(|| panic!("{name}: front-advance produces a path")));
            cases.push((name.to_string(), region, path));
        }
        // The case the old formula was structurally blind to: driving into virgin stock.
        cases.push((
            "deliberate slot".to_string(),
            square(40.0),
            (0..=30).map(|i| Point::new(5.0 + i as f64, 20.0)).collect(),
        ));

        for (name, region, path) in &cases {
            let exact = crate::clearsim::certify(path, r, region).max_engagement;
            let ras = crate::raster::certify(path, r, region, e)
                .unwrap_or_else(|| panic!("{name}: raster builds"))
                .max_engagement;
            assert!(
                ras >= exact - 1e-6,
                "{name}: the raster UNDER-read the truth — a gate that does this ships \
                 slots. raster {ras:.2} vs exact {exact:.2}"
            );
            // And it must stay tight enough to be useful, or it rejects everything.
            assert!(
                ras <= exact + 0.5,
                "{name}: the raster over-reads too far to be a usable gate, \
                 raster {ras:.2} vs exact {exact:.2}"
            );
        }
    }
}
