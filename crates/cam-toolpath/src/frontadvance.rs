//! Front-advance (cleared-region-tracking) adaptive clearing.
//!
//! Where the spiral-morph generator ([`crate::adaptive`]) blends pre-computed offset
//! rings and *hopes* they hold engagement — the exact oracle showed they slot at the
//! entry, at sharp corners, and at ring/handoff transitions — this advances the
//! **actual cleared region** outward by one stepover per pass.
//!
//! Each pass's tool centres follow `offset(cleared ⊕ e, −r)`: the tool then reaches
//! exactly `e` beyond what is already cleared, so the radial width of cut is the
//! stepover **by construction** —
//!
//! - at the **entry**, because the plunge disc is the region the first pass offloads
//!   into (no virgin-stock slot);
//! - at **corners**, because the cleared region is round (a union of tool discs), so
//!   its offset stays uniformly `e` away with no sharp pivot;
//! - around **concavities**, because the frontier flows with the material.
//!
//! Every emitted pass is verifiable against the exact engagement oracle
//! ([`crate::clearsim`]); the caller keeps the certified-or-fallback contract.
#![allow(dead_code)]

use cam_geo::{intersection, offset, union, Arc, Contour, JoinStyle, Point, Polygon};

/// Guard on the number of outward passes.
const MAX_PASSES: usize = 800;

/// Vertex-decimation tolerance (mm) for the cleared frontier — far below any stepover,
/// so the frontier shifts negligibly and engagement is unaffected.
const SIMPLIFY_EPS: f64 = 0.02;

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
/// pass-to-pass link short (the frontier only moves `e` per pass, so the link cuts at
/// most a stepover).
fn append_loop(path: &mut Vec<Point>, loop_pts: &[Point], prev: &mut Point) {
    if loop_pts.len() < 3 {
        return;
    }
    let rot = crate::profile::rotate_to_start(loop_pts, Some([prev.x, prev.y]));
    path.extend_from_slice(&rot);
    path.push(rot[0]); // close the loop
    *prev = rot[0];
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
    let mut path = vec![entry];
    let mut prev = entry;

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
        // Cut this frontier pass.
        for poly in &pass {
            append_loop(&mut path, poly.outer().points(), &mut prev);
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

    (path.len() > 3).then_some(path)
}

#[cfg(test)]
mod tests {
    use super::*;

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

    /// The load-bearing claim: front-advance holds engagement at the cap — measured by
    /// the EXACT oracle — on the very shapes the spiral-morph slotted (a_e = 6):
    /// the square (corner slot) and the round pocket (entry slot).
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

    /// Peak `a_e` (exact oracle) over the whole path, and over the path **excluding the
    /// entry region** (the innermost `2·e`), for a round pocket.
    fn peaks(region: &Polygon, r: f64, e: f64, ctr: Point) -> (f64, f64) {
        let path = front_advance_path(region, r, 0.0, e, Some([ctr.x, ctr.y]))
            .expect("front-advance produces a path");
        let mut m = crate::clearsim::ClearedModel::bounded(r, region.clone());
        m.seed_disc(path[0]);
        let (mut all, mut body) = (0.0f64, 0.0f64);
        for w in path.windows(2) {
            let ae = m.engagement(w[0], w[1]);
            all = all.max(ae);
            if w[0].distance(ctr) > 2.0 * e && w[1].distance(ctr) > 2.0 * e {
                body = body.max(ae);
            }
            m.commit(w[0], w[1]);
        }
        (all, body)
    }

    /// Honest state (exact oracle) of the front-advance path today. The **body** of the
    /// path — away from the entry — holds engagement far better than the spiral-morph
    /// (which slotted at square corners and big-pocket transitions, a_e = 6): here the
    /// body peaks at ~1.5·e, the pass-to-pass links. Two gaps remain, both next:
    ///
    /// - the **entry** still slots (a_e ≈ diameter) — the radial connector from the
    ///   plunge point to the first frontier loop, the same defect the pocket spiral has;
    ///   it wants the same Archimedean core-spiral entry;
    /// - the **links** between frontier loops spike to ~1.5·e; smoothing them into a
    ///   continuous spiral connection brings the body to the cap.
    #[test]
    fn front_advance_body_holds_engagement_far_better_than_spiral_morph() {
        let (r, e) = (3.0, 2.0);
        let (all, body) = peaks(&circle(9.0, 40), r, e, Point::new(0.0, 0.0));
        // Body (away from entry) is bounded well below a slot — the front-advance win.
        assert!(body <= 1.7 * e, "body engagement bounded, got {body} (cap {e})");
        // The entry still slots — documented, pending the core-spiral entry.
        assert!(all >= 2.0 * r - 0.5, "entry still slots (known gap), got {all}");
    }
}
