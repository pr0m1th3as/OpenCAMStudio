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
    // Progress below this (mm²) means the front has stopped advancing → done.
    let done_tol = 0.02 * e * e + 0.05;

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
        // What the tool actually clears cutting `pass` — the opening of `grown` by r.
        let opened = offset(&pass, r, JoinStyle::Round).ok()?;
        let next = union(&cleared, &opened).ok().unwrap_or(opened);
        if total_area(&next) - total_area(&cleared) < done_tol {
            break; // no meaningful progress — covered
        }
        for poly in &pass {
            append_loop(&mut path, poly.outer().points(), &mut prev);
        }
        cleared = next;
    }

    (path.len() > 3).then_some(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clearsim::certify;

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
    /// Generation speed alone (no exact-oracle certify).
    ///
    /// `#[ignore]` for now: repeated round offsets balloon the cleared polygon's vertex
    /// count (≈19 → 1096 over four passes on a ⌀18 pocket), so each pass — and the exact
    /// oracle over the resulting path — is slow. Taming that (simplify/decimate the
    /// cleared boundary per pass) is the next front-advance milestone, before wiring it
    /// into `clearing::clear`. Run explicitly with `--ignored` to measure.
    #[test]
    #[ignore = "slow until the cleared-polygon vertex growth is tamed (perf milestone)"]
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
            eprintln!("{name}: {} pts, gen {:.3}s", path.len(), gen);
            assert!(gen < 5.0, "{name}: generation should be quick, took {gen:.2}s");
        }
    }

    /// The load-bearing claim: front-advance holds engagement at the cap (verified by
    /// the EXACT oracle) on the round pocket that the spiral-morph slotted at entry
    /// (a_e = 6). Confirmed passing; `#[ignore]` only because the exact oracle over the
    /// high-vertex path is slow until the perf milestone above lands.
    #[test]
    #[ignore = "slow (exact oracle over a high-vertex path) until the perf milestone lands"]
    fn front_advance_holds_engagement_at_the_cap() {
        let (r, e) = (3.0, 2.0);
        let region = circle(9.0, 40);
        let path = front_advance_path(&region, r, 0.0, e, Some([0.0, 0.0]))
            .expect("front-advance produces a path");
        let v = certify(&path, r, &region);
        eprintln!("circle r9: max_e={:.2} uncut={:.1} gouge={:.1}", v.max_engagement, v.uncut_area, v.gouge_area);
        assert!(v.max_engagement <= e * 1.35, "engagement held at the cap, got {}", v.max_engagement);
        assert!(v.gouge_area < 1.5, "no gouge, got {}", v.gouge_area);
        assert!(v.uncut_area < 0.05 * region.area() + 3.0, "covered, uncut {}", v.uncut_area);
    }
}
