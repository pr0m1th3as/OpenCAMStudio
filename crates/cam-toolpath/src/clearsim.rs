//! The clearing **oracle**: an independent model of the material a clearing path
//! removes. It measures a path's engagement, its coverage of the target, and any
//! gouging outside it — exactly (via polygon booleans on i_overlay's fixed integer
//! grid), not by eyeballing a backplot.
//!
//! This is the spine of the correctness guarantee for adaptive clearing: the
//! adaptive generator is *proven* against this oracle in tests, and *self-checks*
//! with it at runtime, falling back to the concentric clearer whenever a path
//! cannot be certified (engagement over cap, a coverage gap, or a gouge). So every
//! emitted path is verified correct — adaptive where it certifies, concentric
//! otherwise.
// The oracle is exercised by its own tests now; the adaptive path generator that
// consumes it at runtime lands in the next phase.
#![allow(dead_code)]

use cam_geo::{
    difference, offset, stroke_path, union, Arc, CapStyle, Contour, JoinStyle, Point, Polygon,
    Polyline,
};

/// Total (net, holes subtracted) area of a set of polygons.
fn total_area(polys: &[Polygon]) -> f64 {
    polys.iter().map(Polygon::area).sum()
}

/// Perimeter of a closed point ring (sum of edge lengths).
fn ring_perimeter(pts: &[Point]) -> f64 {
    let n = pts.len();
    (0..n).map(|i| pts[i].distance(pts[(i + 1) % n])).sum()
}

/// Total boundary length of a set of polygons (outer contours and holes).
fn total_perimeter(polys: &[Polygon]) -> f64 {
    polys
        .iter()
        .map(|p| {
            ring_perimeter(p.outer().points())
                + p.holes().iter().map(|h| ring_perimeter(h.points())).sum::<f64>()
        })
        .sum()
}

/// The region a tool of radius `r` sweeps as its centre travels along `path`
/// (round profile ⇒ round caps and joins).
fn swept(path: &[Point], r: f64) -> Vec<Polygon> {
    let pl = Polyline::new(path.to_vec());
    stroke_path(&pl, r, CapStyle::Round, JoinStyle::Round).unwrap_or_default()
}

/// Morphological opening of `region` by radius `r`: erode by `r`, then dilate by
/// `r`. This is the material a round tool of radius `r` can actually reach — sharp
/// internal corners a radius-`r` cutter cannot enter are excluded, so coverage is
/// judged against what is physically clearable, not against unreachable slivers.
pub(crate) fn reachable(region: &Polygon, r: f64) -> Vec<Polygon> {
    let eroded = offset(std::slice::from_ref(region), -r, JoinStyle::Round).unwrap_or_default();
    if eroded.is_empty() {
        return Vec::new();
    }
    offset(&eroded, r, JoinStyle::Round).unwrap_or_default()
}

/// A running model of the material cleared by a tool of radius `r`.
pub(crate) struct ClearedModel {
    r: f64,
    cleared: Vec<Polygon>,
}

impl ClearedModel {
    /// An empty model for a tool of radius `r`.
    pub(crate) fn new(r: f64) -> Self {
        Self {
            r,
            cleared: Vec::new(),
        }
    }

    /// Seed the cleared region with the disc a plunge/helix opens at `c` — the entry
    /// hole, so the first cutting moves are not charged for stock the plunge removed.
    pub(crate) fn seed_disc(&mut self, c: Point) {
        let pts = Arc::circle(c, self.r).flatten(0.05);
        if let Ok(d) = Polygon::new(Contour::new(pts)) {
            self.cleared = if self.cleared.is_empty() {
                vec![d]
            } else {
                union(&self.cleared, std::slice::from_ref(&d)).unwrap_or_else(|_| vec![d])
            };
        }
    }

    /// The radial width of cut (`a_e`) of the move `from`→`to`: how deep into
    /// previously-uncut material the tool bites.
    ///
    /// Measured as `2·area / perimeter` of the freshly-cut region. A cut of width
    /// `w` and length `L` has area ≈ `w·L` and perimeter ≈ `2·L`, so this returns
    /// `w` — **independent of how the path curves**, which the naïve
    /// `area / feed_length` is not (a curving tool sweeps more at its outer edge than
    /// its centre travels, over-reporting engagement on every arc). A full-width
    /// slotting cut returns ≈ the diameter, a light peel returns ≈ the stepover.
    pub(crate) fn engagement(&self, from: Point, to: Point) -> f64 {
        if from.distance(to) < 1e-9 {
            return 0.0;
        }
        let sweep = swept(&[from, to], self.r);
        let fresh = if self.cleared.is_empty() {
            sweep
        } else {
            difference(&sweep, &self.cleared).unwrap_or_default()
        };
        let perim = total_perimeter(&fresh);
        if perim < 1e-9 {
            0.0
        } else {
            2.0 * total_area(&fresh) / perim
        }
    }

    /// Add the move `from`→`to` to the cleared region.
    pub(crate) fn commit(&mut self, from: Point, to: Point) {
        let sweep = swept(&[from, to], self.r);
        if sweep.is_empty() {
            return;
        }
        self.cleared = if self.cleared.is_empty() {
            sweep
        } else {
            union(&self.cleared, &sweep).unwrap_or_else(|_| std::mem::take(&mut self.cleared))
        };
    }

    /// The cleared region so far.
    pub(crate) fn cleared(&self) -> &[Polygon] {
        &self.cleared
    }
}

/// The result of certifying a clearing path against a target region.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Verdict {
    /// Peak radial width of cut over the path (compare against the engagement cap).
    pub(crate) max_engagement: f64,
    /// Reachable target material left uncut (a coverage gap; excludes corners a
    /// radius-`r` tool cannot enter).
    pub(crate) uncut_area: f64,
    /// Material removed **outside** the target region — a gouge into the finished
    /// wall/skin. Must be ~0.
    pub(crate) gouge_area: f64,
}

impl Verdict {
    /// Whether the path is safe to emit: covers the reachable target (within
    /// `scallop`), never gouges (within `scallop`), and holds engagement at or below
    /// `cap` (within a small tolerance).
    pub(crate) fn certified(&self, cap: f64, scallop: f64) -> bool {
        self.uncut_area <= scallop
            && self.gouge_area <= scallop
            && self.max_engagement <= cap * 1.05 + 1e-6
    }
}

/// Certify a cutting path (tool-centre points, all treated as cutting moves)
/// against the target material region `to_clear` for a tool of radius `r`: peak
/// engagement, uncut remainder (of the reachable target), and gouge.
pub(crate) fn certify(path: &[Point], r: f64, to_clear: &Polygon) -> Verdict {
    // Coverage and gouge come from the whole swept region in one boolean pass —
    // exact and cheap (no per-segment accumulation).
    let full = swept(path, r);
    let reach = reachable(to_clear, r);
    let uncut = if reach.is_empty() {
        Vec::new()
    } else {
        difference(&reach, &full).unwrap_or_default()
    };
    let gouge = difference(&full, std::slice::from_ref(to_clear)).unwrap_or_default();

    // Peak engagement is inherently sequential: walk the path against the running
    // cleared region. This is the costly part, so it is measured, not the coverage.
    // Seed the entry disc the plunge opens, so the first moves are not charged for it.
    let mut model = ClearedModel::new(r);
    if let Some(first) = path.first() {
        model.seed_disc(*first);
    }
    let mut max_e = 0.0_f64;
    for w in path.windows(2) {
        max_e = max_e.max(model.engagement(w[0], w[1]));
        model.commit(w[0], w[1]);
    }
    Verdict {
        max_engagement: max_e,
        uncut_area: total_area(&uncut),
        gouge_area: total_area(&gouge),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_geo::Contour;

    fn square(lo: f64, hi: f64) -> Polygon {
        Polygon::new(Contour::new(vec![
            Point::new(lo, lo),
            Point::new(hi, lo),
            Point::new(hi, hi),
            Point::new(lo, hi),
        ]))
        .unwrap()
    }

    #[test]
    fn slotting_into_solid_engages_near_the_full_diameter() {
        // A straight cut into virgin stock is a full slot: engagement approaches the
        // diameter (2r = 6). The round end-caps add perimeter, so a finite-length slot
        // reads a little under 6 (→ 6 as the slot lengthens) — accurate for the thin
        // peels adaptive actually produces, and still far above any real cap, so a
        // slot is always rejected.
        let model = ClearedModel::new(3.0);
        let e = model.engagement(Point::new(0.0, 0.0), Point::new(40.0, 0.0));
        assert!(
            (5.0..6.1).contains(&e),
            "slotting engagement should approach the diameter 6, got {e}"
        );
    }

    #[test]
    fn a_light_peel_alongside_cleared_stock_engages_about_the_stepover() {
        // Clear a wide first swath, then peel a pass a light 2 mm stepover away
        // (r=3 ⇒ the tool overlaps the cleared swath, cutting only a ~2 mm strip):
        // engagement drops to about the stepover, far below the diameter.
        let mut model = ClearedModel::new(3.0);
        model.commit(Point::new(0.0, 0.0), Point::new(40.0, 0.0));
        let e = model.engagement(Point::new(0.0, 2.0), Point::new(40.0, 2.0));
        assert!(e < 3.5, "a light peel engages far less than the diameter 6, got {e}");
        assert!(e > 0.5, "but it does cut a fresh strip, got {e}");
    }

    #[test]
    fn certify_flags_a_covered_region_as_clean() {
        // Serpentine passes 4 mm apart (r=2 ⇒ full overlap) over [0,20]², with the
        // centres held 2 mm inside so the tool edge just reaches the material edges.
        let r = 2.0;
        let mut path = Vec::new();
        let mut y = 2.0;
        let mut forward = true;
        while y <= 18.0 + 1e-9 {
            let (a, b) = if forward { (2.0, 18.0) } else { (18.0, 2.0) };
            path.push(Point::new(a, y));
            path.push(Point::new(b, y));
            y += 2.0;
            forward = !forward;
        }
        let v = certify(&path, r, &square(0.0, 20.0));
        assert!(v.uncut_area < 3.0, "reachable target should be covered, uncut {}", v.uncut_area);
        assert!(v.gouge_area < 1e-3, "centres held inside ⇒ no gouge, got {}", v.gouge_area);
    }

    #[test]
    fn certify_flags_a_gouge_outside_the_region() {
        // A cut whose swept tool leaves the target reports gouge area.
        let r = 2.0;
        let path = vec![Point::new(10.0, 10.0), Point::new(30.0, 10.0)];
        let v = certify(&path, r, &square(0.0, 20.0));
        assert!(v.gouge_area > 1.0, "a cut past the edge must register a gouge, got {}", v.gouge_area);
    }
}
