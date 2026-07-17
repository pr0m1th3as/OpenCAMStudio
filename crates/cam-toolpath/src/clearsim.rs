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

use cam_geo::{difference, offset, stroke_path, union, CapStyle, JoinStyle, Point, Polygon, Polyline};

/// Total (net, holes subtracted) area of a set of polygons.
fn total_area(polys: &[Polygon]) -> f64 {
    polys.iter().map(Polygon::area).sum()
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

    /// The radial width of **new** (previously-uncut) material the move `from`→`to`
    /// cuts: the freshly-swept area divided by the move length. On a short segment
    /// this approximates the instantaneous engagement — a full-width slotting cut
    /// approaches the tool diameter, a light peel alongside cleared stock approaches
    /// the stepover. (The leading round cap slightly inflates the figure on very
    /// short isolated moves; it cancels out along a continuous path, where each
    /// move's trailing cap sits in already-cleared stock.)
    pub(crate) fn engagement(&self, from: Point, to: Point) -> f64 {
        let len = from.distance(to);
        if len < 1e-9 {
            return 0.0;
        }
        let sweep = swept(&[from, to], self.r);
        let fresh = if self.cleared.is_empty() {
            sweep
        } else {
            difference(&sweep, &self.cleared).unwrap_or_default()
        };
        total_area(&fresh) / len
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
    let mut model = ClearedModel::new(r);
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
    fn slotting_into_solid_engages_the_full_diameter() {
        // A straight cut into virgin stock is a full slot: engagement ≈ the diameter
        // (2r = 6), plus a little for the round end-caps over the finite length.
        let model = ClearedModel::new(3.0);
        let e = model.engagement(Point::new(0.0, 0.0), Point::new(40.0, 0.0));
        assert!(
            (6.0..7.5).contains(&e),
            "slotting engagement should be ~diameter 6 (+caps), got {e}"
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
