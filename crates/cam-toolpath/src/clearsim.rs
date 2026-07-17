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
    /// The stock region (target material). `None` ⇒ unbounded virgin stock (used by
    /// the primitive slot/peel unit tests); `Some` bounds where material actually is,
    /// so engagement is not charged for cutting air outside the part.
    material: Option<Polygon>,
}

impl ClearedModel {
    /// An empty model for a tool of radius `r` over **unbounded** stock.
    pub(crate) fn new(r: f64) -> Self {
        Self {
            r,
            cleared: Vec::new(),
            material: None,
        }
    }

    /// An empty model bounded to the stock region `material` — outside it is air, not
    /// uncut stock, so the tool is not charged engagement for a cut that leaves the part.
    pub(crate) fn bounded(r: f64, material: Polygon) -> Self {
        Self {
            r,
            cleared: Vec::new(),
            material: Some(material),
        }
    }

    /// Whether `q` is uncut stock: inside the material region (if bounded) and not yet
    /// cleared.
    fn is_uncut(&self, q: Point) -> bool {
        if let Some(m) = &self.material {
            if !m.contains(q) {
                return false;
            }
        }
        !self.cleared.iter().any(|p| p.contains(q))
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

    /// The **exact** radial width of cut (`a_e`) of the move `from`→`to`, via the
    /// tool-engagement angle.
    ///
    /// At tool centres sampled along the move, `Φ` is the angular span of the tool's
    /// **leading** perimeter (the half facing the feed) that lies in uncut stock. The
    /// radial depth then follows exactly from the circle geometry:
    ///
    /// ```text
    ///   a_e = r · (1 − cos Φ)
    /// ```
    ///
    /// A full slot (the whole leading half in uncut stock, `Φ = π`) reads the diameter
    /// `2r`; a peel of stepover `s` reads `s`; a light skim reads near zero. Only the
    /// *leading* arc is counted, so the material this very move is cutting behind the
    /// tool is never charged, and (unlike `2·area/perimeter`) a momentary slot is not
    /// averaged away. The peak over the sampled centres is returned.
    pub(crate) fn engagement(&self, from: Point, to: Point) -> f64 {
        let (dx, dy) = (to.x - from.x, to.y - from.y);
        let len = dx.hypot(dy);
        if len < 1e-9 {
            return 0.0;
        }
        let d = (dx / len, dy / len);
        // Angular resolution around the tool, and how densely to sample the move.
        const NA: usize = 180;
        // Sample the perimeter a hair *inside* r. The cleared region is a flattened
        // (inscribed) polygon, so probing at exactly r lets perimeter points graze just
        // outside a tangent cleared boundary and read as uncut — a false slot when the
        // tool sits in its own entry disc. The inset (> the flatten sagitta) removes
        // that while costing a negligible under-read of a_e.
        let rp = (self.r - 0.1).max(self.r * 0.9);
        let pos_steps = ((len / (0.5 * self.r).max(1e-3)).ceil() as usize).max(1);
        let mut max_ae = 0.0_f64;
        for s in 0..=pos_steps {
            let t = len * (s as f64) / (pos_steps as f64);
            let c = Point::new(from.x + d.0 * t, from.y + d.1 * t);
            // Angular measure of the leading perimeter arc that is cutting uncut stock.
            let mut engaged = 0usize;
            for k in 0..NA {
                let a = std::f64::consts::TAU * (k as f64) / (NA as f64);
                let (ca, sa) = (a.cos(), a.sin());
                if ca * d.0 + sa * d.1 <= 0.0 {
                    continue; // trailing half — not the cutting edge
                }
                if self.is_uncut(Point::new(c.x + rp * ca, c.y + rp * sa)) {
                    engaged += 1;
                }
            }
            let phi = std::f64::consts::TAU * (engaged as f64) / (NA as f64);
            max_ae = max_ae.max(self.r * (1.0 - phi.cos()));
        }
        max_ae
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
    // Bound it to the target so cutting air outside the part is not charged as
    // engagement. Seed the entry disc the plunge opens, so the first moves are not
    // charged for it.
    let mut model = ClearedModel::bounded(r, to_clear.clone());
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
    fn slotting_into_solid_engages_the_full_diameter() {
        // A straight cut into virgin stock is a full slot: the whole leading half of
        // the tool is in uncut stock (Φ = π), so the engagement-angle oracle reads the
        // diameter exactly (2r = 6) — far above any real cap, so a slot is always
        // rejected.
        let model = ClearedModel::new(3.0);
        let e = model.engagement(Point::new(0.0, 0.0), Point::new(40.0, 0.0));
        assert!((5.9..6.05).contains(&e), "a full slot should read the diameter 6, got {e}");
    }

    #[test]
    fn a_light_peel_alongside_cleared_stock_engages_the_stepover_exactly() {
        // Clear a wide first swath, then peel a pass a light 2 mm stepover away
        // (r=3 ⇒ the tool overlaps the cleared swath, cutting only a 2 mm strip). The
        // peel stays well *inside* the swath's extent (swath −10…50, peel 0…40) so
        // there is no end transient — the exact engagement-angle oracle reads the
        // stepover (2 mm), not an average of it.
        let mut model = ClearedModel::new(3.0);
        model.commit(Point::new(-10.0, 0.0), Point::new(50.0, 0.0));
        let e = model.engagement(Point::new(0.0, 2.0), Point::new(40.0, 2.0));
        assert!((1.85..2.15).contains(&e), "a 2 mm peel should read a_e ≈ 2, got {e}");
    }

    #[test]
    fn end_transient_past_the_swath_engages_more_than_the_stepover() {
        // The oracle's fidelity: when a peel runs off the end of the cleared swath its
        // leading edge bites virgin stock, so engagement rises above the stepover
        // there. The old 2·area/perimeter metric averaged this real spike away.
        let mut model = ClearedModel::new(3.0);
        model.commit(Point::new(0.0, 0.0), Point::new(40.0, 0.0));
        let e = model.engagement(Point::new(0.0, 2.0), Point::new(40.0, 2.0));
        assert!(e > 3.5, "running off the swath end bites virgin stock, got {e}");
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
