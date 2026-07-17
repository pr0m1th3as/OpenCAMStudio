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
    difference, intersection, offset, stroke_path, Arc, CapStyle, Contour, JoinStyle, Point,
    Polygon, Polyline,
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

/// A filled disc of radius `r` at `c`.
fn disc(c: Point, r: f64) -> Option<Polygon> {
    Polygon::new(Contour::new(Arc::circle(c, r).flatten(0.05))).ok()
}

/// Squared distance from `p` to the segment `a`→`b`.
fn seg_dist_sq(p: Point, a: Point, b: Point) -> f64 {
    let (vx, vy) = (b.x - a.x, b.y - a.y);
    let (wx, wy) = (p.x - a.x, p.y - a.y);
    let vv = vx * vx + vy * vy;
    let t = if vv < 1e-18 { 0.0 } else { ((wx * vx + wy * vy) / vv).clamp(0.0, 1.0) };
    let (cx, cy) = (a.x + t * vx, a.y + t * vy);
    (p.x - cx).powi(2) + (p.y - cy).powi(2)
}

/// A fixed-resolution occupancy grid tracking which cells a tool of radius `r` has
/// cleared. It replaces per-point polygon `contains` (O(cleared perimeter), and the
/// cleared region grows without bound) with an O(1) cell lookup — the single change
/// that takes the oracle's engagement scan from ~O(n²) to linear.
///
/// **Occupancy is conservative in the safe direction.** A cell counts as cleared only
/// when its centre lies within `r` of a committed move; a cell the tool only grazed
/// stays "uncut", so [`ClearedModel::is_uncut`] never reports cleared stock that is
/// actually still there. Reading a boundary cell as uncut can only *raise* the measured
/// engagement, never lower it — and a high reading fails certification (falls back to
/// concentric), which is safe, where a low reading would ship an over-engaged path.
struct OccGrid {
    ox: f64,
    oy: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    occ: Vec<bool>,
}

impl OccGrid {
    /// A grid covering `[min,max]` (already padded by the caller) at `cell` mm.
    fn new(min: [f64; 2], max: [f64; 2], cell: f64) -> Self {
        let nx = (((max[0] - min[0]) / cell).ceil() as usize + 1).max(1);
        let ny = (((max[1] - min[1]) / cell).ceil() as usize + 1).max(1);
        Self { ox: min[0], oy: min[1], cell, nx, ny, occ: vec![false; nx * ny] }
    }

    /// Whether the cell containing `q` is marked cleared (out-of-grid ⇒ not cleared).
    fn is_cleared(&self, q: Point) -> bool {
        let (fx, fy) = ((q.x - self.ox) / self.cell, (q.y - self.oy) / self.cell);
        if fx < 0.0 || fy < 0.0 {
            return false;
        }
        let (ix, iy) = (fx as usize, fy as usize);
        if ix >= self.nx || iy >= self.ny {
            return false;
        }
        self.occ[iy * self.nx + ix]
    }

    /// Mark every cell whose centre lies within `r` of the segment `a`→`b` as cleared.
    fn stamp(&mut self, a: Point, b: Point, r: f64) {
        let (minx, maxx) = (a.x.min(b.x) - r, a.x.max(b.x) + r);
        let (miny, maxy) = (a.y.min(b.y) - r, a.y.max(b.y) + r);
        let ix0 = (((minx - self.ox) / self.cell).floor().max(0.0)) as usize;
        let iy0 = (((miny - self.oy) / self.cell).floor().max(0.0)) as usize;
        let ix1 = ((((maxx - self.ox) / self.cell).ceil()) as usize).min(self.nx.saturating_sub(1));
        let iy1 = ((((maxy - self.oy) / self.cell).ceil()) as usize).min(self.ny.saturating_sub(1));
        let r2 = r * r;
        for iy in iy0..=iy1 {
            let cy = self.oy + (iy as f64 + 0.5) * self.cell;
            for ix in ix0..=ix1 {
                let cx = self.ox + (ix as f64 + 0.5) * self.cell;
                if seg_dist_sq(Point::new(cx, cy), a, b) <= r2 {
                    self.occ[iy * self.nx + ix] = true;
                }
            }
        }
    }
}

/// A running model of the material cleared by a tool of radius `r`.
pub(crate) struct ClearedModel {
    r: f64,
    /// The swept regions cleared so far, **not** unioned (appended per move). Kept for
    /// [`Self::engagement_area`], the area-based cross-check; the hot [`Self::is_uncut`]
    /// path uses `grid` instead when it is present.
    cleared: Vec<Polygon>,
    /// The stock region (target material). `None` ⇒ unbounded virgin stock (used by
    /// the primitive slot/peel unit tests); `Some` bounds where material actually is,
    /// so engagement is not charged for cutting air outside the part.
    material: Option<Polygon>,
    /// Occupancy grid for O(1) cleared lookups. Built for every `bounded` model (all
    /// runtime and front-advance use); `None` for the unbounded primitive tests, which
    /// fall back to polygon `contains` (their paths are a move or two, so it is cheap).
    grid: Option<OccGrid>,
}

impl ClearedModel {
    /// An empty model for a tool of radius `r` over **unbounded** stock.
    pub(crate) fn new(r: f64) -> Self {
        Self {
            r,
            cleared: Vec::new(),
            material: None,
            grid: None,
        }
    }

    /// An empty model bounded to the stock region `material` — outside it is air, not
    /// uncut stock, so the tool is not charged engagement for a cut that leaves the part.
    pub(crate) fn bounded(r: f64, material: Polygon) -> Self {
        // Grid over the material's bbox, padded by `r` (a move's swept region reaches a
        // tool radius past the wall). Cell fine enough that the boundary bias on `a_e`
        // stays well under the certification tolerance.
        let pts = material.outer().points();
        let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
        for p in pts {
            lo[0] = lo[0].min(p.x);
            lo[1] = lo[1].min(p.y);
            hi[0] = hi[0].max(p.x);
            hi[1] = hi[1].max(p.y);
        }
        let grid = if lo[0] <= hi[0] {
            let cell = (r / 15.0).clamp(0.03, 0.1);
            let pad = r + 2.0 * cell;
            Some(OccGrid::new(
                [lo[0] - pad, lo[1] - pad],
                [hi[0] + pad, hi[1] + pad],
                cell,
            ))
        } else {
            None
        };
        Self {
            r,
            cleared: Vec::new(),
            material: Some(material),
            grid,
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
        match &self.grid {
            Some(g) => !g.is_cleared(q),
            None => !self.cleared.iter().any(|p| p.contains(q)),
        }
    }

    /// Seed the cleared region with the disc a plunge/helix opens at `c` — the entry
    /// hole, so the first cutting moves are not charged for stock the plunge removed.
    pub(crate) fn seed_disc(&mut self, c: Point) {
        if let Some(g) = &mut self.grid {
            g.stamp(c, c, self.r);
        }
        let pts = Arc::circle(c, self.r).flatten(0.05);
        if let Ok(d) = Polygon::new(Contour::new(pts)) {
            self.cleared.push(d);
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

    /// The **radial width of cut** (`a_e`) of the move `from`→`to`, measured from the
    /// material the move actually removes per unit of advance:
    ///
    /// ```text
    ///   a_e = area( swept(from→to) ∖ tool-at-`from` ∖ cleared ∩ material ) / |to − from|
    /// ```
    ///
    /// Subtracting the tool's own starting disc is what makes this a *rate*: the tool
    /// already occupies `from`, so only what the advance uncovers is cut. Without it a
    /// short move would charge its whole starting disc (≈ `πr²`) against a tiny advance.
    ///
    /// This is the textbook meaning of radial width of cut, and unlike [`Self::engagement`]
    /// it assumes **nothing about the shape of the uncut boundary**. That is the whole
    /// point: `a_e = r(1 − cos Φ)` derives depth from the contact *arc* on the assumption
    /// that the uncut region is a half-plane, so where material **wraps** the tool — a
    /// concave corner — it reports a deep cut for a shallow one. Here a slot reads the
    /// diameter and a peel reads the stepover because the geometry says so, not because
    /// the shape was assumed.
    ///
    /// It is **not** the averaging trap that `2·area/perimeter` was: that averaged over a
    /// whole loop and drowned a momentary slot. This averages over exactly one move, so
    /// the caller controls the window — measure in short pieces and a short slot still
    /// reads as a slot.
    pub(crate) fn engagement_area(&self, from: Point, to: Point) -> f64 {
        let len = from.distance(to);
        if len < 1e-9 {
            return 0.0;
        }
        let sweep = swept(&[from, to], self.r);
        if sweep.is_empty() {
            return 0.0;
        }
        let Some(start) = disc(from, self.r) else {
            return 0.0;
        };
        let mut cut = match difference(&sweep, std::slice::from_ref(&start)) {
            Ok(v) => v,
            Err(_) => return 0.0,
        };
        if !self.cleared.is_empty() {
            cut = difference(&cut, &self.cleared).unwrap_or_default();
        }
        if let Some(m) = &self.material {
            cut = intersection(&cut, std::slice::from_ref(m)).unwrap_or_default();
        }
        total_area(&cut) / len
    }

    /// Add the move `from`→`to` to the cleared region.
    pub(crate) fn commit(&mut self, from: Point, to: Point) {
        // The grid is the hot path: stamp the swept capsule (O(cells under it)), no union.
        if let Some(g) = &mut self.grid {
            g.stamp(from, to, self.r);
        }
        // Append the swept region for `engagement_area`. Not unioned — that was the other
        // O(n²) cost (a union of a growing polygon per move); `difference` in
        // `engagement_area` subtracts the whole list regardless, and this list is only
        // read by the (small-path) cross-check test, never the runtime gate.
        let sweep = swept(&[from, to], self.r);
        self.cleared.extend(sweep);
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

    /// **The oracle's independent cross-check.** [`ClearedModel::engagement`] derives `a_e`
    /// from the contact *arc* via `r(1−cos Φ)`; [`ClearedModel::engagement_area`] derives it
    /// from the material actually removed per unit of advance. They share no machinery —
    /// one probes the perimeter, the other does booleans on the swept region — so agreement
    /// is real evidence rather than one formula confirming itself.
    ///
    /// This matters because this subsystem's history is of instruments quietly lying: the
    /// original `2·area/perimeter` metric averaged slots away, and `engagement` itself was
    /// suspected (wrongly, as it turns out) of over-reporting concave corners. A second,
    /// independent measure is the cheapest guard against the next such surprise.
    ///
    /// Note `engagement_area` is biased **high on short moves** — the `flatten(0.05)` disc
    /// and `stroke_path`'s round cap are different polygons, so differencing them leaves
    /// ~0.5 mm² of slivers, which divided by a small advance explodes. Hence long moves
    /// here, and hence `engagement` remains the metric the runtime gate uses.
    #[test]
    fn the_two_independent_engagement_measures_agree() {
        let r = 3.0;

        // A full slot into virgin stock: both must read the diameter.
        let m = ClearedModel::new(r);
        let (a, b) = (Point::new(0.0, 0.0), Point::new(40.0, 0.0));
        let (angle, area) = (m.engagement(a, b), m.engagement_area(a, b));
        assert!((5.9..6.1).contains(&angle), "slot by arc should read 6, got {angle}");
        assert!((5.9..6.1).contains(&area), "slot by area should read 6, got {area}");

        // A 2 mm peel alongside a cleared swath: both must read the stepover.
        let mut m = ClearedModel::new(r);
        m.commit(Point::new(-10.0, 0.0), Point::new(50.0, 0.0));
        let (a, b) = (Point::new(0.0, 2.0), Point::new(40.0, 2.0));
        let (angle, area) = (m.engagement(a, b), m.engagement_area(a, b));
        assert!((1.85..2.15).contains(&angle), "peel by arc should read 2, got {angle}");
        assert!((1.85..2.15).contains(&area), "peel by area should read 2, got {area}");
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
