//! Raster clearing oracle: a fixed-grid occupancy model of cleared material.
//!
//! Measures the same three things as the polygon oracle ([`crate::clearsim`]) —
//! peak engagement, coverage of the reachable target, and gouging — but by stamping
//! the tool onto a grid and scanning cells, which is ~O(area/px²), i.e. linear in
//! path length. The polygon oracle spends a boolean op (~ms) per move, so large
//! pockets are infeasible there; this is what lets them be certified. The two agree
//! on the same paths (cross-checked in tests), so the raster is a faithful, faster
//! stand-in for the trust anchor.

#![allow(dead_code)]

use cam_geo::{Point, Polygon};

use crate::clearsim::Verdict;

/// A grid occupancy model for a tool of radius `r`.
/// Smallest cell size (mm). Below this the grid buys nothing — the residual error is
/// angular, not spatial (see [`CELL_MAX`]) — and costs memory quadratically.
const CELL_MIN: f64 = 0.05;

/// **Largest cell size (mm), and this is a safety bound, not a performance knob.**
///
/// The engagement probe sits at `r − px`, so a coarse cell pushes it deep inside the tool
/// where it misses the thin uncut band at the perimeter and the raster **under-reads** —
/// the unsafe direction for a gate. Calibrated against the exact oracle
/// ([`crate::clearsim`]) over five paths (front-advance on circle r30 / r12 / square 40 /
/// square 24, plus a deliberate slot), worst error at each cell size:
///
/// ```text
///   px 0.35 → under-reads by 0.31   ← unsafe
///   px 0.20 → under-reads by 0.21   ← unsafe (this used to be the default)
///   px 0.10 → never under-reads; over-reads ≤ 0.21
///   px 0.05 → never under-reads; over-reads ≤ 0.21
/// ```
///
/// At or below 0.10 the sign flips and stays flipped: the raster only ever over-reads, so
/// a pass is trustworthy and the cost is a false rejection (fall back to concentric, which
/// is proven). The residual 0.21 is **not** spatial — it is ~2 samples of the `NA = 180`
/// angular sweep (`da_e = r·sinΦ·dΦ` ≈ 0.10 per step), which is why 0.05 is no better than
/// 0.10. Raise `NA` if that 0.21 ever needs tightening; shrinking cells will not do it.
const CELL_MAX: f64 = 0.10;

pub(crate) struct Raster {
    r: f64,
    px: f64,
    ox: f64,
    oy: f64,
    w: usize,
    h: usize,
    /// Cells the tool has swept.
    cleared: Vec<bool>,
    /// Cells whose centre lies in the target material (for gouge).
    region: Vec<bool>,
    /// Cells a radius-`r` tool can reach (target opened by `r`; for coverage).
    reach: Vec<bool>,
}

/// Rasterize a set of polygons (outer contours filled, holes cleared) onto a
/// `w×h` grid at origin `(ox,oy)` cell `px`, by scanline fill — O(rows·edges + cells),
/// far cheaper than a point-in-polygon test per cell.
fn fill_mask(polys: &[Polygon], ox: f64, oy: f64, px: f64, w: usize, h: usize) -> Vec<bool> {
    let mut mask = vec![false; w * h];
    let mut xs: Vec<f64> = Vec::new();
    for j in 0..h {
        let cy = oy + (j as f64 + 0.5) * px;
        xs.clear();
        for poly in polys {
            let rings = std::iter::once(poly.outer().points())
                .chain(poly.holes().iter().map(|hle| hle.points()));
            for ring in rings {
                let n = ring.len();
                for k in 0..n {
                    let a = ring[k];
                    let b = ring[(k + 1) % n];
                    // Edge crosses the scanline (half-open, so shared vertices count once).
                    if (a.y <= cy) != (b.y <= cy) {
                        let t = (cy - a.y) / (b.y - a.y);
                        xs.push(a.x + t * (b.x - a.x));
                    }
                }
            }
        }
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        for pair in xs.chunks(2) {
            if pair.len() < 2 {
                break;
            }
            let i0 = (((pair[0] - ox) / px).ceil().max(0.0)) as usize;
            let i1 = (((pair[1] - ox) / px).floor()).clamp(0.0, w as f64 - 1.0) as usize;
            for i in i0..=i1.min(w - 1) {
                if i < w {
                    mask[j * w + i] = true;
                }
            }
        }
    }
    mask
}

/// Squared distance from `p` to segment `a→b`.
fn seg_dist_sq(p: Point, a: Point, b: Point) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    if len2 <= 1e-12 {
        return p.distance_sq(a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
    p.distance_sq(Point::new(a.x + dx * t, a.y + dy * t))
}

impl Raster {
    /// Build a grid over `to_clear` (plus a tool-radius margin) for a tool of radius
    /// `r`, resolved fine enough to measure engagement against `cap`.
    pub(crate) fn new(to_clear: &Polygon, r: f64, cap: f64) -> Option<Self> {
        Self::with_px(to_clear, r, (cap.min(r) / 10.0).clamp(CELL_MIN, CELL_MAX))
    }

    /// [`Raster::new`] with the cell size given, so calibration can sweep it.
    pub(crate) fn with_px(to_clear: &Polygon, r: f64, px: f64) -> Option<Self> {
        let pts = to_clear.outer().points();
        let (mut xmin, mut ymin) = (f64::MAX, f64::MAX);
        let (mut xmax, mut ymax) = (f64::MIN, f64::MIN);
        for p in pts {
            xmin = xmin.min(p.x);
            ymin = ymin.min(p.y);
            xmax = xmax.max(p.x);
            ymax = ymax.max(p.y);
        }
        if !xmin.is_finite() || xmax <= xmin {
            return None;
        }
        let margin = r + 3.0 * px;
        let (ox, oy) = (xmin - margin, ymin - margin);
        let w = (((xmax + margin) - ox) / px).ceil() as usize + 1;
        let h = (((ymax + margin) - oy) / px).ceil() as usize + 1;
        if w == 0 || h == 0 || w.saturating_mul(h) > 8_000_000 {
            return None;
        }

        let reach_polys = crate::clearsim::reachable(to_clear, r);
        let region = fill_mask(std::slice::from_ref(to_clear), ox, oy, px, w, h);
        let reach = fill_mask(&reach_polys, ox, oy, px, w, h);
        Some(Self {
            r,
            px,
            ox,
            oy,
            w,
            h,
            cleared: vec![false; w * h],
            region,
            reach,
        })
    }

    #[inline]
    fn cell_of(&self, x: f64, y: f64) -> (usize, usize) {
        let i = (((x - self.ox) / self.px) as isize).clamp(0, self.w as isize - 1) as usize;
        let j = (((y - self.oy) / self.px) as isize).clamp(0, self.h as isize - 1) as usize;
        (i, j)
    }

    #[inline]
    fn centre(&self, i: usize, j: usize) -> Point {
        Point::new(self.ox + (i as f64 + 0.5) * self.px, self.oy + (j as f64 + 0.5) * self.px)
    }

    /// Whether the cell at `p` is currently uncut material (in the target, not yet
    /// cleared). Out-of-grid points read as not-material.
    #[inline]
    /// Whether the cell containing `p` is uncut target material.
    fn is_uncut(&self, p: Point) -> bool {
        let i = ((p.x - self.ox) / self.px).floor();
        let j = ((p.y - self.oy) / self.px).floor();
        if i < 0.0 || j < 0.0 || i >= self.w as f64 || j >= self.h as f64 {
            return false;
        }
        let idx = (j as usize) * self.w + i as usize;
        self.region[idx] && !self.cleared[idx]
    }

    /// Seed the entry disc a plunge opens at `c`.
    pub(crate) fn seed_disc(&mut self, c: Point) {
        self.stamp(c, c);
    }

    /// Stamp every cell within `r` of segment `a→b` as cleared.
    pub(crate) fn stamp(&mut self, a: Point, b: Point) {
        let (i0, j0) = self.cell_of(a.x.min(b.x) - self.r, a.y.min(b.y) - self.r);
        let (i1, j1) = self.cell_of(a.x.max(b.x) + self.r, a.y.max(b.y) + self.r);
        let r2 = self.r * self.r;
        for j in j0..=j1 {
            for i in i0..=i1 {
                let c = self.centre(i, j);
                if seg_dist_sq(c, a, b) <= r2 {
                    self.cleared[j * self.w + i] = true;
                }
            }
        }
    }

    /// The **radial width of cut** (`a_e`) of the move `a`→`b`, by the tool-engagement
    /// angle — the same measure as [`crate::clearsim::ClearedModel::engagement`], but read
    /// off this occupancy grid instead of point-in-polygon tests:
    ///
    /// ```text
    ///   a_e = r · (1 − cos Φ)
    /// ```
    ///
    /// where `Φ` is the angular span of the tool's **leading** perimeter lying in uncut
    /// stock. A full slot reads the diameter; a peel reads the stepover.
    ///
    /// **This replaces a measure that was not engagement at all.** It used to take the
    /// longest contiguous run of uncut cells *along the perpendicular through the tool
    /// centre* — "how wide is the band beside me". That is structurally blind to material
    /// **ahead** of the tool: a cutter driving into a wall with cleared stock either side
    /// reads ≈0 while slotting at full width. It shipped full-diameter slots for exactly
    /// that reason (measured: it read 0.80 where the truth was 6.00, a 7.5× under-read in
    /// the unsafe direction, on a plain square pocket). No amount of resolution fixes a
    /// wrong quantity, so the formula had to go, not the pixel size.
    ///
    /// The grid is what keeps this affordable: `is_uncut` is an O(1) lookup, where the
    /// polygon oracle rescans an accumulating set of cleared polygons per query. Same
    /// answer, without the quadratic blow-up.
    pub(crate) fn engagement(&self, a: Point, b: Point) -> f64 {
        let len = a.distance(b);
        if len < 1e-9 {
            return 0.0;
        }
        let d = ((b.x - a.x) / len, (b.y - a.y) / len);
        /// Angular resolution around the tool.
        const NA: usize = 180;
        // Probe a hair inside `r`: at exactly `r` the perimeter grazes the cell boundary
        // of its own swept disc and reads a false slot.
        let rp = (self.r - self.px).max(self.r * 0.9);
        let pos_steps = ((len / (0.5 * self.r).max(1e-3)).ceil() as usize).max(1);
        let mut max_ae = 0.0_f64;
        for si in 0..=pos_steps {
            let t = len * (si as f64) / (pos_steps as f64);
            let c = Point::new(a.x + d.0 * t, a.y + d.1 * t);
            let mut engaged = 0usize;
            for k in 0..NA {
                let ang = std::f64::consts::TAU * (k as f64) / (NA as f64);
                let (ca, sa) = (ang.cos(), ang.sin());
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

    /// Uncut reachable material remaining (mm²).
    pub(crate) fn uncut_area(&self) -> f64 {
        let px2 = self.px * self.px;
        (0..self.reach.len())
            .filter(|&i| self.reach[i] && !self.cleared[i])
            .count() as f64
            * px2
    }

    /// Material cleared outside the target — a gouge (mm²).
    pub(crate) fn gouge_area(&self) -> f64 {
        let px2 = self.px * self.px;
        (0..self.cleared.len())
            .filter(|&i| self.cleared[i] && !self.region[i])
            .count() as f64
            * px2
    }
}

/// Certify a cutting path against `to_clear` for a tool of radius `r`, using the
/// raster oracle. Returns `None` if the grid could not be built (degenerate target).
pub(crate) fn certify(path: &[Point], r: f64, to_clear: &Polygon, cap: f64) -> Option<Verdict> {
    let mut ras = Raster::new(to_clear, r, cap)?;
    if let Some(first) = path.first() {
        ras.seed_disc(*first);
    }
    let mut max_e = 0.0_f64;
    for w in path.windows(2) {
        max_e = max_e.max(ras.engagement(w[0], w[1]));
        ras.stamp(w[0], w[1]);
    }
    Some(Verdict {
        max_engagement: max_e,
        uncut_area: ras.uncut_area(),
        gouge_area: ras.gouge_area(),
    })
}

/// Certify a path of `(point, is_cut)` moves — as an island/frame path needs, where
/// the tool lifts (rapid) between loop families. A rapid does not cut (no stamp, no
/// engagement); each cut that follows a rapid is a plunge, so the entry disc is
/// seeded there. Returns `None` if the grid could not be built.
pub(crate) fn certify_moves(
    moves: &[(Point, bool)],
    r: f64,
    to_clear: &Polygon,
    cap: f64,
) -> Option<Verdict> {
    let mut ras = Raster::new(to_clear, r, cap)?;
    let mut prev: Option<Point> = None;
    let mut prev_cut = false;
    let mut max_e = 0.0_f64;
    for &(p, cut) in moves {
        if let Some(pp) = prev {
            if cut {
                if !prev_cut {
                    ras.seed_disc(pp); // a cut after a rapid = a plunge at pp
                }
                max_e = max_e.max(ras.engagement(pp, p));
                ras.stamp(pp, p);
            }
        }
        prev = Some(p);
        prev_cut = cut;
    }
    Some(Verdict {
        max_engagement: max_e,
        uncut_area: ras.uncut_area(),
        gouge_area: ras.gouge_area(),
    })
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
    fn raster_slotting_reads_near_the_diameter() {
        let ras = Raster::new(&square(-10.0, 50.0), 3.0, 2.0).unwrap();
        let e = ras.engagement(Point::new(0.0, 20.0), Point::new(40.0, 20.0));
        assert!((5.0..6.6).contains(&e), "slot ≈ diameter 6, got {e}");
    }

    #[test]
    fn raster_peel_reads_about_the_stepover() {
        // Mirrors `clearsim`'s ground case exactly, so the two oracles are held to the
        // same standard: the peel must stay well **inside** the cleared swath's extent
        // (swath −10…50, peel 0…40), or the tool meets virgin stock at the swath's end
        // and the reading is an end transient rather than the peel.
        //
        // This test used to stamp 0…40 and peel 0…40 — starting *at* the swath's end —
        // and accept anything in 1.0..3.2. It could afford that sloppiness because the old
        // formula measured the uncut run on the perpendicular through the tool centre and
        // was blind to the material ahead. With the engagement angle it reads the
        // transient honestly (3.73), so the setup has to be correct now.
        let mut ras = Raster::new(&square(-20.0, 60.0), 3.0, 2.0).unwrap();
        ras.stamp(Point::new(-10.0, 20.0), Point::new(50.0, 20.0));
        let e = ras.engagement(Point::new(0.0, 22.0), Point::new(40.0, 22.0));
        assert!((1.7..2.3).contains(&e), "a 2 mm peel should read a_e ≈ 2, got {e}");
    }

    #[test]
    fn raster_flags_coverage_and_gouge() {
        // A path whose tool stays inside [0,20]² covers it (minus the reachable
        // opening) and does not gouge; one that runs out gouges.
        let mut ras = Raster::new(&square(0.0, 20.0), 2.0, 2.0).unwrap();
        let mut y = 2.0;
        while y <= 18.0 + 1e-9 {
            ras.stamp(Point::new(2.0, y), Point::new(18.0, y));
            y += 2.0;
        }
        assert!(ras.uncut_area() < 8.0, "covered, uncut {}", ras.uncut_area());
        assert!(ras.gouge_area() < 1.0, "no gouge, got {}", ras.gouge_area());

        let mut g = Raster::new(&square(0.0, 20.0), 2.0, 2.0).unwrap();
        g.stamp(Point::new(10.0, 10.0), Point::new(30.0, 10.0));
        assert!(g.gouge_area() > 1.0, "a cut past the edge gouges, got {}", g.gouge_area());
    }
}
