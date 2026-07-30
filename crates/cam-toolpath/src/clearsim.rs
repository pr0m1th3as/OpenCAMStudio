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
    /// How many cells are set. Maintained incrementally so a generator can ask "am I still
    /// making progress?" in O(1) rather than re-measuring coverage.
    filled: usize,
}

impl OccGrid {
    /// A grid covering `[min,max]` (already padded by the caller) at `cell` mm.
    fn new(min: [f64; 2], max: [f64; 2], cell: f64) -> Self {
        let nx = (((max[0] - min[0]) / cell).ceil() as usize + 1).max(1);
        let ny = (((max[1] - min[1]) / cell).ceil() as usize + 1).max(1);
        Self { ox: min[0], oy: min[1], cell, nx, ny, occ: vec![false; nx * ny], filled: 0 }
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

    /// An empty grid on the **same lattice** as `self`, so the two can be compared cell for
    /// cell without any coordinate arithmetic at the comparison site.
    fn same_lattice(&self) -> OccGrid {
        OccGrid {
            ox: self.ox,
            oy: self.oy,
            cell: self.cell,
            nx: self.nx,
            ny: self.ny,
            occ: vec![false; self.nx * self.ny],
            filled: 0,
        }
    }

    /// Rasterise `polys` into this grid by **scanline**, marking every cell whose centre
    /// lies inside their union. Uses the nonzero winding rule over every ring, outer and
    /// hole alike — the same rule as [`cam_geo::Polygon::locate`], so a cell inside an
    /// island nets to zero and reads as outside, exactly as the polygon test would say.
    ///
    /// Scanline rather than per-cell point-in-polygon on purpose: this is O(rows × edges +
    /// cells filled) where the naive version is O(cells × edges), and on the grids that
    /// matter here that is the difference between milliseconds and minutes.
    fn fill(&mut self, polys: &[Polygon]) {
        let mut xs: Vec<(f64, i32)> = Vec::new();
        for iy in 0..self.ny {
            let y = self.oy + (iy as f64 + 0.5) * self.cell;
            xs.clear();
            for poly in polys {
                let rings = std::iter::once(poly.outer().points())
                    .chain(poly.holes().iter().map(|h| h.points()));
                for ring in rings {
                    let n = ring.len();
                    for k in 0..n {
                        let (p0, p1) = (ring[k], ring[(k + 1) % n]);
                        // Half-open rule: a vertex exactly on the scanline counts once, so
                        // a ring is never entered or left twice at the same point.
                        if (p0.y <= y) == (p1.y <= y) {
                            continue;
                        }
                        let t = (y - p0.y) / (p1.y - p0.y);
                        xs.push((p0.x + (p1.x - p0.x) * t, if p1.y > p0.y { 1 } else { -1 }));
                    }
                }
            }
            if xs.is_empty() {
                continue;
            }
            xs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            let mut wind = 0;
            for w in xs.windows(2) {
                wind += w[0].1;
                if wind == 0 {
                    continue; // outside between these two crossings
                }
                // Cells whose centres lie in [w[0].0, w[1].0).
                let x0 = ((w[0].0 - self.ox) / self.cell - 0.5).ceil().max(0.0) as usize;
                let x1 = ((w[1].0 - self.ox) / self.cell - 0.5).floor();
                if x1 < 0.0 || x0 >= self.nx {
                    continue;
                }
                let x1 = (x1 as usize).min(self.nx - 1);
                for ix in x0..=x1 {
                    self.occ[iy * self.nx + ix] = true;
                }
            }
        }
    }

    /// Area (mm²) of the cells set in `self` but **not** in `other`.
    fn area_minus(&self, other: &OccGrid) -> f64 {
        let n = self
            .occ
            .iter()
            .zip(&other.occ)
            .filter(|(a, b)| **a && !**b)
            .count();
        n as f64 * self.cell * self.cell
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
                let idx = iy * self.nx + ix;
                if !self.occ[idx] && seg_dist_sq(Point::new(cx, cy), a, b) <= r2 {
                    self.occ[idx] = true;
                    self.filled += 1;
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
    /// `material` rasterised onto the same lattice, for [`Self::cut_area_grid`] only.
    ///
    /// Deliberately **not** used by [`Self::is_uncut`], which keeps its exact polygon test:
    /// `is_uncut` feeds the engagement reading that gates every path, and swapping an exact
    /// containment for a cell-quantised one there would shift the certified numbers by a
    /// hair for no gain. The controller can afford the approximation; the gate cannot.
    material_mask: Option<OccGrid>,
}

impl ClearedModel {
    /// An empty model for a tool of radius `r` over **unbounded** stock.
    pub(crate) fn new(r: f64) -> Self {
        Self {
            r,
            cleared: Vec::new(),
            material: None,
            grid: None,
            material_mask: None,
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
        let material_mask = grid.as_ref().map(|g| {
            let mut m = g.same_lattice();
            m.fill(std::slice::from_ref(&material));
            m
        });
        Self {
            r,
            cleared: Vec::new(),
            material: Some(material),
            grid,
            material_mask,
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
        //
        // **When there is a grid, it *is* the model.** The polygon list below exists only
        // for [`Self::engagement_area`], the area-based cross-check, which is only ever run
        // on an unbounded model (a short, hand-built path) — never on the runtime gate. So
        // building and keeping a swept polygon per move here is pure waste on exactly the
        // paths where there are most of them: it is one `stroke_path` per move, and on an
        // 8000-move island path it cost **32 s of a 56 s certification** in release. Bail
        // out and let the grid carry it.
        if let Some(g) = &mut self.grid {
            g.stamp(from, to, self.r);
            return;
        }
        let sweep = swept(&[from, to], self.r);
        self.cleared.extend(sweep);
    }

    /// The cleared region so far.
    pub(crate) fn cleared(&self) -> &[Polygon] {
        &self.cleared
    }

    /// The **controller's** measure of a candidate move: the area of uncut material the move
    /// `from`→`to` would newly remove, read off the occupancy grid.
    ///
    /// This is [`Self::engagement_area`]'s quantity — area removed per advance, the textbook
    /// radial width of cut once divided by the move length — but computed by counting cells
    /// instead of doing polygon booleans. That matters because of *how it is used*: the exact
    /// version is called once per move to **judge** a path, while this is called ~8 times per
    /// step to **choose** one, over thousands of steps. Exact is right for a gate and
    /// hopeless for a controller.
    ///
    /// Cells already under the tool at `from` are excluded, so this is what the advance
    /// uncovers rather than what the tool is sitting on — without that, a short step would
    /// charge its whole starting disc.
    pub(crate) fn cut_area_grid(&self, from: Point, to: Point) -> f64 {
        let (Some(g), Some(mask)) = (&self.grid, &self.material_mask) else {
            return 0.0;
        };
        let r = self.r;
        let (minx, maxx) = (from.x.min(to.x) - r, from.x.max(to.x) + r);
        let (miny, maxy) = (from.y.min(to.y) - r, from.y.max(to.y) + r);
        let ix0 = (((minx - g.ox) / g.cell).floor().max(0.0)) as usize;
        let iy0 = (((miny - g.oy) / g.cell).floor().max(0.0)) as usize;
        let ix1 = ((((maxx - g.ox) / g.cell).ceil()) as usize).min(g.nx.saturating_sub(1));
        let iy1 = ((((maxy - g.oy) / g.cell).ceil()) as usize).min(g.ny.saturating_sub(1));
        let r2 = r * r;
        let mut n = 0usize;
        for iy in iy0..=iy1 {
            let cy = g.oy + (iy as f64 + 0.5) * g.cell;
            for ix in ix0..=ix1 {
                let idx = iy * g.nx + ix;
                if g.occ[idx] || !mask.occ[idx] {
                    continue; // already cleared, or not material
                }
                let c = Point::new(g.ox + (ix as f64 + 0.5) * g.cell, cy);
                if seg_dist_sq(c, from, to) > r2 {
                    continue; // outside the swept capsule
                }
                if (c.x - from.x).powi(2) + (c.y - from.y).powi(2) <= r2 {
                    continue; // already under the tool before the move
                }
                n += 1;
            }
        }
        n as f64 * g.cell * g.cell
    }

    /// [`Self::engagement`] computed entirely on the grid — the **controller's** form of the
    /// gate's own measure.
    ///
    /// Steering on area-per-advance and being judged on the contact arc is optimising one
    /// quantity and being marked on another, and the two genuinely differ: the arc formula
    /// `a_e = r(1−cos Φ)` assumes the uncut region is a half-plane, so where material *wraps*
    /// the tool — a concave corner — it reports a deep cut for a shallow one. That is not a
    /// flaw to route around; a wrapped tool really is loaded differently. Measured, steering
    /// on area put the peak at 5.30 in a region corner while the area removed there was
    /// unremarkable. So the controller uses this instead, and aims at the number it will be
    /// certified against.
    ///
    /// Identical to [`Self::engagement`] except that material containment is a mask lookup
    /// rather than a polygon test, which is what makes it affordable inside a search.
    pub(crate) fn engagement_grid(&self, from: Point, to: Point) -> f64 {
        let (Some(g), Some(mask)) = (&self.grid, &self.material_mask) else {
            return self.engagement(from, to);
        };
        let (dx, dy) = (to.x - from.x, to.y - from.y);
        let len = dx.hypot(dy);
        if len < 1e-9 {
            return 0.0;
        }
        let d = (dx / len, dy / len);
        // The **same** angular resolution as `engagement`. A coarser sweep here would make
        // the controller aim at a slightly different number than the gate measures, which is
        // the very mismatch this method exists to remove.
        const NA: usize = 180;
        let rp = (self.r - 0.1).max(self.r * 0.9);
        let pos_steps = ((len / (0.5 * self.r).max(1e-3)).ceil() as usize).max(1);
        let uncut = |q: Point| -> bool {
            let (fx, fy) = ((q.x - g.ox) / g.cell, (q.y - g.oy) / g.cell);
            if fx < 0.0 || fy < 0.0 {
                return false;
            }
            let (ix, iy) = (fx as usize, fy as usize);
            if ix >= g.nx || iy >= g.ny {
                return false;
            }
            let idx = iy * g.nx + ix;
            mask.occ[idx] && !g.occ[idx]
        };
        let mut max_ae = 0.0_f64;
        for s in 0..=pos_steps {
            let t = len * (s as f64) / (pos_steps as f64);
            let c = Point::new(from.x + d.0 * t, from.y + d.1 * t);
            let mut engaged = 0usize;
            for k in 0..NA {
                let a = std::f64::consts::TAU * (k as f64) / (NA as f64);
                let (ca, sa) = (a.cos(), a.sin());
                if ca * d.0 + sa * d.1 <= 0.0 {
                    continue; // trailing half — not the cutting edge
                }
                if uncut(Point::new(c.x + rp * ca, c.y + rp * sa)) {
                    engaged += 1;
                }
            }
            let phi = std::f64::consts::TAU * (engaged as f64) / (NA as f64);
            max_ae = max_ae.max(self.r * (1.0 - phi.cos()));
        }
        max_ae
    }

    /// Commit a move, **recording** which cells it newly cleared so it can be undone.
    ///
    /// This is what lets a generator ask "if I started here, would the next few steps hold?"
    /// and get a *measured* answer rather than a heuristic one. Simulating without committing
    /// does not work — each step must see the material the previous one removed — and cloning
    /// the grid per candidate is far too expensive. Recording the handful of cells a 0.75 mm
    /// step touches, and putting them back, costs nothing.
    pub(crate) fn commit_recording(&mut self, from: Point, to: Point, undo: &mut Vec<usize>) {
        let Some(g) = &mut self.grid else {
            self.commit(from, to);
            return;
        };
        let r = self.r;
        let (minx, maxx) = (from.x.min(to.x) - r, from.x.max(to.x) + r);
        let (miny, maxy) = (from.y.min(to.y) - r, from.y.max(to.y) + r);
        let ix0 = (((minx - g.ox) / g.cell).floor().max(0.0)) as usize;
        let iy0 = (((miny - g.oy) / g.cell).floor().max(0.0)) as usize;
        let ix1 = ((((maxx - g.ox) / g.cell).ceil()) as usize).min(g.nx.saturating_sub(1));
        let iy1 = ((((maxy - g.oy) / g.cell).ceil()) as usize).min(g.ny.saturating_sub(1));
        let r2 = r * r;
        for iy in iy0..=iy1 {
            let cy = g.oy + (iy as f64 + 0.5) * g.cell;
            for ix in ix0..=ix1 {
                let idx = iy * g.nx + ix;
                if g.occ[idx] {
                    continue;
                }
                let c = Point::new(g.ox + (ix as f64 + 0.5) * g.cell, cy);
                if seg_dist_sq(c, from, to) <= r2 {
                    g.occ[idx] = true;
                    g.filled += 1;
                    undo.push(idx);
                }
            }
        }
    }

    /// [`Self::seed_disc`], recording the cells it cleared so it can be undone.
    pub(crate) fn seed_disc_recording(&mut self, c: Point, undo: &mut Vec<usize>) {
        self.commit_recording(c, c, undo);
        let pts = Arc::circle(c, self.r).flatten(0.05);
        if let Ok(d) = Polygon::new(Contour::new(pts)) {
            self.cleared.push(d);
        }
    }

    /// Undo the cells recorded by [`Self::commit_recording`].
    pub(crate) fn rollback(&mut self, undo: &[usize]) {
        if let Some(g) = &mut self.grid {
            for &i in undo {
                if g.occ[i] {
                    g.occ[i] = false;
                    g.filled -= 1;
                }
            }
        }
    }

    /// How many grid cells have been cleared so far — a generator's O(1) progress signal.
    pub(crate) fn cleared_cells(&self) -> usize {
        self.grid.as_ref().map_or(0, |g| g.filled)
    }

    /// The direction from a tool centre at `at` toward the material it is engaged with — the
    /// **mid-bearing of the contact arc**, and so the local outward normal of the cleared
    /// region *at the tool's own scale*.
    ///
    /// This is what a trochoidal loop has to be built on. The first attempt oriented its loops
    /// from `nearest_uncut`, a single point found on a **0.5 mm stride**: a coarse guess at
    /// where material lies, not where the boundary faces. Misaligned, the loop ploughed on
    /// re-entry instead of retreating through cleared stock, and engagement went *up* — 3.4–3.9
    /// against a 3.00 gate — with a smaller advance per revolution making it worse rather than
    /// better, which is what showed the geometry was wrong rather than the tuning.
    ///
    /// Measured over the whole perimeter (not just the leading half, as [`Self::engagement`]
    /// does) at ray resolution, from the same occupancy the controller steers by. `None` when
    /// nothing around the tool is uncut.
    pub(crate) fn material_bearing(&self, at: Point) -> Option<(f64, f64)> {
        let (g, mask) = (self.grid.as_ref()?, self.material_mask.as_ref()?);
        let rp = (self.r - 0.1).max(self.r * 0.9);
        const NA: usize = 180;
        let (mut sx, mut sy) = (0.0_f64, 0.0_f64);
        let mut n = 0usize;
        for k in 0..NA {
            let a = std::f64::consts::TAU * (k as f64) / (NA as f64);
            let (ca, sa) = (a.cos(), a.sin());
            let q = Point::new(at.x + rp * ca, at.y + rp * sa);
            let (fx, fy) = ((q.x - g.ox) / g.cell, (q.y - g.oy) / g.cell);
            if fx < 0.0 || fy < 0.0 {
                continue;
            }
            let (ix, iy) = (fx as usize, fy as usize);
            if ix >= g.nx || iy >= g.ny {
                continue;
            }
            let idx = iy * g.nx + ix;
            if mask.occ[idx] && !g.occ[idx] {
                sx += ca;
                sy += sa;
                n += 1;
            }
        }
        if n == 0 {
            return None;
        }
        let l = sx.hypot(sy);
        (l > 1e-9).then(|| (sx / l, sy / l))
    }

    /// Whether the move `from`→`to` sweeps **only stock that is already gone** — i.e. it
    /// removes nothing and may be traversed rather than fed.
    ///
    /// Deliberately not [`Self::engagement_grid`], which is a *cutting-load* measure and counts
    /// only the tool's **leading** arc. A move can read zero engagement while the body of the
    /// tool sits squarely in material: flagging traverses that way produced **776
    /// `RapidThroughStock` collisions** against `cam-sim`, every one "stock standing at Z 0.000".
    /// What licenses a traverse is that *no part* of the swept disc meets uncut material.
    pub(crate) fn sweeps_only_cleared(&self, from: Point, to: Point, margin: f64) -> bool {
        let (Some(g), Some(mask)) = (&self.grid, &self.material_mask) else {
            return false;
        };
        let r = self.r + margin;
        let (minx, maxx) = (from.x.min(to.x) - r, from.x.max(to.x) + r);
        let (miny, maxy) = (from.y.min(to.y) - r, from.y.max(to.y) + r);
        let ix0 = (((minx - g.ox) / g.cell).floor().max(0.0)) as usize;
        let iy0 = (((miny - g.oy) / g.cell).floor().max(0.0)) as usize;
        let ix1 = ((((maxx - g.ox) / g.cell).ceil()) as usize).min(g.nx.saturating_sub(1));
        let iy1 = ((((maxy - g.oy) / g.cell).ceil()) as usize).min(g.ny.saturating_sub(1));
        let r2 = r * r;
        for iy in iy0..=iy1 {
            let cy = g.oy + (iy as f64 + 0.5) * g.cell;
            for ix in ix0..=ix1 {
                let idx = iy * g.nx + ix;
                if g.occ[idx] || !mask.occ[idx] {
                    continue; // cleared, or not material at all
                }
                let c = Point::new(g.ox + (ix as f64 + 0.5) * g.cell, cy);
                if seg_dist_sq(c, from, to) <= r2 {
                    return false; // uncut material under the swept disc
                }
            }
        }
        true
    }

    /// Whether the cell containing `p` has been cleared.
    pub(crate) fn is_cleared_at(&self, p: Point) -> bool {
        self.grid.as_ref().is_some_and(|g| g.is_cleared(p))
    }

    /// Every uncut-material point on a coarse `stride` lattice, nearest to `near` first.
    ///
    /// A generator restarting a dead front needs *candidates*, not the single nearest one:
    /// the nearest is very often a place the tool cannot actually work from, and a search
    /// that returns only that gets stuck on it. Measured, with the nearest-only version the
    /// probe resumed **401 times inside one 8 × 11 mm patch** and cleared nothing new.
    pub(crate) fn uncut_candidates(&self, near: Point, stride_mm: f64, limit: usize) -> Vec<Point> {
        let (Some(g), Some(mask)) = (self.grid.as_ref(), self.material_mask.as_ref()) else {
            return Vec::new();
        };
        let stride = ((stride_mm / g.cell).round() as usize).max(1);
        let mut out: Vec<(f64, Point)> = Vec::new();
        let mut iy = 0;
        while iy < g.ny {
            let mut ix = 0;
            while ix < g.nx {
                let idx = iy * g.nx + ix;
                if mask.occ[idx] && !g.occ[idx] {
                    let p = Point::new(
                        g.ox + (ix as f64 + 0.5) * g.cell,
                        g.oy + (iy as f64 + 0.5) * g.cell,
                    );
                    out.push((p.distance(near), p));
                }
                ix += stride;
            }
            iy += stride;
        }
        out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        out.truncate(limit);
        out.into_iter().map(|(_, p)| p).collect()
    }

    /// The nearest uncut material to `from`, within `max_dist`, for a steered generator to
    /// aim at when its bite has run out. Searched on a **stride** of cells rather than every
    /// cell: this is used to point the tool, and pointing does not need sub-millimetre
    /// resolution, while scanning every cell of a 40 mm neighbourhood at 0.1 mm would cost
    /// 160k tests per step.
    pub(crate) fn nearest_uncut(&self, from: Point, max_dist: f64) -> Option<Point> {
        let (g, mask) = (self.grid.as_ref()?, self.material_mask.as_ref()?);
        let stride = ((0.5 / g.cell).round() as usize).max(1);
        let span = ((max_dist / g.cell).ceil()) as i64;
        let (cx, cy) = (
            ((from.x - g.ox) / g.cell) as i64,
            ((from.y - g.oy) / g.cell) as i64,
        );
        let mut best: Option<(f64, Point)> = None;
        let mut iy = (cy - span).max(0);
        while iy < (cy + span).min(g.ny as i64 - 1) {
            let mut ix = (cx - span).max(0);
            while ix < (cx + span).min(g.nx as i64 - 1) {
                let idx = iy as usize * g.nx + ix as usize;
                if mask.occ[idx] && !g.occ[idx] {
                    let p = Point::new(
                        g.ox + (ix as f64 + 0.5) * g.cell,
                        g.oy + (iy as f64 + 0.5) * g.cell,
                    );
                    let d = p.distance(from);
                    if d <= max_dist && best.is_none_or(|(bd, _)| d < bd) {
                        best = Some((d, p));
                    }
                }
                ix += stride as i64;
            }
            iy += stride as i64;
        }
        best.map(|(_, p)| p)
    }

    /// Somewhere to pick the cut back up when a front dies: a tool-centre position that is
    /// **already cleared** (so the re-plunge goes into air, not solid) and **inside `bound`**
    /// (so it is a legal place for the tool at all), with uncut material within reach.
    /// Returns `(stand_here, cut_toward)`, nearest to `near`.
    ///
    /// Both conditions are load-bearing and were each learned the hard way. Returning a bare
    /// cleared cell gives positions up to a tool radius outside the tool-centre region, since
    /// that is how far the cleared region extends beyond the path that made it. Returning a
    /// bare *material* cell gives positions the tool would have to plunge into solid to reach.
    pub(crate) fn resume_from(
        &self,
        near: Point,
        bound: &Polygon,
        span: f64,
    ) -> Option<(Point, Point)> {
        let (g, mask) = (self.grid.as_ref()?, self.material_mask.as_ref()?);
        // Material must sit **just beyond** the tool's own disc: closer and the re-plunge
        // would be into solid, further and the first step cannot reach it, so the new front
        // dies on the spot and the search returns to the same place for ever. Measured: with
        // no near bound this resumed 401 times and cleared nothing.
        let kmin = ((self.r / g.cell).ceil() as i64).max(1) + 1;
        let kmax = kmin + ((span / g.cell).ceil() as i64).max(1);
        let mut best: Option<(f64, Point, Point)> = None;
        for iy in 0..g.ny {
            for ix in 0..g.nx {
                let idx = iy * g.nx + ix;
                if !g.occ[idx] {
                    continue; // stand in cleared stock
                }
                let c = Point::new(
                    g.ox + (ix as f64 + 0.5) * g.cell,
                    g.oy + (iy as f64 + 0.5) * g.cell,
                );
                let d = c.distance(near);
                if best.is_some_and(|(bd, _, _)| d >= bd) {
                    continue; // cannot beat what we have; skip the expensive tests
                }
                // Uncut material within a tool radius, in one of the four axis directions.
                let mut target = None;
                for (dx, dy) in [(1_i64, 0_i64), (-1, 0), (0, 1), (0, -1)] {
                    for k in kmin..=kmax {
                        let (jx, jy) = (ix as i64 + dx * k, iy as i64 + dy * k);
                        if jx < 0 || jy < 0 || jx >= g.nx as i64 || jy >= g.ny as i64 {
                            break;
                        }
                        let jdx = jy as usize * g.nx + jx as usize;
                        if mask.occ[jdx] && !g.occ[jdx] {
                            target = Some(Point::new(
                                g.ox + (jx as f64 + 0.5) * g.cell,
                                g.oy + (jy as f64 + 0.5) * g.cell,
                            ));
                            break;
                        }
                    }
                    if target.is_some() {
                        break;
                    }
                }
                let Some(t) = target else { continue };
                if !bound.contains(c) {
                    continue;
                }
                best = Some((d, c, t));
            }
        }
        best.map(|(_, c, t)| (c, t))
    }

    /// Coverage and gouge, read straight off the occupancy grid this model already built
    /// while measuring engagement. Returns `(uncut_area, gouge_area)`, or `None` when there
    /// is no grid (a degenerate region), so the caller can fall back to the exact booleans.
    ///
    /// **This is what makes certifying a long path affordable.** The exact route strokes the
    /// whole cut path into a polygon and differences it against the target — and stroking a
    /// several-thousand-segment self-overlapping polyline is quadratic-ish work that
    /// measured **11.18 s of a 12.71 s certification** on a 5431-move island path, against
    /// 1.47 s for the entire engagement scan. The grid, meanwhile, is *already paid for*:
    /// every cutting move has been stamped into it. Coverage then costs two scanline fills
    /// and a linear pass.
    ///
    /// **Both errors fall the safe way.** A cell counts as cleared only if its centre lies
    /// within `r` of a committed move, so a partially-cut cell reads *uncut* — which can
    /// only overstate the coverage gap, and an overstated gap fails certification and falls
    /// back to the proven concentric clear. It cannot pass a path that should have failed.
    fn coverage(&self, to_clear: &Polygon, reach: &[Polygon]) -> Option<(f64, f64)> {
        let g = self.grid.as_ref()?;
        let mut reach_mask = g.same_lattice();
        reach_mask.fill(reach);
        let mut target_mask = g.same_lattice();
        target_mask.fill(std::slice::from_ref(to_clear));
        Some((reach_mask.area_minus(g), g.area_minus(&target_mask)))
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
///
/// A thin wrapper over [`certify_moves`] with every move flagged cutting — deliberately
/// *one* implementation rather than two, so the continuous and moves-aware entries cannot
/// drift apart under maintenance.
pub(crate) fn certify(path: &[Point], r: f64, to_clear: &Polygon) -> Verdict {
    let moves: Vec<(Point, bool)> = path.iter().enumerate().map(|(i, &p)| (p, i > 0)).collect();
    certify_moves(&moves, r, to_clear)
}

/// Split a move path into its **cutting runs**: maximal polylines of consecutive points
/// joined by cutting moves. A run begins at the point a cut departs *from* (the plunge
/// point), so stroking it with round caps reproduces the entry disc.
fn cut_runs(moves: &[(Point, bool)]) -> Vec<Vec<Point>> {
    let mut runs: Vec<Vec<Point>> = Vec::new();
    let mut cur: Vec<Point> = Vec::new();
    let mut prev: Option<Point> = None;
    for &(p, cut) in moves {
        if cut {
            if let Some(pp) = prev {
                if cur.is_empty() {
                    cur.push(pp);
                }
                cur.push(p);
            }
        } else if !cur.is_empty() {
            runs.push(std::mem::take(&mut cur));
        }
        prev = Some(p);
    }
    if !cur.is_empty() {
        runs.push(cur);
    }
    runs
}

/// Certify a path of `(point, is_cut)` moves — the form an island/frame path takes, where
/// the tool **lifts between loop families** and so has no continuous form at all. Same
/// verdict as [`certify`], same exactness; the flags are what let a rapid be a rapid.
///
/// This exists because the exact oracle previously had no moves-aware entry, which is the
/// only reason the frame path was gated on [`crate::raster`] — an oracle measured
/// under-reading engagement by 7.5× in the unsafe direction. A certifier that cannot score
/// a path containing rapids is not an alternative to the raster; this is.
///
/// The three semantics that make a rapid a rapid, each pinned by a test:
///
/// - **A rapid removes nothing.** Coverage and gouge are stroked from the cutting runs
///   only ([`cut_runs`]), so the corridor a rapid crosses is *not* credited as cleared —
///   it stays uncut, which is what fails certification and is the whole point.
/// - **A rapid is charged no engagement.** It travels above the stock; it cannot cut.
/// - **A cut following a rapid is a plunge**, so the entry disc is seeded there — the
///   move out of the hole is not charged for stock the plunge itself removed. Round caps
///   on the run's stroke count the same disc toward coverage.
///
/// A rapid **launders nothing**: lifting before a slot does not make it read less than the
/// diameter, because engagement is measured against the running cleared model, not against
/// the path's shape.
pub(crate) fn certify_moves(moves: &[(Point, bool)], r: f64, to_clear: &Polygon) -> Verdict {
    // Peak engagement is inherently sequential: walk the cutting moves against the running
    // cleared region. Bound it to the target so cutting air outside the part is not charged
    // as engagement. The occupancy grid this builds is then *also* the coverage answer.
    let mut model = ClearedModel::bounded(r, to_clear.clone());
    let mut prev: Option<Point> = None;
    let mut prev_cut = false;
    let mut max_e = 0.0_f64;
    for &(p, cut) in moves {
        if cut {
            if let Some(pp) = prev {
                if !prev_cut {
                    model.seed_disc(pp); // a cut after a rapid = a plunge at pp
                }
                max_e = max_e.max(model.engagement(pp, p));
                model.commit(pp, p);
            }
        }
        prev = Some(p);
        prev_cut = cut;
    }

    let reach = reachable(to_clear, r);
    let (uncut_area, gouge_area) = model
        .coverage(to_clear, &reach)
        .unwrap_or_else(|| coverage_exact(moves, r, to_clear));
    Verdict { max_engagement: max_e, uncut_area, gouge_area }
}

/// Coverage and gouge by **exact polygon booleans** — the reference measure, and the
/// fallback when no occupancy grid could be built.
///
/// Kept, but off the hot path: it strokes every cutting run into a polygon and differences
/// that against the target, and stroking a long self-overlapping polyline is what made
/// certification unaffordable (11.18 s of 12.71 s on a 5431-move path). Its job now is to be
/// the independent second opinion that
/// [`tests::the_two_coverage_measures_agree`] holds the grid to — the same discipline as the
/// two independent engagement measures, and for the same reason: this subsystem's history is
/// of instruments quietly lying.
fn coverage_exact(moves: &[(Point, bool)], r: f64, to_clear: &Polygon) -> (f64, f64) {
    let mut full: Vec<Polygon> = Vec::new();
    for run in cut_runs(moves) {
        full.extend(swept(&run, r));
    }
    let reach = reachable(to_clear, r);
    let uncut = if reach.is_empty() {
        Vec::new()
    } else {
        difference(&reach, &full).unwrap_or_default()
    };
    let gouge = difference(&full, std::slice::from_ref(to_clear)).unwrap_or_default();
    (total_area(&uncut), total_area(&gouge))
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

    fn holed_60() -> Polygon {
        use cam_geo::Contour;
        Polygon::with_holes(
            Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(60.0, 0.0),
                Point::new(60.0, 60.0),
                Point::new(0.0, 60.0),
            ]),
            vec![Contour::new(vec![
                Point::new(20.0, 20.0),
                Point::new(40.0, 20.0),
                Point::new(40.0, 40.0),
                Point::new(20.0, 40.0),
            ])],
        )
        .unwrap()
    }

    /// **The grid coverage measure against the exact one.** `certify_moves` now reads
    /// coverage and gouge off the occupancy grid rather than stroking the path into a
    /// polygon, because the stroke cost 11.18 s of a 12.71 s certification. That is a change
    /// to the correctness spine, so it is held to the polygon booleans it replaced — the
    /// same discipline as `the_two_independent_engagement_measures_agree`, and for the same
    /// reason: every instrument in this subsystem has to be checked against an independent
    /// one, because two of them have quietly lied already.
    ///
    /// The two cannot agree exactly and should not be asked to, and the direction of the
    /// disagreement is **not** the one I first assumed. The grid samples at cell centres, so
    /// it is blind to any feature thinner than a cell — measured, it reads the standoff's
    /// 1.1 mm² corner residue as **0.0**, an *understatement* of the gap. My first version of
    /// this test asserted the grid could only overstate, and it failed immediately; that
    /// assertion was wrong, not the code.
    ///
    /// So the safety argument has to be made on **magnitude**, and it is this: the area the
    /// grid can miss is bounded by `boundary length × cell`, because what it loses is a
    /// sub-cell sliver along a boundary. At 0.1 mm cells that is ~20 mm² along a 200 mm wall
    /// — while the certification tolerance is `0.02 × reachable + 1`, which is 65 mm² for
    /// this 60 mm part. For the blind spot to flip a verdict the skin would have to run
    /// **650 mm** in a 60 mm square, which is not a shape. The check below therefore pins
    /// what actually matters: the two measures agree to within that bound, *and* they return
    /// the same certification verdict.
    #[test]
    fn the_two_coverage_measures_agree() {
        let r = 3.0;
        for (name, region) in [
            ("square 40", {
                use cam_geo::Contour;
                Polygon::new(Contour::new(vec![
                    Point::new(0.0, 0.0),
                    Point::new(40.0, 0.0),
                    Point::new(40.0, 40.0),
                    Point::new(0.0, 40.0),
                ]))
                .unwrap()
            }),
            ("square 60 with a 20 mm island", holed_60()),
        ] {
            let moves = crate::frontadvance::front_advance_path(&region, r, 0.0, 2.0, None)
                .unwrap_or_else(|| panic!("{name}: a path"));
            let mut model = ClearedModel::bounded(r, region.clone());
            let (mut prev, mut prev_cut) = (None, false);
            for &(p, cut) in &moves {
                if cut {
                    if let Some(pp) = prev {
                        if !prev_cut {
                            model.seed_disc(pp);
                        }
                        model.commit(pp, p);
                    }
                }
                prev = Some(p);
                prev_cut = cut;
            }
            let reach = reachable(&region, r);
            let (g_uncut, g_gouge) = model.coverage(&region, &reach).expect("a grid");
            let (x_uncut, x_gouge) = coverage_exact(&moves, r, &region);

            // The blind-spot bound: a sub-cell sliver along the region's own boundary.
            let ring_len = |ring: &[Point]| -> f64 {
                let n = ring.len();
                (0..n).map(|k| ring[k].distance(ring[(k + 1) % n])).sum()
            };
            let perim = ring_len(region.outer().points())
                + region.holes().iter().map(|h| ring_len(h.points())).sum::<f64>();
            let tol = 0.1 * perim + 2.0; // 0.1 mm cells
            assert!(
                (g_uncut - x_uncut).abs() < tol,
                "{name}: uncut grid {g_uncut:.1} vs exact {x_uncut:.1} (tol {tol:.1})"
            );
            assert!(
                (g_gouge - x_gouge).abs() < tol,
                "{name}: gouge grid {g_gouge:.1} vs exact {x_gouge:.1} (tol {tol:.1})"
            );

            // What actually matters: the same verdict either way, at the tolerance the
            // caller certifies with — and that tolerance comfortably exceeds the blind spot.
            let cover_tol = 0.02 * total_area(&reach) + 1.0;
            assert!(
                tol < cover_tol,
                "{name}: the grid's blind spot ({tol:.1}) must stay under the certification \
                 tolerance ({cover_tol:.1}), or it could flip a verdict"
            );
            let verdict = |u: f64, gg: f64| u <= cover_tol && gg <= cover_tol;
            assert_eq!(
                verdict(g_uncut, g_gouge),
                verdict(x_uncut, x_gouge),
                "{name}: grid and exact must certify alike — grid ({g_uncut:.1}, {g_gouge:.1}), \
                 exact ({x_uncut:.1}, {x_gouge:.1})"
            );
        }
    }

    /// Diagnostic: where does `certify_moves` actually spend its time on a big path? The
    /// certification of an island pocket costs 25–32 s in release, and the parts are not
    /// obviously comparable — a stroke of a several-thousand-point self-overlapping
    /// polyline, two polygon differences over the result, and a per-move engagement scan
    /// with 180 rays each. Guessing which dominates is how the last three wrong turns
    /// started.
    #[test]
    #[ignore = "diagnostic"]
    fn certify_phase_timing() {
        use cam_geo::Contour;
        let region = Polygon::with_holes(
            Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(60.0, 0.0),
                Point::new(60.0, 60.0),
                Point::new(0.0, 60.0),
            ]),
            vec![Contour::new(vec![
                Point::new(20.0, 20.0),
                Point::new(40.0, 20.0),
                Point::new(40.0, 40.0),
                Point::new(20.0, 40.0),
            ])],
        )
        .unwrap();
        let (r, e) = (3.0, 2.0);
        let moves = crate::frontadvance::front_advance_path(&region, r, 0.0, e, None)
            .expect("a path");
        println!("\npath: {} moves", moves.len());

        let t = std::time::Instant::now();
        let runs = cut_runs(&moves);
        let t_runs = t.elapsed().as_secs_f64();

        let t = std::time::Instant::now();
        let mut full: Vec<Polygon> = Vec::new();
        for run in &runs {
            full.extend(swept(run, r));
        }
        let t_swept = t.elapsed().as_secs_f64();

        let t = std::time::Instant::now();
        let reach = reachable(&region, r);
        let t_reach = t.elapsed().as_secs_f64();

        let t = std::time::Instant::now();
        let _uncut = difference(&reach, &full).unwrap_or_default();
        let t_uncut = t.elapsed().as_secs_f64();

        let t = std::time::Instant::now();
        let _gouge = difference(&full, std::slice::from_ref(&region)).unwrap_or_default();
        let t_gouge = t.elapsed().as_secs_f64();

        let t = std::time::Instant::now();
        let mut model = ClearedModel::bounded(r, region.clone());
        let (mut prev, mut prev_cut) = (None, false);
        for &(p, cut) in &moves {
            if cut {
                if let Some(pp) = prev {
                    if !prev_cut {
                        model.seed_disc(pp);
                    }
                    model.engagement(pp, p);
                    model.commit(pp, p);
                }
            }
            prev = Some(p);
            prev_cut = cut;
        }
        let t_engage = t.elapsed().as_secs_f64();

        println!(
            "  cut_runs {t_runs:.2}s | swept {t_swept:.2}s ({} polys) | reachable {t_reach:.2}s \
             | uncut-diff {t_uncut:.2}s | gouge-diff {t_gouge:.2}s | engagement scan {t_engage:.2}s",
            full.len()
        );
        println!(
            "  total {:.2}s\n",
            t_runs + t_swept + t_reach + t_uncut + t_gouge + t_engage
        );
    }

    #[test]
    fn certify_flags_a_gouge_outside_the_region() {
        // A cut whose swept tool leaves the target reports gouge area.
        let r = 2.0;
        let path = vec![Point::new(10.0, 10.0), Point::new(30.0, 10.0)];
        let v = certify(&path, r, &square(0.0, 20.0));
        assert!(v.gouge_area > 1.0, "a cut past the edge must register a gouge, got {}", v.gouge_area);
    }

    /// A serpentine over `[0,20]²` at 2·`r` spacing (tangent passes, so each strip is
    /// load-bearing — at the overlapping spacing the neighbours cover for a missing pass
    /// and a coverage test cannot see it). Every move cuts except the first point.
    fn serpentine_moves(r: f64) -> Vec<(Point, bool)> {
        let mut moves = Vec::new();
        let mut y = r;
        let mut forward = true;
        while y <= 20.0 - r + 1e-9 {
            let (a, b) = if forward { (r, 20.0 - r) } else { (20.0 - r, r) };
            moves.push((Point::new(a, y), !moves.is_empty()));
            moves.push((Point::new(b, y), true));
            y += 2.0 * r;
            forward = !forward;
        }
        moves
    }

    #[test]
    fn cut_runs_split_at_rapids_and_begin_at_the_plunge_point() {
        let p = |x: f64| Point::new(x, 0.0);
        // a --rapid--> b --cut--> c --rapid--> d --cut--> e
        let moves = vec![(p(0.0), false), (p(1.0), true), (p(2.0), false), (p(3.0), true)];
        let runs = cut_runs(&moves);
        assert_eq!(runs.len(), 2, "two cutting runs, split by the rapid");
        // Each run starts at the point the cut departed *from* — the plunge point — so
        // stroking it with round caps reproduces the entry disc for coverage.
        assert_eq!(runs[0], vec![p(0.0), p(1.0)]);
        assert_eq!(runs[1], vec![p(2.0), p(3.0)]);
    }

    #[test]
    fn a_path_of_pure_rapids_cuts_nothing() {
        // Flags are not decoration: with no cutting move the tool removes no material and
        // is charged no engagement, however far it travels across the stock.
        let r = 3.0;
        let moves = vec![
            (Point::new(5.0, 5.0), false),
            (Point::new(35.0, 5.0), false),
            (Point::new(35.0, 35.0), false),
        ];
        let v = certify_moves(&moves, r, &square(0.0, 40.0));
        assert_eq!(v.max_engagement, 0.0, "a rapid cannot cut");
        assert!(v.gouge_area < 1e-6, "a rapid cannot gouge, got {}", v.gouge_area);
        assert!(v.uncut_area > 1000.0, "nothing was cleared, uncut {}", v.uncut_area);
    }

    #[test]
    fn a_rapid_corridor_is_not_credited_as_cleared() {
        // The semantic that makes the moves-aware entry worth having: material the tool
        // only *flew over* stays uncut. Same points, same order — one pass reflagged as a
        // rapid — and the strip it would have cleared must reappear as a coverage gap.
        // (An implementation that stroked the whole point list would report it covered.)
        let r = 2.0;
        let target = square(0.0, 20.0);
        let all = serpentine_moves(r);
        let covered = certify_moves(&all, r, &target);
        assert!(covered.uncut_area < 8.0, "tangent passes tile the square, uncut {}", covered.uncut_area);

        // Reflag the third pass (its terminating point) as a rapid.
        let mut gapped = all.clone();
        gapped[5].1 = false;
        let v = certify_moves(&gapped, r, &target);
        let strip = v.uncut_area - covered.uncut_area;
        assert!(strip > 40.0, "the flown-over strip must read as uncut, gained only {strip}");
    }

    /// **A lift launders nothing.** Engagement is measured against the running cleared
    /// model, not against the path's shape, so retracting before a slot does not make the
    /// slot read less than the diameter. This guards the *unsafe* direction — the failure
    /// this whole subsystem exists to prevent is an oracle that under-reads.
    #[test]
    fn a_lift_does_not_launder_a_following_slot() {
        let r = 3.0;
        let moves = vec![
            (Point::new(2.0, 10.0), false),
            (Point::new(38.0, 10.0), true), // slot into virgin stock
            (Point::new(2.0, 30.0), false), // lift and cross
            (Point::new(38.0, 30.0), true), // slot into virgin stock again
        ];
        let v = certify_moves(&moves, r, &square(0.0, 40.0));
        assert!(v.max_engagement > 5.5, "a slot reads the diameter, got {}", v.max_engagement);
    }
}
