//! A heightfield stock model for 2.5-D material-removal simulation.
//!
//! The stock is a regular XY grid; each cell stores the Z of the top of the
//! remaining material. A flat end mill of radius `r` moving along a segment
//! lowers every cell within `r` of the swept path to the tool's bottom. It is
//! the simplest model that captures 2.5-D removal faithfully — enough to verify
//! that a program clears what it should and never plows a rapid through stock.

use crate::ToolProfile;

/// How the simulated stock compares to a desired target surface — the raw
/// material of gouge / residual verification.
///
/// Signs follow milling intuition: a **gouge** is stock cut *below* the target
/// (material destroyed that should have remained — the dangerous error); a
/// **residual** is stock left *above* the target (uncut material that should have
/// been removed — a quality miss, not a hazard).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SurfaceDiff {
    /// Deepest over-cut below the target, mm (0 if none exceeds tolerance).
    pub max_gouge: f32,
    /// XY of the deepest gouge, if any.
    pub gouge_at: Option<[f64; 2]>,
    /// Simulated Z at the deepest gouge.
    pub gouge_z: f64,
    /// Total volume cut below the target, mm³.
    pub gouge_volume: f64,
    /// Total volume of stock left above the target, mm³.
    pub residual_volume: f64,
    /// Cells whose over-cut exceeds tolerance.
    pub cells_gouged: usize,
    /// Cells whose leftover stock exceeds tolerance.
    pub cells_residual: usize,
}

/// A triangle mesh of the stock surface, for rendering. One vertex per grid cell
/// (at its centre), two triangles per interior quad, wound CCW as seen from `+Z`.
/// Positions are millimetres `(x, y, z)`; normals are unit, `+Z`-ish.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SurfaceMesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

/// A grid of remaining-stock heights.
#[derive(Clone, Debug)]
pub struct Heightfield {
    origin: [f64; 2],
    res: f64,
    nx: usize,
    ny: usize,
    top: f32,
    z: Vec<f32>,
}

impl Heightfield {
    /// A fresh block of stock over `[min, max]` (XY), with `res`-sized cells, all
    /// at height `top`.
    pub fn new(min: [f64; 2], max: [f64; 2], res: f64, top: f64) -> Self {
        let nx = (((max[0] - min[0]) / res).ceil() as usize).max(1);
        let ny = (((max[1] - min[1]) / res).ceil() as usize).max(1);
        Self {
            origin: min,
            res,
            nx,
            ny,
            top: top as f32,
            z: vec![top as f32; nx * ny],
        }
    }

    /// Grid dimensions (columns, rows).
    pub fn dims(&self) -> (usize, usize) {
        (self.nx, self.ny)
    }

    /// Cell size, mm.
    pub fn resolution(&self) -> f64 {
        self.res
    }

    /// The remaining-stock height at `(x, y)` (nearest cell), or the original top
    /// if outside the grid.
    pub fn sample(&self, x: f64, y: f64) -> f32 {
        match self.cell(x, y) {
            Some((ix, iy)) => self.z[iy * self.nx + ix],
            None => self.top,
        }
    }

    /// Lower every cell within `radius` of the segment `a→b` to the (flat) tool
    /// bottom, interpolating the bottom Z along the segment. A convenience for a
    /// flat end mill; [`cut_segment_profile`](Self::cut_segment_profile) handles
    /// shaped tools.
    pub fn cut_segment(&mut self, a: [f64; 3], b: [f64; 3], radius: f64) {
        self.cut_segment_profile(a, b, &ToolProfile::flat(radius));
    }

    /// Lower every cell within the tool's radius of the segment `a→b` to the tool
    /// bottom, accounting for the tool's [`ToolProfile`]: the axis bottom is
    /// interpolated along the segment, and each cell is raised by the profile's
    /// `offset` at its radial distance from the axis (so a ball mill leaves a
    /// rounded floor, a V mill a groove, and a flat mill a flat floor).
    pub fn cut_segment_profile(&mut self, a: [f64; 3], b: [f64; 3], tool: &ToolProfile) {
        let radius = tool.radius;
        let (ix0, ix1) = self.index_range(a[0].min(b[0]) - radius, a[0].max(b[0]) + radius, 0);
        let (iy0, iy1) = self.index_range(a[1].min(b[1]) - radius, a[1].max(b[1]) + radius, 1);
        let r2 = radius * radius;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let (cx, cy) = self.center(ix, iy);
                let (t, dist2) = project(cx, cy, [a[0], a[1]], [b[0], b[1]]);
                if dist2 <= r2 {
                    let axis_z = a[2] + (b[2] - a[2]) * t;
                    let bottom = (axis_z + tool.offset(dist2.sqrt())) as f32;
                    let cell = &mut self.z[iy * self.nx + ix];
                    *cell = cell.min(bottom);
                }
            }
        }
    }

    /// The greatest remaining-stock height within `radius` of the swept XY path
    /// `a→b` — used to detect a rapid plowing through stock. Returns `f32::MIN`
    /// if the path covers no cells.
    pub fn max_height_along(&self, a: [f64; 2], b: [f64; 2], radius: f64) -> f32 {
        let (ix0, ix1) = self.index_range(a[0].min(b[0]) - radius, a[0].max(b[0]) + radius, 0);
        let (iy0, iy1) = self.index_range(a[1].min(b[1]) - radius, a[1].max(b[1]) + radius, 1);
        let r2 = radius * radius;
        let mut max = f32::MIN;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let (cx, cy) = self.center(ix, iy);
                if project(cx, cy, a, b).1 <= r2 {
                    max = max.max(self.z[iy * self.nx + ix]);
                }
            }
        }
        max
    }

    /// Volume of material removed so far, mm³.
    pub fn removed_volume(&self) -> f64 {
        let cell = self.res * self.res;
        self.z
            .iter()
            .map(|&z| (self.top - z).max(0.0) as f64 * cell)
            .sum()
    }

    /// Lower every cell whose centre lies in the XY rectangle `[min, max]` to
    /// `z`, never raising — a primitive for building a target surface (e.g. a
    /// pocket floor) or pre-shaping stock.
    pub fn lower_rect(&mut self, min: [f64; 2], max: [f64; 2], z: f64) {
        let (ix0, ix1) = self.index_range(min[0], max[0], 0);
        let (iy0, iy1) = self.index_range(min[1], max[1], 1);
        let z = z as f32;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let (cx, cy) = self.center(ix, iy);
                if cx >= min[0] && cx <= max[0] && cy >= min[1] && cy <= max[1] {
                    let cell = &mut self.z[iy * self.nx + ix];
                    *cell = cell.min(z);
                }
            }
        }
    }

    /// Compare this (simulated) field against a `target` surface, cell by cell.
    /// `target` is sampled at each of this field's cell centres, so the two grids
    /// need not align. `tol` (mm) is the deviation ignored as grazing.
    pub fn compare(&self, target: &Heightfield, tol: f64) -> SurfaceDiff {
        let tol = tol as f32;
        let cell_area = self.res * self.res;
        let mut diff = SurfaceDiff::default();
        let mut worst = 0.0f32;
        for iy in 0..self.ny {
            for ix in 0..self.nx {
                let actual = self.z[iy * self.nx + ix];
                let (cx, cy) = self.center(ix, iy);
                let over = target.sample(cx, cy) - actual; // >0 ⇒ cut below target
                if over > tol {
                    diff.gouge_volume += over as f64 * cell_area;
                    diff.cells_gouged += 1;
                    if over > worst {
                        worst = over;
                        diff.gouge_at = Some([cx, cy]);
                        diff.gouge_z = actual as f64;
                    }
                } else if -over > tol {
                    diff.residual_volume += (-over) as f64 * cell_area;
                    diff.cells_residual += 1;
                }
            }
        }
        diff.max_gouge = worst;
        diff
    }

    /// Triangulate the current surface into a [`SurfaceMesh`] for rendering.
    /// Per-vertex normals come from central differences of the height grid.
    pub fn to_mesh(&self) -> SurfaceMesh {
        let mut positions = Vec::with_capacity(self.nx * self.ny);
        for iy in 0..self.ny {
            for ix in 0..self.nx {
                let (cx, cy) = self.center(ix, iy);
                positions.push([cx as f32, cy as f32, self.z[iy * self.nx + ix]]);
            }
        }

        let res = self.res as f32;
        let mut normals = Vec::with_capacity(self.nx * self.ny);
        for iy in 0..self.ny {
            for ix in 0..self.nx {
                let xm = ix.saturating_sub(1);
                let xp = (ix + 1).min(self.nx - 1);
                let ym = iy.saturating_sub(1);
                let yp = (iy + 1).min(self.ny - 1);
                let dzdx = (self.z[iy * self.nx + xp] - self.z[iy * self.nx + xm])
                    / ((xp - xm).max(1) as f32 * res);
                let dzdy = (self.z[yp * self.nx + ix] - self.z[ym * self.nx + ix])
                    / ((yp - ym).max(1) as f32 * res);
                let n = [-dzdx, -dzdy, 1.0];
                let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                normals.push([n[0] / len, n[1] / len, n[2] / len]);
            }
        }

        let mut indices = Vec::new();
        for iy in 0..self.ny.saturating_sub(1) {
            for ix in 0..self.nx.saturating_sub(1) {
                let i = (iy * self.nx + ix) as u32;
                let right = i + 1;
                let down = i + self.nx as u32;
                let down_right = down + 1;
                // Two CCW triangles as seen from +Z (front-facing looking down).
                indices.extend_from_slice(&[i, right, down_right]);
                indices.extend_from_slice(&[i, down_right, down]);
            }
        }

        SurfaceMesh {
            positions,
            normals,
            indices,
        }
    }

    /// Centre of cell `(ix, iy)`.
    fn center(&self, ix: usize, iy: usize) -> (f64, f64) {
        (
            self.origin[0] + (ix as f64 + 0.5) * self.res,
            self.origin[1] + (iy as f64 + 0.5) * self.res,
        )
    }

    /// Cell index containing `(x, y)`, if inside the grid.
    fn cell(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let ix = ((x - self.origin[0]) / self.res).floor();
        let iy = ((y - self.origin[1]) / self.res).floor();
        if ix < 0.0 || iy < 0.0 || ix >= self.nx as f64 || iy >= self.ny as f64 {
            None
        } else {
            Some((ix as usize, iy as usize))
        }
    }

    /// Clamped inclusive cell-index range covering `[lo, hi]` on axis `axis`.
    fn index_range(&self, lo: f64, hi: f64, axis: usize) -> (usize, usize) {
        let n = if axis == 0 { self.nx } else { self.ny };
        let i0 =
            (((lo - self.origin[axis]) / self.res).floor()).clamp(0.0, (n - 1) as f64) as usize;
        let i1 =
            (((hi - self.origin[axis]) / self.res).floor()).clamp(0.0, (n - 1) as f64) as usize;
        (i0, i1)
    }
}

/// Project `(px, py)` onto segment `a→b`, returning the clamped parameter `t`
/// and the squared distance to the segment.
fn project(px: f64, py: f64, a: [f64; 2], b: [f64; 2]) -> (f64, f64) {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 < 1e-12 {
        0.0
    } else {
        (((px - a[0]) * dx + (py - a[1]) * dy) / len2).clamp(0.0, 1.0)
    };
    let (qx, qy) = (a[0] + dx * t, a[1] + dy * t);
    let d2 = (px - qx) * (px - qx) + (py - qy) * (py - qy);
    (t, d2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_straight_cut_lowers_a_band_to_depth() {
        let mut hf = Heightfield::new([0.0, 0.0], [20.0, 20.0], 0.5, 0.0);
        // Cut along y=10 from x=2 to x=18 at Z=-3 with a 2 mm-radius tool.
        hf.cut_segment([2.0, 10.0, -3.0], [18.0, 10.0, -3.0], 2.0);
        assert!(
            (hf.sample(10.0, 10.0) + 3.0).abs() < 1e-6,
            "on the cut ⇒ -3"
        );
        assert!(
            (hf.sample(10.0, 10.0) - hf.sample(10.0, 11.5)).abs() < 1e-6,
            "within radius ⇒ cut"
        );
        assert!(
            (hf.sample(10.0, 15.0) - 0.0).abs() < 1e-6,
            "outside radius ⇒ untouched"
        );
    }

    #[test]
    fn max_height_sees_uncut_stock() {
        let mut hf = Heightfield::new([0.0, 0.0], [20.0, 20.0], 0.5, 0.0);
        hf.cut_segment([0.0, 5.0, -2.0], [20.0, 5.0, -2.0], 2.0);
        // A path over the cut band is clear; a path over uncut stock is not.
        assert!(hf.max_height_along([0.0, 5.0], [20.0, 5.0], 1.0) < -1.9);
        assert!(hf.max_height_along([0.0, 15.0], [20.0, 15.0], 1.0) > -0.001);
    }

    #[test]
    fn lower_rect_carves_only_inside_the_rectangle() {
        let mut hf = Heightfield::new([0.0, 0.0], [20.0, 20.0], 0.5, 0.0);
        hf.lower_rect([5.0, 5.0], [15.0, 15.0], -3.0);
        assert!((hf.sample(10.0, 10.0) + 3.0).abs() < 1e-6, "inside ⇒ -3");
        assert!(
            (hf.sample(2.0, 2.0) - 0.0).abs() < 1e-6,
            "outside ⇒ untouched"
        );
        // Never raises: a deeper existing cut survives a shallower lower_rect.
        hf.cut_segment([10.0, 10.0, -6.0], [10.0, 10.0, -6.0], 1.0);
        hf.lower_rect([5.0, 5.0], [15.0, 15.0], -3.0);
        assert!(hf.sample(10.0, 10.0) < -5.9, "deeper cut preserved");
    }

    #[test]
    fn compare_reports_gouge_and_residual() {
        // Simulated: a flat -4 floor. Target: floor at -2 inside a rectangle, top
        // (0) outside. Inside ⇒ gouge (cut 2 mm too deep); outside ⇒ residual
        // (2 mm of stock left standing above the target's -2... no — outside the
        // rect the target is the original top 0, and actual is -4, so it's still
        // a gouge). Use a target lowered everywhere except a raised pad.
        let mut actual = Heightfield::new([0.0, 0.0], [20.0, 20.0], 0.5, 0.0);
        actual.lower_rect([0.0, 0.0], [20.0, 20.0], -4.0); // cut flat to -4

        let mut target = Heightfield::new([0.0, 0.0], [20.0, 20.0], 0.5, 0.0);
        target.lower_rect([0.0, 0.0], [20.0, 20.0], -2.0); // wanted a -2 floor…
        target.lower_rect([5.0, 5.0], [15.0, 15.0], -6.0); // …with a deep pocket

        let diff = actual.compare(&target, 0.05);
        // Outside the pocket: actual -4 vs target -2 ⇒ 2 mm gouge.
        assert!(
            (diff.max_gouge - 2.0).abs() < 1e-3,
            "max gouge {}",
            diff.max_gouge
        );
        assert!(diff.gouge_volume > 0.0 && diff.cells_gouged > 0);
        // Inside the pocket: actual -4 vs target -6 ⇒ 2 mm of stock left = residual.
        assert!(diff.residual_volume > 0.0 && diff.cells_residual > 0);
    }

    #[test]
    fn to_mesh_has_a_vertex_per_cell_and_two_triangles_per_quad() {
        let hf = Heightfield::new([0.0, 0.0], [10.0, 10.0], 1.0, -1.0);
        let (nx, ny) = hf.dims();
        let mesh = hf.to_mesh();
        assert_eq!(mesh.positions.len(), nx * ny);
        assert_eq!(mesh.normals.len(), nx * ny);
        assert_eq!(mesh.indices.len(), (nx - 1) * (ny - 1) * 6);
        // A flat field ⇒ every normal points straight up.
        for n in &mesh.normals {
            assert!((n[2] - 1.0).abs() < 1e-6, "flat ⇒ +Z normal, got {n:?}");
        }
        // Indices stay in bounds.
        assert!(mesh
            .indices
            .iter()
            .all(|&i| (i as usize) < mesh.positions.len()));
    }
}
