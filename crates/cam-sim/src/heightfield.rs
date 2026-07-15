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

/// Unit normal of triangle `a→b→c` from its winding (CCW ⇒ toward the viewer).
fn tri_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-9);
    [n[0] / len, n[1] / len, n[2] / len]
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

    /// Triangulate the stock as a **watertight solid** down to `floor` (the stock
    /// bottom, mm): sloped per-cell tops, a bottom face, and vertical walls **only
    /// at steep steps** and the perimeter. Unlike [`to_mesh`](Self::to_mesh) — a
    /// smooth sheet with no thickness — the block shows its full depth; and unlike
    /// a pure voxel block it keeps gentle slopes smooth, so a chamfer bevel is a
    /// clean ramp while a pocket wall stays crisply vertical. A cell cut to or
    /// below `floor` is left open, so a through feature is see-through.
    ///
    /// Top/bottom faces carry geometric (per-triangle) normals from their
    /// CCW-outward winding. **Wall** normals, though, come from the local *boundary
    /// gradient* — the summed direction to the cell's open neighbours — so a curved
    /// wall (a round hole) shades as a smooth cylinder rather than a granular
    /// stack of axis-aligned facets, while a straight wall's gradient is already
    /// axis-aligned and looks unchanged. This smooths the *shading*; the stepped
    /// silhouette is a heightfield limit deferred to the 3D-milling/kernel work.
    pub fn to_solid_mesh(&self, floor: f64) -> SurfaceMesh {
        let f = floor as f32;
        let res = self.res as f32;
        let (ox, oy) = (self.origin[0] as f32, self.origin[1] as f32);
        // A neighbour drop steeper than this reads as a wall (rendered vertical);
        // gentler steps are a slope the tops ramp across — so a ≤45° chamfer
        // (≈`res` drop per cell) stays smooth while pocket walls stay vertical.
        let wall_step = 2.5 * res;

        let mut positions: Vec<[f32; 3]> = Vec::new();
        let mut normals: Vec<[f32; 3]> = Vec::new();
        let mut indices: Vec<u32> = Vec::new();
        // One triangle with an explicit per-vertex normal.
        let mut tri = |a: [f32; 3], b: [f32; 3], c: [f32; 3], n: [f32; 3]| {
            let base = positions.len() as u32;
            positions.extend_from_slice(&[a, b, c]);
            normals.extend_from_slice(&[n; 3]);
            indices.extend_from_slice(&[base, base + 1, base + 2]);
        };
        // Cell height, or `None` off-grid.
        let h = |ix: i64, iy: i64| -> Option<f32> {
            if ix < 0 || iy < 0 || ix as usize >= self.nx || iy as usize >= self.ny {
                None
            } else {
                Some(self.z[iy as usize * self.nx + ix as usize])
            }
        };
        // Height of a cell corner as seen by a cell at `z0`: average `z0` with the
        // (up to 3) other cells meeting at that corner that are within `wall_step`
        // (gradual). Gentle slopes thus share corner heights (continuous ramp);
        // across a wall the steep neighbour is excluded, so the corner stays at
        // this cell's level and a vertical wall is drawn there instead.
        let corner = |z0: f32, around: [Option<f32>; 3]| -> f32 {
            let (mut sum, mut n) = (z0, 1.0f32);
            for c in around.into_iter().flatten() {
                if (c - z0).abs() <= wall_step {
                    sum += c;
                    n += 1.0;
                }
            }
            sum / n
        };

        for iy in 0..self.ny {
            for ix in 0..self.nx {
                let z = self.z[iy * self.nx + ix];
                if z <= f + 1e-4 {
                    continue; // cut through to the floor ⇒ open (see-through)
                }
                let (i, j) = (ix as i64, iy as i64);
                let (e, w, n_, s) = (h(i + 1, j), h(i - 1, j), h(i, j + 1), h(i, j - 1));
                let (ne, nw) = (h(i + 1, j + 1), h(i - 1, j + 1));
                let (se, sw) = (h(i + 1, j - 1), h(i - 1, j - 1));
                // Corner heights (SW, SE, NE, NW of the cell footprint).
                let c_sw = corner(z, [w, s, sw]);
                let c_se = corner(z, [e, s, se]);
                let c_ne = corner(z, [e, n_, ne]);
                let c_nw = corner(z, [w, n_, nw]);
                let (x0, y0) = (ox + ix as f32 * res, oy + iy as f32 * res);
                let (x1, y1) = (x0 + res, y0 + res);
                // Sloped top — geometric normal per triangle (a planar bevel shades
                // uniformly; a flat top is +Z).
                let (p_sw, p_se, p_ne, p_nw) = (
                    [x0, y0, c_sw],
                    [x1, y0, c_se],
                    [x1, y1, c_ne],
                    [x0, y1, c_nw],
                );
                tri(p_sw, p_se, p_ne, tri_normal(p_sw, p_se, p_ne));
                tri(p_sw, p_ne, p_nw, tri_normal(p_sw, p_ne, p_nw));
                // Flat bottom (−Z).
                let down = [0.0, 0.0, -1.0];
                tri([x0, y0, f], [x0, y1, f], [x1, y1, f], down);
                tri([x0, y0, f], [x1, y1, f], [x1, y0, f], down);

                // Is a neighbour "open" (off-grid / cut through / a steep drop)?
                let open = |nb: Option<f32>| match nb {
                    None => true,
                    Some(zn) => zn <= f + 1e-4 || z - zn > wall_step,
                };
                // Wall shading normal: sum the (unit) directions to every open
                // neighbour in the 8-ring → points into the void, radial around a
                // hole. Falls back to the geometric wall direction if it cancels.
                let mut g = [0.0f32, 0.0];
                for (dx, dy, nb) in [
                    (1, 0, e), (-1, 0, w), (0, 1, n_), (0, -1, s),
                    (1, 1, ne), (-1, 1, nw), (1, -1, se), (-1, -1, sw),
                ] {
                    if open(nb) {
                        let l = ((dx * dx + dy * dy) as f32).sqrt();
                        g[0] += dx as f32 / l;
                        g[1] += dy as f32 / l;
                    }
                }
                let glen = (g[0] * g[0] + g[1] * g[1]).sqrt();
                let grad = (glen > 1e-3).then(|| [g[0] / glen, g[1] / glen, 0.0]);

                // A vertical wall on a side only where that neighbour is open — from
                // this cell's two edge corners down to the neighbour level (clamped
                // to the floor). Shaded by the gradient normal, or the face axis.
                let level = |nb: Option<f32>| nb.map_or(f, |v| v.max(f));
                let mut wall = |a: [f32; 3], b: [f32; 3], c: [f32; 3], d: [f32; 3], axis: [f32; 3]| {
                    let n = grad.unwrap_or(axis);
                    tri(a, b, c, n);
                    tri(a, c, d, n);
                };
                if open(w) {
                    let l = level(w);
                    wall([x0, y0, c_sw], [x0, y1, c_nw], [x0, y1, l], [x0, y0, l], [-1.0, 0.0, 0.0]);
                }
                if open(e) {
                    let l = level(e);
                    wall([x1, y1, c_ne], [x1, y0, c_se], [x1, y0, l], [x1, y1, l], [1.0, 0.0, 0.0]);
                }
                if open(s) {
                    let l = level(s);
                    wall([x1, y0, c_se], [x0, y0, c_sw], [x0, y0, l], [x1, y0, l], [0.0, -1.0, 0.0]);
                }
                if open(n_) {
                    let l = level(n_);
                    wall([x0, y1, c_nw], [x1, y1, c_ne], [x1, y1, l], [x0, y1, l], [0.0, 1.0, 0.0]);
                }
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

    #[test]
    fn solid_mesh_is_a_closed_block_with_walls_and_a_floor() {
        // Uncut 10×10 block, top 0, floor -5.
        let hf = Heightfield::new([0.0, 0.0], [10.0, 10.0], 1.0, 0.0);
        let mesh = hf.to_solid_mesh(-5.0);
        assert!(mesh.indices.iter().all(|&i| (i as usize) < mesh.positions.len()));
        // Must carry top (+Z), bottom (−Z) and side-wall (horizontal) faces — a
        // sheet would have only +Z.
        let has = |pred: fn(&[f32; 3]) -> bool| mesh.normals.iter().any(pred);
        assert!(has(|n| n[2] > 0.9), "has a top");
        assert!(has(|n| n[2] < -0.9), "has a bottom");
        assert!(has(|n| n[0].abs() > 0.9 || n[1].abs() > 0.9), "has perimeter walls");
        // No geometry ever dips below the floor.
        assert!(mesh.positions.iter().all(|p| p[2] >= -5.0 - 1e-4));
    }

    #[test]
    fn gentle_slopes_stay_smooth_while_deep_steps_get_walls() {
        // The chamfer-staircase fix: a shallow step (gentle vs the cell size) ramps
        // smoothly, adding no vertical wall; a deep step keeps a crisp wall.
        let floor = -5.0;
        let wall_faces = |hf: &Heightfield| {
            hf.to_solid_mesh(floor)
                .normals
                .iter()
                .filter(|n| n[2].abs() < 0.1) // near-horizontal normal ⇒ vertical face
                .count()
        };
        let mut shallow = Heightfield::new([0.0, 0.0], [20.0, 20.0], 0.5, 0.0);
        shallow.lower_rect([5.0, 5.0], [15.0, 15.0], -0.5);
        let mut deep = Heightfield::new([0.0, 0.0], [20.0, 20.0], 0.5, 0.0);
        deep.lower_rect([5.0, 5.0], [15.0, 15.0], -3.0);
        assert!(
            wall_faces(&deep) > wall_faces(&shallow),
            "deep {} vs shallow {}: a deep pocket walls its edge; a shallow one ramps",
            wall_faces(&deep),
            wall_faces(&shallow)
        );
    }

    #[test]
    fn curved_walls_shade_radially_not_just_axis_aligned() {
        // A round hole's walls take gradient (radial) normals, so some wall face
        // points diagonally — impossible for pure ±X/±Y voxel walls.
        let mut hf = Heightfield::new([0.0, 0.0], [20.0, 20.0], 0.5, 0.0);
        // A cylindrical hole (radius 4) cut through the floor at the centre.
        hf.cut_segment([9.99, 10.0, -6.0], [10.01, 10.0, -6.0], 4.0);
        let mesh = hf.to_solid_mesh(-5.0);
        let diagonal = mesh
            .normals
            .iter()
            .any(|n| n[2].abs() < 0.2 && n[0].abs() > 0.3 && n[1].abs() > 0.3);
        assert!(
            diagonal,
            "round-hole walls should shade radially (a diagonal normal), not only ±X/±Y"
        );
    }

    #[test]
    fn solid_mesh_leaves_through_cuts_open() {
        // A cell cut clean through (below the floor) contributes no top face, so a
        // through feature reads as see-through rather than a filled recess.
        let floor = -5.0;
        let tops = |hf: &Heightfield| {
            hf.to_solid_mesh(floor)
                .normals
                .iter()
                .filter(|n| n[2] > 0.9)
                .count()
        };
        let mut hf = Heightfield::new([0.0, 0.0], [3.0, 3.0], 1.0, 0.0);
        let before = tops(&hf);
        // Cut the centre cell past the floor.
        hf.cut_segment([1.3, 1.5, -6.0], [1.7, 1.5, -6.0], 0.3);
        assert!(hf.sample(1.5, 1.5) <= -5.9, "centre cell cut through");
        assert!(tops(&hf) < before, "the through-cut cell has no top ⇒ open hole");
    }
}
