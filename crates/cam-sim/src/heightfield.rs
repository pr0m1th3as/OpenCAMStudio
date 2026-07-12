//! A heightfield stock model for 2.5-D material-removal simulation.
//!
//! The stock is a regular XY grid; each cell stores the Z of the top of the
//! remaining material. A flat end mill of radius `r` moving along a segment
//! lowers every cell within `r` of the swept path to the tool's bottom. It is
//! the simplest model that captures 2.5-D removal faithfully — enough to verify
//! that a program clears what it should and never plows a rapid through stock.

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

    /// Lower every cell within `radius` of the segment `a→b` to the tool bottom,
    /// interpolating the bottom Z along the segment.
    pub fn cut_segment(&mut self, a: [f64; 3], b: [f64; 3], radius: f64) {
        let (ix0, ix1) = self.index_range(a[0].min(b[0]) - radius, a[0].max(b[0]) + radius, 0);
        let (iy0, iy1) = self.index_range(a[1].min(b[1]) - radius, a[1].max(b[1]) + radius, 1);
        let r2 = radius * radius;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let (cx, cy) = self.center(ix, iy);
                let (t, dist2) = project(cx, cy, [a[0], a[1]], [b[0], b[1]]);
                if dist2 <= r2 {
                    let bottom = (a[2] + (b[2] - a[2]) * t) as f32;
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
}
