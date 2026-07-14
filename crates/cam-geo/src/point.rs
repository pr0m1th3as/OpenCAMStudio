//! The planar point type shared across the CAM pipeline.

use i_overlay::i_float::float::compatible::FloatPointCompatible;

/// A point in the XY plane, in millimetres.
///
/// This is *our* type — downstream crates use it without pulling in `i_overlay`.
/// It also implements `i_overlay`'s [`FloatPointCompatible`], so slices of
/// `Point` pass straight into the geometry engine with no conversion or copy.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    /// Construct a point from millimetre coordinates.
    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Squared Euclidean distance to `other`. Avoids a `sqrt` when only
    /// comparing distances.
    #[inline]
    pub fn distance_sq(&self, other: Point) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }

    /// Euclidean distance to `other`.
    #[inline]
    pub fn distance(&self, other: Point) -> f64 {
        self.distance_sq(other).sqrt()
    }
}

impl FloatPointCompatible for Point {
    type Scalar = f64;

    #[inline(always)]
    fn from_xy(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    #[inline(always)]
    fn x(&self) -> f64 {
        self.x
    }

    #[inline(always)]
    fn y(&self) -> f64 {
        self.y
    }
}
