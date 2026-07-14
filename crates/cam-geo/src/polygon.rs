//! Filled planar regions: an outer boundary with optional holes.

use crate::{Contour, GeoError, Point, GRID_SCALE};

/// Where a point sits relative to a filled region. See [`Polygon::locate`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Containment {
    /// Strictly inside the region (inside the outer boundary, outside all holes).
    Inside,
    /// Strictly outside the region.
    Outside,
    /// Exactly on a boundary edge (of the outer boundary or a hole).
    OnBoundary,
}

/// A filled region of the plane: one outer boundary plus zero or more holes.
///
/// By convention the outer boundary winds counter-clockwise and holes wind
/// clockwise; [`Polygon::new`] / [`Polygon::with_holes`] normalise orientation
/// so callers need not. Operations that produce polygons ([`crate::offset`],
/// [`crate::union`], …) return them already normalised.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Polygon {
    outer: Contour,
    holes: Vec<Contour>,
}

impl Polygon {
    /// Build a hole-free region from an outer boundary.
    ///
    /// Returns [`GeoError::DegenerateContour`] if the boundary has fewer than
    /// three vertices.
    pub fn new(outer: Contour) -> Result<Self, GeoError> {
        Self::with_holes(outer, Vec::new())
    }

    /// Build a region from an outer boundary and a set of holes.
    ///
    /// The outer boundary is normalised counter-clockwise and each hole
    /// clockwise. Degenerate holes (fewer than three vertices) are rejected.
    pub fn with_holes(outer: Contour, holes: Vec<Contour>) -> Result<Self, GeoError> {
        if !outer.is_valid() {
            return Err(GeoError::DegenerateContour);
        }
        let outer = outer.to_ccw();
        let mut norm_holes = Vec::with_capacity(holes.len());
        for h in holes {
            if !h.is_valid() {
                return Err(GeoError::DegenerateContour);
            }
            norm_holes.push(h.to_cw());
        }
        Ok(Self {
            outer,
            holes: norm_holes,
        })
    }

    /// The outer boundary (counter-clockwise).
    #[inline]
    pub fn outer(&self) -> &Contour {
        &self.outer
    }

    /// The holes (each clockwise).
    #[inline]
    pub fn holes(&self) -> &[Contour] {
        &self.holes
    }

    /// Net enclosed area: outer area minus the area of every hole.
    pub fn area(&self) -> f64 {
        let holes: f64 = self.holes.iter().map(Contour::area).sum();
        self.outer.area() - holes
    }

    /// Classify a point against the filled region: strictly inside, strictly
    /// outside, or exactly on a boundary edge.
    ///
    /// The test runs on the fixed integer grid, so it is robust to
    /// floating-point round-off and fully deterministic — including the
    /// boundary case, which a bare winding number leaves ambiguous. A point is
    /// [`Containment::OnBoundary`] when, once snapped to the grid, it lies on any
    /// edge of the outer boundary or a hole; otherwise a winding-number test
    /// decides inside vs. outside (outer winds CCW at `+1`, holes CW at `-1`, so
    /// a point within a hole nets to zero).
    pub fn locate(&self, p: Point) -> Containment {
        let px = snap(p.x);
        let py = snap(p.y);

        if point_on_ring(self.outer.points(), px, py)
            || self.holes.iter().any(|h| point_on_ring(h.points(), px, py))
        {
            return Containment::OnBoundary;
        }

        let mut wn = winding(self.outer.points(), px, py);
        for h in &self.holes {
            wn += winding(h.points(), px, py);
        }
        if wn != 0 {
            Containment::Inside
        } else {
            Containment::Outside
        }
    }

    /// Whether `p` lies within the filled region, treating the boundary as part
    /// of the region (the closed-set convention): `true` for interior and
    /// on-edge points, `false` only strictly outside. Use [`Polygon::locate`]
    /// when the boundary case must be distinguished.
    pub fn contains(&self, p: Point) -> bool {
        self.locate(p) != Containment::Outside
    }

    /// Convert to an `i_overlay` "shape" (outer contour first, then holes), with
    /// orientation already normalised by construction.
    pub(crate) fn to_shape(&self) -> Vec<Vec<Point>> {
        let mut shape = Vec::with_capacity(1 + self.holes.len());
        shape.push(self.outer.points().to_vec());
        for h in &self.holes {
            shape.push(h.points().to_vec());
        }
        shape
    }

    /// Rebuild a polygon from an `i_overlay` shape: contour 0 is the outer
    /// boundary, the rest are holes.
    pub(crate) fn from_shape(shape: Vec<Vec<Point>>) -> Option<Polygon> {
        let mut rings = shape.into_iter();
        let outer = Contour::new(rings.next()?);
        if !outer.is_valid() {
            return None;
        }
        let holes = rings.map(Contour::new).filter(Contour::is_valid).collect();
        // Orientation is already CCW-outer / CW-holes from the engine, but
        // normalise defensively so the invariant always holds.
        Polygon::with_holes(outer, holes).ok()
    }
}

/// Collect a slice of polygons into an `i_overlay` "shapes" resource.
pub(crate) fn to_shapes(polygons: &[Polygon]) -> Vec<Vec<Vec<Point>>> {
    polygons.iter().map(Polygon::to_shape).collect()
}

/// Rebuild polygons from an `i_overlay` shapes result.
pub(crate) fn from_shapes(shapes: Vec<Vec<Vec<Point>>>) -> Vec<Polygon> {
    shapes.into_iter().filter_map(Polygon::from_shape).collect()
}

/// Snap a millimetre coordinate onto the integer grid.
#[inline]
fn snap(v: f64) -> i64 {
    (v * GRID_SCALE).round() as i64
}

/// Winding-number contribution of a single ring around the grid point
/// `(px, py)`, computed with exact integer arithmetic (Sunday's algorithm).
fn winding(pts: &[Point], px: i64, py: i64) -> i32 {
    if pts.len() < 3 {
        return 0;
    }
    let mut wn = 0i32;
    let mut a = pts[pts.len() - 1];
    for &b in pts {
        let (ax, ay) = (snap(a.x), snap(a.y));
        let (bx, by) = (snap(b.x), snap(b.y));
        if ay <= py {
            if by > py && is_left(ax, ay, bx, by, px, py) > 0 {
                wn += 1;
            }
        } else if by <= py && is_left(ax, ay, bx, by, px, py) < 0 {
            wn -= 1;
        }
        a = b;
    }
    wn
}

/// Sign of the cross product `(b - a) × (p - a)`: >0 if `p` is left of the
/// directed edge `a→b`, <0 if right, 0 if collinear. Widened to `i128` so the
/// products cannot overflow at grid scale.
#[inline]
fn is_left(ax: i64, ay: i64, bx: i64, by: i64, px: i64, py: i64) -> i128 {
    let abx = (bx - ax) as i128;
    let aby = (by - ay) as i128;
    let apx = (px - ax) as i128;
    let apy = (py - ay) as i128;
    abx * apy - aby * apx
}

/// Whether the grid point `(px, py)` lies exactly on any edge of the ring.
fn point_on_ring(pts: &[Point], px: i64, py: i64) -> bool {
    if pts.len() < 2 {
        return false;
    }
    let mut a = pts[pts.len() - 1];
    for &b in pts {
        if on_segment(snap(a.x), snap(a.y), snap(b.x), snap(b.y), px, py) {
            return true;
        }
        a = b;
    }
    false
}

/// Whether the grid point `(px, py)` lies on the closed segment `a→b`: collinear
/// (exact zero cross product) and within the segment's bounding box.
#[inline]
fn on_segment(ax: i64, ay: i64, bx: i64, by: i64, px: i64, py: i64) -> bool {
    if is_left(ax, ay, bx, by, px, py) != 0 {
        return false;
    }
    px >= ax.min(bx) && px <= ax.max(bx) && py >= ay.min(by) && py <= ay.max(by)
}
