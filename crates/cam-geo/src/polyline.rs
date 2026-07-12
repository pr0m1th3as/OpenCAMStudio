//! Open polylines (unclosed paths).
//!
//! Where a [`Contour`](crate::Contour) is a closed ring bounding an area, a
//! `Polyline` is an *open* chain of points — a tool centreline, an open profile,
//! the result of clipping a path to a region. The last point does **not** join
//! back to the first.

use crate::Point;

/// An open chain of points.
#[derive(Clone, Debug, PartialEq)]
pub struct Polyline {
    points: Vec<Point>,
}

impl Polyline {
    /// Build a polyline from its vertices, in order.
    pub fn new(points: Vec<Point>) -> Self {
        Self { points }
    }

    /// The polyline's vertices, in order.
    #[inline]
    pub fn points(&self) -> &[Point] {
        &self.points
    }

    /// Number of vertices.
    #[inline]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Whether the polyline has no vertices.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Whether the polyline has at least two vertices (so it spans a segment).
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 2
    }

    /// Total path length: the sum of the segment lengths.
    pub fn length(&self) -> f64 {
        self.points.windows(2).map(|w| w[0].distance(w[1])).sum()
    }

    /// Return a copy with the vertex order reversed.
    pub fn reversed(&self) -> Polyline {
        let mut pts = self.points.clone();
        pts.reverse();
        Polyline::new(pts)
    }

    /// Consume the polyline, yielding its point vector.
    pub fn into_points(self) -> Vec<Point> {
        self.points
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn length_sums_segments() {
        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0),
            Point::new(3.0, 0.0),
            Point::new(3.0, 4.0),
        ]);
        assert!((pl.length() - 7.0).abs() < 1e-9);
    }

    #[test]
    fn validity_needs_two_points() {
        assert!(!Polyline::new(vec![Point::new(0.0, 0.0)]).is_valid());
        assert!(Polyline::new(vec![Point::new(0.0, 0.0), Point::new(1.0, 0.0)]).is_valid());
    }
}
