//! Closed contours (rings).

use crate::Point;

/// A closed ring of points, implicitly closed (the last vertex connects back to
/// the first — do **not** repeat the start point).
///
/// A contour's [signed area](Contour::signed_area) encodes its orientation:
/// counter-clockwise is positive (a solid boundary), clockwise is negative (a
/// hole), matching the convention used throughout `cam-geo`.
#[derive(Clone, Debug, PartialEq)]
pub struct Contour {
    points: Vec<Point>,
}

impl Contour {
    /// Build a contour from its vertices.
    ///
    /// If the caller repeats the first point at the end (an explicitly closed
    /// ring), the duplicate is dropped so the representation stays canonical.
    pub fn new(mut points: Vec<Point>) -> Self {
        if points.len() >= 2 && points.first() == points.last() {
            points.pop();
        }
        Self { points }
    }

    /// The contour's vertices, in order, without a repeated closing point.
    #[inline]
    pub fn points(&self) -> &[Point] {
        &self.points
    }

    /// Number of vertices.
    #[inline]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Whether the contour has no vertices.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Whether the contour bounds a non-degenerate area (at least three
    /// vertices).
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 3
    }

    /// Signed area via the shoelace formula. Positive for a counter-clockwise
    /// ring, negative for clockwise. The magnitude is the enclosed area in mm².
    pub fn signed_area(&self) -> f64 {
        let pts = &self.points;
        if pts.len() < 3 {
            return 0.0;
        }
        let mut acc = 0.0;
        let mut prev = pts[pts.len() - 1];
        for &cur in pts {
            acc += (prev.x * cur.y) - (cur.x * prev.y);
            prev = cur;
        }
        acc * 0.5
    }

    /// Enclosed area (always non-negative).
    #[inline]
    pub fn area(&self) -> f64 {
        self.signed_area().abs()
    }

    /// Whether the ring winds counter-clockwise (the solid-boundary convention).
    #[inline]
    pub fn is_ccw(&self) -> bool {
        self.signed_area() > 0.0
    }

    /// Reverse the winding direction in place.
    pub fn reverse(&mut self) {
        self.points.reverse();
    }

    /// Return a copy wound counter-clockwise.
    pub fn to_ccw(&self) -> Contour {
        if self.is_ccw() {
            self.clone()
        } else {
            let mut c = self.clone();
            c.reverse();
            c
        }
    }

    /// Return a copy wound clockwise.
    pub fn to_cw(&self) -> Contour {
        if self.is_ccw() {
            let mut c = self.clone();
            c.reverse();
            c
        } else {
            self.clone()
        }
    }

    /// Consume the contour, yielding its point vector.
    pub fn into_points(self) -> Vec<Point> {
        self.points
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_square_ccw() -> Contour {
        Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
        ])
    }

    #[test]
    fn drops_repeated_closing_point() {
        let c = Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 0.0), // explicit close
        ]);
        assert_eq!(c.len(), 3);
    }

    #[test]
    fn signed_area_sign_follows_winding() {
        let ccw = unit_square_ccw();
        assert!(ccw.is_ccw());
        approx(ccw.signed_area(), 1.0);

        let mut cw = ccw.clone();
        cw.reverse();
        assert!(!cw.is_ccw());
        approx(cw.signed_area(), -1.0);
    }

    #[test]
    fn to_ccw_and_to_cw_normalise() {
        let cw = unit_square_ccw().to_cw();
        assert!(!cw.is_ccw());
        assert!(cw.to_ccw().is_ccw());
    }

    fn approx(got: f64, want: f64) {
        assert!((got - want).abs() < 1e-9, "expected {want}, got {got}");
    }
}
