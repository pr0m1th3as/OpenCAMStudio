//! Arc flattening — turning circular arcs into polylines within a chord
//! tolerance.
//!
//! CAM geometry is ultimately polygonal, but arcs arise everywhere: DXF
//! `ARC`/`CIRCLE` entities, filleted corners, tool-radius arcs. Flattening
//! approximates an arc by a chain of segments whose maximum deviation from the
//! true arc (the *chord tolerance*) is bounded.

use core::f64::consts::PI;

use crate::Point;

/// A circular arc, swept from `start_angle` to `end_angle` about `center`.
///
/// Angles are in radians, measured counter-clockwise from the +X axis. The
/// `ccw` flag selects the sweep direction; if the start and end angles are
/// equal the arc is treated as a full circle in that direction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Arc {
    pub center: Point,
    pub radius: f64,
    pub start_angle: f64,
    pub end_angle: f64,
    pub ccw: bool,
}

impl Arc {
    /// Construct an arc.
    pub fn new(center: Point, radius: f64, start_angle: f64, end_angle: f64, ccw: bool) -> Self {
        Self {
            center,
            radius,
            start_angle,
            end_angle,
            ccw,
        }
    }

    /// A full circle, swept counter-clockwise.
    pub fn circle(center: Point, radius: f64) -> Self {
        Self::new(center, radius, 0.0, 2.0 * PI, true)
    }

    /// The signed angular sweep (radians): positive counter-clockwise, negative
    /// clockwise. Equal start/end angles give a full ±2π turn.
    pub fn sweep(&self) -> f64 {
        let raw = self.end_angle - self.start_angle;
        if self.ccw {
            let mut s = raw.rem_euclid(2.0 * PI);
            if s <= 0.0 {
                s += 2.0 * PI;
            }
            s
        } else {
            let mut s = raw.rem_euclid(2.0 * PI);
            if s >= 0.0 {
                s -= 2.0 * PI;
            }
            s
        }
    }

    /// Flatten the arc into a polyline whose deviation from the true arc is at
    /// most `chord_tol` millimetres.
    ///
    /// The returned points include both endpoints, so a flattened arc always has
    /// at least two points (a degenerate zero-radius arc collapses to a single
    /// point). Feed `chord_tol` in the same units as the geometry (mm).
    pub fn flatten(&self, chord_tol: f64) -> Vec<Point> {
        let r = self.radius;
        if r <= 0.0 {
            return vec![self.center];
        }
        let sweep = self.sweep();
        let n = segment_count(r, sweep.abs(), chord_tol);
        let step = sweep / n as f64;
        let mut pts = Vec::with_capacity(n + 1);
        for i in 0..=n {
            let a = self.start_angle + step * i as f64;
            pts.push(Point::new(
                self.center.x + r * a.cos(),
                self.center.y + r * a.sin(),
            ));
        }
        pts
    }
}

/// Number of segments needed to hold an arc of `radius` and angular span
/// `sweep_abs` (radians, non-negative) within `chord_tol`.
fn segment_count(radius: f64, sweep_abs: f64, chord_tol: f64) -> usize {
    if sweep_abs <= 0.0 {
        return 1;
    }
    // Max angular step whose chord deviates from the arc by at most chord_tol:
    // deviation = r · (1 − cos(dθ/2))  ⇒  dθ = 2·acos(1 − tol/r).
    let max_step = if chord_tol > 0.0 && chord_tol < radius {
        2.0 * (1.0 - chord_tol / radius).acos()
    } else {
        // Tolerance meaningless or ≥ radius: fall back to coarse quarter-turns.
        PI / 2.0
    };
    (sweep_abs / max_step).ceil().max(1.0) as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Maximum deviation of a flattened polyline from the true circle.
    fn max_chord_error(arc: &Arc, pts: &[Point]) -> f64 {
        let mut worst: f64 = 0.0;
        for w in pts.windows(2) {
            let mid = Point::new((w[0].x + w[1].x) * 0.5, (w[0].y + w[1].y) * 0.5);
            let d = arc.center.distance(mid);
            worst = worst.max((arc.radius - d).abs());
        }
        worst
    }

    #[test]
    fn flatten_respects_chord_tolerance() {
        let arc = Arc::circle(Point::new(0.0, 0.0), 10.0);
        let tol = 0.01;
        let pts = arc.flatten(tol);
        assert!(max_chord_error(&arc, &pts) <= tol + 1e-9);
    }

    #[test]
    fn full_circle_closes_back_to_start() {
        let arc = Arc::circle(Point::new(1.0, 2.0), 5.0);
        let pts = arc.flatten(0.05);
        let first = pts.first().unwrap();
        let last = pts.last().unwrap();
        assert!(
            first.distance(*last) < 1e-9,
            "full circle should return home"
        );
    }

    #[test]
    fn quarter_arc_endpoints_are_correct() {
        // 0 → π/2 CCW on the unit circle: (1,0) → (0,1).
        let arc = Arc::new(Point::new(0.0, 0.0), 1.0, 0.0, PI / 2.0, true);
        let pts = arc.flatten(0.001);
        assert!(pts.first().unwrap().distance(Point::new(1.0, 0.0)) < 1e-9);
        assert!(pts.last().unwrap().distance(Point::new(0.0, 1.0)) < 1e-9);
        assert!(matches!(arc.sweep(), s if (s - PI / 2.0).abs() < 1e-9));
    }

    #[test]
    fn clockwise_sweep_is_negative() {
        let arc = Arc::new(Point::new(0.0, 0.0), 1.0, 0.0, PI / 2.0, false);
        assert!(arc.sweep() < 0.0);
        // CW from angle 0 to π/2 is the long way round: −3π/2.
        assert!((arc.sweep() + 3.0 * PI / 2.0).abs() < 1e-9);
    }
}
