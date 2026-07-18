//! **Tool generatrix** — a tool's 2D revolve half-profile (`TOOLING_PLAN.md` §3.2).
//!
//! A [`Profile2D`] is the outer boundary of a tool's right half in the `(r, z)` plane
//! (`r ≥ 0`, axis = `+z`), an ordered chain from the **tip on the axis** `(0, 0)` up and
//! around the cutting end, along the side, to the **top on the axis** `(0, length)`.
//! Every segment is a line or a circular arc, tagged **cutting** or **non-cutting** — a
//! cutting surface removes material, a non-cutting one touching stock is a gouge.
//!
//! This module is **kernel-independent**: it knows nothing of `Tool`/`ToolKind` (those
//! live in `cam-model`, which maps a tool onto a [`GeneratrixSpec`]). Both the built-in
//! generators and, later, imported custom tools produce this one representation, so the
//! preview and the (eventual) generatrix-based tool identity treat them uniformly.

use crate::point::Point;

/// The shape of a generatrix segment (from the previous point to its `end`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SegShape {
    /// A straight line.
    Line,
    /// A circular arc about `center`, counter-clockwise (in the `r=x`, `z=y` plane) when
    /// `ccw`.
    Arc { center: Point, ccw: bool },
}

/// One boundary segment, ending at `end`, tagged cutting or not.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProfileSeg {
    /// Line, or arc about a centre.
    pub shape: SegShape,
    /// The `(r, z)` point this segment ends at.
    pub end: Point,
    /// Whether this surface **cuts** (a flute/edge) as opposed to shank/neck/top.
    pub cutting: bool,
}

/// A tool's revolve generatrix: the ordered outer boundary in the `(r, z)` plane, tip at
/// `start` (on the axis), each segment cutting-tagged.
#[derive(Clone, Debug, PartialEq)]
pub struct Profile2D {
    /// Where the boundary starts — the tip, on the axis (`(0, 0)` for every built-in).
    pub start: Point,
    /// The boundary segments, tip → top.
    pub segs: Vec<ProfileSeg>,
}

/// The cutting-end shape a generatrix starts with (kernel-neutral; `cam-model` maps a
/// `ToolKind` onto this).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BottomShape {
    /// Flat bottom (end mill / face mill).
    Flat,
    /// Full ball nose (corner radius = tool radius): a hemisphere, tip at the origin.
    Ball,
    /// Flat centre with a rounded corner of `corner_radius` mm (bull-nose).
    BullNose { corner_radius: f64 },
    /// Conical point (chamfer/V mill, drill), `half_angle_rad` from the axis, with an
    /// optional flat tip of `flat_radius` mm (`0` for a sharp point / twist drill).
    Cone {
        /// Half of the included point angle, from the axis (radians).
        half_angle_rad: f64,
        /// Flat-tip radius, mm.
        flat_radius: f64,
    },
}

/// Neutral parameters for [`generatrix`] — no `cam-model` dependency. All lengths mm.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeneratrixSpec {
    /// Cutting radius.
    pub radius: f64,
    /// Length of cut (cutting edge) from the tip.
    pub flute_length: f64,
    /// Shank radius (≥ radius for a plain tool; == radius = no distinct shank).
    pub shank_radius: f64,
    /// Overall length (stickout).
    pub length: f64,
    /// Reduced-neck length above the flutes (`0` = none).
    pub neck_length: f64,
    /// Reduced-neck radius.
    pub neck_radius: f64,
    /// The cutting-end shape.
    pub bottom: BottomShape,
}

const EPS: f64 = 1e-9;

fn line(end: Point, cutting: bool) -> ProfileSeg {
    ProfileSeg {
        shape: SegShape::Line,
        end,
        cutting,
    }
}

fn arc(end: Point, center: Point, ccw: bool, cutting: bool) -> ProfileSeg {
    ProfileSeg {
        shape: SegShape::Arc { center, ccw },
        end,
        cutting,
    }
}

/// Build the [`Profile2D`] for `spec`. Pure and total: degenerate inputs are clamped
/// (radii ≥ 0, flute clamped into `[z_side, length]`), never panicking.
pub fn generatrix(spec: &GeneratrixSpec) -> Profile2D {
    let r = spec.radius.max(0.0);
    let mut segs: Vec<ProfileSeg> = Vec::new();

    // 1) Cutting end — from the axis tip to full radius. Returns the z where the
    //    full-radius side begins.
    let z_side = match spec.bottom {
        BottomShape::Flat => {
            segs.push(line(Point::new(r, 0.0), true));
            0.0
        }
        BottomShape::Ball => {
            // Hemisphere: tip (0,0), equator (r, r), centre (0, r); a CCW quarter-arc.
            segs.push(arc(Point::new(r, r), Point::new(0.0, r), true, true));
            r
        }
        BottomShape::BullNose { corner_radius } => {
            let cr = corner_radius.clamp(0.0, r);
            if r - cr > EPS {
                segs.push(line(Point::new(r - cr, 0.0), true)); // flat centre
            }
            // Corner fillet: (r-cr, 0) → (r, cr), centre (r-cr, cr), CCW.
            segs.push(arc(Point::new(r, cr), Point::new(r - cr, cr), true, true));
            cr
        }
        BottomShape::Cone {
            half_angle_rad,
            flat_radius,
        } => {
            let rf = flat_radius.clamp(0.0, r);
            if rf > EPS {
                segs.push(line(Point::new(rf, 0.0), true)); // flat tip
            }
            // Cone flank: z = (r - rf) / tan(α) at full radius.
            let z_apex = if half_angle_rad > EPS {
                (r - rf) / half_angle_rad.tan()
            } else {
                0.0
            };
            segs.push(line(Point::new(r, z_apex), true));
            z_apex
        }
    };

    let length = spec.length.max(z_side);

    // 2) Flute side (cutting) up to the flute length (clamped into [z_side, length]).
    let flute_top = spec.flute_length.clamp(z_side, length);
    if flute_top - z_side > EPS {
        segs.push(line(Point::new(r, flute_top), true));
    }

    // 3) Non-cutting: optional neck, shank, top face.
    let r_s = spec.shank_radius.max(0.0);
    let r_n = spec.neck_radius.max(0.0);
    let neck_len = spec.neck_length.max(0.0);
    let mut z = flute_top;
    let mut r_cur = r;

    if neck_len > EPS && flute_top + neck_len <= length + EPS {
        // Step in to the neck, run up it, step out to the shank.
        if (r_n - r_cur).abs() > EPS {
            segs.push(line(Point::new(r_n, z), false));
            r_cur = r_n;
        }
        z = flute_top + neck_len;
        segs.push(line(Point::new(r_cur, z), false));
        if (r_s - r_cur).abs() > EPS {
            segs.push(line(Point::new(r_s, z), false));
            r_cur = r_s;
        }
    } else if (r_s - r_cur).abs() > EPS {
        // A distinct shank diameter but no neck: step across at the flute top.
        segs.push(line(Point::new(r_s, z), false));
        r_cur = r_s;
    }

    // Shank up to the overall length.
    if length - z > EPS {
        segs.push(line(Point::new(r_cur, length), false));
        z = length;
    }
    // Top face back to the axis (skipped if already on the axis, e.g. r == 0).
    if r_cur > EPS {
        segs.push(line(Point::new(0.0, z), false));
    }

    Profile2D {
        start: Point::new(0.0, 0.0),
        segs,
    }
}

impl Profile2D {
    /// The largest radius (`r`) anywhere on the boundary — the tool's cutting/shank
    /// extent.
    pub fn max_radius(&self) -> f64 {
        self.segs
            .iter()
            .map(|s| s.end.x)
            .fold(self.start.x, f64::max)
    }

    /// The overall height (max `z`) of the boundary.
    pub fn height(&self) -> f64 {
        self.segs
            .iter()
            .map(|s| s.end.y)
            .fold(self.start.y, f64::max)
    }

    /// Tessellate the boundary into a `(r, z)` polyline, arcs approximated with at most
    /// `arc_tol` mm of chord deviation. Includes `start` first. Used by the preview and
    /// by tests (bounds/monotonicity checks).
    pub fn polyline(&self, arc_tol: f64) -> Vec<Point> {
        let mut pts = vec![self.start];
        let mut prev = self.start;
        for s in &self.segs {
            match s.shape {
                SegShape::Line => pts.push(s.end),
                SegShape::Arc { center, ccw } => {
                    tessellate_arc(prev, s.end, center, ccw, arc_tol.max(1e-3), &mut pts);
                }
            }
            prev = s.end;
        }
        pts
    }
}

/// Push the interior + end points of the arc `from → to` about `center` onto `out`.
fn tessellate_arc(from: Point, to: Point, center: Point, ccw: bool, tol: f64, out: &mut Vec<Point>) {
    let radius = ((from.x - center.x).powi(2) + (from.y - center.y).powi(2)).sqrt();
    if radius < 1e-9 {
        out.push(to);
        return;
    }
    let a0 = (from.y - center.y).atan2(from.x - center.x);
    let a1 = (to.y - center.y).atan2(to.x - center.x);
    let mut sweep = a1 - a0;
    if ccw && sweep < 0.0 {
        sweep += std::f64::consts::TAU;
    } else if !ccw && sweep > 0.0 {
        sweep -= std::f64::consts::TAU;
    }
    // Step so the chord error ≤ tol: Δθ = 2·acos(1 − tol/R).
    let max_step = 2.0 * (1.0 - (tol / radius).min(1.0)).acos();
    let n = ((sweep.abs() / max_step.max(1e-3)).ceil() as usize).max(1);
    for i in 1..=n {
        let a = a0 + sweep * (i as f64) / (n as f64);
        out.push(Point::new(
            center.x + radius * a.cos(),
            center.y + radius * a.sin(),
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat(radius: f64, flute_length: f64, length: f64) -> GeneratrixSpec {
        GeneratrixSpec {
            radius,
            flute_length,
            shank_radius: radius,
            length,
            neck_length: 0.0,
            neck_radius: radius,
            bottom: BottomShape::Flat,
        }
    }

    #[test]
    fn flat_end_mill_splits_cutting_at_flute_length() {
        // ⌀12 (r=6), flute 20, overall 40 → bottom + side-to-20 cut; side 20→40 + top don't.
        let p = generatrix(&flat(6.0, 20.0, 40.0));
        assert_eq!(p.start, Point::new(0.0, 0.0));
        assert_eq!(p.max_radius(), 6.0);
        assert_eq!(p.height(), 40.0);
        // Boundary: (0,0)→(6,0) cut, →(6,20) cut, →(6,40) non-cut, →(0,40) non-cut.
        let ends: Vec<(Point, bool)> = p.segs.iter().map(|s| (s.end, s.cutting)).collect();
        assert_eq!(
            ends,
            vec![
                (Point::new(6.0, 0.0), true),
                (Point::new(6.0, 20.0), true),
                (Point::new(6.0, 40.0), false),
                (Point::new(0.0, 40.0), false),
            ]
        );
    }

    #[test]
    fn fully_fluted_when_flute_length_reaches_the_top() {
        // flute_length == length ⇒ the whole side cuts; only the top face is non-cutting.
        let p = generatrix(&flat(3.0, 30.0, 30.0));
        assert!(
            p.segs.iter().filter(|s| s.cutting).count() >= 2,
            "bottom + full side cut"
        );
        let non_cut: Vec<Point> = p.segs.iter().filter(|s| !s.cutting).map(|s| s.end).collect();
        assert_eq!(non_cut, vec![Point::new(0.0, 30.0)], "only the top face");
    }

    #[test]
    fn ball_nose_bottom_is_a_cutting_arc_of_radius_r() {
        let mut spec = flat(4.0, 20.0, 30.0);
        spec.bottom = BottomShape::Ball;
        let p = generatrix(&spec);
        let first = p.segs[0];
        assert!(matches!(first.shape, SegShape::Arc { .. }));
        assert!(first.cutting);
        assert_eq!(first.end, Point::new(4.0, 4.0), "equator at (r, r)");
        // The tessellated arc tip stays at the origin and never dips below z=0.
        let poly = p.polyline(0.05);
        assert!(poly.iter().all(|pt| pt.y >= -1e-9 && pt.x >= -1e-9));
    }

    #[test]
    fn drill_point_is_a_cone_to_the_full_radius() {
        // 118° drill: half-angle 59° from axis; z_apex = r / tan(59°).
        let half = 59.0_f64.to_radians();
        let mut spec = flat(2.5, 25.0, 40.0);
        spec.bottom = BottomShape::Cone {
            half_angle_rad: half,
            flat_radius: 0.0,
        };
        let p = generatrix(&spec);
        let cone = p.segs[0];
        assert!(cone.cutting);
        let expected_z = 2.5 / half.tan();
        assert!((cone.end.x - 2.5).abs() < 1e-9 && (cone.end.y - expected_z).abs() < 1e-9);
    }

    #[test]
    fn bull_nose_has_flat_centre_then_corner_arc() {
        let mut spec = flat(5.0, 20.0, 30.0);
        spec.bottom = BottomShape::BullNose { corner_radius: 1.5 };
        let p = generatrix(&spec);
        assert_eq!(p.segs[0].shape, SegShape::Line, "flat centre first");
        assert_eq!(p.segs[0].end, Point::new(3.5, 0.0)); // r - cr
        assert!(matches!(p.segs[1].shape, SegShape::Arc { .. }));
        assert_eq!(p.segs[1].end, Point::new(5.0, 1.5)); // (r, cr)
        assert!(p.segs[0].cutting && p.segs[1].cutting);
    }

    #[test]
    fn reduced_neck_and_shank_are_non_cutting_and_stepped() {
        let spec = GeneratrixSpec {
            radius: 3.0,
            flute_length: 8.0,
            shank_radius: 3.0,
            length: 50.0,
            neck_length: 20.0,
            neck_radius: 2.0,
            bottom: BottomShape::Flat,
        };
        let p = generatrix(&spec);
        // Everything above the flute top is non-cutting; the neck dips to r=2 then back.
        let above: Vec<Point> = p
            .segs
            .iter()
            .filter(|s| !s.cutting)
            .map(|s| s.end)
            .collect();
        assert!(above.contains(&Point::new(2.0, 8.0)), "step in to the neck");
        assert!(above.contains(&Point::new(2.0, 28.0)), "up the neck");
        assert!(above.contains(&Point::new(3.0, 28.0)), "step back out to the shank");
        assert!(above.contains(&Point::new(3.0, 50.0)), "shank to the top");
        assert!(above.last() == Some(&Point::new(0.0, 50.0)), "close on the axis");
        assert!(p.segs.iter().filter(|s| s.cutting).all(|s| s.end.y <= 8.0 + 1e-9));
    }
}
