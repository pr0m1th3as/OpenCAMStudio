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
    /// Carving V-bit cone with a **rounded** tip: a `tip_radius` arc tangent to a cone
    /// of half-angle `half_angle_rad` (from the axis). `tip_radius == 0` = a sharp cone.
    VTip {
        /// Half of the included V angle, from the axis (radians).
        half_angle_rad: f64,
        /// Rounded-tip radius, mm.
        tip_radius: f64,
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
                // Flat tip is **non-cutting** (a chamfer mill cuts only on the flank).
                segs.push(line(Point::new(rf, 0.0), false));
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
        BottomShape::VTip {
            half_angle_rad,
            tip_radius,
        } => {
            let a = half_angle_rad;
            let rt = tip_radius.clamp(0.0, r);
            if rt <= EPS || a <= EPS {
                // Sharp cone (no tip radius).
                let z_apex = if a > EPS { r / a.tan() } else { 0.0 };
                segs.push(line(Point::new(r, z_apex), true));
                z_apex
            } else {
                // Rounded tip: an arc of radius rt (centre on the axis at (0, rt)) tangent
                // to the cone, then the cone flank to the full radius.
                let r_t = rt * a.cos();
                let z_t = rt * (1.0 - a.sin());
                segs.push(arc(Point::new(r_t, z_t), Point::new(0.0, rt), true, true));
                let z_top = z_t + (r - r_t) / a.tan();
                segs.push(line(Point::new(r, z_top), true));
                z_top
            }
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

    /// How far up the tool's **full-radius flank** the surface still cuts, in mm above
    /// the tip — the deepest *vertical wall* it can cut before a non-cutting surface
    /// (neck, shank) would rub against the work.
    ///
    /// Measured as the highest `z` reachable by walking the boundary from the tip
    /// through **contiguously cutting** segments, counting only the part at the full
    /// [`max_radius`](Self::max_radius). A tool with no vertical flank at all — a
    /// V-bit or chamfer mill, whose cone flares straight into the shank — returns `0`:
    /// it cannot cut a vertical wall at any depth.
    pub fn cutting_flank_height(&self) -> f64 {
        let r = self.max_radius();
        if r <= EPS {
            return 0.0;
        }
        let mut prev = self.start;
        let mut best = 0.0_f64;
        let mut started = false;
        for s in &self.segs {
            if !s.cutting {
                // A *leading* non-cutting surface is skipped — a chamfer mill's flat
                // tip does not cut, yet the cone above it does. Once cutting has
                // begun, though, the first non-cutting surface ends the usable flank:
                // anything above it is only reachable by burying the shank.
                if started {
                    break;
                }
                prev = s.end;
                continue;
            }
            started = true;
            // Only a segment standing at the full radius is flank; a bottom or a
            // flare-out contributes no vertical wall.
            if (prev.x - r).abs() <= 1e-9 && (s.end.x - r).abs() <= 1e-9 {
                best = best.max(prev.y.max(s.end.y));
            }
            prev = s.end;
        }
        best
    }

    /// Whether the tool's bottom is a **flat, cutting** face spanning out from the axis
    /// — what a flat floor (facing, a pocket floor) needs.
    ///
    /// True for a square end mill or face mill. False for a ball nose or V-bit (they
    /// cut, but leave a curved/grooved floor) and false for a **chamfer mill**, whose
    /// flat tip is explicitly *non-cutting* and would leave an uncut ridge.
    pub fn cuts_flat_bottom(&self) -> bool {
        let Some(first) = self.segs.first() else {
            return false;
        };
        first.cutting
            && matches!(first.shape, SegShape::Line)
            && (first.end.y - self.start.y).abs() <= 1e-9
            && first.end.x > self.start.x + EPS
    }

    /// Whether the surface **at the tool's axis** cuts — i.e. whether the tool can
    /// plunge straight down into solid material.
    ///
    /// False for a chamfer mill (its tip flat is non-cutting) — plunging one rubs
    /// rather than cuts. True for an end mill, ball nose, drill or V-bit.
    pub fn has_cutting_tip(&self) -> bool {
        // The tip is where the boundary leaves the axis; the segment that starts
        // there carries the tag.
        self.segs
            .first()
            .is_some_and(|s| s.cutting && self.start.x <= EPS)
    }

    /// The boundary as per-segment tessellated polylines paired with their cutting flag —
    /// `(points, cutting)`, each sub-polyline sharing endpoints with its neighbours. Lets
    /// the preview style cutting vs non-cutting **per segment** (solid vs dashed).
    pub fn segment_polylines(&self, arc_tol: f64) -> Vec<(Vec<Point>, bool)> {
        let mut out = Vec::with_capacity(self.segs.len());
        let mut prev = self.start;
        for s in &self.segs {
            let mut pts = vec![prev];
            match s.shape {
                SegShape::Line => pts.push(s.end),
                SegShape::Arc { center, ccw } => {
                    tessellate_arc(prev, s.end, center, ccw, arc_tol.max(1e-3), &mut pts);
                }
            }
            out.push((pts, s.cutting));
            prev = s.end;
        }
        out
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

/// Half the width of the groove a **V-bit** cuts when its tip is sunk `depth` mm
/// below a flat surface — i.e. the radius at which the [`BottomShape::VTip`] profile
/// stands `depth` above its lowest point.
///
/// The tip is a `tip_radius` ball centred on the axis at `(0, tip_radius)`, tangent
/// to the cone at radial `tip_radius·cos α`, height `tip_radius·(1 − sin α)`. So the
/// width is **piecewise**:
///
/// - in the ball (`depth ≤ tip_radius·(1 − sin α)`): `√(2·rt·depth − depth²)`
/// - on the flank: `rt·cos α + (depth − rt·(1 − sin α))·tan α`
///
/// The two agree at the tangent point (both give `rt·cos α`), so the result is
/// continuous. For a **sharp** tip (`tip_radius == 0`) this reduces to the familiar
/// `depth·tan α`; note that for a *tipped* bit the naive `depth·tan α` badly
/// understates the width at shallow depth — exactly the engraving regime — since the
/// ball term grows as `√depth`.
///
/// A non-positive `depth` gives `0`. The result is **not** clamped to the tool's
/// cutting radius: callers that care (an engraving strategy checking the cone has not
/// flared past the shank) should gate on [`vtip_max_depth`] first.
pub fn vtip_half_width(half_angle_rad: f64, tip_radius: f64, depth: f64) -> f64 {
    if depth <= 0.0 {
        return 0.0;
    }
    let a = half_angle_rad;
    let rt = tip_radius.max(0.0);
    if rt <= EPS || a <= EPS {
        // Sharp cone: the flank starts at the point itself.
        return if a > EPS { depth * a.tan() } else { 0.0 };
    }
    let z_t = rt * (1.0 - a.sin()); // tangent height, ball → cone
    if depth <= z_t {
        // On the ball: a circle of radius rt centred at height rt.
        (2.0 * rt * depth - depth * depth).max(0.0).sqrt()
    } else {
        rt * a.cos() + (depth - z_t) * a.tan()
    }
}

/// The greatest depth a **V-bit** of cutting radius `radius` can engrave before the
/// cone flares past its cutting edge into the shank — the height of the
/// [`BottomShape::VTip`] profile at the full cutting radius.
///
/// Cutting deeper than this rubs the shank against the groove walls: no cutting edge
/// is in contact, so it is a hard limit, not a preference.
pub fn vtip_max_depth(half_angle_rad: f64, tip_radius: f64, radius: f64) -> f64 {
    let a = half_angle_rad;
    let r = radius.max(0.0);
    if a <= EPS {
        return 0.0;
    }
    let rt = tip_radius.clamp(0.0, r);
    if rt <= EPS {
        return r / a.tan();
    }
    let r_t = rt * a.cos();
    let z_t = rt * (1.0 - a.sin());
    if r <= r_t {
        // The cutting radius ends inside the tip ball.
        return rt - (rt * rt - r * r).max(0.0).sqrt();
    }
    z_t + (r - r_t) / a.tan()
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

    // --- derived cutting-surface properties (operation guards) ---

    fn spec_of(bottom: BottomShape, radius: f64, flute_length: f64, shank_radius: f64) -> GeneratrixSpec {
        GeneratrixSpec {
            radius,
            flute_length,
            shank_radius,
            length: 40.0,
            neck_length: 0.0,
            neck_radius: radius,
            bottom,
        }
    }

    #[test]
    fn flank_height_is_the_flute_length_for_a_plain_end_mill() {
        let p = generatrix(&spec_of(BottomShape::Flat, 3.0, 12.0, 3.0));
        assert!((p.cutting_flank_height() - 12.0).abs() < 1e-9);
    }

    #[test]
    fn a_vbit_has_no_vertical_flank_at_all() {
        // The cone flares straight into the shank — there is no cylindrical cutting
        // surface, so it can never cut a vertical wall. This is what stops a V-bit
        // being accepted for profiling/pocketing.
        let p = generatrix(&spec_of(
            BottomShape::VTip { half_angle_rad: (30.0_f64).to_radians(), tip_radius: 0.0 },
            3.0,
            0.0,
            3.0,
        ));
        assert_eq!(p.cutting_flank_height(), 0.0);
        let c = generatrix(&spec_of(
            BottomShape::Cone { half_angle_rad: (45.0_f64).to_radians(), flat_radius: 0.5 },
            3.0,
            0.0,
            3.0,
        ));
        assert_eq!(c.cutting_flank_height(), 0.0);
    }

    #[test]
    fn flank_height_stops_at_the_first_non_cutting_surface() {
        // Flute 8 of an overall 40 tool: the shank above it must not count, or a
        // depth guard would happily bury the shank in the work.
        let p = generatrix(&spec_of(BottomShape::Flat, 3.0, 8.0, 3.0));
        assert!((p.cutting_flank_height() - 8.0).abs() < 1e-9);
        assert!(p.height() > 8.0, "the tool is longer than its flutes");
    }

    #[test]
    fn only_a_flat_cutting_bottom_counts_as_flat_bottomed() {
        assert!(generatrix(&spec_of(BottomShape::Flat, 3.0, 10.0, 3.0)).cuts_flat_bottom());
        assert!(!generatrix(&spec_of(BottomShape::Ball, 3.0, 10.0, 3.0)).cuts_flat_bottom());
        assert!(!generatrix(&spec_of(
            BottomShape::VTip { half_angle_rad: 0.5, tip_radius: 0.0 }, 3.0, 0.0, 3.0
        )).cuts_flat_bottom());
        // A chamfer mill's flat IS flat but is tagged non-cutting — it would leave an
        // uncut ridge, so it must not pass as a flat-bottomed tool.
        let cham = generatrix(&spec_of(
            BottomShape::Cone { half_angle_rad: (45.0_f64).to_radians(), flat_radius: 0.5 },
            3.0, 0.0, 3.0,
        ));
        assert!(!cham.cuts_flat_bottom());
    }

    #[test]
    fn a_chamfer_mill_has_no_cutting_tip_but_a_vbit_does() {
        let cham = generatrix(&spec_of(
            BottomShape::Cone { half_angle_rad: (45.0_f64).to_radians(), flat_radius: 0.5 },
            3.0, 0.0, 3.0,
        ));
        assert!(!cham.has_cutting_tip(), "the flat tip does not cut");
        let vbit = generatrix(&spec_of(
            BottomShape::VTip { half_angle_rad: (30.0_f64).to_radians(), tip_radius: 0.1 },
            3.0, 0.0, 3.0,
        ));
        assert!(vbit.has_cutting_tip());
        assert!(generatrix(&spec_of(BottomShape::Flat, 3.0, 10.0, 3.0)).has_cutting_tip());
    }

    // --- V-groove width (engraving) ---

    #[test]
    fn sharp_vtip_groove_is_the_naive_cone_width() {
        // 90° included → α=45°, tan α = 1, so half-width == depth at any depth.
        let a = std::f64::consts::FRAC_PI_4;
        for d in [0.1, 0.5, 1.0, 3.0] {
            assert!((vtip_half_width(a, 0.0, d) - d).abs() < 1e-12);
        }
        // 60° included → α=30°: half-width = d·tan30.
        let a30 = std::f64::consts::FRAC_PI_6;
        assert!((vtip_half_width(a30, 0.0, 2.0) - 2.0 * a30.tan()).abs() < 1e-12);
    }

    #[test]
    fn tipped_vtip_is_continuous_across_the_ball_cone_tangent() {
        // At the tangent point both branches must give rt·cos α — this is what proves
        // the piecewise split is placed correctly, not merely plausible.
        for &(deg, rt) in &[(90.0, 0.2), (60.0, 0.5), (30.0, 0.1), (120.0, 0.3)] {
            let a = (deg * 0.5_f64).to_radians();
            let z_t = rt * (1.0 - a.sin());
            let expect = rt * a.cos();
            let below = vtip_half_width(a, rt, z_t - 1e-9);
            let at = vtip_half_width(a, rt, z_t);
            let above = vtip_half_width(a, rt, z_t + 1e-9);
            assert!((at - expect).abs() < 1e-9, "deg={deg} at={at} want={expect}");
            assert!((below - at).abs() < 1e-6, "deg={deg} ball side jumps");
            assert!((above - at).abs() < 1e-6, "deg={deg} cone side jumps");
        }
    }

    #[test]
    fn a_tipped_vtip_cuts_wider_than_the_naive_formula_when_shallow() {
        // The bug this guards: using depth·tanα for a tipped bit understates the
        // groove badly at engraving depths, because the ball term grows as √depth.
        let a = std::f64::consts::FRAC_PI_6; // 60° included
        let rt = 0.3;
        let d = 0.02; // a shallow engraving pass, well inside the ball
        let naive = d * a.tan();
        let real = vtip_half_width(a, rt, d);
        assert!(real > 3.0 * naive, "real={real} naive={naive}");
        // And it is exactly the circle: √(2·rt·d − d²).
        assert!((real - (2.0 * rt * d - d * d).sqrt()).abs() < 1e-12);
    }

    #[test]
    fn vtip_width_is_monotonic_in_depth_and_zero_at_the_surface() {
        let a = (30.0_f64).to_radians();
        assert_eq!(vtip_half_width(a, 0.25, 0.0), 0.0);
        assert_eq!(vtip_half_width(a, 0.25, -1.0), 0.0);
        let mut prev = 0.0;
        for i in 1..400 {
            let d = i as f64 * 0.01;
            let w = vtip_half_width(a, 0.25, d);
            assert!(w > prev, "not monotonic at d={d}");
            prev = w;
        }
    }

    #[test]
    fn vtip_max_depth_agrees_with_the_generatrix_flank_top() {
        // The gate must equal the profile's own height at the cutting radius.
        for &(deg, rt, r) in &[(60.0, 0.0, 3.0), (90.0, 0.2, 3.0), (30.0, 0.5, 6.0)] {
            let a = (deg * 0.5_f64).to_radians();
            let dmax = vtip_max_depth(a, rt, r);
            // At the limit the groove half-width is exactly the cutting radius.
            assert!(
                (vtip_half_width(a, rt, dmax) - r).abs() < 1e-9,
                "deg={deg} rt={rt} dmax={dmax}"
            );
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
    fn vbit_cone_is_cutting_then_the_shaft_is_not() {
        // A sharp 60° V-bit, shaft ⌀6 (r=3), overall 40. flute_length 0 ⇒ the cone is the
        // cutting portion, the shaft above is non-cutting.
        let spec = GeneratrixSpec {
            radius: 3.0,
            flute_length: 0.0,
            shank_radius: 3.0,
            length: 40.0,
            neck_length: 0.0,
            neck_radius: 3.0,
            bottom: BottomShape::VTip {
                half_angle_rad: 30.0_f64.to_radians(), // 60° full → 30° half
                tip_radius: 0.0,
            },
        };
        let p = generatrix(&spec);
        // Cone flank cutting to (r, r/tan30) then non-cutting shaft.
        let z_cone = 3.0 / 30.0_f64.to_radians().tan();
        assert!(p.segs[0].cutting, "the cone cuts");
        assert!((p.segs[0].end.x - 3.0).abs() < 1e-9 && (p.segs[0].end.y - z_cone).abs() < 1e-9);
        assert!(
            p.segs.iter().filter(|s| s.cutting).all(|s| s.end.y <= z_cone + 1e-9),
            "nothing above the cone cuts"
        );

        // A rounded tip starts with a cutting arc.
        let mut spec2 = spec;
        spec2.bottom = BottomShape::VTip {
            half_angle_rad: 30.0_f64.to_radians(),
            tip_radius: 0.5,
        };
        let p2 = generatrix(&spec2);
        assert!(matches!(p2.segs[0].shape, SegShape::Arc { .. }) && p2.segs[0].cutting);
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
