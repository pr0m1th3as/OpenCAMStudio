//! The viewport scene: colored polylines for the part outline and the toolpath
//! backplot, plus the pure mapping to GPU vertices. Nothing here touches a GPU,
//! so it is unit-tested like the rest of the pipeline.

use cam_cldata::{MoveKind, Point3, Program, Step};
use cam_geo::{Arc, Point, Polygon};

/// RGBA colour, linear, components in `0.0..=1.0`.
pub type Color = [f32; 4];

/// Part / stock outline colour.
pub const PART: Color = [0.80, 0.82, 0.88, 1.0];
/// Rapid / linking moves (non-cutting).
pub const RAPID: Color = [0.85, 0.75, 0.20, 1.0];
/// Cutting moves.
pub const CUT: Color = [0.25, 0.75, 0.35, 1.0];
/// Plunge / lead-in moves.
pub const PLUNGE: Color = [0.90, 0.35, 0.25, 1.0];
/// Tool-change / reorientation traverse (the planner's lift to tool-change height).
/// A blue, deliberately apart from the gold rapid and red-green-safe against the
/// cut/plunge pair.
pub const TRAVERSE: Color = [0.30, 0.60, 0.95, 1.0];

/// Chord tolerance (mm) used when flattening backplot arcs for display.
const ARC_TOL: f64 = 0.02;

/// A non-selected operation's strips fade this far (`0`=unchanged, `1`=full grey)
/// toward [`DIM_GREY`] when another operation is in focus.
const DIM_AMOUNT: f32 = 0.80;
/// The muted, slightly-cool grey non-selected operations recede toward — dark
/// enough to sit behind the (pale) part outline without vanishing.
const DIM_GREY: [f32; 3] = [0.32, 0.33, 0.37];

/// Fade a colour toward [`DIM_GREY`] by [`DIM_AMOUNT`], preserving alpha.
fn dim(c: Color) -> Color {
    let t = DIM_AMOUNT;
    [
        c[0] * (1.0 - t) + DIM_GREY[0] * t,
        c[1] * (1.0 - t) + DIM_GREY[1] * t,
        c[2] * (1.0 - t) + DIM_GREY[2] * t,
        c[3],
    ]
}

/// Whether a backplot move is drawn dashed.
///
/// Only the planner-inserted [`MoveKind::Traverse`] — the lift to tool-change height,
/// the cross, and the descent back to clearance. It is the one move the *operator*
/// never asked for, so it reads differently from the moves that came out of an
/// operation: colour says which kind of move it is, the dash says who put it there.
///
/// Deliberately not the rapid. A rapid is still a move the operation implies, and
/// dashing both would make the distinction the colour already carries harder to see,
/// not easier.
fn dashed_for(kind: MoveKind) -> bool {
    matches!(kind, MoveKind::Traverse)
}

/// The colour a backplot move is drawn in, by its role.
fn color_for(kind: MoveKind) -> Color {
    match kind {
        MoveKind::Link | MoveKind::Retract => RAPID,
        MoveKind::Traverse => TRAVERSE,
        MoveKind::Cutting => CUT,
        MoveKind::Plunge | MoveKind::LeadIn => PLUNGE,
    }
}

/// A vertex handed to the GPU: position (mm) and colour. Plain data; the `gpu`
/// feature makes it `bytemuck`-castable for upload.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "gpu", derive(bytemuck::Pod, bytemuck::Zeroable))]
pub struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

/// A connected run of points drawn in one colour.
#[derive(Clone, Debug, PartialEq)]
pub struct LineStrip {
    pub points: Vec<[f32; 3]>,
    pub color: Color,
    /// The operation this strip belongs to, if any. Backplot motions carry their
    /// CL-data `op_id`; part/stock outlines and pick highlights carry `None`.
    /// Drives selection focus (see [`Scene::focus_operation`]).
    pub op: Option<u32>,
    /// Draw this strip broken into dashes rather than solid.
    ///
    /// Carried on the strip rather than derived from the colour so that dimming an
    /// unfocused operation cannot silently change how a move is drawn — the two are
    /// independent properties of the same line.
    pub dashed: bool,
}

/// A drawable scene: a set of colored polylines.
#[derive(Clone, Debug, Default)]
pub struct Scene {
    pub strips: Vec<LineStrip>,
}

impl Scene {
    /// An empty scene.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a polyline strip with no operation identity (part outline, highlight).
    pub fn add_strip(&mut self, points: Vec<[f32; 3]>, color: Color) {
        self.add_strip_op(points, color, None);
    }

    /// Add a polyline strip tagged with an operation id (a backplot motion).
    pub fn add_strip_op(&mut self, points: Vec<[f32; 3]>, color: Color, op: Option<u32>) {
        self.add_strip_styled(points, color, op, false);
    }

    /// Add a polyline strip, choosing whether it draws solid or dashed.
    pub fn add_strip_styled(
        &mut self,
        points: Vec<[f32; 3]>,
        color: Color,
        op: Option<u32>,
        dashed: bool,
    ) {
        if points.len() >= 2 {
            self.strips.push(LineStrip {
                points,
                color,
                op,
                dashed,
            });
        }
    }

    /// Add a filled region's outlines (outer boundary and every hole), closed.
    pub fn add_region(&mut self, region: &Polygon, color: Color) {
        self.add_ring(region.outer().points(), color);
        for hole in region.holes() {
            self.add_ring(hole.points(), color);
        }
    }

    /// Add an **open** imported path — drawn as-is, *not* closed back to its start,
    /// so a lettering stroke reads as a stroke rather than a spurious loop.
    pub fn add_open_path(&mut self, pts: &[Point], color: Color) {
        if pts.len() < 2 {
            return;
        }
        let strip: Vec<[f32; 3]> = pts.iter().map(|p| [p.x as f32, p.y as f32, 0.0]).collect();
        self.add_strip(strip, color);
    }

    fn add_ring(&mut self, pts: &[Point], color: Color) {
        if pts.len() < 3 {
            return;
        }
        let mut strip: Vec<[f32; 3]> = pts.iter().map(|p| [p.x as f32, p.y as f32, 0.0]).collect();
        strip.push(strip[0]); // close the loop
        self.add_strip(strip, color);
    }

    /// Build a backplot from a CL-data program: one colored segment per motion,
    /// arcs flattened, colored by [`MoveKind`].
    pub fn from_program(program: &Program) -> Scene {
        let mut scene = Scene::new();
        let mut cur: Option<Point3> = None;
        for step in program.steps() {
            match step {
                Step::Rapid { to, tag } => {
                    push_segment(&mut scene, cur, *to, tag.kind, tag.op_id);
                    cur = Some(*to);
                }
                Step::Linear { to, tag, .. } => {
                    push_segment(&mut scene, cur, *to, tag.kind, tag.op_id);
                    cur = Some(*to);
                }
                Step::Arc {
                    end,
                    center,
                    dir,
                    tag,
                    ..
                } => {
                    if let Some(start) = cur {
                        push_arc(
                            &mut scene,
                            start,
                            *end,
                            *center,
                            *dir,
                            tag.kind,
                            tag.op_id,
                        );
                    }
                    cur = Some(*end);
                }
                Step::Drill(cycle) => {
                    // A drilled hole shows as a plunge marker at each point.
                    for &[x, y] in &cycle.points {
                        scene.add_strip_op(
                            vec![
                                [x as f32, y as f32, cycle.z_top as f32],
                                [x as f32, y as f32, cycle.depth as f32],
                            ],
                            color_for(cycle.tag.kind),
                            Some(cycle.tag.op_id),
                        );
                    }
                    cur = None;
                }
                // A work-datum change or a program stop breaks the toolpath: the next
                // group is a different fixturing/orientation, so lift the pen — don't
                // draw a rapid linking it to where the previous group ended.
                Step::Datum(_) | Step::Stop => cur = None,
                _ => {}
            }
        }
        scene
    }

    /// Fade every operation's strips except those in `focus` toward a muted grey,
    /// so the focused operations' toolpaths — keeping their full rapid/cut/plunge
    /// colours — stand out among many. More than one op may be focused at once.
    /// Strips with no op (part/stock outline, pick highlights) are left untouched;
    /// an empty `focus` leaves the whole scene at full colour. Idempotent enough
    /// for per-frame use on a freshly-cloned scene.
    pub fn focus_operations(&mut self, focus: &[u32]) {
        if focus.is_empty() {
            return;
        }
        // Only dim if at least one focused op actually has toolpath here —
        // otherwise (only not-yet-run or excluded ops are focused) leave the scene
        // at full colour rather than greying everything against paths not drawn.
        if !self
            .strips
            .iter()
            .any(|s| s.op.is_some_and(|op| focus.contains(&op)))
        {
            return;
        }
        for strip in &mut self.strips {
            if strip.op.is_some_and(|op| !focus.contains(&op)) {
                strip.color = dim(strip.color);
            }
        }
        // Draw focused strips last. The line pass writes no depth and always
        // passes, so coincident segments are resolved by paint order — without
        // this, a duplicated op sitting exactly on its original would let the
        // dimmed copy overpaint the vivid one (only the non-overlapping approach
        // moves would show through). A stable partition keeps each group's order.
        self.strips
            .sort_by_key(|s| s.op.is_some_and(|op| focus.contains(&op)));
    }

    /// Expand the strips into a flat `LineList` vertex buffer (two vertices per
    /// segment), breaking dashed strips into dashes at [`Scene::dash_period`].
    pub fn line_vertices(&self) -> Vec<Vertex> {
        self.line_vertices_dashed(self.dash_period())
    }

    /// The dash period (mm) this scene draws its dashed strips at: a fixed fraction
    /// of the scene's diagonal, clamped to a legible range.
    ///
    /// **Relative to the scene, not absolute**, because the camera frames on the
    /// scene: a fixed millimetre dash that reads well on a 50 mm part becomes a solid
    /// line on a 500 mm one. Deriving it from the extent keeps the pattern looking the
    /// same whatever the part's size. It is still a *world*-space pattern, so zooming
    /// right in stretches the dashes — at which point the traverse reads as a solid
    /// blue line, exactly as it did before this existed, so the worst case is the old
    /// behaviour rather than a wrong one.
    ///
    /// A screen-constant dash would need the vertex buffer rebuilt every frame against
    /// `world_per_pixel`; today it is built once when the scene changes.
    pub fn dash_period(&self) -> f32 {
        /// Dashes across the scene's diagonal.
        const DASHES: f32 = 90.0;
        /// Below this the pattern reads as a dotted smear, above it as a broken line.
        const MIN_MM: f32 = 0.25;
        const MAX_MM: f32 = 8.0;
        let Some((min, max)) = self.bounds() else {
            return MIN_MM;
        };
        let d = ((max[0] - min[0]).powi(2) + (max[1] - min[1]).powi(2) + (max[2] - min[2]).powi(2))
            .sqrt();
        (d / DASHES).clamp(MIN_MM, MAX_MM)
    }

    /// Expand the strips into a `LineList` buffer, using `period` mm for dashed
    /// strips. A non-positive or non-finite `period` draws everything solid.
    ///
    /// Split out from [`line_vertices`](Self::line_vertices) so the pattern can be
    /// tested at a known period instead of at whatever the scene's extent implies.
    pub fn line_vertices_dashed(&self, period: f32) -> Vec<Vertex> {
        /// Fraction of each period that is drawn. 0.6 leaves a gap wide enough to read
        /// at a glance without the line losing its continuity as a *path*.
        const DUTY: f64 = 0.6;
        /// Most dashes one strip may be broken into. A period fine enough to exceed
        /// this draws sub-pixel at any sane zoom, so the strip falls back to solid
        /// rather than sizing a vertex buffer off an arbitrary caller-supplied period.
        const MAX_DASHES: f64 = 100_000.0;

        let dashing = period.is_finite() && period > 0.0;
        let period = period as f64;
        let dash_on = period * DUTY;

        // In f64: the walk below is indexed off arc length, and accumulating that in
        // f32 over a long strip loses the resolution the dash boundaries need.
        let seg_len = |p: &[[f32; 3]]| -> f64 {
            let (a, b) = (p[0], p[1]);
            (((b[0] - a[0]) as f64).powi(2)
                + ((b[1] - a[1]) as f64).powi(2)
                + ((b[2] - a[2]) as f64).powi(2))
            .sqrt()
        };

        let mut out = Vec::new();
        for strip in &self.strips {
            let total: f64 = strip.points.windows(2).map(seg_len).sum();
            let dash_this = strip.dashed && dashing && total / period <= MAX_DASHES;

            let mut push = |a: [f32; 3], b: [f32; 3]| {
                out.push(Vertex {
                    position: a,
                    color: strip.color,
                });
                out.push(Vertex {
                    position: b,
                    color: strip.color,
                });
            };
            if !dash_this {
                for pair in strip.points.windows(2) {
                    push(pair[0], pair[1]);
                }
                continue;
            }
            // Walk the strip by arc length, indexing the dashes by integer: dash `k`
            // spans [k·period, k·period + dash_on) in the strip's own arc length. The
            // pattern still runs unbroken through corners — that is what makes the
            // index the *strip's* arc length rather than the segment's — but the loop
            // is now over a computed integer range, so it cannot fail to advance.
            //
            // It could before. Carrying a running `phase` and comparing `phase % period`
            // against the dash boundary meant that when the phase landed *on* a
            // boundary, rounding put it a hair inside the dash and yielded a step below
            // one ULP of the walk position: the walk stopped advancing and emitted
            // zero-length dashes until the process was OOM-killed. That hit 77 of 99
            // (length, period) pairs across the range `dash_period` can return.
            let mut s0 = 0.0_f64;
            for pair in strip.points.windows(2) {
                let (a, b) = (pair[0], pair[1]);
                let len = seg_len(pair);
                if len <= f64::EPSILON {
                    continue;
                }
                let (seg_start, s1) = (s0, s0 + len);
                let at = |s: f64| {
                    let f = ((s - seg_start) / len) as f32;
                    [
                        a[0] + (b[0] - a[0]) * f,
                        a[1] + (b[1] - a[1]) * f,
                        a[2] + (b[2] - a[2]) * f,
                    ]
                };
                let k0 = (seg_start / period).floor() as i64;
                let k1 = (s1 / period).floor() as i64;
                for k in k0..=k1 {
                    // Clip dash `k` to this segment; an empty overlap draws nothing.
                    let start = (k as f64 * period).max(seg_start);
                    let end = (k as f64 * period + dash_on).min(s1);
                    if end > start {
                        push(at(start), at(end));
                    }
                }
                s0 = s1;
            }
        }
        out
    }

    /// Axis-aligned bounds of everything in the scene, or `None` if empty.
    pub fn bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        let mut min = [f32::MAX; 3];
        let mut max = [f32::MIN; 3];
        let mut any = false;
        for strip in &self.strips {
            for p in &strip.points {
                any = true;
                for i in 0..3 {
                    min[i] = min[i].min(p[i]);
                    max[i] = max[i].max(p[i]);
                }
            }
        }
        any.then_some((min, max))
    }
}

fn push_segment(scene: &mut Scene, from: Option<Point3>, to: Point3, kind: MoveKind, op: u32) {
    if let Some(a) = from {
        scene.add_strip_styled(
            vec![
                [a.x as f32, a.y as f32, a.z as f32],
                [to.x as f32, to.y as f32, to.z as f32],
            ],
            color_for(kind),
            Some(op),
            dashed_for(kind),
        );
    }
}

#[allow(clippy::too_many_arguments)]
fn push_arc(
    scene: &mut Scene,
    start: Point3,
    end: Point3,
    center: Point3,
    dir: cam_cldata::ArcDir,
    kind: MoveKind,
    op: u32,
) {
    let r = ((start.x - center.x).powi(2) + (start.y - center.y).powi(2)).sqrt();
    let a0 = (start.y - center.y).atan2(start.x - center.x);
    let a1 = (end.y - center.y).atan2(end.x - center.x);
    let ccw = matches!(dir, cam_cldata::ArcDir::Ccw);
    let xy = Arc::new(Point::new(center.x, center.y), r, a0, a1, ccw).flatten(ARC_TOL);
    let n = xy.len().max(2);
    let pts: Vec<[f32; 3]> = xy
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let t = i as f64 / (n - 1) as f64;
            let z = start.z + (end.z - start.z) * t;
            [p.x as f32, p.y as f32, z as f32]
        })
        .collect();
    scene.add_strip_styled(pts, color_for(kind), Some(op), dashed_for(kind));
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_cldata::ProgramBuilder;

    #[test]
    fn backplot_colors_moves_by_kind() {
        let prog = ProgramBuilder::new()
            .op(0)
            .feed(300.0)
            .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
            .linear(Point3::new(0.0, 0.0, -1.0), MoveKind::Plunge)
            .linear(Point3::new(10.0, 0.0, -1.0), MoveKind::Cutting)
            .build();
        let scene = Scene::from_program(&prog);
        // Rapid has no prior position ⇒ no segment; then plunge + cut = 2 strips.
        assert_eq!(scene.strips.len(), 2);
        assert_eq!(scene.strips[0].color, PLUNGE);
        assert_eq!(scene.strips[1].color, CUT);
    }

    #[test]
    fn rapid_between_points_is_drawn_in_rapid_color() {
        let prog = ProgramBuilder::new()
            .op(0)
            .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
            .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
            .build();
        let scene = Scene::from_program(&prog);
        assert_eq!(scene.strips.len(), 1);
        assert_eq!(scene.strips[0].color, RAPID);
    }

    /// A three-operation program: ops 0, 1, 2 each with a rapid + a cut.
    fn three_op_program() -> Program {
        ProgramBuilder::new()
            .op(0)
            .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
            .linear(Point3::new(10.0, 0.0, -1.0), MoveKind::Cutting)
            .op(1)
            .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
            .linear(Point3::new(0.0, 10.0, -1.0), MoveKind::Cutting)
            .op(2)
            .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
            .linear(Point3::new(5.0, 5.0, -1.0), MoveKind::Cutting)
            .build()
    }

    #[test]
    fn focus_dims_other_operations_only() {
        let mut scene = Scene::from_program(&three_op_program());
        scene.add_strip(vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], PART); // op = None

        // Focus two of the three operations at once.
        scene.focus_operations(&[1, 2]);

        // Order-independent (focus reorders strips): focused ops and the untagged
        // part outline keep a full-palette colour; every other op is dimmed off it.
        let vivid = [RAPID, CUT, PLUNGE, PART];
        for strip in &scene.strips {
            let is_vivid = vivid.contains(&strip.color);
            match strip.op {
                Some(1) | Some(2) | None => assert!(is_vivid, "focused/untagged strip stays vivid"),
                Some(_) => assert!(!is_vivid, "unfocused op is dimmed"),
            }
        }
    }

    #[test]
    fn focus_draws_focused_strips_last() {
        // Focused ops must sort to the end so their vivid strips paint over any
        // coincident dimmed ones (the duplicate-on-original case).
        let mut scene = Scene::from_program(&three_op_program());
        scene.focus_operations(&[0]);
        let focused: Vec<bool> = scene
            .strips
            .iter()
            .map(|s| s.op == Some(0))
            .collect();
        // Once we reach the first focused strip, all remaining are focused too.
        let first_focused = focused.iter().position(|&f| f).unwrap();
        assert!(
            focused[first_focused..].iter().all(|&f| f),
            "focused strips are contiguous at the end: {focused:?}"
        );
        assert!(focused.last().copied().unwrap_or(false), "last strip is focused");
    }

    #[test]
    fn empty_focus_leaves_scene_untouched() {
        let before = Scene::from_program(&three_op_program());
        let mut after = before.clone();
        after.focus_operations(&[]);
        assert_eq!(before.strips, after.strips);
    }

    #[test]
    fn focus_on_absent_op_leaves_scene_untouched() {
        // Focusing only an op with no toolpath here must not grey everything.
        let before = Scene::from_program(&three_op_program());
        let mut after = before.clone();
        after.focus_operations(&[99]);
        assert_eq!(before.strips, after.strips);
    }

    #[test]
    fn line_vertices_are_two_per_segment() {
        let mut scene = Scene::new();
        scene.add_strip(vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]], CUT);
        // A 3-point strip is 2 segments ⇒ 4 vertices.
        assert_eq!(scene.line_vertices().len(), 4);
    }

    /// Total drawn length is the property, not the vertex count: the dash pattern is
    /// only correct if it draws the duty fraction of the line and no more.
    fn drawn_length(vs: &[Vertex]) -> f32 {
        vs.chunks(2)
            .map(|p| {
                let (a, b) = (p[0].position, p[1].position);
                ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt()
            })
            .sum()
    }

    #[test]
    fn a_dashed_strip_draws_the_duty_fraction_of_its_length() {
        let mut scene = Scene::new();
        // 100 mm along X, dashed at a 10 mm period ⇒ 10 dashes of 6 mm = 60 mm drawn.
        scene.add_strip_styled(vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]], TRAVERSE, Some(0), true);
        let vs = scene.line_vertices_dashed(10.0);
        assert_eq!(vs.len(), 20, "10 dashes ⇒ 10 segments ⇒ 20 vertices");
        assert!(
            (drawn_length(&vs) - 60.0).abs() < 1e-3,
            "expected 60 mm drawn of 100, got {}",
            drawn_length(&vs)
        );
        // Every dash must lie on the line it came from.
        assert!(vs.iter().all(|v| v.position[1] == 0.0 && v.position[2] == 0.0));
    }

    #[test]
    fn the_dash_phase_carries_across_a_corner() {
        // Two 5 mm legs at a right angle, dashed at a 4 mm period. If the phase reset at
        // the corner each leg would draw 2.4+ mm; carried, the pattern runs straight
        // through and the total is the duty fraction of the whole 10 mm.
        let mut scene = Scene::new();
        scene.add_strip_styled(
            vec![[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [5.0, 5.0, 0.0]],
            TRAVERSE,
            Some(0),
            true,
        );
        let drawn = drawn_length(&scene.line_vertices_dashed(4.0));
        // 10 mm at period 4, duty 0.6: dashes at [0,2.4] [4,6.4] [8,10] = 2.4+2.4+2 = 6.8.
        assert!((drawn - 6.8).abs() < 1e-3, "phase did not carry: drew {drawn}");
    }

    /// A same-orientation tool change must stay **connected** in the backplot: lift,
    /// horizontal cross at tool-change height, descend. A fixture reorientation is the
    /// one that breaks (the pen-lift at `Step::Datum`). That contrast is how an
    /// operator tells the two apart at a glance, so the cross surviving all the way
    /// into the vertex buffer is the property — not merely being in the `Program`,
    /// which is separately tested in `cam-toolpath`.
    #[test]
    fn a_same_orientation_tool_change_keeps_its_horizontal_cross() {
        let prog = ProgramBuilder::new()
            .op(0)
            .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
            .linear(Point3::new(10.0, 10.0, -1.0), MoveKind::Cutting)
            // The planner's transition: up, across, down. No Datum between them.
            .rapid(Point3::new(10.0, 10.0, 42.0), MoveKind::Traverse)
            .rapid(Point3::new(60.0, 40.0, 42.0), MoveKind::Traverse)
            .rapid(Point3::new(60.0, 40.0, 5.0), MoveKind::Traverse)
            .build();
        let scene = Scene::from_program(&prog);

        let horizontal: Vec<&LineStrip> = scene
            .strips
            .iter()
            .filter(|s| {
                s.points.len() == 2
                    && (s.points[0][2] - s.points[1][2]).abs() < 1e-6
                    && (s.points[0][0] - s.points[1][0]).abs() > 1e-6
            })
            .collect();
        assert!(
            horizontal.iter().any(|s| s.color == TRAVERSE),
            "the cross between the lift and the descent must be drawn: {:?}",
            scene.strips
        );

        // And it must reach the buffer — a dash pattern that swallowed a whole strip
        // would leave the operator with the same broken look a reorientation has.
        let cross = horizontal.iter().find(|s| s.color == TRAVERSE).unwrap();
        let mut only = Scene::new();
        only.add_strip_styled(cross.points.clone(), cross.color, cross.op, cross.dashed);
        let vs = only.line_vertices_dashed(scene.dash_period());
        assert!(!vs.is_empty(), "the cross drew nothing at all");
        let drawn = drawn_length(&vs);
        assert!(
            drawn > 0.25 * 58.31,
            "the cross drew only {drawn} mm of its ~58 mm — it would read as absent"
        );
    }

    /// The contrast the test above depends on: a reorientation *does* break, because
    /// `Step::Datum` lifts the pen. If this ever stopped being true the two transitions
    /// would look alike and the dash would be carrying the whole distinction.
    #[test]
    fn a_reorientation_breaks_where_a_tool_change_does_not() {
        let prog = ProgramBuilder::new()
            .op(0)
            .rapid(Point3::new(10.0, 10.0, 42.0), MoveKind::Traverse)
            .datum(2)
            .rapid(Point3::new(60.0, 40.0, 42.0), MoveKind::Traverse)
            .rapid(Point3::new(60.0, 40.0, 5.0), MoveKind::Traverse)
            .build();
        let scene = Scene::from_program(&prog);
        let horizontal = scene.strips.iter().any(|s| {
            s.points.len() == 2
                && (s.points[0][2] - s.points[1][2]).abs() < 1e-6
                && (s.points[0][0] - s.points[1][0]).abs() > 1e-6
        });
        assert!(!horizontal, "a reorientation's cross must stay broken: {:?}", scene.strips);
    }

    /// The two tests above pick their periods by hand, and both happen to be periods
    /// the walk survives. That is not a property of the code — it is luck. This sweeps
    /// the whole range [`Scene::dash_period`] can return, against several lengths.
    ///
    /// The original walk carried a running `phase` and compared `phase % period`
    /// against the dash boundary. Landing on a boundary rounded a hair *inside* the
    /// dash, giving a step below one ULP of the walk position; the walk then stopped
    /// advancing and emitted zero-length dashes until the process was OOM-killed. It
    /// did that for 77 of the 99 (length, period) pairs below — but not for 10 mm at
    /// period 4, nor 100 mm at period 10, so the suite stayed green.
    ///
    /// Note the failure mode a regression here would show: the buffer grows without
    /// bound, so this hangs and takes the machine's memory with it rather than failing
    /// cleanly. That is the loudest signal available from inside the walk itself; the
    /// structural guarantee is that the loop is now over a computed integer range.
    #[test]
    fn the_dash_walk_terminates_and_holds_its_duty_at_every_period() {
        const DUTY: f32 = 0.6;
        for len in [1.0_f32, 5.0, 12.7, 50.0, 100.0, 333.3, 500.0, 1000.0] {
            let mut scene = Scene::new();
            scene.add_strip_styled(
                vec![[0.0, 0.0, 0.0], [len, 0.0, 0.0]],
                TRAVERSE,
                Some(0),
                true,
            );
            // The clamped range of dash_period(), sampled finely enough to catch the
            // periods that divide these lengths exactly — those are the ones that put
            // the old walk's phase on a boundary.
            for i in 0..=64 {
                let period = 0.25 + (i as f32) * (8.0 - 0.25) / 64.0;
                let vs = scene.line_vertices_dashed(period);
                // Drawn length is the duty fraction, give or take the partial dash the
                // strip's end cuts short.
                let (drawn, want) = (drawn_length(&vs), len * DUTY);
                assert!(
                    (drawn - want).abs() <= period * DUTY + 1e-2,
                    "len {len} period {period}: drew {drawn}, expected ~{want}"
                );
                // Two vertices per dash, and at most one dash per period plus a partial
                // one at each end. A buffer past that means the walk emitted degenerate
                // dashes rather than advancing.
                let cap = 2.0 * (len / period + 2.0);
                assert!(
                    (vs.len() as f32) <= cap,
                    "len {len} period {period}: {} vertices exceeds the {cap} a dash \
                     pattern can need",
                    vs.len()
                );
            }
        }
    }

    /// A period fine enough to shatter a strip into an unbounded number of dashes
    /// draws it solid instead. `line_vertices_dashed` is public and takes any period,
    /// so the vertex count must not be a function of untrusted arithmetic.
    #[test]
    fn an_absurdly_fine_period_falls_back_to_solid() {
        let mut scene = Scene::new();
        scene.add_strip_styled(
            vec![[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0]],
            TRAVERSE,
            Some(0),
            true,
        );
        // 1000 mm at a 1 µm period would be a million dashes.
        let vs = scene.line_vertices_dashed(0.001);
        assert_eq!(vs.len(), 2, "expected the solid fallback, got {} verts", vs.len());
        assert!((drawn_length(&vs) - 1000.0).abs() < 1e-3);
    }

    #[test]
    fn only_the_traverse_is_dashed_and_solid_strips_are_untouched() {
        let prog = ProgramBuilder::new()
            .op(0)
            .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
            .rapid(Point3::new(0.0, 0.0, 50.0), MoveKind::Traverse)
            .linear(Point3::new(20.0, 0.0, 50.0), MoveKind::Traverse)
            .linear(Point3::new(20.0, 0.0, -1.0), MoveKind::Cutting)
            .build();
        let scene = Scene::from_program(&prog);
        for s in &scene.strips {
            assert_eq!(
                s.dashed,
                s.color == TRAVERSE,
                "only the traverse may be dashed: {s:?}"
            );
        }
        assert!(scene.strips.iter().any(|s| s.dashed), "the traverse must be dashed");

        // A solid strip yields exactly two vertices per segment, as it always has —
        // dashing must not have leaked into the ordinary path.
        let solid: usize = scene.strips.iter().filter(|s| !s.dashed).map(|s| s.points.len() - 1).sum();
        let vs = scene.line_vertices();
        let dashed_vs = vs.len() - solid * 2;
        assert!(dashed_vs > 0 && vs.len() > solid * 2, "the dashed strips added segments");
    }

    #[test]
    fn a_non_positive_dash_period_draws_everything_solid() {
        // The fallback that keeps a degenerate scene (a single point, no extent) from
        // producing an empty or infinite loop rather than a line.
        let mut scene = Scene::new();
        scene.add_strip_styled(vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]], TRAVERSE, Some(0), true);
        for period in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            let vs = scene.line_vertices_dashed(period);
            assert_eq!(vs.len(), 2, "period {period} must draw solid");
            assert!((drawn_length(&vs) - 10.0).abs() < 1e-6);
        }
    }

    #[test]
    fn the_dash_period_scales_with_the_scene_not_with_the_millimetre() {
        // The reason it is a fraction of the diagonal: a fixed millimetre dash that
        // reads on a small part becomes a solid line on a large one.
        let period_of = |size: f32| {
            let mut s = Scene::new();
            s.add_strip(vec![[0.0, 0.0, 0.0], [size, size, 0.0]], CUT);
            s.dash_period()
        };
        assert!(
            period_of(500.0) > period_of(50.0),
            "a larger part must get a longer dash"
        );
        // Both clamped into the legible band whatever the extreme.
        for size in [0.01_f32, 1.0, 100.0, 100_000.0] {
            let p = period_of(size);
            assert!((0.25..=8.0).contains(&p), "size {size} gave period {p}");
        }
        // An empty scene has no diagonal and must still answer with something drawable.
        assert!(Scene::new().dash_period() > 0.0);
    }

    #[test]
    fn bounds_span_all_points() {
        let mut scene = Scene::new();
        scene.add_strip(vec![[0.0, 0.0, -2.0], [10.0, 5.0, 3.0]], CUT);
        let (min, max) = scene.bounds().unwrap();
        assert_eq!(min, [0.0, 0.0, -2.0]);
        assert_eq!(max, [10.0, 5.0, 3.0]);
    }
}
