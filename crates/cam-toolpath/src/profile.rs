//! The profiling strategy: follow a closed chain at a tool-radius offset, in
//! stepdown passes, down to depth.

use cam_cldata::{ArcDir, CutterComp, MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, Contour, JoinStyle, Point, Polygon};
use cam_model::{Comp, Heights, Lead, Plunge, ProfileOp, Side};

use crate::{CancelToken, Diagnostic, JobEnv, Strategy, StrategyResult};

/// Profiles a single closed chain. Construct from a [`ProfileOp`].
#[derive(Clone, Debug)]
pub struct ProfileStrategy {
    op: ProfileOp,
}

impl ProfileStrategy {
    /// Build a profiling strategy for `op`.
    pub fn new(op: ProfileOp) -> Self {
        Self { op }
    }
}

impl Strategy for ProfileStrategy {
    fn name(&self) -> &str {
        "profile"
    }

    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        let op = &self.op;
        let mut diagnostics = Vec::new();

        // Look up the tool.
        let Some(tool) = env.tool(op.tool) else {
            diagnostics.push(Diagnostic::error(format!(
                "operation {} references tool {} which is not in the setup",
                op.id, op.tool
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        };

        // Validate the geometry and parameters.
        if !op.chain.is_valid() {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: profile chain must be a closed area (≥ 3 vertices)",
                op.id
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        if op.stepdown <= 0.0 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: stepdown must be positive",
                op.id
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        // `depth` is a positive magnitude below the reference; the floor sits at Z = -depth.
        let floor = -op.depth;
        if floor >= env.heights.top_of_stock {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: depth {} does not reach below the stock top {}; nothing to cut",
                op.id, op.depth, env.heights.top_of_stock
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        let region = match Polygon::new(op.chain.clone()) {
            Ok(p) => p,
            Err(e) => {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: chain is not a valid region: {e}",
                    op.id
                )));
                return StrategyResult {
                    diagnostics,
                    ..Default::default()
                };
            }
        };

        // Computed comp offsets the geometry ourselves; control comp keeps the
        // path on the contour and lets the controller (G41/G42) do the radius. The
        // finishing allowance (`offset`) leaves stock on the wall — it moves the
        // path the same way the tool radius does (outward for Outside, inward for
        // Inside), and is baked into the programmed path for control comp too so a
        // roughing pass stops short of the edge. Its direction follows `side`
        // (ignored for `On`, which has no material side).
        let side_sign = match op.side {
            Side::Outside => 1.0,
            Side::Inside => -1.0,
            Side::On => 0.0,
        };
        let (radius, comp) = match op.comp {
            Comp::Computed => (tool.radius(), None),
            Comp::ControlLeft => (0.0, Some(CutterComp::Left(op.tool))),
            Comp::ControlRight => (0.0, Some(CutterComp::Right(op.tool))),
        };
        let signed = side_sign * (radius + op.offset);

        // Radial roughing (stepover) is **outside-only**: clear the frame out to the
        // raw stock in concentric passes, leaving the finishing `offset` on the wall
        // — the roughing half of the rough-then-finish workflow. Inner clearing is a
        // pocket's job (an inner profile is a single-pass wall finish), so stepover
        // is ignored for Inside/On. Not applicable ⇒ fall through to a single pass.
        // (Leads apply to the single-pass finish, not the roughing rings.)
        if op.stepover > 0.0 && matches!(op.comp, Comp::Computed) && side_sign > 0.0 {
            if let Some(region) = roughing_region(op, env.stock) {
                // The finishing allowance is baked into the island, so the wall ring
                // sits at the tool radius.
                let first = radius;
                let rings =
                    match crate::rings::concentric_rings(&region, first, op.stepover, cancel) {
                        Ok(rings) => rings,
                        Err(crate::rings::RingsError::Cancelled) => {
                            return StrategyResult {
                                diagnostics,
                                cancelled: true,
                                ..Default::default()
                            };
                        }
                        Err(crate::rings::RingsError::Offset(e)) => {
                            diagnostics.push(Diagnostic::error(format!(
                                "operation {}: offset failed: {e}",
                                op.id
                            )));
                            return StrategyResult {
                                diagnostics,
                                ..Default::default()
                            };
                        }
                    };
                if !rings.is_empty() {
                    let levels = depth_levels(env.heights.top_of_stock, floor, op.stepdown);
                    let mut program = Program::new();
                    for &z in &levels {
                        if cancel.is_cancelled() {
                            return StrategyResult {
                                program,
                                diagnostics,
                                cancelled: true,
                            };
                        }
                        for ring in &rings {
                            let rotated = rotate_to_start(ring, op.start);
                            crate::rings::emit_ring(
                                &mut program,
                                &rotated,
                                op.id,
                                op.feed,
                                op.plunge_feed,
                                op.plunge,
                                op.lead_overlap,
                                &env.heights,
                                z,
                            );
                        }
                    }
                    return StrategyResult {
                        program,
                        diagnostics,
                        cancelled: false,
                    };
                }
                // No rings (degenerate frame) — fall through to a single pass.
            }
        }

        // An inner profile is a single-pass wall finish. Warn (don't block) if it
        // would leave an uncut core — material the finish loop can't reach: the
        // chain offset inward past the swept band (a tool diameter beyond the wall).
        // That's the signal to rough the pocket first; it's a local check, so it
        // fires whether or not anything actually roughed it.
        if side_sign < 0.0 && matches!(op.comp, Comp::Computed) {
            let core = 2.0 * tool.radius() + op.offset;
            let leaves_core = Polygon::new(op.chain.clone())
                .ok()
                .and_then(|p| offset(&[p], -core, JoinStyle::Round).ok())
                .is_some_and(|v| !v.is_empty());
            if leaves_core {
                diagnostics.push(Diagnostic::warning(format!(
                    "operation {}: inner profile leaves an uncut core — rough it with a pocket first",
                    op.id
                )));
            }
        }

        let loops = if signed == 0.0 {
            vec![region]
        } else {
            match offset(&[region], signed, JoinStyle::Round) {
                Ok(v) => v,
                Err(e) => {
                    diagnostics.push(Diagnostic::error(format!(
                        "operation {}: offset failed: {e}",
                        op.id
                    )));
                    return StrategyResult {
                        diagnostics,
                        ..Default::default()
                    };
                }
            }
        };
        if loops.is_empty() {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: tool (⌀{}) is too large — the offset consumed the whole feature",
                op.id, tool.diameter
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }

        let levels = depth_levels(env.heights.top_of_stock, floor, op.stepdown);
        let mut program = Program::new();
        for poly in &loops {
            if cancel.is_cancelled() {
                return StrategyResult {
                    program,
                    diagnostics,
                    cancelled: true,
                };
            }
            emit_loop(
                &mut program,
                poly.outer().points(),
                op,
                &env.heights,
                &levels,
                comp,
            );
        }

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

/// The region to clear when radial roughing an **outside** profile: the frame
/// between the raw stock and the part, so the region is the stock with the part —
/// dilated by the finishing allowance — as an island. Returns `None` when roughing
/// does not apply (no stock, or the stock does not contain the part), so the caller
/// falls back to a single pass.
fn roughing_region(op: &ProfileOp, stock: Option<([f64; 2], [f64; 2])>) -> Option<Polygon> {
    let chain_poly = Polygon::new(op.chain.clone()).ok()?;
    // The stock must strictly contain the part for there to be a frame.
    let (smin, smax) = stock?;
    let (mut xmin, mut ymin, mut xmax, mut ymax) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for p in op.chain.points() {
        xmin = xmin.min(p.x);
        ymin = ymin.min(p.y);
        xmax = xmax.max(p.x);
        ymax = ymax.max(p.y);
    }
    if !(smin[0] < xmin && smin[1] < ymin && smax[0] > xmax && smax[1] > ymax) {
        return None;
    }
    // The part is an island, dilated by the finishing allowance so that much stock
    // is left on the wall.
    let island = if op.offset > 0.0 {
        offset(&[chain_poly], op.offset, JoinStyle::Round)
            .ok()?
            .into_iter()
            .next()?
            .outer()
            .clone()
    } else {
        op.chain.clone()
    };
    let stock_rect = Contour::new(vec![
        Point::new(smin[0], smin[1]),
        Point::new(smax[0], smin[1]),
        Point::new(smax[0], smax[1]),
        Point::new(smin[0], smax[1]),
    ]);
    Polygon::with_holes(stock_rect, vec![island]).ok()
}

/// Emit approach, stepdown passes, and retract for one closed tool-path loop.
/// When `comp` is set, the cut is bracketed by controller cutter compensation.
fn emit_loop(
    prog: &mut Program,
    pts: &[cam_geo::Point],
    op: &ProfileOp,
    h: &Heights,
    levels: &[f64],
    comp: Option<CutterComp>,
) {
    if pts.len() < 3 {
        return;
    }
    let rotated = rotate_to_start(pts, op.start);
    let pts = rotated.as_slice();
    let start = pts[0];

    // Rich entry (leads, a closure overlap, and/or a non-straight plunge) takes a
    // separate path; the default (no lead + no overlap + straight plunge) keeps the
    // original, byte-stable emission.
    if op.lead_in != Lead::None
        || op.lead_out != Lead::None
        || op.lead_overlap > 0.0
        || op.plunge != Plunge::Straight
    {
        emit_loop_rich(prog, pts, op, h, levels, comp);
        return;
    }

    let link = Tag::new(op.id, MoveKind::Link);
    let plunge = Tag::new(op.id, MoveKind::Plunge);
    let cut = Tag::new(op.id, MoveKind::Cutting);
    let retract = Tag::new(op.id, MoveKind::Retract);

    // Approach: rapid over the start at clearance, then down to the stock top.
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.clearance),
        tag: link,
    });
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.top_of_stock),
        tag: link,
    });

    for &z in levels {
        // Plunge to this level, then cut the loop (arcs refitted) and close it.
        prog.push(Step::Linear {
            to: Point3::new(start.x, start.y, z),
            feed: op.plunge_feed,
            tag: plunge,
        });
        if let Some(c) = comp {
            prog.push(Step::CutterComp(c));
        }
        crate::emit::cut_loop(prog, pts, op.feed, cut, z);
        if comp.is_some() {
            prog.push(Step::CutterComp(CutterComp::Off));
        }
    }

    // Retract clear of the part.
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.clearance),
        tag: retract,
    });
}

/// The rich profile emission (leads and/or a non-straight plunge). Per level: rapid
/// to the entry footprint (off the contour when there is a lead-in), rapid down
/// through the already-cut air to the previous level, enter Z per the plunge
/// strategy, lead onto the contour, cut the loop, lead off, and retract.
fn emit_loop_rich(
    prog: &mut Program,
    pts: &[Point],
    op: &ProfileOp,
    h: &Heights,
    levels: &[f64],
    comp: Option<CutterComp>,
) {
    let start = pts[0];
    let tan_in = start_tangent(pts); // leaving start, into the cut
    let out = outward_normal(pts); // away from the loop interior (leaving tangent)
    let link = Tag::new(op.id, MoveKind::Link);
    let lead = Tag::new(op.id, MoveKind::LeadIn);
    let cut = Tag::new(op.id, MoveKind::Cutting);
    let retract = Tag::new(op.id, MoveKind::Retract);
    let plunge_tag = Tag::new(op.id, MoveKind::Plunge);

    // The cut runs the loop and then keeps going `lead_overlap` mm past the start
    // to a point `exit_on`, so the lead-off (and its junction) is re-machined. With
    // no overlap, `exit_on == start`, `tan_out` is the arrival tangent, and the cut
    // polyline is exactly the closed loop — byte-identical to the prior emission.
    let (loop_pts, exit_on, tan_out) = crate::emit::loop_with_overlap(pts, op.lead_overlap);
    let out_exit = if op.lead_overlap > 0.0 {
        outward_normal_at(tan_out, signed_area2(pts) > 0.0)
    } else {
        out
    };

    let entry = crate::leads::lead_start_point(start, tan_in, out, op.lead_in);
    let exit = crate::leads::lead_end_point(exit_on, tan_out, out_exit, op.lead_out);

    let mut prev_z = h.top_of_stock;
    for &z in levels {
        prog.push(Step::Rapid {
            to: Point3::new(entry.x, entry.y, h.clearance),
            tag: link,
        });
        prog.push(Step::Rapid {
            to: Point3::new(entry.x, entry.y, prev_z),
            tag: link,
        });

        emit_plunge(
            prog,
            entry,
            tan_in,
            out,
            prev_z,
            z,
            op.plunge,
            op.plunge_feed,
            op.feed,
            plunge_tag,
        );

        // Lead onto the contour at depth: entry → start.
        crate::leads::emit_lead(prog, entry, start, start, out, op.lead_in, z, op.feed, lead);

        if let Some(c) = comp {
            prog.push(Step::CutterComp(c));
        }
        crate::emit::cut_polyline(prog, &loop_pts, op.feed, cut, z);
        if comp.is_some() {
            prog.push(Step::CutterComp(CutterComp::Off));
        }

        // Lead off the contour at depth: exit_on (start, or the overlap point) → exit.
        crate::leads::emit_lead(prog, exit_on, exit, exit_on, out_exit, op.lead_out, z, op.feed, lead);

        prog.push(Step::Rapid {
            to: Point3::new(exit.x, exit.y, h.clearance),
            tag: retract,
        });
        prev_z = z;
    }
}

/// Unit vector, or `(1,0)` if degenerate.
pub(crate) fn unit(x: f64, y: f64) -> (f64, f64) {
    let l = (x * x + y * y).sqrt();
    if l > 1e-12 {
        (x / l, y / l)
    } else {
        (1.0, 0.0)
    }
}

/// Unit tangent leaving the start vertex (start → pts[1]).
pub(crate) fn start_tangent(pts: &[Point]) -> (f64, f64) {
    unit(pts[1].x - pts[0].x, pts[1].y - pts[0].y)
}

/// Twice the signed area of the closed loop (shoelace); `> 0` is CCW.
fn signed_area2(pts: &[Point]) -> f64 {
    let n = pts.len();
    (0..n)
        .map(|i| {
            let a = pts[i];
            let b = pts[(i + 1) % n];
            a.x * b.y - b.x * a.y
        })
        .sum()
}

/// Whether the closed loop winds counter-clockwise — the orientation needed to
/// place a lead's outward normal (see [`outward_normal_at`]).
pub(crate) fn is_ccw(pts: &[Point]) -> bool {
    signed_area2(pts) > 0.0
}

/// The outward normal at the start (away from the loop interior), for placing leads
/// and helix centres on the non-material side.
pub(crate) fn outward_normal(pts: &[Point]) -> (f64, f64) {
    outward_normal_at(start_tangent(pts), signed_area2(pts) > 0.0)
}

/// The outward normal for a given travel tangent on a loop of the given
/// orientation (`ccw`), away from the interior — the point-agnostic core of
/// [`outward_normal`], reused at an overlap point where the tangent differs from
/// the start's.
pub(crate) fn outward_normal_at(t: (f64, f64), ccw: bool) -> (f64, f64) {
    // Interior is left of travel for a CCW loop, so outward is the right normal;
    // the reverse for CW.
    if ccw {
        (t.1, -t.0)
    } else {
        (-t.1, t.0)
    }
}

/// Emit the plunge from `p@from_z` down to `p@to_z` (ending at `p` in XY), per the
/// strategy. Bad parameters fall back to a straight plunge (never panic).
#[allow(clippy::too_many_arguments)]
pub(crate) fn emit_plunge(
    prog: &mut Program,
    p: Point,
    tan: (f64, f64),
    out: (f64, f64),
    from_z: f64,
    to_z: f64,
    plunge: Plunge,
    plunge_feed: f64,
    cut_feed: f64,
    tag: Tag,
) {
    let dz = from_z - to_z;
    let straight = |prog: &mut Program| {
        prog.push(Step::Linear {
            to: Point3::new(p.x, p.y, to_z),
            feed: plunge_feed,
            tag,
        });
    };
    if dz <= 0.0 {
        return straight(prog);
    }
    match plunge {
        Plunge::Straight => straight(prog),
        Plunge::Helix { radius, pitch } if radius > 0.0 && pitch > 0.0 => {
            let turns = (dz / pitch).ceil().max(1.0) as usize;
            let centre = Point::new(p.x + out.0 * radius, p.y + out.1 * radius);
            let opp = Point::new(2.0 * centre.x - p.x, 2.0 * centre.y - p.y);
            let dz_half = dz / (turns as f64 * 2.0);
            let mut z = from_z;
            for _ in 0..turns {
                z -= dz_half;
                prog.push(Step::Arc {
                    end: Point3::new(opp.x, opp.y, z),
                    center: Point3::new(centre.x, centre.y, z),
                    dir: ArcDir::Ccw,
                    feed: cut_feed,
                    tag,
                });
                z -= dz_half;
                prog.push(Step::Arc {
                    end: Point3::new(p.x, p.y, z),
                    center: Point3::new(centre.x, centre.y, z),
                    dir: ArcDir::Ccw,
                    feed: cut_feed,
                    tag,
                });
            }
        }
        Plunge::Ramp { angle_deg } if angle_deg > 0.0 && angle_deg < 90.0 => emit_oscillating_ramp(
            prog,
            p,
            tan,
            from_z,
            to_z,
            angle_deg,
            f64::INFINITY,
            cut_feed,
            tag,
        ),
        Plunge::ZigZag { length, angle_deg }
            if length > 0.0 && angle_deg > 0.0 && angle_deg < 90.0 =>
        {
            emit_oscillating_ramp(prog, p, tan, from_z, to_z, angle_deg, length, cut_feed, tag)
        }
        // Bad parameters: safe fallback.
        _ => straight(prog),
    }
}

/// Oscillate along `[p, p + tan·L]` descending from `from_z` to `to_z`, ending back
/// at `p`. `max_len` caps the reach (`INFINITY` for a single V); the number of
/// out-and-back passes is chosen so each stays within the reach and the angle holds.
#[allow(clippy::too_many_arguments)]
fn emit_oscillating_ramp(
    prog: &mut Program,
    p: Point,
    tan: (f64, f64),
    from_z: f64,
    to_z: f64,
    angle_deg: f64,
    max_len: f64,
    feed: f64,
    tag: Tag,
) {
    let dz = from_z - to_z;
    let slope = angle_deg.to_radians().tan(); // dz per unit horizontal
                                              // One out-and-back V descends 2·L·slope; bound L by max_len.
    let passes = if max_len.is_finite() {
        ((dz / (2.0 * max_len * slope)).ceil().max(1.0)) as usize
    } else {
        1
    };
    let dz_v = dz / passes as f64;
    let l = dz_v / (2.0 * slope); // reach of each V
    let far = Point::new(p.x + tan.0 * l, p.y + tan.1 * l);
    let mut z = from_z;
    for _ in 0..passes {
        z -= dz_v / 2.0;
        prog.push(Step::Linear {
            to: Point3::new(far.x, far.y, z),
            feed,
            tag,
        });
        z -= dz_v / 2.0;
        prog.push(Step::Linear {
            to: Point3::new(p.x, p.y, z),
            feed,
            tag,
        });
    }
}

/// Rotate a closed loop so it begins **exactly at the point on the loop nearest
/// `start`** (part XY), preserving winding. `None` leaves the loop unchanged.
///
/// Unlike a nearest-*vertex* rotation, this projects `start` onto the closest
/// edge and, when that lands mid-edge, splits the edge — inserting the projected
/// point as the first vertex — so a Mid or Nearest object-snap really does begin
/// where the operator pointed, not at the corner beside it. A projection onto an
/// existing vertex just rotates (no split).
pub(crate) fn rotate_to_start(
    pts: &[cam_geo::Point],
    start: Option<[f64; 2]>,
) -> Vec<cam_geo::Point> {
    use cam_geo::Point;
    let Some(s) = start else {
        return pts.to_vec();
    };
    let n = pts.len();
    if n < 2 {
        return pts.to_vec();
    }
    let sp = Point::new(s[0], s[1]);
    // The closest point on any edge, and which edge it lies on.
    let (mut best_k, mut best_q, mut best_d2) = (0usize, pts[0], f64::MAX);
    for k in 0..n {
        let q = project_point_seg(sp, pts[k], pts[(k + 1) % n]);
        let d2 = q.distance_sq(sp);
        if d2 < best_d2 {
            (best_k, best_q, best_d2) = (k, q, d2);
        }
    }
    const EPS2: f64 = 1e-12;
    let kb = (best_k + 1) % n;
    let rotate_from = |i: usize| {
        let mut out = Vec::with_capacity(n);
        out.extend_from_slice(&pts[i..]);
        out.extend_from_slice(&pts[..i]);
        out
    };
    // Snap to an existing endpoint → plain rotation, no split.
    if best_q.distance_sq(pts[best_k]) <= EPS2 {
        return rotate_from(best_k);
    }
    if best_q.distance_sq(pts[kb]) <= EPS2 {
        return rotate_from(kb);
    }
    // Mid-edge: begin at the projected point, then the rest of the loop from `kb`
    // around to `best_k` (which the caller closes back to the projected point).
    let mut out = Vec::with_capacity(n + 1);
    out.push(best_q);
    out.extend(rotate_from(kb));
    out
}

/// The closest point to `p` on the segment `a→b` (clamped to the endpoints).
fn project_point_seg(p: cam_geo::Point, a: cam_geo::Point, b: cam_geo::Point) -> cam_geo::Point {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    if len2 <= f64::EPSILON {
        return a;
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
    cam_geo::Point::new(a.x + dx * t, a.y + dy * t)
}

/// The absolute Z of each stepdown pass, from just below `top` down to `depth`
/// (inclusive), never stepping more than `stepdown` at a time.
pub(crate) fn depth_levels(top: f64, depth: f64, stepdown: f64) -> Vec<f64> {
    let mut levels = Vec::new();
    if depth >= top {
        return levels;
    }
    let mut z = top;
    loop {
        z = (z - stepdown).max(depth);
        levels.push(z);
        if z <= depth {
            break;
        }
    }
    levels
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levels_step_down_to_depth() {
        assert_eq!(depth_levels(0.0, -3.0, 1.5), vec![-1.5, -3.0]);
        assert_eq!(depth_levels(0.0, -1.0, 1.5), vec![-1.0]);
        assert_eq!(depth_levels(0.0, -5.0, 2.0), vec![-2.0, -4.0, -5.0]);
        assert!(depth_levels(0.0, 1.0, 1.0).is_empty());
    }

    fn step_end_z(s: &Step) -> f64 {
        match s {
            Step::Linear { to, .. } | Step::Rapid { to, .. } => to.z,
            Step::Arc { end, .. } => end.z,
            _ => f64::NAN,
        }
    }

    fn step_end_xy(s: &Step) -> (f64, f64) {
        match s {
            Step::Linear { to, .. } | Step::Rapid { to, .. } => (to.x, to.y),
            Step::Arc { end, .. } => (end.x, end.y),
            _ => (f64::NAN, f64::NAN),
        }
    }

    #[test]
    fn every_plunge_strategy_descends_monotonically_to_exact_depth() {
        let p = Point::new(0.0, 0.0);
        let tan = (1.0, 0.0);
        let out = (0.0, 1.0);
        let tag = Tag::new(0, MoveKind::Plunge);
        for plunge in [
            Plunge::Straight,
            Plunge::Helix {
                radius: 2.0,
                pitch: 1.0,
            },
            Plunge::Ramp { angle_deg: 20.0 },
            Plunge::ZigZag {
                length: 3.0,
                angle_deg: 15.0,
            },
        ] {
            let mut prog = Program::new();
            emit_plunge(&mut prog, p, tan, out, 0.0, -5.0, plunge, 100.0, 300.0, tag);
            let zs: Vec<f64> = prog.steps.iter().map(step_end_z).collect();
            assert!(!zs.is_empty(), "{plunge:?} emitted nothing");
            for w in zs.windows(2) {
                assert!(w[1] <= w[0] + 1e-9, "{plunge:?} must descend monotonically");
            }
            assert!(
                (zs.last().unwrap() + 5.0).abs() < 1e-6,
                "{plunge:?} must reach the exact depth"
            );
            let (ex, ey) = step_end_xy(prog.steps.last().unwrap());
            assert!(
                (ex - p.x).abs() < 1e-6 && (ey - p.y).abs() < 1e-6,
                "{plunge:?} must end at the entry footprint"
            );
        }
    }

    #[test]
    fn bad_plunge_params_fall_back_to_a_straight_plunge() {
        let tag = Tag::new(0, MoveKind::Plunge);
        let mut prog = Program::new();
        // Zero radius helix / 90° ramp are invalid → one straight linear plunge.
        emit_plunge(
            &mut prog,
            Point::new(0.0, 0.0),
            (1.0, 0.0),
            (0.0, 1.0),
            0.0,
            -3.0,
            Plunge::Helix {
                radius: 0.0,
                pitch: 1.0,
            },
            100.0,
            300.0,
            tag,
        );
        assert_eq!(prog.steps.len(), 1);
        assert!((step_end_z(&prog.steps[0]) + 3.0).abs() < 1e-9);
    }

    #[test]
    fn arc_lead_endpoints_lie_on_the_tangent_circle() {
        let start = Point::new(0.0, 0.0);
        let (tan, out) = ((1.0, 0.0), (0.0, 1.0));
        let r = 3.0;
        let entry = crate::leads::lead_start_point(start, tan, out, Lead::Arc { radius: r });
        let exit = crate::leads::lead_end_point(start, tan, out, Lead::Arc { radius: r });
        let centre = Point::new(start.x + out.0 * r, start.y + out.1 * r);
        for pt in [entry, exit, start] {
            let d = (pt.x - centre.x).hypot(pt.y - centre.y);
            assert!(
                (d - r).abs() < 1e-9,
                "{pt:?} must lie on the radius-{r} circle"
            );
        }
        // A 90° arc: the chord entry→start is √2·r.
        let chord = (entry.x - start.x).hypot(entry.y - start.y);
        assert!((chord - std::f64::consts::SQRT_2 * r).abs() < 1e-9);
    }

    #[test]
    fn linear_lead_points_offset_along_the_tangent() {
        let start = Point::new(1.0, 1.0);
        let (tan, out) = ((0.0, 1.0), (1.0, 0.0));
        assert_eq!(
            crate::leads::lead_start_point(start, tan, out, Lead::Linear { length: 4.0 }),
            Point::new(1.0, -3.0)
        );
        assert_eq!(
            crate::leads::lead_end_point(start, tan, out, Lead::Linear { length: 4.0 }),
            Point::new(1.0, 5.0)
        );
    }

    #[test]
    fn outward_normal_points_away_from_a_ccw_interior() {
        let sq = [
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
        ];
        // Start tangent is +x; the interior is +y, so outward is −y.
        let out = outward_normal(&sq);
        assert!(out.1 < 0.0 && out.0.abs() < 1e-9);
    }

    #[test]
    fn rotate_to_start_begins_exactly_at_the_projected_point() {
        use cam_geo::Point;
        let sq = [
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
        ];
        // None leaves it unchanged.
        assert_eq!(rotate_to_start(&sq, None), sq.to_vec());

        // A point off the right edge (x=10) → begins exactly at its projection
        // (10, 9), splitting that edge; winding intact; loop closes back to it.
        let r = rotate_to_start(&sq, Some([9.5, 9.0]));
        assert_eq!(
            r,
            vec![
                Point::new(10.0, 9.0),
                Point::new(10.0, 10.0),
                Point::new(0.0, 10.0),
                Point::new(0.0, 0.0),
                Point::new(10.0, 0.0),
            ]
        );

        // A point at a vertex (a corner / End snap) just rotates — no split.
        let r = rotate_to_start(&sq, Some([10.0, 10.0]));
        assert_eq!(
            r,
            vec![
                Point::new(10.0, 10.0),
                Point::new(0.0, 10.0),
                Point::new(0.0, 0.0),
                Point::new(10.0, 0.0),
            ]
        );
    }
}
