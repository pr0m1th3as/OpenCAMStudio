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

        // The tool must be able to cut a vertical wall to the requested depth with a
        // *cutting* surface (see `guards`): a pointed tool has no cylindrical flank,
        // and a cut past the flute length drags the shank along the wall.
        if !crate::guards::check_side_milling(op.id, "profile", tool, op.depth, &mut diagnostics) {
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        // Each pass enters with a plunge, so the tip must cut.
        if !crate::guards::check_plunge(op.id, "profile", tool, &mut diagnostics) {
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }

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
                let levels = depth_levels(env.heights.top_of_stock, floor, op.stepdown);
                let mut program = Program::new();
                // Clear the frame through the shared engine (stay-down inside-out,
                // engagement-spaced, climb-oriented). No finishing leads — the
                // single-pass finish that follows the roughing owns the walls. The
                // finishing allowance is baked into the roughing island, so the wall
                // ring sits at the tool radius.
                let job = crate::clearing::ClearJob {
                    id: op.id,
                    radius,
                    // The finishing allowance is baked into the roughing island, so
                    // the tool-centre wall sits at the bare tool radius.
                    finish: 0.0,
                    first: radius,
                    spacing: op.stepover,
                    clearing: op.clearing,
                    plunge: op.plunge,
                    feed: op.feed,
                    plunge_feed: op.plunge_feed,
                    lead_overlap: op.lead_overlap,
                    lead_in: Lead::None,
                    lead_out: Lead::None,
                    start: op.start,
                    guard: &[],
                                    spindle: env.spindle,
                };
                match crate::clearing::clear(
                    &mut program,
                    &region,
                    &job,
                    &env.heights,
                    &levels,
                    cancel,
                ) {
                    // Degenerate frame (no rings) — fall through to a single pass.
                    Ok(0) => {}
                    Ok(_) => {
                        return StrategyResult {
                            program,
                            diagnostics,
                            cancelled: false,
                        }
                    }
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
                }
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

        // Which side of the tool-centre loop is cleared air the lead eases in from:
        // outside the loop for an outside profile, the hole interior for an inside
        // one (On has no material side — leave the lead on its default normal).
        let air_sign = if side_sign < 0.0 { -1.0 } else { 1.0 };

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
                air_sign,
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
    air_sign: f64,
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
        emit_loop_rich(prog, pts, op, h, levels, comp, air_sign);
        return;
    }

    let link = Tag::new(op.id, MoveKind::Link);
    let plunge = Tag::new(op.id, MoveKind::Plunge);
    let cut = Tag::new(op.id, MoveKind::Cutting);
    let retract = Tag::new(op.id, MoveKind::Retract);

    // Approach: rapid over the start at clearance, then down to the **retract
    // plane** — not to the stock top, where a rapid would end with no margin and
    // slightly proud stock or a small Z-zero error would mean rapiding into material.
    // The `max` keeps it never lower than the old stock-top approach.
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.clearance),
        tag: link,
    });
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.retract.max(h.top_of_stock)),
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
    air_sign: f64,
) {
    let start = pts[0];
    let tan_in = start_tangent(pts); // leaving start, into the cut
    // The cleared/air side the lead eases in from. For an outside profile that is
    // outward (away from the part); for an inside one it flips inward, into the hole
    // — otherwise the lead would swing onto the material side and gouge the wall.
    let out = {
        let o = outward_normal(pts);
        (o.0 * air_sign, o.1 * air_sign)
    };
    // An inside profile eases in from the bounded hole interior, so guard the lead
    // against overshooting it (a hole narrower than the lead drops to a plain pass);
    // an outside profile leads into open stock, which needs no bound.
    let guard: Vec<Polygon> = if air_sign < 0.0 {
        Polygon::new(Contour::new(pts.to_vec())).into_iter().collect()
    } else {
        Vec::new()
    };
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
    // The lead-off normal follows the *arrival* tangent, which differs from the
    // start's whenever the loop closes at a corner (a sharp inner offset) — reusing
    // the start normal there yields a degenerate lead. Mid-edge the two coincide, so
    // this stays byte-identical to the shortcut it replaces.
    let out_exit = {
        let o = outward_normal_at(tan_out, signed_area2(pts) > 0.0);
        (o.0 * air_sign, o.1 * air_sign)
    };

    // Drop a lead that would overshoot the hole interior to a plain pass (guard is
    // empty for outside profiles, so this leaves them untouched).
    let lead_in = crate::leads::guard_lead(&guard, start, tan_in, out, op.lead_in, true);
    let lead_out = crate::leads::guard_lead(&guard, exit_on, tan_out, out_exit, op.lead_out, false);
    let entry = crate::leads::lead_start_point(start, tan_in, out, lead_in);
    let exit = crate::leads::lead_end_point(exit_on, tan_out, out_exit, lead_out);

    // The first pass rapids down to the **retract plane**, not to the stock top:
    // ending a rapid exactly on the surface leaves no margin, so slightly proud
    // stock or a small Z-zero error means rapiding into material. Taking the higher
    // of the two is never lower than the old behaviour. Later passes return through
    // air the tool has already cut — but *that* rapid used to end exactly on the
    // previous floor, which is the identical no-margin case one pass down, so it now
    // goes through `emit::descend_to` like every other strategy's entry.
    //
    // The lift between passes is only there to reposition: with a lead-in the pass
    // ends at the lead-*out* point, somewhere else. When the two coincide — no leads,
    // the common case — there is nothing to reposition to, and the tool stays down and
    // plunges to the next level exactly as the unleaded path does. That is not merely
    // faster: a descent that never happens cannot end in metal.
    // A contour ramp starts somewhere else on the loop, and how far back depends on
    // the descent, which differs at the first level (it begins at the retract plane).
    // So it always ends the pass away from where the next one begins — the same
    // condition a lead already creates.
    let ramps = matches!(op.plunge, Plunge::Ramp { angle_deg } if angle_deg > 0.0 && angle_deg < 90.0);
    let perimeter = loop_perimeter(pts);
    // A ramped pass ends further round the loop than an unramped one (it must re-cut
    // the stretch it descended over), so its exit — and the lead-off with it — is a
    // per-level quantity rather than a fixed one.
    let repositions =
        ramps || (exit.x - entry.x).abs() > 1e-9 || (exit.y - entry.y).abs() > 1e-9;
    let mut prev_z = h.retract.max(h.top_of_stock);
    for (i, &z) in levels.iter().enumerate() {
        // **The ramp never travels through air.** Its height is measured from the top
        // of material — the stock surface on the first level, the previous floor after
        // that — not from the retract plane. Descending from the retract plane made two
        // thirds of the first pass's ramp a slow feed through nothing, and dragged the
        // tool around the contour's corners to spend the length.
        let ramp_top = prev_z.min(h.top_of_stock);
        let ramp_len = contour_ramp_len(op.plunge, ramp_top - z);

        // The ramp descends along the lead-in first, and only then along the contour:
        // the lead exists to keep the entry off the finished wall, so it is exactly
        // where a descending tool belongs. A lead long enough (or an angle steep
        // enough) to absorb the whole stepdown keeps the wall untouched entirely —
        // which is how the operator controls this.
        let ramp = ramp_len.map(|in_material| {
            let lead_path = lead_in_path(start, entry, out, lead_in);
            let lead_len = path_length(&lead_path);
            let mut path = lead_path;
            // `walk_loop` starts at the contour point the lead already reached. The
            // contour carries the **whole** material descent: the lead is flown in air.
            path.extend(walk_loop(pts, 0.0, in_material).into_iter().skip(1));
            // **The lead-in never cuts.** It starts above the material by exactly what
            // the same angle covers over its own length, so the tool descends the lead
            // through air and meets the surface *at* the contour start — then carries
            // on into the material at the same angle, one unbroken descent with the
            // air/material transition landing precisely where the cut begins. Before
            // this the lead started *on* the surface and was already cutting as it
            // swung in, which is the opposite of what a lead-in is for.
            //
            // The rise is `lead_len · tan(angle)`, and `tan(angle) = dz / in_material`
            // by construction — so no second look at the angle is needed here.
            let rise = if in_material > 1e-12 {
                lead_len * (ramp_top - z) / in_material
            } else {
                0.0
            };
            (path, in_material, ramp_top + rise)
        });

        // Where the pass begins cutting at depth: the arc position the ramp ended at.
        let cut_from = ramp.as_ref().map_or(0.0, |(_, on_contour, _)| *on_contour);
        // A full perimeter from there, so the ramped stretch is re-machined, plus the
        // operator's overlap.
        let (loop_pts, exit_on, tan_out) = if ramps {
            let walked = walk_loop(pts, cut_from, perimeter + op.lead_overlap);
            let last = walked[walked.len() - 1];
            let prev = walked[walked.len().saturating_sub(2)];
            (walked, last, unit(last.x - prev.x, last.y - prev.y))
        } else {
            (loop_pts.clone(), exit_on, tan_out)
        };
        let out_exit = {
            let o = outward_normal_at(tan_out, signed_area2(pts) > 0.0);
            (o.0 * air_sign, o.1 * air_sign)
        };
        let lead_out = crate::leads::guard_lead(&guard, exit_on, tan_out, out_exit, op.lead_out, false);
        let exit = crate::leads::lead_end_point(exit_on, tan_out, out_exit, lead_out);

        // The tool comes down where the entry begins — the lead-in's own start, which
        // when ramping now sits *above* the cutting plane by exactly the ramp's height,
        // so the descent that follows lands on the contour at depth.
        //
        // A ramp stops at the top of material and feeds from there; every other plunge
        // style still comes down to `prev_z` and descends from it, unchanged.
        let arrive_at = match &ramp {
            // The top of the ramp, which for a lead-in sits *above* the material.
            Some((_, _, top)) => top.max(z),
            None => prev_z,
        };
        if i == 0 || repositions {
            prog.push(Step::Rapid {
                to: Point3::new(entry.x, entry.y, h.clearance),
                tag: link,
            });
            crate::emit::descend_to(prog, entry, arrive_at, h, op.feed, op.id);
        }

        if let Some((path, _, top)) = &ramp {
            emit_descending_path(prog, path, *top, z, op.feed, plunge_tag);
        } else {
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
            crate::leads::emit_lead(prog, entry, start, start, out, lead_in, z, op.feed, lead);
        }

        if let Some(c) = comp {
            prog.push(Step::CutterComp(c));
        }
        crate::emit::cut_polyline(prog, &loop_pts, op.feed, cut, z);
        if comp.is_some() {
            prog.push(Step::CutterComp(CutterComp::Off));
        }

        // Lead off the contour at depth — **flat**. Only the lead-in descends.
        crate::leads::emit_lead(prog, exit_on, exit, exit_on, out_exit, lead_out, z, op.feed, lead);

        // Lift only when the next pass has somewhere else to start from; the final
        // retract is unconditional (the operation must leave the tool clear).
        if repositions || i + 1 == levels.len() {
            prog.push(Step::Rapid {
                to: Point3::new(exit.x, exit.y, h.clearance),
                tag: retract,
            });
        }
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
        // `Plunge::Ramp` is deliberately absent: it is not a point entry. It travels
        // along the contour and therefore *starts* somewhere else, which is a fact the
        // caller has to know before it rapids anywhere — see `contour_ramp_len` and
        // `approach_along_loop`. Callers that own a contour handle it before reaching
        // here; the fallthrough below keeps a caller that does not from emitting
        // nothing at all.
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
/// at `p`. `max_len` caps the reach; the number of out-and-back passes is chosen so
/// each stays within the reach and the angle holds.
///
/// [`Plunge::ZigZag`] alone: the oscillation exists for the slot too narrow to ramp
/// along, and nothing else. [`Plunge::Ramp`] used to come through here with an
/// infinite reach — one V along the straight start tangent — which is why its
/// documented "along the toolpath" was never true.
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

/// How far the tool must travel along the contour to descend `dz` at the ramp's
/// angle. `None` for every plunge style that is not a contour ramp, and for a
/// degenerate angle or a non-descent — both of which fall back to the plain plunge,
/// exactly as an invalid helix always has.
pub(crate) fn contour_ramp_len(plunge: Plunge, dz: f64) -> Option<f64> {
    match plunge {
        Plunge::Ramp { angle_deg } if angle_deg > 0.0 && angle_deg < 90.0 && dz > 0.0 => {
            Some(dz / angle_deg.to_radians().tan())
        }
        _ => None,
    }
}

/// Most laps a contour ramp may take around a loop before it is steepened to fit.
///
/// A shallow angle on a small loop is otherwise unbounded: 0.5° needs 115 mm of
/// travel per millimetre of descent, so a 20 mm circle takes nearly two laps per
/// millimetre and the emitted vertex count grows with it. Beyond the cap the ramp
/// descends over 32 laps instead of the requested length, which is *steeper* than
/// asked — the safe direction to err, since the alternative is an unbounded buffer.
pub(crate) const MAX_RAMP_LAPS: usize = 32;

/// Total length of an open polyline.
pub(crate) fn path_length(pts: &[Point]) -> f64 {
    pts.windows(2).map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y)).sum()
}

/// The perimeter of the closed loop through `pts`.
pub(crate) fn loop_perimeter(pts: &[Point]) -> f64 {
    let n = pts.len();
    (0..n)
        .map(|i| {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            (b.x - a.x).hypot(b.y - a.y)
        })
        .sum()
}

/// Walk the closed loop `pts` **forward** from arc position `from`, for `len` mm,
/// wrapping as often as needed. The first returned point is the position at `from`.
///
/// One walker for both halves of a ramped entry: the stretch the ramp descends over,
/// and the full-perimeter cut that follows it from wherever the ramp ended.
pub(crate) fn walk_loop(pts: &[Point], from: f64, len: f64) -> Vec<Point> {
    let n = pts.len();
    if n < 2 {
        return pts.to_vec();
    }
    let perim = loop_perimeter(pts);
    if perim <= 1e-12 {
        return vec![pts[0]];
    }
    // Where `from` lands: the edge it falls on, and how far along that edge.
    let mut start_at = from.rem_euclid(perim);
    let mut i = 0usize;
    loop {
        let e = (pts[(i + 1) % n].x - pts[i].x).hypot(pts[(i + 1) % n].y - pts[i].y);
        if start_at <= e || i + 1 == n {
            break;
        }
        start_at -= e;
        i += 1;
    }
    let at = |i: usize, d: f64| {
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        let e = (b.x - a.x).hypot(b.y - a.y);
        if e <= 1e-12 {
            a
        } else {
            Point::new(a.x + (b.x - a.x) * d / e, a.y + (b.y - a.y) * d / e)
        }
    };

    let mut out = vec![at(i, start_at)];
    let mut remaining = len;
    let mut edge_left = {
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        (b.x - a.x).hypot(b.y - a.y) - start_at
    };
    // Bounded like the ramp itself: a caller asking for many perimeters gets many
    // laps, but never an unbounded buffer.
    for _ in 0..n.saturating_mul(MAX_RAMP_LAPS) + 2 {
        if remaining <= 0.0 {
            break;
        }
        if edge_left > remaining {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            let e = (b.x - a.x).hypot(b.y - a.y);
            let d = e - edge_left + remaining;
            out.push(at(i, d));
            break;
        }
        remaining -= edge_left;
        i = (i + 1) % n;
        out.push(pts[i]);
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        edge_left = (b.x - a.x).hypot(b.y - a.y);
    }
    out
}

/// The lead-in as a path **from the lead's start point to the contour**, inclusive of
/// both ends — the reverse of [`leads::lead_samples`], which samples outward from the
/// contour. Empty lead ⇒ just the contour point.
///
/// The ramp descends along this before it touches the contour, which is the whole
/// point: the lead exists to keep the entry off the finished wall, so it is where a
/// descending tool belongs.
pub(crate) fn lead_in_path(start: Point, entry: Point, out: (f64, f64), lead: Lead) -> Vec<Point> {
    let mut pts = crate::leads::lead_samples(start, entry, out, lead);
    pts.reverse();
    match pts.last() {
        Some(p) if (p.x - start.x).abs() < 1e-9 && (p.y - start.y).abs() < 1e-9 => {}
        _ => pts.push(start),
    }
    if pts.is_empty() {
        pts.push(start);
    }
    pts
}

/// The stretch of the closed loop `pts` that **arrives at `pts[0]`** after `len` mm
/// of travel, found by walking backwards from the start and wrapping as often as
/// needed. Returned in travel order, so the last point is always `pts[0]`.
///
/// Backwards, so that the ramp ends where the pass begins. The stretch it leaves
/// sloped is then the loop's own final stretch, which the pass re-machines at full
/// depth as it closes — no extra motion, and nothing stranded. A ramp running
/// *forward* from the start would leave that wedge uncut, and would move the point
/// the cut begins at, which is the operator's (possibly snapped) choice.
pub(crate) fn approach_along_loop(pts: &[Point], len: f64) -> Vec<Point> {
    let n = pts.len();
    if n < 2 || len.is_nan() || len <= 0.0 {
        return vec![pts[0]];
    }
    // Collected against the direction of travel, then reversed.
    let mut back = vec![pts[0]];
    let mut remaining = len;
    let mut from = pts[0];
    let mut i = 0usize;
    // Bounded by construction rather than by the arithmetic working out: see
    // MAX_RAMP_LAPS. One step per edge, so `n` steps is one lap.
    for _ in 0..n.saturating_mul(MAX_RAMP_LAPS) {
        if remaining <= 0.0 {
            break;
        }
        let prev = (i + n - 1) % n;
        let to = pts[prev];
        let (dx, dy) = (to.x - from.x, to.y - from.y);
        let seg = (dx * dx + dy * dy).sqrt();
        if seg <= 1e-12 {
            i = prev; // coincident vertices contribute no length
            continue;
        }
        if seg >= remaining {
            let t = remaining / seg;
            back.push(Point::new(from.x + dx * t, from.y + dy * t));
            break;
        }
        remaining -= seg;
        back.push(to);
        from = to;
        i = prev;
    }
    back.reverse();
    back
}

/// The first `len` mm of an **open** path, from `path[0]` forward, truncated at the
/// path's end. Unlike [`approach_along_loop`] there is nothing to wrap around, so a
/// path shorter than `len` yields all of it — the ramp then descends over what
/// travel exists, which is steeper than asked and never a vertical drop.
pub(crate) fn advance_along_path(path: &[Point], len: f64) -> Vec<Point> {
    let mut out = vec![path[0]];
    if path.len() < 2 || len.is_nan() || len <= 0.0 {
        return out;
    }
    let mut remaining = len;
    for w in path.windows(2) {
        let (dx, dy) = (w[1].x - w[0].x, w[1].y - w[0].y);
        let seg = (dx * dx + dy * dy).sqrt();
        if seg <= 1e-12 {
            continue;
        }
        if seg >= remaining {
            let t = remaining / seg;
            out.push(Point::new(w[0].x + dx * t, w[0].y + dy * t));
            return out;
        }
        remaining -= seg;
        out.push(w[1]);
    }
    out
}

/// Descend along an **open** path and retrace it: the tool ramps forward from
/// `path[0]` to depth, then returns along the same stretch at full depth, ending
/// where it began.
///
/// The return leg is not decoration, and it is not the oscillating entry this
/// replaced. A closed loop needs no return because the pass ends where the ramp
/// began, so the loop's last stretch re-machines the slope for free. An open path
/// never comes back, so the wedge would simply be left standing at the entry.
/// Retracing at depth is the cheapest honest answer, and it still follows the
/// toolpath rather than cutting across it.
pub(crate) fn emit_open_ramp(
    prog: &mut Program,
    path: &[Point],
    from_z: f64,
    to_z: f64,
    feed: f64,
    tag: Tag,
) {
    emit_descending_path(prog, path, from_z, to_z, feed, tag);
    // Back along the same points at depth. Skipping the last (we are on it) and
    // walking in reverse leaves the tool at `path[0]`, which is the entry contract
    // every other plunge strategy keeps.
    for p in path.iter().rev().skip(1) {
        prog.push(Step::Linear {
            to: Point3::new(p.x, p.y, to_z),
            feed,
            tag,
        });
    }
}

/// Emit `path` as fed moves descending linearly **in arc length** from `from_z`,
/// reaching exactly `to_z` at the last point. The tool must already sit at `path[0]`.
///
/// Linear in arc length, not in vertex index: a loop's edges differ in length, and
/// interpolating per vertex would tilt the ramp differently on every edge.
pub(crate) fn emit_descending_path(
    prog: &mut Program,
    path: &[Point],
    from_z: f64,
    to_z: f64,
    feed: f64,
    tag: Tag,
) {
    let seg_len = |a: Point, b: Point| ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
    let total: f64 = path.windows(2).map(|w| seg_len(w[0], w[1])).sum();
    let last = path[path.len() - 1];
    if total.is_nan() || total <= 1e-12 {
        // Degenerate path: there is nowhere to ramp, so drop straight to depth rather
        // than emit a descent that never descends.
        prog.push(Step::Linear {
            to: Point3::new(last.x, last.y, to_z),
            feed,
            tag,
        });
        return;
    }
    let mut acc = 0.0;
    let n = path.len();
    for (k, w) in path.windows(2).enumerate() {
        acc += seg_len(w[0], w[1]);
        // The final vertex takes `to_z` outright: the running sum is within a few ULP
        // of `total`, and "reaches the exact depth" is a property worth not rounding.
        let z = if k + 2 == n {
            to_z
        } else {
            from_z + (to_z - from_z) * (acc / total)
        };
        prog.push(Step::Linear {
            to: Point3::new(w[1].x, w[1].y, z),
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

    /// `Plunge::Ramp` is absent by design, not by oversight: it is the one entry that
    /// does *not* end at the footprint it began at, because it travels along the
    /// contour. Its equivalent properties are asserted in the contour-ramp tests below.
    #[test]
    fn every_point_plunge_strategy_descends_monotonically_to_exact_depth() {
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

    /// A 100 mm square loop, used by the contour-ramp tests below. Perimeter 400 mm.
    fn square() -> Vec<Point> {
        vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0),
            Point::new(100.0, 100.0),
            Point::new(0.0, 100.0),
        ]
    }

    fn path_len(p: &[Point]) -> f64 {
        p.windows(2)
            .map(|w| ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt())
            .sum()
    }

    #[test]
    fn the_contour_ramp_travels_the_length_its_angle_requires() {
        // 5 mm of descent at 45° is 5 mm of travel; at ~26.57° (tan = 0.5) it is 10.
        assert!(
            (contour_ramp_len(Plunge::Ramp { angle_deg: 45.0 }, 5.0).unwrap() - 5.0).abs() < 1e-9
        );
        let shallow = contour_ramp_len(Plunge::Ramp { angle_deg: 26.565_051_2 }, 5.0).unwrap();
        assert!((shallow - 10.0).abs() < 1e-6, "got {shallow}");
        // Not a ramp, a degenerate angle, or no descent to make: the plain plunge.
        assert!(contour_ramp_len(Plunge::Straight, 5.0).is_none());
        assert!(contour_ramp_len(Plunge::Ramp { angle_deg: 90.0 }, 5.0).is_none());
        assert!(contour_ramp_len(Plunge::Ramp { angle_deg: 0.0 }, 5.0).is_none());
        assert!(contour_ramp_len(Plunge::Ramp { angle_deg: 20.0 }, 0.0).is_none());
    }

    /// The ramp must arrive **at** the pass's start point, entering the loop behind it.
    /// That direction is the whole reason no extra motion is needed to clean the slope:
    /// the wedge it leaves is the loop's final stretch, which the pass then re-cuts.
    #[test]
    fn the_contour_ramp_arrives_at_the_start_from_behind_it() {
        let pts = square();
        let path = approach_along_loop(&pts, 30.0);
        let last = path[path.len() - 1];
        assert!(
            (last.x - pts[0].x).abs() < 1e-9 && (last.y - pts[0].y).abs() < 1e-9,
            "the ramp must end where the pass begins, got {last:?}"
        );
        assert!((path_len(&path) - 30.0).abs() < 1e-9, "got {}", path_len(&path));
        // Backwards from (0,0) is along the *last* edge, (0,100) → (0,0): so the ramp
        // starts 30 mm up the left-hand side, not 30 mm along the bottom.
        assert!(
            (path[0].x).abs() < 1e-9 && (path[0].y - 30.0).abs() < 1e-9,
            "ramp entered forward, not behind the start: {:?}",
            path[0]
        );
    }

    #[test]
    fn a_contour_ramp_longer_than_the_loop_wraps_instead_of_giving_up() {
        let pts = square();
        // 950 mm on a 400 mm perimeter: two full laps and 150 mm.
        let path = approach_along_loop(&pts, 950.0);
        assert!((path_len(&path) - 950.0).abs() < 1e-6, "got {}", path_len(&path));
        let last = path[path.len() - 1];
        assert!((last.x - pts[0].x).abs() < 1e-9 && (last.y - pts[0].y).abs() < 1e-9);
    }

    /// A shallow enough angle asks for unbounded travel. The cap steepens the ramp
    /// rather than sizing a buffer off the arithmetic — the same lesson as the dashed
    /// backplot's walk: bound the output, do not trust the numbers to stay sane.
    #[test]
    fn an_absurdly_shallow_ramp_is_bounded_by_the_lap_cap() {
        let pts = square();
        let path = approach_along_loop(&pts, 1.0e9);
        assert!(
            path_len(&path) <= 400.0 * MAX_RAMP_LAPS as f64 + 1e-6,
            "ramp ran {} mm, past the {MAX_RAMP_LAPS}-lap cap",
            path_len(&path)
        );
        assert!(path.len() <= 4 * MAX_RAMP_LAPS + 2, "{} vertices", path.len());
        let last = path[path.len() - 1];
        assert!((last.x - pts[0].x).abs() < 1e-9 && (last.y - pts[0].y).abs() < 1e-9);
    }

    #[test]
    fn a_descending_path_falls_monotonically_and_reaches_the_exact_depth() {
        let pts = square();
        let path = approach_along_loop(&pts, 30.0);
        let mut prog = Program::new();
        emit_descending_path(
            &mut prog,
            &path,
            0.0,
            -5.0,
            300.0,
            Tag::new(0, MoveKind::Plunge),
        );
        let zs: Vec<f64> = prog.steps.iter().map(step_end_z).collect();
        assert_eq!(zs.len(), path.len() - 1, "one move per segment");
        for w in zs.windows(2) {
            assert!(w[1] <= w[0] + 1e-12, "the ramp must descend monotonically");
        }
        assert!(
            (zs.last().unwrap() + 5.0).abs() < 1e-12,
            "must reach exact depth, got {}",
            zs.last().unwrap()
        );
        // Descent is linear in arc length: halfway along is halfway down.
        let half = prog
            .steps
            .iter()
            .map(step_end_z)
            .find(|_| true)
            .expect("at least one move");
        assert!(half < 0.0, "the first move must already be descending");
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

    use cam_model::{Tool, ToolKind};

    fn inner_op(hole: f64, lead_r: f64, start: Option<[f64; 2]>) -> ProfileOp {
        ProfileOp {
            spindle_rpm: 0.0,
            work_offset: 1,
            clearing: cam_model::Clearing::default(),
            id: 0,
            tool: 1,
            chain: Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(hole, 0.0),
                Point::new(hole, hole),
                Point::new(0.0, hole),
            ]),
            side: Side::Inside,
            comp: Comp::Computed,
            depth: 2.0,
            stepdown: 2.0,
            offset: 0.0,
            stepover: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            plunge: Plunge::Straight,
            start,
            lead_in: Lead::Arc { radius: lead_r },
            lead_out: Lead::Arc { radius: lead_r },
            lead_overlap: 0.0,
        }
    }

    fn run_inner(op: ProfileOp) -> StrategyResult {
        let ts = [Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }];
        let env = crate::JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &ts,
            stock: None,
            spindle: cam_cldata::SpindleDir::Cw,
        };
        ProfileStrategy::new(op).compute(&env, &crate::CancelToken::new())
    }

    fn lead_arcs(r: &StrategyResult) -> Vec<(Point, Point)> {
        let mut pos = Point::new(0.0, 0.0);
        let mut arcs = Vec::new();
        for s in r.program.steps() {
            match s {
                Step::Rapid { to, .. } | Step::Linear { to, .. } => pos = Point::new(to.x, to.y),
                Step::Arc { end, tag, .. } => {
                    if tag.kind == MoveKind::LeadIn {
                        arcs.push((pos, Point::new(end.x, end.y)));
                    }
                    pos = Point::new(end.x, end.y);
                }
                _ => {}
            }
        }
        arcs
    }

    #[test]
    fn inner_profile_leads_ease_in_from_the_hole_not_the_wall() {
        // ⌀40 square hole, ⌀6 tool: the tool-centre wall loop is [3,37], hole centre
        // (20,20). Starting mid-edge (top edge at x=20), the lead must ease in from the
        // interior — historically it swung the wrong way, out toward the wall (once as
        // far as x=−3, into solid material).
        let r = run_inner(inner_op(40.0, 3.0, Some([20.0, 40.0])));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        let arcs = lead_arcs(&r);
        assert_eq!(arcs.len(), 2, "a lead-in and a lead-out");
        let d = |p: Point| (p.x - 20.0).hypot(p.y - 20.0);
        for (a, b) in &arcs {
            for p in [a, b] {
                assert!(
                    (-1e-6..=40.0 + 1e-6).contains(&p.x) && (-1e-6..=40.0 + 1e-6).contains(&p.y),
                    "lead point ({}, {}) left the hole",
                    p.x,
                    p.y
                );
            }
        }
        let (entry, wall) = arcs[0];
        assert!(
            d(entry) < d(wall),
            "inner lead must come from the interior: entry {:?} wall {:?}",
            (entry.x, entry.y),
            (wall.x, wall.y)
        );
    }

    #[test]
    fn inner_profile_lead_too_big_for_the_hole_is_dropped() {
        // ⌀8 hole, ⌀6 tool: the wall loop is [3,5] — only 2 mm to the centre — so a
        // radius-3 arc lead can't fit even mid-edge; it must drop to a plain pass,
        // never overshoot the wall.
        let r = run_inner(inner_op(8.0, 3.0, Some([4.0, 8.0])));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        assert!(
            lead_arcs(&r).is_empty(),
            "an oversized inner lead must be dropped: {:?}",
            lead_arcs(&r)
        );
        for s in r.program.steps() {
            let p = match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some((to.x, to.y)),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => Some((end.x, end.y)),
                _ => None,
            };
            if let Some((x, y)) = p {
                assert!(
                    (-1e-6..=8.0 + 1e-6).contains(&x) && (-1e-6..=8.0 + 1e-6).contains(&y),
                    "cut left the hole at ({x}, {y})"
                );
            }
        }
    }

    /// An outside profile set up for radial roughing: a square part (the chain) that
    /// the stock frames, with a stepover and (optionally) an engagement cap.
    fn outside_rough_op(engagement: f64) -> ProfileOp {
        ProfileOp {
            spindle_rpm: 0.0,
            work_offset: 1,
            clearing: cam_model::Clearing { engagement, climb: true },
            id: 0,
            tool: 1,
            // A 20×20 part; the stock is a 60×60 block, so the frame around it is the
            // same annulus the adaptive frame clearer certifies.
            chain: Contour::new(vec![
                Point::new(20.0, 20.0),
                Point::new(40.0, 20.0),
                Point::new(40.0, 40.0),
                Point::new(20.0, 40.0),
            ]),
            side: Side::Outside,
            comp: Comp::Computed,
            depth: 2.0,
            stepdown: 2.0, // one depth level
            offset: 0.0,
            stepover: 4.0,
            feed: 300.0,
            plunge_feed: 100.0,
            plunge: Plunge::Straight,
            start: None,
            lead_in: Lead::None,
            lead_out: Lead::None,
            lead_overlap: 0.0,
        }
    }

    fn run_outside_rough(op: ProfileOp) -> StrategyResult {
        let ts = [Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }];
        let env = crate::JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &ts,
            stock: Some(([0.0, 0.0], [60.0, 60.0])),
            spindle: cam_cldata::SpindleDir::Cw,
        };
        ProfileStrategy::new(op).compute(&env, &crate::CancelToken::new())
    }

    /// **The ramp never travels through air.** Its height is the *material* it has to
    /// get through — one stepdown — not the drop from the retract plane. Measured
    /// rather than asserted about: the ramp's own length is `dz / tan(angle)`, so a
    /// ramp that started at the retract plane would be visibly longer on the first
    /// level than on the second.
    #[test]
    fn the_first_levels_ramp_is_no_longer_than_any_others() {
        let mut op = outside_rough_op(0.0);
        op.stepover = 0.0;
        op.depth = 2.0;
        op.stepdown = 1.0; // two levels, both one stepdown of material
        op.plunge = Plunge::Ramp { angle_deg: 5.0 };
        let r = run_outside_rough(op);

        // Length of each run of consecutive descending Plunge moves.
        let mut pos = Point3::new(0.0, 0.0, 0.0);
        let mut runs: Vec<f64> = Vec::new();
        let mut cur = 0.0;
        for st in r.program.steps() {
            let (to, ramping) = match st {
                Step::Linear { to, tag, .. } => (*to, tag.kind == MoveKind::Plunge),
                Step::Rapid { to, .. } => (*to, false),
                Step::Arc { end, .. } => (*end, false),
                _ => continue,
            };
            if ramping && to.z < pos.z - 1e-9 {
                cur += (to.x - pos.x).hypot(to.y - pos.y);
            } else if cur > 0.0 {
                runs.push(cur);
                cur = 0.0;
            }
            pos = to;
        }
        if cur > 0.0 {
            runs.push(cur);
        }
        assert!(runs.len() >= 2, "expected a ramp per level, got {runs:?}");
        let want = 1.0 / 5.0_f64.to_radians().tan(); // one stepdown at 5°
        for (i, l) in runs.iter().enumerate() {
            assert!(
                (l - want).abs() < 0.5,
                "level {i} ramped {l:.2} mm; one stepdown at 5° is {want:.2} mm. A ramp \
                 starting at the retract plane would be far longer on the first level."
            );
        }
    }

    /// The ramp descends along the **lead-in** before it touches the contour — that is
    /// where a descending tool belongs, because the lead exists to keep the entry off
    /// the finished wall. With a lead long enough to absorb the whole stepdown, the
    /// wall is never touched by a descending tool at all.
    #[test]
    fn a_ramp_descends_down_the_lead_in_before_the_contour() {
        let mut op = outside_rough_op(0.0);
        op.stepover = 0.0;
        op.depth = 0.2;
        op.stepdown = 0.2; // a shallow step, so a modest lead can absorb it
        op.plunge = Plunge::Ramp { angle_deg: 5.0 };
        op.lead_in = Lead::Arc { radius: 3.0 };
        op.lead_out = Lead::Arc { radius: 3.0 };
        let r = run_outside_rough(op);

        // The contour's own vertices, to tell "on the wall" from "on the lead".
        let on_contour = |p: Point3| {
            let pts = [
                (20.0, 20.0),
                (40.0, 20.0),
                (40.0, 40.0),
                (20.0, 40.0),
            ];
            pts.iter().any(|&(x, y)| (p.x - x).abs() < 1e-6 && (p.y - y).abs() < 1e-6)
        };
        let first_plunge = r
            .program
            .steps()
            .iter()
            .position(|st| matches!(st, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge))
            .expect("a ramp");
        // Nothing before the ramp may already be at depth, and the ramp itself must
        // begin away from the contour's corners (it is on the lead arc).
        let before: Vec<_> = r.program.steps()[..first_plunge].to_vec();
        assert!(
            !before.iter().any(|st| matches!(st,
                Step::Linear { to, .. } | Step::Rapid { to, .. } if to.z < -1e-9)),
            "nothing may reach cutting depth before the ramp"
        );
        assert!(!on_contour(match &r.program.steps()[first_plunge] {
            Step::Linear { to, .. } => *to,
            _ => unreachable!(),
        }), "the ramp's first move should be along the lead, not the contour");
    }

    /// **The lead-in flies through air and lands on the surface at the contour start.**
    ///
    /// It starts above the material by `lead_length · tan(angle)` and reaches the
    /// material top exactly where the contour begins — then carries on into the
    /// material at the same angle. One unbroken descent, with the air/material
    /// transition landing precisely where the cut starts. Before this the lead began
    /// *on* the surface and was already cutting as it swung in, which is the opposite
    /// of what a lead-in is for (Andreas, 2026-08-01).
    #[test]
    fn the_lead_in_descends_through_air_and_meets_the_surface_at_the_contour() {
        let mut op = outside_rough_op(0.0);
        op.stepover = 0.0;
        op.depth = 1.0;
        op.stepdown = 1.0;
        op.plunge = Plunge::Ramp { angle_deg: 5.0 };
        op.lead_in = Lead::Arc { radius: 3.0 };
        op.lead_out = Lead::None;
        let r = run_outside_rough(op);

        // The ramp: the run of descending Plunge moves.
        let ramp: Vec<Point3> = r
            .program
            .steps()
            .iter()
            .filter_map(|st| match st {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => Some(*to),
                _ => None,
            })
            .collect();
        assert!(!ramp.is_empty(), "no ramp emitted");

        let tan = 5.0_f64.to_radians().tan();
        let lead_len = std::f64::consts::FRAC_PI_2 * 3.0; // a quarter arc of radius 3
        let top_of_stock = 0.0;

        // It starts above the stock by the lead's own rise…
        let first = ramp[0];
        assert!(
            first.z > top_of_stock - lead_len * tan,
            "the ramp began at {:.4}, not above the material",
            first.z
        );

        // …and the point where it crosses the stock top is the contour start, which is
        // where the lead ends. The chain's start corner is (20,20); with an outside
        // profile at r=3 the contour start sits 3 mm out on -Y.
        let crossing = ramp
            .windows(2)
            .find(|w| w[0].z > top_of_stock && w[1].z <= top_of_stock + 1e-9)
            .map(|w| w[1]);
        let crossing = crossing.expect("the ramp must cross the stock top");
        assert!(
            (crossing.z - top_of_stock).abs() < 5e-3,
            "it crossed the surface at Z {:.4}, not at the stock top",
            crossing.z
        );

        // And the whole material descent happens on the contour: from the surface to
        // full depth is one stepdown, so the contour portion is stepdown/tan.
        let in_material: f64 = ramp
            .windows(2)
            .filter(|w| w[1].z < top_of_stock)
            .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
            .sum();
        let want = 1.0 / tan;
        assert!(
            (in_material - want).abs() < 0.6,
            "the in-material ramp ran {in_material:.2} mm; one stepdown at 5° is \
             {want:.2} mm. If it is short, the lead is wrongly absorbing part of the cut."
        );
    }

    /// Only the lead-**in** descends. The lead-out stays at the cutting plane.
    #[test]
    fn the_lead_out_stays_flat_at_the_cutting_plane() {
        let mut op = outside_rough_op(0.0);
        op.stepover = 0.0;
        op.depth = 1.0;
        op.stepdown = 1.0;
        op.plunge = Plunge::Ramp { angle_deg: 5.0 };
        op.lead_in = Lead::Arc { radius: 3.0 };
        op.lead_out = Lead::Arc { radius: 3.0 };
        let r = run_outside_rough(op);
        for st in r.program.steps() {
            if let Step::Arc { end, tag, .. } = st {
                if tag.kind == MoveKind::LeadIn {
                    assert!(
                        (end.z + 1.0).abs() < 1e-9,
                        "a lead arc ended at {} — the lead-out must stay at depth",
                        end.z
                    );
                }
            }
        }
    }

    /// The point of the whole exercise: with a ramp selected, the tool must never drop
    /// straight down into material. A vertical `Plunge` move is exactly the entry a
    /// ramp exists to avoid, and it is what a silent fallback would produce.
    #[test]
    fn a_contour_ramp_never_drops_vertically_into_material() {
        let mut op = outside_rough_op(0.0);
        op.stepdown = 0.5; // four levels, so later entries start from a cut floor
        op.plunge = Plunge::Ramp { angle_deg: 15.0 };
        let r = run_outside_rough(op);
        assert!(
            r.diagnostics
                .iter()
                .all(|d| d.severity != crate::Severity::Error),
            "{:?}",
            r.diagnostics
        );

        let mut pos = Point3::new(0.0, 0.0, 0.0);
        let mut vertical = 0;
        let mut ramped = 0;
        for s in r.program.steps() {
            let (to, kind) = match s {
                Step::Linear { to, tag, .. } => (*to, Some(tag.kind)),
                Step::Rapid { to, .. } => (*to, None),
                Step::Arc { end, .. } => (*end, None),
                _ => continue,
            };
            if kind == Some(MoveKind::Plunge) {
                let flat = (to.x - pos.x).hypot(to.y - pos.y);
                if to.z < pos.z - 1e-9 {
                    if flat <= 1e-9 {
                        vertical += 1;
                    } else {
                        ramped += 1;
                    }
                }
            }
            pos = to;
        }
        assert_eq!(vertical, 0, "a ramped profile emitted {vertical} vertical plunges");
        assert!(ramped > 0, "no descending ramp moves were emitted at all");
    }

    /// The ramp arrives at the pass's start, so the loop's own final stretch re-cuts
    /// the slope. Which means the cut still covers the whole contour — a ramp must not
    /// quietly shorten the pass.
    #[test]
    fn a_ramped_pass_still_cuts_the_whole_contour() {
        let mut straight_op = outside_rough_op(0.0);
        straight_op.stepover = 0.0; // a plain contour pass, no roughing rings
        let mut ramp_op = straight_op.clone();
        ramp_op.plunge = Plunge::Ramp { angle_deg: 15.0 };

        let cut_len = |r: &StrategyResult| -> f64 {
            let mut pos = Point3::new(0.0, 0.0, 0.0);
            let mut total = 0.0;
            for s in r.program.steps() {
                let to = match s {
                    Step::Linear { to, tag, .. } => {
                        if tag.kind == MoveKind::Cutting {
                            total += (to.x - pos.x).hypot(to.y - pos.y);
                        }
                        *to
                    }
                    Step::Rapid { to, .. } => *to,
                    Step::Arc { end, .. } => *end,
                    _ => continue,
                };
                pos = to;
            }
            total
        };
        let plain = cut_len(&run_outside_rough(straight_op));
        let ramped = cut_len(&run_outside_rough(ramp_op));
        assert!(
            (ramped - plain).abs() < 1e-6,
            "the ramp changed the cut length: {plain} plain vs {ramped} ramped"
        );
    }

    fn plunge_count(r: &StrategyResult) -> usize {
        r.program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge))
            .count()
    }

    fn cut_count(r: &StrategyResult) -> usize {
        r.program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Cutting))
            .count()
    }

    #[test]
    fn outside_roughing_honours_the_engagement_cap_by_tightening_the_ring_spacing() {
        // Outside roughing frames the part with the stock as an annulus. It used to route
        // through the adaptive frame path; that path was raster-gated and the raster is
        // blind to slots (see `clearing::clear`), so it now clears concentrically like
        // everything else.
        //
        // The engagement cap is **not** inert: `ClearJob::effective_spacing` caps the ring
        // spacing at it, which bounds the radial width of cut on a straight wall. So an
        // engagement-capped run must still differ observably from the uncapped baseline —
        // it just differs by having more, tighter rings rather than by being adaptive.
        let concentric = run_outside_rough(outside_rough_op(0.0));
        assert!(!concentric.has_errors(), "{:?}", concentric.diagnostics);
        let adaptive = run_outside_rough(outside_rough_op(2.0));
        assert!(!adaptive.has_errors(), "{:?}", adaptive.diagnostics);

        assert!(cut_count(&adaptive) > 50, "the frame cuts, got {}", cut_count(&adaptive));
        assert_ne!(
            (plunge_count(&adaptive), cut_count(&adaptive)),
            (plunge_count(&concentric), cut_count(&concentric)),
            "engagement>0 must change the path — identical output would mean it fell \
             back to concentric instead of certifying the adaptive frame"
        );
    }
}
