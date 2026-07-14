//! The profiling strategy: follow a closed chain at a tool-radius offset, in
//! stepdown passes, down to depth.

use cam_cldata::{ArcDir, CutterComp, MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, JoinStyle, Point, Polygon};
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
        if op.depth >= env.heights.top_of_stock {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: depth {} is at or above the stock top {}; nothing to cut",
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
        // path on the contour and lets the controller (G41/G42) do the offset.
        let (signed, comp) = match op.comp {
            Comp::Computed => {
                let s = match op.side {
                    Side::Outside => tool.radius(),
                    Side::Inside => -tool.radius(),
                    Side::On => 0.0,
                };
                (s, None)
            }
            Comp::ControlLeft => (0.0, Some(CutterComp::Left(op.tool))),
            Comp::ControlRight => (0.0, Some(CutterComp::Right(op.tool))),
        };

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

        let levels = depth_levels(env.heights.top_of_stock, op.depth, op.stepdown);
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

    // Rich entry (leads and/or a non-straight plunge) takes a separate path; the
    // default (no lead + straight plunge) keeps the original, byte-stable emission.
    if op.lead_in != Lead::None || op.lead_out != Lead::None || op.plunge != Plunge::Straight {
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
    let tan_out = end_tangent(pts); // arriving back at start
    let out = outward_normal(pts); // away from the loop interior
    let link = Tag::new(op.id, MoveKind::Link);
    let lead = Tag::new(op.id, MoveKind::LeadIn);
    let cut = Tag::new(op.id, MoveKind::Cutting);
    let retract = Tag::new(op.id, MoveKind::Retract);
    let plunge_tag = Tag::new(op.id, MoveKind::Plunge);

    let entry = lead_start_point(start, tan_in, out, op.lead_in);
    let exit = lead_end_point(start, tan_out, out, op.lead_out);

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
        emit_lead(prog, entry, start, start, out, op.lead_in, z, op.feed, lead);

        if let Some(c) = comp {
            prog.push(Step::CutterComp(c));
        }
        crate::emit::cut_loop(prog, pts, op.feed, cut, z);
        if comp.is_some() {
            prog.push(Step::CutterComp(CutterComp::Off));
        }

        // Lead off the contour at depth: start → exit.
        emit_lead(prog, start, exit, start, out, op.lead_out, z, op.feed, lead);

        prog.push(Step::Rapid {
            to: Point3::new(exit.x, exit.y, h.clearance),
            tag: retract,
        });
        prev_z = z;
    }
}

/// Unit vector, or `(1,0)` if degenerate.
fn unit(x: f64, y: f64) -> (f64, f64) {
    let l = (x * x + y * y).sqrt();
    if l > 1e-12 {
        (x / l, y / l)
    } else {
        (1.0, 0.0)
    }
}

/// Unit tangent leaving the start vertex (start → pts[1]).
fn start_tangent(pts: &[Point]) -> (f64, f64) {
    unit(pts[1].x - pts[0].x, pts[1].y - pts[0].y)
}

/// Unit tangent arriving back at the start vertex (pts[last] → start).
fn end_tangent(pts: &[Point]) -> (f64, f64) {
    let n = pts.len();
    unit(pts[0].x - pts[n - 1].x, pts[0].y - pts[n - 1].y)
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

/// The outward normal at the start (away from the loop interior), for placing leads
/// and helix centres on the non-material side.
fn outward_normal(pts: &[Point]) -> (f64, f64) {
    let t = start_tangent(pts);
    // Interior is left of travel for a CCW loop, so outward is the right normal;
    // the reverse for CW.
    if signed_area2(pts) > 0.0 {
        (t.1, -t.0)
    } else {
        (-t.1, t.0)
    }
}

/// The point the tool plunges at for a lead-in (off the contour), given the start,
/// its tangent, the outward normal, and the lead. `None` plunges on the contour.
fn lead_start_point(start: Point, tan: (f64, f64), out: (f64, f64), lead: Lead) -> Point {
    match lead {
        Lead::None => start,
        Lead::Linear { length } => Point::new(start.x - tan.0 * length, start.y - tan.1 * length),
        // The far end of a 90° tangent arc: centre − tangent·r, centre = start + out·r.
        Lead::Arc { radius } => Point::new(
            start.x + (out.0 - tan.0) * radius,
            start.y + (out.1 - tan.1) * radius,
        ),
    }
}

/// The point a lead-out departs to, mirroring [`lead_start_point`] with the arrival
/// tangent.
fn lead_end_point(start: Point, tan: (f64, f64), out: (f64, f64), lead: Lead) -> Point {
    match lead {
        Lead::None => start,
        Lead::Linear { length } => Point::new(start.x + tan.0 * length, start.y + tan.1 * length),
        Lead::Arc { radius } => Point::new(
            start.x + (out.0 + tan.0) * radius,
            start.y + (out.1 + tan.1) * radius,
        ),
    }
}

/// CW/CCW of the short arc from `from` to `to` about `centre`.
fn short_arc_dir(centre: Point, from: Point, to: Point) -> ArcDir {
    let a = (from.x - centre.x, from.y - centre.y);
    let b = (to.x - centre.x, to.y - centre.y);
    if a.0 * b.1 - a.1 * b.0 > 0.0 {
        ArcDir::Ccw
    } else {
        ArcDir::Cw
    }
}

/// Emit a lead move `from → to` at height `z` (linear, or a tangent arc centred at
/// `on + out·radius`, where `on` is the on-contour endpoint). `None` emits nothing.
#[allow(clippy::too_many_arguments)]
fn emit_lead(
    prog: &mut Program,
    from: Point,
    to: Point,
    on: Point,
    out: (f64, f64),
    lead: Lead,
    z: f64,
    feed: f64,
    tag: Tag,
) {
    match lead {
        Lead::None => {}
        Lead::Linear { .. } => prog.push(Step::Linear {
            to: Point3::new(to.x, to.y, z),
            feed,
            tag,
        }),
        Lead::Arc { radius } => {
            let centre = Point::new(on.x + out.0 * radius, on.y + out.1 * radius);
            let dir = short_arc_dir(centre, from, to);
            prog.push(Step::Arc {
                end: Point3::new(to.x, to.y, z),
                center: Point3::new(centre.x, centre.y, z),
                dir,
                feed,
                tag,
            });
        }
    }
}

/// Emit the plunge from `p@from_z` down to `p@to_z` (ending at `p` in XY), per the
/// strategy. Bad parameters fall back to a straight plunge (never panic).
#[allow(clippy::too_many_arguments)]
fn emit_plunge(
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

/// Rotate a closed loop so it begins at the vertex nearest `start` (part XY).
/// `None` leaves the loop unchanged. The winding order is preserved — only the
/// starting index changes.
pub(crate) fn rotate_to_start(
    pts: &[cam_geo::Point],
    start: Option<[f64; 2]>,
) -> Vec<cam_geo::Point> {
    let Some(s) = start else {
        return pts.to_vec();
    };
    let Some((k, _)) = pts.iter().enumerate().min_by(|(_, a), (_, b)| {
        let da = (a.x - s[0]).powi(2) + (a.y - s[1]).powi(2);
        let db = (b.x - s[0]).powi(2) + (b.y - s[1]).powi(2);
        da.total_cmp(&db)
    }) else {
        return pts.to_vec();
    };
    let mut out = Vec::with_capacity(pts.len());
    out.extend_from_slice(&pts[k..]);
    out.extend_from_slice(&pts[..k]);
    out
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
        let entry = lead_start_point(start, tan, out, Lead::Arc { radius: r });
        let exit = lead_end_point(start, tan, out, Lead::Arc { radius: r });
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
            lead_start_point(start, tan, out, Lead::Linear { length: 4.0 }),
            Point::new(1.0, -3.0)
        );
        assert_eq!(
            lead_end_point(start, tan, out, Lead::Linear { length: 4.0 }),
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
    fn rotate_to_start_leads_with_nearest_vertex() {
        use cam_geo::Point;
        let sq = [
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
        ];
        // None leaves it unchanged.
        assert_eq!(rotate_to_start(&sq, None), sq.to_vec());
        // Near the third vertex (10,10) → the loop begins there, winding intact.
        let r = rotate_to_start(&sq, Some([9.5, 9.0]));
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
