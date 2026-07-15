//! The thread-milling strategy: cut a thread at each hole/boss centre by
//! **helically interpolating** a single-form thread mill — one continuous helix
//! of `length / pitch` turns, advancing exactly `pitch` in Z per revolution. The
//! tool centre orbits at `major_radius ∓ tool_radius` so the cutting edge lands
//! on the major diameter.
//!
//! ## Direction (the correctness-critical part)
//!
//! Two independent choices define the motion: the **orbit direction** (G2/G3) and
//! the **Z travel sense** (cut up vs. down).
//!
//! - **Orbit direction** sets climb vs. conventional. Assuming a standard `M3`
//!   (CW-from-`+Z`) spindle, the textbook rule is `CCW ⇔ climb == internal`
//!   (internal climb → G3; external climb → G2).
//! - **Z sense** is then forced by the thread **hand** so the helix chirality is
//!   right: a right-hand thread is `(CCW, up)` or `(CW, down)`; left-hand is the
//!   mirror. Because the hand is enforced from the chosen orbit direction, **the
//!   hand is always geometrically correct** — only the climb/conventional
//!   *labelling* depends on the `M3` spindle assumption noted above.

use std::f64::consts::PI;

use cam_cldata::{ArcDir, MoveKind, Point3, Program, Step, Tag};
use cam_model::{Hand, ThreadOp};

use crate::{CancelToken, Diagnostic, JobEnv, Strategy, StrategyResult};

/// Mills a thread at each hole/boss centre. Construct from a [`ThreadOp`].
#[derive(Clone, Debug)]
pub struct ThreadStrategy {
    op: ThreadOp,
}

impl ThreadStrategy {
    /// Build a thread-milling strategy for `op`.
    pub fn new(op: ThreadOp) -> Self {
        Self { op }
    }
}

impl Strategy for ThreadStrategy {
    fn name(&self) -> &str {
        "thread"
    }

    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        let op = &self.op;
        let mut diagnostics = Vec::new();

        // Bail helper: return whatever diagnostics we have and no motions.
        macro_rules! bail {
            () => {
                return StrategyResult {
                    diagnostics,
                    ..Default::default()
                }
            };
        }

        let Some(tool) = env.tool(op.tool) else {
            diagnostics.push(Diagnostic::error(format!(
                "operation {} references tool {} which is not in the setup",
                op.id, op.tool
            )));
            bail!();
        };
        let tool_r = tool.radius();

        if op.points.is_empty() {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: no holes to thread",
                op.id
            )));
            bail!();
        }
        if op.pitch <= 0.0 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: pitch must be positive",
                op.id
            )));
            bail!();
        }
        if op.major_dia <= 0.0 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: major diameter must be positive",
                op.id
            )));
            bail!();
        }
        if op.z_top <= op.z_bottom {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: thread top {} must be above the bottom {}",
                op.id, op.z_top, op.z_bottom
            )));
            bail!();
        }
        if op.feed <= 0.0 || op.plunge_feed <= 0.0 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: feeds must be positive",
                op.id
            )));
            bail!();
        }

        let major_r = 0.5 * op.major_dia;
        // Tool-centre orbit radius: inside the major diameter for an internal
        // thread, outside it for an external one.
        let orbit_r = if op.internal {
            major_r - tool_r
        } else {
            major_r + tool_r
        };
        if orbit_r <= 0.0 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: tool diameter {} is too large to mill an internal thread of major diameter {}",
                op.id, tool.diameter, op.major_dia
            )));
            bail!();
        }
        // Approximate 60° minor diameter; a tool wider than this cannot enter the
        // pre-drilled hole. Advisory (thread form/angle is not modelled yet).
        if op.internal {
            let minor_approx = op.major_dia - 1.0825 * op.pitch;
            if tool.diameter > minor_approx {
                diagnostics.push(Diagnostic::warning(format!(
                    "operation {}: tool diameter {:.3} may not clear the ~{:.3} minor diameter of the pre-drilled hole",
                    op.id, tool.diameter, minor_approx
                )));
            }
        }

        // Orbit direction (climb vs conventional under an M3 spindle) and, from
        // it, the Z sense that keeps the thread hand correct. See the module docs.
        let arc_dir = if op.climb == op.internal {
            ArcDir::Ccw
        } else {
            ArcDir::Cw
        };
        let z_up = (op.hand == Hand::Right) == (arc_dir == ArcDir::Ccw);
        let (z_start, z_end) = if z_up {
            (op.z_bottom, op.z_top)
        } else {
            (op.z_top, op.z_bottom)
        };

        // Total angular sweep of the helix: `turns · 2π`. Emitted as arc segments
        // of at most a half-turn — universally supported and unambiguous.
        let turns = (op.z_top - op.z_bottom) / op.pitch;
        let sweep_total = turns * 2.0 * PI;
        let nseg = ((turns * 2.0).ceil() as usize).max(1);

        let mut program = Program::new();
        program.push(Step::Comment(format!(
            "Thread mill: {} {} \u{00d8}{:.3}\u{00d7}{:.3}",
            if op.internal { "internal" } else { "external" },
            match op.hand {
                Hand::Right => "RH",
                Hand::Left => "LH",
            },
            op.major_dia,
            op.pitch,
        )));

        let params = HoleParams {
            op_id: op.id,
            orbit_r,
            internal: op.internal,
            arc_dir,
            z_start,
            z_end,
            sweep_total,
            nseg,
            feed: op.feed,
            plunge_feed: op.plunge_feed,
            clearance: env.heights.clearance,
            retract: env.heights.retract,
        };

        for &[cx, cy] in &op.points {
            if cancel.is_cancelled() {
                return StrategyResult {
                    program,
                    diagnostics,
                    cancelled: true,
                };
            }
            emit_hole(&mut program, &params, cx, cy);
        }

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

/// Everything shared across the holes of one thread operation.
struct HoleParams {
    op_id: u32,
    orbit_r: f64,
    internal: bool,
    arc_dir: ArcDir,
    z_start: f64,
    z_end: f64,
    /// Magnitude of the total angular sweep (radians); the sign comes from `arc_dir`.
    sweep_total: f64,
    nseg: usize,
    feed: f64,
    plunge_feed: f64,
    clearance: f64,
    retract: f64,
}

/// Emit the full motion for one threaded hole/boss centred at `(cx, cy)`.
fn emit_hole(prog: &mut Program, p: &HoleParams, cx: f64, cy: f64) {
    let link = Tag::new(p.op_id, MoveKind::Link);
    let plunge = Tag::new(p.op_id, MoveKind::Plunge);
    let lead = Tag::new(p.op_id, MoveKind::LeadIn);
    let cut = Tag::new(p.op_id, MoveKind::Cutting);
    let retract_tag = Tag::new(p.op_id, MoveKind::Retract);

    let sign = match p.arc_dir {
        ArcDir::Ccw => 1.0,
        ArcDir::Cw => -1.0,
    };

    // Entry point on the orbit, at angle 0 (the +X side of the centre).
    let p0 = orbit_point(cx, cy, p.orbit_r, 0.0);
    // Approach anchor: the hole centre (internal) or an outside staging point
    // radially aligned with the entry (external), where the tool can safely
    // plunge in free air.
    let a0 = anchor(cx, cy, p.orbit_r, 0.0, p.internal);
    // Tangent semicircle from anchor to entry: same sense as the orbit for an
    // internal thread, opposite for an external one (derived from tangency).
    let lead_dir = if p.internal {
        p.arc_dir
    } else {
        opposite(p.arc_dir)
    };

    // Rapid over the anchor at clearance, then down to the retract plane.
    prog.push(Step::Rapid {
        to: Point3::new(a0.0, a0.1, p.clearance),
        tag: link,
    });
    prog.push(Step::Rapid {
        to: Point3::new(a0.0, a0.1, p.retract),
        tag: link,
    });
    // Plunge to the starting Z at the anchor.
    prog.push(Step::Linear {
        to: Point3::new(a0.0, a0.1, p.z_start),
        feed: p.plunge_feed,
        tag: plunge,
    });
    // Lead-in arc onto the orbit.
    let lc_in = midpoint(a0, p0);
    prog.push(Step::Arc {
        end: Point3::new(p0.0, p0.1, p.z_start),
        center: Point3::new(lc_in.0, lc_in.1, p.z_start),
        dir: lead_dir,
        feed: p.feed,
        tag: lead,
    });

    // Helical cut: half-turn (or smaller) arcs, Z advancing linearly with angle.
    for k in 1..=p.nseg {
        let frac = k as f64 / p.nseg as f64;
        let theta = sign * p.sweep_total * frac;
        let z = p.z_start + (p.z_end - p.z_start) * frac;
        let pt = orbit_point(cx, cy, p.orbit_r, theta);
        prog.push(Step::Arc {
            end: Point3::new(pt.0, pt.1, z),
            center: Point3::new(cx, cy, z),
            dir: p.arc_dir,
            feed: p.feed,
            tag: cut,
        });
    }

    // Lead-out arc off the orbit back to an anchor at the final Z, then retract.
    let theta_end = sign * p.sweep_total;
    let pend = orbit_point(cx, cy, p.orbit_r, theta_end);
    let a_end = anchor(cx, cy, p.orbit_r, theta_end, p.internal);
    let lc_out = midpoint(pend, a_end);
    prog.push(Step::Arc {
        end: Point3::new(a_end.0, a_end.1, p.z_end),
        center: Point3::new(lc_out.0, lc_out.1, p.z_end),
        dir: lead_dir,
        feed: p.feed,
        tag: lead,
    });
    prog.push(Step::Rapid {
        to: Point3::new(a_end.0, a_end.1, p.clearance),
        tag: retract_tag,
    });
}

/// A point on the orbit of radius `r` about `(cx, cy)` at angle `theta` (radians).
fn orbit_point(cx: f64, cy: f64, r: f64, theta: f64) -> (f64, f64) {
    (cx + r * theta.cos(), cy + r * theta.sin())
}

/// The approach/exit anchor for the entry at `theta`: the centre for an internal
/// thread (plunge down the open bore), or a point at twice the orbit radius —
/// clear of the boss — for an external one.
fn anchor(cx: f64, cy: f64, orbit_r: f64, theta: f64, internal: bool) -> (f64, f64) {
    if internal {
        (cx, cy)
    } else {
        orbit_point(cx, cy, 2.0 * orbit_r, theta)
    }
}

fn midpoint(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1))
}

fn opposite(d: ArcDir) -> ArcDir {
    match d {
        ArcDir::Ccw => ArcDir::Cw,
        ArcDir::Cw => ArcDir::Ccw,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Severity;
    use cam_model::{Heights, Tool, ToolKind};

    fn tool(dia: f64) -> Tool {
        Tool {
            number: 1,
            diameter: dia,
            length: 30.0,
            flutes: 3,
            kind: ToolKind::ThreadMill,
        }
    }

    /// M10×1.5 base op: one hole at the origin, 6 mm long (an exact 4 turns).
    fn op(internal: bool, hand: Hand, climb: bool) -> ThreadOp {
        ThreadOp {
            id: 0,
            tool: 1,
            points: vec![[0.0, 0.0]],
            internal,
            hand,
            major_dia: 10.0,
            pitch: 1.5,
            z_top: 0.0,
            z_bottom: -6.0,
            climb,
            feed: 200.0,
            plunge_feed: 100.0,
        }
    }

    fn run(op: ThreadOp, tool: Tool) -> StrategyResult {
        let tools = [tool];
        let env = JobEnv {
            heights: Heights::new(10.0, 2.0, 0.0),
            tools: &tools,
        };
        ThreadStrategy::new(op).compute(&env, &CancelToken::new())
    }

    /// The `(end, center, dir)` of every material-removing arc, in order.
    fn cutting_arcs(prog: &Program) -> Vec<(Point3, Point3, ArcDir)> {
        prog.steps
            .iter()
            .filter_map(|s| match s {
                Step::Arc {
                    end, center, dir, tag, ..
                } if tag.kind == MoveKind::Cutting => Some((*end, *center, *dir)),
                _ => None,
            })
            .collect()
    }

    /// The hand is set by the helix chirality; climb by the orbit sense. This
    /// pins the full truth table (assuming an M3 spindle).
    #[test]
    fn direction_matrix() {
        // (internal, hand, climb) -> (orbit dir, cut climbs in +Z)
        let cases = [
            (true, Hand::Right, true, ArcDir::Ccw, true),
            (true, Hand::Right, false, ArcDir::Cw, false),
            (true, Hand::Left, true, ArcDir::Ccw, false),
            (true, Hand::Left, false, ArcDir::Cw, true),
            (false, Hand::Right, true, ArcDir::Cw, false),
            (false, Hand::Right, false, ArcDir::Ccw, true),
            (false, Hand::Left, true, ArcDir::Cw, true),
            (false, Hand::Left, false, ArcDir::Ccw, false),
        ];
        for (internal, hand, climb, exp_dir, up) in cases {
            let r = run(op(internal, hand, climb), tool(5.0));
            assert!(!r.has_errors(), "{internal} {hand:?} {climb}: {:?}", r.diagnostics);
            let arcs = cutting_arcs(&r.program);
            assert!(
                arcs.iter().all(|(_, _, d)| *d == exp_dir),
                "orbit dir for {internal} {hand:?} climb={climb}"
            );
            let first = arcs.first().unwrap().0.z;
            let last = arcs.last().unwrap().0.z;
            if up {
                assert!(last > first, "expected +Z cut for {internal} {hand:?} climb={climb}");
            } else {
                assert!(last < first, "expected -Z cut for {internal} {hand:?} climb={climb}");
            }
        }
    }

    /// The defining property of a thread: exactly `pitch` of Z per revolution.
    /// With 4 exact turns the helix is 8 half-turn segments of `pitch/2` each.
    #[test]
    fn helix_advances_one_pitch_per_turn() {
        let r = run(op(true, Hand::Right, true), tool(5.0));
        let arcs = cutting_arcs(&r.program);
        assert_eq!(arcs.len(), 8, "4 turns → 8 half-turn segments");

        let mut zs = vec![-6.0_f64]; // z_start, then each segment endpoint
        zs.extend(arcs.iter().map(|(e, _, _)| e.z));
        for w in zs.windows(2) {
            assert!(
                ((w[1] - w[0]) - 0.75).abs() < 1e-9,
                "half-turn Z advance {} should be pitch/2",
                w[1] - w[0]
            );
        }
        assert!((zs.last().unwrap() - 0.0).abs() < 1e-9, "ends at z_top");
    }

    /// Cutting stays on the tool-centre orbit about the hole centre, so the edge
    /// lands on the major diameter: internal orbit = major/2 − tool_r.
    #[test]
    fn cutting_stays_on_orbit_radius() {
        let r = run(op(true, Hand::Right, true), tool(5.0));
        let orbit_r = 10.0 / 2.0 - 2.5;
        for (end, center, _) in cutting_arcs(&r.program) {
            assert!((center.x).abs() < 1e-9 && (center.y).abs() < 1e-9);
            let d = ((end.x - center.x).powi(2) + (end.y - center.y).powi(2)).sqrt();
            assert!((d - orbit_r).abs() < 1e-9, "off orbit: {d}");
        }
    }

    #[test]
    fn one_hole_has_two_leads_and_one_plunge() {
        let r = run(op(true, Hand::Right, true), tool(5.0));
        let s = &r.program.steps;
        assert!(matches!(s[0], Step::Comment(_)));
        let leads = s
            .iter()
            .filter(|st| matches!(st, Step::Arc { tag, .. } if tag.kind == MoveKind::LeadIn))
            .count();
        assert_eq!(leads, 2, "one lead-in + one lead-out");
        let plunges = s
            .iter()
            .filter(|st| matches!(st, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge))
            .count();
        assert_eq!(plunges, 1);
    }

    #[test]
    fn tool_too_large_for_internal_errors() {
        let r = run(op(true, Hand::Right, true), tool(12.0));
        assert!(r.has_errors());
        assert!(r.program.is_empty());
    }

    #[test]
    fn empty_points_warns_without_motion() {
        let mut o = op(true, Hand::Right, true);
        o.points.clear();
        let r = run(o, tool(5.0));
        assert!(!r.has_errors());
        assert!(r.program.is_empty());
        assert!(r
            .diagnostics
            .iter()
            .any(|d| d.severity == Severity::Warning));
    }

    #[test]
    fn inverted_z_span_errors() {
        let mut o = op(true, Hand::Right, true);
        o.z_top = -6.0;
        o.z_bottom = 0.0;
        let r = run(o, tool(5.0));
        assert!(r.has_errors());
    }
}
