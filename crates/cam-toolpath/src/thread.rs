//! The thread-milling strategy: cut a thread at each hole/boss centre by
//! **helically interpolating** a single-form thread mill — one continuous helix
//! of `length / pitch` turns, advancing exactly `pitch` in Z per revolution.
//!
//! ## Depth and passes
//!
//! The full radial depth of a 60° form (0.5413·pitch internal, 0.6134·pitch external)
//! is reached in `passes` equal radial steps, each a full helix, stepping the tool-
//! centre orbit outward (internal) or inward (external) so the crest lands from the
//! grazing surface to the root, deepest last; `spring_passes` add full-depth repeats.
//! The tool geometry is enforced up front by three hard gates (min cutting ⌀ vs. the
//! pre-drilled minor bore, max thread depth from the reduced neck, reach vs. length of
//! cut) plus a blind-hole allowance that validates the drilled depth.
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
use cam_model::{Hand, ThreadOp, ToolKind};

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

        // Gate 0 — the tool must be a thread mill. Gates 1–3 below reason about
        // diameter/neck/reach and would pass an end mill happily, which would helix a
        // plain groove instead of a thread form: valid-looking G-code, no thread.
        if !matches!(tool.kind, ToolKind::ThreadMill { .. }) {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: tool {} is a {}; thread milling needs a thread mill —                  the thread form is ground into the tool, so anything else would cut a                  plain helical groove.",
                op.id, op.tool, tool.kind
            )));
            bail!();
        }

        let major_r = 0.5 * op.major_dia;
        // Radial depth of a 60° thread form: an internal thread engages to the tap-drill
        // minor (major − 1.0825·pitch ⇒ 0.5413·pitch radial); an external thread's root
        // is truncated a little deeper (0.6134·pitch). This is how far the crest travels,
        // radially, from grazing the pre-drilled bore / boss OD to full depth.
        let radial_depth = if op.internal {
            0.541_265 * op.pitch
        } else {
            0.613_435 * op.pitch
        };

        // Gate 1 — min cutting ⌀: the tool's crest (its cutting diameter, the smallest
        // hole it can enter) must clear the pre-drilled minor bore of an internal thread.
        if op.internal {
            let minor = op.major_dia - 2.0 * radial_depth; // = major − 1.0825·pitch
            if tool.diameter >= minor {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: tool ⌀{:.3} cannot enter the ~⌀{:.3} pre-drilled minor bore of an M{:.3}×{:.3} thread",
                    op.id, tool.diameter, minor, op.major_dia, op.pitch
                )));
                bail!();
            }
        }

        // Gate 2 — max thread depth: a single-form mill can cut a radial depth of at most
        // (Dmin − Dneck)/2 = r_min − r_neck; a fatter neck rubs the fresh crest before the
        // tooth reaches full depth. (A full-profile mill's depth is ground into its comb,
        // so this gate does not apply to it.)
        if let ToolKind::ThreadMill { pitch: None } = tool.kind {
            let max_depth = tool.radius() - 0.5 * tool.neck_dia();
            if max_depth + 1e-9 < radial_depth {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: single-form mill max thread depth {:.3} (neck ⌀{:.3}) is less than the {:.3} this thread needs — specify a smaller neck ⌀",
                    op.id, max_depth, tool.neck_dia(), radial_depth
                )));
                bail!();
            }
        }

        // Gate 3 — reach: the threaded length must fit inside the tool's length of cut.
        let thread_length = op.z_top - op.z_bottom;
        if thread_length > tool.flute_len() + 1e-9 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: threaded length {:.3} exceeds the tool's length of cut (reach) {:.3}",
                op.id, thread_length, tool.flute_len()
            )));
            bail!();
        }

        // Blind-hole allowance: a positive `drill_clearance` marks a blind hole; the
        // pre-drill must leave at least the allowance (auto = one pitch) below the thread,
        // since the tool cannot thread flush to a blind bottom.
        if op.internal && op.drill_clearance > 0.0 {
            let allowance = if op.blind_allowance > 0.0 {
                op.blind_allowance
            } else {
                op.pitch
            };
            if op.drill_clearance + 1e-9 < allowance {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: blind-hole drill clearance {:.3} below the thread is less than the required allowance {:.3}",
                    op.id, op.drill_clearance, allowance
                )));
                bail!();
            }
        }

        // Cut-edge radius from grazing to full depth, and the tool-centre orbit placing
        // the crest there: inside the cut for an internal thread, outside for external.
        let (cut_start, cut_final) = if op.internal {
            (major_r - radial_depth, major_r)
        } else {
            (major_r, major_r - radial_depth)
        };
        let orbit_of = |cut_r: f64| {
            if op.internal {
                cut_r - tool_r
            } else {
                cut_r + tool_r
            }
        };
        if orbit_of(cut_final) <= 0.0 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: tool diameter {} is too large to mill a thread of major diameter {}",
                op.id, tool.diameter, op.major_dia
            )));
            bail!();
        }
        // Radial infeed passes (equal steps to full depth), then any spring passes at
        // full depth. Each entry is a tool-centre orbit radius; the emitter cuts a full
        // helix at each, deepest last.
        let npass = op.passes.max(1);
        let mut orbits: Vec<f64> = (1..=npass)
            .map(|i| {
                let f = i as f64 / npass as f64;
                orbit_of(cut_start + f * (cut_final - cut_start))
            })
            .collect();
        for _ in 0..op.spring_passes {
            orbits.push(orbit_of(cut_final));
        }

        // Orbit direction (climb vs conventional under an M3 spindle) and, from
        // it, the Z sense that keeps the thread hand correct. See the module docs.
        let arc_dir = if op.climb == op.internal {
            ArcDir::Ccw
        } else {
            ArcDir::Cw
        };
        let z_up = (op.hand == Hand::Right) == (arc_dir == ArcDir::Ccw);
        // Full-profile mills (a tooth comb ground for a specific pitch) cut the
        // whole thread in a single turn; a single-form mill runs one turn per
        // pitch over the length. Same helix generator, different turn count.
        let full_profile = match tool.kind {
            ToolKind::ThreadMill { pitch: Some(p) } => {
                if (p - op.pitch).abs() > 1e-6 {
                    diagnostics.push(Diagnostic::warning(format!(
                        "operation {}: full-profile tool pitch {:.3} differs from the thread pitch {:.3}",
                        op.id, p, op.pitch
                    )));
                }
                true
            }
            _ => false,
        };

        let length = op.z_top - op.z_bottom;
        // Turns of the helix, and the Z it climbs while cutting: the full length
        // for a single-form mill, exactly one pitch for a full-profile comb.
        let (turns, z_span) = if full_profile {
            (1.0, op.pitch.min(length))
        } else {
            (length / op.pitch, length)
        };
        let (z_start, z_end) = if z_up {
            (op.z_bottom, op.z_bottom + z_span)
        } else {
            (op.z_top, op.z_top - z_span)
        };

        // Total angular sweep of the helix: `turns · 2π`. Emitted as arc segments
        // of at most a half-turn — universally supported and unambiguous.
        let sweep_total = turns * 2.0 * PI;
        let nseg = ((turns * 2.0).ceil() as usize).max(1);

        let mut program = Program::new();
        program.push(Step::Comment(format!(
            "Thread mill: {} {} {} \u{00d8}{:.3}\u{00d7}{:.3}",
            if full_profile {
                "full-profile"
            } else {
                "single-form"
            },
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
            orbits,
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
    /// Tool-centre orbit radius per pass, deepest last (incl. spring passes).
    orbits: Vec<f64>,
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

/// Emit the full motion for one threaded hole/boss centred at `(cx, cy)`: one radial
/// infeed pass per orbit radius (deepest last), each retracting to clearance so the
/// re-approach is unambiguously safe.
fn emit_hole(prog: &mut Program, p: &HoleParams, cx: f64, cy: f64) {
    for &orbit_r in &p.orbits {
        emit_pass(prog, p, cx, cy, orbit_r);
    }
}

/// Emit one radial pass: approach, lead-in, helix, lead-out, retract, at `orbit_r`.
fn emit_pass(prog: &mut Program, p: &HoleParams, cx: f64, cy: f64, orbit_r: f64) {
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
    let p0 = orbit_point(cx, cy, orbit_r, 0.0);
    // Approach anchor: the hole centre (internal) or an outside staging point
    // radially aligned with the entry (external), where the tool can safely
    // plunge in free air.
    let a0 = anchor(cx, cy, orbit_r, 0.0, p.internal);
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
        let pt = orbit_point(cx, cy, orbit_r, theta);
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
    let pend = orbit_point(cx, cy, orbit_r, theta_end);
    let a_end = anchor(cx, cy, orbit_r, theta_end, p.internal);
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
            // A real single-form mill needs a reduced neck behind the tooth; ⌀3 leaves a
            // 1.0 mm max thread depth on a ⌀5 mill (clears an M10×1.5's 0.92 mm form).
            neck_diameter: 3.0,
            flutes: 3,
            kind: ToolKind::ThreadMill { pitch: None },
            ..Default::default()
        }
    }

    /// M10×1.5 base op: one hole at the origin, 6 mm long (an exact 4 turns).
    fn op(internal: bool, hand: Hand, climb: bool) -> ThreadOp {
        ThreadOp {
            spindle_rpm: 0.0,
            work_offset: 1,
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
            passes: 1,
            spring_passes: 0,
            drill_clearance: 0.0,
            blind_allowance: 0.0,
            feed: 200.0,
            plunge_feed: 100.0,
        }
    }

    fn run(op: ThreadOp, tool: Tool) -> StrategyResult {
        let tools = [tool];
        let env = JobEnv {
            heights: Heights::new(10.0, 2.0, 0.0),
            tools: &tools,
            stock: None,
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

    /// Distinct tool-centre orbit radii among the cutting arcs, in first-seen order
    /// (consecutive-dedup: each pass has a constant radius, adjacent passes differ,
    /// so a spring pass at the same radius collapses into its infeed pass).
    fn pass_radii(prog: &Program) -> Vec<f64> {
        let mut out: Vec<f64> = Vec::new();
        for (end, center, _) in cutting_arcs(prog) {
            let r = ((end.x - center.x).powi(2) + (end.y - center.y).powi(2)).sqrt();
            if out.last().is_none_or(|last| (last - r).abs() > 1e-6) {
                out.push(r);
            }
        }
        out
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

    fn full_profile_tool(pitch: f64) -> Tool {
        Tool {
            number: 1,
            diameter: 5.0,
            length: 30.0,
            flutes: 3,
            kind: ToolKind::ThreadMill { pitch: Some(pitch) },
            ..Default::default()
        }
    }

    /// A full-profile mill cuts the whole thread in one turn, climbing exactly one
    /// pitch (its tooth comb spans the length) — not one turn per pitch.
    #[test]
    fn full_profile_cuts_one_turn_of_one_pitch() {
        let r = run(op(true, Hand::Right, true), full_profile_tool(1.5));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        let arcs = cutting_arcs(&r.program);
        assert_eq!(arcs.len(), 2, "one turn → two half-turn segments");
        let zmax = arcs.iter().map(|(e, _, _)| e.z).fold(f64::MIN, f64::max);
        // Climbs from z_bottom (−6) by exactly one 1.5 mm pitch.
        assert!((zmax - (-6.0 + 1.5)).abs() < 1e-9, "should climb one pitch");
    }

    #[test]
    fn full_profile_pitch_mismatch_warns() {
        // Tool ground for 2.0 mm, thread wants 1.5 mm.
        let r = run(op(true, Hand::Right, true), full_profile_tool(2.0));
        assert!(r.diagnostics.iter().any(
            |d| d.severity == Severity::Warning && d.message.contains("pitch")
        ));
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

    /// Gate 1: the tool's cutting ⌀ must clear the pre-drilled minor bore.
    #[test]
    fn min_cut_diameter_gate_errors_when_tool_wont_fit_the_bore() {
        // M6×1.0 internal: minor ≈ 6 − 1.0825 = 4.92; a ⌀5 tool cannot enter.
        let mut o = op(true, Hand::Right, true);
        o.major_dia = 6.0;
        o.pitch = 1.0;
        let r = run(o, tool(5.0));
        assert!(r.has_errors());
        assert!(r
            .diagnostics
            .iter()
            .any(|d| d.message.contains("pre-drilled minor bore")));
        assert!(r.program.is_empty());
    }

    /// Gate 2: a single-form mill whose neck nearly equals its cutting ⌀ cannot reach
    /// the thread's radial depth.
    #[test]
    fn fat_neck_depth_gate_errors() {
        let mut t = tool(5.0);
        t.neck_diameter = 4.8; // max depth (5−4.8)/2 = 0.1 ≪ 0.81 needed for M10×1.5
        let r = run(op(true, Hand::Right, true), t);
        assert!(r.has_errors());
        assert!(r
            .diagnostics
            .iter()
            .any(|d| d.message.contains("max thread depth")));
    }

    /// Gate 3: the threaded length cannot exceed the tool's length of cut (reach).
    #[test]
    fn reach_gate_errors_when_thread_is_longer_than_length_of_cut() {
        let mut t = tool(5.0);
        t.flute_length = 4.0; // reach 4 mm
        let mut o = op(true, Hand::Right, true);
        o.z_top = 0.0;
        o.z_bottom = -6.0; // 6 mm thread > 4 mm reach
        let r = run(o, t);
        assert!(r.has_errors());
        assert!(r
            .diagnostics
            .iter()
            .any(|d| d.message.contains("length of cut")));
    }

    /// The blind-hole allowance validates the drilled depth: too shallow errors, deep
    /// enough passes. Auto allowance for M10×1.5 = one pitch = 1.5 mm.
    #[test]
    fn blind_hole_allowance_validates_drilled_depth() {
        let mut shallow = op(true, Hand::Right, true);
        shallow.drill_clearance = 1.0; // < 1.5
        let r = run(shallow, tool(5.0));
        assert!(r.has_errors());
        assert!(r.diagnostics.iter().any(|d| d.message.contains("allowance")));

        let mut deep = op(true, Hand::Right, true);
        deep.drill_clearance = 2.0; // ≥ 1.5
        let r2 = run(deep, tool(5.0));
        assert!(!r2.has_errors(), "{:?}", r2.diagnostics);
    }

    /// An explicit blind allowance overrides the one-pitch auto default.
    #[test]
    fn explicit_blind_allowance_overrides_the_auto_pitch() {
        let mut o = op(true, Hand::Right, true);
        o.drill_clearance = 1.2;
        o.blind_allowance = 1.0; // 1.2 ≥ 1.0 → ok (would fail against the 1.5 auto)
        let r = run(o, tool(5.0));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
    }

    /// Multiple radial passes step the orbit outward (internal), deepest last at the
    /// full-depth orbit major_r − tool_r.
    #[test]
    fn radial_passes_step_out_to_full_depth() {
        let mut o = op(true, Hand::Right, true);
        o.passes = 3;
        let r = run(o, tool(5.0));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        let radii = pass_radii(&r.program);
        assert_eq!(radii.len(), 3, "three radial passes: {radii:?}");
        assert!(
            radii.windows(2).all(|w| w[1] > w[0] + 1e-9),
            "passes deepen: {radii:?}"
        );
        assert!((radii.last().unwrap() - (5.0 - 2.5)).abs() < 1e-9);
    }

    /// A spring pass adds one extra full-depth helix (8 half-turns here) at the final
    /// radius, without deepening further.
    #[test]
    fn spring_pass_adds_a_full_depth_repeat() {
        let base = {
            let mut o = op(true, Hand::Right, true);
            o.passes = 2;
            run(o, tool(5.0))
        };
        let sprung = {
            let mut o = op(true, Hand::Right, true);
            o.passes = 2;
            o.spring_passes = 1;
            run(o, tool(5.0))
        };
        let n_base = cutting_arcs(&base.program).len();
        let n_sprung = cutting_arcs(&sprung.program).len();
        assert_eq!(n_sprung, n_base + 8, "one spring pass = one extra full helix");
        let radii = pass_radii(&sprung.program);
        assert!((radii.last().unwrap() - (5.0 - 2.5)).abs() < 1e-9, "spring at full depth");
    }

    /// The final external orbit places the crest at the thread root (major − radial),
    /// not merely grazing the OD — the flat-cylinder baseline cut no depth externally.
    #[test]
    fn external_thread_cuts_to_root_depth() {
        let r = run(op(false, Hand::Right, true), tool(5.0));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        let radii = pass_radii(&r.program);
        let (major_r, tool_r) = (5.0, 2.5);
        let expect = (major_r - 0.613_435 * 1.5) + tool_r;
        assert!((radii.last().unwrap() - expect).abs() < 1e-6, "got {radii:?}, want {expect}");
        // Not the old grazing radius major_r + tool_r.
        assert!((radii.last().unwrap() - (major_r + tool_r)).abs() > 1e-3);
    }
}
