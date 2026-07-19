//! The **V-carve engraving** strategy: run a V-bit along a path with its tip sunk
//! into the material, ploughing a V-section groove.
//!
//! ## Geometry
//!
//! Unlike [`chamfer`](crate::chamfer), which runs *beside* an edge so its flank forms
//! a bevel, engraving runs the tool **centred on the path** — no side, no radius
//! compensation. The tool axis follows the chain exactly; the groove's width is a
//! consequence of the depth and the bit, not an independent input:
//!
//! ```text
//! half-width = vtip_half_width(α, tip_radius, depth)
//! ```
//!
//! which is piecewise — a circle while the cut is still inside the rounded tip, then
//! the cone flank (see [`cam_geo::vtip_half_width`]). At engraving depths a tipped bit
//! is usually still in the *ball*, where the naive `depth·tan α` is badly wrong.
//!
//! ## Why a chamfer mill is rejected
//!
//! A chamfer mill's bottom is a **flat, non-cutting** tip: only its flank cuts. Plunge
//! one into the material and the flat rubs instead of cutting — a path that backplots
//! perfectly and burns the tool. That is a hard error here, not a warning.
//!
//! ## Depth limit
//!
//! A V-bit's cone flares out to the shank at its full cutting radius. Past that depth
//! the shank — not a cutting edge — is against the groove wall, so
//! [`cam_geo::vtip_max_depth`] is a hard gate too.

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{vtip_half_width, vtip_max_depth};
use cam_model::{EngraveOp, ToolKind};

use crate::{CancelToken, Diagnostic, JobEnv, Strategy, StrategyResult};

/// Engraves a V-section groove along a path. Construct from an [`EngraveOp`].
#[derive(Clone, Debug)]
pub struct EngraveStrategy {
    op: EngraveOp,
}

impl EngraveStrategy {
    /// Build an engraving strategy for `op`.
    pub fn new(op: EngraveOp) -> Self {
        Self { op }
    }
}

impl Strategy for EngraveStrategy {
    fn name(&self) -> &str {
        "engrave"
    }

    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        let op = &self.op;
        let mut diagnostics = Vec::new();

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

        // GATE 1 — the tool must be a V-bit. A chamfer mill is the near miss worth
        // naming explicitly: it looks right (it is conical) but its tip does not cut.
        let (included_angle_deg, tip_radius) = match tool.kind {
            ToolKind::VBit {
                included_angle_deg,
                tip_radius,
            } => (included_angle_deg, tip_radius),
            ToolKind::ChamferMill { .. } => {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: tool {} is a chamfer mill, whose tip is a flat \
                     non-cutting face — it would rub, not cut. Engraving needs a V-bit.",
                    op.id, op.tool
                )));
                bail!();
            }
            _ => {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: tool {} is a {}; engraving needs a V-bit \
                     (the groove's V comes from the tool's point)",
                    op.id, op.tool, tool.kind
                )));
                bail!();
            }
        };

        if op.chain.len() < 2 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: an engraving path needs at least 2 points",
                op.id
            )));
            bail!();
        }
        if op.closed && !op.chain.is_valid() {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: a closed engraving path needs at least 3 points",
                op.id
            )));
            bail!();
        }
        if op.depth <= 0.0 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: engraving depth must be positive",
                op.id
            )));
            bail!();
        }

        // Half the included V angle, from the tool axis.
        let alpha = 0.5 * included_angle_deg.to_radians();
        if !(alpha > 0.0 && alpha < std::f64::consts::FRAC_PI_2) {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: tool included angle {} is not a valid V (0–180°)",
                op.id, included_angle_deg
            )));
            bail!();
        }

        // GATE 2 — depth must keep a cutting edge in contact: past the point where the
        // cone reaches the full cutting radius, the shank rubs the groove walls.
        let max_depth = vtip_max_depth(alpha, tip_radius, tool.radius());
        if op.depth > max_depth + 1e-9 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: depth {:.3} mm exceeds the {:.3} mm at which tool {}'s \
                 cone reaches its full cutting ⌀ {:.3} — deeper, the shank rubs instead \
                 of cutting. Use a larger-⌀ or narrower-angle V-bit.",
                op.id,
                op.depth,
                max_depth,
                op.tool,
                tool.diameter
            )));
            bail!();
        }

        let half_width = vtip_half_width(alpha, tip_radius, op.depth);

        let link = Tag::new(op.id, MoveKind::Link);
        let plunge = Tag::new(op.id, MoveKind::Plunge);
        let cut = Tag::new(op.id, MoveKind::Cutting);
        let retract = Tag::new(op.id, MoveKind::Retract);

        // Depth passes, shallow to deep, landing exactly on `depth`.
        let depths = pass_depths(op.depth, op.stepdown);

        let mut program = Program::new();
        program.push(Step::Comment(format!(
            "Engrave: {:.3} mm deep, {:.3} mm wide groove at {}\u{00b0} in {} pass(es)",
            op.depth,
            2.0 * half_width,
            included_angle_deg,
            depths.len()
        )));

        // The path the tool centre follows. A closed loop may be rotated to begin at
        // the operator's chosen start and is closed back to it; an open stroke must
        // keep its own endpoints, so it is used as given.
        let pts: Vec<cam_geo::Point> = if op.closed {
            let mut v = crate::profile::rotate_to_start(op.chain.points(), op.start);
            v.push(v[0]); // close the loop
            v
        } else {
            op.chain.points().to_vec()
        };
        let start = pts[0];

        let mut prev_z = op.top;
        for &d in &depths {
            if cancel.is_cancelled() {
                return StrategyResult {
                    program,
                    diagnostics,
                    cancelled: true,
                };
            }
            let z = op.top - d;
            // Rapid over the start at clearance, down through the air already cut by
            // the previous pass, then feed-plunge to this pass's depth.
            program.push(Step::Rapid {
                to: Point3::new(start.x, start.y, env.heights.clearance),
                tag: link,
            });
            program.push(Step::Rapid {
                to: Point3::new(start.x, start.y, prev_z),
                tag: link,
            });
            program.push(Step::Linear {
                to: Point3::new(start.x, start.y, z),
                feed: op.plunge_feed,
                tag: plunge,
            });
            crate::emit::cut_polyline(&mut program, &pts, op.feed, cut, z);
            // Lift straight up from wherever the path ended — for an open stroke that
            // is the far end, not the start.
            let end = *pts.last().unwrap();
            program.push(Step::Rapid {
                to: Point3::new(end.x, end.y, env.heights.clearance),
                tag: retract,
            });
            prev_z = z;
        }

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

/// The cumulative depth at each pass, always ending exactly at `depth`.
///
/// `stepdown <= 0` or `>= depth` gives a single full-depth pass — the normal case for
/// shallow engraving. Otherwise equal increments, with the last pass landing on
/// `depth` rather than overshooting or leaving a sliver.
fn pass_depths(depth: f64, stepdown: f64) -> Vec<f64> {
    if stepdown <= 0.0 || stepdown >= depth {
        return vec![depth];
    }
    let mut ds = Vec::new();
    let mut d = stepdown;
    while d < depth {
        ds.push(d);
        d += stepdown;
    }
    if ds.last().is_some_and(|&l| (depth - l).abs() < 1e-9) {
        ds.pop();
    }
    ds.push(depth);
    ds
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_geo::{Contour, Point};
    use cam_model::{Heights, Tool};
    use crate::Severity;

    fn vbit(included_angle_deg: f64, tip_radius: f64) -> Tool {
        Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 1,
            kind: ToolKind::VBit {
                included_angle_deg,
                tip_radius,
            },
            ..Default::default()
        }
    }

    fn tool_of(kind: ToolKind) -> Tool {
        Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind,
            ..Default::default()
        }
    }

    fn stroke() -> Contour {
        Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(20.0, 5.0),
        ])
    }

    fn square() -> Contour {
        Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(20.0, 0.0),
            Point::new(20.0, 20.0),
            Point::new(0.0, 20.0),
        ])
    }

    fn op(depth: f64) -> EngraveOp {
        EngraveOp {
            id: 0,
            tool: 1,
            chain: stroke(),
            closed: false,
            top: 0.0,
            depth,
            stepdown: 0.0,
            feed: 200.0,
            plunge_feed: 100.0,
            start: None,
        }
    }

    fn run(op: EngraveOp, tool: Tool) -> StrategyResult {
        let tools = [tool];
        let env = JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &tools,
            stock: None,
        };
        EngraveStrategy::new(op).compute(&env, &CancelToken::new())
    }

    fn errors(r: &StrategyResult) -> Vec<String> {
        r.diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Error)
            .map(|d| d.message.clone())
            .collect()
    }

    fn cut_zs(prog: &Program) -> Vec<f64> {
        let mut zs: Vec<f64> = prog
            .steps()
            .iter()
            .filter_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(to.z),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => Some(end.z),
                _ => None,
            })
            .collect();
        zs.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
        zs
    }

    // --- the gates ---

    #[test]
    fn a_chamfer_mill_is_rejected_because_its_tip_does_not_cut() {
        let r = run(
            op(0.5),
            tool_of(ToolKind::ChamferMill {
                included_angle_deg: 90.0,
                tip_diameter: 0.0,
            }),
        );
        let e = errors(&r);
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("chamfer mill") && e[0].contains("non-cutting"), "{e:?}");
        assert!(r.program.steps().is_empty(), "no path may be emitted");
    }

    #[test]
    fn a_flat_end_mill_is_rejected_too() {
        let r = run(op(0.5), tool_of(ToolKind::EndMill));
        let e = errors(&r);
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("V-bit"), "{e:?}");
        assert!(r.program.steps().is_empty());
    }

    #[test]
    fn depth_past_the_cone_flare_is_rejected() {
        // 90° V-bit, ⌀6 (r=3), sharp tip → the cone reaches full ⌀ at depth 3.0.
        let t = vbit(90.0, 0.0);
        assert!(errors(&run(op(2.9), t)).is_empty());
        let r = run(op(3.1), t);
        let e = errors(&r);
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("shank rubs"), "{e:?}");
        assert!(r.program.steps().is_empty());
        // Exactly at the limit is allowed.
        assert!(errors(&run(op(3.0), t)).is_empty());
    }

    #[test]
    fn a_non_positive_depth_is_rejected() {
        for d in [0.0, -1.0] {
            let r = run(op(d), vbit(90.0, 0.0));
            assert_eq!(errors(&r).len(), 1, "depth {d}");
        }
    }

    #[test]
    fn a_closed_path_needs_three_points() {
        let mut o = op(0.5);
        o.chain = Contour::new(vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0)]);
        o.closed = true;
        let r = run(o, vbit(90.0, 0.0));
        assert_eq!(errors(&r).len(), 1);
    }

    // --- the path ---

    #[test]
    fn the_tool_centre_follows_the_path_with_no_offset() {
        // The distinguishing property vs. a chamfer: no radius compensation. Every
        // cutting point must lie exactly on the chain.
        let r = run(op(0.5), vbit(90.0, 0.0));
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        let pts: Vec<[f64; 2]> = r
            .program
            .steps()
            .iter()
            .filter_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some([to.x, to.y]),
                _ => None,
            })
            .collect();
        assert_eq!(pts, vec![[10.0, 0.0], [20.0, 5.0]]);
    }

    #[test]
    fn an_open_stroke_does_not_return_to_its_start() {
        let r = run(op(0.5), vbit(90.0, 0.0));
        // It retracts at the far end (20, 5), not back at the origin.
        let last = r.program.steps().last().unwrap();
        match last {
            Step::Rapid { to, tag } => {
                assert_eq!(tag.kind, MoveKind::Retract);
                assert!((to.x - 20.0).abs() < 1e-9 && (to.y - 5.0).abs() < 1e-9, "{to:?}");
            }
            other => panic!("expected a retract, got {other:?}"),
        }
    }

    #[test]
    fn a_closed_path_returns_to_its_start() {
        let mut o = op(0.5);
        o.chain = square();
        o.closed = true;
        let r = run(o, vbit(90.0, 0.0));
        assert!(errors(&r).is_empty());
        let last_cut = r
            .program
            .steps()
            .iter()
            .filter_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some([to.x, to.y]),
                _ => None,
            })
            .next_back()
            .unwrap();
        assert!(last_cut[0].abs() < 1e-9 && last_cut[1].abs() < 1e-9, "{last_cut:?}");
    }

    #[test]
    fn the_groove_width_is_reported_from_the_real_tip_geometry() {
        // A tipped bit at shallow depth is in the ball: width = 2·√(2·rt·d − d²),
        // which is far wider than the naive 2·d·tan α.
        let (rt, d) = (0.3, 0.02);
        let r = run(op(d), vbit(60.0, rt));
        let want = 2.0 * (2.0 * rt * d - d * d).sqrt();
        let comment = r
            .program
            .steps()
            .iter()
            .find_map(|s| match s {
                Step::Comment(c) => Some(c.clone()),
                _ => None,
            })
            .unwrap();
        assert!(
            comment.contains(&format!("{want:.3}")),
            "comment {comment:?} should report width {want:.3}"
        );
    }

    #[test]
    fn stepdown_splits_the_depth_and_lands_exactly_on_it() {
        let mut o = op(1.0);
        o.stepdown = 0.4;
        let r = run(o, vbit(90.0, 0.0));
        assert_eq!(cut_zs(&r.program), vec![-0.4, -0.8, -1.0]);
    }

    #[test]
    fn a_stepdown_that_divides_evenly_leaves_no_sliver_pass() {
        let mut o = op(1.2);
        o.stepdown = 0.4;
        let r = run(o, vbit(90.0, 0.0));
        assert_eq!(cut_zs(&r.program), vec![-0.4, -0.8, -1.2]);
    }

    #[test]
    fn no_stepdown_is_a_single_full_depth_pass() {
        let r = run(op(0.6), vbit(90.0, 0.0));
        assert_eq!(cut_zs(&r.program), vec![-0.6]);
    }

    #[test]
    fn depth_is_measured_down_from_the_top_plane() {
        let mut o = op(0.5);
        o.top = -2.0; // engraving a already-faced surface
        let r = run(o, vbit(90.0, 0.0));
        assert_eq!(cut_zs(&r.program), vec![-2.5]);
    }

    #[test]
    fn pass_depths_are_monotonic_and_end_on_target() {
        for &(d, s) in &[(1.0, 0.3), (2.0, 0.7), (0.5, 0.5), (1.0, 0.0), (1.0, 5.0)] {
            let ds = pass_depths(d, s);
            assert!((ds.last().unwrap() - d).abs() < 1e-12, "d={d} s={s}");
            assert!(ds.windows(2).all(|w| w[1] > w[0]), "d={d} s={s} {ds:?}");
            assert!(ds.iter().all(|&x| x > 0.0 && x <= d + 1e-12));
        }
    }
}
