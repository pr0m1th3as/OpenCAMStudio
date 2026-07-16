//! The chamfering strategy: run a chamfer/V mill around a closed edge at a
//! computed depth so its cone flank forms a bevel.
//!
//! ## Geometry
//!
//! A chamfer mill grinds a point of half-angle `α` from the axis (half the
//! included angle) with an optional flat tip of radius `rt`. To leave a bevel of
//! horizontal width `w` along a top edge at Z `top`, run the tool:
//!
//! - offset to the **air side** of the edge by `rt` (the tool axis clears the
//!   corner by the tip radius), and
//! - at Z `top − w/tan(α)`.
//!
//! Then the flank meets the top face `w` inside the edge and the tip sits at the
//! bevel's lower corner — one pass, since a chamfer is shallow. Derived by placing
//! the cone so the tip lands at the bevel bottom (see the project notes).

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, JoinStyle, Polygon};
use cam_model::{ChamferOp, Side, ToolKind};

use crate::{CancelToken, Diagnostic, JobEnv, Strategy, StrategyResult};

/// Chamfers a closed edge. Construct from a [`ChamferOp`].
#[derive(Clone, Debug)]
pub struct ChamferStrategy {
    op: ChamferOp,
}

impl ChamferStrategy {
    /// Build a chamfering strategy for `op`.
    pub fn new(op: ChamferOp) -> Self {
        Self { op }
    }
}

impl Strategy for ChamferStrategy {
    fn name(&self) -> &str {
        "chamfer"
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

        // A chamfer needs an angled tool — the bevel angle comes from the tool.
        let (included_angle_deg, tip_diameter) = match tool.kind {
            ToolKind::ChamferMill {
                included_angle_deg,
                tip_diameter,
            } => (included_angle_deg, tip_diameter),
            _ => {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: tool {} is not a chamfer/V mill; a chamfer needs its point angle",
                    op.id, op.tool
                )));
                bail!();
            }
        };

        if !op.chain.is_valid() {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: chamfer edge must be a closed area (≥ 3 vertices)",
                op.id
            )));
            bail!();
        }
        if op.width <= 0.0 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: chamfer width must be positive",
                op.id
            )));
            bail!();
        }
        // Half the included angle, measured from the tool axis; the flank rises
        // `1/tan(α)` per mm of radius. Guard the degenerate cone.
        let alpha = 0.5 * included_angle_deg.to_radians();
        let tan_a = alpha.tan();
        if !(alpha > 0.0 && alpha < std::f64::consts::FRAC_PI_2) || tan_a <= 1e-9 {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: tool included angle {} is not a valid cone (0–180°)",
                op.id, included_angle_deg
            )));
            bail!();
        }

        let tip_radius = 0.5 * tip_diameter;
        // The tool flank shares the bevel's angle, so it lies along the finished
        // bevel plane; sliding the tool down that plane changes which flank section
        // cuts. `d_min` is the tip's natural depth (tip exactly at the bevel bottom).
        // A chosen `depth` beyond that plunges the tip `delta` deeper — into the air
        // on an external edge — so a higher flank section does the work. Sliding the
        // tool down the plane by `delta` also shifts its axis to the air side by
        // `delta·tan α`, and — crucially — that shift is the *same* for every
        // partial-width pass, so a multi-pass chamfer is one XY contour cut at a
        // sequence of Z depths.
        let d_min = op.width / tan_a;
        let tip_depth = op.depth.max(d_min);
        let delta = tip_depth - d_min;
        if op.depth > 0.0 && op.depth < d_min {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: depth {:.3} is above the tip's natural depth {:.3}; using the tip",
                op.id, op.depth, d_min
            )));
        }
        if op.side == Side::Inside && delta > 1e-9 {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: internal chamfer plunges the tip {:.3} mm past the bevel — not collision-checked against the bore floor/opposite wall",
                op.id, delta
            )));
        }

        let region = match Polygon::new(op.chain.clone()) {
            Ok(p) => p,
            Err(e) => {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: edge is not a valid region: {e}",
                    op.id
                )));
                bail!();
            }
        };

        // Offset the tool axis to the air side by the deep-plunge shift plus the tip
        // radius (0 for a sharp V at the tip keeps it on the edge). Sign follows the
        // profile convention.
        let off = delta * tan_a + tip_radius;
        let signed = match op.side {
            Side::Outside => off,
            Side::Inside => -off,
            Side::On => 0.0,
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
                    bail!();
                }
            }
        };
        if loops.is_empty() {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: tip offset consumed the whole edge",
                op.id
            )));
            bail!();
        }

        let link = Tag::new(op.id, MoveKind::Link);
        let plunge = Tag::new(op.id, MoveKind::Plunge);
        let cut = Tag::new(op.id, MoveKind::Cutting);
        let lead = Tag::new(op.id, MoveKind::LeadIn);
        let retract = Tag::new(op.id, MoveKind::Retract);

        // Cumulative bevel widths per pass, ending exactly at the target width. The
        // deepest (final) pass reaches `tip_depth`; shallower passes ride the same
        // XY contour at a smaller plunge.
        let widths = pass_widths(op.width, op.step, op.gradual);

        let mut program = Program::new();
        program.push(Step::Comment(format!(
            "Chamfer: {:.3} mm wide at {}\u{00b0} in {} pass(es)",
            op.width,
            included_angle_deg,
            widths.len()
        )));

        for poly in &loops {
            if cancel.is_cancelled() {
                return StrategyResult {
                    program,
                    diagnostics,
                    cancelled: true,
                };
            }
            let pts = poly.outer().points();
            if pts.len() < 3 {
                continue;
            }
            // Begin the loop at the operator's chosen start (object-snap point),
            // so the plunge/entry lands there; `None` keeps the offset's first vertex.
            let rotated = crate::profile::rotate_to_start(pts, op.start);
            let pts = rotated.as_slice();
            let start = pts[0];

            // Lead geometry: the tool eases onto the edge at the start (off the air
            // side when there's a lead-in) and off it after the cut. The cut runs
            // the loop plus any closure overlap to a point `exit_on`; the lead-off
            // recomputes its outward normal there. With no leads and no overlap this
            // reduces to a straight plunge at the start and an in-place retract —
            // byte-identical to before.
            let tan_in = crate::profile::start_tangent(pts);
            let out = crate::profile::outward_normal(pts);
            let (loop_pts, exit_on, tan_out) =
                crate::emit::loop_with_overlap(pts, op.lead_overlap);
            let out_exit = if op.lead_overlap > 0.0 {
                crate::profile::outward_normal_at(tan_out, crate::profile::is_ccw(pts))
            } else {
                out
            };
            let entry = crate::leads::lead_start_point(start, tan_in, out, op.lead_in);
            let exit = crate::leads::lead_end_point(exit_on, tan_out, out_exit, op.lead_out);

            // One pass per cumulative width, shallow to deep. Each: rapid over the
            // entry at clearance and down through the already-cut air, plunge to the
            // pass depth, lead on, cut the loop (+ overlap), lead off, retract.
            let mut prev_z = op.top;
            for &wk in &widths {
                let z = op.top - (wk / tan_a + delta);
                program.push(Step::Rapid {
                    to: Point3::new(entry.x, entry.y, env.heights.clearance),
                    tag: link,
                });
                program.push(Step::Rapid {
                    to: Point3::new(entry.x, entry.y, prev_z),
                    tag: link,
                });
                program.push(Step::Linear {
                    to: Point3::new(entry.x, entry.y, z),
                    feed: op.plunge_feed,
                    tag: plunge,
                });
                crate::leads::emit_lead(
                    &mut program,
                    entry,
                    start,
                    start,
                    out,
                    op.lead_in,
                    z,
                    op.feed,
                    lead,
                );
                crate::emit::cut_polyline(&mut program, &loop_pts, op.feed, cut, z);
                crate::leads::emit_lead(
                    &mut program,
                    exit_on,
                    exit,
                    exit_on,
                    out_exit,
                    op.lead_out,
                    z,
                    op.feed,
                    lead,
                );
                program.push(Step::Rapid {
                    to: Point3::new(exit.x, exit.y, env.heights.clearance),
                    tag: retract,
                });
                prev_z = z;
            }
        }

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

/// The cumulative bevel width at each pass, always ending exactly at `width`.
///
/// - `step <= 0` or `step >= width`: a single pass at the full width.
/// - **uniform** (`!gradual`): equal width increments `step, 2·step, …` — simplest,
///   but the material removed grows each pass (bevel area ∝ width²).
/// - **gradual**: equal *material* per pass, so widths follow `step·√k` (the first
///   pass is `step` wide); the increments shrink as the bevel widens.
fn pass_widths(width: f64, step: f64, gradual: bool) -> Vec<f64> {
    if step <= 0.0 || step >= width {
        return vec![width];
    }
    let mut ws = Vec::new();
    if gradual {
        let mut k = 1.0_f64;
        loop {
            let w = step * k.sqrt();
            if w >= width {
                break;
            }
            ws.push(w);
            k += 1.0;
        }
    } else {
        let mut w = step;
        while w < width {
            ws.push(w);
            w += step;
        }
    }
    // Land the final pass exactly on the target (avoid a duplicate/sliver pass).
    if ws.last().is_some_and(|&l| (width - l).abs() < 1e-9) {
        ws.pop();
    }
    ws.push(width);
    ws
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_cldata::Step;
    use cam_geo::{Contour, Point};
    use cam_model::{Heights, Lead, Tool};

    fn chamfer_tool(included_angle_deg: f64, tip_diameter: f64) -> Tool {
        Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 1,
            kind: ToolKind::ChamferMill {
                included_angle_deg,
                tip_diameter,
            },
        }
    }

    fn square() -> Contour {
        Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(20.0, 0.0),
            Point::new(20.0, 20.0),
            Point::new(0.0, 20.0),
        ])
    }

    fn op(width: f64) -> ChamferOp {
        ChamferOp {
            id: 0,
            tool: 1,
            chain: square(),
            side: Side::Outside,
            width,
            top: 0.0,
            depth: 0.0,
            step: 0.0,
            gradual: false,
            feed: 200.0,
            plunge_feed: 100.0,
            start: None,
            lead_in: Lead::None,
            lead_out: Lead::None,
            lead_overlap: 0.0,
        }
    }

    fn run(op: ChamferOp, tool: Tool) -> StrategyResult {
        let tools = [tool];
        let env = JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &tools,
        };
        ChamferStrategy::new(op).compute(&env, &CancelToken::new())
    }

    fn cut_depth(prog: &Program) -> Option<f64> {
        prog.steps().iter().find_map(|s| match s {
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => Some(to.z),
            _ => None,
        })
    }

    /// A 90° mill (α=45°, tan=1) chamfers to a depth equal to the width.
    #[test]
    fn ninety_degree_depth_equals_width() {
        let r = run(op(1.5), chamfer_tool(90.0, 0.0));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        let z = cut_depth(&r.program).expect("a plunge to depth");
        assert!((z - (-1.5)).abs() < 1e-9, "90° chamfer of 1.5 → depth −1.5, got {z}");
    }

    /// A 60° mill (α=30°, tan≈0.577) cuts deeper than its width for the same bevel.
    #[test]
    fn sharper_tool_cuts_deeper_for_same_width() {
        let r = run(op(1.0), chamfer_tool(60.0, 0.0));
        let z = cut_depth(&r.program).expect("a plunge to depth");
        let expected = -1.0 / (30.0_f64.to_radians().tan());
        assert!((z - expected).abs() < 1e-9, "60° chamfer depth {z} vs {expected}");
    }

    #[test]
    fn non_chamfer_tool_errors() {
        let end_mill = Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
        };
        let r = run(op(1.0), end_mill);
        assert!(r.has_errors());
        assert!(r.program.is_empty());
    }

    #[test]
    fn nonpositive_width_errors() {
        let r = run(op(0.0), chamfer_tool(90.0, 0.0));
        assert!(r.has_errors());
    }

    /// The retract lifts from the start with no overlap, and 2 mm past it (along
    /// the +X first edge) with a 2 mm closure overlap. A sharp V (tip ⌀0) cuts the
    /// chain itself, so the start is (0,0) and the overlap point is exactly (2,0).
    #[test]
    fn overlap_retracts_past_the_start() {
        let retract_xy = |op: ChamferOp| -> Point3 {
            let r = run(op, chamfer_tool(90.0, 0.0));
            assert!(!r.has_errors(), "{:?}", r.diagnostics);
            r.program
                .steps()
                .iter()
                .find_map(|s| match s {
                    Step::Rapid { to, tag } if tag.kind == MoveKind::Retract => Some(*to),
                    _ => None,
                })
                .expect("a retract")
        };

        let at_zero = retract_xy(op(1.5));
        assert!(at_zero.x.abs() < 1e-9 && at_zero.y.abs() < 1e-9, "closes at the start");

        let mut with_overlap = op(1.5);
        with_overlap.lead_overlap = 2.0;
        let past = retract_xy(with_overlap);
        assert!(
            (past.x - 2.0).abs() < 1e-9 && past.y.abs() < 1e-9,
            "should retract 2 mm past the start, got ({}, {})",
            past.x,
            past.y
        );
    }

    /// Arc leads ease the tool onto and off the edge: the plunge moves off the
    /// contour to the lead-in entry, a lead-in arc curves onto the start, and a
    /// lead-out arc curves back off. Sharp V (tip ⌀0) cuts the chain, so the
    /// CCW square starts at (0,0) with tangent +X and outward normal −Y; a radius-2
    /// tangent-arc lead-in therefore enters at (−2,−2).
    #[test]
    fn arc_leads_ease_on_and_off_the_edge() {
        let r = 2.0;
        let mut o = op(1.5);
        o.lead_in = Lead::Arc { radius: r };
        o.lead_out = Lead::Arc { radius: r };
        let result = run(o, chamfer_tool(90.0, 0.0));
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let steps = result.program.steps();

        // The plunge lands off the contour, at the lead-in entry (−2, −2).
        let plunge = steps
            .iter()
            .find_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => Some(*to),
                _ => None,
            })
            .expect("a plunge");
        assert!(
            (plunge.x + r).abs() < 1e-9 && (plunge.y + r).abs() < 1e-9,
            "plunge should sit at the lead-in entry, got ({}, {})",
            plunge.x,
            plunge.y
        );

        // One lead-in arc and one lead-out arc, both tagged as leads.
        let lead_arcs: Vec<Point3> = steps
            .iter()
            .filter_map(|s| match s {
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::LeadIn => Some(*end),
                _ => None,
            })
            .collect();
        assert_eq!(lead_arcs.len(), 2, "a lead-in and a lead-out arc");
        // The lead-in arc lands on the edge start (0,0); cutting begins there.
        assert!(
            lead_arcs[0].x.abs() < 1e-9 && lead_arcs[0].y.abs() < 1e-9,
            "lead-in should end at the edge start, got ({}, {})",
            lead_arcs[0].x,
            lead_arcs[0].y
        );
    }

    fn plunge_count(r: &StrategyResult) -> usize {
        r.program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge))
            .count()
    }

    #[test]
    fn pass_widths_uniform_and_gradual() {
        assert_eq!(pass_widths(3.0, 0.0, false), vec![3.0], "no step ⇒ one pass");
        assert_eq!(pass_widths(3.0, 5.0, false), vec![3.0], "step ≥ width ⇒ one pass");
        assert_eq!(pass_widths(3.0, 1.0, false), vec![1.0, 2.0, 3.0], "uniform steps");

        // Gradual, equal material per pass: widths step·√k, ending on the target.
        // width 3, step 1 ⇒ √1..√9, i.e. exactly 9 passes (√9 = 3).
        let g = pass_widths(3.0, 1.0, true);
        assert_eq!(g.len(), 9, "equal-area passes: {g:?}");
        assert!((g[1] - std::f64::consts::SQRT_2).abs() < 1e-9, "2nd pass at √2");
        assert!((g.last().unwrap() - 3.0).abs() < 1e-9, "ends exactly at width");
    }

    #[test]
    fn deeper_depth_plunges_the_tip_lower() {
        // 90° tool (tan α = 1): the tip's natural depth for a 1.5 mm bevel is 1.5.
        // Setting depth 2.0 plunges the tip 0.5 mm deeper to use a higher flank.
        let base = cut_depth(&run(op(1.5), chamfer_tool(90.0, 0.0)).program).unwrap();
        assert!((base + 1.5).abs() < 1e-9, "tip at bevel bottom by default");

        let mut deep = op(1.5);
        deep.depth = 2.0;
        let z = cut_depth(&run(deep, chamfer_tool(90.0, 0.0)).program).unwrap();
        assert!((z + 2.0).abs() < 1e-9, "tip rides 0.5 mm deeper, got {z}");
    }

    #[test]
    fn depth_below_the_tip_minimum_falls_back_to_the_tip() {
        // depth under width/tanα can't cut the full bevel; clamp to the tip + warn.
        let mut shallow = op(1.5);
        shallow.depth = 0.5;
        let r = run(shallow, chamfer_tool(90.0, 0.0));
        assert!((cut_depth(&r.program).unwrap() + 1.5).abs() < 1e-9, "clamped to the tip");
        assert!(!r.diagnostics.is_empty(), "warns about the under-set depth");
    }

    #[test]
    fn stepping_cuts_one_loop_per_pass() {
        // width 3, step 1, uniform ⇒ 3 passes (one plunge each); the last reaches −3.
        let mut o = op(3.0);
        o.step = 1.0;
        let r = run(o, chamfer_tool(90.0, 0.0));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        assert_eq!(plunge_count(&r), 3, "one plunge per pass");
        // Passes go shallow→deep: first at −1, last at −3.
        let plunge_zs: Vec<f64> = r
            .program
            .steps()
            .iter()
            .filter_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => Some(to.z),
                _ => None,
            })
            .collect();
        assert!((plunge_zs[0] + 1.0).abs() < 1e-9, "first pass shallow at −1");
        assert!((plunge_zs[2] + 3.0).abs() < 1e-9, "final pass at the full depth −3");
    }

    #[test]
    fn gradual_makes_more_but_gentler_passes() {
        let mut o = op(3.0);
        o.step = 1.0;
        o.gradual = true;
        let r = run(o, chamfer_tool(90.0, 0.0));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        assert_eq!(plunge_count(&r), 9, "equal-area gradual ⇒ 9 passes for 3 mm @ step 1");
    }
}
