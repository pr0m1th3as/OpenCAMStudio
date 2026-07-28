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
//!
//! ## Chamfer mill *or* V-bit
//!
//! Either tool chamfers, because a chamfer is cut by the **flank**, which both have.
//! They differ only in what sits at the bottom: a chamfer mill has a flat tip of
//! radius `rt`, a V-bit a *rounded* one. Extending the V-bit's flank line down to the
//! tool's lowest point gives an equivalent flat radius of `rt·(1 − sin α)/cos α` — the
//! ball tucks **under** the flank — after which every formula here is unchanged.
//! (Note the asymmetry: engraving does *not* accept a chamfer mill, because it cuts
//! with the tip, which on a chamfer mill does not cut at all.)

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, Contour, JoinStyle, Polygon};
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

        // A chamfer needs an angled tool — the bevel angle comes from the tool. Both a
        // chamfer mill and a **V-bit** qualify: a chamfer is cut by the *flank*, which
        // both have. (The reverse is not true — engraving needs a cutting tip, so it
        // rejects a chamfer mill; see [`crate::engrave`].)
        //
        // `tip_offset` is the radius at which the cutting flank meets the tool's
        // lowest point — how far the axis must clear the corner:
        //
        // - **chamfer mill**: the flat tip's radius, directly.
        // - **V-bit**: the flat that its *rounded* tip is equivalent to. Extending the
        //   flank line `r = rt·cos α + (z − z_t)·tan α` down to `z = 0` (with the
        //   tangent point `z_t = rt·(1 − sin α)`) gives an intercept of
        //   `rt·(1 − sin α)/cos α` — so the ball tucks *under* the flank and the tool
        //   sits closer to the edge than its tip radius would suggest. Every formula
        //   below then carries over unchanged, since the flank line is identical.
        let (included_angle_deg, tip_offset) = match tool.kind {
            ToolKind::ChamferMill {
                included_angle_deg,
                tip_diameter,
            } => (included_angle_deg, 0.5 * tip_diameter),
            ToolKind::VBit {
                included_angle_deg,
                tip_radius,
            } => {
                let a = 0.5 * included_angle_deg.to_radians();
                let cos_a = a.cos();
                let equiv = if cos_a > 1e-9 {
                    tip_radius * (1.0 - a.sin()) / cos_a
                } else {
                    0.0
                };
                (included_angle_deg, equiv)
            }
            _ => {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: tool {} is a {}; a chamfer needs an angled tool \
                     (a chamfer mill or a V-bit) whose flank forms the bevel",
                    op.id, op.tool, tool.kind
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

        // The bevel is cut by the flank between the tip and `tip_depth` above it, so
        // that whole section must still be cutting surface. This is the upper bound on
        // tip depth that was previously missing (there was no flank-length geometry to
        // check against); the generatrix now supplies it.
        if !crate::guards::check_axial_reach(op.id, "chamfer", tool, tip_depth, &mut diagnostics) {
            bail!();
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
        let off = delta * tan_a + tip_offset;
        let signed = match op.side {
            Side::Outside => off,
            Side::Inside => -off,
            Side::On => 0.0,
        };
        // The cleared/air side the lead eases in from: outward for an external edge,
        // the bore interior for an internal one (flip the normal, or the lead swings
        // onto the material side and gouges).
        let air_sign = if op.side == Side::Inside { -1.0 } else { 1.0 };
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
            "Chamfer: {:.3} mm wide at {}\u{00b0} in {}",
            op.width,
            included_angle_deg,
            crate::passes_phrase(widths.len())
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
            let out = {
                let o = crate::profile::outward_normal(pts);
                (o.0 * air_sign, o.1 * air_sign)
            };
            // An internal chamfer eases in from the bounded bore interior — guard the
            // lead against overshooting it; an external one leads into open air.
            let guard: Vec<Polygon> = if air_sign < 0.0 {
                Polygon::new(Contour::new(pts.to_vec())).into_iter().collect()
            } else {
                Vec::new()
            };
            let (loop_pts, exit_on, tan_out) =
                crate::emit::loop_with_overlap(pts, op.lead_overlap);
            // The lead-off normal follows the arrival tangent (see the profile note):
            // at a corner start it differs from the start's, so deriving it here avoids
            // a degenerate lead; mid-edge the two coincide.
            let out_exit = {
                let o = crate::profile::outward_normal_at(tan_out, crate::profile::is_ccw(pts));
                (o.0 * air_sign, o.1 * air_sign)
            };
            let lead_in = crate::leads::guard_lead(&guard, start, tan_in, out, op.lead_in, true);
            let lead_out =
                crate::leads::guard_lead(&guard, exit_on, tan_out, out_exit, op.lead_out, false);
            let entry = crate::leads::lead_start_point(start, tan_in, out, lead_in);
            let exit = crate::leads::lead_end_point(exit_on, tan_out, out_exit, lead_out);

            // One pass per cumulative width, shallow to deep. Each: rapid over the
            // entry at clearance and down through the already-cut air, plunge to the
            // pass depth, lead on, cut the loop (+ overlap), lead off, retract.
            // The first pass rapids down to the **retract plane**, not to the stock top:
    // ending a rapid exactly on the surface leaves no margin, so slightly proud
    // stock or a small Z-zero error means rapiding into material. Taking the higher
    // of the two is never lower than the old behaviour. Later passes still rapid to
    // the previous depth — that is through air the tool has already cut.
            let mut prev_z = env.heights.retract.max(op.top);
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
                    lead_in,
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
                    lead_out,
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

    fn vbit_tool(included_angle_deg: f64, tip_radius: f64) -> Tool {
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
            ..Default::default()
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
            spindle_rpm: 0.0,
            work_offset: 1,
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
            stock: None,
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
            ..Default::default()
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

    #[test]
    fn internal_chamfer_leads_ease_in_from_the_bore_not_the_wall() {
        // An internal chamfer on a ⌀20 bore (the [0,20] edge) with a sharp V cuts the
        // edge itself. Starting mid-edge (top, x=10), the arc lead must ease in from
        // the bore interior (nearer the centre 10,10), not swing out past the wall.
        let mut o = op(1.5);
        o.side = Side::Inside;
        o.start = Some([10.0, 20.0]);
        o.lead_in = Lead::Arc { radius: 2.0 };
        o.lead_out = Lead::Arc { radius: 2.0 };
        let r = run(o, chamfer_tool(90.0, 0.0));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        // The plunge lands at the lead-in entry; it must sit inside the bore [0,20]
        // and on the interior side of the (10,20) edge point (y < 20, toward centre).
        let plunge = r
            .program
            .steps()
            .iter()
            .find_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => Some(*to),
                _ => None,
            })
            .expect("a plunge at the lead entry");
        assert!(
            plunge.y < 20.0 - 1e-6 && (0.0..=20.0).contains(&plunge.x) && plunge.y >= 0.0,
            "internal chamfer lead must enter from the bore, got ({}, {})",
            plunge.x,
            plunge.y
        );
        let leads = r
            .program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Arc { tag, .. } if tag.kind == MoveKind::LeadIn))
            .count();
        assert_eq!(leads, 2, "a lead-in and a lead-out on the bore edge");
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


    fn xy_of_first_cut(prog: &Program) -> [f64; 2] {
        prog.steps()
            .iter()
            .find_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some([to.x, to.y]),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => Some([end.x, end.y]),
                _ => None,
            })
            .expect("a cutting move")
    }

    #[test]
    fn a_vbit_is_accepted_for_chamfering() {
        let r = run(op(1.0), vbit_tool(90.0, 0.0));
        assert!(
            r.diagnostics
                .iter()
                .all(|d| d.severity != crate::Severity::Error),
            "{:?}",
            r.diagnostics
        );
        assert!(!r.program.steps().is_empty());
    }

    #[test]
    fn a_sharp_vbit_matches_a_sharp_chamfer_mill_exactly() {
        // Same flank, no tip on either → byte-identical geometry.
        let a = run(op(1.0), vbit_tool(90.0, 0.0));
        let b = run(op(1.0), chamfer_tool(90.0, 0.0));
        assert_eq!(a.program.steps(), b.program.steps());
    }

    #[test]
    fn a_rounded_tip_is_equivalent_to_a_smaller_flat_not_its_own_radius() {
        // The bug this guards: treating the V-bit's tip radius as if it were a flat
        // radius would offset the tool too far from the edge, cutting a narrow bevel.
        // The correct equivalent flat is rt·(1 − sin α)/cos α, which is strictly
        // smaller (the ball tucks under the flank).
        let (deg, rt) = (90.0_f64, 0.5_f64);
        let a = 0.5 * deg.to_radians();
        let equiv = rt * (1.0 - a.sin()) / a.cos();
        assert!(equiv < rt, "equiv {equiv} should tuck under rt {rt}");

        let got = xy_of_first_cut(&run(op(1.0), vbit_tool(deg, rt)).program);
        let want = xy_of_first_cut(&run(op(1.0), chamfer_tool(deg, 2.0 * equiv)).program);
        let wrong = xy_of_first_cut(&run(op(1.0), chamfer_tool(deg, 2.0 * rt)).program);
        assert!(
            (got[0] - want[0]).abs() < 1e-9 && (got[1] - want[1]).abs() < 1e-9,
            "got {got:?} want {want:?}"
        );
        assert!(
            (got[0] - wrong[0]).abs() > 1e-6 || (got[1] - wrong[1]).abs() > 1e-6,
            "the naive tip-radius-as-flat reading must differ"
        );
    }

    #[test]
    fn a_flat_end_mill_is_still_rejected() {
        let mut t = chamfer_tool(90.0, 0.0);
        t.kind = ToolKind::EndMill;
        let r = run(op(1.0), t);
        let errs: Vec<_> = r
            .diagnostics
            .iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert_eq!(errs.len(), 1, "{:?}", r.diagnostics);
        assert!(errs[0].message.contains("chamfer mill or a V-bit"));
    }
}
