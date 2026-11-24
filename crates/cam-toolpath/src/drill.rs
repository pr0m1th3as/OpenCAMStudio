//! Simple and legacy drilling retain Tier-2 [`Drill`](cam_cldata::Step::Drill)
//! cycles. Configurable peck schedules produce explicit motion so posts and
//! simulation share the same depth progression and withdrawals.

use cam_cldata::{DrillCycle, MoveKind, Point3, Program, Step, Tag};
use cam_model::{CuttingHeights, DrillCycleMode, DrillCycleSettings, DrillOp, Tool, ToolKind};

use crate::{CancelToken, Diagnostic, JobEnv, Strategy, StrategyResult};

/// Drills a set of holes. Construct from a [`DrillOp`].
#[derive(Clone, Debug)]
pub struct DrillStrategy {
    op: DrillOp,
}

impl DrillStrategy {
    /// Build a drilling strategy for `op`.
    pub fn new(op: DrillOp) -> Self {
        Self { op }
    }
}

impl Strategy for DrillStrategy {
    fn name(&self) -> &str {
        "drill"
    }

    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        if self.op.parameter_expressions.is_empty() {
            return self.compute_resolved_parameters(env, cancel);
        }
        let mut operation = cam_model::Operation::Drill(self.op.clone());
        if let Err(message) = operation.resolve_parameter_expressions(env.tools) {
            return StrategyResult {
                diagnostics: vec![crate::Diagnostic::error(message)],
                ..Default::default()
            };
        }
        let cam_model::Operation::Drill(op) = operation else {
            unreachable!()
        };
        Self::new(op).compute_resolved_parameters(env, cancel)
    }
}
impl DrillStrategy {
    fn compute_resolved_parameters(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        let op = &self.op;
        let mut diagnostics = Vec::new();

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
        let tip_extension = if op.tip_through_bottom {
            let Some(height) = drill_tip_height(tool) else {
                return StrategyResult {
                    diagnostics: vec![Diagnostic::error(format!(
                        "Operation {}: Drill Tip Through Bottom requires a drill with a valid point that cuts to its full diameter",
                        op.id
                    ))],
                    ..Default::default()
                };
            };
            height
        } else {
            0.0
        };
        // Report plunge and reach suitability without suppressing the path.
        crate::guards::warn_plunge(op.id, "drill", tool, &mut diagnostics);
        let max_depth = op
            .hole_heights
            .iter()
            .copied()
            .map(CuttingHeights::depth)
            .reduce(f64::max)
            .unwrap_or(op.depth);
        crate::guards::warn_axial_reach(
            op.id,
            "drill",
            tool,
            max_depth + tip_extension,
            &mut diagnostics,
        );
        // A centre-cutting mill *can* be plunged, but it is not a drill: no point
        // geometry to centre the hole, and full-diameter engagement all the way.
        if !matches!(tool.kind, ToolKind::Drill { .. }) {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: tool {} is a {} being plunged as a drill — it will cut,                  but without a drill point the hole may wander and the full diameter                  engages at once.",
                op.id, op.tool, tool.kind
            )));
        }
        if op.points.is_empty() {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: no holes to drill",
                op.id
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        // Consistent with Face: `start_offset` raises the hole's top plane *above*
        // the stock top (Z0 by convention) — positive starts it above the surface
        // (a proud boss), negative below (a recessed/faced surface) — and `depth` is
        // measured down from there, so the bottom is at `top + offset - depth`.
        let hole_top = env.heights.top_of_stock + op.start_offset;
        let bottom = hole_top - op.depth;
        if op.hole_heights.is_empty() && bottom - tip_extension >= env.heights.top_of_stock {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: the hole bottom {bottom} does not reach below the stock top {}; nothing to drill",
                op.id, env.heights.top_of_stock
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        let mode = op.cycle.mode;
        let peck = match mode {
            None | Some(DrillCycleMode::ChipBreaking | DrillCycleMode::DeepDrilling) => op.peck,
            _ => None,
        };
        let dwell = match mode {
            Some(DrillCycleMode::RapidOut) => None,
            Some(DrillCycleMode::Counterboring) => Some(op.dwell.unwrap_or(0.0)),
            _ => op.dwell,
        };
        let expanded_peck = mode.is_some_and(DrillCycleMode::pecks);
        if matches!(peck, Some(p) if !p.is_finite() || p <= 0.0)
            || (expanded_peck && peck.is_none())
        {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: peck increment must be positive",
                op.id
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        if expanded_peck && !valid_peck_settings(op.cycle, peck.unwrap()) {
            diagnostics.push(Diagnostic::error(format!(
                "Operation {}: use a positive minimum peck no greater than Pecking Depth, a nonnegative reduction and accumulation, and a positive Chip Break Distance",
                op.id
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }

        let default_heights = CuttingHeights {
            top: hole_top,
            bottom,
        };
        let mut holes: Vec<_> = op
            .points
            .iter()
            .copied()
            .enumerate()
            .map(|(index, point)| {
                (
                    point,
                    op.hole_heights
                        .get(index)
                        .copied()
                        .unwrap_or(default_heights),
                )
            })
            .collect();
        if (!op.hole_heights.is_empty() || expanded_peck)
            && ((!op.hole_heights.is_empty() && op.hole_heights.len() != op.points.len())
                || holes.iter().any(|(_, heights)| {
                    heights.validate().is_err()
                        || heights.top > env.heights.retract
                        || heights.top > env.heights.clearance
                }))
        {
            diagnostics.push(Diagnostic::error(format!(
                "Operation {}: each drill hole requires a valid cutting interval at or below clearance and retract",
                op.id
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        if op.tip_through_bottom {
            for (_, heights) in &mut holes {
                let tip_bottom = heights.bottom - tip_extension;
                if heights.validate().is_err()
                    || !tip_bottom.is_finite()
                    || tip_bottom >= heights.bottom
                {
                    diagnostics.push(Diagnostic::error(format!(
                        "Operation {}: cannot extend the drill tip below the programmed bottom",
                        op.id
                    )));
                    return StrategyResult {
                        diagnostics,
                        ..Default::default()
                    };
                }
                heights.bottom = tip_bottom;
            }
        }
        holes.sort_by(|a, b| {
            if op.geometry.order_by_depth {
                a.1.top.total_cmp(&b.1.top)
            } else {
                b.1.top.total_cmp(&a.1.top)
            }
        });
        let mut start = 0;
        while start < holes.len() {
            let top = holes[start].1.top;
            let end = start
                + 1
                + holes[start + 1..].partition_point(|hole| (hole.1.top - top).abs() < 1e-5);
            crate::bore::order_holes(
                &mut holes[start..end],
                op.geometry.order,
                |hole| hole.0,
                cancel,
            );
            if cancel.is_cancelled() {
                return StrategyResult {
                    diagnostics,
                    cancelled: true,
                    ..Default::default()
                };
            }
            start = end;
        }
        if op.geometry.reverse_order {
            holes.reverse();
        }
        let mut program = Program::new();
        let mut remaining_pecks = 100_000;
        for group in holes.chunk_by(|a, b| a.1 == b.1) {
            let cycle = DrillCycle {
                points: group.iter().map(|hole| hole.0).collect(),
                z_top: group[0].1.top,
                depth: group[0].1.bottom,
                retract: env.heights.retract,
                peck,
                dwell,
                feed: op.feed,
                tag: Tag::new(op.id, MoveKind::Plunge),
            };
            if expanded_peck {
                if let Err(reason) =
                    expand_pecks(&mut program, &cycle, op.cycle, &mut remaining_pecks, cancel)
                {
                    diagnostics.push(Diagnostic::error(format!("Operation {}: {reason}", op.id)));
                    return StrategyResult {
                        diagnostics,
                        ..Default::default()
                    };
                }
                if cancel.is_cancelled() {
                    return StrategyResult {
                        diagnostics,
                        cancelled: true,
                        ..Default::default()
                    };
                }
            } else {
                program.push(Step::Drill(cycle));
            }
        }

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

fn valid_peck_settings(settings: DrillCycleSettings, first_peck: f64) -> bool {
    let reduction_ok = settings.peck_reduction.is_finite() && settings.peck_reduction >= 0.0;
    let minimum_ok = settings.minimum_peck > 0.0 && settings.minimum_peck <= first_peck;
    let chip_break_ok = settings.mode != Some(DrillCycleMode::ChipBreaking)
        || (settings.accumulated_peck.is_finite()
            && settings.accumulated_peck >= 0.0
            && settings.chip_break_distance.is_finite()
            && settings.chip_break_distance > 0.0);
    reduction_ok && minimum_ok && chip_break_ok
}

fn expand_pecks(
    program: &mut Program,
    cycle: &DrillCycle,
    settings: DrillCycleSettings,
    remaining_pecks: &mut usize,
    cancel: &CancelToken,
) -> Result<(), String> {
    const REENTRY_CLEARANCE: f64 = 0.5;
    let full_retract = settings.mode == Some(DrillCycleMode::DeepDrilling);
    let rapid = |to, kind| Step::Rapid {
        to,
        tag: Tag { kind, ..cycle.tag },
    };
    for &[x, y] in &cycle.points {
        program.push(rapid(Point3::new(x, y, cycle.retract), MoveKind::Link));
        let mut previous = cycle.z_top;
        let mut peck = cycle.peck.unwrap();
        let mut last_full_retract = cycle.z_top;
        while previous > cycle.depth {
            if cancel.is_cancelled() {
                return Ok(());
            }
            *remaining_pecks = remaining_pecks
                .checked_sub(1)
                .ok_or("Drilling exceeds 100000 pecks; increase the peck depth")?;
            let target = (previous - peck).max(cycle.depth);
            if target >= previous {
                return Err("Drill peck cannot advance at this depth".into());
            }
            program.push(Step::Linear {
                to: Point3::new(x, y, target),
                feed: cycle.feed,
                tag: cycle.tag,
            });
            let finished = target == cycle.depth;
            if finished {
                if let Some(seconds) = cycle.dwell.filter(|&seconds| seconds != 0.0) {
                    program.push(Step::Dwell { seconds });
                }
            }
            let accumulated = settings.accumulated_peck > 0.0
                && last_full_retract - target >= settings.accumulated_peck - 1e-9;
            if finished || full_retract || accumulated {
                program.push(rapid(Point3::new(x, y, cycle.retract), MoveKind::Retract));
                if !finished {
                    let resume = (target + REENTRY_CLEARANCE).min(cycle.retract);
                    program.push(rapid(Point3::new(x, y, resume), MoveKind::Link));
                }
                last_full_retract = target;
            } else {
                let withdraw = (target + settings.chip_break_distance).min(cycle.retract);
                if withdraw <= target {
                    return Err("Chip Break Distance cannot retract at this depth".into());
                }
                program.push(rapid(Point3::new(x, y, withdraw), MoveKind::Retract));
            }
            previous = target;
            peck = (peck - settings.peck_reduction).max(settings.minimum_peck);
        }
    }
    Ok(())
}

fn drill_tip_height(tool: &Tool) -> Option<f64> {
    let ToolKind::Drill { point_angle_deg } = tool.kind else {
        return None;
    };
    let radius = tool.radius();
    if !radius.is_finite() || radius <= 0.0 || !(0.0 < point_angle_deg && point_angle_deg < 180.0) {
        return None;
    }
    tool.profile()
        .segs
        .into_iter()
        .find(|segment| segment.end.x == radius)
        .filter(|segment| segment.cutting && segment.end.y.is_finite() && segment.end.y > 0.0)
        .map(|segment| segment.end.y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Severity;
    use cam_cldata::Step;
    use cam_model::Heights;

    fn tool() -> Tool {
        Tool {
            number: 1,
            diameter: 5.0,
            length: 40.0,
            flutes: 2,
            kind: ToolKind::Drill {
                point_angle_deg: 118.0,
            },
            ..Default::default()
        }
    }

    fn op(depth: f64, start_offset: f64) -> DrillOp {
        DrillOp {
            cycle: Default::default(),
            tip_through_bottom: false,
            geometry: Default::default(),
            hole_heights: Vec::new(),
            parameter_expressions: Default::default(),
            spindle_rpm: 0.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            id: 7,
            tool: 1,
            points: vec![[10.0, 20.0]],
            depth,
            start_offset,
            peck: None,
            dwell: None,
            feed: 100.0,
        }
    }

    /// Extract `(z_top, bottom)` from the single emitted drill cycle.
    fn cycle_planes(op: DrillOp) -> (f64, f64) {
        let tools = [tool()];
        let env = JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &tools,
            stock: None,
            spindle: cam_cldata::SpindleDir::Cw,
        };
        let res = DrillStrategy::new(op).compute(&env, &CancelToken::new());
        assert!(
            !res.diagnostics
                .iter()
                .any(|d| d.severity == Severity::Error),
            "no errors"
        );
        match res.program.steps().iter().find_map(|s| match s {
            Step::Drill(c) => Some(c),
            _ => None,
        }) {
            Some(c) => (c.z_top, c.depth),
            None => panic!("no drill cycle emitted"),
        }
    }

    #[test]
    fn no_offset_starts_at_the_stock_top_and_measures_depth_from_there() {
        // top_of_stock = 0: the classic case, unchanged from before start_offset.
        let (z_top, bottom) = cycle_planes(op(8.0, 0.0));
        assert_eq!(z_top, 0.0);
        assert_eq!(bottom, -8.0);
    }

    #[test]
    fn a_positive_offset_starts_above_the_stock_top() {
        // A proud boss 3 mm above the stock top (Face convention): the whole hole
        // shifts up, and it is still `depth` deep from its start.
        let (z_top, bottom) = cycle_planes(op(8.0, 3.0));
        assert_eq!(z_top, 3.0, "starts 3 mm above the stock top");
        assert_eq!(bottom, -5.0, "8 mm deep from the +3 start");
    }

    #[test]
    fn a_negative_offset_starts_below_the_stock_top() {
        // Drilling from a surface 2 mm below the stock top (recessed/faced).
        let (z_top, bottom) = cycle_planes(op(8.0, -2.0));
        assert_eq!(z_top, -2.0);
        assert_eq!(bottom, -10.0);
    }

    #[test]
    fn drill_tip_through_bottom_compensates_common_and_individual_bottoms() {
        let mut drill = op(8.0, 0.0);
        drill.tip_through_bottom = true;
        let tip_height = 2.5 / 59.0_f64.to_radians().tan();
        assert_eq!(cycle_planes(drill.clone()), (0.0, -8.0 - tip_height));

        drill.hole_heights = vec![CuttingHeights {
            top: -1.0,
            bottom: -10.0,
        }];
        assert_eq!(cycle_planes(drill), (-1.0, -10.0 - tip_height));
    }

    #[test]
    fn drill_tip_through_bottom_requires_a_full_diameter_cutting_point() {
        let mut drill = op(8.0, 0.0);
        drill.tip_through_bottom = true;
        for tool in [
            Tool {
                kind: ToolKind::EndMill,
                ..tool()
            },
            Tool {
                flute_length: 0.5,
                ..tool()
            },
        ] {
            let env = JobEnv {
                heights: Heights::new(5.0, 2.0, 0.0),
                tools: &[tool],
                stock: None,
                spindle: cam_cldata::SpindleDir::Cw,
            };
            let result = DrillStrategy::new(drill.clone()).compute(&env, &CancelToken::new());
            assert!(result.has_errors());
            assert!(result.program.is_empty());
        }
    }

    #[test]
    fn explicit_drill_cycles_override_inactive_dwell_and_peck_values() {
        let tools = [tool()];
        let env = JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &tools,
            stock: None,
            spindle: cam_cldata::SpindleDir::Cw,
        };
        let mut drill = op(3.0, 0.0);
        drill.peck = Some(1.0);
        drill.dwell = Some(0.25);
        for (mode, dwell) in [
            (DrillCycleMode::RapidOut, None),
            (DrillCycleMode::Counterboring, Some(0.25)),
        ] {
            drill.set_cycle_mode(mode);
            let result = DrillStrategy::new(drill.clone()).compute(&env, &CancelToken::new());
            assert!(matches!(
                result.program.steps(),
                [Step::Drill(cycle)] if cycle.peck.is_none() && cycle.dwell == dwell
            ));
        }
    }

    #[test]
    fn peck_modes_use_reduction_minimum_and_the_selected_retract_policy() {
        let tools = [tool()];
        let env = JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &tools,
            stock: None,
            spindle: cam_cldata::SpindleDir::Cw,
        };
        let mut drill = op(3.25, 0.0);
        drill.peck = Some(1.25);
        drill.dwell = Some(0.25);
        drill.cycle.minimum_peck = 0.5;
        drill.cycle.peck_reduction = 0.5;
        drill.cycle.accumulated_peck = 2.0;
        for (mode, expected_retracts) in [
            (DrillCycleMode::ChipBreaking, [-1.15, 2.0, -2.4, -2.9, 2.0]),
            (DrillCycleMode::DeepDrilling, [2.0; 5]),
        ] {
            drill.set_cycle_mode(mode);
            let result = DrillStrategy::new(drill.clone()).compute(&env, &CancelToken::new());
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
            let mut depths = Vec::new();
            let mut retracts = Vec::new();
            let mut dwell = Vec::new();
            for step in result.program.steps() {
                match step {
                    Step::Linear { to, .. } => depths.push(to.z),
                    Step::Rapid { to, tag } if tag.kind == MoveKind::Retract => retracts.push(to.z),
                    Step::Dwell { seconds } => dwell.push((depths.last().copied(), *seconds)),
                    _ => {}
                }
            }
            assert_eq!(depths, [-1.25, -2.0, -2.5, -3.0, -3.25]);
            assert_eq!(retracts.len(), expected_retracts.len());
            for (actual, expected) in retracts.iter().zip(expected_retracts) {
                assert!((actual - expected).abs() < 1e-9, "{retracts:?}");
            }
            assert_eq!(dwell, [(Some(-3.25), 0.25)]);
        }
    }
}
