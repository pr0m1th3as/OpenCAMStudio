//! The drilling strategy: emit a Tier-2 [`Drill`](cam_cldata::Step::Drill) cycle
//! intent. It stays high-level on purpose — each post lowers it per its
//! capabilities (a canned `G83` on Fanuc, explicit pecks on grbl).

use cam_cldata::{DrillCycle, MoveKind, Program, Step, Tag};
use cam_model::DrillOp;

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

    fn compute(&self, env: &JobEnv, _cancel: &CancelToken) -> StrategyResult {
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
        // Drilling is a pure plunge: the surface on the axis must cut, and the hole
        // must not run deeper than the cutting edge.
        if !crate::guards::check_plunge(op.id, "drill", tool, &mut diagnostics)
            || !crate::guards::check_axial_reach(op.id, "drill", tool, op.depth, &mut diagnostics)
        {
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        // A centre-cutting mill *can* be plunged, but it is not a drill: no point
        // geometry to centre the hole, and full-diameter engagement all the way.
        if !matches!(tool.kind, cam_model::ToolKind::Drill { .. }) {
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
        if bottom >= env.heights.top_of_stock {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: the hole bottom {bottom} does not reach below the stock top {}; nothing to drill",
                op.id, env.heights.top_of_stock
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        if matches!(op.peck, Some(p) if p <= 0.0) {
            diagnostics.push(Diagnostic::error(format!(
                "operation {}: peck increment must be positive",
                op.id
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }

        let mut program = Program::new();
        program.push(Step::Drill(DrillCycle {
            points: op.points.clone(),
            z_top: hole_top,
            depth: bottom,
            retract: env.heights.retract,
            peck: op.peck,
            dwell: op.dwell,
            feed: op.feed,
            tag: Tag::new(op.id, MoveKind::Plunge),
        }));

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Severity;
    use cam_cldata::Step;
    use cam_model::{Heights, Tool, ToolKind};

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
            spindle_rpm: 0.0,
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
        };
        let res = DrillStrategy::new(op).compute(&env, &CancelToken::new());
        assert!(
            !res.diagnostics.iter().any(|d| d.severity == Severity::Error),
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
}
