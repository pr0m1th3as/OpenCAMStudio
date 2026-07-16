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

        if env.tool(op.tool).is_none() {
            diagnostics.push(Diagnostic::error(format!(
                "operation {} references tool {} which is not in the setup",
                op.id, op.tool
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
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
        // `depth` is a positive magnitude below the reference; the bottom is Z = -depth.
        let bottom = -op.depth;
        if bottom >= env.heights.top_of_stock {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: depth {} does not reach below the stock top {}; nothing to drill",
                op.id, op.depth, env.heights.top_of_stock
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
            z_top: env.heights.top_of_stock,
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
