//! The profiling strategy: follow a closed chain at a tool-radius offset, in
//! stepdown passes, down to depth.

use cam_cldata::{CutterComp, MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, JoinStyle, Polygon};
use cam_model::{Comp, Heights, ProfileOp, Side};

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
    let start = pts[0];
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
}
