//! The facing strategy: clear the top of the stock with parallel passes.
//!
//! Passes run along X, stepping over in Y. Each pass line is clipped to the
//! boundary (grown outward by the tool radius, so the tool overhangs the edges
//! and faces the whole surface), and the in-region pieces are cut. Passes
//! alternate direction (a zig-zag).

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{clip_path, offset, JoinStyle, Point, Polygon, Polyline};
use cam_model::{FaceOp, Heights};

use crate::profile::depth_levels;
use crate::{CancelToken, Diagnostic, JobEnv, Strategy, StrategyResult};

/// Faces a surface. Construct from a [`FaceOp`].
#[derive(Clone, Debug)]
pub struct FaceStrategy {
    op: FaceOp,
}

impl FaceStrategy {
    /// Build a facing strategy for `op`.
    pub fn new(op: FaceOp) -> Self {
        Self { op }
    }
}

impl Strategy for FaceStrategy {
    fn name(&self) -> &str {
        "face"
    }

    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        let op = &self.op;
        let mut diagnostics = Vec::new();

        macro_rules! fail {
            ($($arg:tt)*) => {{
                diagnostics.push(Diagnostic::error(format!($($arg)*)));
                return StrategyResult { diagnostics, ..Default::default() };
            }};
        }

        let Some(tool) = env.tool(op.tool) else {
            fail!("operation {}: tool {} is not in the setup", op.id, op.tool);
        };
        if !op.boundary.is_valid() {
            fail!("operation {}: face boundary is not a closed area", op.id);
        }
        if op.stepover <= 0.0 || op.stepdown <= 0.0 {
            fail!(
                "operation {}: stepover and stepdown must be positive",
                op.id
            );
        }
        if op.depth >= env.heights.top_of_stock {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: depth is at or above the stock top; nothing to face",
                op.id
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }

        let region = match Polygon::new(op.boundary.clone()) {
            Ok(p) => p,
            Err(e) => fail!("operation {}: invalid face boundary: {e}", op.id),
        };
        // Grow the boundary by the tool radius so passes reach the edges.
        let grown = match offset(
            std::slice::from_ref(&region),
            tool.radius(),
            JoinStyle::Round,
        ) {
            Ok(v) if !v.is_empty() => v,
            Ok(_) => fail!("operation {}: face region vanished under offset", op.id),
            Err(e) => fail!("operation {}: offset failed: {e}", op.id),
        };

        let (min, max) = bounds(&grown);
        let mut ys = Vec::new();
        let mut y = min.1 + op.stepover * 0.5;
        while y < max.1 {
            ys.push(y);
            y += op.stepover;
        }
        if ys.is_empty() {
            ys.push(0.5 * (min.1 + max.1));
        }

        let levels = depth_levels(env.heights.top_of_stock, op.depth, op.stepdown);
        let mut program = Program::new();
        for &z in &levels {
            if cancel.is_cancelled() {
                return StrategyResult {
                    program,
                    diagnostics,
                    cancelled: true,
                };
            }
            let mut forward = true;
            for &yp in &ys {
                let (x0, x1) = if forward {
                    (min.0, max.0)
                } else {
                    (max.0, min.0)
                };
                let line = Polyline::new(vec![Point::new(x0, yp), Point::new(x1, yp)]);
                let pieces = match clip_path(&line, &grown, true) {
                    Ok(p) => p,
                    Err(e) => fail!("operation {}: clip failed: {e}", op.id),
                };
                for seg in &pieces {
                    emit_pass(&mut program, seg.points(), op, &env.heights, z);
                }
                forward = !forward;
            }
        }

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

/// XY bounding box over the outer boundaries of some regions, as `(min, max)`.
fn bounds(regions: &[Polygon]) -> ((f64, f64), (f64, f64)) {
    let mut min = (f64::MAX, f64::MAX);
    let mut max = (f64::MIN, f64::MIN);
    for region in regions {
        for p in region.outer().points() {
            min.0 = min.0.min(p.x);
            min.1 = min.1.min(p.y);
            max.0 = max.0.max(p.x);
            max.1 = max.1.max(p.y);
        }
    }
    (min, max)
}

/// Emit approach, plunge, one straight pass, and retract.
fn emit_pass(prog: &mut Program, pts: &[Point], op: &FaceOp, h: &Heights, z: f64) {
    if pts.len() < 2 {
        return;
    }
    let start = pts[0];
    let end = *pts.last().unwrap();
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.clearance),
        tag: Tag::new(op.id, MoveKind::Link),
    });
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.top_of_stock),
        tag: Tag::new(op.id, MoveKind::Link),
    });
    prog.push(Step::Linear {
        to: Point3::new(start.x, start.y, z),
        feed: op.plunge_feed,
        tag: Tag::new(op.id, MoveKind::Plunge),
    });
    for p in &pts[1..] {
        prog.push(Step::Linear {
            to: Point3::new(p.x, p.y, z),
            feed: op.feed,
            tag: Tag::new(op.id, MoveKind::Cutting),
        });
    }
    prog.push(Step::Rapid {
        to: Point3::new(end.x, end.y, h.clearance),
        tag: Tag::new(op.id, MoveKind::Retract),
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_geo::Contour;
    use cam_model::{Tool, ToolKind};

    #[test]
    fn faces_a_rectangle_in_parallel_passes() {
        // 60×40 face, ⌀10 tool (r5) ⇒ grown region ~70×50, 6 mm stepover ⇒
        // passes at y = -2, 4, …, 40 (8 of them); depth -2 at stepdown 2 ⇒ 1
        // level ⇒ 8 plunges.
        let op = FaceOp {
            id: 0,
            tool: 1,
            boundary: Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(60.0, 0.0),
                Point::new(60.0, 40.0),
                Point::new(0.0, 40.0),
            ]),
            depth: -2.0,
            stepdown: 2.0,
            stepover: 6.0,
            feed: 400.0,
            plunge_feed: 150.0,
        };
        let tools = [Tool {
            number: 1,
            diameter: 10.0,
            flutes: 2,
            kind: ToolKind::EndMill,
        }];
        let env = JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &tools,
        };
        let result = FaceStrategy::new(op).compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let plunges = result
            .program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge))
            .count();
        assert_eq!(plunges, 8, "one plunge per parallel pass");
    }
}
