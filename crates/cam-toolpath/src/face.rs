//! The facing strategy: clear the top of the stock with parallel passes.
//!
//! Passes run along the chosen axis, stepping over by `diameter·(1−overlap)`. The
//! whole surface is cut as a **continuous serpentine**: the tool plunges once per
//! depth level, then snakes back and forth, reversing with a 180° arc turnaround
//! (radius = half the pass spacing) at each end so the machine never stops to
//! change direction. Between depth levels it plunges straight down in place and
//! carries on, so there is a single retract at the very end.
//!
//! Z uses the magnitude convention: `start_offset` is the top cutting plane above
//! the drawing reference, `depth` is a positive amount removed downward, and the
//! final faced plane is `start_offset − depth`.

use cam_cldata::{ArcDir, MoveKind, Point3, Program, Step, Tag};
use cam_geo::Point;
use cam_model::{Axis, FaceOp};

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
        // Facing leaves a floor: a non-cutting tip cannot make one at all, a
        // non-flat tool only makes a worse one (warning — the machinist stays free).
        if !crate::guards::check_flat_floor(op.id, "face", tool, &mut diagnostics) {
            return StrategyResult { diagnostics, ..Default::default() };
        }
        if !crate::guards::check_axial_reach(op.id, "face", tool, op.depth, &mut diagnostics) {
            return StrategyResult { diagnostics, ..Default::default() };
        }
        if !op.boundary.is_valid() {
            fail!("operation {}: face boundary is not a closed area", op.id);
        }
        if op.stepdown <= 0.0 {
            fail!("operation {}: stepdown must be positive", op.id);
        }
        if !(0.0..1.0).contains(&op.overlap) {
            fail!(
                "operation {}: overlap must be a fraction in [0, 1)",
                op.id
            );
        }
        if op.overshoot < 0.0 {
            fail!("operation {}: overshoot must be >= 0", op.id);
        }
        if op.depth <= 0.0 {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: depth is not positive; nothing to face",
                op.id
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }

        let r = tool.radius();
        let p = tool.diameter * (1.0 - op.overlap); // pass spacing
        if p <= 1e-9 {
            fail!("operation {}: overlap leaves no stepover", op.id);
        }

        // Part bounds in the (travel, step) frame for the chosen pass direction.
        let (bx, by) = bounds(op.boundary.points());
        let (u_min, u_max, v_min, v_max) = match op.direction {
            Axis::X => (bx.0, bx.1, by.0, by.1), // pass along X, step in Y
            Axis::Y => (by.0, by.1, bx.0, bx.1), // pass along Y, step in X
        };

        // Pass centres along the step axis. The first sits so the tool's far edge
        // is `p` inside the stock edge (a `p`-wide first strip); step by `p` until
        // the far edge clears the opposite side, so the whole width is covered.
        let mut centres = vec![v_min - r + p];
        while centres.last().unwrap() + r < v_max {
            let next = centres.last().unwrap() + p;
            centres.push(next);
        }
        // Travel ends: overshoot past each stock edge before the turnaround arc.
        let u_lo = u_min - op.overshoot;
        let u_hi = u_max + op.overshoot;

        let top = op.start_offset;
        let bottom = op.start_offset - op.depth;
        let levels = depth_levels(top, bottom, op.stepdown);
        if levels.is_empty() {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: nothing to face at this depth",
                op.id
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }

        // Map a (travel u, step v) coordinate to world XY, and the world travel dir.
        let to_world = |u: f64, v: f64| match op.direction {
            Axis::X => Point::new(u, v),
            Axis::Y => Point::new(v, u),
        };
        let tdir = match op.direction {
            Axis::X => (1.0, 0.0),
            Axis::Y => (0.0, 1.0),
        };

        let cut = Tag::new(op.id, MoveKind::Cutting);
        let link = Tag::new(op.id, MoveKind::Link);
        let plunge = Tag::new(op.id, MoveKind::Plunge);
        let retract = Tag::new(op.id, MoveKind::Retract);

        let n = centres.len();
        let mut program = Program::new();
        // Serpentine order flips each level so the next level continues from the
        // corner where the previous one ended (plunge in place, no retract).
        let mut order: Vec<usize> = (0..n).collect();
        let mut start_forward = true;
        let mut cur = to_world(u_lo, centres[0]);

        for (li, &z) in levels.iter().enumerate() {
            if cancel.is_cancelled() {
                return StrategyResult {
                    program,
                    diagnostics,
                    cancelled: true,
                };
            }

            let v_first = centres[order[0]];
            let s0 = if start_forward {
                to_world(u_lo, v_first)
            } else {
                to_world(u_hi, v_first)
            };
            if li == 0 {
                // Approach over the first pass and down to the top cutting plane.
                program.push(Step::Rapid {
                    to: Point3::new(s0.x, s0.y, env.heights.clearance),
                    tag: link,
                });
                program.push(Step::Rapid {
                    to: Point3::new(s0.x, s0.y, top),
                    tag: link,
                });
            }
            // Plunge to this level (a vertical move in place for levels after the
            // first, since `s0` is exactly where the previous level ended).
            program.push(Step::Linear {
                to: Point3::new(s0.x, s0.y, z),
                feed: op.plunge_feed,
                tag: plunge,
            });

            let mut forward = start_forward;
            let mut last_forward = start_forward;
            for (pos, &ci) in order.iter().enumerate() {
                let v = centres[ci];
                let end = if forward {
                    to_world(u_hi, v)
                } else {
                    to_world(u_lo, v)
                };
                program.push(Step::Linear {
                    to: Point3::new(end.x, end.y, z),
                    feed: op.feed,
                    tag: cut,
                });
                if pos + 1 < n {
                    let nv = centres[order[pos + 1]];
                    let next_start = if forward {
                        to_world(u_hi, nv)
                    } else {
                        to_world(u_lo, nv)
                    };
                    let outward = if forward {
                        tdir
                    } else {
                        (-tdir.0, -tdir.1)
                    };
                    turnaround(&mut program, end, next_start, outward, z, op.feed, cut);
                } else {
                    cur = end;
                }
                last_forward = forward;
                forward = !forward;
            }

            order.reverse();
            start_forward = !last_forward;
        }

        program.push(Step::Rapid {
            to: Point3::new(cur.x, cur.y, env.heights.clearance),
            tag: retract,
        });

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

/// XY bounding box over a point list, as `((xmin, xmax), (ymin, ymax))`.
fn bounds(pts: &[Point]) -> ((f64, f64), (f64, f64)) {
    let (mut xmin, mut ymin) = (f64::MAX, f64::MAX);
    let (mut xmax, mut ymax) = (f64::MIN, f64::MIN);
    for p in pts {
        xmin = xmin.min(p.x);
        ymin = ymin.min(p.y);
        xmax = xmax.max(p.x);
        ymax = ymax.max(p.y);
    }
    ((xmin, xmax), (ymin, ymax))
}

/// Emit a 180° turnaround arc `s → e` at height `z`, bulging toward `outward` (a
/// unit direction) so the semicircle swings clear of the part. The centre is the
/// midpoint of `s`/`e` (they are one pass-spacing apart).
fn turnaround(prog: &mut Program, s: Point, e: Point, outward: (f64, f64), z: f64, feed: f64, tag: Tag) {
    let centre = Point::new(0.5 * (s.x + e.x), 0.5 * (s.y + e.y));
    let radius = 0.5 * ((e.x - s.x).hypot(e.y - s.y));
    let apex = Point::new(centre.x + outward.0 * radius, centre.y + outward.1 * radius);
    // Orientation of the path s → apex → e picks G2/G3 (CCW when the turn is left).
    let cross = (apex.x - s.x) * (e.y - s.y) - (apex.y - s.y) * (e.x - s.x);
    let dir = if cross > 0.0 { ArcDir::Ccw } else { ArcDir::Cw };
    prog.push(Step::Arc {
        end: Point3::new(e.x, e.y, z),
        center: Point3::new(centre.x, centre.y, z),
        dir,
        feed,
        tag,
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_geo::Contour;
    use cam_model::{Heights, Tool, ToolKind};

    fn rect(w: f64, h: f64) -> Contour {
        Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(w, 0.0),
            Point::new(w, h),
            Point::new(0.0, h),
        ])
    }

    fn tool(diameter: f64) -> Tool {
        Tool {
            number: 1,
            diameter,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }
    }

    fn face_op() -> FaceOp {
        FaceOp {
            spindle_rpm: 0.0,
            work_offset: 1,
            id: 0,
            tool: 1,
            boundary: rect(60.0, 40.0),
            start_offset: 1.0,
            depth: 1.0,
            stepdown: 2.0,
            overlap: 0.5,
            overshoot: 2.0,
            direction: Axis::X,
            feed: 400.0,
            plunge_feed: 150.0,
        }
    }

    fn run(op: FaceOp) -> StrategyResult {
        let tools = [tool(10.0)];
        let env = JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &tools,
            stock: None,
        };
        FaceStrategy::new(op).compute(&env, &CancelToken::new())
    }

    fn count<F: Fn(&Step) -> bool>(r: &StrategyResult, f: F) -> usize {
        r.program.steps().iter().filter(|s| f(s)).count()
    }

    #[test]
    fn continuous_serpentine_plunges_once_per_level() {
        // ⌀10, 50% overlap ⇒ p=5, step span [0,40] ⇒ centres 0,5,…,35 = 8 passes.
        // start_offset 1, depth 1, stepdown 2 ⇒ one level (final Z 0).
        let r = run(face_op());
        assert!(!r.has_errors(), "{:?}", r.diagnostics);

        let plunges = count(&r, |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge));
        assert_eq!(plunges, 1, "one plunge for the single level, not one per pass");

        let passes = count(&r, |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Cutting));
        assert_eq!(passes, 8, "eight parallel passes");

        let arcs = count(&r, |s| matches!(s, Step::Arc { tag, .. } if tag.kind == MoveKind::Cutting));
        assert_eq!(arcs, 7, "seven turnaround arcs linking the passes");

        // The whole cut sits on the faced plane Z=0.
        for s in r.program.steps() {
            if let Step::Linear { to, tag, .. } = s {
                if tag.kind == MoveKind::Cutting {
                    assert!((to.z - 0.0).abs() < 1e-9, "cut at the faced plane");
                }
            }
        }
    }

    fn first_cut_y(r: &StrategyResult) -> f64 {
        r.program
            .steps()
            .iter()
            .find_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(to.y),
                _ => None,
            })
            .expect("a cutting pass")
    }

    #[test]
    fn overlap_places_the_first_pass_a_strip_in_from_the_edge() {
        // First pass centre sits at v_min - r + p = 0 - 5 + 5 = 0 (Y for X-passes),
        // so the first cutting pass runs along y = 0.
        assert!(
            (first_cut_y(&run(face_op())) - 0.0).abs() < 1e-9,
            "first pass cuts the p-wide edge strip"
        );

        // 80% overlap on ⌀10 ⇒ p=2, first centre at 0-5+2 = -3.
        let mut op = face_op();
        op.overlap = 0.8;
        assert!(
            (first_cut_y(&run(op)) + 3.0).abs() < 1e-9,
            "high overlap starts the tool off the edge"
        );
    }

    #[test]
    fn overshoot_extends_the_pass_past_the_edge() {
        // X passes over [0,60] with 2 mm overshoot run from x=-2 to x=62.
        let r = run(face_op());
        let first_pass_end_x = r.program.steps().iter().find_map(|s| match s {
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(to.x),
            _ => None,
        });
        assert_eq!(first_pass_end_x, Some(62.0), "pass overshoots the far edge by 2 mm");
    }

    #[test]
    fn y_direction_passes_run_along_y() {
        let mut op = face_op();
        op.direction = Axis::Y;
        let r = run(op);
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        // First Y-pass steps in X: centre x = 0, runs y from -2 to 42.
        let first = r.program.steps().iter().find_map(|s| match s {
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some((to.x, to.y)),
            _ => None,
        });
        assert_eq!(first, Some((0.0, 42.0)), "pass runs along Y at x=0");
    }

    #[test]
    fn multiple_levels_plunge_once_each_without_retracting() {
        // depth 3 at stepdown 1.5 ⇒ two levels; one plunge each, one final retract.
        let mut op = face_op();
        op.start_offset = 0.0;
        op.depth = 3.0;
        op.stepdown = 1.5;
        let r = run(op);
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        let plunges = count(&r, |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge));
        assert_eq!(plunges, 2, "one plunge per level");
        let retracts = count(&r, |s| matches!(s, Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract));
        assert_eq!(retracts, 1, "a single retract at the very end");
    }

    #[test]
    fn path_is_continuous_and_arcs_are_well_formed() {
        // Every cutting/plunge move begins where the previous ended (no hidden
        // repositioning), and each turnaround is a true semicircle: centre
        // equidistant from both ends, diameter equal to the pass spacing (p=5).
        let r = run(face_op());
        let mut pos: Option<Point3> = None;
        for s in r.program.steps() {
            match s {
                Step::Rapid { to, .. } => pos = Some(*to),
                Step::Linear { to, tag, .. } => {
                    if tag.kind == MoveKind::Cutting {
                        let from = pos.expect("a prior position");
                        assert!(
                            (from.z - to.z).abs() < 1e-9,
                            "cutting moves stay at depth (no lift)"
                        );
                    }
                    pos = Some(*to);
                }
                Step::Arc { end, center, .. } => {
                    let from = pos.expect("an arc needs a prior position");
                    let rs = (from.x - center.x).hypot(from.y - center.y);
                    let re = (end.x - center.x).hypot(end.y - center.y);
                    assert!((rs - re).abs() < 1e-6, "arc centre equidistant ({rs} vs {re})");
                    assert!((rs - 2.5).abs() < 1e-6, "turnaround radius is p/2 = 2.5");
                    assert!(
                        (from.z - end.z).abs() < 1e-9,
                        "turnaround stays at depth"
                    );
                    pos = Some(*end);
                }
                _ => {}
            }
        }
    }

    #[test]
    fn bad_overlap_errors() {
        let mut op = face_op();
        op.overlap = 1.0;
        assert!(run(op).has_errors(), "overlap must be < 1");
    }
}
