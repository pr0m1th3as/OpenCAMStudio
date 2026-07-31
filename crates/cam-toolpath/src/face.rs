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

        // Face the whole **stock** top (fall back to the picked boundary only when
        // there is no stock). Bounds in the (travel, step) frame for the pass direction.
        let (bx, by) = match env.stock {
            Some((smin, smax)) => ((smin[0], smax[0]), (smin[1], smax[1])),
            None => bounds(op.boundary.points()),
        };
        let (u_min, u_max, v_min, v_max) = match op.direction {
            Axis::X => (bx.0, bx.1, by.0, by.1), // pass along X, step in Y
            Axis::Y => (by.0, by.1, bx.0, bx.1), // pass along Y, step in X
        };

        // Pass centres along the step axis. The first sits so the tool's far edge
        // is `p` inside the near edge (a `p`-wide first strip); step by `p` until
        // the far edge clears the opposite side, so the whole width is covered.
        let mut centres = vec![v_min - r + p];
        while centres.last().unwrap() + r < v_max {
            let next = centres.last().unwrap() + p;
            centres.push(next);
        }
        // Travel ends: `overshoot` is the clearance between the cutter **edge** and the
        // material edge at the plunge and turnarounds, so the tool always plunges clear
        // and feeds in — the centre therefore sits a radius plus the overshoot outside
        // each end. `overshoot = 0` is tangent (just clears) — note that is a coincidence
        // of geometry, not a margin, which is why the entry below keeps a vertical one
        // regardless. A **negative** overshoot plunges into the stock. That's the user's
        // prerogative (a soft or pre-cleared top), so we warn rather than forbid.
        let u_lo = u_min - r - op.overshoot;
        let u_hi = u_max + r + op.overshoot;
        if env.stock.is_some() && op.overshoot < 0.0 {
            // Say where the tool *is* and by how much, not just that a number is
            // negative — the operator's next question is always "by how much", and the
            // answer is not on screen anywhere else.
            //
            // It is the **plunge** that enters material, not the descent: the approach
            // rapid now stops `FLOOR_CLEARANCE` above the cutting plane (see the entry
            // below), so with the usual `start_offset` = stock top it comes to rest in
            // air and the feed below it does the cutting. Worth being exact about, since
            // a warning that named the wrong move would send someone looking at the
            // wrong line of the program.
            //
            // Deliberately still a warning: a soft or pre-cleared top is the user's
            // prerogative, and `overshoot` is never clamped.
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: overshoot {:.3} mm is negative — the cutter edge starts \
                 that far inside the stock edge, so the tool enters over material and \
                 feeds down into it instead of entering clear of the edge",
                op.id,
                -op.overshoot
            )));
        }

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
                // Approach over the first pass at clearance, then rapid down — but never
                // onto the top cutting plane itself, which for facing is normally the
                // stock surface exactly (`start_offset` is set to where the material
                // starts).
                //
                // `retract.max(top)` was not enough. Whenever `start_offset >= retract`
                // the `max` picks `top` and the rapid comes to rest precisely on the
                // skin — stock standing 3 mm proud with a 2 mm retract is enough to do
                // it. What kept that safe was purely *horizontal*: the descent sits at
                // `u_lo`, a radius plus `overshoot` outside the stock edge. But the two
                // margins are independent, so at `overshoot = 0` — documented as
                // "tangent (just clears)" — both are zero at once, and the tool arrives
                // at the stock's top corner at rapid speed with clearance in neither
                // axis. Tangency is a coincidence, not a margin.
                //
                // So the vertical margin is now unconditional: stop `FLOOR_CLEARANCE`
                // above the cutting plane and let the plunge below feed the rest. Same
                // number and the same argument as `emit::rapid_floor`, which the five
                // stepdown descents use. Note this could *not* be delegated to
                // `emit::descend_to`: that helper clamps `.max(from_z)` because it ends
                // at a floor it is about to cut, so it would return this very Z.
                //
                // Costs one short feed per facing operation (not per pass), and only
                // when `top + FLOOR_CLEARANCE` clears the retract plane at all — in the
                // ordinary `start_offset = 0` setup the retract plane still wins and
                // nothing changes.
                program.push(Step::Rapid {
                    to: Point3::new(s0.x, s0.y, env.heights.clearance),
                    tag: link,
                });
                program.push(Step::Rapid {
                    to: Point3::new(
                        s0.x,
                        s0.y,
                        env.heights.retract.max(top + crate::emit::FLOOR_CLEARANCE),
                    ),
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
    fn overshoot_is_edge_clearance_past_the_edge() {
        // `overshoot` is the cutter-*edge* clearance past the material edge, so the far
        // pass end (tool centre) sits at edge + radius + overshoot. No stock ⇒ the
        // boundary is the fallback: [0,60] X, r=5, overshoot=2 ⇒ far end centre 67, and
        // the trailing edge (67-5=62) clears the boundary by the 2 mm overshoot.
        let r = run(face_op());
        let first_pass_end_x = r.program.steps().iter().find_map(|s| match s {
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(to.x),
            _ => None,
        });
        assert_eq!(first_pass_end_x, Some(67.0), "far end = edge + radius + overshoot");
    }

    fn run_stock(op: FaceOp, stock: ([f64; 2], [f64; 2])) -> StrategyResult {
        let tools = [tool(10.0)];
        let env = JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &tools,
            stock: Some(stock),
        };
        FaceStrategy::new(op).compute(&env, &CancelToken::new())
    }

    fn first_plunge_x(r: &StrategyResult) -> f64 {
        r.program
            .steps()
            .iter()
            .find_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => Some(to.x),
                _ => None,
            })
            .expect("a plunge move")
    }

    /// The Z the approach rapid comes to rest at — the first rapid that leaves the
    /// clearance plane. (`run_stock` uses clearance 5, retract 2.)
    fn approach_rapid_z(r: &StrategyResult) -> f64 {
        r.program
            .steps()
            .iter()
            .find_map(|s| match s {
                Step::Rapid { to, .. } if to.z < 5.0 - 1e-9 => Some(to.z),
                _ => None,
            })
            .expect("a descending approach rapid")
    }

    #[test]
    fn the_approach_rapid_never_lands_on_the_top_cutting_plane() {
        // The hazard this guards: whenever `start_offset >= retract` the old
        // `retract.max(top)` picked `top`, and for facing the top cutting plane *is* the
        // stock surface — so the rapid came to rest exactly on the skin. What kept that
        // safe was only that the descent sits outside the stock in XY, which at
        // `overshoot = 0` is itself a zero margin. Two zero margins at once is not a
        // margin.
        //
        // Retract 2, start_offset 3: the old rule gave 3.000, the plane itself.
        let mut op = face_op();
        op.start_offset = 3.0;
        op.depth = 1.0;
        op.overshoot = 0.0; // the horizontal margin gone too — the worst case
        let r = run_stock(op, ([-5.0, -5.0], [65.0, 45.0]));
        let z = approach_rapid_z(&r);
        assert!(
            z >= 3.0 + crate::emit::FLOOR_CLEARANCE - 1e-9,
            "the approach rapid stopped at Z{z:.3}, on or under the 3.000 cutting plane"
        );
    }

    #[test]
    fn the_ordinary_facing_setup_still_stops_at_the_retract_plane() {
        // The other half: the new floor must not lift every facing job. With
        // `start_offset = 0` and a 2 mm retract the retract plane is still the higher of
        // the two, so nothing moves — which is why no golden changed.
        let mut op = face_op();
        op.start_offset = 0.0;
        let r = run_stock(op, ([-5.0, -5.0], [65.0, 45.0]));
        assert!(
            (approach_rapid_z(&r) - 2.0).abs() < 1e-9,
            "expected the 2 mm retract plane, got {}",
            approach_rapid_z(&r)
        );
    }

    #[test]
    fn overshoot_is_measured_from_the_cutter_edge_to_the_stock_edge() {
        // Stock spans x=[-5,65] (proud of the 0..60 boundary). The r=5 mill plunges with
        // its edge `overshoot` clear of the stock edge (-5): centre = -5 - r - overshoot.
        let os = 3.0;
        let mut op = face_op();
        op.overshoot = os;
        let x = first_plunge_x(&run_stock(op, ([-5.0, -5.0], [65.0, 45.0])));
        assert!((x - (-5.0 - 5.0 - os)).abs() < 1e-6, "plunge centre -5-r-overshoot: {x}");
        // Edge (= centre + r) sits `overshoot` clear of the stock edge.
        assert!((x + 5.0 - (-5.0 - os)).abs() < 1e-6, "edge clears stock by overshoot");
    }

    #[test]
    fn zero_overshoot_leaves_the_cutter_edge_tangent_to_the_stock_and_warns_nothing() {
        let mut op = face_op();
        op.overshoot = 0.0;
        let r = run_stock(op, ([-5.0, -5.0], [65.0, 45.0]));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        assert!(r.diagnostics.is_empty(), "tangent is not a warning: {:?}", r.diagnostics);
        // Edge at the stock edge: centre = -5 - r.
        assert!((first_plunge_x(&r) - (-10.0)).abs() < 1e-6);
    }

    #[test]
    fn negative_overshoot_plunges_into_the_stock_and_warns() {
        let mut op = face_op();
        op.overshoot = -2.0;
        let r = run_stock(op, ([-5.0, -5.0], [65.0, 45.0]));
        assert!(!r.has_errors(), "a warning, not an error: {:?}", r.diagnostics);
        // Checked for substance rather than a phrase: the operator needs to be told
        // *how far* inside the edge the cutter starts, and that the whole entry is over
        // material — not merely that a number was negative.
        let warning = r
            .diagnostics
            .iter()
            .find(|d| d.message.contains("overshoot"))
            .unwrap_or_else(|| panic!("an overshoot warning: {:?}", r.diagnostics));
        assert!(
            warning.message.contains("2.000"),
            "names the distance inside the edge: {}",
            warning.message
        );
        assert!(
            warning.message.contains("inside the stock edge"),
            "says where the cutter starts: {}",
            warning.message
        );
        // Edge 2 mm *inside* the stock edge (-5): centre = -5 - r - (-2) = -8.
        assert!((first_plunge_x(&r) - (-8.0)).abs() < 1e-6);
    }

    #[test]
    fn y_direction_passes_run_along_y() {
        let mut op = face_op();
        op.direction = Axis::Y;
        let r = run(op);
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        // First Y-pass steps in X: centre x = 0, far end y = edge + radius + overshoot
        // = 40 + 5 + 2 = 47 (boundary fallback, no stock).
        let first = r.program.steps().iter().find_map(|s| match s {
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some((to.x, to.y)),
            _ => None,
        });
        assert_eq!(first, Some((0.0, 47.0)), "pass runs along Y at x=0");
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
