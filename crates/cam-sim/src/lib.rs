//! # cam-sim — material-removal simulation & verification
//!
//! Runs a CL-data [`Program`] against a [`Heightfield`] stock model, removing
//! material as the tool cuts and **flagging collisions** — most importantly a
//! rapid that would plow through remaining stock. This is the verification stage:
//! a plausible backplot is not proof; a simulation that clears what it should and
//! never crashes is much closer.
//!
//! The core is headless and unit-tested. Rendering the resulting heightfield in
//! the viewport is a separate concern (as with `cam-render`).

mod heightfield;

pub use heightfield::Heightfield;

use cam_cldata::{ArcDir, Point3, Program, Step};
use cam_geo::{Arc, Point};

/// How finely to simulate, and the tool it runs.
#[derive(Clone, Copy, Debug)]
pub struct SimOptions {
    /// Heightfield cell size, mm.
    pub resolution: f64,
    /// Tool radius, mm (the swept footprint).
    pub tool_radius: f64,
}

impl Default for SimOptions {
    fn default() -> Self {
        Self {
            resolution: 0.5,
            tool_radius: 3.0,
        }
    }
}

/// The kind of a detected problem.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionKind {
    /// A rapid (`G0`) traverse passes through remaining stock.
    RapidThroughStock,
}

/// A detected problem, with where it happened.
#[derive(Clone, Debug, PartialEq)]
pub struct Collision {
    pub kind: CollisionKind,
    pub at: [f64; 3],
    pub message: String,
}

/// The outcome of a simulation: the final stock, any collisions, and how much
/// material was removed.
#[derive(Clone, Debug)]
pub struct SimResult {
    pub field: Heightfield,
    pub collisions: Vec<Collision>,
    pub removed_volume: f64,
}

impl SimResult {
    /// Whether the run found no collisions.
    pub fn is_clean(&self) -> bool {
        self.collisions.is_empty()
    }
}

/// Height above the clearance plane, in mm, that counts as a collision (tolerates
/// grazing the surface).
const CLEARANCE_EPS: f32 = 0.001;

/// Simulate `program` removing material from a block of stock spanning `[min,
/// max]`, using a tool of `options.tool_radius`.
pub fn simulate(
    program: &Program,
    min: [f64; 3],
    max: [f64; 3],
    options: &SimOptions,
) -> SimResult {
    let mut field = Heightfield::new(
        [min[0], min[1]],
        [max[0], max[1]],
        options.resolution,
        max[2],
    );
    let r = options.tool_radius;
    let mut cur: Option<Point3> = None;
    let mut collisions = Vec::new();

    for step in program.steps() {
        match step {
            Step::Rapid { to, .. } => {
                if let Some(a) = cur {
                    let clearance = a.z.min(to.z) as f32;
                    let stock = field.max_height_along([a.x, a.y], [to.x, to.y], r);
                    if stock > clearance + CLEARANCE_EPS {
                        collisions.push(Collision {
                            kind: CollisionKind::RapidThroughStock,
                            at: [to.x, to.y, to.z],
                            message: format!(
                                "rapid at Z {:.3} passes through stock standing at Z {:.3}",
                                to.z, stock
                            ),
                        });
                    }
                }
                cur = Some(*to);
            }
            Step::Linear { to, .. } => {
                if let Some(a) = cur {
                    field.cut_segment([a.x, a.y, a.z], [to.x, to.y, to.z], r);
                }
                cur = Some(*to);
            }
            Step::Arc {
                end, center, dir, ..
            } => {
                if let Some(a) = cur {
                    let pts = arc_points(a, *end, *center, *dir);
                    for w in pts.windows(2) {
                        field.cut_segment(w[0], w[1], r);
                    }
                }
                cur = Some(*end);
            }
            Step::Drill(cycle) => {
                for &[x, y] in &cycle.points {
                    field.cut_segment([x, y, cycle.depth], [x, y, cycle.depth], r);
                }
                cur = None;
            }
            _ => {}
        }
    }

    let removed_volume = field.removed_volume();
    SimResult {
        field,
        collisions,
        removed_volume,
    }
}

/// Flatten an arc move into points (XY on the circle, Z interpolated end to end).
fn arc_points(a: Point3, end: Point3, center: Point3, dir: ArcDir) -> Vec<[f64; 3]> {
    let r = ((a.x - center.x).powi(2) + (a.y - center.y).powi(2)).sqrt();
    let a0 = (a.y - center.y).atan2(a.x - center.x);
    let a1 = (end.y - center.y).atan2(end.x - center.x);
    let ccw = matches!(dir, ArcDir::Ccw);
    let xy = Arc::new(Point::new(center.x, center.y), r, a0, a1, ccw).flatten(0.1);
    let n = xy.len().max(2);
    xy.iter()
        .enumerate()
        .map(|(i, p)| {
            let t = i as f64 / (n - 1) as f64;
            [p.x, p.y, a.z + (end.z - a.z) * t]
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_cldata::{MoveKind, ProgramBuilder, Tag};

    const STOCK_MIN: [f64; 3] = [0.0, 0.0, -10.0];
    const STOCK_MAX: [f64; 3] = [40.0, 40.0, 0.0];

    fn opts() -> SimOptions {
        SimOptions {
            resolution: 0.5,
            tool_radius: 2.0,
        }
    }

    #[test]
    fn a_safe_program_removes_material_without_collisions() {
        // Rapid in high, plunge, cut across, retract — all safe.
        let prog = ProgramBuilder::new()
            .op(0)
            .feed(300.0)
            .rapid(Point3::new(5.0, 20.0, 5.0), MoveKind::Link)
            .linear(Point3::new(5.0, 20.0, -2.0), MoveKind::Plunge)
            .linear(Point3::new(35.0, 20.0, -2.0), MoveKind::Cutting)
            .rapid(Point3::new(35.0, 20.0, 5.0), MoveKind::Retract)
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts());
        assert!(sim.is_clean(), "collisions: {:?}", sim.collisions);
        assert!(sim.removed_volume > 0.0, "some material removed");
    }

    #[test]
    fn a_rapid_through_uncut_stock_is_flagged() {
        // A lateral rapid at Z -2 across untouched stock (top Z 0) is a crash.
        let prog = ProgramBuilder::new()
            .op(0)
            .rapid(Point3::new(2.0, 20.0, -2.0), MoveKind::Link)
            .rapid(Point3::new(38.0, 20.0, -2.0), MoveKind::Link)
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts());
        assert!(!sim.is_clean(), "should flag the rapid");
        assert_eq!(sim.collisions[0].kind, CollisionKind::RapidThroughStock);
    }

    #[test]
    fn drilling_removes_stock_at_each_hole() {
        let prog = ProgramBuilder::new()
            .drill(cam_cldata::DrillCycle {
                points: vec![[10.0, 10.0], [30.0, 30.0]],
                z_top: 0.0,
                depth: -5.0,
                retract: 2.0,
                peck: None,
                dwell: None,
                feed: 100.0,
                tag: Tag::new(0, MoveKind::Plunge),
            })
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts());
        assert!(sim.field.sample(10.0, 10.0) < -4.9, "hole 1 drilled");
        assert!(sim.field.sample(30.0, 30.0) < -4.9, "hole 2 drilled");
        assert!(
            sim.field.sample(20.0, 20.0) > -0.1,
            "between holes untouched"
        );
    }
}
