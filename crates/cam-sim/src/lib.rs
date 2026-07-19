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

pub use heightfield::{Heightfield, SurfaceDiff, SurfaceMesh};

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

/// The bottom shape of a tool within its cutting radius — what makes a ball mill
/// leave a rounded floor where an end mill leaves a flat one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ProfileShape {
    /// Flat bottom (end mill / face mill).
    Flat,
    /// Full ball nose — corner radius equals the tool radius.
    Ball,
    /// Flat centre with a rounded corner of `corner_radius` mm (bull-nose).
    BullNose { corner_radius: f64 },
    /// Conical point (chamfer/V mill, drill): the surface rises as `d/tan(α)`
    /// outside an optional flat tip, where `half_angle_rad` (α) is measured from
    /// the tool axis.
    Cone {
        /// Half of the included point angle, measured from the axis (radians).
        half_angle_rad: f64,
        /// Radius of the flat tip, mm (0 for a sharp point / drill).
        flat_radius: f64,
    },
    /// Carving **V-bit**: a cone of half-angle `α` rising from a *rounded* tip — a
    /// ball of `tip_radius` centred on the axis at height `tip_radius`, tangent to
    /// the cone. Distinct from [`ProfileShape::Cone`], whose tip is a **flat**: the
    /// ball leaves a rounded groove root and, at the shallow depths engraving works
    /// at, a *wider* groove than the cone alone would give.
    VTip {
        /// Half of the included V angle, measured from the axis (radians).
        half_angle_rad: f64,
        /// Radius of the rounded tip, mm (0 for a sharp point).
        tip_radius: f64,
    },
}

/// The cutting profile of a tool: its radius and bottom shape. [`offset`] gives
/// how far the cutting surface sits *above* the tool's lowest point at radial
/// distance `d` from the axis, so the heightfield lowers a covered cell to
/// `axis_bottom + offset(d)`.
///
/// [`offset`]: ToolProfile::offset
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToolProfile {
    /// Cutting radius, mm — the swept footprint.
    pub radius: f64,
    /// The bottom shape within that radius.
    pub shape: ProfileShape,
}

impl ToolProfile {
    /// A flat-bottomed tool of the given radius.
    pub fn flat(radius: f64) -> Self {
        Self {
            radius,
            shape: ProfileShape::Flat,
        }
    }

    /// Height of the cutting surface above the tool's lowest point at radial
    /// distance `d` from the axis (clamped to `[0, radius]`).
    pub fn offset(&self, d: f64) -> f64 {
        let d = d.clamp(0.0, self.radius);
        match self.shape {
            ProfileShape::Flat => 0.0,
            ProfileShape::Ball => {
                let r = self.radius;
                r - (r * r - d * d).max(0.0).sqrt()
            }
            ProfileShape::BullNose { corner_radius } => {
                let flat = (self.radius - corner_radius).max(0.0);
                if d <= flat {
                    0.0
                } else {
                    let x = d - flat; // distance into the corner arc
                    corner_radius - (corner_radius * corner_radius - x * x).max(0.0).sqrt()
                }
            }
            ProfileShape::Cone {
                half_angle_rad,
                flat_radius,
            } => {
                if d <= flat_radius {
                    0.0
                } else {
                    let t = half_angle_rad.tan();
                    if t <= 1e-9 {
                        0.0
                    } else {
                        (d - flat_radius) / t
                    }
                }
            }
            ProfileShape::VTip {
                half_angle_rad,
                tip_radius,
            } => {
                let a = half_angle_rad;
                let rt = tip_radius.clamp(0.0, self.radius);
                let t = a.tan();
                if t <= 1e-9 {
                    return 0.0;
                }
                if rt <= 1e-9 {
                    return d / t;
                }
                // Tangent point between the ball and the cone flank.
                let r_t = rt * a.cos();
                if d <= r_t {
                    // On the ball: a circle of radius rt centred at height rt.
                    rt - (rt * rt - d * d).max(0.0).sqrt()
                } else {
                    rt * (1.0 - a.sin()) + (d - r_t) / t
                }
            }
        }
    }
}

/// A tool in the sim's tool table, keyed by the `Tn` number the program selects
/// with a [`Step::ToolChange`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimTool {
    /// Tool number (matches `ToolChange { tool }`).
    pub number: u32,
    /// The tool's cutting profile.
    pub profile: ToolProfile,
}

/// The kind of a detected problem.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionKind {
    /// A rapid (`G0`) traverse passes through remaining stock.
    RapidThroughStock,
    /// The tool cut below the desired target surface — an over-cut (gouge) that
    /// destroys part material.
    Gouge,
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
/// max]`. The tool in the spindle follows the program's `ToolChange` steps,
/// looked up in `tools`; before the first change (or for any unlisted number) a
/// flat tool of `options.tool_radius` is used.
pub fn simulate(
    program: &Program,
    min: [f64; 3],
    max: [f64; 3],
    options: &SimOptions,
    tools: &[SimTool],
) -> SimResult {
    let mut field = Heightfield::new(
        [min[0], min[1]],
        [max[0], max[1]],
        options.resolution,
        max[2],
    );
    let default_tool = ToolProfile::flat(options.tool_radius);
    let mut tool = default_tool;
    let mut cur: Option<Point3> = None;
    let mut collisions = Vec::new();

    for step in program.steps() {
        match step {
            Step::ToolChange { tool: number } => {
                tool = tools
                    .iter()
                    .find(|t| t.number == *number)
                    .map_or(default_tool, |t| t.profile);
            }
            Step::Rapid { to, .. } => {
                if let Some(a) = cur {
                    // A vertical lift (no XY travel, moving up) is always safe: the tool
                    // retreats straight up through the column it just cut, so it cannot
                    // plow through stock. The tool-radius disc below would otherwise catch
                    // a neighbouring ridge — e.g. the scallops a V/chamfer tool leaves
                    // between face passes — and false-flag the retract. (A *downward*
                    // in-place move is still checked: strategies plunge with a feed, not a
                    // rapid, so a descending rapid into stock is a real fault.) A lateral
                    // rapid sweeps the tool horizontally and is always checked.
                    let lateral = (a.x - to.x).hypot(a.y - to.y) > 1e-9;
                    let vertical_lift = !lateral && to.z >= a.z;
                    if !vertical_lift {
                        // Profile-aware: the tool is only as wide as its own shape at
                        // the depth it sits, so a pointed tool descending into the
                        // narrow groove it just cut is not stopped by the uncut
                        // surface alongside it. For a flat tool `offset` is zero
                        // everywhere, so this reduces exactly to the old radius test.
                        let tip_z = a.z.min(to.z);
                        if let Some((intrusion, stock)) =
                            field.max_intrusion_along([a.x, a.y], [to.x, to.y], tip_z, &tool)
                        {
                            if intrusion > CLEARANCE_EPS {
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
                    }
                }
                cur = Some(*to);
            }
            Step::Linear { to, .. } => {
                if let Some(a) = cur {
                    field.cut_segment_profile([a.x, a.y, a.z], [to.x, to.y, to.z], &tool);
                }
                cur = Some(*to);
            }
            Step::Arc {
                end, center, dir, ..
            } => {
                if let Some(a) = cur {
                    let pts = arc_points(a, *end, *center, *dir);
                    for w in pts.windows(2) {
                        field.cut_segment_profile(w[0], w[1], &tool);
                    }
                }
                cur = Some(*end);
            }
            Step::Drill(cycle) => {
                for &[x, y] in &cycle.points {
                    field.cut_segment_profile([x, y, cycle.depth], [x, y, cycle.depth], &tool);
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

/// Verify a simulated `field` against a desired `target` surface: if the tool cut
/// more than `tol` mm below the target anywhere, return a [`CollisionKind::Gouge`]
/// summarising the worst point and total over-cut. Returns `None` when the run is
/// within tolerance of (or leaves stock above) the target.
///
/// This is the verification a backplot cannot give: a program can be perfectly
/// collision-free against the stock yet still cut into the finished part.
pub fn check_gouge(field: &Heightfield, target: &Heightfield, tol: f64) -> Option<Collision> {
    let diff = field.compare(target, tol);
    if (diff.max_gouge as f64) <= tol {
        return None;
    }
    let at = diff.gouge_at.unwrap_or([0.0, 0.0]);
    Some(Collision {
        kind: CollisionKind::Gouge,
        at: [at[0], at[1], diff.gouge_z],
        message: format!(
            "gouge: cut {:.3} mm below target at ({:.2}, {:.2}) — {} cells, {:.1} mm³ over-cut",
            diff.max_gouge, at[0], at[1], diff.cells_gouged, diff.gouge_volume
        ),
    })
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
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts(), &[]);
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
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts(), &[]);
        assert!(!sim.is_clean(), "should flag the rapid");
        assert_eq!(sim.collisions[0].kind, CollisionKind::RapidThroughStock);
    }

    /// Options sized to a *pointed* tool's actual cut, as `cam-app`'s
    /// `narrowest_cut_width` does: a 60° V-bit engraving ~0.4 mm deep cuts a ~0.58 mm
    /// groove, so cells must be a fraction of that. The module default (0.5 mm) is
    /// coarser than the whole groove and cannot resolve it in either direction.
    fn fine_opts() -> SimOptions {
        SimOptions {
            resolution: 0.19,
            tool_radius: 3.0,
        }
    }

    fn vbit(number: u32, radius: f64, half_angle_rad: f64) -> SimTool {
        SimTool {
            number,
            profile: ToolProfile {
                radius,
                shape: ProfileShape::VTip {
                    half_angle_rad,
                    tip_radius: 0.1,
                },
            },
        }
    }

    #[test]
    fn a_pointed_tool_may_descend_into_the_groove_it_just_cut() {
        // Multi-pass engraving: pass 2 rapids down to pass 1's depth before feeding
        // deeper. The tool is only as wide as its cone has opened at that depth — a
        // ⌀6 V-bit 0.15 mm down is a 0.3 mm sliver — so the uncut surface 3 mm away
        // must not stop it. Regression for a RapidThroughStock that blocked export of
        // every multi-pass engrave.
        let tool = vbit(1, 3.0, (30.0_f64).to_radians());
        let prog = ProgramBuilder::new()
            .op(0)
            .feed(300.0)
            .tool_change(1)
            .rapid(Point3::new(5.0, 20.0, 5.0), MoveKind::Link)
            .linear(Point3::new(5.0, 20.0, -0.15), MoveKind::Plunge)
            .linear(Point3::new(35.0, 20.0, -0.15), MoveKind::Cutting)
            .rapid(Point3::new(35.0, 20.0, 5.0), MoveKind::Retract)
            // Pass 2: back to the start, down into the groove just cut.
            .rapid(Point3::new(5.0, 20.0, 5.0), MoveKind::Link)
            .rapid(Point3::new(5.0, 20.0, -0.15), MoveKind::Link)
            .linear(Point3::new(5.0, 20.0, -0.3), MoveKind::Plunge)
            .linear(Point3::new(35.0, 20.0, -0.3), MoveKind::Cutting)
            .rapid(Point3::new(35.0, 20.0, 5.0), MoveKind::Retract)
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &fine_opts(), &[tool]);
        assert!(sim.is_clean(), "collisions: {:?}", sim.collisions);
    }

    #[test]
    fn a_pointed_tool_diving_into_uncut_stock_is_still_flagged() {
        // The other side of the same coin — the check must not have been disabled for
        // pointed tools. Same tool, same depth, but descending where nothing was cut.
        let tool = vbit(1, 3.0, (30.0_f64).to_radians());
        let prog = ProgramBuilder::new()
            .op(0)
            .tool_change(1)
            .rapid(Point3::new(5.0, 20.0, 5.0), MoveKind::Link)
            .rapid(Point3::new(20.0, 20.0, -0.3), MoveKind::Link)
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &fine_opts(), &[tool]);
        assert!(!sim.is_clean(), "a dive into virgin stock is a crash");
        assert_eq!(sim.collisions[0].kind, CollisionKind::RapidThroughStock);
    }

    #[test]
    fn a_pointed_tool_dragged_sideways_below_the_surface_is_still_flagged() {
        // A lateral rapid at depth across uncut stock: the classic crash, and the
        // case the profile-aware test must never soften.
        let tool = vbit(1, 3.0, (30.0_f64).to_radians());
        let prog = ProgramBuilder::new()
            .op(0)
            .tool_change(1)
            .rapid(Point3::new(2.0, 20.0, -1.0), MoveKind::Link)
            .rapid(Point3::new(38.0, 20.0, -1.0), MoveKind::Link)
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &fine_opts(), &[tool]);
        assert!(!sim.is_clean(), "should flag the lateral rapid");
        assert_eq!(sim.collisions[0].kind, CollisionKind::RapidThroughStock);
    }

    #[test]
    fn a_flat_tool_is_unaffected_by_the_profile_aware_test() {
        // A flat tool's `offset` is zero at every radius, so the new test reduces
        // exactly to the old one: an honest crash still reports.
        let prog = ProgramBuilder::new()
            .op(0)
            .rapid(Point3::new(2.0, 20.0, -2.0), MoveKind::Link)
            .rapid(Point3::new(38.0, 20.0, -2.0), MoveKind::Link)
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts(), &[]);
        assert_eq!(sim.collisions.len(), 1);
        assert_eq!(sim.collisions[0].kind, CollisionKind::RapidThroughStock);
    }

    #[test]
    fn a_vertical_lift_beside_a_proud_ridge_is_not_flagged() {
        // A V/chamfer tool plunges a cone (narrow at depth, wide at the max radius),
        // leaving uncut stock proud of the cut within the tool's footprint — the scallop
        // a chamfer tool leaves when (mis)used to face. Lifting straight up out of that
        // cone must NOT be flagged: the tool retreats through the column it just cut and
        // cannot plow through the ridge beside it. Regression for a false RapidThroughStock
        // that blocked export of a face op run with a chamfer tool.
        let vee = SimTool {
            number: 1,
            profile: ToolProfile {
                radius: 2.0,
                shape: ProfileShape::Cone {
                    half_angle_rad: std::f64::consts::FRAC_PI_4,
                    flat_radius: 0.0,
                },
            },
        };
        let prog = ProgramBuilder::new()
            .op(0)
            .tool_change(1)
            .feed(300.0)
            .rapid(Point3::new(20.0, 20.0, 5.0), MoveKind::Link)
            .linear(Point3::new(20.0, 20.0, -1.0), MoveKind::Plunge)
            .rapid(Point3::new(20.0, 20.0, 5.0), MoveKind::Retract)
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts(), &[vee]);
        assert!(
            sim.is_clean(),
            "a vertical lift out of a cone must not flag the ridge beside it: {:?}",
            sim.collisions
        );
    }

    #[test]
    fn a_lateral_rapid_low_over_a_ridge_is_still_flagged() {
        // The fix must not blunt the real check: with the same cone tool, a *lateral*
        // rapid at the plunge depth, out across proud stock, is a genuine crash.
        let vee = SimTool {
            number: 1,
            profile: ToolProfile {
                radius: 2.0,
                shape: ProfileShape::Cone {
                    half_angle_rad: std::f64::consts::FRAC_PI_4,
                    flat_radius: 0.0,
                },
            },
        };
        let prog = ProgramBuilder::new()
            .op(0)
            .tool_change(1)
            .feed(300.0)
            .rapid(Point3::new(20.0, 20.0, 5.0), MoveKind::Link)
            .linear(Point3::new(20.0, 20.0, -1.0), MoveKind::Plunge)
            // Straight sideways at depth across untouched stock — a real plow.
            .rapid(Point3::new(35.0, 20.0, -1.0), MoveKind::Link)
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts(), &[vee]);
        assert!(!sim.is_clean(), "a lateral rapid at depth over stock must flag");
        assert_eq!(sim.collisions[0].kind, CollisionKind::RapidThroughStock);
    }

    #[test]
    fn a_cut_within_tolerance_of_target_is_not_a_gouge() {
        // Simulate a flat -3 floor; target the same floor. No gouge.
        let prog = ProgramBuilder::new()
            .op(0)
            .feed(300.0)
            .rapid(Point3::new(2.0, 20.0, 5.0), MoveKind::Link)
            .linear(Point3::new(2.0, 20.0, -3.0), MoveKind::Plunge)
            .linear(Point3::new(38.0, 20.0, -3.0), MoveKind::Cutting)
            .rapid(Point3::new(38.0, 20.0, 5.0), MoveKind::Retract)
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts(), &[]);
        let mut target = Heightfield::new([0.0, 0.0], [40.0, 40.0], 0.5, 0.0);
        target.lower_rect([0.0, 16.0], [40.0, 24.0], -3.0);
        assert!(check_gouge(&sim.field, &target, 0.05).is_none());
    }

    #[test]
    fn cutting_below_the_target_is_flagged_as_a_gouge() {
        // Cut to -5 where the target only wanted -2: a 3 mm gouge.
        let prog = ProgramBuilder::new()
            .op(0)
            .feed(300.0)
            .rapid(Point3::new(2.0, 20.0, 5.0), MoveKind::Link)
            .linear(Point3::new(2.0, 20.0, -5.0), MoveKind::Plunge)
            .linear(Point3::new(38.0, 20.0, -5.0), MoveKind::Cutting)
            .rapid(Point3::new(38.0, 20.0, 5.0), MoveKind::Retract)
            .build();
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts(), &[]);
        let mut target = Heightfield::new([0.0, 0.0], [40.0, 40.0], 0.5, 0.0);
        target.lower_rect([0.0, 16.0], [40.0, 24.0], -2.0);
        let gouge = check_gouge(&sim.field, &target, 0.05).expect("gouge must be caught");
        assert_eq!(gouge.kind, CollisionKind::Gouge);
        assert!(
            (gouge.at[2] + 5.0).abs() < 0.2,
            "gouge Z near -5: {:?}",
            gouge.at
        );
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
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts(), &[]);
        assert!(sim.field.sample(10.0, 10.0) < -4.9, "hole 1 drilled");
        assert!(sim.field.sample(30.0, 30.0) < -4.9, "hole 2 drilled");
        assert!(
            sim.field.sample(20.0, 20.0) > -0.1,
            "between holes untouched"
        );
    }

    #[test]
    fn vtip_offset_is_the_inverse_of_the_toolpath_width_function() {
        // The sim's surface and the engraving strategy's groove width must be the
        // *same* geometry seen from two sides: if the cutting surface stands `z`
        // above the tip at radius `d`, then engraving `z` deep cuts a groove of
        // half-width `d`. Disagreement here means the backplot lies about the cut.
        for &(deg, rt) in &[(60.0, 0.0), (60.0, 0.2), (90.0, 0.5), (30.0, 0.1)] {
            let a = (deg * 0.5_f64).to_radians();
            let p = ToolProfile {
                radius: 3.0,
                shape: ProfileShape::VTip {
                    half_angle_rad: a,
                    tip_radius: rt,
                },
            };
            for step in 1..30 {
                let d = step as f64 * 0.1;
                let z = p.offset(d);
                let back = cam_geo::vtip_half_width(a, rt, z);
                assert!(
                    (back - d).abs() < 1e-9,
                    "deg={deg} rt={rt} d={d} z={z} back={back}"
                );
            }
        }
    }

    #[test]
    fn a_vtip_is_not_the_same_surface_as_a_flat_tipped_cone() {
        // Guards the approximation this replaced: modelling a V-bit as a cone with a
        // flat of the tip radius. Near the tip the two differ materially.
        let a = (30.0_f64).to_radians();
        let rt = 0.4;
        let vtip = ToolProfile {
            radius: 3.0,
            shape: ProfileShape::VTip {
                half_angle_rad: a,
                tip_radius: rt,
            },
        };
        let flat = ToolProfile {
            radius: 3.0,
            shape: ProfileShape::Cone {
                half_angle_rad: a,
                flat_radius: rt,
            },
        };
        // The old model says the tool is flat out to rt; the real one is already
        // climbing, so it removes less material there.
        assert_eq!(flat.offset(0.3), 0.0);
        assert!(vtip.offset(0.3) > 0.05, "{}", vtip.offset(0.3));
    }

    #[test]
    fn profile_offsets_match_geometry() {
        // Flat: no rise anywhere.
        let flat = ToolProfile::flat(2.0);
        assert_eq!(flat.offset(0.0), 0.0);
        assert_eq!(flat.offset(2.0), 0.0);

        // Ball R2: sphere — 0 at the axis, R at the rim.
        let ball = ToolProfile {
            radius: 2.0,
            shape: ProfileShape::Ball,
        };
        assert!(ball.offset(0.0).abs() < 1e-9);
        assert!((ball.offset(2.0) - 2.0).abs() < 1e-9);
        assert!((ball.offset(1.5) - (2.0 - (4.0_f64 - 2.25).sqrt())).abs() < 1e-9);

        // Bull-nose R3, corner 1: flat out to d=2, then a 1 mm corner arc.
        let bull = ToolProfile {
            radius: 3.0,
            shape: ProfileShape::BullNose { corner_radius: 1.0 },
        };
        assert_eq!(bull.offset(2.0), 0.0);
        assert!((bull.offset(3.0) - 1.0).abs() < 1e-9);

        // 90° cone (α=45°): rises 1:1 with radius past a 0.5 mm flat tip.
        let cone = ToolProfile {
            radius: 5.0,
            shape: ProfileShape::Cone {
                half_angle_rad: std::f64::consts::FRAC_PI_4,
                flat_radius: 0.5,
            },
        };
        assert_eq!(cone.offset(0.5), 0.0);
        assert!((cone.offset(2.5) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn a_ball_mill_leaves_a_rounded_floor() {
        let mut hf = Heightfield::new([0.0, 0.0], [10.0, 10.0], 0.25, 0.0);
        let ball = ToolProfile {
            radius: 2.0,
            shape: ProfileShape::Ball,
        };
        // Cut along y=5 at Z −3.
        hf.cut_segment_profile([2.0, 5.0, -3.0], [8.0, 5.0, -3.0], &ball);
        let center = hf.sample(5.0, 5.0); // on the tool axis → deepest
        let side = hf.sample(5.0, 6.5); // 1.5 mm off → floor rises
        assert!((center + 3.0).abs() < 0.3, "axis floor near −3: {center}");
        assert!(side > center + 0.3, "floor rounds up toward the edge");
    }

    #[test]
    fn simulate_honours_tool_change_profile() {
        // Select tool 2 (a ball mill) and cut a straight slot; the floor must be
        // rounded — proof the sim tracked the tool change, not the default flat.
        let prog = ProgramBuilder::new()
            .tool_change(2)
            .op(0)
            .feed(300.0)
            .rapid(Point3::new(2.0, 20.0, 5.0), MoveKind::Link)
            .linear(Point3::new(2.0, 20.0, -3.0), MoveKind::Plunge)
            .linear(Point3::new(38.0, 20.0, -3.0), MoveKind::Cutting)
            .rapid(Point3::new(38.0, 20.0, 5.0), MoveKind::Retract)
            .build();
        let tools = [SimTool {
            number: 2,
            profile: ToolProfile {
                radius: 2.0,
                shape: ProfileShape::Ball,
            },
        }];
        let sim = simulate(&prog, STOCK_MIN, STOCK_MAX, &opts(), &tools);
        let center = sim.field.sample(20.0, 20.0);
        let side = sim.field.sample(20.0, 21.5);
        assert!((center + 3.0).abs() < 0.3, "slot axis near −3: {center}");
        assert!(side > center + 0.3, "ball tool leaves a rounded slot floor");
    }
}
