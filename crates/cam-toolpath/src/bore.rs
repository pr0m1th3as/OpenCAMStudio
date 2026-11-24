//! Helical bore milling. Straight walls use half-turn G2/G3 arcs; tapered
//! spirals use chord-error-bounded lines. Each pass completes a bottom circle
//! before disengaging, so fractional turns cannot leave an uncut bottom sector.
use crate::{CancelToken, Diagnostic, JobEnv, Strategy, StrategyResult};
use cam_cldata::{ArcDir, MoveKind, Point3, Program, Step, Tag};
use cam_model::{BoreOp, BoreOrder};
use std::f64::consts::{PI, TAU};

#[derive(Clone, Debug)]
pub struct BoreStrategy {
    op: BoreOp,
}
impl BoreStrategy {
    pub fn new(op: BoreOp) -> Self {
        Self { op }
    }
}

impl Strategy for BoreStrategy {
    fn name(&self) -> &str {
        "bore"
    }
    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        if self.op.parameter_expressions.is_empty() {
            return self.compute_resolved_parameters(env, cancel);
        }
        let mut operation = cam_model::Operation::Bore(self.op.clone());
        if let Err(message) = operation.resolve_parameter_expressions(env.tools) {
            return StrategyResult {
                diagnostics: vec![crate::Diagnostic::error(message)],
                ..Default::default()
            };
        }
        let cam_model::Operation::Bore(op) = operation else {
            unreachable!()
        };
        Self::new(op).compute_resolved_parameters(env, cancel)
    }
}
impl BoreStrategy {
    fn compute_resolved_parameters(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        if cancel.is_cancelled() {
            return StrategyResult {
                cancelled: true,
                ..Default::default()
            };
        }
        let op = &self.op;
        let fail = |reason: &str| StrategyResult {
            diagnostics: vec![Diagnostic::error(format!("Bore {}: {reason}", op.id))],
            ..Default::default()
        };
        let Some(tool) = env.tool(op.tool) else {
            return fail("tool is missing from the setup");
        };
        if let Err(reason) = op.entry_feeds.validate() {
            return fail(&reason);
        }
        if [
            op.diameter,
            op.z_top,
            op.z_bottom,
            op.taper_angle,
            op.pitch,
            op.tolerance,
            op.stepover,
            op.finishing_stepover,
            op.radial_stock,
            op.axial_stock,
            op.safe_distance,
            op.lead_radius,
            op.feed,
            op.plunge_feed,
            tool.diameter,
            env.heights.clearance,
            env.heights.retract,
        ]
        .iter()
        .any(|v| !v.is_finite())
            || op.points.iter().flatten().any(|v| !v.is_finite())
        {
            return fail("dimensions, centres, heights and feeds must be finite");
        }
        if op.points.is_empty() {
            return fail("select at least one circular feature");
        }
        if op.diameter <= 0.
            || tool.diameter <= 0.
            || op.pitch <= 0.
            || op.tolerance <= 0.
            || op.feed <= 0.
            || op.plunge_feed <= 0.
            || op.safe_distance <= 0.
            || op.lead_radius < 0.
            || op.finishing_stepover < 0.
            || op.stepover < 0.
            || (op.stepovers > 0 && op.stepover == 0.)
        {
            return fail("diameter, pitch, tolerance, feeds and safe distance must be positive; pass and lead sizes cannot be negative");
        }
        if op
            .ramp_angle
            .is_some_and(|a| !a.is_finite() || a <= 0. || a >= 90.)
        {
            return fail("ramp angle must be between 0 and 90 degrees");
        }
        if op.taper_angle.abs() >= 80.
            || (op.internal && op.taper_angle < 0.)
            || (!op.internal && op.taper_angle > 0.)
        {
            return fail("taper would undercut with a cylindrical mill: holes must narrow downward and bosses must widen downward (angle magnitude below 80 degrees)");
        }
        let bottom = op.z_bottom + op.axial_stock;
        let depth = op.z_top - bottom;
        if op.z_top <= op.z_bottom
            || depth <= 0.
            || !depth.is_finite()
            || env.heights.retract < op.z_top + op.safe_distance
            || env.heights.clearance < env.heights.retract
        {
            return fail("top must exceed bottom plus axial stock; retract must clear top by safe distance and clearance must be at least retract");
        }
        if op.radial_stock <= -tool.radius() {
            return fail("negative radial stock must be smaller than the tool radius");
        }
        let mut diagnostics = Vec::new();
        crate::guards::warn_side_milling(op.id, "bore", tool, depth, &mut diagnostics);
        crate::guards::warn_flat_floor(op.id, "bore", tool, &mut diagnostics);
        // Material is outside a hole and inside a boss. All radial allowances
        // use this same sign, including tool-centre compensation.
        let side = if op.internal { -1. } else { 1. };
        let slope = op.taper_angle.to_radians().tan();
        let wall_top = op.diameter / 2.;
        let wall_bottom = wall_top - depth * slope;
        if wall_top.min(wall_bottom) + side * op.radial_stock <= 0. || !wall_bottom.is_finite() {
            return fail("taper or stock allowance collapses the feature before its bottom");
        }
        let final_top = wall_top + side * (tool.radius() + op.radial_stock);
        let final_bottom = wall_bottom + side * (tool.radius() + op.radial_stock);
        if op.stepovers > 10000 || op.points.len() > 10000 {
            return fail("too many passes or features");
        }
        let rough_allowance = op.stepovers as f64 * op.stepover + op.finishing_stepover;
        let min_orbit =
            final_top.min(final_bottom) + if op.internal { -rough_allowance } else { 0. };
        let max_orbit =
            final_top.max(final_bottom) + if op.internal { 0. } else { rough_allowance };
        if min_orbit <= 1e-6 || !max_orbit.is_finite() {
            return fail("tool and radial passes do not fit inside the smallest bore diameter");
        }
        if op
            .points
            .iter()
            .flatten()
            .any(|c| !(c.abs() + max_orbit + 2. * op.lead_radius.max(op.safe_distance)).is_finite())
        {
            return fail("feature coordinates exceed the representable machining range");
        }
        let pitch = op
            .ramp_angle
            .map_or(op.pitch, |a| TAU * min_orbit * a.to_radians().tan());
        let sweep = depth / pitch * TAU;
        // For a linear-radius spiral, |d²p/dθ²| <= r_max + 2|dr/dθ|.
        // The interpolation error is bounded by M * Δθ² / 8.
        let tapered = slope.abs() > 1e-12;
        let segment_angle = if tapered {
            (8. * op.tolerance / (max_orbit + 2. * (depth * slope / sweep).abs()))
                .sqrt()
                .min(PI / 8.)
        } else {
            PI
        };
        let segments = (sweep / segment_angle).ceil().max(1.);
        let pass_count = op.stepovers as usize
            + 1
            + usize::from(op.finishing_stepover > 0.)
            + usize::from(op.repeat_pass);
        if !segments.is_finite()
            || !sweep.is_finite()
            || sweep <= 0.
            || (segments + 9.) * pass_count as f64 * op.points.len() as f64 > 1_000_000.
        {
            return fail(
                "pitch or tolerance produces too many segments; increase it or split the operation",
            );
        }
        let ccw = (op.climb == op.internal) == env.climb_follows_winding();
        let dir = if ccw { ArcDir::Ccw } else { ArcDir::Cw };
        let sign = if ccw { 1. } else { -1. };
        let mut allowances: Vec<f64> = (0..=op.stepovers)
            .rev()
            .map(|i| i as f64 * op.stepover + op.finishing_stepover)
            .collect();
        if op.finishing_stepover > 0. {
            allowances.push(0.);
        }
        if op.repeat_pass {
            allowances.push(0.);
        }
        let mut program = Program::new();
        program.push(Step::Comment(format!(
            "Bore: {} diameter {:.3}, {}",
            if op.internal { "hole" } else { "boss" },
            op.diameter,
            crate::passes_phrase(pass_count)
        )));
        for [cx, cy] in ordered_points(op, cancel) {
            for allowance in &allowances {
                if cancel.is_cancelled() {
                    return StrategyResult {
                        program,
                        diagnostics,
                        cancelled: true,
                    };
                }
                let rt = final_top + side * allowance;
                let rb = final_bottom + side * allowance;
                let lead_r = |r: f64| {
                    if op.internal {
                        if op.lead_to_center {
                            r / 2.
                        } else {
                            op.lead_radius.min(r / 2.)
                        }
                    } else {
                        op.lead_radius.max(op.safe_distance / 2.)
                    }
                };
                let point = |r: f64, angle: f64, z: f64| {
                    Point3::new(cx + r * angle.cos(), cy + r * angle.sin(), z)
                };
                let tag = |kind| Tag::new(op.id, kind);
                let lead_dir = if op.internal {
                    dir
                } else if ccw {
                    ArcDir::Cw
                } else {
                    ArcDir::Ccw
                };
                let entry_r = rt + side * 2. * lead_r(rt);
                program.push(Step::Rapid {
                    to: point(entry_r, 0., env.heights.clearance),
                    tag: tag(MoveKind::Link),
                });
                program.push(Step::Rapid {
                    to: point(entry_r, 0., env.heights.retract),
                    tag: tag(MoveKind::Link),
                });
                program.push(Step::Linear {
                    to: point(entry_r, 0., op.z_top),
                    feed: op.plunge_feed,
                    tag: tag(MoveKind::Plunge),
                });
                if lead_r(rt) > 0. {
                    program.push(Step::Arc {
                        end: point(rt, 0., op.z_top),
                        center: point(rt + side * lead_r(rt), 0., op.z_top),
                        dir: lead_dir,
                        feed: op.entry_feeds.lead_in(op.feed),
                        tag: tag(MoveKind::LeadIn),
                    });
                }
                for i in 1..=segments as usize {
                    if cancel.is_cancelled() {
                        return StrategyResult {
                            program,
                            diagnostics,
                            cancelled: true,
                        };
                    }
                    let fraction = i as f64 / segments;
                    let z = op.z_top - fraction * depth;
                    let end = point(rt + fraction * (rb - rt), sign * sweep * fraction, z);
                    let feed = op.entry_feeds.ramp(op.feed);
                    if tapered {
                        program.push(Step::Linear {
                            to: end,
                            feed,
                            tag: tag(MoveKind::Cutting),
                        });
                    } else {
                        program.push(Step::Arc {
                            end,
                            center: Point3::new(cx, cy, z),
                            dir,
                            feed,
                            tag: tag(MoveKind::Cutting),
                        });
                    }
                }
                let angle = sign * sweep;
                for half in 1..=2 {
                    program.push(Step::Arc {
                        end: point(rb, angle + sign * PI * half as f64, bottom),
                        center: Point3::new(cx, cy, bottom),
                        dir,
                        feed: op.feed,
                        tag: tag(MoveKind::Cutting),
                    });
                }
                let exit_r = rb + side * 2. * lead_r(rb);
                if lead_r(rb) > 0. {
                    program.push(Step::Arc {
                        end: point(exit_r, angle, bottom),
                        center: point(rb + side * lead_r(rb), angle, bottom),
                        dir: lead_dir,
                        feed: op.entry_feeds.lead_out(op.feed),
                        tag: tag(MoveKind::LeadOut),
                    });
                }
                program.push(Step::Rapid {
                    to: point(exit_r, angle, env.heights.clearance),
                    tag: tag(MoveKind::Retract),
                });
            }
        }
        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

fn ordered_points(op: &BoreOp, cancel: &CancelToken) -> Vec<[f64; 2]> {
    let mut points = op.points.clone();
    order_holes(&mut points, op.order, |point| *point, cancel);
    if op.reverse_order {
        points.reverse();
    }
    points
}

pub(crate) fn order_holes<T>(
    holes: &mut [T],
    order: BoreOrder,
    point: impl Fn(&T) -> [f64; 2],
    cancel: &CancelToken,
) {
    let distance = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]);
    let sort_axis = |a: &T, b: &T, axis: usize| {
        let (a, b) = (point(a), point(b));
        a[axis]
            .total_cmp(&b[axis])
            .then(a[1 - axis].total_cmp(&b[1 - axis]))
    };
    match order {
        BoreOrder::Selected => {}
        BoreOrder::X => holes.sort_by(|a, b| sort_axis(a, b, 0)),
        BoreOrder::Y => holes.sort_by(|a, b| sort_axis(a, b, 1)),
        BoreOrder::InsideOut => {
            let center = [
                holes.iter().map(|p| point(p)[0] / holes.len() as f64).sum(),
                holes.iter().map(|p| point(p)[1] / holes.len() as f64).sum(),
            ];
            holes.sort_by(|a, b| distance(point(a), center).total_cmp(&distance(point(b), center)));
        }
        BoreOrder::Optimized => {
            for i in 1..holes.len() {
                if cancel.is_cancelled() {
                    break;
                }
                let previous = point(&holes[i - 1]);
                let nearest = (i..holes.len())
                    .min_by(|&a, &b| {
                        distance(point(&holes[a]), previous)
                            .total_cmp(&distance(point(&holes[b]), previous))
                    })
                    .unwrap();
                holes[i..=nearest].rotate_right(1);
            }
        }
    }
}
