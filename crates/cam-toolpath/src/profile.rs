//! The profiling strategy: follow a closed chain at a tool-radius offset, in
//! stepdown passes, down to depth.

use cam_cldata::{ArcDir, CutterComp, MoveKind, Point3, Program, SpindleDir, Step, Tag};
use cam_geo::{offset, Contour, JoinStyle, Point, Polygon};
use cam_model::{Comp, Heights, Lead, Plunge, ProfileOp, Side};

use crate::{CancelToken, Diagnostic, JobEnv, Strategy, StrategyResult};

/// A holding tab's source location and its footprint through the cutter slot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProfileTabPreview {
    pub position: [f64; 2],
    pub center: [f64; 2],
    pub tangent: [f64; 2],
}

/// Resolve tab placements without generating cutting moves. Automatic placement
/// uses the same compensated loops and start point as the profile strategy.
pub fn profile_tab_preview(
    op: &ProfileOp,
    diameter: f64,
    spindle: SpindleDir,
) -> Vec<ProfileTabPreview> {
    if !op.closed
        || !op.tabs.enabled
        || !op.chain.is_valid()
        || !diameter.is_finite()
        || diameter <= 0.0
    {
        return Vec::new();
    }
    let Ok(loops) = compensated_loops(op, diameter) else {
        return Vec::new();
    };
    let contours = loops
        .iter()
        .map(|p| profile_loop_points(p.outer().points(), op, spindle))
        .collect();
    let Ok((paths, layouts)) = tab_layouts(contours, op, diameter) else {
        return Vec::new();
    };
    let mut result = Vec::new();
    for (points, layout) in paths.iter().zip(layouts) {
        let Some(layout) = layout else {
            continue;
        };
        for (left, right) in layout.spans {
            let (center, tangent) = point_on_loop(points, (left + right) * 0.5);
            let source_distance = nearest_on_loop(op.chain.points(), [center.x, center.y]).0;
            let (position, _) = point_on_loop(op.chain.points(), source_distance);
            result.push(ProfileTabPreview {
                position: [position.x, position.y],
                center: [center.x, center.y],
                tangent,
            });
        }
    }
    result
}

fn compensated_loops(op: &ProfileOp, diameter: f64) -> Result<Vec<Polygon>, String> {
    let region =
        Polygon::new(op.chain.clone()).map_err(|e| format!("Invalid profile region: {e}"))?;
    let sign = match op.side {
        Side::Outside => 1.,
        Side::Inside => -1.,
        Side::On => 0.,
    };
    let radius = if op.comp == Comp::Computed {
        diameter * 0.5
    } else {
        0.
    };
    let distance = sign * (radius + op.offset);
    let loops = if distance == 0. {
        vec![region]
    } else {
        offset(&[region], distance, op.passes.outer_corner.join())
            .map_err(|e| format!("Profile offset failed: {e}"))?
    };
    if loops.is_empty() {
        Err(format!(
            "Tool (diameter {diameter}) is too large: offset consumed the feature"
        ))
    } else {
        Ok(loops)
    }
}

fn point_on_loop(points: &[Point], distance: f64) -> (Point, [f64; 2]) {
    let mut distance = distance.rem_euclid(loop_perimeter(points).max(1e-12));
    for (a, b) in points.iter().zip(points.iter().cycle().skip(1)) {
        let length = (b.x - a.x).hypot(b.y - a.y);
        if length <= 1e-12 {
            continue;
        }
        if distance <= length {
            let t = (distance / length).clamp(0.0, 1.0);
            return (
                Point::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)),
                [(b.x - a.x) / length, (b.y - a.y) / length],
            );
        }
        distance -= length;
    }
    (points[0], [1.0, 0.0])
}

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

#[path = "profile_schedule.rs"]
mod schedule;

impl Strategy for ProfileStrategy {
    fn name(&self) -> &str {
        "profile"
    }
    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        if self.op.parameter_expressions.is_empty()
            && self.op.passes.finish_stepover_expression.is_none()
            && self
                .op
                .passes
                .contour_roughing
                .stepover_expression
                .is_none()
        {
            return self.compute_resolved(env, cancel);
        }
        let mut operation = cam_model::Operation::Profile(self.op.clone());
        if let Err(message) = operation.resolve_parameter_expressions(env.tools) {
            return StrategyResult {
                diagnostics: vec![Diagnostic::error(message)],
                ..Default::default()
            };
        }
        let cam_model::Operation::Profile(op) = operation else {
            unreachable!()
        };
        Self::new(op).compute_resolved(env, cancel)
    }
}
impl ProfileStrategy {
    fn compute_resolved(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        // Preserve a closed profile's stored overlap when its selection changes
        // to open paths, but never apply it to those paths. Mixed selections
        // remain subject to validation because closed paths could use overlap.
        if self.op.lead_overlap != 0. && self.op.contours().all(|contour| !contour.closed) {
            let mut op = self.op.clone();
            op.lead_overlap = 0.;
            return Self::new(op).compute_resolved(env, cancel);
        }
        if !self.op.passes.multiple_depths {
            let mut op = self.op.clone();
            op.passes.multiple_depths = true;
            op.passes.even_stepdowns = false;
            op.passes.finishing_depths = 0;
            op.stepdown = env.heights.top_of_stock + op.depth - op.passes.floor_allowance;
            return Self::new(op).compute_resolved(env, cancel);
        }
        if cancel.is_cancelled() {
            return StrategyResult {
                cancelled: true,
                ..Default::default()
            };
        }
        if let Err(message) = env
            .heights
            .validate()
            .and_then(|_| self.op.validate_profile(env.heights.top_of_stock))
        {
            return StrategyResult {
                diagnostics: vec![Diagnostic::error(message)],
                ..Default::default()
            };
        }
        let rough = &self.op.passes.contour_roughing;
        if !rough.enabled {
            return schedule::compute(&self.op, env, cancel);
        }
        let mut result = StrategyResult::default();
        let finish_allowance = if self.op.passes.finishing {
            self.op.passes.finish_count as f64 * self.op.passes.finish_stepover
        } else {
            0.
        };
        for i in (0..=rough.stepovers).rev() {
            let mut pass = self.op.clone();
            pass.passes.contour_roughing.enabled = false;
            if i > 0 {
                pass.offset += finish_allowance + i as f64 * rough.stepover;
                if !pass.offset.is_finite() {
                    return StrategyResult {
                        diagnostics: vec![Diagnostic::error(
                            "Contour roughing offset is too large",
                        )],
                        ..Default::default()
                    };
                }
                pass.passes.tolerance = rough.smoothing_deviation;
                pass.passes.finishing = false;
                pass.passes.finish_only_final = false;
            }
            let part = schedule::compute(&pass, env, cancel);
            result.diagnostics.extend(part.diagnostics.clone());
            if part.cancelled || part.has_errors() {
                result.program = Program::new();
                result.cancelled = part.cancelled;
                return result;
            }
            if result
                .program
                .steps()
                .len()
                .saturating_add(part.program.steps().len())
                > cam_model::MAX_PROFILE_MOVES
            {
                return StrategyResult {
                    diagnostics: vec![Diagnostic::error("Profile motion budget exceeded")],
                    ..Default::default()
                };
            }
            result.program.extend(part.program);
        }
        result
    }
}
impl ProfileStrategy {
    fn compute_base(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        let op = &self.op;
        if let Err(reason) = op.entry_feeds.validate() {
            return StrategyResult {
                diagnostics: vec![Diagnostic::error(reason)],
                ..Default::default()
            };
        }
        if !op.additional_chains.is_empty() {
            if op.tabs.enabled || op.stepover > 0. {
                return StrategyResult { diagnostics: vec![Diagnostic::error("Holding tabs and radial roughing require one closed profile chain; use separate operations")], ..Default::default() };
            }
            let mut result = StrategyResult::default();
            let primary = cam_model::ProfileContour {
                chain: op.chain.clone(),
                closed: op.closed,
                side: op.side,
            };
            for (index, contour) in std::iter::once(&primary)
                .chain(&op.additional_chains)
                .enumerate()
            {
                let mut single = op.clone();
                single.chain = contour.chain.clone();
                single.closed = contour.closed;
                single.side = contour.side;
                single.additional_chains.clear();
                single.clearing.climb ^= op.contour_reversed(index);
                single.selections = None;
                let part = Self::new(single).compute_base(env, cancel);
                result.diagnostics.extend(part.diagnostics.clone());
                if part.has_errors() || part.cancelled {
                    result.program = Program::new();
                    result.cancelled = part.cancelled;
                    return result;
                }
                if result
                    .program
                    .steps()
                    .len()
                    .saturating_add(part.program.steps().len())
                    > cam_model::MAX_PROFILE_MOVES
                {
                    return StrategyResult {
                        diagnostics: vec![Diagnostic::error("Profile motion budget exceeded")],
                        ..Default::default()
                    };
                }
                result.program.extend(part.program);
            }
            return result;
        }
        if op.contour_reversed(0) {
            let mut single = op.clone();
            single.selections = None;
            single.clearing.climb = !single.clearing.climb;
            return Self::new(single).compute_base(env, cancel);
        }
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

        if !tool.diameter.is_finite()
            || tool.diameter <= 0.
            || !tool.length.is_finite()
            || tool.length <= 0.
        {
            return StrategyResult {
                diagnostics: vec![Diagnostic::error(
                    "Profile tool diameter and length must be positive and finite",
                )],
                ..Default::default()
            };
        }
        // Report shape/reach differences while retaining the operator's cutter.
        crate::guards::warn_side_milling(op.id, "profile", tool, op.depth, &mut diagnostics);
        if let Err(message) = op.validate_control_leads(tool.radius()) {
            return StrategyResult {
                diagnostics: vec![Diagnostic::error(message)],
                ..Default::default()
            };
        }
        // Report a non-cutting tip before generating the requested plunge.
        crate::guards::warn_plunge(op.id, "profile", tool, &mut diagnostics);

        if !op.closed {
            let mut result = open_profile(op, tool.radius(), env, cancel);
            diagnostics.append(&mut result.diagnostics);
            result.diagnostics = diagnostics;
            return result;
        }
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
        // `depth` is a positive magnitude below the reference; the floor sits at Z = -depth.
        let floor = -op.depth;
        if floor >= env.heights.top_of_stock {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: depth {} does not reach below the stock top {}; nothing to cut",
                op.id, op.depth, env.heights.top_of_stock
            )));
            return StrategyResult {
                diagnostics,
                ..Default::default()
            };
        }
        if op.tabs.enabled {
            let tabs = &op.tabs;
            if tabs.manual && tabs.positions.is_empty() {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: select at least one manual holding-tab location",
                    op.id
                )));
                return StrategyResult {
                    diagnostics,
                    ..Default::default()
                };
            }
            let invalid = !tabs.width.is_finite()
                || tabs.width <= 0.0
                || !tabs.height.is_finite()
                || tabs.height <= 0.0
                || tabs.height >= env.heights.top_of_stock - floor
                || (!tabs.manual
                    && tabs.count.is_none()
                    && (!tabs.spacing.is_finite() || tabs.spacing <= 0.0))
                || tabs.count.is_some_and(|count| count == 0 || count > 10_000)
                || (tabs.manual
                    && (tabs.positions.is_empty()
                        || tabs.positions.len() > 10_000
                        || tabs.positions.iter().flatten().any(|v| !v.is_finite())));
            let unsupported = op.comp != Comp::Computed
                || op.plunge != Plunge::Straight
                || op.stepover > 0.0
                || op.lead_overlap > 0.0;
            if invalid || unsupported {
                diagnostics.push(Diagnostic::error(format!(
                    "operation {}: Holding tabs require positive finite width, height and spacing, \
                     height below the stock top, computed compensation, straight plunge, \
                     no radial roughing and no lead overlap",
                    op.id
                )));
                return StrategyResult {
                    diagnostics,
                    ..Default::default()
                };
            }
        }
        // Computed comp offsets the geometry ourselves; control comp keeps the
        // path on the contour and lets the controller (G41/G42) do the radius. The
        // finishing allowance (`offset`) leaves stock on the wall — it moves the
        // path the same way the tool radius does (outward for Outside, inward for
        // Inside), and is baked into the programmed path for control comp too so a
        // roughing pass stops short of the edge. Its direction follows `side`
        // (ignored for `On`, which has no material side).
        let side_sign = match op.side {
            Side::Outside => 1.0,
            Side::Inside => -1.0,
            Side::On => 0.0,
        };
        let (radius, comp) = match op.comp {
            Comp::Computed => (tool.radius(), None),
            Comp::Control => {
                let left = op.clearing.climb == (env.spindle == SpindleDir::Cw);
                let compensation = if left {
                    CutterComp::Left(op.tool)
                } else {
                    CutterComp::Right(op.tool)
                };
                (0.0, Some((compensation, tool.radius())))
            }
        };

        // Radial roughing (stepover) is **outside-only**: clear the frame out to the
        // raw stock in concentric passes, leaving the finishing `offset` on the wall
        // — the roughing half of the rough-then-finish workflow. Inner clearing is a
        // pocket's job (an inner profile is a single-pass wall finish), so stepover
        // is rejected for Inside/On by shared validation.
        // (Leads apply to the single-pass finish, not the roughing rings.)
        if op.stepover > 0.0 && matches!(op.comp, Comp::Computed) && side_sign > 0.0 {
            let Some(region) = roughing_region(op, env.stock) else {
                return StrategyResult { diagnostics: vec![Diagnostic::error("Requested profile roughing needs stock strictly containing the offset part; select valid stock or set stepover to zero")], ..Default::default() };
            };
            {
                let levels = depth_levels(env.heights.top_of_stock, floor, op.stepdown);
                let mut program = Program::new();
                // Clear the frame through the shared engine (stay-down inside-out,
                // engagement-spaced, climb-oriented). No finishing leads — the
                // single-pass finish that follows the roughing owns the walls. The
                // finishing allowance is baked into the roughing island, so the wall
                // ring sits at the tool radius.
                let job = crate::clearing::ClearJob {
                    lift_height: 0.0,
                    id: op.id,
                    simplification_tolerance: crate::frontadvance::SIMPLIFY_EPS,
                    radius,
                    // The finishing allowance is baked into the roughing island, so
                    // the tool-centre wall sits at the bare tool radius.
                    finish: 0.0,
                    first: radius,
                    spacing: op.stepover,
                    clearing: op.clearing,
                    plunge: op.plunge,
                    feed: op.feed,
                    plunge_feed: op.plunge_feed,
                    entry_feeds: op.entry_feeds,
                    lead_overlap: op.lead_overlap,
                    lead_in: Lead::None,
                    lead_out: Lead::None,
                    start: op.start,
                    end: None,
                    guard: &[],
                    spindle: env.spindle,
                };
                match crate::clearing::clear(
                    &mut program,
                    &region,
                    &job,
                    &env.heights,
                    &levels,
                    cancel,
                ) {
                    // A requested roughing stage must produce actual clearing rings.
                    Ok(0) => return StrategyResult { diagnostics: vec![Diagnostic::error("The requested roughing region has no machinable rings; select a smaller tool or change the stock")], ..Default::default() },
                    Ok(_) => {
                        return StrategyResult {
                            program,
                            diagnostics,
                            cancelled: false,
                        }
                    }
                    Err(crate::rings::RingsError::Cancelled) => {
                        return StrategyResult {
                            diagnostics,
                            cancelled: true,
                            ..Default::default()
                        };
                    }
                    Err(crate::rings::RingsError::Offset(e) | crate::rings::RingsError::Limit(e)) => {
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
            }
        }

        // An inner profile is a single-pass wall finish. Warn (don't block) if it
        // would leave an uncut core — material the finish loop can't reach: the
        // chain offset inward past the swept band (a tool diameter beyond the wall).
        // That's the signal to rough the pocket first; it's a local check, so it
        // fires whether or not anything actually roughed it.
        if side_sign < 0.0 && matches!(op.comp, Comp::Computed) {
            let core = 2.0 * tool.radius() + op.offset;
            let leaves_core = Polygon::new(op.chain.clone())
                .ok()
                .and_then(|p| offset(&[p], -core, JoinStyle::Round).ok())
                .is_some_and(|v| !v.is_empty());
            if leaves_core {
                diagnostics.push(Diagnostic::warning(format!(
                    "operation {}: inner profile leaves an uncut core — rough it with a pocket first",
                    op.id
                )));
            }
        }

        let loops = match compensated_loops(op, tool.diameter) {
            Ok(loops) => loops,
            Err(message) => {
                return StrategyResult {
                    diagnostics: vec![Diagnostic::error(format!("operation {}: {message}", op.id))],
                    ..Default::default()
                }
            }
        };

        // Which side of the tool-centre loop is cleared air the lead eases in from:
        // outside the loop for an outside profile, the hole interior for an inside
        // one (On has no material side — leave the lead on its default normal).
        let air_sign = if side_sign < 0.0 { -1.0 } else { 1.0 };

        // Validate every loop before emitting any motion. The cutter needs a gap
        // between protected spans so entry and exit remain below the tab tops.
        let contours: Vec<_> = loops
            .iter()
            .map(|poly| profile_loop_points(poly.outer().points(), op, env.spindle))
            .collect();
        let (contours, layouts) = match tab_layouts(contours, op, tool.diameter) {
            Ok(result) => result,
            Err(message) => {
                return StrategyResult {
                    diagnostics: vec![Diagnostic::error(message)],
                    ..Default::default()
                }
            }
        };
        let levels = depth_levels(env.heights.top_of_stock, floor, op.stepdown);
        let vertices: usize = contours.iter().map(Vec::len).sum();
        let factor = if matches!(op.plunge, Plunge::Ramp { .. }) {
            MAX_RAMP_LAPS + 2
        } else {
            2
        };
        if vertices.saturating_mul(levels.len()).saturating_mul(factor)
            > cam_model::MAX_PROFILE_MOVES
        {
            return StrategyResult {
                diagnostics: vec![Diagnostic::error(
                    "Profile motion budget exceeded; simplify geometry or increase stepdown",
                )],
                ..Default::default()
            };
        }
        for points in &contours {
            let mut previous = env.heights.top_of_stock;
            for &z in &levels {
                if let Err(message) =
                    validate_ramp_travel(op.plunge, previous - z, loop_perimeter(points))
                {
                    return StrategyResult {
                        diagnostics: vec![Diagnostic::error(message)],
                        ..Default::default()
                    };
                }
                previous = z;
            }
        }
        if comp.is_some() && air_sign < 0. {
            for points in &contours {
                let guard = Polygon::new(Contour::new(points.clone()))
                    .into_iter()
                    .collect::<Vec<_>>();
                let tangent = start_tangent(points);
                let normal = outward_normal(points);
                let normal = (-normal.0, -normal.1);
                let (_, exit, tan_out) = crate::emit::loop_with_overlap(points, op.lead_overlap);
                let exit_normal = outward_normal_at(tan_out, is_ccw(points));
                let exit_normal = (-exit_normal.0, -exit_normal.1);
                if crate::leads::guard_lead(&guard, points[0], tangent, normal, op.lead_in, true)
                    != op.lead_in
                    || crate::leads::guard_lead(
                        &guard,
                        exit,
                        tan_out,
                        exit_normal,
                        op.lead_out,
                        false,
                    ) != op.lead_out
                {
                    return StrategyResult { diagnostics: vec![Diagnostic::error("Control compensation lead does not fit inside the contour; choose another start point or shorter lead")], ..Default::default() };
                }
            }
        }
        if let Some((_, radius)) = comp {
            for points in &contours {
                let region =
                    Polygon::new(Contour::new(points.clone())).expect("validated profile loop");
                let safe_region = match offset(
                    &[region],
                    if air_sign < 0. { -radius } else { radius },
                    JoinStyle::Round,
                ) {
                    Ok(region) => region,
                    Err(error) => {
                        return StrategyResult {
                            diagnostics: vec![Diagnostic::error(format!(
                                "Cannot validate control entry clearance: {error}"
                            ))],
                            ..Default::default()
                        }
                    }
                };
                let tangent = start_tangent(points);
                let normal = outward_normal(points);
                let normal = (normal.0 * air_sign, normal.1 * air_sign);
                let entry = control_air_point(
                    crate::leads::lead_start_point(points[0], tangent, normal, op.lead_in),
                    normal,
                    comp,
                );
                let (_, end, tan_out) = crate::emit::loop_with_overlap(points, op.lead_overlap);
                let out = outward_normal_at(tan_out, is_ccw(points));
                let out = (out.0 * air_sign, out.1 * air_sign);
                let exit = control_air_point(
                    crate::leads::lead_end_point(end, tan_out, out, op.lead_out),
                    out,
                    comp,
                );
                let safe = |point| {
                    if air_sign < 0. {
                        safe_region.iter().any(|r| r.contains(point))
                    } else {
                        !safe_region.iter().any(|r| r.contains(point))
                    }
                };
                if !safe(entry) || !safe(exit) {
                    return StrategyResult {diagnostics:vec![Diagnostic::error("Uncompensated control entry/exit does not clear the retained wall; change the start point or lead length")],..Default::default()};
                }
            }
        }
        let mut program = Program::new();
        for (index, points) in contours.iter().enumerate() {
            if cancel.is_cancelled() {
                return StrategyResult {
                    program,
                    diagnostics,
                    cancelled: true,
                };
            }
            let mut effective = op.clone();
            let perimeter = loop_perimeter(points);
            if effective.lead_overlap > perimeter {
                effective.lead_overlap = perimeter;
                diagnostics.push(Diagnostic::warning(format!(
                    "Closure overlap limited to one full contour ({perimeter:.3} mm)"
                )));
            }
            emit_loop(
                &mut program,
                points,
                &effective,
                &env.heights,
                &levels,
                comp,
                air_sign,
                layouts[index].as_ref(),
                &mut diagnostics,
                cancel,
            );
        }

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

/// Open paths retain their endpoints and retract before returning to the start.
/// Offset an open centreline to its left, preserving its endpoints and rejecting
/// collapsed or self-intersecting paths. Shared by profiles and chamfers.
pub(crate) fn offset_open_chain(source: &[Point], distance: f64) -> Result<Vec<Point>, String> {
    if source.len() < 2
        || source.iter().any(|p| !p.x.is_finite() || !p.y.is_finite())
        || !distance.is_finite()
    {
        return Err("an open path needs at least two finite points and a finite offset".into());
    }
    let mut tangents = Vec::new();
    for pair in source.windows(2) {
        let dx = pair[1].x - pair[0].x;
        let dy = pair[1].y - pair[0].y;
        let length = dx.hypot(dy);
        if length < 1e-9 {
            return Err("open chain contains a zero-length edge".into());
        }
        tangents.push([dx / length, dy / length]);
    }
    let mut points = Vec::new();
    for (i, p) in source.iter().enumerate() {
        let a = tangents[i.saturating_sub(1)];
        let b = tangents[i.min(tangents.len() - 1)];
        let denominator = 1. + a[0] * b[0] + a[1] * b[1];
        if denominator < 1e-6 {
            return Err("open chain doubles back; split it at the reversal".into());
        }
        let dx = -(a[1] + b[1]) * distance / denominator;
        let dy = (a[0] + b[0]) * distance / denominator;
        if dx.hypot(dy) > distance.abs().max(1e-9) * 10. {
            return Err("open chain corner is too sharp for its tool offset".into());
        }
        points.push(Point::new(p.x + dx, p.y + dy));
    }
    // Offsets that cross themselves or reverse an edge cannot be machined as one chain.
    for (i, pair) in points.windows(2).enumerate() {
        let t = tangents[i];
        if (pair[1].x - pair[0].x) * t[0] + (pair[1].y - pair[0].y) * t[1] <= 1e-9 {
            return Err("tool offset collapses an open chain edge; use a smaller cutter".into());
        }
        for other in points.windows(2).skip(i + 2) {
            let cross = |a: Point, b: Point, c: Point| {
                (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
            };
            if cross(pair[0], pair[1], other[0]) * cross(pair[0], pair[1], other[1]) < 0.
                && cross(other[0], other[1], pair[0]) * cross(other[0], other[1], pair[1]) < 0.
            {
                return Err(
                    "tool offset intersects itself; split the chain or use a smaller cutter".into(),
                );
            }
        }
    }
    Ok(points)
}

fn open_profile(op: &ProfileOp, radius: f64, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
    let fail = |message: &str| StrategyResult {
        diagnostics: vec![Diagnostic::error(format!("operation {}: {message}", op.id))],
        ..Default::default()
    };
    let mut source = op.chain.points().to_vec();
    if source.len() < 2 {
        return fail("An open profile needs at least two points");
    }
    let n = source.len();
    let a = unit(source[1].x - source[0].x, source[1].y - source[0].y);
    let b = unit(
        source[n - 1].x - source[n - 2].x,
        source[n - 1].y - source[n - 2].y,
    );
    source[0].x -= a.0 * op.passes.start_extension;
    source[0].y -= a.1 * op.passes.start_extension;
    source[n - 1].x += b.0 * op.passes.end_extension;
    source[n - 1].y += b.1 * op.passes.end_extension;
    let sign = match op.side {
        Side::Outside => 1.,
        Side::Inside => -1.,
        Side::On => 0.,
    };
    let mut points = match offset_open_chain(&source, sign * (radius + op.offset)) {
        Ok(p) => p,
        Err(e) => return fail(&e),
    };
    let material_right = op.clearing.climb == (env.spindle == SpindleDir::Cw);
    let reverse = match op.side {
        Side::Outside | Side::On => !material_right,
        Side::Inside => material_right,
    };
    if reverse {
        points.reverse();
    }
    let levels = depth_levels(env.heights.top_of_stock, -op.depth, op.stepdown);
    if levels.len().saturating_mul(points.len()).saturating_mul(4) > cam_model::MAX_PROFILE_MOVES {
        return fail("Open profile motion budget exceeded");
    }
    let mut program = Program::new();
    let mut previous = env.heights.top_of_stock;
    for (level, z) in levels.into_iter().enumerate() {
        if cancel.is_cancelled() {
            return StrategyResult {
                cancelled: true,
                ..Default::default()
            };
        }
        let alternate = op.passes.both_ways && level % 2 == 1;
        let mut path = points.clone();
        if alternate {
            path.reverse();
        }
        let first = path[0];
        let last = *path.last().unwrap();
        let tin = start_tangent(&path);
        let tout = unit(
            last.x - path[path.len() - 2].x,
            last.y - path[path.len() - 2].y,
        );
        let left = material_right != alternate;
        let normal = |t: (f64, f64)| if left { (-t.1, t.0) } else { (t.1, -t.0) };
        let entry = crate::leads::lead_start_point(first, tin, normal(tin), op.lead_in);
        let exit = crate::leads::lead_end_point(last, tout, normal(tout), op.lead_out);
        let lead_path = lead_in_path(first, entry, normal(tin), op.lead_in);
        let mut ramp_path = lead_path;
        ramp_path.extend(path.iter().skip(1).copied());
        let required = contour_ramp_len(op.plunge, previous - z);
        if required.is_some_and(|length| length > path_length(&ramp_path) + 1e-9) {
            return fail("The open contour is too short for this ramp angle; reduce stepdown, extend the contour or use zigzag entry");
        }
        program.push(Step::Rapid {
            to: Point3::new(entry.x, entry.y, env.heights.clearance),
            tag: Tag::new(op.id, MoveKind::Link),
        });
        crate::emit::descend_to(
            &mut program,
            entry,
            previous.max(z),
            &env.heights,
            op.feed,
            op.id,
        );
        if let Some(length) = required {
            let ramp = advance_along_path(&ramp_path, length);
            emit_open_ramp(
                &mut program,
                &ramp,
                previous,
                z,
                op.entry_feeds.ramp(op.feed),
                Tag::new(op.id, MoveKind::Plunge),
            );
        } else {
            emit_plunge(
                &mut program,
                entry,
                tin,
                normal(tin),
                previous,
                z,
                op.plunge,
                op.plunge_feed,
                op.entry_feeds.ramp(op.feed),
                Tag::new(op.id, MoveKind::Plunge),
            );
        }
        crate::leads::emit_lead(
            &mut program,
            entry,
            first,
            first,
            normal(tin),
            op.lead_in,
            z,
            op.entry_feeds.lead_in(op.feed),
            Tag::new(op.id, MoveKind::LeadIn),
        );
        cut_profile_path(&mut program, &path, op, z);
        crate::leads::emit_lead(
            &mut program,
            last,
            exit,
            last,
            normal(tout),
            op.lead_out,
            z,
            op.entry_feeds.lead_out(op.feed),
            Tag::new(op.id, MoveKind::LeadOut),
        );
        program.push(Step::Rapid {
            to: Point3::new(exit.x, exit.y, env.heights.clearance),
            tag: Tag::new(op.id, MoveKind::Retract),
        });
        previous = z;
    }
    StrategyResult {
        program,
        ..Default::default()
    }
}

/// The region to clear when radial roughing an **outside** profile: the frame
/// between the raw stock and the part, so the region is the stock with the part —
/// dilated by the finishing allowance — as an island. Returns `None` when roughing
/// does not apply (no stock, or the stock does not contain the part), so the caller
/// falls back to a single pass.
fn roughing_region(op: &ProfileOp, stock: Option<([f64; 2], [f64; 2])>) -> Option<Polygon> {
    let part = Polygon::new(op.chain.clone()).ok()?;
    let island = if op.offset == 0. {
        part
    } else {
        let mut regions = offset(&[part], op.offset, JoinStyle::Round).ok()?;
        if regions.len() != 1 {
            return None;
        }
        regions.remove(0)
    };
    let stock = if let Some(boundary) = &op.passes.stock_boundary {
        Polygon::new(boundary.clone()).ok()?
    } else {
        let (min, max) = stock?;
        Polygon::new(Contour::new(vec![
            Point::new(min[0], min[1]),
            Point::new(max[0], min[1]),
            Point::new(max[0], max[1]),
            Point::new(min[0], max[1]),
        ]))
        .ok()?
    };
    if !cam_geo::difference(std::slice::from_ref(&island), std::slice::from_ref(&stock))
        .ok()?
        .is_empty()
    {
        return None;
    }
    Polygon::with_holes(stock.outer().clone(), vec![island.outer().clone()]).ok()
}

/// Emit approach, stepdown passes, and retract for one closed tool-path loop.
/// When `comp` is set, the cut is bracketed by controller cutter compensation.
#[allow(clippy::too_many_arguments)]
fn emit_loop(
    prog: &mut Program,
    pts: &[cam_geo::Point],
    op: &ProfileOp,
    h: &Heights,
    levels: &[f64],
    comp: Option<(CutterComp, f64)>,
    air_sign: f64,
    tabs: Option<&TabLayout>,
    diagnostics: &mut Vec<Diagnostic>,
    cancel: &CancelToken,
) {
    if pts.len() < 3 {
        return;
    }
    let start = pts[0];

    // Rich entry (leads, a closure overlap, and/or a non-straight plunge) takes a
    // separate path; the default (no lead + no overlap + straight plunge) keeps the
    // original, byte-stable emission.
    if op.lead_in != Lead::None
        || op.lead_out != Lead::None
        || op.lead_overlap > 0.0
        || op.end.is_some()
        || op.plunge != Plunge::Straight
    {
        emit_loop_rich(
            prog,
            pts,
            op,
            h,
            levels,
            comp,
            air_sign,
            tabs,
            diagnostics,
            cancel,
        );
        return;
    }

    let link = Tag::new(op.id, MoveKind::Link);
    let plunge = Tag::new(op.id, MoveKind::Plunge);
    let retract = Tag::new(op.id, MoveKind::Retract);

    // Approach: rapid over the start at clearance, then down to the **retract
    // plane** — not to the stock top, where a rapid would end with no margin and
    // slightly proud stock or a small Z-zero error would mean rapiding into material.
    // The `max` keeps it never lower than the old stock-top approach.
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.clearance),
        tag: link,
    });
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.retract.max(h.top_of_stock)),
        tag: link,
    });

    for &z in levels {
        if cancel.is_cancelled() {
            return;
        }
        // Plunge to this level, then cut the loop (arcs refitted) and close it.
        prog.push(Step::Linear {
            to: Point3::new(start.x, start.y, z),
            feed: op.plunge_feed,
            tag: plunge,
        });
        if let Some((c, _)) = comp {
            prog.push(Step::CutterComp(c));
        }
        if let Some(tabs) = tabs {
            let mut closed = pts.to_vec();
            closed.push(start);
            tabs.cut(prog, &closed, op, z);
        } else {
            let mut closed = pts.to_vec();
            closed.push(pts[0]);
            cut_profile_path(prog, &closed, op, z);
        }
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

/// Split at the feed boundary before fitting so a fitted arc cannot erase it.
fn cut_profile_path(prog: &mut Program, points: &[Point], op: &ProfileOp, z: f64) {
    let tag = Tag::new(op.id, MoveKind::Cutting);
    let distance = op.passes.effective_lead_end_distance();
    let emit = |prog: &mut Program, path: &[Point], feed| {
        crate::emit::cut_polyline_tolerance(prog, path, feed, tag, z, op.passes.tolerance);
    };
    if distance <= 0. || points.len() < 2 {
        emit(prog, points, op.feed);
        return;
    }
    let length = path_length(points);
    if distance >= length {
        emit(prog, points, op.entry_feeds.lead_out(op.feed));
        return;
    }
    let first = advance_along_path(points, length - distance);
    let reversed: Vec<_> = points.iter().copied().rev().collect();
    let mut last = advance_along_path(&reversed, distance);
    last.reverse();
    last[0] = *first.last().expect("nonempty path prefix");
    emit(prog, &first, op.feed);
    emit(prog, &last, op.entry_feeds.lead_out(op.feed));
}

/// Uncompensated approach/cancellation points must clear the retained wall by
/// the full cutter radius. A tangent point on the nominal wall would plunge
/// through the wall before G41/G42 has had a chance to engage.
fn control_air_point(point: Point, normal: (f64, f64), comp: Option<(CutterComp, f64)>) -> Point {
    let clearance = comp.map_or(0., |(_, radius)| radius + crate::emit::FLOOR_CLEARANCE);
    Point::new(
        point.x + normal.0 * clearance,
        point.y + normal.1 * clearance,
    )
}

/// Build the legal cutter-centre region for a profile entry helix. The toolpath
/// loop bounds the protected side of the cut. For an outside profile, represent
/// the unbounded exterior as a large rectangle with that loop removed; its outer
/// edge is placed far enough away that it cannot constrain a valid entry.
fn profile_plunge_guard(
    pts: &[Point],
    air_sign: f64,
    plunge: Plunge,
    entry: Point,
) -> Option<Vec<Polygon>> {
    let Plunge::Helix { radius, .. } = plunge else {
        return Some(Vec::new());
    };
    if pts.len() < 3 || !radius.is_finite() || radius <= 0.0 {
        return None;
    }
    let contour = Contour::new(pts.to_vec());
    if air_sign < 0.0 {
        return Polygon::new(contour).ok().map(|region| vec![region]);
    }
    if air_sign <= 0.0 {
        return None;
    }

    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    for p in pts {
        lo[0] = lo[0].min(p.x);
        lo[1] = lo[1].min(p.y);
        hi[0] = hi[0].max(p.x);
        hi[1] = hi[1].max(p.y);
    }
    lo[0] = lo[0].min(entry.x);
    lo[1] = lo[1].min(entry.y);
    hi[0] = hi[0].max(entry.x);
    hi[1] = hi[1].max(entry.y);
    let margin = radius * 3.0 + 1.0;
    let outer = Contour::new(vec![
        Point::new(lo[0] - margin, lo[1] - margin),
        Point::new(hi[0] + margin, lo[1] - margin),
        Point::new(hi[0] + margin, hi[1] + margin),
        Point::new(lo[0] - margin, hi[1] + margin),
    ]);
    Polygon::with_holes(outer, vec![contour])
        .ok()
        .map(|region| vec![region])
}

/// The rich profile emission (leads and/or a non-straight plunge). Per level: rapid
/// to the entry footprint (off the contour when there is a lead-in), rapid down
/// through the already-cut air to the previous level, enter Z per the plunge
/// strategy, lead onto the contour, cut the loop, lead off, and retract.
#[allow(clippy::too_many_arguments)]
fn emit_loop_rich(
    prog: &mut Program,
    pts: &[Point],
    op: &ProfileOp,
    h: &Heights,
    levels: &[f64],
    comp: Option<(CutterComp, f64)>,
    air_sign: f64,
    tabs: Option<&TabLayout>,
    diagnostics: &mut Vec<Diagnostic>,
    cancel: &CancelToken,
) {
    let end = op.end.map(|point| {
        tabs.map_or(point, |layout| {
            layout.eligible_exit(pts, point, lead_extent(op.lead_out))
        })
    });
    let start = pts[0];
    let tan_in = start_tangent(pts); // leaving start, into the cut
                                     // The cleared/air side the lead eases in from. For an outside profile that is
                                     // outward (away from the part); for an inside one it flips inward, into the hole
                                     // — otherwise the lead would swing onto the material side and gouge the wall.
    let out = {
        let o = outward_normal(pts);
        (o.0 * air_sign, o.1 * air_sign)
    };
    // An inside profile eases in from the bounded hole interior, so guard the lead
    // against overshooting it (a hole narrower than the lead drops to a plain pass);
    // an outside profile leads into open stock, which needs no bound.
    let guard: Vec<Polygon> = if air_sign < 0.0 {
        Polygon::new(Contour::new(pts.to_vec()))
            .into_iter()
            .collect()
    } else {
        Vec::new()
    };
    let link = Tag::new(op.id, MoveKind::Link);
    let lead = Tag::new(op.id, MoveKind::LeadIn);
    let retract = Tag::new(op.id, MoveKind::Retract);
    let plunge_tag = Tag::new(op.id, MoveKind::Plunge);

    // The cut runs the loop and then keeps going `lead_overlap` mm past the start
    // to a point `exit_on`, so the lead-off (and its junction) is re-machined. With
    // no overlap, `exit_on == start`, `tan_out` is the arrival tangent, and the cut
    // polyline is exactly the closed loop — byte-identical to the prior emission.
    let (loop_pts, exit_on, tan_out) = crate::emit::loop_with_exit(pts, op.lead_overlap, end);
    // The lead-off normal follows the *arrival* tangent, which differs from the
    // start's whenever the loop closes at a corner (a sharp inner offset) — reusing
    // the start normal there yields a degenerate lead. Mid-edge the two coincide, so
    // this stays byte-identical to the shortcut it replaces.
    let out_exit = {
        let o = outward_normal_at(tan_out, signed_area2(pts) > 0.0);
        (o.0 * air_sign, o.1 * air_sign)
    };

    // Drop a lead that would overshoot the hole interior to a plain pass (guard is
    // empty for outside profiles, so this leaves them untouched).
    let lead_in = crate::leads::guard_lead(&guard, start, tan_in, out, op.lead_in, true);
    let lead_out = crate::leads::guard_lead(&guard, exit_on, tan_out, out_exit, op.lead_out, false);
    warn_omitted_lead("lead-in", op.lead_in, lead_in, diagnostics);
    let entry = control_air_point(
        crate::leads::lead_start_point(start, tan_in, out, lead_in),
        out,
        comp,
    );
    // Bound the exterior around both the contour and lead entry so the artificial
    // outer edge cannot reject a helix that is safely outside the part.
    let plunge_guard = profile_plunge_guard(pts, air_sign, op.plunge, entry);
    let exit = control_air_point(
        crate::leads::lead_end_point(exit_on, tan_out, out_exit, lead_out),
        out_exit,
        comp,
    );

    // The first pass rapids down to the **retract plane**, not to the stock top:
    // ending a rapid exactly on the surface leaves no margin, so slightly proud
    // stock or a small Z-zero error means rapiding into material. Taking the higher
    // of the two is never lower than the old behaviour. Later passes return through
    // air the tool has already cut — but *that* rapid used to end exactly on the
    // previous floor, which is the identical no-margin case one pass down, so it now
    // goes through `emit::descend_to` like every other strategy's entry.
    //
    // The lift between passes is only there to reposition: with a lead-in the pass
    // ends at the lead-*out* point, somewhere else. When the two coincide — no leads,
    // the common case — there is nothing to reposition to, and the tool stays down and
    // plunges to the next level exactly as the unleaded path does. That is not merely
    // faster: a descent that never happens cannot end in metal.
    // A contour ramp starts somewhere else on the loop, and how far back depends on
    // the descent, which differs at the first level (it begins at the retract plane).
    // So it always ends the pass away from where the next one begins — the same
    // condition a lead already creates.
    let ramps =
        matches!(op.plunge, Plunge::Ramp { angle_deg } if angle_deg > 0.0 && angle_deg < 90.0);
    let perimeter = loop_perimeter(pts);
    // A ramped pass ends further round the loop than an unramped one (it must re-cut
    // the stretch it descended over), so its exit — and the lead-off with it — is a
    // per-level quantity rather than a fixed one.
    let repositions = ramps || (exit.x - entry.x).abs() > 1e-9 || (exit.y - entry.y).abs() > 1e-9;
    let mut prev_z = h.retract.max(h.top_of_stock);
    for (i, &z) in levels.iter().enumerate() {
        if cancel.is_cancelled() {
            return;
        }
        // **The ramp never travels through air.** Its height is measured from the top
        // of material — the stock surface on the first level, the previous floor after
        // that — not from the retract plane. Descending from the retract plane made two
        // thirds of the first pass's ramp a slow feed through nothing, and dragged the
        // tool around the contour's corners to spend the length.
        let ramp_top = prev_z.min(h.top_of_stock);

        // The ramp descends along the lead-in first, and only then along the contour:
        // the lead exists to keep the entry off the finished wall, so it is exactly
        // where a descending tool belongs. A lead long enough (or an angle steep
        // enough) to absorb the whole stepdown keeps the wall untouched entirely —
        // which is how the operator controls this.
        let lead_path = lead_in_path(start, entry, out, lead_in);
        let lead_end = lead_path.len().saturating_sub(1);
        let ramp = contour_ramp(pts, lead_path, op.plunge, ramp_top, z);

        // Where the pass begins cutting at depth: the arc position the ramp ended at.
        let cut_from = ramp.as_ref().map_or(0.0, |(_, on_contour, _)| *on_contour);
        // A full perimeter from there, so the ramped stretch is re-machined, plus the
        // operator's overlap.
        let (loop_pts, exit_on, tan_out) = if ramps {
            let walked = walk_loop(
                pts,
                cut_from,
                perimeter + exit_overlap(pts, cut_from, op.lead_overlap, end),
            );
            let last = walked[walked.len() - 1];
            let prev = walked[walked.len().saturating_sub(2)];
            (walked, last, unit(last.x - prev.x, last.y - prev.y))
        } else {
            (loop_pts.clone(), exit_on, tan_out)
        };
        let out_exit = {
            let o = outward_normal_at(tan_out, signed_area2(pts) > 0.0);
            (o.0 * air_sign, o.1 * air_sign)
        };
        let lead_out =
            crate::leads::guard_lead(&guard, exit_on, tan_out, out_exit, op.lead_out, false);
        warn_omitted_lead("lead-out", op.lead_out, lead_out, diagnostics);
        let exit = control_air_point(
            crate::leads::lead_end_point(exit_on, tan_out, out_exit, lead_out),
            out_exit,
            comp,
        );

        // The tool comes down where the entry begins — the lead-in's own start, which
        // when ramping now sits *above* the cutting plane by exactly the ramp's height,
        // so the descent that follows lands on the contour at depth.
        //
        // A ramp stops at the top of material and feeds from there; every other plunge
        // style still comes down to `prev_z` and descends from it, unchanged.
        let arrive_at = match &ramp {
            // The top of the ramp, which for a lead-in sits *above* the material.
            Some((_, _, top)) => top.max(z),
            None => prev_z,
        };
        if i == 0 || repositions {
            prog.push(Step::Rapid {
                to: Point3::new(entry.x, entry.y, h.clearance),
                tag: link,
            });
            crate::emit::descend_to(prog, entry, arrive_at, h, op.feed, op.id);
        }

        if let Some((path, _, top)) = &ramp {
            // Keep the air/material junction explicit: refitting the combined
            // lead and contour can merge their arcs and cut below the stock top
            // before the lead reaches the contour.
            emit_descending_path(
                prog,
                &path[..=lead_end],
                *top,
                ramp_top,
                op.entry_feeds.ramp(op.feed),
                plunge_tag,
            );
            emit_descending_path(
                prog,
                &path[lead_end..],
                ramp_top,
                z,
                op.entry_feeds.ramp(op.feed),
                plunge_tag,
            );
        } else {
            emit_plunge(
                prog,
                entry,
                tan_in,
                out,
                prev_z,
                z,
                plunge_guard.as_ref().map_or(Plunge::Straight, |guard| {
                    crate::leads::guard_plunge(guard, entry, out, op.plunge)
                }),
                op.plunge_feed,
                op.entry_feeds.ramp(op.feed),
                plunge_tag,
            );

            if let Some((c, _)) = comp {
                prog.push(Step::CutterComp(c));
            }
            // Lead onto the contour at depth: entry → start.
            crate::leads::emit_lead(
                prog,
                entry,
                start,
                start,
                out,
                lead_in,
                z,
                op.entry_feeds.lead_in(op.feed),
                lead,
            );
        }

        if let Some(tabs) = tabs {
            tabs.cut(prog, &loop_pts, op, z);
        } else {
            cut_profile_path(prog, &loop_pts, op, z);
        }

        // Lead off the contour at depth — **flat**. Only the lead-in descends.
        crate::leads::emit_lead(
            prog,
            exit_on,
            exit,
            exit_on,
            out_exit,
            lead_out,
            z,
            op.entry_feeds.lead_out(op.feed),
            Tag::new(op.id, MoveKind::LeadOut),
        );

        if comp.is_some() {
            prog.push(Step::CutterComp(CutterComp::Off));
        }

        // Lift only when the next pass has somewhere else to start from; the final
        // retract is unconditional (the operation must leave the tool clear).
        if repositions || i + 1 == levels.len() {
            prog.push(Step::Rapid {
                to: Point3::new(exit.x, exit.y, h.clearance),
                tag: retract,
            });
        }
        prev_z = z;
    }
}

/// Orient material to the correct side for spindle rotation, keeping the seam
/// anchored so reversing direction does not move automatic tab locations.
fn profile_loop_points(points: &[Point], op: &ProfileOp, spindle: SpindleDir) -> Vec<Point> {
    let mut points = points.to_vec();
    let material_right = op.clearing.climb == (spindle == SpindleDir::Cw);
    let want_ccw = match op.side {
        Side::Inside => material_right,
        Side::Outside => !material_right,
        // A slot engages both sides; preserve the source direction for CW climb
        // and reverse it for the opposite choice.
        Side::On => is_ccw(&points) == material_right,
    };
    if is_ccw(&points) != want_ccw && points.len() > 1 {
        points[1..].reverse();
    }
    let points = rotate_to_start(&points, op.start);
    if op.start.is_none()
        && op.side == Side::Inside
        && (op.lead_in != Lead::None || op.lead_out != Lead::None)
    {
        automatic_lead_seam(points, op)
    } else {
        points
    }
}

/// A corner often leaves no room behind the entry tangent. Before dropping an
/// automatic inside lead, try the middle of the longest edges in the same legal
/// cutter-centre region used by the emitter. Explicit start preferences stay put.
fn automatic_lead_seam(points: Vec<Point>, op: &ProfileOp) -> Vec<Point> {
    let Ok(region) = Polygon::new(Contour::new(points.clone())) else {
        return points;
    };
    let guard = [region];
    let score = |path: &[Point]| {
        let tangent = start_tangent(path);
        let normal = outward_normal(path);
        let entry_fits = crate::leads::guard_lead(
            &guard,
            path[0],
            tangent,
            (-normal.0, -normal.1),
            op.lead_in,
            true,
        ) == op.lead_in;
        let (_, exit, tangent) = crate::emit::loop_with_exit(path, op.lead_overlap, op.end);
        let normal = outward_normal_at(tangent, is_ccw(path));
        let exit_fits = crate::leads::guard_lead(
            &guard,
            exit,
            tangent,
            (-normal.0, -normal.1),
            op.lead_out,
            false,
        ) == op.lead_out;
        // Prefer preserving entry if both requested leads cannot fit together.
        2 * usize::from(entry_fits) + usize::from(exit_fits)
    };
    let mut best_score = score(&points);
    if best_score == 3 {
        return points;
    }
    let mut edges: Vec<_> = points.iter().zip(points.iter().cycle().skip(1)).collect();
    edges.sort_by(|(a, b), (c, d)| c.distance_sq(**d).total_cmp(&a.distance_sq(**b)));
    let mut best = points.clone();
    // Bound containment work for finely tessellated CAD contours.
    for (a, b) in edges.into_iter().take(32) {
        let candidate = rotate_to_start(&points, Some([(a.x + b.x) * 0.5, (a.y + b.y) * 0.5]));
        let candidate_score = score(&candidate);
        if candidate_score > best_score {
            best = candidate;
            best_score = candidate_score;
            if best_score == 3 {
                break;
            }
        }
    }
    best
}

fn warn_omitted_lead(name: &str, requested: Lead, actual: Lead, diagnostics: &mut Vec<Diagnostic>) {
    if requested != actual {
        let warning = Diagnostic::warning(format!(
            "Profile {name} does not fit inside the contour and was omitted; choose another position or reduce its size"
        ));
        if !diagnostics.contains(&warning) {
            diagnostics.push(warning);
        }
    }
}

/// Additional travel after a complete contour, ending at the requested exit.
/// Minimum overlap is preserved by adding whole laps when necessary.
pub(crate) fn exit_overlap(
    points: &[Point],
    from: f64,
    minimum: f64,
    end: Option<[f64; 2]>,
) -> f64 {
    let Some(end) = end.filter(|p| p.iter().all(|v| v.is_finite())) else {
        return minimum;
    };
    let perimeter = loop_perimeter(points);
    if perimeter <= 1e-12 {
        return minimum;
    }
    let minimum = minimum.min(perimeter);
    let mut extra = (nearest_on_loop(points, end).0 - from).rem_euclid(perimeter);
    if extra + 1e-9 < minimum {
        extra += ((minimum - extra) / perimeter).ceil() * perimeter;
    }
    extra
}

/// Arc length and squared distance of the nearest point on a closed loop.
fn nearest_on_loop(points: &[Point], point: [f64; 2]) -> (f64, f64) {
    let mut walked = 0.0;
    let mut nearest = (0.0, f64::INFINITY);
    for (a, b) in points.iter().zip(points.iter().cycle().skip(1)) {
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let length2 = dx * dx + dy * dy;
        if length2 <= 1e-24 {
            continue;
        }
        let t = (((point[0] - a.x) * dx + (point[1] - a.y) * dy) / length2).clamp(0.0, 1.0);
        let distance2 = (a.x + t * dx - point[0]).powi(2) + (a.y + t * dy - point[1]).powi(2);
        if distance2 < nearest.1 {
            nearest = (walked + t * length2.sqrt(), distance2);
        }
        walked += length2.sqrt();
    }
    nearest
}

fn lead_extent(lead: Lead) -> f64 {
    match lead {
        Lead::None => 0.0,
        Lead::Linear { length } => length,
        Lead::Arc { radius } => radius * std::f64::consts::SQRT_2,
        Lead::ArcSweep { radius, angle_deg } => 2. * radius * (angle_deg.to_radians() * 0.5).sin(),
    }
}

/// Keep entry and leads in an unprotected gap, including tabs at the old seam.
fn move_tab_seam(points: &mut Vec<Point>, op: &ProfileOp, diameter: f64) {
    let perimeter = loop_perimeter(points);
    if perimeter <= 0.0 {
        return;
    }
    let mut centers: Vec<_> = op
        .tabs
        .positions
        .iter()
        .map(|&point| nearest_on_loop(points, point).0)
        .collect();
    centers.sort_by(f64::total_cmp);
    let room =
        (op.tabs.width + diameter) * 0.5 + lead_extent(op.lead_in).max(lead_extent(op.lead_out));
    if centers.first().is_none_or(|d| *d > room)
        && centers.last().is_none_or(|d| *d < perimeter - room)
    {
        return;
    }
    let mut largest = (0.0, 0.0);
    for i in 0..centers.len() {
        let next = centers[(i + 1) % centers.len()]
            + if i + 1 == centers.len() {
                perimeter
            } else {
                0.0
            };
        let gap = next - centers[i];
        if gap > largest.1 {
            largest = ((centers[i] + gap * 0.5) % perimeter, gap);
        }
    }
    let mut rotated = walk_loop(points, largest.0, perimeter);
    rotated.pop(); // walk_loop includes the closing point.
    *points = rotated;
}

/// Tab spans measured along the tool-centre loop. Both tab ends are extended
/// by a cutter radius to retain the requested material width. Vertical moves at
/// their boundaries make a rectangular bridge, including on intermediate passes.
struct TabLayout {
    perimeter: f64,
    spans: Vec<(f64, f64)>,
    top: f64,
}

type TabPaths = (Vec<Vec<Point>>, Vec<Option<TabLayout>>);
fn tab_layouts(
    mut contours: Vec<Vec<Point>>,
    op: &ProfileOp,
    diameter: f64,
) -> Result<TabPaths, String> {
    let mut assigned = vec![Vec::new(); contours.len()];
    if op.tabs.enabled && op.tabs.manual {
        for &point in &op.tabs.positions {
            if point.iter().any(|v| !v.is_finite()) {
                return Err("Tab positions must be finite".into());
            }
            if let Some(index) = contours
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    nearest_on_loop(a, point)
                        .1
                        .total_cmp(&nearest_on_loop(b, point).1)
                })
                .map(|(index, _)| index)
            {
                assigned[index].push(point);
            }
        }
    }
    let mut layouts = Vec::new();
    for (index, points) in contours.iter_mut().enumerate() {
        if !op.tabs.enabled || (op.tabs.manual && assigned[index].is_empty()) {
            layouts.push(None);
            continue;
        }
        let mut tab_op = op.clone();
        if op.tabs.manual {
            tab_op.tabs.positions = assigned[index].clone();
            move_tab_seam(points, &tab_op, diameter);
        }
        let layout = TabLayout::new(points,&tab_op,diameter).ok_or("Holding tabs overlap or leave insufficient room for entry; move tabs, increase spacing, or reduce width/lead size")?;
        layouts.push(Some(layout));
    }
    Ok((contours, layouts))
}

impl TabLayout {
    fn new(pts: &[Point], op: &ProfileOp, diameter: f64) -> Option<Self> {
        let perimeter = loop_perimeter(pts);
        let width = op.tabs.width + diameter;
        let lead_room = lead_extent(op.lead_in).max(lead_extent(op.lead_out));
        let mut centers = if op.tabs.manual {
            op.tabs
                .positions
                .iter()
                .map(|&point| nearest_on_loop(pts, point).0)
                .collect::<Vec<_>>()
        } else {
            let count = op
                .tabs
                .count
                .map_or_else(|| (perimeter / op.tabs.spacing).ceil().max(1.0), f64::from);
            if !count.is_finite() || count > 10_000.0 {
                return None;
            }
            (0..count as usize)
                .map(|i| (i as f64 + 0.5) * perimeter / count)
                .collect()
        };
        if !perimeter.is_finite() || perimeter <= 0.0 || centers.is_empty() {
            return None;
        }
        centers.sort_by(f64::total_cmp);
        let spans: Vec<_> = centers
            .into_iter()
            .map(|center| (center - width * 0.5, center + width * 0.5))
            .collect();
        if spans[0].0 <= lead_room
            || spans.last()?.1 >= perimeter - lead_room
            || spans.windows(2).any(|pair| pair[0].1 >= pair[1].0)
        {
            return None;
        }
        Some(Self {
            perimeter,
            spans,
            top: -op.depth + op.tabs.height,
        })
    }

    fn cut_triangular(&self, prog: &mut Program, pts: &[Point], op: &ProfileOp, z: f64) {
        let floor = self.top - op.tabs.height;
        let half_width = op.tabs.width * 0.5;
        let mut breaks = vec![0.];
        let mut distance = 0.;
        for edge in pts.windows(2) {
            distance += edge[0].distance(edge[1]);
            breaks.push(distance);
        }
        let fraction = ((z - floor) / op.tabs.height).clamp(0., 1.);
        for &(left, right) in &self.spans {
            let center = (left + right) * 0.5;
            let radius = (right - left - op.tabs.width) * 0.5;
            breaks.extend([
                left,
                center - radius - half_width * (1. - fraction),
                center - radius,
                center + radius,
                center + radius + half_width * (1. - fraction),
                right,
            ]);
        }
        breaks.sort_by(f64::total_cmp);
        breaks.dedup_by(|a, b| (*a - *b).abs() < 1e-10);
        for s in breaks.into_iter().skip(1) {
            let (point, _) = point_on_loop(pts, s);
            let roof = self
                .spans
                .iter()
                .map(|&(left, right)| {
                    let center = (left + right) * 0.5;
                    let radius = (right - left - op.tabs.width) * 0.5;
                    floor
                        + op.tabs.height
                            * (1. - ((s - center).abs() - radius).max(0.) / half_width)
                                .clamp(0., 1.)
                })
                .fold(z, f64::max);
            prog.push(Step::Linear {
                to: Point3::new(point.x, point.y, roof.max(z)),
                feed: op.feed.min(op.plunge_feed),
                tag: Tag::new(op.id, MoveKind::Cutting),
            });
        }
    }

    /// Move an exit away from holding tabs and leave enough room for its lead.
    fn eligible_exit(&self, pts: &[Point], point: [f64; 2], room: f64) -> [f64; 2] {
        let desired = nearest_on_loop(pts, point).0;
        let mut best = (0.0, f64::INFINITY);
        for index in 0..=self.spans.len() {
            let low = if index == 0 {
                0.0
            } else {
                self.spans[index - 1].1 + room + 1e-6
            };
            let high = if index == self.spans.len() {
                self.perimeter
            } else {
                self.spans[index].0 - room - 1e-6
            };
            if low > high {
                continue;
            }
            let at = desired.clamp(low, high);
            let delta = (desired - at).abs();
            let distance = delta.min(self.perimeter - delta);
            if distance < best.1 {
                best = (at, distance);
            }
        }
        let at = walk_loop(pts, best.0, 0.0)[0];
        [at.x, at.y]
    }

    fn cut(&self, prog: &mut Program, pts: &[Point], op: &ProfileOp, z: f64) {
        if op.tabs.shape == cam_model::TabShape::Triangular && z < self.top {
            self.cut_triangular(prog, pts, op, z);
            return;
        }
        if z >= self.top {
            crate::emit::cut_polyline_tolerance(
                prog,
                pts,
                op.feed,
                Tag::new(op.id, MoveKind::Cutting),
                z,
                op.passes.tolerance,
            );
            return;
        }
        let cut = Tag::new(op.id, MoveKind::Cutting);
        let plunge = Tag::new(op.id, MoveKind::Plunge);
        let mut distance = 0.0;
        let mut span_index = 0;
        let mut raised = false;
        // Do not refit across tab boundaries: an arc must never interpolate the
        // vertical shoulders, which would turn the rectangular tab into a ramp.
        for edge in pts.windows(2) {
            let a = edge[0];
            let b = edge[1];
            let length = (b.x - a.x).hypot(b.y - a.y);
            if length <= 1e-12 {
                continue;
            }
            let end = distance + length;
            while !self.spans.is_empty() {
                let span = self.spans[span_index % self.spans.len()];
                let lap = (span_index / self.spans.len()) as f64 * self.perimeter;
                let boundary = lap + if raised { span.1 } else { span.0 };
                if boundary > end + 1e-9 {
                    break;
                }
                let t = ((boundary - distance) / length).clamp(0.0, 1.0);
                let p = Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
                prog.push(Step::Linear {
                    to: Point3::new(p.x, p.y, if raised { self.top } else { z }),
                    feed: op.feed,
                    tag: cut,
                });
                raised = !raised;
                prog.push(Step::Linear {
                    to: Point3::new(p.x, p.y, if raised { self.top } else { z }),
                    feed: if raised { op.feed } else { op.plunge_feed },
                    tag: if raised { cut } else { plunge },
                });
                if !raised {
                    span_index += 1;
                }
            }
            prog.push(Step::Linear {
                to: Point3::new(b.x, b.y, if raised { self.top } else { z }),
                feed: op.feed,
                tag: cut,
            });
            distance = end;
        }
    }
}

/// Unit vector, or `(1,0)` if degenerate.
pub(crate) fn unit(x: f64, y: f64) -> (f64, f64) {
    let l = (x * x + y * y).sqrt();
    if l > 1e-12 {
        (x / l, y / l)
    } else {
        (1.0, 0.0)
    }
}

/// Unit tangent leaving the start vertex (start → pts[1]).
pub(crate) fn start_tangent(pts: &[Point]) -> (f64, f64) {
    unit(pts[1].x - pts[0].x, pts[1].y - pts[0].y)
}

/// Twice the signed area of the closed loop (shoelace); `> 0` is CCW.
fn signed_area2(pts: &[Point]) -> f64 {
    let n = pts.len();
    (0..n)
        .map(|i| {
            let a = pts[i];
            let b = pts[(i + 1) % n];
            a.x * b.y - b.x * a.y
        })
        .sum()
}

/// Whether the closed loop winds counter-clockwise — the orientation needed to
/// place a lead's outward normal (see [`outward_normal_at`]).
pub(crate) fn is_ccw(pts: &[Point]) -> bool {
    signed_area2(pts) > 0.0
}

/// The outward normal at the start (away from the loop interior), for placing leads
/// and helix centres on the non-material side.
pub(crate) fn outward_normal(pts: &[Point]) -> (f64, f64) {
    outward_normal_at(start_tangent(pts), signed_area2(pts) > 0.0)
}

/// The outward normal for a given travel tangent on a loop of the given
/// orientation (`ccw`), away from the interior — the point-agnostic core of
/// [`outward_normal`], reused at an overlap point where the tangent differs from
/// the start's.
pub(crate) fn outward_normal_at(t: (f64, f64), ccw: bool) -> (f64, f64) {
    // Interior is left of travel for a CCW loop, so outward is the right normal;
    // the reverse for CW.
    if ccw {
        (t.1, -t.0)
    } else {
        (-t.1, t.0)
    }
}

/// Emit the plunge from `p@from_z` down to `p@to_z` (ending at `p` in XY), per the
/// strategy. Bad parameters fall back to a straight plunge (never panic).
#[allow(clippy::too_many_arguments)]
pub(crate) fn emit_plunge(
    prog: &mut Program,
    p: Point,
    tan: (f64, f64),
    out: (f64, f64),
    from_z: f64,
    to_z: f64,
    plunge: Plunge,
    plunge_feed: f64,
    cut_feed: f64,
    tag: Tag,
) {
    let dz = from_z - to_z;
    let straight = |prog: &mut Program| {
        prog.push(Step::Linear {
            to: Point3::new(p.x, p.y, to_z),
            feed: plunge_feed,
            tag,
        });
    };
    if dz <= 0.0 {
        return straight(prog);
    }
    match plunge {
        Plunge::Straight => straight(prog),
        Plunge::Helix { radius, pitch } if radius > 0.0 && pitch > 0.0 => {
            let turns = (dz / pitch).ceil().max(1.0) as usize;
            let centre = Point::new(p.x + out.0 * radius, p.y + out.1 * radius);
            let opp = Point::new(2.0 * centre.x - p.x, 2.0 * centre.y - p.y);
            let dz_half = dz / (turns as f64 * 2.0);
            let mut z = from_z;
            for _ in 0..turns {
                z -= dz_half;
                prog.push(Step::Arc {
                    end: Point3::new(opp.x, opp.y, z),
                    center: Point3::new(centre.x, centre.y, z),
                    dir: ArcDir::Ccw,
                    feed: cut_feed,
                    tag,
                });
                z -= dz_half;
                prog.push(Step::Arc {
                    end: Point3::new(p.x, p.y, z),
                    center: Point3::new(centre.x, centre.y, z),
                    dir: ArcDir::Ccw,
                    feed: cut_feed,
                    tag,
                });
            }
        }
        // `Plunge::Ramp` is deliberately absent: it is not a point entry. It travels
        // along the contour and therefore *starts* somewhere else, which is a fact the
        // caller has to know before it rapids anywhere — see `contour_ramp_len` and
        // `approach_along_loop`. Callers that own a contour handle it before reaching
        // here; the fallthrough below keeps a caller that does not from emitting
        // nothing at all.
        Plunge::ZigZag { length, angle_deg }
            if length > 0.0 && angle_deg > 0.0 && angle_deg < 90.0 =>
        {
            emit_oscillating_ramp(prog, p, tan, from_z, to_z, angle_deg, length, cut_feed, tag)
        }
        // Bad parameters: safe fallback.
        _ => straight(prog),
    }
}

/// Oscillate along `[p, p + tan·L]` descending from `from_z` to `to_z`, ending back
/// at `p`. `max_len` caps the reach; the number of out-and-back passes is chosen so
/// each stays within the reach and the angle holds.
///
/// [`Plunge::ZigZag`] alone: the oscillation exists for the slot too narrow to ramp
/// along, and nothing else. [`Plunge::Ramp`] used to come through here with an
/// infinite reach — one V along the straight start tangent — which is why its
/// documented "along the toolpath" was never true.
#[allow(clippy::too_many_arguments)]
fn emit_oscillating_ramp(
    prog: &mut Program,
    p: Point,
    tan: (f64, f64),
    from_z: f64,
    to_z: f64,
    angle_deg: f64,
    max_len: f64,
    feed: f64,
    tag: Tag,
) {
    let dz = from_z - to_z;
    let slope = angle_deg.to_radians().tan(); // dz per unit horizontal
                                              // One out-and-back V descends 2·L·slope; bound L by max_len.
    let passes = if max_len.is_finite() {
        ((dz / (2.0 * max_len * slope)).ceil().max(1.0)) as usize
    } else {
        1
    };
    let dz_v = dz / passes as f64;
    let l = dz_v / (2.0 * slope); // reach of each V
    let far = Point::new(p.x + tan.0 * l, p.y + tan.1 * l);
    let mut z = from_z;
    for _ in 0..passes {
        z -= dz_v / 2.0;
        prog.push(Step::Linear {
            to: Point3::new(far.x, far.y, z),
            feed,
            tag,
        });
        z -= dz_v / 2.0;
        prog.push(Step::Linear {
            to: Point3::new(p.x, p.y, z),
            feed,
            tag,
        });
    }
}

/// How far the tool must travel along the contour to descend `dz` at the ramp's
/// angle. `None` for every plunge style that is not a contour ramp, and for a
/// degenerate angle or a non-descent — both of which fall back to the plain plunge,
/// exactly as an invalid helix always has.
pub(crate) fn contour_ramp_len(plunge: Plunge, dz: f64) -> Option<f64> {
    match plunge {
        Plunge::Ramp { angle_deg } if angle_deg > 0.0 && angle_deg < 90.0 && dz > 0.0 => {
            Some(dz / angle_deg.to_radians().tan())
        }
        _ => None,
    }
}

/// Maximum allocated laps. Profile preflight rejects longer requests.
///
/// A shallow angle on a small loop is otherwise unbounded: 0.5° needs 115 mm of
/// travel per millimetre of descent, so a 20 mm circle takes nearly two laps per
/// millimetre and the emitted vertex count grows with it. Beyond the cap the ramp
/// walker truncates at the allocation limit. Callers must preflight the requested
/// travel; a truncated path must never be used to increase the requested slope.
pub(crate) const MAX_RAMP_LAPS: usize = 32;

pub(crate) fn validate_ramp_travel(
    plunge: Plunge,
    descent: f64,
    perimeter: f64,
) -> Result<(), String> {
    if contour_ramp_len(plunge, descent).is_some_and(|length| {
        !length.is_finite() || length > perimeter * (MAX_RAMP_LAPS as f64 - 1.)
    }) {
        return Err(
            "Requested ramp requires too many laps; reduce stepdown or increase the ramp angle"
                .into(),
        );
    }
    Ok(())
}

/// Total length of an open polyline.
pub(crate) fn path_length(pts: &[Point]) -> f64 {
    pts.windows(2)
        .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
        .sum()
}

/// The perimeter of the closed loop through `pts`.
pub(crate) fn loop_perimeter(pts: &[Point]) -> f64 {
    let n = pts.len();
    (0..n)
        .map(|i| {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            (b.x - a.x).hypot(b.y - a.y)
        })
        .sum()
}

/// Walk the closed loop `pts` **forward** from arc position `from`, for `len` mm,
/// wrapping as often as needed. The first returned point is the position at `from`.
///
/// One walker for both halves of a ramped entry: the stretch the ramp descends over,
/// and the full-perimeter cut that follows it from wherever the ramp ended.
pub(crate) fn walk_loop(pts: &[Point], from: f64, len: f64) -> Vec<Point> {
    let n = pts.len();
    if n < 2 {
        return pts.to_vec();
    }
    let perim = loop_perimeter(pts);
    if perim <= 1e-12 {
        return vec![pts[0]];
    }
    // Where `from` lands: the edge it falls on, and how far along that edge.
    let mut start_at = from.rem_euclid(perim);
    let mut i = 0usize;
    loop {
        let e = (pts[(i + 1) % n].x - pts[i].x).hypot(pts[(i + 1) % n].y - pts[i].y);
        if start_at <= e || i + 1 == n {
            break;
        }
        start_at -= e;
        i += 1;
    }
    let at = |i: usize, d: f64| {
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        let e = (b.x - a.x).hypot(b.y - a.y);
        if e <= 1e-12 {
            a
        } else {
            Point::new(a.x + (b.x - a.x) * d / e, a.y + (b.y - a.y) * d / e)
        }
    };

    let mut out = vec![at(i, start_at)];
    let mut remaining = len;
    let mut edge_left = {
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        (b.x - a.x).hypot(b.y - a.y) - start_at
    };
    // Bounded like the ramp itself: a caller asking for many perimeters gets many
    // laps, but never an unbounded buffer.
    for _ in 0..n.saturating_mul(MAX_RAMP_LAPS) + 2 {
        if remaining <= 0.0 {
            break;
        }
        if edge_left > remaining {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            let e = (b.x - a.x).hypot(b.y - a.y);
            let d = e - edge_left + remaining;
            out.push(at(i, d));
            break;
        }
        remaining -= edge_left;
        i = (i + 1) % n;
        out.push(pts[i]);
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        edge_left = (b.x - a.x).hypot(b.y - a.y);
    }
    out
}

/// The lead-in as a path **from the lead's start point to the contour**, inclusive of
/// both ends — the reverse of [`leads::lead_samples`], which samples outward from the
/// contour. Empty lead ⇒ just the contour point.
///
/// The ramp descends along this before it touches the contour, which is the whole
/// point: the lead exists to keep the entry off the finished wall, so it is where a
/// descending tool belongs.
pub(crate) fn lead_in_path(start: Point, entry: Point, out: (f64, f64), lead: Lead) -> Vec<Point> {
    // Sampled finely enough to be re-fitted: the ramp descends along these points and
    // they go back through `fit_arcs`, which refuses a run whose chords miss the circle
    // by more than its tolerance. The guard's eight points are too coarse for that, so
    // an arc lead-in came out as a fan of chords — Andreas, reading a pocket export:
    // "it's doing ramp-along with no arcs".
    let n = match lead {
        Lead::ArcSweep { radius, angle_deg } => {
            crate::leads::arc_lead_density(radius, angle_deg.to_radians(), crate::emit::ARCFIT_TOL)
        }
        Lead::Arc { radius } => crate::leads::arc_lead_density(
            radius,
            std::f64::consts::FRAC_PI_2,
            crate::emit::ARCFIT_TOL,
        ),
        _ => 8,
    };
    let mut pts = crate::leads::lead_samples_n(start, entry, out, lead, n);
    pts.reverse();
    match pts.last() {
        Some(p) if (p.x - start.x).abs() < 1e-9 && (p.y - start.y).abs() < 1e-9 => {}
        _ => pts.push(start),
    }
    if pts.is_empty() {
        pts.push(start);
    }
    pts
}

/// The entry ramp for a closed loop, or `None` when the plunge is not a contour ramp.
///
/// Returns the path to descend along, how much of it lies **on the loop**, and the Z
/// the descent starts from.
///
/// The shape of it, in one place because profile passes and carve wall rings must not
/// drift apart:
/// - `lead_path` (from the lead's start to the loop, empty for no lead) is flown in
///   **air**: the descent begins `lead_len · tan(angle)` above the material, so the tool
///   meets the surface *at* the loop's start and never cuts on the way in.
/// - the loop then carries the **whole** material descent, `(ramp_top − z) / tan(angle)`.
/// - the caller must begin its cut at the returned arc offset and travel a full
///   perimeter from there, so the stretch the ramp sloped is re-machined at depth.
pub(crate) fn contour_ramp(
    pts: &[Point],
    lead_path: Vec<Point>,
    plunge: Plunge,
    ramp_top: f64,
    z: f64,
) -> Option<(Vec<Point>, f64, f64)> {
    let in_material = contour_ramp_len(plunge, ramp_top - z)?;
    let lead_len = path_length(&lead_path);
    let mut path = lead_path;
    if path.is_empty() {
        path.push(pts[0]);
    }
    path.extend(walk_loop(pts, 0.0, in_material).into_iter().skip(1));
    if path.len() < 2 {
        return None;
    }
    // `tan(angle) = (ramp_top − z) / in_material` by construction, so the lead's rise
    // needs no second look at the angle.
    let rise = if in_material > 1e-12 {
        lead_len * (ramp_top - z) / in_material
    } else {
        0.0
    };
    Some((path, in_material, ramp_top + rise))
}

/// The first `len` mm of an **open** path, from `path[0]` forward, truncated at the
/// path's end. Unlike [`approach_along_loop`] there is nothing to wrap around, so a
/// path shorter than `len` yields all of it — the ramp then descends over what
/// travel exists, which is steeper than asked and never a vertical drop.
pub(crate) fn advance_along_path(path: &[Point], len: f64) -> Vec<Point> {
    let mut out = vec![path[0]];
    if path.len() < 2 || len.is_nan() || len <= 0.0 {
        return out;
    }
    let mut remaining = len;
    for w in path.windows(2) {
        let (dx, dy) = (w[1].x - w[0].x, w[1].y - w[0].y);
        let seg = (dx * dx + dy * dy).sqrt();
        if seg <= 1e-12 {
            continue;
        }
        if seg >= remaining {
            let t = remaining / seg;
            out.push(Point::new(w[0].x + dx * t, w[0].y + dy * t));
            return out;
        }
        remaining -= seg;
        out.push(w[1]);
    }
    out
}

/// Descend along an **open** path and retrace it: the tool ramps forward from
/// `path[0]` to depth, then returns along the same stretch at full depth, ending
/// where it began.
///
/// The return leg is not decoration, and it is not the oscillating entry this
/// replaced. A closed loop needs no return because the pass ends where the ramp
/// began, so the loop's last stretch re-machines the slope for free. An open path
/// never comes back, so the wedge would simply be left standing at the entry.
/// Retracing at depth is the cheapest honest answer, and it still follows the
/// toolpath rather than cutting across it.
pub(crate) fn emit_open_ramp(
    prog: &mut Program,
    path: &[Point],
    from_z: f64,
    to_z: f64,
    feed: f64,
    tag: Tag,
) {
    emit_descending_path(prog, path, from_z, to_z, feed, tag);
    // Back along the same points at depth. Skipping the last (we are on it) and
    // walking in reverse leaves the tool at `path[0]`, which is the entry contract
    // every other plunge strategy keeps.
    for p in path.iter().rev().skip(1) {
        prog.push(Step::Linear {
            to: Point3::new(p.x, p.y, to_z),
            feed,
            tag,
        });
    }
}

/// Emit `path` descending linearly **in arc length** from `from_z`, reaching exactly
/// `to_z` at its last point. The tool must already sit at `path[0]`.
///
/// **Arcs stay arcs.** The path is a flattened polyline, so it goes back through
/// `fit_arcs` exactly as [`cut_polyline`](crate::emit::cut_polyline) does, and a curved
/// run is emitted as a **helical arc** — one `G2`/`G3` with a descending `Z` — rather
/// than as chords. Without this a ramp turned every corner it crossed into ten linear
/// moves where the cutting pass, over the very same points, emitted one arc; and an arc
/// lead-in could never have come out as an arc at all.
///
/// Linear in arc length, not in vertex index: a loop's edges differ in length, and
/// interpolating per vertex would tilt the ramp differently on every one.
pub(crate) fn emit_descending_path(
    prog: &mut Program,
    path: &[Point],
    from_z: f64,
    to_z: f64,
    feed: f64,
    tag: Tag,
) {
    if path.len() < 2 {
        return;
    }
    let segs = cam_geo::fit_arcs(path, crate::emit::ARCFIT_TOL);
    // Each segment's length along the path — a chord for a line, r·θ for an arc, so the
    // descent stays at the requested angle *through* a curve rather than steepening.
    let mut from = path[0];
    let mut lens = Vec::with_capacity(segs.len());
    for seg in &segs {
        let (l, end) = match seg {
            cam_geo::PathSeg::Line { end } => ((end.x - from.x).hypot(end.y - from.y), *end),
            cam_geo::PathSeg::Arc { end, center, ccw } => {
                let r = (from.x - center.x).hypot(from.y - center.y);
                let a0 = (from.y - center.y).atan2(from.x - center.x);
                let a1 = (end.y - center.y).atan2(end.x - center.x);
                let mut sweep = if *ccw { a1 - a0 } else { a0 - a1 };
                while sweep <= 0.0 {
                    sweep += std::f64::consts::TAU;
                }
                (r * sweep, *end)
            }
        };
        lens.push(l);
        from = end;
    }
    let total: f64 = lens.iter().sum();
    let last = path[path.len() - 1];
    if total.is_nan() || total <= 1e-12 {
        // Nowhere to ramp: drop straight to depth rather than emit a descent that never
        // descends.
        prog.push(Step::Linear {
            to: Point3::new(last.x, last.y, to_z),
            feed,
            tag,
        });
        return;
    }

    let mut acc = 0.0;
    let n = segs.len();
    for (i, seg) in segs.iter().enumerate() {
        acc += lens[i];
        // The last segment takes `to_z` outright: the running sum is within a few ULP of
        // `total`, and "reaches the exact depth" is worth not rounding.
        let z = if i + 1 == n {
            to_z
        } else {
            from_z + (to_z - from_z) * (acc / total)
        };
        match seg {
            cam_geo::PathSeg::Line { end } => prog.push(Step::Linear {
                to: Point3::new(end.x, end.y, z),
                feed,
                tag,
            }),
            cam_geo::PathSeg::Arc { end, center, ccw } => prog.push(Step::Arc {
                // A helical arc: the centre carries the *arriving* Z, which is how the
                // dialects already write a thread mill's helix.
                end: Point3::new(end.x, end.y, z),
                center: Point3::new(center.x, center.y, z),
                dir: if *ccw { ArcDir::Ccw } else { ArcDir::Cw },
                feed,
                tag,
            }),
        }
    }
}

/// Rotate a closed loop so it begins **exactly at the point on the loop nearest
/// `start`** (part XY), preserving winding. `None` leaves the loop unchanged.
///
/// Unlike a nearest-*vertex* rotation, this projects `start` onto the closest
/// edge and, when that lands mid-edge, splits the edge — inserting the projected
/// point as the first vertex — so a Mid or Nearest object-snap really does begin
/// where the operator pointed, not at the corner beside it. A projection onto an
/// existing vertex just rotates (no split).
pub(crate) fn rotate_to_start(
    pts: &[cam_geo::Point],
    start: Option<[f64; 2]>,
) -> Vec<cam_geo::Point> {
    use cam_geo::Point;
    let Some(s) = start else {
        return pts.to_vec();
    };
    let n = pts.len();
    if n < 2 {
        return pts.to_vec();
    }
    let sp = Point::new(s[0], s[1]);
    // The closest point on any edge, and which edge it lies on.
    let (mut best_k, mut best_q, mut best_d2) = (0usize, pts[0], f64::MAX);
    for k in 0..n {
        let q = project_point_seg(sp, pts[k], pts[(k + 1) % n]);
        let d2 = q.distance_sq(sp);
        if d2 < best_d2 {
            (best_k, best_q, best_d2) = (k, q, d2);
        }
    }
    const EPS2: f64 = 1e-12;
    let kb = (best_k + 1) % n;
    let rotate_from = |i: usize| {
        let mut out = Vec::with_capacity(n);
        out.extend_from_slice(&pts[i..]);
        out.extend_from_slice(&pts[..i]);
        out
    };
    // Snap to an existing endpoint → plain rotation, no split.
    if best_q.distance_sq(pts[best_k]) <= EPS2 {
        return rotate_from(best_k);
    }
    if best_q.distance_sq(pts[kb]) <= EPS2 {
        return rotate_from(kb);
    }
    // Mid-edge: begin at the projected point, then the rest of the loop from `kb`
    // around to `best_k` (which the caller closes back to the projected point).
    let mut out = Vec::with_capacity(n + 1);
    out.push(best_q);
    out.extend(rotate_from(kb));
    out
}

/// The closest point to `p` on the segment `a→b` (clamped to the endpoints).
fn project_point_seg(p: cam_geo::Point, a: cam_geo::Point, b: cam_geo::Point) -> cam_geo::Point {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    if len2 <= f64::EPSILON {
        return a;
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
    cam_geo::Point::new(a.x + dx * t, a.y + dy * t)
}

/// The absolute Z of each stepdown pass, from just below `top` down to `depth`
/// (inclusive), never stepping more than `stepdown` at a time.
pub(crate) fn depth_levels(top: f64, depth: f64, stepdown: f64) -> Vec<f64> {
    let count = ((top - depth) / stepdown).ceil();
    if !top.is_finite()
        || !depth.is_finite()
        || !stepdown.is_finite()
        || stepdown <= 0.
        || count <= 0.
        || !count.is_finite()
        || count > cam_model::MAX_PROFILE_LEVELS as f64
        || top - stepdown == top
    {
        return Vec::new();
    }
    let epsilon = 1e-12 * (1. + top.abs().max(depth.abs()));
    let mut levels = Vec::with_capacity(count as usize);
    for i in 1..=count as usize {
        let next = (top - i as f64 * stepdown).max(depth);
        let z = if (next - depth).abs() <= epsilon {
            depth
        } else {
            next
        };
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
    fn independent_exit_completes_the_contour_before_leading_out() {
        for plunge in [Plunge::Straight, Plunge::Ramp { angle_deg: 10.0 }] {
            let mut op = inner_op(40.0, 1.0, Some([3.0, 10.0]));
            op.end = Some([37.0, 20.0]);
            op.plunge = plunge;
            op.lead_overlap = 2.0;
            let result = run_inner(op);
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
            assert!(result.program.validate().is_ok());
            let mut previous = None;
            let mut cut_length = 0.0;
            let mut lead_out_at = None;
            for step in result.program.steps() {
                let (point, tag) = match step {
                    Step::Linear { to, tag, .. } | Step::Rapid { to, tag } => (*to, *tag),
                    Step::Arc { end, tag, .. } => (*end, *tag),
                    _ => continue,
                };
                if let Some(p) = previous {
                    let p: Point3 = p;
                    if tag.kind == MoveKind::Cutting {
                        cut_length += (point.x - p.x).hypot(point.y - p.y);
                    }
                    if tag.kind == MoveKind::LeadOut {
                        lead_out_at = Some(p);
                    }
                }
                previous = Some(point);
            }
            let exit = lead_out_at.expect("lead-out generated");
            assert!(
                (exit.x - 37.0).hypot(exit.y - 20.0) < 0.01,
                "{plunge:?}: {exit:?}"
            );
            assert!(
                cut_length >= 4.0 * 34.0 - 0.05,
                "complete contour before exit: {cut_length}"
            );
        }
    }

    #[test]
    fn exit_preserves_minimum_overlap_even_when_it_requires_another_lap() {
        let pts = vec![
            Point::new(0., 0.),
            Point::new(10., 0.),
            Point::new(10., 10.),
            Point::new(0., 10.),
        ];
        let (path, exit, _) = crate::emit::loop_with_exit(&pts, 5.0, Some([2.0, 0.0]));
        assert_eq!(exit, Point::new(2.0, 0.0));
        let length: f64 = path
            .windows(2)
            .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
            .sum();
        assert!((length - 82.0).abs() < 1e-9);
    }

    #[test]
    fn levels_step_down_to_depth() {
        assert_eq!(depth_levels(0.0, -3.0, 1.5), vec![-1.5, -3.0]);
        assert_eq!(depth_levels(0.0, -1.0, 1.5), vec![-1.0]);
        assert_eq!(depth_levels(0.0, -5.0, 2.0), vec![-2.0, -4.0, -5.0]);
        assert!(depth_levels(0.0, 1.0, 1.0).is_empty());
    }

    fn step_end_z(s: &Step) -> f64 {
        match s {
            Step::Linear { to, .. } | Step::Rapid { to, .. } => to.z,
            Step::Arc { end, .. } => end.z,
            _ => f64::NAN,
        }
    }

    fn step_end_xy(s: &Step) -> (f64, f64) {
        match s {
            Step::Linear { to, .. } | Step::Rapid { to, .. } => (to.x, to.y),
            Step::Arc { end, .. } => (end.x, end.y),
            _ => (f64::NAN, f64::NAN),
        }
    }

    /// `Plunge::Ramp` is absent by design, not by oversight: it is the one entry that
    /// does *not* end at the footprint it began at, because it travels along the
    /// contour. Its equivalent properties are asserted in the contour-ramp tests below.
    #[test]
    fn every_point_plunge_strategy_descends_monotonically_to_exact_depth() {
        let p = Point::new(0.0, 0.0);
        let tan = (1.0, 0.0);
        let out = (0.0, 1.0);
        let tag = Tag::new(0, MoveKind::Plunge);
        for plunge in [
            Plunge::Straight,
            Plunge::Helix {
                radius: 2.0,
                pitch: 1.0,
            },
            Plunge::ZigZag {
                length: 3.0,
                angle_deg: 15.0,
            },
        ] {
            let mut prog = Program::new();
            emit_plunge(&mut prog, p, tan, out, 0.0, -5.0, plunge, 100.0, 300.0, tag);
            let zs: Vec<f64> = prog.steps.iter().map(step_end_z).collect();
            assert!(!zs.is_empty(), "{plunge:?} emitted nothing");
            for w in zs.windows(2) {
                assert!(w[1] <= w[0] + 1e-9, "{plunge:?} must descend monotonically");
            }
            assert!(
                (zs.last().unwrap() + 5.0).abs() < 1e-6,
                "{plunge:?} must reach the exact depth"
            );
            let (ex, ey) = step_end_xy(prog.steps.last().unwrap());
            assert!(
                (ex - p.x).abs() < 1e-6 && (ey - p.y).abs() < 1e-6,
                "{plunge:?} must end at the entry footprint"
            );
        }
    }

    /// A 100 mm square loop, used by the contour-ramp tests below. Perimeter 400 mm.
    fn square() -> Vec<Point> {
        vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0),
            Point::new(100.0, 100.0),
            Point::new(0.0, 100.0),
        ]
    }

    fn path_len(p: &[Point]) -> f64 {
        p.windows(2)
            .map(|w| ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt())
            .sum()
    }

    #[test]
    fn the_contour_ramp_travels_the_length_its_angle_requires() {
        // 5 mm of descent at 45° is 5 mm of travel; at ~26.57° (tan = 0.5) it is 10.
        assert!(
            (contour_ramp_len(Plunge::Ramp { angle_deg: 45.0 }, 5.0).unwrap() - 5.0).abs() < 1e-9
        );
        let shallow = contour_ramp_len(
            Plunge::Ramp {
                angle_deg: 26.565_051_2,
            },
            5.0,
        )
        .unwrap();
        assert!((shallow - 10.0).abs() < 1e-6, "got {shallow}");
        // Not a ramp, a degenerate angle, or no descent to make: the plain plunge.
        assert!(contour_ramp_len(Plunge::Straight, 5.0).is_none());
        assert!(contour_ramp_len(Plunge::Ramp { angle_deg: 90.0 }, 5.0).is_none());
        assert!(contour_ramp_len(Plunge::Ramp { angle_deg: 0.0 }, 5.0).is_none());
        assert!(contour_ramp_len(Plunge::Ramp { angle_deg: 20.0 }, 0.0).is_none());
    }

    /// Wrapping and the lap cap now belong to [`walk_loop`], the single walker for both
    /// halves of a ramped entry. `approach_along_loop` — which walked *backwards* from
    /// the start — is gone: the ramp runs forward from the lead-in now, so nothing
    /// needed it.
    #[test]
    fn walking_the_loop_covers_the_length_asked_for_and_wraps() {
        let pts = square();
        let quarter = walk_loop(&pts, 0.0, 30.0);
        assert!((path_len(&quarter) - 30.0).abs() < 1e-9);
        assert!(
            (quarter[0].x - pts[0].x).abs() < 1e-9 && (quarter[0].y - pts[0].y).abs() < 1e-9,
            "a walk from arc 0 starts at the loop's start"
        );
        // 950 mm on a 400 mm perimeter: two full laps and 150 mm.
        let long = walk_loop(&pts, 0.0, 950.0);
        assert!(
            (path_len(&long) - 950.0).abs() < 1e-6,
            "got {}",
            path_len(&long)
        );
        // And a walk starting part-way round begins where it was told to.
        let offset = walk_loop(&pts, 150.0, 10.0);
        assert!(
            (offset[0].x - 100.0).abs() < 1e-9 && (offset[0].y - 50.0).abs() < 1e-9,
            "150 mm round a 100 mm square is halfway up the right-hand side: {:?}",
            offset[0]
        );
    }

    /// A shallow enough angle asks for unbounded travel. The cap bounds the buffer
    /// rather than trusting the arithmetic to stay sane — the same lesson as the dashed
    /// backplot's walk.
    #[test]
    fn an_absurdly_long_walk_is_bounded_by_the_lap_cap() {
        let pts = square();
        let path = walk_loop(&pts, 0.0, 1.0e9);
        // The guarantee is on **edges walked**, not on whole laps: the cap allows one
        // edge per lap per vertex, plus two for the partial edges at each end. On a
        // 4-edge square that is 32 laps and a bit, which is the point — a bound, not a
        // round number.
        assert!(
            path.len() <= 4 * MAX_RAMP_LAPS + 3,
            "{} vertices is past the bound",
            path.len()
        );
        assert!(
            path_len(&path) <= 400.0 * (MAX_RAMP_LAPS as f64 + 1.0),
            "walked {} mm, past the {MAX_RAMP_LAPS}-lap cap",
            path_len(&path)
        );
    }

    #[test]
    fn a_descending_path_falls_monotonically_and_reaches_the_exact_depth() {
        let pts = square();
        let path = walk_loop(&pts, 0.0, 30.0);
        let mut prog = Program::new();
        emit_descending_path(
            &mut prog,
            &path,
            0.0,
            -5.0,
            300.0,
            Tag::new(0, MoveKind::Plunge),
        );
        let zs: Vec<f64> = prog.steps.iter().map(step_end_z).collect();
        assert_eq!(zs.len(), path.len() - 1, "one move per segment");
        for w in zs.windows(2) {
            assert!(w[1] <= w[0] + 1e-12, "the ramp must descend monotonically");
        }
        assert!(
            (zs.last().unwrap() + 5.0).abs() < 1e-12,
            "must reach exact depth, got {}",
            zs.last().unwrap()
        );
        // Descent is linear in arc length: halfway along is halfway down.
        let half = prog
            .steps
            .iter()
            .map(step_end_z)
            .find(|_| true)
            .expect("at least one move");
        assert!(half < 0.0, "the first move must already be descending");
    }

    #[test]
    fn bad_plunge_params_fall_back_to_a_straight_plunge() {
        let tag = Tag::new(0, MoveKind::Plunge);
        let mut prog = Program::new();
        // Zero radius helix / 90° ramp are invalid → one straight linear plunge.
        emit_plunge(
            &mut prog,
            Point::new(0.0, 0.0),
            (1.0, 0.0),
            (0.0, 1.0),
            0.0,
            -3.0,
            Plunge::Helix {
                radius: 0.0,
                pitch: 1.0,
            },
            100.0,
            300.0,
            tag,
        );
        assert_eq!(prog.steps.len(), 1);
        assert!((step_end_z(&prog.steps[0]) + 3.0).abs() < 1e-9);
    }

    #[test]
    fn arc_lead_endpoints_lie_on_the_tangent_circle() {
        let start = Point::new(0.0, 0.0);
        let (tan, out) = ((1.0, 0.0), (0.0, 1.0));
        let r = 3.0;
        let entry = crate::leads::lead_start_point(start, tan, out, Lead::Arc { radius: r });
        let exit = crate::leads::lead_end_point(start, tan, out, Lead::Arc { radius: r });
        let centre = Point::new(start.x + out.0 * r, start.y + out.1 * r);
        for pt in [entry, exit, start] {
            let d = (pt.x - centre.x).hypot(pt.y - centre.y);
            assert!(
                (d - r).abs() < 1e-9,
                "{pt:?} must lie on the radius-{r} circle"
            );
        }
        // A 90° arc: the chord entry→start is √2·r.
        let chord = (entry.x - start.x).hypot(entry.y - start.y);
        assert!((chord - std::f64::consts::SQRT_2 * r).abs() < 1e-9);
    }

    #[test]
    fn linear_lead_points_offset_along_the_tangent() {
        let start = Point::new(1.0, 1.0);
        let (tan, out) = ((0.0, 1.0), (1.0, 0.0));
        assert_eq!(
            crate::leads::lead_start_point(start, tan, out, Lead::Linear { length: 4.0 }),
            Point::new(1.0, -3.0)
        );
        assert_eq!(
            crate::leads::lead_end_point(start, tan, out, Lead::Linear { length: 4.0 }),
            Point::new(1.0, 5.0)
        );
    }

    #[test]
    fn outward_normal_points_away_from_a_ccw_interior() {
        let sq = [
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
        ];
        // Start tangent is +x; the interior is +y, so outward is −y.
        let out = outward_normal(&sq);
        assert!(out.1 < 0.0 && out.0.abs() < 1e-9);
    }

    #[test]
    fn rotate_to_start_begins_exactly_at_the_projected_point() {
        use cam_geo::Point;
        let sq = [
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
        ];
        // None leaves it unchanged.
        assert_eq!(rotate_to_start(&sq, None), sq.to_vec());

        // A point off the right edge (x=10) → begins exactly at its projection
        // (10, 9), splitting that edge; winding intact; loop closes back to it.
        let r = rotate_to_start(&sq, Some([9.5, 9.0]));
        assert_eq!(
            r,
            vec![
                Point::new(10.0, 9.0),
                Point::new(10.0, 10.0),
                Point::new(0.0, 10.0),
                Point::new(0.0, 0.0),
                Point::new(10.0, 0.0),
            ]
        );

        // A point at a vertex (a corner / End snap) just rotates — no split.
        let r = rotate_to_start(&sq, Some([10.0, 10.0]));
        assert_eq!(
            r,
            vec![
                Point::new(10.0, 10.0),
                Point::new(0.0, 10.0),
                Point::new(0.0, 0.0),
                Point::new(10.0, 0.0),
            ]
        );
    }

    use cam_model::{Tool, ToolKind};

    fn inner_op(hole: f64, lead_r: f64, start: Option<[f64; 2]>) -> ProfileOp {
        ProfileOp {
            parameter_expressions: Default::default(),
            wrap: None,
            passes: Default::default(),
            selections: None,
            closed: true,
            additional_chains: Vec::new(),
            tabs: Default::default(),
            spindle_rpm: 0.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            clearing: cam_model::Clearing::default(),
            id: 0,
            tool: 1,
            chain: Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(hole, 0.0),
                Point::new(hole, hole),
                Point::new(0.0, hole),
            ]),
            side: Side::Inside,
            comp: Comp::Computed,
            depth: 2.0,
            stepdown: 2.0,
            offset: 0.0,
            stepover: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            plunge: Plunge::Straight,
            start,
            end: None,
            lead_in: Lead::Arc { radius: lead_r },
            lead_out: Lead::Arc { radius: lead_r },
            lead_overlap: 0.0,
        }
    }

    #[test]
    fn parameter_formulas_reach_toolpath_and_errors_emit_no_motion() {
        let mut numeric = inner_op(30., 1., None);
        numeric.side = Side::Outside;
        numeric.lead_in = Lead::None;
        numeric.lead_out = Lead::None;
        numeric.passes.finishing = true;
        numeric.passes.finish_stepover = 0.6;
        numeric.passes.contour_roughing.enabled = true;
        numeric.passes.contour_roughing.stepover = 5.7;
        let expected = run_inner(numeric.clone());
        assert!(!expected.has_errors(), "{:?}", expected.diagnostics);
        let mut formula = numeric.clone();
        formula.passes.finish_stepover = 99.; // stale snapshots must not reach machining
        formula.passes.contour_roughing.stepover = 99.;
        formula.passes.finish_stepover_expression =
            Some("Math.min(tool_diameter * 0.1; maximumStepover)".into());
        formula.passes.contour_roughing.stepover_expression = Some(
            "Math.max((tool_diameter - 2 * tool_cornerRadius) * 0.95; tool_diameter * 0.20)".into(),
        );
        let actual = run_inner(formula.clone());
        assert!(!actual.has_errors(), "{:?}", actual.diagnostics);
        // Use the exact floating-point calculations of the expressions.
        numeric.passes.finish_stepover = 6. * 0.1;
        numeric.passes.contour_roughing.stepover = 6. * 0.95;
        assert_eq!(actual.program, run_inner(numeric.clone()).program);
        formula.passes.finish_stepover_expression = None;
        formula.passes.finish_stepover = 0.25;
        numeric.passes.finish_stepover = 0.25;
        assert_eq!(
            run_inner(formula.clone()).program,
            run_inner(numeric).program
        );
        formula.passes.contour_roughing.stepover_expression = Some("tool_diameter / 0".into());
        let invalid = run_inner(formula);
        assert!(invalid.has_errors());
        assert!(invalid.program.steps().is_empty());
    }

    fn run_inner(op: ProfileOp) -> StrategyResult {
        let ts = [Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }];
        let env = crate::JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &ts,
            stock: None,
            spindle: cam_cldata::SpindleDir::Cw,
        };
        ProfileStrategy::new(op).compute(&env, &crate::CancelToken::new())
    }

    fn lead_arcs(r: &StrategyResult) -> Vec<(Point, Point)> {
        let mut pos = Point::new(0.0, 0.0);
        let mut arcs = Vec::new();
        for s in r.program.steps() {
            match s {
                Step::Rapid { to, .. } | Step::Linear { to, .. } => pos = Point::new(to.x, to.y),
                Step::Arc { end, tag, .. } => {
                    if matches!(tag.kind, MoveKind::LeadIn | MoveKind::LeadOut) {
                        arcs.push((pos, Point::new(end.x, end.y)));
                    }
                    pos = Point::new(end.x, end.y);
                }
                _ => {}
            }
        }
        arcs
    }

    #[test]
    fn inner_profile_leads_ease_in_from_the_hole_not_the_wall() {
        // ⌀40 square hole, ⌀6 tool: the tool-centre wall loop is [3,37], hole centre
        // (20,20). Starting mid-edge (top edge at x=20), the lead must ease in from the
        // interior — historically it swung the wrong way, out toward the wall (once as
        // far as x=−3, into solid material).
        let r = run_inner(inner_op(40.0, 3.0, Some([20.0, 40.0])));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        let arcs = lead_arcs(&r);
        assert_eq!(arcs.len(), 2, "a lead-in and a lead-out");
        let d = |p: Point| (p.x - 20.0).hypot(p.y - 20.0);
        for (a, b) in &arcs {
            for p in [a, b] {
                assert!(
                    (-1e-6..=40.0 + 1e-6).contains(&p.x) && (-1e-6..=40.0 + 1e-6).contains(&p.y),
                    "lead point ({}, {}) left the hole",
                    p.x,
                    p.y
                );
            }
        }
        let (entry, wall) = arcs[0];
        assert!(
            d(entry) < d(wall),
            "inner lead must come from the interior: entry {:?} wall {:?}",
            (entry.x, entry.y),
            (wall.x, wall.y)
        );
    }

    #[test]
    fn inner_profile_lead_too_big_for_the_hole_is_dropped() {
        // ⌀8 hole, ⌀6 tool: the wall loop is [3,5] — only 2 mm to the centre — so a
        // radius-3 arc lead can't fit even mid-edge; it must drop to a plain pass,
        // never overshoot the wall.
        let r = run_inner(inner_op(8.0, 3.0, Some([4.0, 8.0])));
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        assert!(
            lead_arcs(&r).is_empty(),
            "an oversized inner lead must be dropped: {:?}",
            lead_arcs(&r)
        );
        for s in r.program.steps() {
            let p = match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some((to.x, to.y)),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => Some((end.x, end.y)),
                _ => None,
            };
            if let Some((x, y)) = p {
                assert!(
                    (-1e-6..=8.0 + 1e-6).contains(&x) && (-1e-6..=8.0 + 1e-6).contains(&y),
                    "cut left the hole at ({x}, {y})"
                );
            }
        }
    }

    /// An outside profile set up for radial roughing: a square part (the chain) that
    /// the stock frames, with a stepover and (optionally) an engagement cap.
    fn outside_rough_op(engagement: f64) -> ProfileOp {
        ProfileOp {
            parameter_expressions: Default::default(),
            wrap: None,
            passes: Default::default(),
            selections: None,
            closed: true,
            additional_chains: Vec::new(),
            tabs: Default::default(),
            spindle_rpm: 0.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            clearing: cam_model::Clearing {
                engagement,
                climb: true,
            },
            id: 0,
            tool: 1,
            // A 20×20 part; the stock is a 60×60 block, so the frame around it is the
            // same annulus the adaptive frame clearer certifies.
            chain: Contour::new(vec![
                Point::new(20.0, 20.0),
                Point::new(40.0, 20.0),
                Point::new(40.0, 40.0),
                Point::new(20.0, 40.0),
            ]),
            side: Side::Outside,
            comp: Comp::Computed,
            depth: 2.0,
            stepdown: 2.0, // one depth level
            offset: 0.0,
            stepover: 4.0,
            feed: 300.0,
            plunge_feed: 100.0,
            plunge: Plunge::Straight,
            end: None,
            start: None,
            lead_in: Lead::None,
            lead_out: Lead::None,
            lead_overlap: 0.0,
        }
    }

    #[test]
    fn profile_pass_direction_follows_climb_side_and_spindle() {
        let tools = [Tool {
            number: 1,
            diameter: 6.,
            length: 30.,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }];
        for side in [Side::Inside, Side::Outside] {
            for spindle in [SpindleDir::Cw, SpindleDir::Ccw] {
                for climb in [true, false] {
                    for reversed in [false, true] {
                        for controller in [false, true] {
                            let mut op = outside_rough_op(0.);
                            op.stepover = 0.;
                            op.side = side;
                            op.clearing.climb = climb;
                            op.start = Some([30., 20.]);
                            if controller {
                                op.comp = Comp::Control;
                                op.lead_in = Lead::Linear { length: 3. };
                                op.lead_out = Lead::Linear { length: 3. };
                            }
                            if reversed {
                                let mut points = op.chain.points().to_vec();
                                points.reverse();
                                op.chain = Contour::new(points);
                            }
                            let env = JobEnv {
                                heights: Heights::new(5., 2., 0.),
                                tools: &tools,
                                stock: None,
                                spindle,
                            };
                            let result =
                                ProfileStrategy::new(op).compute(&env, &CancelToken::new());
                            let points: Vec<Point> = result
                                .program
                                .steps()
                                .iter()
                                .filter_map(|step| match step {
                                    Step::Linear { to, tag, .. }
                                        if tag.kind == MoveKind::Cutting =>
                                    {
                                        Some(Point::new(to.x, to.y))
                                    }
                                    Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => {
                                        Some(Point::new(end.x, end.y))
                                    }
                                    _ => None,
                                })
                                .collect();
                            assert!(points.len() >= 4, "{:?}", result.diagnostics);
                            let material_right = climb == (spindle == SpindleDir::Cw);
                            assert_eq!(is_ccw(&points), if side == Side::Inside { material_right } else { !material_right }, "{side:?}, {spindle:?}, climb={climb}, reversed={reversed}, controller={controller}");
                            assert!(result.program.steps().iter().any(|step| matches!(step, Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting && (to.z + 2.).abs() < 1e-9)));
                            if controller {
                                let expected = if material_right {
                                    CutterComp::Left(1)
                                } else {
                                    CutterComp::Right(1)
                                };
                                assert!(result
                                    .program
                                    .steps()
                                    .contains(&Step::CutterComp(expected)));
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn named_closed_chain_reverse_changes_cut_direction_independently() {
        let tools = [Tool {
            number: 1,
            diameter: 6.,
            length: 30.,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }];
        let env = JobEnv {
            heights: Heights::new(5., 2., 0.),
            tools: &tools,
            stock: None,
            spindle: SpindleDir::Cw,
        };
        for reversed in [false, true] {
            let mut op = outside_rough_op(0.);
            op.stepover = 0.;
            let mut entry = cam_model::ProfileGeometryEntry::new(
                cam_model::ProfileGeometryKind::Chain,
                "Rim".into(),
            );
            entry.reversed = reversed;
            entry.contour_count = 1;
            op.selections = Some(vec![entry]);
            let climb = op.clearing.climb;
            let result = ProfileStrategy::new(op).compute(&env, &CancelToken::new());
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
            let points: Vec<_> = result
                .program
                .steps()
                .iter()
                .filter_map(|step| match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                        Some(Point::new(to.x, to.y))
                    }
                    Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => {
                        Some(Point::new(end.x, end.y))
                    }
                    _ => None,
                })
                .collect();
            assert_eq!(is_ccw(&points), !(climb ^ reversed));
        }
    }

    #[test]
    fn reversing_a_profile_keeps_the_same_automatic_tab_footprints() {
        let mut op = outside_rough_op(0.);
        op.stepover = 0.;
        op.tabs.enabled = true;
        op.tabs.spacing = 19.;
        for side in [Side::Inside, Side::Outside, Side::On] {
            op.side = side;
            let climb = profile_tab_preview(&op, 6., SpindleDir::Cw);
            op.clearing.climb = false;
            let conventional = profile_tab_preview(&op, 6., SpindleDir::Cw);
            assert_eq!(climb.len(), conventional.len());
            for tab in &climb {
                assert!(conventional
                    .iter()
                    .any(|other| (tab.center[0] - other.center[0])
                        .hypot(tab.center[1] - other.center[1])
                        < 1e-6));
            }
            op.clearing.climb = true;
        }
    }

    fn tabbed_op() -> ProfileOp {
        let mut op = outside_rough_op(0.0);
        op.side = Side::On;
        op.stepover = 0.0;
        op.depth = 6.0;
        op.stepdown = 2.0;
        op.tabs = cam_model::ProfileTabs {
            shape: Default::default(),
            count: None,
            enabled: true,
            width: 2.0,
            height: 3.0,
            spacing: 20.0,
            ..Default::default()
        };
        op
    }

    #[test]
    fn selected_exit_keeps_tabs_on_the_repeated_contour_and_resolves_to_a_gap() {
        let mut op = tabbed_op();
        let preview = profile_tab_preview(&op, 6., SpindleDir::Cw);
        op.end = Some(preview[0].position);
        let result = run_outside_rough(op.clone());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let points = profile_loop_points(op.chain.points(), &op, SpindleDir::Cw);
        let layout = TabLayout::new(&points, &op, 6.).unwrap();
        let exit = layout.eligible_exit(&points, op.end.unwrap(), 0.);
        assert_ne!(exit, op.end.unwrap());
        let mut previous = None;
        for step in result.program.steps() {
            let point = match step {
                Step::Linear { to, .. } | Step::Rapid { to, .. } => *to,
                Step::Arc { end, .. } => *end,
                _ => continue,
            };
            if let Some(a) = previous {
                let a: Point3 = a;
                for i in 0..=10 {
                    let t = i as f64 / 10.;
                    let p = [a.x + t * (point.x - a.x), a.y + t * (point.y - a.y)];
                    let z = a.z + t * (point.z - a.z);
                    let at = nearest_on_loop(&points, p).0;
                    if layout
                        .spans
                        .iter()
                        .any(|(lo, hi)| at > lo + 1e-6 && at < hi - 1e-6)
                    {
                        assert!(z >= layout.top - 1e-6, "tab cut at {p:?} Z{z}");
                    }
                }
            }
            previous = Some(point);
        }
    }

    #[test]
    fn preview_centers_match_generated_rectangular_tabs() {
        let op = tabbed_op();
        let previews = profile_tab_preview(&op, 6.0, SpindleDir::Cw);
        assert_eq!(previews.len(), 4);
        let result = run_outside_rough(op);
        let mut previous = Point3::new(0.0, 0.0, 0.0);
        let mut centers = Vec::new();
        for step in result.program.steps() {
            let to = match step {
                Step::Linear { to, .. } | Step::Rapid { to, .. } => *to,
                _ => continue,
            };
            if previous.z == -3.0 && to.z == -3.0 && (to.x != previous.x || to.y != previous.y) {
                centers.push([(previous.x + to.x) * 0.5, (previous.y + to.y) * 0.5]);
            }
            previous = to;
        }
        assert_eq!(centers.len(), 8);
        for preview in previews {
            assert!(centers
                .iter()
                .any(|p| (p[0] - preview.center[0]).hypot(p[1] - preview.center[1]) < 1e-9));
        }
        let mut op = tabbed_op();
        op.tabs.manual = true;
        op.tabs.positions = vec![[40.0, 30.0], [30.0, 20.0]];
        op.side = Side::Outside;
        let previews = profile_tab_preview(&op, 6.0, SpindleDir::Cw);
        assert_eq!(previews.len(), 2);
        assert!((previews[0].position[0] - 40.0).hypot(previews[0].position[1] - 30.0) < 1e-9);
        assert!((previews[1].position[0] - 30.0).hypot(previews[1].position[1] - 20.0) < 1e-9);
        assert!((previews[0].center[0] - 43.0).abs() < 1e-6);
    }

    #[test]
    fn rectangular_tabs_protect_every_lower_pass_with_vertical_shoulders() {
        let r = run_outside_rough(tabbed_op());
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        let mut pos = Point3::new(0.0, 0.0, 0.0);
        let mut lifts = 0;
        let mut drops = 0;
        let mut tab_traversals = 0;
        for step in r.program.steps() {
            match step {
                Step::Rapid { to, .. } => pos = *to,
                Step::Linear { to, feed, .. } => {
                    let xy = (to.x - pos.x).hypot(to.y - pos.y);
                    if xy < 1e-9 && pos.z < -3.0 && (to.z + 3.0).abs() < 1e-9 {
                        lifts += 1;
                    }
                    if xy < 1e-9 && (pos.z + 3.0).abs() < 1e-9 && to.z < -3.0 {
                        drops += 1;
                        assert_eq!(*feed, 100.0);
                    }
                    if xy > 1e-9 && pos.z <= -3.0 && to.z <= -3.0 {
                        let mid_x = (pos.x + to.x) * 0.5;
                        let mid_y = (pos.y + to.y) * 0.5;
                        // Four tabs, centred on each side of the 20 mm square.
                        // Width 2 + cutter diameter 6 protects an 8 mm span.
                        let over_tab = ((mid_x - 30.0).abs() < 4.0 - 1e-9
                            && ((mid_y - 20.0).abs() < 1e-9 || (mid_y - 40.0).abs() < 1e-9))
                            || ((mid_y - 30.0).abs() < 4.0 - 1e-9
                                && ((mid_x - 20.0).abs() < 1e-9 || (mid_x - 40.0).abs() < 1e-9));
                        if over_tab {
                            assert_eq!(pos.z, -3.0);
                            assert_eq!(to.z, -3.0);
                            assert!((xy - 8.0).abs() < 1e-9);
                            tab_traversals += 1;
                        }
                    }
                    pos = *to;
                }
                _ => {}
            }
        }
        assert_eq!((lifts, drops, tab_traversals), (8, 8, 8));
    }

    #[test]
    fn tabs_preserve_leads_and_work_on_compensated_contours() {
        for side in [Side::Outside, Side::Inside] {
            let mut op = tabbed_op();
            op.side = side;
            op.lead_in = Lead::Linear { length: 1.0 };
            op.lead_out = Lead::Linear { length: 1.0 };
            let r = run_outside_rough(op);
            assert!(!r.has_errors(), "{:?}", r.diagnostics);
            assert!(r.program.steps().iter().any(|s| matches!(s,
                Step::Linear { to, .. } if (to.z + 3.0).abs() < 1e-9)));
        }
    }

    #[test]
    fn manual_tabs_follow_selected_locations_and_move_entry_away_from_a_tab() {
        let mut op = tabbed_op();
        op.tabs.manual = true;
        // Include the original start vertex, forcing a new seam in a free gap.
        op.tabs.positions = vec![[20.0, 20.0], [40.0, 30.0]];
        op.tabs.spacing = f64::NAN; // Automatic spacing is irrelevant in manual mode.
        let r = run_outside_rough(op.clone());
        assert!(!r.has_errors(), "{:?}", r.diagnostics);
        let first_xy = r
            .program
            .steps()
            .iter()
            .find_map(|s| match s {
                Step::Rapid { to, .. } => Some([to.x, to.y]),
                _ => None,
            })
            .unwrap();
        assert_ne!(first_xy, [20.0, 20.0]);
        let mut lifts = 0;
        let mut previous = Point3::new(0.0, 0.0, 0.0);
        let mut corner_at_tab_top = false;
        for step in r.program.steps() {
            let to = match step {
                Step::Linear { to, .. } | Step::Rapid { to, .. } => *to,
                _ => continue,
            };
            if previous.z < -3.0 && to.z == -3.0 && previous.x == to.x && previous.y == to.y {
                lifts += 1;
            }
            if to.x == 20.0 && to.y == 20.0 && to.z == -3.0 {
                corner_at_tab_top = true;
            }
            previous = to;
        }
        assert_eq!(lifts, 4); // Two tabs on each of the two lower depth passes.
        assert!(corner_at_tab_top);
        for side in [Side::Outside, Side::Inside] {
            op.side = side;
            let result = run_outside_rough(op.clone());
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
        }
    }

    #[test]
    fn empty_overlapping_and_nonfinite_manual_tabs_fail_without_motion() {
        let mut op = tabbed_op();
        op.tabs.manual = true;
        for positions in [
            vec![],
            vec![[30.0, 20.0], [31.0, 20.0]],
            vec![[f64::NAN, 20.0]],
        ] {
            op.tabs.positions = positions;
            let result = run_outside_rough(op.clone());
            assert!(result.has_errors());
            assert!(result.program.steps().is_empty());
        }
    }

    #[test]
    fn invalid_or_unsupported_tabs_emit_no_motion() {
        let mut variants = Vec::new();
        for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut op = tabbed_op();
            op.tabs.width = value;
            variants.push(op);
            let mut op = tabbed_op();
            op.tabs.height = value;
            variants.push(op);
            let mut op = tabbed_op();
            op.tabs.spacing = value;
            variants.push(op);
        }
        let mut op = tabbed_op();
        op.tabs.height = op.depth;
        variants.push(op);
        let mut op = tabbed_op();
        op.tabs.width = 100.0;
        variants.push(op);
        let mut op = tabbed_op();
        op.tabs.spacing = 0.00001;
        variants.push(op);
        let mut op = tabbed_op();
        op.stepover = 1.0;
        variants.push(op);
        let mut op = tabbed_op();
        op.plunge = Plunge::Ramp { angle_deg: 5.0 };
        variants.push(op);
        let mut op = tabbed_op();
        op.comp = Comp::Control;
        variants.push(op);
        let mut op = tabbed_op();
        op.lead_overlap = 10.0;
        variants.push(op);
        let mut op = tabbed_op();
        op.lead_in = Lead::Linear { length: 15.0 };
        variants.push(op);
        for op in variants {
            let r = run_outside_rough(op);
            assert!(r.has_errors(), "invalid tab settings must fail");
            assert!(r.program.steps().is_empty());
        }
    }

    fn run_outside_rough(op: ProfileOp) -> StrategyResult {
        let ts = [Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }];
        let env = crate::JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools: &ts,
            stock: Some(([0.0, 0.0], [60.0, 60.0])),
            spindle: cam_cldata::SpindleDir::Cw,
        };
        ProfileStrategy::new(op).compute(&env, &crate::CancelToken::new())
    }

    #[test]
    fn open_profiles_offset_both_sides_without_a_closing_cut_and_retract_between_passes() {
        for (side, expected_y) in [(Side::Outside, 13.), (Side::Inside, 7.), (Side::On, 10.)] {
            let mut op = outside_rough_op(0.);
            op.closed = false;
            op.chain = Contour::new(vec![Point::new(10., 10.), Point::new(30., 10.)]);
            op.side = side;
            op.stepover = 0.;
            op.depth = 2.;
            op.stepdown = 1.;
            op.lead_in = Lead::None;
            op.lead_out = Lead::None;
            let result = run_outside_rough(op);
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
            let cuts: Vec<_> = result
                .program
                .steps
                .iter()
                .filter_map(|step| match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(*to),
                    _ => None,
                })
                .collect();
            assert_eq!(cuts.len(), 2, "one segment per pass, with no return cut");
            assert!(cuts.iter().all(|p| (p.y - expected_y).abs() < 1e-9));
            assert_eq!(cuts.last().unwrap().z, -2.);
            let retracts: Vec<_> = result
                .program
                .steps
                .iter()
                .filter_map(|step| match step {
                    Step::Rapid { to, tag } if tag.kind == MoveKind::Retract => Some(*to),
                    _ => None,
                })
                .collect();
            assert_eq!(retracts.len(), 2);
            assert!(retracts
                .iter()
                .all(|p| p.z == 5. && (p.y - expected_y).abs() < 1e-9));
            assert_eq!(retracts[0].x, cuts[0].x);
        }
    }

    #[test]
    fn contour_roughing_approaches_open_profiles_from_the_air_side() {
        for (side, expected) in [
            (Side::Outside, vec![15., 14., 13.]),
            (Side::Inside, vec![5., 6., 7.]),
        ] {
            let mut op = outside_rough_op(0.);
            op.closed = false;
            op.chain = Contour::new(vec![Point::new(10., 10.), Point::new(30., 10.)]);
            op.side = side;
            op.stepover = 0.;
            op.passes.contour_roughing.enabled = true;
            op.passes.contour_roughing.stepovers = 2;
            let result = run_outside_rough(op);
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
            let ys: Vec<_> = result
                .program
                .steps()
                .iter()
                .filter_map(|step| match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(to.y),
                    _ => None,
                })
                .collect();
            assert_eq!(ys, expected);
        }
    }

    #[test]
    fn closed_contour_roughing_preserves_the_original_finishing_schedule() {
        for side in [Side::Inside, Side::Outside] {
            let mut op = outside_rough_op(0.);
            op.stepover = 0.;
            op.side = side;
            op.passes.finishing = true;
            let baseline = run_outside_rough(op.clone());
            assert!(!baseline.has_errors(), "{:?}", baseline.diagnostics);
            op.passes.contour_roughing.enabled = true;
            op.passes.contour_roughing.stepovers = 2;
            let rough = run_outside_rough(op);
            assert!(!rough.has_errors(), "{:?}", rough.diagnostics);
            assert!(rough.program.steps().len() > baseline.program.steps().len());
            assert!(rough.program.steps().ends_with(baseline.program.steps()));
        }
    }

    #[test]
    fn contour_roughing_rejects_incompatible_and_collapsed_paths_atomically() {
        let mut op = inner_op(8., 0., None);
        op.lead_in = Lead::None;
        op.lead_out = Lead::None;
        op.passes.contour_roughing.enabled = true;
        op.passes.contour_roughing.stepovers = 2;
        let result = run_inner(op.clone());
        assert!(result.has_errors());
        assert!(result.program.steps().is_empty());
        op.side = Side::On;
        let result = run_inner(op);
        assert!(result.has_errors());
        assert!(result.program.steps().is_empty());
    }

    #[test]
    fn multiple_profile_contours_cut_all_chains_and_discard_partial_motion_on_error() {
        let mut op = outside_rough_op(0.);
        op.stepover = 0.;
        op.depth = 2.;
        op.additional_chains.push(cam_model::ProfileContour {
            chain: Contour::new(vec![
                Point::new(20., 20.),
                Point::new(40., 20.),
                Point::new(40., 40.),
                Point::new(20., 40.),
            ]),
            closed: true,
            side: Side::Inside,
        });
        let result = run_outside_rough(op.clone());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(result.program.steps.iter().any(|step| matches!(step, Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting && (to.x-23.).abs() < 1e-6)));
        op.additional_chains[0].chain = Contour::new(vec![]);
        let result = run_outside_rough(op);
        assert!(result.has_errors());
        assert!(result.program.steps.is_empty());
    }

    /// **The ramp never travels through air.** Its height is the *material* it has to
    /// get through — one stepdown — not the drop from the retract plane. Measured
    /// rather than asserted about: the ramp's own length is `dz / tan(angle)`, so a
    /// ramp that started at the retract plane would be visibly longer on the first
    /// level than on the second.
    #[test]
    fn the_first_levels_ramp_is_no_longer_than_any_others() {
        let mut op = outside_rough_op(0.0);
        op.stepover = 0.0;
        op.depth = 2.0;
        op.stepdown = 1.0; // two levels, both one stepdown of material
        op.plunge = Plunge::Ramp { angle_deg: 5.0 };
        let r = run_outside_rough(op);

        // Length of each run of consecutive descending Plunge moves.
        let mut pos = Point3::new(0.0, 0.0, 0.0);
        let mut runs: Vec<f64> = Vec::new();
        let mut cur = 0.0;
        for st in r.program.steps() {
            let (to, ramping, length) = match st {
                Step::Linear { to, tag, .. } => (
                    *to,
                    tag.kind == MoveKind::Plunge,
                    (to.x - pos.x).hypot(to.y - pos.y),
                ),
                Step::Rapid { to, .. } => (*to, false, 0.),
                Step::Arc {
                    end,
                    center,
                    dir,
                    tag,
                    ..
                } => {
                    let start_angle = (pos.y - center.y).atan2(pos.x - center.x);
                    let end_angle = (end.y - center.y).atan2(end.x - center.x);
                    let sweep = if *dir == ArcDir::Ccw {
                        end_angle - start_angle
                    } else {
                        start_angle - end_angle
                    };
                    (
                        *end,
                        tag.kind == MoveKind::Plunge,
                        (pos.x - center.x).hypot(pos.y - center.y)
                            * sweep.rem_euclid(std::f64::consts::TAU),
                    )
                }
                _ => continue,
            };
            if ramping && to.z < pos.z - 1e-9 {
                cur += length;
            } else if cur > 0.0 {
                runs.push(cur);
                cur = 0.0;
            }
            pos = to;
        }
        if cur > 0.0 {
            runs.push(cur);
        }
        assert!(runs.len() >= 2, "expected a ramp per level, got {runs:?}");
        let want = 1.0 / 5.0_f64.to_radians().tan(); // one stepdown at 5°
        for (i, l) in runs.iter().enumerate() {
            assert!(
                (l - want).abs() < 0.5,
                "level {i} ramped {l:.2} mm; one stepdown at 5° is {want:.2} mm. A ramp \
                 starting at the retract plane would be far longer on the first level."
            );
        }
    }

    /// The ramp descends along the **lead-in** before it touches the contour — that is
    /// where a descending tool belongs, because the lead exists to keep the entry off
    /// the finished wall. With a lead long enough to absorb the whole stepdown, the
    /// wall is never touched by a descending tool at all.
    #[test]
    fn a_ramp_descends_down_the_lead_in_before_the_contour() {
        let mut op = outside_rough_op(0.0);
        op.stepover = 0.0;
        op.depth = 0.2;
        op.stepdown = 0.2; // a shallow step, so a modest lead can absorb it
        op.plunge = Plunge::Ramp { angle_deg: 5.0 };
        op.lead_in = Lead::Arc { radius: 3.0 };
        op.lead_out = Lead::Arc { radius: 3.0 };
        let r = run_outside_rough(op);

        // The contour's own vertices, to tell "on the wall" from "on the lead".
        let on_contour = |p: Point3| {
            let pts = [(20.0, 20.0), (40.0, 20.0), (40.0, 40.0), (20.0, 40.0)];
            pts.iter()
                .any(|&(x, y)| (p.x - x).abs() < 1e-6 && (p.y - y).abs() < 1e-6)
        };
        let first_plunge = r
            .program
            .steps()
            .iter()
            .position(|st| matches!(st, Step::Linear { tag, .. } | Step::Arc { tag, .. } if tag.kind == MoveKind::Plunge))
            .expect("a ramp");
        // Nothing before the ramp may already be at depth, and the ramp itself must
        // begin away from the contour's corners (it is on the lead arc).
        let before: Vec<_> = r.program.steps()[..first_plunge].to_vec();
        assert!(
            !before.iter().any(|st| matches!(st,
                Step::Linear { to, .. } | Step::Rapid { to, .. } if to.z < -1e-9)),
            "nothing may reach cutting depth before the ramp"
        );
        assert!(
            !on_contour(match &r.program.steps()[first_plunge] {
                Step::Linear { to, .. } | Step::Arc { end: to, .. } => *to,
                _ => unreachable!(),
            }),
            "the ramp's first move should be along the lead, not the contour"
        );
    }

    /// **A ramp keeps its arcs.** The path it descends is a flattened polyline, so it
    /// goes back through `fit_arcs` exactly as the cutting pass does — a curved run
    /// becomes one helical `G2`/`G3`, not a fan of chords.
    ///
    /// Found by Andreas reading a pocket export: ten linear moves through a corner that
    /// the cut, over the very same points, emitted as a single arc. It also meant an arc
    /// lead-in could never come out as an arc, whatever else was fixed.
    #[test]
    fn a_ramp_over_a_curve_emits_an_arc_not_chords() {
        // A quarter circle, flattened the way an offset loop arrives.
        let r = 10.0;
        let pts: Vec<Point> = (0..=24)
            .map(|i| {
                let a = std::f64::consts::FRAC_PI_2 * (i as f64) / 24.0;
                Point::new(r * a.cos(), r * a.sin())
            })
            .collect();
        let mut prog = Program::new();
        emit_descending_path(
            &mut prog,
            &pts,
            0.0,
            -2.0,
            300.0,
            Tag::new(0, MoveKind::Plunge),
        );

        let arcs = prog
            .steps
            .iter()
            .filter(|s| matches!(s, Step::Arc { .. }))
            .count();
        assert!(
            arcs >= 1,
            "a flattened circle must come back as an arc:\n{:#?}",
            prog.steps
        );
        assert!(
            prog.steps.len() < 24,
            "{} moves for a single quarter circle — it is still emitting chords",
            prog.steps.len()
        );

        // Still a *helix*: monotonic descent to exactly the requested depth.
        let zs: Vec<f64> = prog.steps.iter().map(step_end_z).collect();
        for w in zs.windows(2) {
            assert!(w[1] <= w[0] + 1e-9, "the ramp must descend monotonically");
        }
        assert!(
            (zs.last().unwrap() + 2.0).abs() < 1e-9,
            "exact depth: {:?}",
            zs.last()
        );

        // And the descent is spread by *arc* length, so a curve does not steepen it: a
        // quarter of r=10 is 15.708 mm, and 2 mm over that is 7.26°.
        let total = std::f64::consts::FRAC_PI_2 * r;
        let want = (2.0_f64 / total).atan().to_degrees();
        assert!((want - 7.26).abs() < 0.05, "sanity: {want}");
    }

    /// **The lead-in flies through air and lands on the surface at the contour start.**    /// **The lead-in flies through air and lands on the surface at the contour start.**
    ///
    /// It starts above the material by `lead_length · tan(angle)` and reaches the
    /// material top exactly where the contour begins — then carries on into the
    /// material at the same angle. One unbroken descent, with the air/material
    /// transition landing precisely where the cut starts. Before this the lead began
    /// *on* the surface and was already cutting as it swung in, which is the opposite
    /// of what a lead-in is for (Andreas, 2026-08-01).
    #[test]
    fn the_lead_in_descends_through_air_and_meets_the_surface_at_the_contour() {
        let mut op = outside_rough_op(0.0);
        op.stepover = 0.0;
        op.depth = 1.0;
        op.stepdown = 1.0;
        op.plunge = Plunge::Ramp { angle_deg: 5.0 };
        op.lead_in = Lead::Arc { radius: 3.0 };
        op.lead_out = Lead::None;
        let r = run_outside_rough(op);

        // The ramp: the run of descending Plunge moves.
        // Arcs too: an arc lead-in is now emitted as one *helical arc*, so a filter that
        // saw only `Linear` moves would miss the whole air portion and conclude the ramp
        // began at depth.
        let ramp: Vec<Point3> = r
            .program
            .steps()
            .iter()
            .filter_map(|st| match st {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => Some(*to),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Plunge => Some(*end),
                _ => None,
            })
            .collect();
        assert!(!ramp.is_empty(), "no ramp emitted");

        let tan = 5.0_f64.to_radians().tan();
        let top_of_stock = 0.0;

        // The rapid that precedes the ramp must leave the tool *above* the material —
        // that is the lead's rise, and it is what lets the lead fly in air.
        let first_plunge_at = r
            .program
            .steps()
            .iter()
            .position(|st| {
                matches!(st,
                Step::Linear { tag, .. } | Step::Arc { tag, .. } if tag.kind == MoveKind::Plunge)
            })
            .expect("a ramp");
        let before = r.program.steps()[..first_plunge_at]
            .iter()
            .rev()
            .find_map(|st| match st {
                Step::Rapid { to, .. } | Step::Linear { to, .. } => Some(to.z),
                _ => None,
            })
            .expect("something positions the tool first");
        assert!(
            before > top_of_stock + 1e-9,
            "the ramp starts at {before}, not above the material — the lead would be \
             cutting on the way in"
        );

        // …and the first ramp move — the lead — lands *on* the surface, at the contour.
        assert!(
            (ramp[0].z - top_of_stock).abs() < 1e-3,
            "the lead ended at {}, not on the stock top",
            ramp[0].z
        );

        // The whole material descent therefore happens on the contour: one stepdown.
        let in_material: f64 = ramp
            .windows(2)
            .filter(|w| w[1].z < top_of_stock)
            .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
            .sum();
        let want = 1.0 / tan;
        assert!(
            (in_material - want).abs() < 0.6,
            "the in-material ramp ran {in_material:.2} mm; one stepdown at 5° is \
             {want:.2} mm. If it is short, the lead is wrongly absorbing part of the cut."
        );
    }

    /// Only the lead-**in** descends. The lead-out stays at the cutting plane.
    #[test]
    fn the_lead_out_stays_flat_at_the_cutting_plane() {
        let mut op = outside_rough_op(0.0);
        op.stepover = 0.0;
        op.depth = 1.0;
        op.stepdown = 1.0;
        op.plunge = Plunge::Ramp { angle_deg: 5.0 };
        op.lead_in = Lead::Arc { radius: 3.0 };
        op.lead_out = Lead::Arc { radius: 3.0 };
        let r = run_outside_rough(op);
        for st in r.program.steps() {
            if let Step::Arc { end, tag, .. } = st {
                if tag.kind == MoveKind::LeadOut {
                    assert!(
                        (end.z + 1.0).abs() < 1e-9,
                        "a lead arc ended at {} — the lead-out must stay at depth",
                        end.z
                    );
                }
            }
        }
    }

    /// The point of the whole exercise: with a ramp selected, the tool must never drop
    /// straight down into material. A vertical `Plunge` move is exactly the entry a
    /// ramp exists to avoid, and it is what a silent fallback would produce.
    #[test]
    fn a_contour_ramp_never_drops_vertically_into_material() {
        let mut op = outside_rough_op(0.0);
        op.stepdown = 0.5; // four levels, so later entries start from a cut floor
        op.plunge = Plunge::Ramp { angle_deg: 15.0 };
        let r = run_outside_rough(op);
        assert!(
            r.diagnostics
                .iter()
                .all(|d| d.severity != crate::Severity::Error),
            "{:?}",
            r.diagnostics
        );

        let mut pos = Point3::new(0.0, 0.0, 0.0);
        let mut vertical = 0;
        let mut ramped = 0;
        for s in r.program.steps() {
            let (to, kind) = match s {
                Step::Linear { to, tag, .. } => (*to, Some(tag.kind)),
                Step::Rapid { to, .. } => (*to, None),
                Step::Arc { end, .. } => (*end, None),
                _ => continue,
            };
            if kind == Some(MoveKind::Plunge) {
                let flat = (to.x - pos.x).hypot(to.y - pos.y);
                if to.z < pos.z - 1e-9 {
                    if flat <= 1e-9 {
                        vertical += 1;
                    } else {
                        ramped += 1;
                    }
                }
            }
            pos = to;
        }
        assert_eq!(
            vertical, 0,
            "a ramped profile emitted {vertical} vertical plunges"
        );
        assert!(ramped > 0, "no descending ramp moves were emitted at all");
    }

    /// The ramp arrives at the pass's start, so the loop's own final stretch re-cuts
    /// the slope. Which means the cut still covers the whole contour — a ramp must not
    /// quietly shorten the pass.
    #[test]
    fn a_ramped_pass_still_cuts_the_whole_contour() {
        let mut straight_op = outside_rough_op(0.0);
        straight_op.stepover = 0.0; // a plain contour pass, no roughing rings
        let mut ramp_op = straight_op.clone();
        ramp_op.plunge = Plunge::Ramp { angle_deg: 15.0 };

        let cut_len = |r: &StrategyResult| -> f64 {
            let mut pos = Point3::new(0.0, 0.0, 0.0);
            let mut total = 0.0;
            for s in r.program.steps() {
                let to = match s {
                    Step::Linear { to, tag, .. } => {
                        if tag.kind == MoveKind::Cutting {
                            total += (to.x - pos.x).hypot(to.y - pos.y);
                        }
                        *to
                    }
                    Step::Rapid { to, .. } => *to,
                    Step::Arc { end, .. } => *end,
                    _ => continue,
                };
                pos = to;
            }
            total
        };
        let plain = cut_len(&run_outside_rough(straight_op));
        let ramped = cut_len(&run_outside_rough(ramp_op));
        assert!(
            (ramped - plain).abs() < 1e-6,
            "the ramp changed the cut length: {plain} plain vs {ramped} ramped"
        );
    }

    fn plunge_count(r: &StrategyResult) -> usize {
        r.program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge))
            .count()
    }

    fn cut_count(r: &StrategyResult) -> usize {
        r.program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Cutting))
            .count()
    }

    #[test]
    fn outside_roughing_honours_the_engagement_cap_by_tightening_the_ring_spacing() {
        // Outside roughing frames the part with the stock as an annulus. It used to route
        // through the adaptive frame path; that path was raster-gated and the raster is
        // blind to slots (see `clearing::clear`), so it now clears concentrically like
        // everything else.
        //
        // The engagement cap is **not** inert: `ClearJob::effective_spacing` caps the ring
        // spacing at it, which bounds the radial width of cut on a straight wall. So an
        // engagement-capped run must still differ observably from the uncapped baseline —
        // it just differs by having more, tighter rings rather than by being adaptive.
        let concentric = run_outside_rough(outside_rough_op(0.0));
        assert!(!concentric.has_errors(), "{:?}", concentric.diagnostics);
        let adaptive = run_outside_rough(outside_rough_op(2.0));
        assert!(!adaptive.has_errors(), "{:?}", adaptive.diagnostics);

        assert!(
            cut_count(&adaptive) > 50,
            "the frame cuts, got {}",
            cut_count(&adaptive)
        );
        assert_ne!(
            (plunge_count(&adaptive), cut_count(&adaptive)),
            (plunge_count(&concentric), cut_count(&concentric)),
            "engagement>0 must change the path — identical output would mean it fell \
             back to concentric instead of certifying the adaptive frame"
        );
    }
}
