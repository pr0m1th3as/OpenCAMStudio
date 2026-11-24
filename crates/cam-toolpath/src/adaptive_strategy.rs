//! 3D adaptive roughing over conservative CAD-derived layer shadows.
use cam_cldata::Program;
use cam_geo::{difference, offset, Contour, JoinStyle, Point, Polygon};
use cam_geometry3d::AdaptiveGeometry;
use cam_model::{AdaptiveOp, Clearing, Lead, Plunge, StockToLeave};

use crate::{
    clearing::ClearResult, rings::RingsError, CancelToken, Diagnostic, JobEnv, Strategy,
    StrategyResult,
};

#[derive(Clone, Debug)]
pub struct AdaptiveStrategy {
    op: AdaptiveOp,
    stock_to_leave: StockToLeave,
    geometry: AdaptiveGeometry,
}

impl AdaptiveStrategy {
    /// Invalidate persisted paths when machining domain, linking or safety rules change.
    pub const DOMAIN_REVISION: u32 = 11;

    pub fn new(op: AdaptiveOp, stock_to_leave: StockToLeave, geometry: AdaptiveGeometry) -> Self {
        Self {
            op,
            stock_to_leave,
            geometry,
        }
    }
}

impl Strategy for AdaptiveStrategy {
    fn name(&self) -> &str {
        "adaptive"
    }

    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        self.compute_with_preview(env, cancel, &mut |_| {})
    }
}
impl AdaptiveStrategy {
    pub(crate) fn compute_with_preview(
        &self,
        env: &JobEnv,
        cancel: &CancelToken,
        publish: &mut dyn FnMut(&Program),
    ) -> StrategyResult {
        if self.op.parameter_expressions.is_empty() {
            return self.compute_resolved_parameters(env, cancel, publish);
        }
        let mut operation = cam_model::Operation::Adaptive(self.op.clone());
        if let Err(message) = operation.resolve_parameter_expressions(env.tools) {
            return StrategyResult {
                diagnostics: vec![crate::Diagnostic::error(message)],
                ..Default::default()
            };
        }
        let cam_model::Operation::Adaptive(op) = operation else {
            unreachable!()
        };
        Self { op, ..self.clone() }.compute_resolved_parameters(env, cancel, publish)
    }

    fn compute_resolved_parameters(
        &self,
        env: &JobEnv,
        cancel: &CancelToken,
        publish: &mut dyn FnMut(&Program),
    ) -> StrategyResult {
        let op = &self.op;
        let mut diagnostics = Vec::new();
        macro_rules! fail { ($($arg:tt)*) => {{ diagnostics.push(Diagnostic::error(format!($($arg)*))); return StrategyResult { diagnostics, ..Default::default() }; }} }
        if let Err(reason) = op.entry_feeds.validate() {
            fail!("operation {}: {reason}", op.id);
        }
        if !op.params.lift_height.is_finite() || op.params.lift_height < 0.0 {
            fail!(
                "operation {}: lift height must be finite and nonnegative",
                op.id
            );
        }
        for (name, radius) in [
            ("lead-in", op.params.lead_in_radius),
            ("lead-out", op.params.lead_out_radius),
        ] {
            if !radius.is_finite() || radius < 0.0 {
                fail!(
                    "operation {}: {name} arc radius must be finite and nonnegative",
                    op.id
                );
            }
        }
        if !op.params.tolerance.is_finite() || op.params.tolerance <= 0.0 {
            fail!("operation {}: tolerance must be finite and positive", op.id);
        }
        if !op.params.optimal_load.is_finite() || op.params.optimal_load <= 0.0 {
            fail!(
                "operation {}: optimal load must be finite and positive",
                op.id
            );
        }
        if !op.params.stepdown.is_finite()
            || op.params.stepdown <= 0.0
            || !op.params.fine_stepdown.is_finite()
            || op.params.fine_stepdown <= 0.0
        {
            fail!(
                "operation {}: roughing and fine stepdown must be finite and positive",
                op.id
            );
        }
        if !op.params.feed.is_finite()
            || op.params.feed <= 0.0
            || !op.params.plunge_feed.is_finite()
            || op.params.plunge_feed <= 0.0
        {
            fail!(
                "operation {}: cutting and plunge feeds must be finite and positive",
                op.id
            );
        }
        if !op.params.additional_offset.is_finite() {
            fail!(
                "operation {}: adaptive additional offset must be finite",
                op.id
            );
        }
        if self.stock_to_leave.enabled
            && (!self.stock_to_leave.radial.is_finite()
                || self.stock_to_leave.radial < 0.0
                || !self.stock_to_leave.axial.is_finite()
                || self.stock_to_leave.axial < 0.0)
        {
            fail!(
                "operation {}: stock allowance must be finite and nonnegative",
                op.id
            );
        }
        let Some(tool) = env.tool(op.tool) else {
            fail!("operation {}: tool {} is not in the setup", op.id, op.tool);
        };
        if self.geometry.layers.is_empty() {
            fail!(
                "operation {}: adaptive geometry has no cutting levels",
                op.id
            );
        }
        if !self.geometry.model_top.is_finite()
            || !env.heights.clearance.is_finite()
            || env.heights.clearance <= self.geometry.model_top
        {
            fail!(
                "operation {}: clearance must be above every setup body (model top Z{:.4})",
                op.id,
                self.geometry.model_top
            );
        }
        let z_top = self
            .geometry
            .layers
            .iter()
            .map(|l| l.z)
            .fold(f64::NEG_INFINITY, f64::max);
        let z_bottom = self
            .geometry
            .layers
            .iter()
            .map(|l| l.z)
            .fold(f64::INFINITY, f64::min);
        crate::guards::warn_side_milling(
            op.id,
            "3D adaptive",
            tool,
            z_top - z_bottom,
            &mut diagnostics,
        );
        crate::guards::warn_flat_floor(op.id, "3D adaptive", tool, &mut diagnostics);
        crate::guards::warn_plunge(op.id, "3D adaptive", tool, &mut diagnostics);
        if op.params.optimal_load > tool.diameter {
            fail!(
                "operation {}: optimal load cannot exceed the tool diameter",
                op.id
            );
        }
        let (stock_min, stock_max) = match env.stock {
            Some(bounds) => bounds,
            None => fail!(
                "operation {}: 3D Adaptive requires defined stock bounds",
                op.id
            ),
        };
        if !stock_min.into_iter().chain(stock_max).all(f64::is_finite)
            || stock_min[0] >= stock_max[0]
            || stock_min[1] >= stock_max[1]
        {
            fail!("operation {}: stock XY bounds are invalid", op.id);
        }
        let stock_region = match Polygon::new(Contour::new(vec![
            Point::new(stock_min[0], stock_min[1]),
            Point::new(stock_max[0], stock_min[1]),
            Point::new(stock_max[0], stock_max[1]),
            Point::new(stock_min[0], stock_max[1]),
        ])) {
            Ok(p) => p,
            Err(e) => fail!("operation {}: invalid stock bounds: {e}", op.id),
        };

        let containment_expansion = match op.params.containment {
            cam_model::AdaptiveContainment::Inside => 0.0,
            cam_model::AdaptiveContainment::Center => tool.radius(),
            cam_model::AdaptiveContainment::Outside => tool.diameter,
        };
        let user_expansion = match op.params.containment {
            cam_model::AdaptiveContainment::Inside => -op.params.additional_offset,
            cam_model::AdaptiveContainment::Center | cam_model::AdaptiveContainment::Outside => {
                op.params.additional_offset
            }
        };
        let domain_expansion = containment_expansion + user_expansion;
        // Clearing insets its input by the cutter radius. Allow the same runout
        // at stock edges as at selected boundaries, or that inset forces the
        // cutter back inside stock even with Center/Outside containment.
        let stock_limit = match offset(
            std::slice::from_ref(&stock_region),
            domain_expansion.max(0.0),
            JoinStyle::Round,
        ) {
            Ok(regions) => regions,
            Err(e) => fail!(
                "operation {}: could not expand the stock machining limit: {e}",
                op.id
            ),
        };

        let largest_radius = (tool.diameter.max(tool.shank_dia()).max(tool.neck_dia())) / 2.0;
        let extra_shank = largest_radius - tool.radius();
        let allowance = if self.stock_to_leave.enabled {
            self.stock_to_leave.radial
        } else {
            0.0
        };
        let safety_radius = largest_radius + allowance + self.geometry.error_bound;
        // Frontier simplification may move a chord inward. Reserve that motion
        // as well as arc refitting before the final swept-cutter check.
        let protected_growth = extra_shank
            + allowance
            + self.geometry.error_bound
            + op.params.simplification_tolerance();
        let emitted_radius = largest_radius
            + allowance
            + (self.geometry.error_bound - crate::emit::ARCFIT_TOL).max(0.0);
        let mut program = Program::new();
        let mut cleared_levels = 0usize;
        let mut previous_z: Option<f64> = None;
        let mut cleared_slice = Vec::new();
        let mut used_concentric = false;
        for layer in &self.geometry.layers {
            if cancel.is_cancelled() {
                return StrategyResult {
                    program,
                    diagnostics,
                    cancelled: true,
                };
            }
            if !layer.z.is_finite() {
                fail!(
                    "operation {}: adaptive geometry contains a nonfinite level",
                    op.id
                );
            }
            // A toolpath above stock cannot remove material. In particular,
            // do not count a selected face's entry plane as a successful pass
            // when every reachable level is protected or outside its domain.
            if layer.z >= env.heights.top_of_stock - 1e-9 {
                continue;
            }
            let safe = if layer.protected.is_empty() {
                Vec::new()
            } else {
                match offset(&layer.protected, protected_growth, JoinStyle::Round) {
                    Ok(v) => v,
                    Err(e) => fail!(
                        "operation {}: could not create the cutter safety envelope: {e}",
                        op.id
                    ),
                }
            };
            let domains = if layer.body_domains.is_empty() && layer.face_domains.is_empty() {
                match offset(
                    std::slice::from_ref(&stock_region),
                    domain_expansion,
                    JoinStyle::Round,
                ) {
                    Ok(regions) => regions,
                    Err(e) => fail!(
                        "operation {}: could not offset the stock machining domain: {e}",
                        op.id
                    ),
                }
            } else {
                // Inset the combined selection, so shared edges between
                // adjacent faces do not leave strips of untouched stock.
                let selected = match cam_geo::union(&layer.body_domains, &layer.face_domains) {
                    Ok(regions) => regions,
                    Err(e) => fail!(
                        "operation {}: could not combine the selected machining domains: {e}",
                        op.id
                    ),
                };
                let selected_domains = match offset(
                    &selected,
                    domain_expansion - self.geometry.error_bound - 2.0 * cam_geo::GRID_MM,
                    JoinStyle::Round,
                ) {
                    Ok(regions) => regions,
                    Err(e) => fail!(
                        "operation {}: could not offset the selected machining domains by the certified path error: {e}",
                        op.id
                    ),
                };
                match cam_geo::intersection(&selected_domains, &stock_limit) {
                    Ok(regions) => regions,
                    Err(e) => fail!(
                        "operation {}: could not clip selected machining domains to stock: {e}",
                        op.id
                    ),
                }
            };
            let waste = match difference(&domains, &safe) {
                Ok(v) => v,
                Err(e) => fail!(
                    "operation {}: could not derive the layer clearing area: {e}",
                    op.id
                ),
            };
            // Levels ascend only while refining the preceding roughing slice.
            // Compare individual disconnected regions too: a slope elsewhere
            // must not make a straight-sided pocket repeat all its cleared paths.
            let refinement = previous_z.is_some_and(|z| layer.z > z + 1e-9);
            previous_z = Some(layer.z);
            if !refinement {
                cleared_slice.clear();
            }
            let already_cleared = if refinement && !cleared_slice.is_empty() {
                match offset(
                    &cleared_slice,
                    (op.params.tolerance - self.geometry.error_bound).max(0.0),
                    JoinStyle::Round,
                ) {
                    Ok(regions) => Some(regions),
                    Err(e) => fail!(
                        "operation {}: could not compare fine clearing regions: {e}",
                        op.id
                    ),
                }
            } else {
                None
            };
            for region in &waste {
                if cancel.is_cancelled() {
                    return StrategyResult {
                        program,
                        diagnostics,
                        cancelled: true,
                    };
                }
                let refinement_regions = if let Some(cleared) = &already_cleared {
                    let rest = match difference(std::slice::from_ref(region), cleared) {
                        Ok(rest) => rest,
                        Err(e) => fail!(
                            "operation {}: could not derive fine rest material: {e}",
                            op.id
                        ),
                    };
                    if rest.is_empty() {
                        continue;
                    }
                    if op.params.start.is_none() && op.params.end.is_none() {
                        fine_cutting_regions(region, &rest, tool.radius(), op.params.tolerance)
                            .ok()
                            .filter(|regions| !regions.is_empty())
                    } else {
                        None
                    }
                } else {
                    None
                };
                let generate = |regions: &[Polygon]| {
                    let mut layer_program = Program::new();
                    let mut cut_regions = Vec::new();
                    let mut concentric = false;
                    for region in regions {
                        if cancel.is_cancelled() {
                            return Err(RingsError::Cancelled);
                        }
                        let result = self.clear_region(
                            &mut layer_program,
                            region,
                            layer,
                            env,
                            tool,
                            safety_radius,
                            cancel,
                        )?;
                        if result.paths > 0 {
                            cut_regions.push(region.clone());
                            concentric |= result.concentric;
                        }
                    }
                    Ok((layer_program, cut_regions, concentric))
                };
                let full_region = std::slice::from_ref(region);
                let result = if let Some(regions) = refinement_regions.as_deref() {
                    generate(regions).or_else(|e| match e {
                        RingsError::Cancelled => Err(e),
                        _ => generate(full_region),
                    })
                } else {
                    generate(full_region)
                };
                let (layer_program, cut_regions, concentric) = match result {
                    Ok(result) => result,
                    Err(RingsError::Cancelled) => {
                        return StrategyResult {
                            program,
                            diagnostics,
                            cancelled: true,
                        };
                    }
                    Err(e) => fail!(
                        "operation {}: adaptive layer clearing failed at Z{:.3}: {e:?}",
                        op.id,
                        layer.z
                    ),
                };
                if cut_regions.is_empty() {
                    continue;
                }
                // Validate the emitted moves independently, accounting for the
                // arc-refitting budget included in the model envelope.
                match crate::adaptive_safety::certify_layer(
                    &layer_program,
                    &layer.protected,
                    emitted_radius,
                    layer.z,
                    env.heights.clearance,
                    cancel,
                ) {
                    Ok(()) => {}
                    Err(crate::adaptive_safety::SafetyFailure::Cancelled) => {
                        return StrategyResult {
                            program,
                            diagnostics,
                            cancelled: true,
                        };
                    }
                    Err(crate::adaptive_safety::SafetyFailure::Motion { index, reason }) => {
                        diagnostics.push(Diagnostic::error(format!(
                            "operation {}: 3D Adaptive safety check rejected move {} at Z{:.4}: {reason}",
                            op.id, index + 1, layer.z,
                        )).with_code(crate::DiagnosticCode::InvalidMotion).for_op(op.id));
                        return StrategyResult {
                            diagnostics,
                            ..Default::default()
                        };
                    }
                }
                used_concentric |= concentric;
                cleared_levels += 1;
                // Track only the regions with emitted cuts. A skipped change
                // can accumulate into a later pass.
                cleared_slice = match cam_geo::union(&cleared_slice, &cut_regions) {
                    Ok(regions) => regions,
                    Err(e) => fail!(
                        "operation {}: could not track cleared Adaptive regions: {e}",
                        op.id
                    ),
                };
                if !cancel.is_cancelled() {
                    publish(&layer_program);
                }
                program.extend(layer_program);
            }
        }
        if cleared_levels == 0 {
            fail!(
                "operation {}: no accessible stock was clearable in the selected machining region below the stock top",
                op.id
            );
        }
        if used_concentric {
            diagnostics.push(Diagnostic::warning(format!(
                "operation {}: some 3D Adaptive regions use concentric clearing where the frontier fragments or cannot be certified; optimal load is not a maximum engagement guarantee in these regions",
                op.id,
            )).for_op(op.id));
        }
        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn clear_region(
        &self,
        program: &mut Program,
        region: &Polygon,
        layer: &cam_geometry3d::AdaptiveLayer,
        env: &JobEnv,
        tool: &cam_model::Tool,
        safety_radius: f64,
        cancel: &CancelToken,
    ) -> Result<ClearResult, RingsError> {
        let op = &self.op;
        let z = layer.z;
        let protected = &layer.protected;
        let spacing = op.params.optimal_load;
        let radius = tool.radius();
        let guard = offset(std::slice::from_ref(region), -radius, JoinStyle::Round)
            .map_err(|e| RingsError::Offset(format!("cutter-centre entry guard: {e}")))?;
        if guard.is_empty() {
            return Ok(ClearResult {
                paths: 0,
                concentric: false,
            });
        }
        let fine_pitch = op.params.fine_stepdown.min(op.params.stepdown);
        let arc = |radius| {
            if radius > 0.0 {
                Lead::Arc { radius }
            } else {
                Lead::None
            }
        };
        let job = crate::clearing::ClearJob {
            lift_height: op.params.lift_height,
            keep_tool_down: false,
            id: op.id,
            simplification_tolerance: op.params.simplification_tolerance(),
            radius,
            finish: 0.0,
            first: radius,
            spacing,
            clearing: Clearing {
                engagement: op.params.optimal_load,
                climb: op.params.climb,
            },
            spindle: env.spindle,
            plunge: Plunge::Helix {
                radius: (radius * 0.5).max(0.1),
                pitch: fine_pitch,
            },
            feed: op.params.feed,
            plunge_feed: op.params.plunge_feed,
            entry_feeds: op.entry_feeds,
            lead_overlap: 0.0,
            lead_in: arc(op.params.lead_in_radius),
            lead_out: arc(op.params.lead_out_radius),
            start: op.params.start,
            end: op.params.end,
            guard: &guard,
            helix: None,
        };
        crate::clearing::clear_validated(
            program,
            region,
            &job,
            &env.heights,
            &[z],
            cancel,
            Some(tool),
            |candidate| {
                crate::adaptive_safety::certify_layer(
                    candidate,
                    protected,
                    safety_radius,
                    z,
                    env.heights.clearance,
                    cancel,
                )
                .is_ok()
            },
        )
    }
}

/// A cutter touching newly exposed stock can reach at most one diameter into
/// the previously cleared slice. Keep that overlap so the cropped target still
/// admits every valid cutter position that can reach the new material.
fn fine_cutting_regions(
    region: &Polygon,
    rest: &[Polygon],
    radius: f64,
    tolerance: f64,
) -> Result<Vec<Polygon>, cam_geo::GeoError> {
    let band = offset(
        rest,
        2.0 * radius + tolerance + 4.0 * cam_geo::GRID_MM,
        JoinStyle::Round,
    )?;
    cam_geo::intersection(std::slice::from_ref(region), &band)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_cldata::{SpindleDir, Step};
    use cam_geo::{Containment, JoinStyle};
    use cam_model::{Heights, Tool, ToolKind};

    fn fixture() -> (AdaptiveStrategy, Tool) {
        let tool = Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flute_length: 20.0,
            flutes: 3,
            kind: ToolKind::EndMill,
            ..Default::default()
        };
        let part = Polygon::new(Contour::new(vec![
            Point::new(10., 10.),
            Point::new(20., 10.),
            Point::new(20., 20.),
            Point::new(10., 20.),
        ]))
        .unwrap();
        let geometry = AdaptiveGeometry {
            layers: vec![
                cam_geometry3d::AdaptiveLayer {
                    z: 8.,
                    protected: vec![part.clone()],
                    body_domains: vec![],
                    face_domains: vec![],
                },
                cam_geometry3d::AdaptiveLayer {
                    z: 4.,
                    protected: vec![part],
                    body_domains: vec![],
                    face_domains: vec![],
                },
            ],
            fingerprint: "fixture".into(),
            model_top: 10.0,
            error_bound: 0.02,
        };
        let mut op = AdaptiveOp {
            parameter_expressions: Default::default(),
            id: 3,
            tool: 1,
            spindle_rpm: 6000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            params: Default::default(),
        };
        op.params.optimal_load = 2.0;
        op.params.stepdown = 4.0;
        op.params.fine_stepdown = 1.0;
        op.params.feed = 300.;
        op.params.plunge_feed = 100.;
        (
            AdaptiveStrategy::new(
                op,
                StockToLeave {
                    enabled: true,
                    radial: 0.5,
                    axial: 0.25,
                },
                geometry,
            ),
            tool,
        )
    }

    #[test]
    fn previews_checked_regions_in_program_order() {
        let (mut strategy, tool) = fixture();
        strategy.op.params.climb = false;
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [30., 30.])),
            spindle: SpindleDir::Cw,
        };
        let cancel = CancelToken::new();
        let mut preview = Program::new();
        let result = strategy.compute_with_preview(&env, &cancel, &mut |region| {
            assert!(region.validate().is_ok());
            preview.extend(region.clone());
        });
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(!preview.is_empty());
        assert_eq!(preview, result.program);

        let mut regions = 0;
        let result = strategy.compute_with_preview(&env, &cancel, &mut |_| {
            regions += 1;
            cancel.cancel();
        });
        assert!(result.cancelled);
        assert_eq!(regions, 1);
    }

    #[test]
    fn lift_height_does_not_raise_concentric_joins_into_uncut_stock() {
        let (mut strategy, tool) = fixture();
        strategy.geometry.layers.truncate(1);
        strategy.op.params.climb = false; // Exercise concentric layer links.
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [30., 30.])),
            spindle: SpindleDir::Cw,
        };
        let baseline = strategy.compute(&env, &CancelToken::new());
        assert!(!baseline.has_errors(), "{:?}", baseline.diagnostics);
        strategy.op.params.lift_height = 2.;
        let lifted = strategy.compute(&env, &CancelToken::new());
        assert!(!lifted.has_errors(), "{:?}", lifted.diagnostics);
        assert!(lifted.program.validate().is_ok());
        assert_eq!(lifted.program, baseline.program);
        for lift in [-1., f64::NAN, f64::INFINITY] {
            strategy.op.params.lift_height = lift;
            let result = strategy.compute(&env, &CancelToken::new());
            assert!(result.has_errors());
            assert!(result.program.steps().is_empty());
        }
    }

    #[test]
    fn selected_entry_and_exit_change_the_path_without_bypassing_layer_safety() {
        let (mut strategy, tool) = fixture();
        strategy.geometry.layers.truncate(1);
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [30., 30.])),
            spindle: SpindleDir::Cw,
        };
        let baseline = strategy.compute(&env, &CancelToken::new());
        assert!(!baseline.has_errors(), "{:?}", baseline.diagnostics);
        // Pick on the model, so the preference must be resolved outside the
        // protected island rather than used as a literal plunge coordinate.
        strategy.op.params.start = Some([15., 12.]);
        let entry_only = strategy.compute(&env, &CancelToken::new());
        assert!(!entry_only.has_errors(), "{:?}", entry_only.diagnostics);
        strategy.op.params.end = Some([5., 25.]);
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert_ne!(baseline.program, result.program);
        assert!(result.program.validate().is_ok());
        assert_ne!(entry_only.program, result.program);
        let exit = result
            .program
            .steps()
            .iter()
            .rev()
            .find_map(|s| match s {
                Step::Rapid { to, .. } => Some(*to),
                _ => None,
            })
            .unwrap();
        assert!((exit.x - 5.).hypot(exit.y - 25.) < 3.0, "{exit:?}");
        // compute() certifies the new exit as well as every other emitted move.
    }

    #[test]
    fn adjacent_face_domains_clear_the_shared_edge() {
        let (mut strategy, tool) = fixture();
        let rectangle = |x0, x1| {
            Polygon::new(Contour::new(vec![
                Point::new(x0, 0.),
                Point::new(x1, 0.),
                Point::new(x1, 20.),
                Point::new(x0, 20.),
            ]))
            .unwrap()
        };
        strategy.op.params.climb = false;
        strategy.stock_to_leave = StockToLeave::default();
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [24., 20.])),
            spindle: SpindleDir::Cw,
        };
        for (gap, expected_z) in [(0., 8.), (1., 10.)] {
            strategy.geometry.layers = vec![cam_geometry3d::AdaptiveLayer {
                z: 8.,
                protected: vec![],
                body_domains: vec![],
                face_domains: vec![rectangle(0., 12.05), rectangle(12.05 + gap, 24.)],
            }];
            let result = strategy.compute(&env, &CancelToken::new());
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
            let sim = cam_sim::simulate(
                &result.program,
                [0., 0., 0.],
                [24., 20., 10.],
                &cam_sim::SimOptions {
                    resolution: 0.1,
                    tool_radius: tool.radius(),
                },
                &[],
            );
            assert!(sim.is_clean(), "{:?}", sim.collisions);
            assert_eq!(sim.field.sample(12.05 + gap / 2., 10.), expected_z);
        }
    }

    #[test]
    fn fine_passes_only_revisit_regions_with_newly_exposed_material() {
        let (mut strategy, tool) = fixture();
        let rectangle = |x: f64| {
            Polygon::new(Contour::new(vec![
                Point::new(x, 0.),
                Point::new(x + 12., 0.),
                Point::new(x + 12., 12.),
                Point::new(x, 12.),
            ]))
            .unwrap()
        };
        let left = rectangle(0.);
        let right = rectangle(30.);
        strategy.stock_to_leave = StockToLeave::default();
        strategy.geometry.layers = vec![
            cam_geometry3d::AdaptiveLayer {
                z: 8.,
                protected: vec![right.clone()],
                body_domains: vec![],
                face_domains: vec![left.clone(), right.clone()],
            },
            cam_geometry3d::AdaptiveLayer {
                z: 9.,
                protected: vec![],
                body_domains: vec![],
                face_domains: vec![left, right],
            },
        ];
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [42., 12.])),
            spindle: SpindleDir::Cw,
        };
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let mut coarse = 0;
        let mut fine = 0;
        for step in result.program.steps() {
            let point = match step {
                Step::Linear { to, tag, .. } if tag.kind == cam_cldata::MoveKind::Cutting => to,
                Step::Arc { end, tag, .. } if tag.kind == cam_cldata::MoveKind::Cutting => end,
                _ => continue,
            };
            if (point.z - 8.).abs() < 1e-9 {
                coarse += 1;
                assert!(point.x < 12.);
            } else if (point.z - 9.).abs() < 1e-9 {
                fine += 1;
                assert!(
                    point.x > 30.,
                    "fine pass repeats an unchanged region: {point:?}"
                );
            }
        }
        assert!(coarse > 0 && fine > 0);
    }

    #[test]
    fn fine_passes_localize_a_terrace_within_a_connected_pocket() {
        let (mut strategy, tool) = fixture();
        let wall = |x: f64| {
            Polygon::new(Contour::new(vec![
                Point::new(0., -2.),
                Point::new(x, -2.),
                Point::new(x, 42.),
                Point::new(0., 42.),
            ]))
            .unwrap()
        };
        strategy.op.params.climb = false;
        strategy.stock_to_leave = StockToLeave::default();
        strategy.geometry.layers = [(8., 30.), (9., 28.)]
            .into_iter()
            .map(|(z, x)| cam_geometry3d::AdaptiveLayer {
                z,
                protected: vec![wall(x)],
                body_domains: vec![],
                face_domains: vec![],
            })
            .collect();
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [90., 40.])),
            spindle: SpindleDir::Cw,
        };
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(result.program.validate().is_ok());
        let fine_cuts: Vec<_> = result
            .program
            .steps()
            .iter()
            .filter_map(|step| match step {
                Step::Linear { to, tag, .. } | Step::Arc { end: to, tag, .. }
                    if tag.kind == cam_cldata::MoveKind::Cutting && to.z == 9. =>
                {
                    Some(to)
                }
                _ => None,
            })
            .collect();
        assert!(fine_cuts.iter().any(|p| (p.x - 31.).abs() < 0.2));
        assert!(fine_cuts.iter().all(|p| p.x < 34.));
    }

    #[test]
    fn small_skipped_refinements_accumulate_against_the_last_cut() {
        let (mut strategy, tool) = fixture();
        let rectangle = |width: f64| {
            Polygon::new(Contour::new(vec![
                Point::new(0., 0.),
                Point::new(width, 0.),
                Point::new(width, 20.),
                Point::new(0., 20.),
            ]))
            .unwrap()
        };
        strategy.stock_to_leave = StockToLeave::default();
        strategy.geometry.layers = (0..4)
            .map(|i| cam_geometry3d::AdaptiveLayer {
                z: 8.0 + i as f64 * 0.25,
                protected: vec![rectangle(10.0 - i as f64 * 0.03)],
                body_domains: vec![],
                face_domains: vec![rectangle(30.)],
            })
            .collect();
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [30., 20.])),
            spindle: SpindleDir::Cw,
        };
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let mut depths = std::collections::BTreeSet::new();
        for step in result.program.steps() {
            let point = match step {
                Step::Linear { to, tag, .. } if tag.kind == cam_cldata::MoveKind::Cutting => to,
                Step::Arc { end, tag, .. } if tag.kind == cam_cldata::MoveKind::Cutting => end,
                _ => continue,
            };
            depths.insert((point.z * 1000.).round() as i64);
        }
        // Each 0.03 mm change is within the 0.08 mm remaining budget, but
        // three skipped changes together exceed it and must trigger a pass.
        assert_eq!(depths, std::collections::BTreeSet::from([8000, 8750]));
    }

    #[test]
    fn invalid_tolerances_fail_before_any_motion() {
        let (mut strategy, tool) = fixture();
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [30., 30.])),
            spindle: SpindleDir::Cw,
        };
        for value in [0.0, -0.001, f64::NAN, f64::INFINITY] {
            strategy.op.params.tolerance = value;
            let result = strategy.compute(&env, &CancelToken::new());
            assert!(result.has_errors(), "must reject tolerance {value}");
            assert!(result.program.steps().is_empty());
            assert!(result
                .diagnostics
                .iter()
                .any(|d| d.message.contains("tolerance must be finite and positive")));
        }
    }

    #[test]
    fn layer_path_stays_outside_cad_shadow_with_cutter_and_stock_allowance() {
        let (strategy, tool) = fixture();
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [30., 30.])),
            spindle: SpindleDir::Cw,
        };
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(!result.program.steps().is_empty());
        let protected = offset(
            &[Polygon::new(Contour::new(vec![
                Point::new(10., 10.),
                Point::new(20., 10.),
                Point::new(20., 20.),
                Point::new(10., 20.),
            ]))
            .unwrap()],
            tool.radius() + 0.5 + 0.02,
            JoinStyle::Round,
        )
        .unwrap();
        let mut cuts = 0;
        for step in result.program.steps() {
            let p = match step {
                Step::Linear { to, .. } => Some([to.x, to.y]),
                Step::Arc { end, .. } => Some([end.x, end.y]),
                _ => None,
            };
            if let Some([x, y]) = p {
                if matches!(step, Step::Linear { .. } | Step::Arc { .. }) {
                    cuts += 1;
                    assert!(
                        protected
                            .iter()
                            .all(|poly| poly.locate(Point::new(x, y)) == Containment::Outside),
                        "cut motion enters protected stock shadow at ({x},{y})"
                    );
                }
            }
        }
        assert!(cuts > 0, "expected emitted cutting moves");
    }

    #[test]
    fn selected_face_domain_contains_sampled_lines_and_arc_refits() {
        let (mut strategy, tool) = fixture();
        let face = Polygon::new(Contour::new(vec![
            Point::new(0., 0.),
            Point::new(60., 0.),
            Point::new(60., 60.),
            Point::new(0., 60.),
        ]))
        .unwrap();
        for layer in &mut strategy.geometry.layers {
            layer.body_domains.clear();
            layer.face_domains = vec![face.clone()];
        }
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [60., 60.])),
            spindle: SpindleDir::Cw,
        };
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let center_domain = offset(
            std::slice::from_ref(&face),
            -(tool.radius() + strategy.geometry.error_bound + 2.0 * cam_geo::GRID_MM),
            JoinStyle::Round,
        )
        .unwrap();
        let mut current = None;
        let mut sampled = 0usize;
        let mut arc_samples = 0usize;
        for step in result.program.steps() {
            match step {
                Step::Rapid { to, .. } => current = Some(*to),
                Step::Linear { to, .. } => {
                    if let Some(start) = current {
                        for i in 1..=20 {
                            let t = i as f64 / 20.0;
                            let p = Point::new(
                                start.x + (to.x - start.x) * t,
                                start.y + (to.y - start.y) * t,
                            );
                            assert!(
                                center_domain
                                    .iter()
                                    .any(|poly| poly.locate(p) != Containment::Outside),
                                "linear cut leaves selected face center domain at {p:?}"
                            );
                            sampled += 1;
                        }
                    }
                    current = Some(*to);
                }
                Step::Arc {
                    end, center, dir, ..
                } => {
                    if let Some(start) = current {
                        let start_angle = (start.y - center.y).atan2(start.x - center.x);
                        let end_angle = (end.y - center.y).atan2(end.x - center.x);
                        let raw = end_angle - start_angle;
                        let sweep = match dir {
                            cam_cldata::ArcDir::Ccw => raw.rem_euclid(std::f64::consts::TAU),
                            cam_cldata::ArcDir::Cw => -(-raw).rem_euclid(std::f64::consts::TAU),
                        };
                        for i in 1..=40 {
                            let t = i as f64 / 40.0;
                            let angle = start_angle + sweep * t;
                            let p = Point::new(
                                center.x
                                    + (start.x - center.x).hypot(start.y - center.y) * angle.cos(),
                                center.y
                                    + (start.x - center.x).hypot(start.y - center.y) * angle.sin(),
                            );
                            assert!(
                                center_domain
                                    .iter()
                                    .any(|poly| poly.locate(p) != Containment::Outside),
                                "arc-refit cut leaves selected face center domain at {p:?}"
                            );
                            sampled += 1;
                            arc_samples += 1;
                        }
                    }
                    current = Some(*end);
                }
                _ => {}
            }
        }
        assert!(sampled > 0, "expected sampled cutting moves");
        assert!(arc_samples > 0, "expected sampled arc-refit moves");
    }

    #[test]
    fn containment_and_signed_offset_shift_emitted_cutter_centers_as_documented() {
        let run = |containment, additional_offset| {
            let (mut strategy, tool) = fixture();
            let face = Polygon::new(Contour::new(vec![
                Point::new(10., 10.),
                Point::new(30., 10.),
                Point::new(30., 30.),
                Point::new(10., 30.),
            ]))
            .unwrap();
            let protected = Polygon::new(Contour::new(vec![
                Point::new(18., 18.),
                Point::new(22., 18.),
                Point::new(22., 22.),
                Point::new(18., 22.),
            ]))
            .unwrap();
            strategy.geometry.layers.truncate(1);
            for layer in &mut strategy.geometry.layers {
                layer.face_domains = vec![face.clone()];
                layer.body_domains.clear();
                layer.protected = vec![protected.clone()];
            }
            strategy.op.params.containment = containment;
            strategy.op.params.additional_offset = additional_offset;
            let env = JobEnv {
                heights: Heights::new(15., 12., 10.),
                tools: std::slice::from_ref(&tool),
                stock: Some(([0., 0.], [40., 40.])),
                spindle: SpindleDir::Cw,
            };
            let result = strategy.compute(&env, &CancelToken::new());
            assert!(!result.has_errors(), "{:?}", result.diagnostics);
            let x_values: Vec<_> = result
                .program
                .steps()
                .iter()
                .filter_map(|step| match step {
                    Step::Linear { to, .. } => Some(to.x),
                    Step::Arc { end, .. } => Some(end.x),
                    _ => None,
                })
                .collect();
            assert!(!x_values.is_empty());
            (
                *x_values.iter().min_by(|a, b| a.total_cmp(b)).unwrap(),
                *x_values.iter().max_by(|a, b| a.total_cmp(b)).unwrap(),
            )
        };
        use cam_model::AdaptiveContainment::{Center, Inside, Outside};
        let inside = run(Inside, 0.0);
        let inside_offset = run(Inside, 1.0);
        let inside_negative = run(Inside, -1.0);
        let center = run(Center, 0.0);
        let center_offset = run(Center, 1.0);
        let center_negative = run(Center, -1.0);
        let outside = run(Outside, 0.0);
        let outside_offset = run(Outside, 1.0);
        assert!(
            inside_offset.0 > inside.0 + 0.8,
            "inside positive offset should shrink the region: {inside:?} -> {inside_offset:?}"
        );
        assert!(
            inside_negative.0 < inside.0 - 0.8,
            "inside negative offset should expand the region: {inside:?} -> {inside_negative:?}"
        );
        assert!(
            center_offset.0 < center.0 - 0.8,
            "center positive offset should expand the region: {center:?} -> {center_offset:?}"
        );
        assert!(
            center_negative.0 > center.0 + 0.8,
            "center negative offset should shrink the region: {center:?} -> {center_negative:?}"
        );
        assert!(
            outside_offset.0 < outside.0 - 0.8,
            "outside positive offset should expand the region: {outside:?} -> {outside_offset:?}"
        );
        assert!(
            inside.0 > center.0 && center.0 > outside.0,
            "tool center should move outward by containment mode: {inside:?}, {center:?}, {outside:?}"
        );
    }

    #[test]
    fn clearance_below_an_unselected_setup_body_rejects_all_motion() {
        let (mut strategy, tool) = fixture();
        strategy.geometry.model_top = 16.;
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [30., 30.])),
            spindle: SpindleDir::Cw,
        };
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(result.has_errors());
        assert!(result.program.is_empty());
        assert!(result.diagnostics.iter().any(|d| {
            d.message
                .contains("clearance must be above every setup body")
        }));
    }

    #[test]
    fn outside_entries_keep_the_entire_cutter_clear_of_a_sloped_part() {
        use cam_cldata::{ArcDir, MoveKind, Point3};
        let (mut strategy, tool) = fixture();
        let rectangle = |x0, y0, x1, y1| {
            Polygon::new(Contour::new(vec![
                Point::new(x0, y0),
                Point::new(x1, y0),
                Point::new(x1, y1),
                Point::new(x0, y1),
            ]))
            .unwrap()
        };
        strategy.stock_to_leave = StockToLeave::default();
        strategy.op.params.containment = cam_model::AdaptiveContainment::Outside;
        // The solid's top is the plane z=y+40 over the front slope, then z=20.
        // Its side edges at x=+/-50 leave narrow runout strips for entry.
        strategy.geometry.layers = [19., 15., 10.5]
            .into_iter()
            .map(|z| cam_geometry3d::AdaptiveLayer {
                z,
                protected: vec![rectangle(
                    -50.,
                    z - 40. - strategy.geometry.error_bound,
                    50.,
                    30.,
                )],
                body_domains: vec![],
                face_domains: vec![rectangle(-50., -30., 50., -20.)],
            })
            .collect();
        let env = JobEnv {
            heights: Heights::new(36., 26., 21.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([-51., -31.], [51., 31.])),
            spindle: SpindleDir::Cw,
        };
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let mut current: Option<Point3> = None;
        let mut side_entries = 0;
        let mut samples = 0;
        for step in result.program.steps() {
            let mut check = |p: Point3| {
                // A cutter centred outside a side edge can still gouge with its
                // flank. The disc's highest Y inside the solid determines the
                // exact maximum surface height beneath this flat end mill.
                let dx = (p.x.abs() - 50.).max(0.);
                let reach = (tool.radius().powi(2) - dx * dx).max(0.).sqrt();
                if dx < tool.radius() && p.y + reach > -30. && p.y - reach < 30. {
                    let surface = (p.y + reach).min(-20.) + 40.;
                    assert!(
                        p.z >= surface - 0.02,
                        "cutter gouges slope by {:.4} mm at {p:?}: {step:?}",
                        surface - p.z
                    );
                }
                samples += 1;
            };
            match step {
                Step::Rapid { to, .. } | Step::Linear { to, .. } => {
                    if let Some(a) = current {
                        let n = ((to.x - a.x).hypot(to.y - a.y).hypot(to.z - a.z) / 0.05)
                            .ceil()
                            .max(1.) as usize;
                        for i in 0..=n {
                            let t = i as f64 / n as f64;
                            check(Point3::new(
                                a.x + (to.x - a.x) * t,
                                a.y + (to.y - a.y) * t,
                                a.z + (to.z - a.z) * t,
                            ));
                        }
                    }
                    if matches!(step, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge)
                        && to.x.abs() > 50.
                        && to.x.abs() < 50. + tool.radius()
                    {
                        side_entries += 1;
                    }
                    current = Some(*to);
                }
                Step::Arc {
                    end, center, dir, ..
                } => {
                    if let Some(a) = current {
                        let radius = (a.x - center.x).hypot(a.y - center.y);
                        let start = (a.y - center.y).atan2(a.x - center.x);
                        let raw = (end.y - center.y).atan2(end.x - center.x) - start;
                        let sweep = match dir {
                            ArcDir::Ccw => raw.rem_euclid(std::f64::consts::TAU),
                            ArcDir::Cw => -(-raw).rem_euclid(std::f64::consts::TAU),
                        };
                        let n = (radius * sweep.abs() / 0.05).ceil().max(1.) as usize;
                        for i in 0..=n {
                            let t = i as f64 / n as f64;
                            let angle = start + sweep * t;
                            check(Point3::new(
                                center.x + radius * angle.cos(),
                                center.y + radius * angle.sin(),
                                a.z + (end.z - a.z) * t,
                            ));
                        }
                    }
                    current = Some(*end);
                }
                _ => {}
            }
        }
        assert!(samples > 0);
        assert!(
            side_entries > 0,
            "exercise entries whose cutter overlaps the side edge"
        );
    }

    #[test]
    fn containment_at_stock_edges_allows_selected_face_and_body_runout() {
        use cam_model::AdaptiveContainment::{Center, Inside, Outside};
        for body_selection in [false, true] {
            for (containment, extra, expected_min) in [
                (Inside, 0.0, 3.0),
                (Center, 0.0, 0.0),
                (Outside, 0.0, -3.0),
                (Outside, 1.0, -4.0),
            ] {
                let (mut strategy, tool) = fixture();
                let boundary = Polygon::new(Contour::new(vec![
                    Point::new(0., 0.),
                    Point::new(30., 0.),
                    Point::new(30., 30.),
                    Point::new(0., 30.),
                ]))
                .unwrap();
                strategy.geometry.layers.truncate(1);
                let layer = &mut strategy.geometry.layers[0];
                if body_selection {
                    layer.body_domains = vec![boundary];
                } else {
                    layer.face_domains = vec![boundary];
                }
                strategy.op.params.containment = containment;
                strategy.op.params.additional_offset = extra;
                let env = JobEnv {
                    heights: Heights::new(15., 12., 10.),
                    tools: std::slice::from_ref(&tool),
                    stock: Some(([0., 0.], [30., 30.])),
                    spindle: SpindleDir::Cw,
                };
                let result = strategy.compute(&env, &CancelToken::new());
                assert!(!result.has_errors(), "{:?}", result.diagnostics);
                let mut min_x = f64::INFINITY;
                let mut max_x = f64::NEG_INFINITY;
                for step in result.program.steps() {
                    let point = match step {
                        Step::Linear { to, tag, .. }
                            if tag.kind == cam_cldata::MoveKind::Cutting =>
                        {
                            to
                        }
                        Step::Arc { end, tag, .. } if tag.kind == cam_cldata::MoveKind::Cutting => {
                            end
                        }
                        _ => continue,
                    };
                    min_x = min_x.min(point.x);
                    max_x = max_x.max(point.x);
                }
                assert!(
                    (min_x - expected_min).abs() < 0.15,
                    "{containment:?}, offset {extra}, body={body_selection}: min X {min_x}, expected {expected_min}"
                );
                assert!(
                    (max_x - (30.0 - expected_min)).abs() < 0.15,
                    "{containment:?}, offset {extra}, body={body_selection}: max X {max_x}"
                );
            }
        }
    }

    #[test]
    fn selected_body_subset_clips_output_to_its_local_runout_domain() {
        let (mut strategy, tool) = fixture();
        let body = Polygon::new(Contour::new(vec![
            Point::new(20., 20.),
            Point::new(30., 20.),
            Point::new(30., 30.),
            Point::new(20., 30.),
        ]))
        .unwrap();
        for layer in &mut strategy.geometry.layers {
            layer.body_domains = vec![body.clone()];
            layer.face_domains.clear();
        }
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [60., 60.])),
            spindle: SpindleDir::Cw,
        };
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        let expanded_domain = offset(
            std::slice::from_ref(&body),
            2.0 * tool.diameter - strategy.geometry.error_bound - 2.0 * cam_geo::GRID_MM,
            JoinStyle::Round,
        )
        .unwrap();
        let center_stays_in_domain = |point: Point| {
            expanded_domain
                .iter()
                .any(|poly| poly.locate(point) != Containment::Outside)
        };
        let mut current = None;
        let mut sampled = 0;
        for step in result.program.steps() {
            match step {
                Step::Rapid { to, .. } => current = Some(*to),
                Step::Linear { to, .. } => {
                    if let Some(start) = current {
                        for i in 1..=20 {
                            let t = i as f64 / 20.0;
                            let point = Point::new(
                                start.x + (to.x - start.x) * t,
                                start.y + (to.y - start.y) * t,
                            );
                            assert!(
                                center_stays_in_domain(point),
                                "linear cut leaves selected body domain at {point:?}"
                            );
                            sampled += 1;
                        }
                    }
                    current = Some(*to);
                }
                Step::Arc {
                    end, center, dir, ..
                } => {
                    if let Some(start) = current {
                        let start_angle = (start.y - center.y).atan2(start.x - center.x);
                        let raw = (end.y - center.y).atan2(end.x - center.x) - start_angle;
                        let sweep = match dir {
                            cam_cldata::ArcDir::Ccw => raw.rem_euclid(std::f64::consts::TAU),
                            cam_cldata::ArcDir::Cw => -(-raw).rem_euclid(std::f64::consts::TAU),
                        };
                        let radius = (start.x - center.x).hypot(start.y - center.y);
                        for i in 1..=40 {
                            let angle = start_angle + sweep * i as f64 / 40.0;
                            let point = Point::new(
                                center.x + radius * angle.cos(),
                                center.y + radius * angle.sin(),
                            );
                            assert!(
                                center_stays_in_domain(point),
                                "arc-refit cut leaves selected body domain at {point:?}"
                            );
                            sampled += 1;
                        }
                    }
                    current = Some(*end);
                }
                _ => {}
            }
        }
        assert!(
            sampled > 0,
            "expected emitted cutting motion within selected body domain"
        );
    }

    #[test]
    fn protected_face_and_air_only_layers_do_not_report_a_successful_cut() {
        let (mut strategy, tool) = fixture();
        let face = Polygon::new(Contour::new(vec![
            Point::new(10., 10.),
            Point::new(20., 10.),
            Point::new(20., 20.),
            Point::new(10., 20.),
        ]))
        .unwrap();
        for layer in &mut strategy.geometry.layers {
            layer.body_domains.clear();
            layer.face_domains = vec![face.clone()];
        }
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [30., 30.])),
            spindle: SpindleDir::Cw,
        };
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(result.has_errors());
        assert!(result
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.message.contains("no accessible stock")));
        assert!(!result
            .program
            .steps()
            .iter()
            .any(|step| matches!(step, Step::Linear { .. } | Step::Arc { .. })));

        strategy
            .geometry
            .layers
            .iter_mut()
            .for_each(|layer| layer.z += 10.);
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(result.has_errors());
        assert!(result
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.message.contains("no accessible stock")));
        assert!(!result
            .program
            .steps()
            .iter()
            .any(|step| matches!(step, Step::Linear { .. } | Step::Arc { .. })));
    }

    #[test]
    fn cancellation_and_missing_stock_fail_explicitly() {
        let (strategy, tool) = fixture();
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [30., 30.])),
            spindle: SpindleDir::Cw,
        };
        let token = CancelToken::new();
        token.cancel();
        assert!(strategy.compute(&env, &token).cancelled);
        let no_stock = JobEnv { stock: None, ..env };
        assert!(strategy
            .compute(&no_stock, &CancelToken::new())
            .has_errors());
    }

    #[test]
    fn warns_and_generates_for_tools_unsuited_to_side_milling_the_full_depth() {
        let (strategy, mut tool) = fixture();
        tool.kind = ToolKind::Drill {
            point_angle_deg: 118.,
        };
        let env = JobEnv {
            heights: Heights::new(15., 12., 10.),
            tools: std::slice::from_ref(&tool),
            stock: Some(([0., 0.], [30., 30.])),
            spindle: SpindleDir::Cw,
        };
        let result = strategy.compute(&env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(!result.program.is_empty());
        assert!(result
            .diagnostics
            .iter()
            .any(|d| d.severity == crate::Severity::Warning && d.message.contains("drill bit")));

        let short_flute = Tool {
            flute_length: 2.,
            kind: ToolKind::EndMill,
            ..tool.clone()
        };
        let short_flute_env = JobEnv {
            tools: std::slice::from_ref(&short_flute),
            ..env
        };
        let result = strategy.compute(&short_flute_env, &CancelToken::new());
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        assert!(!result.program.is_empty());
        assert!(
            result
                .diagnostics
                .iter()
                .any(|d| d.severity == crate::Severity::Warning
                    && d.message.contains("length of cut"))
        );
    }
}
