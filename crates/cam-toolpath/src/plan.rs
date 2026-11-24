//! Job planning: run every operation's strategy and assemble the results into a
//! single, machine-ready CL-data program.
//!
//! This is the thin layer that wraps the pure per-operation strategies with the
//! job-level machine control — tool changes, spindle, coolant — and splices the
//! operation fragments together in order.

use cam_cldata::{Coolant, MoveKind, Point3, Program, SpindleDir, Step, Tag};
use cam_model::{Document, Operation};

use crate::{
    AdaptiveStrategy, CancelToken, CarveStrategy, ChamferStrategy, Diagnostic, DrillStrategy,
    EngraveStrategy, FaceStrategy, JobEnv, PocketStrategy, ProfileStrategy, Strategy,
    StrategyResult, ThreadStrategy,
};

/// Whether all requested operations finished successfully. A default result is
/// incomplete until the planner has established otherwise.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum JobCompletion {
    /// No completed result has been established yet.
    #[default]
    Incomplete,
    /// All requested operations finished without error diagnostics.
    Complete,
    /// A cancellation request or strategy cancellation stopped the job.
    Cancelled,
    /// At least one operation reported an error.
    Failed,
}

/// A planned job, including the completion state required before export.
/// Cancelled/failed jobs may retain completed operations for inspection; their
/// program is not a complete manufacturing job and must not be posted.
#[derive(Clone, Debug, Default, PartialEq)]
#[must_use = "check job completion before posting the program"]
pub struct JobResult {
    pub program: Program,
    pub diagnostics: Vec<Diagnostic>,
    pub completion: JobCompletion,
}

/// Assemble a whole-job [`Program`] from a [`Document`].
///
/// Each operation is turned into its strategy, computed against the setup's
/// heights and tools, and its motions are spliced between machine transitions.
/// Each operation's cooling request surrounds its fragment; shutdown stops the spindle. A
/// tool change is inserted whenever the tool number changes between operations.
///
/// Diagnostics from every operation are collected; if any operation errors, its
/// motions are omitted but planning continues so the caller sees all problems at
/// once. Returns the program, combined diagnostics and explicit completion state.
/// Only [`JobCompletion::Complete`] is eligible for export. An intentionally
/// empty job can be complete; cancellation is never inferred from program size.
pub fn build_job(
    doc: &Document,
    spindle_rpm: f64,
    dir: SpindleDir,
    stock: Option<([f64; 2], [f64; 2])>,
    tool_change_height: f64,
    cancel: &CancelToken,
) -> JobResult {
    build_job_with_compute(
        doc,
        spindle_rpm,
        dir,
        stock,
        tool_change_height,
        cancel,
        |op, env, token, stock_to_leave, geometry, heights| {
            compute_with_stock(op, env, token, stock_to_leave, geometry, heights)
        },
    )
}

/// Immutable planning configuration, shared by synchronous and worker callers.
pub struct PlanRequest<'a> {
    pub document: &'a Document,
    pub spindle_rpm: f64,
    pub dir: SpindleDir,
    pub stock: Option<([f64; 2], [f64; 2])>,
    pub tool_change_height: f64,
}

/// Reuse only matching operation fragments. Linking is always rebuilt in order.
pub fn build_job_cached(
    request: PlanRequest<'_>,
    cancel: &CancelToken,
    cache: &mut crate::OperationCache,
    mut progress: impl FnMut(u32),
) -> JobResult {
    build_job_cached_with_progress(request, cancel, cache, |operation| {
        if let CalculationProgress::Started(id) = operation {
            progress(id);
        }
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalculationProgress {
    Started(u32),
    Finished { operation: u32, success: bool },
}

/// Report the start and end of actual fragment computations; cache hits do not
/// represent recalculation and therefore do not produce progress events.
pub fn build_job_cached_with_progress(
    request: PlanRequest<'_>,
    cancel: &CancelToken,
    cache: &mut crate::OperationCache,
    progress: impl FnMut(CalculationProgress),
) -> JobResult {
    build_job_cached_with_preview(request, cancel, cache, progress, None)
}

/// Publish checked Adaptive regions for display while computing a fragment.
/// These regions precede job linking and verification. Cache hits produce no
/// partial regions.
pub fn build_job_cached_with_preview(
    request: PlanRequest<'_>,
    cancel: &CancelToken,
    cache: &mut crate::OperationCache,
    mut progress: impl FnMut(CalculationProgress),
    mut preview: Option<&mut dyn FnMut(&Program)>,
) -> JobResult {
    cache.reset_stats();
    build_job_with_compute(
        request.document,
        request.spindle_rpm,
        request.dir,
        request.stock,
        request.tool_change_height,
        cancel,
        |op, env, token, stock_to_leave, geometry, heights| {
            let mut geometry_fingerprint = geometry
                .bindings
                .get(&op.id())
                .map(|binding| geometry.binding_fingerprint(binding));
            if let Some(index) = request.document.rotary_indexes.get(&op.id()) {
                geometry_fingerprint = Some(format!(
                    "{}:index:{index:?}",
                    geometry_fingerprint.unwrap_or_default()
                ));
            }
            if let Some(wrap) = op.rotary_wrap() {
                geometry_fingerprint = Some(format!(
                    "{}:wrap:{}:periodic:{}",
                    geometry_fingerprint.unwrap_or_default(),
                    crate::ROTARY_WRAP_REVISION,
                    wrap.periodic
                ));
            }
            if let Operation::Pocket(p) = op {
                if p.stock_contours.enabled {
                    let model_dependencies: Vec<_> = geometry
                        .instances
                        .iter()
                        .map(|instance| {
                            (
                                instance.id,
                                instance.asset_id,
                                instance.body_id,
                                instance.placement,
                                geometry
                                    .asset(instance.asset_id)
                                    .map(|asset| asset.revision),
                            )
                        })
                        .collect();
                    geometry_fingerprint = Some(format!(
                        "{}:model:{model_dependencies:?}",
                        geometry_fingerprint.unwrap_or_default(),
                    ));
                }
                if let Some(source) = p
                    .stock_contours
                    .source
                    .as_ref()
                    .filter(|_| p.stock_contours.enabled)
                {
                    geometry_fingerprint = Some(format!(
                        "{}:stock:{}",
                        geometry_fingerprint.unwrap_or_default(),
                        geometry.binding_fingerprint(source)
                    ));
                }
            }
            cache.compute_with_dependencies(
                op,
                env,
                token,
                crate::cache::CacheExtras {
                    stock_to_leave,
                    geometry_fingerprint,
                    cutting_heights: heights,
                },
                || {
                    progress(CalculationProgress::Started(op.id()));
                    let result = compute_with_stock_and_preview(
                        op,
                        env,
                        token,
                        stock_to_leave,
                        geometry,
                        heights,
                        &mut |region| {
                            let Some(preview) = preview.as_deref_mut() else {
                                return;
                            };
                            if let Some(index) = request.document.rotary_indexes.get(&op.id()) {
                                if let Ok(indexed) = crate::index_program(region, index, token) {
                                    preview(&indexed);
                                }
                            } else {
                                preview(region);
                            }
                        },
                    );
                    progress(CalculationProgress::Finished {
                        operation: op.id(),
                        success: !result.cancelled && !result.has_errors() && !token.is_cancelled(),
                    });
                    result
                },
            )
        },
    )
}

fn build_job_with_compute(
    doc: &Document,
    spindle_rpm: f64,
    dir: SpindleDir,
    stock: Option<([f64; 2], [f64; 2])>,
    tool_change_height: f64,
    cancel: &CancelToken,
    mut compute: impl FnMut(
        &Operation,
        &JobEnv,
        &CancelToken,
        cam_model::StockToLeave,
        &cam_model::GeometryState,
        Option<cam_model::CuttingHeights>,
    ) -> StrategyResult,
) -> JobResult {
    let Some(setup) = doc.active_setup() else {
        return JobResult {
            program: Program::new(),
            diagnostics: vec![Diagnostic::error(
                "Create a setup before generating toolpaths.",
            )],
            completion: JobCompletion::Failed,
        };
    };
    let fixtures = match setup.fixture_envelopes(&doc.geometry) {
        Ok(fixtures) => fixtures,
        Err(reason) => {
            return JobResult {
                diagnostics: vec![Diagnostic::error(reason).for_setup(setup.id)],
                completion: JobCompletion::Failed,
                ..Default::default()
            }
        }
    };
    let fixture_top = fixtures
        .iter()
        .map(|f| f.max[2] + 0.01)
        .fold(f64::NEG_INFINITY, f64::max);
    // Strategy inputs contain machining models only. Fixture envelopes are
    // checked independently against the fully assembled program below.
    let mut machining_document;
    let doc = if fixtures.is_empty() {
        doc
    } else {
        machining_document = doc.clone();
        machining_document
            .geometry
            .instances
            .retain(|i| !setup.fixture_instances.contains(&i.id));
        &machining_document
    };
    let setup = doc.setup();
    let mut env = JobEnv {
        heights: setup.heights,
        tools: &setup.tools,
        stock,
        // The same direction the job's `Step::Spindle` will command, so a strategy's
        // climb/conventional choice and the spindle it is cut with cannot disagree.
        spindle: dir,
    };

    let mut program = Program::new();
    let mut diagnostics = Vec::new();
    let mut completion = JobCompletion::Complete;
    if let Err(reason) = setup.validate_patterns() {
        return JobResult {
            program,
            diagnostics: vec![
                Diagnostic::error(format!("invalid operation pattern: {reason}"))
                    .for_setup(setup.id),
            ],
            completion: JobCompletion::Failed,
        };
    }
    let surface_operation_ids: std::collections::HashSet<_> = setup
        .operations
        .iter()
        .filter(|operation| doc.operation_is_included(setup.id, operation.id()))
        .filter(|operation| {
            matches!(
                operation,
                Operation::Adaptive(_) | Operation::Parallel(_) | Operation::Spiral(_)
            )
        })
        .map(Operation::id)
        .collect();
    if let Some(pattern) = setup.patterns.iter().find(|pattern| {
        !pattern.suppressed
            && pattern.kind.instance_count().is_some_and(|count| count > 1)
            && pattern
                .members
                .iter()
                .any(|member| surface_operation_ids.contains(member))
    }) {
        return JobResult {
            program,
            diagnostics: vec![Diagnostic::error(
                "3D surface strategies cannot be repeated in an operation pattern because its path is resolved against the setup's actual model instances; create separate operations for each machining location",
            ).for_setup(setup.id).for_pattern(pattern.id, 1)],
            completion: JobCompletion::Failed,
        };
    }
    if let Err(reason) = setup.validate_stock_to_leave() {
        return JobResult {
            program,
            diagnostics: vec![Diagnostic::error(format!(
                "invalid stock-to-leave settings: {reason}"
            ))
            .for_setup(setup.id)],
            completion: JobCompletion::Failed,
        };
    }
    let mut rotary_axes = std::collections::HashMap::new();
    for operation in setup
        .operations
        .iter()
        .filter(|op| doc.operation_is_included(setup.id, op.id()))
    {
        if let Some(index) = doc.rotary_indexes.get(&operation.id()) {
            let reason = index
                .validate()
                .err()
                .or_else(|| {
                    operation
                        .rotary_wrap()
                        .is_some()
                        .then(|| "An operation cannot be both indexed and wrapped".to_string())
                })
                .or_else(|| {
                    let entry = (
                        index.axis,
                        match index.axis {
                            cam_model::RotaryAxis::A => [index.center[1], index.center[2]],
                            cam_model::RotaryAxis::B => [index.center[0], index.center[2]],
                        },
                    );
                    rotary_axes.insert(0, entry).and_then(|old| {
                        (old.0 != entry.0
                            || old.1.iter().zip(entry.1).any(|(a, b)| (a - b).abs() > 1e-6))
                        .then(|| {
                            "Indexed and wrapped operations must share one rotary axis and centre"
                                .to_string()
                        })
                    })
                });
            if let Some(reason) = reason {
                return JobResult {
                    completion: JobCompletion::Failed,
                    diagnostics: vec![Diagnostic::error(reason).for_op(operation.id())],
                    ..Default::default()
                };
            }
        }
        if let Some(wrap) = operation.rotary_wrap() {
            let reason=wrap.validate().err().or_else(|| {
                let entry=(wrap.axis,match wrap.axis {cam_model::RotaryAxis::A=>[wrap.center[1],wrap.center[2]],cam_model::RotaryAxis::B=>[wrap.center[0],wrap.center[2]]});
                rotary_axes.insert(0,entry).and_then(|old| {
                    (old.0!=entry.0 || old.1.iter().zip(entry.1).any(|(a,b)|(a-b).abs()>1e-6)).then(||"Wrapped operations in a setup must share one physical rotary axis and centre".to_string())
                })
            });
            if let Some(reason) = reason {
                return JobResult {
                    completion: JobCompletion::Failed,
                    diagnostics: vec![Diagnostic::error(reason)
                        .for_op(operation.id())
                        .for_setup(setup.id)],
                    ..Default::default()
                };
            }
        }
    }

    // Every use of this value assumes it is *above* the work: `lift_to_tool_change`
    // is named for what it is supposed to do, and the cross that follows travels at
    // it. Nothing validated that. A setup whose Tool Change Height sits below the
    // clearance plane would turn the lift into a descent and cross the part below
    // clearance — so raise it, and say so rather than silently overruling the value
    // the operator typed.
    let max_clearance = setup
        .operations
        .iter()
        .filter(|op| doc.operation_is_included(setup.id, op.id()))
        .map(|op| {
            setup
                .height_settings
                .get(&op.id())
                .and_then(|settings| settings.resolve(doc, op).ok())
                .map_or(setup.heights_for(op.id()).clearance, |heights| {
                    heights.clearance
                })
        })
        .fold(setup.heights.clearance, f64::max);
    // CAD bounds can include sub-micron padding. Always clamp to clearance,
    // but do not warn about numerical noise in otherwise equal safety planes.
    if max_clearance - tool_change_height > 1e-6 {
        let origin_z = setup.origin[2];
        diagnostics.push(Diagnostic::warning(format!(
            "tool-change height {:.6} mm is below the clearance plane \
             {:.6} mm (base WCS Z); using the clearance plane instead — a tool change or re-fixture \
             must not bring the tool down.",
            tool_change_height - origin_z,
            max_clearance - origin_z
        )));
    }
    let mut tool_change_height = tool_change_height.max(max_clearance).max(fixture_top);

    program.push(Step::Comment(setup.name.clone()));

    // A selected entry belongs to the first executed instance of its source.
    // Compute one additional, automatic-entry seed for the remaining instances;
    // both variants go through the same geometry, height and cache handling.
    let automatic_operations: std::collections::HashMap<u32, Operation> = setup
        .operations
        .iter()
        .filter(|op| {
            doc.operation_is_included(setup.id, op.id())
                && setup.patterns.iter().any(|pattern| {
                    !pattern.suppressed
                        && pattern.members.contains(&op.id())
                        && pattern.kind.instance_count().is_some_and(|count| {
                            count.saturating_sub(usize::from(!pattern.options.keep_original)) > 1
                        })
                })
        })
        .filter_map(|op| automatic_pattern_entry(op).map(|automatic| (op.id(), automatic)))
        .collect();
    // Coordinates are in the part frame; the post re-references each group to its origin.
    let mut seeds: Vec<Planned> = Vec::new();
    let mut automatic_seeds = std::collections::HashMap::new();
    for (operation, automatic_entry) in setup.operations.iter().flat_map(|op| {
        std::iter::once((op, false)).chain(automatic_operations.get(&op.id()).map(|op| (op, true)))
    }) {
        if !doc.operation_is_included(setup.id, operation.id()) {
            continue;
        }
        if cancel.is_cancelled() {
            completion = JobCompletion::Cancelled;
            break;
        }
        let mut resolved_operation;
        let (operation, parameter_error) = if operation.has_parameter_expressions() {
            resolved_operation = operation.clone();
            let error = resolved_operation
                .resolve_parameter_expressions(&setup.tools)
                .err();
            if error.is_none() {
                resolved_operation.freeze_parameter_expressions();
            }
            (&resolved_operation, error)
        } else {
            (operation, None)
        };
        let indexed_geometry = doc
            .rotary_indexes
            .get(&operation.id())
            .map(|index| index.geometry(&doc.geometry));
        let operation_geometry = indexed_geometry.as_ref().unwrap_or(&doc.geometry);
        env.heights = setup.heights_for(operation.id());
        if let Some(wrap) = operation.rotary_wrap() {
            env.heights.top_of_stock = wrap.surface_z;
        }
        let stock_to_leave = setup.stock_to_leave_for(operation.id());
        let configured = setup
            .height_settings
            .get(&operation.id())
            .map(|settings| settings.resolve(doc, operation));
        if let Some(Ok(heights)) = configured {
            env.heights = heights.safety();
        }
        env.heights.clearance = env.heights.clearance.max(fixture_top);
        env.heights.retract = env.heights.retract.max(fixture_top);
        let mut result = if let Some(reason) = parameter_error {
            StrategyResult {
                diagnostics: vec![Diagnostic::error(reason)],
                ..Default::default()
            }
        } else if let Some(Err(reason)) = &configured {
            StrategyResult {
                diagnostics: vec![Diagnostic::error(reason.clone())],
                ..Default::default()
            }
        } else if let Some(reason) = setup
            .operation_heights
            .get(&operation.id())
            .and_then(|h| h.validate().err())
        {
            StrategyResult {
                diagnostics: vec![Diagnostic::error(reason)],
                ..Default::default()
            }
        } else if doc.cad_linked_operations.contains(&operation.id())
            && !operation_geometry.bindings.contains_key(&operation.id())
        {
            StrategyResult {
                diagnostics: vec![Diagnostic::error(
                    "Linked CAD geometry binding is missing; reselect the source geometry before planning",
                ).with_code(crate::DiagnosticCode::GeometryUnresolved)],
                ..Default::default()
            }
        } else if let Some(binding) = operation_geometry.bindings.get(&operation.id()) {
            match &binding.status {
                cam_model::ResolutionStatus::NeedsReselection { reason } => StrategyResult {
                    diagnostics: vec![Diagnostic::error(format!("Needs reselection: {reason}"))
                        .with_code(crate::DiagnosticCode::GeometryUnresolved)],
                    ..Default::default()
                },
                cam_model::ResolutionStatus::Resolved => {
                    if !binding
                        .selected
                        .iter()
                        .all(|r| operation_geometry.entity_exists(r))
                        || binding.resolved_fingerprint.as_ref()
                            != Some(&operation_geometry.binding_fingerprint(binding))
                    {
                        StrategyResult {
                            diagnostics: vec![Diagnostic::error("Linked CAD geometry has no matching resolved snapshot; resolve it before planning").with_code(crate::DiagnosticCode::GeometryUnresolved)],
                            ..Default::default()
                        }
                    } else {
                        compute_at_heights(
                            operation,
                            &env,
                            configured
                                .as_ref()
                                .and_then(|r| r.as_ref().ok())
                                .map_or(binding.effective_heights(), |h| h.cutting()),
                            cancel,
                            ComputeContext {
                                stock_to_leave,
                                compute: &mut compute,
                                geometry: operation_geometry,
                            },
                            true,
                        )
                    }
                }
            }
        } else if let Some(Ok(heights)) = configured {
            compute_at_heights(
                operation,
                &env,
                heights.cutting(),
                cancel,
                ComputeContext {
                    stock_to_leave,
                    compute: &mut compute,
                    geometry: operation_geometry,
                },
                false,
            )
        } else if operation.rotary_wrap().is_some() {
            compute_at_heights(
                operation,
                &env,
                doc.operation_base_cutting_heights(operation),
                cancel,
                ComputeContext {
                    stock_to_leave,
                    compute: &mut compute,
                    geometry: &doc.geometry,
                },
                false,
            )
        } else {
            compute(
                operation,
                &env,
                cancel,
                stock_to_leave,
                operation_geometry,
                None,
            )
        };
        if let Some(Ok(heights)) = configured {
            if let Some(feed_z) = heights.feed {
                // A contour can be below the stock top (for example an underside
                // loop). Its feed plane does not prove the material above it is
                // clear. Only turn the approach into a rapid through known air.
                let air_floor = feed_z.max(heights.stock_top);
                match apply_feed_height(&result.program, air_floor, operation) {
                    Ok(program) => result.program = program,
                    Err(reason) => {
                        result.program = Program::new();
                        result.diagnostics.push(Diagnostic::error(reason));
                    }
                }
            }
        }
        let failed = result.has_errors();
        let cancelled = result.cancelled || cancel.is_cancelled();
        // Stamp ownership here: the planner knows which operation produced these,
        // the strategy does not need to.
        diagnostics.extend(
            result
                .diagnostics
                .into_iter()
                .map(|d| d.for_op(operation.id()).for_setup(setup.id)),
        );
        if cancelled {
            // Discard the unfinished fragment, including cancellation in the last
            // operation where there is no next loop iteration to observe the token.
            completion = JobCompletion::Cancelled;
            break;
        }
        if failed {
            completion = JobCompletion::Failed;
            continue;
        }
        if result.program.is_empty() {
            continue;
        }
        // Per-operation spindle speed: the op's own value if set, else the job default
        // (which keeps existing documents — every op at rpm 0 — unchanged).
        let rpm = if operation.spindle_rpm() > 0.0 {
            operation.spindle_rpm()
        } else {
            spindle_rpm
        };
        let seed = Planned {
            op_id: operation.id(),
            clearance: env.heights.clearance,
            tool: operation.tools()[0],
            rpm,
            cooling: operation.cooling(),
            // A multi-tool operation (Carve) emits its own tool change mid-fragment;
            // whatever it leaves in the spindle is what the next instance compares
            // against, so read it back rather than assume.
            ends_with: last_tool_change(&result.program),
            datum: operation.work_offset(),
            fragment: result.program,
            pattern_id: None,
            instance_index: None,
        };
        if automatic_entry {
            automatic_seeds.insert(operation.id(), seed);
        } else {
            seeds.push(seed);
        }
    }

    // Expand only after every included seed variant has been computed once.
    // Transitions and machine control are assembled later around each derived fragment.
    let seeds_by_id: std::collections::HashMap<u32, Planned> =
        seeds.into_iter().map(|p| (p.op_id, p)).collect();
    let mut planned: Vec<Planned> = Vec::new();
    if cancel.is_cancelled() {
        // Preserve completed ordinary work as a preview if cancellation arrived while
        // computing seed fragments. Pattern expansion can multiply that work many
        // thousands of times, so a canceled run never begins or continues expansion.
        let grouped: std::collections::HashSet<u32> = setup
            .patterns
            .iter()
            .flat_map(|pattern| pattern.members.iter().copied())
            .collect();
        for operation in &setup.operations {
            if !grouped.contains(&operation.id()) {
                if let Some(seed) = seeds_by_id.get(&operation.id()) {
                    planned.push(seed.clone());
                }
            }
        }
        completion = JobCompletion::Cancelled;
    } else {
        for item in setup.execution_items() {
            if cancel.is_cancelled() {
                completion = JobCompletion::Cancelled;
                break;
            }
            match item {
                cam_model::ExecutionItem::Operation(operation) => {
                    if let Some(seed) = seeds_by_id.get(&operation.id()) {
                        planned.push(seed.clone());
                    }
                }
                cam_model::ExecutionItem::Pattern(pattern) => {
                    if pattern.suppressed {
                        continue;
                    }
                    let count = match pattern.kind.instance_count() {
                        Some(n) if n <= 10_000 => n,
                        _ => {
                            diagnostics.push(
                                Diagnostic::error(
                                    "pattern instance count exceeds the supported limit",
                                )
                                .for_setup(setup.id),
                            );
                            completion = JobCompletion::Failed;
                            continue;
                        }
                    };
                    let indices = pattern.options.instance_indices(pattern.kind);
                    let count = count - usize::from(!pattern.options.keep_original);
                    let required = pattern
                        .members
                        .iter()
                        .filter_map(|id| seeds_by_id.get(id))
                        .try_fold(0usize, |sum, seed| {
                            let steps = seed.fragment.steps().len();
                            let repeated_steps = automatic_seeds
                                .get(&seed.op_id)
                                .map_or(steps, |seed| seed.fragment.steps().len());
                            repeated_steps
                                .checked_mul(count.saturating_sub(1))?
                                .checked_add(steps)?
                                .checked_add(sum)
                        });
                    let current = planned
                        .iter()
                        .try_fold(0usize, |sum, p| sum.checked_add(p.fragment.steps().len()));
                    if required.is_none()
                        || current
                            .and_then(|n| n.checked_add(required.unwrap_or(usize::MAX)))
                            .is_none_or(|n| n > 5_000_000)
                    {
                        diagnostics.push(
                            Diagnostic::error(
                                "expanded pattern run exceeds 5,000,000 motion steps",
                            )
                            .for_setup(setup.id),
                        );
                        completion = JobCompletion::Failed;
                        continue;
                    }
                    let center = match pattern.kind {
                        cam_model::PatternKind::Circular { center_xy, .. } => center_xy,
                        _ => [0.0, 0.0],
                    };
                    let mut append = |op_id: u32, index: usize| {
                        if cancel.is_cancelled() {
                            return;
                        }
                        let seed = if Some(&index) != indices.first()
                            && automatic_operations.contains_key(&op_id)
                        {
                            automatic_seeds.get(&op_id)
                        } else {
                            seeds_by_id.get(&op_id)
                        };
                        let Some(seed) = seed else {
                            return;
                        };
                        let Some((translation, degrees)) =
                            pattern.options.transform_xy(pattern.kind, index)
                        else {
                            completion = JobCompletion::Failed;
                            return;
                        };
                        let fragment = seed
                            .fragment
                            .transformed_xy(center, translation, degrees)
                            .with_pattern_owner(pattern.id, index as u32);
                        planned.push(Planned {
                            fragment,
                            pattern_id: Some(pattern.id),
                            instance_index: Some(index as u32),
                            ..seed.clone()
                        });
                    };
                    match pattern.order {
                        cam_model::PatternOrder::ByInstance => {
                            for index in &indices {
                                for member in &pattern.members {
                                    append(*member, *index);
                                }
                            }
                        }
                        cam_model::PatternOrder::ByOperation | cam_model::PatternOrder::ByTool => {
                            let mut members = pattern.members.clone();
                            if pattern.order == cam_model::PatternOrder::ByTool {
                                members.sort_by_key(|id| seeds_by_id.get(id).map(|seed| seed.tool));
                            }
                            for member in &members {
                                for index in &indices {
                                    append(*member, *index);
                                }
                            }
                        }
                    }
                    drop(append);
                    if cancel.is_cancelled() {
                        completion = JobCompletion::Cancelled;
                    }
                }
            }
        }
    }
    if planned
        .iter()
        .try_fold(0usize, |sum, p| sum.checked_add(p.fragment.steps().len()))
        .is_none_or(|n| n > 5_000_000)
    {
        diagnostics.push(
            Diagnostic::error("expanded pattern run exceeds 5,000,000 motion steps")
                .for_setup(setup.id),
        );
        completion = JobCompletion::Failed;
        planned.clear();
    }

    // Expand patterns in the unwrapped plane, then wrap each derived instance.
    for pl in &mut planned {
        if let Some(index) = doc.rotary_indexes.get(&pl.op_id) {
            match crate::index_program(&pl.fragment, index, cancel) {
                Ok(fragment) => pl.fragment = fragment,
                Err(reason) => {
                    pl.fragment = Program::new();
                    diagnostics.push(Diagnostic::error(reason).for_op(pl.op_id));
                    completion = if cancel.is_cancelled() {
                        JobCompletion::Cancelled
                    } else {
                        JobCompletion::Failed
                    };
                }
            }
            continue;
        }
        let Some(wrap) = setup
            .operations
            .iter()
            .find(|op| op.id() == pl.op_id)
            .and_then(Operation::rotary_wrap)
        else {
            continue;
        };
        if pl.clearance <= wrap.surface_z {
            pl.fragment = Program::new();
            diagnostics.push(
                Diagnostic::error("Wrapped clearance must be above the unwrapped cylinder surface")
                    .for_op(pl.op_id),
            );
            completion = JobCompletion::Failed;
            continue;
        }
        match crate::wrap_program(&pl.fragment, wrap, cancel) {
            Ok(fragment) => {
                pl.fragment = fragment;
                pl.clearance =
                    wrap.center[2] + wrap.effective_radius() + pl.clearance - wrap.surface_z;
                tool_change_height = tool_change_height.max(pl.clearance);
            }
            Err(reason) => {
                pl.fragment = Program::new();
                diagnostics.push(
                    Diagnostic::error(reason)
                        .for_op(pl.op_id)
                        .for_setup(setup.id),
                );
                completion = if cancel.is_cancelled() {
                    JobCompletion::Cancelled
                } else {
                    JobCompletion::Failed
                };
            }
        }
    }

    // Group the operations by origin, in the setup's origin order, so each datum's
    // block is contiguous however the operations happen to be ordered in the list — a
    // reorientation is done once, never ping-ponged. A stable sort keeps each group's
    // internal order; an operation whose `work_offset` names no known origin falls into
    // the base group (rank 0), never silently dropped.
    let rank: std::collections::HashMap<u32, usize> = setup
        .origin_indices()
        .into_iter()
        .enumerate()
        .map(|(i, idx)| (idx, i))
        .collect();
    planned.sort_by_key(|pl| rank.get(&pl.datum).copied().unwrap_or(0));

    // Emit each operation once, under the origin its `work_offset` names. Every new
    // context — the first operation, a tool change, or a reorientation — opens by lifting
    // to the tool-change height and descending to clearance (a reorientation adds the
    // `M00` operator stop for re-fixturing); the emitter handles that.
    let mut st = EmitState::new();
    // A cancelled planning run retains its completed prefix as a non-exportable
    // preview. Cancellation first arriving during assembly stops at a boundary.
    let cancelled_before_assembly = cancel.is_cancelled();
    for pl in &planned {
        if !cancelled_before_assembly && cancel.is_cancelled() {
            completion = JobCompletion::Cancelled;
            break;
        }
        emit_instance(
            &mut program,
            pl,
            dir,
            &mut st,
            tool_change_height,
            pl.clearance,
        );
    }

    // The same shutdown as a re-fixture halt, and for the same reason: the operator is
    // about to put hands on the work — here to take the part out. It used to be two
    // lines in a different order with no lift, because it was written at first light
    // and the re-fixture path was written eight weeks later; nothing ever produced
    // both in one program until a multi-datum job with a tool change did.
    if st.spindle_started {
        let last_op = planned.last().map(|pl| pl.op_id).unwrap_or(0);
        shut_down(&mut program, tool_change_height, last_op);
    }

    // Covers cancellation during assembly and deliberately empty documents too.
    if cancel.is_cancelled() {
        completion = JobCompletion::Cancelled;
    }
    if !fixtures.is_empty()
        && !program.has_rotary_motion()
        && completion != JobCompletion::Cancelled
        && program.validate().is_ok()
    {
        let profiles = setup
            .operations
            .iter()
            .filter_map(|operation| {
                if let Operation::Profile(profile) = operation {
                    if profile.comp == cam_model::Comp::Computed
                        && profile.plunge == cam_model::Plunge::Straight
                        && !profile.tabs.enabled
                    {
                        return Some((profile.id, profile.plunge_feed));
                    }
                }
                None
            })
            .collect();
        match crate::fixtures::avoid_profile_fixtures(
            &program,
            &profiles,
            &setup.tools,
            &fixtures,
            max_clearance.max(fixture_top),
            cancel,
        ) {
            Ok(routed) => {
                if routed.has_errors() {
                    completion = JobCompletion::Failed;
                }
                program = routed.program;
                diagnostics.extend(
                    routed
                        .diagnostics
                        .into_iter()
                        .map(|d| d.for_setup(setup.id)),
                );
            }
            Err(reason) => {
                completion = if cancel.is_cancelled() {
                    JobCompletion::Cancelled
                } else {
                    JobCompletion::Failed
                };
                diagnostics.push(Diagnostic::error(reason).for_setup(setup.id));
            }
        }
    }
    if let Err(error) = program.validate() {
        let mut diagnostic = Diagnostic::error(error.to_string());
        if let Some(op) = error.op {
            diagnostic = diagnostic.for_op(op);
        }
        if let Some(tag) = program.steps().get(error.step).and_then(Step::tag) {
            if let (Some(pattern_id), Some(instance_index)) = (tag.pattern_id, tag.instance_index) {
                diagnostic = diagnostic.for_pattern(pattern_id, instance_index);
            }
        }
        diagnostics.push(
            diagnostic
                .with_code(crate::DiagnosticCode::InvalidMotion)
                .for_setup(setup.id)
                .for_segment(error.step),
        );
        if completion != JobCompletion::Cancelled {
            completion = JobCompletion::Failed;
        }
    }
    if !fixtures.is_empty() && completion != JobCompletion::Cancelled && program.validate().is_ok()
    {
        match crate::verify_fixture_clearance(&program, &setup.tools, &fixtures, cancel) {
            Ok(contacts) => {
                for contact in contacts {
                    let mut diagnostic = Diagnostic::error(format!("Tool or shank enters the clearance envelope of fixture {}. Change the machining boundary, heights, tool or fixture placement.", contact.fixture_name))
                    .for_setup(setup.id).for_op(contact.tag.op_id).for_segment(contact.step_index);
                    if let (Some(pattern), Some(instance)) =
                        (contact.tag.pattern_id, contact.tag.instance_index)
                    {
                        diagnostic = diagnostic.for_pattern(pattern, instance);
                    }
                    diagnostics.push(diagnostic);
                    completion = JobCompletion::Failed;
                }
            }
            Err(reason) => {
                completion = if cancel.is_cancelled() {
                    JobCompletion::Cancelled
                } else {
                    JobCompletion::Failed
                };
                diagnostics.push(Diagnostic::error(reason).for_setup(setup.id));
            }
        }
    }
    JobResult {
        program,
        diagnostics,
        completion,
    }
}

/// Adapt the legacy strategies to an explicit absolute cutting interval. Pocket
/// and profile compute in a local Z frame so reach checks measure the physical
/// interval, even when the imported floor lies above world zero. Posting still
/// applies the work datum exactly once.
type StrategyCompute<'a> = dyn FnMut(
        &Operation,
        &JobEnv,
        &CancelToken,
        cam_model::StockToLeave,
        &cam_model::GeometryState,
        Option<cam_model::CuttingHeights>,
    ) -> StrategyResult
    + 'a;

/// Restore the strategy's usual entry without changing leads, exits or linking.
fn automatic_pattern_entry(operation: &Operation) -> Option<Operation> {
    let mut automatic = operation.clone();
    let start = match &mut automatic {
        Operation::Profile(op) => &mut op.start,
        Operation::Pocket(op) => &mut op.start,
        Operation::Chamfer(op) => &mut op.start,
        Operation::Engrave(op) => &mut op.start,
        Operation::Carve(op) => &mut op.start,
        Operation::Adaptive(op) => &mut op.params.start,
        Operation::Drill(_)
        | Operation::Face(_)
        | Operation::Parallel(_)
        | Operation::Spiral(_)
        | Operation::Bore(_)
        | Operation::Thread(_) => return None,
    };
    start.take()?;
    Some(automatic)
}

struct ComputeContext<'a> {
    stock_to_leave: cam_model::StockToLeave,
    compute: &'a mut StrategyCompute<'a>,
    geometry: &'a cam_model::GeometryState,
}

fn compute_at_heights(
    operation: &Operation,
    env: &JobEnv,
    heights: cam_model::CuttingHeights,
    cancel: &CancelToken,
    context: ComputeContext<'_>,
    linked: bool,
) -> StrategyResult {
    if let Operation::Profile(p) = operation {
        if let Err(reason) = p.validate_profile_source(linked) {
            return StrategyResult {
                diagnostics: vec![Diagnostic::error(reason)],
                ..Default::default()
            };
        }
    }
    if let Err(reason) = heights.validate() {
        return StrategyResult {
            diagnostics: vec![Diagnostic::error(reason)],
            ..Default::default()
        };
    }
    if env.heights.clearance < heights.top || env.heights.retract < heights.top {
        return StrategyResult {
            diagnostics: vec![Diagnostic::error(format!(
                "Clearance Z{:.3} mm and retract Z{:.3} mm must both be at or above \
                 cutting top Z{:.3} mm. Adjust the references and offsets in Operation > Heights \
                 to resolve at or above Z{:.3} mm",
                env.heights.clearance, env.heights.retract, heights.top, heights.top,
            ))],
            ..Default::default()
        };
    }
    let mut op = operation.clone();
    let depth = heights.top - heights.bottom;
    let shift = match &mut op {
        Operation::Pocket(p) => {
            p.depth = depth;
            heights.top
        }
        Operation::Profile(p) => {
            p.depth = depth;
            heights.top
        }
        Operation::Drill(p) => {
            p.depth = depth;
            p.start_offset = 0.0;
            0.0
        }
        Operation::Face(p) => {
            p.depth = depth;
            p.start_offset = heights.top;
            0.0
        }
        Operation::Chamfer(p) => {
            p.top = heights.top;
            p.depth = depth;
            0.0
        }
        Operation::Engrave(p) => {
            p.top = heights.top;
            p.depth = depth;
            0.0
        }
        Operation::Adaptive(_) | Operation::Parallel(_) | Operation::Spiral(_) => 0.0,
        Operation::Carve(p) => {
            p.top = heights.top;
            p.depth = depth;
            0.0
        }
        Operation::Bore(p) => {
            p.z_top = heights.top;
            p.z_bottom = heights.bottom;
            0.0
        }
        Operation::Thread(p) => {
            p.z_top = heights.top;
            p.z_bottom = heights.bottom;
            0.0
        }
    };
    let local = JobEnv {
        heights: cam_model::Heights::new(
            env.heights.clearance - shift,
            env.heights.retract - shift,
            if shift != 0.0 || matches!(op, Operation::Pocket(_) | Operation::Profile(_)) {
                0.0
            } else if matches!(op, Operation::Drill(_)) {
                heights.top
            } else {
                env.heights.top_of_stock
            },
        ),
        tools: env.tools,
        // The existing facing strategy sweeps a rectangle. Its resolver only
        // certifies whole-body rectangular top faces; use that selected extent.
        stock: if linked && matches!(op, Operation::Face(_)) {
            None
        } else {
            env.stock
        },
        spindle: env.spindle,
    };
    let mut result = (context.compute)(
        &op,
        &local,
        cancel,
        context.stock_to_leave,
        context.geometry,
        Some(heights),
    );
    if shift != 0.0 {
        result.program = result.program.translated([0.0, 0.0, shift]);
    }
    result
}

/// Separate the air approach from the feed descent at the configured feed plane.
/// Drilling cycles are expanded when their feed plane differs from their retract
/// plane, because canned cycles share those planes on several target controls.
fn apply_feed_height(
    program: &Program,
    feed_z: f64,
    operation: &Operation,
) -> Result<Program, String> {
    let peck_reentry = matches!(operation, Operation::Drill(drill)
        if drill.cycle.mode.is_some_and(cam_model::DrillCycleMode::pecks));
    let plunge = match operation {
        Operation::Profile(o) => o.plunge_feed,
        Operation::Pocket(o) => o.clear.plunge_feed,
        Operation::Face(o) => o.plunge_feed,
        Operation::Adaptive(o) => o.params.plunge_feed,
        Operation::Parallel(o) => o.params.plunge_feed,
        Operation::Spiral(o) => o.params.plunge_feed,
        Operation::Drill(o) => o.feed,
        Operation::Bore(o) => o.plunge_feed,
        Operation::Thread(o) => o.plunge_feed,
        Operation::Chamfer(o) => o.plunge_feed,
        Operation::Engrave(o) => o.plunge_feed,
        Operation::Carve(o) => o.plunge_feed,
    };
    let mut expanded = Program::new();
    for step in program.steps() {
        if let Step::Drill(c) = step {
            if (c.retract - feed_z).abs() > 1e-9 {
                let entry = feed_z.min(c.retract);
                for &[x, y] in &c.points {
                    expanded.push(Step::Rapid {
                        to: Point3::new(x, y, c.retract.max(feed_z)),
                        tag: Tag {
                            kind: MoveKind::Link,
                            ..c.tag
                        },
                    });
                    expanded.push(Step::Rapid {
                        to: Point3::new(x, y, entry),
                        tag: Tag {
                            kind: MoveKind::Link,
                            ..c.tag
                        },
                    });
                    let mut previous = c.z_top;
                    let peck = c.peck.unwrap_or(c.z_top - c.depth);
                    let count = ((c.z_top - c.depth) / peck).ceil();
                    // Strategy validation bounds the interval; cap expansion against
                    // malicious/tiny peck settings, matching the post's own bound.
                    if !count.is_finite() || count * c.points.len() as f64 > 100_000. {
                        return Err(
                            "Feed-height drilling exceeds 100000 pecks; increase the peck depth"
                                .into(),
                        );
                    }
                    for index in 1..=count as usize {
                        let z = (c.z_top - index as f64 * peck).max(c.depth);
                        if index > 1 {
                            expanded.push(Step::Rapid {
                                to: Point3::new(x, y, (previous + 0.5).min(entry)),
                                tag: Tag {
                                    kind: MoveKind::Link,
                                    ..c.tag
                                },
                            });
                        }
                        expanded.push(Step::Linear {
                            to: Point3::new(x, y, z),
                            feed: c.feed,
                            tag: c.tag,
                        });
                        if z <= c.depth {
                            if let Some(seconds) = c.dwell {
                                expanded.push(Step::Dwell { seconds });
                            }
                        }
                        expanded.push(Step::Rapid {
                            to: Point3::new(x, y, c.retract.max(feed_z)),
                            tag: Tag {
                                kind: MoveKind::Retract,
                                ..c.tag
                            },
                        });
                        previous = z;
                    }
                }
                continue;
            }
        }
        expanded.push(step.clone());
    }
    let mut out = Program::new();
    let mut previous: Option<Point3> = None;
    for step in expanded.steps() {
        match (previous, step) {
            (Some(from), Step::Rapid { to, tag })
                if (from.x - to.x).abs() < 1e-9
                    && (from.y - to.y).abs() < 1e-9
                    && to.z < from.z
                    && to.z < feed_z
                    // An explicit peck has already drilled its re-entry segment.
                    && !(peck_reentry && tag.kind == MoveKind::Link) =>
            {
                if from.z > feed_z {
                    out.push(Step::Rapid {
                        to: Point3::new(to.x, to.y, feed_z),
                        tag: *tag,
                    });
                }
                out.push(Step::Linear {
                    to: *to,
                    feed: plunge,
                    // This is now a cutting entry at plunge feed. Keeping the
                    // rapid's Link tag would make verification treat the feed
                    // descent through stock as a non-cutting reposition.
                    tag: Tag {
                        kind: MoveKind::Plunge,
                        ..*tag
                    },
                });
            }
            (Some(from), Step::Linear { to, feed, tag })
                if matches!(tag.kind, MoveKind::Plunge | MoveKind::Link)
                    && (from.x - to.x).abs() < 1e-9
                    && (from.y - to.y).abs() < 1e-9
                    && to.z < from.z
                    && from.z > feed_z =>
            {
                out.push(Step::Rapid {
                    to: Point3::new(to.x, to.y, feed_z.max(to.z)),
                    tag: *tag,
                });
                if to.z < feed_z {
                    out.push(Step::Linear {
                        to: *to,
                        feed: *feed,
                        tag: *tag,
                    });
                }
            }
            _ => out.push(step.clone()),
        }
        previous = match step {
            Step::Rapid { to, .. } | Step::Linear { to, .. } => Some(*to),
            Step::Arc { end, .. } => Some(*end),
            Step::Drill(c) => c.points.last().map(|[x, y]| Point3::new(*x, *y, c.retract)),
            _ => previous,
        };
    }
    Ok(out)
}

/// One operation, computed and ready to be emitted. The fragment's coordinates are in
/// the part frame; the post re-references them to the operation's origin.
#[derive(Clone)]
struct Planned {
    /// The emitting operation's id — for tagging the group's start-point rapid.
    op_id: u32,
    clearance: f64,
    fragment: Program,
    tool: u32,
    rpm: f64,
    cooling: cam_model::Cooling,
    /// The tool a multi-tool fragment (Carve) leaves in the spindle, if any.
    ends_with: Option<u32>,
    /// The operation's origin index (its `work_offset`) — the group it belongs to.
    datum: u32,
    pattern_id: Option<u64>,
    instance_index: Option<u32>,
}

impl Planned {
    fn tag(&self, kind: MoveKind) -> Tag {
        let mut tag = Tag::new(self.op_id, kind);
        tag.pattern_id = self.pattern_id;
        tag.instance_index = self.instance_index;
        tag
    }
}

/// Running machine state threaded across emitted operation instances, so a tool,
/// spindle speed, datum or coolant is re-commanded only when it actually changes.
struct EmitState {
    spindle_started: bool,
    current_rpm: Option<f64>,
    current_tool: Option<u32>,
    current_datum: u32,
    last_clearance: Option<f64>,
}

impl EmitState {
    fn new() -> Self {
        // Datum 1 is the base WCS and the initial state, so an op on datum 1 emits no
        // `Step::Datum`; a single-datum job's output is unchanged from before multi-WCS.
        Self {
            spindle_started: false,
            current_rpm: None,
            current_tool: None,
            current_datum: 1,
            last_clearance: None,
        }
    }
}

/// Emit one operation instance, opening every new context from the tool-change height.
///
/// The datum select precedes the tool change so a post can state `G15 H<n>` in the
/// tool-section head; tool/spindle/coolant are re-commanded only on change. Then, for
/// the first operation, a tool change, or a reorientation, the planner crosses to the
/// operation's start at the tool-change height and descends to clearance before handing
/// off — so the tool always reaches a known safe height before descending, whatever the
/// machine's prior state. A reorientation additionally halts (`M00`) for the manual
/// re-fixture, and its cross is left visually broken in the backplot (the pen-lift at the
/// `Datum`). Same-tool, same-datum consecutive operations get no transition — they hop at
/// clearance via the operation's own approach.
fn emit_instance(
    program: &mut Program,
    pl: &Planned,
    dir: SpindleDir,
    st: &mut EmitState,
    tool_change_height: f64,
    clearance: f64,
) {
    let datum_changing = st.current_datum != pl.datum;
    let tool_changing = st.current_tool != Some(pl.tool);

    if datum_changing {
        // A new datum is a physical reorientation of the part. Before the operator
        // handles it, stop the spindle and coolant (`M05`/`M09`) so nothing is live
        // during the flip, lift clear to the tool-change height so the part is free to
        // handle, then halt (`M00`) — but not before the *first* group (nothing to
        // re-fixture from). Force spindle and coolant to be re-commanded for the new
        // group by clearing their running state.
        if st.spindle_started {
            shut_down(program, tool_change_height, pl.op_id);
            program.push(Step::Stop);
            st.current_rpm = None;
            st.spindle_started = false;
        }
        program.push(Step::Datum(pl.datum));
        st.current_datum = pl.datum;
    } else if tool_changing && st.spindle_started {
        // A same-datum tool change: lift clear to the tool-change height in place before
        // the `M6`, so the ATC swings with the tool out of the work. The first tool load
        // (spindle not yet started) has nothing to lift away from — the cross below still
        // takes it up to the tool-change height first.
        lift_to_tool_change(program, tool_change_height, pl.op_id);
    }
    if tool_changing {
        program.push(Step::ToolChange { tool: pl.tool });
        st.current_tool = Some(pl.tool);
    }
    if st
        .current_rpm
        .is_none_or(|c| (c - pl.rpm).abs() > f64::EPSILON)
    {
        program.push(Step::Spindle { rpm: pl.rpm, dir });
        st.current_rpm = Some(pl.rpm);
    }
    st.spindle_started = true;
    // Index at tool-change clearance before crossing to the next operation.
    // All cutting blocks of an indexed fragment carry the same absolute angle.
    if let Some(Step::Rotary {
        angle,
        axis,
        center,
        ..
    }) = pl
        .fragment
        .steps()
        .iter()
        .find(|s| matches!(s, Step::Rotary { .. }))
    {
        let previous_angle = program
            .steps()
            .iter()
            .rev()
            .find_map(|s| {
                if let Step::Rotary { angle, .. } = s {
                    Some(*angle)
                } else {
                    None
                }
            })
            .unwrap_or(0.);
        if (*angle - previous_angle).abs() > 1e-9 {
            lift_to_tool_change(program, tool_change_height.max(clearance), pl.op_id);
            if last_position(program).is_none() {
                if let Some(start) = first_position(&pl.fragment) {
                    program.push(Step::Rapid {
                        to: Point3::new(start.x, start.y, tool_change_height.max(clearance)),
                        tag: pl.tag(MoveKind::Traverse),
                    });
                }
            }
            if let Some(to) = last_position(program) {
                let length = (*angle - previous_angle).to_radians().abs() * (to.z - center.z).abs();
                program.push(Step::Rotary {
                    to,
                    angle: *angle,
                    axis: *axis,
                    center: *center,
                    feed: None,
                    length,
                    tag: pl.tag(MoveKind::Traverse),
                });
            }
        }
    }
    // Ordinary machining resumes at rotary zero, with the cutter lifted before
    // indexing. Retain the full turn count rather than choosing a shortest path.
    if !pl.fragment.has_rotary_motion() {
        if let Some(Step::Rotary {
            angle,
            axis,
            center,
            ..
        }) = program
            .steps()
            .iter()
            .rev()
            .find(|s| matches!(s, Step::Rotary { .. }))
        {
            let (angle, axis, center) = (*angle, *axis, *center);
            if angle.abs() > 1e-9 {
                lift_to_tool_change(program, tool_change_height.max(clearance), pl.op_id);
                if let Some(to) = last_position(program) {
                    let length = angle.to_radians().abs() * (to.z - center.z).abs();
                    program.push(Step::Rotary {
                        to,
                        angle: 0.,
                        axis,
                        center,
                        feed: None,
                        length,
                        tag: pl.tag(MoveKind::Traverse),
                    });
                }
            }
        }
    }
    // Resolve the requested state for this operation's cutting fragment. Cooling belongs
    // to the cut, so it is switched off again at the end of each fragment.
    let coolant = match pl.cooling {
        cam_model::Cooling::None => Coolant::Off,
        cam_model::Cooling::Mms => Coolant::Mist,
    };
    // Open every new context from the tool-change height: cross to this operation's start
    // XY at that height, then descend to clearance and hand off (the operation owns
    // clearance and below). Tagged `Traverse` so the backplot draws it apart. For the
    // first op and same-datum tool changes it draws as one continuous up-over-down; for a
    // reorientation the pen-lift at the `Datum` breaks the cross, leaving it distinct.
    // Same-tool, same-datum consecutive operations skip this and hop at clearance.
    if tool_changing || datum_changing {
        if let Some(start) = first_position(&pl.fragment) {
            program.push(Step::Rapid {
                to: Point3::new(start.x, start.y, tool_change_height),
                tag: pl.tag(MoveKind::Traverse),
            });
            program.push(Step::Rapid {
                to: Point3::new(start.x, start.y, clearance),
                tag: pl.tag(MoveKind::Traverse),
            });
        }
    } else if let (Some(previous), Some(start)) =
        (last_position(program), first_position(&pl.fragment))
    {
        // Operations may now have different safety planes. Lift vertically before
        // crossing XY, then descend at the destination; never traverse diagonally
        // upward from a lower operation's clearance into a taller workpiece.
        if (previous.z - clearance).abs() > 1e-9 {
            let travel_z = previous
                .z
                .max(clearance)
                .max(st.last_clearance.unwrap_or(previous.z));
            let tag = pl.tag(MoveKind::Traverse);
            program.push(Step::Rapid {
                to: Point3::new(previous.x, previous.y, travel_z),
                tag,
            });
            program.push(Step::Rapid {
                to: Point3::new(start.x, start.y, travel_z),
                tag,
            });
            program.push(Step::Rapid {
                to: Point3::new(start.x, start.y, clearance),
                tag,
            });
        }
    }
    program.push(Step::Coolant(coolant));
    for step in pl.fragment.steps() {
        if let Step::Rotary {
            to,
            angle,
            axis,
            center,
            feed: None,
            length,
            tag,
        } = step
        {
            if *length == 0. {
                let previous_angle = program
                    .steps()
                    .iter()
                    .rev()
                    .find_map(|s| match s {
                        Step::Rotary { angle, .. } => Some(*angle),
                        _ => None,
                    })
                    .unwrap_or(0.);
                let previous = last_position(program).unwrap_or(*to);
                let rotation = (*angle - previous_angle).to_radians()
                    * ((previous.z + to.z) * 0.5 - center.z).abs();
                let length = (to.x - previous.x)
                    .hypot(to.y - previous.y)
                    .hypot(to.z - previous.z)
                    .hypot(rotation);
                program.push(Step::Rotary {
                    to: *to,
                    angle: *angle,
                    axis: *axis,
                    center: *center,
                    feed: None,
                    length,
                    tag: *tag,
                });
                continue;
            }
        }
        // Compound Carve operations switch cutters inside their strategy fragment.
        // Keep lubrication off while the tool is being changed.
        if matches!(step, Step::ToolChange { .. }) && coolant != Coolant::Off {
            program.push(Step::Coolant(Coolant::Off));
            program.push(step.clone());
            program.push(Step::Coolant(coolant));
        } else {
            program.push(step.clone());
        }
    }
    if coolant != Coolant::Off {
        program.push(Step::Coolant(Coolant::Off));
    }
    st.last_clearance = Some(clearance);
    if let Some(t) = pl.ends_with {
        st.current_tool = Some(t);
    }
}

/// Bring the machine to a state a person can safely approach: spindle stopped,
/// coolant off, tool lifted clear of the work.
///
/// **One function, two callers, because they are one physical situation** — the
/// re-fixture halt and the end of the program both end with the operator reaching in,
/// to flip the part or to take it out. They were written eight weeks apart and had
/// drifted: the end of the program stopped coolant before the spindle and never
/// lifted at all, so a finished job left the tool sitting at clearance over the part.
/// Neither was wrong in isolation; nothing had ever emitted both in one program.
///
/// **Spindle before coolant**, deliberately: the spindle is the half that hurts, so it
/// is the half that stops first.
fn shut_down(program: &mut Program, tool_change_height: f64, op_id: u32) {
    program.push(Step::SpindleOff);
    program.push(Step::Coolant(Coolant::Off));
    lift_to_tool_change(program, tool_change_height, op_id);
}

/// Rapid straight up to the tool-change height, in place (X/Y unchanged), so a tool
/// change or manual re-fixture happens with the tool clear of the work. Tagged
/// [`MoveKind::Traverse`] so the backplot renders it apart from an operation's own
/// rapids. A no-op if nothing has moved yet — there is no prior position to lift from.
///
/// **Never descends.** The name is a promise: a "lift" that moved the tool down would
/// be the exact opposite of what every caller wants it for. `build_job` already raises
/// a below-clearance tool-change height, so this is the second line of defence rather
/// than the first — but a lift that can go the wrong way is not worth leaving to one
/// guard.
fn lift_to_tool_change(program: &mut Program, z: f64, op_id: u32) {
    if let Some(p) = last_position(program) {
        if p.z >= z {
            return;
        }
        program.push(Step::Rapid {
            to: Point3::new(p.x, p.y, z),
            tag: Tag::new(op_id, MoveKind::Traverse),
        });
    }
}

/// The tool's current position: the endpoint of the last positional step emitted so
/// far. A drill cycle ends above its last hole, at that cycle's retract plane.
fn last_position(program: &Program) -> Option<Point3> {
    program.steps().iter().rev().find_map(|s| match s {
        Step::Rotary { to, .. } | Step::Rapid { to, .. } | Step::Linear { to, .. } => Some(*to),
        Step::Arc { end, .. } => Some(*end),
        Step::Drill(c) => c.points.last().map(|[x, y]| Point3::new(*x, *y, c.retract)),
        _ => None,
    })
}

/// The first point an operation's fragment moves to — its approach start. Used to aim
/// the tool-change cross-and-descend at the next operation before handing off.
fn first_position(program: &Program) -> Option<Point3> {
    program.steps().iter().find_map(|s| match s {
        Step::Rotary { to, .. } | Step::Rapid { to, .. } | Step::Linear { to, .. } => Some(*to),
        Step::Arc { end, .. } => Some(*end),
        Step::Drill(c) => c
            .points
            .first()
            .map(|[x, y]| Point3::new(*x, *y, c.retract)),
        _ => None,
    })
}

/// Dispatch an operation to its strategy and compute it.
fn compute_with_stock(
    operation: &Operation,
    env: &JobEnv,
    cancel: &CancelToken,
    stock_to_leave: cam_model::StockToLeave,
    geometry_state: &cam_model::GeometryState,
    cutting_heights: Option<cam_model::CuttingHeights>,
) -> StrategyResult {
    compute_with_stock_and_preview(
        operation,
        env,
        cancel,
        stock_to_leave,
        geometry_state,
        cutting_heights,
        &mut |_| {},
    )
}

fn compute_with_stock_and_preview(
    operation: &Operation,
    env: &JobEnv,
    cancel: &CancelToken,
    stock_to_leave: cam_model::StockToLeave,
    geometry_state: &cam_model::GeometryState,
    cutting_heights: Option<cam_model::CuttingHeights>,
    publish: &mut dyn FnMut(&Program),
) -> StrategyResult {
    if let Some(tool) = operation
        .tools()
        .into_iter()
        .filter_map(|number| env.tool(number))
        .find(|tool| tool.kind.is_side_cutting_disc())
    {
        return StrategyResult {
            diagnostics: vec![Diagnostic::error(format!(
                "Tool {} ({}) requires a side-cutting strategy",
                tool.number, tool.kind
            ))],
            ..Default::default()
        };
    }
    let (radial, axial) = stock_to_leave.effective();
    match operation {
        Operation::Spiral(op) => {
            let resolved = geometry_state
                .bindings
                .get(&op.id)
                .ok_or_else(|| "Spiral geometry binding is missing".to_string())
                .and_then(|binding| cam_geometry3d::resolve_parallel(geometry_state, binding));
            match resolved {
                Ok(mut geometry) => {
                    geometry.heights = cutting_heights.unwrap_or(geometry.heights);
                    crate::SpiralStrategy::new(op.clone(), stock_to_leave, geometry)
                        .compute(env, cancel)
                }
                Err(reason) => StrategyResult {
                    diagnostics: vec![Diagnostic::error(reason)],
                    ..Default::default()
                },
            }
        }
        Operation::Parallel(op) => {
            let resolved = geometry_state
                .bindings
                .get(&op.id)
                .ok_or_else(|| "Parallel geometry binding is missing".to_string())
                .and_then(|binding| cam_geometry3d::resolve_parallel(geometry_state, binding));
            match resolved {
                Ok(mut geometry) => {
                    geometry.heights = cutting_heights.unwrap_or(geometry.heights);
                    crate::ParallelStrategy::new(op.clone(), stock_to_leave, geometry)
                        .compute(env, cancel)
                }
                Err(reason) => StrategyResult {
                    diagnostics: vec![Diagnostic::error(reason)],
                    ..Default::default()
                },
            }
        }
        Operation::Adaptive(op) => {
            let Some(binding) = geometry_state.bindings.get(&op.id) else {
                return StrategyResult {
                    diagnostics: vec![Diagnostic::error(format!(
                        "operation {}: adaptive geometry binding is missing",
                        op.id
                    ))],
                    ..Default::default()
                };
            };
            let heights = cutting_heights.unwrap_or_else(|| binding.effective_heights());
            let resolved = match cam_geometry3d::resolve_adaptive(
                geometry_state,
                binding,
                cam_geometry3d::AdaptiveResolveConfig {
                    heights,
                    stock_to_leave,
                    stepdown: op.params.stepdown,
                    fine_stepdown: op.params.fine_stepdown,
                    tolerance: op.params.tolerance,
                },
                || cancel.is_cancelled(),
            ) {
                Ok(geometry) => geometry,
                Err(_) if cancel.is_cancelled() => {
                    return StrategyResult {
                        cancelled: true,
                        ..Default::default()
                    }
                }
                Err(reason) => {
                    return StrategyResult {
                        diagnostics: vec![Diagnostic::error(reason)],
                        ..Default::default()
                    }
                }
            };
            AdaptiveStrategy::new(op.clone(), stock_to_leave, resolved)
                .compute_with_preview(env, cancel, publish)
        }
        Operation::Profile(op) => {
            if axial > 0.0 && axial >= op.depth {
                return stock_allowance_error(op.id, axial, op.depth);
            }
            let mut op = op.clone();
            op.offset += radial;
            op.depth -= axial;
            ProfileStrategy::new(op).compute(env, cancel)
        }
        Operation::Drill(op) => DrillStrategy::new(op.clone()).compute(env, cancel),
        Operation::Pocket(op) => {
            if axial > 0.0 && axial >= op.depth {
                return stock_allowance_error(op.id, axial, op.depth);
            }
            let mut op = op.clone();
            if let Some(source) = op
                .stock_contours
                .source
                .as_ref()
                .filter(|_| op.stock_contours.enabled)
            {
                let resolved = if !matches!(
                    source.recipe,
                    cam_model::ExtractionRecipe::PocketBoundary
                        | cam_model::ExtractionRecipe::ProfileSilhouette
                ) {
                    Err("Invalid stock contour geometry recipe".to_string())
                } else {
                    cam_geometry3d::resolve_binding_advisory(geometry_state, source).and_then(
                        |input| {
                            if input.regions.is_empty() {
                                Err("Selected stock contours have no closed area".to_string())
                            } else {
                                Ok(input)
                            }
                        },
                    )
                };
                match resolved {
                    Ok(input) => op.stock_contours.regions = input.regions,
                    Err(reason) => {
                        return StrategyResult {
                            diagnostics: vec![Diagnostic::error(format!(
                                "Stock contours need reselection: {reason}"
                            ))
                            .with_code(crate::DiagnosticCode::GeometryUnresolved)],
                            ..Default::default()
                        }
                    }
                }
            }
            op.clear.offset += radial;
            op.depth -= axial;
            if op.stock_contours.enabled && !geometry_state.instances.is_empty() {
                let binding = geometry_state.bindings.get(&op.id);
                let floor = cutting_heights.map_or(-op.depth, |h| h.bottom) + axial;
                let selected = binding
                    .filter(|b| b.recipe == cam_model::ExtractionRecipe::PocketFloor)
                    .map_or(&[][..], |b| b.selected.as_slice());
                match cam_geometry3d::pocket_stock_shadow(geometry_state, selected, floor, || {
                    cancel.is_cancelled()
                }) {
                    Ok(Some(mut walls)) => {
                        if !selected.is_empty() {
                            // Face inner loops include descending holes as well as
                            // rising islands. The complete CAD shadow protects
                            // actual material above the cut; do not turn every
                            // floor opening into an additional solid obstacle.
                            op.islands.clear();
                            op.additional_regions = match op
                                .additional_regions
                                .iter()
                                .map(|region| cam_geo::Polygon::new(region.outer().clone()))
                                .collect::<Result<Vec<_>, _>>()
                            {
                                Ok(regions) => regions,
                                Err(reason) => {
                                    return StrategyResult {
                                        diagnostics: vec![Diagnostic::error(reason.to_string())],
                                        ..Default::default()
                                    }
                                }
                            };
                        } else {
                            let mut pockets = op.additional_regions.clone();
                            match cam_geo::Polygon::with_holes(
                                op.boundary.clone(),
                                op.islands.clone(),
                            ) {
                                Ok(p) => pockets.push(p),
                                Err(reason) => {
                                    return StrategyResult {
                                        diagnostics: vec![Diagnostic::error(reason.to_string())],
                                        ..Default::default()
                                    }
                                }
                            }
                            walls = match cam_geo::difference(&walls, &pockets) {
                                Ok(walls) => walls,
                                Err(reason) => {
                                    return StrategyResult {
                                        diagnostics: vec![Diagnostic::error(reason.to_string())],
                                        ..Default::default()
                                    }
                                }
                            };
                        }
                        let mut strategy = PocketStrategy::new(op.clone()).with_model_walls(walls);
                        if !selected.is_empty() {
                            if let Some((min, max)) = env.stock {
                                let mut bounds = [min, max];
                                for point in op
                                    .stock_contours
                                    .regions
                                    .iter()
                                    .flat_map(|p| p.outer().points())
                                {
                                    for (axis, value) in [point.x, point.y].into_iter().enumerate()
                                    {
                                        bounds[0][axis] = bounds[0][axis].min(value);
                                        bounds[1][axis] = bounds[1][axis].max(value);
                                    }
                                }
                                let finish = if op.finishing.enabled {
                                    op.finishing.count as f64 * op.finishing.stepover
                                } else {
                                    0.
                                };
                                let margin = env.tool(op.tool).map_or(0., |t| t.diameter)
                                    + op.clear.offset
                                    + finish;
                                for value in &mut bounds[0] {
                                    *value -= margin;
                                }
                                for value in &mut bounds[1] {
                                    *value += margin;
                                }
                                match cam_geometry3d::pocket_stock_domains(
                                    geometry_state,
                                    selected,
                                    floor,
                                    bounds,
                                ) {
                                    Ok(domains) => strategy = strategy.with_model_domains(domains),
                                    Err(reason) => {
                                        return StrategyResult {
                                            diagnostics: vec![Diagnostic::error(reason)],
                                            ..Default::default()
                                        }
                                    }
                                }
                            }
                        }
                        return strategy.compute(env, cancel);
                    }
                    Ok(None) => {}
                    Err(reason) => {
                        return StrategyResult {
                            diagnostics: vec![Diagnostic::error(reason)],
                            cancelled: cancel.is_cancelled(),
                            ..Default::default()
                        }
                    }
                }
            }
            let envelope = if op.stock_contours.enabled && !geometry_state.instances.is_empty() {
                match pocket_model_envelope(geometry_state) {
                    Ok(regions) => Some(regions),
                    Err(reason) => {
                        return StrategyResult {
                            diagnostics: vec![Diagnostic::error(reason)],
                            ..Default::default()
                        }
                    }
                }
            } else {
                None
            };
            let strategy = PocketStrategy::new(op);
            match envelope {
                Some(regions) => strategy.with_model_envelope(regions).compute(env, cancel),
                None => strategy.compute(env, cancel),
            }
        }
        Operation::Face(op) => {
            if radial > 0.0 {
                return StrategyResult {
                    diagnostics: vec![Diagnostic::error("radial stock to leave is not supported for Face; set radial allowance to 0 mm")],
                    ..Default::default()
                };
            }
            if axial > 0.0 && axial >= op.depth {
                return stock_allowance_error(op.id, axial, op.depth);
            }
            let mut op = op.clone();
            op.depth -= axial;
            FaceStrategy::new(op).compute(env, cancel)
        }
        Operation::Chamfer(op) => ChamferStrategy::new(op.clone()).compute(env, cancel),
        Operation::Bore(op) => crate::BoreStrategy::new(op.clone()).compute(env, cancel),
        Operation::Thread(op) => ThreadStrategy::new(op.clone()).compute(env, cancel),
        Operation::Engrave(op) => EngraveStrategy::new(op.clone()).compute(env, cancel),
        Operation::Carve(op) => CarveStrategy::new(op.clone()).compute(env, cancel),
    }
}

/// Conservative model bounds keep stock margins from becoming fictitious
/// pocket walls. Curved/nonrectangular model perimeters may leave extra stock.
fn pocket_model_envelope(
    geometry: &cam_model::GeometryState,
) -> Result<Vec<cam_geo::Polygon>, String> {
    use cam_geo::{Contour, Point, Polygon};
    geometry
        .instances
        .iter()
        .map(|instance| {
            let body = geometry
                .asset(instance.asset_id)
                .and_then(|asset| {
                    asset
                        .topology
                        .bodies
                        .iter()
                        .find(|b| b.id == instance.body_id)
                })
                .ok_or("Pocket model body is missing")?;
            let mut min = [f64::INFINITY; 2];
            let mut max = [f64::NEG_INFINITY; 2];
            for x in [body.bounds[0][0], body.bounds[1][0]] {
                for y in [body.bounds[0][1], body.bounds[1][1]] {
                    for z in [body.bounds[0][2], body.bounds[1][2]] {
                        let p = instance.placement.transform_point([x, y, z]);
                        if !p.into_iter().all(f64::is_finite) {
                            return Err("Pocket model bounds must be finite".into());
                        }
                        for axis in 0..2 {
                            min[axis] = min[axis].min(p[axis]);
                            max[axis] = max[axis].max(p[axis]);
                        }
                    }
                }
            }
            Polygon::new(Contour::new(vec![
                Point::new(min[0], min[1]),
                Point::new(max[0], min[1]),
                Point::new(max[0], max[1]),
                Point::new(min[0], max[1]),
            ]))
            .map_err(|e| e.to_string())
        })
        .collect()
}

fn stock_allowance_error(operation_id: u32, axial: f64, depth: f64) -> StrategyResult {
    StrategyResult {
        diagnostics: vec![Diagnostic::error(format!(
            "operation {operation_id}: axial stock to leave ({axial:.3} mm) must be less than the cutting depth ({depth:.3} mm)"
        ))],
        ..Default::default()
    }
}

/// The tool left in the spindle by `fragment` — the last [`Step::ToolChange`] it emits,
/// or `None` if it emits none (the single-tool case, which is every operation but Carve).
fn last_tool_change(fragment: &Program) -> Option<u32> {
    fragment.steps().iter().rev().find_map(|s| match s {
        Step::ToolChange { tool } => Some(*tool),
        _ => None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_geo::{Contour, Point};
    use cam_model::{DrillOp, Heights, Setup, Stock, Tool, ToolKind};

    fn profile_operation() -> Operation {
        Operation::Profile(cam_model::ProfileOp {
            parameter_expressions: Default::default(),
            wrap: None,
            passes: Default::default(),
            selections: None,
            closed: true,
            additional_chains: Vec::new(),
            tabs: Default::default(),
            id: 0,
            tool: 1,
            chain: Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(30.0, 0.0),
                Point::new(30.0, 30.0),
                Point::new(0.0, 30.0),
            ]),
            side: cam_model::Side::Inside,
            comp: cam_model::Comp::Computed,
            offset: 0.25,
            depth: 2.0,
            stepdown: 2.0,
            stepover: 0.0,
            spindle_rpm: 1000.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            feed: 300.0,
            plunge_feed: 100.0,
            end: None,
            start: None,
            lead_in: cam_model::Lead::None,
            lead_out: cam_model::Lead::None,
            lead_overlap: 0.0,
            plunge: cam_model::Plunge::Straight,
            clearing: Default::default(),
        })
    }

    fn plan_request(document: &Document) -> crate::PlanRequest<'_> {
        crate::PlanRequest {
            document,
            spindle_rpm: 1000.0,
            dir: SpindleDir::Cw,
            stock: None,
            tool_change_height: 50.0,
        }
    }

    fn document(operations: u32) -> Document {
        Document::new(Setup {
            id: 0,
            frame: Default::default(),
            material: String::new(),
            material_id: None,
            model_instances: Vec::new(),
            fixture_instances: Vec::new(),
            fixture_clearance: Default::default(),
            program_name: String::new(),
            name: "completion test".into(),
            heights: Heights::new(5.0, 2.0, 0.0),
            stock: Stock::BoundingBox {
                x_offset: 0.0,
                y_offset: 0.0,
                top: 0.0,
                thickness: 10.0,
            },
            tools: vec![Tool {
                number: 1,
                diameter: 3.0,
                length: 30.0,
                kind: ToolKind::Drill {
                    point_angle_deg: 118.0,
                },
                ..Default::default()
            }],
            operations: (0..operations)
                .map(|id| {
                    Operation::Drill(DrillOp {
                        cycle: Default::default(),
                        tip_through_bottom: false,
                        geometry: Default::default(),
                        hole_heights: Vec::new(),
                        parameter_expressions: Default::default(),
                        id,
                        tool: 1,
                        points: vec![[10.0 + id as f64, 10.0]],
                        depth: 1.0,
                        start_offset: 0.0,
                        peck: None,
                        dwell: None,
                        spindle_rpm: 1000.0,
                        tool_preset: None,
                        cooling: Default::default(),
                        entry_feeds: Default::default(),
                        work_offset: 1,
                        feed: 100.0,
                    })
                })
                .collect(),
            patterns: Vec::new(),
            origin: [0.0; 3],
            extra_origins: Vec::new(),
            origin_index: 1,
            operation_heights: Default::default(),
            height_settings: Default::default(),
            stock_to_leave: Default::default(),
            tool_change_height: None,
        })
    }

    #[test]
    fn side_cutting_discs_require_a_dedicated_strategy() {
        for kind in [ToolKind::GrooveCutter, ToolKind::SlittingSaw] {
            let mut doc = document(0);
            let setup = doc.setup_mut();
            setup.tools[0] = Tool {
                number: 1,
                diameter: 20.0,
                flute_length: 1.5,
                shank_diameter: 6.0,
                length: 50.0,
                flutes: 8,
                kind,
                ..Default::default()
            };
            setup.operations.push(profile_operation());
            let job = build_job(&doc, 1000., SpindleDir::Cw, None, 50., &CancelToken::new());
            assert_eq!(job.completion, JobCompletion::Failed);
            assert!(job
                .program
                .steps()
                .iter()
                .all(|step| matches!(step, Step::Comment(_))));
            assert!(job.diagnostics.iter().any(|finding| {
                finding.op == Some(0) && finding.message.contains("requires a side-cutting strategy")
            }));
        }
    }

    #[test]
    fn progress_reports_only_fragments_actually_recomputed() {
        let doc = document(3);
        let mut cache = crate::OperationCache::default();
        let cancel = CancelToken::new();
        let mut events = Vec::new();
        assert_eq!(
            build_job_cached_with_progress(plan_request(&doc), &cancel, &mut cache, |event| events
                .push(event))
            .completion,
            JobCompletion::Complete
        );
        assert_eq!(
            events,
            [
                CalculationProgress::Started(0),
                CalculationProgress::Finished {
                    operation: 0,
                    success: true
                },
                CalculationProgress::Started(1),
                CalculationProgress::Finished {
                    operation: 1,
                    success: true
                },
                CalculationProgress::Started(2),
                CalculationProgress::Finished {
                    operation: 2,
                    success: true
                }
            ]
        );
        events.clear();
        assert_eq!(
            build_job_cached_with_progress(plan_request(&doc), &cancel, &mut cache, |event| events
                .push(event))
            .completion,
            JobCompletion::Complete
        );
        assert!(events.is_empty());
        cache.invalidate_operations(&[2]);
        assert_eq!(
            build_job_cached_with_progress(plan_request(&doc), &cancel, &mut cache, |event| events
                .push(event))
            .completion,
            JobCompletion::Complete
        );
        assert_eq!(
            events,
            [
                CalculationProgress::Started(2),
                CalculationProgress::Finished {
                    operation: 2,
                    success: true
                }
            ]
        );
        assert_eq!(cache.stats.reused, 2);
        assert_eq!(cache.stats.computed, 1);
    }

    #[test]
    fn setup_stock_allowance_changes_profile_path_and_invalidates_cached_fragment() {
        let mut doc = document(0);
        doc.setup_mut().tools = vec![Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flute_length: 20.0,
            kind: ToolKind::EndMill,
            ..Default::default()
        }];
        doc.setup_mut().operations.push(profile_operation());
        let mut cache = crate::OperationCache::default();
        let original =
            crate::build_job_cached(plan_request(&doc), &CancelToken::new(), &mut cache, |_| {});
        assert_eq!(
            original.completion,
            JobCompletion::Complete,
            "{:?}",
            original.diagnostics
        );
        assert_eq!(cache.stats.computed, 1);

        doc.setup_mut().stock_to_leave.insert(
            0,
            cam_model::StockToLeave {
                enabled: false,
                radial: 0.75,
                axial: 0.5,
            },
        );
        let disabled =
            crate::build_job_cached(plan_request(&doc), &CancelToken::new(), &mut cache, |_| {});
        assert_eq!(
            disabled.completion,
            JobCompletion::Complete,
            "{:?}",
            disabled.diagnostics
        );
        assert_eq!(
            disabled.program, original.program,
            "disabled allowances must preserve the exact legacy path"
        );
        assert_eq!(
            cache.stats.reused, 1,
            "disabled values must not change toolpath dependencies"
        );

        doc.setup_mut().stock_to_leave.insert(
            0,
            cam_model::StockToLeave {
                enabled: true,
                radial: 0.5,
                axial: 0.5,
            },
        );
        let allowed =
            crate::build_job_cached(plan_request(&doc), &CancelToken::new(), &mut cache, |_| {});
        assert_eq!(
            allowed.completion,
            JobCompletion::Complete,
            "{:?}",
            allowed.diagnostics
        );
        assert_eq!(
            cache.stats.computed, 1,
            "changed allowances must not reuse the old path"
        );
        assert_ne!(allowed.program, original.program);
        let first_cut_xy = |program: &Program| {
            program
                .steps()
                .iter()
                .find_map(|step| match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                        Some([to.x, to.y])
                    }
                    Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => {
                        Some([end.x, end.y])
                    }
                    _ => None,
                })
                .unwrap()
        };
        let a = first_cut_xy(&original.program);
        let b = first_cut_xy(&allowed.program);
        assert!(
            (a[0] - b[0]).hypot(a[1] - b[1]) > 0.1,
            "radial allowance must offset the wall path: {a:?} -> {b:?}"
        );
        let deepest_cut = allowed
            .program
            .steps()
            .iter()
            .filter_map(|step| match step {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(to.z),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => Some(end.z),
                _ => None,
            })
            .fold(f64::INFINITY, f64::min);
        assert!(
            (deepest_cut - -1.5).abs() < 1e-9,
            "axial allowance must raise the floor: {deepest_cut}"
        );
    }

    #[test]
    fn stock_contours_clear_cad_floor_holes_but_protect_bosses_and_explicit_islands() {
        use cam_model::*;
        use std::sync::Arc;

        let square = |x, y, size| {
            Contour::new(vec![
                Point::new(x, y),
                Point::new(x + size, y),
                Point::new(x + size, y + size),
                Point::new(x, y + size),
            ])
        };
        let hole = square(6., 6., 12.);
        let boss = square(24., 24., 8.);
        let mut geometry = GeometryState::default();
        let mut mesh = CadMesh::default();
        let mut faces = Vec::new();
        // Selected floor and the top of a raised boss. The floor's inner
        // loops deliberately have identical 2D semantics in the operation.
        for (id, contour, z) in [(1, square(0., 0., 40.), -2.), (2, boss.clone(), 0.)] {
            let base = mesh.vertices.len() as u32;
            mesh.vertices
                .extend(contour.points().iter().map(|p| [p.x, p.y, z]));
            mesh.triangles
                .extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
            mesh.triangle_faces.extend([id, id]);
            faces.push(CadFace {
                id,
                body_id: 1,
                surface: CadSurface::Plane {
                    origin: [0., 0., z],
                    normal: [0., 0., 1.],
                },
                loop_ids: vec![],
                bounds: [[0., 0., z], [40., 40., z]],
            });
        }
        geometry.assets.push(GeometryAsset {
            id: 1,
            revision: 1,
            name: "floor with hole and boss".into(),
            source_format: CadFormat::Step,
            source_path: None,
            source_units: SourceUnits::Millimetres,
            source_data: Arc::new(vec![]),
            source_fingerprint: String::new(),
            import_options: None,
            import_diagnostics: vec![],
            backend: "test".into(),
            topology: Arc::new(TopologySnapshot {
                bodies: vec![CadBody {
                    id: 1,
                    name: "part".into(),
                    valid_solid: true,
                    bounds: [[0., 0., -4.], [40., 40., 0.]],
                }],
                faces,
                projection: Some(CadProjectionMesh {
                    mesh,
                    chord_tolerance: 0.0025,
                }),
                ..Default::default()
            }),
        });
        geometry.instances.push(GeometryInstance {
            id: 1,
            asset_id: 1,
            body_id: 1,
            name: "part".into(),
            placement: Default::default(),
            visible: true,
        });
        geometry.bindings.insert(
            2,
            GeometryBinding {
                selected: vec![EntityRef {
                    asset_id: 1,
                    revision: 1,
                    instance_id: 1,
                    body_id: 1,
                    kind: EntityKind::Face,
                    id: 1,
                }],
                recipe: ExtractionRecipe::PocketFloor,
                heights: CuttingHeights {
                    top: 0.,
                    bottom: -2.,
                },
                overrides: Default::default(),
                status: ResolutionStatus::Resolved,
                resolved_fingerprint: None,
            },
        );
        let tools = [Tool {
            number: 1,
            diameter: 4.,
            length: 30.,
            flute_length: 20.,
            ..Default::default()
        }];
        let env = JobEnv {
            tools: &tools,
            heights: Heights::new(10., 5., 0.),
            stock: Some(([0., 0.], [40., 40.])),
            spindle: SpindleDir::Cw,
        };
        let mut pocket = PocketOp {
            maximum_stepover: None,
            parameter_expressions: Default::default(),
            wrap: None,
            id: 2,
            tool: 1,
            boundary: square(0., 0., 40.),
            islands: vec![hole.clone(), boss.clone()],
            additional_regions: vec![],
            stock_contours: StockContours {
                enabled: true,
                ..Default::default()
            },
            depth: 2.,
            spindle_rpm: 1000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            start: None,
            end: None,
            finishing: Default::default(),
            clear: ClearParams {
                multiple_depths: Default::default(),
                adaptive_enabled: None,
                pocket_overlap: None,
                keep_tool_down: true,
                lift_height: 0.,
                stepdown: 2.,
                overlap: 0.5,
                offset: 0.,
                feed: 300.,
                plunge_feed: 100.,
                plunge: Plunge::Straight,
                lead_in: Lead::None,
                lead_out: Lead::None,
                lead_overlap: 0.,
                clearing: Default::default(),
            },
        };
        let token = CancelToken::new();
        for engagement in [0., 1.] {
            pocket.clear.clearing.engagement = engagement;
            for additional in [false, true] {
                let mut op = pocket.clone();
                if additional {
                    op.additional_regions =
                        vec![
                            cam_geo::Polygon::with_holes(op.boundary.clone(), op.islands.clone())
                                .unwrap(),
                        ];
                    op.boundary = square(-20., 0., 15.);
                    op.islands.clear();
                }
                let result = compute_with_stock(
                    &Operation::Pocket(op),
                    &env,
                    &token,
                    Default::default(),
                    &geometry,
                    Some(CuttingHeights {
                        top: 0.,
                        bottom: -2.,
                    }),
                );
                assert!(!result.has_errors(), "{:?}", result.diagnostics);
                result.program.validate().unwrap();
                assert!(
                    result.program.steps().iter().any(|s| match s {
                        Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting =>
                            (8. ..16.).contains(&to.x) && (8. ..16.).contains(&to.y),
                        _ => false,
                    }),
                    "stock above the descending hole must be cleared"
                );
                crate::adaptive_safety::certify_layer(
                    &result.program,
                    &[cam_geo::Polygon::new(boss.clone()).unwrap()],
                    2.,
                    -2.,
                    cam_geo::GRID_MM,
                    &token,
                )
                .expect("rising boss must remain protected along every cut and link");
            }
        }
        // Boundary/drawing selections keep their explicitly selected islands.
        geometry.bindings.clear();
        let operation = Operation::Pocket(pocket);
        let result = compute_with_stock(
            &operation,
            &env,
            &token,
            Default::default(),
            &geometry,
            None,
        );
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        crate::adaptive_safety::certify_layer(
            &result.program,
            &[cam_geo::Polygon::new(hole).unwrap()],
            2.,
            -2.,
            cam_geo::GRID_MM,
            &token,
        )
        .expect("explicit islands must remain protected");

        // Reopening a project must discard the old fragment that left a core
        // above a floor hole, while still reusing newly generated fragments.
        let mut cache = crate::OperationCache::default();
        cache.compute(&operation, &env, &token, StrategyResult::default);
        let mut saved = serde_json::to_value(cache).unwrap();
        saved["entries"][0]["key"]["pocket_stock_revision"] = 2.into();
        let mut reopened: crate::OperationCache = serde_json::from_value(saved).unwrap();
        reopened.compute(&operation, &env, &token, StrategyResult::default);
        assert_eq!(reopened.stats.computed, 1);
        assert_eq!(reopened.stats.reused, 0);
        reopened.compute(&operation, &env, &token, || {
            panic!("current fragment must be reused")
        });
        assert_eq!(reopened.stats.reused, 1);
    }

    #[test]
    fn stock_allowances_apply_to_pocket_walls_and_face_floor() {
        let tools = [Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flute_length: 20.0,
            kind: ToolKind::EndMill,
            ..Default::default()
        }];
        let env = JobEnv {
            heights: Heights::new(10.0, 5.0, 0.0),
            tools: &tools,
            stock: None,
            spindle: SpindleDir::Cw,
        };
        let cancel = CancelToken::new();
        let geometry = cam_model::GeometryState::default();
        let allowance = cam_model::StockToLeave {
            enabled: true,
            radial: 0.5,
            axial: 0.5,
        };
        let square = || {
            Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(30.0, 0.0),
                Point::new(30.0, 30.0),
                Point::new(0.0, 30.0),
            ])
        };
        let pocket = Operation::Pocket(cam_model::PocketOp {
            maximum_stepover: None,
            parameter_expressions: Default::default(),
            wrap: None,
            additional_regions: Vec::new(),
            stock_contours: Default::default(),
            finishing: Default::default(),
            id: 2,
            tool: 1,
            boundary: square(),
            islands: Vec::new(),
            depth: 2.0,
            spindle_rpm: 1000.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            start: None,
            end: None,
            clear: cam_model::ClearParams {
                multiple_depths: Default::default(),
                adaptive_enabled: None,
                pocket_overlap: None,
                keep_tool_down: false,
                lift_height: 0.0,
                stepdown: 2.0,
                overlap: 0.5,
                offset: 0.2,
                feed: 300.0,
                plunge_feed: 100.0,
                plunge: cam_model::Plunge::Straight,
                lead_in: cam_model::Lead::None,
                lead_out: cam_model::Lead::None,
                lead_overlap: 0.0,
                clearing: Default::default(),
            },
        });
        let pocket_base =
            compute_with_stock(&pocket, &env, &cancel, Default::default(), &geometry, None);
        let pocket_allowed = compute_with_stock(&pocket, &env, &cancel, allowance, &geometry, None);
        assert!(!pocket_base.has_errors(), "{:?}", pocket_base.diagnostics);
        assert!(
            !pocket_allowed.has_errors(),
            "{:?}",
            pocket_allowed.diagnostics
        );
        assert_ne!(
            pocket_allowed.program, pocket_base.program,
            "radial allowance adds to the saved pocket wall offset"
        );
        let min_cut_z = |program: &Program| {
            program
                .steps()
                .iter()
                .filter_map(|step| match step {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(to.z),
                    Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => Some(end.z),
                    _ => None,
                })
                .fold(f64::INFINITY, f64::min)
        };
        assert!((min_cut_z(&pocket_allowed.program) - -1.5).abs() < 1e-9);

        let face = Operation::Face(cam_model::FaceOp {
            absolute_overlap: None,
            multiple_depths: Default::default(),
            parameter_expressions: Default::default(),
            id: 3,
            tool: 1,
            boundary: square(),
            start_offset: 0.0,
            depth: 2.0,
            stepdown: 2.0,
            overlap: 0.25,
            overshoot: 0.0,
            direction: cam_model::Axis::X,
            spindle_rpm: 1000.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            feed: 300.0,
            plunge_feed: 100.0,
        });
        let face_base =
            compute_with_stock(&face, &env, &cancel, Default::default(), &geometry, None);
        let face_allowed = compute_with_stock(
            &face,
            &env,
            &cancel,
            cam_model::StockToLeave {
                enabled: true,
                radial: 0.0,
                axial: 0.5,
            },
            &geometry,
            None,
        );
        assert!(!face_base.has_errors(), "{:?}", face_base.diagnostics);
        assert!(!face_allowed.has_errors(), "{:?}", face_allowed.diagnostics);
        assert!((min_cut_z(&face_base.program) - -2.0).abs() < 1e-9);
        assert!((min_cut_z(&face_allowed.program) - -1.5).abs() < 1e-9);
        let radial_face = compute_with_stock(
            &face,
            &env,
            &cancel,
            cam_model::StockToLeave {
                enabled: true,
                radial: 0.1,
                axial: 0.0,
            },
            &geometry,
            None,
        );
        assert!(radial_face.has_errors());
        assert!(radial_face.diagnostics[0]
            .message
            .contains("radial stock to leave is not supported for Face"));
    }

    #[test]
    fn unsupported_stock_allowances_fail_setup_validation() {
        let mut doc = document(1);
        doc.setup_mut().stock_to_leave.insert(
            0,
            cam_model::StockToLeave {
                enabled: true,
                radial: 0.5,
                axial: 0.25,
            },
        );
        let result = build_job(
            &doc,
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &CancelToken::new(),
        );
        assert_eq!(result.completion, JobCompletion::Failed);
        assert!(result.diagnostics.iter().any(|d| d
            .message
            .contains("supported for Profile, Pocket, Face, Adaptive, Parallel, and Spiral only")));
    }

    #[test]
    fn rectangular_pattern_expands_row_major_with_each_seed_once_and_instance_provenance() {
        let mut doc = document(2);
        doc.setup_mut().patterns.push(cam_model::OperationPattern {
            id: 77,
            name: "Grid".into(),
            members: vec![0, 1],
            kind: cam_model::PatternKind::Rectangular {
                nx: 3,
                ny: 2,
                dx: 20.0,
                dy: -15.0,
            },
            order: cam_model::PatternOrder::ByInstance,
            suppressed: false,
            options: Default::default(),
        });
        let mut cache = crate::OperationCache::default();
        let request = crate::PlanRequest {
            document: &doc,
            spindle_rpm: 1000.0,
            dir: SpindleDir::Cw,
            stock: None,
            tool_change_height: 50.0,
        };
        let result = crate::build_job_cached(request, &CancelToken::new(), &mut cache, |_| {});
        assert_eq!(
            result.completion,
            JobCompletion::Complete,
            "{:?}",
            result.diagnostics
        );
        assert_eq!(
            cache.stats.computed, 2,
            "each source fragment is generated once"
        );
        let holes: Vec<_> = result
            .program
            .steps()
            .iter()
            .filter_map(|step| match step {
                Step::Drill(c) => Some((c.points[0], c.tag)),
                _ => None,
            })
            .collect();
        assert_eq!(holes.len(), 12);
        let expected = [
            [10., 10.],
            [11., 10.],
            [30., 10.],
            [31., 10.],
            [50., 10.],
            [51., 10.],
            [10., -5.],
            [11., -5.],
            [30., -5.],
            [31., -5.],
            [50., -5.],
            [51., -5.],
        ];
        for ((point, tag), [x, y]) in holes.iter().zip(expected) {
            assert!((point[0] - x).abs() < 1e-9 && (point[1] - y).abs() < 1e-9);
            assert_eq!(tag.pattern_id, Some(77));
            assert!(tag.instance_index.is_some());
        }
    }

    #[test]
    fn pattern_selected_entry_is_used_only_for_each_sources_first_executed_instance() {
        let mut doc = document(0);
        doc.setup_mut().tools[0].kind = ToolKind::EndMill;
        let mut selected = profile_operation();
        if let Operation::Profile(op) = &mut selected {
            op.start = Some([30.0, 30.0]);
            op.end = Some([0.0, 30.0]);
            op.lead_in = cam_model::Lead::Arc { radius: 0.5 };
            op.lead_out = cam_model::Lead::Arc { radius: 0.5 };
        }
        doc.setup_mut().operations.push(selected.clone());
        let selected_result = crate::build_job_cached(
            plan_request(&doc),
            &CancelToken::new(),
            &mut crate::OperationCache::default(),
            |_| {},
        );
        doc.setup_mut().operations[0] = automatic_pattern_entry(&selected).unwrap();
        let automatic_result = crate::build_job_cached(
            plan_request(&doc),
            &CancelToken::new(),
            &mut crate::OperationCache::default(),
            |_| {},
        );
        let fragment = |program: &Program| {
            let mut fragment = Program::new();
            for step in program
                .steps()
                .iter()
                .filter(|step| step.tag().is_some_and(|tag| tag.kind != MoveKind::Traverse))
            {
                fragment.push(step.clone());
            }
            fragment
        };
        assert_eq!(
            selected_result.completion,
            JobCompletion::Complete,
            "{:?}",
            selected_result.diagnostics
        );
        assert_eq!(
            automatic_result.completion,
            JobCompletion::Complete,
            "{:?}",
            automatic_result.diagnostics
        );
        let selected_fragment = fragment(&selected_result.program);
        let automatic_fragment = fragment(&automatic_result.program);
        assert_ne!(
            first_position(&selected_fragment),
            first_position(&automatic_fragment)
        );
        doc.setup_mut().operations[0] = selected;
        // A second source keeps its own first entry under every execution order.
        let mut second = doc.setup().operations[0].clone();
        if let Operation::Profile(op) = &mut second {
            op.id = 1;
        }
        doc.setup_mut().operations.push(second);
        for kind in [
            cam_model::PatternKind::Rectangular {
                nx: 2,
                ny: 1,
                dx: 40.0,
                dy: 0.0,
            },
            cam_model::PatternKind::Rectangular {
                nx: 3,
                ny: 1,
                dx: 40.0,
                dy: 0.0,
            },
            cam_model::PatternKind::Circular {
                center_xy: [5.0, -10.0],
                count: 3,
                distribution: cam_model::CircularDistribution::Full,
                direction: cam_model::PatternDirection::Ccw,
            },
        ] {
            for order in [
                cam_model::PatternOrder::ByInstance,
                cam_model::PatternOrder::ByOperation,
                cam_model::PatternOrder::ByTool,
            ] {
                for (keep_original, reverse) in
                    [(true, false), (true, true), (false, false), (false, true)]
                {
                    let pattern = cam_model::OperationPattern {
                        id: 77,
                        name: "Entries".into(),
                        members: vec![0, 1],
                        kind,
                        order,
                        suppressed: false,
                        options: cam_model::PatternOptions {
                            keep_original,
                            reverse,
                            ..Default::default()
                        },
                    };
                    let indices = pattern.options.instance_indices(kind);
                    doc.setup_mut().patterns = vec![pattern.clone()];
                    let original = doc.clone();
                    let mut cache = crate::OperationCache::default();
                    for run in 0..2 {
                        let mut progress = Vec::new();
                        let result = crate::build_job_cached(
                            plan_request(&doc),
                            &CancelToken::new(),
                            &mut cache,
                            |id| progress.push(id),
                        );
                        assert_eq!(
                            result.completion,
                            JobCompletion::Complete,
                            "{:?}",
                            result.diagnostics
                        );
                        assert_eq!(doc, original, "planning must preserve the selected entry");
                        let variants = if indices.len() > 1 { 4 } else { 2 };
                        let expected_progress: &[u32] = if run > 0 {
                            &[]
                        } else if variants == 4 {
                            &[0, 0, 1, 1]
                        } else {
                            &[0, 1]
                        };
                        assert_eq!(progress, expected_progress);
                        assert_eq!(
                            (cache.stats.computed, cache.stats.reused),
                            if run == 0 {
                                (variants, 0)
                            } else {
                                (0, variants)
                            }
                        );
                        let center = match kind {
                            cam_model::PatternKind::Circular { center_xy, .. } => center_xy,
                            _ => [0.0; 2],
                        };
                        for op_id in [0, 1] {
                            for (ordinal, &index) in indices.iter().enumerate() {
                                let mut actual = Program::new();
                                for step in result.program.steps().iter().filter(|step| {
                                    step.tag().is_some_and(|tag| {
                                        tag.op_id == op_id
                                            && tag.pattern_id == Some(77)
                                            && tag.instance_index == Some(index as u32)
                                            && tag.kind != MoveKind::Traverse
                                    })
                                }) {
                                    let mut step = step.clone();
                                    match &mut step {
                                        Step::Rapid { tag, .. }
                                        | Step::Linear { tag, .. }
                                        | Step::Arc { tag, .. } => tag.op_id = 0,
                                        _ => unreachable!(),
                                    }
                                    actual.push(step);
                                }
                                let seed = if ordinal == 0 {
                                    &selected_fragment
                                } else {
                                    &automatic_fragment
                                };
                                let (translation, degrees) =
                                    pattern.options.transform_xy(kind, index).unwrap();
                                let expected = seed
                                    .transformed_xy(center, translation, degrees)
                                    .with_pattern_owner(77, index as u32);
                                assert_eq!(actual, expected, "{kind:?}, {order:?}, {keep_original}, {reverse}, source {op_id}, instance {index}");
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn configured_pattern_applies_extent_direction_original_filter_and_tool_order() {
        let mut doc = document(3);
        let mut second_tool = doc.setup().tools[0].clone();
        second_tool.number = 2;
        doc.setup_mut().tools.push(second_tool);
        if let Operation::Drill(op) = &mut doc.setup_mut().operations[0] {
            op.tool = 2;
        }
        if let Operation::Drill(op) = &mut doc.setup_mut().operations[2] {
            op.tool = 2;
        }
        let mut options = cam_model::PatternOptions {
            spacing: cam_model::PatternSpacing::Extent,
            keep_original: false,
            reverse: true,
            second_direction: false,
            ..Default::default()
        };
        options.directions[0].angle_degrees = 90.0;
        options.directions[0].reversed = true;
        doc.setup_mut().patterns.push(cam_model::OperationPattern {
            id: 81,
            name: "Configured".into(),
            members: vec![0, 1, 2],
            kind: cam_model::PatternKind::Rectangular {
                nx: 3,
                ny: 1,
                dx: 40.0,
                dy: 0.0,
            },
            order: cam_model::PatternOrder::ByTool,
            suppressed: false,
            options,
        });
        let result = build_job(
            &doc,
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &CancelToken::new(),
        );
        assert_eq!(
            result.completion,
            JobCompletion::Complete,
            "{:?}",
            result.diagnostics
        );
        let cycles: Vec<_> = result
            .program
            .steps()
            .iter()
            .filter_map(|step| match step {
                Step::Drill(cycle) => {
                    Some((cycle.tag.op_id, cycle.tag.instance_index, cycle.points[0]))
                }
                _ => None,
            })
            .collect();
        assert_eq!(
            cycles,
            vec![
                (1, Some(2), [11.0, -30.0]),
                (1, Some(1), [11.0, -10.0]),
                (0, Some(2), [10.0, -30.0]),
                (0, Some(1), [10.0, -10.0]),
                (2, Some(2), [12.0, -30.0]),
                (2, Some(1), [12.0, -10.0]),
            ]
        );
    }

    #[test]
    fn adaptive_patterns_fail_before_reusing_a_path_against_unchecked_models() {
        let mut doc = document(0);
        doc.setup_mut()
            .operations
            .push(Operation::Adaptive(cam_model::AdaptiveOp {
                parameter_expressions: Default::default(),
                id: 4,
                tool: 1,
                spindle_rpm: 1000.0,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                params: Default::default(),
            }));
        doc.setup_mut().patterns.push(cam_model::OperationPattern {
            id: 79,
            name: "Repeated adaptive part".into(),
            members: vec![4],
            kind: cam_model::PatternKind::Rectangular {
                nx: 2,
                ny: 1,
                dx: 20.0,
                dy: 0.0,
            },
            order: cam_model::PatternOrder::ByInstance,
            suppressed: false,
            options: Default::default(),
        });
        let result = build_job(
            &doc,
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &CancelToken::new(),
        );
        assert_eq!(result.completion, JobCompletion::Failed);
        assert!(result.diagnostics.iter().any(|diagnostic| diagnostic
            .message
            .contains("cannot be repeated in an operation pattern")));
    }

    #[test]
    fn suppressed_pattern_skips_seed_generation_and_bad_membership_fails_headless_planning() {
        let mut doc = document(1);
        doc.setup_mut().patterns.push(cam_model::OperationPattern {
            id: 78,
            name: "Off".into(),
            members: vec![0],
            kind: cam_model::PatternKind::Circular {
                center_xy: [1.0, 1.0],
                count: 4,
                distribution: cam_model::CircularDistribution::Full,
                direction: cam_model::PatternDirection::Ccw,
            },
            order: cam_model::PatternOrder::ByInstance,
            suppressed: true,
            options: Default::default(),
        });
        let mut cache = crate::OperationCache::default();
        let request = crate::PlanRequest {
            document: &doc,
            spindle_rpm: 1000.0,
            dir: SpindleDir::Cw,
            stock: None,
            tool_change_height: 50.0,
        };
        let result = crate::build_job_cached(request, &CancelToken::new(), &mut cache, |_| {});
        assert_eq!(result.completion, JobCompletion::Complete);
        assert_eq!(cache.stats.computed, 0);
        doc.setup_mut().patterns[0].members.push(404);
        let result = build_job(
            &doc,
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &CancelToken::new(),
        );
        assert_eq!(result.completion, JobCompletion::Failed);
        assert!(result.diagnostics[0]
            .message
            .contains("missing operation 404"));
    }

    #[test]
    fn circular_pattern_rotates_drill_cycles_about_nonzero_center_and_by_operation_orders_instances(
    ) {
        let mut doc = document(1);
        doc.setup_mut().patterns.push(cam_model::OperationPattern {
            id: 79,
            name: "Rotor".into(),
            members: vec![0],
            kind: cam_model::PatternKind::Circular {
                center_xy: [5.0, 5.0],
                count: 4,
                distribution: cam_model::CircularDistribution::Full,
                direction: cam_model::PatternDirection::Ccw,
            },
            order: cam_model::PatternOrder::ByInstance,
            suppressed: false,
            options: Default::default(),
        });
        let result = build_job(
            &doc,
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &CancelToken::new(),
        );
        assert_eq!(
            result.completion,
            JobCompletion::Complete,
            "{:?}",
            result.diagnostics
        );
        let cycles: Vec<_> = result
            .program
            .steps()
            .iter()
            .filter_map(|step| match step {
                Step::Drill(cycle) => Some((cycle.points[0], cycle.tag.instance_index)),
                _ => None,
            })
            .collect();
        assert_eq!(cycles.len(), 4);
        assert_eq!(cycles[0].0, [10.0, 10.0]);
        assert!((cycles[1].0[0] - 0.0).abs() < 1e-9);
        assert!((cycles[1].0[1] - 10.0).abs() < 1e-9);
        assert_eq!(
            cycles.iter().map(|(_, index)| *index).collect::<Vec<_>>(),
            vec![Some(0), Some(1), Some(2), Some(3)]
        );

        let mut ordered = document(2);
        ordered
            .setup_mut()
            .patterns
            .push(cam_model::OperationPattern {
                id: 80,
                name: "Operations".into(),
                members: vec![0, 1],
                kind: cam_model::PatternKind::Rectangular {
                    nx: 2,
                    ny: 1,
                    dx: 3.0,
                    dy: 0.0,
                },
                order: cam_model::PatternOrder::ByOperation,
                suppressed: false,
                options: Default::default(),
            });
        let result = build_job(
            &ordered,
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &CancelToken::new(),
        );
        let ownership: Vec<_> = result
            .program
            .steps()
            .iter()
            .filter_map(|step| match step {
                Step::Drill(cycle) => Some((cycle.tag.op_id, cycle.tag.instance_index)),
                _ => None,
            })
            .collect();
        assert_eq!(
            ownership,
            vec![(0, Some(0)), (0, Some(1)), (1, Some(0)), (1, Some(1))]
        );
    }

    #[test]
    fn reference_heights_drive_cutting_limits_and_independent_feed_with_pecks_and_dwell() {
        use cam_model::{HeightPlaneKind as P, HeightReference as R, OperationHeightSettings};
        let mut doc = document(1);
        if let Operation::Drill(op) = &mut doc.setup_mut().operations[0] {
            op.depth = 4.;
            op.peck = Some(1.);
            op.dwell = Some(0.25);
        }
        let mut settings = OperationHeightSettings::default();
        settings.definition_mut(P::Feed).offset = 2.;
        settings.definition_mut(P::Bottom).offset = -0.5;
        doc.setup_mut().height_settings.insert(0, settings);
        let result = build_job(&doc, 1000., SpindleDir::Cw, None, 50., &CancelToken::new());
        assert_eq!(
            result.completion,
            JobCompletion::Complete,
            "{:?}",
            result.diagnostics
        );
        assert!(!result
            .program
            .steps()
            .iter()
            .any(|s| matches!(s, Step::Drill(_))));
        assert!(result
            .program
            .steps()
            .iter()
            .any(|s| matches!(s,Step::Linear{to,..} if to.z == -4.5)));
        assert!(result
            .program
            .steps()
            .iter()
            .any(|s| matches!(s,Step::Dwell{seconds} if *seconds == 0.25)));
        let mut previous = None;
        for step in result.program.steps() {
            match step {
                Step::Rapid { to, .. } => {
                    if let Some(from) = previous {
                        assert!(
                            to.z >= from || to.z >= 2.,
                            "rapid descended below feed height"
                        );
                    }
                    previous = Some(to.z);
                }
                Step::Linear { to, .. } => previous = Some(to.z),
                _ => {}
            }
        }
        doc.setup_mut()
            .height_settings
            .get_mut(&0)
            .unwrap()
            .definition_mut(P::Feed)
            .reference = R::Disabled;
        let result = build_job(&doc, 1000., SpindleDir::Cw, None, 50., &CancelToken::new());
        assert_eq!(result.completion, JobCompletion::Complete);
        assert!(result
            .program
            .steps()
            .iter()
            .any(|s| matches!(s,Step::Drill(c) if c.depth == -4.5)));
    }

    #[test]
    fn explicit_peck_drilling_keeps_cutting_feed_on_entry_and_rapid_reentry() {
        use cam_model::{DrillCycleMode, HeightPlaneKind as P, OperationHeightSettings};
        let mut doc = document(1);
        let Operation::Drill(drill) = &mut doc.setup_mut().operations[0] else {
            panic!()
        };
        drill.set_cycle_mode(DrillCycleMode::DeepDrilling);
        drill.depth = 3.0;
        let mut settings = OperationHeightSettings::default();
        settings.definition_mut(P::Feed).offset = 1.0;
        doc.setup_mut().height_settings.insert(0, settings);

        let result = build_job(&doc, 1000., SpindleDir::Cw, None, 50., &CancelToken::new());
        assert_eq!(
            result.completion,
            JobCompletion::Complete,
            "{:?}",
            result.diagnostics
        );
        let mut feed_depths = Vec::new();
        let mut resume = Vec::new();
        for step in result.program.steps() {
            match step {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => {
                    feed_depths.push(to.z)
                }
                Step::Rapid { to, tag } if tag.kind == MoveKind::Link && to.z < 0.0 => {
                    resume.push(to.z)
                }
                _ => {}
            }
        }
        assert_eq!(feed_depths, [-0.75, -1.5, -2.25, -3.0]);
        assert_eq!(resume, [-0.25, -1.0, -1.75]);
    }

    #[test]
    fn feed_approach_below_stock_cuts_and_still_checks_nonplunging_tools() {
        let operation = profile_operation();
        let link = Tag {
            pattern_id: Some(2),
            instance_index: Some(0),
            ..Tag::new(operation.id(), MoveKind::Link)
        };
        let program = Program {
            steps: vec![
                Step::ToolChange { tool: 1 },
                Step::Rapid {
                    to: Point3::new(10., 10., 29.),
                    tag: link,
                },
                Step::Rapid {
                    to: Point3::new(10., 10., 1.),
                    tag: link,
                },
            ],
        };
        let approach = apply_feed_height(&program, 14., &operation).unwrap();
        assert!(
            matches!(approach.steps().last(), Some(Step::Linear { tag, .. })
            if tag.kind == MoveKind::Plunge && tag.pattern_id == Some(2)
                && tag.instance_index == Some(0))
        );
        for noncutting_tip_radius in [None, Some(1.)] {
            let result = cam_sim::simulate(
                &approach,
                [0., 0., 0.],
                [20., 20., 14.],
                &cam_sim::SimOptions {
                    resolution: 0.2,
                    tool_radius: 3.,
                },
                &[cam_sim::SimTool {
                    number: 1,
                    profile: cam_sim::ToolProfile::flat(3.),
                    noncutting_tip_radius,
                }],
            );
            if noncutting_tip_radius.is_none() {
                assert!(result.collisions.is_empty(), "{:?}", result.collisions);
                assert!(result.removed_volume > 0.);
            } else {
                assert!(result
                    .collisions
                    .iter()
                    .any(|c| c.kind == cam_sim::CollisionKind::NonCuttingThroughStock));
                assert_eq!(result.removed_volume, 0.);
            }
        }
    }

    #[test]
    fn feed_height_edits_reuse_strategy_fragments_but_rebuild_the_approach() {
        use cam_model::{HeightPlaneKind as P, OperationHeightSettings};
        let mut doc = document(1);
        let mut settings = OperationHeightSettings::default();
        settings.definition_mut(P::Feed).offset = 2.;
        doc.setup_mut().height_settings.insert(0, settings);
        let mut cache = crate::OperationCache::default();
        fn request(doc: &Document) -> PlanRequest<'_> {
            PlanRequest {
                document: doc,
                spindle_rpm: 1000.,
                dir: SpindleDir::Cw,
                stock: None,
                tool_change_height: 50.,
            }
        }
        let first = build_job_cached(request(&doc), &CancelToken::new(), &mut cache, |_| {});
        assert_eq!(first.completion, JobCompletion::Complete);
        assert_eq!(cache.stats.computed, 1);
        doc.setup_mut()
            .height_settings
            .get_mut(&0)
            .unwrap()
            .definition_mut(P::Feed)
            .offset = 3.;
        let next = build_job_cached(request(&doc), &CancelToken::new(), &mut cache, |_| {});
        assert_eq!(next.completion, JobCompletion::Complete);
        assert_eq!(cache.stats.reused, 1);
        assert_ne!(first.program, next.program);
    }

    #[test]
    fn operation_planes_reach_strategies_and_safe_inter_operation_links() {
        let mut doc = document(2);
        doc.setup_mut()
            .operation_heights
            .insert(1, Heights::new(20., 12., 3.));
        let mut seen = Vec::new();
        let result = build_job_with_compute(
            &doc,
            1000.,
            SpindleDir::Cw,
            None,
            10.,
            &CancelToken::new(),
            |op, env, _, _, _, _| {
                seen.push((op.id(), env.heights));
                let mut program = Program::new();
                program.push(Step::Rapid {
                    to: Point3::new(op.id() as f64 * 10., 0., env.heights.clearance),
                    tag: Tag::new(op.id(), MoveKind::Link),
                });
                StrategyResult {
                    program,
                    ..Default::default()
                }
            },
        );
        assert_eq!(result.completion, JobCompletion::Complete);
        assert_eq!(
            seen,
            [
                (0, Heights::new(5., 2., 0.)),
                (1, Heights::new(20., 12., 3.))
            ]
        );
        let rapids: Vec<_> = result
            .program
            .steps()
            .iter()
            .filter_map(|s| match s {
                Step::Rapid { to, .. } => Some(*to),
                _ => None,
            })
            .collect();
        // The tool-change height is raised above *all* operation clearances.
        assert_eq!(rapids[0].z, 20.);
        // Before crossing from op 0 to taller op 1, lift at the previous XY.
        assert!(rapids.windows(2).any(
            |pair| pair[0] == Point3::new(0., 0., 20.) && pair[1] == Point3::new(10., 0., 20.)
        ));
    }

    #[test]
    fn invalid_operation_planes_fail_before_strategy_computation() {
        let mut doc = document(1);
        doc.setup_mut()
            .operation_heights
            .insert(0, Heights::new(1., 2., 0.));
        let result = build_job_with_compute(
            &doc,
            1000.,
            SpindleDir::Cw,
            None,
            50.,
            &CancelToken::new(),
            |_, _, _, _, _, _| panic!("invalid planes must not reach a strategy"),
        );
        assert_eq!(result.completion, JobCompletion::Failed);
        assert!(result.diagnostics.iter().any(|d| d.op == Some(0)));
    }

    #[test]
    fn headless_planning_honors_persistent_manufacturing_suppression() {
        let mut doc = document(2);
        doc.excluded_operations.insert(0);
        let result = build_job(&doc, 1000., SpindleDir::Cw, None, 50., &CancelToken::new());
        assert_eq!(result.completion, JobCompletion::Complete);
        assert!(!result
            .program
            .steps()
            .iter()
            .any(|step| matches!(step, Step::Drill(cycle) if cycle.tag.op_id == 0)));
        assert!(result
            .program
            .steps()
            .iter()
            .any(|step| matches!(step, Step::Drill(cycle) if cycle.tag.op_id == 1)));
        doc.disabled_origins
            .entry(doc.setup().id)
            .or_default()
            .insert(1);
        let result = build_job(&doc, 1000., SpindleDir::Cw, None, 50., &CancelToken::new());
        assert_eq!(result.completion, JobCompletion::Complete);
        assert!(!result
            .program
            .steps()
            .iter()
            .any(|step| matches!(step, Step::Drill(_))));
    }

    fn fragment(operation: &Operation) -> StrategyResult {
        let mut program = Program::new();
        program.push(Step::Rapid {
            to: Point3::new(10.0 + operation.id() as f64, 10.0, 5.0),
            tag: Tag::new(operation.id(), MoveKind::Link),
        });
        program.push(Step::Linear {
            to: Point3::new(10.0 + operation.id() as f64, 10.0, -1.0),
            feed: 100.0,
            tag: Tag::new(operation.id(), MoveKind::Cutting),
        });
        StrategyResult {
            program,
            ..Default::default()
        }
    }

    fn contains_cut(job: &JobResult, op: u32) -> bool {
        job.program.steps().iter().any(|step| {
            matches!(step, Step::Linear { tag, .. }
                if tag.op_id == op && tag.kind == MoveKind::Cutting)
        })
    }

    #[test]
    fn linked_operation_without_binding_fails_instead_of_using_cached_contour() {
        let mut doc = document(1);
        doc.cad_linked_operations.insert(0);
        let job = build_job_with_compute(
            &doc,
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &CancelToken::new(),
            |op, _, _, _, _, _| fragment(op),
        );
        assert_eq!(job.completion, JobCompletion::Failed);
        assert!(!contains_cut(&job, 0));
        assert!(job
            .diagnostics
            .iter()
            .any(|d| d.message.contains("binding is missing")));
    }

    #[test]
    fn invalid_motion_from_a_strategy_fails_the_job_with_operation_provenance() {
        let job = build_job_with_compute(
            &document(1),
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &CancelToken::new(),
            |op, _, _, _, _, _| {
                let mut result = fragment(op);
                if let Step::Linear { feed, .. } = &mut result.program.steps[1] {
                    *feed = f64::NAN;
                }
                result
            },
        );
        assert_eq!(job.completion, JobCompletion::Failed);
        assert_eq!(job.diagnostics.len(), 1);
        assert_eq!(job.diagnostics[0].op, Some(0));
        assert!(job.diagnostics[0].message.contains("feed must be finite"));
    }

    #[test]
    fn pre_cancelled_jobs_never_compute_an_operation() {
        let token = CancelToken::new();
        token.cancel();
        let job = build_job_with_compute(
            &document(1),
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &token,
            |_, _, _, _, _, _| panic!("cancelled work must not begin"),
        );
        assert_eq!(job.completion, JobCompletion::Cancelled);
        assert!(!contains_cut(&job, 0));
    }

    #[test]
    fn strategy_cancellation_discards_its_fragment_and_stops_later_operations() {
        let token = CancelToken::new();
        let mut called = Vec::new();
        let job = build_job_with_compute(
            &document(2),
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &token,
            |op, _, _, _, _, _| {
                called.push(op.id());
                StrategyResult {
                    cancelled: true,
                    ..fragment(op)
                }
            },
        );
        assert!(
            !token.is_cancelled(),
            "the strategy's own flag must propagate"
        );
        assert_eq!(called, [0]);
        assert_eq!(job.completion, JobCompletion::Cancelled);
        assert!(!contains_cut(&job, 0));
        assert!(!contains_cut(&job, 1));
    }

    #[test]
    fn cancellation_during_the_last_operation_retains_only_completed_fragments() {
        let token = CancelToken::new();
        let job = build_job_with_compute(
            &document(2),
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &token,
            |op, _, token, _, _, _| {
                if op.id() == 1 {
                    token.cancel();
                }
                // The planner must observe the token even if a strategy misses it.
                fragment(op)
            },
        );
        assert_eq!(job.completion, JobCompletion::Cancelled);
        assert!(contains_cut(&job, 0));
        assert!(!contains_cut(&job, 1));
    }

    #[test]
    fn failed_fragments_are_omitted_while_diagnostics_and_other_operations_continue() {
        let mut called = Vec::new();
        let job = build_job_with_compute(
            &document(2),
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &CancelToken::new(),
            |op, _, _, _, _, _| {
                called.push(op.id());
                let mut result = fragment(op);
                if op.id() == 0 {
                    result
                        .diagnostics
                        .push(Diagnostic::error("cannot finish this operation"));
                }
                result
            },
        );
        assert_eq!(called, [0, 1]);
        assert_eq!(job.completion, JobCompletion::Failed);
        assert!(!contains_cut(&job, 0));
        assert!(contains_cut(&job, 1));
        assert_eq!(job.diagnostics[0].op, Some(0));
    }

    #[test]
    fn warnings_do_not_make_successful_empty_operations_incomplete() {
        let job = build_job_with_compute(
            &document(1),
            1000.0,
            SpindleDir::Cw,
            None,
            50.0,
            &CancelToken::new(),
            |_, _, _, _, _, _| StrategyResult {
                diagnostics: vec![Diagnostic::warning("nothing to cut")],
                ..Default::default()
            },
        );
        assert_eq!(job.completion, JobCompletion::Complete);
        assert!(!contains_cut(&job, 0));
        assert_eq!(job.diagnostics[0].op, Some(0));
    }

    #[test]
    fn compound_tool_changes_disable_and_restore_mms() {
        for cooling in [cam_model::Cooling::Mms, cam_model::Cooling::None] {
            let mut fragment = Program::new();
            let tag = Tag::new(1, MoveKind::Cutting);
            fragment.push(Step::Linear {
                to: Point3::new(10., 10., -1.),
                feed: 100.,
                tag,
            });
            fragment.push(Step::ToolChange { tool: 2 });
            fragment.push(Step::Linear {
                to: Point3::new(12., 10., -1.),
                feed: 100.,
                tag,
            });
            let planned = Planned {
                op_id: 1,
                clearance: 5.,
                fragment,
                tool: 1,
                rpm: 1000.,
                cooling,
                ends_with: Some(2),
                datum: 1,
                pattern_id: None,
                instance_index: None,
            };
            let mut program = Program::new();
            emit_instance(
                &mut program,
                &planned,
                SpindleDir::Cw,
                &mut EmitState::new(),
                50.,
                5.,
            );
            let steps = program.steps();
            let change = steps
                .iter()
                .position(|s| matches!(s, Step::ToolChange { tool: 2 }))
                .unwrap();
            if cooling == cam_model::Cooling::Mms {
                assert_eq!(steps[change - 1], Step::Coolant(Coolant::Off));
                assert_eq!(steps[change + 1], Step::Coolant(Coolant::Mist));
                assert_eq!(steps.last(), Some(&Step::Coolant(Coolant::Off)));
            } else {
                assert!(!steps
                    .iter()
                    .any(|s| matches!(s, Step::Coolant(Coolant::Flood | Coolant::Mist))));
            }
        }
    }

    #[test]
    fn empty_jobs_are_complete_unless_cancelled() {
        let token = CancelToken::new();
        let empty = document(0);
        let job = build_job(&empty, 1000.0, SpindleDir::Cw, None, 50.0, &token);
        assert_eq!(job.completion, JobCompletion::Complete);
        token.cancel();
        let job = build_job(&empty, 1000.0, SpindleDir::Cw, None, 50.0, &token);
        assert_eq!(job.completion, JobCompletion::Cancelled);
    }
}
