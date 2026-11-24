//! Submit snapshots and publish only matching service results.
use super::*;
use crate::jobs::{JobInputs, JobProgress, JobStamp};
use std::sync::Arc;

impl AppController {
    #[cfg(feature = "gui")]
    pub(crate) fn operation_preview_cache(&self) -> cam_toolpath::OperationCache {
        self.operation_cache.clone()
    }

    /// Session revision increases through edits and undo/redo; it never rolls back.
    pub fn revision(&self) -> u64 {
        self.document_epoch
    }

    /// Change simulation quality without modifying the machining document or history.
    /// Cancel old work so a result with the previous grid cannot publish afterwards.
    pub fn set_simulation_accuracy(&mut self, value: f32) -> bool {
        let value = crate::settings::clamp_simulation_accuracy(value);
        if self.simulation_accuracy == value {
            return false;
        }
        self.simulation_accuracy = value;
        self.cancel_run();
        true
    }

    pub(crate) fn job_inputs(&self) -> JobInputs {
        // Keep the complete validated setup graph in worker snapshots. Exclusion and
        // disabled datums are run filters applied by the planner; pruning source ops
        // here would leave pattern membership dangling.
        let mut document = self.document().clone();
        document.active_setup_id = self.active_setup_id;
        let (min, max) = self.stock_box();
        let frame_error = self.validate_setup_frame(self.setup()).err().or_else(|| {
            if self.setup().stock != Stock::PreviousSetup {
                return None;
            }
            let at = document
                .setups
                .iter()
                .position(|s| s.id == self.active_setup_id)
                .unwrap();
            let start = document.setups[..at]
                .iter()
                .rposition(|s| s.stock != Stock::PreviousSetup)
                .unwrap_or(0);
            document.setups[start..at].iter().find_map(|s| {
                self.validate_setup_frame(s)
                    .err()
                    .map(|e| format!("{}: {e}", s.name))
            })
        });
        JobInputs {
            document: Arc::new(document),
            machine: self.machine.clone(),
            regions: Arc::new(self.regions.clone()),
            open_paths: Arc::new(self.open_paths.clone()),
            min,
            max,
            spindle_rpm: self.defaults.spindle_rpm,
            tool_diameter: self.defaults.tool_diameter,
            frame_error,
            preview_program: None,
            preview_resolution: None,
            toolpath_preview: false,
            display_only: false,
            removal_analysis: false,
            stock_force_surfaces: false,
            simulation_accuracy: self.simulation_accuracy,
        }
    }
    /// Generation, verification and mesh preparation run in the bounded worker lane.
    pub fn start_run(&mut self) -> JobStamp {
        self.automatic_toolpath_run = false;
        let inputs = self.job_inputs();
        self.submit_run(inputs, None)
    }

    /// Apply changed operation configuration without reviewing unchanged later
    /// operations. Those retain their yellow dependency status until a user run.
    pub fn start_automatic_run(&mut self) -> JobStamp {
        let inputs = self.job_inputs();
        self.automatic_toolpath_run = true;
        self.submit_run(inputs, None)
    }

    /// Run the active setup or one of its included operations. Filtering the
    /// immutable snapshot preserves document order, suppression and undo history.
    pub fn start_toolpath_run(
        &mut self,
        operation: Option<u32>,
        recompute: bool,
    ) -> Result<JobStamp, String> {
        self.start_toolpath_run_with_predecessors(operation, recompute, false)
    }

    /// Build an operation simulation from the setup's earlier included cuts so
    /// the selected operation starts with the remaining stock it will encounter.
    pub fn start_toolpath_run_with_predecessors(
        &mut self,
        operation: Option<u32>,
        recompute: bool,
        include_predecessors: bool,
    ) -> Result<JobStamp, String> {
        if self.setups().is_empty() {
            return Err("Create a setup before generating toolpaths.".into());
        }
        let mut inputs = self.job_inputs();
        if let Some(id) = operation {
            let document = Arc::make_mut(&mut inputs.document);
            let operations = &document.setup().operations;
            let target = operations
                .iter()
                .position(|op| op.id() == id)
                .ok_or_else(|| "The selected operation no longer exists.".to_string())?;
            let ids: Vec<u32> = operations
                .iter()
                .enumerate()
                .filter(|(index, op)| {
                    if include_predecessors {
                        *index > target
                    } else {
                        op.id() != id
                    }
                })
                .map(|(_, op)| op.id())
                .collect();
            document.excluded_operations.extend(ids);
        }
        let ids: Vec<_> = inputs
            .document
            .setup()
            .operations
            .iter()
            .filter(|op| {
                inputs
                    .document
                    .operation_is_included(inputs.document.setup().id, op.id())
            })
            .map(Operation::id)
            .collect();
        if ids.is_empty() {
            return Err(
                "No included toolpaths to process. The operation or its origin may be suppressed."
                    .into(),
            );
        }
        let cache = recompute.then(|| {
            let mut cache = self.operation_cache.clone();
            cache.invalidate_operations(&ids);
            cache
        });
        self.automatic_toolpath_run = false;
        Ok(self.submit_run(inputs, cache))
    }

    /// Run exactly one pattern, retaining every included source at each instance.
    pub fn start_pattern_toolpath_run(
        &mut self,
        pattern_id: u64,
        recompute: bool,
    ) -> Result<JobStamp, String> {
        if self.setups().is_empty() {
            return Err("Create a setup before generating toolpaths.".into());
        }
        let mut inputs = self.job_inputs();
        let members: BTreeSet<u32> = inputs
            .document
            .setup()
            .patterns
            .iter()
            .find(|p| p.id == pattern_id)
            .ok_or_else(|| "Pattern no longer exists.".to_string())?
            .members
            .iter()
            .copied()
            .collect();
        let document = Arc::make_mut(&mut inputs.document);
        let ids_all: Vec<u32> = document
            .setup()
            .operations
            .iter()
            .map(Operation::id)
            .collect();
        document
            .excluded_operations
            .extend(ids_all.into_iter().filter(|id| !members.contains(id)));
        let setup_id = document.setup().id;
        let ids: Vec<_> = document
            .setup()
            .operations
            .iter()
            .filter(|op| document.operation_is_included(setup_id, op.id()))
            .map(Operation::id)
            .collect();
        if ids.is_empty() {
            return Err("No included source operations in this pattern.".into());
        }
        let cache = recompute.then(|| {
            let mut cache = self.operation_cache.clone();
            cache.invalidate_operations(&ids);
            cache
        });
        self.automatic_toolpath_run = false;
        Ok(self.submit_run(inputs, cache))
    }

    fn verification_fingerprint(&self, inputs: &JobInputs) -> String {
        // Source blobs are represented by immutable asset revisions/content hashes,
        // not serialized again on every submit. Revisions + exact stamp equality
        // provide the publication guard; this hash is provenance, not cryptography.
        let geometry = &inputs.document.geometry;
        let assets: Vec<_> = geometry
            .assets
            .iter()
            .map(|a| (a.id, a.revision, &a.source_fingerprint))
            .collect();
        let bytes = serde_json::to_vec(&(
            (
                &inputs.document.setups,
                inputs.document.active_setup_id,
                &inputs.document.rotary_indexes,
                env!("CARGO_PKG_VERSION"),
                // Verified programs and stock must reflect current machining rules.
                12u32,
            ),
            &inputs.document.excluded_operations,
            &inputs.document.disabled_origins,
            &geometry.bindings,
            &inputs.document.cad_linked_operations,
            &geometry.instances,
            assets,
            &geometry.drawing_regions,
            &geometry.drawing_paths,
            &inputs.machine,
            inputs.spindle_rpm,
            inputs.tool_diameter,
            inputs.simulation_accuracy,
            inputs.min,
            inputs.max,
            self.post_kind,
        ))
        .expect("job inputs serialize");
        cam_model::GeometryAsset::fingerprint_source(&bytes)
    }

    fn submit_run(
        &mut self,
        inputs: JobInputs,
        cache: Option<cam_toolpath::OperationCache>,
    ) -> JobStamp {
        let fingerprint = self.verification_fingerprint(&inputs);
        self.outcome = None;
        self.nc = None;
        self.jobs.start(
            self.document_epoch,
            fingerprint,
            inputs,
            cache.unwrap_or_else(|| self.operation_cache.clone()),
        )
    }
    #[cfg(test)]
    pub(crate) fn set_run_test_hook(&mut self, hook: Arc<dyn Fn(&CancelToken) + Send + Sync>) {
        self.jobs.set_test_hook(hook);
    }
    /// Returns true only when a current result was installed.
    pub fn poll_run(&mut self) -> bool {
        let Some(result) = self.jobs.poll(self.document_epoch) else {
            return false;
        };
        let fingerprint = result.stamp.input_fingerprint.clone();
        self.operation_cache = result.cache;
        self.outcome = Some(result.outcome);
        self.record_toolpath_fingerprints();
        self.remember_full_verification(Some(&fingerprint));
        self.nc = None;
        true
    }
    pub(super) fn remember_full_verification(&mut self, submitted: Option<&str>) {
        let inputs = self.job_inputs();
        let fingerprint = self.verification_fingerprint(&inputs);
        if submitted.is_some_and(|saved| saved != fingerprint) {
            return;
        }
        let Some(outcome) = self.outcome.as_ref() else {
            return;
        };
        if outcome.completion == JobCompletion::Failed {
            self.saved_verification.remove(&self.active_setup_id);
            return;
        }
        if outcome.completion != JobCompletion::Complete {
            return;
        }
        self.saved_verification.insert(
            self.active_setup_id,
            crate::project::StoredVerification {
                fingerprint,
                program: outcome.program.clone(),
                diagnostics: outcome.diagnostics.clone(),
                collisions: outcome.collisions.clone(),
                stock_snapshot: outcome.stock_snapshot.clone(),
                initial_stock_snapshot: outcome.initial_stock_snapshot.clone(),
                requires_stock_cache: false,
                initial_stock: outcome
                    .initial_stock
                    .as_ref()
                    .filter(|_| outcome.initial_stock_snapshot.is_none())
                    .cloned(),
                stock_vertices: if outcome.stock_snapshot.is_none() {
                    outcome.stock_vertices.clone()
                } else {
                    Vec::new()
                },
                stock_indices: if outcome.stock_snapshot.is_none() {
                    outcome.stock_indices.clone()
                } else {
                    Vec::new()
                },
                stock_machined: if outcome.stock_snapshot.is_none() {
                    outcome.stock_machined.clone()
                } else {
                    Vec::new()
                },
                stock_operations: if outcome.stock_snapshot.is_none() {
                    outcome.stock_operations.clone()
                } else {
                    Vec::new()
                },
                stock_cut_sources: if outcome.stock_snapshot.is_none() {
                    outcome.stock_cut_sources.clone()
                } else {
                    Vec::new()
                },
            },
        );
    }

    pub(super) fn restore_saved_verification(&mut self) -> bool {
        if self.setups().is_empty() {
            return false;
        }
        let inputs = self.job_inputs();
        let fingerprint = self.verification_fingerprint(&inputs);
        let Some(saved) = self.saved_verification.get(&self.active_setup_id) else {
            return false;
        };
        let vertices_valid = |vertices: &[cam_render::MeshVertex], indices: &[u32]| {
            indices.len().is_multiple_of(3)
                && indices.iter().all(|&i| (i as usize) < vertices.len())
                && vertices.iter().all(|v| {
                    v.position
                        .iter()
                        .chain(&v.normal)
                        .chain(&v.color)
                        .all(|n| n.is_finite())
                })
        };
        if saved.fingerprint != fingerprint
            || saved.program.validate().is_err()
            || (saved.requires_stock_cache
                && saved.stock_snapshot.is_none()
                && saved.stock_vertices.is_empty()
                && saved.initial_stock.is_none())
            || !vertices_valid(&saved.stock_vertices, &saved.stock_indices)
            || saved
                .initial_stock
                .as_ref()
                .is_some_and(|(v, i)| !vertices_valid(v, i))
            || saved.stock_machined.len() != saved.stock_vertices.len()
            || saved.stock_operations.len() != saved.stock_vertices.len()
            || (!saved.stock_cut_sources.is_empty()
                && saved.stock_cut_sources.len() != saved.stock_vertices.len())
            || saved
                .collisions
                .iter()
                .any(|c| !c.fraction.is_finite() || c.at.iter().any(|v| !v.is_finite()))
        {
            return false;
        }
        let render = |snapshot: &cam_sim::StockSnapshot, tiled: bool| {
            let (field, floor) = snapshot.restore().ok()?;
            if !field.matches_grid(
                inputs.min,
                inputs.max,
                inputs.stock_resolution(&saved.program),
            ) || floor != inputs.min[2]
            {
                return None;
            }
            let surface = if tiled {
                cam_sim::StockMeshCache::default().mesh_controlled(&field, floor, &|| false)?
            } else {
                field.to_solid_mesh_controlled(floor, &|| false)?
            };
            let vertices = cam_render::stock_mesh_vertices_controlled(
                &surface.positions,
                &surface.normals,
                &inputs.document.setup().frame,
                &|| false,
            )?;
            Some((vertices, surface))
        };
        let (stock_vertices, stock_indices, stock_machined, stock_operations, stock_cut_sources) =
            if let Some(snapshot) = &saved.stock_snapshot {
                let Some((vertices, surface)) = render(snapshot, true) else {
                    return false;
                };
                (
                    vertices,
                    surface.indices,
                    surface.machined,
                    surface.operations,
                    surface.cut_sources.unwrap_or_default(),
                )
            } else {
                (
                    saved.stock_vertices.clone(),
                    saved.stock_indices.clone(),
                    saved.stock_machined.clone(),
                    saved.stock_operations.clone(),
                    saved.stock_cut_sources.clone(),
                )
            };
        let initial_stock = if let Some(snapshot) = &saved.initial_stock_snapshot {
            let Some((vertices, surface)) = render(snapshot, false) else {
                return false;
            };
            Some((vertices, surface.indices))
        } else {
            saved.initial_stock.clone()
        };
        let scene = inputs.program_scene(&saved.program, &CancelToken::new());
        self.outcome = Some(RunOutcome {
            completion: JobCompletion::Complete,
            stamp: Some(JobStamp {
                request_id: 0,
                revision: self.document_epoch,
                input_fingerprint: fingerprint,
            }),
            program: saved.program.clone(),
            diagnostics: saved.diagnostics.clone(),
            scene,
            initial_stock,
            stock_vertices,
            stock_machined,
            stock_operations,
            stock_cut_sources,
            stock_indices,
            stock_snapshot: saved.stock_snapshot.clone(),
            initial_stock_snapshot: saved.initial_stock_snapshot.clone(),
            collisions: saved.collisions.clone(),
            ..Default::default()
        });
        true
    }

    #[cfg(feature = "gui")]
    pub(crate) fn run_preview(&self) -> Option<&RunOutcome> {
        self.jobs.preview()
    }
    #[cfg(feature = "gui")]
    pub(crate) fn run_preview_revision(&self) -> Option<usize> {
        self.jobs.preview_revision()
    }
    pub fn run_pending(&self) -> bool {
        self.jobs.pending()
    }
    pub fn run_needs_poll(&self) -> bool {
        self.jobs.needs_poll()
    }
    pub fn run_progress(&self) -> Option<JobProgress> {
        self.jobs.progress()
    }
    pub fn cancel_run(&mut self) {
        self.jobs.cancel();
        self.outcome = None;
        self.nc = None;
    }
    /// Status follows execution order: changes upstream invalidate every later
    /// operation, even when its independent cutting fragment remains reusable.
    pub fn toolpath_is_up_to_date(&self, id: u32) -> bool {
        self.current_toolpath_fingerprints()
            .get(&id)
            .zip(self.toolpath_fingerprints.get(&id))
            .is_some_and(|(current, saved)| current == saved)
    }

    pub fn toolpath_statuses(&self) -> BTreeMap<u32, bool> {
        self.toolpath_statuses_for(&self.current_toolpath_fingerprints())
    }

    pub(crate) fn toolpath_statuses_for(
        &self,
        fingerprints: &BTreeMap<u32, String>,
    ) -> BTreeMap<u32, bool> {
        fingerprints
            .iter()
            .map(|(id, current)| {
                let ready = self.toolpath_fingerprints.get(id) == Some(current);
                (*id, ready)
            })
            .collect()
    }

    pub(crate) fn current_toolpath_fingerprints(&self) -> BTreeMap<u32, String> {
        let document = self.document();
        let mut result = BTreeMap::new();
        let mut previous_setup = String::new();
        for setup in &document.setups {
            // Common machining configuration; omit names and view state.
            let stock_axis_instance = match setup.stock {
                Stock::FixedRound { axis, .. } | Stock::RelativeRound { axis, .. } => axis
                    .and_then(|axis| match axis {
                        cam_model::StockAxis::Edge(edge) => Some(edge.instance_id),
                        cam_model::StockAxis::Origin { .. } => None,
                    }),
                _ => None,
            };
            let models: Vec<_> = document
                .geometry
                .instances
                .iter()
                .filter(|instance| {
                    setup.model_instances.contains(&instance.id)
                        || setup.fixture_instances.contains(&instance.id)
                        || stock_axis_instance == Some(instance.id)
                })
                .map(|instance| {
                    let asset = document.geometry.asset(instance.asset_id).map(|asset| {
                        (
                            asset.id,
                            asset.revision,
                            &asset.source_fingerprint,
                            &asset.backend,
                        )
                    });
                    (instance, asset)
                })
                .collect();
            let common = serde_json::to_vec(&(
                setup.id,
                &setup.frame,
                &setup.stock,
                (setup.stock == Stock::PreviousSetup).then_some(&previous_setup),
                document.stock_bounds_for(setup),
                (
                    &setup.model_instances,
                    &setup.fixture_instances,
                    setup.fixture_clearance,
                    (!setup.fixture_instances.is_empty())
                        .then_some(cam_toolpath::FIXTURE_ROUTING_REVISION),
                ),
                &setup.extra_origins,
                setup.origin,
                setup.origin_index,
                setup.tool_change_height,
                self.defaults,
                models,
                &document.geometry.drawing_regions,
                &document.geometry.drawing_paths,
            ))
            .expect("toolpath dependencies serialize");
            let common = cam_model::GeometryAsset::fingerprint_source(&common);
            let mut previous = common.clone();
            for op in &setup.operations {
                let id = op.id();
                let tools: Vec<_> = setup
                    .tools
                    .iter()
                    .filter(|tool| op.tools().contains(&tool.number))
                    .collect();
                let geometry_binding = document.geometry.bindings.get(&id);
                let binding =
                    geometry_binding.map(|binding| document.geometry.binding_fingerprint(binding));
                let patterns: Vec<_> = setup
                    .patterns
                    .iter()
                    .filter(|pattern| pattern.members.contains(&id))
                    .collect();
                let mut bytes = serde_json::to_vec(&(
                    &common,
                    op,
                    document.rotary_indexes.get(&id),
                    tools,
                    setup.operation_heights.get(&id).unwrap_or(&setup.heights),
                    setup.height_settings.get(&id),
                    setup.stock_to_leave_for(id),
                    binding,
                    document.cad_linked_operations.contains(&id),
                    document.operation_is_included(setup.id, id),
                    patterns,
                ))
                .expect("operation dependencies serialize");
                if matches!(op, Operation::Parallel(_)) {
                    bytes.extend_from_slice(
                        &cam_toolpath::ParallelStrategy::DOMAIN_REVISION.to_le_bytes(),
                    );
                }
                if matches!(op, Operation::Spiral(_)) {
                    bytes.extend_from_slice(
                        &cam_toolpath::SpiralStrategy::DOMAIN_REVISION.to_le_bytes(),
                    );
                }
                if op.rotary_wrap().is_some() {
                    bytes.extend_from_slice(&cam_toolpath::ROTARY_WRAP_REVISION.to_le_bytes());
                }
                if matches!(op, Operation::Adaptive(_)) {
                    bytes.extend_from_slice(
                        &cam_toolpath::AdaptiveStrategy::DOMAIN_REVISION.to_le_bytes(),
                    );
                }
                if matches!(op, Operation::Pocket(p) if p.stock_contours.enabled) {
                    bytes.extend_from_slice(
                        &cam_toolpath::PocketStrategy::STOCK_CONTOUR_REVISION.to_le_bytes(),
                    );
                }
                if geometry_binding
                    .is_some_and(|b| b.recipe == cam_model::ExtractionRecipe::PocketFloor)
                {
                    bytes.extend_from_slice(&cam_geometry3d::POCKET_FLOOR_REVISION.to_le_bytes());
                }
                if let Some(revision) = cam_toolpath::concentric_clearing_revision(op) {
                    bytes.extend_from_slice(&revision.to_le_bytes());
                }
                let own = cam_model::GeometryAsset::fingerprint_source(&bytes);
                previous = cam_model::GeometryAsset::fingerprint_source(
                    format!("{previous}:{own}").as_bytes(),
                );
                result.insert(id, format!("{own}:{previous}"));
            }
            previous_setup = previous;
        }
        result
    }

    pub(crate) fn record_toolpath_fingerprints(&mut self) {
        let Some(outcome) = &self.outcome else {
            return;
        };
        if outcome.completion != JobCompletion::Complete {
            return;
        }
        let generated: BTreeSet<_> = outcome
            .scene
            .strips
            .iter()
            .filter_map(|strip| strip.op)
            .collect();
        let current = self.current_toolpath_fingerprints();
        for id in generated {
            if let Some(fingerprint) = current.get(&id) {
                let own_unchanged = self
                    .toolpath_fingerprints
                    .get(&id)
                    .is_some_and(|saved| saved.split(':').next() == fingerprint.split(':').next());
                if !self.automatic_toolpath_run || !own_unchanged {
                    self.toolpath_fingerprints.insert(id, fingerprint.clone());
                }
            }
        }
    }

    pub fn cache_stats(&self) -> cam_toolpath::CacheStats {
        self.operation_cache.stats
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{
        atomic::{AtomicUsize, Ordering},
        mpsc, Mutex,
    };
    use std::time::{Duration, Instant};

    fn app() -> AppController {
        let mut app = AppController::new(crate::default_machine());
        app.edit_stock(|stock| {
            *stock = Stock::FixedBox {
                size: [8., 8., 4.],
                model_position: [0.; 3],
            }
        });
        for point in [[2., 2.], [6., 6.]] {
            app.add_operation(Operation::Drill(DrillOp {
                parameter_expressions: Default::default(),
                id: 0,
                tool: 1,
                points: vec![point],
                depth: 2.,
                start_offset: 0.,
                peck: None,
                dwell: None,
                spindle_rpm: 1000.,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                feed: 100.,
            }));
        }
        app
    }
    fn drain(app: &mut AppController) {
        let deadline = Instant::now() + Duration::from_secs(15);
        while app.run_needs_poll() {
            app.poll_run();
            assert!(Instant::now() < deadline, "worker did not finish");
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    fn gate(app: &mut AppController) -> (mpsc::Receiver<CancelToken>, mpsc::Sender<()>) {
        let (entered, receiver) = mpsc::channel();
        let (release, wait) = mpsc::channel();
        let wait = Mutex::new(wait);
        let calls = AtomicUsize::new(0);
        app.set_run_test_hook(Arc::new(move |token| {
            if calls.fetch_add(1, Ordering::Relaxed) == 0 {
                entered.send(token.clone()).unwrap();
                wait.lock()
                    .unwrap()
                    .recv_timeout(Duration::from_secs(15))
                    .unwrap();
            }
        }));
        (receiver, release)
    }
    #[test]
    fn completed_full_setup_verification_is_saved_restored_and_invalidated() {
        let mut app = app();
        let original = app.run(&CancelToken::new()).clone();
        assert_eq!(original.completion, JobCompletion::Complete);
        let path =
            std::env::temp_dir().join(format!("saved-verification-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        let restored = reopened.outcome().expect("verification restored on open");
        assert_eq!(restored.program, original.program);
        assert_eq!(restored.collisions, original.collisions);
        assert_eq!(restored.stock_indices, original.stock_indices);
        assert_eq!(restored.stock_vertices, original.stock_vertices);
        assert!(!reopened.run_pending());
        assert_eq!(reopened.export_nc().is_ok(), app.export_nc().is_ok());
        reopened.edit_operation(0, |op| {
            if let Operation::Drill(d) = op {
                d.depth = 1.;
            }
        });
        assert!(!reopened.restore_saved_verification());
        assert!(reopened.outcome().is_none());
        let mut different_machine = AppController::new(crate::default_machine());
        different_machine.edit_machine(|m| m.envelope.max.z += 1.);
        different_machine.open_project(&path).unwrap();
        assert!(different_machine.outcome().is_none());
        let mut different_index = AppController::new(crate::default_machine());
        different_index.open_project(&path).unwrap();
        different_index.edit_document(|doc| {
            doc.rotary_indexes.insert(
                0,
                cam_model::RotaryIndex {
                    angle: 30.,
                    ..Default::default()
                },
            );
        });
        assert!(!different_index.restore_saved_verification());
        let mut different_accuracy = AppController::new(crate::default_machine());
        different_accuracy.set_simulation_accuracy(0.5);
        different_accuracy.open_project(&path).unwrap();
        assert!(different_accuracy.outcome().is_none());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn inherited_stock_snapshots_restore_both_input_and_result() {
        let mut app = app();
        let first = app.active_setup_id();
        let second = app.add_setup();
        app.edit_setup(|s| s.stock = Stock::PreviousSetup);
        let original = app.run(&CancelToken::new()).clone();
        assert_eq!(original.completion, JobCompletion::Complete);
        assert!(original.initial_stock_snapshot.is_some());
        assert!(original.stock_snapshot.is_some());
        let path =
            std::env::temp_dir().join(format!("inherited-snapshots-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        reopened.activate_setup(second).unwrap();
        let restored = reopened.outcome().expect("saved inherited setup restored");
        assert_eq!(restored.initial_stock, original.initial_stock);
        assert_eq!(restored.stock_vertices, original.stock_vertices);
        assert_eq!(restored.stock_indices, original.stock_indices);
        assert_eq!(restored.stock_machined, original.stock_machined);
        assert_eq!(restored.stock_operations, original.stock_operations);
        assert_eq!(restored.stock_cut_sources, original.stock_cut_sources);
        assert_eq!(restored.program, original.program);
        reopened.activate_setup(first).unwrap();
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn restored_collision_uses_current_export_preference() {
        use serde_json::json;

        let mut app = app();
        app.run(&CancelToken::new());
        app.outcome
            .as_mut()
            .unwrap()
            .collisions
            .push(cam_sim::Collision {
                kind: cam_sim::CollisionKind::RapidThroughStock,
                at: [1., 2., 3.],
                message: "saved collision".into(),
                op_id: Some(0),
                pattern_id: None,
                instance_index: None,
                step_index: Some(1),
                fraction: 0.5,
            });
        app.remember_full_verification(None);
        let path = std::env::temp_dir().join(format!(
            "collision-verification-{}.ocam",
            std::process::id()
        ));
        app.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        assert_eq!(
            reopened.outcome().unwrap().collisions[0].message,
            "saved collision"
        );
        assert!(matches!(
            reopened.export_nc(),
            Err(ExportError::RapidThroughStock(_))
        ));
        let revision = reopened.revision();
        let mut session = crate::mcp::Session {
            controller: reopened,
            ..Default::default()
        };
        assert_eq!(
            session.call("get_run_status", json!({})).unwrap()["outcome"]["collisions"][0]
                ["severity"],
            "Error"
        );
        session
            .call(
                "update_preferences",
                json!({"patch":{"block_export_on_verification_errors":false}}),
            )
            .unwrap();
        assert_eq!(session.controller.revision(), revision);
        assert_eq!(
            session.call("get_run_status", json!({})).unwrap()["outcome"]["collisions"][0]
                ["severity"],
            "Warning"
        );
        let nc = session.call("export_nc", json!({})).unwrap();
        let nc_path = path.with_extension("nc");
        session.call("export_nc", json!({"path":nc_path})).unwrap();
        assert_eq!(
            std::fs::read_to_string(&nc_path).unwrap(),
            nc.as_str().unwrap()
        );

        session.call("restore_preferences", json!({})).unwrap();
        assert!(session.controller.exported_nc().is_none());
        assert!(matches!(
            session.controller.export_nc(),
            Err(ExportError::RapidThroughStock(1))
        ));
        std::fs::remove_file(nc_path).unwrap();
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn verification_is_restored_per_setup_and_rejects_invalid_saved_mesh() {
        let mut app = app();
        let first = app.active_setup_id();
        let second = app.add_setup();
        app.activate_setup(first).unwrap();
        let first_program = app.run(&CancelToken::new()).program.clone();
        app.activate_setup(second).unwrap();
        let second_program = app.run(&CancelToken::new()).program.clone();
        let path =
            std::env::temp_dir().join(format!("setup-verification-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        assert_eq!(reopened.outcome().unwrap().program, first_program);
        reopened.activate_setup(second).unwrap();
        assert_eq!(reopened.outcome().unwrap().program, second_program);
        reopened.activate_setup(first).unwrap();
        assert_eq!(reopened.outcome().unwrap().program, first_program);
        reopened
            .saved_verification
            .get_mut(&first)
            .unwrap()
            .stock_indices
            .push(u32::MAX);
        reopened.cancel_run();
        assert!(!reopened.restore_saved_verification());
        assert!(reopened.outcome().is_none());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn scoped_verification_does_not_replace_full_setup_result() {
        let mut app = app();
        let full = app.run(&CancelToken::new()).program.clone();
        app.start_toolpath_run(Some(0), true).unwrap();
        drain(&mut app);
        let path =
            std::env::temp_dir().join(format!("scoped-verification-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        assert_eq!(reopened.outcome().unwrap().program, full);
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn toolpaths_and_status_survive_reopen_without_recalculation() {
        let mut app = app();
        assert!(!app.toolpath_is_up_to_date(0));
        let program = app.run(&CancelToken::new()).program.clone();
        assert!(app.toolpath_is_up_to_date(0));
        assert!(app.toolpath_is_up_to_date(1));
        let path = std::env::temp_dir().join(format!("toolpath-cache-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        assert!(reopened.toolpath_is_up_to_date(0));
        assert!(reopened.toolpath_is_up_to_date(1));
        assert_eq!(reopened.run(&CancelToken::new()).program, program);
        assert_eq!(
            reopened.cache_stats(),
            cam_toolpath::CacheStats {
                reused: 2,
                computed: 0
            }
        );
        reopened.start_toolpath_run(Some(0), true).unwrap();
        drain(&mut reopened);
        assert_eq!(
            reopened.cache_stats(),
            cam_toolpath::CacheStats {
                reused: 0,
                computed: 1
            }
        );
        reopened.new_project();
        assert!(reopened.toolpath_fingerprints.is_empty());
    }

    #[test]
    fn operation_status_tracks_upstream_edits_order_and_undo() {
        let mut app = app();
        app.run(&CancelToken::new());
        app.edit_operation(1, |op| {
            if let Operation::Drill(drill) = op {
                drill.depth = 1.;
            }
        });
        assert!(
            app.toolpath_is_up_to_date(0),
            "later edits cannot invalidate earlier operations"
        );
        assert!(!app.toolpath_is_up_to_date(1));
        assert!(app.undo());
        assert!(app.toolpath_is_up_to_date(1));
        app.edit_operation(0, |op| {
            if let Operation::Drill(drill) = op {
                drill.depth = 1.;
            }
        });
        assert!(!app.toolpath_is_up_to_date(0));
        assert!(
            !app.toolpath_is_up_to_date(1),
            "upstream changes mark unchanged later paths yellow"
        );
        app.start_toolpath_run(Some(0), false).unwrap();
        drain(&mut app);
        assert!(app.toolpath_is_up_to_date(0));
        assert!(
            !app.toolpath_is_up_to_date(1),
            "a scoped job cannot mark unrelated operations green"
        );
        app.start_toolpath_run(Some(1), false).unwrap();
        drain(&mut app);
        assert!(app.toolpath_is_up_to_date(1));
        assert_eq!(
            app.cache_stats().computed,
            0,
            "unchanged cutting fragments remain reusable"
        );
        app.move_operation(1, true);
        assert!(!app.toolpath_is_up_to_date(0));
        assert!(!app.toolpath_is_up_to_date(1));
    }

    #[test]
    fn automatic_calculation_keeps_later_paths_yellow_until_user_runs_them() {
        let mut app = app();
        app.run(&CancelToken::new());
        app.edit_operation(0, |op| {
            if let Operation::Drill(drill) = op {
                drill.depth = 1.;
            }
        });
        app.start_automatic_run();
        drain(&mut app);
        assert!(app.toolpath_is_up_to_date(0));
        assert!(!app.toolpath_is_up_to_date(1));
        assert_eq!(
            app.cache_stats(),
            cam_toolpath::CacheStats {
                reused: 1,
                computed: 1
            }
        );
        app.start_toolpath_run(Some(1), true).unwrap();
        drain(&mut app);
        assert!(app.toolpath_is_up_to_date(1));
    }

    #[test]
    fn stale_status_and_old_fragments_survive_save_and_reopen() {
        let mut app = app();
        app.run(&CancelToken::new());
        app.edit_operation(0, |op| {
            if let Operation::Drill(drill) = op {
                drill.depth = 1.;
            }
        });
        let path =
            std::env::temp_dir().join(format!("stale-toolpaths-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert!(!reopened.toolpath_is_up_to_date(0));
        assert!(!reopened.toolpath_is_up_to_date(1));
        reopened.run(&CancelToken::new());
        assert_eq!(
            reopened.cache_stats(),
            cam_toolpath::CacheStats {
                reused: 1,
                computed: 1
            }
        );
        assert!(reopened.toolpath_is_up_to_date(0));
        assert!(reopened.toolpath_is_up_to_date(1));
    }
    #[test]
    fn simulation_accuracy_changes_worker_inputs_without_editing_the_project() {
        let mut app = app();
        let document = app.document().clone();
        let revision = app.revision();
        assert!(!app.set_simulation_accuracy(1.0));
        assert!(app.set_simulation_accuracy(2.0));
        assert_eq!(app.job_inputs().simulation_accuracy, 2.0);
        assert_eq!(app.document(), &document);
        assert_eq!(app.revision(), revision);
        app.start_run();
        assert!(app.set_simulation_accuracy(0.5));
        drain(&mut app);
        assert!(
            app.outcome().is_none(),
            "a previous-quality result cannot publish"
        );
        assert_eq!(app.job_inputs().simulation_accuracy, 0.5);
    }

    #[test]
    fn context_jobs_scope_generation_export_and_forced_recompute() {
        let mut app = app();
        let original = app.document().clone();
        app.start_toolpath_run(Some(1), false).unwrap();
        drain(&mut app);
        let outcome = app.outcome().unwrap();
        assert_eq!(outcome.completion, JobCompletion::Complete);
        assert!(outcome
            .scene
            .strips
            .iter()
            .filter_map(|s| s.op)
            .all(|id| id == 1));
        assert!(app.export_nc().is_ok());
        assert_eq!(app.document(), &original);
        app.start_toolpath_run(None, false).unwrap();
        drain(&mut app);
        assert_eq!(
            app.cache_stats(),
            cam_toolpath::CacheStats {
                reused: 1,
                computed: 1
            }
        );
        app.start_toolpath_run(Some(1), true).unwrap();
        drain(&mut app);
        assert_eq!(
            app.cache_stats(),
            cam_toolpath::CacheStats {
                reused: 0,
                computed: 1
            }
        );
        app.start_toolpath_run(None, false).unwrap();
        drain(&mut app);
        assert_eq!(app.cache_stats().reused, 2);
        app.set_operation_excluded(1, true);
        assert!(app.start_toolpath_run(Some(1), false).is_err());
        app.start_toolpath_run(None, true).unwrap();
        drain(&mut app);
        assert!(app
            .outcome()
            .unwrap()
            .scene
            .strips
            .iter()
            .filter_map(|s| s.op)
            .all(|id| id == 0));
        app.set_origin_disabled(1, true);
        assert!(app.start_toolpath_run(None, false).is_err());
    }

    #[test]
    fn pattern_job_scope_keeps_full_membership_and_skips_suppressed_bad_cad_sources() {
        let mut app = app();
        let setup_id = app.active_setup_id();
        let ids: Vec<_> = app
            .document()
            .setup()
            .operations
            .iter()
            .map(Operation::id)
            .collect();
        let kind = cam_model::PatternKind::Rectangular {
            nx: 2,
            ny: 1,
            dx: 10.0,
            dy: 0.0,
        };
        let first = app
            .create_pattern(
                setup_id,
                &[ids[0]],
                "First".into(),
                kind,
                cam_model::PatternOrder::ByInstance,
            )
            .unwrap();
        let second = app
            .create_pattern(
                setup_id,
                &[ids[1]],
                "Second".into(),
                kind,
                cam_model::PatternOrder::ByOperation,
            )
            .unwrap();
        app.edit_document(|doc| {
            doc.cad_linked_operations.insert(ids[1]);
        });
        app.set_pattern_suppressed(setup_id, second, true);
        let full = app.job_inputs();
        assert_eq!(
            full.document.setup().patterns.len(),
            2,
            "worker snapshots retain both groups"
        );
        assert!(full
            .document
            .setup()
            .patterns
            .iter()
            .find(|p| p.id == second)
            .unwrap()
            .members
            .contains(&ids[1]));

        app.start_pattern_toolpath_run(first, false).unwrap();
        drain(&mut app);
        let outcome = app.outcome().unwrap();
        assert_eq!(
            outcome.completion,
            JobCompletion::Complete,
            "excluded stale CAD sources are not resolved: {:?}",
            outcome.diagnostics
        );
        assert!(outcome
            .scene
            .strips
            .iter()
            .filter_map(|strip| strip.op)
            .all(|id| id == ids[0]));
        assert!(app
            .document()
            .setup()
            .patterns
            .iter()
            .any(|p| p.id == second && p.suppressed));

        app.start_run();
        drain(&mut app);
        assert_eq!(app.outcome().unwrap().completion, JobCompletion::Complete);
        assert!(app
            .outcome()
            .unwrap()
            .scene
            .strips
            .iter()
            .filter_map(|strip| strip.op)
            .all(|id| id == ids[0]));
    }
    #[test]
    fn long_worker_leaves_controller_responsive_and_edit_cancels_it() {
        let mut app = app();
        let (entered, release) = gate(&mut app);
        let stamp = app.start_run();
        let token = entered.recv_timeout(Duration::from_secs(5)).unwrap();
        assert!(app.run_pending());
        assert!(app.run_progress().is_some());
        app.select(Selection::Setup); // completes while worker is deliberately blocked
        assert_eq!(app.revision(), stamp.revision);
        app.edit_operation(0, |op| {
            if let Operation::Drill(d) = op {
                d.depth = 1.;
            }
        });
        assert!(app.revision() > stamp.revision);
        assert!(token.is_cancelled());
        assert_eq!(app.export_nc(), Err(ExportError::NothingToExport));
        release.send(()).unwrap();
        drain(&mut app);
        assert!(
            app.outcome().is_none(),
            "late cancelled result cannot publish"
        );
        assert!(!app.run_pending());
    }
    #[test]
    fn cancelling_forced_recompute_preserves_existing_paths_and_statuses() {
        let mut app = app();
        app.run(&CancelToken::new());
        let (entered, release) = gate(&mut app);
        app.start_toolpath_run(Some(1), true).unwrap();
        let token = entered.recv_timeout(Duration::from_secs(5)).unwrap();
        assert!(app.toolpath_is_up_to_date(0));
        assert!(app.toolpath_is_up_to_date(1));
        app.cancel_run();
        assert!(token.is_cancelled());
        release.send(()).unwrap();
        drain(&mut app);
        app.run(&CancelToken::new());
        assert_eq!(app.cache_stats().reused, 2);
        assert_eq!(app.cache_stats().computed, 0);
    }
    #[test]
    fn replacement_job_is_latest_and_only_it_can_publish() {
        let mut app = app();
        let (entered, release) = gate(&mut app);
        let old = app.start_run();
        let token = entered.recv_timeout(Duration::from_secs(5)).unwrap();
        app.edit_operation(0, |op| {
            if let Operation::Drill(d) = op {
                d.depth = 1.;
            }
        });
        let new = app.start_run();
        assert!(new.request_id > old.request_id);
        assert_ne!(new.input_fingerprint, old.input_fingerprint);
        assert!(token.is_cancelled());
        release.send(()).unwrap();
        drain(&mut app);
        assert_eq!(app.outcome().unwrap().completion, JobCompletion::Complete);
        let asynchronous = app.outcome().unwrap().program.clone();
        assert_eq!(asynchronous, app.run(&CancelToken::new()).program);
    }
    #[test]
    fn explicit_cancel_and_machine_post_changes_reject_late_results() {
        for change in 0..3 {
            let mut app = app();
            let (entered, release) = gate(&mut app);
            app.start_run();
            let token = entered.recv_timeout(Duration::from_secs(5)).unwrap();
            match change {
                0 => app.cancel_run(),
                1 => app.edit_machine(|m| m.envelope.max.z += 1.),
                _ => app.set_post_kind(PostKind::Fanuc),
            }
            assert!(token.is_cancelled());
            release.send(()).unwrap();
            drain(&mut app);
            assert!(app.outcome().is_none());
            assert_eq!(app.export_nc(), Err(ExportError::NothingToExport));
        }
    }
    #[test]
    fn caches_track_actual_dependencies_and_reassemble_execution_order() {
        let mut app = app();
        let first = app.run(&CancelToken::new()).program.clone();
        assert_eq!(app.cache_stats().computed, 2);
        assert_eq!(app.run(&CancelToken::new()).program, first);
        assert_eq!(app.cache_stats().reused, 2);
        app.edit_operation(0, |op| {
            if let Operation::Drill(d) = op {
                d.depth = 1.;
            }
        });
        app.run(&CancelToken::new());
        assert_eq!(
            app.cache_stats(),
            cam_toolpath::CacheStats {
                reused: 1,
                computed: 1
            }
        );
        assert!(app.undo());
        assert_eq!(app.run(&CancelToken::new()).program, first);
        assert_eq!(app.cache_stats().reused, 2);
        assert!(app.redo());
        app.run(&CancelToken::new());
        assert_eq!(app.cache_stats().reused, 2);
        app.move_operation(1, true);
        let reordered = app.run(&CancelToken::new()).program.clone();
        assert_eq!(app.cache_stats().reused, 2);
        assert_ne!(reordered, first);
        app.edit_tool(0, |t| t.diameter += 0.5);
        app.run(&CancelToken::new());
        assert_eq!(app.cache_stats().computed, 2);
        app.edit_heights(|h| h.clearance += 1.);
        app.run(&CancelToken::new());
        assert_eq!(
            app.cache_stats().reused,
            2,
            "operation reference heights are independent of legacy setup defaults"
        );
        app.edit_stock(|s| {
            if let Stock::FixedBox { size, .. } = s {
                size[0] += 1.;
            }
        });
        app.run(&CancelToken::new());
        assert_eq!(app.cache_stats().computed, 2);
        app.edit_machine(|m| m.envelope.max.z += 10.);
        assert!(app.outcome().is_none());
        app.run(&CancelToken::new());
        assert_eq!(
            app.cache_stats().reused,
            2,
            "machine changes rebuild transitions, not cutting fragments"
        );
    }
    #[test]
    fn suppression_is_manufacturing_state_across_history_setups_and_reopen() {
        let mut app = app();
        app.set_operation_excluded(0, true);
        assert!(app.undo());
        assert!(!app.is_operation_excluded(0));
        assert!(app.redo());
        assert!(app.is_operation_excluded(0));
        let first_setup = app.active_setup_id();
        app.set_origin_disabled(1, true);
        let second = app.add_setup();
        assert!(!app.is_origin_disabled(1));
        app.activate_setup(first_setup).unwrap();
        assert!(app.is_origin_disabled(1));
        let path = std::env::temp_dir().join(format!("m1-suppression-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert!(reopened.is_operation_excluded(0));
        assert!(reopened.is_origin_disabled(1));
        reopened.activate_setup(second).unwrap();
        assert!(!reopened.is_origin_disabled(1));
    }
    #[test]
    fn deleted_and_undone_ids_are_not_reused() {
        let mut app = app();
        let template = app.operation(1).unwrap().clone();
        app.select(Selection::Operation(1));
        app.delete_selected_operation();
        app.add_operation(template.clone());
        assert_eq!(app.selected_operation().unwrap().id(), 2);
        assert!(app.undo());
        app.add_operation(template);
        assert_eq!(app.selected_operation().unwrap().id(), 3);
        let second = app.add_setup();
        app.delete_setup(second).unwrap();
        assert!(app.add_setup() > second);
    }
    #[test]
    fn worker_panics_fail_closed_with_a_diagnostic_code() {
        let mut app = app();
        app.set_run_test_hook(Arc::new(|_| panic!("injected worker failure")));
        app.start_run();
        drain(&mut app);
        let result = app.outcome().unwrap();
        assert_eq!(result.completion, JobCompletion::Failed);
        assert_eq!(
            result.diagnostics[0].code,
            cam_toolpath::DiagnosticCode::WorkerFailed
        );
        assert_eq!(app.export_nc(), Err(ExportError::HasErrors));
    }
    #[test]
    fn repeated_requests_coalesce_to_one_queued_worker() {
        let mut app = app();
        let (entered, release) = gate(&mut app);
        let first = app.start_run();
        entered.recv_timeout(Duration::from_secs(5)).unwrap();
        let mut newest = first;
        for _ in 0..100 {
            newest = app.start_run();
        }
        release.send(()).unwrap();
        drain(&mut app);
        assert_eq!(app.outcome().unwrap().stamp.as_ref(), Some(&newest));
        assert_eq!(app.outcome().unwrap().completion, JobCompletion::Complete);
    }
    #[test]
    fn save_after_undo_preserves_id_marks_on_reopen() {
        let mut app = app();
        let template = app.operation(1).unwrap().clone();
        app.add_operation(template.clone()); // allocate 2
        app.undo();
        let path = std::env::temp_dir().join(format!("m1-id-marks-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        reopened.add_operation(template);
        assert_eq!(reopened.selected_operation().unwrap().id(), 3);
    }
    #[test]
    fn unresolved_geometry_diagnostics_preserve_operation_ownership() {
        let mut app = app();
        app.edit_document(|doc| {
            doc.cad_linked_operations.insert(0);
        });
        let setup = app.active_setup_id();
        let result = app.run(&CancelToken::new());
        assert_eq!(result.completion, JobCompletion::Failed);
        let diagnostic = &result.diagnostics[0];
        assert_eq!(
            diagnostic.code,
            cam_toolpath::DiagnosticCode::GeometryUnresolved
        );
        assert_eq!(diagnostic.source.setup, Some(setup));
        assert_eq!(diagnostic.source.operation, Some(0));
        assert_eq!(diagnostic.op, Some(0));
    }
}
