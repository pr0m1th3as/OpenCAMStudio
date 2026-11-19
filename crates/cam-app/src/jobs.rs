//! One bounded worker lane, one replaceable pending request. Edits cancel all
//! phases; request/revision/fingerprint guards are checked before publication.
use crate::controller::{narrowest_cut_width, sim_profile, MAX_SIM_CELLS_ACROSS};
use crate::RunOutcome;
use cam_cldata::{Program, SpindleDir};
use cam_geo::{Polygon, Polyline};
use cam_model::{Document, Machine};
use cam_render::{MeshVertex, Scene, PART};
use cam_sim::{Collision, SimOptions, SimTool, SimulationCache};
use cam_toolpath::{
    build_job_cached_with_preview, CancelToken, Diagnostic, DiagnosticCode, JobCompletion,
    OperationCache, PlanRequest,
};
use std::sync::{mpsc, Arc, Mutex};

#[path = "job_simulation.rs"]
pub(crate) mod simulation;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JobStamp {
    pub request_id: u64,
    pub revision: u64,
    pub input_fingerprint: String,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum JobPhase {
    #[default]
    Queued,
    GeometryResolution,
    Generation,
    Verification,
    RenderPreparation,
}
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct JobProgress {
    pub phase: JobPhase,
    pub completed: usize,
    pub total: usize,
    pub operation: Option<u32>,
    pub calculated_operations: Vec<u32>,
}
#[derive(Clone, Default)]
pub(crate) struct Progress(Arc<Mutex<JobProgress>>);
impl Progress {
    pub fn set(&self, phase: JobPhase, completed: usize, total: usize, operation: Option<u32>) {
        let mut progress = self.0.lock().unwrap();
        progress.phase = phase;
        progress.completed = completed;
        progress.total = total;
        progress.operation = operation;
    }
    fn calculated(&self, id: u32, success: bool) {
        let mut progress = self.0.lock().unwrap();
        progress.calculated_operations.retain(|old| *old != id);
        if success {
            progress.calculated_operations.push(id);
        }
    }
    fn get(&self) -> JobProgress {
        self.0.lock().unwrap().clone()
    }
}

/// Only owned immutable domain data crosses the worker boundary. Imported
/// source bytes/topology remain Arc snapshots; CAD/GPU handles never cross it.
#[derive(Clone)]
pub(crate) struct JobInputs {
    pub document: Arc<Document>,
    pub machine: Machine,
    pub regions: Arc<Vec<Polygon>>,
    pub open_paths: Arc<Vec<Polyline>>,
    pub min: [f64; 3],
    pub max: [f64; 3],
    pub spindle_rpm: f64,
    pub tool_diameter: f64,
    pub frame_error: Option<String>,
    /// A playback prefix bypasses planning; it never replaces the verified job.
    pub preview_program: Option<Program>,
    pub simulation_accuracy: f32,
    /// Fixed from the full playback program so partial frames share one grid.
    pub preview_resolution: Option<f64>,
    /// Generate editable source paths without simulating stock removal.
    pub toolpath_preview: bool,
    /// Skip fixture verification for browser previews.
    pub display_only: bool,
    /// Sample actual removal along an existing program, without meshing/export changes.
    pub removal_analysis: bool,
    /// Detailed cut-face provenance, only needed for force-colored stock playback.
    pub stock_force_surfaces: bool,
}
impl JobInputs {
    pub fn execute(
        &self,
        cancel: &CancelToken,
        cache: &mut OperationCache,
        progress: &Progress,
        simulation: &mut simulation::StockSimulationCache,
    ) -> RunOutcome {
        self.execute_with_preview(cancel, cache, progress, simulation, None)
    }

    fn execute_with_preview(
        &self,
        cancel: &CancelToken,
        cache: &mut OperationCache,
        progress: &Progress,
        simulation: &mut simulation::StockSimulationCache,
        mut publish_preview: Option<&mut dyn FnMut(PreviewUpdate)>,
    ) -> RunOutcome {
        cache.reset_stats();
        if self.document.setups.is_empty() {
            return RunOutcome {
                completion: JobCompletion::Failed,
                diagnostics: vec![
                    Diagnostic::error("Create a setup before generating toolpaths.")
                        .with_code(DiagnosticCode::InvalidInput),
                ],
                ..Default::default()
            };
        }
        if cancel.is_cancelled() {
            return RunOutcome {
                completion: JobCompletion::Cancelled,
                ..Default::default()
            };
        }
        if self.removal_analysis {
            return match self.analyze_stock_removal(cancel, cache, progress) {
                Ok(samples) => RunOutcome {
                    completion: JobCompletion::Complete,
                    removal_samples: Arc::new(samples),
                    ..Default::default()
                },
                Err(error) => RunOutcome {
                    completion: if cancel.is_cancelled() {
                        JobCompletion::Cancelled
                    } else {
                        JobCompletion::Failed
                    },
                    diagnostics: vec![error],
                    ..Default::default()
                },
            };
        }
        let mut streamed = false;
        let mut job = if let Some(program) = &self.preview_program {
            cam_toolpath::JobResult {
                program: program.clone(),
                diagnostics: Vec::new(),
                completion: JobCompletion::Complete,
            }
        } else {
            progress.set(JobPhase::GeometryResolution, 0, 1, None);
            let setup_id = self.document.setup().id;
            let resolved = self.frame_error.clone().map_or_else(
                || {
                    self.machine
                        .validate()
                        .map_err(|error| {
                            Diagnostic::error(error.to_string())
                                .with_code(DiagnosticCode::InvalidInput)
                                .for_setup(setup_id)
                        })
                        .and_then(|()| {
                            crate::cad_operations::resolve_document_checked(&self.document, &|| {
                                cancel.is_cancelled()
                            })
                        })
                },
                |error| {
                    Err(Diagnostic::error(error)
                        .with_code(DiagnosticCode::GeometryUnresolved)
                        .for_setup(setup_id))
                },
            );
            if cancel.is_cancelled() {
                return RunOutcome {
                    completion: JobCompletion::Cancelled,
                    ..Default::default()
                };
            }
            let job = match resolved {
                Ok(document) => {
                    let total = document
                        .setup()
                        .operations
                        .iter()
                        .filter(|op| document.operation_is_included(setup_id, op.id()))
                        .count();
                    let mut completed = 0;
                    progress.set(JobPhase::Generation, 0, total, None);
                    let show_preview = publish_preview.is_some();
                    let mut publish_region = |region: &Program| {
                        if let Some(publish) = publish_preview.as_deref_mut() {
                            let mut scene = self.path_scene(region, cancel);
                            if !scene.strips.is_empty() && !cancel.is_cancelled() {
                                if !streamed {
                                    self.add_program_outlines(&mut scene, cancel);
                                    streamed = true;
                                }
                                publish(PreviewUpdate::Append(scene));
                            }
                        }
                    };
                    let publish_region: &mut dyn FnMut(&Program) = &mut publish_region;
                    let job = build_job_cached_with_preview(
                        PlanRequest {
                            document: &document,
                            spindle_rpm: self.spindle_rpm,
                            dir: SpindleDir::Cw,
                            stock: Some(([self.min[0], self.min[1]], [self.max[0], self.max[1]])),
                            tool_change_height: document
                                .setup()
                                .resolved_tool_change_height(self.machine.envelope.max.z),
                        },
                        cancel,
                        cache,
                        |event| match event {
                            cam_toolpath::CalculationProgress::Started(id) => {
                                progress.calculated(id, false);
                                progress.set(JobPhase::Generation, completed, total, Some(id));
                            }
                            cam_toolpath::CalculationProgress::Finished { operation, success } => {
                                completed += 1;
                                progress.calculated(operation, success);
                                progress.set(JobPhase::Generation, completed, total, None);
                            }
                        },
                        show_preview.then_some(publish_region),
                    );
                    progress.set(JobPhase::Generation, completed, total, None);
                    job
                }
                Err(error) => cam_toolpath::JobResult {
                    program: Program::new(),
                    diagnostics: vec![error],
                    completion: JobCompletion::Failed,
                },
            };
            job
        };
        let mut completion = job.completion;
        // Display the assembled path during stock and fixture verification.
        if let Some(publish) = publish_preview.filter(|_| {
            !self.toolpath_preview
                && self.preview_program.is_none()
                && completion == JobCompletion::Complete
                && !cancel.is_cancelled()
                && self.document.setup().operations.iter().any(|op| {
                    op.rotary_wrap().is_some()
                        || (matches!(op, cam_model::Operation::Adaptive(_))
                            && self
                                .document
                                .operation_is_included(self.document.setup().id, op.id()))
                })
        }) {
            let scene = self.program_scene(&job.program, cancel);
            if !cancel.is_cancelled() {
                publish(PreviewUpdate::Replace(RunOutcome {
                    completion: JobCompletion::Incomplete,
                    program: job.program.clone(),
                    scene,
                    ..Default::default()
                }));
            }
        }
        let mut stock = if !self.toolpath_preview
            && completion == JobCompletion::Complete
            && !cancel.is_cancelled()
        {
            match self.simulate_stock(&mut job.program, cancel, cache, progress, simulation) {
                Ok(stock) => stock,
                Err(error) => {
                    completion = JobCompletion::Failed;
                    job.diagnostics.push(error);
                    simulation::StockSimulation::default()
                }
            }
        } else {
            simulation::StockSimulation::default()
        };
        // Verify the actual output after stock-link repairs as well as playback
        // prefixes and pattern previews, even when there is no stock. Browser
        // previews skip fixture verification.
        if completion != JobCompletion::Cancelled && !self.display_only {
            match self.fixture_collisions(&job.program, cancel) {
                Ok(contacts) => stock.collisions.extend(contacts),
                Err(error) => {
                    completion = if cancel.is_cancelled() {
                        JobCompletion::Cancelled
                    } else {
                        JobCompletion::Failed
                    };
                    job.diagnostics.push(error);
                }
            }
        }
        progress.set(JobPhase::RenderPreparation, 0, 1, None);
        // Playback constructs its display scene separately with timeline clipping and
        // visibility options. Avoid rebuilding the entire growing prefix here.
        let scene = self.program_scene(&job.program, cancel);
        if cancel.is_cancelled() {
            completion = JobCompletion::Cancelled;
        }
        if !cancel.is_cancelled() {
            progress.set(JobPhase::RenderPreparation, 1, 1, None);
        }
        RunOutcome {
            completion,
            stamp: None,
            program: job.program,
            diagnostics: job.diagnostics,
            scene,
            initial_stock: stock.initial,
            stock_snapshot: stock.snapshot,
            initial_stock_snapshot: stock.initial_snapshot,
            stock_vertices: stock.vertices,
            stock_machined: stock.machined,
            stock_operations: stock.operations,
            stock_cut_sources: stock.cut_sources,
            removal_samples: Arc::default(),
            stock_indices: stock.indices,
            collisions: stock.collisions,
        }
    }

    pub(crate) fn program_scene(&self, program: &Program, cancel: &CancelToken) -> Scene {
        let mut scene = self.path_scene(program, cancel);
        self.add_program_outlines(&mut scene, cancel);
        scene
    }

    fn path_scene(&self, program: &Program, cancel: &CancelToken) -> Scene {
        let mut scene = if self.preview_program.is_none() && program.validate().is_ok() {
            Scene::from_program_controlled(program, &|| cancel.is_cancelled()).unwrap_or_default()
        } else {
            Scene::new()
        };
        for strip in &mut scene.strips {
            if cancel.is_cancelled() {
                break;
            }
            for point in &mut strip.points {
                *point = self
                    .document
                    .setup()
                    .frame
                    .to_world_vector(point.map(f64::from))
                    .map(|n| n as f32);
            }
        }
        for cone in &mut scene.path_cones {
            if cancel.is_cancelled() {
                break;
            }
            let frame = &self.document.setup().frame;
            cone.at = frame
                .to_world_vector(cone.at.map(f64::from))
                .map(|n| n as f32);
            cone.direction = frame
                .to_world_vector(cone.direction.map(f64::from))
                .map(|n| n as f32);
        }
        scene
    }

    fn add_program_outlines(&self, scene: &mut Scene, cancel: &CancelToken) {
        for region in self
            .regions
            .iter()
            .filter(|_| self.preview_program.is_none())
        {
            if cancel.is_cancelled() {
                break;
            }
            scene.add_region(region, PART);
        }
        for path in self
            .open_paths
            .iter()
            .filter(|_| self.preview_program.is_none())
        {
            if cancel.is_cancelled() {
                break;
            }
            scene.add_open_path(path.points(), PART);
        }
    }
}

struct JobRequest {
    stamp: JobStamp,
    inputs: JobInputs,
    cache: OperationCache,
}
struct Running {
    stamp: JobStamp,
    cancel: CancelToken,
    receiver: mpsc::Receiver<JobEvent>,
}
enum JobEvent {
    Preview(JobStamp, PreviewUpdate),
    Finished(JobOutput),
}
enum PreviewUpdate {
    Append(Scene),
    Replace(RunOutcome),
}
pub(crate) struct JobOutput {
    pub stamp: JobStamp,
    pub outcome: RunOutcome,
    pub cache: OperationCache,
}
#[cfg(test)]
type TestHook = Arc<dyn Fn(&CancelToken) + Send + Sync>;
#[derive(Default)]
pub(crate) struct JobService {
    simulation: Arc<Mutex<simulation::StockSimulationCache>>,
    serial: u64,
    expected: Option<JobStamp>,
    running: Option<Running>,
    queued: Option<JobRequest>,
    progress: Progress,
    preview: Option<RunOutcome>,
    preview_revision: usize,
    #[cfg(test)]
    hook: Option<TestHook>,
}
impl JobService {
    pub fn start(
        &mut self,
        revision: u64,
        fingerprint: String,
        inputs: JobInputs,
        cache: OperationCache,
    ) -> JobStamp {
        self.cancel();
        self.serial = self.serial.checked_add(1).expect("request IDs exhausted");
        let stamp = JobStamp {
            request_id: self.serial,
            revision,
            input_fingerprint: fingerprint,
        };
        self.expected = Some(stamp.clone());
        self.queued = Some(JobRequest {
            stamp: stamp.clone(),
            inputs,
            cache,
        });
        self.progress = Progress::default();
        self.launch_queued();
        stamp
    }
    fn launch_queued(&mut self) {
        if self.running.is_some() {
            return;
        }
        let Some(request) = self.queued.take() else {
            return;
        };
        let stamp = request.stamp.clone();
        let cancel = CancelToken::new();
        let token = cancel.clone();
        let progress = self.progress.clone();
        let (sender, receiver) = mpsc::channel();
        #[cfg(test)]
        let hook = self.hook.clone();
        let failure_sender = sender.clone();
        let setup_id = request.inputs.document.setup().id;
        let simulation = self.simulation.clone();
        let spawned = std::thread::Builder::new()
            .name("machining-job".into())
            .spawn(move || {
                let mut cache = request.cache;
                let mut outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    #[cfg(test)]
                    if let Some(hook) = hook {
                        hook(&token);
                    }
                    let mut publish = |preview| {
                        let _ = sender.send(JobEvent::Preview(request.stamp.clone(), preview));
                    };
                    let publish: &mut dyn FnMut(PreviewUpdate) = &mut publish;
                    request.inputs.execute_with_preview(
                        &token,
                        &mut cache,
                        &progress,
                        &mut simulation.lock().unwrap_or_else(|e| e.into_inner()),
                        cfg!(any(feature = "gui", test)).then_some(publish),
                    )
                }))
                .unwrap_or_else(|_| RunOutcome {
                    completion: JobCompletion::Failed,
                    diagnostics: vec![Diagnostic::error("Machining worker failed; run again")
                        .with_code(DiagnosticCode::WorkerFailed)
                        .for_setup(setup_id)],
                    ..Default::default()
                });
                outcome.stamp = Some(request.stamp.clone());
                let _ = sender.send(JobEvent::Finished(JobOutput {
                    stamp: request.stamp,
                    outcome,
                    cache,
                }));
            });
        if let Err(error) = spawned {
            let _ = failure_sender.send(JobEvent::Finished(JobOutput {
                stamp: stamp.clone(),
                cache: OperationCache::default(),
                outcome: RunOutcome {
                    completion: JobCompletion::Failed,
                    stamp: Some(stamp.clone()),
                    diagnostics: vec![Diagnostic::error(format!(
                        "Could not start machining worker: {error}"
                    ))
                    .with_code(DiagnosticCode::WorkerFailed)
                    .for_setup(setup_id)],
                    ..Default::default()
                },
            }));
        }
        self.running = Some(Running {
            stamp,
            cancel,
            receiver,
        });
    }
    #[cfg(test)]
    pub fn set_test_hook(&mut self, hook: TestHook) {
        self.hook = Some(hook);
    }
    pub fn pending(&self) -> bool {
        self.expected.is_some()
    }
    pub fn progress(&self) -> Option<JobProgress> {
        self.pending().then(|| self.progress.get())
    }
    pub fn cancel(&mut self) {
        if let Some(running) = &self.running {
            running.cancel.cancel();
        }
        self.expected = None;
        self.queued = None;
        self.preview = None;
    }
    #[cfg(any(feature = "gui", test))]
    pub fn preview(&self) -> Option<&RunOutcome> {
        self.preview.as_ref()
    }
    #[cfg(feature = "gui")]
    pub fn preview_revision(&self) -> Option<usize> {
        self.preview.as_ref().map(|_| self.preview_revision)
    }

    pub fn poll(&mut self, revision: u64) -> Option<JobOutput> {
        let running = self.running.as_ref()?;
        let result = loop {
            match running.receiver.try_recv() {
                Ok(JobEvent::Preview(stamp, update)) => {
                    if self.expected.as_ref() != Some(&stamp) || stamp.revision != revision {
                        continue;
                    }
                    match update {
                        PreviewUpdate::Append(mut scene) => {
                            let preview = self.preview.get_or_insert_with(|| RunOutcome {
                                completion: JobCompletion::Incomplete,
                                stamp: Some(stamp),
                                ..Default::default()
                            });
                            preview.scene.strips.append(&mut scene.strips);
                            preview.scene.path_cones.append(&mut scene.path_cones);
                        }
                        PreviewUpdate::Replace(mut preview) => {
                            preview.stamp = Some(stamp);
                            self.preview = Some(preview);
                        }
                    }
                    self.preview_revision = self.preview_revision.wrapping_add(1);
                }
                Ok(JobEvent::Finished(result)) => break Some(result),
                Err(mpsc::TryRecvError::Empty) => return None,
                Err(mpsc::TryRecvError::Disconnected) => {
                    break Some(JobOutput {
                        stamp: running.stamp.clone(),
                        cache: Default::default(),
                        outcome: RunOutcome {
                            completion: JobCompletion::Failed,
                            diagnostics: vec![Diagnostic::error(
                                "Machining worker stopped without a result",
                            )
                            .with_code(DiagnosticCode::WorkerFailed)],
                            ..Default::default()
                        },
                    })
                }
            }
        };
        self.running = None;
        let result = result.filter(|output| {
            self.expected.as_ref() == Some(&output.stamp) && output.stamp.revision == revision
        });
        if result.is_some() {
            self.expected = None;
            self.preview = None;
        } else if self
            .expected
            .as_ref()
            .is_some_and(|stamp| stamp.revision != revision)
        {
            self.cancel();
        }
        self.launch_queued();
        if self.running.is_none() {
            self.expected = None;
        }
        result
    }
    /// Keep polling until a cancelled lane drains, even if no result can publish.
    pub fn needs_poll(&self) -> bool {
        self.running.is_some() || self.queued.is_some()
    }
}
impl Drop for JobService {
    fn drop(&mut self) {
        self.cancel();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn calculation_progress_keeps_successful_operations_during_verification() {
        let progress = Progress::default();
        progress.set(JobPhase::Generation, 0, 2, Some(1));
        progress.calculated(1, true);
        progress.set(JobPhase::Generation, 1, 2, Some(2));
        progress.calculated(2, false);
        progress.set(JobPhase::Verification, 0, 10, None);
        assert_eq!(progress.get().calculated_operations, vec![1]);
        progress.calculated(1, false);
        assert!(progress.get().calculated_operations.is_empty());
    }

    #[test]
    fn preview_is_display_only_and_discarded_on_cancel_or_replacement() {
        let stamp = JobStamp {
            request_id: 1,
            revision: 7,
            input_fingerprint: "a".into(),
        };
        let (sender, receiver) = mpsc::channel();
        let mut jobs = JobService::default();
        jobs.expected = Some(stamp.clone());
        jobs.running = Some(Running {
            stamp: stamp.clone(),
            cancel: CancelToken::new(),
            receiver,
        });
        for x in [1., 2.] {
            let mut scene = Scene::new();
            scene.add_strip(vec![[x, 0., 0.], [x, 1., 0.]], PART);
            sender
                .send(JobEvent::Preview(
                    stamp.clone(),
                    PreviewUpdate::Append(scene),
                ))
                .unwrap();
        }
        assert!(jobs.poll(7).is_none());
        assert_eq!(jobs.preview_revision, 2);
        assert_eq!(jobs.preview().unwrap().scene.strips.len(), 2);
        assert!(jobs.preview().unwrap().program.is_empty());
        assert!(jobs.pending());
        assert_eq!(
            jobs.preview().unwrap().completion,
            JobCompletion::Incomplete
        );
        sender
            .send(JobEvent::Preview(
                stamp.clone(),
                PreviewUpdate::Replace(RunOutcome {
                    completion: JobCompletion::Incomplete,
                    ..Default::default()
                }),
            ))
            .unwrap();
        assert!(jobs.poll(7).is_none());
        assert_eq!(jobs.preview_revision, 3);
        assert!(jobs.preview().unwrap().scene.strips.is_empty());
        jobs.cancel();
        assert!(jobs.preview().is_none());
        sender
            .send(JobEvent::Preview(
                stamp,
                PreviewUpdate::Append(Scene::new()),
            ))
            .unwrap();
        assert!(jobs.poll(7).is_none());
        assert!(jobs.preview().is_none());
    }

    #[test]
    fn completed_result_requires_request_revision_and_fingerprint_match() {
        for mismatch in 0..3 {
            let stamp = JobStamp {
                request_id: 3,
                revision: 7,
                input_fingerprint: "abc".into(),
            };
            let mut wrong = stamp.clone();
            match mismatch {
                0 => wrong.request_id += 1,
                1 => wrong.revision += 1,
                _ => wrong.input_fingerprint = "def".into(),
            }
            let (sender, receiver) = mpsc::channel();
            sender
                .send(JobEvent::Finished(JobOutput {
                    stamp: wrong,
                    outcome: RunOutcome {
                        completion: JobCompletion::Complete,
                        ..Default::default()
                    },
                    cache: Default::default(),
                }))
                .unwrap();
            let mut jobs = JobService::default();
            jobs.serial = 3;
            jobs.expected = Some(stamp.clone());
            jobs.running = Some(Running {
                stamp,
                receiver,
                cancel: CancelToken::new(),
            });
            assert!(jobs.poll(7).is_none());
            assert!(!jobs.pending());
        }
    }
}
