//! Browser and pattern previews share a worker: previewing paths must neither
//! publish a partial export program nor depend on the last simulation's scope.
use super::*;
use crate::jobs::JobService;

#[derive(Clone, Debug, PartialEq)]
enum PreviewScope {
    Through(u32),
    /// Only source identity changes require calculation; placement edits reuse paths.
    PatternSources(Vec<u32>),
}

#[derive(Clone, Debug, PartialEq)]
struct PreviewKey {
    revision: u64,
    setup: u64,
    scope: PreviewScope,
    accuracy: f32,
    source: Option<crate::JobStamp>,
}

// Stock meshes and backplots can be large. Keep a bounded set of recently
// viewed operation prefixes, sharing the active result instead of cloning it.
const MAX_COMPLETED_PREVIEWS: usize = 8;

// Temporary two-instance patterns let the planner calculate both entry variants.
// Their second placement is shifted back before the draft places the cached paths.
pub(super) fn pattern_source_preview_id(op: u32) -> u64 {
    u64::MAX - u64::from(op)
}

#[derive(Default)]
pub(super) struct OperationPreview {
    key: Option<PreviewKey>,
    jobs: JobService,
    cache: Option<cam_toolpath::OperationCache>,
    cache_inputs: Option<PreviewKey>,
    outcome: Option<Arc<crate::RunOutcome>>,
    completed: Vec<(PreviewKey, Arc<crate::RunOutcome>)>,
}

impl App {
    fn operation_preview_key(&self) -> Option<PreviewKey> {
        if self.model_view.is_some()
            || (self.pattern_draft.is_none()
                && (self.operation_preview_active()
                    || self
                        .hidden_setups
                        .contains(&self.controller.active_setup_id())))
        {
            return None;
        }
        let scope = if let Some(draft) = &self.pattern_draft {
            let members: Vec<_> = self
                .controller
                .inspection_setup()
                .operations
                .iter()
                .map(Operation::id)
                .filter(|id| draft.members.contains(id))
                .collect();
            if members.is_empty() {
                return None;
            }
            PreviewScope::PatternSources(members)
        } else {
            let through = self
                .controller
                .inspection_setup()
                .operations
                .iter()
                .rev()
                .map(Operation::id)
                .find(|id| self.focus_ops.contains(id) && !self.hidden_ops.contains(id))?;
            PreviewScope::Through(through)
        };
        Some(PreviewKey {
            revision: self.controller.revision(),
            setup: self.controller.active_setup_id(),
            scope,
            accuracy: self.settings.simulation.accuracy,
            source: self.controller.outcome().and_then(|o| o.stamp.clone()),
        })
    }

    pub(super) fn sync_operation_preview(&mut self) {
        self.operation_preview.completed.retain(|(key, _)| {
            key.revision == self.controller.revision()
                && key.accuracy == self.settings.simulation.accuracy
                && key.source == self.controller.outcome().and_then(|o| o.stamp.clone())
        });
        // Verification/simulation has priority over background browser work.
        if self.simulation.is_some() || self.controller.run_pending() {
            self.operation_preview.jobs.cancel();
            self.operation_preview.key = None;
            self.operation_preview.outcome = None;
            return;
        }
        let key = self.operation_preview_key();
        if key == self.operation_preview.key {
            return;
        }
        self.operation_preview.jobs.cancel();
        self.operation_preview.outcome = None;
        let Some(key) = key else {
            self.operation_preview.key = None;
            return;
        };
        if let Some(index) = self
            .operation_preview
            .completed
            .iter()
            .position(|(cached, _)| *cached == key)
        {
            let cached = self.operation_preview.completed.remove(index);
            self.operation_preview.outcome = Some(cached.1.clone());
            self.operation_preview.completed.push(cached);
            self.operation_preview.key = Some(key);
            return;
        }
        let reuse = self
            .operation_preview
            .cache_inputs
            .as_ref()
            .is_some_and(|old| {
                old.revision == key.revision && old.setup == key.setup && old.source == key.source
            });
        if !reuse {
            self.operation_preview.cache = Some(self.controller.operation_preview_cache());
            self.operation_preview.cache_inputs = Some(key.clone());
        }
        let mut inputs = self.controller.job_inputs();
        inputs.simulation_accuracy = key.accuracy;
        let document = Arc::make_mut(&mut inputs.document);
        match &key.scope {
            PreviewScope::PatternSources(members) => {
                let others: Vec<_> = document
                    .setup()
                    .operations
                    .iter()
                    .map(Operation::id)
                    .filter(|id| !members.contains(id))
                    .collect();
                // Calculate entry variants independently of draft placement. Only
                // sources with a selected entry need a second cached path.
                document.setup_mut().patterns.clear();
                let patterns = document
                    .setup()
                    .operations
                    .iter()
                    .filter(|op| {
                        members.contains(&op.id())
                            && !matches!(op, Operation::Adaptive(_))
                            && operation_tabs::operation_start(op).is_some()
                    })
                    .map(|op| cam_model::OperationPattern {
                        id: pattern_source_preview_id(op.id()),
                        name: "Entry preview".into(),
                        members: vec![op.id()],
                        kind: cam_model::PatternKind::Rectangular {
                            nx: 2,
                            ny: 1,
                            dx: 1.0,
                            dy: 0.0,
                        },
                        order: cam_model::PatternOrder::ByInstance,
                        suppressed: false,
                        options: Default::default(),
                    })
                    .collect();
                document.setup_mut().patterns = patterns;
                document.excluded_operations.extend(others);
                inputs.toolpath_preview = true;
            }
            PreviewScope::Through(through) => {
                // Browser previews need the stock left by this operation and its
                // predecessors. Keep fixture verification and export publication
                // out of this background job, but calculate the stock mesh.
                inputs.display_only = true;
                let later: Vec<_> = document
                    .setup()
                    .operations
                    .iter()
                    .skip_while(|op| op.id() != *through)
                    .skip(1)
                    .map(Operation::id)
                    .collect();
                document.excluded_operations.extend(later);
            }
        }
        self.operation_preview.jobs.start(
            key.revision,
            format!("operation-preview:{}:{:?}", key.setup, key.scope),
            inputs,
            self.operation_preview.cache.clone().unwrap_or_default(),
        );
        self.operation_preview.key = Some(key);
    }

    pub(super) fn poll_operation_preview(&mut self) {
        if let Some(result) = self.operation_preview.jobs.poll(self.controller.revision()) {
            self.operation_preview.cache = Some(result.cache);
            let outcome = Arc::new(result.outcome);
            if outcome.completion == JobCompletion::Complete {
                if let Some(key) = &self.operation_preview.key {
                    if self.operation_preview.completed.len() >= MAX_COMPLETED_PREVIEWS {
                        self.operation_preview.completed.remove(0);
                    }
                    self.operation_preview
                        .completed
                        .push((key.clone(), outcome.clone()));
                }
            }
            self.operation_preview.outcome = Some(outcome);
        }
    }

    pub(super) fn cancel_operation_preview(&mut self) {
        self.operation_preview.jobs.cancel();
        self.operation_preview.outcome = None;
        // Keep the current key so the next UI event does not restart a preview
        // the user explicitly cancelled. A new selection/edit can request one.
        self.operation_preview.key = self.operation_preview_key();
    }

    pub(super) fn operation_preview_outcome(&self) -> Option<&crate::RunOutcome> {
        (self.operation_preview_key() == self.operation_preview.key)
            .then_some(self.operation_preview.outcome.as_deref())
            .flatten()
    }

    #[cfg(test)]
    pub(super) fn drain_operation_preview(&mut self) {
        self.sync_operation_preview();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
        while self.operation_preview.needs_poll() {
            let _ = self.update(Message::PollRun);
            assert!(
                std::time::Instant::now() < deadline,
                "operation preview timed out"
            );
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
}

impl OperationPreview {
    pub(super) fn progress(&self) -> Option<crate::JobProgress> {
        self.jobs.progress()
    }

    pub(super) fn running_preview(&self) -> Option<&crate::RunOutcome> {
        self.jobs.preview()
    }

    pub(super) fn preview_revision(&self) -> Option<usize> {
        self.jobs.preview_revision()
    }

    pub(super) fn needs_poll(&self) -> bool {
        self.jobs.needs_poll()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn preview_app() -> (App, [u32; 2]) {
        let (mut app, _) = App::new();
        app.settings = crate::Settings::default();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample.dxf").unwrap();
        app.setup_draft = None;
        app.model_view = None;
        app.inspector_active = false;
        let ids = [10., 25.].map(|x| {
            app.controller
                .add_operation(Operation::Drill(cam_model::DrillOp {
                    parameter_expressions: Default::default(),
                    id: 0,
                    tool: 1,
                    points: vec![[x, 10.]],
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
            let Selection::Operation(id) = app.controller.selection() else {
                panic!()
            };
            id
        });
        (app, ids)
    }

    fn focus(app: &mut App, id: u32) {
        app.controller.select(Selection::Operation(id));
        app.focus_ops = BTreeSet::from([id]);
        app.sync_operation_preview();
    }

    #[test]
    fn browsing_shows_removed_stock_without_cancelling_the_active_run() {
        let (mut app, ids) = preview_app();
        focus(&mut app, ids[0]);
        app.drain_operation_preview();
        let preview = app.operation_preview_outcome().unwrap();
        assert!(!preview.stock_indices.is_empty());
        assert!(preview.stock_operations.contains(&Some(ids[0])));
        assert!(!preview.stock_operations.contains(&Some(ids[1])));
        assert!(preview.scene.strips.iter().any(|s| s.op == Some(ids[0])));

        let (entered, receiver) = std::sync::mpsc::channel();
        let (release, wait) = std::sync::mpsc::channel();
        let wait = std::sync::Mutex::new(wait);
        app.controller.set_run_test_hook(Arc::new(move |token| {
            entered.send(token.clone()).unwrap();
            wait.lock().unwrap().recv().unwrap();
        }));
        app.controller.start_run();
        let token = receiver
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap();
        focus(&mut app, ids[1]);
        focus(&mut app, ids[0]);
        assert!(
            !token.is_cancelled(),
            "selection must not cancel verification"
        );
        app.controller.edit_operation(ids[0], |op| {
            let Operation::Drill(op) = op else { panic!() };
            op.depth = 1.;
        });
        assert!(
            token.is_cancelled(),
            "toolpath edits cancel obsolete verification"
        );
        release.send(()).unwrap();
    }

    #[test]
    fn completed_operation_previews_survive_switching_and_unmodified_editor() {
        let (mut app, ids) = preview_app();
        let mut outcomes = Vec::new();
        for id in ids {
            focus(&mut app, id);
            app.drain_operation_preview();
            let outcome = app.operation_preview.outcome.clone().unwrap();
            assert_eq!(
                outcome.completion,
                JobCompletion::Complete,
                "{:?}",
                outcome.diagnostics
            );
            outcomes.push(outcome);
        }
        // Restoring a preview must publish exactly the original result without
        // submitting generation, verification or render preparation again.
        for (id, expected) in ids.into_iter().zip(&outcomes) {
            focus(&mut app, id);
            assert!(app.operation_preview.progress().is_none());
            assert!(Arc::ptr_eq(
                app.operation_preview.outcome.as_ref().unwrap(),
                expected
            ));
        }
        app.begin_operation_draft(ids[1]);
        app.sync_operation_preview();
        assert!(app.operation_preview_outcome().is_none());
        let _ = app.update(Message::CancelOperationDraft);
        focus(&mut app, ids[1]);
        assert!(app.operation_preview.progress().is_none());
        assert!(Arc::ptr_eq(
            app.operation_preview.outcome.as_ref().unwrap(),
            &outcomes[1]
        ));

        app.focus_ops.clear();
        app.sync_operation_preview();
        focus(&mut app, ids[0]);
        assert!(app.operation_preview.progress().is_none());
        assert!(Arc::ptr_eq(
            app.operation_preview.outcome.as_ref().unwrap(),
            &outcomes[0]
        ));
        assert!(
            app.controller.outcome().is_none(),
            "browser work must not publish an export program"
        );
    }

    #[test]
    fn operation_preview_cache_invalidates_on_edits_accuracy_and_new_source() {
        let (mut app, ids) = preview_app();
        focus(&mut app, ids[0]);
        app.drain_operation_preview();
        let initial = app.operation_preview.outcome.clone().unwrap();
        app.settings.simulation.accuracy = 0.5;
        focus(&mut app, ids[0]);
        assert!(app.operation_preview_outcome().is_none());
        assert!(app.operation_preview.progress().is_some());
        app.drain_operation_preview();
        assert!(!Arc::ptr_eq(
            app.operation_preview.outcome.as_ref().unwrap(),
            &initial
        ));
        let accurate = app.operation_preview.outcome.clone().unwrap();
        app.controller.edit_operation(ids[0], |op| {
            let Operation::Drill(op) = op else { panic!() };
            op.depth = 1.;
        });
        focus(&mut app, ids[0]);
        assert!(app.operation_preview.completed.is_empty());
        assert!(app.operation_preview_outcome().is_none());
        app.drain_operation_preview();
        assert!(!Arc::ptr_eq(
            app.operation_preview.outcome.as_ref().unwrap(),
            &accurate
        ));

        app.controller.start_run();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while app.controller.run_needs_poll() {
            app.controller.poll_run();
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        focus(&mut app, ids[0]);
        assert!(app.operation_preview.completed.is_empty());
        assert!(app.operation_preview_outcome().is_none());
        app.drain_operation_preview();
    }

    fn has_hole(scene: &Scene, operation: u32, instance: u32, at: [f32; 2]) -> bool {
        scene.strips.iter().any(|strip| {
            strip.op == Some(operation)
                && strip.instance_index == Some(instance)
                && strip
                    .points
                    .iter()
                    .all(|point| (point[0] - at[0]).abs() < 1e-4 && (point[1] - at[1]).abs() < 1e-4)
                && strip.points.first().unwrap()[2] != strip.points.last().unwrap()[2]
        })
    }

    #[test]
    fn pattern_preview_calculates_sources_once_and_updates_live_without_committing() {
        let (mut app, ids) = preview_app();
        app.focus_ops = BTreeSet::from([ids[0]]);
        let document = app.controller.document().clone();
        let revision = app.controller.revision();
        let _ = app.update(Message::BeginPattern(false));
        let pending = app.machining_viewport();
        app.drain_operation_preview();
        let sources = app.operation_preview.outcome.clone().unwrap();
        assert_eq!(
            sources.completion,
            JobCompletion::Complete,
            "{:?}",
            sources.diagnostics
        );
        assert!(
            sources.stock_vertices.is_empty(),
            "placement previews do not simulate stock"
        );
        assert!(sources
            .scene
            .strips
            .iter()
            .filter_map(|strip| strip.op)
            .all(|id| id == ids[0]));
        let initial = app.pattern_placement_preview();
        assert!(has_hole(&initial, ids[0], 0, [10., 10.]));
        assert!(has_hole(&initial, ids[0], 1, [30., 10.]));
        let initial_view = app.machining_viewport();
        assert!(
            !Arc::ptr_eq(&pending.vertices, &initial_view.vertices),
            "a finished seed refreshes the cached viewport"
        );
        for message in [
            Message::PatternValue(0, "3".into()),
            Message::PatternValue(2, "40".into()),
            Message::PatternSecondDirection(true),
            Message::PatternValue(1, "2".into()),
            Message::PatternValue(3, "15".into()),
        ] {
            let _ = app.update(message);
        }
        assert!(has_hole(
            &app.pattern_placement_preview(),
            ids[0],
            5,
            [90., 25.]
        ));
        let _ = app.update(Message::PatternAxisReversed(0, true));
        assert!(has_hole(
            &app.pattern_placement_preview(),
            ids[0],
            5,
            [-70., 25.]
        ));
        let _ = app.update(Message::PatternKeepOriginal(false));
        let _ = app.update(Message::PatternReverse(true));
        let changed = app.pattern_placement_preview();
        assert!(changed
            .strips
            .iter()
            .all(|strip| strip.instance_index != Some(0)));
        assert_eq!(changed.strips[0].instance_index, Some(5));
        let changed_view = app.machining_viewport();
        assert!(
            !Arc::ptr_eq(&initial_view.vertices, &changed_view.vertices),
            "parameter edits redraw the viewport before Apply"
        );
        assert!(
            changed_view.frame_bounds.unwrap().0[0] <= -70.0,
            "camera framing includes repeats beyond the original stock"
        );
        let _ = app.update(Message::PatternTypeChanged(true));
        let _ = app.update(Message::PatternValue(1, "5".into()));
        let _ = app.update(Message::PatternValue(2, "5".into()));
        let _ = app.update(Message::PatternValue(3, "180".into()));
        assert!(has_hole(
            &app.pattern_placement_preview(),
            ids[0],
            3,
            [0., 0.]
        ));
        assert!(Arc::ptr_eq(
            app.operation_preview.outcome.as_ref().unwrap(),
            &sources
        ));
        assert!(
            !app.operation_preview.needs_poll(),
            "placement edits reuse the source result synchronously"
        );
        assert_eq!(app.controller.document(), &document);
        assert_eq!(app.controller.revision(), revision);
        assert!(
            app.controller.outcome().is_none(),
            "draft preview cannot publish an export program"
        );
        let _ = app.update(Message::PatternCancel);
        assert!(app.pattern_placement_preview().strips.is_empty());
    }

    #[test]
    fn pattern_preview_uses_the_selected_entry_only_at_the_first_placement() {
        let (mut app, _) = preview_app();
        app.controller.new_operation(OpKind::Profile);
        let Selection::Operation(id) = app.controller.selection() else {
            panic!()
        };
        app.controller.edit_operation(id, |op| {
            let Operation::Profile(op) = op else { panic!() };
            op.chain = cam_geo::Contour::new(vec![
                cam_geo::Point::new(0.0, 0.0),
                cam_geo::Point::new(30.0, 0.0),
                cam_geo::Point::new(30.0, 30.0),
                cam_geo::Point::new(0.0, 30.0),
            ]);
            op.start = Some([30.0, 30.0]);
            op.side = cam_model::Side::On;
            op.depth = 2.0;
            op.stepdown = 2.0;
            op.plunge = cam_model::Plunge::Straight;
            op.lead_in = cam_model::Lead::None;
        });
        app.focus_ops = BTreeSet::from([id]);
        let document = app.controller.document().clone();
        let _ = app.update(Message::BeginPattern(false));
        app.drain_operation_preview();
        let sources = app.operation_preview.outcome.clone().unwrap();
        assert_eq!(
            sources.completion,
            JobCompletion::Complete,
            "{:?}",
            sources.diagnostics
        );
        let entries: Vec<_> = (0..2)
            .map(|variant| {
                sources
                    .program
                    .steps()
                    .iter()
                    .find_map(|step| match step {
                        cam_cldata::Step::Linear { to, tag, .. }
                            if tag.op_id == id
                                && tag.kind == cam_cldata::MoveKind::Plunge
                                && tag.instance_index == Some(variant) =>
                        {
                            Some([to.x - f64::from(variant), to.y, to.z])
                        }
                        _ => None,
                    })
                    .expect("both entry variants were calculated")
            })
            .collect();
        assert_ne!(entries[0], entries[1]);
        let _ = app.update(Message::PatternValue(0, "3".into()));
        for reversed in [false, true] {
            let _ = app.update(Message::PatternKeepOriginal(!reversed));
            let _ = app.update(Message::PatternReverse(reversed));
            let indices = if reversed { vec![2, 1] } else { vec![0, 1, 2] };
            let scene = app.pattern_placement_preview();
            for (ordinal, index) in indices.into_iter().enumerate() {
                let entry = entries[usize::from(ordinal > 0)];
                let expected = [
                    (entry[0] + f64::from(index) * 20.0) as f32,
                    entry[1] as f32,
                    entry[2] as f32,
                ];
                assert!(
                    scene
                        .strips
                        .iter()
                        .filter(|strip| strip.instance_index == Some(index))
                        .any(|strip| strip.points.windows(2).any(|pair| {
                            pair[1] == expected
                                && pair[0][0] == expected[0]
                                && pair[0][1] == expected[1]
                                && pair[0][2] > expected[2]
                        })),
                    "instance {index} must plunge at {expected:?}"
                );
            }
            assert!(Arc::ptr_eq(
                app.operation_preview.outcome.as_ref().unwrap(),
                &sources
            ));
        }
        assert_eq!(app.controller.document(), &document);
    }

    #[test]
    fn pattern_preview_refreshes_sources_and_removes_committed_instance_transforms() {
        let (mut app, ids) = preview_app();
        let setup = app.controller.active_setup_id();
        let id = app
            .controller
            .create_pattern_with_options(
                setup,
                &[ids[0]],
                "Repeats".into(),
                cam_model::PatternKind::Rectangular {
                    nx: 3,
                    ny: 1,
                    dx: 20.,
                    dy: 0.,
                },
                cam_model::PatternOrder::ByInstance,
                cam_model::PatternOptions {
                    keep_original: false,
                    reverse: true,
                    ..Default::default()
                },
            )
            .unwrap();
        let _ = app.update(Message::EditPattern(id));
        app.drain_operation_preview();
        let sources = app.operation_preview.outcome.clone().unwrap();
        assert_eq!(
            sources.completion,
            JobCompletion::Complete,
            "{:?}",
            sources.diagnostics
        );
        assert!(sources
            .scene
            .strips
            .iter()
            .filter(|strip| strip.op.is_some())
            .all(|strip| strip.pattern_id.is_none()));
        assert!(has_hole(
            &app.pattern_placement_preview(),
            ids[0],
            2,
            [50., 10.]
        ));
        let _ = app.update(Message::PatternMember(ids[1], true));
        app.drain_operation_preview();
        assert!(!Arc::ptr_eq(
            app.operation_preview.outcome.as_ref().unwrap(),
            &sources
        ));
        assert!(has_hole(
            &app.pattern_placement_preview(),
            ids[1],
            2,
            [65., 10.]
        ));
        assert_eq!(
            app.controller.inspection_setup().patterns[0].members,
            vec![ids[0]],
            "source edits stay in the draft"
        );
    }
}
