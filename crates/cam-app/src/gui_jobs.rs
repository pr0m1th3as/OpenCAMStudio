//! Background job presentation and output view.
use super::*;
use iced::widget::column;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum ToolpathAction {
    Recompute,
    Simulate,
    Export,
    ProcessingTime,
}

impl App {
    /// Keep calculation feedback visible independently of the docked Output pane.
    pub(super) fn calculation_progress(&self) -> Option<Element<'_, Message>> {
        use crate::JobPhase;
        use iced::widget::progress_bar;

        let cancel = if self.controller.run_progress().is_some() {
            Message::CancelRun
        } else {
            Message::CancelOperationPreview
        };
        let progress = self
            .controller
            .run_progress()
            .or_else(|| self.operation_preview.progress())?;
        let has_preview = self.controller.run_preview().is_some()
            || self.operation_preview.running_preview().is_some();
        let label = match progress.phase {
            JobPhase::Queued => "Waiting to calculate…".into(),
            JobPhase::GeometryResolution => "Resolving machining geometry…".into(),
            JobPhase::Generation => {
                let operation = progress
                    .operation
                    .map_or(String::new(), |id| format!(" · operation #{id}"));
                let preview = if has_preview {
                    " · partial preview"
                } else {
                    ""
                };
                format!("Calculating toolpaths{operation}{preview}…")
            }
            JobPhase::Verification if progress.total > 0 => format!(
                "Verifying toolpaths{} · {:.0}%",
                if has_preview {
                    " · preview is unverified"
                } else {
                    ""
                },
                (progress.completed as f64 / progress.total as f64).clamp(0.0, 1.0) * 100.0
            ),
            JobPhase::Verification if has_preview => {
                "Verifying toolpaths · preview is unverified…".into()
            }
            JobPhase::Verification => "Verifying toolpaths…".into(),
            JobPhase::RenderPreparation => "Preparing preview…".into(),
        };
        let bar: Element<'_, Message> =
            if progress.phase == JobPhase::Verification && progress.total > 0 {
                progress_bar(
                    0.0..=1.0,
                    (progress.completed as f32 / progress.total as f32).clamp(0.0, 1.0),
                )
                .girth(6)
                .into()
            } else {
                // Generation reports operation boundaries, not progress within a long
                // operation. Animate a moving segment instead of inventing a percentage.
                // The existing 50 ms worker polling subscription also redraws this bar.
                let tick = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_millis()
                    % 2_000;
                let position = if tick < 1_000 { tick } else { 2_000 - tick };
                let left = 1 + (position / 10) as u16;
                row![
                    Space::new().width(Length::FillPortion(left)),
                    progress_bar(0.0..=1.0, 1.0)
                        .length(Length::FillPortion(30))
                        .girth(6),
                    Space::new().width(Length::FillPortion(102 - left)),
                ]
                .into()
            };
        Some(
            container(
                column![
                    row![
                        text(label).size(12).width(Length::Fill),
                        inspector_button("Cancel").on_press(cancel),
                    ]
                    .align_y(Alignment::Center)
                    .spacing(12),
                    bar,
                ]
                .spacing(6),
            )
            .padding([8, 12])
            .width(Length::Fill)
            .into(),
        )
    }

    /// Submit a snapshot; the event loop never runs machining services inline.
    pub(super) fn rerun(&mut self) {
        self.rerun_with_review(false);
    }

    pub(super) fn rerun_all(&mut self) {
        self.rerun_with_review(true);
    }

    fn rerun_with_review(&mut self, review_all: bool) {
        self.controller
            .set_simulation_accuracy(self.settings.simulation.accuracy);
        self.pending_toolpath_export = None;
        self.pending_simulation = None;
        self.pending_processing_time = None;
        self.processing_time_dialog = None;
        self.simulation = None;
        if self.controller.inspection_setup().operations.is_empty()
            && self.controller.inspection_setup().stock != cam_model::Stock::PreviousSetup
        {
            self.controller.cancel_run();
            self.status = "No operations to run.".into();
            return;
        }
        if review_all {
            self.controller.start_run();
        } else {
            self.controller.start_automatic_run();
        }
        self.status = "Generating toolpaths…".into();
    }

    pub(super) fn toolpath_command(
        &mut self,
        node: TreeNode,
        action: ToolpathAction,
    ) -> iced::Task<Message> {
        if self.setup_draft_dirty()
            || self.pattern_draft.is_some()
            || self.inspector_dirty()
            || self.any_field_invalid()
            || !self.operation_height_overrides_valid()
            || self.controller.pending_op().is_some()
            || self.cad_pending_kind.is_some()
            || self.cad_editing_op.is_some()
            || self.drawing_editing_op.is_some()
        {
            self.status = "Apply or cancel inspector changes and geometry selection before processing toolpaths.".into();
            return iced::Task::none();
        }
        let (setup, operation, pattern) = match node {
            TreeNode::Setup(id) => (id, None, None),
            TreeNode::Pattern(id) => {
                let Some(setup) = self
                    .controller
                    .setups()
                    .iter()
                    .find(|s| s.patterns.iter().any(|p| p.id == id))
                else {
                    self.status = "Pattern no longer exists.".into();
                    return iced::Task::none();
                };
                (setup.id, None, Some(id))
            }
            TreeNode::Operation(id) => {
                let Some(setup) = self
                    .controller
                    .setups()
                    .iter()
                    .find(|setup| setup.operations.iter().any(|op| op.id() == id))
                else {
                    self.status = "Operation no longer exists.".into();
                    return iced::Task::none();
                };
                (setup.id, Some(id), None)
            }
            _ => return iced::Task::none(),
        };
        if let Err(error) = self.controller.activate_setup(setup) {
            self.status = error;
            return iced::Task::none();
        }
        self.pending_toolpath_export = None;
        self.pending_simulation = None;
        self.pending_processing_time = None;
        self.processing_time_dialog = None;
        self.simulation = None;
        self.tree_menu = None;
        self.open_op_menu = None;
        self.expanded_setups.insert(setup);
        self.model_view = None;
        self.focus_ops.clear();
        self.focus_ops.extend(operation);
        if let Some(pattern_id) = pattern {
            if let Some(pattern) = self
                .controller
                .setups()
                .iter()
                .find_map(|s| s.patterns.iter().find(|p| p.id == pattern_id))
            {
                self.focus_ops.extend(pattern.members.iter().copied());
            }
        }
        self.end_setup_draft();
        self.operation_draft = None;
        self.controller
            .select(operation.map_or(Selection::Setup, Selection::Operation));
        if let Some(id) = operation {
            self.seed_rotary_index_draft(id);
        } else {
            self.rotary_index_draft = None;
        }
        self.refresh_fields();
        self.switch_workspace(Workspace::Machining);
        self.controller
            .set_simulation_accuracy(self.settings.simulation.accuracy);
        let run = if let Some(pattern_id) = pattern {
            self.controller
                .start_pattern_toolpath_run(pattern_id, action == ToolpathAction::Recompute)
        } else {
            self.controller.start_toolpath_run_with_predecessors(
                operation,
                action == ToolpathAction::Recompute,
                action == ToolpathAction::Simulate && operation.is_some(),
            )
        };
        match run {
            Ok(stamp) => {
                if action == ToolpathAction::Export {
                    self.pending_toolpath_export = Some(stamp.clone());
                }
                if action == ToolpathAction::Simulate {
                    self.pending_simulation = Some(stamp.clone());
                    self.show_stock = true;
                }
                if action == ToolpathAction::ProcessingTime {
                    self.pending_processing_time =
                        Some(super::processing_time_view::PendingProcessingTime {
                            stamp: stamp.clone(),
                            label: match node {
                                TreeNode::Setup(id) => self
                                    .controller
                                    .setups()
                                    .iter()
                                    .find(|setup| setup.id == id)
                                    .map(|setup| format!("{} · setup #{id}", setup.name))
                                    .unwrap_or_else(|| format!("Setup #{id}")),
                                TreeNode::Operation(id) => format!("Operation #{id}"),
                                TreeNode::Pattern(id) => format!("Pattern #{id}"),
                                _ => "Program".to_string(),
                            },
                            feed_override: "100".to_string(),
                        });
                }
                self.status = match action {
                    ToolpathAction::Recompute => "Recomputing toolpaths…",
                    ToolpathAction::Simulate => "Simulating toolpaths…",
                    ToolpathAction::Export => "Preparing toolpaths for post processing…",
                    ToolpathAction::ProcessingTime => "Calculating processing time…",
                }
                .into();
            }
            Err(error) => self.status = error,
        }
        iced::Task::none()
    }
    pub(super) fn report_run(&mut self) {
        let Some(outcome) = self.controller.outcome() else {
            return;
        };
        let errors = outcome
            .diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Error)
            .count();
        let has_collisions = !outcome.collisions.is_empty();
        let collision_severity = self.controller.verification_collision_severity();
        self.status = match outcome.completion {
            JobCompletion::Complete if errors > 0 || (has_collisions && collision_severity == Severity::Error) =>
                "Toolpath verification errors — click a setup or operation status indicator for details.".into(),
            JobCompletion::Complete if has_collisions || outcome.diagnostics.iter().any(|d| d.severity == Severity::Warning) =>
                "Toolpath ready with warnings — click a setup or operation status indicator for details.".into(),
            JobCompletion::Complete => "Toolpath ready. Export when you like.".to_string(),
            JobCompletion::Failed => format!("{errors} error(s) — click a setup or operation status indicator for details."),
            JobCompletion::Cancelled => {
                "Run cancelled. Run the job again before exporting.".to_string()
            }
            JobCompletion::Incomplete => {
                "Run incomplete. Run the job again before exporting.".to_string()
            }
        };
    }
    /// A read-only editor provides native drag selection, Select All and Copy.
    pub(super) fn output(&self) -> Element<'_, Message> {
        let mut controls = row![
            inspector_button(if self.output_content.selection().is_some() {
                "Copy selection"
            } else {
                "Copy output"
            })
            .on_press(Message::OutputCopy),
            inspector_button("Clear").on_press(Message::OutputClear)
        ]
        .spacing(8);
        if self.output_content.selection().is_some() {
            controls = controls.push(text("Text updates pause while selected").size(11));
        }
        if self.controller.cad_import_pending() {
            controls =
                controls.push(inspector_button("Cancel import").on_press(Message::CancelCadImport));
        }
        if self.controller.run_progress().is_some() {
            controls = controls.push(inspector_button("Cancel run").on_press(Message::CancelRun));
        }
        column![
            controls,
            iced::widget::text_editor(&self.output_content)
                .on_action(Message::OutputAction)
                .font(iced::Font::MONOSPACE)
                .size(12)
                .height(Length::Fill)
        ]
        .spacing(4)
        .padding(8)
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simulation_preserves_output_pane_visibility() {
        for visible in [false, true] {
            let (mut app, _) = App::new();
            app.controller
                .open_dxf(SAMPLE_DXF, "simulation.dxf")
                .unwrap();
            app.end_setup_draft();
            app.controller
                .add_operation(Operation::Drill(cam_model::DrillOp {
                    cycle: Default::default(),
                    tip_through_bottom: false,
                    geometry: Default::default(),
                    hole_heights: Vec::new(),
                    parameter_expressions: Default::default(),
                    id: 0,
                    tool: 1,
                    points: vec![[10., 10.]],
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
            app.controller.select(Selection::Setup);
            app.refresh_fields();
            app.set_pane_visible(Pane::Output, visible);
            let setup = app.controller.inspection_setup().id;

            let _ = app.toolpath_command(TreeNode::Setup(setup), ToolpathAction::Simulate);

            assert!(app.pending_simulation.is_some(), "{}", app.status);
            assert_eq!(app.pane_handle(Pane::Output).is_some(), visible);
        }
    }

    #[test]
    fn opening_saved_paths_reuses_them_and_legacy_projects_wait_for_a_user_run() {
        let mut source = AppController::new(crate::default_machine());
        source.open_dxf(SAMPLE_DXF, "saved-paths.dxf").unwrap();
        assert_eq!(
            source.run(&cam_toolpath::CancelToken::new()).completion,
            JobCompletion::Complete
        );
        let path =
            std::env::temp_dir().join(format!("gui-saved-paths-{}.ocam", std::process::id()));
        source.save_project(&path).unwrap();
        let (mut app, _) = App::new();
        app.library.tools.clear();
        app.open_project(&path).unwrap();
        assert!(
            !app.controller.run_needs_poll(),
            "saved verification must not start a worker"
        );
        assert_eq!(
            app.controller.outcome().unwrap().program,
            source.outcome().unwrap().program
        );
        assert_eq!(
            app.controller.outcome().unwrap().collisions,
            source.outcome().unwrap().collisions
        );

        let mut legacy: serde_json::Value =
            serde_json::from_str(&crate::OcamFile::read(&path).unwrap().to_json().unwrap())
                .unwrap();
        legacy.as_object_mut().unwrap().remove("toolpaths");
        std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
        app.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert!(
            !app.controller.run_needs_poll(),
            "opening cannot calculate missing paths"
        );
        assert!(app
            .controller
            .toolpath_statuses()
            .values()
            .all(|ready| !ready));
    }
}
