//! Import completion and explicitly opened warning/error dialogs.
use super::*;
use crate::Settings;

#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct Notice {
    pub title: String,
    pub detail: String,
    pub error: bool,
    pub entries: Vec<NoticeEntry>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct NoticeEntry {
    pub severity: Severity,
    pub message: String,
    pub source: Option<String>,
    pub verification_collision: bool,
}

impl Notice {
    pub(super) fn error(title: &str, detail: String) -> Self {
        Self {
            title: title.into(),
            entries: vec![NoticeEntry {
                severity: Severity::Error,
                message: detail.clone(),
                source: None,
                verification_collision: false,
            }],
            detail,
            error: true,
        }
    }
}

#[cfg(test)]
pub(super) fn job_notice(outcome: &crate::RunOutcome) -> Option<Notice> {
    let notice = job_notice_with_info(outcome, &Settings::default())?;
    notice
        .entries
        .iter()
        .any(|entry| entry.severity != Severity::Info)
        .then_some(notice)
}

pub(super) fn job_notice_with_info(
    outcome: &crate::RunOutcome,
    settings: &Settings,
) -> Option<Notice> {
    let mut entries: Vec<_> = outcome
        .diagnostics
        .iter()
        .map(|diagnostic| NoticeEntry {
            severity: diagnostic.severity,
            message: settings.format_diagnostic(diagnostic).into_owned(),
            source: None,
            verification_collision: false,
        })
        .collect();
    if matches!(
        outcome.completion,
        JobCompletion::Failed | JobCompletion::Incomplete
    ) && !outcome.has_errors()
    {
        entries.push(NoticeEntry {
            severity: Severity::Error,
            message: "The toolpath or verification did not complete. Export is unavailable.".into(),
            source: None,
            verification_collision: false,
        });
    }
    entries.extend(outcome.collisions.iter().map(|collision| NoticeEntry {
        severity: Severity::Error,
        message: format!("Verification: {}", collision.message),
        source: None,
        verification_collision: true,
    }));
    let error = entries
        .iter()
        .any(|entry| entry.severity == Severity::Error);
    (!entries.is_empty()).then(|| Notice {
        title: if error {
            "Toolpath / verification errors"
        } else {
            "Toolpath warnings"
        }
        .into(),
        detail: entries
            .iter()
            .map(|entry| entry.message.as_str())
            .collect::<Vec<_>>()
            .join("\n\n"),
        entries,
        error,
    })
}

impl App {
    pub(super) fn queue_notice(&mut self, notice: Notice) -> iced::Task<Message> {
        self.notice_queue.push_back(notice);
        self.show_next_notice()
    }

    pub(super) fn show_next_notice(&mut self) -> iced::Task<Message> {
        if self.notice.is_some() {
            return iced::Task::none();
        }
        let Some(notice) = self.notice_queue.pop_front() else {
            return iced::Task::none();
        };
        self.cancel_tree_drag();
        self.notice_filters = [true; 3];
        self.notice = Some(notice);
        iced::Task::none()
    }

    pub(super) fn notice_overlay(&self) -> Option<Element<'_, Message>> {
        let notice = self.notice.as_ref()?;
        let mut list = iced::widget::column![].spacing(8);
        for entry in &notice.entries {
            let (index, icon, label, color) = match entry.severity {
                Severity::Error => (0, "✕", "Error", Color::from_rgb(0.95, 0.48, 0.40)),
                Severity::Warning => (1, "⚠", "Warning", palette::WARN),
                Severity::Info => (2, "ℹ", "Info", palette::ACCENT_BLUE),
            };
            if !self.notice_filters[index] {
                continue;
            }
            let mut body = iced::widget::column![text(label).size(12).color(color)];
            if let Some(source) = &entry.source {
                body = body.push(text(source).size(12).color(palette::GROUP_LABEL));
            }
            body = body.push(text(&entry.message).size(13));
            list = list.push(
                container(row![text(icon).size(16).color(color), body.spacing(3)].spacing(10))
                    .width(Length::Fill)
                    .padding(10)
                    .style(|theme: &iced::Theme| container::Style {
                        background: Some(Background::Color(
                            theme.extended_palette().background.base.color,
                        )),
                        border: Border {
                            color: palette::SEPARATOR,
                            width: 1.,
                            radius: 4.0.into(),
                        },
                        ..Default::default()
                    }),
            );
        }
        if !notice.entries.iter().any(|entry| {
            self.notice_filters[match entry.severity {
                Severity::Error => 0,
                Severity::Warning => 1,
                Severity::Info => 2,
            }]
        }) {
            list = list.push(text("No messages match the selected filters.").size(13));
        }
        let counts = [Severity::Error, Severity::Warning, Severity::Info].map(|severity| {
            notice
                .entries
                .iter()
                .filter(|entry| entry.severity == severity)
                .count()
        });
        let card = container(
            iced::widget::column![
                text(&notice.title).size(18).color(if notice.error {
                    Color::from_rgb(0.95, 0.48, 0.40)
                } else {
                    palette::WARN
                }),
                row![
                    checkbox(self.notice_filters[0])
                        .label(format!("Errors ({})", counts[0]))
                        .size(14)
                        .text_size(12)
                        .on_toggle(|value| Message::NoticeFilterChanged(0, value)),
                    checkbox(self.notice_filters[1])
                        .label(format!("Warnings ({})", counts[1]))
                        .size(14)
                        .text_size(12)
                        .on_toggle(|value| Message::NoticeFilterChanged(1, value)),
                    checkbox(self.notice_filters[2])
                        .label(format!("Info ({})", counts[2]))
                        .size(14)
                        .text_size(12)
                        .on_toggle(|value| Message::NoticeFilterChanged(2, value)),
                ]
                .spacing(16),
                container(scrollable(list))
                    .max_height((self.window.height * 0.45).clamp(100., 340.)),
                row![
                    Space::new().width(Length::Fill),
                    button("OK").on_press(Message::NoticeDismissed)
                ]
                .spacing(8)
            ]
            .spacing(16),
        )
        .max_width(560)
        .padding(20)
        .style(|theme: &iced::Theme| container::Style {
            background: Some(Background::Color(
                theme.extended_palette().background.weak.color,
            )),
            border: Border {
                color: palette::SEPARATOR,
                width: 1.,
                radius: 6.0.into(),
            },
            ..Default::default()
        });
        Some(
            mouse_area(
                container(card)
                    .padding(24)
                    .width(Length::Fill)
                    .height(Length::Fill)
                    .align_x(Alignment::Center)
                    .align_y(Alignment::Center)
                    .style(|_| container::Style {
                        background: Some(Background::Color(Color::from_rgba(0., 0., 0., 0.55))),
                        ..Default::default()
                    }),
            )
            .on_press(Message::NoticeBackdrop)
            .on_right_press(Message::NoticeBackdrop)
            .interaction(iced::mouse::Interaction::Idle)
            .into(),
        )
    }

    pub(super) fn complete_import(&mut self, result: Result<usize, String>) -> iced::Task<Message> {
        let preview = self.controller.cad_import_preview();
        let replacement = self.controller.cad_import_replacement();
        let reloading = self.controller.cad_import_reloading();
        let previous_revision = replacement
            .and_then(|id| self.controller.geometry_state().asset(id))
            .map(|asset| asset.revision);
        let linked_replacement = replacement.is_some_and(|asset| {
            self.controller
                .geometry_state()
                .bindings
                .values()
                .any(|binding| {
                    binding
                        .selected
                        .iter()
                        .any(|source| source.asset_id == asset)
                })
        });
        let result = result.and_then(|_| {
            if self.setup_draft_dirty()
                || self.inspector_dirty()
                || self.any_field_invalid()
                || self.creating_operation()
                || self.cad_editing_op.is_some()
                || self.drawing_editing_op.is_some()
                || (reloading && (self.move_draft.is_some() || self.pattern_draft.is_some()))
            {
                self.controller.cancel_cad_import();
                return Err("An editor or geometry selection changed during import. Finish or Cancel that task, then import again.".into());
            }
            self.controller.commit_cad_preview()
        });
        match result {
            Err(error) => {
                self.controller.cancel_cad_import();
                let (title, status) = if reloading {
                    ("Cannot reload model", "Reload failed")
                } else {
                    ("Cannot import geometry", "Import failed")
                };
                self.status = format!("{status}: {error}");
                self.queue_notice(Notice::error(title, error))
            }
            Ok(count) => {
                if !reloading {
                    self.cad_hidden.clear();
                    self.hidden_assets.clear();
                    self.model_view = (!self.controller.geometry_state().instances.is_empty())
                        .then_some(TreeNode::Models);
                    self.view = ViewControls::default();
                }
                self.cad_hover = None;
                self.cad_candidates.clear();
                self.cad_pending_kind = None;
                self.cad_editing_op = None;
                self.cad_inspecting = false;
                self.planar_selection.clear();
                self.focus_ops.clear();
                self.operation_draft = None;
                self.operation_geometry = None;
                self.operation_binding_preview = None;
                self.operation_tools.clear();
                self.controller.select(Selection::Setup);
                self.end_setup_draft();
                self.refresh_fields();
                self.status = if reloading {
                    "Model reloaded from file.".into()
                } else {
                    format!("Imported {count} geometry item(s).")
                };
                let mut warnings = preview
                    .map(|preview| preview.diagnostics)
                    .unwrap_or_default();
                let current_revision = replacement
                    .and_then(|id| self.controller.geometry_state().asset(id))
                    .map(|asset| asset.revision);
                if linked_replacement && previous_revision != current_revision {
                    let change = if reloading {
                        "The reloaded model geometry changed."
                    } else {
                        "The model was replaced."
                    };
                    warnings.push(format!("{change} Dependent operations require geometry reselection before generation or export."));
                }
                if warnings.is_empty() {
                    iced::Task::none()
                } else {
                    self.queue_notice(Notice {
                        title: "Geometry import warnings".into(),
                        detail: warnings.join("\n\n"),
                        entries: warnings
                            .into_iter()
                            .map(|message| NoticeEntry {
                                severity: Severity::Warning,
                                message,
                                source: None,
                                verification_collision: false,
                            })
                            .collect(),
                        error: false,
                    })
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    fn app() -> App {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.setup_draft = None;
        app.refresh_fields();
        app
    }

    fn poll_import(app: &mut App, path: std::path::PathBuf) {
        let _ = app.update(Message::CadToImport(Some(path)));
        finish_import(app);
    }

    fn finish_import(app: &mut App) {
        let deadline = Instant::now() + Duration::from_secs(10);
        while app.controller.cad_import_pending() {
            let _ = app.update(Message::PollCadImport);
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    #[test]
    fn dxf_completion_installs_geometry_without_an_add_to_project_step() {
        let mut app = app();
        let before = app.controller.document().clone();
        let path = std::env::temp_dir().join(format!("immediate-dxf-{}.dxf", std::process::id()));
        std::fs::write(&path, SAMPLE_DXF).unwrap();
        poll_import(&mut app, path.clone());
        std::fs::remove_file(path).unwrap();
        assert!(!app.controller.regions().is_empty());
        assert!(app.controller.cad_import_preview().is_none());
        assert_eq!(app.controller.selection(), Selection::Setup);
        assert!(!app.setup_draft_dirty());
        assert!(app.controller.inspection_setup().operations.is_empty());
        assert!(app.status.starts_with("Imported "));
        assert!(app.controller.undo());
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn import_failure_keeps_the_project_and_requests_an_error_dialog() {
        let mut app = app();
        let before = app.controller.document().clone();
        let path = std::env::temp_dir().join(format!("missing-import-{}.step", std::process::id()));
        poll_import(&mut app, path);
        assert_eq!(app.controller.document(), &before);
        assert!(app
            .notice
            .as_ref()
            .is_some_and(|notice| notice.error && notice.title == "Cannot import geometry"));
        assert!(app.notice_queue.is_empty());
    }

    #[test]
    fn import_warning_uses_geometry_immediately_and_requests_a_warning_dialog() {
        let mut app = app();
        let path = std::env::temp_dir().join(format!("warning-import-{}.dxf", std::process::id()));
        let drawing = "0\nSECTION\n2\nENTITIES\n0\nCIRCLE\n10\n0\n20\n0\n40\n5\n0\nTEXT\n10\n0\n20\n0\n40\n2.5\n1\nannotation\n0\nENDSEC\n0\nEOF\n";
        std::fs::write(&path, drawing).unwrap();
        poll_import(&mut app, path.clone());
        std::fs::remove_file(path).unwrap();
        assert!(
            !app.controller.regions().is_empty(),
            "warnings must not require an import confirmation"
        );
        let notice = app.notice.as_ref().expect("warning dialog");
        assert!(!notice.error);
        assert!(
            notice.detail.contains("unsupported entity"),
            "{}",
            notice.detail
        );
        assert!(app.controller.cad_import_preview().is_none());
    }

    #[cfg(feature = "occt")]
    #[test]
    fn step_completion_installs_native_geometry_directly() {
        let mut app = app();
        let before = app.controller.document().clone();
        poll_import(
            &mut app,
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../cam-kernel/fixtures/box-mm.step"),
        );
        assert!(!app.controller.geometry_state().instances.is_empty());
        assert!(app.controller.cad_import_preview().is_none());
        assert!(app.status.starts_with("Imported "));
        assert!(
            app.notice.is_none(),
            "a clean STEP import needs no warning dialog"
        );
        assert!(app.notice_queue.is_empty());
        assert!(!app.setup_draft_dirty());
        assert!(app.controller.undo());
        assert_eq!(app.controller.document(), &before);
    }

    #[cfg(feature = "occt")]
    #[test]
    fn reload_from_saved_source_keeps_operations_and_reads_updated_geometry() {
        let mut app = app();
        let files = tempfile::tempdir().unwrap();
        let source = files.path().join("model.step");
        let project = files.path().join("project.ocam");
        let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("../cam-kernel/fixtures");
        std::fs::copy(fixtures.join("box-mm.step"), &source).unwrap();
        poll_import(&mut app, source.clone());

        let geometry = app.controller.geometry_state();
        let asset = &geometry.assets[0];
        let instance = &geometry.instances[0];
        let face = asset.topology.faces.iter().find(|face| {
            matches!(face.surface, cam_model::CadSurface::Plane { normal, .. } if normal[2] > 0.9)
        }).unwrap();
        let selected = EntityRef {
            asset_id: asset.id,
            revision: asset.revision,
            instance_id: instance.id,
            body_id: instance.body_id,
            kind: EntityKind::Face,
            id: face.id,
        };
        let placement = cam_model::RigidTransform {
            translation: [2., 3., 0.],
            ..instance.placement
        };
        app.controller
            .place_cad_instance(selected.instance_id, placement)
            .unwrap();
        app.controller
            .edit_setup(|setup| setup.model_instances = vec![selected.instance_id]);
        let op = app
            .controller
            .create_cad_operation(OpKind::Face, vec![selected.clone()])
            .unwrap();
        app.controller.save_project(&project).unwrap();
        app.controller.open_project(&project).unwrap();
        app.refresh_fields();
        let before = app.controller.document().clone();

        let reload = |app: &mut App| {
            let _ = app.update(Message::Tree(TreeMessage::Open(TreeNode::Asset(
                selected.asset_id,
            ))));
            let _ = app.update(Message::Tree(TreeMessage::Command(Box::new(
                Message::ReloadCadAsset(selected.asset_id),
            ))));
            assert!(app.controller.cad_import_pending());
            finish_import(app);
        };
        reload(&mut app);
        assert_eq!(app.status, "Model reloaded from file.");
        assert_eq!(app.controller.document(), &before);
        assert!(app.notice.is_none());

        std::fs::copy(fixtures.join("pocket-island-holes-mm.step"), &source).unwrap();
        reload(&mut app);
        assert_eq!(app.status, "Model reloaded from file.");
        let after = app.controller.document();
        assert_eq!(after.setups, before.setups);
        assert_eq!(after.geometry.instances, before.geometry.instances);
        assert_ne!(
            after.geometry.assets[0].topology,
            before.geometry.assets[0].topology
        );
        assert_eq!(after.geometry.assets[0].id, selected.asset_id);
        assert_eq!(after.geometry.assets[0].revision, selected.revision + 1);
        assert_eq!(
            after.geometry.bindings[&op].selected,
            vec![selected.clone()]
        );
        assert!(matches!(
            after.geometry.bindings[&op].status,
            cam_model::ResolutionStatus::NeedsReselection { .. }
        ));
        assert!(app
            .notice
            .as_ref()
            .is_some_and(|notice| notice.detail.contains("require geometry reselection")));
        assert!(app.controller.undo());
        assert_eq!(app.controller.document(), &before);

        let _ = app.update(Message::NoticeDismissed);
        std::fs::remove_file(source).unwrap();
        reload(&mut app);
        assert_eq!(app.controller.document(), &before);
        assert!(app
            .notice
            .as_ref()
            .is_some_and(|notice| notice.error && notice.title == "Cannot reload model"));
    }

    #[test]
    fn run_errors_wait_for_a_status_indicator_click() {
        let mut app = app();
        app.controller
            .add_operation(Operation::Drill(cam_model::DrillOp {
                parameter_expressions: Default::default(),
                id: 0,
                tool: 999,
                points: vec![[0., 0.]],
                depth: 1.,
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
        app.rerun();
        let deadline = Instant::now() + Duration::from_secs(10);
        while app.controller.run_needs_poll() {
            let _ = app.update(Message::PollRun);
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(app.notice.is_none(), "run errors must not interrupt work");
        let target = StatusTarget::Setup(app.controller.active_setup_id());
        assert!(app
            .status_notice(target, &app.controller.current_toolpath_fingerprints())
            .is_some_and(|notice| notice.error));
        let _ = app.update(Message::ShowStatus(target));
        assert!(app.notice.as_ref().is_some_and(|notice| notice.error));
        let _ = app.update(Message::NoticeDismissed);
        for _ in 0..3 {
            let _ = app.update(Message::PollRun);
        }
        assert!(app.notice_queue.is_empty());
        assert!(app.notice.is_none());
    }

    #[test]
    fn warnings_and_verification_collisions_are_serialized_as_dialogs() {
        let mut app = app();
        let mut outcome = crate::RunOutcome {
            completion: JobCompletion::Complete,
            ..Default::default()
        };
        outcome
            .diagnostics
            .push(cam_toolpath::Diagnostic::info("ordinary information"));
        assert!(job_notice(&outcome).is_none());
        outcome
            .diagnostics
            .push(cam_toolpath::Diagnostic::warning("strategy warning"));
        let _ = app.queue_notice(job_notice(&outcome).unwrap());
        assert!(!app.notice.as_ref().unwrap().error);
        outcome.collisions.push(cam_sim::Collision {
            kind: cam_sim::CollisionKind::RapidThroughStock,
            at: [0.; 3],
            message: "rapid through stock".into(),
            op_id: None,
            pattern_id: None,
            instance_index: None,
            step_index: None,
            fraction: 0.,
        });
        let _ = app.queue_notice(job_notice(&outcome).unwrap());
        assert_eq!(app.notice_queue.len(), 1);
        let _ = app.update(Message::NoticeDismissed);
        let notice = app.notice.as_ref().unwrap();
        assert!(notice.error && notice.detail.contains("rapid through stock"));
        assert!(app.notice_queue.is_empty());
    }
    #[test]
    fn filters_change_while_modal_is_open_and_reset_for_next_notice() {
        let mut app = app();
        let notice = Notice::error("Cannot import geometry", "First error".into());
        let _ = app.queue_notice(notice.clone());
        let _ = app.queue_notice(notice);
        let _ = app.update(Message::NoticeFilterChanged(0, false));
        assert_eq!(app.notice_filters, [false, true, true]);
        let _ = app.update(Message::NoticeDismissed);
        assert_eq!(app.notice_filters, [true; 3]);
        assert!(app.notice.is_some());
    }

    #[test]
    fn warning_dialog_blocks_project_actions_until_acknowledged() {
        let mut app = app();
        app.controller.new_operation(OpKind::Drill);
        app.controller.open_dxf(SAMPLE_DXF, "sample").unwrap();
        let before = app.controller.document().clone();
        let _ = app.queue_notice(Notice {
            title: "Warning".into(),
            detail: "Test warning".into(),
            entries: vec![NoticeEntry {
                severity: Severity::Warning,
                message: "Test warning".into(),
                source: None,
                verification_collision: false,
            }],
            error: false,
        });
        let _ = app.update(Message::NewProject);
        let _ = app.update(Message::NoticeBackdrop);
        assert_eq!(app.controller.document(), &before);
        assert!(app.notice.is_some());
        let _ = app.update(Message::NoticeDismissed);
        assert!(app.notice.is_none());
    }
}
