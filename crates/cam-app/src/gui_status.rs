//! Non-interrupting machining status, with diagnostics opened on demand.
use super::*;
use iced::widget::column;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum StatusTarget {
    Setup(u64),
    Operation(u32),
}

struct RecordedStatus {
    dependencies: BTreeMap<u32, String>,
    outcome: crate::RunOutcome,
    machine: cam_model::Machine,
    accuracy: f32,
}

#[derive(Default)]
pub(super) struct StatusMessages(BTreeMap<StatusTarget, RecordedStatus>);

impl StatusMessages {
    fn record(
        &mut self,
        outcome: &crate::RunOutcome,
        setup: &cam_model::Setup,
        fingerprints: &BTreeMap<u32, String>,
        machine: &cam_model::Machine,
        accuracy: f32,
    ) {
        // Only replace operations touched by this run: a single-operation run
        // must preserve the messages belonging to its siblings.
        let mut touched: BTreeSet<_> = outcome.scene.strips.iter().filter_map(|s| s.op).collect();
        touched.extend(
            outcome
                .diagnostics
                .iter()
                .filter_map(|d| d.source.operation.or(d.op)),
        );
        touched.extend(outcome.collisions.iter().filter_map(|c| c.op_id));
        let mut targets = vec![StatusTarget::Setup(setup.id)];
        targets.extend(touched.into_iter().map(StatusTarget::Operation));
        for target in targets {
            let operation = match target {
                StatusTarget::Setup(_) => None,
                StatusTarget::Operation(id) => Some(id),
            };
            let scoped = crate::RunOutcome {
                diagnostics: outcome
                    .diagnostics
                    .iter()
                    .filter(|d| d.source.operation.or(d.op) == operation)
                    .cloned()
                    .collect(),
                collisions: outcome
                    .collisions
                    .iter()
                    .filter(|c| c.op_id == operation)
                    .cloned()
                    .collect(),
                // A failed run without any diagnostic still needs a setup error.
                completion: if operation.is_none() && !outcome.has_errors() {
                    outcome.completion
                } else {
                    JobCompletion::Complete
                },
                ..Default::default()
            };
            self.0.remove(&target);
            if !scoped.diagnostics.is_empty()
                || !scoped.collisions.is_empty()
                || matches!(
                    scoped.completion,
                    JobCompletion::Failed | JobCompletion::Incomplete
                )
            {
                let dependencies = fingerprints
                    .iter()
                    .filter(|(id, _)| {
                        operation.map_or_else(
                            || setup.operations.iter().any(|op| op.id() == **id),
                            |operation| operation == **id,
                        )
                    })
                    .map(|(id, value)| (*id, value.clone()))
                    .collect();
                self.0.insert(
                    target,
                    RecordedStatus {
                        dependencies,
                        outcome: scoped,
                        machine: machine.clone(),
                        accuracy,
                    },
                );
            }
        }
    }
}

fn target_calculating(
    target: StatusTarget,
    setup_active: bool,
    progress: Option<&crate::JobProgress>,
) -> bool {
    setup_active
        && progress.is_some_and(|progress| match target {
            StatusTarget::Setup(_) => true,
            StatusTarget::Operation(id) => {
                progress.phase == crate::JobPhase::Generation && progress.operation == Some(id)
            }
        })
}

impl App {
    pub(super) fn record_status_messages(&mut self) {
        if let Some(outcome) = self.controller.outcome() {
            self.status_messages.record(
                outcome,
                self.controller.inspection_setup(),
                &self.controller.current_toolpath_fingerprints(),
                self.controller.machine(),
                self.settings.simulation.accuracy,
            );
        }
    }

    pub(super) fn status_notice(
        &self,
        target: StatusTarget,
        fingerprints: &BTreeMap<u32, String>,
    ) -> Option<Notice> {
        let setup = self.controller.setups().iter().find(|setup| match target {
            StatusTarget::Setup(id) => setup.id == id,
            StatusTarget::Operation(id) => setup.operations.iter().any(|op| op.id() == id),
        })?;
        let mut targets = vec![StatusTarget::Setup(setup.id)];
        match target {
            StatusTarget::Setup(_) => targets.extend(
                setup
                    .operations
                    .iter()
                    .filter(|op| {
                        self.controller
                            .document()
                            .operation_is_included(setup.id, op.id())
                    })
                    .map(|op| StatusTarget::Operation(op.id())),
            ),
            StatusTarget::Operation(id) => {
                if !self
                    .controller
                    .document()
                    .operation_is_included(setup.id, id)
                {
                    return None;
                }
                targets.push(target);
            }
        }
        let mut lines = Vec::new();
        let mut entries = Vec::new();
        let collision_severity = self.controller.verification_collision_severity();
        for source in targets {
            let Some(recorded) = self.status_messages.0.get(&source) else {
                continue;
            };
            // Dependencies include upstream cuts, tools, geometry and machine
            // settings. Never present an old failure as a current one after edits.
            if recorded.machine != *self.controller.machine()
                || recorded.accuracy != self.settings.simulation.accuracy
                || !recorded
                    .dependencies
                    .iter()
                    .all(|(id, value)| fingerprints.get(id) == Some(value))
            {
                continue;
            }
            let label = match source {
                StatusTarget::Setup(_) => setup.name.clone(),
                StatusTarget::Operation(id) => format!("Operation #{id}"),
            };
            let Some(notice) = notices::job_notice_with_info(&recorded.outcome, &self.settings)
            else {
                continue;
            };
            lines.push(format!("{label}\n{}", notice.detail));
            entries.extend(notice.entries.into_iter().map(|mut entry| {
                if entry.verification_collision {
                    entry.severity = collision_severity;
                }
                entry.source = Some(label.clone());
                entry
            }));
        }
        if !entries.iter().any(|entry| entry.severity != Severity::Info) {
            return None;
        }
        let title = match target {
            StatusTarget::Setup(_) => setup.name.clone(),
            StatusTarget::Operation(id) => format!(
                "{} · operation #{id}",
                op_kind(setup.operations.iter().find(|op| op.id() == id)?)
            ),
        };
        let error = entries
            .iter()
            .any(|entry| entry.severity == Severity::Error);
        Some(Notice {
            title,
            detail: lines.join("\n\n"),
            entries,
            error,
        })
    }

    pub(super) fn status_indicator(
        &self,
        target: StatusTarget,
        statuses: &BTreeMap<u32, bool>,
        fingerprints: &BTreeMap<u32, String>,
    ) -> Element<'_, Message> {
        let Some(setup) = self.controller.setups().iter().find(|setup| match target {
            StatusTarget::Setup(id) => setup.id == id,
            StatusTarget::Operation(id) => setup.operations.iter().any(|op| op.id() == id),
        }) else {
            return Space::new().into();
        };
        let operations: Vec<_> = setup
            .operations
            .iter()
            .filter(|op| match target {
                StatusTarget::Setup(_) => true,
                StatusTarget::Operation(id) => op.id() == id,
            })
            .collect();
        let included: Vec<_> = operations
            .iter()
            .copied()
            .filter(|op| {
                self.controller
                    .document()
                    .operation_is_included(setup.id, op.id())
            })
            .collect();
        let ready = !included.is_empty()
            && included
                .iter()
                .all(|op| statuses.get(&op.id()).copied().unwrap_or(false));
        let suppressed = !operations.is_empty() && included.is_empty();
        let tools_available = included.iter().all(|op| {
            op.tools()
                .iter()
                .all(|number| setup.tools.iter().any(|tool| tool.number == *number))
        });
        let notice = self.status_notice(target, fingerprints);
        let running = !suppressed
            && !operations.is_empty()
            && target_calculating(
                target,
                setup.id == self.controller.active_setup_id(),
                self.controller.run_progress().as_ref(),
            );
        let calculated = match target {
            StatusTarget::Operation(id) if setup.id == self.controller.active_setup_id() => self
                .controller
                .run_progress()
                .is_some_and(|p| p.calculated_operations.contains(&id)),
            _ => false,
        };
        let (symbol, label, color) = if suppressed {
            ("–", "Suppressed", palette::GROUP_LABEL)
        } else if running {
            ("◌", "Calculating", palette::ACCENT_BLUE)
        } else if let Some(notice) = &notice {
            if notice.error {
                ("!", "Error", Color::from_rgb(0.95, 0.48, 0.40))
            } else {
                ("!", "Warning", palette::WARN)
            }
        } else if ready {
            ("✓", "Ready", Color::from_rgb8(70, 175, 85))
        } else if calculated {
            (
                "◌",
                "Calculated · verification pending",
                palette::ACCENT_BLUE,
            )
        } else if operations.is_empty() {
            ("–", "Empty", palette::GROUP_LABEL)
        } else {
            ("●", "Out of date", palette::WARN)
        };
        let mut summary = column![text(label.to_string()).size(14).color(color)].spacing(5);
        if !included.is_empty() {
            summary = summary
                .push(
                    text(if tools_available {
                        "✓ Tools available"
                    } else {
                        "! Missing tool"
                    })
                    .size(12),
                )
                .push(
                    text(if running {
                        "Toolpath calculation in progress"
                    } else if ready {
                        "✓ Toolpath up to date"
                    } else if calculated {
                        "Toolpath calculated; verification pending"
                    } else {
                        "Toolpath requires calculation"
                    })
                    .size(12),
                );
        }
        if notice.is_some() {
            summary = summary.push(text("Click for warning / error details").size(12));
        }
        // Capture the indicator click before the surrounding row's selection and
        // drag handlers, including when there is no diagnostic to open.
        let indicator = mouse_area(
            container(text(symbol).size(14).color(color))
                .width(18)
                .height(18)
                .center_x(18)
                .center_y(18),
        )
        .on_press(Message::ShowStatus(target))
        .interaction(if notice.is_some() {
            iced::mouse::Interaction::Pointer
        } else {
            iced::mouse::Interaction::Idle
        });
        delayed_tooltip(
            indicator,
            container(summary).padding(12).style(popup_surface),
            tooltip::Position::Right,
        )
        .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_toolpath::Diagnostic;

    fn app() -> App {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "status.dxf").unwrap();
        app.end_setup_draft();
        app.refresh_fields();
        app
    }

    fn record(app: &mut App, diagnostics: Vec<Diagnostic>) {
        app.status_messages.record(
            &crate::RunOutcome {
                completion: JobCompletion::Complete,
                diagnostics,
                ..Default::default()
            },
            app.controller.inspection_setup(),
            &app.controller.current_toolpath_fingerprints(),
            app.controller.machine(),
            app.settings.simulation.accuracy,
        );
    }

    fn notice(app: &App, target: StatusTarget) -> Option<Notice> {
        app.status_notice(target, &app.controller.current_toolpath_fingerprints())
    }

    #[test]
    fn operation_click_scopes_messages_and_setup_click_collects_them() {
        let mut app = app();
        let ids: Vec<_> = app
            .controller
            .inspection_setup()
            .operations
            .iter()
            .map(Operation::id)
            .collect();
        record(
            &mut app,
            vec![
                Diagnostic::warning("first warning").for_op(ids[0]),
                Diagnostic::error("second error").for_op(ids[1]),
                Diagnostic::info("ordinary information"),
            ],
        );
        assert!(app.notice.is_none());
        let first = notice(&app, StatusTarget::Operation(ids[0])).unwrap();
        assert!(!first.error && first.detail.contains("first warning"));
        assert!(!first.detail.contains("second error"));
        let setup = StatusTarget::Setup(app.controller.active_setup_id());
        let combined = notice(&app, setup).unwrap();
        assert!(
            combined.error
                && combined.detail.contains("first warning")
                && combined.detail.contains("second error")
        );
        assert!(combined.entries.iter().any(
            |entry| entry.severity == Severity::Info && entry.message == "ordinary information"
        ));

        let selection = app.controller.selection();
        let _ = app.update(Message::ShowStatus(StatusTarget::Operation(ids[0])));
        assert!(app
            .notice
            .as_ref()
            .is_some_and(|n| n.detail == first.detail));
        assert_eq!(app.controller.selection(), selection);
        assert!(app.tree_drag.is_none());
        let _ = app.update(Message::NoticeDismissed);
        assert!(app.notice.is_none());
        // Dismissing a dialog retains its messages for subsequent inspection.
        let _ = app.update(Message::ShowStatus(setup));
        assert!(app
            .notice
            .as_ref()
            .is_some_and(|n| n.detail == combined.detail));
    }

    #[test]
    fn messages_survive_setup_switches_but_follow_input_changes_and_suppression() {
        let mut app = app();
        let setup = app.controller.active_setup_id();
        let id = app.controller.inspection_setup().operations[0].id();
        let target = StatusTarget::Operation(id);
        record(
            &mut app,
            vec![Diagnostic::error("invalid operation").for_op(id)],
        );
        app.controller.add_setup();
        assert!(notice(&app, target).is_some());
        app.controller.activate_setup(setup).unwrap();
        assert!(notice(&app, target).is_some());
        app.controller.set_operation_excluded(id, true);
        assert!(notice(&app, target).is_none());
        app.controller.set_operation_excluded(id, false);
        assert!(notice(&app, target).is_some());
        app.controller
            .edit_setup(|s| s.operations[0].set_work_offset(2));
        assert!(notice(&app, target).is_none());
    }

    #[test]
    fn calculation_indicator_tracks_only_the_current_operation() {
        let mut progress = crate::JobProgress {
            phase: crate::JobPhase::Generation,
            operation: Some(3),
            ..Default::default()
        };
        assert!(target_calculating(
            StatusTarget::Setup(1),
            true,
            Some(&progress)
        ));
        assert!(target_calculating(
            StatusTarget::Operation(3),
            true,
            Some(&progress)
        ));
        assert!(!target_calculating(
            StatusTarget::Operation(1),
            true,
            Some(&progress)
        ));
        assert!(!target_calculating(
            StatusTarget::Operation(3),
            false,
            Some(&progress)
        ));
        progress.operation = Some(4);
        assert!(!target_calculating(
            StatusTarget::Operation(3),
            true,
            Some(&progress)
        ));
        assert!(target_calculating(
            StatusTarget::Operation(4),
            true,
            Some(&progress)
        ));
        progress.phase = crate::JobPhase::Verification;
        assert!(!target_calculating(
            StatusTarget::Operation(4),
            true,
            Some(&progress)
        ));
    }

    #[test]
    fn recalculating_one_operation_preserves_sibling_messages_and_clears_its_old_error() {
        let mut app = app();
        let ids: Vec<_> = app
            .controller
            .inspection_setup()
            .operations
            .iter()
            .map(Operation::id)
            .collect();
        record(
            &mut app,
            vec![
                Diagnostic::error("old error").for_op(ids[0]),
                Diagnostic::warning("sibling warning").for_op(ids[1]),
            ],
        );
        // A successfully regenerated backplot identifies which operation ran.
        let mut outcome = crate::RunOutcome {
            completion: JobCompletion::Complete,
            ..Default::default()
        };
        outcome.scene.strips.push(cam_render::LineStrip {
            points: vec![],
            color: cam_render::PART,
            width: 1.,
            op: Some(ids[0]),
            pattern_id: None,
            instance_index: None,
            dashed: false,
        });
        app.status_messages.record(
            &outcome,
            app.controller.inspection_setup(),
            &app.controller.current_toolpath_fingerprints(),
            app.controller.machine(),
            app.settings.simulation.accuracy,
        );
        assert!(notice(&app, StatusTarget::Operation(ids[0])).is_none());
        assert!(notice(&app, StatusTarget::Operation(ids[1])).is_some());
        let _ = app.update(Message::ShowStatus(StatusTarget::Operation(ids[0])));
        assert!(
            app.notice.is_none(),
            "clean status clicks do not open a modal"
        );
    }

    #[test]
    fn collisions_are_errors_and_setup_failures_are_visible_to_operations() {
        let mut app = app();
        let id = app.controller.inspection_setup().operations[0].id();
        let mut outcome = crate::RunOutcome {
            completion: JobCompletion::Complete,
            ..Default::default()
        };
        outcome.collisions.push(cam_sim::Collision {
            kind: cam_sim::CollisionKind::RapidThroughStock,
            at: [0.; 3],
            message: "rapid through stock".into(),
            op_id: Some(id),
            pattern_id: None,
            instance_index: None,
            step_index: None,
            fraction: 0.,
        });
        app.status_messages.record(
            &outcome,
            app.controller.inspection_setup(),
            &app.controller.current_toolpath_fingerprints(),
            app.controller.machine(),
            app.settings.simulation.accuracy,
        );
        assert!(notice(&app, StatusTarget::Operation(id))
            .is_some_and(|n| n.error && n.detail.contains("rapid through stock")));
        app.controller
            .set_block_export_on_verification_errors(false);
        let warning = notice(&app, StatusTarget::Operation(id)).unwrap();
        assert!(!warning.error);
        assert_eq!(warning.entries[0].severity, Severity::Warning);
        assert!(warning.detail.contains("rapid through stock"));
        let setup_id = app.controller.active_setup_id();
        record(
            &mut app,
            vec![Diagnostic::error("setup frame invalid").for_setup(setup_id)],
        );
        assert!(notice(&app, StatusTarget::Operation(id))
            .is_some_and(|n| n.detail.contains("setup frame invalid")));
        app.controller.edit_machine(|m| m.name.push_str(" changed"));
        assert!(notice(&app, StatusTarget::Operation(id)).is_none());
    }

    #[test]
    fn incomplete_runs_without_diagnostics_still_have_clickable_errors() {
        let mut app = app();
        let outcome = crate::RunOutcome {
            completion: JobCompletion::Incomplete,
            ..Default::default()
        };
        app.status_messages.record(
            &outcome,
            app.controller.inspection_setup(),
            &app.controller.current_toolpath_fingerprints(),
            app.controller.machine(),
            app.settings.simulation.accuracy,
        );
        assert!(
            notice(&app, StatusTarget::Setup(app.controller.active_setup_id()))
                .is_some_and(|n| n.error && n.detail.contains("did not complete"))
        );
        let _ = app.update(Message::NewProject);
        let _ = app.update(Message::DiscardConfirmed);
        assert!(app.status_messages.0.is_empty());
    }
}
