//! Transactional thread-size presets and geometry-driven suggestions.
use super::*;
use parameter_options::{operation_picker, THREAD_HAND, THREAD_SERIES, THREAD_SIZING};
use cam_model::{
    matching_thread_standards, thread_standards, ThreadDiameterBasis, ThreadSeries, ThreadStandard,
};
use iced::widget::column;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub(super) enum ThreadMatchMode {
    #[default]
    Manual,
    PreparedBore,
    Nominal,
}
impl ThreadMatchMode {
    const ALL: [Self; 3] = [Self::Manual, Self::PreparedBore, Self::Nominal];
    fn basis(self) -> ThreadDiameterBasis {
        if self == Self::Nominal {
            ThreadDiameterBasis::Nominal
        } else {
            ThreadDiameterBasis::PreparedBore
        }
    }
}
impl std::fmt::Display for ThreadMatchMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Manual => "Manual",
            Self::PreparedBore => "Match prepared bore",
            Self::Nominal => "Match nominal diameter",
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub(super) enum ThreadFamily {
    #[default]
    All,
    MetricCoarse,
    MetricFine,
    Unc,
    Unf,
}
impl ThreadFamily {
    const ALL: [Self; 5] = [
        Self::All,
        Self::MetricCoarse,
        Self::MetricFine,
        Self::Unc,
        Self::Unf,
    ];
    fn series(self) -> Option<ThreadSeries> {
        match self {
            Self::All => None,
            Self::MetricCoarse => Some(ThreadSeries::MetricCoarse),
            Self::MetricFine => Some(ThreadSeries::MetricFine),
            Self::Unc => Some(ThreadSeries::Unc),
            Self::Unf => Some(ThreadSeries::Unf),
        }
    }
}
impl std::fmt::Display for ThreadFamily {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(series) = self.series() {
            series.fmt(f)
        } else {
            f.write_str("All metric / Unified")
        }
    }
}

#[derive(Clone, Debug, Default)]
pub(super) struct ThreadParameters {
    pub mode: ThreadMatchMode,
    family: ThreadFamily,
    last_match: Option<(f64, bool, ThreadMatchMode, ThreadFamily)>,
}
impl ThreadParameters {
    pub fn for_creation(kind: OpKind) -> Self {
        Self {
            mode: if kind == OpKind::Thread {
                ThreadMatchMode::PreparedBore
            } else {
                ThreadMatchMode::Manual
            },
            ..Default::default()
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct ThreadChoice(Option<ThreadStandard>);
impl std::fmt::Display for ThreadChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(standard) = self.0 {
            standard.fmt(f)
        } else {
            f.write_str("Custom diameter / pitch")
        }
    }
}
#[derive(Clone, Debug)]
pub(super) enum ThreadMessage {
    Mode(ThreadMatchMode),
    Family(ThreadFamily),
    Preset(ThreadChoice),
}

impl App {
    /// Source diameter is independent of the editable nominal thread size. Do
    /// not match against major_dia after a preset has overwritten that value.
    fn selected_thread_diameter(&self) -> Option<(f64, bool)> {
        let operation = self.operation_draft.as_ref().or_else(|| {
            if let Selection::Operation(id) = self.controller.selection() {
                self.controller.operation(id)
            } else {
                None
            }
        })?;
        let Operation::Thread(thread) = operation else {
            return None;
        };
        if self.operation_uses_cad_geometry() {
            let refs = if self.cad_pending_kind.is_some() || self.cad_editing_op.is_some() {
                self.controller.cad_selection()
            } else {
                self.operation_binding_preview
                    .as_ref()
                    .or_else(|| self.controller.geometry_state().bindings.get(&thread.id))?
                    .selected
                    .as_slice()
            };
            let mut result: Option<(f64, bool)> = None;
            for reference in refs {
                if reference.kind != EntityKind::Face {
                    return None;
                }
                let asset = self.controller.geometry_state().asset(reference.asset_id)?;
                if asset.revision != reference.revision {
                    return None;
                }
                let face = asset
                    .topology
                    .faces
                    .iter()
                    .find(|face| face.id == reference.id)?;
                let cam_model::CadSurface::Cylinder { radius, inward, .. } = face.surface else {
                    return None;
                };
                let current = (radius * 2., inward);
                if result.is_some_and(|old| (old.0 - current.0).abs() > 1e-5 || old.1 != current.1)
                {
                    return None;
                }
                result = Some(current);
            }
            result
        } else {
            let refs = if self.creating_operation() || self.drawing_editing_op.is_some() {
                self.controller.pending_operation_geometry()
            } else {
                self.drawing_sources_for_operation(operation)
            };
            let Operation::Thread(source) = self
                .controller
                .operation_with_drawing_geometry(operation, &refs)
                .ok()?
            else {
                return None;
            };
            Some((source.major_dia, thread.internal))
        }
    }

    pub(super) fn thread_parameters_message(&mut self, message: ThreadMessage) {
        match message {
            ThreadMessage::Mode(mode) => self.thread_parameters.mode = mode,
            ThreadMessage::Family(family) => self.thread_parameters.family = family,
            ThreadMessage::Preset(choice) => {
                self.thread_parameters.mode = ThreadMatchMode::Manual;
                if let Some(standard) = choice.0 {
                    self.apply_thread_standard(standard);
                }
            }
        }
        self.thread_parameters.last_match = None;
    }

    fn apply_thread_standard(&mut self, standard: ThreadStandard) {
        self.edit_operation_draft(|operation| {
            if let Operation::Thread(thread) = operation {
                standard.apply(thread);
            }
        });
        self.fields
            .insert(Field::MajorDia, fmt_num(standard.major_mm));
        self.fields.insert(Field::Pitch, fmt_num(standard.pitch_mm));
        if self.creating_operation() {
            self.creation_edited_fields
                .extend([Field::MajorDia, Field::Pitch]);
        }
    }

    pub(super) fn sync_thread_standard(&mut self) {
        if self.thread_parameters.mode == ThreadMatchMode::Manual || self.geometry_editor.is_some()
        {
            return;
        }
        let Some((diameter, internal)) = self.selected_thread_diameter() else {
            self.thread_parameters.last_match = None;
            return;
        };
        let state = &self.thread_parameters;
        let key = (diameter, internal, state.mode, state.family);
        if state.last_match == Some(key) {
            return;
        }
        self.thread_parameters.last_match = Some(key);
        if let Some(candidate) =
            matching_thread_standards(diameter, internal, key.2.basis(), key.3.series()).first()
        {
            self.apply_thread_standard(candidate.standard);
        }
    }

    pub(super) fn thread_geometry_controls(
        &self,
        thread: &cam_model::ThreadOp,
    ) -> Element<'static, Message> {
        let state = &self.thread_parameters;
        let source = self.selected_thread_diameter();
        let candidates = source
            .map(|(diameter, internal)| {
                matching_thread_standards(
                    diameter,
                    internal,
                    state.mode.basis(),
                    state.family.series(),
                )
            })
            .unwrap_or_default();
        let mut standards: Vec<_> = candidates.iter().map(|m| m.standard).collect();
        for standard in thread_standards().iter().copied().filter(|s| {
            state
                .family
                .series()
                .is_none_or(|series| s.series == series)
        }) {
            if !standards.contains(&standard) {
                standards.push(standard);
            }
        }
        let value = |field, fallback| {
            if self.creating_operation()
                && field == Field::MajorDia
                && !self.creation_edited_fields.contains(&field)
            {
                fmt_num(fallback)
            } else {
                self.fields
                    .get(&field)
                    .cloned()
                    .unwrap_or_else(|| fmt_num(fallback))
            }
        };
        let major_value = value(Field::MajorDia, thread.major_dia);
        let pitch_value = value(Field::Pitch, thread.pitch);
        let selected = ThreadChoice(
            major_value
                .parse::<f64>()
                .ok()
                .zip(pitch_value.parse::<f64>().ok())
                .and_then(|(major, pitch)| {
                    standards
                        .iter()
                        .find(|s| s.matches_parameters(major, pitch))
                        .copied()
                }),
        );
        let options: Vec<_> = std::iter::once(ThreadChoice(None))
            .chain(standards.into_iter().map(|s| ThreadChoice(Some(s))))
            .collect();
        let mut body = column![
            label_help("Sizing", "Prepared-bore matching uses the basic minor diameter or approximate cut-thread drill size. Nominal matching uses the major diameter. Nearby sizes are suggestions; review the intended specification.", self.tooltips),
            THREAD_SIZING.picker(state.mode, &ThreadMatchMode::ALL, Length::Fill),
            text("Thread series").size(12),
            THREAD_SERIES.picker(state.family, &ThreadFamily::ALL, Length::Fill),
            label_help("Standard size", "Applies the standard nominal major diameter and pitch. Metric coarse/fine and Unified UNC/UNF are cylindrical 60-degree threads. Hand, cutting data and heights remain as configured. Nearby diameter matches appear first; diameter alone cannot identify a unique thread or certify its tolerance class.",self.tooltips),
            pick_list(options,Some(selected),|v| Message::ThreadParameters(ThreadMessage::Preset(v)))
                .text_size(12).padding([7,9]).style(inspector_picker).width(Length::Fill),
        ].spacing(10);
        let hint = match source {
            Some((diameter, internal)) => {
                let kind = if internal { "bore" } else { "boss" };
                if let Some(best) = candidates.first() {
                    format!("Selected {kind}: Ø{diameter:.3} mm · closest: {} (difference {:.3} mm). {} close option(s).",best.standard,best.error_mm,candidates.len())
                } else {
                    format!("Selected {kind}: Ø{diameter:.3} mm · no close match in this series. Current parameters retained.")
                }
            }
            None => "Select equal-diameter bores or bosses to suggest a standard size.".into(),
        };
        body = body.push(text(hint).size(11).color(INSPECTOR_MUTED));
        for (field, value) in [(Field::MajorDia, major_value), (Field::Pitch, pitch_value)] {
            body = body.push(self.parameter_field_row(field, field.label(), field.help(), &value));
        }
        body.push(operation_picker(
            "Hand",
            help::HAND,
            self.tooltips,
            thread.hand,
            &Hand::ALL,
            THREAD_HAND,
        ))
        .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cad_thread() -> App {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_project(
                Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../testfiles/3d_toolpath_calculation.ocam"),
            )
            .unwrap();
        let geometry = app.controller.geometry_state();
        let face = geometry.assets[0]
            .topology
            .faces
            .iter()
            .find(|face| {
                matches!(
                    face.surface,
                    cam_model::CadSurface::Cylinder {
                        radius: 15.,
                        inward: true,
                        ..
                    }
                )
            })
            .unwrap();
        let reference = EntityRef {
            asset_id: geometry.assets[0].id,
            revision: geometry.assets[0].revision,
            instance_id: geometry.instances[0].id,
            body_id: face.body_id,
            kind: EntityKind::Face,
            id: face.id,
        };
        app.cad_pending_kind = Some(OpKind::Thread);
        app.operation_draft = Some(app.controller.operation_template(OpKind::Thread, 42));
        app.thread_parameters = ThreadParameters::for_creation(OpKind::Thread);
        app.controller.set_cad_selection(vec![reference]);
        app.refresh_fields();
        app
    }

    #[test]
    fn automatic_prepared_bore_preset_sets_size_without_committing() {
        if !crate::test_fixture::available("3d_toolpath_calculation.ocam") {
            return;
        }
        let mut app = cad_thread();
        let before = app.controller.document().clone();
        app.fields.insert(Field::Feed, "321.75".into());
        app.sync_thread_standard();
        assert_eq!(app.fields[&Field::MajorDia], "32");
        assert_eq!(app.fields[&Field::Pitch], "2");
        assert_eq!(app.fields[&Field::Feed], "321.75");
        assert!(app.creation_edited_fields.contains(&Field::MajorDia));
        assert_eq!(app.controller.document(), &before);
        let _ = app.thread_geometry_controls(match app.operation_draft.as_ref().unwrap() {
            Operation::Thread(t) => t,
            _ => unreachable!(),
        });
    }

    #[test]
    fn manual_edits_stop_auto_matching_and_the_two_tabs_share_values() {
        if !crate::test_fixture::available("3d_toolpath_calculation.ocam") {
            return;
        }
        let mut app = cad_thread();
        app.sync_thread_standard();
        let _ = app.update(Message::FieldChanged(Field::Pitch, "1.75".into()));
        let _ = app.update(Message::ThreadHandChanged(Hand::Left));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Passes));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Geometry));
        assert_eq!(app.thread_parameters.mode, ThreadMatchMode::Manual);
        assert_eq!(app.fields[&Field::Pitch], "1.75");
        assert!(
            matches!(app.operation_draft,Some(Operation::Thread(ref t)) if t.hand == Hand::Left)
        );
        let _ = app.update(Message::ThreadParameters(ThreadMessage::Mode(
            ThreadMatchMode::PreparedBore,
        )));
        assert_eq!(app.fields[&Field::Pitch], "2");
    }

    #[test]
    fn explicit_imperial_preset_preserves_hand_feeds_and_exact_tpi_conversion() {
        if !crate::test_fixture::available("3d_toolpath_calculation.ocam") {
            return;
        }
        let mut app = cad_thread();
        let before = app.controller.document().clone();
        let _ = app.update(Message::ThreadHandChanged(Hand::Left));
        app.fields.insert(Field::Feed, "312.5".into());
        let standard = *thread_standards()
            .iter()
            .find(|s| s.series == ThreadSeries::Unf && s.inch_size == "1/4")
            .unwrap();
        let _ = app.update(Message::ThreadParameters(ThreadMessage::Preset(
            ThreadChoice(Some(standard)),
        )));
        assert_eq!(app.fields[&Field::MajorDia].parse::<f64>().unwrap(), 6.35);
        assert_eq!(
            app.fields[&Field::Pitch].parse::<f64>().unwrap(),
            25.4 / 28.
        );
        assert_eq!(app.fields[&Field::Feed], "312.5");
        assert!(
            matches!(app.operation_draft,Some(Operation::Thread(ref t)) if t.hand == Hand::Left)
        );
        assert_eq!(app.controller.document(), &before);
        let fractional = *thread_standards()
            .iter()
            .find(|s| s.series == ThreadSeries::Unf && s.inch_size == "7/16")
            .unwrap();
        let _ = app.update(Message::ThreadParameters(ThreadMessage::Preset(
            ThreadChoice(Some(fractional)),
        )));
        assert_eq!(app.fields[&Field::MajorDia], "11.1125");
        let _ = app.update(Message::CancelOp);
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn geometry_subdialog_does_not_apply_a_preset_before_confirmation() {
        if !crate::test_fixture::available("3d_toolpath_calculation.ocam") {
            return;
        }
        let mut app = cad_thread();
        let _ = app.update(Message::OperationGeometry(
            operation_geometry::GeometryMessage::Edit,
        ));
        assert!(app.geometry_editor.is_some());
        app.sync_thread_standard();
        assert!(!app.creation_edited_fields.contains(&Field::MajorDia));
        let _ = app.update(Message::OperationGeometry(
            operation_geometry::GeometryMessage::Cancel,
        ));
        assert_eq!(app.fields[&Field::MajorDia], "32");
        // Cancelling selection restores its original source before automatic
        // sizing runs; an unconfirmed alternative cannot size the operation.
        assert_eq!(app.selected_thread_diameter(), Some((30., true)));
    }

    #[test]
    fn committed_preset_survives_save_reopen_and_undo() {
        if !crate::test_fixture::available("3d_toolpath_calculation.ocam") {
            return;
        }
        let mut app = cad_thread();
        let reference = app.controller.cad_selection()[0].clone();
        app.cad_pending_kind = None;
        app.operation_draft = None;
        let tool = app.controller.use_tool(cam_model::Tool {
            number: 43,
            diameter: 8.,
            neck_diameter: 4.,
            flute_length: 32.,
            length: 62.,
            kind: ToolKind::ThreadMill { pitch: None },
            ..Default::default()
        });
        let id = app
            .controller
            .create_cad_operation_with_tool(OpKind::Thread, vec![reference], tool)
            .unwrap();
        let original = app.controller.operation(id).unwrap().clone();
        app.begin_operation_draft(id);
        let _ = app.update(Message::ThreadParameters(ThreadMessage::Mode(
            ThreadMatchMode::PreparedBore,
        )));
        let _ = app.update(Message::ThreadHandChanged(Hand::Left));
        let _ = app.update(Message::Apply);
        assert!(
            matches!(app.controller.operation(id),Some(Operation::Thread(t)) if t.major_dia == 32. && t.pitch == 2. && t.hand == Hand::Left),
            "{}",
            app.status
        );
        let path =
            std::env::temp_dir().join(format!("thread-preset-ui-{}.ocam", std::process::id()));
        app.controller.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert_eq!(reopened.operation(id), app.controller.operation(id));
        assert!(app.controller.undo());
        assert_eq!(app.controller.operation(id), Some(&original));
    }
}
