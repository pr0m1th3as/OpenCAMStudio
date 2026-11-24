//! Drill cycle selection and its mode-specific Passes fields.
use super::*;
use cam_model::{DrillCycleMode, DrillOp};

pub(super) const CYCLE_HELP: &str = "Cycle Type controls how the drill feeds, pauses and withdraws. Rigid tapping synchronizes the spindle and Z feed for entry and withdrawal and is shown for reference.";

pub(super) const FIELDS: [Field; 6] = [
    Field::Peck,
    Field::PeckReduction,
    Field::MinimumPeck,
    Field::AccumulatedPeck,
    Field::ChipBreakDistance,
    Field::Dwell,
];

pub(super) fn fields(mode: DrillCycleMode) -> &'static [Field] {
    match mode {
        DrillCycleMode::RapidOut => &[],
        DrillCycleMode::Counterboring => &[Field::Dwell],
        DrillCycleMode::ChipBreaking => &FIELDS,
        DrillCycleMode::DeepDrilling => &[
            Field::Peck,
            Field::PeckReduction,
            Field::MinimumPeck,
            Field::Dwell,
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_drill_cycle_defaults_and_mode_switches_use_the_passes_draft() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample.dxf").unwrap();
        app.controller.select(Selection::Setup);
        let before = app.controller.document().clone();
        let _ = app.update(Message::BeginOp(OpKind::Drill));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Passes));
        assert!(
            matches!(app.current_operation(0), Some(Operation::Drill(drill))
            if drill.cycle_mode() == DrillCycleMode::RapidOut)
        );
        assert!(FIELDS.iter().all(|field| !app.fields.contains_key(field)));

        let _ = app.update(Message::DrillCycleChanged(DrillCycleMode::Counterboring));
        assert_eq!(app.parsed_field_value(Field::Dwell).unwrap(), 0.0);
        assert!(!app.fields.contains_key(&Field::Peck));
        let _ = app.update(Message::DrillCycleChanged(DrillCycleMode::ChipBreaking));
        for (field, value) in FIELDS.into_iter().zip([0.75, 0.0, 0.75, 7.5, 0.1, 0.0]) {
            assert_eq!(app.parsed_field_value(field).unwrap(), value, "{field:?}");
        }
        let _ = app.update(Message::FieldChanged(
            Field::ChipBreakDistance,
            "0.2".into(),
        ));
        let _ = app.update(Message::DrillCycleChanged(DrillCycleMode::DeepDrilling));
        assert!(!app.fields.contains_key(&Field::ChipBreakDistance));
        assert!(!app.fields.contains_key(&Field::AccumulatedPeck));
        let _ = app.update(Message::DrillCycleChanged(DrillCycleMode::ChipBreaking));
        assert_eq!(
            app.parsed_field_value(Field::ChipBreakDistance).unwrap(),
            0.2
        );
        assert_eq!(app.controller.document(), &before);

        let hole = app
            .controller
            .regions()
            .iter()
            .enumerate()
            .flat_map(|(region, polygon)| {
                std::iter::once(LoopRef::outer(region))
                    .chain((0..polygon.holes().len()).map(move |hole| LoopRef::hole(region, hole)))
            })
            .find(|hole| {
                app.controller
                    .planar_operation_eligibility(OpKind::Drill, &[*hole])
                    .is_ok()
            })
            .unwrap();
        app.controller
            .set_pending_operation_geometry(&[hole], None)
            .unwrap();
        let _ = app.update(Message::ConfirmOp);
        assert!(!app.creating_operation(), "{}", app.status);
        let saved = serde_json::to_string(app.controller.document()).unwrap();
        let reopened: cam_model::Document = serde_json::from_str(&saved).unwrap();
        assert!(
            matches!(reopened.setup().operations.last(), Some(Operation::Drill(drill))
            if drill.cycle_mode() == DrillCycleMode::ChipBreaking
                && drill.cycle.chip_break_distance == 0.2)
        );
    }
}

pub(super) fn help(mode: DrillCycleMode) -> &'static str {
    match mode {
        DrillCycleMode::RapidOut => "Feeds straight to the hole bottom, then rapids back to the retract plane.",
        DrillCycleMode::Counterboring => "Feeds to the hole bottom and pauses for the Dwelling Period before the rapid withdrawal.",
        DrillCycleMode::ChipBreaking => "Feeds in pecks with short withdrawals to break chips. Accumulated Pecking Depth controls when the drill pulls out to the retract plane.",
        DrillCycleMode::DeepDrilling => "Pulls the drill out to the retract plane between every peck, then rapids back above the previous depth.",
    }
}

impl App {
    pub(super) fn drill_cycle_controls(&self, drill: &DrillOp) -> Element<'_, Message> {
        let mode = drill.cycle_mode();
        let mut controls = iced::widget::Column::new().spacing(10).push(
            row![
                help_wrap(text("Cycle Type").size(12), help(mode), self.tooltips),
                parameter_options::DRILL_CYCLE.picker(mode, &DrillCycleMode::ALL, Length::Fill),
            ]
            .spacing(8)
            .align_y(Alignment::Center),
        );
        for &field in fields(mode) {
            let value = self
                .fields
                .get(&field)
                .map(String::as_str)
                .unwrap_or_default();
            controls =
                controls.push(self.parameter_field_row(field, field.label(), field.help(), value));
        }
        controls.into()
    }

    pub(super) fn change_drill_cycle(&mut self, mode: DrillCycleMode) {
        let sources: Vec<_> = FIELDS
            .into_iter()
            .filter_map(|field| Some((field, self.fields.remove(&field)?)))
            .collect();
        self.edit_operation_draft(|operation| {
            for (field, source) in &sources {
                let valid = source.trim_start().starts_with('=')
                    || source.trim().parse::<f64>().is_ok_and(|v| {
                        !operation_tabs::operation_field_invalid(operation, *field, v)
                    });
                if valid {
                    if let Some(parameter) = parameter_expressions::parameter(*field) {
                        parameter.set_source(operation, source);
                    }
                }
            }
            if let Operation::Drill(drill) = operation {
                drill.set_cycle_mode(mode);
            }
        });
        self.refresh_operation_fields();
    }
}
