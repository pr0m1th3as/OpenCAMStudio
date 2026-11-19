//! Pocket finishing controls; all cutter compensation is computed by the strategy.
use super::*;
use cam_model::PocketFinishing;

pub(super) const DEFAULT_STEPDOWN: &str = "=min(tool_fluteLength * 0.75; tool_diameter * 2.5)";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum Number {
    Count,
    Stepover,
    Feed,
    Overlap,
}
impl Number {
    pub const ALL: [Self; 4] = [Self::Count, Self::Stepover, Self::Feed, Self::Overlap];
    pub fn label(self) -> &'static str {
        match self {
            Self::Count => "Number of finishing passes",
            Self::Stepover => "Finishing stepover (mm)",
            Self::Feed => "Finishing feed (mm/min)",
            Self::Overlap => "Finishing overlap (mm)",
        }
    }
    pub fn help(self) -> &'static str {
        match self {
            Self::Count => "Radial finishing cuts on the pocket walls and islands after roughing at each depth, or only at the final depth when selected. Includes the final wall pass.",
            Self::Stepover => "Material reserved for each finishing cut. Roughing leaves the pass count times this stepover in addition to the wall stock to leave.",
            Self::Feed => "Cutting feed for finishing passes and the repeat pass. Defaults to the pocket cutting feed. Entry feed overrides still apply.",
            Self::Overlap => "Extra cutting distance past each finishing loop's start to blend the closure mark.",
        }
    }
    pub fn get(self, p: &PocketFinishing) -> f64 {
        match self {
            Self::Count => p.count as f64,
            Self::Stepover => p.stepover,
            Self::Feed => p.feed,
            Self::Overlap => p.overlap,
        }
    }
    pub fn set(self, p: &mut PocketFinishing, value: f64) {
        match self {
            Self::Count => p.count = value as u32,
            Self::Stepover => p.stepover = value,
            Self::Feed => p.feed = value,
            Self::Overlap => p.overlap = value,
        }
    }
    pub fn invalid(self, value: f64) -> bool {
        !value.is_finite()
            || match self {
                Self::Count => value < 1. || value > 1000. || value.fract() != 0.,
                Self::Stepover => value <= 0.,
                Self::Feed | Self::Overlap => value < 0.,
            }
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) enum Toggle {
    Enabled,
    AllLeads,
    Repeat,
    FinishOnlyFinal,
}
impl Toggle {
    pub fn set(self, p: &mut PocketFinishing, value: bool) {
        match self {
            Self::Enabled => p.enabled = value,
            Self::AllLeads => p.lead_in_out_all = value,
            Self::Repeat => p.repeat_pass = value,
            Self::FinishOnlyFinal => p.finish_only_final = value,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) enum DepthToggle {
    Enabled,
    Even,
    ByDepth,
    ByArea,
}
impl DepthToggle {
    pub fn set(self, p: &mut cam_model::MultipleDepths, value: bool) {
        match self {
            Self::Enabled => p.enabled = value,
            Self::Even => p.even_stepdowns = value,
            Self::ByDepth => p.order_by_depth = value,
            Self::ByArea => p.order_by_area = value,
        }
    }
}

impl App {
    pub(super) fn multiple_depth_controls(
        &self,
        d: cam_model::MultipleDepths,
        stepdown: f64,
        face: bool,
    ) -> operation_sections::Form<'_> {
        let mut list = operation_sections::Form::new(self, OperationTab::Passes, None)
            .optional_section(
                "Multiple Depths",
                d.enabled,
                Some(|v| Message::PocketDepthToggle(DepthToggle::Enabled, v)),
            );
        if !d.enabled {
            return list;
        }
        let field = Field::Stepdown;
        let value = self
            .fields
            .get(&field)
            .cloned()
            .unwrap_or_else(|| fmt_num(stepdown));
        list = list.push(self.parameter_field_row(
            field,
            "Maximum Roughing Stepdown (mm)",
            "Maximum axial depth removed in a roughing pass.",
            &value,
        ));
        for (label, help, value, toggle) in [
            (
                "Use Even Stepdowns",
                "Distribute the total depth evenly without exceeding the maximum stepdown.",
                d.even_stepdowns,
                DepthToggle::Even,
            ),
            (
                "Order by Depth",
                "Clear every pocket region at each depth before descending to the next depth.",
                d.order_by_depth,
                DepthToggle::ByDepth,
            ),
            (
                "Order By Area",
                "Clear larger disconnected pocket regions first.",
                d.order_by_area,
                DepthToggle::ByArea,
            ),
        ] {
            if face && matches!(toggle, DepthToggle::ByDepth | DepthToggle::ByArea) {
                continue;
            }
            list = list.push(
                row![
                    label_help(label, help, self.tooltips),
                    checkbox(value)
                        .size(15)
                        .on_toggle(move |v| Message::PocketDepthToggle(toggle, v))
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
        }
        list
    }
    pub(super) fn pocket_finishing_controls(
        &self,
        p: &PocketFinishing,
    ) -> operation_sections::Form<'_> {
        let mut list = operation_sections::Form::new(self, OperationTab::Passes, None)
            .optional_section(
                "Finishing passes",
                p.enabled,
                Some(|value| Message::PocketFinishToggle(Toggle::Enabled, value)),
            );
        if !p.enabled {
            return list;
        }
        let number = |n: Number| {
            let field = Field::PocketFinish(n);
            let value = self
                .fields
                .get(&field)
                .cloned()
                .unwrap_or_else(|| fmt_num(n.get(p)));
            self.parameter_field_row(field, n.label(), n.help(), &value)
        };
        let toggle = |label, help, value, t| -> Element<'_, Message> {
            row![
                label_help(label, help, self.tooltips),
                checkbox(value)
                    .size(15)
                    .on_toggle(move |value| Message::PocketFinishToggle(t, value))
            ]
            .spacing(8)
            .align_y(Alignment::Center)
            .into()
        };
        list = list
            .push(number(Number::Count))
            .push(number(Number::Stepover));
        list = list.push(toggle("Lead-in/out for all finishing passes",
            "Apply the leads selected in Linking to every finishing pass. Otherwise only the first pass leads in and the last pass leads out. Disconnected loops retract to clearance.",
            p.lead_in_out_all, Toggle::AllLeads));
        list = list.push(number(Number::Feed));
        list = list.push(toggle("Repeat finishing pass",
            "Repeat the final wall pass once at the same size and finishing feed to remove material left by tool deflection.",
            p.repeat_pass, Toggle::Repeat));
        list = list.push(toggle("Finish only on final depth",
            "Run finishing passes and the repeat pass only at the final depth. Roughing still leaves the finishing allowance at every depth.",
            p.finish_only_final, Toggle::FinishOnlyFinal));
        list.push(number(Number::Overlap))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn editor() -> (App, u32) {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "pocket.dxf").unwrap();
        app.controller.new_operation(OpKind::Pocket);
        let id = app
            .controller
            .inspection_setup()
            .operations
            .last()
            .unwrap()
            .id();
        app.begin_operation_draft(id);
        (app, id)
    }

    #[test]
    fn adaptive_toggle_retains_the_stepover_and_initializes_legacy_pockets() {
        let (mut app, _) = editor();
        let _ = app.update(Message::PocketAdaptiveToggled(true));
        assert_eq!(app.fields[&Field::Engagement], "=tool_diameter * 0.4");
        let _ = app.update(Message::FieldChanged(Field::Engagement, "1.25".into()));
        let _ = app.update(Message::PocketAdaptiveToggled(false));
        assert!(
            matches!(&app.operation_draft, Some(Operation::Pocket(p)) if !p.clear.pocket_adaptive_enabled() && p.clear.clearing.engagement == 1.25)
        );
        let _ = app.update(Message::PocketAdaptiveToggled(true));
        assert_eq!(app.fields[&Field::Engagement], "1.25");
        assert!(
            matches!(&app.operation_draft, Some(Operation::Pocket(p)) if p.clear.pocket_adaptive_enabled())
        );
    }

    #[test]
    fn new_pockets_use_tool_diameter_for_adaptive_stepover() {
        let (mut app, _) = editor();
        let _ = app.update(Message::CancelOperationDraft);
        let _ = app.update(Message::BeginOp(OpKind::Pocket));
        let op = app.operation_draft.as_ref().unwrap();
        let Operation::Pocket(p) = op else {
            panic!("pocket")
        };
        assert!(!p.clear.pocket_adaptive_enabled());
        assert_eq!(
            cam_model::OperationParameter::Engagement.expression(op),
            Some("tool_diameter * 0.4")
        );
        let _ = app.update(Message::PocketAdaptiveToggled(true));
        let tool = app.wizard_tool().unwrap();
        assert!(
            (app.parsed_field_value(Field::Engagement).unwrap() - tool.diameter * 0.4).abs() < 1e-9
        );
    }

    #[test]
    fn face_defaults_and_multiple_depth_controls() {
        let (mut app, _) = editor();
        let _ = app.update(Message::CancelOperationDraft);
        let _ = app.update(Message::BeginOp(OpKind::Face));
        let tool = app.wizard_tool().unwrap();
        assert!(
            (app.parsed_field_value(Field::FaceOverlap).unwrap() - tool.diameter * 0.5).abs()
                < 1e-8
        );
        assert!(
            (app.parsed_field_value(Field::FaceOvershoot).unwrap() - (tool.diameter / 2.0 + 0.1))
                .abs()
                < 1e-8
        );
        let mut larger = tool.clone();
        larger.diameter *= 2.0;
        let values = app
            .operation_draft
            .as_ref()
            .unwrap()
            .evaluate_parameter_sources(Some(&larger), &Default::default())
            .unwrap();
        assert!(
            (values[&cam_model::OperationParameter::FaceOvershoot] - (larger.diameter / 2.0 + 0.1))
                .abs()
                < 1e-8
        );
        let _ = app.update(Message::PocketDepthToggle(DepthToggle::Even, true));
        let _ = app.update(Message::PocketDepthToggle(DepthToggle::Enabled, false));
        assert!(
            matches!(&app.operation_draft, Some(Operation::Face(f)) if f.multiple_depths.even_stepdowns && !f.multiple_depths.enabled)
        );
        let _ = app.update(Message::PocketDepthToggle(DepthToggle::Enabled, true));
        assert!(
            matches!(&app.operation_draft, Some(Operation::Face(f)) if f.multiple_depths.even_stepdowns && f.multiple_depths.enabled)
        );
    }

    #[test]
    fn finishing_feed_defaults_to_cutting_feed_formula() {
        let (mut app, _) = editor();
        let _ = app.update(Message::CancelOperationDraft);
        let _ = app.update(Message::BeginOp(OpKind::Pocket));
        let _ = app.update(Message::PocketFinishToggle(Toggle::Enabled, true));
        let field = Field::PocketFinish(Number::Feed);
        assert_eq!(app.fields[&field], "=feed");
        let _ = app.update(Message::FieldChanged(Field::Feed, "420".into()));
        assert!((app.parsed_field_value(field).unwrap() - 420.).abs() < 1e-9);
        assert_eq!(Number::Feed.label(), "Finishing feed (mm/min)");
    }

    #[test]
    fn pocket_stepdown_defaults_to_tool_formula() {
        let (mut app, _) = editor();
        let _ = app.update(Message::CancelOperationDraft);
        let _ = app.update(Message::BeginOp(OpKind::Pocket));
        let op = app.operation_draft.as_ref().unwrap();
        assert_eq!(
            cam_model::OperationParameter::Stepdown.expression(op),
            Some("min(tool_fluteLength * 0.75; tool_diameter * 2.5)")
        );
        let tool = app.wizard_tool().unwrap();
        assert!(
            (app.parsed_field_value(Field::Stepdown).unwrap()
                - (tool.flute_len() * 0.75).min(tool.diameter * 2.5))
            .abs()
                < 1e-8
        );
        let mut long_tool = tool.clone();
        long_tool.flute_length = tool.diameter * 10.0;
        let values = op
            .evaluate_parameter_sources(Some(&long_tool), &Default::default())
            .unwrap();
        assert!(
            (values[&cam_model::OperationParameter::Stepdown] - tool.diameter * 2.5).abs() < 1e-8
        );
    }

    #[test]
    fn pocket_maximum_stepover_defaults_to_tool_formula() {
        let (mut app, _) = editor();
        let _ = app.update(Message::CancelOperationDraft);
        let _ = app.update(Message::BeginOp(OpKind::Pocket));
        let tool = app.wizard_tool().unwrap();
        let field = Field::PocketStepover;
        assert!((app.parsed_field_value(field).unwrap() - tool.diameter * 0.95).abs() < 1e-8);
        let mut ball = tool.clone();
        ball.kind = ToolKind::BallMill;
        let values = app
            .operation_draft
            .as_ref()
            .unwrap()
            .evaluate_parameter_sources(Some(&ball), &Default::default())
            .unwrap();
        let stepover = values[&cam_model::OperationParameter::MaximumStepover];
        assert!((stepover - ball.diameter * 0.20).abs() < 1e-8);
    }

    #[test]
    fn saved_overlap_displays_as_stepover_and_can_be_overridden() {
        use cam_model::OperationParameter::{MaximumStepover, Overlap, PocketOverlap};

        let (mut app, id) = editor();
        let tool_number = app.controller.operation(id).unwrap().tool();
        let index = app
            .controller
            .inspection_setup()
            .tools
            .iter()
            .position(|tool| tool.number == tool_number)
            .unwrap();
        app.controller.edit_tool(index, |tool| tool.diameter = 6.);
        let field = Field::PocketStepover;
        for (expression, value, expected) in [
            (None, None, 3.0),
            (None, Some(5.7), 0.3),
            (Some(Overlap), None, 3.6),
            (Some(PocketOverlap), Some(0.5), 0.3),
        ] {
            app.controller.edit_operation(id, |op| {
                let Operation::Pocket(pocket) = op else {
                    unreachable!()
                };
                pocket.clear.pocket_overlap = value;
                pocket.parameter_expressions.clear();
                match expression {
                    Some(Overlap) => Overlap.set_source(op, "=40"),
                    Some(PocketOverlap) => PocketOverlap.set_source(op, "=tool_diameter * 0.95"),
                    _ => {}
                }
            });
            app.begin_operation_draft(id);
            assert!(!app.inspector_dirty());
            assert!((app.parsed_field_value(field).unwrap() - expected).abs() < 1e-8);
            let mut candidate = app.operation_draft.clone().unwrap();
            app.apply_parameter_sources(&mut candidate).unwrap();
            assert!(matches!(&candidate, Operation::Pocket(p) if p.maximum_stepover.is_none()));
            assert!(MaximumStepover.expression(&candidate).is_none());
        }
        let _ = app.update(Message::FieldChanged(field, "6.1".into()));
        assert!(app.field_invalid(field));
        let _ = app.update(Message::FieldChanged(field, "6".into()));
        assert!(!app.field_invalid(field));
        let _ = app.update(Message::FieldChanged(field, "3.6".into()));
        let _ = app.update(Message::Apply);
        let saved = serde_json::to_string(app.controller.operation(id).unwrap()).unwrap();
        assert!(
            matches!(serde_json::from_str::<Operation>(&saved).unwrap(), Operation::Pocket(p) if p.maximum_stepover == Some(3.6))
        );
        app.begin_operation_draft(id);
        assert_eq!(app.parsed_field_value(field).unwrap(), 3.6);
    }

    #[test]
    fn maximum_stepover_can_drive_other_pass_parameters() {
        let (mut app, _) = editor();
        let _ = app.update(Message::FieldChanged(Field::PocketStepover, "1.6".into()));
        let _ = app.update(Message::PocketFinishToggle(Toggle::Enabled, true));
        let field = Field::PocketFinish(Number::Stepover);
        let _ = app.update(Message::FieldChanged(field, "=maximumStepover / 4".into()));
        assert_eq!(app.parsed_field_value(field).unwrap(), 0.4);
    }

    #[test]
    fn stock_contours_stage_independently_apply_cancel_and_undo() {
        let (mut app, id) = editor();
        app.operation_tab = OperationTab::Geometry;
        let before = app.controller.operation(id).unwrap().clone();
        let fields = app.fields.clone();
        let message = |m| Message::OperationGeometry(m);
        let _ = app.update(message(GeometryMessage::StockContours(true)));
        let _ = app.update(message(GeometryMessage::SelectStock));
        assert!(app.stock_contour_editor());
        let _ = app.operation_geometry_settings();
        app.controller
            .set_pending_operation_geometry(&[LoopRef::outer(0)], None)
            .unwrap();
        let _ = app.update(message(GeometryMessage::Confirm));
        assert!(!app.stock_contour_editor());
        assert!(matches!(app.operation_draft, Some(Operation::Pocket(ref p))
            if p.stock_contours.enabled && p.stock_contours.regions.len() == 1));
        assert_eq!(app.controller.operation(id), Some(&before));
        assert_eq!(app.fields, fields);
        let selected = app.operation_draft.clone();
        let _ = app.update(message(GeometryMessage::SelectStock));
        let _ = app.update(Message::GeometryClear);
        let _ = app.update(message(GeometryMessage::Cancel));
        assert_eq!(app.operation_draft, selected);
        let _ = app.update(Message::Apply);
        let committed = app.controller.operation(id).unwrap().clone();
        assert!(matches!(committed, Operation::Pocket(ref p)
        if p.stock_contours.enabled && p.boundary == match &before {
            Operation::Pocket(old) => old.boundary.clone(), _ => unreachable!(),
        }));
        assert!(app.controller.undo());
        assert_eq!(app.controller.operation(id), Some(&before));
        assert!(app.controller.redo());
        assert_eq!(app.controller.operation(id), Some(&committed));
        app.begin_operation_draft(id);
        let _ = app.update(message(GeometryMessage::ResetStock));
        assert!(matches!(app.operation_draft, Some(Operation::Pocket(ref p))
            if p.stock_contours.enabled && p.stock_contours.regions.is_empty()));
        let _ = app.update(Message::CancelOperationDraft);
        assert_eq!(app.controller.operation(id), Some(&committed));
    }

    #[test]
    fn stock_contours_creation_restores_the_pocket_selection() {
        let (mut app, _) = editor();
        let _ = app.update(Message::CancelOperationDraft);
        let _ = app.update(Message::BeginOp(OpKind::Pocket));
        app.controller
            .set_pending_operation_geometry(&[LoopRef::outer(0)], None)
            .unwrap();
        let pending = app.controller.pending_operation_geometry();
        let _ = app.update(Message::OperationGeometry(GeometryMessage::StockContours(
            true,
        )));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::SelectStock));
        assert!(app.controller.pending_operation_geometry().is_empty());
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert_eq!(app.controller.pending_operation_geometry(), pending);
        let (Operation::Pocket(op), _) = app.creation_operation().unwrap() else {
            panic!("pocket")
        };
        assert!(op.stock_contours.enabled && op.stock_contours.regions.is_empty());
    }

    #[test]
    fn use_setup_stock_enables_stock_and_clears_the_active_selector() {
        let (mut app, id) = editor();
        let before = app.controller.operation(id).unwrap().clone();
        let _ = app.update(Message::OperationGeometry(GeometryMessage::ResetStock));
        assert!(
            matches!(&app.operation_draft, Some(Operation::Pocket(p)) if p.stock_contours.enabled)
        );
        assert!(app.status.contains("Setup stock selected"));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::SelectStock));
        app.controller
            .set_pending_operation_geometry(&[LoopRef::outer(0)], None)
            .unwrap();
        let _ = app.update(Message::OperationGeometry(GeometryMessage::ResetStock));
        assert!(app.controller.pending_operation_geometry().is_empty());
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(matches!(&app.operation_draft, Some(Operation::Pocket(p))
            if p.stock_contours.enabled && p.stock_contours.regions.is_empty() && p.stock_contours.source.is_none()));
        assert_eq!(app.controller.operation(id), Some(&before));
    }

    #[test]
    fn keep_tool_down_draft_applies_cancels_and_undoes() {
        let (mut app, id) = editor();
        let checked = |op: &Operation| matches!(op, Operation::Pocket(p) if p.clear.keep_tool_down);
        let _ = app.update(Message::PocketKeepToolDownToggled(true));
        assert!(checked(app.current_operation(id).unwrap()));
        assert!(!checked(app.controller.operation(id).unwrap()));
        let _ = app.update(Message::CancelOperationDraft);
        assert!(!checked(app.controller.operation(id).unwrap()));
        app.begin_operation_draft(id);
        let _ = app.update(Message::PocketKeepToolDownToggled(true));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Linking));
        let _ = app.update(Message::Apply);
        assert!(
            checked(app.controller.operation(id).unwrap()),
            "{}",
            app.status
        );
        assert!(app.controller.undo());
        assert!(!checked(app.controller.operation(id).unwrap()));
        assert!(app.controller.redo());
        assert!(checked(app.controller.operation(id).unwrap()));
    }

    #[test]
    fn finishing_draft_keeps_settings_commits_and_undoes_atomically() {
        let (mut app, id) = editor();
        let before = app.controller.operation(id).unwrap().clone();
        assert!(!operation_fields(&before)
            .iter()
            .any(|f| matches!(f, Field::PocketFinish(_))));
        let _ = app.update(Message::PocketFinishToggle(Toggle::Enabled, true));
        for (number, value) in [
            (Number::Count, "3"),
            (Number::Stepover, "0.4"),
            (Number::Feed, "123"),
            (Number::Overlap, "0.2"),
        ] {
            let _ = app.update(Message::FieldChanged(
                Field::PocketFinish(number),
                value.into(),
            ));
        }
        let _ = app.update(Message::PocketFinishToggle(Toggle::AllLeads, true));
        let _ = app.update(Message::PocketFinishToggle(Toggle::Repeat, true));
        let _ = app.update(Message::PocketFinishToggle(Toggle::FinishOnlyFinal, true));
        let _ = app.update(Message::PocketFinishToggle(Toggle::Enabled, false));
        let _ = app.update(Message::PocketFinishToggle(Toggle::Enabled, true));
        assert_eq!(app.fields[&Field::PocketFinish(Number::Count)], "3");
        assert_eq!(app.fields[&Field::PocketFinish(Number::Feed)], "123");
        let _ = app.update(Message::OperationSectionToggled(
            operation_sections::Section::new(OperationTab::Passes, "Finishing passes"),
        ));
        assert_eq!(app.controller.operation(id), Some(&before));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Linking));
        let _ = app.update(Message::Apply);
        let Some(Operation::Pocket(op)) = app.controller.operation(id) else {
            panic!("expected pocket");
        };
        assert_eq!(
            op.finishing,
            PocketFinishing {
                enabled: true,
                count: 3,
                stepover: 0.4,
                feed: 123.,
                overlap: 0.2,
                repeat_pass: true,
                finish_only_final: true,
                lead_in_out_all: true,
            }
        );
        assert!(app.controller.undo());
        assert_eq!(app.controller.operation(id), Some(&before));
    }

    #[test]
    fn invalid_finishing_fields_block_apply_and_cancel_preserves_operation() {
        let (mut app, id) = editor();
        let before = app.controller.operation(id).unwrap().clone();
        let _ = app.update(Message::PocketFinishToggle(Toggle::Enabled, true));
        for (number, value) in [
            (Number::Count, "1.5"),
            (Number::Count, "0"),
            (Number::Stepover, "0"),
            (Number::Feed, "-1"),
            (Number::Overlap, "NaN"),
        ] {
            let _ = app.update(Message::FieldChanged(
                Field::PocketFinish(number),
                value.into(),
            ));
            assert!(app.field_invalid(Field::PocketFinish(number)));
            assert!(app.any_field_invalid());
            let _ = app.update(Message::Apply);
            assert_eq!(app.controller.operation(id), Some(&before));
            let valid = if number == Number::Count { "1" } else { "0.6" };
            let _ = app.update(Message::FieldChanged(
                Field::PocketFinish(number),
                valid.into(),
            ));
        }
        let _ = app.update(Message::CancelOperationDraft);
        assert_eq!(app.controller.operation(id), Some(&before));
    }
}
