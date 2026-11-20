//! Formula-aware inspector adapters. Source buffers remain separate from the
//! value shown outside editing, so focusing a calculated value restores `=…`.
use super::*;
use cam_model::OperationParameter;
#[path = "gui_parameter_input.rs"]
mod input;

#[derive(Default)]
pub(super) struct Editor {
    pub focused: Option<Field>,
    pub cursor: usize,
    pub suggestion: usize,
}

pub(super) fn parameter(field: Field) -> Option<OperationParameter> {
    match field {
        Field::CuttingSpeed => Some(OperationParameter::CuttingSpeed),
        Field::FeedPerTooth => Some(OperationParameter::FeedPerTooth),
        Field::LiftHeight => Some(OperationParameter::LiftHeight),
        Field::OperationTop => Some(OperationParameter::OperationTop),
        Field::SpindleRpm => Some(OperationParameter::SpindleRpm),
        Field::RampFeed => Some(OperationParameter::RampFeed),
        Field::LeadInFeed => Some(OperationParameter::LeadInFeed),
        Field::LeadOutFeed => Some(OperationParameter::LeadOutFeed),
        Field::ContourRoughStepover => Some(OperationParameter::MaximumStepover),
        Field::ContourRoughSmoothing => Some(OperationParameter::ContourRoughSmoothing),
        Field::ContourRoughCount => Some(OperationParameter::ContourRoughCount),
        Field::Depth => Some(OperationParameter::Depth),
        Field::Stepover => Some(OperationParameter::Stepover),
        Field::TabWidth => Some(OperationParameter::TabWidth),
        Field::TabHeight => Some(OperationParameter::TabHeight),
        Field::TabSpacing => Some(OperationParameter::TabSpacing),
        Field::ProfileOffset => Some(OperationParameter::ProfileOffset),
        Field::Stepdown => Some(OperationParameter::Stepdown),
        Field::Feed => Some(OperationParameter::Feed),
        Field::PlungeFeed => Some(OperationParameter::PlungeFeed),
        Field::LeadInSize => Some(OperationParameter::LeadInSize),
        Field::LeadOutSize => Some(OperationParameter::LeadOutSize),
        Field::LeadOverlap => Some(OperationParameter::LeadOverlap),
        Field::RingStep => Some(OperationParameter::RingStep),
        Field::Scallop => Some(OperationParameter::Scallop),
        Field::PlungeA => Some(OperationParameter::PlungeA),
        Field::PlungeB => Some(OperationParameter::PlungeB),
        Field::ClearStepdown => Some(OperationParameter::ClearStepdown),
        Field::ClearOverlap => Some(OperationParameter::ClearOverlap),
        Field::ClearOffset => Some(OperationParameter::ClearOffset),
        Field::ClearEngagement => Some(OperationParameter::ClearEngagement),
        Field::ClearFeed => Some(OperationParameter::ClearFeed),
        Field::ClearPlungeFeed => Some(OperationParameter::ClearPlungeFeed),
        Field::ClearLeadOverlap => Some(OperationParameter::ClearLeadOverlap),
        Field::ClearLeadInSize => Some(OperationParameter::ClearLeadInSize),
        Field::ClearLeadOutSize => Some(OperationParameter::ClearLeadOutSize),
        Field::ClearPlungeA => Some(OperationParameter::ClearPlungeA),
        Field::ClearPlungeB => Some(OperationParameter::ClearPlungeB),
        Field::Engagement => Some(OperationParameter::Engagement),
        Field::Overlap => Some(OperationParameter::Overlap),
        Field::PocketStepover => Some(OperationParameter::MaximumStepover),
        Field::FaceOverlap => Some(OperationParameter::FaceOverlap),
        Field::FaceStartOffset => Some(OperationParameter::FaceStartOffset),
        Field::FaceOvershoot => Some(OperationParameter::FaceOvershoot),
        Field::DrillStartOffset => Some(OperationParameter::DrillStartOffset),
        Field::Peck => Some(OperationParameter::Peck),
        Field::Dwell => Some(OperationParameter::Dwell),
        Field::ChamferTop => Some(OperationParameter::ChamferTop),
        Field::ChamferWidth => Some(OperationParameter::ChamferWidth),
        Field::ChamferDepth => Some(OperationParameter::ChamferDepth),
        Field::ChamferTipOffset => Some(OperationParameter::ChamferTipOffset),
        Field::ChamferStep => Some(OperationParameter::ChamferStep),
        Field::BoreDiameter => Some(OperationParameter::BoreDiameter),
        Field::BoreTaper => Some(OperationParameter::BoreTaper),
        Field::BorePitch => Some(OperationParameter::BorePitch),
        Field::BoreAngle => Some(OperationParameter::BoreAngle),
        Field::BoreStepovers => Some(OperationParameter::BoreStepovers),
        Field::BoreStepover => Some(OperationParameter::BoreStepover),
        Field::BoreFinishing => Some(OperationParameter::BoreFinishing),
        Field::BoreRadialStock => Some(OperationParameter::BoreRadialStock),
        Field::BoreAxialStock => Some(OperationParameter::BoreAxialStock),
        Field::BoreTolerance => Some(OperationParameter::BoreTolerance),
        Field::BoreSafeDistance => Some(OperationParameter::BoreSafeDistance),
        Field::BoreLeadRadius => Some(OperationParameter::BoreLeadRadius),
        Field::MajorDia => Some(OperationParameter::MajorDia),
        Field::Pitch => Some(OperationParameter::Pitch),
        Field::ThreadTop => Some(OperationParameter::ThreadTop),
        Field::ThreadBottom => Some(OperationParameter::ThreadBottom),
        Field::ThreadPasses => Some(OperationParameter::ThreadPasses),
        Field::ThreadSpringPasses => Some(OperationParameter::ThreadSpringPasses),
        Field::ThreadDrillClearance => Some(OperationParameter::ThreadDrillClearance),
        Field::ThreadBlindAllowance => Some(OperationParameter::ThreadBlindAllowance),
        Field::SpiralAxialOffset => Some(OperationParameter::SpiralAxialOffset),
        Field::SpiralStepdowns => Some(OperationParameter::SpiralStepdowns),
        Field::SpiralStepover => Some(OperationParameter::SpiralStepover),
        Field::SpiralInnerLimit => Some(OperationParameter::SpiralInnerLimit),
        Field::SpiralOuterLimit => Some(OperationParameter::SpiralOuterLimit),
        Field::SpiralFromSlope => Some(OperationParameter::SpiralFromSlope),
        Field::SpiralToSlope => Some(OperationParameter::SpiralToSlope),
        Field::SpiralStayDownDistance => Some(OperationParameter::SpiralStayDownDistance),
        Field::ParallelStepover => Some(OperationParameter::ParallelStepover),
        Field::ParallelAngle => Some(OperationParameter::ParallelAngle),
        Field::ScallopSafeDistance => Some(OperationParameter::ScallopSafeDistance),
        Field::ScallopSmoothingTolerance => Some(OperationParameter::ScallopSmoothingTolerance),
        Field::AdaptiveTolerance => Some(OperationParameter::AdaptiveTolerance),
        Field::AdaptiveOffset => Some(OperationParameter::AdaptiveOffset),
        Field::AdaptiveOptimalLoad => Some(OperationParameter::AdaptiveOptimalLoad),
        Field::AdaptiveFineStepdown => Some(OperationParameter::AdaptiveFineStepdown),
        Field::Profile(profile_parameters::Number::FinishCount) => {
            Some(OperationParameter::ProfileFinishCount)
        }
        Field::Profile(profile_parameters::Number::FinishStep) => {
            Some(OperationParameter::FinishingStepover)
        }
        Field::Profile(profile_parameters::Number::FinishFeed) => {
            Some(OperationParameter::ProfileFinishFeed)
        }
        Field::Profile(profile_parameters::Number::SpringPasses) => {
            Some(OperationParameter::ProfileSpringPasses)
        }
        Field::Profile(profile_parameters::Number::FloorLeave) => {
            Some(OperationParameter::ProfileFloorLeave)
        }
        Field::Profile(profile_parameters::Number::FinishDepths) => {
            Some(OperationParameter::ProfileFinishDepths)
        }
        Field::Profile(profile_parameters::Number::FinishDepthStep) => {
            Some(OperationParameter::ProfileFinishDepthStep)
        }
        Field::Profile(profile_parameters::Number::TangentialExtensionDistance) => {
            Some(OperationParameter::ProfileTangentialExtensionDistance)
        }
        Field::Profile(profile_parameters::Number::StartExtension) => {
            Some(OperationParameter::ProfileStartExtension)
        }
        Field::Profile(profile_parameters::Number::EndExtension) => {
            Some(OperationParameter::ProfileEndExtension)
        }
        Field::Profile(profile_parameters::Number::Tolerance) => {
            Some(OperationParameter::ProfileTolerance)
        }
        Field::Profile(profile_parameters::Number::Taper) => Some(OperationParameter::ProfileTaper),
        Field::Profile(profile_parameters::Number::LeadEndDistance) => {
            Some(OperationParameter::ProfileLeadEndDistance)
        }
        Field::Profile(profile_parameters::Number::RampRpm) => {
            Some(OperationParameter::ProfileRampRpm)
        }
        Field::Profile(profile_parameters::Number::TabCount) => {
            Some(OperationParameter::ProfileTabCount)
        }
        Field::PocketFinish(pocket_parameters::Number::Count) => {
            Some(OperationParameter::PocketFinishCount)
        }
        Field::PocketFinish(pocket_parameters::Number::Stepover) => {
            Some(OperationParameter::PocketFinishStepover)
        }
        Field::PocketFinish(pocket_parameters::Number::Feed) => {
            Some(OperationParameter::PocketFinishFeed)
        }
        Field::PocketFinish(pocket_parameters::Number::Overlap) => {
            Some(OperationParameter::PocketFinishOverlap)
        }
        Field::Profile(profile_parameters::Number::LeadInSweep) => {
            Some(OperationParameter::ProfileLeadInSweep)
        }
        Field::Profile(profile_parameters::Number::LeadOutSweep) => {
            Some(OperationParameter::ProfileLeadOutSweep)
        }
        Field::StartX => Some(OperationParameter::StartX),
        Field::StartY => Some(OperationParameter::StartY),
        _ => None,
    }
}
fn input_id(field: Field) -> String {
    format!("parameter-expression-{field:?}")
}
fn validate_source(source: &str) -> Result<(), String> {
    let source = source.trim();
    if source.starts_with('=') || source.parse::<f64>().is_ok() {
        Ok(())
    } else {
        Err("Start a formula with =, or enter a number".into())
    }
}
/// Return the entire identifier under the caret and its typed prefix. Caret
/// positions use the same grapheme indexing as iced's text input.
fn completion_token(source: &str, cursor: usize) -> Option<(std::ops::Range<usize>, &str)> {
    if !source.trim_start().starts_with('=') {
        return None;
    }
    let caret = iced::widget::text_input::Value::new(source)
        .until(cursor)
        .to_string()
        .len();
    let identifier = |byte: u8| byte.is_ascii_alphanumeric() || byte == b'_' || byte == b'.';
    let mut start = caret;
    while start > 0 && identifier(source.as_bytes()[start - 1]) {
        start -= 1;
    }
    let prefix = &source[start..caret];
    if !prefix.starts_with(|c: char| c.is_ascii_alphabetic() || c == '_') {
        return None;
    }
    let mut end = caret;
    while end < source.len() && identifier(source.as_bytes()[end]) {
        end += 1;
    }
    Some((start..end, prefix))
}
impl App {
    pub(super) fn parameter_input_state(&mut self, field: Field, focused: bool, cursor: usize) {
        if focused {
            if let Some(previous) = self
                .parameter_editor
                .focused
                .filter(|previous| *previous != field)
            {
                self.finish_parameter_edit(previous);
            }
            if self.parameter_editor.focused != Some(field)
                || self.parameter_editor.cursor != cursor
            {
                self.parameter_editor.suggestion = 0;
            }
            self.parameter_editor.focused = Some(field);
            self.parameter_editor.cursor = cursor;
        } else if self.parameter_editor.focused == Some(field) {
            self.finish_parameter_edit(field);
        }
    }
    fn finish_parameter_edit(&mut self, field: Field) {
        if let Some(mut operation) = self.parameter_operation().cloned() {
            // Invalid text stays visible and editable; the last numeric snapshot
            // remains intact until a valid edit is completed.
            if self.apply_parameter_sources(&mut operation).is_ok() {
                let values = operation
                    .evaluate_parameter_sources(self.parameter_tool(&operation), &BTreeMap::new())
                    .ok();
                self.edit_operation_draft(|draft| *draft = operation);
                for field in [
                    Field::SpindleRpm,
                    Field::Feed,
                    Field::CuttingSpeed,
                    Field::FeedPerTooth,
                ] {
                    if self
                        .fields
                        .get(&field)
                        .is_some_and(|s| !s.trim_start().starts_with('='))
                    {
                        if let Some(value) = parameter(field).and_then(|p| values.as_ref()?.get(&p))
                        {
                            let value =
                                if matches!(field, Field::CuttingSpeed | Field::FeedPerTooth) {
                                    self.field_value(field).unwrap_or(*value)
                                } else {
                                    *value
                                };
                            self.fields.insert(field, fmt_num(value));
                        }
                    }
                }
            }
        }
        if self.parameter_editor.focused == Some(field) {
            self.parameter_editor = Editor::default();
        }
    }
    fn variable_names(&self, field: Field) -> Vec<&'static str> {
        let Some(op) = self.parameter_operation() else {
            return Vec::new();
        };
        let mut names = vec![
            "tool_diameter",
            "tool_cornerRadius",
            "tool_length",
            "tool_fluteLength",
            "tool_flutes",
        ];
        names.extend(
            OperationParameter::ALL
                .iter()
                .filter(|&&p| Some(p) != parameter(field) && p.value(op).is_some())
                .filter(|&&p| {
                    !matches!(op, Operation::Pocket(_))
                        || !matches!(
                            p,
                            OperationParameter::Overlap | OperationParameter::PocketOverlap
                        )
                })
                .map(|p| p.name()),
        );
        names
    }
    pub(super) fn parameter_suggestions(&self, field: Field) -> Vec<&'static str> {
        if self.parameter_editor.focused != Some(field) {
            return Vec::new();
        }
        let Some(source) = self.fields.get(&field) else {
            return Vec::new();
        };
        let Some((_, prefix)) = completion_token(source, self.parameter_editor.cursor) else {
            return Vec::new();
        };
        let prefix_lower = prefix.to_ascii_lowercase();
        self.variable_names(field)
            .into_iter()
            .filter(|name| name.to_ascii_lowercase().starts_with(&prefix_lower) && *name != prefix)
            .collect()
    }
    pub(super) fn move_parameter_suggestion(&mut self, field: Field, delta: i32) {
        let len = self.parameter_suggestions(field).len();
        if len > 0 {
            self.parameter_editor.suggestion =
                (self.parameter_editor.suggestion as i32 + delta).rem_euclid(len as i32) as usize;
        }
    }
    pub(super) fn complete_parameter(
        &mut self,
        field: Field,
        name: &'static str,
        caret: usize,
    ) -> iced::Task<Message> {
        // A mouse press can blur the input immediately before this message. Use
        // the preserved source/caret so selecting a suggestion never loses text.
        let Some(source) = self.fields.get(&field) else {
            return iced::Task::none();
        };
        let Some((range, _)) = completion_token(source, caret) else {
            return iced::Task::none();
        };
        if !self.variable_names(field).contains(&name) {
            return iced::Task::none();
        }
        let cursor =
            iced::widget::text_input::Value::new(&source[..range.start]).len() + name.len();
        let mut source = source.clone();
        source.replace_range(range, name);
        self.fields.insert(field, source);
        if self.creating_operation() {
            self.creation_edited_fields.insert(field);
        }
        self.parameter_editor = Editor {
            focused: Some(field),
            cursor,
            suggestion: 0,
        };
        iced::widget::operation::focus(input_id(field)).chain(
            iced::widget::operation::move_cursor_to(input_id(field), cursor),
        )
    }
    pub(super) fn parsed_operation_fields(&self) -> BTreeMap<Field, f64> {
        self.fields
            .keys()
            .filter_map(|&field| {
                self.parsed_field_value(field)
                    .ok()
                    .map(|value| (field, value))
            })
            .collect()
    }
    fn parameter_operation(&self) -> Option<&Operation> {
        if self.library_mode() {
            return None;
        }
        self.operation_draft.as_ref().or_else(|| {
            if let Selection::Operation(id) = self.controller.selection() {
                self.controller.operation(id)
            } else {
                None
            }
        })
    }
    fn parameter_tool(&self, op: &Operation) -> Option<&cam_model::Tool> {
        self.operation_tools
            .iter()
            .chain(self.controller.inspection_setup().tools.iter())
            .find(|tool| tool.number == op.tool())
    }
    fn parameter_buffers(&self) -> BTreeMap<OperationParameter, String> {
        self.fields
            .iter()
            .filter_map(|(field, source)| parameter(*field).map(|p| (p, source.clone())))
            .collect()
    }
    fn parameter_overrides(&self) -> Result<BTreeMap<OperationParameter, String>, String> {
        let mut overrides = BTreeMap::new();
        for (field, source) in &self.fields {
            if let Some(parameter) = parameter(*field) {
                validate_source(source)?;
                overrides.insert(parameter, source.clone());
            }
        }
        Ok(overrides)
    }
    /// Validation, previews, creation and Apply consume the same source graph,
    /// including parameters whose sections have been collapsed.
    pub(super) fn parsed_field_value(&self, field: Field) -> Result<f64, String> {
        if let (Some(parameter), Some(op)) = (parameter(field), self.parameter_operation()) {
            if matches!(field, Field::CuttingSpeed | Field::FeedPerTooth) {
                if let Some(source) = self
                    .fields
                    .get(&field)
                    .filter(|s| !s.trim_start().starts_with('='))
                {
                    let literal = source.trim().parse::<f64>().ok().filter(|v| v.is_finite());
                    let pending_default = literal == Some(0.)
                        && self.creating_operation()
                        && !self.creation_edited_fields.contains(&field);
                    if !pending_default && !literal.is_some_and(|v| v > 0.) {
                        return Err(
                            "Enter a positive cutting value or a formula starting with =".into(),
                        );
                    }
                }
            }
            if matches!(field, Field::CuttingSpeed | Field::FeedPerTooth)
                && ![
                    Field::CuttingSpeed,
                    Field::FeedPerTooth,
                    Field::SpindleRpm,
                    Field::Feed,
                ]
                .iter()
                .any(|f| {
                    self.fields
                        .get(f)
                        .is_some_and(|s| s.trim_start().starts_with('='))
                })
            {
                if let Some(tool) = self.operation_preset_tool() {
                    let rpm = self
                        .fields
                        .get(&Field::SpindleRpm)
                        .and_then(|s| s.parse::<f64>().ok())
                        .unwrap_or(op.spindle_rpm());
                    let rpm = self.effective_cutting_rpm(rpm);
                    let feed = self
                        .fields
                        .get(&Field::Feed)
                        .and_then(|s| s.parse::<f64>().ok())
                        .or_else(|| op_field(op, Field::Feed))
                        .unwrap_or(0.);
                    return if field == Field::CuttingSpeed {
                        tool.cutting_speed(rpm)
                    } else {
                        tool.feed_per_tooth(rpm, feed)
                    }
                    .ok_or_else(|| "Cutting data unavailable".into());
                }
            }
            if field != Field::PocketStepover
                && !op.has_parameter_expressions()
                && !self
                    .fields
                    .values()
                    .any(|source| source.trim_start().starts_with('='))
            {
                return self
                    .fields
                    .get(&field)
                    .and_then(|s| s.trim().parse::<f64>().ok())
                    .filter(|value| value.is_finite())
                    .ok_or_else(|| "Enter a number or a formula starting with =".into());
            }
            if parameter.value(op).is_some() {
                return op
                    .evaluate_parameter_source_results(
                        self.parameter_tool(op),
                        &self.parameter_buffers(),
                    )
                    .remove(&parameter)
                    .unwrap_or_else(|| Err("Parameter unavailable".into()));
            }
        }
        self.fields
            .get(&field)
            .and_then(|s| s.trim().parse::<f64>().ok())
            .filter(|v| v.is_finite())
            .ok_or_else(|| "Enter a finite number".into())
    }
    pub(super) fn apply_parameter_sources(&self, op: &mut Operation) -> Result<(), String> {
        let mut overrides = self.parameter_overrides()?;
        let stepover = OperationParameter::MaximumStepover;
        if let (Some(source), Operation::Pocket(pocket)) = (overrides.get(&stepover), &mut *op) {
            if self
                .parameter_operation()
                .and_then(|op| self.legacy_pocket_stepover_source(op))
                .as_ref()
                == Some(source)
            {
                // The inspector's displayed conversion can still follow a later tool change.
                overrides.remove(&stepover);
                pocket.maximum_stepover = None;
            }
        }
        // Validate the source graph before writing the pending edits.
        let values = op.evaluate_parameter_sources(self.parameter_tool(op), &overrides)?;
        for (parameter, source) in &overrides {
            if matches!(
                parameter,
                OperationParameter::CuttingSpeed | OperationParameter::FeedPerTooth
            ) && !source.trim_start().starts_with('=')
            {
                continue;
            }
            if let Some(value) = values.get(parameter) {
                parameter.validate_value(op, *value)?;
            }
        }
        for (parameter, source) in overrides {
            parameter.set_source(op, &source);
        }
        let tools: Vec<_> = self
            .operation_tools
            .iter()
            .chain(self.controller.inspection_setup().tools.iter())
            .cloned()
            .collect();
        op.resolve_parameter_expressions(&tools)
    }
    fn legacy_pocket_stepover_source(&self, op: &Operation) -> Option<String> {
        let Operation::Pocket(pocket) = op else {
            return None;
        };
        if pocket.maximum_stepover.is_some()
            || OperationParameter::MaximumStepover.expression(op).is_some()
        {
            return None;
        }
        if let Some(source) = pocket.legacy_stepover_expression() {
            return Some(format!("={source}"));
        }
        let tool = self.parameter_tool(op)?;
        Some(fmt_num(pocket.concentric_stepover(tool.diameter)))
    }
    pub(super) fn saved_field_source(&self, field: Field) -> Option<String> {
        if field == Field::PocketStepover {
            if let Some(source) = self
                .parameter_operation()
                .and_then(|op| self.legacy_pocket_stepover_source(op))
            {
                return Some(source);
            }
        }
        if field == Field::FaceOverlap {
            if let Some(op @ Operation::Face(face)) = self.parameter_operation() {
                if face.absolute_overlap.is_none()
                    && cam_model::OperationParameter::FaceOverlap
                        .expression(op)
                        .is_none()
                {
                    if cam_model::OperationParameter::Overlap
                        .expression(op)
                        .is_some()
                    {
                        return Some("=overlap * tool_diameter / 100".into());
                    }
                    if let Some(tool) = self.parameter_tool(op) {
                        return Some(fmt_num(face.overlap * tool.diameter));
                    }
                }
            }
        }
        if let (Some(parameter), Some(op)) = (parameter(field), self.parameter_operation()) {
            if let Some(source) = parameter.expression(op) {
                return Some(format!("={source}"));
            }
        }
        self.field_value(field).map(fmt_num)
    }
    pub(super) fn parameter_fields_dirty(&self) -> bool {
        self.fields
            .iter()
            .filter(|(field, _)| parameter(**field).is_some())
            .any(|(&field, buffer)| match self.saved_field_source(field) {
                Some(saved) => match (buffer.trim().parse::<f64>(), saved.trim().parse::<f64>()) {
                    (Ok(a), Ok(b)) => a != b,
                    _ => buffer.trim() != saved.trim(),
                },
                _ => false,
            })
    }
    fn parameter_display_result(&self, field: Field) -> Result<f64, String> {
        // Other fields keep their resolved values until this edit is finished.
        if self
            .parameter_editor
            .focused
            .is_some_and(|active| active != field)
        {
            return self
                .field_value(field)
                .ok_or_else(|| "Parameter unavailable".into());
        }
        let value = self.parsed_field_value(field)?;
        if let (Some(parameter), Some(operation)) = (parameter(field), self.parameter_operation()) {
            parameter.validate_value(operation, value)?;
        }
        Ok(value)
    }
    pub(super) fn parameter_display_value(&self, field: Field, source: &str) -> String {
        if self.parameter_editor.focused == Some(field) {
            return source.into();
        }
        {
            if let Ok(value) = self.parameter_display_result(field) {
                return if parameter(field).is_some_and(|p| p.integer()) {
                    format!("{value:.0}")
                } else {
                    self.settings.format_number(value)
                };
            }
        }
        self.settings.format_numeric_input(source, false)
    }
    pub(super) fn parameter_field_row<'a>(
        &self,
        field: Field,
        label: &'a str,
        help: &'static str,
        source: &str,
    ) -> Element<'a, Message> {
        if parameter(field).is_none() || self.parameter_operation().is_none() {
            return field_row_labeled(
                field,
                label,
                help,
                source,
                self.tooltips,
                self.field_invalid(field),
            );
        }
        let editing = self.parameter_editor.focused == Some(field);
        let result = (!editing).then(|| self.parameter_display_result(field));
        let value = self.parameter_display_value(field, source);
        let expanded = editing && source.trim_start().starts_with('=');
        let mut text_input = inspector_input("Number or =formula", &value)
            .id(input_id(field))
            .on_input(move |value| Message::FieldChanged(field, value))
            .align_x(if expanded {
                iced::alignment::Horizontal::Left
            } else {
                iced::alignment::Horizontal::Right
            })
            .width(if expanded {
                Length::Fill
            } else {
                Length::Fixed(INSPECTOR_INPUT_W)
            });
        if matches!(result, Some(Err(_))) {
            text_input = text_input.style(|theme, status| {
                let mut style = inspector_input_style(theme, status);
                style.border.color = palette::ERROR;
                style.value = palette::ERROR;
                style
            });
        }
        let suggestions = self.parameter_suggestions(field);
        let active = self
            .parameter_editor
            .suggestion
            .min(suggestions.len().saturating_sub(1));
        let control = input::formula_input(
            text_input.into(),
            field,
            source,
            &value,
            editing,
            self.parameter_editor.cursor,
            suggestions.clone(),
            active,
        );
        // Keep the same widget hierarchy in both modes. Replacing a row with a
        // column here destroys the text-input state as soon as '=' is typed.
        let label = container(label_help(label, help, self.tooltips)).width(if expanded {
            Length::Fixed(110.)
        } else {
            Length::Fill
        });
        let mut form =
            iced::widget::column![row![label, control].spacing(8).align_y(Alignment::Center)]
                .spacing(3);
        if !suggestions.is_empty() {
            let mut choices = iced::widget::Column::new().spacing(1);
            for (index, name) in suggestions.into_iter().enumerate() {
                let choice = container(text(name).size(12))
                    .padding([5, 8])
                    .width(Length::Fill)
                    .style(move |_| container::Style {
                        background: (index == active)
                            .then_some(Color::from_rgb(0.20, 0.27, 0.34).into()),
                        text_color: Some(INSPECTOR_TEXT),
                        ..Default::default()
                    });
                choices = choices.push(
                    iced::widget::mouse_area(choice)
                        .on_press(Message::ParameterComplete(
                            field,
                            name,
                            self.parameter_editor.cursor,
                        ))
                        .interaction(iced::mouse::Interaction::Pointer),
                );
            }
            form = form.push(
                container(choices)
                    .padding(3)
                    .width(Length::Fill)
                    .style(|_| container::Style {
                        background: Some(Color::from_rgb(0.12, 0.13, 0.15).into()),
                        border: iced::Border {
                            color: INSPECTOR_MUTED,
                            width: 1.,
                            radius: 4.0.into(),
                        },
                        ..Default::default()
                    }),
            );
        }
        if let Some(Err(error)) = result {
            form = form.push(text(error).size(11).color(palette::ERROR));
        }
        form.into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const FINISH: Field = Field::Profile(profile_parameters::Number::FinishStep);
    const MAXIMUM: Field = Field::ContourRoughStepover;
    fn editor() -> (App, u32) {
        let (mut app, _) = App::new();
        app.settings.number_decimals = 2;
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_dxf(SAMPLE_DXF, "expressions.dxf")
            .unwrap();
        app.controller.new_operation(OpKind::Profile);
        let id = app
            .controller
            .inspection_setup()
            .operations
            .last()
            .unwrap()
            .id();
        let number = app.controller.operation(id).unwrap().tool();
        let index = app
            .controller
            .inspection_setup()
            .tools
            .iter()
            .position(|t| t.number == number)
            .unwrap();
        app.controller.edit_tool(index, |tool| tool.diameter = 6.);
        app.begin_operation_draft(id);
        let _ = app.update(Message::ProfilePassToggle(
            profile_parameters::Toggle::Finishing,
            true,
        ));
        let _ = app.update(Message::ContourRoughingToggled(true));
        (app, id)
    }
    fn enter(app: &mut App, field: Field, source: &str) {
        let _ = app.update(Message::FieldChanged(field, source.into()));
    }
    fn formulas(app: &mut App) {
        enter(
            app,
            MAXIMUM,
            &format!(
                "={}",
                OperationParameter::MaximumStepover.default_expression()
            ),
        );
        enter(
            app,
            FINISH,
            &format!(
                "={}",
                OperationParameter::FinishingStepover.default_expression()
            ),
        );
    }
    #[test]
    fn formulas_apply_reopen_override_and_undo() {
        let (mut app, id) = editor();
        formulas(&mut app);
        assert!((app.parsed_field_value(MAXIMUM).unwrap() - 5.7).abs() < 1e-12);
        assert!((app.parsed_field_value(FINISH).unwrap() - 0.6).abs() < 1e-12);
        assert!(
            !app.any_field_invalid(),
            "{:?}",
            app.profile_parameter_error()
        );
        let _ = app.update(Message::Apply);
        let saved = app.controller.operation(id).unwrap().clone();
        assert_eq!(
            OperationParameter::FinishingStepover.expression(&saved),
            Some(OperationParameter::FinishingStepover.default_expression()),
            "{}",
            app.status
        );
        app.begin_operation_draft(id);
        assert!(!app.inspector_dirty());
        assert_eq!(
            app.fields[&FINISH],
            format!(
                "={}",
                OperationParameter::FinishingStepover.default_expression()
            )
        );
        enter(&mut app, MAXIMUM, "0.2");
        assert_eq!(app.parsed_field_value(FINISH).unwrap(), 0.2);
        enter(&mut app, FINISH, "0.15");
        let _ = app.update(Message::Apply);
        let overridden = app.controller.operation(id).unwrap();
        assert_eq!(
            OperationParameter::FinishingStepover.expression(overridden),
            None
        );
        assert_eq!(
            OperationParameter::FinishingStepover.value(overridden),
            Some(0.15)
        );
        assert!(app.controller.undo());
        assert_eq!(app.controller.operation(id).unwrap(), &saved);
        assert!(app.controller.redo());
        assert_eq!(
            OperationParameter::FinishingStepover.value(app.controller.operation(id).unwrap()),
            Some(0.15)
        );
    }
    #[test]
    fn formulas_survive_collapsed_sections_and_cancel() {
        let (mut app, id) = editor();
        let before = app.controller.operation(id).unwrap().clone();
        formulas(&mut app);
        let _ = app.update(Message::ProfilePassToggle(
            profile_parameters::Toggle::Finishing,
            false,
        ));
        let _ = app.update(Message::ContourRoughingToggled(false));
        let _ = app.update(Message::ProfilePassToggle(
            profile_parameters::Toggle::Finishing,
            true,
        ));
        let _ = app.update(Message::ContourRoughingToggled(true));
        assert_eq!(
            app.fields[&FINISH],
            format!(
                "={}",
                OperationParameter::FinishingStepover.default_expression()
            )
        );
        assert_eq!(
            app.fields[&MAXIMUM],
            format!(
                "={}",
                OperationParameter::MaximumStepover.default_expression()
            )
        );
        assert!((app.parsed_field_value(FINISH).unwrap() - 0.6).abs() < 1e-12);
        let _ = app.update(Message::CancelOperationDraft);
        assert_eq!(app.controller.operation(id).unwrap(), &before);
    }
    #[test]
    fn invalid_expressions_block_apply_and_can_be_overridden() {
        let (mut app, id) = editor();
        let before = app.controller.operation(id).unwrap().clone();
        formulas(&mut app);
        for source in [
            "=tool_diameter / 0",
            "=unknown_parameter",
            "=finishingStepover",
            "tool_diameter * 0.1",
            "-1",
        ] {
            enter(&mut app, MAXIMUM, source);
            assert!(app.any_field_invalid(), "{source}");
            let _ = app.update(Message::Apply);
            assert_eq!(app.controller.operation(id).unwrap(), &before);
        }
        enter(&mut app, MAXIMUM, "0.4");
        assert_eq!(app.parsed_field_value(FINISH).unwrap(), 0.4);
        assert!(!app.any_field_invalid());
    }
    #[test]
    fn tool_changes_recalculate_formulas_and_keep_literal_overrides() {
        let (mut app, _) = editor();
        formulas(&mut app);
        let number = app.operation_draft.as_ref().unwrap().tool();
        let tool = app
            .controller
            .inspection_setup()
            .tools
            .iter()
            .find(|t| t.number == number)
            .unwrap();
        let mut tool = tool.clone();
        tool.diameter = 10.;
        app.operation_tools.push(tool);
        app.refresh_operation_fields();
        assert!((app.parsed_field_value(MAXIMUM).unwrap() - 9.5).abs() < 1e-12);
        assert_eq!(app.parsed_field_value(FINISH).unwrap(), 1.);
        enter(&mut app, FINISH, "0.25");
        app.operation_tools[0].diameter = 20.;
        assert_eq!(app.parsed_field_value(FINISH).unwrap(), 0.25);
    }
    #[test]
    fn new_operation_accepts_formulas_before_commit() {
        let (mut app, _) = editor();
        let _ = app.update(Message::CancelOperationDraft);
        let _ = app.update(Message::BeginOp(OpKind::Profile));
        assert!(app.creating_operation());
        app.controller
            .set_pending_operation_geometry(&[crate::controller::LoopRef::outer(0)], None)
            .unwrap();
        let tool = cam_model::Tool {
            number: 1,
            diameter: 6.,
            flute_length: 20.,
            ..Default::default()
        };
        let _ = app.update(Message::OperationProjectToolPicked(tool));
        let _ = app.update(Message::ProfilePassToggle(
            profile_parameters::Toggle::Finishing,
            true,
        ));
        let _ = app.update(Message::ContourRoughingToggled(true));
        formulas(&mut app);
        let (op, _) = app
            .creation_operation()
            .expect("formulas must pass creation validation");
        assert_eq!(
            OperationParameter::FinishingStepover.expression(&op),
            Some(OperationParameter::FinishingStepover.default_expression())
        );
        assert!((OperationParameter::FinishingStepover.value(&op).unwrap() - 0.6).abs() < 1e-12);
    }

    #[test]
    fn editing_reveals_source_and_finishing_shows_result_without_applying() {
        let (mut app, id) = editor();
        let before = app.controller.operation(id).unwrap().clone();
        app.parameter_input_state(FINISH, true, 0);
        enter(&mut app, FINISH, "=tool_diameter * 0.1");
        assert_eq!(
            app.parameter_display_value(FINISH, &app.fields[&FINISH]),
            "=tool_diameter * 0.1"
        );
        assert_eq!(
            OperationParameter::FinishingStepover.expression(app.current_operation(id).unwrap()),
            None
        );
        app.parameter_input_state(FINISH, false, 0);
        assert_eq!(
            app.parameter_display_value(FINISH, &app.fields[&FINISH]),
            "0.60"
        );
        assert_eq!(
            OperationParameter::FinishingStepover.expression(app.current_operation(id).unwrap()),
            Some("tool_diameter * 0.1")
        );
        assert_eq!(
            app.controller.operation(id).unwrap(),
            &before,
            "finishing a field must not apply the operation"
        );
        app.parameter_input_state(FINISH, true, 0);
        assert_eq!(
            app.parameter_display_value(FINISH, &app.fields[&FINISH]),
            "=tool_diameter * 0.1"
        );
        enter(&mut app, FINISH, "0.25");
        app.parameter_input_state(FINISH, false, 0);
        assert_eq!(
            OperationParameter::FinishingStepover.expression(app.current_operation(id).unwrap()),
            None
        );
        assert_eq!(app.parsed_field_value(FINISH).unwrap(), 0.25);
    }
    #[test]
    fn suggestions_filter_at_caret_and_replace_only_the_current_identifier() {
        let (mut app, _) = editor();
        enter(&mut app, FINISH, "=Math.min(tool_di, maximumStepover)");
        let caret = "=Math.min(tool_di".len();
        app.parameter_input_state(FINISH, true, caret);
        assert_eq!(app.parameter_suggestions(FINISH), vec!["tool_diameter"]);
        let _ = app.complete_parameter(FINISH, "tool_diameter", caret);
        assert_eq!(
            app.fields[&FINISH],
            "=Math.min(tool_diameter, maximumStepover)"
        );
        assert!(app.parameter_suggestions(FINISH).is_empty());
        enter(&mut app, FINISH, "=tool_");
        app.parameter_input_state(FINISH, true, 6);
        assert_eq!(
            app.parameter_suggestions(FINISH),
            vec![
                "tool_diameter",
                "tool_cornerRadius",
                "tool_length",
                "tool_fluteLength",
                "tool_flutes"
            ]
        );
        app.move_parameter_suggestion(FINISH, 1);
        assert_eq!(app.parameter_editor.suggestion, 1);
        enter(&mut app, FINISH, "=tool_c");
        app.parameter_input_state(FINISH, true, 7);
        assert_eq!(app.parameter_suggestions(FINISH), vec!["tool_cornerRadius"]);
        assert_eq!(app.parameter_editor.suggestion, 0);
        enter(&mut app, FINISH, "=max");
        app.parameter_input_state(FINISH, true, 4);
        assert_eq!(app.parameter_suggestions(FINISH), vec!["maximumStepover"]);
        assert!(!app.variable_names(FINISH).contains(&"finishingStepover"));
        enter(&mut app, FINISH, "tool_");
        assert!(app.parameter_suggestions(FINISH).is_empty());
    }
    #[test]
    fn mouse_completion_survives_blur_and_invalid_text_stays_editable() {
        let (mut app, id) = editor();
        enter(&mut app, FINISH, "=tool_di * 0.1");
        app.parameter_input_state(FINISH, true, 8);
        app.parameter_input_state(FINISH, false, 8);
        assert!(app.parameter_editor.focused.is_none());
        assert_eq!(
            app.parameter_display_value(FINISH, &app.fields[&FINISH]),
            "=tool_di * 0.1"
        );
        let _ = app.complete_parameter(FINISH, "tool_diameter", 8);
        assert_eq!(app.fields[&FINISH], "=tool_diameter * 0.1");
        assert_eq!(app.parameter_editor.focused, Some(FINISH));
        app.parameter_input_state(FINISH, false, 0);
        assert_eq!(
            app.parameter_display_value(FINISH, &app.fields[&FINISH]),
            "0.60"
        );
        enter(&mut app, FINISH, "tool_diameter * 0.1");
        assert!(app
            .parsed_field_value(FINISH)
            .unwrap_err()
            .contains("Start a formula"));
        let _ = app.update(Message::ProfilePassToggle(
            profile_parameters::Toggle::Finishing,
            false,
        ));
        assert_eq!(app.fields[&FINISH], "tool_diameter * 0.1");
        assert!(app.any_field_invalid());
        let _ = app.update(Message::ProfilePassToggle(
            profile_parameters::Toggle::Finishing,
            true,
        ));
        assert_eq!(app.fields[&FINISH], "tool_diameter * 0.1");
        assert_eq!(
            OperationParameter::FinishingStepover.expression(app.current_operation(id).unwrap()),
            Some("tool_diameter * 0.1")
        );
    }
    #[test]
    fn constant_formulas_remain_formulas_until_replaced_with_a_number() {
        let (mut app, id) = editor();
        enter(&mut app, FINISH, "=0.4");
        let _ = app.update(Message::Apply);
        let saved = app.controller.operation(id).unwrap();
        assert_eq!(
            OperationParameter::FinishingStepover.expression(saved),
            Some("0.4")
        );
        app.begin_operation_draft(id);
        assert_eq!(app.fields[&FINISH], "=0.4");
        assert!(!app.inspector_dirty());
        assert_eq!(
            app.parameter_display_value(FINISH, &app.fields[&FINISH]),
            "0.40"
        );
    }

    #[test]
    fn display_precision_never_rounds_sources_calculations_or_saved_values() {
        let (mut app, id) = editor();
        enter(&mut app, Field::Stepdown, "=tool_diameter / 7");
        enter(&mut app, FINISH, "=stepdown / 13");
        let exact_step = 6. / 7.;
        let exact_finish = exact_step / 13.;
        for decimals in [0, 2, 5, 8] {
            app.settings.number_decimals = decimals;
            assert_eq!(
                app.parameter_display_value(FINISH, &app.fields[&FINISH]),
                app.settings.format_number(exact_finish)
            );
            assert_eq!(
                app.parsed_field_value(Field::Stepdown).unwrap().to_bits(),
                exact_step.to_bits()
            );
            assert_eq!(
                app.parsed_field_value(FINISH).unwrap().to_bits(),
                exact_finish.to_bits()
            );
            app.parameter_input_state(FINISH, true, 0);
            assert_eq!(
                app.parameter_display_value(FINISH, &app.fields[&FINISH]),
                "=stepdown / 13"
            );
            app.parameter_input_state(FINISH, false, 0);
        }
        let _ = app.update(Message::Apply);
        let saved = app.controller.operation(id).unwrap();
        assert_eq!(
            op_field(saved, FINISH).unwrap().to_bits(),
            exact_finish.to_bits(),
            "{}",
            app.status
        );
        assert_eq!(
            op_field(saved, Field::Stepdown).unwrap().to_bits(),
            exact_step.to_bits()
        );
        assert_eq!(app.fields[&FINISH], "=stepdown / 13");
        app.begin_operation_draft(id);
        enter(&mut app, FINISH, "0.123456789012345");
        app.settings.number_decimals = 2;
        assert_eq!(
            app.parameter_display_value(FINISH, &app.fields[&FINISH]),
            "0.12"
        );
        app.parameter_input_state(FINISH, true, 0);
        assert_eq!(
            app.parameter_display_value(FINISH, &app.fields[&FINISH]),
            "0.123456789012345"
        );
        app.parameter_input_state(FINISH, false, 0);
        let _ = app.update(Message::Apply);
        assert_eq!(
            op_field(app.controller.operation(id).unwrap(), FINISH)
                .unwrap()
                .to_bits(),
            0.123456789012345_f64.to_bits()
        );
    }
    #[test]
    fn integer_formulas_keep_existing_bounds_and_numeric_overrides() {
        let (mut app, id) = editor();
        let count = Field::ContourRoughCount;
        enter(&mut app, count, "=Math.ceil(tool_diameter / 4)");
        assert_eq!(app.parsed_field_value(count).unwrap(), 2.);
        assert_eq!(app.parameter_display_value(count, &app.fields[&count]), "2");
        let _ = app.update(Message::Apply);
        assert_eq!(
            op_field(app.controller.operation(id).unwrap(), count),
            Some(2.)
        );
        app.begin_operation_draft(id);
        let before = app.controller.operation(id).unwrap().clone();
        enter(&mut app, count, "=tool_diameter / 4");
        assert!(app.any_field_invalid());
        let _ = app.update(Message::Apply);
        assert_eq!(app.controller.operation(id).unwrap(), &before);
        enter(&mut app, count, "3");
        let _ = app.update(Message::Apply);
        assert_eq!(
            op_field(app.controller.operation(id).unwrap(), count),
            Some(3.)
        );
        assert_eq!(
            OperationParameter::ContourRoughCount.expression(app.controller.operation(id).unwrap()),
            None
        );
    }
    #[test]
    fn cutting_speed_and_tooth_formulas_drive_full_precision_rpm_and_feed() {
        let (mut app, id) = editor();
        enter(&mut app, Field::CuttingSpeed, "=1000 / 7");
        enter(&mut app, Field::FeedPerTooth, "=1 / 73");
        let tool = app
            .parameter_tool(app.current_operation(id).unwrap())
            .unwrap();
        let expected_rpm = (1000. / 7.) * 1000. / (std::f64::consts::PI * tool.diameter);
        let expected_feed = (1. / 73.) * tool.flutes as f64 * expected_rpm;
        assert!(expected_feed > 0.);
        assert_eq!(
            app.parsed_field_value(Field::SpindleRpm).unwrap().to_bits(),
            expected_rpm.to_bits()
        );
        assert_eq!(
            app.parsed_field_value(Field::Feed).unwrap().to_bits(),
            expected_feed.to_bits()
        );
        let _ = app.update(Message::Apply);
        let saved = app.controller.operation(id).unwrap();
        assert_eq!(
            saved.spindle_rpm().to_bits(),
            expected_rpm.to_bits(),
            "{}",
            app.status
        );
        assert_eq!(
            op_field(saved, Field::Feed).unwrap().to_bits(),
            expected_feed.to_bits()
        );
        app.begin_operation_draft(id);
        enter(&mut app, Field::Feed, "123.456789012345");
        let _ = app.update(Message::Apply);
        let saved = app.controller.operation(id).unwrap();
        assert_eq!(OperationParameter::FeedPerTooth.expression(saved), None);
        assert_eq!(
            op_field(saved, Field::Feed).unwrap().to_bits(),
            123.456789012345_f64.to_bits()
        );
    }
    #[test]
    fn model_parameters_round_trip_through_every_operation_adapter() {
        let (mut app, _) = editor();
        for kind in [
            OpKind::Profile,
            OpKind::Pocket,
            OpKind::Face,
            OpKind::Drill,
            OpKind::Engrave,
            OpKind::Carve,
            OpKind::Chamfer,
            OpKind::Bore,
            OpKind::Thread,
            OpKind::Adaptive,
            OpKind::Parallel,
            OpKind::Spiral,
        ] {
            let operation = app.controller.operation_template(kind, 1);
            for parameter in OperationParameter::ALL {
                let Some(value) = parameter.value(&operation) else {
                    continue;
                };
                if parameter.validate_value(&operation, value).is_err()
                    || matches!(
                        parameter,
                        OperationParameter::CuttingSpeed | OperationParameter::FeedPerTooth
                    )
                {
                    continue;
                }
                let mut candidate = operation.clone();
                parameter.set_source(&mut candidate, &format!("={value}"));
                candidate
                    .resolve_parameter_expressions(&app.controller.inspection_setup().tools)
                    .unwrap_or_else(|e| panic!("{kind:?} {parameter:?}: {e}"));
                assert_eq!(
                    parameter.value(&candidate).unwrap().to_bits(),
                    value.to_bits(),
                    "{kind:?} {parameter:?}"
                );
                let json = serde_json::to_string(&candidate).unwrap();
                assert_eq!(serde_json::from_str::<Operation>(&json).unwrap(), candidate);
            }
            app.operation_draft = Some(operation.clone());
            for field in operation_fields(&operation) {
                if let Some(parameter) = parameter(field) {
                    assert_eq!(
                        parameter.value(&operation),
                        op_field(&operation, field),
                        "{kind:?} {field:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn incomplete_formula_does_not_invalidate_unrelated_fields() {
        let (mut app, id) = editor();
        let before = app.controller.operation(id).unwrap().clone();
        for source in ["=", "=tool_", "=1 / 0", "unfinished", ""] {
            enter(&mut app, Field::RampFeed, source);
            app.parameter_input_state(Field::RampFeed, true, source.len());
            assert!(app.parameter_display_result(Field::Feed).is_ok());
            app.parameter_input_state(Field::RampFeed, false, source.len());
            assert!(
                app.parameter_display_result(Field::RampFeed).is_err(),
                "{source}"
            );
            for field in [
                Field::Feed,
                Field::PlungeFeed,
                Field::LeadInFeed,
                Field::LeadOutFeed,
                FINISH,
                MAXIMUM,
            ] {
                assert!(
                    app.parameter_display_result(field).is_ok(),
                    "{source}: {field:?}"
                );
                assert!(!app.field_invalid(field), "{source}: {field:?}");
            }
            assert!(app.any_field_invalid());
            let _ = app.update(Message::Apply);
            assert_eq!(app.controller.operation(id).unwrap(), &before);
        }
        enter(&mut app, Field::RampFeed, "=tool_diameter * 5");
        app.parameter_input_state(Field::RampFeed, true, 0);
        app.parameter_input_state(Field::RampFeed, false, 0);
        assert_eq!(app.parameter_display_result(Field::RampFeed).unwrap(), 30.);
        assert!(!app.any_field_invalid());
    }
    #[test]
    fn actual_dependents_report_errors_without_contaminating_other_parameters() {
        let (mut app, _) = editor();
        enter(&mut app, Field::RampFeed, "=1 / 0");
        enter(&mut app, Field::LeadInFeed, "=rampFeed * 2");
        enter(&mut app, FINISH, "=maximumStepover / 7");
        assert!(app.parsed_field_value(Field::RampFeed).is_err());
        assert!(app
            .parsed_field_value(Field::LeadInFeed)
            .unwrap_err()
            .contains("rampFeed"));
        assert!(app.parsed_field_value(Field::Feed).is_ok());
        assert!(app.parsed_field_value(Field::PlungeFeed).is_ok());
        assert_eq!(
            app.parsed_field_value(FINISH).unwrap(),
            app.parsed_field_value(MAXIMUM).unwrap() / 7.
        );
        // A dependency cycle is isolated in the same way.
        enter(&mut app, Field::RampFeed, "=leadInFeed");
        assert!(app
            .parsed_field_value(Field::RampFeed)
            .unwrap_err()
            .contains("Circular"));
        assert!(app.parsed_field_value(Field::LeadOutFeed).is_ok());
    }
    #[test]
    #[ignore = "requires a graphics adapter; verifies focus survives numeric/formula layout transitions"]
    fn formula_row_keeps_focus_when_equals_is_typed() {
        use iced::advanced::renderer::Headless;
        use iced::advanced::{layout, mouse, widget, Layout, Shell};
        let (mut app, _) = editor();
        let renderer = iced::futures::executor::block_on(<iced::Renderer as Headless>::new(
            iced::Font::DEFAULT,
            iced::Pixels(12.),
            None,
        ))
        .expect("graphics adapter");
        let size = iced::Size::new(380., 300.);
        let mut tree = widget::Tree::empty();
        fn keyboard(text: &str) -> iced::Event {
            iced::Event::Keyboard(iced::keyboard::Event::KeyPressed {
                key: iced::keyboard::Key::Character(text.into()),
                modified_key: iced::keyboard::Key::Character(text.into()),
                physical_key: iced::keyboard::key::Physical::Code(iced::keyboard::key::Code::KeyA),
                location: iced::keyboard::Location::Standard,
                modifiers: iced::keyboard::Modifiers::default(),
                text: Some(text.into()),
                repeat: false,
            })
        }
        let click = iced::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left));
        let redraw = iced::Event::Window(iced::window::Event::RedrawRequested(
            std::time::Instant::now(),
        ));
        for (event, cursor) in [
            (click, mouse::Cursor::Available(iced::Point::new(340., 14.))),
            (keyboard("="), mouse::Cursor::Unavailable),
            (redraw.clone(), mouse::Cursor::Unavailable),
            (keyboard("t"), mouse::Cursor::Unavailable),
            (redraw, mouse::Cursor::Unavailable),
        ] {
            let source = app.fields[&Field::RampFeed].clone();
            let mut element = app.parameter_field_row(
                Field::RampFeed,
                "Ramp feed (0=cutting)",
                "Ramp feed",
                &source,
            );
            tree.diff(&element);
            let node = element.as_widget_mut().layout(
                &mut tree,
                &renderer,
                &layout::Limits::new(iced::Size::ZERO, size),
            );
            let mut messages = Vec::new();
            element.as_widget_mut().update(
                &mut tree,
                &event,
                Layout::new(&node),
                cursor,
                &renderer,
                &mut iced::advanced::clipboard::Null,
                &mut Shell::new(&mut messages),
                &iced::Rectangle::with_size(size),
            );
            for message in messages {
                let _ = app.update(message);
            }
            assert_eq!(app.parameter_editor.focused, Some(Field::RampFeed));
        }
        assert_eq!(app.fields[&Field::RampFeed], "=t");
        assert!(app
            .parameter_suggestions(Field::RampFeed)
            .contains(&"tool_diameter"));
        assert!(app.parameter_display_result(Field::Feed).is_ok());
    }

    #[test]
    #[ignore = "requires a graphics adapter; exports formula controls to /tmp"]
    fn render_parameter_expressions() {
        use iced::advanced::renderer::Headless;
        let (mut app, _) = editor();
        formulas(&mut app);
        let size = iced::Size::new(380., 440.);
        let mut renderer = iced::futures::executor::block_on(<iced::Renderer as Headless>::new(
            iced::Font::DEFAULT,
            iced::Pixels(12.),
            None,
        ))
        .expect("graphics adapter");
        app.parameter_input_state(MAXIMUM, true, 0);
        app.parameter_input_state(MAXIMUM, false, 0);
        for mode in ["values", "editing", "suggestions"] {
            if mode == "editing" {
                app.parameter_input_state(FINISH, true, app.fields[&FINISH].len());
            } else if mode == "suggestions" {
                enter(&mut app, FINISH, "=Math.min(tool_, maximumStepover)");
                app.parameter_input_state(FINISH, true, "=Math.min(tool_".len());
            }
            let mut element: Element<'_, Message> = container(
                iced::widget::column![
                    app.parameter_field_row(
                        FINISH,
                        "Finish spacing (mm)",
                        "Finishing stepover",
                        &app.fields[&FINISH]
                    ),
                    app.parameter_field_row(
                        MAXIMUM,
                        "Maximum stepover (mm)",
                        "Roughing stepover",
                        &app.fields[&MAXIMUM]
                    ),
                ]
                .spacing(20),
            )
            .padding(16)
            .width(Length::Fill)
            .style(|_| container::Style {
                background: Some(Color::from_rgb(0.13, 0.14, 0.16).into()),
                ..Default::default()
            })
            .into();
            let mut tree = iced::advanced::widget::Tree::new(&element);
            let node = element.as_widget_mut().layout(
                &mut tree,
                &renderer,
                &iced::advanced::layout::Limits::new(iced::Size::ZERO, size),
            );
            let mut messages = Vec::new();
            element.as_widget_mut().update(
                &mut tree,
                &iced::Event::Window(iced::window::Event::RedrawRequested(
                    std::time::Instant::now(),
                )),
                iced::advanced::Layout::new(&node),
                iced::mouse::Cursor::Unavailable,
                &renderer,
                &mut iced::advanced::clipboard::Null,
                &mut iced::advanced::Shell::new(&mut messages),
                &iced::Rectangle::with_size(size),
            );
            element.as_widget().draw(
                &tree,
                &mut renderer,
                &iced::Theme::Dark,
                &iced::advanced::renderer::Style {
                    text_color: iced::Theme::Dark.palette().text,
                },
                iced::advanced::Layout::new(&node),
                iced::mouse::Cursor::Unavailable,
                &iced::Rectangle::with_size(size),
            );
            let pixels =
                Headless::screenshot(&mut renderer, iced::Size::new(380, 440), 1., Color::WHITE);
            let mut ppm = b"P6\n380 440\n255\n".to_vec();
            for pixel in pixels.as_chunks::<4>().0 {
                ppm.extend_from_slice(&pixel[..3]);
            }
            std::fs::write(
                format!("/tmp/opencam-parameter-expressions-{mode}.ppm"),
                ppm,
            )
            .unwrap();
        }
    }
}
