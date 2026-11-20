//! Context menu for programmed and per-user defaults for numeric operation parameters.
use super::*;
use parameter_expressions::parameter;
use parameter_options::OptionMenu;

#[derive(Clone, Copy, Debug)]
pub(super) enum Action {
    Save,
    ResetUser,
    ResetSystem,
    CopyName,
}

pub(super) enum Target {
    Numeric(Field),
    Option(OptionMenu),
}

pub(super) fn menu_button(open: Message) -> iced::widget::Button<'static, Message> {
    inspector_button("⋮").padding([2, 6]).on_press(open)
}

fn operation_key(op: &Operation) -> &'static str {
    // Parallel and Scallop are modes of the same operation.
    if matches!(op, Operation::Parallel(_)) {
        "3D Parallel"
    } else {
        op_kind(op)
    }
}

impl App {
    fn parameter_default_key(&self, field: Field) -> Option<String> {
        let op = self.parameter_operation()?;
        let parameter = parameter(field)?;
        parameter.value(op)?;
        let mut name = parameter.name().to_owned();
        // The same parameter slots carry different dimensions for ramp, helix and zig-zag.
        if let Some((plunge, _, _)) = self.plunge_param(field) {
            name = format!("{name}.{:?}", PlungeKind::of(plunge));
        }
        Some(name)
    }

    pub(super) fn user_default_source(&self, op: &Operation, name: &str) -> Option<&str> {
        self.settings
            .parameter_defaults
            .get(operation_key(op))?
            .get(name)
            .map(String::as_str)
    }

    pub(super) fn user_option_default_source(&self, op: &Operation, name: &str) -> Option<&str> {
        self.user_default_source(op, &format!("option.{name}"))
    }

    fn user_parameter_default(&self, field: Field) -> Option<&str> {
        self.user_default_source(
            self.parameter_operation()?,
            &self.parameter_default_key(field)?,
        )
    }

    pub(super) fn seed_user_parameter_defaults(&mut self, include: impl Fn(Field) -> bool) {
        if !self.creating_operation() || self.settings.parameter_defaults.is_empty() {
            return;
        }
        let defaults: Vec<_> = self
            .inspector_fields()
            .into_iter()
            .filter(|field| include(*field) && !self.creation_edited_fields.contains(field))
            .filter_map(|field| Some((field, self.user_parameter_default(field)?.to_owned())))
            .collect();
        for (field, source) in defaults {
            self.fields.insert(field, source);
            self.edit_cutting_field(field);
        }
    }

    fn system_parameter_default(&self, field: Field) -> Option<String> {
        let op = self.parameter_operation()?;
        let parameter = parameter(field)?;
        if let Some((plunge, first, _)) = self.plunge_param(field) {
            let (a, b) = plunge_params(PlungeKind::of(plunge).to_plunge());
            return Some(fmt_num(if first { a } else { b }));
        }
        let kind = AppController::operation_kind(op);
        let mut template = self.controller.system_operation_template(kind);
        Self::seed_programmed_parameter_defaults(&mut template);
        if is_clear_field(field) {
            if let Operation::Carve(carve) = &mut template {
                carve.clear = Some(CarveClearing {
                    tool: op.tool(),
                    params: ClearParams {
                        feed: carve.feed,
                        plunge_feed: carve.plunge_feed,
                        ..ClearParams::default()
                    },
                });
            }
        }
        if let Some(source) = parameter.expression(&template) {
            return Some(format!("={source}"));
        }
        let value = match field {
            Field::CuttingSpeed | Field::FeedPerTooth => {
                self.operation_cutting_value(&template, field)
            }
            Field::BoreDiameter => Some(cam_model::BoreOp::default().diameter),
            // The selected geometry or thread standard supplies the major diameter.
            Field::MajorDia => None,
            Field::Profile(profile_parameters::Number::TabCount) => {
                // Enabling a count starts with one tab. The template uses spacing.
                Some(1.)
            }
            _ => op_field(&template, field),
        }?;
        (parameter.validate_value(op, value).is_ok()
            && !operation_tabs::operation_field_invalid(op, field, value))
        .then(|| fmt_num(value))
    }

    pub(super) fn open_parameter_menu(&mut self, field: Field) -> iced::Task<Message> {
        if self.parameter_default_key(field).is_none() || !self.fields.contains_key(&field) {
            return iced::Task::none();
        }
        self.open_default_menu(Target::Numeric(field))
    }

    pub(super) fn open_parameter_option_menu(&mut self, option: OptionMenu) -> iced::Task<Message> {
        if self.parameter_operation().is_none() {
            return iced::Task::none();
        }
        self.open_default_menu(Target::Option(option))
    }

    fn open_default_menu(&mut self, target: Target) -> iced::Task<Message> {
        if let Some(focused) = self.parameter_editor.focused {
            self.finish_parameter_edit(focused);
        }
        self.parameter_menu = Some((target, self.window_cursor));
        iced::advanced::widget::operate(iced::advanced::widget::operation::focusable::unfocus())
    }

    pub(super) fn parameter_menu_action(&mut self, action: Action) -> iced::Task<Message> {
        let Some((target, _)) = self.parameter_menu.take() else {
            return iced::Task::none();
        };
        let field = match target {
            Target::Numeric(field) => field,
            Target::Option(option) => return self.parameter_option_action(option, action),
        };
        match action {
            Action::Save => self.save_parameter_default(field, Self::save_preferences),
            Action::ResetUser | Action::ResetSystem => {
                let source = if matches!(action, Action::ResetUser) {
                    self.user_parameter_default(field).map(str::to_owned)
                } else {
                    self.system_parameter_default(field)
                };
                if let Some(source) = source {
                    return self.update_message(Message::FieldChanged(field, source));
                }
            }
            Action::CopyName => {
                if let Some(parameter) = parameter(field) {
                    return iced::clipboard::write(parameter.name().to_owned());
                }
            }
        }
        iced::Task::none()
    }

    pub(super) fn save_option_default(
        &mut self,
        option: &OptionMenu,
        save: impl FnOnce(&mut Self, crate::Settings) -> std::io::Result<()>,
    ) {
        if let Some(op) = self
            .parameter_operation()
            .filter(|_| option.allows(&option.source))
        {
            self.save_default_source(
                operation_key(op),
                format!("option.{}", option.spec.name),
                option.source.clone(),
                save,
            );
        }
    }

    fn option_reset(&self, option: &OptionMenu, action: Action) -> Option<Message> {
        let op = self.parameter_operation()?;
        let source = match action {
            Action::ResetUser => self
                .user_option_default_source(op, option.spec.name)?
                .to_owned(),
            Action::ResetSystem => {
                let mut template = self
                    .controller
                    .system_operation_template(AppController::operation_kind(op));
                Self::seed_programmed_parameter_defaults(&mut template);
                option.seed_system_context(op, &mut template);
                (option.spec.read)(&template)?
            }
            _ => return None,
        };
        let message = (option.spec.change)(&source)?;
        (option.allows(&source) && self.option_default_allowed(&message)).then_some(message)
    }

    fn parameter_option_action(
        &mut self,
        option: OptionMenu,
        action: Action,
    ) -> iced::Task<Message> {
        match action {
            Action::Save => self.save_option_default(&option, Self::save_preferences),
            Action::ResetUser | Action::ResetSystem => {
                if let Some(message) = self.option_reset(&option, action) {
                    return self.update_message(message);
                }
            }
            Action::CopyName => return iced::clipboard::write(option.spec.name.to_owned()),
        }
        iced::Task::none()
    }

    fn save_parameter_default(
        &mut self,
        field: Field,
        save: impl FnOnce(&mut Self, crate::Settings) -> std::io::Result<()>,
    ) {
        let Some(op) = self.parameter_operation() else {
            return;
        };
        let Some(name) = self.parameter_default_key(field) else {
            return;
        };
        let Some(source) = self.fields.get(&field) else {
            return;
        };
        if self.field_invalid(field) || self.parameter_display_result(field).is_err() {
            return;
        }
        self.save_default_source(operation_key(op), name, source.trim().to_owned(), save);
    }

    fn save_default_source(
        &mut self,
        kind: &'static str,
        name: String,
        source: String,
        save: impl FnOnce(&mut Self, crate::Settings) -> std::io::Result<()>,
    ) {
        let mut settings = self.settings.clone();
        settings
            .parameter_defaults
            .entry(kind.into())
            .or_default()
            .insert(name, source);
        self.status = match save(self, settings) {
            Ok(()) => "Parameter saved as user default.".into(),
            Err(error) => format!("Could not save parameter default: {error}"),
        };
    }

    fn numeric_menu_preview(
        &self,
        field: Field,
    ) -> Option<(iced::widget::Column<'_, Message>, bool)> {
        let source = self.fields.get(&field)?.trim();
        let formula = source.strip_prefix('=');
        let value = self.parameter_display_result(field);
        let parameter = parameter(field)?;
        let label = self.field_label(field);
        let unit = label
            .rsplit_once(" (")
            .and_then(|(_, suffix)| suffix.strip_suffix(')'))
            .map(|suffix| suffix.split(',').next().unwrap_or(suffix))
            .filter(|suffix| {
                matches!(
                    *suffix,
                    "mm" | "mm/min" | "m/min" | "mm/tooth" | "RPM" | "°" | "%" | "s"
                )
            });
        let display = value.as_ref().map_or_else(
            |_| source.to_owned(),
            |value| {
                let number = if parameter.integer() {
                    fmt_num(*value)
                } else {
                    self.settings.format_number(*value)
                };
                match unit {
                    Some(unit) => format!("{number} {unit}"),
                    None => number,
                }
            },
        );
        let mut content = iced::widget::Column::new().spacing(2);
        if let Some(formula) = formula {
            content = content.push(
                container(
                    text(formula.trim())
                        .size(12)
                        .wrapping(iced::widget::text::Wrapping::None),
                )
                .padding([3, 10])
                .clip(true),
            );
        }
        if formula.is_none() || value.is_ok() {
            content = content.push(container(text(display).size(12)).padding([3, 10]));
        }
        Some((content, value.is_ok() && !self.field_invalid(field)))
    }

    pub(super) fn parameter_menu_overlay(&self) -> Option<Element<'_, Message>> {
        let (target, pos) = self.parameter_menu.as_ref()?;
        let (content, can_save, can_reset_user, can_reset_system) = match target {
            Target::Numeric(field) => {
                let (content, can_save) = self.numeric_menu_preview(*field)?;
                (
                    content,
                    can_save,
                    self.user_parameter_default(*field).is_some(),
                    self.system_parameter_default(*field).is_some(),
                )
            }
            Target::Option(option) => {
                let content = iced::widget::Column::new()
                    .spacing(2)
                    .push(container(text(&option.display).size(12)).padding([3, 10]));
                (
                    content,
                    option.allows(&option.source),
                    self.option_reset(option, Action::ResetUser).is_some(),
                    self.option_reset(option, Action::ResetSystem).is_some(),
                )
            }
        };
        let item = |label, action, enabled: bool| {
            button(text(label).size(13))
                .width(Length::Fill)
                .padding([6, 10])
                .on_press_maybe(enabled.then_some(Message::ParameterMenuAction(action)))
                .style(|_, status| command_button_style(status))
        };
        let content = content
            .push(iced::widget::rule::horizontal(1))
            .push(item("Save as User Default", Action::Save, can_save))
            .push(item(
                "Reset to User Default",
                Action::ResetUser,
                can_reset_user,
            ))
            .push(item(
                "Reset to System Default",
                Action::ResetSystem,
                can_reset_system,
            ))
            .push(item("Copy Parameter Name", Action::CopyName, true));
        let width = 225.;
        let popup = container(content)
            .width(width)
            .padding(4)
            .style(|_| container::Style {
                background: Some(palette::RIBBON_BG.into()),
                border: Border {
                    color: palette::BORDER_DARK,
                    width: 1.,
                    radius: 3.0.into(),
                },
                ..Default::default()
            });
        Some(
            container(iced::widget::opaque(popup))
                .width(Length::Fill)
                .height(Length::Fill)
                .padding(Padding {
                    top: pos.y.min((self.window.height - 190.).max(0.)),
                    left: pos.x.min((self.window.width - width).max(0.)),
                    right: 0.,
                    bottom: 0.,
                })
                .into(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::tests::Scratch;

    #[test]
    fn pocket_engagement_has_independent_system_and_user_resets() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "defaults.dxf").unwrap();
        app.controller.new_operation(OpKind::Pocket);
        let Selection::Operation(id) = app.controller.selection() else {
            panic!("pocket should be selected");
        };
        app.begin_operation_draft(id);
        let _ = app.update(Message::PocketAdaptiveToggled(true));
        let _ = app.update(Message::FieldChanged(Field::Engagement, "1.25".into()));
        let _ = app.update(Message::OpenParameterMenu(Field::Engagement));
        assert!(app.parameter_menu_overlay().is_some());
        let _ = app.update(Message::ParameterMenuAction(Action::ResetUser));
        assert_eq!(app.fields[&Field::Engagement], "1.25");

        let scratch = Scratch::new("system-parameter-default");
        let path = scratch.file("settings.json");
        let formula = "=tool_diameter * 0.25";
        let _ = app.update(Message::FieldChanged(Field::Engagement, formula.into()));
        app.save_parameter_default(Field::Engagement, |app, settings| {
            app.save_preferences_to(settings, &path)
        });
        let _ = app.update(Message::OpenParameterMenu(Field::Engagement));
        let _ = app.update(Message::ParameterMenuAction(Action::ResetSystem));

        assert!(app.parameter_menu.is_none());
        assert_eq!(
            app.fields[&Field::Engagement],
            creation::DEFAULT_POCKET_ENGAGEMENT
        );
        let diameter = app.operation_preset_tool().unwrap().diameter;
        let value = app.parsed_field_value(Field::Engagement).unwrap();
        assert!((value - diameter * 0.4).abs() < 1e-12);
        let _ = app.update(Message::Apply);
        let saved = app.controller.operation(id).unwrap();
        assert_eq!(
            parameter(Field::Engagement).unwrap().expression(saved),
            Some("tool_diameter * 0.4")
        );
        app.begin_operation_draft(id);
        let _ = app.update(Message::OpenParameterMenu(Field::Engagement));
        let _ = app.update(Message::ParameterMenuAction(Action::ResetUser));
        assert_eq!(app.fields[&Field::Engagement], formula);
    }

    #[test]
    fn user_defaults_round_trip_and_seed_only_the_matching_operation() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "defaults.dxf").unwrap();
        let _ = app.update(Message::BeginOp(OpKind::Pocket));
        let scratch = Scratch::new("parameter-defaults");
        let path = scratch.file("settings.json");
        let save = |app: &mut App, settings| app.save_preferences_to(settings, &path);
        let field = Field::Feed;
        let formula = "=tool_diameter * 123.456789";

        let _ = app.update(Message::FieldChanged(field, "=1 / 0".into()));
        app.save_parameter_default(field, save);
        assert!(!path.exists());
        let _ = app.update(Message::FieldChanged(field, formula.into()));
        app.save_parameter_default(field, save);

        let (settings, outcome) = crate::settings::load_from(&path);
        assert_eq!(outcome, crate::settings::LoadOutcome::Loaded);
        app.settings = settings;
        let _ = app.update(Message::FieldChanged(field, "200".into()));
        let _ = app.update(Message::OpenParameterMenu(field));
        let _ = app.update(Message::ParameterMenuAction(Action::ResetUser));
        assert_eq!(app.fields[&field], formula);
        let _ = app.update(Message::CancelOp);

        let _ = app.update(Message::BeginOp(OpKind::Profile));
        assert_ne!(app.fields[&field], formula);
        let _ = app.update(Message::CancelOp);
        let _ = app.update(Message::BeginOp(OpKind::Pocket));
        assert_eq!(app.fields[&field], formula);
        let diameter = app.wizard_tool().unwrap().diameter;
        assert!((app.wizard_cutting().unwrap().feed - diameter * 123.456789).abs() < 1e-12);
    }
}
