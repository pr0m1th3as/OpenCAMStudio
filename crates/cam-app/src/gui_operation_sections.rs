//! Shared sections for every operation tab. Disclosure never edits machining options.
use super::*;
use iced::widget::{column, Column};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) struct Section {
    pub tab: OperationTab,
    pub label: &'static str,
}
impl Section {
    pub const fn new(tab: OperationTab, label: &'static str) -> Self {
        Self { tab, label }
    }
}

struct Group<'a> {
    section: Option<Section>,
    toggle: Option<Element<'a, Message>>,
    controls: Vec<Element<'a, Message>>,
}

/// Collect controls by meaning even when a strategy supplies them in separate helpers.
pub(super) struct Form<'a> {
    app: &'a App,
    tab: OperationTab,
    groups: Vec<Group<'a>>,
    current: usize,
}
impl<'a> Form<'a> {
    pub fn new(app: &'a App, tab: OperationTab, label: Option<&'static str>) -> Self {
        Self {
            app,
            tab,
            groups: vec![Group {
                section: label.map(|label| Section::new(tab, label)),
                toggle: None,
                controls: Vec::new(),
            }],
            current: 0,
        }
    }

    pub fn section(mut self, label: &'static str) -> Self {
        let section = Some(Section::new(self.tab, label));
        self.current = self
            .groups
            .iter()
            .position(|group| group.section == section)
            .unwrap_or_else(|| {
                self.groups.push(Group {
                    section,
                    toggle: None,
                    controls: Vec::new(),
                });
                self.groups.len() - 1
            });
        self
    }

    pub fn optional_section(
        self,
        label: &'static str,
        enabled: bool,
        setting: Option<parameter_options::Setting<bool>>,
    ) -> Self {
        let mut form = self.section(label);
        form.groups[form.current].toggle = Some(
            checkbox(enabled)
                .size(14)
                .on_toggle_maybe(setting.map(|setting| setting.on_change()))
                .into(),
        );
        form
    }

    pub fn plain(mut self) -> Self {
        self.groups.push(Group {
            section: None,
            toggle: None,
            controls: Vec::new(),
        });
        self.current = self.groups.len() - 1;
        self
    }

    pub fn push(mut self, control: impl Into<Element<'a, Message>>) -> Self {
        self.groups[self.current].controls.push(control.into());
        self
    }

    pub fn extend(mut self, other: Self) -> Self {
        for mut incoming in other.groups {
            if let Some(existing) = incoming.section.and_then(|section| {
                self.groups
                    .iter_mut()
                    .find(|group| group.section == Some(section))
            }) {
                existing.controls.append(&mut incoming.controls);
                if incoming.toggle.is_some() {
                    existing.toggle = incoming.toggle;
                }
            } else {
                self.groups.push(incoming);
            }
        }
        self
    }

    pub fn finish(self) -> Element<'a, Message> {
        let mut form = Column::new().spacing(12);
        for group in self.groups {
            if group.controls.is_empty() && group.toggle.is_none() {
                continue;
            }
            let body = Column::with_children(group.controls).spacing(10);
            form = if let Some(section) = group.section {
                form.push(self.app.operation_section(section, body, group.toggle))
            } else {
                form.push(body)
            };
        }
        form.into()
    }
}

impl App {
    pub(super) fn operation_section<'a>(
        &self,
        section: Section,
        body: impl Into<Element<'a, Message>>,
        toggle: Option<Element<'a, Message>>,
    ) -> Element<'a, Message> {
        let collapsed = self.collapsed_operation_sections.contains(&section);
        let heading = inspector_disclosure_heading(
            section.label,
            collapsed,
            Message::OperationSectionToggled(section),
            toggle,
        );
        let heading = match section.label {
            "Cycle" => help_wrap(heading, drill_cycle::CYCLE_HELP, self.tooltips),
            "Holding tabs" => help_wrap(heading, illustrated_help::HOLDING_TABS, self.tooltips),
            "Finishing passes" => help_wrap(
                heading,
                if self.geometry_operation_kind() == Some(OpKind::Profile) {
                    profile_parameters::PROFILE_FINISHING_HELP
                } else {
                    profile_parameters::Toggle::Finishing.help()
                },
                self.tooltips,
            ),
            "Stock to Leave" => help_wrap(heading, illustrated_help::STOCK_TO_LEAVE, self.tooltips),
            "Clearing pass (end mill)" => help_wrap(heading, help::CARVE_CLEAR, self.tooltips),
            _ => heading,
        };
        let mut group = column![heading].spacing(10);
        if !collapsed {
            group = group.push(body);
        }
        group.into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collapsing_sections_preserves_invalid_input_and_optional_pass_settings() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample.dxf").unwrap();
        app.controller.new_operation(OpKind::Profile);
        let id = app
            .controller
            .inspection_setup()
            .operations
            .last()
            .unwrap()
            .id();
        app.begin_operation_draft(id);
        let saved = app.controller.document().clone();
        let _ = app.update(Message::FieldChanged(Field::Feed, "-".into()));
        let fields = app.fields.clone();
        let cutting_data = Section::new(OperationTab::Tool, "Cutting data");
        let _ = app.update(Message::OperationSectionToggled(cutting_data));
        assert!(
            app.any_field_invalid(),
            "folding must not bypass validation"
        );
        assert_eq!(app.fields, fields);
        let _ = app.update(Message::OperationSectionToggled(cutting_data));
        assert_eq!(app.fields, fields, "reopening must retain unfinished input");
        let _ = app.update(Message::FieldChanged(Field::Feed, "321".into()));
        let _ = app.update(Message::StockToLeaveToggled(true));
        let _ = app.update(Message::ProfilePassToggle(
            profile_parameters::Toggle::Finishing,
            true,
        ));
        let draft = app.operation_draft.clone();
        let stock = app.stock_to_leave_draft;
        let fields = app.fields.clone();
        for label in ["Stock to Leave", "Finishing passes", "Passes"] {
            let section = Section::new(OperationTab::Passes, label);
            let _ = app.update(Message::OperationSectionToggled(section));
            let _ = app.update(Message::OperationTabChanged(OperationTab::Passes));
            let _ = app.operation_inspector(id);
            let _ = app.update(Message::OperationSectionToggled(section));
        }
        assert_eq!(app.operation_draft, draft);
        assert_eq!(app.stock_to_leave_draft, stock);
        assert_eq!(app.fields, fields);
        assert_eq!(app.controller.document(), &saved);
    }
}
