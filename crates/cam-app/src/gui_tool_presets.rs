//! Shared material management and transactional cutting preset controls.
use super::*;
use cam_model::{Material, ToolPreset};
use iced::widget::column;

#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct MaterialChoice {
    pub id: Option<u64>,
    pub name: String,
}
impl std::fmt::Display for MaterialChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.name)
    }
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct PresetChoice {
    pub id: Option<u64>,
    pub name: String,
}
impl std::fmt::Display for PresetChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.name)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum PresetField {
    Rpm,
    CuttingSpeed,
    Feed,
    FeedPerTooth,
    Plunge,
    Ramp,
    LeadIn,
    LeadOut,
    Stepdown,
    Overlap,
}
impl PresetField {
    const ALL: [Self; 10] = [
        Self::Rpm,
        Self::CuttingSpeed,
        Self::Feed,
        Self::FeedPerTooth,
        Self::Plunge,
        Self::Ramp,
        Self::LeadIn,
        Self::LeadOut,
        Self::Stepdown,
        Self::Overlap,
    ];
    fn label(self) -> &'static str {
        match self {
            Self::Rpm => "Spindle (rpm)",
            Self::CuttingSpeed => "Cutting speed (m/min)",
            Self::Feed => "Cutting feed (mm/min)",
            Self::FeedPerTooth => "Feed per tooth (mm/tooth)",
            Self::Plunge => "Plunge feed (mm/min)",
            Self::Ramp => "Ramp feed (mm/min, optional)",
            Self::LeadIn => "Lead-in feed (mm/min, optional)",
            Self::LeadOut => "Lead-out feed (mm/min, optional)",
            Self::Stepdown => "Stepdown (mm, optional)",
            Self::Overlap => "Overlap (%, optional)",
        }
    }
    fn valid(self, value: &str) -> bool {
        if value.trim().is_empty() {
            return matches!(
                self,
                Self::Stepdown | Self::Overlap | Self::Ramp | Self::LeadIn | Self::LeadOut
            );
        }
        value.trim().parse::<f64>().ok().is_some_and(|v| {
            v.is_finite()
                && match self {
                    Self::Rpm => v >= 0.,
                    Self::Overlap => (0. ..=99.).contains(&v),
                    _ => v > 0.,
                }
        })
    }
}
#[derive(Clone, Copy, Debug)]
pub(super) enum ToolDetail {
    Description,
    Manufacturer,
    ProductId,
    ProductLink,
    CutterMaterial,
    Coating,
}

/// Preserve each cutter's pending edits while browsing another cutter.
#[derive(Clone, Debug)]
pub(super) struct LibraryToolDraft {
    pub tool: cam_model::Tool,
    fields: BTreeMap<Field, String>,
    preset: Option<u64>,
    preset_values: BTreeMap<PresetField, String>,
    preset_buffers: BTreeMap<u64, BTreeMap<PresetField, String>>,
}

impl App {
    pub(super) fn begin_preset_edit(&mut self) {
        if self.preset_editor_snapshot.is_none() {
            self.edited_preset_input = None;
            if let Some(tool) = &self.tool_edit {
                self.preset_editor_snapshot = Some(LibraryToolDraft {
                    tool: tool.clone(),
                    fields: self.fields.clone(),
                    preset: self.library_preset,
                    preset_values: self.preset_values.clone(),
                    preset_buffers: self.preset_buffers.clone(),
                });
            }
        }
        self.show_preset_editor = self.library_preset.is_some();
    }

    pub(super) fn cancel_preset_edit(&mut self) {
        if let Some(snapshot) = self.preset_editor_snapshot.take() {
            self.tool_edit = Some(snapshot.tool);
            self.fields = snapshot.fields;
            self.library_preset = snapshot.preset;
            self.preset_values = snapshot.preset_values;
            self.preset_buffers = snapshot.preset_buffers;
        }
        self.show_preset_editor = false;
        self.edited_preset_input = None;
    }

    pub(super) fn select_library_tool(&mut self, index: usize) {
        if index == self.lib_sel || index >= self.library.tools.len() {
            return;
        }
        if self.library_mode() && (self.inspector_dirty() || self.any_field_invalid()) {
            if let Some(tool) = &self.tool_edit {
                self.tool_drafts.insert(
                    self.lib_sel,
                    LibraryToolDraft {
                        tool: tool.clone(),
                        fields: self.fields.clone(),
                        preset: self.library_preset,
                        preset_values: self.preset_values.clone(),
                        preset_buffers: self.preset_buffers.clone(),
                    },
                );
            }
        }
        self.lib_sel = index;
        self.reload_tool_edit();
        if let Some(draft) = self.tool_drafts.remove(&index) {
            self.tool_edit = Some(draft.tool);
            self.fields = draft.fields;
            self.library_preset = draft.preset;
            self.preset_values = draft.preset_values;
            self.preset_buffers = draft.preset_buffers;
        }
    }

    fn material_choices(&self, unlinked: &str) -> Vec<MaterialChoice> {
        std::iter::once(MaterialChoice {
            id: None,
            name: unlinked.into(),
        })
        .chain(self.library.materials.iter().map(|m| MaterialChoice {
            id: Some(m.id),
            name: m.name.clone(),
        }))
        .collect()
    }
    pub(super) fn stock_material_picker(&self, setup: &cam_model::Setup) -> Element<'_, Message> {
        let mut choices = self.material_choices("No managed material");
        if let Some(id) = setup.material_id {
            if !choices.iter().any(|m| m.id == Some(id)) {
                choices.push(MaterialChoice {
                    id: Some(id),
                    name: format!("{} (unavailable in library)", setup.material),
                });
            }
        }
        let selected = choices.iter().find(|m| m.id == setup.material_id).cloned();
        column![
            pick_list(choices, selected, Message::SetupMaterialPicked)
                .text_size(12)
                .padding([7, 9])
                .style(inspector_picker)
                .width(Length::Fill),
            text(
                if setup.material_id.is_none() && !setup.material.is_empty() {
                    format!("Legacy material: {}", setup.material)
                } else {
                    "Manage materials in Tools → Materials.".into()
                }
            )
            .size(11)
            .color(INSPECTOR_MUTED),
        ]
        .spacing(6)
        .into()
    }
    pub(super) fn library_preset_mut(&mut self) -> Option<&mut ToolPreset> {
        let id = self.library_preset?;
        self.tool_edit
            .as_mut()?
            .presets
            .iter_mut()
            .find(|p| p.id == id)
    }
    pub(super) fn select_library_preset(&mut self, id: Option<u64>) {
        self.edited_preset_input = None;
        if let Some(current) = self.library_preset {
            if self
                .tool_edit
                .as_ref()
                .is_some_and(|tool| tool.presets.iter().any(|p| p.id == current))
            {
                self.preset_buffers
                    .insert(current, self.preset_values.clone());
            }
        }
        self.library_preset = id;
        self.preset_values.clear();
        if let Some(values) = id.and_then(|id| self.preset_buffers.remove(&id)) {
            self.preset_values = values;
            return;
        }
        if let Some(p) = self
            .tool_edit
            .as_ref()
            .and_then(|t| t.presets.iter().find(|p| Some(p.id) == id))
        {
            self.preset_values = [
                (PresetField::Rpm, fmt_num(p.spindle_rpm)),
                (PresetField::Feed, fmt_num(p.feed)),
                (PresetField::Plunge, fmt_num(p.plunge_feed)),
                (
                    PresetField::Ramp,
                    p.entry_feeds.ramp.map(fmt_num).unwrap_or_default(),
                ),
                (
                    PresetField::LeadIn,
                    p.entry_feeds.lead_in.map(fmt_num).unwrap_or_default(),
                ),
                (
                    PresetField::LeadOut,
                    p.entry_feeds.lead_out.map(fmt_num).unwrap_or_default(),
                ),
                (
                    PresetField::Stepdown,
                    p.stepdown.map(fmt_num).unwrap_or_default(),
                ),
                (
                    PresetField::Overlap,
                    p.overlap.map(|v| fmt_num(v * 100.)).unwrap_or_default(),
                ),
            ]
            .into_iter()
            .collect();
        }
        self.refresh_preset_cutting_fields(None);
    }
    pub(super) fn add_library_preset(&mut self, duplicate: bool) {
        if self.preset_edit_invalid() {
            self.status = "Correct the current preset before adding another.".into();
            return;
        }
        let defaults = *self.controller.defaults();
        let Some(tool) = &mut self.tool_edit else {
            return;
        };
        let id = tool.next_preset_id();
        let source = if duplicate {
            tool.presets
                .iter()
                .find(|p| Some(p.id) == self.library_preset)
                .cloned()
        } else {
            None
        };
        let preset = ToolPreset {
            id,
            name: format!("Preset {id}"),
            ..source.unwrap_or(ToolPreset {
                id,
                name: String::new(),
                material_id: None,
                spindle_rpm: tool.nominal_rpm,
                feed: if tool.nominal_feed > 0. {
                    tool.nominal_feed
                } else {
                    defaults.feed
                },
                plunge_feed: if tool.nominal_plunge_feed > 0. {
                    tool.nominal_plunge_feed
                } else {
                    defaults.plunge_feed
                },
                stepdown: None,
                overlap: None,
                cooling: tool.cooling,
                entry_feeds: Default::default(),
            })
        };
        tool.presets.push(preset);
        self.select_library_preset(Some(id));
    }
    pub(super) fn delete_library_preset(&mut self) {
        if let Some(tool) = &mut self.tool_edit {
            tool.presets.retain(|p| Some(p.id) != self.library_preset);
            if let Some(id) = self.library_preset {
                self.preset_buffers.remove(&id);
            }
            let next = tool.presets.first().map(|p| p.id);
            self.select_library_preset(next);
        }
    }
    pub(super) fn edit_preset_value(&mut self, field: PresetField, value: String) {
        self.edited_preset_input = Some(field);
        let tool = self.tool_edit.clone();
        let fallback_rpm = self.controller.defaults().spindle_rpm;
        let mut linked = None;
        if field.valid(&value) {
            let parsed = value.trim().parse::<f64>().ok();
            if let Some(p) = self.library_preset_mut() {
                match field {
                    PresetField::Rpm => p.spindle_rpm = parsed.unwrap(),
                    PresetField::CuttingSpeed => {
                        if let Some(rpm) = tool
                            .as_ref()
                            .and_then(|t| t.rpm_for_cutting_speed(parsed.unwrap()))
                            .filter(|v| *v > 0.)
                        {
                            p.spindle_rpm = rpm;
                            linked = Some((PresetField::Rpm, rpm));
                        }
                    }
                    PresetField::Feed => p.feed = parsed.unwrap(),
                    PresetField::FeedPerTooth => {
                        let rpm = if p.spindle_rpm == 0. {
                            fallback_rpm
                        } else {
                            p.spindle_rpm
                        };
                        if let Some(feed) = tool
                            .as_ref()
                            .and_then(|t| t.feed_for_tooth(rpm, parsed.unwrap()))
                            .filter(|v| *v > 0.)
                        {
                            p.feed = feed;
                            linked = Some((PresetField::Feed, feed));
                        }
                    }
                    PresetField::Plunge => p.plunge_feed = parsed.unwrap(),
                    PresetField::Ramp => p.entry_feeds.ramp = parsed,
                    PresetField::LeadIn => p.entry_feeds.lead_in = parsed,
                    PresetField::LeadOut => p.entry_feeds.lead_out = parsed,
                    PresetField::Stepdown => p.stepdown = parsed,
                    PresetField::Overlap => p.overlap = parsed.map(|v| v / 100.),
                }
            }
        }
        if let Some((field, value)) = linked {
            self.preset_values.insert(field, fmt_num(value));
        }
        self.preset_values.insert(field, value);
        if matches!(
            field,
            PresetField::Rpm
                | PresetField::CuttingSpeed
                | PresetField::Feed
                | PresetField::FeedPerTooth
        ) {
            self.refresh_preset_cutting_fields(Some(field));
        }
    }
    pub(super) fn refresh_preset_cutting_fields(&mut self, edited: Option<PresetField>) {
        let Some(tool) = &self.tool_edit else {
            return;
        };
        let Some(preset) = tool
            .presets
            .iter()
            .find(|p| Some(p.id) == self.library_preset)
        else {
            return;
        };
        let rpm = self.effective_cutting_rpm(preset.spindle_rpm);
        for (field, value) in [
            (PresetField::CuttingSpeed, tool.cutting_speed(rpm)),
            (
                PresetField::FeedPerTooth,
                tool.feed_per_tooth(rpm, preset.feed),
            ),
        ] {
            if field == PresetField::CuttingSpeed
                && matches!(edited, Some(PresetField::Feed | PresetField::FeedPerTooth))
            {
                continue;
            }
            if edited != Some(field) {
                if let Some(value) = value {
                    self.preset_values.insert(field, fmt_num(value));
                } else {
                    self.preset_values.remove(&field);
                }
            }
        }
    }
    pub(super) fn preset_edit_invalid(&self) -> bool {
        self.tool_edit
            .as_ref()
            .is_some_and(|t| t.presets.iter().any(|p| p.validate().is_err()))
            || self
                .preset_values
                .iter()
                .any(|(field, value)| !field.valid(value))
            || self
                .preset_buffers
                .values()
                .any(|values| values.iter().any(|(field, value)| !field.valid(value)))
    }
    pub(super) fn library_preset_editor(&self) -> Element<'_, Message> {
        let Some(tool) = &self.tool_edit else {
            return column![].into();
        };
        let choices: Vec<_> = tool
            .presets
            .iter()
            .map(|p| PresetChoice {
                id: Some(p.id),
                name: p.name.clone(),
            })
            .collect();
        let selected = choices
            .iter()
            .find(|c| c.id == self.library_preset)
            .cloned();
        fn control_row<'a>(
            label: &'static str,
            control: Element<'a, Message>,
        ) -> Element<'a, Message> {
            row![text(label).size(12).width(245), control]
                .spacing(12)
                .align_y(Alignment::Center)
                .into()
        }
        let mut list = column![control_row(
            "Preset",
            pick_list(choices, selected, |c| Message::LibraryPresetSelected(
                c.id.unwrap()
            ))
            .placeholder("No presets")
            .text_size(12)
            .padding([7, 9])
            .style(inspector_picker)
            .width(Length::Fill)
            .into()
        ),]
        .spacing(6);
        if let Some(preset) = tool
            .presets
            .iter()
            .find(|p| Some(p.id) == self.library_preset)
        {
            let mut materials = self.material_choices("Any material");
            if let Some(id) = preset.material_id {
                if !materials.iter().any(|m| m.id == Some(id)) {
                    materials.push(MaterialChoice {
                        id: Some(id),
                        name: format!("Unavailable material #{id}"),
                    });
                }
            }
            let selected = materials
                .iter()
                .find(|m| m.id == preset.material_id)
                .cloned();
            list = list.push(control_row(
                "Name",
                inspector_input("", &preset.name)
                    .on_input(Message::PresetNameChanged)
                    .into(),
            ));
            list = list.push(control_row(
                "Material",
                pick_list(materials, selected, Message::PresetMaterialChanged)
                    .text_size(12)
                    .padding([7, 9])
                    .style(inspector_picker)
                    .width(Length::Fill)
                    .into(),
            ));
            list = list.push(control_row(
                "Cooling",
                pick_list(
                    &cam_model::Cooling::ALL[..],
                    Some(preset.cooling),
                    Message::PresetCoolingChanged,
                )
                .text_size(12)
                .padding([7, 9])
                .style(inspector_picker)
                .width(Length::Fill)
                .into(),
            ));
            for field in PresetField::ALL {
                let unavailable =
                    matches!(field, PresetField::CuttingSpeed | PresetField::FeedPerTooth)
                        && !self.preset_values.contains_key(&field);
                let value = self
                    .preset_values
                    .get(&field)
                    .map(String::as_str)
                    .unwrap_or("");
                let shown = self
                    .settings
                    .format_numeric_input(value, self.edited_preset_input == Some(field));
                let input = inspector_input(
                    if unavailable {
                        "Cutter data unavailable"
                    } else {
                        ""
                    },
                    &shown,
                );
                let input = if unavailable {
                    input
                } else {
                    input.on_input(move |v| Message::PresetValueChanged(field, v))
                };
                list = list.push(control_row(field.label(), input.into()));
                if !unavailable && !field.valid(value) {
                    list = list.push(text("Enter a valid value").size(11).color(palette::ERROR));
                }
            }
            if preset.name.trim().is_empty() {
                list = list.push(text("Enter a preset name").size(11).color(palette::ERROR));
            }
        }
        list.into()
    }
    /// Material IDs are persisted on presets; names are resolved from the shop library.
    pub(super) fn cutting_presets_table(&self) -> Element<'_, Message> {
        let Some(tool) = &self.tool_edit else {
            return column![].into();
        };
        let fmt_num = |value| self.settings.format_number(value);
        let selected = self.library_preset;
        let action = |icon, label, message| {
            render_command(&cmd_help(icon, label, message, label), true, true)
        };
        let toolbar = row![
            text("Cutting parameters").size(14),
            Space::new().width(Length::Fill),
            action(Icon::New, "Add preset", Some(Message::AddPreset)),
            action(
                Icon::Edit,
                "Edit preset",
                selected.map(|_| Message::EditPreset)
            ),
            action(
                Icon::Duplicate,
                "Duplicate preset",
                selected.map(|_| Message::DuplicatePreset)
            ),
            action(
                Icon::Delete,
                "Remove preset",
                selected.map(|_| Message::DeletePreset)
            ),
        ]
        .spacing(8)
        .align_y(Alignment::Center);
        let widths = [
            150., 150., 80., 75., 85., 145., 115., 150., 115., 105., 105., 105., 95., 90.,
        ];
        let cells = |values: Vec<String>| {
            let mut cells = row![].spacing(4);
            for (value, width) in values.into_iter().zip(widths) {
                cells = cells.push(text(value).size(12).width(Length::Fixed(width)));
            }
            cells
        };
        let mut rows = column![container(cells(
            [
                "Preset",
                "Material",
                "Material ID",
                "Cooling",
                "RPM",
                "Cutting speed m/min",
                "Feed mm/min",
                "Feed per tooth mm/tooth",
                "Plunge mm/min",
                "Ramp mm/min",
                "Lead-in mm/min",
                "Lead-out mm/min",
                "Stepdown mm",
                "Overlap %",
            ]
            .into_iter()
            .map(str::to_owned)
            .collect()
        ))
        .padding(7)]
        .spacing(2);
        // Show the cutter's fallback cutting data even before a named preset is added.
        let nominal = |value| {
            if value > 0. {
                fmt_num(value)
            } else {
                "Job default".into()
            }
        };
        rows = rows.push(
            container(cells(vec![
                "Tool defaults".into(),
                "Any material".into(),
                "—".into(),
                tool.cooling.to_string(),
                nominal(tool.nominal_rpm),
                tool.cutting_speed(self.effective_cutting_rpm(tool.nominal_rpm))
                    .map(fmt_num)
                    .unwrap_or_else(|| "—".into()),
                nominal(tool.nominal_feed),
                tool.feed_per_tooth(
                    self.effective_cutting_rpm(tool.nominal_rpm),
                    if tool.nominal_feed > 0. {
                        tool.nominal_feed
                    } else {
                        self.controller.defaults().feed
                    },
                )
                .map(fmt_num)
                .unwrap_or_else(|| "—".into()),
                nominal(tool.nominal_plunge_feed),
                "—".into(),
                "—".into(),
                "—".into(),
                "—".into(),
                "—".into(),
            ]))
            .padding(7),
        );
        let optional = |v: Option<f64>| v.map(fmt_num).unwrap_or_else(|| "—".into());
        for preset in &tool.presets {
            let material = preset
                .material_id
                .map(|id| {
                    self.library
                        .materials
                        .iter()
                        .find(|m| m.id == id)
                        .map(|m| m.name.clone())
                        .unwrap_or_else(|| format!("Unavailable #{id}"))
                })
                .unwrap_or_else(|| "Any material".into());
            let active = selected == Some(preset.id);
            let item =
                container(cells(vec![
                    preset.name.clone(),
                    material,
                    preset
                        .material_id
                        .map(|id| id.to_string())
                        .unwrap_or_else(|| "—".into()),
                    preset.cooling.to_string(),
                    fmt_num(preset.spindle_rpm),
                    optional(tool.cutting_speed(self.effective_cutting_rpm(preset.spindle_rpm))),
                    fmt_num(preset.feed),
                    optional(tool.feed_per_tooth(
                        self.effective_cutting_rpm(preset.spindle_rpm),
                        preset.feed,
                    )),
                    fmt_num(preset.plunge_feed),
                    optional(preset.entry_feeds.ramp),
                    optional(preset.entry_feeds.lead_in),
                    optional(preset.entry_feeds.lead_out),
                    optional(preset.stepdown),
                    optional(preset.overlap.map(|v| v * 100.)),
                ]))
                .padding(7)
                .width(Length::Fill)
                .style(move |_| container::Style {
                    background: active.then_some(Background::Color(palette::SELECT_BG)),
                    ..Default::default()
                });
            rows = rows.push(
                mouse_area(item)
                    .on_press(Message::LibraryPresetSelected(preset.id))
                    .on_double_click(Message::EditLibraryPreset(preset.id))
                    .interaction(iced::mouse::Interaction::Pointer),
            );
        }
        if tool.presets.is_empty() {
            rows = rows.push(text("No cutting presets for this cutter. Add a preset to define material-specific parameters.").size(12));
        }
        container(
            column![
                toolbar,
                scrollable(rows.width(1750))
                    .direction(scrollable::Direction::Both {
                        vertical: scrollable::Scrollbar::default(),
                        horizontal: scrollable::Scrollbar::default(),
                    })
                    .width(Length::Fill)
                    .height(Length::Fill),
                text("Edit tool defaults in the inspector. Apply changes saves cutter details and presets.")
                    .size(11)
                    .color(INSPECTOR_MUTED),
            ]
            .spacing(8),
        )
        .padding(16)
        .height(Length::FillPortion(2))
        .width(Length::Fill)
        .into()
    }

    pub(super) fn tool_management_overlay(&self) -> Option<Element<'_, Message>> {
        let (title, content, close) = if self.show_materials {
            (
                "Materials",
                self.managed_material_editor(),
                Message::CloseMaterials,
            )
        } else if self.show_preset_editor {
            (
                "Edit cutting preset",
                self.library_preset_editor(),
                Message::ClosePresetEditor,
            )
        } else {
            return None;
        };
        let can_close = self.show_materials || !self.preset_edit_invalid();
        let mut body = column![
            text(title).size(18),
            scrollable(content).height(Length::Fill),
        ]
        .spacing(12);
        if self.show_materials {
            body = body.push(text(self.status.clone()).size(11).color(INSPECTOR_MUTED));
        }
        let card = container(body.push(row![
            Space::new().width(Length::Fill),
            if self.show_materials {
                row![inspector_button("Done").on_press(close)]
            } else {
                row![
                    inspector_button("Cancel").on_press(Message::CancelPresetEditor),
                    inspector_button("OK").on_press_maybe(can_close.then_some(close))
                ]
                .spacing(8)
            }
        ]))
        .width(600)
        .height(
            (self.window.height - 60.).clamp(200., if self.show_materials { 820. } else { 700. }),
        )
        .padding(16)
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
        let backdrop = mouse_area(
            container(Space::new().width(Length::Fill).height(Length::Fill)).style(|_| {
                container::Style {
                    background: Some(Background::Color(Color::from_rgba(0., 0., 0., 0.45))),
                    ..Default::default()
                }
            }),
        )
        .on_press(Message::Noop);
        Some(
            iced::widget::stack![
                backdrop,
                container(iced::widget::opaque(card))
                    .width(Length::Fill)
                    .height(Length::Fill)
                    .align_x(Alignment::Center)
                    .align_y(Alignment::Center)
            ]
            .into(),
        )
    }

    pub(super) fn managed_material_editor(&self) -> Element<'_, Message> {
        let selected = self
            .selected_material
            .filter(|id| self.library.materials.iter().any(|m| m.id == *id));
        let action = |icon, label, message| {
            render_command(&cmd_help(icon, label, message, label), true, true)
        };
        let toolbar = row![
            text("Materials").size(14),
            Space::new().width(Length::Fill),
            action(Icon::New, "New", Some(Message::NewMaterial)),
            action(Icon::Edit, "Edit", selected.map(|_| Message::EditMaterial)),
            action(
                Icon::Delete,
                "Delete",
                selected.map(Message::DeleteMaterial)
            ),
        ]
        .spacing(8)
        .align_y(Alignment::Center);
        let mut list = column![
            toolbar,
            text("Shared across projects. IDs stay fixed when names change.")
                .size(11)
                .color(INSPECTOR_MUTED),
            inspector_button("Add starter materials").on_press(Message::AddStarterMaterials),
            text("Starter kc values are approximate; adjust them for your material and cut.")
                .size(11)
                .color(INSPECTOR_MUTED),
            container(
                row![
                    text("ID").size(12).width(70),
                    text("Material").size(12).width(Length::Fill),
                    text("kc (N/mm²)").size(12).width(140),
                ]
                .spacing(4)
            )
            .padding(7),
        ]
        .spacing(8);
        for material in &self.library.materials {
            let id = material.id;
            let active = selected == Some(id);
            let item = container(
                row![
                    text(id.to_string()).size(12).width(70),
                    text(&material.name).size(12).width(Length::Fill),
                    text(
                        material
                            .specific_cutting_force
                            .map(|v| v.to_string())
                            .unwrap_or_else(|| "Unset".into())
                    )
                    .size(12)
                    .width(140),
                ]
                .spacing(4),
            )
            .padding(7)
            .width(Length::Fill)
            .style(move |_| container::Style {
                background: active.then_some(Background::Color(palette::SELECT_BG)),
                ..Default::default()
            });
            list = list.push(
                mouse_area(item)
                    .on_press(Message::SelectMaterial(id))
                    .on_double_click(Message::EditMaterialRow(id))
                    .interaction(iced::mouse::Interaction::Pointer),
            );
        }
        if self.creating_material {
            list = list.push(
                row![
                    inspector_input("New material", &self.new_material_name)
                        .on_input(Message::NewMaterialNameChanged),
                    inspector_button("Add").on_press_maybe(
                        (!self.new_material_name.trim().is_empty()).then_some(Message::AddMaterial)
                    ),
                ]
                .spacing(6),
            );
        }
        if let Some(material) = self
            .editing_material
            .and_then(|id| self.library.materials.iter().find(|m| m.id == id))
        {
            let id = material.id;
            let kc = self
                .material_force_inputs
                .get(&id)
                .cloned()
                .unwrap_or_else(|| {
                    material
                        .specific_cutting_force
                        .map(|v| v.to_string())
                        .unwrap_or_default()
                });
            let invalid = super::cutting_forces::parse_material_kc(&kc).is_err();
            list = list.push(
                column![
                    text("Edit material").size(14),
                    inspector_input("Material name", &material.name)
                        .on_input(move |v| Message::MaterialRenamed(id, v)),
                    text("Specific cutting force kc (N/mm²)").size(11),
                    inspector_input("Unset", &kc)
                        .on_input(move |v| Message::MaterialCuttingForceChanged(id, v)),
                    text(if invalid {
                        "Enter a finite positive kc, or leave blank. Invalid input is not saved."
                    } else {
                        "Effective kc for your cutting conditions. Blank disables force estimates."
                    })
                    .size(11)
                    .color(if invalid {
                        palette::ERROR
                    } else {
                        INSPECTOR_MUTED
                    }),
                ]
                .spacing(6),
            );
        }
        list.into()
    }

    pub(super) fn add_managed_material(&mut self) {
        let name = self.new_material_name.trim();
        if name.is_empty()
            || self
                .library
                .materials
                .iter()
                .any(|m| m.name.eq_ignore_ascii_case(name))
        {
            self.status = "Use a unique material name.".into();
            return;
        }
        self.library.material_id_floor = self.library.material_id_floor.max(
            self.library
                .materials
                .iter()
                .map(|m| m.id)
                .max()
                .unwrap_or(0),
        ) + 1;
        self.library.materials.push(Material {
            id: self.library.material_id_floor,
            name: name.into(),
            specific_cutting_force: None,
        });
        self.new_material_name.clear();
        self.selected_material = Some(self.library.material_id_floor);
        self.creating_material = false;
        self.library.save();
    }
    pub(super) fn rename_managed_material(&mut self, id: u64, name: String) {
        if name.trim().is_empty()
            || self
                .library
                .materials
                .iter()
                .any(|m| m.id != id && m.name.eq_ignore_ascii_case(name.trim()))
        {
            return;
        }
        if let Some(material) = self.library.materials.iter_mut().find(|m| m.id == id) {
            material.name = name;
        }
        self.library.save();
        if let Some(player) = &mut self.simulation {
            player.sync_force_material(&self.library);
        }
    }
    pub(super) fn delete_managed_material(&mut self, id: u64) {
        let in_use = self
            .library
            .tools
            .iter()
            .chain(self.tool_edit.iter())
            .any(|t| t.presets.iter().any(|p| p.material_id == Some(id)))
            || self
                .controller
                .document()
                .setups
                .iter()
                .any(|s| s.material_id == Some(id))
            || self
                .setup_draft
                .as_ref()
                .is_some_and(|s| s.material_id == Some(id));
        if in_use {
            self.status =
                "This material is used by a stock or preset. Unlink it before deleting.".into();
            return;
        }
        self.library.materials.retain(|m| m.id != id);
        self.material_force_inputs.remove(&id);
        if self.selected_material == Some(id) {
            self.selected_material = None;
        }
        if self.editing_material == Some(id) {
            self.editing_material = None;
        }
        self.library.save();
    }
    pub(super) fn operation_preset_tool(&self) -> Option<&cam_model::Tool> {
        let op = self.operation_draft.as_ref().or_else(|| {
            if let Selection::Operation(id) = self.controller.selection() {
                self.controller.operation(id)
            } else {
                None
            }
        })?;
        if self.cad_pending_kind.is_some() {
            return self.cad_selected_project_tool.as_ref().or_else(|| {
                self.library
                    .tools
                    .iter()
                    .find(|t| Some(t.number) == self.cad_tool_pick)
            });
        }
        self.operation_tools
            .iter()
            .chain(self.controller.inspection_setup().tools.iter())
            .find(|t| t.number == op.tool())
    }
    pub(super) fn auto_operation_preset(&mut self) {
        if let Some(cooling) = self.operation_preset_tool().map(|t| t.cooling) {
            self.edit_operation_draft(|op| op.set_cooling(cooling));
        }
        let preset = self
            .operation_preset_tool()
            .and_then(|t| t.preset_for_material(self.controller.inspection_setup().material_id))
            .cloned();
        if let Some(mut preset) = preset {
            if self.creating_operation()
                && matches!(
                    self.operation_draft,
                    Some(Operation::Pocket(_) | Operation::Face(_) | Operation::Profile(_))
                )
                && preset.stepdown == Some(0.0)
            {
                preset.stepdown = None;
            }
            self.apply_operation_preset(&preset);
        }
        if self.creating_operation()
            && !self.creation_edited_fields.contains(&Field::Stepdown)
            && matches!(
                self.operation_draft,
                Some(Operation::Pocket(_) | Operation::Face(_) | Operation::Profile(_))
            )
        {
            self.edit_operation_draft(|op| {
                cam_model::OperationParameter::Stepdown
                    .set_source(op, pocket_parameters::DEFAULT_STEPDOWN)
            });
            self.fields
                .insert(Field::Stepdown, pocket_parameters::DEFAULT_STEPDOWN.into());
        }
    }
    fn apply_operation_preset(&mut self, preset: &ToolPreset) {
        let creating = self.creating_operation();
        let tool_diameter = self.operation_preset_tool().map(|tool| tool.diameter);
        let Some(op) = &mut self.operation_draft else {
            return;
        };
        if let Err(error) = op.apply_tool_preset(preset) {
            self.status = error;
            return;
        }
        if matches!(
            op,
            Operation::Pocket(_) | Operation::Face(_) | Operation::Profile(_)
        ) {
            if let Some(stepdown) = preset.stepdown {
                cam_model::OperationParameter::Stepdown.set_source(op, &stepdown.to_string());
            }
        }
        if let (Operation::Pocket(pocket), Some(overlap), Some(diameter)) =
            (&mut *op, preset.overlap, tool_diameter)
        {
            if creating {
                let stepover = cam_model::OperationParameter::MaximumStepover;
                let source = stepover.default_expression();
                stepover.set_source(op, source);
                self.fields.insert(
                    Field::PocketStepover,
                    format!("={source}"),
                );
            } else {
                let stepover = diameter - overlap * diameter;
                pocket.maximum_stepover = Some(stepover);
                self.fields
                    .insert(Field::PocketStepover, fmt_num(stepover));
            }
        }
        if let (Operation::Face(face), Some(overlap), Some(diameter)) =
            (&mut *op, preset.overlap, tool_diameter)
        {
            if creating {
                face.absolute_overlap = None;
                cam_model::OperationParameter::FaceOverlap.set_source(op, "=tool_diameter * 0.5");
                self.fields
                    .insert(Field::FaceOverlap, "=tool_diameter * 0.5".into());
            } else {
                face.absolute_overlap = Some(overlap * diameter);
                self.fields
                    .insert(Field::FaceOverlap, fmt_num(overlap * diameter));
            }
        }
        // Refresh only the cutting values, preserving unrelated in-progress text.
        for field in [
            Field::SpindleRpm,
            Field::Feed,
            Field::PlungeFeed,
            Field::RampFeed,
            Field::LeadInFeed,
            Field::LeadOutFeed,
            Field::Stepdown,
            Field::Overlap,
            Field::Peck,
        ] {
            if matches!(field, Field::Stepdown | Field::Peck) && preset.stepdown.is_none() {
                continue;
            }
            if field == Field::Overlap && preset.overlap.is_none() {
                continue;
            }
            if (field == Field::RampFeed && preset.entry_feeds.ramp.is_none())
                || (field == Field::LeadInFeed && preset.entry_feeds.lead_in.is_none())
                || (field == Field::LeadOutFeed && preset.entry_feeds.lead_out.is_none())
            {
                continue;
            }
            if let Some(value) = op_field(op, field) {
                self.fields.insert(field, fmt_num(value));
            }
        }
        self.refresh_cutting_fields(None);
    }
    pub(super) fn pick_operation_preset(&mut self, id: Option<u64>) {
        let Some(id) = id else {
            self.edit_operation_draft(Operation::clear_tool_preset);
            return;
        };
        let preset = self
            .operation_preset_tool()
            .and_then(|t| t.presets.iter().find(|p| p.id == id))
            .or_else(|| {
                self.operation_draft
                    .as_ref()?
                    .tool_preset()
                    .filter(|p| p.id == id)
            })
            .cloned();
        if let Some(preset) = preset {
            self.apply_operation_preset(&preset);
        }
    }
    pub(super) fn operation_preset_picker(&self, op: &Operation) -> Element<'_, Message> {
        let mut choices = vec![PresetChoice {
            id: None,
            name: "Manual cutting data".into(),
        }];
        if let Some(tool) = self.operation_preset_tool() {
            choices.extend(tool.presets.iter().map(|p| PresetChoice {
                id: Some(p.id),
                name: p.name.clone(),
            }));
        }
        let selected = if let Some(preset) = op.tool_preset() {
            if !choices.iter().any(|p| p.id == Some(preset.id)) {
                choices.push(PresetChoice {
                    id: Some(preset.id),
                    name: preset.name.clone(),
                });
            }
            choices.iter().find(|p| p.id == Some(preset.id)).cloned()
        } else {
            choices.first().cloned()
        };
        let mut list = column![
            text("Cutting preset").size(12),
            pick_list(choices, selected, Message::OperationPresetPicked)
                .text_size(12)
                .padding([7, 9])
                .style(inspector_picker)
                .width(Length::Fill),
            text("Cooling").size(12),
            parameter_options::COOLING.picker(op.cooling(), &cam_model::Cooling::ALL, Length::Fill),
        ]
        .spacing(6);
        if let Some(preset) = op.tool_preset() {
            let mut baseline = op.clone();
            if baseline.apply_tool_preset(preset).is_ok() && !baseline.same_work(op) {
                list = list.push(
                    text("Preset values overridden for this operation.")
                        .size(11)
                        .color(INSPECTOR_MUTED),
                );
            }
        }
        list.into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn preset(id: u64, material: u64, feed: f64) -> ToolPreset {
        ToolPreset {
            id,
            name: format!("Preset {id}"),
            material_id: Some(material),
            spindle_rpm: 5000.,
            feed,
            plunge_feed: 100.,
            stepdown: Some(0.5),
            overlap: Some(0.25),
            cooling: Default::default(),
            entry_feeds: Default::default(),
        }
    }
    fn app() -> App {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample").unwrap();
        app.setup_draft = None;
        app.controller.edit_setup(|s| {
            s.material_id = Some(7);
            s.material = "Aluminum".into();
        });
        let mut tool = crate::tool_library::default_tool(1, ToolKind::EndMill);
        tool.presets = vec![preset(1, 7, 500.), preset(2, 8, 200.)];
        app.library = ToolLibrary {
            tools: vec![tool],
            ..Default::default()
        };
        app.controller.select(Selection::Setup);
        app.refresh_fields();
        app
    }
    fn send(app: &mut App, message: Message) {
        let _ = app.update(message);
    }
    fn number(app: &App, field: Field) -> f64 {
        app.fields[&field].parse().unwrap()
    }
    fn near(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-8, "{actual} != {expected}");
    }

    #[test]
    fn starter_material_button_adds_missing_materials_to_existing_library() {
        let mut app = app();
        app.new_material_name = "Pine".into();
        app.add_managed_material();
        let existing = app.library.materials[0].clone();
        send(&mut app, Message::AddStarterMaterials);
        assert_eq!(app.library.materials.len(), 10);
        assert_eq!(app.library.materials[0], existing);
        assert!(app.status.contains("9 starter"));
        send(&mut app, Message::AddStarterMaterials);
        assert_eq!(app.library.materials.len(), 10);
        let _editor = app.managed_material_editor();
    }

    #[test]
    fn linked_operation_cutting_data_survives_apply_cancel_and_undo() {
        let mut app = app();
        app.library.tools[0].diameter = 6.;
        app.library.tools[0].flutes = 2;
        app.library.tools[0].presets[0].spindle_rpm = 24000.;
        app.library.tools[0].presets[0].feed = 3000.;
        send(&mut app, Message::BeginOp(OpKind::Pocket));
        near(number(&app, Field::CuttingSpeed), 452.3893421169302);
        near(number(&app, Field::FeedPerTooth), 0.0625);
        send(
            &mut app,
            Message::FieldChanged(Field::CuttingSpeed, "226.1946710584651".into()),
        );
        near(number(&app, Field::SpindleRpm), 12000.);
        near(number(&app, Field::Feed), 3000.);
        near(number(&app, Field::FeedPerTooth), 0.125);
        send(
            &mut app,
            Message::FieldChanged(Field::FeedPerTooth, "0.05".into()),
        );
        near(number(&app, Field::Feed), 1200.);
        send(&mut app, Message::FieldChanged(Field::Feed, "1800".into()));
        near(number(&app, Field::FeedPerTooth), 0.075);
        send(
            &mut app,
            Message::FieldChanged(Field::SpindleRpm, "24000".into()),
        );
        near(number(&app, Field::CuttingSpeed), 452.3893421169302);
        near(number(&app, Field::FeedPerTooth), 0.0375);
        app.controller
            .set_pending_operation_geometry(&[LoopRef::outer(0)], None)
            .unwrap();
        assert!(app.creation_ready(), "{}", app.status);
        send(&mut app, Message::ConfirmOp);
        let id = app
            .controller
            .inspection_setup()
            .operations
            .last()
            .unwrap()
            .id();
        let saved = app.controller.document().clone();
        app.begin_operation_draft(id);
        assert!(!app.inspector_dirty());
        send(
            &mut app,
            Message::FieldChanged(Field::FeedPerTooth, "0.1".into()),
        );
        send(&mut app, Message::CancelOperationDraft);
        assert_eq!(app.controller.document(), &saved);
        app.begin_operation_draft(id);
        near(number(&app, Field::FeedPerTooth), 0.0375);
        send(
            &mut app,
            Message::FieldChanged(Field::FeedPerTooth, "0.1".into()),
        );
        send(&mut app, Message::Apply);
        near(
            op_field(app.controller.operation(id).unwrap(), Field::Feed).unwrap(),
            4800.,
        );
        assert!(app.controller.undo());
        assert_eq!(app.controller.document(), &saved);
    }

    #[test]
    fn preset_cutting_data_links_both_ways_and_round_trips_with_the_cutter() {
        let mut app = app();
        app.library.tools[0].diameter = 6.;
        app.library.tools[0].flutes = 2;
        app.switch_workspace(Workspace::Tools);
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Rpm, "24000".into()),
        );
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "3000".into()),
        );
        near(
            app.preset_values[&PresetField::CuttingSpeed]
                .parse()
                .unwrap(),
            452.3893421169302,
        );
        near(
            app.preset_values[&PresetField::FeedPerTooth]
                .parse()
                .unwrap(),
            0.0625,
        );
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::CuttingSpeed, "226.1946710584651".into()),
        );
        near(
            app.preset_values[&PresetField::Rpm].parse().unwrap(),
            12000.,
        );
        near(
            app.preset_values[&PresetField::FeedPerTooth]
                .parse()
                .unwrap(),
            0.125,
        );
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::FeedPerTooth, "0.05".into()),
        );
        near(
            app.preset_values[&PresetField::Feed].parse().unwrap(),
            1200.,
        );
        let tool = app.tool_edit.as_ref().unwrap();
        let restored: cam_model::Tool =
            serde_json::from_str(&serde_json::to_string(tool).unwrap()).unwrap();
        let preset = &restored.presets[0];
        near(
            restored.cutting_speed(preset.spindle_rpm).unwrap(),
            226.1946710584651,
        );
        near(
            restored
                .feed_per_tooth(preset.spindle_rpm, preset.feed)
                .unwrap(),
            0.05,
        );
        send(&mut app, Message::LibraryPresetSelected(2));
        send(&mut app, Message::LibraryPresetSelected(1));
        near(
            app.preset_values[&PresetField::FeedPerTooth]
                .parse()
                .unwrap(),
            0.05,
        );
        send(
            &mut app,
            Message::FieldChanged(Field::ToolDiameter, "12".into()),
        );
        near(
            app.preset_values[&PresetField::CuttingSpeed]
                .parse()
                .unwrap(),
            452.3893421169302,
        );
        send(&mut app, Message::FieldChanged(Field::Flutes, "4".into()));
        near(
            app.preset_values[&PresetField::FeedPerTooth]
                .parse()
                .unwrap(),
            0.025,
        );
    }

    #[test]
    fn invalid_derived_edits_remain_visible_and_do_not_change_saved_cutting_data() {
        let mut app = app();
        send(&mut app, Message::BeginOp(OpKind::Pocket));
        let rpm = app.fields[&Field::SpindleRpm].clone();
        for invalid in ["", "-", "-1", "0", "NaN", "inf"] {
            send(
                &mut app,
                Message::FieldChanged(Field::CuttingSpeed, invalid.into()),
            );
            assert!(app.field_invalid(Field::CuttingSpeed));
            assert_eq!(app.fields[&Field::SpindleRpm], rpm);
        }
        app.refresh_operation_fields();
        assert_eq!(app.fields[&Field::CuttingSpeed], "inf");
        send(&mut app, Message::FieldChanged(Field::Feed, "1000".into()));
        assert_eq!(app.fields[&Field::CuttingSpeed], "inf");
        send(&mut app, Message::CancelOp);
        app.switch_workspace(Workspace::Tools);
        let saved = app.tool_edit.clone();
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::CuttingSpeed, "NaN".into()),
        );
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Plunge, "120".into()),
        );
        assert!(app.preset_edit_invalid());
        assert_eq!(app.preset_values[&PresetField::CuttingSpeed], "NaN");
        assert_eq!(
            app.tool_edit.as_ref().unwrap().presets[0].spindle_rpm,
            saved.unwrap().presets[0].spindle_rpm
        );
    }

    #[test]
    fn inherited_rpm_and_missing_flutes_are_handled_without_division_by_zero() {
        let mut app = app();
        app.library.tools[0].presets[0].spindle_rpm = 0.;
        send(&mut app, Message::BeginOp(OpKind::Pocket));
        let tool = app.operation_preset_tool().unwrap();
        let rpm = app.controller.defaults().spindle_rpm;
        near(
            number(&app, Field::CuttingSpeed),
            tool.cutting_speed(rpm).unwrap(),
        );
        near(
            number(&app, Field::FeedPerTooth),
            tool.feed_per_tooth(rpm, 500.).unwrap(),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::FeedPerTooth, "0.1".into()),
        );
        near(
            number(&app, Field::Feed),
            rpm * f64::from(app.operation_preset_tool().unwrap().flutes) * 0.1,
        );
        send(&mut app, Message::CancelOp);
        app.library.tools[0].flutes = 0;
        send(&mut app, Message::BeginOp(OpKind::Pocket));
        let fields = app.inspector_fields();
        assert!(fields.contains(&Field::CuttingSpeed));
        assert!(!fields.contains(&Field::FeedPerTooth));
        send(&mut app, Message::CancelOp);
        app.switch_workspace(Workspace::Tools);
        assert!(!app.preset_values.contains_key(&PresetField::FeedPerTooth));
        assert!(!app.preset_edit_invalid());
    }
    #[test]
    fn creating_switching_overriding_and_undoing_presets_is_transactional() {
        let mut app = app();
        let before = app.controller.document().clone();
        send(&mut app, Message::BeginOp(OpKind::Pocket));
        assert_eq!(
            app.operation_draft
                .as_ref()
                .unwrap()
                .tool_preset()
                .unwrap()
                .id,
            1
        );
        assert_eq!(app.fields[&Field::Feed], "500");
        app.controller
            .set_pending_operation_geometry(&[LoopRef::outer(0)], None)
            .unwrap();
        send(&mut app, Message::FieldChanged(Field::Depth, "2.5".into()));
        send(
            &mut app,
            Message::OperationPresetPicked(PresetChoice {
                id: Some(2),
                name: "Preset 2".into(),
            }),
        );
        assert_eq!(app.fields[&Field::Feed], "200");
        assert_eq!(app.fields[&Field::Depth], "2.5");
        send(&mut app, Message::FieldChanged(Field::Feed, "321".into()));
        send(
            &mut app,
            Message::FieldChanged(Field::Stepdown, "0.2".into()),
        );
        send(&mut app, Message::OperationTabChanged(OperationTab::Passes));
        send(&mut app, Message::OperationTabChanged(OperationTab::Tool));
        assert_eq!(app.fields[&Field::Feed], "321");
        assert!(app.creation_ready(), "{}", app.status);
        send(&mut app, Message::ConfirmOp);
        let operation = app.controller.inspection_setup().operations.last().unwrap();
        let Operation::Pocket(o) = operation else {
            panic!()
        };
        assert_eq!(o.clear.feed, 321.);
        assert_eq!(o.clear.stepdown, 0.2);
        assert_eq!(o.tool_preset.as_ref().unwrap().id, 2);
        let after = app.controller.document().clone();
        assert!(app.controller.undo());
        assert_eq!(app.controller.document(), &before);
        assert!(app.controller.redo());
        assert_eq!(app.controller.document(), &after);
    }
    #[test]
    fn choosing_a_cutter_uses_current_library_presets_and_cancel_keeps_saved_values() {
        let mut app = app();
        send(&mut app, Message::BeginOp(OpKind::Profile));
        app.controller
            .set_pending_operation_geometry(&[LoopRef::outer(0)], None)
            .unwrap();
        send(&mut app, Message::ConfirmOp);
        let id = app
            .controller
            .inspection_setup()
            .operations
            .last()
            .unwrap()
            .id();
        let before = app.controller.document().clone();
        app.library.tools[0].presets[0].feed = 650.;
        send(&mut app, Message::OperationToolPicked(0));
        assert_eq!(app.fields[&Field::Feed], "650");
        assert_eq!(app.controller.document(), &before);
        send(&mut app, Message::CancelOperationDraft);
        assert_eq!(app.controller.document(), &before);
        app.begin_operation_draft(id);
        assert_eq!(app.fields[&Field::Feed], "500");
        send(&mut app, Message::OperationToolPicked(0));
        send(&mut app, Message::Apply);
        assert_eq!(
            app.controller
                .operation(id)
                .unwrap()
                .tool_preset()
                .unwrap()
                .feed,
            650.
        );
        assert_eq!(
            app.controller
                .inspection_setup()
                .tools
                .iter()
                .find(|t| t.number == app.controller.operation(id).unwrap().tool())
                .unwrap()
                .presets[0]
                .feed,
            650.
        );
    }
    #[test]
    fn invalid_preset_buffers_block_apply_and_cancel_discards_preset_edits() {
        let mut app = app();
        app.switch_workspace(Workspace::Tools);
        app.lib_sel = 0;
        app.reload_tool_edit();
        let saved = app.library.tools[0].clone();
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "NaN".into()),
        );
        assert!(app.any_field_invalid());
        app.apply_inspector();
        assert_eq!(app.library.tools[0], saved);
        send(&mut app, Message::CancelToolDraft);
        assert!(!app.preset_edit_invalid());
        assert_eq!(app.tool_edit.as_ref().unwrap(), &saved);
        send(&mut app, Message::AddPreset);
        assert_eq!(app.tool_edit.as_ref().unwrap().presets.len(), 3);
        assert_eq!(app.library.tools[0].presets.len(), 2);
        send(&mut app, Message::CancelToolDraft);
        assert_eq!(app.tool_edit.as_ref().unwrap(), &saved);
    }
    #[test]
    fn cooling_preset_edits_and_operation_overrides_are_transactional() {
        use cam_model::Cooling;
        let mut app = app();
        app.library.tools[0].cooling = Cooling::Mms;
        app.library.tools[0].presets[0].cooling = Cooling::Mms;
        let before = app.controller.document().clone();
        send(&mut app, Message::BeginOp(OpKind::Pocket));
        assert_eq!(
            app.operation_draft.as_ref().unwrap().cooling(),
            Cooling::Mms
        );
        send(&mut app, Message::OperationCoolingChanged(Cooling::None));
        let op = app.operation_draft.as_ref().unwrap();
        assert_eq!(op.cooling(), Cooling::None);
        assert_eq!(op.tool_preset().unwrap().cooling, Cooling::Mms);
        send(
            &mut app,
            Message::OperationPresetPicked(PresetChoice {
                id: Some(1),
                name: "Preset 1".into(),
            }),
        );
        assert_eq!(
            app.operation_draft.as_ref().unwrap().cooling(),
            Cooling::Mms
        );
        send(&mut app, Message::CancelOp);
        assert_eq!(app.controller.document(), &before);
        app.switch_workspace(Workspace::Tools);
        let saved = app.library.tools[0].clone();
        send(&mut app, Message::PresetCoolingChanged(Cooling::None));
        assert_eq!(
            app.tool_edit.as_ref().unwrap().presets[0].cooling,
            Cooling::None
        );
        assert_eq!(app.library.tools[0], saved);
        send(&mut app, Message::CancelToolDraft);
        assert_eq!(app.tool_edit.as_ref().unwrap(), &saved);
        send(&mut app, Message::ToolCoolingChanged(Cooling::None));
        assert!(app.inspector_dirty());
        send(&mut app, Message::CancelToolDraft);
        assert_eq!(app.tool_edit.as_ref().unwrap().cooling, Cooling::Mms);
    }

    #[test]
    fn selecting_a_preset_does_not_block_selecting_another_cutter() {
        let mut app = app();
        let mut other = crate::tool_library::default_tool(2, ToolKind::BallMill);
        other.presets = vec![preset(3, 7, 900.)];
        app.library.tools.push(other.clone());
        app.switch_workspace(Workspace::Tools);
        send(&mut app, Message::LibraryPresetSelected(2));
        assert!(!app.inspector_dirty());
        send(&mut app, Message::SelectLibraryTool(1));
        assert_eq!(app.lib_sel, 1);
        assert_eq!(app.tool_edit.as_ref(), Some(&other));
        assert_eq!(app.library_preset, Some(3));
        assert!(app.tool_drafts.is_empty());
    }

    #[test]
    fn browsing_cutters_retains_pending_preset_values_and_invalid_buffers() {
        let mut app = app();
        app.library
            .tools
            .push(crate::tool_library::default_tool(2, ToolKind::BallMill));
        app.switch_workspace(Workspace::Tools);
        let saved = app.library.tools.clone();
        send(&mut app, Message::LibraryPresetSelected(2));
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "750".into()),
        );
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Rpm, "invalid".into()),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::ToolDiameter, "unfinished".into()),
        );
        send(&mut app, Message::SelectLibraryTool(1));
        assert_eq!(app.lib_sel, 1);
        assert!(!app.inspector_dirty());
        assert!(!app.any_field_invalid());
        send(
            &mut app,
            Message::ToolDetailChanged(ToolDetail::Description, "Other cutter".into()),
        );
        send(&mut app, Message::LibToolMenu(0));
        assert_eq!(app.lib_sel, 0);
        assert_eq!(app.library_preset, Some(2));
        assert_eq!(app.preset_values[&PresetField::Rpm], "invalid");
        assert_eq!(app.fields[&Field::ToolDiameter], "unfinished");
        assert_eq!(app.tool_edit.as_ref().unwrap().presets[1].feed, 750.);
        assert!(app.any_field_invalid());
        assert_eq!(app.library.tools, saved);
        send(&mut app, Message::CancelToolDraft);
        assert_eq!(app.tool_edit.as_ref(), Some(&saved[0]));
        send(&mut app, Message::SelectLibraryTool(1));
        assert_eq!(app.tool_edit.as_ref().unwrap().description, "Other cutter");
        assert!(app.tool_drafts.is_empty());
        send(&mut app, Message::CancelToolDraft);
        assert_eq!(app.tool_edit.as_ref(), Some(&saved[1]));
    }

    #[test]
    fn catalog_edits_are_transactional_and_survive_kind_changes() {
        let mut app = app();
        app.switch_workspace(Workspace::Tools);
        app.lib_sel = 0;
        app.reload_tool_edit();
        let saved = app.library.tools[0].clone();
        for (field, value) in [
            (ToolDetail::Description, "Finishing cutter"),
            (ToolDetail::Manufacturer, "Example Tools"),
            (ToolDetail::ProductId, "EM-6"),
            (ToolDetail::ProductLink, "https://example.com/EM-6"),
            (ToolDetail::CutterMaterial, "Carbide"),
            (ToolDetail::Coating, "TiAlN"),
        ] {
            send(&mut app, Message::ToolDetailChanged(field, value.into()));
        }
        assert!(app.inspector_dirty());
        assert_eq!(app.library.tools[0], saved);
        let edited = app.tool_edit.clone().unwrap();
        send(
            &mut app,
            Message::ToolKindChanged(ToolKind::Drill {
                point_angle_deg: 118.,
            }),
        );
        assert_eq!(
            app.tool_edit.as_ref().unwrap().description,
            edited.description
        );
        assert_eq!(
            app.tool_edit.as_ref().unwrap().manufacturer,
            edited.manufacturer
        );
        assert_eq!(
            app.tool_edit.as_ref().unwrap().product_id,
            edited.product_id
        );
        assert_eq!(
            app.tool_edit.as_ref().unwrap().product_link,
            edited.product_link
        );
        assert_eq!(app.tool_edit.as_ref().unwrap().cutter_material, edited.cutter_material);
        assert_eq!(app.tool_edit.as_ref().unwrap().coating, edited.coating);
        send(&mut app, Message::CancelToolDraft);
        assert_eq!(app.tool_edit.as_ref().unwrap(), &saved);
        send(&mut app, Message::ToolDetailChanged(ToolDetail::CutterMaterial, "Carbide".into()));
        send(&mut app, Message::ToolDetailChanged(ToolDetail::Coating, "TiAlN".into()));
        send(&mut app, Message::Apply);
        assert_eq!(app.library.tools[0].cutter_material, "Carbide");
        assert_eq!(app.library.tools[0].coating, "TiAlN");
    }

    #[test]
    fn compression_direction_can_be_applied_to_a_tool() {
        let mut app = app();
        app.switch_workspace(Workspace::Tools);
        app.lib_sel = 0;
        app.reload_tool_edit();
        send(&mut app, Message::ToolCuttingDirChanged(CutDir::Compression));
        assert!(app.inspector_dirty());
        assert!(library_extra(app.tool_edit.as_ref().unwrap()).unwrap().contains(", C"));
        send(&mut app, Message::Apply);
        assert_eq!(app.library.tools[0].cutting_direction, CutDir::Compression);
    }

    #[test]
    fn display_rounding_keeps_tool_and_preset_values_exact() {
        let mut app = app();
        app.switch_workspace(Workspace::Tools);
        send(
            &mut app,
            Message::FieldChanged(Field::ToolDiameter, "6.123456".into()),
        );
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "123.456789".into()),
        );
        let before = app.tool_edit.clone();
        let fields = app.fields.clone();
        let preset_values = app.preset_values.clone();
        app.settings.number_decimals = 2;
        app.begin_preset_edit();
        let _ = app.library_editor();
        let _ = app.library_preset_editor();
        let _ = app.cutting_presets_table();
        assert_eq!(app.tool_edit, before);
        assert_eq!(app.fields, fields);
        assert_eq!(app.preset_values, preset_values);
        assert_eq!(
            app.settings
                .format_number(app.tool_edit.as_ref().unwrap().diameter),
            "6.12"
        );
        assert_eq!(
            app.settings
                .format_number(app.tool_edit.as_ref().unwrap().presets[0].feed),
            "123.46"
        );
        app.settings.number_decimals = 4;
        assert_eq!(
            app.settings
                .format_number(app.tool_edit.as_ref().unwrap().diameter),
            "6.1235"
        );
    }

    #[test]
    fn double_click_edits_the_clicked_preset() {
        let mut app = app();
        app.switch_workspace(Workspace::Tools);
        send(&mut app, Message::EditLibraryPreset(2));
        assert_eq!(app.library_preset, Some(2));
        assert!(app.show_preset_editor);
        assert_eq!(app.preset_values[&PresetField::Feed], "200");
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "350".into()),
        );
        send(&mut app, Message::ClosePresetEditor);
        assert_eq!(app.tool_edit.as_ref().unwrap().presets[1].feed, 350.);
        assert_eq!(app.tool_edit.as_ref().unwrap().presets[0].feed, 500.);
    }

    #[test]
    fn duplicate_copies_selected_preset_while_add_uses_tool_defaults() {
        let mut app = app();
        app.switch_workspace(Workspace::Tools);
        send(&mut app, Message::LibraryPresetSelected(2));
        let source = app.tool_edit.as_ref().unwrap().presets[1].clone();
        send(&mut app, Message::DuplicatePreset);
        let copy = app.tool_edit.as_ref().unwrap().presets.last().unwrap();
        assert_ne!(copy.id, source.id);
        assert_ne!(copy.name, source.name);
        let mut expected = source;
        expected.id = copy.id;
        expected.name = copy.name.clone();
        assert_eq!(copy, &expected);
        assert!(app.show_preset_editor);
        send(&mut app, Message::ClosePresetEditor);
        send(&mut app, Message::AddPreset);
        let added = app.tool_edit.as_ref().unwrap().presets.last().unwrap();
        assert_eq!(added.material_id, None);
        assert_eq!(added.stepdown, None);
        assert_eq!(
            added.spindle_rpm,
            app.tool_edit.as_ref().unwrap().nominal_rpm
        );
    }

    #[test]
    fn switching_presets_retains_invalid_input_without_blocking_navigation() {
        let mut app = app();
        app.switch_workspace(Workspace::Tools);
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "invalid".into()),
        );
        send(&mut app, Message::LibraryPresetSelected(2));
        assert_eq!(app.library_preset, Some(2));
        assert_eq!(app.preset_values[&PresetField::Feed], "200");
        assert!(app.preset_edit_invalid());
        send(&mut app, Message::LibraryPresetSelected(1));
        assert_eq!(app.preset_values[&PresetField::Feed], "invalid");
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "600".into()),
        );
        assert!(!app.preset_edit_invalid());
    }

    #[test]
    fn preset_dialog_cancel_restores_prior_draft_and_ok_keeps_edits() {
        let mut app = app();
        app.switch_workspace(Workspace::Tools);
        send(&mut app, Message::PresetNameChanged("Prior draft".into()));
        let prior = app.tool_edit.clone();
        send(&mut app, Message::EditPreset);
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "750".into()),
        );
        send(&mut app, Message::CancelPresetEditor);
        assert_eq!(app.tool_edit, prior);
        assert!(!app.show_preset_editor);
        send(&mut app, Message::AddPreset);
        assert_eq!(app.tool_edit.as_ref().unwrap().presets.len(), 3);
        send(&mut app, Message::CancelPresetEditor);
        assert_eq!(app.tool_edit, prior);
        send(&mut app, Message::EditPreset);
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "750".into()),
        );
        send(&mut app, Message::ClosePresetEditor);
        assert!(!app.show_preset_editor);
        assert_eq!(app.tool_edit.as_ref().unwrap().presets[0].feed, 750.);
        assert_eq!(app.library.tools[0].presets[0].feed, 500.);
    }

    #[test]
    fn preset_modal_validates_ok_and_escape_cancels_dialog_edits() {
        let mut app = app();
        app.switch_workspace(Workspace::Tools);
        app.lib_sel = 0;
        app.reload_tool_edit();
        let saved = app.library.tools[0].clone();
        send(&mut app, Message::EditPreset);
        assert!(app.show_preset_editor);
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "invalid".into()),
        );
        send(&mut app, Message::ClosePresetEditor);
        assert!(app.show_preset_editor);
        send(
            &mut app,
            Message::PresetValueChanged(PresetField::Feed, "750".into()),
        );
        send(&mut app, Message::CadEscape);
        assert!(!app.show_preset_editor);
        assert_eq!(app.tool_edit.as_ref().unwrap(), &saved);
        assert_eq!(app.library.tools[0], saved);
        assert!(!app.preset_edit_invalid());
        send(
            &mut app,
            Message::PresetNameChanged("Pending tool edit".into()),
        );
        send(&mut app, Message::OpenMaterials);
        assert!(app.show_materials);
        send(&mut app, Message::CadEscape);
        assert!(!app.show_materials);
        assert!(app.inspector_dirty());
    }

    #[test]
    fn new_operation_entry_feed_defaults_follow_cutting_feeds() {
        let mut app = app();
        app.cad_pending_kind = Some(OpKind::Profile);
        app.cad_tool_pick = Some(1);
        app.controller.cancel_operation();
        app.seed_creation_editor(OpKind::Profile);
        for (field, source) in [
            (Field::RampFeed, "=plungeFeed"),
            (Field::LeadInFeed, "=feed"),
            (Field::LeadOutFeed, "=feed"),
        ] {
            assert_eq!(app.fields[&field], source);
        }
        send(&mut app, Message::FieldChanged(Field::Feed, "640".into()));
        send(
            &mut app,
            Message::FieldChanged(Field::PlungeFeed, "180".into()),
        );
        assert_eq!(app.parsed_field_value(Field::RampFeed).unwrap(), 180.);
        assert_eq!(app.parsed_field_value(Field::LeadInFeed).unwrap(), 640.);
        assert_eq!(app.parsed_field_value(Field::LeadOutFeed).unwrap(), 640.);
    }

    #[test]
    fn cad_creation_uses_the_same_material_match() {
        let mut app = app();
        app.cad_pending_kind = Some(OpKind::Profile);
        app.cad_tool_pick = Some(1);
        app.controller.cancel_operation();
        app.seed_creation_editor(OpKind::Profile);
        assert_eq!(
            app.operation_draft
                .as_ref()
                .unwrap()
                .tool_preset()
                .unwrap()
                .id,
            1
        );
        assert_eq!(app.fields[&Field::Feed], "500");
        send(
            &mut app,
            Message::OperationPresetPicked(PresetChoice {
                id: Some(2),
                name: "Preset 2".into(),
            }),
        );
        assert_eq!(app.fields[&Field::Feed], "200");
    }
    #[test]
    fn pocket_default_stepdown_formula_survives_automatic_preset() {
        let mut app = app();
        send(&mut app, Message::BeginOp(OpKind::Pocket));
        assert_eq!(
            app.fields[&Field::Stepdown],
            pocket_parameters::DEFAULT_STEPDOWN
        );
        send(
            &mut app,
            Message::OperationPresetPicked(PresetChoice {
                id: Some(2),
                name: "Preset 2".into(),
            }),
        );
        assert_eq!(app.fields[&Field::Stepdown], "0.5");
        assert!(cam_model::OperationParameter::Stepdown
            .expression(app.operation_draft.as_ref().unwrap())
            .is_none());
    }

    #[test]
    fn absent_optional_preset_values_preserve_manual_cutting_buffers() {
        let mut app = app();
        app.library.tools[0].presets[1].stepdown = None;
        app.library.tools[0].presets[1].overlap = None;
        send(&mut app, Message::BeginOp(OpKind::Pocket));
        assert!(app.creating_operation(), "{}", app.status);
        send(
            &mut app,
            Message::FieldChanged(Field::Stepdown, "0.25".into()),
        );
        send(&mut app, Message::FieldChanged(Field::Overlap, "55".into()));
        send(
            &mut app,
            Message::OperationPresetPicked(PresetChoice {
                id: Some(2),
                name: "Preset 2".into(),
            }),
        );
        assert_eq!(app.fields[&Field::Stepdown], "0.25");
        assert_eq!(app.fields[&Field::Overlap], "55");
        assert_eq!(app.fields[&Field::Feed], "200");
    }
    #[test]
    fn managed_materials_keep_ids_on_rename_and_protect_links_on_delete() {
        let mut app = app();
        app.new_material_name = "Wood".into();
        app.add_managed_material();
        let id = app.library.materials[0].id;
        app.rename_managed_material(id, "Hardwood".into());
        assert_eq!(
            app.library.materials[0],
            Material {
                id,
                name: "Hardwood".into(),
                specific_cutting_force: None,
            }
        );
        app.library.tools[0].presets[0].material_id = Some(id);
        app.delete_managed_material(id);
        assert_eq!(app.library.materials.len(), 1);
        app.library.tools[0].presets[0].material_id = None;
        app.delete_managed_material(id);
        assert!(app.library.materials.is_empty());
        app.new_material_name = "Brass".into();
        app.add_managed_material();
        assert!(app.library.materials[0].id > id);
        assert_eq!(
            serde_json::to_value(&app.library.materials[0])
                .unwrap()
                .as_object()
                .unwrap()
                .len(),
            2
        );
    }
    #[test]
    fn independent_entry_feeds_seed_from_presets_and_remain_editable_per_operation() {
        let mut app = app();
        app.library.tools[0].presets[0].entry_feeds = cam_model::EntryFeeds {
            ramp: Some(170.),
            lead_in: Some(210.),
            lead_out: Some(220.),
        };
        send(&mut app, Message::BeginOp(OpKind::Profile));
        for (field, expected) in [
            (Field::RampFeed, "170"),
            (Field::LeadInFeed, "210"),
            (Field::LeadOutFeed, "220"),
        ] {
            assert!(app.inspector_fields().contains(&field));
            assert_eq!(app.fields[&field], expected);
            assert_eq!(operation_tabs::field_tab(field), OperationTab::Tool);
        }
        app.controller
            .set_pending_operation_geometry(&[LoopRef::outer(0)], None)
            .unwrap();
        send(
            &mut app,
            Message::FieldChanged(Field::RampFeed, "180".into()),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::LeadInFeed, "0".into()),
        );
        send(
            &mut app,
            Message::FieldChanged(Field::LeadOutFeed, "230".into()),
        );
        send(&mut app, Message::ConfirmOp);
        let op = app.controller.inspection_setup().operations.last().unwrap();
        assert_eq!(
            *op.entry_feeds(),
            cam_model::EntryFeeds {
                ramp: Some(180.),
                lead_in: None,
                lead_out: Some(230.)
            }
        );
        assert_eq!(op.tool_preset().unwrap().entry_feeds.ramp, Some(170.));
    }
}
