//! Transactional setup/tool/operation inspector view.
use super::*;
use iced::widget::column;

const ESTLCAM_OUTPUT_CHOICES: &[&str] = &[
    "None",
    "Output 1 (M8 / M9)",
    "Output 2 (M10 / M11)",
    "Both outputs",
];

impl App {
    pub(super) fn inspector(&self) -> Element<'_, Message> {
        if let Some(draft) = &self.scale_draft {
            return self.scale_inspector(draft);
        }
        if self.move_draft.is_some() {
            return self.move_inspector();
        }
        if self.pattern_draft.is_some() {
            return self.pattern_editor();
        }
        if self.library_mode() {
            return self.library_editor();
        }
        if let Some(editor) = &self.profile_entry_editor {
            return self.profile_entry_settings(editor);
        }
        if self.geometry_editor.is_some() {
            return self.operation_geometry_settings();
        }
        if self.drawing_editing_op.is_some() {
            return self.drawing_geometry_picker();
        }
        if self.creating_operation() {
            return self.operation_inspector(0);
        }
        if self.cad_inspecting || self.cad_editing_op.is_some() {
            return self.cad_inspector();
        }
        if self.controller.selection() == Selection::Setup {
            return self.setup_editor();
        }
        if let Selection::Operation(id) = self.controller.selection() {
            return self.operation_inspector(id);
        }
        let heading = match self.controller.selection() {
            Selection::Setup => "Setup".to_string(),
            Selection::Origin => {
                let index = self.controller.active_origin();
                match self.controller.datum_label(index) {
                    Some(d) => format!("Workpiece origin {index} · {d}"),
                    None => format!("Workpiece origin {index} · no work offset"),
                }
            }
            Selection::Stock => "Stock".to_string(),
            Selection::Machine => "Machine".to_string(),
            Selection::Tool(i) => format!("Tool {}", i + 1),
            Selection::Operation(id) => match self.current_operation(id) {
                Some(op) => format!("Operation {id} — {}", op_kind(op)),
                None => "Operation".to_string(),
            },
        };

        let mut list = column![inspector_heading(heading)].spacing(10).padding(14);
        if let Selection::Stock = self.controller.selection() {
            // The concrete block the offsets/heights resolve to, as read-only
            // context above the editable fields.
            let (min, max) = self.controller.stock_box();
            list = list.push(
                text(format!(
                    "Resolved box\n  X {:.1}…{:.1}   Y {:.1}…{:.1}   Z {:.1}…{:.1}",
                    min[0], max[0], min[1], max[1], min[2], max[2]
                ))
                .size(11)
                .color(palette::GROUP_LABEL),
            );
        }

        if let Selection::Machine = self.controller.selection() {
            // Which machine is active — above its own fields, because everything below
            // edits *this* one. The library is local: a machine is a property of your
            // shop, and a project file records the one it was built for as provenance
            // only (it can never set yours, or it could disarm the travel check).
            list = list.push(
                row![
                    label_help("Active", help::ACTIVE_MACHINE, self.tooltips),
                    pick_list(
                        self.machines.names(),
                        Some(self.active_machine.clone()),
                        Message::ActiveMachineChanged,
                    )
                    .text_size(12)
                    .padding([7, 9])
                    .style(inspector_picker)
                    .width(INSPECTOR_PICKER_W),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
            // Machine name (a free-text tag), above the travel fields. Committed on
            // change so multiple machines can be told apart later.
            list = list.push(
                row![
                    label_help("Name", help::MACHINE_NAME, self.tooltips),
                    inspector_input(
                        "machine",
                        self.machine_name_edit
                            .as_deref()
                            .unwrap_or(&self.controller.machine().name),
                    )
                    .on_input(Message::MachineNameChanged)
                    .width(Length::Fixed(120.0)),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
            // The post/controller dialect used on export.
            list = list.push(profile_picker_w(
                "Post",
                help::POST,
                self.tooltips,
                self.machine_post_edit
                    .unwrap_or(self.controller.post_kind()),
                &PostKind::ALL[..],
                Message::PostKindChanged,
                INSPECTOR_PICKER_W,
            ));
            if self
                .machine_post_edit
                .unwrap_or(self.controller.post_kind())
                == PostKind::Estlcam
            {
                let options = self
                    .machine_estlcam_edit
                    .unwrap_or(self.controller.estlcam_options());
                let selected_output = match (options.output_1, options.output_2) {
                    (false, false) => "None",
                    (true, false) => "Output 1 (M8 / M9)",
                    (false, true) => "Output 2 (M10 / M11)",
                    (true, true) => "Both outputs",
                };
                list = list.push(text("Coolant / MMS outputs").size(12)).push(
                    row![
                        text("Cooling output").size(12).width(Length::Fill),
                        pick_list(ESTLCAM_OUTPUT_CHOICES, Some(selected_output), |choice| {
                            Message::EstlcamOutputSelectionChanged(match choice {
                                "Output 1 (M8 / M9)" => 1,
                                "Output 2 (M10 / M11)" => 2,
                                "Both outputs" => 3,
                                _ => 0,
                            })
                        })
                        .text_size(12)
                        .padding([7, 9])
                        .style(inspector_picker)
                        .width(Length::Fill),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            }
        }

        let ordered = self.inspector_fields();
        if ordered.is_empty() {
            list = list.push(text("Nothing to edit here yet.").size(12));
        }
        for field in ordered {
            if matches!(self.controller.selection(),Selection::Operation(id) if self.controller.geometry_state().bindings.contains_key(&id))
                && matches!(
                    field,
                    Field::Depth | Field::DrillStartOffset | Field::FaceStartOffset
                )
            {
                continue;
            }
            // Clearing-pass fields render in their own section (below), not mixed in here.
            if is_clear_field(field) {
                continue;
            }
            let value = self.fields.get(&field).cloned().unwrap_or_default();
            list = list.push(self.parameter_field_row(
                field,
                self.field_label(field),
                self.field_help(field),
                &value,
            ));
        }
        // A non-base origin is a reorientation: note the operator stop so the part can be
        // re-fixtured. The `H index` / position fields render in the generic loop above.
        if let Selection::Origin = self.controller.selection() {
            if self.controller.active_origin() != self.controller.base_origin_index() {
                list = list.push(
                    text(
                        "Teach this G15 H<n> on the control to the point set here. A stop \
                         (M00) precedes this group so you can re-fixture the part.",
                    )
                    .size(11)
                    .color(palette::GROUP_LABEL),
                );
            }
        }
        // The tool geometry class is an enum, so it gets a picker (committed
        // immediately) rather than a text field.
        if let Selection::Tool(i) = self.controller.selection() {
            if let Some(tool) = self.controller.inspection_setup().tools.get(i) {
                list = list.push(
                    row![
                        help_wrap(
                            text("Type").width(Length::Fill).size(12),
                            help::TOOL_TYPE,
                            self.tooltips,
                        ),
                        pick_list(
                            &ToolKindPick::ALL[..],
                            Some(ToolKindPick::of(tool.kind)),
                            |p| Message::ToolKindChanged(p.to_kind())
                        )
                        .text_size(12)
                        .padding([7, 9])
                        .style(inspector_picker)
                        .width(Length::Fixed(177.0)),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            }
        }
        let footer = inspector_footer(
            column![
                row![
                    Space::new().width(Length::Fill),
                    inspector_button("Apply changes")
                        .style(inspector_primary)
                        .on_press_maybe(
                            (!self.any_field_invalid() && self.inspector_dirty())
                                .then_some(Message::Apply)
                        ),
                ],
                text("Typed values apply together. Lists and checkboxes update immediately.")
                    .size(11)
                    .color(INSPECTOR_MUTED),
            ]
            .spacing(8),
        );
        column![
            scrollable(list).width(Length::Fill).height(Length::Fill),
            footer
        ]
        .height(Length::Fill)
        .into()
    }
}
