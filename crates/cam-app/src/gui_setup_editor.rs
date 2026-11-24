//! Setup form with shared inspector controls and a persistent action bar.
use super::*;
use iced::widget::column;

fn setup_dimensions_row(label: &'static str, dimensions: [f64; 3]) -> Element<'static, Message> {
    row![
        text(label)
            .size(12)
            .color(INSPECTOR_TEXT)
            .width(Length::Fill),
        text(format!(
            "{:.3} × {:.3} × {:.3} mm",
            dimensions[0], dimensions[1], dimensions[2]
        ))
        .size(11)
        .color(INSPECTOR_MUTED),
    ]
    .spacing(8)
    .align_y(Alignment::Center)
    .into()
}

impl App {
    fn setup_geometry_list(&self, draft: &cam_model::Setup, fixture: bool) -> Element<'_, Message> {
        let instances = if fixture {
            &draft.fixture_instances
        } else {
            &draft.model_instances
        };
        let picking = self.setup_model_pick && self.setup_fixture_pick == fixture;
        let label = match instances.len() {
            0 => "Select".to_string(),
            1 => "Body".to_string(),
            count => format!("{count} Bodies"),
        };
        let pick = if fixture {
            Message::SetupFixturePick
        } else {
            Message::SetupModelPick
        };
        let button = button(
            row![
                iced::widget::svg(iced::widget::svg::Handle::from_memory(include_bytes!(
                    "../assets/icons/point-selection.svg"
                )))
                .width(18)
                .height(18),
                text(label).size(12),
            ]
            .spacing(6)
            .align_y(Alignment::Center),
        )
        .padding([6, 8])
        .height(32)
        .style(move |theme, status| inspector_pick_style(theme, status, picking))
        .on_press_maybe(self.controller.has_geometry().then_some(pick));
        let mut form = column![row![
            text(if fixture { "Fixture" } else { "Model" })
                .size(12)
                .color(INSPECTOR_TEXT)
                .width(Length::Fill),
            button,
            inspector_button("×")
                .width(28)
                .height(32)
                .padding(0)
                .style(button::text)
                .on_press_maybe(
                    (!instances.is_empty()).then_some(Message::SetupGeometryClear(fixture))
                ),
        ]
        .spacing(6)
        .align_y(Alignment::Center)];
        if picking {
            form = form.push(
                text("Click bodies in the view to select or deselect them. Click the button when finished.")
                    .size(11)
                    .color(INSPECTOR_MUTED),
            );
        } else if !self.controller.has_geometry() {
            form = form.push(
                text("Import a model to add setup geometry.")
                    .size(11)
                    .color(INSPECTOR_MUTED),
            );
        }
        form.spacing(6).into()
    }

    pub(super) fn setup_editor(&self) -> Element<'_, Message> {
        let Some(draft) = &self.setup_draft else {
            return column![
                inspector_heading("Setup"),
                text("Double-click a setup, or right-click it and choose Edit setup, to edit its settings.").size(12)
            ]
            .spacing(8)
            .padding(14)
            .into();
        };
        let tabs = row(SetupEditorTab::ALL.into_iter().map(|tab| {
            let active = tab == self.setup_tab;
            inspector_button(tab.to_string())
                .width(Length::FillPortion(if tab == SetupEditorTab::Postprocess {
                    3
                } else {
                    2
                }))
                .padding([8, 4])
                .style(move |theme, status| inspector_tab(theme, status, active))
                .on_press(Message::SetupTabChanged(tab))
                .into()
        }))
        .spacing(4);
        let header = container(
            column![
                text(format!("SETUP {}", draft.id))
                    .size(10)
                    .color(INSPECTOR_MUTED),
                inspector_heading(draft.name.clone()),
                tabs,
            ]
            .spacing(10),
        )
        .padding(14)
        .width(Length::Fill);
        // Leave a gutter for the scrollbar so fields never touch the panel edge.
        let mut list = column![]
            .spacing(8)
            .padding(Padding {
                top: 0.0,
                right: 18.0,
                bottom: 16.0,
                left: 14.0,
            })
            .width(Length::Fill);
        match self.setup_tab {
            SetupEditorTab::Setup => {
                list = list.push(inspector_section("General"));
                list = list.push(text("Setup name").size(12).color(INSPECTOR_TEXT));
                list = list.push(
                    inspector_input("Setup name", &draft.name).on_input(Message::SetupNameChanged),
                );
                list = list.push(inspector_section("Model"));
                list = list.push(self.setup_geometry_list(draft, false));
                list = list.push(self.setup_wcs_editor(draft));
                list = list.push(inspector_section("Fixture"));
                list = list.push(self.setup_geometry_list(draft, true));
                for (axis, label, clearance) in [
                    (0, "Radial clearance", draft.fixture_clearance.radial),
                    (1, "Axial clearance", draft.fixture_clearance.axial),
                ] {
                    let value = self.setup_value(42 + axis, fmt_num(clearance));
                    list = list.push(inspector_value(label, &value, "mm", move |v| {
                        Message::SetupFixtureClearanceChanged(axis, v)
                    }));
                }
                list = list.push(text("Clearance protects conservative fixture body boxes, including holes and recesses, against the cutter and shank. Selecting a fixture removes it from machining models.").size(11).color(INSPECTOR_MUTED));
                if let Err(reason) = draft.validate_fixtures(self.controller.geometry_state()) {
                    list = list.push(text(reason).size(11).color(palette::ERROR));
                }
                list = list.push(inspector_section("Tool change"));
                list = list.push(
                    checkbox(draft.tool_change_height.is_none())
                        .label("Use machine maximum Z")
                        .text_size(12)
                        .on_toggle(Message::SetupToolChangeAutomatic),
                );
                let height = draft
                    .resolved_tool_change_height(self.controller.machine().envelope.max.z)
                    - draft.origin[2];
                if draft.tool_change_height.is_some() {
                    let value = self.setup_value(41, fmt_num(height));
                    list = list.push(inspector_value(
                        "Tool-change height",
                        &value,
                        "mm",
                        Message::SetupToolChangeHeightChanged,
                    ));
                } else {
                    list = list.push(
                        text(format!("Tool-change height: {} mm", fmt_num(height)))
                            .size(12)
                            .color(INSPECTOR_TEXT),
                    );
                    list = list.push(text("Uses the machine's configured maximum Z relative to the base WCS; verify it for this setup. Z travel alone does not determine this height.").size(11).color(INSPECTOR_MUTED));
                }
                list = list.push(text(
                    "Z relative to the base WCS origin. Used before tool changes and re-fixturing; must be at or above all operation clearance planes."
                ).size(11).color(INSPECTOR_MUTED));
            }
            SetupEditorTab::Stock => {
                list = list.push(inspector_section("Material"));
                list = list.push(self.stock_material_picker(draft));
                list = list.push(inspector_section("Stock definition"));
                let kind = match draft.stock {
                    cam_model::Stock::FixedBox { .. } => StockEditorKind::FixedBox,
                    cam_model::Stock::RelativeBox { .. } => StockEditorKind::RelativeBox,
                    cam_model::Stock::BoundingBox { .. } => StockEditorKind::LegacyBox,
                    cam_model::Stock::FromSolid { .. } => StockEditorKind::FromSolid,
                    cam_model::Stock::FixedRound { tube: false, .. } => {
                        StockEditorKind::FixedCylinder
                    }
                    cam_model::Stock::FixedRound { tube: true, .. } => StockEditorKind::FixedTube,
                    cam_model::Stock::RelativeRound { tube: false, .. } => {
                        StockEditorKind::RelativeCylinder
                    }
                    cam_model::Stock::RelativeRound { tube: true, .. } => {
                        StockEditorKind::RelativeTube
                    }
                    cam_model::Stock::PreviousSetup => StockEditorKind::PreviousSetup,
                };
                let count = if self
                    .controller
                    .setups()
                    .first()
                    .is_some_and(|s| s.id != draft.id)
                {
                    StockEditorKind::ALL.len()
                } else {
                    StockEditorKind::ALL.len() - 1
                };
                list = list.push(icon_picker::picker(
                    iced::advanced::widget::Id::from("stock-type-menu"),
                    StockEditorKind::ALL[..count].to_vec(),
                    kind,
                    |entry| {
                        row![
                            iced::widget::svg(iced::widget::svg::Handle::from_memory(entry.icon()))
                                .width(22)
                                .height(22),
                            text(entry.to_string()).size(12).width(Length::Fill),
                        ]
                        .spacing(8)
                        .align_y(Alignment::Center)
                        .into()
                    },
                    |_| 0,
                    Message::SetupStockKindChanged,
                ));
                if let cam_model::Stock::FixedRound { axis, .. }
                | cam_model::Stock::RelativeRound { axis, .. } = draft.stock
                {
                    let picking = self.setup_axis_pick == Some(3);
                    list = list.push(
                        checkbox(axis.is_some() || picking)
                            .label("Select axis")
                            .text_size(12)
                            .on_toggle(|on| {
                                if on {
                                    Message::SetupAxisPick(3)
                                } else {
                                    Message::SetupStockAxisClear
                                }
                            }),
                    );
                    if axis.is_some() || picking {
                        let label = if picking {
                            "Select an axis…".to_string()
                        } else {
                            match axis.unwrap() {
                                cam_model::StockAxis::Edge(edge) => {
                                    format!("Edge {}", edge.edge_id)
                                }
                                cam_model::StockAxis::Origin { origin_axis } => {
                                    format!("Origin {}", ["X", "Y", "Z"][origin_axis.index()])
                                }
                            }
                        };
                        list = list.push(
                            row![
                                text("Axis").size(12).color(INSPECTOR_TEXT).width(80),
                                inspector_pick_button(label, picking)
                                    .width(Length::Fill)
                                    .on_press(Message::SetupAxisPick(3)),
                                inspector_button("×").on_press(Message::SetupStockAxisClear),
                            ]
                            .spacing(8)
                            .align_y(Alignment::Center),
                        );
                        if picking {
                            list = list.push(
                                text(
                                    "Choose an origin axis or select a straight edge in the view.",
                                )
                                .size(11)
                                .color(INSPECTOR_MUTED),
                            );
                            list = list.push(
                                row![
                                    inspector_button("Origin X")
                                        .on_press(Message::SetupStockOriginAxis(0))
                                        .width(Length::Fill),
                                    inspector_button("Origin Y")
                                        .on_press(Message::SetupStockOriginAxis(1))
                                        .width(Length::Fill),
                                    inspector_button("Origin Z")
                                        .on_press(Message::SetupStockOriginAxis(2))
                                        .width(Length::Fill),
                                ]
                                .spacing(4),
                            );
                        }
                        if picking && self.cad_candidates.len() > 1 {
                            list = list.push(
                                text("Overlapping edge candidates")
                                    .size(11)
                                    .color(INSPECTOR_MUTED),
                            );
                            for candidate in &self.cad_candidates {
                                list = list.push(
                                    inspector_button(format!(
                                        "Edge {} · instance {}{}",
                                        candidate.entity.id,
                                        candidate.entity.instance_id,
                                        if candidate.occluded { " · behind" } else { "" }
                                    ))
                                    .on_press(Message::CadSelect(candidate.entity.clone())),
                                );
                            }
                        }
                    }
                }
                match draft.stock {
                    cam_model::Stock::FromSolid { instance } => {
                        let choices: Vec<_> = self
                            .controller
                            .geometry_state()
                            .instances
                            .iter()
                            .filter(|i| {
                                self.controller
                                    .geometry_state()
                                    .asset(i.asset_id)
                                    .and_then(|a| {
                                        a.topology.bodies.iter().find(|b| b.id == i.body_id)
                                    })
                                    .is_some_and(|b| b.valid_solid)
                            })
                            .map(|i| StockSolidChoice {
                                id: i.id,
                                name: i.name.clone(),
                            })
                            .collect();
                        let selected = choices.iter().find(|c| Some(c.id) == instance).cloned();
                        list = list.push(text("Stock solid").size(12).color(INSPECTOR_TEXT));
                        list = list.push(
                            pick_list(choices, selected, Message::SetupStockSolidChanged)
                                .placeholder("Select a solid")
                                .text_size(12)
                                .padding([7, 9])
                                .style(inspector_picker)
                                .width(Length::Fill),
                        );
                    }
                    cam_model::Stock::PreviousSetup => {
                        let previous = self
                            .controller
                            .setups()
                            .iter()
                            .take_while(|s| s.id != draft.id)
                            .last();
                        list = list.push(text(previous.map_or_else(
                            || "The first setup needs box stock.".into(),
                            |s| format!("Uses the remaining material after {}. Included operations are recalculated when generating or simulating.", s.name),
                        )).size(11).color(INSPECTOR_MUTED));
                    }
                    cam_model::Stock::FixedBox {
                        size,
                        model_position,
                    } => {
                        list = list.push(inspector_section("Size and placement"));
                        for i in 0..3 {
                            let value = self.setup_value(20 + i, fmt_num(size[i]));
                            list = list.push(inspector_value(
                                format!("{} size", ["X", "Y", "Z"][i]),
                                &value,
                                "mm",
                                move |v| Message::SetupStockNumberChanged(i, v),
                            ));
                            list = list.push(
                                row![
                                    text("Model position")
                                        .size(11)
                                        .color(INSPECTOR_MUTED)
                                        .width(Length::Fill),
                                    pick_list(
                                        &BoxAnchor::ALL[..],
                                        Some(BoxAnchor::of(model_position[i])),
                                        move |anchor| Message::SetupStockPositionChanged(i, anchor)
                                    )
                                    .text_size(12)
                                    .padding([7, 9])
                                    .style(inspector_picker)
                                    .width(120),
                                ]
                                .spacing(8)
                                .align_y(Alignment::Center),
                            );
                        }
                    }
                    cam_model::Stock::RelativeBox { offsets } => {
                        list = list.push(inspector_section("Stock offsets"));
                        for (side, side_offsets) in offsets.iter().enumerate() {
                            for (axis, offset) in side_offsets.iter().enumerate() {
                                let i = side * 3 + axis;
                                let value = self.setup_value(20 + i, fmt_num(*offset));
                                list = list.push(inspector_value(
                                    format!(
                                        "{} {}",
                                        if side == 0 { "−" } else { "+" },
                                        ["X", "Y", "Z"][axis]
                                    ),
                                    &value,
                                    "mm",
                                    move |v| Message::SetupStockNumberChanged(i, v),
                                ));
                            }
                        }
                    }
                    cam_model::Stock::FixedRound {
                        diameter,
                        inner_diameter,
                        length,
                        position,
                        tube,
                        ..
                    } => {
                        list = list.push(inspector_section("Size and placement"));
                        for (label, index, number, unit) in [
                            ("Outer diameter", 0, diameter, "mm"),
                            ("Inner diameter", 1, inner_diameter, "mm"),
                            ("Length", 2, length, "mm"),
                            ("Model position (0–1)", 3, position, ""),
                        ] {
                            if index == 1 && !tube {
                                continue;
                            }
                            let value = self.setup_value(20 + index, fmt_num(number));
                            list = list.push(inspector_value(label, &value, unit, move |v| {
                                Message::SetupStockNumberChanged(index, v)
                            }));
                        }
                    }
                    cam_model::Stock::RelativeRound {
                        radial,
                        inner_diameter,
                        bottom,
                        top,
                        tube,
                        ..
                    } => {
                        list = list.push(inspector_section("Stock allowance"));
                        for (label, index, number) in [
                            ("Radial allowance", 0, radial),
                            ("Inner diameter", 1, inner_diameter),
                            ("Bottom allowance", 2, bottom),
                            ("Top allowance", 3, top),
                        ] {
                            if index == 1 && !tube {
                                continue;
                            }
                            let value = self.setup_value(20 + index, fmt_num(number));
                            list = list.push(inspector_value(label, &value, "mm", move |v| {
                                Message::SetupStockNumberChanged(index, v)
                            }));
                        }
                    }
                    cam_model::Stock::BoundingBox {
                        x_offset,
                        y_offset,
                        top,
                        thickness,
                    } => {
                        list = list.push(inspector_section("Stock dimensions"));
                        for (label, key, value) in [
                            ("X allowance", 50, x_offset),
                            ("Y allowance", 51, y_offset),
                            ("Top Z", 52, top),
                            ("Thickness", 53, thickness),
                        ] {
                            let value = self.setup_value(20 + key, fmt_num(value));
                            list = list.push(inspector_value(label, &value, "mm", move |v| {
                                Message::SetupStockNumberChanged(key, v)
                            }));
                        }
                    }
                }
                if matches!(
                    draft.stock,
                    cam_model::Stock::FixedRound { tube: true, .. }
                        | cam_model::Stock::RelativeRound { tube: true, .. }
                        | cam_model::Stock::FromSolid { .. }
                ) {
                    list = list.push(text("Check that the selected stock encloses the model; bounding boxes do not verify internal cavities.").size(11).color(INSPECTOR_MUTED));
                }
                let (model_min, model_max) = self.controller.pending_setup_model_bounds(draft);
                let dimensions = std::array::from_fn::<_, 3, _>(|i| model_max[i] - model_min[i]);
                let (stock_min, stock_max) = self.controller.pending_setup_stock_bounds(draft);
                list = list.push(inspector_section("Information"));
                list = list.push(setup_dimensions_row("Model bound", dimensions));
                if !matches!(
                    draft.stock,
                    cam_model::Stock::FixedBox { .. } | cam_model::Stock::FixedRound { .. }
                ) {
                    let stock_dimensions = std::array::from_fn(|i| stock_max[i] - stock_min[i]);
                    list = list.push(setup_dimensions_row("Stock size", stock_dimensions));
                }
                if draft.stock != cam_model::Stock::PreviousSetup
                    && (0..3).any(|i| {
                        stock_min[i] > model_min[i] + 1e-6 || stock_max[i] < model_max[i] - 1e-6
                    })
                {
                    let (message, color) = match draft.stock {
                        cam_model::Stock::RelativeBox { .. } => {
                            ("Part of the model lies outside the stock.", INSPECTOR_MUTED)
                        }
                        _ => (
                            "Stock does not contain the selected machining model.",
                            palette::ERROR,
                        ),
                    };
                    list = list.push(text(message).size(11).color(color));
                }
            }
            SetupEditorTab::Postprocess => {
                list = list.push(inspector_section("Program output"));
                let offset = self.setup_value(40, draft.origin_index.to_string());
                list = list.push(inspector_value(
                    "Work offset",
                    &offset,
                    "",
                    Message::SetupWorkOffsetChanged,
                ));
                list = list.push(text("Program name / number").size(12).color(INSPECTOR_TEXT));
                list = list.push(
                    inspector_input("Setup program", &draft.program_name)
                        .on_input(Message::SetupProgramChanged),
                );
                list = list.push(
                    text("Each physical setup exports as a separate NC program.")
                        .size(11)
                        .color(INSPECTOR_MUTED),
                );
            }
        }
        let numbers_valid = self
            .setup_values
            .values()
            .all(|v| v.trim().parse::<f64>().is_ok_and(|n| n.is_finite()))
            && self
                .setup_values
                .get(&40)
                .is_none_or(|v| v.trim().parse::<u32>().is_ok());
        let can_commit = numbers_valid
            && self.setup_frame_issue(draft).is_none()
            && draft
                .validate_fixtures(self.controller.geometry_state())
                .is_ok();
        let footer = inspector_footer(
            row![
                inspector_button("Cancel").on_press(Message::SetupCancel),
                Space::new().width(Length::Fill),
                inspector_button("Apply setup")
                    .style(inspector_primary)
                    .on_press_maybe(can_commit.then_some(Message::SetupCommit)),
            ]
            .spacing(8)
            .align_y(Alignment::Center),
        );
        column![
            header,
            scrollable(list).width(Length::Fill).height(Length::Fill),
            footer
        ]
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
    }
}
