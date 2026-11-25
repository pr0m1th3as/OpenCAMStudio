//! Transactional reference/offset controls shared by every operation kind.
use super::*;
use cam_model::{
    HeightPlaneKind as Plane, HeightReference as Reference, OperationHeightSettings,
    ResolvedOperationHeights,
};
use iced::widget::column;

pub(super) fn plane_input_id(plane: Plane) -> String {
    format!("plane-offset-{plane:?}")
}

impl App {
    /// Show Z relative to this operation's workpiece datum, matching NC output.
    /// Keep entered precision; model-space plane positions and offsets stay exact.
    pub(super) fn height_display_value(&self, id: u32, plane: Plane, value: f64) -> String {
        let work_offset = self
            .current_operation(id)
            .map_or(self.controller.active_origin(), Operation::work_offset);
        let value = if let Some(wrap) = self.current_operation(id).and_then(Operation::rotary_wrap)
        {
            let offset = self
                .fields
                .get(&Field::Wrap(super::rotary_view::Number::Offset))
                .and_then(|s| s.parse::<f64>().ok())
                .unwrap_or(wrap.offset);
            wrap.radius + offset + value - wrap.surface_z
        } else {
            value
                - self
                    .controller
                    .inspection_setup()
                    .origin_position(work_offset)[2]
        };
        fn precision(text: &str) -> usize {
            let (mantissa, exponent) = text
                .trim()
                .split_once(['e', 'E'])
                .map_or((text.trim(), 0), |(m, e)| {
                    (m, e.parse::<i32>().unwrap_or(0))
                });
            let decimals = mantissa.split_once('.').map_or(0, |(_, tail)| tail.len());
            (decimals as i32).saturating_sub(exponent).clamp(0, 15) as usize
        }
        fn entered(app: &App, plane: Plane, visited: &mut BTreeSet<Plane>) -> usize {
            if !visited.insert(plane) {
                return 0;
            }
            let own = app.height_offsets.get(&plane).map_or(0, |s| precision(s));
            let Some(settings) = app.height_draft.as_ref() else {
                return own;
            };
            let definition = settings.definition(plane);
            let sources: Vec<_> = match definition.reference {
                Reference::Plane(source) => vec![source],
                Reference::Highest | Reference::Lowest => definition
                    .sources
                    .iter()
                    .filter_map(|r| {
                        if let Reference::Plane(p) = r {
                            Some(*p)
                        } else {
                            None
                        }
                    })
                    .collect(),
                _ => vec![],
            };
            sources.into_iter().fold(own, |digits, source| {
                digits.max(entered(app, source, visited))
            })
        }
        let digits = entered(self, plane, &mut BTreeSet::new());
        let mut label = format!("{value:.precision$}", precision = digits.max(3));
        while label.contains('.')
            && label.ends_with('0')
            && label.split_once('.').unwrap().1.len() > digits
        {
            label.pop();
        }
        if label.ends_with('.') {
            label.pop();
        }
        if label.starts_with('-') && label.parse::<f64>() == Ok(0.) {
            label.remove(0);
        }
        label
    }

    pub(super) fn seed_height_editor(&mut self, id: u32) {
        self.height_draft_id = Some(id);
        self.height_disabled_feed_reference = None;
        self.height_reference_pick = None;
        let Some(op) = self.controller.operation(id) else {
            return;
        };
        let settings = self
            .controller
            .inspection_setup()
            .height_settings
            .get(&id)
            .cloned()
            .unwrap_or_else(|| OperationHeightSettings::legacy(self.controller.document(), op));
        self.height_offsets = Plane::ALL
            .into_iter()
            .map(|p| (p, fmt_num(settings.definition(p).offset)))
            .collect();
        self.height_draft = Some(settings);
    }
    pub(super) fn height_rules_dirty(&self) -> bool {
        let (Some(id), Some(draft)) = (self.height_draft_id, self.height_draft.as_ref()) else {
            return false;
        };
        if self.controller.selection() != Selection::Operation(id) {
            return false;
        }
        let Some(op) = self.controller.operation(id) else {
            return false;
        };
        let baseline = self
            .controller
            .inspection_setup()
            .height_settings
            .get(&id)
            .cloned()
            .unwrap_or_else(|| OperationHeightSettings::legacy(self.controller.document(), op));
        draft != &baseline
            || Plane::ALL.into_iter().any(|p| {
                self.height_offsets.get(&p).is_some_and(|s| {
                    s.trim().parse::<f64>().ok() != Some(baseline.definition(p).offset)
                })
            })
    }
    pub(super) fn edited_height_settings(&self) -> Result<OperationHeightSettings, String> {
        let mut settings = self
            .height_draft
            .clone()
            .ok_or("Open an operation to edit its heights")?;
        settings.resolved = None;
        for plane in Plane::ALL {
            if settings.definition(plane).reference == Reference::Disabled {
                continue;
            }
            let value = self
                .height_offsets
                .get(&plane)
                .and_then(|s| s.trim().parse::<f64>().ok())
                .filter(|v| v.is_finite())
                .ok_or_else(|| format!("{plane}: offset must be a finite number"))?;
            settings.definition_mut(plane).offset = value;
        }
        settings.validate_structure()?;
        Ok(settings)
    }
    pub(super) fn resolve_height_editor(
        &self,
        id: u32,
    ) -> Result<ResolvedOperationHeights, String> {
        let mut document = self.rotary_draft_document();
        for tool in &self.operation_tools {
            if !document
                .setup()
                .tools
                .iter()
                .any(|saved| saved.number == tool.number)
            {
                document.setup_mut().tools.push(tool.clone());
            }
        }
        let mut op = if self.creating_operation() {
            let (op, binding) = self.creation_operation()?;
            if let Some(binding) = binding {
                document.geometry.bindings.insert(id, binding);
            }
            op
        } else {
            self.current_operation(id)
                .cloned()
                .ok_or("Operation unavailable")?
        };
        let parsed = self.parsed_operation_fields();
        apply_op_fields(&mut op, &parsed);
        if op.rotary_wrap().is_some() {
            let geometry = crate::cad_operations::geometry_for_setup(&document)?;
            crate::cad_operations::resolve_wrapped_operation(&geometry, &mut op)?;
            return self.edited_height_settings()?.resolve(&document, &op);
        }
        if let Operation::Profile(profile) = &mut op {
            if profile.selections.is_some() {
                if let Some(mut binding) =
                    crate::cad_operations::resolve_profile_entries(&document, profile)?
                {
                    if let Some(previous) = document.geometry.bindings.get(&id) {
                        binding.overrides = previous.overrides.clone();
                    }
                    document.geometry.bindings.insert(id, binding);
                } else {
                    document.geometry.bindings.remove(&id);
                }
            } else if let Some(binding) = &self.operation_binding_preview {
                document.geometry.bindings.insert(id, binding.clone());
            }
        } else if let Some(binding) = &self.operation_binding_preview {
            document.geometry.bindings.insert(id, binding.clone());
        }
        let settings = self.edited_height_settings()?;
        if settings.uses_hole_references() {
            return crate::cad_operations::resolve_height_settings(&document, &op, &settings);
        }
        if document.rotary_indexes.contains_key(&id)
            && !matches!(&op, Operation::Profile(p) if p.selections.is_some())
        {
            if let Some(mut binding) = document.geometry.bindings.get(&id).cloned() {
                if crate::cad_operations::surface_recipe(&binding.recipe) {
                    let cutting = document.operation_base_cutting_heights(&op);
                    binding.heights.bottom = binding.heights.top - cutting.depth();
                }
                if matches!(binding.recipe, cam_model::ExtractionRecipe::FaceRegion) {
                    if let Some(view) = document.indexed_view(id)? {
                        binding.heights.top = view.stock_bounds_for(view.setup()).1[2];
                    }
                }
                let geometry = crate::cad_operations::geometry_for_operation(&document, id)?;
                binding.heights =
                    cam_geometry3d::resolve_binding_advisory(&geometry, &binding)?.heights;
                binding.overrides = cam_model::GeometryOverrides::default();
                document.geometry.bindings.insert(id, binding);
            }
        }
        settings.resolve(&document, &op)
    }
    pub(super) fn height_reference_selected(&mut self, plane: Plane, reference: Reference) {
        if let Some(settings) = &mut self.height_draft {
            let d = settings.definition_mut(plane);
            if plane == Plane::Feed
                && reference == Reference::Disabled
                && d.reference != Reference::Disabled
            {
                self.height_disabled_feed_reference = Some(d.reference);
            }
            d.reference = reference;
            if matches!(reference, Reference::Highest | Reference::Lowest) && d.sources.is_empty() {
                d.sources = vec![Reference::StockTop, Reference::ModelTop];
            }
        }
        self.focused_height = (reference != Reference::Disabled).then(|| plane_input_id(plane));
        self.height_reference_pick = None;
    }
    pub(super) fn height_geometry_selected(&mut self, entity: EntityRef) {
        if let (Some(plane), Some(settings)) = (self.height_reference_pick, &mut self.height_draft)
        {
            let d = settings.definition_mut(plane);
            if let Some(index) = d.entities.iter().position(|r| r == &entity) {
                d.entities.remove(index);
            } else {
                d.entities.push(entity);
            }
            self.focused_height = Some(plane_input_id(plane));
            self.status =
                "Height reference selected. Pick more geometry or choose Done picking.".into();
        }
    }
    pub(super) fn reference_height_controls(&self, id: u32) -> Element<'_, Message> {
        let Some(settings) = self.height_draft.as_ref() else {
            return text("Open the operation editor to set heights").into();
        };
        let resolved = self.resolve_height_editor(id);
        let operation = self.current_operation(id);
        let mut controls = column![row![
            help_wrap(
                inspector_section_label("Reference planes and offsets").width(Length::Fill),
                "Offsets are in mm along setup Z. Planes are shown while Heights is selected.",
                self.tooltips,
            ),
            inspector_button("Use defaults").on_press(Message::HeightDefaults),
        ]
        .spacing(8),]
        .spacing(10);
        for plane in Plane::ALL {
            let d = settings.definition(plane);
            let enabled = d.reference != Reference::Disabled;
            let collapsed = !enabled || self.collapsed_heights.contains(&plane);
            let toggle = (plane == Plane::Feed).then(|| {
                checkbox(enabled)
                    .size(14)
                    .on_toggle(Message::HeightFeedEnabled)
                    .into()
            });
            let heading = inspector_disclosure_heading(
                &plane.to_string(),
                collapsed,
                Message::HeightSectionToggle(plane),
                toggle,
            );
            let heading = help_wrap(heading, illustrated_help::height_help(plane), self.tooltips);
            let mut section = column![heading].spacing(10);
            if !collapsed {
                section = section.push(
                    row![
                        help_wrap(
                            text("From").size(12).width(70),
                            illustrated_help::height_help(plane),
                            self.tooltips
                        ),
                        super::height_reference::picker(plane, d.reference, operation)
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
                if d.reference != Reference::Disabled {
                    let value = self
                        .height_offsets
                        .get(&plane)
                        .cloned()
                        .unwrap_or_else(|| fmt_num(d.offset));
                    let invalid = value
                        .trim()
                        .parse::<f64>()
                        .ok()
                        .is_none_or(|v| !v.is_finite());
                    section = section.push(
                        row![
                            help_wrap(
                                text("Offset (mm)").size(12).width(70),
                                illustrated_help::height_help(plane),
                                self.tooltips
                            ),
                            inspector_input("0", &value)
                                .id(plane_input_id(plane))
                                .on_input(move |v| Message::HeightOffsetChanged(plane, v))
                                .on_submit(Message::Apply)
                                .align_x(iced::alignment::Horizontal::Right)
                                .width(Length::Fill)
                                .style(move |theme, status| {
                                    let mut style = inspector_input_style(theme, status);
                                    if invalid {
                                        style.border.color = palette::ERROR;
                                    }
                                    style
                                }),
                        ]
                        .spacing(8)
                        .align_y(Alignment::Center),
                    );
                    if matches!(d.reference, Reference::Highest | Reference::Lowest) {
                        section = section.push(
                            text("Compare these references:")
                                .size(11)
                                .color(INSPECTOR_MUTED),
                        );
                        for reference in Reference::options_for(plane, operation)
                            .into_iter()
                            .filter(|r| {
                                !matches!(
                                    r,
                                    Reference::Disabled | Reference::Highest | Reference::Lowest
                                )
                            })
                        {
                            section = section.push(
                                checkbox(d.sources.contains(&reference))
                                    .label(reference.to_string())
                                    .size(14)
                                    .text_size(11)
                                    .on_toggle(move |on| {
                                        Message::HeightSourceToggled(plane, reference, on)
                                    }),
                            );
                        }
                    }
                    let needs_geometry = matches!(
                        d.reference,
                        Reference::Selection | Reference::FixtureTop | Reference::FixtureBottom
                    ) || d.sources.iter().any(|r| {
                        matches!(
                            r,
                            Reference::Selection | Reference::FixtureTop | Reference::FixtureBottom
                        )
                    }) && matches!(
                        d.reference,
                        Reference::Highest | Reference::Lowest
                    );
                    if needs_geometry {
                        section = section.push(
                            row![
                                inspector_pick_button(
                                    if self.height_reference_pick == Some(plane) {
                                        "Done picking"
                                    } else {
                                        "Pick reference geometry"
                                    },
                                    self.height_reference_pick == Some(plane)
                                )
                                .on_press(Message::HeightPickToggle(plane)),
                                inspector_button("Clear")
                                    .on_press(Message::HeightSelectionClear(plane))
                            ]
                            .spacing(6),
                        );
                        section = section.push(
                            text(format!(
                                "{} selected · {}",
                                d.entities.len() + d.drawings.len(),
                                if matches!(
                                    d.reference,
                                    Reference::FixtureTop | Reference::FixtureBottom
                                ) {
                                    "pick fixture bodies"
                                } else {
                                    "pick faces, edges or vertices"
                                }
                            ))
                            .size(11)
                            .color(INSPECTOR_MUTED),
                        );
                        if self.height_reference_pick == Some(plane) {
                            section = section.push(
                                row![
                                    text("Pick").size(11),
                                    pick_list(
                                        &CadFilter::ALL[..],
                                        Some(self.cad_filter),
                                        Message::CadFilterChanged
                                    )
                                    .text_size(11)
                                    .style(inspector_picker)
                                ]
                                .spacing(8),
                            );
                        }
                        for (index, entity) in d.entities.iter().enumerate() {
                            section = section.push(
                                row![
                                    text(format!(
                                        "{:?} {} · instance {}",
                                        entity.kind, entity.id, entity.instance_id
                                    ))
                                    .size(11)
                                    .width(Length::Fill),
                                    inspector_button("×")
                                        .on_press(Message::HeightEntityRemove(plane, index))
                                ]
                                .spacing(6),
                            );
                        }
                        for (index, drawing) in d.drawings.iter().enumerate() {
                            section = section.push(
                                row![
                                    text(format!(
                                        "Drawing {} · {} points",
                                        if drawing.closed { "contour" } else { "path" },
                                        drawing.points.len()
                                    ))
                                    .size(11)
                                    .width(Length::Fill),
                                    inspector_button("×").on_press(Message::HeightEntityRemove(
                                        plane,
                                        d.entities.len() + index
                                    ))
                                ]
                                .spacing(6),
                            );
                        }
                    }
                }
            }
            if plane == Plane::Bottom && !collapsed {
                if let Some(Operation::Drill(drill)) = self.current_operation(id) {
                    section = section.push(help_wrap(
                        row![
                            text("Drill Tip Through Bottom")
                                .size(12)
                                .color(INSPECTOR_TEXT)
                                .width(Length::Fill),
                            checkbox(drill.tip_through_bottom)
                                .size(15)
                                .on_toggle(Message::DrillTipThroughBottomToggled),
                        ]
                        .spacing(8)
                        .align_y(Alignment::Center),
                        "Extend the drill point beyond the programmed bottom so the cutting lip reaches full diameter at that plane.",
                        self.tooltips,
                    ));
                }
            }
            controls = controls.push(section.width(Length::Fill));
        }
        if let Err(reason) = resolved {
            controls = controls.push(text(reason).size(11).color(palette::ERROR));
        }
        controls.into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configured_maximum_z_sets_automatic_tool_change_height_without_changing_travel() {
        let mut machine = crate::default_machine();
        assert_eq!(machine.envelope.extent().2, 100.0);
        assert_eq!(machine.envelope.max.z, 50.0);
        set_machine_z_limits(&mut machine.envelope, 100.0, 100.0);
        assert_eq!(machine.envelope.extent().2, 100.0);
        assert_eq!(machine.envelope.min.z, 0.0);

        let mut controller = AppController::new(machine);
        controller.edit_setup(|setup| setup.origin[2] = 12.0);
        let setup = controller.inspection_setup();
        assert_eq!(
            setup.resolved_tool_change_height(controller.machine().envelope.max.z)
                - setup.origin[2],
            100.0
        );
    }

    #[test]
    fn tool_change_height_uses_wcs_values_in_the_setup_inspector() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.edit_setup(|setup| setup.origin[2] = 54.);
        app.controller.select(Selection::Setup);
        app.refresh_fields();
        assert_eq!(app.field_value(Field::ToolChangeHeight), Some(50.));
        assert_eq!(
            app.controller
                .inspection_setup()
                .resolved_tool_change_height(50.),
            104.
        );
        app.fields.insert(Field::ToolChangeHeight, "40".into());
        app.apply_inspector();
        assert_eq!(
            app.controller.inspection_setup().tool_change_height,
            Some(94.)
        );
        assert_eq!(app.field_value(Field::ToolChangeHeight), Some(40.));
        // An explicit stored plane must not receive the datum offset twice.
        assert_eq!(
            app.controller
                .inspection_setup()
                .resolved_tool_change_height(50.),
            94.
        );
    }

    #[test]
    fn feed_toggle_preserves_reference_and_offsets_without_committing() {
        let (mut app, _) = App::new();
        app.height_draft = Some(OperationHeightSettings::default());
        app.height_offsets = Plane::ALL
            .into_iter()
            .map(|plane| (plane, "0".to_string()))
            .collect();
        app.height_reference_selected(Plane::Feed, Reference::Highest);
        let before = app.height_draft.clone().unwrap();
        app.height_reference_pick = Some(Plane::Feed);
        app.collapsed_heights.insert(Plane::Feed);
        // A hidden invalid offset must not block applying the remaining heights.
        app.height_offsets.insert(Plane::Feed, "unfinished".into());

        let _ = app.update(Message::HeightFeedEnabled(false));
        assert_eq!(
            app.height_draft
                .as_ref()
                .unwrap()
                .definition(Plane::Feed)
                .reference,
            Reference::Disabled
        );
        assert!(app.height_reference_pick.is_none());
        assert!(app.focused_height.is_none());
        assert!(app.edited_height_settings().is_ok());

        let _ = app.update(Message::HeightFeedEnabled(true));
        assert_eq!(app.height_draft.as_ref(), Some(&before));
        assert!(!app.collapsed_heights.contains(&Plane::Feed));
        assert_eq!(app.height_offsets[&Plane::Feed], "unfinished");
        assert!(app.edited_height_settings().is_err());
        assert!(app.controller.inspection_setup().height_settings.is_empty());
    }

    #[test]
    fn saved_disabled_feed_enables_from_top_and_defaults_clear_cached_reference() {
        let (mut app, _) = App::new();
        let mut settings = OperationHeightSettings::default();
        settings.definition_mut(Plane::Feed).reference = Reference::Disabled;
        app.height_draft = Some(settings);
        let _ = app.update(Message::HeightFeedEnabled(true));
        assert_eq!(
            app.height_draft
                .as_ref()
                .unwrap()
                .definition(Plane::Feed)
                .reference,
            Reference::Plane(Plane::Top)
        );
        app.height_reference_selected(Plane::Feed, Reference::ModelTop);
        let _ = app.update(Message::HeightFeedEnabled(false));
        let _ = app.update(Message::HeightDefaults);
        assert!(app.height_disabled_feed_reference.is_none());
        assert_eq!(app.height_draft, Some(OperationHeightSettings::default()));
    }
}
