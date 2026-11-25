//! Transactional editor for setup-level operation patterns.
use super::*;
use iced::widget::column;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PatternTab {
    Sources,
    Placement,
    Preview,
}

impl PatternTab {
    const ALL: [Self; 3] = [Self::Sources, Self::Placement, Self::Preview];
}

impl std::fmt::Display for PatternTab {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Sources => "Sources",
            Self::Placement => "Placement",
            Self::Preview => "Preview",
        })
    }
}

pub(super) fn kind_from_draft(draft: &PatternDraft) -> Result<cam_model::PatternKind, String> {
    configuration_from_draft(draft).map(|(kind, _)| kind)
}

pub(super) fn configuration_from_draft(
    draft: &PatternDraft,
) -> Result<(cam_model::PatternKind, cam_model::PatternOptions), String> {
    let options = draft.options.clone();
    let kind = if draft.circular {
        let count = draft.values[0]
            .parse::<u32>()
            .map_err(|_| "Enter an integer instance count")?;
        let cx = draft.values[1]
            .parse::<f64>()
            .map_err(|_| "Enter a finite center X")?;
        let cy = draft.values[2]
            .parse::<f64>()
            .map_err(|_| "Enter a finite center Y")?;
        let sweep = draft.values[3]
            .parse::<f64>()
            .map_err(|_| "Enter a sweep angle")?;
        let distribution = if options.spacing == cam_model::PatternSpacing::Spacing {
            cam_model::CircularDistribution::Spacing {
                angle_degrees: sweep,
            }
        } else if sweep == 360.0 {
            cam_model::CircularDistribution::Full
        } else {
            cam_model::CircularDistribution::Partial {
                sweep_degrees: sweep,
            }
        };
        cam_model::PatternKind::Circular {
            center_xy: [cx, cy],
            count,
            distribution,
            direction: if draft.clockwise {
                cam_model::PatternDirection::Cw
            } else {
                cam_model::PatternDirection::Ccw
            },
        }
    } else {
        cam_model::PatternKind::Rectangular {
            nx: draft.values[0]
                .parse::<u32>()
                .map_err(|_| "Enter an integer direction 1 count")?,
            ny: if options.second_direction {
                draft.values[1]
                    .parse::<u32>()
                    .map_err(|_| "Enter an integer direction 2 count")?
            } else {
                1
            },
            dx: draft.values[2]
                .parse::<f64>()
                .map_err(|_| "Enter direction 1 distance")?,
            dy: if options.second_direction {
                draft.values[3]
                    .parse::<f64>()
                    .map_err(|_| "Enter direction 2 distance")?
            } else {
                0.0
            },
        }
    };
    options.validate(kind)?;
    Ok((kind, options))
}

fn reference_action(draft: &PatternDraft, axis: usize) -> Message {
    let selected = if axis < 2 {
        draft.options.directions[axis].reference.is_some()
    } else {
        draft.options.axis.is_some()
    };
    if selected {
        Message::PatternHighlight(axis)
    } else {
        Message::PatternPick(axis)
    }
}

impl App {
    fn pattern_reference_controls(&self, axis: usize) -> Element<'_, Message> {
        let draft = self.pattern_draft.as_ref().expect("pattern editor");
        let selected = if axis < 2 {
            draft.options.directions[axis].reference.is_some()
        } else {
            draft.options.axis.is_some()
        };
        let picking = draft.picking == Some(axis);
        let mut controls = row![inspector_pick_button(
            if picking {
                "Selecting…"
            } else if selected {
                "Edge selected"
            } else {
                "Select edge"
            },
            picking
        )
        .on_press(reference_action(draft, axis))]
        .spacing(6);
        if selected {
            controls = controls.push(
                inspector_pick_button("Reselect", picking).on_press(Message::PatternPick(axis)),
            );
        }
        controls
            .push(inspector_button("Clear").on_press(Message::PatternClear(axis)))
            .into()
    }

    pub(super) fn change_pattern_type(&mut self, circular: bool) {
        let origin = self
            .controller
            .inspection_setup()
            .origin_position(self.controller.active_origin());
        let Some(draft) = &mut self.pattern_draft else {
            return;
        };
        if draft.circular == circular {
            return;
        }
        draft.circular = circular;
        draft.picking = None;
        draft.highlighted = None;
        draft.options.axis = None;
        if !circular {
            draft.options.directions = cam_model::PatternOptions::default().directions;
        }
        draft.options.spacing = if circular {
            cam_model::PatternSpacing::Extent
        } else {
            cam_model::PatternSpacing::Spacing
        };
        draft.values = if circular {
            [
                "4".into(),
                origin[0].to_string(),
                origin[1].to_string(),
                "360".into(),
            ]
        } else {
            ["2".into(), "2".into(), "20".into(), "20".into()]
        };
    }

    pub(super) fn pattern_direction_picked(
        &mut self,
        axis: usize,
        world: [f64; 3],
        reference: Option<EntityRef>,
    ) {
        let frame = &self.controller.inspection_setup().frame;
        let local = frame.to_setup_vector(world);
        let length = local[0].hypot(local[1]);
        if !length.is_finite() || length < 1e-9 || local[2].abs() > length * 1e-6 {
            self.status = "Choose a nonzero straight edge parallel to setup XY.".into();
            return;
        }
        if let Some(draft) = &mut self.pattern_draft {
            draft.options.directions[axis].angle_degrees = local[1].atan2(local[0]).to_degrees();
            draft.options.directions[axis].reference = reference;
            draft.picking = None;
            draft.highlighted = Some(axis);
        }
        self.cad_candidates.clear();
        self.status = format!("Pattern direction {} selected.", axis + 1);
    }

    pub(super) fn pattern_entity_picked(&mut self, entity: EntityRef) {
        let Some(axis) = self.pattern_draft.as_ref().and_then(|draft| draft.picking) else {
            return;
        };
        let state = self.controller.geometry_state();
        if entity.kind != EntityKind::Edge || !state.entity_exists(&entity) {
            self.status = "The selected edge is stale or missing.".into();
            return;
        }
        let edge = state.asset(entity.asset_id).and_then(|asset| {
            asset
                .topology
                .edges
                .iter()
                .find(|edge| edge.id == entity.id && edge.body_id == entity.body_id)
        });
        let instance = state.instance(entity.instance_id);
        let (Some(edge), Some(instance)) = (edge, instance) else {
            self.status = "The selected edge is missing.".into();
            return;
        };
        let (point, direction) = match &edge.curve {
            cam_model::CadCurve::Line { start, end } => {
                (*start, std::array::from_fn(|i| end[i] - start[i]))
            }
            cam_model::CadCurve::CircleArc { center, normal, .. } if axis == 2 => {
                (*center, *normal)
            }
            _ => {
                self.status = "Choose a straight direction edge or a circular axis edge.".into();
                return;
            }
        };
        let direction = instance.placement.transform_vector(direction);
        if axis < 2 {
            self.pattern_direction_picked(axis, direction, Some(entity));
            return;
        }
        let frame = &self.controller.inspection_setup().frame;
        let local = frame.to_setup_vector(direction);
        if !local.iter().all(|v| v.is_finite())
            || local[2].abs() < 1e-9
            || local[0].hypot(local[1]) > local[2].abs() * 1e-6
        {
            self.status = "The circular axis must be parallel to setup Z.".into();
            return;
        }
        let center = frame.to_setup_vector(instance.placement.transform_point(point));
        if let Some(draft) = &mut self.pattern_draft {
            draft.values[1] = center[0].to_string();
            draft.values[2] = center[1].to_string();
            draft.options.axis = Some(entity);
            draft.picking = None;
            draft.highlighted = Some(axis);
        }
        self.cad_candidates.clear();
        self.status = "Circular pattern axis selected.".into();
    }

    fn pattern_draft_error(&self) -> Option<String> {
        let draft = self.pattern_draft.as_ref()?;
        let setup = self
            .controller
            .setups()
            .iter()
            .find(|s| s.id == draft.setup_id)?;
        if draft.name.trim().is_empty() {
            return Some("Enter a pattern name.".into());
        }
        if draft.members.is_empty() {
            return Some("Choose at least one source operation.".into());
        }
        let mut datum = None;
        for id in &draft.members {
            let Some(op) = setup.operations.iter().find(|op| op.id() == *id) else {
                return Some(format!("Source operation {id} no longer exists."));
            };
            if let Some(other) = setup
                .patterns
                .iter()
                .find(|p| Some(p.id) != draft.pattern_id && p.members.contains(id))
            {
                return Some(format!(
                    "Operation {id} already belongs to pattern '{}'.",
                    other.name
                ));
            }
            if datum.is_some_and(|value| value != op.work_offset()) {
                return Some("Pattern sources must share one work datum.".into());
            }
            datum = Some(op.work_offset());
        }
        kind_from_draft(draft).err()
    }

    pub(super) fn pattern_editor(&self) -> Element<'_, Message> {
        let draft = self.pattern_draft.as_ref().expect("checked above");
        let tab = self.pattern_tab;
        let header = column![
            inspector_heading(if draft.circular {
                "Circular pattern"
            } else {
                "Linear pattern"
            }),
            row![
                text("Name")
                    .size(12)
                    .color(INSPECTOR_TEXT)
                    .width(Length::FillPortion(5)),
                inspector_input("Pattern", &draft.name)
                    .on_input(Message::PatternName)
                    .width(Length::FillPortion(6)),
            ]
            .spacing(8)
            .align_y(Alignment::Center),
            row(PatternTab::ALL.into_iter().map(|value| {
                inspector_button(value.to_string())
                    .padding([7, 4])
                    .width(Length::Fill)
                    .style(move |theme, status| inspector_tab(theme, status, tab == value))
                    .on_press(Message::PatternTabChanged(value))
                    .into()
            }))
            .spacing(3),
        ]
        .spacing(10)
        .padding(14);
        let mut list = column![inspector_section(&tab.to_string())]
            .spacing(10)
            .padding(Padding {
                top: 0.0,
                right: 18.0,
                bottom: 16.0,
                left: 14.0,
            });
        let placements = configuration_from_draft(draft).ok();
        let instances = placements
            .as_ref()
            .map_or(0, |(kind, options)| options.instance_indices(*kind).len());
        match tab {
            PatternTab::Sources => {
                list = list.push(
                    text("Select source operations in their execution order.")
                        .size(11)
                        .color(INSPECTOR_MUTED),
                );
                if let Some(setup) = self
                    .controller
                    .setups()
                    .iter()
                    .find(|s| s.id == draft.setup_id)
                {
                    for op in &setup.operations {
                        let id = op.id();
                        let used = setup
                            .patterns
                            .iter()
                            .any(|p| Some(p.id) != draft.pattern_id && p.members.contains(&id));
                        list = list.push(
                            checkbox(draft.members.contains(&id))
                                .label(format!(
                                    "{} · operation {id}{}",
                                    op_kind(op),
                                    if used { " (already grouped)" } else { "" }
                                ))
                                .size(14)
                                .text_size(12)
                                .spacing(8)
                                .on_toggle(move |checked| Message::PatternMember(id, checked)),
                        );
                    }
                    if setup.operations.is_empty() {
                        list = list.push(
                            text("Create an operation before adding pattern sources.")
                                .size(11)
                                .color(INSPECTOR_MUTED),
                        );
                    }
                }
            }
            PatternTab::Placement => {
                list = list.push(profile_picker_w(
                    "Pattern type",
                    "Repeat along linear directions or around an axis.",
                    self.tooltips,
                    if draft.circular {
                        "Circular pattern"
                    } else {
                        "Linear pattern"
                    },
                    &["Linear pattern", "Circular pattern"],
                    |value| Message::PatternTypeChanged(value == "Circular pattern"),
                    INSPECTOR_PICKER_W,
                ));
                list = list.push(profile_picker_w(
                    "Distance type", "Spacing is the distance between adjacent instances. Extent is the total distance or angle.", self.tooltips,
                    if draft.options.spacing == cam_model::PatternSpacing::Spacing { "Spacing" } else { "Extent" },
                    &["Spacing", "Extent"],
                    |value| Message::PatternSpacingChanged(if value == "Spacing" { cam_model::PatternSpacing::Spacing } else { cam_model::PatternSpacing::Extent }), INSPECTOR_PICKER_W,
                ));
                if draft.circular {
                    list = list.push(
                        row![
                            text("Axis")
                                .size(12)
                                .color(INSPECTOR_TEXT)
                                .width(Length::Fill),
                            self.pattern_reference_controls(2),
                        ]
                        .spacing(6)
                        .align_y(Alignment::Center),
                    );
                    list = list.push(
                        text("Axis parallel to setup Z; center in setup coordinates.")
                            .size(11)
                            .color(INSPECTOR_MUTED),
                    );
                    for (index, label, unit) in [
                        (1, "Center X", "mm"),
                        (2, "Center Y", "mm"),
                        (3, "Angle", "deg"),
                        (0, "Instance count", ""),
                    ] {
                        list = list.push(inspector_value(
                            label,
                            &draft.values[index],
                            unit,
                            move |value| Message::PatternValue(index, value),
                        ));
                    }
                    list = list.push(
                        checkbox(draft.clockwise)
                            .label("Reverse direction (clockwise)")
                            .size(14)
                            .text_size(12)
                            .on_toggle(Message::PatternDirectionChanged),
                    );
                } else {
                    list = list.push(
                        text("Select direction edges; defaults are setup X and Y.")
                            .size(11)
                            .color(INSPECTOR_MUTED),
                    );
                    for axis in 0..2 {
                        if axis == 1 {
                            list = list.push(
                                checkbox(draft.options.second_direction)
                                    .label("Additional direction")
                                    .size(14)
                                    .text_size(12)
                                    .on_toggle(Message::PatternSecondDirection),
                            );
                            if !draft.options.second_direction {
                                continue;
                            }
                        }
                        list = list.push(inspector_section(if axis == 0 {
                            "Direction 1"
                        } else {
                            "Direction 2"
                        }));
                        let direction = &draft.options.directions[axis];
                        list = list.push(self.pattern_reference_controls(axis));
                        list = list.push(
                            checkbox(direction.reversed)
                                .label("Reverse direction")
                                .size(14)
                                .text_size(12)
                                .on_toggle(move |value| Message::PatternAxisReversed(axis, value)),
                        );
                        list = list.push(inspector_value(
                            "Quantity",
                            &draft.values[axis],
                            "",
                            move |value| Message::PatternValue(axis, value),
                        ));
                        let index = 2 + axis;
                        list = list.push(inspector_value(
                            if draft.options.spacing == cam_model::PatternSpacing::Spacing {
                                "Distance"
                            } else {
                                "Extent"
                            },
                            &draft.values[index],
                            "mm",
                            move |value| Message::PatternValue(index, value),
                        ));
                        list = list.push(profile_picker_w(
                            "Direction type", "Symmetric alternates placements on either side of the original, starting in the positive direction.", self.tooltips,
                            if direction.symmetric { "Symmetric" } else { "One direction" }, &["One direction", "Symmetric"],
                            move |value| Message::PatternSymmetric(axis, value == "Symmetric"), INSPECTOR_PICKER_W,
                        ));
                    }
                }
                list = list.push(inspector_section("Execution"));
                list = list.push(
                    checkbox(draft.options.keep_original)
                        .label("Keep original")
                        .size(14)
                        .text_size(12)
                        .on_toggle(Message::PatternKeepOriginal),
                );
                list = list.push(
                    checkbox(draft.options.reverse)
                        .label("Reverse instance order")
                        .size(14)
                        .text_size(12)
                        .on_toggle(Message::PatternReverse),
                );
                list = list.push(profile_picker_w(
                    "Operation order", "By instance completes each placement. By operation repeats each source. By tool groups sources by tool number.", self.tooltips,
                    match draft.order { cam_model::PatternOrder::ByInstance => "By instance", cam_model::PatternOrder::ByOperation => "By operation", cam_model::PatternOrder::ByTool => "By tool" },
                    &["By instance", "By operation", "By tool"],
                    |value| Message::PatternOrderChanged(match value { "By operation" => cam_model::PatternOrder::ByOperation, "By tool" => cam_model::PatternOrder::ByTool, _ => cam_model::PatternOrder::ByInstance }), INSPECTOR_PICKER_W,
                ));
            }
            PatternTab::Preview => {
                let markers = placements
                    .as_ref()
                    .map(|(kind, options)| {
                        options
                            .instance_indices(*kind)
                            .into_iter()
                            .take(12)
                            .filter_map(|index| {
                                options.transform_xy(*kind, index).map(|(offset, angle)| {
                                    format!(
                                        "{index}: ({:.3}, {:.3}), {angle:.3}°",
                                        offset[0], offset[1]
                                    )
                                })
                            })
                            .collect::<Vec<_>>()
                            .join("\n")
                    })
                    .unwrap_or_default();
                list = list.push(
                    text("Placements in setup XY")
                        .size(12)
                        .color(INSPECTOR_TEXT),
                );
                list = list.push(text(markers).size(12).color(INSPECTOR_MUTED));
                list = list.push(text("The viewport updates as you edit. Source toolpaths are calculated once in the background; crosses mark placements while they load.")
                    .size(11).color(INSPECTOR_MUTED));
            }
        }
        let error = self.pattern_draft_error();
        let mut feedback = column![text(format!(
            "{instances} instances · {} · {} source operation(s)",
            if draft.options.keep_original {
                "original included"
            } else {
                "original excluded"
            },
            draft.members.len()
        ))
        .size(11)
        .color(INSPECTOR_MUTED)]
        .spacing(8);
        if let Some(reason) = &error {
            feedback = feedback.push(text(reason.clone()).size(11).color(palette::WARN));
        }
        let footer = inspector_footer(
            feedback.push(
                row![
                    inspector_button("Cancel").on_press(Message::PatternCancel),
                    Space::new().width(Length::Fill),
                    inspector_button(if draft.pattern_id.is_some() {
                        "Apply changes"
                    } else {
                        "Create pattern"
                    })
                    .style(inspector_primary)
                    .on_press_maybe(error.is_none().then_some(Message::PatternCommit)),
                ]
                .spacing(8),
            ),
        );
        column![
            header,
            scrollable(list).width(Length::Fill).height(Length::Fill),
            footer,
        ]
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn app() -> App {
        let (mut app, _) = App::new();
        let _ = app.update(Message::OpenSample);
        app.focus_ops
            .insert(app.controller.inspection_setup().operations[0].id());
        app
    }

    #[test]
    fn pattern_controls_commit_roundtrip_and_cancel_in_one_history_edit() {
        let mut app = app();
        let _ = app.update(Message::BeginPattern(false));
        for message in [
            Message::PatternSpacingChanged(cam_model::PatternSpacing::Extent),
            Message::PatternSecondDirection(true),
            Message::PatternValue(0, "3".into()),
            Message::PatternValue(1, "4".into()),
            Message::PatternValue(2, "50".into()),
            Message::PatternValue(3, "30".into()),
            Message::PatternAxisReversed(0, true),
            Message::PatternSymmetric(1, true),
            Message::PatternKeepOriginal(false),
            Message::PatternReverse(true),
            Message::PatternOrderChanged(cam_model::PatternOrder::ByTool),
        ] {
            let _ = app.update(message);
        }
        let expected = configuration_from_draft(app.pattern_draft.as_ref().unwrap()).unwrap();
        for tab in PatternTab::ALL {
            app.pattern_tab = tab;
            let _ = app.pattern_editor();
        }
        let _ = app.update(Message::PatternCommit);
        assert!(app.pattern_draft.is_none(), "{}", app.status);
        let pattern = app.controller.inspection_setup().patterns[0].clone();
        assert_eq!((pattern.kind, pattern.options.clone()), expected);
        assert_eq!(pattern.order, cam_model::PatternOrder::ByTool);
        let encoded = serde_json::to_string(&pattern).unwrap();
        assert_eq!(
            serde_json::from_str::<cam_model::OperationPattern>(&encoded).unwrap(),
            pattern
        );
        let _ = app.update(Message::EditPattern(pattern.id));
        assert_eq!(
            configuration_from_draft(app.pattern_draft.as_ref().unwrap()).unwrap(),
            expected
        );
        let _ = app.update(Message::PatternKeepOriginal(true));
        let _ = app.update(Message::PatternCancel);
        assert_eq!(app.controller.inspection_setup().patterns[0], pattern);
        assert!(app.controller.undo());
        assert!(app.controller.inspection_setup().patterns.is_empty());
    }

    #[test]
    fn circular_spacing_and_type_controls_use_the_same_validation_as_commit() {
        let mut app = app();
        let _ = app.update(Message::BeginPattern(false));
        let _ = app.update(Message::PatternTypeChanged(true));
        let _ = app.update(Message::PatternSpacingChanged(
            cam_model::PatternSpacing::Spacing,
        ));
        let _ = app.update(Message::PatternValue(3, "120".into()));
        let _ = app.update(Message::PatternCommit);
        assert!(
            app.pattern_draft.is_some(),
            "four instances at 120 degrees overlap"
        );
        let _ = app.update(Message::PatternValue(3, "45".into()));
        let _ = app.update(Message::PatternDirectionChanged(true));
        let _ = app.update(Message::PatternCommit);
        let pattern = app.controller.inspection_setup().patterns[0].clone();
        assert_eq!(pattern.kind.transform_xy(3).unwrap().1, -135.0);
        let _ = app.update(Message::EditPattern(pattern.id));
        assert_eq!(
            app.pattern_draft.as_ref().unwrap().options.spacing,
            cam_model::PatternSpacing::Spacing
        );
        assert_eq!(
            configuration_from_draft(app.pattern_draft.as_ref().unwrap())
                .unwrap()
                .0,
            pattern.kind
        );
    }

    #[test]
    fn pattern_direction_pick_uses_setup_frame_and_escape_preserves_draft() {
        let mut app = app();
        app.controller.edit_setup(|setup| {
            setup.frame = cam_model::SetupFrame::from_xy([0., 1., 0.], [0., 0., 1.]).unwrap();
        });
        let _ = app.update(Message::BeginPattern(false));
        let _ = app.update(Message::PatternPick(0));
        let _ = app.update(Message::SetupStockEdgePicked([1., 0., 0.]));
        assert_eq!(
            app.pattern_draft.as_ref().unwrap().picking,
            Some(0),
            "out-of-plane edges leave picker armed"
        );
        let _ = app.update(Message::SetupStockEdgePicked([0., 0., 8.]));
        let draft = app.pattern_draft.as_ref().unwrap();
        assert_eq!(draft.picking, None);
        assert_eq!(draft.options.directions[0].angle_degrees, 90.0);
        assert!(app.setup_draft.is_none());
        let _ = app.update(Message::PatternPick(1));
        let _ = app.update(Message::CadEscape);
        assert_eq!(app.pattern_draft.as_ref().unwrap().picking, None);
        assert_eq!(
            app.pattern_draft.as_ref().unwrap().options.directions[0].angle_degrees,
            90.0
        );
    }

    #[test]
    fn pattern_model_edges_resolve_instance_placement_and_circular_center() {
        use cam_model::{
            CadBody, CadCurve, CadEdge, CadFormat, CadVertex, GeometryAsset, GeometryInstance,
            RigidTransform, SourceUnits, TopologySnapshot,
        };
        let mut app = app();
        app.controller
            .add_cad_asset(
                GeometryAsset {
                    id: 1,
                    revision: 1,
                    name: "Pattern references".into(),
                    source_format: CadFormat::Step,
                    source_path: None,
                    source_data: vec![].into(),
                    source_units: SourceUnits::Millimetres,
                    backend: "test".into(),
                    import_options: None,
                    import_diagnostics: vec![],
                    source_fingerprint: String::new(),
                    topology: TopologySnapshot {
                        bodies: vec![CadBody {
                            id: 1,
                            name: "Edges".into(),
                            valid_solid: false,
                            bounds: [[-3., -2., 0.], [8., 8., 0.]],
                        }],
                        vertices: vec![
                            CadVertex {
                                id: 1,
                                body_id: 1,
                                point: [0., 0., 0.],
                            },
                            CadVertex {
                                id: 2,
                                body_id: 1,
                                point: [8., 0., 0.],
                            },
                            CadVertex {
                                id: 3,
                                body_id: 1,
                                point: [7., 3., 0.],
                            },
                        ],
                        edges: vec![
                            CadEdge {
                                id: 1,
                                body_id: 1,
                                vertices: [1, 2],
                                face_ids: vec![],
                                curve: CadCurve::Line {
                                    start: [0., 0., 0.],
                                    end: [8., 0., 0.],
                                },
                            },
                            CadEdge {
                                id: 2,
                                body_id: 1,
                                vertices: [3, 3],
                                face_ids: vec![],
                                curve: CadCurve::CircleArc {
                                    center: [2., 3., 0.],
                                    normal: [0., 0., 1.],
                                    x_axis: [1., 0., 0.],
                                    radius: 5.,
                                    start_angle: 0.,
                                    end_angle: std::f64::consts::TAU,
                                },
                            },
                        ],
                        ..Default::default()
                    }
                    .into(),
                },
                vec![GeometryInstance {
                    id: 1,
                    asset_id: 1,
                    body_id: 1,
                    name: "Placed edges".into(),
                    visible: true,
                    placement: RigidTransform {
                        rotation: [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]],
                        translation: [10., 20., 30.],
                    },
                }],
            )
            .unwrap();
        let instance = &app.controller.geometry_state().instances[0];
        let edge = |id| EntityRef {
            revision: 1,
            asset_id: instance.asset_id,
            instance_id: instance.id,
            body_id: 1,
            kind: EntityKind::Edge,
            id,
        };
        let line = edge(1);
        let circle = edge(2);
        let _ = app.update(Message::BeginPattern(false));
        let _ = app.update(Message::PatternPick(0));
        let _ = app.update(Message::CadSelect(line.clone()));
        let draft = app.pattern_draft.as_ref().unwrap();
        assert_eq!(draft.options.directions[0].angle_degrees, 90.0);
        assert_eq!(draft.options.directions[0].reference, Some(line.clone()));
        let before = configuration_from_draft(draft).unwrap();
        let revision = app.controller.revision();
        let action = reference_action(draft, 0);
        assert!(matches!(action, Message::PatternHighlight(0)));
        let _ = app.update(action);
        assert_eq!(app.editor_selected_geometry(), vec![line.clone()]);
        assert_eq!(app.pattern_draft.as_ref().unwrap().picking, None);
        assert_eq!(
            configuration_from_draft(app.pattern_draft.as_ref().unwrap()).unwrap(),
            before
        );
        assert_eq!(app.controller.revision(), revision);
        assert!(
            app.machining_viewport()
                .vertices
                .iter()
                .any(|vertex| vertex.color == [0.2, 0.6, 0.98, 1.0] && vertex.width > 1.0),
            "selected edge is highlighted in the rendered view"
        );
        let _ = app.update(Message::PatternPick(0));
        assert_eq!(
            app.pattern_draft.as_ref().unwrap().options.directions[0].reference,
            Some(line)
        );
        let action = reference_action(app.pattern_draft.as_ref().unwrap(), 0);
        let _ = app.update(action);
        assert_eq!(app.pattern_draft.as_ref().unwrap().picking, None);
        let _ = app.update(Message::PatternTypeChanged(true));
        let _ = app.update(Message::PatternPick(2));
        let _ = app.update(Message::CadSelect(circle.clone()));
        let draft = app.pattern_draft.as_ref().unwrap();
        assert_eq!(
            (&draft.values[1], &draft.values[2]),
            (&"7".to_string(), &"22".to_string())
        );
        assert_eq!(draft.options.axis, Some(circle.clone()));
        assert_eq!(draft.picking, None);
        let before = configuration_from_draft(draft).unwrap();
        let action = reference_action(draft, 2);
        assert!(matches!(action, Message::PatternHighlight(2)));
        let _ = app.update(action);
        assert_eq!(app.editor_selected_geometry(), vec![circle]);
        assert_eq!(
            configuration_from_draft(app.pattern_draft.as_ref().unwrap()).unwrap(),
            before
        );
        assert_eq!(app.pattern_draft.as_ref().unwrap().picking, None);
    }
}
