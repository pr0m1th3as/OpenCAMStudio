use super::*;
use cam_model::RotaryWrap;

pub(super) const INDEX_FACE_COLOR: cam_render::Color = [1.0, 0.48, 0.12, 1.0];

/// Convert a point in the indexed machine frame back to the displayed model frame.
pub(super) fn index_point_to_world(
    frame: &cam_model::SetupFrame,
    index: Option<&cam_model::RotaryIndex>,
    point: [f64; 3],
) -> [f64; 3] {
    let point = if let Some(index) = index {
        let center = cam_cldata::Point3::new(index.center[0], index.center[1], index.center[2]);
        let p = index.axis.material_point(
            cam_cldata::Point3::new(point[0], point[1], point[2]),
            center,
            index.angle,
        );
        [p.x, p.y, p.z]
    } else {
        point
    };
    frame.to_world_vector(point)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum Number {
    Radius,
    Offset,
    CenterX,
    CenterY,
    CenterZ,
    SurfaceZ,
    Seam,
    Tolerance,
}

#[cfg(test)]
mod index_tests {
    use super::*;
    #[test]
    fn rotary_picks_override_profile_contour_filter() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.cad_pending_kind = Some(OpKind::Profile);
        app.operation_draft = Some(app.controller.operation_template(OpKind::Profile, 1));
        app.rotary_action(Action::Indexed(true));
        app.rotary_action(Action::IndexMode(IndexMode::Face));
        for (pick, expected) in [
            (Pick::IndexFace, EntityKind::Face),
            (Pick::Cylinder, EntityKind::Face),
            (Pick::Faces, EntityKind::Face),
            (Pick::Edges, EntityKind::Edge),
            (Pick::Loops, EntityKind::Loop),
        ] {
            app.rotary_action(Action::Pick(pick));
            let viewport = app.machining_viewport();
            assert_eq!(viewport.cad_filter, expected);
            assert!(viewport.cad_filter_explicit);
            assert!(!viewport.cad_silhouette);
        }
    }
    #[test]
    fn indexed_mode_is_staged_and_committed_with_operation_fields_in_one_undo() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        let op = app.controller.operation_template(OpKind::Drill, 1);
        app.controller.add_operation(op);
        let id = app.controller.inspection_setup().operations[0].id();
        app.begin_operation_draft(id);
        app.rotary_action(Action::Indexed(true));
        app.rotary_action(Action::IndexValue(0, "90".into()));
        assert!(app.controller.document().rotary_indexes.is_empty());
        assert!(app.operation_draft_dirty());
        app.apply_inspector();
        assert_eq!(
            app.controller
                .document()
                .rotary_indexes
                .get(&id)
                .map(|i| i.angle),
            Some(90.)
        );
        app.controller.undo();
        assert!(app.controller.document().rotary_indexes.is_empty());
    }
    #[test]
    fn face_angles_align_normals_and_flip_to_the_other_side() {
        for (axis, normal) in [
            (cam_model::RotaryAxis::A, [0., 0.6, 0.8]),
            (cam_model::RotaryAxis::B, [0.6, 0., 0.8]),
        ] {
            for flip in [false, true] {
                let angle = index_angle_for_normal(normal, axis, flip).unwrap();
                let n = axis.material_point(
                    cam_cldata::Point3::new(normal[0], normal[1], normal[2]),
                    cam_cldata::Point3::new(0., 0., 0.),
                    -angle,
                );
                assert!(n.x.abs() < 1e-9 && n.y.abs() < 1e-9);
                assert!((n.z - if flip { -1. } else { 1. }).abs() < 1e-9);
            }
        }
        assert!(index_angle_for_normal([0.5, 0., 0.866], cam_model::RotaryAxis::A, false).is_err());
    }
    #[test]
    fn invalid_index_input_cannot_commit_a_previous_valid_number() {
        let (mut app, _) = App::new();
        app.rotary_action(Action::Indexed(true));
        app.rotary_action(Action::IndexValue(0, "unfinished".into()));
        assert!(app.validate_rotary_index_draft().is_err());
    }
}
impl Number {
    pub(super) const ALL: [Self; 8] = [
        Self::Radius,
        Self::Offset,
        Self::CenterX,
        Self::CenterY,
        Self::CenterZ,
        Self::SurfaceZ,
        Self::Seam,
        Self::Tolerance,
    ];
    pub(super) fn label(self) -> &'static str {
        match self {
            Self::Radius => "Cylinder radius (mm)",
            Self::Offset => "Cylinder offset (mm)",
            Self::CenterX => "Axis centre X (mm)",
            Self::CenterY => "Axis centre Y (mm)",
            Self::CenterZ => "Axis centre Z (mm)",
            Self::SurfaceZ => "Unwrapped surface Z (mm)",
            Self::Seam => "Seam angle (degrees)",
            Self::Tolerance => "Wrap tolerance (mm)",
        }
    }
    pub(super) fn get(self, w: &RotaryWrap) -> f64 {
        match self {
            Self::Radius => w.radius,
            Self::Offset => w.offset,
            Self::CenterX => w.center[0],
            Self::CenterY => w.center[1],
            Self::CenterZ => w.center[2],
            Self::SurfaceZ => w.surface_z,
            Self::Seam => w.seam_angle,
            Self::Tolerance => w.tolerance,
        }
    }
    pub(super) fn set(self, w: &mut RotaryWrap, v: f64) {
        match self {
            Self::Radius => w.radius = v,
            Self::Offset => w.offset = v,
            Self::CenterX => w.center[0] = v,
            Self::CenterY => w.center[1] = v,
            Self::CenterZ => w.center[2] = v,
            Self::SurfaceZ => w.surface_z = v,
            Self::Seam => w.seam_angle = v,
            Self::Tolerance => w.tolerance = v,
        }
    }
    pub(super) fn invalid(self, v: f64) -> bool {
        !v.is_finite()
            || match self {
                Self::Radius => v <= 0.,
                Self::Tolerance => v <= 0. || v > 1.,
                _ => false,
            }
    }
    pub(super) fn cylinder_dimension(self) -> bool {
        matches!(
            self,
            Self::Radius | Self::CenterX | Self::CenterY | Self::CenterZ
        )
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Pick {
    Cylinder,
    Loops,
    Edges,
    Faces,
    IndexFace,
    IndexLoop,
    IndexEdge,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub(super) enum IndexMode {
    Manual,
    Face,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub(super) enum MachiningType {
    ThreeAxis,
    Indexed,
    Wrap,
}
impl MachiningType {
    fn icon(self) -> Element<'static, Self> {
        let bytes: &'static [u8] = match self {
            Self::ThreeAxis => include_bytes!("../assets/icons/machining/three_axis.svg"),
            Self::Indexed => include_bytes!("../assets/icons/machining/indexed.svg"),
            Self::Wrap => include_bytes!("../assets/icons/machining/wrapped.svg"),
        };
        iced::widget::svg(iced::widget::svg::Handle::from_memory(bytes))
            .width(Length::Fixed(22.))
            .height(Length::Fixed(22.))
            .into()
    }
}
impl std::fmt::Display for MachiningType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::ThreeAxis => "3-axis",
            Self::Indexed => "3+1 Indexed",
            Self::Wrap => "Wrap",
        })
    }
}
impl std::fmt::Display for IndexMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Manual => "Manual angle",
            Self::Face => "Selection",
        })
    }
}
#[derive(Clone, Debug)]
pub(super) enum Action {
    Indexed(bool),
    IndexAxis(cam_model::RotaryAxis),
    IndexMode(IndexMode),
    IndexFlip(bool),
    IndexValue(usize, String),
    Enable(bool),
    Unwrap(bool),
    ClearCylinder,
    Pick(Pick),
    Done,
    Clear,
}
fn index_angle_for_normal(
    n: [f64; 3],
    axis: cam_model::RotaryAxis,
    flip: bool,
) -> Result<f64, String> {
    let length = n.iter().map(|x| x * x).sum::<f64>().sqrt();
    if !length.is_finite() || length < 1e-10 {
        return Err("Selection direction is invalid".into());
    }
    let (along, angle) = match axis {
        cam_model::RotaryAxis::A => (n[0], n[1].atan2(n[2]).to_degrees()),
        cam_model::RotaryAxis::B => (n[1], (-n[0]).atan2(n[2]).to_degrees()),
    };
    if along.abs() / length > 1e-6 {
        return Err(
            "This selection cannot be aligned with Z using the selected rotary axis".into(),
        );
    }
    Ok(if flip { angle + 180. } else { angle })
}
impl App {
    /// Draw the orientation reference independently of machining selections.
    pub(super) fn index_face_overlay(&self, cad_scene: &cam_render::CadScene) -> Scene {
        let mut overlay = Scene::new();
        let Some(index) = &self.rotary_index_draft else {
            return overlay;
        };
        let Some(reference) = &index.face else {
            return overlay;
        };
        let geometry = self.controller.geometry_state();
        let Some(instance) = geometry
            .instance(reference.instance_id)
            .filter(|i| i.visible)
        else {
            return overlay;
        };
        let Some(asset) = geometry.asset(reference.asset_id) else {
            return overlay;
        };
        if !geometry.entity_exists(reference) {
            return overlay;
        }
        if reference.kind != EntityKind::Face {
            let outline = cad_scene.edges(std::slice::from_ref(reference), None);
            for strip in outline.strips.into_iter().filter(|s| s.width > 1.0) {
                overlay.add_highlight(strip.points, INDEX_FACE_COLOR);
            }
            if overlay.strips.is_empty() {
                return overlay;
            }
            let Ok(setup_geometry) =
                crate::cad_operations::geometry_for_setup(self.controller.document())
            else {
                return overlay;
            };
            let Ok(normal) = cam_geometry3d::index_selection_direction(&setup_geometry, reference)
            else {
                return overlay;
            };
            let length = normal.iter().map(|v| v * v).sum::<f64>().sqrt();
            if length < 1e-10 {
                return overlay;
            }
            let frame = &self.controller.document().setup().frame;
            let direction = frame
                .to_world_vector(normal.map(|v| v / length * if index.flip { -1. } else { 1. }));
            let points: Vec<_> = overlay
                .strips
                .iter()
                .flat_map(|s| s.points.iter())
                .collect();
            let mut min = [f32::INFINITY; 3];
            let mut max = [f32::NEG_INFINITY; 3];
            for p in points {
                for i in 0..3 {
                    min[i] = min[i].min(p[i]);
                    max[i] = max[i].max(p[i]);
                }
            }
            let tip = std::array::from_fn::<_, 3, _>(|i| (min[i] + max[i]) as f64 * 0.5);
            let length = (0..3)
                .map(|i| (max[i] - min[i]) as f64)
                .fold(0.0_f64, f64::max)
                .mul_add(0.35, 0.)
                .clamp(1., 15.);
            let seed = if direction[0].abs() < 0.8 {
                [1., 0., 0.]
            } else {
                [0., 1., 0.]
            };
            let wing = [
                direction[1] * seed[2] - direction[2] * seed[1],
                direction[2] * seed[0] - direction[0] * seed[2],
                direction[0] * seed[1] - direction[1] * seed[0],
            ];
            let wing_length = wing.iter().map(|v| v * v).sum::<f64>().sqrt();
            let tail = std::array::from_fn::<_, 3, _>(|i| tip[i] + direction[i] * length);
            let left = std::array::from_fn::<_, 3, _>(|i| {
                tip[i] + direction[i] * length * 0.25 + wing[i] / wing_length * length * 0.13
            });
            let right = std::array::from_fn::<_, 3, _>(|i| {
                tip[i] + direction[i] * length * 0.25 - wing[i] / wing_length * length * 0.13
            });
            overlay.add_highlight(
                vec![tail.map(|v| v as f32), tip.map(|v| v as f32)],
                INDEX_FACE_COLOR,
            );
            overlay.add_highlight(
                vec![
                    left.map(|v| v as f32),
                    tip.map(|v| v as f32),
                    right.map(|v| v as f32),
                ],
                INDEX_FACE_COLOR,
            );
            return overlay;
        }
        let Some(face) = asset.topology.faces.iter().find(|f| f.id == reference.id) else {
            return overlay;
        };
        let cam_model::CadSurface::Plane { origin, normal } = face.surface else {
            return overlay;
        };
        // CadScene omits hidden instances. Retain only the face's highlighted
        // edges so the overlay does not redraw the rest of the model.
        let outline = cad_scene.edges(std::slice::from_ref(reference), None);
        for strip in outline.strips.into_iter().filter(|s| s.width > 1.0) {
            overlay.add_highlight(strip.points, INDEX_FACE_COLOR);
        }
        if overlay.strips.is_empty() {
            return overlay;
        }
        let norm = normal.iter().map(|x| x * x).sum::<f64>().sqrt();
        if !norm.is_finite() || norm < 1e-10 {
            return overlay;
        }
        let n = normal.map(|v| v / norm);
        let center =
            std::array::from_fn::<_, 3, _>(|i| (face.bounds[0][i] + face.bounds[1][i]) * 0.5);
        let distance: f64 = (0..3).map(|i| (center[i] - origin[i]) * n[i]).sum();
        let center = std::array::from_fn(|i| center[i] - distance * n[i]);
        let tip = instance.placement.transform_point(center);
        let direction = instance
            .placement
            .transform_vector(n)
            .map(|v| if index.flip { -v } else { v });
        let extent = (0..3)
            .map(|i| face.bounds[1][i] - face.bounds[0][i])
            .fold(0.0_f64, f64::max);
        let length = (extent * 0.35).clamp(1.0, 15.0);
        let tangent_seed = if direction[0].abs() < 0.8 {
            [1., 0., 0.]
        } else {
            [0., 1., 0.]
        };
        let cross = [
            direction[1] * tangent_seed[2] - direction[2] * tangent_seed[1],
            direction[2] * tangent_seed[0] - direction[0] * tangent_seed[2],
            direction[0] * tangent_seed[1] - direction[1] * tangent_seed[0],
        ];
        let cross_length = cross.iter().map(|v| v * v).sum::<f64>().sqrt();
        let wing = cross.map(|v| v / cross_length);
        let tail = std::array::from_fn::<_, 3, _>(|i| tip[i] + direction[i] * length);
        let head = std::array::from_fn::<_, 3, _>(|i| tip[i] + direction[i] * length * 0.25);
        let left = std::array::from_fn::<_, 3, _>(|i| head[i] + wing[i] * length * 0.13);
        let right = std::array::from_fn::<_, 3, _>(|i| head[i] - wing[i] * length * 0.13);
        let point = |p: [f64; 3]| p.map(|v| v as f32);
        overlay.add_highlight(vec![point(tail), point(tip)], INDEX_FACE_COLOR);
        overlay.add_highlight(
            vec![point(left), point(tip), point(right)],
            INDEX_FACE_COLOR,
        );
        overlay
    }

    pub(super) fn rotary_draft_document(&self) -> cam_model::Document {
        let mut document = self.controller.document().clone();
        if let Some(op) = &self.operation_draft {
            if let Some(index) = &self.rotary_index_draft {
                document.rotary_indexes.insert(op.id(), index.clone());
            } else {
                document.rotary_indexes.remove(&op.id());
            }
        }
        document
    }
    pub(super) fn indexed_draft_geometry(&self) -> Result<cam_model::GeometryState, String> {
        crate::cad_operations::geometry_for_operation(
            &self.rotary_draft_document(),
            self.operation_draft.as_ref().map_or(0, Operation::id),
        )
    }
    pub(super) fn seed_rotary_index_values(&mut self) {
        let index = self.rotary_index_draft.clone().unwrap_or_default();
        self.rotary_index_values = [
            fmt_num(index.angle),
            fmt_num(index.center[0]),
            fmt_num(index.center[1]),
            fmt_num(index.center[2]),
        ];
    }
    pub(super) fn validate_rotary_index_draft(&self) -> Result<(), String> {
        if let Some(index) = &self.rotary_index_draft {
            if self.rotary_index_mode == IndexMode::Face && index.face.is_none() {
                return Err("Select an index reference".into());
            }
            if self
                .rotary_index_values
                .iter()
                .any(|v| v.parse::<f64>().is_err())
            {
                return Err("Enter a finite index angle and axis centre".into());
            }
            index.validate()?;
        }
        Ok(())
    }
    pub(super) fn rotary_controls(&self, op: &Operation) -> Element<'_, Message> {
        use iced::widget::column;
        use parameter_options::{MACHINING_TYPE, ROTARY_ANGLE_MODE, ROTARY_AXIS, ROTARY_FLIP, ROTARY_UNWRAP};
        let enabled = op.rotary_wrap().is_some();
        let indexed = self.rotary_index_draft.is_some();
        let selected_mode = if enabled {
            MachiningType::Wrap
        } else if indexed {
            MachiningType::Indexed
        } else {
            MachiningType::ThreeAxis
        };
        let mut machining_types = vec![MachiningType::ThreeAxis, MachiningType::Indexed];
        if op.supports_rotary_wrap() {
            machining_types.push(MachiningType::Wrap);
        }
        let machining_type_picker = container(icon_picker::picker(
            iced::advanced::widget::Id::from("machining-type-menu"),
            machining_types.clone(),
            selected_mode,
            |mode| {
                row![mode.icon(), text(mode.to_string()).size(12).width(Length::Fill)]
                    .spacing(8)
                    .align_y(Alignment::Center)
                    .into()
            },
            |_| 0,
            MACHINING_TYPE.on_change(),
        ))
        .width(Length::Fixed(INSPECTOR_PICKER_W));
        let mut form = column![row![
            text("Machining Type").size(12).width(Length::Fill),
            MACHINING_TYPE.menu(
                &selected_mode,
                selected_mode.to_string(),
                &machining_types,
                machining_type_picker,
            ),
        ]
        .spacing(8)
        .align_y(Alignment::Center)]
        .spacing(12);
        if let Some(index) = &self.rotary_index_draft {
            use cam_model::RotaryAxis;

            let axes = [RotaryAxis::A, RotaryAxis::B];
            let label = match index.axis {
                RotaryAxis::A => "A (X)",
                RotaryAxis::B => "B (Y)",
            };
            let axis_picker = pick_list(["A (X)", "B (Y)"], Some(label), |axis| {
                ROTARY_AXIS.on_change()(if axis == "A (X)" {
                    RotaryAxis::A
                } else {
                    RotaryAxis::B
                })
            })
            .text_size(12)
            .padding([7, 9])
            .style(inspector_picker)
            .width(Length::Fixed(INSPECTOR_INPUT_W));
            form = form.push(
                row![
                    text("Rotary axis").size(12).width(Length::Fill),
                    ROTARY_AXIS.menu(&index.axis, label.into(), &axes, axis_picker),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
            form = form.push(
                row![
                    text("Angle mode").size(12).width(Length::Fill),
                    ROTARY_ANGLE_MODE.picker(
                        self.rotary_index_mode,
                        &[IndexMode::Manual, IndexMode::Face],
                        Length::Fixed(INSPECTOR_INPUT_W),
                    ),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
            if self.rotary_index_mode == IndexMode::Face {
                form = form.push(
                    row![
                        text("Index reference").size(12).width(Length::Fill),
                        row![
                            inspector_pick_button(
                                "Face…",
                                self.rotary_pick == Some(Pick::IndexFace)
                            )
                            .on_press(Message::Rotary(Action::Pick(Pick::IndexFace))),
                            inspector_pick_button(
                                "Chain…",
                                self.rotary_pick == Some(Pick::IndexLoop)
                            )
                            .on_press(Message::Rotary(Action::Pick(Pick::IndexLoop))),
                            inspector_pick_button(
                                "Edge…",
                                self.rotary_pick == Some(Pick::IndexEdge)
                            )
                            .on_press(Message::Rotary(Action::Pick(Pick::IndexEdge))),
                        ]
                        .spacing(4),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
                form = form.push(
                    row![
                        text("Index angle").size(12).width(Length::Fill),
                        text(format!("{}°", self.settings.format_number(index.angle)))
                            .size(12)
                            .width(Length::Fixed(INSPECTOR_INPUT_W))
                            .align_x(iced::alignment::Horizontal::Right),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
                form = form.push(
                    row![
                        help_wrap(
                            text("Flip tool engagement")
                                .size(12)
                                .width(Length::Fill),
                            "Flip adds 180° to the index angle, turning the selected face away from the tool. Select geometry accessible from the resulting direction. Orange marks the orientation face and tool approach; blue marks cutting geometry.",
                            self.tooltips,
                        ),
                        ROTARY_FLIP.checkbox(index.flip, 15),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            } else {
                form = form.push(
                    row![
                        text("Index angle (degrees)").size(12).width(Length::Fill),
                        inspector_input("0", &self.rotary_index_values[0])
                            .on_input(|v| Message::Rotary(Action::IndexValue(0, v)))
                            .width(Length::Fixed(INSPECTOR_INPUT_W))
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            }
            for (i, label) in [
                "Axis centre X (mm)",
                "Axis centre Y (mm)",
                "Axis centre Z (mm)",
            ]
            .into_iter()
            .enumerate()
            {
                let i = i + 1;
                form = form.push(
                    row![
                        help_wrap(
                            text(label).size(12).width(Length::Fill),
                            "A point on the rotary axis in setup coordinates. CAD geometry is machined at this fixed workpiece angle; drawing coordinates and heights use the indexed machine frame.",
                            self.tooltips,
                        ),
                        inspector_input("0", &self.rotary_index_values[i])
                            .on_input(move |v| Message::Rotary(Action::IndexValue(i, v)))
                            .width(Length::Fixed(INSPECTOR_INPUT_W))
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            }
            return form.into();
        }
        let Some(wrap) = op.rotary_wrap() else {
            return form
                .push(text("Tool Orientation").size(14))
                .push(text("Uses setup orientation").size(12))
                .into();
        };
        form = form.push(text("Wrap").size(14));
        form = form.push(
            row![
                text("Unwrap Geometry").size(12).width(Length::Fill),
                ROTARY_UNWRAP.checkbox(wrap.unwrap_geometry, 15)
            ]
            .spacing(8)
            .align_y(Alignment::Center),
        );
        form = form.push(
            row![
                text("Wrap Cylinder").size(12).width(Length::Fill),
                inspector_pick_button(
                    if wrap.cylinder.is_some() {
                        "Face"
                    } else {
                        "Select face…"
                    },
                    self.rotary_pick == Some(Pick::Cylinder)
                )
                .width(Length::Fixed(INSPECTOR_INPUT_W))
                .on_press(Message::Rotary(Action::Pick(Pick::Cylinder))),
                inspector_button("×").on_press(Message::Rotary(Action::ClearCylinder))
            ]
            .spacing(8)
            .align_y(Alignment::Center),
        );
        let field = Field::Wrap(Number::Offset);
        let value = self
            .fields
            .get(&field)
            .cloned()
            .unwrap_or_else(|| fmt_num(wrap.offset));
        form = form.push(self.parameter_field_row(field, "Offset (mm)", field.help(), &value));
        form = form.push(
            row![
                text("Radius").size(12).width(Length::Fill),
                text(format!(
                    "{} mm",
                    self.settings
                        .format_number(wrap.radius + value.parse::<f64>().unwrap_or(wrap.offset))
                ))
                .size(12)
                .width(Length::Fixed(INSPECTOR_INPUT_W))
                .align_x(iced::alignment::Horizontal::Right)
            ]
            .spacing(8)
            .align_y(Alignment::Center),
        );
        form.into()
    }
    pub(super) fn rotary_geometry_controls(&self, op: &Operation) -> Element<'_, Message> {
        use iced::widget::column;
        let count = op.rotary_wrap().map_or(0, |w| w.geometry.len());
        column![
            text(format!("{count} selected machining boundaries")).size(12),
            row![
                inspector_pick_button("Loops", self.rotary_pick == Some(Pick::Loops))
                    .on_press(Message::Rotary(Action::Pick(Pick::Loops))),
                inspector_pick_button("Edges", self.rotary_pick == Some(Pick::Edges))
                    .on_press(Message::Rotary(Action::Pick(Pick::Edges))),
                inspector_pick_button("Faces", self.rotary_pick == Some(Pick::Faces))
                    .on_press(Message::Rotary(Action::Pick(Pick::Faces)))
            ]
            .spacing(4),
            row![
                inspector_button("Done selecting").on_press(Message::Rotary(Action::Done)),
                inspector_button("Use drawing geometry").on_press(Message::Rotary(Action::Clear))
            ]
            .spacing(4)
        ]
        .spacing(8)
        .into()
    }
    pub(super) fn rotary_action(&mut self, action: Action) {
        match action {
            Action::Indexed(enabled) => {
                if enabled {
                    let initializing = self.rotary_index_draft.is_none();
                    let mut index = self.rotary_index_draft.clone().unwrap_or_default();
                    if let Some(wrap) = self
                        .operation_draft
                        .as_ref()
                        .and_then(Operation::rotary_wrap)
                    {
                        index.axis = wrap.axis;
                        index.center = wrap.center;
                    }
                    self.rotary_index_draft = Some(index);
                    self.edit_operation_draft(|op| op.set_rotary_wrap(None));
                    self.seed_rotary_index_values();
                    if initializing {
                        self.seed_user_rotary_index_options();
                    }
                } else {
                    self.rotary_index_draft = None;
                }
            }
            Action::IndexAxis(axis) => {
                if let Some(index) = &mut self.rotary_index_draft {
                    index.axis = axis;
                }
                self.recalculate_index_face();
            }
            Action::IndexMode(mode) => {
                self.rotary_index_mode = mode;
                self.rotary_pick = None;
                if mode == IndexMode::Manual {
                    if let Some(index) = &mut self.rotary_index_draft {
                        index.face = None;
                    }
                } else {
                    self.rotary_action(Action::Pick(Pick::IndexFace));
                }
            }
            Action::IndexFlip(flip) => {
                if let Some(index) = &mut self.rotary_index_draft {
                    index.flip = flip;
                }
                self.recalculate_index_face();
            }
            Action::IndexValue(i, value) => {
                if i < 4 {
                    if let (Some(index), Ok(v)) =
                        (&mut self.rotary_index_draft, value.parse::<f64>())
                    {
                        if i == 0 {
                            index.angle = v;
                        } else {
                            index.center[i - 1] = v;
                        }
                    }
                    self.rotary_index_values[i] = value;
                }
            }
            Action::Enable(enabled) => {
                self.rotary_index_draft = None;
                self.rotary_index_mode = IndexMode::Manual;
                let current = self
                    .operation_draft
                    .as_ref()
                    .and_then(Operation::rotary_wrap)
                    .cloned();
                if current.is_some() == enabled {
                    return;
                }
                if !enabled {
                    self.rotary_wrap_draft = current.map(|mut wrap| {
                        if let Some(value) = self
                            .fields
                            .get(&Field::Wrap(Number::Offset))
                            .and_then(|s| s.parse().ok())
                        {
                            wrap.offset = value;
                        }
                        wrap
                    });
                }
                let restoring = self.rotary_wrap_draft.is_some();
                let mut wrap = if enabled {
                    self.rotary_wrap_draft.take().unwrap_or_default()
                } else {
                    RotaryWrap::default()
                };
                if let Some(op) = self.operation_draft.as_ref().filter(|_| !restoring) {
                    if let Some(binding) = self.controller.geometry_state().bindings.get(&op.id()) {
                        wrap.geometry = binding.selected.clone();
                    } else if self.cad_pending_kind.is_some() {
                        wrap.geometry = self.controller.cad_selection().to_vec();
                    }
                }
                self.edit_operation_draft(|op| {
                    op.set_rotary_wrap(enabled.then_some(wrap));
                    if enabled {
                        if let Operation::Profile(p) = op {
                            p.comp = cam_model::Comp::Computed;
                        }
                    }
                });
                self.rotary_pick = None;
                self.refresh_operation_fields();
                if enabled && !restoring {
                    self.seed_user_rotary_wrap_options();
                }
            }
            Action::Unwrap(value) => self.edit_operation_draft(|op| {
                if let Some(mut w) = op.rotary_wrap().cloned() {
                    w.unwrap_geometry = value;
                    op.set_rotary_wrap(Some(w));
                }
            }),
            Action::ClearCylinder => self.edit_operation_draft(|op| {
                if let Some(mut w) = op.rotary_wrap().cloned() {
                    w.cylinder = None;
                    op.set_rotary_wrap(Some(w));
                }
            }),
            Action::Pick(pick) => {
                self.rotary_pick = Some(pick);
                self.cad_filter = match pick {
                    Pick::Cylinder | Pick::Faces | Pick::IndexFace => CadFilter::Face,
                    Pick::Edges | Pick::IndexEdge => CadFilter::Edge,
                    Pick::Loops | Pick::IndexLoop => CadFilter::Loop,
                };
                self.cad_filter_explicit = true;
                self.status=if matches!(pick, Pick::IndexFace | Pick::IndexLoop | Pick::IndexEdge) {"Select a planar face, closed planar chain, or straight edge for indexing."} else if pick==Pick::Cylinder {"Select the exterior cylinder face."}else{"Select wrapped boundaries, then Done selecting. Click a selected boundary to remove it."}.into();
            }
            Action::Done => {
                self.rotary_pick = None;
                self.status = "Wrapped geometry selection finished.".into();
            }
            Action::Clear => {
                self.rotary_pick = None;
                self.edit_operation_draft(|op| {
                    if let Some(mut w) = op.rotary_wrap().cloned() {
                        w.geometry.clear();
                        op.set_rotary_wrap(Some(w));
                    }
                });
            }
        }
    }
    fn index_face_angle(&self, entity: &EntityRef) -> Result<f64, String> {
        let geometry = crate::cad_operations::geometry_for_setup(self.controller.document())?;
        let n = cam_geometry3d::index_selection_direction(&geometry, entity)?;
        let index = self
            .rotary_index_draft
            .as_ref()
            .ok_or("Indexing is disabled")?;
        index_angle_for_normal(n, index.axis, index.flip)
    }

    fn recalculate_index_face(&mut self) {
        let Some(face) = self
            .rotary_index_draft
            .as_ref()
            .and_then(|i| i.face.clone())
        else {
            return;
        };
        match self.index_face_angle(&face) {
            Ok(angle) => {
                if let Some(index) = &mut self.rotary_index_draft {
                    index.angle = angle;
                }
                self.rotary_index_values[0] = fmt_num(angle);
            }
            Err(e) => {
                if let Some(index) = &mut self.rotary_index_draft {
                    index.face = None;
                }
                self.report_error(e);
            }
        }
    }
    pub(super) fn rotary_cad_pick(&mut self, entity: &EntityRef) -> bool {
        let Some(pick) = self.rotary_pick else {
            return false;
        };
        if matches!(pick, Pick::IndexFace | Pick::IndexLoop | Pick::IndexEdge) {
            match self.index_face_angle(entity) {
                Ok(angle) => {
                    if let Some(index) = &mut self.rotary_index_draft {
                        index.face = Some(entity.clone());
                        index.angle = angle;
                    }
                    self.rotary_index_values[0] = fmt_num(angle);
                    self.rotary_pick = None;
                    self.status = "Index reference selected.".into();
                }
                Err(e) => self.report_error(e),
            }
            return true;
        }
        let Some(mut wrap) = self
            .operation_draft
            .as_ref()
            .and_then(Operation::rotary_wrap)
            .cloned()
        else {
            self.rotary_pick = None;
            return false;
        };
        if pick == Pick::Cylinder {
            let result = crate::cad_operations::geometry_for_setup(self.controller.document())
                .and_then(|g| cam_geometry3d::wrap_cylinder(&g, entity, wrap));
            match result {
                Ok(w) => {
                    self.edit_operation_draft(|op| op.set_rotary_wrap(Some(w)));
                    self.rotary_pick = None;
                    self.fields
                        .retain(|f, _| !matches!(f, Field::Wrap(n) if n.cylinder_dimension()));
                    self.refresh_operation_fields();
                    self.status = "Wrap cylinder selected.".into();
                }
                Err(e) => self.report_error(e),
            }
        } else {
            if let Some(index) = wrap.geometry.iter().position(|r| r == entity) {
                wrap.geometry.remove(index);
            } else {
                wrap.geometry.push(entity.clone());
            }
            let count = wrap.geometry.len();
            self.edit_operation_draft(|op| op.set_rotary_wrap(Some(wrap)));
            self.status = format!(
                "{count} wrapped boundary selections. Select more or click Done selecting."
            );
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn wrap_inventory_exposes_only_offset_as_an_editable_number() {
        let controller = AppController::new(crate::default_machine());
        let mut operation = controller.operation_template(OpKind::Pocket, 1);
        operation.set_rotary_wrap(Some(RotaryWrap::default()));
        let fields: Vec<_> = operation_fields(&operation)
            .into_iter()
            .filter(|f| matches!(f, Field::Wrap(_)))
            .collect();
        assert_eq!(fields, vec![Field::Wrap(Number::Offset)]);
    }
}
