//! Workpiece axes follow the setup draft and pending picks. A canvas overlay
//! keeps the datum and labels readable even when the origin is inside a solid.
use super::*;
use iced::widget::column;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum SetupOrientation {
    Model,
    Zx,
    Zy,
    Xy,
    SelectedAxes,
}
impl SetupOrientation {
    const ALL: [Self; 5] = [
        Self::Model,
        Self::Zx,
        Self::Zy,
        Self::Xy,
        Self::SelectedAxes,
    ];
    fn axes(self) -> &'static [usize] {
        match self {
            Self::Model => &[],
            Self::Zx => &[2, 0],
            Self::Zy => &[2, 1],
            Self::Xy => &[0, 1],
            Self::SelectedAxes => &[0, 1, 2],
        }
    }
}
impl std::fmt::Display for SetupOrientation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Model => "Model orientation",
            Self::Zx => "Select Z axis and X axis",
            Self::Zy => "Select Z axis and Y axis",
            Self::Xy => "Select X and Y axes",
            Self::SelectedAxes => "Selected axes",
        })
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum SetupOriginMode {
    Model,
    Point,
    StockBox,
    ModelBox,
    TwoPoints,
    Coordinates,
}
impl SetupOriginMode {
    const ALL: [Self; 6] = [
        Self::Model,
        Self::Point,
        Self::StockBox,
        Self::ModelBox,
        Self::TwoPoints,
        Self::Coordinates,
    ];
    pub(super) fn is_box(self) -> bool {
        matches!(self, Self::StockBox | Self::ModelBox)
    }
}
impl std::fmt::Display for SetupOriginMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Model => "Model origin",
            Self::Point => "Selected point",
            Self::StockBox => "Stock box point",
            Self::ModelBox => "Model box point",
            Self::TwoPoints => "From two points",
            Self::Coordinates => "Coordinates",
        })
    }
}
impl App {
    fn setup_orientation(&self, draft: &cam_model::Setup) -> SetupOrientation {
        self.setup_orientation.unwrap_or_else(|| {
            let refs = &draft.frame.references;
            let present = [refs.x.is_some(), refs.y.is_some(), refs.z.is_some()];
            let present: [bool; 3] =
                std::array::from_fn(|i| present[i] || refs.stock_axes[i].is_some());
            match present {
                [true, true, false] => SetupOrientation::Xy,
                [true, false, true] => SetupOrientation::Zx,
                [false, true, true] => SetupOrientation::Zy,
                [false, false, false]
                    if refs.model_instance.is_some()
                        || draft.frame.world_from_setup.rotation
                            == cam_model::SetupFrame::default().world_from_setup.rotation =>
                {
                    SetupOrientation::Model
                }
                _ => SetupOrientation::SelectedAxes,
            }
        })
    }
    pub(super) fn setup_origin_mode(&self) -> SetupOriginMode {
        self.setup_origin_mode.unwrap_or_else(|| {
            match self
                .setup_draft
                .as_ref()
                .map(|draft| &draft.frame.references.origin)
            {
                Some(cam_model::FrameOrigin::ModelOrigin) => SetupOriginMode::Model,
                Some(cam_model::FrameOrigin::SelectedPoint(_)) => SetupOriginMode::Point,
                Some(cam_model::FrameOrigin::StockBoxPoint(_)) => SetupOriginMode::StockBox,
                Some(cam_model::FrameOrigin::ModelBoxPoint(_)) => SetupOriginMode::ModelBox,
                Some(cam_model::FrameOrigin::TwoPoints(_)) => SetupOriginMode::TwoPoints,
                _ => SetupOriginMode::Coordinates,
            }
        })
    }
    pub(super) fn change_setup_orientation(&mut self, mode: SetupOrientation) {
        self.cancel_setup_picks();
        self.cad_candidates.clear();
        self.setup_orientation = Some(mode);
        if mode == SetupOrientation::Model {
            let instance = self.setup_draft.as_ref().and_then(|draft| {
                draft
                    .frame
                    .references
                    .model_instance
                    .or_else(|| draft.model_instances.first().copied())
            });
            if let Some(instance) = instance {
                let _ = self.update(Message::SetupUseModelFrame(instance));
            } else if let Some(draft) = &mut self.setup_draft {
                let world_origin = draft.frame.to_world_vector(draft.origin);
                let origin_reference = draft.frame.references.origin.clone();
                draft.frame = cam_model::SetupFrame::default();
                draft.origin = world_origin;
                draft.frame.references.origin = origin_reference.with_coordinates(draft.origin);
                self.clear_setup_origin_buffers();
            }
            return;
        }
        if let Some(draft) = &mut self.setup_draft {
            let refs = &mut draft.frame.references;
            for axis in 0..3 {
                if !mode.axes().contains(&axis) {
                    match axis {
                        0 => refs.x = None,
                        1 => refs.y = None,
                        _ => refs.z = None,
                    }
                    refs.stock_axes[axis] = None;
                    refs.stock_flipped[axis] = false;
                }
            }
            refs.model_instance = None;
        }
        self.resolve_setup_axes();
    }
    pub(super) fn change_setup_origin_mode(&mut self, mode: SetupOriginMode) {
        self.cancel_setup_picks();
        self.setup_origin_mode = Some(mode);
        if mode == SetupOriginMode::Model {
            let instance = self.setup_draft.as_ref().and_then(|draft| {
                draft
                    .frame
                    .references
                    .model_instance
                    .or_else(|| draft.model_instances.first().copied())
            });
            if let Some(instance) = instance {
                let _ = self.update(Message::SetupUseModelOrigin(instance));
            } else if let Some(draft) = &mut self.setup_draft {
                draft.origin = [0.; 3];
                draft.frame.references.origin = cam_model::FrameOrigin::ModelOrigin;
                self.clear_setup_origin_buffers();
            }
        }
    }
    pub(super) fn setup_wcs_editor(&self, draft: &cam_model::Setup) -> Element<'_, Message> {
        let orientation = self.setup_orientation(draft);
        let origin = self.setup_origin_mode();
        let mut form = column![
            inspector_section("Workpiece coordinate system (WCS)"),
            row![
                text("Orientation").size(12).color(INSPECTOR_TEXT).width(80),
                pick_list(
                    &SetupOrientation::ALL[..],
                    Some(orientation),
                    Message::SetupOrientationChanged
                )
                .text_size(12)
                .padding([7, 9])
                .style(inspector_picker)
                .width(Length::Fill),
            ]
            .spacing(8)
            .align_y(Alignment::Center),
        ]
        .spacing(8);
        if orientation == SetupOrientation::Model {
            // Coordinate systems belong to model instances. Offer every included
            // instance rather than silently taking the first body in a multi-model setup.
            if draft.model_instances.len() > 1 {
                for &id in &draft.model_instances {
                    if let Some(instance) = self.controller.geometry_state().instance(id) {
                        let active = draft.frame.references.model_instance == Some(id);
                        form = form.push(
                            inspector_button(instance.name.clone())
                                .style(move |theme, status| inspector_tab(theme, status, active))
                                .width(Length::Fill)
                                .on_press(Message::SetupUseModelFrame(id)),
                        );
                    }
                }
            }
        } else {
            form = form.push(
                text(if self.setup_axis_pick.is_some() {
                    "Pick a straight model or stock edge in the view."
                } else {
                    "Select axis edges. Z should point away from the machining plane."
                })
                .size(11)
                .color(INSPECTOR_MUTED),
            );
            let refs = &draft.frame.references;
            for &axis in orientation.axes() {
                let reference = [&refs.x, &refs.y, &refs.z][axis].as_ref();
                let stock = refs.stock_axes[axis].is_some();
                let picking = self.setup_axis_pick == Some(axis);
                let label = if picking {
                    "Picking…".to_string()
                } else if stock {
                    "Stock edge".into()
                } else {
                    reference
                        .map(|r| format!("Edge {}", r.entity.id))
                        .unwrap_or_else(|| "Select edge".into())
                };
                form = form.push(
                    row![
                        text(format!("{} axis", ["X", "Y", "Z"][axis]))
                            .size(12)
                            .color(INSPECTOR_TEXT)
                            .width(80),
                        inspector_pick_button(label, picking)
                            .width(Length::Fill)
                            .on_press(Message::SetupAxisPick(axis)),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
                if reference.is_some() || stock {
                    let flipped = reference.map_or(refs.stock_flipped[axis], |r| r.flipped);
                    form = form.push(
                        row![
                            Space::new().width(80),
                            checkbox(flipped)
                                .label(format!("Flip {} axis", ["X", "Y", "Z"][axis]))
                                .size(14)
                                .text_size(11)
                                .spacing(8)
                                .on_toggle(move |on| Message::SetupAxisFlip(axis, on)),
                            Space::new().width(Length::Fill),
                            inspector_button("Clear").on_press(Message::SetupAxisClear(axis)),
                        ]
                        .spacing(8)
                        .align_y(Alignment::Center),
                    );
                }
            }
            if self.setup_axis_pick.is_some() && self.cad_candidates.len() > 1 {
                form = form.push(
                    text("Overlapping edge candidates")
                        .size(11)
                        .color(INSPECTOR_MUTED),
                );
                for candidate in &self.cad_candidates {
                    form = form.push(
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
        form = form.push(
            row![
                text("Origin").size(12).color(INSPECTOR_TEXT).width(80),
                pick_list(
                    &SetupOriginMode::ALL[..],
                    Some(origin),
                    Message::SetupOriginModeChanged
                )
                .text_size(12)
                .padding([7, 9])
                .style(inspector_picker)
                .width(Length::Fill),
            ]
            .spacing(8)
            .align_y(Alignment::Center),
        );
        if matches!(
            origin,
            SetupOriginMode::Point
                | SetupOriginMode::StockBox
                | SetupOriginMode::ModelBox
                | SetupOriginMode::TwoPoints
        ) {
            let picking = self.in_origin_pick();
            let has_point = matches!(
                (origin, &draft.frame.references.origin),
                (
                    SetupOriginMode::Point,
                    cam_model::FrameOrigin::SelectedPoint(_)
                ) | (
                    SetupOriginMode::StockBox,
                    cam_model::FrameOrigin::StockBoxPoint(_)
                ) | (
                    SetupOriginMode::ModelBox,
                    cam_model::FrameOrigin::ModelBoxPoint(_)
                ) | (
                    SetupOriginMode::TwoPoints,
                    cam_model::FrameOrigin::TwoPoints(_)
                )
            );
            let label = if picking {
                "Cancel point pick"
            } else if has_point {
                "Point"
            } else {
                match origin {
                    SetupOriginMode::StockBox | SetupOriginMode::ModelBox => "Point on box",
                    SetupOriginMode::TwoPoints => "Select two points",
                    _ => "Select point",
                }
            };
            form = form.push(
                row![
                    text(if origin == SetupOriginMode::StockBox {
                        "Stock point"
                    } else if origin == SetupOriginMode::ModelBox {
                        "Model point"
                    } else {
                        "Point"
                    })
                    .size(12)
                    .color(INSPECTOR_TEXT)
                    .width(80),
                    inspector_pick_button(label, picking)
                        .width(Length::Fill)
                        .on_press(if origin == SetupOriginMode::TwoPoints {
                            Message::ToggleSetOrigin2pt
                        } else {
                            Message::ToggleSetOrigin
                        }),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
            if origin.is_box() {
                form = form.push(
                    text("Choose a corner, edge midpoint, face center or box center in the view.")
                        .size(11)
                        .color(INSPECTOR_MUTED),
                );
            }
        }
        if origin == SetupOriginMode::Coordinates {
            for i in 0..3 {
                let value = self.setup_value(i, fmt_num(draft.origin[i]));
                form = form.push(inspector_value(
                    ["Origin X", "Origin Y", "Origin Z"][i],
                    &value,
                    "mm",
                    move |v| Message::SetupOriginChanged(i, v),
                ));
            }
        } else {
            form = form.push(
                text(format!(
                    "X {}   Y {}   Z {} mm",
                    fmt_num(draft.origin[0]),
                    fmt_num(draft.origin[1]),
                    fmt_num(draft.origin[2])
                ))
                .size(11)
                .color(INSPECTOR_MUTED),
            );
        }
        if let Some(error) = self.setup_frame_issue(draft) {
            form = form.push(text(error).size(11).color(palette::ERROR));
        }
        form.into()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct WorkpieceAxes {
    pub origin: [f64; 3],
    pub directions: [[f64; 3]; 3],
    pub provisional: bool,
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
impl WorkpieceAxes {
    fn from_setup(setup: &cam_model::Setup, geometry: &cam_model::GeometryState) -> Self {
        let refs = &setup.frame.references;
        let mut axes = std::array::from_fn(|i| {
            setup
                .frame
                .to_world_vector(std::array::from_fn(|j| if i == j { 1. } else { 0. }))
        });
        let resolved = setup.frame.resolve_references(geometry, refs.clone());
        if let Ok(frame) = &resolved {
            axes = std::array::from_fn(|i| {
                frame.to_world_vector(std::array::from_fn(|j| if i == j { 1. } else { 0. }))
            });
        }
        Self {
            origin: setup.frame.to_world_vector(setup.origin),
            directions: axes,
            provisional: resolved.is_err(),
        }
    }
}
impl App {
    pub(super) fn workpiece_axes(&self, picking: &SetupPickGeometry) -> Option<WorkpieceAxes> {
        if self.model_view.is_some()
            || self.controller.selection() != Selection::Setup
            || self.cad_inspecting
            || self.controller.pending_op().is_some()
            || self.cad_pending_kind.is_some()
        {
            return None;
        }
        let mut setup = self
            .setup_draft
            .as_ref()
            .unwrap_or_else(|| self.controller.inspection_setup())
            .clone();
        if let Some(axis) = self.setup_axis_pick.filter(|i| *i < 3) {
            if let Some(direction) = self
                .setup_hover
                .edge
                .as_ref()
                .and_then(|edge| picking.direction(edge))
            {
                let refs = &mut setup.frame.references;
                let target = match axis {
                    0 => &mut refs.x,
                    1 => &mut refs.y,
                    _ => &mut refs.z,
                };
                let flipped = matches!(self.setup_hover.edge, Some(SetupFeature::Model(_)))
                    && target.as_ref().is_some_and(|r| r.flipped);
                *target = None;
                refs.stock_axes[axis] = Some(direction);
                refs.stock_flipped[axis] = flipped;
            }
        }
        let mut axes = WorkpieceAxes::from_setup(&setup, self.controller.geometry_state());
        if self.in_origin_pick() {
            if let Some(hit) = &self.setup_hover.snap {
                axes.origin = if let Some(first) = self.origin_first {
                    let second = setup.frame.to_setup_vector(hit.point);
                    setup
                        .frame
                        .to_world_vector([first[0], second[1], (first[2] + second[2]) / 2.])
                } else {
                    hit.point
                };
            }
        }
        Some(axes)
    }
}

pub(super) struct WorkpieceOverlay {
    pub axes: WorkpieceAxes,
    pub camera: OrbitCamera,
    pub scale: f32,
    pub interactive: bool,
    pub browser_overlay_width: f32,
    pub browser_overlay_height: f32,
}
impl WorkpieceOverlay {
    fn axis_at(&self, bounds: iced::Rectangle, cursor: iced::mouse::Cursor) -> Option<usize> {
        if !self.interactive
            || cursor.position().is_some_and(|point| {
                browser_region_contains(
                    point,
                    bounds,
                    self.browser_overlay_width,
                    self.browser_overlay_height,
                )
            })
        {
            return None;
        }
        let point = cursor.position_in(bounds)?;
        let (origin, tips) = self.projected(bounds.size())?;
        tips.iter()
            .enumerate()
            .filter_map(|(axis, tip)| {
                let delta = *tip - origin;
                let length = delta.x.hypot(delta.y);
                let distance = if length < 7. {
                    point.distance(origin)
                } else {
                    let offset = point - origin;
                    let along = (offset.x * delta.x + offset.y * delta.y) / (length * length);
                    // Keep the shared datum clear so the visible arrows are unambiguous.
                    if along * length < 12. {
                        return None;
                    }
                    point.distance(origin + delta * along.clamp(0., 1.))
                };
                (distance <= 8.).then_some((axis, distance))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(axis, _)| axis)
    }
    fn projected(&self, size: iced::Size) -> Option<(iced::Point, [iced::Point; 3])> {
        if size.width <= 0. || size.height <= 0. {
            return None;
        }
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, size);
        let origin = picking_view::project(self.camera, bounds, self.axes.origin)?.0;
        let length = 64. * self.scale * self.camera.world_per_pixel(size.height)
            / (size.width / size.height).min(1.);
        let tips = self.axes.directions.map(|axis| {
            let point = std::array::from_fn(|i| self.axes.origin[i] + axis[i] * length as f64);
            picking_view::project(self.camera, bounds, point).unwrap().0
        });
        Some((origin, tips))
    }
}
impl canvas::Program<Message> for WorkpieceOverlay {
    type State = ();
    fn update(
        &self,
        _: &mut (),
        event: &canvas::Event,
        bounds: iced::Rectangle,
        cursor: iced::mouse::Cursor,
    ) -> Option<canvas::Action<Message>> {
        if matches!(
            event,
            canvas::Event::Mouse(iced::mouse::Event::ButtonPressed(iced::mouse::Button::Left))
        ) {
            let axis = self.axis_at(bounds, cursor)?;
            return Some(canvas::Action::publish(Message::SetupAxisToggle(axis)).and_capture());
        }
        None
    }
    fn mouse_interaction(
        &self,
        _: &(),
        bounds: iced::Rectangle,
        cursor: iced::mouse::Cursor,
    ) -> iced::mouse::Interaction {
        if self.axis_at(bounds, cursor).is_some() {
            iced::mouse::Interaction::Pointer
        } else {
            iced::mouse::Interaction::default()
        }
    }
    fn draw(
        &self,
        _: &(),
        renderer: &iced::Renderer,
        _: &iced::Theme,
        bounds: iced::Rectangle,
        _: iced::mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let mut frame = canvas::Frame::new(renderer, bounds.size());
        let Some((origin, tips)) = self.projected(bounds.size()) else {
            return vec![];
        };
        let colors = [
            Color::from_rgb(1., 0.32, 0.28),
            Color::from_rgb(0.3, 0.9, 0.4),
            Color::from_rgb(0.3, 0.63, 1.),
        ];
        let stroke = |color, width| {
            canvas::Stroke::default()
                .with_color(color)
                .with_width(width)
        };
        let label = |content: &str, position, color| canvas::Text {
            content: content.into(),
            position,
            color,
            size: 14.0.into(),
            ..Default::default()
        };
        for i in 0..3 {
            let end = tips[i];
            let delta = end - origin;
            let length = delta.x.hypot(delta.y);
            if length < 7. {
                // An axis aimed at the camera still has a readable symbol.
                frame.stroke(&canvas::Path::circle(origin, 6.), stroke(colors[i], 2.));
                let toward = dot(
                    self.axes.directions[i],
                    self.camera.pick_ray(0., 0., 1.).direction,
                ) < 0.;
                if toward {
                    frame.fill(&canvas::Path::circle(origin, 2.), colors[i]);
                } else {
                    frame.stroke(
                        &canvas::Path::line(
                            origin + iced::Vector::new(-3., -3.),
                            origin + iced::Vector::new(3., 3.),
                        ),
                        stroke(colors[i], 1.5),
                    );
                    frame.stroke(
                        &canvas::Path::line(
                            origin + iced::Vector::new(-3., 3.),
                            origin + iced::Vector::new(3., -3.),
                        ),
                        stroke(colors[i], 1.5),
                    );
                }
                frame.fill_text(label(
                    ["X", "Y", "Z"][i],
                    origin + iced::Vector::new(10., -19.),
                    colors[i],
                ));
                continue;
            }
            let direction = delta / length;
            let perpendicular = iced::Vector::new(-direction.y, direction.x);
            let line = canvas::Path::line(origin, end);
            // A dark outline preserves legibility over shaded stock and toolpaths.
            frame.stroke(&line, stroke(Color::from_rgba(0.05, 0.06, 0.07, 0.85), 5.));
            frame.stroke(&line, stroke(colors[i], 2.5));
            let arrow = canvas::Path::new(|p| {
                p.move_to(end);
                p.line_to(end - direction * 10. + perpendicular * 4.);
                p.line_to(end - direction * 10. - perpendicular * 4.);
                p.close();
            });
            frame.fill(&arrow, colors[i]);
            frame.fill_text(label(
                ["X", "Y", "Z"][i],
                end + direction * 9. + iced::Vector::new(-4., -9.),
                colors[i],
            ));
        }
        frame.stroke(&canvas::Path::circle(origin, 3.), stroke(Color::WHITE, 1.5));
        frame.fill_text(label(
            if self.axes.provisional {
                "WCS preview"
            } else {
                "WCS"
            },
            origin + iced::Vector::new(9., 12.),
            Color::WHITE,
        ));
        vec![frame.into_geometry()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn app() -> App {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample").unwrap();
        app.controller.select(Selection::Setup);
        app.begin_setup_draft();
        app
    }
    fn axes(app: &App) -> Option<WorkpieceAxes> {
        let setup = app
            .setup_draft
            .as_ref()
            .unwrap_or_else(|| app.controller.inspection_setup());
        let picking = SetupPickGeometry::new(&app.controller, setup, &app.cad_hidden);
        app.workpiece_axes(&picking)
    }
    fn close(a: [f64; 3], b: [f64; 3]) {
        assert!((0..3).all(|i| (a[i] - b[i]).abs() < 1e-8), "{a:?} != {b:?}");
    }
    #[test]
    fn round_stock_edge_picker_commits_and_clears_a_straight_axis() {
        use cam_model::*;
        let mut app = app();
        app.controller
            .add_cad_asset(
                GeometryAsset {
                    id: 1,
                    revision: 1,
                    name: "Axis body".into(),
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
                            name: "Body".into(),
                            valid_solid: true,
                            bounds: [[0., 0., 0.], [10., 10., 10.]],
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
                                point: [10., 0., 0.],
                            },
                        ],
                        edges: vec![CadEdge {
                            id: 1,
                            body_id: 1,
                            vertices: [1, 2],
                            face_ids: vec![],
                            curve: CadCurve::Line {
                                start: [0., 0., 0.],
                                end: [10., 0., 0.],
                            },
                        }],
                        ..Default::default()
                    }
                    .into(),
                },
                vec![GeometryInstance {
                    id: 1,
                    asset_id: 1,
                    body_id: 1,
                    name: "Body".into(),
                    placement: Default::default(),
                    visible: true,
                }],
            )
            .unwrap();
        app.begin_setup_draft();
        app.setup_draft.as_mut().unwrap().model_instances = vec![1];
        let _ = app.update(Message::SetupStockKindChanged(
            StockEditorKind::RelativeCylinder,
        ));
        let _ = app.update(Message::SetupAxisPick(3));
        app.show_project_browser = false;
        app.show_gizmo = false;
        app.view.yaw = 0.6;
        app.view.pitch = 0.7;
        let viewport = app.machining_viewport();
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(900., 600.));
        let cursor = picking_view::project(viewport.camera(), bounds, [5., 0., 0.])
            .unwrap()
            .0;
        assert!(matches!(
            viewport.setup_hover_at(cursor, bounds).edge,
            Some(SetupFeature::Model(_))
        ));
        use shader::Program;
        let action = viewport
            .update(
                &mut ViewportState::default(),
                &iced::Event::Mouse(iced::mouse::Event::ButtonPressed(iced::mouse::Button::Left)),
                bounds,
                iced::mouse::Cursor::Available(cursor),
            )
            .unwrap();
        let (picked, _, _) = action.into_inner();
        assert!(
            matches!(picked, Some(Message::CadPicked(ref candidates)) if candidates.iter().any(|candidate| candidate.entity.id == 1))
        );
        let entity = EntityRef {
            asset_id: 1,
            revision: 1,
            instance_id: 1,
            body_id: 1,
            kind: EntityKind::Edge,
            id: 1,
        };
        let mut missing = entity.clone();
        missing.id = 99;
        let _ = app.update(Message::CadPicked(vec![PickCandidate {
            entity: missing,
            depth: 0.,
            occluded: false,
        }]));
        assert_eq!(app.setup_axis_pick, Some(3));
        assert!(app.status.contains("missing"));
        let _ = app.update(Message::CadPicked(vec![PickCandidate {
            entity: entity.clone(),
            depth: 0.,
            occluded: false,
        }]));
        assert!(app.setup_axis_pick.is_none());
        let draft = app.setup_draft.as_ref().unwrap();
        assert!(
            matches!(draft.stock, Stock::RelativeRound { axis: Some(axis), .. } if matches!(axis, StockAxis::Edge(edge) if edge.entity() == entity))
        );
        let (frame, _, _) = app
            .controller
            .document()
            .round_stock_geometry(draft)
            .unwrap()
            .unwrap();
        close(frame.to_world_vector([0., 0., 1.]), [1., 0., 0.]);
        let _ = app.update(Message::SetupStockKindChanged(
            StockEditorKind::FixedCylinder,
        ));
        assert!(
            matches!(app.setup_draft.as_ref().unwrap().stock, Stock::FixedRound { axis: Some(axis), .. } if matches!(axis, StockAxis::Edge(edge) if edge.entity() == entity))
        );
        let _ = app.update(Message::SetupStockKindChanged(
            StockEditorKind::RelativeCylinder,
        ));
        assert!(
            matches!(app.setup_draft.as_ref().unwrap().stock, Stock::RelativeRound { axis: Some(axis), .. } if matches!(axis, StockAxis::Edge(edge) if edge.entity() == entity))
        );
        let _ = app.update(Message::SetupCommit);
        assert!(
            matches!(app.controller.inspection_setup().stock, Stock::RelativeRound { axis: Some(axis), .. } if matches!(axis, StockAxis::Edge(edge) if edge.entity() == entity)),
            "{}",
            app.status
        );
        let outcome = app.controller.run(&cam_toolpath::CancelToken::new());
        assert_eq!(
            outcome.completion,
            JobCompletion::Complete,
            "{:?}",
            outcome.diagnostics
        );
        assert!(!outcome.stock_indices.is_empty());
        app.begin_setup_draft();
        let _ = app.update(Message::SetupStockAxisClear);
        assert!(matches!(
            app.setup_draft.as_ref().unwrap().stock,
            Stock::RelativeRound { axis: None, .. }
        ));
    }

    #[test]
    fn orientation_modes_clear_hidden_references_and_preserve_the_world_datum() {
        let mut app = app();
        let committed = app.controller.inspection_setup().clone();
        for (axis, value) in ["12", "23", "34"].into_iter().enumerate() {
            let _ = app.update(Message::SetupOriginChanged(axis, value.into()));
        }
        for mode in [
            SetupOrientation::Zx,
            SetupOrientation::Zy,
            SetupOrientation::Xy,
        ] {
            let _ = app.update(Message::SetupOrientationChanged(mode));
            for &axis in mode.axes() {
                app.set_setup_stock_axis(
                    axis,
                    std::array::from_fn(|i| if i == axis { 1. } else { 0. }),
                );
            }
            let draft = app.setup_draft.as_ref().unwrap();
            for axis in 0..3 {
                assert_eq!(
                    draft.frame.references.stock_axes[axis].is_some(),
                    mode.axes().contains(&axis)
                );
            }
            assert!(app.setup_frame_issue(draft).is_none());
            close(axes(&app).unwrap().origin, [12., 23., 34.]);
        }
        app.set_setup_stock_axis(0, [0., 1., 0.]);
        app.set_setup_stock_axis(1, [0., 0., 1.]);
        let _ = app.update(Message::SetupOrientationChanged(SetupOrientation::Model));
        let draft = app.setup_draft.as_ref().unwrap();
        close(draft.origin, [12., 23., 34.]);
        assert_eq!(
            draft.frame.references.origin,
            cam_model::FrameOrigin::Explicit(draft.origin)
        );
        assert_eq!(app.controller.inspection_setup(), &committed);
        let _ = app.update(Message::SetupCancel);
        assert_eq!(app.controller.inspection_setup(), &committed);
    }

    #[test]
    fn model_orientation_preserves_explicit_datum_and_model_origin_is_separate() {
        use cam_model::{
            CadBody, CadFormat, GeometryAsset, GeometryInstance, RigidTransform, SourceUnits,
            TopologySnapshot,
        };
        let mut app = app();
        let rotation = cam_model::SetupFrame::from_xy([0., 1., 0.], [0., 0., 1.])
            .unwrap()
            .world_from_setup
            .rotation;
        app.controller
            .add_cad_asset(
                GeometryAsset {
                    id: 1,
                    revision: 1,
                    name: "Fixture".into(),
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
                            name: "Body".into(),
                            valid_solid: true,
                            bounds: [[0.; 3], [10.; 3]],
                        }],
                        ..Default::default()
                    }
                    .into(),
                },
                vec![GeometryInstance {
                    id: 1,
                    asset_id: 1,
                    body_id: 1,
                    name: "Body".into(),
                    visible: true,
                    placement: RigidTransform {
                        rotation,
                        translation: [50., 60., 70.],
                    },
                }],
            )
            .unwrap();
        app.setup_draft.as_mut().unwrap().model_instances = vec![1];
        for (axis, value) in ["12", "23", "34"].into_iter().enumerate() {
            let _ = app.update(Message::SetupOriginChanged(axis, value.into()));
        }
        let _ = app.update(Message::SetupOrientationChanged(SetupOrientation::Model));
        close(axes(&app).unwrap().origin, [12., 23., 34.]);
        assert_eq!(
            app.setup_draft
                .as_ref()
                .unwrap()
                .frame
                .world_from_setup
                .rotation,
            rotation
        );
        assert!(app
            .setup_frame_issue(app.setup_draft.as_ref().unwrap())
            .is_none());
        let _ = app.update(Message::SetupOriginModeChanged(SetupOriginMode::Model));
        close(axes(&app).unwrap().origin, [50., 60., 70.]);
        let draft = app.setup_draft.as_ref().unwrap().clone();
        let _ = app.update(Message::SetupCommit);
        assert_eq!(app.controller.inspection_setup(), &draft, "{}", app.status);
    }

    #[test]
    fn origin_source_changes_cancel_pending_picks_and_reset_model_origin() {
        let mut app = app();
        let committed = app.controller.inspection_setup().clone();
        for mode in [
            SetupOriginMode::StockBox,
            SetupOriginMode::ModelBox,
            SetupOriginMode::Point,
            SetupOriginMode::TwoPoints,
        ] {
            app.setup_axis_pick = Some(0);
            app.setting_origin_2pt = true;
            app.origin_first = Some([1., 2., 3.]);
            let _ = app.update(Message::SetupOriginModeChanged(mode));
            assert!(!app.in_origin_pick());
            assert!(app.setup_axis_pick.is_none());
            assert!(app.origin_first.is_none());
            assert_eq!(app.setup_origin_mode(), mode);
        }
        let _ = app.update(Message::SetupOriginChanged(0, "42".into()));
        assert_eq!(app.setup_origin_mode(), SetupOriginMode::Coordinates);
        let _ = app.update(Message::SetupOriginModeChanged(SetupOriginMode::Model));
        close(app.setup_draft.as_ref().unwrap().origin, [0.; 3]);
        assert!(!app.setup_values.contains_key(&0));
        assert_eq!(app.controller.inspection_setup(), &committed);
    }

    #[test]
    fn picked_origin_source_survives_reopening_and_serialization() {
        for mode in [
            SetupOriginMode::Point,
            SetupOriginMode::StockBox,
            SetupOriginMode::ModelBox,
            SetupOriginMode::TwoPoints,
        ] {
            let mut app = app();
            let _ = app.update(Message::SetupOriginModeChanged(mode));
            if mode == SetupOriginMode::TwoPoints {
                let _ = app.update(Message::ToggleSetOrigin2pt);
                let _ = app.update(Message::OriginPointPicked([1., 2., 3.]));
            } else {
                let _ = app.update(Message::ToggleSetOrigin);
            }
            let _ = app.update(Message::OriginPointPicked([4., 5., 6.]));
            let _ = app.update(Message::SetupOrientationChanged(SetupOrientation::Model));
            let _ = app.update(Message::SetupCommit);
            let saved = app.controller.inspection_setup().clone();
            let json = serde_json::to_string(&saved).unwrap();
            let restored: cam_model::Setup = serde_json::from_str(&json).unwrap();
            assert_eq!(
                restored.frame.references.origin,
                saved.frame.references.origin
            );
            app.begin_setup_draft();
            assert_eq!(app.setup_origin_mode(), mode);
            assert_eq!(app.setup_draft.as_ref().unwrap().origin, saved.origin);
        }
    }

    #[test]
    fn escape_aborts_wcs_edge_pick_and_preserves_setup_edits() {
        let mut app = app();
        let _ = app.update(Message::SetupNameChanged("Fixture draft".into()));
        let before = app.setup_draft.clone().unwrap();
        let committed = app.controller.document().clone();
        for axis in 0..3 {
            let _ = app.update(Message::SetupAxisPick(axis));
            assert_eq!(app.setup_axis_pick, Some(axis));
            app.setup_hover.edge = Some(SetupFeature::Stock(0));
            let _ = app.update(Message::CadEscape);
            assert!(app.setup_axis_pick.is_none());
            assert!(app.setup_hover.edge.is_none());
            assert_eq!(app.setup_draft.as_ref(), Some(&before));
            assert_eq!(app.controller.document(), &committed);
            assert!(!app.machining_viewport().setup_axis);
        }
    }
    #[test]
    fn selected_setup_always_shows_axes_and_numeric_origin_edits_preview_at_true_z() {
        let mut app = app();
        app.show_origin = false;
        assert!(axes(&app).is_some());
        let committed = app.controller.inspection_setup().clone();
        for (i, value) in ["12.5", "23.5", "7.5"].into_iter().enumerate() {
            let _ = app.update(Message::SetupOriginChanged(i, value.into()));
        }
        close(axes(&app).unwrap().origin, [12.5, 23.5, 7.5]);
        assert_eq!(app.controller.inspection_setup(), &committed);
        let _ = app.update(Message::SetupCancel);
        close(
            axes(&app).unwrap().origin,
            committed.frame.to_world_vector(committed.origin),
        );
        app.controller.select(Selection::Machine);
        assert!(axes(&app).is_none());
    }
    #[test]
    fn first_edge_then_second_edge_and_flips_preview_without_applying() {
        let mut app = app();
        let committed = app.controller.inspection_setup().clone();
        app.set_setup_stock_axis(0, [0., 1., 0.]);
        let first = axes(&app).unwrap();
        assert!(!first.provisional);
        close(first.directions[0], [0., 1., 0.]);
        close(
            first.origin,
            committed.frame.to_world_vector(committed.origin),
        );
        app.set_setup_stock_axis(1, [0., 0., 1.]);
        let second = axes(&app).unwrap();
        assert!(!second.provisional);
        assert_eq!(
            second.directions,
            [[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]]
        );
        let _ = app.update(Message::SetupAxisFlip(0, true));
        let flipped = axes(&app).unwrap();
        assert_eq!(
            flipped.directions,
            [[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]]
        );
        assert_eq!(app.controller.inspection_setup(), &committed);
        let _ = app.update(Message::SetupCancel);
        assert_eq!(
            axes(&app).unwrap().directions,
            [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
        );
    }
    #[test]
    fn a_single_axis_can_apply_round_trip_and_undo_with_the_preview_orientation() {
        for axis in 0..3 {
            let mut app = app();
            let committed = app.controller.inspection_setup().clone();
            for (index, value) in ["12", "23", "34"].into_iter().enumerate() {
                let _ = app.update(Message::SetupOriginChanged(index, value.into()));
            }
            app.set_setup_stock_axis(axis, [1., 2., 3.]);
            let preview = axes(&app).unwrap();
            close(preview.origin, [12., 23., 34.]);
            let draft = app.setup_draft.clone().unwrap();
            assert!(app.setup_frame_issue(&draft).is_none());
            let _ = app.update(Message::SetupCommit);
            assert_eq!(app.controller.inspection_setup(), &draft, "{}", app.status);
            let saved = app.controller.inspection_setup();
            for index in 0..3 {
                close(
                    saved.frame.to_world_vector(std::array::from_fn(|i| {
                        if i == index {
                            1.
                        } else {
                            0.
                        }
                    })),
                    preview.directions[index],
                );
            }
            let json = serde_json::to_string(app.controller.document()).unwrap();
            let loaded: cam_model::Document = serde_json::from_str(&json).unwrap();
            assert!(app.setup_frame_issue(loaded.setup()).is_none());
            assert_eq!(loaded.setup().frame, draft.frame);
            assert!(app.controller.undo());
            assert_eq!(app.controller.inspection_setup(), &committed);
            assert!(app.controller.redo());
            assert_eq!(app.controller.inspection_setup(), &draft);
        }
    }
    #[test]
    fn arrow_flips_preserve_handedness_and_datum_for_every_reference_combination() {
        for mask in 0..8 {
            for axis in 0..3 {
                let mut app = app();
                for index in 0..3 {
                    if mask & (1 << index) != 0 {
                        app.set_setup_stock_axis(
                            index,
                            std::array::from_fn(|i| if i == index { 1. } else { 0. }),
                        );
                    }
                }
                for (index, value) in ["12", "23", "34"].into_iter().enumerate() {
                    let _ = app.update(Message::SetupOriginChanged(index, value.into()));
                }
                let before = app.setup_draft.clone().unwrap();
                let original = axes(&app).unwrap();
                let _ = app.update(Message::SetupAxisToggle(axis));
                let flipped = axes(&app).unwrap();
                close(
                    flipped.directions[axis],
                    original.directions[axis].map(|v| -v),
                );
                close(flipped.origin, original.origin);
                let draft = app.setup_draft.clone().unwrap();
                assert!(
                    app.setup_frame_issue(&draft).is_none(),
                    "mask={mask}, axis={axis}"
                );
                app.controller.apply_setup_settings(draft).unwrap();
                let _ = app.update(Message::SetupAxisToggle(axis));
                assert_eq!(app.setup_draft.as_ref().unwrap(), &before);
            }
        }
    }
    #[test]
    fn round_stock_can_pick_origin_arrows_without_flipping_the_wcs() {
        use canvas::Program;
        let mut app = app();
        let original = app.setup_draft.as_ref().unwrap().frame.clone();
        let _ = app.update(Message::SetupStockKindChanged(
            StockEditorKind::RelativeCylinder,
        ));
        let _ = app.update(Message::SetupAxisPick(3));
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(900., 600.));
        let mut camera = OrbitCamera::framed([-50.; 3], [50.; 3]);
        camera.orient = cam_render::orientation(0.6, 0.7);
        let overlay = WorkpieceOverlay {
            axes: axes(&app).unwrap(),
            camera,
            scale: 1.,
            interactive: true,
            browser_overlay_width: 0.,
            browser_overlay_height: f32::INFINITY,
        };
        let (_, tips) = overlay.projected(bounds.size()).unwrap();
        let action = overlay
            .update(
                &mut (),
                &canvas::Event::Mouse(iced::mouse::Event::ButtonPressed(iced::mouse::Button::Left)),
                bounds,
                iced::mouse::Cursor::Available(tips[0]),
            )
            .unwrap();
        let (message, _, _) = action.into_inner();
        let _ = app.update(message.unwrap());
        assert!(matches!(
            app.setup_draft.as_ref().unwrap().stock,
            cam_model::Stock::RelativeRound {
                axis: Some(cam_model::StockAxis::Origin {
                    origin_axis: cam_model::StockOriginAxis::X
                }),
                ..
            }
        ));
        assert_eq!(app.setup_draft.as_ref().unwrap().frame, original);
        assert!(app.setup_axis_pick.is_none());
        for (index, expected) in [
            cam_model::StockOriginAxis::X,
            cam_model::StockOriginAxis::Y,
            cam_model::StockOriginAxis::Z,
        ]
        .into_iter()
        .enumerate()
        {
            let _ = app.update(Message::SetupStockOriginAxis(index));
            assert!(
                matches!(app.setup_draft.as_ref().unwrap().stock, cam_model::Stock::RelativeRound { axis: Some(cam_model::StockAxis::Origin { origin_axis }), .. } if origin_axis == expected)
            );
        }
        let _ = app.update(Message::SetupCommit);
        assert!(
            matches!(
                app.controller.inspection_setup().stock,
                cam_model::Stock::RelativeRound {
                    axis: Some(cam_model::StockAxis::Origin {
                        origin_axis: cam_model::StockOriginAxis::Z
                    }),
                    ..
                }
            ),
            "{}",
            app.status
        );
        app.begin_setup_draft();
        let _ = app.update(Message::SetupStockAxisClear);
        assert!(matches!(
            app.setup_draft.as_ref().unwrap().stock,
            cam_model::Stock::RelativeRound { axis: None, .. }
        ));
    }

    #[test]
    fn viewport_arrows_publish_flip_clicks_and_leave_other_clicks_available() {
        use canvas::Program;
        let app = app();
        let bounds = iced::Rectangle::new(iced::Point::new(50., 80.), iced::Size::new(900., 600.));
        let mut camera = OrbitCamera::framed([-50.; 3], [50.; 3]);
        camera.orient = cam_render::orientation(0.6, 0.7);
        let overlay = WorkpieceOverlay {
            axes: axes(&app).unwrap(),
            camera,
            scale: 1.,
            interactive: true,
            browser_overlay_width: 0.,
            browser_overlay_height: f32::INFINITY,
        };
        let (_, tips) = overlay.projected(bounds.size()).unwrap();
        for (axis, point) in tips.into_iter().enumerate() {
            let cursor =
                iced::mouse::Cursor::Available(point + iced::Vector::new(bounds.x, bounds.y));
            assert_eq!(overlay.axis_at(bounds, cursor), Some(axis));
            assert_eq!(
                overlay.mouse_interaction(&(), bounds, cursor),
                iced::mouse::Interaction::Pointer
            );
            let action = overlay
                .update(
                    &mut (),
                    &canvas::Event::Mouse(iced::mouse::Event::ButtonPressed(
                        iced::mouse::Button::Left,
                    )),
                    bounds,
                    cursor,
                )
                .unwrap();
            let (message, _, _) = action.into_inner();
            assert!(matches!(message, Some(Message::SetupAxisToggle(index)) if index == axis));
        }
        let tip = iced::mouse::Cursor::Available(tips[0] + iced::Vector::new(bounds.x, bounds.y));
        let mut overlay = overlay;
        overlay.interactive = false;
        assert!(overlay.axis_at(bounds, tip).is_none());
        overlay.interactive = true;
        overlay.browser_overlay_width = bounds.width;
        assert!(overlay.axis_at(bounds, tip).is_none());
        overlay.browser_overlay_width = 0.;
        let cursor = iced::mouse::Cursor::Available(iced::Point::new(bounds.x + 5., bounds.y + 5.));
        assert!(overlay
            .update(
                &mut (),
                &canvas::Event::Mouse(iced::mouse::Event::ButtonPressed(iced::mouse::Button::Left)),
                bounds,
                cursor
            )
            .is_none());
    }
    #[test]
    fn edge_hover_and_snapped_origin_preview_are_temporary() {
        let mut app = app();
        let before = app.setup_draft.clone().unwrap();
        let picking = SetupPickGeometry::new(&app.controller, &before, &app.cad_hidden);
        let (id, _) = picking
            .stock_lines()
            .find(|(_, edge)| (edge[1][1] - edge[0][1]).abs() > 1.)
            .unwrap();
        app.setup_axis_pick = Some(0);
        app.setup_hover.edge = Some(SetupFeature::Stock(id));
        let hover = app.workpiece_axes(&picking).unwrap();
        close(hover.directions[0], [0., 1., 0.]);
        assert_eq!(app.setup_draft.as_ref().unwrap(), &before);
        app.setup_axis_pick = None;
        app.setting_origin = true;
        app.setup_hover.snap = Some(picking_view::SetupSnap {
            point: [4., 5., 6.],
            kind: SnapKind::End,
            source: SetupFeature::Stock(id),
        });
        close(app.workpiece_axes(&picking).unwrap().origin, [4., 5., 6.]);
        app.setting_origin = false;
        app.setting_origin_2pt = true;
        app.origin_first = Some([10., 20., 30.]);
        close(app.workpiece_axes(&picking).unwrap().origin, [10., 5., 18.]);
        assert_eq!(app.setup_draft.as_ref().unwrap(), &before);
    }
    #[test]
    fn triad_stays_at_the_datum_and_legible_at_different_zooms_and_aspects() {
        let app = app();
        let mut axes = axes(&app).unwrap();
        axes.origin = [8., 12., 25.];
        let mut camera = OrbitCamera::framed([-50.; 3], [50.; 3]);
        for zoom in [0., 2., -2.] {
            camera.zoom = zoom;
            for size in [iced::Size::new(900., 600.), iced::Size::new(400., 900.)] {
                let overlay = WorkpieceOverlay {
                    axes: axes.clone(),
                    camera,
                    scale: 1.,
                    interactive: true,
                    browser_overlay_width: 0.,
                    browser_overlay_height: f32::INFINITY,
                };
                let (origin, tips) = overlay.projected(size).unwrap();
                let expected = picking_view::project(
                    camera,
                    iced::Rectangle::new(iced::Point::ORIGIN, size),
                    axes.origin,
                )
                .unwrap()
                .0;
                assert!(origin.distance(expected) < 0.01);
                assert!((origin.distance(tips[0]) - 64.).abs() < 0.01);
                assert!((origin.distance(tips[1]) - 64.).abs() < 0.01);
                assert!(origin.distance(tips[2]) < 0.01);
            }
        }
    }
}
