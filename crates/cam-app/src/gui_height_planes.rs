//! Operation height preview: machining-plane rectangles and labels.
use super::*;

pub(super) fn field_id(field: Field) -> String {
    format!("height-{field:?}")
}

#[derive(Clone, Debug)]
pub(super) struct HeightPlane {
    pub name: &'static str,
    pub z: f64,
    pub value_label: String,
    pub color: Color,
    pub active: bool,
    pub corners: [[f32; 3]; 4],
    pub outlines: Vec<Vec<[f32; 3]>>,
}

pub(super) struct HeightOverlay {
    pub camera: OrbitCamera,
    pub planes: Vec<HeightPlane>,
    pub browser_width: f32,
}

impl App {
    pub(super) fn preview_safety_heights(&self, id: u32) -> cam_model::Heights {
        let saved = self.controller.inspection_setup().heights_for(id);
        let value = |field, fallback| {
            self.fields
                .get(&field)
                .and_then(|s| s.trim().parse::<f64>().ok())
                .filter(|v| v.is_finite())
                .unwrap_or(fallback)
        };
        cam_model::Heights::new(
            value(Field::Clearance, saved.clearance),
            value(Field::Retract, saved.retract),
            value(Field::TopOfStock, saved.top_of_stock),
        )
    }

    pub(super) fn height_input_ids(&self) -> Vec<String> {
        let mut ids: Vec<_> = self
            .inspector_fields()
            .into_iter()
            .filter(|f| operation_tabs::field_tab(*f) == OperationTab::Heights)
            .map(field_id)
            .collect();
        ids.extend(["height-cad-top".into(), "height-cad-bottom".into()]);
        ids.extend(
            cam_model::HeightPlaneKind::ALL
                .into_iter()
                .map(super::height_editor::plane_input_id),
        );
        ids
    }

    pub(super) fn height_preview(
        &self,
        camera: OrbitCamera,
        browser_width: f32,
    ) -> Option<HeightOverlay> {
        let focused = self
            .height_reference_pick
            .map(super::height_editor::plane_input_id)
            .or_else(|| self.focused_height.clone());
        if self.workspace != Workspace::Machining
            || self.operation_tab != OperationTab::Heights
            || !self.inspector_enabled
            || !self.inspector_active
            || self.cad_editing_op.is_some()
            || self.drawing_editing_op.is_some()
        {
            return None;
        }
        let id = if self.creating_operation() {
            0
        } else if let Selection::Operation(id) = self.controller.selection() {
            id
        } else {
            return None;
        };
        let heights = self.resolve_height_editor(id).ok()?;
        let setup = self.controller.inspection_setup();
        let document = self.rotary_draft_document();
        let indexed = document.indexed_view(id).ok()?;
        let (min, max) = indexed.as_ref().map_or_else(
            || self.controller.stock_box(),
            |view| view.stock_bounds_for(view.setup()),
        );
        let mut operation = self.current_operation(id)?.clone();
        let parsed = self.parsed_operation_fields();
        apply_op_fields(&mut operation, &parsed);
        if operation.rotary_wrap().is_some() {
            let geometry =
                crate::cad_operations::geometry_for_setup(self.controller.document()).ok()?;
            crate::cad_operations::resolve_wrapped_operation(&geometry, &mut operation).ok()?;
        }
        let pad = (max[0] - min[0]).max(max[1] - min[1]).max(1.) * 0.08;
        let colors = [
            Color::from_rgb(1., 0.38, 0.16),
            Color::from_rgb(0.87, 0.75, 0.16),
            Color::from_rgb(0.25, 0.94, 0.35),
            Color::from_rgb(0.12, 0.78, 1.),
            Color::from_rgb(0.77, 0.52, 1.),
        ];
        let names = ["Clearance", "Retract", "Feed", "Top", "Bottom"];
        let planes = cam_model::HeightPlaneKind::ALL
            .into_iter()
            .filter_map(|plane| {
                let z = heights.plane(plane)?;
                if operation
                    .rotary_wrap()
                    .is_some_and(|w| w.effective_radius() + z - w.surface_z <= 0.)
                {
                    return None;
                }
                let corners = [
                    [min[0] - pad, min[1] - pad, z],
                    [max[0] + pad, min[1] - pad, z],
                    [max[0] + pad, max[1] + pad, z],
                    [min[0] - pad, max[1] + pad, z],
                ]
                .map(|p| {
                    rotary_view::index_point_to_world(
                        &setup.frame,
                        self.rotary_index_draft.as_ref(),
                        p,
                    )
                    .map(|v| v as f32)
                });
                let outlines = operation.rotary_wrap().map_or_else(Vec::new, |wrap| {
                    wrapped_plane_outlines(wrap, z, min, max, &setup.frame)
                });
                Some(HeightPlane {
                    name: names[plane.index()],
                    z,
                    value_label: self.height_display_value(id, plane, z),
                    color: colors[plane.index()],
                    active: focused.as_deref()
                        == Some(super::height_editor::plane_input_id(plane).as_str()),
                    corners,
                    outlines,
                })
            })
            .collect();
        Some(HeightOverlay {
            camera,
            planes,
            browser_width,
        })
    }
}

fn wrapped_plane_outlines(
    wrap: &cam_model::RotaryWrap,
    z: f64,
    min: [f64; 3],
    max: [f64; 3],
    frame: &cam_model::SetupFrame,
) -> Vec<Vec<[f32; 3]>> {
    let radius = wrap.effective_radius() + z - wrap.surface_z;
    if !radius.is_finite() || radius <= 0. {
        return Vec::new();
    }
    let axial = match wrap.axis {
        cam_model::RotaryAxis::A => 0,
        cam_model::RotaryAxis::B => 1,
    };
    let pad = (max[axial] - min[axial]).max(1.) * 0.08;
    let ends = [min[axial] - pad, max[axial] + pad];
    let point = |along: f64, angle: f64| {
        let (s, c) = angle.sin_cos();
        let p = match wrap.axis {
            cam_model::RotaryAxis::A => [
                along,
                wrap.center[1] + radius * s,
                wrap.center[2] + radius * c,
            ],
            cam_model::RotaryAxis::B => [
                wrap.center[0] + radius * s,
                along,
                wrap.center[2] + radius * c,
            ],
        };
        frame.to_world_vector(p).map(|v| v as f32)
    };
    let mut paths: Vec<_> = ends
        .into_iter()
        .map(|along| {
            (0..=128)
                .map(|i| point(along, std::f64::consts::TAU * i as f64 / 128.))
                .collect()
        })
        .collect();
    paths.extend((0..4).map(|i| {
        ends.into_iter()
            .map(|along| point(along, std::f64::consts::FRAC_PI_2 * i as f64))
            .collect()
    }));
    paths
}

fn project(camera: OrbitCamera, point: [f32; 3], size: iced::Size) -> iced::Point {
    let matrix = camera.view_proj(size.width / size.height.max(1.));
    let clip: [f32; 4] = std::array::from_fn(|r| {
        matrix[0][r] * point[0] + matrix[1][r] * point[1] + matrix[2][r] * point[2] + matrix[3][r]
    });
    iced::Point::new(
        (clip[0] / clip[3] + 1.) * size.width / 2.,
        (1. - clip[1] / clip[3]) * size.height / 2.,
    )
}

impl canvas::Program<Message> for HeightOverlay {
    type State = ();
    fn draw(
        &self,
        _: &(),
        renderer: &iced::Renderer,
        _: &iced::Theme,
        bounds: iced::Rectangle,
        _: iced::mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let mut frame = canvas::Frame::new(renderer, bounds.size());
        let left = if self.browser_width > 0. {
            self.browser_width + 16.
        } else {
            8.
        };
        // Finish every outline before painting labels so another plane's edge
        // cannot run through a label (especially at coincident top planes).
        for plane in &self.planes {
            if !plane.z.is_finite() {
                continue;
            }
            let outlines = if plane.outlines.is_empty() {
                vec![plane
                    .corners
                    .into_iter()
                    .chain(std::iter::once(plane.corners[0]))
                    .collect()]
            } else {
                plane.outlines.clone()
            };
            for outline in outlines {
                let points: Vec<_> = outline
                    .into_iter()
                    .map(|p| project(self.camera, p, bounds.size()))
                    .collect();
                if points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
                    continue;
                }
                let path = canvas::Path::new(|p| {
                    p.move_to(points[0]);
                    for point in &points[1..] {
                        p.line_to(*point);
                    }
                });
                frame.stroke(
                    &path,
                    canvas::Stroke::default()
                        .with_color(plane.color)
                        .with_width(if plane.active { 2.5 } else { 1.2 }),
                );
            }
        }
        let mut labels: Vec<iced::Rectangle> = Vec::new();
        let mut captions = Vec::new();
        for (index, plane) in self.planes.iter().enumerate() {
            let points: Vec<_> = if let Some(ring) = plane.outlines.first() {
                ring.iter()
                    .map(|p| project(self.camera, *p, bounds.size()))
                    .collect()
            } else {
                plane
                    .corners
                    .map(|p| project(self.camera, p, bounds.size()))
                    .to_vec()
            };
            if points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
                continue;
            }
            // Distribute labels along the same edge; resolve collisions for top views
            // and coincident stock/cutting planes without losing their true Z outline.
            let t = (index as f32 + 0.5) / self.planes.len().max(1) as f32;
            let anchor = if plane.outlines.is_empty() {
                points[0] + (points[1] - points[0]) * t
            } else {
                points[((points.len() - 1) as f32 * t) as usize]
            };
            let content = format!("{}: {} mm", plane.name, plane.value_label);
            let width = content.len() as f32 * 6.8 + 16.;
            let mut rect = iced::Rectangle {
                x: anchor.x.clamp(
                    left.min((bounds.width - width - 8.).max(0.)),
                    (bounds.width - width - 8.).max(left),
                ),
                y: (anchor.y - 23.).clamp(8., (bounds.height - 30.).max(8.)),
                width,
                height: 22.,
            };
            for _ in 0..self.planes.len() {
                if !labels.iter().any(|r| r.intersects(&rect)) {
                    break;
                }
                rect.y += 25.;
                if rect.y + rect.height > bounds.height - 8. {
                    rect.y = 8.;
                }
            }
            labels.push(rect);
            let target = iced::Point::new(
                anchor.x.clamp(rect.x, rect.x + rect.width),
                anchor.y.clamp(rect.y, rect.y + rect.height),
            );
            frame.stroke(
                &canvas::Path::line(anchor, target),
                canvas::Stroke::default()
                    .with_color(plane.color)
                    .with_width(1.),
            );
            captions.push((plane, content, rect));
        }
        for (plane, content, rect) in captions {
            frame.fill_rectangle(rect.position(), rect.size(), plane.color);
            frame.fill_text(canvas::Text {
                content,
                position: rect.position() + iced::Vector::new(8., 3.),
                size: 12.0.into(),
                color: Color::from_rgb(0.05, 0.07, 0.09),
                ..Default::default()
            });
        }
        vec![frame.into_geometry()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn operation_editor() -> (App, u32) {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample.dxf").unwrap();
        // Height expectations use the drawing surface as stock top.
        app.controller.edit_stock(|s| {
            *s = cam_model::Stock::RelativeBox {
                offsets: [[1., 1., 0.]; 2],
            }
        });
        app.controller.new_operation(OpKind::Profile);
        let id = app
            .controller
            .inspection_setup()
            .operations
            .last()
            .unwrap()
            .id();
        app.begin_operation_draft(id);
        let _ = app.update(Message::OperationTabChanged(OperationTab::Heights));
        (app, id)
    }

    #[test]
    fn reference_offsets_are_transactional_persisted_and_operation_scoped() {
        use cam_model::HeightPlaneKind as P;
        let (mut app, id) = operation_editor();
        let original = app.controller.document().clone();
        let _ = app.update(Message::HeightOffsetChanged(P::Clearance, "20".into()));
        let _ = app.update(Message::HeightOffsetChanged(P::Retract, "8".into()));
        assert_eq!(app.resolve_height_editor(id).unwrap().clearance, 28.);
        assert_eq!(app.controller.document(), &original);
        let _ = app.update(Message::OperationTabChanged(OperationTab::Tool));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Heights));
        assert_eq!(app.height_offsets[&P::Clearance], "20");
        let _ = app.update(Message::Apply);
        assert_eq!(
            app.controller.inspection_setup().height_settings[&id]
                .definition(P::Clearance)
                .offset,
            20.
        );
        assert_eq!(
            app.controller.inspection_setup().heights,
            original.setup().heights
        );
        assert!(app
            .controller
            .inspection_setup()
            .operations
            .iter()
            .filter(|o| o.id() != id)
            .all(|o| !app
                .controller
                .inspection_setup()
                .height_settings
                .contains_key(&o.id())));
        let path = std::env::temp_dir().join(format!(
            "opencam-relative-heights-{}.ocam",
            std::process::id()
        ));
        app.controller.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert_eq!(
            reopened.document().setup().height_settings[&id],
            app.controller.inspection_setup().height_settings[&id]
        );
        let _ = app.update(Message::Undo);
        assert_eq!(app.controller.document(), &original);
        assert!(!app.inspector_dirty());
        let _ = app.update(Message::Redo);
        assert!(!app.inspector_dirty());
        app.begin_operation_draft(id);
        let _ = app.update(Message::HeightOffsetChanged(P::Clearance, "40".into()));
        let _ = app.update(Message::CancelOperationDraft);
        assert_eq!(app.height_offsets[&P::Clearance], "20");
    }
    #[test]
    fn all_five_planes_preview_live_offsets_in_the_setup_orientation_while_heights_is_selected() {
        use cam_model::HeightPlaneKind as P;
        let (mut app, id) = operation_editor();
        let camera = OrbitCamera::framed([0., 0., -10.], [100., 80., 25.]);
        let preview = app.height_preview(camera, 0.).unwrap();
        assert_eq!(preview.planes.len(), 5);
        assert!(preview.planes.iter().all(|p| !p.active));
        let _ = app.update(Message::HeightOffsetChanged(P::Clearance, "23.5".into()));
        let preview = app.height_preview(camera, 0.).unwrap();
        assert_eq!(preview.planes.len(), 5);
        assert!(preview.planes[0].active);
        assert_eq!(preview.planes[0].z, 28.5);
        assert!(preview.planes[0].corners.iter().all(|p| p[2] == 28.5));
        let _ = app.update(Message::HeightOffsetChanged(P::Bottom, "-0.25".into()));
        let preview = app.height_preview(camera, 0.).unwrap();
        assert_eq!(
            preview
                .planes
                .iter()
                .find(|p| p.name == "Bottom")
                .unwrap()
                .z,
            -4.25
        );
        app.controller.edit_setup(|s| {
            s.frame.world_from_setup.rotation = [[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]]
        });
        let preview = app.height_preview(camera, 0.).unwrap();
        assert!(preview.planes[0]
            .corners
            .iter()
            .all(|p| p[0] == preview.planes[0].z as f32));
        let _ = app.update(Message::HeightFocus(
            super::super::height_editor::plane_input_id(P::Bottom),
            false,
        ));
        let preview = app.height_preview(camera, 0.).unwrap();
        assert_eq!(preview.planes.len(), 5);
        assert!(preview.planes.iter().all(|p| !p.active));
        let _ = app.update(Message::HeightOffsetChanged(P::Clearance, "24".into()));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Tool));
        assert!(app.height_preview(camera, 0.).is_none());
        assert!(app
            .controller
            .inspection_setup()
            .height_settings
            .contains_key(&id));
    }

    #[test]
    fn indexed_height_planes_match_rotated_stock_and_machine_frame_for_both_axes() {
        use cam_model::{HeightPlaneKind as P, HeightReference as R, RotaryAxis};
        let (mut app, id) = operation_editor();
        app.controller.edit_stock(|stock| {
            *stock = cam_model::Stock::FixedBox {
                size: [30., 20., 12.],
                model_position: [0.3, 0.4, 0.2],
            }
        });
        app.controller.edit_setup(|setup| {
            setup.frame.world_from_setup.rotation = [[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]];
        });
        app.rotary_action(rotary_view::Action::Indexed(true));
        for (plane, reference, offset) in [
            (P::Top, R::StockTop, 0.),
            (P::Bottom, R::StockBottom, 0.),
            (P::Feed, R::Plane(P::Top), 2.),
            (P::Retract, R::Plane(P::Top), 4.),
            (P::Clearance, R::Plane(P::Retract), 6.),
        ] {
            *app.height_draft.as_mut().unwrap().definition_mut(plane) =
                cam_model::HeightDefinition::new(reference, offset);
            app.height_offsets.insert(plane, fmt_num(offset));
        }
        let (min, max) = app.controller.stock_box();
        let frame = app.controller.inspection_setup().frame.clone();
        let camera = OrbitCamera::framed([-50.; 3], [100.; 3]);
        for (axis, angle) in [
            (RotaryAxis::A, 90.),
            (RotaryAxis::B, -45.),
            (RotaryAxis::A, 270.),
        ] {
            app.rotary_action(rotary_view::Action::IndexAxis(axis));
            for (i, value) in [(0, angle), (1, 2.), (2, 3.), (3, -4.)] {
                app.rotary_action(rotary_view::Action::IndexValue(i, value.to_string()));
            }
            let center = cam_cldata::Point3::new(2., 3., -4.);
            let rotated: Vec<_> = (0..8)
                .map(|bits| {
                    let p = std::array::from_fn::<_, 3, _>(|i| {
                        if bits & (1 << i) == 0 {
                            min[i]
                        } else {
                            max[i]
                        }
                    });
                    axis.material_point(cam_cldata::Point3::new(p[0], p[1], p[2]), center, -angle)
                })
                .collect();
            let zmin = rotated.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
            let zmax = rotated
                .iter()
                .map(|p| p.z)
                .fold(f64::NEG_INFINITY, f64::max);
            let heights = app.resolve_height_editor(id).unwrap();
            assert!((heights.top - zmax).abs() < 1e-8);
            assert!((heights.bottom - zmin).abs() < 1e-8);
            assert!((heights.clearance - zmax - 10.).abs() < 1e-8);
            let overlay = app.height_preview(camera, 0.).unwrap();
            assert_eq!(overlay.planes.len(), 5);
            for plane in overlay.planes {
                for point in plane.corners {
                    let p = frame.to_setup_vector(point.map(f64::from));
                    let machine = axis.material_point(
                        cam_cldata::Point3::new(p[0], p[1], p[2]),
                        center,
                        -angle,
                    );
                    assert!(
                        (machine.z - plane.z).abs() < 1e-4,
                        "{} {axis:?} {angle}",
                        plane.name
                    );
                }
                assert!((plane.value_label.parse::<f64>().unwrap() - plane.z).abs() < 0.001);
            }
        }
    }

    #[test]
    fn indexed_round_stock_heights_follow_the_rotated_cylinder() {
        use cam_model::{HeightPlaneKind as P, HeightReference as R, RotaryAxis};
        let (mut app, id) = operation_editor();
        app.controller.edit_stock(|stock| {
            *stock = cam_model::Stock::FixedRound {
                diameter: 20.,
                inner_diameter: 0.,
                length: 50.,
                position: 0.5,
                tube: false,
                axis: Some(cam_model::StockAxis::Origin {
                    origin_axis: cam_model::StockOriginAxis::X,
                }),
            }
        });
        app.rotary_action(rotary_view::Action::Indexed(true));
        *app.height_draft.as_mut().unwrap().definition_mut(P::Top) =
            cam_model::HeightDefinition::new(R::StockTop, 0.);
        *app.height_draft.as_mut().unwrap().definition_mut(P::Bottom) =
            cam_model::HeightDefinition::new(R::StockBottom, 0.);
        app.height_offsets.insert(P::Top, "0".into());
        app.height_offsets.insert(P::Bottom, "0".into());
        let setup = app.controller.inspection_setup();
        let (frame, min, max) = app
            .controller
            .document()
            .round_stock_geometry(setup)
            .unwrap()
            .unwrap();
        let setup_frame = setup.frame.clone();
        for (axis, angle) in [(RotaryAxis::A, 47.), (RotaryAxis::B, 90.)] {
            app.rotary_action(rotary_view::Action::IndexAxis(axis));
            app.rotary_action(rotary_view::Action::IndexValue(0, angle.to_string()));
            app.rotary_action(rotary_view::Action::IndexValue(2, "3".into()));
            let center = cam_cldata::Point3::new(0., 3., 0.);
            let mut lower = f64::INFINITY;
            let mut upper = f64::NEG_INFINITY;
            for axial in [min[2], max[2]] {
                for i in 0..1440 {
                    let angle_sample = std::f64::consts::TAU * i as f64 / 1440.;
                    let p = [
                        (min[0] + max[0]) * 0.5 + (max[0] - min[0]) * 0.5 * angle_sample.cos(),
                        (min[1] + max[1]) * 0.5 + (max[0] - min[0]) * 0.5 * angle_sample.sin(),
                        axial,
                    ];
                    let p = setup_frame.to_setup_vector(frame.to_world_vector(p));
                    let p = axis.material_point(
                        cam_cldata::Point3::new(p[0], p[1], p[2]),
                        center,
                        -angle,
                    );
                    lower = lower.min(p.z);
                    upper = upper.max(p.z);
                }
            }
            let heights = app.resolve_height_editor(id).unwrap();
            assert!(
                (heights.top - upper).abs() < 0.001,
                "{axis:?}: {} vs {upper}",
                heights.top
            );
            assert!(
                (heights.bottom - lower).abs() < 0.001,
                "{axis:?}: {} vs {lower}",
                heights.bottom
            );
        }
    }
    #[test]
    fn cycles_invalid_offsets_and_missing_selections_block_apply_and_defaults_recover() {
        use cam_model::{HeightPlaneKind as P, HeightReference as R};
        let (mut app, id) = operation_editor();
        let before = app.controller.document().clone();
        let _ = app.update(Message::HeightReferenceChanged(
            P::Retract,
            R::Plane(P::Clearance),
        ));
        assert!(app
            .resolve_height_editor(id)
            .unwrap_err()
            .contains("Circular"));
        let _ = app.update(Message::Apply);
        assert_eq!(app.controller.document(), &before);
        let _ = app.update(Message::HeightDefaults);
        let _ = app.update(Message::HeightOffsetChanged(P::Top, "NaN".into()));
        assert!(app.any_field_invalid());
        let _ = app.update(Message::HeightDefaults);
        let _ = app.update(Message::HeightReferenceChanged(P::Top, R::Selection));
        assert!(app.any_field_invalid());
        let _ = app.update(Message::HeightDefaults);
        let _ = app.update(Message::HeightReferenceChanged(P::Feed, R::Disabled));
        assert!(app.resolve_height_editor(id).unwrap().feed.is_none());
        let _ = app.update(Message::Apply);
        assert_eq!(
            app.controller.inspection_setup().height_settings[&id]
                .definition(P::Feed)
                .reference,
            R::Disabled
        );
    }
    #[test]
    fn selected_drawing_reference_is_persistent_and_picking_preserves_the_operation_editor() {
        use cam_model::{HeightPlaneKind as P, HeightReference as R};
        let (mut app, id) = operation_editor();
        let _ = app.update(Message::HeightReferenceChanged(P::Top, R::Selection));
        let _ = app.update(Message::HeightPickToggle(P::Top));
        let _ = app.update(Message::PickWorld([10., 10.], 1.));
        assert_eq!(app.controller.selection(), Selection::Operation(id));
        assert!(!app.cad_inspecting);
        assert_eq!(
            app.height_draft
                .as_ref()
                .unwrap()
                .definition(P::Top)
                .drawings
                .len(),
            1
        );
        assert_eq!(app.resolve_height_editor(id).unwrap().top, 0.);
        let _ = app.update(Message::HeightPickToggle(P::Top));
        let _ = app.update(Message::Apply);
        assert_eq!(
            app.controller.inspection_setup().height_settings[&id]
                .definition(P::Top)
                .drawings
                .len(),
            1
        );
    }
    #[test]
    fn every_operation_gets_valid_reference_defaults_including_natural_chamfer_depth() {
        use cam_model::{HeightPlaneKind as P, HeightReference as R};
        let (mut app, _) = operation_editor();
        for kind in [
            OpKind::Profile,
            OpKind::Pocket,
            OpKind::Face,
            OpKind::Drill,
            OpKind::Thread,
            OpKind::Chamfer,
            OpKind::Engrave,
            OpKind::Carve,
        ] {
            app.controller.new_operation(kind);
            let id = app
                .controller
                .inspection_setup()
                .operations
                .last()
                .unwrap()
                .id();
            app.begin_operation_draft(id);
            let heights = app
                .resolve_height_editor(id)
                .unwrap_or_else(|e| panic!("{kind:?}: {e}"));
            assert_eq!(
                app.height_draft
                    .as_ref()
                    .unwrap()
                    .definition(P::Clearance)
                    .reference,
                R::Plane(P::Retract)
            );
            assert_eq!(
                app.height_draft
                    .as_ref()
                    .unwrap()
                    .definition(P::Clearance)
                    .offset,
                10.
            );
            assert!(heights.top > heights.bottom);
            if kind == OpKind::Chamfer {
                assert!((heights.bottom + 1.).abs() < 1e-9);
                let _ = app.update(Message::HeightOffsetChanged(P::Bottom, "0.5".into()));
                let error = app.resolve_height_editor(id).unwrap_err();
                assert!(
                    error.contains("selected chamfer and cutter geometry"),
                    "{error}"
                );
                let _ = app.update(Message::CancelOperationDraft);
            }
        }
    }
    #[test]
    fn height_labels_use_the_operation_datum_while_planes_stay_in_model_space() {
        use cam_model::HeightPlaneKind as P;
        let (mut app, id) = operation_editor();
        app.controller.edit_setup(|setup| {
            setup.origin = [-476.8, 367., 54.];
            setup.extra_origins.push(cam_model::Origin {
                index: 2,
                position: [-476.8, 367., 50.],
            });
            setup.frame.world_from_setup.rotation = [[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]];
        });
        // The editor may be editing a different datum than the tree selection.
        app.operation_draft.as_mut().unwrap().set_work_offset(2);
        assert_eq!(app.height_display_value(id, P::Top, 74.), "24");
        assert_eq!(app.height_display_value(id, P::Bottom, 55.), "5");
        let camera = OrbitCamera::framed([0., 0., 0.], [100., 100., 100.]);
        let preview = app.height_preview(camera, 0.).unwrap();
        for plane in preview.planes {
            assert!(plane.corners.iter().all(|p| p[1] == plane.z as f32));
            assert_eq!(plane.value_label.parse::<f64>().unwrap(), plane.z - 50.);
        }
        app.operation_draft.as_mut().unwrap().set_work_offset(1);
        assert_eq!(app.height_display_value(id, P::Top, 74.), "20");
        assert_eq!(app.height_display_value(id, P::Bottom, 55.), "1");
    }

    #[test]
    fn height_labels_keep_entered_precision_without_float_tails() {
        use cam_model::{HeightPlaneKind as P, HeightReference as R};
        let (mut app, id) = operation_editor();
        let _ = app.update(Message::HeightReferenceChanged(P::Retract, R::Origin));
        let _ = app.update(Message::HeightOffsetChanged(P::Retract, "0.1".into()));
        let _ = app.update(Message::HeightOffsetChanged(P::Clearance, "0.20".into()));
        let _ = app.update(Message::HeightReferenceChanged(P::Feed, R::Disabled));
        let z = app.resolve_height_editor(id).unwrap().clearance;
        assert_eq!(app.height_display_value(id, P::Clearance, z), "0.30");
        assert_eq!(app.height_offsets[&P::Clearance], "0.20");
        assert_eq!(app.height_display_value(id, P::Bottom, -0.00000000001), "0");
        let _ = app.update(Message::HeightOffsetChanged(
            P::Clearance,
            "0.12345678".into(),
        ));
        assert_eq!(
            app.height_display_value(id, P::Clearance, 0.22345678),
            "0.22345678"
        );
        let _ = app.update(Message::HeightOffsetChanged(P::Clearance, "2e-4".into()));
        assert_eq!(app.height_display_value(id, P::Clearance, 0.1002), "0.1002");
    }
    #[test]
    fn wrapped_planes_are_cylinders_in_both_rotary_orientations() {
        for axis in [cam_model::RotaryAxis::A, cam_model::RotaryAxis::B] {
            let wrap = cam_model::RotaryWrap {
                axis,
                radius: 100.,
                offset: 1.,
                center: [10., 20., 30.],
                ..Default::default()
            };
            let frame = cam_model::SetupFrame::default();
            let paths = wrapped_plane_outlines(&wrap, 15., [-40.; 3], [40.; 3], &frame);
            assert_eq!(paths.len(), 6);
            assert_eq!(paths[0].first(), paths[0].last());
            for point in paths.iter().take(2).flatten() {
                let radius = match axis {
                    cam_model::RotaryAxis::A => {
                        (point[1] as f64 - 20.).hypot(point[2] as f64 - 30.)
                    }
                    cam_model::RotaryAxis::B => {
                        (point[0] as f64 - 10.).hypot(point[2] as f64 - 30.)
                    }
                };
                assert!((radius - 116.).abs() < 0.0001, "{radius}");
            }
        }
    }
}
