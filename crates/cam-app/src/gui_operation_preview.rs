//! Draft geometry and holding-tab solids, without a generated cutting program.
use super::*;

fn pattern_preview_world_point(
    frame: &cam_model::SetupFrame,
    world: [f64; 3],
    center: [f64; 2],
    translation: [f64; 2],
    degrees: f64,
) -> [f64; 3] {
    let local = frame.to_setup_vector(world);
    let (sin, cos) = degrees.to_radians().sin_cos();
    let x = local[0] - center[0];
    let y = local[1] - center[1];
    frame.to_world_vector([
        center[0] + x * cos - y * sin + translation[0],
        center[1] + x * sin + y * cos + translation[1],
        local[2],
    ])
}

impl App {
    /// Reposition cached seed paths on every valid draft edit. Source calculation
    /// runs separately, so changing placement never blocks the event loop.
    pub(super) fn pattern_placement_preview(&self) -> Scene {
        let mut scene = Scene::new();
        let Some(draft) = &self.pattern_draft else {
            return scene;
        };
        let Ok((kind, options)) = super::pattern_editor_view::configuration_from_draft(draft)
        else {
            return scene;
        };
        let Some(setup) = self
            .controller
            .setups()
            .iter()
            .find(|setup| setup.id == draft.setup_id)
        else {
            return scene;
        };
        let frame = &setup.frame;
        let center = match kind {
            cam_model::PatternKind::Circular { center_xy, .. } => center_xy,
            _ => [0.0, 0.0],
        };
        let existing = setup
            .patterns
            .iter()
            .find(|pattern| Some(pattern.id) == draft.pattern_id);
        let source_indices = existing.map(|pattern| pattern.options.instance_indices(pattern.kind));
        let preview = self.operation_preview_outcome();
        let mut sources = [Scene::new(), Scene::new()];
        if let Some(outcome) = preview.or_else(|| self.controller.outcome()) {
            for (variant, sources) in sources.iter_mut().enumerate() {
                let source_instance = source_indices
                    .as_ref()
                    .and_then(|indices| indices.get(variant).or_else(|| indices.first()).copied());
                for strip in outcome.scene.strips.iter().filter(|strip| {
                    strip.op.is_some_and(|op| draft.members.contains(&op))
                        && (strip.pattern_id.is_none()
                            || if preview.is_some() {
                                strip.pattern_id
                                    == strip
                                        .op
                                        .map(backplot_preview_view::pattern_source_preview_id)
                                    && strip.instance_index == Some(variant as u32)
                            } else {
                                strip.pattern_id == draft.pattern_id
                                    && strip.instance_index == source_instance.map(|i| i as u32)
                            })
                }) {
                    let mut strip = strip.clone();
                    // Recover both entry variants in the source coordinate frame.
                    let placement = if strip.pattern_id.is_none() {
                        None
                    } else if preview.is_some() {
                        Some(([0.0; 2], [variant as f64, 0.0], 0.0))
                    } else {
                        existing.and_then(|pattern| {
                            let old_center = match pattern.kind {
                                cam_model::PatternKind::Circular { center_xy, .. } => center_xy,
                                _ => [0.0; 2],
                            };
                            source_instance
                                .and_then(|i| pattern.options.transform_xy(pattern.kind, i))
                                .map(|(offset, angle)| (old_center, offset, angle))
                        })
                    };
                    if let Some((old_center, offset, angle)) = placement {
                        let (sin, cos) = (-angle).to_radians().sin_cos();
                        let inverse = [
                            -offset[0] * cos + offset[1] * sin,
                            -offset[0] * sin - offset[1] * cos,
                        ];
                        for point in &mut strip.points {
                            *point = pattern_preview_world_point(
                                frame,
                                point.map(f64::from),
                                old_center,
                                inverse,
                                -angle,
                            )
                            .map(|v| v as f32);
                        }
                    }
                    sources.strips.push(strip);
                }
            }
        }
        let anchor = sources[0]
            .bounds()
            .map(|(min, max)| std::array::from_fn(|i| (min[i] as f64 + max[i] as f64) / 2.0))
            .unwrap_or_else(|| {
                frame.to_world_vector(setup.origin_position(self.controller.active_origin()))
            });
        for (ordinal, index) in options
            .instance_indices(kind)
            .into_iter()
            .take(1000)
            .enumerate()
        {
            let sources = &sources[usize::from(ordinal > 0)];
            let Some((translation, degrees)) = options.transform_xy(kind, index) else {
                break;
            };
            if sources.strips.is_empty() {
                let at = pattern_preview_world_point(frame, anchor, center, translation, degrees);
                let local = frame.to_setup_vector(at);
                for axis in 0..2 {
                    let mut a = local;
                    let mut b = local;
                    a[axis] -= 1.5;
                    b[axis] += 1.5;
                    scene.add_highlight(
                        vec![
                            frame.to_world_vector(a).map(|v| v as f32),
                            frame.to_world_vector(b).map(|v| v as f32),
                        ],
                        PICK_BOUNDARY,
                    );
                }
            }
            for strip in &sources.strips {
                let points = strip
                    .points
                    .iter()
                    .map(|point| {
                        pattern_preview_world_point(
                            frame,
                            point.map(f64::from),
                            center,
                            translation,
                            degrees,
                        )
                        .map(|v| v as f32)
                    })
                    .collect();
                let color = if index == 0 {
                    strip.color
                } else {
                    [0.95, 0.62, 0.18, 0.9]
                };
                scene.add_strip_with_owner(
                    points,
                    color,
                    strip.op,
                    strip.dashed,
                    draft.pattern_id,
                    Some(index as u32),
                );
                if let Some(placed) = scene.strips.last_mut() {
                    placed.width = strip.width;
                }
            }
        }
        scene
    }

    pub(super) fn operation_preview_active(&self) -> bool {
        self.workspace == Workspace::Machining
            && self.inspector_enabled
            && self.inspector_active
            && (self.creating_operation()
                || self.operation_draft.as_ref().is_some_and(|draft| {
                    self.controller.selection() == Selection::Operation(draft.id())
                }))
    }

    pub(super) fn editor_selected_geometry(&self) -> Vec<EntityRef> {
        if let Some(wrap) = self
            .operation_draft
            .as_ref()
            .and_then(Operation::rotary_wrap)
        {
            if self.wrapped_pocket_geometry_editor() {
                return self.controller.cad_selection().to_vec();
            }
            return wrap
                .geometry
                .iter()
                .chain(wrap.cylinder.iter())
                .cloned()
                .collect();
        }
        if let Some(draft) = &self.pattern_draft {
            return draft
                .highlighted
                .and_then(|axis| {
                    if axis < 2 {
                        draft.options.directions[axis].reference.clone()
                    } else {
                        draft.options.axis.clone()
                    }
                })
                .into_iter()
                .collect();
        }
        if self.profile_entry_editor.is_some()
            || matches!(&self.operation_draft,Some(Operation::Profile(p)) if p.selections.is_some())
        {
            return self
                .preview_profile_entries()
                .into_iter()
                .flat_map(|e| match e.source {
                    cam_model::ProfileGeometrySource::Cad(refs) => refs,
                    _ => Vec::new(),
                })
                .collect();
        }
        if self.cad_pending_kind.is_some() || self.cad_editing_op.is_some() {
            return self.controller.cad_selection().to_vec();
        }
        let Selection::Operation(id) = self.controller.selection() else {
            return Vec::new();
        };
        self.operation_geometry
            .clone()
            .or_else(|| {
                self.operation_binding_preview
                    .as_ref()
                    .or_else(|| self.controller.geometry_state().bindings.get(&id))
                    .map(|binding| binding.selected.clone())
            })
            .unwrap_or_default()
    }

    pub(super) fn operation_geometry_preview(&self) -> Scene {
        let mut scene = Scene::new();
        if self.wrapped_pocket_geometry_editor() {
            return scene;
        }
        let (operation, binding) = if self.creating_operation() {
            match self.creation_operation() {
                Ok((op, binding)) => (Some(op), binding),
                Err(_) => (None, None),
            }
        } else {
            let operation = self.operation_draft.clone();
            let binding = operation
                .as_ref()
                .filter(|op| op.rotary_wrap().is_none())
                .and_then(|op| {
                    self.operation_binding_preview
                        .as_ref()
                        .or_else(|| self.controller.geometry_state().bindings.get(&op.id()))
                })
                .cloned();
            (operation, binding)
        };
        let Some(mut op) = operation else {
            return scene;
        };
        if op.rotary_wrap().is_some() {
            if let Ok(geometry) =
                crate::cad_operations::geometry_for_setup(self.controller.document())
            {
                if crate::cad_operations::resolve_wrapped_operation(&geometry, &mut op).is_err() {
                    return scene;
                }
            }
        }
        // The saved binding and cached operation contours still describe the old
        // selection while the geometry picker is open.
        if self.cad_editing_op.is_some() || self.drawing_editing_op.is_some() {
            return scene;
        }
        let frame = &self.controller.inspection_setup().frame;
        let mut outlines = Vec::new();
        let mut centers = Vec::new();
        let mut z = 0.0;
        if matches!(op, Operation::Profile(_)) {
            if let Some((profile, source_z)) = self.profile_tab_geometry() {
                z = source_z;
                outlines.push((profile.chain.points().to_vec(), profile.closed));
                outlines.extend(
                    profile
                        .additional_chains
                        .iter()
                        .map(|p| (p.chain.points().to_vec(), p.closed)),
                );
            }
        } else if let Some(binding) = binding.as_ref() {
            if let Ok(geometry) =
                crate::cad_operations::geometry_for_setup(self.controller.document())
            {
                if let Ok(resolved) = cam_geometry3d::resolve_binding_advisory(&geometry, binding) {
                    z = if binding
                        .selected
                        .iter()
                        .any(|r| r.kind == EntityKind::Silhouette)
                    {
                        0.0
                    } else if binding.recipe == cam_model::ExtractionRecipe::PocketBoundary
                        || matches!(
                            op,
                            Operation::Drill(_)
                                | Operation::Bore(_)
                                | Operation::Thread(_)
                                | Operation::Chamfer(_)
                                | Operation::Engrave(_)
                                | Operation::Carve(_)
                        )
                    {
                        resolved.heights.top
                    } else {
                        resolved.heights.bottom
                    };
                    add_chamfer_faces(&mut scene, &resolved.chamfers, frame);
                    if matches!(op, Operation::Chamfer(_) | Operation::Engrave(_)) {
                        outlines.extend(
                            resolved
                                .profiles
                                .iter()
                                .map(|p| (p.chain.points().to_vec(), p.closed)),
                        );
                    }
                    for region in resolved
                        .regions
                        .into_iter()
                        .filter(|_| !matches!(op, Operation::Chamfer(_) | Operation::Engrave(_)))
                    {
                        outlines.push((region.outer().points().to_vec(), true));
                        outlines.extend(
                            region
                                .holes()
                                .iter()
                                .map(|hole| (hole.points().to_vec(), true)),
                        );
                    }
                    centers.extend(
                        resolved
                            .holes
                            .into_iter()
                            .map(|hole| [hole.center.x, hole.center.y]),
                    );
                }
            }
        } else {
            match &op {
                Operation::Pocket(p) => {
                    outlines.push((p.boundary.points().to_vec(), true));
                    outlines.extend(p.islands.iter().map(|c| (c.points().to_vec(), true)));
                    for region in &p.additional_regions {
                        outlines.push((region.outer().points().to_vec(), true));
                        outlines.extend(region.holes().iter().map(|c| (c.points().to_vec(), true)));
                    }
                }
                Operation::Carve(p) => {
                    outlines.push((p.boundary.points().to_vec(), true));
                    outlines.extend(p.islands.iter().map(|c| (c.points().to_vec(), true)));
                }
                Operation::Face(p) => outlines.push((p.boundary.points().to_vec(), true)),
                Operation::Chamfer(p) if !p.faces.is_empty() => {
                    add_chamfer_faces(&mut scene, &p.faces, frame)
                }
                Operation::Chamfer(p) => outlines.push((p.chain.points().to_vec(), p.closed)),
                Operation::Engrave(p) => outlines.push((p.chain.points().to_vec(), p.closed)),
                Operation::Drill(p) => centers.extend(p.points.iter().copied()),
                Operation::Bore(p) => centers.extend(p.points.iter().copied()),
                Operation::Thread(p) => centers.extend(p.points.iter().copied()),
                Operation::Profile(_)
                | Operation::Adaptive(_)
                | Operation::Parallel(_)
                | Operation::Spiral(_) => {}
            }
        }
        if let Some(wrap) = op.rotary_wrap() {
            let mut program = cam_cldata::Program::new();
            let tag = cam_cldata::Tag::new(op.id(), cam_cldata::MoveKind::Cutting);
            for (mut points, closed) in outlines {
                if closed {
                    if let Some(first) = points.first().copied() {
                        points.push(first);
                    }
                }
                for (i, point) in points.iter().enumerate() {
                    let to = cam_cldata::Point3::new(point.x, point.y, wrap.surface_z);
                    program.push(if i == 0 {
                        cam_cldata::Step::Rapid { to, tag }
                    } else {
                        cam_cldata::Step::Linear { to, feed: 1., tag }
                    });
                }
            }
            if let Ok(wrapped) =
                cam_toolpath::wrap_program(&program, wrap, &cam_toolpath::CancelToken::new())
            {
                scene = Scene::from_program(&wrapped);
                for strip in &mut scene.strips {
                    strip.color = PICK_BOUNDARY;
                    strip.width = 3.;
                    for point in &mut strip.points {
                        *point = frame
                            .to_world_vector(point.map(f64::from))
                            .map(|n| n as f32);
                    }
                }
            }
            return scene;
        }
        for (points, closed) in outlines {
            let mut world: Vec<_> = points
                .iter()
                .map(|p| frame.to_world_vector([p.x, p.y, z]).map(|v| v as f32))
                .collect();
            if closed {
                if let Some(first) = world.first().copied() {
                    world.push(first);
                }
            }
            scene.add_highlight(world, PICK_BOUNDARY);
        }
        for point in centers {
            for axis in 0..2 {
                let mut a = [point[0], point[1], z];
                let mut b = a;
                a[axis] -= 1.;
                b[axis] += 1.;
                scene.add_highlight(
                    vec![
                        frame.to_world_vector(a).map(|v| v as f32),
                        frame.to_world_vector(b).map(|v| v as f32),
                    ],
                    PICK_BOUNDARY,
                );
            }
        }
        let position_tab = if matches!(op, Operation::Spiral(_)) {
            OperationTab::Geometry
        } else {
            OperationTab::Linking
        };
        if self.operation_tab == position_tab {
            use operation_positions::PositionKind::{Entry, Exit};
            let z = binding.as_ref().map_or(
                self.controller.inspection_setup().heights.top_of_stock,
                |binding| binding.heights.top,
            );
            for (kind, point) in [
                (Entry, operation_tabs::operation_start(&op)),
                (Exit, operation_positions::operation_end(&op)),
            ] {
                if let Some(point) = point {
                    for axis in 0..2 {
                        let mut a = self.operation_position_marker(kind, point, z);
                        let mut b = a;
                        a[axis] -= 1.0;
                        b[axis] += 1.0;
                        scene.add_highlight(
                            vec![
                                frame.to_world_vector(a).map(|v| v as f32),
                                frame.to_world_vector(b).map(|v| v as f32),
                            ],
                            kind.color(),
                        );
                    }
                }
            }
        }
        let index = if self.operation_draft.is_some() {
            self.rotary_index_draft.as_ref()
        } else {
            self.controller.document().rotary_indexes.get(&op.id())
        };
        if let Some(index) = index {
            let center = cam_cldata::Point3::new(index.center[0], index.center[1], index.center[2]);
            for strip in &mut scene.strips {
                for point in &mut strip.points {
                    let p = frame.to_setup_vector(point.map(f64::from));
                    let p = index.axis.material_point(
                        cam_cldata::Point3::new(p[0], p[1], p[2]),
                        center,
                        index.angle,
                    );
                    *point = frame.to_world_vector([p.x, p.y, p.z]).map(|v| v as f32);
                }
            }
        }
        scene
    }

    pub(super) fn profile_tab_diameter(&self, op: &cam_model::ProfileOp) -> Option<f64> {
        self.operation_tools
            .iter()
            .chain(self.controller.inspection_setup().tools.iter())
            .find(|tool| tool.number == op.tool)
            .map(|tool| tool.diameter)
            .or_else(|| self.wizard_tool().map(|tool| tool.diameter))
    }

    pub(super) fn profile_tab_locations(&self) -> Vec<[f64; 2]> {
        if !matches!(self.operation_draft.as_ref(), Some(Operation::Profile(op)) if op.tabs.enabled)
        {
            return Vec::new();
        }
        let Some(op) = self.profile_tab_operation() else {
            return Vec::new();
        };
        if !op.tabs.enabled {
            return Vec::new();
        }
        self.profile_tab_diameter(&op)
            .map(|diameter| {
                cam_toolpath::profile_tab_preview(&op, diameter, cam_cldata::SpindleDir::Cw)
                    .into_iter()
                    .map(|tab| tab.position)
                    .collect()
            })
            .unwrap_or_else(|| {
                if op.tabs.manual {
                    op.tabs.positions.clone()
                } else {
                    Vec::new()
                }
            })
    }

    pub(super) fn profile_tab_mesh(&self) -> (Vec<MeshVertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        if !matches!(self.operation_draft.as_ref(), Some(Operation::Profile(op)) if op.tabs.enabled)
        {
            return (vertices, indices);
        }
        let Some(op) = self.profile_tab_operation() else {
            return (vertices, indices);
        };
        if !op.tabs.enabled
            || !op.tabs.width.is_finite()
            || op.tabs.width <= 0.
            || !op.tabs.height.is_finite()
            || op.tabs.height <= 0.
        {
            return (vertices, indices);
        }
        let Some(diameter) = self.profile_tab_diameter(&op) else {
            return (vertices, indices);
        };
        let bottom = self
            .resolve_height_editor(if self.creating_operation() { 0 } else { op.id })
            .map(|h| h.bottom)
            .unwrap_or(-op.depth);
        if !bottom.is_finite() {
            return (vertices, indices);
        }
        let frame = &self.controller.inspection_setup().frame;
        for (index, tab) in
            cam_toolpath::profile_tab_preview(&op, diameter, cam_cldata::SpindleDir::Cw)
                .into_iter()
                .enumerate()
        {
            let u = [tab.tangent[0], tab.tangent[1], 0.];
            let v = [-tab.tangent[1], tab.tangent[0], 0.];
            let color = if self.profile_tab_selection == Some(index) {
                [1., 0.60, 0.08, 1.]
            } else {
                [0.86, 0.86, 0.65, 1.]
            };
            let corners: [[f64; 3]; 8] = std::array::from_fn(|corner| {
                let along = if op.tabs.shape == cam_model::TabShape::Triangular && corner & 4 != 0 {
                    0.
                } else if corner & 1 == 0 {
                    -op.tabs.width * 0.5
                } else {
                    op.tabs.width * 0.5
                };
                let across = if corner & 2 == 0 {
                    -diameter * 0.5
                } else {
                    diameter * 0.5
                };
                [
                    tab.center[0] + u[0] * along + v[0] * across,
                    tab.center[1] + u[1] * along + v[1] * across,
                    bottom + if corner & 4 == 0 { 0. } else { op.tabs.height },
                ]
            });
            for (quad, normal) in [
                ([0, 4, 6, 2], u.map(|x| -x)),
                ([1, 3, 7, 5], u),
                ([0, 1, 5, 4], v.map(|x| -x)),
                ([2, 6, 7, 3], v),
                ([0, 2, 3, 1], [0., 0., -1.]),
                ([4, 5, 7, 6], [0., 0., 1.]),
            ] {
                let normal = if op.tabs.shape == cam_model::TabShape::Triangular {
                    let a = corners[quad[0]];
                    let b = corners[quad[1]];
                    let c = corners[quad[2]];
                    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                    let cross = [
                        ab[1] * ac[2] - ab[2] * ac[1],
                        ab[2] * ac[0] - ab[0] * ac[2],
                        ab[0] * ac[1] - ab[1] * ac[0],
                    ];
                    let length = cross.iter().map(|v| v * v).sum::<f64>().sqrt();
                    if length > 1e-12 {
                        cross.map(|v| v / length)
                    } else {
                        normal
                    }
                } else {
                    normal
                };
                let base = vertices.len() as u32;
                let normal = if let Some(index) = &self.rotary_index_draft {
                    let n = index.axis.material_point(
                        cam_cldata::Point3::new(normal[0], normal[1], normal[2]),
                        cam_cldata::Point3::new(0., 0., 0.),
                        index.angle,
                    );
                    [n.x, n.y, n.z]
                } else {
                    normal
                };
                vertices.extend(quad.map(|corner| {
                    MeshVertex {
                        position: rotary_view::index_point_to_world(
                            frame,
                            self.rotary_index_draft.as_ref(),
                            corners[corner],
                        )
                        .map(|v| v as f32),
                        normal: frame.to_world_vector(normal).map(|v| v as f32),
                        color,
                    }
                }));
                indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
            }
        }
        (vertices, indices)
    }
}

fn add_chamfer_faces(
    scene: &mut Scene,
    faces: &[cam_model::ChamferFace],
    frame: &cam_model::SetupFrame,
) {
    for face in faces {
        let upper = face.upper.map(|p| {
            frame
                .to_world_vector([p[0], p[1], face.top])
                .map(|v| v as f32)
        });
        let lower = face.lower.map(|p| {
            frame
                .to_world_vector([p[0], p[1], face.bottom])
                .map(|v| v as f32)
        });
        scene.add_highlight(upper.to_vec(), PICK_BOUNDARY);
        scene.add_highlight(lower.to_vec(), PICK_BOUNDARY);
        for end in 0..2 {
            scene.add_highlight(vec![upper[end], lower[end]], PICK_BOUNDARY);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pattern_preview_unframes_world_paths_before_setup_xy_transform() {
        let frame = cam_model::SetupFrame {
            world_from_setup: cam_model::RigidTransform {
                rotation: [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
                translation: [0.0; 3],
            },
            references: Default::default(),
        };
        let world_source = frame.to_world_vector([10.0, 0.0, 0.0]);
        let placed = pattern_preview_world_point(&frame, world_source, [0.0; 2], [2.0, 0.0], 0.0);
        assert!((placed[0] - 0.0).abs() < 1e-9);
        assert!((placed[1] - 12.0).abs() < 1e-9);
    }

    fn profile_editor() -> (App, u32) {
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
        app.show_stock = false;
        let _ = app.update(Message::ProfileTabsEnabled(true));
        let _ = app.update(Message::ProfileTabPositioning(
            profile_tabs::Positioning::Manual,
        ));
        let Operation::Profile(profile) = app.operation_draft.as_mut().unwrap() else {
            unreachable!()
        };
        let pts = profile.chain.points();
        profile.tabs.positions = [0, 2]
            .map(|index| {
                let a = pts[index];
                let b = pts[(index + 1) % pts.len()];
                [(a.x + b.x) * 0.5, (a.y + b.y) * 0.5]
            })
            .to_vec();
        let _ = app.update(Message::FieldChanged(Field::TabWidth, "3".into()));
        let _ = app.update(Message::FieldChanged(Field::TabHeight, "1".into()));
        (app, id)
    }

    #[test]
    fn preview_blocks_follow_draft_dimensions_and_location_selection() {
        let (mut app, id) = profile_editor();
        let (initial, indices) = app.profile_tab_mesh();
        assert_eq!((initial.len(), indices.len()), (48, 72));
        let extent = |vertices: &[MeshVertex], axis: usize| {
            let min = vertices
                .iter()
                .map(|v| v.position[axis])
                .fold(f32::INFINITY, f32::min);
            let max = vertices
                .iter()
                .map(|v| v.position[axis])
                .fold(f32::NEG_INFINITY, f32::max);
            max - min
        };
        assert!((extent(&initial[..24], 2) - 1.).abs() < 1e-5);
        let bottom = app.resolve_height_editor(id).unwrap().bottom as f32;
        assert!(
            (initial
                .iter()
                .map(|v| v.position[2])
                .fold(f32::INFINITY, f32::min)
                - bottom)
                .abs()
                < 1e-5
        );
        let _ = app.update(Message::ProfileTabSelected(1));
        let (highlighted, _) = app.profile_tab_mesh();
        assert_eq!(highlighted[0].color, initial[0].color);
        assert_ne!(highlighted[24].color, initial[24].color);
        assert_eq!(app.profile_tab_selection, Some(1));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Passes));
        assert!(app
            .profile_tab_overlay(OrbitCamera::framed([-50.; 3], [100.; 3]), 0.)
            .is_some());
        let _ = app.update(Message::FieldChanged(Field::TabWidth, "9".into()));
        let _ = app.update(Message::FieldChanged(Field::TabHeight, "2".into()));
        let (resized, _) = app.profile_tab_mesh();
        assert!((extent(&resized[..24], 2) - 2.).abs() < 1e-5);
        assert!(
            extent(&resized[..24], 0) > extent(&initial[..24], 0) + 5.
                || extent(&resized[..24], 1) > extent(&initial[..24], 1) + 5.
        );
        let _ = app.update(Message::ProfileTabRemove(0));
        assert_eq!(app.profile_tab_selection, Some(0));
        assert_eq!(app.profile_tab_mesh().0.len(), 24);
        let _ = app.update(Message::ProfileTabRemove(0));
        assert_eq!(app.profile_tab_selection, None);
        assert!(app.profile_tab_mesh().0.is_empty());
        let _ = app.update(Message::ProfileTabsEnabled(false));
        assert!(app
            .operation_geometry_preview()
            .strips
            .iter()
            .any(|s| s.color == PICK_BOUNDARY));
    }

    #[test]
    fn automatic_tab_locations_and_blocks_update_with_spacing() {
        let (mut app, _) = profile_editor();
        let _ = app.update(Message::ProfileTabPositioning(
            profile_tabs::Positioning::ByDistance,
        ));
        let _ = app.update(Message::FieldChanged(Field::TabSpacing, "30".into()));
        let locations = app.profile_tab_locations();
        assert!(locations.len() > 1);
        assert_eq!(app.profile_tab_mesh().0.len(), locations.len() * 24);
        let _ = app.update(Message::ProfileTabSelected(locations.len() - 1));
        assert_eq!(app.profile_tab_selection, Some(locations.len() - 1));
        let _ = app.update(Message::FieldChanged(Field::TabSpacing, "10000".into()));
        assert_eq!(app.profile_tab_locations().len(), 1);
        assert_eq!(app.profile_tab_mesh().0.len(), 24);
        assert_eq!(app.profile_tab_selection, None);
    }

    #[test]
    fn removing_an_automatic_tab_keeps_the_remaining_locations() {
        let (mut app, id) = profile_editor();
        let committed = app.controller.operation(id).unwrap().clone();
        let _ = app.update(Message::ProfileTabPositioning(
            profile_tabs::Positioning::ByDistance,
        ));
        let _ = app.update(Message::FieldChanged(Field::TabSpacing, "30".into()));
        let mut locations = app.profile_tab_locations();
        let op = app.profile_tab_operation().unwrap();
        let diameter = app.profile_tab_diameter(&op).unwrap();
        let mut footprints =
            cam_toolpath::profile_tab_preview(&op, diameter, cam_cldata::SpindleDir::Cw);
        assert!(locations.len() > 1);
        let _ = app.update(Message::ProfileTabSelected(1));
        let _ = app.update(Message::ProfileTabRemove(0));
        locations.remove(0);
        footprints.remove(0);
        let op = app.profile_tab_operation().unwrap();
        assert!(op.tabs.manual);
        let remaining =
            cam_toolpath::profile_tab_preview(&op, diameter, cam_cldata::SpindleDir::Cw);
        assert_eq!(remaining.len(), footprints.len());
        for (actual, expected) in remaining.iter().zip(&footprints) {
            assert!(
                (actual.center[0] - expected.center[0])
                    .hypot(actual.center[1] - expected.center[1])
                    < 1e-6
            );
        }
        assert_eq!(app.profile_tab_locations().len(), locations.len());
        assert_eq!(app.profile_tab_mesh().0.len(), locations.len() * 24);
        assert_eq!(app.profile_tab_selection, Some(0));
        assert_eq!(app.controller.operation(id), Some(&committed));
        let _ = app.update(Message::CancelOperationDraft);
        assert_eq!(app.controller.operation(id), Some(&committed));
    }

    #[test]
    fn editing_replaces_generated_paths_with_geometry_until_the_editor_closes() {
        let (mut app, id) = profile_editor();
        // Generate the committed operation; draft tabs stay uncommitted.
        app.controller.run(&cam_toolpath::CancelToken::new());
        let generated = app.controller.outcome().unwrap().program.clone();
        app.focus_ops.insert(id);
        let viewport = app.machining_viewport();
        assert!(!viewport
            .vertices
            .iter()
            .any(|v| v.color == cam_render::CUT || v.color == cam_render::RAPID));
        assert!(viewport.vertices.iter().any(|v| v.color == PICK_BOUNDARY));
        assert!(viewport.drill_marks.is_empty());
        assert_eq!(viewport.mesh_indices.len(), 72);
        assert_eq!(app.controller.outcome().unwrap().program, generated);
        app.operation_draft = None;
        app.inspector_active = false;
        assert!(app
            .machining_viewport()
            .vertices
            .iter()
            .any(|v| v.color == cam_render::CUT));
        assert_eq!(app.controller.outcome().unwrap().program, generated);
    }
}
