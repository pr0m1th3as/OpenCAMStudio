//! Main Machining/Tools surfaces. Camera and semantic selection stay in App.
use super::*;
use iced::widget::column;

#[cfg(test)]
fn stock_to_machine_view(
    frame: &cam_model::SetupFrame,
    axis: cam_cldata::RotaryAxis,
    center: cam_cldata::Point3,
    angle: f64,
    point: [f32; 3],
    vector: bool,
) -> [f32; 3] {
    let p = frame.to_setup_vector(point.map(f64::from));
    let origin = if vector {
        cam_cldata::Point3::new(0., 0., 0.)
    } else {
        center
    };
    let p = axis.material_point(cam_cldata::Point3::new(p[0], p[1], p[2]), origin, -angle);
    frame.to_world_vector([p.x, p.y, p.z]).map(|v| v as f32)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) enum Workspace {
    #[default]
    Machining,
    Tools,
}
impl Workspace {
    pub(super) fn label(self) -> &'static str {
        match self {
            Self::Machining => "Machining",
            Self::Tools => "Tools",
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) enum ToolColumnDrag {
    Resize {
        column: usize,
        start_x: f32,
        width: f32,
    },
    Reorder {
        column: usize,
        start: iced::Point,
        target: Option<(usize, bool)>,
    },
}

pub(super) fn reorder_tool_column(
    order: &mut [usize; 8],
    column: usize,
    target: usize,
    after: bool,
) {
    if column == target {
        return;
    }
    let Some(from) = order.iter().position(|&item| item == column) else {
        return;
    };
    let Some(to) = order.iter().position(|&item| item == target) else {
        return;
    };
    let insertion = to + usize::from(after);
    let insertion = if from < insertion {
        insertion - 1
    } else {
        insertion
    };
    if from < insertion {
        order[from..=insertion].rotate_left(1);
    } else if from > insertion {
        order[insertion..=from].rotate_right(1);
    }
}

pub(super) const TOOL_TABLE_LABELS: [&str; 8] = [
    "Tool",
    "Description",
    "Cutter",
    "Manufacturer",
    "Product ID",
    "Ø mm",
    "Length mm",
    "Flute length mm",
];

fn tool_table_matches(
    values: &[String; 8],
    search: &str,
    filters: &[Option<BTreeSet<String>>; 8],
) -> bool {
    let search = search.trim().to_lowercase();
    (search.is_empty()
        || values
            .iter()
            .any(|value| value.to_lowercase().contains(&search)))
        && values.iter().zip(filters).all(|(value, filter)| {
            filter
                .as_ref()
                .is_none_or(|selected| selected.contains(value))
        })
}

pub(super) fn stock_world_bounds(controller: &AppController) -> Option<([f32; 3], [f32; 3])> {
    stock_world_bounds_for_setup(controller, controller.inspection_setup())
}

pub(super) fn stock_world_bounds_for_setup(
    controller: &AppController,
    setup: &cam_model::Setup,
) -> Option<([f32; 3], [f32; 3])> {
    let (min, max) = controller.pending_setup_stock_bounds(setup);
    if min[0] >= max[0] || min[1] >= max[1] {
        return None;
    }
    let frame = &setup.frame;
    let points: Vec<_> = (0..8)
        .map(|index| {
            frame
                .to_world_vector(std::array::from_fn(|axis| {
                    if index & (1 << axis) == 0 {
                        min[axis]
                    } else {
                        max[axis]
                    }
                }))
                .map(|v| v as f32)
        })
        .collect();
    Some((
        std::array::from_fn(|axis| points.iter().map(|p| p[axis]).fold(f32::INFINITY, f32::min)),
        std::array::from_fn(|axis| {
            points
                .iter()
                .map(|p| p[axis])
                .fold(f32::NEG_INFINITY, f32::max)
        }),
    ))
}

/// The configured input blank: a box, or the preceding setup's processed stock.
pub(super) fn uncut_stock_mesh(controller: &AppController) -> (Vec<MeshVertex>, Vec<u32>) {
    uncut_stock_mesh_for_setup(controller, controller.inspection_setup())
}

pub(super) fn uncut_stock_mesh_for_setup(
    controller: &AppController,
    setup: &cam_model::Setup,
) -> (Vec<MeshVertex>, Vec<u32>) {
    if setup.stock == cam_model::Stock::PreviousSetup {
        if controller.inspection_setup().stock != cam_model::Stock::PreviousSetup {
            return (Vec::new(), Vec::new());
        }
        return controller
            .outcome()
            .filter(|o| o.completion == JobCompletion::Complete)
            .and_then(|o| o.initial_stock.clone())
            .unwrap_or_default();
    }
    let (min, max) = controller.pending_setup_stock_bounds(setup);
    if !controller.has_geometry() || (0..3).any(|axis| max[axis] <= min[axis]) {
        return (vec![], vec![]);
    }
    let frame = &setup.frame;
    let stock = setup.stock;
    let shape = crate::stock_shape::oriented_round_mesh(controller.document(), setup)
        .ok()
        .flatten()
        .or_else(|| crate::stock_shape::solid_mesh(controller.document(), setup));
    if let Some(mesh) = shape {
        let vertices = mesh
            .positions
            .iter()
            .zip(&mesh.normals)
            .map(|(p, n)| MeshVertex {
                position: frame.to_world_vector(p.map(f64::from)).map(|v| v as f32),
                normal: frame.to_world_vector(n.map(f64::from)).map(|v| v as f32),
                color: [0.72, 0.66, 0.22, 1.],
            })
            .collect();
        return (vertices, mesh.indices);
    }
    if matches!(
        stock,
        cam_model::Stock::FromSolid { .. }
            | cam_model::Stock::FixedRound { .. }
            | cam_model::Stock::RelativeRound { .. }
    ) {
        return (vec![], vec![]);
    }
    let corners: [[f64; 3]; 8] = std::array::from_fn(|index| {
        std::array::from_fn(|axis| {
            if index & (1 << axis) == 0 {
                min[axis]
            } else {
                max[axis]
            }
        })
    });
    let mut vertices = Vec::with_capacity(24);
    let mut indices = Vec::with_capacity(36);
    for (quad, normal) in [
        ([0, 4, 6, 2], [-1., 0., 0.]),
        ([1, 3, 7, 5], [1., 0., 0.]),
        ([0, 1, 5, 4], [0., -1., 0.]),
        ([2, 6, 7, 3], [0., 1., 0.]),
        ([0, 2, 3, 1], [0., 0., -1.]),
        ([4, 5, 7, 6], [0., 0., 1.]),
    ] {
        let base = vertices.len() as u32;
        vertices.extend(quad.map(|corner| MeshVertex {
            position: frame.to_world_vector(corners[corner]).map(|v| v as f32),
            normal: frame.to_world_vector(normal).map(|v| v as f32),
            color: [0.72, 0.66, 0.22, 1.],
        }));
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    (vertices, indices)
}
/// Only semantic scene inputs belong here. Camera, pointer position and menu
/// state are applied to the shared snapshot without rebuilding its geometry.
#[derive(Clone, PartialEq)]
pub(super) struct ViewportKey {
    revision: u64,
    setup: u64,
    origin: u32,
    selection: Selection,
    pending: Option<crate::PendingOp>,
    outcome: Option<crate::JobStamp>,
    run_preview_revision: Option<usize>,
    simulation: Option<SimulationViewportKey>,
    operation_preview: Option<crate::JobStamp>,
    operation_preview_revision: Option<usize>,
    cad_selection: Vec<EntityRef>,
    cad_hover: Option<EntityRef>,
    cad_hidden: BTreeSet<InstanceId>,
    focus: BTreeSet<u32>,
    hidden: Vec<u32>,
    model_view: bool,
    tree_body: Option<InstanceId>,
    show_stock: bool,
    show_origin: bool,
    envelope: Option<[f32; 3]>,
    snap: Option<(SnapHit, f64)>,
    hover_loop: Option<LoopRef>,
    set_origin: bool,
    origin_mode: Option<SetupOriginMode>,
    origin_first: Option<[f64; 3]>,
    view_prefs: crate::settings::ViewPrefs,
    snap_prefs: crate::settings::SnapPrefs,
    draft: Option<cam_model::Setup>,
    editor: Option<EditorKey>,
    pattern: Option<PatternDraft>,
}

#[derive(Clone)]
struct SimulationViewportKey {
    display_token: Arc<()>,
    options: crate::settings::SimulationPrefs,
    collision: bool,
}

impl PartialEq for SimulationViewportKey {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.display_token, &other.display_token)
            && self.options == other.options
            && self.collision == other.collision
    }
}

#[derive(Clone, PartialEq)]
struct EditorKey {
    creating: bool,
    wrapped_geometry_editor: bool,
    cad_kind: Option<OpKind>,
    cad_editing: Option<u32>,
    drawing_editing: Option<u32>,
    tool: Option<cam_model::Tool>,
    operation: Option<Operation>,
    rotary_index: Option<cam_model::RotaryIndex>,
    binding: Option<cam_model::GeometryBinding>,
    geometry: Vec<EntityRef>,
    fields: BTreeMap<Field, String>,
    edited_fields: BTreeSet<Field>,
    tools: Vec<cam_model::Tool>,
    heights: Option<cam_model::OperationHeightSettings>,
    height_id: Option<u32>,
    offsets: BTreeMap<cam_model::HeightPlaneKind, String>,
    tab_selection: Option<usize>,
    position_markers: [Option<[f64; 3]>; 2],
    tab: OperationTab,
}

#[derive(Clone, PartialEq)]
pub(super) struct CadSceneKey {
    revision: u64,
    hidden: BTreeSet<InstanceId>,
    frame: cam_model::SetupFrame,
}

impl App {
    pub(super) fn project_view_changed(&self) -> bool {
        self.controller
            .project_view()
            .is_some_and(|saved| *saved != self.capture_project_view())
    }

    pub(super) fn capture_project_view(&self) -> crate::project::ProjectView {
        crate::project::ProjectView {
            active_setup: self.controller.active_setup_id(),
            active_origin: self.controller.active_origin(),
            selection: self.controller.selection(),
            cad_selection: self.controller.cad_selection().to_vec(),
            expanded_setups: self.expanded_setups.clone(),
            collapsed_assets: self.collapsed_assets.clone(),
            expanded_operations: self.expanded_operations.clone(),
            expanded_patterns: self.expanded_patterns.clone(),
            tools_expanded: self.tools_expanded,
            hidden_assets: self.hidden_assets.clone(),
            model_view: self.model_view,
            browser_view: self.browser_view,
            models_collapsed: self.models_collapsed,
            setups_collapsed: self.setups_collapsed,
            hidden_setups: self.hidden_setups.clone(),
            hidden_ops: self.hidden_ops.clone(),
            cad_hidden: self.cad_hidden.clone(),
            focus_ops: self.focus_ops.clone(),
            planar_selection: self.planar_selection.clone(),
            show_stock: self.show_stock,
            show_origin: self.show_origin,
            show_project_browser: self.show_project_browser,
        }
    }

    pub(super) fn restore_project_view(&mut self) {
        if let Some(view) = self.controller.project_view().cloned() {
            self.expanded_setups = view.expanded_setups;
            self.collapsed_assets = view.collapsed_assets;
            self.expanded_operations = view.expanded_operations;
            self.expanded_patterns = view.expanded_patterns;
            self.tools_expanded = view.tools_expanded;
            self.hidden_assets = view.hidden_assets;
            self.model_view = view.model_view;
            self.browser_view = view.browser_view;
            self.models_collapsed = view.models_collapsed;
            self.setups_collapsed = view.setups_collapsed;
            self.hidden_setups = view.hidden_setups;
            self.hidden_ops = view.hidden_ops;
            self.cad_hidden = view.cad_hidden;
            self.focus_ops = view.focus_ops;
            self.planar_selection = view.planar_selection;
            self.show_stock = view.show_stock;
            self.show_origin = view.show_origin;
            // Pane visibility is a user preference, including for older projects
            // that still carry a browser-visibility snapshot.
        }
    }

    pub(super) fn reset_model_browser(&mut self) {
        self.cutting_force_inputs.clear();
        self.model_view = Some(TreeNode::Models);
        self.browser_view = TreeNode::Models;
        self.models_collapsed = true;
        self.setups_collapsed = false;
        self.cad_hidden.clear();
        self.hidden_assets.clear();
        self.collapsed_assets.clear();
        self.expanded_setups = self
            .controller
            .setups()
            .first()
            .map(|s| s.id)
            .into_iter()
            .collect();
        self.expanded_patterns.clear();
        self.expanded_operations.clear();
        self.tools_expanded = false;
        self.hidden_setups.clear();
        self.hidden_ops.clear();
    }

    /// One visibility mask for rendering, picking, snapping and camera framing.
    /// Choosing setup members temporarily exposes the full model collection.
    pub(super) fn viewport_hidden_instances(&self) -> BTreeSet<InstanceId> {
        let setup = self
            .setup_draft
            .as_ref()
            .unwrap_or_else(|| self.controller.inspection_setup());
        let mut hidden = self.cad_hidden.clone();
        for instance in &self.controller.geometry_state().instances {
            let in_scope = if self.setup_model_pick || self.controller.setups().is_empty() {
                true
            } else {
                match &self.model_view {
                    Some(TreeNode::Asset(id)) => instance.asset_id == *id,
                    Some(TreeNode::Model(id)) => self
                        .controller
                        .geometry_state()
                        .instance(*id)
                        .is_some_and(|selected| selected.asset_id == instance.asset_id),
                    Some(_) => true,
                    None => {
                        setup.model_instances.contains(&instance.id)
                            || setup.fixture_instances.contains(&instance.id)
                    }
                }
            };
            if !in_scope || self.hidden_assets.contains(&instance.asset_id) {
                hidden.insert(instance.id);
            }
        }
        hidden
    }

    pub(super) fn switch_workspace(&mut self, workspace: Workspace) {
        if self.workspace == workspace {
            return;
        }
        if self.setup_draft_dirty()
            || !self.tool_drafts.is_empty()
            || self.inspector_dirty()
            || self.any_field_invalid()
            || !self.operation_height_overrides_valid()
        {
            self.status = "Apply or Cancel inspector changes before switching workspaces.".into();
            return;
        }
        if self.controller.pending_op().is_some()
            || self.cad_pending_kind.is_some()
            || self.cad_editing_op.is_some()
        {
            self.status =
                "Confirm or Cancel operation selection before switching workspaces.".into();
            return;
        }
        self.cancel_tree_drag();
        if workspace == Workspace::Tools {
            self.machining_tab = self.active_tab;
            self.active_tab = RibbonTab::Tools;
        } else {
            self.active_tab = self.machining_tab;
        }
        self.workspace = workspace;
        if self.setup_draft.is_some() {
            self.end_setup_draft();
        }
        self.reload_tool_edit();
        self.status = format!("{} workspace.", workspace.label());
        self.remember();
    }
    pub(super) fn workspace_main(&self) -> Element<'_, Message> {
        if self.workspace == Workspace::Tools {
            let mut content = row![self.libraries_sidebar()]
                .height(Length::Fill)
                .width(Length::Fill);
            if !self.settings.tool_table.library_sidebar_collapsed {
                content = content.push(
                    mouse_area(
                        container(Space::new().width(1).height(Length::Fill))
                            .center_x(8)
                            .height(Length::Fill)
                            .style(|_| container::Style {
                                background: Some(Background::Color(palette::BORDER_DARK)),
                                ..container::Style::default()
                            }),
                    )
                    .interaction(iced::mouse::Interaction::ResizingColumn)
                    .on_press(Message::LibrarySidebarResizeStart),
                );
            }
            return content.push(self.tools_table()).into();
        }
        let viewport = self.machining_viewport();
        let workpiece =
            self.workpiece_axes(&viewport.setup_geometry)
                .map(|axes| wcs_view::WorkpieceOverlay {
                    axes,
                    camera: viewport.camera(),
                    scale: self.settings.view.origin_marker_scale,
                    interactive: !self.in_origin_pick() && self.operation_position_pick.is_none(),
                    browser_overlay_width: viewport.browser_overlay_width,
                    browser_overlay_height: viewport.browser_overlay_height,
                });
        let labels = grid_view::GridLabels {
            camera: viewport.camera(),
            grid_size: viewport.grid_size,
            frame: viewport.grid_frame,
            measurement: grid_view::selection_measurement(
                self.controller.geometry_state(),
                self.controller.cad_selection(),
            ),
        };
        let height_preview = self.height_preview(viewport.camera(), viewport.browser_overlay_width);
        let viewport_camera = viewport.camera();
        let viewport_browser_width = viewport.browser_overlay_width;
        let side_overlay = self.profile_side_overlay(viewport_camera, viewport_browser_width);
        let tab_overlay = self
            .profile_tab_overlay(viewport_camera, viewport_browser_width)
            .map(|overlay| {
                overlay.with_scene_mesh(
                    viewport.mesh_vertices.clone(),
                    viewport.mesh_indices.clone(),
                )
            });
        let mut layers = iced::widget::stack![
            shader(viewport).width(Length::Fill).height(Length::Fill),
            canvas::Canvas::new(labels)
                .width(Length::Fill)
                .height(Length::Fill)
        ];
        // Keep the browser at a stable sibling index. Optional scene overlays
        // appear/disappear on selection; shifting the browser would discard its
        // mouse-area state between the first and second click.
        let mut overlays =
            iced::widget::stack![Space::new().width(Length::Fill).height(Length::Fill)];
        if let Some(preview) = height_preview {
            overlays = overlays.push(
                canvas::Canvas::new(preview)
                    .width(Length::Fill)
                    .height(Length::Fill),
            );
        }
        if let Some(workpiece) = workpiece {
            overlays = overlays.push(
                canvas::Canvas::new(workpiece)
                    .width(Length::Fill)
                    .height(Length::Fill),
            );
        }
        if let Some(tabs) = tab_overlay {
            overlays = overlays.push(
                canvas::Canvas::new(tabs)
                    .width(Length::Fill)
                    .height(Length::Fill),
            );
        }
        if let Some(sides) = side_overlay {
            overlays = overlays.push(
                canvas::Canvas::new(sides)
                    .width(Length::Fill)
                    .height(Length::Fill),
            );
        }
        if let Some(handles) = self.move_handles(viewport_camera) {
            overlays = overlays.push(
                canvas::Canvas::new(handles)
                    .width(Length::Fill)
                    .height(Length::Fill),
            );
        }
        if let Some(legend) = self.simulation.as_ref().and_then(|player| {
            player.force_viewport_legend(
                viewport_browser_width,
                self.show_gizmo.then_some(self.gizmo_size),
            )
        }) {
            overlays = overlays.push(legend);
        }
        layers = layers.push(overlays.width(Length::Fill).height(Length::Fill));
        if let Some(input) = self.move_handle_input() {
            layers = layers.push(
                container(input)
                    .width(Length::Fill)
                    .height(Length::Fill)
                    .align_x(iced::alignment::Horizontal::Right)
                    .align_y(iced::alignment::Vertical::Bottom)
                    .padding(18),
            );
        }
        if self.show_project_browser {
            let browser = container(
                column![text(self.project_title()).size(13), self.project_tree()].spacing(6),
            )
            .width(Length::Fixed(self.project_px.clamp(220., 360.)))
            .height(Length::Fill)
            .max_height(self.project_browser_height())
            .padding(8)
            .style(|_| container::Style::default());
            layers = layers.push(container(browser).padding(8).height(Length::Fill));
        }
        column![
            layers.width(Length::Fill).height(Length::Fill),
            if self.operation_preview_active() {
                Element::from(Space::new())
            } else {
                self.simulation_controls()
            },
            self.viewport_pick_toolbar()
        ]
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
    }
    fn cached_cad_scene(&self, hidden: &BTreeSet<InstanceId>) -> Arc<CadScene> {
        let key = CadSceneKey {
            revision: self.controller.revision(),
            hidden: hidden.clone(),
            frame: self.controller.inspection_setup().frame.clone(),
        };
        let mut cache = self.cad_scene_cache.borrow_mut();
        if let Some((previous, scene)) = cache.as_ref() {
            if *previous == key {
                return scene.clone();
            }
        }
        let scene = Arc::new(CadScene::from_geometry_in_setup(
            self.controller.geometry_state(),
            hidden,
            &key.frame,
        ));
        *cache = Some((key, scene.clone()));
        scene
    }

    pub(super) fn machining_viewport(&self) -> Viewport {
        let mut cad_hidden = self.viewport_hidden_instances();
        if self
            .simulation
            .as_ref()
            .is_some_and(|p| !p.options.show_model)
        {
            cad_hidden.extend(
                self.controller
                    .geometry_state()
                    .instances
                    .iter()
                    .filter(|i| {
                        !self
                            .controller
                            .inspection_setup()
                            .fixture_instances
                            .contains(&i.id)
                    })
                    .map(|i| i.id),
            );
        }
        let model_view = self.model_view.is_some() || self.controller.setups().is_empty();
        let hidden: Vec<_> = self
            .controller
            .setups()
            .iter()
            .flat_map(|setup| {
                let hidden_setup = model_view || self.hidden_setups.contains(&setup.id);
                setup
                    .operations
                    .iter()
                    .map(|op| op.id())
                    .filter(move |id| hidden_setup || self.hidden_ops.contains(id))
            })
            .collect();
        let editing = self.operation_preview_active() || self.pattern_draft.is_some();
        let tree_body = match self.model_view {
            Some(TreeNode::Model(id))
                if !editing
                    && self.move_draft.is_none()
                    && self.controller.cad_selection().is_empty() =>
            {
                Some(id)
            }
            _ => None,
        };
        let selected_geometry = self.editor_selected_geometry();
        let operation_preview = self
            .operation_preview_outcome()
            .or_else(|| self.operation_preview.running_preview());
        // Playback geometry changes only when a new display frame is published.
        // Reuse its geometry during intervening ticks and camera/menu events.
        // Drafts have their own small semantic key as well.
        // Unstamped synchronous/headless results have no publication identity.
        let cacheable = self.move_draft.is_none()
            && self.controller.outcome().is_none_or(|o| o.stamp.is_some());
        let key = cacheable.then(|| ViewportKey {
            revision: self.controller.revision(),
            setup: self.controller.active_setup_id(),
            origin: self.controller.active_origin(),
            selection: self.controller.selection(),
            pending: self.controller.pending_op(),
            outcome: self
                .controller
                .outcome()
                .or_else(|| self.controller.run_preview())
                .and_then(|o| o.stamp.clone()),
            run_preview_revision: self.controller.run_preview_revision(),
            simulation: self
                .simulation
                .as_ref()
                .map(|player| SimulationViewportKey {
                    display_token: player.display_token.clone(),
                    options: player.options.clone(),
                    collision: player.collision_active(),
                }),
            operation_preview: operation_preview.and_then(|o| o.stamp.clone()),
            operation_preview_revision: self.operation_preview.preview_revision(),
            cad_selection: self.controller.cad_selection().to_vec(),
            cad_hover: self.cad_hover.clone(),
            cad_hidden: cad_hidden.clone(),
            focus: self.focus_ops.clone(),
            hidden: hidden.clone(),
            model_view,
            tree_body,
            show_stock: self.show_stock,
            show_origin: self.show_origin,
            envelope: self.show_envelope.then(|| {
                let (x, y, z) = self.controller.machine().envelope.extent();
                [x as f32, y as f32, z as f32]
            }),
            snap: self.snap_hover.map(|hit| (hit, self.snap_aperture)),
            hover_loop: self.hover_loop,
            set_origin: self.in_origin_pick(),
            origin_mode: self.in_origin_pick().then(|| self.setup_origin_mode()),
            origin_first: self.origin_first,
            view_prefs: self.settings.view.clone(),
            snap_prefs: self.settings.snapping.clone(),
            draft: self.setup_draft.clone(),
            pattern: self.pattern_draft.clone(),
            editor: editing.then(|| EditorKey {
                creating: self.creating_operation(),
                wrapped_geometry_editor: self.wrapped_pocket_geometry_editor(),
                cad_kind: self.cad_pending_kind,
                cad_editing: self.cad_editing_op,
                drawing_editing: self.drawing_editing_op,
                tool: self.wizard_tool(),
                operation: self.operation_draft.clone(),
                rotary_index: self.rotary_index_draft.clone(),
                binding: self.operation_binding_preview.clone(),
                geometry: selected_geometry.clone(),
                fields: self.fields.clone(),
                edited_fields: self.creation_edited_fields.clone(),
                tools: self.operation_tools.clone(),
                heights: self.height_draft.clone(),
                height_id: self.height_draft_id,
                offsets: self.height_offsets.clone(),
                tab_selection: self.profile_tab_selection,
                position_markers: self.operation_position_markers,
                tab: self.operation_tab,
            }),
        });
        let cached = self
            .viewport_cache
            .borrow()
            .as_ref()
            .and_then(|(previous, viewport)| {
                (key.as_ref() == Some(previous)).then(|| viewport.clone())
            });
        let mut viewport_settings = self.settings.clone();
        if let Some(player) = &self.simulation {
            viewport_settings.view.transparent_stock = player.options.transparent_stock;
        }
        let mut viewport = cached.unwrap_or_else(|| {
            // Resolve the full profile silhouette first. The base viewport can
            // then reuse its outside outline for selection highlighting.
            let editor_preview = editing.then(|| self.operation_geometry_preview());
            let silhouette_bottom = if selected_geometry
                .iter()
                .any(|reference| reference.kind == EntityKind::Silhouette)
            {
                self.operation_draft.as_ref().and_then(|operation| {
                    let id = if self.creating_operation() {
                        0
                    } else {
                        operation.id()
                    };
                    self.resolve_height_editor(id)
                        .ok()
                        .map(|heights| heights.bottom)
                })
            } else {
                None
            };
            let mut viewport = Viewport::new(
                &self.controller,
                self.simulation
                    .as_ref()
                    .filter(|_| !model_view)
                    .map(|player| &player.preview),
                operation_preview,
                if self.setup_model_pick {
                    EntityKind::Body
                } else {
                    self.cad_filter.kind()
                },
                self.cad_hover.as_ref(),
                &cad_hidden,
                self.cad_select_through,
                self.simulation.as_ref().map_or(
                    self.show_stock || self.controller.selection() == Selection::Setup,
                    |p| p.options.show_stock,
                ) && !model_view,
                self.view,
                self.show_gizmo,
                self.gizmo_size,
                &viewport_settings,
                &self.focus_ops.iter().copied().collect::<Vec<_>>(),
                &hidden,
                self.snap_hover.map(|hit| (hit, self.snap_aperture)),
                self.hover_loop,
                self.in_origin_pick(),
                self.show_origin && self.controller.selection() != Selection::Setup,
                self.origin_first.map(|p| {
                    self.setup_draft
                        .as_ref()
                        .unwrap_or_else(|| self.controller.inspection_setup())
                        .frame
                        .to_world_vector(p)
                }),
                self.show_envelope.then(|| {
                    let (x, y, z) = self.controller.machine().envelope.extent();
                    [x as f32, y as f32, z as f32]
                }),
                editing.then_some(selected_geometry.as_slice()),
                silhouette_bottom,
                Some(
                    self.move_preview_scene(&cad_hidden)
                        .unwrap_or_else(|| self.cached_cad_scene(&cad_hidden)),
                ),
                self.setup_draft.as_ref(),
                tree_body,
            );
            if let Some(player) = self.simulation.as_ref().filter(|_| !model_view && !editing) {
                if player.options.rotary_view
                    == crate::settings::SimulationRotaryView::RotatingStock
                {
                    if let Some((axis, center, angle)) = player.rotary_pose() {
                        let frame = &self.controller.inspection_setup().frame;
                        let rotate =
                            cam_render::RotaryStockTransform::new(frame, axis, center, angle);
                        rotate
                            .apply_mesh(Arc::make_mut(&mut viewport.mesh_vertices).as_mut_slice());
                        rotate.apply_mesh(
                            Arc::make_mut(&mut viewport.transparent_stock_vertices).as_mut_slice(),
                        );
                        for vertex in Arc::make_mut(&mut viewport.vertices)
                            .iter_mut()
                            .chain(Arc::make_mut(&mut viewport.cad_edges).iter_mut())
                        {
                            vertex.position = rotate.point(vertex.position);
                        }
                        for mark in Arc::make_mut(&mut viewport.drill_marks).iter_mut() {
                            mark.at = rotate.point(mark.at);
                        }
                    }
                }
                let (vertices, indices) = player.tool_mesh();
                let (target_vertices, target_indices) = if player.options.transparent_tool {
                    (
                        &mut viewport.transparent_stock_vertices,
                        &mut viewport.transparent_stock_indices,
                    )
                } else {
                    (&mut viewport.mesh_vertices, &mut viewport.mesh_indices)
                };
                let offset = target_vertices.len() as u32;
                Arc::make_mut(target_vertices).extend(vertices);
                Arc::make_mut(target_indices).extend(indices.into_iter().map(|i| i + offset));
            }
            viewport.setup_geometry = Arc::new(SetupPickGeometry::new(
                &self.controller,
                self.setup_draft
                    .as_ref()
                    .unwrap_or_else(|| self.controller.inspection_setup()),
                &cad_hidden,
            ));
            if self.in_origin_pick() && self.setup_origin_mode().is_box() {
                if let Some(draft) = &self.setup_draft {
                    let (min, max) = self.controller.pending_setup_model_bounds(draft);
                    let (min, max) = if self.setup_origin_mode() == SetupOriginMode::StockBox {
                        self.controller.pending_setup_stock_bounds(draft)
                    } else {
                        (min, max)
                    };
                    viewport.setup_geometry = Arc::new(
                        Arc::unwrap_or_clone(viewport.setup_geometry)
                            .with_origin_box(draft, min, max),
                    );
                }
            }
            if model_view {
                viewport.setup_geometry = Arc::default();
                viewport.drill_marks = Arc::default();
            }
            if let Some(draft) = &self.setup_draft {
                let geometry = self.controller.geometry_state();
                let selected: Vec<_> = draft
                    .model_instances
                    .iter()
                    .chain(&draft.fixture_instances)
                    .filter_map(|id| {
                        let instance = geometry.instance(*id)?;
                        let asset = geometry.asset(instance.asset_id)?;
                        Some(EntityRef {
                            asset_id: asset.id,
                            revision: asset.revision,
                            instance_id: *id,
                            body_id: instance.body_id,
                            kind: EntityKind::Body,
                            id: instance.body_id,
                        })
                    })
                    .collect();
                viewport.cad_edges = Arc::new(
                    viewport
                        .cad_scene
                        .edges(&selected, self.cad_hover.as_ref())
                        .line_vertices(),
                );
            }
            if let Some(preview) = editor_preview {
                Arc::make_mut(&mut viewport.vertices).extend(preview.line_vertices());
                let (vertices, indices) = self.profile_tab_mesh();
                let offset = viewport.mesh_vertices.len() as u32;
                Arc::make_mut(&mut viewport.mesh_vertices).extend(vertices);
                Arc::make_mut(&mut viewport.mesh_indices)
                    .extend(indices.into_iter().map(|i| i + offset));
            }
            if editing {
                let overlay = self.index_face_overlay(&viewport.cad_scene);
                Arc::make_mut(&mut viewport.vertices).extend(overlay.line_vertices());
            }
            if self.pattern_draft.is_some() {
                let preview = self.pattern_placement_preview();
                viewport.bounds = union_bounds(viewport.bounds, preview.bounds());
                Arc::make_mut(&mut viewport.vertices).extend(preview.line_vertices());
            }
            if let Some(key) = key {
                *self.viewport_cache.borrow_mut() = Some((key, viewport.clone()));
            }
            viewport
        });
        viewport.cad_filter = if self.setup_model_pick {
            EntityKind::Body
        } else {
            self.cad_filter.kind()
        };
        viewport.cad_filter_explicit = self.cad_filter_explicit
            || self.setup_model_pick
            || self.height_reference_pick.is_some();
        viewport.geometry_pick_active = (self.geometry_additive || self.geometry_auto_pick)
            && self.operation_tab == OperationTab::Geometry;
        viewport.stock_contour_picking = self.stock_contour_editor();
        viewport.stock_contour_setup_selected = self.stock_contour_editor()
            && if self.operation_uses_cad_geometry() {
                self.controller.cad_selection().is_empty()
            } else {
                self.controller.pending_operation_geometry().is_empty()
            };
        viewport.cad_select_through = self.cad_select_through;
        let (grid_x, grid_y, grid_z) = self.controller.machine().envelope.extent();
        viewport.grid_size = [grid_x as f32, grid_y as f32, grid_z as f32];
        viewport.grid_frame = if model_view {
            Default::default()
        } else if let Some(setup) = self
            .setup_draft
            .as_ref()
            .or_else(|| self.controller.document().active_setup())
        {
            cam_model::RigidTransform {
                rotation: setup.frame.world_from_setup.rotation,
                translation: setup
                    .frame
                    .to_world_vector(setup.origin_position(self.controller.active_origin())),
            }
        } else {
            Default::default()
        };
        viewport.controls = self.view;
        viewport.show_gizmo = self.show_gizmo;
        viewport.gizmo_size = self.gizmo_size;
        let pattern_pick = self.pattern_draft.as_ref().and_then(|draft| draft.picking);
        viewport.setup_axis =
            self.setup_axis_pick.is_some() || pattern_pick.is_some_and(|axis| axis < 2);
        viewport.stock_axis_model_only = self.setup_axis_pick == Some(3);
        if pattern_pick.is_some() {
            viewport.picking = true;
            viewport.cad_filter = EntityKind::Edge;
            viewport.cad_filter_explicit = true;
            viewport.cad_operation_kind = None;
        }
        viewport.operation_position_picking = self.operation_position_pick.is_some()
            && self.operation_preview_active()
            && self.operation_tab == OperationTab::Linking
            && !model_view;
        viewport.operation_position_color = self
            .operation_position_pick
            .unwrap_or(operation_positions::PositionKind::Entry)
            .color();
        viewport.setup_hover = self.setup_hover.clone();
        viewport.setup_snaps = if self.settings.snapping.enabled {
            self.snaps.clone()
        } else {
            vec![]
        };
        viewport.frame_bounds = union_bounds(
            if model_view {
                None
            } else {
                viewport.setup_geometry.stock_bounds()
            },
            viewport.cad_scene.bounds(),
        );
        if self.move_draft.is_some() {
            viewport.frame_bounds = self.cached_cad_scene(&cad_hidden).bounds();
        }
        if self.pattern_draft.is_some() {
            viewport.frame_bounds = union_bounds(viewport.frame_bounds, viewport.bounds);
        }
        viewport.cad_silhouette = self.modifiers.control()
            && self.height_reference_pick.is_none()
            && !self.setup_model_pick;
        viewport.cad_operation_kind = self
            .cad_pending_kind
            .or_else(|| {
                self.cad_editing_op.and_then(|id| {
                    self.current_operation(id)
                        .map(AppController::operation_kind)
                })
            })
            .or_else(|| {
                (self.geometry_auto_pick && self.operation_tab == OperationTab::Geometry)
                    .then(|| self.geometry_operation_kind())
                    .flatten()
            });
        if self
            .profile_entry_editor
            .as_ref()
            .is_some_and(|e| e.picking)
        {
            viewport.cad_operation_kind = Some(OpKind::Profile);
        }
        if viewport.cad_operation_kind == Some(OpKind::Profile)
            && !self.setup_model_pick
            && !viewport.setup_axis
            && self.height_reference_pick.is_none()
        {
            viewport.cad_filter = match self.profile_selection_mode {
                ProfileSelectionMode::FaceContours => EntityKind::Face,
                ProfileSelectionMode::Chain if self.profile_chain_edges => EntityKind::Edge,
                _ => EntityKind::Loop,
            };
            viewport.cad_silhouette |= self.cad_filter_explicit
                && self.profile_selection_mode == ProfileSelectionMode::Silhouette;
        }
        if matches!(
            viewport.cad_operation_kind,
            Some(
                OpKind::Pocket
                    | OpKind::Chamfer
                    | OpKind::Engrave
                    | OpKind::Carve
                    | OpKind::Bore
                    | OpKind::Thread
            )
        ) {
            viewport.cad_silhouette = false;
        }
        if self.height_reference_pick.is_some() {
            viewport.cad_operation_kind = None;
        }
        if self.rotary_pick.is_some() || self.wrapped_pocket_geometry_editor() {
            viewport.cad_operation_kind = Some(OpKind::Engrave);
            viewport.cad_filter_explicit = true;
            viewport.cad_silhouette = false;
            if let Some(pick) = self.rotary_pick {
                if matches!(
                    pick,
                    rotary_view::Pick::IndexFace
                        | rotary_view::Pick::IndexLoop
                        | rotary_view::Pick::IndexEdge
                ) {
                    // Index references may lie in any plane, not just setup XY.
                    viewport.cad_operation_kind = None;
                }
                // A pending drawing operation otherwise intercepts the click as a
                // planar pick before the CAD face can be hit-tested.
                viewport.picking = false;
                viewport.cad_filter = match pick {
                    rotary_view::Pick::Cylinder
                    | rotary_view::Pick::Faces
                    | rotary_view::Pick::IndexFace => EntityKind::Face,
                    rotary_view::Pick::Edges | rotary_view::Pick::IndexEdge => EntityKind::Edge,
                    rotary_view::Pick::Loops | rotary_view::Pick::IndexLoop => EntityKind::Loop,
                };
            }
        }
        if self.stock_contour_editor() {
            viewport.cad_operation_kind = None;
            viewport.cad_filter_explicit = true;
            viewport.cad_silhouette =
                self.cad_filter == CadFilter::Silhouette || self.modifiers.control();
        }
        if pattern_pick.is_some() {
            viewport.cad_operation_kind = None;
            viewport.cad_filter = EntityKind::Edge;
            viewport.cad_silhouette = false;
        }
        viewport.browser_overlay_width = if self.show_project_browser {
            self.project_px.clamp(220., 360.)
        } else {
            0.
        };
        viewport.browser_overlay_height = self.project_browser_height();
        viewport.planar_features = if self.simulation.is_some() {
            Vec::new()
        } else {
            self.planar_selection.clone()
        };
        viewport
    }
    fn viewport_pick_toolbar(&self) -> Element<'_, Message> {
        let toggle = |label: &'static str, active, message| {
            button(text(label).size(12))
                .padding([4, 8])
                .on_press(message)
                .style(move |_, status| snap_toggle_style(active, status))
        };
        if let Some(player) = &self.simulation {
            let p = &player.options;
            return container(
                row![
                    toggle(
                        "Tool",
                        p.show_tool,
                        Message::SimulationOption(SimulationOption::Tool(!p.show_tool))
                    ),
                    toggle(
                        "Toolpath",
                        p.show_toolpath,
                        Message::SimulationOption(SimulationOption::Toolpath(!p.show_toolpath))
                    ),
                    toggle("Stock · F8", p.show_stock, Message::ToggleStock),
                    toggle(
                        "Transparent stock",
                        p.transparent_stock,
                        Message::ToggleTransparentStock
                    ),
                    toggle(
                        "Model",
                        p.show_model,
                        Message::SimulationOption(SimulationOption::Model(!p.show_model))
                    ),
                ]
                .spacing(6)
                .wrap(),
            )
            .padding(6)
            .width(Length::Fill)
            .into();
        }
        let hint = self
            .setup_hover
            .snap
            .as_ref()
            .map(|hit| {
                let kind = match hit.kind {
                    SnapKind::End => "Endpoint",
                    SnapKind::Mid => "Midpoint",
                    SnapKind::Quadrant => "Quadrant",
                    SnapKind::Nearest => "Nearest",
                };
                let source = match hit.source {
                    SetupFeature::Stock(_) => "stock",
                    SetupFeature::BoxPoint(_) => "box",
                    SetupFeature::Model(_) => "model",
                    SetupFeature::Drawing(_) => "drawing",
                };
                format!("{} · {}", kind, source)
            })
            .unwrap_or_default();
        let position_pick = self.operation_position_pick.is_some()
            && self.operation_preview_active()
            && self.operation_tab == OperationTab::Linking
            && self.model_view.is_none()
            && !self.controller.setups().is_empty();
        let origin_pick = self.in_origin_pick() && !self.setup_origin_mode().is_box();
        let axis_pick = self.setup_axis_pick.is_some()
            || self
                .pattern_draft
                .as_ref()
                .and_then(|draft| draft.picking)
                .is_some_and(|axis| axis < 2);
        let drawing_pick = self
            .controller
            .pending_op()
            .is_some_and(|p| op_uses_snaps(p.kind));
        let mut controls = row![
            toggle("Show stock · F8", self.show_stock, Message::ToggleStock),
            toggle(
                "Transparent stock",
                self.settings.view.transparent_stock,
                Message::ToggleTransparentStock
            ),
        ];
        if origin_pick || position_pick || axis_pick {
            controls = controls.push(toggle(
                "Highlight features",
                self.settings.snapping.highlight_features,
                Message::ToggleFeatureHighlights,
            ));
        }
        if origin_pick || position_pick || drawing_pick {
            controls = controls
                .push(toggle(
                    "Snap",
                    self.settings.snapping.enabled,
                    Message::ToggleSnapping,
                ))
                .push(toggle(
                    "Endpoints",
                    self.snaps.contains(&SnapKind::End),
                    Message::ToggleSnap(SnapKind::End),
                ))
                .push(toggle(
                    "Midpoints",
                    self.snaps.contains(&SnapKind::Mid),
                    Message::ToggleSnap(SnapKind::Mid),
                ))
                .push(toggle(
                    "Quadrants",
                    self.snaps.contains(&SnapKind::Quadrant),
                    Message::ToggleSnap(SnapKind::Quadrant),
                ))
                .push(toggle(
                    "Nearest",
                    self.snaps.contains(&SnapKind::Nearest),
                    Message::ToggleSnap(SnapKind::Nearest),
                ))
                .push(text(hint).size(12));
        }
        let controls = controls.spacing(4).align_y(Alignment::Center);
        container(
            scrollable(controls).direction(scrollable::Direction::Horizontal(
                scrollable::Scrollbar::default(),
            )),
        )
        .padding(6)
        .width(Length::Fill)
        .style(|_| container::Style {
            background: Some(Background::Color(palette::RIBBON_BG)),
            ..Default::default()
        })
        .into()
    }
    pub(super) fn tool_table_row_values(&self, tool: &cam_model::Tool) -> [String; 8] {
        [
            format!("T{}", tool.number),
            tool.description.clone(),
            tool.kind.to_string(),
            tool.manufacturer.clone(),
            tool.product_id.clone(),
            self.settings.format_number(tool.diameter),
            self.settings.format_number(tool.length),
            self.settings.format_number(tool.flute_len()),
        ]
    }

    pub(super) fn tool_table_values(&self, column: usize) -> BTreeSet<String> {
        self.library
            .tools
            .iter()
            .enumerate()
            .map(|(index, committed)| {
                let tool = if self.operation_tool_selector.is_some() {
                    committed
                } else if index == self.lib_sel {
                    self.tool_edit.as_ref().unwrap_or(committed)
                } else {
                    self.tool_drafts
                        .get(&index)
                        .map(|draft| &draft.tool)
                        .unwrap_or(committed)
                };
                self.tool_table_row_values(tool)[column].clone()
            })
            .collect()
    }

    pub(super) fn tool_table_menu_overlay(&self) -> Option<Element<'_, Message>> {
        let (index, visibility) = self.tool_table_menu?;
        let title = TOOL_TABLE_LABELS.get(index).copied().unwrap_or("Columns");
        let mut content = column![text(title).size(14)].spacing(6);
        if visibility {
            if index < 8 {
                content = content.push(
                    inspector_button("Hide this column")
                        .on_press(Message::SetToolTableColumn(index, false)),
                );
            }
            for (column, label) in TOOL_TABLE_LABELS.iter().enumerate() {
                content = content.push(
                    checkbox(self.tool_table_columns[column])
                        .size(15)
                        .text_size(12)
                        .spacing(8)
                        .label(*label)
                        .on_toggle(move |checked| Message::SetToolTableColumn(column, checked)),
                );
            }
            content = content.push(inspector_button("Done").on_press(Message::CloseToolTableMenu));
        } else {
            let search = self.tool_table_menu_search.trim().to_lowercase();
            let values: Vec<_> = self
                .tool_table_values(index)
                .into_iter()
                .filter(|value| value.to_lowercase().contains(&search))
                .collect();
            let all_selected = values
                .iter()
                .all(|value| self.tool_table_filter_draft.contains(value));
            let mut choices = column![checkbox(all_selected)
                .size(15)
                .text_size(12)
                .spacing(8)
                .label("Select all")
                .on_toggle(Message::ToolTableFilterAll)]
            .spacing(4);
            for value in values {
                let checked = self.tool_table_filter_draft.contains(&value);
                let label = if value.is_empty() {
                    "(Blanks)".to_string()
                } else {
                    value.clone()
                };
                choices = choices.push(
                    checkbox(checked)
                        .size(15)
                        .text_size(12)
                        .spacing(8)
                        .label(label)
                        .on_toggle(move |checked| {
                            Message::ToolTableFilterValue(value.clone(), checked)
                        }),
                );
            }
            content = content
                .push(
                    inspector_button("Clear filter").on_press(Message::ClearToolTableColumnFilter),
                )
                .push(
                    inspector_input("Search values", &self.tool_table_menu_search)
                        .on_input(Message::ToolTableMenuSearch),
                )
                .push(scrollable(choices).height(180))
                .push(
                    row![
                        inspector_button("OK")
                            .style(inspector_primary)
                            .on_press(Message::ApplyToolTableFilter),
                        inspector_button("Cancel").on_press(Message::CloseToolTableMenu)
                    ]
                    .spacing(8),
                );
        }
        let popup = container(content)
            .width(230)
            .padding(10)
            .style(|_| container::Style {
                background: Some(Background::Color(palette::RIBBON_BG)),
                border: Border {
                    color: palette::BORDER_DARK,
                    width: 1.0,
                    radius: 3.0.into(),
                },
                ..Default::default()
            });
        Some(
            container(popup)
                .width(Length::Fill)
                .height(Length::Fill)
                .align_x(iced::alignment::Horizontal::Left)
                .align_y(iced::alignment::Vertical::Top)
                .padding(Padding {
                    top: self
                        .tool_table_menu_pos
                        .y
                        .min((self.window.height - 390.0).max(0.0)),
                    left: self
                        .tool_table_menu_pos
                        .x
                        .min((self.window.width - 230.0).max(0.0)),
                    right: 0.0,
                    bottom: 0.0,
                })
                .into(),
        )
    }

    /// Library selection and management are kept beside the cutter table.
    fn libraries_sidebar(&self) -> Element<'_, Message> {
        if self.settings.tool_table.library_sidebar_collapsed {
            return container(inspector_button("›").on_press(Message::ToggleLibrarySidebar))
                .padding(5)
                .height(Length::Fill)
                .style(|_| container::Style {
                    background: Some(Background::Color(palette::RIBBON_BG)),
                    ..container::Style::default()
                })
                .into();
        }
        let mut entries = column![].spacing(2);
        for name in self.libraries.libraries.keys() {
            if self.library_renaming && *name == self.libraries.active {
                entries = entries.push(
                    row![
                        inspector_input("Library name", &self.library_name_edit)
                            .on_input(Message::LibraryNameChanged)
                            .on_submit(Message::RenameToolLibrary)
                            .width(Length::Fill),
                        inspector_button("✓").on_press(Message::RenameToolLibrary),
                        inspector_button("×").on_press(Message::CancelLibraryRename),
                    ]
                    .spacing(3),
                );
            } else {
                entries = entries.push(
                    mouse_area(select_row(
                        name.clone(),
                        *name == self.libraries.active,
                        Message::SelectToolLibrary(name.clone()),
                    ))
                    .on_right_press(Message::OpenLibraryContextMenu(name.clone())),
                );
            }
        }
        let active = self.libraries.active.clone();
        let heading = row![
            text("Libraries").size(14),
            Space::new().width(Length::Fill),
            inspector_button("+").on_press(Message::NewToolLibrary),
            inspector_button("⋯").on_press(Message::OpenLibraryContextMenu(active.clone())),
            inspector_button("‹").on_press(Message::ToggleLibrarySidebar),
        ]
        .spacing(6)
        .align_y(Alignment::Center);
        let list = mouse_area(
            container(scrollable(entries).height(Length::Fill))
                .width(Length::Fill)
                .height(Length::Fill),
        )
        .on_right_press(Message::OpenLibraryContextMenu(active));
        container(column![heading, list].spacing(8).padding(10))
            .width(self.settings.tool_table.library_sidebar_width)
            .height(Length::Fill)
            .style(|_| container::Style {
                background: Some(Background::Color(palette::RIBBON_BG)),
                border: Border {
                    color: palette::BORDER_DARK,
                    width: 1.0,
                    radius: 0.0.into(),
                },
                ..container::Style::default()
            })
            .into()
    }

    pub(super) fn tools_table(&self) -> Element<'_, Message> {
        let mode = |view, label| {
            let active = self.library_view == view;
            button(text(label).size(12))
                .on_press(Message::SetLibraryView(view))
                .style(move |_, status| snap_toggle_style(active, status))
        };
        let widths = self.tool_table_widths;
        let mut controls = row![].spacing(12).align_y(Alignment::Center);
        if self.operation_tool_selector.is_some() {
            controls = controls
                .push(text("Tool library").size(14))
                .push(self.library_picker());
        }
        controls = controls
            .push(mode(LibraryView::Ordered, "By number"))
            .push(mode(LibraryView::Grouped, "By family"))
            .push(
                inspector_input("Search tools", &self.tool_table_search)
                    .on_input(Message::ToolTableSearch)
                    .width(180),
            )
            .push(inspector_button("Clear filters").on_press(Message::ClearToolTableFilters))
            .push(inspector_button("Columns").on_press(Message::OpenToolTableMenu(8, true)));
        let mut headers = row![].spacing(4);
        for &index in &self.tool_table_order {
            if self.tool_table_columns[index] {
                let indicator = if self.tool_table_filters[index].is_some() {
                    "▼•"
                } else {
                    "▼"
                };
                let target = matches!(self.tool_table_drag, Some(ToolColumnDrag::Reorder { target: Some((target, _)), .. }) if target == index);
                let label = mouse_area(
                    container(text(TOOL_TABLE_LABELS[index]).size(12))
                        .width(Length::Fill)
                        .clip(true),
                )
                .interaction(iced::mouse::Interaction::Grab)
                .on_press(Message::ToolColumnReorderStart(index));
                let handle = mouse_area(
                    container(text("│").size(12).color(palette::GROUP_LABEL))
                        .width(8)
                        .center_x(8),
                )
                .interaction(iced::mouse::Interaction::ResizingColumn)
                .on_press(Message::ToolColumnResizeStart(index));
                let header = row![
                    label,
                    button(text(indicator).size(10))
                        .padding([2, 4])
                        .style(move |_, status| command_toggle_style(
                            self.tool_table_filters[index].is_some(),
                            status
                        ))
                        .on_press(Message::OpenToolTableMenu(index, false)),
                    handle
                ]
                .align_y(Alignment::Center)
                .width(widths[index]);
                headers = headers.push(
                    mouse_area(container(header).style(move |_| container::Style {
                        background: target.then_some(Background::Color(palette::SELECT_BG)),
                        ..Default::default()
                    }))
                    .on_move(move |point| {
                        Message::ToolColumnHover(index, point.x >= widths[index] / 2.0)
                    })
                    .on_exit(Message::ToolColumnLeave(index))
                    .on_right_press(Message::OpenToolTableMenu(index, true)),
                );
            }
        }
        let toolbar = column![controls].spacing(6).padding([8, 16]);
        let mut list = column![container(headers).padding(Padding::from([0.0, 7.0]))]
            .spacing(6)
            .padding([0, 16]);
        let mut tools: Vec<_> = self.library.tools.iter().enumerate().collect();
        match self.library_view {
            LibraryView::Ordered => tools.sort_by_key(|(_, tool)| tool.number),
            LibraryView::Grouped => tools.sort_by(|(_, a), (_, b)| {
                kind_order(a.kind)
                    .cmp(&kind_order(b.kind))
                    .then(a.diameter.total_cmp(&b.diameter))
            }),
        }
        for (index, committed) in tools {
            let selecting = self.operation_tool_selector.is_some();
            let selected = if selecting {
                self.operation_preset_tool()
                    .is_some_and(|t| t.identity() == committed.identity())
            } else {
                index == self.lib_sel
            };
            let draft = self.tool_drafts.get(&index);
            let tool = if self.operation_tool_selector.is_some() {
                committed
            } else if selected {
                self.tool_edit.as_ref().unwrap_or(committed)
            } else {
                draft.map(|draft| &draft.tool).unwrap_or(committed)
            };
            let pending = !selecting && (draft.is_some() || (selected && self.inspector_dirty()));
            let values = self.tool_table_row_values(tool);
            if !tool_table_matches(&values, &self.tool_table_search, &self.tool_table_filters) {
                continue;
            }
            let mut cells = row![].spacing(4);
            for &column in &self.tool_table_order {
                let value = &values[column];
                if self.tool_table_columns[column] {
                    let display = if column == 0 && pending {
                        format!("{value} *")
                    } else {
                        value.clone()
                    };
                    cells = cells.push(
                        container(text(display).size(12))
                            .width(widths[column])
                            .clip(true),
                    );
                }
            }
            let item = container(cells)
                .padding(7)
                .width(Length::Fill)
                .style(move |_| container::Style {
                    background: selected.then_some(Background::Color(palette::SELECT_BG)),
                    ..Default::default()
                });
            if selecting {
                let supported = self
                    .operation_tool_selector
                    .as_ref()
                    .is_some_and(|selector| selector.supports(tool));
                list = list.push(
                    button(item)
                        .padding(0)
                        .style(inspector_secondary)
                        .on_press_maybe(supported.then_some(Message::OperationToolPicked(index))),
                );
            } else {
                list = list.push(
                    mouse_area(item)
                        .on_press(Message::SelectLibraryTool(index))
                        .on_right_press(Message::LibToolMenu(index)),
                );
            }
        }
        let cutters = scrollable(list.width(Length::Shrink))
            .direction(scrollable::Direction::Both {
                vertical: scrollable::Scrollbar::default(),
                horizontal: scrollable::Scrollbar::default(),
            })
            .height(Length::FillPortion(3))
            .width(Length::Fill);
        if self.operation_tool_selector.is_some() {
            return column![toolbar, cutters].height(Length::Fill).into();
        }
        column![toolbar, cutters, self.cutting_presets_table()]
            .spacing(8)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }
    pub(super) fn tools_inspector(&self) -> Element<'_, Message> {
        let mut body = column![].spacing(6);
        if let Some(tool) = self.preview_tool() {
            body = body.push(
                canvas::Canvas::new(ToolCanvas::new(&tool).selected(self.selected_tool_parameter))
                    .width(Length::Fill)
                    .height(270),
            );
        }
        body = body.push(self.library_editor());
        if !self.tool_drafts.is_empty() {
            body = body.push(
                text(format!(
                "Unsaved changes in {} other tool(s). Select each marked tool to Apply or Cancel.",
                self.tool_drafts.len()
            ))
                .size(11)
                .color(INSPECTOR_MUTED),
            );
        }
        column![
            body.height(Length::Fill),
            inspector_footer(
                row![
                    inspector_button("Cancel").on_press(Message::CancelToolDraft),
                    Space::new().width(Length::Fill),
                    inspector_button("Apply changes")
                        .style(inspector_primary)
                        .on_press_maybe(
                            (!self.any_field_invalid() && self.inspector_dirty())
                                .then_some(Message::Apply)
                        ),
                ]
                .spacing(8)
            )
        ]
        .height(Length::Fill)
        .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotating_stock_view_inverts_material_coordinates_in_setup_axes() {
        let frame = cam_model::SetupFrame::from_xy([0., 1., 0.], [-1., 0., 0.]).unwrap();
        let center = cam_cldata::Point3::new(3., 4., 5.);
        let machine = cam_cldata::Point3::new(8., 9., 10.);
        for axis in [cam_cldata::RotaryAxis::A, cam_cldata::RotaryAxis::B] {
            let material = axis.material_point(machine, center, 37.);
            let world_material = frame
                .to_world_vector([material.x, material.y, material.z])
                .map(|v| v as f32);
            let result = stock_to_machine_view(&frame, axis, center, 37., world_material, false);
            let expected = frame.to_world_vector([machine.x, machine.y, machine.z]);
            for i in 0..3 {
                assert!((f64::from(result[i]) - expected[i]).abs() < 1e-5);
            }
        }
    }

    #[test]
    fn selection_shows_only_selected_toolpaths_and_setup_shows_configured_stock() {
        let mut app = picking_app();
        app.end_setup_draft();
        app.show_origin = false;
        app.show_envelope = false;
        app.show_stock = false;
        let mut ids = Vec::new();
        for x in [10., 25.] {
            app.controller
                .add_operation(Operation::Drill(cam_model::DrillOp {
                    parameter_expressions: Default::default(),
                    id: 0,
                    tool: 1,
                    points: vec![[x, 10.]],
                    depth: 2.,
                    start_offset: 0.,
                    peck: Some(1.),
                    dwell: Some(0.2),
                    spindle_rpm: 1000.,
                    tool_preset: None,
                    cooling: Default::default(),
                    entry_feeds: Default::default(),
                    work_offset: 1,
                    feed: 100.,
                }));
            let Selection::Operation(id) = app.controller.selection() else {
                panic!()
            };
            ids.push(id);
        }
        app.controller.run(&cam_toolpath::CancelToken::new());
        assert_eq!(
            app.controller.outcome().unwrap().completion,
            JobCompletion::Complete
        );
        let scene = app.controller.outcome().unwrap().scene.clone();
        let expected = |selected: &[u32]| {
            let mut scene = scene.clone();
            scene
                .strips
                .retain(|strip| strip.op.is_none_or(|id| selected.contains(&id)));
            scene.line_vertices()
        };
        app.clear_browser_selection();
        assert_eq!(app.machining_viewport().vertices.as_slice(), expected(&[]));
        assert!(!app.machining_viewport().show_stock);
        assert!(app.machining_viewport().drill_marks.is_empty());
        assert!(app.machining_viewport().mesh_indices.is_empty());

        let _ = app.update(Message::ClickOp(ids[0]));
        assert_eq!(
            app.machining_viewport().vertices.as_slice(),
            expected(&ids[..1])
        );
        let first_marks = app.machining_viewport().drill_marks;
        assert!(!first_marks.is_empty());
        assert!(app.operation_draft.is_none());
        assert!(app.pane_handle(Pane::Inspector).is_none());

        app.modifiers = iced::keyboard::Modifiers::COMMAND;
        let _ = app.update(Message::ClickOp(ids[1]));
        assert_eq!(app.machining_viewport().vertices.as_slice(), expected(&ids));
        assert_eq!(
            app.machining_viewport().drill_marks.len(),
            first_marks.len() * 2
        );
        app.hidden_ops.insert(ids[0]);
        assert_eq!(
            app.machining_viewport().vertices.as_slice(),
            expected(&ids[1..])
        );
        assert_eq!(
            app.machining_viewport().drill_marks.len(),
            first_marks.len()
        );
        app.hidden_ops.clear();
        app.modifiers = iced::keyboard::Modifiers::default();
        app.show_stock = true;
        let _ = app.update(Message::CadPicked(vec![]));
        assert_eq!(app.machining_viewport().vertices.as_slice(), expected(&[]));
        assert!(!app.machining_viewport().show_stock);

        let setup = app.controller.active_setup_id();
        let _ = app.update(Message::SelectSetup(setup));
        let viewport = app.machining_viewport();
        let (vertices, indices) = uncut_stock_mesh(&app.controller);
        assert!(
            viewport.show_stock,
            "setup selection exposes its stock even with the toggle off"
        );
        assert_eq!(viewport.mesh_indices.as_slice(), indices);
        assert_eq!(viewport.mesh_vertices.as_slice(), vertices);
        assert_eq!(viewport.vertices.as_slice(), expected(&[]));
        assert!(viewport.drill_marks.is_empty());
        assert!(app.pane_handle(Pane::Inspector).is_none());

        let _ = app.update(Message::PickWorld([10., 0.], 1.));
        assert!(
            !app.machining_viewport().show_stock,
            "drawing selection leaves only the model"
        );
        assert!(app.model_view.is_some());
        let _ = app.update(Message::ClickOp(ids[0]));
        let _ = app.update(Message::ClickOp(ids[0]));
        assert_eq!(app.machining_viewport().vertices.as_slice(), expected(&[]));
        assert!(!app.machining_viewport().show_stock);
    }

    #[test]
    fn inherited_setup_shows_the_processed_input_blank() {
        let mut app = picking_app();
        app.end_setup_draft();
        app.controller.edit_setup(|s| s.operations.clear());
        app.controller.edit_stock(|s| {
            *s = cam_model::Stock::FixedBox {
                size: [10., 10., 5.],
                model_position: [0., 0., 1.],
            }
        });
        app.controller
            .edit_heights(|h| *h = cam_model::Heights::new(5., 2., 0.));
        app.controller.edit_tool(0, |t| {
            t.diameter = 2.;
            t.flute_length = 12.;
        });
        let (min, max) = app.controller.stock_box();
        let center = [(min[0] + max[0]) / 2., (min[1] + max[1]) / 2.];
        app.controller
            .add_operation(Operation::Drill(cam_model::DrillOp {
                parameter_expressions: Default::default(),
                id: 0,
                tool: 1,
                points: vec![center],
                depth: 3.,
                start_offset: 0.,
                peck: None,
                dwell: None,
                spindle_rpm: 1000.,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                feed: 100.,
            }));
        app.controller.add_setup();
        app.controller
            .edit_stock(|s| *s = cam_model::Stock::PreviousSetup);
        assert!(
            uncut_stock_mesh(&app.controller).0.is_empty(),
            "uncomputed stock must not appear as a fresh box"
        );
        app.rerun();
        assert!(
            app.controller.run_pending(),
            "an empty inherited-stock setup still needs its input blank calculated"
        );
        let outcome = app.controller.run(&cam_toolpath::CancelToken::new());
        assert_eq!(
            outcome.completion,
            JobCompletion::Complete,
            "{:?}",
            outcome.diagnostics
        );
        let initial = outcome.initial_stock.clone().unwrap();
        assert!(initial.0.iter().any(|v| (v.position[2] + 3.).abs() < 1e-4));
        app.controller.select(Selection::Setup);
        assert_eq!(uncut_stock_mesh(&app.controller), initial);
        let viewport = app.machining_viewport();
        assert_eq!(viewport.mesh_vertices.as_slice(), initial.0);
        assert_eq!(viewport.mesh_indices.as_slice(), initial.1);
    }

    #[test]
    fn stock_visibility_and_transparency_work_before_and_after_simulation() {
        let mut app = picking_app();
        app.setup_draft = None;
        app.show_stock = true;
        app.settings.view.transparent_stock = false;
        let opaque = app.machining_viewport();
        assert_eq!(opaque.mesh_indices.len(), 36, "uncut stock before a run");
        assert!(opaque.transparent_stock_indices.is_empty());
        assert!(opaque
            .mesh_vertices
            .iter()
            .all(|v| v.color == [0.72, 0.66, 0.22, 1.0]));
        let frame_bounds = opaque.frame_bounds;
        let drawing = opaque.vertices.clone();

        app.settings.view.stock_color = [0.8, 0.3, 0.1];
        app.settings.view.transparent_stock = true;
        app.settings.view.stock_transparency = 0.5;
        let transparent = app.machining_viewport();
        assert!(transparent.mesh_indices.is_empty());
        assert_eq!(transparent.transparent_stock_indices.len(), 36);
        assert!(transparent
            .transparent_stock_vertices
            .iter()
            .all(|v| v.color == [0.72, 0.66, 0.22, 0.5]));
        assert_eq!(transparent.frame_bounds, frame_bounds);
        assert_eq!(transparent.vertices, drawing, "drawing stays visible");

        app.show_stock = false;
        let hidden = app.machining_viewport();
        assert!(hidden.mesh_indices.is_empty() && hidden.transparent_stock_indices.is_empty());
        assert_eq!(
            hidden.frame_bounds, frame_bounds,
            "visibility must not reframe"
        );

        app.controller
            .add_operation(Operation::Drill(cam_model::DrillOp {
                parameter_expressions: Default::default(),
                id: 0,
                tool: 1,
                points: vec![[10., 10.]],
                depth: 2.,
                start_offset: 0.,
                peck: None,
                dwell: None,
                spindle_rpm: 1000.,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                feed: 100.,
            }));
        app.controller.run(&cam_toolpath::CancelToken::new());
        app.show_stock = true;
        let Selection::Operation(id) = app.controller.selection() else {
            panic!()
        };
        app.focus_ops.insert(id);
        let simulated = app.machining_viewport();
        let outcome = app.controller.outcome().unwrap();
        assert_eq!(
            simulated.transparent_stock_indices.as_slice(),
            outcome.stock_indices
        );
        assert_eq!(
            simulated.transparent_stock_vertices.len(),
            outcome.stock_vertices.len()
        );
        assert!(!simulated.transparent_stock_indices.is_empty());
        assert!(outcome.stock_machined.iter().any(|&cut| cut));
        assert!(outcome.stock_machined.iter().any(|&cut| !cut));
        for (vertex, owner) in simulated
            .transparent_stock_vertices
            .iter()
            .zip(&outcome.stock_operations)
        {
            let expected = if *owner == Some(id) {
                [0.8, 0.3, 0.1, 0.5]
            } else {
                [0.72, 0.66, 0.22, 0.5]
            };
            assert_eq!(vertex.color, expected);
        }
        app.settings.view.transparent_stock = false;
        app.settings.view.stock_color = [0.2, 0.65, 0.3];
        let opaque = app.machining_viewport();
        for (vertex, owner) in opaque.mesh_vertices.iter().zip(&outcome.stock_operations) {
            let expected = if *owner == Some(id) {
                [0.2, 0.65, 0.3, 1.0]
            } else {
                [0.72, 0.66, 0.22, 1.0]
            };
            assert_eq!(vertex.color, expected);
        }
    }

    #[test]
    fn stock_highlights_follow_selected_operations_and_hidden_paths() {
        let mut app = picking_app();
        app.end_setup_draft();
        app.show_origin = false;
        app.show_stock = true;
        app.settings.view.transparent_stock = false;
        app.settings.view.stock_color = [0.1, 0.9, 0.2];
        let mut ids = Vec::new();
        for x in [10., 25.] {
            app.controller
                .add_operation(Operation::Drill(cam_model::DrillOp {
                    parameter_expressions: Default::default(),
                    id: 0,
                    tool: 1,
                    points: vec![[x, 10.]],
                    depth: 2.,
                    start_offset: 0.,
                    peck: None,
                    dwell: None,
                    spindle_rpm: 1000.,
                    tool_preset: None,
                    cooling: Default::default(),
                    entry_feeds: Default::default(),
                    work_offset: 1,
                    feed: 100.,
                }));
            let Selection::Operation(id) = app.controller.selection() else {
                panic!()
            };
            ids.push(id);
        }
        app.controller.run(&cam_toolpath::CancelToken::new());
        let outcome = app.controller.outcome().unwrap().clone();
        assert!(ids
            .iter()
            .all(|id| outcome.stock_operations.contains(&Some(*id))));
        assert!(
            outcome
                .stock_operations
                .iter()
                .flatten()
                .any(|op| !ids.contains(op)),
            "the setup also contains an earlier, unselected operation"
        );
        for &selected in &ids {
            app.clear_browser_selection();
            let _ = app.update(Message::ClickOp(selected));
            app.drain_operation_preview();
            let viewport = app.machining_viewport();
            let scoped = app.operation_preview_outcome().unwrap();
            assert!(!scoped.stock_indices.is_empty());
            assert!(scoped.stock_operations.contains(&Some(selected)));
            assert_eq!(viewport.path_cones.len(), 2);
            assert!(viewport.path_cones.iter().all(|cone| cone.op == selected));
            assert_eq!(viewport.mesh_indices.as_slice(), scoped.stock_indices);
            if selected == ids[0] {
                assert!(!scoped.stock_operations.contains(&Some(ids[1])));
            }
            for (vertex, owner) in viewport.mesh_vertices.iter().zip(&scoped.stock_operations) {
                let expected = if *owner == Some(selected) {
                    [0.1, 0.9, 0.2, 1.]
                } else {
                    [0.72, 0.66, 0.22, 1.]
                };
                assert_eq!(vertex.color, expected);
            }
        }
        app.modifiers = iced::keyboard::Modifiers::COMMAND;
        let _ = app.update(Message::ClickOp(ids[0]));
        app.drain_operation_preview();
        let both = app.machining_viewport();
        assert_eq!(both.path_cones.len(), 4);
        for (vertex, owner) in both.mesh_vertices.iter().zip(&outcome.stock_operations) {
            assert_eq!(
                vertex.color == [0.1, 0.9, 0.2, 1.],
                owner.is_some_and(|op| ids.contains(&op))
            );
        }
        app.hidden_ops.extend(ids);
        let hidden = app.machining_viewport();
        assert!(hidden.path_cones.is_empty());
        assert!(hidden
            .mesh_vertices
            .iter()
            .all(|v| v.color == [0.72, 0.66, 0.22, 1.]));
    }

    #[test]
    fn setup_stock_edits_update_the_visible_draft_without_committing() {
        let mut app = picking_app();
        app.show_stock = true;
        app.settings.view.transparent_stock = true;
        let saved = app.controller.inspection_setup().stock;
        let before = app.machining_viewport().transparent_stock_vertices.clone();
        assert!(!before.is_empty());
        let _ = app.update(Message::SetupStockNumberChanged(0, "15".into()));
        let after = app.machining_viewport().transparent_stock_vertices.clone();
        assert_ne!(before.as_ref(), after.as_ref());
        assert_eq!(app.controller.inspection_setup().stock, saved);
        let draft = app.setup_draft.as_ref().unwrap();
        let expected = uncut_stock_mesh_for_setup(&app.controller, draft).0;
        assert_eq!(after.len(), expected.len());
        for (visible, expected) in after.iter().zip(expected.iter()) {
            assert_eq!(visible.position, expected.position);
        }
    }

    #[test]
    fn uncut_stock_follows_rotated_setup_in_world_space() {
        let mut app = picking_app();
        let mut setup = app.controller.inspection_setup().clone();
        setup.stock = cam_model::Stock::RelativeBox {
            offsets: [[5.; 3]; 2],
        };
        setup.frame =
            cam_model::SetupFrame::from_axes(Some([0., 1., 0.]), Some([0., 0., 1.]), None).unwrap();
        app.controller.apply_setup_settings(setup).unwrap();
        let (min, max) = app.controller.stock_box();
        let frame = &app.controller.inspection_setup().frame;
        let (vertices, indices) = uncut_stock_mesh(&app.controller);
        assert_eq!(indices.len(), 36);
        for vertex in vertices {
            let point = frame.to_setup_vector(vertex.position.map(f64::from));
            for axis in 0..3 {
                assert!(
                    (point[axis] - min[axis]).abs() < 1e-4
                        || (point[axis] - max[axis]).abs() < 1e-4
                );
            }
        }
    }

    fn picking_app() -> App {
        let (mut app, _) = App::new();
        app.settings = crate::Settings::default();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample").unwrap();
        app.begin_setup_draft();
        let draft = app.setup_draft.as_mut().unwrap();
        draft.stock = cam_model::Stock::RelativeBox {
            offsets: [[5., 5., 5.], [5., 5., 5.]],
        };
        app.show_project_browser = false;
        app.show_gizmo = false;
        app.settings.snapping.enabled = true;
        app.snaps = vec![SnapKind::End, SnapKind::Mid, SnapKind::Quadrant];
        app
    }
    fn assert_point(a: [f64; 3], b: [f64; 3]) {
        assert!((0..3).all(|i| (a[i] - b[i]).abs() < 1e-5), "{a:?} != {b:?}");
    }
    #[test]
    fn grid_uses_active_work_origin_and_setup_draft() {
        let mut app = picking_app();
        let setup = app.setup_draft.as_mut().unwrap();
        setup.frame = cam_model::SetupFrame::from_xy([0., 1., 0.], [-1., 0., 0.]).unwrap();
        setup.origin = [4., 5., 6.];
        let expected = setup.frame.to_world_vector(setup.origin);
        let viewport = app.machining_viewport();
        assert_eq!(viewport.grid_frame.translation, expected);
        assert_eq!(viewport.grid_frame.rotation, setup_rotation(&app));
        app.setup_draft = None;
        app.controller.add_origin();
        app.controller
            .edit_active_origin(|origin| *origin = [12., 13., 14.]);
        let viewport = app.machining_viewport();
        assert_eq!(viewport.grid_frame.translation, [12., 13., 14.]);
        app.model_view = Some(TreeNode::Models);
        assert_eq!(app.machining_viewport().grid_frame, Default::default());
    }

    fn setup_rotation(app: &App) -> [[f64; 3]; 3] {
        app.setup_draft
            .as_ref()
            .unwrap()
            .frame
            .world_from_setup
            .rotation
    }

    #[test]
    fn box_origin_modes_pick_only_their_bounds_without_snap_preferences_or_fallback() {
        let mut app = picking_app();
        app.settings.snapping.enabled = false;
        app.view.yaw = 0.63;
        app.view.pitch = 0.71;
        app.setup_draft.as_mut().unwrap().frame =
            cam_model::SetupFrame::from_xy([0., 0., 1.], [1., 0., 0.]).unwrap();
        let committed = app.controller.inspection_setup().clone();
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(900., 600.));
        // Build and cache a stock-box viewport, then change the source while
        // picking stays armed. The model box must replace the cached stock box.
        let mut picked = Vec::new();
        for mode in [SetupOriginMode::StockBox, SetupOriginMode::ModelBox] {
            let _ = app.update(Message::SetupOriginModeChanged(mode));
            let _ = app.update(Message::ToggleSetOrigin);
            let draft = app.setup_draft.as_ref().unwrap();
            let (min, max) = app.controller.pending_setup_model_bounds(draft);
            let (min, max) = if mode == SetupOriginMode::StockBox {
                app.controller.pending_setup_stock_bounds(draft)
            } else {
                (min, max)
            };
            let point = draft
                .frame
                .to_world_vector([min[0], (min[1] + max[1]) / 2., max[2]]);
            let viewport = app.machining_viewport();
            let cursor = picking_view::project(viewport.camera(), bounds, point)
                .unwrap()
                .0;
            let hover = viewport.setup_hover_at(cursor, bounds).snap.unwrap();
            assert_point(hover.point, point);
            picked.push(viewport.origin_point_at(cursor, bounds).unwrap());
            assert_point(*picked.last().unwrap(), point);
            assert!(viewport
                .origin_point_at(iced::Point::new(-10000., -10000.), bounds)
                .is_none());
            assert_eq!(app.controller.inspection_setup(), &committed);
        }
        assert_ne!(picked[0], picked[1]);
        let _ = app.update(Message::OriginPointPicked(picked[1]));
        let draft = app.setup_draft.as_ref().unwrap();
        assert_point(draft.frame.to_world_vector(draft.origin), picked[1]);
        let _ = app.update(Message::SetupCancel);
        assert_eq!(app.controller.inspection_setup(), &committed);
    }

    #[test]
    fn stock_origin_hover_and_click_agree_in_rotated_and_portrait_views() {
        use shader::Program;
        let mut app = picking_app();
        let frame = cam_model::SetupFrame::from_xy([0., 0., 1.], [1., 0., 0.]).unwrap();
        app.setup_draft.as_mut().unwrap().frame = frame.clone();
        app.setting_origin = true;
        for (yaw, pitch) in [(0., 0.), (0.63, 0.71)] {
            app.view.yaw = yaw;
            app.view.pitch = pitch;
            for (width, height) in [(900., 600.), (400., 900.)] {
                let bounds =
                    iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(width, height));
                let viewport = app.machining_viewport();
                // Choose the frontmost corner so coincident front/back points in a
                // top view have an unambiguous expected depth.
                let point = viewport
                    .setup_geometry
                    .stock_lines()
                    .flat_map(|(_, p)| p)
                    .min_by(|a, b| {
                        picking_view::project(viewport.camera(), bounds, *a)
                            .unwrap()
                            .1
                            .total_cmp(
                                &picking_view::project(viewport.camera(), bounds, *b)
                                    .unwrap()
                                    .1,
                            )
                    })
                    .unwrap();
                let projected = picking_view::project(viewport.camera(), bounds, point)
                    .unwrap()
                    .0;
                let cursor = iced::Point::new(projected.x + 3., projected.y - 2.);
                let hover = viewport.setup_hover_at(cursor, bounds).snap.unwrap();
                assert!(matches!(hover.source, SetupFeature::Stock(_)));
                assert_point(hover.point, point);
                let action = viewport
                    .update(
                        &mut ViewportState::default(),
                        &iced::Event::Mouse(iced::mouse::Event::ButtonPressed(
                            iced::mouse::Button::Left,
                        )),
                        bounds,
                        iced::mouse::Cursor::Available(cursor),
                    )
                    .unwrap();
                let (message, _, _) = action.into_inner();
                let Some(Message::OriginPointPicked(picked)) = message else {
                    panic!("expected origin pick")
                };
                assert_point(picked, hover.point);
                let _ = app.update(Message::OriginPointPicked(picked));
                assert_point(
                    app.setup_draft.as_ref().unwrap().origin,
                    frame.to_setup_vector(point),
                );
                assert_ne!(app.setup_draft.as_ref().unwrap().origin[2], 0.);
                app.setting_origin = true;
            }
        }
    }
    #[test]
    fn stock_axis_clicks_commit_round_trip_and_undo_without_cad_entities() {
        use shader::Program;
        let mut app = picking_app();
        let committed = app.controller.inspection_setup().clone();
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(900., 600.));
        app.view.yaw = 0.6;
        app.view.pitch = 0.7;
        for axis in 0..2 {
            let _ = app.update(Message::SetupAxisPick(axis));
            let viewport = app.machining_viewport();
            let (_, edge) = viewport
                .setup_geometry
                .stock_lines()
                .find(|(_, p)| (p[1][axis] - p[0][axis]).abs() > 1.)
                .unwrap();
            let midpoint = std::array::from_fn(|i| (edge[0][i] + edge[1][i]) / 2.);
            let cursor = picking_view::project(viewport.camera(), bounds, midpoint)
                .unwrap()
                .0;
            assert!(matches!(
                viewport.setup_hover_at(cursor, bounds).edge,
                Some(SetupFeature::Stock(_))
            ));
            let action = viewport
                .update(
                    &mut ViewportState::default(),
                    &iced::Event::Mouse(iced::mouse::Event::ButtonPressed(
                        iced::mouse::Button::Left,
                    )),
                    bounds,
                    iced::mouse::Cursor::Available(cursor),
                )
                .unwrap();
            let (message, _, _) = action.into_inner();
            assert!(matches!(message, Some(Message::SetupStockEdgePicked(_))));
            let _ = app.update(message.unwrap());
            assert!(app.setup_axis_pick.is_none());
        }
        let draft = app.setup_draft.clone().unwrap();
        assert!(app.setup_frame_issue(&draft).is_none());
        assert!(draft.frame.references.stock_axes[0].is_some());
        assert!(draft.frame.references.stock_axes[1].is_some());
        app.controller.apply_setup_settings(draft.clone()).unwrap();
        let json = serde_json::to_string(app.controller.document()).unwrap();
        let loaded: cam_model::Document = serde_json::from_str(&json).unwrap();
        assert_eq!(loaded.setup().frame, draft.frame);
        assert!(app.controller.undo());
        assert_eq!(app.controller.inspection_setup(), &committed);
        assert!(app.controller.redo());
        assert_eq!(app.controller.inspection_setup().frame, draft.frame);
    }
    #[test]
    fn snap_filters_disable_catching_and_free_origin_uses_the_setup_plane() {
        let mut app = picking_app();
        app.setting_origin = true;
        app.setup_draft.as_mut().unwrap().frame =
            cam_model::SetupFrame::from_xy([0., 0., 1.], [1., 0., 0.]).unwrap();
        app.setup_draft.as_mut().unwrap().heights.top_of_stock = 3.;
        app.view.yaw = 0.5;
        app.view.pitch = 0.6;
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(400., 900.));
        let viewport = app.machining_viewport();
        let (_, edge) = viewport.setup_geometry.stock_lines().next().unwrap();
        let midpoint = std::array::from_fn(|i| (edge[0][i] + edge[1][i]) / 2.);
        let cursor = picking_view::project(viewport.camera(), bounds, midpoint)
            .unwrap()
            .0;
        assert_eq!(
            viewport.setup_hover_at(cursor, bounds).snap.unwrap().kind,
            SnapKind::Mid
        );
        app.snaps = vec![SnapKind::End];
        assert!(app
            .machining_viewport()
            .setup_hover_at(cursor, bounds)
            .snap
            .is_none());
        app.settings.snapping.enabled = false;
        let viewport = app.machining_viewport();
        assert!(viewport.setup_hover_at(cursor, bounds).snap.is_none());
        let point = viewport.origin_point_at(cursor, bounds).unwrap();
        let local = app
            .setup_draft
            .as_ref()
            .unwrap()
            .frame
            .to_setup_vector(point);
        assert!((local[2] - 3.).abs() < 1e-5);
    }
    #[cfg(feature = "occt")]
    #[test]
    fn native_cad_vertices_snap_in_3d_and_hidden_instances_do_not_contribute() {
        let mut app = picking_app();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .import_cad(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../cam-kernel/fixtures/box-mm.step"),
            )
            .unwrap();
        let model = app.controller.geometry_state().instances[0].id;
        app.controller
            .edit_setup(|setup| setup.model_instances = vec![model]);
        app.begin_setup_draft();
        app.setup_draft.as_mut().unwrap().stock = cam_model::Stock::RelativeBox {
            offsets: [[5.; 3], [5.; 3]],
        };
        app.setting_origin = true;
        app.view.yaw = 0.6;
        app.view.pitch = 0.7;
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(900., 600.));
        let viewport = app.machining_viewport();
        let instance = &app.controller.geometry_state().instances[0];
        let asset = app
            .controller
            .geometry_state()
            .asset(instance.asset_id)
            .unwrap();
        let point = asset
            .topology
            .vertices
            .iter()
            .map(|v| instance.placement.transform_point(v.point))
            .min_by(|a, b| {
                picking_view::project(viewport.camera(), bounds, *a)
                    .unwrap()
                    .1
                    .total_cmp(
                        &picking_view::project(viewport.camera(), bounds, *b)
                            .unwrap()
                            .1,
                    )
            })
            .unwrap();
        let cursor = picking_view::project(viewport.camera(), bounds, point)
            .unwrap()
            .0;
        let hit = viewport.setup_hover_at(cursor, bounds).snap.unwrap();
        assert!(matches!(hit.source, SetupFeature::Model(_)));
        assert_point(hit.point, point);
        app.cad_hidden.insert(instance.id);
        assert!(app
            .machining_viewport()
            .setup_hover_at(cursor, bounds)
            .snap
            .is_none());
    }
    #[test]
    fn library_sidebar_resize_and_collapse_preserve_width() {
        let (mut app, _) = App::new();
        app.switch_workspace(Workspace::Tools);
        assert!(!app.settings.tool_table.library_sidebar_collapsed);
        let _ = app.update(Message::WindowCursor(iced::Point::new(210.0, 30.0)));
        let _ = app.update(Message::LibrarySidebarResizeStart);
        let _ = app.update(Message::WindowCursor(iced::Point::new(310.0, 30.0)));
        assert_eq!(app.settings.tool_table.library_sidebar_width, 310.0);
        let _ = app.update(Message::ToolColumnRelease);
        let _ = app.update(Message::WindowCursor(iced::Point::new(700.0, 30.0)));
        assert_eq!(app.settings.tool_table.library_sidebar_width, 310.0);
        let _ = app.update(Message::ToggleLibrarySidebar);
        assert!(app.settings.tool_table.library_sidebar_collapsed);
        let _ = app.update(Message::ToggleLibrarySidebar);
        assert!(!app.settings.tool_table.library_sidebar_collapsed);
        assert_eq!(app.settings.tool_table.library_sidebar_width, 310.0);
        let _ = app.update(Message::LibrarySidebarResizeStart);
        let _ = app.update(Message::WindowCursor(iced::Point::new(-1000.0, 30.0)));
        assert_eq!(app.settings.tool_table.library_sidebar_width, 180.0);
    }

    #[test]
    fn default_workspace_and_clean_round_trip_preserve_machining_context() {
        let (mut app, _) = App::new();
        assert_eq!(app.workspace, Workspace::Machining);
        assert!(app.show_project_browser);
        assert!(
            app.pane_handle(Pane::Project).is_none(),
            "browser overlays the main view"
        );
        app.controller.open_dxf(SAMPLE_DXF, "sample").unwrap();
        app.setup_draft = None;
        app.refresh_fields();
        app.view.zoom = 1.4;
        let before = app.view;
        let selection = app.controller.selection();
        let setup = app.controller.active_setup_id();
        app.switch_workspace(Workspace::Tools);
        assert!(app.library_mode());
        app.switch_workspace(Workspace::Machining);
        assert_eq!(app.controller.selection(), selection);
        assert_eq!(app.controller.active_setup_id(), setup);
        assert_eq!(app.view.zoom, before.zoom);
        assert_eq!(app.view.pan, before.pan);
    }
    #[test]
    fn dirty_tool_is_preserved_while_browsing_and_blocks_workspace_navigation() {
        let (mut app, _) = App::new();
        app.library = ToolLibrary {
            tools: vec![
                crate::tool_library::default_tool(1, ToolKind::EndMill),
                crate::tool_library::default_tool(2, ToolKind::BallMill),
            ],
            ..Default::default()
        };
        app.switch_workspace(Workspace::Tools);
        let original_index = app.lib_sel;
        let original = app.library.tools[original_index].clone();
        app.tool_edit.as_mut().unwrap().length += 1.;
        app.switch_workspace(Workspace::Machining);
        assert_eq!(app.workspace, Workspace::Tools);
        let other = (original_index + 1) % app.library.tools.len();
        let _ = app.update(Message::SelectLibraryTool(other));
        assert_eq!(app.lib_sel, other);
        assert!(app.tool_drafts.contains_key(&original_index));
        app.switch_workspace(Workspace::Machining);
        assert_eq!(app.workspace, Workspace::Tools);
        let _ = app.update(Message::SelectLibraryTool(original_index));
        assert_eq!(app.tool_edit.clone().unwrap().length, original.length + 1.);
        let _ = app.update(Message::CancelToolDraft);
        assert_eq!(app.tool_edit.clone().unwrap(), original);
        app.switch_workspace(Workspace::Machining);
        assert_eq!(app.workspace, Workspace::Machining);
    }
    #[test]
    fn browser_blocks_viewport_click_drag_wheel_and_releases_armed_picks() {
        use iced::mouse::{Button, Cursor, Event as Mouse, ScrollDelta};
        use shader::Program;
        let controller = AppController::new(crate::default_machine());
        let mut viewport = Viewport::new(
            &controller,
            None,
            None,
            EntityKind::Face,
            None,
            &BTreeSet::new(),
            false,
            true,
            ViewControls::default(),
            true,
            80.,
            &crate::Settings::default(),
            &[],
            &[],
            None,
            None,
            false,
            true,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        );
        viewport.browser_overlay_width = 250.;
        let bounds = iced::Rectangle {
            x: 40.,
            y: 60.,
            width: 800.,
            height: 600.,
        };
        let point = iced::Point::new(100., 620.); // blank lower browser area
        for event in [
            Mouse::ButtonPressed(Button::Left),
            Mouse::ButtonReleased(Button::Left),
            Mouse::CursorMoved { position: point },
            Mouse::WheelScrolled {
                delta: ScrollDelta::Lines { x: 0., y: 1. },
            },
        ] {
            let mut state = ViewportState {
                drag: Some(DragMode::Orbit),
                last: Some(point),
                press: Some(point),
                gizmo_arm: Some((1., 1.)),
                cad_arm: Some(vec![]),
                planar_arm: Some(([0., 0.], 1.)),
            };
            let action = viewport
                .update(
                    &mut state,
                    &iced::Event::Mouse(event),
                    bounds,
                    Cursor::Available(point),
                )
                .unwrap();
            let (message, _, status) = action.into_inner();
            assert!(
                message.is_none(),
                "overlay must not publish picks or camera movement"
            );
            assert_eq!(status, iced::event::Status::Captured);
            assert!(
                state.drag.is_none()
                    && state.cad_arm.is_none()
                    && state.planar_arm.is_none()
                    && state.gizmo_arm.is_none()
            );
        }
        assert!(
            !browser_contains(point, bounds, 0.),
            "hidden browser cannot intercept input"
        );
        assert!(!browser_contains(
            iced::Point::new(400., 200.),
            bounds,
            250.
        ));
    }
    #[test]
    fn renumber_and_tool_enum_changes_cannot_discard_invalid_buffers() {
        let (mut app, _) = App::new();
        app.switch_workspace(Workspace::Tools);
        let original = app.library.clone();
        let _ = app.update(Message::FieldChanged(Field::ToolLength, "invalid".into()));
        let _ = app.update(Message::ToolCuttingDirChanged(CutDir::Up));
        assert_eq!(app.fields[&Field::ToolLength], "invalid");
        assert!(app.any_field_invalid());
        let _ = app.update(Message::RenumberLibrary);
        let _ = app.update(Message::RenumberConfirmed(true));
        assert_eq!(app.library.tools, original.tools);
        assert_eq!(app.fields[&Field::ToolLength], "invalid");
        let _ = app.update(Message::CancelToolDraft);
        assert!(!app.inspector_dirty());
    }
    #[test]
    fn creating_setup_leaves_passive_cad_inspection_for_the_setup_editor() {
        let (mut app, _) = App::new();
        app.cad_inspecting = true;
        let _ = app.update(Message::SetupNew);
        assert!(!app.cad_inspecting);
        assert_eq!(app.controller.selection(), Selection::Setup);
        assert!(app.setup_draft.is_some());
    }
    #[test]
    fn first_motion_past_click_slop_orbits_without_a_second_move_event() {
        use shader::Program;
        let controller = AppController::new(crate::default_machine());
        let viewport = Viewport::new(
            &controller,
            None,
            None,
            EntityKind::Face,
            None,
            &BTreeSet::new(),
            false,
            true,
            ViewControls::default(),
            true,
            80.,
            &crate::Settings::default(),
            &[],
            &[],
            None,
            None,
            false,
            true,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        );
        let bounds = iced::Rectangle {
            x: 0.,
            y: 0.,
            width: 800.,
            height: 600.,
        };
        let start = iced::Point::new(400., 300.);
        let end = iced::Point::new(440., 330.);
        for arm in 0..3 {
            let mut state = ViewportState {
                drag: Some(DragMode::Orbit),
                last: Some(start),
                press: Some(start),
                ..Default::default()
            };
            match arm {
                0 => state.planar_arm = Some(([0., 0.], 1.)),
                1 => state.cad_arm = Some(vec![]),
                _ => state.gizmo_arm = Some((0., 0.)),
            }
            let action = viewport
                .update(
                    &mut state,
                    &iced::Event::Mouse(iced::mouse::Event::CursorMoved { position: end }),
                    bounds,
                    iced::mouse::Cursor::Available(end),
                )
                .unwrap();
            let (message, _, status) = action.into_inner();
            assert!(matches!(message, Some(Message::OrbitBy(x, y)) if x > 0. && y > 0.));
            assert_eq!(status, iced::event::Status::Captured);
            assert!(
                state.planar_arm.is_none() && state.cad_arm.is_none() && state.gizmo_arm.is_none()
            );
        }
    }
    #[cfg(feature = "occt")]
    #[test]
    fn native_step_face_click_selects_profile_boundary_in_the_operation_viewport() {
        use shader::Program;
        let mut controller = AppController::new(crate::default_machine());
        controller
            .import_cad(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../cam-kernel/fixtures/box-mm.step"),
            )
            .unwrap();
        let models = controller
            .geometry_state()
            .instances
            .iter()
            .map(|instance| instance.id)
            .collect();
        controller.edit_setup(|setup| setup.model_instances = models);
        let mut viewport = Viewport::new(
            &controller,
            None,
            None,
            EntityKind::Loop,
            None,
            &BTreeSet::new(),
            false,
            true,
            ViewControls::default(),
            true,
            80.,
            &crate::Settings::default(),
            &[],
            &[],
            None,
            None,
            false,
            true,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        );
        viewport.cad_operation_kind = Some(OpKind::Profile);
        let body = &controller.geometry_state().assets[0].topology.bodies[0];
        let world = [
            (body.bounds[0][0] + body.bounds[1][0]) as f32 * 0.5,
            (body.bounds[0][1] + body.bounds[1][1]) as f32 * 0.5,
            body.bounds[1][2] as f32,
        ];
        let bounds = iced::Rectangle {
            x: 0.,
            y: 0.,
            width: 1000.,
            height: 700.,
        };
        let matrix = viewport.camera().view_proj(bounds.width / bounds.height);
        let clip: [f32; 4] = std::array::from_fn(|row| {
            matrix[0][row] * world[0]
                + matrix[1][row] * world[1]
                + matrix[2][row] * world[2]
                + matrix[3][row]
        });
        let point = iced::Point::new(
            (clip[0] / clip[3] + 1.) * bounds.width / 2.,
            (1. - clip[1] / clip[3]) * bounds.height / 2.,
        );
        let mut state = ViewportState::default();
        let _ = viewport.update(
            &mut state,
            &iced::Event::Mouse(iced::mouse::Event::ButtonPressed(iced::mouse::Button::Left)),
            bounds,
            iced::mouse::Cursor::Available(point),
        );
        let action = viewport
            .update(
                &mut state,
                &iced::Event::Mouse(iced::mouse::Event::ButtonReleased(
                    iced::mouse::Button::Left,
                )),
                bounds,
                iced::mouse::Cursor::Available(point),
            )
            .unwrap();
        let (message, _, _) = action.into_inner();
        assert!(
            matches!(message, Some(Message::CadPicked(ref candidates)) if !candidates.is_empty() && candidates[0].entity.kind == EntityKind::Loop)
        );
        // Ctrl takes precedence over the operation/filter. The actual click
        // message retains the body-instance reference and replaces face picks.
        viewport.cad_silhouette = true;
        let candidates = viewport.cad_candidates_at(point, bounds);
        assert_eq!(candidates[0].entity.kind, EntityKind::Silhouette);
        let selected = candidates[0].entity.clone();
        let (mut app, _) = App::new();
        app.controller = controller;
        app.modifiers = iced::keyboard::Modifiers::CTRL;
        let _ = app.update(Message::CadPicked(candidates.clone()));
        assert_eq!(
            app.controller.cad_selection(),
            std::slice::from_ref(&selected)
        );
        let bottom = app
            .controller
            .document()
            .height_selection_bounds(std::slice::from_ref(&selected))
            .unwrap()
            .0[2];
        assert_eq!(bottom, -10.);
        let viewport = app.machining_viewport();
        let highlights: Vec<_> = viewport
            .vertices
            .iter()
            .filter(|vertex| vertex.color == PICK_BOUNDARY)
            .collect();
        assert!(
            !highlights.is_empty(),
            "the silhouette remains visible through the stock"
        );
        assert!(
            highlights
                .iter()
                .all(|vertex| (f64::from(vertex.position[2]) - bottom).abs() < 1e-6),
            "inspection silhouettes are projected onto the model bottom"
        );
        let _ = app.update(Message::CadPicked(candidates.clone()));
        assert_eq!(app.controller.cad_selection(), &[selected]);
        app.modifiers = iced::keyboard::Modifiers::SHIFT;
        let _ = app.update(Message::CadPicked(candidates));
        assert!(app.controller.cad_selection().is_empty());
    }
}

#[cfg(test)]
mod performance_tests {
    use super::*;
    use std::{hint::black_box, time::Instant};

    /// Isolate the rotating-stock display transform, excluding removal and GPU work.
    #[test]
    #[ignore = "large display-transform benchmark; run with --ignored --nocapture"]
    fn rotating_stock_transform_benchmark() {
        let frame = cam_model::SetupFrame::from_xy([1., 2., 3.], [-2., 1., 0.]).unwrap();
        let axis = cam_cldata::RotaryAxis::A;
        let center = cam_cldata::Point3::new(3., 4., 5.);
        let original: Vec<_> = (0..771_068)
            .map(|i| MeshVertex {
                position: [
                    (i % 1024) as f32 * 0.03,
                    (i / 1024) as f32 * 0.03,
                    (i as f32 * 0.01).sin(),
                ],
                normal: [0.6, 0.8, 0.],
                color: [0.1, 0.2, 0.3, 1.],
            })
            .collect();
        let (mut scalar_seconds, mut bulk_seconds) = (0., 0.);
        for angle in [37., 90., 173.] {
            let mut reference = original.clone();
            let start = Instant::now();
            for vertex in &mut reference {
                vertex.position =
                    stock_to_machine_view(&frame, axis, center, angle, vertex.position, false);
                vertex.normal =
                    stock_to_machine_view(&frame, axis, center, angle, vertex.normal, true);
            }
            scalar_seconds += start.elapsed().as_secs_f64();
            black_box(&reference);
            let mut optimized = original.clone();
            let start = Instant::now();
            let transform = cam_render::RotaryStockTransform::new(&frame, axis, center, angle);
            transform.apply_mesh(&mut optimized);
            bulk_seconds += start.elapsed().as_secs_f64();
            black_box(&optimized);
            for (a, b) in reference.iter().zip(&optimized) {
                for i in 0..3 {
                    assert!((a.position[i] - b.position[i]).abs() < 1e-5);
                    assert!((a.normal[i] - b.normal[i]).abs() < 1e-6);
                }
                assert_eq!(a.color, b.color);
            }
        }
        println!("rotating stock, {} vertices: scalar {:.3} ms/frame; bulk {:.3} ms/frame; {:.1}x transform speedup",
            original.len(), scalar_seconds / 3. * 1000., bulk_seconds / 3. * 1000., scalar_seconds / bulk_seconds);
    }

    fn cad_app() -> App {
        let (mut app, _) = App::new();
        app.settings = crate::Settings::default();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_project(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../testfiles/test_project.ocam"
            ))
            .unwrap();
        app
    }

    #[test]
    fn transparent_stock_reveals_the_model_and_opaque_stock_keeps_reference_edges() {
        let (mut app, _) = App::new();
        app.settings = crate::Settings::default();
        app.controller = AppController::new(crate::default_machine());
        let path =
            std::env::temp_dir().join(format!("rotary-stock-view-{}.ocam", std::process::id()));
        std::fs::write(&path, include_str!("../tests/fixtures/stepped_rotary.ocam")).unwrap();
        app.controller.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        app.controller.select(Selection::Operation(0));
        app.model_view = None;
        app.setup_draft = None;
        app.show_stock = false;
        let model = app.machining_viewport();
        assert!(!model.mesh_indices.is_empty());
        assert!(!model.cad_edges.is_empty());
        let (vertices, indices) = uncut_stock_mesh(&app.controller);
        assert!(!indices.is_empty());
        app.show_stock = true;
        app.settings.view.transparent_stock = false;
        let stock = app.machining_viewport();
        assert_eq!(stock.mesh_vertices.len(), vertices.len());
        assert_eq!(*stock.mesh_indices, indices);
        assert_eq!(stock.cad_edges, model.cad_edges);
        app.settings.view.transparent_stock = true;
        let transparent = app.machining_viewport();
        assert_eq!(transparent.mesh_indices, model.mesh_indices);
        assert_eq!(transparent.mesh_vertices, model.mesh_vertices);
        assert_eq!(*transparent.transparent_stock_indices, indices);
        assert!(transparent.transparent_stock_convex);
        app.show_stock = false;
        assert_eq!(app.machining_viewport().mesh_indices, model.mesh_indices);
        app.show_stock = true;
        app.controller.edit_stock(|stock| {
            if let cam_model::Stock::RelativeRound {
                tube,
                inner_diameter,
                ..
            } = stock
            {
                *tube = true;
                *inner_diameter = 10.;
            }
        });
        assert!(
            !app.machining_viewport().transparent_stock_convex,
            "hollow stock is not convex"
        );
    }

    #[test]
    fn simulation_reuses_unchanged_frames_and_refreshes_after_seek_and_options() {
        let (mut app, _) = App::new();
        app.settings = crate::Settings::default();
        app.model_view = None;
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "playback.dxf").unwrap();
        app.controller.new_operation(OpKind::Profile);
        app.controller.start_run();
        let deadline = Instant::now() + std::time::Duration::from_secs(15);
        while app.controller.run_needs_poll() {
            app.controller.poll_run();
            assert!(Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        let mut player = SimulationPlayer::new_for_operation(&app.controller, None).unwrap();
        while player.needs_tick() {
            player.tick();
            assert!(Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        app.simulation = Some(player);
        let initial = app.machining_viewport();
        assert!(!initial.mesh_vertices.is_empty());
        let _ = app.update(Message::OrbitBy(0.3, 0.2));
        app.simulation.as_mut().unwrap().tick();
        let unchanged = app.machining_viewport();
        assert!(Arc::ptr_eq(
            &initial.mesh_vertices,
            &unchanged.mesh_vertices
        ));
        assert!(Arc::ptr_eq(&initial.vertices, &unchanged.vertices));
        assert!(Arc::ptr_eq(
            &initial.setup_geometry,
            &unchanged.setup_geometry
        ));
        assert_ne!(
            initial.camera().view_proj(1.),
            unchanged.camera().view_proj(1.)
        );

        let player = app.simulation.as_mut().unwrap();
        let mut prefs = player.options.clone();
        prefs.show_tool = false;
        player.configure(&prefs, [0.2, 0.6, 0.3]);
        let hidden_tool = app.machining_viewport();
        assert!(!Arc::ptr_eq(
            &unchanged.mesh_vertices,
            &hidden_tool.mesh_vertices
        ));
        assert!(hidden_tool.mesh_vertices.len() < unchanged.mesh_vertices.len());
        let player = app.simulation.as_mut().unwrap();
        player.seek_end(true);
        while player.needs_tick() {
            player.tick();
            assert!(Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        let advanced = app.machining_viewport();
        assert!(!Arc::ptr_eq(
            &hidden_tool.mesh_vertices,
            &advanced.mesh_vertices
        ));
        assert!(Arc::ptr_eq(
            &advanced.mesh_vertices,
            &app.machining_viewport().mesh_vertices
        ));
        app.simulation = None;
        assert!(!Arc::ptr_eq(
            &advanced.mesh_vertices,
            &app.machining_viewport().mesh_vertices
        ));
    }

    #[test]
    fn camera_and_menu_events_reuse_geometry_but_hover_and_visibility_refresh_it() {
        if !crate::test_fixture::available("test_project.ocam") {
            return;
        }
        let mut app = cad_app();
        let initial = app.machining_viewport();
        assert!(!initial.mesh_indices.is_empty());
        let _ = app.update(Message::WindowCursor(iced::Point::new(20., 20.)));
        let _ = app.update(Message::ToggleRibbonGroup(0));
        let _ = app.update(Message::OrbitBy(0.3, 0.2));
        let moved = app.machining_viewport();
        assert!(Arc::ptr_eq(&initial.mesh_vertices, &moved.mesh_vertices));
        assert!(Arc::ptr_eq(&initial.setup_geometry, &moved.setup_geometry));
        assert_ne!(initial.camera().view_proj(1.), moved.camera().view_proj(1.));
        let instance = &app.controller.geometry_state().instances[0];
        let asset = &app.controller.geometry_state().assets[0];
        let reference = EntityRef {
            asset_id: asset.id,
            revision: asset.revision,
            instance_id: instance.id,
            body_id: instance.body_id,
            kind: EntityKind::Face,
            id: asset.topology.faces[0].id,
        };
        let instance_id = instance.id;
        let _ = app.update(Message::CadHover(Some(reference)));
        let hovered = app.machining_viewport();
        assert!(Arc::ptr_eq(&moved.cad_scene, &hovered.cad_scene));
        assert_ne!(moved.mesh_vertices, hovered.mesh_vertices);
        let _ = app.update(Message::CadHover(app.cad_hover.clone()));
        assert!(Arc::ptr_eq(
            &hovered.mesh_vertices,
            &app.machining_viewport().mesh_vertices
        ));
        app.cad_hidden.insert(instance_id);
        let hidden = app.machining_viewport();
        assert!(hidden.mesh_indices.is_empty());
        assert!(hidden.cad_scene.bounds().is_none());
        app.cad_hidden.clear();
        assert_eq!(app.machining_viewport().mesh_indices, initial.mesh_indices);
        app.controller.edit_setup(|setup| {
            setup.frame = cam_model::SetupFrame::from_xy([0., 1., 0.], [-1., 0., 0.]).unwrap()
        });
        assert!(!Arc::ptr_eq(
            &hovered.cad_scene,
            &app.machining_viewport().cad_scene
        ));
    }

    #[test]
    fn simulation_draws_cad_without_selected_or_hovered_features_and_preserves_selection() {
        if !crate::test_fixture::available("test_project.ocam") {
            return;
        }
        let mut app = cad_app();
        let geometry = app.controller.geometry_state();
        let instance = &geometry.instances[0];
        let asset = geometry.asset(instance.asset_id).unwrap();
        let face = EntityRef {
            asset_id: asset.id,
            revision: asset.revision,
            instance_id: instance.id,
            body_id: instance.body_id,
            kind: EntityKind::Face,
            id: asset.topology.faces[0].id,
        };
        let focus: Vec<_> = geometry.bindings.keys().copied().collect();
        assert!(!focus.is_empty(), "exercise operation-bound features too");
        let build = |controller: &AppController,
                     preview: Option<&crate::RunOutcome>,
                     hover: Option<&EntityRef>,
                     focus: &[u32],
                     show_stock: bool| {
            Viewport::new(
                controller,
                preview,
                None,
                EntityKind::Face,
                hover,
                &BTreeSet::new(),
                false,
                show_stock,
                ViewControls::default(),
                false,
                110.,
                &crate::Settings::default(),
                focus,
                &[],
                None,
                None,
                false,
                false,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
            )
        };
        let neutral = build(&app.controller, None, None, &[], false);
        app.controller.set_cad_selection(vec![face.clone()]);
        let selected = build(&app.controller, None, Some(&face), &focus, false);
        assert_ne!(selected.mesh_vertices, neutral.mesh_vertices);
        let preview = crate::RunOutcome::default();
        let simulated = build(&app.controller, Some(&preview), Some(&face), &focus, false);
        assert_eq!(simulated.mesh_vertices, neutral.mesh_vertices);
        assert_eq!(simulated.cad_edges, neutral.cad_edges);
        assert!(simulated.vertices.iter().all(|v| v.width <= 1.));
        let stock = crate::RunOutcome {
            stock_vertices: vec![
                MeshVertex {
                    position: [0.; 3],
                    normal: [0., 0., 1.],
                    color: [1., 0., 0., 1.]
                };
                3
            ],
            stock_indices: vec![0, 1, 2],
            ..Default::default()
        };
        let with_stock = build(&app.controller, Some(&stock), None, &[], true);
        assert_eq!(
            *with_stock.mesh_vertices, stock.stock_vertices,
            "nominal CAD faces must not hide thread cut faces"
        );
        assert_eq!(
            with_stock.cad_edges, neutral.cad_edges,
            "retain model reference edges"
        );

        assert_eq!(app.controller.cad_selection(), std::slice::from_ref(&face));
        let restored = build(&app.controller, None, Some(&face), &focus, false);
        assert_eq!(restored.mesh_vertices, selected.mesh_vertices);
        assert_eq!(restored.vertices, selected.vertices);
    }

    #[test]
    fn cached_editor_and_stock_follow_uncommitted_edits() {
        if !crate::test_fixture::available("test_project.ocam") {
            return;
        }
        let mut app = cad_app();
        app.model_view = None;
        app.controller.select(Selection::Setup);
        app.show_stock = true;
        app.settings.view.transparent_stock = false;
        let opaque = app.machining_viewport();
        app.settings.view.transparent_stock = true;
        let transparent = app.machining_viewport();
        assert!(!transparent.transparent_stock_indices.is_empty());
        let (cad_vertices, cad_indices) = transparent.cad_scene.mesh(&[], None);
        assert!(!cad_indices.is_empty());
        assert_eq!(*transparent.mesh_indices, cad_indices);
        assert_eq!(transparent.mesh_vertices.len(), cad_vertices.len());
        assert!(!opaque.mesh_indices.is_empty());
        let id = app
            .controller
            .inspection_setup()
            .operations
            .iter()
            .find(|op| matches!(op, Operation::Profile(_)))
            .unwrap()
            .id();
        app.begin_operation_draft(id);
        let _ = app.update(Message::ProfileTabsEnabled(true));
        let _ = app.update(Message::FieldChanged(Field::TabWidth, "3".into()));
        let draft = app.machining_viewport();
        let _ = app.update(Message::OrbitBy(0.1, 0.1));
        assert!(Arc::ptr_eq(
            &draft.mesh_vertices,
            &app.machining_viewport().mesh_vertices
        ));
        let _ = app.update(Message::FieldChanged(Field::TabWidth, "9".into()));
        let edited = app.machining_viewport();
        assert_ne!(draft.mesh_vertices, edited.mesh_vertices);
        assert!(
            matches!(app.controller.operation(id), Some(Operation::Profile(op)) if !op.tabs.enabled)
        );
        assert!(Arc::ptr_eq(&draft.cad_scene, &edited.cad_scene));
    }

    /// Exercise actual wgpu validation and repeated uploads on a local GPU.
    #[test]
    #[ignore = "requires a graphics adapter"]
    fn viewport_gpu_smoke() {
        use shader::{Pipeline, Primitive, Program};
        let instance = wgpu::Instance::default();
        let adapter =
            iced::futures::executor::block_on(instance.request_adapter(&Default::default()))
                .unwrap();
        let (device, queue) =
            iced::futures::executor::block_on(adapter.request_device(&Default::default())).unwrap();
        let mut pipeline = ViewportPipeline::new(&device, &queue, wgpu::TextureFormat::Rgba8Unorm);
        let target = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("viewport smoke test"),
            size: wgpu::Extent3d {
                width: 1000,
                height: 700,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let target = target.create_view(&Default::default());
        let bounds = iced::Rectangle::with_size(iced::Size::new(1000., 700.));
        let viewport = shader::Viewport::with_physical_size(iced::Size::new(1000, 700), 1.);
        let mut app = cad_app();
        app.model_view = None;
        app.show_stock = true;
        let geometry = app.controller.geometry_state();
        let instance = &geometry.instances[0];
        app.cad_hover = Some(EntityRef {
            asset_id: instance.asset_id,
            revision: geometry.assets[0].revision,
            instance_id: instance.id,
            body_id: instance.body_id,
            kind: EntityKind::Body,
            id: instance.body_id,
        });
        for (index, action) in [
            Message::WindowCursor(iced::Point::ORIGIN),
            Message::OrbitBy(0.3, 0.2),
            Message::WindowCursor(iced::Point::ORIGIN),
            Message::OrbitBy(0.2, 0.1),
        ]
        .into_iter()
        .enumerate()
        {
            // Change only in-memory preferences; the smoke test must not save settings.
            app.settings.view.transparent_stock = index >= 2;
            let _ = app.update(action);
            let scene = app.machining_viewport();
            assert!(scene.cad_edges.iter().any(|vertex| vertex.width == 3.));
            let primitive = scene.draw(
                &Default::default(),
                iced::mouse::Cursor::Unavailable,
                bounds,
            );
            primitive.prepare(&mut pipeline, &device, &queue, &bounds, &viewport);
            // A second preparation of the same snapshot must keep all meshes usable.
            primitive.prepare(&mut pipeline, &device, &queue, &bounds, &viewport);
            let mut encoder = device.create_command_encoder(&Default::default());
            primitive.render(
                &pipeline,
                &mut encoder,
                &target,
                &iced::Rectangle {
                    x: 0,
                    y: 0,
                    width: 1000,
                    height: 700,
                },
            );
            queue.submit([encoder.finish()]);
        }
        device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
    }

    /// Read actual rendered pixels so a world-fixed light cannot pass unnoticed.
    #[test]
    #[ignore = "requires a graphics adapter"]
    fn viewport_lighting_stays_constant_when_rotating() {
        let instance = wgpu::Instance::default();
        let adapter =
            iced::futures::executor::block_on(instance.request_adapter(&Default::default()))
                .unwrap();
        let (device, queue) =
            iced::futures::executor::block_on(adapter.request_device(&Default::default())).unwrap();
        let texture = |label, format, usage| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: wgpu::Extent3d {
                    width: 64,
                    height: 64,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage,
                view_formats: &[],
            })
        };
        let target = texture(
            "lighting regression",
            wgpu::TextureFormat::Rgba8Unorm,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        );
        let depth = texture(
            "lighting regression depth",
            cam_render::DEPTH_FORMAT,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        );
        let target_view = target.create_view(&Default::default());
        let depth_view = depth.create_view(&Default::default());
        let readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("lighting regression pixels"),
            size: 64 * 256,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let (cube, indices) = cam_render::unit_cube();
        let vertices: Vec<_> = cube
            .iter()
            .map(|v| MeshVertex {
                position: v.position,
                normal: v.normal,
                color: [0.7, 0.7, 0.7, 1.0],
            })
            .collect();
        let mut renderer = cam_render::MeshRenderer::new(&device, wgpu::TextureFormat::Rgba8Unorm);
        let mut reference: Option<u8> = None;
        // The first quad is the +X face. Drawing it alone also checks the
        // back of an open shell, which must remain illuminated.
        for shell in [false, true] {
            renderer.upload(
                &device,
                &vertices,
                if shell { &indices[..6] } else { &indices },
            );
            for normal in [
                [1., 0., 0.],
                [-1., 0., 0.],
                [0., 1., 0.],
                [0., -1., 0.],
                [0., 0., 1.],
                [0., 0., -1.],
            ] {
                if shell && normal[0] == 0. {
                    continue;
                }
                for (aspect, zoom) in [(0.5, -0.4), (1., 0.), (2., 0.7)] {
                    let (yaw, pitch) = face_view(normal);
                    let mut camera = OrbitCamera::framed([-1.; 3], [1.; 3]);
                    camera.orient = cam_render::orientation(yaw, pitch);
                    camera.zoom = zoom;
                    camera.pan = [0.1, -0.1, 0.1];
                    renderer.set_camera(&queue, camera.view_proj(aspect), camera.orient);
                    let mut encoder = device.create_command_encoder(&Default::default());
                    {
                        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &target_view,
                                depth_slice: None,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                    store: wgpu::StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: Some(
                                wgpu::RenderPassDepthStencilAttachment {
                                    view: &depth_view,
                                    depth_ops: Some(wgpu::Operations {
                                        load: wgpu::LoadOp::Clear(1.),
                                        store: wgpu::StoreOp::Store,
                                    }),
                                    stencil_ops: None,
                                },
                            ),
                            ..Default::default()
                        });
                        renderer.draw(&mut pass);
                    }
                    encoder.copy_texture_to_buffer(
                        target.as_image_copy(),
                        wgpu::TexelCopyBufferInfo {
                            buffer: &readback,
                            layout: wgpu::TexelCopyBufferLayout {
                                offset: 0,
                                bytes_per_row: Some(256),
                                rows_per_image: Some(64),
                            },
                        },
                        target.size(),
                    );
                    queue.submit([encoder.finish()]);
                    let (tx, rx) = std::sync::mpsc::channel();
                    readback
                        .slice(..)
                        .map_async(wgpu::MapMode::Read, move |result| {
                            tx.send(result).unwrap();
                        });
                    device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
                    rx.recv().unwrap().unwrap();
                    let pixels = readback.slice(..).get_mapped_range();
                    let brightness = pixels[32 * 256 + 32 * 4];
                    assert!(brightness > 140, "visible face is too dark: {brightness}");
                    let expected = *reference.get_or_insert(brightness);
                    assert!(
                        brightness.abs_diff(expected) <= 2,
                        "illumination changed: {brightness} vs {expected}, normal={normal:?}, \
                         aspect={aspect}, zoom={zoom}, shell={shell}"
                    );
                    drop(pixels);
                    readback.unmap();
                }
            }
        }
    }

    /// Reproduce menu pointer traffic and camera motion against the bundled CAD job.
    /// Run manually with `cargo test -p cam-app --features gui viewport_interaction_benchmark -- --ignored --nocapture`.
    #[test]
    #[ignore = "manual performance measurement"]
    fn viewport_interaction_benchmark() {
        let mut app = cad_app();
        black_box(app.machining_viewport());
        for (label, orbit) in [("menu pointer", false), ("orbit", true)] {
            let start = Instant::now();
            for i in 0..60 {
                let message = if orbit {
                    Message::OrbitBy(0.01, 0.005)
                } else {
                    Message::WindowCursor(iced::Point::new(i as f32, 20.))
                };
                let _ = app.update(message);
                let viewport = app.machining_viewport();
                black_box(shader::Program::draw(
                    &viewport,
                    &Default::default(),
                    iced::mouse::Cursor::Unavailable,
                    iced::Rectangle::with_size(iced::Size::new(1000., 700.)),
                ));
            }
            eprintln!(
                "{label}: {:.3} ms/update+draw",
                start.elapsed().as_secs_f64() * 1000. / 60.
            );
        }
        for editing in [false, true] {
            if editing {
                let id = app.controller.inspection_setup().operations[0].id();
                app.begin_operation_draft(id);
            }
            black_box(app.view());
            let start = Instant::now();
            for _ in 0..60 {
                let _ = app.update(Message::OrbitBy(0.01, 0.005));
                black_box(app.view());
            }
            eprintln!(
                "full UI construction (editing={editing}): {:.3} ms/update+view",
                start.elapsed().as_secs_f64() * 1000. / 60.
            );
        }
    }

    /// Profile a supplied saved project without changing it. Override the input
    /// with OPENCAM_BENCH_PROJECT; run with --ignored --nocapture.
    #[test]
    #[ignore = "manual supplied-project performance measurement"]
    fn supplied_project_interaction_benchmark() {
        let (mut app, _) = App::new();
        app.settings = crate::Settings::default();
        app.controller = AppController::new(crate::default_machine());
        let path = std::env::var_os("OPENCAM_BENCH_PROJECT")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| {
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testfiles/test2.ocam")
            });
        macro_rules! measure {
            ($label:expr, $n:expr, $body:expr) => {{
                let start = Instant::now();
                for _ in 0..$n {
                    black_box($body);
                }
                eprintln!(
                    "{}: {:.3} ms/call",
                    $label,
                    start.elapsed().as_secs_f64() * 1000. / $n as f64
                );
            }};
        }
        measure!(
            "open project",
            1,
            app.controller.open_project(&path).unwrap()
        );
        measure!("initial viewport", 1, app.machining_viewport());
        measure!("setup update + view", 5, {
            let _ = app.update(Message::OrbitBy(0.01, 0.005));
            app.view()
        });
        let id = app.controller.inspection_setup().operations[0].id();
        measure!("open operation editor", 1, app.begin_operation_draft(id));
        measure!("first editing view (cold geometry)", 1, app.view());
        measure!("dirty check", 100, app.controller.has_unsaved_changes());
        let geometry = app.controller.setup_geometry_state().unwrap();
        let binding = &app.controller.geometry_state().bindings[&id];
        measure!(
            "first resolve binding",
            1,
            cam_geometry3d::resolve_binding(&geometry, binding).unwrap()
        );
        measure!(
            "resolve binding (warm)",
            3,
            cam_geometry3d::resolve_binding(&geometry, binding).unwrap()
        );
        measure!(
            "profile selection geometry",
            3,
            app.profile_selection_geometry()
        );
        measure!(
            "validate editor geometry",
            3,
            app.validate_active_profile_geometry()
        );
        measure!("first editing viewport", 1, app.machining_viewport());
        measure!("cached viewport", 3, app.machining_viewport());
        measure!("inspector", 3, app.inspector());
        measure!("editing update + view", 3, {
            let _ = app.update(Message::OrbitBy(0.01, 0.005));
            app.view()
        });
    }
}

#[cfg(test)]
mod tool_table_filter_tests {
    use super::*;
    #[test]
    fn reorder_columns_before_and_after_targets() {
        let mut order = [0, 1, 2, 3, 4, 5, 6, 7];
        reorder_tool_column(&mut order, 1, 4, true);
        assert_eq!(order, [0, 2, 3, 4, 1, 5, 6, 7]);
        reorder_tool_column(&mut order, 6, 2, false);
        assert_eq!(order, [0, 6, 2, 3, 4, 1, 5, 7]);
        reorder_tool_column(&mut order, 6, 6, true);
        assert_eq!(order, [0, 6, 2, 3, 4, 1, 5, 7]);
    }

    #[test]
    fn search_and_value_filters_combine() {
        let values = [
            "T1", "Finisher", "End mill", "Maker", "123", "6.00", "50.00", "20.00",
        ]
        .map(String::from);
        let mut filters: [Option<BTreeSet<String>>; 8] = Default::default();
        filters[2] = Some(["End mill".to_string()].into_iter().collect());
        filters[5] = Some(["6.00".to_string()].into_iter().collect());
        assert!(tool_table_matches(&values, "maker", &filters));
        assert!(!tool_table_matches(&values, "missing", &filters));
        filters[5] = Some(BTreeSet::new());
        assert!(!tool_table_matches(&values, "", &filters));
    }
}
