//! Icon-based geometry collections and settings for surface and thread operations.
use super::geometry_selection::{
    geometry_collection, selection_hint as hint, selection_icon as icon, GeometryRow,
};
use super::*;
use iced::widget::column;

#[derive(Clone, Debug)]
pub(super) enum GeometryMessage {
    StockContours(bool),
    SelectStock,
    ResetStock,
    Add(CadFilter),
    Edit,
    Remove,
    Confirm,
    Cancel,
}

pub(super) struct GeometryEditor {
    stock: bool,
    wrapped: bool,
    kind: OpKind,
    cad_selection: Vec<EntityRef>,
    pending: Option<crate::PendingOp>,
    cad_editing: Option<u32>,
    drawing_editing: Option<u32>,
    inspecting: bool,
    additive: bool,
    row: Option<usize>,
    filter: CadFilter,
    filter_explicit: bool,
    heights: Option<cam_model::OperationHeightSettings>,
    offsets: BTreeMap<cam_model::HeightPlaneKind, String>,
    height_defaults: Option<cam_model::OperationHeightSettings>,
}

fn modes(kind: OpKind, cad: bool) -> Vec<(CadFilter, &'static str, &'static str)> {
    match kind {
        OpKind::Chamfer if cad => vec![
            (CadFilter::Face, "face", "Add modeled Chamfer Faces. The upper and lower face boundaries define the bevel; small cutters finish it in multiple stepdowns."),
            (CadFilter::Loop, "chain", "Add Chain. Select a rim or connected edges; set Chamfer width in Passes."),
        ],
        OpKind::Chamfer | OpKind::Engrave => vec![(CadFilter::Loop, "chain", if kind == OpKind::Chamfer {
            "Add Chain. Select an open or closed chamfer path. Configure individual edge selection in settings."
        } else {
            "Add Chain. Select an open or closed engraving path. Configure individual edge selection in settings."
        })],
        OpKind::Pocket if cad => vec![
            (CadFilter::Face, "face", "Add Pocket Floor. Select planar pocket floors at the same height, preserving their islands."),
            (CadFilter::Loop, "chain", "Add Chain. Select closed pocket boundaries; set the cutting depth in Heights."),
        ],
        OpKind::Pocket => vec![(CadFilter::Loop, "chain", "Add Chain. Select a closed drawing boundary and optional islands.")],
        OpKind::Carve if cad => vec![
            (CadFilter::Face, "face", "Add Face Contours. Select a planar face, preserving its holes as islands."),
            (CadFilter::Loop, "chain", "Add Chain. Select one closed boundary enclosing the carving region."),
        ],
        OpKind::Carve => vec![(CadFilter::Loop, "chain", "Add Chain. Select a closed drawing boundary; its holes remain uncut islands.")],
        OpKind::Bore => vec![(CadFilter::Face, "bore", "Select cylindrical holes or bosses with equal diameter and heights, or circular drawing loops.")],
        OpKind::Thread => vec![(CadFilter::Face, "thread", "Add Thread Centres. Select cylindrical bores or bosses with the same diameter and cutting heights; for a drawing, select circular holes.")],
        OpKind::Spiral => vec![(CadFilter::Body, "silhouette", "Add solid bodies to 3D Spiral finishing."), (CadFilter::Face, "face", "Add machining faces. All setup bodies remain protected.")],
        OpKind::Parallel => vec![(CadFilter::Body, "silhouette", "Add solid bodies to 3D Parallel finishing."), (CadFilter::Face, "face", "Add machining faces. All setup bodies remain protected.")],
        OpKind::Adaptive => vec![
            (CadFilter::Body, "silhouette", "Add solid bodies to 3D Adaptive Clearing. Multiple bodies and instances can be selected."),
            (CadFilter::Face, "face", "Add individual faces to 3D Adaptive Clearing. Faces need a positive-area projection onto setup XY; selection never expands to the whole body."),
        ],
        _ => Vec::new(),
    }
}

impl App {
    pub(super) fn stock_contour_editor(&self) -> bool {
        self.geometry_editor
            .as_ref()
            .is_some_and(|editor| editor.stock)
    }

    fn selected_stock_contours(&self) -> Result<cam_model::StockContours, String> {
        let mut stock = cam_model::StockContours {
            enabled: true,
            ..Default::default()
        };
        if self.operation_uses_cad_geometry() {
            let selected = self.controller.cad_selection();
            if !selected.is_empty() {
                if selected.iter().any(|r| {
                    !matches!(
                        r.kind,
                        EntityKind::Loop | EntityKind::Edge | EntityKind::Silhouette
                    )
                }) {
                    return Err(
                        "Select closed stock loops, connected edges, or a body silhouette".into(),
                    );
                }
                let geometry =
                    crate::cad_operations::geometry_for_setup(self.controller.document())?;
                let source = cam_geometry3d::resolve_selection_advisory(
                    &geometry,
                    selected,
                    if selected.len() == 1 && selected[0].kind == EntityKind::Silhouette {
                        cam_model::ExtractionRecipe::ProfileSilhouette
                    } else {
                        cam_model::ExtractionRecipe::PocketBoundary
                    },
                    self.controller.stock_box().1[2],
                )?;
                stock.regions =
                    cam_geometry3d::resolve_binding_advisory(&geometry, &source)?.regions;
                stock.source = Some(source);
            }
        } else {
            let selected = self.controller.pending_operation_geometry();
            if let Some(boundary) = selected.first() {
                self.controller
                    .planar_operation_eligibility(OpKind::Pocket, &selected)?;
                let outer = self
                    .controller
                    .loop_contour(*boundary)
                    .ok_or("Select a closed stock contour")?;
                let holes = selected
                    .iter()
                    .skip(1)
                    .map(|r| {
                        self.controller
                            .loop_contour(*r)
                            .cloned()
                            .ok_or("Stock islands must be closed")
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                stock.regions.push(
                    cam_geo::Polygon::with_holes(outer.clone(), holes)
                        .map_err(|e| e.to_string())?,
                );
            }
        }
        Ok(stock)
    }

    pub(super) fn stock_to_leave_supported(operation: &Operation) -> bool {
        matches!(
            operation,
            Operation::Profile(_)
                | Operation::Pocket(_)
                | Operation::Face(_)
                | Operation::Adaptive(_)
                | Operation::Parallel(_)
                | Operation::Spiral(_)
        )
    }

    pub(super) fn stock_to_leave_enabled(&self) -> bool {
        self.stock_to_leave_value().enabled
    }

    pub(super) fn stock_to_leave_value(&self) -> cam_model::StockToLeave {
        self.stock_to_leave_draft
            .unwrap_or_else(|| match self.controller.selection() {
                Selection::Operation(id) => {
                    self.controller.inspection_setup().stock_to_leave_for(id)
                }
                _ => cam_model::StockToLeave::default(),
            })
    }

    pub(super) fn operation_uses_cad_geometry(&self) -> bool {
        self.cad_pending_kind.is_some()
            || self.cad_editing_op.is_some()
            || self.operation_draft.as_ref().is_some_and(|op| {
                matches!(op, Operation::Pocket(p) if p.wrap.as_ref().is_some_and(|wrap| !wrap.geometry.is_empty())) || self.controller
                    .geometry_state()
                    .bindings
                    .contains_key(&op.id())
            })
    }

    pub(super) fn wrapped_pocket_geometry_editor(&self) -> bool {
        self.geometry_editor
            .as_ref()
            .is_some_and(|editor| editor.wrapped)
    }

    pub(super) fn operation_geometry_collection(
        &self,
        kind: OpKind,
        labels: Vec<String>,
    ) -> Element<'_, Message> {
        let selected = self
            .geometry_row_selection
            .filter(|index| *index < labels.len())
            .or_else(|| (!labels.is_empty()).then_some(0));
        let cad = self.operation_uses_cad_geometry();
        let sources = if self.wrapped_pocket_geometry_editor()
            || (self
                .operation_draft
                .as_ref()
                .and_then(Operation::rotary_wrap)
                .is_none()
                && (self.cad_pending_kind.is_some() || self.cad_editing_op.is_some()))
        {
            self.controller.cad_selection()
        } else if let Some(wrap) = self
            .operation_draft
            .as_ref()
            .and_then(Operation::rotary_wrap)
        {
            wrap.geometry.as_slice()
        } else {
            self.operation_binding_preview
                .as_ref()
                .or_else(|| {
                    self.operation_draft
                        .as_ref()
                        .and_then(|op| self.controller.geometry_state().bindings.get(&op.id()))
                })
                .map_or(&[][..], |binding| binding.selected.as_slice())
        };
        let rows = labels
            .into_iter()
            .enumerate()
            .map(|(index, label)| GeometryRow {
                icon: if kind == OpKind::Bore {
                    "bore"
                } else if kind == OpKind::Thread {
                    "thread"
                } else if matches!(kind, OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral)
                    && sources
                        .get(index)
                        .is_some_and(|r| r.kind == EntityKind::Body)
                {
                    "silhouette"
                } else if sources
                    .get(index)
                    .is_some_and(|r| r.kind == EntityKind::Face)
                {
                    "face"
                } else {
                    "chain"
                },
                label,
                action: Some(Message::GeometryRowSelected(index)),
            })
            .collect();
        let wrapped = kind == OpKind::Pocket
            && self
                .operation_draft
                .as_ref()
                .and_then(Operation::rotary_wrap)
                .is_some()
            && cad;
        let choices = if wrapped {
            vec![
                (CadFilter::Face, "face", "Add a cylindrical machining face."),
                (
                    CadFilter::Loop,
                    "chain",
                    "Add a closed developed boundary or connected edges.",
                ),
            ]
        } else {
            modes(kind, cad)
        };
        let mut toolbar = row![].spacing(4).align_y(Alignment::Center);
        for &(filter, name, description) in &choices {
            toolbar = toolbar.push(hint(
                button(icon(name, 28.))
                    .padding(4)
                    .style(inspector_secondary)
                    .on_press(Message::OperationGeometry(GeometryMessage::Add(filter))),
                description,
                self.tooltips,
            ));
        }
        toolbar = toolbar.push(hint(
            button(icon("remove", 28.))
                .padding(4)
                .style(inspector_secondary)
                .on_press_maybe(
                    selected.map(|_| Message::OperationGeometry(GeometryMessage::Remove)),
                ),
            "Delete the selected geometry item.",
            self.tooltips,
        ));
        toolbar = toolbar.push(hint(
            button(icon("settings", 28.))
                .padding(4)
                .style(inspector_secondary)
                .on_press_maybe(
                    selected.map(|_| Message::OperationGeometry(GeometryMessage::Edit)),
                ),
            "Configure the selected geometry and its selection mode.",
            self.tooltips,
        ));
        geometry_collection(
            if kind == OpKind::Bore {
                "Bore selection"
            } else if kind == OpKind::Thread {
                "Thread selection"
            } else if matches!(kind, OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral) {
                "Machining geometry"
            } else {
                "Contour selection"
            },
            rows,
            selected,
            Some(Message::OperationGeometry(if selected.is_some() {
                GeometryMessage::Edit
            } else {
                GeometryMessage::Add(choices[0].0)
            })),
            toolbar,
        )
    }

    pub(super) fn chamfer_faces_active(&self) -> bool {
        if self.cad_pending_kind == Some(OpKind::Chamfer) || self.cad_editing_op.is_some() {
            return self
                .controller
                .cad_selection()
                .first()
                .is_some_and(|r| r.kind == EntityKind::Face)
                && self.geometry_operation_kind() == Some(OpKind::Chamfer);
        }
        matches!(self.operation_draft, Some(Operation::Chamfer(ref op)) if !op.faces.is_empty())
    }

    pub(super) fn sync_pocket_geometry_height_defaults(
        &mut self,
        binding: &cam_model::GeometryBinding,
    ) {
        let Some(operation @ Operation::Pocket(_)) = self.operation_draft.as_ref() else {
            return;
        };
        let previous = self.operation_binding_preview.as_ref().or_else(|| {
            self.controller
                .geometry_state()
                .bindings
                .get(&operation.id())
        });
        let Some(previous) = previous else {
            return;
        };
        if previous.recipe == binding.recipe && previous.selected == binding.selected {
            return;
        }
        let defaults_for = |recipe: &cam_model::ExtractionRecipe| {
            if *recipe == cam_model::ExtractionRecipe::PocketBoundary {
                // Recognize defaults saved by older versions, which treated
                // the boundary as the top and cut the old depth below it.
                let mut settings = cam_model::OperationHeightSettings::default();
                *settings.definition_mut(cam_model::HeightPlaneKind::Top) =
                    cam_model::HeightDefinition::new(
                        cam_model::HeightReference::SelectedContours,
                        0.,
                    );
                *settings.definition_mut(cam_model::HeightPlaneKind::Bottom) =
                    cam_model::HeightDefinition::new(
                        cam_model::HeightReference::Plane(cam_model::HeightPlaneKind::Top),
                        -self
                            .controller
                            .document()
                            .operation_base_cutting_heights(operation)
                            .depth(),
                    );
                settings
            } else {
                cam_model::OperationHeightSettings::default()
            }
        };
        let old = defaults_for(&previous.recipe);
        // Changing from a floor to its boundary does not turn that boundary
        // into the top of a pocket with the old depth below it. Match "Use
        // defaults": cut from stock top down to the selected floor/contour.
        let new = cam_model::OperationHeightSettings::default();
        if let Some(settings) = &mut self.height_draft {
            for plane in cam_model::HeightPlaneKind::ALL {
                let definition = old.definition(plane);
                if settings.definition(plane) == definition
                    && self
                        .height_offsets
                        .get(&plane)
                        .and_then(|s| s.parse::<f64>().ok())
                        == Some(definition.offset)
                {
                    *settings.definition_mut(plane) = new.definition(plane).clone();
                    self.height_offsets
                        .insert(plane, fmt_num(new.definition(plane).offset));
                }
            }
        }
    }

    pub(super) fn sync_chamfer_face_height_defaults(&mut self, before: Option<&Operation>) {
        let (Some(before @ Operation::Chamfer(_)), Some(after @ Operation::Chamfer(chamfer))) =
            (before, self.operation_draft.as_ref())
        else {
            return;
        };
        if chamfer.faces.is_empty() {
            return;
        }
        let before = before.clone();
        let after = after.clone();
        self.sync_chamfer_height_defaults(&before, &after);
    }

    pub(super) fn sync_chamfer_height_defaults(&mut self, before: &Operation, after: &Operation) {
        let previous =
            crate::cad_operations::selection_height_defaults(self.controller.document(), before);
        let defaults =
            crate::cad_operations::selection_height_defaults(self.controller.document(), after);
        if let Some(settings) = &mut self.height_draft {
            for plane in cam_model::HeightPlaneKind::ALL {
                let old = previous.definition(plane);
                if settings.definition(plane) == old
                    && self
                        .height_offsets
                        .get(&plane)
                        .and_then(|v| v.parse::<f64>().ok())
                        == Some(old.offset)
                {
                    *settings.definition_mut(plane) = defaults.definition(plane).clone();
                    self.height_offsets
                        .insert(plane, fmt_num(defaults.definition(plane).offset));
                }
            }
        }
    }

    pub(super) fn sync_chamfer_geometry_fields(&mut self) {
        let Some(operation @ Operation::Chamfer(_)) = self.operation_draft.as_ref() else {
            return;
        };
        let values: Vec<_> = self
            .geometry_operation_fields(operation)
            .into_iter()
            .filter_map(|field| op_field(operation, field).map(|value| (field, fmt_num(value))))
            .collect();
        for (field, value) in values {
            self.fields.entry(field).or_insert(value);
        }
    }

    pub(super) fn geometry_operation_fields(&self, operation: &Operation) -> Vec<Field> {
        let mut fields = operation_fields(operation);
        self.add_cutting_fields(operation, &mut fields);
        if self.stock_to_leave_draft.is_some_and(|stock| stock.enabled) {
            if matches!(
                operation,
                Operation::Profile(_)
                    | Operation::Pocket(_)
                    | Operation::Adaptive(_)
                    | Operation::Parallel(_)
                    | Operation::Spiral(_)
            ) {
                fields.push(Field::StockLeaveRadial);
            }
            if matches!(
                operation,
                Operation::Profile(_)
                    | Operation::Pocket(_)
                    | Operation::Face(_)
                    | Operation::Adaptive(_)
                    | Operation::Parallel(_)
                    | Operation::Spiral(_)
            ) {
                fields.push(Field::StockLeaveAxial);
            }
        }
        if self.chamfer_faces_active() {
            fields.retain(|field| {
                !matches!(
                    field,
                    Field::ChamferWidth
                        | Field::ChamferDepth
                        | Field::ChamferStep
                        | Field::LeadOverlap
                )
            });
        }
        fields
    }

    pub(super) fn drawing_sources_for_operation(&self, operation: &Operation) -> Vec<LoopRef> {
        let candidates: Vec<_> = self
            .controller
            .regions()
            .iter()
            .enumerate()
            .flat_map(|(index, region)| {
                std::iter::once(LoopRef::outer(index))
                    .chain((0..region.holes().len()).map(move |hole| LoopRef::hole(index, hole)))
            })
            .chain((0..self.controller.open_paths().len()).map(LoopRef::open))
            .collect();
        if let Operation::Bore(op) = operation {
            return op.points.iter().map(|point| candidates.iter().copied().find(|reference| {
                matches!(self.controller.operation_with_drawing_geometry(operation, &[*reference]), Ok(Operation::Bore(replacement))
                    if replacement.points.iter().any(|p| (p[0] - point[0]).abs() < 1e-8 && (p[1] - point[1]).abs() < 1e-8))
            })).collect::<Option<Vec<_>>>().unwrap_or_default();
        }
        if let Operation::Thread(op) = operation {
            return op.points.iter().map(|point| candidates.iter().copied().find(|reference| {
                matches!(self.controller.operation_with_drawing_geometry(operation, &[*reference]), Ok(Operation::Thread(replacement))
                    if replacement.points.iter().any(|p| (p[0] - point[0]).abs() < 1e-8 && (p[1] - point[1]).abs() < 1e-8))
            })).collect::<Option<Vec<_>>>().unwrap_or_default();
        }
        let (contours, closed): (Vec<_>, _) = match operation {
            Operation::Chamfer(op) => (vec![&op.chain], op.closed),
            Operation::Engrave(op) => (vec![&op.chain], op.closed),
            Operation::Pocket(cam_model::PocketOp {
                boundary, islands, ..
            }) => (
                std::iter::once(boundary).chain(islands.iter()).collect(),
                true,
            ),
            Operation::Carve(op) => (
                std::iter::once(&op.boundary)
                    .chain(op.islands.iter())
                    .collect(),
                true,
            ),
            _ => return Vec::new(),
        };
        contours
            .into_iter()
            .map(|contour| {
                candidates.iter().copied().find(|reference| {
                    self.controller
                        .loop_points(*reference)
                        .is_some_and(|(points, is_closed)| {
                            is_closed == closed && points == contour.points()
                        })
                })
            })
            .collect::<Option<Vec<_>>>()
            .unwrap_or_default()
    }

    pub(super) fn operation_geometry_message(&mut self, message: GeometryMessage) {
        match message {
            GeometryMessage::StockContours(enabled) => {
                self.edit_operation_draft(|op| {
                    if let Operation::Pocket(p) = op {
                        p.stock_contours.enabled = enabled;
                    }
                });
            }
            GeometryMessage::ResetStock => {
                if self.stock_contour_editor() {
                    self.controller.set_cad_selection(vec![]);
                    if self.controller.pending_op().is_some() {
                        let _ = self.controller.set_pending_operation_geometry(&[], None);
                    }
                } else {
                    self.edit_operation_draft(|op| {
                        if let Operation::Pocket(p) = op {
                            p.stock_contours.enabled = true;
                            p.stock_contours.regions.clear();
                            p.stock_contours.source = None;
                        }
                    });
                }
                self.show_stock = true;
                let (min, max) = self.controller.stock_box();
                self.status = format!("Setup stock selected: X{:.3}…{:.3}, Y{:.3}…{:.3} mm. OK applies the stock footprint.", min[0], max[0], min[1], max[1]);
            }
            GeometryMessage::Add(_) | GeometryMessage::Edit | GeometryMessage::SelectStock => {
                if self.geometry_editor.is_some() {
                    return;
                }
                let Some(
                    kind @ (OpKind::Pocket
                    | OpKind::Chamfer
                    | OpKind::Engrave
                    | OpKind::Carve
                    | OpKind::Bore
                    | OpKind::Thread
                    | OpKind::Adaptive
                    | OpKind::Parallel
                    | OpKind::Spiral),
                ) = self.geometry_operation_kind()
                else {
                    return;
                };
                let pending = self.controller.pending_op();
                let editor = GeometryEditor {
                    stock: matches!(message, GeometryMessage::SelectStock),
                    wrapped: !matches!(message, GeometryMessage::SelectStock)
                        && kind == OpKind::Pocket
                        && self
                            .operation_draft
                            .as_ref()
                            .and_then(Operation::rotary_wrap)
                            .is_some()
                        && self.operation_uses_cad_geometry(),
                    kind,
                    cad_selection: self.controller.cad_selection().to_vec(),
                    pending,
                    cad_editing: self.cad_editing_op,
                    drawing_editing: self.drawing_editing_op,
                    inspecting: self.cad_inspecting,
                    additive: self.geometry_additive,
                    row: self.geometry_row_selection,
                    filter: self.cad_filter,
                    filter_explicit: self.cad_filter_explicit,
                    heights: self.height_draft.clone(),
                    offsets: self.height_offsets.clone(),
                    height_defaults: self.creation_geometry_height_defaults.clone(),
                };
                self.height_reference_pick = None;
                if editor.wrapped {
                    let selected = self
                        .operation_draft
                        .as_ref()
                        .and_then(Operation::rotary_wrap)
                        .map(|wrap| wrap.geometry.clone())
                        .unwrap_or_default();
                    self.controller.set_cad_selection(selected);
                    self.geometry_additive = true;
                    if let GeometryMessage::Add(filter) = message {
                        self.cad_filter = filter;
                    } else {
                        self.cad_filter =
                            match self.controller.cad_selection().first().map(|r| r.kind) {
                                Some(EntityKind::Edge) => CadFilter::Edge,
                                Some(EntityKind::Loop) => CadFilter::Loop,
                                _ => CadFilter::Face,
                            };
                    }
                    self.cad_filter_explicit = true;
                    self.cad_hover = None;
                    self.cad_candidates.clear();
                } else {
                    let _ = self.pick_operation_geometry();
                }
                if self.drawing_editing_op.is_some() && editor.pending.is_none() {
                    if let Some(operation) = self.operation_draft.as_ref() {
                        let references = self.drawing_sources_for_operation(operation);
                        let _ = self
                            .controller
                            .set_pending_operation_geometry(&references, None);
                    }
                }
                if let GeometryMessage::Add(filter) = message {
                    if !editor.wrapped && self.operation_uses_cad_geometry() {
                        let _ = self.update(Message::GeometryFilterChanged(filter));
                    }
                }
                if editor.stock {
                    let stock = match self.operation_draft.as_ref() {
                        Some(Operation::Pocket(p)) => p.stock_contours.clone(),
                        _ => return,
                    };
                    self.cad_filter = if stock
                        .source
                        .as_ref()
                        .and_then(|s| s.selected.first())
                        .is_some_and(|r| r.kind == EntityKind::Silhouette)
                    {
                        CadFilter::Silhouette
                    } else if stock
                        .source
                        .as_ref()
                        .and_then(|s| s.selected.first())
                        .is_some_and(|r| r.kind == EntityKind::Edge)
                    {
                        CadFilter::Edge
                    } else {
                        CadFilter::Loop
                    };
                    self.cad_filter_explicit = true;
                    self.show_stock = true;
                    if self.operation_uses_cad_geometry() {
                        self.controller
                            .set_cad_selection(stock.source.map_or_else(Vec::new, |b| b.selected));
                    } else {
                        // A stock contour uses its own drawing selection. Do not
                        // let it replace the pocket boundary in the parent draft.
                        let _ = self.controller.set_pending_operation_geometry(&[], None);
                        if let Some(region) = stock.regions.first() {
                            let references: Vec<_> = self
                                .controller
                                .regions()
                                .iter()
                                .enumerate()
                                .flat_map(|(i, p)| {
                                    std::iter::once((LoopRef::outer(i), p.outer())).chain(
                                        p.holes()
                                            .iter()
                                            .enumerate()
                                            .map(move |(h, c)| (LoopRef::hole(i, h), c)),
                                    )
                                })
                                .collect();
                            let selected: Vec<_> = std::iter::once(region.outer())
                                .chain(region.holes())
                                .filter_map(|c| {
                                    references
                                        .iter()
                                        .find(|(_, candidate)| candidate.to_ccw() == c.to_ccw())
                                        .map(|(r, _)| *r)
                                })
                                .collect();
                            let _ = self
                                .controller
                                .set_pending_operation_geometry(&selected, None);
                        }
                    }
                    self.status = "Select a stock box edge, closed model contour, connected edges, or Silhouette. Use setup stock selects the setup box.".into();
                }
                self.geometry_editor = Some(editor);
            }
            GeometryMessage::Remove => {
                let index = self.geometry_row_selection.unwrap_or(0);
                let editing_operation = !self.creating_operation();
                if editing_operation {
                    self.operation_geometry_message(GeometryMessage::Edit);
                }
                self.remove_operation_geometry(Some(index));
                if editing_operation && self.geometry_settings_valid().is_ok() {
                    self.operation_geometry_message(GeometryMessage::Confirm);
                }
            }
            GeometryMessage::Cancel => {
                let Some(editor) = self.geometry_editor.take() else {
                    return;
                };
                self.controller.set_cad_selection(editor.cad_selection);
                self.controller.restore_pending_operation(editor.pending);
                self.cad_editing_op = editor.cad_editing;
                self.drawing_editing_op = editor.drawing_editing;
                self.cad_inspecting = editor.inspecting;
                self.geometry_additive = editor.additive;
                self.geometry_row_selection = editor.row;
                self.cad_filter = editor.filter;
                self.cad_filter_explicit = editor.filter_explicit;
                self.height_draft = editor.heights;
                self.height_offsets = editor.offsets;
                self.creation_geometry_height_defaults = editor.height_defaults;
                self.cad_candidates.clear();
                self.cad_hover = None;
                self.status = "Geometry settings cancelled; other operation edits retained.".into();
            }
            GeometryMessage::Confirm => {
                if self.stock_contour_editor() {
                    match self.selected_stock_contours() {
                        Ok(stock) => {
                            self.operation_geometry_message(GeometryMessage::Cancel);
                            self.edit_operation_draft(|op| {
                                if let Operation::Pocket(p) = op {
                                    p.stock_contours = stock;
                                }
                            });
                            self.status = "Stock contours staged. Operation OK/Cancel applies or discards the changes.".into();
                        }
                        Err(reason) => self.status = reason,
                    }
                    return;
                }
                if let Err(error) = self.geometry_settings_valid() {
                    self.status = error;
                    return;
                }
                if self.wrapped_pocket_geometry_editor() {
                    let selected = self.controller.cad_selection().to_vec();
                    self.edit_operation_draft(|op| {
                        if let Some(mut wrap) = op.rotary_wrap().cloned() {
                            wrap.geometry = selected;
                            op.set_rotary_wrap(Some(wrap));
                        }
                    });
                    self.geometry_editor = None;
                    self.geometry_additive = false;
                    self.geometry_row_selection = None;
                    self.status =
                        "Geometry staged. The operation's OK/Cancel applies or discards all tabs."
                            .into();
                    return;
                }
                if self.cad_editing_op.is_some() {
                    let _ = self.update(Message::ConfirmCadOperation);
                    if self.cad_editing_op.is_some() {
                        return;
                    }
                } else if self.drawing_editing_op.is_some() {
                    let fields = self.fields.clone();
                    let _ = self.update(Message::UseDrawingGeometry);
                    self.fields = fields;
                    if self.drawing_editing_op.is_some() {
                        return;
                    }
                }
                self.geometry_editor = None;
                self.geometry_additive = false;
                self.operation_tab = OperationTab::Geometry;
                self.status =
                    "Geometry staged. The operation's OK/Cancel applies or discards all tabs."
                        .into();
            }
        }
    }

    fn geometry_settings_valid(&self) -> Result<(), String> {
        if self.stock_contour_editor() {
            return self.selected_stock_contours().map(|_| ());
        }
        let editor = self
            .geometry_editor
            .as_ref()
            .ok_or("Open geometry settings")?;
        if editor.wrapped {
            let selected = self.controller.cad_selection();
            if selected.is_empty() {
                return Err("Select at least one cylindrical face or closed boundary".into());
            }
            let mut operation = self
                .operation_draft
                .clone()
                .ok_or("Operation unavailable")?;
            let mut wrap = operation.rotary_wrap().cloned().ok_or("Wrap unavailable")?;
            wrap.geometry = selected.to_vec();
            let has_cylinder = wrap.cylinder.is_some();
            operation.set_rotary_wrap(Some(wrap));
            if !has_cylinder {
                return Ok(());
            }
            let geometry = crate::cad_operations::geometry_for_setup(self.controller.document())?;
            return crate::cad_operations::resolve_wrapped_operation(&geometry, &mut operation);
        }
        if let Some(id) = self.cad_editing_op {
            self.preview_geometry_draft(id, self.controller.cad_selection().to_vec())
                .map(|_| ())
        } else if let Some(id) = self.drawing_editing_op {
            let operation = self.current_operation(id).ok_or("Operation unavailable")?;
            self.controller
                .operation_with_drawing_geometry(
                    operation,
                    &self.controller.pending_operation_geometry(),
                )
                .map(|_| ())
        } else if self.cad_pending_kind.is_some() {
            self.validate_active_profile_geometry().unwrap_or_else(|| {
                self.controller
                    .preview_cad_operation(editor.kind, self.controller.cad_selection())
                    .map(|_| ())
            })
        } else {
            self.controller
                .planar_operation_eligibility(
                    editor.kind,
                    &self.controller.pending_operation_geometry(),
                )
                .map(|_| ())
        }
    }

    pub(super) fn operation_geometry_settings(&self) -> Element<'_, Message> {
        let editor = self.geometry_editor.as_ref().unwrap();
        let cad = self.operation_uses_cad_geometry();
        let kind = editor.kind;
        let name = if editor.stock {
            "Stock contours"
        } else {
            match kind {
                OpKind::Pocket => "Pocket",
                OpKind::Chamfer => "Chamfer",
                OpKind::Engrave => "Engrave",
                OpKind::Carve => "Carve",
                OpKind::Bore => "Bore",
                OpKind::Thread => "Thread",
                OpKind::Parallel => "3D Parallel",
                OpKind::Spiral => "3D Spiral",
                OpKind::Adaptive => "3D Adaptive Clearing",
                _ => unreachable!(),
            }
        };
        let labels = if cad {
            cad_geometry_labels(self.controller.cad_selection())
        } else {
            drawing_geometry_labels(&self.controller.pending_operation_geometry())
        };
        let stock_button: Element<'_, Message> = if editor.stock {
            inspector_button("Use setup stock")
                .on_press(Message::OperationGeometry(GeometryMessage::ResetStock))
                .into()
        } else {
            Space::new().width(0).into()
        };
        let mut body = column![
            row![
                icon(
                    if kind == OpKind::Thread {
                        "thread"
                    } else if matches!(kind, OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral) {
                        if self.cad_filter == CadFilter::Body {
                            "silhouette"
                        } else {
                            "face"
                        }
                    } else if matches!(kind, OpKind::Pocket | OpKind::Carve)
                        && cad
                        && self.cad_filter == CadFilter::Face
                    {
                        "face"
                    } else {
                        "chain"
                    },
                    24.
                ),
                text(format!("{name} geometry settings")).size(15)
            ]
            .spacing(8)
            .align_y(Alignment::Center),
            row![
                text("Selection").size(12).width(90),
                inspector_pick_button(
                    if self.geometry_additive {
                        "Selecting…"
                    } else {
                        "Select"
                    },
                    self.geometry_additive
                )
                .on_press(Message::GeometryPick),
                inspector_button("Clear")
                    .on_press_maybe((!labels.is_empty()).then_some(Message::GeometryClear)),
                stock_button
            ]
            .spacing(6)
            .align_y(Alignment::Center),
            geometry_selection::selected_feature_names(labels),
        ]
        .spacing(9);
        if cad
            && matches!(
                kind,
                OpKind::Pocket
                    | OpKind::Chamfer
                    | OpKind::Engrave
                    | OpKind::Carve
                    | OpKind::Adaptive
                    | OpKind::Parallel
                    | OpKind::Spiral
            )
        {
            let options = if editor.stock {
                vec![
                    (
                        CadFilter::Silhouette,
                        "silhouette",
                        "Select the complete body silhouette projected onto setup XY",
                    ),
                    (CadFilter::Loop, "closed", "Closed stock boundary loop"),
                    (
                        CadFilter::Edge,
                        "chain",
                        "Connected edges forming a closed stock boundary",
                    ),
                ]
            } else if matches!(kind, OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral) {
                vec![
                    (CadFilter::Body, "silhouette", "Whole solid body"),
                    (
                        CadFilter::Face,
                        "face",
                        "Selected faces only; each needs a positive-area setup XY projection",
                    ),
                ]
            } else if kind == OpKind::Pocket {
                vec![
                    (
                        CadFilter::Face,
                        "face",
                        if editor.wrapped {
                            "Cylindrical machining face"
                        } else {
                            "Pocket floor and islands"
                        },
                    ),
                    (CadFilter::Loop, "closed", "Closed boundary loop"),
                    (
                        CadFilter::Edge,
                        "chain",
                        "Select connected edges forming a closed boundary",
                    ),
                ]
            } else if kind == OpKind::Carve {
                vec![
                    (CadFilter::Face, "face", "Face and islands"),
                    (CadFilter::Loop, "chain", "Closed boundary loop"),
                ]
            } else if kind == OpKind::Chamfer {
                vec![
                    (
                        CadFilter::Face,
                        "face",
                        "Modeled chamfer faces: use their upper and lower boundaries",
                    ),
                    (CadFilter::Loop, "closed", "Boundary loop"),
                    (
                        CadFilter::Edge,
                        "chain",
                        "Select connected individual edges",
                    ),
                ]
            } else {
                vec![
                    (CadFilter::Loop, "closed", "Boundary loop"),
                    (
                        CadFilter::Edge,
                        "chain",
                        "Select connected individual edges",
                    ),
                ]
            };
            let mut controls = row![text("Mode").size(12).width(90)]
                .spacing(6)
                .align_y(Alignment::Center);
            for (filter, image, description) in options {
                let active = self.cad_filter == filter;
                controls = controls.push(hint(
                    button(icon(image, 27.))
                        .padding(4)
                        .style(move |theme, status| inspector_tab(theme, status, active))
                        .on_press(Message::GeometryFilterChanged(filter)),
                    description,
                    self.tooltips,
                ));
            }
            body = body.push(controls);
        }
        let description = if editor.stock {
            "Select a stock box edge, closed model loop, connected edge chain, or body silhouette. Use setup stock restores the setup box. Stock contours describe material; the cutter can approach from outside the footprint. Leave unchecked for closed pockets."
        } else if editor.wrapped {
            "Select cylindrical faces, closed loops, or connected edges. Select the wrap cylinder on the Multi-axis tab. Faces at different radii require separate operations."
        } else {
            match kind {
            OpKind::Pocket => "Select planar pocket floors with their islands, closed boundary loops, or closed edge chains. Multiple pockets must share one floor height. Default heights cut from stock top down to the selected geometry.",
            OpKind::Chamfer => "Select modeled inclined chamfer faces to use their upper and lower boundaries, or select a single edge, connected chain or boundary loop and set Chamfer width in Passes. Face machining automatically splits the bevel into bands that fit the cutter; set Stepdown on Passes for smaller bands.",
            OpKind::Engrave => "Select a boundary loop, or connected edges for an open or closed engraving path.",
            OpKind::Carve => "Select one closed region. Face selection preserves its inner boundaries as islands.",
            OpKind::Bore => "Select cylindrical holes or bosses, or circular drawing loops. Selected features must share diameter and cutting heights.",
            OpKind::Thread => "Select cylindrical bores or bosses, or circular drawing holes. Selected features must share diameter and cutting heights. A verified rounded entrance is supported; only the cylinder is threaded. Choose a thread mill, then review the suggested standard size in Geometry > Thread or enter a custom major diameter and pitch.",
            OpKind::Spiral => "Select solid bodies or machining faces for spiral finishing with a ball nose tool.",
            OpKind::Parallel => "Select solid bodies or machining faces for ball nose finishing. Containment uses the selected geometry bounding box.",
            OpKind::Adaptive => "Select whole solid bodies, individual faces, or both. Body selection includes all of its faces; face selection limits machining to the chosen face footprint.",
            _ => unreachable!(),
        }
        };
        body = body.push(text(description).size(12));
        if cad && !editor.stock && !editor.wrapped {
            if let Some(info) =
                crate::cad_operations::geometry_for_setup(self.controller.document())
                    .ok()
                    .and_then(|geometry| {
                        crate::cad_operations::recipe(
                            kind,
                            &geometry,
                            self.controller.cad_selection(),
                        )
                        .ok()
                        .and_then(|recipe| {
                            cam_geometry3d::verify_selection(
                                &geometry,
                                self.controller.cad_selection(),
                                recipe,
                                self.controller.stock_box().1[2],
                            )
                        })
                    })
            {
                body = body.push(
                    text(format!(
                        "Info: Automatic verification: {info}. Operation settings are used."
                    ))
                    .size(11)
                    .color(INSPECTOR_MUTED),
                );
            }
        }
        let valid = self.geometry_settings_valid();
        if let Err(error) = &valid {
            body = body.push(text(error.clone()).size(11).color(palette::WARN));
        }
        column![
            scrollable(body.padding(14))
                .width(Length::Fill)
                .height(Length::Fill),
            inspector_footer(
                row![
                    inspector_button("Cancel")
                        .on_press(Message::OperationGeometry(GeometryMessage::Cancel)),
                    Space::new().width(Length::Fill),
                    inspector_button("OK")
                        .style(inspector_primary)
                        .on_press_maybe(
                            valid
                                .is_ok()
                                .then_some(Message::OperationGeometry(GeometryMessage::Confirm))
                        )
                ]
                .spacing(8)
            )
        ]
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wrapped_pocket_fixture() -> (App, EntityRef, EntityRef) {
        static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        let path = std::env::temp_dir().join(format!(
            "wrapped-pocket-picker-{}-{}.ocam",
            std::process::id(),
            NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        ));
        std::fs::write(&path, include_str!("../tests/fixtures/stepped_rotary.ocam")).unwrap();
        app.controller.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        let face = app
            .controller
            .operation(0)
            .unwrap()
            .rotary_wrap()
            .unwrap()
            .cylinder
            .clone()
            .unwrap();
        let boundary = EntityRef {
            kind: EntityKind::Loop,
            id: 26,
            ..face.clone()
        };
        app.controller.edit_operation(0, |op| {
            let mut wrap = op.rotary_wrap().unwrap().clone();
            wrap.geometry = vec![face.clone()];
            op.set_rotary_wrap(Some(wrap));
        });
        app.setup_draft = None;
        app.begin_operation_draft(0);
        let _ = app.update(Message::OperationTabChanged(OperationTab::Geometry));
        (app, face, boundary)
    }

    #[test]
    fn wrapped_pocket_uses_shared_geometry_editor_and_stages_changes() {
        let (mut app, face, boundary) = wrapped_pocket_fixture();
        let before = app.controller.document().clone();
        let _ = app.operation_inspector(0);
        let _ = app.update(Message::FieldChanged(Field::Feed, "321".into()));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Edit));
        assert!(app.wrapped_pocket_geometry_editor());
        assert!(app.operation_geometry_preview().strips.is_empty());
        assert_eq!(app.controller.cad_selection(), std::slice::from_ref(&face));
        let _ = app.operation_geometry_settings();
        let larger_face = EntityRef {
            id: 2,
            ..face.clone()
        };
        app.controller
            .set_cad_selection(vec![face.clone(), larger_face]);
        assert!(app
            .geometry_settings_valid()
            .unwrap_err()
            .contains("different radii"));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(app.wrapped_pocket_geometry_editor());
        app.controller.set_cad_selection(vec![face.clone()]);
        let _ = app.update(Message::GeometryFilterChanged(CadFilter::Loop));
        assert!(app.controller.cad_selection().is_empty());
        let viewport = app.machining_viewport();
        assert_eq!(viewport.cad_filter, EntityKind::Loop);
        assert_eq!(viewport.cad_operation_kind, Some(OpKind::Engrave));
        let _ = app.update(Message::CadSelect(boundary.clone()));
        assert_eq!(app.controller.cad_selection(), std::slice::from_ref(&boundary));
        assert_eq!(app.editor_selected_geometry(), vec![boundary.clone()]);
        assert!(
            app.geometry_settings_valid().is_ok(),
            "{:?}",
            app.geometry_settings_valid()
        );
        assert_eq!(
            app.operation_draft
                .as_ref()
                .unwrap()
                .rotary_wrap()
                .unwrap()
                .geometry,
            vec![face.clone()]
        );
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Cancel));
        assert!(!app.wrapped_pocket_geometry_editor());
        assert_eq!(app.controller.document(), &before);
        assert_eq!(app.fields[&Field::Feed], "321");
        assert_eq!(
            app.operation_draft
                .as_ref()
                .unwrap()
                .rotary_wrap()
                .unwrap()
                .geometry,
            vec![face]
        );

        let _ = app.update(Message::OperationGeometry(GeometryMessage::Add(
            CadFilter::Loop,
        )));
        let _ = app.update(Message::GeometryClear);
        assert!(app.geometry_settings_valid().is_err());
        let _ = app.update(Message::CadSelect(boundary.clone()));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(app.geometry_editor.is_none(), "{}", app.status);
        assert_eq!(
            app.operation_draft
                .as_ref()
                .unwrap()
                .rotary_wrap()
                .unwrap()
                .geometry,
            vec![boundary.clone()]
        );
        assert_eq!(app.controller.document(), &before);
        let _ = app.update(Message::Apply);
        assert_eq!(
            app.controller
                .operation(0)
                .unwrap()
                .rotary_wrap()
                .unwrap()
                .geometry,
            vec![boundary]
        );
        assert!(app.controller.undo());
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn wrapped_pocket_click_and_remove_use_the_shared_staged_picker() {
        let (mut app, face, boundary) = wrapped_pocket_fixture();
        let before = app.controller.document().clone();
        let _ = app.update(Message::CadSelect(boundary.clone()));
        assert!(app.wrapped_pocket_geometry_editor(), "{}", app.status);
        assert_eq!(app.controller.cad_selection(), &[boundary]);
        let _ = app.update(Message::GeometryClear);
        let _ = app.update(Message::CadSelect(face.clone()));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(app.geometry_editor.is_none(), "{}", app.status);
        let _ = app.update(Message::GeometryRowSelected(0));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Remove));
        assert!(app.wrapped_pocket_geometry_editor());
        assert!(app.controller.cad_selection().is_empty());
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Cancel));
        assert_eq!(
            app.operation_draft
                .as_ref()
                .unwrap()
                .rotary_wrap()
                .unwrap()
                .geometry,
            vec![face]
        );
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn wrapped_pocket_creation_preserves_and_edits_the_existing_cad_selection() {
        let (mut app, face, _) = wrapped_pocket_fixture();
        let _ = app.update(Message::CancelOperationDraft);
        let before = app.controller.document().clone();
        let _ = app.update(Message::BeginOp(OpKind::Pocket));
        assert_eq!(app.cad_pending_kind, Some(OpKind::Pocket));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Geometry));
        let _ = app.update(Message::CadSelect(face.clone()));
        let _ = app.update(Message::Rotary(rotary_view::Action::Enable(true)));
        assert_eq!(
            app.operation_draft
                .as_ref()
                .unwrap()
                .rotary_wrap()
                .unwrap()
                .geometry,
            vec![face.clone()]
        );
        let _ = app.update(Message::Rotary(rotary_view::Action::Pick(
            rotary_view::Pick::Cylinder,
        )));
        let _ = app.update(Message::CadSelect(face.clone()));
        assert_eq!(
            app.operation_draft
                .as_ref()
                .unwrap()
                .rotary_wrap()
                .unwrap()
                .cylinder,
            Some(face.clone())
        );
        let _ = app.operation_inspector(0);
        let _ = app.update(Message::GeometryRowSelected(0));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Remove));
        assert!(app
            .operation_draft
            .as_ref()
            .unwrap()
            .rotary_wrap()
            .unwrap()
            .geometry
            .is_empty());
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Add(
            CadFilter::Face,
        )));
        let _ = app.update(Message::CadSelect(face.clone()));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(app.geometry_editor.is_none(), "{}", app.status);
        assert_eq!(
            app.operation_draft
                .as_ref()
                .unwrap()
                .rotary_wrap()
                .unwrap()
                .geometry,
            vec![face]
        );
        assert!(
            app.creation_operation().is_ok(),
            "{:?}",
            app.creation_operation().err()
        );
        assert_eq!(app.controller.document(), &before);
        let _ = app.update(Message::CancelOp);
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn wrapped_drawing_pocket_uses_the_standard_drawing_geometry_editor() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample.dxf").unwrap();
        app.controller.new_operation(OpKind::Pocket);
        let Selection::Operation(id) = app.controller.selection() else {
            panic!("pocket selected")
        };
        app.setup_draft = None;
        app.begin_operation_draft(id);
        let _ = app.update(Message::Rotary(rotary_view::Action::Enable(true)));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Geometry));
        assert!(!app.operation_uses_cad_geometry());
        let before = app.controller.document().clone();
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Add(
            CadFilter::Loop,
        )));
        assert!(app.geometry_editor.is_some());
        assert!(!app.wrapped_pocket_geometry_editor());
        assert!(app.drawing_editing_op.is_some());
        let _ = app.update(Message::GeometryClear);
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Cancel));
        assert!(app.geometry_editor.is_none());
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    #[ignore = "manual supplied-project regression; set OCAM_STOCK_PROJECT"]
    fn supplied_stock_contours_project() {
        use shader::Program;
        let path = std::env::var("OCAM_STOCK_PROJECT").unwrap();
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_project(Path::new(&path)).unwrap();
        app.begin_operation_draft(0);
        app.operation_tab = OperationTab::Geometry;
        let before = app.controller.operation(0).unwrap().clone();
        let (min, max) = app.controller.stock_box();
        let plan = |enabled: bool| {
            let started = std::time::Instant::now();
            eprintln!("Planning supplied project, stock contours {enabled}");
            let mut document = app.controller.document().clone();
            let Operation::Pocket(p) = &mut document.setup_mut().operations[0] else {
                panic!("pocket")
            };
            p.stock_contours.enabled = enabled;
            let resolved = crate::cad_operations::resolve_document(&document).unwrap();
            if enabled {
                let b = &resolved.geometry.bindings[&0];
                let shadow = cam_geometry3d::pocket_stock_shadow(
                    &resolved.geometry,
                    &b.selected,
                    b.effective_heights().bottom,
                    || false,
                )
                .unwrap()
                .unwrap();
                eprintln!(
                    "Stock walls: {} polygons, {} vertices",
                    shadow.len(),
                    shadow
                        .iter()
                        .map(|p| p.outer().len() + p.holes().iter().map(|h| h.len()).sum::<usize>())
                        .sum::<usize>()
                );
            }
            let result = cam_toolpath::build_job(
                &resolved,
                1000.,
                cam_cldata::SpindleDir::Cw,
                Some(([min[0], min[1]], [max[0], max[1]])),
                200.,
                &cam_toolpath::CancelToken::new(),
            );
            eprintln!(
                "Planned in {:?}, {} moves",
                started.elapsed(),
                result.program.len()
            );
            result
        };
        let closed = plan(false);
        let setup = plan(true);
        assert_eq!(
            setup.completion,
            cam_toolpath::JobCompletion::Complete,
            "{:?}",
            setup.diagnostics
        );
        let max_cut_x = |job: &cam_toolpath::JobResult| {
            job.program
                .steps()
                .iter()
                .filter_map(|s| match s {
                    cam_cldata::Step::Linear { to, tag, .. }
                        if tag.kind == cam_cldata::MoveKind::Cutting =>
                    {
                        Some(to.x)
                    }
                    cam_cldata::Step::Arc { end, tag, .. }
                        if tag.kind == cam_cldata::MoveKind::Cutting =>
                    {
                        Some(end.x)
                    }
                    _ => None,
                })
                .fold(f64::NEG_INFINITY, f64::max)
        };
        eprintln!(
            "closed max X {}, setup stock max X {}",
            max_cut_x(&closed),
            max_cut_x(&setup)
        );
        assert!(max_cut_x(&setup) > max[0]);
        assert_ne!(setup.program, closed.program);

        let _ = app.update(Message::OperationGeometry(GeometryMessage::SelectStock));
        let viewport = app.machining_viewport();
        let bounds = iced::Rectangle {
            x: 0.,
            y: 0.,
            width: 1000.,
            height: 700.,
        };
        let mut loops = Vec::new();
        for world in [
            [100., 0., 25.],
            [100., 0., -25.],
            [50., 0., 25.],
            [75., 10., 25.],
        ] {
            let point = picking_view::project(viewport.camera(), bounds, world)
                .unwrap()
                .0;
            let candidates = viewport.cad_candidates_at(point, bounds);
            if candidates.iter().any(|c| c.entity.kind == EntityKind::Loop) {
                loops = candidates;
                break;
            }
        }
        assert!(
            !loops.is_empty(),
            "stock mode must pick the actual model rim"
        );
        let _ = app.update(Message::CadPicked(loops));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(!app.stock_contour_editor(), "{}", app.status);
        assert!(
            matches!(&app.operation_draft, Some(Operation::Pocket(p)) if !p.stock_contours.regions.is_empty())
        );
        let _ = app.update(Message::OperationGeometry(GeometryMessage::SelectStock));
        let _ = app.update(Message::GeometryFilterChanged(CadFilter::Silhouette));
        let viewport = app.machining_viewport();
        assert!(viewport.stock_contour_picking && viewport.cad_silhouette);
        assert!(viewport.cad_operation_kind.is_none());
        let point = picking_view::project(viewport.camera(), bounds, [40., 0., -75.])
            .unwrap()
            .0;
        let candidates = viewport.cad_candidates_at(point, bounds);
        assert!(candidates
            .iter()
            .any(|c| c.entity.kind == EntityKind::Silhouette));
        let _ = app.update(Message::CadPicked(candidates));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(!app.stock_contour_editor(), "{}", app.status);
        assert!(matches!(&app.operation_draft, Some(Operation::Pocket(p))
            if p.stock_contours.source.as_ref().unwrap().recipe == cam_model::ExtractionRecipe::ProfileSilhouette));

        let _ = app.update(Message::OperationGeometry(GeometryMessage::SelectStock));
        let viewport = app.machining_viewport();
        let mut clicked = false;
        for (_, edge) in viewport.setup_geometry.stock_lines() {
            let midpoint = std::array::from_fn(|i| (edge[0][i] + edge[1][i]) / 2.);
            let Some((point, _)) = picking_view::project(viewport.camera(), bounds, midpoint)
            else {
                continue;
            };
            let action = viewport.update(
                &mut ViewportState::default(),
                &iced::Event::Mouse(iced::mouse::Event::ButtonPressed(iced::mouse::Button::Left)),
                bounds,
                iced::mouse::Cursor::Available(point),
            );
            if let Some(action) = action {
                let (message, _, _) = action.into_inner();
                if let Some(message @ Message::OperationGeometry(GeometryMessage::ResetStock)) =
                    message
                {
                    let _ = app.update(message);
                    clicked = true;
                    break;
                }
            }
        }
        assert!(clicked, "stock wireframe must be selectable");
        assert!(app.controller.cad_selection().is_empty());
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(matches!(&app.operation_draft, Some(Operation::Pocket(p))
            if p.stock_contours.enabled && p.stock_contours.source.is_none() && p.stock_contours.regions.is_empty()));
        assert_eq!(app.controller.operation(0), Some(&before));
    }

    /// Read-only benchmark for the supplied local tray; no project is saved.
    #[test]
    #[ignore = "manual supplied-project performance measurement"]
    fn tray_face_selection_benchmark() {
        use std::hint::black_box;
        use std::time::Instant;
        let (mut app, _) = App::new();
        app.settings = crate::Settings::default();
        app.controller = AppController::new(crate::default_machine());
        let start = Instant::now();
        app.library.tools.clear();
        app.open_project(
            &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../testfiles/tray-parallel.ocam"),
        )
        .unwrap();
        let deadline = Instant::now() + std::time::Duration::from_secs(120);
        while app.controller.run_needs_poll() {
            let _ = app.update(Message::PollRun);
            assert!(Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        eprintln!("open project {:?}", start.elapsed());
        app.setup_draft = None;
        let id = app
            .controller
            .inspection_setup()
            .operations
            .iter()
            .find(|op| matches!(op, Operation::Parallel(_)))
            .unwrap()
            .id();
        let start = Instant::now();
        app.begin_operation_draft(id);
        eprintln!("open editor {:?}", start.elapsed());
        let start = Instant::now();
        black_box(app.view());
        eprintln!("first editor view {:?}", start.elapsed());
        app.operation_geometry_message(GeometryMessage::Add(CadFilter::Face));
        let saved_faces = app.controller.cad_selection().to_vec();
        let state = app.controller.geometry_state();
        let instance = &state.instances[0];
        let asset = state.asset(instance.asset_id).unwrap();
        let faces: Vec<_> = asset
            .topology
            .faces
            .iter()
            .filter(|face| face.body_id == instance.body_id)
            .map(|face| EntityRef {
                asset_id: asset.id,
                revision: asset.revision,
                instance_id: instance.id,
                body_id: instance.body_id,
                kind: EntityKind::Face,
                id: face.id,
            })
            .collect();
        for count in [1, 10, faces.len()] {
            app.controller.set_cad_selection(faces[..count].to_vec());
            let start = Instant::now();
            let _ = black_box(app.geometry_settings_valid());
            eprintln!("{count} faces: validation {:?}", start.elapsed());
            let start = Instant::now();
            for _ in 0..3 {
                let _ = app.update(Message::OrbitBy(0.01, 0.005));
                black_box(app.view());
            }
            eprintln!(
                "{count} faces: {:.3} ms/update+view",
                start.elapsed().as_secs_f64() * 1000. / 3.
            );
        }
        app.controller.set_cad_selection(saved_faces.clone());
        for face in saved_faces.iter().take(5).chain(saved_faces.iter().take(5)) {
            let start = Instant::now();
            let _ = app.update(Message::CadSelect(face.clone()));
            eprintln!("toggle face {} {:?}", face.id, start.elapsed());
            let start = Instant::now();
            black_box(app.view());
            eprintln!("view after face toggle {:?}", start.elapsed());
            let start = Instant::now();
            let _ = app.update(Message::CadHover(Some(face.clone())));
            black_box(app.view());
            eprintln!("hover after face toggle {:?}", start.elapsed());
        }
        app.controller.set_cad_selection(saved_faces);
        let start = Instant::now();
        app.operation_geometry_message(GeometryMessage::Confirm);
        eprintln!("confirm faces {:?}", start.elapsed());
        let start = Instant::now();
        let _ = app.update(Message::Apply);
        eprintln!("apply operation {:?}", start.elapsed());
    }

    fn drawing_app() -> App {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample.dxf").unwrap();
        app.controller.select(Selection::Setup);
        app.setup_draft = None;
        app.refresh_fields();
        app
    }

    #[test]
    fn adaptive_geometry_modes_preserve_mixed_body_and_face_selection() {
        if !crate::test_fixture::available("chamfer_test.ocam") {
            return;
        }
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_project(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../testfiles/chamfer_test.ocam"),
            )
            .unwrap();
        app.controller.select(Selection::Setup);
        app.setup_draft = None;
        let geometry = app.controller.geometry_state();
        let instance = &geometry.instances[0];
        let asset = geometry.asset(instance.asset_id).unwrap();
        let body = EntityRef {
            asset_id: asset.id,
            revision: asset.revision,
            instance_id: instance.id,
            body_id: instance.body_id,
            kind: EntityKind::Body,
            id: instance.body_id,
        };
        let face = EntityRef {
            kind: EntityKind::Face,
            id: asset.topology.faces[0].id,
            ..body.clone()
        };
        assert_eq!(
            modes(OpKind::Adaptive, true)
                .into_iter()
                .map(|(filter, _, _)| filter)
                .collect::<Vec<_>>(),
            vec![CadFilter::Body, CadFilter::Face]
        );
        app.cad_pending_kind = Some(OpKind::Adaptive);
        app.controller
            .try_set_adaptive_cad_selection(vec![body.clone()])
            .unwrap();
        let _ = app.update(Message::GeometryFilterChanged(CadFilter::Face));
        assert_eq!(app.cad_filter, CadFilter::Face);
        assert_eq!(app.controller.cad_selection(), std::slice::from_ref(&body));
        app.geometry_additive = true;
        let _ = app.update(Message::CadSelect(face.clone()));
        assert_eq!(app.controller.cad_selection(), &[body, face]);
    }

    #[test]
    fn geometry_tab_auto_selects_supported_adaptive_feature_and_resets_for_idle_view() {
        if !crate::test_fixture::available("chamfer_test.ocam") {
            return;
        }
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_project(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../testfiles/chamfer_test.ocam"),
            )
            .unwrap();
        app.controller.select(Selection::Setup);
        app.setup_draft = None;
        let (body, face, face_bounds, placement) = {
            let geometry = app.controller.geometry_state();
            let instance = &geometry.instances[0];
            let asset = geometry.asset(instance.asset_id).unwrap();
            let face_topology = asset
                .topology
                .faces
                .iter()
                .find(|candidate| {
                    let reference = EntityRef {
                        asset_id: asset.id,
                        revision: asset.revision,
                        instance_id: instance.id,
                        body_id: instance.body_id,
                        kind: EntityKind::Face,
                        id: candidate.id,
                    };
                    app.controller
                        .preview_cad_operation(OpKind::Adaptive, &[reference])
                        .is_ok()
                })
                .expect("fixture has a face with a positive-area setup XY projection");
            (
                EntityRef {
                    asset_id: asset.id,
                    revision: asset.revision,
                    instance_id: instance.id,
                    body_id: instance.body_id,
                    kind: EntityKind::Body,
                    id: instance.body_id,
                },
                EntityRef {
                    asset_id: asset.id,
                    revision: asset.revision,
                    instance_id: instance.id,
                    body_id: instance.body_id,
                    kind: EntityKind::Face,
                    id: face_topology.id,
                },
                face_topology.bounds,
                instance.placement,
            )
        };

        let _ = app.update(Message::BeginOp(OpKind::Adaptive));
        assert_eq!(app.cad_pending_kind, Some(OpKind::Adaptive));
        assert_eq!(app.cad_filter, CadFilter::Body);
        let _ = app.update(Message::OperationTabChanged(OperationTab::Geometry));
        assert!(app.geometry_additive);
        assert!(!app.cad_filter_explicit);

        let _ = app.update(Message::CadSelect(face.clone()));
        assert_eq!(app.cad_filter, CadFilter::Face);
        assert_eq!(app.controller.cad_selection(), std::slice::from_ref(&face));
        assert!(!app.cad_filter_explicit);

        let _ = app.update(Message::CadEscape);
        assert!(app.cad_pending_kind.is_none());
        assert!(!app.cad_filter_explicit);
        let viewport = app.machining_viewport();
        assert!(viewport.cad_operation_kind.is_none());
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(900., 600.));
        let point: [f64; 3] =
            std::array::from_fn(|axis| (face_bounds[0][axis] + face_bounds[1][axis]) / 2.);
        let point = placement.transform_point(point);
        let cursor = picking_view::project(viewport.camera(), bounds, point)
            .unwrap()
            .0;
        let candidates = viewport.cad_candidates_at(cursor, bounds);
        assert!(candidates
            .iter()
            .any(|pick| pick.entity.kind == EntityKind::Face));
        assert!(candidates
            .iter()
            .any(|pick| pick.entity.kind == EntityKind::Body));

        let id = app
            .controller
            .create_cad_operation(OpKind::Adaptive, vec![body.clone()])
            .unwrap();
        app.begin_operation_draft(id);
        let _ = app.update(Message::OperationTabChanged(OperationTab::Geometry));
        assert_eq!(app.cad_editing_op, None, "opening the tab must stay clean");
        assert!(app.geometry_auto_pick);
        assert_eq!(
            app.machining_viewport().cad_operation_kind,
            Some(OpKind::Adaptive)
        );
        assert!(!app.cad_filter_explicit);
        let _ = app.update(Message::GeometryClear);
        let _ = app.update(Message::CadSelect(face.clone()));
        assert_eq!(app.cad_editing_op, Some(id));
        assert_eq!(app.cad_filter, CadFilter::Face);
        let selected = app.controller.cad_selection().to_vec();
        assert_eq!(selected, vec![face]);
        let preview = app
            .controller
            .preview_cad_operation(OpKind::Adaptive, &selected);
        assert!(preview.is_ok(), "{preview:?}");
    }

    #[test]
    fn profile_geometry_auto_pick_edits_a_named_entry_without_legacy_geometry_mode() {
        if !crate::test_fixture::available("chamfer_test.ocam") {
            return;
        }
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_project(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../testfiles/chamfer_test.ocam"),
            )
            .unwrap();
        app.controller.select(Selection::Setup);
        app.setup_draft = None;
        let body = {
            let geometry = app.controller.geometry_state();
            let instance = &geometry.instances[0];
            let asset = geometry.asset(instance.asset_id).unwrap();
            EntityRef {
                asset_id: asset.id,
                revision: asset.revision,
                instance_id: instance.id,
                body_id: instance.body_id,
                kind: EntityKind::Body,
                id: instance.body_id,
            }
        };
        let silhouette = EntityRef {
            kind: EntityKind::Silhouette,
            id: body.body_id,
            ..body.clone()
        };
        let id = app
            .controller
            .create_cad_operation(OpKind::Profile, vec![silhouette.clone()])
            .unwrap();
        app.controller.edit_operation(id, |operation| {
            if let Operation::Profile(profile) = operation {
                profile.selections = Some(Vec::new());
            }
        });
        app.controller.select(Selection::Operation(id));
        app.begin_operation_draft(id);
        let _ = app.update(Message::OperationTabChanged(OperationTab::Geometry));
        assert!(app.geometry_auto_pick);
        assert_eq!(app.cad_editing_op, None);

        let _ = app.update(Message::CadSelect(silhouette.clone()));
        assert!(app
            .profile_entry_editor
            .as_ref()
            .is_some_and(|entry| entry.picking));
        assert_eq!(app.cad_editing_op, None);
        app.profile_entry_message(profile_entries_view::EntryMessage::Confirm);
        assert!(
            app.profile_entry_editor.is_none(),
            "the picked entry must validate and close"
        );
        assert_eq!(app.cad_editing_op, None);
        assert!(app.profile_entries().iter().any(|entry| {
            matches!(&entry.source, cam_model::ProfileGeometrySource::Cad(refs) if refs.contains(&silhouette))
        }));
        let _ = app.update(Message::Apply);
        assert_eq!(app.cad_editing_op, None);
        assert!(
            matches!(app.controller.operation(id), Some(Operation::Profile(profile))
            if profile.selections.as_ref().is_some_and(|entries| entries.iter().any(|entry|
                matches!(&entry.source, cam_model::ProfileGeometrySource::Cad(refs) if refs.contains(&silhouette)))))
        );
    }

    #[test]
    fn cancelling_settings_restores_each_drawing_wizard_without_discarding_other_tabs() {
        let mut app = drawing_app();
        let candidates: Vec<_> = app
            .controller
            .regions()
            .iter()
            .enumerate()
            .flat_map(|(region, p)| {
                std::iter::once(LoopRef::outer(region))
                    .chain((0..p.holes().len()).map(move |hole| LoopRef::hole(region, hole)))
            })
            .collect();
        for kind in [
            OpKind::Chamfer,
            OpKind::Engrave,
            OpKind::Carve,
            OpKind::Thread,
        ] {
            let _ = app.update(Message::BeginOp(kind));
            let source = *candidates
                .iter()
                .find(|reference| {
                    app.controller
                        .planar_operation_eligibility(kind, &[**reference])
                        .is_ok()
                })
                .unwrap();
            app.controller
                .set_pending_operation_geometry(&[source], Some([2., 3.]))
                .unwrap();
            let _ = app.update(Message::FieldChanged(Field::Feed, "431.25".into()));
            let before = app.controller.document().clone();
            let pending = app.controller.pending_op();
            let heights = app.height_draft.clone();
            let _ = app.update(Message::OperationGeometry(GeometryMessage::Edit));
            assert!(app.geometry_editor.is_some());
            let _ = app.inspector();
            let _ = app.update(Message::GeometryClear);
            assert!(app.controller.pending_operation_geometry().is_empty());
            let _ = app.update(Message::CadEscape);
            assert!(app.geometry_editor.is_none());
            assert!(app.creating_operation());
            assert_eq!(app.controller.pending_op(), pending);
            assert_eq!(app.height_draft, heights);
            assert_eq!(app.fields[&Field::Feed], "431.25");
            assert_eq!(app.controller.document(), &before);
            let _ = app.update(Message::CancelOp);
        }
    }

    #[test]
    fn settings_ok_stages_drawing_geometry_and_enter_does_not_create_the_operation() {
        let mut app = drawing_app();
        let _ = app.update(Message::BeginOp(OpKind::Engrave));
        let before = app.controller.document().clone();
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Add(
            CadFilter::Loop,
        )));
        assert!(app.geometry_settings_valid().is_err());
        app.controller
            .set_pending_operation_geometry(&[LoopRef::outer(0)], None)
            .unwrap();
        assert!(app.geometry_settings_valid().is_ok());
        let _ = app.update(Message::Apply);
        assert!(app.geometry_editor.is_none());
        assert!(app.creating_operation());
        assert_eq!(
            app.controller.pending_operation_geometry(),
            vec![LoopRef::outer(0)]
        );
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn committed_thread_settings_restore_sources_and_preserve_edited_nominal_diameter() {
        let mut app = drawing_app();
        let source = app
            .controller
            .regions()
            .iter()
            .enumerate()
            .flat_map(|(region, p)| {
                std::iter::once(LoopRef::outer(region))
                    .chain((0..p.holes().len()).map(move |hole| LoopRef::hole(region, hole)))
            })
            .find(|reference| {
                app.controller
                    .planar_operation_eligibility(OpKind::Thread, &[*reference])
                    .is_ok()
            })
            .unwrap();
        app.controller.begin_operation(OpKind::Thread);
        app.controller.set_pending_tool(1);
        app.controller
            .set_pending_operation_geometry(&[source], None)
            .unwrap();
        let prototype = app.controller.operation_template(OpKind::Thread, 1);
        let operation = app
            .controller
            .operation_with_drawing_geometry(&prototype, &[source])
            .unwrap();
        app.controller.cancel_operation();
        app.controller.add_operation(operation);
        let Selection::Operation(id) = app.controller.selection() else {
            panic!("thread operation");
        };
        app.begin_operation_draft(id);
        let _ = app.update(Message::FieldChanged(Field::MajorDia, "7.5".into()));
        let _ = app.update(Message::FieldChanged(Field::Feed, "245.0".into()));
        let before = app.controller.document().clone();
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Edit));
        assert_eq!(app.controller.pending_operation_geometry(), vec![source]);
        assert!(app.geometry_settings_valid().is_ok());
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(app.geometry_editor.is_none());
        assert!(app.drawing_editing_op.is_none());
        assert_eq!(app.fields[&Field::MajorDia], "7.5");
        assert_eq!(app.fields[&Field::Feed], "245.0");
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn single_chamfer_edge_keeps_editable_width_and_open_preview() {
        if !crate::test_fixture::available("chamfer_test.ocam") {
            return;
        }
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_project(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../testfiles/chamfer_test.ocam"),
            )
            .unwrap();
        app.controller.select(Selection::Setup);
        app.setup_draft = None;
        app.refresh_fields();
        let _ = app.update(Message::BeginOp(OpKind::Chamfer));
        let _ = app.update(Message::FieldChanged(Field::ChamferWidth, "1.5".into()));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Add(
            CadFilter::Edge,
        )));
        let _ = app.update(Message::CadSelect(EntityRef {
            asset_id: 2,
            revision: 1,
            instance_id: 2,
            body_id: 1,
            kind: EntityKind::Edge,
            id: 12,
        }));
        assert!(app.geometry_settings_valid().is_ok(), "{}", app.status);
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(app.geometry_editor.is_none(), "{}", app.status);
        assert_eq!(app.operation_tab, OperationTab::Geometry);
        assert!(app.inspector_fields().contains(&Field::ChamferWidth));
        assert_eq!(
            operation_tabs::field_tab(Field::ChamferWidth),
            OperationTab::Passes
        );
        let (Operation::Chamfer(op), _) = app.creation_operation().unwrap() else {
            panic!("chamfer")
        };
        assert!(!op.closed);
        assert_eq!(op.chain.len(), 2);
        assert_eq!(op.width, 1.5);
        let _ = app.inspector();
        let _ = app.update(Message::FieldChanged(Field::ChamferWidth, "0.75".into()));
        let (Operation::Chamfer(op), _) = app.creation_operation().unwrap() else {
            panic!("chamfer")
        };
        assert_eq!(op.width, 0.75);
        assert!(!op.closed);
    }

    #[test]
    fn supplied_modeled_faces_use_face_icons_both_height_boundaries_and_a_stepdown() {
        if !crate::test_fixture::available("chamfer_test.ocam") {
            return;
        }
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_project(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../testfiles/chamfer_test.ocam"),
            )
            .unwrap();
        app.controller.select(Selection::Setup);
        app.setup_draft = None;
        app.refresh_fields();
        let before = app.controller.document().clone();
        let _ = app.update(Message::BeginOp(OpKind::Chamfer));
        let _ = app.update(Message::FieldChanged(Field::Stepdown, "1.0".into()));
        // An old edge-width buffer must not block face selection or override the modeled width.
        let _ = app.update(Message::FieldChanged(
            Field::ChamferWidth,
            "old edge input".into(),
        ));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Add(
            CadFilter::Face,
        )));
        for id in [2, 3, 4, 5] {
            let _ = app.update(Message::CadSelect(EntityRef {
                asset_id: 2,
                revision: 1,
                instance_id: 2,
                body_id: 1,
                kind: EntityKind::Face,
                id,
            }));
        }
        assert_eq!(app.controller.cad_selection().len(), 4);
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(app.geometry_editor.is_none(), "{}", app.status);
        assert!(app.creation_ready(), "{:?}", app.creation_operation());
        let (Operation::Chamfer(operation), _) = app.creation_operation().unwrap() else {
            panic!("chamfer");
        };
        assert_eq!(operation.faces.len(), 4);
        assert_eq!(operation.width, 10.);
        assert_eq!(operation.stepdown, 1.);
        assert!(app.inspector_fields().contains(&Field::Stepdown));
        assert!(!app.inspector_fields().contains(&Field::ChamferWidth));
        let heights = app.resolve_height_editor(0).unwrap();
        assert_eq!((heights.top, heights.bottom), (50., 40.));
        let preview = app.operation_geometry_preview();
        let z: Vec<_> = preview
            .strips
            .iter()
            .flat_map(|strip| strip.points.iter())
            .map(|p| p[2])
            .collect();
        assert_eq!(z.iter().copied().fold(f32::INFINITY, f32::min), 40.);
        assert_eq!(z.iter().copied().fold(f32::NEG_INFINITY, f32::max), 50.);
        for tab in OperationTab::ALL {
            let _ = app.update(Message::OperationTabChanged(tab));
            let _ = app.inspector();
        }
        assert_eq!(app.controller.document(), &before);
        let _ = app.update(Message::ConfirmOp);
        assert!(!app.creating_operation(), "{}", app.status);
        assert!(app.controller.undo());
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn replacing_a_rim_with_modeled_faces_updates_default_heights_in_the_editor() {
        if !crate::test_fixture::available("chamfer_test.ocam") {
            return;
        }
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_project(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../testfiles/chamfer_test.ocam"),
            )
            .unwrap();
        app.setup_draft = None;
        let tool = app.controller.use_tool(cam_model::Tool {
            number: 10,
            diameter: 6.,
            length: 30.,
            flutes: 2,
            kind: cam_model::ToolKind::ChamferMill {
                included_angle_deg: 90.,
                tip_diameter: 1.,
            },
            ..Default::default()
        });
        let rim = EntityRef {
            asset_id: 2,
            revision: 1,
            instance_id: 2,
            body_id: 1,
            kind: EntityKind::Loop,
            id: 50,
        };
        let id = app
            .controller
            .create_cad_operation_with_tool(OpKind::Chamfer, vec![rim.clone()], tool)
            .unwrap();
        app.begin_operation_draft(id);
        let before = app.controller.document().clone();
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Add(
            CadFilter::Face,
        )));
        let _ = app.update(Message::CadSelect(EntityRef {
            kind: EntityKind::Face,
            id: 2,
            ..rim
        }));
        let _ = app.update(Message::OperationGeometry(GeometryMessage::Confirm));
        assert!(app.geometry_editor.is_none(), "{}", app.status);
        let heights = app.resolve_height_editor(id).unwrap();
        assert_eq!((heights.top, heights.bottom), (50., 40.));
        assert_eq!(app.controller.document(), &before);
    }
}
