//! Named geometry collections and their transactional configuration submenu.
use super::geometry_selection::{
    geometry_collection, selection_hint as hint, selection_icon as icon, GeometryRow,
};
use super::*;
use cam_model::*;
use iced::widget::column;

#[derive(Clone, Debug)]
pub(super) enum EntryMessage {
    Add(ProfileGeometryKind),
    EditSelected,
    RemoveSelected,
    Pick,
    ClearSource,
    Name(String),
    Coplanar(bool),
    Curves(ProfileCurves),
    Side(ProfileSideRule),
    ChainMode(ProfileChainMode),
    IndividualEdges(bool),
    Reverse(bool),
    SetupModels(bool),
    Tolerance(String),
    Confirm,
    Cancel,
}
pub(super) struct EntryEditor {
    pub index: usize,
    pub draft: ProfileGeometryEntry,
    pub picking: bool,
    tolerance: String,
    previous: Option<Vec<ProfileGeometryEntry>>,
    previous_row: Option<usize>,
    previous_filter_explicit: bool,
    error: Option<String>,
}

fn kind_name(kind: ProfileGeometryKind) -> &'static str {
    match kind {
        ProfileGeometryKind::FaceContours => "Face Contours",
        ProfileGeometryKind::Chain => "Chain",
        ProfileGeometryKind::Silhouette => "Silhouette",
    }
}
fn kind_icon(kind: ProfileGeometryKind) -> &'static str {
    match kind {
        ProfileGeometryKind::FaceContours => "face",
        ProfileGeometryKind::Chain => "chain",
        ProfileGeometryKind::Silhouette => "silhouette",
    }
}

impl App {
    /// Direct picks and named entries share geometry extraction and advisory verification.
    pub(super) fn active_profile_geometry_entries(&self) -> Option<Vec<ProfileGeometryEntry>> {
        let Some(Operation::Profile(profile)) = &self.operation_draft else {
            return None;
        };
        let selected = self.controller.cad_selection();
        let direct_selection =
            self.cad_pending_kind == Some(OpKind::Profile) && !selected.is_empty();
        (profile.selections.is_some() || direct_selection).then(|| self.preview_profile_entries())
    }

    pub(super) fn validate_active_profile_geometry(&self) -> Option<Result<(), String>> {
        let entries = self.active_profile_geometry_entries()?;
        let Some(Operation::Profile(mut profile)) = self.operation_draft.clone() else {
            return None;
        };
        profile.selections = Some(entries);
        Some(self.resolve_profile_geometry(&mut profile).map(|_| ()))
    }

    pub(super) fn profile_entries(&self) -> Vec<ProfileGeometryEntry> {
        let Some(Operation::Profile(profile)) = &self.operation_draft else {
            return Vec::new();
        };
        if let Some(entries) = &profile.selections {
            return entries.clone();
        }
        let refs = if self.cad_pending_kind.is_some() || self.cad_editing_op.is_some() {
            self.controller.cad_selection().to_vec()
        } else {
            self.operation_binding_preview
                .as_ref()
                .or_else(|| self.controller.geometry_state().bindings.get(&profile.id))
                .map(|b| b.selected.clone())
                .unwrap_or_default()
        };
        let mut entries = crate::cad_operations::profile_entries_from_selection(&refs);
        for entry in &mut entries {
            // Preserve the older envelope-only recipe when opening its settings.
            if entry.kind == ProfileGeometryKind::Silhouette
                && self
                    .controller
                    .geometry_state()
                    .bindings
                    .get(&profile.id)
                    .is_some_and(|binding| {
                        matches!(binding.recipe, ExtractionRecipe::ProfileLoop { .. })
                    })
            {
                entry.curves = ProfileCurves::Outer;
            }
        }
        if entries.is_empty() && (self.cad_pending_kind.is_some() || self.cad_editing_op.is_some())
        {
            return entries;
        }
        if entries.is_empty() {
            let contours = if self.controller.pending_op().is_some() {
                self.controller
                    .pending_operation_geometry()
                    .into_iter()
                    .filter_map(|r| {
                        self.controller
                            .loop_points(r)
                            .map(|(points, closed)| ProfileContour {
                                chain: cam_geo::Contour::new(points),
                                closed,
                                side: if matches!(r.part, crate::LoopPart::Hole(_)) {
                                    Side::Inside
                                } else {
                                    Side::Outside
                                },
                            })
                    })
                    .collect()
            } else {
                profile.contours().collect::<Vec<_>>()
            };
            for contour in contours {
                let mut entry = ProfileGeometryEntry::new(
                    ProfileGeometryKind::Chain,
                    format!(
                        "{} Chain {}",
                        if contour.closed { "Closed" } else { "Open" },
                        entries.len() + 1
                    ),
                );
                entry.chain_mode = if contour.closed {
                    ProfileChainMode::Closed
                } else {
                    ProfileChainMode::Open
                };
                entry.source = ProfileGeometrySource::Contours(vec![contour]);
                entries.push(entry);
            }
        } else if !self.creating_operation() {
            let saved: Vec<_> = profile.contours().collect();
            if let Some(resolved) = self.cached_profile_contours(&entries) {
                for (entry, result) in entries.iter_mut().zip(resolved.iter()) {
                    if let Ok((contours, _)) = result {
                        for (index, contour) in contours.iter().enumerate() {
                            if let Some(old) = saved
                                .iter()
                                .find(|p| p.chain == contour.chain && p.closed == contour.closed)
                            {
                                if old.side != contour.side {
                                    entry.side_overrides.insert(index, old.side);
                                }
                            }
                        }
                    }
                }
            }
        }
        entries
    }
    pub(super) fn preview_profile_entries(&self) -> Vec<ProfileGeometryEntry> {
        let mut entries = self.profile_entries();
        if let Some(editor) = &self.profile_entry_editor {
            if let Some(entry) = entries.get_mut(editor.index) {
                *entry = editor.draft.clone();
            }
        }
        entries
    }
    fn setup_profile_bodies(&self) -> Vec<EntityRef> {
        self.controller
            .geometry_state()
            .instances
            .iter()
            .filter(|i| {
                self.controller
                    .inspection_setup()
                    .model_instances
                    .contains(&i.id)
            })
            .filter_map(|instance| {
                let asset = self.controller.geometry_state().asset(instance.asset_id)?;
                Some(EntityRef {
                    asset_id: asset.id,
                    revision: asset.revision,
                    instance_id: instance.id,
                    body_id: instance.body_id,
                    kind: EntityKind::Silhouette,
                    id: instance.body_id,
                })
            })
            .collect()
    }
    pub(super) fn profile_entry_message(&mut self, message: EntryMessage) {
        match message {
            EntryMessage::Add(kind) => {
                if self.profile_entry_editor.is_some() {
                    return;
                }
                self.profile_chain_edges = false;
                let previous = match &self.operation_draft {
                    Some(Operation::Profile(p)) => p.selections.clone(),
                    _ => None,
                };
                let previous_row = self.geometry_row_selection;
                let mut entries = self.profile_entries();
                let index = entries.len();
                let mut entry =
                    ProfileGeometryEntry::new(kind, format!("{} {}", kind_name(kind), index + 1));
                if kind == ProfileGeometryKind::Chain {
                    entry.chain_mode = ProfileChainMode::Closed;
                }
                if kind == ProfileGeometryKind::Silhouette {
                    entry.source = ProfileGeometrySource::Cad(self.setup_profile_bodies());
                }
                entries.push(entry.clone());
                self.edit_operation_draft(|op| {
                    if let Operation::Profile(p) = op {
                        p.selections = Some(entries);
                    }
                });
                self.geometry_row_selection = Some(index);
                self.profile_entry_editor = Some(EntryEditor {
                    index,
                    draft: entry,
                    picking: false,
                    tolerance: "0.010".into(),
                    previous,
                    previous_row,
                    previous_filter_explicit: self.cad_filter_explicit,
                    error: None,
                });
                self.operation_geometry = None;
                self.operation_binding_preview = None;
                self.profile_entry_start_pick();
            }
            EntryMessage::EditSelected => {
                if self.profile_entry_editor.is_some() {
                    return;
                }
                let previous = match &self.operation_draft {
                    Some(Operation::Profile(p)) => p.selections.clone(),
                    _ => None,
                };
                let entries = self.profile_entries();
                let index = self
                    .geometry_row_selection
                    .filter(|i| *i < entries.len())
                    .unwrap_or(0);
                let previous_row = self.geometry_row_selection;
                if let Some(entry) = entries.get(index).cloned() {
                    self.profile_chain_edges = entry.kind == ProfileGeometryKind::Chain
                        && (entry.chain_mode == ProfileChainMode::Open
                            || matches!(&entry.source, ProfileGeometrySource::Cad(refs)
                                if refs.iter().any(|reference| reference.kind == EntityKind::Edge)));
                    let tolerance = fmt_num(entry.tolerance);
                    self.edit_operation_draft(|op| {
                        if let Operation::Profile(p) = op {
                            p.selections = Some(entries);
                        }
                    });
                    self.profile_entry_editor = Some(EntryEditor {
                        index,
                        draft: entry,
                        picking: false,
                        tolerance,
                        previous,
                        previous_row,
                        previous_filter_explicit: self.cad_filter_explicit,
                        error: None,
                    });
                    self.operation_geometry = None;
                    self.operation_binding_preview = None;
                }
            }
            EntryMessage::RemoveSelected => {
                if self.profile_entry_editor.is_some() {
                    return;
                }
                let mut entries = self.profile_entries();
                let index = self.geometry_row_selection.unwrap_or(0);
                let before = self.operation_draft.clone();
                let removed = index < entries.len();
                if index < entries.len() {
                    entries.remove(index);
                    self.geometry_row_selection = if entries.is_empty() {
                        None
                    } else {
                        Some(index.min(entries.len() - 1))
                    };
                }
                self.edit_operation_draft(|op| {
                    if let Operation::Profile(p) = op {
                        p.selections = Some(entries);
                    }
                });
                self.operation_geometry = None;
                self.operation_binding_preview = None;
                self.profile_side_overrides.clear();
                self.sync_profile_entry_cache();
                if removed && !self.creating_operation() {
                    self.refresh_geometry_height_defaults(before.as_ref());
                }
            }
            EntryMessage::Pick => self.profile_entry_start_pick(),
            EntryMessage::Confirm => {
                let Some(mut editor) = self.profile_entry_editor.take() else {
                    return;
                };
                match editor.tolerance.trim().parse::<f64>() {
                    Ok(value) if value.is_finite() && value > 0. => editor.draft.tolerance = value,
                    _ => {
                        editor.error =
                            Some("Enter a positive, finite tolerance in millimetres.".into());
                        self.profile_entry_editor = Some(editor);
                        return;
                    }
                }
                if editor.draft.name.trim().is_empty() {
                    editor.draft.name =
                        format!("{} {}", kind_name(editor.draft.kind), editor.index + 1);
                }
                let result = self.indexed_draft_geometry().and_then(|g| {
                    cam_geometry3d::profile_entry_contours(&g, &editor.draft).map(|_| ())
                });
                if let Err(error) = result {
                    self.report_error(format!(
                        "Profile geometry entry '{}' rejected: {error}",
                        editor.draft.name
                    ));
                    editor.error = Some(error);
                    self.profile_entry_editor = Some(editor);
                    return;
                }
                let before = self.operation_draft.clone();
                let changed = self
                    .profile_entries()
                    .get(editor.index)
                    .is_none_or(|entry| entry.source != editor.draft.source);
                self.edit_operation_draft(|op| {
                    if let Operation::Profile(p) = op {
                        if let Some(entry) = p
                            .selections
                            .as_mut()
                            .and_then(|entries| entries.get_mut(editor.index))
                        {
                            *entry = editor.draft;
                        }
                    }
                });
                self.cad_filter_explicit = editor.previous_filter_explicit;
                self.geometry_additive = false;
                self.profile_side_overrides.clear();
                self.sync_profile_entry_cache();
                if changed && !self.creating_operation() {
                    self.refresh_geometry_height_defaults(before.as_ref());
                }
                self.refresh_operation_fields();
            }
            EntryMessage::Cancel => {
                if let Some(editor) = self.profile_entry_editor.take() {
                    self.geometry_row_selection = editor.previous_row;
                    self.edit_operation_draft(|op| {
                        if let Operation::Profile(p) = op {
                            p.selections = editor.previous;
                        }
                    });
                    self.cad_filter_explicit = editor.previous_filter_explicit;
                }
                self.geometry_additive = false;
            }
            EntryMessage::SetupModels(enabled) => {
                let bodies = enabled.then(|| self.setup_profile_bodies());
                if let Some(editor) = &mut self.profile_entry_editor {
                    editor.draft.use_setup_models = enabled;
                    if let Some(bodies) = bodies {
                        editor.draft.source = ProfileGeometrySource::Cad(bodies);
                    }
                    editor.error = None;
                }
            }
            message => {
                if let Some(editor) = &mut self.profile_entry_editor {
                    editor.error = None;
                    match message {
                        EntryMessage::Name(name) => editor.draft.name = name,
                        EntryMessage::ClearSource => {
                            editor.draft.source = ProfileGeometrySource::Cad(Vec::new())
                        }
                        EntryMessage::Coplanar(value) => editor.draft.coplanar = value,
                        EntryMessage::Curves(value) => {
                            editor.draft.curves = value;
                            editor.draft.side_overrides.clear();
                        }
                        EntryMessage::Side(value) => {
                            editor.draft.side_rule = value;
                            editor.draft.side_overrides.clear();
                        }
                        EntryMessage::ChainMode(value) => {
                            editor.draft.chain_mode = value;
                            if value == ProfileChainMode::Open {
                                self.profile_chain_edges = true;
                            }
                        }
                        EntryMessage::IndividualEdges(value) => self.profile_chain_edges = value,
                        EntryMessage::Reverse(value) => editor.draft.reversed = value,
                        EntryMessage::Tolerance(value) => editor.tolerance = value,
                        _ => {}
                    }
                }
            }
        }
    }
    fn profile_entry_start_pick(&mut self) {
        if let Some(editor) = &mut self.profile_entry_editor {
            editor.picking = true;
            self.profile_selection_mode = match editor.draft.kind {
                ProfileGeometryKind::FaceContours => ProfileSelectionMode::FaceContours,
                ProfileGeometryKind::Chain => ProfileSelectionMode::Chain,
                ProfileGeometryKind::Silhouette => ProfileSelectionMode::Silhouette,
            };
            if editor.draft.kind != ProfileGeometryKind::Chain {
                self.profile_chain_edges = false;
            }
            self.cad_filter_explicit = true;
            self.profile_tab_picking = false;
            self.geometry_additive = true;
            self.status = format!(
                "{}: select geometry in the viewport, then OK to keep this entry.",
                kind_name(editor.draft.kind)
            );
        }
    }
    pub(super) fn profile_entry_cad_pick(&mut self, entity: EntityRef) -> bool {
        let Some(editor) = self.profile_entry_editor.as_mut().filter(|e| e.picking) else {
            return false;
        };
        let expected = match editor.draft.kind {
            ProfileGeometryKind::FaceContours => EntityKind::Face,
            ProfileGeometryKind::Silhouette => EntityKind::Silhouette,
            ProfileGeometryKind::Chain => entity.kind,
        };
        if entity.kind != expected {
            self.status = format!(
                "Select {} geometry for this entry.",
                kind_name(editor.draft.kind)
            );
            return true;
        }
        if !self.controller.geometry_state().entity_exists(&entity) {
            self.status = "Source geometry is missing or stale.".into();
            return true;
        }
        if editor.draft.kind == ProfileGeometryKind::Silhouette && editor.draft.use_setup_models {
            editor.draft.source = ProfileGeometrySource::Cad(Vec::new());
        }
        let refs = match &mut editor.draft.source {
            ProfileGeometrySource::Cad(refs) => refs,
            _ => {
                editor.draft.source = ProfileGeometrySource::Cad(Vec::new());
                let ProfileGeometrySource::Cad(refs) = &mut editor.draft.source else {
                    unreachable!()
                };
                refs
            }
        };
        if editor.draft.kind == ProfileGeometryKind::Chain
            && refs.first().is_some_and(|old| old.kind != entity.kind)
        {
            refs.clear();
        }
        if let Some(index) = refs.iter().position(|old| *old == entity) {
            refs.remove(index);
        } else {
            refs.push(entity);
        }
        editor.draft.use_setup_models = false;
        editor.draft.side_overrides.clear();
        editor.error = None;
        true
    }
    pub(super) fn profile_entry_drawing_pick(&mut self, world: [f32; 2], aperture: f32) -> bool {
        if !self
            .profile_entry_editor
            .as_ref()
            .is_some_and(|e| e.picking)
        {
            return false;
        }
        let hit = self
            .controller
            .nearest_drawing_feature(world.map(f64::from), aperture as f64)
            .and_then(|(r, _)| {
                self.controller
                    .loop_points(r)
                    .map(|(points, closed)| (r, points, closed))
            });
        let Some((reference, points, closed)) = hit else {
            return true;
        };
        let editor = self.profile_entry_editor.as_mut().unwrap();
        if editor.draft.kind != ProfileGeometryKind::Chain {
            self.status = "Select a CAD face or body for this entry.".into();
            return true;
        }
        let contour = ProfileContour {
            chain: cam_geo::Contour::new(points),
            closed,
            side: if matches!(reference.part, crate::LoopPart::Hole(_)) {
                Side::Inside
            } else {
                Side::Outside
            },
        };
        editor.draft.source = ProfileGeometrySource::Contours(vec![contour]);
        editor.draft.side_overrides.clear();
        editor.error = None;
        true
    }

    pub(super) fn profile_geometry_collection(&self) -> Element<'_, Message> {
        let entries = self.profile_entries();
        let selected = self
            .geometry_row_selection
            .filter(|i| *i < entries.len())
            .or_else(|| (!entries.is_empty()).then_some(0));
        let rows = entries
            .iter()
            .enumerate()
            .map(|(index, entry)| {
                let source_empty = match &entry.source {
                    ProfileGeometrySource::Cad(refs) => refs.is_empty(),
                    ProfileGeometrySource::Contours(c) => c.is_empty(),
                };
                GeometryRow {
                    icon: kind_icon(entry.kind),
                    label: if source_empty {
                        format!("{} · Select geometry", entry.name)
                    } else {
                        entry.name.clone()
                    },
                    action: self
                        .profile_entry_editor
                        .as_ref()
                        .is_none_or(|e| e.index == index)
                        .then_some(Message::GeometryRowSelected(index)),
                }
            })
            .collect();
        let editing = self.profile_entry_editor.is_some();
        let mut toolbar = row![].spacing(4).align_y(Alignment::Center);
        for kind in [
            ProfileGeometryKind::FaceContours,
            ProfileGeometryKind::Chain,
            ProfileGeometryKind::Silhouette,
        ] {
            let available = !editing
                && (kind == ProfileGeometryKind::Chain
                    || !self.controller.geometry_state().instances.is_empty());
            let description = match kind {
                ProfileGeometryKind::FaceContours => "Select a planar face to include its outer boundary and holes. Optionally include coplanar faces.",
                ProfileGeometryKind::Chain => "Select an open or closed contour and configure its direction and cutter side. Use Closed mode for a planar loop along rounded walls; set its cutting depth in Heights.",
                ProfileGeometryKind::Silhouette => "Select bodies to use their outlines and through openings viewed along the setup Z axis.",
            };
            toolbar = toolbar.push(hint(
                button(icon(kind_icon(kind), 28.))
                    .padding(4)
                    .style(inspector_secondary)
                    .on_press_maybe(
                        available.then_some(Message::ProfileEntry(EntryMessage::Add(kind))),
                    ),
                format!("Add {}. {description}", kind_name(kind)),
                self.tooltips,
            ));
        }
        toolbar = toolbar.push(hint(
            button(icon("remove", 28.))
                .padding(4)
                .style(inspector_secondary)
                .on_press_maybe(
                    (!editing && selected.is_some())
                        .then_some(Message::ProfileEntry(EntryMessage::RemoveSelected)),
                ),
            "Delete the selected geometry entry.",
            self.tooltips,
        ));
        toolbar = toolbar.push(hint(
            button(icon("settings", 28.))
                .padding(4)
                .style(inspector_secondary)
                .on_press_maybe(
                    (!editing && selected.is_some())
                        .then_some(Message::ProfileEntry(EntryMessage::EditSelected)),
                ),
            "Configure the selected geometry entry.",
            self.tooltips,
        ));
        geometry_collection(
            "Contour selection",
            rows,
            selected,
            Some(Message::ProfileEntry(if selected.is_some() {
                EntryMessage::EditSelected
            } else {
                EntryMessage::Add(ProfileGeometryKind::Chain)
            })),
            toolbar,
        )
    }

    pub(super) fn profile_entry_settings<'a>(
        &'a self,
        editor: &'a EntryEditor,
    ) -> Element<'a, Message> {
        let entry = &editor.draft;
        let mut body = column![
            row![
                icon(kind_icon(entry.kind), 24.),
                text(format!("{} settings", kind_name(entry.kind))).size(15)
            ]
            .spacing(8)
            .align_y(Alignment::Center),
            row![
                text("Name").size(12).width(90),
                inspector_input("Entry name", &entry.name)
                    .on_input(|v| Message::ProfileEntry(EntryMessage::Name(v)))
                    .width(Length::Fill)
            ]
            .spacing(6),
            row![
                text(if entry.kind == ProfileGeometryKind::Silhouette {
                    "Bodies"
                } else {
                    "Selection"
                })
                .size(12)
                .width(90),
                inspector_pick_button(
                    if editor.picking {
                        "Selecting…"
                    } else {
                        "Select"
                    },
                    editor.picking
                )
                .on_press(Message::ProfileEntry(EntryMessage::Pick)),
                inspector_button("Clear")
                    .on_press(Message::ProfileEntry(EntryMessage::ClearSource))
            ]
            .spacing(6)
            .align_y(Alignment::Center)
        ]
        .spacing(9);
        let labels = match &entry.source {
            ProfileGeometrySource::Cad(refs) => cad_geometry_labels(refs),
            ProfileGeometrySource::Contours(contours) => contours
                .iter()
                .enumerate()
                .map(|(index, contour)| {
                    format!(
                        "{} contour {}",
                        if contour.closed { "Closed" } else { "Open" },
                        index + 1
                    )
                })
                .collect(),
        };
        body = body.push(geometry_selection::selected_feature_names(labels));
        if entry.kind == ProfileGeometryKind::FaceContours {
            body = body.push(hint(
                checkbox(entry.coplanar)
                    .label("Select coplanar faces")
                    .size(15)
                    .text_size(12)
                    .on_toggle(|v| Message::ProfileEntry(EntryMessage::Coplanar(v))),
                "Include upward planar faces on the same plane in each selected body.",
                self.tooltips,
            ));
        }
        if entry.kind == ProfileGeometryKind::Chain {
            if !self.controller.geometry_state().instances.is_empty() {
                body = body.push(
                    checkbox(self.profile_chain_edges)
                        .label("Select individual edges")
                        .size(15)
                        .text_size(12)
                        .on_toggle(|value| {
                            Message::ProfileEntry(EntryMessage::IndividualEdges(value))
                        }),
                );
            }
            let mut modes = row![text("Mode").size(12).width(90)]
                .spacing(6)
                .align_y(Alignment::Center);
            for (mode, name, label) in [
                (ProfileChainMode::Closed, "closed", "Closed chain: use a closed planar boundary. Along rounded walls, the operation's Top and Bottom heights define the cut depth."),
                (ProfileChainMode::Open, "open", "Open chain: follow planar edges at the operation's Top and Bottom heights, including edges beside rounded surfaces. The cut extends the selected curve vertically."),
            ] {
                let active = entry.chain_mode == mode
                    || (entry.chain_mode == ProfileChainMode::Auto
                        && mode == ProfileChainMode::Closed);
                modes = modes.push(hint(
                    button(icon(name, 27.))
                        .padding(4)
                        .style(move |theme, status| inspector_tab(theme, status, active))
                        .on_press(Message::ProfileEntry(EntryMessage::ChainMode(mode))),
                    label, self.tooltips));
            }
            body=body.push(modes).push(hint(checkbox(entry.reversed).label("Reverse direction").size(15).text_size(12).on_toggle(|v|Message::ProfileEntry(EntryMessage::Reverse(v))),"Reverse this chain's selected direction. Open-chain left and right follow that direction.", self.tooltips));
        } else {
            if entry.kind == ProfileGeometryKind::Silhouette {
                body=body.push(hint(checkbox(entry.use_setup_models).label("Use setup models").size(15).text_size(12).on_toggle(|v|Message::ProfileEntry(EntryMessage::SetupModels(v))),"Use the bodies currently included in the active setup. Select bodies manually to customize the collection.", self.tooltips));
                body = body.push(
                    row![
                        text("Tolerance (mm)").size(12).width(90),
                        inspector_input("0.010", &editor.tolerance)
                            .on_input(|v| Message::ProfileEntry(EntryMessage::Tolerance(v)))
                            .width(Length::Fill)
                    ]
                    .spacing(6),
                );
            }
            let mut filters = row![text("Curves").size(12).width(90)]
                .spacing(6)
                .align_y(Alignment::Center);
            for (value, name, label) in [
                (ProfileCurves::All, "all", "All contours"),
                (ProfileCurves::Outer, "outer", "Outer contours only"),
                (ProfileCurves::Inner, "inner", "Inner contours only"),
            ] {
                let active = entry.curves == value;
                filters = filters.push(hint(
                    button(icon(name, 27.))
                        .padding(4)
                        .style(move |theme, status| inspector_tab(theme, status, active))
                        .on_press(Message::ProfileEntry(EntryMessage::Curves(value))),
                    label,
                    self.tooltips,
                ));
            }
            body = body.push(filters);
        }
        const SIDES: [ProfileSideRule; 5] = [
            ProfileSideRule::StartOutside,
            ProfileSideRule::StartInside,
            ProfileSideRule::AlwaysOutside,
            ProfileSideRule::AlwaysInside,
            ProfileSideRule::On,
        ];
        body = body.push(profile_picker(
            "Side",
            illustrated_help::PROFILE_ENTRY_SIDES,
            self.tooltips,
            entry.side_rule,
            &SIDES,
            |v| Message::ProfileEntry(EntryMessage::Side(v)),
        ));
        if let Some(error) = &editor.error {
            body = body.push(text(error).size(11).color(palette::WARN));
        }
        if let Some(info) = self.cached_profile_entry_verification(entry) {
            body = body.push(text(format!("Info: Automatic verification: {info}. The selected geometry and operation settings are used."))
                .size(11).color(INSPECTOR_MUTED));
        }
        let footer = inspector_footer(
            row![
                inspector_button("Cancel").on_press(Message::ProfileEntry(EntryMessage::Cancel)),
                Space::new().width(Length::Fill),
                inspector_button("OK")
                    .style(inspector_primary)
                    .on_press(Message::ProfileEntry(EntryMessage::Confirm))
            ]
            .spacing(8),
        );
        column![
            scrollable(body.padding(14))
                .width(Length::Fill)
                .height(Length::Fill),
            footer
        ]
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
    }
}

impl App {
    fn sync_profile_entry_cache(&mut self) {
        if let Some((resolved, _)) = self.profile_selection_geometry() {
            self.edit_operation_draft(|op| {
                if let Operation::Profile(p) = op {
                    p.chain = resolved.chain;
                    p.closed = resolved.closed;
                    p.side = resolved.side;
                    p.additional_chains = resolved.additional_chains;
                    p.selections = resolved.selections;
                }
            });
        }
    }
    pub(super) fn refresh_geometry_height_defaults(&mut self, before: Option<&Operation>) {
        let Some(before) = before else { return };
        let old =
            crate::cad_operations::selection_height_defaults(self.controller.document(), before);
        let defaults = OperationHeightSettings::default();
        if let Some(settings) = &mut self.height_draft {
            for plane in HeightPlaneKind::ALL {
                let definition = old.definition(plane);
                if settings.definition(plane) == definition
                    && self
                        .height_offsets
                        .get(&plane)
                        .and_then(|v| v.parse::<f64>().ok())
                        == Some(definition.offset)
                {
                    *settings.definition_mut(plane) = defaults.definition(plane).clone();
                    self.height_offsets
                        .insert(plane, fmt_num(defaults.definition(plane).offset));
                }
            }
        }
    }
    pub(super) fn set_profile_entry_contour_side(&mut self, index: usize, side: Side) {
        if let Some(entries) = self.active_profile_geometry_entries() {
            if matches!(&self.operation_draft, Some(Operation::Profile(p)) if p.selections.is_none())
            {
                self.edit_operation_draft(|operation| {
                    if let Operation::Profile(profile) = operation {
                        profile.selections = Some(entries);
                    }
                });
            }
        }
        let Ok(geometry) = self.indexed_draft_geometry() else {
            return;
        };
        let mut offset = 0;
        for (entry_index, entry) in self.preview_profile_entries().iter().enumerate() {
            let Ok((contours, _)) = cam_geometry3d::profile_entry_contours(&geometry, entry) else {
                continue;
            };
            if index < offset + contours.len() {
                let local = index - offset;
                if let Some(editor) = &mut self.profile_entry_editor {
                    if editor.index != entry_index {
                        return;
                    }
                    editor.draft.side_overrides.insert(local, side);
                } else {
                    self.edit_operation_draft(|op| {
                        if let Operation::Profile(p) = op {
                            if let Some(entry) = p
                                .selections
                                .as_mut()
                                .and_then(|entries| entries.get_mut(entry_index))
                            {
                                entry.side_overrides.insert(local, side);
                            }
                        }
                    });
                }
                self.profile_contour_selection = index;
                self.geometry_row_selection = Some(entry_index);
                self.status = format!("{}: contour {} uses {side}.", entry.name, local + 1);
                if self.profile_entry_editor.is_none() {
                    self.sync_profile_entry_cache();
                }
                return;
            }
            offset += contours.len();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cad_loop_app(file: &str, loop_id: u64) -> (App, EntityRef) {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_project(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../testfiles")
                    .join(file),
            )
            .unwrap();
        app.controller.select(Selection::Setup);
        app.setup_draft = None;
        app.operation_draft = None;
        app.refresh_fields();
        let geometry = app.controller.geometry_state();
        let instance = &geometry.instances[0];
        let asset = geometry.asset(instance.asset_id).unwrap();
        let reference = EntityRef {
            asset_id: asset.id,
            revision: asset.revision,
            instance_id: instance.id,
            body_id: instance.body_id,
            kind: EntityKind::Loop,
            id: loop_id,
        };
        (app, reference)
    }

    fn check_direct_loop_creation(file: &str, loop_id: u64, feature_first: bool) {
        let (mut app, reference) = cad_loop_app(file, loop_id);
        assert!(
            cam_geometry3d::resolve_selection(
                &app.controller.setup_geometry_state().unwrap(),
                std::slice::from_ref(&reference),
                ExtractionRecipe::ProfileContours,
                0.
            )
            .is_err(),
            "the fixture must exercise a loop without an automatic wall-depth certificate"
        );
        if feature_first {
            let _ = app.update(Message::CadSelect(reference.clone()));
        }
        let _ = app.update(Message::BeginOp(OpKind::Profile));
        if !feature_first {
            let _ = app.update(Message::CadSelect(reference.clone()));
        }
        assert!(app.profile_entry_editor.is_none());
        assert!(
            app.profile_selection_geometry().is_some(),
            "source contour preview must work"
        );
        let (Operation::Profile(profile), Some(binding)) = app.creation_operation().unwrap() else {
            panic!("linked profile")
        };
        assert!(profile.closed);
        assert_eq!(binding.selected, vec![reference]);
        assert_eq!(
            profile.selections.unwrap()[0].chain_mode,
            ProfileChainMode::Closed
        );
        app.set_profile_contour_side(0, Side::On);
        let (Operation::Profile(profile), _) = app.creation_operation().unwrap() else {
            panic!("profile")
        };
        assert_eq!(
            profile.side,
            Side::On,
            "direct loop cutter-side edits must survive resolution"
        );
        app.log_geometry_validation();
        assert!(
            app.output_last_geometry_error.is_none(),
            "{:?}",
            app.output_last_geometry_error
        );
        assert!(
            !app.output_content
                .text()
                .contains("Geometry validation error"),
            "{}",
            app.output_content.text()
        );
        let _ = app.update(Message::ConfirmOp);
        assert!(!app.creating_operation(), "{}", app.status);
        let Selection::Operation(id) = app.controller.selection() else {
            panic!("operation")
        };
        assert!(
            matches!(app.controller.operation(id), Some(Operation::Profile(p)) if p.side == Side::On)
        );
    }

    #[test]
    fn direct_closed_loop_picks_use_chain_resolution_in_both_creation_orders() {
        if !crate::test_fixture::available("chamfer_test.ocam") {
            return;
        }
        for feature_first in [false, true] {
            check_direct_loop_creation("chamfer_test.ocam", 50, feature_first);
        }
    }

    #[test]
    fn confirmed_named_chain_does_not_log_a_legacy_wall_error() {
        if !crate::test_fixture::available("chamfer_test.ocam") {
            return;
        }
        let (mut app, reference) = cad_loop_app("chamfer_test.ocam", 50);
        let _ = app.update(Message::BeginOp(OpKind::Profile));
        app.profile_entry_message(EntryMessage::Add(ProfileGeometryKind::Chain));
        assert!(app.profile_entry_cad_pick(reference.clone()));
        // A previous inspection pick may still exist independently of entries.
        app.controller.set_cad_selection(vec![reference]);
        let _ = app.update(Message::ProfileEntry(EntryMessage::Confirm));
        assert!(app.profile_entry_editor.is_none(), "{}", app.status);
        assert!(app.creation_operation().is_ok());
        assert!(
            app.output_last_geometry_error.is_none(),
            "{:?}",
            app.output_last_geometry_error
        );
        assert!(
            !app.output_content
                .text()
                .contains("Geometry validation error"),
            "{}",
            app.output_content.text()
        );
    }

    fn select_closed_edges(mut app: App, references: Vec<EntityRef>) -> App {
        let _ = app.update(Message::BeginOp(OpKind::Profile));
        let _ = app.update(Message::ProfileEntry(EntryMessage::Add(
            ProfileGeometryKind::Chain,
        )));
        let _ = app.update(Message::ProfileEntry(EntryMessage::IndividualEdges(true)));
        assert_eq!(app.machining_viewport().cad_filter, EntityKind::Edge);
        for reference in &references {
            let _ = app.update(Message::CadSelect(reference.clone()));
            assert!(
                app.output_last_geometry_error.is_none(),
                "building a chain is not a verification error"
            );
        }
        let selected = app.profile_entry_editor.as_ref().unwrap().draft.clone();
        let geometry = app.controller.setup_geometry_state().unwrap();
        let (contours, _) = cam_geometry3d::profile_entry_contours(&geometry, &selected).unwrap();
        assert!(contours[0].closed);
        assert!(cam_geometry3d::verify_profile_entry(&geometry, &selected).is_some());
        let _ = app.update(Message::ProfileEntry(EntryMessage::ChainMode(
            ProfileChainMode::Open,
        )));
        let open = app.profile_entry_editor.as_ref().unwrap().draft.clone();
        let (open_contours, _) = cam_geometry3d::profile_entry_contours(&geometry, &open).unwrap();
        assert!(!open_contours[0].closed);
        let _ = app.update(Message::ProfileEntry(EntryMessage::ChainMode(
            ProfileChainMode::Closed,
        )));
        assert!(
            app.profile_chain_edges,
            "chain mode must not change the picking granularity"
        );
        assert_eq!(app.machining_viewport().cad_filter, EntityKind::Edge);
        let _ = app.update(Message::ProfileEntry(EntryMessage::Pick));
        assert!(app.profile_chain_edges);
        let _ = app.update(Message::ProfileEntry(EntryMessage::Confirm));
        assert!(app.profile_entry_editor.is_none(), "{}", app.status);
        let (Operation::Profile(profile), Some(binding)) = app.creation_operation().unwrap() else {
            panic!("profile")
        };
        assert!(profile.closed);
        assert_eq!(binding.selected, references);
        assert!(app
            .output_content
            .text()
            .contains("Geometry verification (info)"));
        let _ = app.update(Message::ConfirmOp);
        assert!(!app.creating_operation(), "{}", app.status);
        app
    }

    #[test]
    fn individual_edges_can_make_a_closed_chain_without_an_upward_face_certificate() {
        if !crate::test_fixture::available("chamfer_test.ocam") {
            return;
        }
        let (app, reference) = cad_loop_app("chamfer_test.ocam", 53);
        let asset = app
            .controller
            .geometry_state()
            .asset(reference.asset_id)
            .unwrap();
        let wire = asset
            .topology
            .loops
            .iter()
            .find(|wire| wire.id == reference.id)
            .unwrap();
        let references = wire
            .edge_uses
            .iter()
            .map(|usage| EntityRef {
                kind: EntityKind::Edge,
                id: usage.edge_id,
                ..reference.clone()
            })
            .collect();
        select_closed_edges(app, references);
    }



    fn app() -> App {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample").unwrap();
        app.controller.select(Selection::Setup);
        app.operation_draft = None;
        app.setup_draft = None;
        app.refresh_fields();
        let _ = app.update(Message::BeginOp(OpKind::Profile));
        app.operation_tab = OperationTab::Geometry;
        app
    }
    fn pick(app: &mut App, reference: LoopRef) {
        let (points, _) = app.controller.loop_points(reference).unwrap();
        let p = points[0];
        assert!(app.profile_entry_drawing_pick([p.x as f32, p.y as f32], 0.5));
    }
    #[test]
    fn icon_adds_a_named_entry_settings_cancel_rolls_back_and_delete_uses_selected_row() {
        let mut app = app();
        let before = app.controller.document().clone();
        assert!(app.profile_entries().is_empty());
        app.profile_entry_message(EntryMessage::Add(ProfileGeometryKind::Chain));
        assert_eq!(app.profile_entries().len(), 1);
        assert!(app.profile_entry_editor.is_some());
        pick(&mut app, LoopRef::outer(0));
        app.profile_entry_message(EntryMessage::Name("Outside edge".into()));
        app.profile_entry_message(EntryMessage::Side(ProfileSideRule::On));
        app.profile_entry_message(EntryMessage::Confirm);
        assert!(app.profile_entry_editor.is_none());
        assert_eq!(app.profile_entries()[0].name, "Outside edge");
        app.profile_entry_message(EntryMessage::Add(ProfileGeometryKind::Chain));
        assert_eq!(app.profile_entries().len(), 2);
        app.profile_entry_message(EntryMessage::Cancel);
        assert_eq!(app.profile_entries().len(), 1);
        app.profile_entry_message(EntryMessage::EditSelected);
        assert!(app.profile_entry_editor.is_some());
        app.profile_entry_message(EntryMessage::Name("Discard this".into()));
        app.profile_entry_message(EntryMessage::Reverse(true));
        app.profile_entry_message(EntryMessage::Cancel);
        assert_eq!(app.profile_entries()[0].name, "Outside edge");
        assert!(!app.profile_entries()[0].reversed);
        app.geometry_row_selection = Some(0);
        app.profile_entry_message(EntryMessage::RemoveSelected);
        assert!(app.profile_entries().is_empty());
        assert!(app.creation_operation().is_err());
        assert_eq!(app.controller.document(), &before);
    }
    #[test]
    fn removing_profile_geometry_refreshes_default_heights_and_preserves_custom_offsets() {
        let mut app = app();
        app.profile_entry_message(EntryMessage::Add(ProfileGeometryKind::Chain));
        pick(&mut app, LoopRef::outer(0));
        app.profile_entry_message(EntryMessage::Confirm);
        let _ = app.update(Message::ConfirmOp);
        let Selection::Operation(id) = app.controller.selection() else {
            panic!("operation: {}", app.status)
        };
        app.begin_operation_draft(id);
        let previous = crate::cad_operations::selection_height_defaults(
            app.controller.document(),
            app.operation_draft.as_ref().unwrap(),
        );
        app.height_offsets = HeightPlaneKind::ALL
            .into_iter()
            .map(|plane| (plane, fmt_num(previous.definition(plane).offset)))
            .collect();
        app.height_draft = Some(previous);
        app.height_offsets
            .insert(HeightPlaneKind::Clearance, "23".into());
        app.profile_entry_message(EntryMessage::RemoveSelected);
        assert!(app.editor_selected_geometry().is_empty());
        assert!(app.operation_geometry_preview().strips.is_empty());
        assert_eq!(
            app.height_draft
                .as_ref()
                .unwrap()
                .definition(HeightPlaneKind::Bottom),
            OperationHeightSettings::default().definition(HeightPlaneKind::Bottom)
        );
        assert_eq!(app.height_offsets[&HeightPlaneKind::Clearance], "23");
    }
    #[test]
    fn entry_submenu_side_and_direction_drive_preview_and_survive_save_reopen() {
        let mut app = app();
        app.profile_entry_message(EntryMessage::Add(ProfileGeometryKind::Chain));
        pick(&mut app, LoopRef::outer(0));
        app.profile_entry_message(EntryMessage::Name("Open reversed perimeter".into()));
        app.profile_entry_message(EntryMessage::ChainMode(ProfileChainMode::Open));
        app.profile_entry_message(EntryMessage::Reverse(true));
        app.profile_entry_message(EntryMessage::Side(ProfileSideRule::AlwaysInside));
        app.profile_entry_message(EntryMessage::Confirm);
        let (Operation::Profile(profile), binding) = app.creation_operation().unwrap() else {
            panic!("profile")
        };
        assert!(binding.is_none());
        assert!(!profile.closed);
        assert_eq!(profile.side, Side::Inside);
        assert_eq!(
            profile.chain.points().first(),
            app.controller.regions()[0].outer().points().last()
        );
        let _ = app.update(Message::ConfirmOp);
        assert!(!app.creating_operation(), "{}", app.status);
        let Selection::Operation(id) = app.controller.selection() else {
            panic!("operation")
        };
        let path = std::env::temp_dir().join(format!("named-profile-{}.ocam", std::process::id()));
        app.controller.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert_eq!(reopened.operation(id), app.controller.operation(id));
        let Some(Operation::Profile(profile)) = reopened.operation(id) else {
            panic!("profile")
        };
        assert_eq!(
            profile.selections.as_ref().unwrap()[0].name,
            "Open reversed perimeter"
        );
    }
    #[test]
    fn all_selection_icons_are_valid_svgs() {
        for svg in [
            include_str!("../assets/icons/profile-selection/face.svg"),
            include_str!("../assets/icons/profile-selection/chain.svg"),
            include_str!("../assets/icons/profile-selection/silhouette.svg"),
            include_str!("../assets/icons/profile-selection/settings.svg"),
            include_str!("../assets/icons/profile-selection/closed.svg"),
            include_str!("../assets/icons/profile-selection/open.svg"),
            include_str!("../assets/icons/profile-selection/all.svg"),
            include_str!("../assets/icons/profile-selection/outer.svg"),
            include_str!("../assets/icons/profile-selection/inner.svg"),
        ] {
            let parsed = roxmltree::Document::parse(svg).unwrap();
            assert_eq!(parsed.root_element().tag_name().name(), "svg");
        }
    }
}
