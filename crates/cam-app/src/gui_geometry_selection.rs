//! Operation geometry collections shared by creation, replacement and editing.
use super::*;
use crate::LoopPart;
use iced::widget::{column, svg};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) enum ProfileSelectionMode {
    FaceContours,
    #[default]
    Chain,
    Silhouette,
}
impl ProfileSelectionMode {
    pub(super) const ALL: [Self; 3] = [Self::FaceContours, Self::Chain, Self::Silhouette];
    pub(super) fn label(self) -> &'static str {
        match self {
            Self::FaceContours => "Face Contours",
            Self::Chain => "Chain",
            Self::Silhouette => "Silhouette",
        }
    }
    pub(super) fn hint(self) -> &'static str {
        match self {
            Self::FaceContours => "Select a planar face to use all of its boundary contours, including holes. Enable Select coplanar faces to include other upward faces on the same plane in this body.",
            Self::Chain => "Select an open or closed contour. For CAD, enable Select individual edges to build a connected chain. Open chains retain their endpoints; Outside offsets left and Inside offsets right of the selected direction.",
            Self::Silhouette => "Select a body to use its silhouette as viewed along the active setup Z axis. This includes the outside outline and any through openings, projected onto setup XY. Blind recesses do not create inner silhouette contours.",
        }
    }
}

pub(super) fn cad_geometry_labels(selected: &[EntityRef]) -> Vec<String> {
    selected
        .iter()
        .map(|entity| {
            if entity.kind == EntityKind::Silhouette {
                format!(
                    "XY silhouette · Instance {} · Body {}",
                    entity.instance_id, entity.body_id
                )
            } else {
                format!(
                    "{:?} {} · Instance {} · Body {}",
                    entity.kind, entity.id, entity.instance_id, entity.body_id
                )
            }
        })
        .collect()
}

/// Read-only source names belong with the Selection controls in geometry settings.
pub(super) fn selected_feature_names(labels: Vec<String>) -> Element<'static, Message> {
    let names = if labels.is_empty() {
        "No geometry selected".to_owned()
    } else {
        labels.join("\n")
    };
    row![
        Space::new().width(90),
        text(names).size(12).width(Length::Fill),
    ]
    .spacing(6)
    .into()
}

pub(super) fn drawing_geometry_labels(selected: &[LoopRef]) -> Vec<String> {
    selected
        .iter()
        .map(|reference| match reference.part {
            LoopPart::Outer => format!("Boundary {}", reference.region + 1),
            LoopPart::Hole(index) => {
                format!("Hole {} · Boundary {}", index + 1, reference.region + 1)
            }
            LoopPart::Open => format!("Path {}", reference.region + 1),
        })
        .collect()
}

pub(super) fn operation_geometry_labels(op: &Operation) -> Vec<String> {
    match op {
        Operation::Drill(op) => (0..op.points.len())
            .map(|i| format!("Hole {}", i + 1))
            .collect(),
        Operation::Bore(op) => (0..op.points.len())
            .map(|i| format!("Bore centre {}", i + 1))
            .collect(),
        Operation::Thread(op) => (0..op.points.len())
            .map(|i| format!("Thread centre {}", i + 1))
            .collect(),
        Operation::Adaptive(_) | Operation::Parallel(_) | Operation::Spiral(_) => {
            vec!["Machining geometry".into()]
        }
        Operation::Pocket(op) => std::iter::once("Boundary".into())
            .chain((0..op.islands.len()).map(|i| format!("Island {}", i + 1)))
            .collect(),
        Operation::Carve(op) => std::iter::once("Boundary".into())
            .chain((0..op.islands.len()).map(|i| format!("Island {}", i + 1)))
            .collect(),
        Operation::Profile(op) => std::iter::once(format!(
            "{} chain 1",
            if op.closed { "Closed" } else { "Open" }
        ))
        .chain(op.additional_chains.iter().enumerate().map(|(i, p)| {
            format!(
                "{} chain {}",
                if p.closed { "Closed" } else { "Open" },
                i + 2
            )
        }))
        .collect(),
        Operation::Engrave(op) if !op.closed => vec!["Open path".into()],
        _ => vec!["Boundary".into()],
    }
}

pub(super) fn selection_icon(name: &str, size: f32) -> iced::widget::Svg<'static> {
    let bytes: &'static [u8] = match name {
        "face" => include_bytes!("../assets/icons/profile-selection/face.svg"),
        "chain" => include_bytes!("../assets/icons/profile-selection/chain.svg"),
        "silhouette" => include_bytes!("../assets/icons/profile-selection/silhouette.svg"),
        "closed" => include_bytes!("../assets/icons/profile-selection/closed.svg"),
        "open" => include_bytes!("../assets/icons/profile-selection/open.svg"),
        "all" => include_bytes!("../assets/icons/profile-selection/all.svg"),
        "outer" => include_bytes!("../assets/icons/profile-selection/outer.svg"),
        "inner" => include_bytes!("../assets/icons/profile-selection/inner.svg"),
        "drill" => include_bytes!("../assets/icons/drill.svg"),
        "bore" => include_bytes!("../assets/icons/bore.svg"),
        "thread" => include_bytes!("../assets/icons/thread.svg"),
        "remove" => include_bytes!("../assets/icons/delete_operation.svg"),
        _ => include_bytes!("../assets/icons/profile-selection/settings.svg"),
    };
    svg(svg::Handle::from_memory(bytes))
        .width(size)
        .height(size)
}

pub(super) fn selection_hint<'a>(
    control: impl Into<Element<'a, Message>>,
    label: impl Into<String>,
    show: bool,
) -> Element<'a, Message> {
    let control = control.into();
    if !show {
        return control;
    }
    delayed_tooltip(
        control,
        illustrated_help::content(&label.into()),
        tooltip::Position::Left,
    )
    .into()
}

/// Shared frame, rows and toolbar placement for every operation's collection.
pub(super) struct GeometryRow {
    pub icon: &'static str,
    pub label: String,
    pub action: Option<Message>,
}
pub(super) fn geometry_collection<'a>(
    title: &'static str,
    rows: Vec<GeometryRow>,
    selected: Option<usize>,
    select: Option<Message>,
    toolbar: iced::widget::Row<'a, Message>,
) -> Element<'a, Message> {
    let empty = rows.is_empty();
    let mut items = column![].spacing(2);
    for (index, item) in rows.into_iter().enumerate() {
        let active = selected == Some(index);
        items = items.push(
            button(
                row![
                    selection_icon(item.icon, 25.),
                    text(item.label).size(12).width(Length::Fill)
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            )
            .width(Length::Fill)
            .padding([7, 8])
            .style(move |theme, status| inspector_tab(theme, status, active))
            .on_press_maybe(item.action),
        );
    }
    if empty {
        items = items.push(
            container(text("No geometry selected").size(12).color(INSPECTOR_MUTED)).padding(8),
        );
    }
    column![
        row![
            text(title).size(12).width(Length::Fill),
            inspector_button("Select").on_press_maybe(select)
        ]
        .spacing(6)
        .align_y(Alignment::Center),
        container(scrollable(items).height(120).width(Length::Fill))
            .padding(4)
            .width(Length::Fill)
            .style(|_| container::Style {
                background: Some(Background::Color(Color::from_rgb(0.12, 0.14, 0.17))),
                border: Border {
                    color: Color::from_rgb(0.25, 0.28, 0.32),
                    width: 1.,
                    radius: 5.0.into()
                },
                ..Default::default()
            }),
        toolbar
    ]
    .spacing(6)
    .into()
}

impl App {
    pub(super) fn geometry_list(&self, labels: Vec<String>) -> Element<'_, Message> {
        if self.profile_geometry_active() {
            return self.profile_geometry_collection();
        }
        if let Some(
            kind @ (OpKind::Pocket
            | OpKind::Chamfer
            | OpKind::Engrave
            | OpKind::Carve
            | OpKind::Drill
            | OpKind::Bore
            | OpKind::Thread
            | OpKind::Adaptive
            | OpKind::Parallel
            | OpKind::Spiral),
        ) = self.geometry_operation_kind()
        {
            return self.operation_geometry_collection(kind, labels);
        }
        let count = labels.len();
        let selected = self
            .geometry_row_selection
            .filter(|index| *index < count)
            .or_else(|| (count == 1).then_some(0));
        let mut items = column![].spacing(2).width(Length::Fill);
        for (index, label) in labels.into_iter().enumerate() {
            let active = selected == Some(index);
            items = items.push(
                container(
                    row![
                        button(text(label).size(12).width(Length::Fill))
                            .padding([8, 9])
                            .width(Length::Fill)
                            .style(move |theme, status| inspector_tab(theme, status, active))
                            .on_press(Message::GeometryRowSelected(index)),
                        inspector_button("×").on_press(Message::GeometryRemove(index)),
                    ]
                    .spacing(2)
                    .align_y(Alignment::Center),
                )
                .width(Length::Fill),
            );
        }
        if count == 0 {
            items = items.push(
                container(text("No geometry selected").size(12).color(INSPECTOR_MUTED)).padding(9),
            );
        }
        let mut selection = column![
            row![
                text("Geometry selection").size(12).width(Length::Fill),
                inspector_button(if self.geometry_additive {
                    "Selecting…"
                } else {
                    "Select"
                })
                .style(|theme, status| inspector_tab(theme, status, self.geometry_additive))
                .on_press(Message::GeometryPick),
                inspector_button("Clear")
                    .on_press_maybe((count > 0).then_some(Message::GeometryClear)),
            ]
            .spacing(6)
            .align_y(Alignment::Center),
            container(scrollable(items).width(Length::Fill).height(104))
                .padding(4)
                .width(Length::Fill)
                .style(|_| container::Style {
                    background: Some(Background::Color(Color::from_rgb(0.12, 0.14, 0.17))),
                    border: Border {
                        color: Color::from_rgb(0.25, 0.28, 0.32),
                        width: 1.0,
                        radius: 5.0.into()
                    },
                    ..container::Style::default()
                }),
        ]
        .spacing(6);
        if self.profile_geometry_active() {
            selection = selection.push(self.profile_selection_controls());
        }
        selection.into()
    }

    pub(super) fn geometry_operation_kind(&self) -> Option<OpKind> {
        self.cad_pending_kind
            .or_else(|| self.controller.pending_op().map(|p| p.kind))
            .or_else(|| {
                self.operation_draft
                    .as_ref()
                    .map(AppController::operation_kind)
            })
    }

    pub(super) fn profile_geometry_active(&self) -> bool {
        self.cad_pending_kind == Some(OpKind::Profile)
            || self
                .controller
                .pending_op()
                .is_some_and(|p| p.kind == OpKind::Profile)
            || self
                .operation_draft
                .as_ref()
                .is_some_and(|op| matches!(op, Operation::Profile(_)))
    }

    fn profile_selection_controls(&self) -> Element<'_, Message> {
        let mut modes = row![].spacing(4);
        for mode in ProfileSelectionMode::ALL {
            let active = mode == self.profile_selection_mode;
            let control = button(text(mode.label()).size(11))
                .padding([7, 6])
                .style(move |theme, status| inspector_tab(theme, status, active))
                .on_press_maybe(
                    (mode == ProfileSelectionMode::Chain
                        || !self.controller.geometry_state().instances.is_empty())
                    .then_some(Message::ProfileSelectionModeChanged(mode)),
                );
            modes = modes.push(delayed_tooltip(
                control,
                container(
                    column![text(mode.label()).size(15), text(mode.hint()).size(12)]
                        .spacing(8)
                        .width(300),
                )
                .padding(12)
                .style(popup_surface),
                tooltip::Position::Left,
            ));
        }
        let mut controls = column![modes].spacing(6);
        if self.profile_selection_mode == ProfileSelectionMode::FaceContours {
            controls = controls.push(delayed_tooltip(
                checkbox(self.profile_coplanar).label("Select coplanar faces").size(15).text_size(12)
                    .on_toggle(Message::ProfileCoplanarChanged),
                container(text("Include all upward planar faces at the selected face's height in the same body instance. The common cutting depth stops at the shallowest qualified wall.").size(12).width(280)).padding(10).style(popup_surface),
                tooltip::Position::Left));
        }
        if self.profile_selection_mode == ProfileSelectionMode::Chain
            && !self.controller.geometry_state().instances.is_empty()
        {
            controls = controls.push(delayed_tooltip(
                checkbox(self.profile_chain_edges).label("Select individual edges").size(15).text_size(12)
                    .on_toggle(Message::ProfileChainEdgesChanged),
                container(text("Click connected CAD edges to add or remove them. An unclosed chain is machined as an open path; a complete loop is closed. The cutter side is inferred from the adjoining face boundary.").size(12).width(280)).padding(10).style(popup_surface), tooltip::Position::Left));
        }
        controls.into()
    }

    pub(super) fn pick_operation_geometry(&mut self) -> iced::Task<Message> {
        self.profile_tab_picking = false;
        self.geometry_additive = true;
        if self.wrapped_pocket_geometry_editor() {
            self.status = "Select geometry in the viewport, then use OK to stage it.".into();
            return iced::Task::none();
        }
        if self.creating_operation()
            || self.cad_editing_op.is_some()
            || self.drawing_editing_op.is_some()
        {
            self.status = if self.geometry_editor.is_some() {
                "Select geometry in the viewport, then use OK to stage it."
            } else {
                "Click geometry in the viewport to add it. Use × to remove an item."
            }
            .into();
            return iced::Task::none();
        }
        if let Selection::Operation(id) = self.controller.selection() {
            let message = if self.controller.geometry_state().bindings.contains_key(&id) {
                Message::EditCadGeometry(id)
            } else {
                Message::EditDrawingGeometry(id)
            };
            return self.update(message);
        }
        iced::Task::none()
    }

    pub(super) fn remove_operation_geometry(&mut self, index: Option<usize>) {
        if self.wrapped_pocket_geometry_editor() {
            let mut selected = self.controller.cad_selection().to_vec();
            if let Some(index) = index {
                if index < selected.len() {
                    selected.remove(index);
                }
            } else {
                selected.clear();
            }
            self.controller.set_cad_selection(selected);
            self.geometry_row_selection = None;
            self.cad_candidates.clear();
            return;
        }
        if self.operation_uses_cad_geometry()
            && matches!(self.operation_draft.as_ref(), Some(Operation::Pocket(p)) if p.wrap.is_some())
        {
            let mut wrap = self
                .operation_draft
                .as_ref()
                .unwrap()
                .rotary_wrap()
                .unwrap()
                .clone();
            if let Some(index) = index {
                if index < wrap.geometry.len() {
                    wrap.geometry.remove(index);
                }
            } else {
                wrap.geometry.clear();
            }
            self.controller.set_cad_selection(wrap.geometry.clone());
            self.edit_operation_draft(|op| op.set_rotary_wrap(Some(wrap)));
            self.geometry_row_selection = None;
            self.cad_candidates.clear();
            return;
        }
        if self.cad_pending_kind.is_none()
            && self.cad_editing_op.is_none()
            && self.controller.pending_op().is_none()
        {
            if let (Some(index), Selection::Operation(id)) = (index, self.controller.selection()) {
                if !self.controller.geometry_state().bindings.contains_key(&id) {
                    if let Some(mut op) = self.current_operation(id).cloned() {
                        let removed = match &mut op {
                            Operation::Profile(op)
                                if !op.additional_chains.is_empty()
                                    && index <= op.additional_chains.len() =>
                            {
                                if index == 0 {
                                    let first = op.additional_chains.remove(0);
                                    op.chain = first.chain;
                                    op.closed = first.closed;
                                    op.side = first.side;
                                } else {
                                    op.additional_chains.remove(index - 1);
                                }
                                true
                            }
                            Operation::Drill(op)
                                if op.points.len() > 1 && index < op.points.len() =>
                            {
                                op.points.remove(index);
                                if index < op.hole_heights.len() {
                                    op.hole_heights.remove(index);
                                }
                                true
                            }
                            Operation::Thread(op)
                                if op.points.len() > 1 && index < op.points.len() =>
                            {
                                op.points.remove(index);
                                true
                            }
                            Operation::Pocket(op) if index > 0 && index <= op.islands.len() => {
                                op.islands.remove(index - 1);
                                true
                            }
                            Operation::Carve(op) if index > 0 && index <= op.islands.len() => {
                                op.islands.remove(index - 1);
                                true
                            }
                            _ => false,
                        };
                        if removed {
                            self.operation_draft = Some(op);
                            self.geometry_row_selection = None;
                            return;
                        }
                    }
                }
            }
            let _ = self.pick_operation_geometry();
        }
        self.geometry_row_selection = None;
        self.cad_candidates.clear();
        if self.cad_pending_kind.is_some() || self.cad_editing_op.is_some() {
            let mut selected = self.controller.cad_selection().to_vec();
            if let Some(index) = index {
                if index < selected.len() {
                    selected.remove(index);
                }
            } else {
                selected.clear();
            }
            let result = if matches!(
                self.geometry_operation_kind(),
                Some(OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral)
            ) {
                self.controller.try_set_adaptive_cad_selection(selected)
            } else {
                self.controller.try_set_cad_selection(selected)
            };
            if let Err(error) = result {
                self.status = error;
            }
        } else if self.controller.pending_op().is_some() {
            let mut selected = self.controller.pending_operation_geometry();
            if let Some(index) = index {
                if index < selected.len() {
                    selected.remove(index);
                }
            } else {
                selected.clear();
            }
            if let Err(error) = self
                .controller
                .set_pending_operation_geometry(&selected, None)
            {
                self.status = error;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profile_mode_controls_route_picks_and_preserve_the_committed_operation() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.setup_draft = None;
        app.cad_pending_kind = Some(OpKind::Profile);
        app.operation_draft = Some(app.controller.operation_template(OpKind::Profile, 1));
        let before = app.controller.document().clone();
        for mode in ProfileSelectionMode::ALL {
            let _ = app.update(Message::ProfileSelectionModeChanged(mode));
            assert_eq!(app.profile_selection_mode, mode);
            let viewport = app.machining_viewport();
            assert_eq!(viewport.cad_operation_kind, Some(OpKind::Profile));
            assert_eq!(
                viewport.cad_silhouette,
                mode == ProfileSelectionMode::Silhouette
            );
            assert_eq!(
                viewport.cad_filter,
                if mode == ProfileSelectionMode::FaceContours {
                    EntityKind::Face
                } else {
                    EntityKind::Loop
                }
            );
            assert_eq!(app.controller.document(), &before);
            assert!(!mode.hint().is_empty());
            let _ = app.profile_selection_controls();
        }
        let _ = app.update(Message::ProfileSelectionModeChanged(
            ProfileSelectionMode::Chain,
        ));
        let _ = app.update(Message::ProfileChainEdgesChanged(true));
        assert_eq!(app.machining_viewport().cad_filter, EntityKind::Edge);
        assert_eq!(app.controller.document(), &before);
    }
}
