//! Independent model picks for operation entry and exit preferences.
use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PositionKind {
    Entry,
    Exit,
}

impl PositionKind {
    pub(super) fn index(self) -> usize {
        match self {
            Self::Entry => 0,
            Self::Exit => 1,
        }
    }
    pub(super) fn color(self) -> [f32; 4] {
        match self {
            Self::Entry => [0.15, 0.8, 1.0, 1.0],
            Self::Exit => [1.0, 0.6, 0.15, 1.0],
        }
    }
}

pub(super) fn operation_end(op: &Operation) -> Option<[f64; 2]> {
    match op {
        Operation::Profile(o) if o.closed => o.end,
        Operation::Pocket(o) => o.end,
        Operation::Chamfer(o) if o.closed && o.faces.is_empty() => o.end,
        Operation::Engrave(o) if o.closed => o.end,
        Operation::Carve(o) => o.end,
        Operation::Adaptive(o) => o.params.end,
        _ => None,
    }
}
fn set_end(op: &mut Operation, point: Option<[f64; 2]>) {
    match op {
        Operation::Profile(o) if o.closed => o.end = point,
        Operation::Pocket(o) => o.end = point,
        Operation::Chamfer(o) if o.closed && o.faces.is_empty() => o.end = point,
        Operation::Engrave(o) if o.closed => o.end = point,
        Operation::Carve(o) => o.end = point,
        Operation::Adaptive(o) => o.params.end = point,
        _ => {}
    }
}
impl App {
    pub(super) fn operation_position_controls(&self, op: &Operation) -> Element<'_, Message> {
        let mut controls = iced::widget::Column::new().spacing(6);
        let spiral = matches!(op, Operation::Spiral(_));
        for (kind, title, point) in [
            (
                PositionKind::Entry,
                if spiral {
                    "Spiral center"
                } else {
                    "Lead-in position"
                },
                operation_tabs::operation_start(op),
            ),
            (PositionKind::Exit, "Lead-out position", operation_end(op)),
        ] {
            if kind == PositionKind::Exit
                && (spiral || matches!(op, Operation::Parallel(p) if p.params.scallop))
            {
                continue;
            }
            let picking = self.operation_position_pick == Some(kind);
            controls = controls.push(
                row![
                    label_help(
                        title,
                        if spiral {
                            "Pick the center on the model. Clear the pick to use the center of the selected geometry."
                        } else {
                            "Select a point on the model: blue = entry, orange = exit. Positions resolve to eligible toolpath points. Use × to restore automatic placement."
                        },
                        self.tooltips,
                    ),
                    help_wrap(
                        button(
                            row![
                                iced::widget::svg(iced::widget::svg::Handle::from_memory(
                                    include_bytes!("../assets/icons/point-selection.svg"),
                                )).width(18).height(18),
                                text(if point.is_some() { "Point" } else { "Select" }).size(12),
                            ].spacing(6).align_y(Alignment::Center),
                        )
                        .padding([6, 8])
                        .width(100)
                        .height(32)
                        .style(move |theme, status| {
                            let mut style = inspector_pick_style(theme, status, picking);
                            if point.is_some() && !picking {
                                style.border.color = INSPECTOR_MUTED;
                            }
                            style
                        })
                        .on_press(Message::OperationPositionPick(kind)),
                        "Pick or replace a point on the model. Click again or press Escape to cancel selection.",
                        self.tooltips,
                    ),
                    help_wrap(
                        inspector_button("×")
                            .width(28)
                            .height(32)
                            .padding(0)
                            .style(button::text)
                            .on_press_maybe(point.map(|_| Message::OperationPositionClear(kind))),
                        "Clear the selected point and restore automatic placement.",
                        self.tooltips,
                    ),
                ]
                .spacing(6)
                .align_y(Alignment::Center),
            );
        }
        controls.into()
    }
    pub(super) fn begin_position_pick(&mut self, kind: PositionKind) {
        self.operation_position_pick = if self.operation_position_pick == Some(kind) {
            None
        } else {
            Some(kind)
        };
        if self.operation_position_pick.is_some() {
            // Browsing operations and Apply can leave the inspector showing a
            // saved operation without a draft. Picking needs the editor preview
            // just like a field edit does; preserve its numeric buffers.
            self.edit_operation_draft(|_| {});
            self.inspector_active = true;
            self.model_view = None;
        }
        self.geometry_additive = false;
        self.geometry_auto_pick = false;
        self.profile_tab_picking = false;
        self.height_reference_pick = None;
        self.cad_hover = None;
        self.cad_candidates.clear();
        self.setup_hover = SetupHover::default();
        self.status = if self.operation_position_pick.is_some() {
            "Select the preferred position on the model; Escape cancels selection.".into()
        } else {
            "Position selection cancelled.".into()
        };
    }
    pub(super) fn clear_operation_position(&mut self, kind: PositionKind) {
        self.operation_position_pick = None;
        self.operation_position_markers[kind.index()] = None;
        self.edit_operation_draft(|op| match kind {
            PositionKind::Entry => operation_tabs::set_operation_start(op, false),
            PositionKind::Exit => set_end(op, None),
        });
        self.refresh_operation_fields();
        self.status =
            "Position cleared; automatic placement restored. OK saves the operation.".into();
    }
    pub(super) fn pick_operation_surface_position(&mut self, point: [f64; 3]) {
        let Some(kind) = self.operation_position_pick else {
            return;
        };
        if !point.into_iter().all(f64::is_finite) {
            return;
        }
        self.pick_operation_position([point[0], point[1]]);
        // Height belongs to the visual pick only. Toolpath preferences remain in
        // setup XY, while the marker stays on the surface the user selected.
        self.operation_position_markers[kind.index()] = Some(point);
    }

    pub(super) fn operation_position_marker(
        &self,
        kind: PositionKind,
        xy: [f64; 2],
        z: f64,
    ) -> [f64; 3] {
        self.operation_position_markers[kind.index()]
            .filter(|point| point[..2] == xy)
            .unwrap_or([xy[0], xy[1], z])
    }

    pub(super) fn pick_operation_position(&mut self, point: [f64; 2]) {
        if !point.into_iter().all(f64::is_finite) {
            return;
        }
        let Some(kind) = self.operation_position_pick.take() else {
            return;
        };
        self.operation_position_markers[kind.index()] = None;
        self.edit_operation_draft(|op| match kind {
            PositionKind::Entry => {
                operation_tabs::set_operation_start(op, true);
                for (axis, value) in point.into_iter().enumerate() {
                    operation_tabs::set_operation_start_coordinate(op, axis, value);
                }
            }
            PositionKind::Exit => set_end(op, Some(point)),
        });
        if kind == PositionKind::Entry {
            if self.creating_operation() {
                self.creation_edited_fields
                    .extend([Field::StartX, Field::StartY]);
            }
            self.fields.remove(&Field::StartX);
            self.fields.remove(&Field::StartY);
        }
        self.refresh_operation_fields();
        self.status = format!(
            "Preferred {} selected. Apply changes to save the operation.",
            if self.geometry_operation_kind() == Some(OpKind::Spiral) {
                "spiral center"
            } else if kind == PositionKind::Entry {
                "entry"
            } else {
                "exit"
            },
        );
    }
}
