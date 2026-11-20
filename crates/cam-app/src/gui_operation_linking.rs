//! One owner for linking controls, grouped in machining order.
use super::*;
use parameter_options::*;
use cam_model::Comp;
use iced::widget::{column, Column};

pub(super) const KEEP_TOOL_DOWN_HELP: &str = "Continue from the current cutting height into the next depth or finishing pass using a safe feed link. Unsafe or disconnected transitions still retract to clearance.";
pub(super) const BORE_CENTER_HELP: &str = "For an internal bore, start and end the tangent leads at the hole centre. The lead radius becomes half the tool-centre orbit. Each pass retracts to clearance.";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum Section {
    Moves,
    Leads,
    Ramping,
    Positions,
}
impl Section {
    fn label(self) -> &'static str {
        match self {
            Self::Moves => "Linking moves",
            Self::Leads => "Leads and transitions",
            Self::Ramping => "Ramping",
            Self::Positions => "Positions",
        }
    }
    pub(super) fn id(self) -> operation_sections::Section {
        operation_sections::Section::new(OperationTab::Linking, self.label())
    }
    fn field(field: Field) -> Self {
        match field {
            Field::LeadInSize
            | Field::LeadOutSize
            | Field::LeadOverlap
            | Field::ClearLeadInSize
            | Field::ClearLeadOutSize
            | Field::ClearLeadOverlap
            | Field::BoreLeadRadius => Self::Leads,
            Field::PlungeA | Field::PlungeB | Field::ClearPlungeA | Field::ClearPlungeB => {
                Self::Ramping
            }
            Field::StartX | Field::StartY => Self::Positions,
            _ => Self::Moves,
        }
    }
}

impl App {
    fn linking_field_controls(
        &self,
        op: &Operation,
        section: Section,
    ) -> Vec<Element<'static, Message>> {
        let mut fields = Vec::new();
        for field in self.geometry_operation_fields(op) {
            if operation_tabs::field_tab(field) != OperationTab::Linking
                || Section::field(field) != section
                || (is_clear_field(field) && !matches!(op, Operation::Carve(_)))
                || (matches!(op, Operation::Profile(_))
                    && section == Section::Leads
                    && matches!(
                        field,
                        Field::LeadInSize | Field::LeadOutSize | Field::LeadOverlap
                    ))
                || (matches!(op, Operation::Chamfer(_))
                    && section == Section::Leads
                    && matches!(
                        field,
                        Field::LeadInSize | Field::LeadOutSize | Field::LeadOverlap
                    ))
            {
                continue;
            }
            let value = self.fields.get(&field).cloned().unwrap_or_default();
            let (label, help) = match (op, field) {
                (Operation::Adaptive(_), Field::LeadInSize) => (
                    "Lead-in arc radius (mm)",
                    "Radius of the tangent arc into the cut from a cleared link. Zero disables the arc.",
                ),
                (Operation::Adaptive(_), Field::LeadOutSize) => (
                    "Lead-out arc radius (mm)",
                    "Radius of the tangent arc away from the cut before a cleared link. Zero disables the arc.",
                ),
                _ => (self.field_label(field), self.field_help(field)),
            };
            fields.push(self.parameter_field_row(field, label, help, &value));
        }
        fields
    }

    pub(super) fn operation_linking_controls(
        &self,
        op: &Operation,
        linked: bool,
    ) -> Element<'_, Message> {
        let mut moves: Vec<Element<'_, Message>> = Vec::new();
        let mut leads: Vec<Element<'_, Message>> = Vec::new();
        let mut ramping: Vec<Element<'_, Message>> = Vec::new();
        match op {
            Operation::Profile(p) => {
                let field = |field: Field| {
                    let value = self.fields.get(&field).cloned().unwrap_or_default();
                    let label = match field {
                        Field::LeadInSize
                            if matches!(p.lead_in, Lead::Arc { .. } | Lead::ArcSweep { .. }) =>
                        {
                            "Horizontal Lead-In Radius (mm)"
                        }
                        Field::LeadOutSize
                            if matches!(p.lead_out, Lead::Arc { .. } | Lead::ArcSweep { .. }) =>
                        {
                            "Horizontal Lead-Out Radius (mm)"
                        }
                        _ => self.field_label(field),
                    };
                    self.parameter_field_row(field, label, self.field_help(field), &value)
                };
                let sweep = |number: profile_parameters::Number| {
                    let field = Field::Profile(number);
                    let value = self
                        .fields
                        .get(&field)
                        .cloned()
                        .unwrap_or_else(|| fmt_num(number.get(p)));
                    self.parameter_field_row(field, number.label(), number.help(), &value)
                };
                let keep_down_supported =
                    p.comp == Comp::Computed && !p.tabs.enabled && p.additional_chains.is_empty();
                moves.push(help_wrap(
                    row![
                        text("Keep tool down").size(12).width(Length::Fill),
                        if keep_down_supported {
                            PROFILE_KEEP_DOWN.checkbox(p.passes.keep_tool_down, 15)
                        } else {
                            checkbox(p.passes.keep_tool_down).size(15).into()
                        },
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                    if keep_down_supported { KEEP_TOOL_DOWN_HELP } else {
                        "Keep tool down requires computed cutter compensation, a single contour, and no holding tabs."
                    },
                    self.tooltips,
                ));
                moves.push(help_wrap(
                    row![
                        text("Retract at lead-out feed")
                            .size(12)
                            .width(Length::Fill),
                        PROFILE_FEED_RETRACT.checkbox(p.passes.feed_retract, 15),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                    "Use the lead-out feed for the retract after leaving the contour.",
                    self.tooltips,
                ));
                let entry_enabled = p.passes.lead_in_enabled.unwrap_or(p.lead_in != Lead::None);
                leads.push(
                    row![
                        label_help("Lead-In (Entry)", help::LEAD_IN, self.tooltips),
                        PROFILE_ENTRY.checkbox(entry_enabled, 15)
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center)
                    .into(),
                );
                if entry_enabled {
                    {
                        leads.push(operation_picker(
                            "Entry lead type",
                            help::LEAD_IN,
                            self.tooltips,
                            LeadKind::of(p.lead_in),
                            if p.comp == Comp::Control {
                                &LeadKind::ALL[1..2]
                            } else {
                                &LeadKind::ALL[1..]
                            },
                            LEAD_IN,
                        ));
                    }
                    leads.push(field(Field::LeadInSize));
                    if matches!(p.lead_in, Lead::Arc { .. } | Lead::ArcSweep { .. }) {
                        leads.push(sweep(profile_parameters::Number::LeadInSweep));
                    }
                }
                let exit_enabled = p.passes.lead_out_enabled.unwrap_or(
                    p.lead_out != Lead::None || (p.passes.same_lead_out && p.lead_in != Lead::None),
                );
                leads.push(
                    row![
                        label_help(
                            "Lead-Out (Exit)",
                            profile_parameters::Toggle::LeadOutEnabled.help(),
                            self.tooltips
                        ),
                        PROFILE_EXIT.checkbox(exit_enabled, 15)
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center)
                    .into(),
                );
                if exit_enabled && entry_enabled {
                    leads.push(
                        row![
                            label_help(
                                "Same as Lead-In",
                                profile_parameters::Toggle::SameLeadOut.help(),
                                self.tooltips
                            ),
                            PROFILE_SAME_OUT.checkbox(p.passes.same_lead_out, 15)
                        ]
                        .spacing(8)
                        .align_y(Alignment::Center)
                        .into(),
                    );
                }
                if exit_enabled && (!p.passes.same_lead_out || !entry_enabled) {
                    {
                        leads.push(operation_picker(
                            "Exit lead type",
                            help::LEAD_OUT,
                            self.tooltips,
                            LeadKind::of(p.lead_out),
                            if p.comp == Comp::Control {
                                &LeadKind::ALL[1..2]
                            } else {
                                &LeadKind::ALL[1..]
                            },
                            LEAD_OUT,
                        ));
                    }
                    if p.lead_out != Lead::None {
                        leads.push(field(Field::LeadOutSize));
                        if matches!(p.lead_out, Lead::Arc { .. } | Lead::ArcSweep { .. }) {
                            leads.push(sweep(profile_parameters::Number::LeadOutSweep));
                        }
                    }
                }
                if p.contours().all(|contour| contour.closed) {
                    moves.push(field(Field::LeadOverlap));
                }
                ramping.push(operation_picker(
                    "Plunge",
                    help::PLUNGE,
                    self.tooltips,
                    PlungeKind::of(p.plunge),
                    if p.capabilities(linked, self.controller.post_kind().supports_cutter_comp())
                        .ramp
                    {
                        &PlungeKind::ALL[..]
                    } else {
                        &PlungeKind::ALL[..1]
                    },
                    PLUNGE,
                ));
            }
            Operation::Pocket(p) => {
                moves.push(help_wrap(
                    row![
                        text("Keep tool down").size(12).width(Length::Fill),
                        POCKET_STAY_DOWN.checkbox(p.clear.keep_tool_down, 15),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                    KEEP_TOOL_DOWN_HELP,
                    self.tooltips,
                ));
                leads.push(operation_picker(
                    "Wall lead-in",
                    help::LEAD_IN,
                    self.tooltips,
                    LeadKind::of(p.clear.lead_in),
                    &LeadKind::ALL[..],
                    LEAD_IN,
                ));
                leads.push(operation_picker(
                    "Wall lead-out",
                    help::LEAD_OUT,
                    self.tooltips,
                    LeadKind::of(p.clear.lead_out),
                    &LeadKind::ALL[..],
                    LEAD_OUT,
                ));
                ramping.push(operation_picker(
                    "Plunge",
                    help::PLUNGE,
                    self.tooltips,
                    PlungeKind::of(p.clear.plunge),
                    &PlungeKind::ALL[..],
                    PLUNGE,
                ));
            }
            Operation::Parallel(p) => {
                moves.push(self.finishing_retraction_control(p.params.retraction_policy()));
                if !p.params.scallop {
                    leads.push(operation_picker(
                        "Transition type",
                        "No contact retracts between passes. Straight line joins the pass ends at the boundary. Shortest path prefers the shortest checked cutting connection. Smooth rounds beyond the pass ends where the cutter can clear the model.",
                        self.tooltips,
                        p.params.transition,
                        &cam_model::ParallelTransition::ALL[..],
                        PARALLEL_TRANSITION,
                    ));
                }
            }
            Operation::Spiral(p) => {
                moves.push(self.finishing_retraction_control(p.params.retraction));
            }
            Operation::Adaptive(_) => {
                moves.push(label_help(
                    "Retract along cleared path",
                    "The exit follows the final connected cleared path before retracting. A preferred exit resolves along this safe path.",
                    self.tooltips,
                ));
                ramping.push(label_help(
                    "Helical entry",
                    "3D Adaptive Clearing uses safe helical entries near the preferred entry. Set Ramp feed on Tool to override the entry rate.",
                    self.tooltips,
                ));
            }
            Operation::Drill(_) => {
                moves.push(label_help(
                    "Cycle approach and retract",
                    "Approach and retract follow the drilling cycle and operation safety heights.",
                    self.tooltips,
                ));
            }
            Operation::Face(_) => {
                moves.push(label_help("Automatic links and turnarounds",
                    "Facing links and turnaround arcs are determined by the strategy. Overshoot is on Passes.", self.tooltips));
            }
            Operation::Thread(_) => {
                leads.push(label_help(
                    "Helical entry and exit",
                    "Helical entry and exit are determined by the threading strategy.",
                    self.tooltips,
                ));
            }
            Operation::Engrave(o) if !o.closed => {
                leads.push(label_help("Entry and exit follow the stroke",
                    "Open strokes start at their first vertex. Entry and exit are determined by the engraving strategy.", self.tooltips));
            }
            Operation::Chamfer(c) => {
                if !c.faces.is_empty() || self.chamfer_faces_active() {
                    moves.push(label_help("Air-side approach", "Each face band approaches from the air side and retracts before repositioning.", self.tooltips));
                }
                let field = |field: Field| {
                    let value = self.fields.get(&field).cloned().unwrap_or_default();
                    let label = match field {
                        Field::LeadInSize
                            if matches!(c.lead_in, Lead::Arc { .. } | Lead::ArcSweep { .. }) =>
                        {
                            "Horizontal Lead-In Radius (mm)"
                        }
                        Field::LeadOutSize
                            if matches!(c.lead_out, Lead::Arc { .. } | Lead::ArcSweep { .. }) =>
                        {
                            "Horizontal Lead-Out Radius (mm)"
                        }
                        _ => self.field_label(field),
                    };
                    self.parameter_field_row(field, label, self.field_help(field), &value)
                };
                let entry_enabled = c.lead_in != Lead::None && c.lead_in_enabled.unwrap_or(true);
                if c.closed && c.faces.is_empty() && !self.chamfer_faces_active() {
                    moves.push(field(Field::LeadOverlap));
                }
                leads.push(
                    row![
                        label_help("Lead-In (Entry)", help::LEAD_IN, self.tooltips),
                        CHAMFER_ENTRY.checkbox(entry_enabled, 15)
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center)
                    .into(),
                );
                if entry_enabled {
                    {
                        leads.push(operation_picker(
                            "Entry lead type",
                            help::LEAD_IN,
                            self.tooltips,
                            LeadKind::of(c.lead_in),
                            &LeadKind::ALL[1..],
                            LEAD_IN,
                        ));
                    }
                    leads.push(field(Field::LeadInSize));
                    if matches!(c.lead_in, Lead::Arc { .. } | Lead::ArcSweep { .. }) {
                        leads.push(field(Field::Profile(
                            profile_parameters::Number::LeadInSweep,
                        )));
                    }
                }
                let exit_enabled = c.lead_out_enabled.unwrap_or(true)
                    && (c.lead_out != Lead::None || (c.same_lead_out && entry_enabled));
                leads.push(
                    row![
                        text("Lead-Out (Exit)").size(12).width(Length::Fill),
                        CHAMFER_EXIT.checkbox(exit_enabled, 15)
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center)
                    .into(),
                );
                if exit_enabled && entry_enabled {
                    leads.push(
                        row![
                            text("Same as Lead-In").size(12).width(Length::Fill),
                            CHAMFER_SAME_LEAD.checkbox(c.same_lead_out, 15)
                        ]
                        .spacing(8)
                        .align_y(Alignment::Center)
                        .into(),
                    );
                }
                if exit_enabled && (!c.same_lead_out || !entry_enabled) {
                    {
                        leads.push(operation_picker(
                            "Exit lead type",
                            help::LEAD_OUT,
                            self.tooltips,
                            LeadKind::of(c.lead_out),
                            &LeadKind::ALL[1..],
                            LEAD_OUT,
                        ));
                    }
                    if c.lead_out != Lead::None {
                        leads.push(field(Field::LeadOutSize));
                        if matches!(c.lead_out, Lead::Arc { .. } | Lead::ArcSweep { .. }) {
                            leads.push(field(Field::Profile(
                                profile_parameters::Number::LeadOutSweep,
                            )));
                        }
                    }
                }
            }
            Operation::Bore(b) => {
                if b.internal {
                    leads.push(help_wrap(
                        row![
                            text("Lead to centre").size(12).width(Length::Fill),
                            BORE_CENTER.checkbox(b.lead_to_center, 15),
                        ]
                        .spacing(8),
                        BORE_CENTER_HELP,
                        self.tooltips,
                    ));
                }
            }
            Operation::Carve(c) => {
                moves.push(
                    (row![
                        label_help("Stay down", help::CARVE_STAY_DOWN, self.tooltips),
                        CARVE_STAY_DOWN.checkbox(c.stay_down, 15),
                    ]
                    .spacing(8))
                    .into(),
                );
                moves.push(help_wrap(
                    inspector_section_checkbox("Clearing pass (end mill)", c.clear.is_some())
                        .on_toggle(Message::CarveClearToggled),
                    help::CARVE_CLEAR,
                    self.tooltips,
                ));
                ramping.push(operation_picker(
                    "Plunge",
                    help::CARVE_PLUNGE,
                    self.tooltips,
                    PlungeKind::of(c.plunge),
                    &PlungeKind::ALL[..],
                    PLUNGE,
                ));
                if let Some(cl) = &c.clear {
                    leads.push(operation_picker(
                        "Clearing lead-in",
                        help::LEAD_IN,
                        self.tooltips,
                        LeadKind::of(cl.params.lead_in),
                        &LeadKind::ALL[..],
                        CARVE_CLEAR_LEAD_IN,
                    ));
                    leads.push(operation_picker(
                        "Clearing lead-out",
                        help::LEAD_OUT,
                        self.tooltips,
                        LeadKind::of(cl.params.lead_out),
                        &LeadKind::ALL[..],
                        CARVE_CLEAR_LEAD_OUT,
                    ));
                    ramping.push(operation_picker(
                        "Clearing plunge",
                        help::CARVE_CLEAR_PLUNGE,
                        self.tooltips,
                        PlungeKind::of(cl.params.plunge),
                        &PlungeKind::ALL[..],
                        CARVE_CLEAR_PLUNGE,
                    ));
                }
            }
            _ => {}
        }
        moves.extend(self.linking_field_controls(op, Section::Moves));
        leads.extend(self.linking_field_controls(op, Section::Leads));
        ramping.extend(self.linking_field_controls(op, Section::Ramping));
        if let Operation::Profile(p) = op {
            moves.push(
                self.profile_extra_controls(p, OperationTab::Linking, linked)
                    .finish(),
            );
        }
        let mut sections = Column::new().spacing(12);
        for (section, body) in [
            (Section::Moves, moves),
            (Section::Leads, leads),
            (Section::Ramping, ramping),
        ] {
            if !body.is_empty() {
                sections = sections.push(self.operation_section(
                    section.id(),
                    Column::with_children(body).spacing(10),
                    None,
                ));
            }
        }
        let supports_positions = matches!(
            op,
            Operation::Profile(_)
                | Operation::Pocket(_)
                | Operation::Carve(_)
                | Operation::Adaptive(_)
        ) || matches!(op, Operation::Engrave(o) if o.closed)
            || matches!(op, Operation::Parallel(o) if o.params.scallop)
            || matches!(op, Operation::Chamfer(o) if o.closed && o.faces.is_empty());
        if supports_positions {
            let positions = column![self.operation_position_controls(op)].spacing(10);
            sections =
                sections.push(self.operation_section(Section::Positions.id(), positions, None));
        }
        sections.into()
    }
    fn finishing_retraction_control(
        &self,
        policy: cam_model::ScallopRetraction,
    ) -> Element<'_, Message> {
        operation_picker(
                    "Retraction policy",
                    "Choose a full retract, a checked raised traverse, or a checked straight feed link between disconnected passes.",
                    self.tooltips,
                    policy,
                    &[
                        cam_model::ScallopRetraction::Full,
                        cam_model::ScallopRetraction::Minimum,
                        cam_model::ScallopRetraction::Shortest,
                    ][..],
                    RETRACTION,
                )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chamfer_disabling_same_as_entry_keeps_exit_enabled() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        let mut operation = app.controller.operation_template(OpKind::Chamfer, 1);
        let Operation::Chamfer(chamfer) = &mut operation else {
            unreachable!()
        };
        chamfer.lead_in = Lead::ArcSweep {
            radius: 1.2,
            angle_deg: 60.,
        };
        chamfer.lead_out = Lead::None;
        chamfer.same_lead_out = true;
        chamfer.lead_out_enabled = None;
        app.controller.add_operation(operation);
        let id = app.controller.inspection_setup().operations[0].id();
        app.begin_operation_draft(id);
        let _ = app.update(Message::ChamferSameLeadOut(false));
        let Some(Operation::Chamfer(chamfer)) = &app.operation_draft else {
            panic!("chamfer draft")
        };
        assert!(!chamfer.same_lead_out);
        assert_eq!(
            chamfer.lead_out,
            Lead::ArcSweep {
                radius: 1.2,
                angle_deg: 60.
            }
        );
        assert_eq!(app.fields[&Field::LeadOutSize], "1.2");
        assert_eq!(
            app.fields[&Field::Profile(profile_parameters::Number::LeadOutSweep)],
            "60"
        );
    }

    #[test]
    fn modeled_chamfer_exposes_and_stages_lead_settings() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        let mut operation = app.controller.operation_template(OpKind::Chamfer, 1);
        let Operation::Chamfer(chamfer) = &mut operation else {
            unreachable!()
        };
        chamfer.faces = vec![cam_model::ChamferFace {
            upper: [[0., 0.], [20., 0.]],
            lower: [[0., -1.], [20., -1.]],
            top: 0.,
            bottom: -1.,
            outward: [0., -1.],
        }];
        app.controller.add_operation(operation);
        let id = app.controller.inspection_setup().operations[0].id();
        app.begin_operation_draft(id);
        assert!(app.chamfer_faces_active());
        assert_eq!(app.fields[&Field::ChamferTipOffset], "1");
        let _ = app.update(Message::FieldChanged(Field::ChamferTipOffset, "0.4".into()));
        assert!(app.fields.contains_key(&Field::LeadInSize));
        assert!(app.fields.contains_key(&Field::LeadOutSize));
        let _ = app.update(Message::FieldChanged(Field::LeadOutSize, "1.25".into()));
        let _ = app.update(Message::ChamferSameLeadOut(true));
        assert!(!app.fields.contains_key(&Field::LeadOutSize));
        let _ = app.update(Message::ChamferSameLeadOut(false));
        assert_eq!(app.fields[&Field::LeadOutSize], "1.25");
        assert!(
            matches!(&app.operation_draft, Some(Operation::Chamfer(op)) if (op.tip_offset - 0.4).abs() < 1e-8)
        );
    }

    #[test]
    fn collapsing_positions_cancels_picking_and_preserves_the_draft() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "sample.dxf").unwrap();
        app.controller.new_operation(OpKind::Pocket);
        let id = app
            .controller
            .inspection_setup()
            .operations
            .last()
            .unwrap()
            .id();
        app.setup_draft = None;
        app.begin_operation_draft(id);
        let _ = app.update(Message::OperationPositionPick(
            operation_positions::PositionKind::Entry,
        ));
        let _ = app.update(Message::OperationPositionPicked([0., 0., 0.]));
        let _ = app.update(Message::FieldChanged(Field::StartX, "12.5".into()));
        let draft = app.operation_draft.clone();
        let fields = app.fields.clone();
        let saved = app.controller.document().clone();
        let _ = app.update(Message::OperationPositionPick(
            operation_positions::PositionKind::Exit,
        ));
        let _ = app.update(Message::OperationSectionToggled(Section::Positions.id()));
        assert_eq!(app.operation_position_pick, None);
        let _ = app.update(Message::OperationPositionPicked([99., 99., 0.]));
        let _ = app.update(Message::OperationSectionToggled(Section::Positions.id()));
        assert!(!app
            .collapsed_operation_sections
            .contains(&Section::Positions.id()));
        assert_eq!(app.fields, fields);
        assert_eq!(app.operation_draft, draft);
        assert_eq!(app.controller.document(), &saved);
    }
}
