//! Shared DXF/STEP operation creation controls.
use super::*;
use iced::widget::column;
impl App {
    pub(super) fn creating_operation(&self) -> bool {
        self.cad_pending_kind.is_some()
            || (self.controller.pending_op().is_some() && self.drawing_editing_op.is_none())
    }
    pub(super) fn wizard_tool(&self) -> Option<cam_model::Tool> {
        if self.cad_pending_kind.is_some() {
            self.cad_selected_project_tool
                .as_ref()
                .or_else(|| {
                    self.library
                        .tools
                        .iter()
                        .find(|tool| Some(tool.number) == self.cad_tool_pick)
                })
                .cloned()
        } else {
            let number = self.controller.pending_op()?.tool?;
            self.operation_tools
                .iter()
                .chain(self.controller.inspection_setup().tools.iter())
                .find(|tool| tool.number == number)
                .cloned()
        }
    }
    pub(super) fn wizard_seeded_cutting(&self) -> Option<CuttingData> {
        let tool = self.wizard_tool()?;
        let defaults = self.controller.defaults();
        Some(CuttingData {
            rpm: tool.nominal_rpm,
            feed: if tool.nominal_feed > 0. {
                tool.nominal_feed
            } else {
                defaults.feed
            },
            plunge_feed: if tool.nominal_plunge_feed > 0. {
                tool.nominal_plunge_feed
            } else {
                defaults.plunge_feed
            },
        })
    }
    pub(super) fn seed_wizard_cutting(&mut self) {
        if let Some(cutting) = self.wizard_seeded_cutting() {
            self.fields.insert(Field::SpindleRpm, fmt_num(cutting.rpm));
            self.fields.insert(Field::Feed, fmt_num(cutting.feed));
            self.fields
                .insert(Field::PlungeFeed, fmt_num(cutting.plunge_feed));
            self.refresh_cutting_fields(None);
        }
    }
    pub(super) fn wizard_cutting(&self) -> Option<CuttingData> {
        let seeded = self.wizard_seeded_cutting()?;
        let parse = |field, fallback| {
            self.fields
                .get(&field)
                .map_or(Some(fallback), |text| text.parse::<f64>().ok())
                .filter(|value| value.is_finite())
        };
        let cutting = CuttingData {
            rpm: parse(Field::SpindleRpm, seeded.rpm)?,
            feed: parse(Field::Feed, seeded.feed)?,
            plunge_feed: parse(Field::PlungeFeed, seeded.plunge_feed)?,
        };
        (cutting.rpm >= 0. && cutting.feed > 0. && cutting.plunge_feed > 0.).then_some(cutting)
    }

    pub(super) fn seed_creation_editor(&mut self, kind: OpKind) {
        self.rotary_pick = None;
        self.rotary_wrap_draft = None;
        self.rotary_index_draft = None;
        self.rotary_index_mode = rotary_view::IndexMode::Manual;
        self.seed_rotary_index_values();
        self.cutting_force_inputs
            .remove(&(self.controller.inspection_setup().id, None));
        self.thread_parameters = thread_parameters::ThreadParameters::for_creation(kind);
        self.geometry_editor = None;
        self.creation_geometry_height_defaults = None;
        self.profile_contour_selection = 0;
        self.profile_side_overrides.clear();
        self.profile_entry_editor = None;
        self.operation_position_pick = None;
        self.operation_position_markers = [None; 2];
        self.operation_tab = OperationTab::Tool;
        self.creation_edited_fields.clear();
        self.inspector_active = true;
        self.operation_binding_preview = None;
        self.operation_geometry = None;
        self.operation_draft = Some(self.controller.operation_template(kind, 0));
        if let Some(op) = &mut self.operation_draft {
            if let Operation::Pocket(pocket) = op {
                pocket.clear.adaptive_enabled = Some(false);
                cam_model::OperationParameter::Engagement.set_source(op, "=tool_diameter * 0.4");
                cam_model::OperationParameter::PocketFinishFeed.set_source(op, "=feed");
                cam_model::OperationParameter::Stepdown
                    .set_source(op, pocket_parameters::DEFAULT_STEPDOWN);
                let stepover = cam_model::OperationParameter::MaximumStepover;
                stepover.set_source(op, stepover.default_expression());
            }
            if let Operation::Profile(_) = op {
                cam_model::OperationParameter::Stepdown
                    .set_source(op, pocket_parameters::DEFAULT_STEPDOWN);
            }
            if let Operation::Face(_) = op {
                cam_model::OperationParameter::Stepdown
                    .set_source(op, pocket_parameters::DEFAULT_STEPDOWN);
                cam_model::OperationParameter::FaceOverlap.set_source(op, "=tool_diameter * 0.5");
                cam_model::OperationParameter::FaceOvershoot
                    .set_source(op, "=tool_diameter / 2 + 0.1");
            }
            let fields = operation_fields(op);
            for (field, parameter, source) in [
                (
                    Field::RampFeed,
                    cam_model::OperationParameter::RampFeed,
                    "=plungeFeed",
                ),
                (
                    Field::LeadInFeed,
                    cam_model::OperationParameter::LeadInFeed,
                    "=feed",
                ),
                (
                    Field::LeadOutFeed,
                    cam_model::OperationParameter::LeadOutFeed,
                    "=feed",
                ),
            ] {
                if fields.contains(&field) {
                    parameter.set_source(op, source);
                }
            }
        }
        self.stock_to_leave_draft = Some(
            self.controller
                .pending_op()
                .and_then(|pending| pending.replacing)
                .map(|id| self.controller.inspection_setup().stock_to_leave_for(id))
                .unwrap_or_default(),
        );
        self.stock_to_leave_initialized = self.stock_to_leave_value().enabled;
        self.height_draft_id = Some(0);
        let settings = cam_model::OperationHeightSettings::default();
        self.height_offsets = cam_model::HeightPlaneKind::ALL
            .into_iter()
            .map(|p| (p, fmt_num(settings.definition(p).offset)))
            .collect();
        self.height_draft = Some(settings);
        self.height_disabled_feed_reference = None;
        self.sync_creation_tool();
        let work_offset = self.controller.active_origin();
        self.edit_operation_draft(|op| op.set_work_offset(work_offset));
        self.refresh_fields();
        self.seed_wizard_cutting();
        self.auto_operation_preset();
        self.sync_creation_geometry_heights();
    }

    /// Follow the selected surface while retaining every height definition the
    /// operator has edited, including edits made before picking geometry.
    pub(super) fn sync_creation_geometry_heights(&mut self) {
        if !matches!(
            self.cad_pending_kind,
            Some(
                OpKind::Pocket
                    | OpKind::Chamfer
                    | OpKind::Engrave
                    | OpKind::Carve
                    | OpKind::Bore
                    | OpKind::Thread
                    | OpKind::Adaptive
                    | OpKind::Parallel
                    | OpKind::Spiral
            )
        ) {
            return;
        }
        let operation = self
            .creation_operation()
            .ok()
            .map(|p| p.0)
            .or_else(|| self.operation_draft.clone());
        let Some(operation) = operation else {
            return;
        };
        let mut document = self.controller.document().clone();
        for tool in &self.operation_tools {
            if let Some(existing) = document
                .setup_mut()
                .tools
                .iter_mut()
                .find(|existing| existing.number == tool.number)
            {
                *existing = tool.clone();
            } else {
                document.setup_mut().tools.push(tool.clone());
            }
        }
        let defaults = crate::cad_operations::selection_height_defaults(&document, &operation);
        let previous = self
            .creation_geometry_height_defaults
            .clone()
            .unwrap_or_default();
        if let Some(settings) = &mut self.height_draft {
            for plane in cam_model::HeightPlaneKind::ALL {
                let old = previous.definition(plane);
                if settings.definition(plane) == old
                    && self
                        .height_offsets
                        .get(&plane)
                        .and_then(|s| s.parse::<f64>().ok())
                        == Some(old.offset)
                {
                    *settings.definition_mut(plane) = defaults.definition(plane).clone();
                    self.height_offsets
                        .insert(plane, fmt_num(defaults.definition(plane).offset));
                }
            }
        }
        self.creation_geometry_height_defaults = Some(defaults);
    }

    pub(super) fn sync_creation_tool(&mut self) {
        if let Some(tool) = self.wizard_tool() {
            let number = self.stage_operation_tool(tool);
            self.edit_operation_draft(|op| {
                op.set_tool(number);
                op.clear_tool_preset();
            });
            self.auto_operation_preset();
        }
        self.sync_creation_geometry_heights();
    }

    pub(super) fn creation_operation(
        &self,
    ) -> Result<(Operation, Option<cam_model::GeometryBinding>), String> {
        let mut draft = self
            .operation_draft
            .clone()
            .ok_or("Operation unavailable")?;
        self.wizard_tool().ok_or("Choose a cutter")?;
        self.wizard_cutting().ok_or("Enter valid cutting data")?;
        let fields = self
            .inspector_fields()
            .into_iter()
            .map(|field| {
                let value = self.parsed_field_value(field).map_err(|error| {
                    format!(
                        "Correct invalid operation fields: {} ({error})",
                        self.field_label(field)
                    )
                })?;
                if operation_tabs::operation_field_invalid(&draft, field, value) {
                    return Err(format!(
                        "Correct invalid operation fields: {} is outside its allowed range",
                        self.field_label(field)
                    ));
                }
                Ok((field, value))
            })
            .collect::<Result<BTreeMap<_, _>, _>>()?;
        apply_op_fields(&mut draft, &fields);
        self.apply_parameter_sources(&mut draft)?;
        if draft
            .rotary_wrap()
            .is_some_and(|wrap| !wrap.geometry.is_empty())
        {
            let geometry = crate::cad_operations::geometry_for_setup(self.controller.document())?;
            crate::cad_operations::resolve_wrapped_operation(&geometry, &mut draft)?;
            return Ok((draft, None));
        }
        if self.cad_pending_kind.is_some() && draft.rotary_wrap().is_some() {
            return Err("Select machining geometry on the Geometry tab".into());
        }
        if let Operation::Profile(profile) = &mut draft {
            if profile.selections.is_none() {
                profile.selections = self.active_profile_geometry_entries();
            }
            if profile.selections.is_some() {
                let binding = self.resolve_profile_geometry(profile)?;
                return Ok((draft, binding));
            }
        }
        if let Some(kind) = self.cad_pending_kind {
            let mut document = self.rotary_draft_document();
            for tool in &self.operation_tools {
                if let Some(existing) = document
                    .setup_mut()
                    .tools
                    .iter_mut()
                    .find(|t| t.number == tool.number)
                {
                    *existing = tool.clone();
                } else {
                    document.setup_mut().tools.push(tool.clone());
                }
            }
            let (geometry, mut binding) = crate::cad_operations::create_operation(
                &document,
                kind,
                self.controller.cad_selection().to_vec(),
                0,
                draft.tool(),
                self.controller.active_origin(),
                *self.controller.defaults(),
            )?;
            match (&mut draft, geometry) {
                (Operation::Profile(op), Operation::Profile(new)) => {
                    op.chain = new.chain;
                    op.closed = new.closed;
                    op.additional_chains = new.additional_chains;
                    op.side = new.side;
                    op.comp = new.comp;
                    self.apply_profile_side_overrides(op);
                }
                (Operation::Pocket(op), Operation::Pocket(new)) => {
                    op.boundary = new.boundary;
                    op.islands = new.islands;
                    op.additional_regions = new.additional_regions;
                }
                (Operation::Face(op), Operation::Face(new)) => op.boundary = new.boundary,
                (Operation::Drill(op), Operation::Drill(new)) => op.points = new.points,
                (Operation::Chamfer(op), Operation::Chamfer(new)) => {
                    op.chain = new.chain;
                    op.closed = new.closed;
                    if !op.closed {
                        op.start = None;
                        op.lead_overlap = 0.;
                    }
                    op.side = new.side;
                    op.top = new.top;
                    op.faces = new.faces;
                    if !op.faces.is_empty() {
                        op.width = new.width;
                        op.depth = new.depth;
                    }
                }
                (Operation::Engrave(op), Operation::Engrave(new)) => {
                    op.chain = new.chain;
                    op.closed = new.closed;
                    op.top = new.top;
                    if !op.closed {
                        op.start = None;
                    }
                }
                (Operation::Carve(op), Operation::Carve(new)) => {
                    op.boundary = new.boundary;
                    op.islands = new.islands;
                    op.top = new.top;
                }
                (Operation::Bore(op), Operation::Bore(new)) => {
                    op.points = new.points;
                    op.internal = new.internal;
                    op.diameter = new.diameter;
                    op.z_top = new.z_top;
                    op.z_bottom = new.z_bottom;
                }
                (Operation::Thread(op), Operation::Thread(new)) => {
                    op.points = new.points;
                    op.internal = new.internal;
                    if !self.creation_edited_fields.contains(&Field::MajorDia) {
                        op.major_dia = new.major_dia;
                    }
                    op.z_top = new.z_top;
                    op.z_bottom = new.z_bottom;
                }
                (Operation::Adaptive(_), Operation::Adaptive(_))
                | (Operation::Parallel(_), Operation::Parallel(_))
                | (Operation::Spiral(_), Operation::Spiral(_)) => {}
                _ => return Err("Geometry does not match operation".into()),
            }
            if binding.recipe == cam_model::ExtractionRecipe::PocketBoundary {
                binding.heights.bottom =
                    binding.heights.top - document.operation_base_cutting_heights(&draft).depth();
            } else if crate::cad_operations::surface_recipe(&binding.recipe) {
                binding.heights = document.operation_base_cutting_heights(&draft);
            }
            Ok((draft, Some(binding)))
        } else {
            let pending = self
                .controller
                .pending_op()
                .ok_or("Operation unavailable")?;
            if !self.controller.pending_ready() {
                return Err("Select compatible geometry and a cutter".into());
            }
            let mut operation = self.controller.operation_with_drawing_geometry(
                &draft,
                &self.controller.pending_operation_geometry(),
            )?;
            if let Operation::Profile(profile) = &mut operation {
                self.apply_profile_side_overrides(profile);
            }
            if self.creation_edited_fields.contains(&Field::BoreDiameter) {
                if let (Operation::Bore(op), Operation::Bore(edited)) = (&mut operation, &draft) {
                    op.diameter = edited.diameter;
                }
            }
            if self.creation_edited_fields.contains(&Field::MajorDia) {
                if let (Operation::Thread(op), Operation::Thread(edited)) = (&mut operation, &draft)
                {
                    op.major_dia = edited.major_dia;
                }
            }
            if operation_tabs::operation_start(&operation).is_none() {
                if let Some(start) = pending.start {
                    operation_tabs::set_operation_start(&mut operation, true);
                    operation_tabs::set_operation_start_coordinate(&mut operation, 0, start[0]);
                    operation_tabs::set_operation_start_coordinate(&mut operation, 1, start[1]);
                }
            }
            Ok((operation, None))
        }
    }

    pub(super) fn creation_ready(&self) -> bool {
        self.creation_operation()
            .and_then(|(op, binding)| {
                let mut document = self.rotary_draft_document();
                for tool in &self.operation_tools {
                    if let Some(existing) = document
                        .setup_mut()
                        .tools
                        .iter_mut()
                        .find(|t| t.number == tool.number)
                    {
                        *existing = tool.clone();
                    } else {
                        document.setup_mut().tools.push(tool.clone());
                    }
                }
                if let Some(binding) = binding {
                    document.geometry.bindings.insert(0, binding);
                }
                self.edited_height_settings()?
                    .resolve(&document, &op)
                    .map(|_| ())
            })
            .is_ok()
    }

    pub(super) fn creation_geometry_controls(&self) -> Element<'_, Message> {
        let kind = self
            .cad_pending_kind
            .or_else(|| self.controller.pending_op().map(|p| p.kind))
            .unwrap();
        let mut col = column![].spacing(10);
        if self.cad_pending_kind.is_some() {
            col =
                col.push(self.geometry_list(cad_geometry_labels(self.controller.cad_selection())));
            let valid = self.validate_active_profile_geometry().unwrap_or_else(|| {
                self.controller
                    .preview_cad_operation(kind, self.controller.cad_selection())
                    .map(|_| ())
            });
            if let Err(reason) = valid {
                if !self.controller.cad_selection().is_empty() {
                    col = col.push(text(reason).size(12).color(palette::WARN));
                }
            }
        } else {
            if op_uses_snaps(kind) {
                col = col.push(self.snap_toolbar());
            }
            col = col.push(self.geometry_list(drawing_geometry_labels(
                &self.controller.pending_operation_geometry(),
            )));
        }
        col.into()
    }
}
