//! Physical setup management. Geometry is shared; machining is evaluated in the
//! active setup orientation, with the datum translated only when posting.
use super::*;
use cam_model::{EntityRef, FrameAxisReference, SetupFrame};

pub(crate) fn validate_stock_to_leave(
    document: &Document,
    operation: &Operation,
    stock: cam_model::StockToLeave,
) -> Result<(), String> {
    if !stock.radial.is_finite()
        || !stock.axial.is_finite()
        || stock.radial < 0.0
        || stock.axial < 0.0
    {
        return Err("Stock to Leave values must be finite and nonnegative.".into());
    }
    if !stock.enabled {
        return Ok(());
    }
    let depth = match operation {
        Operation::Profile(op) => op.depth,
        Operation::Pocket(op) => op.depth,
        Operation::Face(op) => {
            if stock.radial > 0.0 {
                return Err("Radial stock to leave is not supported for Face; set it to 0 mm.".into());
            }
            op.depth
        }
        Operation::Adaptive(_) | Operation::Parallel(_)
 | Operation::Spiral(_) => document
            .geometry
            .bindings
            .get(&operation.id())
            .map(|binding| binding.effective_heights().depth())
            .ok_or("3D surface operations require linked solid geometry")?,
        _ => return Err("Stock to Leave is supported for Profile, Pocket, Face, Adaptive Clearing, Parallel, and Spiral.".into()),
    };
    if stock.axial > 0.0 && stock.axial >= depth {
        return Err(format!("Axial stock to leave ({:.3} mm) must be less than the operation depth ({depth:.3} mm).", stock.axial));
    }
    Ok(())
}

impl AppController {
    /// Set a fixed fourth-axis pose for any ordinary machining operation.
    pub fn set_operation_rotary_index(
        &mut self,
        id: u32,
        index: Option<cam_model::RotaryIndex>,
    ) -> Result<(), String> {
        let operation = self
            .document()
            .setup()
            .operations
            .iter()
            .find(|op| op.id() == id)
            .ok_or("Operation does not exist")?;
        if let Some(index) = &index {
            index.validate()?;
            if operation.rotary_wrap().is_some() {
                return Err("An operation cannot be both indexed and wrapped".into());
            }
        }
        if self.document().rotary_indexes.get(&id) == index.as_ref() {
            return Ok(());
        }
        self.edit_document(move |doc| {
            if let Some(index) = index {
                doc.rotary_indexes.insert(id, index);
            } else {
                doc.rotary_indexes.remove(&id);
            }
        });
        self.invalidate();
        Ok(())
    }
    pub fn active_setup_id(&self) -> u64 {
        self.active_setup_id
    }
    pub fn setups(&self) -> &[Setup] {
        &self.document().setups
    }

    pub fn activate_setup(&mut self, id: u64) -> Result<(), String> {
        if !self.setups().iter().any(|s| s.id == id) {
            return Err("Setup no longer exists".into());
        }
        if id == self.active_setup_id {
            return Ok(());
        }
        self.active_setup_id = id;
        self.document.update_view(|doc| doc.active_setup_id = id);
        self.active_origin = self.setup().origin_index;
        self.pending_op = None;
        self.selection = Selection::Setup;
        self.cad_selection.clear();
        self.invalidate();
        self.restore_saved_verification();
        Ok(())
    }

    pub(super) fn restore_active_setup_view(&mut self) {
        if !self.setups().iter().any(|s| s.id == self.active_setup_id) {
            self.active_setup_id = self.setups().first().map_or(0, |s| s.id);
        }
        let id = self.active_setup_id;
        self.document.update_view(|doc| doc.active_setup_id = id);
        if !self.setup().origin_indices().contains(&self.active_origin) {
            self.active_origin = self.setup().origin_index;
        }
    }

    pub fn add_setup(&mut self) -> u64 {
        let selected = self.cad_selection.iter().map(|r| r.instance_id).collect();
        self.add_setup_with_models(selected)
    }

    /// Seed a new setup with the selected body instances, without copying operations.
    pub fn add_setup_with_models(&mut self, selected: Vec<cam_model::InstanceId>) -> u64 {
        let mut setup = self.setup().clone();
        setup.id = self.document().next_setup_id().max(self.setup_id_floor);
        setup.name = format!("Setup {}", setup.id);
        setup.program_name = format!("setup-{}", setup.id);
        setup.model_instances = selected
            .into_iter()
            .filter(|id| self.geometry_state().instance(*id).is_some())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        setup.fixture_instances.clear();
        setup.operations.clear();
        setup.patterns.clear();
        setup.operation_heights.clear();
        setup.height_settings.clear();
        setup.stock_to_leave.clear();
        setup.stock = default_stock();
        setup.extra_origins.clear();
        let id = setup.id;
        self.edit_document(move |doc| doc.setups.push(setup));
        self.activate_setup(id).expect("new setup exists");
        id
    }

    pub fn duplicate_setup(&mut self, id: u64) -> Result<u64, String> {
        let mut copy = self
            .setups()
            .iter()
            .find(|s| s.id == id)
            .cloned()
            .ok_or("Missing setup")?;
        copy.id = self.document().next_setup_id().max(self.setup_id_floor);
        copy.name = format!("{} (copy)", copy.name);
        copy.program_name = format!("setup-{}", copy.id);
        // The copy is inserted immediately after its source and continues machining
        // the same workpiece, including material already removed by that setup.
        copy.stock = Stock::PreviousSetup;
        let mut next_id = self.next_op_id();
        let mut bindings = Vec::new();
        let mut linked = Vec::new();
        let mut operation_ids = std::collections::HashMap::new();
        let old_heights = std::mem::take(&mut copy.operation_heights);
        let old_settings = std::mem::take(&mut copy.height_settings);
        let old_stock_to_leave = std::mem::take(&mut copy.stock_to_leave);
        for op in &mut copy.operations {
            let old_id = op.id();
            if let Some(settings) = old_settings.get(&op.id()) {
                copy.height_settings.insert(next_id, settings.clone());
            }
            if let Some(heights) = old_heights.get(&op.id()) {
                copy.operation_heights.insert(next_id, *heights);
            }
            if let Some(stock) = old_stock_to_leave.get(&op.id()) {
                copy.stock_to_leave.insert(next_id, *stock);
            }
            if self.document().cad_linked_operations.contains(&op.id()) {
                linked.push(next_id);
            }
            if let Some(mut binding) = self.geometry_state().bindings.get(&op.id()).cloned() {
                binding.resolved_fingerprint = None;
                bindings.push((next_id, binding));
            }
            op.set_id(next_id);
            operation_ids.insert(old_id, next_id);
            next_id = next_id
                .checked_add(1)
                .ok_or("Operation ID range exhausted")?;
        }
        let mut next_pattern_id = self.next_pattern_id();
        for pattern in &mut copy.patterns {
            pattern.id = next_pattern_id;
            next_pattern_id = next_pattern_id
                .checked_add(1)
                .ok_or("Pattern ID range exhausted")?;
            for member in &mut pattern.members {
                *member = *operation_ids
                    .get(member)
                    .ok_or("Pattern refers to a missing operation")?;
            }
        }
        let new_id = copy.id;
        let excluded: Vec<u32> = self
            .document()
            .excluded_operations
            .iter()
            .filter_map(|old| operation_ids.get(old).copied())
            .collect();
        let disabled_origins = self.document().disabled_origins.get(&id).cloned();
        let indexes: Vec<_> = operation_ids
            .iter()
            .filter_map(|(old, new)| {
                self.document()
                    .rotary_indexes
                    .get(old)
                    .cloned()
                    .map(|index| (*new, index))
            })
            .collect();
        self.edit_document(move |doc| {
            doc.rotary_indexes.extend(indexes);
            let at = doc.setups.iter().position(|s| s.id == id).unwrap() + 1;
            doc.setups.insert(at, copy);
            doc.geometry.bindings.extend(bindings);
            doc.cad_linked_operations.extend(linked);
            doc.excluded_operations.extend(excluded);
            if let Some(disabled) = disabled_origins {
                doc.disabled_origins.insert(new_id, disabled);
            }
        });
        self.activate_setup(new_id)?;
        Ok(new_id)
    }

    pub fn delete_setup(&mut self, id: u64) -> Result<(), String> {
        if self.setups().is_empty() {
            return Err("Setup no longer exists".into());
        }
        if self.setups().len() == 1 {
            return Err("Keep at least one setup in the project".into());
        }
        if self.setups()[0].id == id && self.setups()[1].stock == Stock::PreviousSetup {
            return Err("The next setup uses this setup's stock. Choose a box stock for it before deleting this setup".into());
        }
        let removed = self
            .setups()
            .iter()
            .find(|s| s.id == id)
            .ok_or("Missing setup")?;
        let ids: BTreeSet<_> = removed.operations.iter().map(Operation::id).collect();
        self.edit_document(move |doc| {
            doc.setups.retain(|s| s.id != id);
            doc.excluded_operations.retain(|op| !ids.contains(op));
            doc.disabled_origins.remove(&id);
            doc.geometry.bindings.retain(|op, _| !ids.contains(op));
            doc.cad_linked_operations.retain(|op| !ids.contains(op));
            doc.rotary_indexes.retain(|op, _| !ids.contains(op));
            doc.operation_names.retain(|op, _| !ids.contains(op));
        });
        self.restore_active_setup_view();
        self.active_origin = self.setup().origin_index;
        self.pending_op = None;
        self.selection = Selection::Setup;
        self.cad_selection.clear();
        self.invalidate();
        Ok(())
    }

    pub fn move_setup(&mut self, id: u64, up: bool) {
        let Some(at) = self.setups().iter().position(|s| s.id == id) else {
            return;
        };
        let next = if up {
            at.checked_sub(1)
        } else {
            (at + 1 < self.setups().len()).then_some(at + 1)
        };
        if let Some(next) = next {
            if (next == 0 && self.setups()[at].stock == Stock::PreviousSetup)
                || (at == 0 && self.setups()[next].stock == Stock::PreviousSetup)
            {
                return;
            }
            self.edit_document(|doc| doc.setups.swap(at, next));
            self.invalidate();
        }
    }

    /// Put a setup immediately before another stable ID, or last with `None`.
    /// Invalid targets and unchanged order do not create an undo checkpoint.
    pub fn reorder_setup_to(&mut self, id: u64, before: Option<u64>) -> Result<(), String> {
        let from = self
            .setups()
            .iter()
            .position(|s| s.id == id)
            .ok_or("Missing setup")?;
        let target = match before {
            Some(target) => self
                .setups()
                .iter()
                .position(|s| s.id == target)
                .ok_or("Missing destination setup")?,
            None => self.setups().len(),
        };
        if before == Some(id) {
            return Ok(());
        }
        let at = target - usize::from(from < target);
        if from == at {
            return Ok(());
        }
        let first = if at == 0 {
            from
        } else if from == 0 {
            1
        } else {
            0
        };
        if self.setups()[first].stock == Stock::PreviousSetup {
            return Err("The first setup needs box stock; it has no previous setup".into());
        }
        self.edit_document(move |doc| {
            let setup = doc.setups.remove(from);
            doc.setups.insert(at, setup);
        });
        self.invalidate();
        Ok(())
    }

    /// Put an operation before a destination sibling, or append with `None`.
    /// Cross-setup moves use the same frame, CAD binding and tool validation as
    /// menu transfers, publishing the transfer and its order as one edit.
    pub fn reorder_operation_to(
        &mut self,
        id: u32,
        destination: u64,
        before: Option<u32>,
    ) -> Result<(), String> {
        let source = self
            .setups()
            .iter()
            .find(|s| s.operations.iter().any(|op| op.id() == id))
            .ok_or("Missing operation")?;
        let target = self
            .setups()
            .iter()
            .find(|s| s.id == destination)
            .ok_or("Missing destination setup")?;
        let position = match before {
            Some(target_id) => target
                .operations
                .iter()
                .position(|op| op.id() == target_id)
                .ok_or("Drop target is not in the destination setup")?,
            None => target.operations.len(),
        };
        let source_pattern = source.patterns.iter().find(|p| p.members.contains(&id));
        let target_pattern =
            before.and_then(|anchor| target.patterns.iter().find(|p| p.members.contains(&anchor)));
        if let Some(pattern) = source_pattern {
            if source.id != destination || target_pattern.map(|p| p.id) != Some(pattern.id) {
                return Err("Pattern members move within their group; dissolve the pattern before moving a source elsewhere".into());
            }
            if before == Some(id) {
                return Ok(());
            }
            let mut reordered = pattern.members.clone();
            reordered.retain(|member| *member != id);
            let insert_at = before
                .and_then(|anchor| reordered.iter().position(|member| *member == anchor))
                .unwrap_or(reordered.len());
            reordered.insert(insert_at, id);
            let first = target
                .operations
                .iter()
                .position(|op| op.id() == pattern.members[0])
                .unwrap();
            let pattern_id = pattern.id;
            let selected: BTreeSet<u32> = pattern.members.iter().copied().collect();
            let by_id: std::collections::HashMap<u32, Operation> = target
                .operations
                .iter()
                .filter(|op| selected.contains(&op.id()))
                .cloned()
                .map(|op| (op.id(), op))
                .collect();
            self.edit_document(move |doc| {
                let setup = doc.setups.iter_mut().find(|s| s.id == destination).unwrap();
                setup.operations.drain(first..first + reordered.len());
                setup.operations.splice(
                    first..first,
                    reordered.iter().filter_map(|id| by_id.get(id).cloned()),
                );
                setup
                    .patterns
                    .iter_mut()
                    .find(|p| p.id == pattern_id)
                    .unwrap()
                    .members = reordered;
            });
            self.invalidate();
            return Ok(());
        }
        if target_pattern.is_some() {
            return Err("A drop cannot insert an ungrouped operation into the middle of a pattern; edit pattern membership instead".into());
        }
        if source.id != destination {
            self.transfer_operation_to_setup_at(id, destination, false, before)?;
            return Ok(());
        }
        let from = target
            .operations
            .iter()
            .position(|op| op.id() == id)
            .unwrap();
        if let Some(anchor) =
            before.and_then(|anchor| target.operations.iter().find(|op| op.id() == anchor))
        {
            if anchor.work_offset() != target.operations[from].work_offset() {
                return Err("Operations execute in work-origin groups; use Move to Origin before reordering across groups".into());
            }
        }
        if before == Some(id) {
            return Ok(());
        }
        let at = position - usize::from(from < position);
        if at == from {
            return Ok(());
        }
        self.edit_document(move |doc| {
            let ops = &mut doc
                .setups
                .iter_mut()
                .find(|s| s.id == destination)
                .unwrap()
                .operations;
            let operation = ops.remove(from);
            ops.insert(at, operation);
        });
        self.invalidate();
        Ok(())
    }

    /// Reorder a tree source and change pattern membership in one undo step.
    /// A pattern header appends; a member row inserts before its anchor.
    pub fn move_operation_in_tree(
        &mut self,
        id: u32,
        destination: u64,
        before: Option<u32>,
        pattern: Option<u64>,
    ) -> Result<(), String> {
        let source = self
            .setups()
            .iter()
            .find(|s| s.operations.iter().any(|op| op.id() == id))
            .ok_or("Missing operation")?;
        if source.id != destination {
            if pattern.is_some() {
                return Err(
                    "Move the operation to the destination setup before adding it to a pattern"
                        .into(),
                );
            }
            return self.reorder_operation_to(id, destination, before);
        }
        let original = source.clone();
        let mut setup = original.clone();
        let from = setup
            .operations
            .iter()
            .position(|op| op.id() == id)
            .unwrap();
        if let Some(anchor) = before {
            let anchor = setup
                .operations
                .iter()
                .find(|op| op.id() == anchor)
                .ok_or("Missing drop target")?;
            if anchor.work_offset() != setup.operations[from].work_offset() {
                return Err("Operations execute in work-origin groups; use Move to Origin before reordering across groups".into());
            }
        }
        if let Some(pattern_id) = pattern {
            let target = setup
                .patterns
                .iter()
                .find(|p| p.id == pattern_id)
                .ok_or("Missing pattern")?;
            if target
                .members
                .iter()
                .filter(|member| **member != id)
                .any(|member| {
                    setup.operations.iter().any(|op| {
                        op.id() == *member
                            && op.work_offset() != setup.operations[from].work_offset()
                    })
                })
            {
                return Err("Pattern sources must share one work datum".into());
            }
        }
        if before == Some(id) {
            return Ok(());
        }
        let operation = setup.operations.remove(from);
        for group in &mut setup.patterns {
            group.members.retain(|member| *member != id);
        }
        let mut insertion = before
            .and_then(|anchor| setup.operations.iter().position(|op| op.id() == anchor))
            .unwrap_or(setup.operations.len());
        if let Some(pattern_id) = pattern {
            let group = setup
                .patterns
                .iter_mut()
                .find(|p| p.id == pattern_id)
                .unwrap();
            let member_at = before
                .and_then(|anchor| group.members.iter().position(|member| *member == anchor))
                .unwrap_or(group.members.len());
            if member_at == group.members.len() {
                if let Some(last) = group.members.last() {
                    insertion = setup
                        .operations
                        .iter()
                        .position(|op| op.id() == *last)
                        .unwrap()
                        + 1;
                }
            }
            group.members.insert(member_at, id);
        }
        setup.patterns.retain(|p| !p.members.is_empty());
        setup.operations.insert(insertion, operation);
        // Preserve each group's contiguous execution block when extracting a source.
        for group in &setup.patterns {
            let first = setup
                .operations
                .iter()
                .position(|op| group.members.contains(&op.id()))
                .unwrap();
            let members: Vec<_> = group
                .members
                .iter()
                .filter_map(|id| setup.operations.iter().find(|op| op.id() == *id).cloned())
                .collect();
            setup
                .operations
                .retain(|op| !group.members.contains(&op.id()));
            setup.operations.splice(first..first, members);
        }
        if setup == original {
            return Ok(());
        }
        self.edit_document(move |doc| {
            *doc.setups.iter_mut().find(|s| s.id == destination).unwrap() = setup
        });
        self.invalidate();
        Ok(())
    }

    /// Transfer an operation definition and its CAD link, resolving in the
    /// destination frame before publishing a single undoable edit.
    pub fn transfer_operation_to_setup(
        &mut self,
        id: u32,
        destination: u64,
        copy: bool,
    ) -> Result<u32, String> {
        self.transfer_operation_to_setup_at(id, destination, copy, None)
    }

    fn transfer_operation_to_setup_at(
        &mut self,
        id: u32,
        destination: u64,
        copy: bool,
        before: Option<u32>,
    ) -> Result<u32, String> {
        let source = self
            .setups()
            .iter()
            .find(|s| s.operations.iter().any(|op| op.id() == id))
            .ok_or("Missing operation")?;
        if source
            .patterns
            .iter()
            .any(|pattern| pattern.members.contains(&id))
        {
            return Err(
                "Dissolve the pattern before transferring or copying one of its source operations"
                    .into(),
            );
        }
        let target = self
            .setups()
            .iter()
            .find(|s| s.id == destination)
            .ok_or("Missing destination setup")?;
        let insertion = match before {
            Some(before) => target
                .operations
                .iter()
                .position(|op| op.id() == before)
                .ok_or("Drop target is not in the destination setup")?,
            None => target.operations.len(),
        };
        if before.is_some() && target.operations[insertion].work_offset() != target.origin_index {
            return Err("Transfer to the setup's main work origin first; use Move to Origin for another work-origin group".into());
        }
        if source.id == destination && !copy {
            return Ok(id);
        }
        if !self.geometry_state().bindings.contains_key(&id)
            && source.frame.world_from_setup != target.frame.world_from_setup
        {
            return Err("An operation with explicit planar geometry needs the same setup axes; create a linked CAD operation for another orientation".into());
        }
        if self.document().rotary_indexes.contains_key(&id)
            && source.frame.world_from_setup != target.frame.world_from_setup
        {
            return Err("Clear the rotary index before transferring to different setup axes, then define the index in the destination setup".into());
        }
        let source_id = source.id;
        let mut definition = source.clone();
        definition.operations.retain(|op| op.id() == id);
        let required = definition.operations[0].tools();
        definition
            .tools
            .retain(|tool| required.contains(&tool.number));
        if definition.tools.len() != required.len() {
            return Err("Source operation has a missing tool".into());
        }
        let mut reserved: BTreeSet<_> = target.tools.iter().map(|tool| tool.number).collect();
        let mut mapping = std::collections::BTreeMap::new();
        for tool in &mut definition.tools {
            let old = tool.number;
            let number = if let Some(existing) = target
                .tools
                .iter()
                .find(|existing| existing.identity() == tool.identity())
            {
                existing.number
            } else {
                let mut number = old.max(1);
                while reserved.contains(&number) {
                    number = number.checked_add(1).ok_or("Tool number range exhausted")?;
                }
                number
            };
            reserved.insert(number);
            mapping.insert(old, number);
            tool.number = number;
        }
        for op in &mut definition.operations {
            op.map_tools(|old| mapping.get(&old).copied().unwrap_or(old));
        }
        let heights = source.heights_for(id);
        let settings = source.height_settings.get(&id).cloned();
        let stock_to_leave = source.stock_to_leave_for(id);
        let mut operation = definition.operations.remove(0);
        let new_id = if copy { self.next_op_id() } else { id };
        operation.set_id(new_id);
        operation.set_work_offset(target.origin_index);
        let mut document = self.document().clone();
        if copy {
            if let Some(index) = document.rotary_indexes.get(&id).cloned() {
                document.rotary_indexes.insert(new_id, index);
            }
            if document.cad_linked_operations.contains(&id) {
                document.cad_linked_operations.insert(new_id);
            }
            if let Some(mut binding) = document.geometry.bindings.get(&id).cloned() {
                binding.resolved_fingerprint = None;
                document.geometry.bindings.insert(new_id, binding);
            }
        } else {
            document
                .setups
                .iter_mut()
                .find(|s| s.id == source_id)
                .unwrap()
                .operations
                .retain(|op| op.id() != id);
            if let Some(binding) = document.geometry.bindings.get_mut(&id) {
                binding.resolved_fingerprint = None;
            }
        }
        if !copy {
            document
                .setups
                .iter_mut()
                .find(|s| s.id == source_id)
                .unwrap()
                .operation_heights
                .remove(&id);
        }
        if !copy {
            document
                .setups
                .iter_mut()
                .find(|s| s.id == source_id)
                .unwrap()
                .height_settings
                .remove(&id);
        }
        document.active_setup_id = destination;
        let target = document.setup_mut();
        if !copy {
            target.stock_to_leave.remove(&id);
        }
        if stock_to_leave.enabled {
            target.stock_to_leave.insert(new_id, stock_to_leave);
        } else {
            target.stock_to_leave.remove(&new_id);
        }
        for tool in definition.tools {
            if !target
                .tools
                .iter()
                .any(|existing| existing.number == tool.number)
            {
                target.tools.push(tool);
            }
        }
        if let Some(settings) = settings {
            target.height_settings.insert(new_id, settings);
        }
        target.operation_heights.insert(new_id, heights);
        target.operations.insert(insertion, operation);
        if document.geometry.bindings.contains_key(&new_id) {
            crate::cad_operations::validate_binding_edit(&mut document, new_id)?;
        }
        self.edit_document(move |current| *current = document);
        self.active_setup_id = destination;
        self.active_origin = self.setup().origin_index;
        self.pending_op = None;
        self.select(Selection::Operation(new_id));
        self.invalidate();
        Ok(new_id)
    }

    /// Commit an entire operation inspector draft, including linked-height
    /// overrides, without publishing any intermediate dropdown/checkbox edit.
    pub fn apply_operation_settings(
        &mut self,
        operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
    ) -> Result<(), String> {
        self.apply_operation_settings_with_tools(operation, overrides, Vec::new())
    }

    pub fn apply_operation_settings_with_tools(
        &mut self,
        operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
        tools: Vec<Tool>,
    ) -> Result<(), String> {
        self.apply_operation_settings_with_geometry(operation, overrides, tools, None)
    }

    /// Commit every operation tab and its geometry field in one undo transaction.
    pub fn apply_operation_settings_with_geometry(
        &mut self,
        operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
        tools: Vec<Tool>,
        geometry: Option<Vec<EntityRef>>,
    ) -> Result<(), String> {
        self.apply_operation_settings_with_heights(operation, overrides, tools, geometry, None)
    }

    /// Apply safety planes together with every operation tab in one undo step.
    pub fn apply_operation_settings_with_heights(
        &mut self,
        operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
        tools: Vec<Tool>,
        geometry: Option<Vec<EntityRef>>,
        heights: Option<Heights>,
    ) -> Result<(), String> {
        let document =
            self.prepare_operation_settings(operation, overrides, tools, geometry, heights, None)?;
        if document == *self.document() {
            return Ok(());
        }
        self.edit_document(move |current| *current = document);
        self.invalidate();
        Ok(())
    }

    /// Commit operation fields, safety heights, and Stock to Leave in one undo step.
    pub fn apply_operation_settings_with_heights_and_stock(
        &mut self,
        operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
        tools: Vec<Tool>,
        geometry: Option<Vec<EntityRef>>,
        heights: Option<Heights>,
        stock_to_leave: cam_model::StockToLeave,
    ) -> Result<(), String> {
        let index = self.document().rotary_indexes.get(&operation.id()).cloned();
        self.apply_operation_settings_with_heights_and_stock_and_index(
            operation,
            overrides,
            tools,
            geometry,
            heights,
            stock_to_leave,
            index,
        )
    }

    #[allow(clippy::too_many_arguments)] // One atomic edit across the operation tabs.
    pub fn apply_operation_settings_with_heights_and_stock_and_index(
        &mut self,
        operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
        tools: Vec<Tool>,
        geometry: Option<Vec<EntityRef>>,
        heights: Option<Heights>,
        stock_to_leave: cam_model::StockToLeave,
        index: Option<cam_model::RotaryIndex>,
    ) -> Result<(), String> {
        let id = operation.id();
        let mut document = self.prepare_operation_settings(
            operation,
            overrides,
            tools,
            geometry,
            heights,
            Some(index),
        )?;
        if stock_to_leave.enabled {
            let operation = document
                .setup()
                .operations
                .iter()
                .find(|op| op.id() == id)
                .unwrap();
            validate_stock_to_leave(&document, operation, stock_to_leave)?;
        }
        if stock_to_leave.enabled {
            document
                .setup_mut()
                .stock_to_leave
                .insert(id, stock_to_leave);
        } else {
            document.setup_mut().stock_to_leave.remove(&id);
        }
        if document != *self.document() {
            self.edit_document(move |current| *current = document);
            self.invalidate();
        }
        Ok(())
    }

    /// Commit references, offsets, geometry and cutting parameters atomically.
    pub fn apply_operation_height_settings(
        &mut self,
        operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
        tools: Vec<Tool>,
        geometry: Option<Vec<EntityRef>>,
        mut settings: cam_model::OperationHeightSettings,
    ) -> Result<(), String> {
        settings.resolved = None;
        let id = operation.id();
        let mut document =
            self.prepare_operation_settings(operation, overrides, tools, geometry, None, None)?;
        crate::cad_operations::resolve_height_settings(
            &document,
            document
                .setup()
                .operations
                .iter()
                .find(|o| o.id() == id)
                .unwrap(),
            &settings,
        )?;
        document.setup_mut().height_settings.insert(id, settings);
        if document != *self.document() {
            self.edit_document(move |current| *current = document);
            self.invalidate();
        }
        Ok(())
    }

    /// Commit references, offsets, geometry, heights, cutting parameters and allowance atomically.
    pub fn apply_operation_height_settings_and_stock(
        &mut self,
        operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
        tools: Vec<Tool>,
        geometry: Option<Vec<EntityRef>>,
        settings: cam_model::OperationHeightSettings,
        stock_to_leave: cam_model::StockToLeave,
    ) -> Result<(), String> {
        let index = self.document().rotary_indexes.get(&operation.id()).cloned();
        self.apply_operation_height_settings_and_stock_and_index(
            operation,
            overrides,
            tools,
            geometry,
            settings,
            stock_to_leave,
            index,
        )
    }

    #[allow(clippy::too_many_arguments)] // One atomic edit across the operation tabs.
    pub fn apply_operation_height_settings_and_stock_and_index(
        &mut self,
        operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
        tools: Vec<Tool>,
        geometry: Option<Vec<EntityRef>>,
        mut settings: cam_model::OperationHeightSettings,
        stock_to_leave: cam_model::StockToLeave,
        index: Option<cam_model::RotaryIndex>,
    ) -> Result<(), String> {
        settings.resolved = None;
        let id = operation.id();
        let mut document = self.prepare_operation_settings(
            operation,
            overrides,
            tools,
            geometry,
            None,
            Some(index),
        )?;
        crate::cad_operations::resolve_height_settings(
            &document,
            document
                .setup()
                .operations
                .iter()
                .find(|o| o.id() == id)
                .unwrap(),
            &settings,
        )?;
        document.setup_mut().height_settings.insert(id, settings);
        if stock_to_leave.enabled {
            let operation = document
                .setup()
                .operations
                .iter()
                .find(|op| op.id() == id)
                .unwrap();
            validate_stock_to_leave(&document, operation, stock_to_leave)?;
            document
                .setup_mut()
                .stock_to_leave
                .insert(id, stock_to_leave);
        } else {
            document.setup_mut().stock_to_leave.remove(&id);
        }
        if document != *self.document() {
            self.edit_document(move |current| *current = document);
            self.invalidate();
        }
        Ok(())
    }

    fn prepare_operation_settings(
        &self,
        mut operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
        tools: Vec<Tool>,
        geometry: Option<Vec<EntityRef>>,
        safety_heights: Option<Heights>,
        index_override: Option<Option<cam_model::RotaryIndex>>,
    ) -> Result<Document, String> {
        let id = operation.id();
        let old = self
            .operation(id)
            .ok_or("Operation is not in the active setup")?;
        if self.document().cad_linked_operations.contains(&id)
            && !self.geometry_state().bindings.contains_key(&id)
        {
            return Err("Linked CAD geometry is missing; repair the geometry before applying operation settings".into());
        }
        let mut document = self.document().clone();
        if let Some(index) = index_override {
            if let Some(index) = index {
                index.validate()?;
                if operation.rotary_wrap().is_some() {
                    return Err("An operation cannot be both indexed and wrapped".into());
                }
                document.rotary_indexes.insert(id, index);
            } else {
                document.rotary_indexes.remove(&id);
            }
        }
        for tool in tools {
            if let Some(existing) = document
                .setup()
                .tools
                .iter()
                .find(|existing| existing.number == tool.number)
            {
                if existing.identity() != tool.identity() {
                    return Err(
                        "A draft tool number conflicts with the setup; choose the tool again"
                            .into(),
                    );
                }
                let index = document
                    .setup()
                    .tools
                    .iter()
                    .position(|t| t.number == tool.number)
                    .unwrap();
                document.setup_mut().tools[index] = tool;
            } else {
                document.setup_mut().tools.push(tool);
            }
        }
        if operation
            .tools()
            .iter()
            .any(|number| !document.setup().tools.iter().any(|t| t.number == *number))
        {
            return Err("Operation refers to a missing tool".into());
        }
        operation.resolve_parameter_expressions(&document.setup().tools)?;
        let before = Some(document.operation_base_cutting_heights(old));
        if let Some(heights) = safety_heights {
            heights.validate()?;
            document.setup_mut().height_settings.remove(&id);
            if heights != document.setup().heights_for(id) {
                document.setup_mut().operation_heights.insert(id, heights);
            }
        }
        let heights = document.setup().heights_for(id);
        if operation.rotary_wrap().is_some() {
            let geometry = crate::cad_operations::geometry_for_operation(&document, id)?;
            crate::cad_operations::resolve_wrapped_operation(&geometry, &mut operation)?;
            document.geometry.bindings.remove(&id);
            document.cad_linked_operations.remove(&id);
            let saved = document
                .setup_mut()
                .operations
                .iter_mut()
                .find(|op| op.id() == id)
                .ok_or("Operation missing")?;
            *saved = operation;
            return Ok(document);
        }
        let geometry = geometry
            .filter(|_| !matches!(&operation,Operation::Profile(p) if p.selections.is_some()));
        if let Some(selected) = &geometry {
            if matches!(
                operation,
                Operation::Adaptive(_) | Operation::Parallel(_) | Operation::Spiral(_)
            ) {
                cam_geometry3d::validate_adaptive_selection(&document.geometry, selected)?;
            } else {
                crate::cad_operations::validate_selection(&document.geometry, selected)?;
            }
            let profile_recipe = if matches!(operation, Operation::Profile(_)) {
                let geometry = crate::cad_operations::geometry_for_operation(&document, id)?;
                let recipe = if let Some(binding) = document
                    .geometry
                    .bindings
                    .get(&id)
                    .filter(|b| b.selected == *selected)
                {
                    binding.recipe.clone()
                } else {
                    crate::cad_operations::recipe(OpKind::Profile, &geometry, selected)?
                };
                if document
                    .geometry
                    .bindings
                    .get(&id)
                    .is_some_and(|b| b.selected != *selected)
                {
                    let binding = cam_geometry3d::resolve_selection_advisory(
                        &geometry,
                        selected,
                        recipe.clone(),
                        0.,
                    )?;
                    let input = cam_geometry3d::resolve_binding_advisory(&geometry, &binding)?;
                    if let Operation::Profile(profile) = &mut operation {
                        let previous: Vec<_> = profile.contours().collect();
                        let mut contours = input.profiles;
                        if contours.is_empty() {
                            let side = match recipe {
                                cam_model::ExtractionRecipe::ProfileLoop { side } => side,
                                _ => profile.side,
                            };
                            contours.push(cam_model::ProfileContour {
                                chain: input
                                    .regions
                                    .first()
                                    .ok_or("No profile contour")?
                                    .outer()
                                    .clone(),
                                closed: true,
                                side,
                            });
                        }
                        for contour in &mut contours {
                            if let Some(edited) = previous.iter().find(|old| {
                                old.chain == contour.chain && old.closed == contour.closed
                            }) {
                                contour.side = edited.side;
                            }
                        }
                        let first = contours.remove(0);
                        profile.chain = first.chain;
                        profile.closed = first.closed;
                        profile.side = first.side;
                        profile.additional_chains = contours;
                    }
                }
                Some(recipe)
            } else {
                let geometry = crate::cad_operations::geometry_for_operation(&document, id)?;
                Some(crate::cad_operations::recipe(
                    Self::operation_kind(&operation),
                    &geometry,
                    selected,
                )?)
            };
            let binding = document
                .geometry
                .bindings
                .get_mut(&id)
                .ok_or("Geometry associations are available only for CAD operations")?;
            if let Some(recipe) = profile_recipe {
                binding.recipe = recipe;
            }
            binding.selected = selected.clone();
            binding.status = cam_model::ResolutionStatus::Resolved;
            binding.resolved_fingerprint = None;
        }
        if let Some(binding) = document.geometry.bindings.get_mut(&id) {
            let previous_overrides = binding.overrides.clone();
            crate::cad_operations::capture_height_override(
                Some(binding),
                before,
                &operation,
                &heights,
            );
            if let Some(overrides) = overrides {
                // Unchanged linked-height fields must not erase a depth edit
                // captured from the ordinary numeric operation controls.
                if overrides != previous_overrides {
                    binding.overrides = overrides;
                }
            }
            binding.resolved_fingerprint = None;
        } else if overrides.is_some() {
            return Err("Height links are available only for CAD operations".into());
        }
        if let Operation::Profile(profile) = &mut operation {
            if profile.selections.is_some() {
                let previous = document
                    .geometry
                    .bindings
                    .get(&id)
                    .map(|b| b.overrides.clone());
                if let Some(mut binding) =
                    crate::cad_operations::resolve_profile_entries(&document, profile)?
                {
                    if let Some(overrides) = previous {
                        binding.overrides = overrides;
                    }
                    document.geometry.bindings.insert(id, binding);
                    document.cad_linked_operations.insert(id);
                } else {
                    document.geometry.bindings.remove(&id);
                    document.cad_linked_operations.remove(&id);
                }
            }
        }
        *document
            .setup_mut()
            .operations
            .iter_mut()
            .find(|op| op.id() == id)
            .unwrap() = operation;
        if document.geometry.bindings.contains_key(&id) {
            crate::cad_operations::validate_binding_edit(&mut document, id)?;
        }
        if let Some(after @ Operation::Chamfer(_)) =
            document.setup().operations.iter().find(|op| op.id() == id)
        {
            if matches!(old, Operation::Chamfer(_)) {
                let previous =
                    crate::cad_operations::selection_height_defaults(self.document(), old);
                let defaults = crate::cad_operations::selection_height_defaults(&document, after);
                if let Some(settings) = document.setup_mut().height_settings.get_mut(&id) {
                    for plane in cam_model::HeightPlaneKind::ALL {
                        if settings.definition(plane) == previous.definition(plane) {
                            *settings.definition_mut(plane) = defaults.definition(plane).clone();
                        }
                    }
                }
            }
        }
        Ok(document)
    }

    /// Preview the exact prospective transaction, including staged cutter and heights.
    pub fn preview_operation_settings_with_geometry(
        &self,
        operation: Operation,
        overrides: Option<cam_model::GeometryOverrides>,
        tools: Vec<Tool>,
        selected: Vec<EntityRef>,
    ) -> Result<cam_model::GeometryBinding, String> {
        let id = operation.id();
        let document = self.prepare_operation_settings(
            operation,
            overrides,
            tools,
            Some(selected),
            None,
            None,
        )?;
        Ok(document.geometry.bindings[&id].clone())
    }

    pub fn preview_operation_geometry(
        &self,
        id: u32,
        selected: Vec<EntityRef>,
    ) -> Result<(), String> {
        self.preview_operation_binding(id, selected).map(|_| ())
    }

    /// Resolve replacement sources without changing the document or its history.
    pub fn preview_operation_binding(
        &self,
        id: u32,
        selected: Vec<EntityRef>,
    ) -> Result<cam_model::GeometryBinding, String> {
        let operation = self.operation(id).cloned().ok_or("Operation unavailable")?;
        let document = self.prepare_operation_settings(
            operation,
            None,
            Vec::new(),
            Some(selected),
            None,
            None,
        )?;
        Ok(document.geometry.bindings[&id].clone())
    }

    /// Validate every tab before a single undoable commit. The caller owns the
    /// draft; rejected settings leave committed state and toolpaths untouched.
    pub fn apply_setup_settings(&mut self, mut setup: Setup) -> Result<(), String> {
        let old = self
            .setups()
            .iter()
            .find(|s| s.id == setup.id)
            .ok_or("Missing setup")?;
        if setup.id != self.active_setup_id {
            return Err("Activate the setup before editing its settings".into());
        }
        if old.operations != setup.operations || old.tools != setup.tools {
            return Err("Operations or tools changed while editing; reopen setup settings".into());
        }
        if setup.name.trim().is_empty() {
            return Err("Enter a setup name".into());
        }
        self.validate_setup_frame(&setup)?;
        setup.validate_fixtures(self.geometry_state())?;
        if !setup.origin.iter().all(|v| v.is_finite()) {
            return Err("Origin must contain finite coordinates".into());
        }
        if setup.origin_index == 0
            || setup
                .extra_origins
                .iter()
                .any(|o| o.index == setup.origin_index)
        {
            return Err(
                "Choose a positive work offset distinct from the legacy extra origins".into(),
            );
        }
        if ![
            setup.heights.clearance,
            setup.heights.retract,
            setup.heights.top_of_stock,
        ]
        .iter()
        .all(|v| v.is_finite())
            || setup.heights.clearance < setup.heights.retract
            || setup.heights.retract < setup.heights.top_of_stock
        {
            return Err(
                "Clearance must be at or above retract, and retract at or above stock top".into(),
            );
        }
        for settings in setup.height_settings.values() {
            settings.validate_structure()?;
        }
        for heights in setup.operation_heights.values() {
            heights.validate()?;
        }
        if let Some(height) = setup.tool_change_height {
            if !height.is_finite() || height < setup.heights.clearance {
                return Err("Tool-change height must be at or above clearance".into());
            }
        }
        if setup
            .model_instances
            .iter()
            .any(|id| self.geometry_state().instance(*id).is_none())
        {
            return Err("A selected machining model no longer exists".into());
        }
        let (min, max) = self.pending_setup_model_bounds(&setup);
        match setup.stock {
            Stock::FromSolid { instance }
                if instance
                    .and_then(|id| self.geometry_state().instance(id))
                    .and_then(|i| {
                        self.geometry_state().asset(i.asset_id).and_then(|a| {
                            a.topology
                                .bodies
                                .iter()
                                .find(|b| b.id == i.body_id && b.valid_solid)
                        })
                    })
                    .is_none() =>
            {
                return Err("Select an available solid body for stock".into());
            }
            Stock::PreviousSetup if self.setups()[0].id == setup.id => {
                return Err("The first setup has no previous setup; choose box stock".into());
            }
            Stock::FixedBox {
                size,
                model_position,
            } if size.iter().any(|v| !v.is_finite() || *v <= 0.)
                || model_position
                    .iter()
                    .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v)) =>
            {
                return Err(
                    "Choose positive stock dimensions and model positions within the stock".into(),
                )
            }
            Stock::RelativeBox { offsets } if offsets.iter().flatten().any(|v| !v.is_finite()) => {
                return Err("Stock offsets must be finite".into());
            }
            Stock::FixedRound {
                diameter,
                inner_diameter,
                length,
                position,
                tube,
                ..
            } if !diameter.is_finite()
                || diameter <= 0.
                || !length.is_finite()
                || length <= 0.
                || !position.is_finite()
                || !(0. ..=1.).contains(&position)
                || !inner_diameter.is_finite()
                || inner_diameter < 0.
                || (tube && (inner_diameter <= 0. || inner_diameter >= diameter)) =>
            {
                return Err("Choose positive round stock dimensions and an inner diameter smaller than the outer diameter".into());
            }
            Stock::RelativeRound {
                radial,
                inner_diameter,
                bottom,
                top,
                tube,
                ..
            } if ![radial, inner_diameter, bottom, top]
                .iter()
                .all(|v| v.is_finite() && *v >= 0.)
                || (tube && inner_diameter <= 0.) =>
            {
                return Err("Choose non-negative allowances and a valid inner diameter".into());
            }
            Stock::BoundingBox {
                x_offset,
                y_offset,
                top,
                thickness,
            } if ![x_offset, y_offset, top, thickness]
                .iter()
                .all(|v| v.is_finite())
                || thickness <= 0. =>
            {
                return Err("Stock dimensions must be finite and thickness positive".into())
            }
            _ => {}
        }
        if let Some((_, lo, hi)) = self.document().round_stock_geometry(&setup)? {
            if let Stock::RelativeRound {
                inner_diameter,
                tube: true,
                ..
            } = setup.stock
            {
                if inner_diameter >= hi[0] - lo[0] {
                    return Err("The inner diameter must be smaller than the outer diameter".into());
                }
            }
        }
        let (stock_min, stock_max) = self.pending_setup_stock_bounds(&setup);
        if (0..3).any(|i| {
            !stock_min[i].is_finite() || !stock_max[i].is_finite() || stock_max[i] <= stock_min[i]
        }) {
            return Err("Stock must have finite, positive dimensions on all axes".into());
        }
        // A negative relative offset deliberately puts a stock face inside the model bounds.
        if !matches!(
            setup.stock,
            Stock::RelativeBox { .. } | Stock::PreviousSetup
        ) && !self.geometry_state().instances.is_empty()
            && (0..3).any(|i| stock_min[i] > min[i] + 1e-6 || stock_max[i] < max[i] - 1e-6)
        {
            return Err("Stock does not contain the selected machining model".into());
        }
        let previous_offset = old.origin_index;
        if setup.origin_index != previous_offset {
            for operation in &mut setup.operations {
                if operation.work_offset() == previous_offset {
                    operation.set_work_offset(setup.origin_index);
                }
            }
        }
        if *old == setup {
            return Ok(());
        }
        self.edit_document(move |doc| {
            let slot = doc.setups.iter_mut().find(|s| s.id == setup.id).unwrap();
            *slot = setup;
            crate::cad_operations::refresh_binding_heights(doc);
        });
        self.active_origin = self.setup().origin_index;
        self.invalidate();
        Ok(())
    }

    pub fn pending_frame_from_model(
        &self,
        instance_id: u64,
    ) -> Result<(SetupFrame, [f64; 3]), String> {
        let frame = SetupFrame::from_model_instance(self.geometry_state(), instance_id)?;
        let instance = self
            .geometry_state()
            .instance(instance_id)
            .ok_or("Missing model instance")?;
        let origin = frame.to_setup_vector(instance.placement.translation);
        Ok((frame, origin))
    }

    pub fn pending_frame_from_edges(
        &self,
        references: [Option<EntityRef>; 3],
        flips: [bool; 3],
    ) -> Result<SetupFrame, String> {
        let [x, y, z] = references;
        self.setup().frame.resolve_references(
            self.geometry_state(),
            cam_model::FrameReferences {
                x: x.map(|entity| FrameAxisReference {
                    entity,
                    flipped: flips[0],
                }),
                y: y.map(|entity| FrameAxisReference {
                    entity,
                    flipped: flips[1],
                }),
                z: z.map(|entity| FrameAxisReference {
                    entity,
                    flipped: flips[2],
                }),
                ..Default::default()
            },
        )
    }

    pub(super) fn validate_setup_frame(&self, setup: &Setup) -> Result<(), String> {
        setup.frame.validate()?;
        self.document().round_stock_geometry(setup)?;
        let refs = &setup.frame.references;
        let linked = if refs.x.is_some()
            || refs.y.is_some()
            || refs.z.is_some()
            || refs.stock_axes.iter().any(Option::is_some)
        {
            Some(
                setup
                    .frame
                    .resolve_references(self.geometry_state(), refs.clone())?,
            )
        } else if let Some(id) = refs.model_instance {
            Some(SetupFrame::from_model_instance(self.geometry_state(), id)?)
        } else {
            None
        };
        if let Some(linked) = linked {
            if linked
                .world_from_setup
                .rotation
                .iter()
                .flatten()
                .zip(setup.frame.world_from_setup.rotation.iter().flatten())
                .any(|(a, b)| (a - b).abs() > 1e-6)
            {
                return Err("WCS references changed; reselect the setup coordinate system".into());
            }
        }
        Ok(())
    }

    pub fn setup_geometry_state(&self) -> Result<cam_model::GeometryState, String> {
        if self.setups().is_empty() {
            return Err(
                "Create a setup and select its models before creating an operation.".into(),
            );
        }
        self.validate_setup_frame(self.setup())?;
        crate::cad_operations::geometry_for_setup(self.document())
    }

    pub fn pending_setup_stock_bounds(&self, setup: &Setup) -> ([f64; 3], [f64; 3]) {
        self.document().stock_bounds_for(setup)
    }

    pub fn setup_model_bounds(&self) -> ([f64; 3], [f64; 3]) {
        self.pending_setup_model_bounds(self.setup())
    }

    pub fn pending_setup_model_bounds(&self, setup: &Setup) -> ([f64; 3], [f64; 3]) {
        crate::cad_operations::model_bounds(&self.document().geometry, setup)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn drill() -> Operation {
        Operation::Drill(DrillOp {
            cycle: Default::default(),
            tip_through_bottom: false,
            geometry: Default::default(),
            hole_heights: Vec::new(),
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
        })
    }

    #[test]
    fn operation_safety_planes_follow_copies_moves_deletion_and_undo() {
        let mut app = AppController::new(crate::default_machine());
        app.add_operation(drill());
        let first = app.active_setup_id();
        let id = app.setup().operations[0].id();
        let heights = Heights::new(25., 10., 3.);
        app.apply_operation_settings_with_heights(
            app.operation(id).unwrap().clone(),
            None,
            vec![],
            None,
            Some(heights),
        )
        .unwrap();
        app.select(Selection::Operation(id));
        app.duplicate_selected_operation();
        let operation_copy = app.selected_operation().unwrap().id();
        assert_eq!(app.setup().heights_for(operation_copy), heights);
        assert!(!app.setup().height_settings.contains_key(&operation_copy));
        assert!(app.undo());

        let duplicated = app.duplicate_setup(first).unwrap();
        let copy_id = app.setup().operations[0].id();
        assert_ne!(copy_id, id);
        assert_eq!(app.setup().heights_for(copy_id), heights);
        assert!(!app.setup().operation_heights.contains_key(&id));
        let second = app.add_setup();
        assert!(app.setup().operation_heights.is_empty());
        let copied = app
            .transfer_operation_to_setup(copy_id, second, true)
            .unwrap();
        assert_eq!(app.setup().heights_for(copied), heights);
        app.transfer_operation_to_setup(id, second, false).unwrap();
        assert_eq!(app.setup().heights_for(id), heights);
        assert!(!app
            .setups()
            .iter()
            .find(|s| s.id == first)
            .unwrap()
            .operation_heights
            .contains_key(&id));
        app.delete_selected_operation();
        assert!(!app.setup().operation_heights.contains_key(&id));
        app.undo();
        assert_eq!(app.setup().heights_for(id), heights);
        app.activate_setup(duplicated).unwrap();
        assert_eq!(app.setup().heights_for(copy_id), heights);
    }

    #[test]
    fn tree_order_uses_stable_ids_and_survives_undo_redo_and_reopen() {
        let mut app = AppController::new(crate::default_machine());
        let first = app.add_setup();
        for _ in 0..3 {
            app.add_operation(drill());
        }
        let ids: Vec<_> = app.setup().operations.iter().map(Operation::id).collect();
        let original = app.document().clone();
        app.reorder_operation_to(ids[2], first, Some(ids[0]))
            .unwrap();
        let order: Vec<_> = app.setup().operations.iter().map(Operation::id).collect();
        assert_eq!(order, [ids[2], ids[0], ids[1]]);
        let planned = cam_toolpath::build_job(
            app.document(),
            1000.,
            cam_cldata::SpindleDir::Cw,
            None,
            10.,
            &CancelToken::new(),
        );
        assert_eq!(planned.completion, JobCompletion::Complete);
        let actual: Vec<_> = planned
            .program
            .steps
            .iter()
            .filter_map(|step| match step {
                cam_cldata::Step::Drill(cycle) => Some(cycle.tag.op_id),
                _ => None,
            })
            .collect();
        assert_eq!(actual, order);
        assert!(app.undo());
        assert_eq!(app.document(), &original);
        assert!(app.redo());
        let second = app.add_setup();
        app.reorder_setup_to(second, Some(first)).unwrap();
        assert_eq!(
            app.setups().iter().map(|s| s.id).collect::<Vec<_>>(),
            [second, first]
        );
        assert!(app.undo());
        assert_eq!(app.setups()[0].id, first);
        assert!(app.redo());
        let path =
            std::env::temp_dir().join(format!("opencam-tree-order-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = AppController::new(crate::default_machine());
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert_eq!(reopened.setups()[0].id, second);
        assert_eq!(
            reopened.setups()[1]
                .operations
                .iter()
                .map(Operation::id)
                .collect::<Vec<_>>(),
            order
        );
    }

    #[test]
    fn drop_transfer_inserts_and_remaps_tools_in_one_undo_step() {
        let mut app = AppController::new(crate::default_machine());
        app.add_operation(drill());
        let source = app.active_setup_id();
        let moving = app.setup().operations[0].id();
        let destination = app.add_setup();
        app.edit_tool(0, |tool| tool.diameter = 2.);
        app.add_operation(drill());
        let anchor = app.setup().operations[0].id();
        app.activate_setup(source).unwrap();
        let before = app.document().clone();
        app.reorder_operation_to(moving, destination, Some(anchor))
            .unwrap();
        assert_eq!(
            app.setup()
                .operations
                .iter()
                .map(Operation::id)
                .collect::<Vec<_>>(),
            [moving, anchor]
        );
        assert_ne!(
            app.setup().operations[0].tool(),
            app.setup().operations[1].tool()
        );
        assert!(app
            .setups()
            .iter()
            .find(|s| s.id == source)
            .unwrap()
            .operations
            .is_empty());
        assert!(app.undo());
        assert_eq!(app.document(), &before);
        assert!(app.redo());
        assert_eq!(app.setup().operations[0].id(), moving);
    }

    #[test]
    fn invalid_and_unchanged_drops_preserve_document_and_history() {
        let mut app = AppController::new(crate::default_machine());
        app.add_operation(drill());
        let source = app.active_setup_id();
        let operation = app.setup().operations[0].id();
        let destination = app.add_setup();
        app.edit_setup(|s| {
            s.frame = SetupFrame::from_axes(Some([1., 0., 0.]), Some([0., -1., 0.]), None).unwrap()
        });
        let before = app.document().clone();
        let checkpoint_count = app.document.undo_len();
        assert!(app
            .reorder_operation_to(operation, destination, Some(operation))
            .is_err());
        assert!(app
            .reorder_operation_to(operation, destination, None)
            .is_err());
        assert!(app
            .reorder_operation_to(u32::MAX, destination, None)
            .is_err());
        assert!(app.reorder_setup_to(source, Some(u64::MAX)).is_err());
        app.reorder_setup_to(source, Some(source)).unwrap();
        app.reorder_operation_to(operation, source, None).unwrap();
        assert_eq!(app.document(), &before);
        assert_eq!(app.document.undo_len(), checkpoint_count);
    }

    #[test]
    fn cross_origin_drop_is_rejected_instead_of_claiming_an_ignored_order() {
        let mut app = AppController::new(crate::default_machine());
        app.add_operation(drill());
        app.add_operation(drill());
        let setup = app.active_setup_id();
        let moving = app.setup().operations[0].id();
        let anchor = app.setup().operations[1].id();
        app.add_origin();
        let origin = app.setup().extra_origins[0].index;
        app.set_operation_origin(anchor, origin);
        let before = app.document().clone();
        let undo_count = app.document.undo_len();
        let reason = app
            .reorder_operation_to(moving, setup, Some(anchor))
            .unwrap_err();
        assert!(reason.contains("Move to Origin"));
        assert_eq!(app.document(), &before);
        assert_eq!(app.document.undo_len(), undo_count);
    }

    #[test]
    fn operation_copy_remaps_conflicting_tools_and_undoes_as_one_command() {
        let mut app = AppController::new(crate::default_machine());
        app.add_operation(drill());
        let first = app.active_setup_id();
        let id = app.setup().operations[0].id();
        let second = app.add_setup();
        app.edit_tool(0, |t| t.diameter = 2.);
        app.activate_setup(first).unwrap();
        let copied = app.transfer_operation_to_setup(id, second, true).unwrap();
        assert_ne!(copied, id);
        let tool = app
            .setup()
            .tools
            .iter()
            .find(|t| t.number == app.operation(copied).unwrap().tool())
            .unwrap();
        assert_eq!(tool.diameter, 6.);
        assert_ne!(tool.number, 1);
        assert!(app
            .setups()
            .iter()
            .find(|s| s.id == first)
            .unwrap()
            .operations
            .iter()
            .any(|op| op.id() == id));
        assert!(app.undo());
        assert!(app.setup().operations.is_empty());
        assert_eq!(app.setup().tools.len(), 1);
        assert!(app.redo());
        assert_eq!(app.setup().operations[0].id(), copied);
    }

    #[test]
    fn failed_transfer_and_invalid_setup_drafts_never_modify_committed_state() {
        let mut app = AppController::new(crate::default_machine());
        app.add_operation(drill());
        let first = app.active_setup_id();
        let second = app.add_setup();
        app.edit_setup(|s| {
            s.frame = SetupFrame::from_axes(Some([1., 0., 0.]), Some([0., -1., 0.]), None).unwrap()
        });
        app.activate_setup(first).unwrap();
        let before = app.document().clone();
        assert!(app.transfer_operation_to_setup(0, second, false).is_err());
        assert_eq!(app.document(), &before);
        let mut draft = app.setup().clone();
        draft.stock = Stock::FixedBox {
            size: [10., 10., 10.],
            model_position: [2.; 3],
        };
        assert!(app.apply_setup_settings(draft).is_err());
        assert_eq!(app.document(), &before);
    }

    #[test]
    fn work_offset_edits_update_operations_and_are_one_undoable_transaction() {
        let mut app = AppController::new(crate::default_machine());
        app.edit_stock(|s| {
            *s = Stock::FixedBox {
                size: [20.; 3],
                model_position: [0.5; 3],
            }
        });
        app.add_operation(drill());
        let mut draft = app.setup().clone();
        draft.name = "Top".into();
        draft.material = "Wood".into();
        draft.origin_index = 2;
        app.apply_setup_settings(draft).unwrap();
        assert_eq!(app.setup().operations[0].work_offset(), 2);
        assert_eq!(app.setup().material, "Wood");
        assert!(app.undo());
        assert_eq!(app.setup().origin_index, 1);
        assert_eq!(app.setup().operations[0].work_offset(), 1);
        assert!(app.setup().material.is_empty());
    }
}
