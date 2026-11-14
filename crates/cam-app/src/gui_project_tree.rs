//! Overlay browser, context commands and stable-ID drag/drop routing.
use super::*;
use iced::widget::column;

const TREE_ROW_H: f32 = 30.0;
const TREE_TEXT_SIZE: u32 = 12;

pub(super) use crate::project::TreeNode;

#[derive(Clone, Debug)]
pub(super) struct TreeDrag {
    node: TreeNode,
    start: iced::Point,
    dragging: bool,
    edit_on_release: bool,
}

#[derive(Clone, Debug)]
pub(super) enum TreeMessage {
    Press(TreeNode),
    RenameStart(u32, String),
    RenameChanged(String),
    RenameCommit,
    View(TreeNode),
    ToggleModels,
    ToggleSetups,
    Hover(TreeNode, bool),
    Leave(TreeNode),
    Open(TreeNode),
    DoubleClick(TreeNode),
    Release,
    Command(Box<Message>),
    ShowPatternMenu,
    Close,
}

impl App {
    /// The radio selection survives temporary operation and geometry selections.
    pub(super) fn restore_browser_view(&mut self) {
        match self.browser_view {
            TreeNode::Setup(id) if self.controller.setups().iter().any(|s| s.id == id) => {
                let _ = self.controller.activate_setup(id);
                self.model_view = None;
            }
            TreeNode::Asset(id)
                if self
                    .controller
                    .geometry_state()
                    .assets
                    .iter()
                    .any(|a| a.id == id) =>
            {
                self.model_view = Some(TreeNode::Asset(id))
            }
            TreeNode::Model(id)
                if self
                    .controller
                    .geometry_state()
                    .instances
                    .iter()
                    .any(|i| i.id == id) =>
            {
                self.model_view = Some(TreeNode::Model(id))
            }
            _ => {
                self.browser_view = TreeNode::Models;
                self.model_view = Some(TreeNode::Models);
            }
        }
    }

    fn browser_radio(&self, node: TreeNode) -> Element<'_, Message> {
        tooltip(
            iced::widget::radio("", node, Some(self.browser_view), |node| {
                Message::Tree(TreeMessage::View(node))
            })
            .size(14)
            .spacing(0),
            text("Use this view when no operation is selected").size(12),
            tooltip::Position::Right,
        )
        .into()
    }

    fn browser_visibility(&self, visible: bool, message: Message) -> Element<'_, Message> {
        let slash = if visible {
            ""
        } else {
            "<path d='M3 3l18 18'/>"
        };
        let image = format!("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='none' stroke='#c6cbd2' stroke-width='1.6' stroke-linecap='round' stroke-linejoin='round'><path d='M2 12s3.5-7 10-7 10 7 10 7-3.5 7-10 7S2 12 2 12z'/><circle cx='12' cy='12' r='3'/>{slash}</svg>");
        tooltip(
            button(
                iced::widget::svg(iced::widget::svg::Handle::from_memory(image.into_bytes()))
                    .width(16)
                    .height(16),
            )
            .width(22)
            .height(22)
            .padding(3)
            .style(button::text)
            .on_press(message),
            text(if visible {
                "Hide in viewport"
            } else {
                "Show in viewport"
            })
            .size(12),
            tooltip::Position::Right,
        )
        .into()
    }

    fn browser_disclosure(&self, expanded: bool, message: Message) -> Element<'_, Message> {
        button(
            container(text(if expanded { "▾" } else { "▸" }).size(13))
                .width(18)
                .height(20)
                .center_x(18)
                .center_y(20),
        )
        .padding(0)
        .style(button::text)
        .on_press(message)
        .into()
    }

    pub(super) fn project_title(&self) -> String {
        self.controller
            .current_path()
            .and_then(|path| path.file_stem())
            .and_then(|name| name.to_str())
            .filter(|name| !name.is_empty())
            .unwrap_or("Untitled")
            .to_owned()
    }

    fn tree_body_selected(&self, id: InstanceId) -> bool {
        self.model_view == Some(TreeNode::Model(id))
            || (self.setup_draft.is_some()
                && (self.setup_model_selection == Some(id)
                    || self.setup_fixture_selection == Some(id)))
            || self
                .controller
                .cad_selection()
                .iter()
                .any(|r| r.instance_id == id)
    }

    pub(super) fn reveal_selected_bodies(&mut self) {
        for entity in self.controller.cad_selection() {
            self.collapsed_assets.remove(&entity.asset_id);
        }
    }

    pub(super) fn selected_setup_models(&self) -> Vec<InstanceId> {
        // Geometry selection takes precedence over the collection used to display it.
        let selected: BTreeSet<_> = self
            .controller
            .cad_selection()
            .iter()
            .map(|r| r.instance_id)
            .collect();
        if !selected.is_empty() {
            return selected.into_iter().collect();
        }
        self.controller
            .geometry_state()
            .instances
            .iter()
            .filter(|instance| match self.model_view {
                Some(TreeNode::Model(id)) => instance.id == id,
                Some(TreeNode::Asset(id)) => instance.asset_id == id,
                _ => false,
            })
            .map(|instance| instance.id)
            .collect()
    }

    pub(super) fn clear_browser_selection(&mut self) {
        self.pending_simulation = None;
        self.pending_processing_time = None;
        self.cad_filter_explicit = false;
        self.geometry_additive = false;
        self.geometry_auto_pick = false;
        self.end_setup_draft();
        self.operation_draft = None;
        self.stock_to_leave_draft = None;
        self.operation_tools.clear();
        self.operation_geometry = None;
        self.operation_binding_preview = None;
        self.controller.select(Selection::Setup);
        self.controller.set_cad_selection(vec![]);
        self.focus_ops.clear();
        self.restore_browser_view();
        self.refresh_fields();
    }

    pub(super) fn cancel_tree_drag(&mut self) {
        self.tree_drag = None;
        self.tree_menu = None;
        self.pattern_menu_y = None;
        self.tree_hover = None;
        self.open_op_menu = None;
    }

    pub(super) fn tree_pointer_moved(&mut self, point: iced::Point) {
        if let Some(drag) = self.tree_drag.as_mut() {
            if point.distance(drag.start) >= 5.0 {
                drag.dragging = true;
            }
        }
    }

    fn tree_edit_blocked(&mut self) -> bool {
        if self.move_draft.is_some()
            || self.controller.pending_op().is_some()
            || self.pattern_draft.is_some()
            || self.cad_pending_kind.is_some()
            || self.cad_editing_op.is_some()
            || self.drawing_editing_op.is_some()
            || self.setup_draft_dirty()
            || (self.inspector_active
                && matches!(self.controller.selection(), Selection::Operation(_))
                && self.inspector_dirty())
        {
            self.status = "Apply or cancel inspector changes before changing selection or restructuring the project.".into();
            true
        } else {
            false
        }
    }

    fn select_tree_node(&mut self, node: &TreeNode) -> iced::Task<Message> {
        match node {
            TreeNode::Setup(id) => {
                if self.model_view.is_none()
                    && self.controller.selection() == Selection::Setup
                    && self.controller.active_setup_id() == *id
                    && self.setup_draft.is_none()
                {
                    self.clear_browser_selection();
                    iced::Task::none()
                } else {
                    self.update(Message::SelectSetup(*id))
                }
            }
            TreeNode::Operation(id) => {
                if let Some(owner) = self
                    .controller
                    .setups()
                    .iter()
                    .find(|s| s.operations.iter().any(|op| op.id() == *id))
                    .map(|s| s.id)
                {
                    if owner != self.controller.active_setup_id() {
                        self.controller.activate_setup(owner).ok();
                        self.end_setup_draft();
                        self.expanded_setups.insert(owner);
                    }
                }
                self.update(Message::ClickOp(*id))
            }
            TreeNode::Pattern(id) => {
                if let Some((setup_id, pattern)) = self.controller.setups().iter().find_map(|s| {
                    s.patterns
                        .iter()
                        .find(|p| p.id == *id)
                        .map(|p| (s.id, p.clone()))
                }) {
                    if setup_id != self.controller.active_setup_id() {
                        self.controller.activate_setup(setup_id).ok();
                    }
                    self.focus_ops = pattern.members.iter().copied().collect();
                    self.controller.select(Selection::Setup);
                    self.model_view = None;
                    self.expanded_patterns.insert(*id);
                    self.pending_simulation = None;
                    self.pending_processing_time = None;
                }
                iced::Task::none()
            }
            _ => {
                self.pending_simulation = None;
                self.pending_processing_time = None;
                self.end_setup_draft();
                self.operation_draft = None;
                self.stock_to_leave_draft = None;
                self.controller.select(Selection::Setup);
                self.controller.set_cad_selection(vec![]);
                self.cad_hover = None;
                self.cad_candidates.clear();
                self.cad_inspecting = false;
                self.focus_ops.clear();
                self.model_view = Some(*node);
                iced::Task::none()
            }
        }
    }

    pub(super) fn update_tree(&mut self, message: TreeMessage) -> iced::Task<Message> {
        match message {
            TreeMessage::RenameStart(id, name) => {
                self.operation_rename = Some((id, name));
                return iced::widget::operation::focus("operation-rename");
            }
            TreeMessage::RenameChanged(name) => {
                if let Some((_, value)) = &mut self.operation_rename {
                    *value = name;
                }
            }
            TreeMessage::RenameCommit => {
                if let Some((id, name)) = self.operation_rename.take() {
                    self.controller.rename_operation(id, name);
                }
            }
            TreeMessage::ToggleModels => self.models_collapsed = !self.models_collapsed,
            TreeMessage::ToggleSetups => self.setups_collapsed = !self.setups_collapsed,
            TreeMessage::View(node) => {
                if self.tree_edit_blocked() {
                    return iced::Task::none();
                }
                self.browser_view = node;
                self.clear_browser_selection();
            }
            TreeMessage::Hover(node, after) => {
                // Row movement is authoritative too: global cursor updates may
                // arrive after widget messages or be coalesced during a drag.
                if let Some(drag) = self.tree_drag.as_mut() {
                    if drag.node != node {
                        drag.dragging = true;
                    }
                }
                self.tree_hover = Some((node, after));
            }
            TreeMessage::Leave(node) => {
                if self
                    .tree_hover
                    .as_ref()
                    .is_some_and(|(hover, _)| hover == &node)
                {
                    self.tree_hover = None;
                }
            }
            TreeMessage::Press(node) => {
                if let Some((id, name)) = self.operation_rename.take() {
                    self.controller.rename_operation(id, name);
                }
                self.tree_menu = None;
                self.pattern_menu_y = None;
                self.open_op_menu = None;
                if self.tree_edit_blocked() {
                    return iced::Task::none();
                }
                self.tree_drag = Some(TreeDrag {
                    node,
                    start: self.window_cursor,
                    dragging: false,
                    edit_on_release: false,
                });
            }
            TreeMessage::Open(node) => {
                self.tree_drag = None;
                if self.tree_edit_blocked() {
                    return iced::Task::none();
                }
                match &node {
                    TreeNode::Operation(id) => {
                        let owner = self
                            .controller
                            .setups()
                            .iter()
                            .find(|s| s.operations.iter().any(|op| op.id() == *id))
                            .map(|s| s.id);
                        if let Some(owner) = owner {
                            self.controller.activate_setup(owner).ok();
                            self.end_setup_draft();
                            self.expanded_setups.insert(owner);
                        }
                        let _ = self.update(Message::OpMenu(*id));
                    }
                    TreeNode::Setup(id) => {
                        if self.controller.active_setup_id() != *id {
                            let _ = self.update(Message::SelectSetup(*id));
                        }
                    }
                    TreeNode::Asset(_) | TreeNode::Model(_) | TreeNode::Pattern(_) => {
                        let _ = self.select_tree_node(&node);
                    }
                    _ => {}
                }
                self.pattern_menu_y = None;
                self.op_menu_pos = self.window_cursor;
                self.tree_menu = Some(node);
            }
            TreeMessage::DoubleClick(node) => {
                if let Some(drag) = self.tree_drag.as_mut() {
                    if drag.node == node {
                        drag.edit_on_release = true;
                        return iced::Task::none();
                    }
                }
                self.tree_drag = None;
                self.tree_menu = None;
                self.pattern_menu_y = None;
                self.open_op_menu = None;
                if self.tree_edit_blocked() {
                    return iced::Task::none();
                }
                match node {
                    TreeNode::Setup(id) => return self.update(Message::EditSetup(id)),
                    TreeNode::Pattern(id) => return self.update(Message::EditPattern(id)),
                    TreeNode::Operation(id) => {
                        let Some(owner) = self
                            .controller
                            .setups()
                            .iter()
                            .find(|setup| setup.operations.iter().any(|op| op.id() == id))
                            .map(|setup| setup.id)
                        else {
                            return iced::Task::none();
                        };
                        let _ = self.controller.activate_setup(owner);
                        self.expanded_setups.insert(owner);
                        self.inspector_enabled = true;
                        return self.update(Message::Select(Selection::Operation(id)));
                    }
                    _ => return iced::Task::none(),
                }
            }
            TreeMessage::ShowPatternMenu => {
                if matches!(
                    self.tree_menu,
                    Some(TreeNode::Setup(_) | TreeNode::Operation(_))
                ) {
                    self.pattern_menu_y
                        .get_or_insert(self.window_cursor.y - 14.0);
                }
            }
            TreeMessage::Command(command) => {
                self.tree_menu = None;
                self.pattern_menu_y = None;
                return self.update(*command);
            }
            TreeMessage::Close => {
                self.tree_menu = None;
                self.pattern_menu_y = None;
                self.open_op_menu = None;
                return self.update(Message::CloseOpMenu);
            }
            TreeMessage::Release => {
                let Some(drag) = self.tree_drag.take() else {
                    return iced::Task::none();
                };
                if self.tree_edit_blocked() {
                    return iced::Task::none();
                }
                if !drag.dragging {
                    return if drag.edit_on_release {
                        self.update_tree(TreeMessage::DoubleClick(drag.node))
                    } else {
                        self.select_tree_node(&drag.node)
                    };
                }
                let Some((target, after)) = self.tree_hover.take() else {
                    self.status = "Drop cancelled: release over a setup or operation row.".into();
                    return iced::Task::none();
                };
                let revision = self.controller.revision();
                let result = match (&drag.node, target) {
                    (TreeNode::Setup(id), TreeNode::Setup(target)) => {
                        let before = if after {
                            self.controller
                                .setups()
                                .iter()
                                .skip_while(|s| s.id != target)
                                .nth(1)
                                .map(|s| s.id)
                        } else {
                            Some(target)
                        };
                        self.controller.reorder_setup_to(*id, before)
                    }
                    (TreeNode::Operation(id), TreeNode::Setup(destination)) => self
                        .controller
                        .move_operation_in_tree(*id, destination, None, None),
                    (TreeNode::Operation(id), TreeNode::Pattern(pattern)) => {
                        if let Some(destination) = self
                            .controller
                            .setups()
                            .iter()
                            .find(|s| s.patterns.iter().any(|p| p.id == pattern))
                            .map(|s| s.id)
                        {
                            self.controller.move_operation_in_tree(
                                *id,
                                destination,
                                None,
                                Some(pattern),
                            )
                        } else {
                            Err("Pattern no longer exists".into())
                        }
                    }
                    (TreeNode::Operation(id), TreeNode::Operation(anchor)) => {
                        let destination = self
                            .controller
                            .setups()
                            .iter()
                            .find(|s| s.operations.iter().any(|op| op.id() == anchor));
                        if let Some(destination) = destination {
                            let anchor_offset = destination
                                .operations
                                .iter()
                                .find(|op| op.id() == anchor)
                                .unwrap()
                                .work_offset();
                            let crosses_setup =
                                !destination.operations.iter().any(|op| op.id() == *id);
                            if crosses_setup && anchor_offset != destination.origin_index {
                                self.status = "Drop rejected: transfer onto the setup first, then use Move to Origin for another work-origin group.".into();
                                return iced::Task::none();
                            }
                            let before = if after {
                                destination
                                    .operations
                                    .iter()
                                    .skip_while(|op| op.id() != anchor)
                                    .skip(1)
                                    .find(|op| op.work_offset() == anchor_offset)
                                    .map(Operation::id)
                            } else {
                                Some(anchor)
                            };
                            let pattern = destination
                                .patterns
                                .iter()
                                .find(|p| p.members.contains(&anchor))
                                .map(|p| p.id);
                            self.controller.move_operation_in_tree(
                                *id,
                                destination.id,
                                before,
                                pattern,
                            )
                        } else {
                            Err("Drop target no longer exists".into())
                        }
                    }
                    _ => Err(
                        "Drop an operation on a setup or operation; drop a setup on another setup."
                            .into(),
                    ),
                };
                match result {
                    Ok(()) => {
                        if self.controller.revision() == revision {
                            self.status = "Execution order unchanged.".into();
                            return iced::Task::none();
                        }
                        self.end_setup_draft();
                        self.operation_draft = None;
                        self.refresh_fields();
                        self.rerun();
                        self.status =
                            "Execution order updated. Undo restores the previous order.".into();
                    }
                    Err(error) => self.status = format!("Drop rejected: {error}"),
                }
            }
        }
        iced::Task::none()
    }

    fn tree_target_active(&self, node: &TreeNode) -> bool {
        self.tree_drag.as_ref().is_some_and(|drag| drag.dragging)
            && self
                .tree_hover
                .as_ref()
                .is_some_and(|(hover, _)| hover == node)
    }

    fn browser_row_surface<'a>(
        &self,
        content: Element<'a, Message>,
        selected: bool,
        hovered: bool,
        target: bool,
    ) -> Element<'a, Message> {
        container(content)
            .width(Length::Shrink)
            .height(Length::Fixed(TREE_ROW_H))
            .padding([2, 4])
            .style(move |_| container::Style {
                background: Some(Background::Color(if selected {
                    palette::SELECT_BG
                } else if hovered {
                    palette::ROW_HOVER
                } else {
                    Color::from_rgba(0.20, 0.23, 0.27, 0.88)
                })),
                border: Border {
                    color: if target {
                        palette::ACCENT_BLUE
                    } else {
                        Color::TRANSPARENT
                    },
                    width: if target { 1.0 } else { 0.0 },
                    ..Border::default()
                },
                ..container::Style::default()
            })
            .into()
    }

    fn tree_row<'a>(
        &self,
        node: TreeNode,
        content: Element<'a, Message>,
        selected: bool,
    ) -> Element<'a, Message> {
        let target = self.tree_target_active(&node);
        let hovered = self
            .tree_hover
            .as_ref()
            .is_some_and(|(hover, _)| hover == &node);
        let depth = match node {
            TreeNode::Models | TreeNode::Setups => 0,
            TreeNode::Asset(_) | TreeNode::Setup(_) => 1,
            TreeNode::Model(_) | TreeNode::Pattern(_) => 2,
            TreeNode::Operation(id) => {
                if self
                    .controller
                    .setups()
                    .iter()
                    .any(|s| s.patterns.iter().any(|p| p.members.contains(&id)))
                {
                    3
                } else {
                    2
                }
            }
        };
        // Paint only the contents, but accept vertical drags across the full
        // browser width: shorter labels must remain reachable drop targets.
        container(
            mouse_area(
                container(self.browser_row_surface(content, selected, hovered, target))
                    .width(Length::Fill),
            )
            .on_press(Message::Tree(TreeMessage::Press(node)))
            .on_release(Message::Tree(TreeMessage::Release))
            .on_double_click(Message::Tree(TreeMessage::DoubleClick(node)))
            .on_right_press(Message::Tree(TreeMessage::Open(node)))
            .on_move(move |point| {
                Message::Tree(TreeMessage::Hover(node, point.y >= TREE_ROW_H / 2.0))
            })
            .on_exit(Message::Tree(TreeMessage::Leave(node))),
        )
        .padding(Padding {
            left: depth as f32 * 12.,
            ..Padding::ZERO
        })
        .into()
    }
    /// Bound the input shield to the visible browser, leaving the viewport below usable.
    pub(super) fn project_browser_height(&self) -> f32 {
        let mut height = 3. * (TREE_ROW_H + 2.) + 46.; // groups and project title
        if !self.models_collapsed {
            let geometry = self.controller.geometry_state();
            for asset in &geometry.assets {
                let count = geometry
                    .instances
                    .iter()
                    .filter(|i| i.asset_id == asset.id)
                    .count();
                if count == 0 {
                    continue;
                }
                height += TREE_ROW_H + 2.;
                if !self.collapsed_assets.contains(&asset.id) {
                    height += count as f32 * (TREE_ROW_H + 2.);
                }
            }
        }
        if self.tools_expanded {
            height += self.controller.used_tools().len() as f32 * (TREE_ROW_H + 2.);
        }
        if !self.setups_collapsed {
            for setup in self.controller.setups() {
                height += TREE_ROW_H + 2.;
                if !self.expanded_setups.contains(&setup.id) {
                    continue;
                }
                height += setup.patterns.len() as f32 * (TREE_ROW_H + 2.);
                for op in &setup.operations {
                    if setup.patterns.iter().any(|p| {
                        p.members.contains(&op.id()) && !self.expanded_patterns.contains(&p.id)
                    }) {
                        continue;
                    }
                    height += TREE_ROW_H + 2.;
                    if self.expanded_operations.contains(&op.id()) {
                        height += op.tools().len() as f32 * 24.;
                    }
                }
                if setup.origin_indices().len() > 1 {
                    height += setup.origin_indices().len() as f32 * 24.;
                }
            }
        }
        height.min(560.)
    }

    pub(super) fn project_tree(&self) -> Element<'_, Message> {
        // Workpiece origins live as group headers in the operation list below (H1
        // above the first op, added origins each heading their own group), not as a
        // node here — an origin *is* the datum a group of operations runs under.
        let mut list = column![self.tree_row(
            TreeNode::Models,
            row![
                self.browser_disclosure(
                    !self.models_collapsed,
                    Message::Tree(TreeMessage::ToggleModels)
                ),
                icon_svg(Icon::ShowCube, 16.),
                text("Models").size(TREE_TEXT_SIZE).width(Length::Shrink),
                self.browser_radio(TreeNode::Models)
            ]
            .spacing(5)
            .align_y(Alignment::Center)
            .into(),
            self.model_view == Some(TreeNode::Models) && self.controller.cad_selection().is_empty(),
        )]
        .spacing(2);

        let geometry = self.controller.geometry_state();
        for asset in geometry.assets.iter().filter(|_| !self.models_collapsed) {
            let instances: Vec<_> = geometry
                .instances
                .iter()
                .filter(|instance| instance.asset_id == asset.id)
                .collect();
            if instances.is_empty() {
                continue;
            }
            let expanded = !self.collapsed_assets.contains(&asset.id);
            let asset_row = row![
                self.browser_disclosure(expanded, Message::ToggleAssetExpanded(asset.id)),
                icon_svg(Icon::ShowCube, 16.),
                self.browser_visibility(
                    !self.hidden_assets.contains(&asset.id),
                    Message::SetAssetVisible(asset.id, self.hidden_assets.contains(&asset.id))
                ),
                text(&asset.name).size(TREE_TEXT_SIZE).width(Length::Shrink),
                self.browser_radio(TreeNode::Asset(asset.id)),
            ]
            .spacing(4)
            .align_y(Alignment::Center);
            list = list.push(self.tree_row(
                TreeNode::Asset(asset.id),
                asset_row.into(),
                self.model_view == Some(TreeNode::Asset(asset.id)),
            ));
            if !expanded {
                continue;
            }
            for instance in instances {
                let hidden = !instance.visible || self.cad_hidden.contains(&instance.id);
                let model_row = row![
                    Space::new().width(18),
                    icon_svg(Icon::ShowCube, 16.),
                    self.browser_visibility(
                        !hidden,
                        Message::SetInstanceVisible(instance.id, hidden)
                    ),
                    text(&instance.name)
                        .size(TREE_TEXT_SIZE)
                        .width(Length::Shrink),
                    self.browser_radio(TreeNode::Model(instance.id)),
                ]
                .spacing(4)
                .align_y(Alignment::Center);
                list = list.push(self.tree_row(
                    TreeNode::Model(instance.id),
                    model_row.into(),
                    self.tree_body_selected(instance.id),
                ));
            }
        }

        list = list.push(
            self.tree_row(
                TreeNode::Setups,
                row![
                    self.browser_disclosure(
                        !self.setups_collapsed,
                        Message::Tree(TreeMessage::ToggleSetups)
                    ),
                    icon_svg(Icon::Open, 16.),
                    text("Setups").size(TREE_TEXT_SIZE)
                ]
                .spacing(5)
                .align_y(Alignment::Center)
                .into(),
                false,
            ),
        );
        let status_fingerprints = self.controller.current_toolpath_fingerprints();
        let toolpath_statuses = self.controller.toolpath_statuses_for(&status_fingerprints);
        for setup in self
            .controller
            .setups()
            .iter()
            .filter(|_| !self.setups_collapsed)
        {
            let active = setup.id == self.controller.active_setup_id();
            let expanded = self.expanded_setups.contains(&setup.id);
            let setup_row = row![
                self.browser_disclosure(expanded, Message::ToggleSetupExpanded(setup.id)),
                icon_svg(Icon::Open, 16.),
                self.browser_visibility(
                    !self.hidden_setups.contains(&setup.id),
                    Message::SetSetupVisible(setup.id, self.hidden_setups.contains(&setup.id))
                ),
                self.status_indicator(
                    StatusTarget::Setup(setup.id),
                    &toolpath_statuses,
                    &status_fingerprints,
                ),
                mouse_area(text(&setup.name).size(TREE_TEXT_SIZE).width(Length::Shrink))
                    .on_double_click(Message::Tree(TreeMessage::DoubleClick(TreeNode::Setup(
                        setup.id
                    ))),),
                self.browser_radio(TreeNode::Setup(setup.id)),
            ]
            .spacing(3)
            .align_y(Alignment::Center);
            list = list.push(self.tree_row(
                TreeNode::Setup(setup.id),
                setup_row.into(),
                active && self.model_view.is_none() && self.focus_ops.is_empty(),
            ));
            if !expanded {
                continue;
            }
            // The planner executes contiguous WCS groups in origin order. Display
            // that same stable order so a row's position is its execution order.
            let origins = setup.origin_indices();
            let mut operations: Vec<_> = setup.operations.iter().collect();
            let mut operation_counts = std::collections::BTreeMap::new();
            operations.sort_by_key(|op| {
                origins
                    .iter()
                    .position(|origin| *origin == op.work_offset())
                    .unwrap_or(0)
            });
            let mut group = None;
            for op in operations {
                let kind = op_kind(op);
                let count = operation_counts.entry(kind).or_insert(0usize);
                *count += 1;
                let operation_name = self
                    .controller
                    .document()
                    .operation_names
                    .get(&op.id())
                    .cloned()
                    .unwrap_or_else(|| format!("{kind} {count}"));
                if origins.len() > 1 && group != Some(op.work_offset()) {
                    list = list.push(tree_note(&format!("    Work origin {}", op.work_offset())));
                    group = Some(op.work_offset());
                }
                let id = op.id();
                if let Some(pattern) = setup
                    .patterns
                    .iter()
                    .find(|pattern| pattern.members.first() == Some(&id))
                {
                    let count = pattern.kind.instance_count().unwrap_or(0);
                    let expanded = self.expanded_patterns.contains(&pattern.id);
                    let pattern_row = row![
                        self.browser_disclosure(
                            expanded,
                            Message::TogglePatternExpanded(pattern.id)
                        ),
                        icon_svg(Icon::Pattern, 16.),
                        text(format!(
                            "{} · {count} instances{}",
                            pattern.name,
                            if pattern.suppressed {
                                " · suppressed"
                            } else {
                                ""
                            }
                        ))
                        .size(TREE_TEXT_SIZE),
                    ]
                    .spacing(5)
                    .align_y(Alignment::Center);
                    list = list.push(self.tree_row(
                        TreeNode::Pattern(pattern.id),
                        pattern_row.into(),
                        false,
                    ));
                }
                if let Some(pattern) = setup.patterns.iter().find(|p| p.members.contains(&id)) {
                    if !self.expanded_patterns.contains(&pattern.id) {
                        continue;
                    }
                }
                let expanded = self.expanded_operations.contains(&id);
                let name_control: Element<'_, Message> = if let Some((_, value)) = self
                    .operation_rename
                    .as_ref()
                    .filter(|(editing, _)| *editing == id)
                {
                    iced::widget::text_input::<Message, iced::Theme, iced::Renderer>(
                        "Operation name",
                        value,
                    )
                    .id("operation-rename")
                    .on_input(|value| Message::Tree(TreeMessage::RenameChanged(value)))
                    .on_submit(Message::Tree(TreeMessage::RenameCommit))
                    .size(TREE_TEXT_SIZE)
                    .width(Length::Fixed(140.0))
                    .into()
                } else {
                    let label = text(operation_name.clone())
                        .size(TREE_TEXT_SIZE)
                        .width(Length::Shrink);
                    if self.focus_ops.len() == 1 && self.focus_ops.contains(&id) {
                        mouse_area(label)
                            .on_press(Message::Tree(TreeMessage::RenameStart(id, operation_name)))
                            .into()
                    } else {
                        label.into()
                    }
                };
                let controls = row![
                    self.browser_disclosure(expanded, Message::ToggleOperationExpanded(id)),
                    self.browser_visibility(
                        !self.hidden_ops.contains(&id),
                        Message::SetOperationVisible(id, self.hidden_ops.contains(&id))
                    ),
                    self.status_indicator(
                        StatusTarget::Operation(id),
                        &toolpath_statuses,
                        &status_fingerprints,
                    ),
                    icon_svg(operation_icon(op), 18.0),
                    text(format!("[T{}]", op.tool())).size(TREE_TEXT_SIZE),
                    name_control
                ]
                .spacing(3)
                .align_y(Alignment::Center);
                list = list.push(self.tree_row(
                    TreeNode::Operation(id),
                    controls.into(),
                    self.focus_ops.contains(&id),
                ));
                if expanded {
                    for number in op.tools() {
                        let label = setup
                            .tools
                            .iter()
                            .find(|tool| tool.number == number)
                            .map(|tool| {
                                format!("T{} · ⌀{} {}", number, fmt_num(tool.diameter), tool.kind)
                            })
                            .unwrap_or_else(|| format!("T{number} · missing tool"));
                        list = list.push(container(text(label).size(TREE_TEXT_SIZE)).padding(
                            Padding {
                                left: if setup.patterns.iter().any(|p| p.members.contains(&id)) {
                                    48.
                                } else {
                                    36.
                                },
                                top: 2.,
                                bottom: 2.,
                                right: 4.,
                            },
                        ));
                    }
                }
            }
        }
        // Tools in use. A tool that matches the shop library shows its **shop number**
        // (`T4 …`); one that isn't in the library shows **no number** — the simple,
        // clear signal that it's project-local. Right-clicking an un-numbered tool
        // offers "Add to library" (which registers it and gives it a number).
        list = list.push(
            mouse_area(
                self.browser_row_surface(
                    row![
                        self.browser_disclosure(self.tools_expanded, Message::ToggleToolsExpanded),
                        icon_svg(Icon::NewTool, 16.),
                        text("Tools in use").size(TREE_TEXT_SIZE),
                    ]
                    .spacing(5)
                    .align_y(Alignment::Center)
                    .into(),
                    false,
                    false,
                    false,
                ),
            )
            .on_press(Message::ToggleToolsExpanded),
        );
        if self.tools_expanded {
            let used = self.controller.used_tools();
            for t in used {
                let in_library = self
                    .library
                    .tools
                    .iter()
                    .any(|lt| lt.identity() == t.identity());
                // Numbered ⇒ in the library; un-numbered ⇒ project-local. The number goes
                // at the *end* (`⌀6 End mill (T5)`) so descriptions align left; a
                // project-local tool has no `(Tn)` at all — the presence of the number *is*
                // the distinction (kept deliberately simple).
                let label = if in_library {
                    format!("⌀{} {} (T{})", fmt_num(t.diameter), t.kind, t.number)
                } else {
                    format!("⌀{} {}", fmt_num(t.diameter), t.kind)
                };
                let row = container(
                    self.browser_row_surface(
                        row![
                            Space::new().width(18),
                            icon_svg(Icon::NewTool, 16.),
                            text(label).size(TREE_TEXT_SIZE).color(if in_library {
                                palette::LABEL_COLOR
                            } else {
                                palette::GROUP_LABEL
                            })
                        ]
                        .spacing(5)
                        .align_y(Alignment::Center)
                        .into(),
                        false,
                        false,
                        false,
                    ),
                )
                .padding(Padding {
                    left: 12.,
                    ..Padding::ZERO
                });
                if in_library {
                    list = list.push(row);
                } else {
                    // Only un-numbered (not-in-library) tools carry the right-click menu.
                    list = list.push(mouse_area(row).on_right_press(Message::ToolMenu(t.number)));
                }
            }
        }
        scrollable(list.padding(2))
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    pub(super) fn tree_menu_overlay(&self) -> Option<Element<'_, Message>> {
        let node = self.tree_menu.as_ref()?;
        let item = |label: String, message: Message| {
            let icon = match &message {
                Message::ToolpathCommand(_, ToolpathAction::Recompute) => Some(Icon::Toolpath),
                Message::ToolpathCommand(_, ToolpathAction::Simulate) => Some(Icon::Run),
                Message::ToolpathCommand(_, ToolpathAction::Export) => Some(Icon::Export),
                Message::ToolpathCommand(_, ToolpathAction::ProcessingTime) => {
                    Some(Icon::ToolpathTime)
                }
                Message::DuplicateOp | Message::SetupDuplicate(_) => Some(Icon::Duplicate),
                Message::BeginPattern(true) => Some(Icon::CircularPattern),
                Message::BeginPattern(false) => Some(Icon::Pattern),
                Message::EditPattern(id) => Some(
                    if self
                        .controller
                        .setups()
                        .iter()
                        .flat_map(|setup| &setup.patterns)
                        .any(|pattern| {
                            pattern.id == *id
                                && matches!(pattern.kind, cam_model::PatternKind::Circular { .. })
                        })
                    {
                        Icon::CircularPattern
                    } else {
                        Icon::Pattern
                    },
                ),
                Message::ToggleSetOrigin => Some(Icon::SetOrigin),
                Message::Select(Selection::Operation(_)) | Message::EditSetup(_) => {
                    Some(Icon::Edit)
                }
                Message::SetOpExcluded(_, true) => Some(Icon::Suppress),
                Message::SetOpExcluded(_, false) => Some(Icon::Run),
                Message::DeleteOp | Message::SetupDelete(_) => Some(Icon::DeleteOperation),
                Message::SetupNew => Some(Icon::New),
                _ => None,
            };
            let mut label_row = row![].spacing(8).align_y(Alignment::Center);
            if let Some(icon) = icon {
                label_row = label_row.push(icon_svg(icon, 22.));
            } else {
                label_row = label_row.push(Space::new().width(22).height(22));
            }
            label_row = label_row.push(text(label).size(12));
            button(label_row)
                .width(Length::Fill)
                .padding([3, 7])
                .on_press(Message::Tree(TreeMessage::Command(Box::new(message))))
                .style(|theme, status| popup_item_style(theme, status, false))
        };
        let pattern_item = || {
            let show = Message::Tree(TreeMessage::ShowPatternMenu);
            mouse_area(
                item("Create pattern".into(), Message::BeginPattern(false)).on_press(show.clone()),
            )
            .on_enter(show)
        };
        let mut items = column![].spacing(2);
        if let Some((label, message)) = match node {
            TreeNode::Setup(id) => Some(("Edit setup", Message::EditSetup(*id))),
            TreeNode::Operation(id) => {
                Some(("Edit operation", Message::Select(Selection::Operation(*id))))
            }
            _ => None,
        } {
            items = items.push(item(label.into(), message));
            items = items.push(popup_separator());
            for (label, action) in [
                ("Recompute toolpath", ToolpathAction::Recompute),
                ("Simulate toolpath", ToolpathAction::Simulate),
                ("Post process and export…", ToolpathAction::Export),
                ("Calculate processing time…", ToolpathAction::ProcessingTime),
            ] {
                items = items.push(item(label.into(), Message::ToolpathCommand(*node, action)));
            }
            items = items.push(popup_separator());
        }
        match node {
            TreeNode::Setups => {
                items = items.push(item("New setup".into(), Message::SetupNew));
            }
            TreeNode::Setup(id) => {
                items = items.push(pattern_item());
                items = items.push(popup_separator());
                items = items.push(item("Duplicate setup".into(), Message::SetupDuplicate(*id)));
                if self.controller.setups().len() > 1 {
                    items = items.push(popup_separator());
                    items = items.push(item("Delete setup".into(), Message::SetupDelete(*id)));
                }
            }
            TreeNode::Operation(id) => {
                items = items
                    .push(item("Duplicate".into(), Message::DuplicateOp))
                    .push(item("Reinitialize".into(), Message::ReinitOp))
                    .push(pattern_item())
                    .push(item(
                        if self.controller.is_operation_excluded(*id) {
                            "Include in machining"
                        } else {
                            "Suppress machining"
                        }
                        .into(),
                        Message::SetOpExcluded(*id, !self.controller.is_operation_excluded(*id)),
                    ));
                for origin in self.origin_move_targets_for(*id) {
                    items = items.push(item(
                        format!("Move to {}", self.origin_menu_label(origin)),
                        Message::MoveOpToOrigin(origin),
                    ));
                }
                for setup in self
                    .controller
                    .setups()
                    .iter()
                    .filter(|s| !s.operations.iter().any(|op| op.id() == *id))
                {
                    items = items
                        .push(item(
                            format!("Move to setup {}", setup.name),
                            Message::TransferOpToSetup(setup.id, false),
                        ))
                        .push(item(
                            format!("Copy to setup {}", setup.name),
                            Message::TransferOpToSetup(setup.id, true),
                        ));
                }
                items = items.push(popup_separator());
                items = items.push(item("Delete operation".into(), Message::DeleteOp));
            }
            TreeNode::Pattern(id) => {
                if let Some(pattern) = self
                    .controller
                    .setups()
                    .iter()
                    .find_map(|s| s.patterns.iter().find(|p| p.id == *id))
                {
                    items = items.push(item("Edit pattern…".into(), Message::EditPattern(*id)));
                    items = items.push(item(
                        if pattern.suppressed {
                            "Include pattern"
                        } else {
                            "Suppress pattern"
                        }
                        .into(),
                        Message::PatternSuppression(*id, !pattern.suppressed),
                    ));
                    items = items.push(item(
                        "Dissolve pattern".into(),
                        Message::PatternDissolve(*id),
                    ));
                    items = items.push(item(
                        format!(
                            "Delete pattern and {} source operation(s)",
                            pattern.members.len()
                        ),
                        Message::PatternDelete(*id),
                    ));
                    items = items.push(popup_separator());
                    for (label, action) in [
                        ("Recompute toolpath", ToolpathAction::Recompute),
                        ("Simulate toolpath", ToolpathAction::Simulate),
                        ("Post process and export…", ToolpathAction::Export),
                        ("Calculate processing time…", ToolpathAction::ProcessingTime),
                    ] {
                        items =
                            items.push(item(label.into(), Message::ToolpathCommand(*node, action)));
                    }
                }
            }
            TreeNode::Models => {
                items = items
                    .push(item("Import geometry…".into(), Message::ImportCad))
                    .push(item("Show all geometry".into(), Message::CadShowAll));
            }
            TreeNode::Asset(id) => {
                let hidden = self.hidden_assets.contains(id);
                items = items
                    .push(item(
                        if hidden { "Show model" } else { "Hide model" }.into(),
                        Message::SetAssetVisible(*id, hidden),
                    ))
                    .push(item(
                        "Move model…".into(),
                        Message::MoveOpen(TreeNode::Asset(*id)),
                    ))
                    .push(item("Replace model…".into(), Message::ReplaceCadAsset(*id)))
                    .push(item("Remove model".into(), Message::RemoveCadAsset(*id)));
            }
            TreeNode::Model(id) => {
                let hidden = self.cad_hidden.contains(id)
                    || self
                        .controller
                        .geometry_state()
                        .instance(*id)
                        .is_some_and(|instance| !instance.visible);
                items = items
                    .push(item(
                        if hidden { "Show body" } else { "Hide body" }.into(),
                        Message::SetInstanceVisible(*id, hidden),
                    ))
                    .push(item(
                        "Isolate body".into(),
                        Message::CadIsolateInstance(*id),
                    ))
                    .push(item("Show all geometry".into(), Message::CadShowAll))
                    .push(item(
                        "Move body…".into(),
                        Message::MoveOpen(TreeNode::Model(*id)),
                    ));
            }
        }
        let menu = container(scrollable(items.padding(4)).height(Length::Shrink))
            .width(244)
            .max_height((self.window.height - self.op_menu_pos.y - 12.0).max(100.0))
            .style(popup_surface);
        let position = iced::Point::new(
            self.op_menu_pos.x.min((self.window.width - 250.0).max(0.0)),
            self.op_menu_pos
                .y
                .min((self.window.height - 110.0).max(0.0)),
        );
        let main_menu: Element<'_, Message> = container(menu)
            .width(Length::Fill)
            .height(Length::Fill)
            .align_x(iced::alignment::Horizontal::Left)
            .align_y(iced::alignment::Vertical::Top)
            .padding(Padding {
                top: position.y,
                left: position.x,
                right: 0.0,
                bottom: 0.0,
            })
            .into();
        let Some(y) = self.pattern_menu_y else {
            return Some(main_menu);
        };
        let submenu = container(
            column![
                item("Rectangular".into(), Message::BeginPattern(false)),
                item("Circular".into(), Message::BeginPattern(true)),
            ]
            .spacing(2)
            .padding(4),
        )
        .width(180)
        .style(popup_surface);
        let left = if position.x + 244.0 + 180.0 <= self.window.width {
            position.x + 244.0
        } else {
            (position.x - 180.0).max(0.0)
        };
        let submenu = container(submenu)
            .width(Length::Fill)
            .height(Length::Fill)
            .align_x(iced::alignment::Horizontal::Left)
            .align_y(iced::alignment::Vertical::Top)
            .padding(Padding {
                top: y.clamp(0.0, (self.window.height - 80.0).max(0.0)),
                left,
                right: 0.0,
                bottom: 0.0,
            });
        Some(iced::widget::stack![main_menu, submenu].into())
    }
}

fn operation_icon(op: &Operation) -> Icon {
    match op {
        Operation::Profile(_) => Icon::Profile,
        Operation::Drill(_) => Icon::Drill,
        Operation::Pocket(_) => Icon::Pocket,
        Operation::Face(_) => Icon::Face,
        Operation::Chamfer(_) => Icon::Chamfer,
        Operation::Bore(_) => Icon::Bore,
        Operation::Thread(_) => Icon::Thread,
        Operation::Engrave(_) => Icon::Engrave,
        Operation::Carve(_) => Icon::Carve,
        Operation::Adaptive(_) => Icon::Adaptive,
        Operation::Parallel(_) => Icon::Parallel,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn send(app: &mut App, message: Message) {
        let _task = app.update(message);
    }

    fn tree_app() -> (App, Vec<u32>) {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.workspace = Workspace::Machining;
        app.active_tab = RibbonTab::Home;
        app.operation_draft = None;
        app.setup_draft = None;
        app.fields.clear();
        for _ in 0..3 {
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
        }
        app.controller.select(Selection::Setup);
        let ids = app
            .controller
            .inspection_setup()
            .operations
            .iter()
            .map(Operation::id)
            .collect();
        app.begin_setup_draft();
        (app, ids)
    }

    fn add_browser_models(app: &mut App) {
        use cam_model::*;
        for asset_id in 1..=2 {
            let points = vec![[0., 0., 0.], [10., 0., 0.], [0., 10., 0.]];
            let topology = TopologySnapshot {
                bodies: vec![CadBody {
                    id: 1,
                    name: "Plate".into(),
                    valid_solid: true,
                    bounds: [[0., 0., 0.], [10., 10., 0.]],
                }],
                faces: vec![CadFace {
                    id: 2,
                    body_id: 1,
                    surface: CadSurface::Plane {
                        origin: [0.; 3],
                        normal: [0., 0., 1.],
                    },
                    loop_ids: vec![],
                    bounds: [[0., 0., 0.], [10., 10., 0.]],
                }],
                vertices: points
                    .iter()
                    .enumerate()
                    .map(|(i, point)| CadVertex {
                        id: 3 + i as u64,
                        body_id: 1,
                        point: *point,
                    })
                    .collect(),
                mesh: CadMesh {
                    vertices: points,
                    triangles: vec![[0, 1, 2]],
                    triangle_faces: vec![2],
                },
                ..Default::default()
            };
            let instances = (0..2)
                .map(|i| GeometryInstance {
                    id: (asset_id - 1) * 2 + i + 1,
                    asset_id,
                    body_id: 1,
                    name: format!("Plate {i}"),
                    placement: RigidTransform {
                        translation: [((asset_id - 1) * 40 + i * 20) as f64, 0., 0.],
                        ..Default::default()
                    },
                    visible: true,
                })
                .collect();
            app.controller
                .add_cad_asset(
                    GeometryAsset {
                        id: asset_id,
                        revision: 1,
                        name: format!("Model {asset_id}"),
                        source_format: CadFormat::Step,
                        source_data: vec![].into(),
                        import_options: None,
                        import_diagnostics: vec![],
                        source_fingerprint: String::new(),
                        source_units: SourceUnits::Millimetres,
                        backend: "analytic test".into(),
                        topology: topology.into(),
                    },
                    instances,
                )
                .unwrap();
        }
        if !app.controller.setups().is_empty() {
            app.controller
                .edit_setup(|setup| setup.model_instances = vec![1]);
        }
        app.end_setup_draft();
        app.refresh_fields();
    }

    fn visible_bodies(app: &App) -> Vec<u64> {
        let scene = app.machining_viewport().cad_scene;
        (1..=4)
            .filter(|id| {
                let x = ((*id - 1) * 20) as f64 + 2.;
                scene
                    .pick_candidates(
                        cam_render::Ray {
                            origin: [x, 2., 10.],
                            direction: [0., 0., -1.],
                        },
                        EntityKind::Body,
                        0.1,
                        false,
                    )
                    .iter()
                    .any(|candidate| candidate.entity.instance_id == *id)
            })
            .collect()
    }

    #[test]
    fn moving_model_previews_in_inspector_and_cancel_discards() {
        let (mut app, _) = tree_app();
        add_browser_models(&mut app);
        let before = app.controller.geometry_state().instance(1).unwrap().placement;
        let _ = app.update(Message::MoveOpen(TreeNode::Asset(1)));
        assert!(app.pane_handle(Pane::Inspector).is_some());
        let _ = app.update(Message::MoveValue(0, "10".into()));
        assert_eq!(app.controller.geometry_state().instance(1).unwrap().placement, before);
        let scene = app.machining_viewport().cad_scene;
        assert!(scene.pick_candidates(cam_render::Ray { origin: [12., 2., 10.], direction: [0.,0.,-1.] }, EntityKind::Body, 0.1, false).iter().any(|c| c.entity.instance_id == 1));
        let _ = app.update(Message::MoveCancel);
        assert_eq!(app.controller.geometry_state().instance(1).unwrap().placement, before);
        let _ = app.update(Message::MoveOpen(TreeNode::Asset(1)));
        let _ = app.update(Message::MoveValue(0, "10".into()));
        let _ = app.update(Message::MoveApply);
        assert_eq!(app.controller.geometry_state().instance(1).unwrap().placement.translation[0], 10.);
        assert_eq!(app.controller.geometry_state().instance(2).unwrap().placement.translation[0], 30.);
        assert!(app.controller.undo());
        assert_eq!(app.controller.geometry_state().instance(1).unwrap().placement, before);
    }

    #[test]
    fn move_gizmo_tracks_rotation_and_drags_in_local_axes() {
        let (mut app, _) = tree_app();
        add_browser_models(&mut app);
        let _ = app.update(Message::MoveOpen(TreeNode::Model(1)));
        let _ = app.update(Message::MoveValue(5, "90".into()));
        for index in 6..9 { let _ = app.update(Message::MoveValue(index, "0".into())); }
        let camera = app.machining_viewport().camera();
        let axes = app.move_handles(camera).unwrap().axes;
        assert!(axes[0][0].abs() < 1e-9 && (axes[0][1] - 1.).abs() < 1e-9);
        let base = [0., 0., 0., 0., 0., 90., 0., 0., 0.];
        let _ = app.update(Message::MoveHandleStart(0, base, axes[0]));
        let _ = app.update(Message::MoveHandleInput("5".into()));
        let base = [0., 5., 0., 0., 0., 90., 0., 0., 0.];
        let _ = app.update(Message::MoveHandleStart(3, base, axes[0]));
        let _ = app.update(Message::MoveHandleInput("90".into()));
        let _ = app.update(Message::MoveApply);
        let placement = app.controller.geometry_state().instance(1).unwrap().placement;
        assert!(placement.translation[0].abs() < 1e-8 && (placement.translation[1] - 5.).abs() < 1e-8);
        let rotated_z = placement.transform_vector([0., 0., 1.]);
        assert!((rotated_z[0] - 1.).abs() < 1e-8 && rotated_z[1].abs() < 1e-8 && rotated_z[2].abs() < 1e-8);
    }

    #[test]
    fn move_rotation_remains_correct_through_all_quadrants() {
        let (mut app, _) = tree_app();
        add_browser_models(&mut app);
        let _ = app.update(Message::MoveOpen(TreeNode::Model(1)));
        for index in 6..9 { let _ = app.update(Message::MoveValue(index, "0".into())); }
        let _ = app.update(Message::MoveHandleStart(4, [0.; 9], [0., 1., 0.]));
        for angle in [-360_f64, -270., -180., -90., 0., 90., 180., 270., 360., 450.] {
            let _ = app.update(Message::MoveHandleDelta(angle));
            let axes = app.move_handles(app.machining_viewport().camera()).unwrap().axes;
            let (sin, cos) = angle.to_radians().sin_cos();
            assert!((axes[0][0] - cos).abs() < 1e-8 && (axes[0][2] + sin).abs() < 1e-8, "angle {angle}: {:?}", axes[0]);
        }
    }

    #[test]
    fn moving_body_only_changes_selected_instance() {
        let (mut app, _) = tree_app();
        add_browser_models(&mut app);
        let _ = app.update(Message::MoveOpen(TreeNode::Model(1)));
        let _ = app.update(Message::MoveValue(1, "5".into()));
        let _ = app.update(Message::MoveApply);
        assert_eq!(app.controller.geometry_state().instance(1).unwrap().placement.translation, [0., 5., 0.]);
        assert_eq!(app.controller.geometry_state().instance(2).unwrap().placement.translation, [20., 0., 0.]);
    }

    #[test]
    fn radio_view_survives_operation_selection_and_visibility_changes() {
        let (mut app, ids) = tree_app();
        add_browser_models(&mut app);
        let setup = app.controller.active_setup_id();
        send(
            &mut app,
            Message::Tree(TreeMessage::View(TreeNode::Setup(setup))),
        );
        assert!(app.model_view.is_none());
        send(&mut app, Message::ClickOp(ids[0]));
        send(&mut app, Message::ClickOp(ids[0]));
        assert_eq!(app.browser_view, TreeNode::Setup(setup));
        assert!(app.model_view.is_none());
        send(
            &mut app,
            Message::Tree(TreeMessage::View(TreeNode::Asset(2))),
        );
        send(&mut app, Message::ClickOp(ids[0]));
        assert!(app.model_view.is_none());
        send(&mut app, Message::SetAssetVisible(2, false));
        assert_eq!(app.browser_view, TreeNode::Asset(2));
        send(&mut app, Message::ClickOp(ids[0]));
        assert_eq!(app.model_view, Some(TreeNode::Asset(2)));
        assert!(app.viewport_hidden_instances().contains(&3));
        send(&mut app, Message::SetAssetVisible(2, true));
        assert_eq!(visible_bodies(&app), [3, 4]);
    }

    #[test]
    fn transparent_space_below_browser_accepts_viewport_input() {
        let bounds = iced::Rectangle::new(iced::Point::ORIGIN, iced::Size::new(1000., 800.));
        assert!(browser_region_contains(
            iced::Point::new(100., 80.),
            bounds,
            280.,
            200.
        ));
        assert!(!browser_region_contains(
            iced::Point::new(100., 400.),
            bounds,
            280.,
            200.
        ));
        assert!(!browser_region_contains(
            iced::Point::new(400., 80.),
            bounds,
            280.,
            200.
        ));
        let (mut app, _) = tree_app();
        let expanded = app.project_browser_height();
        send(&mut app, Message::Tree(TreeMessage::ToggleSetups));
        assert!(app.project_browser_height() < expanded);
    }

    #[test]
    fn missing_radio_target_falls_back_to_models() {
        let (mut app, _) = tree_app();
        app.browser_view = TreeNode::Setup(u64::MAX);
        app.clear_browser_selection();
        assert_eq!(app.browser_view, TreeNode::Models);
        assert_eq!(app.model_view, Some(TreeNode::Models));
    }

    #[test]
    fn body_picks_reveal_their_tree_rows_and_seed_the_first_setup() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        add_browser_models(&mut app);
        app.reset_model_browser();
        assert!(app.controller.setups().is_empty());
        assert_eq!(visible_bodies(&app), [1, 2, 3, 4]);
        app.collapsed_assets.insert(1);
        let body = EntityRef {
            asset_id: 1,
            revision: 1,
            instance_id: 2,
            body_id: 1,
            kind: EntityKind::Body,
            id: 1,
        };
        send(
            &mut app,
            Message::CadPicked(vec![PickCandidate {
                entity: body.clone(),
                depth: 0.,
                occluded: false,
            }]),
        );
        assert_eq!(app.controller.cad_selection(), &[body]);
        assert!(app.tree_body_selected(2));
        assert!(!app.tree_body_selected(1));
        assert!(!app.collapsed_assets.contains(&1));
        assert_eq!(visible_bodies(&app), [1, 2, 3, 4]);
        let _ = app.project_tree();
        send(&mut app, Message::SetupNew);
        assert_eq!(app.controller.setups().len(), 1);
        assert_eq!(app.setup_draft.as_ref().unwrap().model_instances, [2]);
        assert_eq!(app.controller.document().setup().model_instances, [2]);
        assert!(app.controller.undo());
        assert!(app.controller.setups().is_empty());
        app.end_setup_draft();
        assert_eq!(visible_bodies(&app), [1, 2, 3, 4]);
        assert!(app.controller.redo());
        assert_eq!(app.controller.document().setup().model_instances, [2]);
    }

    #[test]
    fn new_setups_use_tree_body_or_model_selection() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        add_browser_models(&mut app);
        let _ = app.select_tree_node(&TreeNode::Model(4));
        assert!(app.tree_body_selected(4));
        send(&mut app, Message::SetupNew);
        assert_eq!(app.controller.document().setup().model_instances, [4]);
        send(&mut app, Message::SetupCancel);
        let _ = app.select_tree_node(&TreeNode::Asset(1));
        send(&mut app, Message::SetupNew);
        assert_eq!(app.controller.document().setup().model_instances, [1, 2]);
    }

    #[test]
    fn projects_without_setups_round_trip_and_use_the_filename_as_browser_title() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        assert_eq!(app.project_title(), "Untitled");
        assert!(app.controller.setups().is_empty());
        let _ = app.view();
        add_browser_models(&mut app);
        app.reset_model_browser();
        let path = std::env::temp_dir().join(format!("Browser title {}.ocam", std::process::id()));
        app.save_to(&path);
        assert_eq!(
            app.project_title(),
            path.file_stem().unwrap().to_str().unwrap()
        );
        let (reopened, _) = App::new_with_setup(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert!(reopened.controller.setups().is_empty());
        assert_eq!(reopened.controller.geometry_state().instances.len(), 4);
        assert_eq!(visible_bodies(&reopened), [1, 2, 3, 4]);
        let _ = reopened.view();
    }

    #[test]
    fn saved_project_restores_browser_selection_expansion_and_visibility() {
        let (mut app, ids) = tree_app();
        add_browser_models(&mut app);
        let first_setup = app.controller.active_setup_id();
        let second_setup = app.controller.add_setup_with_models(vec![3, 4]);
        let copied = app.controller.duplicate_setup(first_setup).unwrap();
        app.controller.activate_setup(copied).unwrap();
        let operation = app.controller.document().setup().operations[0].id();
        app.controller.edit_setup(|setup| {
            setup.model_instances = vec![3, 4];
            setup.patterns.push(cam_model::OperationPattern {
                id: 17,
                name: "Copies".into(),
                members: vec![operation],
                kind: cam_model::PatternKind::Rectangular {
                    nx: 2,
                    ny: 1,
                    dx: 20.,
                    dy: 0.,
                },
                order: cam_model::PatternOrder::ByInstance,
                suppressed: false,
                options: Default::default(),
            });
        });
        app.controller.select(Selection::Operation(operation));
        app.controller.set_cad_selection(vec![EntityRef {
            asset_id: 2,
            revision: 1,
            instance_id: 4,
            body_id: 1,
            kind: EntityKind::Body,
            id: 1,
        }]);
        app.model_view = None;
        app.browser_view = TreeNode::Setup(second_setup);
        app.models_collapsed = false;
        app.setups_collapsed = true;
        app.expanded_setups = BTreeSet::from([copied]);
        app.collapsed_assets = BTreeSet::from([1]);
        app.expanded_operations = BTreeSet::from([operation]);
        app.expanded_patterns = BTreeSet::from([17]);
        app.tools_expanded = true;
        app.hidden_assets = BTreeSet::from([1]);
        app.cad_hidden = BTreeSet::from([3]);
        app.hidden_setups = BTreeSet::from([first_setup]);
        app.hidden_ops = BTreeSet::from([ids[0]]);
        app.focus_ops = BTreeSet::from([operation]);
        app.show_stock = true;
        app.show_origin = false;
        let expected = app.capture_project_view();
        let bodies = visible_bodies(&app);
        let path = std::env::temp_dir().join(format!("browser-view-{}.ocam", std::process::id()));
        app.save_to(&path);
        assert!(app.status.starts_with("Saved"), "{}", app.status);
        let (reopened, _) = App::new_with_setup(&path).unwrap();
        assert_eq!(reopened.capture_project_view(), expected);
        assert!(!reopened.project_view_changed());
        assert_eq!(visible_bodies(&reopened), bodies);
        // Headless consumers must retain the desktop state when saving a project.
        let mut controller = AppController::new(crate::default_machine());
        controller.open_project(&path).unwrap();
        controller.save_project(&path).unwrap();
        let (reopened, _) = App::new_with_setup(&path).unwrap();
        assert_eq!(reopened.capture_project_view(), expected);
        let mut reopened = reopened;
        reopened.tools_expanded = !reopened.tools_expanded;
        assert!(reopened.project_view_changed());
        assert!(reopened.has_changes_to_discard());
        assert!(reopened.window_title().ends_with(" *"));
        reopened.save_to(&path.join("cannot-save.ocam"));
        assert!(reopened.status.starts_with("Save failed:"));
        assert!(reopened.project_view_changed());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn legacy_project_resets_browser_state_instead_of_inheriting_previous_project() {
        let (mut app, _) = tree_app();
        let path =
            std::env::temp_dir().join(format!("legacy-browser-view-{}.ocam", std::process::id()));
        app.controller.save_project(&path).unwrap();
        app.expanded_setups.insert(99);
        app.expanded_patterns.insert(99);
        app.hidden_assets.insert(1);
        app.models_collapsed = false;
        app.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert_eq!(app.controller.selection(), Selection::Setup);
        assert_eq!(
            app.expanded_setups,
            BTreeSet::from([app.controller.active_setup_id()])
        );
        assert!(app.expanded_patterns.is_empty());
        assert!(app.hidden_assets.is_empty());
        assert!(app.models_collapsed);
    }

    fn drain_context_job(app: &mut App) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(15);
        while app.controller.run_needs_poll() {
            send(app, Message::PollRun);
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        while app.notice.is_some() {
            send(app, Message::NoticeDismissed);
        }
    }

    #[test]
    fn operation_selection_and_context_menu_open_editor_only_on_edit() {
        let (mut app, ids) = tree_app();
        app.end_setup_draft();
        app.refresh_fields();
        send(&mut app, Message::ClickOp(ids[0]));
        assert_eq!(app.controller.selection(), Selection::Operation(ids[0]));
        assert!(app.operation_draft.is_none());
        assert!(app.pane_handle(Pane::Inspector).is_none());
        send(
            &mut app,
            Message::Tree(TreeMessage::Open(TreeNode::Operation(ids[1]))),
        );
        assert_eq!(app.controller.selection(), Selection::Operation(ids[1]));
        assert!(app.operation_draft.is_none());
        assert!(app.pane_handle(Pane::Inspector).is_none());
        send(
            &mut app,
            Message::Tree(TreeMessage::Command(Box::new(Message::Select(
                Selection::Operation(ids[1]),
            )))),
        );
        assert!(app.operation_draft.is_some());
        assert!(app.pane_handle(Pane::Inspector).is_some());
        send(&mut app, Message::ClickOp(ids[0]));
        assert!(app.operation_draft.is_none());
        assert!(app.pane_handle(Pane::Inspector).is_none());
    }

    #[test]
    fn operation_double_click_from_setup_radio_view_opens_editor() {
        let (mut app, ids) = tree_app();
        let setup = app.controller.active_setup_id();
        send(
            &mut app,
            Message::Tree(TreeMessage::View(TreeNode::Setup(setup))),
        );
        let before = iced::advanced::widget::Tree::new(&app.workspace_main());
        let browser_index = before.children[0].children.len() - 1;
        send(
            &mut app,
            Message::Tree(TreeMessage::Press(TreeNode::Operation(ids[0]))),
        );
        send(&mut app, Message::Tree(TreeMessage::Release));
        let after = iced::advanced::widget::Tree::new(&app.workspace_main());
        assert_eq!(
            after.children[0].children.len() - 1,
            browser_index,
            "selection must preserve the browser widget and its double-click state"
        );
        send(
            &mut app,
            Message::Tree(TreeMessage::Press(TreeNode::Operation(ids[0]))),
        );
        send(
            &mut app,
            Message::Tree(TreeMessage::DoubleClick(TreeNode::Operation(ids[0]))),
        );
        send(&mut app, Message::Tree(TreeMessage::Release));
        assert_eq!(
            app.operation_draft.as_ref().map(Operation::id),
            Some(ids[0]),
            "{}",
            app.status
        );
        assert!(app.pane_handle(Pane::Inspector).is_some());
    }

    #[test]
    fn browsing_with_stale_fields_can_open_operation_editor() {
        let (mut app, ids) = tree_app();
        app.end_setup_draft();
        send(&mut app, Message::ClickOp(ids[0]));
        let field = app.inspector_fields().into_iter().next().unwrap();
        app.fields.insert(field, "999999".into());
        assert!(app.inspector_dirty());
        assert!(!app.inspector_active);
        app.inspector_enabled = false;

        send(
            &mut app,
            Message::Tree(TreeMessage::Open(TreeNode::Operation(ids[0]))),
        );
        send(
            &mut app,
            Message::Tree(TreeMessage::Command(Box::new(Message::Select(
                Selection::Operation(ids[0]),
            )))),
        );
        assert_eq!(
            app.operation_draft.as_ref().map(Operation::id),
            Some(ids[0])
        );
        assert!(app.pane_handle(Pane::Inspector).is_some());
    }

    #[test]
    fn operation_double_click_opens_editor_even_when_inspector_is_hidden() {
        let (mut app, ids) = tree_app();
        app.inspector_enabled = false;
        app.tree_drag = Some(TreeDrag {
            node: TreeNode::Operation(ids[0]),
            start: iced::Point::ORIGIN,
            dragging: false,
            edit_on_release: false,
        });
        send(
            &mut app,
            Message::Tree(TreeMessage::DoubleClick(TreeNode::Operation(ids[0]))),
        );
        // The global release after the double-click must not clear the editor.
        send(&mut app, Message::Tree(TreeMessage::Release));
        assert_eq!(app.controller.selection(), Selection::Operation(ids[0]));
        assert_eq!(
            app.operation_draft.as_ref().map(Operation::id),
            Some(ids[0])
        );
        assert!(app.pane_handle(Pane::Inspector).is_some());
    }

    #[test]
    fn selected_operation_context_menu_preserves_multiselection_for_pattern_creation() {
        let (mut app, ids) = tree_app();
        app.end_setup_draft();
        app.focus_ops = ids.iter().copied().collect();
        send(
            &mut app,
            Message::Tree(TreeMessage::Open(TreeNode::Operation(ids[1]))),
        );
        assert_eq!(app.focus_ops, ids.iter().copied().collect());
        assert_eq!(app.controller.selection(), Selection::Operation(ids[1]));
        send(
            &mut app,
            Message::Tree(TreeMessage::Command(Box::new(Message::BeginPattern(false)))),
        );
        assert_eq!(app.pattern_draft.as_ref().unwrap().members, ids);
    }

    #[test]
    fn pattern_submenu_opens_visible_editors_from_setup_and_operation_menus() {
        for circular in [false, true] {
            for from_setup in [false, true] {
                let (mut app, ids) = tree_app();
                app.end_setup_draft();
                app.inspector_enabled = false;
                let node = if from_setup {
                    TreeNode::Setup(app.controller.active_setup_id())
                } else {
                    TreeNode::Operation(ids[0])
                };
                send(&mut app, Message::Tree(TreeMessage::Open(node.clone())));
                send(&mut app, Message::Tree(TreeMessage::ShowPatternMenu));
                assert_eq!(app.tree_menu, Some(node.clone()));
                assert!(app.pattern_menu_y.is_some());
                assert!(
                    app.pattern_draft.is_none(),
                    "opening the submenu does not start a draft"
                );
                assert!(app.tree_menu_overlay().is_some());

                send(&mut app, Message::Tree(TreeMessage::Close));
                assert!(app.pattern_menu_y.is_none());
                assert!(app.pattern_draft.is_none());

                send(&mut app, Message::Tree(TreeMessage::Open(node)));
                send(&mut app, Message::Tree(TreeMessage::ShowPatternMenu));
                send(
                    &mut app,
                    Message::Tree(TreeMessage::Command(Box::new(Message::BeginPattern(
                        circular,
                    )))),
                );
                assert!(app.tree_menu.is_none());
                assert!(app.pattern_menu_y.is_none());
                assert_eq!(app.pattern_draft.as_ref().unwrap().circular, circular);
                assert!(app.pane_handle(Pane::Inspector).is_some());
                send(&mut app, Message::PatternCancel);
                assert!(app.pane_handle(Pane::Inspector).is_none());
            }
        }
    }

    #[test]
    fn simulation_context_commands_open_scoped_player_and_clear_it_after_edits() {
        let (mut app, ids) = tree_app();
        app.end_setup_draft();
        app.refresh_fields();
        send(
            &mut app,
            Message::Tree(TreeMessage::Open(TreeNode::Operation(ids[1]))),
        );
        send(
            &mut app,
            Message::Tree(TreeMessage::Command(Box::new(Message::ToolpathCommand(
                TreeNode::Operation(ids[1]),
                ToolpathAction::Simulate,
            )))),
        );
        assert!(app.tree_menu.is_none());
        assert!(app.open_op_menu.is_none());
        drain_context_job(&mut app);
        assert!(app.simulation.is_some());
        assert!(app.show_stock);
        assert!(
            app.pane_handle(Pane::Inspector).is_some(),
            "simulation exposes its menu"
        );
        let frame_bounds = app.machining_viewport().frame_bounds;
        app.show_stock = false;
        app.settings.view.transparent_stock = true;
        app.simulation.as_mut().unwrap().options.show_stock = false;
        let tool_only = app.machining_viewport();
        assert!(
            !tool_only.mesh_indices.is_empty(),
            "playback shows a solid cutter"
        );
        assert!(tool_only.mesh_vertices.iter().all(|v| v.color[3] == 1.));
        assert!(tool_only.transparent_stock_indices.is_empty());
        assert_eq!(
            tool_only.frame_bounds, frame_bounds,
            "the moving tool must not reframe the part"
        );
        app.simulation.as_mut().unwrap().options.transparent_tool = true;
        let transparent_tool = app.machining_viewport();
        assert!(transparent_tool.mesh_indices.is_empty());
        assert!(!transparent_tool.transparent_stock_indices.is_empty());
        assert!(transparent_tool
            .transparent_stock_vertices
            .iter()
            .all(|v| v.color[3] == 0.3));
        app.simulation.as_mut().unwrap().options.show_tool = false;
        let hidden = app.machining_viewport();
        assert!(hidden.mesh_indices.is_empty());
        assert!(hidden.transparent_stock_indices.is_empty());
        app.simulation.as_mut().unwrap().options.show_tool = true;
        app.simulation.as_mut().unwrap().options.show_stock = true;
        app.simulation.as_mut().unwrap().options.transparent_tool = false;
        app.show_stock = true;
        assert!(app
            .controller
            .outcome()
            .unwrap()
            .scene
            .strips
            .iter()
            .filter_map(|strip| strip.op)
            .all(|id| id == ids[0] || id == ids[1]));
        assert!(
            app.simulation.as_ref().unwrap().position > 0.,
            "operation playback opens at the selected operation after its predecessors"
        );
        {
            send(
                &mut app,
                Message::SimulationOption(crate::gui::SimulationOption::Mode(
                    crate::settings::SimulationToolpathMode::All,
                )),
            );
            let player = app.simulation.as_ref().unwrap();
            assert!(player
                .preview
                .scene
                .strips
                .iter()
                .any(|s| s.op == Some(ids[1])));
            assert!(player
                .preview
                .scene
                .strips
                .iter()
                .filter_map(|s| s.op)
                .all(|id| id == ids[1]));
        }
        send(&mut app, Message::SimulationSpeed(4.));
        send(&mut app, Message::SimulationPlayPause);
        assert!(app.simulation.as_ref().unwrap().playing);
        assert_eq!(app.simulation.as_ref().unwrap().speed, 4.);
        send(&mut app, Message::SimulationSeek(0.));
        assert!(!app.simulation.as_ref().unwrap().playing);
        let setup = app.controller.active_setup_id();
        send(
            &mut app,
            Message::ToolpathCommand(TreeNode::Setup(setup), ToolpathAction::Simulate),
        );
        drain_context_job(&mut app);
        assert!(app.simulation.is_some());
        let visible: BTreeSet<_> = app
            .controller
            .outcome()
            .unwrap()
            .scene
            .strips
            .iter()
            .filter_map(|strip| strip.op)
            .collect();
        assert_eq!(visible, ids.iter().copied().collect());
        send(&mut app, Message::SetOpExcluded(ids[0], true));
        assert!(app.simulation.is_none());
    }

    #[test]
    fn context_export_is_cancelled_and_stale_targets_do_not_process_another_scope() {
        let (mut app, ids) = tree_app();
        app.end_setup_draft();
        app.refresh_fields();
        send(
            &mut app,
            Message::ToolpathCommand(TreeNode::Operation(ids[0]), ToolpathAction::Export),
        );
        assert!(app.pending_toolpath_export.is_some());
        send(&mut app, Message::CancelRun);
        assert!(app.pending_toolpath_export.is_none());
        send(
            &mut app,
            Message::ToolpathCommand(TreeNode::Operation(u32::MAX), ToolpathAction::Recompute),
        );
        assert!(app.status.contains("no longer exists"));
        assert!(!app.controller.run_pending());
    }

    #[test]
    fn closing_operation_simulation_then_selecting_a_later_operation_restores_its_path() {
        let (mut app, ids) = tree_app();
        app.end_setup_draft();
        app.refresh_fields();
        send(
            &mut app,
            Message::ToolpathCommand(TreeNode::Operation(ids[0]), ToolpathAction::Simulate),
        );
        drain_context_job(&mut app);
        assert!(app.simulation.is_some());
        let verified = app.controller.outcome().unwrap().program.clone();
        assert!(verified
            .steps()
            .iter()
            .filter_map(Step::tag)
            .all(|t| t.op_id == ids[0]));
        send(&mut app, Message::SimulationClose);
        send(&mut app, Message::ClickOp(ids[2]));
        app.drain_operation_preview();
        let preview = app.operation_preview_outcome().unwrap();
        assert_eq!(preview.completion, JobCompletion::Complete);
        assert!(preview
            .scene
            .strips
            .iter()
            .any(|strip| strip.op == Some(ids[2])));
        assert!(app
            .machining_viewport()
            .path_cones
            .iter()
            .any(|cone| cone.op == ids[2]));
        assert_eq!(
            app.controller.outcome().unwrap().program,
            verified,
            "browser previews leave the verified export program intact"
        );
    }

    #[test]
    fn changing_selection_and_cancelling_preview_discards_obsolete_work() {
        let (mut app, ids) = tree_app();
        app.end_setup_draft();
        app.refresh_fields();
        app.controller.run(&cam_toolpath::CancelToken::new());
        let verified = app.controller.outcome().unwrap().program.clone();
        send(&mut app, Message::ClickOp(ids[2]));
        send(&mut app, Message::ClickOp(ids[0]));
        app.drain_operation_preview();
        let preview = app.operation_preview_outcome().unwrap();
        assert!(preview
            .program
            .steps()
            .iter()
            .filter_map(Step::tag)
            .all(|tag| tag.op_id == ids[0]));
        send(&mut app, Message::ClickOp(ids[1]));
        send(&mut app, Message::CancelOperationPreview);
        app.drain_operation_preview();
        assert!(app.operation_preview_outcome().is_none());
        assert_eq!(app.controller.outcome().unwrap().program, verified);
        send(&mut app, Message::ClickOp(ids[2]));
        app.drain_operation_preview();
        assert!(app
            .operation_preview_outcome()
            .unwrap()
            .scene
            .strips
            .iter()
            .any(|s| s.op == Some(ids[2])));
    }

    #[test]
    #[ignore = "uses the supplied local testfiles/tray-parallel.ocam CAD project"]
    fn tray_parallel_browser_stock_and_paths_follow_selection_after_simulation() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testfiles/tray-parallel.ocam");
        app.controller.open_project(path).unwrap();
        app.end_setup_draft();
        app.operation_draft = None;
        app.show_stock = true;
        app.refresh_fields();
        let parallel = app
            .controller
            .inspection_setup()
            .operations
            .iter()
            .find(|op| matches!(op, Operation::Parallel(_)))
            .unwrap()
            .id();
        let profile = app
            .controller
            .inspection_setup()
            .operations
            .iter()
            .find(|op| matches!(op, Operation::Profile(_)))
            .unwrap()
            .id();
        send(&mut app, Message::ClickOp(parallel));
        app.drain_operation_preview();
        let preview = app.operation_preview_outcome().unwrap();
        assert_eq!(
            preview.completion,
            JobCompletion::Complete,
            "{:?}",
            preview.diagnostics
        );
        assert!(preview.stock_operations.contains(&Some(parallel)));
        assert!(!preview.stock_operations.contains(&Some(profile)));
        assert!(
            app.controller.outcome().is_none(),
            "preview is independent of verification"
        );

        send(
            &mut app,
            Message::ToolpathCommand(TreeNode::Operation(parallel), ToolpathAction::Simulate),
        );
        drain_context_job(&mut app);
        assert!(app.simulation.is_some());
        send(&mut app, Message::SimulationClose);
        app.drain_operation_preview();
        assert!(!app
            .operation_preview_outcome()
            .unwrap()
            .stock_operations
            .contains(&Some(profile)));
        let parallel_stamp = app.operation_preview_outcome().unwrap().stamp.clone();
        send(&mut app, Message::ClickOp(profile));
        app.drain_operation_preview();
        let preview = app.operation_preview_outcome().unwrap();
        assert_eq!(
            preview.completion,
            JobCompletion::Complete,
            "{:?}",
            preview.diagnostics
        );
        assert!(preview.stock_operations.contains(&Some(profile)));
        assert!(preview.scene.strips.iter().any(|s| s.op == Some(profile)));
        assert!(app
            .machining_viewport()
            .path_cones
            .iter()
            .any(|c| c.op == profile));
        let profile_stamp = preview.stamp.clone();
        for (id, stamp) in [(parallel, parallel_stamp), (profile, profile_stamp)] {
            let start = std::time::Instant::now();
            send(&mut app, Message::ClickOp(id));
            assert!(
                app.operation_preview.progress().is_none(),
                "unchanged operation must not start a worker"
            );
            assert_eq!(app.operation_preview_outcome().unwrap().stamp, stamp);
            let _ = app.machining_viewport();
            eprintln!("tray cached operation {id}: {:?}", start.elapsed());
        }
    }

    #[test]
    fn processing_time_is_scoped_to_the_requested_operation_and_cancelled_runs_show_nothing() {
        let (mut app, ids) = tree_app();
        app.end_setup_draft();
        app.refresh_fields();
        send(
            &mut app,
            Message::ToolpathCommand(TreeNode::Operation(ids[1]), ToolpathAction::ProcessingTime),
        );
        assert_eq!(app.focus_ops, BTreeSet::from([ids[1]]));
        assert!(app.pending_processing_time.is_some());
        drain_context_job(&mut app);
        assert!(app.pending_processing_time.is_none());
        assert!(app.processing_time_dialog.is_some());
        let selected_index = ids.iter().position(|id| *id == ids[1]).unwrap() as u32;
        let motion_ops: BTreeSet<_> = app
            .controller
            .outcome()
            .unwrap()
            .program
            .steps()
            .iter()
            .filter_map(cam_cldata::Step::tag)
            .map(|tag| tag.op_id)
            .collect();
        assert_eq!(motion_ops, BTreeSet::from([selected_index]));
        send(&mut app, Message::ProcessingTimeFeedOverride("0".into()));
        assert!(app
            .processing_time_dialog
            .as_ref()
            .unwrap()
            .estimate
            .is_err());
        send(&mut app, Message::ProcessingTimeClose);
        assert!(app.processing_time_dialog.is_none());

        let setup_id = app.controller.active_setup_id();
        send(
            &mut app,
            Message::ToolpathCommand(TreeNode::Setup(setup_id), ToolpathAction::ProcessingTime),
        );
        assert!(app.pending_processing_time.is_some());
        send(&mut app, Message::CancelRun);
        assert!(app.pending_processing_time.is_none());
        assert!(app.processing_time_dialog.is_none());

        let setup_id = app.controller.active_setup_id();
        send(
            &mut app,
            Message::ToolpathCommand(TreeNode::Setup(setup_id), ToolpathAction::ProcessingTime),
        );
        drain_context_job(&mut app);
        assert!(app.processing_time_dialog.is_some());
        let setup_motion_ops: BTreeSet<_> = app
            .controller
            .outcome()
            .unwrap()
            .program
            .steps()
            .iter()
            .filter_map(cam_cldata::Step::tag)
            .map(|tag| tag.op_id)
            .collect();
        assert_eq!(setup_motion_ops, BTreeSet::from([0, 1, 2]));
    }

    #[test]
    fn model_and_operation_expansion_and_whole_model_visibility_are_view_only() {
        let (mut app, ids) = tree_app();
        add_browser_models(&mut app);
        let before = app.controller.document().clone();
        assert!(!app.tools_expanded);
        assert!(app.expanded_operations.is_empty());
        send(&mut app, Message::ToggleToolsExpanded);
        send(&mut app, Message::ToggleOperationExpanded(ids[0]));
        send(&mut app, Message::ToggleAssetExpanded(1));
        assert!(app.tools_expanded && app.expanded_operations.contains(&ids[0]));
        assert!(app.collapsed_assets.contains(&1));
        // A parent hide preserves the user's per-body choices.
        app.model_view = Some(TreeNode::Models);
        send(&mut app, Message::SetInstanceVisible(1, false));
        send(&mut app, Message::SetAssetVisible(1, false));
        assert_eq!(visible_bodies(&app), [3, 4]);
        send(&mut app, Message::SetAssetVisible(1, true));
        assert_eq!(visible_bodies(&app), [2, 3, 4]);
        assert_eq!(app.controller.document(), &before);
        let _ = app.project_tree();
    }

    #[test]
    fn tree_body_selection_highlights_with_siblings_visible() {
        let (mut app, _) = tree_app();
        add_browser_models(&mut app);
        let before = app.controller.document().clone();
        let _ = app.select_tree_node(&TreeNode::Model(1));
        assert_eq!(visible_bodies(&app), [1, 2]);
        assert!(app.controller.cad_selection().is_empty(), "tree highlighting must not change machining selection");
        let view = app.machining_viewport();
        let selected = [0.20, 0.60, 0.98, 1.];
        let neutral = [0.55, 0.58, 0.63, 1.];
        assert!(view.mesh_vertices.iter().any(|v| v.position[0] < 15. && v.color == selected));
        assert!(view.mesh_vertices.iter().any(|v| v.position[0] > 15. && v.color == neutral));
        let _ = app.select_tree_node(&TreeNode::Model(2));
        let view = app.machining_viewport();
        assert!(view.mesh_vertices.iter().any(|v| v.position[0] < 15. && v.color == neutral));
        assert!(view.mesh_vertices.iter().any(|v| v.position[0] > 15. && v.color == selected));
        send(&mut app, Message::CadIsolateInstance(2));
        assert_eq!(visible_bodies(&app), [2]);
        send(&mut app, Message::CadShowAll);
        assert_eq!(visible_bodies(&app), [1, 2]);
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn tree_selection_scopes_models_setups_operations_and_member_picking() {
        let (mut app, ids) = tree_app();
        add_browser_models(&mut app);
        let first_setup = app.controller.active_setup_id();
        let second_setup = app.controller.add_setup();
        app.controller
            .edit_setup(|setup| setup.model_instances = vec![4]);
        app.refresh_fields();
        send(&mut app, Message::SelectSetup(second_setup));
        assert_eq!(visible_bodies(&app), [4]);
        let _ = app.select_tree_node(&TreeNode::Operation(ids[0]));
        assert_eq!(app.controller.active_setup_id(), first_setup);
        assert_eq!(visible_bodies(&app), [1]);
        let _ = app.select_tree_node(&TreeNode::Asset(2));
        assert_eq!(visible_bodies(&app), [3, 4]);
        assert!(!app.machining_viewport().show_stock);
        assert!(app
            .machining_viewport()
            .setup_geometry
            .stock_bounds()
            .is_none());
        let _ = app.select_tree_node(&TreeNode::Model(2));
        assert_eq!(visible_bodies(&app), [1, 2]);
        let _ = app.select_tree_node(&TreeNode::Models);
        assert_eq!(visible_bodies(&app), [1, 2, 3, 4]);
        send(&mut app, Message::EditSetup(first_setup));
        assert_eq!(visible_bodies(&app), [1]);
        send(&mut app, Message::SetupModelPick);
        assert_eq!(visible_bodies(&app), [1, 2, 3, 4]);
        send(&mut app, Message::SetupModelPick);
        assert_eq!(visible_bodies(&app), [1]);
        send(&mut app, Message::SetupModelToggled(2, true));
        assert_eq!(
            visible_bodies(&app),
            [1, 2],
            "preview follows the setup draft"
        );
        send(&mut app, Message::SetupCancel);
        assert_eq!(visible_bodies(&app), [1]);
        app.controller
            .edit_setup(|setup| setup.model_instances.clear());
        assert!(
            visible_bodies(&app).is_empty(),
            "empty setup must not show all bodies"
        );
    }

    #[test]
    fn dragging_on_second_click_does_not_open_editor_or_cancel_reorder() {
        let (mut app, ids) = tree_app();
        send(&mut app, Message::ClickOp(ids[2]));
        send(
            &mut app,
            Message::Tree(TreeMessage::Press(TreeNode::Operation(ids[2]))),
        );
        send(
            &mut app,
            Message::Tree(TreeMessage::DoubleClick(TreeNode::Operation(ids[2]))),
        );
        assert!(app.operation_draft.is_none());
        send(
            &mut app,
            Message::Tree(TreeMessage::Hover(TreeNode::Operation(ids[0]), false)),
        );
        send(&mut app, Message::Tree(TreeMessage::Release));
        assert_eq!(
            app.controller
                .inspection_setup()
                .operations
                .iter()
                .map(Operation::id)
                .collect::<Vec<_>>(),
            [ids[2], ids[0], ids[1]]
        );
        assert!(app.operation_draft.is_none());
        app.controller.cancel_run();
    }

    #[test]
    fn dragging_sources_into_within_and_out_of_patterns_is_undoable() {
        let (mut app, ids) = tree_app();
        let setup = app.controller.active_setup_id();
        app.focus_ops = ids[..2].iter().copied().collect();
        send(&mut app, Message::BeginPattern(false));
        send(&mut app, Message::PatternCommit);
        assert!(!app.setup_draft_dirty());
        let pattern = app.controller.inspection_setup().patterns[0].id;
        let original = app.controller.document().clone();
        let drop = |app: &mut App, source, target, after| {
            send(
                app,
                Message::Tree(TreeMessage::Press(TreeNode::Operation(source))),
            );
            send(app, Message::Tree(TreeMessage::Hover(target, after)));
            send(app, Message::Tree(TreeMessage::Release));
            app.controller.cancel_run();
        };
        drop(&mut app, ids[2], TreeNode::Pattern(pattern), false);
        assert_eq!(
            app.controller.inspection_setup().patterns[0].members,
            ids,
            "{}",
            app.status
        );
        drop(&mut app, ids[2], TreeNode::Operation(ids[0]), false);
        assert_eq!(
            app.controller.inspection_setup().patterns[0].members,
            [ids[2], ids[0], ids[1]]
        );
        drop(&mut app, ids[2], TreeNode::Setup(setup), false);
        assert_eq!(
            app.controller.inspection_setup().patterns[0].members,
            ids[..2]
        );
        assert_eq!(
            app.controller
                .inspection_setup()
                .operations
                .iter()
                .map(Operation::id)
                .collect::<Vec<_>>(),
            ids
        );
        for _ in 0..3 {
            assert!(app.controller.undo());
        }
        assert_eq!(app.controller.document(), &original);
    }

    #[test]
    fn crossing_rows_reorders_without_a_global_cursor_update() {
        let (mut app, ids) = tree_app();
        send(
            &mut app,
            Message::Tree(TreeMessage::Press(TreeNode::Operation(ids[2]))),
        );
        send(
            &mut app,
            Message::Tree(TreeMessage::Hover(TreeNode::Operation(ids[0]), false)),
        );
        send(&mut app, Message::Tree(TreeMessage::Release));
        assert_eq!(
            app.controller
                .inspection_setup()
                .operations
                .iter()
                .map(Operation::id)
                .collect::<Vec<_>>(),
            [ids[2], ids[0], ids[1]]
        );
        send(&mut app, Message::Tree(TreeMessage::Release));
        app.controller.cancel_run();
        assert!(app.controller.undo());
        assert_eq!(
            app.controller
                .inspection_setup()
                .operations
                .iter()
                .map(Operation::id)
                .collect::<Vec<_>>(),
            ids
        );
    }

    #[test]
    fn global_pointer_release_publishes_one_undoable_tree_order() {
        let (mut app, ids) = tree_app();
        let before = app.controller.document().clone();
        send(&mut app, Message::WindowCursor(iced::Point::new(10., 10.)));
        send(
            &mut app,
            Message::Tree(TreeMessage::Press(TreeNode::Operation(ids[2]))),
        );
        send(&mut app, Message::WindowCursor(iced::Point::new(10., 40.)));
        send(
            &mut app,
            Message::Tree(TreeMessage::Hover(TreeNode::Operation(ids[0]), false)),
        );
        send(&mut app, Message::Tree(TreeMessage::Release));
        assert_eq!(
            app.controller
                .inspection_setup()
                .operations
                .iter()
                .map(Operation::id)
                .collect::<Vec<_>>(),
            [ids[2], ids[0], ids[1]]
        );
        assert!(app.tree_drag.is_none());
        // A widget release followed by the global release cannot apply twice.
        send(&mut app, Message::Tree(TreeMessage::Release));
        app.controller.cancel_run();
        assert!(app.controller.undo());
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn click_threshold_and_release_outside_do_not_restructure_document() {
        let (mut app, ids) = tree_app();
        let before = app.controller.document().clone();
        send(&mut app, Message::WindowCursor(iced::Point::new(10., 10.)));
        send(
            &mut app,
            Message::Tree(TreeMessage::Press(TreeNode::Operation(ids[1]))),
        );
        send(&mut app, Message::WindowCursor(iced::Point::new(13., 10.)));
        send(&mut app, Message::Tree(TreeMessage::Release));
        assert_eq!(app.controller.selection(), Selection::Operation(ids[1]));
        assert_eq!(app.controller.document(), &before);
        send(
            &mut app,
            Message::Tree(TreeMessage::Press(TreeNode::Operation(ids[1]))),
        );
        send(&mut app, Message::WindowCursor(iced::Point::new(40., 40.)));
        send(
            &mut app,
            Message::Tree(TreeMessage::Hover(TreeNode::Operation(ids[0]), false)),
        );
        send(
            &mut app,
            Message::Tree(TreeMessage::Leave(TreeNode::Operation(ids[0]))),
        );
        send(&mut app, Message::Tree(TreeMessage::Release));
        assert_eq!(app.controller.document(), &before);
        assert!(app.status.contains("cancelled"));
    }

    #[test]
    fn numeric_inspector_drafts_block_tree_selection_and_context_restructure() {
        let (mut app, ids) = tree_app();
        app.controller.select(Selection::Operation(ids[0]));
        app.begin_operation_draft(ids[0]);
        send(&mut app, Message::FieldChanged(Field::Feed, "137".into()));
        assert!(!app.operation_draft_dirty());
        assert!(app.inspector_dirty());
        let before = app.controller.document().clone();
        send(
            &mut app,
            Message::Tree(TreeMessage::Press(TreeNode::Operation(ids[1]))),
        );
        assert!(app.tree_drag.is_none());
        send(
            &mut app,
            Message::Tree(TreeMessage::Open(TreeNode::Operation(ids[1]))),
        );
        assert!(app.tree_menu.is_none());
        assert_eq!(app.controller.selection(), Selection::Operation(ids[0]));
        assert_eq!(
            app.fields.get(&Field::Feed).map(String::as_str),
            Some("137")
        );
        assert_eq!(app.controller.document(), &before);
    }

    #[test]
    fn operation_visibility_and_machining_suppression_are_independent() {
        let (mut app, ids) = tree_app();
        let before = app.controller.document().clone();
        send(&mut app, Message::SetOperationVisible(ids[0], false));
        assert!(app.hidden_ops.contains(&ids[0]));
        assert!(!app.controller.is_operation_excluded(ids[0]));
        assert_eq!(app.controller.document(), &before);
        send(
            &mut app,
            Message::Tree(TreeMessage::Open(TreeNode::Operation(ids[0]))),
        );
        assert!(app.tree_menu_overlay().is_some());
        send(
            &mut app,
            Message::Tree(TreeMessage::Command(Box::new(Message::SetOpExcluded(
                ids[0], true,
            )))),
        );
        app.controller.cancel_run();
        assert!(app.controller.is_operation_excluded(ids[0]));
        assert!(app.hidden_ops.contains(&ids[0]));
        assert!(app.controller.undo());
        assert!(!app.controller.is_operation_excluded(ids[0]));
        assert!(app.hidden_ops.contains(&ids[0]));
    }

    #[test]
    fn context_operation_in_inactive_setup_targets_its_owner() {
        let (mut app, ids) = tree_app();
        let owner = app.controller.active_setup_id();
        let other = app.controller.add_setup();
        app.operation_draft = None;
        app.begin_setup_draft();
        send(
            &mut app,
            Message::Tree(TreeMessage::Open(TreeNode::Operation(ids[1]))),
        );
        assert_ne!(owner, other);
        assert_eq!(app.controller.active_setup_id(), owner);
        assert_eq!(app.controller.selection(), Selection::Operation(ids[1]));
        assert_eq!(app.open_op_menu, Some(ids[1]));
        assert!(app.tree_menu_overlay().is_some());
    }
}
