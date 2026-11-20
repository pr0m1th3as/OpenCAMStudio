//! Operation cutter selection, using project cutters and the shared library table.
use super::*;
use iced::widget::{column, row};

pub(super) struct ToolSelector {
    kind: OpKind,
    previous_search: String,
    previous_filters: [Option<BTreeSet<String>>; 8],
    previous_view: LibraryView,
}

impl ToolSelector {
    pub(super) fn supports(&self, tool: &cam_model::Tool) -> bool {
        families_for(self.kind).contains(&ToolKindPick::of(tool.kind))
    }
}

// Defaults guide the first view; compatibility still permits deliberate substitutions.
fn recommended_families(kind: OpKind) -> &'static [ToolKindPick] {
    use ToolKindPick as F;
    match kind {
        OpKind::Profile | OpKind::Pocket | OpKind::Bore | OpKind::Adaptive => {
            &[F::EndMill, F::RoughingMill, F::BullNose]
        }
        OpKind::Face => &[F::FaceMill, F::EndMill, F::RoughingMill],
        OpKind::Drill => &[F::Drill],
        OpKind::Parallel | OpKind::Spiral => &[F::BallMill],
        OpKind::Thread => &[F::ThreadMill],
        OpKind::Chamfer => &[F::ChamferMill, F::VBit, F::RadiusMill],
        OpKind::Engrave => &[F::ChamferMill, F::VBit],
        OpKind::Carve => &[F::VBit],
    }
}

pub(super) fn tool_button_label(tool: Option<&cam_model::Tool>) -> String {
    let Some(tool) = tool else {
        return "select...".into();
    };
    let label = format!("T{} ⌀{} {}", tool.number, fmt_num(tool.diameter), tool.kind);
    if tool.description.is_empty() {
        label
    } else {
        format!("{label} · {}", tool.description)
    }
}

pub(super) fn selector_message(message: &Message) -> bool {
    matches!(
        message,
        Message::SelectToolLibrary(_)
            | Message::CloseOperationToolSelector
            | Message::OperationToolPicked(_)
            | Message::OperationProjectToolPicked(_)
            | Message::SetLibraryView(_)
            | Message::ToolColumnResizeStart(_)
            | Message::ToolColumnReorderStart(_)
            | Message::ToolColumnHover(_, _)
            | Message::ToolColumnLeave(_)
            | Message::ToolColumnRelease
            | Message::OpenToolTableMenu(_, _)
            | Message::CloseToolTableMenu
            | Message::ToolTableMenuSearch(_)
            | Message::ToolTableFilterValue(_, _)
            | Message::ToolTableFilterAll(_)
            | Message::ApplyToolTableFilter
            | Message::ClearToolTableColumnFilter
            | Message::SetToolTableColumn(_, _)
            | Message::ToolTableSearch(_)
            | Message::ClearToolTableFilters
            | Message::WindowResized(_)
            | Message::WindowCursor(_)
            | Message::ModifiersChanged(_)
            | Message::PollRun
            | Message::PollCadImport
            | Message::NoticeDismissed
    )
}

impl App {
    fn selector_operation(&self) -> Option<&Operation> {
        self.operation_draft.as_ref().or_else(|| {
            if let Selection::Operation(id) = self.controller.selection() {
                self.controller.operation(id)
            } else {
                None
            }
        })
    }

    pub(super) fn open_operation_tool_selector(&mut self) {
        if self.operation_tool_selector.is_some() {
            return;
        }
        let Some(kind) = self.selector_operation().map(AppController::operation_kind) else {
            return;
        };
        self.operation_tool_selector = Some(ToolSelector {
            kind,
            previous_search: std::mem::take(&mut self.tool_table_search),
            previous_filters: std::mem::take(&mut self.tool_table_filters),
            previous_view: self.library_view,
        });
        self.tool_table_filters[2] = Some(
            recommended_families(kind)
                .iter()
                .map(|family| family.to_kind().to_string())
                .collect(),
        );
        self.tool_table_menu = None;
        self.tool_table_drag = None;
    }

    pub(super) fn close_operation_tool_selector(&mut self) {
        if let Some(selector) = self.operation_tool_selector.take() {
            self.tool_table_search = selector.previous_search;
            self.tool_table_filters = selector.previous_filters;
            self.library_view = selector.previous_view;
        }
        self.tool_table_menu = None;
        self.tool_table_drag = None;
    }

    pub(super) fn choose_operation_tool(
        &mut self,
        tool: cam_model::Tool,
        project: bool,
    ) -> iced::Task<Message> {
        let Some(kind) = self.selector_operation().map(AppController::operation_kind) else {
            return iced::Task::none();
        };
        if !families_for(kind).contains(&ToolKindPick::of(tool.kind)) {
            self.status = "This cutter kind is not supported by the selected operation.".into();
            return iced::Task::none();
        }
        self.close_operation_tool_selector();
        if self.creating_operation() {
            if !project {
                if let Some(index) = self.library.tools.iter().position(|t| t == &tool) {
                    return self.update(Message::SetPendingLibraryTool(index));
                }
                return iced::Task::none();
            }
            if self.cad_pending_kind.is_some() {
                self.cad_tool_pick = None;
                self.cad_selected_project_tool = Some(tool);
            } else {
                let number = self.stage_operation_tool(tool);
                self.controller.set_pending_tool(number);
            }
            self.sync_creation_tool();
            self.refresh_operation_fields();
            self.seed_user_parameter_defaults(|_| true);
        } else {
            let number = self.stage_operation_tool(tool);
            self.edit_operation_draft(|op| {
                op.set_tool(number);
                op.clear_tool_preset();
            });
            self.refresh_operation_fields();
            self.auto_operation_preset();
            self.refresh_cutting_fields(None);
        }
        iced::Task::none()
    }

    /// Only cutters referenced by operations, across every setup. Prefer the active
    /// setup and its staged values when a physical cutter occurs more than once.
    pub(super) fn used_project_cutters(&self) -> Vec<cam_model::Tool> {
        let mut tools: Vec<cam_model::Tool> = Vec::new();
        let active = self.controller.inspection_setup();
        for setup in std::iter::once(active).chain(
            self.controller
                .setups()
                .iter()
                .filter(|s| s.id != active.id),
        ) {
            let numbers: BTreeSet<_> = setup.operations.iter().flat_map(Operation::tools).collect();
            for saved in setup.tools.iter().filter(|t| numbers.contains(&t.number)) {
                let tool = if setup.id == active.id {
                    self.operation_tools
                        .iter()
                        .find(|t| t.number == saved.number)
                        .unwrap_or(saved)
                } else {
                    saved
                };
                if !tools.iter().any(|t| t.identity() == tool.identity()) {
                    tools.push(tool.clone());
                }
            }
        }
        if let Some(op) = self.selector_operation() {
            for number in op.tools() {
                if let Some(tool) = self.operation_tools.iter().find(|t| t.number == number) {
                    if !tools.iter().any(|t| t.identity() == tool.identity()) {
                        tools.push(tool.clone());
                    }
                }
            }
        }
        tools.sort_by_key(|t| t.number);
        tools
    }

    pub(super) fn operation_tool_selector_overlay(&self) -> Option<Element<'_, Message>> {
        let selector = self.operation_tool_selector.as_ref()?;
        let widths = self.tool_table_widths;
        let mut header = row![].spacing(4);
        for &index in &self.tool_table_order {
            if self.tool_table_columns[index] {
                header = header.push(
                    container(text(workspace_view::TOOL_TABLE_LABELS[index]).size(12))
                        .width(widths[index])
                        .clip(true),
                );
            }
        }
        let mut project = column![container(header).padding([0, 7])].spacing(4);
        let tools = self.used_project_cutters();
        if tools.is_empty() {
            project = project.push(text("No cutters used in this project yet.").size(12));
        }
        for tool in tools {
            let supported = selector.supports(&tool);
            let selected = self
                .operation_preset_tool()
                .is_some_and(|current| current.identity() == tool.identity());
            let values = self.tool_table_row_values(&tool);
            let mut cells = row![].spacing(4);
            for &index in &self.tool_table_order {
                if self.tool_table_columns[index] {
                    cells = cells.push(
                        container(text(values[index].clone()).size(12))
                            .width(widths[index])
                            .clip(true),
                    );
                }
            }
            project = project.push(
                button(
                    container(cells)
                        .padding(7)
                        .style(move |_| container::Style {
                            background: selected.then_some(Background::Color(palette::SELECT_BG)),
                            ..Default::default()
                        }),
                )
                .padding(0)
                .style(inspector_secondary)
                .on_press_maybe(supported.then_some(Message::OperationProjectToolPicked(tool))),
            );
        }
        let project = scrollable(project.width(Length::Shrink))
            .direction(scrollable::Direction::Both {
                vertical: scrollable::Scrollbar::default(),
                horizontal: scrollable::Scrollbar::default(),
            })
            .height(Length::FillPortion(1))
            .width(Length::Fill);
        let content = column![
            row![
                text("Select cutter").size(18),
                Space::new().width(Length::Fill),
                inspector_button("Cancel").on_press(Message::CloseOperationToolSelector)
            ]
            .spacing(12)
            .align_y(Alignment::Center),
            text("Tools used in this project").size(14),
            project,
            popup_separator(),
            self.tools_table(),
        ]
        .spacing(10)
        .height((self.window.height - 100.0).clamp(240.0, 800.0));
        Some(
            container(
                container(content)
                    .padding(16)
                    .width((self.window.width - 48.0).clamp(300.0, 1250.0))
                    .style(popup_surface),
            )
            .width(Length::Fill)
            .height(Length::Fill)
            .center_x(Length::Fill)
            .center_y(Length::Fill)
            .into(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn app() -> App {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller
            .open_dxf(SAMPLE_DXF, "selector test")
            .unwrap();
        app.controller.new_operation(OpKind::Profile);
        app.setup_draft = None;
        app.library.tools = vec![
            crate::tool_library::default_tool(1, ToolKind::EndMill),
            crate::tool_library::default_tool(
                2,
                ToolKind::Drill {
                    point_angle_deg: 118.0,
                },
            ),
        ];
        app.refresh_fields();
        app
    }

    #[test]
    fn library_context_menu_targets_selected_library_and_rename_can_be_cancelled() {
        let mut app = app();
        let _ = app.update(Message::NewToolLibrary);
        let _ = app.update(Message::SelectToolLibrary("Default tools".into()));
        let _ = app.update(Message::OpenLibraryContextMenu("New library".into()));
        assert_eq!(app.libraries.active, "New library");
        assert!(app.library_context_menu.is_some());
        let _ = app.update(Message::BeginLibraryRename);
        assert!(app.library_context_menu.is_none());
        assert!(app.library_renaming);
        let _ = app.update(Message::LibraryNameChanged("changed".into()));
        let _ = app.update(Message::CancelLibraryRename);
        assert_eq!(app.library_name_edit, "New library");
        assert!(!app.library_renaming);
    }

    #[test]
    fn libraries_can_be_created_renamed_and_deleted() {
        let mut app = app();
        let initial = app.libraries.active.clone();
        let _ = app.update(Message::NewToolLibrary);
        assert_eq!(app.libraries.active, "New library");
        let _ = app.update(Message::LibraryNameChanged("Shop tools".into()));
        let _ = app.update(Message::RenameToolLibrary);
        assert_eq!(app.libraries.active, "Shop tools");
        let _ = app.update(Message::DeleteToolLibraryConfirmed((
            "Shop tools".into(),
            false,
        )));
        assert!(app.libraries.libraries.contains_key("Shop tools"));
        let _ = app.update(Message::DeleteToolLibraryConfirmed((
            "Shop tools".into(),
            true,
        )));
        assert_eq!(app.libraries.active, initial);
        assert!(!app.libraries.libraries.contains_key("Shop tools"));
        let _ = app.update(Message::DeleteToolLibraryConfirmed((initial, true)));
        assert_eq!(app.libraries.active, "Default tools");
        assert_eq!(app.libraries.libraries.len(), 1);
        assert_eq!(app.library, ToolLibrary::defaults());
    }

    #[test]
    fn selector_switches_between_libraries_without_losing_tools() {
        let mut app = app();
        let original = app.library.clone();
        let mut other = ToolLibrary::default();
        other
            .tools
            .push(crate::tool_library::default_tool(7, ToolKind::EndMill));
        app.libraries
            .libraries
            .insert("Other".into(), other.clone());
        let original_name = app.libraries.active.clone();
        let _ = app.update(Message::OpenOperationToolSelector);
        let _ = app.update(Message::SelectToolLibrary("Other".into()));
        assert_eq!(app.library, other);
        let _ = app.update(Message::SelectToolLibrary(original_name));
        assert_eq!(app.library, original);
    }

    #[test]
    fn selector_presets_family_filters_and_restores_tools_view_on_cancel_and_pick() {
        let mut app = app();
        app.tool_table_search = "shop search".into();
        app.tool_table_filters[3] = Some(BTreeSet::from(["Maker".into()]));
        let previous_filters = app.tool_table_filters.clone();
        let preferences = app.settings.tool_table.clone();
        let _ = app.update(Message::OpenOperationToolSelector);
        assert!(app.tool_table_search.is_empty());
        assert_eq!(
            app.tool_table_filters[2],
            Some(
                recommended_families(OpKind::Profile)
                    .iter()
                    .map(|f| f.to_kind().to_string())
                    .collect()
            )
        );
        assert!(app.tool_table_filters[3].is_none());
        let _ = app.update(Message::ToolTableSearch("end mill".into()));
        assert_eq!(app.settings.tool_table.search, preferences.search);
        let _ = app.update(Message::CadEscape);
        assert!(app.operation_tool_selector.is_none());
        assert_eq!(app.tool_table_search, "shop search");
        assert_eq!(app.tool_table_filters, previous_filters);
        let _ = app.update(Message::OpenOperationToolSelector);
        let _ = app.update(Message::OperationToolPicked(0));
        assert!(app.operation_tool_selector.is_none());
        assert_eq!(app.tool_table_filters, previous_filters);
        assert_eq!(app.tool_table_search, "shop search");
        assert_eq!(tool_button_label(None), "select...");
    }

    #[test]
    fn used_project_tools_span_setups_and_ignore_unused_and_duplicate_cutters() {
        let mut app = app();
        let active = app.controller.active_setup_id();
        let op = app.controller.inspection_setup().operations[0].clone();
        let used = app
            .controller
            .inspection_setup()
            .tools
            .iter()
            .find(|t| t.number == op.tool())
            .unwrap()
            .clone();
        let unused = cam_model::Tool {
            number: 99,
            diameter: 99.0,
            ..used.clone()
        };
        app.controller.edit_setup(|s| s.tools.push(unused));
        app.controller.add_setup();
        let other = cam_model::Tool {
            number: 8,
            diameter: 12.25,
            ..used.clone()
        };
        let mut other_op = op.clone();
        other_op.set_tool(8);
        app.controller.edit_setup(|s| {
            s.tools = vec![used.clone(), other.clone()];
            s.operations = vec![op.clone(), other_op];
        });
        app.controller.activate_setup(active).unwrap();
        let tools = app.used_project_cutters();
        assert_eq!(tools.len(), 2);
        assert!(tools.iter().any(|t| t.identity() == used.identity()));
        assert!(tools.iter().any(|t| t.identity() == other.identity()));
    }

    #[test]
    fn project_tool_selection_stages_embedded_values_without_committing_the_project() {
        let mut app = app();
        let before = app.controller.inspection_setup().clone();
        let id = before.operations[0].id();
        app.controller.select(Selection::Operation(id));
        let tool = cam_model::Tool {
            number: 1,
            diameter: 9.25,
            description: "Project finishing cutter".into(),
            ..app.library.tools[0].clone()
        };
        let _ = app.update(Message::OpenOperationToolSelector);
        let _ = app.update(Message::OperationProjectToolPicked(tool.clone()));
        assert!(app.operation_tool_selector.is_none());
        assert_eq!(
            app.operation_preset_tool().unwrap().identity(),
            tool.identity()
        );
        assert!(tool_button_label(app.operation_preset_tool()).contains("Project finishing cutter"));
        assert_eq!(app.controller.inspection_setup(), &before);
        let _ = app.update(Message::CancelOperationDraft);
        assert_eq!(app.controller.inspection_setup(), &before);
        assert_ne!(app.operation_preset_tool().unwrap().diameter, 9.25);
    }

    #[test]
    fn selector_rejects_incompatible_tools_and_blocks_background_shortcuts() {
        let mut app = app();
        app.controller.new_operation(OpKind::Carve);
        assert_eq!(
            AppController::operation_kind(app.selector_operation().unwrap()),
            OpKind::Carve
        );
        let before = app.controller.inspection_setup().clone();
        let _ = app.update(Message::OpenOperationToolSelector);
        let _ = app.update(Message::DeleteOp);
        let _ = app.update(Message::OperationToolPicked(1));
        assert!(app.operation_tool_selector.is_some());
        assert_eq!(app.controller.inspection_setup(), &before);
        let _ = app.update(Message::OpenToolTableMenu(2, false));
        let _ = app.update(Message::CadEscape);
        assert!(app.operation_tool_selector.is_some());
        assert!(app.tool_table_menu.is_none());
        let _ = app.update(Message::CadEscape);
        assert!(app.operation_tool_selector.is_none());
    }

    #[test]
    fn new_planar_and_cad_operations_accept_project_cutters_without_library_entries() {
        for cad in [false, true] {
            let mut app = app();
            app.controller.select(Selection::Setup);
            let tool = cam_model::Tool {
                number: 17,
                diameter: 7.25,
                nominal_rpm: 16000.0,
                ..app.library.tools[0].clone()
            };
            if cad {
                app.cad_pending_kind = Some(OpKind::Profile);
                app.cad_tool_pick = Some(1);
                app.seed_creation_editor(OpKind::Profile);
            } else {
                let _ = app.update(Message::BeginOp(OpKind::Profile));
            }
            assert!(app.creating_operation());
            let _ = app.update(Message::OpenOperationToolSelector);
            let _ = app.update(Message::OperationProjectToolPicked(tool.clone()));
            assert_eq!(app.wizard_tool().unwrap().identity(), tool.identity());
            assert_eq!(app.operation_preset_tool().unwrap().nominal_rpm, 16000.0);
            let _ = app.update(Message::OpenOperationToolSelector);
            let _ = app.update(Message::OperationToolPicked(0));
            assert!(app.cad_selected_project_tool.is_none());
            assert_eq!(
                app.wizard_tool().unwrap().identity(),
                app.library.tools[0].identity()
            );
        }
    }

    #[test]
    #[ignore = "requires a graphics adapter; exports a tools workspace preview to /tmp"]
    fn render_tools_workspace_libraries() {
        use iced::advanced::renderer::Headless;
        let (mut app, _) = App::new();
        app.window = iced::Size::new(1200.0, 850.0);
        app.switch_workspace(workspace_view::Workspace::Tools);
        let mut renderer = iced::futures::executor::block_on(<iced::Renderer as Headless>::new(
            iced::Font::DEFAULT,
            iced::Pixels(12.0),
            None,
        ))
        .expect("graphics adapter");
        let mut element = app.workspace_main();
        let mut tree = iced::advanced::widget::Tree::new(&element);
        let node = element.as_widget_mut().layout(
            &mut tree,
            &renderer,
            &iced::advanced::layout::Limits::new(iced::Size::ZERO, app.window),
        );
        let mut messages = Vec::new();
        element.as_widget_mut().update(
            &mut tree,
            &iced::Event::Window(iced::window::Event::RedrawRequested(
                std::time::Instant::now(),
            )),
            iced::advanced::Layout::new(&node),
            iced::mouse::Cursor::Unavailable,
            &renderer,
            &mut iced::advanced::clipboard::Null,
            &mut iced::advanced::Shell::new(&mut messages),
            &iced::Rectangle::with_size(app.window),
        );
        element.as_widget().draw(
            &tree,
            &mut renderer,
            &iced::Theme::Dark,
            &iced::advanced::renderer::Style {
                text_color: iced::Theme::Dark.palette().text,
            },
            iced::advanced::Layout::new(&node),
            iced::mouse::Cursor::Unavailable,
            &iced::Rectangle::with_size(app.window),
        );
        let pixels =
            Headless::screenshot(&mut renderer, iced::Size::new(1200, 850), 1.0, Color::WHITE);
        let mut ppm = b"P6\n1200 850\n255\n".to_vec();
        for pixel in pixels.as_chunks::<4>().0 {
            ppm.extend_from_slice(&pixel[..3]);
        }
        std::fs::write("/tmp/opencam-tools-workspace.ppm", ppm).unwrap();
    }

    #[test]
    #[ignore = "requires a graphics adapter; exports a selector preview to /tmp"]
    fn render_operation_tool_selector() {
        use iced::advanced::renderer::Headless;
        let mut app = app();
        app.window = iced::Size::new(1200.0, 850.0);
        app.open_operation_tool_selector();
        let mut renderer = iced::futures::executor::block_on(<iced::Renderer as Headless>::new(
            iced::Font::DEFAULT,
            iced::Pixels(12.0),
            None,
        ))
        .expect("graphics adapter");
        let mut element = app.operation_tool_selector_overlay().unwrap();
        let mut tree = iced::advanced::widget::Tree::new(&element);
        let node = element.as_widget_mut().layout(
            &mut tree,
            &renderer,
            &iced::advanced::layout::Limits::new(iced::Size::ZERO, app.window),
        );
        let mut messages = Vec::new();
        element.as_widget_mut().update(
            &mut tree,
            &iced::Event::Window(iced::window::Event::RedrawRequested(
                std::time::Instant::now(),
            )),
            iced::advanced::Layout::new(&node),
            iced::mouse::Cursor::Unavailable,
            &renderer,
            &mut iced::advanced::clipboard::Null,
            &mut iced::advanced::Shell::new(&mut messages),
            &iced::Rectangle::with_size(app.window),
        );
        element.as_widget().draw(
            &tree,
            &mut renderer,
            &iced::Theme::Dark,
            &iced::advanced::renderer::Style {
                text_color: iced::Theme::Dark.palette().text,
            },
            iced::advanced::Layout::new(&node),
            iced::mouse::Cursor::Unavailable,
            &iced::Rectangle::with_size(app.window),
        );
        let pixels =
            Headless::screenshot(&mut renderer, iced::Size::new(1200, 850), 1.0, Color::WHITE);
        let mut ppm = b"P6\n1200 850\n255\n".to_vec();
        for pixel in pixels.as_chunks::<4>().0 {
            ppm.extend_from_slice(&pixel[..3]);
        }
        std::fs::write("/tmp/opencam-operation-tool-selector.ppm", ppm).unwrap();
    }
}
