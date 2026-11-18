//! The native Preferences window and its uncommitted settings.
use super::{
    display_shortcut, inspector_button, inspector_footer, inspector_picker, inspector_primary,
    inspector_secondary, inspector_tab, mcp, palette, shortcut_binding, shortcut_value, App,
    Message, INSPECTOR_MUTED, INSPECTOR_TEXT, INSPECTOR_TEXT_SIZE, SHORTCUTS,
};
use crate::Settings;
use iced::widget::{
    button, checkbox, column, container, pick_list, row, scrollable, slider, text, Space,
};
use iced::{Alignment, Background, Color, Element, Length, Theme};
use std::sync::LazyLock;

pub(super) fn theme() -> Theme {
    static THEME: LazyLock<Theme> = LazyLock::new(|| {
        Theme::custom(
            "Open CAM Studio".to_owned(),
            iced::theme::Palette {
                background: palette::PANE_BG,
                text: INSPECTOR_TEXT,
                primary: palette::ACCENT_BLUE,
                ..Theme::Dark.palette()
            },
        )
    });
    THEME.clone()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Tab {
    General,
    Snapping,
    Stock,
    Simulation,
    Shortcuts,
    Mcp,
}

impl Tab {
    const ALL: [(Self, &'static str); 6] = [
        (Self::General, "General"),
        (Self::Snapping, "Snapping"),
        (Self::Stock, "Stock"),
        (Self::Simulation, "Simulation"),
        (Self::Shortcuts, "Shortcuts"),
        (Self::Mcp, "MCP"),
    ];
}

#[derive(Clone, Debug)]
pub(super) enum Edit {
    SelectTab(Tab),
    NumberDecimals(u8),
    Pickbox(f32),
    MarkerScale(f32),
    OriginMarker(f32),
    ToggleStockPalette,
    StockColor([f32; 3]),
    StockTransparency(f32),
    SimulationAccuracy(f32),
    LocalMcp(bool),
    CopyMcpConfiguration,
    CaptureShortcut(&'static str),
    ClearShortcut(&'static str),
    ShortcutKey(Option<char>, iced::keyboard::Modifiers),
    RestoreDefaults,
    Escape,
    Cancel,
    Accept,
}

pub(super) struct Preferences {
    pub(super) id: iced::window::Id,
    tab: Tab,
    initial: Settings,
    draft: Settings,
    reset: bool,
    show_stock_palette: bool,
    shortcut_capture: Option<&'static str>,
    shortcut_feedback: String,
    save_error: Option<String>,
}

impl Preferences {
    fn new(id: iced::window::Id, settings: &Settings) -> Self {
        Self {
            id,
            tab: Tab::General,
            initial: settings.clone(),
            draft: settings.clone(),
            reset: false,
            show_stock_palette: false,
            shortcut_capture: None,
            shortcut_feedback: String::new(),
            save_error: None,
        }
    }

    fn edit(&mut self, edit: Edit) {
        match edit {
            Edit::SelectTab(tab) => {
                self.tab = tab;
                self.shortcut_capture = None;
                self.shortcut_feedback.clear();
                self.show_stock_palette = false;
            }
            Edit::NumberDecimals(digits) => self.draft.number_decimals = digits.min(8),
            Edit::Pickbox(value) => {
                let (lo, hi) = crate::PICKBOX_RANGE;
                self.draft.snapping.pickbox_px = value.clamp(lo, hi);
            }
            Edit::MarkerScale(value) => {
                let (lo, hi) = crate::MARKER_SCALE_RANGE;
                self.draft.snapping.marker_scale = value.clamp(lo, hi);
            }
            Edit::OriginMarker(value) => {
                let (lo, hi) = crate::ORIGIN_MARKER_RANGE;
                self.draft.view.origin_marker_scale = value.clamp(lo, hi);
            }
            Edit::ToggleStockPalette => self.show_stock_palette = !self.show_stock_palette,
            Edit::StockColor(color) => {
                self.draft.view.stock_color = color;
                self.show_stock_palette = false;
            }
            Edit::StockTransparency(value) => {
                self.draft.view.stock_transparency = value.clamp(0.0, 1.0);
            }
            Edit::SimulationAccuracy(value) => {
                let (lo, hi) = crate::SIMULATION_ACCURACY_RANGE;
                self.draft.simulation.accuracy = value.clamp(lo, hi);
            }
            Edit::LocalMcp(enabled) => self.draft.mcp_enabled = enabled,
            Edit::CaptureShortcut(id) => {
                self.shortcut_capture = Some(id);
                self.shortcut_feedback = "Press a shortcut, or Escape to cancel.".into();
            }
            Edit::ClearShortcut(id) => {
                self.draft.shortcuts.insert(id.into(), String::new());
                self.shortcut_capture = None;
                self.shortcut_feedback = "Shortcut cleared.".into();
            }
            Edit::ShortcutKey(key, modifiers) => self.capture_key(key, modifiers),
            Edit::RestoreDefaults => {
                self.draft = Settings::default();
                self.reset = true;
                self.shortcut_capture = None;
                self.shortcut_feedback.clear();
                self.show_stock_palette = false;
                self.save_error = None;
            }
            Edit::Escape => {
                self.shortcut_capture = None;
                self.shortcut_feedback = "Shortcut change cancelled.".into();
            }
            Edit::CopyMcpConfiguration | Edit::Cancel | Edit::Accept => unreachable!(),
        }
    }

    fn capture_key(&mut self, key: Option<char>, modifiers: iced::keyboard::Modifiers) {
        let Some(id) = self.shortcut_capture else {
            return;
        };
        let Some(binding) = shortcut_binding(key, modifiers) else {
            self.shortcut_feedback =
                "Use Command/Ctrl with a letter or number, optionally with Shift.".into();
            return;
        };
        if ["Cmd+A", "Cmd+C", "Cmd+V", "Cmd+X"].contains(&binding.as_str()) {
            self.shortcut_feedback = "That shortcut is reserved for text editing.".into();
        } else if let Some((_, label, _)) = SHORTCUTS
            .iter()
            .find(|(other, _, _)| *other != id && shortcut_value(&self.draft, other) == binding)
        {
            self.shortcut_feedback =
                format!("Already assigned to {label}. Choose another shortcut.");
        } else {
            self.draft.shortcuts.insert(id.into(), binding);
            self.shortcut_capture = None;
            self.shortcut_feedback.clear();
        }
    }

    fn settings_to_save(&self, live: &Settings) -> Settings {
        if self.reset {
            return self.draft.clone();
        }

        // The workspace stays usable with Preferences open. Merge edits into the
        // latest settings so accepting does not overwrite changes made there.
        let mut settings = live.clone();
        macro_rules! merge {
            ($($field:ident).+) => {
                if self.draft.$($field).+ != self.initial.$($field).+ {
                    settings.$($field).+.clone_from(&self.draft.$($field).+);
                }
            };
        }
        merge!(number_decimals);
        merge!(snapping.pickbox_px);
        merge!(snapping.marker_scale);
        merge!(view.origin_marker_scale);
        merge!(view.stock_color);
        merge!(view.stock_transparency);
        merge!(simulation.accuracy);
        merge!(mcp_enabled);
        for (id, binding) in &self.draft.shortcuts {
            if self.initial.shortcuts.get(id) != Some(binding) {
                settings.shortcuts.insert(id.clone(), binding.clone());
            }
        }
        settings
    }

    pub(super) fn view<'a>(&'a self, app: &'a App) -> Element<'a, Edit> {
        let tabs = row(Tab::ALL.map(|(tab, label)| {
            let active = self.tab == tab;
            inspector_button(label)
                .width(Length::Fill)
                .style(move |theme, status| inspector_tab(theme, status, active))
                .on_press(Edit::SelectTab(tab))
                .into()
        }))
        .spacing(4);

        let content = match self.tab {
            Tab::General => self.general(),
            Tab::Snapping => self.snapping(),
            Tab::Stock => self.stock(),
            Tab::Simulation => self.simulation(),
            Tab::Shortcuts => self.shortcuts(),
            Tab::Mcp => self.mcp(app),
        };
        let body = iced::widget::keyed_column![(
            self.tab,
            scrollable(container(content).padding([8, 14]).width(Length::Fill))
                .height(Length::Fill)
        )]
        .height(Length::Fill);

        let mut footer = column![].spacing(8);
        if self.reset {
            footer = footer.push(note("Defaults will be restored when you click OK.".into()));
        }
        if let Some(error) = &self.save_error {
            footer = footer.push(text(error).size(INSPECTOR_TEXT_SIZE).color(palette::ERROR));
        }
        footer = footer.push(
            row![
                inspector_button("Restore defaults").on_press(Edit::RestoreDefaults),
                Space::new().width(Length::Fill),
                inspector_button("Cancel").width(82).on_press(Edit::Cancel),
                inspector_button("OK")
                    .width(82)
                    .style(inspector_primary)
                    .on_press(Edit::Accept),
            ]
            .spacing(8)
            .align_y(Alignment::Center),
        );

        column![
            container(tabs).padding([12, 14]).width(Length::Fill),
            body,
            inspector_footer(footer),
        ]
        .into()
    }

    fn general(&self) -> Element<'_, Edit> {
        row![
            text("Decimal places").size(INSPECTOR_TEXT_SIZE).width(150),
            pick_list(
                (0u8..=8).collect::<Vec<_>>(),
                Some(self.draft.number_decimals),
                Edit::NumberDecimals,
            )
            .text_size(INSPECTOR_TEXT_SIZE)
            .padding([7, 9])
            .style(inspector_picker)
            .width(100),
        ]
        .spacing(10)
        .align_y(Alignment::Center)
        .into()
    }

    fn snapping(&self) -> Element<'_, Edit> {
        column![
            slider_row(
                "Pickbox size",
                crate::PICKBOX_RANGE,
                self.draft.snapping.pickbox_px,
                1.0,
                format!("{:.0} px", self.draft.snapping.pickbox_px),
                Edit::Pickbox,
            ),
            // Shown, not settable. The catch distance is a fixed multiple of the
            // pickbox on purpose: two absolute knobs would let it be set *smaller*
            // than the box that feeds it. Showing the derived number keeps the
            // relationship visible instead of merely documented.
            note(format!(
                "Object snaps catch within {:.0} px — {:.1}× the pickbox.",
                self.draft.snap_catch_px(),
                crate::SNAP_CATCH_MULTIPLE,
            )),
            slider_row(
                "Snap marker size",
                crate::MARKER_SCALE_RANGE,
                self.draft.snapping.marker_scale,
                0.1,
                format!("{:.1}×", self.draft.snapping.marker_scale),
                Edit::MarkerScale,
            ),
            slider_row(
                "Origin marker size",
                crate::ORIGIN_MARKER_RANGE,
                self.draft.view.origin_marker_scale,
                0.1,
                format!("{:.1}×", self.draft.view.origin_marker_scale),
                Edit::OriginMarker,
            ),
        ]
        .spacing(6)
        .into()
    }

    fn stock(&self) -> Element<'_, Edit> {
        let swatch =
            color_swatch(self.draft.view.stock_color, 42, 26).on_press(Edit::ToggleStockPalette);
        let mut stock = column![
            row![
                text("Stock color").size(INSPECTOR_TEXT_SIZE).width(150),
                swatch
            ]
            .spacing(12)
            .align_y(Alignment::Center),
            slider_row(
                "Transparency",
                (0.0, 1.0),
                self.draft.view.stock_transparency,
                0.01,
                format!("{:.0}%", self.draft.view.stock_transparency * 100.0),
                Edit::StockTransparency,
            ),
            note("Used when transparent stock is enabled.".into()),
        ]
        .spacing(6);
        if self.show_stock_palette {
            // A grid of hue, saturation and brightness choices, plus neutral grays.
            for level in 0..6 {
                let mut colors = row![].spacing(3);
                for hue in 0..12 {
                    let h = hue as f32 / 2.0;
                    let c = if level < 3 {
                        1.0 - level as f32 * 0.25
                    } else {
                        1.0
                    };
                    let value = if level < 3 {
                        1.0
                    } else {
                        1.0 - (level - 2) as f32 * 0.22
                    };
                    let x = c * (1.0 - ((h % 2.0) - 1.0).abs());
                    let (rr, gg, bb) = match hue / 2 {
                        0 => (c, x, 0.0),
                        1 => (x, c, 0.0),
                        2 => (0.0, c, x),
                        3 => (0.0, x, c),
                        4 => (x, 0.0, c),
                        _ => (c, 0.0, x),
                    };
                    let m = 1.0 - c;
                    let color = [(rr + m) * value, (gg + m) * value, (bb + m) * value];
                    colors =
                        colors.push(color_swatch(color, 26, 20).on_press(Edit::StockColor(color)));
                }
                stock = stock.push(colors);
            }
            let mut grays = row![].spacing(3);
            for n in 0..12 {
                let v = n as f32 / 11.0;
                let color = [v, v, v];
                grays = grays.push(color_swatch(color, 26, 20).on_press(Edit::StockColor(color)));
            }
            stock = stock.push(grays);
        }

        stock.into()
    }

    fn simulation(&self) -> Element<'_, Edit> {
        column![
            slider_row(
                "Accuracy",
                crate::SIMULATION_ACCURACY_RANGE,
                self.draft.simulation.accuracy,
                0.25,
                format!("{:.2}×", self.draft.simulation.accuracy),
                Edit::SimulationAccuracy,
            ),
            note("Higher accuracy resolves finer cuts and uses more time and memory. Applies to stock cut simulation and playback.".into()),
        ].spacing(6).into()
    }

    fn shortcuts(&self) -> Element<'_, Edit> {
        let mut shortcuts = column![note("Select Change, then press Command/Ctrl and a letter or number (Shift is optional). Escape cancels.".into())].spacing(5);
        for &(id, label, _) in SHORTCUTS {
            let shown = if self.shortcut_capture == Some(id) {
                "Press shortcut…".to_string()
            } else {
                display_shortcut(shortcut_value(&self.draft, id))
            };
            shortcuts = shortcuts.push(
                row![
                    text(label).size(INSPECTOR_TEXT_SIZE).width(160),
                    text(shown).size(INSPECTOR_TEXT_SIZE).width(150),
                    inspector_button("Change").on_press(Edit::CaptureShortcut(id)),
                    inspector_button("Clear").on_press(Edit::ClearShortcut(id)),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
        }
        shortcuts
            .push(
                text(&self.shortcut_feedback)
                    .size(11)
                    .color(INSPECTOR_MUTED),
            )
            .into()
    }

    fn mcp<'a>(&'a self, app: &'a App) -> Element<'a, Edit> {
        let mut mcp_settings = column![].spacing(6);
        if matches!(&app.mcp, Some(mcp::DesktopMcp::Stdio(_))) {
            mcp_settings = mcp_settings.push(note(
                "Connected through the client that started this window.".into(),
            ));
        } else {
            mcp_settings = mcp_settings
                .push(
                    checkbox(self.draft.mcp_enabled)
                        .label("Enable MCP")
                        .text_size(INSPECTOR_TEXT_SIZE)
                        .on_toggle(Edit::LocalMcp),
                )
                .push(note(
                    "Allow a local MCP client to control Open CAM Studio whenever the app is open."
                        .into(),
                ));
            if let Some(mcp::DesktopMcp::Local(server)) = &app.mcp {
                mcp_settings = mcp_settings.push(
                    row![
                        note(format!("Listening on 127.0.0.1:{}", server.port())),
                        inspector_button("Copy client configuration")
                            .on_press(Edit::CopyMcpConfiguration),
                    ]
                    .spacing(12)
                    .align_y(Alignment::Center),
                );
            }
        }
        if let Some(feedback) = &app.mcp_feedback {
            mcp_settings = mcp_settings.push(text(feedback).size(11).color(INSPECTOR_MUTED));
        }

        mcp_settings.into()
    }
}

fn color_swatch<'a>([r, g, b]: [f32; 3], width: u32, height: u32) -> button::Button<'a, Edit> {
    button(Space::new().width(width).height(height))
        .padding(0)
        .style(move |theme, status| {
            let mut style = inspector_secondary(theme, status);
            style.background = Some(Background::Color(Color::from_rgb(r, g, b)));
            if matches!(status, button::Status::Hovered | button::Status::Pressed) {
                style.border.color = INSPECTOR_TEXT;
            }
            style
        })
}

fn note<'a>(t: String) -> iced::widget::Text<'a> {
    text(t)
        .size(INSPECTOR_TEXT_SIZE)
        .color(INSPECTOR_MUTED)
        .wrapping(iced::widget::text::Wrapping::Word)
}

fn slider_row<'a>(
    label: &'static str,
    range: (f32, f32),
    value: f32,
    step: f32,
    shown: String,
    msg: fn(f32) -> Edit,
) -> Element<'a, Edit> {
    row![
        text(label).size(INSPECTOR_TEXT_SIZE).width(150),
        slider(range.0..=range.1, value, msg).step(step).width(190),
        text(shown).size(INSPECTOR_TEXT_SIZE).width(64),
    ]
    .spacing(10)
    .align_y(Alignment::Center)
    .into()
}

pub(super) fn keyboard_event(event: iced::Event, status: iced::event::Status) -> Option<Message> {
    use iced::keyboard::{key::Named, Event::KeyPressed, Key};

    let edit = match event {
        iced::Event::Keyboard(KeyPressed {
            key: Key::Named(Named::Escape),
            repeat: false,
            ..
        }) if status == iced::event::Status::Ignored => Edit::Escape,
        iced::Event::Keyboard(KeyPressed {
            key: Key::Character(key),
            modifiers,
            repeat: false,
            ..
        }) => Edit::ShortcutKey(key.chars().next(), modifiers),
        _ => return None,
    };
    Some(Message::Preferences(edit))
}

impl App {
    pub(super) fn open_preferences(&mut self) -> iced::Task<Message> {
        if let Some(prefs) = &self.preferences {
            return iced::window::gain_focus(prefs.id);
        }

        let (id, open) = iced::window::open(iced::window::Settings {
            size: iced::Size::new(570.0, 490.0),
            min_size: Some(iced::Size::new(520.0, 300.0)),
            position: iced::window::Position::Centered,
            exit_on_close_request: false,
            ..Default::default()
        });
        self.preferences = Some(Preferences::new(id, &self.settings));
        open.discard()
    }

    pub(super) fn is_preferences_window(&self, id: iced::window::Id) -> bool {
        self.preferences
            .as_ref()
            .is_some_and(|prefs| prefs.id == id)
    }

    pub(super) fn close_preferences(&mut self) -> iced::Task<Message> {
        self.preferences
            .take()
            .map_or_else(iced::Task::none, |prefs| iced::window::close(prefs.id))
    }

    pub(super) fn edit_preferences(&mut self, edit: Edit) -> iced::Task<Message> {
        let Some(prefs) = &mut self.preferences else {
            return iced::Task::none();
        };
        match edit {
            Edit::Cancel => return self.close_preferences(),
            Edit::Escape if prefs.shortcut_capture.is_none() => return self.close_preferences(),
            Edit::Accept => return self.accept_preferences(Self::save_preferences),
            Edit::CopyMcpConfiguration => return self.copy_local_mcp_configuration(),
            _ => prefs.edit(edit),
        }
        iced::Task::none()
    }

    fn accept_preferences(
        &mut self,
        save: impl FnOnce(&mut Self, Settings) -> std::io::Result<()>,
    ) -> iced::Task<Message> {
        let Some(prefs) = &self.preferences else {
            return iced::Task::none();
        };
        let settings = prefs.settings_to_save(&self.settings);
        let reset = prefs.reset;
        let decimals_changed = settings.number_decimals != self.settings.number_decimals;
        let accuracy_changed = settings.simulation.accuracy != self.settings.simulation.accuracy;
        let simulation_changed = settings.simulation != self.settings.simulation
            || settings.view.stock_color != self.settings.view.stock_color;
        if settings != self.settings {
            if let Err(error) = save(self, settings) {
                if let Some(prefs) = &mut self.preferences {
                    prefs.save_error = Some(format!("Could not save preferences: {error}"));
                }
                return iced::Task::none();
            }
        }
        if decimals_changed || reset {
            self.edited_preset_input = None;
            self.edited_tool_input = None;
        }
        if reset {
            self.restore_tool_table_preferences();
            self.sync_workspace_preferences();
            self.restore_pane_preferences();
            self.apply_fixed_layout();
        }
        if (accuracy_changed || reset)
            && self
                .controller
                .set_simulation_accuracy(self.settings.simulation.accuracy)
        {
            self.rerun();
        }
        if simulation_changed {
            if let Some(player) = &mut self.simulation {
                player.configure(&self.settings.simulation, self.settings.view.stock_color);
            }
        }
        if reset {
            self.status = "Preferences restored to defaults.".into();
        }
        self.close_preferences()
    }

    pub(super) fn sync_workspace_preferences(&mut self) {
        let settings = &self.settings;
        self.show_stock = settings.view.show_stock;
        self.show_gizmo = settings.view.show_gizmo;
        self.show_origin = settings.view.show_origin;
        self.show_envelope = settings.view.show_envelope;
        self.tooltips = settings.view.tooltips;
        self.gizmo_size = settings.view.gizmo_size;
        self.snaps = settings.snapping.default_snaps.clone();
        self.project_px = settings.panes.project_px;
        self.inspector_px = settings.panes.inspector_px;
        self.output_px = settings.panes.output_px;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::tests::Scratch;

    #[test]
    fn cancel_and_window_close_discard_the_draft() {
        let (mut app, _) = App::new();
        let original = app.settings.clone();
        let _ = app.update(Message::ShowPrefs);
        let id = app.preferences.as_ref().unwrap().id;
        let _ = app.update(Message::Preferences(Edit::NumberDecimals(8)));
        let _ = app.update(Message::Preferences(Edit::SelectTab(Tab::Stock)));
        let _ = app.update(Message::ShowPrefs);
        let prefs = app.preferences.as_ref().unwrap();
        assert_eq!(prefs.id, id);
        assert_eq!(prefs.draft.number_decimals, 8);
        assert_eq!(prefs.tab, Tab::Stock);
        assert_eq!(app.settings, original);
        let _ = app.update(Message::Preferences(Edit::Cancel));
        assert!(app.preferences.is_none());
        assert_eq!(app.settings, original);

        let _ = app.update(Message::ShowPrefs);
        let id = app.preferences.as_ref().unwrap().id;
        let _ = app.update(Message::Preferences(Edit::NumberDecimals(7)));
        let _ = app.update(Message::CloseRequested(id));
        assert!(app.preferences.is_none());
        assert_eq!(app.settings, original);
    }

    #[test]
    fn accepting_persists_edits_across_tabs_and_preserves_live_workspace_changes() {
        let (mut app, _) = App::new();
        let scratch = Scratch::new("accept-preferences");
        let path = scratch.file("settings.json");
        let _ = app.update(Message::ShowPrefs);
        let _ = app.update(Message::Preferences(Edit::NumberDecimals(6)));
        let _ = app.update(Message::Preferences(Edit::SelectTab(Tab::Snapping)));
        let _ = app.update(Message::Preferences(Edit::Pickbox(18.0)));
        let _ = app.update(Message::Preferences(Edit::SelectTab(Tab::Shortcuts)));
        let _ = app.update(Message::Preferences(Edit::ClearShortcut(SHORTCUTS[0].0)));
        app.settings.view.tooltips = false;
        app.settings.panes.project_px = 360.0;
        let _ = app.accept_preferences(|app, settings| app.save_preferences_to(settings, &path));
        assert!(app.preferences.is_none());
        let (saved, outcome) = crate::load_settings_from(&path);
        assert_eq!(outcome, crate::settings::LoadOutcome::Loaded);
        assert_eq!(saved, app.settings);
        assert_eq!(saved.number_decimals, 6);
        assert_eq!(saved.snapping.pickbox_px, 18.0);
        assert_eq!(saved.shortcuts[SHORTCUTS[0].0], "");
        assert!(!saved.view.tooltips);
        assert_eq!(saved.panes.project_px, 360.0);
    }

    #[test]
    fn failed_save_keeps_the_reset_pending_and_allows_retry() {
        let (mut app, _) = App::new();
        let scratch = Scratch::new("reset-preferences");
        app.settings.number_decimals = 6;
        let _ = app.update(Message::ShowPrefs);
        let _ = app.update(Message::Preferences(Edit::RestoreDefaults));
        assert_eq!(app.settings.number_decimals, 6);

        let _ =
            app.accept_preferences(|app, settings| app.save_preferences_to(settings, &scratch.0));
        assert_eq!(app.settings.number_decimals, 6);
        let prefs = app.preferences.as_ref().unwrap();
        assert!(prefs.save_error.is_some());
        assert_eq!(prefs.draft, Settings::default());

        let path = scratch.file("settings.json");
        let _ = app.accept_preferences(|app, settings| app.save_preferences_to(settings, &path));
        assert_eq!(app.settings, Settings::default());
        assert!(app.preferences.is_none());
    }
}
