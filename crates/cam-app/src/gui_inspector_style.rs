//! Shared visual language for inspector forms, navigation and actions.
//! Reuse these helpers for inspector controls, spacing, typography and optional sections.
use super::*;
use iced::widget::column;

pub(super) const INSPECTOR_TEXT: Color = Color::from_rgb(0.86, 0.88, 0.91);
pub(super) const INSPECTOR_MUTED: Color = Color::from_rgb(0.61, 0.65, 0.71);
const SURFACE: Color = Color::from_rgb(0.12, 0.14, 0.17);
const OUTLINE: Color = Color::from_rgb(0.25, 0.28, 0.32);
pub(super) const INSPECTOR_TEXT_SIZE: f32 = 12.0;
pub(super) const INSPECTOR_SECTION_FONT: iced::Font = iced::Font {
    weight: iced::font::Weight::Bold,
    ..iced::Font::DEFAULT
};

/// Shared popup treatment for reference selectors and project context menus.
pub(super) fn popup_item_style(
    _: &iced::Theme,
    status: button::Status,
    active: bool,
) -> button::Style {
    button::Style {
        background: Some(Background::Color(
            if matches!(status, button::Status::Hovered | button::Status::Pressed) {
                Color::from_rgb(0.23, 0.32, 0.42)
            } else if active {
                Color::from_rgb(0.19, 0.29, 0.40)
            } else {
                Color::TRANSPARENT
            },
        )),
        text_color: INSPECTOR_TEXT,
        border: Border {
            radius: 3.0.into(),
            ..Default::default()
        },
        ..Default::default()
    }
}

pub(super) fn popup_surface(_: &iced::Theme) -> container::Style {
    container::Style {
        background: Some(Background::Color(Color::from_rgb(0.12, 0.14, 0.17))),
        border: Border {
            color: INSPECTOR_MUTED,
            width: 1.,
            radius: 5.0.into(),
        },
        shadow: iced::Shadow {
            color: Color::BLACK.scale_alpha(0.3),
            offset: iced::Vector::new(0., 3.),
            blur_radius: 10.,
        },
        ..Default::default()
    }
}

pub(super) fn popup_separator<M: 'static>() -> Element<'static, M> {
    container(iced::widget::rule::horizontal(1))
        .padding([5, 4])
        .into()
}

pub(super) fn inspector_heading(label: impl Into<String>) -> Element<'static, Message> {
    text(label.into())
        .size(17)
        .font(iced::Font {
            weight: iced::font::Weight::Semibold,
            ..iced::Font::DEFAULT
        })
        .color(INSPECTOR_TEXT)
        .into()
}

pub(super) fn inspector_section(label: &str) -> Element<'static, Message> {
    inspector_section_frame(inspector_section_label(label))
}

/// One disclosure header for operation sections and individual height planes.
pub(super) fn inspector_disclosure_heading<'a>(
    label: &str,
    collapsed: bool,
    on_press: Message,
    trailing: Option<Element<'a, Message>>,
) -> Element<'a, Message> {
    let arrow: &'static [u8] = if collapsed {
        include_bytes!("../assets/icons/disclosure-right.svg")
    } else {
        include_bytes!("../assets/icons/disclosure-down.svg")
    };
    let arrow = iced::widget::svg(iced::widget::svg::Handle::from_memory(arrow))
        .width(12)
        .height(14);
    let heading: Element<'a, Message> = if let Some(toggle) = trailing {
        row![
            button(arrow)
                .padding(0)
                .height(24)
                .style(button::text)
                .on_press(on_press.clone()),
            toggle,
            button(inspector_section_label(label))
                .padding(0)
                .height(24)
                .width(Length::Fill)
                .style(button::text)
                .on_press(on_press),
        ]
        .spacing(8)
        .align_y(Alignment::Center)
        .into()
    } else {
        button(
            row![arrow, inspector_section_label(label)]
                .spacing(8)
                .align_y(Alignment::Center),
        )
        .padding(0)
        .height(24)
        .width(Length::Fill)
        .style(button::text)
        .on_press(on_press)
        .into()
    };
    inspector_section_frame(heading)
}

pub(super) fn inspector_section_label(label: &str) -> iced::widget::Text<'static> {
    text(label.to_owned())
        .size(INSPECTOR_TEXT_SIZE)
        .font(INSPECTOR_SECTION_FONT)
        .color(INSPECTOR_TEXT)
}

/// The checkbox label is clickable too.
pub(super) fn inspector_section_checkbox(
    label: &str,
    enabled: bool,
) -> iced::widget::Checkbox<'static, Message> {
    checkbox(enabled)
        .label(label.to_owned())
        .size(14)
        .spacing(8)
        .text_size(INSPECTOR_TEXT_SIZE)
        .font(INSPECTOR_SECTION_FONT)
}

fn inspector_section_frame<'a>(content: impl Into<Element<'a, Message>>) -> Element<'a, Message> {
    let content: Element<'a, Message> = content.into();
    container(
        column![
            iced::widget::rule::horizontal(1).style(|_| iced::widget::rule::Style {
                color: OUTLINE,
                radius: 0.0.into(),
                fill_mode: iced::widget::rule::FillMode::Full,
                snap: true,
            }),
            content,
        ]
        .spacing(10),
    )
    .padding(Padding {
        top: 10.0,
        bottom: 2.0,
        ..Padding::ZERO
    })
    .width(Length::Fill)
    .into()
}

pub(super) fn inspector_button<'a, M: 'a>(
    label: impl Into<String>,
) -> iced::widget::Button<'a, M> {
    button(
        text(label.into())
            .size(INSPECTOR_TEXT_SIZE)
            .align_x(iced::alignment::Horizontal::Center),
    )
    .padding([7, 10])
    .style(inspector_secondary)
}

/// Use the regular inspector button treatment for controls that start a
/// viewport pick, with a restrained accent while picking is active.
pub(super) fn inspector_pick_button<'a>(
    label: impl Into<String>,
    picking: bool,
) -> iced::widget::Button<'a, Message> {
    inspector_button(label).style(move |theme, status| inspector_pick_style(theme, status, picking))
}

pub(super) fn inspector_pick_style(
    theme: &iced::Theme,
    status: button::Status,
    picking: bool,
) -> button::Style {
    let mut style = inspector_secondary(theme, status);
    if picking {
        style.border.color = palette::ACCENT_BLUE;
        style.background = Some(Background::Color(Color::from_rgb(0.15, 0.20, 0.26)));
    }
    style
}

pub(super) fn inspector_secondary(_: &iced::Theme, status: button::Status) -> button::Style {
    let disabled = matches!(status, button::Status::Disabled);
    button::Style {
        background: Some(Background::Color(match status {
            button::Status::Hovered => Color::from_rgb(0.23, 0.26, 0.30),
            button::Status::Pressed => Color::from_rgb(0.12, 0.15, 0.18),
            _ => SURFACE,
        })),
        text_color: if disabled {
            INSPECTOR_MUTED.scale_alpha(0.5)
        } else {
            INSPECTOR_TEXT
        },
        border: Border {
            color: if disabled {
                OUTLINE.scale_alpha(0.5)
            } else {
                OUTLINE
            },
            width: 1.0,
            radius: 5.0.into(),
        },
        ..button::Style::default()
    }
}

pub(super) fn inspector_primary(theme: &iced::Theme, status: button::Status) -> button::Style {
    if matches!(status, button::Status::Disabled) {
        return inspector_secondary(theme, status);
    }
    button::Style {
        background: Some(Background::Color(match status {
            button::Status::Hovered => Color::from_rgb(0.26, 0.59, 0.92),
            button::Status::Pressed => Color::from_rgb(0.14, 0.40, 0.69),
            _ => Color::from_rgb(0.18, 0.49, 0.80),
        })),
        text_color: Color::WHITE,
        border: Border {
            radius: 5.0.into(),
            ..Border::default()
        },
        ..button::Style::default()
    }
}

pub(super) fn inspector_tab(
    _: &iced::Theme,
    status: button::Status,
    active: bool,
) -> button::Style {
    button::Style {
        background: Some(Background::Color(if active {
            Color::from_rgb(0.19, 0.29, 0.40)
        } else if matches!(status, button::Status::Hovered) {
            SURFACE
        } else {
            Color::TRANSPARENT
        })),
        text_color: if active {
            Color::from_rgb(0.56, 0.77, 1.0)
        } else {
            INSPECTOR_MUTED
        },
        border: Border {
            color: if active {
                Color::from_rgb(0.27, 0.43, 0.59)
            } else {
                Color::TRANSPARENT
            },
            width: 1.0,
            radius: 5.0.into(),
        },
        ..button::Style::default()
    }
}

pub(super) fn inspector_input<'a>(
    placeholder: &str,
    value: &str,
) -> iced::widget::TextInput<'a, Message> {
    text_input(placeholder, value)
        .size(INSPECTOR_TEXT_SIZE)
        .padding([7, 9])
        .style(inspector_input_style)
}

pub(super) fn inspector_input_style(
    theme: &iced::Theme,
    status: text_input::Status,
) -> text_input::Style {
    let mut style = text_input::default(theme, status);
    style.background = Background::Color(Color::from_rgb(0.12, 0.14, 0.17));
    style.border = Border {
        color: match status {
            text_input::Status::Focused { .. } => palette::ACCENT_BLUE,
            text_input::Status::Hovered => INSPECTOR_MUTED,
            _ => OUTLINE,
        },
        width: 1.0,
        radius: 5.0.into(),
    };
    style.value = INSPECTOR_TEXT;
    style.placeholder = INSPECTOR_MUTED;
    style.selection = palette::SELECT_BG;
    style
}

pub(super) fn inspector_picker(_: &iced::Theme, status: pick_list::Status) -> pick_list::Style {
    pick_list::Style {
        text_color: INSPECTOR_TEXT,
        placeholder_color: INSPECTOR_MUTED,
        handle_color: INSPECTOR_MUTED,
        background: Background::Color(Color::from_rgb(0.12, 0.14, 0.17)),
        border: Border {
            color: if matches!(status, pick_list::Status::Active) {
                OUTLINE
            } else {
                palette::ACCENT_BLUE
            },
            width: 1.0,
            radius: 5.0.into(),
        },
    }
}

/// Right-aligned values share a label column and an explicit unit gutter.
pub(super) fn inspector_value<'a>(
    label: impl Into<String>,
    value: &str,
    unit: &'a str,
    on_input: impl Fn(String) -> Message + 'a,
) -> Element<'a, Message> {
    let invalid = value.trim().parse::<f64>().is_err();
    row![
        text(label.into())
            .size(INSPECTOR_TEXT_SIZE)
            .color(INSPECTOR_TEXT)
            .width(Length::FillPortion(5)),
        inspector_input("", value)
            .align_x(iced::alignment::Horizontal::Right)
            .style(move |theme, status| {
                let mut style = inspector_input_style(theme, status);
                if invalid {
                    style.border.color = palette::ERROR;
                    style.border.width = 1.5;
                    style.value = palette::ERROR;
                }
                style
            })
            .on_input(on_input)
            .width(Length::FillPortion(6)),
        text(unit).size(11).color(INSPECTOR_MUTED).width(24),
    ]
    .spacing(8)
    .align_y(Alignment::Center)
    .into()
}

pub(super) fn inspector_footer<'a, M: 'a>(
    content: impl Into<Element<'a, M>>,
) -> Element<'a, M> {
    container(content)
        .padding([12, 14])
        .width(Length::Fill)
        .style(|_| container::Style {
            background: Some(Background::Color(Color::from_rgb(0.14, 0.15, 0.18))),
            border: Border {
                color: OUTLINE,
                width: 1.0,
                ..Border::default()
            },
            ..container::Style::default()
        })
        .into()
}
