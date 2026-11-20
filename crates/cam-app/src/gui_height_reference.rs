//! Icon-labelled height references in a grouped, scrollable popup.
use super::*;
use cam_model::{HeightPlaneKind as Plane, HeightReference as Reference};
use iced::advanced::widget;

fn menu_id(plane: Plane) -> widget::Id {
    widget::Id::from(format!("height-reference-menu-{plane:?}"))
}

fn group(reference: Reference) -> u8 {
    match reference {
        Reference::Plane(_) | Reference::Disabled => 0,
        Reference::ModelTop
        | Reference::ModelBottom
        | Reference::StockTop
        | Reference::StockBottom
        | Reference::FixtureTop
        | Reference::FixtureBottom
        | Reference::SelectedContours => 1,
        Reference::Selection | Reference::Origin => 2,
        Reference::Highest | Reference::Lowest => 3,
    }
}

fn icon_bytes(reference: Reference) -> &'static [u8] {
    match reference {
        Reference::Plane(Plane::Clearance) => {
            include_bytes!("../assets/icons/heights/clearance.svg")
        }
        Reference::Plane(Plane::Retract) => include_bytes!("../assets/icons/heights/retract.svg"),
        Reference::Plane(Plane::Feed) => include_bytes!("../assets/icons/heights/feed.svg"),
        Reference::Plane(Plane::Top) => include_bytes!("../assets/icons/heights/top.svg"),
        Reference::Plane(Plane::Bottom) => include_bytes!("../assets/icons/heights/bottom.svg"),
        Reference::Disabled => include_bytes!("../assets/icons/heights/disabled.svg"),
        Reference::ModelTop => include_bytes!("../assets/icons/heights/model_top.svg"),
        Reference::ModelBottom => include_bytes!("../assets/icons/heights/model_bottom.svg"),
        Reference::StockTop => include_bytes!("../assets/icons/heights/stock_top.svg"),
        Reference::StockBottom => include_bytes!("../assets/icons/heights/stock_bottom.svg"),
        Reference::FixtureTop => include_bytes!("../assets/icons/heights/fixture_top.svg"),
        Reference::FixtureBottom => include_bytes!("../assets/icons/heights/fixture_bottom.svg"),
        Reference::SelectedContours => include_bytes!("../assets/icons/heights/contours.svg"),
        Reference::Selection => include_bytes!("../assets/icons/heights/selection.svg"),
        Reference::Origin => include_bytes!("../assets/icons/heights/origin.svg"),
        Reference::Highest => include_bytes!("../assets/icons/heights/highest.svg"),
        Reference::Lowest => include_bytes!("../assets/icons/heights/lowest.svg"),
    }
}

fn label<M: 'static>(reference: Reference) -> Element<'static, M> {
    row![
        iced::widget::svg(iced::widget::svg::Handle::from_memory(icon_bytes(
            reference
        )))
        .width(22)
        .height(22),
        text(reference.to_string()).size(12).width(Length::Fill),
    ]
    .spacing(8)
    .align_y(Alignment::Center)
    .into()
}

pub(super) fn picker(plane: Plane, selected: Reference) -> Element<'static, Message> {
    let options = Reference::options(plane);
    let setting = parameter_options::height_reference_setting(plane);
    let widget = icon_picker::picker(
        menu_id(plane),
        options.clone(),
        selected,
        label,
        group,
        setting.on_change(),
    );
    setting.menu(&selected, selected.to_string(), &options, widget)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_height_reference_icons_are_valid_square_svgs() {
        let references = Reference::options(Plane::Feed)
            .into_iter()
            .chain([Reference::Plane(Plane::Feed)]);
        for reference in references {
            let source = std::str::from_utf8(icon_bytes(reference)).unwrap();
            let document = roxmltree::Document::parse(source).unwrap();
            let root = document.root_element();
            assert_eq!(root.tag_name().name(), "svg", "{reference}");
            assert_eq!(root.attribute("viewBox"), Some("0 0 24 24"), "{reference}");
            assert!(root.children().any(|node| node.is_element()), "{reference}");
        }
    }
}
