//! Explanations and illustrations for parameter hover help.
use super::*;
use profile_parameters::{Number, Toggle};

pub(super) const PROFILE_ENTRY_SIDES: &str = "Start outside/inside alternates the cutter side for outer and inner boundaries. Always outside/inside uses the same side for every contour in this entry. On follows each contour with the cutter centre. Open left/right follows the selected chain direction.";

pub(super) const HOLDING_TABS: &str = "Leave small bridges of material to hold the part in place. Configure their shape, size and placement on a single closed contour.";
pub(super) const STOCK_TO_LEAVE: &str = "Leave material for a finishing operation. Radial allowance remains on walls; axial allowance remains on floors. Facing uses axial allowance only.";

pub(super) fn height_help(plane: cam_model::HeightPlaneKind) -> &'static str {
    use cam_model::HeightPlaneKind as Plane;
    match plane {
        Plane::Clearance => "Clearance is the safe height for rapid travel between features. Choose a reference and offset along setup Z that clears the workpiece and fixtures.",
        Plane::Retract => "Retract is the return height between passes or plunges. Choose its reference and setup-Z offset above material that must be cleared.",
        Plane::Feed => "Feed height is where the approach changes from rapid travel to the plunge feed. Choose a reference and setup-Z offset, or disable this optional plane to use the normal strategy approach.",
        Plane::Top => "Top is the upper cutting reference. Choose a model, stock, contour or other available reference and apply an offset along setup Z.",
        Plane::Bottom => "Bottom is the lower cutting limit. Choose its reference and setup-Z offset to set the final machining depth.",
    }
}

fn illustration(help: &str) -> Option<&'static [u8]> {
    if help == drill_cycle::CYCLE_HELP
        || cam_model::DrillCycleMode::ALL
            .into_iter()
            .any(|mode| drill_cycle::help(mode) == help)
    {
        return Some(include_bytes!("../assets/help/drill-cycles.svg").as_slice());
    }
    if help == help::CLIMB {
        Some(include_bytes!("../assets/help/climb-conventional.svg").as_slice())
    } else if [
        Number::FinishDepths.help(),
        Number::FinishDepthStep.help(),
        Toggle::Even.help(),
    ]
    .contains(&help)
    {
        Some(include_bytes!("../assets/help/profile-depths.svg").as_slice())
    } else if help == Toggle::RoughFinal.help() {
        Some(include_bytes!("../assets/help/profile-depth-policy.svg").as_slice())
    } else if help == Toggle::FinishOnlyFinal.help() {
        Some(include_bytes!("../assets/help/profile-finish-depth.svg").as_slice())
    } else if [
        Number::FinishStep.help(),
        Number::FinishCount.help(),
        Number::SpringPasses.help(),
        profile_parameters::PROFILE_FINISHING_HELP,
    ]
    .contains(&help)
    {
        Some(include_bytes!("../assets/help/profile-finishing.svg").as_slice())
    } else if help == Number::FloorLeave.help() {
        Some(include_bytes!("../assets/help/profile-floor.svg").as_slice())
    } else if help == profile_parameters::WALL_ALLOWANCE_HELP {
        Some(include_bytes!("../assets/help/profile-wall.svg").as_slice())
    } else if help == profile_parameters::CUT_ORDER_HELP {
        Some(include_bytes!("../assets/help/profile-order.svg").as_slice())
    } else if help == profile_parameters::CORNER_HELP {
        Some(include_bytes!("../assets/help/profile-corners.svg").as_slice())
    } else if help == Number::Taper.help() {
        Some(include_bytes!("../assets/help/profile-taper.svg").as_slice())
    } else if help == Number::Tolerance.help() {
        Some(include_bytes!("../assets/help/profile-tolerance.svg").as_slice())
    } else if [help::LEAD_IN, help::LEAD_OUT, Toggle::SameLeadOut.help()].contains(&help) {
        Some(include_bytes!("../assets/help/lead-modes.svg").as_slice())
    } else if [
        Field::LeadInSize.help(),
        Field::LeadOutSize.help(),
        Field::ClearLeadInSize.help(),
        Field::ClearLeadOutSize.help(),
    ]
    .contains(&help)
    {
        Some(include_bytes!("../assets/help/lead-size.svg").as_slice())
    } else if [Field::LeadOverlap.help(), Field::ClearLeadOverlap.help()].contains(&help) {
        Some(include_bytes!("../assets/help/lead-overlap.svg").as_slice())
    } else if [Number::LeadInSweep.help(), Number::LeadOutSweep.help()].contains(&help) {
        Some(include_bytes!("../assets/help/lead-sweep.svg").as_slice())
    } else if help == Number::LeadEndDistance.help() {
        Some(include_bytes!("../assets/help/lead-feed.svg").as_slice())
    } else if [
        Field::BoreLeadRadius.help(),
        operation_linking::BORE_CENTER_HELP,
    ]
    .contains(&help)
    {
        Some(include_bytes!("../assets/help/lead-bore.svg").as_slice())
    } else if [help::PLUNGE, Field::PlungeA.help(), Field::PlungeB.help()].contains(&help)
        || [
            Plunge::Ramp { angle_deg: 5.0 },
            Plunge::Helix {
                radius: 1.0,
                pitch: 1.0,
            },
            Plunge::ZigZag {
                length: 1.0,
                angle_deg: 5.0,
            },
        ]
        .into_iter()
        .any(|plunge| {
            [true, false]
                .into_iter()
                .any(|first| plunge_param_help(plunge, first) == Some(help))
        })
    {
        Some(include_bytes!("../assets/help/plunge.svg").as_slice())
    } else if [
        HOLDING_TABS,
        Field::TabWidth.help(),
        Field::TabHeight.help(),
        Field::TabSpacing.help(),
        Number::TabCount.help(),
        "Rectangular tabs keep a flat top; triangular tabs rise and fall gradually.",
    ]
    .contains(&help)
    {
        Some(include_bytes!("../assets/help/tabs.svg").as_slice())
    } else if [
        STOCK_TO_LEAVE,
        Field::ProfileOffset.help(),
        Field::StockLeaveRadial.help(),
        Field::StockLeaveAxial.help(),
    ]
    .contains(&help)
    {
        Some(include_bytes!("../assets/help/allowances.svg").as_slice())
    } else if [
        Field::Stepdown.help(),
        Field::Stepover.help(),
        Toggle::Finishing.help(),
    ]
    .contains(&help)
    {
        Some(include_bytes!("../assets/help/passes.svg").as_slice())
    } else if [
        Field::Clearance.help(),
        Field::Retract.help(),
        Field::TopOfStock.help(),
        Field::Depth.help(),
    ]
    .contains(&help)
        || cam_model::HeightPlaneKind::ALL
            .into_iter()
            .any(|plane| height_help(plane) == help)
    {
        Some(include_bytes!("../assets/help/heights.svg").as_slice())
    } else if [help::SIDE, PROFILE_ENTRY_SIDES].contains(&help) {
        Some(include_bytes!("../assets/help/sides.svg").as_slice())
    } else {
        None
    }
}

pub(super) fn content(help: &str) -> Element<'static, Message> {
    let asset = illustration(help);
    let mut body = iced::widget::Column::new()
        .spacing(8)
        .push(text(help.to_owned()).size(12));
    if let Some(bytes) = asset {
        body = body.push(
            iced::widget::svg(iced::widget::svg::Handle::from_memory(bytes)).width(Length::Fill),
        );
    }
    container(body.width(if asset.is_some() { 360 } else { 280 }))
        .padding(10)
        .style(popup_surface)
        .into()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parameter_specific_help_keeps_its_illustration() {
        for help in [
            help::LEAD_IN,
            help::LEAD_OUT,
            Field::LeadInSize.help(),
            Field::LeadOutSize.help(),
            Field::ClearLeadInSize.help(),
            Field::ClearLeadOutSize.help(),
            Field::LeadOverlap.help(),
            Field::ClearLeadOverlap.help(),
            Field::BoreLeadRadius.help(),
            operation_linking::BORE_CENTER_HELP,
            Toggle::SameLeadOut.help(),
            Number::LeadInSweep.help(),
            Number::LeadOutSweep.help(),
            Number::LeadEndDistance.help(),
        ] {
            assert!(illustration(help).is_some(), "missing illustration: {help}");
        }
        assert_eq!(illustration(help::LEAD_IN), illustration(help::LEAD_OUT));
        let plunge_image = illustration(help::PLUNGE).unwrap();
        for plunge in [
            Plunge::Ramp { angle_deg: 5.0 },
            Plunge::Helix {
                radius: 1.0,
                pitch: 1.0,
            },
            Plunge::ZigZag {
                length: 1.0,
                angle_deg: 5.0,
            },
        ] {
            for first in [true, false] {
                if let Some(help) = plunge_param_help(plunge, first) {
                    assert_eq!(illustration(help), Some(plunge_image));
                }
            }
        }
        let heights_image = illustration(Field::Clearance.help()).unwrap();
        for plane in cam_model::HeightPlaneKind::ALL {
            assert_eq!(illustration(height_help(plane)), Some(heights_image));
        }
    }

    #[test]
    fn profile_pass_help_uses_matching_diagrams() {
        let depth = illustration(Number::FinishDepths.help()).unwrap();
        assert_eq!(illustration(Number::FinishDepthStep.help()), Some(depth));
        assert_eq!(illustration(Toggle::Even.help()), Some(depth));
        let finishing = illustration(Number::FinishCount.help()).unwrap();
        assert_eq!(illustration(Number::FinishStep.help()), Some(finishing));
        assert_eq!(illustration(Number::SpringPasses.help()), Some(finishing));
        for help in [
            Number::FloorLeave.help(),
            Number::Tolerance.help(),
            Number::Taper.help(),
            Toggle::RoughFinal.help(),
            Toggle::FinishOnlyFinal.help(),
            profile_parameters::WALL_ALLOWANCE_HELP,
            profile_parameters::CUT_ORDER_HELP,
            profile_parameters::CORNER_HELP,
        ] {
            let image = illustration(help).expect("profile pass help needs an illustration");
            assert_ne!(image, depth);
            assert_ne!(image, finishing);
        }
    }

    #[test]
    fn illustrated_help_assets_are_valid_and_have_text_descriptions() {
        for (name, bytes) in [
            (
                "drill cycles",
                include_str!("../assets/help/drill-cycles.svg"),
            ),
            ("sides", include_str!("../assets/help/sides.svg")),
            ("allowances", include_str!("../assets/help/allowances.svg")),
            ("passes", include_str!("../assets/help/passes.svg")),
            ("leads", include_str!("../assets/help/leads.svg")),
            ("lead modes", include_str!("../assets/help/lead-modes.svg")),
            ("lead size", include_str!("../assets/help/lead-size.svg")),
            (
                "lead overlap",
                include_str!("../assets/help/lead-overlap.svg"),
            ),
            ("lead sweep", include_str!("../assets/help/lead-sweep.svg")),
            ("lead feed", include_str!("../assets/help/lead-feed.svg")),
            ("lead bore", include_str!("../assets/help/lead-bore.svg")),
            (
                "profile depths",
                include_str!("../assets/help/profile-depths.svg"),
            ),
            (
                "depth policy",
                include_str!("../assets/help/profile-depth-policy.svg"),
            ),
            (
                "finish depth",
                include_str!("../assets/help/profile-finish-depth.svg"),
            ),
            (
                "profile finishing",
                include_str!("../assets/help/profile-finishing.svg"),
            ),
            (
                "profile floor",
                include_str!("../assets/help/profile-floor.svg"),
            ),
            (
                "profile wall",
                include_str!("../assets/help/profile-wall.svg"),
            ),
            (
                "profile order",
                include_str!("../assets/help/profile-order.svg"),
            ),
            (
                "profile corners",
                include_str!("../assets/help/profile-corners.svg"),
            ),
            (
                "profile taper",
                include_str!("../assets/help/profile-taper.svg"),
            ),
            (
                "profile tolerance",
                include_str!("../assets/help/profile-tolerance.svg"),
            ),
            ("plunge", include_str!("../assets/help/plunge.svg")),
            ("heights", include_str!("../assets/help/heights.svg")),
            ("tabs", include_str!("../assets/help/tabs.svg")),
        ] {
            let doc = roxmltree::Document::parse(bytes).unwrap_or_else(|e| panic!("{name}: {e}"));
            assert!(doc
                .descendants()
                .any(|n| n.has_tag_name("title") && n.text().is_some()));
        }
    }
}
