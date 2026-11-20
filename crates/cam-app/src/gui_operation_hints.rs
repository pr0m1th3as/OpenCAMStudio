//! Illustrated hover cards for the Machining ribbon.
use super::{
    inspector_heading, popup_surface, Message, OpKind, INSPECTOR_MUTED, INSPECTOR_TEXT,
    INSPECTOR_TEXT_SIZE,
};
use iced::widget::{column, container, row, svg, text, Space};
use iced::{Element, Length};

macro_rules! example {
    ($file:literal) => {
        include_bytes!(concat!("../assets/operation-hints/", $file, ".svg"))
    };
}

struct Hint {
    title: &'static str,
    purpose: &'static str,
    detail: &'static str,
    examples: &'static [&'static [u8]],
}

fn hint(kind: OpKind) -> Hint {
    match kind {
        OpKind::Face => Hint {
            title: "Face",
            purpose: "Skims the top of the stock to create a flat surface.",
            detail: "Overlapping passes sweep across the stock and turn beyond its edges.",
            examples: &[example!("face")],
        },
        OpKind::Profile => Hint {
            title: "Profile",
            purpose: "Mills along the edge of a part or along an open contour.",
            detail: "The path is offset for the cutter radius when cutting inside or outside an edge. Stepdowns divide a deep cut into levels.",
            examples: &[example!("profile")],
        },
        OpKind::Pocket => Hint {
            title: "Pocket",
            purpose: "Clears material inside a closed boundary down to a flat floor.",
            detail: "Use concentric clearing (left) or enable Adaptive clearing under Passes for 2D Adaptive Clearing (right). Both can leave islands standing.",
            examples: &[example!("pocket"), example!("pocket-adaptive")],
        },
        OpKind::Thread => Hint {
            title: "Thread Milling",
            purpose: "Mills an internal thread in a prepared hole or an external thread on a boss.",
            detail: "The cutter follows a helix whose pitch and hand define the thread.",
            examples: &[example!("thread")],
        },
        OpKind::Bore => Hint {
            title: "Bore",
            purpose: "Finishes the circular wall of an existing hole or boss with an end mill.",
            detail: "The cutter changes depth as it orbits. The helix pitch sets the axial travel per turn.",
            examples: &[example!("bore")],
        },
        OpKind::Chamfer => Hint {
            title: "Chamfer",
            purpose: "Cuts a bevel along a selected edge with a chamfer cutter.",
            detail: "The cutter follows the edge at an offset that sets the bevel width.",
            examples: &[example!("chamfer")],
        },
        OpKind::Engrave => Hint {
            title: "Engrave",
            purpose: "Cuts a narrow groove along a line or contour with a V-bit.",
            detail: "Cut depth and tool angle determine how wide the groove appears.",
            examples: &[example!("engrave")],
        },
        OpKind::Carve => Hint {
            title: "Carve",
            purpose: "Cuts a recessed shape inside a closed outline with a V-bit.",
            detail: "The cutter goes deeper in wider areas, up to the set depth limit.",
            examples: &[example!("carve")],
        },
        OpKind::Drill => Hint {
            title: "Drill",
            purpose: "Drills a hole at each selected centre.",
            detail: "A peck cycle retracts between depth increments to help clear chips. A dwell can pause the drill at the bottom.",
            examples: &[example!("drill")],
        },
        OpKind::Adaptive => Hint {
            title: "3D Adaptive Clearing",
            purpose: "Roughs stock around selected 3D geometry in depth layers.",
            detail: "Curving passes aim to keep cutter engagement consistent while leaving the model intact.",
            examples: &[example!("adaptive")],
        },
        OpKind::Parallel => Hint {
            title: "3D Parallel / Scallop",
            purpose: "Finishes a 3D surface by varying tool height to follow its shape.",
            detail: "Parallel uses straight rows in plan view. Scallop uses successive offsets along the surface.",
            examples: &[
                example!("parallel"),
                example!("scallop"),
            ],
        },
        OpKind::Spiral => Hint {
            title: "3D Spiral",
            purpose: "Finishes a 3D surface with passes that wind around a centre.",
            detail: "The tool follows the surface as the path changes radius, making it useful for round features.",
            examples: &[example!("spiral")],
        },
    }
}

pub(super) fn content(kind: OpKind, shortcut: &str) -> Element<'static, Message> {
    let hint = hint(kind);
    let mut examples = row![].spacing(8);
    for &example in hint.examples {
        examples = examples.push(svg(svg::Handle::from_memory(example)));
    }
    let mut body = column![
        inspector_heading(hint.title),
        text(hint.purpose)
            .size(INSPECTOR_TEXT_SIZE)
            .color(INSPECTOR_TEXT),
        text(hint.detail)
            .size(INSPECTOR_TEXT_SIZE)
            .color(INSPECTOR_TEXT),
        examples,
    ]
    .spacing(8);
    if shortcut != "Unassigned" {
        body = body.push(row![
            Space::new().width(Length::Fill),
            text(format!("Shortcut  {shortcut}"))
                .size(11)
                .color(INSPECTOR_MUTED),
        ]);
    }
    container(body)
        .width(Length::Fixed(360.0))
        .padding(10)
        .style(popup_surface)
        .into()
}
