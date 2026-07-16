//! The iced desktop shell — a thin view over [`crate::AppController`].
//!
//! A professional-CAM layout built on iced's [`pane_grid`]: a tabbed **ribbon**
//! (Home / Operations / Tooling / View / Windows) across the top, then four
//! docked, resizable panes — a **Project** tree
//! (select a node), the **Viewport** (backplot + simulated stock), an
//! **Inspector** (edit the selected node), and an **Output** console
//! (diagnostics + status). Any pane can be shown/hidden from the **Windows**
//! ribbon tab (hiding one keeps the others' sizes). All behaviour is delegated to
//! the controller; this module only translates messages and draws. Only compiled
//! with the `gui` feature.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use iced::widget::pane_grid::{self, PaneGrid};
use iced::widget::{
    button, checkbox, column, container, mouse_area, pick_list, row, scrollable, shader, slider,
    text, text_input, Space,
};
use iced::{Alignment, Background, Border, Color, Element, Length, Padding};

use cam_model::{Axis, Envelope, Hand, Lead, Machine, Operation, Plunge, Point3, Side, ToolKind};

use crate::tool_library::ToolLibrary;

/// Ribbon palette and metrics, adopted from **OpenCADStudio**'s ribbon
/// (`HakanSeven12/OpenCADStudio`, GPL-3.0) so users moving CAD → CAM meet a
/// familiar surface. Values are its own `TOPBAR_BG`/`RIBBON_BG`/… constants; the
/// widget structure here is ours (a simpler five-tab band, no density modes).
mod palette {
    use iced::Color;

    const fn rgb(r: f32, g: f32, b: f32) -> Color {
        Color { r, g, b, a: 1.0 }
    }

    /// Tab-strip background.
    pub const TOPBAR_BG: Color = rgb(0.17, 0.17, 0.17);
    /// Ribbon-band background.
    pub const RIBBON_BG: Color = rgb(0.22, 0.22, 0.22);
    /// The band's dark framing border.
    pub const BORDER_DARK: Color = rgb(0.12, 0.12, 0.12);
    /// Active-tab accent.
    pub const ACCENT_BLUE: Color = rgb(0.20, 0.55, 0.90);
    /// Normal control text.
    pub const LABEL_COLOR: Color = rgb(0.82, 0.82, 0.82);
    /// Group caption text (and disabled controls).
    pub const GROUP_LABEL: Color = rgb(0.50, 0.50, 0.50);
    /// Hover fill for tabs.
    pub const TOOL_HOVER: Color = rgb(0.32, 0.32, 0.32);
    /// Hover fill for command rows.
    pub const ROW_HOVER: Color = rgb(0.24, 0.24, 0.24);
    /// Pane content background.
    pub const PANE_BG: Color = rgb(0.15, 0.15, 0.15);
    /// Visible divider line framing each pane.
    pub const SEPARATOR: Color = rgb(0.34, 0.34, 0.34);
    /// Selected project-tree row highlight (a muted fill, not the loud button blue).
    pub const SELECT_BG: Color = rgb(0.20, 0.28, 0.38);
    /// Warning accent (amber) — e.g. the exact-duplicate operation marker. Paired
    /// with a ⚠ glyph so the meaning survives colour-vision deficiency.
    pub const WARN: Color = rgb(0.95, 0.62, 0.10);
}

/// Ribbon command icons. Each variant is a small embedded SVG (see
/// `assets/icons/CREDITS.md` — most reused from OpenCADStudio, GPL-3.0; the
/// CAM-specific ones drawn to match). The SVGs carry their own colours, so we
/// render them untinted.
#[derive(Clone, Copy, Debug)]
enum Icon {
    New,
    Open,
    Save,
    Import,
    Export,
    Undo,
    Redo,
    Run,
    Profile,
    Pocket,
    Drill,
    Thread,
    Chamfer,
    Face,
    NewTool,
    Duplicate,
    Delete,
    ShowStock,
    ResetView,
    ShowCube,
    SetOrigin,
}

impl Icon {
    fn bytes(self) -> &'static [u8] {
        match self {
            Icon::New => include_bytes!("../assets/icons/new.svg"),
            Icon::Open => include_bytes!("../assets/icons/open.svg"),
            Icon::Save => include_bytes!("../assets/icons/save.svg"),
            Icon::Import => include_bytes!("../assets/icons/cui_import.svg"),
            Icon::Export => include_bytes!("../assets/icons/cui_export.svg"),
            Icon::Undo => include_bytes!("../assets/icons/undo.svg"),
            Icon::Redo => include_bytes!("../assets/icons/redo.svg"),
            Icon::Run => include_bytes!("../assets/icons/run.svg"),
            Icon::Profile => include_bytes!("../assets/icons/profile.svg"),
            Icon::Pocket => include_bytes!("../assets/icons/pocket.svg"),
            Icon::Drill => include_bytes!("../assets/icons/drill.svg"),
            Icon::Thread => include_bytes!("../assets/icons/thread.svg"),
            Icon::Chamfer => include_bytes!("../assets/icons/chamfer.svg"),
            Icon::Face => include_bytes!("../assets/icons/face.svg"),
            Icon::NewTool => include_bytes!("../assets/icons/endmill.svg"),
            Icon::Duplicate => include_bytes!("../assets/icons/copy.svg"),
            Icon::Delete => include_bytes!("../assets/icons/erase.svg"),
            Icon::ShowStock => include_bytes!("../assets/icons/box3d.svg"),
            Icon::ResetView => include_bytes!("../assets/icons/zoom_ext.svg"),
            Icon::ShowCube => include_bytes!("../assets/icons/viewcube.svg"),
            Icon::SetOrigin => include_bytes!("../assets/icons/origin.svg"),
        }
    }

    /// An iced SVG handle for this icon. The handle id is derived from the bytes,
    /// so iced caches the rasterisation across frames.
    fn handle(self) -> iced::widget::svg::Handle {
        iced::widget::svg::Handle::from_memory(self.bytes())
    }
}

/// An SVG icon widget sized to a square `size` px.
fn icon_svg(icon: Icon, size: f32) -> Element<'static, Message> {
    iced::widget::svg(icon.handle())
        .width(Length::Fixed(size))
        .height(Length::Fixed(size))
        .into()
}
use cam_render::{MeshVertex, OrbitCamera, Scene, Vertex, PART};
use cam_toolpath::{CancelToken, Severity};

use crate::{
    op_selects_circles, AppController, LoopRef, OpKind, PendingOp, PickResult, Selection, SnapHit,
    SnapKind,
};

/// A small sample part (rectangle + circular hole) so the app is useful without
/// a file dialog on first run.
const SAMPLE_DXF: &str = "\
0\nSECTION\n2\nENTITIES\n\
0\nLINE\n10\n10.0\n20\n10.0\n11\n70.0\n21\n10.0\n\
0\nLINE\n10\n70.0\n20\n10.0\n11\n70.0\n21\n50.0\n\
0\nLINE\n10\n70.0\n20\n50.0\n11\n10.0\n21\n50.0\n\
0\nLINE\n10\n10.0\n20\n50.0\n11\n10.0\n21\n10.0\n\
0\nCIRCLE\n10\n40.0\n20\n30.0\n40\n5.0\n\
0\nENDSEC\n0\nEOF\n";

/// Launch the desktop application.
pub fn run() -> iced::Result {
    iced::application(App::new, App::update, App::view)
        .title("OpenCAMStudio")
        .theme(theme)
        .subscription(App::subscription)
        .run()
}

fn theme(_state: &App) -> iced::Theme {
    iced::Theme::Dark
}

/// Prompt (native dialog) for an existing file matching `exts`, returning its path.
async fn pick_open(name: &'static str, exts: &'static [&'static str]) -> Option<PathBuf> {
    rfd::AsyncFileDialog::new()
        .add_filter(name, exts)
        .pick_file()
        .await
        .map(|h| h.path().to_path_buf())
}

/// Prompt (native dialog) for a save location, seeded with `default_name`.
async fn pick_save(
    name: &'static str,
    default_name: &'static str,
    exts: &'static [&'static str],
) -> Option<PathBuf> {
    rfd::AsyncFileDialog::new()
        .add_filter(name, exts)
        .set_file_name(default_name)
        .save_file()
        .await
        .map(|h| h.path().to_path_buf())
}

/// A short human-readable summary of the exact-duplicate operation groups, for
/// the export confirm dialog. Each group is rendered as `#a = #b (= #c…)`.
fn describe_duplicates(groups: &[Vec<u32>]) -> String {
    let list = groups
        .iter()
        .map(|g| {
            g.iter()
                .map(|id| format!("#{id}"))
                .collect::<Vec<_>>()
                .join(" = ")
        })
        .collect::<Vec<_>>()
        .join(";  ");
    format!(
        "These operations are exact duplicates and would each post the same \
         toolpath — the machine would cut it more than once:\n\n    {list}\n\n\
         Export all of them anyway?"
    )
}

/// Native Yes/No warning dialog gating an export that contains exact-duplicate
/// operations. Returns `true` only if the user chose to proceed.
async fn confirm_export_duplicates(detail: String) -> bool {
    rfd::AsyncMessageDialog::new()
        .set_level(rfd::MessageLevel::Warning)
        .set_title("Duplicate operations")
        .set_description(detail)
        .set_buttons(rfd::MessageButtons::YesNo)
        .show()
        .await
        == rfd::MessageDialogResult::Yes
}

/// The docked panes. Any of them can be shown or hidden from the Windows ribbon
/// tab (except the Viewport, which is always present as the main view).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pane {
    Project,
    Viewport,
    Inspector,
    Output,
}

impl Pane {
    fn name(self) -> &'static str {
        match self {
            Pane::Project => "Project",
            Pane::Viewport => "Viewport",
            Pane::Inspector => "Inspector",
            Pane::Output => "Output",
        }
    }

    /// This pane's minimum size (px) along whichever axis it is split — enforced
    /// individually while resizing (see `App::clamp_resize`).
    fn min_size(self) -> f32 {
        match self {
            Pane::Project => 200.0,   // fits the Duplicate/Delete row
            Pane::Viewport => 200.0,  // the main view stays usable
            Pane::Inspector => 240.0, // fits the field rows
            Pane::Output => 60.0,     // a short console is fine
        }
    }

    /// The edge this pane docks to when re-shown. The Viewport has no fixed edge
    /// — it simply takes the space where it is split back in.
    fn dock_edge(self) -> Option<pane_grid::Edge> {
        match self {
            Pane::Project => Some(pane_grid::Edge::Left),
            Pane::Inspector => Some(pane_grid::Edge::Right),
            Pane::Output => Some(pane_grid::Edge::Bottom),
            Pane::Viewport => None,
        }
    }
}

/// Every pane, in Windows-menu order.
const ALL_PANES: [Pane; 4] = [Pane::Project, Pane::Viewport, Pane::Inspector, Pane::Output];

/// A tab in the top-bar ribbon. Each tab shows a band of grouped commands.
/// Operations and Tooling are added as those capabilities land.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RibbonTab {
    Home,
    Operations,
    Edit,
    Tooling,
    View,
    Windows,
}

impl RibbonTab {
    /// The tabs shown in the strip, left to right.
    const ALL: [RibbonTab; 6] = [
        RibbonTab::Home,
        RibbonTab::Operations,
        RibbonTab::Edit,
        RibbonTab::Tooling,
        RibbonTab::View,
        RibbonTab::Windows,
    ];

    fn label(self) -> &'static str {
        match self {
            RibbonTab::Home => "Home",
            RibbonTab::Operations => "Operations",
            RibbonTab::Edit => "Edit",
            RibbonTab::Tooling => "Tooling",
            RibbonTab::View => "View",
            RibbonTab::Windows => "Windows",
        }
    }
}

/// The kind of a [`Lead`], for the inspector picker (params come from fields).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LeadKind {
    None,
    Linear,
    Arc,
}

impl LeadKind {
    const ALL: [LeadKind; 3] = [LeadKind::None, LeadKind::Linear, LeadKind::Arc];

    fn of(lead: Lead) -> Self {
        match lead {
            Lead::None => LeadKind::None,
            Lead::Linear { .. } => LeadKind::Linear,
            Lead::Arc { .. } => LeadKind::Arc,
        }
    }

    /// A `Lead` of this kind, carrying over the previous size where sensible.
    fn to_lead(self, prev: Lead) -> Lead {
        let size = match prev {
            Lead::None => 3.0,
            Lead::Linear { length } => length,
            Lead::Arc { radius } => radius,
        };
        match self {
            LeadKind::None => Lead::None,
            LeadKind::Linear => Lead::Linear { length: size },
            LeadKind::Arc => Lead::Arc { radius: size },
        }
    }
}

impl std::fmt::Display for LeadKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            LeadKind::None => "None",
            LeadKind::Linear => "Linear",
            LeadKind::Arc => "Arc",
        })
    }
}

/// The kind of a [`Plunge`], for the inspector picker.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PlungeKind {
    Straight,
    Ramp,
    Helix,
    ZigZag,
}

impl PlungeKind {
    const ALL: [PlungeKind; 4] = [
        PlungeKind::Straight,
        PlungeKind::Ramp,
        PlungeKind::Helix,
        PlungeKind::ZigZag,
    ];

    fn of(plunge: Plunge) -> Self {
        match plunge {
            Plunge::Straight => PlungeKind::Straight,
            Plunge::Ramp { .. } => PlungeKind::Ramp,
            Plunge::Helix { .. } => PlungeKind::Helix,
            Plunge::ZigZag { .. } => PlungeKind::ZigZag,
        }
    }

    /// A `Plunge` of this kind with sensible default parameters.
    fn to_plunge(self) -> Plunge {
        match self {
            PlungeKind::Straight => Plunge::Straight,
            PlungeKind::Ramp => Plunge::Ramp { angle_deg: 5.0 },
            PlungeKind::Helix => Plunge::Helix {
                radius: 2.0,
                pitch: 1.0,
            },
            PlungeKind::ZigZag => Plunge::ZigZag {
                length: 5.0,
                angle_deg: 5.0,
            },
        }
    }
}

impl std::fmt::Display for PlungeKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            PlungeKind::Straight => "Straight",
            PlungeKind::Ramp => "Ramp",
            PlungeKind::Helix => "Helix",
            PlungeKind::ZigZag => "Zig-zag",
        })
    }
}

/// Whether a thread is cut into a bore or onto a boss, for the inspector picker
/// (a friendlier face on `ThreadOp::internal`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Bore {
    Internal,
    External,
}

impl Bore {
    const ALL: [Bore; 2] = [Bore::Internal, Bore::External];

    fn of(internal: bool) -> Self {
        if internal {
            Bore::Internal
        } else {
            Bore::External
        }
    }
}

impl std::fmt::Display for Bore {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Bore::Internal => "Internal",
            Bore::External => "External",
        })
    }
}

/// Climb vs. conventional milling, for the inspector picker (a friendlier face on
/// `ThreadOp::climb`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CutStyle {
    Climb,
    Conventional,
}

impl CutStyle {
    const ALL: [CutStyle; 2] = [CutStyle::Climb, CutStyle::Conventional];

    fn of(climb: bool) -> Self {
        if climb {
            CutStyle::Climb
        } else {
            CutStyle::Conventional
        }
    }
}

impl std::fmt::Display for CutStyle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            CutStyle::Climb => "Climb",
            CutStyle::Conventional => "Conventional",
        })
    }
}

/// The tool-geometry class as a plain discriminant, for the inspector picker
/// (a friendlier face on the data-carrying [`ToolKind`], mirroring `PlungeKind`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ToolKindPick {
    EndMill,
    BallMill,
    BullNose,
    ChamferMill,
    Drill,
    FaceMill,
    ThreadMill,
}

impl ToolKindPick {
    const ALL: [ToolKindPick; 7] = [
        ToolKindPick::EndMill,
        ToolKindPick::BallMill,
        ToolKindPick::BullNose,
        ToolKindPick::ChamferMill,
        ToolKindPick::Drill,
        ToolKindPick::FaceMill,
        ToolKindPick::ThreadMill,
    ];

    fn of(kind: ToolKind) -> Self {
        match kind {
            ToolKind::EndMill => ToolKindPick::EndMill,
            ToolKind::BallMill => ToolKindPick::BallMill,
            ToolKind::BullNose { .. } => ToolKindPick::BullNose,
            ToolKind::ChamferMill { .. } => ToolKindPick::ChamferMill,
            ToolKind::Drill { .. } => ToolKindPick::Drill,
            ToolKind::FaceMill => ToolKindPick::FaceMill,
            ToolKind::ThreadMill { .. } => ToolKindPick::ThreadMill,
        }
    }

    /// A `ToolKind` of this class with sensible default parameters.
    fn to_kind(self) -> ToolKind {
        match self {
            ToolKindPick::EndMill => ToolKind::EndMill,
            ToolKindPick::BallMill => ToolKind::BallMill,
            ToolKindPick::BullNose => ToolKind::BullNose { corner_radius: 1.0 },
            ToolKindPick::ChamferMill => ToolKind::ChamferMill {
                included_angle_deg: 90.0,
                tip_diameter: 0.0,
            },
            ToolKindPick::Drill => ToolKind::Drill {
                point_angle_deg: 118.0,
            },
            ToolKindPick::FaceMill => ToolKind::FaceMill,
            ToolKindPick::ThreadMill => ToolKind::ThreadMill { pitch: None },
        }
    }
}

impl std::fmt::Display for ToolKindPick {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.to_kind().to_string().as_str())
    }
}

/// The kind-specific inspector fields for a tool of `kind` (empty for kinds fully
/// described by diameter).
fn tool_kind_fields(kind: ToolKind) -> Vec<Field> {
    match kind {
        ToolKind::BullNose { .. } => vec![Field::CornerRadius],
        ToolKind::ChamferMill { .. } => vec![Field::ChamferAngle, Field::TipDiameter],
        ToolKind::Drill { .. } => vec![Field::PointAngle],
        ToolKind::ThreadMill { .. } => vec![Field::ToolThreadPitch],
        ToolKind::EndMill | ToolKind::BallMill | ToolKind::FaceMill => Vec::new(),
    }
}

/// Read a tool's kind-specific parameter for a field, if it applies.
fn tool_kind_field(kind: ToolKind, field: Field) -> Option<f64> {
    match (kind, field) {
        (ToolKind::BullNose { corner_radius }, Field::CornerRadius) => Some(corner_radius),
        (ToolKind::ChamferMill { included_angle_deg, .. }, Field::ChamferAngle) => {
            Some(included_angle_deg)
        }
        (ToolKind::ChamferMill { tip_diameter, .. }, Field::TipDiameter) => Some(tip_diameter),
        (ToolKind::Drill { point_angle_deg }, Field::PointAngle) => Some(point_angle_deg),
        // Single-form (None) shows as 0 — "any pitch".
        (ToolKind::ThreadMill { pitch }, Field::ToolThreadPitch) => Some(pitch.unwrap_or(0.0)),
        _ => None,
    }
}

/// Write the parsed inspector fields onto a tool's kind-specific parameters.
fn apply_tool_kind_fields(kind: &mut ToolKind, parsed: &BTreeMap<Field, f64>) {
    let get = |f: Field| parsed.get(&f).copied();
    match kind {
        ToolKind::BullNose { corner_radius } => {
            if let Some(v) = get(Field::CornerRadius) {
                *corner_radius = v;
            }
        }
        ToolKind::ChamferMill {
            included_angle_deg,
            tip_diameter,
        } => {
            if let Some(v) = get(Field::ChamferAngle) {
                *included_angle_deg = v;
            }
            if let Some(v) = get(Field::TipDiameter) {
                *tip_diameter = v;
            }
        }
        ToolKind::Drill { point_angle_deg } => {
            if let Some(v) = get(Field::PointAngle) {
                *point_angle_deg = v;
            }
        }
        ToolKind::ThreadMill { pitch } => {
            if let Some(v) = get(Field::ToolThreadPitch) {
                // 0 (or negative) means single-form; a positive value is full-profile.
                *pitch = (v > 0.0).then_some(v);
            }
        }
        ToolKind::EndMill | ToolKind::BallMill | ToolKind::FaceMill => {}
    }
}

/// An editable inspector field, keyed independently of which node owns it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Field {
    Clearance,
    Retract,
    TopOfStock,
    /// Stock: material added on both X sides of the part's bounding box (mm).
    StockXOffset,
    /// Stock: material added on both Y sides of the part's bounding box (mm).
    StockYOffset,
    /// Stock: absolute Z of the top surface (mm).
    StockTop,
    /// Stock: block thickness below the top (mm); bottom = top − thickness.
    StockThickness,
    /// Workpiece origin (datum) X / Y / Z, part-space mm.
    OriginX,
    OriginY,
    OriginZ,
    /// Program start-point offset X / Y / Z from the origin (mm).
    StartOffX,
    StartOffY,
    StartOffZ,
    ToolDiameter,
    ToolLength,
    Flutes,
    /// Bull-nose corner radius (mm).
    CornerRadius,
    /// Chamfer/V mill included point angle (deg).
    ChamferAngle,
    /// Chamfer/V mill flat-tip diameter (mm).
    TipDiameter,
    /// Drill point angle (deg).
    PointAngle,
    /// Thread mill's ground pitch (mm); 0 means single-form (any pitch).
    ToolThreadPitch,
    Depth,
    Stepdown,
    Stepover,
    /// Profile finishing allowance left on the wall (mm).
    ProfileOffset,
    Feed,
    PlungeFeed,
    /// Thread major diameter (mm).
    MajorDia,
    /// Thread pitch (mm).
    Pitch,
    /// Absolute Z of the top of a threaded length (mm).
    ThreadTop,
    /// Absolute Z of the bottom of a threaded length (mm).
    ThreadBottom,
    /// Chamfer width (mm).
    ChamferWidth,
    /// Chamfer tip depth below the top edge (mm); selects the flank section.
    ChamferDepth,
    /// Chamfer width increment per pass (mm); 0 cuts in one pass.
    ChamferStep,
    /// Lead-in size (length for Linear, radius for Arc).
    LeadInSize,
    /// Lead-out size.
    LeadOutSize,
    /// Closure overlap: distance kept cutting past the start to erase the
    /// entry/exit witness (profile, pocket, chamfer).
    LeadOverlap,
    /// Face: top cutting plane above the drawing reference (mm).
    FaceStartOffset,
    /// Face: pass overlap as a percentage of the tool diameter.
    FaceOverlap,
    /// Face: overshoot past the stock edge before the turnaround (mm).
    FaceOvershoot,
    /// Plunge parameter A: ramp/zig-zag angle, or helix radius.
    PlungeA,
    /// Plunge parameter B: zig-zag length, or helix pitch.
    PlungeB,
}

impl Field {
    fn label(self) -> &'static str {
        match self {
            Field::Clearance => "Clearance (mm)",
            Field::Retract => "Retract (mm)",
            Field::TopOfStock => "Top of stock (mm)",
            Field::StockXOffset => "X offset (mm)",
            Field::StockYOffset => "Y offset (mm)",
            Field::StockTop => "Stock top (mm)",
            Field::StockThickness => "Thickness (mm)",
            Field::OriginX => "Origin X (mm)",
            Field::OriginY => "Origin Y (mm)",
            Field::OriginZ => "Origin Z (mm)",
            Field::StartOffX => "Offset X (mm)",
            Field::StartOffY => "Offset Y (mm)",
            Field::StartOffZ => "Offset Z (mm)",
            Field::ToolDiameter => "Tool ⌀ (mm)",
            Field::ToolLength => "Length (mm)",
            Field::Flutes => "Flutes",
            Field::CornerRadius => "Corner radius (mm)",
            Field::ChamferAngle => "Point angle (deg)",
            Field::TipDiameter => "Tip ⌀ (mm)",
            Field::PointAngle => "Point angle (deg)",
            Field::ToolThreadPitch => "Tool pitch (mm, 0=any)",
            Field::Depth => "Depth (mm)",
            Field::ProfileOffset => "Offset / leave (mm)",
            Field::Stepdown => "Stepdown (mm)",
            Field::Stepover => "Stepover (mm)",
            Field::Feed => "Feed (mm/min)",
            Field::PlungeFeed => "Plunge feed (mm/min)",
            Field::MajorDia => "Major ⌀ (mm)",
            Field::Pitch => "Pitch (mm)",
            Field::ThreadTop => "Thread top (mm)",
            Field::ThreadBottom => "Thread bottom (mm)",
            Field::ChamferWidth => "Chamfer width (mm)",
            Field::ChamferDepth => "Tip depth (mm, 0=tip)",
            Field::ChamferStep => "Step (mm, 0=one pass)",
            Field::LeadInSize => "Lead-in size (mm)",
            Field::LeadOutSize => "Lead-out size (mm)",
            Field::LeadOverlap => "Lead overlap (mm)",
            Field::FaceStartOffset => "Start offset (mm)",
            Field::FaceOverlap => "Overlap (%)",
            Field::FaceOvershoot => "Overshoot (mm)",
            Field::PlungeA => "Plunge angle/radius",
            Field::PlungeB => "Plunge length/pitch",
        }
    }
}

/// The size (length/radius) carried by a lead, or 0 for `None`.
fn lead_size(lead: Lead) -> f64 {
    match lead {
        Lead::None => 0.0,
        Lead::Linear { length } => length,
        Lead::Arc { radius } => radius,
    }
}

/// Plunge parameters as `(a, b)` for the inspector: `(angle, —)` for ramp,
/// `(radius, pitch)` for helix, `(angle, length)` for zig-zag, `(0, 0)` straight.
fn plunge_params(plunge: Plunge) -> (f64, f64) {
    match plunge {
        Plunge::Straight => (0.0, 0.0),
        Plunge::Ramp { angle_deg } => (angle_deg, 0.0),
        Plunge::Helix { radius, pitch } => (radius, pitch),
        Plunge::ZigZag { length, angle_deg } => (angle_deg, length),
    }
}

/// Set a lead's size (length/radius), keeping its kind.
fn set_lead_size(lead: Lead, size: f64) -> Lead {
    match lead {
        Lead::None => Lead::None,
        Lead::Linear { .. } => Lead::Linear { length: size },
        Lead::Arc { .. } => Lead::Arc { radius: size },
    }
}

/// Set a plunge's parameters from the inspector `(a, b)`, keeping its kind.
fn set_plunge_params(plunge: Plunge, a: f64, b: f64) -> Plunge {
    match plunge {
        Plunge::Straight => Plunge::Straight,
        Plunge::Ramp { .. } => Plunge::Ramp { angle_deg: a },
        Plunge::Helix { .. } => Plunge::Helix {
            radius: a,
            pitch: b,
        },
        Plunge::ZigZag { .. } => Plunge::ZigZag {
            angle_deg: a,
            length: b,
        },
    }
}

/// A library tool as offered in the wizard's tool picker (carries its library index
/// so a pick embeds the right entry).
#[derive(Clone, Copy, PartialEq)]
struct ToolChoice {
    index: usize,
    number: u32,
    diameter: f64,
    kind: ToolKind,
}

impl std::fmt::Display for ToolChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "T{} ⌀{} {}",
            self.number,
            fmt_num(self.diameter),
            self.kind
        )
    }
}

struct App {
    controller: AppController,
    panes: pane_grid::State<Pane>,
    /// Current window size, tracked so pane resizes can be clamped to per-pane
    /// pixel minimums (pane_grid works in ratios, not pixels).
    window: iced::Size,
    /// Fixed pixel sizes for the non-Viewport panes. Held constant across window
    /// resizes (only the Viewport absorbs the change); updated when the user drags a
    /// divider. Project / Inspector are widths, Output is a height.
    project_px: f32,
    inspector_px: f32,
    output_px: f32,
    /// The active ribbon tab.
    active_tab: RibbonTab,
    /// The index (within the active tab's groups) of the open collapse-popup, if any.
    open_group: Option<usize>,
    /// Edit buffers for the inspector fields of the current selection.
    fields: BTreeMap<Field, String>,
    /// Whether the viewport overlays the simulated stock surface.
    show_stock: bool,
    /// Whether the orientation-cube gizmo is shown (toggleable).
    show_gizmo: bool,
    /// On-screen edge length of the orientation cube, in **logical pixels** —
    /// fixed (not scaled with the window), adjustable at runtime via the View tab.
    gizmo_size: f32,
    /// The orbit-camera orientation, owned here; clicking a cube face or dragging
    /// the viewport reports changes back as messages.
    view: ViewControls,
    /// Last known cursor position over the viewport (window coords), for drawing
    /// the pickbox while a geometry pick is pending.
    cursor: Option<iced::Point>,
    /// Cross-project tool library (loaded from the config dir), picked from during
    /// op setup and managed from the Tooling tab.
    library: ToolLibrary,
    /// The library tool selected for editing in the Tooling-tab library editor.
    lib_sel: usize,
    /// The operation whose right-click context menu is open, and where to anchor it
    /// (window-absolute coords, captured from `window_cursor` at right-click time).
    open_op_menu: Option<u32>,
    op_menu_pos: iced::Point,
    /// Last known cursor position in **window-absolute** coords (from a global
    /// event subscription). Overlays are placed from the window origin, so this —
    /// not a widget-local position — is what anchors the op context menu exactly
    /// under the cursor.
    window_cursor: iced::Point,
    /// Operations whose toolpath is highlighted (vivid) in the viewport while all
    /// others are dimmed. A *set* so several ops can be lit at once — this is the
    /// viewport highlight, kept distinct from the single-item `Selection` (which
    /// drives the inspector). Empty = show every operation at full colour.
    focus_ops: BTreeSet<u32>,
    /// Live keyboard modifiers (from a global subscription), so an op click can
    /// tell a plain click (single focus) from ⌘/Ctrl-click (add/remove from set).
    modifiers: iced::keyboard::Modifiers,
    /// Enabled viewport object-snaps for operation picking (End/Mid/…); the order
    /// is irrelevant, membership is what counts.
    snaps: Vec<SnapKind>,
    /// The object-snap currently under the cursor while a pick is pending (drawn
    /// as a marker and used as the start when clicked). `None` when nothing snaps.
    snap_hover: Option<SnapHit>,
    /// The world-mm pickbox aperture from the last hover, sizing the snap marker
    /// (roughly constant on screen).
    snap_aperture: f64,
    /// The loop under the cursor while a pick is pending — highlighted so it is
    /// obvious which loop a click will select (vital for concentric circles).
    hover_loop: Option<LoopRef>,
    /// "Set origin" pick mode: a viewport click drops the workpiece datum (using
    /// object snaps for corners/centres, or a free point elsewhere).
    setting_origin: bool,
    /// Two-point origin mode: X from the 1st pick, Y from the 2nd, Z the midpoint.
    setting_origin_2pt: bool,
    /// The first point captured in two-point origin mode (awaiting the second).
    origin_first: Option<[f64; 3]>,
    /// Whether the workpiece-origin datum marker is drawn (View toggle).
    show_origin: bool,
    status: String,
}

/// The pickbox aperture, px — its half-size is the vertex-snap tolerance.
const PICKBOX_PX: f32 = 12.0;
/// The object-snap catch aperture, px — larger than the pickbox so snaps engage
/// from a comfortable distance (and the marker, sized from it, reads clearly).
/// TODO(prefs): expose this (snap distance) in user preferences — see WORKSTATE.
const SNAP_PICK_PX: f32 = 1.5 * PICKBOX_PX;
/// Snap marker size as a multiple of the snap aperture.
/// TODO(prefs): expose this (snap-shape size) in user preferences.
const SNAP_MARK_SCALE: f32 = 1.2;

/// Whether an operation kind uses a start/lead-in point, and so honours object
/// snaps. Face/Drill/Thread have no start, so snaps are inert (and hidden) there.
fn op_uses_snaps(kind: OpKind) -> bool {
    matches!(kind, OpKind::Profile | OpKind::Chamfer | OpKind::Pocket)
}

/// The axis index (0=X, 1=Y, 2=Z) a start-point offset field addresses.
fn start_axis(field: Field) -> usize {
    match field {
        Field::StartOffX => 0,
        Field::StartOffY => 1,
        _ => 2,
    }
}

/// Whether a field belongs to the start-point section (rendered there, not in the
/// generic Setup field loop).
fn is_start_field(field: Field) -> bool {
    matches!(field, Field::StartOffX | Field::StartOffY | Field::StartOffZ)
}

/// Orientation-cube on-screen size (logical px): default and the slider's range.
const GIZMO_SIZE_DEFAULT: f32 = 110.0;
const GIZMO_SIZE_MIN: f32 = 60.0;
const GIZMO_SIZE_MAX: f32 = 220.0;
/// Inset of the cube from the viewport's top-right corner (logical px).
const GIZMO_MARGIN: f32 = 8.0;

/// Highlight colour for a picked boundary loop (accent blue).
const PICK_BOUNDARY: [f32; 4] = [0.20, 0.55, 0.90, 1.0];
/// Highlight colour for an excluded island loop (gold).
const PICK_ISLAND: [f32; 4] = [0.90, 0.65, 0.10, 1.0];
/// Highlight colour for the loop under the cursor during a pick (bright yellow) —
/// shows which loop a click will select, e.g. among concentric circles.
const PICK_HOVER: [f32; 4] = [0.95, 0.92, 0.25, 1.0];
/// Object-snap marker colour (bright cyan — reads over part and stock alike).
const SNAP_MARK: [f32; 4] = [0.20, 0.95, 0.95, 1.0];
/// Workpiece-origin datum marker colour (magenta — distinct from the snaps/loops
/// and legible under red-green colour deficiency).
const ORIGIN_MARK: [f32; 4] = [0.95, 0.30, 0.85, 1.0];

/// Draw the workpiece-origin datum: a ringed crosshair at `origin` (part XY at
/// `z`), sized `r` (mm). Always shown, so the datum the G-code is referenced to
/// is visible in the viewport.
fn add_origin_marker(scene: &mut Scene, origin: [f64; 3], z: f32, r: f32) {
    let (cx, cy) = (origin[0] as f32, origin[1] as f32);
    // Cross.
    scene.add_strip(vec![[cx - r, cy, z], [cx + r, cy, z]], ORIGIN_MARK);
    scene.add_strip(vec![[cx, cy - r, z], [cx, cy + r, z]], ORIGIN_MARK);
    // Ring (octagon at 0.6·r).
    let rr = r * 0.6;
    let ring: Vec<[f32; 3]> = (0..=8)
        .map(|i| {
            let a = i as f32 / 8.0 * std::f32::consts::TAU;
            [cx + rr * a.cos(), cy + rr * a.sin(), z]
        })
        .collect();
    scene.add_strip(ring, ORIGIN_MARK);
}

/// Draw the object-snap marker at `hit.point` as a glyph specific to the snap
/// kind (AutoCAD idiom: square = End, triangle = Mid, diamond = Quadrant,
/// hourglass = Nearest), sized by the pickbox `aperture` so it reads at any zoom.
fn add_snap_marker(scene: &mut Scene, hit: SnapHit, aperture: f64, z: f32) {
    let (cx, cy) = (hit.point[0] as f32, hit.point[1] as f32);
    // A touch larger than the (already doubled) snap aperture, so the engaged
    // marker reads clearly in place of the pickbox.
    let h = (aperture as f32 * SNAP_MARK_SCALE).max(0.01);
    let p = |dx: f32, dy: f32| [cx + dx * h, cy + dy * h, z];
    let strip: Vec<[f32; 3]> = match hit.kind {
        // Square.
        SnapKind::End => vec![p(-1.0, -1.0), p(1.0, -1.0), p(1.0, 1.0), p(-1.0, 1.0), p(-1.0, -1.0)],
        // Upward triangle.
        SnapKind::Mid => vec![p(-1.0, -0.8), p(1.0, -0.8), p(0.0, 1.0), p(-1.0, -0.8)],
        // Diamond.
        SnapKind::Quadrant => vec![p(0.0, -1.2), p(1.2, 0.0), p(0.0, 1.2), p(-1.2, 0.0), p(0.0, -1.2)],
        // Hourglass (two triangles sharing the centre) drawn as one open path.
        SnapKind::Nearest => vec![
            p(-1.0, 1.0),
            p(1.0, 1.0),
            p(-1.0, -1.0),
            p(1.0, -1.0),
            p(-1.0, 1.0),
        ],
    };
    scene.add_strip(strip, SNAP_MARK);
}

/// Add a closed contour to the scene as a highlight strip in `color`.
fn add_loop_highlight(scene: &mut Scene, c: &cam_geo::Contour, color: [f32; 4]) {
    let mut strip: Vec<[f32; 3]> = c
        .points()
        .iter()
        .map(|p| [p.x as f32, p.y as f32, 0.0])
        .collect();
    if let Some(&first) = strip.first() {
        strip.push(first); // close the loop
    }
    scene.add_strip(strip, color);
}

/// The `(yaw, pitch)` that views a cube face (given by its outward normal)
/// straight on, **the right way up** — the orientation a click on that gizmo face
/// snaps to. Side and front/back views use `pitch = −90°` so world +Z is screen
/// up (not −Z, which reads upside down).
fn face_view(normal: [f32; 3]) -> (f32, f32) {
    use std::f32::consts::{FRAC_PI_2, PI};
    if normal[2] > 0.5 {
        (0.0, 0.0) // +Z top → +Y up
    } else if normal[2] < -0.5 {
        (0.0, PI) // −Z bottom
    } else if normal[1] > 0.5 {
        (PI, -FRAC_PI_2) // +Y back → +Z up
    } else if normal[1] < -0.5 {
        (0.0, -FRAC_PI_2) // −Y front → +Z up
    } else if normal[0] > 0.5 {
        (-FRAC_PI_2, -FRAC_PI_2) // +X right → +Z up
    } else {
        (FRAC_PI_2, -FRAC_PI_2) // −X left → +Z up
    }
}

#[derive(Debug, Clone)]
enum Message {
    OpenSample,
    Select(Selection),
    FieldChanged(Field, String),
    /// Commit the inspector fields (one undo step) and recompute the toolpath.
    Apply,
    Undo,
    Redo,
    // --- File I/O (native dialogs via rfd) ---
    /// Start a fresh, empty project.
    NewProject,
    /// Prompt for and open a `.ocam` project.
    OpenProject,
    /// The chosen project to open (`None` = cancelled).
    ProjectToOpen(Option<PathBuf>),
    /// Save to the current path, or prompt if there is none.
    SaveProject,
    /// Always prompt for a save location.
    SaveProjectAs,
    /// The chosen path to save to (`None` = cancelled).
    ProjectToSave(Option<PathBuf>),
    /// Prompt for and import a `.dxf`/`.dwg` file.
    ImportCad,
    /// The chosen CAD file to import (`None` = cancelled).
    CadToImport(Option<PathBuf>),
    /// Prompt for a `.nc` export location.
    ExportNc,
    /// Answer to the "export exact-duplicate operations anyway?" confirm dialog:
    /// `true` proceeds to the save prompt, `false` aborts the export.
    ExportDupConfirmed(bool),
    /// The chosen `.nc` path (`None` = cancelled).
    NcToExport(Option<PathBuf>),
    // --- Operation-creation wizard ---
    /// Begin creating an operation of `kind` (enter geometry-pick mode).
    BeginOp(OpKind),
    /// Cancel the pending operation.
    CancelOp,
    /// Confirm a pending operation (finalise a pocket boundary + islands).
    ConfirmOp,
    /// Change the selected profile's cut side.
    SideChanged(Side),
    /// Change the selected thread between internal (bore) and external (boss).
    ThreadInternalChanged(bool),
    /// Change the selected thread's hand.
    ThreadHandChanged(Hand),
    /// Change the selected thread between climb and conventional milling.
    ThreadClimbChanged(bool),
    /// A world `(x, y)` picked in the viewport plus the pickbox aperture in world
    /// mm (completes a pending operation).
    PickWorld([f32; 2], f32),
    /// While a pick is pending: the cursor's screen point (for the pickbox), its
    /// world `(x, y)`, and the pickbox aperture (world mm) — resolves and previews
    /// the object-snap under the cursor.
    HoverWorld(iced::Point, [f32; 2], f32),
    /// The cursor moved over the viewport (window coords) while a pick is pending,
    /// but no world point was available (draws the pickbox, clears any snap).
    ViewportCursor(iced::Point),
    /// Toggle a viewport object-snap on/off during operation picking.
    ToggleSnap(SnapKind),
    /// Enable/disable the program start point (Setup inspector).
    ToggleStartPoint(bool),
    /// Enter/leave single-point "set workpiece origin" pick mode (Edit tab).
    ToggleSetOrigin,
    /// Enter/leave two-point origin pick mode: X from the 1st pick, Y from the
    /// 2nd, Z the midpoint of both.
    ToggleSetOrigin2pt,
    /// Show or hide the workpiece-origin datum marker (View tab).
    ToggleShowOrigin,
    /// A viewport click while setting the origin (either mode): world `(x,y)` +
    /// aperture, resolved to a snapped or free point.
    OriginPointPicked([f32; 2], f32),
    /// Structural edits to the selected operation.
    DuplicateOp,
    DeleteOp,
    /// Move operation `id` one step earlier (`true`) or later.
    MoveOp(u32, bool),
    /// Include (`false`) or exclude (`true`) an operation from toolpath output.
    SetOpExcluded(u32, bool),
    ToggleStock,
    /// Relative camera changes reported by dragging in the viewport.
    OrbitBy(f32, f32),
    PanBy([f32; 3]),
    ZoomBy(f32),
    /// Snap to a `(yaw, pitch)` orientation (a clicked cube face), re-framing.
    SetView(f32, f32),
    /// Reset to the framed top view.
    ResetView,
    /// Show or hide the orientation cube.
    ToggleGizmo,
    /// Set the orientation cube's on-screen size (logical px).
    SetGizmoSize(f32),
    PaneResized(pane_grid::ResizeEvent),
    PaneDragged(pane_grid::DragEvent),
    /// The window was resized (tracked for pixel-accurate pane minimums).
    WindowResized(iced::Size),
    /// Change the selected tool's geometry kind (committed immediately).
    ToolKindChanged(ToolKind),
    /// Change the selected profile's lead-in / lead-out / plunge kind (committed
    /// immediately with default parameters; sizes are then edited as fields).
    LeadInKindChanged(LeadKind),
    LeadOutKindChanged(LeadKind),
    PlungeKindChanged(PlungeKind),
    /// Change the selected face op's pass direction (committed immediately).
    FaceDirectionChanged(Axis),
    /// Toggle the selected chamfer's gradual (equal-material) stepping.
    ChamferGradualToggled(bool),
    /// Create a new default tool and select it.
    NewTool,
    /// Delete the selected tool.
    DeleteTool,
    /// Switch the active ribbon tab.
    SelectRibbonTab(RibbonTab),
    /// Open/close the collapse-popup for a collapsed group (index in the active tab).
    ToggleRibbonGroup(usize),
    /// Close any open collapse-popup.
    CloseRibbonPopup,
    /// Show (`true`) or hide (`false`) a pane.
    SetPaneVisible(Pane, bool),
    /// Pick a library tool (by index) for the pending op — embeds it into the setup.
    SetPendingLibraryTool(usize),
    /// Select a library tool for editing in the Tooling-tab library editor.
    SelectLibraryTool(usize),
    /// Open the right-click context menu for operation `id` (anchored under the
    /// cursor), and select that operation.
    OpMenu(u32),
    /// Dismiss the operation context menu.
    CloseOpMenu,
    /// Track the window-absolute cursor position (global subscription) so overlays
    /// can be placed exactly under the cursor.
    WindowCursor(iced::Point),
    /// A left-click on operation `id`'s tree row. Plain click focuses it alone;
    /// ⌘/Ctrl-click toggles it in the viewport highlight set (multi-select);
    /// plain-clicking the sole focused op clears the highlight.
    ClickOp(u32),
    /// Track live keyboard modifiers (global subscription) for click semantics.
    ModifiersChanged(iced::keyboard::Modifiers),
}

fn default_machine() -> Machine {
    Machine {
        name: "desktop".into(),
        rapid_rate: 2000.0,
        max_spindle_rpm: 10_000.0,
        max_feed: 800.0,
        envelope: Envelope::new(
            Point3::new(0.0, 0.0, -50.0),
            Point3::new(300.0, 180.0, 50.0),
        ),
        safe_z: 5.0,
        tool_change_pos: None,
    }
}

/// Grab leeway (px) for the pane_grid resize dividers. Shared with the viewport
/// so it can ignore presses in the same band (see [`in_resize_band`]).
const PANE_RESIZE_LEEWAY: f32 = 8.0;

/// pane_grid's own (global) floor; the real per-pane minimums are applied in
/// [`App::clamp_resize`].
const PANE_MIN_PX: f32 = 40.0;
/// Gap between panes.
const PANE_SPACING: f32 = 4.0;
/// Approximate menu-bar height, subtracted from the window to get the grid area.
const MENU_BAR_H: f32 = 44.0;

/// The initial layout: Project | (Viewport | Inspector), with Output below.
fn initial_panes() -> pane_grid::State<Pane> {
    use pane_grid::{Axis, Configuration};
    pane_grid::State::with_configuration(Configuration::Split {
        axis: Axis::Horizontal,
        ratio: 0.78,
        a: Box::new(Configuration::Split {
            axis: Axis::Vertical,
            ratio: 0.19,
            a: Box::new(Configuration::Pane(Pane::Project)),
            b: Box::new(Configuration::Split {
                axis: Axis::Vertical,
                ratio: 0.74,
                a: Box::new(Configuration::Pane(Pane::Viewport)),
                b: Box::new(Configuration::Pane(Pane::Inspector)),
            }),
        }),
        b: Box::new(Configuration::Pane(Pane::Output)),
    })
}

impl App {
    fn new() -> (Self, iced::Task<Message>) {
        let mut app = Self {
            controller: AppController::new(default_machine()),
            panes: initial_panes(),
            window: iced::Size::new(1280.0, 800.0),
            project_px: 220.0,
            inspector_px: 250.0,
            output_px: 140.0,
            active_tab: RibbonTab::Home,
            open_group: None,
            fields: BTreeMap::new(),
            show_stock: false,
            show_gizmo: true,
            gizmo_size: GIZMO_SIZE_DEFAULT,
            view: ViewControls::default(),
            cursor: None,
            library: ToolLibrary::load(),
            lib_sel: 0,
            open_op_menu: None,
            op_menu_pos: iced::Point::ORIGIN,
            window_cursor: iced::Point::ORIGIN,
            focus_ops: BTreeSet::new(),
            modifiers: iced::keyboard::Modifiers::default(),
            // End + Mid + Quadrant on by default; Nearest is opt-in (AutoCAD-style).
            snaps: vec![SnapKind::End, SnapKind::Mid, SnapKind::Quadrant],
            snap_hover: None,
            snap_aperture: 1.0,
            hover_loop: None,
            setting_origin: false,
            setting_origin_2pt: false,
            origin_first: None,
            show_origin: true,
            status: "Open the sample part to begin.".to_string(),
        };
        app.refresh_fields();
        // Seed the fixed layout against the assumed window size; the first real
        // resize event re-applies it against the true size.
        app.apply_fixed_layout();
        (app, iced::Task::none())
    }

    fn update(&mut self, message: Message) -> iced::Task<Message> {
        // Any action other than opening a group-popup closes the open one (so a
        // command picked from a popup, or a click elsewhere, dismisses it).
        if !matches!(message, Message::ToggleRibbonGroup(_)) {
            self.open_group = None;
        }
        match message {
            Message::OpenSample => match self.controller.open_dxf(SAMPLE_DXF, "sample.dxf") {
                Ok(n) => {
                    self.focus_ops.clear();
                    self.refresh_fields();
                    self.status = format!("Imported {n} region(s).");
                    self.rerun();
                }
                Err(e) => self.status = format!("Import failed: {e}"),
            },
            Message::Select(selection) => {
                self.controller.select(selection);
                // Keep the viewport highlight in step: a single operation focuses
                // just it; any non-operation node (Setup/Stock/Tool) clears the
                // highlight so everything shows vivid again.
                self.focus_ops.clear();
                if let Selection::Operation(id) = selection {
                    self.focus_ops.insert(id);
                }
                self.refresh_fields();
            }
            Message::ClickOp(id) => {
                if self.modifiers.command() {
                    // ⌘/Ctrl-click toggles this op in the highlight set.
                    if !self.focus_ops.remove(&id) {
                        self.focus_ops.insert(id);
                    }
                } else if self.focus_ops.len() == 1 && self.focus_ops.contains(&id) {
                    // Plain-clicking the sole focused op clears the highlight.
                    self.focus_ops.clear();
                } else {
                    // Plain click focuses just this op.
                    self.focus_ops.clear();
                    self.focus_ops.insert(id);
                }
                // The inspector tracks the clicked op if it stays focused, else any
                // remaining focused op, else the setup (so edits don't target a
                // path that is no longer highlighted).
                let sel = if self.focus_ops.contains(&id) {
                    Selection::Operation(id)
                } else if let Some(&other) = self.focus_ops.iter().next_back() {
                    Selection::Operation(other)
                } else {
                    Selection::Setup
                };
                self.controller.select(sel);
                self.refresh_fields();
            }
            Message::ModifiersChanged(m) => self.modifiers = m,
            // Field edits only touch the local buffer; nothing is applied or
            // recomputed until Apply, so undo has one step per real change.
            Message::FieldChanged(field, value) => {
                self.fields.insert(field, value);
            }
            Message::Apply => self.apply_inspector(),
            Message::NewProject => {
                self.controller.new_project();
                self.focus_ops.clear();
                self.refresh_fields();
                self.status = "New project.".to_string();
            }
            Message::OpenProject => {
                return iced::Task::perform(
                    pick_open("OpenCAMStudio project", &["ocam"]),
                    Message::ProjectToOpen,
                );
            }
            Message::ProjectToOpen(Some(path)) => {
                self.status = match self.controller.open_project(&path) {
                    Ok(()) => {
                        self.focus_ops.clear();
                        self.refresh_fields();
                        self.rerun();
                        format!("Opened {}.", path.display())
                    }
                    Err(e) => format!("Open failed: {e}"),
                };
            }
            Message::SaveProject => match self.controller.current_path().map(PathBuf::from) {
                Some(path) => self.save_to(&path),
                None => {
                    return iced::Task::perform(
                        pick_save("OpenCAMStudio project", "project.ocam", &["ocam"]),
                        Message::ProjectToSave,
                    )
                }
            },
            Message::SaveProjectAs => {
                return iced::Task::perform(
                    pick_save("OpenCAMStudio project", "project.ocam", &["ocam"]),
                    Message::ProjectToSave,
                );
            }
            Message::ProjectToSave(Some(path)) => self.save_to(&path),
            Message::ImportCad => {
                return iced::Task::perform(
                    pick_open("CAD drawing", &["dxf", "dwg"]),
                    Message::CadToImport,
                );
            }
            Message::CadToImport(Some(path)) => {
                self.status = match self.controller.import_cad(&path) {
                    Ok(n) => {
                        self.focus_ops.clear();
                        self.refresh_fields();
                        self.rerun();
                        format!("Imported {n} region(s) from {}.", path.display())
                    }
                    Err(e) => format!("Import failed: {e}"),
                };
            }
            Message::ExportNc => {
                // Guardrail: if any included operations are exact duplicates, they
                // would post the same toolpath twice. Confirm before the machine
                // sees it — but don't block (a spring/finishing pass is legitimate).
                let groups = self.controller.duplicate_operation_groups();
                if groups.is_empty() {
                    return iced::Task::perform(
                        pick_save("G-code", "program.nc", &["nc"]),
                        Message::NcToExport,
                    );
                }
                return iced::Task::perform(
                    confirm_export_duplicates(describe_duplicates(&groups)),
                    Message::ExportDupConfirmed,
                );
            }
            Message::ExportDupConfirmed(true) => {
                return iced::Task::perform(
                    pick_save("G-code", "program.nc", &["nc"]),
                    Message::NcToExport,
                );
            }
            Message::ExportDupConfirmed(false) => {
                self.status =
                    "Export cancelled — exclude or edit the duplicate operation(s).".to_string();
            }
            Message::NcToExport(Some(path)) => {
                self.status = match self.controller.export_nc_to(&path) {
                    Ok(()) => format!("Exported G-code to {}.", path.display()),
                    Err(e) => format!("Export blocked: {e}"),
                };
            }
            // Cancelled dialogs — nothing to do.
            Message::ProjectToOpen(None)
            | Message::ProjectToSave(None)
            | Message::CadToImport(None)
            | Message::NcToExport(None) => {}
            Message::Undo => {
                if self.controller.undo() {
                    self.refresh_fields();
                    self.rerun();
                    self.status = format!("Undid change. {}", self.status);
                } else {
                    self.status = "Nothing to undo.".to_string();
                }
            }
            Message::Redo => {
                if self.controller.redo() {
                    self.refresh_fields();
                    self.rerun();
                    self.status = format!("Redid change. {}", self.status);
                } else {
                    self.status = "Nothing to redo.".to_string();
                }
            }
            Message::ToggleStartPoint(on) => {
                self.controller
                    .edit_start_offset(|off| *off = on.then_some([0.0, 0.0, 0.0]));
                self.refresh_fields();
            }
            Message::ToggleShowOrigin => self.show_origin = !self.show_origin,
            Message::ToggleSetOrigin => {
                let on = !self.setting_origin;
                self.setting_origin = on;
                self.setting_origin_2pt = false;
                self.origin_first = None;
                self.snap_hover = None;
                self.hover_loop = None;
                if on {
                    self.controller.cancel_operation();
                    self.controller.select(Selection::Origin);
                    self.focus_ops.clear();
                    self.refresh_fields();
                    self.status =
                        "Set origin: click a corner/centre (snaps) or any point.".to_string();
                }
            }
            Message::ToggleSetOrigin2pt => {
                let on = !self.setting_origin_2pt;
                self.setting_origin_2pt = on;
                self.setting_origin = false;
                self.origin_first = None;
                self.snap_hover = None;
                self.hover_loop = None;
                if on {
                    self.controller.cancel_operation();
                    self.controller.select(Selection::Origin);
                    self.focus_ops.clear();
                    self.refresh_fields();
                    self.status = "2-point origin: click point 1 (sets X).".to_string();
                }
            }
            Message::OriginPointPicked(w, aperture) => {
                let world = [w[0] as f64, w[1] as f64];
                // Snap to geometry if a snap catches, else take the free point; Z is
                // the pick plane (top of stock).
                let p2 = self
                    .controller
                    .snap_at(world, aperture as f64, &self.snaps)
                    .map(|h| h.point)
                    .unwrap_or(world);
                let z = self.controller.document().setup.heights.top_of_stock;
                let p = [p2[0], p2[1], z];
                self.snap_hover = None;
                self.hover_loop = None;
                if self.setting_origin {
                    // Single point: X and Y (Z stays; edit it in the fields).
                    self.controller.edit_origin(|o| {
                        o[0] = p[0];
                        o[1] = p[1];
                    });
                    self.setting_origin = false;
                    self.refresh_fields();
                    self.status = format!("Origin set to X{:.3} Y{:.3}.", p[0], p[1]);
                } else if let Some(first) = self.origin_first.take() {
                    // Two-point: X from the 1st pick, Y from the 2nd, Z the midpoint.
                    let origin = [first[0], p[1], (first[2] + p[2]) / 2.0];
                    self.controller.edit_origin(|o| *o = origin);
                    self.setting_origin_2pt = false;
                    self.refresh_fields();
                    self.status = format!(
                        "Origin set to X{:.3} Y{:.3} Z{:.3}.",
                        origin[0], origin[1], origin[2]
                    );
                } else {
                    // Two-point: first pick — store it, await the second (Y).
                    self.origin_first = Some(p);
                    self.status = "2-point origin: click point 2 (sets Y).".to_string();
                }
            }
            Message::BeginOp(kind) => {
                self.setting_origin = false;
                self.setting_origin_2pt = false;
                self.origin_first = None;
                self.controller.begin_operation(kind);
                // Seed the op with the first library tool so it always has a valid
                // tool; the user can change it in the wizard picker.
                if self.controller.pending_op().is_some() {
                    if let Some(&tool) = self.library.tools.first() {
                        let number = self.controller.use_tool(tool);
                        self.controller.set_pending_tool(number);
                    }
                }
                self.refresh_fields();
                self.status = if self.controller.pending_op().is_some() {
                    "Click a boundary line in the viewport (or Cancel in the Inspector)."
                        .to_string()
                } else {
                    "Open a part first.".to_string()
                };
            }
            Message::SetPendingLibraryTool(i) => {
                if let Some(&tool) = self.library.tools.get(i) {
                    let number = self.controller.use_tool(tool);
                    self.controller.set_pending_tool(number);
                }
            }
            Message::CancelOp => {
                self.controller.cancel_operation();
                self.controller.prune_unused_tools();
                self.status = "Cancelled operation creation.".to_string();
            }
            Message::ConfirmOp => {
                if self.controller.confirm_operation() {
                    self.cursor = None;
                    self.refresh_fields();
                    self.rerun();
                    self.status = "Operation created.".to_string();
                }
            }
            Message::PickWorld(w, aperture) => {
                // Snaps only apply to op kinds with a start; others select by the
                // nearest point on the loop.
                let snaps: &[SnapKind] = if self
                    .controller
                    .pending_op()
                    .is_some_and(|p| op_uses_snaps(p.kind))
                {
                    &self.snaps
                } else {
                    &[]
                };
                match self.controller.pick_operation_geometry(
                    [w[0] as f64, w[1] as f64],
                    aperture as f64,
                    snaps,
                ) {
                    PickResult::Created => {
                        self.cursor = None;
                        self.snap_hover = None;
                        self.hover_loop = None;
                        self.refresh_fields();
                        self.rerun();
                        self.status = "Operation created.".to_string();
                    }
                    PickResult::Selecting => {
                        let n = self.controller.pending_op().map_or(0, |p| p.islands.len());
                        self.status =
                            format!("Boundary set — click areas to exclude ({n}), then Confirm.");
                    }
                    PickResult::Missed => {
                        // Word the miss to match what is actually pickable for this
                        // op — a circular edge for drill/thread, else any boundary.
                        let circles = self
                            .controller
                            .pending_op()
                            .is_some_and(|p| op_selects_circles(p.kind));
                        self.status = if circles {
                            "No arc there — click on a circular edge.".to_string()
                        } else {
                            "No line there — click a boundary edge.".to_string()
                        };
                    }
                }
            }
            Message::ViewportCursor(p) => {
                self.cursor = Some(p);
                self.snap_hover = None;
                self.hover_loop = None;
            }
            Message::HoverWorld(screen, w, aperture) => {
                self.cursor = Some(screen);
                self.snap_aperture = aperture as f64;
                // Preview the object-snap under the cursor (drawn as a marker).
                // Active for set-origin, and for op kinds that use a start (inert
                // for Face/Drill/Thread).
                let use_snaps = self.in_origin_pick()
                    || self
                        .controller
                        .pending_op()
                        .is_some_and(|p| op_uses_snaps(p.kind));
                self.snap_hover = if use_snaps {
                    self.controller
                        .snap_at([w[0] as f64, w[1] as f64], aperture as f64, &self.snaps)
                } else {
                    None
                };
                // Highlight the loop a click would select (op picks only —
                // disambiguates concentric circles; drill/thread → circles only).
                self.hover_loop = if self.in_origin_pick() {
                    None
                } else {
                    let circles = self
                        .controller
                        .pending_op()
                        .is_some_and(|p| op_selects_circles(p.kind));
                    self.snap_hover.map(|h| h.loop_ref).or_else(|| {
                        self.controller
                            .nearest_loop_point([w[0] as f64, w[1] as f64], aperture as f64, circles)
                            .map(|(l, _)| l)
                    })
                };
            }
            Message::ToggleSnap(kind) => {
                if let Some(pos) = self.snaps.iter().position(|k| *k == kind) {
                    self.snaps.remove(pos);
                } else {
                    self.snaps.push(kind);
                }
            }
            Message::DuplicateOp => {
                self.open_op_menu = None;
                self.controller.duplicate_selected_operation();
                self.focus_selected_op();
                self.refresh_fields();
                self.rerun();
            }
            Message::SetOpExcluded(id, excluded) => {
                self.controller.set_operation_excluded(id, excluded);
                self.rerun();
            }
            Message::DeleteOp => {
                self.open_op_menu = None;
                self.controller.delete_selected_operation();
                // A tool no longer used by any op drops out of the setup.
                self.controller.prune_unused_tools();
                self.focus_selected_op();
                self.refresh_fields();
                self.rerun();
            }
            Message::MoveOp(id, up) => {
                self.controller.move_operation(id, up);
                self.rerun();
            }
            Message::ToggleStock => {
                self.show_stock = !self.show_stock;
                self.status = if self.show_stock {
                    "Showing simulated stock.".to_string()
                } else {
                    "Hiding simulated stock.".to_string()
                };
            }
            Message::OrbitBy(dyaw, dpitch) => {
                // Turntable: horizontal drag spins about world up, vertical tilts.
                // Unclamped, so pitch can go all the way round to the underside.
                self.view.yaw += dyaw;
                self.view.pitch += dpitch;
            }
            Message::PanBy(delta) => {
                for (p, d) in self.view.pan.iter_mut().zip(delta) {
                    *p += d;
                }
            }
            Message::ZoomBy(dz) => {
                self.view.zoom = (self.view.zoom + dz).clamp(-4.0, 6.0);
            }
            Message::SetView(yaw, pitch) => {
                self.view = ViewControls {
                    yaw,
                    pitch,
                    zoom: 0.0,
                    pan: [0.0; 3],
                };
            }
            Message::ResetView => self.view = ViewControls::default(),
            Message::ToggleGizmo => self.show_gizmo = !self.show_gizmo,
            Message::SetGizmoSize(v) => {
                self.gizmo_size = v.clamp(GIZMO_SIZE_MIN, GIZMO_SIZE_MAX)
            }
            Message::PaneResized(pane_grid::ResizeEvent { split, ratio }) => {
                let ratio = self.clamp_resize(split, ratio);
                self.panes.resize(split, ratio);
                // Persist the dragged size so a later window resize keeps it.
                self.capture_side_px(split, ratio);
            }
            Message::PaneDragged(pane_grid::DragEvent::Dropped { pane, target }) => {
                self.panes.drop(pane, target);
            }
            Message::PaneDragged(_) => {}
            Message::WindowResized(size) => {
                self.window = size;
                // Hold the side panes + Output at their fixed pixel sizes; the
                // Viewport absorbs the change.
                self.apply_fixed_layout();
            }
            Message::ToolKindChanged(kind) => {
                // Route to whichever tool the inspector is editing: the library
                // entry (Tooling tab) or the selected setup tool. Switching kind
                // resets its parameters to the new kind's defaults.
                if self.library_mode() {
                    if let Some(t) = self.library.tools.get_mut(self.lib_sel) {
                        t.kind = kind;
                        self.library.save();
                    }
                } else if let Selection::Tool(i) = self.controller.selection() {
                    self.controller.edit_tool(i, |t| t.kind = kind);
                    self.rerun();
                }
                // The kind-specific fields depend on the kind — repopulate them.
                self.refresh_fields();
            }
            Message::LeadInKindChanged(kind) => {
                self.controller.edit_selected_operation(|op| match op {
                    Operation::Profile(p) => p.lead_in = kind.to_lead(p.lead_in),
                    Operation::Chamfer(c) => c.lead_in = kind.to_lead(c.lead_in),
                    _ => {}
                });
                self.refresh_fields();
                self.rerun();
            }
            Message::LeadOutKindChanged(kind) => {
                self.controller.edit_selected_operation(|op| match op {
                    Operation::Profile(p) => p.lead_out = kind.to_lead(p.lead_out),
                    Operation::Chamfer(c) => c.lead_out = kind.to_lead(c.lead_out),
                    _ => {}
                });
                self.refresh_fields();
                self.rerun();
            }
            Message::PlungeKindChanged(kind) => {
                self.controller.edit_selected_operation(|op| match op {
                    Operation::Profile(p) => p.plunge = kind.to_plunge(),
                    Operation::Pocket(p) => p.plunge = kind.to_plunge(),
                    _ => {}
                });
                self.refresh_fields();
                self.rerun();
            }
            Message::FaceDirectionChanged(axis) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Face(f) = op {
                        f.direction = axis;
                    }
                });
                self.refresh_fields();
                self.rerun();
            }
            Message::ChamferGradualToggled(on) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Chamfer(c) = op {
                        c.gradual = on;
                    }
                });
                self.refresh_fields();
                self.rerun();
            }
            Message::SideChanged(side) => {
                self.controller.edit_selected_operation(|op| match op {
                    Operation::Profile(p) => p.side = side,
                    Operation::Chamfer(c) => c.side = side,
                    _ => {}
                });
                self.rerun();
            }
            Message::ThreadInternalChanged(internal) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Thread(t) = op {
                        t.internal = internal;
                    }
                });
                self.rerun();
            }
            Message::ThreadHandChanged(hand) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Thread(t) = op {
                        t.hand = hand;
                    }
                });
                self.rerun();
            }
            Message::ThreadClimbChanged(climb) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Thread(t) = op {
                        t.climb = climb;
                    }
                });
                self.rerun();
            }
            Message::NewTool => {
                // Add a tool to the library and select it for editing. If an op
                // wizard is active, also embed it and pick it for the pending op.
                self.lib_sel = self.library.add_default();
                self.library.save();
                if self.controller.pending_op().is_some() {
                    if let Some(&tool) = self.library.tools.get(self.lib_sel) {
                        let number = self.controller.use_tool(tool);
                        self.controller.set_pending_tool(number);
                    }
                }
                self.refresh_fields();
            }
            Message::DeleteTool => {
                if self.lib_sel < self.library.tools.len() && self.library.tools.len() > 1 {
                    self.library.tools.remove(self.lib_sel);
                    self.lib_sel = self.lib_sel.min(self.library.tools.len() - 1);
                    self.library.save();
                    self.refresh_fields();
                }
            }
            Message::SelectLibraryTool(i) => {
                self.lib_sel = i;
                self.refresh_fields();
            }
            Message::OpMenu(id) => {
                self.controller.select(Selection::Operation(id));
                self.focus_ops.clear();
                self.focus_ops.insert(id);
                self.refresh_fields();
                self.open_op_menu = Some(id);
                self.op_menu_pos = self.window_cursor;
            }
            Message::CloseOpMenu => self.open_op_menu = None,
            Message::WindowCursor(p) => self.window_cursor = p,
            Message::SelectRibbonTab(tab) => {
                self.active_tab = tab;
                // The Tooling tab turns the Inspector into the library editor, so the
                // field buffers must reload for the new context.
                self.refresh_fields();
            }
            Message::ToggleRibbonGroup(i) => {
                self.open_group = if self.open_group == Some(i) {
                    None
                } else {
                    Some(i)
                };
            }
            // The preamble already cleared it; the arm keeps the match exhaustive.
            Message::CloseRibbonPopup => {}
            Message::SetPaneVisible(pane, show) => self.set_pane_visible(pane, show),
        }
        iced::Task::none()
    }

    fn subscription(&self) -> iced::Subscription<Message> {
        iced::Subscription::batch([
            iced::window::resize_events().map(|(_id, size)| Message::WindowResized(size)),
            // Track the window-absolute cursor so overlays (the op context menu)
            // anchor exactly under the pointer — widget-local positions are offset
            // by the pane's origin, which is what caused the menu to appear astray.
            iced::event::listen_with(|event, _status, _window| match event {
                iced::Event::Mouse(iced::mouse::Event::CursorMoved { position }) => {
                    Some(Message::WindowCursor(position))
                }
                // Live modifiers, so an op-row click can tell plain from ⌘/Ctrl.
                iced::Event::Keyboard(iced::keyboard::Event::ModifiersChanged(m)) => {
                    Some(Message::ModifiersChanged(m))
                }
                _ => None,
            }),
        ])
    }

    /// Clamp a split's new `ratio` so neither side falls below its panes' pixel
    /// minimums (pane_grid only offers one global floor, so we do it per pane).
    fn clamp_resize(&self, split: pane_grid::Split, ratio: f32) -> f32 {
        let bounds = iced::Size::new(
            self.window.width,
            (self.window.height - MENU_BAR_H).max(1.0),
        );
        let regions = self
            .panes
            .layout()
            .split_regions(PANE_SPACING, PANE_MIN_PX, bounds);
        let Some(&(axis, rect, _)) = regions.get(&split) else {
            return ratio;
        };
        let dim = match axis {
            pane_grid::Axis::Vertical => rect.width,
            pane_grid::Axis::Horizontal => rect.height,
        };
        let Some((a, b)) = find_split(self.panes.layout(), split) else {
            return ratio;
        };
        if dim <= 1.0 {
            return ratio;
        }
        let lo = (subtree_min(a, &self.panes, axis) / dim).clamp(0.0, 0.95);
        let hi = (1.0 - subtree_min(b, &self.panes, axis) / dim).clamp(lo, 1.0);
        ratio.clamp(lo, hi)
    }

    /// The grid handle of `pane`, if it's currently shown.
    fn pane_handle(&self, pane: Pane) -> Option<pane_grid::Pane> {
        self.panes
            .iter()
            .find(|(_, p)| **p == pane)
            .map(|(h, _)| *h)
    }

    /// Show or hide a pane. The Viewport is always visible (no-op). Hiding closes a
    /// pane; showing splits it back off an existing pane and docks it to its fixed
    /// edge. Either way the fixed layout is re-derived so the remaining side panes
    /// keep their sizes and the Viewport takes the rest.
    fn set_pane_visible(&mut self, pane: Pane, show: bool) {
        if pane == Pane::Viewport {
            return;
        }
        match (show, self.pane_handle(pane)) {
            (false, Some(handle)) => {
                self.panes.close(handle);
            }
            (true, None) => {
                let anchor = self.panes.iter().next().map(|(h, _)| *h);
                if let Some(anchor) = anchor {
                    if let Some((new_pane, _)) =
                        self.panes.split(pane_grid::Axis::Vertical, anchor, pane)
                    {
                        if let Some(edge) = pane.dock_edge() {
                            self.panes.move_to_edge(new_pane, edge);
                        }
                    }
                }
            }
            _ => {}
        }
        self.apply_fixed_layout();
    }

    /// Resize the split that bounds `pane` so `pane` occupies `px` pixels along its
    /// split axis, handing the remainder to its sibling (the Viewport side). The
    /// pixel math mirrors `maximize_pane`; `px` is floored at the pane's minimum.
    fn set_pane_px(&mut self, pane: Pane, px: f32) {
        let Some(handle) = self.pane_handle(pane) else {
            return;
        };
        let Some(path) = splits_to_pane(self.panes.layout(), handle) else {
            return;
        };
        // The pane's own boundary is the deepest split on its path.
        let Some(&(split, in_a)) = path.last() else {
            return;
        };
        let bounds = iced::Size::new(
            self.window.width,
            (self.window.height - MENU_BAR_H).max(1.0),
        );
        let regions = self
            .panes
            .layout()
            .split_regions(PANE_SPACING, PANE_MIN_PX, bounds);
        let Some(&(axis, rect, _)) = regions.get(&split) else {
            return;
        };
        let dim = match axis {
            pane_grid::Axis::Vertical => rect.width,
            pane_grid::Axis::Horizontal => rect.height,
        };
        if dim <= 1.0 {
            return;
        }
        let frac = (px.max(pane.min_size()) / dim).clamp(0.0, 1.0);
        let ratio = if in_a { frac } else { 1.0 - frac };
        let ratio = self.clamp_resize(split, ratio);
        self.panes.resize(split, ratio);
    }

    /// Hold the non-Viewport panes at their fixed pixel sizes, letting the Viewport
    /// absorb the rest. Outermost split first so a parent's dimension is settled
    /// before its children read it: Output (a height) and Project (a width) are
    /// independent, while Inspector's region width depends on Project — so it is set
    /// last. Hidden panes are simply skipped (`set_pane_px` early-returns).
    fn apply_fixed_layout(&mut self) {
        self.set_pane_px(Pane::Output, self.output_px);
        self.set_pane_px(Pane::Project, self.project_px);
        self.set_pane_px(Pane::Inspector, self.inspector_px);
    }

    /// After a manual divider drag, store the affected pane's new pixel size so a
    /// later window resize preserves it. `split`/`ratio` are the just-applied resize.
    fn capture_side_px(&mut self, split: pane_grid::Split, ratio: f32) {
        let bounds = iced::Size::new(
            self.window.width,
            (self.window.height - MENU_BAR_H).max(1.0),
        );
        let regions = self
            .panes
            .layout()
            .split_regions(PANE_SPACING, PANE_MIN_PX, bounds);
        let Some(&(axis, rect, _)) = regions.get(&split) else {
            return;
        };
        let dim = match axis {
            pane_grid::Axis::Vertical => rect.width,
            pane_grid::Axis::Horizontal => rect.height,
        };
        if dim <= 1.0 {
            return;
        }
        for pane in [Pane::Project, Pane::Inspector, Pane::Output] {
            let Some(handle) = self.pane_handle(pane) else {
                continue;
            };
            let Some(path) = splits_to_pane(self.panes.layout(), handle) else {
                continue;
            };
            let Some(&(pane_split, in_a)) = path.last() else {
                continue;
            };
            if pane_split != split {
                continue;
            }
            let px = if in_a {
                ratio * dim
            } else {
                (1.0 - ratio) * dim
            };
            match pane {
                Pane::Project => self.project_px = px,
                Pane::Inspector => self.inspector_px = px,
                Pane::Output => self.output_px = px,
                Pane::Viewport => {}
            }
        }
    }

    /// Reload the inspector edit buffers from the model for the current
    /// selection.
    fn refresh_fields(&mut self) {
        self.fields.clear();
        for field in self.inspector_fields() {
            if let Some(value) = self.field_value(field) {
                self.fields.insert(field, fmt_num(value));
            }
        }
    }

    /// Whether either origin-pick mode (single or two-point) is active.
    fn in_origin_pick(&self) -> bool {
        self.setting_origin || self.setting_origin_2pt
    }

    /// Reset the viewport highlight to the controller's current single selection
    /// (an operation, or nothing). Used after structural edits (duplicate/delete)
    /// that move the selection, so the highlight set never keeps stale ids.
    fn focus_selected_op(&mut self) {
        self.focus_ops.clear();
        if let Selection::Operation(id) = self.controller.selection() {
            self.focus_ops.insert(id);
        }
    }

    /// Whether the Inspector is in tool-library editing mode (driven by the Tooling
    /// ribbon tab, independent of the project selection).
    fn library_mode(&self) -> bool {
        self.active_tab == RibbonTab::Tooling
    }

    /// Which fields the inspector shows for the current selection.
    fn inspector_fields(&self) -> Vec<Field> {
        // Common tool fields plus the selected tool's kind-specific parameters.
        let tool_fields = |kind: Option<ToolKind>| {
            let mut f = vec![Field::ToolDiameter, Field::ToolLength, Field::Flutes];
            if let Some(k) = kind {
                f.extend(tool_kind_fields(k));
            }
            f
        };
        if self.library_mode() {
            return tool_fields(self.library.tools.get(self.lib_sel).map(|t| t.kind));
        }
        match self.controller.selection() {
            Selection::Setup => vec![Field::Clearance, Field::Retract, Field::TopOfStock],
            Selection::Origin => {
                let mut f = vec![Field::OriginX, Field::OriginY, Field::OriginZ];
                // Start-point offset fields, only when enabled. Rendered in the
                // start-point section, not the generic field loop.
                if self.controller.document().setup.start_offset.is_some() {
                    f.extend([Field::StartOffX, Field::StartOffY, Field::StartOffZ]);
                }
                f
            }
            Selection::Tool(i) => {
                tool_fields(self.controller.document().setup.tools.get(i).map(|t| t.kind))
            }
            Selection::Stock => vec![
                Field::StockXOffset,
                Field::StockYOffset,
                Field::StockTop,
                Field::StockThickness,
            ],
            Selection::Operation(id) => match self.controller.operation(id) {
                Some(Operation::Profile(p)) => {
                    let mut fields = vec![
                        Field::Depth,
                        Field::Stepdown,
                        Field::ProfileOffset,
                        Field::Feed,
                        Field::PlungeFeed,
                    ];
                    // Lead/plunge sizes appear only when the kind uses them.
                    if p.lead_in != Lead::None {
                        fields.push(Field::LeadInSize);
                    }
                    if p.lead_out != Lead::None {
                        fields.push(Field::LeadOutSize);
                    }
                    fields.push(Field::LeadOverlap);
                    match p.plunge {
                        Plunge::Straight => {}
                        Plunge::Ramp { .. } => fields.push(Field::PlungeA),
                        Plunge::Helix { .. } | Plunge::ZigZag { .. } => {
                            fields.push(Field::PlungeA);
                            fields.push(Field::PlungeB);
                        }
                    }
                    fields
                }
                Some(Operation::Pocket(p)) => {
                    let mut fields = vec![
                        Field::Depth,
                        Field::Stepdown,
                        Field::Stepover,
                        Field::Feed,
                        Field::PlungeFeed,
                        Field::LeadOverlap,
                    ];
                    match p.plunge {
                        Plunge::Straight => {}
                        Plunge::Ramp { .. } => fields.push(Field::PlungeA),
                        Plunge::Helix { .. } | Plunge::ZigZag { .. } => {
                            fields.push(Field::PlungeA);
                            fields.push(Field::PlungeB);
                        }
                    }
                    fields
                }
                Some(Operation::Face(_)) => vec![
                    Field::FaceStartOffset,
                    Field::Depth,
                    Field::Stepdown,
                    Field::FaceOverlap,
                    Field::FaceOvershoot,
                    Field::Feed,
                    Field::PlungeFeed,
                ],
                Some(Operation::Drill(_)) => vec![Field::Depth, Field::Feed],
                Some(Operation::Chamfer(c)) => {
                    let mut fields = vec![
                        Field::ChamferWidth,
                        Field::ChamferDepth,
                        Field::ChamferStep,
                        Field::Feed,
                        Field::PlungeFeed,
                    ];
                    if c.lead_in != Lead::None {
                        fields.push(Field::LeadInSize);
                    }
                    if c.lead_out != Lead::None {
                        fields.push(Field::LeadOutSize);
                    }
                    fields.push(Field::LeadOverlap);
                    fields
                }
                Some(Operation::Thread(_)) => vec![
                    Field::MajorDia,
                    Field::Pitch,
                    Field::ThreadTop,
                    Field::ThreadBottom,
                    Field::Feed,
                    Field::PlungeFeed,
                ],
                None => Vec::new(),
            },
        }
    }

    /// The model value backing a field for the current selection, if any.
    fn field_value(&self, field: Field) -> Option<f64> {
        if self.library_mode() {
            let t = self.library.tools.get(self.lib_sel)?;
            return match field {
                Field::ToolDiameter => Some(t.diameter),
                Field::ToolLength => Some(t.length),
                Field::Flutes => Some(t.flutes as f64),
                _ => tool_kind_field(t.kind, field),
            };
        }
        let setup = &self.controller.document().setup;
        match field {
            Field::Clearance => Some(setup.heights.clearance),
            Field::Retract => Some(setup.heights.retract),
            Field::TopOfStock => Some(setup.heights.top_of_stock),
            Field::OriginX => Some(setup.origin[0]),
            Field::OriginY => Some(setup.origin[1]),
            Field::OriginZ => Some(setup.origin[2]),
            Field::StartOffX | Field::StartOffY | Field::StartOffZ => {
                setup.start_offset.map(|off| off[start_axis(field)])
            }
            Field::StockXOffset | Field::StockYOffset | Field::StockTop | Field::StockThickness => {
                let cam_model::Stock::BoundingBox {
                    x_offset,
                    y_offset,
                    top,
                    thickness,
                } = setup.stock;
                Some(match field {
                    Field::StockXOffset => x_offset,
                    Field::StockYOffset => y_offset,
                    Field::StockTop => top,
                    _ => thickness,
                })
            }
            Field::ToolDiameter => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.diameter),
                _ => None,
            },
            Field::ToolLength => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.length),
                _ => None,
            },
            Field::Flutes => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.flutes as f64),
                _ => None,
            },
            _ => match self.controller.selection() {
                // Kind-specific tool parameters (corner radius, chamfer angle, …).
                Selection::Tool(i) => {
                    setup.tools.get(i).and_then(|t| tool_kind_field(t.kind, field))
                }
                _ => self
                    .controller
                    .selected_operation()
                    .and_then(|op| op_field(op, field)),
            },
        }
    }

    /// Parse the inspector buffers and commit them to the selected node as one
    /// undoable change, then recompute.
    fn apply_inspector(&mut self) {
        let mut parsed: BTreeMap<Field, f64> = BTreeMap::new();
        for (&field, text) in &self.fields {
            match text.trim().parse::<f64>() {
                Ok(v) => {
                    parsed.insert(field, v);
                }
                Err(_) => {
                    self.status = format!("{} is not a valid number.", field.label());
                    return;
                }
            }
        }

        // Library-tool editing writes to the library file, not the project — an
        // embedded copy in a project is a snapshot and is left untouched.
        if self.library_mode() {
            if let Some(t) = self.library.tools.get_mut(self.lib_sel) {
                if let Some(&v) = parsed.get(&Field::ToolDiameter) {
                    t.diameter = v;
                }
                if let Some(&v) = parsed.get(&Field::ToolLength) {
                    t.length = v;
                }
                if let Some(&v) = parsed.get(&Field::Flutes) {
                    t.flutes = v.round().max(1.0) as u32;
                }
                apply_tool_kind_fields(&mut t.kind, &parsed);
            }
            self.library.save();
            self.refresh_fields();
            return;
        }

        match self.controller.selection() {
            Selection::Setup => self.controller.edit_heights(|h| {
                if let Some(&v) = parsed.get(&Field::Clearance) {
                    h.clearance = v;
                }
                if let Some(&v) = parsed.get(&Field::Retract) {
                    h.retract = v;
                }
                if let Some(&v) = parsed.get(&Field::TopOfStock) {
                    h.top_of_stock = v;
                }
            }),
            Selection::Origin => {
                self.controller.edit_origin(|o| {
                    if let Some(&v) = parsed.get(&Field::OriginX) {
                        o[0] = v;
                    }
                    if let Some(&v) = parsed.get(&Field::OriginY) {
                        o[1] = v;
                    }
                    if let Some(&v) = parsed.get(&Field::OriginZ) {
                        o[2] = v;
                    }
                });
                self.controller.edit_start_offset(|start| {
                    if let Some(off) = start {
                        for (f, i) in [
                            (Field::StartOffX, 0),
                            (Field::StartOffY, 1),
                            (Field::StartOffZ, 2),
                        ] {
                            if let Some(&v) = parsed.get(&f) {
                                off[i] = v;
                            }
                        }
                    }
                });
            }
            Selection::Tool(i) => self.controller.edit_tool(i, |t| {
                if let Some(&v) = parsed.get(&Field::ToolDiameter) {
                    t.diameter = v;
                }
                if let Some(&v) = parsed.get(&Field::ToolLength) {
                    t.length = v;
                }
                if let Some(&v) = parsed.get(&Field::Flutes) {
                    // Flutes is an integer count; round the typed value.
                    t.flutes = v.round().max(1.0) as u32;
                }
                apply_tool_kind_fields(&mut t.kind, &parsed);
            }),
            Selection::Operation(_) => self
                .controller
                .edit_selected_operation(|op| apply_op_fields(op, &parsed)),
            Selection::Stock => self.controller.edit_stock(|stock| {
                let cam_model::Stock::BoundingBox {
                    x_offset,
                    y_offset,
                    top,
                    thickness,
                } = stock;
                if let Some(&v) = parsed.get(&Field::StockXOffset) {
                    *x_offset = v.max(0.0);
                }
                if let Some(&v) = parsed.get(&Field::StockYOffset) {
                    *y_offset = v.max(0.0);
                }
                if let Some(&v) = parsed.get(&Field::StockTop) {
                    *top = v;
                }
                if let Some(&v) = parsed.get(&Field::StockThickness) {
                    *thickness = v.max(0.0);
                }
            }),
        }
        self.refresh_fields();
        self.rerun();
    }

    /// Recompute the toolpath for the current document and report the result.
    fn rerun(&mut self) {
        if self.controller.document().setup.operations.is_empty() {
            self.status = "Open a part first.".to_string();
            return;
        }
        let outcome = self.controller.run(&CancelToken::new());
        let errors = outcome
            .diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Error)
            .count();
        self.status = if errors > 0 {
            format!("{errors} error(s) — see Output.")
        } else {
            "Toolpath ready. Export when you like.".to_string()
        };
    }

    /// Save the project to `path` and report the result.
    fn save_to(&mut self, path: &Path) {
        self.status = match self.controller.save_project(path) {
            Ok(()) => format!("Saved {}.", path.display()),
            Err(e) => format!("Save failed: {e}"),
        };
    }

    fn view(&self) -> Element<'_, Message> {
        let grid = PaneGrid::new(&self.panes, |_id, pane, _is_max| {
            pane_grid::Content::new(self.pane_content(*pane))
                .title_bar(pane_grid::TitleBar::new(text(pane.name()).size(13)).padding(4))
        })
        .spacing(4)
        .min_size(PANE_MIN_PX)
        .on_resize(PANE_RESIZE_LEEWAY, Message::PaneResized)
        .on_drag(Message::PaneDragged)
        .width(Length::Fill)
        .height(Length::Fill);

        let mut layers = iced::widget::stack![column![self.ribbon(), grid]];
        if let Some(popup) = self.ribbon_popup() {
            // A full-window catcher under the popup so a click off it dismisses.
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseRibbonPopup);
            layers = layers.push(catcher).push(popup);
        }
        if let Some(menu) = self.op_menu_overlay() {
            // A full-window catcher under the menu so a click off it dismisses.
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseOpMenu);
            layers = layers.push(catcher).push(menu);
        }
        if let Some(pickbox) = self.pickbox_overlay() {
            layers = layers.push(pickbox);
        }
        layers.into()
    }

    /// The operation right-click context menu (Delete / Duplicate), its top-left
    /// anchored exactly under the cursor. `None` unless a menu is open. Reuses the
    /// ribbon-popup overlay pattern: positioned in the top-level view stack over a
    /// click-off catcher.
    fn op_menu_overlay(&self) -> Option<Element<'_, Message>> {
        self.open_op_menu?;
        let item = |icon: Icon, label: &str, msg: Message| {
            button(
                row![icon_svg(icon, 14.0), text(label.to_string()).size(13)]
                    .spacing(6)
                    .align_y(Alignment::Center),
            )
            .width(Length::Fixed(130.0))
            .padding(Padding::from([4.0, 8.0]))
            .on_press(msg)
            .style(|_theme, status| command_button_style(status))
        };
        let menu = container(
            column![
                item(Icon::Delete, "Delete", Message::DeleteOp),
                item(Icon::Duplicate, "Duplicate", Message::DuplicateOp),
            ]
            .spacing(2),
        )
        .padding(4)
        .style(|_theme| container::Style {
            background: Some(Background::Color(palette::RIBBON_BG)),
            border: Border {
                color: palette::BORDER_DARK,
                width: 1.0,
                radius: 3.0.into(),
            },
            ..container::Style::default()
        });
        let positioned = container(menu)
            .width(Length::Fill)
            .height(Length::Fill)
            .align_x(iced::alignment::Horizontal::Left)
            .align_y(iced::alignment::Vertical::Top)
            .padding(Padding {
                top: self.op_menu_pos.y,
                left: self.op_menu_pos.x,
                right: 0.0,
                bottom: 0.0,
            });
        Some(positioned.into())
    }

    /// The pickbox: a small accent square drawn over the cursor while a geometry
    /// pick is pending, its half-size the vertex-snap aperture. Purely visual — it
    /// does not intercept clicks (they fall through to the viewport).
    fn pickbox_overlay(&self) -> Option<Element<'_, Message>> {
        if self.controller.pending_op().is_none() && !self.in_origin_pick() {
            return None;
        }
        // Once a snap engages, its (bolder) in-scene marker stands in for the
        // pickbox — so the blue aperture square shows only while nothing snaps.
        if self.snap_hover.is_some() {
            return None;
        }
        let c = self.cursor?;
        let half = PICKBOX_PX / 2.0;
        let square = container(Space::new())
            .width(Length::Fixed(PICKBOX_PX))
            .height(Length::Fixed(PICKBOX_PX))
            .style(|_theme| container::Style {
                border: Border {
                    color: palette::ACCENT_BLUE,
                    width: 1.5,
                    radius: 0.0.into(),
                },
                ..container::Style::default()
            });
        let positioned = container(square)
            .width(Length::Fill)
            .height(Length::Fill)
            .padding(Padding {
                top: (c.y - half).max(0.0),
                left: (c.x - half).max(0.0),
                right: 0.0,
                bottom: 0.0,
            });
        Some(positioned.into())
    }

    /// The top-bar ribbon: a tab strip (darker bar, active tab in accent) over a
    /// framed band showing the active tab's grouped commands. Styled after
    /// OpenCADStudio's ribbon (see the [`palette`] module).
    fn ribbon(&self) -> Element<'_, Message> {
        let tab_btn = |tab: RibbonTab| {
            let active = tab == self.active_tab;
            button(text(tab.label()).size(12))
                .padding(Padding::from([5.0, 14.0]))
                .on_press(Message::SelectRibbonTab(tab))
                .style(move |_theme, status| tab_button_style(active, status))
        };
        let mut tabs = row![].spacing(6);
        for tab in RibbonTab::ALL {
            tabs = tabs.push(tab_btn(tab));
        }
        let strip = container(tabs.padding(2).align_y(Alignment::Center))
            .width(Length::Fill)
            .style(|_theme| container::Style {
                background: Some(Background::Color(palette::TOPBAR_BG)),
                ..container::Style::default()
            });

        let band = container(self.ribbon_body())
            .padding(6)
            .width(Length::Fill)
            .style(|_theme| container::Style {
                background: Some(Background::Color(palette::RIBBON_BG)),
                border: Border {
                    color: palette::BORDER_DARK,
                    width: 1.0,
                    radius: 0.0.into(),
                },
                ..container::Style::default()
            });

        column![strip, band].into()
    }

    /// The command groups for the active tab, or `None` for the Windows tab (which
    /// is a pane-toggle list, not icon commands).
    fn ribbon_specs(&self) -> Option<Vec<GroupSpec>> {
        let has_geo = !self.controller.regions().is_empty();
        // A new op needs geometry to pick; its tool is drawn from the library, so the
        // library must have at least one tool (it seeds defaults, so it always does).
        let can_create = has_geo && !self.library.tools.is_empty();
        let begin = |kind: OpKind| can_create.then_some(Message::BeginOp(kind));
        let specs = match self.active_tab {
            RibbonTab::Home => vec![
                GroupSpec {
                    title: "Project",
                    commands: vec![
                        cmd(Icon::New, "New", Some(Message::NewProject)),
                        cmd(Icon::Open, "Open", Some(Message::OpenProject)),
                        cmd(Icon::Save, "Save", Some(Message::SaveProject)),
                        cmd(Icon::Save, "Save As", Some(Message::SaveProjectAs)),
                    ],
                },
                GroupSpec {
                    title: "Data",
                    commands: vec![
                        cmd(Icon::Import, "Import", Some(Message::ImportCad)),
                        cmd(Icon::Export, "Export", Some(Message::ExportNc)),
                        cmd(Icon::Open, "Sample", Some(Message::OpenSample)),
                    ],
                },
                GroupSpec {
                    title: "Edit",
                    commands: vec![
                        cmd(Icon::Undo, "Undo", Some(Message::Undo)),
                        cmd(Icon::Redo, "Redo", Some(Message::Redo)),
                        cmd(Icon::Run, "Run", Some(Message::Apply)),
                    ],
                },
            ],
            RibbonTab::Operations => vec![GroupSpec {
                title: "Create",
                commands: vec![
                    cmd(Icon::Profile, "Profile", begin(OpKind::Profile)),
                    cmd(Icon::Pocket, "Pocket", begin(OpKind::Pocket)),
                    cmd(Icon::Drill, "Drill", begin(OpKind::Drill)),
                    cmd(Icon::Thread, "Thread", begin(OpKind::Thread)),
                    cmd(Icon::Chamfer, "Chamfer", begin(OpKind::Chamfer)),
                    cmd(Icon::Face, "Face", begin(OpKind::Face)),
                ],
            }],
            RibbonTab::Edit => vec![GroupSpec {
                title: "Workpiece",
                commands: vec![
                    toggle_cmd(
                        Icon::SetOrigin,
                        "Set Origin",
                        self.setting_origin,
                        Message::ToggleSetOrigin,
                    ),
                    toggle_cmd(
                        Icon::SetOrigin,
                        "Origin 2-pt",
                        self.setting_origin_2pt,
                        Message::ToggleSetOrigin2pt,
                    ),
                ],
            }],
            RibbonTab::Tooling => vec![GroupSpec {
                title: "Library",
                commands: vec![
                    cmd(Icon::NewTool, "New", Some(Message::NewTool)),
                    cmd(
                        Icon::Delete,
                        "Delete",
                        // Keep at least one tool in the library.
                        (self.library.tools.len() > 1).then_some(Message::DeleteTool),
                    ),
                ],
            }],
            RibbonTab::View => vec![GroupSpec {
                title: "View",
                commands: vec![
                    toggle_cmd(
                        Icon::ShowStock,
                        "Stock",
                        self.show_stock,
                        Message::ToggleStock,
                    ),
                    cmd(Icon::ResetView, "Reset", Some(Message::ResetView)),
                    toggle_cmd(
                        Icon::ShowCube,
                        "Cube",
                        self.show_gizmo,
                        Message::ToggleGizmo,
                    ),
                    toggle_cmd(
                        Icon::SetOrigin,
                        "Origin",
                        self.show_origin,
                        Message::ToggleShowOrigin,
                    ),
                ],
            }],
            RibbonTab::Windows => return None,
        };
        Some(specs)
    }

    /// The densities the active tab's groups should render at, given the window
    /// width. Empty for the Windows tab.
    fn ribbon_densities(&self, specs: &[GroupSpec]) -> Vec<Density> {
        let counts: Vec<usize> = specs.iter().map(|g| g.commands.len()).collect();
        let available = (self.window.width - RIBBON_CHROME).max(0.0);
        solve_densities(&counts, available)
    }

    /// The groups shown for the active ribbon tab, each at its solved density.
    fn ribbon_body(&self) -> Element<'_, Message> {
        let Some(specs) = self.ribbon_specs() else {
            return self.windows_body();
        };
        let densities = self.ribbon_densities(&specs);
        let mut band = row![].spacing(GROUP_GAP).align_y(Alignment::Start);
        for (i, (spec, &density)) in specs.iter().zip(&densities).enumerate() {
            band = band.push(render_group(spec, density, i));
        }
        // The View tab gets a live orientation-cube size control (a slider has no
        // place in the icon-command band, so it is appended as its own group).
        if self.active_tab == RibbonTab::View {
            band = band.push(self.cube_size_group());
        }
        band.into()
    }

    /// A labelled slider setting the orientation cube's on-screen size, appended
    /// to the View tab. Disabled (greyed) while the cube is hidden.
    fn cube_size_group(&self) -> Element<'_, Message> {
        let control: Element<'_, Message> = if self.show_gizmo {
            slider(
                GIZMO_SIZE_MIN..=GIZMO_SIZE_MAX,
                self.gizmo_size,
                Message::SetGizmoSize,
            )
            .step(1.0_f32)
            .width(Length::Fixed(120.0))
            .into()
        } else {
            // Keep the footprint stable when the cube is off: an inert placeholder.
            container(
                Space::new()
                    .width(Length::Fixed(120.0))
                    .height(Length::Fixed(16.0)),
            )
            .into()
        };
        let value = text(format!("{} px", self.gizmo_size as i32))
            .size(11)
            .color(palette::GROUP_LABEL);
        ribbon_group(
            "Cube size",
            column![control, value].spacing(4).align_x(Alignment::Center),
        )
    }

    /// The Windows tab: a checkbox per pane (naturally narrow, no collapse).
    fn windows_body(&self) -> Element<'_, Message> {
        let mut panes = column![].spacing(4);
        // The Viewport is always visible and has no toggle.
        for pane in ALL_PANES.into_iter().filter(|p| *p != Pane::Viewport) {
            let shown = self.pane_handle(pane).is_some();
            panes = panes.push(
                row![
                    checkbox(shown)
                        .size(15)
                        .on_toggle(move |v| Message::SetPaneVisible(pane, v)),
                    text(pane.name()).size(13),
                ]
                .spacing(6)
                .align_y(Alignment::Center),
            );
        }
        row![ribbon_group("Panes", panes)].into()
    }

    /// The floating panel for an open collapsed-group popup, positioned under its
    /// button. `None` unless a group is open *and* actually collapsed at the
    /// current width. Its x-offset is the exact sum of the preceding groups' drawn
    /// widths — the analytic layout doubles as popup positioning.
    fn ribbon_popup(&self) -> Option<Element<'_, Message>> {
        let index = self.open_group?;
        let specs = self.ribbon_specs()?;
        let spec = specs.get(index)?;
        let densities = self.ribbon_densities(&specs);
        if !densities.get(index)?.is_popup() {
            return None;
        }
        let x = 6.0
            + densities[..index]
                .iter()
                .zip(&specs[..index])
                .map(|(&d, s)| group_width(s.commands.len(), d) + GROUP_GAP)
                .sum::<f32>();

        let mut commands = row![].spacing(CMD_GAP);
        for command in &spec.commands {
            commands = commands.push(render_command(command, false));
        }
        let panel = container(commands)
            .padding(6)
            .style(|_theme| container::Style {
                background: Some(Background::Color(palette::RIBBON_BG)),
                border: Border {
                    color: palette::BORDER_DARK,
                    width: 1.0,
                    radius: 3.0.into(),
                },
                ..container::Style::default()
            });
        // Drop the panel just below the ribbon, offset to the group's column.
        let positioned = container(panel)
            .width(Length::Fill)
            .height(Length::Fill)
            .align_x(iced::alignment::Horizontal::Left)
            .align_y(iced::alignment::Vertical::Top)
            .padding(Padding {
                top: RIBBON_H,
                left: x,
                right: 0.0,
                bottom: 0.0,
            });
        Some(positioned.into())
    }

    fn pane_content(&self, pane: Pane) -> Element<'_, Message> {
        let inner: Element<'_, Message> = match pane {
            Pane::Project => self.project_tree(),
            Pane::Viewport => container(
                shader(Viewport::new(
                    &self.controller,
                    self.show_stock,
                    self.view,
                    self.show_gizmo,
                    self.gizmo_size,
                    &self.focus_ops.iter().copied().collect::<Vec<_>>(),
                    self.snap_hover.map(|h| (h, self.snap_aperture)),
                    self.hover_loop,
                    self.in_origin_pick(),
                    self.show_origin,
                    self.origin_first,
                ))
                .width(Length::Fill)
                .height(Length::Fill),
            )
            .width(Length::Fill)
            .height(Length::Fill)
            .into(),
            Pane::Inspector => self.inspector(),
            Pane::Output => self.output(),
        };
        // A bordered panel so the gaps between panes read as visible separators.
        container(inner)
            .width(Length::Fill)
            .height(Length::Fill)
            .style(|_theme| container::Style {
                background: Some(Background::Color(palette::PANE_BG)),
                border: Border {
                    color: palette::SEPARATOR,
                    width: 1.0,
                    radius: 0.0.into(),
                },
                ..container::Style::default()
            })
            .into()
    }

    /// The project tree: a structure view of the setup, stock, the tools in use
    /// (read-only), and the operations. Rows are plain selectable text (the selected
    /// row is highlighted, not a button). Each operation row carries inline controls
    /// (include checkbox + reorder arrows) and a right-click menu (Duplicate/Delete).
    fn project_tree(&self) -> Element<'_, Message> {
        let setup = &self.controller.document().setup;
        let sel = self.controller.selection();

        let o = setup.origin;
        let mut list = column![
            select_row(
                format!("Setup — {}", setup.name),
                sel == Selection::Setup,
                Message::Select(Selection::Setup),
            ),
            select_row(
                format!("Origin — X{} Y{} Z{}", fmt_num(o[0]), fmt_num(o[1]), fmt_num(o[2])),
                sel == Selection::Origin,
                Message::Select(Selection::Origin),
            ),
            select_row(
                "Stock".to_string(),
                sel == Selection::Stock,
                Message::Select(Selection::Stock),
            ),
        ]
        .spacing(2);

        // Tools in use — read-only; tools are chosen from the library during op setup.
        list = list.push(tree_header("Tools (in use)"));
        let used = self.controller.used_tools();
        if used.is_empty() {
            list = list.push(tree_note("none yet — set up an operation"));
        }
        for t in used {
            list = list.push(
                container(
                    text(format!("T{} ⌀{} {}", t.number, fmt_num(t.diameter), t.kind))
                        .size(13)
                        .color(palette::LABEL_COLOR),
                )
                .padding(Padding::from([3.0, 6.0])),
            );
        }

        // Exact-duplicate operations (identical bar their id, both included),
        // computed once: `twins[id]` is the ids of the other ops it duplicates.
        let dup_groups = self.controller.duplicate_operation_groups();
        let mut twins: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
        for group in &dup_groups {
            for &id in group {
                twins.insert(id, group.iter().copied().filter(|&x| x != id).collect());
            }
        }
        // Header carries a ⚠ flag when any duplicate exists, so it reads at a glance.
        list = list.push(if dup_groups.is_empty() {
            tree_header("Operations")
        } else {
            container(
                row![
                    text("Operations").size(11).color(palette::GROUP_LABEL),
                    text("⚠ duplicates exist").size(11).color(palette::WARN),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            )
            .padding(Padding::from([6.0, 4.0]))
            .into()
        });
        if setup.operations.is_empty() {
            list = list.push(tree_note("none — add one from the Operations tab"));
        }
        let op_count = setup.operations.len();
        for (i, op) in setup.operations.iter().enumerate() {
            let id = op.id();
            // Row highlight tracks the viewport focus set, so every op lit in the
            // viewport reads as selected here — including a multi-selection.
            let active = self.focus_ops.contains(&id);
            let excluded = self.controller.is_operation_excluded(id);
            // Inline controls: an include checkbox (checked = machined) and reorder
            // arrows. Left-click the row selects it; right-click opens Duplicate/Delete.
            let mut label = format!("{id}: {}", op_kind(op));
            if excluded {
                label.push_str("  (excluded)");
            }
            let include = checkbox(!excluded)
                .size(15)
                .on_toggle(move |checked| Message::SetOpExcluded(id, !checked));
            let name = text(label).size(13).width(Length::Fill).color(if excluded {
                palette::GROUP_LABEL
            } else {
                palette::LABEL_COLOR
            });
            let up = button(text("↑").size(12))
                .on_press_maybe((i > 0).then_some(Message::MoveOp(id, true)));
            let down = button(text("↓").size(12))
                .on_press_maybe((i + 1 < op_count).then_some(Message::MoveOp(id, false)));
            // On an exact duplicate, mark it ⚠ and name its twin(s) by id — both
            // would post the same toolpath.
            let mut controls = row![include, name].spacing(4).align_y(Alignment::Center);
            if let Some(t) = twins.get(&id) {
                let ids = t
                    .iter()
                    .map(|x| format!("#{x}"))
                    .collect::<Vec<_>>()
                    .join(", ");
                controls = controls.push(text(format!("⚠ {ids}")).size(11).color(palette::WARN));
            }
            let inner = container(
                controls
                    .push(up)
                    .push(down)
                    .spacing(4)
                    .align_y(Alignment::Center),
            )
            .width(Length::Fill)
            .padding(Padding::from([2.0, 6.0]))
            .style(move |_theme| container::Style {
                background: active.then_some(Background::Color(palette::SELECT_BG)),
                border: Border {
                    radius: 3.0.into(),
                    ..Border::default()
                },
                ..container::Style::default()
            });
            list = list.push(
                mouse_area(inner)
                    .on_press(Message::ClickOp(id))
                    .on_right_press(Message::OpMenu(id)),
            );
        }

        // The right-click menu anchors to the window-absolute cursor tracked by the
        // global subscription, so the pane itself needs no cursor tracking.
        scrollable(list.padding(6))
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    /// The inspector: editable fields for the selected node.
    /// The new-operation wizard shown in the Inspector while a geometry pick is
    /// pending: the kind, a tool picker, a prompt, and Cancel.
    fn op_wizard(&self, pending: PendingOp) -> Element<'_, Message> {
        let kind = match pending.kind {
            OpKind::Profile => "Profile",
            OpKind::Pocket => "Pocket",
            OpKind::Drill => "Drill",
            OpKind::Face => "Face",
            OpKind::Chamfer => "Chamfer",
            OpKind::Thread => "Thread",
        };
        // The tool is picked from the cross-project library; picking embeds a copy
        // into the setup (see `use_tool`). The current selection is the library entry
        // whose geometry matches the pending op's embedded tool.
        let tools: Vec<ToolChoice> = self
            .library
            .tools
            .iter()
            .enumerate()
            .map(|(index, t)| ToolChoice {
                index,
                number: t.number,
                diameter: t.diameter,
                kind: t.kind,
            })
            .collect();
        let embedded = self
            .controller
            .document()
            .setup
            .tools
            .iter()
            .find(|t| t.number == pending.tool)
            .copied();
        let selected = embedded
            .and_then(|e| {
                self.library.tools.iter().position(|l| {
                    l.diameter == e.diameter
                        && l.length == e.length
                        && l.flutes == e.flutes
                        && l.kind == e.kind
                })
            })
            .and_then(|i| tools.get(i).copied());
        let picker = pick_list(tools, selected, |c| Message::SetPendingLibraryTool(c.index))
            .text_size(13)
            .width(Length::Fill);
        let mut col = column![
            text(format!("New {kind} operation")).size(15),
            text("Tool").size(12),
            row![
                picker,
                button(text("＋ New").size(13)).on_press(Message::NewTool),
            ]
            .spacing(6)
            .align_y(Alignment::Center),
        ]
        .spacing(10)
        .padding(8);

        // Object snaps govern where the start/lead-in lands; show them for the
        // kinds that carry a start, while still awaiting that (boundary) pick.
        if op_uses_snaps(pending.kind) && pending.boundary.is_none() {
            col = col.push(self.snap_toolbar());
        }

        // Pocket island mode begins once the boundary is picked.
        if pending.kind == OpKind::Pocket && pending.boundary.is_some() {
            col = col.push(
                text(format!(
                    "Click enclosed areas to exclude ({} selected), then Confirm.",
                    pending.islands.len()
                ))
                .size(12),
            );
            col = col.push(
                row![
                    button(text("Confirm").size(13)).on_press(Message::ConfirmOp),
                    button(text("Cancel").size(13)).on_press(Message::CancelOp),
                ]
                .spacing(8),
            );
        } else {
            col = col.push(text("Click a boundary line in the viewport.").size(12));
            col = col.push(button(text("Cancel").size(13)).on_press(Message::CancelOp));
        }
        col.into()
    }

    /// The object-snap toggle row shown during a pick. Enabled snaps read as
    /// filled; Nearest is the opt-in fallback. (Quadrant arrives in Phase 2.)
    fn snap_toolbar(&self) -> Element<'_, Message> {
        let toggle = |kind: SnapKind, label: &'static str| {
            let on = self.snaps.contains(&kind);
            button(text(label).size(12))
                .padding(Padding::from([3.0, 8.0]))
                .on_press(Message::ToggleSnap(kind))
                .style(move |_theme, status| snap_toggle_style(on, status))
        };
        column![
            text("Object snaps").size(12).color(palette::GROUP_LABEL),
            row![
                toggle(SnapKind::End, "End"),
                toggle(SnapKind::Mid, "Mid"),
                toggle(SnapKind::Quadrant, "Quad"),
                toggle(SnapKind::Nearest, "Nearest"),
            ]
            .spacing(6),
        ]
        .spacing(4)
        .into()
    }

    /// The Setup node's program-start-point editor: an enable toggle, a base
    /// choice (from the origin, or from a reference point), and the offset (plus
    /// the reference when chosen). The numeric rows reuse the field pipeline
    /// (parse + Apply); the toggles commit immediately.
    fn start_point_editor(&self) -> Element<'_, Message> {
        let on = self.controller.document().setup.start_offset.is_some();
        let mut col = column![text("Program start point (offset from origin)")
            .size(12)
            .color(palette::GROUP_LABEL)]
        .spacing(6);
        col = col.push(
            row![
                checkbox(on).size(15).on_toggle(Message::ToggleStartPoint),
                text("Rapid to a start point").size(13),
            ]
            .spacing(6)
            .align_y(Alignment::Center),
        );
        if on {
            let field =
                |f: Field| field_row(f, &self.fields.get(&f).cloned().unwrap_or_default());
            col = col.push(field(Field::StartOffX));
            col = col.push(field(Field::StartOffY));
            col = col.push(field(Field::StartOffZ));
        }
        col.into()
    }

    /// The Tooling-tab tool-library editor: a selectable list of library tools plus
    /// the selected tool's editable fields (reusing the inspector field pipeline).
    /// New / Delete live on the Tooling ribbon tab.
    fn library_editor(&self) -> Element<'_, Message> {
        let mut list = column![
            text("Tool Library").size(15),
            text("Reusable across projects · New / Delete on the Tooling tab").size(11),
        ]
        .spacing(8)
        .padding(8);

        let mut rows = column![].spacing(2);
        for (i, t) in self.library.tools.iter().enumerate() {
            let label = format!("T{} ⌀{} {}", t.number, fmt_num(t.diameter), t.kind);
            rows = rows.push(select_row(
                label,
                i == self.lib_sel,
                Message::SelectLibraryTool(i),
            ));
        }
        list = list.push(rows);

        for field in self.inspector_fields() {
            let value = self.fields.get(&field).cloned().unwrap_or_default();
            list = list.push(field_row(field, &value));
        }
        if let Some(t) = self.library.tools.get(self.lib_sel) {
            list = list.push(row![
                text("Type").width(Length::Fixed(150.0)).size(13),
                pick_list(&ToolKindPick::ALL[..], Some(ToolKindPick::of(t.kind)), |p| {
                    Message::ToolKindChanged(p.to_kind())
                })
                .text_size(13)
                .width(Length::Fixed(140.0)),
            ]);
            list = list.push(button("Apply").on_press(Message::Apply));
        }

        scrollable(list)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    fn inspector(&self) -> Element<'_, Message> {
        // While a new-operation wizard is active, the inspector is the wizard.
        if let Some(pending) = self.controller.pending_op() {
            return self.op_wizard(pending);
        }
        // The Tooling tab turns the Inspector into the tool-library editor.
        if self.library_mode() {
            return self.library_editor();
        }
        let heading = match self.controller.selection() {
            Selection::Setup => "Setup".to_string(),
            Selection::Origin => "Workpiece origin".to_string(),
            Selection::Stock => "Stock".to_string(),
            Selection::Tool(i) => format!("Tool {}", i + 1),
            Selection::Operation(id) => match self.controller.operation(id) {
                Some(op) => format!("Operation {id} — {}", op_kind(op)),
                None => "Operation".to_string(),
            },
        };

        let mut list = column![text(heading).size(15)].spacing(8).padding(8);

        if let Selection::Stock = self.controller.selection() {
            // The concrete block the offsets/heights resolve to, as read-only
            // context above the editable fields.
            let (min, max) = self.controller.stock_box();
            list = list.push(
                text(format!(
                    "Resolved box\n  X {:.1}…{:.1}   Y {:.1}…{:.1}   Z {:.1}…{:.1}",
                    min[0], max[0], min[1], max[1], min[2], max[2]
                ))
                .size(11)
                .color(palette::GROUP_LABEL),
            );
        }

        let ordered = self.inspector_fields();
        if ordered.is_empty() {
            list = list.push(text("Nothing to edit here yet.").size(12));
        }
        for field in ordered {
            // Start-point fields render in their own section (below), not here.
            if is_start_field(field) {
                continue;
            }
            let value = self.fields.get(&field).cloned().unwrap_or_default();
            list = list.push(
                row![
                    text(field.label()).width(Length::Fixed(150.0)).size(13),
                    text_input("", &value)
                        .on_input(move |v| Message::FieldChanged(field, v))
                        .on_submit(Message::Apply)
                        .width(Length::Fixed(90.0)),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
        }
        // The program start-point editor lives on the Origin node (offset from it).
        if let Selection::Origin = self.controller.selection() {
            list = list.push(self.start_point_editor());
        }
        // The tool geometry class is an enum, so it gets a picker (committed
        // immediately) rather than a text field.
        if let Selection::Tool(i) = self.controller.selection() {
            if let Some(tool) = self.controller.document().setup.tools.get(i) {
                list = list.push(
                    row![
                        text("Type").width(Length::Fixed(150.0)).size(13),
                        pick_list(
                            &ToolKindPick::ALL[..],
                            Some(ToolKindPick::of(tool.kind)),
                            |p| Message::ToolKindChanged(p.to_kind())
                        )
                        .text_size(13)
                        .width(Length::Fixed(140.0)),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            }
        }
        // Lead / plunge strategy pickers (their numeric parameters appear as fields
        // above when the kind uses them). Profiles get lead-in/out + plunge; pockets
        // get plunge only.
        if let Selection::Operation(id) = self.controller.selection() {
            match self.controller.operation(id) {
                Some(Operation::Profile(p)) => {
                    list = list.push(profile_picker(
                        "Side",
                        p.side,
                        &Side::ALL[..],
                        Message::SideChanged,
                    ));
                    list = list.push(profile_picker(
                        "Lead-in",
                        LeadKind::of(p.lead_in),
                        &LeadKind::ALL[..],
                        Message::LeadInKindChanged,
                    ));
                    list = list.push(profile_picker(
                        "Lead-out",
                        LeadKind::of(p.lead_out),
                        &LeadKind::ALL[..],
                        Message::LeadOutKindChanged,
                    ));
                    list = list.push(profile_picker(
                        "Plunge",
                        PlungeKind::of(p.plunge),
                        &PlungeKind::ALL[..],
                        Message::PlungeKindChanged,
                    ));
                }
                Some(Operation::Pocket(p)) => {
                    list = list.push(profile_picker(
                        "Plunge",
                        PlungeKind::of(p.plunge),
                        &PlungeKind::ALL[..],
                        Message::PlungeKindChanged,
                    ));
                }
                Some(Operation::Face(f)) => {
                    list = list.push(profile_picker(
                        "Direction",
                        f.direction,
                        &Axis::ALL[..],
                        Message::FaceDirectionChanged,
                    ));
                }
                Some(Operation::Chamfer(c)) => {
                    list = list.push(profile_picker(
                        "Side",
                        c.side,
                        &Side::ALL[..],
                        Message::SideChanged,
                    ));
                    list = list.push(profile_picker(
                        "Lead-in",
                        LeadKind::of(c.lead_in),
                        &LeadKind::ALL[..],
                        Message::LeadInKindChanged,
                    ));
                    list = list.push(profile_picker(
                        "Lead-out",
                        LeadKind::of(c.lead_out),
                        &LeadKind::ALL[..],
                        Message::LeadOutKindChanged,
                    ));
                    // Gradual stepping (equal material per pass) — only meaningful
                    // when the chamfer is cut in multiple passes.
                    list = list.push(
                        row![
                            text("Gradual").width(Length::Fixed(150.0)).size(13),
                            checkbox(c.gradual)
                                .size(15)
                                .on_toggle(Message::ChamferGradualToggled),
                        ]
                        .spacing(8)
                        .align_y(Alignment::Center),
                    );
                }
                Some(Operation::Thread(t)) => {
                    list = list.push(profile_picker(
                        "Bore",
                        Bore::of(t.internal),
                        &Bore::ALL[..],
                        |b| Message::ThreadInternalChanged(b == Bore::Internal),
                    ));
                    list = list.push(profile_picker(
                        "Hand",
                        t.hand,
                        &Hand::ALL[..],
                        Message::ThreadHandChanged,
                    ));
                    list = list.push(profile_picker(
                        "Cut",
                        CutStyle::of(t.climb),
                        &CutStyle::ALL[..],
                        |c| Message::ThreadClimbChanged(c == CutStyle::Climb),
                    ));
                }
                _ => {}
            }
        }

        list = list.push(button("Apply").on_press(Message::Apply));

        scrollable(list)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    /// The output dock: run diagnostics and the status line.
    fn output(&self) -> Element<'_, Message> {
        let mut list = column![text(self.status.clone()).size(13)]
            .spacing(4)
            .padding(8);
        if let Some(outcome) = self.controller.outcome() {
            if outcome.diagnostics.is_empty() {
                list = list.push(text("No diagnostics.").size(12));
            } else {
                list = list.push(text("Diagnostics:").size(13));
                for d in &outcome.diagnostics {
                    list = list.push(text(format!("• {:?}: {}", d.severity, d.message)).size(12));
                }
            }
            // Material-removal simulation results: collisions a backplot can't show.
            if outcome.collisions.is_empty() {
                list = list.push(text("Simulation: no collisions.").size(12));
            } else {
                list =
                    list.push(text(format!("Collisions ({}):", outcome.collisions.len())).size(13));
                for c in &outcome.collisions {
                    list = list.push(text(format!("⚠ {:?}: {}", c.kind, c.message)).size(12));
                }
            }
        }
        scrollable(list)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }
}

/// The `a` and `b` subtrees of the split with the given id, if found.
fn find_split(
    node: &pane_grid::Node,
    split: pane_grid::Split,
) -> Option<(&pane_grid::Node, &pane_grid::Node)> {
    match node {
        pane_grid::Node::Split { id, a, b, .. } => {
            if *id == split {
                Some((a, b))
            } else {
                find_split(a, split).or_else(|| find_split(b, split))
            }
        }
        pane_grid::Node::Pane(_) => None,
    }
}

/// The splits on the path from the root to `pane`, outermost first. Each is
/// paired with whether `pane` lies in that split's `a` subtree (`true`) or its
/// `b` subtree (`false`). `None` if the pane isn't in the tree.
fn splits_to_pane(
    node: &pane_grid::Node,
    pane: pane_grid::Pane,
) -> Option<Vec<(pane_grid::Split, bool)>> {
    match node {
        pane_grid::Node::Pane(p) => (*p == pane).then(Vec::new),
        pane_grid::Node::Split { id, a, b, .. } => {
            if let Some(mut rest) = splits_to_pane(a, pane) {
                rest.insert(0, (*id, true));
                Some(rest)
            } else if let Some(mut rest) = splits_to_pane(b, pane) {
                rest.insert(0, (*id, false));
                Some(rest)
            } else {
                None
            }
        }
    }
}

/// The minimum size (px) a subtree needs along `axis`, **honouring the current
/// internal split ratios**. This matters because dragging one boundary only
/// changes that one split's ratio — every descendant split keeps its ratio — so
/// the space handed to a subtree is apportioned by fixed fractions, not freely.
/// A child split along `axis` with ratio `r` gives region `a` about `r·(W−gap)`,
/// so to keep both children above their mins the subtree needs
/// `gap + max(min_a/r, min_b/(1−r))`; perpendicular splits take the larger child.
/// (A plain sum-of-mins would under-count and let one pane starve while its
/// sibling stays wide.)
fn subtree_min(
    node: &pane_grid::Node,
    panes: &pane_grid::State<Pane>,
    axis: pane_grid::Axis,
) -> f32 {
    match node {
        pane_grid::Node::Pane(p) => panes.get(*p).map_or(0.0, |pane| pane.min_size()),
        pane_grid::Node::Split {
            axis: sub_axis,
            a,
            b,
            ratio,
            ..
        } => {
            let (ma, mb) = (subtree_min(a, panes, axis), subtree_min(b, panes, axis));
            if *sub_axis == axis {
                let r = ratio.clamp(0.05, 0.95);
                PANE_SPACING + (ma / r).max(mb / (1.0 - r))
            } else {
                ma.max(mb)
            }
        }
    }
}

/// A labelled ribbon group: its command widgets stacked over a small caption in
/// muted grey (OpenCADStudio places the group label at the bottom).
fn ribbon_group<'a>(
    title: &'a str,
    content: impl Into<Element<'a, Message>>,
) -> Element<'a, Message> {
    let caption = text(title).size(10).color(palette::GROUP_LABEL);
    container(column![content.into(), caption].spacing(4))
        .padding(Padding::from([3.0, 4.0]))
        .into()
}

/// Fixed width of a large (Full-density) ribbon command button, px.
const COMMAND_W: f32 = 64.0;

/// Tab-strip button: transparent normally, accent-filled when active, a subtle
/// fill on hover.
fn tab_button_style(active: bool, status: button::Status) -> button::Style {
    let background = if active {
        Some(Background::Color(palette::ACCENT_BLUE))
    } else if matches!(status, button::Status::Hovered | button::Status::Pressed) {
        Some(Background::Color(palette::TOOL_HOVER))
    } else {
        None
    };
    button::Style {
        background,
        text_color: if active {
            Color::WHITE
        } else {
            palette::LABEL_COLOR
        },
        border: Border {
            radius: 3.0.into(),
            ..Border::default()
        },
        ..button::Style::default()
    }
}

/// Object-snap toggle: filled accent when enabled, outlined when off.
fn snap_toggle_style(on: bool, status: button::Status) -> button::Style {
    let hovered = matches!(status, button::Status::Hovered | button::Status::Pressed);
    let background = if on {
        Some(Background::Color(palette::ACCENT_BLUE))
    } else if hovered {
        Some(Background::Color(palette::TOOL_HOVER))
    } else {
        None
    };
    button::Style {
        background,
        text_color: if on {
            Color::WHITE
        } else {
            palette::LABEL_COLOR
        },
        border: Border {
            color: palette::SEPARATOR,
            width: 1.0,
            radius: 3.0.into(),
        },
        ..button::Style::default()
    }
}

/// Ribbon command button: flat, muted-grey when disabled, a row-hover fill
/// otherwise.
fn command_button_style(status: button::Status) -> button::Style {
    let background = match status {
        button::Status::Hovered | button::Status::Pressed => {
            Some(Background::Color(palette::ROW_HOVER))
        }
        _ => None,
    };
    let text_color = if matches!(status, button::Status::Disabled) {
        palette::GROUP_LABEL
    } else {
        palette::LABEL_COLOR
    };
    button::Style {
        background,
        text_color,
        border: Border {
            radius: 3.0.into(),
            ..Border::default()
        },
        ..button::Style::default()
    }
}

/// Ribbon toggle button: accent-tinted while `on`, otherwise a normal command
/// button.
fn command_toggle_style(on: bool, status: button::Status) -> button::Style {
    if !on {
        return command_button_style(status);
    }
    let mut accent = palette::ACCENT_BLUE;
    // A muted accent fill so the icon/label stay readable; brighter on hover.
    accent.a = if matches!(status, button::Status::Hovered | button::Status::Pressed) {
        0.55
    } else {
        0.38
    };
    button::Style {
        background: Some(Background::Color(accent)),
        text_color: palette::LABEL_COLOR,
        border: Border {
            radius: 3.0.into(),
            ..Border::default()
        },
        ..button::Style::default()
    }
}

// --- Responsive ribbon collapse ------------------------------------------------
//
// As the window narrows, each group degrades through four densities so the ribbon
// never overflows — modelled on OpenCADStudio's ribbon. The layout maths is a pure
// function of the per-group command counts and the available width, so the
// degradation order is unit-tested without any GUI.

/// How densely a ribbon group is drawn. Ordered loosest → tightest.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Density {
    /// Large icon-over-label buttons in a row.
    Full,
    /// Icon-only buttons in a row (labels dropped).
    Compact,
    /// One representative button (icon + caption) that opens a popup of the group.
    Collapsed,
    /// One representative button (icon only) that opens a popup.
    Tight,
}

impl Density {
    /// A collapsed/tight group is a single popup-opening button, not its commands.
    fn is_popup(self) -> bool {
        matches!(self, Density::Collapsed | Density::Tight)
    }
}

const CMD_GAP: f32 = 2.0; // between command buttons within a group
const GROUP_GAP: f32 = 8.0; // between groups
const GROUP_PAD: f32 = 4.0; // per side, inside a group container
const COMPACT_W: f32 = 34.0; // icon-only command button
const COLLAPSED_W: f32 = 64.0; // single group button, with caption
const TIGHT_W: f32 = 34.0; // single group button, icon only
/// Ribbon chrome outside the group row (band padding + a little slack).
const RIBBON_CHROME: f32 = 28.0;
/// Approximate full ribbon height (tab strip + band), for dropping popups just
/// below it. Eyeballed; a few px off only shifts the popup slightly.
const RIBBON_H: f32 = 96.0;

/// The drawn width of a group of `n` commands at a given density.
fn group_width(n: usize, density: Density) -> f32 {
    let (btn_w, buttons) = match density {
        Density::Full => (COMMAND_W, n),
        Density::Compact => (COMPACT_W, n),
        Density::Collapsed => (COLLAPSED_W, 1),
        Density::Tight => (TIGHT_W, 1),
    };
    let inner = buttons as f32 * btn_w + buttons.saturating_sub(1) as f32 * CMD_GAP;
    // Full/Compact groups wrap in `ribbon_group`, a container padded `GROUP_PAD`
    // per side; Collapsed/Tight render as a bare fixed-width button with no such
    // wrapper. Counting the padding for the popup densities made this width — and
    // thus the popup x-offset that sums it over preceding groups — drift right by
    // `2*GROUP_PAD` per collapsed group.
    if density.is_popup() {
        inner
    } else {
        inner + GROUP_PAD * 2.0
    }
}

/// The total drawn width of the group row at the given densities (with gaps).
fn row_width(counts: &[usize], densities: &[Density]) -> f32 {
    let groups: f32 = counts
        .iter()
        .zip(densities)
        .map(|(&n, &d)| group_width(n, d))
        .sum();
    let gaps = counts.len().saturating_sub(1) as f32 * GROUP_GAP;
    groups + gaps
}

/// Assign each group a density so the row fits `available` px. Groups degrade
/// **right-to-left**, one density level at a time (all groups drop to Compact from
/// the right before any drop to Collapsed, etc.) — OpenCADStudio's behaviour. If
/// everything is Tight and it still overflows, the row is left tight (iced clips).
fn solve_densities(counts: &[usize], available: f32) -> Vec<Density> {
    let mut densities = vec![Density::Full; counts.len()];
    for level in [Density::Compact, Density::Collapsed, Density::Tight] {
        if row_width(counts, &densities) <= available {
            break;
        }
        for i in (0..densities.len()).rev() {
            if densities[i] < level {
                densities[i] = level;
            }
            if row_width(counts, &densities) <= available {
                break;
            }
        }
    }
    densities
}

/// One ribbon command in a group's data model — enough to render it at any
/// density (icon+label, icon-only, or inside a popup).
struct Command {
    icon: Icon,
    label: &'static str,
    /// `None` disables the button (greyed, unclickable).
    action: Option<Message>,
    /// `Some(on)` renders a toggle (accent-tinted while on); `None` is a plain action.
    toggle: Option<bool>,
}

/// A ribbon group's data model.
struct GroupSpec {
    title: &'static str,
    commands: Vec<Command>,
}

impl GroupSpec {
    /// The icon shown when the group collapses to a single button.
    fn rep_icon(&self) -> Icon {
        self.commands.first().map_or(Icon::Run, |c| c.icon)
    }
}

fn cmd(icon: Icon, label: &'static str, action: Option<Message>) -> Command {
    Command {
        icon,
        label,
        action,
        toggle: None,
    }
}

fn toggle_cmd(icon: Icon, label: &'static str, on: bool, msg: Message) -> Command {
    Command {
        icon,
        label,
        action: Some(msg),
        toggle: Some(on),
    }
}

/// Render a single command at Full (`compact = false`, icon over label) or Compact
/// (`compact = true`, icon only) density.
fn render_command(command: &Command, compact: bool) -> Element<'static, Message> {
    let (icon_px, width) = if compact {
        (22.0, COMPACT_W)
    } else {
        (26.0, COMMAND_W)
    };
    let content: Element<'static, Message> = if compact {
        icon_svg(command.icon, icon_px)
    } else {
        column![
            icon_svg(command.icon, icon_px),
            text(command.label).size(10)
        ]
        .spacing(3)
        .align_x(Alignment::Center)
        .into()
    };
    let base = button(content)
        .width(Length::Fixed(width))
        .padding(Padding::from([4.0, 2.0]));
    match command.toggle {
        Some(on) => base
            .on_press_maybe(command.action.clone())
            .style(move |_theme, status| command_toggle_style(on, status))
            .into(),
        None => base
            .on_press_maybe(command.action.clone())
            .style(|_theme, status| command_button_style(status))
            .into(),
    }
}

/// Render a whole group at its assigned density. Collapsed/Tight groups become a
/// single button that toggles the group's popup (`index` in the active tab).
fn render_group(spec: &GroupSpec, density: Density, index: usize) -> Element<'static, Message> {
    if density.is_popup() {
        let (icon_px, width) = if density == Density::Tight {
            (22.0, TIGHT_W)
        } else {
            (26.0, COLLAPSED_W)
        };
        let content: Element<'static, Message> = if density == Density::Collapsed {
            column![
                icon_svg(spec.rep_icon(), icon_px),
                text(spec.title).size(10)
            ]
            .spacing(3)
            .align_x(Alignment::Center)
            .into()
        } else {
            icon_svg(spec.rep_icon(), icon_px)
        };
        return button(content)
            .width(Length::Fixed(width))
            .padding(Padding::from([4.0, 2.0]))
            .on_press(Message::ToggleRibbonGroup(index))
            .style(|_theme, status| command_button_style(status))
            .into();
    }
    let compact = density == Density::Compact;
    let mut commands = row![].spacing(CMD_GAP);
    for command in &spec.commands {
        commands = commands.push(render_command(command, compact));
    }
    ribbon_group(spec.title, commands)
}

/// A short label for an operation's kind.
fn op_kind(op: &Operation) -> &'static str {
    match op {
        Operation::Profile(_) => "Profile",
        Operation::Drill(_) => "Drill",
        Operation::Pocket(_) => "Pocket",
        Operation::Face(_) => "Face",
        Operation::Chamfer(_) => "Chamfer",
        Operation::Thread(_) => "Thread",
    }
}

/// Read an operation's value for a given field, if the op has it.
fn op_field(op: &Operation, field: Field) -> Option<f64> {
    match (op, field) {
        (Operation::Profile(o), Field::Depth) => Some(o.depth),
        (Operation::Profile(o), Field::ProfileOffset) => Some(o.offset),
        (Operation::Profile(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Profile(o), Field::Feed) => Some(o.feed),
        (Operation::Profile(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Profile(o), Field::LeadInSize) => Some(lead_size(o.lead_in)),
        (Operation::Profile(o), Field::LeadOutSize) => Some(lead_size(o.lead_out)),
        (Operation::Profile(o), Field::LeadOverlap) => Some(o.lead_overlap),
        (Operation::Profile(o), Field::PlungeA) => Some(plunge_params(o.plunge).0),
        (Operation::Profile(o), Field::PlungeB) => Some(plunge_params(o.plunge).1),
        (Operation::Pocket(o), Field::Depth) => Some(o.depth),
        (Operation::Pocket(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Pocket(o), Field::Stepover) => Some(o.stepover),
        (Operation::Pocket(o), Field::Feed) => Some(o.feed),
        (Operation::Pocket(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Pocket(o), Field::LeadOverlap) => Some(o.lead_overlap),
        (Operation::Pocket(o), Field::PlungeA) => Some(plunge_params(o.plunge).0),
        (Operation::Pocket(o), Field::PlungeB) => Some(plunge_params(o.plunge).1),
        (Operation::Face(o), Field::FaceStartOffset) => Some(o.start_offset),
        (Operation::Face(o), Field::Depth) => Some(o.depth),
        (Operation::Face(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Face(o), Field::FaceOverlap) => Some(o.overlap * 100.0),
        (Operation::Face(o), Field::FaceOvershoot) => Some(o.overshoot),
        (Operation::Face(o), Field::Feed) => Some(o.feed),
        (Operation::Face(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Drill(o), Field::Depth) => Some(o.depth),
        (Operation::Drill(o), Field::Feed) => Some(o.feed),
        (Operation::Chamfer(o), Field::ChamferWidth) => Some(o.width),
        (Operation::Chamfer(o), Field::ChamferDepth) => Some(o.depth),
        (Operation::Chamfer(o), Field::ChamferStep) => Some(o.step),
        (Operation::Chamfer(o), Field::Feed) => Some(o.feed),
        (Operation::Chamfer(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Chamfer(o), Field::LeadInSize) => Some(lead_size(o.lead_in)),
        (Operation::Chamfer(o), Field::LeadOutSize) => Some(lead_size(o.lead_out)),
        (Operation::Chamfer(o), Field::LeadOverlap) => Some(o.lead_overlap),
        (Operation::Thread(o), Field::MajorDia) => Some(o.major_dia),
        (Operation::Thread(o), Field::Pitch) => Some(o.pitch),
        (Operation::Thread(o), Field::ThreadTop) => Some(o.z_top),
        (Operation::Thread(o), Field::ThreadBottom) => Some(o.z_bottom),
        (Operation::Thread(o), Field::Feed) => Some(o.feed),
        (Operation::Thread(o), Field::PlungeFeed) => Some(o.plunge_feed),
        _ => None,
    }
}

/// Write the parsed inspector fields onto an operation.
fn apply_op_fields(op: &mut Operation, parsed: &BTreeMap<Field, f64>) {
    let get = |f: Field| parsed.get(&f).copied();
    match op {
        Operation::Profile(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v;
            }
            if let Some(v) = get(Field::ProfileOffset) {
                o.offset = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
            if let Some(v) = get(Field::LeadInSize) {
                o.lead_in = set_lead_size(o.lead_in, v);
            }
            if let Some(v) = get(Field::LeadOutSize) {
                o.lead_out = set_lead_size(o.lead_out, v);
            }
            if let Some(v) = get(Field::LeadOverlap) {
                o.lead_overlap = v.max(0.0);
            }
            let (a, b) = plunge_params(o.plunge);
            let a = get(Field::PlungeA).unwrap_or(a);
            let b = get(Field::PlungeB).unwrap_or(b);
            o.plunge = set_plunge_params(o.plunge, a, b);
        }
        Operation::Pocket(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v;
            }
            if let Some(v) = get(Field::Stepover) {
                o.stepover = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
            if let Some(v) = get(Field::LeadOverlap) {
                o.lead_overlap = v.max(0.0);
            }
            let (a, b) = plunge_params(o.plunge);
            let a = get(Field::PlungeA).unwrap_or(a);
            let b = get(Field::PlungeB).unwrap_or(b);
            o.plunge = set_plunge_params(o.plunge, a, b);
        }
        Operation::Face(o) => {
            if let Some(v) = get(Field::FaceStartOffset) {
                o.start_offset = v.max(0.0);
            }
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v;
            }
            if let Some(v) = get(Field::FaceOverlap) {
                // Stored as a fraction; the field edits a percentage. Clamp below
                // 100 % so the pass spacing stays positive.
                o.overlap = (v / 100.0).clamp(0.0, 0.99);
            }
            if let Some(v) = get(Field::FaceOvershoot) {
                o.overshoot = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
        }
        Operation::Drill(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
        }
        Operation::Chamfer(o) => {
            if let Some(v) = get(Field::ChamferWidth) {
                o.width = v;
            }
            if let Some(v) = get(Field::ChamferDepth) {
                o.depth = v.max(0.0);
            }
            if let Some(v) = get(Field::ChamferStep) {
                o.step = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
            if let Some(v) = get(Field::LeadInSize) {
                o.lead_in = set_lead_size(o.lead_in, v);
            }
            if let Some(v) = get(Field::LeadOutSize) {
                o.lead_out = set_lead_size(o.lead_out, v);
            }
            if let Some(v) = get(Field::LeadOverlap) {
                o.lead_overlap = v.max(0.0);
            }
        }
        Operation::Thread(o) => {
            if let Some(v) = get(Field::MajorDia) {
                o.major_dia = v;
            }
            if let Some(v) = get(Field::Pitch) {
                o.pitch = v;
            }
            if let Some(v) = get(Field::ThreadTop) {
                o.z_top = v;
            }
            if let Some(v) = get(Field::ThreadBottom) {
                o.z_bottom = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
        }
    }
}

/// Format a number for display (Rust's shortest round-trippable form).
fn fmt_num(v: f64) -> String {
    format!("{v}")
}

/// A selectable project-tree / list row: plain text that highlights when active
/// (a muted fill, not a button). Left-click sends `on_press`.
fn select_row<'a>(label: String, active: bool, on_press: Message) -> Element<'a, Message> {
    let content = container(text(label).size(13).color(palette::LABEL_COLOR))
        .width(Length::Fill)
        .padding(Padding::from([3.0, 6.0]))
        .style(move |_theme| container::Style {
            background: active.then_some(Background::Color(palette::SELECT_BG)),
            border: Border {
                radius: 3.0.into(),
                ..Border::default()
            },
            ..container::Style::default()
        });
    mouse_area(content).on_press(on_press).into()
}

/// A section header row in the project tree.
fn tree_header<'a>(label: &str) -> Element<'a, Message> {
    container(text(label.to_string()).size(11).color(palette::GROUP_LABEL))
        .padding(Padding::from([6.0, 4.0]))
        .into()
}

/// A muted placeholder note in the project tree (e.g. an empty section).
fn tree_note<'a>(label: &str) -> Element<'a, Message> {
    container(
        text(format!("({label})"))
            .size(11)
            .color(palette::GROUP_LABEL),
    )
    .padding(Padding::from([1.0, 12.0]))
    .into()
}

/// An inspector field row: a label and a numeric text input bound to `field`.
fn field_row<'a>(field: Field, value: &str) -> Element<'a, Message> {
    row![
        text(field.label()).width(Length::Fixed(150.0)).size(13),
        text_input("", value)
            .on_input(move |v| Message::FieldChanged(field, v))
            .on_submit(Message::Apply)
            .width(Length::Fixed(90.0)),
    ]
    .spacing(8)
    .align_y(Alignment::Center)
    .into()
}

/// A labelled strategy picker row (lead / plunge kind) for the profile inspector.
fn profile_picker<T>(
    label: &str,
    selected: T,
    options: &'static [T],
    on_select: impl Fn(T) -> Message + 'static,
) -> Element<'static, Message>
where
    T: ToString + PartialEq + Clone + 'static,
{
    row![
        text(label.to_string()).width(Length::Fixed(150.0)).size(13),
        pick_list(options, Some(selected), on_select)
            .text_size(13)
            .width(Length::Fixed(140.0)),
    ]
    .spacing(8)
    .align_y(Alignment::Center)
    .into()
}

// ---------------------------------------------------------------------------
// The wgpu viewport, hosted in iced's shader widget — a 3D orbit camera.
// ---------------------------------------------------------------------------

/// Persistent orbit state — an *unclamped turntable*: `yaw` spins about the
/// world up axis, `pitch` tilts. Both are top view at `0`, and neither is
/// clamped, so pitch can carry all the way round to the underside. A turntable
/// (rather than a free trackball) keeps the horizon level and every drag
/// predictable.
#[derive(Clone, Copy, Debug, Default)]
struct ViewControls {
    yaw: f32,
    pitch: f32,
    zoom: f32,
    pan: [f32; 3],
}

/// Which button is dragging the view.
#[derive(Clone, Copy, Debug)]
enum DragMode {
    Orbit,
    Pan,
}

/// The shader widget's transient state: the active drag plus a pending cube
/// snap. The camera lives in [`App`] so the view-cube buttons can drive it; drags
/// report *relative* deltas back as messages (loss-free even across a burst of
/// events).
#[derive(Default)]
struct ViewportState {
    drag: Option<DragMode>,
    last: Option<iced::Point>,
    /// A gizmo-face snap `(yaw, pitch)` armed on press, fired on release only if
    /// the press stays a click. Cleared the moment the press becomes a drag, so a
    /// rotation that merely *begins* over the cube orbits instead of snapping.
    gizmo_arm: Option<(f32, f32)>,
    /// Where the (armed) press began, to measure click-vs-drag travel.
    press: Option<iced::Point>,
}

/// Orbit sensitivity (radians per pixel).
const ORBIT_SENS: f32 = 0.008;
/// Cursor travel (logical px) past which an armed cube press becomes an orbit
/// drag rather than a snap-to-view click.
const GIZMO_CLICK_SLOP: f32 = 4.0;
/// Zoom sensitivity (exponent per wheel line).
const ZOOM_SENS: f32 = 0.15;

/// Half-width (px) of the band along the viewport edges reserved for the
/// pane_grid resize dividers, whose grab zone (`PANE_RESIZE_LEEWAY`) extends a
/// little into the pane. Slightly larger than the leeway for safety.
const RESIZE_BAND: f32 = PANE_RESIZE_LEEWAY + 2.0;

/// Whether `pos` (absolute) is within [`RESIZE_BAND`] of any edge of `bounds` —
/// i.e. where a press likely means "drag the pane divider", not "orbit".
fn in_resize_band(pos: iced::Point, bounds: iced::Rectangle) -> bool {
    pos.x - bounds.x < RESIZE_BAND
        || bounds.x + bounds.width - pos.x < RESIZE_BAND
        || pos.y - bounds.y < RESIZE_BAND
        || bounds.y + bounds.height - pos.y < RESIZE_BAND
}

/// The orientation cube's square rectangle, anchored to the top-right of a
/// `w × h` viewport. `size` and `margin` are given in the same units as `w,h`
/// (logical px for the click hit-test, physical px for drawing), so the cube is a
/// **fixed** on-screen size rather than a fraction of the window. Clamped so it
/// always fits within the viewport with its margin.
fn gizmo_rect(w: f32, h: f32, size: f32, margin: f32) -> (f32, f32, f32) {
    let size = size.clamp(1.0, (w.min(h) - 2.0 * margin).max(1.0));
    (w - size - margin, margin, size)
}

struct Viewport {
    vertices: Arc<Vec<Vertex>>,
    mesh_vertices: Arc<Vec<MeshVertex>>,
    mesh_indices: Arc<Vec<u32>>,
    bounds: Option<([f32; 3], [f32; 3])>,
    controls: ViewControls,
    show_gizmo: bool,
    /// On-screen cube size, logical px (fixed; independent of window size).
    gizmo_size: f32,
    /// Geometry-pick mode (a new-operation wizard is awaiting a region click).
    picking: bool,
    /// "Set origin" pick mode — a click drops the workpiece datum.
    set_origin: bool,
    /// An object-snap is engaged under the cursor — its marker replaces the
    /// crosshair/pickbox.
    snap_engaged: bool,
    /// The world Z of the plane clicks are projected onto (top of stock).
    pick_z: f32,
}

impl Viewport {
    #[allow(clippy::too_many_arguments)] // cohesive per-frame view inputs
    fn new(
        controller: &AppController,
        show_stock: bool,
        controls: ViewControls,
        show_gizmo: bool,
        gizmo_size: f32,
        focus_ops: &[u32],
        snap: Option<(SnapHit, f64)>,
        hover_loop: Option<LoopRef>,
        set_origin: bool,
        show_origin: bool,
        origin_first: Option<[f64; 3]>,
    ) -> Self {
        let picking = controller.pending_op().is_some();
        let pick_z = controller.document().setup.heights.top_of_stock as f32;
        // After a run, show the full backplot; before it, at least show the
        // imported part outlines so opening a file is visibly reflected.
        let mut scene = match controller.outcome() {
            Some(outcome) => outcome.scene.clone(),
            None => {
                let mut scene = Scene::new();
                for region in controller.regions() {
                    scene.add_region(region, PART);
                }
                scene
            }
        };
        // Frame the camera on the *stable* part/backplot only — capture bounds
        // before the transient pick overlays (hover highlight, snap marker), or
        // the whole view would re-fit and drift as the cursor moves.
        let bounds = scene.bounds();
        // While the pick wizard is active, highlight the chosen boundary (accent)
        // and any excluded islands (gold) so the user sees what they picked.
        if let Some(pending) = controller.pending_op() {
            // The loop under the cursor (what a click selects), drawn first so the
            // boundary/island highlights paint over it once chosen.
            if let Some(c) = hover_loop
                .filter(|l| Some(*l) != pending.boundary)
                .and_then(|l| controller.loop_contour(l))
            {
                add_loop_highlight(&mut scene, c, PICK_HOVER);
            }
            if let Some(c) = pending.boundary.and_then(|b| controller.loop_contour(b)) {
                add_loop_highlight(&mut scene, c, PICK_BOUNDARY);
            }
            for island in &pending.islands {
                if let Some(c) = controller.loop_contour(*island) {
                    add_loop_highlight(&mut scene, c, PICK_ISLAND);
                }
            }
        }
        // The workpiece-origin datum marker (View toggle), only once geometry is
        // loaded — never on an empty document at startup — and sized to the scene.
        if show_origin && !controller.regions().is_empty() {
            if let Some((mn, mx)) = bounds {
                let origin = controller.document().setup.origin;
                let r = ((mx[0] - mn[0]).max(mx[1] - mn[1]) * 0.06).max(1.0);
                add_origin_marker(&mut scene, origin, pick_z, r);
            }
        }
        // The first point captured in two-point origin mode (awaiting the second).
        if let Some(first) = origin_first {
            let r = bounds
                .map(|(mn, mx)| ((mx[0] - mn[0]).max(mx[1] - mn[1]) * 0.04).max(1.0))
                .unwrap_or(3.0);
            add_origin_marker(&mut scene, first, pick_z, r);
        }
        // The object-snap marker under the cursor (op pick *or* set-origin).
        if let Some((hit, aperture)) = snap {
            add_snap_marker(&mut scene, hit, aperture, pick_z);
        }
        // When one or more operations are focused, dim every *other* operation's
        // toolpath so the focused ones stand out — vital when a part has dozens of
        // ops. An empty focus set leaves everything vivid.
        scene.focus_operations(focus_ops);
        // The simulated stock is drawn under the backplot, only when toggled on
        // and available (a run has produced it).
        let (mesh_vertices, mesh_indices) = match controller.outcome() {
            Some(outcome) if show_stock => (
                outcome.stock_vertices.clone(),
                outcome.stock_indices.clone(),
            ),
            _ => (Vec::new(), Vec::new()),
        };
        Self {
            vertices: Arc::new(scene.line_vertices()),
            mesh_vertices: Arc::new(mesh_vertices),
            mesh_indices: Arc::new(mesh_indices),
            bounds,
            controls,
            show_gizmo,
            gizmo_size,
            picking,
            set_origin,
            snap_engaged: snap.is_some(),
            pick_z,
        }
    }

    /// The orbit camera framed on the current scene, with the current controls.
    fn camera(&self) -> OrbitCamera {
        let (min, max) = self.bounds.unwrap_or(([0.0, 0.0, 0.0], [1.0, 1.0, 0.0]));
        let mut cam = OrbitCamera::framed(min, max);
        cam.orient = cam_render::orientation(self.controls.yaw, self.controls.pitch);
        cam.zoom = self.controls.zoom;
        cam.pan = self.controls.pan;
        cam
    }

    /// The orientation cube's camera: the same rotation as the part, framed on
    /// the unit cube (no zoom / pan).
    fn gizmo_camera(&self) -> OrbitCamera {
        let mut cam = OrbitCamera::framed([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
        cam.orient = cam_render::orientation(self.controls.yaw, self.controls.pitch);
        cam
    }

    /// If `pos` (absolute) lands on a gizmo face, the `(yaw, pitch)` to snap to.
    fn gizmo_pick(&self, pos: iced::Point, bounds: iced::Rectangle) -> Option<(f32, f32)> {
        if !self.show_gizmo {
            return None;
        }
        let (gx, gy, size) = gizmo_rect(bounds.width, bounds.height, self.gizmo_size, GIZMO_MARGIN);
        let (gxa, gya) = (bounds.x + gx, bounds.y + gy);
        if pos.x < gxa || pos.x > gxa + size || pos.y < gya || pos.y > gya + size {
            return None;
        }
        let u = 2.0 * (pos.x - gxa) / size - 1.0;
        let v = 1.0 - 2.0 * (pos.y - gya) / size; // screen-y down → NDC-y up
        let cam = self.gizmo_camera();
        let normal = cam_render::pick_face(cam.orient, cam.half_height(), u, v)?;
        Some(face_view(normal))
    }
}

impl shader::Program<Message> for Viewport {
    type State = ViewportState;
    type Primitive = ScenePrimitive;

    fn update(
        &self,
        state: &mut Self::State,
        event: &iced::Event,
        bounds: iced::Rectangle,
        cursor: iced::mouse::Cursor,
    ) -> Option<shader::Action<Message>> {
        let iced::Event::Mouse(mouse_event) = event else {
            return None;
        };
        use iced::mouse::{Button, Event as Mouse, ScrollDelta};
        match mouse_event {
            Mouse::ButtonPressed(button) => {
                let pos = cursor.position_over(bounds)?;
                // A left-press on a gizmo face *arms* a snap-to-view: it fires on
                // release only if the press stays a click (see ButtonReleased). We
                // still begin an orbit drag, so if the user drags — a rotation that
                // merely started over the cube — it orbits and the snap is dropped.
                if matches!(button, Button::Left) {
                    if let Some(view) = self.gizmo_pick(pos, bounds) {
                        state.gizmo_arm = Some(view);
                        state.drag = Some(DragMode::Orbit);
                        state.last = Some(pos);
                        state.press = Some(pos);
                        return Some(shader::Action::capture());
                    }
                }
                // In geometry-pick mode a left-click selects geometry (projected
                // onto the stock-top plane) instead of orbiting; the pickbox
                // half-size becomes the world-space snap aperture.
                if (self.picking || self.set_origin) && matches!(button, Button::Left) {
                    let aspect = if bounds.height > 0.0 {
                        bounds.width / bounds.height
                    } else {
                        1.0
                    };
                    let u = 2.0 * (pos.x - bounds.x) / bounds.width - 1.0;
                    let v = 1.0 - 2.0 * (pos.y - bounds.y) / bounds.height;
                    let cam = self.camera();
                    if let Some(w) = cam.pick_plane(u, v, aspect, self.pick_z) {
                        let aperture = 0.5 * SNAP_PICK_PX * cam.world_per_pixel(bounds.height);
                        let msg = if self.set_origin {
                            Message::OriginPointPicked(w, aperture)
                        } else {
                            Message::PickWorld(w, aperture)
                        };
                        return Some(shader::Action::publish(msg).and_capture());
                    }
                    return Some(shader::Action::capture());
                }
                // Leave presses in the pane-resize band along the edges to the
                // pane_grid divider — its grab zone bleeds a few px into the pane,
                // and capturing them here would orbit while the user resizes.
                if in_resize_band(pos, bounds) {
                    return None;
                }
                let mode = match button {
                    Button::Left => DragMode::Orbit,
                    Button::Right => DragMode::Pan,
                    _ => return None,
                };
                state.drag = Some(mode);
                state.last = Some(pos);
                return Some(shader::Action::capture());
            }
            Mouse::CursorMoved { position } => {
                if let (Some(mode), Some(last)) = (state.drag, state.last) {
                    // While a cube snap is armed, hold off orbiting: a small wiggle
                    // still counts as a click, but travel past the slop commits to
                    // an orbit and drops the snap.
                    if let Some(press) = state.press {
                        if state.gizmo_arm.is_some() {
                            let moved = (position.x - press.x).hypot(position.y - press.y);
                            if moved < GIZMO_CLICK_SLOP {
                                return Some(shader::Action::capture());
                            }
                            state.gizmo_arm = None;
                            state.last = Some(*position); // orbit from here, no jump
                            return Some(shader::Action::capture());
                        }
                    }
                    let (dx, dy) = (position.x - last.x, position.y - last.y);
                    state.last = Some(*position);
                    // Report a relative change; App owns and accumulates it.
                    let message = match mode {
                        DragMode::Orbit => Message::OrbitBy(dx * ORBIT_SENS, dy * ORBIT_SENS),
                        DragMode::Pan => {
                            let cam = self.camera();
                            let wpp = cam.world_per_pixel(bounds.height);
                            let (r, u) = (cam.right(), cam.up());
                            // Drag moves the scene with the cursor.
                            Message::PanBy([
                                (-r[0] * dx + u[0] * dy) * wpp,
                                (-r[1] * dx + u[1] * dy) * wpp,
                                (-r[2] * dx + u[2] * dy) * wpp,
                            ])
                        }
                    };
                    return Some(shader::Action::publish(message).and_capture());
                }
                // While a pick is pending (op or set-origin), track the cursor
                // (pickbox) and its world point (snap preview). Passive tracking.
                if (self.picking || self.set_origin) && cursor.position_over(bounds).is_some() {
                    let aspect = if bounds.height > 0.0 {
                        bounds.width / bounds.height
                    } else {
                        1.0
                    };
                    let u = 2.0 * (position.x - bounds.x) / bounds.width - 1.0;
                    let v = 1.0 - 2.0 * (position.y - bounds.y) / bounds.height;
                    let cam = self.camera();
                    let msg = match cam.pick_plane(u, v, aspect, self.pick_z) {
                        Some(w) => {
                            let aperture = 0.5 * SNAP_PICK_PX * cam.world_per_pixel(bounds.height);
                            Message::HoverWorld(*position, w, aperture)
                        }
                        None => Message::ViewportCursor(*position),
                    };
                    return Some(shader::Action::publish(msg));
                }
            }
            Mouse::ButtonReleased(_) => {
                // A still-armed press never became a drag: it's a click — snap now.
                if let Some((yaw, pitch)) = state.gizmo_arm.take() {
                    state.drag = None;
                    state.last = None;
                    state.press = None;
                    return Some(
                        shader::Action::publish(Message::SetView(yaw, pitch)).and_capture(),
                    );
                }
                if state.drag.take().is_some() {
                    state.last = None;
                    state.press = None;
                    return Some(shader::Action::capture());
                }
            }
            Mouse::WheelScrolled { delta } if cursor.position_over(bounds).is_some() => {
                let lines = match delta {
                    ScrollDelta::Lines { y, .. } => *y,
                    ScrollDelta::Pixels { y, .. } => *y / 20.0,
                };
                return Some(
                    shader::Action::publish(Message::ZoomBy(lines * ZOOM_SENS)).and_capture(),
                );
            }
            _ => {}
        }
        None
    }

    fn draw(
        &self,
        _state: &Self::State,
        _cursor: iced::mouse::Cursor,
        bounds: iced::Rectangle,
    ) -> Self::Primitive {
        let aspect = if bounds.height > 0.0 {
            bounds.width / bounds.height
        } else {
            1.0
        };
        ScenePrimitive {
            vertices: self.vertices.clone(),
            mesh_vertices: self.mesh_vertices.clone(),
            mesh_indices: self.mesh_indices.clone(),
            view_proj: self.camera().view_proj(aspect),
            gizmo_view_proj: self.gizmo_camera().view_proj(1.0),
            show_gizmo: self.show_gizmo,
            gizmo_size: self.gizmo_size,
            logical_width: bounds.width,
        }
    }

    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: iced::Rectangle,
        cursor: iced::mouse::Cursor,
    ) -> iced::mouse::Interaction {
        let over = cursor.position_over(bounds).is_some();
        if (self.picking || self.set_origin) && over {
            // Aiming a pick: a crosshair, but a plain arrow once a snap engages so
            // its marker reads on its own. Never the orbit "hand" while picking.
            if self.snap_engaged {
                iced::mouse::Interaction::default()
            } else {
                iced::mouse::Interaction::Crosshair
            }
        } else if state.drag.is_some() {
            iced::mouse::Interaction::Grabbing
        } else if over {
            iced::mouse::Interaction::Grab
        } else {
            iced::mouse::Interaction::default()
        }
    }
}

/// A depth texture sized to the render target.
struct DepthTarget {
    view: wgpu::TextureView,
    size: (u32, u32),
    _texture: wgpu::Texture,
}

/// The shared GPU state for the viewport — iced constructs it once and hands it
/// back to us each frame. It owns both renderers (solid stock drawn first, the
/// backplot lines over it) and a depth buffer so the rotated solid occludes
/// itself correctly.
struct ViewportPipeline {
    lines: cam_render::LineRenderer,
    mesh: cam_render::MeshRenderer,
    gizmo: cam_render::GizmoRenderer,
    depth: Option<DepthTarget>,
}

impl shader::Pipeline for ViewportPipeline {
    fn new(device: &wgpu::Device, queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        ViewportPipeline {
            lines: cam_render::LineRenderer::new(device, format),
            mesh: cam_render::MeshRenderer::new(device, format),
            gizmo: cam_render::GizmoRenderer::new(device, queue, format),
            depth: None,
        }
    }
}

impl ViewportPipeline {
    /// Ensure a depth texture matching the full render target (`size` pixels).
    fn ensure_depth(&mut self, device: &wgpu::Device, size: (u32, u32)) {
        if self.depth.as_ref().map(|d| d.size) == Some(size) {
            return;
        }
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("cam-render viewport depth"),
            size: wgpu::Extent3d {
                width: size.0,
                height: size.1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: cam_render::DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        self.depth = Some(DepthTarget {
            view,
            size,
            _texture: texture,
        });
    }
}

#[derive(Debug)]
struct ScenePrimitive {
    vertices: Arc<Vec<Vertex>>,
    mesh_vertices: Arc<Vec<MeshVertex>>,
    mesh_indices: Arc<Vec<u32>>,
    view_proj: [[f32; 4]; 4],
    /// The orientation cube's view-projection (same rotation, its own framing).
    gizmo_view_proj: [[f32; 4]; 4],
    /// Whether to draw the orientation cube.
    show_gizmo: bool,
    /// The cube's on-screen edge length, logical px (fixed; window-independent).
    gizmo_size: f32,
    /// The widget's logical width, to recover the physical-per-logical scale in
    /// `render` (which is handed physical `clip_bounds`) and size the cube in px.
    logical_width: f32,
}

impl shader::Primitive for ScenePrimitive {
    type Pipeline = ViewportPipeline;

    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _bounds: &iced::Rectangle,
        viewport: &shader::Viewport,
    ) {
        pipeline.lines.upload(device, &self.vertices);
        pipeline
            .mesh
            .upload(device, &self.mesh_vertices, &self.mesh_indices);
        pipeline.lines.set_camera(queue, self.view_proj);
        pipeline.mesh.set_camera(queue, self.view_proj);
        pipeline.gizmo.set_camera(queue, self.gizmo_view_proj);
        let size = viewport.physical_size();
        pipeline.ensure_depth(device, (size.width.max(1), size.height.max(1)));
    }

    fn render(
        &self,
        pipeline: &Self::Pipeline,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        clip_bounds: &iced::Rectangle<u32>,
    ) {
        let Some(depth) = &pipeline.depth else {
            return;
        };
        if clip_bounds.width == 0 || clip_bounds.height == 0 {
            return;
        }
        // Our own pass: preserve iced's UI (Load), clear only our depth. We draw
        // the solid (depth-writing) then the backplot lines (always on top),
        // scissored to this widget's rectangle.
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("cam-render viewport pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &depth.view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });
        pass.set_viewport(
            clip_bounds.x as f32,
            clip_bounds.y as f32,
            clip_bounds.width as f32,
            clip_bounds.height as f32,
            0.0,
            1.0,
        );
        pass.set_scissor_rect(
            clip_bounds.x,
            clip_bounds.y,
            clip_bounds.width,
            clip_bounds.height,
        );
        pipeline.mesh.draw(&mut pass);
        pipeline.lines.draw(&mut pass);
        drop(pass);

        // Second pass: the orientation cube in the top-right corner, with its own
        // depth cleared so the part never occludes it. Same rect as the click
        // hit-test (gizmo_rect), offset into the widget's clip region.
        if !self.show_gizmo {
            return;
        }
        // `clip_bounds` is physical px; the cube size is fixed in logical px.
        // Recover the physical-per-logical scale from the widget's logical width
        // so the cube stays the same on-screen size at any DPI / window size.
        let scale = if self.logical_width > 0.0 {
            clip_bounds.width as f32 / self.logical_width
        } else {
            1.0
        };
        let (lx, ly, size) = gizmo_rect(
            clip_bounds.width as f32,
            clip_bounds.height as f32,
            self.gizmo_size * scale,
            GIZMO_MARGIN * scale,
        );
        let size = size as u32;
        if size == 0 {
            return;
        }
        let gx = clip_bounds.x + lx as u32;
        let gy = clip_bounds.y + ly as u32;
        let mut gizmo_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("cam-render gizmo pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &depth.view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });
        gizmo_pass.set_viewport(gx as f32, gy as f32, size as f32, size as f32, 0.0, 1.0);
        gizmo_pass.set_scissor_rect(gx, gy, size, size);
        pipeline.gizmo.draw(&mut gizmo_pass);
    }
}

#[cfg(test)]
mod ribbon_tests {
    use super::*;

    // Home tab shape: File(2) + Edit(3).
    const HOME: [usize; 2] = [2, 3];

    #[test]
    fn wide_window_keeps_all_groups_full() {
        let d = solve_densities(&HOME, 2000.0);
        assert_eq!(d, vec![Density::Full, Density::Full]);
    }

    #[test]
    fn narrowing_degrades_right_group_first() {
        // A width that fits Full+Compact but not Full+Full: the rightmost (Edit)
        // group must be the one that drops.
        let full_full = row_width(&HOME, &[Density::Full, Density::Full]);
        let full_compact = row_width(&HOME, &[Density::Full, Density::Compact]);
        let w = 0.5 * (full_full + full_compact); // between the two
        let d = solve_densities(&HOME, w);
        assert_eq!(d, vec![Density::Full, Density::Compact]);
    }

    #[test]
    fn very_narrow_collapses_everything_to_tight() {
        let d = solve_densities(&HOME, 1.0);
        assert_eq!(d, vec![Density::Tight, Density::Tight]);
    }

    #[test]
    fn degradation_is_monotonic_in_width() {
        // As available width shrinks, total tightness never decreases.
        let tightness = |d: &[Density]| d.iter().map(|&x| x as u8 as u32).sum::<u32>();
        let mut prev = 0;
        for w in (0..1200).step_by(20).map(|x| x as f32) {
            let t = tightness(&solve_densities(&HOME, w));
            // Wider (later, larger w) must be <= tighter earlier — iterate ascending
            // width and assert non-increasing tightness relative to the previous
            // (narrower) width.
            assert!(t <= prev || prev == 0, "tightness rose as width grew");
            prev = t;
        }
    }
}
