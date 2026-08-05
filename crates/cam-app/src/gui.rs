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
    button, canvas, checkbox, column, container, mouse_area, pick_list, row, scrollable, shader,
    slider, text, text_input, tooltip, Space,
};
use iced::{Alignment, Background, Border, Color, Element, Length, Padding};

use cam_model::{
    Axis, CarveClearing, ClearParams, CutDir, Hand, Lead, Operation, Plunge, Side, ToolKind,
};
use cam_post::PostKind;

use crate::project::OcamFile;
// `ToolKindPick`/`families_for` are plain data (an operation's tool families), so they
// live with the library rather than in this shell -- that is what lets the headless
// tests assert the starter library can start every operation.
use crate::tool_library::{families_for, ToolKindPick, ToolLibrary};

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
    /// Error accent (red) for an invalid inspector field. A thickened border carries the
    /// same signal by weight, so it does not rely on hue alone.
    pub const ERROR: Color = rgb(0.92, 0.30, 0.28);
}

/// The smallest tip **radius** a ground pointed tool can physically have, in mm.
///
/// Neither a chamfer mill nor a V-bit has a true `r = 0` point — it cannot be ground
/// and would not survive contact. 50 µm is the practical floor for a ground tip; below
/// it the number is fiction and the geometry degenerates (a V-bit's tip arc collapses,
/// and a chamfer mill's non-cutting flat — the very thing that distinguishes it from a
/// V-bit — disappears).
///
/// Applied as a **radius** to both, so the two are held to the same physical size: a
/// V-bit's tip radius directly, a chamfer mill's tip *diameter* at twice this.
pub const MIN_TIP_RADIUS_MM: f64 = 0.05;

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
    Engrave,
    Carve,
    Face,
    NewTool,
    Renumber,
    ImportLibrary,
    ExportLibrary,
    Duplicate,
    Delete,
    ShowStock,
    ResetView,
    ShowCube,
    SetOrigin,
    Info,
    Machine,
    License,
}

impl Icon {
    /// Every icon, so tests can sweep the whole bundled set. Kept beside the enum:
    /// a variant added without a line here fails `every_icon_is_listed_in_all`.
    #[cfg(test)]
    const ALL: [Icon; 29] = [
        Icon::New,
        Icon::Open,
        Icon::Save,
        Icon::Import,
        Icon::Export,
        Icon::Undo,
        Icon::Redo,
        Icon::Run,
        Icon::Profile,
        Icon::Pocket,
        Icon::Drill,
        Icon::Thread,
        Icon::Chamfer,
        Icon::Engrave,
        Icon::Carve,
        Icon::Face,
        Icon::NewTool,
        Icon::Renumber,
        Icon::ImportLibrary,
        Icon::ExportLibrary,
        Icon::Duplicate,
        Icon::Delete,
        Icon::ShowStock,
        Icon::ResetView,
        Icon::ShowCube,
        Icon::SetOrigin,
        Icon::Info,
        Icon::Machine,
        Icon::License,
    ];
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
            Icon::Engrave => include_bytes!("../assets/icons/engrave.svg"),
            Icon::Carve => include_bytes!("../assets/icons/carve.svg"),
            Icon::Face => include_bytes!("../assets/icons/face.svg"),
            Icon::NewTool => include_bytes!("../assets/icons/endmill.svg"),
            Icon::Renumber => include_bytes!("../assets/icons/renumber.svg"),
            Icon::ImportLibrary => include_bytes!("../assets/icons/import_library.svg"),
            Icon::ExportLibrary => include_bytes!("../assets/icons/export_library.svg"),
            Icon::Duplicate => include_bytes!("../assets/icons/copy.svg"),
            Icon::Delete => include_bytes!("../assets/icons/erase.svg"),
            Icon::ShowStock => include_bytes!("../assets/icons/box3d.svg"),
            Icon::ResetView => include_bytes!("../assets/icons/zoom_ext.svg"),
            Icon::ShowCube => include_bytes!("../assets/icons/viewcube.svg"),
            Icon::SetOrigin => include_bytes!("../assets/icons/origin.svg"),
            Icon::Info => include_bytes!("../assets/icons/info.svg"),
            Icon::Machine => include_bytes!("../assets/icons/machine.svg"),
            Icon::License => include_bytes!("../assets/icons/license.svg"),
        }
    }

    /// A one-line description of the command this icon stands for, shown as a hover
    /// tooltip on its ribbon button (the label under the icon is terser, and vanishes
    /// entirely at Compact density — the tooltip is where the icon explains itself).
    fn help(self) -> &'static str {
        match self {
            Icon::New => "Start a new, empty project.",
            Icon::Open => "Open a saved .ocam project from disk.",
            Icon::Save => "Save the current project to disk.",
            Icon::Import => "Import part geometry from a DXF or DWG drawing.",
            Icon::Export => "Export the generated toolpaths as G-code (NC) for your machine.",
            Icon::Undo => "Undo the last change.",
            Icon::Redo => "Redo the change just undone.",
            Icon::Run => "Generate toolpaths for every operation and simulate the result.",
            Icon::Profile => "Add a Profile operation — cut along a chain, inside/outside/on the line.",
            Icon::Pocket => "Add a Pocket operation — clear the area inside a closed boundary.",
            Icon::Drill => "Add a Drill operation — peck/drill the selected hole circles.",
            Icon::Thread => "Add a Thread-milling operation on a bore or boss.",
            Icon::Chamfer => "Add a Chamfer operation — break an edge with a V/chamfer tool.",
            Icon::Engrave => "Add an Engrave operation — plough a V-groove along a path with a V-bit. \
                              Groove width follows from the depth and the bit's angle; a chamfer mill \
                              will not do (its tip does not cut).",
            Icon::Carve => "Add a Carve operation — carve out the AREA a closed boundary encloses \
                            with a V-bit. Unlike engraving, the tool never touches the boundary: its \
                            flanks land on it, and the depth follows from the shape's own width.",
            Icon::Face => "Add a Face operation — skim the top of the stock flat.",
            Icon::NewTool => "Add a new tool to the library.",
            Icon::Renumber => {
                "Renumber the whole library sequentially in the current tab order \
                 (asks for confirmation — it rewrites every tool number)."
            }
            Icon::ImportLibrary => {
                "Import a tool library from a .ocam file, replacing the working library."
            }
            Icon::ExportLibrary => "Export the working tool library to a .ocam file to share.",
            Icon::Duplicate => "Duplicate the selected operation.",
            Icon::Delete => "Delete the selected item.",
            Icon::ShowStock => "Show or hide the simulated stock surface in the viewport.",
            Icon::ResetView => "Reset the camera to frame the whole part.",
            Icon::ShowCube => "Show or hide the orientation cube.",
            Icon::SetOrigin => "Show or hide the workpiece-origin datum marker.",
            Icon::Info => "Show or hide hover tooltips on inspector fields and ribbon icons.",
            Icon::Machine => {
                "Set up the machine — working travel and the post-processor (G-code dialect, \
                 e.g. grbl / FluidNC / Fanuc) the exported NC is written for."
            }
            Icon::License => {
                "Licence and credits — the GNU GPL v3 this program is released under, \
                 and the third-party work it builds on."
            }
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
use cam_cldata::{Program, Step};
use cam_render::{MeshVertex, OrbitCamera, Scene, Vertex, ENVELOPE, ENVELOPE_OVER, PART};
use cam_toolpath::{CancelToken, Severity};

use crate::{
    op_accepts_open_paths, op_selects_circles, op_takes_islands, AppController, CuttingData,
    LoopRef, OpKind,
    PendingOp,
    PickResult, Selection, SnapHit,
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
        .title("Open CAM Studio")
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

/// Native error dialog for an export that cannot happen.
///
/// A dialog rather than the status line, because the status line lives in the **Output**
/// pane and the operator can hide it — at which point a refusal would be entirely
/// silent. A blocked export is exactly the case that must not be.
async fn report_export_blocked(detail: String) {
    rfd::AsyncMessageDialog::new()
        .set_level(rfd::MessageLevel::Error)
        .set_title("Cannot export")
        .set_description(detail)
        .set_buttons(rfd::MessageButtons::Ok)
        .show()
        .await;
}

/// Native Yes/No warning gating the **bulk renumber** (it rewrites every tool's number).
async fn confirm_renumber(detail: String) -> bool {
    rfd::AsyncMessageDialog::new()
        .set_level(rfd::MessageLevel::Warning)
        .set_title("Renumber tools")
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
    Library,
    Viewport,
    Inspector,
    Output,
}

impl Pane {
    fn name(self) -> &'static str {
        match self {
            Pane::Project => "Project",
            Pane::Library => "Tool Library",
            Pane::Viewport => "Viewport",
            Pane::Inspector => "Inspector",
            Pane::Output => "Output",
        }
    }

    /// The short label this pane uses in the ribbon's **Panes** group. Only the tool
    /// library differs: it compacts to "Tools" so the group stays narrow in a band
    /// that already carries five commands and the cube slider. The pane's own title
    /// bar keeps the full name from [`name`](Self::name).
    fn ribbon_label(self) -> &'static str {
        match self {
            Pane::Library => "Tools",
            other => other.name(),
        }
    }

    /// This pane's minimum size (px) along whichever axis it is split — enforced
    /// individually while resizing (see `App::clamp_resize`).
    /// The size this pane may not shrink past, from the user's preferences.
    ///
    /// Was a hard-coded match. The shipped defaults still fit real content — the
    /// Project pane's Duplicate/Delete row, the Inspector's field rows — but they were
    /// chosen on one monitor, and a value picked on one display is a bug on another
    /// (a 240 px Inspector plus a 200 px Project eats a third of a 1366-wide screen).
    fn min_size(self, prefs: &crate::PanePrefs) -> f32 {
        match self {
            Pane::Project => prefs.min_project_px,
            Pane::Library => prefs.min_library_px,
            Pane::Viewport => prefs.min_viewport_px,
            Pane::Inspector => prefs.min_inspector_px,
            Pane::Output => prefs.min_output_px,
        }
    }

    /// The edge this pane docks to when re-shown. The Viewport has no fixed edge
    /// — it simply takes the space where it is split back in.
    fn dock_edge(self) -> Option<pane_grid::Edge> {
        match self {
            // Library docks Left, like Project — it substitutes for it on the
            // Tooling tab, and can also sit alongside it.
            Pane::Project | Pane::Library => Some(pane_grid::Edge::Left),
            Pane::Inspector => Some(pane_grid::Edge::Right),
            Pane::Output => Some(pane_grid::Edge::Bottom),
            Pane::Viewport => None,
        }
    }
}

/// Every pane, in Windows-menu order.
const ALL_PANES: [Pane; 5] = [
    Pane::Project,
    Pane::Library,
    Pane::Viewport,
    Pane::Inspector,
    Pane::Output,
];

/// The Library-pane right-click menu state. Starts as a "Set number…" item; clicking it
/// morphs the same popup into a number input (`input = Some(buffer)`).
#[derive(Clone, Debug)]
struct LibMenu {
    index: usize,
    input: Option<String>,
}

/// How the Tool Library pane lists its tools — the pane's two internal tabs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LibraryView {
    /// Ordered by tool number (T1, T2, …).
    Ordered,
    /// Grouped by tool family (all end mills, then drills, …), sorted by diameter.
    Grouped,
}

/// A tab in the top-bar ribbon. Each tab shows a band of grouped commands.
/// Operations and Tooling are added as those capabilities land.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RibbonTab {
    Home,
    Operations,
    Edit,
    Tooling,
    /// The shop's machines. Beside **Tooling** deliberately: both are *installation*
    /// scope, not document scope — a machine is never stored in a project (a file that
    /// could set yours could disarm the travel check), exactly as the tool library is
    /// not. `Edit` is the document, which is why Machine did not belong there.
    Machinery,
    View,
}

impl RibbonTab {
    /// The tabs shown in the strip, left to right.
    const ALL: [RibbonTab; 6] = [
        RibbonTab::Home,
        RibbonTab::Edit,
        RibbonTab::Operations,
        RibbonTab::Tooling,
        RibbonTab::Machinery,
        RibbonTab::View,
    ];

    fn label(self) -> &'static str {
        match self {
            RibbonTab::Home => "Home",
            RibbonTab::Operations => "Operations",
            RibbonTab::Edit => "Edit",
            RibbonTab::Tooling => "Tooling",
            RibbonTab::Machinery => "Machinery",
            RibbonTab::View => "View",
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
            PlungeKind::Ramp => "Ramp along path",
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

/// A thread mill's cutting form — the single-tooth vs multiple-teeth toggle. Maps onto
/// `ThreadMill { pitch }`: single-point ⇔ `None` (one tooth, cuts any pitch by its
/// helical lead), full-form ⇔ `Some(pitch)` (a stack of teeth at a fixed pitch).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ThreadForm {
    SinglePoint,
    FullForm,
}

impl ThreadForm {
    const ALL: [ThreadForm; 2] = [ThreadForm::SinglePoint, ThreadForm::FullForm];
}

impl std::fmt::Display for ThreadForm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            ThreadForm::SinglePoint => "Single-point",
            ThreadForm::FullForm => "Full-form",
        })
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
        ToolKind::VBit { .. } => vec![Field::PointAngle, Field::TipRadius],
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
        (ToolKind::VBit { included_angle_deg, .. }, Field::PointAngle) => Some(included_angle_deg),
        (ToolKind::VBit { tip_radius, .. }, Field::TipRadius) => Some(tip_radius),
        _ => None,
    }
}

/// Write the parsed inspector fields onto a tool's kind-specific parameters.
/// Write the parsed common tool dimensions onto `t`, enforcing the length constraint
/// **overall = flute + shank**: editing flute or shank recomputes overall; editing
/// overall recomputes the shank (flute held fixed), with shank floored at 0. The
/// effective flute (`flute_len()`) is materialised into `flute_length` on any edit so
/// the three stay consistent. Kind-specific parameters are handled separately by
/// [`apply_tool_kind_fields`].
fn apply_tool_dims(t: &mut cam_model::Tool, parsed: &BTreeMap<Field, f64>) {
    if let Some(&v) = parsed.get(&Field::ToolDiameter) {
        t.diameter = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::ShankDiameter) {
        t.shank_diameter = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::NeckLength) {
        t.neck_length = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::NeckDiameter) {
        t.neck_diameter = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::Flutes) {
        t.flutes = v.round().max(1.0) as u32;
    }
    // Nominal cutting data (library defaults). Clamped non-negative; 0 = unset.
    if let Some(&v) = parsed.get(&Field::NominalRpm) {
        t.nominal_rpm = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::NominalFeed) {
        t.nominal_feed = v.max(0.0);
    }
    if let Some(&v) = parsed.get(&Field::NominalPlungeFeed) {
        t.nominal_plunge_feed = v.max(0.0);
    }

    // overall = flute + shank. Snapshot the effective flute/shank *before* the edit.
    let old_flute = t.flute_len();
    let old_shank = (t.length - old_flute).max(0.0);
    let new_flute = parsed
        .get(&Field::FluteLength)
        .map(|&v| v.max(0.0))
        .unwrap_or(old_flute);
    t.flute_length = new_flute; // materialise (drops the 0-sentinel)
    let new_length = if let Some(&sh) = parsed.get(&Field::ShankLength) {
        new_flute + sh.max(0.0) // shank edited ⇒ overall follows
    } else if let Some(&ov) = parsed.get(&Field::ToolLength) {
        ov // overall edited ⇒ shank absorbs (derived below)
    } else {
        new_flute + old_shank // flute may have changed; preserve the shank
    };
    t.length = new_length.max(new_flute); // shank ≥ 0
}

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
                // The single-point/full-form choice is the toggle's job; the pitch field
                // only sets the value, and only when already full-form.
                if pitch.is_some() && v > 0.0 {
                    *pitch = Some(v);
                }
            }
        }
        ToolKind::VBit {
            included_angle_deg,
            tip_radius,
        } => {
            if let Some(v) = get(Field::PointAngle) {
                *included_angle_deg = v;
            }
            if let Some(v) = get(Field::TipRadius) {
                *tip_radius = v.max(0.0);
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
    /// Setup: the Z the planner lifts to before a tool change or reorientation.
    ToolChangeHeight,
    /// Stock: material added on both X sides of the part's bounding box (mm).
    StockXOffset,
    /// Stock: material added on both Y sides of the part's bounding box (mm).
    StockYOffset,
    /// Stock: absolute Z of the top surface (mm).
    StockTop,
    /// Stock: block thickness below the top (mm); bottom = top − thickness.
    StockThickness,
    /// Machine travel (working volume) on X / Y / Z, mm.
    MachineTravelX,
    MachineTravelY,
    MachineTravelZ,
    /// Workpiece origin (datum) X / Y / Z, part-space mm.
    OriginX,
    OriginY,
    OriginZ,
    /// The active origin's machine work-coordinate index (`G54`-`G59`, or
    /// `G15 H<n>` on Okuma — see `PostKind::datum_label`).
    OriginIndex,
    ToolDiameter,
    ToolLength,
    /// Length of cut (flute length) from the tip, mm. Also reused, kind-aware, as a face
    /// mill's body height and a thread mill's thread length.
    FluteLength,
    /// Shank diameter, mm (a face mill's arbor diameter, kind-aware).
    ShankDiameter,
    /// Shank length, mm — a *derived* field: overall = flute + shank (editing it moves
    /// overall length; editing overall moves it back).
    ShankLength,
    /// Reduced-neck length from the cutting end, mm; 0 = no reduced neck.
    NeckLength,
    /// Reduced-neck diameter, mm; 0 = same as the cutting diameter.
    NeckDiameter,
    Flutes,
    /// Tool nominal spindle speed (rpm) — the library default that seeds a new
    /// operation's own RPM. `0` = unset.
    NominalRpm,
    /// Tool nominal cutting feed (mm/min) — seeds a new operation's feed. `0` = unset.
    NominalFeed,
    /// Tool nominal plunge feed (mm/min) — seeds a new operation's plunge feed.
    /// `0` = unset.
    NominalPlungeFeed,
    /// Bull-nose corner radius (mm).
    CornerRadius,
    /// Chamfer/V mill included point angle (deg).
    ChamferAngle,
    /// Chamfer/V mill flat-tip diameter (mm).
    TipDiameter,
    /// V-bit rounded-tip radius (mm).
    TipRadius,
    /// Drill point angle (deg).
    PointAngle,
    /// Thread mill's ground pitch (mm); 0 means single-form (any pitch).
    ToolThreadPitch,
    Depth,
    /// Drill start plane, as a distance below the stock top (mm); 0 = at the top.
    DrillStartOffset,
    /// Drill peck increment (mm); 0 = drill straight to depth (no peck).
    Peck,
    /// Drill dwell at the hole bottom (seconds); 0 = no dwell.
    Dwell,
    Stepdown,
    Stepover,
    /// Profile finishing allowance left on the wall (mm).
    ProfileOffset,
    /// Operation spindle speed (rpm) — `M3 S<rpm>`. Seeded from the tool's nominal
    /// RPM when the operation is created; `0` falls back to the job default.
    SpindleRpm,
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
    /// Thread radial infeed passes (count, ≥1).
    ThreadPasses,
    /// Thread spring passes at full depth (count, 0 = none).
    ThreadSpringPasses,
    /// Thread blind-hole drill clearance below the thread bottom (mm); 0 = through hole.
    ThreadDrillClearance,
    /// Thread blind-hole required allowance (mm); 0 = auto (one pitch).
    ThreadBlindAllowance,
    /// Chamfer: absolute Z of the top edge being chamfered (mm). Seeded from the
    /// stock top, editable because the edge is not always there.
    ChamferTop,
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
    Overlap,
    /// Face: overshoot past the stock edge before the turnaround (mm).
    FaceOvershoot,
    /// Adaptive-clearing stepover: the radial width of cut along a *straight* wall
    /// (mm); 0 disables adaptive clearing (pocket, profile outside-roughing). Not a
    /// hard ceiling — see the tooltip on the geometric floor.
    Engagement,
    /// Plunge parameter A: ramp/zig-zag angle, or helix radius.
    PlungeA,
    /// Plunge parameter B: zig-zag length, or helix pitch.
    PlungeB,
    /// Carve: radial spacing of the carve rings (mm); 0 = auto.
    RingStep,
    /// Carve clearing pass: max depth per pass (mm); 0 = full depth in one.
    ClearStepdown,
    /// Carve clearing pass: ring overlap as a percentage of the tool diameter.
    ClearOverlap,
    /// Carve clearing pass: adaptive stepover along a straight wall (mm); 0 = plain
    /// concentric.
    ClearEngagement,
    /// Carve clearing pass: cutting feed (mm/min); 0 inherits the carve's.
    ClearFeed,
    /// Carve clearing pass: plunge feed (mm/min); 0 inherits the carve's.
    ClearPlungeFeed,
    /// Carve clearing pass: closure overlap past the start (mm).
    ClearLeadOverlap,
    /// Carve clearing pass: lead-in size (mm).
    ClearLeadInSize,
    /// Carve clearing pass: lead-out size (mm).
    ClearLeadOutSize,
    /// Carve clearing pass: plunge parameter A (ramp/zig-zag angle, or helix radius).
    ClearPlungeA,
    /// Carve clearing pass: plunge parameter B (zig-zag length, or helix pitch).
    ClearPlungeB,
    /// Carve clearing pass: allowance left off the carved surface for the V-bit (mm).
    ClearOffset,
    /// Carve: target ridge height left on the flat floor (mm); 0 = auto.
    Scallop,
}

impl Field {
    fn label(self) -> &'static str {
        match self {
            Field::Clearance => "Clearance (mm)",
            Field::Retract => "Retract (mm)",
            Field::ToolChangeHeight => "Tool Change Height (mm)",
            Field::TopOfStock => "Top of stock (mm)",
            Field::StockXOffset => "X offset (mm)",
            Field::StockYOffset => "Y offset (mm)",
            Field::StockTop => "Stock top (mm)",
            Field::StockThickness => "Thickness (mm)",
            Field::MachineTravelX => "X travel (mm)",
            Field::MachineTravelY => "Y travel (mm)",
            Field::MachineTravelZ => "Z travel (mm)",
            Field::OriginX => "Origin X (mm)",
            Field::OriginY => "Origin Y (mm)",
            Field::OriginZ => "Origin Z (mm)",
            Field::OriginIndex => "H index",
            Field::ToolDiameter => "Flute diameter (mm)",
            Field::ToolLength => "Overall length (mm)",
            Field::FluteLength => "Flute length (mm)",
            Field::ShankDiameter => "Shank diameter (mm)",
            Field::ShankLength => "Shank length (mm)",
            Field::NeckLength => "Neck length (mm, 0=none)",
            Field::NeckDiameter => "Neck ⌀ (mm, 0=cut ⌀)",
            Field::Flutes => "Flutes",
            Field::NominalRpm => "Nominal RPM",
            Field::NominalFeed => "Nominal feed (mm/min)",
            Field::NominalPlungeFeed => "Nominal plunge (mm/min)",
            Field::CornerRadius => "Corner radius (mm)",
            Field::ChamferAngle => "Point angle (deg)",
            Field::TipDiameter => "Tip ⌀ (mm)",
            Field::TipRadius => "Tip radius (mm)",
            Field::PointAngle => "Point angle (deg)",
            Field::ToolThreadPitch => "Pitch (mm)",
            Field::Depth => "Depth (mm)",
            Field::DrillStartOffset => "Start offset (mm)",
            Field::Peck => "Peck (mm, 0=off)",
            Field::Dwell => "Dwell (s, 0=off)",
            Field::ProfileOffset => "Offset / leave (mm)",
            Field::Stepdown => "Stepdown (mm)",
            Field::Stepover => "Stepover (mm)",
            Field::SpindleRpm => "Spindle (rpm)",
            Field::Feed => "Feed (mm/min)",
            Field::PlungeFeed => "Plunge feed (mm/min)",
            Field::MajorDia => "Major ⌀ (mm)",
            Field::Pitch => "Pitch (mm)",
            Field::ThreadTop => "Thread top (mm)",
            Field::ThreadBottom => "Thread bottom (mm)",
            Field::ThreadPasses => "Passes",
            Field::ThreadSpringPasses => "Spring passes",
            Field::ThreadDrillClearance => "Drill clearance (mm, 0=through)",
            Field::ThreadBlindAllowance => "Blind allowance (mm, 0=auto)",
            Field::ChamferTop => "Top edge Z (mm)",
            Field::ChamferWidth => "Chamfer width (mm)",
            Field::ChamferDepth => "Tip depth (mm, 0=tip)",
            Field::ChamferStep => "Step (mm, 0=one pass)",
            Field::LeadInSize => "Lead-in size (mm)",
            Field::LeadOutSize => "Lead-out size (mm)",
            Field::LeadOverlap => "Lead overlap (mm)",
            Field::FaceStartOffset => "Start offset (mm)",
            Field::Overlap => "Overlap (%)",
            Field::FaceOvershoot => "Overshoot (mm)",
            Field::Engagement => "Adaptive stepover (mm, 0=off)",
            Field::RingStep => "Ring step (mm, 0=auto)",
            Field::ClearStepdown => "Clear stepdown (mm, 0=full)",
            Field::ClearOverlap => "Clear overlap (%)",
            Field::ClearEngagement => "Clear adaptive stepover (0=off)",
            Field::ClearFeed => "Clear feed (mm/min, 0=same)",
            Field::ClearPlungeFeed => "Clear plunge feed (0=same)",
            Field::ClearLeadOverlap => "Clear lead overlap (mm)",
            Field::ClearLeadInSize => "Clear lead-in size (mm)",
            Field::ClearLeadOutSize => "Clear lead-out size (mm)",
            Field::ClearPlungeA => "Clearing plunge parameter 1",
            Field::ClearPlungeB => "Clearing plunge parameter 2",
            Field::ClearOffset => "Clear allowance (mm)",
            Field::Scallop => "Floor scallop (mm, 0=auto)",
            // Fallbacks only: `plunge_label` names these per style (a ramp's is an
            // angle, a helix's a radius — never both).
            Field::PlungeA => "Plunge parameter 1",
            Field::PlungeB => "Plunge parameter 2",
        }
    }

    /// A one- or two-sentence, new-user-friendly explanation of what the field does,
    /// shown as a hover tooltip on its label in the inspector.
    fn help(self) -> &'static str {
        match self {
            Field::Clearance => {
                "Safe height for rapid (non-cutting) moves above the part. The tool \
                 traverses at this Z between features so it never drags through stock."
            }
            Field::Retract => {
                "Height the tool pulls back to between passes or plunges — lower than \
                 Clearance for speed, but still above any uncut material."
            }
            Field::ToolChangeHeight => {
                "Z the tool lifts to before a tool change (M6) or a manual reorientation \
                 (M00) — above the Clearance plane, so the changer or your hands have room. \
                 Defaults to the top of the machine's Z travel; set it lower only if you \
                 have a reason."
            }
            Field::TopOfStock => {
                "Absolute Z of the top surface of the raw material. Cutting depths are \
                 measured downward from here."
            }
            Field::StockXOffset => {
                "Extra material left on both +X and −X sides of the part's bounding box \
                 — the raw block is this much wider than the part in X."
            }
            Field::StockYOffset => {
                "Extra material left on both +Y and −Y sides of the part's bounding box \
                 — the raw block is this much wider than the part in Y."
            }
            Field::StockTop => "Absolute Z of the stock's top surface (mm).",
            Field::StockThickness => {
                "Height of the raw block below its top: the bottom sits at (top − \
                 thickness)."
            }
            Field::MachineTravelX => {
                "Working-envelope size in X. Toolpaths that exceed it are flagged so a \
                 program can't command the machine past its limits."
            }
            Field::MachineTravelY => "Working-envelope size in Y (see X travel).",
            Field::MachineTravelZ => "Working-envelope size in Z (see X travel).",
            Field::OriginX => {
                "Part-space X of the workpiece datum — the (0,0,0) the G-code is written \
                 about. Set it to the corner or feature you'll touch off on the machine."
            }
            Field::OriginY => "Part-space Y of the workpiece datum (see Origin X).",
            Field::OriginZ => "Part-space Z of the workpiece datum (see Origin X).",
            Field::OriginIndex => {
                "The machine work-coordinate index this origin selects — the fixture \
                 offset you've taught on the control. How it is written depends on the \
                 post: G54-G59 on ISO controls (so only six exist), G15 H<n> on Okuma. \
                 Choosing an index another origin already uses swaps the two."
            }
            Field::ToolDiameter => "The tool's cutting diameter (mm).",
            Field::ToolLength => {
                "Overall length of the tool — the stickout below the holder. Used for \
                 reach checks and the backplot; does not change the cutting geometry."
            }
            Field::FluteLength => {
                "Length of the cutting edge (length of cut) measured from the tip. The \
                 rest of the tool, up to the overall length, is the non-cutting shank; \
                 the split is used for gouge/collision checks."
            }
            Field::ShankDiameter => {
                "Diameter of the non-cutting shank above the cutting portion (typically \
                 equal to the cutting diameter)."
            }
            Field::ShankLength => {
                "Length of the non-cutting shank above the cutting portion. Overall length \
                 = cutting length + shank length; editing the shank moves the overall \
                 length, and editing the overall length moves the shank."
            }
            Field::NeckLength => {
                "Length of a reduced-diameter neck above the flutes (reach/stub tools). \
                 0 = no reduced neck."
            }
            Field::NeckDiameter => {
                "Diameter of the reduced neck. 0 = the same as the cutting diameter."
            }
            Field::Flutes => {
                "Number of cutting edges (or inserts). Informational for now — \
                 feed-per-tooth math comes later; it does not change the toolpath."
            }
            Field::NominalRpm => {
                "Default spindle speed for this tool (rpm). Seeds a new operation's RPM \
                 when the tool is chosen; the operation can override it. 0 = unset."
            }
            Field::NominalFeed => {
                "Default cutting feed for this tool (mm/min). Seeds a new operation's feed; \
                 the operation can override it. A per-tool feed is only a starting point — \
                 it depends on the cut too. 0 = unset (uses the job default)."
            }
            Field::NominalPlungeFeed => {
                "Default plunge (Z-entry) feed for this tool (mm/min). Seeds a new \
                 operation's plunge feed; the operation can override it. 0 = unset."
            }
            Field::CornerRadius => {
                "Corner radius of the rounded-edge (bull-nose) end mill — the fillet \
                 between the flat bottom and the side. Must be smaller than the tool \
                 radius (⌀/2)."
            }
            Field::ChamferAngle => {
                "Included angle of the chamfer mill's cone (the full point angle, e.g. \
                 90° for a 45° chamfer). Sets how cut depth maps to chamfer width."
            }
            Field::TipDiameter => {
                "Diameter of the flat, non-cutting tip of a chamfer mill (min 0.10 mm — \
                 a chamfer mill is always ground with a flat, and one cannot be ground \
                 sharper). Only the angled flank cuts, which is why a chamfer mill \
                 cannot engrave: use a V-bit."
            }
            Field::TipRadius => {
                "Rounded-tip radius of a carving V-bit (min 0.05 mm — a point is always \
                 ground to a radius, and one cannot be ground sharper). Unlike a \
                 chamfer mill's flat, this rounded tip cuts, which is what lets a V-bit \
                 engrave."
            }
            Field::PointAngle => {
                "Included angle of the tool's point (the full cone angle) — e.g. a \
                 drill's 118°/135° point, or a V-bit's 60°/90° included angle."
            }
            Field::ToolThreadPitch => {
                "Ground pitch of a full-form thread mill (mm) — the axial spacing of its \
                 stacked teeth. (Single-point mills, set via the toggle, have no fixed \
                 pitch.)"
            }
            Field::Depth => {
                "Total cut depth below the top of stock (a positive distance). The \
                 feature's floor sits this far down."
            }
            Field::DrillStartOffset => {
                "Where the hole begins, as a height above the stock top (mm) — the same \
                 convention as the facing start offset. 0 starts at the stock top; a \
                 positive value starts above it (a proud boss); negative starts below \
                 (a recessed or faced surface). Depth is measured down from here."
            }
            Field::Peck => {
                "Peck-drilling increment (mm): the drill cuts this deep, fully retracts \
                 to clear chips, then returns and repeats until it reaches depth. \
                 0 = drill straight to depth in one plunge (no pecking)."
            }
            Field::Dwell => {
                "Dwell at the bottom of the hole (seconds): the tool pauses with the \
                 spindle turning before retracting, cleaning up the hole bottom. \
                 0 = no dwell."
            }
            Field::ProfileOffset => {
                "Finishing allowance left on the wall (mm): the roughing pass stops this \
                 far from the final profile so a finish pass can clean it up. 0 = cut to \
                 size."
            }
            Field::Stepdown => {
                "Maximum depth removed per Z level. The cut is split into passes no \
                 deeper than this; smaller is gentler on the tool."
            }
            Field::Stepover => {
                "Radial width of cut between adjacent passes (mm). For outside roughing \
                 it sets the concentric-pass spacing."
            }
            Field::SpindleRpm => {
                "Spindle speed for this operation (rpm), emitted as M3 S<rpm>. Seeded from \
                 the tool's nominal RPM when the operation is created; edit it for this cut. \
                 0 falls back to the job default."
            }
            Field::Feed => "Cutting feed rate — how fast the tool advances while cutting (mm/min).",
            Field::PlungeFeed => {
                "Feed rate for downward (Z) entry moves — usually slower than the \
                 cutting feed, since end mills cut poorly straight down (mm/min)."
            }
            Field::MajorDia => {
                "Major (nominal) diameter of the thread — the crest diameter for an \
                 external thread, the bore's tapped size for internal (mm)."
            }
            Field::Pitch => {
                "Thread pitch: the axial distance advanced per revolution (mm). One turn \
                 of the helix climbs by this much."
            }
            Field::ThreadTop => "Absolute Z of the top of the threaded length (mm).",
            Field::ThreadBottom => "Absolute Z of the bottom of the threaded length (mm).",
            Field::ThreadPasses => {
                "Radial infeed passes: cut the thread to full depth in this many equal \
                 radial steps (each a full helix). More passes lighten the cut for hard \
                 material. 1 = single full-depth pass."
            }
            Field::ThreadSpringPasses => {
                "Extra spring passes at full depth to clean up elastic spring-back after \
                 the last cutting pass. 0 = none."
            }
            Field::ThreadDrillClearance => {
                "For a blind hole: how far the pre-drilled hole extends below the thread \
                 bottom (mm). Must be at least the blind allowance. 0 = through hole (no \
                 check)."
            }
            Field::ThreadBlindAllowance => {
                "Required clearance between the last thread and the bottom of a blind hole \
                 (mm) — the tool cannot thread flush to a blind bottom. 0 = auto (one pitch)."
            }
            Field::ChamferTop => {
                "Absolute Z of the edge being chamfered (mm) — where the bevel starts, \
                 and what Tip depth is measured down from. Seeded from the top of \
                 stock, which is right for an edge on the raw surface; set it lower to \
                 chamfer the rim of a pocket or a step that an earlier operation cut."
            }
            Field::ChamferWidth => {
                "Width of the chamfer face measured along the slope (mm). With the tool \
                 angle this sets how deep the tool drops."
            }
            Field::ChamferDepth => {
                "How far below the top edge the tool tip rides (mm), which picks where \
                 on the tool's flank the cut lands. 0 = cut at the tip."
            }
            Field::ChamferStep => {
                "Chamfer width added per pass (mm) for a multi-pass chamfer. 0 = cut the \
                 whole chamfer in a single pass."
            }
            Field::LeadInSize => {
                "Size of the lead-in that eases the tool onto the wall — arc radius for \
                 an arc lead, ramp length for a linear one (mm). Avoids a dwell mark."
            }
            Field::LeadOutSize => "Size of the lead-out that eases the tool off the wall (see lead-in).",
            Field::LeadOverlap => {
                "How far the tool keeps cutting past the start point before leaving, to \
                 erase the entry/exit witness mark (mm)."
            }
            Field::FaceStartOffset => {
                "Height of the first facing cut above the drawing reference (mm) — start \
                 above the true top to skim scale, or at 0 to cut to size."
            }
            Field::Overlap => {
                "How much each facing pass overlaps the previous, as a percent of tool \
                 diameter. Higher = smoother floor, more passes."
            }
            Field::FaceOvershoot => {
                "How far the tool runs past the stock edge before turning around (mm), \
                 so the cutter fully clears the edge."
            }
            Field::Engagement => {
                "Adaptive-clearing stepover: the radial width of cut taken along a \
                 STRAIGHT wall (mm). Keeps tool load roughly constant for high-speed \
                 clearing. 0 = plain concentric clearing (off). Climb only.\n\n\
                 It is not a hard maximum. Where the path curves tightly — inner \
                 corners, the loops near a pocket's centre — the geometry forces more \
                 than you ask for, up to roughly 1.4x this value, and no spiral clearer \
                 can do better. What adaptive clearing removes is the full-diameter \
                 SLOT, which is the tool-breaking hazard; the residual rise on tight \
                 loops is a feed-rate matter, so slow the feed if the tool complains."
            }
            Field::PlungeA => {
                "First plunge parameter: the ramp/zig-zag angle in degrees, or the helix \
                 radius in mm — depending on the plunge type chosen below."
            }
            Field::PlungeB => {
                "Second plunge parameter: the zig-zag length, or the helix pitch (mm) — \
                 depending on the plunge type. Unused for a straight plunge."
            }
            Field::RingStep => {
                "How far apart the WALL rings step inward (mm) -- a roughing control, not \
                 a finish one. The finished wall is cut by the deepest ring alone, whose \
                 flank spans from the boundary to its tip; the shallower rings limit how \
                 much one pass takes and reach into corners. Coarser costs tool load, not \
                 surface quality. 0 = choose automatically."
            }
            Field::Scallop => {
                "The ridge you will accept on the flat FLOOR (mm) -- the real finish \
                 control. A cone cannot leave a flat floor, so adjacent passes leave a \
                 ridge between them; asking for a height rather than a spacing lets the \
                 spacing open up where the tool's geometry allows. 0 = a fine default."
            }
            Field::ClearStepdown => {
                "Maximum depth the clearing tool removes per pass (mm). 0 takes the whole \
                 flat-area depth in one pass."
            }
            Field::ClearOverlap => {
                "How much each clearing ring overlaps the previous, as a percent of the \
                 clearing tool's diameter. Higher = smoother floor, more passes."
            }
            Field::ClearEngagement => {
                "Adaptive-clearing stepover for the clearing pass: the radial width of \
                 cut it takes along a STRAIGHT wall (mm). 0 = plain concentric \
                 clearing. Climb only. As above, tight loops geometrically exceed it by \
                 up to about 1.4x — that is a feed-rate matter, not a slot."
            }
            Field::ClearFeed => {
                "Cutting feed for the clearing pass (mm/min). 0 uses the carve's own \
                 feed -- but an end mill and a V-bit rarely want the same number."
            }
            Field::ClearPlungeFeed => {
                "Plunge feed for the clearing pass (mm/min). 0 uses the carve's own."
            }
            Field::ClearLeadOverlap => {
                "How far each clearing ring keeps cutting past its start before leaving \
                 (mm), to erase the entry/exit witness mark."
            }
            Field::ClearLeadInSize => {
                "Size of the clearing pass's lead-in onto the flat area's edge -- arc \
                 radius for an arc lead, ramp length for a linear one (mm)."
            }
            Field::ClearLeadOutSize => "Size of the clearing pass's lead-out (see lead-in).",
            Field::ClearPlungeA => {
                "First clearing-plunge parameter: the ramp/zig-zag angle in degrees, or \
                 the helix radius in mm, depending on the plunge type."
            }
            Field::ClearPlungeB => {
                "Second clearing-plunge parameter: the zig-zag length, or the helix pitch \
                 (mm). Unused for a straight plunge."
            }
            Field::ClearOffset => {
                "How far the end mill stays off the carved surface (mm), leaving that skin \
                 for the V-bit -- which finishes it better, with the flank of its cone \
                 rather than the corner of a cylinder. Nothing is abandoned: the V-bit's \
                 own passes are computed from what the end mill actually swept."
            }
        }
    }
}

/// An inspector label wrapped in a hover tooltip carrying its help text — the shared
/// way every parameter (field, picker, checkbox) explains itself to new users. Keeps
/// the fixed 135-px label column so rows stay aligned.
/// Wrap any element in a hover tooltip carrying `help` (appearing to its left, toward
/// the viewport), when `show`. The shared primitive behind every inspector tooltip.
fn help_wrap<'a>(
    content: impl Into<Element<'a, Message>>,
    help: &'static str,
    show: bool,
) -> Element<'a, Message> {
    let content = content.into();
    if !show {
        return content;
    }
    tooltip(
        content,
        container(text(help).size(12))
            .padding(8)
            .max_width(300.0)
            .style(container::rounded_box),
        tooltip::Position::Left,
    )
    .into()
}

/// A fixed-width (135 px) inspector label wrapped in a hover tooltip carrying its help
/// text when `show` — how every field/picker/checkbox row explains itself, keeping the
/// label column aligned.
fn label_help<'a>(
    label: impl iced::widget::text::IntoFragment<'a>,
    help: &'static str,
    show: bool,
) -> Element<'a, Message> {
    help_wrap(text(label).width(Length::Fixed(135.0)).size(13), help, show)
}

/// Hover-help text for the inspector's non-numeric controls (pickers, checkboxes).
/// Numeric fields carry their own via [`Field::help`].
mod help {
    pub const SIDE: &str =
        "Which side of the selected chain the tool runs. Outside keeps the chain as the \
         finished part (cut around it); Inside cuts a pocket/bore to the chain; On centres \
         the tool on the line.";
    pub const LEAD_IN: &str =
        "How the tool eases onto the wall at the start of a pass instead of diving \
         straight in — an arc or a straight ramp — to avoid a dwell/witness mark. None \
         starts cutting directly.";
    pub const LEAD_OUT: &str = "How the tool eases off the wall at the end of a pass (see lead-in).";
    pub const PLUNGE: &str =
        "How the tool enters downward at each level. Straight drops vertically (needs a \
         centre-cutting tool or pre-drilled hole); Ramp descends along the toolpath itself, \
         arriving on the contour at full depth; Zig-zag oscillates in place, for a slot too \
         narrow to ramp along; Helix spirals down — all gentler than a straight plunge.";
    pub const CLIMB: &str =
        "Milling direction. Climb (recommended) gives a cleaner finish and is required for \
         adaptive clearing; unticking it is conventional milling, which reverses the pass \
         direction.";
    pub const CARVE_CLEAR: &str =
        "Clear the flat areas the depth cap leaves with a second, flat-bottomed tool \
         before the V-bit runs. A cone cannot leave a flat floor, so without this those \
         areas come out ridged. The end mill also takes the bulk material, sparing the \
         carving tip.";
    pub const CARVE_PLUNGE: &str =
        "How the V-bit enters downward at each ring. A V-bit always has a cutting tip, so \
         a straight drop is safe -- but a carve enters hundreds of times, and ramping \
         along the ring is kinder to the tip and to the finish. This is the V-bit's own \
         entry; the clearing tool has its own picker below.";
    pub const CARVE_CLEAR_PLUNGE: &str =
        "How the clearing tool enters downward. The flat area is entered in solid stock, \
         so a tool that is not centre-cutting -- or a deep carve in hard material -- \
         wants a ramp or helix rather than a straight drop.";
    pub const CARVE_STAY_DOWN: &str =
        "Link the carve rings without lifting where it is safe to do so, instead of \
         retracting to clearance for each. A carve can run to hundreds of rings, so this \
         saves most of the cycle time; each link is checked individually, and any that \
         would gouge still lifts.";
    pub const FACE_DIRECTION: &str =
        "The axis the facing passes sweep along. Defaults to the boundary edge you picked; \
         otherwise the longest edge.";
    pub const CHAMFER_GRADUAL: &str =
        "Spread the chamfer over its passes so each removes an equal amount of material, \
         rather than full-width passes stepping down. Only matters for a multi-pass chamfer.";
    pub const THREAD_GRADUAL: &str =
        "Size the radial infeed passes so each removes an equal amount of material, \
         instead of stepping out by an equal radius each time. A thread form is a V, so \
         the groove widens as it deepens and equal radial steps make the LAST pass the \
         heaviest — the pass that cuts the finished flank. Gradual reverses that, \
         leaving the finishing pass the lightest. Only matters with more than one pass.";
    pub const BORE: &str =
        "Whether the thread is cut into a hole (Internal) or onto a boss/stud (External). \
         Sets which side of the pitch line the tool orbits.";
    pub const HAND: &str =
        "Thread hand: Right-hand advances when turned clockwise (the common case), \
         Left-hand the opposite. Sets the helix direction.";
    pub const THREAD_CUT: &str =
        "Climb vs conventional milling for the threading orbit — climb usually finishes \
         cleaner.";
    pub const ACTIVE_MACHINE: &str =
        "Which machine you are working on. The fields below edit it, and an export is \
         checked against its travel — so this is the one thing that decides whether a \
         job can be cut here. Machines are local to this installation: opening a project \
         built elsewhere never changes it.";
    pub const POST: &str =
        "The machine controller/dialect the exported G-code is written for (grbl, Fanuc, \
         …). Pick the one your control speaks.";
    pub const TOOL_TYPE: &str =
        "The cutter geometry class. Changing it reveals the geometry fields that class \
         needs (corner radius, point angle, tip diameter, …) and how the toolpath treats \
         the tip.";
    pub const MACHINE_NAME: &str =
        "A free-text label to tell your machines apart. It does not affect the toolpath \
         or output.";
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
    /// The preferences panel is open.
    show_prefs: bool,
    /// Every machine this installation knows about. The **active** one is mirrored into
    /// the controller (which gates exports against it); this is the set to pick from.
    machines: crate::MachineLibrary,
    /// The active machine's name — the key the selection is remembered by.
    active_machine: String,
    /// Uncommitted machine edits, awaiting **Apply** — the same promise the numeric
    /// fields make. A rename that took effect per keystroke had no way to be abandoned,
    /// wrote both config files on every letter, and gave the operator no signal that
    /// anything was pending (Andreas, 2026-08-01). Only the *Active* picker stays
    /// instant: switching machines is a navigation, not an edit.
    machine_name_edit: Option<String>,
    machine_post_edit: Option<PostKind>,
    /// The user's preferences, as loaded at startup and written back on change.
    ///
    /// The view/snap/pane fields below stay the live state — the settings copy is
    /// refreshed through `remember`, which is the single place that knows the mapping.
    settings: crate::Settings,
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
    /// Whether hover tooltips (inspector parameters + ribbon icons) are shown. On by
    /// default to help new users; toggled off from the View tab once they're fluent.
    tooltips: bool,
    /// Whether the licence-and-credits overlay is open (View tab -> About -> Licence).
    show_license: bool,
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
    /// A live **working copy** of the selected library tool while editing in the Tooling
    /// tab. Field edits apply here first (so the viewport previews them instantly without
    /// an Apply); the committed library entry is only overwritten on Apply. `None` outside
    /// Tooling mode. Dirty ⇔ this differs from the committed entry.
    tool_edit: Option<cam_model::Tool>,
    /// Which internal tab the Tool Library pane shows (Serial / Family).
    library_view: LibraryView,
    /// The tool family chosen in the operation wizard, narrowing its Tool list.
    wizard_family: Option<ToolKindPick>,
    /// The operation whose right-click context menu is open, and where to anchor it
    /// (window-absolute coords, captured from `window_cursor` at right-click time).
    open_op_menu: Option<u32>,
    op_menu_pos: iced::Point,
    /// The project tool (by number) whose right-click "Add to library" menu is open,
    /// and where to anchor it. `None` unless a menu is open.
    open_tool_menu: Option<u32>,
    tool_menu_pos: iced::Point,
    /// The Library-pane right-click menu: which library index, whether it has morphed
    /// into the "set number" input (and its buffer), and where to anchor it.
    lib_menu: Option<LibMenu>,
    lib_menu_pos: iced::Point,
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
    /// Draw the active machine's travel as a box around the job.
    show_envelope: bool,
    status: String,
}

// The pickbox aperture, the object-snap catch distance derived from it, and the snap
// marker's size are all preferences now (`Settings::snapping`) — see `SnapPrefs`.
// The catch distance is deliberately not its own control: it stays a fixed multiple
// of the pickbox, because two absolute knobs would let a user set a catch distance
// smaller than the pickbox feeding it.

/// Whether an operation kind uses a start/lead-in point, and so honours object
/// snaps. Face/Drill/Thread have no start, so snaps are inert (and hidden) there.
fn op_uses_snaps(kind: OpKind) -> bool {
    matches!(
        kind,
        OpKind::Profile | OpKind::Chamfer | OpKind::Pocket | OpKind::Engrave | OpKind::Carve
    )
}

/// Whether any of `visible`'s edit buffers has diverged from the model.
///
/// Free-standing, and taking the model read as a closure, so the rule can be tested
/// without standing up a GUI — `inspector_dirty` is otherwise unreachable from a
/// headless test, and an always-on Apply button is exactly the kind of thing that
/// goes unnoticed for months.
///
/// A buffer that will not parse counts as **dirty**: the operator has typed
/// something, even if it is not yet a number. Apply stays disabled in that case
/// anyway, via `any_field_invalid`. A field with no buffer at all counts as clean —
/// nothing has been typed into it.
fn fields_are_dirty(
    visible: &[Field],
    buffers: &BTreeMap<Field, String>,
    committed: impl Fn(Field) -> Option<f64>,
) -> bool {
    visible.iter().any(|&f| {
        let Some(model) = committed(f) else {
            return false;
        };
        match buffers.get(&f) {
            // `fmt_num` round-trips an f64 exactly, so an untouched buffer parses
            // back to precisely the value it was seeded with.
            Some(text) => text.parse::<f64>().map(|v| v != model).unwrap_or(true),
            None => false,
        }
    })
}

/// Whether a field belongs to a carve's **clearing pass** rather than to the carve
/// itself. These render in their own section under the clearing tool, not mixed in with
/// the V-bit's own numbers — two tools, two blocks.
fn is_clear_field(field: Field) -> bool {
    matches!(
        field,
        Field::ClearStepdown
            | Field::ClearOverlap
            | Field::ClearOffset
            | Field::ClearEngagement
            | Field::ClearFeed
            | Field::ClearPlungeFeed
            | Field::ClearLeadOverlap
            | Field::ClearLeadInSize
            | Field::ClearLeadOutSize
            | Field::ClearPlungeA
            | Field::ClearPlungeB
    )
}

/// Orientation-cube on-screen size (logical px): the slider's range. The *default*
/// now lives in `Settings` alone — the session reads it from there, so a constant
/// here would be a second copy to keep in step.
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
fn add_snap_marker(scene: &mut Scene, hit: SnapHit, aperture: f64, scale: f32, z: f32) {
    let (cx, cy) = (hit.point[0] as f32, hit.point[1] as f32);
    // A touch larger than the (already doubled) snap aperture, so the engaged
    // marker reads clearly in place of the pickbox.
    let h = (aperture as f32 * scale).max(0.01);
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

/// Highlight a picked path. `closed` closes it back to its start; an **open**
/// imported stroke must not be closed, or the highlight would show a segment the
/// toolpath will never cut.
fn add_path_highlight(
    scene: &mut Scene,
    pts: &[cam_geo::Point],
    closed: bool,
    color: [f32; 4],
) {
    let mut strip: Vec<[f32; 3]> = pts.iter().map(|p| [p.x as f32, p.y as f32, 0.0]).collect();
    if closed {
        if let Some(&first) = strip.first() {
            strip.push(first);
        }
    }
    scene.add_strip(strip, color);
}

/// Colour of the drilling annotations — a light blue, distinct from the cyan snap
/// and magenta origin markers and legible under red-green colour deficiency.
const DRILL_MARK: [f32; 4] = [0.40, 0.72, 1.0, 1.0];
/// Radius of a peck ring / half-length of a dwell bar, in **screen pixels** (so the
/// mark never scales with the model — it is sized against `world_per_pixel` at draw).
const DRILL_MARK_PX: f32 = 4.4;
/// Facets in a peck ring (a billboarded circle drawn as a line loop).
const DRILL_RING_SEGMENTS: usize = 16;
/// Safety cap on rings per hole, so a pathologically tiny peck can't flood the buffer.
const DRILL_MAX_RINGS: u32 = 200;

/// A drilling annotation anchored in world space: a **ring** at a peck retract
/// depth, or a horizontal **bar** at the hole bottom when it dwells. The pixel
/// sizing/billboarding happens at draw time (see [`Viewport::draw`]).
#[derive(Clone, Copy, Debug, PartialEq)]
struct DrillMark {
    at: [f32; 3],
    kind: DrillMarkKind,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum DrillMarkKind {
    /// A peck retract level (locates the pecking).
    PeckRing,
    /// The exact bottom where a dwelling drill stops.
    DwellBar,
}

/// Collect the drilling annotation anchors from a backplot program: a ring at each
/// **intermediate** peck retract (strictly between the hole top and bottom — the
/// bottom is the plunge-line end, not a peck), and a bar at the bottom of every
/// hole that dwells. Pure; the screen sizing is applied later, per frame.
fn drill_marks_of(program: &Program) -> Vec<DrillMark> {
    let mut marks = Vec::new();
    for step in program.steps() {
        let Step::Drill(c) = step else { continue };
        for &[x, y] in &c.points {
            let (x, y) = (x as f32, y as f32);
            if let Some(peck) = c.peck.filter(|p| *p > 0.0) {
                let mut k = 1u32;
                loop {
                    let z = c.z_top - peck * k as f64;
                    if z <= c.depth + 1e-6 || k > DRILL_MAX_RINGS {
                        break; // reached (or passed) the bottom — that's not a peck
                    }
                    marks.push(DrillMark {
                        at: [x, y, z as f32],
                        kind: DrillMarkKind::PeckRing,
                    });
                    k += 1;
                }
            }
            if c.dwell.is_some() {
                marks.push(DrillMark {
                    at: [x, y, c.depth as f32],
                    kind: DrillMarkKind::DwellBar,
                });
            }
        }
    }
    marks
}

/// Append `mark`, sized to `px_world` (world mm for the desired pixel size) and
/// billboarded into the camera's `right`/`up` plane, to a `LineList` vertex buffer.
fn push_drill_mark(out: &mut Vec<Vertex>, mark: DrillMark, px_world: f32, right: [f32; 3], up: [f32; 3]) {
    let at = mark.at;
    let point = |du: f32, dv: f32| Vertex {
        position: [
            at[0] + du * right[0] + dv * up[0],
            at[1] + du * right[1] + dv * up[1],
            at[2] + du * right[2] + dv * up[2],
        ],
        color: DRILL_MARK,
    };
    match mark.kind {
        DrillMarkKind::PeckRing => {
            let r = px_world;
            let n = DRILL_RING_SEGMENTS;
            for i in 0..n {
                let a0 = i as f32 / n as f32 * std::f32::consts::TAU;
                let a1 = (i + 1) as f32 / n as f32 * std::f32::consts::TAU;
                out.push(point(r * a0.cos(), r * a0.sin()));
                out.push(point(r * a1.cos(), r * a1.sin()));
            }
        }
        DrillMarkKind::DwellBar => {
            // A little wider than a ring so the exact stop reads as a crisp line.
            let h = px_world * 1.4;
            out.push(point(-h, 0.0));
            out.push(point(h, 0.0));
        }
    }
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
    /// Rename the machine.
    MachineNameChanged(String),
    /// Choose the post/controller dialect for export.
    PostKindChanged(PostKind),
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
    /// Prompt for a `.ocam` to **export** the working tool library to.
    ExportLibrary,
    /// The chosen path to write the library to (`None` = cancelled).
    LibraryToExport(Option<PathBuf>),
    /// Prompt for a `.ocam` tool library to **import** (replaces the working library).
    ImportLibrary,
    /// The chosen library file to import (`None` = cancelled).
    LibraryToImport(Option<PathBuf>),
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
    /// Toggle whether the selected carve links its rings without lifting.
    CarveStayDownToggled(bool),
    /// Turn the selected carve's flat-area clearing pass on or off.
    CarveClearToggled(bool),
    /// Choose the library tool (by index) that clears the selected carve's flat areas.
    CarveClearToolChanged(usize),
    /// Change how the selected carve's clearing tool enters in Z.
    CarveClearPlungeChanged(PlungeKind),
    /// Climb vs conventional for the selected carve's clearing pass.
    CarveClearClimbToggled(bool),
    /// Wall lead-in kind for the selected carve's clearing pass.
    CarveClearLeadInChanged(LeadKind),
    /// Wall lead-out kind for the selected carve's clearing pass.
    CarveClearLeadOutChanged(LeadKind),
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
    /// Focus a workpiece origin — makes it active + selects it.
    SelectOrigin(u32),
    /// Add a new workpiece origin (a reorientation group).
    AddOrigin,
    /// Delete the workpiece origin with the given index (extras only).
    DeleteOrigin(u32),
    /// Freeze/unfreeze an origin's operations (drop them from the run/viewport).
    ToggleOriginDisabled(u32, bool),
    /// Enter/leave single-point "set workpiece origin" pick mode (Edit tab).
    ToggleSetOrigin,
    /// Enter/leave two-point origin pick mode: X from the 1st pick, Y from the
    /// 2nd, Z the midpoint of both.
    ToggleSetOrigin2pt,
    /// Show or hide the workpiece-origin datum marker (View tab).
    ToggleShowOrigin,
    /// Show / hide the machine-travel envelope around the job.
    ToggleEnvelope,
    /// A viewport click while setting the origin (either mode): world `(x,y)` +
    /// aperture, resolved to a snapped or free point.
    OriginPointPicked([f32; 2], f32),
    /// Structural edits to the selected operation.
    DuplicateOp,
    DeleteOp,
    /// Restart the creation wizard for the context-menu operation, replacing it in
    /// place (same kind, same position in the order).
    ReinitOp,
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
    /// Show or hide hover tooltips (inspector + ribbon icons).
    ToggleTooltips,
    /// Open the licence-and-credits overlay.
    ShowLicense,
    /// Dismiss the licence-and-credits overlay.
    CloseLicense,
    /// Set the orientation cube's on-screen size (logical px).
    SetGizmoSize(f32),
    /// A continuous control has settled (slider released) — write preferences now
    /// rather than on every intermediate value.
    SettingsSettled,
    /// Make this machine the active one — and adopt its control with it.
    ActiveMachineChanged(String),
    /// Add a machine to the library: a copy of the active one, ready to edit.
    NewMachine,
    /// Remove the active machine (never the last).
    DeleteMachine,
    /// Open / close the preferences panel.
    ShowPrefs,
    ClosePrefs,
    /// Pickbox aperture, logical px. The object-snap catch distance follows it.
    SetPickbox(f32),
    /// Snap marker size, as a multiple of the catch aperture.
    SetMarkerScale(f32),
    /// The smallest `pane` may be dragged to, logical px.
    SetPaneMin(Pane, f32),
    /// Workpiece-origin marker size, as a multiple of the shipped size.
    SetOriginMarker(f32),
    /// Put every preference back to its shipped value.
    RestoreDefaults,
    PaneResized(pane_grid::ResizeEvent),
    PaneDragged(pane_grid::DragEvent),
    /// The window was resized (tracked for pixel-accurate pane minimums).
    WindowResized(iced::Size),
    /// Change the selected tool's geometry kind (committed immediately).
    ToolKindChanged(ToolKind),
    /// Change the selected tool's cutting direction (down-cut / up-cut).
    ToolCuttingDirChanged(CutDir),
    /// Toggle a thread mill between single-point (one tooth) and full-form (teeth at a
    /// fixed pitch).
    ThreadFormChanged(ThreadForm),
    /// Change the selected profile's lead-in / lead-out / plunge kind (committed
    /// immediately with default parameters; sizes are then edited as fields).
    LeadInKindChanged(LeadKind),
    LeadOutKindChanged(LeadKind),
    PlungeKindChanged(PlungeKind),
    /// Change the selected face op's pass direction (committed immediately).
    FaceDirectionChanged(Axis),
    /// Toggle climb (true) vs conventional (false) clearing on the selected
    /// pocket / profile-roughing op.
    ClearingClimbToggled(bool),
    /// Toggle the selected chamfer's gradual (equal-material) stepping.
    ChamferGradualToggled(bool),
    /// Toggle the selected thread's gradual (equal-material) radial infeed.
    ThreadGradualToggled(bool),
    /// Create a new default tool and select it.
    NewTool,
    /// Delete the selected tool.
    DeleteTool,
    /// Switch the Tool Library pane's internal tab (Serial / Family).
    SetLibraryView(LibraryView),
    /// Open the Library-pane right-click menu for the library tool at this index.
    LibToolMenu(usize),
    /// Morph the Library menu into the "set number" input.
    LibMenuSetNumber,
    /// Update the "set number" input buffer.
    LibNumberInput(String),
    /// Commit the "set number" input (swaps on collision).
    LibNumberCommit,
    /// Dismiss the Library-pane menu / number input.
    CloseLibMenu,
    /// Bulk-renumber the library (opens a confirm dialog first).
    RenumberLibrary,
    /// The confirm-dialog result for the bulk renumber.
    RenumberConfirmed(bool),
    /// Open the project-tree right-click menu for the tool numbered `u32` (only wired
    /// for tools not in the library).
    ToolMenu(u32),
    /// Dismiss the tool context menu.
    CloseToolMenu,
    /// Promote the project tool numbered `u32` into the shop library (§6.3).
    AddToolToLibrary(u32),
    /// Switch the active ribbon tab.
    SelectRibbonTab(RibbonTab),
    /// Open/close the collapse-popup for a collapsed group (index in the active tab).
    ToggleRibbonGroup(usize),
    /// Close any open collapse-popup.
    CloseRibbonPopup,
    /// Show (`true`) or hide (`false`) a pane.
    SetPaneVisible(Pane, bool),
    /// Pick a library tool (by index) for the pending op — embeds it into the setup.
    /// Choose the tool **family** in the operation wizard, narrowing the tool list.
    /// Clears any tool already picked, since it belongs to the previous family.
    SetPendingFamily(ToolKindPick),
    SetPendingLibraryTool(usize),
    /// Select a library tool for editing in the Tooling-tab library editor.
    SelectLibraryTool(usize),
    /// Open the right-click context menu for operation `id` (anchored under the
    /// cursor), and select that operation.
    OpMenu(u32),
    /// Move the context-menu's operation to the workpiece origin with this index —
    /// i.e. into that origin's group in the project tree, and so under its work offset
    /// at post time. Until this existed the only way across was delete-and-recreate.
    MoveOpToOrigin(u32),
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
        // Destructured rather than discarded: a library that failed to load is the one
        // thing at startup the operator must be *told* about, or they meet the stock
        // tools and assume theirs were lost. They are not — the file is untouched, with
        // a `.bak` beside it — but only if nobody edits a tool first.
        let (library, library_load) = ToolLibrary::load();
        // Preferences seed the session. A rejected settings file is deliberately not
        // reported the way a rejected *library* is: defaults here cost the user a
        // layout, not their tools.
        let (settings, _) = crate::load_settings();
        let (machines, machine_load) = crate::load_machines();
        // Resolve where you left off. A name that no longer resolves lands on the first
        // machine — never on nothing — and is *reported*, because silently authoring
        // against a different machine's travel is the failure this whole area exists to
        // prevent.
        let remembered = settings.session.machine.clone();
        let stale = remembered.is_some() && !machines.resolves(remembered.as_deref());
        let active = machines
            .resolve(remembered.as_deref())
            .cloned()
            .unwrap_or_else(|| crate::MachineEntry::new(crate::default_machine()));
        let active_machine = active.name().to_string();
        let mut app = Self {
            controller: AppController::new(active.machine.clone()),
            panes: initial_panes(),
            show_license: false,
            show_prefs: false,
            window: iced::Size::new(1280.0, 800.0),
            project_px: settings.panes.project_px,
            inspector_px: settings.panes.inspector_px,
            output_px: settings.panes.output_px,
            active_tab: RibbonTab::Home,
            open_group: None,
            fields: BTreeMap::new(),
            show_stock: settings.view.show_stock,
            show_gizmo: settings.view.show_gizmo,
            tooltips: settings.view.tooltips,
            gizmo_size: settings.view.gizmo_size,
            view: ViewControls::default(),
            cursor: None,
            library,
            machines,
            active_machine,
            machine_name_edit: None,
            machine_post_edit: None,
            lib_sel: 0,
            tool_edit: None,
            library_view: LibraryView::Ordered,
            wizard_family: None,
            open_op_menu: None,
            op_menu_pos: iced::Point::ORIGIN,
            open_tool_menu: None,
            tool_menu_pos: iced::Point::ORIGIN,
            lib_menu: None,
            lib_menu_pos: iced::Point::ORIGIN,
            window_cursor: iced::Point::ORIGIN,
            focus_ops: BTreeSet::new(),
            modifiers: iced::keyboard::Modifiers::default(),
            snaps: settings.snapping.default_snaps.clone(),
            snap_hover: None,
            snap_aperture: 1.0,
            hover_loop: None,
            setting_origin: false,
            setting_origin_2pt: false,
            origin_first: None,
            show_origin: settings.view.show_origin,
            show_envelope: settings.view.show_envelope,
            status: "Open the sample part to begin.".to_string(),
            // Last: every field above reads from it.
            settings,
        };
        // The active machine carries its control, so resolving the machine above chose
        // the post too — one source of truth, which is why `session.post` was retired.
        app.controller.set_post_kind(active.post);
        if stale {
            app.status = format!(
                "The machine you last used is no longer in the library; using \"{}\" \
                 instead. Check it before cutting.",
                app.active_machine
            );
        } else if let crate::MachineLoad::Rejected(why) = &machine_load {
            app.status = format!(
                "Machine library {why}. Using the default machine; your file is untouched \
                 (a copy is beside it as machines.json.bak)."
            );
        }
        if let crate::LibraryLoad::Rejected(why) = &library_load {
            app.status = format!(
                "Tool library {why}. Using the starter tools; your file is untouched \
                 (a copy is beside it as tools.json.bak)."
            );
        }
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
                // In the Tooling editor, every field edit flows into the working copy so
                // the viewport previews it instantly (no Apply needed); the length trio
                // also refreshes its derived buffers. Apply then commits the working copy.
                if self.library_mode() {
                    self.live_edit(field);
                }
            }
            Message::MachineNameChanged(name) => {
                // Buffered, not committed: Apply lights up, exactly as it does for the
                // travel fields beside it.
                self.machine_name_edit = Some(name);
            }
            Message::PostKindChanged(kind) => {
                // Also buffered. The picker sits in the Machine inspector, so it edits
                // *this machine's* control — and an edit waits for Apply.
                self.machine_post_edit = Some(kind);
            }
            Message::Apply => {
                // Blocked while any field is invalid or there is nothing to commit (also
                // catches Enter-to-apply).
                if !self.any_field_invalid() && self.inspector_dirty() {
                    self.apply_inspector();
                }
            }
            Message::NewProject => {
                // The post is *not* touched: a new project does not move you to a
                // different machine or control, so there is nothing to re-target.
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
                        // Reconcile the project's tool numbers against the shop library
                        // (TOOLING_PLAN §6): matched tools adopt the shop's numbering.
                        let report = self.controller.reconcile_tools(&self.library.tools);
                        self.focus_ops.clear();
                        self.refresh_fields();
                        self.rerun();
                        match report.summary() {
                            Some(s) => format!("Opened {}. Tools: {s}.", path.display()),
                            None => format!("Opened {}.", path.display()),
                        }
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
            Message::ExportLibrary => {
                return iced::Task::perform(
                    pick_save("OpenCAMStudio tool library", "tools.ocam", &["ocam"]),
                    Message::LibraryToExport,
                );
            }
            Message::LibraryToExport(Some(path)) => {
                let file = OcamFile::Library(self.library.clone());
                self.status = match file.to_json() {
                    Ok(json) => match std::fs::write(&path, json) {
                        Ok(()) => format!("Exported tool library to {}.", path.display()),
                        Err(e) => format!("Library export failed: {e}"),
                    },
                    Err(e) => format!("Library export failed: {e}"),
                };
            }
            Message::ImportLibrary => {
                return iced::Task::perform(
                    pick_open("OpenCAMStudio tool library", &["ocam"]),
                    Message::LibraryToImport,
                );
            }
            Message::LibraryToImport(Some(path)) => {
                self.status = match std::fs::read_to_string(&path) {
                    Ok(text) => match OcamFile::from_json(&text) {
                        Ok(OcamFile::Library(lib)) => {
                            let n = lib.tools.len();
                            self.library = lib;
                            self.library.save(); // becomes the working (config-dir) library
                            self.lib_sel = 0;
                            self.reload_tool_edit();
                            format!("Imported {n} tool(s) from {}.", path.display())
                        }
                        Ok(OcamFile::Project(_)) => {
                            "That .ocam is a project, not a tool library.".to_string()
                        }
                        Err(e) => format!("Library import failed: {e}"),
                    },
                    Err(e) => format!("Library import failed: {e}"),
                };
            }
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
                // **Post before opening the dialog.** The refusal used to come *after*
                // the operator had picked a folder and a filename, which reads as "saved"
                // — they close the dialog believing the job exported and go looking for a
                // file that was never written. Doing the work first means the only dialog
                // they ever see is one that will produce a file.
                if let Err(e) = self.controller.export_nc() {
                    self.status = format!("Export blocked: {e}");
                    return iced::Task::perform(
                        report_export_blocked(format!("{e}")),
                        |()| Message::CloseRibbonPopup,
                    );
                }
                // Guardrail: if any included operations are exact duplicates, they
                // would post the same toolpath twice. Confirm before the machine
                // sees it — but don't block (a spring/finishing pass is legitimate).
                let groups = self.controller.duplicate_operation_groups();
                if groups.is_empty() {
                    let kind = self.controller.post_kind();
                    return iced::Task::perform(
                        pick_save("G-code", kind.default_file_name(), kind.file_extensions()),
                        Message::NcToExport,
                    );
                }
                return iced::Task::perform(
                    confirm_export_duplicates(describe_duplicates(&groups)),
                    Message::ExportDupConfirmed,
                );
            }
            Message::ExportDupConfirmed(true) => {
                let kind = self.controller.post_kind();
                return iced::Task::perform(
                    pick_save("G-code", kind.default_file_name(), kind.file_extensions()),
                    Message::NcToExport,
                );
            }
            Message::ExportDupConfirmed(false) => {
                self.status =
                    "Export cancelled — exclude or edit the duplicate operation(s).".to_string();
            }
            Message::NcToExport(Some(path)) => {
                // The post already succeeded (checked before the dialog opened), so a
                // failure here is the *write* — a read-only folder, a full disk. Still
                // worth a dialog: the operator has just been told a file was coming.
                match self.controller.export_nc_to(&path) {
                    Ok(()) => {
                        self.status = format!("Exported G-code to {}.", path.display());
                    }
                    Err(e) => {
                        self.status = format!("Export failed: {e}");
                        return iced::Task::perform(
                            report_export_blocked(format!(
                                "Could not write {}.\n\n{e}",
                                path.display()
                            )),
                            |()| Message::CloseRibbonPopup,
                        );
                    }
                }
            }
            // Cancelled dialogs — nothing to do.
            Message::ProjectToOpen(None)
            | Message::ProjectToSave(None)
            | Message::LibraryToExport(None)
            | Message::LibraryToImport(None)
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
            Message::SelectOrigin(index) => {
                self.controller.select_origin(index);
                self.refresh_fields();
            }
            Message::AddOrigin => {
                self.controller.add_origin();
                self.refresh_fields();
            }
            Message::DeleteOrigin(index) => {
                self.controller.delete_origin(index);
                self.refresh_fields();
                self.rerun();
            }
            Message::ToggleOriginDisabled(index, disabled) => {
                self.controller.set_origin_disabled(index, disabled);
                self.rerun();
            }
            Message::ToggleShowOrigin => {
                self.show_origin = !self.show_origin;
                self.remember();
            }
            Message::ToggleEnvelope => {
                self.show_envelope = !self.show_envelope;
                self.remember();
                self.status = if self.show_envelope {
                    format!(
                        "Showing {}'s travel around the job — a size check, not its \
                         position on the machine.",
                        self.controller.machine().name
                    )
                } else {
                    "Hiding the machine envelope.".to_string()
                };
            }
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
                    // Single point: X and Y (Z stays; edit it in the fields). Sets the
                    // *active* origin — the one selected in the tree.
                    self.controller.edit_active_origin(|o| {
                        o[0] = p[0];
                        o[1] = p[1];
                    });
                    self.setting_origin = false;
                    self.refresh_fields();
                    // `edit_active_origin` invalidates the run; repaint the backplot (the
                    // typed-origin path reruns via `apply_inspector`, the pick path must too).
                    self.rerun();
                    self.status = format!("Origin set to X{:.3} Y{:.3}.", p[0], p[1]);
                } else if let Some(first) = self.origin_first.take() {
                    // Two-point: X from the 1st pick, Y from the 2nd, Z the midpoint.
                    let origin = [first[0], p[1], (first[2] + p[2]) / 2.0];
                    self.controller.edit_active_origin(|o| *o = origin);
                    self.setting_origin_2pt = false;
                    self.refresh_fields();
                    self.rerun();
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
                // Seed the op with a sensible default tool so it always has a valid
                // one; the user can change it in the wizard picker. Face prefers the
                // largest flat end/face mill (see `ToolLibrary::default_tool_for`).
                // Pre-fill family + tool from the by-kind default, but only when that
                // default is in a family this operation actually offers — otherwise
                // leave both blank rather than seed something the wizard forbids.
                self.wizard_family = None;
                if self.controller.pending_op().is_some() {
                    if let Some(tool) = self.library.default_tool_for(kind) {
                        let family = ToolKindPick::of(tool.kind);
                        if families_for(kind).contains(&family) {
                            self.wizard_family = Some(family);
                            let number = self.controller.use_tool(tool);
                            self.controller.set_pending_tool(number);
                        }
                    }
                }
                self.refresh_fields();
                // Seed the wizard's cutting-data row from the (possibly pre-filled) tool.
                self.seed_wizard_cutting();
                self.status = if self.controller.pending_op().is_some() {
                    if op_accepts_open_paths(kind) && !self.controller.open_paths().is_empty() {
                        "Choose a tool and click a boundary or an open stroke — in \
                         either order, then Confirm."
                            .to_string()
                    } else {
                        "Choose a tool and click the geometry — in either order, then Confirm."
                            .to_string()
                    }
                } else {
                    "Open a part first.".to_string()
                };
            }
            Message::SetPendingFamily(f) => {
                self.wizard_family = Some(f);
                // The previously picked tool is from another family — drop it, and
                // with it the Confirm gate, rather than leave a stale selection.
                self.controller.clear_pending_tool();
                self.controller.prune_unused_tools();
            }
            Message::SetPendingLibraryTool(i) => {
                if let Some(&tool) = self.library.tools.get(i) {
                    let number = self.controller.use_tool(tool);
                    self.controller.set_pending_tool(number);
                    // Re-seed the cutting-data defaults from the newly chosen tool.
                    self.seed_wizard_cutting();
                }
            }
            Message::CancelOp => {
                self.controller.cancel_operation();
                self.controller.prune_unused_tools();
                self.status = "Cancelled operation creation.".to_string();
            }
            Message::ConfirmOp => {
                let cutting = self.wizard_cutting();
                if self.controller.confirm_operation(cutting) {
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
                    PickResult::Selecting => {
                        let pending = self.controller.pending_op();
                        let has_tool = pending.as_ref().is_some_and(|p| p.tool.is_some());
                        let islands =
                            pending.as_ref().is_some_and(|p| op_takes_islands(p.kind));
                        let n = pending.map_or(0, |p| p.islands.len());
                        self.status = if islands {
                            format!("Boundary set — click areas to exclude ({n}), then Confirm.")
                        } else if has_tool {
                            "Geometry set — Confirm to create the operation.".to_string()
                        } else {
                            "Geometry set — now choose a tool, then Confirm.".to_string()
                        };
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
                self.remember();
            }
            Message::ReinitOp => {
                let id = self.open_op_menu.take();
                if let Some(id) = id {
                    if self.controller.reinitialize_operation(id) {
                        // Same two-step tool choice as a fresh operation; seed the
                        // family from the op's own kind default where it is offered.
                        self.wizard_family = None;
                        if let Some(kind) = self.controller.pending_op().map(|p| p.kind) {
                            if let Some(tool) = self.library.default_tool_for(kind) {
                                let family = ToolKindPick::of(tool.kind);
                                if families_for(kind).contains(&family) {
                                    self.wizard_family = Some(family);
                                    let number = self.controller.use_tool(tool);
                                    self.controller.set_pending_tool(number);
                                }
                            }
                        }
                        self.refresh_fields();
                        self.seed_wizard_cutting();
                        self.status = "Reinitialising: pick a tool, then the geometry \
                                       (the operation keeps its place)."
                            .to_string();
                    } else {
                        self.status = "Open a part first.".to_string();
                    }
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
            Message::MoveOpToOrigin(origin) => {
                if let Some(id) = self.open_op_menu.take() {
                    self.controller.set_operation_origin(id, origin);
                    // The op keeps its identity and moves group; re-focus it so the
                    // tree scrolls/highlights where it landed rather than leaving the
                    // eye at the row it left.
                    self.focus_selected_op();
                    self.refresh_fields();
                    self.rerun();
                    let name = self.origin_menu_label(origin);
                    self.status = format!("Operation {id} moved to {name}.");
                }
            }
            Message::ToggleStock => {
                self.show_stock = !self.show_stock;
                self.status = if self.show_stock {
                    "Showing simulated stock.".to_string()
                } else {
                    "Hiding simulated stock.".to_string()
                };
                self.remember();
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
            Message::ToggleGizmo => {
                self.show_gizmo = !self.show_gizmo;
                self.remember();
            }
            Message::ShowLicense => self.show_license = true,
            Message::CloseLicense => self.show_license = false,
            Message::ToggleTooltips => {
                self.tooltips = !self.tooltips;
                self.status = if self.tooltips {
                    "Tooltips on.".to_string()
                } else {
                    "Tooltips off.".to_string()
                };
                self.remember();
            }
            Message::SetGizmoSize(v) => {
                // Not persisted here: the slider emits continuously while dragged, and
                // writing the file on every tick would be a hundred writes for one
                // decision. `SettingsSettled` fires on release.
                self.gizmo_size = v.clamp(GIZMO_SIZE_MIN, GIZMO_SIZE_MAX)
            }
            Message::SettingsSettled => self.remember(),
            Message::ActiveMachineChanged(name) => {
                if let Some(entry) = self.machines.by_name(&name).cloned() {
                    // A pending edit belongs to the machine it was typed on; carrying it
                    // across would apply it to the wrong one.
                    self.machine_name_edit = None;
                    self.machine_post_edit = None;
                    self.active_machine = name;
                    self.controller.set_machine(entry.machine);
                    // The machine carries its control: picking the machine picks the
                    // post. This is the error the library itself would otherwise create
                    // — right machine, wrong control, because the last job used another.
                    self.controller.set_post_kind(entry.post);
                    self.remember();
                    self.rerun();
                    self.status = format!("Machine: {} ({}).", self.active_machine, entry.post);
                }
            }
            Message::NewMachine => {
                // A copy of the active one: machines in a shop differ from each other in
                // one or two numbers, so duplicating beats starting from nothing.
                let mut entry = self
                    .machines
                    .by_name(&self.active_machine)
                    .cloned()
                    .unwrap_or_else(|| crate::MachineEntry::new(crate::default_machine()));
                entry.machine.name = format!("{} copy", entry.name());
                self.active_machine = self.machines.add(entry);
                self.machines.save();
                self.sync_active_machine();
                self.status = format!("Added machine \"{}\".", self.active_machine);
            }
            Message::DeleteMachine => {
                if self.machines.remove(&self.active_machine) {
                    self.machines.save();
                    // `remove` refuses the last one, so there is always something here.
                    let entry = self.machines.machines[0].clone();
                    self.active_machine = entry.name().to_string();
                    self.controller.set_machine(entry.machine);
                    self.controller.set_post_kind(entry.post);
                    self.remember();
                    self.rerun();
                    self.status = format!("Machine: {}.", self.active_machine);
                }
            }
            Message::ShowPrefs => self.show_prefs = true,
            Message::ClosePrefs => {
                self.show_prefs = false;
                self.remember();
            }
            Message::SetPickbox(v) => {
                let (lo, hi) = crate::PICKBOX_RANGE;
                self.settings.snapping.pickbox_px = v.clamp(lo, hi);
            }
            Message::SetMarkerScale(v) => {
                let (lo, hi) = crate::MARKER_SCALE_RANGE;
                self.settings.snapping.marker_scale = v.clamp(lo, hi);
            }
            Message::SetOriginMarker(v) => {
                let (lo, hi) = crate::ORIGIN_MARKER_RANGE;
                self.settings.view.origin_marker_scale = v.clamp(lo, hi);
            }
            Message::SetPaneMin(pane, v) => {
                let (lo, hi) = crate::PANE_MIN_RANGE;
                let v = v.clamp(lo, hi);
                let p = &mut self.settings.panes;
                match pane {
                    Pane::Project => p.min_project_px = v,
                    Pane::Library => p.min_library_px = v,
                    Pane::Viewport => p.min_viewport_px = v,
                    Pane::Inspector => p.min_inspector_px = v,
                    Pane::Output => p.min_output_px = v,
                }
                // Live-apply: a minimum only means something once the layout obeys it,
                // and the point of the control is to *see* the pane stop shrinking.
                self.apply_fixed_layout();
            }
            Message::RestoreDefaults => {
                // The only escape from a layout the user cannot otherwise undo, so it
                // resets *everything* — pane sizes and minimums included — and applies
                // at once rather than behind a confirm they might not be able to reach.
                self.settings = crate::Settings::default();
                let s = &self.settings;
                self.show_stock = s.view.show_stock;
                self.show_gizmo = s.view.show_gizmo;
                self.show_origin = s.view.show_origin;
                self.tooltips = s.view.tooltips;
                self.gizmo_size = s.view.gizmo_size;
                self.snaps = s.snapping.default_snaps.clone();
                // origin_marker_scale and the default post are read straight from
                // `settings` wherever they are used, so resetting the struct is enough.
                self.project_px = s.panes.project_px;
                self.inspector_px = s.panes.inspector_px;
                self.output_px = s.panes.output_px;
                self.apply_fixed_layout();
                self.settings.save();
                self.status = "Preferences restored to defaults.".to_string();
            }
            Message::PaneResized(pane_grid::ResizeEvent { split, ratio }) => {
                let ratio = self.clamp_resize(split, ratio);
                self.panes.resize(split, ratio);
                // Persist the dragged size so a later window resize keeps it —
                // and, now, so the next run opens with the layout the user chose.
                self.capture_side_px(split, ratio);
                self.remember();
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
                // Straight flute only applies to a Square End Mill; changing to any other
                // kind falls back to a down-cut.
                let fix_dir = |t: &mut cam_model::Tool| {
                    t.kind = kind;
                    if !matches!(kind, ToolKind::EndMill) && t.cutting_direction == CutDir::Straight
                    {
                        t.cutting_direction = CutDir::Down;
                    }
                };
                if self.library_mode() {
                    // Edit the working copy (a pending change committed on Apply), not the
                    // library entry — keeps any in-progress numeric edits from being lost.
                    if let Some(t) = self.tool_edit.as_mut() {
                        // Crossing tool families resets the dimensions to the new kind's
                        // defaults (a face mill shouldn't inherit an end mill's ⌀6);
                        // staying within the end-mill family keeps the dimensions and just
                        // swaps the cutting end.
                        if end_mill_family(t.kind) && end_mill_family(kind) {
                            fix_dir(t);
                        } else {
                            *t = crate::tool_library::default_tool(t.number, kind);
                        }
                    }
                    // The kind-specific fields depend on the kind — repopulate them from
                    // the working copy (refresh_fields does *not* reset the baseline).
                    self.refresh_fields();
                } else if let Selection::Tool(i) = self.controller.selection() {
                    self.controller.edit_tool(i, fix_dir);
                    self.rerun();
                    self.refresh_fields();
                }
            }
            Message::ToolCuttingDirChanged(dir) => {
                if self.library_mode() {
                    if let Some(t) = self.tool_edit.as_mut() {
                        t.cutting_direction = dir;
                    }
                    self.refresh_fields();
                } else if let Selection::Tool(i) = self.controller.selection() {
                    self.controller.edit_tool(i, |t| t.cutting_direction = dir);
                    self.rerun();
                    self.refresh_fields();
                }
            }
            Message::ThreadFormChanged(form) => {
                // Single-point ⇒ pitch None + a reduced neck (the single-tooth head);
                // full-form ⇒ Some(pitch), a stacked-teeth band (neck ⌀ unused).
                let set_form = |t: &mut cam_model::Tool| {
                    if let ToolKind::ThreadMill { pitch } = &mut t.kind {
                        match form {
                            ThreadForm::SinglePoint => {
                                *pitch = None;
                                if t.neck_diameter <= 0.0 || t.neck_diameter >= t.diameter {
                                    t.neck_diameter = (t.diameter * 0.7).max(0.1);
                                }
                            }
                            ThreadForm::FullForm => *pitch = Some(pitch.unwrap_or(1.0)),
                        }
                    }
                };
                if self.library_mode() {
                    if let Some(t) = self.tool_edit.as_mut() {
                        set_form(t);
                    }
                    self.refresh_fields();
                } else if let Selection::Tool(i) = self.controller.selection() {
                    self.controller.edit_tool(i, set_form);
                    self.rerun();
                    self.refresh_fields();
                }
            }
            Message::LeadInKindChanged(kind) => {
                self.controller.edit_selected_operation(|op| match op {
                    Operation::Profile(p) => p.lead_in = kind.to_lead(p.lead_in),
                    Operation::Chamfer(c) => c.lead_in = kind.to_lead(c.lead_in),
                    Operation::Pocket(p) => p.clear.lead_in = kind.to_lead(p.clear.lead_in),
                    _ => {}
                });
                self.refresh_fields();
                self.rerun();
            }
            Message::LeadOutKindChanged(kind) => {
                self.controller.edit_selected_operation(|op| match op {
                    Operation::Profile(p) => p.lead_out = kind.to_lead(p.lead_out),
                    Operation::Chamfer(c) => c.lead_out = kind.to_lead(c.lead_out),
                    Operation::Pocket(p) => p.clear.lead_out = kind.to_lead(p.clear.lead_out),
                    _ => {}
                });
                self.refresh_fields();
                self.rerun();
            }
            Message::PlungeKindChanged(kind) => {
                self.controller.edit_selected_operation(|op| match op {
                    Operation::Profile(p) => p.plunge = kind.to_plunge(),
                    Operation::Pocket(p) => p.clear.plunge = kind.to_plunge(),
                    // The V-bit's own entry — `CarveClearPlungeChanged` is the clearing
                    // pass's, and the two must not be crossed.
                    Operation::Carve(c) => c.plunge = kind.to_plunge(),
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
            Message::ThreadGradualToggled(on) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Thread(t) = op {
                        t.gradual = on;
                    }
                });
                self.refresh_fields();
                self.rerun();
            }
            Message::ClearingClimbToggled(on) => {
                self.controller.edit_selected_operation(|op| match op {
                    Operation::Pocket(p) => p.clear.clearing.climb = on,
                    Operation::Profile(p) => p.clearing.climb = on,
                    _ => {}
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
                // The visible field set can change with this choice, so reseed the
                // buffers -- a newly revealed field would otherwise render blank.
                self.refresh_fields();
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
            Message::CarveStayDownToggled(on) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Carve(c) = op {
                        c.stay_down = on;
                    }
                });
                self.rerun();
            }
            Message::CarveClearToggled(on) => {
                // Turning it on seeds the first flat-bottomed tool in the library, so
                // the operator gets a working clearing pass from one click and refines
                // it after. Turning it off leaves the carve V-bit-only (and ridged).
                let seed = on
                    .then(|| {
                        self.library
                            .tools
                            .iter()
                            .find(|t| matches!(t.kind, ToolKind::EndMill | ToolKind::BullNose { .. }))
                            .copied()
                    })
                    .flatten()
                    .map(|t| self.controller.use_tool(t));
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Carve(c) = op {
                        c.clear = match (on, seed) {
                            // Seed the feeds from the carve's own, so the pass is
                            // runnable from one click; they are editable straight after.
                            (true, Some(tool)) => Some(CarveClearing {
                                tool,
                                params: ClearParams {
                                    feed: c.feed,
                                    plunge_feed: c.plunge_feed,
                                    ..Default::default()
                                },
                            }),
                            _ => None,
                        };
                    }
                });
                // The visible field set can change with this choice, so reseed the
                // buffers -- a newly revealed field would otherwise render blank.
                self.refresh_fields();
                self.rerun();
            }
            Message::CarveClearToolChanged(index) => {
                let Some(tool) = self.library.tools.get(index).copied() else {
                    return iced::Task::none();
                };
                let number = self.controller.use_tool(tool);
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Carve(c) = op {
                        // Keep the parameters; only the cutter changes.
                        match &mut c.clear {
                            Some(cl) => cl.tool = number,
                            none => {
                                *none = Some(CarveClearing {
                                    tool: number,
                                    params: ClearParams::default(),
                                })
                            }
                        }
                    }
                });
                self.rerun();
            }
            Message::CarveClearPlungeChanged(kind) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Carve(c) = op {
                        if let Some(cl) = &mut c.clear {
                            cl.params.plunge = kind.to_plunge();
                        }
                    }
                });
                // The visible field set can change with this choice, so reseed the
                // buffers -- a newly revealed field would otherwise render blank.
                self.refresh_fields();
                self.rerun();
            }
            Message::CarveClearClimbToggled(on) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Carve(c) = op {
                        if let Some(cl) = &mut c.clear {
                            cl.params.clearing.climb = on;
                        }
                    }
                });
                self.rerun();
            }
            Message::CarveClearLeadInChanged(kind) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Carve(c) = op {
                        if let Some(cl) = &mut c.clear {
                            cl.params.lead_in = kind.to_lead(cl.params.lead_in);
                        }
                    }
                });
                // The visible field set can change with this choice, so reseed the
                // buffers -- a newly revealed field would otherwise render blank.
                self.refresh_fields();
                self.rerun();
            }
            Message::CarveClearLeadOutChanged(kind) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Carve(c) = op {
                        if let Some(cl) = &mut c.clear {
                            cl.params.lead_out = kind.to_lead(cl.params.lead_out);
                        }
                    }
                });
                // The visible field set can change with this choice, so reseed the
                // buffers -- a newly revealed field would otherwise render blank.
                self.refresh_fields();
                self.rerun();
            }
            Message::NewTool => {
                // Add a tool to the library and select it for editing. Its **type**
                // seeds from the currently-selected tool (a chamfer mill begets a
                // chamfer mill), defaulting to an end mill when nothing is selected. If
                // an op wizard is active, also embed it and pick it for the pending op.
                let seed_kind = self
                    .library
                    .tools
                    .get(self.lib_sel)
                    .map(|t| t.kind)
                    .unwrap_or(ToolKind::EndMill);
                self.lib_sel = self.library.add_of_kind(seed_kind);
                self.library.save();
                if self.controller.pending_op().is_some() {
                    if let Some(&tool) = self.library.tools.get(self.lib_sel) {
                        let number = self.controller.use_tool(tool);
                        self.controller.set_pending_tool(number);
                    }
                }
                self.reload_tool_edit();
            }
            Message::DeleteTool => {
                if self.lib_sel < self.library.tools.len() && self.library.tools.len() > 1 {
                    self.library.tools.remove(self.lib_sel);
                    self.lib_sel = self.lib_sel.min(self.library.tools.len() - 1);
                    self.library.save();
                    self.reload_tool_edit();
                }
            }
            Message::ToolMenu(number) => {
                self.open_tool_menu = Some(number);
                self.tool_menu_pos = self.window_cursor;
            }
            Message::CloseToolMenu => self.open_tool_menu = None,
            Message::AddToolToLibrary(number) => {
                // Promote the project tool numbered `number` into the shop library, then
                // reconcile so it adopts the shop number (§6.3). Idempotent by geometry.
                self.open_tool_menu = None;
                let tool = self
                    .controller
                    .document()
                    .setup
                    .tools
                    .iter()
                    .find(|t| t.number == number)
                    .copied();
                if let Some(tool) = tool {
                    self.library.add_tool(tool);
                    self.library.save();
                    let report = self.controller.reconcile_tools(&self.library.tools);
                    self.refresh_fields();
                    self.status = match report.summary() {
                        Some(s) => format!("Added tool to library. {s}."),
                        None => "Added tool to library.".to_string(),
                    };
                }
            }
            Message::SelectLibraryTool(i) => {
                self.lib_sel = i;
                self.reload_tool_edit();
            }
            Message::SetLibraryView(view) => self.library_view = view,
            Message::LibToolMenu(i) => {
                self.lib_sel = i;
                self.reload_tool_edit();
                self.lib_menu = Some(LibMenu { index: i, input: None });
                self.lib_menu_pos = self.window_cursor;
            }
            Message::LibMenuSetNumber => {
                if let Some(menu) = &mut self.lib_menu {
                    let cur = self
                        .library
                        .tools
                        .get(menu.index)
                        .map(|t| t.number.to_string())
                        .unwrap_or_default();
                    menu.input = Some(cur);
                }
            }
            Message::LibNumberInput(s) => {
                if let Some(menu) = &mut self.lib_menu {
                    menu.input = Some(s);
                }
            }
            Message::LibNumberCommit => {
                if let Some(menu) = &self.lib_menu {
                    if let Some(n) = menu.input.as_ref().and_then(|s| s.trim().parse::<u32>().ok()) {
                        self.library.set_number(menu.index, n);
                        self.library.save();
                        self.reload_tool_edit();
                    }
                }
                self.lib_menu = None;
            }
            Message::CloseLibMenu => self.lib_menu = None,
            Message::RenumberLibrary => {
                let n = self.library.tools.len();
                let by = match self.library_view {
                    LibraryView::Ordered => "current order (compacting any gaps)",
                    LibraryView::Grouped => "family, then diameter",
                };
                let detail = format!(
                    "Renumber all {n} library tools sequentially (T1…T{n}) by {by}?\n\n\
                     This changes every tool's number and cannot be undone. Open projects \
                     re-align to the new numbering when next opened.",
                );
                return iced::Task::perform(confirm_renumber(detail), Message::RenumberConfirmed);
            }
            Message::RenumberConfirmed(true) => {
                let order = self.renumber_order();
                self.library.set_numbers_in_order(&order);
                self.library.save();
                self.reload_tool_edit();
                self.status = "Renumbered the tool library.".to_string();
            }
            Message::RenumberConfirmed(false) => {}
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
                let was_tooling = self.active_tab == RibbonTab::Tooling;
                let now_tooling = tab == RibbonTab::Tooling;
                self.active_tab = tab;
                // Default behaviour: entering the Tooling tab substitutes the Tool
                // Library pane for the Project pane (and vice versa on leaving). The
                // user can still re-open either from the Windows tab to see both.
                if now_tooling && !was_tooling {
                    self.set_pane_visible(Pane::Project, false);
                    self.set_pane_visible(Pane::Library, true);
                } else if was_tooling && !now_tooling {
                    self.set_pane_visible(Pane::Library, false);
                    self.set_pane_visible(Pane::Project, true);
                }
                // Entering Machinery puts the Inspector on the machine, the same idiom
                // Tooling uses for tools — which is why the tab needs no "Machine" button
                // of its own: the tab *is* the button.
                if tab == RibbonTab::Machinery {
                    self.controller.select(Selection::Machine);
                    self.refresh_fields();
                }
                // The Tooling tab turns the Inspector into the library editor, so the
                // field buffers (and the working-copy baseline) must reload for context.
                self.reload_tool_edit();
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
        let mins = &self.settings.panes;
        let lo = (subtree_min(a, &self.panes, axis, mins) / dim).clamp(0.0, 0.95);
        let hi = (1.0 - subtree_min(b, &self.panes, axis, mins) / dim).clamp(lo, 1.0);
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
        let frac = (px.max(pane.min_size(&self.settings.panes)) / dim).clamp(0.0, 1.0);
        let ratio = if in_a { frac } else { 1.0 - frac };
        let ratio = self.clamp_resize(split, ratio);
        self.panes.resize(split, ratio);
    }

    /// Write the controller's machine and post back into the library entry, following a
    /// rename, and persist.
    ///
    /// The Machine inspector edits the *active* machine in place, and the name is the
    /// handle the selection is remembered by — so a rename has to move the entry and the
    /// remembered selection together, or the next launch resolves nothing.
    fn sync_active_machine(&mut self) {
        let entry = crate::MachineEntry {
            machine: self.controller.machine().clone(),
            post: self.controller.post_kind(),
        };
        let settled = self.machines.replace(&self.active_machine, entry);
        // `replace` keeps names unique, so a rename onto a name already taken comes back
        // disambiguated. Push that back, or the inspector would show one name while the
        // library held another — and the selection is remembered by the library's.
        if settled != self.controller.machine().name {
            self.controller.edit_machine(|m| m.name = settled.clone());
        }
        self.active_machine = settled;
        self.machines.save();
        self.remember();
    }

    /// Copy the live view/snap/pane state into the preferences and write them out.
    ///
    /// Called from every handler that changes one of them. The mapping itself lives in
    /// `Settings::remember_session` so it can be tested headlessly; this is only the
    /// gathering.
    fn remember(&mut self) {
        self.settings.remember_session(
            crate::ViewPrefs {
                show_stock: self.show_stock,
                show_gizmo: self.show_gizmo,
                show_origin: self.show_origin,
                show_envelope: self.show_envelope,
                tooltips: self.tooltips,
                gizmo_size: self.gizmo_size,
                // Not mirrored on `App` — the panel writes it straight into `settings`,
                // so carry the current value through rather than a default.
                origin_marker_scale: self.settings.view.origin_marker_scale,
                extra: Default::default(),
            },
            self.snaps.clone(),
            crate::SessionState {
                machine: Some(self.active_machine.clone()),
                extra: Default::default(),
            },
            crate::PanePrefs {
                project_px: self.project_px,
                inspector_px: self.inspector_px,
                output_px: self.output_px,
                // The minimums are not session state — they are set in preferences,
                // and remembering a layout must not reset them.
                ..self.settings.panes.clone()
            },
        );
        self.settings.save();
    }

    /// Hold the non-Viewport panes at their fixed pixel sizes, letting the Viewport
    /// absorb the rest. Outermost split first so a parent's dimension is settled
    /// before its children read it: Output (a height) and Project (a width) are
    /// independent, while Inspector's region width depends on Project — so it is set
    /// last. Hidden panes are simply skipped (`set_pane_px` early-returns).
    fn apply_fixed_layout(&mut self) {
        self.set_pane_px(Pane::Output, self.output_px);
        self.set_pane_px(Pane::Project, self.project_px);
        // The Library pane shares the left slot's width with Project.
        self.set_pane_px(Pane::Library, self.project_px);
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
                Pane::Project | Pane::Library => self.project_px = px,
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

    /// Seed the wizard's editable cutting-data buffers (RPM / feed / plunge) from the
    /// pending operation's chosen tool nominals. A no-op unless a tool is set.
    fn seed_wizard_cutting(&mut self) {
        let Some(tool) = self.controller.pending_op().and_then(|p| p.tool) else {
            return;
        };
        let c = self.controller.seeded_cutting_for(tool);
        self.fields.insert(Field::SpindleRpm, fmt_num(c.rpm));
        self.fields.insert(Field::Feed, fmt_num(c.feed));
        self.fields.insert(Field::PlungeFeed, fmt_num(c.plunge_feed));
    }

    /// The cutting data currently in the wizard buffers, parsed. Missing/blank fields
    /// fall back to the pending tool's seeded nominals, so a confirm never loses them.
    fn wizard_cutting(&self) -> Option<CuttingData> {
        let tool = self.controller.pending_op().and_then(|p| p.tool)?;
        let seeded = self.controller.seeded_cutting_for(tool);
        let buf = |f: Field| self.fields.get(&f).and_then(|s| s.parse::<f64>().ok());
        Some(CuttingData {
            rpm: buf(Field::SpindleRpm).unwrap_or(seeded.rpm).max(0.0),
            feed: buf(Field::Feed).unwrap_or(seeded.feed).max(0.0),
            plunge_feed: buf(Field::PlungeFeed).unwrap_or(seeded.plunge_feed).max(0.0),
        })
    }

    /// Reset the Tooling working copy to the committed library tool — the clean baseline
    /// the dirty check (and thus the Apply button) compares against — and reload the
    /// field buffers from it. Called at every "load" point (tool selection, New, Delete,
    /// tab switch, post-Apply), but *not* on an in-place field/picker edit.
    fn reload_tool_edit(&mut self) {
        self.tool_edit = if self.library_mode() {
            self.library.tools.get(self.lib_sel).copied()
        } else {
            None
        };
        self.refresh_fields();
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

    /// The library indices in the order the bulk renumber will assign 1..N — the same
    /// order the current Library-pane tab displays (Ordered by number, Grouped by
    /// family then diameter).
    fn renumber_order(&self) -> Vec<usize> {
        let mut idx: Vec<usize> = (0..self.library.tools.len()).collect();
        match self.library_view {
            LibraryView::Ordered => idx.sort_by_key(|&i| self.library.tools[i].number),
            LibraryView::Grouped => idx.sort_by(|&a, &b| {
                let (ta, tb) = (&self.library.tools[a], &self.library.tools[b]);
                kind_order(ta.kind)
                    .cmp(&kind_order(tb.kind))
                    .then(ta.diameter.total_cmp(&tb.diameter))
                    .then(ta.number.cmp(&tb.number))
            }),
        }
        idx
    }

    /// The plunge style an operation enters with, and (separately) the one its clearing
    /// pass uses — the two are independent settings on a carve.
    fn op_plunge_styles(op: &Operation) -> (Option<Plunge>, Option<Plunge>) {
        match op {
            Operation::Profile(p) => (Some(p.plunge), None),
            Operation::Pocket(p) => (Some(p.clear.plunge), None),
            Operation::Carve(c) => (Some(c.plunge), c.clear.as_ref().map(|cl| cl.params.plunge)),
            _ => (None, None),
        }
    }

    /// The label for a plunge parameter, named for what it actually **is** under the
    /// selected style. `None` when the field is not a plunge parameter.
    ///
    /// These were positional — "Plunge angle/radius" and "Plunge length/pitch" — one
    /// slot standing for two physically distinct quantities depending on the style, and
    /// the label had to say so because it could not name them. An angle and a radius
    /// are not the same kind of thing (Andreas, 2026-08-01): **a ramp is always an
    /// angle in degrees, never a radius; a helix is always a radius and a pitch, both
    /// in millimetres.** The tool inspector has been kind-aware for a long time; this
    /// is the same treatment for the entry styles.
    fn plunge_label(&self, field: Field) -> Option<&'static str> {
        let op = self.controller.selected_operation()?;
        let (own, clear) = Self::op_plunge_styles(op);
        let (plunge, clearing) = match field {
            Field::PlungeA | Field::PlungeB => (own?, false),
            Field::ClearPlungeA | Field::ClearPlungeB => (clear?, true),
            _ => return None,
        };
        let first = matches!(field, Field::PlungeA | Field::ClearPlungeA);
        Some(match (plunge, first, clearing) {
            (Plunge::Ramp { .. }, true, false) => "Ramp angle (°)",
            (Plunge::Ramp { .. }, true, true) => "Clearing ramp angle (°)",
            (Plunge::Helix { .. }, true, false) => "Helix radius (mm)",
            (Plunge::Helix { .. }, false, false) => "Helix pitch (mm)",
            (Plunge::Helix { .. }, true, true) => "Clearing helix radius (mm)",
            (Plunge::Helix { .. }, false, true) => "Clearing helix pitch (mm)",
            (Plunge::ZigZag { .. }, true, false) => "Zig-zag angle (°)",
            (Plunge::ZigZag { .. }, false, false) => "Zig-zag length (mm)",
            (Plunge::ZigZag { .. }, true, true) => "Clearing zig-zag angle (°)",
            (Plunge::ZigZag { .. }, false, true) => "Clearing zig-zag length (mm)",
            _ => return None,
        })
    }

    /// The kind of the tool the inspector is editing (library entry or project tool).
    fn inspected_tool_kind(&self) -> Option<ToolKind> {
        if self.library_mode() {
            // Prefer the working copy so a Type change (which edits it, not the committed
            // entry) immediately drives the kind-specific field set, labels and validation.
            return self
                .tool_edit
                .map(|t| t.kind)
                .or_else(|| self.library.tools.get(self.lib_sel).map(|t| t.kind));
        }
        if let Selection::Tool(i) = self.controller.selection() {
            self.controller.document().setup.tools.get(i).map(|t| t.kind)
        } else {
            None
        }
    }

    /// The inspector label for a field, kind-aware. A V-bit's transverse measurement is
    /// its shaft, not a (variable) flute diameter, so `ToolDiameter` reads "Shank
    /// diameter" there.
    fn field_label(&self, field: Field) -> &'static str {
        match (self.inspected_tool_kind(), field) {
            // V-bit and chamfer mill: the single transverse ⌀ is the shaft they flare to.
            (Some(ToolKind::VBit { .. } | ToolKind::ChamferMill { .. }), Field::ToolDiameter) => {
                "Shank diameter (mm)"
            }
            // Face mill: cutting-⌀ disc, and the flute/shank fields describe the shell-mill
            // body and its arbor.
            (Some(ToolKind::FaceMill), Field::ToolDiameter) => "Cutting diameter (mm)",
            (Some(ToolKind::FaceMill), Field::FluteLength) => "Body height (mm)",
            (Some(ToolKind::FaceMill), Field::ShankDiameter) => "Arbor diameter (mm)",
            // Thread mill: a single-point mill's ⌀ is the *minimum* cutting ⌀ (smallest
            // hole); FluteLength is its length of cut (reach) and it exposes the reduced
            // neck. A full-form mill uses the plain cutting ⌀ and thread length.
            (Some(ToolKind::ThreadMill { pitch: None }), Field::ToolDiameter) => {
                "Min cutting diameter (mm)"
            }
            (Some(ToolKind::ThreadMill { pitch: Some(_) }), Field::ToolDiameter) => {
                "Cutting diameter (mm)"
            }
            (Some(ToolKind::ThreadMill { pitch: None }), Field::FluteLength) => "Length of cut (mm)",
            (Some(ToolKind::ThreadMill { pitch: Some(_) }), Field::FluteLength) => {
                "Thread length (mm)"
            }
            (Some(ToolKind::ThreadMill { .. }), Field::NeckDiameter) => "Neck diameter (mm)",
            _ => self.plunge_label(field).unwrap_or_else(|| field.label()),
        }
    }

    /// Kind-aware tooltip for a Tooling-inspector field — the same field means different
    /// things across tool kinds (a V-bit's `ToolDiameter` is its shaft, a face mill's is
    /// the cutting disc, …), so the help matches the (kind-aware) label. Non-tool fields,
    /// and kinds where the generic wording already fits, fall back to `field.help()`.
    fn field_help(&self, field: Field) -> &'static str {
        use ToolKind::*;
        match (self.inspected_tool_kind(), field) {
            // Cutting/flute diameter — the meaning turns on the kind.
            (Some(EndMill | BallMill | BullNose { .. }), Field::ToolDiameter) => {
                "Diameter across the flutes — the end mill's cutting diameter."
            }
            (Some(Drill { .. }), Field::ToolDiameter) => {
                "The drill's cutting diameter — the ⌀ of the hole it produces."
            }
            (Some(VBit { .. } | ChamferMill { .. }), Field::ToolDiameter) => {
                "Shaft diameter — the cone flares up to this, and the shaft continues \
                 above it. (These tools have no single flute ⌀: the cutting ⌀ varies \
                 continuously along the cone.)"
            }
            (Some(FaceMill), Field::ToolDiameter) => {
                "Cutting diameter — the ⌀ of the disc the inserts sweep; it drives the \
                 facing stepover."
            }
            (Some(ThreadMill { pitch: None }), Field::ToolDiameter) => {
                "Minimum cutting diameter — the tooth-crest ⌀, i.e. the smallest hole the \
                 single-point mill can enter and thread."
            }
            (Some(ThreadMill { pitch: Some(_) }), Field::ToolDiameter) => {
                "Cutting diameter of the thread mill — it must clear the thread's minor \
                 diameter to enter the hole."
            }
            (Some(ThreadMill { .. }), Field::NeckDiameter) => {
                "Neck diameter — the reduced undercut behind the tooth. It sets the maximum \
                 thread depth (and so the coarsest pitch): (min cutting ⌀ − neck ⌀) / 2. A \
                 smaller neck clears deeper forms."
            }
            // Cutting-length family.
            (Some(Drill { .. }), Field::FluteLength) => {
                "Flute length — the fluted (cutting) portion from the tip; the plain shank \
                 continues above it. Must be at least the drill-point cone height."
            }
            (Some(FaceMill), Field::FluteLength) => {
                "Body height — the axial length of the wide cutting body; the narrower \
                 arbor continues above it up to the overall length."
            }
            (Some(ThreadMill { pitch: None }), Field::FluteLength) => {
                "Length of cut — how far down from the tip the single tooth can thread \
                 (the reduced-neck reach); it sets the maximum threaded depth."
            }
            (Some(ThreadMill { pitch: Some(_) }), Field::FluteLength) => {
                "Thread length — the length of the threaded cutting band, which bounds the \
                 thread depth reachable in one helical pass."
            }
            // Shank / arbor.
            (Some(FaceMill), Field::ShankDiameter) => {
                "Arbor diameter — the mounting stub above the cutting body (narrower than \
                 the cutting ⌀)."
            }
            // Point angle serves both the drill and the V-bit.
            (Some(Drill { .. }), Field::PointAngle) => {
                "Included angle of the drill point (e.g. 118° or 135°), placed so the full \
                 diameter reaches the intended depth. Bounded to 90°–135°."
            }
            (Some(VBit { .. }), Field::PointAngle) => {
                "Included angle of the V-bit's cone (e.g. 60° or 90°) — the point angle \
                 that sets how cut depth maps to cut width."
            }
            _ => field.help(),
        }
    }

    /// Whether a field's current buffer value is invalid (flagged red). Rules:
    /// - **Corner radius** ≤ flute radius (⌀/2) — rounded-edge end mill.
    /// - **Flute length** > 0 everywhere; ≥ corner radius (rounded-edge); ≥ flute radius
    ///   ⌀/2 (ball nose) — the cutting-end feature must fit inside the flute.
    fn field_invalid(&self, field: Field) -> bool {
        let buf = |f: Field| self.fields.get(&f).and_then(|s| s.parse::<f64>().ok());
        match field {
            Field::CornerRadius => {
                matches!((buf(Field::CornerRadius), buf(Field::ToolDiameter)),
                    (Some(cr), Some(d)) if cr > d * 0.5 + 1e-9)
            }
            Field::PointAngle => {
                let Some(a) = buf(Field::PointAngle) else {
                    return true;
                };
                match self.inspected_tool_kind() {
                    // Drill point angle is bounded to [90°, 135°].
                    Some(ToolKind::Drill { .. }) => !(90.0..=135.0).contains(&a),
                    // A V-bit's included angle just has to be a valid cone.
                    Some(ToolKind::VBit { .. }) => !(0.0 < a && a < 180.0),
                    _ => false,
                }
            }
            Field::TipRadius => {
                // Physically a V-bit's point is always *rounded* — a true r=0 edge
                // cannot be ground and would not survive contact. The rounded tip is
                // also what makes a V-bit cut at its centre (and so engrave), which is
                // exactly what distinguishes it from a chamfer mill's flat. So the
                // radius must be positive, and cannot exceed the shaft radius (⌀/2).
                match (buf(Field::TipRadius), buf(Field::ToolDiameter)) {
                    (Some(tr), _) if tr < MIN_TIP_RADIUS_MM - 1e-9 => true,
                    (Some(tr), Some(d)) => tr > d * 0.5 + 1e-9,
                    _ => false,
                }
            }
            Field::ChamferAngle => {
                // A chamfer mill's included angle just has to be a valid cone.
                let Some(a) = buf(Field::ChamferAngle) else {
                    return true;
                };
                !(0.0 < a && a < 180.0)
            }
            Field::TipDiameter => {
                // Physically a chamfer mill always has a *flat* at its point — it is
                // ground that way, and that flat is precisely why it does not cut at
                // its centre (and so cannot engrave). A zero flat would make it a
                // V-bit, not a chamfer mill, so it must be positive; and it must stay
                // narrower than the shaft ⌀, else there is no cone at all.
                match (buf(Field::TipDiameter), buf(Field::ToolDiameter)) {
                    // Held to the same physical radius as a V-bit's tip, hence 2×.
                    (Some(tip), _) if tip < 2.0 * MIN_TIP_RADIUS_MM - 1e-9 => true,
                    (Some(tip), Some(d)) => tip >= d - 1e-9,
                    _ => false,
                }
            }
            Field::ShankDiameter => {
                // A face mill's arbor cannot be wider than its cutting body (⌀), else the
                // silhouette flares out above the body. Only enforced for face mills.
                matches!(self.inspected_tool_kind(), Some(ToolKind::FaceMill))
                    && matches!((buf(Field::ShankDiameter), buf(Field::ToolDiameter)),
                        (Some(arbor), Some(d)) if arbor > d + 1e-9)
            }
            Field::ToolThreadPitch => {
                // Shown only for a full-form thread mill, where the pitch must be positive.
                !matches!(buf(Field::ToolThreadPitch), Some(p) if p > 0.0)
            }
            Field::NeckDiameter => {
                // A single-point thread mill's neck must be strictly reduced (smaller than
                // the min cutting ⌀), else there is no undercut / thread-depth clearance.
                matches!(self.inspected_tool_kind(), Some(ToolKind::ThreadMill { .. }))
                    && matches!((buf(Field::NeckDiameter), buf(Field::ToolDiameter)),
                        (Some(neck), Some(d)) if neck >= d - 1e-9)
            }
            Field::FluteLength => {
                let Some(fl) = buf(Field::FluteLength) else {
                    return true; // empty / unparseable
                };
                if fl <= 1e-9 {
                    return true; // flute length cannot be 0 (anywhere)
                }
                match self.inspected_tool_kind() {
                    Some(ToolKind::BullNose { .. }) => {
                        matches!(buf(Field::CornerRadius), Some(cr) if fl < cr - 1e-9)
                    }
                    Some(ToolKind::BallMill) => {
                        matches!(buf(Field::ToolDiameter), Some(d) if fl < d * 0.5 - 1e-9)
                    }
                    Some(ToolKind::Drill { .. }) => {
                        // Must be at least the point cone height = (⌀/2) / tan(angle/2).
                        matches!((buf(Field::ToolDiameter), buf(Field::PointAngle)),
                            (Some(d), Some(pa)) if pa > 0.0
                                && fl < (d * 0.5) / (pa * 0.5).to_radians().tan() - 1e-9)
                    }
                    _ => false,
                }
            }
            _ => false,
        }
    }

    /// Whether any inspector field is currently invalid (Apply is disabled if so).
    fn any_field_invalid(&self) -> bool {
        self.inspector_fields().iter().any(|&f| self.field_invalid(f))
    }

    /// Apply the flute/shank/overall length constraint to the selected library tool
    /// **live** from the `edited` field's current buffer, then refresh the *other* two
    /// length buffers so the derived value shows immediately. Only fires on a valid
    /// number; the edited field's own buffer is left untouched (mid-typing).
    fn live_edit(&mut self, edited: Field) {
        let Some(v) = self.fields.get(&edited).and_then(|s| s.parse::<f64>().ok()) else {
            return; // empty / unparseable: hold the last valid working copy
        };
        if v < 0.0 || (edited == Field::FluteLength && v <= 1e-9) {
            return; // never write a zero/negative flute (keeps the last valid state)
        }
        let mut parsed: BTreeMap<Field, f64> = BTreeMap::new();
        parsed.insert(edited, v);
        // Apply just the edited field to the working copy (single-field semantics keep
        // the overall = flute + shank coupling correct), then read back the derived
        // length trio to refresh the *other* two buffers for display.
        let derived = {
            let Some(t) = self.tool_edit.as_mut() else {
                return;
            };
            apply_tool_dims(t, &parsed);
            apply_tool_kind_fields(&mut t.kind, &parsed);
            [
                (Field::FluteLength, t.flute_len()),
                (Field::ShankLength, (t.length - t.flute_len()).max(0.0)),
                (Field::ToolLength, t.length),
            ]
        };
        for (f, val) in derived {
            if f != edited {
                self.fields.insert(f, fmt_num(val));
            }
        }
    }

    /// The tool whose cross-section the viewport should preview (Phase 5): the selected
    /// library tool while the Tooling tab is active, else a selected project tool. `None`
    /// ⇒ show the normal 3D backplot.
    fn preview_tool(&self) -> Option<cam_model::Tool> {
        if self.library_mode() {
            // The live working copy (which carries the in-progress edits); fall back to
            // the committed tool before the first edit / working copy is set.
            return self.tool_edit.or(self.library.tools.get(self.lib_sel).copied());
        }
        match self.controller.selection() {
            Selection::Tool(i) => self.controller.document().setup.tools.get(i).copied(),
            _ => None,
        }
    }

    /// Whether the Tooling inspector has uncommitted edits — the working copy differs
    /// from the committed library entry. Outside Tooling mode there is no working copy,
    /// so the Apply button keeps its usual (validity-only) gating.
    fn inspector_dirty(&self) -> bool {
        if self.machine_edit_dirty() {
            return true;
        }
        if !self.library_mode() {
            // Same promise the Tooling editor makes, everywhere else: Apply is live only
            // when something is actually pending. The buffers are seeded from the model
            // by `refresh_fields`, so an untouched inspector compares equal all the way
            // down and the button stays grey — which is what tells the operator, at a
            // glance, whether an edit has been committed or is still sitting in the box.
            return fields_are_dirty(&self.inspector_fields(), &self.fields, |f| {
                self.field_value(f)
            });
        }
        match (self.tool_edit, self.library.tools.get(self.lib_sel)) {
            (Some(edit), Some(saved)) => edit != *saved,
            _ => false,
        }
    }

    /// Which fields the inspector shows for the current selection.
    fn inspector_fields(&self) -> Vec<Field> {
        // The field set is **kind-specific** — the end mill has its own characterisation
        // (Andreas): flute ⌀, flute length, shank ⌀, shank length, overall length,
        // flutes — no neck. Other kinds keep the generic set until characterised in turn.
        // The Cutting-direction control is rendered separately (it's a picker, not a
        // numeric field), as is the Type picker.
        let tool_fields = |kind: Option<ToolKind>| {
            let mut fields = match kind {
            // Square & Ball Nose end mills share the exact end-mill field set.
            Some(ToolKind::EndMill | ToolKind::BallMill) => vec![
                Field::ToolDiameter, // "Flute ⌀"
                Field::FluteLength,
                Field::ShankDiameter,
                Field::ShankLength,
                Field::ToolLength, // "Overall length"
                Field::Flutes,
            ],
            // Rounded-Edge (bull-nose) end mill: the same, plus a corner radius.
            Some(ToolKind::BullNose { .. }) => vec![
                Field::ToolDiameter,
                Field::CornerRadius,
                Field::FluteLength,
                Field::ShankDiameter,
                Field::ShankLength,
                Field::ToolLength,
                Field::Flutes,
            ],
            // Drill bit: flute/shank/overall + point angle. No flutes count, no direction.
            Some(ToolKind::Drill { .. }) => vec![
                Field::ToolDiameter,
                Field::FluteLength,
                Field::ShankDiameter,
                Field::ShankLength,
                Field::ToolLength,
                Field::PointAngle,
            ],
            // V-bit: a single shaft ⌀ (rendered via ToolDiameter, relabelled "Shank
            // diameter" — see `field_label`), overall length, point angle, tip radius.
            // A V-bit has *no* flute diameter: the cutting ⌀ varies along the cone, so
            // the only fixed transverse measurement is the shaft it flares up to.
            Some(ToolKind::VBit { .. }) => vec![
                Field::ToolDiameter,
                Field::ToolLength,
                Field::PointAngle,
                Field::TipRadius,
            ],
            // Chamfer mill: like a V-bit but its tip is a flat (non-cutting) instead of a
            // rounded one — a single shaft ⌀, overall length, point angle, and a flat tip
            // ⌀ in place of the V-bit's tip radius.
            Some(ToolKind::ChamferMill { .. }) => vec![
                Field::ToolDiameter,
                Field::ToolLength,
                Field::ChamferAngle,
                Field::TipDiameter,
            ],
            // Face mill (shell mill): a wide cutting body on a narrower arbor. FluteLength
            // is relabelled "Body height" and ShankDiameter "Arbor diameter" (see
            // `field_label`). No shank-length / neck — the arbor length is overall − body.
            Some(ToolKind::FaceMill) => vec![
                Field::ToolDiameter,
                Field::FluteLength,
                Field::ShankDiameter,
                Field::ToolLength,
                Field::Flutes,
            ],
            // Thread mill (single-point/full-form is a toggle, rendered separately):
            // - single-point (single profile): one 60° tooth + a reduced neck. Min cutting
            //   ⌀ (smallest hole), neck ⌀ (sets max thread depth), length of cut (reach),
            //   shank ⌀/length, overall. The blind-hole allowance is derived per operation.
            // - full-form: a long threaded band at a fixed pitch (cutting ⌀, thread length,
            //   shank ⌀, shank length, overall, pitch). No neck.
            Some(ToolKind::ThreadMill { pitch: None }) => vec![
                Field::ToolDiameter,
                Field::NeckDiameter,
                Field::FluteLength,
                Field::ShankDiameter,
                Field::ShankLength,
                Field::ToolLength,
                Field::Flutes,
            ],
            Some(ToolKind::ThreadMill { pitch: Some(_) }) => vec![
                Field::ToolDiameter,
                Field::FluteLength,
                Field::ShankDiameter,
                Field::ShankLength,
                Field::ToolLength,
                Field::ToolThreadPitch,
                Field::Flutes,
            ],
            other => {
                let mut f = vec![
                    Field::ToolDiameter,
                    Field::ToolLength,
                    Field::FluteLength,
                    Field::ShankDiameter,
                    Field::NeckLength,
                    Field::NeckDiameter,
                    Field::Flutes,
                ];
                if let Some(k) = other {
                    f.extend(tool_kind_fields(k));
                }
                f
            }
            };
            // Nominal cutting data is common to every tool kind — the library defaults
            // that seed a new operation's RPM/feed/plunge.
            fields.extend([
                Field::NominalRpm,
                Field::NominalFeed,
                Field::NominalPlungeFeed,
            ]);
            fields
        };
        if self.library_mode() {
            // Use the working-copy kind (via inspected_tool_kind) so a Type change swaps
            // in the new field set immediately, without waiting for Apply.
            return tool_fields(self.inspected_tool_kind());
        }
        match self.controller.selection() {
            // Ordered top-down by height: tool-change height, clearance, retract, then
            // the stock top at the bottom — the inspector reads like the Z stack.
            Selection::Setup => vec![
                Field::ToolChangeHeight,
                Field::Clearance,
                Field::Retract,
                Field::TopOfStock,
            ],
            Selection::Machine => vec![
                Field::MachineTravelX,
                Field::MachineTravelY,
                Field::MachineTravelZ,
            ],
            Selection::Origin => vec![
                Field::OriginIndex,
                Field::OriginX,
                Field::OriginY,
                Field::OriginZ,
            ],
            Selection::Tool(i) => {
                tool_fields(self.controller.document().setup.tools.get(i).map(|t| t.kind))
            }
            Selection::Stock => vec![
                Field::StockXOffset,
                Field::StockYOffset,
                Field::StockTop,
                Field::StockThickness,
            ],
            Selection::Operation(id) => self
                .controller
                .operation(id)
                .map(operation_fields)
                .unwrap_or_default(),
        }
    }
    /// The model value backing a field for the current selection, if any.
    fn field_value(&self, field: Field) -> Option<f64> {
        if self.library_mode() {
            // Prefer the live working copy so kind/picker edits repopulate from it.
            let t = self
                .tool_edit
                .as_ref()
                .or_else(|| self.library.tools.get(self.lib_sel))?;
            return match field {
                Field::ToolDiameter => Some(t.diameter),
                Field::ToolLength => Some(t.length),
                // Effective flute (resolves the 0-sentinel), so shank = overall − flute.
                Field::FluteLength => Some(t.flute_len()),
                Field::ShankDiameter => Some(t.shank_dia()),
                Field::ShankLength => Some((t.length - t.flute_len()).max(0.0)),
                Field::NeckLength => Some(t.neck_length),
                Field::NeckDiameter => Some(t.neck_diameter),
                Field::Flutes => Some(t.flutes as f64),
                Field::NominalRpm => Some(t.nominal_rpm),
                Field::NominalFeed => Some(t.nominal_feed),
                Field::NominalPlungeFeed => Some(t.nominal_plunge_feed),
                _ => tool_kind_field(t.kind, field),
            };
        }
        if let Field::MachineTravelX | Field::MachineTravelY | Field::MachineTravelZ = field {
            let (x, y, z) = self.controller.machine().envelope.extent();
            return Some(match field {
                Field::MachineTravelX => x,
                Field::MachineTravelY => y,
                _ => z,
            });
        }
        let setup = &self.controller.document().setup;
        match field {
            Field::Clearance => Some(setup.heights.clearance),
            Field::Retract => Some(setup.heights.retract),
            Field::TopOfStock => Some(setup.heights.top_of_stock),
            // Shows the *effective* height: the explicit value if set, else the
            // machine-Z-travel default the planner would resolve.
            Field::ToolChangeHeight => {
                Some(setup.resolved_tool_change_height(self.controller.machine().envelope.max.z))
            }
            Field::OriginX => Some(self.controller.origin_position(self.controller.active_origin())[0]),
            Field::OriginY => Some(self.controller.origin_position(self.controller.active_origin())[1]),
            Field::OriginZ => Some(self.controller.origin_position(self.controller.active_origin())[2]),
            Field::OriginIndex => Some(self.controller.active_origin() as f64),
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
            Field::FluteLength => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.flute_len()),
                _ => None,
            },
            Field::ShankDiameter => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.shank_dia()),
                _ => None,
            },
            Field::ShankLength => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| (t.length - t.flute_len()).max(0.0)),
                _ => None,
            },
            Field::Flutes => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.flutes as f64),
                _ => None,
            },
            Field::NominalRpm => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.nominal_rpm),
                _ => None,
            },
            Field::NominalFeed => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.nominal_feed),
                _ => None,
            },
            Field::NominalPlungeFeed => match self.controller.selection() {
                Selection::Tool(i) => setup.tools.get(i).map(|t| t.nominal_plunge_feed),
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
    /// Whether a machine name or post is typed but not yet applied.
    fn machine_edit_dirty(&self) -> bool {
        let name = self
            .machine_name_edit
            .as_ref()
            .is_some_and(|n| n != &self.controller.machine().name);
        let post = self
            .machine_post_edit
            .is_some_and(|p| p != self.controller.post_kind());
        name || post
    }

    /// Commit any pending machine name / post, writing them into the library.
    fn apply_machine_edit(&mut self) {
        if !self.machine_edit_dirty() {
            return;
        }
        if let Some(name) = self.machine_name_edit.clone() {
            self.controller.edit_machine(|m| m.name = name);
        }
        if let Some(post) = self.machine_post_edit {
            self.controller.set_post_kind(post);
        }
        self.sync_active_machine();
        // Re-seed from what was actually stored — `sync_active_machine` may have
        // disambiguated a rename onto a name already taken, and the box must show what
        // the library holds, not what was typed.
        self.machine_name_edit = None;
        self.machine_post_edit = None;
        self.status = format!("Machine: {} ({}).", self.active_machine, self.controller.post_kind());
    }

    fn apply_inspector(&mut self) {
        self.apply_machine_edit();
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
        // embedded copy in a project is a snapshot and is left untouched. The working
        // copy already carries every live edit, so committing is a copy-through; refresh
        // then resets it as the new clean baseline.
        if self.library_mode() {
            if let (Some(edit), Some(slot)) =
                (self.tool_edit, self.library.tools.get_mut(self.lib_sel))
            {
                *slot = edit;
            }
            self.library.save();
            self.reload_tool_edit();
            return;
        }

        match self.controller.selection() {
            Selection::Setup => self.controller.edit_setup(|s| {
                if let Some(&v) = parsed.get(&Field::Clearance) {
                    s.heights.clearance = v;
                }
                if let Some(&v) = parsed.get(&Field::Retract) {
                    s.heights.retract = v;
                }
                if let Some(&v) = parsed.get(&Field::TopOfStock) {
                    s.heights.top_of_stock = v;
                }
                if let Some(&v) = parsed.get(&Field::ToolChangeHeight) {
                    s.tool_change_height = Some(v);
                }
            }),
            Selection::Machine => self.controller.edit_machine(|m| {
                // Travel is the working-volume extent; keep the min corner and set
                // the max to min + travel (clamped positive).
                let e = &mut m.envelope;
                if let Some(&v) = parsed.get(&Field::MachineTravelX) {
                    e.max.x = e.min.x + v.max(0.0);
                }
                if let Some(&v) = parsed.get(&Field::MachineTravelY) {
                    e.max.y = e.min.y + v.max(0.0);
                }
                if let Some(&v) = parsed.get(&Field::MachineTravelZ) {
                    e.max.z = e.min.z + v.max(0.0);
                }
            }),
            Selection::Origin => {
                self.controller.edit_active_origin(|o| {
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
                // The H index applies on Apply (like every field). A round to the
                // nearest positive integer; changing it may swap with another origin.
                if let Some(&v) = parsed.get(&Field::OriginIndex) {
                    let idx = v.round().max(1.0) as u32;
                    if idx != self.controller.active_origin() {
                        self.controller.set_active_origin_index(idx);
                    }
                }
            }
            Selection::Tool(i) => self.controller.edit_tool(i, |t| {
                apply_tool_dims(t, &parsed);
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
        if let Some(menu) = self.tool_menu_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseToolMenu);
            layers = layers.push(catcher).push(menu);
        }
        if let Some(menu) = self.lib_menu_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseLibMenu);
            layers = layers.push(catcher).push(menu);
        }
        if let Some(pickbox) = self.pickbox_overlay() {
            layers = layers.push(pickbox);
        }
        if let Some(card) = self.prefs_overlay() {
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::ClosePrefs);
            layers = layers.push(catcher).push(card);
        }
        if let Some(card) = self.license_overlay() {
            // Same catcher pattern as the menus: a click anywhere off the card closes.
            let catcher = mouse_area(Space::new().width(Length::Fill).height(Length::Fill))
                .on_press(Message::CloseLicense);
            layers = layers.push(catcher).push(card);
        }
        layers.into()
    }

    /// The preferences panel. `None` unless open.
    ///
    /// **Only settings with no other control appear here.** The View ribbon already
    /// toggles stock/cube/origin/tips and sizes the cube, and the snap buttons arm the
    /// object snaps — duplicating those would create two places to change one thing and
    /// the inevitable question of which is authoritative. What has no home anywhere
    /// else is the picking tolerances and the pane minimums, so that is what this is.
    ///
    /// **Live-apply, not Apply-on-dirty.** The two-commit-class convention governs edits
    /// to the *model* — the thing that becomes G-code. These are not that, and a
    /// tolerance you cannot see change while dragging is a tolerance you cannot set.
    fn prefs_overlay(&self) -> Option<Element<'_, Message>> {
        if !self.show_prefs {
            return None;
        }
        let heading = |t: &'static str| text(t).size(14);
        let note = |t: String| text(t).size(11);

        // A labelled slider with its value shown, the shape every row here takes.
        let row_for = |label: &'static str,
                       range: (f32, f32),
                       value: f32,
                       step: f32,
                       shown: String,
                       msg: fn(f32) -> Message|
         -> Element<'_, Message> {
            row![
                text(label).size(12).width(Length::Fixed(150.0)),
                slider(range.0..=range.1, value, msg)
                    .step(step)
                    .on_release(Message::SettingsSettled)
                    .width(Length::Fixed(190.0)),
                text(shown).size(12).width(Length::Fixed(64.0)),
            ]
            .spacing(10)
            .align_y(Alignment::Center)
            .into()
        };

        let snapping = column![
            heading("Picking and snapping"),
            row_for(
                "Pickbox size",
                crate::PICKBOX_RANGE,
                self.settings.snapping.pickbox_px,
                1.0,
                format!("{:.0} px", self.settings.snapping.pickbox_px),
                Message::SetPickbox,
            ),
            // Shown, not settable. The catch distance is a fixed multiple of the
            // pickbox on purpose: two absolute knobs would let it be set *smaller*
            // than the box that feeds it. Showing the derived number keeps the
            // relationship visible instead of merely documented.
            note(format!(
                "Object snaps catch within {:.0} px — {:.1}× the pickbox.",
                self.settings.snap_catch_px(),
                crate::SNAP_CATCH_MULTIPLE,
            )),
            row_for(
                "Snap marker size",
                crate::MARKER_SCALE_RANGE,
                self.settings.snapping.marker_scale,
                0.1,
                format!("{:.1}×", self.settings.snapping.marker_scale),
                Message::SetMarkerScale,
            ),
            row_for(
                "Origin marker size",
                crate::ORIGIN_MARKER_RANGE,
                self.settings.view.origin_marker_scale,
                0.1,
                format!("{:.1}×", self.settings.view.origin_marker_scale),
                Message::SetOriginMarker,
            ),
        ]
        .spacing(6);

        let p = &self.settings.panes;
        let pane_row = |pane: Pane, value: f32| -> Element<'_, Message> {
            // Project / Library / Viewport / Inspector are docked left, centre and
            // right, so their minimum is a **width**; Output is docked along the
            // bottom, so its minimum is a **height**. Same number, different axis —
            // and nothing on screen says so unless the label does.
            let axis = if pane == Pane::Output { "height" } else { "width" };
            row![
                text(format!("{} ({axis})", pane.name()))
                    .size(12)
                    .width(Length::Fixed(150.0)),
                slider(crate::PANE_MIN_RANGE.0..=crate::PANE_MIN_RANGE.1, value, move |v| {
                    Message::SetPaneMin(pane, v)
                })
                .step(5.0_f32)
                .on_release(Message::SettingsSettled)
                .width(Length::Fixed(190.0)),
                text(format!("{value:.0} px")).size(12).width(Length::Fixed(64.0)),
            ]
            .spacing(10)
            .align_y(Alignment::Center)
            .into()
        };

        let panes = column![
            heading("Smallest a pane may be"),
            note(
                "Widths for the side panes, height for Output. These are logical pixels, \
                 so a high-DPI screen with display scaling is already handled. Raise them \
                 on a large screen; lower them on a small or unscaled one, where the \
                 shipped values can leave the viewport too narrow to work in."
                    .to_string()
            ),
            pane_row(Pane::Project, p.min_project_px),
            pane_row(Pane::Library, p.min_library_px),
            pane_row(Pane::Viewport, p.min_viewport_px),
            pane_row(Pane::Inspector, p.min_inspector_px),
            pane_row(Pane::Output, p.min_output_px),
        ]
        .spacing(6);

        let btn = |label: &'static str, msg: Message| {
            button(
                text(label)
                    .size(13)
                    .line_height(iced::widget::text::LineHeight::Relative(1.0))
                    .align_y(Alignment::Center),
            )
            .padding(Padding::from([5.0, 14.0]))
            .on_press(msg)
        };

        let card = container(
            column![
                text("Preferences").size(20),
                note(
                    "Saved to settings.json in your configuration folder, and applied as \
                     you change them."
                        .to_string()
                ),
                snapping,
                panes,
                row![
                    // Left, away from Close: it is destructive, and the two should not
                    // be adjacent when one of them is the escape from an unusable layout.
                    btn("Restore defaults", Message::RestoreDefaults),
                    Space::new().width(Length::Fill),
                    btn("Close", Message::ClosePrefs),
                ]
                .align_y(Alignment::Center),
            ]
            .spacing(12)
            .padding(4),
        )
        .width(520)
        .padding(16)
        .style(|theme: &iced::Theme| container::Style {
            background: Some(Background::Color(theme.extended_palette().background.weak.color)),
            border: Border {
                color: theme.extended_palette().background.strong.color,
                width: 1.0,
                radius: 6.0.into(),
            },
            ..Default::default()
        });

        Some(
            container(card)
                .width(Length::Fill)
                .height(Length::Fill)
                .align_x(Alignment::Center)
                .align_y(Alignment::Center)
                .into(),
        )
    }

    /// The licence-and-credits overlay (View tab -> About -> Licence). `None` unless
    /// open.
    ///
    /// The **full GPL text is embedded**, not merely referenced: GPL-3.0 section 4
    /// asks that a copy of the licence accompany the program, and a shipped binary --
    /// an AppImage, a .dmg, an MSI -- travels without the repository's LICENSE file
    /// beside it. `include_str!` makes the copy part of the executable, so it is
    /// there wherever the program is.
    ///
    /// The credits below are the same facts as `assets/icons/CREDITS.md`, restated
    /// here because a user who has only the binary cannot read that file. They are
    /// obligations, not courtesies: the ribbon icons are GPL-3.0 from OpenCADStudio,
    /// and the application icon is the project's one CC BY-SA 4.0 exception.
    fn license_overlay(&self) -> Option<Element<'_, Message>> {
        if !self.show_license {
            return None;
        }
        const GPL: &str = include_str!("../../../LICENSE");

        let heading = |t: &'static str| text(t).size(14);
        let body = |t: String| text(t).size(12);

        let credits = column![
            heading("Third-party work"),
            body(
                "Ribbon icons — several are taken unmodified from OpenCADStudio \
                 (github.com/HakanSeven12/OpenCADStudio), © its contributors, GPL-3.0, \
                 reused here under the same licence. The CAM-specific icons are original."
                    .to_string()
            ),
            body(
                "Application icon — © Andreas Bertsatos, CC BY-SA 4.0 International. \
                 This is a deliberate exception to this program's GPL-3.0-only licence, \
                 so the mark may be used and attributed on Wikimedia. Its letterforms \
                 are outlines of DejaVu Sans Bold (Bitstream Vera Fonts Copyright)."
                    .to_string()
            ),
            body(
                "DXF/DWG import — acadrust, MPL-2.0. Geometry, toolpaths, posts and the \
                 simulator are original to this project."
                    .to_string()
            ),
        ]
        .spacing(6);

        let card = container(
            column![
                row![
                    iced::widget::svg(Icon::License.handle()).width(30).height(30),
                    text(format!("Open CAM Studio {}", crate::version_string())).size(20),
                ]
                .spacing(10)
                .align_y(Alignment::Center),
                body("A CAM application for CNC toolpath generation.".to_string()),
                body("Copyright © 2026 Andreas Bertsatos.".to_string()),
                heading("Licence"),
                body(
                    "This program is free software: you can redistribute it and/or modify \
                     it under the terms of the GNU General Public License, version 3 only, \
                     as published by the Free Software Foundation.\n\n\
                     This program is distributed in the hope that it will be useful, but \
                     WITHOUT ANY WARRANTY; without even the implied warranty of \
                     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU \
                     General Public License below for more details."
                        .to_string()
                ),
                credits,
                heading("GNU General Public License, version 3"),
                container(scrollable(text(GPL).size(11).font(iced::Font::MONOSPACE)).height(260))
                    .padding(8)
                    .style(|theme: &iced::Theme| container::Style {
                        background: Some(Background::Color(theme.palette().background)),
                        border: Border {
                            color: theme.extended_palette().background.strong.color,
                            width: 1.0,
                            radius: 3.0.into(),
                        },
                        ..Default::default()
                    }),
                row![
                    Space::new().width(Length::Fill),
                    // Explicit padding and a 1.0 line height, matching the other
                    // buttons in the app. Left to iced's defaults the label sits high
                    // in the button: the default line box for a 13pt text is taller
                    // than the glyphs, and uniform default padding centres the BOX,
                    // not the ink inside it.
                    button(
                        text("Close")
                            .size(13)
                            .line_height(iced::widget::text::LineHeight::Relative(1.0))
                            .align_y(Alignment::Center)
                    )
                    .padding(Padding::from([5.0, 14.0]))
                    .on_press(Message::CloseLicense),
                ]
                .align_y(Alignment::Center),
            ]
            .spacing(10)
            .padding(4),
        )
        .width(620)
        .padding(16)
        .style(|theme: &iced::Theme| container::Style {
            background: Some(Background::Color(theme.extended_palette().background.weak.color)),
            border: Border {
                color: theme.extended_palette().background.strong.color,
                width: 1.0,
                radius: 6.0.into(),
            },
            ..Default::default()
        });

        Some(
            container(card)
                .width(Length::Fill)
                .height(Length::Fill)
                .align_x(Alignment::Center)
                .align_y(Alignment::Center)
                .into(),
        )
    }

    /// The origins operation `id` could be moved to. Thin wrapper over
    /// [`origin_move_targets`], which holds the rule and can be tested without an app.
    fn origin_move_targets_for(&self, id: u32) -> Vec<u32> {
        let current = self
            .controller
            .document()
            .setup
            .operations
            .iter()
            .find(|op| op.id() == id)
            .map(|op| op.work_offset());
        origin_move_targets(
            &self.controller.origin_indices(),
            current,
            self.controller.base_origin_index(),
        )
    }

    /// How an origin reads wherever it is named — `"Origin 2 · G55"`, or plain
    /// `"Origin 7"` when the selected post has no word for that datum (the case the tree
    /// marks with a ⚠). The vocabulary comes from the post, never from a `match` here.
    ///
    /// Used by the tree header, the move-to menu and the status line, so a job cannot
    /// call the same origin two different things depending on where you read it.
    fn origin_menu_label(&self, index: u32) -> String {
        match self.controller.datum_label(index) {
            Some(word) => format!("Origin {index} · {word}"),
            None => format!("Origin {index}"),
        }
    }

    /// The operation right-click context menu (Delete / Duplicate), its top-left
    /// anchored exactly under the cursor. `None` unless a menu is open. Reuses the
    /// ribbon-popup overlay pattern: positioned in the top-level view stack over a
    /// click-off catcher.
    fn op_menu_overlay(&self) -> Option<Element<'_, Message>> {
        let id = self.open_op_menu?;
        // The rows are as wide as the widest label, and a "Move to Origin 2 · G55" row
        // is much wider than "Duplicate". Sized once for the whole menu so the items
        // stay a column rather than a ragged edge.
        let targets = self.origin_move_targets_for(id);
        let width = if targets.is_empty() { 130.0 } else { 190.0 };
        let item = move |icon: Icon, label: &str, msg: Message| {
            button(
                row![icon_svg(icon, 14.0), text(label.to_string()).size(13)]
                    .spacing(6)
                    .align_y(Alignment::Center),
            )
            .width(Length::Fixed(width))
            .padding(Padding::from([4.0, 8.0]))
            .on_press(msg)
            .style(|_theme, status| command_button_style(status))
        };
        let mut items = column![
            item(Icon::Delete, "Delete", Message::DeleteOp),
            item(Icon::Duplicate, "Duplicate", Message::DuplicateOp),
            item(Icon::Redo, "Reinitialize", Message::ReinitOp),
        ]
        .spacing(2);
        // Cross-group reassignment. One row per *other* origin — no submenu, because
        // the list is bounded by what the controls carry (six work offsets on the ISO
        // families) and a flat list is one click instead of two. Absent entirely on a
        // single-origin job, where there is nowhere to move to.
        for index in &targets {
            items = items.push(item(
                Icon::SetOrigin,
                &format!("Move to {}", self.origin_menu_label(*index)),
                Message::MoveOpToOrigin(*index),
            ));
        }
        let menu = container(items)
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

    /// The project-tree tool right-click menu — a single "Add to library" action,
    /// anchored under the cursor. Only opens for a project tool not already in the
    /// library (the un-numbered rows), so the action is always applicable.
    fn tool_menu_overlay(&self) -> Option<Element<'_, Message>> {
        let number = self.open_tool_menu?;
        let menu = container(
            button(
                row![
                    icon_svg(Icon::ImportLibrary, 14.0),
                    text("Add to library").size(13)
                ]
                .spacing(6)
                .align_y(Alignment::Center),
            )
            .width(Length::Fixed(150.0))
            .padding(Padding::from([4.0, 8.0]))
            .on_press(Message::AddToolToLibrary(number))
            .style(|_theme, status| command_button_style(status)),
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
                top: self.tool_menu_pos.y,
                left: self.tool_menu_pos.x,
                right: 0.0,
                bottom: 0.0,
            });
        Some(positioned.into())
    }

    /// The Library-pane right-click menu, anchored under the cursor. Shows a "Set number…"
    /// item that morphs into a small number input (Enter or Set commits; swaps on clash).
    fn lib_menu_overlay(&self) -> Option<Element<'_, Message>> {
        let menu = self.lib_menu.as_ref()?;
        let content: Element<'_, Message> = match &menu.input {
            None => button(text("Set number…").size(13))
                .width(Length::Fixed(140.0))
                .padding(Padding::from([4.0, 8.0]))
                .on_press(Message::LibMenuSetNumber)
                .style(|_theme, status| command_button_style(status))
                .into(),
            Some(buf) => row![
                text_input("T#", buf)
                    .on_input(Message::LibNumberInput)
                    .on_submit(Message::LibNumberCommit)
                    .width(Length::Fixed(64.0)),
                button(text("Set").size(13)).on_press(Message::LibNumberCommit),
            ]
            .spacing(6)
            .align_y(Alignment::Center)
            .into(),
        };
        let popup = container(content).padding(4).style(|_theme| container::Style {
            background: Some(Background::Color(palette::RIBBON_BG)),
            border: Border {
                color: palette::BORDER_DARK,
                width: 1.0,
                radius: 3.0.into(),
            },
            ..container::Style::default()
        });
        let positioned = container(popup)
            .width(Length::Fill)
            .height(Length::Fill)
            .align_x(iced::alignment::Horizontal::Left)
            .align_y(iced::alignment::Vertical::Top)
            .padding(Padding {
                top: self.lib_menu_pos.y,
                left: self.lib_menu_pos.x,
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
        let px = self.settings.snapping.pickbox_px;
        let half = px / 2.0;
        let square = container(Space::new())
            .width(Length::Fixed(px))
            .height(Length::Fixed(px))
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
        // `About` is NOT a tab: it opens the licence-and-credits popup instead of
        // switching the band below. It sits immediately after View, where a user
        // looks for it, rather than exiled to the window's right edge. A tab strip
        // does promise that every entry changes the ribbon under it, so a hairline
        // divider marks the boundary -- enough to say "different kind of thing"
        // without separating it from the group it belongs with.
        tabs = tabs
            .push(
                // The colour goes on the 1px inner container; the padding that spaces
                // it from its neighbours goes on the outer one. A container paints its
                // background across its padding as well, so styling the padded
                // container directly would draw a 15px filled block, not a hairline.
                container(
                    container(Space::new().width(1).height(16)).style(|_theme| {
                        container::Style {
                            background: Some(Background::Color(palette::BORDER_DARK)),
                            ..container::Style::default()
                        }
                    }),
                )
                .padding(Padding::from([0.0, 7.0])),
            )
            .push(
                button(text("Preferences").size(12))
                    .padding(Padding::from([5.0, 14.0]))
                    .on_press(Message::ShowPrefs)
                    .style(|_theme, status| tab_button_style(false, status)),
            )
            .push(
                button(text("About").size(12))
                    .padding(Padding::from([5.0, 14.0]))
                    .on_press(Message::ShowLicense)
                    .style(|_theme, status| tab_button_style(false, status)),
            );
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

    /// The icon-command groups for the active tab. Always non-empty — the pane
    /// toggles and the cube slider are appended separately by [`Self::ribbon_body`],
    /// since neither is an icon command the density solver can size.
    fn ribbon_specs(&self) -> Vec<GroupSpec> {
        let has_geo = self.controller.has_geometry();
        // A new op needs geometry to pick; its tool is drawn from the library, so the
        // library must have at least one tool (it seeds defaults, so it always does).
        let can_create = has_geo && !self.library.tools.is_empty();
        let begin = |kind: OpKind| can_create.then_some(Message::BeginOp(kind));
        match self.active_tab {
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
                // Roughly the order a part is actually made: skim the blank flat,
                // then cut its shape, then the features, then the decoration.
                commands: vec![
                    cmd(Icon::Face, "Face", begin(OpKind::Face)),
                    cmd(Icon::Profile, "Profile", begin(OpKind::Profile)),
                    cmd(Icon::Pocket, "Pocket", begin(OpKind::Pocket)),
                    cmd(Icon::Drill, "Drill", begin(OpKind::Drill)),
                    cmd(Icon::Thread, "Thread", begin(OpKind::Thread)),
                    cmd(Icon::Chamfer, "Chamfer", begin(OpKind::Chamfer)),
                    cmd(Icon::Engrave, "Engrave", begin(OpKind::Engrave)),
                    cmd(Icon::Carve, "Carve", begin(OpKind::Carve)),
                ],
            }],
            RibbonTab::Edit => vec![
                GroupSpec {
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
                },
            ],
            RibbonTab::Machinery => vec![GroupSpec {
                title: "Machines",
                commands: vec![
                    // Its own icon, not the tool's — borrowing one is what made "Add
                    // machine" read "Add a new tool to the library".
                    cmd_help(
                        Icon::Machine,
                        "New",
                        Some(Message::NewMachine),
                        "Add a machine to this installation, copied from the active one. \
                         Machines are local: a project records the machine it was built \
                         for but never sets yours, because an export is checked against \
                         the active machine's travel.",
                    ),
                    cmd_help(
                        Icon::Delete,
                        "Delete",
                        // Never the last: an export has to be checked against something.
                        (self.machines.machines.len() > 1).then_some(Message::DeleteMachine),
                        "Remove the active machine. The last one cannot be removed — an \
                         export has to be checked against something.",
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
                    // Bulk renumber (guarded by a confirm dialog).
                    cmd(Icon::Renumber, "Renumber", Some(Message::RenumberLibrary)),
                    // ("Add to library" lives on the project-tree tool right-click menu,
                    // shown only for a project tool that isn't already in the library.)
                ],
            },
            GroupSpec {
                title: "Library file",
                commands: vec![
                    cmd(Icon::ImportLibrary, "Import", Some(Message::ImportLibrary)),
                    cmd(Icon::ExportLibrary, "Export", Some(Message::ExportLibrary)),
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
                    toggle_cmd(
                        Icon::Machine,
                        "Envelope",
                        self.show_envelope,
                        Message::ToggleEnvelope,
                    ),
                    toggle_cmd(
                        Icon::Info,
                        "Tips",
                        self.tooltips,
                        Message::ToggleTooltips,
                    ),
                ],
            }],
        }
    }

    /// The densities the active tab's groups should render at, given the window
    /// width.
    fn ribbon_densities(&self, specs: &[GroupSpec]) -> Vec<Density> {
        let counts: Vec<usize> = specs.iter().map(|g| g.commands.len()).collect();
        let available = (self.window.width - RIBBON_CHROME).max(0.0);
        solve_densities(&counts, available)
    }

    /// The groups shown for the active ribbon tab, each at its solved density.
    fn ribbon_body(&self) -> Element<'_, Message> {
        let specs = self.ribbon_specs();
        let densities = self.ribbon_densities(&specs);
        let mut band = row![].spacing(GROUP_GAP).align_y(Alignment::Start);
        for (i, (spec, &density)) in specs.iter().zip(&densities).enumerate() {
            band = band.push(render_group(spec, density, i, self.tooltips));
        }
        // The View tab gets a live orientation-cube size control (a slider has no
        // place in the icon-command band, so it is appended as its own group).
        // The View tab gets the cube-size slider, then the pane-visibility toggles —
        // both sit outside the density solver, which sizes icon-command groups only.
        if self.active_tab == RibbonTab::View {
            band = band.push(self.cube_size_group());
            band = band.push(self.panes_group());
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
            .on_release(Message::SettingsSettled)
            .width(Length::Fixed(140.0))
            .into()
        } else {
            // Keep the footprint stable when the cube is off: an inert placeholder.
            container(
                Space::new()
                    .width(Length::Fixed(140.0))
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
    /// The pane-visibility toggles, appended to the **View** tab as a compact block
    /// captioned "Panes". Laid out in two balanced columns: the ribbon band is only
    /// ~70 px tall, so a single stacked column of four would overflow it.
    ///
    /// The Viewport is always visible and has no toggle. Labels come from
    /// [`Pane::ribbon_label`], so the tool library reads "Tools" here while its own
    /// title bar keeps the full name.
    fn panes_group(&self) -> Element<'_, Message> {
        let toggle = |pane: Pane| -> Element<'_, Message> {
            let shown = self.pane_handle(pane).is_some();
            row![
                checkbox(shown)
                    .size(15)
                    .on_toggle(move |v| Message::SetPaneVisible(pane, v)),
                text(pane.ribbon_label()).size(12),
            ]
            .spacing(5)
            .align_y(Alignment::Center)
            .into()
        };
        // Derived from ALL_PANES rather than listed, so a pane added later appears
        // here automatically instead of silently going missing.
        let toggles: Vec<Pane> = ALL_PANES
            .into_iter()
            .filter(|p| *p != Pane::Viewport)
            .collect();
        let split = toggles.len().div_ceil(2);
        let mut grid = row![].spacing(12).align_y(Alignment::Start);
        for chunk in toggles.chunks(split) {
            let mut col = column![].spacing(4);
            for &pane in chunk {
                col = col.push(toggle(pane));
            }
            grid = grid.push(col);
        }
        ribbon_group("Panes", grid)
    }

    /// The floating panel for an open collapsed-group popup, positioned under its
    /// button. `None` unless a group is open *and* actually collapsed at the
    /// current width. Its x-offset is the exact sum of the preceding groups' drawn
    /// widths — the analytic layout doubles as popup positioning.
    fn ribbon_popup(&self) -> Option<Element<'_, Message>> {
        let index = self.open_group?;
        let specs = self.ribbon_specs();
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
            commands = commands.push(render_command(command, false, self.tooltips));
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
            Pane::Library => self.library_pane(),
            // In Tooling mode (or with a tool selected) the viewport shows the tool's 2D
            // cross-section instead of the 3D backplot (Phase 5).
            Pane::Viewport => match self.preview_tool() {
                Some(tool) => container(
                    canvas(ToolCanvas::new(&tool))
                        .width(Length::Fill)
                        .height(Length::Fill),
                )
                .width(Length::Fill)
                .height(Length::Fill)
                .into(),
                None => container(
                    shader(Viewport::new(
                        &self.controller,
                        self.show_stock,
                        self.view,
                        self.show_gizmo,
                        self.gizmo_size,
                        &self.settings,
                        &self.focus_ops.iter().copied().collect::<Vec<_>>(),
                        self.snap_hover.map(|h| (h, self.snap_aperture)),
                        self.hover_loop,
                        self.in_origin_pick(),
                        self.show_origin,
                        self.origin_first,
                        self.show_envelope.then(|| {
                            let (x, y, z) = self.controller.machine().envelope.extent();
                            [x as f32, y as f32, z as f32]
                        }),
                    ))
                    .width(Length::Fill)
                    .height(Length::Fill),
                )
                .width(Length::Fill)
                .height(Length::Fill)
                .into(),
            },
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
    /// A workpiece-origin group header in the operation list: `Origin <n> · <datum>`
    /// with its position, a freeze checkbox (drops the group's ops from the run/
    /// viewport), and a delete button (extras only — the base origin is not
    /// removable). Clicking the row makes the origin active so new ops join it and the
    /// inspector edits it.
    ///
    /// The index is the identity — it is what each operation stores and what the
    /// reorder arrows move — and the datum word beside it is the *selected post's*
    /// name for it (`G55` on Fanuc, `H2` on Okuma), so switching post relabels every
    /// row. A post with no word for this datum (a seventh fixture on an ISO control,
    /// which carries only `G54`-`G59`) is marked ⚠ here rather than left to fail at
    /// export, after the job has been built.
    fn origin_header_row(&self, index: u32) -> Element<'_, Message> {
        let pos = self.controller.origin_position(index);
        let disabled = self.controller.is_origin_disabled(index);
        let active = self.controller.selection() == Selection::Origin
            && self.controller.active_origin() == index;
        let freeze = checkbox(!disabled)
            .size(15)
            .on_toggle(move |on| Message::ToggleOriginDisabled(index, !on));
        // The datum word joins the index when the post has one; when it does not, the
        // ⚠ below says so rather than the row saying it twice.
        let datum = self.controller.datum_label(index);
        let named = self.origin_menu_label(index);
        let label = text(format!(
            "{named} — X{} Y{} Z{}",
            fmt_num(pos[0]),
            fmt_num(pos[1]),
            fmt_num(pos[2])
        ))
        .size(13)
        .width(Length::Fill)
        .color(if active {
            palette::LABEL_COLOR
        } else {
            palette::GROUP_LABEL
        });
        let mut controls = row![freeze, label].spacing(6).align_y(Alignment::Center);
        if datum.is_none() {
            // Glyph, not colour — the meaning has to survive colour-vision deficiency.
            controls = controls.push(
                text(format!(
                    "⚠ no {} work offset",
                    self.controller.post_kind()
                ))
                .size(11)
                .color(palette::WARN),
            );
        }
        if index != self.controller.base_origin_index() {
            controls = controls.push(
                button(text("✕").size(12)).on_press(Message::DeleteOrigin(index)),
            );
        }
        let inner = container(controls.padding(Padding::from([2.0, 6.0])))
            .width(Length::Fill)
            .style(move |_theme| container::Style {
                background: active.then_some(Background::Color(palette::SELECT_BG)),
                border: Border {
                    radius: 3.0.into(),
                    ..Border::default()
                },
                ..container::Style::default()
            });
        mouse_area(inner)
            .on_press(Message::SelectOrigin(index))
            .into()
    }

    fn project_tree(&self) -> Element<'_, Message> {
        let setup = &self.controller.document().setup;
        let sel = self.controller.selection();

        // Workpiece origins live as group headers in the operation list below (H1
        // above the first op, added origins each heading their own group), not as a
        // node here — an origin *is* the datum a group of operations runs under.
        let mut list = column![
            select_row(
                format!("Setup — {}", setup.name),
                sel == Selection::Setup,
                Message::Select(Selection::Setup),
            ),
            select_row(
                "Stock".to_string(),
                sel == Selection::Stock,
                Message::Select(Selection::Stock),
            ),
        ]
        .spacing(2);

        // Tools in use. A tool that matches the shop library shows its **shop number**
        // (`T4 …`); one that isn't in the library shows **no number** — the simple,
        // clear signal that it's project-local. Right-clicking an un-numbered tool
        // offers "Add to library" (which registers it and gives it a number).
        list = list.push(tree_header("Tools (in use)"));
        let used = self.controller.used_tools();
        if used.is_empty() {
            list = list.push(tree_note("none yet — set up an operation"));
        }
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
                text(label).size(13).color(if in_library {
                    palette::LABEL_COLOR
                } else {
                    palette::GROUP_LABEL
                }),
            )
            .padding(Padding::from([3.0, 6.0]));
            if in_library {
                list = list.push(row);
            } else {
                // Only un-numbered (not-in-library) tools carry the right-click menu.
                list = list.push(mouse_area(row).on_right_press(Message::ToolMenu(t.number)));
            }
        }

        // Exact-duplicate operations (identical bar their id, both included),
        // computed once: `twins[id]` is the ids of the other ops it duplicates.
        let dup_groups = self.controller.duplicate_operation_groups();
        // Operations whose last run produced an error, so the tree can say *which*
        // one failed instead of leaving the status bar's count to be hunted down.
        // Errors do not delete the operation — it stays editable so the offending
        // field (or tool) can be corrected in place — and export is blocked while any
        // remain, so a marked row can never reach the machine.
        let failed: BTreeMap<u32, String> = self
            .controller
            .outcome()
            .map(|o| {
                let mut m: BTreeMap<u32, String> = BTreeMap::new();
                for d in o
                    .diagnostics
                    .iter()
                    .filter(|d| d.severity == Severity::Error)
                {
                    if let Some(id) = d.op {
                        m.entry(id).or_insert_with(|| d.message.clone());
                    }
                }
                m
            })
            .unwrap_or_default();
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
        // Operations are grouped under their origin (`H<n>`) header, in origin order.
        // An op whose `work_offset` names no current origin falls under the base (H1).
        let origin_indices = self.controller.origin_indices();
        let base_index = self.controller.base_origin_index();
        let group_of = |op: &Operation| {
            let wo = op.work_offset();
            if origin_indices.contains(&wo) {
                wo
            } else {
                base_index
            }
        };
        for &oidx in &origin_indices {
            list = list.push(self.origin_header_row(oidx));
            let group: Vec<(usize, &Operation)> = setup
                .operations
                .iter()
                .enumerate()
                .filter(|(_, op)| group_of(op) == oidx)
                .collect();
            if group.is_empty() {
                list = list.push(tree_note("    (no operations)"));
            }
            let glen = group.len();
            for (gi, (_flat, op)) in group.iter().copied().enumerate() {
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
            // Reorder within the origin group: enabled only when there's a sibling in
            // that direction under the same origin.
            let up = button(text("↑").size(12))
                .on_press_maybe((gi > 0).then_some(Message::MoveOp(id, true)));
            let down = button(text("↓").size(12))
                .on_press_maybe((gi + 1 < glen).then_some(Message::MoveOp(id, false)));
            // On an exact duplicate, mark it ⚠ and name its twin(s) by id — both
            // would post the same toolpath.
            let mut controls = row![include, name].spacing(4).align_y(Alignment::Center);
            // A multi-tool operation names both, in cutting order, so the tree tells
            // the truth about what the spindle does. Muted, so it reads as information
            // rather than as the warnings beside it. Single-tool ops show nothing.
            let tools = op.tools();
            if tools.len() > 1 {
                let list = tools
                    .iter()
                    .map(|t| format!("T{t}"))
                    .collect::<Vec<_>>()
                    .join(" + ");
                controls = controls.push(text(list).size(11).color(palette::GROUP_LABEL));
            }
            // An operation that failed its last run: ⚠ in the error colour, with the
            // message on hover. Glyph + colour together, so it survives colour-vision
            // deficiency.
            if let Some(msg) = failed.get(&id) {
                let mark = text("⚠").size(12).color(palette::ERROR);
                let marker: Element<'_, Message> = if self.tooltips {
                    tooltip(
                        mark,
                        container(text(msg.clone()).size(12))
                            .padding(8)
                            .max_width(360.0)
                            .style(container::rounded_box),
                        tooltip::Position::Left,
                    )
                    .into()
                } else {
                    mark.into()
                };
                controls = controls.push(marker);
            }
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
        }
        // A control to add another origin (a reorientation of the part).
        list = list.push(
            button(text("+ New origin").size(13)).on_press(Message::AddOrigin),
        );

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
            OpKind::Engrave => "Engrave",
            OpKind::Carve => "Carve",
        };
        // The tool is chosen in two steps — **family**, then a tool within it — so a
        // library of hundreds does not have to be scrolled for every operation. The
        // families offered are bounded by the operation (see `families_for`).
        let families = families_for(pending.kind);
        let family = self.wizard_family.filter(|f| families.contains(f));
        let family_picker = pick_list(families.to_vec(), family, Message::SetPendingFamily)
            .placeholder("Choose a family")
            .text_size(13)
            .width(Length::Fill);

        // Tools of the chosen family only. Nothing is listed until a family is picked.
        let tools: Vec<ToolChoice> = match family {
            Some(f) => self
                .library
                .tools
                .iter()
                .enumerate()
                .filter(|(_, t)| ToolKindPick::of(t.kind) == f)
                .map(|(index, t)| ToolChoice {
                    index,
                    number: t.number,
                    diameter: t.diameter,
                    kind: t.kind,
                })
                .collect(),
            None => Vec::new(),
        };
        // The current selection is the library entry matching the pending op's
        // embedded tool (picking embeds a copy into the setup — see `use_tool`).
        let embedded = pending.tool.and_then(|n| {
            self.controller
                .document()
                .setup
                .tools
                .iter()
                .find(|t| t.number == n)
                .copied()
        });
        let selected = embedded
            .and_then(|e| {
                self.library.tools.iter().position(|l| {
                    l.diameter == e.diameter
                        && l.length == e.length
                        && l.flutes == e.flutes
                        && l.kind == e.kind
                })
            })
            .and_then(|i| tools.iter().find(|c| c.index == i).copied());
        let tool_picker = pick_list(tools.clone(), selected, |c| {
            Message::SetPendingLibraryTool(c.index)
        })
        .placeholder(if family.is_some() {
            "Choose a tool"
        } else {
            "Choose a family first"
        })
        .text_size(13)
        .width(Length::Fill);

        let verb = if pending.replacing.is_some() {
            "Reinitialize"
        } else {
            "New"
        };
        let mut col = column![
            text(format!("{verb} {kind} operation")).size(15),
            text("Tool family").size(12),
            family_picker,
            text("Tool").size(12),
            tool_picker,
        ]
        .spacing(6)
        .padding(8);

        // A family with nothing in it is a dead end now that the wizard has no
        // "New tool" shortcut — say where to go rather than leaving an empty list.
        if let Some(f) = family {
            if tools.is_empty() {
                col = col.push(
                    text(format!(
                        "No {f} in the library — add one in the Tooling tab."
                    ))
                    .size(12)
                    .color(palette::WARN),
                );
            }
        }

        // Cutting data for the new operation, seeded from the tool's nominals and
        // editable here (or later in the operation's inspector). Shown once a tool is
        // chosen — before then there is nothing to seed from.
        if pending.tool.is_some() {
            col = col.push(text("Cutting data").size(12));
            for f in [Field::SpindleRpm, Field::Feed, Field::PlungeFeed] {
                let value = self.fields.get(&f).cloned().unwrap_or_default();
                col = col.push(field_row_labeled(
                    f,
                    self.field_label(f),
                    self.field_help(f),
                    &value,
                    self.tooltips,
                    self.field_invalid(f),
                ));
            }
        }

        // Object snaps govern where the start/lead-in lands; show them for the
        // kinds that carry a start, while still awaiting that (boundary) pick.
        if op_uses_snaps(pending.kind) && pending.boundary.is_none() {
            col = col.push(self.snap_toolbar());
        }

        // Progress, then the one commit point. Tool and geometry may be chosen in
        // either order; Confirm stays disabled until both are settled.
        col = col.push(
            text(match (pending.tool.is_some(), pending.boundary.is_some()) {
                (false, false) => "Choose a tool and click the geometry in the viewport.".to_string(),
                (true, false) => "Now click the geometry in the viewport.".to_string(),
                (false, true) => "Geometry selected — now choose a tool.".to_string(),
                (true, true) if op_takes_islands(pending.kind) => format!(
                    "Click enclosed areas to exclude ({} selected), then Confirm.",
                    pending.islands.len()
                ),
                (true, true) => "Ready — Confirm to create the operation.".to_string(),
            })
            .size(12),
        );
        col = col.push(
            row![
                button(text("Confirm").size(13))
                    .on_press_maybe(self.controller.pending_ready().then_some(Message::ConfirmOp)),
                button(text("Cancel").size(13)).on_press(Message::CancelOp),
            ]
            .spacing(8),
        );
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

    /// The **Tool Library pane** — a selectable list of every library tool, in one of
    /// two internal tabs: **Serial** (by number) or **Family** (grouped by kind, sorted
    /// by diameter). Substitutes for the Project pane on the Tooling tab; the selected
    /// tool's *fields* live in the Inspector (kept clean). New / Delete are on the
    /// Tooling ribbon.
    fn library_pane(&self) -> Element<'_, Message> {
        let tab = |label: &str, view: LibraryView| {
            let active = self.library_view == view;
            button(text(label.to_string()).size(12))
                .padding(Padding::from([3.0, 12.0]))
                .on_press(Message::SetLibraryView(view))
                .style(move |_theme, status| snap_toggle_style(active, status))
        };
        let mut list = column![row![
            tab("Ordered", LibraryView::Ordered),
            tab("Grouped", LibraryView::Grouped),
        ]
        .spacing(6)]
        .spacing(4)
        .padding(6);

        if self.library.tools.is_empty() {
            list = list.push(tree_note("empty — add a tool with New"));
        }

        // (index, tool) pairs so a selection keeps the real library index whatever the
        // display order.
        let mut items: Vec<(usize, &cam_model::Tool)> =
            self.library.tools.iter().enumerate().collect();
        match self.library_view {
            LibraryView::Ordered => {
                // Number **first** in the library (the number is the tool's identity
                // here) — the reverse of the Project pane, e.g. "T1: ⌀6 End mill (2 flutes)".
                items.sort_by_key(|(_, t)| t.number);
                for (i, t) in items {
                    let base = format!("T{}: ⌀{} {}", t.number, fmt_num(t.diameter), t.kind);
                    let label = match library_extra(t) {
                        Some(extra) => format!("{base} ({extra})"),
                        None => base,
                    };
                    list = list.push(
                        mouse_area(select_row(label, i == self.lib_sel, Message::SelectLibraryTool(i)))
                            .on_right_press(Message::LibToolMenu(i)),
                    );
                }
            }
            LibraryView::Grouped => {
                items.sort_by(|(_, a), (_, b)| {
                    kind_order(a.kind)
                        .cmp(&kind_order(b.kind))
                        .then(a.diameter.total_cmp(&b.diameter))
                        .then(a.number.cmp(&b.number))
                });
                let mut current: Option<String> = None;
                for (i, t) in items {
                    let fam = t.kind.to_string();
                    if current.as_deref() != Some(fam.as_str()) {
                        // Category headers are plural ("Square End Mills", "Drills", …).
                        list = list.push(tree_header(&format!("{fam}s")));
                        current = Some(fam);
                    }
                    // Kind is the group header, so the row omits it: "T2: ⌀6 (4 flutes)".
                    let base = format!("T{}: ⌀{}", t.number, fmt_num(t.diameter));
                    let label = match library_extra(t) {
                        Some(extra) => format!("{base} ({extra})"),
                        None => base,
                    };
                    list = list.push(
                        mouse_area(select_row(label, i == self.lib_sel, Message::SelectLibraryTool(i)))
                            .on_right_press(Message::LibToolMenu(i)),
                    );
                }
            }
        }

        scrollable(list)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }

    /// The Tooling-tab tool-library editor in the **Inspector**: just the selected
    /// tool's editable fields (the list lives in the Tool Library pane). New / Delete
    /// live on the Tooling ribbon tab.
    fn library_editor(&self) -> Element<'_, Message> {
        // Read the header from the working copy so ⌀/kind track the live edits, matching
        // the viewport preview (the committed list/pane updates on Apply).
        let name = self
            .preview_tool()
            .map(|t| format!("T{} — ⌀{} {}", t.number, fmt_num(t.diameter), t.kind))
            .unwrap_or_else(|| "Tool Library".to_string());
        let mut list = column![text(name).size(15)].spacing(8).padding(8);

        for field in self.inspector_fields() {
            let value = self.fields.get(&field).cloned().unwrap_or_default();
            list = list.push(field_row_labeled(field, self.field_label(field), self.field_help(field), &value, self.tooltips, self.field_invalid(field)));
        }
        if let Some(t) = self.preview_tool() {
            // Read from the working copy so the pickers (cutting direction, Type, thread
            // form) track live edits, matching the fields and viewport.
            // Cutting direction (a picker, not a numeric field) applies to the end-mill
            // family only; "Straight flute" is offered only for a Square End Mill.
            let is_end_mill = matches!(
                t.kind,
                ToolKind::EndMill | ToolKind::BallMill | ToolKind::BullNose { .. }
            );
            if is_end_mill {
                let dir_opts = if matches!(t.kind, ToolKind::EndMill) {
                    vec![CutDir::Down, CutDir::Up, CutDir::Straight]
                } else {
                    vec![CutDir::Down, CutDir::Up]
                };
                list = list.push(
                    row![
                        help_wrap(
                            text("Cutting direction").width(Length::Fixed(112.0)).size(13),
                            "Down-cut vs up-cut (helix direction); a physical property of the \
                             tool, like the flute count. Square end mills also allow a straight \
                             (axial) flute.",
                            self.tooltips,
                        ),
                        pick_list(
                            dir_opts,
                            Some(t.cutting_direction),
                            Message::ToolCuttingDirChanged
                        )
                        .text_size(13)
                        .width(Length::Fixed(113.0)),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            }
            // Thread form (single-point vs full-form) — a thread-mill-only toggle.
            if let ToolKind::ThreadMill { pitch } = t.kind {
                let form = if pitch.is_some() {
                    ThreadForm::FullForm
                } else {
                    ThreadForm::SinglePoint
                };
                list = list.push(
                    row![
                        help_wrap(
                            text("Thread form").width(Length::Fixed(112.0)).size(13),
                            "Single-point mills carry one tooth and cut any pitch by their \
                             helical lead; full-form mills carry a stack of teeth at a fixed \
                             ground pitch.",
                            self.tooltips,
                        ),
                        pick_list(&ThreadForm::ALL[..], Some(form), Message::ThreadFormChanged)
                            .text_size(13)
                            .width(Length::Fixed(113.0)),
                    ]
                    .spacing(8)
                    .align_y(Alignment::Center),
                );
            }
            // Short label + fixed-width picker whose right edge lands on the value-box
            // right edge above (135 label + 8 gap + 90 box = 233), so the boxes align.
            list = list.push(
                row![
                    help_wrap(
                        text("Type").width(Length::Fixed(48.0)).size(13),
                        help::TOOL_TYPE,
                        self.tooltips,
                    ),
                    pick_list(&ToolKindPick::ALL[..], Some(ToolKindPick::of(t.kind)), |p| {
                        Message::ToolKindChanged(p.to_kind())
                    })
                    .text_size(13)
                    .width(Length::Fixed(177.0)),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
            // Apply is live only when an edit is pending and valid — the same rule in
            // both inspectors, so the button means one thing everywhere.
            list = list.push(
                button("Apply").on_press_maybe((!self.any_field_invalid() && self.inspector_dirty()).then_some(Message::Apply)),
            );
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
            Selection::Origin => {
                let index = self.controller.active_origin();
                match self.controller.datum_label(index) {
                    Some(d) => format!("Workpiece origin {index} · {d}"),
                    None => format!("Workpiece origin {index} · no work offset"),
                }
            }
            Selection::Stock => "Stock".to_string(),
            Selection::Machine => "Machine".to_string(),
            Selection::Tool(i) => format!("Tool {}", i + 1),
            Selection::Operation(id) => match self.controller.operation(id) {
                Some(op) => format!("Operation {id} — {}", op_kind(op)),
                None => "Operation".to_string(),
            },
        };

        let mut list = column![text(heading).size(15)].spacing(8).padding(8);

        // An operation's tool is fixed once created: it is chosen in the creation
        // wizard alongside the geometry, and changing it afterwards is done by
        // right-click -> Reinitialize in the Project pane, which re-runs the whole
        // pick. No tool control belongs in the inspector.
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

        if let Selection::Machine = self.controller.selection() {
            // Which machine is active — above its own fields, because everything below
            // edits *this* one. The library is local: a machine is a property of your
            // shop, and a project file records the one it was built for as provenance
            // only (it can never set yours, or it could disarm the travel check).
            list = list.push(
                row![
                    label_help("Active", help::ACTIVE_MACHINE, self.tooltips),
                    pick_list(
                        self.machines.names(),
                        Some(self.active_machine.clone()),
                        Message::ActiveMachineChanged,
                    )
                    .text_size(12)
                    .width(INSPECTOR_PICKER_W),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
            // Machine name (a free-text tag), above the travel fields. Committed on
            // change so multiple machines can be told apart later.
            list = list.push(
                row![
                    label_help("Name", help::MACHINE_NAME, self.tooltips),
                    text_input(
                        "machine",
                        self.machine_name_edit
                            .as_deref()
                            .unwrap_or(&self.controller.machine().name),
                    )
                        .on_input(Message::MachineNameChanged)
                        .width(Length::Fixed(120.0)),
                ]
                .spacing(8)
                .align_y(Alignment::Center),
            );
            // The post/controller dialect used on export.
            list = list.push(profile_picker_w(
                "Post",
                help::POST,
                self.tooltips,
                self.machine_post_edit.unwrap_or(self.controller.post_kind()),
                &PostKind::ALL[..],
                Message::PostKindChanged,
                INSPECTOR_PICKER_W,
            ));
            list = list.push(
                text("Dry-run / air-cut generated code on your control before cutting.")
                    .size(11)
                    .color(palette::GROUP_LABEL),
            );
        }

        let ordered = self.inspector_fields();
        if ordered.is_empty() {
            list = list.push(text("Nothing to edit here yet.").size(12));
        }
        for field in ordered {
            // Clearing-pass fields render in their own section (below), not mixed in here.
            if is_clear_field(field) {
                continue;
            }
            let value = self.fields.get(&field).cloned().unwrap_or_default();
            list = list.push(field_row_labeled(field, self.field_label(field), self.field_help(field), &value, self.tooltips, self.field_invalid(field)));
        }
        // A non-base origin is a reorientation: note the operator stop so the part can be
        // re-fixtured. The `H index` / position fields render in the generic loop above.
        if let Selection::Origin = self.controller.selection() {
            if self.controller.active_origin() != self.controller.base_origin_index() {
                list = list.push(
                    text(
                        "Teach this G15 H<n> on the control to the point set here. A stop \
                         (M00) precedes this group so you can re-fixture the part.",
                    )
                    .size(11)
                    .color(palette::GROUP_LABEL),
                );
            }
        }
        // The tool geometry class is an enum, so it gets a picker (committed
        // immediately) rather than a text field.
        if let Selection::Tool(i) = self.controller.selection() {
            if let Some(tool) = self.controller.document().setup.tools.get(i) {
                list = list.push(
                    row![
                        help_wrap(
                            text("Type").width(Length::Fixed(48.0)).size(13),
                            help::TOOL_TYPE,
                            self.tooltips,
                        ),
                        pick_list(
                            &ToolKindPick::ALL[..],
                            Some(ToolKindPick::of(tool.kind)),
                            |p| Message::ToolKindChanged(p.to_kind())
                        )
                        .text_size(13)
                        .width(Length::Fixed(177.0)),
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
                        help::SIDE,
                        self.tooltips,
                        p.side,
                        &Side::ALL[..],
                        Message::SideChanged,
                    ));
                    list = list.push(profile_picker(
                        "Lead-in",
                        help::LEAD_IN,
                        self.tooltips,
                        LeadKind::of(p.lead_in),
                        &LeadKind::ALL[..],
                        Message::LeadInKindChanged,
                    ));
                    list = list.push(profile_picker(
                        "Lead-out",
                        help::LEAD_OUT,
                        self.tooltips,
                        LeadKind::of(p.lead_out),
                        &LeadKind::ALL[..],
                        Message::LeadOutKindChanged,
                    ));
                    list = list.push(profile_picker(
                        "Plunge",
                        help::PLUNGE,
                        self.tooltips,
                        PlungeKind::of(p.plunge),
                        &PlungeKind::ALL[..],
                        Message::PlungeKindChanged,
                    ));
                    // Climb/conventional applies to outside-roughing clearing only.
                    if p.side == Side::Outside {
                        list = list.push(
                            row![
                                label_help("Climb", help::CLIMB, self.tooltips),
                                checkbox(p.clearing.climb)
                                    .size(15)
                                    .on_toggle(Message::ClearingClimbToggled),
                            ]
                            .spacing(8)
                            .align_y(Alignment::Center),
                        );
                    }
                }
                Some(Operation::Pocket(p)) => {
                    list = list.push(profile_picker(
                        "Plunge",
                        help::PLUNGE,
                        self.tooltips,
                        PlungeKind::of(p.clear.plunge),
                        &PlungeKind::ALL[..],
                        Message::PlungeKindChanged,
                    ));
                    list = list.push(
                        row![
                            label_help("Climb", help::CLIMB, self.tooltips),
                            checkbox(p.clear.clearing.climb)
                                .size(15)
                                .on_toggle(Message::ClearingClimbToggled),
                        ]
                        .spacing(8)
                        .align_y(Alignment::Center),
                    );
                    list = list.push(profile_picker(
                        "Wall lead-in",
                        help::LEAD_IN,
                        self.tooltips,
                        LeadKind::of(p.clear.lead_in),
                        &LeadKind::ALL[..],
                        Message::LeadInKindChanged,
                    ));
                    list = list.push(profile_picker(
                        "Wall lead-out",
                        help::LEAD_OUT,
                        self.tooltips,
                        LeadKind::of(p.clear.lead_out),
                        &LeadKind::ALL[..],
                        Message::LeadOutKindChanged,
                    ));
                }
                Some(Operation::Face(f)) => {
                    list = list.push(profile_picker(
                        "Direction",
                        help::FACE_DIRECTION,
                        self.tooltips,
                        f.direction,
                        &Axis::ALL[..],
                        Message::FaceDirectionChanged,
                    ));
                }
                Some(Operation::Chamfer(c)) => {
                    list = list.push(profile_picker(
                        "Side",
                        help::SIDE,
                        self.tooltips,
                        c.side,
                        &Side::ALL[..],
                        Message::SideChanged,
                    ));
                    list = list.push(profile_picker(
                        "Lead-in",
                        help::LEAD_IN,
                        self.tooltips,
                        LeadKind::of(c.lead_in),
                        &LeadKind::ALL[..],
                        Message::LeadInKindChanged,
                    ));
                    list = list.push(profile_picker(
                        "Lead-out",
                        help::LEAD_OUT,
                        self.tooltips,
                        LeadKind::of(c.lead_out),
                        &LeadKind::ALL[..],
                        Message::LeadOutKindChanged,
                    ));
                    // Gradual stepping (equal material per pass) — only meaningful
                    // when the chamfer is cut in multiple passes.
                    list = list.push(
                        row![
                            label_help("Gradual", help::CHAMFER_GRADUAL, self.tooltips),
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
                        help::BORE,
                        self.tooltips,
                        Bore::of(t.internal),
                        &Bore::ALL[..],
                        |b| Message::ThreadInternalChanged(b == Bore::Internal),
                    ));
                    list = list.push(profile_picker_w(
                        "Hand",
                        help::HAND,
                        self.tooltips,
                        t.hand,
                        &Hand::ALL[..],
                        Message::ThreadHandChanged,
                        INSPECTOR_PICKER_W,
                    ));
                    list = list.push(profile_picker_w(
                        "Cut",
                        help::THREAD_CUT,
                        self.tooltips,
                        CutStyle::of(t.climb),
                        &CutStyle::ALL[..],
                        |c| Message::ThreadClimbChanged(c == CutStyle::Climb),
                        INSPECTOR_PICKER_W,
                    ));
                    // Equal material per infeed pass — only meaningful with more than one.
                    list = list.push(
                        row![
                            label_help("Gradual", help::THREAD_GRADUAL, self.tooltips),
                            checkbox(t.gradual)
                                .size(15)
                                .on_toggle(Message::ThreadGradualToggled),
                        ]
                        .spacing(8)
                        .align_y(Alignment::Center),
                    );
                }
                Some(Operation::Carve(c)) => {
                    // The V-bit's own entry. The clearing pass has its own picker far
                    // below, under the tool it belongs to — they are different tools
                    // cutting different things, and only one of them can plunge freely.
                    list = list.push(profile_picker(
                        "Plunge",
                        help::CARVE_PLUNGE,
                        self.tooltips,
                        PlungeKind::of(c.plunge),
                        &PlungeKind::ALL[..],
                        Message::PlungeKindChanged,
                    ));
                    list = list.push(
                        row![
                            label_help("Stay down", help::CARVE_STAY_DOWN, self.tooltips),
                            checkbox(c.stay_down)
                                .size(15)
                                .on_toggle(Message::CarveStayDownToggled),
                        ]
                        .spacing(8)
                        .align_y(Alignment::Center),
                    );

                    // What the shape itself allows, computed live from region + tool +
                    // depth — no run needed. This is the line that tells the operator
                    // whether to deepen, accept, or add the second tool.
                    let shape = self
                        .controller
                        .document()
                        .setup
                        .tools
                        .iter()
                        .find(|t| t.number == c.tool)
                        .and_then(|t| cam_toolpath::carve_shape(c, t));
                    if let Some(sh) = shape {
                        let reach = if sh.full_depth > sh.tool_max_depth + 1e-9 {
                            format!(", past tool {}'s {:.2} mm limit", c.tool, sh.tool_max_depth)
                        } else {
                            String::new()
                        };
                        let line = if sh.flat_areas == 0 {
                            format!(
                                "Carves out at {:.2} mm{reach} — no flat areas.",
                                sh.full_depth
                            )
                        } else if sh.flat_areas == 1 {
                            format!(
                                "Full depth for this shape is {:.2} mm{reach}; this cap \
                                 leaves 1 flat area.",
                                sh.full_depth
                            )
                        } else {
                            format!(
                                "Full depth for this shape is {:.2} mm{reach}; this cap \
                                 leaves {} flat areas.",
                                sh.full_depth, sh.flat_areas
                            )
                        };
                        list = list.push(text(line).size(12).color(palette::GROUP_LABEL));

                        // The clearing tool is offered only when there is something for
                        // it to clear — otherwise it buys a tool change and nothing else.
                        if sh.flat_areas > 0 {
                            // Everything below this line belongs to the *other* tool.
                            // Two tools in one operation is unusual enough that the
                            // inspector has to say where one ends and the next begins.
                            list = list.push(iced::widget::rule::horizontal(1));
                            list = list.push(
                                text("Clearing pass (end mill)")
                                    .size(12)
                                    .color(palette::GROUP_LABEL),
                            );
                            list = list.push(
                                row![
                                    label_help(
                                        "Clear flat areas",
                                        help::CARVE_CLEAR,
                                        self.tooltips
                                    ),
                                    checkbox(c.clear.is_some())
                                        .size(15)
                                        .on_toggle(Message::CarveClearToggled),
                                ]
                                .spacing(8)
                                .align_y(Alignment::Center),
                            );
                            if let Some(cl) = &c.clear {
                                // Flat-bottomed families only: the clearing pass exists
                                // to leave a flat floor, and the strategy errors on a
                                // tool that cannot.
                                let choices: Vec<ToolChoice> = self
                                    .library
                                    .tools
                                    .iter()
                                    .enumerate()
                                    .filter(|(_, t)| {
                                        matches!(
                                            t.kind,
                                            ToolKind::EndMill | ToolKind::BullNose { .. }
                                        )
                                    })
                                    .map(|(index, t)| ToolChoice {
                                        index,
                                        number: t.number,
                                        diameter: t.diameter,
                                        kind: t.kind,
                                    })
                                    .collect();
                                let current = Some(cl.tool).and_then(|n| {
                                    let embedded = self
                                        .controller
                                        .document()
                                        .setup
                                        .tools
                                        .iter()
                                        .find(|t| t.number == n)
                                        .copied()?;
                                    let i = self.library.tools.iter().position(|l| {
                                        l.diameter == embedded.diameter
                                            && l.length == embedded.length
                                            && l.flutes == embedded.flutes
                                            && l.kind == embedded.kind
                                    })?;
                                    choices.iter().find(|ch| ch.index == i).cloned()
                                });
                                list = list.push(
                                    row![
                                        label_help("Clearing tool", help::CARVE_CLEAR, self.tooltips),
                                        pick_list(choices, current, |ch| {
                                            Message::CarveClearToolChanged(ch.index)
                                        })
                                        .placeholder("Choose an end mill")
                                        .text_size(13)
                                        .width(Length::Fixed(INSPECTOR_TOOL_PICKER_W)),
                                    ]
                                    .spacing(8)
                                    .align_y(Alignment::Center),
                                );
                                // A pocket's controls, because that is what this pass is
                                // — held back from the main field block above so they sit
                                // under the tool they belong to.
                                for f in self.inspector_fields() {
                                    if !is_clear_field(f) {
                                        continue;
                                    }
                                    let value =
                                        self.fields.get(&f).cloned().unwrap_or_default();
                                    list = list.push(field_row_labeled(
                                        f,
                                        self.field_label(f),
                                        self.field_help(f),
                                        &value,
                                        self.tooltips,
                                        self.field_invalid(f),
                                    ));
                                }
                                list = list.push(profile_picker(
                                    "Clearing plunge",
                                    help::CARVE_CLEAR_PLUNGE,
                                    self.tooltips,
                                    PlungeKind::of(cl.params.plunge),
                                    &PlungeKind::ALL[..],
                                    Message::CarveClearPlungeChanged,
                                ));
                                list = list.push(
                                    row![
                                        label_help("Clearing climb", help::CLIMB, self.tooltips),
                                        checkbox(cl.params.clearing.climb)
                                            .size(15)
                                            .on_toggle(Message::CarveClearClimbToggled),
                                    ]
                                    .spacing(8)
                                    .align_y(Alignment::Center),
                                );
                                list = list.push(profile_picker(
                                    "Clearing lead-in",
                                    help::LEAD_IN,
                                    self.tooltips,
                                    LeadKind::of(cl.params.lead_in),
                                    &LeadKind::ALL[..],
                                    Message::CarveClearLeadInChanged,
                                ));
                                list = list.push(profile_picker(
                                    "Clearing lead-out",
                                    help::LEAD_OUT,
                                    self.tooltips,
                                    LeadKind::of(cl.params.lead_out),
                                    &LeadKind::ALL[..],
                                    Message::CarveClearLeadOutChanged,
                                ));
                            }
                        }
                    }
                }
                _ => {}
            }
        }

        list = list.push(
            button("Apply").on_press_maybe((!self.any_field_invalid() && self.inspector_dirty()).then_some(Message::Apply)),
        );
        // Two classes of control live in this inspector and they commit differently, so
        // say which is which: a typed number is only a number once you have finished
        // typing it, but a drop-down or a checkbox has no half-way state and is written
        // straight to the document (undoably) the moment it changes. Without this the
        // greyed-out button reads as "your change was ignored" rather than "already done".
        // The Tooling editor has no such split — everything there waits for Apply — so it
        // gets no hint.
        list = list.push(
            text("Applies typed values. Drop-downs and checkboxes take effect as you set them.")
                .size(11)
                .color(palette::GROUP_LABEL),
        );

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

/// The numeric parameters the inspector shows for an operation, in display order.
///
/// A free function taking only the operation, so the list can be checked without
/// standing up a GUI — `visible_fields` is a method on the iced `App` and nothing
/// headless can reach it. The same reason `origin_move_targets` was extracted: a
/// rule the tests cannot see is a rule that drifts, and this one silently governs
/// whether an operation's parameter is reachable by the operator at all.
fn operation_fields(op: &Operation) -> Vec<Field> {
    match op {
        Operation::Profile(p) => {
            let mut fields = vec![Field::Depth, Field::Stepdown];
            // Radial roughing (stepover) is outside-only; an inner profile is
            // a single-pass wall finish (rough the pocket first).
            if p.side == Side::Outside {
                fields.push(Field::Stepover);
                fields.push(Field::Engagement);
            }
            fields.extend([Field::ProfileOffset, Field::SpindleRpm, Field::Feed, Field::PlungeFeed]);
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
        Operation::Pocket(p) => {
            let mut fields = vec![
                Field::Depth,
                Field::Stepdown,
                Field::Overlap,
                Field::Engagement,
                Field::ProfileOffset,
                Field::SpindleRpm,
                Field::Feed,
                Field::PlungeFeed,
                Field::LeadOverlap,
            ];
            if p.clear.lead_in != Lead::None {
                fields.push(Field::LeadInSize);
            }
            if p.clear.lead_out != Lead::None {
                fields.push(Field::LeadOutSize);
            }
            match p.clear.plunge {
                Plunge::Straight => {}
                Plunge::Ramp { .. } => fields.push(Field::PlungeA),
                Plunge::Helix { .. } | Plunge::ZigZag { .. } => {
                    fields.push(Field::PlungeA);
                    fields.push(Field::PlungeB);
                }
            }
            fields
        }
        Operation::Face(_) => vec![
            Field::FaceStartOffset,
            Field::Depth,
            Field::Stepdown,
            Field::Overlap,
            Field::FaceOvershoot,
            Field::SpindleRpm,
            Field::Feed,
            Field::PlungeFeed,
        ],
        Operation::Drill(_) => vec![
            Field::Depth,
            Field::DrillStartOffset,
            Field::Peck,
            Field::Dwell,
            Field::SpindleRpm,
            Field::Feed,
        ],
        Operation::Chamfer(c) => {
            let mut fields = vec![
                // Heights first and top-down, as the Setup inspector reads.
                Field::ChamferTop,
                Field::ChamferWidth,
                Field::ChamferDepth,
                Field::ChamferStep,
                Field::SpindleRpm,
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
        Operation::Engrave(_) => {
            vec![
                Field::Depth,
                Field::Stepdown,
                Field::SpindleRpm,
                Field::Feed,
                Field::PlungeFeed,
            ]
        }
        // Carve has no stepdown (single-pass, deferred) and no stepover: its
        // ring spacing plays that role, and `Depth` is a *cap* the shape may
        // not reach.
        Operation::Carve(c) => {
            let mut fields = vec![
                Field::Depth,
                Field::ProfileOffset,
                Field::RingStep,
                Field::Scallop,
                Field::SpindleRpm,
                Field::Feed,
                Field::PlungeFeed,
            ];
            // The V-bit's own entry parameters, exactly as a profile's.
            match c.plunge {
                Plunge::Straight => {}
                Plunge::Ramp { .. } => fields.push(Field::PlungeA),
                Plunge::Helix { .. } | Plunge::ZigZag { .. } => {
                    fields.push(Field::PlungeA);
                    fields.push(Field::PlungeB);
                }
            }
            // The clearing pass is a pocket over a derived region, so it gets a
            // pocket's controls -- all but two. `Depth` is not its own (the
            // carve's cap sets it), and a wall finishing allowance would leave a
            // full-depth ridge at the wall/floor junction, since the V-bit's
            // innermost ring runs along that boundary rather than beside it.
            if let Some(cl) = &c.clear {
                fields.extend([
                    Field::ClearStepdown,
                    Field::ClearOverlap,
                    Field::ClearOffset,
                    Field::ClearEngagement,
                    Field::ClearFeed,
                    Field::ClearPlungeFeed,
                    Field::ClearLeadOverlap,
                ]);
                if cl.params.lead_in != Lead::None {
                    fields.push(Field::ClearLeadInSize);
                }
                if cl.params.lead_out != Lead::None {
                    fields.push(Field::ClearLeadOutSize);
                }
                match cl.params.plunge {
                    Plunge::Straight => {}
                    Plunge::Ramp { .. } => fields.push(Field::ClearPlungeA),
                    Plunge::Helix { .. } | Plunge::ZigZag { .. } => {
                        fields.push(Field::ClearPlungeA);
                        fields.push(Field::ClearPlungeB);
                    }
                }
            }
            fields
        }
        Operation::Thread(t) => {
            let mut fields = vec![
                Field::MajorDia,
                Field::Pitch,
                Field::ThreadTop,
                Field::ThreadBottom,
                Field::ThreadPasses,
                Field::ThreadSpringPasses,
            ];
            // Blind-hole fields only for internal threads (a boss has no bore).
            if t.internal {
                fields.push(Field::ThreadDrillClearance);
                if t.drill_clearance > 0.0 {
                    fields.push(Field::ThreadBlindAllowance);
                }
            }
            fields.push(Field::SpindleRpm);
            fields.push(Field::Feed);
            fields.push(Field::PlungeFeed);
            fields
        }
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
    mins: &crate::PanePrefs,
) -> f32 {
    match node {
        pane_grid::Node::Pane(p) => panes.get(*p).map_or(0.0, |pane| pane.min_size(mins)),
        pane_grid::Node::Split {
            axis: sub_axis,
            a,
            b,
            ratio,
            ..
        } => {
            let (ma, mb) = (subtree_min(a, panes, axis, mins), subtree_min(b, panes, axis, mins));
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
    /// Tooltip for *this* command, when the icon's own words do not fit.
    ///
    /// Help used to be keyed by icon alone, which is right while an icon means one
    /// thing. The moment one is reused — the tool icon standing in for "add a machine" —
    /// it silently describes the wrong action, which is worse than no tooltip.
    help: Option<&'static str>,
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
        help: None,
    }
}

/// A command whose tooltip is its own, because it borrows an icon that means something
/// else elsewhere.
///
/// Help keyed by icon alone silently describes the wrong action the moment an icon is
/// reused — which is how "Add machine" came to read "Add a new tool to the library".
fn cmd_help(
    icon: Icon,
    label: &'static str,
    action: Option<Message>,
    help: &'static str,
) -> Command {
    Command {
        help: Some(help),
        ..cmd(icon, label, action)
    }
}

fn toggle_cmd(icon: Icon, label: &'static str, on: bool, msg: Message) -> Command {
    Command {
        icon,
        label,
        action: Some(msg),
        help: None,
        toggle: Some(on),
    }
}

/// Render a single command at Full (`compact = false`, icon over label) or Compact
/// (`compact = true`, icon only) density. When `show`, the button carries a hover
/// tooltip describing the command — the icon's only self-explanation at Compact.
fn render_command(command: &Command, compact: bool, show: bool) -> Element<'static, Message> {
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
    let btn: Element<'static, Message> = match command.toggle {
        Some(on) => base
            .on_press_maybe(command.action.clone())
            .style(move |_theme, status| command_toggle_style(on, status))
            .into(),
        None => base
            .on_press_maybe(command.action.clone())
            .style(|_theme, status| command_button_style(status))
            .into(),
    };
    if !show {
        return btn;
    }
    tooltip(
        btn,
        container(text(command.help.unwrap_or_else(|| command.icon.help())).size(12))
            .padding(8)
            .max_width(300.0)
            .style(container::rounded_box),
        tooltip::Position::Bottom,
    )
    .into()
}

/// Render a whole group at its assigned density. Collapsed/Tight groups become a
/// single button that toggles the group's popup (`index` in the active tab).
fn render_group(spec: &GroupSpec, density: Density, index: usize, show: bool) -> Element<'static, Message> {
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
        commands = commands.push(render_command(command, compact, show));
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
        Operation::Engrave(_) => "Engrave",
        Operation::Carve(_) => "Carve",
    }
}

/// Read an operation's value for a given field, if the op has it.
fn op_field(op: &Operation, field: Field) -> Option<f64> {
    match (op, field) {
        // Spindle speed is common to every operation kind.
        (op, Field::SpindleRpm) => Some(op.spindle_rpm()),
        (Operation::Profile(o), Field::Depth) => Some(o.depth),
        (Operation::Profile(o), Field::Stepover) => Some(o.stepover),
        (Operation::Profile(o), Field::ProfileOffset) => Some(o.offset),
        (Operation::Profile(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Profile(o), Field::Feed) => Some(o.feed),
        (Operation::Profile(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Profile(o), Field::LeadInSize) => Some(lead_size(o.lead_in)),
        (Operation::Profile(o), Field::LeadOutSize) => Some(lead_size(o.lead_out)),
        (Operation::Profile(o), Field::LeadOverlap) => Some(o.lead_overlap),
        (Operation::Carve(o), Field::Depth) => Some(o.depth),
        (Operation::Carve(o), Field::ProfileOffset) => Some(o.offset),
        (Operation::Carve(o), Field::RingStep) => Some(o.ring_step),
        (Operation::Carve(o), Field::Scallop) => Some(o.scallop),
        (Operation::Carve(o), Field::Feed) => Some(o.feed),
        (Operation::Carve(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Carve(o), Field::PlungeA) => Some(plunge_params(o.plunge).0),
        (Operation::Carve(o), Field::PlungeB) => Some(plunge_params(o.plunge).1),
        (Operation::Carve(o), Field::ClearStepdown) => Some(o.clear?.params.stepdown),
        (Operation::Carve(o), Field::ClearOverlap) => Some(o.clear?.params.overlap * 100.0),
        (Operation::Carve(o), Field::ClearOffset) => Some(o.clear?.params.offset),
        (Operation::Carve(o), Field::ClearEngagement) => Some(o.clear?.params.clearing.engagement),
        (Operation::Carve(o), Field::ClearFeed) => Some(o.clear?.params.feed),
        (Operation::Carve(o), Field::ClearPlungeFeed) => Some(o.clear?.params.plunge_feed),
        (Operation::Carve(o), Field::ClearLeadOverlap) => Some(o.clear?.params.lead_overlap),
        (Operation::Carve(o), Field::ClearLeadInSize) => Some(lead_size(o.clear?.params.lead_in)),
        (Operation::Carve(o), Field::ClearLeadOutSize) => Some(lead_size(o.clear?.params.lead_out)),
        (Operation::Carve(o), Field::ClearPlungeA) => Some(plunge_params(o.clear?.params.plunge).0),
        (Operation::Carve(o), Field::ClearPlungeB) => Some(plunge_params(o.clear?.params.plunge).1),
        (Operation::Profile(o), Field::Engagement) => Some(o.clearing.engagement),
        (Operation::Profile(o), Field::PlungeA) => Some(plunge_params(o.plunge).0),
        (Operation::Profile(o), Field::PlungeB) => Some(plunge_params(o.plunge).1),
        (Operation::Pocket(o), Field::Depth) => Some(o.depth),
        (Operation::Pocket(o), Field::Stepdown) => Some(o.clear.stepdown),
        (Operation::Pocket(o), Field::Overlap) => Some(o.clear.overlap * 100.0),
        (Operation::Pocket(o), Field::ProfileOffset) => Some(o.clear.offset),
        (Operation::Pocket(o), Field::LeadInSize) => Some(lead_size(o.clear.lead_in)),
        (Operation::Pocket(o), Field::LeadOutSize) => Some(lead_size(o.clear.lead_out)),
        (Operation::Pocket(o), Field::Feed) => Some(o.clear.feed),
        (Operation::Pocket(o), Field::PlungeFeed) => Some(o.clear.plunge_feed),
        (Operation::Pocket(o), Field::LeadOverlap) => Some(o.clear.lead_overlap),
        (Operation::Pocket(o), Field::Engagement) => Some(o.clear.clearing.engagement),
        (Operation::Pocket(o), Field::PlungeA) => Some(plunge_params(o.clear.plunge).0),
        (Operation::Pocket(o), Field::PlungeB) => Some(plunge_params(o.clear.plunge).1),
        (Operation::Face(o), Field::FaceStartOffset) => Some(o.start_offset),
        (Operation::Face(o), Field::Depth) => Some(o.depth),
        (Operation::Face(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Face(o), Field::Overlap) => Some(o.overlap * 100.0),
        (Operation::Face(o), Field::FaceOvershoot) => Some(o.overshoot),
        (Operation::Face(o), Field::Feed) => Some(o.feed),
        (Operation::Face(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Drill(o), Field::Depth) => Some(o.depth),
        (Operation::Drill(o), Field::DrillStartOffset) => Some(o.start_offset),
        (Operation::Drill(o), Field::Peck) => Some(o.peck.unwrap_or(0.0)),
        (Operation::Drill(o), Field::Dwell) => Some(o.dwell.unwrap_or(0.0)),
        (Operation::Drill(o), Field::Feed) => Some(o.feed),
        (Operation::Engrave(o), Field::Depth) => Some(o.depth),
        (Operation::Engrave(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Engrave(o), Field::Feed) => Some(o.feed),
        (Operation::Engrave(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Chamfer(o), Field::ChamferTop) => Some(o.top),
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
        (Operation::Thread(o), Field::ThreadPasses) => Some(o.passes as f64),
        (Operation::Thread(o), Field::ThreadSpringPasses) => Some(o.spring_passes as f64),
        (Operation::Thread(o), Field::ThreadDrillClearance) => Some(o.drill_clearance),
        (Operation::Thread(o), Field::ThreadBlindAllowance) => Some(o.blind_allowance),
        (Operation::Thread(o), Field::Feed) => Some(o.feed),
        (Operation::Thread(o), Field::PlungeFeed) => Some(o.plunge_feed),
        _ => None,
    }
}

/// Write the parsed inspector fields onto an operation.
fn apply_op_fields(op: &mut Operation, parsed: &BTreeMap<Field, f64>) {
    let get = |f: Field| parsed.get(&f).copied();
    // Spindle speed is common to every operation kind.
    if let Some(v) = get(Field::SpindleRpm) {
        op.set_spindle_rpm(v.max(0.0));
    }
    match op {
        Operation::Profile(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v;
            }
            if let Some(v) = get(Field::Stepover) {
                o.stepover = v.max(0.0);
            }
            if let Some(v) = get(Field::Engagement) {
                o.clearing.engagement = v.max(0.0);
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
                o.clear.stepdown = v;
            }
            if let Some(v) = get(Field::Overlap) {
                o.clear.overlap = (v / 100.0).clamp(0.0, 0.99);
            }
            if let Some(v) = get(Field::Engagement) {
                o.clear.clearing.engagement = v.max(0.0);
            }
            if let Some(v) = get(Field::ProfileOffset) {
                o.clear.offset = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.clear.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.clear.plunge_feed = v;
            }
            if let Some(v) = get(Field::LeadOverlap) {
                o.clear.lead_overlap = v.max(0.0);
            }
            if let Some(v) = get(Field::LeadInSize) {
                o.clear.lead_in = set_lead_size(o.clear.lead_in, v);
            }
            if let Some(v) = get(Field::LeadOutSize) {
                o.clear.lead_out = set_lead_size(o.clear.lead_out, v);
            }
            let (a, b) = plunge_params(o.clear.plunge);
            let a = get(Field::PlungeA).unwrap_or(a);
            let b = get(Field::PlungeB).unwrap_or(b);
            o.clear.plunge = set_plunge_params(o.clear.plunge, a, b);
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
            if let Some(v) = get(Field::Overlap) {
                // Stored as a fraction; the field edits a percentage. Clamp below
                // 100 % so the pass spacing stays positive.
                o.overlap = (v / 100.0).clamp(0.0, 0.99);
            }
            if let Some(v) = get(Field::FaceOvershoot) {
                // Negative is allowed — it plunges into the stock (the strategy warns).
                o.overshoot = v;
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
            if let Some(v) = get(Field::DrillStartOffset) {
                o.start_offset = v;
            }
            if let Some(v) = get(Field::Peck) {
                // 0 (or negative) clears pecking; the toolpath requires peck > 0.
                o.peck = (v > 0.0).then_some(v);
            }
            if let Some(v) = get(Field::Dwell) {
                o.dwell = (v > 0.0).then_some(v);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
        }
        Operation::Chamfer(o) => {
            // Deliberately unclamped: an edge below the part datum is an ordinary
            // thing to chamfer, so a negative Z is a value, not a mistake.
            if let Some(v) = get(Field::ChamferTop) {
                o.top = v;
            }
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
            if let Some(v) = get(Field::ThreadPasses) {
                o.passes = (v.round() as u32).max(1);
            }
            if let Some(v) = get(Field::ThreadSpringPasses) {
                o.spring_passes = v.round().max(0.0) as u32;
            }
            if let Some(v) = get(Field::ThreadDrillClearance) {
                o.drill_clearance = v.max(0.0);
            }
            if let Some(v) = get(Field::ThreadBlindAllowance) {
                o.blind_allowance = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
        }
        Operation::Engrave(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
        }
        Operation::Carve(o) => {
            if let Some(cl) = &mut o.clear {
                let p = &mut cl.params;
                if let Some(v) = get(Field::ClearStepdown) {
                    p.stepdown = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearOverlap) {
                    p.overlap = (v / 100.0).clamp(0.0, 0.99);
                }
                if let Some(v) = get(Field::ClearOffset) {
                    p.offset = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearEngagement) {
                    p.clearing.engagement = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearFeed) {
                    p.feed = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearPlungeFeed) {
                    p.plunge_feed = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearLeadOverlap) {
                    p.lead_overlap = v.max(0.0);
                }
                if let Some(v) = get(Field::ClearLeadInSize) {
                    p.lead_in = set_lead_size(p.lead_in, v);
                }
                if let Some(v) = get(Field::ClearLeadOutSize) {
                    p.lead_out = set_lead_size(p.lead_out, v);
                }
                let (a, b) = plunge_params(p.plunge);
                let a = get(Field::ClearPlungeA).unwrap_or(a);
                let b = get(Field::ClearPlungeB).unwrap_or(b);
                p.plunge = set_plunge_params(p.plunge, a, b);
            }
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::ProfileOffset) {
                o.offset = v;
            }
            if let Some(v) = get(Field::RingStep) {
                o.ring_step = v.max(0.0);
            }
            if let Some(v) = get(Field::Scallop) {
                o.scallop = v.max(0.0);
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
            }
            if let Some(v) = get(Field::PlungeFeed) {
                o.plunge_feed = v;
            }
            let (a, b) = plunge_params(o.plunge);
            let a = get(Field::PlungeA).unwrap_or(a);
            let b = get(Field::PlungeB).unwrap_or(b);
            o.plunge = set_plunge_params(o.plunge, a, b);
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
/// A tool's flute count as a short label, e.g. `"2 flutes"` / `"1 flute"`.
fn flute_count(flutes: u32) -> String {
    format!("{flutes} flute{}", if flutes == 1 { "" } else { "s" })
}

/// The end-mill family — square, ball-nose and rounded-edge (bull-nose) end mills —
/// which share a field set and dimensions, so switching between them preserves the
/// tool's measurements (a Type change *across* families resets to defaults instead).
fn end_mill_family(kind: ToolKind) -> bool {
    matches!(
        kind,
        ToolKind::EndMill | ToolKind::BallMill | ToolKind::BullNose { .. }
    )
}

/// The **Tool Library** parenthetical — the extra, kind-specific descriptor shown only
/// in the Library pane (never the Project pane, which carries just number + compact
/// form). Radii use a shared `r…` form (`r0.5`, `r0`) for consistency across kinds, and
/// cutting direction a single letter (D/U/S). `None` = no parenthetical for this kind.
///
/// - End mills (square / ball): `(2 flutes, D)`
/// - Rounded-edge end mills: `(2 flutes, D, r1)` — trailing corner radius
/// - V-bits: `(60°, r0.25)` — point angle + tip radius
/// - Face / chamfer / thread mills: `(2 flutes)`
/// - Drill bits: none
fn library_extra(t: &cam_model::Tool) -> Option<String> {
    let dir = |d: CutDir| match d {
        CutDir::Down => "D",
        CutDir::Up => "U",
        CutDir::Straight => "S",
    };
    match t.kind {
        ToolKind::EndMill | ToolKind::BallMill => {
            Some(format!("{}, {}", flute_count(t.flutes), dir(t.cutting_direction)))
        }
        ToolKind::BullNose { corner_radius } => Some(format!(
            "{}, {}, r{}",
            flute_count(t.flutes),
            dir(t.cutting_direction),
            fmt_num(corner_radius)
        )),
        ToolKind::VBit {
            included_angle_deg,
            tip_radius,
        } => Some(format!("{}°, r{}", fmt_num(included_angle_deg), fmt_num(tip_radius))),
        // Chamfer mill mirrors the V-bit, but its tip is a flat ⌀ (non-cutting), not a
        // rounded radius — hence `⌀…` rather than `r…`.
        ToolKind::ChamferMill {
            included_angle_deg,
            tip_diameter,
        } => Some(format!("{}°, ⌀{}", fmt_num(included_angle_deg), fmt_num(tip_diameter))),
        // Thread mill: pitch first (its distinguishing trait), then flute count.
        ToolKind::ThreadMill { pitch } => Some(format!(
            "{}, {}",
            match pitch {
                Some(p) => format!("P{}", fmt_num(p)),
                None => "any pitch".to_string(),
            },
            flute_count(t.flutes)
        )),
        ToolKind::FaceMill => Some(flute_count(t.flutes)),
        ToolKind::Drill { .. } => None,
    }
}

/// A stable family sort/order key for the Tool Library pane's Family view.
fn kind_order(kind: ToolKind) -> u8 {
    match kind {
        ToolKind::EndMill => 0,
        ToolKind::BallMill => 1,
        ToolKind::BullNose { .. } => 2,
        ToolKind::FaceMill => 3,
        ToolKind::ChamferMill { .. } => 4,
        ToolKind::VBit { .. } => 5,
        ToolKind::Drill { .. } => 6,
        ToolKind::ThreadMill { .. } => 7,
    }
}

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

/// Width of an inspector's editable control, logical px.
///
/// Numeric inputs and pickers share it so their right edges line up down the column —
/// with a carve's two tools stacked in one inspector, a ragged edge reads as disorder.
const INSPECTOR_INPUT_W: f32 = 90.0;

/// Width for a picker whose options do not fit [`INSPECTOR_INPUT_W`] — "Conventional",
/// "Right-hand", a post's name. This is the width every picker had before the column was
/// introduced, kept unchanged for the ones that were already right.
const INSPECTOR_PICKER_W: f32 = 120.0;

/// Width for a tool picker. Narrower than its longest entry on purpose: the text wraps,
/// which costs a line, where sizing to content would move the column every time the
/// selection changed.
const INSPECTOR_TOOL_PICKER_W: f32 = 140.0;

/// An inspector field row: an explicit label + tooltip and a numeric text input bound to
/// `field`, drawn **invalid** (red border + value) when `invalid` — the thicker border
/// carries the signal by weight, not hue alone. The explicit label lets a field be
/// renamed and re-explained per tool kind (e.g. a V-bit's `ToolDiameter` reads "Shank
/// diameter" with matching help).
fn field_row_labeled<'a>(
    field: Field,
    label: &'a str,
    help: &'static str,
    value: &str,
    show: bool,
    invalid: bool,
) -> Element<'a, Message> {
    let mut input = text_input("", value)
        .on_input(move |v| Message::FieldChanged(field, v))
        .on_submit(Message::Apply)
        .width(Length::Fixed(INSPECTOR_INPUT_W));
    if invalid {
        input = input.style(|theme, status| {
            let mut s = iced::widget::text_input::default(theme, status);
            s.border.color = palette::ERROR;
            s.border.width = 1.5;
            s.value = palette::ERROR;
            s
        });
    }
    row![label_help(label, help, show), input]
        .spacing(8)
        .align_y(Alignment::Center)
        .into()
}

/// A labelled strategy picker row (lead / plunge kind, side, …) for the inspector,
/// with a hover tooltip (`help`) explaining the choice when `show`.
fn profile_picker<T>(
    label: &str,
    help: &'static str,
    show: bool,
    selected: T,
    options: &'static [T],
    on_select: impl Fn(T) -> Message + 'static,
) -> Element<'static, Message>
where
    T: ToString + PartialEq + Clone + 'static,
{
    profile_picker_w(label, help, show, selected, options, on_select, INSPECTOR_INPUT_W)
}

/// [`profile_picker`] at an explicit width, for the few whose option text does not fit
/// the shared column.
#[allow(clippy::too_many_arguments)]
fn profile_picker_w<T>(
    label: &str,
    help: &'static str,
    show: bool,
    selected: T,
    options: &'static [T],
    on_select: impl Fn(T) -> Message + 'static,
    width: f32,
) -> Element<'static, Message>
where
    T: ToString + PartialEq + Clone + 'static,
{
    row![
        label_help(label.to_string(), help, show),
        pick_list(options, Some(selected), on_select)
            .text_size(13)
            .width(Length::Fixed(width)),
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

/// The origins an operation could be moved to: every origin in `indices` except the one
/// the operation already sits under, keeping `indices`' order so the menu matches the
/// tree. Empty on a single-origin job, which is what hides the menu section entirely.
///
/// `current` is the operation's stored `work_offset`, which need not name a live origin
/// — deleting an origin leaves its operations pointing at an index that is gone. The
/// tree groups such an operation under `base`, so this treats it as being there. Reading
/// the stale index literally would offer the base as a destination for a row already
/// drawn under the base, and moving it there would look like nothing happened.
///
/// A free function, not a method: it is the whole rule, and this way it can be tested
/// without standing up an application.
fn origin_move_targets(indices: &[u32], current: Option<u32>, base: u32) -> Vec<u32> {
    let here = current
        .filter(|wo| indices.contains(wo))
        .unwrap_or(base);
    indices.iter().copied().filter(|i| *i != here).collect()
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
    /// Scene extent (backplot + overlays) — used to *size* markers to the view.
    bounds: Option<([f32; 3], [f32; 3])>,
    /// The box the camera frames on: the **stock**, which is stable across
    /// operation-parameter edits. Framing on this (rather than the toolpath) keeps
    /// the part a constant on-screen size — tweaking an offset/stepover must not
    /// appear to resize the design.
    frame_bounds: Option<([f32; 3], [f32; 3])>,
    controls: ViewControls,
    show_gizmo: bool,
    /// On-screen cube size, logical px (fixed; independent of window size).
    gizmo_size: f32,
    /// The object-snap catch aperture, logical px. Carried per frame like `gizmo_size`
    /// rather than read from a constant, because it is a user preference now.
    ///
    /// The marker *scale* is not held here: the marker is built into the scene during
    /// [`Viewport::new`], where the preferences are still in scope, so storing it
    /// would be a copy with no reader.
    snap_catch_px: f32,
    /// Geometry-pick mode (a new-operation wizard is awaiting a region click).
    picking: bool,
    /// "Set origin" pick mode — a click drops the workpiece datum.
    set_origin: bool,
    /// An object-snap is engaged under the cursor — its marker replaces the
    /// crosshair/pickbox.
    snap_engaged: bool,
    /// The world Z of the plane clicks are projected onto (top of stock).
    pick_z: f32,
    /// Drilling annotations (peck rings / dwell bars) anchored in world space;
    /// sized to constant screen pixels and billboarded in [`Self::draw`].
    drill_marks: Vec<DrillMark>,
}

impl Viewport {
    #[allow(clippy::too_many_arguments)] // cohesive per-frame view inputs
    fn new(
        controller: &AppController,
        show_stock: bool,
        controls: ViewControls,
        show_gizmo: bool,
        gizmo_size: f32,
        prefs: &crate::Settings,
        focus_ops: &[u32],
        snap: Option<(SnapHit, f64)>,
        hover_loop: Option<LoopRef>,
        set_origin: bool,
        show_origin: bool,
        origin_first: Option<[f64; 3]>,
        // The active machine's travel per axis, when the envelope is shown.
        envelope: Option<[f32; 3]>,
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
                // Open imported strokes too, or an engravable path is invisible until
                // (and unless) a run happens to include it.
                for path in controller.open_paths() {
                    scene.add_open_path(path.points(), PART);
                }
                scene
            }
        };
        // Drilling annotations from the backplot (peck rings + dwell bars), once a
        // run has produced a program. Anchors only — sized to the screen at draw.
        let drill_marks = controller
            .outcome()
            .map(|o| drill_marks_of(&o.program))
            .unwrap_or_default();
        // Frame the camera on the *stable* part/backplot only — capture bounds
        // before the transient pick overlays (hover highlight, snap marker), or
        // the whole view would re-fit and drift as the cursor moves.
        let bounds = scene.bounds();
        // While the pick wizard is active, highlight the chosen boundary (accent)
        // and any excluded islands (gold) so the user sees what they picked.
        if let Some(pending) = controller.pending_op() {
            // The loop under the cursor (what a click selects), drawn first so the
            // boundary/island highlights paint over it once chosen.
            if let Some((pts, closed)) = hover_loop
                .filter(|l| Some(*l) != pending.boundary)
                .and_then(|l| controller.loop_points(l))
            {
                add_path_highlight(&mut scene, &pts, closed, PICK_HOVER);
            }
            if let Some((pts, closed)) = pending.boundary.and_then(|b| controller.loop_points(b)) {
                add_path_highlight(&mut scene, &pts, closed, PICK_BOUNDARY);
            }
            for island in &pending.islands {
                if let Some((pts, closed)) = controller.loop_points(*island) {
                    add_path_highlight(&mut scene, &pts, closed, PICK_ISLAND);
                }
            }
        }
        // The workpiece-origin datum marker (View toggle), only once geometry is
        // loaded — never on an empty document at startup — and sized to the scene.
        if show_origin && controller.has_geometry() {
            if let Some((mn, mx)) = bounds {
                let scale = prefs.view.origin_marker_scale;
                let base_r = ((mx[0] - mn[0]).max(mx[1] - mn[1]) * 0.06 * scale).max(1.0);
                let active = controller.active_origin();
                // One marker per origin so every datum is visible; the active origin
                // (the one the pick flow and inspector edit) is drawn larger, so it
                // reads without relying on colour.
                for idx in controller.origin_indices() {
                    let pos = controller.origin_position(idx);
                    let r = if idx == active { base_r } else { base_r * 0.65 };
                    add_origin_marker(&mut scene, pos, pick_z, r);
                }
            }
        }
        // The first point captured in two-point origin mode (awaiting the second).
        if let Some(first) = origin_first {
            let r = bounds
                .map(|(mn, mx)| {
                    ((mx[0] - mn[0]).max(mx[1] - mn[1]) * 0.04 * prefs.view.origin_marker_scale)
                        .max(1.0)
                })
                .unwrap_or(3.0);
            add_origin_marker(&mut scene, first, pick_z, r);
        }
        // The machine's travel, around the job. Added *after* `bounds` was captured, so a
        // large machine cannot re-frame the camera and shrink the part — the box is
        // context, not content.
        //
        // Red when the job does not fit: `check_travel` compares the program's span
        // against travel per axis, so this shows the same comparison before the export
        // refuses it rather than after.
        if let Some(travel) = envelope {
            if let Some((lo, hi)) = bounds {
                let over = (hi[0] - lo[0]) > travel[0]
                    || (hi[1] - lo[1]) > travel[1]
                    || (hi[2] - lo[2]) > travel[2];
                let colour = if over { ENVELOPE_OVER } else { ENVELOPE };
                scene.add_envelope(travel, (lo, hi), colour);
            }
        }
        // The object-snap marker under the cursor (op pick *or* set-origin).
        if let Some((hit, aperture)) = snap {
            add_snap_marker(&mut scene, hit, aperture, prefs.snapping.marker_scale, pick_z);
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
        // Frame on the stock (stable across parameter edits) so the part keeps a
        // constant on-screen size; fall back to the scene extent before any
        // geometry is loaded.
        let frame_bounds = if !controller.has_geometry() {
            bounds
        } else {
            let (mn, mx) = controller.stock_box();
            Some((
                [mn[0] as f32, mn[1] as f32, mn[2] as f32],
                [mx[0] as f32, mx[1] as f32, mx[2] as f32],
            ))
        };
        Self {
            vertices: Arc::new(scene.line_vertices()),
            mesh_vertices: Arc::new(mesh_vertices),
            mesh_indices: Arc::new(mesh_indices),
            bounds,
            frame_bounds,
            controls,
            show_gizmo,
            gizmo_size,
            snap_catch_px: prefs.snapping.pickbox_px * crate::SNAP_CATCH_MULTIPLE,
            picking,
            set_origin,
            snap_engaged: snap.is_some(),
            pick_z,
            drill_marks,
        }
    }

    /// The orbit camera framed on the stable stock box, with the current controls.
    fn camera(&self) -> OrbitCamera {
        let (min, max) = self
            .frame_bounds
            .or(self.bounds)
            .unwrap_or(([0.0, 0.0, 0.0], [1.0, 1.0, 0.0]));
        let mut cam = OrbitCamera::framed(min, max);
        // Keep the lateral framing on the stock (stable on-screen size), but stretch the
        // depth range to cover the whole toolpath — tall tool-change lifts reach well
        // above the stock, and would otherwise clip on rotate at any zoom.
        if let Some((smin, smax)) = self.bounds {
            cam.cover_depth(smin, smax);
        }
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
                        let aperture =
                            0.5 * self.snap_catch_px * cam.world_per_pixel(bounds.height);
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
                            let aperture =
                                0.5 * self.snap_catch_px * cam.world_per_pixel(bounds.height);
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
        // Drilling annotations are sized here, where the viewport's pixel height is
        // known, so they stay a constant screen size at any zoom (never scaling with
        // the model) and billboard to face the camera at any orbit angle.
        let vertices = if self.drill_marks.is_empty() {
            self.vertices.clone()
        } else {
            let cam = self.camera();
            let px_world = DRILL_MARK_PX * cam.world_per_pixel(bounds.height);
            let (right, up) = (cam.right(), cam.up());
            let mut v = (*self.vertices).clone();
            for &mark in &self.drill_marks {
                push_drill_mark(&mut v, mark, px_world, right, up);
            }
            Arc::new(v)
        };
        ScenePrimitive {
            vertices,
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

/// A 2D cross-section preview of a tool's generatrix (TOOLING_PLAN Phase 5), shown in
/// place of the 3D backplot while a tool is being edited. The generatrix is mirrored
/// about the axis for a full-silhouette read; **cutting** surfaces draw as a solid,
/// thicker line and **non-cutting** (shank/neck/top) as a thin **dashed** line — the
/// distinction is by *linestyle, never hue* (colour-blind-safe). A few dimension labels
/// annotate the driving values.
struct ToolCanvas {
    profile: cam_geo::Profile2D,
    diameter: f64,
    length: f64,
    flute_length: f64,
    flutes: u32,
    cutting_direction: CutDir,
    /// Whether to draw the flute helix at all (off for V-bits).
    draw_flutes: bool,
    /// Axial advance per helix turn (drill bits ≈ 3·⌀ for a realistic pitch; other
    /// kinds do one turn over the cutting region).
    flute_pitch: f64,
    /// Sign of the helix lean (+1 leans one way, −1 the other). Derived from the
    /// cutting direction, but inverted for a twist drill so it reads right-hand.
    flute_sign: f64,
    /// Draw a row of 90° square inserts seated at the bottom of the body (face mills),
    /// so the silhouette reads as a real shell mill rather than a bare inverted-T.
    draw_face_inserts: bool,
}

impl ToolCanvas {
    fn new(tool: &cam_model::Tool) -> Self {
        let profile = tool.profile();
        // The flute helix spans the actual cutting region — the flute length for an end
        // mill, the cone for a V-bit / drill — read off the profile's cutting segments.
        let cutting_top = profile
            .segs
            .iter()
            .filter(|s| s.cutting)
            .map(|s| s.end.y)
            .fold(0.0_f64, f64::max);
        let cutting_top = cutting_top.max(1e-3);
        Self {
            diameter: tool.diameter,
            length: tool.length,
            flute_length: cutting_top,
            flutes: tool.flutes,
            cutting_direction: tool.cutting_direction,
            // V-bits, chamfer mills, face mills and thread mills show no flute helix (the
            // cone / shell-mill body / thread teeth read cleaner without it); a drill's
            // helix revolves every ~3·⌀.
            draw_flutes: !matches!(
                tool.kind,
                ToolKind::VBit { .. }
                    | ToolKind::ChamferMill { .. }
                    | ToolKind::FaceMill
                    | ToolKind::ThreadMill { .. }
            ),
            flute_pitch: match tool.kind {
                ToolKind::Drill { .. } => (3.0 * tool.diameter).max(1e-3),
                _ => cutting_top,
            },
            flute_sign: {
                let base = if tool.cutting_direction == CutDir::Up { 1.0 } else { -1.0 };
                // A twist drill leans the opposite way to a down-cut end mill.
                if matches!(tool.kind, ToolKind::Drill { .. }) { -base } else { base }
            },
            draw_face_inserts: matches!(tool.kind, ToolKind::FaceMill),
            profile,
        }
    }
}

impl canvas::Program<Message> for ToolCanvas {
    type State = ();

    fn draw(
        &self,
        _state: &(),
        renderer: &iced::Renderer,
        theme: &iced::Theme,
        bounds: iced::Rectangle,
        _cursor: iced::mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        use iced::Point as P;
        let mut frame = canvas::Frame::new(renderer, bounds.size());
        let fg = theme.palette().text;
        let faint = Color { a: 0.4, ..fg };

        let max_r = self.profile.max_radius().max(1e-3) as f32;
        let height = self.profile.height().max(1e-3) as f32;
        let margin = 28.0_f32;
        let avail_w = (bounds.width - 2.0 * margin).max(1.0);
        let avail_h = (bounds.height - 2.0 * margin).max(1.0);
        // Fit the *mirrored* silhouette (width 2·max_r) and the height, keeping aspect.
        let scale = (avail_w / (2.0 * max_r)).min(avail_h / height);
        let cx = bounds.width / 2.0;
        let bottom = bounds.height - margin;
        let map = |r: f64, z: f64, sign: f32| P::new(cx + sign * (r as f32) * scale, bottom - (z as f32) * scale);

        // Axis of revolution: a faint dashed vertical line spanning the whole viewport
        // (top to bottom), not just the tool's extent — a full symmetry reference.
        let axis_dash = [4.0_f32, 4.0];
        let axis = canvas::Path::line(P::new(cx, 0.0), P::new(cx, bounds.height));
        frame.stroke(
            &axis,
            canvas::Stroke {
                line_dash: canvas::LineDash { segments: &axis_dash, offset: 0 },
                ..canvas::Stroke::default().with_color(faint).with_width(1.0)
            },
        );

        // The generatrix, per segment, mirrored to both sides.
        let dash = [5.0_f32, 4.0];
        for (pts, cutting) in self.profile.segment_polylines(0.05) {
            for sign in [1.0_f32, -1.0] {
                let path = canvas::Path::new(|b| {
                    let mut first = true;
                    for p in &pts {
                        let sp = map(p.x, p.y, sign);
                        if first {
                            b.move_to(sp);
                            first = false;
                        } else {
                            b.line_to(sp);
                        }
                    }
                });
                let stroke = if cutting {
                    canvas::Stroke::default().with_color(fg).with_width(2.2)
                } else {
                    canvas::Stroke {
                        line_dash: canvas::LineDash { segments: &dash, offset: 0 },
                        ..canvas::Stroke::default().with_color(fg).with_width(1.2)
                    }
                };
                frame.stroke(&path, stroke);
            }
        }

        // Face-mill inserts: a row of small 90° squares seated on the bottom face, their
        // outer edges defining the cutting ⌀ — turns the bare body+arbor silhouette into a
        // recognisable shell mill. (90° square inserts only; other insert shapes deferred.)
        if self.draw_face_inserts {
            let r = self.profile.max_radius();
            let body_h = self.flute_length.max(1e-3);
            let n = self.flutes.max(1);
            // Insert side: a small glyph, never taller than the body (min-then-max avoids
            // clamp's panic when the body is shorter than the floor).
            let s = (r * 0.22).min(body_h * 0.6).max(0.1);
            let insert_fill = Color { a: 0.30, ..fg };
            for i in 0..n {
                // Centres span the diameter; the two end inserts sit at ±(r − s/2) so
                // their outer edge lands exactly on the periphery (±r).
                let cxi = if n == 1 {
                    0.0
                } else {
                    -(r - s / 2.0) + (i as f64) * (2.0 * (r - s / 2.0)) / ((n - 1) as f64)
                };
                let (x0, x1) = (cxi - s / 2.0, cxi + s / 2.0);
                let rect = canvas::Path::new(|b| {
                    b.move_to(map(x0, 0.0, 1.0));
                    b.line_to(map(x1, 0.0, 1.0));
                    b.line_to(map(x1, s, 1.0));
                    b.line_to(map(x0, s, 1.0));
                    b.close();
                });
                frame.fill(&rect, insert_fill);
                frame.stroke(&rect, canvas::Stroke::default().with_color(fg).with_width(1.2));
            }
        }

        // (Thread-mill teeth are part of the generatrix now — the saw-tooth is the cutting
        // boundary itself — so there is no separate overlay: the profile loop above draws
        // the threads solid and the non-cutting bottom/shank dashed.)

        // Flutes + cutting direction. Each flute is projected onto the side view, drawn
        // only where it faces the viewer, and **tapered to the tool's actual radius at
        // that height** (`radius_at`) so on a ball / rounded nose the flutes converge at
        // the tip instead of running full width. Count the lines for the flute number; a
        // helical lean shows the cutting direction (Down vs Up), while straight (axial)
        // flutes read as vertical. (Helix pitch = one turn over the flute length is a
        // legibility choice, not the true helix angle.)
        use std::f64::consts::{PI, TAU};
        let boundary = self.profile.polyline(0.05);
        let radius_at = |z: f64| -> f64 {
            let mut best = 0.0_f64;
            for w in boundary.windows(2) {
                let (a, b) = (w[0], w[1]);
                if z + 1e-9 < a.y.min(b.y) || z - 1e-9 > a.y.max(b.y) {
                    continue;
                }
                let r = if (b.y - a.y).abs() < 1e-9 {
                    a.x.max(b.x)
                } else {
                    let t = ((z - a.y) / (b.y - a.y)).clamp(0.0, 1.0);
                    a.x + t * (b.x - a.x)
                };
                best = best.max(r);
            }
            best
        };
        if self.draw_flutes {
            let n = self.flutes.max(1);
            let flute_len = self.flute_length.max(1e-3);
            let pitch = self.flute_pitch.max(1e-3);
            let flute_col = Color { a: 0.75, ..fg };
            // More samples when the helix wraps more times (keeps a tight drill smooth).
            let turns = (flute_len / pitch).max(1.0);
            let steps = ((48.0 * turns).ceil() as usize).clamp(48, 240);

            let mut flute_paths: Vec<Vec<P>> = Vec::new();
            match self.cutting_direction {
                CutDir::Straight => {
                    // ~N/2 visible front flutes, spread across the width, tapering to the tip.
                    let front = n.div_ceil(2).max(1);
                    for j in 0..front {
                        let theta = ((j as f64 + 0.5) / front as f64) * PI - PI / 2.0;
                        let pts = (0..=steps)
                            .map(|i| {
                                let z = flute_len * (i as f64) / (steps as f64);
                                map(radius_at(z) * theta.sin(), z, 1.0)
                            })
                            .collect();
                        flute_paths.push(pts);
                    }
                }
                _ => {
                    let s = self.flute_sign;
                    for k in 0..n {
                        let psi0 = (k as f64) * TAU / (n as f64);
                        let mut seg: Vec<P> = Vec::new();
                        for i in 0..=steps {
                            let z = flute_len * (i as f64) / (steps as f64);
                            let psi = psi0 + s * TAU * (z / pitch);
                            if psi.cos() > 0.0 {
                                seg.push(map(radius_at(z) * psi.sin(), z, 1.0));
                            } else if seg.len() >= 2 {
                                flute_paths.push(std::mem::take(&mut seg));
                            } else {
                                seg.clear();
                            }
                        }
                        if seg.len() >= 2 {
                            flute_paths.push(seg);
                        }
                    }
                }
            }
            for pts in flute_paths {
                if pts.len() < 2 {
                    continue;
                }
                let path = canvas::Path::new(|b| {
                    b.move_to(pts[0]);
                    for p in &pts[1..] {
                        b.line_to(*p);
                    }
                });
                frame.stroke(&path, canvas::Stroke::default().with_color(flute_col).with_width(1.0));
            }
        }

        // Dimension annotations (text only — no scale bars, kept minimal).
        let label = |frame: &mut canvas::Frame, s: String, y: f32| {
            frame.fill_text(canvas::Text {
                content: s,
                position: P::new(8.0, y),
                color: fg,
                size: iced::Pixels(12.0),
                ..canvas::Text::default()
            });
        };
        label(&mut frame, format!("⌀ {:.3} mm", self.diameter), 6.0);
        label(&mut frame, format!("length {:.1} mm", self.length), 22.0);
        if self.draw_flutes {
            label(&mut frame, format!("flute {:.1} mm", self.flute_length), 38.0);
            label(
                &mut frame,
                format!(
                    "{} flute{}, {}",
                    self.flutes,
                    if self.flutes == 1 { "" } else { "s" },
                    self.cutting_direction
                ),
                54.0,
            );
        }

        vec![frame.into_geometry()]
    }
}

#[cfg(test)]
mod origin_move_tests {
    use super::origin_move_targets;

    #[test]
    fn a_single_origin_job_offers_nowhere_to_move() {
        // What keeps the menu section off an ordinary job: with one origin there is no
        // destination, and a "Move to Origin 1" row on the operation already in origin 1
        // would be a control that does nothing.
        assert!(origin_move_targets(&[1], Some(1), 1).is_empty());
    }

    #[test]
    fn every_other_origin_is_offered_in_tree_order() {
        // Order matters: the menu reads down the same sequence as the project tree, so
        // "the third one" means the same thing in both.
        assert_eq!(origin_move_targets(&[1, 2, 3], Some(2), 1), vec![1, 3]);
        assert_eq!(origin_move_targets(&[1, 2, 3], Some(1), 1), vec![2, 3]);
    }

    #[test]
    fn a_raised_base_is_not_assumed_to_be_origin_one() {
        // The base origin's `H<n>` is editable, so the indices need not start at 1 and
        // "the base" is whatever the setup says. Hard-coding 1 anywhere here would offer
        // an operation a move to the group it is already in.
        assert_eq!(origin_move_targets(&[4, 5], Some(4), 4), vec![5]);
        assert_eq!(origin_move_targets(&[4, 5], Some(5), 4), vec![4]);
    }

    #[test]
    fn an_operation_pointing_at_a_deleted_origin_counts_as_being_on_the_base() {
        // Deleting an origin leaves its operations carrying an index that no longer
        // exists; the tree draws them under the base. If that stale index were read
        // literally the base would be offered as a destination for a row already shown
        // under the base — a move that appears to do nothing.
        assert_eq!(origin_move_targets(&[1, 3], Some(2), 1), vec![3]);
        // Same for an operation with no origin recorded at all.
        assert_eq!(origin_move_targets(&[1, 3], None, 1), vec![3]);
    }
}

/// What is left of the migration's scaffolding.
///
/// This module existed to assert that `Settings`'s defaults equalled the constants the
/// GUI actually used, while the two coexisted. They no longer do: `PICKBOX_PX`,
/// `SNAP_PICK_PX`, `SNAP_MARK_SCALE`, `GIZMO_SIZE_DEFAULT` and `Pane::min_size`'s
/// hard-coded match are all gone, and `Settings` is the only place those values are
/// written. **A cross-check deleted because the duplication went away is the good
/// outcome** — the remaining one guards a value that genuinely still lives in two
/// places, because the slider needs literal bounds.
#[cfg(test)]
mod settings_agree_with_constants {
    use super::*;

    #[test]
    fn the_gizmo_slider_range_is_the_preferences_range() {
        assert_eq!(
            crate::GIZMO_SIZE_RANGE,
            (GIZMO_SIZE_MIN, GIZMO_SIZE_MAX),
            "the preference's range must be the slider's range"
        );
    }

    /// Every pane must take its minimum from preferences — a `min_size` arm that
    /// ignored `prefs` and returned a constant would silently pin one pane, and the
    /// symptom (one divider that will not go past a size) reads as a layout quirk
    /// rather than a bug.
    #[test]
    fn every_pane_reads_its_minimum_from_the_preferences() {
        let mut prefs = crate::PanePrefs {
            min_project_px: 111.0,
            min_library_px: 122.0,
            min_viewport_px: 133.0,
            min_inspector_px: 144.0,
            min_output_px: 155.0,
            ..Default::default()
        };
        for (pane, want) in [
            (Pane::Project, 111.0),
            (Pane::Library, 122.0),
            (Pane::Viewport, 133.0),
            (Pane::Inspector, 144.0),
            (Pane::Output, 155.0),
        ] {
            assert_eq!(pane.min_size(&prefs), want, "{pane:?} ignored the preference");
        }
        // And they must not be reading a *shared* field either.
        prefs.min_inspector_px = 999.0;
        assert_eq!(Pane::Project.min_size(&prefs), 111.0);
        assert_eq!(Pane::Inspector.min_size(&prefs), 999.0);
    }
}

#[cfg(test)]
mod inspector_field_tests {
    use super::*;
    use cam_model::{DrillOp, Tool};

    #[test]
    fn new_pointed_tools_seed_a_physically_real_tip() {
        // Ground truth: neither tool has a true r=0 point — one cannot be ground and
        // would not survive contact. The kinds differ by what the tip *is*: a chamfer
        // mill's flat does not cut (so it cannot engrave), a V-bit's rounded tip does.
        // A zero on either would collapse them into the same geometry.
        match ToolKindPick::ChamferMill.to_kind() {
            ToolKind::ChamferMill { tip_diameter, .. } => {
                assert!(
                    tip_diameter >= 2.0 * MIN_TIP_RADIUS_MM,
                    "a chamfer mill is ground with a flat, at the physical floor"
                );
            }
            other => panic!("{other:?}"),
        }
        match ToolKindPick::VBit.to_kind() {
            ToolKind::VBit { tip_radius, .. } => {
                assert!(
                    tip_radius >= MIN_TIP_RADIUS_MM,
                    "a V-bit's point is ground to a radius, at the physical floor"
                );
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_seeded_chamfer_mill_cannot_engrave_but_a_seeded_vbit_can() {
        // The seeds must land on the right side of the engraving guard: this is the
        // whole practical consequence of flat-vs-rounded.
        let cham = Tool {
            number: 1,
            diameter: 6.0,
            length: 40.0,
            flutes: 2,
            kind: ToolKindPick::ChamferMill.to_kind(),
            ..Default::default()
        };
        let vbit = Tool {
            kind: ToolKindPick::VBit.to_kind(),
            ..cham
        };
        assert!(!cham.profile().has_cutting_tip(), "flat tip must not cut");
        assert!(vbit.profile().has_cutting_tip(), "rounded tip must cut");
    }

    // --- the Apply button's dirty rule ---

    #[test]
    fn an_untouched_inspector_is_not_dirty() {
        // The whole point: Apply greys out until something is actually pending, so the
        // operator can see at a glance what has been committed and what has not.
        let visible = [Field::Depth, Field::Feed, Field::Stepdown];
        let model = |f: Field| match f {
            Field::Depth => Some(4.0),
            Field::Feed => Some(300.0),
            Field::Stepdown => Some(2.0),
            _ => None,
        };
        let mut buffers = BTreeMap::new();
        for f in visible {
            buffers.insert(f, fmt_num(model(f).unwrap()));
        }
        assert!(!fields_are_dirty(&visible, &buffers, model));

        // Retyping the same number in a different but equal form is still not a change.
        buffers.insert(Field::Depth, "4.000".to_string());
        assert!(!fields_are_dirty(&visible, &buffers, model));
    }

    #[test]
    fn a_changed_field_makes_the_inspector_dirty() {
        let visible = [Field::Depth, Field::Feed];
        let model = |f: Field| match f {
            Field::Depth => Some(4.0),
            Field::Feed => Some(300.0),
            _ => None,
        };
        let mut buffers = BTreeMap::new();
        buffers.insert(Field::Depth, "4".to_string());
        buffers.insert(Field::Feed, "300".to_string());
        assert!(!fields_are_dirty(&visible, &buffers, model));
        buffers.insert(Field::Depth, "4.5".to_string());
        assert!(fields_are_dirty(&visible, &buffers, model));
    }

    #[test]
    fn half_typed_counts_as_dirty_but_an_unseeded_field_does_not() {
        let visible = [Field::Depth, Field::Feed];
        let model = |f: Field| match f {
            Field::Depth => Some(4.0),
            Field::Feed => Some(300.0),
            _ => None,
        };
        // Mid-typing something that is not yet a number ("-", "") is a pending edit.
        // Apply stays disabled regardless, via `any_field_invalid`.
        let mut buffers = BTreeMap::new();
        buffers.insert(Field::Depth, "-".to_string());
        assert!(fields_are_dirty(&visible, &buffers, model));

        // But "4." is not half-typed, it is four: Rust parses it, and the comparison is
        // numeric, so it correctly reads as no change at all.
        buffers.insert(Field::Depth, "4.".to_string());
        assert!(!fields_are_dirty(&visible, &buffers, model));

        // A field the model exposes but that has no buffer yet -- a control just
        // revealed by a picker -- is not an edit. Nothing has been typed into it.
        let buffers = BTreeMap::new();
        assert!(!fields_are_dirty(&visible, &buffers, model));
    }

    #[test]
    fn a_field_the_model_has_no_value_for_is_ignored() {
        // A field the model returns `None` for (e.g. not applicable to the current
        // selection) with a stale buffer must not light Apply up forever.
        let visible = [Field::OriginX];
        let mut buffers = BTreeMap::new();
        buffers.insert(Field::OriginX, "12".to_string());
        assert!(!fields_are_dirty(&visible, &buffers, |_| None));
    }

    fn drill(peck: Option<f64>, dwell: Option<f64>) -> Operation {
        Operation::Drill(DrillOp {
            spindle_rpm: 0.0,
            work_offset: 1,
            id: 1,
            tool: 1,
            points: vec![[0.0, 0.0]],
            depth: 10.0,
            start_offset: 0.0,
            peck,
            dwell,
            feed: 100.0,
        })
    }

    #[test]
    fn drill_exposes_peck_and_dwell_fields() {
        // This used to hand-build the list it then asserted on, so it could only ever
        // agree with itself. It now asks the inspector's own rule.
        let fields = operation_fields(&drill(None, None));
        assert!(fields.contains(&Field::Peck));
        assert!(fields.contains(&Field::Dwell));
    }

    fn chamfer(top: f64) -> Operation {
        Operation::Chamfer(cam_model::ChamferOp {
            id: 1,
            tool: 1,
            chain: cam_geo::Contour::new(vec![
                cam_geo::Point::new(0.0, 0.0),
                cam_geo::Point::new(10.0, 0.0),
                cam_geo::Point::new(10.0, 10.0),
                cam_geo::Point::new(0.0, 10.0),
            ]),
            side: Side::Outside,
            width: 1.0,
            top,
            depth: 0.0,
            step: 0.0,
            gradual: false,
            spindle_rpm: 0.0,
            work_offset: 1,
            feed: 200.0,
            plunge_feed: 100.0,
            start: None,
            lead_in: Lead::None,
            lead_out: Lead::None,
            lead_overlap: 0.0,
        })
    }

    #[test]
    fn a_chamfers_top_edge_is_editable_and_may_sit_below_the_datum() {
        // `top` was seeded from the stock top and then unreachable, which is only ever
        // right for an edge on the raw surface. The rim of a pocket, or a step an
        // earlier operation cut, is the ordinary case that had no way to be stated.
        let mut op = chamfer(3.0);
        assert!(
            operation_fields(&op).contains(&Field::ChamferTop),
            "the inspector must show the top edge"
        );
        assert_eq!(op_field(&op, Field::ChamferTop), Some(3.0));

        // Deliberately unclamped: an edge below the part datum is a value, not a slip.
        let mut parsed = BTreeMap::new();
        parsed.insert(Field::ChamferTop, -4.5);
        apply_op_fields(&mut op, &parsed);
        assert_eq!(op_field(&op, Field::ChamferTop), Some(-4.5));
        let Operation::Chamfer(c) = &op else {
            unreachable!()
        };
        assert_eq!(c.top, -4.5, "a negative top must survive to the model");
    }

    /// One operation of every kind, built the way the app builds them.
    fn one_of_every_kind() -> Vec<Operation> {
        let mut app = AppController::new(crate::default_machine());
        app.open_dxf(SAMPLE_DXF, "sample.dxf").expect("sample loads");
        for kind in [
            OpKind::Profile,
            OpKind::Pocket,
            OpKind::Face,
            OpKind::Drill,
            OpKind::Thread,
            OpKind::Chamfer,
            OpKind::Engrave,
            OpKind::Carve,
        ] {
            app.new_operation(kind);
        }
        app.document().setup.operations.clone()
    }

    #[test]
    fn every_field_the_inspector_shows_can_be_written_back() {
        // The gap this closes: a `Field` can be declared, labelled, given a tooltip and
        // listed by `operation_fields`, and still have no arm in `op_field` or
        // `apply_op_fields` — in which case the box renders, accepts typing, and springs
        // back on Apply. Nothing else in the crate calls either function, so only this
        // catches it. Asserted per operation kind, since the arms are per kind.
        let ops = one_of_every_kind();
        assert!(ops.len() >= 6, "the sample part must yield operations to check");
        for op in &ops {
            for field in operation_fields(op) {
                let Some(before) = op_field(op, field) else {
                    panic!("{}: {field:?} is shown but cannot be read", op_kind(op));
                };
                // +1 clears every lower clamp in `apply_op_fields` (all are `max(0.0)`
                // or `max(1)`) without reaching the one upper clamp (overlap, 99%).
                let want = before + 1.0;
                let mut edited = op.clone();
                let mut parsed = BTreeMap::new();
                parsed.insert(field, want);
                apply_op_fields(&mut edited, &parsed);
                assert_eq!(
                    op_field(&edited, field),
                    Some(want),
                    "{}: {field:?} is shown but does not accept an edit ({before} -> {want})",
                    op_kind(op)
                );
            }
        }
    }

    #[test]
    fn a_carves_two_plunges_are_edited_separately() {
        // A carve is the only operation with **two** plunge styles — the V-bit's own
        // (v13) and the clearing end mill's — and they share one pair of parameter slots
        // split by prefix (`PlungeA` vs `ClearPlungeA`). Crossing them is a one-word
        // mistake, both sides are a `Plunge`, so nothing else would notice: the field
        // would read, write and round-trip perfectly, on the wrong tool.
        //
        // Not covered by `every_field_the_inspector_shows_can_be_written_back` either,
        // which only sees fields a *default* operation shows — and the default is
        // `Straight`, whose parameter boxes do not appear at all.
        let mut op = one_of_every_kind()
            .into_iter()
            .find(|o| matches!(o, Operation::Carve(_)))
            .expect("the sample part yields a carve");
        let Operation::Carve(c) = &mut op else {
            unreachable!()
        };
        c.plunge = Plunge::Ramp { angle_deg: 5.0 };
        c.clear = Some(CarveClearing {
            tool: 2,
            params: ClearParams {
                plunge: Plunge::Helix {
                    radius: 1.0,
                    pitch: 0.5,
                },
                ..Default::default()
            },
        });

        let fields = operation_fields(&op);
        assert!(fields.contains(&Field::PlungeA), "the V-bit's ramp angle is not shown");
        assert!(
            fields.contains(&Field::ClearPlungeA),
            "the clearing helix radius is not shown"
        );
        assert_eq!(op_field(&op, Field::PlungeA), Some(5.0), "the V-bit's own angle");
        assert_eq!(
            op_field(&op, Field::ClearPlungeA),
            Some(1.0),
            "the clearing pass's own radius"
        );

        // Editing one must leave the other exactly as it was.
        let mut parsed = BTreeMap::new();
        parsed.insert(Field::PlungeA, 12.0);
        apply_op_fields(&mut op, &parsed);
        let Operation::Carve(c) = &op else {
            unreachable!()
        };
        assert_eq!(c.plunge, Plunge::Ramp { angle_deg: 12.0 });
        assert_eq!(
            c.clear.expect("the clearing pass").params.plunge,
            Plunge::Helix {
                radius: 1.0,
                pitch: 0.5
            },
            "editing the V-bit's plunge moved the clearing pass's"
        );
    }

    #[test]
    fn peck_and_dwell_round_trip_with_a_zero_off_sentinel() {
        // Set: an on-value writes Some; then the same read shows it back.
        let mut op = drill(None, None);
        let mut parsed = BTreeMap::new();
        parsed.insert(Field::Peck, 2.5);
        parsed.insert(Field::Dwell, 0.75);
        apply_op_fields(&mut op, &parsed);
        assert_eq!(op_field(&op, Field::Peck), Some(2.5));
        assert_eq!(op_field(&op, Field::Dwell), Some(0.75));
        if let Operation::Drill(o) = &op {
            assert_eq!(o.peck, Some(2.5));
            assert_eq!(o.dwell, Some(0.75));
        }

        // 0 clears both back to None (off), and a disabled field reads as 0.
        let mut off = BTreeMap::new();
        off.insert(Field::Peck, 0.0);
        off.insert(Field::Dwell, 0.0);
        apply_op_fields(&mut op, &off);
        assert_eq!(op_field(&op, Field::Peck), Some(0.0));
        assert_eq!(op_field(&op, Field::Dwell), Some(0.0));
        if let Operation::Drill(o) = &op {
            assert_eq!(o.peck, None, "0 must clear peck to None (toolpath needs peck>0)");
            assert_eq!(o.dwell, None);
        }
    }

    fn drilled(peck: Option<f64>, dwell: Option<f64>) -> Program {
        use cam_cldata::{DrillCycle, MoveKind, Tag};
        let mut prog = Program::new();
        prog.push(Step::Drill(DrillCycle {
            points: vec![[0.0, 0.0], [5.0, 0.0]],
            z_top: 0.0,
            depth: -6.0,
            retract: 2.0,
            peck,
            dwell,
            feed: 100.0,
            tag: Tag::new(1, MoveKind::Plunge),
        }));
        prog
    }

    #[test]
    fn peck_rings_land_on_intermediate_depths_only_never_the_bottom() {
        // depth 6, peck 2 ⇒ retracts at -2 and -4; the -6 bottom is not a peck.
        let marks = drill_marks_of(&drilled(Some(2.0), None));
        let rings: Vec<f32> = marks
            .iter()
            .filter(|m| m.kind == DrillMarkKind::PeckRing && m.at[0] == 0.0)
            .map(|m| m.at[2])
            .collect();
        assert_eq!(rings, vec![-2.0, -4.0], "one ring per intermediate peck");
        assert!(
            !marks.iter().any(|m| m.at[2] <= -6.0),
            "no mark sits at or below the bottom for a non-dwelling hole"
        );
        // Two holes ⇒ the rings are mirrored at the second point.
        assert_eq!(
            marks.iter().filter(|m| m.kind == DrillMarkKind::PeckRing).count(),
            4
        );
    }

    #[test]
    fn a_dwelling_hole_gets_one_bar_at_the_exact_bottom_per_point() {
        let marks = drill_marks_of(&drilled(None, Some(0.5)));
        let bars: Vec<[f32; 3]> = marks
            .iter()
            .filter(|m| m.kind == DrillMarkKind::DwellBar)
            .map(|m| m.at)
            .collect();
        assert_eq!(bars, vec![[0.0, 0.0, -6.0], [5.0, 0.0, -6.0]]);
        assert!(!marks.iter().any(|m| m.kind == DrillMarkKind::PeckRing));
    }

    #[test]
    fn a_plain_hole_has_no_annotations() {
        assert!(drill_marks_of(&drilled(None, None)).is_empty());
    }

    #[test]
    fn a_peck_bigger_than_the_hole_rings_nothing() {
        // First peck already reaches the bottom ⇒ no intermediate retract to mark.
        let marks = drill_marks_of(&drilled(Some(20.0), None));
        assert!(marks.is_empty());
    }

    #[test]
    fn a_negative_peck_is_treated_as_off_not_stored() {
        let mut op = drill(Some(1.0), None);
        let mut parsed = BTreeMap::new();
        parsed.insert(Field::Peck, -3.0);
        apply_op_fields(&mut op, &parsed);
        if let Operation::Drill(o) = &op {
            assert_eq!(o.peck, None, "a negative peck must not reach the toolpath");
        }
    }
}

#[cfg(test)]
mod ribbon_tests {
    use super::*;

    #[test]
    fn families_offered_match_what_each_operation_can_actually_cut() {
        use ToolKindPick as F;
        // The agreed table. Narrower than "whatever the guards would not reject":
        // no ball-nose for facing, no face mill for profile/pocket, no end mill for
        // drilling — those are possible but deliberately not offered.
        assert_eq!(families_for(OpKind::Profile), &[F::EndMill, F::BallMill, F::BullNose]);
        assert_eq!(families_for(OpKind::Pocket), &[F::EndMill, F::BallMill, F::BullNose]);
        assert_eq!(families_for(OpKind::Face), &[F::EndMill, F::BullNose, F::FaceMill]);
        assert_eq!(families_for(OpKind::Drill), &[F::Drill]);
        assert_eq!(families_for(OpKind::Thread), &[F::ThreadMill]);
        assert_eq!(families_for(OpKind::Chamfer), &[F::ChamferMill, F::VBit]);
        assert_eq!(families_for(OpKind::Engrave), &[F::VBit]);
        // Carving is a V-bit operation for the same reason engraving is: the cut's
        // shape is the tool's own cone.
        assert_eq!(families_for(OpKind::Carve), &[F::VBit]);
    }

    #[test]
    fn every_bundled_icon_is_well_formed_svg() {
        // carve.svg once shipped with "--" inside an XML comment, which XML forbids.
        // The renderer does not complain: it simply draws nothing, so the button came
        // out blank and no test, build or clippy run had a word to say about it.
        // Parse every icon so a malformed one fails loudly instead of silently.
        for icon in Icon::ALL {
            let bytes = icon.bytes();
            let text = std::str::from_utf8(bytes).expect("an icon must be UTF-8");
            let doc = roxmltree::Document::parse(text)
                .unwrap_or_else(|e| panic!("{icon:?} is not well-formed SVG: {e}"));
            assert_eq!(
                doc.root_element().tag_name().name(),
                "svg",
                "{icon:?} is not an SVG"
            );
        }
    }

    #[test]
    fn every_svg_in_the_assets_tree_is_well_formed() {
        // The sweep above only covers what `Icon::ALL` names. The application icon is
        // not a ribbon icon, so it is in no enum and nothing in the code references
        // it -- it is read by the packaging workflows instead, where a malformed file
        // would surface as a blank icon on a shipped AppImage rather than as a test
        // failure. So walk the directory rather than the enum: an asset is checked
        // because it exists, not because some code happens to mention it.
        //
        // NB this proves well-formedness and nothing more. Parsing would not have
        // caught either of the other two ways this icon's SVG went wrong (rsvg
        // ignoring `textLength`, and `dominant-baseline` honoured by rsvg but not by
        // desktop viewers) -- both parse perfectly and simply render differently.
        fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for entry in std::fs::read_dir(dir).expect("assets dir must be readable") {
                let path = entry.expect("readable entry").path();
                if path.is_dir() {
                    walk(&path, out);
                } else if path.extension().is_some_and(|e| e == "svg") {
                    out.push(path);
                }
            }
        }
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("assets");
        let mut svgs = Vec::new();
        walk(&root, &mut svgs);
        assert!(
            svgs.len() > Icon::ALL.len(),
            "expected the ribbon icons plus at least the application icon, found {}",
            svgs.len()
        );
        for path in &svgs {
            let text = std::fs::read_to_string(path)
                .unwrap_or_else(|e| panic!("{}: unreadable: {e}", path.display()));
            let doc = roxmltree::Document::parse(&text)
                .unwrap_or_else(|e| panic!("{}: not well-formed SVG: {e}", path.display()));
            assert_eq!(
                doc.root_element().tag_name().name(),
                "svg",
                "{}: root element is not <svg>",
                path.display()
            );
        }
    }

    #[test]
    fn the_application_icon_carries_its_author_and_licence() {
        // The app icon is the project's one deliberate departure from GPL-3.0-only
        // (CC BY-SA 4.0, so the mark can live on Wikimedia). That exception is only
        // worth anything if the claim actually travels with the file, and the file
        // goes through editors -- Inkscape rewrote it once already -- so pin it.
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("assets")
            .join("opencamstudio.svg");
        let text = std::fs::read_to_string(&path).expect("the application icon must exist");
        let doc = roxmltree::Document::parse(&text).expect("well-formed");
        assert!(
            text.contains("Andreas Bertsatos"),
            "the application icon must name its author"
        );
        assert!(
            text.contains("creativecommons.org/licenses/by-sa/4.0/"),
            "the application icon must declare CC BY-SA 4.0"
        );
        // Outlines, not live text: a <text> element would make the rendered icon
        // depend on whichever font the packaging runner happens to have installed,
        // and Linux, macOS and Windows would each resolve it differently.
        assert!(
            !doc.descendants().any(|n| n.has_tag_name("text")),
            "the application icon must contain no live <text>; convert glyphs to paths"
        );
    }

    #[test]
    fn every_icon_is_listed_in_all() {
        // A cheap guard on the sweep above: if a variant is added without extending
        // ALL, the icon it names goes unchecked.
        let mut seen: Vec<&[u8]> = Icon::ALL.iter().map(|i| i.bytes()).collect();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), Icon::ALL.len(), "two variants share one asset");
    }

    #[test]
    fn no_operation_offers_a_family_its_strategy_would_reject() {
        use ToolKindPick as F;
        // The hard floor: engraving needs a cutting tip, threading needs a thread
        // mill, and side-milling needs a cylindrical flank. Offering any of these
        // would hand the user a combination that cannot produce G-code.
        assert!(!families_for(OpKind::Engrave).contains(&F::ChamferMill));
        for op in [OpKind::Profile, OpKind::Pocket] {
            assert!(!families_for(op).contains(&F::VBit), "{op:?}");
            assert!(!families_for(op).contains(&F::ChamferMill), "{op:?}");
            assert!(!families_for(op).contains(&F::Drill), "{op:?}");
            assert!(!families_for(op).contains(&F::ThreadMill), "{op:?}");
        }
        assert_eq!(families_for(OpKind::Thread), &[F::ThreadMill]);
        // Facing must not offer a tool whose tip does not cut.
        assert!(!families_for(OpKind::Face).contains(&F::ChamferMill));
    }

    #[test]
    fn every_operation_offers_at_least_one_family() {
        for op in [
            OpKind::Profile,
            OpKind::Pocket,
            OpKind::Face,
            OpKind::Drill,
            OpKind::Thread,
            OpKind::Chamfer,
            OpKind::Engrave,
            OpKind::Carve,
        ] {
            assert!(!families_for(op).is_empty(), "{op:?} offers nothing");
        }
    }

    #[test]
    fn tabs_read_left_to_right_in_workflow_order() {
        // Home -> Edit -> Operations -> Tooling -> Machinery -> View: set up, then edit,
        // then cut, then tools, then machines, then look. Windows is gone — its pane
        // toggles live in View.
        //
        // **Machinery sits beside Tooling, not in Edit.** Both are *installation* scope:
        // neither a machine nor the tool library is stored in a project, and a machine
        // deliberately cannot be set by one (a file that could would disarm the travel
        // check). Edit is the document — which is why Machine was in the wrong place.
        let labels: Vec<&str> = RibbonTab::ALL.iter().map(|t| t.label()).collect();
        assert_eq!(
            labels,
            vec!["Home", "Edit", "Operations", "Tooling", "Machinery", "View"]
        );
    }

    #[test]
    fn the_tool_library_compacts_to_tools_in_the_ribbon_only() {
        // The ribbon band is tight, so the label is shortened there — but the pane's
        // own identity (its title bar, and anything keyed on the name) is unchanged.
        assert_eq!(Pane::Library.ribbon_label(), "Tools");
        assert_eq!(Pane::Library.name(), "Tool Library");
        // Every other pane reads the same in both places.
        for pane in ALL_PANES.into_iter().filter(|p| *p != Pane::Library) {
            assert_eq!(pane.ribbon_label(), pane.name(), "{pane:?}");
        }
    }

    #[test]
    fn the_panes_group_covers_every_pane_except_the_viewport() {
        // The viewport is always visible and deliberately has no toggle; everything
        // else must be reachable, so a pane added later cannot go missing.
        let toggled: Vec<Pane> = ALL_PANES
            .into_iter()
            .filter(|p| *p != Pane::Viewport)
            .collect();
        assert_eq!(toggled.len(), ALL_PANES.len() - 1);
        assert!(!toggled.contains(&Pane::Viewport));
        // Two balanced columns, so the block stays inside the ~70 px band.
        let split = toggled.len().div_ceil(2);
        assert_eq!(toggled.chunks(split).count(), 2, "must be two columns");
    }

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
