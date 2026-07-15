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

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use iced::widget::pane_grid::{self, PaneGrid};
use iced::widget::{
    button, checkbox, column, container, mouse_area, pick_list, row, scrollable, shader, text,
    text_input, Space,
};
use iced::{Alignment, Background, Border, Color, Element, Length, Padding};

use cam_model::{Envelope, Lead, Machine, Operation, Plunge, Point3, ToolKind};

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
    Face,
    NewTool,
    Duplicate,
    Delete,
    ShowStock,
    ResetView,
    ShowCube,
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
            Icon::Profile => include_bytes!("../assets/icons/offset.svg"),
            Icon::Pocket => include_bytes!("../assets/icons/pocket.svg"),
            Icon::Drill => include_bytes!("../assets/icons/drill.svg"),
            Icon::Face => include_bytes!("../assets/icons/face.svg"),
            Icon::NewTool => include_bytes!("../assets/icons/endmill.svg"),
            Icon::Duplicate => include_bytes!("../assets/icons/copy.svg"),
            Icon::Delete => include_bytes!("../assets/icons/erase.svg"),
            Icon::ShowStock => include_bytes!("../assets/icons/box3d.svg"),
            Icon::ResetView => include_bytes!("../assets/icons/zoom_ext.svg"),
            Icon::ShowCube => include_bytes!("../assets/icons/viewcube.svg"),
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

use crate::{AppController, OpKind, PendingOp, Selection};

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
    Tooling,
    View,
    Windows,
}

impl RibbonTab {
    /// The tabs shown in the strip, left to right.
    const ALL: [RibbonTab; 5] = [
        RibbonTab::Home,
        RibbonTab::Operations,
        RibbonTab::Tooling,
        RibbonTab::View,
        RibbonTab::Windows,
    ];

    fn label(self) -> &'static str {
        match self {
            RibbonTab::Home => "Home",
            RibbonTab::Operations => "Operations",
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

/// An editable inspector field, keyed independently of which node owns it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Field {
    Clearance,
    Retract,
    TopOfStock,
    ToolDiameter,
    ToolLength,
    Flutes,
    Depth,
    Stepdown,
    Stepover,
    Feed,
    PlungeFeed,
    /// Lead-in size (length for Linear, radius for Arc).
    LeadInSize,
    /// Lead-out size.
    LeadOutSize,
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
            Field::ToolDiameter => "Tool ⌀ (mm)",
            Field::ToolLength => "Length (mm)",
            Field::Flutes => "Flutes",
            Field::Depth => "Depth (mm)",
            Field::Stepdown => "Stepdown (mm)",
            Field::Stepover => "Stepover (mm)",
            Field::Feed => "Feed (mm/min)",
            Field::PlungeFeed => "Plunge feed (mm/min)",
            Field::LeadInSize => "Lead-in size (mm)",
            Field::LeadOutSize => "Lead-out size (mm)",
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

/// A tool as offered in the wizard's tool picker.
#[derive(Clone, Copy, PartialEq)]
struct ToolChoice {
    number: u32,
    diameter: f64,
}

impl std::fmt::Display for ToolChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "T{} ⌀{}", self.number, fmt_num(self.diameter))
    }
}

struct App {
    controller: AppController,
    panes: pane_grid::State<Pane>,
    /// Current window size, tracked so pane resizes can be clamped to per-pane
    /// pixel minimums (pane_grid works in ratios, not pixels).
    window: iced::Size,
    /// Whether the one-shot startup layout (Output shrunk to its minimum) has been
    /// re-applied against the real window size yet.
    did_initial_layout: bool,
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
    /// The orbit-camera orientation, owned here; clicking a cube face or dragging
    /// the viewport reports changes back as messages.
    view: ViewControls,
    /// Last known cursor position over the viewport (window coords), for drawing
    /// the pickbox while a geometry pick is pending.
    cursor: Option<iced::Point>,
    status: String,
}

/// The pickbox aperture, px — its half-size is the vertex-snap tolerance.
const PICKBOX_PX: f32 = 12.0;

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
    /// The chosen `.nc` path (`None` = cancelled).
    NcToExport(Option<PathBuf>),
    // --- Operation-creation wizard ---
    /// Begin creating an operation of `kind` (enter geometry-pick mode).
    BeginOp(OpKind),
    /// Change the pending operation's tool.
    SetPendingTool(u32),
    /// Cancel the pending operation.
    CancelOp,
    /// A world `(x, y)` picked in the viewport plus the pickbox aperture in world
    /// mm (completes a pending operation).
    PickWorld([f32; 2], f32),
    /// The cursor moved over the viewport (window coords) while a pick is pending.
    ViewportCursor(iced::Point),
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
            did_initial_layout: false,
            active_tab: RibbonTab::Home,
            open_group: None,
            fields: BTreeMap::new(),
            show_stock: false,
            show_gizmo: true,
            view: ViewControls::default(),
            cursor: None,
            status: "Open the sample part to begin.".to_string(),
        };
        app.refresh_fields();
        // Start with the Output console at its minimum height (using the assumed
        // window size; the first real resize event refines it).
        app.minimize_pane(Pane::Output);
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
                    self.refresh_fields();
                    self.status = format!("Imported {n} region(s).");
                    self.rerun();
                }
                Err(e) => self.status = format!("Import failed: {e}"),
            },
            Message::Select(selection) => {
                self.controller.select(selection);
                self.refresh_fields();
            }
            // Field edits only touch the local buffer; nothing is applied or
            // recomputed until Apply, so undo has one step per real change.
            Message::FieldChanged(field, value) => {
                self.fields.insert(field, value);
            }
            Message::Apply => self.apply_inspector(),
            Message::NewProject => {
                self.controller.new_project();
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
                        self.refresh_fields();
                        self.rerun();
                        format!("Imported {n} region(s) from {}.", path.display())
                    }
                    Err(e) => format!("Import failed: {e}"),
                };
            }
            Message::ExportNc => {
                return iced::Task::perform(
                    pick_save("G-code", "program.nc", &["nc"]),
                    Message::NcToExport,
                );
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
            Message::BeginOp(kind) => {
                self.controller.begin_operation(kind);
                self.refresh_fields();
                self.status = if self.controller.pending_op().is_some() {
                    "Pick a region in the viewport (or Cancel in the Inspector).".to_string()
                } else {
                    "Open a part and add a tool first.".to_string()
                };
            }
            Message::SetPendingTool(number) => self.controller.set_pending_tool(number),
            Message::CancelOp => {
                self.controller.cancel_operation();
                self.status = "Cancelled operation creation.".to_string();
            }
            Message::PickWorld(w, aperture) => {
                if self
                    .controller
                    .pick_operation_geometry([w[0] as f64, w[1] as f64], aperture as f64)
                {
                    self.cursor = None;
                    self.refresh_fields();
                    self.rerun();
                    self.status = "Operation created.".to_string();
                } else {
                    self.status = "No geometry there — click a vertex or region.".to_string();
                }
            }
            Message::ViewportCursor(p) => self.cursor = Some(p),
            Message::DuplicateOp => {
                self.controller.duplicate_selected_operation();
                self.refresh_fields();
                self.rerun();
            }
            Message::SetOpExcluded(id, excluded) => {
                self.controller.set_operation_excluded(id, excluded);
                self.rerun();
            }
            Message::DeleteOp => {
                self.controller.delete_selected_operation();
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
            Message::PaneResized(pane_grid::ResizeEvent { split, ratio }) => {
                let ratio = self.clamp_resize(split, ratio);
                self.panes.resize(split, ratio);
            }
            Message::PaneDragged(pane_grid::DragEvent::Dropped { pane, target }) => {
                self.panes.drop(pane, target);
            }
            Message::PaneDragged(_) => {}
            Message::WindowResized(size) => {
                self.window = size;
                // Re-apply the startup layout once now that the true window size
                // is known — then leave the user's manual sizing alone.
                if !self.did_initial_layout {
                    self.did_initial_layout = true;
                    self.minimize_pane(Pane::Output);
                }
            }
            Message::ToolKindChanged(kind) => {
                if let Selection::Tool(i) = self.controller.selection() {
                    self.controller.edit_tool(i, |t| t.kind = kind);
                    self.rerun();
                }
            }
            Message::LeadInKindChanged(kind) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Profile(p) = op {
                        p.lead_in = kind.to_lead(p.lead_in);
                    }
                });
                self.refresh_fields();
                self.rerun();
            }
            Message::LeadOutKindChanged(kind) => {
                self.controller.edit_selected_operation(|op| {
                    if let Operation::Profile(p) = op {
                        p.lead_out = kind.to_lead(p.lead_out);
                    }
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
            Message::NewTool => {
                self.controller.add_tool();
                self.refresh_fields();
            }
            Message::DeleteTool => {
                if let Selection::Tool(i) = self.controller.selection() {
                    self.controller.delete_tool(i);
                    self.refresh_fields();
                    self.rerun();
                }
            }
            Message::SelectRibbonTab(tab) => self.active_tab = tab,
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
        iced::window::resize_events().map(|(_id, size)| Message::WindowResized(size))
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

    /// Show or hide a pane. Hiding closes it (the others keep their sizes; the
    /// last remaining pane can't be closed). Showing splits it back off an
    /// existing pane and, if it has a fixed edge, docks it there.
    fn set_pane_visible(&mut self, pane: Pane, show: bool) {
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
                // Re-showing the Viewport reclaims the room: shrink every pane it
                // is split against to that pane's minimum so the view gets the rest.
                if pane == Pane::Viewport {
                    self.maximize_pane(Pane::Viewport);
                }
            }
            _ => {}
        }
    }

    /// Resize the splits along the path from the root to `pane` so that, at each
    /// one, the subtree *not* containing `pane` is squeezed to its minimum size —
    /// giving `pane` as much of the window as the other panes' minimums allow.
    /// Splits are walked outermost-first so a parent's dimension is settled before
    /// its children read it.
    fn maximize_pane(&mut self, pane: Pane) {
        let Some(handle) = self.pane_handle(pane) else {
            return;
        };
        let Some(path) = splits_to_pane(self.panes.layout(), handle) else {
            return;
        };
        for (split, in_a) in path {
            let bounds = iced::Size::new(
                self.window.width,
                (self.window.height - MENU_BAR_H).max(1.0),
            );
            let regions = self
                .panes
                .layout()
                .split_regions(PANE_SPACING, PANE_MIN_PX, bounds);
            let Some(&(axis, rect, _)) = regions.get(&split) else {
                continue;
            };
            let Some((a, b)) = find_split(self.panes.layout(), split) else {
                continue;
            };
            let dim = match axis {
                pane_grid::Axis::Vertical => rect.width,
                pane_grid::Axis::Horizontal => rect.height,
            };
            if dim <= 1.0 {
                continue;
            }
            // Shrink the sibling subtree (the one without `pane`) to its minimum.
            let ratio = if in_a {
                1.0 - subtree_min(b, &self.panes, axis) / dim
            } else {
                subtree_min(a, &self.panes, axis) / dim
            };
            let ratio = self.clamp_resize(split, ratio.clamp(0.0, 1.0));
            self.panes.resize(split, ratio);
        }
    }

    /// Resize the pane's immediate parent split so `pane` gets just its minimum
    /// size, handing the rest to its sibling. Used to seed the startup layout with
    /// a short Output console.
    fn minimize_pane(&mut self, pane: Pane) {
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
        let frac = pane.min_size() / dim;
        let ratio = if in_a { frac } else { 1.0 - frac };
        let ratio = self.clamp_resize(split, ratio.clamp(0.0, 1.0));
        self.panes.resize(split, ratio);
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

    /// Which fields the inspector shows for the current selection.
    fn inspector_fields(&self) -> Vec<Field> {
        match self.controller.selection() {
            Selection::Setup => vec![Field::Clearance, Field::Retract, Field::TopOfStock],
            Selection::Tool(_) => vec![Field::ToolDiameter, Field::ToolLength, Field::Flutes],
            Selection::Stock => Vec::new(),
            Selection::Operation(id) => match self.controller.operation(id) {
                Some(Operation::Profile(p)) => {
                    let mut fields = vec![
                        Field::Depth,
                        Field::Stepdown,
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
                Some(Operation::Face(_)) => vec![Field::Depth, Field::Stepdown, Field::Feed],
                Some(Operation::Drill(_)) => vec![Field::Depth, Field::Feed],
                None => Vec::new(),
            },
        }
    }

    /// The model value backing a field for the current selection, if any.
    fn field_value(&self, field: Field) -> Option<f64> {
        let setup = &self.controller.document().setup;
        match field {
            Field::Clearance => Some(setup.heights.clearance),
            Field::Retract => Some(setup.heights.retract),
            Field::TopOfStock => Some(setup.heights.top_of_stock),
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
            _ => self
                .controller
                .selected_operation()
                .and_then(|op| op_field(op, field)),
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
            }),
            Selection::Operation(_) => self
                .controller
                .edit_selected_operation(|op| apply_op_fields(op, &parsed)),
            Selection::Stock => {}
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
        if let Some(pickbox) = self.pickbox_overlay() {
            layers = layers.push(pickbox);
        }
        layers.into()
    }

    /// The pickbox: a small accent square drawn over the cursor while a geometry
    /// pick is pending, its half-size the vertex-snap aperture. Purely visual — it
    /// does not intercept clicks (they fall through to the viewport).
    fn pickbox_overlay(&self) -> Option<Element<'_, Message>> {
        self.controller.pending_op()?;
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
        let has_tool = matches!(self.controller.selection(), Selection::Tool(_));
        // A new op needs geometry to pick and at least one tool.
        let can_create = has_geo && !self.controller.document().setup.tools.is_empty();
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
                    cmd(Icon::Face, "Face", begin(OpKind::Face)),
                ],
            }],
            RibbonTab::Tooling => vec![GroupSpec {
                title: "Tools",
                commands: vec![
                    cmd(Icon::NewTool, "New", Some(Message::NewTool)),
                    cmd(
                        Icon::Delete,
                        "Delete",
                        has_tool.then_some(Message::DeleteTool),
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
        band.into()
    }

    /// The Windows tab: a checkbox per pane (naturally narrow, no collapse).
    fn windows_body(&self) -> Element<'_, Message> {
        let mut panes = column![].spacing(4);
        for pane in ALL_PANES {
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
        match pane {
            Pane::Project => self.project_tree(),
            Pane::Viewport => container(
                shader(Viewport::new(
                    &self.controller,
                    self.show_stock,
                    self.view,
                    self.show_gizmo,
                ))
                .width(Length::Fill)
                .height(Length::Fill),
            )
            .width(Length::Fill)
            .height(Length::Fill)
            .into(),
            Pane::Inspector => self.inspector(),
            Pane::Output => self.output(),
        }
    }

    /// The project tree: setup, stock, tools, and operations as selectable rows.
    fn project_tree(&self) -> Element<'_, Message> {
        let setup = &self.controller.document().setup;
        let sel = self.controller.selection();
        let node = |label: String, target: Selection, active: bool| {
            let mark = if active { "▸ " } else { "  " };
            button(text(format!("{mark}{label}")).size(13))
                .on_press(Message::Select(target))
                .width(Length::Fill)
        };

        let mut list = column![
            node(
                format!("Setup — {}", setup.name),
                Selection::Setup,
                sel == Selection::Setup
            ),
            node(
                "Stock".to_string(),
                Selection::Stock,
                sel == Selection::Stock
            ),
            text("  Tools").size(12),
        ]
        .spacing(2);
        for (i, tool) in setup.tools.iter().enumerate() {
            list = list.push(node(
                format!("  T{} ⌀{}", tool.number, fmt_num(tool.diameter)),
                Selection::Tool(i),
                sel == Selection::Tool(i),
            ));
        }
        list = list.push(text("  Operations").size(12));
        if setup.operations.is_empty() {
            list = list.push(text("    (none — add one from the Operations tab)").size(11));
        }
        let op_count = setup.operations.len();
        for (i, op) in setup.operations.iter().enumerate() {
            let id = op.id();
            let active = sel == Selection::Operation(id);
            let excluded = self.controller.is_operation_excluded(id);
            // Each op row carries its own controls: an include checkbox (checked =
            // machined), the selectable name, and inline reorder arrows. An
            // excluded op stays in the tree, marked, but is not cut.
            let mark = if active { "▸ " } else { "  " };
            let mut label = format!("{mark}{id}: {}", op_kind(op));
            if excluded {
                label.push_str("  (excluded)");
            }
            let include = checkbox(!excluded)
                .size(15)
                .on_toggle(move |checked| Message::SetOpExcluded(id, !checked));
            let name = button(text(label).size(13))
                .on_press(Message::Select(Selection::Operation(id)))
                .width(Length::Fill);
            let up = button(text("↑").size(12))
                .on_press_maybe((i > 0).then_some(Message::MoveOp(id, true)));
            let down = button(text("↓").size(12))
                .on_press_maybe((i + 1 < op_count).then_some(Message::MoveOp(id, false)));
            list = list.push(
                row![include, name, up, down]
                    .spacing(4)
                    .align_y(Alignment::Center),
            );
        }

        // Structural editing of the selected operation: Duplicate/Delete. New ops
        // are created (per kind) from the Operations ribbon tab; reorder is inline
        // per row, above.
        let has_op = matches!(sel, Selection::Operation(_));
        // Intrinsic-width buttons (small icon + label), each sized to its own
        // content. The pane can't be dragged narrow enough to spill thanks to
        // PANE_MIN_RATIO.
        let op_btn = |icon: Icon, label: &str, msg: Message, enabled: bool| {
            let content = row![icon_svg(icon, 14.0), text(label.to_string()).size(12)]
                .spacing(4)
                .align_y(Alignment::Center);
            button(content).on_press_maybe(enabled.then_some(msg))
        };
        list = list.push(text(" ").size(6));
        list = list.push(
            row![
                op_btn(Icon::Duplicate, "Duplicate", Message::DuplicateOp, has_op),
                op_btn(Icon::Delete, "Delete", Message::DeleteOp, has_op),
            ]
            .spacing(4),
        );

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
        };
        let tools: Vec<ToolChoice> = self
            .controller
            .document()
            .setup
            .tools
            .iter()
            .map(|t| ToolChoice {
                number: t.number,
                diameter: t.diameter,
            })
            .collect();
        let selected = tools.iter().copied().find(|c| c.number == pending.tool);
        let picker = pick_list(tools, selected, |c| Message::SetPendingTool(c.number))
            .text_size(13)
            .width(Length::Fill);
        column![
            text(format!("New {kind} operation")).size(15),
            text("Tool").size(12),
            picker,
            text("Click a region in the viewport to place it.").size(12),
            button(text("Cancel").size(13)).on_press(Message::CancelOp),
        ]
        .spacing(10)
        .padding(8)
        .into()
    }

    fn inspector(&self) -> Element<'_, Message> {
        // While a new-operation wizard is active, the inspector is the wizard.
        if let Some(pending) = self.controller.pending_op() {
            return self.op_wizard(pending);
        }
        let heading = match self.controller.selection() {
            Selection::Setup => "Setup".to_string(),
            Selection::Stock => "Stock".to_string(),
            Selection::Tool(i) => format!("Tool {}", i + 1),
            Selection::Operation(id) => match self.controller.operation(id) {
                Some(op) => format!("Operation {id} — {}", op_kind(op)),
                None => "Operation".to_string(),
            },
        };

        let mut list = column![text(heading).size(15)].spacing(8).padding(8);

        if let Selection::Stock = self.controller.selection() {
            let cam_model::Stock::Box { min, max } = self.controller.document().setup.stock;
            list = list.push(
                text(format!(
                    "Box  X {:.1}…{:.1}\n     Y {:.1}…{:.1}\n     Z {:.1}…{:.1}",
                    min[0], max[0], min[1], max[1], min[2], max[2]
                ))
                .size(12),
            );
            return scrollable(list)
                .width(Length::Fill)
                .height(Length::Fill)
                .into();
        }

        let ordered = self.inspector_fields();
        if ordered.is_empty() {
            list = list.push(text("Nothing to edit here yet.").size(12));
        }
        for field in ordered {
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
        // The tool geometry class is an enum, so it gets a picker (committed
        // immediately) rather than a text field.
        if let Selection::Tool(i) = self.controller.selection() {
            if let Some(tool) = self.controller.document().setup.tools.get(i) {
                list = list.push(
                    row![
                        text("Type").width(Length::Fixed(150.0)).size(13),
                        pick_list(
                            &ToolKind::ALL[..],
                            Some(tool.kind),
                            Message::ToolKindChanged
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
    inner + GROUP_PAD * 2.0
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
    }
}

/// Read an operation's value for a given field, if the op has it.
fn op_field(op: &Operation, field: Field) -> Option<f64> {
    match (op, field) {
        (Operation::Profile(o), Field::Depth) => Some(o.depth),
        (Operation::Profile(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Profile(o), Field::Feed) => Some(o.feed),
        (Operation::Profile(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Profile(o), Field::LeadInSize) => Some(lead_size(o.lead_in)),
        (Operation::Profile(o), Field::LeadOutSize) => Some(lead_size(o.lead_out)),
        (Operation::Profile(o), Field::PlungeA) => Some(plunge_params(o.plunge).0),
        (Operation::Profile(o), Field::PlungeB) => Some(plunge_params(o.plunge).1),
        (Operation::Pocket(o), Field::Depth) => Some(o.depth),
        (Operation::Pocket(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Pocket(o), Field::Stepover) => Some(o.stepover),
        (Operation::Pocket(o), Field::Feed) => Some(o.feed),
        (Operation::Pocket(o), Field::PlungeFeed) => Some(o.plunge_feed),
        (Operation::Pocket(o), Field::PlungeA) => Some(plunge_params(o.plunge).0),
        (Operation::Pocket(o), Field::PlungeB) => Some(plunge_params(o.plunge).1),
        (Operation::Face(o), Field::Depth) => Some(o.depth),
        (Operation::Face(o), Field::Stepdown) => Some(o.stepdown),
        (Operation::Face(o), Field::Feed) => Some(o.feed),
        (Operation::Drill(o), Field::Depth) => Some(o.depth),
        (Operation::Drill(o), Field::Feed) => Some(o.feed),
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
            let (a, b) = plunge_params(o.plunge);
            let a = get(Field::PlungeA).unwrap_or(a);
            let b = get(Field::PlungeB).unwrap_or(b);
            o.plunge = set_plunge_params(o.plunge, a, b);
        }
        Operation::Face(o) => {
            if let Some(v) = get(Field::Depth) {
                o.depth = v;
            }
            if let Some(v) = get(Field::Stepdown) {
                o.stepdown = v;
            }
            if let Some(v) = get(Field::Feed) {
                o.feed = v;
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
    }
}

/// Format a number for display (Rust's shortest round-trippable form).
fn fmt_num(v: f64) -> String {
    format!("{v}")
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

/// The shader widget's transient state: only the active drag. The camera lives
/// in [`App`] so the view-cube buttons can drive it; drags report *relative*
/// deltas back as messages (loss-free even across a burst of events).
#[derive(Default)]
struct ViewportState {
    drag: Option<DragMode>,
    last: Option<iced::Point>,
}

/// Orbit sensitivity (radians per pixel).
const ORBIT_SENS: f32 = 0.008;
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

/// The orientation cube's square rectangle in the top-right of a `w × h`
/// viewport (widget-local coordinates). Fractional, so it is the same in logical
/// and physical pixels — the click hit-test and the drawn cube stay aligned.
fn gizmo_rect(w: f32, h: f32) -> (f32, f32, f32) {
    let size = (w.min(h) * 0.26).max(1.0);
    let margin = 8.0;
    (w - size - margin, margin, size)
}

struct Viewport {
    vertices: Arc<Vec<Vertex>>,
    mesh_vertices: Arc<Vec<MeshVertex>>,
    mesh_indices: Arc<Vec<u32>>,
    bounds: Option<([f32; 3], [f32; 3])>,
    controls: ViewControls,
    show_gizmo: bool,
    /// Geometry-pick mode (a new-operation wizard is awaiting a region click).
    picking: bool,
    /// The world Z of the plane clicks are projected onto (top of stock).
    pick_z: f32,
}

impl Viewport {
    fn new(
        controller: &AppController,
        show_stock: bool,
        controls: ViewControls,
        show_gizmo: bool,
    ) -> Self {
        let picking = controller.pending_op().is_some();
        let pick_z = controller.document().setup.heights.top_of_stock as f32;
        // After a run, show the full backplot; before it, at least show the
        // imported part outlines so opening a file is visibly reflected.
        let scene = match controller.outcome() {
            Some(outcome) => outcome.scene.clone(),
            None => {
                let mut scene = Scene::new();
                for region in controller.regions() {
                    scene.add_region(region, PART);
                }
                scene
            }
        };
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
            bounds: scene.bounds(),
            controls,
            show_gizmo,
            picking,
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
        let (gx, gy, size) = gizmo_rect(bounds.width, bounds.height);
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
                // A left-click on a gizmo face snaps the view to that side. (The
                // gizmo sits inset from the edges, so it clears the band below.)
                if matches!(button, Button::Left) {
                    if let Some((yaw, pitch)) = self.gizmo_pick(pos, bounds) {
                        return Some(
                            shader::Action::publish(Message::SetView(yaw, pitch)).and_capture(),
                        );
                    }
                }
                // In geometry-pick mode a left-click selects geometry (projected
                // onto the stock-top plane) instead of orbiting; the pickbox
                // half-size becomes the world-space snap aperture.
                if self.picking && matches!(button, Button::Left) {
                    let aspect = if bounds.height > 0.0 {
                        bounds.width / bounds.height
                    } else {
                        1.0
                    };
                    let u = 2.0 * (pos.x - bounds.x) / bounds.width - 1.0;
                    let v = 1.0 - 2.0 * (pos.y - bounds.y) / bounds.height;
                    let cam = self.camera();
                    if let Some(w) = cam.pick_plane(u, v, aspect, self.pick_z) {
                        let aperture = 0.5 * PICKBOX_PX * cam.world_per_pixel(bounds.height);
                        return Some(
                            shader::Action::publish(Message::PickWorld(w, aperture)).and_capture(),
                        );
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
                // While a pick is pending, track the cursor so the App can draw the
                // pickbox over it. Don't capture — this is passive tracking.
                if self.picking && cursor.position_over(bounds).is_some() {
                    return Some(shader::Action::publish(Message::ViewportCursor(*position)));
                }
            }
            Mouse::ButtonReleased(_) => {
                if state.drag.take().is_some() {
                    state.last = None;
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
        }
    }

    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: iced::Rectangle,
        cursor: iced::mouse::Cursor,
    ) -> iced::mouse::Interaction {
        let over = cursor.position_over(bounds).is_some();
        if self.picking && over {
            // A crosshair under the drawn pickbox — AutoCAD-style precise aiming.
            iced::mouse::Interaction::Crosshair
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
        let (lx, ly, size) = gizmo_rect(clip_bounds.width as f32, clip_bounds.height as f32);
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
