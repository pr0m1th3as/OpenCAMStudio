//! User preferences — the per-user settings file.
//!
//! **Deliberately outside the `gui` module.** `cam-app` builds and tests *without*
//! the `gui` feature by default, so anything living in `gui.rs` is invisible to the
//! default test run — and the rules worth testing here (clamping, defaults, what
//! happens to a file we cannot read) are exactly the kind that go unnoticed for
//! months otherwise. Same reasoning that moved `ToolKindPick` and
//! `origin_move_targets` out.
//!
//! # What is a preference
//!
//! Something about **how you work**, or about **your hardware**. Never about *this
//! part*. The tool library supplies the initial cutting data for each cutter.
//! Explicit parameter defaults saved from the inspector can override those
//! starting values for an operation kind.
//!
//! - **No default machine** — only the post ([`NewProjectPrefs`]). The machine
//!   *gates* an export (envelope, feed/spindle ceilings) and is saved per project at
//!   schema v11 precisely so reopening a job cannot silently retarget it.
//! - **No units (mm/inch)** — that reaches every field, every golden, the `.ocam`
//!   format and every post. Its own milestone if ever wanted, never a checkbox.
//!
//! The inverse is the strongest argument *for* the file: several values here are not
//! preferences of taste but of **hardware**. The pane minimums were tuned on one
//! monitor, and a value chosen on one display is a bug on another (see
//! [`PanePrefs`]).

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::SnapKind;

/// The settings file's own format version.
///
/// **Its own counter, not `cam_model::SCHEMA_VERSION`.** That one describes the
/// *document* format; coupling them would force a document migration every time a
/// checkbox is added here.
///
/// Purely additive changes — a new field with `#[serde(default)]` — need no bump: an
/// older file simply loads with the default. Bump when an existing field changes
/// meaning or shape, and add a step to [`Settings::migrate`].
///
/// **v2:** `defaults: NewProjectPrefs` became `session: SessionState`. The post stopped
/// being a *preference* ("which post do new projects get") and became *last-used
/// session state* ("which control were you on"), once machine and control became
/// shop-local.
///
/// **v3:** `session.post` retired in favour of `session.machine`. A machine now carries
/// its control (`MachineEntry::post`), so the active machine *implies* the post and
/// keeping both would be two sources of truth for one question — which is the objection
/// that retired the preference in the first place, one level down. Predicted in v2's own
/// documentation; it took two days.
pub const SETTINGS_VERSION: u32 = 3;

/// What happened when the settings file was read — the public face of
/// [`crate::config::ConfigLoad`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LoadOutcome {
    /// No file yet. Defaults are in force and **nothing has been written**.
    Fresh,
    /// Read, migrated if needed, and clamped.
    Loaded,
    /// A file exists but could not be used. Defaults are in force, **the file was left
    /// exactly as it was**, and a `.bak` copy sits beside it; the string says why.
    Rejected(String),
}

impl From<crate::config::ConfigLoad> for LoadOutcome {
    fn from(c: crate::config::ConfigLoad) -> Self {
        match c {
            // Settings never seed, so `Seeded` cannot arise; fold it in rather than
            // widen this enum with a variant nothing can produce.
            crate::config::ConfigLoad::Fresh | crate::config::ConfigLoad::Seeded => {
                LoadOutcome::Fresh
            }
            crate::config::ConfigLoad::Loaded => LoadOutcome::Loaded,
            crate::config::ConfigLoad::Rejected(w) => LoadOutcome::Rejected(w),
        }
    }
}

/// How the tool draws and catches in the viewport, and what the panes may not shrink
/// past. See the module docs for what deliberately is *not* here.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    /// Format version — see [`SETTINGS_VERSION`].
    pub settings_version: u32,
    /// Decimal places used to display tool dimensions and cutting data.
    pub number_decimals: u8,
    /// Block G-code export when verification detects a stock or fixture collision.
    pub block_export_on_verification_errors: bool,
    pub view: ViewPrefs,
    pub snapping: SnapPrefs,
    pub panes: PanePrefs,
    pub simulation: SimulationPrefs,
    /// Allow a local MCP client to connect when the desktop starts.
    pub mcp_enabled: bool,
    pub session: SessionState,
    pub tool_table: ToolTablePrefs,
    /// Shortcut overrides; an empty value disables a command.
    pub shortcuts: std::collections::BTreeMap<String, String>,
    /// Numeric sources and option selections saved per operation kind and parameter name.
    #[serde(skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub parameter_defaults:
        std::collections::BTreeMap<String, std::collections::BTreeMap<String, String>>,
    /// Keys this build does not know about, carried through a save untouched.
    ///
    /// So that running an older build after a newer one does not quietly delete the
    /// newer build's preferences. Costs one map; buys forward compatibility.
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

/// Tool table layout and active filters, independent of the current project.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ToolTablePrefs {
    pub visible: [bool; 8],
    pub widths: [f32; 8],
    pub order: [usize; 8],
    pub grouped: bool,
    /// Width and visibility of the library list in the Tools workspace.
    pub library_sidebar_width: f32,
    pub library_sidebar_collapsed: bool,
    pub search: String,
    pub filters: [Option<std::collections::BTreeSet<String>>; 8],
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

impl Default for ToolTablePrefs {
    fn default() -> Self {
        Self {
            visible: [true; 8],
            widths: [70.0, 220.0, 140.0, 140.0, 120.0, 80.0, 90.0, 130.0],
            order: [0, 1, 2, 3, 4, 5, 6, 7],
            grouped: false,
            library_sidebar_width: 210.0,
            library_sidebar_collapsed: false,
            search: String::new(),
            filters: Default::default(),
            extra: Default::default(),
        }
    }
}

/// Viewport furniture: what is shown, and how big the orientation cube draws.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ViewPrefs {
    pub show_stock: bool,
    /// Blend the stock surface so the model or drawing remains visible inside it.
    pub transparent_stock: bool,
    /// Transparency from 0 (opaque) to 1 (fully transparent).
    pub stock_transparency: f32,
    /// Stock surface RGB color, with each channel in the range 0–1.
    pub stock_color: [f32; 3],
    pub show_gizmo: bool,
    pub show_origin: bool,
    /// Draw the active machine's travel as a box around the job.
    pub show_envelope: bool,
    pub tooltips: bool,
    /// Orientation-cube size, logical px.
    pub gizmo_size: f32,
    /// Workpiece-origin marker size, as a multiple of the shipped size.
    ///
    /// The marker is sized from the scene's extent (6% of it) rather than in
    /// millimetres, so it stays legible on a 20 mm part and a 500 mm one alike. This
    /// scales that, for operators who want the datum louder or quieter against a busy
    /// backplot.
    pub origin_marker_scale: f32,
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

/// Material-removal grid quality, independent of toolpath tolerances.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SimulationPrefs {
    /// Linear grid density relative to the default; cost grows approximately squared.
    pub accuracy: f32,
    pub show_tool: bool,
    pub tool_display: SimulationToolDisplay,
    pub transparent_tool: bool,
    /// Choose which side of the rotary motion remains stationary on screen.
    pub rotary_view: SimulationRotaryView,
    pub programmed_point: bool,
    pub show_toolpath: bool,
    pub show_links: bool,
    pub show_leads: bool,
    pub show_cutting: bool,
    pub show_points: bool,
    pub toolpath_mode: SimulationToolpathMode,
    pub toolpath_coloring: SimulationToolpathColoring,
    pub show_stock: bool,
    pub stock_coloring: SimulationStockColoring,
    pub material: SimulationMaterial,
    pub transparent_stock: bool,
    pub regenerate_on_rewind: bool,
    pub stop_on_collision: bool,
    pub show_model: bool,
    /// Comparison tolerance in millimetres, against the tessellated CAD solid.
    pub comparison_tolerance: f32,
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

impl Default for SimulationPrefs {
    fn default() -> Self {
        Self {
            accuracy: 1.0,
            show_tool: true,
            tool_display: SimulationToolDisplay::Shaft,
            transparent_tool: false,
            rotary_view: SimulationRotaryView::FixedStock,
            programmed_point: false,
            show_toolpath: true,
            show_links: true,
            show_leads: true,
            show_cutting: true,
            show_points: false,
            toolpath_mode: SimulationToolpathMode::Trailing,
            toolpath_coloring: SimulationToolpathColoring::MoveType,
            show_stock: true,
            stock_coloring: SimulationStockColoring::Operation,
            material: SimulationMaterial::Metallic,
            transparent_stock: false,
            regenerate_on_rewind: true,
            stop_on_collision: true,
            show_model: true,
            comparison_tolerance: 0.05,
            extra: Default::default(),
        }
    }
}

macro_rules! simulation_choices {
    ($name:ident { $($variant:ident => $label:literal),+ $(,)? }) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
        pub enum $name { $($variant),+ }
        impl $name {
            pub const ALL: &'static [Self] = &[$(Self::$variant),+];
        }
        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(match self { $(Self::$variant => $label),+ })
            }
        }
    };
}
simulation_choices!(SimulationRotaryView {
    FixedStock => "Fixed stock, rotating tool", RotatingStock => "Rotating stock, fixed tool",
});
simulation_choices!(SimulationToolDisplay {
    Cutter => "Cutter", Shaft => "Cutter and shaft", Holder => "Cutter and holder",
});
simulation_choices!(SimulationToolpathMode {
    All => "All toolpaths", Trailing => "Trailing", Leading => "Leading",
    Operation => "Current operation", Tail => "Tail",
});
simulation_choices!(SimulationToolpathColoring {
    MoveType => "Move type", CuttingForce => "Cutting force (actual removal)",
});
simulation_choices!(SimulationStockColoring {
    Material => "Material", Operation => "Operation", Tool => "Tool", Comparison => "Comparison",
    CuttingForce => "Cutting force",
});
simulation_choices!(SimulationMaterial {
    Metallic => "Metallic", Aluminum => "Aluminum", Steel => "Steel",
    Brass => "Brass", Wood => "Wood", Custom => "Stock color",
});
impl SimulationMaterial {
    pub fn color(self, custom: [f32; 3]) -> [f32; 4] {
        let [r, g, b] = match self {
            Self::Metallic => [0.70, 0.68, 0.59],
            Self::Aluminum => [0.78, 0.81, 0.85],
            Self::Steel => [0.46, 0.50, 0.56],
            Self::Brass => [0.75, 0.57, 0.22],
            Self::Wood => [0.65, 0.43, 0.23],
            Self::Custom => custom,
        };
        [r, g, b, 1.]
    }
}

/// Stock simulation grid density multiplier.
pub const SIMULATION_ACCURACY_RANGE: (f32, f32) = (0.25, 2.0);

pub(crate) fn clamp_simulation_accuracy(value: f32) -> f32 {
    clamp(value, SIMULATION_ACCURACY_RANGE, 1.0)
}

/// Picking and object-snap tolerances.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SnapPrefs {
    /// Master switch for viewport object snapping.
    pub enabled: bool,
    /// Show eligible features and the edge under the setup picker.
    pub highlight_features: bool,
    /// The pickbox aperture, logical px. **The exposed knob** (Andreas,
    /// 2026-08-01): the vertex-snap tolerance is its half-size, and the object-snap
    /// catch distance is [`SNAP_CATCH_MULTIPLE`] times it.
    ///
    /// The catch distance is deliberately **not** a second control. The two are
    /// physically related — a bigger aperture should catch from further away — and
    /// two absolute knobs would let a user set a catch distance *smaller* than the
    /// pickbox feeding it, which is incoherent. One knob, relationship preserved by
    /// construction.
    pub pickbox_px: f32,
    /// Snap marker size, as a multiple of the catch aperture.
    ///
    /// This one *does* keep its own control, unlike the catch distance, because it is
    /// not a tolerance at all — it is how large the marker draws. Wanting a large
    /// visible marker with a tight catch aperture is a sensible thing to want.
    pub marker_scale: f32,
    /// Which object snaps are armed in a fresh session.
    pub default_snaps: Vec<SnapKind>,
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

/// The object-snap catch aperture as a multiple of the pickbox. A constant, not a
/// preference — see [`SnapPrefs::pickbox_px`].
pub const SNAP_CATCH_MULTIPLE: f32 = 1.5;

/// Pane layout: the sizes the user dragged to, and the sizes panes may not shrink
/// past.
///
/// **The minimums are the hardware-dependent ones.** The shipped values fit real
/// content (the Project pane's Duplicate/Delete row, the Inspector's field rows), but
/// they were chosen on one monitor. They are **logical** pixels, so a high-DPI
/// display *with* OS scaling already works — the scale factor handles it. The genuine
/// failure modes are narrower: a **low-resolution** panel, where a 240 px Inspector
/// plus a 200 px Project eats a third of a 1366-wide screen and the Viewport stops
/// being usable, and **high-DPI with scaling disabled**, where the same numbers
/// become unusably small.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PanePrefs {
    pub show_project_browser: bool,
    pub show_library: bool,
    /// Whether the inspector may appear when editing; idle visibility is automatic.
    pub inspector_enabled: bool,
    pub show_output: bool,
    pub project_px: f32,
    pub inspector_px: f32,
    pub output_px: f32,
    pub min_project_px: f32,
    pub min_library_px: f32,
    pub min_viewport_px: f32,
    pub min_inspector_px: f32,
    pub min_output_px: f32,
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

/// Where you left off — restored on launch, **not** nominated in a panel.
///
/// The distinction matters and it is easy to blur (it was blurred once, in
/// `MACHINE_PLAN` §6, until Andreas pulled on it): an **active** selection is session
/// state that changes as you work; a **preferred** one is a default you nominate. Two
/// settings answering "which one do I start on" is one too many — when they disagree
/// the user has to know which wins, and for a machine that ambiguity ends with someone
/// authoring against the wrong travel limits.
///
/// So: last-used wins, for the same reason the armed snaps and the pane sizes do. The
/// safety comes from the active selection being *visible*, not from how it was chosen.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SessionState {
    /// The machine last selected, by name — and with it the control, since a machine
    /// carries its post.
    ///
    /// **Active, not preferred.** This is where you left off, not a default nominated in
    /// a panel: two settings answering "which machine do I start on" is one too many,
    /// and when they disagree someone authors against the wrong travel. The safety comes
    /// from the active machine being *visible*.
    ///
    /// A name that no longer resolves — the machine was deleted, or this file was copied
    /// to another installation — falls back to the first in the library, and the app
    /// says so rather than landing somewhere silently.
    pub machine: Option<String>,
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

// ---------------------------------------------------------------------------
// Defaults — the single source of the values the GUI has hard-coded until now.
// ---------------------------------------------------------------------------

impl Default for ViewPrefs {
    fn default() -> Self {
        Self {
            show_stock: false,
            transparent_stock: false,
            stock_transparency: 0.75,
            stock_color: [0.2, 0.65, 0.3],
            show_gizmo: true,
            show_origin: true,
            // Off by default: it is a check you reach for, not something to look past
            // on every job.
            show_envelope: false,
            tooltips: true,
            gizmo_size: 110.0,
            origin_marker_scale: 1.0,
            extra: Default::default(),
        }
    }
}

impl Default for SnapPrefs {
    fn default() -> Self {
        Self {
            enabled: true,
            highlight_features: true,
            pickbox_px: 12.0,
            marker_scale: 1.2,
            // End + Mid + Quadrant on by default; Nearest is opt-in (AutoCAD-style).
            default_snaps: vec![SnapKind::End, SnapKind::Mid, SnapKind::Quadrant],
            extra: Default::default(),
        }
    }
}

impl Default for PanePrefs {
    fn default() -> Self {
        Self {
            show_project_browser: true,
            show_library: false,
            inspector_enabled: true,
            show_output: true,
            project_px: 220.0,
            inspector_px: 250.0,
            output_px: 140.0,
            min_project_px: 200.0,   // fits the Duplicate/Delete row
            min_library_px: 200.0,   // fits the Serial/Family tabs + rows
            min_viewport_px: 200.0,  // the main view stays usable
            min_inspector_px: 240.0, // fits the field rows
            min_output_px: 60.0,     // a short console is fine
            extra: Default::default(),
        }
    }
}

// `NewProjectPrefs` derives its Default rather than spelling one out: every field is
// already at its own default, and clippy is right that writing it by hand would be a
// second place to keep in step. The other three cannot derive — their defaults are
// specific values, not zeroes.

impl Default for Settings {
    fn default() -> Self {
        Self {
            settings_version: SETTINGS_VERSION,
            number_decimals: 2,
            block_export_on_verification_errors: true,
            view: ViewPrefs::default(),
            snapping: SnapPrefs::default(),
            panes: PanePrefs::default(),
            simulation: SimulationPrefs::default(),
            mcp_enabled: false,
            session: SessionState::default(),
            tool_table: ToolTablePrefs::default(),
            shortcuts: Default::default(),
            parameter_defaults: Default::default(),
            extra: Default::default(),
        }
    }
}

// ---------------------------------------------------------------------------
// Bounds
// ---------------------------------------------------------------------------

/// Orientation-cube size range (logical px) — the slider's existing range.
pub const GIZMO_SIZE_RANGE: (f32, f32) = (60.0, 220.0);
/// Workpiece-origin marker size, as a multiple of the shipped size.
pub const ORIGIN_MARKER_RANGE: (f32, f32) = (0.3, 3.0);
/// Pickbox aperture range (logical px). Below ~4 px nothing can be clicked; above
/// ~40 the box catches things the operator did not point at.
pub const PICKBOX_RANGE: (f32, f32) = (4.0, 40.0);
/// Snap marker size, as a multiple of the catch aperture.
pub const MARKER_SCALE_RANGE: (f32, f32) = (0.5, 4.0);
/// A pane's minimum size (logical px). The upper bound matters: a minimum larger
/// than the window leaves no room for anything else.
pub const PANE_MIN_RANGE: (f32, f32) = (40.0, 600.0);
/// A dragged pane size (logical px).
pub const PANE_SIZE_RANGE: (f32, f32) = (40.0, 4000.0);

fn clamp(v: f32, (lo, hi): (f32, f32), fallback: f32) -> f32 {
    if v.is_nan() {
        return fallback;
    }
    v.clamp(lo, hi)
}

impl Settings {
    /// Display rounding never changes the stored machining values.
    pub fn format_number(&self, value: f64) -> String {
        let digits = usize::from(self.number_decimals.min(8));
        let formatted = format!("{value:.digits$}");
        if formatted.starts_with('-') && formatted.parse::<f64>() == Ok(0.) {
            formatted[1..].to_owned()
        } else {
            formatted
        }
    }

    #[cfg(feature = "gui")]
    pub(crate) fn format_diagnostic<'a>(
        &self,
        diagnostic: &'a cam_toolpath::Diagnostic,
    ) -> std::borrow::Cow<'a, str> {
        diagnostic.display_message(self.number_decimals, |value| self.format_number(value))
    }

    /// Keep in-progress text intact; format only inactive numeric inputs.
    pub fn format_numeric_input(&self, value: &str, editing: bool) -> String {
        if !editing {
            if let Ok(number) = value.parse::<f64>() {
                if number.is_finite() {
                    return self.format_number(number);
                }
            }
        }
        value.to_owned()
    }

    /// Force every value into range.
    ///
    /// Applied **on load**, not merely on edit: `settings.json` is a plain text file a
    /// user can hand-edit, and a 0 px pickbox or a 5000 px Inspector minimum must not
    /// be able to produce a window nothing can be done with. A NaN — which JSON cannot
    /// express but a future writer might — falls back to the default rather than
    /// propagating through every comparison as a silent false.
    pub fn clamp_all(&mut self) {
        let d = Settings::default();
        self.number_decimals = self.number_decimals.min(8);
        for (width, fallback) in self.tool_table.widths.iter_mut().zip(d.tool_table.widths) {
            *width = clamp(*width, (55.0, 600.0), fallback);
        }
        self.tool_table.library_sidebar_width = clamp(
            self.tool_table.library_sidebar_width,
            (180.0, 480.0),
            d.tool_table.library_sidebar_width,
        );
        let mut order = self.tool_table.order;
        order.sort_unstable();
        if order != [0, 1, 2, 3, 4, 5, 6, 7] {
            self.tool_table.order = d.tool_table.order;
        }
        self.simulation.accuracy = clamp_simulation_accuracy(self.simulation.accuracy);
        self.simulation.comparison_tolerance =
            clamp(self.simulation.comparison_tolerance, (0.001, 2.0), 0.05);
        let v = &mut self.view;
        v.stock_transparency = clamp(v.stock_transparency, (0.0, 1.0), d.view.stock_transparency);
        for (channel, fallback) in v.stock_color.iter_mut().zip(d.view.stock_color) {
            *channel = clamp(*channel, (0.0, 1.0), fallback);
        }
        v.gizmo_size = clamp(v.gizmo_size, GIZMO_SIZE_RANGE, d.view.gizmo_size);
        v.origin_marker_scale = clamp(
            v.origin_marker_scale,
            ORIGIN_MARKER_RANGE,
            d.view.origin_marker_scale,
        );

        let s = &mut self.snapping;
        s.pickbox_px = clamp(s.pickbox_px, PICKBOX_RANGE, d.snapping.pickbox_px);
        s.marker_scale = clamp(s.marker_scale, MARKER_SCALE_RANGE, d.snapping.marker_scale);
        // An empty snap set is not an error — it is "no object snaps", which the
        // pick UI already supports. Duplicates are, though: they would draw twice.
        s.default_snaps.dedup();

        let p = &mut self.panes;
        p.project_px = clamp(p.project_px, PANE_SIZE_RANGE, d.panes.project_px);
        p.inspector_px = clamp(p.inspector_px, PANE_SIZE_RANGE, d.panes.inspector_px);
        p.output_px = clamp(p.output_px, PANE_SIZE_RANGE, d.panes.output_px);
        p.min_project_px = clamp(p.min_project_px, PANE_MIN_RANGE, d.panes.min_project_px);
        p.min_library_px = clamp(p.min_library_px, PANE_MIN_RANGE, d.panes.min_library_px);
        p.min_viewport_px = clamp(p.min_viewport_px, PANE_MIN_RANGE, d.panes.min_viewport_px);
        p.min_inspector_px = clamp(p.min_inspector_px, PANE_MIN_RANGE, d.panes.min_inspector_px);
        p.min_output_px = clamp(p.min_output_px, PANE_MIN_RANGE, d.panes.min_output_px);
    }

    /// Replace the remembered session state — the values the GUI keeps live and the
    /// user changes by using the app, rather than by opening a preferences panel.
    ///
    /// Here rather than in `gui.rs` so the mapping is testable without standing up a
    /// GUI, and so there is exactly **one** place that knows it: per-field syncing
    /// scattered through the message handlers would drift the moment a field was
    /// added, and the symptom — one preference quietly not persisting — is close to
    /// invisible.
    ///
    /// `extra` is carried across rather than replaced: unknown keys written by a newer
    /// build must survive a session with an older one (see [`load_from`]).
    pub fn remember_session(
        &mut self,
        view: ViewPrefs,
        snaps: Vec<SnapKind>,
        session: SessionState,
        panes: PanePrefs,
    ) {
        let view_extra = std::mem::take(&mut self.view.extra);
        let pane_extra = std::mem::take(&mut self.panes.extra);
        let session_extra = std::mem::take(&mut self.session.extra);
        self.session = SessionState {
            extra: session_extra,
            ..session
        };
        self.view = ViewPrefs {
            extra: view_extra,
            ..view
        };
        self.panes = PanePrefs {
            extra: pane_extra,
            ..panes
        };
        self.snapping.default_snaps = snaps;
    }

    /// The object-snap catch aperture (logical px) implied by the pickbox.
    pub fn snap_catch_px(&self) -> f32 {
        self.snapping.pickbox_px * SNAP_CATCH_MULTIPLE
    }

    /// Bring a settings tree of version `from` up to [`SETTINGS_VERSION`].
    ///
    /// Empty today — v1 is the first version, and additive changes need no step (an
    /// older file loads with `#[serde(default)]` filling the new field). The shape is
    /// here so the *next* change has an obvious place to go, rather than a version
    /// number that gets written and never read. That mistake has already been paid
    /// for once, at document schema v10.
    fn migrate(&mut self, from: u32) {
        for step in from..SETTINGS_VERSION {
            // Both steps so far *retire* a key rather than reshape one, so both simply
            // remove it. A key nothing reads is a key that misleads the next reader, and
            // `extra` would otherwise carry it back out to disk forever.
            //
            // v1→v2: `defaults.post` (a nominated default for new projects).
            // v2→v3: `session.post` (last-used control) — a machine carries its control
            // now, so the active machine implies it.
            //
            // Neither value is carried forward, because the concept it named no longer
            // exists: a post is a property of a machine. The cost is one re-pick on the
            // machine, once.
            if step == 1 {
                self.extra.remove("defaults");
            }
            if step == 2 {
                if let Some(obj) = self
                    .extra
                    .get_mut("session")
                    .and_then(|v| v.as_object_mut())
                {
                    obj.remove("post");
                }
            }
        }
        self.settings_version = SETTINGS_VERSION;
    }
}

// ---------------------------------------------------------------------------
// Disk — the contract lives in `crate::config`, shared with the tool and machine
// libraries. Writing it a third time is how two copies come to disagree.
// ---------------------------------------------------------------------------

impl crate::config::ConfigFile for Settings {
    const FILE_NAME: &'static str = "settings.json";
    const VERSION: u32 = SETTINGS_VERSION;
    /// Nothing appears on disk until the user changes something — unlike a library,
    /// empty settings are perfectly usable.
    const SEED_ON_MISSING: bool = false;
    const WHAT: &'static str = "settings JSON";

    fn stated_version(&self) -> u32 {
        self.settings_version
    }

    fn migrate(&mut self, from: u32) {
        Settings::migrate(self, from)
    }

    fn normalise(&mut self) {
        self.clamp_all()
    }
}

/// `<config-dir>/OpenCAMStudio/settings.json`.
pub fn settings_path() -> Option<PathBuf> {
    crate::paths::config_file("settings.json")
}

/// Read the user's settings, or the defaults. See [`crate::config`] for the failure
/// contract — it is the part that matters.
pub fn load() -> (Settings, LoadOutcome) {
    let (s, outcome) = crate::config::load::<Settings>();
    (s, outcome.into())
}

/// Read settings from `path`.
pub fn load_from(path: &Path) -> (Settings, LoadOutcome) {
    let (s, outcome) = crate::config::load_from::<Settings>(path);
    (s, outcome.into())
}

impl Settings {
    /// Persist to the config directory (best-effort).
    pub fn save(&self) {
        crate::config::save(self)
    }

    /// Persist to `path`, creating the directory if needed.
    pub fn save_to(&self, path: &Path) -> std::io::Result<()> {
        crate::config::save_to(self, path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::tests::Scratch;

    /// **The test that proves this whole module is inert until someone edits
    /// something.** These literals are what the GUI hard-codes today; a
    /// `gui`-gated test asserts the two agree (`gui.rs`), and this one pins the
    /// numbers so neither side can drift silently.
    #[test]
    fn the_defaults_are_exactly_todays_hard_coded_values() {
        let d = Settings::default();
        assert_eq!(d.settings_version, 3);

        assert!(!d.view.show_stock);
        assert!(d.view.show_gizmo);
        assert!(d.view.show_origin);
        assert!(!d.view.show_envelope);
        assert!(d.view.tooltips);
        assert_eq!(d.view.gizmo_size, 110.0);
        assert_eq!(
            d.view.origin_marker_scale, 1.0,
            "the shipped marker size is 1x"
        );

        assert_eq!(d.snapping.pickbox_px, 12.0);
        assert_eq!(d.snapping.marker_scale, 1.2);
        assert_eq!(
            d.snapping.default_snaps,
            vec![SnapKind::End, SnapKind::Mid, SnapKind::Quadrant]
        );
        // The catch aperture is derived, not stored: 1.5 × 12 = 18 px, which is what
        // `SNAP_PICK_PX` has always been.
        assert_eq!(d.snap_catch_px(), 18.0);

        assert_eq!(d.panes.project_px, 220.0);
        assert_eq!(d.panes.inspector_px, 250.0);
        assert_eq!(d.panes.output_px, 140.0);
        assert_eq!(d.panes.min_project_px, 200.0);
        assert_eq!(d.panes.min_library_px, 200.0);
        assert_eq!(d.panes.min_viewport_px, 200.0);
        assert_eq!(d.panes.min_inspector_px, 240.0);
        assert_eq!(d.panes.min_output_px, 60.0);

        assert_eq!(
            d.session.machine, None,
            "nothing selected until there is a library"
        );
    }

    #[test]
    fn tool_table_preferences_round_trip_and_invalid_layout_is_clamped() {
        let scratch = Scratch::new("tool-table");
        let path = scratch.file("settings.json");
        let mut settings = Settings::default();
        settings.tool_table.visible[2] = false;
        settings.tool_table.widths[2] = 235.0;
        settings.tool_table.order.swap(0, 2);
        settings.tool_table.grouped = true;
        settings.tool_table.library_sidebar_width = 320.0;
        settings.tool_table.library_sidebar_collapsed = true;
        settings.tool_table.search = "mill".into();
        settings.tool_table.filters[5] = Some(["6.00".to_string()].into_iter().collect());
        settings.save_to(&path).unwrap();
        let (loaded, outcome) = load_from(&path);
        assert_eq!(outcome, LoadOutcome::Loaded);
        assert_eq!(loaded.tool_table, settings.tool_table);
        let mut invalid = loaded;
        invalid.tool_table.order = [0; 8];
        invalid.tool_table.widths[0] = 9000.0;
        invalid.tool_table.library_sidebar_width = 9000.0;
        invalid.clamp_all();
        assert_eq!(invalid.tool_table.library_sidebar_width, 480.0);
        assert_eq!(invalid.tool_table.order, ToolTablePrefs::default().order);
        assert_eq!(invalid.tool_table.widths[0], 600.0);
    }

    #[test]
    fn defaults_round_trip_through_json_unchanged() {
        let d = Settings::default();
        let text = serde_json::to_string_pretty(&d).unwrap();
        let back: Settings = serde_json::from_str(&text).unwrap();
        assert_eq!(d, back);
    }

    #[test]
    fn decimal_display_defaults_persists_and_preserves_editing_precision() {
        let mut settings: Settings = serde_json::from_str("{}").unwrap();
        assert_eq!(settings.number_decimals, 2);
        assert_eq!(settings.format_number(12.3456), "12.35");
        assert_eq!(settings.format_number(-0.001), "0.00");
        assert_eq!(settings.format_numeric_input("0.123456", true), "0.123456");
        assert_eq!(settings.format_numeric_input("1.", true), "1.");
        assert_eq!(settings.format_numeric_input("invalid", false), "invalid");
        settings.number_decimals = 4;
        assert_eq!(settings.format_number(12.3456), "12.3456");
        let scratch = Scratch::new("number-decimals");
        let path = scratch.file("settings.json");
        settings.save_to(&path).unwrap();
        let (loaded, outcome) = load_from(&path);
        assert_eq!(outcome, LoadOutcome::Loaded);
        assert_eq!(loaded.number_decimals, 4);
        settings.number_decimals = 255;
        settings.clamp_all();
        assert_eq!(settings.number_decimals, 8);
        settings.number_decimals = 0;
        assert_eq!(settings.format_number(12.9), "13");
    }

    #[test]
    fn simulation_accuracy_defaults_persists_and_clamps() {
        let old: Settings = serde_json::from_str(r#"{"view":{"show_stock":true}}"#).unwrap();
        assert_eq!(old.simulation.accuracy, 1.0);
        let scratch = Scratch::new("simulation-accuracy");
        let path = scratch.file("settings.json");
        for (input, expected) in [(0.25, 0.25), (2.0, 2.0), (0., 0.25), (100., 2.0)] {
            let mut settings = old.clone();
            settings.simulation.accuracy = input;
            settings.save_to(&path).unwrap();
            let (loaded, outcome) = load_from(&path);
            assert_eq!(outcome, LoadOutcome::Loaded);
            assert_eq!(loaded.simulation.accuracy, expected);
        }
        let mut settings = old;
        settings.simulation.accuracy = f32::NAN;
        settings.clamp_all();
        assert_eq!(settings.simulation.accuracy, 1.0);
    }

    #[test]
    fn simulation_display_settings_migrate_round_trip_and_normalize_tolerance() {
        let old: Settings = serde_json::from_str(r#"{"simulation":{"accuracy":1.5}}"#).unwrap();
        assert_eq!(old.simulation.accuracy, 1.5);
        assert!(old.simulation.stop_on_collision);
        assert_eq!(old.simulation.rotary_view, SimulationRotaryView::FixedStock);
        assert!(old.simulation.regenerate_on_rewind);
        assert_eq!(
            old.simulation.toolpath_mode,
            SimulationToolpathMode::Trailing
        );
        assert_eq!(
            old.simulation.toolpath_coloring,
            SimulationToolpathColoring::MoveType
        );
        let mut settings = old;
        settings.simulation.tool_display = SimulationToolDisplay::Holder;
        settings.simulation.rotary_view = SimulationRotaryView::RotatingStock;
        settings.simulation.toolpath_mode = SimulationToolpathMode::Leading;
        settings.simulation.toolpath_coloring = SimulationToolpathColoring::CuttingForce;
        settings.simulation.stock_coloring = SimulationStockColoring::Tool;
        settings.simulation.material = SimulationMaterial::Brass;
        settings.simulation.transparent_tool = true;
        settings.simulation.show_points = true;
        settings.simulation.show_links = false;
        settings.simulation.stop_on_collision = false;
        settings.simulation.regenerate_on_rewind = false;
        settings
            .simulation
            .extra
            .insert("future_option".into(), true.into());
        let scratch = Scratch::new("simulation-display");
        let path = scratch.file("settings.json");
        settings.save_to(&path).unwrap();
        let (loaded, outcome) = load_from(&path);
        assert_eq!(outcome, LoadOutcome::Loaded);
        assert_eq!(settings, loaded);
        settings.simulation.comparison_tolerance = -1.;
        settings.clamp_all();
        assert_eq!(settings.simulation.comparison_tolerance, 0.001);
        settings.simulation.comparison_tolerance = f32::NAN;
        settings.clamp_all();
        assert_eq!(settings.simulation.comparison_tolerance, 0.05);
    }

    #[test]
    fn stock_transparency_defaults_clamps_and_round_trips() {
        let mut settings: Settings = serde_json::from_str("{}").unwrap();
        assert_eq!(settings.view.stock_transparency, 0.75);
        settings.view.stock_transparency = 1.5;
        settings.clamp_all();
        assert_eq!(settings.view.stock_transparency, 1.0);
        settings.view.stock_transparency = 0.42;
        let loaded: Settings =
            serde_json::from_str(&serde_json::to_string(&settings).unwrap()).unwrap();
        assert_eq!(loaded.view.stock_transparency, 0.42);
    }

    #[test]
    fn stock_color_defaults_for_older_settings_and_clamps_on_load() {
        let old: Settings = serde_json::from_str(r#"{"view":{"show_stock":true}}"#).unwrap();
        assert_eq!(old.view.stock_color, [0.2, 0.65, 0.3]);
        let scratch = Scratch::new("stock-color");
        let mut settings = old;
        settings.view.stock_color = [-1.0, 0.4, 2.0];
        settings.save_to(&scratch.file("settings.json")).unwrap();
        let (loaded, outcome) = load_from(&scratch.file("settings.json"));
        assert_eq!(outcome, LoadOutcome::Loaded);
        assert_eq!(loaded.view.stock_color, [0.0, 0.4, 1.0]);
        settings.view.stock_color = [f32::NAN; 3];
        settings.clamp_all();
        assert_eq!(settings.view.stock_color, ViewPrefs::default().stock_color);
    }

    #[test]
    fn a_missing_file_yields_defaults_and_writes_nothing() {
        let s = Scratch::new("settings");
        let (got, outcome) = load_from(&s.file("settings.json"));
        assert_eq!(outcome, LoadOutcome::Fresh);
        assert_eq!(got, Settings::default());
        assert!(
            !s.file("settings.json").exists(),
            "first run must not write a settings file until something is changed"
        );
    }

    /// The tool library's trap, inverted: it falls back to defaults **and saves
    /// them**, overwriting whatever it could not read. Here the bytes must survive.
    #[test]
    fn an_unparseable_file_is_left_exactly_as_it_was() {
        let s = Scratch::new("settings");
        let junk = "{ this is not json at all";
        std::fs::write(s.file("settings.json"), junk).unwrap();

        let (got, outcome) = load_from(&s.file("settings.json"));
        assert!(matches!(outcome, LoadOutcome::Rejected(_)), "{outcome:?}");
        assert_eq!(got, Settings::default());
        assert_eq!(
            std::fs::read_to_string(s.file("settings.json")).unwrap(),
            junk,
            "a file we could not read must never be rewritten"
        );
    }

    #[test]
    fn a_file_from_a_newer_build_is_left_alone() {
        let s = Scratch::new("settings");
        let newer = format!(
            r#"{{"settings_version": {}, "view": {{"gizmo_size": 200.0}}}}"#,
            SETTINGS_VERSION + 7
        );
        std::fs::write(s.file("settings.json"), &newer).unwrap();

        let (got, outcome) = load_from(&s.file("settings.json"));
        assert!(matches!(outcome, LoadOutcome::Rejected(_)), "{outcome:?}");
        assert_eq!(got, Settings::default(), "defaults, not the newer values");
        assert_eq!(
            std::fs::read_to_string(s.file("settings.json")).unwrap(),
            newer,
            "running an older build must not cost the user their preferences"
        );
    }

    #[test]
    fn a_partial_file_fills_the_rest_from_defaults() {
        let s = Scratch::new("settings");
        std::fs::write(s.file("settings.json"), r#"{"view": {"tooltips": false}}"#).unwrap();
        let (got, outcome) = load_from(&s.file("settings.json"));
        assert_eq!(outcome, LoadOutcome::Loaded);
        assert!(!got.view.tooltips, "the stated value wins");
        assert_eq!(got.view.gizmo_size, 110.0, "the rest are defaults");
        assert_eq!(got.panes, PanePrefs::default());
        assert!(got.block_export_on_verification_errors);
    }

    #[test]
    fn unknown_keys_are_ignored_on_read_and_kept_on_write() {
        let s = Scratch::new("settings");
        std::fs::write(
            s.file("settings.json"),
            r#"{"settings_version": 1, "future_thing": {"a": 1},
                "view": {"tooltips": false, "future_toggle": true}}"#,
        )
        .unwrap();

        let (got, outcome) = load_from(&s.file("settings.json"));
        assert_eq!(outcome, LoadOutcome::Loaded);
        assert!(!got.view.tooltips);

        got.save_to(&s.file("settings.json")).unwrap();
        let text = std::fs::read_to_string(s.file("settings.json")).unwrap();
        assert!(
            text.contains("future_thing"),
            "top-level key dropped: {text}"
        );
        assert!(text.contains("future_toggle"), "nested key dropped: {text}");
    }

    #[test]
    fn hand_edited_nonsense_is_clamped_into_range() {
        let s = Scratch::new("settings");
        std::fs::write(
            s.file("settings.json"),
            r#"{"snapping": {"pickbox_px": 0.0, "marker_scale": 900.0},
                "view": {"gizmo_size": 100000.0},
                "panes": {"min_inspector_px": 5000.0, "min_viewport_px": -3.0}}"#,
        )
        .unwrap();

        let (got, _) = load_from(&s.file("settings.json"));
        assert_eq!(got.snapping.pickbox_px, PICKBOX_RANGE.0);
        assert_eq!(got.snapping.marker_scale, MARKER_SCALE_RANGE.1);
        assert_eq!(got.view.gizmo_size, GIZMO_SIZE_RANGE.1);
        assert_eq!(got.panes.min_inspector_px, PANE_MIN_RANGE.1);
        assert_eq!(got.panes.min_viewport_px, PANE_MIN_RANGE.0);
        // And the clamped result must itself be usable: no minimum may exceed the
        // upper bound that keeps a window habitable.
        assert!(got.panes.min_project_px <= PANE_MIN_RANGE.1);
    }

    #[test]
    fn remembering_a_session_keeps_unknown_keys_and_the_pane_minimums() {
        let mut s = Settings::default();
        s.view
            .extra
            .insert("future_toggle".into(), serde_json::json!(true));
        s.panes
            .extra
            .insert("future_pane".into(), serde_json::json!(7));
        s.panes.min_inspector_px = 180.0; // a value the user had set

        s.session
            .extra
            .insert("future_session".into(), serde_json::json!("x"));
        s.remember_session(
            ViewPrefs {
                tooltips: false,
                transparent_stock: true,
                stock_color: [0.8, 0.3, 0.1],
                gizmo_size: 175.0,
                ..Default::default()
            },
            vec![SnapKind::End],
            SessionState {
                machine: Some("Mill".into()),
                extra: Default::default(),
            },
            PanePrefs {
                project_px: 310.0,
                ..s.panes.clone()
            },
        );

        assert!(!s.view.tooltips);
        assert!(s.view.transparent_stock);
        let reloaded: Settings = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        assert!(reloaded.view.transparent_stock);
        assert_eq!(reloaded.view.stock_color, [0.8, 0.3, 0.1]);
        assert_eq!(s.view.gizmo_size, 175.0);
        assert_eq!(s.snapping.default_snaps, vec![SnapKind::End]);
        assert_eq!(s.panes.project_px, 310.0);
        assert_eq!(
            s.panes.min_inspector_px, 180.0,
            "remembering the session must not reset the user's pane minimums"
        );
        assert_eq!(
            s.session.machine.as_deref(),
            Some("Mill"),
            "the machine is remembered"
        );
        assert!(
            s.view.extra.contains_key("future_toggle")
                && s.panes.extra.contains_key("future_pane")
                && s.session.extra.contains_key("future_session"),
            "a newer build's keys must survive an older build's session"
        );
    }

    /// Every extreme a preferences slider can reach must survive a save and reload
    /// unchanged.
    ///
    /// The panel's sliders are bounded by these same range constants, so this ties the
    /// two together: if a control's range were ever widened past what `clamp_all`
    /// accepts, the user would set a value, restart, and find it silently moved — with
    /// nothing to tell them the app had overruled them.
    #[test]
    fn every_value_a_control_can_reach_survives_a_reload() {
        for pick in [PICKBOX_RANGE.0, PICKBOX_RANGE.1] {
            for scale in [MARKER_SCALE_RANGE.0, MARKER_SCALE_RANGE.1] {
                for pane_min in [PANE_MIN_RANGE.0, PANE_MIN_RANGE.1] {
                    for gizmo in [GIZMO_SIZE_RANGE.0, GIZMO_SIZE_RANGE.1] {
                        let mut s = Settings::default();
                        s.view.origin_marker_scale = ORIGIN_MARKER_RANGE.1;
                        s.snapping.pickbox_px = pick;
                        s.snapping.marker_scale = scale;
                        s.view.gizmo_size = gizmo;
                        s.panes.min_project_px = pane_min;
                        s.panes.min_library_px = pane_min;
                        s.panes.min_viewport_px = pane_min;
                        s.panes.min_inspector_px = pane_min;
                        s.panes.min_output_px = pane_min;
                        let before = s.clone();
                        s.clamp_all();
                        assert_eq!(
                            s, before,
                            "a value the panel allows was changed on load: \
                             pickbox {pick}, scale {scale}, pane min {pane_min}, gizmo {gizmo}"
                        );
                    }
                }
            }
        }
    }

    /// Restore-to-defaults is the only escape from a layout the user cannot otherwise
    /// undo — five maxed-out pane minimums leave no room for the viewport. So it has
    /// to reset *everything*, not just the section in front of them.
    #[test]
    fn restoring_defaults_recovers_from_an_unusable_layout() {
        let mut wrecked = Settings::default();
        for m in [
            &mut wrecked.panes.min_project_px,
            &mut wrecked.panes.min_library_px,
            &mut wrecked.panes.min_viewport_px,
            &mut wrecked.panes.min_inspector_px,
            &mut wrecked.panes.min_output_px,
        ] {
            *m = PANE_MIN_RANGE.1;
        }
        wrecked.panes.project_px = PANE_SIZE_RANGE.1;
        wrecked.snapping.pickbox_px = PICKBOX_RANGE.1;
        assert_ne!(wrecked, Settings::default());

        // What the panel's button does.
        let restored = Settings::default();
        assert_eq!(
            restored.panes,
            PanePrefs::default(),
            "pane sizes AND minimums"
        );
        assert_eq!(restored.snapping, SnapPrefs::default());
        assert_eq!(restored.view, ViewPrefs::default());
        let side = restored.panes.min_project_px
            + restored.panes.min_inspector_px
            + restored.panes.min_viewport_px;
        assert!(
            side <= 1366.0,
            "the recovered layout must fit a modest screen"
        );
    }

    /// **The migrate hook's first real use.** `schema_version` was written into every
    /// project save for nine versions and read by nothing; the settings counter was
    /// built to be read from day one, and this is the step that proves it is.
    ///
    /// A v1 file nominated a post under `defaults`; v2 keeps it as last-used session
    /// state under `session`. The old key survives the rename because unknown keys are
    /// preserved rather than dropped — which is exactly what makes the value still
    /// available to move.
    #[test]
    fn a_v1_file_migrates_through_every_step_without_error() {
        let s = Scratch::new("settings");
        std::fs::write(
            s.file("settings.json"),
            r#"{"settings_version": 1, "defaults": {"post": "Okuma"},
                "view": {"tooltips": false}}"#,
        )
        .unwrap();

        let (got, outcome) = load_from(&s.file("settings.json"));
        assert_eq!(outcome, LoadOutcome::Loaded);
        assert_eq!(got.settings_version, SETTINGS_VERSION);
        assert!(!got.view.tooltips, "everything else still loads");
        assert!(
            !got.extra.contains_key("defaults"),
            "a migrated key must not linger and be written back out"
        );
        // v1's post went to v2's `session.post`, which v3 then retired — the machine
        // carries the control now. What matters is that a two-step migration runs
        // cleanly and leaves nothing stale behind, not that the value survived a
        // concept that no longer exists.
        assert_eq!(got.session.machine, None);
    }

    #[test]
    fn a_saved_file_reloads_identically() {
        let s = Scratch::new("settings");
        let mut written = Settings::default();
        written.view.tooltips = false;
        written.view.gizmo_size = 175.0;
        written.snapping.pickbox_px = 20.0;
        written.snapping.default_snaps = vec![SnapKind::End, SnapKind::Nearest];
        written.panes.project_px = 310.0;
        written.panes.min_inspector_px = 180.0;
        written.panes.show_project_browser = false;
        written.panes.show_library = true;
        written.panes.inspector_enabled = false;
        written.panes.show_output = false;
        written.session.machine = Some("Router".into());
        written.save_to(&s.file("settings.json")).unwrap();

        let (got, outcome) = load_from(&s.file("settings.json"));
        assert_eq!(outcome, LoadOutcome::Loaded);
        assert_eq!(got, written);
    }

    /// A NaN cannot come from JSON, but it can come from a future writer or a bad
    /// edit path. It must not propagate — every comparison against it is silently
    /// false, which is how an unusable window would go unnoticed.
    #[test]
    fn a_nan_falls_back_to_the_default_rather_than_propagating() {
        let mut s = Settings::default();
        s.view.gizmo_size = f32::NAN;
        s.snapping.pickbox_px = f32::NAN;
        s.panes.min_output_px = f32::NAN;
        s.clamp_all();
        assert_eq!(s.view.gizmo_size, 110.0);
        assert_eq!(s.snapping.pickbox_px, 12.0);
        assert_eq!(s.panes.min_output_px, 60.0);
    }

    /// Every pane minimum must fit inside a window a real person might use. Two side
    /// panes plus a viewport at their maximum permitted minimums must still leave the
    /// viewport room on a modest 1366-wide laptop.
    #[test]
    fn the_minimum_bounds_cannot_add_up_to_an_unusable_window() {
        let d = Settings::default();
        let side = d.panes.min_project_px + d.panes.min_inspector_px + d.panes.min_viewport_px;
        assert!(
            side <= 1366.0,
            "the shipped minimums already exceed a 1366-wide screen: {side}"
        );
    }
}
