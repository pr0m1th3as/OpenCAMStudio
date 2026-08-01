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
//! part*. Three exclusions follow from that and are settled (Andreas, 2026-08-01):
//!
//! - **No cutting parameters** — stepover, feeds, plunge style are a tool-library
//!   matter. Per-tool nominal cutting data already seeds a new operation; a second
//!   default here would be a competing source of truth, and the two would drift with
//!   no way for the operator to tell which produced the number in front of them.
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

use cam_post::PostKind;
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
/// meaning or shape, and add a step to [`migrate`].
pub const SETTINGS_VERSION: u32 = 1;

/// What happened when the settings file was read. Returned rather than swallowed so
/// the caller can say something in the Output pane instead of the user wondering why
/// their preferences reverted.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LoadOutcome {
    /// No file yet. Defaults are in force and **nothing has been written** — the
    /// file appears the first time the user changes something.
    Fresh,
    /// Read, migrated if needed, and clamped.
    Loaded,
    /// A file exists but could not be used. Defaults are in force and **the file was
    /// left exactly as it was**; the string says why.
    Rejected(String),
}

/// How the tool draws and catches in the viewport, and what the panes may not shrink
/// past. See the module docs for what deliberately is *not* here.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    /// Format version — see [`SETTINGS_VERSION`].
    pub settings_version: u32,
    pub view: ViewPrefs,
    pub snapping: SnapPrefs,
    pub panes: PanePrefs,
    pub defaults: NewProjectPrefs,
    /// Keys this build does not know about, carried through a save untouched.
    ///
    /// So that running an older build after a newer one does not quietly delete the
    /// newer build's preferences. Costs one map; buys forward compatibility.
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

/// Viewport furniture: what is shown, and how big the orientation cube draws.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ViewPrefs {
    pub show_stock: bool,
    pub show_gizmo: bool,
    pub show_origin: bool,
    pub tooltips: bool,
    /// Orientation-cube size, logical px.
    pub gizmo_size: f32,
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

/// Picking and object-snap tolerances.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SnapPrefs {
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

/// Seeds for a **newly created** project — and nothing else.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct NewProjectPrefs {
    /// The post a fresh project targets.
    ///
    /// **Applies at new-project creation only.** An opened `.ocam` keeps its own post
    /// (schema v11), and a file predating v11 says nothing and leaves the session's
    /// choice alone — reading absence as "use the preference" would retarget an old
    /// job on open, which is exactly what v11 exists to prevent. Andreas, 2026-08-01:
    /// *"Existing projects opened respect `.ocam`, we never retarget unless the user
    /// explicitly does so."*
    pub post: PostKind,
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
            show_gizmo: true,
            show_origin: true,
            tooltips: true,
            gizmo_size: 110.0,
            extra: Default::default(),
        }
    }
}

impl Default for SnapPrefs {
    fn default() -> Self {
        Self {
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
            view: ViewPrefs::default(),
            snapping: SnapPrefs::default(),
            panes: PanePrefs::default(),
            defaults: NewProjectPrefs::default(),
            extra: Default::default(),
        }
    }
}

// ---------------------------------------------------------------------------
// Bounds
// ---------------------------------------------------------------------------

/// Orientation-cube size range (logical px) — the slider's existing range.
pub const GIZMO_SIZE_RANGE: (f32, f32) = (60.0, 220.0);
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
    /// Force every value into range.
    ///
    /// Applied **on load**, not merely on edit: `settings.json` is a plain text file a
    /// user can hand-edit, and a 0 px pickbox or a 5000 px Inspector minimum must not
    /// be able to produce a window nothing can be done with. A NaN — which JSON cannot
    /// express but a future writer might — falls back to the default rather than
    /// propagating through every comparison as a silent false.
    pub fn clamp_all(&mut self) {
        let d = Settings::default();
        let v = &mut self.view;
        v.gizmo_size = clamp(v.gizmo_size, GIZMO_SIZE_RANGE, d.view.gizmo_size);

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
    pub fn remember_session(&mut self, view: ViewPrefs, snaps: Vec<SnapKind>, panes: PanePrefs) {
        let view_extra = std::mem::take(&mut self.view.extra);
        let pane_extra = std::mem::take(&mut self.panes.extra);
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
        for _step in from..SETTINGS_VERSION {
            // match _step { 1 => …, _ => {} }
        }
        self.settings_version = SETTINGS_VERSION;
    }
}

// ---------------------------------------------------------------------------
// Disk
// ---------------------------------------------------------------------------

/// `<config-dir>/OpenCAMStudio/settings.json`.
pub fn settings_path() -> Option<PathBuf> {
    crate::paths::config_file("settings.json")
}

/// Read the user's settings, or the defaults.
///
/// See [`load_from`] for the failure contract — it is the part that matters.
pub fn load() -> (Settings, LoadOutcome) {
    match settings_path() {
        Some(p) => load_from(&p),
        // No config dir on this machine: defaults, and nothing to write.
        None => (Settings::default(), LoadOutcome::Fresh),
    }
}

/// Read settings from `path`.
///
/// **A file we could not read is never rewritten.** That is the whole contract, and
/// it is deliberately *unlike* [`ToolLibrary::load`](crate::ToolLibrary::load), which
/// falls back to defaults **and immediately saves them** — so a format change there
/// would silently overwrite a real tool library with the stock set. Here a file that
/// will not parse, or that a newer build wrote, is left exactly as it is: the user
/// runs the older build with defaults, then goes back to the newer one and finds
/// their preferences intact.
pub fn load_from(path: &Path) -> (Settings, LoadOutcome) {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        // Missing is the ordinary first-run case, not a failure.
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return (Settings::default(), LoadOutcome::Fresh)
        }
        Err(e) => {
            return (
                Settings::default(),
                LoadOutcome::Rejected(format!("could not be read ({e})")),
            )
        }
    };
    let mut s: Settings = match serde_json::from_str(&text) {
        Ok(s) => s,
        Err(e) => {
            return (
                Settings::default(),
                LoadOutcome::Rejected(format!("is not valid settings JSON ({e})")),
            )
        }
    };
    if s.settings_version > SETTINGS_VERSION {
        return (
            Settings::default(),
            LoadOutcome::Rejected(format!(
                "was written by a newer version (format {}, this build understands {SETTINGS_VERSION})",
                s.settings_version
            )),
        );
    }
    s.migrate(s.settings_version);
    s.clamp_all();
    (s, LoadOutcome::Loaded)
}

impl Settings {
    /// Persist to the config directory. Best-effort: a read-only config dir simply
    /// means changes do not survive the run, which is already the tool library's
    /// behaviour and is not worth interrupting the user over.
    pub fn save(&self) {
        if let Some(path) = settings_path() {
            let _ = self.save_to(&path);
        }
    }

    /// Persist to `path`, creating the directory if needed.
    pub fn save_to(&self, path: &Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let text = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        std::fs::write(path, text)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    /// A scratch directory that removes itself. No `tempfile` dependency for the sake
    /// of a handful of tests.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new() -> Self {
            static N: AtomicU32 = AtomicU32::new(0);
            let d = std::env::temp_dir().join(format!(
                "ocam-settings-{}-{}",
                std::process::id(),
                N.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&d).expect("scratch dir");
            Self(d)
        }
        fn file(&self) -> PathBuf {
            self.0.join("settings.json")
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// **The test that proves this whole module is inert until someone edits
    /// something.** These literals are what the GUI hard-codes today; a
    /// `gui`-gated test asserts the two agree (`gui.rs`), and this one pins the
    /// numbers so neither side can drift silently.
    #[test]
    fn the_defaults_are_exactly_todays_hard_coded_values() {
        let d = Settings::default();
        assert_eq!(d.settings_version, 1);

        assert!(!d.view.show_stock);
        assert!(d.view.show_gizmo);
        assert!(d.view.show_origin);
        assert!(d.view.tooltips);
        assert_eq!(d.view.gizmo_size, 110.0);

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

        assert_eq!(d.defaults.post, PostKind::default());
    }

    #[test]
    fn defaults_round_trip_through_json_unchanged() {
        let d = Settings::default();
        let text = serde_json::to_string_pretty(&d).unwrap();
        let back: Settings = serde_json::from_str(&text).unwrap();
        assert_eq!(d, back);
    }

    #[test]
    fn a_missing_file_yields_defaults_and_writes_nothing() {
        let s = Scratch::new();
        let (got, outcome) = load_from(&s.file());
        assert_eq!(outcome, LoadOutcome::Fresh);
        assert_eq!(got, Settings::default());
        assert!(
            !s.file().exists(),
            "first run must not write a settings file until something is changed"
        );
    }

    /// The tool library's trap, inverted: it falls back to defaults **and saves
    /// them**, overwriting whatever it could not read. Here the bytes must survive.
    #[test]
    fn an_unparseable_file_is_left_exactly_as_it_was() {
        let s = Scratch::new();
        let junk = "{ this is not json at all";
        std::fs::write(s.file(), junk).unwrap();

        let (got, outcome) = load_from(&s.file());
        assert!(matches!(outcome, LoadOutcome::Rejected(_)), "{outcome:?}");
        assert_eq!(got, Settings::default());
        assert_eq!(
            std::fs::read_to_string(s.file()).unwrap(),
            junk,
            "a file we could not read must never be rewritten"
        );
    }

    #[test]
    fn a_file_from_a_newer_build_is_left_alone() {
        let s = Scratch::new();
        let newer = format!(
            r#"{{"settings_version": {}, "view": {{"gizmo_size": 200.0}}}}"#,
            SETTINGS_VERSION + 7
        );
        std::fs::write(s.file(), &newer).unwrap();

        let (got, outcome) = load_from(&s.file());
        assert!(matches!(outcome, LoadOutcome::Rejected(_)), "{outcome:?}");
        assert_eq!(got, Settings::default(), "defaults, not the newer values");
        assert_eq!(
            std::fs::read_to_string(s.file()).unwrap(),
            newer,
            "running an older build must not cost the user their preferences"
        );
    }

    #[test]
    fn a_partial_file_fills_the_rest_from_defaults() {
        let s = Scratch::new();
        std::fs::write(s.file(), r#"{"view": {"tooltips": false}}"#).unwrap();
        let (got, outcome) = load_from(&s.file());
        assert_eq!(outcome, LoadOutcome::Loaded);
        assert!(!got.view.tooltips, "the stated value wins");
        assert_eq!(got.view.gizmo_size, 110.0, "the rest are defaults");
        assert_eq!(got.panes, PanePrefs::default());
    }

    #[test]
    fn unknown_keys_are_ignored_on_read_and_kept_on_write() {
        let s = Scratch::new();
        std::fs::write(
            s.file(),
            r#"{"settings_version": 1, "future_thing": {"a": 1},
                "view": {"tooltips": false, "future_toggle": true}}"#,
        )
        .unwrap();

        let (got, outcome) = load_from(&s.file());
        assert_eq!(outcome, LoadOutcome::Loaded);
        assert!(!got.view.tooltips);

        got.save_to(&s.file()).unwrap();
        let text = std::fs::read_to_string(s.file()).unwrap();
        assert!(text.contains("future_thing"), "top-level key dropped: {text}");
        assert!(text.contains("future_toggle"), "nested key dropped: {text}");
    }

    #[test]
    fn hand_edited_nonsense_is_clamped_into_range() {
        let s = Scratch::new();
        std::fs::write(
            s.file(),
            r#"{"snapping": {"pickbox_px": 0.0, "marker_scale": 900.0},
                "view": {"gizmo_size": 100000.0},
                "panes": {"min_inspector_px": 5000.0, "min_viewport_px": -3.0}}"#,
        )
        .unwrap();

        let (got, _) = load_from(&s.file());
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
        s.view.extra.insert("future_toggle".into(), serde_json::json!(true));
        s.panes.extra.insert("future_pane".into(), serde_json::json!(7));
        s.panes.min_inspector_px = 180.0; // a value the user had set

        s.remember_session(
            ViewPrefs {
                tooltips: false,
                gizmo_size: 175.0,
                ..Default::default()
            },
            vec![SnapKind::End],
            PanePrefs {
                project_px: 310.0,
                ..s.panes.clone()
            },
        );

        assert!(!s.view.tooltips);
        assert_eq!(s.view.gizmo_size, 175.0);
        assert_eq!(s.snapping.default_snaps, vec![SnapKind::End]);
        assert_eq!(s.panes.project_px, 310.0);
        assert_eq!(
            s.panes.min_inspector_px, 180.0,
            "remembering the session must not reset the user's pane minimums"
        );
        assert!(
            s.view.extra.contains_key("future_toggle") && s.panes.extra.contains_key("future_pane"),
            "a newer build's keys must survive an older build's session"
        );
    }

    #[test]
    fn a_saved_file_reloads_identically() {
        let s = Scratch::new();
        let mut written = Settings::default();
        written.view.tooltips = false;
        written.view.gizmo_size = 175.0;
        written.snapping.pickbox_px = 20.0;
        written.snapping.default_snaps = vec![SnapKind::End, SnapKind::Nearest];
        written.panes.project_px = 310.0;
        written.panes.min_inspector_px = 180.0;
        written.defaults.post = PostKind::Okuma;
        written.save_to(&s.file()).unwrap();

        let (got, outcome) = load_from(&s.file());
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
