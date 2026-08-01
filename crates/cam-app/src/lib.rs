//! # cam-app — the OpenCAMStudio application
//!
//! The app is split in two:
//!
//! - [`AppController`] — all behaviour (open DXF, edit parameters undoably, run
//!   the strategies, build the viewport scene, export G-code), with **no GUI
//!   dependency**, so it is unit-tested like the rest of the pipeline.
//! - The **`gui` feature**'s iced shell — a thin view over the controller. It
//!   needs a windowing/graphics stack, so it is compiled and run on the desktop,
//!   not in headless tests.

mod controller;
// Where per-user files live. One resolver, shared by the tool library and the
// settings file — two copies of a platform convention drift silently.
mod paths;
mod project;
// Not GUI-gated, deliberately: `cam-app` builds and tests without the `gui` feature
// by default, so preferences living in `gui.rs` would be untestable.
mod settings;
// Not GUI-gated: the library type and the `.ocam` file union are plain serializable
// data + config-dir I/O (only their *use* is GUI). Phase 3 (`TOOLING_PLAN.md`) lets
// the ungated `project` module reference `ToolLibrary` for the `OcamFile` union.
mod tool_library;

pub use controller::{
    op_accepts_open_paths, op_selects_circles, op_takes_islands, AppController, CuttingData,
    ExportError, ExportToError,
    JobParams, LoopPart, LoopRef, OpKind, PendingOp, PickResult, ProjectError, RunOutcome,
    Selection, SnapHit, SnapKind,
};
pub use project::{OcamFile, Project};
pub use settings::{
    load as load_settings, load_from as load_settings_from, settings_path, LoadOutcome,
    PanePrefs, SessionState, Settings, SnapPrefs, ViewPrefs, GIZMO_SIZE_RANGE,
    MARKER_SCALE_RANGE, ORIGIN_MARKER_RANGE, PANE_MIN_RANGE, PANE_SIZE_RANGE, PICKBOX_RANGE,
    SETTINGS_VERSION,
    SNAP_CATCH_MULTIPLE,
};
pub use tool_library::{families_for, LibraryLoad, ToolKindPick, ToolLibrary, LIBRARY_VERSION};

/// The human-facing version string, including git provenance on a dev build.
///
/// A clean tagged release shows just the semver (`0.1.0`); any build past the tag,
/// or a dirty tree, shows the full `git describe` so a bug report can be pinned to
/// an exact commit -- the version field alone cannot do this, because the manifest
/// version only changes at release time (see `VERSIONING.md`). The provenance
/// is stamped in by `build.rs`; see [`emit_git_provenance`](../build.rs).
pub fn version_string() -> String {
    format_version(
        env!("CARGO_PKG_VERSION"),
        env!("OCAM_GIT_DESCRIBE"),
        env!("OCAM_BUILD_DATE"),
    )
}

/// Pure formatter behind [`version_string`], split out so its three branches are
/// testable without depending on the ambient git state at build time.
fn format_version(version: &str, describe: &str, date: &str) -> String {
    // No git at build time (e.g. a source tarball): the semver is all we have.
    if describe.is_empty() {
        return version.to_string();
    }
    // A clean release: `git describe` is exactly the tag (`v0.1.0`) -- no
    // `-N-gHASH` offset, no `-dirty`. The provenance would only echo the version.
    if describe.trim_start_matches('v') == version {
        return version.to_string();
    }
    // A dev build: pin it to the commit, dating it when we have the commit date.
    if date.is_empty() {
        format!("{version} ({describe})")
    } else {
        format!("{version} ({describe}, {date})")
    }
}

#[cfg(test)]
mod version_tests {
    use super::format_version;

    #[test]
    fn no_git_shows_bare_semver() {
        // Source-tarball build: build.rs found no `.git`, so both stamps are empty.
        assert_eq!(format_version("0.1.0", "", ""), "0.1.0");
    }

    #[test]
    fn clean_release_shows_bare_semver() {
        // `git describe` on the exact tag echoes it (with the `v` prefix); the
        // provenance adds nothing, so it is suppressed -- with or without a date.
        assert_eq!(format_version("0.1.0", "v0.1.0", "2026-07-28"), "0.1.0");
        assert_eq!(format_version("0.1.0", "v0.1.0", ""), "0.1.0");
    }

    #[test]
    fn dev_build_pins_the_commit() {
        assert_eq!(
            format_version("0.1.0", "v0.1.0-4-g748f9ea", "2026-07-28"),
            "0.1.0 (v0.1.0-4-g748f9ea, 2026-07-28)"
        );
    }

    #[test]
    fn dirty_tree_is_carried_through() {
        // The `-dirty` suffix must survive to the UI -- it is the signal that the
        // working tree diverged from any commit.
        assert_eq!(
            format_version("0.1.0", "v0.1.0-4-g748f9ea-dirty", "2026-07-28"),
            "0.1.0 (v0.1.0-4-g748f9ea-dirty, 2026-07-28)"
        );
    }

    #[test]
    fn dev_build_without_date_still_pins_the_commit() {
        assert_eq!(
            format_version("0.1.0", "748f9ea", ""),
            "0.1.0 (748f9ea)"
        );
    }
}

#[cfg(feature = "gui")]
pub mod gui;
