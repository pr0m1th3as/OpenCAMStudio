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
mod project;
// Not GUI-gated: the library type and the `.ocam` file union are plain serializable
// data + config-dir I/O (only their *use* is GUI). Phase 3 (`TOOLING_PLAN.md`) lets
// the ungated `project` module reference `ToolLibrary` for the `OcamFile` union.
mod tool_library;

pub use controller::{
    op_selects_circles, AppController, ExportError, ExportToError, JobParams, LoopPart, LoopRef,
    OpKind, PendingOp, PickResult, ProjectError, RunOutcome, Selection, SnapHit, SnapKind,
};
pub use project::{OcamFile, Project};
pub use tool_library::ToolLibrary;

#[cfg(feature = "gui")]
pub mod gui;
