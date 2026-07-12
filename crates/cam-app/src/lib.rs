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

pub use controller::{AppController, ExportError, JobParams, RunOutcome};

#[cfg(feature = "gui")]
pub mod gui;
