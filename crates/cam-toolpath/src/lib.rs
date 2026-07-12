//! # cam-toolpath — strategies that turn geometry into CL-data
//!
//! A **strategy** consumes model geometry (via `cam-geo`) and emits controller-
//! neutral [`Program`](cam_cldata::Program) motions. Strategies are the left-hand
//! plugin kind of the hourglass; posts are the right.
//!
//! ## Design rules honoured here
//!
//! - **Pure & cancellable.** [`Strategy::compute`] is a pure function of its
//!   inputs — no I/O, no globals, deterministic — so it is trivially testable and
//!   `cam-app` can run it as a cancellable background task via a [`CancelToken`].
//! - **Diagnostics, not panics.** A strategy never panics on bad input; it
//!   returns a [`StrategyResult`] carrying whatever motions it could produce plus
//!   typed [`Diagnostic`]s (errors and warnings).
//! - **ABI-friendly seam.** Only plain data crosses [`Strategy::compute`]
//!   ([`JobEnv`] in, [`StrategyResult`] out), so extracting the trait into a
//!   `cdylib` plugin ABI later is cheap.

mod drill;
mod emit;
mod face;
mod plan;
mod pocket;
mod profile;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use cam_cldata::Program;
use cam_model::{Heights, Tool};

pub use drill::DrillStrategy;
pub use face::FaceStrategy;
pub use plan::build_job;
pub use pocket::PocketStrategy;
pub use profile::ProfileStrategy;

/// Severity of a [`Diagnostic`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Severity {
    /// Informational note.
    Info,
    /// A concern that did not stop toolpath generation (e.g. an unsupported
    /// refinement was skipped).
    Warning,
    /// A problem that prevented (part of) the toolpath from being generated.
    Error,
}

/// A typed message from a strategy, referencing what went wrong. (Geometry
/// references — a specific vertex or chain — will attach here as the model
/// grows; for now it carries a human-readable message.)
#[derive(Clone, Debug, PartialEq)]
pub struct Diagnostic {
    /// How serious the diagnostic is.
    pub severity: Severity,
    /// Human-readable description.
    pub message: String,
}

impl Diagnostic {
    /// An error diagnostic.
    pub fn error(message: impl Into<String>) -> Self {
        Self {
            severity: Severity::Error,
            message: message.into(),
        }
    }

    /// A warning diagnostic.
    pub fn warning(message: impl Into<String>) -> Self {
        Self {
            severity: Severity::Warning,
            message: message.into(),
        }
    }

    /// An informational diagnostic.
    pub fn info(message: impl Into<String>) -> Self {
        Self {
            severity: Severity::Info,
            message: message.into(),
        }
    }
}

/// The outcome of a strategy: the motions it produced, any diagnostics, and
/// whether it stopped early because it was cancelled.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StrategyResult {
    /// The CL-data the strategy produced (possibly empty on error).
    pub program: Program,
    /// Diagnostics gathered during computation.
    pub diagnostics: Vec<Diagnostic>,
    /// Whether computation stopped early due to cancellation.
    pub cancelled: bool,
}

impl StrategyResult {
    /// Whether any diagnostic is an [`Severity::Error`].
    pub fn has_errors(&self) -> bool {
        self.diagnostics
            .iter()
            .any(|d| d.severity == Severity::Error)
    }
}

/// A cheap, cloneable cancellation flag. `cam-app` sets it from another thread to
/// ask an in-flight strategy to stop; the strategy polls it at loop boundaries.
#[derive(Clone, Debug, Default)]
pub struct CancelToken(Arc<AtomicBool>);

impl CancelToken {
    /// A fresh, un-cancelled token.
    pub fn new() -> Self {
        Self::default()
    }

    /// Request cancellation.
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    /// Whether cancellation has been requested.
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}

/// Everything a strategy needs from the surrounding job that is not part of the
/// operation itself: the setup's safety heights and its tool list.
#[derive(Clone, Copy, Debug)]
pub struct JobEnv<'a> {
    /// The setup's safety planes.
    pub heights: Heights,
    /// The setup's tools.
    pub tools: &'a [Tool],
}

impl<'a> JobEnv<'a> {
    /// Look up a tool by its number.
    pub fn tool(&self, number: u32) -> Option<&'a Tool> {
        self.tools.iter().find(|t| t.number == number)
    }
}

/// A toolpath strategy: a pure, cancellable function from job context to CL-data.
pub trait Strategy {
    /// A short identifier (e.g. `"profile"`).
    fn name(&self) -> &str;

    /// Compute the toolpath. Must be pure (no I/O, deterministic) and should poll
    /// `cancel` at loop boundaries so long jobs stop promptly.
    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult;
}
