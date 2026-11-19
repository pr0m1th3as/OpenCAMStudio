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

mod parallel;
mod rotary;
pub use parallel::{ParallelStrategy, SpiralStrategy};
pub use rotary::index_program;
pub use rotary::wrap_program;
pub use rotary::ROTARY_WRAP_REVISION;
mod bore;
pub use bore::BoreStrategy;
mod adaptive_linking;
mod adaptive_safety;
mod adaptive_strategy;
mod carve;
mod chamfer;
mod clearing;
mod clearsim;
mod drill;
mod emit;
mod engrave;
mod face;
mod frontadvance;
mod guards;
mod leads;
mod linking;
mod plan;
mod pocket;
mod profile;
mod raster;
mod rings;
mod steer;
mod thread;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use cam_cldata::{Program, SpindleDir};
use cam_model::{Heights, Tool};

pub use adaptive_strategy::AdaptiveStrategy;
pub use cache::{CacheStats, OperationCache};
pub use carve::{carve_shape, CarveShape, CarveStrategy};
pub use chamfer::ChamferStrategy;
pub use drill::DrillStrategy;
pub use engrave::EngraveStrategy;
pub use face::FaceStrategy;
pub use plan::{
    build_job, build_job_cached, build_job_cached_with_preview, build_job_cached_with_progress,
    CalculationProgress, JobCompletion, JobResult, PlanRequest,
};
mod cache;
pub use pocket::PocketStrategy;
pub use profile::{profile_tab_preview, ProfileStrategy, ProfileTabPreview};
pub use thread::ThreadStrategy;

/// `"1 pass"` / `"3 passes"` — correct pluralisation for a program comment.
///
/// Written out rather than `pass(es)`: the post strips parentheses from comments
/// (they delimit the comment, so a nested pair would end it early), which turned
/// `pass(es)` into the ungrammatical `passes` for every count.
pub(crate) fn passes_phrase(n: usize) -> String {
    if n == 1 {
        "1 pass".to_string()
    } else {
        format!("{n} passes")
    }
}

/// Severity of a [`Diagnostic`].
#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum Severity {
    /// Informational note.
    Info,
    /// A concern that did not stop toolpath generation (e.g. an unsupported
    /// refinement was skipped).
    Warning,
    /// A problem that prevented (part of) the toolpath from being generated.
    Error,
}

/// Stable machine-readable diagnostic categories; message text is presentation.
#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum DiagnosticCode {
    InvalidInput,
    GeometryUnresolved,
    InvalidMotion,
    StrategyWarning,
    Information,
    WorkerFailed,
    Cancelled,
}

/// Source ownership survives worker/service boundaries.
#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SourceRef {
    pub setup: Option<u64>,
    pub operation: Option<u32>,
    pub segment: Option<usize>,
    pub pattern_id: Option<u64>,
    pub instance_index: Option<u32>,
}

/// A typed message from a strategy, referencing what went wrong. (Geometry
/// references — a specific vertex or chain — will attach here as the model
/// grows; for now it carries a human-readable message.)
#[derive(serde::Serialize, serde::Deserialize, Clone, Debug, PartialEq)]
pub struct Diagnostic {
    /// How serious the diagnostic is.
    pub severity: Severity,
    pub code: DiagnosticCode,
    pub source: SourceRef,
    /// Human-readable description.
    pub message: String,
    /// The operation this diagnostic belongs to, when known. Strategies do not set
    /// it — [`build_job`] stamps it as it collects each operation's result, since the
    /// planner is what knows the ownership. Lets the UI mark *which* operation failed
    /// instead of only reporting a count.
    pub op: Option<u32>,
}

impl Diagnostic {
    /// An error diagnostic.
    pub fn error(message: impl Into<String>) -> Self {
        Self {
            severity: Severity::Error,
            code: DiagnosticCode::InvalidInput,
            source: SourceRef::default(),
            message: message.into(),
            op: None,
        }
    }

    /// Attach the operation this diagnostic came from.
    pub fn for_op(mut self, id: u32) -> Self {
        self.op = Some(id);
        self.source.operation = Some(id);
        self
    }

    pub fn for_segment(mut self, index: usize) -> Self {
        self.source.segment = Some(index);
        self
    }
    pub fn with_code(mut self, code: DiagnosticCode) -> Self {
        self.code = code;
        self
    }
    pub fn for_setup(mut self, id: u64) -> Self {
        self.source.setup = Some(id);
        self
    }

    /// Attach the derived pattern instance that owns this motion or diagnostic.
    pub fn for_pattern(mut self, pattern_id: u64, instance_index: u32) -> Self {
        self.source.pattern_id = Some(pattern_id);
        self.source.instance_index = Some(instance_index);
        self
    }

    /// A warning diagnostic.
    pub fn warning(message: impl Into<String>) -> Self {
        Self {
            severity: Severity::Warning,
            code: DiagnosticCode::StrategyWarning,
            source: SourceRef::default(),
            message: message.into(),
            op: None,
        }
    }

    /// An informational diagnostic.
    pub fn info(message: impl Into<String>) -> Self {
        Self {
            severity: Severity::Info,
            code: DiagnosticCode::Information,
            source: SourceRef::default(),
            message: message.into(),
            op: None,
        }
    }
}

/// The outcome of a strategy: the motions it produced, any diagnostics, and
/// whether it stopped early because it was cancelled.
#[derive(serde::Serialize, serde::Deserialize, Clone, Debug, Default, PartialEq)]
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
/// operation itself: the setup's safety heights, its tool list, the resolved stock
/// extent, and which way the spindle turns.
#[derive(Clone, Copy, Debug)]
pub struct JobEnv<'a> {
    /// The setup's safety planes.
    pub heights: Heights,
    /// The setup's tools.
    pub tools: &'a [Tool],
    /// The resolved stock box in XY `(min, max)`, if known — needed by roughing
    /// strategies (e.g. profile stepover) that clear the material out to the stock.
    /// `None` when the caller has no stock bounds; such strategies then skip
    /// roughing and cut a single pass.
    pub stock: Option<([f64; 2], [f64; 2])>,
    /// Which way the spindle turns — `Cw` for the usual `M3`.
    ///
    /// **Climb versus conventional is not a property of the toolpath alone.** It is
    /// the relation between the direction of travel and the direction the cutting
    /// edge is moving where it meets the material, so reversing the spindle
    /// exchanges the two for the same geometry. Every strategy that turns a
    /// climb/conventional *request* into a winding or an arc direction therefore
    /// needs this, and until now each simply assumed `M3` in a comment.
    pub spindle: SpindleDir,
}

impl<'a> JobEnv<'a> {
    /// Look up a tool by its number.
    pub fn tool(&self, number: u32) -> Option<&'a Tool> {
        self.tools.iter().find(|t| t.number == number)
    }

    /// Whether a **climb** cut runs the tool the same way round the material as the
    /// geometry's own winding — true for the ordinary `M3` spindle, false for `M4`.
    ///
    /// The single place the spindle's sense is turned into a path decision, so a
    /// strategy states its intent (`climb`) and never re-derives the convention.
    pub fn climb_follows_winding(&self) -> bool {
        self.spindle == SpindleDir::Cw
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

mod fixtures;
pub use fixtures::{verify_fixture_clearance, FixtureContact, FIXTURE_ROUTING_REVISION};
