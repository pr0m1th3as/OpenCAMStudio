//! # cam-post — lowering CL-data to G-code
//!
//! A post-processor is **capabilities + a formatter**, not a monolith. The
//! [`Post`] trait exposes what a controller [`Capabilities`] supports and a
//! [`post`](Post::post) method that lowers a controller-neutral
//! [`Program`](cam_cldata::Program) into G-code text, **querying the
//! [`Machine`](cam_model::Machine)** for physical limits as it goes (Machine ≠
//! Post — one post drives many machines).
//!
//! The capabilities model does real work: a Tier-2 cycle such as
//! [`Drill`](cam_cldata::DrillCycle) is emitted as a canned cycle by a controller
//! that has one, or **expanded to explicit `G0`/`G1` pecks** by one that does not
//! (grbl). See [`grbl::GrblPost`].

mod fanuc;
mod grbl;
mod words;
mod writer;

pub use fanuc::FanucPost;
pub use grbl::GrblPost;

use core::fmt;

use cam_cldata::Program;
use cam_model::Machine;

/// What a controller's dialect can express. A post lowers CL-data according to
/// these flags; where a capability is missing, it must synthesise the behaviour
/// from primitives or refuse.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Capabilities {
    /// Native circular interpolation (`G2`/`G3`).
    pub arcs: bool,
    /// Canned drilling cycles (`G81`/`G82`/`G83`). When false, drilling is
    /// expanded to explicit moves.
    pub canned_drill: bool,
    /// Cutter radius compensation (`G41`/`G42`).
    pub cutter_comp: bool,
    /// Work coordinate systems (`G54`–`G59`).
    pub work_offsets: bool,
    /// Coolant control (`M7`/`M8`/`M9`).
    pub coolant: bool,
    /// Tool changes (`Tn M6`).
    pub tool_change: bool,
}

/// The work coordinate system a program runs in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum WorkOffset {
    /// `G54` — the usual default.
    #[default]
    G54,
    /// `G55`.
    G55,
    /// `G56`.
    G56,
    /// `G57`.
    G57,
    /// `G58`.
    G58,
    /// `G59`.
    G59,
}

impl WorkOffset {
    /// The G-code word for this work offset.
    pub fn code(self) -> &'static str {
        match self {
            WorkOffset::G54 => "G54",
            WorkOffset::G55 => "G55",
            WorkOffset::G56 => "G56",
            WorkOffset::G57 => "G57",
            WorkOffset::G58 => "G58",
            WorkOffset::G59 => "G59",
        }
    }
}

/// Knobs that shape a post's output without changing its meaning.
#[derive(Clone, Debug, PartialEq)]
pub struct PostOptions {
    /// Work coordinate system to select in the preamble.
    pub work_offset: WorkOffset,
    /// Number of decimal places for coordinates.
    pub precision: usize,
    /// Optional program name, emitted as a header comment.
    pub program_name: Option<String>,
}

impl Default for PostOptions {
    fn default() -> Self {
        Self {
            work_offset: WorkOffset::G54,
            precision: 3,
            program_name: None,
        }
    }
}

/// Why a post could not produce G-code.
#[derive(Clone, Debug, PartialEq)]
pub enum PostError {
    /// The program uses a feature this post cannot express or synthesise.
    Unsupported(String),
    /// A coordinate falls outside the machine's work envelope.
    OutOfEnvelope { x: f64, y: f64, z: f64 },
    /// A requested spindle speed exceeds the machine's maximum.
    SpindleOutOfRange(f64),
    /// A requested feed exceeds the machine's maximum.
    FeedOutOfRange(f64),
    /// An arc was encountered with no known current position to anchor `I`/`J`.
    ArcWithoutStart,
}

impl fmt::Display for PostError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PostError::Unsupported(what) => write!(f, "unsupported by this post: {what}"),
            PostError::OutOfEnvelope { x, y, z } => {
                write!(
                    f,
                    "coordinate ({x}, {y}, {z}) is outside the machine envelope"
                )
            }
            PostError::SpindleOutOfRange(rpm) => {
                write!(f, "spindle speed {rpm} rpm exceeds the machine maximum")
            }
            PostError::FeedOutOfRange(feed) => {
                write!(f, "feed {feed} mm/min exceeds the machine maximum")
            }
            PostError::ArcWithoutStart => {
                write!(f, "arc move has no preceding position to anchor I/J")
            }
        }
    }
}

impl std::error::Error for PostError {}

/// A post-processor: a controller dialect that lowers CL-data to G-code text.
pub trait Post {
    /// A short identifier for this post (e.g. `"grbl"`).
    fn name(&self) -> &str;

    /// What this post's controller can express.
    fn capabilities(&self) -> Capabilities;

    /// Lower `program` to G-code for `machine`, shaped by `options`.
    fn post(
        &self,
        program: &Program,
        machine: &Machine,
        options: &PostOptions,
    ) -> Result<String, PostError>;
}
