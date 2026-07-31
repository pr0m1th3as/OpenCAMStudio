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

mod dialect;
mod fanuc;
mod grbl;
mod okuma;
mod words;
mod writer;

pub use fanuc::FanucPost;
pub use grbl::GrblPost;
pub use okuma::OkumaPost;

use core::fmt;

use cam_cldata::{Point3, Program, Step};
use cam_model::Machine;

/// A selectable post/controller dialect, for the export picker. Each maps to a
/// [`dialect::Dialect`] that drives emission.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PostKind {
    /// grbl (Arduino/ESP32 hobby control).
    #[default]
    Grbl,
    /// FluidNC (grbl for ESP32).
    FluidNc,
    /// grblHAL (32-bit grbl with real tool-change).
    GrblHal,
    /// LinuxCNC (RS-274NGC).
    LinuxCnc,
    /// Fanuc (industrial standard).
    Fanuc,
    /// Haas (Fanuc-family job-shop control).
    Haas,
    /// Okuma OSP — a fourth output family, not a Fanuc parameterisation (`G15 H`
    /// work offsets, `G56 H` tool length, `M02` end, `G71`/`M53` cycles). See
    /// [`okuma`].
    Okuma,
}

impl PostKind {
    /// Every post, in a stable order — for the picker.
    pub const ALL: [PostKind; 7] = [
        PostKind::Grbl,
        PostKind::FluidNc,
        PostKind::GrblHal,
        PostKind::LinuxCnc,
        PostKind::Fanuc,
        PostKind::Haas,
        PostKind::Okuma,
    ];

    /// The flat-knob [`dialect::Dialect`] backing this post, or `None` for families
    /// (Okuma) whose frame is too divergent for the shared walker and carry their own
    /// emitter instead.
    fn dialect(self) -> Option<&'static dialect::Dialect> {
        Some(match self {
            PostKind::Grbl => &dialect::GRBL,
            PostKind::FluidNc => &dialect::FLUIDNC,
            PostKind::GrblHal => &dialect::GRBLHAL,
            PostKind::LinuxCnc => &dialect::LINUXCNC,
            PostKind::Fanuc => &dialect::FANUC,
            PostKind::Haas => &dialect::HAAS,
            PostKind::Okuma => return None,
        })
    }

    /// The display label for the picker.
    fn label(self) -> &'static str {
        match self.dialect() {
            Some(d) => d.name,
            None => "Okuma",
        }
    }

    /// How this control names work datum `datum` (1-based) in a program based at
    /// `base` — `Some("G55")`, `Some("H2")` — or `None` when the datum has no word on
    /// this control, which is exactly when [`post`](Self::post) would refuse it.
    ///
    /// This exists so the UI can *show* the datum an origin will become without
    /// re-deriving it: the ISO arm runs the same [`WorkOffset::advanced_by`] the shared
    /// walker emits from, so a label and the posted file cannot disagree. Two rules,
    /// not one — Okuma's `G15 H<n>` takes the index literally and is independent of
    /// `base`, while the ISO families count up from the program's own work offset —
    /// which is why a `match` written a second time in the GUI would be wrong.
    pub fn datum_label(self, datum: u32, base: WorkOffset) -> Option<String> {
        let steps_up = datum.checked_sub(1)?;
        match self {
            // OSP work-coordinate select; the emitter states any index it is given, so
            // no ceiling is claimed here that the post would not honour.
            PostKind::Okuma => Some(format!("H{datum}")),
            PostKind::Grbl
            | PostKind::FluidNc
            | PostKind::GrblHal
            | PostKind::LinuxCnc
            | PostKind::Fanuc
            | PostKind::Haas => base
                .advanced_by(steps_up as usize)
                .map(|o| o.code().to_string()),
        }
    }

    /// The conventional file extension(s) for this dialect's programs, for the
    /// export dialog's filter. Okuma OSP programs are `.MIN`; the rest are `.nc`.
    pub fn file_extensions(self) -> &'static [&'static str] {
        match self {
            PostKind::Okuma => &["min"],
            _ => &["nc"],
        }
    }

    /// The default file name the export dialog is seeded with — carries the
    /// dialect's conventional extension (see [`file_extensions`](Self::file_extensions)).
    pub fn default_file_name(self) -> &'static str {
        match self {
            PostKind::Okuma => "program.min",
            _ => "program.nc",
        }
    }

    /// Post `program` in this dialect.
    pub fn post(
        self,
        program: &Program,
        machine: &Machine,
        options: &PostOptions,
    ) -> Result<String, PostError> {
        match self.dialect() {
            Some(d) => dialect::emit(program, machine, options, d),
            None => okuma::emit(program, machine, options),
        }
    }
}

impl fmt::Display for PostKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

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
    /// The six work coordinate systems in machine order. Every ISO-family control we
    /// target carries exactly these; past them the families diverge — `G54.1 P<n>` on
    /// Fanuc/Haas (an *option*, not always fitted), `G59.1`–`G59.3` on LinuxCNC,
    /// nothing at all on grbl — so a per-operation datum stops here rather than guess.
    pub const ALL: [WorkOffset; 6] = [
        WorkOffset::G54,
        WorkOffset::G55,
        WorkOffset::G56,
        WorkOffset::G57,
        WorkOffset::G58,
        WorkOffset::G59,
    ];

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

    /// This offset's position in [`WorkOffset::ALL`] — 0 for `G54`.
    pub fn index(self) -> usize {
        match self {
            WorkOffset::G54 => 0,
            WorkOffset::G55 => 1,
            WorkOffset::G56 => 2,
            WorkOffset::G57 => 3,
            WorkOffset::G58 => 4,
            WorkOffset::G59 => 5,
        }
    }

    /// The work offset `steps` beyond this one, or `None` past `G59`.
    ///
    /// This is how a per-operation work datum reaches a G-code word: datum *n* is the
    /// (*n*−1)-th offset after the program's base, so datum 1 *is* the base and a
    /// single-datum program states nothing beyond its preamble.
    pub fn advanced_by(self, steps: usize) -> Option<WorkOffset> {
        WorkOffset::ALL.get(self.index() + steps).copied()
    }

    /// How many work offsets are reachable from this one, itself included.
    pub fn remaining(self) -> usize {
        WorkOffset::ALL.len() - self.index()
    }
}

#[cfg(test)]
mod work_offset_tests {
    use super::WorkOffset;

    #[test]
    fn all_and_index_agree() {
        // `index` is a hand-written match, `ALL` a hand-written array; nothing but this
        // keeps them in step, and `advanced_by` is wrong in silence if they drift.
        for (i, o) in WorkOffset::ALL.iter().enumerate() {
            assert_eq!(o.index(), i, "{} sits at {i} in ALL", o.code());
        }
    }

    #[test]
    fn advancing_walks_the_table_and_stops_at_g59() {
        assert_eq!(WorkOffset::G54.advanced_by(0), Some(WorkOffset::G54));
        assert_eq!(WorkOffset::G54.advanced_by(5), Some(WorkOffset::G59));
        assert_eq!(WorkOffset::G54.advanced_by(6), None);
        // From a raised base the ceiling arrives sooner — six offsets exist, not six
        // *beyond* wherever the program started.
        assert_eq!(WorkOffset::G57.advanced_by(2), Some(WorkOffset::G59));
        assert_eq!(WorkOffset::G57.advanced_by(3), None);
        assert_eq!(WorkOffset::G57.remaining(), 3);
    }
}

/// Knobs that shape a post's output without changing its meaning.
#[derive(Clone, Debug, PartialEq)]
pub struct PostOptions {
    /// Work coordinate system to select in the preamble — and, for a multi-fixture
    /// job, the *base* the per-operation datums count up from: datum 1 is this
    /// offset, datum 2 the next, and so on to `G59`. (Okuma is unaffected; its
    /// `G15 H<n>` is a separate number space and takes the datum index directly.)
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
    /// The toolpath is larger than the machine's travel on some axis. Reported as
    /// a *span*, not an absolute coordinate: the operator's work offset (G54) can
    /// place the datum anywhere in travel, so only the size has to fit.
    TravelExceeded { axis: char, span: f64, travel: f64 },
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
            PostError::TravelExceeded { axis, span, travel } => {
                write!(
                    f,
                    "toolpath spans {span:.1} mm in {axis}, over the machine's {travel:.1} mm of travel"
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

/// Grow `min`/`max` to include `p`.
fn expand(min: &mut Point3, max: &mut Point3, p: Point3) {
    min.x = min.x.min(p.x);
    min.y = min.y.min(p.y);
    min.z = min.z.min(p.z);
    max.x = max.x.max(p.x);
    max.y = max.y.max(p.y);
    max.z = max.z.max(p.z);
}

/// The XYZ bounding box of every tool position in the program, or `None` if it has
/// no moves.
fn program_bounds(program: &Program) -> Option<(Point3, Point3)> {
    let mut min = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    let mut any = false;
    for step in program.steps() {
        match step {
            Step::Rapid { to, .. } | Step::Linear { to, .. } => {
                expand(&mut min, &mut max, *to);
                any = true;
            }
            Step::Arc { end, .. } => {
                expand(&mut min, &mut max, *end);
                any = true;
            }
            Step::Drill(c) => {
                for pt in &c.points {
                    expand(&mut min, &mut max, Point3::new(pt[0], pt[1], c.retract));
                    expand(&mut min, &mut max, Point3::new(pt[0], pt[1], c.depth));
                    any = true;
                }
            }
            _ => {}
        }
    }
    any.then_some((min, max))
}

/// Verify the toolpath fits within the machine's **travel** on every axis — its
/// span, not its absolute position (the operator's work offset places the datum
/// within travel, so a program in work coordinates only needs to fit by size).
pub(crate) fn check_travel(program: &Program, machine: &Machine) -> Result<(), PostError> {
    let Some((min, max)) = program_bounds(program) else {
        return Ok(());
    };
    let (ex, ey, ez) = machine.envelope.extent();
    const EPS: f64 = 1e-6;
    for (axis, span, travel) in [
        ('X', max.x - min.x, ex),
        ('Y', max.y - min.y, ey),
        ('Z', max.z - min.z, ez),
    ] {
        if span > travel + EPS {
            return Err(PostError::TravelExceeded { axis, span, travel });
        }
    }
    Ok(())
}

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
