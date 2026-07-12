//! # cam-cldata — the controller-neutral cutter-location IR
//!
//! This is the narrow waist of OpenCAMStudio's hourglass: strategies emit
//! CL-data, posts consume it. Between them the IR knows **nothing** about any
//! controller dialect, work offsets, or output units.
//!
//! ## Canonical frame
//!
//! Every coordinate is **millimetres, absolute, in the part/WCS frame**. The IR
//! never applies a work offset (G54…), never chooses G90/G91, never converts to
//! inches — those are a post's job. A [`Program`] is therefore portable across
//! every controller.
//!
//! ## Two tiers
//!
//! - **Tier 1 — primitive moves** ([`Step::Rapid`], [`Step::Linear`],
//!   [`Step::Arc`]) and machine control ([`Step::Dwell`], [`Step::Spindle`], …).
//!   Arcs are **first-class** — not linearised — with an absolute centre, so a
//!   post can emit `I`/`J` and preserve true circular motion.
//! - **Tier 2 — cycle intents** ([`Step::Drill`]). A high-level intent that each
//!   post **lowers per its capabilities**: a Fanuc post emits `G83`/`G80`, while
//!   a grbl post (no canned cycles) expands the same intent into explicit
//!   `G0`/`G1` pecks. The capabilities model doing real work.
//!
//! ## Tags
//!
//! Every motion carries a light [`Tag`] — an operation id plus a [`MoveKind`]
//! (lead-in / cutting / link / retract / plunge). It costs nothing, and drives
//! backplot colouring and correctness checks such as "no rapid through stock".

mod builder;

pub use builder::ProgramBuilder;

/// A point in 3-D space, millimetres, in the part/WCS frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    /// Construct a point from millimetre coordinates.
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// What a motion is *for* — its role in the toolpath. Drives backplot colouring
/// and safety checks (e.g. a [`MoveKind::Link`] rapid must not pass through
/// stock).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveKind {
    /// Approach into the cut (ramp/lead-in).
    LeadIn,
    /// Material-removing motion.
    Cutting,
    /// A non-cutting reposition between cutting motions (usually a rapid).
    Link,
    /// Withdrawal to a safe height.
    Retract,
    /// A vertical entry into the material.
    Plunge,
}

/// A motion's provenance tag: which operation emitted it, and its role.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tag {
    /// Index of the emitting operation within the job (0-based).
    pub op_id: u32,
    /// The motion's role.
    pub kind: MoveKind,
}

impl Tag {
    /// Construct a tag.
    #[inline]
    pub const fn new(op_id: u32, kind: MoveKind) -> Self {
        Self { op_id, kind }
    }
}

/// Spindle rotation sense, looking down the tool axis (`+Z` toward the viewer).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpindleDir {
    /// Clockwise — the usual sense for right-hand cutting tools (`M3`).
    Cw,
    /// Counter-clockwise (`M4`).
    Ccw,
}

/// Coolant state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Coolant {
    /// Flood coolant (`M8`).
    Flood,
    /// Mist coolant (`M7`).
    Mist,
    /// Coolant off (`M9`).
    Off,
}

/// Arc direction in the XY plane, viewed from `+Z`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArcDir {
    /// Clockwise (`G2`).
    Cw,
    /// Counter-clockwise (`G3`).
    Ccw,
}

/// Cutter-radius compensation state, applied by the *controller* (not computed
/// into the geometry). A post that lacks the capability must reject it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CutterComp {
    /// Cancel compensation (`G40`).
    Off,
    /// Compensate to the left of travel (`G41`), reading the tool offset from
    /// the given register.
    Left(u32),
    /// Compensate to the right of travel (`G42`).
    Right(u32),
}

/// Tier-2 drilling cycle intent.
///
/// The tool visits each XY point in [`points`](DrillCycle::points), starting from
/// the [`retract`](DrillCycle::retract) plane, drills from
/// [`z_top`](DrillCycle::z_top) down to the absolute [`depth`](DrillCycle::depth),
/// optionally in [`peck`](DrillCycle::peck) increments with an optional
/// [`dwell`](DrillCycle::dwell) at the bottom, then withdraws. All Z values are
/// absolute millimetres; `retract ≥ z_top > depth`.
#[derive(Clone, Debug, PartialEq)]
pub struct DrillCycle {
    /// XY positions of the holes, in order.
    pub points: Vec<[f64; 2]>,
    /// Absolute Z of the material surface where cutting begins.
    pub z_top: f64,
    /// Absolute Z of the hole bottom (the deepest point).
    pub depth: f64,
    /// Absolute Z of the rapid clearance plane between holes.
    pub retract: f64,
    /// Peck increment (mm, > 0) for interrupted drilling; `None` drills straight.
    pub peck: Option<f64>,
    /// Dwell at the bottom of each hole, in seconds; `None` for no dwell.
    pub dwell: Option<f64>,
    /// Plunge feed rate, mm/min.
    pub feed: f64,
    /// Provenance tag for every motion this cycle expands to.
    pub tag: Tag,
}

/// One step of a CL-data program: a primitive move, a machine-control action, or
/// a Tier-2 cycle intent.
#[derive(Clone, Debug, PartialEq)]
pub enum Step {
    /// Rapid traverse to a point (`G0`) — non-cutting, at the machine's rapid
    /// rate.
    Rapid { to: Point3, tag: Tag },
    /// Linear cutting move to a point (`G1`) at `feed` mm/min.
    Linear { to: Point3, feed: f64, tag: Tag },
    /// Circular/helical move to `end` about the absolute `center` (`G2`/`G3`) at
    /// `feed` mm/min. Helical when `end.z` differs from the start Z.
    Arc {
        end: Point3,
        center: Point3,
        dir: ArcDir,
        feed: f64,
        tag: Tag,
    },
    /// Pause in place for `seconds` (`G4`).
    Dwell { seconds: f64 },
    /// Start the spindle at `rpm` in direction `dir` (`M3`/`M4` + `S`).
    Spindle { rpm: f64, dir: SpindleDir },
    /// Stop the spindle (`M5`).
    SpindleOff,
    /// Set the coolant state (`M7`/`M8`/`M9`).
    Coolant(Coolant),
    /// Change to tool number `tool` (`Tn M6`).
    ToolChange { tool: u32 },
    /// A free-text comment for the operator/backplot.
    Comment(String),
    /// Set the controller's cutter-radius compensation (`G40`/`G41`/`G42`).
    CutterComp(CutterComp),
    /// A Tier-2 drilling cycle intent (lowered per post capabilities).
    Drill(DrillCycle),
}

/// An ordered CL-data program: the complete, controller-neutral description of a
/// job, ready to be handed to any post.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Program {
    /// The steps, in execution order.
    pub steps: Vec<Step>,
}

impl Program {
    /// An empty program.
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a step.
    pub fn push(&mut self, step: Step) {
        self.steps.push(step);
    }

    /// Append every step of `other`, consuming it — used to splice per-operation
    /// fragments into a whole-job program.
    pub fn extend(&mut self, other: Program) {
        self.steps.extend(other.steps);
    }

    /// The steps, in execution order.
    pub fn steps(&self) -> &[Step] {
        &self.steps
    }

    /// Number of steps.
    pub fn len(&self) -> usize {
        self.steps.len()
    }

    /// Whether the program has no steps.
    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }
}
