//! # cam-cldata — the controller-neutral cutter-location IR
//!
//! This is the narrow waist of OpenCAMStudio's hourglass: strategies emit
//! CL-data, posts consume it. Between them the IR knows **nothing** about any
//! controller dialect, work offsets, or output units.
//!
//! ## Canonical frame
//!
//! Every coordinate is **millimetres, absolute, in the part/WCS frame**. The IR
//! never applies a work-offset *value* — the `G54…` shift the operator dials into
//! the control — never chooses G90/G91, never converts to inches: those are a
//! post's job. It *does* carry the work-datum **identity** ([`Step::Datum`]) —
//! which setup an operation belongs to, an abstract 1-based index the post lowers
//! to a dialect code (Okuma `G15 H<n>`, a Fanuc-family post `G54`+(n−1)). Identity
//! is job intent, like a tool number; only the offset value is the post's to own.
//! A [`Program`] is therefore portable across every controller.
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
    /// A planner-inserted rapid up to the **tool-change height** — the lift that
    /// brackets a tool change (`M6`) or a manual reorientation (`M00`), distinct from
    /// an operation's own `Link`/`Retract` so the backplot can render it apart.
    Traverse,
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
    /// Select work coordinate datum `index` (1-based; datum 1 is the default).
    ///
    /// An **abstract** datum identity, never a controller code: the post maps it
    /// to its dialect — Okuma `G15 H<index>`, a Fanuc-family post `G54`+(index−1).
    /// The IR carries *which* setup an operation belongs to, never the offset
    /// *value*; that stays with the post and the operator. Emitted by the planner
    /// only when the effective datum changes, so a single-datum job carries none
    /// and every post's output is unchanged.
    Datum(u32),
    /// A mandatory program stop (`M00`): the machine halts and waits for the operator
    /// to press cycle-start. Emitted before a datum change that is a physical
    /// reorientation, so the operator can re-fixture the part before the next group.
    Stop,
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

    /// A copy of the program with every absolute coordinate shifted by `d` (mm).
    /// Used to re-reference a whole job to a **work origin** before posting (pass
    /// `-origin`): arc `I/J` offsets are relative to the start, so they survive a
    /// uniform shift unchanged; comments/spindle/coolant carry through untouched.
    pub fn translated(&self, d: [f64; 3]) -> Program {
        Program {
            steps: self.steps.iter().map(|s| s.translated(d)).collect(),
        }
    }

    /// A copy re-referenced **per work datum**: each step is shifted by
    /// `offset_for(datum)`, where `datum` is the [`Step::Datum`] index in force at that
    /// step (starting at 1). Pass `|idx| -origin(idx)` to subtract each group's own
    /// origin before posting, so operations under different origins emit relative to
    /// their own zero — the reorientation case. With one datum this is exactly
    /// [`translated`](Self::translated) by `-origin(1)`.
    pub fn translated_per_datum(&self, offset_for: impl Fn(u32) -> [f64; 3]) -> Program {
        let mut datum = 1u32;
        let steps = self
            .steps
            .iter()
            .map(|s| {
                if let Step::Datum(n) = s {
                    datum = *n;
                }
                s.translated(offset_for(datum))
            })
            .collect();
        Program { steps }
    }
}

impl Step {
    /// This step with every absolute coordinate shifted by `d` (mm).
    pub fn translated(&self, d: [f64; 3]) -> Step {
        let p = |q: Point3| Point3::new(q.x + d[0], q.y + d[1], q.z + d[2]);
        match self {
            Step::Rapid { to, tag } => Step::Rapid { to: p(*to), tag: *tag },
            Step::Linear { to, feed, tag } => Step::Linear {
                to: p(*to),
                feed: *feed,
                tag: *tag,
            },
            Step::Arc {
                end,
                center,
                dir,
                feed,
                tag,
            } => Step::Arc {
                end: p(*end),
                center: p(*center),
                dir: *dir,
                feed: *feed,
                tag: *tag,
            },
            Step::Drill(c) => Step::Drill(DrillCycle {
                points: c.points.iter().map(|[x, y]| [x + d[0], y + d[1]]).collect(),
                z_top: c.z_top + d[2],
                depth: c.depth + d[2],
                retract: c.retract + d[2],
                ..c.clone()
            }),
            other => other.clone(),
        }
    }
}

#[cfg(test)]
mod translate_tests {
    use super::*;

    #[test]
    fn translated_shifts_moves_and_arc_end_but_preserves_relative_ij() {
        let prog = ProgramBuilder::new()
            .op(0)
            .feed(300.0)
            .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
            .arc(
                Point3::new(20.0, 10.0, 5.0),
                Point3::new(15.0, 10.0, 5.0),
                ArcDir::Cw,
                MoveKind::Cutting,
            )
            .build();
        let t = prog.translated([-10.0, -10.0, 0.0]);
        match &t.steps()[0] {
            Step::Rapid { to, .. } => assert_eq!(*to, Point3::new(0.0, 0.0, 5.0)),
            other => panic!("{other:?}"),
        }
        match &t.steps()[1] {
            Step::Arc { end, center, .. } => {
                assert_eq!(*end, Point3::new(10.0, 0.0, 5.0), "end shifted");
                assert_eq!(*center, Point3::new(5.0, 0.0, 5.0), "center shifted");
                // I/J = center - start, preserved by a uniform shift (both moved).
                let ij = (center.x - end.x, center.y - end.y);
                assert_eq!(ij, (-5.0, 0.0));
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn translated_per_datum_re_references_each_group_to_its_own_origin() {
        // A move under datum 1, then a datum switch, then a move under datum 2. Each
        // group is shifted by its own origin: datum 1 by −[10,0,0], datum 2 by −[0,20,0].
        let prog = ProgramBuilder::new()
            .op(0)
            .feed(300.0)
            .linear(Point3::new(10.0, 5.0, -1.0), MoveKind::Cutting) // datum 1
            .datum(2)
            .linear(Point3::new(3.0, 25.0, -1.0), MoveKind::Cutting) // datum 2
            .build();
        let t = prog.translated_per_datum(|idx| match idx {
            1 => [-10.0, 0.0, 0.0],
            2 => [0.0, -20.0, 0.0],
            _ => [0.0, 0.0, 0.0],
        });
        // Group 1 move: (10,5) − (10,0) = (0,5).
        match &t.steps()[0] {
            Step::Linear { to, .. } => assert_eq!(*to, Point3::new(0.0, 5.0, -1.0)),
            other => panic!("{other:?}"),
        }
        // Group 2 move (after the Datum marker): (3,25) − (0,20) = (3,5).
        match &t.steps()[2] {
            Step::Linear { to, .. } => assert_eq!(*to, Point3::new(3.0, 5.0, -1.0)),
            other => panic!("{other:?}"),
        }
    }
}
