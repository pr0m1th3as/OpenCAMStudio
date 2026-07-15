//! The document model (P3 first-light slice).
//!
//! A [`Document`] carries a schema version and one [`Setup`]. A [`Setup`] fixes
//! the **heights** (the safety planes), a **stock** description, the available
//! tools, and an ordered list of **operations**. This is deliberately small —
//! enough to drive first light (a profile → G-code) — and grows toward the full
//! `Project → Setup → Stock → Operation → Tool` tree as later phases need it.

use cam_geo::Contour;

use crate::Tool;

/// The document schema version. Bumped when the on-disk model format changes;
/// present from the start so save-files are versioned before there is a loader.
pub const SCHEMA_VERSION: u32 = 1;

/// The safety planes for a setup, all **absolute Z in millimetres**. By
/// convention WCS Z0 is the top of stock, so `top_of_stock` is usually `0.0`.
///
/// Heights are first-class (a core design rule): unsafe Z is a primary hazard
/// and must never be implicit.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Heights {
    /// Z for rapid traverses between features — the highest, safest plane.
    pub clearance: f64,
    /// Z to retract to between passes within a feature (lower than clearance).
    pub retract: f64,
    /// Z of the top of the stock (cutting starts here).
    pub top_of_stock: f64,
}

impl Heights {
    /// A sensible default: clearance and retract above a stock top at Z0.
    pub fn new(clearance: f64, retract: f64, top_of_stock: f64) -> Self {
        Self {
            clearance,
            retract,
            top_of_stock,
        }
    }
}

/// A description of the raw material. The first-light slice models only a
/// rectangular block; `from-model + offsets` stock arrives with the kernel.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Stock {
    /// An axis-aligned block spanning `[min, max]` in each axis (mm).
    Box { min: [f64; 3], max: [f64; 3] },
}

/// Which side of a profiled chain the tool runs on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Side {
    /// Tool outside the closed chain (leaves the enclosed region intact) — the
    /// usual choice for cutting a part free of stock.
    Outside,
    /// Tool inside the closed chain (removes the enclosed region) — e.g. opening
    /// a hole or slot to size.
    Inside,
    /// Tool centre exactly on the chain (engraving/scribing).
    On,
}

impl Side {
    /// Every side, in a stable order — for pickers.
    pub const ALL: [Side; 3] = [Side::Outside, Side::Inside, Side::On];
}

impl std::fmt::Display for Side {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Side::Outside => "Outside",
            Side::Inside => "Inside",
            Side::On => "On",
        })
    }
}

/// The hand of a thread — which way the helix winds. Right-hand is the common
/// case (advances into the work when turned clockwise, viewed from the entering
/// end); left-hand is used where a right-hand thread would loosen in service.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Hand {
    /// Right-hand thread.
    Right,
    /// Left-hand thread.
    Left,
}

impl Hand {
    /// Both hands, in a stable order — for pickers.
    pub const ALL: [Hand; 2] = [Hand::Right, Hand::Left];
}

impl std::fmt::Display for Hand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Hand::Right => "Right-hand",
            Hand::Left => "Left-hand",
        })
    }
}

/// How cutter-radius compensation is applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Comp {
    /// We compute the offset geometry ourselves (kernel-independent). The only
    /// mode for first light.
    Computed,
    /// Left-hand control compensation (`G41`) — deferred to a later phase.
    ControlLeft,
    /// Right-hand control compensation (`G42`) — deferred to a later phase.
    ControlRight,
}

/// How the tool eases onto (lead-in) or off (lead-out) a profiled contour at the
/// start point, to avoid a witness mark from a direct plunge.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Lead {
    /// No lead — plunge directly onto the contour (the default).
    None,
    /// A straight lead of `length` mm, tangent (collinear) to the contour.
    Linear { length: f64 },
    /// A tangent arc of `radius` mm onto/off the contour.
    Arc { radius: f64 },
}

/// How the tool enters the material in Z at the start of each pass.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Plunge {
    /// A straight vertical plunge (the default; fine for drills/centre-cutting mills).
    Straight,
    /// Descend at `angle_deg` from horizontal along the toolpath (linear ramp).
    Ramp { angle_deg: f64 },
    /// Spiral down on a helix of `radius` mm, `pitch` mm of descent per turn.
    Helix { radius: f64, pitch: f64 },
    /// Back-and-forth ramp of `length` mm at `angle_deg`, for narrow slots.
    ZigZag { length: f64, angle_deg: f64 },
}

/// A 2.5-D profiling operation: follow a closed chain at an offset, in stepdown
/// passes, down to a depth.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ProfileOp {
    /// Operation id (0-based within the setup); stamped onto every emitted tag.
    pub id: u32,
    /// Tool number to select from the setup's tool list.
    pub tool: u32,
    /// The closed chain to profile, in the part/WCS frame.
    pub chain: Contour,
    /// Which side of the chain the tool runs on.
    pub side: Side,
    /// How radius compensation is applied.
    pub comp: Comp,
    /// Absolute Z of the final (deepest) pass.
    pub depth: f64,
    /// Maximum depth removed per pass (mm, > 0).
    pub stepdown: f64,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed, mm/min.
    pub plunge_feed: f64,
    /// The chosen start point (part XY), if any: the loop is rotated to begin at
    /// the offset vertex nearest this point. `None` starts at the chain's first
    /// vertex (the default).
    pub start: Option<[f64; 2]>,
    /// Lead-in onto the contour at the start point.
    pub lead_in: Lead,
    /// Lead-out off the contour after the cut.
    pub lead_out: Lead,
    /// How the tool enters the material in Z at each pass.
    pub plunge: Plunge,
}

/// A drilling operation: a set of holes taken to a depth, optionally pecked and
/// dwelled. It is emitted as a Tier-2 cycle intent, so each post lowers it per
/// its capabilities (canned `G83` on Fanuc, explicit pecks on grbl).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DrillOp {
    /// Operation id.
    pub id: u32,
    /// Tool number.
    pub tool: u32,
    /// Hole positions in the part/WCS frame.
    pub points: Vec<[f64; 2]>,
    /// Absolute Z of the hole bottom.
    pub depth: f64,
    /// Peck increment (mm, > 0); `None` drills straight.
    pub peck: Option<f64>,
    /// Dwell at the bottom, seconds; `None` for no dwell.
    pub dwell: Option<f64>,
    /// Plunge feed, mm/min.
    pub feed: f64,
}

/// A 2.5-D pocket-clearing operation: remove all material inside a closed
/// boundary (leaving any islands standing), in concentric offset rings, in
/// stepdown passes, down to a depth.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PocketOp {
    /// Operation id.
    pub id: u32,
    /// Tool number.
    pub tool: u32,
    /// The closed pocket boundary.
    pub boundary: Contour,
    /// Closed islands to leave uncut (holes within the pocket).
    pub islands: Vec<Contour>,
    /// Absolute Z of the pocket floor.
    pub depth: f64,
    /// Maximum depth removed per pass (mm, > 0).
    pub stepdown: f64,
    /// Radial stepover between concentric rings (mm, > 0).
    pub stepover: f64,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed, mm/min.
    pub plunge_feed: f64,
    /// How the tool enters the material in Z at each pass.
    pub plunge: Plunge,
}

/// A facing operation: clear the top of the stock over a boundary with parallel
/// passes, in stepdown passes, down to a depth.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FaceOp {
    /// Operation id.
    pub id: u32,
    /// Tool number.
    pub tool: u32,
    /// The area to face (usually the stock outline).
    pub boundary: Contour,
    /// Absolute Z of the faced surface.
    pub depth: f64,
    /// Maximum depth removed per pass (mm, > 0).
    pub stepdown: f64,
    /// Lateral stepover between parallel passes (mm, > 0).
    pub stepover: f64,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed, mm/min.
    pub plunge_feed: f64,
}

/// A thread-milling operation: cut internal or external threads at a set of hole
/// centres by helically interpolating a thread mill. The thread *form* (a full-
/// profile mill cutting the whole length in one turn vs. a single-form mill
/// stacking one turn per pitch) is chosen by the strategy; this describes the
/// geometry to cut.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThreadOp {
    /// Operation id.
    pub id: u32,
    /// Tool number.
    pub tool: u32,
    /// Thread-hole (or boss) centres in the part/WCS frame.
    pub points: Vec<[f64; 2]>,
    /// Internal thread (bore) when `true`; external thread (boss) when `false`.
    pub internal: bool,
    /// Thread hand — which way the helix winds.
    pub hand: Hand,
    /// Nominal major diameter of the thread, mm.
    pub major_dia: f64,
    /// Thread pitch (Z advance per turn), mm (> 0).
    pub pitch: f64,
    /// Absolute Z of the top of the threaded length.
    pub z_top: f64,
    /// Absolute Z of the bottom of the threaded length.
    pub z_bottom: f64,
    /// Climb-mill when `true`, conventional when `false` — together with `hand`
    /// and `internal` this fixes the helix direction and travel sense.
    pub climb: bool,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed for the approach in Z, mm/min.
    pub plunge_feed: f64,
}

/// An operation in a setup. An enum so a setup holds a heterogeneous, ordered
/// list.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Operation {
    /// A profiling operation.
    Profile(ProfileOp),
    /// A drilling operation.
    Drill(DrillOp),
    /// A pocket-clearing operation.
    Pocket(PocketOp),
    /// A facing operation.
    Face(FaceOp),
    /// A thread-milling operation.
    Thread(ThreadOp),
}

impl Operation {
    /// The operation's id.
    pub fn id(&self) -> u32 {
        match self {
            Operation::Profile(op) => op.id,
            Operation::Drill(op) => op.id,
            Operation::Pocket(op) => op.id,
            Operation::Face(op) => op.id,
            Operation::Thread(op) => op.id,
        }
    }

    /// The number of the tool this operation cuts with.
    pub fn tool(&self) -> u32 {
        match self {
            Operation::Profile(op) => op.tool,
            Operation::Drill(op) => op.tool,
            Operation::Pocket(op) => op.tool,
            Operation::Face(op) => op.tool,
            Operation::Thread(op) => op.tool,
        }
    }
}

/// A machining setup: one fixturing of the stock, its safety planes, and the
/// ordered operations performed in it.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Setup {
    /// Human-readable name.
    pub name: String,
    /// Safety planes.
    pub heights: Heights,
    /// The raw stock.
    pub stock: Stock,
    /// Tools available in this setup.
    pub tools: Vec<Tool>,
    /// Operations, in execution order.
    pub operations: Vec<Operation>,
}

/// The top-level document: a schema version and a setup.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Document {
    /// Schema version this document conforms to.
    pub schema_version: u32,
    /// The setup.
    pub setup: Setup,
}

impl Document {
    /// Wrap a setup in a document stamped with the current [`SCHEMA_VERSION`].
    pub fn new(setup: Setup) -> Self {
        Self {
            schema_version: SCHEMA_VERSION,
            setup,
        }
    }
}
