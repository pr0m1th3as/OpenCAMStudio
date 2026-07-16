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
/// v2: `ToolKind` became a data-carrying enum (per-kind geometry).
/// v3: `Stock` became a part-relative spec (offsets + top + thickness).
/// v4: `Setup` gained a workpiece `origin` (datum) + optional `start_offset`.
pub const SCHEMA_VERSION: u32 = 4;

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
/// rectangular block, defined **relative to the part** so it auto-fits: the
/// part's XY bounding box grown by per-axis offsets, hanging `thickness` below
/// an explicit top Z. The app resolves it to a concrete box with the loaded
/// geometry's bounds ([`Stock::resolve`]). `from-model` stock arrives with the
/// kernel as a further variant.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Stock {
    /// A block sized from the part's XY bounding box: grown by `x_offset` on both
    /// X sides and `y_offset` on both Y sides, spanning from `top` down by
    /// `thickness` (so the bottom sits at `top − thickness`). All in mm.
    BoundingBox {
        x_offset: f64,
        y_offset: f64,
        top: f64,
        thickness: f64,
    },
}

impl Stock {
    /// Resolve to an axis-aligned box `(min, max)` (mm) given the part's XY
    /// bounds. Pure — the caller supplies the bounds (which live outside the
    /// document), keeping the stock definition part-relative and auto-fitting.
    pub fn resolve(&self, min_xy: [f64; 2], max_xy: [f64; 2]) -> ([f64; 3], [f64; 3]) {
        match *self {
            Stock::BoundingBox {
                x_offset,
                y_offset,
                top,
                thickness,
            } => (
                [
                    min_xy[0] - x_offset,
                    min_xy[1] - y_offset,
                    top - thickness,
                ],
                [max_xy[0] + x_offset, max_xy[1] + y_offset, top],
            ),
        }
    }
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
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Lead {
    /// No lead — plunge directly onto the contour (the default).
    #[default]
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
    /// Finishing allowance left on the wall (mm): the profile stops this far short
    /// of the chain edge, on the same side as the tool, so a later finishing
    /// operation can take the whole vertical face in one pass. `0.0` (the default)
    /// profiles to size. Applied on top of the tool-radius offset.
    #[serde(default)]
    pub offset: f64,
    /// Cut depth below the reference plane (Z=0), as a positive magnitude (mm).
    /// The final (deepest) pass sits at absolute Z `-depth`.
    pub depth: f64,
    /// Maximum depth removed per pass (mm, > 0).
    pub stepdown: f64,
    /// Radial stepover for XY roughing passes (mm). `0` (the default) cuts a single
    /// pass at the profile; `> 0` clears the material out to the raw stock in
    /// concentric passes stepping in by this much (like `stepdown`, but in XY),
    /// leaving the finishing `offset` on the wall. Needs the stock bounds; without
    /// them it falls back to a single pass.
    #[serde(default)]
    pub stepover: f64,
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
    /// Distance (mm, >= 0) to keep cutting past the start point before leading off,
    /// so the lead-in/lead-out junction is re-machined and leaves no witness dent.
    /// `0.0` (the default) leads off exactly at the start, as before.
    #[serde(default)]
    pub lead_overlap: f64,
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
    /// Hole depth below the reference plane (Z=0), as a positive magnitude (mm).
    /// The hole bottom sits at absolute Z `-depth`.
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
    /// Pocket depth below the reference plane (Z=0), as a positive magnitude (mm).
    /// The floor sits at absolute Z `-depth`.
    pub depth: f64,
    /// Maximum depth removed per pass (mm, > 0).
    pub stepdown: f64,
    /// Fraction of the tool diameter that adjacent concentric rings overlap (0..1),
    /// as on the face op. The radial ring spacing is `diameter * (1 - overlap)`.
    pub overlap: f64,
    /// Finishing allowance left on every wall (mm): the rings stop this far short of
    /// the boundary and each island, so a later profile finishes the walls to size.
    /// `0.0` (the default) clears to the boundary/island edges. Applied on top of
    /// the tool radius.
    #[serde(default)]
    pub offset: f64,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed, mm/min.
    pub plunge_feed: f64,
    /// How the tool enters the material in Z at each pass.
    pub plunge: Plunge,
    /// Preferred lead-in location (part XY): the clearing begins on the ring point
    /// nearest here, so the plunge/entry witness mark lands where the machinist
    /// chose. `None` uses the strategy's default entry.
    pub start: Option<[f64; 2]>,
    /// Distance (mm, >= 0) each ring keeps cutting past its plunge/start point
    /// before retracting, so the loop-closure junction is re-machined and leaves
    /// no witness dent. `0.0` (the default) closes exactly at the start, as before.
    #[serde(default)]
    pub lead_overlap: f64,
}

/// A principal axis in the XY plane — used to orient facing passes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Axis {
    /// Parallel cutting lines run along X (stepping over in Y).
    #[default]
    X,
    /// Parallel cutting lines run along Y (stepping over in X).
    Y,
}

impl Axis {
    /// Both axes, in a stable order — for pickers.
    pub const ALL: [Axis; 2] = [Axis::X, Axis::Y];
}

impl std::fmt::Display for Axis {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Axis::X => "X",
            Axis::Y => "Y",
        })
    }
}

/// A facing operation: clear the top of the stock over a boundary with parallel
/// passes, in stepdown passes, down to a depth. The passes form a continuous
/// serpentine (one plunge per level, arc turnarounds) rather than lifting between
/// passes.
///
/// Z uses the magnitude convention: `start_offset` is the top cutting plane above
/// the drawing reference (Z=0), `depth` is how far *down* to face as a positive
/// number, so the final faced plane is `start_offset - depth`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FaceOp {
    /// Operation id.
    pub id: u32,
    /// Tool number.
    pub tool: u32,
    /// The area to face (usually the stock outline).
    pub boundary: Contour,
    /// Z of the top cutting plane, above the drawing reference (mm, >= 0). Set
    /// this to face stock that stands proud of Z=0 down toward it; `0` starts the
    /// cut at the reference plane.
    #[serde(default)]
    pub start_offset: f64,
    /// Depth removed *downward* from `start_offset`, as a positive magnitude (mm).
    /// The final faced plane sits at `start_offset - depth`.
    pub depth: f64,
    /// Maximum depth removed per pass (mm, > 0).
    pub stepdown: f64,
    /// Fraction of the tool diameter that adjacent passes overlap (0..1). The pass
    /// spacing is `diameter * (1 - overlap)`, and the first pass is placed so it
    /// cuts a strip of exactly that width along the stock edge.
    #[serde(default)]
    pub overlap: f64,
    /// Distance the tool overshoots past each stock edge (mm, >= 0) before the
    /// 180-degree turnaround arc, so the arc swings clear of the part.
    #[serde(default)]
    pub overshoot: f64,
    /// Orientation of the parallel cutting lines.
    #[serde(default)]
    pub direction: Axis,
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

/// A chamfer/bevel along a closed edge: run a chamfer/V mill around the contour
/// at a computed depth so its cone flank forms a bevel of `width`. A single pass
/// (chamfers are shallow); the tool runs on the air side of the edge, offset by
/// its tip radius.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ChamferOp {
    /// Operation id.
    pub id: u32,
    /// Tool number (must select a chamfer/V mill — the strategy reads its angle).
    pub tool: u32,
    /// The closed edge to chamfer, in the part/WCS frame.
    pub chain: Contour,
    /// Which side of the chain holds material; the tool runs on the other (air)
    /// side, offset by its tip radius.
    pub side: Side,
    /// Horizontal chamfer width, mm (> 0). Together with the tool angle this fixes
    /// the finished bevel; `depth` then chooses which section of the flank cuts it.
    pub width: f64,
    /// Absolute Z of the top edge, where the chamfer begins (usually the stock top).
    pub top: f64,
    /// Tip depth below the top edge, mm — where the tool's bottom edge rides, which
    /// selects the section of the cutting flank used. `0` (or anything up to the
    /// natural `width/tan(α)`) uses the very tip at the bevel bottom (the classic
    /// case). Larger plunges the tip deeper into the air so a higher flank section
    /// cuts the same bevel — useful when the tip section is worn or there is no room
    /// to plunge further with the tip.
    #[serde(default)]
    pub depth: f64,
    /// Chamfer-width increment per pass, mm. `0` (or `>= width`) cuts the whole
    /// bevel in a single pass; otherwise the bevel is reached in steps so the tool
    /// is not overloaded (protects the tool and cleans the cut on wide bevels/hard
    /// stock).
    #[serde(default)]
    pub step: f64,
    /// When `true`, pass widths are sized for **equal material per pass** (widths
    /// `step·√k`) instead of equal width increments, since a fixed width step
    /// removes more material as the bevel widens. `step` sets the first pass.
    #[serde(default)]
    pub gradual: bool,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed for the approach in Z, mm/min.
    pub plunge_feed: f64,
    /// Preferred start location (part XY): the chamfer loop begins on the point
    /// nearest here (where the lead/entry lands). `None` starts at the chain's
    /// first vertex.
    pub start: Option<[f64; 2]>,
    /// Lead-in onto the edge at the start point (eases the tool on, off the air
    /// side). `None` (the default) plunges directly onto the edge, as before.
    #[serde(default)]
    pub lead_in: Lead,
    /// Lead-out off the edge after the cut. `None` (the default) retracts in place.
    #[serde(default)]
    pub lead_out: Lead,
    /// Distance (mm, >= 0) to keep cutting past the start point before leading off,
    /// so the loop-closure junction is re-machined and leaves no witness dent.
    /// `0.0` (the default) closes exactly at the start, as before.
    #[serde(default)]
    pub lead_overlap: f64,
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
    /// A chamfering operation.
    Chamfer(ChamferOp),
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
            Operation::Chamfer(op) => op.id,
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
            Operation::Chamfer(op) => op.tool,
            Operation::Thread(op) => op.tool,
        }
    }

    /// Overwrite the operation's id, whatever its kind.
    pub fn set_id(&mut self, id: u32) {
        match self {
            Operation::Profile(op) => op.id = id,
            Operation::Drill(op) => op.id = id,
            Operation::Pocket(op) => op.id = id,
            Operation::Face(op) => op.id = id,
            Operation::Chamfer(op) => op.id = id,
            Operation::Thread(op) => op.id = id,
        }
    }

    /// Whether two operations describe the **same work** — identical in every
    /// field *except* their `id` (and so the same kind, tool, geometry, depths,
    /// feeds, leads…). Two such operations emit byte-identical toolpaths, so if
    /// both reach the post the machine cuts the same path twice. Used to flag
    /// exact duplicates before export.
    pub fn same_work(&self, other: &Operation) -> bool {
        // Normalise the id on a copy, then lean on the derived structural
        // equality — which automatically covers any field added later.
        let mut probe = self.clone();
        probe.set_id(other.id());
        &probe == other
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
    /// The **workpiece origin** (datum): the part-space point that becomes G-code
    /// `(0,0,0)`. The post subtracts it from every emitted coordinate, so the
    /// operator zeros the machine's work offset (G54) at this point. `[0,0,0]`
    /// means the part frame *is* the program frame. Design/sim stay in part space.
    #[serde(default)]
    pub origin: [f64; 3],
    /// Optional **program start point**, as an offset (mm, part axes) **from the
    /// origin**: the toolpath begins with a rapid to `origin + start_offset`, so
    /// the first motion originates at a known safe spot. `None` starts straight
    /// into the first operation.
    #[serde(default)]
    pub start_offset: Option<[f64; 3]>,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Comp, Lead, Plunge, Side};
    use cam_geo::{Contour, Point};

    fn profile(id: u32) -> Operation {
        Operation::Profile(ProfileOp {
            id,
            tool: 1,
            chain: Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(10.0, 0.0),
                Point::new(10.0, 10.0),
                Point::new(0.0, 10.0),
            ]),
            side: Side::Outside,
            comp: Comp::Computed,
            offset: 0.0,
            depth: 4.0,
            stepdown: 2.0,
            stepover: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            start: None,
            lead_in: Lead::None,
            lead_out: Lead::None,
            lead_overlap: 0.0,
            plunge: Plunge::Straight,
        })
    }

    #[test]
    fn same_work_ignores_id_only() {
        // Equal in everything but id → same work.
        assert!(profile(0).same_work(&profile(7)));
        assert!(profile(7).same_work(&profile(0)));
    }

    #[test]
    fn same_work_sees_a_real_difference() {
        let a = profile(0);
        let mut b = profile(1);
        if let Operation::Profile(op) = &mut b {
            op.feed = 301.0; // one field differs
        }
        assert!(!a.same_work(&b), "a differing feed is different work");
    }

    #[test]
    fn stock_resolves_offsets_and_thickness() {
        let stock = Stock::BoundingBox {
            x_offset: 2.0,
            y_offset: 5.0,
            top: 0.0,
            thickness: 20.0,
        };
        let (min, max) = stock.resolve([10.0, 10.0], [70.0, 50.0]);
        assert_eq!(min, [8.0, 5.0, -20.0], "grown per-axis, bottom = top - thickness");
        assert_eq!(max, [72.0, 55.0, 0.0]);
    }

    #[test]
    fn same_work_across_kinds_is_false() {
        let prof = profile(0);
        let drill = Operation::Drill(DrillOp {
            id: 0,
            tool: 1,
            points: vec![[0.0, 0.0]],
            depth: 4.0,
            peck: None,
            dwell: None,
            feed: 100.0,
        });
        assert!(!prof.same_work(&drill));
    }
}
