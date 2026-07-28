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
/// v5: `Tool` gained nominal cutting data (`nominal_rpm`/`nominal_feed`/
///     `nominal_plunge_feed`) and each `Operation` gained a per-op `spindle_rpm`.
///     All are `#[serde(default)]`, so this bump is a record, not a load barrier —
///     v4 and earlier files still open (the new fields default to 0).
/// v6: each `Operation` gained a per-op `work_offset` — the 1-based work-datum
///     index for multi-WCS output (Okuma `G15 H<n>`). `#[serde(default =
///     "default_work_offset")]` → 1, so v5 and earlier files still open on datum 1.
/// v7: `Setup` gained a work-datum `work_offsets` registry (`Vec<Datum>`) and an
///     optional `replication` order (Workflow A). Both `#[serde(default)]` — the
///     registry defaults to a single base datum — so v6 and earlier files still open.
pub const SCHEMA_VERSION: u32 = 7;

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
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Plunge {
    /// A straight vertical plunge (the default; fine for drills/centre-cutting mills).
    #[default]
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
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
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
    /// Constant-engagement clearing parameters for outside-roughing (used when
    /// `stepover > 0`). `engagement <= 0` (the default) keeps plain concentric
    /// roughing.
    #[serde(default)]
    pub clearing: Clearing,
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
    /// Hole depth (mm, a positive magnitude) measured **down from where the hole
    /// starts** — i.e. from `top_of_stock + start_offset`. The bottom sits at
    /// absolute Z `top_of_stock + start_offset - depth`.
    pub depth: f64,
    /// Where the hole begins, as a height **above the stock top** (mm), matching the
    /// Face convention. `0` (the default) starts at the stock top; positive starts it
    /// above the surface (a proud boss); negative below (a recessed/faced surface).
    /// Depth is measured down from here.
    #[serde(default)]
    pub start_offset: f64,
    /// Peck increment (mm, > 0); `None` drills straight.
    pub peck: Option<f64>,
    /// Dwell at the bottom, seconds; `None` for no dwell.
    pub dwell: Option<f64>,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Plunge feed, mm/min.
    pub feed: f64,
}

/// Default for a `#[serde(default)]` boolean field that should default to `true`.
pub(crate) fn default_true() -> bool {
    true
}

/// Default for a per-operation `work_offset`: datum 1, the base work coordinate
/// system. 1-based so a missing field (v5 and earlier files) opens on datum 1.
pub(crate) fn default_work_offset() -> u32 {
    1
}

/// Constant-engagement (trochoidal) clearing parameters, shared by pocket clearing
/// and profile outside-roughing. When `engagement > 0` the region is cleared with a
/// path that keeps the tool's radial engagement bounded (higher feeds, kinder to the
/// tool); `engagement <= 0` (the default) falls back to plain concentric-ring
/// clearing, so existing documents are unchanged.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Clearing {
    /// Maximum radial width of cut (mm) — the engagement cap. Bounds how much fresh
    /// material the tool takes at once. `<= 0` disables adaptive clearing.
    #[serde(default)]
    pub engagement: f64,
    /// Climb milling (`true`, the default) vs conventional (`false`) for the
    /// clearing path.
    #[serde(default = "default_true")]
    pub climb: bool,
}

impl Default for Clearing {
    fn default() -> Self {
        Self {
            engagement: 0.0,
            climb: true,
        }
    }
}

/// The parameters of an **area-clearing pass** — everything a pocket takes except its
/// geometry and depth.
///
/// Factored out because a carve's clearing pass is not a reduced pocket, it is a pocket
/// over a derived region: the same engine, the same controls, the same expectations. A
/// set of `clear_*` scalars copied beside it would drift from the real thing the first
/// time either gained a control the other did not.
///
/// [`PocketOp`] still carries these flat, and should adopt this struct as a deliberate
/// schema bump with a migration — its fields are already on disk in saved projects,
/// which this struct's first user ([`CarveOp`]) is not.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ClearParams {
    /// Maximum depth removed per pass, mm. `0` clears the full depth in one pass.
    #[serde(default)]
    pub stepdown: f64,
    /// Fraction of the tool diameter that adjacent rings overlap (0..1). The radial
    /// ring spacing is `diameter * (1 - overlap)`.
    #[serde(default)]
    pub overlap: f64,
    /// Finishing allowance left on the walls, mm.
    ///
    /// In a pocket this skin is taken off later by a finishing profile. In a **carve**
    /// it is how far the end mill stays off the carved surface, leaving that skin for
    /// the V-bit — which finishes it better, with the flank of its cone rather than the
    /// corner of a cylinder. Nothing is left behind: the V-bit's floor pass is computed
    /// against what the clearing tool *actually swept*, so a larger allowance simply
    /// hands more of the work back to it.
    #[serde(default)]
    pub offset: f64,
    /// Cutting feed, mm/min.
    #[serde(default)]
    pub feed: f64,
    /// Plunge feed, mm/min.
    #[serde(default)]
    pub plunge_feed: f64,
    /// How the tool enters the material in Z at each pass.
    #[serde(default)]
    pub plunge: Plunge,
    /// Lead-in onto the finished walls, eased from the cleared interior.
    #[serde(default)]
    pub lead_in: Lead,
    /// Lead-out off the walls after the finishing pass.
    #[serde(default)]
    pub lead_out: Lead,
    /// Distance each ring keeps cutting past its start before retracting, mm, so the
    /// loop-closure junction leaves no witness dent.
    #[serde(default)]
    pub lead_overlap: f64,
    /// Constant-engagement clearing parameters (engagement cap + climb).
    #[serde(default)]
    pub clearing: Clearing,
}

impl Default for ClearParams {
    fn default() -> Self {
        Self {
            stepdown: 0.0,
            // Half the diameter: the same spacing a pocket's own default overlap gives.
            overlap: 0.5,
            offset: 0.0,
            feed: 0.0,
            plunge_feed: 0.0,
            plunge: Plunge::Straight,
            lead_in: Lead::None,
            lead_out: Lead::None,
            lead_overlap: 0.0,
            clearing: Clearing::default(),
        }
    }
}

/// A carve's optional **clearing pass**: which tool clears the flat land, and how.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CarveClearing {
    /// The flat-bottomed tool that clears the flat land at full depth, before the
    /// V-bit runs.
    pub tool: u32,
    /// How it clears. Feeds of `0` inherit the carve's own.
    #[serde(default)]
    pub params: ClearParams,
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
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
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
    /// Lead-in onto the finished walls (boundary and islands), eased in from the
    /// cleared interior so a one-pass wall finish leaves no witness. `None` (the
    /// default) plunges/links straight onto the wall, as before.
    #[serde(default)]
    pub lead_in: Lead,
    /// Lead-out off the walls after the finishing pass. `None` (the default) leaves
    /// in place.
    #[serde(default)]
    pub lead_out: Lead,
    /// Constant-engagement clearing parameters. `engagement <= 0` (the default)
    /// keeps the plain concentric clearing.
    #[serde(default)]
    pub clearing: Clearing,
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

    /// Infer a facing pass direction from **the boundary edge the user clicked**:
    /// the passes run parallel to the edge nearest `pick`, snapped to the nearer
    /// principal axis. This is the intent when a boundary is picked by clicking one
    /// of its edges — face along the side you pointed at. Degenerate input (fewer
    /// than two vertices) falls back to `X`.
    pub fn along_edge_at(pts: &[cam_geo::Point], pick: cam_geo::Point) -> Axis {
        let n = pts.len();
        if n < 2 {
            return Axis::X;
        }
        let mut best_d2 = f64::MAX;
        let mut axis = Axis::X;
        for i in 0..n {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            let d2 = point_seg_dist_sq(pick, a, b);
            if d2 < best_d2 {
                best_d2 = d2;
                let (dx, dy) = ((b.x - a.x).abs(), (b.y - a.y).abs());
                axis = if dx >= dy { Axis::X } else { Axis::Y };
            }
        }
        axis
    }

    /// Infer a facing pass direction from a boundary's **longest edge**, snapped to
    /// the nearer principal axis — the fallback when no pick point is available.
    /// Facing along the long side gives the fewest, longest passes. Degenerate input
    /// falls back to `X`.
    pub fn along_longest_edge(pts: &[cam_geo::Point]) -> Axis {
        let n = pts.len();
        if n < 2 {
            return Axis::X;
        }
        let mut best = f64::MIN;
        let mut axis = Axis::X;
        for i in 0..n {
            let a = pts[i];
            let b = pts[(i + 1) % n];
            let (dx, dy) = ((b.x - a.x).abs(), (b.y - a.y).abs());
            let len2 = dx * dx + dy * dy;
            if len2 > best {
                best = len2;
                axis = if dx >= dy { Axis::X } else { Axis::Y };
            }
        }
        axis
    }
}

/// Squared distance from `p` to the segment `a→b` (clamped to the endpoints).
fn point_seg_dist_sq(p: cam_geo::Point, a: cam_geo::Point, b: cam_geo::Point) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    if len2 <= f64::EPSILON {
        return p.distance_sq(a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
    p.distance_sq(cam_geo::Point::new(a.x + dx * t, a.y + dy * t))
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
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
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
    /// Number of **radial infeed passes**: the thread is cut to full depth in this
    /// many equal radial steps (each a full helix, stepping the orbit outward for an
    /// internal thread / inward for an external one). `1` (the default) cuts the whole
    /// depth in a single pass; more passes lighten the cut for hard material.
    #[serde(default)]
    pub passes: u32,
    /// Extra **spring passes** at the final (full) depth, to clean up elastic spring-back
    /// after the last cutting pass. `0` (the default) adds none.
    #[serde(default)]
    pub spring_passes: u32,
    /// For an internal **blind** hole: how far (mm) the pre-drilled hole extends *below*
    /// the thread bottom (`z_bottom`). `0` (the default) means a **through hole** — no
    /// blind-bottom check. When positive it must be at least [`blind_allowance`](Self::
    /// blind_allowance), else the operation errors (the tool cannot thread flush to a
    /// blind bottom).
    #[serde(default)]
    pub drill_clearance: f64,
    /// Required standoff (mm) between the last thread and the bottom of a blind hole; the
    /// [`drill_clearance`](Self::drill_clearance) is validated against it. `0` (the
    /// default) means *auto* — one thread pitch.
    #[serde(default)]
    pub blind_allowance: f64,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
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
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
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

/// A **V-carve engraving** operation: run a V-bit along a path with its tip *in* the
/// material, so the cone ploughs a V-section groove.
///
/// Distinct from [`ChamferOp`] in three ways that matter:
///
/// - **The tool must be a V-bit** ([`crate::ToolKind::VBit`]), never a chamfer mill —
///   a chamfer mill's bottom is a *non-cutting* flat, so engraving with one would rub
///   rather than cut. The strategy rejects it outright.
/// - **No side offset.** A chamfer runs beside an edge (offset to the air side); an
///   engraving groove is centred *on* the path, so the tool axis follows the chain
///   exactly, with no radius compensation.
/// - **The path may be open.** Lettering and decorative strokes are open polylines,
///   not closed regions — see `closed`.
///
/// The groove's width follows from `depth` and the tool's geometry
/// ([`cam_geo::vtip_half_width`]); it is not specified independently.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EngraveOp {
    /// Operation id.
    pub id: u32,
    /// Tool number — must select a V-bit (the strategy reads its angle and tip radius).
    pub tool: u32,
    /// The path to engrave, in the part/WCS frame. The tool centre follows it exactly.
    pub chain: Contour,
    /// Whether `chain` is a closed loop (the last vertex joins back to the first) or an
    /// **open** stroke that stops at the last vertex. [`Contour`] is implicitly closed,
    /// so open strokes — the common case for lettering — are carried by this flag.
    /// Defaults to `false`: an engraved chain is a stroke unless said otherwise.
    #[serde(default)]
    pub closed: bool,
    /// Absolute Z of the surface being engraved (usually the stock top).
    pub top: f64,
    /// Engraving depth below `top`, mm (> 0), as a positive magnitude. The groove
    /// width follows from this and the tool's V angle and tip radius.
    pub depth: f64,
    /// Maximum depth per pass, mm. `0` (the default) cuts the full `depth` in one
    /// pass — normal for shallow engraving; a deeper groove steps down.
    #[serde(default)]
    pub stepdown: f64,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the tool's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed for the approach in Z, mm/min.
    pub plunge_feed: f64,
    /// Preferred start location (part XY) for a **closed** path: the loop begins at the
    /// vertex nearest here. Ignored for an open stroke, which must start at its own
    /// first vertex. `None` starts at the chain's first vertex.
    #[serde(default)]
    pub start: Option<[f64; 2]>,
}

/// A **V-carving** operation: the boundary outlines an *area* to carve, and the V-bit's
/// flanks land on that boundary rather than its tip running along it.
///
/// The relation to [`EngraveOp`] is exactly Profile → Pocket: an engraving *is* the path
/// it follows, a carving *consumes* the region a path encloses.
///
/// - **The tool never touches the boundary.** It follows inward offsets of it, and at
///   inward distance `w` it sits at the depth where its groove is `2·w` wide
///   ([`cam_geo::vtip_depth_for_half_width`]). So the carved wall meets the boundary at
///   the surface, and the depth *varies* with the shape.
/// - **Depth is derived, not dictated.** `depth` is a **cap**: the carve reaches it only
///   where the region is wide enough. Where the region is wider still, a flat land
///   remains at `depth` — which is what `clear_tool` is for.
/// - **The path must be closed.** There is no interior to consume otherwise.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CarveOp {
    /// Operation id.
    pub id: u32,
    /// The V-bit — the defining tool, chosen in the creation wizard. The strategy reads
    /// its half-angle and tip radius; the carve's shape follows from them.
    pub tool: u32,
    /// The clearing pass that removes the flat land at full depth **before** the V-bit
    /// runs. `None` means the V-bit does the lot, which cuts but leaves a **ridged**
    /// floor — a cone cannot leave a flat one — and is warned about, not rejected.
    #[serde(default)]
    pub clear: Option<CarveClearing>,
    /// The closed boundary of the region to carve, in the part/WCS frame.
    pub boundary: Contour,
    /// Closed islands left uncut — holes, and the counters of letters. Auto-populated
    /// from the picked region's nesting.
    #[serde(default)]
    pub islands: Vec<Contour>,
    /// Absolute Z of the surface being carved (usually the stock top).
    pub top: f64,
    /// **Maximum** depth below `top`, mm (> 0), as a positive magnitude. The shape sets
    /// the actual depth everywhere; this caps it.
    pub depth: f64,
    /// Hold-off from the boundary, mm. Positive leaves material (same sign convention as
    /// [`ProfileOp::offset`]), shrinking the carved region.
    #[serde(default)]
    pub offset: f64,
    /// Radial spacing of the **wall** rings, mm — a *roughing* control, not a finish
    /// one.
    ///
    /// The finished wall is cut by the deepest ring alone: that pass's flank spans from
    /// the boundary down to its tip, so every shallower ring cuts a narrower V entirely
    /// inside it. They exist to limit how much material one pass takes, and to reach
    /// into convex corners, which the deepest ring cannot. A coarser step therefore
    /// costs tool load, not surface quality. `0` (the default) derives one.
    #[serde(default)]
    pub ring_step: f64,
    /// Target ridge height left on the **flat floor**, mm — the true finish control.
    ///
    /// Where the floor is flat but the tool is a cone, adjacent passes leave a ridge of
    /// `vtip_depth_for_half_width(half the spacing)` between them. Asking for a ridge
    /// height instead of a spacing lets the spacing open up wherever the tool's geometry
    /// allows: a rounded tip clears a much wider band at the same ridge than a sharp one
    /// does. `0` (the default) uses a sensible fine value.
    #[serde(default)]
    pub scallop: f64,
    /// Spindle speed for this operation, rpm (`M3 S<rpm>`). Seeded from the V-bit's
    /// nominal RPM when the operation is created; `0.0` falls back to the job default.
    /// Applies to the whole carve, including any clearing pass.
    #[serde(default)]
    pub spindle_rpm: f64,
    /// Work-datum index for this operation (1-based; datum 1 is the default). The
    /// post lowers it to a dialect code — Okuma `G15 H<n>` — so ops on different
    /// fixtures/setups emit under different work coordinate systems.
    #[serde(default = "default_work_offset")]
    pub work_offset: u32,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed for the approach in Z, mm/min.
    pub plunge_feed: f64,
    /// Link the rings **without lifting** where it is safe to do so, instead of
    /// retracting to clearance and plunging afresh for every ring.
    ///
    /// A carve can run to hundreds of rings, and a retract cycle each costs far more
    /// time than the cut. Staying down is safe because the tool at a ring's own depth
    /// never cuts below the intended surface anywhere at or beyond that ring's inward
    /// distance — see the strategy's module docs for why — and the strategy verifies
    /// each individual link against that region, retracting for the ones that fail.
    ///
    /// `false` (the serde default, the conservative reading of an absent field) lifts
    /// between every ring.
    #[serde(default)]
    pub stay_down: bool,
    /// Preferred start location (part XY): each ring begins at the point nearest here.
    /// `None` uses the strategy's default entry.
    #[serde(default)]
    pub start: Option<[f64; 2]>,
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
    /// A V-carve engraving operation.
    Engrave(EngraveOp),
    /// A V-carving operation over a region.
    Carve(CarveOp),
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
            Operation::Engrave(op) => op.id,
            Operation::Carve(op) => op.id,
        }
    }

    /// The number of the operation's **defining** tool — the one that does the work the
    /// operation is named for. An operation may use more than one; see [`tools`](Self::tools).
    pub fn tool(&self) -> u32 {
        match self {
            Operation::Profile(op) => op.tool,
            Operation::Drill(op) => op.tool,
            Operation::Pocket(op) => op.tool,
            Operation::Face(op) => op.tool,
            Operation::Chamfer(op) => op.tool,
            Operation::Thread(op) => op.tool,
            Operation::Engrave(op) => op.tool,
            Operation::Carve(op) => op.tool,
        }
    }

    /// Every tool the operation uses, **in cutting order** — so `tools()[0]` is the tool
    /// that must be in the spindle when the operation's fragment begins, and the last is
    /// the one left there when it ends.
    ///
    /// All single-tool operations return just [`tool`](Self::tool). Only [`CarveOp`]
    /// returns two, and only when its clearing tool is set: the end mill clears the flat
    /// land *first*, then the V-bit carves. The operation's own strategy emits the
    /// intervening [`cam_cldata::Step::ToolChange`], since it alone knows the order.
    pub fn tools(&self) -> Vec<u32> {
        match self {
            Operation::Carve(op) => match op.clear.map(|c| c.tool) {
                Some(clear) if clear != op.tool => vec![clear, op.tool],
                _ => vec![op.tool],
            },
            other => vec![other.tool()],
        }
    }

    /// The operation's commanded spindle speed, rpm. `0.0` means "unset" — the
    /// planner falls back to the job default rather than commanding a zero.
    pub fn spindle_rpm(&self) -> f64 {
        match self {
            Operation::Profile(op) => op.spindle_rpm,
            Operation::Drill(op) => op.spindle_rpm,
            Operation::Pocket(op) => op.spindle_rpm,
            Operation::Face(op) => op.spindle_rpm,
            Operation::Chamfer(op) => op.spindle_rpm,
            Operation::Thread(op) => op.spindle_rpm,
            Operation::Engrave(op) => op.spindle_rpm,
            Operation::Carve(op) => op.spindle_rpm,
        }
    }

    /// Set the operation's commanded spindle speed, rpm, whatever its kind.
    pub fn set_spindle_rpm(&mut self, rpm: f64) {
        match self {
            Operation::Profile(op) => op.spindle_rpm = rpm,
            Operation::Drill(op) => op.spindle_rpm = rpm,
            Operation::Pocket(op) => op.spindle_rpm = rpm,
            Operation::Face(op) => op.spindle_rpm = rpm,
            Operation::Chamfer(op) => op.spindle_rpm = rpm,
            Operation::Thread(op) => op.spindle_rpm = rpm,
            Operation::Engrave(op) => op.spindle_rpm = rpm,
            Operation::Carve(op) => op.spindle_rpm = rpm,
        }
    }

    /// The operation's work-datum index (1-based). Datum 1 is the base work
    /// coordinate system; higher values select further fixtures/setups, which a
    /// post lowers to its dialect (Okuma `G15 H<n>`).
    pub fn work_offset(&self) -> u32 {
        match self {
            Operation::Profile(op) => op.work_offset,
            Operation::Drill(op) => op.work_offset,
            Operation::Pocket(op) => op.work_offset,
            Operation::Face(op) => op.work_offset,
            Operation::Chamfer(op) => op.work_offset,
            Operation::Thread(op) => op.work_offset,
            Operation::Engrave(op) => op.work_offset,
            Operation::Carve(op) => op.work_offset,
        }
    }

    /// Set the operation's work-datum index (1-based), whatever its kind.
    pub fn set_work_offset(&mut self, work_offset: u32) {
        match self {
            Operation::Profile(op) => op.work_offset = work_offset,
            Operation::Drill(op) => op.work_offset = work_offset,
            Operation::Pocket(op) => op.work_offset = work_offset,
            Operation::Face(op) => op.work_offset = work_offset,
            Operation::Chamfer(op) => op.work_offset = work_offset,
            Operation::Thread(op) => op.work_offset = work_offset,
            Operation::Engrave(op) => op.work_offset = work_offset,
            Operation::Carve(op) => op.work_offset = work_offset,
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
            Operation::Engrave(op) => op.id = id,
            Operation::Carve(op) => op.id = id,
        }
    }

    /// Overwrite the number of the operation's **defining** tool, whatever its kind.
    /// Secondary tools are untouched — to rewrite *every* reference (renumbering), use
    /// [`map_tools`](Self::map_tools).
    pub fn set_tool(&mut self, tool: u32) {
        match self {
            Operation::Profile(op) => op.tool = tool,
            Operation::Drill(op) => op.tool = tool,
            Operation::Pocket(op) => op.tool = tool,
            Operation::Face(op) => op.tool = tool,
            Operation::Chamfer(op) => op.tool = tool,
            Operation::Thread(op) => op.tool = tool,
            Operation::Engrave(op) => op.tool = tool,
            Operation::Carve(op) => op.tool = tool,
        }
    }

    /// Rewrite **every** tool reference the operation holds through `f` — the defining
    /// tool and any secondary one. Used by tool-number reconciliation (see
    /// [`crate::reconcile`]) when a setup adopts the shop's canonical numbering; a
    /// secondary reference left behind would dangle or, worse, silently point at a
    /// different tool that inherited the old number.
    pub fn map_tools(&mut self, f: impl Fn(u32) -> u32) {
        let t = f(self.tool());
        self.set_tool(t);
        if let Operation::Carve(op) = self {
            if let Some(c) = &mut op.clear {
                c.tool = f(c.tool);
            }
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

/// How a work datum is reached — and therefore whether switching to it is free.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DatumKind {
    /// Several datums mounted at once (multiple vises, a router grid): switching is a
    /// taught offset, no operator action. The kind used by replication (Workflow A).
    #[default]
    Simultaneous,
    /// Reaching this datum needs the operator to re-fixture/reorient the part
    /// (Workflow B): the post halts with a program stop before its operations run.
    Reorient,
}

/// One entry in a setup's work-datum registry. The `index` is the abstract selector
/// the post lowers to a dialect code (Okuma `G15 H<index>`, Fanuc `G54`+(index−1));
/// the physical location lives in the machine's offset table, not here.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Datum {
    /// The 1-based work-coordinate selector.
    pub index: u32,
    /// Operator-facing label (e.g. "Vise 2", "Back face"). UI only.
    #[serde(default)]
    pub label: String,
    /// How the datum is reached.
    #[serde(default)]
    pub kind: DatumKind,
}

impl Datum {
    /// The base datum every setup starts with: index 1, simultaneous, unlabelled.
    pub fn base() -> Self {
        Self {
            index: 1,
            label: String::new(),
            kind: DatumKind::Simultaneous,
        }
    }
}

/// The order in which replication (Workflow A) visits fixtures and operations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ReplicationOrder {
    /// Fixtures inside each operation: `O1→H1,H2,H3`, then `O2→H1,H2,H3`, … Operation
    /// order is preserved and replication adds **no** tool changes over a single part.
    /// The default — the usual router/production choice.
    #[default]
    ByTool,
    /// The whole operation list per fixture: `H1: O1,O2,O3`, then `H2: O1,O2,O3`, …
    /// Tool loads multiply by fixture count; matches the `PL-0-3T.MIN` house style.
    ByFixture,
}

/// The default work-datum registry: a single base datum, so a v6-or-earlier document
/// (no registry on disk) opens as a one-datum setup exactly as before.
fn default_work_offsets() -> Vec<Datum> {
    vec![Datum::base()]
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
    /// The work-datum registry: the coordinate systems this setup runs under. The
    /// machine *owns* their physical locations (the operator teaches each `G15 H<n>`
    /// / `G54…`); we carry only the index. Always has at least the base datum 1.
    #[serde(default = "default_work_offsets")]
    pub work_offsets: Vec<Datum>,
    /// Replication (Workflow A): when `Some(order)`, the whole operation list is run
    /// across every [`Simultaneous`](DatumKind::Simultaneous) datum in
    /// [`work_offsets`](Self::work_offsets), in that order — ops authored once,
    /// expanded at plan time. `None` (the default) runs each operation under its own
    /// [`work_offset`](Operation::work_offset) (a single datum, or Workflow B groups).
    #[serde(default)]
    pub replication: Option<ReplicationOrder>,
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
            spindle_rpm: 0.0,
            work_offset: 1,
            clearing: Clearing::default(),
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

    fn carve(clear_tool: Option<u32>) -> Operation {
        Operation::Carve(CarveOp {
            spindle_rpm: 0.0,
            work_offset: 1,
            id: 3,
            tool: 4,
            clear: clear_tool.map(|t| CarveClearing { tool: t, params: ClearParams::default() }),
            boundary: Contour::new(vec![
                Point::new(0.0, 0.0),
                Point::new(10.0, 0.0),
                Point::new(10.0, 10.0),
            ]),
            islands: Vec::new(),
            top: 0.0,
            depth: 1.0,
            offset: 0.0,
            ring_step: 0.0,
            scallop: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            stay_down: false,
            start: None,
        })
    }

    #[test]
    fn a_single_tool_operation_reports_exactly_one_tool() {
        assert_eq!(profile(1).tools(), vec![1]);
    }

    #[test]
    fn a_carve_reports_the_clearing_tool_first() {
        // Cutting order, not declaration order: the end mill takes the bulk out of the
        // flat land before the fine V-bit ever touches it. `tools()[0]` is what must be
        // in the spindle when the operation's fragment begins, so the order is load
        // bearing, not cosmetic.
        assert_eq!(carve(Some(7)).tools(), vec![7, 4]);
        assert_eq!(carve(None).tools(), vec![4]);
        // A clearing tool that *is* the carving tool is one tool, not two — no change
        // should be emitted between the passes.
        assert_eq!(carve(Some(4)).tools(), vec![4]);
    }

    #[test]
    fn map_tools_rewrites_the_secondary_reference_too() {
        // A renumbering that missed `clear_tool` would leave it pointing at whatever
        // tool inherited the old number — silently machining with the wrong cutter.
        let mut op = carve(Some(7));
        op.map_tools(|t| t + 10);
        assert_eq!(op.tools(), vec![17, 14]);
        // set_tool, by contrast, is deliberately narrow: the defining tool only.
        let mut op = carve(Some(7));
        op.set_tool(9);
        assert_eq!(op.tools(), vec![7, 9]);
    }

    #[test]
    fn face_direction_follows_the_picked_edge() {
        // 60×40 rectangle. Clicking a horizontal edge ⇒ pass along X; clicking a
        // vertical edge ⇒ pass along Y — regardless of which is longer.
        let rect = [
            Point::new(0.0, 0.0),
            Point::new(60.0, 0.0),
            Point::new(60.0, 40.0),
            Point::new(0.0, 40.0),
        ];
        // Pick near the bottom edge (horizontal) → X.
        assert_eq!(Axis::along_edge_at(&rect, Point::new(30.0, 0.5)), Axis::X);
        // Pick near the right edge (vertical) → Y, even though it is the short side.
        assert_eq!(Axis::along_edge_at(&rect, Point::new(59.5, 20.0)), Axis::Y);
        // Degenerate input falls back to X.
        assert_eq!(Axis::along_edge_at(&[Point::new(1.0, 1.0)], Point::new(0.0, 0.0)), Axis::X);
    }

    #[test]
    fn face_direction_longest_edge_fallback() {
        // No pick: default to the longest edge. 40×60 ⇒ long side is Y.
        let tall = [
            Point::new(0.0, 0.0),
            Point::new(40.0, 0.0),
            Point::new(40.0, 60.0),
            Point::new(0.0, 60.0),
        ];
        assert_eq!(Axis::along_longest_edge(&tall), Axis::Y);
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
            spindle_rpm: 0.0,
            work_offset: 1,
            id: 0,
            tool: 1,
            points: vec![[0.0, 0.0]],
            depth: 4.0,
            start_offset: 0.0,
            peck: None,
            dwell: None,
            feed: 100.0,
        });
        assert!(!prof.same_work(&drill));
    }
}
