//! # cam-model — saved projects, setups, operations, tools and machines
//!
//! ## Machine ≠ Post
//!
//! A **[`Machine`]** is the *physical* thing — rapid rate, spindle ceiling, feed
//! limits, work envelope, tool-change position, safe height. A **post** (in
//! `cam-post`) is the *dialect* — how those get spelled as G-code. The post
//! **queries** the machine, so one grbl post can drive many grbl machines just
//! by swapping the [`Machine`] it is handed. Dependency isolation is enforced
//! by `scripts/check_architecture.py`.

mod cutting_data;
mod rotary;
pub use rotary::{RotaryAxis, RotaryIndex, RotaryWrap};
mod cutting_forces;
pub use cutting_forces::{CuttingEngagement, CuttingForceEstimate, CuttingLoad};
mod pocket;
pub use pocket::{MultipleDepths, PocketFinishing, StockContours};
mod parallel;
pub use parallel::*;
mod spiral;
pub use spiral::*;
mod bore;
pub use bore::{BoreOp, BoreOrder};
mod drill_cycle;
pub use drill_cycle::{DrillCycleMode, DrillCycleSettings};
mod thread_standards;
pub use thread_standards::*;
mod document;
mod fixtures;
pub use fixtures::{FixtureClearance, FixtureEnvelope};
mod profile_selection;
pub use profile_selection::*;
mod parameter_expression;
pub use parameter_expression::{
    evaluate_expression, evaluate_parameter_results, evaluate_parameters,
};
mod operation_parameters;
pub use operation_parameters::OperationParameter;
mod profile_parameters;
pub use profile_parameters::*;
mod operation_heights;
pub use operation_heights::*;
mod cad_curves;
mod geometry;
pub use cad_curves::*;
mod history;
pub mod migrate;
mod reconcile;
mod validate;

pub use cam_cldata::Point3;
pub use document::{
    AdaptiveContainment, AdaptiveOp, AdaptiveParams, Axis, CarveClearing, CarveOp, ChamferFace,
    ChamferOp, CircularDistribution, ClearParams, Clearing, Comp, Document, DrillGeometry, DrillOp,
    EngraveOp, ExecutionItem, FaceOp, Hand, Heights, Lead, LinearPatternDirection, Operation,
    OperationPattern, Origin, ParallelOp, PatternDirection, PatternKind, PatternOptions,
    PatternOrder, PatternSpacing, Plunge, PocketOp, ProfileContour, ProfileOp, ProfileTabs, Setup,
    Side, SpiralOp, Stock, StockAxis, StockAxisReference, StockOriginAxis, StockToLeave, ThreadOp,
    SCHEMA_VERSION,
};
pub use geometry::*;
pub use history::History;
pub use reconcile::{reconcile_tool_numbers, ReconcileReport, ToolIdentity};
pub use validate::MachineValidationError;

/// An axis-aligned working volume, in millimetres, in the machine/WCS frame.
///
/// Saved as `{"min": [x, y, z], "max": [x, y, z]}`. The corners are [`Point3`], which
/// carries no serde derives of its own — `cam-cldata` is the one crate in the workspace
/// with no dependencies at all, the neutral waist the whole design narrows through, and
/// a save format is not reason enough to widen it. Writing the corners as three-element
/// arrays also keeps them spelled the way [`Setup::origin`] and [`Origin::position`]
/// already are, so a `.ocam` file says a 3-vector one way throughout.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(from = "EnvelopeRepr", into = "EnvelopeRepr")]
pub struct Envelope {
    /// The minimum corner (smallest X, Y, Z).
    pub min: Point3,
    /// The maximum corner (largest X, Y, Z).
    pub max: Point3,
}

/// The on-disk spelling of an [`Envelope`]. Private: the array form is a serialization
/// detail, not a second public description of a working volume.
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
struct EnvelopeRepr {
    min: [f64; 3],
    max: [f64; 3],
}

fn to_xyz(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn from_xyz([x, y, z]: [f64; 3]) -> Point3 {
    Point3::new(x, y, z)
}

impl From<EnvelopeRepr> for Envelope {
    fn from(r: EnvelopeRepr) -> Self {
        Envelope {
            min: from_xyz(r.min),
            max: from_xyz(r.max),
        }
    }
}

impl From<Envelope> for EnvelopeRepr {
    fn from(e: Envelope) -> Self {
        EnvelopeRepr {
            min: to_xyz(e.min),
            max: to_xyz(e.max),
        }
    }
}

impl Envelope {
    /// Construct an envelope from two opposite corners (as given — callers should
    /// pass a true min/max).
    pub fn new(min: Point3, max: Point3) -> Self {
        Self { min, max }
    }

    /// Whether the point `(x, y, z)` lies within the closed envelope.
    pub fn contains(&self, x: f64, y: f64, z: f64) -> bool {
        x >= self.min.x
            && x <= self.max.x
            && y >= self.min.y
            && y <= self.max.y
            && z >= self.min.z
            && z <= self.max.z
    }

    /// The machine's travel extent `(x, y, z)` — how far it can move on each axis.
    /// This, not the absolute corners, is what a toolpath must fit within: the
    /// operator's work offset (G54) can place the datum anywhere in travel, so a
    /// program in work coordinates is checked by span, not absolute position.
    pub fn extent(&self) -> (f64, f64, f64) {
        (
            self.max.x - self.min.x,
            self.max.y - self.min.y,
            self.max.z - self.min.z,
        )
    }
}

/// The physical machine a post drives. Its fields are the questions a post (or a
/// verification pass) asks: how fast may I rapid, how high may the spindle spin,
/// does this coordinate fit, where is it safe to be.
///
/// Serializable since schema v11, so a project remembers what it was built for rather
/// than adopting whatever machine the session happens to hold.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(from = "MachineRepr", into = "MachineRepr")]
pub struct Machine {
    /// Human-readable machine name.
    pub name: String,
    /// Rapid traverse rate, mm/min (used for time estimates and as the ceiling
    /// for `G0`).
    pub rapid_rate: f64,
    /// Seconds spent on each tool change, used by processing-time estimates.
    pub tool_change_seconds: f64,
    /// Minimum running spindle speed, rpm. Zero means no additional minimum.
    pub min_spindle_rpm: f64,
    /// Maximum spindle speed, rpm.
    pub max_spindle_rpm: f64,
    /// Maximum cutting feed, mm/min.
    pub max_feed: f64,
    /// The working volume.
    pub envelope: Envelope,
    /// Absolute Z considered safe for rapid traverse — nothing should rapid below
    /// it while repositioning. The machine's global clearance height.
    pub safe_z: f64,
    /// Where the machine parks for a (manual) tool change, if it has a fixed
    /// position.
    pub tool_change_pos: Option<Point3>,
}

/// The on-disk spelling of a [`Machine`] — identical but for the tool-change position,
/// written as `[x, y, z]` for the reason given on [`Envelope`].
#[derive(Clone, serde::Serialize, serde::Deserialize)]
struct MachineRepr {
    name: String,
    rapid_rate: f64,
    #[serde(default)]
    tool_change_seconds: f64,
    #[serde(default)]
    min_spindle_rpm: f64,
    max_spindle_rpm: f64,
    max_feed: f64,
    envelope: Envelope,
    safe_z: f64,
    tool_change_pos: Option<[f64; 3]>,
}

impl From<MachineRepr> for Machine {
    fn from(r: MachineRepr) -> Self {
        Machine {
            name: r.name,
            rapid_rate: r.rapid_rate,
            tool_change_seconds: r.tool_change_seconds,
            min_spindle_rpm: r.min_spindle_rpm,
            max_spindle_rpm: r.max_spindle_rpm,
            max_feed: r.max_feed,
            envelope: r.envelope,
            safe_z: r.safe_z,
            tool_change_pos: r.tool_change_pos.map(from_xyz),
        }
    }
}

impl From<Machine> for MachineRepr {
    fn from(m: Machine) -> Self {
        MachineRepr {
            name: m.name,
            rapid_rate: m.rapid_rate,
            tool_change_seconds: m.tool_change_seconds,
            min_spindle_rpm: m.min_spindle_rpm,
            max_spindle_rpm: m.max_spindle_rpm,
            max_feed: m.max_feed,
            envelope: m.envelope,
            safe_z: m.safe_z,
            tool_change_pos: m.tool_change_pos.map(to_xyz),
        }
    }
}

impl Machine {
    /// Whether `rpm` is positive and within the spindle's configured range.
    pub fn spindle_ok(&self, rpm: f64) -> bool {
        rpm > 0.0 && rpm >= self.min_spindle_rpm && rpm <= self.max_spindle_rpm
    }

    /// Whether `feed` is within `(0, max_feed]`.
    pub fn feed_ok(&self, feed: f64) -> bool {
        feed > 0.0 && feed <= self.max_feed
    }
}

/// The cutting-tool geometry a cycle/strategy reasons about. A data-carrying enum:
/// each kind holds the parameters that define its cutting profile (the ones that
/// have any — `EndMill`, `BallMill` and `FaceMill` are fully described by the
/// tool's diameter). The nominal cutting radius is always `Tool::radius()`; these
/// refine the *shape* within that envelope.
///
/// Serde uses the default external tagging, so the parameter-free variants stay
/// wire-compatible with the earlier flat enum (`"EndMill"` still round-trips).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ToolKind {
    /// Flat-bottomed square end mill.
    EndMill,
    /// Flat-bottomed roughing end mill; its serrations are approximated by the nominal cutting envelope.
    RoughingMill,
    /// Ball-nosed end mill (corner radius = tool radius).
    BallMill,
    /// Bull-nose (corner-radius) end mill: flat bottom with a rounded corner of
    /// `corner_radius` mm (`0 < r ≤ radius`; `EndMill` and `BallMill` are the
    /// degenerate ends of this family, kept as named kinds for clarity).
    BullNose {
        /// Corner radius, mm.
        corner_radius: f64,
    },
    /// Concave quarter-round cutter, with a cylindrical tip below the radius.
    RadiusMill { corner_radius: f64, tip_length: f64 },
    /// Chamfer / V mill: a point of `included_angle_deg` (full included angle),
    /// optionally with a flat tip of `tip_diameter` mm (0 for a true V).
    ChamferMill {
        /// Full included point angle, degrees.
        included_angle_deg: f64,
        /// Flat-tip diameter, mm (0 for a sharp V).
        tip_diameter: f64,
    },
    /// Twist drill with a point of `point_angle_deg` (full included angle).
    Drill {
        /// Full included point angle, degrees (commonly 118 or 135).
        point_angle_deg: f64,
    },
    /// Face mill.
    FaceMill,
    /// Square-edged groove cutter (keyseat cutter) on a shank or reduced neck.
    GrooveCutter,
    /// Slitting saw blade mounted on an arbor.
    SlittingSaw,
    /// Thread mill — helically interpolated to cut internal/external threads.
    /// `pitch` is `None` for a single-form (pitch-agnostic) mill, or `Some(p)` for
    /// a full-profile mill whose tooth comb is ground for pitch `p` mm.
    ThreadMill {
        /// Full-profile pitch, mm; `None` for a single-form mill.
        pitch: Option<f64>,
    },
    /// Carving **V-bit**: a cone rising at `included_angle_deg` (full V angle) from a
    /// rounded tip of `tip_radius` mm (0 = a sharp point) up to the shaft diameter. The
    /// cone is the cutting portion; there is no separate flute length.
    VBit {
        /// Full included V angle, degrees.
        included_angle_deg: f64,
        /// Rounded-tip radius, mm (0 for a sharp point).
        tip_radius: f64,
    },
}

impl std::fmt::Display for ToolKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            ToolKind::EndMill => "Square End Mill",
            ToolKind::RoughingMill => "Roughing End Mill",
            ToolKind::BallMill => "Ball Nose End Mill",
            ToolKind::BullNose { .. } => "Rounded-Edge End Mill",
            ToolKind::RadiusMill { .. } => "Radius mill",
            ToolKind::ChamferMill { .. } => "Chamfer mill",
            ToolKind::Drill { .. } => "Drill bit",
            ToolKind::FaceMill => "Face mill",
            ToolKind::GrooveCutter => "Groove cutter",
            ToolKind::SlittingSaw => "Slitting saw",
            ToolKind::ThreadMill { .. } => "Thread mill",
            ToolKind::VBit { .. } => "V-bit",
        };
        f.write_str(s)
    }
}

impl ToolKind {
    /// A radial cutter with an axial cutting width set by the head or blade.
    pub fn is_side_cutting_disc(self) -> bool {
        matches!(self, Self::GrooveCutter | Self::SlittingSaw)
    }
}

/// The cutting/helix direction of a milling tool — a **down-cut** (flutes push chips
/// down, a clean top edge) or **up-cut** (chips lifted, clears deep cuts). Like the
/// flute count, it's a physical property of the tool: a down-cut and an up-cut ⌀6 end
/// mill are *different* tools, so it counts toward [`ToolIdentity`](crate::ToolIdentity).
/// (A later consumer is the adaptive-clearing strategy; wiring is deferred.)
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, Hash, Default, serde::Serialize, serde::Deserialize,
)]
pub enum CutDir {
    /// Down-cut (down-milling helix).
    #[default]
    Down,
    /// Up-cut (up-milling helix).
    Up,
    /// Straight (axial) flutes — no helix. Only meaningful for a square end mill.
    Straight,
    /// Up-cut near the tip and down-cut above it, directing chips inward.
    Compression,
}

impl std::fmt::Display for CutDir {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            CutDir::Down => "Down-cut",
            CutDir::Up => "Up-cut",
            CutDir::Straight => "Straight flute",
            CutDir::Compression => "Compression",
        })
    }
}

/// A cutting tool. The P2 slice was `number/diameter/length/flutes/kind`; the tool
/// subsystem (see `TOOLING_PLAN.md`) adds the **non-cutting** geometry the sim's
/// gouge checks and the tool-library preview reason about — all `#[serde(default)]`
/// so pre-existing tool libraries and `.ocam` projects load unchanged.
///
/// The enriched dimensions use `0.0` as an **"unspecified" sentinel** resolved by the
/// [`Tool::flute_len`], [`Tool::shank_dia`], [`Tool::neck_dia`] accessors, so an old
/// tool (all sentinels) behaves exactly as before: fully fluted, no distinct shank.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Tool {
    /// Optional catalog details; empty for tools saved before these fields existed.
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub manufacturer: String,
    #[serde(default)]
    pub product_id: String,
    #[serde(default)]
    pub product_link: String,
    /// Cutter substrate, such as carbide or high-speed steel.
    #[serde(default)]
    pub cutter_material: String,
    /// Cutter coating, such as TiAlN; empty when unspecified.
    #[serde(default)]
    pub coating: String,
    /// Tool number (emitted as `Tn`).
    pub number: u32,
    /// Cutting diameter, mm.
    pub diameter: f64,
    /// Overall tool length — the stickout below the holder, mm.
    pub length: f64,
    /// Axial length of cut from the tip, mm (head width or blade thickness for a disc).
    /// `0.0` = unspecified for older tool kinds;
    /// see [`Tool::flute_len`] for the fallback. Drives the
    /// cutting/non-cutting split of a generated generatrix.
    #[serde(default)]
    pub flute_length: f64,
    /// Face mill body height; zero means the legacy flute length.
    #[serde(default)]
    pub face_body_height: f64,
    /// Shank diameter, mm. `0.0` = unspecified → equals the cutting `diameter` (no
    /// distinct shank); see [`Tool::shank_dia`].
    #[serde(default)]
    pub shank_diameter: f64,
    /// Reduced-neck length from the cutting end, mm. `0.0` = no reduced neck.
    #[serde(default)]
    pub neck_length: f64,
    /// Reduced-neck diameter, mm. `0.0` = unspecified → equals the cutting `diameter`;
    /// see [`Tool::neck_dia`].
    #[serde(default)]
    pub neck_diameter: f64,
    /// Number of flutes.
    pub flutes: u32,
    /// Cutting/helix direction (down-cut / up-cut) — a physical tool property, in
    /// identity. `#[serde(default)]` (→ `Down`) so older tools load unchanged.
    #[serde(default)]
    pub cutting_direction: CutDir,
    /// Nominal cutting data — the tool's default spindle speed and feeds, used to
    /// **seed** a new operation's own values (which stay overridable per operation).
    /// `0.0` = unspecified (older tools load this way via `#[serde(default)]`), in
    /// which case the job/operation falls back to its existing default rather than
    /// commanding a zero. A later physics pass (material + surface speed) will compute
    /// these; for now they are hand-entered library defaults.
    #[serde(default)]
    pub nominal_rpm: f64,
    /// Nominal cutting feed, mm/min. See [`nominal_rpm`](Self::nominal_rpm).
    #[serde(default)]
    pub nominal_feed: f64,
    /// Nominal plunge feed, mm/min. See [`nominal_rpm`](Self::nominal_rpm).
    #[serde(default)]
    pub nominal_plunge_feed: f64,
    #[serde(default)]
    pub cooling: Cooling,
    /// Named cutting data snapshots, independent of physical cutter identity.
    #[serde(default)]
    pub presets: Vec<ToolPreset>,
    /// High-water mark: deleted preset IDs are never reused.
    #[serde(default)]
    pub preset_id_floor: u64,
    /// Tool geometry class.
    pub kind: ToolKind,
}

impl Default for Tool {
    /// A bare ⌀0 end mill — a base for struct-update literals (`..Default::default()`).
    /// Not a usable tool on its own; callers set at least number/diameter/kind.
    fn default() -> Self {
        Self {
            description: String::new(),
            manufacturer: String::new(),
            product_id: String::new(),
            product_link: String::new(),
            cutter_material: String::new(),
            coating: String::new(),
            number: 0,
            diameter: 0.0,
            length: 0.0,
            flute_length: 0.0,
            face_body_height: 0.0,
            shank_diameter: 0.0,
            neck_length: 0.0,
            neck_diameter: 0.0,
            flutes: 0,
            cutting_direction: CutDir::Down,
            nominal_rpm: 0.0,
            nominal_feed: 0.0,
            nominal_plunge_feed: 0.0,
            cooling: Cooling::None,
            presets: Vec::new(),
            preset_id_floor: 0,
            kind: ToolKind::EndMill,
        }
    }
}

impl Tool {
    /// The tool radius (half the diameter), mm.
    pub fn radius(&self) -> f64 {
        0.5 * self.diameter
    }

    /// Effective length of cut: the explicit [`flute_length`](Self::flute_length), or
    /// the overall length when unspecified. For a chamfer mill the unspecified
    /// length is the cone height, preserving its original cutting profile.
    pub fn flute_len(&self) -> f64 {
        if self.flute_length > 0.0 {
            self.flute_length
        } else if let ToolKind::ChamferMill {
            included_angle_deg,
            tip_diameter,
        } = self.kind
        {
            let tangent = (included_angle_deg * 0.5).to_radians().tan();
            if tangent > 0.0 {
                ((self.diameter - tip_diameter).max(0.0) * 0.5 / tangent).min(self.length)
            } else {
                self.length
            }
        } else {
            self.length
        }
    }

    /// Effective shank diameter: the explicit [`shank_diameter`](Self::shank_diameter),
    /// or the cutting [`diameter`](Self::diameter) when unspecified.
    pub fn shank_dia(&self) -> f64 {
        if self.shank_diameter > 0.0 {
            self.shank_diameter
        } else {
            self.diameter
        }
    }

    /// Face mill body height, with a fallback for older saved tools.
    pub fn face_body_height(&self) -> f64 {
        if self.face_body_height > 0.0 { self.face_body_height } else { self.flute_len() }
    }

    /// Effective neck diameter: the explicit [`neck_diameter`](Self::neck_diameter),
    /// or the cutting [`diameter`](Self::diameter) when unspecified.
    pub fn neck_dia(&self) -> f64 {
        if self.neck_diameter > 0.0 {
            self.neck_diameter
        } else {
            self.diameter
        }
    }

    /// Check the explicit dimensions needed to define a side-cutting disc.
    pub fn validate_disc_dimensions(&self) -> Result<(), String> {
        if !self.kind.is_side_cutting_disc() {
            return Ok(());
        }
        let positive = |v: f64| v.is_finite() && v > 0.0;
        if !positive(self.diameter) || !positive(self.flute_length) || self.flutes == 0 {
            return Err(format!(
                "Tool {} ({}) needs a positive cutting diameter, width and tooth count",
                self.number, self.kind
            ));
        }
        if !self.length.is_finite() || self.length <= self.flute_length {
            return Err(format!(
                "Tool {} must be longer than its cutting width",
                self.number
            ));
        }
        if !positive(self.shank_diameter) {
            return Err(format!(
                "Tool {} needs a positive shank or arbor diameter",
                self.number
            ));
        }
        if !self.neck_length.is_finite()
            || self.neck_length < 0.0
            || self.neck_length >= self.length - self.flute_length
            || (self.kind == ToolKind::SlittingSaw && self.neck_length != 0.0)
        {
            return Err(format!("Tool {} has an invalid neck length", self.number));
        }
        let support = if self.neck_length > 0.0 {
            self.neck_diameter
        } else {
            self.shank_diameter
        };
        if !positive(support) || support >= self.diameter {
            return Err(format!(
                "Tool {} needs a neck, shank or arbor at the disc narrower than its cutting diameter",
                self.number
            ));
        }
        Ok(())
    }

    /// This tool's **2D revolve generatrix** (`TOOLING_PLAN.md` Phase 4): the built-in
    /// kinds mapped onto a kernel-neutral [`cam_geo::GeneratrixSpec`] and generated. The
    /// cutting-end shape comes from [`kind`](Self::kind); the flute/shank split from the
    /// effective cutter dimensions. Thread mills include their single tooth or
    /// repeated full-form teeth, with the non-cutting tip, neck and shank tagged.
    pub fn profile(&self) -> cam_geo::Profile2D {
        use cam_geo::BottomShape;
        if matches!(self.kind, ToolKind::FaceMill) {
            use cam_geo::{Point, Profile2D, ProfileSeg, SegShape};
            let r = self.radius().max(0.0);
            let body = self.face_body_height().max(0.0);
            let flute = self.flute_len().clamp(0.0, body);
            let length = self.length.max(body);
            let shank = self.shank_dia().max(0.0) * 0.5;
            let line = |r, z, cutting| ProfileSeg { shape: SegShape::Line, end: Point::new(r, z), cutting };
            let mut segs = vec![line(r, 0.0, true), line(r, flute, true)];
            if body > flute { segs.push(line(r, body, false)); }
            if (shank - r).abs() > 1e-9 { segs.push(line(shank, body, false)); }
            if length > body { segs.push(line(shank, length, false)); }
            if shank > 1e-9 { segs.push(line(0.0, length, false)); }
            return Profile2D { start: Point::new(0.0, 0.0), segs };
        }
        // Single-profile (single-point) thread mill: one 60° tooth at the tip, a sharply
        // reduced neck over the length of cut (its reach), then the shank. The neck ⌀ sets
        // the max thread depth = (min cutting ⌀ − neck ⌀)/2. Built directly, since the
        // generatrix has no single-tooth bottom.
        if let ToolKind::ThreadMill { pitch: None } = self.kind {
            use cam_geo::{Point, Profile2D, ProfileSeg, SegShape};
            let r_min = self.radius(); // min cutting ⌀ = tooth crest
            let r_neck = (self.neck_dia() * 0.5).min(r_min);
            let r_shank = self.shank_dia() * 0.5;
            let l_cut = self.flute_len().max(1e-3); // length of cut (reach)
            let oal = self.length.max(l_cut);
            // Symmetric 60° tooth (crest r_min, roots r_neck): height 2·depth·tan30°.
            let h = (2.0 * (r_min - r_neck) * (30.0_f64).to_radians().tan()).min(l_cut);
            let line = |r: f64, z: f64, cutting| ProfileSeg {
                shape: SegShape::Line,
                end: Point::new(r, z),
                cutting,
            };
            let mut segs = vec![
                line(r_neck, 0.0, false),   // tip flat at the neck radius
                line(r_min, h * 0.5, true), // rising flank to the crest (cutting)
                line(r_neck, h, true),      // falling flank back to the neck (cutting)
                line(r_neck, l_cut, false), // reduced neck up to the length of cut
            ];
            if (r_shank - r_neck).abs() > 1e-9 {
                segs.push(line(r_shank, l_cut, false)); // step to the shank
            }
            segs.push(line(r_shank, oal, false)); // shank to the overall length
            if r_shank > 1e-9 {
                segs.push(line(0.0, oal, false)); // top face back to the axis
            }
            return Profile2D {
                start: Point::new(0.0, 0.0),
                segs,
            };
        }
        // Full-form thread mill: the threads *are* the cutting surface, so the boundary is
        // a 60° saw-tooth up the side (crest = cutting ⌀, root = crest − depth, depth =
        // 0.866·pitch for a 60° form). The bottom face and shank are non-cutting.
        if let ToolKind::ThreadMill { pitch: Some(pitch) } = self.kind {
            use cam_geo::{Point, Profile2D, ProfileSeg, SegShape};
            let r = self.radius(); // crest / cutting ⌀
            let r_shank = self.shank_dia() * 0.5;
            let tl = self.flute_len().max(1e-3); // thread length
            let oal = self.length.max(tl);
            let p = pitch.max(0.05);
            let depth = (0.866_025_4 * p).min(r * 0.7);
            let r_root = (r - depth).max(0.0);
            let count = ((tl / p).floor() as usize).max(1);
            let line = |r: f64, z: f64, cutting| ProfileSeg {
                shape: SegShape::Line,
                end: Point::new(r, z),
                cutting,
            };
            // Bottom face (non-cutting) out to the minor ⌀, then the toothed side rising
            // root → crest → root at each half-pitch (cutting).
            let mut segs = vec![line(r_root, 0.0, false)];
            for k in 1..=2 * count {
                let z = (k as f64) * (p * 0.5);
                let rr = if k % 2 == 1 { r } else { r_root };
                segs.push(line(rr, z, true));
            }
            let z_top = (count as f64) * p;
            segs.push(line(r_shank, z_top, false)); // step to the shank
            segs.push(line(r_shank, oal, false)); // shank to the overall length
            if r_shank > 1e-9 {
                segs.push(line(0.0, oal, false)); // top face back to the axis
            }
            return Profile2D {
                start: Point::new(0.0, 0.0),
                segs,
            };
        }
        // A V-bit's cutting is the whole cone (no separate flute length); passing
        // flute_length 0 clamps the flute top to the cone top, so the cone is cutting
        // and the shaft above is non-cutting.
        let mut flute_length = self.flute_len();
        // Shaft radius above the cutting end. A V-bit is defined by a single diameter —
        // the shaft it flares up to — so its cone always meets the shank exactly (the
        // "flute diameter" of a V-bit is undefined: it varies along the cone). This also
        // makes a stale `shank_diameter` incapable of producing an arrow.
        let mut shank_radius = self.shank_dia() * 0.5;
        let bottom = match self.kind {
            ToolKind::EndMill
            | ToolKind::RoughingMill
            | ToolKind::FaceMill
            | ToolKind::ThreadMill { .. } => BottomShape::Flat,
            ToolKind::GrooveCutter | ToolKind::SlittingSaw => BottomShape::SideCuttingDisc,
            ToolKind::BallMill => BottomShape::Ball,
            ToolKind::BullNose { corner_radius } => BottomShape::BullNose { corner_radius },
            ToolKind::RadiusMill {
                corner_radius,
                tip_length,
            } => BottomShape::RadiusMill {
                corner_radius,
                tip_length,
            },
            ToolKind::ChamferMill {
                included_angle_deg,
                tip_diameter,
            } => {
                // Like a V-bit but with a flat, **non-cutting** tip instead of a rounded
                // one: the cone flank is the only cutting surface, flaring exactly to the
                // shaft. The flute length controls where the cutting edge ends.
                shank_radius = self.radius();
                BottomShape::Cone {
                    half_angle_rad: (included_angle_deg * 0.5).to_radians(),
                    flat_radius: tip_diameter * 0.5,
                }
            }
            ToolKind::Drill { point_angle_deg } => BottomShape::Cone {
                half_angle_rad: (point_angle_deg * 0.5).to_radians(),
                flat_radius: 0.0,
            },
            ToolKind::VBit {
                included_angle_deg,
                tip_radius,
            } => {
                flute_length = 0.0;
                shank_radius = self.radius(); // cone flares exactly to the shaft
                BottomShape::VTip {
                    half_angle_rad: (included_angle_deg * 0.5).to_radians(),
                    tip_radius,
                }
            }
        };
        cam_geo::generatrix(&cam_geo::GeneratrixSpec {
            radius: self.radius(),
            flute_length,
            shank_radius,
            length: self.length,
            neck_length: self.neck_length,
            neck_radius: self.neck_dia() * 0.5,
            bottom,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn machine() -> Machine {
        Machine {
            name: "test-3018".into(),
            rapid_rate: 2000.0,
            tool_change_seconds: 0.0,
            min_spindle_rpm: 0.0,
            max_spindle_rpm: 10_000.0,
            max_feed: 800.0,
            envelope: Envelope::new(Point3::new(0.0, 0.0, -50.0), Point3::new(300.0, 180.0, 0.0)),
            safe_z: 5.0,
            tool_change_pos: None,
        }
    }

    #[test]
    fn machine_without_tool_change_time_loads_with_zero_and_round_trips() {
        let json = r#"{"name":"legacy","rapid_rate":2000.0,"max_spindle_rpm":10000.0,"max_feed":800.0,"envelope":{"min":[0.0,0.0,-50.0],"max":[300.0,180.0,0.0]},"safe_z":5.0,"tool_change_pos":null}"#;
        let machine: Machine = serde_json::from_str(json).unwrap();
        assert_eq!(machine.tool_change_seconds, 0.0);
        assert_eq!(machine.min_spindle_rpm, 0.0);
        let mut configured = machine;
        configured.tool_change_seconds = 15.0;
        configured.min_spindle_rpm = 1000.0;
        let restored: Machine =
            serde_json::from_str(&serde_json::to_string(&configured).unwrap()).unwrap();
        assert_eq!(restored, configured);
    }

    #[test]
    fn envelope_containment() {
        let m = machine();
        assert!(m.envelope.contains(10.0, 10.0, -5.0));
        assert!(!m.envelope.contains(-1.0, 10.0, -5.0));
        assert!(!m.envelope.contains(10.0, 10.0, 1.0));
    }

    #[test]
    fn spindle_and_feed_limits() {
        let m = machine();
        assert!(m.spindle_ok(8000.0));
        assert!(!m.spindle_ok(0.0));
        assert!(!m.spindle_ok(12_000.0));
        assert!(m.feed_ok(300.0));
        assert!(!m.feed_ok(1000.0));
    }

    #[test]
    fn tool_radius() {
        let t = Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        };
        assert_eq!(t.radius(), 3.0);
    }

    #[test]
    fn tool_kind_variants_round_trip() {
        for kind in [
            ToolKind::EndMill,
            ToolKind::RoughingMill,
            ToolKind::BallMill,
            ToolKind::BullNose { corner_radius: 1.5 },
            ToolKind::ChamferMill {
                included_angle_deg: 90.0,
                tip_diameter: 0.5,
            },
            ToolKind::Drill {
                point_angle_deg: 118.0,
            },
            ToolKind::FaceMill,
            ToolKind::ThreadMill { pitch: Some(1.25) },
            ToolKind::ThreadMill { pitch: None },
        ] {
            let json = serde_json::to_string(&kind).unwrap();
            let back: ToolKind = serde_json::from_str(&json).unwrap();
            assert_eq!(kind, back, "round-trip failed for {kind:?} via {json}");
        }
    }

    #[test]
    fn parameter_free_kinds_are_wire_compatible_with_v1() {
        // The pre-v2 flat enum serialized unit variants as bare strings; those
        // must still deserialize so existing tool libraries load unchanged.
        assert_eq!(
            serde_json::from_str::<ToolKind>("\"EndMill\"").unwrap(),
            ToolKind::EndMill
        );
        assert_eq!(
            serde_json::from_str::<ToolKind>("\"FaceMill\"").unwrap(),
            ToolKind::FaceMill
        );
    }

    #[test]
    fn enriched_dims_resolve_their_unspecified_sentinels() {
        // All sentinels (an "old" tool): fully fluted, shank == cutting diameter.
        let bare = Tool {
            number: 1,
            diameter: 6.0,
            length: 40.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        };
        assert_eq!(
            bare.flute_len(),
            40.0,
            "unspecified flute length ⇒ overall length"
        );
        assert_eq!(
            bare.shank_dia(),
            6.0,
            "unspecified shank ⇒ cutting diameter"
        );
        assert_eq!(bare.neck_dia(), 6.0, "unspecified neck ⇒ cutting diameter");

        // Explicit values win.
        let stub = Tool {
            flute_length: 12.0,
            shank_diameter: 8.0,
            neck_length: 20.0,
            neck_diameter: 4.0,
            ..bare
        };
        assert_eq!(stub.flute_len(), 12.0);
        assert_eq!(stub.shank_dia(), 8.0);
        assert_eq!(stub.neck_dia(), 4.0);
    }

    #[test]
    fn tool_json_round_trips_with_enriched_dims() {
        let t = Tool {
            number: 3,
            diameter: 10.0,
            length: 60.0,
            flute_length: 30.0,
            shank_diameter: 10.0,
            neck_length: 15.0,
            neck_diameter: 6.0,
            flutes: 3,
            cutting_direction: CutDir::Up,
            nominal_rpm: 8000.0,
            nominal_feed: 900.0,
            nominal_plunge_feed: 300.0,
            presets: Vec::new(),
            preset_id_floor: 0,
            kind: ToolKind::BullNose { corner_radius: 1.0 },
            cooling: Cooling::Mms,
            ..Default::default()
        };
        let back: Tool = serde_json::from_str(&serde_json::to_string(&t).unwrap()).unwrap();
        assert_eq!(t, back);
    }

    #[test]
    fn compression_direction_round_trips_and_has_distinct_identity() {
        let down = Tool { kind: ToolKind::EndMill, ..Default::default() };
        let compression = Tool { cutting_direction: CutDir::Compression, ..down.clone() };
        let loaded: Tool = serde_json::from_str(&serde_json::to_string(&compression).unwrap()).unwrap();
        assert_eq!(loaded.cutting_direction, CutDir::Compression);
        assert_ne!(compression.identity(), down.identity());
    }

    #[test]
    fn radius_mill_profile_and_identity_round_trip() {
        use cam_geo::{Point, SegShape};
        let tool = Tool {
            diameter: 12.0,
            length: 72.0,
            flute_length: 12.0,
            shank_diameter: 24.0,
            kind: ToolKind::RadiusMill {
                corner_radius: 6.0,
                tip_length: 2.0,
            },
            ..Default::default()
        };
        let profile = tool.profile();
        assert_eq!(profile.segs[0].end, Point::new(6.0, 0.0));
        assert_eq!(profile.segs[1].end, Point::new(6.0, 2.0));
        assert_eq!(profile.segs[2].end, Point::new(12.0, 8.0));
        assert!(
            matches!(profile.segs[2].shape, SegShape::Arc { center, ccw: false } if center == Point::new(12.0, 2.0))
        );
        assert_eq!(profile.max_radius(), 12.0);
        let loaded: Tool = serde_json::from_str(&serde_json::to_string(&tool).unwrap()).unwrap();
        assert_eq!(tool, loaded);
        assert_eq!(tool.identity(), loaded.identity());
        let mut different = tool.clone();
        different.kind = ToolKind::RadiusMill {
            corner_radius: 6.0,
            tip_length: 1.0,
        };
        assert_ne!(tool.identity(), different.identity());
    }

    #[test]
    fn profile_maps_each_kind_to_its_cutting_end() {
        use cam_geo::SegShape;
        let mk = |kind| Tool {
            number: 1,
            diameter: 8.0,
            length: 40.0,
            flute_length: 20.0,
            flutes: 2,
            kind,
            ..Default::default()
        };
        // End mill: flat bottom (first segment a line to (r, 0)).
        let em = mk(ToolKind::EndMill).profile();
        assert_eq!(em.segs[0].shape, SegShape::Line);
        assert!(em.segs[0].cutting);
        assert_eq!(em.max_radius(), 4.0);
        assert_eq!(em.height(), 40.0);
        // Roughing serrations use the same nominal envelope as a flat end mill.
        assert_eq!(mk(ToolKind::RoughingMill).profile(), em);
        // Ball: first segment is a cutting arc.
        let ball = mk(ToolKind::BallMill).profile();
        assert!(matches!(ball.segs[0].shape, SegShape::Arc { .. }));
        // Chamfer with a flat tip: the flat tip line is **non-cutting**, only the cone
        // flank cuts (a chamfer mill cuts on the angled flank alone).
        let cham = mk(ToolKind::ChamferMill {
            included_angle_deg: 90.0,
            tip_diameter: 1.0,
        })
        .profile();
        assert!(!cham.segs[0].cutting, "flat tip is non-cutting");
        assert!(cham.segs[1].cutting, "cone flank cuts");
        // The cone flares exactly to the shaft (⌀8 → r4); no arrow, and the shaft above
        // the flank is non-cutting.
        assert_eq!(cham.max_radius(), 4.0);
        assert!(cham
            .segs
            .iter()
            .filter(|s| s.cutting)
            .all(|s| s.end.x <= 4.0 + 1e-9));
        let mut short = mk(ToolKind::ChamferMill {
            included_angle_deg: 90.0,
            tip_diameter: 0.0,
        });
        short.flute_length = 2.0;
        let profile = short.profile();
        assert!(profile.segs.iter().any(|s| s.cutting && (s.end.y - 2.0).abs() < 1e-9));
        assert!(profile.segs.iter().filter(|s| s.cutting).all(|s| s.end.y <= 2.0 + 1e-9));
        assert!(profile.segs.iter().any(|s| !s.cutting && (s.end.y - 4.0).abs() < 1e-9));
        short.flute_length = 0.0;
        assert!((short.flute_len() - 4.0).abs() < 1e-9);
        // Flute split honoured: nothing above z=20 cuts.
        assert!(em
            .segs
            .iter()
            .filter(|s| s.cutting)
            .all(|s| s.end.y <= 20.0 + 1e-9));
    }

    #[test]
    fn face_mill_body_and_cutting_length_are_distinct() {
        let mut tool = Tool { diameter: 50.0, length: 60.0, flute_length: 25.0, shank_diameter: 16.0, kind: ToolKind::FaceMill, ..Default::default() };
        assert_eq!(tool.face_body_height(), 25.0);
        tool.face_body_height = 25.0;
        tool.flute_length = 5.0;
        let profile = tool.profile();
        assert!(profile.segs.iter().any(|s| s.cutting && s.end.y == 5.0));
        assert!(profile.segs.iter().any(|s| !s.cutting && s.end.x == 25.0 && s.end.y == 25.0));
        let json = serde_json::to_string(&tool).unwrap();
        assert_eq!(serde_json::from_str::<Tool>(&json).unwrap(), tool);
    }

    #[test]
    fn pre_tooling_tool_json_loads_with_default_sentinels() {
        // A tool saved before the enriched dims existed (only the P2 fields) must
        // still deserialize — the new fields are `#[serde(default)]` → 0.0 sentinels.
        let legacy = r#"{"number":2,"diameter":6.0,"length":30.0,"flutes":2,"kind":"EndMill"}"#;
        let t: Tool = serde_json::from_str(legacy).unwrap();
        assert_eq!(t.diameter, 6.0);
        assert_eq!(t.flute_length, 0.0);
        assert_eq!(t.flute_len(), 30.0, "legacy tool reads as fully fluted");
        assert_eq!(t.shank_dia(), 6.0);
    }
}

mod tool_presets;
pub use tool_presets::{Cooling, EntryFeeds, Material, ToolPreset};
