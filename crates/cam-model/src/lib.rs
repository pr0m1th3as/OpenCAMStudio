//! # cam-model — the document model (P2 slice)
//!
//! Eventually this crate holds the full save-file document
//! (`Project → Setup → Stock → Operation → Tool`). The P2 slice is only what a
//! post needs to query: the [`Machine`].
//!
//! ## Machine ≠ Post
//!
//! A **[`Machine`]** is the *physical* thing — rapid rate, spindle ceiling, feed
//! limits, work envelope, tool-change position, safe height. A **post** (in
//! `cam-post`) is the *dialect* — how those get spelled as G-code. The post
//! **queries** the machine, so one grbl post can drive many grbl machines just
//! by swapping the [`Machine`] it is handed. Keeping them separate is a core
//! design rule (see `ARCHITECTURE.md`).

mod document;
mod history;

pub use cam_cldata::Point3;
pub use document::{
    Axis, ChamferOp, Clearing, Comp, Document, DrillOp, FaceOp, Hand, Heights, Lead, Operation,
    Plunge, PocketOp, ProfileOp, Setup, Side, Stock, ThreadOp, SCHEMA_VERSION,
};
pub use history::History;

/// An axis-aligned working volume, in millimetres, in the machine/WCS frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Envelope {
    /// The minimum corner (smallest X, Y, Z).
    pub min: Point3,
    /// The maximum corner (largest X, Y, Z).
    pub max: Point3,
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
#[derive(Clone, Debug, PartialEq)]
pub struct Machine {
    /// Human-readable machine name.
    pub name: String,
    /// Rapid traverse rate, mm/min (used for time estimates and as the ceiling
    /// for `G0`).
    pub rapid_rate: f64,
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

impl Machine {
    /// Whether `rpm` is within the spindle's range `(0, max_spindle_rpm]`.
    pub fn spindle_ok(&self, rpm: f64) -> bool {
        rpm > 0.0 && rpm <= self.max_spindle_rpm
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
    /// Ball-nosed end mill (corner radius = tool radius).
    BallMill,
    /// Bull-nose (corner-radius) end mill: flat bottom with a rounded corner of
    /// `corner_radius` mm (`0 < r ≤ radius`; `EndMill` and `BallMill` are the
    /// degenerate ends of this family, kept as named kinds for clarity).
    BullNose {
        /// Corner radius, mm.
        corner_radius: f64,
    },
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
    /// Thread mill — helically interpolated to cut internal/external threads.
    /// `pitch` is `None` for a single-form (pitch-agnostic) mill, or `Some(p)` for
    /// a full-profile mill whose tooth comb is ground for pitch `p` mm.
    ThreadMill {
        /// Full-profile pitch, mm; `None` for a single-form mill.
        pitch: Option<f64>,
    },
}

impl std::fmt::Display for ToolKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            ToolKind::EndMill => "End mill",
            ToolKind::BallMill => "Ball mill",
            ToolKind::BullNose { .. } => "Bull-nose mill",
            ToolKind::ChamferMill { .. } => "Chamfer mill",
            ToolKind::Drill { .. } => "Drill",
            ToolKind::FaceMill => "Face mill",
            ToolKind::ThreadMill { .. } => "Thread mill",
        };
        f.write_str(s)
    }
}

/// A cutting tool. The P2 slice is the minimum a post/cycle needs; feeds, speeds
/// and a richer library land with the document model at P3.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Tool {
    /// Tool number (emitted as `Tn`).
    pub number: u32,
    /// Cutting diameter, mm.
    pub diameter: f64,
    /// Overall tool length, mm (informational for now; the tool library and
    /// gouge-against-holder checks are the eventual consumers).
    pub length: f64,
    /// Number of flutes.
    pub flutes: u32,
    /// Tool geometry class.
    pub kind: ToolKind,
}

impl Tool {
    /// The tool radius (half the diameter), mm.
    pub fn radius(&self) -> f64 {
        0.5 * self.diameter
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn machine() -> Machine {
        Machine {
            name: "test-3018".into(),
            rapid_rate: 2000.0,
            max_spindle_rpm: 10_000.0,
            max_feed: 800.0,
            envelope: Envelope::new(Point3::new(0.0, 0.0, -50.0), Point3::new(300.0, 180.0, 0.0)),
            safe_z: 5.0,
            tool_change_pos: None,
        }
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
        };
        assert_eq!(t.radius(), 3.0);
    }

    #[test]
    fn tool_kind_variants_round_trip() {
        for kind in [
            ToolKind::EndMill,
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
}
