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

pub use cam_cldata::Point3;

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

/// The cutting-tool geometry a cycle/strategy reasons about.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ToolKind {
    /// Flat-bottomed square end mill.
    EndMill,
    /// Ball-nosed end mill.
    BallMill,
    /// Twist drill.
    Drill,
    /// Chamfer/V mill.
    ChamferMill,
    /// Face mill.
    FaceMill,
}

/// A cutting tool. The P2 slice is the minimum a post/cycle needs; feeds, speeds
/// and a richer library land with the document model at P3.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tool {
    /// Tool number (emitted as `Tn`).
    pub number: u32,
    /// Cutting diameter, mm.
    pub diameter: f64,
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
            flutes: 2,
            kind: ToolKind::EndMill,
        };
        assert_eq!(t.radius(), 3.0);
    }
}
