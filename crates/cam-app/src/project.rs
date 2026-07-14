//! The on-disk project format (`.ocam`) — a self-contained JSON snapshot of a
//! session, so a saved file reopens exactly as it was left.

use cam_geo::Polygon;
use cam_model::Document;

use crate::JobParams;

/// A saved OpenCAMStudio project: the editable document plus the imported geometry
/// it was built from and the seed defaults. Self-contained — no references to
/// external CAD files — so opening it needs nothing else on disk.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Project {
    /// Schema version (mirrors [`cam_model::SCHEMA_VERSION`]).
    pub schema_version: u32,
    /// The editable document (setup, stock, tools, operations).
    pub document: Document,
    /// The imported regions — for the viewport and for creating new operations.
    pub regions: Vec<Polygon>,
    /// Seed defaults for newly generated operations.
    pub defaults: JobParams,
    /// The source CAD file name the geometry came from (display only).
    pub source_name: String,
}

impl Project {
    /// Serialize to pretty JSON.
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Parse from JSON.
    pub fn from_json(text: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(text)
    }
}
