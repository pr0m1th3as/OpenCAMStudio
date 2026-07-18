//! The on-disk `.ocam` format — a self-contained JSON snapshot. A single file family
//! (`TOOLING_PLAN.md` §3.1) carries **either** a [`Project`] (a machining session) or a
//! [`ToolLibrary`], distinguished by an explicit top-level `"ocam"` tag rather than by
//! sniffing the contents. A pre-tag project (a bare [`Project`] object) still loads via
//! the legacy fallback in [`OcamFile::from_json`].

use cam_geo::Polygon;
use cam_model::Document;

use crate::tool_library::ToolLibrary;
use crate::JobParams;

/// The tagged union of everything a `.ocam` file can hold. Serialized with an explicit
/// discriminant (`{"ocam":"project", …}` / `{"ocam":"library", …}`), so the loader
/// never guesses.
// A short-lived (de)serialization wrapper used one variant at a time — never stored in
// bulk — so the Project/Library size gap doesn't matter; boxing would only add deref
// friction at every call site.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "ocam", rename_all = "lowercase")]
pub enum OcamFile {
    /// A tool library (Save/Load in the Tooling ribbon).
    Library(ToolLibrary),
    /// A machining project (File ▸ Open/Save).
    Project(Project),
}

impl OcamFile {
    /// Serialize to pretty JSON (tagged).
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Parse a `.ocam` file, accepting a **legacy untagged project** (a bare [`Project`]
    /// object written before the `"ocam"` tag existed) as `Project`.
    pub fn from_json(text: &str) -> Result<Self, serde_json::Error> {
        match serde_json::from_str::<OcamFile>(text) {
            Ok(f) => Ok(f),
            // Fallback: a pre-tag project has no `"ocam"` field. Try it as a bare
            // Project; surface *that* error if it also fails (it's the useful one).
            Err(tagged_err) => match Project::from_json(text) {
                Ok(p) => Ok(OcamFile::Project(p)),
                Err(_) => Err(tagged_err),
            },
        }
    }
}

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tool_library::ToolLibrary;

    #[test]
    fn library_ocam_round_trips_with_an_explicit_tag() {
        let file = OcamFile::Library(ToolLibrary::defaults());
        let json = file.to_json().unwrap();
        assert!(
            json.contains("\"ocam\": \"library\""),
            "the file self-describes with a tag:\n{json}"
        );
        assert_eq!(OcamFile::from_json(&json).unwrap(), file);
    }

    #[test]
    fn an_untagged_non_project_object_is_rejected() {
        // A bare tool-library object (no `ocam` tag) is neither a tagged OcamFile nor a
        // legacy Project, so it must fail rather than silently mis-parse.
        let untagged = serde_json::to_string(&ToolLibrary::defaults()).unwrap();
        assert!(OcamFile::from_json(&untagged).is_err());
    }
}
