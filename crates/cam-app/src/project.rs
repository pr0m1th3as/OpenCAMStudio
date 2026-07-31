//! The on-disk `.ocam` format — a self-contained JSON snapshot. A single file family
//! (`TOOLING_PLAN.md` §3.1) carries **either** a [`Project`] (a machining session) or a
//! [`ToolLibrary`], distinguished by an explicit top-level `"ocam"` tag rather than by
//! sniffing the contents. A pre-tag project (a bare [`Project`] object) still loads via
//! the legacy fallback in [`OcamFile::from_json`].
//!
//! ## Reading is a two-stage operation
//!
//! Since schema v10 a saved project is **migrated before it is deserialized**: the text
//! is parsed to a JSON tree, [`cam_model::migrate`] brings the embedded document up to
//! [`SCHEMA_VERSION`](cam_model::SCHEMA_VERSION), and only then does serde build a
//! [`Document`]. Every load goes through it, including one already at the current
//! version, so the migrating path is the ordinary path rather than a rarely-exercised
//! branch.

use cam_geo::{Polygon, Polyline};
use cam_model::{migrate, Document};

use crate::tool_library::ToolLibrary;
use crate::JobParams;

/// Why a `.ocam` file could not be read.
#[derive(Clone, Debug, PartialEq)]
pub enum LoadError {
    /// The bytes are not the JSON this format is made of.
    Json(String),
    /// The file parsed, but its document could not be brought to the current schema.
    Migration(migrate::MigrationError),
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LoadError::Json(e) => write!(f, "{e}"),
            LoadError::Migration(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for LoadError {}

impl From<serde_json::Error> for LoadError {
    fn from(e: serde_json::Error) -> Self {
        LoadError::Json(e.to_string())
    }
}

impl From<migrate::MigrationError> for LoadError {
    fn from(e: migrate::MigrationError) -> Self {
        LoadError::Migration(e)
    }
}

/// Migrate the `document` inside a serialized project tree, in place.
///
/// The version is read from the **document**, not from the project wrapper. Both carry
/// one and every file we have ever written has them equal, but the document's is the one
/// that describes the thing being migrated; taking the outer value would let a wrapper
/// edited by hand steer a rewrite of contents it does not describe.
///
/// A missing `schema_version` is read as v1. That is not a guess about unknown files: it
/// is the version whose format is "whatever has no version stamp", and v1→v9 are all
/// identity steps, so the outcome is the same as today's lenient parse.
fn migrate_project(value: &mut serde_json::Value) -> Result<(), migrate::MigrationError> {
    let Some(document) = value.get_mut("document") else {
        // No document at all: leave it be and let serde produce the error, which names
        // the missing field far better than anything phrased here could.
        return Ok(());
    };
    let from = document
        .get("schema_version")
        .and_then(serde_json::Value::as_u64)
        .unwrap_or(1) as u32;
    migrate::document(document, from)
}

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
    ///
    /// A project is migrated to the current schema on the way through; a library is not,
    /// because [`ToolLibrary`] carries no schema version and has only ever grown fields
    /// with serde defaults.
    pub fn from_json(text: &str) -> Result<Self, LoadError> {
        let mut value: serde_json::Value = serde_json::from_str(text)?;

        // The tag decides what this is. A project — tagged or, for a pre-tag file,
        // untagged — is migrated before serde sees it; anything else goes straight
        // through. Checking the tag rather than trying and retrying keeps a *migration*
        // failure from being reported as "this isn't a project", which is what a
        // try-then-fall-back arrangement would do to a genuinely too-new file.
        let tag = value.get("ocam").and_then(serde_json::Value::as_str);
        match tag {
            Some("library") => Ok(serde_json::from_value(value)?),
            Some("project") => {
                migrate_project(&mut value)?;
                Ok(serde_json::from_value(value)?)
            }
            // Untagged. A pre-tag file is a bare `Project`, which is the only untagged
            // shape ever written; migrate it and wrap it. Anything else fails in serde
            // below, as it did before.
            _ => {
                migrate_project(&mut value)?;
                Ok(OcamFile::Project(serde_json::from_value(value)?))
            }
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
    /// The imported **open** paths (engravable strokes the importer could not close).
    /// `#[serde(default)]` so projects saved before open-path import still load.
    #[serde(default)]
    pub open_paths: Vec<Polyline>,
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

    /// Parse from JSON, migrating the embedded document to the current schema first.
    pub fn from_json(text: &str) -> Result<Self, LoadError> {
        let mut value: serde_json::Value = serde_json::from_str(text)?;
        migrate_project(&mut value)?;
        Ok(serde_json::from_value(value)?)
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

    /// A v9 `.ocam` project carrying one pocket, written the way v9 wrote it: flat
    /// clearing fields on the operation, `schema_version` 9 in both the wrapper and the
    /// document.
    ///
    /// This is the file the migration exists for, so it is spelled out rather than
    /// generated. A fixture built by serializing today's `Project` would silently follow
    /// the model forward and stop being a v9 file the moment the schema moved again —
    /// which is the one thing it must never do.
    fn v9_project_json() -> String {
        serde_json::json!({
            "ocam": "project",
            "schema_version": 9,
            "document": {
                "schema_version": 9,
                "setup": {
                    "name": "bracket",
                    "heights": { "clearance": 50.0, "retract": 5.0, "top_of_stock": 0.0 },
                    "stock": { "BoundingBox": {
                        "x_offset": 0.0, "y_offset": 0.0, "top": 0.0, "thickness": 12.0
                    }},
                    "tools": [{
                        "number": 1, "diameter": 6.0, "length": 40.0,
                        "flutes": 2, "kind": "EndMill"
                    }],
                    "operations": [{ "Pocket": {
                        "id": 1,
                        "tool": 1,
                        "boundary": { "points": [[0.0,0.0],[40.0,0.0],[40.0,40.0],[0.0,40.0]] },
                        "islands": [],
                        "depth": 4.0,
                        "stepdown": 2.0,
                        "overlap": 0.45,
                        "offset": 0.3,
                        "spindle_rpm": 9500.0,
                        "work_offset": 1,
                        "feed": 750.0,
                        "plunge_feed": 250.0,
                        "plunge": "Straight",
                        "start": null,
                        "lead_overlap": 0.5,
                        "lead_in": "None",
                        "lead_out": "None",
                        "clearing": { "engagement": 1.8, "climb": true }
                    }}],
                    "extra_origins": [],
                    "origin": [0.0, 0.0, 0.0],
                    "origin_index": 1
                }
            },
            "regions": [],
            "defaults": crate::JobParams::default(),
            "source_name": "bracket.dxf"
        })
        .to_string()
    }

    #[test]
    fn a_v9_project_opens_and_keeps_its_clearing_values() {
        // The end-to-end claim the migration machinery exists to make: a project saved
        // before `ClearParams` opens, and every number the machinist entered survives to
        // the field it now lives in. A migration that parses but shifts a value is worse
        // than one that fails.
        let OcamFile::Project(p) = OcamFile::from_json(&v9_project_json()).expect("v9 opens")
        else {
            panic!("expected a project");
        };
        assert_eq!(p.document.schema_version, cam_model::SCHEMA_VERSION);
        let cam_model::Operation::Pocket(op) = &p.document.setup.operations[0] else {
            panic!("expected a pocket");
        };
        assert_eq!(op.depth, 4.0);
        assert_eq!(op.spindle_rpm, 9500.0);
        assert_eq!(op.clear.stepdown, 2.0);
        assert_eq!(op.clear.overlap, 0.45);
        assert_eq!(op.clear.offset, 0.3);
        assert_eq!(op.clear.feed, 750.0);
        assert_eq!(op.clear.plunge_feed, 250.0);
        assert_eq!(op.clear.lead_overlap, 0.5);
        assert_eq!(op.clear.clearing.engagement, 1.8);
    }

    #[test]
    fn a_v9_project_without_the_ocam_tag_is_migrated_too() {
        // The pre-tag legacy path is a *second* way into the loader, and the one most
        // likely to be forgotten: it existed before migration did. A v0.1.0 user's oldest
        // files come through here.
        let tagged: serde_json::Value = serde_json::from_str(&v9_project_json()).unwrap();
        let mut bare = tagged.as_object().unwrap().clone();
        bare.remove("ocam");
        let untagged = serde_json::Value::Object(bare).to_string();

        let OcamFile::Project(p) = OcamFile::from_json(&untagged).expect("untagged v9 opens")
        else {
            panic!("expected a project");
        };
        let cam_model::Operation::Pocket(op) = &p.document.setup.operations[0] else {
            panic!("expected a pocket");
        };
        assert_eq!(op.clear.feed, 750.0, "the legacy path skipped the migration");
    }

    #[test]
    fn a_project_from_a_newer_build_is_refused_by_version_not_by_shape() {
        // The failure has to be a *schema* refusal. If it came back as a parse error the
        // user would be told their file is damaged when it is perfectly good and their
        // application is simply too old — and the advice that follows is the opposite.
        let mut v: serde_json::Value = serde_json::from_str(&v9_project_json()).unwrap();
        v["document"]["schema_version"] =
            serde_json::json!(cam_model::SCHEMA_VERSION + 1);
        assert!(matches!(
            OcamFile::from_json(&v.to_string()),
            Err(LoadError::Migration(
                cam_model::migrate::MigrationError::FromTheFuture { .. }
            ))
        ));
    }

    #[test]
    fn an_unmigrated_v9_document_cannot_parse_as_v10() {
        // The safety net under the whole arrangement. `PocketOp::clear` carries no serde
        // default precisely so that a document reaching the deserializer un-migrated
        // fails there. Were it defaulted, this file would load as a pocket with zero feed
        // and zero stepdown — a silently broken operation, discovered at the machine.
        let v: serde_json::Value = serde_json::from_str(&v9_project_json()).unwrap();
        let err = serde_json::from_value::<Document>(v["document"].clone())
            .expect_err("a v9 document must not parse as v10");
        assert!(
            err.to_string().contains("clear"),
            "the error should name the missing field, got: {err}"
        );
    }

    #[test]
    fn a_current_project_round_trips_through_the_migrating_loader() {
        // A file at the current version takes the same path as a migrating one, so this
        // pins that the path is a no-op when there is nothing to do.
        let OcamFile::Project(once) = OcamFile::from_json(&v9_project_json()).unwrap() else {
            panic!("expected a project");
        };
        let json = OcamFile::Project(once.clone()).to_json().unwrap();
        let OcamFile::Project(twice) = OcamFile::from_json(&json).unwrap() else {
            panic!("expected a project");
        };
        assert_eq!(once, twice);
    }
}
