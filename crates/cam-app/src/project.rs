//! Self-contained `.ocam` projects use versioned ZIP containers with editable
//! JSON and losslessly compressed material-state caches. Display meshes rebuild
//! from that state at the saved resolution. Legacy tagged/untagged JSON projects
//! still load; tool libraries remain tagged JSON.
//!
//! All embedded documents pass through [`cam_model::migrate`] before typed
//! deserialization and validation, including documents in current-format archives.

use std::collections::BTreeSet;

use cam_geo::{Polygon, Polyline};
use cam_model::{migrate, Document};

use crate::tool_library::ToolLibrary;
use crate::JobParams;
#[path = "project_archive.rs"]
mod archive;

/// Why a `.ocam` file could not be read.
#[derive(Clone, Debug, PartialEq)]
pub enum LoadError {
    /// Invalid JSON, archive contents or project data.
    Json(String),
    /// The file parsed, but its document could not be brought to the current schema.
    Migration(migrate::MigrationError),
    /// A tool library written by a newer build. Refused rather than read leniently:
    /// Import Library *replaces* the working set, so misreading one costs the user
    /// their tools.
    LibraryVersion { found: u32, supported: u32 },
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LoadError::Json(e) => write!(f, "{e}"),
            LoadError::Migration(e) => write!(f, "{e}"),
            LoadError::LibraryVersion { found, supported } => write!(
                f,
                "this tool library is format {found}; this build understands \
                 {supported}. Update OpenCAMStudio to open it."
            ),
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
    /// Serialize tagged JSON. Projects use compact JSON because embedded CAD
    /// meshes can contain millions of numeric array entries; libraries stay pretty.
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        if let OcamFile::Project(project) = self {
            project.validate().map_err(serde::ser::Error::custom)?;
        }
        if let OcamFile::Library(library) = self {
            library.validate().map_err(serde::ser::Error::custom)?;
        }
        match self {
            OcamFile::Project(_) => serde_json::to_string(self),
            OcamFile::Library(_) => serde_json::to_string_pretty(self),
        }
    }

    /// Parse a `.ocam` file, accepting a **legacy untagged project** (a bare [`Project`]
    /// object written before the `"ocam"` tag existed) as `Project`.
    ///
    /// A project is migrated to the current schema on the way through. A library carries
    /// its own [`LIBRARY_VERSION`](crate::LIBRARY_VERSION) and needs no migration yet —
    /// it has only ever grown fields with serde defaults — but a library from a *newer*
    /// build is refused rather than read leniently, because importing one replaces the
    /// working tool set.
    pub fn from_json(text: &str) -> Result<Self, LoadError> {
        Self::from_value(serde_json::from_str(text)?)
    }

    fn from_value(mut value: serde_json::Value) -> Result<Self, LoadError> {
        // The tag decides what this is. A project — tagged or, for a pre-tag file,
        // untagged — is migrated before serde sees it; anything else goes straight
        // through. Checking the tag rather than trying and retrying keeps a *migration*
        // failure from being reported as "this isn't a project", which is what a
        // try-then-fall-back arrangement would do to a genuinely too-new file.
        let tag = value.get("ocam").and_then(serde_json::Value::as_str);
        match tag {
            Some("library") => {
                let mut lib: crate::ToolLibrary = serde_json::from_value(value)?;
                if lib.library_version > crate::LIBRARY_VERSION {
                    return Err(LoadError::LibraryVersion {
                        found: lib.library_version,
                        supported: crate::LIBRARY_VERSION,
                    });
                }
                lib.library_version = crate::LIBRARY_VERSION;
                lib.validate().map_err(LoadError::Json)?;
                Ok(OcamFile::Library(lib))
            }
            Some("project") => {
                migrate_project(&mut value)?;
                let project: Project = serde_json::from_value(value)?;
                project.validate().map_err(LoadError::Json)?;
                Ok(OcamFile::Project(project))
            }
            // Untagged. A pre-tag file is a bare `Project`, which is the only untagged
            // shape ever written; migrate it and wrap it. Anything else fails in serde
            // below, as it did before.
            _ => {
                migrate_project(&mut value)?;
                let project: Project = serde_json::from_value(value)?;
                project.validate().map_err(LoadError::Json)?;
                Ok(OcamFile::Project(project))
            }
        }
    }
}

/// Persisted cutting fragments and the input state they were last reviewed against.
/// Missing on legacy projects; those operations start out of date.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StoredToolpaths {
    pub cache: cam_toolpath::OperationCache,
    /// Each value combines the local input hash and the ordered dependency hash.
    pub fingerprints: std::collections::BTreeMap<u32, String>,
    /// Completed, full-setup verification results, keyed by setup identity.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub verification: std::collections::BTreeMap<u64, StoredVerification>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StoredVerification {
    pub fingerprint: String,
    pub program: cam_cldata::Program,
    pub diagnostics: Vec<cam_toolpath::Diagnostic>,
    pub collisions: Vec<cam_sim::Collision>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stock_snapshot: Option<cam_sim::StockSnapshot>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub initial_stock_snapshot: Option<cam_sim::StockSnapshot>,
    /// Archive manifests with missing/corrupt derived data must not restore a result.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub requires_stock_cache: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub initial_stock: Option<(Vec<cam_render::MeshVertex>, Vec<u32>)>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stock_vertices: Vec<cam_render::MeshVertex>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stock_indices: Vec<u32>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stock_machined: Vec<bool>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stock_operations: Vec<Option<u32>>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stock_cut_sources: Vec<Option<cam_sim::SurfaceLocation>>,
}

/// A stable project-browser node, shared with the persisted view.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum TreeNode {
    #[default]
    Models,
    Setups,
    Asset(u64),
    Drawing(usize),
    Model(u64),
    Setup(u64),
    Pattern(u64),
    Operation(u32),
}

/// Project-local browser, selection and viewport visibility at the last save.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ProjectView {
    pub active_setup: u64,
    pub active_origin: u32,
    pub selection: crate::Selection,
    pub cad_selection: Vec<cam_model::EntityRef>,
    pub expanded_setups: BTreeSet<u64>,
    pub collapsed_assets: BTreeSet<u64>,
    pub expanded_operations: BTreeSet<u32>,
    pub expanded_patterns: BTreeSet<u64>,
    pub tools_expanded: bool,
    pub hidden_assets: BTreeSet<u64>,
    pub model_view: Option<TreeNode>,
    pub browser_view: TreeNode,
    pub models_collapsed: bool,
    pub setups_collapsed: bool,
    pub hidden_setups: BTreeSet<u64>,
    pub hidden_ops: BTreeSet<u32>,
    pub cad_hidden: BTreeSet<u64>,
    pub focus_ops: BTreeSet<u32>,
    pub planar_selection: Vec<crate::LoopRef>,
    pub show_stock: bool,
    pub show_origin: bool,
    pub show_project_browser: bool,
}

/// A saved OpenCAMStudio project: the editable document plus the imported geometry
/// it was built from and the seed defaults. Self-contained — no references to
/// external CAD files — so opening it needs nothing else on disk.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Project {
    /// Absent in older projects, which retain the default opening view.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub view: Option<ProjectView>,
    #[serde(default)]
    pub toolpaths: StoredToolpaths,
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
    /// The machine this project was built for (schema v11).
    ///
    /// `None` — every project written before v11 — means the file says nothing, so
    /// opening it leaves the session's machine alone. That is the only honest reading:
    /// a file that never recorded a machine cannot be taken to prefer one.
    ///
    /// When it *is* present it is adopted, which is the point of saving it: the envelope
    /// and the feed/spindle ceilings are what gate an export, so a job must be re-checked
    /// against the machine it was written for rather than whatever was last selected.
    #[serde(default)]
    pub machine: Option<cam_model::Machine>,
    /// The post/controller dialect this project exports through (schema v11). `None`
    /// on pre-v11 files; see [`machine`](Self::machine) for why absence is not a default.
    #[serde(default)]
    pub post: Option<cam_post::PostKind>,
}

impl Project {
    /// Serialize to compact JSON, without expanding embedded CAD arrays.
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        self.validate().map_err(serde::ser::Error::custom)?;
        serde_json::to_string(self)
    }

    /// Parse from JSON, migrating the embedded document to the current schema first.
    pub fn from_json(text: &str) -> Result<Self, LoadError> {
        let mut value: serde_json::Value = serde_json::from_str(text)?;
        migrate_project(&mut value)?;
        let project: Self = serde_json::from_value(value)?;
        project.validate().map_err(LoadError::Json)?;
        Ok(project)
    }

    /// Validate the complete persisted project before it is accepted or written.
    /// Operation identities are document-wide because CAD bindings are keyed by
    /// operation ID, even though operation lists live under individual setups.
    pub fn validate(&self) -> Result<(), String> {
        if self
            .document
            .setups
            .first()
            .is_some_and(|s| s.stock == cam_model::Stock::PreviousSetup)
        {
            return Err("The first setup has no previous setup; choose box stock".into());
        }
        let mut setup_ids = BTreeSet::new();
        let mut operation_ids = BTreeSet::new();
        let mut pattern_ids = BTreeSet::new();
        for setup in &self.document.setups {
            setup.validate_patterns()?;
            if setup.id == 0 || !setup_ids.insert(setup.id) {
                return Err("setup IDs must be nonzero and unique".into());
            }
            setup
                .frame
                .validate()
                .map_err(|e| format!("setup {}: {e}", setup.name))?;
            if !setup.heights.clearance.is_finite()
                || !setup.heights.retract.is_finite()
                || !setup.heights.top_of_stock.is_finite()
                || setup.origin.iter().any(|v| !v.is_finite())
            {
                return Err(format!(
                    "setup {} contains non-finite coordinates or heights",
                    setup.name
                ));
            }
            if setup.heights.clearance < setup.heights.retract
                || setup.heights.retract < setup.heights.top_of_stock
            {
                return Err(format!(
                    "setup {} has invalid safety height ordering",
                    setup.name
                ));
            }
            for settings in setup.height_settings.values() {
                settings.validate_structure()?;
            }
            for (id, heights) in &setup.operation_heights {
                heights
                    .validate()
                    .map_err(|error| format!("Operation {id}: {error}"))?;
            }
            let mut offsets = BTreeSet::new();
            if setup.origin_index == 0 || !offsets.insert(setup.origin_index) {
                return Err(format!(
                    "setup {} has an invalid base work offset",
                    setup.name
                ));
            }
            for origin in &setup.extra_origins {
                if origin.index == 0
                    || !offsets.insert(origin.index)
                    || origin.position.iter().any(|v| !v.is_finite())
                {
                    return Err(format!(
                        "setup {} has an invalid or duplicate extra work offset",
                        setup.name
                    ));
                }
            }
            if let cam_model::Stock::FromSolid { instance } = setup.stock {
                let valid = instance
                    .and_then(|id| self.document.geometry.instance(id))
                    .and_then(|body| {
                        self.document
                            .geometry
                            .asset(body.asset_id)
                            .and_then(|asset| {
                                asset
                                    .topology
                                    .bodies
                                    .iter()
                                    .find(|b| b.id == body.body_id && b.valid_solid)
                            })
                    })
                    .is_some();
                if !valid {
                    return Err(format!(
                        "setup {} refers to an unavailable stock solid",
                        setup.name
                    ));
                }
            }
            if let cam_model::Stock::FixedBox {
                size,
                model_position,
            } = setup.stock
            {
                if size.iter().any(|v| !v.is_finite() || *v <= 0.0)
                    || model_position
                        .iter()
                        .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
                {
                    return Err(format!(
                        "setup {} has invalid fixed box stock dimensions",
                        setup.name
                    ));
                }
            }
            let mut selected_instances = BTreeSet::new();
            for instance_id in &setup.model_instances {
                if !selected_instances.insert(*instance_id) {
                    return Err(format!(
                        "setup {} lists model instance {instance_id} more than once",
                        setup.name
                    ));
                }
                if self.document.geometry.instance(*instance_id).is_none() {
                    return Err(format!(
                        "setup {} refers to missing model instance {instance_id}",
                        setup.name
                    ));
                }
            }
            for tool in &setup.tools {
                tool.validate_disc_dimensions()?;
                tool.validate_presets()?;
            }
            for operation in &setup.operations {
                operation.entry_feeds().validate()?;
                if let Some(preset) = operation.tool_preset() {
                    preset.validate()?;
                }
                if !operation_ids.insert(operation.id()) {
                    return Err(format!(
                        "operation ID {} is used by more than one setup",
                        operation.id()
                    ));
                }
            }
            for pattern in &setup.patterns {
                if !pattern_ids.insert(pattern.id) {
                    return Err(format!("pattern ID {} is used more than once", pattern.id));
                }
            }
        }
        for id in &self.document.cad_linked_operations {
            if !operation_ids.contains(id) {
                return Err(format!(
                    "CAD-linked marker refers to missing operation {id}"
                ));
            }
            if !self.document.geometry.bindings.contains_key(id) {
                return Err(format!(
                    "CAD-linked operation {id} is missing its geometry binding"
                ));
            }
        }
        self.document.geometry.validate()?;
        for (id, binding) in &self.document.geometry.bindings {
            if !operation_ids.contains(id) {
                return Err(format!("CAD binding refers to missing operation {id}"));
            }
            if !self.document.cad_linked_operations.contains(id) {
                return Err(format!(
                    "CAD binding for operation {id} is not marked as CAD-linked"
                ));
            }
            let owner = self
                .document
                .setups
                .iter()
                .find(|setup| setup.operations.iter().any(|op| op.id() == *id))
                .unwrap();
            if matches!(&binding.status, cam_model::ResolutionStatus::Resolved)
                && binding
                    .selected
                    .iter()
                    .any(|entity| !owner.model_instances.contains(&entity.instance_id))
            {
                return Err(format!(
                    "CAD binding for operation {id} refers to a model outside setup {}",
                    owner.name
                ));
            }
        }
        Ok(())
    }
}

/// Write a validated self-contained snapshot without exposing a half-written project.
pub(crate) fn atomic_save(path: &std::path::Path, project: &Project) -> Result<(), String> {
    project.validate()?;
    project.document.geometry.validate()?;
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(std::path::Path::new("."));
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|e| e.to_string())?
        .as_nanos();
    static NEXT_SAVE: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let serial = NEXT_SAVE.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let temp = parent.join(format!(
        ".ocam-save-{}-{stamp}-{serial}.tmp",
        std::process::id()
    ));
    let mut owned = false;
    let result = (|| {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temp)
            .map_err(|e| e.to_string())?;
        owned = true;
        archive::write(&mut file, project)?;
        file.sync_all().map_err(|e| e.to_string())?;
        drop(file);
        let check = std::fs::File::open(&temp).map_err(|e| e.to_string())?;
        let restored = archive::read(check, true).map_err(|e| e.to_string())?;
        if let OcamFile::Project(restored) = restored {
            restored.document.geometry.validate()?;
        }
        std::fs::rename(&temp, path).map_err(|e| e.to_string())?;
        Ok(())
    })();
    if result.is_err() && owned {
        let _ = std::fs::remove_file(&temp);
    }
    result
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
    fn large_project_arrays_save_compactly_and_keep_exact_geometry() {
        let OcamFile::Project(mut project) = OcamFile::from_json(&v9_project_json()).unwrap()
        else {
            panic!("expected project")
        };
        project.open_paths.push(Polyline::new(
            (0..10_000)
                .map(|i| cam_geo::Point::new(i as f64 * 0.123456789012345, (i as f64).sin()))
                .collect(),
        ));
        let pretty = serde_json::to_string_pretty(&project).unwrap();
        let compact = project.to_json().unwrap();
        assert!(compact.len() * 3 < pretty.len() * 2);
        assert_eq!(Project::from_json(&compact).unwrap(), project);
        let tagged = OcamFile::Project(project.clone()).to_json().unwrap();
        assert!(tagged.len() * 3 < pretty.len() * 2);
        assert_eq!(
            OcamFile::from_json(&tagged).unwrap(),
            OcamFile::Project(project)
        );
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
    pub(super) fn v9_project_json() -> String {
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
        let cam_model::Operation::Pocket(op) = &p.document.setup().operations[0] else {
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
        let cam_model::Operation::Pocket(op) = &p.document.setup().operations[0] else {
            panic!("expected a pocket");
        };
        assert_eq!(
            op.clear.feed, 750.0,
            "the legacy path skipped the migration"
        );
    }

    #[test]
    fn a_project_from_a_newer_build_is_refused_by_version_not_by_shape() {
        // The failure has to be a *schema* refusal. If it came back as a parse error the
        // user would be told their file is damaged when it is perfectly good and their
        // application is simply too old — and the advice that follows is the opposite.
        let mut v: serde_json::Value = serde_json::from_str(&v9_project_json()).unwrap();
        v["document"]["schema_version"] = serde_json::json!(cam_model::SCHEMA_VERSION + 1);
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
        let err = serde_json::from_value::<cam_model::Setup>(v["document"]["setup"].clone())
            .expect_err("a v9 setup must not parse before its operation fields migrate");
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

    #[test]
    fn project_round_trip_preserves_operation_stock_to_leave() {
        let OcamFile::Project(mut project) = OcamFile::from_json(&v9_project_json()).unwrap()
        else {
            panic!("expected a project");
        };
        let operation_id = project.document.setups[0].operations[0].id();
        let stock = cam_model::StockToLeave {
            enabled: true,
            radial: 0.35,
            axial: 0.2,
        };
        project.document.setups[0]
            .stock_to_leave
            .insert(operation_id, stock);

        let encoded = OcamFile::Project(project).to_json().unwrap();
        let OcamFile::Project(reloaded) = OcamFile::from_json(&encoded).unwrap() else {
            panic!("expected a project");
        };
        assert_eq!(
            reloaded.document.setups[0].stock_to_leave_for(operation_id),
            stock
        );
    }

    #[test]
    fn the_v14_single_setup_migrates_to_setup_one_without_changing_its_operation() {
        let OcamFile::Project(project) = OcamFile::from_json(&v9_project_json()).unwrap() else {
            panic!("expected project");
        };
        assert_eq!(project.document.setups.len(), 1);
        assert_eq!(project.document.setups[0].id, 1);
        assert_eq!(project.document.setups[0].name, "bracket");
        assert_eq!(project.document.setups[0].operations[0].id(), 1);
        assert_eq!(
            project.document.active_setup_id, 0,
            "active setup is session state"
        );
        assert_eq!(
            project.document.setup().id,
            1,
            "the first setup is the fallback"
        );
    }

    #[test]
    fn project_loader_accepts_no_setups_and_rejects_duplicate_ids_and_orphan_bindings() {
        let OcamFile::Project(project) = OcamFile::from_json(&v9_project_json()).unwrap() else {
            panic!("expected project");
        };
        let mut value: serde_json::Value =
            serde_json::from_str(&OcamFile::Project(project.clone()).to_json().unwrap()).unwrap();

        value["document"]["setups"] = serde_json::json!([]);
        let OcamFile::Project(empty) = OcamFile::from_json(&value.to_string()).unwrap() else {
            panic!("expected project");
        };
        assert!(empty.document.setups.is_empty());

        let mut value: serde_json::Value =
            serde_json::from_str(&OcamFile::Project(project.clone()).to_json().unwrap()).unwrap();
        let mut second = value["document"]["setups"][0].clone();
        second["id"] = serde_json::json!(2);
        value["document"]["setups"]
            .as_array_mut()
            .unwrap()
            .push(second);
        assert!(OcamFile::from_json(&value.to_string())
            .unwrap_err()
            .to_string()
            .contains("more than one setup"));

        let mut missing_binding = project.clone();
        missing_binding.document.cad_linked_operations.insert(1);
        assert!(missing_binding
            .validate()
            .unwrap_err()
            .contains("missing its geometry binding"));

        let mut stale_marker = project.clone();
        stale_marker.document.cad_linked_operations.insert(99);
        assert!(stale_marker
            .validate()
            .unwrap_err()
            .contains("missing operation 99"));

        let mut orphan_binding = project;
        orphan_binding.document.cad_linked_operations.insert(99);
        orphan_binding.document.geometry.bindings.insert(
            99,
            cam_model::GeometryBinding {
                selected: vec![cam_model::EntityRef {
                    asset_id: 1,
                    revision: 1,
                    instance_id: 1,
                    body_id: 1,
                    kind: cam_model::EntityKind::Body,
                    id: 1,
                }],
                recipe: cam_model::ExtractionRecipe::FaceRegion,
                heights: cam_model::CuttingHeights {
                    top: 0.0,
                    bottom: -1.0,
                },
                overrides: cam_model::GeometryOverrides::default(),
                status: cam_model::ResolutionStatus::NeedsReselection {
                    reason: "legacy".into(),
                },
                resolved_fingerprint: None,
            },
        );
        assert!(orphan_binding
            .validate()
            .unwrap_err()
            .contains("missing operation 99"));
    }

    #[test]
    fn project_loader_rejects_malformed_pattern_membership() {
        let OcamFile::Project(mut project) = OcamFile::from_json(&v9_project_json()).unwrap()
        else {
            panic!("expected a project");
        };
        project
            .document
            .setup_mut()
            .patterns
            .push(cam_model::OperationPattern {
                id: 1,
                name: "Invalid source".into(),
                members: vec![999],
                kind: cam_model::PatternKind::Rectangular {
                    nx: 2,
                    ny: 1,
                    dx: 1.0,
                    dy: 0.0,
                },
                order: cam_model::PatternOrder::ByInstance,
                suppressed: false,
                options: Default::default(),
            });
        assert!(OcamFile::Project(project.clone())
            .to_json()
            .unwrap_err()
            .to_string()
            .contains("missing operation 999"));
        let json = serde_json::to_string(&OcamFile::Project(project)).unwrap();
        assert!(OcamFile::from_json(&json)
            .unwrap_err()
            .to_string()
            .contains("missing operation 999"));
    }

    #[test]
    fn validation_checks_secondary_setup_frames_too() {
        let OcamFile::Project(mut project) = OcamFile::from_json(&v9_project_json()).unwrap()
        else {
            panic!("expected project");
        };
        let mut second = project.document.setups[0].clone();
        second.id = 2;
        second.frame.world_from_setup.rotation[0][0] = 2.0;
        project.document.setups.push(second);
        assert!(project
            .validate()
            .unwrap_err()
            .contains("right-handed orthonormal frame"));
    }
}
