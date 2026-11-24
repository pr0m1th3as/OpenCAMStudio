//! The headless application controller.
//!
//! All of the app's behaviour lives here, with no GUI dependency: open a DXF,
//! hold an editable [`Document`] with a [`Selection`], adjust the selected node
//! (undoably), run the strategies, build the viewport scene, simulate the stock,
//! and export G-code. The iced shell is a thin view over this — so the app's
//! logic is unit-tested exactly like the rest of the pipeline.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use cam_geo::{Contour, Point, Polygon, Polyline};
use cam_import::{read_dxf_str, ImportError, ImportOptions};

use crate::project::{OcamFile, Project};
use cam_model::{
    reconcile_tool_numbers, AdaptiveOp, AdaptiveParams, Axis, CarveOp, ChamferOp, ClearParams,
    Comp, Document, DrillOp, EngraveOp, FaceOp, Hand, Heights, History, Lead, Machine, Operation,
    Origin, ParallelOp, ParallelParams, Plunge, PocketOp, ProfileOp, ReconcileReport, Setup, Side,
    Stock, ThreadOp, Tool, ToolKind,
};
use cam_post::{PostError, PostKind, PostOptions};
use cam_render::{MeshVertex, Scene};
use cam_sim::{Collision, CollisionKind, ProfileShape, ToolProfile};

use cam_cldata::Program;
use cam_toolpath::{CancelToken, Diagnostic, JobCompletion, Severity};

#[path = "cad_creation.rs"]
mod cad_creation;
#[path = "drawing_drafts.rs"]
mod drawing_drafts;

#[path = "commands.rs"]
mod commands;

#[path = "selection.rs"]
mod selection;

#[path = "export.rs"]
mod export;

#[path = "job_controller.rs"]
mod job_controller;

#[path = "setups.rs"]
mod setups;

/// Seed defaults for a freshly-imported document: the values every generated
/// operation and the setup's heights start from. Once a document exists, editing
/// happens on the document, not here.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct JobParams {
    pub tool_diameter: f64,
    pub depth: f64,
    pub stepdown: f64,
    /// Radial/lateral stepover for area-clearing ops (pocket/face), mm.
    pub stepover: f64,
    pub feed: f64,
    pub plunge_feed: f64,
    pub spindle_rpm: f64,
    pub clearance: f64,
    pub retract: f64,
    pub top_of_stock: f64,
}

impl Default for JobParams {
    fn default() -> Self {
        Self {
            tool_diameter: 6.0,
            depth: 4.0,
            stepdown: 2.0,
            stepover: 3.0,
            feed: 300.0,
            plunge_feed: 100.0,
            spindle_rpm: 1000.0,
            clearance: 5.0,
            retract: 2.0,
            top_of_stock: 0.0,
        }
    }
}

/// Which kind of operation [`AppController::new_operation`] should create from the
/// loaded geometry. The strategies already exist; this picks the default variant.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum OpKind {
    Profile,
    Pocket,
    Drill,
    Face,
    Chamfer,
    Thread,
    Bore,
    Engrave,
    Carve,
    Adaptive,
    Parallel,
    Spiral,
}

/// Whether an operation kind restricts geometry selection to **circular** loops.
/// Drilling and thread-milling target holes (the pick gives the centre, and for
/// threads the diameter), so a rectangle or open edge is not a valid pick.
pub fn op_selects_circles(kind: OpKind) -> bool {
    matches!(kind, OpKind::Drill | OpKind::Bore | OpKind::Thread)
}

/// Which node of the document is currently selected — what the tree highlights
/// and the inspector edits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Selection {
    /// The setup itself (its heights).
    #[default]
    Setup,
    /// The workpiece origin (datum) and program start point.
    Origin,
    /// The raw stock.
    Stock,
    /// The machine (travel limits + name).
    Machine,
    /// A tool in the setup's tool list, by index.
    Tool(usize),
    /// An operation, by its id.
    Operation(u32),
}

/// The result of a run: the CL-data program, its diagnostics, the viewport
/// scene (part outlines + backplot), and a simulated stock surface (the material
/// left after the toolpath cuts), ready for [`cam_render::MeshRenderer`].
#[derive(Clone, Debug, Default)]
pub struct RunOutcome {
    /// Only a fully completed job is eligible for export.
    pub completion: JobCompletion,
    /// Provenance of a background result, checked before publication.
    pub stamp: Option<crate::JobStamp>,
    pub program: Program,
    pub diagnostics: Vec<Diagnostic>,
    pub scene: Scene,
    /// Processed input blank for an inherited-stock setup, before its own cuts.
    pub initial_stock: Option<(Vec<MeshVertex>, Vec<u32>)>,
    /// Lossless material caches; render meshes are derived and need not be persisted.
    pub stock_snapshot: Option<cam_sim::StockSnapshot>,
    pub initial_stock_snapshot: Option<cam_sim::StockSnapshot>,
    /// Interleaved position+normal vertices of the cut stock surface.
    pub stock_vertices: Vec<MeshVertex>,
    /// Per-vertex flag distinguishing cut surfaces from original stock faces.
    pub stock_machined: Vec<bool>,
    /// Operation that exposed each stock vertex, used for selection highlighting.
    pub stock_operations: Vec<Option<u32>>,
    pub stock_cut_sources: Vec<Option<cam_sim::SurfaceLocation>>,
    /// Populated only by the separate, read-only removal/force analysis lane.
    pub removal_samples: Arc<Vec<cam_sim::RemovalSample>>,
    /// Triangle indices into `stock_vertices`.
    pub stock_indices: Vec<u32>,
    /// Collisions the material-removal simulation found (e.g. a rapid plowing
    /// through remaining stock) — the verification a green backplot can't give.
    pub collisions: Vec<Collision>,
}

impl RunOutcome {
    /// Whether any diagnostic is an error.
    pub fn has_errors(&self) -> bool {
        self.diagnostics
            .iter()
            .any(|d| d.severity == Severity::Error)
    }
}

/// Why an export could not be produced.
#[derive(Clone, Debug, PartialEq)]
pub enum ExportError {
    /// Nothing has been run yet.
    NothingToExport,
    /// Planning was cancelled; any retained motions are only a partial preview.
    Cancelled,
    /// Planning has not established a completed result.
    Incomplete,
    /// The last run produced errors, so its G-code is unsafe to emit.
    HasErrors,
    /// The simulation found `n` rapid moves through remaining stock.
    RapidThroughStock(usize),
    /// Non-cutting feeds or cutter tips contact remaining stock.
    NonCuttingThroughStock(usize),
    /// Tool or shank contacts a fixture clearance envelope.
    FixtureCollision(usize),
    /// The post rejected the program.
    Post(PostError),
}

impl std::fmt::Display for ExportError {
    /// Words an operator can act on.
    ///
    /// There was no `Display` at all, so `ExportToError` fell back to `{e:?}` and a
    /// blocked export reported itself as `Post(TravelExceeded { axis: 'X', span: 400.0,
    /// … })`. That is a developer's view of the problem shown to someone standing at a
    /// machine. `PostError` already words itself properly; this stops swallowing it.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExportError::NothingToExport => {
                write!(f, "there is no toolpath yet — run the job first (Home ▸ Run).")
            }
            ExportError::Cancelled => write!(
                f, "the last run was cancelled. Run the job again before exporting."
            ),
            ExportError::Incomplete => write!(
                f, "the last run is incomplete. Run the job again before exporting."
            ),
            ExportError::HasErrors => write!(
                f,
                "the last run reported errors. Fix the operations marked ⚠ in the project                  tree, then run again."
            ),
            ExportError::RapidThroughStock(n) => write!(
                f,
                "{n} rapid move{} would plow through material that has not been cut yet —                  a crash hazard, so the export is blocked.",
                if *n == 1 { "" } else { "s" }
            ),
            ExportError::NonCuttingThroughStock(n) => write!(
                f,
                "{n} non-cutting move(s) or cutter tips contact remaining stock — a crash hazard, so the export is blocked."
            ),
            ExportError::FixtureCollision(n) => write!(f, "{n} fixture clearance collision(s). Move the fixture or change the toolpath before exporting."),
            ExportError::Post(e) => write!(f, "{e}."),
        }
    }
}

impl std::error::Error for ExportError {}

impl From<PostError> for ExportError {
    fn from(e: PostError) -> Self {
        ExportError::Post(e)
    }
}

/// The application state and operations, GUI-agnostic.
pub struct AppController {
    machine: Machine,
    /// The selected post/controller dialect for export. **Session state, never read
    /// from a project file** — see [`open_project`](Self::open_project).
    post_kind: PostKind,
    estlcam: cam_post::EstlcamOptions,
    /// What the last opened project said it was built for. Reported, not applied.
    provenance: Provenance,
    regions: Vec<Polygon>,
    /// Open imported chains the importer could not close — engravable strokes.
    open_paths: Vec<Polyline>,
    document: History<Document>,
    defaults: JobParams,
    selection: Selection,
    /// The origin new operations are assigned to, and the one the origin inspector /
    /// pick flow edits. Defaults to the base origin (index 1).
    active_origin: u32,
    /// Active physical setup is view state, separate from the undoable definitions.
    active_setup_id: u64,
    /// Inspector defaults when there is no machining setup; never persisted.
    setup_defaults: Setup,
    source_name: String,
    /// The `.ocam` path this project was last saved to / opened from.
    current_path: Option<PathBuf>,
    saved_project: Option<ProjectSnapshot>,
    project_view: Option<crate::project::ProjectView>,
    /// An operation being created, awaiting a geometry pick in the viewport.
    pending_op: Option<PendingOp>,
    outcome: Option<RunOutcome>,
    nc: Option<String>,
    cad_selection: Vec<cam_model::EntityRef>,
    import_jobs: crate::import_jobs::ImportJobs<crate::cad_operations::PreparedImport>,
    document_epoch: u64,
    import_preview: Option<(u64, crate::cad_operations::PreparedImport)>,
    import_update: Option<CadModelUpdate>,
    jobs: crate::jobs::JobService,
    operation_cache: cam_toolpath::OperationCache,
    toolpath_fingerprints: BTreeMap<u32, String>,
    saved_verification: BTreeMap<u64, crate::project::StoredVerification>,
    automatic_toolpath_run: bool,
    simulation_accuracy: f32,
    block_export_on_verification_errors: bool,
    operation_id_floor: u32,
    setup_id_floor: u64,
    asset_id_floor: u64,
    instance_id_floor: u64,
    pattern_id_floor: u64,
}

#[derive(Clone, Copy)]
enum CadModelUpdate {
    Replace(u64),
    Reload(u64),
}

impl CadModelUpdate {
    fn asset_id(self) -> u64 {
        match self {
            Self::Replace(id) | Self::Reload(id) => id,
        }
    }
}

/// User-authored project state, excluding derived toolpaths and shop settings.
#[derive(PartialEq)]
struct ProjectSnapshot {
    document: Document,
    defaults: JobParams,
    source_name: String,
    regions: Vec<Polygon>,
    open_paths: Vec<Polyline>,
}

/// Which closed loop of an imported region a pick refers to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LoopPart {
    /// The region's outer boundary.
    Outer,
    /// The region's `n`-th hole.
    Hole(usize),
    /// An **open** imported path — a chain the importer could not close (lettering,
    /// a decorative stroke). These are not regions, so [`LoopRef::region`] indexes
    /// [`AppController::open_paths`] instead. Only operations that can machine an
    /// open path (engraving) may reference one.
    Open,
}

/// A reference to one closed loop (outer or a hole) of one region.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct LoopRef {
    /// Index into the regions — or, when `part` is [`LoopPart::Open`], into the
    /// open paths.
    pub region: usize,
    pub part: LoopPart,
}

impl LoopRef {
    pub fn outer(region: usize) -> Self {
        Self {
            region,
            part: LoopPart::Outer,
        }
    }
    /// A reference to the `i`-th **open** imported path.
    pub fn open(i: usize) -> Self {
        Self {
            region: i,
            part: LoopPart::Open,
        }
    }

    /// A reference to the `h`-th hole of region `region`.
    pub fn hole(region: usize, h: usize) -> Self {
        Self {
            region,
            part: LoopPart::Hole(h),
        }
    }

    /// Whether this refers to an open path rather than a closed loop.
    pub fn is_open(self) -> bool {
        matches!(self.part, LoopPart::Open)
    }
}

/// Profile, Chamfer and Engrave can machine open paths without inventing a closing edge.
pub fn op_accepts_open_paths(kind: OpKind) -> bool {
    matches!(kind, OpKind::Engrave | OpKind::Profile | OpKind::Chamfer)
}

/// Whether an operation kind takes **islands** — enclosed areas left standing. Only the
/// kinds that consume an *area* do: a pocket must be told what not to clear, and a carve
/// what not to carve (the counters of letters, most of all).
pub fn op_takes_islands(kind: OpKind) -> bool {
    matches!(kind, OpKind::Pocket | OpKind::Carve)
}

/// The spindle speed and feeds a new operation is created with — the wizard's
/// editable cutting-data row, seeded from the tool's nominals (see
/// [`Controller::seeded_cutting_for`]).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CuttingData {
    /// Spindle speed, rpm. `0` = unset (falls back to the job default at plan time).
    pub rpm: f64,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge (Z-entry) feed, mm/min.
    pub plunge_feed: f64,
}

/// An operation being created via the pick wizard: the kind, the tool, the picked
/// boundary/path loop (once chosen), and — for a pocket — the loops toggled as
/// excluded islands.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PendingOp {
    pub kind: OpKind,
    /// The chosen tool, or `None` until one is picked. Nothing is committed until
    /// [`AppController::confirm_operation`], so tool and geometry may be chosen in
    /// **either order** and either may be changed until then.
    pub tool: Option<u32>,
    /// The boundary/path loop, set on the first pick. `None` while awaiting it.
    pub boundary: Option<LoopRef>,
    /// Loops toggled as excluded islands (pocket island mode).
    pub islands: Vec<LoopRef>,
    /// Further circular holes for the Drill/Thread point-list backend.
    pub additional_holes: Vec<LoopRef>,
    /// The snapped start/lead-in point (part XY) captured with the boundary pick.
    pub start: Option<[f64; 2]>,
    /// When set, this wizard is **reinitialising** an existing operation: on
    /// completion the new operation takes that one's place in the order rather than
    /// being appended. Lets a wrong pick be redone without losing the operation's
    /// position in the job.
    pub replacing: Option<u32>,
}

/// A viewport object-snap: which kind of point on the geometry the cursor
/// resolves to during an operation pick. Priority runs End → Mid → Quadrant →
/// Nearest (Nearest is the always-catches fallback when enabled).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum SnapKind {
    /// A real corner (segment endpoint at an angle).
    End,
    /// The midpoint of a straight edge (between two corners).
    Mid,
    /// A cardinal quadrant of an arc/circle (Phase 2).
    Quadrant,
    /// The nearest point on the edge under the cursor (opt-in fallback).
    Nearest,
}

impl SnapKind {
    pub(crate) fn priority(self) -> u8 {
        match self {
            SnapKind::End => 0,
            SnapKind::Mid => 1,
            SnapKind::Quadrant => 2,
            SnapKind::Nearest => 3,
        }
    }
}

/// A resolved object-snap: the loop it sits on, the point (part XY), and kind.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SnapHit {
    pub loop_ref: LoopRef,
    pub point: [f64; 2],
    pub kind: SnapKind,
}

/// The result of a viewport pick during the wizard.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PickResult {
    /// Geometry was recorded — the boundary was set (or an island toggled) and the
    /// wizard is still active. Picking never finalises: creation happens only in
    /// [`AppController::confirm_operation`], so the tool and the geometry can be
    /// chosen in either order and either can be changed until Confirm.
    Selecting,
    /// The pick missed every boundary line.
    Missed,
}

/// Why a project save/open failed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProjectError {
    /// The file could not be read or written.
    Io(String),
    /// The project JSON could not be produced or parsed.
    Json(String),
    /// The file parsed, but its schema version could not be brought to this build's —
    /// too new to understand, too old to still be migrated, or damaged in a way a
    /// migration step could name.
    ///
    /// Separate from [`Json`](Self::Json) because the two ask the user for different
    /// things: a format error means the file is broken, while a schema refusal usually
    /// means the file is *fine* and this build is the wrong one to open it with.
    Schema(cam_model::migrate::MigrationError),
    /// The `.ocam` opened as a project is actually a **tool library**.
    NotAProject,
}

impl std::fmt::Display for ProjectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProjectError::Io(e) => write!(f, "file error: {e}"),
            ProjectError::Json(e) => write!(f, "project format error: {e}"),
            // The migration error already reads as a sentence to a user; prefixing it
            // with a category would only bury the part that says what to do.
            ProjectError::Schema(e) => write!(f, "{e}"),
            ProjectError::NotAProject => {
                write!(f, "this .ocam file is a tool library, not a project")
            }
        }
    }
}

impl From<crate::project::LoadError> for ProjectError {
    fn from(e: crate::project::LoadError) -> Self {
        match e {
            crate::project::LoadError::Json(e) => ProjectError::Json(e),
            crate::project::LoadError::Migration(e) => ProjectError::Schema(e),
            // A too-new *library* reaching the project path means the user opened one
            // as a project. Report the version, not "not a project": the version is
            // the actionable half, and it is why the file could not be read at all.
            e @ crate::project::LoadError::LibraryVersion { .. } => {
                ProjectError::Json(e.to_string())
            }
        }
    }
}

impl std::error::Error for ProjectError {}

/// What a project recorded about the machine and post it was built for.
///
/// **Provenance only.** Opening a project never adopts either: a machine is shop-local,
/// and the envelope is what gates an export, so a file that could change your machine
/// could disarm that gate. Kept so the app can *say* what a job was authored for, which
/// is genuinely useful when a file arrives from someone else's shop.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Provenance {
    /// The machine the project was saved with (schema v11+), if it recorded one.
    pub machine: Option<Machine>,
    /// The post it was saved with, if it recorded one.
    pub post: Option<PostKind>,
}

impl Provenance {
    /// A one-line note for the operator when the file was built for something other
    /// than what is selected now — `None` when it matches, or said nothing.
    ///
    /// The machine half names the travel, because that is the number that decides
    /// whether the job can be cut here at all.
    pub fn mismatch_note(&self, machine: &Machine, post: PostKind) -> Option<String> {
        let mut parts = Vec::new();
        if let Some(m) = &self.machine {
            if m.envelope != machine.envelope || m.name != machine.name {
                let (bx, by, bz) = m.envelope.extent();
                let (cx, cy, cz) = machine.envelope.extent();
                parts.push(format!(
                    "built for \"{}\" ({bx:.0}×{by:.0}×{bz:.0} mm); yours is \"{}\" \
                     ({cx:.0}×{cy:.0}×{cz:.0} mm)",
                    m.name, machine.name
                ));
            }
        }
        if let Some(p) = self.post {
            if p != post {
                parts.push(format!("built for the {p} post; yours is {post}"));
            }
        }
        (!parts.is_empty()).then(|| format!("This job was {}.", parts.join(", and ")))
    }
}

/// Why exporting G-code to a file failed.
#[derive(Clone, Debug, PartialEq)]
pub enum ExportToError {
    /// The toolpath could not be posted (see [`ExportError`]).
    Export(ExportError),
    /// The `.nc` file could not be written.
    Io(String),
}

impl std::fmt::Display for ExportToError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExportToError::Export(e) => write!(f, "{e}"),
            ExportToError::Io(e) => write!(f, "file error: {e}"),
        }
    }
}

impl std::error::Error for ExportToError {}

impl AppController {
    /// A fresh controller for `machine`, with an empty "Untitled" document and no
    /// geometry.
    pub fn new(machine: Machine) -> Self {
        let defaults = JobParams::default();
        let mut controller = Self {
            machine,
            post_kind: PostKind::default(),
            estlcam: cam_post::EstlcamOptions::default(),
            provenance: Provenance::default(),
            regions: Vec::new(),
            open_paths: Vec::new(),
            document: History::new(Document::empty()),
            setup_defaults: default_setup(&defaults),
            defaults,
            selection: Selection::default(),
            active_origin: 1,
            active_setup_id: 0,
            source_name: String::new(),
            current_path: None,
            saved_project: None,
            project_view: None,
            pending_op: None,
            outcome: None,
            nc: None,
            cad_selection: Vec::new(),
            import_jobs: Default::default(),
            document_epoch: 0,
            import_preview: None,
            import_update: None,
            jobs: Default::default(),
            operation_cache: Default::default(),
            toolpath_fingerprints: Default::default(),
            saved_verification: Default::default(),
            automatic_toolpath_run: false,
            simulation_accuracy: 1.0,
            block_export_on_verification_errors: true,
            operation_id_floor: 0,
            setup_id_floor: 1,
            asset_id_floor: 1,
            instance_id_floor: 1,
            pattern_id_floor: 1,
        };
        controller.mark_project_saved();
        controller
    }

    /// The machine being driven.
    pub fn machine(&self) -> &Machine {
        &self.machine
    }

    /// The selected post/controller dialect for export.
    pub fn post_kind(&self) -> PostKind {
        self.post_kind
    }

    /// Choose the post dialect. Clears any cached `.nc` so the next export re-posts.
    pub fn set_post_kind(&mut self, kind: PostKind) {
        if self.post_kind != kind {
            self.post_kind = kind;
            self.invalidate_export();
        }
    }

    /// The active machine's Estlcam coolant/MMS output wiring.
    pub fn estlcam_options(&self) -> cam_post::EstlcamOptions {
        self.estlcam
    }

    /// Changing output wiring requires re-posting, but no toolpath recalculation.
    pub fn set_estlcam_options(&mut self, options: cam_post::EstlcamOptions) {
        if self.estlcam != options {
            self.estlcam = options;
            self.invalidate_export();
        }
    }

    /// The options every export posts with. One construction, so anything that needs
    /// to *predict* the output (see [`datum_label`](Self::datum_label)) reads the same
    /// settings the file is actually written from.
    fn post_options(&self) -> PostOptions {
        PostOptions {
            program_name: Some(self.program_name()),
            estlcam: self.estlcam,
            ..Default::default()
        }
    }

    /// What the selected post will call work datum `index` — `"G55"`, `"H2"` — or
    /// `None` when that control has no word for it, in which case an export would
    /// refuse. The vocabulary belongs to the post (Okuma counts `H<n>` literally, the
    /// ISO families count up from the program's work offset), so it is asked rather
    /// than re-derived here.
    pub fn datum_label(&self, index: u32) -> Option<String> {
        self.post_kind
            .datum_label(index, self.post_options().work_offset)
    }

    /// Edit the machine (envelope/name/limits). Re-run because its max Z can
    /// define the tool-change height, and its validity is a planning prerequisite.
    /// Make `machine` the active one — what an export is gated against.
    ///
    /// Session state: never read from a project file, because the envelope is what gates
    /// an export and a file that could set it could disarm that gate. Clears any cached
    /// `.nc`, since the ceilings it was checked against have changed.
    pub fn set_machine(&mut self, machine: Machine) {
        if self.machine != machine {
            self.machine = machine;
            self.invalidate();
        }
    }

    pub fn edit_machine(&mut self, f: impl FnOnce(&mut Machine)) {
        let mut machine = self.machine.clone();
        f(&mut machine);
        self.set_machine(machine);
    }

    /// The current document.
    pub fn document(&self) -> &Document {
        self.document.current()
    }

    pub fn next_pattern_id(&self) -> u64 {
        self.pattern_id_floor
            .max(self.document().pattern_id_floor)
            .max(1)
    }

    /// The current setup, or read-only creation defaults before one exists.
    fn setup(&self) -> &Setup {
        self.inspection_setup()
    }

    /// Read-only values for inspectors and the viewport before a setup exists.
    /// Machining still requires an actual entry in `document.setups`.
    pub fn inspection_setup(&self) -> &Setup {
        self.document()
            .active_setup()
            .unwrap_or(&self.setup_defaults)
    }

    /// The seed defaults used when a document is generated on import.
    pub fn defaults(&self) -> &JobParams {
        &self.defaults
    }

    /// The current selection.
    pub fn selection(&self) -> Selection {
        self.selection
    }

    /// The imported regions.
    pub fn regions(&self) -> &[Polygon] {
        &self.regions
    }

    /// The imported **open** paths — chains that could not be closed, machinable by
    /// engraving.
    pub fn open_paths(&self) -> &[Polyline] {
        &self.open_paths
    }

    /// Whether any geometry is loaded at all — closed regions **or** open paths.
    ///
    /// Use this rather than `regions().is_empty()` for "is there something to work
    /// on": a drawing can legitimately be nothing but engravable strokes, and
    /// treating those as "no geometry" hides them from the viewport and blocks
    /// operation creation.
    pub fn has_geometry(&self) -> bool {
        !self.regions.is_empty()
            || !self.open_paths.is_empty()
            || !self.document().geometry.instances.is_empty()
    }

    /// The name of the loaded drawing.
    pub fn source_name(&self) -> &str {
        &self.source_name
    }

    /// The most recent run, if any.
    pub fn outcome(&self) -> Option<&RunOutcome> {
        self.outcome.as_ref()
    }

    /// The most recently exported G-code, if any.
    pub fn exported_nc(&self) -> Option<&str> {
        self.nc.as_deref()
    }

    /// The operation with `id`, if present.
    pub fn operation(&self, id: u32) -> Option<&Operation> {
        self.setup().operations.iter().find(|o| o.id() == id)
    }

    /// The currently-selected operation, if an operation is selected.
    pub fn selected_operation(&self) -> Option<&Operation> {
        match self.selection {
            Selection::Operation(id) => self.operation(id),
            _ => None,
        }
    }

    /// Load geometry from DXF text, replacing any current drawing (used for the
    /// bundled sample and tests). Returns the number of regions.
    pub fn open_dxf(&mut self, text: &str, name: impl Into<String>) -> Result<usize, ImportError> {
        let import = read_dxf_str(text, &ImportOptions::default())?;
        self.install_import(import.regions, import.open_chains, name.into(), true);
        Ok(self.regions.len())
    }

    /// Import a CAD file (`.dxf` ASCII/binary or `.dwg`) via acadrust, replacing
    /// any current drawing with **geometry only** — no operations are fabricated;
    /// the user creates them by picking boundaries. Returns the region count. This
    /// does not set the project path — an imported drawing is saved as a new
    /// `.ocam` project.
    pub fn import_cad(&mut self, path: impl AsRef<Path>) -> Result<usize, ImportError> {
        let prepared =
            crate::cad_operations::prepare_import(path.as_ref()).map_err(ImportError::Io)?;
        self.commit_cad_import(prepared).map_err(ImportError::Io)
    }

    /// Persistent imported models and operation links.
    pub fn geometry_state(&self) -> &cam_model::GeometryState {
        &self.document().geometry
    }
    pub fn cad_selection(&self) -> &[cam_model::EntityRef] {
        &self.cad_selection
    }
    pub fn set_cad_selection(&mut self, entities: Vec<cam_model::EntityRef>) {
        // Keep the full requested inspection selection. Eligibility must report
        // stale or incompatible references rather than silently discard them.
        self.cad_selection = entities;
    }
    /// Replace an operation geometry field atomically after validating every
    /// reference. Topology/strategy eligibility is reported by the preview.
    pub fn try_set_cad_selection(
        &mut self,
        entities: Vec<cam_model::EntityRef>,
    ) -> Result<(), String> {
        crate::cad_operations::validate_selection(self.geometry_state(), &entities)?;
        self.cad_selection = entities;
        Ok(())
    }
    /// Adaptive selections may combine faces and bodies across setup instances.
    pub fn try_set_adaptive_cad_selection(
        &mut self,
        entities: Vec<cam_model::EntityRef>,
    ) -> Result<(), String> {
        crate::cad_operations::validate_selection(self.geometry_state(), &entities)?;
        self.cad_selection = entities;
        Ok(())
    }
    /// Apply a viewport/tree pick, with Ctrl/Command allowing homogeneous
    /// multi-selection. Rejected additions preserve the previous selection.
    pub fn toggle_cad_selection(
        &mut self,
        entity: cam_model::EntityRef,
        additive: bool,
    ) -> Result<(), String> {
        let mut selected = if additive {
            self.cad_selection.clone()
        } else {
            Vec::new()
        };
        if let Some(index) = selected.iter().position(|r| *r == entity) {
            selected.remove(index);
        } else {
            selected.push(entity);
        }
        self.try_set_cad_selection(selected)
    }
    pub fn toggle_adaptive_cad_selection(
        &mut self,
        entity: cam_model::EntityRef,
        additive: bool,
    ) -> Result<(), String> {
        let mut selected = if additive {
            self.cad_selection.clone()
        } else {
            Vec::new()
        };
        if let Some(index) = selected.iter().position(|reference| *reference == entity) {
            selected.remove(index);
        } else {
            selected.push(entity);
        }
        self.try_set_adaptive_cad_selection(selected)
    }
    pub fn start_cad_import(&mut self, path: PathBuf) -> u64 {
        self.import_preview = None;
        self.import_update = None;
        self.import_jobs.start(self.document_epoch, move || {
            crate::cad_operations::prepare_import(&path)
        })
    }
    pub fn start_cad_replacement(&mut self, asset_id: u64, path: PathBuf) -> Result<u64, String> {
        self.start_cad_model_update(CadModelUpdate::Replace(asset_id), path)
    }
    /// Reread a CAD file in the background for an existing model.
    pub fn start_cad_reload(&mut self, asset_id: u64, path: PathBuf) -> Result<u64, String> {
        self.start_cad_model_update(CadModelUpdate::Reload(asset_id), path)
    }
    fn start_cad_model_update(
        &mut self,
        update: CadModelUpdate,
        path: PathBuf,
    ) -> Result<u64, String> {
        if self.geometry_state().asset(update.asset_id()).is_none() {
            return Err("Missing CAD asset".into());
        }
        let request = self.start_cad_import(path);
        self.import_update = Some(update);
        Ok(request)
    }
    pub fn cad_import_replacement(&self) -> Option<u64> {
        self.import_update.map(CadModelUpdate::asset_id)
    }
    #[cfg(feature = "gui")]
    pub(crate) fn cad_import_reloading(&self) -> bool {
        matches!(self.import_update, Some(CadModelUpdate::Reload(_)))
    }
    pub fn cad_import_pending(&self) -> bool {
        self.import_jobs.pending()
    }
    pub fn cancel_cad_import(&mut self) {
        self.import_jobs.cancel();
        self.import_preview = None;
        self.import_update = None;
    }
    pub fn poll_cad_import(&mut self) -> Option<Result<usize, String>> {
        self.import_jobs.poll(self.document_epoch).map(|result| {
            result.map(|p| {
                let count = p.preview().items;
                self.import_preview = Some((self.document_epoch, p));
                count
            })
        })
    }
    pub fn cad_import_preview(&self) -> Option<crate::CadImportPreview> {
        self.import_preview.as_ref().map(|(_, p)| p.preview())
    }
    pub fn commit_cad_preview(&mut self) -> Result<usize, String> {
        let (epoch, prepared) = self.import_preview.take().ok_or("No import preview")?;
        if epoch != self.document_epoch {
            return Err("Project changed after preview; import again".into());
        }
        if let Some(update) = self.import_update.take() {
            let id = update.asset_id();
            match prepared {
                crate::cad_operations::PreparedImport::Solid { asset, instances } => {
                    let mut existing: Vec<_> = self
                        .geometry_state()
                        .instances
                        .iter()
                        .filter(|i| i.asset_id == id)
                        .map(|i| i.body_id)
                        .collect();
                    let mut imported: Vec<_> = instances.iter().map(|i| i.body_id).collect();
                    existing.sort_unstable();
                    imported.sort_unstable();
                    if existing != imported {
                        return Err("The source changes the model's body instances. Import it as a new model and review its geometry.".into());
                    }
                    let count = instances.len();
                    self.update_cad_asset(update, asset)?;
                    Ok(count)
                }
                _ => Err("Updating a model requires a STEP or IGES solid".into()),
            }
        } else {
            self.commit_cad_import(prepared)
        }
    }
    /// Add an immutable imported asset and its placements after preview validation.
    pub fn add_cad_asset(
        &mut self,
        asset: cam_model::GeometryAsset,
        instances: Vec<cam_model::GeometryInstance>,
    ) -> Result<usize, String> {
        self.commit_cad_import(crate::cad_operations::PreparedImport::Solid { asset, instances })
    }
    fn commit_cad_import(
        &mut self,
        prepared: crate::cad_operations::PreparedImport,
    ) -> Result<usize, String> {
        match prepared {
            crate::cad_operations::PreparedImport::Drawing { import, name } => {
                let count = import.regions.len() + import.open_chains.len();
                if self.source_name.is_empty() {
                    self.source_name = name;
                    self.project_view = None;
                }
                self.edit_document(move |doc| {
                    doc.geometry.drawing_regions.extend(import.regions);
                    doc.geometry.drawing_paths.extend(import.open_chains);
                });
                self.sync_drawing_geometry();
                self.pending_op = None;
                self.cad_selection.clear();
                self.invalidate();
                Ok(count)
            }
            crate::cad_operations::PreparedImport::Solid {
                mut asset,
                mut instances,
            } => {
                let state = self.geometry_state();
                asset.id = (state.assets.iter().map(|a| a.id).max().unwrap_or(0) + 1)
                    .max(self.document().asset_id_floor)
                    .max(self.asset_id_floor);
                asset.revision = 1;
                let next_instance = (state.instances.iter().map(|i| i.id).max().unwrap_or(0) + 1)
                    .max(self.document().instance_id_floor)
                    .max(self.instance_id_floor);
                for (index, instance) in instances.iter_mut().enumerate() {
                    instance.id = next_instance + index as u64;
                    instance.asset_id = asset.id;
                }
                let count = instances.len();
                let mut preview = state.clone();
                preview.assets.push(asset.clone());
                preview.instances.extend(instances.clone());
                preview.validate()?;
                if self.source_name.is_empty() {
                    self.source_name = asset.name.clone();
                }
                self.edit_document(move |doc| {
                    doc.geometry.assets.push(asset);
                    doc.geometry.instances.extend(instances);
                });
                self.selection = Selection::Setup;
                self.cad_selection.clear();
                self.invalidate();
                Ok(count)
            }
        }
    }
    pub fn preview_cad_operation(
        &self,
        kind: OpKind,
        entities: &[cam_model::EntityRef],
    ) -> Result<cam_geometry3d::ResolvedInput, String> {
        let geometry = self.setup_geometry_state()?;
        if matches!(kind, OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral) {
            cam_geometry3d::validate_adaptive_selection(&geometry, entities)?;
        } else {
            crate::cad_operations::validate_selection(&geometry, entities)?;
        }
        let binding = cam_geometry3d::resolve_selection_advisory(
            &geometry,
            entities,
            crate::cad_operations::recipe(kind, &geometry, entities)?,
            self.stock_box().1[2],
        )?;
        cam_geometry3d::resolve_binding_advisory(&geometry, &binding)
    }
    pub(crate) fn observe_geometry_binding(
        &self,
        kind: OpKind,
        binding: &cam_model::GeometryBinding,
    ) {
        let finding = self.setup_geometry_state().ok().and_then(|geometry| {
            cam_geometry3d::resolve_binding(&geometry, binding)
                .err()
                .or_else(|| {
                    cam_geometry3d::verify_selection(
                        &geometry,
                        &binding.selected,
                        binding.recipe.clone(),
                        self.stock_box().1[2],
                    )
                })
        });
        if let Err(error) = crate::geometry_verification::record(
            self,
            &binding.selected,
            Some(kind),
            None,
            Some(binding),
            finding.as_deref(),
            None,
        ) {
            eprintln!("Could not record geometry verification: {error}");
        }
    }
    /// Both creation orders and the confirmation button require extractable
    /// geometry; automatic feature-recognition findings remain advisory.
    pub fn cad_operation_eligibility(
        &self,
        kind: OpKind,
    ) -> Result<cam_geometry3d::ResolvedInput, String> {
        self.preview_cad_operation(kind, self.cad_selection())
    }
    /// Align a selected planar face with the fixed machining XY plane at Z0.
    /// The longest straight boundary edge sets X; the chosen face origin is datum.
    pub fn align_cad_top_face(&mut self, reference: cam_model::EntityRef) -> Result<(), String> {
        use cam_model::{CadCurve, CadSurface, EntityKind, RigidTransform};
        if reference.kind != EntityKind::Face || !self.geometry_state().entity_exists(&reference) {
            return Err("Select one planar top face".into());
        }
        let asset = self.geometry_state().asset(reference.asset_id).unwrap();
        let face = asset
            .topology
            .faces
            .iter()
            .find(|f| f.id == reference.id)
            .unwrap();
        let CadSurface::Plane { origin, normal } = face.surface else {
            return Err("Machining top must be planar".into());
        };
        let normalize = |v: [f64; 3]| {
            let length = v.iter().map(|n| n * n).sum::<f64>().sqrt();
            v.map(|n| n / length)
        };
        let normal = normalize(normal);
        let mut candidates = Vec::new();
        for wire in asset
            .topology
            .loops
            .iter()
            .filter(|w| w.face_id == face.id && w.is_outer)
        {
            for u in &wire.edge_uses {
                if let Some(edge) = asset.topology.edges.iter().find(|e| e.id == u.edge_id) {
                    if let CadCurve::Line { start, end } = edge.curve {
                        let v = std::array::from_fn::<_, 3, _>(|i| end[i] - start[i]);
                        let d = v.iter().zip(normal).map(|(a, b)| a * b).sum::<f64>();
                        let projected = std::array::from_fn::<_, 3, _>(|i| v[i] - d * normal[i]);
                        let length = projected.iter().map(|n| n * n).sum::<f64>();
                        if length > 1e-12 {
                            candidates.push((length, projected));
                        }
                    }
                }
            }
        }
        let x = if let Some((_, v)) = candidates.into_iter().max_by(|a, b| a.0.total_cmp(&b.0)) {
            normalize(v)
        } else {
            let seed = if normal[0].abs() < 0.9 {
                [1., 0., 0.]
            } else {
                [0., 1., 0.]
            };
            let d = seed.iter().zip(normal).map(|(a, b)| a * b).sum::<f64>();
            normalize(std::array::from_fn(|i| seed[i] - d * normal[i]))
        };
        let y = [
            normal[1] * x[2] - normal[2] * x[1],
            normal[2] * x[0] - normal[0] * x[2],
            normal[0] * x[1] - normal[1] * x[0],
        ];
        let rotation = [x, y, normal];
        let translation =
            rotation.map(|row| -row.iter().zip(origin).map(|(a, b)| a * b).sum::<f64>());
        self.place_cad_instance(
            reference.instance_id,
            RigidTransform {
                rotation,
                translation,
            },
        )
    }
    /// Resolve and commit selected CAD geometry as a single undoable operation.
    pub fn create_cad_operation(
        &mut self,
        kind: OpKind,
        entities: Vec<cam_model::EntityRef>,
    ) -> Result<u32, String> {
        self.create_cad_operation_with_tool(kind, entities, self.first_tool_number())
    }
    pub fn create_cad_operation_with_tool(
        &mut self,
        kind: OpKind,
        entities: Vec<cam_model::EntityRef>,
        tool: u32,
    ) -> Result<u32, String> {
        let id = self.next_op_id();
        let (mut op, binding) = crate::cad_operations::create_operation(
            self.document(),
            kind,
            entities,
            id,
            tool,
            self.active_origin,
            self.defaults,
        )?;
        if !self.setup().tools.iter().any(|t| t.number == op.tool()) {
            return Err(
                "Choose a tool from the tool library before creating the CAD operation".into(),
            );
        }
        if let Some(cooling) = self
            .setup()
            .tools
            .iter()
            .find(|t| t.number == tool)
            .map(|t| t.cooling)
        {
            op.set_cooling(cooling);
        }
        if let Some(preset) = self
            .setup()
            .tools
            .iter()
            .find(|t| t.number == tool)
            .and_then(|t| t.preset_for_material(self.setup().material_id))
        {
            op.apply_tool_preset(preset)?;
        }
        let mut preview = self.document().clone();
        preview.geometry.bindings.insert(id, binding.clone());
        let settings = if binding.recipe == cam_model::ExtractionRecipe::PocketBoundary
            || matches!(
                kind,
                OpKind::Chamfer
                    | OpKind::Engrave
                    | OpKind::Carve
                    | OpKind::Thread
                    | OpKind::Adaptive
                    | OpKind::Parallel
                    | OpKind::Spiral
                    | OpKind::Bore
            ) {
            crate::cad_operations::selection_height_defaults(&preview, &op)
        } else {
            cam_model::OperationHeightSettings::default()
        };
        preview.setup_mut().height_settings.insert(id, settings);
        preview.setup_mut().operations.push(op);
        preview.cad_linked_operations.insert(id);
        crate::cad_operations::validate_binding_edit(&mut preview, id)?;
        self.observe_geometry_binding(kind, &binding);
        self.edit_document(move |doc| *doc = preview);
        self.select(Selection::Operation(id));
        self.invalidate();
        Ok(id)
    }
    pub fn edit_cad_geometry(
        &mut self,
        id: u32,
        entities: Vec<cam_model::EntityRef>,
    ) -> Result<(), String> {
        let mut doc = self.document().clone();
        if matches!(
            self.operation(id),
            Some(Operation::Adaptive(_) | Operation::Parallel(_) | Operation::Spiral(_))
        ) {
            cam_geometry3d::validate_adaptive_selection(self.geometry_state(), &entities)?;
        } else {
            crate::cad_operations::validate_selection(self.geometry_state(), &entities)?;
        }
        let pocket_recipe = if matches!(self.operation(id), Some(Operation::Pocket(_))) {
            Some(crate::cad_operations::recipe(
                OpKind::Pocket,
                &crate::cad_operations::geometry_for_setup(&doc)?,
                &entities,
            )?)
        } else {
            None
        };
        let binding = doc
            .geometry
            .bindings
            .get_mut(&id)
            .ok_or("Operation has explicit geometry")?;
        if let Some(recipe) = pocket_recipe {
            binding.recipe = recipe;
        }
        binding.selected = entities;
        binding.status = cam_model::ResolutionStatus::Resolved;
        crate::cad_operations::validate_binding_edit(&mut doc, id)?;
        self.edit_document(move |current| *current = doc);
        self.select(Selection::Operation(id));
        self.invalidate();
        Ok(())
    }
    pub fn override_cad_heights(
        &mut self,
        id: u32,
        top: Option<f64>,
        bottom: Option<f64>,
    ) -> Result<(), String> {
        let mut doc = self.document().clone();
        let binding = doc
            .geometry
            .bindings
            .get_mut(&id)
            .ok_or("Operation has explicit geometry")?;
        binding.overrides = cam_model::GeometryOverrides { top, bottom };
        crate::cad_operations::validate_binding_edit(&mut doc, id)?;
        self.edit_document(move |current| *current = doc);
        if let Some(operation) = self.operation(id) {
            self.observe_geometry_binding(
                Self::operation_kind(operation),
                &self.geometry_state().bindings[&id],
            );
        }
        self.invalidate();
        Ok(())
    }
    pub fn place_cad_instance(
        &mut self,
        id: u64,
        placement: cam_model::RigidTransform,
    ) -> Result<(), String> {
        self.place_cad_instances(vec![(id, placement)])
    }

    /// Apply placements as one undoable edit, validating the entire selection first.
    pub fn place_cad_instances(
        &mut self,
        placements: Vec<(u64, cam_model::RigidTransform)>,
    ) -> Result<(), String> {
        let mut ids = std::collections::BTreeSet::new();
        for (id, placement) in &placements {
            if !placement.is_rigid() {
                return Err("Placement must be a finite rigid transform".into());
            }
            if !ids.insert(*id) || self.geometry_state().instance(*id).is_none() {
                return Err("Missing or duplicate CAD instance".into());
            }
        }
        if placements.is_empty() {
            return Err("No CAD bodies selected".into());
        }
        self.edit_document(move |doc| {
            for (id, placement) in &placements {
                if let Some(instance) = doc.geometry.instances.iter_mut().find(|i| i.id == *id) {
                    instance.placement = *placement;
                }
            }
            let geometry = doc.geometry.clone();
            for setup in &mut doc.setups {
                let refs = &setup.frame.references;
                if let Some((id, placement)) = placements
                    .iter()
                    .find(|(id, _)| refs.model_instance == Some(*id))
                {
                    if refs.x.is_none() && refs.y.is_none() && refs.z.is_none() {
                        if let Ok(mut frame) =
                            cam_model::SetupFrame::from_model_instance(&geometry, *id)
                        {
                            frame.references.origin = refs.origin.clone();
                            if matches!(refs.origin, cam_model::FrameOrigin::ModelOrigin) {
                                setup.origin = frame.to_setup_vector(placement.translation);
                            }
                            setup.frame = frame;
                        }
                    }
                }
            }
            crate::cad_operations::refresh_binding_heights(doc);
        });
        self.invalidate();
        Ok(())
    }
    /// Replacement never guesses entity identity across exports. Existing bindings
    /// retain their source selection and overrides until the user reviews a new one.
    pub fn replace_cad_asset(
        &mut self,
        id: u64,
        replacement: cam_model::GeometryAsset,
    ) -> Result<(), String> {
        self.update_cad_asset(CadModelUpdate::Replace(id), replacement)
    }

    fn update_cad_asset(
        &mut self,
        update: CadModelUpdate,
        mut replacement: cam_model::GeometryAsset,
    ) -> Result<(), String> {
        let id = update.asset_id();
        let old = self.geometry_state().asset(id).ok_or("Missing CAD asset")?;
        replacement.id = id;
        // Reuse references only when the entire topology, including entity IDs, matches.
        let keep_references =
            matches!(update, CadModelUpdate::Reload(_)) && replacement.topology == old.topology;
        replacement.revision = if keep_references {
            replacement.topology = old.topology.clone();
            old.revision
        } else {
            old.revision + 1
        };
        if replacement == *old {
            return Ok(());
        }
        replacement.topology.validate()?;
        if self
            .geometry_state()
            .instances
            .iter()
            .filter(|i| i.asset_id == id)
            .any(|i| {
                !replacement
                    .topology
                    .bodies
                    .iter()
                    .any(|b| b.id == i.body_id)
            })
        {
            return Err(
                "Replacement changes body identities; remove and import it as a new asset".into(),
            );
        }
        let mut preview = self.geometry_state().clone();
        if let Some(asset) = preview.assets.iter_mut().find(|a| a.id == id) {
            *asset = replacement;
        }
        if !keep_references {
            let reason = match update {
                CadModelUpdate::Replace(_) => "Model replaced; review and reselect its geometry",
                CadModelUpdate::Reload(_) => {
                    "Reloaded model geometry changed; review and reselect it"
                }
            };
            for binding in preview
                .bindings
                .values_mut()
                .filter(|b| b.selected.iter().any(|r| r.asset_id == id))
            {
                binding.status = cam_model::ResolutionStatus::NeedsReselection {
                    reason: reason.into(),
                };
            }
        }
        preview.validate()?;
        self.edit_document(move |doc| doc.geometry = preview);
        self.cad_selection.clear();
        self.invalidate();
        Ok(())
    }
    pub fn remove_cad_asset(&mut self, id: u64) -> bool {
        if self.geometry_state().asset(id).is_none() {
            return false;
        }
        self.edit_document(move |doc| {
            doc.geometry.assets.retain(|a| a.id != id);
            doc.geometry.instances.retain(|i| i.asset_id != id);
            let remaining: BTreeSet<_> = doc.geometry.instances.iter().map(|i| i.id).collect();
            for setup in &mut doc.setups {
                setup.model_instances.retain(|id| remaining.contains(id));
            }
            for binding in doc
                .geometry
                .bindings
                .values_mut()
                .filter(|b| b.selected.iter().any(|r| r.asset_id == id))
            {
                binding.status = cam_model::ResolutionStatus::NeedsReselection {
                    reason: "Source model removed".into(),
                };
            }
        });
        self.cad_selection.clear();
        self.invalidate();
        true
    }

    /// Install freshly imported regions: generate a document, select the first
    /// operation (or the Setup when none is seeded), and reset derived state.
    /// Shared by every import path. `seed_ops` seeds a profile per boundary/hole
    /// (the bundled sample demo) versus bringing in bare geometry (real imports).
    fn install_import(
        &mut self,
        regions: Vec<Polygon>,
        open_paths: Vec<Polyline>,
        name: String,
        seed_ops: bool,
    ) {
        self.regions = regions;
        self.open_paths = open_paths;
        self.source_name = name;
        self.project_view = None;
        let mut document = self.generate_document(seed_ops);
        document.geometry.drawing_regions = self.regions.clone();
        document.geometry.drawing_paths = self.open_paths.clone();
        self.operation_cache = Default::default();
        self.toolpath_fingerprints.clear();
        self.saved_verification.clear();
        self.document = History::new(document);
        self.active_setup_id = self.setup().id;
        self.cad_selection.clear();
        self.active_origin = self.setup().origin_index;
        self.pending_op = None;
        self.current_path = None;
        self.selection = self
            .setup()
            .operations
            .first()
            .map(|op| Selection::Operation(op.id()))
            .unwrap_or(Selection::Setup);
        self.invalidate();
    }

    /// Reset to a fresh, empty "Untitled" project.
    pub fn new_project(&mut self) {
        self.cancel_cad_import();
        self.cad_selection.clear();
        self.provenance = Provenance::default();
        self.regions.clear();
        self.open_paths.clear();
        self.operation_cache = Default::default();
        self.toolpath_fingerprints.clear();
        self.saved_verification.clear();
        self.document = History::new(Document::empty());
        self.setup_defaults = default_setup(&self.defaults);
        self.active_setup_id = 0;
        self.active_origin = 1;
        self.source_name.clear();
        self.project_view = None;
        self.current_path = None;
        self.pending_op = None;
        self.selection = Selection::Setup;
        self.invalidate();
        self.mark_project_saved();
    }

    /// What the most recently opened project recorded about the machine and post it was
    /// built for — **provenance, never adopted**.
    ///
    /// Empty for a project that recorded neither (anything before schema v11) and after
    /// a fresh project.
    pub fn provenance(&self) -> &Provenance {
        &self.provenance
    }

    /// The `.ocam` file this project was last saved to / opened from, if any.
    pub fn current_path(&self) -> Option<&Path> {
        self.current_path.as_deref()
    }

    /// Desktop view last opened or supplied before saving; preserved by headless edits.
    pub fn project_view(&self) -> Option<&crate::project::ProjectView> {
        self.project_view.as_ref()
    }

    pub fn set_project_view(&mut self, view: Option<crate::project::ProjectView>) {
        self.project_view = view;
    }

    fn current_project_view(&self) -> Option<crate::project::ProjectView> {
        self.project_view.clone().map(|mut view| {
            view.active_setup = self.active_setup_id;
            view.active_origin = self.active_origin;
            view.selection = self.selection;
            view.cad_selection = self.cad_selection.clone();
            view
        })
    }

    fn project_snapshot(&self) -> ProjectSnapshot {
        let mut document = self.document().clone();
        // Allocation bookkeeping must not make undoing back to saved content dirty.
        document.operation_id_floor = 0;
        document.setup_id_floor = 0;
        document.asset_id_floor = 0;
        document.instance_id_floor = 0;
        document.pattern_id_floor = 0;
        ProjectSnapshot {
            document,
            defaults: self.defaults,
            source_name: self.source_name.clone(),
            regions: self.regions.clone(),
            open_paths: self.open_paths.clone(),
        }
    }

    fn mark_project_saved(&mut self) {
        self.saved_project = Some(self.project_snapshot());
    }

    /// Whether user-authored project content differs from its last successful
    /// save/open (or the initial empty project). Undoing to that content is clean.
    pub fn has_unsaved_changes(&self) -> bool {
        self.saved_project.as_ref() != Some(&self.project_snapshot())
    }

    /// Save the current project (document + geometry + defaults) to `path` as JSON,
    /// and remember it as the current path.
    pub fn save_project(&mut self, path: impl AsRef<Path>) -> Result<(), ProjectError> {
        // Keep allocation marks after undo + save + reopen without an undo edit.
        let op_floor = self.operation_id_floor;
        let setup_floor = self.setup_id_floor;
        let asset_floor = self.asset_id_floor;
        let instance_floor = self.instance_id_floor;
        let pattern_floor = self.pattern_id_floor;
        self.document.update_view(|doc| {
            doc.record_ids();
            doc.operation_id_floor = doc.operation_id_floor.max(op_floor);
            doc.setup_id_floor = doc.setup_id_floor.max(setup_floor);
            doc.asset_id_floor = doc.asset_id_floor.max(asset_floor);
            doc.instance_id_floor = doc.instance_id_floor.max(instance_floor);
            doc.pattern_id_floor = doc.pattern_id_floor.max(pattern_floor);
        });
        let project = Project {
            view: self.current_project_view(),
            toolpaths: crate::project::StoredToolpaths {
                cache: self.operation_cache.clone(),
                fingerprints: self.toolpath_fingerprints.clone(),
                verification: self.saved_verification.clone(),
            },
            schema_version: cam_model::SCHEMA_VERSION,
            document: self.document.current().clone(),
            regions: self.regions.clone(),
            open_paths: self.open_paths.clone(),
            defaults: self.defaults,
            source_name: self.source_name.clone(),
            // Always written (v11): a saved job records the machine and control it was
            // built for. Only a *pre*-v11 file can be silent about them.
            machine: Some(self.machine.clone()),
            post: Some(self.post_kind),
        };
        let path = path.as_ref();
        self.document()
            .geometry
            .validate()
            .map_err(ProjectError::Json)?;
        crate::project::atomic_save(path, &project).map_err(ProjectError::Io)?;
        self.current_path = Some(path.to_path_buf());
        self.mark_project_saved();
        Ok(())
    }

    /// Open a `.ocam` project from `path`, replacing the current session.
    pub fn open_project(&mut self, path: impl AsRef<Path>) -> Result<(), ProjectError> {
        let path = path.as_ref();
        let bytes = std::fs::read(path).map_err(|e| ProjectError::Io(e.to_string()))?;
        // Route through the tagged union; a legacy untagged project still parses (§3.1)
        // and is migrated to the current schema on the way through.
        let project = match OcamFile::from_bytes(&bytes).map_err(ProjectError::from)? {
            OcamFile::Project(p) => p,
            OcamFile::Library(_) => return Err(ProjectError::NotAProject),
        };
        let mut project = project;
        crate::cad_operations::mark_missing_bindings(&mut project.document.geometry);
        project
            .document
            .geometry
            .validate()
            .map_err(ProjectError::Json)?;
        // Legacy wrapper geometry becomes undoable document state on first load.
        if project.document.geometry.drawing_regions.is_empty()
            && project.document.geometry.drawing_paths.is_empty()
        {
            project.document.geometry.drawing_regions = project.regions.clone();
            project.document.geometry.drawing_paths = project.open_paths.clone();
        }
        self.cancel_cad_import();
        self.cad_selection.clear();
        self.regions = project.regions;
        self.open_paths = project.open_paths;
        self.defaults = project.defaults;
        self.setup_defaults = default_setup(&self.defaults);
        self.source_name = project.source_name;
        // **Neither is adopted. Both are provenance.** (Andreas, 2026-08-01: "machine +
        // control local, ocam file as provenance".)
        //
        // A machine is shop-local. A project regularly travels between users and shops,
        // and the envelope is what *gates* an export — so taking the machine from the
        // file disarms the one check that stops a job being cut on a machine it does not
        // fit. Concretely: a job authored on a 1000 mm router, emailed to someone with a
        // 300 mm mill, used to replace their machine with the sender's on open, after
        // which `check_travel` compared the program against the *sender's* travel and
        // passed. Silently, and it overwrote their configured machine as a side effect.
        //
        // The post goes the same way for the same reason: which control you are cutting
        // on is a property of your shop, not of the file. Its failure mode is louder —
        // foreign words, an alarming control — but the rule is easier to hold when it is
        // one rule.
        //
        // What the file states is still worth *knowing*, so it is kept and offered to
        // the caller to report; see [`provenance`](Self::provenance).
        self.provenance = Provenance {
            machine: project.machine,
            post: project.post,
        };
        self.operation_cache = project.toolpaths.cache;
        self.toolpath_fingerprints = project.toolpaths.fingerprints;
        self.saved_verification = project.toolpaths.verification;
        self.document = History::new(project.document);
        self.active_setup_id = self.document.current().active_setup().map_or(0, |s| s.id);
        self.active_origin = self.setup().origin_index;
        self.pending_op = None;
        self.current_path = Some(path.to_path_buf());
        self.selection = Selection::Setup;
        self.project_view = project.view;
        if let Some(view) = self.project_view.clone() {
            let _ = self.activate_setup(view.active_setup);
            self.select(view.selection);
            if self.setup().origin_indices().contains(&view.active_origin) {
                self.active_origin = view.active_origin;
            }
            self.cad_selection = view.cad_selection;
        } else {
            self.reconcile_selection();
        }
        self.invalidate();
        self.restore_saved_verification();
        self.mark_project_saved();
        Ok(())
    }

    /// Edit the setup's heights as one undoable change.
    pub fn edit_heights(&mut self, f: impl FnOnce(&mut Heights)) {
        self.edit_setup_document(|doc| f(&mut doc.setup_mut().heights));
        self.invalidate();
    }

    /// Edit setup-level fields beyond [`Heights`] — e.g. the tool-change height.
    pub fn edit_setup(&mut self, f: impl FnOnce(&mut cam_model::Setup)) {
        self.edit_setup_document(|doc| f(doc.setup_mut()));
        self.invalidate();
    }

    /// Edit the workpiece origin (datum) as one undoable change. Re-references the
    /// posted G-code; the design/sim frame is unaffected, so no re-run is needed —
    /// but `invalidate` keeps the pipeline honest.
    pub fn edit_origin(&mut self, f: impl FnOnce(&mut [f64; 3])) {
        self.edit_setup_document(|doc| f(&mut doc.setup_mut().origin));
        self.invalidate();
    }

    /// The origin (`H<n>` index) new operations attach to and the origin inspector
    /// edits.
    pub fn active_origin(&self) -> u32 {
        self.active_origin
    }

    /// The index the **base** origin currently holds (its position lives in
    /// `Setup::origin`). Editable/swappable, so not necessarily 1.
    pub fn base_origin_index(&self) -> u32 {
        self.setup().origin_index
    }

    /// Focus an origin: make it active (new ops and the pick flow target it) and select
    /// its node. Falls back to the base origin if the index is unknown.
    pub fn select_origin(&mut self, index: u32) {
        self.active_origin = if self.setup().origin_indices().contains(&index) {
            index
        } else {
            self.setup().origin_index
        };
        self.selection = Selection::Origin;
    }

    /// Every origin index in this setup, base (1) first — for the tree headers.
    pub fn origin_indices(&self) -> Vec<u32> {
        self.setup().origin_indices()
    }

    /// The part-space position of origin `index` (base or an extra).
    pub fn origin_position(&self, index: u32) -> [f64; 3] {
        self.setup().origin_position(index)
    }

    /// Edit the **active** origin's position as one undoable change — the base
    /// [`Setup::origin`] when it holds the active index, otherwise the matching
    /// `extra_origins` entry.
    pub fn edit_active_origin(&mut self, f: impl FnOnce(&mut [f64; 3])) {
        let index = self.active_origin;
        self.edit_setup_document(|doc| {
            if doc.setup_mut().origin_index == index {
                f(&mut doc.setup_mut().origin);
            } else if let Some(o) = doc
                .setup_mut()
                .extra_origins
                .iter_mut()
                .find(|o| o.index == index)
            {
                f(&mut o.position);
            }
        });
        self.invalidate();
    }

    /// Add a new workpiece origin (a reorientation of the part), indexed one past the
    /// highest, seeded at the base origin's position, and make it active + selected.
    pub fn add_origin(&mut self) {
        let index = self.setup().next_origin_index();
        let position = self.setup().origin;
        self.edit_setup_document(move |doc| {
            doc.setup_mut()
                .extra_origins
                .push(Origin { index, position })
        });
        self.active_origin = index;
        self.selection = Selection::Origin;
        self.invalidate();
    }

    /// Delete the origin with `index` (the base origin — the one holding
    /// `origin_index` — is never removed). Its operations are reassigned to the base,
    /// and it drops from the active / disabled sets.
    pub fn delete_origin(&mut self, index: u32) {
        if self.setups().is_empty() {
            return;
        }
        let base = self.setup().origin_index;
        if index == base {
            return;
        }
        self.edit_document(move |doc| {
            doc.setup_mut().extra_origins.retain(|o| o.index != index);
            if let Some(disabled) = doc.disabled_origins.get_mut(&doc.active_setup_id) {
                disabled.remove(&index);
            }
            for op in &mut doc.setup_mut().operations {
                if op.work_offset() == index {
                    op.set_work_offset(base);
                }
            }
        });
        if self.active_origin == index {
            self.active_origin = base;
        }
        self.invalidate();
    }

    /// Renumber the active origin's `H<n>` to `new_index` (≥ 1). If another origin
    /// already uses `new_index`, the two **swap** indices (tool-library style) rather
    /// than colliding; operations are remapped to follow their origin. A no-op if
    /// unchanged or `new_index` is 0.
    pub fn set_active_origin_index(&mut self, new_index: u32) {
        let old = self.active_origin;
        if new_index == old || new_index == 0 {
            return;
        }
        let swap = self.setup().origin_indices().contains(&new_index);
        self.edit_setup_document(move |doc| {
            let s = doc.setup_mut();
            if swap {
                // Park the other origin at a temporary index so the two reindexes
                // don't collide, then move old→new and parked→old.
                set_origin_index_value(s, new_index, u32::MAX);
                set_origin_index_value(s, old, new_index);
                set_origin_index_value(s, u32::MAX, old);
            } else {
                set_origin_index_value(s, old, new_index);
            }
            for op in &mut s.operations {
                let wo = op.work_offset();
                if wo == old {
                    op.set_work_offset(new_index);
                } else if swap && wo == new_index {
                    op.set_work_offset(old);
                }
            }
            // Suppression follows the renamed datum in this same command.
            let disabled = doc.disabled_origins.entry(doc.active_setup_id).or_default();
            let was_old = disabled.remove(&old);
            let was_new = swap && disabled.remove(&new_index);
            if was_old {
                disabled.insert(new_index);
            }
            if was_new {
                disabled.insert(old);
            }
        });
        self.active_origin = new_index;
        self.invalidate();
    }

    /// Reassign operation `id` to origin `index` (its group), as one undoable change.
    pub fn set_operation_origin(&mut self, id: u32, index: u32) {
        if self
            .setup()
            .patterns
            .iter()
            .any(|pattern| pattern.members.contains(&id))
        {
            return;
        }
        self.edit_document(move |doc| {
            if let Some(op) = doc.setup_mut().operations.iter_mut().find(|o| o.id() == id) {
                op.set_work_offset(index);
            }
        });
        self.invalidate();
    }

    /// Whether origin `index`'s operations are frozen out of the run.
    pub fn is_origin_disabled(&self, index: u32) -> bool {
        self.document()
            .disabled_origins
            .get(&self.active_setup_id)
            .is_some_and(|set| set.contains(&index))
    }

    /// Freeze/unfreeze an origin's operations (kept in the tree, dropped from the run —
    /// so its toolpaths leave the viewport). The base can be frozen like any other.
    pub fn set_origin_disabled(&mut self, index: u32, disabled: bool) {
        if !self.setup().origin_indices().contains(&index)
            || self.is_origin_disabled(index) == disabled
        {
            return;
        }
        let setup = self.active_setup_id;
        self.edit_document(|doc| {
            let set = doc.disabled_origins.entry(setup).or_default();
            if disabled {
                set.insert(index);
            } else {
                set.remove(&index);
            }
        });
        self.invalidate();
    }

    /// Reconcile the just-loaded setup's tool numbers against the shop `shop` library
    /// (`TOOLING_PLAN.md` §6, Phase 2). Matched tools adopt the shop's number and every
    /// operation reference is rewritten; unmatched tools stay project-local. This is
    /// **load-time normalisation**, so on any change the undo baseline is reset to the
    /// reconciled state (there is no "undo the renumber" — the project simply opened
    /// this way). Returns the report so the shell can surface a summary.
    pub fn reconcile_tools(&mut self, shop: &[Tool]) -> ReconcileReport {
        if self.setups().is_empty() {
            return ReconcileReport::default();
        }
        let mut doc = self.document.current().clone();
        let report = reconcile_tool_numbers(doc.setup_mut(), shop);
        if report.changed() {
            self.document = History::new(doc);
            self.reconcile_selection();
            self.invalidate();
        }
        report
    }

    /// Edit a tool as one undoable change. A no-op if the index is out of range.
    pub fn edit_tool(&mut self, index: usize, f: impl FnOnce(&mut Tool)) {
        let in_range = index < self.setup().tools.len();
        if !in_range {
            return;
        }
        self.edit_setup_document(|doc| f(&mut doc.setup_mut().tools[index]));
        self.invalidate();
    }

    /// Append a fresh default tool (numbered one past the highest existing tool)
    /// and select it, as one undoable change.
    pub fn add_tool(&mut self) {
        let number = self
            .setup()
            .tools
            .iter()
            .map(|t| t.number)
            .max()
            .map_or(1, |m| m + 1);
        let p = self.defaults;
        let tool = Tool {
            number,
            diameter: p.tool_diameter,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        };
        self.edit_setup_document(move |doc| doc.setup_mut().tools.push(tool));
        let index = self.setup().tools.len() - 1;
        self.selection = Selection::Tool(index);
        self.invalidate();
    }

    /// Delete the tool at `index`, selecting a neighbour (or the setup if none is
    /// left). A no-op if the index is out of range. Operations that referenced the
    /// tool's number are left as-is — they surface a "references tool N which is
    /// not in the setup" diagnostic on the next run rather than being rewritten.
    pub fn delete_tool(&mut self, index: usize) {
        if self.setups().is_empty() {
            return;
        }
        let count = self.setup().tools.len();
        if index >= count {
            return;
        }
        self.edit_document(move |doc| {
            doc.setup_mut().tools.remove(index);
        });
        self.selection = match count - 1 {
            0 => Selection::Setup,
            remaining => Selection::Tool(index.min(remaining - 1)),
        };
        self.invalidate();
    }

    /// Embed a tool (chosen from the cross-project library) into this project's
    /// setup and return the project-local number to store on an operation. If an
    /// identical tool (same geometry, ignoring number) is already embedded, its
    /// number is reused; otherwise a copy is added with a fresh project-unique
    /// number. One undoable change when it adds.
    pub fn use_tool(&mut self, tool: Tool) -> u32 {
        if let Some(existing) = self
            .setup()
            .tools
            .iter()
            .find(|t| same_tool_geometry(t, &tool))
        {
            return existing.number;
        }
        let number = self
            .setup()
            .tools
            .iter()
            .map(|t| t.number)
            .max()
            .map_or(1, |m| m + 1);
        let embedded = Tool { number, ..tool };
        self.edit_setup_document(move |doc| doc.setup_mut().tools.push(embedded));
        self.invalidate();
        number
    }

    /// The tools actually referenced by at least one operation, in setup order.
    /// This is what the read-only Project→Tools view shows.
    pub fn used_tools(&self) -> Vec<&Tool> {
        let setup = self.setup();
        // Every tool an operation uses, not just its defining one — a carve's clearing
        // end mill is on the setup sheet and in the changer, so it belongs here.
        let used: BTreeSet<u32> = setup.operations.iter().flat_map(Operation::tools).collect();
        setup
            .tools
            .iter()
            .filter(|t| used.contains(&t.number))
            .collect()
    }

    /// Drop embedded tools no longer referenced by any operation (keeps the setup —
    /// and the saved `.ocam` — to just the tools in use). One undoable change if it
    /// removes anything.
    pub fn prune_unused_tools(&mut self) {
        if self.setups().is_empty() {
            return;
        }
        // Must count EVERY tool an operation references: pruning on the defining tool
        // alone would drop a carve's clearing end mill out of the setup, leaving the
        // operation pointing at a tool that is no longer there.
        let used: BTreeSet<u32> = self
            .setup()
            .operations
            .iter()
            .flat_map(Operation::tools)
            .collect();
        let has_unused = self.setup().tools.iter().any(|t| !used.contains(&t.number));
        if !has_unused {
            return;
        }
        self.edit_document(move |doc| doc.setup_mut().tools.retain(|t| used.contains(&t.number)));
        self.invalidate();
    }

    /// Edit the selected operation as one undoable change. A no-op unless an
    /// operation is selected.
    /// Edit operation `id` in place (undoable). Used where the target is explicit
    /// rather than whatever is selected — e.g. changing an operation's tool from the
    /// inspector.
    pub fn rename_operation(&mut self, id: u32, name: String) {
        if !self
            .document()
            .setups
            .iter()
            .any(|s| s.operations.iter().any(|op| op.id() == id))
        {
            return;
        }
        let name = name.trim().to_owned();
        if self.document().operation_names.get(&id).map(String::as_str) == Some(name.as_str())
            || (name.is_empty() && !self.document().operation_names.contains_key(&id))
        {
            return;
        }
        self.edit_document(move |doc| {
            if name.is_empty() {
                doc.operation_names.remove(&id);
            } else {
                doc.operation_names.insert(id, name);
            }
        });
    }

    pub fn edit_operation(&mut self, id: u32, f: impl FnOnce(&mut Operation)) {
        if self.operation(id).is_none() {
            return;
        }
        let mut edited = false;
        self.edit_document(|doc| {
            let heights = doc.setup().heights;
            let mut binding = doc.geometry.bindings.remove(&id);
            if let Some(op) = doc.setup_mut().operations.iter_mut().find(|o| o.id() == id) {
                let before = op.legacy_cutting_heights(&heights);
                f(op);
                crate::cad_operations::capture_height_override(
                    binding.as_mut(),
                    before,
                    op,
                    &heights,
                );
                edited = true;
            }
            if let Some(binding) = binding {
                doc.geometry.bindings.insert(id, binding);
            }
        });
        if edited {
            self.invalidate();
        }
    }

    pub fn edit_selected_operation(&mut self, f: impl FnOnce(&mut Operation)) {
        let Selection::Operation(id) = self.selection else {
            return;
        };
        self.edit_operation(id, f);
    }

    /// The next free operation id (max existing + 1).
    fn next_op_id(&self) -> u32 {
        self.document
            .current()
            .setups
            .iter()
            .flat_map(|setup| &setup.operations)
            .map(|o| o.id())
            .max()
            .map_or(0, |m| m.saturating_add(1))
            .max(self.operation_id_floor)
            .max(self.document().operation_id_floor)
    }

    /// Append `op` (renumbered with a fresh id) to the setup and select it, as one
    /// undoable change.
    pub fn add_operation(&mut self, mut op: Operation) {
        let id = self.next_op_id();
        set_op_id(&mut op, id);
        // A new operation joins the active origin's group (H<n>).
        op.set_work_offset(self.active_origin);
        self.edit_setup_document(move |doc| {
            doc.setup_mut()
                .height_settings
                .insert(id, cam_model::OperationHeightSettings::default());
            doc.setup_mut().operations.push(op);
        });
        self.selection = Selection::Operation(id);
        self.invalidate();
    }

    /// Replace operation `replaced` with `op`, keeping the original's **position** in
    /// the order and its id, so a reinitialised operation stays where it was in the
    /// job. Falls back to appending if the original has gone.
    fn replace_operation(&mut self, replaced: u32, mut op: Operation) {
        let exists = self.operation(replaced).is_some();
        if !exists {
            self.add_operation(op);
            return;
        }
        set_op_id(&mut op, replaced);
        self.edit_setup_document(move |doc| {
            if let Some(slot) = doc
                .setup_mut()
                .operations
                .iter_mut()
                .find(|o| o.id() == replaced)
            {
                *slot = op;
            }
        });
        self.selection = Selection::Operation(replaced);
        self.invalidate();
    }

    /// Duplicate the selected operation, inserting the copy right after it and
    /// selecting it. A no-op unless an operation is selected.
    pub fn duplicate_selected_operation(&mut self) {
        let Selection::Operation(sel) = self.selection else {
            return;
        };
        let Some(mut copy) = self.operation(sel).cloned() else {
            return;
        };
        let insert_after = self
            .setup()
            .patterns
            .iter()
            .find(|p| p.members.contains(&sel))
            .and_then(|p| p.members.last())
            .and_then(|last| {
                self.setup()
                    .operations
                    .iter()
                    .position(|op| op.id() == *last)
            })
            .or_else(|| self.setup().operations.iter().position(|op| op.id() == sel))
            .map(|index| index + 1)
            .unwrap_or(self.setup().operations.len());
        let new_id = self.next_op_id();
        set_op_id(&mut copy, new_id);
        self.edit_document(move |doc| {
            if let Some(name) = doc.operation_names.get(&sel).cloned() {
                doc.operation_names.insert(new_id, name);
            }
            if let Some(index) = doc.rotary_indexes.get(&sel).cloned() {
                doc.rotary_indexes.insert(new_id, index);
            }
            if let Some(binding) = doc.geometry.bindings.get(&sel).cloned() {
                doc.geometry.bindings.insert(new_id, binding);
            }
            if doc.cad_linked_operations.contains(&sel) {
                doc.cad_linked_operations.insert(new_id);
            }
            let stock = doc.setup().stock_to_leave_for(sel);
            if stock.enabled {
                doc.setup_mut().stock_to_leave.insert(new_id, stock);
            }
            doc.setup_mut().operations.insert(insert_after, copy);
        });
        self.selection = Selection::Operation(new_id);
        self.invalidate();
    }

    /// Create a new default operation of `kind` from the **first** imported region
    /// with the first tool, and select it. A convenience over the pick wizard,
    /// kept for headless use/tests. A no-op if no geometry is loaded.
    pub fn new_operation(&mut self, kind: OpKind) {
        if !self.has_geometry() {
            return;
        }
        let tool = self.first_tool_number();
        // Prefer the first closed region; with a strokes-only drawing fall back to the
        // first open path (which only an engraving operation will accept).
        let boundary = if self.regions.is_empty() {
            LoopRef::open(0)
        } else {
            LoopRef {
                region: 0,
                part: LoopPart::Outer,
            }
        };
        if let Some(op) = self.build_op(kind, boundary, &[], tool, None, None) {
            self.add_operation(op);
        }
    }

    /// The cutting data (spindle speed + feeds) an operation starts life with: the
    /// chosen tool's nominals, falling back to the job defaults where the tool leaves
    /// one unset (0). The wizard shows these as editable defaults; the value that is
    /// finally committed comes back in via [`Self::confirm_operation`].
    pub fn seeded_cutting_for(&self, tool: u32) -> CuttingData {
        let p = self.defaults;
        let td = self.setup().tools.iter().find(|t| t.number == tool);
        if let Some(preset) = td.and_then(|t| t.preset_for_material(self.setup().material_id)) {
            return CuttingData {
                rpm: preset.spindle_rpm,
                feed: preset.feed,
                plunge_feed: preset.plunge_feed,
            };
        }
        CuttingData {
            rpm: td.map_or(0.0, |t| t.nominal_rpm),
            feed: td
                .map(|t| t.nominal_feed)
                .filter(|f| *f > 0.0)
                .unwrap_or(p.feed),
            plunge_feed: td
                .map(|t| t.nominal_plunge_feed)
                .filter(|f| *f > 0.0)
                .unwrap_or(p.plunge_feed),
        }
    }

    /// Build a default operation of `kind` on the picked `boundary` loop with tool
    /// `tool`, treating each of `islands` as an excluded loop (pocket only). Returns
    /// `None` if a loop reference does not resolve. A sane starting point the user
    /// then edits in the inspector:
    /// - **Profile** — profile the picked loop; `side` defaults Outside (flip in the
    ///   inspector), `start` at the picked vertex.
    /// - **Pocket** — clear inside the picked loop, leaving the chosen islands.
    /// - **Drill** — one hole at the picked loop's centroid.
    /// - **Face** — face inside the picked loop.
    fn build_op(
        &self,
        kind: OpKind,
        boundary: LoopRef,
        islands: &[LoopRef],
        tool: u32,
        start: Option<[f64; 2]>,
        cutting: Option<CuttingData>,
    ) -> Option<Operation> {
        let selected: Vec<_> = std::iter::once(boundary)
            .chain(islands.iter().copied())
            .collect();
        if !islands.is_empty() {
            self.planar_operation_eligibility(kind, &selected).ok()?;
        }
        // Area strategies still require a closed region.
        if boundary.is_open() && !op_accepts_open_paths(kind) {
            return None;
        }
        let (points, closed) = self.loop_points(boundary)?;
        let chain = Contour::new(points);
        self.build_op_shape(
            kind,
            chain,
            closed,
            islands,
            tool,
            start,
            cutting,
            self.defaults,
        )
    }

    /// An uncommitted parameter template; its placeholder contour is always
    /// replaced by validated user geometry before publishing an operation.
    pub fn operation_template(&self, kind: OpKind, tool: u32) -> Operation {
        self.operation_template_with_defaults(kind, tool, self.defaults, None)
    }

    /// The programmed operation template before project and cutter overrides.
    #[cfg(feature = "gui")]
    pub(crate) fn system_operation_template(&self, kind: OpKind) -> Operation {
        let defaults = JobParams::default();
        let cutting = CuttingData {
            rpm: defaults.spindle_rpm,
            feed: defaults.feed,
            plunge_feed: defaults.plunge_feed,
        };
        self.operation_template_with_defaults(kind, 0, defaults, Some(cutting))
    }

    fn operation_template_with_defaults(
        &self,
        kind: OpKind,
        tool: u32,
        defaults: JobParams,
        cutting: Option<CuttingData>,
    ) -> Operation {
        let chain = Contour::new(vec![
            Point::new(0., 0.),
            Point::new(10., 0.),
            Point::new(10., 10.),
            Point::new(0., 10.),
        ]);
        let chain = if kind == OpKind::Bore {
            Contour::new(
                (0..64)
                    .map(|i| {
                        let a = std::f64::consts::TAU * i as f64 / 64.;
                        Point::new(6. * a.cos(), 6. * a.sin())
                    })
                    .collect(),
            )
        } else {
            chain
        };
        self.build_op_shape(kind, chain, true, &[], tool, None, cutting, defaults)
            .unwrap()
    }

    #[allow(clippy::too_many_arguments)]
    fn build_op_shape(
        &self,
        kind: OpKind,
        chain: Contour,
        closed: bool,
        islands: &[LoopRef],
        tool: u32,
        start: Option<[f64; 2]>,
        cutting: Option<CuttingData>,
        defaults: JobParams,
    ) -> Option<Operation> {
        // Cutting data: the values chosen in the wizard if given, else the tool's
        // seeded nominals. A `spindle_rpm` of 0 falls back to the job default at plan
        // time; the inspector can override any of these per operation afterwards.
        let CuttingData {
            rpm: spindle_rpm,
            feed,
            plunge_feed,
        } = cutting.unwrap_or_else(|| self.seeded_cutting_for(tool));
        // id 0 is a placeholder — add_operation renumbers with a fresh id.
        let mut op = match kind {
            OpKind::Profile => Operation::Profile(ProfileOp {
                parameter_expressions: Default::default(),
                wrap: None,
                passes: cam_model::ProfilePasses {
                    lead_end_distance: 0.6,
                    lead_end_distance_enabled: Some(false),
                    ..Default::default()
                },
                selections: None,
                closed,
                additional_chains: islands
                    .iter()
                    .map(|r| {
                        let (points, closed) = self.loop_points(*r)?;
                        Some(cam_model::ProfileContour {
                            chain: Contour::new(points),
                            closed,
                            side: if matches!(r.part, crate::LoopPart::Hole(_)) {
                                Side::Inside
                            } else {
                                Side::Outside
                            },
                        })
                    })
                    .collect::<Option<Vec<_>>>()?,
                tabs: Default::default(),
                clearing: cam_model::Clearing::default(),
                id: 0,
                tool,
                chain,
                side: Side::Outside,
                comp: Comp::Computed,
                offset: 0.0,
                depth: defaults.depth,
                stepdown: defaults.stepdown,
                stepover: 0.0,
                spindle_rpm,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                feed,
                plunge_feed,
                start,
                end: None,
                lead_in: Lead::Arc { radius: 0.6 },
                lead_out: Lead::Arc { radius: 0.6 },
                lead_overlap: if closed { 0.6 } else { 0.0 },
                plunge: Plunge::Straight,
            }),
            OpKind::Pocket => {
                let island_contours = islands
                    .iter()
                    .map(|l| self.loop_contour(*l).cloned())
                    .collect::<Option<Vec<_>>>()?;
                Operation::Pocket(PocketOp {
                    maximum_stepover: None,
                    parameter_expressions: Default::default(),
                    wrap: None,
                    additional_regions: Vec::new(),
                    stock_contours: Default::default(),
                    finishing: Default::default(),
                    id: 0,
                    tool,
                    boundary: chain,
                    islands: island_contours,
                    depth: defaults.depth,
                    spindle_rpm,
                    tool_preset: None,
                    cooling: Default::default(),
                    entry_feeds: Default::default(),
                    work_offset: 1,
                    start,
                    end: None,
                    clear: ClearParams {
                        multiple_depths: Default::default(),
                        adaptive_enabled: None,
                        pocket_overlap: None,
                        keep_tool_down: false,
                        lift_height: 0.2,
                        clearing: cam_model::Clearing::default(),
                        stepdown: defaults.stepdown,
                        overlap: 0.5,
                        offset: 0.0,
                        feed,
                        plunge_feed,
                        plunge: Plunge::Straight,
                        lead_overlap: 0.0,
                        lead_in: Lead::Arc { radius: 0.6 },
                        lead_out: Lead::Arc { radius: 0.6 },
                    },
                })
            }
            OpKind::Drill => Operation::Drill(DrillOp {
                parameter_expressions: Default::default(),
                id: 0,
                tool,
                points: vec![centroid(&chain)],
                depth: defaults.depth,
                start_offset: 0.0,
                peck: None,
                dwell: None,
                spindle_rpm,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                feed: plunge_feed,
            }),
            OpKind::Face => {
                // Face along the edge the user clicked to pick the boundary (the
                // snapped pick point rides on it); with no pick, fall back to the
                // longest edge. The inspector picker still overrides either way.
                let direction = match start {
                    Some([x, y]) => Axis::along_edge_at(chain.points(), cam_geo::Point::new(x, y)),
                    None => Axis::along_longest_edge(chain.points()),
                };
                Operation::Face(FaceOp {
                    absolute_overlap: None,
                    multiple_depths: Default::default(),
                    parameter_expressions: Default::default(),
                    id: 0,
                    tool,
                    boundary: chain,
                    // Face at the reference plane by default; a shallow skim the user
                    // then tunes (start_offset raises it to level proud stock).
                    start_offset: 0.0,
                    depth: 1.0,
                    stepdown: defaults.stepdown,
                    overlap: 0.5,
                    // `overshoot` is the cutter-edge clearance past the stock edge; the
                    // strategy handles the radius and stock geometry, so this is just a
                    // 5 mm comfort margin. Negative would plunge into the stock (warned).
                    overshoot: 5.0,
                    direction,
                    spindle_rpm,
                    tool_preset: None,
                    cooling: Default::default(),
                    entry_feeds: Default::default(),
                    work_offset: 1,
                    feed,
                    plunge_feed,
                })
            }
            OpKind::Chamfer => Operation::Chamfer(ChamferOp {
                parameter_expressions: Default::default(),
                contour_roughing: Default::default(),
                tip_offset: 1.0,
                lead_in_enabled: Some(true),
                same_lead_out: false,
                lead_out_enabled: Some(true),
                closed,
                faces: Vec::new(),
                stepdown: 0.,
                id: 0,
                tool,
                chain,
                side: Side::Outside,
                width: 1.0,
                top: defaults.top_of_stock,
                // 0 depth ⇒ use the tool tip at the bevel bottom; 0 step ⇒ one pass.
                depth: 0.0,
                step: 0.0,
                gradual: false,
                spindle_rpm,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                feed,
                plunge_feed,
                start,
                end: None,
                lead_in: Lead::Arc { radius: 0.6 },
                lead_out: Lead::Arc { radius: 0.6 },
                lead_overlap: if closed { 0.6 } else { 0.0 },
            }),
            OpKind::Bore => {
                let (center, radius) = fit_circle(chain.points())?;
                Operation::Bore(cam_model::BoreOp {
                    tool,
                    points: vec![center],
                    diameter: 2. * radius,
                    z_top: defaults.top_of_stock,
                    z_bottom: defaults.top_of_stock - defaults.depth,
                    spindle_rpm,
                    feed,
                    plunge_feed,
                    ..Default::default()
                })
            }
            OpKind::Thread => {
                // Seed one thread at the picked loop's centre; a circular hole
                // loop gives its diameter as the major diameter. Pitch/hand are
                // refined in the inspector.
                let pts = chain.points();
                let (mut xmin, mut xmax) = (f64::MAX, f64::MIN);
                let (mut ymin, mut ymax) = (f64::MAX, f64::MIN);
                for pt in pts {
                    xmin = xmin.min(pt.x);
                    xmax = xmax.max(pt.x);
                    ymin = ymin.min(pt.y);
                    ymax = ymax.max(pt.y);
                }
                let dia = (xmax - xmin).max(ymax - ymin);
                Operation::Thread(ThreadOp {
                    parameter_expressions: Default::default(),
                    id: 0,
                    tool,
                    points: vec![centroid(&chain)],
                    internal: true,
                    hand: Hand::Right,
                    major_dia: if dia.is_finite() && dia > 0.0 {
                        dia
                    } else {
                        6.0
                    },
                    pitch: 1.0,
                    // Thread span is absolute Z; `depth` is now a positive magnitude
                    // below the top, so the bottom sits that far under it.
                    z_top: defaults.top_of_stock,
                    z_bottom: defaults.top_of_stock - defaults.depth,
                    climb: true,
                    passes: 1,
                    spring_passes: 0,
                    gradual: false,
                    drill_clearance: 0.0,
                    blind_allowance: 0.0,
                    spindle_rpm,
                    tool_preset: None,
                    cooling: Default::default(),
                    entry_feeds: Default::default(),
                    work_offset: 1,
                    feed,
                    plunge_feed,
                })
            }
            OpKind::Carve => {
                // Islands are carved around, exactly as a pocket leaves them — the
                // counters of letters are the everyday case.
                let island_contours = islands
                    .iter()
                    .map(|l| self.loop_contour(*l).cloned())
                    .collect::<Option<Vec<_>>>()?;
                Operation::Carve(CarveOp {
                    parameter_expressions: Default::default(),
                    id: 0,
                    tool,
                    // No clearing tool by default: the V-bit alone is the simple case,
                    // and the inspector offers the second tool only once the shape is
                    // known to leave a flat land.
                    clear: None,
                    boundary: chain,
                    islands: island_contours,
                    top: defaults.top_of_stock,
                    // A cap, not a command: the shape decides the actual depth. Deeper
                    // than an engraving default, since a carve is meant to be seen.
                    depth: 1.0,
                    offset: 0.0,
                    ring_step: 0.0,
                    scallop: 0.0,
                    spindle_rpm,
                    tool_preset: None,
                    cooling: Default::default(),
                    entry_feeds: Default::default(),
                    work_offset: 1,
                    feed,
                    plunge_feed,
                    // Straight by default, as every carve was before the style existed: a
                    // V-bit has a cutting tip, so it is the one entry that is always
                    // legal. Ramping is the operator's choice, not ours to assume.
                    plunge: Plunge::Straight,
                    // On by default: a carve is hundreds of rings, and every link is
                    // verified before it is taken (the ones that would gouge still lift).
                    stay_down: true,
                    start,
                    end: None,
                })
            }
            OpKind::Engrave => Operation::Engrave(EngraveOp {
                parameter_expressions: Default::default(),
                id: 0,
                tool,
                chain,
                // Closed for a picked region loop; open for an imported stroke.
                closed,
                top: defaults.top_of_stock,
                // A shallow default: engraving is a surface mark, not a cut. One
                // pass (stepdown 0) — normal at this depth.
                depth: 0.3,
                stepdown: 0.0,
                spindle_rpm,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                feed,
                plunge_feed,
                start,
                end: None,
            }),
            OpKind::Adaptive => Operation::Adaptive(AdaptiveOp {
                parameter_expressions: Default::default(),
                id: 0,
                tool,
                spindle_rpm,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                params: AdaptiveParams::default(),
            }),
            OpKind::Parallel => Operation::Parallel(ParallelOp {
                parameter_expressions: Default::default(),
                id: 0,
                tool,
                spindle_rpm,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                params: ParallelParams::default(),
            }),
            OpKind::Spiral => Operation::Spiral(cam_model::SpiralOp {
                parameter_expressions: Default::default(),
                id: 0,
                tool,
                spindle_rpm,
                tool_preset: None,
                cooling: Default::default(),
                entry_feeds: Default::default(),
                work_offset: 1,
                params: cam_model::SpiralParams::default(),
            }),
        };
        if let Some(cooling) = self
            .setup()
            .tools
            .iter()
            .find(|t| t.number == tool)
            .map(|t| t.cooling)
        {
            op.set_cooling(cooling);
        }
        if cutting.is_none() {
            if let Some(preset) = self
                .setup()
                .tools
                .iter()
                .find(|t| t.number == tool)
                .and_then(|t| t.preset_for_material(self.setup().material_id))
            {
                op.apply_tool_preset(preset).ok()?;
            }
        }
        Some(op)
    }

    /// The lowest tool number in the setup (the default for a new operation), or 1.
    fn first_tool_number(&self) -> u32 {
        self.setup()
            .tools
            .iter()
            .map(|t| t.number)
            .min()
            .unwrap_or(1)
    }

    // --- Operation-creation wizard (pick a tool, then geometry) ----------------

    /// The operation currently being created, if the wizard is active.
    pub fn pending_op(&self) -> Option<PendingOp> {
        self.pending_op.clone()
    }

    /// The contour of a loop reference, if it resolves.
    /// The points of a picked loop **or open path**, with whether they form a closed
    /// loop. Open paths are the only case where `false` comes back, and only
    /// engraving may consume one.
    pub fn loop_points(&self, l: LoopRef) -> Option<(Vec<Point>, bool)> {
        if l.is_open() {
            let path = self.open_paths.get(l.region)?;
            return Some((path.points().to_vec(), false));
        }
        Some((self.loop_contour(l)?.points().to_vec(), true))
    }

    pub fn loop_contour(&self, l: LoopRef) -> Option<&Contour> {
        let region = self.regions.get(l.region)?;
        match l.part {
            LoopPart::Outer => Some(region.outer()),
            LoopPart::Hole(i) => region.holes().get(i),
            // An open path is not a closed contour; callers that can machine one go
            // through `loop_points`, which reports the open-ness.
            LoopPart::Open => None,
        }
    }

    /// Begin creating an operation of `kind`: enter geometry-pick mode. A no-op if no
    /// geometry is loaded. The tool need not be embedded yet — it is picked from the
    /// library during setup (the GUI seeds a default and calls [`Self::use_tool`]);
    /// `tool` starts at the first embedded tool's number, or 1 as a placeholder.
    pub fn begin_operation(&mut self, kind: OpKind) {
        if !self.has_geometry() {
            return;
        }
        self.pending_op = Some(PendingOp {
            kind,
            tool: None,
            boundary: None,
            islands: Vec::new(),
            additional_holes: Vec::new(),
            start: None,
            replacing: None,
        });
    }

    /// Restart the creation wizard for an existing operation, **replacing it in
    /// place**: the same kind, a fresh tool and geometry pick, and on completion the
    /// result keeps the original's position in the operation order.
    ///
    /// The wizard chooses the tool *before* the geometry, so without this a tool
    /// picked in error could only be undone by deleting the operation and re-picking
    /// its contour — losing its place in the job. (To change only the tool, edit it
    /// on the operation directly; this is for redoing the pick as a whole.)
    pub fn reinitialize_operation(&mut self, id: u32) -> bool {
        if self.regions.is_empty() && self.open_paths.is_empty() {
            return false;
        }
        let Some(kind) = self.operation(id).map(op_kind_of) else {
            return false;
        };
        self.pending_op = Some(PendingOp {
            kind,
            tool: None,
            boundary: None,
            islands: Vec::new(),
            additional_holes: Vec::new(),
            start: None,
            replacing: Some(id),
        });
        self.selection = Selection::Operation(id);
        true
    }

    /// Forget the pending operation's tool — used when the wizard's family changes,
    /// since the chosen tool belonged to the previous family.
    pub fn clear_pending_tool(&mut self) {
        if let Some(pending) = self.pending_op.as_mut() {
            pending.tool = None;
        }
    }

    /// Change the tool of the pending operation. A no-op unless the wizard is active.
    pub fn set_pending_tool(&mut self, number: u32) {
        if let Some(pending) = self.pending_op.as_mut() {
            pending.tool = Some(number);
        }
    }

    /// Cancel the pending operation without creating anything.
    pub fn cancel_operation(&mut self) {
        self.pending_op = None;
    }

    /// Cancel a nested geometry edit without losing the outer wizard's selections.
    #[cfg(feature = "gui")]
    pub(crate) fn restore_pending_operation(&mut self, pending: Option<PendingOp>) {
        self.pending_op = pending;
    }

    /// Finalise the pending operation — the **single commit point** for every kind.
    /// Requires both a chosen tool and a picked boundary; [`Self::pending_ready`]
    /// reports whether those hold, so the UI can gate its Confirm button on the same
    /// condition. Returns `true` if an operation was created.
    pub fn confirm_operation(&mut self, cutting: Option<CuttingData>) -> bool {
        if !self.pending_ready() {
            return false;
        }
        let Some(pending) = self.pending_op.clone() else {
            return false;
        };
        let (Some(boundary), Some(tool)) = (pending.boundary, pending.tool) else {
            return false;
        };
        if let Some(mut op) = self.build_op(
            pending.kind,
            boundary,
            &pending.islands,
            tool,
            pending.start,
            cutting,
        ) {
            if !pending.additional_holes.is_empty() {
                let Some(points) = std::iter::once(boundary)
                    .chain(pending.additional_holes.iter().copied())
                    .map(|reference| self.loop_contour(reference).map(centroid))
                    .collect::<Option<Vec<_>>>()
                else {
                    return false;
                };
                match &mut op {
                    Operation::Drill(operation) => operation.points = points,
                    Operation::Bore(operation) => operation.points = points,
                    Operation::Thread(operation) => operation.points = points,
                    _ => return false,
                }
            }
            match pending.replacing {
                Some(old) => self.replace_operation(old, op),
                None => self.add_operation(op),
            }
            self.pending_op = None;
            // Switching tools mid-wizard embeds each one; drop any left unreferenced.
            self.prune_unused_tools();
            return true;
        }
        false
    }

    /// Whether the pending operation has everything it needs to be confirmed: a tool
    /// **and** a boundary. Either may be chosen first.
    pub fn pending_ready(&self) -> bool {
        self.pending_op.as_ref().is_some_and(|p| {
            let Some(boundary) = p.boundary else {
                return false;
            };
            let additional = if op_selects_circles(p.kind) {
                &p.additional_holes
            } else {
                &p.islands
            };
            let selected: Vec<_> = std::iter::once(boundary)
                .chain(additional.iter().copied())
                .collect();
            p.tool.is_some() && self.planar_operation_eligibility(p.kind, &selected).is_ok()
        })
    }

    /// Run synchronously for CLI/tests, through the same services as GUI workers.
    pub fn run(&mut self, cancel: &CancelToken) -> &RunOutcome {
        self.cancel_run();
        self.automatic_toolpath_run = false;
        let inputs = self.job_inputs();
        let mut cache = std::mem::take(&mut self.operation_cache);
        let result = inputs.execute(
            cancel,
            &mut cache,
            &crate::jobs::Progress::default(),
            &mut Default::default(),
        );
        self.operation_cache = cache;
        self.outcome = Some(result);
        self.record_toolpath_fingerprints();
        self.remember_full_verification(None);
        self.nc = None;
        self.outcome.as_ref().unwrap()
    }
    /// Build a document from the current geometry and seed defaults. When
    /// `seed_ops` is set, seed an outside profile for each region's boundary and
    /// an inside profile for each hole (the bundled sample demo); otherwise the
    /// document carries the geometry, stock, and default tool but **no operations**
    /// (a real import — the user creates ops by picking).
    fn generate_document(&self, seed_ops: bool) -> Document {
        let p = self.defaults;
        // Real imports start with no tools — the user picks them from the library
        // during op setup (`use_tool` embeds them). The Sample seeds one tool so its
        // pre-made profiles still cut.
        let tools = if seed_ops {
            vec![Tool {
                number: 1,
                diameter: p.tool_diameter,
                length: 30.0,
                flutes: 2,
                kind: ToolKind::EndMill,
                ..Default::default()
            }]
        } else {
            Vec::new()
        };

        let mut operations = Vec::new();
        if seed_ops {
            let mut id = 0u32;
            let outer_profile = |id, chain| {
                Operation::Profile(ProfileOp {
                    parameter_expressions: Default::default(),
                    wrap: None,
                    passes: cam_model::ProfilePasses {
                        lead_end_distance: 0.6,
                        lead_end_distance_enabled: Some(false),
                        ..Default::default()
                    },
                    selections: None,
                    closed: true,
                    additional_chains: Vec::new(),
                    tabs: Default::default(),
                    clearing: cam_model::Clearing::default(),
                    id,
                    tool: 1,
                    chain,
                    side: Side::Outside,
                    comp: Comp::Computed,
                    offset: 0.0,
                    depth: p.depth,
                    stepdown: p.stepdown,
                    stepover: 0.0,
                    spindle_rpm: 0.0,
                    tool_preset: None,
                    cooling: Default::default(),
                    entry_feeds: Default::default(),
                    work_offset: 1,
                    feed: p.feed,
                    plunge_feed: p.plunge_feed,
                    end: None,
                    start: None,
                    lead_in: Lead::Arc { radius: 0.6 },
                    lead_out: Lead::Arc { radius: 0.6 },
                    lead_overlap: 0.6,
                    plunge: Plunge::Straight,
                })
            };
            for region in &self.regions {
                operations.push(outer_profile(id, region.outer().clone()));
                id += 1;
                for hole in region.holes() {
                    // A hole that fits the tool is a drill; a larger one is a pocket.
                    // (An inner profile would leave an uncut core — see the profile
                    // strategy's warning — so we never seed one for a hole.)
                    let pts = hole.points();
                    let (mut xmin, mut ymin, mut xmax, mut ymax) =
                        (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
                    for pt in pts {
                        xmin = xmin.min(pt.x);
                        ymin = ymin.min(pt.y);
                        xmax = xmax.max(pt.x);
                        ymax = ymax.max(pt.y);
                    }
                    let dia = (xmax - xmin).min(ymax - ymin);
                    if dia <= p.tool_diameter + 1e-9 {
                        operations.push(Operation::Drill(DrillOp {
                            parameter_expressions: Default::default(),
                            id,
                            tool: 1,
                            points: vec![centroid(hole)],
                            depth: p.depth,
                            start_offset: 0.0,
                            peck: None,
                            dwell: None,
                            spindle_rpm: 0.0,
                            tool_preset: None,
                            cooling: Default::default(),
                            entry_feeds: Default::default(),
                            work_offset: 1,
                            feed: p.plunge_feed,
                        }));
                    } else {
                        operations.push(Operation::Pocket(PocketOp {
                            maximum_stepover: None,
                            parameter_expressions: Default::default(),
                            wrap: None,
                            additional_regions: Vec::new(),
                            stock_contours: Default::default(),
                            finishing: Default::default(),
                            id,
                            tool: 1,
                            boundary: hole.clone(),
                            islands: Vec::new(),
                            depth: p.depth,
                            spindle_rpm: 0.0,
                            tool_preset: None,
                            cooling: Default::default(),
                            entry_feeds: Default::default(),
                            work_offset: 1,
                            end: None,
                            start: None,
                            clear: ClearParams {
                                multiple_depths: Default::default(),
                                adaptive_enabled: None,
                                pocket_overlap: None,
                                keep_tool_down: false,
                                lift_height: 0.2,
                                clearing: cam_model::Clearing::default(),
                                stepdown: p.stepdown,
                                overlap: 0.5,
                                offset: 0.0,
                                feed: p.feed,
                                plunge_feed: p.plunge_feed,
                                plunge: Plunge::Straight,
                                lead_overlap: 0.0,
                                lead_in: Lead::Arc { radius: 0.6 },
                                lead_out: Lead::Arc { radius: 0.6 },
                            },
                        }));
                    }
                    id += 1;
                }
            }
        }

        Document::new(Setup {
            id: 1,
            frame: Default::default(),
            material: String::new(),
            material_id: None,
            model_instances: Vec::new(),
            fixture_instances: Vec::new(),
            fixture_clearance: Default::default(),
            program_name: String::new(),
            name: self.program_name(),
            heights: Heights::new(p.clearance, p.retract, p.top_of_stock),
            stock: default_stock(),
            tools,
            operations,
            patterns: Vec::new(),
            origin: [0.0, 0.0, 0.0],
            extra_origins: vec![],
            origin_index: 1,
            operation_heights: Default::default(),
            height_settings: Default::default(),
            stock_to_leave: Default::default(),
            tool_change_height: None,
        })
    }

    /// The current stock resolved to a concrete box `(min, max)` (mm), fitting the
    /// stock spec to the loaded geometry's XY bounds.
    pub fn stock_box(&self) -> ([f64; 3], [f64; 3]) {
        self.document().stock_bounds_for(self.setup())
    }

    /// Edit the setup's stock definition as one undoable change.
    pub fn edit_stock(&mut self, f: impl FnOnce(&mut Stock)) {
        self.edit_setup_document(|doc| {
            f(&mut doc.setup_mut().stock);
            crate::cad_operations::refresh_binding_heights(doc);
        });
        self.invalidate();
    }

    fn program_name(&self) -> String {
        if !self.setup().program_name.trim().is_empty() {
            self.setup().program_name.clone()
        } else if self.document().setups.len() > 1 {
            format!(
                "{}-setup-{}",
                if self.source_name.is_empty() {
                    "opencamstudio"
                } else {
                    &self.source_name
                },
                self.active_setup_id
            )
        } else if self.source_name.is_empty() {
            "opencamstudio".to_string()
        } else {
            self.source_name.clone()
        }
    }

    fn sync_drawing_geometry(&mut self) {
        self.regions = self.document().geometry.drawing_regions.clone();
        self.open_paths = self.document().geometry.drawing_paths.clone();
        self.cad_selection
            .retain(|r| self.document.current().geometry.entity_exists(r));
    }

    /// After an undo/redo, drop a selection that no longer points at anything.
    fn reconcile_selection(&mut self) {
        let sel = self.selection;
        self.select(sel);
    }

    fn edit_document(&mut self, f: impl FnOnce(&mut Document)) {
        let op_floor = self.operation_id_floor;
        let setup_floor = self.setup_id_floor;
        let asset_floor = self.asset_id_floor;
        let instance_floor = self.instance_id_floor;
        let pattern_floor = self.pattern_id_floor;
        self.document.edit(|doc| {
            doc.record_ids();
            doc.operation_id_floor = doc.operation_id_floor.max(op_floor);
            doc.setup_id_floor = doc.setup_id_floor.max(setup_floor);
            doc.asset_id_floor = doc.asset_id_floor.max(asset_floor);
            doc.instance_id_floor = doc.instance_id_floor.max(instance_floor);
            doc.pattern_id_floor = doc.pattern_id_floor.max(pattern_floor);
            f(doc);
            doc.record_ids();
        });
        self.operation_id_floor = self
            .operation_id_floor
            .max(self.document().operation_id_floor);
        self.setup_id_floor = self.setup_id_floor.max(self.document().setup_id_floor);
        self.asset_id_floor = self.asset_id_floor.max(self.document().asset_id_floor);
        self.instance_id_floor = self
            .instance_id_floor
            .max(self.document().instance_id_floor);
        self.pattern_id_floor = self.pattern_id_floor.max(self.document().pattern_id_floor);
    }

    /// A direct setup edit can create the first setup, in the same undo step.
    fn edit_setup_document(&mut self, f: impl FnOnce(&mut Document)) {
        let mut seed = default_setup(&self.defaults);
        seed.id = self.document().next_setup_id().max(self.setup_id_floor);
        self.edit_document(move |doc| {
            if doc.setups.is_empty() {
                doc.active_setup_id = seed.id;
                doc.setups.push(seed);
            }
            f(doc);
        });
        self.restore_active_setup_view();
    }

    fn invalidate_export(&mut self) {
        self.document_epoch = self
            .document_epoch
            .checked_add(1)
            .expect("document revision exhausted");
        self.jobs.cancel();
        self.nc = None;
    }

    /// Drop any run/export that no longer reflects the inputs.
    fn invalidate(&mut self) {
        self.document_epoch = self
            .document_epoch
            .checked_add(1)
            .expect("document revision exhausted");
        self.jobs.cancel();
        self.outcome = None;
        self.nc = None;
    }
}

/// The average of a contour's vertices — a good-enough default drill point for a
/// closed loop (exact for a circle, sensible for the polygonal holes we import).
fn centroid(c: &Contour) -> [f64; 2] {
    let pts = c.points();
    let n = pts.len().max(1) as f64;
    let (sx, sy) = pts
        .iter()
        .fold((0.0, 0.0), |(sx, sy), p| (sx + p.x, sy + p.y));
    [sx / n, sy / n]
}

/// Squared distance from point `p` to the segment `a`–`b`.
fn dist_point_seg2(p: Point, a: Point, b: Point) -> f64 {
    let (abx, aby) = (b.x - a.x, b.y - a.y);
    let len2 = abx * abx + aby * aby;
    let t = if len2 > 0.0 {
        (((p.x - a.x) * abx + (p.y - a.y) * aby) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let (cx, cy) = (a.x + t * abx, a.y + t * aby);
    (p.x - cx).powi(2) + (p.y - cy).powi(2)
}

/// Closed-loop corner detection: indices of vertices where the turn between the
/// incoming and outgoing edge exceeds ~20° — real corners, not the many small
/// turns of a flattened arc (whose facets stay well under the threshold).
const CORNER_COS: f64 = 0.94; // cos(20°) ≈ 0.9397

pub(crate) fn loop_corners(pts: &[Point]) -> Vec<usize> {
    let n = pts.len();
    let mut out = Vec::new();
    if n < 3 {
        return out;
    }
    for i in 0..n {
        let (prev, cur, next) = (pts[(i + n - 1) % n], pts[i], pts[(i + 1) % n]);
        let (ax, ay) = (cur.x - prev.x, cur.y - prev.y);
        let (bx, by) = (next.x - cur.x, next.y - cur.y);
        let (la, lb) = ((ax * ax + ay * ay).sqrt(), (bx * bx + by * by).sqrt());
        if la < 1e-9 || lb < 1e-9 {
            continue;
        }
        // Angle between successive edge directions; a corner turns more than ~20°.
        if (ax * bx + ay * by) / (la * lb) < CORNER_COS {
            out.push(i);
        }
    }
    out
}

/// Arc-length midpoints of the spans between consecutive corners — the "mid" of
/// each real edge. A loop with no corners (a circle) has no straight edges, so
/// returns nothing (its mid is a Phase-2 arc concern).
pub(crate) fn loop_mids(pts: &[Point], corners: &[usize]) -> Vec<[f64; 2]> {
    if corners.len() < 2 {
        return Vec::new();
    }
    let n = pts.len();
    let mut mids = Vec::with_capacity(corners.len());
    for k in 0..corners.len() {
        let (c0, c1) = (corners[k], corners[(k + 1) % corners.len()]);
        // Walk the span c0 → c1 (cyclic), accumulating arc length to its half.
        let mut span = vec![pts[c0]];
        let mut i = c0;
        while i != c1 {
            i = (i + 1) % n;
            span.push(pts[i]);
        }
        let total: f64 = span.windows(2).map(|w| w[0].distance_sq(w[1]).sqrt()).sum();
        let half = total / 2.0;
        let mut acc = 0.0;
        let mut mid = [span[0].x, span[0].y];
        for w in span.windows(2) {
            let seg = w[0].distance_sq(w[1]).sqrt();
            if acc + seg >= half {
                let t = if seg > 1e-9 { (half - acc) / seg } else { 0.0 };
                mid = [
                    w[0].x + (w[1].x - w[0].x) * t,
                    w[0].y + (w[1].y - w[0].y) * t,
                ];
                break;
            }
            acc += seg;
        }
        mids.push(mid);
    }
    mids
}

/// Fit a circle to a closed loop, returning `(center, radius)` only if the loop
/// really is round: enough points, and every one within 5% of the mean radius
/// about the centroid. Used for **Quadrant** snaps on circular holes/bosses; a
/// polygon (few points, or corners) or an ellipse is rejected. Partial arcs need
/// the deferred arc-carrying geometry model.
pub(crate) fn fit_circle(pts: &[Point]) -> Option<([f64; 2], f64)> {
    if pts.len() < 8 {
        return None; // a rectangle/hexagon is not a circle
    }
    let n = pts.len() as f64;
    let cx = pts.iter().map(|p| p.x).sum::<f64>() / n;
    let cy = pts.iter().map(|p| p.y).sum::<f64>() / n;
    let radii: Vec<f64> = pts
        .iter()
        .map(|p| ((p.x - cx).powi(2) + (p.y - cy).powi(2)).sqrt())
        .collect();
    let r = radii.iter().sum::<f64>() / n;
    if r < 1e-6 {
        return None;
    }
    let max_dev = radii.iter().map(|ri| (ri - r).abs()).fold(0.0, f64::max);
    (max_dev / r <= 0.05).then_some(([cx, cy], r))
}

/// The nearest point on a closed contour to `w`, and its squared distance.
fn nearest_point_on_contour(pts: &[Point], w: Point) -> ([f64; 2], f64) {
    nearest_point_on_path(pts, w, true)
}

/// Nearest point on a polyline. With `closed`, the last→first segment is included;
/// for an **open** path it is not — otherwise a stroke would be pickable along a
/// phantom segment joining its two ends.
fn nearest_point_on_path(pts: &[Point], w: Point, closed: bool) -> ([f64; 2], f64) {
    let n = pts.len();
    let mut best = (
        [
            pts.first().map_or(0.0, |p| p.x),
            pts.first().map_or(0.0, |p| p.y),
        ],
        f64::MAX,
    );
    let last = if closed { n } else { n.saturating_sub(1) };
    for k in 0..last {
        let (a, b) = (pts[k], pts[(k + 1) % n]);
        let (abx, aby) = (b.x - a.x, b.y - a.y);
        let len2 = abx * abx + aby * aby;
        let t = if len2 > 0.0 {
            (((w.x - a.x) * abx + (w.y - a.y) * aby) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (cx, cy) = (a.x + t * abx, a.y + t * aby);
        let d2 = (w.x - cx).powi(2) + (w.y - cy).powi(2);
        if d2 < best.1 {
            best = ([cx, cy], d2);
        }
    }
    best
}

/// Whether two tools have the same cutting geometry, ignoring their numbers — used
/// to dedupe when embedding a library tool into a project's setup.
fn same_tool_geometry(a: &Tool, b: &Tool) -> bool {
    a.diameter == b.diameter && a.length == b.length && a.flutes == b.flutes && a.kind == b.kind
}

/// The most cells the material-removal sim may use across the part's larger side.
/// A hard ceiling on [`narrowest_cut_width`]-driven refinement: cost grows as the
/// square, so this bounds a fine-groove job to ~1.4 M cells rather than letting a
/// 0.1 mm feature on a 300 mm plate ask for nine billion.
pub(crate) const MAX_SIM_CELLS_ACROSS: f64 = 1200.0;

/// The width of the **narrowest cut** `program` makes, in mm — the feature size the
/// removal sim has to resolve.
///
/// For a tool whose footprint is its full diameter (end/ball/bull/face mill, drill)
/// that is simply the diameter. For a **pointed** tool it is not: a V-bit or chamfer
/// mill cuts a groove whose width depends on how deep it goes, and at engraving
/// depths that is far narrower than the tool. Those are measured from the program's
/// actual deepest cut with that tool, against `surface_z`.
///
/// Returns [`f64::INFINITY`] when nothing is cut, so callers fall back to their
/// span-derived resolution.
pub(crate) fn narrowest_cut_width(program: &Program, tools: &[Tool], surface_z: f64) -> f64 {
    use cam_cldata::{MoveKind, Step};
    use std::collections::BTreeMap;

    // The deepest cutting Z reached under each tool number, walking the tool changes.
    let mut current: Option<u32> = None;
    let mut deepest: BTreeMap<u32, f64> = BTreeMap::new();
    for step in program.steps() {
        match step {
            Step::ToolChange { tool } => current = Some(*tool),
            Step::Linear { to, tag, .. }
            | Step::Rotary {
                to,
                tag,
                feed: Some(_),
                ..
            } if tag.kind == MoveKind::Cutting => {
                if let Some(n) = current {
                    let e = deepest.entry(n).or_insert(f64::INFINITY);
                    *e = e.min(to.z);
                }
            }
            Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => {
                if let Some(n) = current {
                    let e = deepest.entry(n).or_insert(f64::INFINITY);
                    *e = e.min(end.z);
                }
            }
            _ => {}
        }
    }

    let mut narrowest = f64::INFINITY;
    for (number, min_z) in deepest {
        let Some(tool) = tools.iter().find(|t| t.number == number) else {
            continue;
        };
        let depth = (surface_z - min_z).max(0.0);
        let width = match tool.kind {
            // Pointed tools: the groove is as wide as the cone has opened at depth.
            ToolKind::VBit {
                included_angle_deg,
                tip_radius,
            } => {
                let a = 0.5 * included_angle_deg.to_radians();
                2.0 * cam_geo::vtip_half_width(a, tip_radius, depth)
            }
            ToolKind::ChamferMill {
                included_angle_deg,
                tip_diameter,
            } => {
                let a = 0.5 * included_angle_deg.to_radians();
                tip_diameter + 2.0 * depth * a.tan()
            }
            // Everything else cuts its full width as soon as it is in the material.
            _ => tool.diameter,
        };
        // A tool never cuts wider than itself, and a zero-depth "cut" is not a
        // feature to resolve.
        let width = width.min(tool.diameter);
        if width > 1e-9 {
            narrowest = narrowest.min(width);
        }
    }
    narrowest
}

/// Translate a document [`Tool`] into the simulator's [`ToolProfile`] — how its
/// bottom removes material (flat, ball, bull-nose corner, or a chamfer/drill cone).
pub(crate) fn sim_profile(tool: &Tool) -> Option<ToolProfile> {
    let radius = tool.radius()
        + match tool.kind {
            ToolKind::RadiusMill { corner_radius, .. } => corner_radius.max(0.0),
            _ => 0.0,
        };
    let shape = match tool.kind {
        ToolKind::EndMill | ToolKind::RoughingMill | ToolKind::FaceMill => ProfileShape::Flat,
        ToolKind::GrooveCutter | ToolKind::SlittingSaw => return None,
        ToolKind::BallMill => ProfileShape::Ball,
        ToolKind::BullNose { corner_radius } => ProfileShape::BullNose {
            corner_radius: corner_radius.clamp(0.0, radius),
        },
        ToolKind::RadiusMill {
            corner_radius,
            tip_length,
        } => ProfileShape::RadiusMill {
            corner_radius,
            tip_length,
        },
        ToolKind::ChamferMill {
            included_angle_deg,
            tip_diameter,
        } => ProfileShape::Cone {
            half_angle_rad: included_angle_deg.to_radians() / 2.0,
            flat_radius: (tip_diameter / 2.0).clamp(0.0, radius),
        },
        ToolKind::Drill { point_angle_deg } => ProfileShape::Cone {
            half_angle_rad: point_angle_deg.to_radians() / 2.0,
            flat_radius: 0.0,
        },
        // A V-bit's tip is a ball tangent to the cone, not a flat — the sim models it
        // exactly (`ProfileShape::VTip`), so an engraved groove simulates at its true
        // width rather than the narrower one a flat-tipped cone would leave.
        ToolKind::VBit {
            included_angle_deg,
            tip_radius,
        } => ProfileShape::VTip {
            half_angle_rad: included_angle_deg.to_radians() / 2.0,
            tip_radius: tip_radius.clamp(0.0, radius),
        },
        ToolKind::ThreadMill { pitch } => {
            let outline = tool.profile();
            ProfileShape::Thread {
                root_radius: outline.segs[0].end.x,
                tooth_height: outline.segs[2].end.y,
                pitch: pitch.unwrap_or(0.).max(0.),
                tooth_count: (outline.segs.iter().filter(|s| s.cutting).count() / 2) as u32,
            }
        }
    };
    Some(ToolProfile { radius, shape })
}

/// Set an operation's id, whatever its kind.
/// The [`OpKind`] an existing operation was built as — so reinitialising it offers
/// the same kind of pick.
fn op_kind_of(op: &Operation) -> OpKind {
    match op {
        Operation::Profile(_) => OpKind::Profile,
        Operation::Drill(_) => OpKind::Drill,
        Operation::Pocket(_) => OpKind::Pocket,
        Operation::Face(_) => OpKind::Face,
        Operation::Chamfer(_) => OpKind::Chamfer,
        Operation::Bore(_) => OpKind::Bore,
        Operation::Thread(_) => OpKind::Thread,
        Operation::Engrave(_) => OpKind::Engrave,
        Operation::Carve(_) => OpKind::Carve,
        Operation::Adaptive(_) => OpKind::Adaptive,
        Operation::Parallel(_) => OpKind::Parallel,
        Operation::Spiral(_) => OpKind::Spiral,
    }
}

fn set_op_id(op: &mut Operation, id: u32) {
    op.set_id(id);
}

/// Retarget whichever origin currently holds index `from` to `to` — the base origin
/// (its index lives in `Setup::origin_index`) or a matching extra. Used to renumber /
/// swap origin indices.
fn set_origin_index_value(setup: &mut Setup, from: u32, to: u32) {
    if setup.origin_index == from {
        setup.origin_index = to;
    } else if let Some(o) = setup.extra_origins.iter_mut().find(|o| o.index == from) {
        o.index = to;
    }
}

/// New relative stock extends one millimetre beyond each model face.
fn default_stock() -> Stock {
    Stock::RelativeBox {
        offsets: [[1.0; 3]; 2],
    }
}

/// Creation defaults: model-relative stock, one default end mill, no operations.
/// These values are not a saved setup until a setup is explicitly created or edited.
fn default_setup(p: &JobParams) -> Setup {
    Setup {
        id: 1,
        frame: Default::default(),
        material: String::new(),
        material_id: None,
        model_instances: Vec::new(),
        fixture_instances: Vec::new(),
        fixture_clearance: Default::default(),
        program_name: String::new(),
        name: "Untitled".to_string(),
        heights: Heights::new(p.clearance, p.retract, p.top_of_stock),
        stock: default_stock(),
        tools: vec![Tool {
            number: 1,
            diameter: p.tool_diameter,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }],
        operations: Vec::new(),
        patterns: Vec::new(),
        origin: [0.0, 0.0, 0.0],
        extra_origins: vec![],
        origin_index: 1,
        operation_heights: Default::default(),
        height_settings: Default::default(),
        stock_to_leave: Default::default(),
        tool_change_height: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_model::{Envelope, Point3};

    #[cfg(feature = "gui")]
    #[test]
    fn system_operation_template_uses_programmed_depth_and_cutting_data() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.defaults.depth = 6.;
        let tool = app.setup().tools[0].number;
        app.edit_tool(0, |tool| {
            tool.nominal_feed = 575.;
            tool.presets.clear();
        });
        let Operation::Profile(project) = app.operation_template(OpKind::Profile, tool) else {
            panic!("profile template");
        };
        let Operation::Profile(system) = app.system_operation_template(OpKind::Profile) else {
            panic!("profile template");
        };
        assert_eq!((project.depth, project.feed), (6., 575.));
        let defaults = JobParams::default();
        assert_eq!((system.depth, system.feed), (defaults.depth, defaults.feed));
        assert_eq!(system.spindle_rpm, defaults.spindle_rpm);
    }

    #[test]
    fn unsaved_changes_follow_successful_save_open_undo_and_redo() {
        let mut app = AppController::new(machine());
        assert!(!app.has_unsaved_changes());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert!(app.has_unsaved_changes());
        let path = std::env::temp_dir().join(format!("unsaved-state-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        assert!(!app.has_unsaved_changes());
        app.edit_setup(|setup| setup.name = "Changed".into());
        assert!(app.has_unsaved_changes());
        assert!(app.undo());
        assert!(!app.has_unsaved_changes());
        assert!(app.redo());
        assert!(app.has_unsaved_changes());
        // Failed writes/reads never reset the saved baseline or current content.
        assert!(app.save_project(path.join("cannot-save.ocam")).is_err());
        assert!(app.open_project(path.join("cannot-open.ocam")).is_err());
        assert!(app.has_unsaved_changes());
        assert_eq!(app.document().setup().name, "Changed");
        app.open_project(&path).unwrap();
        assert!(!app.has_unsaved_changes());
        app.new_project();
        assert!(!app.has_unsaved_changes());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn unsaved_changes_ignore_session_settings_and_allocation_bookkeeping() {
        let mut app = AppController::new(machine());
        app.edit_machine(|machine| machine.name = "Other shop".into());
        app.set_post_kind(PostKind::Grbl);
        app.document.update_view(|document| {
            document.operation_id_floor += 10;
            document.active_setup_id = 99;
        });
        assert!(!app.has_unsaved_changes());
        app.defaults.depth += 1.0;
        assert!(app.has_unsaved_changes());
    }

    const PART_DXF: &str = "\
0\nSECTION\n2\nENTITIES\n\
0\nLINE\n10\n10.0\n20\n10.0\n11\n70.0\n21\n10.0\n\
0\nLINE\n10\n70.0\n20\n10.0\n11\n70.0\n21\n50.0\n\
0\nLINE\n10\n70.0\n20\n50.0\n11\n10.0\n21\n50.0\n\
0\nLINE\n10\n10.0\n20\n50.0\n11\n10.0\n21\n10.0\n\
0\nCIRCLE\n10\n40.0\n20\n30.0\n40\n5.0\n\
0\nENDSEC\n0\nEOF\n";

    /// The same part plus two disconnected LINEs forming an open "V" stroke — the
    /// lettering-like geometry the importer keeps as an open chain.
    const PART_WITH_STROKE_DXF: &str = "\
0\nSECTION\n2\nENTITIES\n\
0\nLINE\n10\n10.0\n20\n10.0\n11\n70.0\n21\n10.0\n\
0\nLINE\n10\n70.0\n20\n10.0\n11\n70.0\n21\n50.0\n\
0\nLINE\n10\n70.0\n20\n50.0\n11\n10.0\n21\n50.0\n\
0\nLINE\n10\n10.0\n20\n50.0\n11\n10.0\n21\n10.0\n\
0\nLINE\n10\n20.0\n20\n20.0\n11\n30.0\n21\n40.0\n\
0\nLINE\n10\n30.0\n20\n40.0\n11\n40.0\n21\n20.0\n\
0\nENDSEC\n0\nEOF\n";

    fn machine() -> Machine {
        Machine {
            name: "test".into(),
            rapid_rate: 2000.0,
            tool_change_seconds: 0.0,
            min_spindle_rpm: 0.0,
            max_spindle_rpm: 10_000.0,
            max_feed: 800.0,
            envelope: Envelope::new(
                Point3::new(0.0, 0.0, -50.0),
                Point3::new(300.0, 180.0, 50.0),
            ),
            safe_z: 5.0,
            tool_change_pos: None,
        }
    }

    fn cutting_program(tool: u32, z: f64) -> Program {
        use cam_cldata::{MoveKind, Point3, Step, Tag};
        let mut p = Program::new();
        p.push(Step::ToolChange { tool });
        p.push(Step::Linear {
            to: Point3::new(10.0, 0.0, z),
            feed: 300.0,
            tag: Tag::new(0, MoveKind::Cutting),
        });
        p
    }

    fn tool_of(number: u32, diameter: f64, kind: ToolKind) -> Tool {
        Tool {
            number,
            diameter,
            length: 30.0,
            flutes: 2,
            kind,
            ..Default::default()
        }
    }

    #[test]
    fn narrowest_cut_is_the_diameter_for_a_flat_tool() {
        let tools = [tool_of(1, 6.0, ToolKind::EndMill)];
        let w = narrowest_cut_width(&cutting_program(1, -3.0), &tools, 0.0);
        assert!((w - 6.0).abs() < 1e-9, "{w}");
    }

    #[test]
    fn narrowest_cut_for_a_vbit_is_the_groove_not_the_tool() {
        // The whole point of the refinement: a ⌀6 V-bit engraving 0.3 mm deep cuts a
        // groove far narrower than 6 mm, and the sim must resolve the groove.
        let tools = [tool_of(
            1,
            6.0,
            ToolKind::VBit {
                included_angle_deg: 60.0,
                tip_radius: 0.0,
            },
        )];
        let w = narrowest_cut_width(&cutting_program(1, -0.3), &tools, 0.0);
        let want = 2.0 * 0.3 * (30.0_f64).to_radians().tan();
        assert!((w - want).abs() < 1e-9, "{w} want {want}");
        assert!(w < 0.4, "a groove, not the tool: {w}");
    }

    #[test]
    fn narrowest_cut_never_exceeds_the_tool_diameter() {
        // A V-bit driven deep would open past its own ⌀ by the formula; clamp it.
        let tools = [tool_of(
            1,
            6.0,
            ToolKind::VBit {
                included_angle_deg: 120.0,
                tip_radius: 0.0,
            },
        )];
        let w = narrowest_cut_width(&cutting_program(1, -50.0), &tools, 0.0);
        assert!((w - 6.0).abs() < 1e-9, "{w}");
    }

    #[test]
    fn narrowest_cut_takes_the_minimum_across_tools() {
        use cam_cldata::{MoveKind, Point3, Step, Tag};
        let tools = [
            tool_of(1, 6.0, ToolKind::EndMill),
            tool_of(
                2,
                6.0,
                ToolKind::VBit {
                    included_angle_deg: 60.0,
                    tip_radius: 0.0,
                },
            ),
        ];
        let mut p = cutting_program(1, -3.0);
        p.push(Step::ToolChange { tool: 2 });
        p.push(Step::Linear {
            to: Point3::new(20.0, 0.0, -0.3),
            feed: 300.0,
            tag: Tag::new(1, MoveKind::Cutting),
        });
        let w = narrowest_cut_width(&p, &tools, 0.0);
        assert!(w < 0.4, "the engraving groove must win: {w}");
    }

    #[test]
    fn narrowest_cut_is_infinite_when_nothing_cuts() {
        let tools = [tool_of(1, 6.0, ToolKind::EndMill)];
        assert!(narrowest_cut_width(&Program::new(), &tools, 0.0).is_infinite());
    }

    fn depth_of(op: &Operation) -> f64 {
        match op {
            Operation::Profile(o) => o.depth,
            Operation::Pocket(o) => o.depth,
            Operation::Face(o) => o.depth,
            Operation::Drill(o) => o.depth,
            Operation::Bore(o) => o.z_bottom,
            Operation::Thread(o) => o.z_bottom,
            Operation::Chamfer(o) => o.top,
            Operation::Engrave(o) => o.depth,
            Operation::Carve(o) => o.depth,
            Operation::Adaptive(o) => o.params.stepdown,
            Operation::Parallel(_) | Operation::Spiral(_) => 1.0,
        }
    }

    #[test]
    fn open_generates_ops_and_selects_the_first() {
        let mut app = AppController::new(machine());
        assert_eq!(app.open_dxf(PART_DXF, "part.dxf").unwrap(), 1);
        // Outside profile for the boundary + a hole op (drill/pocket, never an
        // inner profile) = 2 operations.
        assert_eq!(app.document().setup().operations.len(), 2);
        assert_eq!(app.selection(), Selection::Operation(0));
        assert!(app.selected_operation().is_some());
    }

    #[test]
    fn open_run_export_round_trip() {
        let mut app = AppController::new(machine());
        assert_eq!(app.open_dxf(PART_DXF, "part.dxf").unwrap(), 1);

        let outcome = app.run(&CancelToken::new());
        assert!(
            !outcome.has_errors(),
            "no errors: {:?}",
            outcome.diagnostics
        );
        assert!(!outcome.program.is_empty());
        assert!(
            !outcome.scene.strips.is_empty(),
            "scene should have geometry"
        );

        let nc = app.export_nc().unwrap();
        assert!(nc.contains("M30") && nc.contains("G54"));
    }

    #[test]
    fn estlcam_export_passes_setup_tool_details_and_operation_speed() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.edit_setup(|setup| {
            let tool = &mut setup.tools[0];
            tool.description = "Workshop rougher".into();
            tool.flute_length = 12.0;
            tool.nominal_rpm = 8000.0;
            for operation in &mut setup.operations {
                operation.set_spindle_rpm(6000.0);
            }
        });
        app.set_post_kind(PostKind::Estlcam);
        app.run(&CancelToken::new());
        let nc = app.export_nc().unwrap();
        let change = nc.lines().find(|line| line.contains(" M6 ")).unwrap();
        assert!(change.contains(" - Workshop rougher - Square End Mill - D6.000 - FL12.000 - CR=0.000 - F2 - RPM 6000)"), "{change}");
        assert!(!change.contains("RPM 8000"));
    }

    #[test]
    fn estlcam_output_changes_repost_the_same_toolpath() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let ids: Vec<_> = app.setup().operations.iter().map(Operation::id).collect();
        for id in ids {
            app.edit_operation(id, |op| op.set_cooling(cam_model::Cooling::Mms));
        }
        app.set_post_kind(PostKind::Estlcam);
        app.run(&CancelToken::new());
        let first = app.export_nc().unwrap().to_string();
        assert!(first.lines().any(|line| line == "M8"));
        let program = app.outcome().unwrap().program.clone();
        let mms = cam_post::EstlcamOptions {
            output_1: false,
            output_2: true,
        };
        app.set_estlcam_options(mms);
        assert!(app.exported_nc().is_none());
        assert_eq!(app.outcome().unwrap().program, program);
        let nc = app.export_nc().unwrap();
        assert!(nc.lines().any(|line| line == "M10"));
        assert!(!nc.lines().any(|line| matches!(line, "M8" | "M9")));
        assert!(nc.ends_with("M5\nM11\n"));
        app.new_project();
        assert_eq!(app.estlcam_options(), mms);
    }

    #[test]
    fn run_produces_a_simulated_stock_mesh() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let outcome = app.run(&CancelToken::new());
        assert!(
            !outcome.stock_vertices.is_empty(),
            "a run should simulate a stock surface"
        );
        assert!(
            !outcome.stock_indices.is_empty() && outcome.stock_indices.len().is_multiple_of(3),
            "indices form whole triangles"
        );
        assert!(
            outcome
                .stock_indices
                .iter()
                .all(|&i| (i as usize) < outcome.stock_vertices.len()),
            "every index is in range"
        );
    }

    #[test]
    fn editing_the_selected_operation_is_undoable_and_invalidates() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.run(&CancelToken::new());
        assert!(app.outcome().is_some());

        // Op 0 is selected after import.
        app.edit_selected_operation(|op| {
            if let Operation::Profile(o) = op {
                o.depth = 8.0;
            }
        });
        assert_eq!(depth_of(app.operation(0).unwrap()), 8.0);
        assert!(app.outcome().is_none(), "editing invalidates the stale run");
        assert!(app.can_undo());

        assert!(app.undo());
        assert_eq!(depth_of(app.operation(0).unwrap()), 4.0, "undo restores");
    }

    fn op_ids(app: &AppController) -> Vec<u32> {
        app.document()
            .setup()
            .operations
            .iter()
            .map(|o| o.id())
            .collect()
    }

    #[test]
    fn patterns_group_noncontiguous_sources_and_round_trip_through_history_and_disk() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let existing = op_ids(&app);
        assert_eq!(existing.len(), 2);
        let unselected = existing[1];
        let third = 900;
        app.edit_document(|doc| {
            doc.setup_mut()
                .operations
                .push(Operation::Drill(cam_model::DrillOp {
                    parameter_expressions: Default::default(),
                    id: third,
                    tool: 1,
                    points: vec![[25.0, 25.0]],
                    depth: 1.0,
                    start_offset: 0.0,
                    peck: None,
                    dwell: None,
                    spindle_rpm: 1000.0,
                    tool_preset: None,
                    cooling: Default::default(),
                    entry_feeds: Default::default(),
                    work_offset: 1,
                    feed: 100.0,
                }));
        });
        let first = existing[0];
        let setup_id = app.active_setup_id();
        let pattern_id = app
            .create_pattern(
                setup_id,
                &[first, third],
                "Pair".into(),
                cam_model::PatternKind::Rectangular {
                    nx: 2,
                    ny: 1,
                    dx: 12.0,
                    dy: 0.0,
                },
                cam_model::PatternOrder::ByInstance,
            )
            .unwrap();
        let setup = app.document().setup();
        assert_eq!(setup.patterns[0].members, vec![first, third]);
        assert_eq!(op_ids(&app), vec![first, third, unselected]);
        setup.validate_patterns().unwrap();
        app.move_operation(unselected, true);
        assert_eq!(
            op_ids(&app),
            vec![first, third, unselected],
            "plain operations cannot split a neighboring group"
        );
        app.document().setup().validate_patterns().unwrap();

        assert!(app.undo());
        assert!(app.document().setup().patterns.is_empty());
        assert_eq!(op_ids(&app), vec![first, unselected, third]);
        // Undo must not return an allocated identifier to the pool.
        let next_id = app
            .create_pattern(
                setup_id,
                &[first],
                "Next".into(),
                cam_model::PatternKind::Rectangular {
                    nx: 2,
                    ny: 1,
                    dx: 1.0,
                    dy: 0.0,
                },
                cam_model::PatternOrder::ByOperation,
            )
            .unwrap();
        assert!(next_id > pattern_id);
        assert!(app.undo());
        assert!(app.redo());
        assert_eq!(app.document().setup().patterns[0].id, next_id);

        let path = temp_path("patterns.ocam");
        app.save_project(&path).unwrap();
        let expected = app.document().clone();
        app.new_project();
        app.open_project(&path).unwrap();
        assert_eq!(app.document(), &expected);
        assert_eq!(app.next_pattern_id(), expected.pattern_id_floor);
        std::fs::remove_file(path).ok();
    }

    #[test]
    fn pattern_id_exhaustion_does_not_mutate_the_document() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let setup_id = app.active_setup_id();
        let source = op_ids(&app)[0];
        app.edit_document(|doc| doc.pattern_id_floor = u64::MAX);
        let before = app.document().clone();

        let error = app
            .create_pattern(
                setup_id,
                &[source],
                "Exhausted".into(),
                cam_model::PatternKind::Rectangular {
                    nx: 2,
                    ny: 1,
                    dx: 1.0,
                    dy: 0.0,
                },
                cam_model::PatternOrder::ByInstance,
            )
            .unwrap_err();

        assert_eq!(error, "Pattern ID range exhausted");
        assert_eq!(app.document(), &before);
    }

    #[test]
    fn pattern_commands_reject_mixed_datums_and_dissolve_or_delete_atomically() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let ids = op_ids(&app);
        assert!(ids.len() >= 2);
        let setup_id = app.active_setup_id();
        app.edit_document(|doc| {
            if let Some(op) = doc
                .setup_mut()
                .operations
                .iter_mut()
                .find(|op| op.id() == ids[1])
            {
                op.set_work_offset(2);
            }
        });
        let rect = cam_model::PatternKind::Rectangular {
            nx: 2,
            ny: 1,
            dx: 1.0,
            dy: 0.0,
        };
        assert!(app
            .create_pattern(
                setup_id,
                &ids,
                "Bad datum".into(),
                rect,
                cam_model::PatternOrder::ByInstance
            )
            .is_err());
        app.edit_document(|doc| {
            if let Some(op) = doc
                .setup_mut()
                .operations
                .iter_mut()
                .find(|op| op.id() == ids[1])
            {
                op.set_work_offset(1);
            }
        });
        let pattern = app
            .create_pattern(
                setup_id,
                &ids,
                "Group".into(),
                rect,
                cam_model::PatternOrder::ByInstance,
            )
            .unwrap();
        app.edit_pattern(
            setup_id,
            pattern,
            &[ids[0]],
            "Reduced".into(),
            cam_model::PatternKind::Rectangular {
                nx: 2,
                ny: 1,
                dx: 2.0,
                dy: 0.0,
            },
            cam_model::PatternOrder::ByOperation,
        )
        .unwrap();
        assert_eq!(app.document().setup().patterns[0].members, vec![ids[0]]);
        assert_eq!(
            op_ids(&app),
            ids,
            "removed members remain immediately after the group"
        );
        app.document().setup().validate_patterns().unwrap();
        assert!(app.undo());
        assert_eq!(app.document().setup().patterns[0].members, ids);
        app.dissolve_pattern(setup_id, pattern);
        assert!(app.document().setup().patterns.is_empty());
        assert_eq!(op_ids(&app), ids);
        assert!(app.undo());
        assert_eq!(app.document().setup().patterns[0].id, pattern);
        app.delete_pattern_and_operations(setup_id, pattern);
        assert!(app.document().setup().operations.is_empty());
        assert!(app.document().setup().patterns.is_empty());
        assert!(app.undo());
        assert_eq!(app.document().setup().operations.len(), ids.len());
        assert_eq!(app.document().setup().patterns[0].id, pattern);
    }

    #[test]
    fn duplicating_a_setup_remaps_pattern_sources_and_blocks_individual_transfer() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let source_id = app.active_setup_id();
        let source_members = op_ids(&app);
        let pattern = app
            .create_pattern(
                source_id,
                &source_members,
                "Duplicated group".into(),
                cam_model::PatternKind::Circular {
                    center_xy: [3.0, 4.0],
                    count: 3,
                    distribution: cam_model::CircularDistribution::Partial {
                        sweep_degrees: 90.0,
                    },
                    direction: cam_model::PatternDirection::Cw,
                },
                cam_model::PatternOrder::ByOperation,
            )
            .unwrap();
        let datum = app.document().setup().operations[0].work_offset();
        app.set_pattern_suppressed(source_id, pattern, true);
        app.edit_document(|doc| {
            doc.excluded_operations.insert(source_members[0]);
            doc.disabled_origins
                .entry(source_id)
                .or_default()
                .insert(datum);
        });
        let target_id = app.duplicate_setup(source_id).unwrap();
        let source = app.setups().iter().find(|s| s.id == source_id).unwrap();
        let target = app.setups().iter().find(|s| s.id == target_id).unwrap();
        assert_eq!(target.patterns.len(), 1);
        assert_ne!(target.patterns[0].id, pattern);
        assert_ne!(target.patterns[0].members, source_members);
        assert_eq!(target.patterns[0].kind, source.patterns[0].kind);
        assert!(
            target.patterns[0].suppressed,
            "pattern suppression is preserved"
        );
        assert!(target.patterns[0]
            .members
            .iter()
            .all(|id| target.operations.iter().any(|op| op.id() == *id)));
        assert!(
            app.document()
                .excluded_operations
                .contains(&target.patterns[0].members[0]),
            "excluded source state follows remapped IDs"
        );
        assert!(
            app.document()
                .disabled_origins
                .get(&target_id)
                .is_some_and(|indices| indices.contains(&datum)),
            "disabled datum state is copied to the new setup"
        );
        assert!(target.validate_patterns().is_ok());
        assert!(app
            .transfer_operation_to_setup(source_members[0], target_id, false)
            .is_err());
    }

    #[test]
    fn fresh_projects_drawings_and_new_setups_default_to_model_relative_stock() {
        let mut app = AppController::new(machine());
        assert!(app.setups().is_empty());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert_eq!(app.document().setup().stock, default_stock());
        app.edit_stock(|stock| {
            *stock = Stock::FixedBox {
                size: [100.0; 3],
                model_position: [0.5; 3],
            }
        });
        let original = app.active_setup_id();
        let copy = app.duplicate_setup(original).unwrap();
        assert_eq!(app.document().setup().stock, Stock::PreviousSetup);
        assert!(matches!(app.setups()[0].stock, Stock::FixedBox { .. }));
        app.add_setup();
        assert_eq!(app.document().setup().stock, default_stock());
        app.activate_setup(copy).unwrap();
        assert_eq!(app.document().setup().stock, Stock::PreviousSetup);
        app.new_project();
        assert!(app.setups().is_empty());
        app.add_setup();
        assert_eq!(app.document().setup().stock, default_stock());
    }

    #[test]
    fn stock_auto_fits_the_part_and_offsets_grow_it() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let (min3, max3) = app.setup_model_bounds();
        let (bmin, bmax) = ([min3[0], min3[1]], [max3[0], max3[1]]);

        // Default stock extends one millimetre beyond each model face.
        let (min, max) = app.stock_box();
        assert_eq!(min, min3.map(|v| v - 1.0));
        assert_eq!(max, max3.map(|v| v + 1.0));

        // Grow it: 3 mm in X, 5 mm in Y, 25 mm thick from a top of 0.
        app.edit_stock(|s| {
            *s = Stock::BoundingBox {
                x_offset: 3.0,
                y_offset: 5.0,
                top: 0.0,
                thickness: 25.0,
            };
        });
        let (min, max) = app.stock_box();
        assert_eq!(min, [bmin[0] - 3.0, bmin[1] - 5.0, -25.0]);
        assert_eq!(max, [bmax[0] + 3.0, bmax[1] + 5.0, 0.0]);

        // Editing stock is one undoable step.
        assert!(app.undo());
        let (min, _) = app.stock_box();
        assert_eq!([min[0], min[1]], [bmin[0] - 1.0, bmin[1] - 1.0]);
    }

    #[test]
    fn exact_duplicates_are_detected_and_gated_by_exclusion() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        // Two distinct ops (outer profile + hole) — nothing duplicated yet.
        assert_eq!(op_ids(&app), vec![0, 1]);
        assert!(app.duplicate_operation_groups().is_empty());

        // Duplicate op 0: the copy is an exact twin of it.
        app.select(Selection::Operation(0));
        app.duplicate_selected_operation();
        assert_eq!(op_ids(&app), vec![0, 2, 1]);
        assert_eq!(
            app.duplicate_operation_groups(),
            vec![vec![0, 2]],
            "op 0 and its copy 2 are exact duplicates"
        );
        assert_eq!(app.duplicate_operation_ids(), BTreeSet::from([0, 2]));

        // Excluding one twin removes it from output, so the pair no longer clashes.
        app.set_operation_excluded(2, true);
        assert!(
            app.duplicate_operation_groups().is_empty(),
            "an excluded twin does not reach the post, so it is not flagged"
        );

        // Editing the copy so it differs also clears the flag.
        app.set_operation_excluded(2, false);
        app.select(Selection::Operation(2));
        app.edit_selected_operation(|op| {
            if let Operation::Profile(o) = op {
                o.feed = 999.0;
            }
        });
        assert!(
            app.duplicate_operation_groups().is_empty(),
            "different work"
        );
    }

    #[test]
    fn operation_names_persist_and_are_undoable() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.rename_operation(0, "  Roughing  ".into());
        assert_eq!(
            app.document().operation_names.get(&0).map(String::as_str),
            Some("Roughing")
        );
        let saved = serde_json::to_string(app.document()).unwrap();
        let restored: Document = serde_json::from_str(&saved).unwrap();
        assert_eq!(
            restored.operation_names.get(&0).map(String::as_str),
            Some("Roughing")
        );
        assert!(app.undo());
        assert!(!app.document().operation_names.contains_key(&0));
    }

    #[test]
    fn duplicate_delete_and_reorder_operations() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert_eq!(op_ids(&app), vec![0, 1]);

        // Duplicate op 0: the copy gets a fresh id, is inserted right after it,
        // and becomes the selection.
        app.select(Selection::Operation(0));
        app.duplicate_selected_operation();
        assert_eq!(op_ids(&app), vec![0, 2, 1]);
        assert_eq!(app.selection(), Selection::Operation(2));

        // Move the copy down past op 1, then back up.
        app.move_selected_operation(false);
        assert_eq!(op_ids(&app), vec![0, 1, 2]);
        app.move_selected_operation(true);
        assert_eq!(op_ids(&app), vec![0, 2, 1]);
        assert_eq!(
            app.selection(),
            Selection::Operation(2),
            "selection follows"
        );

        // Delete the copy; selection falls back to the first op.
        app.delete_selected_operation();
        assert_eq!(op_ids(&app), vec![0, 1]);
        assert_eq!(app.selection(), Selection::Operation(0));

        // All of it is undoable, one step at a time.
        assert!(app.undo()); // undo delete
        assert_eq!(op_ids(&app), vec![0, 2, 1]);
    }

    #[test]
    fn new_operation_adds_a_selected_profile() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert_eq!(op_ids(&app), vec![0, 1]);
        app.new_operation(OpKind::Profile);
        assert_eq!(op_ids(&app), vec![0, 1, 2]);
        assert_eq!(app.selection(), Selection::Operation(2));
        assert!(matches!(app.operation(2), Some(Operation::Profile(_))));
    }

    #[test]
    fn new_operation_builds_each_kind_from_geometry() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let (min3, max3) = app.setup_model_bounds();
        let (bmin, bmax) = ([min3[0], min3[1]], [max3[0], max3[1]]);

        // The convenience builder targets the first region's outer loop with no islands.
        app.new_operation(OpKind::Pocket);
        match app.selected_operation() {
            Some(Operation::Pocket(o)) => {
                assert!(o.islands.is_empty(), "no islands by default");
                assert_eq!(o.clear.lift_height, 0.2);
                assert_eq!(o.clear.lead_in, Lead::Arc { radius: 0.6 });
                assert_eq!(o.clear.lead_out, Lead::Arc { radius: 0.6 });
                assert!(
                    (0.0..1.0).contains(&o.clear.overlap),
                    "a sane default overlap"
                );
            }
            other => panic!("expected pocket, got {other:?}"),
        }

        app.new_operation(OpKind::Drill);
        match app.selected_operation() {
            // One drill point at the outer loop's centroid.
            Some(Operation::Drill(o)) => assert_eq!(o.points.len(), 1),
            other => panic!("expected drill, got {other:?}"),
        }

        app.new_operation(OpKind::Face);
        match app.selected_operation() {
            Some(Operation::Face(o)) => {
                // Boundary is the XY bounding rectangle of the geometry.
                let xs: Vec<f64> = o.boundary.points().iter().map(|p| p.x).collect();
                let ys: Vec<f64> = o.boundary.points().iter().map(|p| p.y).collect();
                let fmin = |v: &[f64]| v.iter().cloned().fold(f64::MAX, f64::min);
                let fmax = |v: &[f64]| v.iter().cloned().fold(f64::MIN, f64::max);
                assert!((fmin(&xs) - bmin[0]).abs() < 1e-9);
                assert!((fmax(&xs) - bmax[0]).abs() < 1e-9);
                assert!((fmin(&ys) - bmin[1]).abs() < 1e-9);
                assert!((fmax(&ys) - bmax[1]).abs() < 1e-9);
            }
            other => panic!("expected face, got {other:?}"),
        }
    }

    #[test]
    fn the_scene_draws_open_paths_as_well_as_regions() {
        // Regression: the viewport built its scene from regions alone, so an
        // imported stroke was invisible — the drawing looked like it had lost
        // geometry. Both the run-path scene (here) and the GUI's pre-run scene must
        // include open paths.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_WITH_STROKE_DXF, "part.dxf").unwrap();
        // The rectangle ring plus the V stroke: at least one strip must trace the
        // stroke's vertices, and it must NOT be closed back to its start.
        let stroke = app.open_paths()[0].points().to_vec();
        let out = app.run(&CancelToken::new());
        let first = [stroke[0].x as f32, stroke[0].y as f32, 0.0];
        let hit = out.scene.strips.iter().find(|s| {
            s.points.len() == stroke.len()
                && s.points[0][0] == first[0]
                && s.points[0][1] == first[1]
        });
        let hit = hit.expect("the open stroke must be in the scene");
        assert_ne!(
            hit.points.first(),
            hit.points.last(),
            "an open stroke must not be drawn closed"
        );
    }

    #[test]
    fn geometry_is_present_when_only_open_paths_were_imported() {
        // A drawing can legitimately be nothing but engravable strokes. Treating
        // "no regions" as "no geometry" would hide it and block op creation.
        const STROKES_ONLY: &str = "\
0\nSECTION\n2\nENTITIES\n\
0\nLINE\n10\n20.0\n20\n20.0\n11\n30.0\n21\n40.0\n\
0\nLINE\n10\n30.0\n20\n40.0\n11\n40.0\n21\n20.0\n\
0\nENDSEC\n0\nEOF\n";
        let mut app = AppController::new(machine());
        app.open_dxf(STROKES_ONLY, "strokes.dxf").unwrap();
        assert!(app.regions().is_empty(), "nothing closes");
        assert_eq!(app.open_paths().len(), 1);
        assert!(app.has_geometry(), "strokes are geometry");
        // And the creation wizard must be willing to start.
        app.begin_operation(OpKind::Engrave);
        assert!(app.pending_op().is_some(), "engraving must be startable");
    }

    #[test]
    fn an_open_chain_is_imported_and_kept_as_an_open_path() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_WITH_STROKE_DXF, "part.dxf").unwrap();
        assert_eq!(app.open_paths().len(), 1, "the V stroke stays open");
        assert_eq!(app.regions().len(), 1, "the rectangle still closes");
        // Three vertices, ends free.
        let pts = app.open_paths()[0].points();
        assert_eq!(pts.len(), 3, "{pts:?}");
        assert!(pts.first() != pts.last(), "an open path must not close");
    }

    #[test]
    fn an_open_path_engraves_as_an_open_stroke() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_WITH_STROKE_DXF, "part.dxf").unwrap();
        let op = app
            .build_op(OpKind::Engrave, LoopRef::open(0), &[], 1, None, None)
            .expect("engraving accepts an open path");
        match op {
            Operation::Engrave(o) => {
                assert!(!o.closed, "an imported stroke must engrave open");
                assert_eq!(o.chain.len(), 3);
            }
            other => panic!("expected engrave, got {other:?}"),
        }
    }

    #[test]
    fn a_picked_region_loop_still_engraves_closed() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_WITH_STROKE_DXF, "part.dxf").unwrap();
        let op = app
            .build_op(
                OpKind::Engrave,
                LoopRef {
                    region: 0,
                    part: LoopPart::Outer,
                },
                &[],
                1,
                None,
                None,
            )
            .expect("engraving accepts a closed loop too");
        match op {
            Operation::Engrave(o) => assert!(o.closed),
            other => panic!("expected engrave, got {other:?}"),
        }
    }

    #[test]
    fn an_open_path_cannot_become_a_closed_region_operation() {
        // Area strategies require a closed boundary.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_WITH_STROKE_DXF, "part.dxf").unwrap();
        for kind in [OpKind::Pocket, OpKind::Face, OpKind::Drill, OpKind::Thread] {
            assert!(
                app.build_op(kind, LoopRef::open(0), &[], 1, None, None)
                    .is_none(),
                "{kind:?} must refuse an open path"
            );
        }
    }

    #[test]
    fn open_paths_are_only_pickable_for_operations_that_accept_them() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_WITH_STROKE_DXF, "part.dxf").unwrap();
        // A point on the stroke, far from the rectangle.
        let on_stroke = [25.0, 30.0];
        assert!(!app.open_paths_pickable(), "no pending op, no open picking");

        app.begin_operation(OpKind::Profile);
        assert!(app.open_paths_pickable());
        let hit = app.nearest_loop_point(on_stroke, 2.0, false).unwrap();
        assert!(hit.0.is_open(), "profile can machine an open chain");
        let Operation::Profile(profile) = app
            .build_op(OpKind::Profile, hit.0, &[], 1, None, None)
            .unwrap()
        else {
            unreachable!()
        };
        assert!(!profile.closed);
        assert_eq!(profile.chain.points(), app.open_paths()[0].points());

        app.begin_operation(OpKind::Chamfer);
        assert!(app.open_paths_pickable());
        let Operation::Chamfer(chamfer) = app
            .build_op(OpKind::Chamfer, LoopRef::open(0), &[], 1, None, None)
            .unwrap()
        else {
            unreachable!()
        };
        assert!(!chamfer.closed);
        assert_eq!(chamfer.chain.points(), app.open_paths()[0].points());

        app.begin_operation(OpKind::Engrave);
        assert!(app.open_paths_pickable());
        let (l, _) = app
            .nearest_loop_point(on_stroke, 2.0, false)
            .expect("the stroke is pickable for engraving");
        assert!(l.is_open(), "expected the open stroke, got {l:?}");
    }

    #[test]
    fn open_paths_survive_a_project_round_trip() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_WITH_STROKE_DXF, "part.dxf").unwrap();
        let project = Project {
            view: None,
            toolpaths: Default::default(),
            schema_version: cam_model::SCHEMA_VERSION,
            document: app.document().clone(),
            regions: app.regions().to_vec(),
            open_paths: app.open_paths().to_vec(),
            defaults: *app.defaults(),
            source_name: app.source_name().to_string(),
            machine: None,
            post: None,
        };
        let json = project.to_json().unwrap();
        let back = Project::from_json(&json).unwrap();
        assert_eq!(back.open_paths, project.open_paths);
        assert_eq!(back.open_paths.len(), 1);
    }

    #[test]
    fn a_project_saved_before_open_paths_still_loads() {
        // #[serde(default)] back-compat: the field simply is not in older files.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let project = Project {
            view: None,
            toolpaths: Default::default(),
            schema_version: cam_model::SCHEMA_VERSION,
            document: app.document().clone(),
            regions: app.regions().to_vec(),
            open_paths: Vec::new(),
            defaults: *app.defaults(),
            source_name: app.source_name().to_string(),
            machine: None,
            post: None,
        };
        let mut v: serde_json::Value = serde_json::from_str(&project.to_json().unwrap()).unwrap();
        v.as_object_mut().unwrap().remove("open_paths");
        let back = Project::from_json(&v.to_string()).expect("legacy project loads");
        assert!(back.open_paths.is_empty());
    }

    #[test]
    fn reinitialize_replaces_in_place_keeping_id_and_position() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        // Three ops so position is observable.
        app.new_operation(OpKind::Face);
        let ids_before: Vec<u32> = op_ids(&app);
        assert!(ids_before.len() >= 3, "{ids_before:?}");
        let target = ids_before[1];
        let pos_before = ids_before.iter().position(|&x| x == target).unwrap();

        assert!(app.reinitialize_operation(target));
        assert_eq!(
            app.pending_op().unwrap().replacing,
            Some(target),
            "the wizard must know it is replacing"
        );
        // Completing the pick replaces rather than appends.
        let picked = LoopRef {
            region: 0,
            part: LoopPart::Outer,
        };
        let op = app
            .build_op(OpKind::Profile, picked, &[], 1, None, None)
            .unwrap();
        app.replace_operation(target, op);

        let ids_after = op_ids(&app);
        assert_eq!(
            ids_after.len(),
            ids_before.len(),
            "no operation added or lost"
        );
        assert_eq!(
            ids_after.iter().position(|&x| x == target),
            Some(pos_before),
            "the replacement keeps its place in the order"
        );
    }

    #[test]
    fn reinitialize_offers_the_same_operation_kind() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.new_operation(OpKind::Face);
        let id = match app.selection() {
            Selection::Operation(i) => i,
            other => panic!("{other:?}"),
        };
        assert!(app.reinitialize_operation(id));
        assert_eq!(app.pending_op().unwrap().kind, OpKind::Face);
    }

    #[test]
    fn an_operations_tool_can_be_changed_after_creation() {
        // The gap this closes: the wizard picks the tool before the geometry, so a
        // wrong pick previously meant deleting the op and re-picking its contour.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let id = match app.selection() {
            Selection::Operation(i) => i,
            other => panic!("{other:?}"),
        };
        let before = app.operation(id).unwrap().tool();
        let n = app.use_tool(Tool {
            number: 77,
            diameter: 4.0,
            flute_length: 15.0,
            length: 40.0,
            flutes: 3,
            kind: ToolKind::EndMill,
            ..Default::default()
        });
        assert_ne!(n, before);
        app.edit_operation(id, |op| op.set_tool(n));
        assert_eq!(app.operation(id).unwrap().tool(), n);
        // And it is undoable, like every other document edit.
        assert!(app.undo());
        assert_eq!(app.operation(id).unwrap().tool(), before);
    }

    #[test]
    fn diagnostics_name_the_operation_they_came_from() {
        // What lets the project tree mark *which* operation failed.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let id = match app.selection() {
            Selection::Operation(i) => i,
            other => panic!("{other:?}"),
        };
        // Cutter suitability is advisory; use an invalid numeric setting to
        // exercise a blocking diagnostic with operation context.
        let n = app.use_tool(Tool {
            number: 55,
            diameter: 6.0,
            flute_length: 12.0,
            length: 40.0,
            flutes: 2,
            kind: ToolKind::VBit {
                included_angle_deg: 60.0,
                tip_radius: 0.1,
            },
            ..Default::default()
        });
        app.edit_operation(id, |op| op.set_tool(n));
        app.edit_operation(id, |op| {
            if let Operation::Profile(profile) = op {
                profile.stepdown = 0.;
            }
        });
        let out = app.run(&CancelToken::new());
        let errors: Vec<&Diagnostic> = out
            .diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Error)
            .collect();
        assert!(!errors.is_empty(), "zero stepdown must error");
        assert!(
            errors.iter().all(|d| d.op == Some(id)),
            "every error must name its operation: {errors:?}"
        );
    }

    #[test]
    fn new_engrave_seeds_a_shallow_closed_groove() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.new_operation(OpKind::Engrave);
        match app.selected_operation() {
            Some(Operation::Engrave(o)) => {
                assert!(o.depth > 0.0, "a default engraving depth");
                assert!(o.depth < 1.0, "engraving is a surface mark, not a cut");
                assert!(o.closed, "a picked loop is a closed boundary");
                assert_eq!(o.stepdown, 0.0, "one pass at this depth");
                assert!(o.chain.len() >= 3);
            }
            other => panic!("expected engrave, got {other:?}"),
        }
    }

    #[test]
    fn new_chamfer_seeds_from_geometry() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.new_operation(OpKind::Chamfer);
        match app.selected_operation() {
            Some(Operation::Chamfer(o)) => {
                assert!(o.width > 0.0, "a default chamfer width");
                assert_eq!(
                    o.side,
                    Side::Outside,
                    "chamfers default to the outside edge"
                );
            }
            other => panic!("expected chamfer, got {other:?}"),
        }
    }

    #[test]
    fn new_thread_seeds_from_geometry_and_lowers_to_helical_arcs() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.new_operation(OpKind::Thread);

        // Thread milling requires an actual thread mill (guarded in the strategy —
        // an end mill would helix a plain groove, not a thread form). The starter
        // library holds only end mills, so put one in and point the op at it.
        let tm = app.use_tool(Tool {
            number: 9,
            diameter: 4.0,
            flute_length: 20.0,
            length: 50.0,
            flutes: 3,
            kind: ToolKind::ThreadMill { pitch: None },
            neck_diameter: 2.0,
            ..Default::default()
        });
        app.edit_selected_operation(|op| op.set_tool(tm));

        let id = match app.selection() {
            Selection::Operation(i) => i,
            other => panic!("expected an operation selection, got {other:?}"),
        };
        match app.selected_operation() {
            Some(Operation::Thread(o)) => {
                assert!(o.internal, "threads default to internal");
                assert!(o.major_dia > 0.0, "major diameter seeded from geometry");
                assert_eq!(o.points.len(), 1, "one thread at the loop centre");
            }
            other => panic!("expected thread, got {other:?}"),
        }

        // The whole pipeline (document → build_job → CL-data) must emit the
        // thread's cutting arcs, and their end Z must advance — a helix, not a
        // flat circle.
        let out = app.run(&CancelToken::new());
        let cut_zs: Vec<f64> = out
            .program
            .steps()
            .iter()
            .filter_map(|s| match s {
                cam_cldata::Step::Arc { end, tag, .. }
                    if tag.op_id == id && tag.kind == cam_cldata::MoveKind::Cutting =>
                {
                    Some(end.z)
                }
                _ => None,
            })
            .collect();
        assert!(cut_zs.len() >= 2, "thread should emit several cutting arcs");
        let zmin = cut_zs.iter().cloned().fold(f64::MAX, f64::min);
        let zmax = cut_zs.iter().cloned().fold(f64::MIN, f64::max);
        assert!(zmax - zmin > 1e-6, "cutting arcs must climb in Z (helix)");
    }

    #[test]
    fn excluding_an_operation_drops_it_from_the_run() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let full = app.run(&CancelToken::new()).program.steps().len();

        app.set_operation_excluded(0, true);
        assert!(app.is_operation_excluded(0));
        let reduced = app.run(&CancelToken::new()).program.steps().len();
        assert!(reduced < full, "excluded op should shorten the program");

        // The op is still in the tree, just not machined.
        assert_eq!(op_ids(&app), vec![0, 1]);
        // Re-including restores the full program.
        app.set_operation_excluded(0, false);
        assert_eq!(app.run(&CancelToken::new()).program.steps().len(), full);
    }

    #[test]
    fn structural_edits_no_op_without_an_operation_selected() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.select(Selection::Setup);
        app.duplicate_selected_operation();
        app.delete_selected_operation();
        app.move_selected_operation(true);
        assert_eq!(op_ids(&app), vec![0, 1], "no structural change");
    }

    #[test]
    fn editing_heights_is_undoable() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.edit_heights(|h| h.clearance = 12.0);
        assert_eq!(app.document().setup().heights.clearance, 12.0);
        assert!(app.undo());
        assert_eq!(app.document().setup().heights.clearance, 5.0);
    }

    #[test]
    fn selecting_a_missing_node_falls_back_to_setup() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.select(Selection::Operation(99));
        assert_eq!(app.selection(), Selection::Setup);
        app.select(Selection::Tool(7));
        assert_eq!(app.selection(), Selection::Setup);
        app.select(Selection::Tool(0));
        assert_eq!(app.selection(), Selection::Tool(0));
    }

    #[test]
    fn invalid_machine_prevents_planning_simulation_and_export() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.machine.max_feed = f64::NAN;
        let outcome = app.run(&CancelToken::new());
        assert_eq!(outcome.completion, JobCompletion::Failed);
        assert!(outcome.program.is_empty());
        assert!(outcome.stock_vertices.is_empty());
        assert!(outcome.has_errors());
        assert!(outcome.diagnostics[0].message.contains("maximum feed"));
        assert_eq!(app.export_nc(), Err(ExportError::HasErrors));
    }

    #[test]
    fn export_before_run_fails() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert_eq!(app.export_nc(), Err(ExportError::NothingToExport));
    }

    #[test]
    fn cancelled_runs_cannot_export_or_reuse_the_previous_cached_program() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert_eq!(
            app.run(&CancelToken::new()).completion,
            JobCompletion::Complete
        );
        let original = app.export_nc().unwrap().to_string();
        assert!(app.exported_nc().is_some());

        let token = CancelToken::new();
        token.cancel();
        let outcome = app.run(&token);
        assert_eq!(outcome.completion, JobCompletion::Cancelled);
        assert!(
            !outcome.has_errors(),
            "cancellation is separate from diagnostic errors"
        );
        assert!(app.exported_nc().is_none());
        assert_eq!(app.export_nc(), Err(ExportError::Cancelled));

        assert_eq!(
            app.run(&CancelToken::new()).completion,
            JobCompletion::Complete
        );
        assert_eq!(app.export_nc().unwrap(), original);
    }

    #[test]
    fn cancelled_file_export_neither_creates_nor_overwrites_a_program() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let token = CancelToken::new();
        token.cancel();
        app.run(&token);
        let path = temp_path("cancelled.nc");
        assert!(matches!(
            app.export_nc_to(&path),
            Err(ExportToError::Export(ExportError::Cancelled))
        ));
        assert!(!path.exists());

        std::fs::write(&path, "previous verified program").unwrap();
        assert!(matches!(
            app.export_nc_to(&path),
            Err(ExportToError::Export(ExportError::Cancelled))
        ));
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            "previous verified program"
        );
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn non_complete_previews_cannot_export_even_without_error_diagnostics() {
        let mut app = AppController::new(machine());
        app.set_block_export_on_verification_errors(false);
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let complete = app.run(&CancelToken::new()).clone();
        assert!(!complete.has_errors());
        let path = temp_path("incomplete.nc");
        std::fs::write(&path, "previous verified program").unwrap();

        for (completion, expected) in [
            (JobCompletion::Incomplete, ExportError::Incomplete),
            (JobCompletion::Cancelled, ExportError::Cancelled),
            (JobCompletion::Failed, ExportError::HasErrors),
        ] {
            // A full-looking preview is not proof of successful completion.
            app.outcome = Some(RunOutcome {
                completion,
                ..complete.clone()
            });
            assert_eq!(app.export_nc(), Err(expected));
            assert!(matches!(
                app.export_nc_to(&path),
                Err(ExportToError::Export(_))
            ));
            assert_eq!(
                std::fs::read_to_string(&path).unwrap(),
                "previous verified program"
            );
        }
        std::fs::remove_file(path).unwrap();
        assert_eq!(RunOutcome::default().completion, JobCompletion::Incomplete);
    }

    #[test]
    fn a_completed_empty_job_is_distinct_from_a_cancelled_empty_job() {
        let mut app = AppController::new(machine());
        app.add_setup();
        assert_eq!(
            app.run(&CancelToken::new()).completion,
            JobCompletion::Complete
        );
        assert!(app.export_nc().is_ok());
        let token = CancelToken::new();
        token.cancel();
        assert_eq!(app.run(&token).completion, JobCompletion::Cancelled);
        assert_eq!(app.export_nc(), Err(ExportError::Cancelled));
    }

    #[test]
    fn cancellation_after_a_completed_run_does_not_reclassify_its_result() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let token = CancelToken::new();
        assert_eq!(app.run(&token).completion, JobCompletion::Complete);
        token.cancel();
        assert!(
            app.export_nc().is_ok(),
            "completed results do not retain a live token"
        );
    }

    #[test]
    fn unsafe_heights_are_rejected_before_profile_motion() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert!(app.run(&CancelToken::new()).collisions.is_empty());
        app.edit_heights(|h| {
            h.clearance = -1.;
            h.retract = -1.;
        });
        let outcome = app.run(&CancelToken::new());
        assert!(
            outcome.has_errors(),
            "Invalid safety planes must prevent generation"
        );
        assert_ne!(outcome.completion, JobCompletion::Complete);
        assert!(
            app.export_nc().is_err(),
            "Invalid safety planes must block export"
        );
    }

    #[test]
    fn oversized_tool_reports_errors_and_blocks_export() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        // A ⌀12 tool cannot open the 10 mm hole.
        app.edit_tool(0, |t| t.diameter = 12.0);
        let outcome = app.run(&CancelToken::new());
        assert!(outcome.has_errors());
        assert_eq!(outcome.completion, JobCompletion::Failed);
        assert_eq!(app.export_nc(), Err(ExportError::HasErrors));
    }

    #[test]
    fn add_and_delete_tool_are_undoable() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert_eq!(app.document().setup().tools.len(), 1);

        app.add_tool();
        let tools = &app.document().setup().tools;
        assert_eq!(tools.len(), 2);
        // Numbered one past the highest existing tool, and selected.
        assert_eq!(tools[1].number, 2);
        assert_eq!(app.selection(), Selection::Tool(1));

        app.delete_tool(1);
        assert_eq!(app.document().setup().tools.len(), 1);

        // Both mutations are single undo steps.
        assert!(app.undo()); // undo delete
        assert_eq!(app.document().setup().tools.len(), 2);
        assert!(app.undo()); // undo add
        assert_eq!(app.document().setup().tools.len(), 1);
    }

    #[test]
    fn tool_length_round_trips_through_edit() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.edit_tool(0, |t| t.length = 42.0);
        assert_eq!(app.document().setup().tools[0].length, 42.0);
    }

    #[test]
    fn deleting_the_last_tool_selects_the_setup() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.delete_tool(0);
        assert!(app.document().setup().tools.is_empty());
        assert_eq!(app.selection(), Selection::Setup);
    }

    fn temp_path(name: &str) -> PathBuf {
        let mut p = std::env::temp_dir();
        p.push(format!(
            "ocam-test-{}-{}-{name}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        p
    }

    #[test]
    fn project_round_trips_through_disk() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.edit_tool(0, |t| t.length = 55.0);
        let doc_before = app.document().clone();
        let regions_before = app.regions().to_vec();

        let path = temp_path("proj.ocam");
        app.save_project(&path).unwrap();
        assert_eq!(app.current_path(), Some(path.as_path()));

        // Wipe, then reopen the saved file.
        app.new_project();
        assert!(app.regions().is_empty());
        app.open_project(&path).unwrap();

        assert_eq!(app.document(), &doc_before);
        assert_eq!(app.regions(), regions_before.as_slice());
        assert_eq!(app.current_path(), Some(path.as_path()));
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_project_records_the_machine_and_post_it_was_built_for_without_imposing_them() {
        // v11 made these round-trip *and be adopted*, reasoning that the envelope and
        // feed ceilings gate an export so the re-check must use the machine the job was
        // built for. The premise was right and the conclusion inverted: because they
        // gate the export, they must come from the machine that will **cut** it.
        // Superseded 2026-08-01 — recorded, reported, never applied.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.set_post_kind(PostKind::Okuma);
        app.edit_machine(|m| {
            m.name = "Okuma MB-46VAE".into();
            m.max_feed = 15_000.0;
            m.envelope = Envelope::new(
                Point3::new(0.0, 0.0, -400.0),
                Point3::new(762.0, 460.0, 0.0),
            );
        });
        let saved_machine = app.machine().clone();

        let path = temp_path("machine.ocam");
        app.save_project(&path).unwrap();

        // A fresh session on a different machine, posting somewhere else.
        let mut other = AppController::new(machine());
        other.set_post_kind(PostKind::Fanuc);
        let own_machine = other.machine().clone();
        other.open_project(&path).unwrap();

        assert_eq!(
            other.post_kind(),
            PostKind::Fanuc,
            "the local control stands"
        );
        assert_eq!(other.machine(), &own_machine, "and the local machine");

        // The file's answer survives as provenance, envelope included — it still has to
        // round-trip through the `[x, y, z]` on-disk form to be reportable.
        assert_eq!(other.provenance().machine.as_ref(), Some(&saved_machine));
        assert_eq!(other.provenance().post, Some(PostKind::Okuma));
        assert_eq!(
            other.provenance().machine.as_ref().unwrap().envelope.max.z,
            0.0,
            "the envelope round-trips through its [x, y, z] on-disk form"
        );
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_pre_v11_project_leaves_the_sessions_machine_and_post_alone() {
        // Now true of *every* project, not just those predating v11 — kept because a
        // file that records nothing is still the case most likely to be got wrong.
        // The half that is easy to get wrong. An older file records neither, and absence
        // must not be read as "use the default" — that would quietly reset a configured
        // machine to the built-in one every time an old job was opened, and silently
        // widen or narrow the limits an export is checked against.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let project = Project {
            view: None,
            toolpaths: Default::default(),
            schema_version: cam_model::SCHEMA_VERSION,
            document: app.document().clone(),
            regions: app.regions().to_vec(),
            open_paths: Vec::new(),
            defaults: *app.defaults(),
            source_name: app.source_name().to_string(),
            machine: None,
            post: None,
        };
        // Strip the fields entirely, the way a genuine pre-v11 file has them.
        let mut v: serde_json::Value =
            serde_json::from_str(&OcamFile::Project(project).to_json().unwrap()).unwrap();
        let obj = v.as_object_mut().unwrap();
        obj.remove("machine");
        obj.remove("post");
        let path = temp_path("pre_v11.ocam");
        std::fs::write(&path, v.to_string()).unwrap();

        let mut session = AppController::new(machine());
        session.edit_machine(|m| m.name = "the operator's own mill".into());
        session.set_post_kind(PostKind::LinuxCnc);
        session.open_project(&path).unwrap();

        assert_eq!(session.post_kind(), PostKind::LinuxCnc);
        assert_eq!(session.machine().name, "the operator's own mill");
        std::fs::remove_file(&path).ok();
    }

    /// The preference applies to a **new** project and to nothing else. Paired with
    /// `a_pre_v11_project_leaves_the_sessions_machine_and_post_alone`, these two pin
    /// Andreas's rule: "existing projects opened respect `.ocam`, we never retarget
    /// unless the user explicitly does so".
    #[test]
    fn neither_a_new_project_nor_an_opened_one_changes_the_control() {
        // The post is session state: which control you are cutting on is a property of
        // your shop, not of the job in front of you. So neither File ▸ New nor opening
        // someone's `.ocam` moves it. There was briefly a "default post for new
        // projects" preference; it was retired once machine and control went local,
        // because the machine library will carry the post and two sources of truth for
        // one question is one too many.
        let mut app = AppController::new(machine());
        app.set_post_kind(PostKind::Okuma);

        app.new_project();
        assert_eq!(
            app.post_kind(),
            PostKind::Okuma,
            "a new project keeps your control"
        );

        let path = temp_path("keeps_post.ocam");
        app.set_post_kind(PostKind::Fanuc);
        app.save_project(&path).unwrap();

        let mut session = AppController::new(machine());
        session.set_post_kind(PostKind::LinuxCnc);
        session.open_project(&path).unwrap();
        assert_eq!(
            session.post_kind(),
            PostKind::LinuxCnc,
            "and opening a job built elsewhere keeps yours"
        );
        std::fs::remove_file(&path).ok();
    }

    /// A blocked export must say something an operator can act on. There was no
    /// `Display` for `ExportError`, so the message fell back to `{e:?}` — a developer's
    /// view of the problem, shown to someone standing at a machine.
    #[test]
    fn a_blocked_export_explains_itself_in_words() {
        use cam_post::PostError;
        let cases = [
            (ExportError::NothingToExport, "run the job"),
            (ExportError::Cancelled, "cancelled"),
            (ExportError::Incomplete, "incomplete"),
            (ExportError::HasErrors, "⚠"),
            (ExportError::RapidThroughStock(3), "crash hazard"),
            (
                ExportError::Post(PostError::TravelExceeded {
                    axis: 'X',
                    span: 400.0,
                    travel: 300.0,
                }),
                "travel",
            ),
        ];
        for (e, expect) in cases {
            let said = e.to_string();
            assert!(
                said.contains(expect),
                "{e:?} reads {said:?}, which does not mention {expect:?}"
            );
            assert!(
                !said.contains('{') && !said.contains("Post("),
                "{said:?} is still a Debug rendering"
            );
        }
        // Plurality, because "1 rapid moves" is the kind of thing that makes a warning
        // look unconsidered.
        assert!(ExportError::RapidThroughStock(1)
            .to_string()
            .contains("1 rapid move would"));
        assert!(ExportError::RapidThroughStock(2)
            .to_string()
            .contains("2 rapid moves would"));
    }

    #[test]
    fn legacy_untagged_project_still_opens() {
        // A `.ocam` written before the `OcamFile` tag existed is a bare Project object.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let project = Project {
            view: None,
            toolpaths: Default::default(),
            schema_version: cam_model::SCHEMA_VERSION,
            document: app.document().clone(),
            regions: app.regions().to_vec(),
            open_paths: app.open_paths().to_vec(),
            defaults: *app.defaults(),
            source_name: app.source_name().to_string(),
            machine: None,
            post: None,
        };
        let path = temp_path("legacy.ocam");
        std::fs::write(&path, project.to_json().unwrap()).unwrap(); // untagged, on purpose

        app.new_project();
        app.open_project(&path)
            .expect("legacy untagged project must load");
        assert!(!app.regions().is_empty(), "geometry came back");
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn opening_a_tool_library_as_a_project_is_rejected() {
        use crate::tool_library::ToolLibrary;
        let mut app = AppController::new(machine());
        let path = temp_path("lib.ocam");
        let file = crate::project::OcamFile::Library(ToolLibrary::defaults());
        std::fs::write(&path, file.to_json().unwrap()).unwrap();
        assert!(matches!(
            app.open_project(&path),
            Err(ProjectError::NotAProject)
        ));
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn project_json_is_stable() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let project = Project {
            view: None,
            toolpaths: Default::default(),
            schema_version: cam_model::SCHEMA_VERSION,
            document: app.document().clone(),
            regions: app.regions().to_vec(),
            open_paths: app.open_paths().to_vec(),
            defaults: *app.defaults(),
            source_name: app.source_name().to_string(),
            machine: None,
            post: None,
        };
        let json = project.to_json().unwrap();
        assert_eq!(Project::from_json(&json).unwrap(), project);
    }

    #[test]
    fn import_cad_reads_a_dxf_file_via_acadrust() {
        let path = temp_path("part.dxf");
        std::fs::write(&path, PART_DXF).unwrap();
        let mut app = AppController::new(machine());
        let n = app.import_cad(&path).unwrap();
        assert!(n >= 1, "acadrust should read the rectangle+hole part");
        assert_eq!(
            app.source_name(),
            path.file_name().unwrap().to_str().unwrap()
        );
        // A real import brings in geometry only — no operations are fabricated
        // (unlike the bundled sample); selection falls back to the Setup.
        assert!(
            app.setups().is_empty(),
            "import must not auto-create setups"
        );
        assert_eq!(app.selection(), Selection::Setup);
        assert!(app.undo());
        assert!(app.regions().is_empty());
        assert!(app.redo());
        assert_eq!(app.regions().len(), n);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn use_tool_embeds_dedupes_and_reports_used() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap(); // sample: seeds tool T1 + 2 ops
        let base = app.document().setup().tools.len();

        // Embedding a brand-new tool adds it with a fresh number.
        let bit = Tool {
            number: 99, // library numbering is ignored on embed
            diameter: 4.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        };
        let n1 = app.use_tool(bit.clone());
        assert_eq!(app.document().setup().tools.len(), base + 1);
        assert_ne!(
            n1, 99,
            "embedded number is project-local, not the library's"
        );

        // Embedding the same geometry again reuses the number (no duplicate).
        let n2 = app.use_tool(bit);
        assert_eq!(n1, n2);
        assert_eq!(app.document().setup().tools.len(), base + 1);

        // used_tools reflects only tools referenced by operations. The freshly
        // embedded tool isn't used by any op yet, so it must not appear.
        assert!(app.used_tools().iter().all(|t| t.number != n1));
        // The sample's ops reference T1, which must appear.
        assert!(app.used_tools().iter().any(|t| t.number == 1));

        // Pruning drops the unreferenced embedded tool.
        app.prune_unused_tools();
        assert_eq!(app.document().setup().tools.len(), base);
        assert!(app.document().setup().tools.iter().all(|t| t.number != n1));
    }

    #[test]
    fn nearest_loop_picks_edges_not_areas() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        // The part is a 10..70 × 10..50 rectangle (Outer) with a ⌀10 hole at (40,30)
        // (Hole 0). Near the left edge → the outer loop.
        assert_eq!(
            app.nearest_loop([10.4, 25.0], 2.0).map(|(l, _)| l),
            Some(LoopRef {
                region: 0,
                part: LoopPart::Outer
            })
        );
        // Near the circle edge → the hole.
        assert_eq!(
            app.nearest_loop([45.4, 30.0], 2.0).map(|(l, _)| l),
            Some(LoopRef {
                region: 0,
                part: LoopPart::Hole(0)
            })
        );
        // Inside the plate but far from every edge → nothing (this is line picking).
        assert_eq!(app.nearest_loop([30.0, 25.0], 2.0), None);
    }

    #[test]
    fn picking_a_loop_creates_a_profile_with_the_start_at_the_picked_vertex() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.add_tool(); // tool number 2
        app.begin_operation(OpKind::Profile);
        app.set_pending_tool(2);

        // A click far from any line misses; the wizard stays active.
        assert_eq!(
            app.pick_operation_geometry([30.0, 25.0], 2.0, &[SnapKind::End]),
            PickResult::Missed
        );
        assert!(app.pending_op().is_some());

        // Near the (70,50) corner selects the outer loop — but nothing is created
        // until Confirm, so the wizard stays live and the pick can still be redone.
        assert_eq!(
            app.pick_operation_geometry([69.6, 49.6], 2.0, &[SnapKind::End]),
            PickResult::Selecting
        );
        assert!(app.pending_op().is_some(), "still pending until Confirm");
        assert!(app.pending_ready(), "tool + boundary are both set");
        assert!(app.confirm_operation(None));
        assert!(app.pending_op().is_none());
        match app.selected_operation() {
            Some(Operation::Profile(o)) => {
                assert_eq!(o.tool, 2);
                assert_eq!(o.start, Some([70.0, 50.0]));
                assert_eq!(o.side, Side::Outside);
            }
            other => panic!("expected a profile, got {other:?}"),
        }
    }

    #[test]
    fn clicking_the_inner_circle_profiles_the_circle_not_the_rectangle() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.begin_operation(OpKind::Profile);
        app.set_pending_tool(1);
        // Click on the circle edge (centre 40,30, r5) → the profile follows the hole.
        assert_eq!(
            app.pick_operation_geometry([45.4, 30.0], 2.0, &[SnapKind::End]),
            PickResult::Selecting
        );
        assert!(app.confirm_operation(None));
        let hole = app.regions()[0].holes()[0].clone();
        match app.selected_operation() {
            Some(Operation::Profile(o)) => assert_eq!(o.chain, hole, "chain is the circle"),
            other => panic!("expected a profile on the circle, got {other:?}"),
        }
    }

    #[test]
    fn nothing_is_created_until_confirm_whatever_the_order() {
        // The annoyance this fixes: picking geometry first used to create the
        // operation immediately with a default tool. Now either order works and the
        // choice stays editable until Confirm.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let before = op_ids(&app).len();

        // Geometry FIRST, no tool yet.
        app.begin_operation(OpKind::Profile);
        app.pick_operation_geometry([69.6, 49.6], 2.0, &[SnapKind::End]);
        assert!(app.pending_op().unwrap().boundary.is_some());
        assert!(app.pending_op().unwrap().tool.is_none());
        assert!(!app.pending_ready(), "no tool yet");
        assert!(!app.confirm_operation(None), "must refuse without a tool");
        assert_eq!(op_ids(&app).len(), before, "nothing created");

        // Tool second → now ready.
        app.set_pending_tool(1);
        assert!(app.pending_ready());
        assert!(app.confirm_operation(None));
        assert_eq!(op_ids(&app).len(), before + 1);
    }

    #[test]
    fn the_geometry_pick_can_be_changed_before_confirm() {
        // Re-picking moves the boundary rather than being ignored or treated as a
        // second selection.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.begin_operation(OpKind::Profile);
        app.set_pending_tool(1);
        app.pick_operation_geometry([10.4, 25.0], 2.0, &[SnapKind::End]);
        let first = app.pending_op().unwrap().boundary.unwrap();
        assert_eq!(first.part, LoopPart::Outer);
        // Now click the circle instead.
        app.pick_operation_geometry([45.4, 30.0], 2.0, &[SnapKind::End]);
        let second = app.pending_op().unwrap().boundary.unwrap();
        assert_ne!(second, first, "the pick must move, not stick");
        assert!(app.confirm_operation(None));
    }

    #[test]
    fn a_tool_alone_is_not_enough_to_confirm() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.begin_operation(OpKind::Profile);
        app.set_pending_tool(1);
        assert!(!app.pending_ready(), "no geometry yet");
        assert!(!app.confirm_operation(None));
    }

    #[test]
    fn pocket_wizard_picks_a_boundary_then_toggles_islands() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.begin_operation(OpKind::Pocket);
        app.set_pending_tool(1);

        // First pick = the boundary (outer). Stays in island mode.
        assert_eq!(
            app.pick_operation_geometry([10.4, 25.0], 2.0, &[SnapKind::End]),
            PickResult::Selecting
        );
        let pending = app.pending_op().unwrap();
        assert_eq!(pending.boundary.unwrap().part, LoopPart::Outer);
        assert!(pending.islands.is_empty());

        // Click the circle → adds it as an island; clicking again removes it.
        app.pick_operation_geometry([45.4, 30.0], 2.0, &[SnapKind::End]);
        assert_eq!(app.pending_op().unwrap().islands.len(), 1);
        app.pick_operation_geometry([45.4, 30.0], 2.0, &[SnapKind::End]);
        assert!(app.pending_op().unwrap().islands.is_empty());

        // Re-add it and confirm → a pocket bounded by the rectangle with one island.
        app.pick_operation_geometry([45.4, 30.0], 2.0, &[SnapKind::End]);
        assert!(app.confirm_operation(None));
        assert!(app.pending_op().is_none());
        match app.selected_operation() {
            Some(Operation::Pocket(o)) => assert_eq!(o.islands.len(), 1),
            other => panic!("expected a pocket with one island, got {other:?}"),
        }
    }

    #[test]
    fn a_carve_auto_includes_the_regions_holes_as_islands() {
        // A letter's counter must be left standing, and a counter is exactly a hole of
        // the picked region. Making the operator click each one is busywork the
        // geometry already answers -- but they stay togglable, as a pocket's are.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.begin_operation(OpKind::Carve);
        app.set_pending_tool(1);
        app.pick_operation_geometry([10.4, 25.0], 2.0, &[SnapKind::End]);
        let pending = app.pending_op().unwrap();
        assert_eq!(pending.boundary.unwrap().part, LoopPart::Outer);
        assert_eq!(
            pending.islands,
            vec![LoopRef::hole(0, 0)],
            "the circle should be an island without a second click"
        );
        // And it can still be toggled off.
        app.pick_operation_geometry([45.4, 30.0], 2.0, &[SnapKind::End]);
        assert!(app.pending_op().unwrap().islands.is_empty());

        // Re-add and confirm: the carve carries the island through.
        app.pick_operation_geometry([45.4, 30.0], 2.0, &[SnapKind::End]);
        assert!(app.confirm_operation(None));
        match app.selected_operation() {
            Some(Operation::Carve(o)) => assert_eq!(o.islands.len(), 1),
            other => panic!("expected a carve with one island, got {other:?}"),
        }
    }

    #[test]
    fn a_pocket_still_starts_with_no_islands() {
        // The auto-include is carve-only: a pocket's islands are a deliberate choice
        // about what to leave standing, not a property of the drawing.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.begin_operation(OpKind::Pocket);
        app.set_pending_tool(1);
        app.pick_operation_geometry([10.4, 25.0], 2.0, &[SnapKind::End]);
        assert!(app.pending_op().unwrap().islands.is_empty());
    }

    #[test]
    fn a_carves_clearing_tool_counts_as_in_use_and_survives_pruning() {
        // The bug this pins: counting only the defining tool would show an incomplete
        // setup sheet, and -- far worse -- prune the clearing end mill out of the
        // document, leaving the operation referencing a tool that is no longer there.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let mill = Tool {
            number: 7,
            diameter: 4.0,
            length: 30.0,
            flute_length: 20.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        };
        app.begin_operation(OpKind::Carve);
        app.set_pending_tool(1);
        app.pick_operation_geometry([10.4, 25.0], 2.0, &[SnapKind::End]);
        assert!(app.confirm_operation(None));
        let id = match app.selection() {
            Selection::Operation(id) => id,
            other => panic!("expected the carve selected, got {other:?}"),
        };
        // Embedded after creation, exactly as the inspector's "Clear flat areas" does:
        // confirming an operation prunes unreferenced tools, so a tool embedded before
        // the carve exists would be swept away again.
        let number = app.use_tool(mill);
        app.edit_operation(id, |op| {
            if let Operation::Carve(c) = op {
                c.clear = Some(cam_model::CarveClearing {
                    tool: number,
                    params: cam_model::ClearParams::default(),
                });
            }
        });
        assert!(
            app.used_tools().iter().any(|t| t.number == number),
            "the clearing tool belongs on the setup sheet"
        );
        app.prune_unused_tools();
        assert!(
            app.document()
                .setup()
                .tools
                .iter()
                .any(|t| t.number == number),
            "pruning must not drop a tool an operation still references"
        );
    }

    #[test]
    fn snap_at_resolves_end_mid_and_nearest_by_priority() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let close =
            |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).abs() < 0.05 && (a[1] - b[1]).abs() < 0.05;

        // End near the (70,50) corner.
        let h = app.snap_at([69.4, 49.4], 1.5, &[SnapKind::End]).unwrap();
        assert_eq!(h.kind, SnapKind::End);
        assert!(close(h.point, [70.0, 50.0]), "end {:?}", h.point);

        // Mid of the right edge (70,10)–(70,50) → (70,30).
        let h = app.snap_at([69.5, 30.0], 1.5, &[SnapKind::Mid]).unwrap();
        assert_eq!(h.kind, SnapKind::Mid);
        assert!(close(h.point, [70.0, 30.0]), "mid {:?}", h.point);

        // Nearest on the right edge, away from corner and mid.
        let h = app
            .snap_at([69.6, 20.0], 1.5, &[SnapKind::Nearest])
            .unwrap();
        assert_eq!(h.kind, SnapKind::Nearest);
        assert!(close(h.point, [70.0, 20.0]), "nearest {:?}", h.point);

        // Priority: at a corner with all on, End beats Mid/Nearest.
        let h = app
            .snap_at(
                [69.6, 49.6],
                1.5,
                &[SnapKind::End, SnapKind::Mid, SnapKind::Nearest],
            )
            .unwrap();
        assert_eq!(h.kind, SnapKind::End);

        // Mid-edge with only End enabled ⇒ nothing catches (the pick then falls
        // back to the nearest point on its own).
        assert!(app.snap_at([70.0, 20.0], 1.5, &[SnapKind::End]).is_none());

        // The circle has no corners ⇒ End finds nothing there.
        assert!(app.snap_at([45.0, 30.0], 1.5, &[SnapKind::End]).is_none());
    }

    #[test]
    fn quadrant_snap_finds_a_circle_cardinal_but_not_a_polygon() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        // The hole (centre 40,30, r5) → +X quadrant at (45,30).
        let h = app
            .snap_at([44.6, 30.0], 1.5, &[SnapKind::Quadrant])
            .unwrap();
        assert_eq!(h.kind, SnapKind::Quadrant);
        assert!(
            (h.point[0] - 45.0).abs() < 0.1 && (h.point[1] - 30.0).abs() < 0.1,
            "quadrant {:?}",
            h.point
        );
        // The rectangle has corners ⇒ it is not treated as a circle: no quadrant.
        assert!(app
            .snap_at([69.5, 30.0], 1.5, &[SnapKind::Quadrant])
            .is_none());
    }

    #[test]
    fn drill_only_selects_circular_holes() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.begin_operation(OpKind::Drill);
        app.set_pending_tool(1);
        // The rectangle edge is not a hole ⇒ a drill pick there misses.
        assert_eq!(
            app.pick_operation_geometry([10.4, 25.0], 2.0, &[]),
            PickResult::Missed
        );
        assert!(app.pending_op().is_some(), "still awaiting a hole");
        // The circle hole is selectable ⇒ a drill at its centre.
        assert_eq!(
            app.pick_operation_geometry([45.4, 30.0], 2.0, &[]),
            PickResult::Selecting
        );
        assert!(app.confirm_operation(None));
        match app.selected_operation() {
            Some(Operation::Drill(o)) => {
                assert_eq!(o.points.len(), 1);
                let c = o.points[0];
                assert!(
                    (c[0] - 40.0).abs() < 0.1 && (c[1] - 30.0).abs() < 0.1,
                    "centre {c:?}"
                );
            }
            other => panic!("expected a drill, got {other:?}"),
        }
    }

    #[test]
    fn adding_origins_grows_the_index_list_and_activates_the_new_one() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert_eq!(app.origin_indices(), vec![1], "starts with the base origin");
        app.add_origin();
        assert_eq!(app.origin_indices(), vec![1, 2], "second origin present");
        assert_eq!(app.active_origin(), 2, "new origin becomes active");
        app.add_origin();
        assert_eq!(app.origin_indices(), vec![1, 2, 3]);
        app.delete_origin(2);
        assert_eq!(
            app.origin_indices(),
            vec![1, 3],
            "delete removes just that origin"
        );
    }

    #[test]
    fn reassigning_an_operation_moves_it_between_origin_groups_and_reaches_the_gcode() {
        // Multi-datum v1 shipped with no way across groups: a new op joined the active
        // origin and the only correction was delete-and-recreate, which loses the op's
        // parameters. `set_operation_origin` existed but nothing called it.
        //
        // Asserted through to the *posted words*, not just the model field, because that
        // is the claim an operator cares about — the op runs under the other fixture's
        // work offset. The model field agreeing while the emitted program did not would
        // be exactly the kind of near-miss that reaches a machine.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.set_post_kind(PostKind::Fanuc);
        // One of the operations the import seeded, so it carries real geometry and
        // actually posts. A freshly created op is pending until its geometry is picked
        // and would export nothing to inspect.
        let id = app.document().setup().operations[0].id();
        let offset_of = |app: &AppController, id: u32| {
            app.document()
                .setup()
                .operations
                .iter()
                .find(|o| o.id() == id)
                .expect("the operation is still there")
                .work_offset()
        };
        assert_eq!(offset_of(&app, id), 1, "starts on the base origin");

        app.add_origin(); // origin 2, active
        app.edit_active_origin(|o| *o = [60.0, 0.0, 0.0]);
        app.set_operation_origin(id, 2);
        assert_eq!(
            offset_of(&app, id),
            2,
            "the operation joined the second origin's group"
        );

        app.run(&CancelToken::new());
        let nc = app.export_nc().expect("posts").to_string();
        assert!(
            nc.contains("G55"),
            "the op should now post under the second work offset:\n{nc}"
        );

        // ...and back again, which must leave no trace of the detour: a single-datum
        // job states only the preamble's own offset.
        app.set_operation_origin(id, 1);
        app.run(&CancelToken::new());
        let back = app.export_nc().expect("posts").to_string();
        assert!(
            !back.contains("G55"),
            "moving back should leave a plain single-datum program:\n{back}"
        );
    }

    #[test]
    fn reassigning_an_operation_is_one_undoable_step() {
        // It goes through `document.edit`, so Ctrl-Z restores the previous group rather
        // than leaving the op stranded under an origin the operator was undoing away.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.new_operation(OpKind::Profile);
        let id = app.selected_operation().expect("an operation").id();
        let offset_of = |app: &AppController, id: u32| {
            app.document()
                .setup()
                .operations
                .iter()
                .find(|o| o.id() == id)
                .expect("the operation is still there")
                .work_offset()
        };
        app.add_origin();
        app.set_operation_origin(id, 2);
        assert_eq!(offset_of(&app, id), 2);
        app.undo();
        assert_eq!(
            offset_of(&app, id),
            1,
            "undo put the operation back in its original group"
        );
    }

    #[test]
    fn setting_an_origin_index_to_an_existing_one_swaps_them() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.add_origin(); // H2, active
        assert_eq!(app.origin_indices(), vec![1, 2]);
        // Renumber the base (H1) to H2 -> the two swap: base becomes H2, extra becomes H1.
        app.select_origin(1);
        app.set_active_origin_index(2);
        assert_eq!(app.base_origin_index(), 2, "base took the new index");
        assert_eq!(app.active_origin(), 2, "the edited origin is now H2");
        let mut idx = app.origin_indices();
        idx.sort_unstable();
        assert_eq!(idx, vec![1, 2], "both indices still present, just swapped");
        // A plain renumber to an unused index (no swap).
        app.set_active_origin_index(5);
        assert_eq!(app.base_origin_index(), 5);
        idx = app.origin_indices();
        idx.sort_unstable();
        assert_eq!(idx, vec![1, 5]);
    }

    #[test]
    fn picking_with_mid_snap_starts_the_profile_at_the_edge_midpoint() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.begin_operation(OpKind::Profile);
        app.set_pending_tool(1);
        assert_eq!(
            app.pick_operation_geometry([69.5, 30.0], 1.5, &[SnapKind::Mid]),
            PickResult::Selecting
        );
        assert!(app.confirm_operation(None));
        match app.selected_operation() {
            Some(Operation::Profile(o)) => {
                let s = o.start.expect("mid snap sets a start");
                assert!(
                    (s[0] - 70.0).abs() < 0.05 && (s[1] - 30.0).abs() < 0.05,
                    "start {s:?}"
                );
            }
            other => panic!("expected a profile, got {other:?}"),
        }
    }

    #[test]
    fn begin_operation_needs_geometry_not_tools() {
        // With no geometry loaded, beginning an op is a no-op.
        let mut app = AppController::new(machine());
        app.begin_operation(OpKind::Profile);
        assert!(app.pending_op().is_none(), "no geometry ⇒ no wizard");

        // With geometry but no embedded tools, the wizard still starts — the tool is
        // picked from the library during setup.
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.delete_tool(0); // strip the sample's seeded tool
        assert!(app.document().setup().tools.is_empty());
        app.begin_operation(OpKind::Profile);
        assert!(app.pending_op().is_some(), "no tool is fine now");
    }

    #[test]
    fn the_datum_label_follows_the_selected_post() {
        // The tree names an origin by what the *chosen* post will call it, so switching
        // post relabels every origin. Two different number spaces, not one renaming:
        // Okuma takes the index literally, the ISO families count up from `G54`.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.add_origin(); // origin 2
        app.set_post_kind(PostKind::Fanuc);
        assert_eq!(app.datum_label(1).as_deref(), Some("G54"));
        assert_eq!(app.datum_label(2).as_deref(), Some("G55"));
        app.set_post_kind(PostKind::Okuma);
        assert_eq!(app.datum_label(1).as_deref(), Some("H1"));
        assert_eq!(app.datum_label(2).as_deref(), Some("H2"));
    }

    #[test]
    fn an_origin_the_iso_posts_cannot_express_has_no_label() {
        // A seventh fixture: `G54`-`G59` runs out, so the tree can mark it before the
        // job is built rather than leaving it to fail at export. Okuma, whose `H<n>` has
        // no such ceiling, still names it — which is the whole reason the label is the
        // post's to give.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        for _ in 0..6 {
            app.add_origin(); // origins 2..=7
        }
        app.set_post_kind(PostKind::Haas);
        assert_eq!(
            app.datum_label(6).as_deref(),
            Some("G59"),
            "the last ISO offset"
        );
        assert_eq!(app.datum_label(7), None, "and nothing past it");
        app.set_post_kind(PostKind::Okuma);
        assert_eq!(app.datum_label(7).as_deref(), Some("H7"));
    }
}
