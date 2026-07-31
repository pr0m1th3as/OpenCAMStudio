//! The headless application controller.
//!
//! All of the app's behaviour lives here, with no GUI dependency: open a DXF,
//! hold an editable [`Document`] with a [`Selection`], adjust the selected node
//! (undoably), run the strategies, build the viewport scene, simulate the stock,
//! and export G-code. The iced shell is a thin view over this — so the app's
//! logic is unit-tested exactly like the rest of the pipeline.

use std::borrow::Cow;
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use cam_geo::{Contour, Point, Polygon, Polyline};
use cam_import::{read_cad_file, read_dxf_str, ImportError, ImportOptions};

use crate::project::{OcamFile, Project};
use cam_model::{
    CarveOp, EngraveOp,
    reconcile_tool_numbers, Axis, ChamferOp, Comp, Document, DrillOp, FaceOp, Hand, Heights,
    History, Lead, Machine, Operation, Origin, Plunge, PocketOp, ProfileOp, ReconcileReport, Setup,
    Side, Stock, ThreadOp, Tool, ToolKind,
};
use cam_post::{PostError, PostKind, PostOptions};
use cam_render::{mesh_vertices, MeshVertex, Scene, PART};
use cam_sim::{simulate, Collision, CollisionKind, ProfileShape, SimOptions, SimTool, ToolProfile};

use cam_cldata::{Program, SpindleDir};
use cam_toolpath::{build_job, CancelToken, Diagnostic, Severity};

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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpKind {
    Profile,
    Pocket,
    Drill,
    Face,
    Chamfer,
    Thread,
    Engrave,
    Carve,
}

/// Whether an operation kind restricts geometry selection to **circular** loops.
/// Drilling and thread-milling target holes (the pick gives the centre, and for
/// threads the diameter), so a rectangle or open edge is not a valid pick.
pub fn op_selects_circles(kind: OpKind) -> bool {
    matches!(kind, OpKind::Drill | OpKind::Thread)
}

/// Which node of the document is currently selected — what the tree highlights
/// and the inspector edits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
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
    pub program: Program,
    pub diagnostics: Vec<Diagnostic>,
    pub scene: Scene,
    /// Interleaved position+normal vertices of the cut stock surface.
    pub stock_vertices: Vec<MeshVertex>,
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
    /// The last run produced errors, so its G-code is unsafe to emit.
    HasErrors,
    /// The simulation found `n` rapid(s) plowing through remaining stock — a
    /// machine-crash hazard, blocked by default. (A future preference could let
    /// the user downgrade this to a warning.)
    RapidThroughStock(usize),
    /// The post rejected the program.
    Post(PostError),
}

impl From<PostError> for ExportError {
    fn from(e: PostError) -> Self {
        ExportError::Post(e)
    }
}

/// The application state and operations, GUI-agnostic.
pub struct AppController {
    machine: Machine,
    /// The selected post/controller dialect for export.
    post_kind: PostKind,
    regions: Vec<Polygon>,
    /// Open imported chains the importer could not close — engravable strokes.
    open_paths: Vec<Polyline>,
    document: History<Document>,
    defaults: JobParams,
    selection: Selection,
    /// Operation ids excluded from toolpath generation (kept in the tree).
    excluded: BTreeSet<u32>,
    /// The origin new operations are assigned to, and the one the origin inspector /
    /// pick flow edits. Defaults to the base origin (index 1).
    active_origin: u32,
    /// Origin indices whose operations are frozen out of the run (kept in the tree) —
    /// the group-level analogue of [`excluded`], for working one orientation at a time.
    disabled_origins: BTreeSet<u32>,
    source_name: String,
    /// The `.ocam` path this project was last saved to / opened from.
    current_path: Option<PathBuf>,
    /// An operation being created, awaiting a geometry pick in the viewport.
    pending_op: Option<PendingOp>,
    outcome: Option<RunOutcome>,
    nc: Option<String>,
}

/// Which closed loop of an imported region a pick refers to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoopRef {
    /// Index into the regions — or, when `part` is [`LoopPart::Open`], into the
    /// open paths.
    pub region: usize,
    pub part: LoopPart,
}

impl LoopRef {
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

/// Whether an operation kind can machine an **open** path. Only engraving can: every
/// other strategy needs a closed region to offset, clear, or bound.
pub fn op_accepts_open_paths(kind: OpKind) -> bool {
    matches!(kind, OpKind::Engrave)
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
#[derive(Clone, Copy, Debug, PartialEq)]
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
#[derive(Clone, Debug, PartialEq)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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
    fn priority(self) -> u8 {
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
    /// The `.ocam` opened as a project is actually a **tool library**.
    NotAProject,
}

impl std::fmt::Display for ProjectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProjectError::Io(e) => write!(f, "file error: {e}"),
            ProjectError::Json(e) => write!(f, "project format error: {e}"),
            ProjectError::NotAProject => {
                write!(f, "this .ocam file is a tool library, not a project")
            }
        }
    }
}

impl std::error::Error for ProjectError {}

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
            ExportToError::Export(e) => write!(f, "{e:?}"),
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
        Self {
            machine,
            post_kind: PostKind::default(),
            regions: Vec::new(),
            open_paths: Vec::new(),
            document: History::new(empty_document(&defaults)),
            defaults,
            selection: Selection::default(),
            excluded: BTreeSet::new(),
            active_origin: 1,
            disabled_origins: BTreeSet::new(),
            source_name: String::new(),
            current_path: None,
            pending_op: None,
            outcome: None,
            nc: None,
        }
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
        self.post_kind = kind;
        self.nc = None;
    }

    /// The options every export posts with. One construction, so anything that needs
    /// to *predict* the output (see [`datum_label`](Self::datum_label)) reads the same
    /// settings the file is actually written from.
    fn post_options(&self) -> PostOptions {
        PostOptions {
            program_name: Some(self.program_name()),
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

    /// Edit the machine (envelope/name/limits). No re-run needed: the backplot is
    /// independent of the machine, and `export_nc` always re-posts against the
    /// current machine, so limit changes are re-checked at the next export.
    pub fn edit_machine(&mut self, f: impl FnOnce(&mut Machine)) {
        f(&mut self.machine);
    }

    /// The current document.
    pub fn document(&self) -> &Document {
        self.document.current()
    }

    /// The current setup (shorthand for `document().setup`).
    fn setup(&self) -> &Setup {
        &self.document.current().setup
    }

    /// The seed defaults used when a document is generated on import.
    pub fn defaults(&self) -> &JobParams {
        &self.defaults
    }

    /// The current selection.
    pub fn selection(&self) -> Selection {
        self.selection
    }

    /// Select a node. Clamped to a valid target: an out-of-range tool or a
    /// missing operation falls back to the setup.
    pub fn select(&mut self, selection: Selection) {
        self.selection = match selection {
            Selection::Tool(i) if i < self.document.current().setup.tools.len() => selection,
            Selection::Operation(id) if self.operation(id).is_some() => selection,
            Selection::Setup | Selection::Origin | Selection::Stock | Selection::Machine => {
                selection
            }
            _ => Selection::Setup,
        };
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
        !self.regions.is_empty() || !self.open_paths.is_empty()
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
        self.document
            .current()
            .setup
            .operations
            .iter()
            .find(|o| o.id() == id)
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
        let path = path.as_ref();
        let import = read_cad_file(path, &ImportOptions::default())?;
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("imported")
            .to_string();
        self.install_import(import.regions, import.open_chains, name, false);
        Ok(self.regions.len())
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
        let document = self.generate_document(seed_ops);
        self.document = History::new(document);
        self.excluded.clear();
        self.disabled_origins.clear();
        self.active_origin = self.document.current().setup.origin_index;
        self.pending_op = None;
        self.current_path = None;
        self.selection = self
            .document
            .current()
            .setup
            .operations
            .first()
            .map(|op| Selection::Operation(op.id()))
            .unwrap_or(Selection::Setup);
        self.invalidate();
    }

    /// Reset to a fresh, empty "Untitled" project.
    pub fn new_project(&mut self) {
        self.regions.clear();
        self.open_paths.clear();
        self.document = History::new(empty_document(&self.defaults));
        self.excluded.clear();
        self.disabled_origins.clear();
        self.active_origin = self.document.current().setup.origin_index;
        self.source_name.clear();
        self.current_path = None;
        self.pending_op = None;
        self.selection = Selection::Setup;
        self.invalidate();
    }

    /// The `.ocam` file this project was last saved to / opened from, if any.
    pub fn current_path(&self) -> Option<&Path> {
        self.current_path.as_deref()
    }

    /// Save the current project (document + geometry + defaults) to `path` as JSON,
    /// and remember it as the current path.
    pub fn save_project(&mut self, path: impl AsRef<Path>) -> Result<(), ProjectError> {
        let project = Project {
            schema_version: cam_model::SCHEMA_VERSION,
            document: self.document.current().clone(),
            regions: self.regions.clone(),
            open_paths: self.open_paths.clone(),
            defaults: self.defaults,
            source_name: self.source_name.clone(),
        };
        // Written through the tagged `.ocam` union (§3.1) so the file self-describes.
        let json = OcamFile::Project(project)
            .to_json()
            .map_err(|e| ProjectError::Json(e.to_string()))?;
        let path = path.as_ref();
        std::fs::write(path, json).map_err(|e| ProjectError::Io(e.to_string()))?;
        self.current_path = Some(path.to_path_buf());
        Ok(())
    }

    /// Open a `.ocam` project from `path`, replacing the current session.
    pub fn open_project(&mut self, path: impl AsRef<Path>) -> Result<(), ProjectError> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|e| ProjectError::Io(e.to_string()))?;
        // Route through the tagged union; a legacy untagged project still parses (§3.1).
        let project = match OcamFile::from_json(&text).map_err(|e| ProjectError::Json(e.to_string()))?
        {
            OcamFile::Project(p) => p,
            OcamFile::Library(_) => return Err(ProjectError::NotAProject),
        };
        self.regions = project.regions;
        self.open_paths = project.open_paths;
        self.defaults = project.defaults;
        self.source_name = project.source_name;
        self.document = History::new(project.document);
        self.excluded.clear();
        self.disabled_origins.clear();
        self.active_origin = self.document.current().setup.origin_index;
        self.pending_op = None;
        self.current_path = Some(path.to_path_buf());
        self.selection = Selection::Setup;
        self.reconcile_selection();
        self.invalidate();
        Ok(())
    }

    /// Export the current toolpath as G-code to `path`. Runs the same gating as
    /// [`AppController::export_nc`] (a rapid through stock blocks the write).
    pub fn export_nc_to(&mut self, path: impl AsRef<Path>) -> Result<(), ExportToError> {
        let nc = self.export_nc().map_err(ExportToError::Export)?.to_string();
        std::fs::write(path.as_ref(), nc).map_err(|e| ExportToError::Io(e.to_string()))?;
        Ok(())
    }

    /// Edit the setup's heights as one undoable change.
    pub fn edit_heights(&mut self, f: impl FnOnce(&mut Heights)) {
        self.document.edit(|doc| f(&mut doc.setup.heights));
        self.invalidate();
    }

    /// Edit setup-level fields beyond [`Heights`] — e.g. the tool-change height.
    pub fn edit_setup(&mut self, f: impl FnOnce(&mut cam_model::Setup)) {
        self.document.edit(|doc| f(&mut doc.setup));
        self.invalidate();
    }

    /// Edit the workpiece origin (datum) as one undoable change. Re-references the
    /// posted G-code; the design/sim frame is unaffected, so no re-run is needed —
    /// but `invalidate` keeps the pipeline honest.
    pub fn edit_origin(&mut self, f: impl FnOnce(&mut [f64; 3])) {
        self.document.edit(|doc| f(&mut doc.setup.origin));
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
        self.document.edit(|doc| {
            if doc.setup.origin_index == index {
                f(&mut doc.setup.origin);
            } else if let Some(o) = doc.setup.extra_origins.iter_mut().find(|o| o.index == index) {
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
        self.document.edit(move |doc| {
            doc.setup.extra_origins.push(Origin { index, position })
        });
        self.active_origin = index;
        self.selection = Selection::Origin;
        self.invalidate();
    }

    /// Delete the origin with `index` (the base origin — the one holding
    /// `origin_index` — is never removed). Its operations are reassigned to the base,
    /// and it drops from the active / disabled sets.
    pub fn delete_origin(&mut self, index: u32) {
        let base = self.setup().origin_index;
        if index == base {
            return;
        }
        self.document.edit(move |doc| {
            doc.setup.extra_origins.retain(|o| o.index != index);
            for op in &mut doc.setup.operations {
                if op.work_offset() == index {
                    op.set_work_offset(base);
                }
            }
        });
        self.disabled_origins.remove(&index);
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
        self.document.edit(move |doc| {
            let s = &mut doc.setup;
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
        });
        // Mirror the swap in the transient disabled set.
        let was_old = self.disabled_origins.remove(&old);
        let was_new = swap && self.disabled_origins.remove(&new_index);
        if was_old {
            self.disabled_origins.insert(new_index);
        }
        if was_new {
            self.disabled_origins.insert(old);
        }
        self.active_origin = new_index;
        self.invalidate();
    }

    /// Reassign operation `id` to origin `index` (its group), as one undoable change.
    pub fn set_operation_origin(&mut self, id: u32, index: u32) {
        self.document.edit(move |doc| {
            if let Some(op) = doc.setup.operations.iter_mut().find(|o| o.id() == id) {
                op.set_work_offset(index);
            }
        });
        self.invalidate();
    }

    /// Whether origin `index`'s operations are frozen out of the run.
    pub fn is_origin_disabled(&self, index: u32) -> bool {
        self.disabled_origins.contains(&index)
    }

    /// Freeze/unfreeze an origin's operations (kept in the tree, dropped from the run —
    /// so its toolpaths leave the viewport). The base can be frozen like any other.
    pub fn set_origin_disabled(&mut self, index: u32, disabled: bool) {
        if disabled {
            self.disabled_origins.insert(index);
        } else {
            self.disabled_origins.remove(&index);
        }
        self.invalidate();
    }

    /// Reconcile the just-loaded setup's tool numbers against the shop `shop` library
    /// (`TOOLING_PLAN.md` §6, Phase 2). Matched tools adopt the shop's number and every
    /// operation reference is rewritten; unmatched tools stay project-local. This is
    /// **load-time normalisation**, so on any change the undo baseline is reset to the
    /// reconciled state (there is no "undo the renumber" — the project simply opened
    /// this way). Returns the report so the shell can surface a summary.
    pub fn reconcile_tools(&mut self, shop: &[Tool]) -> ReconcileReport {
        let mut doc = self.document.current().clone();
        let report = reconcile_tool_numbers(&mut doc.setup, shop);
        if report.changed() {
            self.document = History::new(doc);
            self.reconcile_selection();
            self.invalidate();
        }
        report
    }

    /// Edit a tool as one undoable change. A no-op if the index is out of range.
    pub fn edit_tool(&mut self, index: usize, f: impl FnOnce(&mut Tool)) {
        let in_range = index < self.document.current().setup.tools.len();
        if !in_range {
            return;
        }
        self.document.edit(|doc| f(&mut doc.setup.tools[index]));
        self.invalidate();
    }

    /// Append a fresh default tool (numbered one past the highest existing tool)
    /// and select it, as one undoable change.
    pub fn add_tool(&mut self) {
        let number = self
            .document
            .current()
            .setup
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
        self.document.edit(move |doc| doc.setup.tools.push(tool));
        let index = self.document.current().setup.tools.len() - 1;
        self.selection = Selection::Tool(index);
        self.invalidate();
    }

    /// Delete the tool at `index`, selecting a neighbour (or the setup if none is
    /// left). A no-op if the index is out of range. Operations that referenced the
    /// tool's number are left as-is — they surface a "references tool N which is
    /// not in the setup" diagnostic on the next run rather than being rewritten.
    pub fn delete_tool(&mut self, index: usize) {
        let count = self.document.current().setup.tools.len();
        if index >= count {
            return;
        }
        self.document.edit(move |doc| {
            doc.setup.tools.remove(index);
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
            .document
            .current()
            .setup
            .tools
            .iter()
            .find(|t| same_tool_geometry(t, &tool))
        {
            return existing.number;
        }
        let number = self
            .document
            .current()
            .setup
            .tools
            .iter()
            .map(|t| t.number)
            .max()
            .map_or(1, |m| m + 1);
        let embedded = Tool { number, ..tool };
        self.document
            .edit(move |doc| doc.setup.tools.push(embedded));
        self.invalidate();
        number
    }

    /// The tools actually referenced by at least one operation, in setup order.
    /// This is what the read-only Project→Tools view shows.
    pub fn used_tools(&self) -> Vec<&Tool> {
        let setup = &self.document.current().setup;
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
        // Must count EVERY tool an operation references: pruning on the defining tool
        // alone would drop a carve's clearing end mill out of the setup, leaving the
        // operation pointing at a tool that is no longer there.
        let used: BTreeSet<u32> = self
            .document
            .current()
            .setup
            .operations
            .iter()
            .flat_map(Operation::tools)
            .collect();
        let has_unused = self
            .document
            .current()
            .setup
            .tools
            .iter()
            .any(|t| !used.contains(&t.number));
        if !has_unused {
            return;
        }
        self.document
            .edit(move |doc| doc.setup.tools.retain(|t| used.contains(&t.number)));
        self.invalidate();
    }

    /// Edit the selected operation as one undoable change. A no-op unless an
    /// operation is selected.
    /// Edit operation `id` in place (undoable). Used where the target is explicit
    /// rather than whatever is selected — e.g. changing an operation's tool from the
    /// inspector.
    pub fn edit_operation(&mut self, id: u32, f: impl FnOnce(&mut Operation)) {
        let mut edited = false;
        self.document.edit(|doc| {
            if let Some(op) = doc.setup.operations.iter_mut().find(|o| o.id() == id) {
                f(op);
                edited = true;
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
        let mut edited = false;
        self.document.edit(|doc| {
            if let Some(op) = doc.setup.operations.iter_mut().find(|o| o.id() == id) {
                f(op);
                edited = true;
            }
        });
        if edited {
            self.invalidate();
        }
    }

    /// The next free operation id (max existing + 1).
    fn next_op_id(&self) -> u32 {
        self.document
            .current()
            .setup
            .operations
            .iter()
            .map(|o| o.id())
            .max()
            .map_or(0, |m| m + 1)
    }

    /// Append `op` (renumbered with a fresh id) to the setup and select it, as one
    /// undoable change.
    pub fn add_operation(&mut self, mut op: Operation) {
        let id = self.next_op_id();
        set_op_id(&mut op, id);
        // A new operation joins the active origin's group (H<n>).
        op.set_work_offset(self.active_origin);
        self.document.edit(move |doc| doc.setup.operations.push(op));
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
        self.document.edit(move |doc| {
            if let Some(slot) = doc
                .setup
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
        let new_id = self.next_op_id();
        set_op_id(&mut copy, new_id);
        self.document.edit(move |doc| {
            let at = doc
                .setup
                .operations
                .iter()
                .position(|o| o.id() == sel)
                .map_or(doc.setup.operations.len(), |p| p + 1);
            doc.setup.operations.insert(at, copy);
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
        let td = self
            .document
            .current()
            .setup
            .tools
            .iter()
            .find(|t| t.number == tool);
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
        // An open imported path can only feed an operation that machines one; every
        // other strategy needs a closed region to offset, clear or bound. Refusing
        // here keeps a mis-picked stroke from becoming a nonsense profile.
        if boundary.is_open() && !op_accepts_open_paths(kind) {
            return None;
        }
        let (points, closed) = self.loop_points(boundary)?;
        let chain = Contour::new(points);
        let p = self.defaults;
        // Cutting data: the values chosen in the wizard if given, else the tool's
        // seeded nominals. A `spindle_rpm` of 0 falls back to the job default at plan
        // time; the inspector can override any of these per operation afterwards.
        let CuttingData {
            rpm: spindle_rpm,
            feed,
            plunge_feed,
        } = cutting.unwrap_or_else(|| self.seeded_cutting_for(tool));
        // id 0 is a placeholder — add_operation renumbers with a fresh id.
        let op = match kind {
            OpKind::Profile => Operation::Profile(ProfileOp {
                clearing: cam_model::Clearing::default(),
                id: 0,
                tool,
                chain,
                side: Side::Outside,
                comp: Comp::Computed,
                offset: 0.0,
                depth: p.depth,
                stepdown: p.stepdown,
                stepover: 0.0,
                spindle_rpm,
                work_offset: 1,
                feed,
                plunge_feed,
                start,
                lead_in: Lead::None,
                lead_out: Lead::None,
                lead_overlap: 0.0,
                plunge: Plunge::Straight,
            }),
            OpKind::Pocket => {
                let island_contours = islands
                    .iter()
                    .filter_map(|l| self.loop_contour(*l).cloned())
                    .collect();
                Operation::Pocket(PocketOp {
                    clearing: cam_model::Clearing::default(),
                    id: 0,
                    tool,
                    boundary: chain,
                    islands: island_contours,
                    depth: p.depth,
                    stepdown: p.stepdown,
                    overlap: 0.5,
                    offset: 0.0,
                    spindle_rpm,
                    work_offset: 1,
                    feed,
                    plunge_feed,
                    plunge: Plunge::Straight,
                    start,
                    lead_overlap: 0.0,
                    lead_in: Lead::None,
                    lead_out: Lead::None,
                })
            }
            OpKind::Drill => Operation::Drill(DrillOp {
                id: 0,
                tool,
                points: vec![centroid(&chain)],
                depth: p.depth,
                start_offset: 0.0,
                peck: None,
                dwell: None,
                spindle_rpm,
                work_offset: 1,
                feed: plunge_feed,
            }),
            OpKind::Face => {
                // Face along the edge the user clicked to pick the boundary (the
                // snapped pick point rides on it); with no pick, fall back to the
                // longest edge. The inspector picker still overrides either way.
                let direction = match start {
                    Some([x, y]) => {
                        Axis::along_edge_at(chain.points(), cam_geo::Point::new(x, y))
                    }
                    None => Axis::along_longest_edge(chain.points()),
                };
                Operation::Face(FaceOp {
                    id: 0,
                    tool,
                    boundary: chain,
                    // Face at the reference plane by default; a shallow skim the user
                    // then tunes (start_offset raises it to level proud stock).
                    start_offset: 0.0,
                    depth: 1.0,
                    stepdown: p.stepdown,
                    overlap: 0.5,
                    // `overshoot` is the cutter-edge clearance past the stock edge; the
                    // strategy handles the radius and stock geometry, so this is just a
                    // 5 mm comfort margin. Negative would plunge into the stock (warned).
                    overshoot: 5.0,
                    direction,
                    spindle_rpm,
                    work_offset: 1,
                    feed,
                    plunge_feed,
                })
            }
            OpKind::Chamfer => Operation::Chamfer(ChamferOp {
                id: 0,
                tool,
                chain,
                side: Side::Outside,
                width: 1.0,
                top: p.top_of_stock,
                // 0 depth ⇒ use the tool tip at the bevel bottom; 0 step ⇒ one pass.
                depth: 0.0,
                step: 0.0,
                gradual: false,
                spindle_rpm,
                work_offset: 1,
                feed,
                plunge_feed,
                start,
                lead_in: Lead::None,
                lead_out: Lead::None,
                lead_overlap: 0.0,
            }),
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
                    id: 0,
                    tool,
                    points: vec![centroid(&chain)],
                    internal: true,
                    hand: Hand::Right,
                    major_dia: if dia.is_finite() && dia > 0.0 { dia } else { 6.0 },
                    pitch: 1.0,
                    // Thread span is absolute Z; `depth` is now a positive magnitude
                    // below the top, so the bottom sits that far under it.
                    z_top: p.top_of_stock,
                    z_bottom: p.top_of_stock - p.depth,
                    climb: true,
                    passes: 1,
                    spring_passes: 0,
                    drill_clearance: 0.0,
                    blind_allowance: 0.0,
                    spindle_rpm,
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
                    .filter_map(|l| self.loop_contour(*l).cloned())
                    .collect();
                Operation::Carve(CarveOp {
                    id: 0,
                    tool,
                    // No clearing tool by default: the V-bit alone is the simple case,
                    // and the inspector offers the second tool only once the shape is
                    // known to leave a flat land.
                    clear: None,
                    boundary: chain,
                    islands: island_contours,
                    top: p.top_of_stock,
                    // A cap, not a command: the shape decides the actual depth. Deeper
                    // than an engraving default, since a carve is meant to be seen.
                    depth: 1.0,
                    offset: 0.0,
                    ring_step: 0.0,
                    scallop: 0.0,
                    spindle_rpm,
                    work_offset: 1,
                    feed,
                    plunge_feed,
                    // On by default: a carve is hundreds of rings, and every link is
                    // verified before it is taken (the ones that would gouge still lift).
                    stay_down: true,
                    start,
                })
            }
            OpKind::Engrave => Operation::Engrave(EngraveOp {
                id: 0,
                tool,
                chain,
                // Closed for a picked region loop; open for an imported stroke.
                closed,
                top: p.top_of_stock,
                // A shallow default: engraving is a surface mark, not a cut. One
                // pass (stepdown 0) — normal at this depth.
                depth: 0.3,
                stepdown: 0.0,
                spindle_rpm,
                work_offset: 1,
                feed,
                plunge_feed,
                start,
            }),
        };
        Some(op)
    }

    /// The lowest tool number in the setup (the default for a new operation), or 1.
    fn first_tool_number(&self) -> u32 {
        self.document
            .current()
            .setup
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

    /// The boundary loop whose **edge** is closest to `world` within `aperture`
    /// world-mm, together with the nearest vertex on it (the start point). This is
    /// line picking: a point-to-segment distance to each closed loop's edges across
    /// every region — a click inside an empty area (near no edge) returns `None`.
    pub fn nearest_loop(&self, world: [f64; 2], aperture: f64) -> Option<(LoopRef, [f64; 2])> {
        let w = Point::new(world[0], world[1]);
        // (loop, nearest vertex, closest edge distance²)
        let mut best: Option<(LoopRef, [f64; 2], f64)> = None;
        for (ri, region) in self.regions.iter().enumerate() {
            let loops = std::iter::once((LoopPart::Outer, region.outer())).chain(
                region
                    .holes()
                    .iter()
                    .enumerate()
                    .map(|(hi, h)| (LoopPart::Hole(hi), h)),
            );
            for (part, contour) in loops {
                let pts = contour.points();
                if pts.len() < 2 {
                    continue;
                }
                let edge_d2 = (0..pts.len())
                    .map(|k| dist_point_seg2(w, pts[k], pts[(k + 1) % pts.len()]))
                    .fold(f64::MAX, f64::min);
                if best.is_none_or(|(_, _, bd2)| edge_d2 < bd2) {
                    let nv = pts
                        .iter()
                        .min_by(|p, q| w.distance_sq(**p).total_cmp(&w.distance_sq(**q)))
                        .unwrap();
                    best = Some((LoopRef { region: ri, part }, [nv.x, nv.y], edge_d2));
                }
            }
        }
        best.filter(|(_, _, d2)| *d2 <= aperture * aperture)
            .map(|(l, v, _)| (l, v))
    }

    /// The loop whose edge is closest to `world` within `aperture`, together with
    /// the exact **nearest point on that edge** (not a vertex). The always-works
    /// fallback for boundary selection when no object-snap catches. With
    /// `circles_only`, only circular loops are selectable (drill/thread target
    /// holes) — a rectangle or open edge is ignored.
    pub fn nearest_loop_point(
        &self,
        world: [f64; 2],
        aperture: f64,
        circles_only: bool,
    ) -> Option<(LoopRef, [f64; 2])> {
        let w = Point::new(world[0], world[1]);
        let mut best: Option<(LoopRef, [f64; 2], f64)> = None;
        for (loop_ref, pts, closed) in self.iter_pickable() {
            // Drill/thread target holes: an open stroke is never a hole.
            if circles_only && (!closed || fit_circle(pts).is_none()) {
                continue;
            }
            let (pt, d2) = nearest_point_on_path(pts, w, closed);
            if best.is_none_or(|(_, _, bd2)| d2 < bd2) {
                best = Some((loop_ref, pt, d2));
            }
        }
        best.filter(|(_, _, d2)| *d2 <= aperture * aperture)
            .map(|(l, p, _)| (l, p))
    }

    /// The best object-snap under the cursor among the `enabled` kinds, within
    /// `aperture` world-mm, or `None`. Higher-priority kinds (End → Mid →
    /// Quadrant → Nearest) win over lower ones even if slightly farther, so a
    /// corner beats a mere nearest-point when both are in the box. Read-only —
    /// the GUI calls it on hover to preview the snap marker and on click to set
    /// the start.
    pub fn snap_at(&self, world: [f64; 2], aperture: f64, enabled: &[SnapKind]) -> Option<SnapHit> {
        let w = Point::new(world[0], world[1]);
        let ap2 = aperture * aperture;
        // (priority, dist², hit) — lower priority number wins.
        let mut best: Option<(u8, f64, SnapHit)> = None;
        let mut consider = |kind: SnapKind, point: [f64; 2], loop_ref: LoopRef| {
            let d2 = w.distance_sq(Point::new(point[0], point[1]));
            if d2 > ap2 {
                return;
            }
            let prio = kind.priority();
            let better = best
                .as_ref()
                .is_none_or(|(bp, bd, _)| prio < *bp || (prio == *bp && d2 < *bd));
            if better {
                best = Some((prio, d2, SnapHit { loop_ref, point, kind }));
            }
        };
        for (loop_ref, pts, closed) in self.iter_pickable() {
            if pts.len() < 2 {
                continue;
            }
            // An open stroke has no corner *loop*: its two free ends are the snaps a
            // machinist wants (that is where the groove starts and stops), and the
            // mid/quadrant analyses assume a closed ring, so they are skipped.
            if !closed {
                if enabled.contains(&SnapKind::End) {
                    for p in [pts[0], pts[pts.len() - 1]] {
                        consider(SnapKind::End, [p.x, p.y], loop_ref);
                    }
                }
                if enabled.contains(&SnapKind::Nearest) {
                    let (q, _) = nearest_point_on_path(pts, w, false);
                    consider(SnapKind::Nearest, q, loop_ref);
                }
                continue;
            }
            let corners = loop_corners(pts);
            if enabled.contains(&SnapKind::End) {
                for &ci in &corners {
                    consider(SnapKind::End, [pts[ci].x, pts[ci].y], loop_ref);
                }
            }
            if enabled.contains(&SnapKind::Mid) {
                for m in loop_mids(pts, &corners) {
                    consider(SnapKind::Mid, m, loop_ref);
                }
            }
            // Quadrant: the cardinal points of a **circular** loop (a cornerless
            // loop that fits a circle). Partial arcs await the arc-carrying model.
            if enabled.contains(&SnapKind::Quadrant) && corners.is_empty() {
                if let Some(([cx, cy], r)) = fit_circle(pts) {
                    for (dx, dy) in [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)] {
                        consider(SnapKind::Quadrant, [cx + dx * r, cy + dy * r], loop_ref);
                    }
                }
            }
            if enabled.contains(&SnapKind::Nearest) {
                let (q, _) = nearest_point_on_contour(pts, w);
                consider(SnapKind::Nearest, q, loop_ref);
            }
        }
        best.map(|(_, _, hit)| hit)
    }

    /// Whether open imported paths are currently pickable — true exactly when the
    /// operation being created can machine one, so hover preview and click agree.
    fn open_paths_pickable(&self) -> bool {
        self.pending_op
            .as_ref()
            .is_some_and(|p| op_accepts_open_paths(p.kind))
    }

    /// Every pickable path: the closed region loops, plus the open imported paths
    /// when the pending operation can machine them. Yields `(ref, points, closed)`.
    fn iter_pickable(&self) -> impl Iterator<Item = (LoopRef, &[Point], bool)> {
        let open = if self.open_paths_pickable() {
            &self.open_paths[..]
        } else {
            &[][..]
        };
        self.iter_loops()
            .map(|(r, c)| (r, c.points(), true))
            .chain(
                open.iter()
                    .enumerate()
                    .map(|(i, p)| (LoopRef::open(i), p.points(), false)),
            )
    }

    /// Every closed loop across all regions as `(ref, contour)`.
    fn iter_loops(&self) -> impl Iterator<Item = (LoopRef, &Contour)> {
        self.regions.iter().enumerate().flat_map(|(ri, region)| {
            std::iter::once((
                LoopRef {
                    region: ri,
                    part: LoopPart::Outer,
                },
                region.outer(),
            ))
            .chain(region.holes().iter().enumerate().map(move |(hi, h)| {
                (
                    LoopRef {
                        region: ri,
                        part: LoopPart::Hole(hi),
                    },
                    h,
                )
            }))
        })
    }

    /// Record a viewport pick into the pending operation at `world`, with a pickbox
    /// of `aperture` world-mm.
    ///
    /// **Nothing is committed here.** The pick sets (or replaces) the boundary; for a
    /// **Pocket** that already has a boundary, further picks toggle islands. The
    /// operation is created only by [`confirm_operation`], so the tool and the
    /// geometry may be chosen in **either order**, and either may be changed until
    /// Confirm. Re-picking before Confirm simply moves the boundary.
    pub fn pick_operation_geometry(
        &mut self,
        world: [f64; 2],
        aperture: f64,
        snaps: &[SnapKind],
    ) -> PickResult {
        let Some(pending) = self.pending_op.clone() else {
            return PickResult::Missed;
        };
        // Resolve the loop and start point: prefer an object-snap (corner / mid /
        // …), else the nearest point on the loop under the box. Drill/thread are
        // restricted to circular loops (holes).
        let circles = op_selects_circles(pending.kind);
        let (picked, start) = match self.snap_at(world, aperture, snaps) {
            Some(hit) => (hit.loop_ref, hit.point),
            None => match self.nearest_loop_point(world, aperture, circles) {
                Some(lp) => lp,
                None => return PickResult::Missed,
            },
        };
        // Islands are a second stage, once a boundary exists, for the kinds that
        // consume an area and so must be told what to leave standing.
        let island_mode = op_takes_islands(pending.kind) && pending.boundary.is_some();
        // Carving a letter needs its counter left uncut, and a counter is exactly a
        // hole of the picked region — so seed the islands from the geometry's own
        // nesting rather than making the operator click each one. They stay togglable.
        let nested: Vec<LoopRef> = match (pending.kind, picked.part) {
            (OpKind::Carve, LoopPart::Outer) => self
                .regions
                .get(picked.region)
                .map(|r| (0..r.holes().len()).map(|h| LoopRef::hole(picked.region, h)).collect())
                .unwrap_or_default(),
            _ => Vec::new(),
        };
        let p = self.pending_op.as_mut().unwrap();
        if island_mode {
            if Some(picked) == p.boundary {
                return PickResult::Selecting; // the boundary is not an island
            }
            match p.islands.iter().position(|l| *l == picked) {
                Some(pos) => {
                    p.islands.remove(pos);
                }
                None => p.islands.push(picked),
            }
        } else {
            p.boundary = Some(picked);
            p.start = Some(start);
            p.islands = nested;
        }
        PickResult::Selecting
    }

    /// Finalise the pending operation — the **single commit point** for every kind.
    /// Requires both a chosen tool and a picked boundary; [`Self::pending_ready`]
    /// reports whether those hold, so the UI can gate its Confirm button on the same
    /// condition. Returns `true` if an operation was created.
    pub fn confirm_operation(&mut self, cutting: Option<CuttingData>) -> bool {
        let Some(pending) = self.pending_op.clone() else {
            return false;
        };
        let (Some(boundary), Some(tool)) = (pending.boundary, pending.tool) else {
            return false;
        };
        if let Some(op) =
            self.build_op(pending.kind, boundary, &pending.islands, tool, pending.start, cutting)
        {
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
        self.pending_op
            .as_ref()
            .is_some_and(|p| p.tool.is_some() && p.boundary.is_some())
    }

    /// Delete the selected operation, selecting a neighbour (or the setup). A
    /// no-op unless an operation is selected.
    pub fn delete_selected_operation(&mut self) {
        let Selection::Operation(id) = self.selection else {
            return;
        };
        self.document
            .edit(|doc| doc.setup.operations.retain(|o| o.id() != id));
        self.excluded.remove(&id);
        self.selection = self
            .document
            .current()
            .setup
            .operations
            .first()
            .map_or(Selection::Setup, |o| Selection::Operation(o.id()));
        self.invalidate();
    }

    /// Whether operation `id` is excluded from toolpath generation.
    pub fn is_operation_excluded(&self, id: u32) -> bool {
        self.excluded.contains(&id)
    }

    /// Clusters of **included** operations that are exact duplicates of one
    /// another — identical in every field but their id (see
    /// [`Operation::same_work`]). Each returned group holds ≥2 ids in document
    /// order; the first is the "original", the rest its exact copies.
    ///
    /// Excluded operations never reach the post, so they are ignored: excluding a
    /// twin is the sanctioned way to keep an exact duplicate around (e.g. a spring
    /// pass that is currently switched off) without tripping the warning.
    pub fn duplicate_operation_groups(&self) -> Vec<Vec<u32>> {
        let ops: Vec<&Operation> = self
            .document()
            .setup
            .operations
            .iter()
            .filter(|op| !self.is_operation_excluded(op.id()))
            .collect();
        let mut groups: Vec<Vec<u32>> = Vec::new();
        let mut taken = vec![false; ops.len()];
        for i in 0..ops.len() {
            if taken[i] {
                continue;
            }
            let mut group = vec![ops[i].id()];
            for j in (i + 1)..ops.len() {
                if !taken[j] && ops[i].same_work(ops[j]) {
                    taken[j] = true;
                    group.push(ops[j].id());
                }
            }
            if group.len() > 1 {
                groups.push(group);
            }
        }
        groups
    }

    /// The set of operation ids that are exact duplicates of another included
    /// operation (the flattened [`Self::duplicate_operation_groups`]). Drives the
    /// project-tree "duplicate" marker.
    pub fn duplicate_operation_ids(&self) -> BTreeSet<u32> {
        self.duplicate_operation_groups()
            .into_iter()
            .flatten()
            .collect()
    }

    /// Include or exclude operation `id` from toolpath generation (it stays in
    /// the tree either way).
    pub fn set_operation_excluded(&mut self, id: u32, excluded: bool) {
        let changed = if excluded {
            self.excluded.insert(id)
        } else {
            self.excluded.remove(&id)
        };
        if changed {
            self.invalidate();
        }
    }

    /// Move operation `id` one step earlier (`up`) or later in execution order.
    /// A no-op if it can't move that way.
    pub fn move_operation(&mut self, id: u32, up: bool) {
        self.document.edit(|doc| {
            let ops = &mut doc.setup.operations;
            if let Some(i) = ops.iter().position(|o| o.id() == id) {
                // Reorder within the operation's own origin group: swap with the nearest
                // sibling of the same `work_offset` in the chosen direction. This keeps
                // moves confined to a group, matching how the tree renders them.
                let wo = ops[i].work_offset();
                let j = if up {
                    ops[..i].iter().rposition(|o| o.work_offset() == wo)
                } else {
                    ops[i + 1..]
                        .iter()
                        .position(|o| o.work_offset() == wo)
                        .map(|k| i + 1 + k)
                };
                if let Some(j) = j {
                    ops.swap(i, j);
                }
            }
        });
        self.invalidate();
    }

    /// Move the selected operation one step in execution order.
    pub fn move_selected_operation(&mut self, up: bool) {
        if let Selection::Operation(id) = self.selection {
            self.move_operation(id, up);
        }
    }

    /// Undo the last document edit.
    pub fn undo(&mut self) -> bool {
        let changed = self.document.undo();
        if changed {
            self.reconcile_selection();
            self.invalidate();
        }
        changed
    }

    /// Redo an undone document edit.
    pub fn redo(&mut self) -> bool {
        let changed = self.document.redo();
        if changed {
            self.reconcile_selection();
            self.invalidate();
        }
        changed
    }

    /// Whether an undo / redo is available.
    pub fn can_undo(&self) -> bool {
        self.document.can_undo()
    }
    pub fn can_redo(&self) -> bool {
        self.document.can_redo()
    }

    /// Run the strategies for the current document, producing the program,
    /// diagnostics, viewport scene, and simulated stock. Returns a reference to
    /// the outcome. `cancel` allows a long run to be aborted.
    pub fn run(&mut self, cancel: &CancelToken) -> &RunOutcome {
        // Excluded operations, and operations under a frozen origin, are dropped from
        // the job (kept in the tree, just not machined) by running a filtered copy.
        let base = self.document.current();
        let document = if self.excluded.is_empty() && self.disabled_origins.is_empty() {
            Cow::Borrowed(base)
        } else {
            let mut d = base.clone();
            d.setup
                .operations
                .retain(|o| !self.excluded.contains(&o.id()) && !self.disabled_origins.contains(&o.work_offset()));
            Cow::Owned(d)
        };
        // The resolved stock box (XY) lets roughing strategies (profile stepover)
        // clear out to the raw material.
        let (smin, smax) = self.stock_box();
        let stock = Some(([smin[0], smin[1]], [smax[0], smax[1]]));
        // Tool-change lift height: the setup's explicit value, else the top of the
        // machine's Z envelope.
        let tool_change_height = document
            .setup
            .resolved_tool_change_height(self.machine.envelope.max.z);
        let (program, diagnostics) = build_job(
            &document,
            self.defaults.spindle_rpm,
            SpindleDir::Cw,
            stock,
            tool_change_height,
            cancel,
        );

        let mut scene = Scene::from_program(&program);
        for region in &self.regions {
            scene.add_region(region, PART);
        }
        for path in &self.open_paths {
            scene.add_open_path(path.points(), PART);
        }

        let (stock_vertices, stock_indices, collisions) = self.simulate_stock(&program);

        self.nc = None;
        self.outcome = Some(RunOutcome {
            program,
            diagnostics,
            scene,
            stock_vertices,
            stock_indices,
            collisions,
        });
        self.outcome.as_ref().unwrap()
    }

    /// Post the most recent run to grbl G-code, caching and returning it. Fails
    /// if nothing has run or the run had errors.
    pub fn export_nc(&mut self) -> Result<&str, ExportError> {
        let outcome = self.outcome.as_ref().ok_or(ExportError::NothingToExport)?;
        if outcome.has_errors() {
            return Err(ExportError::HasErrors);
        }
        // A rapid plowing through stock would crash the machine — block by default.
        let rapids = outcome
            .collisions
            .iter()
            .filter(|c| c.kind == CollisionKind::RapidThroughStock)
            .count();
        if rapids > 0 {
            return Err(ExportError::RapidThroughStock(rapids));
        }
        let options = self.post_options();
        // Re-reference each group to its own workpiece origin (datum): shift every
        // coordinate by −origin so that group's chosen part point becomes G-code
        // (0,0,0), matched by the operator's touch-off for `G15 H<n>`. Operations under
        // different origins (a reorientation) each subtract their own; with one origin
        // this is the whole job shifted by −origin. Design/sim stay in part space.
        let setup = &self.document.current().setup;
        let program = outcome.program.translated_per_datum(|idx| {
            let o = setup.origin_position(idx);
            [-o[0], -o[1], -o[2]]
        });
        let nc = self.post_kind.post(&program, &self.machine, &options)?;
        self.nc = Some(nc);
        Ok(self.nc.as_deref().unwrap())
    }

    /// Simulate material removal for `program`: triangulate the remaining stock
    /// into render-ready vertices + indices, and return any collisions found.
    /// Empty when there is no stock (no geometry loaded).
    fn simulate_stock(&self, program: &Program) -> (Vec<MeshVertex>, Vec<u32>, Vec<Collision>) {
        let (min, max) = self.stock_box();
        if min[0] >= max[0] || min[1] >= max[1] {
            return (Vec::new(), Vec::new(), Vec::new());
        }
        let setup_tools = &self.document.current().setup.tools;
        let tool_diameter = setup_tools
            .first()
            .map(|t| t.diameter)
            .unwrap_or(self.defaults.tool_diameter);
        // The sim's tool table: each setup tool as its cutting profile, selected by
        // the program's tool-change numbers.
        let sim_tools: Vec<SimTool> = setup_tools
            .iter()
            .map(|t| SimTool {
                number: t.number,
                profile: sim_profile(t),
            })
            .collect();
        // Aim for ~300 cells across the larger side, bounded so a big part stays
        // cheap and a small one stays crisp. (A modest, 2.5D-cosmetic bump from
        // 200 to sharpen curved-feature silhouettes; dial back if a rerun lags.)
        let span = (max[0] - min[0]).max(max[1] - min[1]);
        // …but a part-sized grid can be blind to a *narrow* feature: an engraved
        // V-groove is often under a millimetre wide, so at the span-derived cell size
        // it would simply not appear and the backplot would read as a failed
        // toolpath. Refine until the narrowest cut spans a few cells, capped by a
        // cell budget so refining never turns a preview into a hang.
        let feature = narrowest_cut_width(program, setup_tools, max[2]);
        let by_span = span / 300.0;
        let by_feature = feature / 3.0;
        let budget = span / MAX_SIM_CELLS_ACROSS;
        let resolution = by_span.min(by_feature).max(budget).clamp(0.02, 2.0);
        let sim = simulate(
            program,
            min,
            max,
            &SimOptions {
                resolution,
                tool_radius: tool_diameter / 2.0,
            },
            &sim_tools,
        );
        // Render the stock as a watertight solid down to its floor, so the full
        // thickness shows and cut walls shade crisply (not a smooth top sheet).
        let surface = sim.field.to_solid_mesh(min[2]);
        let vertices = mesh_vertices(&surface.positions, &surface.normals);
        (vertices, surface.indices, sim.collisions)
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
                    work_offset: 1,
                    feed: p.feed,
                    plunge_feed: p.plunge_feed,
                    start: None,
                    lead_in: Lead::None,
                    lead_out: Lead::None,
                    lead_overlap: 0.0,
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
                            id,
                            tool: 1,
                            points: vec![centroid(hole)],
                            depth: p.depth,
                            start_offset: 0.0,
                            peck: None,
                            dwell: None,
                            spindle_rpm: 0.0,
                            work_offset: 1,
                            feed: p.plunge_feed,
                        }));
                    } else {
                        operations.push(Operation::Pocket(PocketOp {
                            clearing: cam_model::Clearing::default(),
                            id,
                            tool: 1,
                            boundary: hole.clone(),
                            islands: Vec::new(),
                            depth: p.depth,
                            stepdown: p.stepdown,
                            overlap: 0.5,
                            offset: 0.0,
                            spindle_rpm: 0.0,
                            work_offset: 1,
                            feed: p.feed,
                            plunge_feed: p.plunge_feed,
                            plunge: Plunge::Straight,
                            start: None,
                            lead_overlap: 0.0,
                            lead_in: Lead::None,
                            lead_out: Lead::None,
                        }));
                    }
                    id += 1;
                }
            }
        }

        Document::new(Setup {
            name: self.program_name(),
            heights: Heights::new(p.clearance, p.retract, p.top_of_stock),
            stock: self.stock(p.top_of_stock, p.depth),
            tools,
            operations,
            origin: [0.0, 0.0, 0.0],
            extra_origins: vec![],
            origin_index: 1,
            tool_change_height: None,
        })
    }

    /// The XY bounding box of all imported geometry, `([min_x, min_y], [max_x,
    /// max_y])`. Collapses to the origin when nothing is loaded.
    fn bounds_xy(&self) -> ([f64; 2], [f64; 2]) {
        let mut min = [f64::MAX, f64::MAX];
        let mut max = [f64::MIN, f64::MIN];
        for region in &self.regions {
            for pt in region.outer().points() {
                min[0] = min[0].min(pt.x);
                min[1] = min[1].min(pt.y);
                max[0] = max[0].max(pt.x);
                max[1] = max[1].max(pt.y);
            }
        }
        // Open paths count too: a drawing that is *only* engraving strokes would
        // otherwise report no bounds, leaving the stock and the framed view empty.
        for path in &self.open_paths {
            for pt in path.points() {
                min[0] = min[0].min(pt.x);
                min[1] = min[1].min(pt.y);
                max[0] = max[0].max(pt.x);
                max[1] = max[1].max(pt.y);
            }
        }
        if min[0] > max[0] {
            ([0.0, 0.0], [0.0, 0.0])
        } else {
            (min, max)
        }
    }

    /// The initial part-relative stock: a snug fit to the geometry (no XY offset)
    /// hanging from `top` down to the deepest cut `depth`.
    fn stock(&self, top: f64, depth: f64) -> Stock {
        Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top,
            // `depth` is a positive magnitude below the reference; the stock hangs
            // from `top` down to the deepest cut at Z = -depth.
            thickness: top + depth,
        }
    }

    /// The current stock resolved to a concrete box `(min, max)` (mm), fitting the
    /// stock spec to the loaded geometry's XY bounds.
    pub fn stock_box(&self) -> ([f64; 3], [f64; 3]) {
        let (min, max) = self.bounds_xy();
        self.document.current().setup.stock.resolve(min, max)
    }

    /// Edit the setup's stock definition as one undoable change.
    pub fn edit_stock(&mut self, f: impl FnOnce(&mut Stock)) {
        self.document.edit(|doc| f(&mut doc.setup.stock));
        self.invalidate();
    }

    fn program_name(&self) -> String {
        if self.source_name.is_empty() {
            "opencamstudio".to_string()
        } else {
            self.source_name.clone()
        }
    }

    /// After an undo/redo, drop a selection that no longer points at anything.
    fn reconcile_selection(&mut self) {
        let sel = self.selection;
        self.select(sel);
    }

    /// Drop any run/export that no longer reflects the inputs.
    fn invalidate(&mut self) {
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

fn loop_corners(pts: &[Point]) -> Vec<usize> {
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
fn loop_mids(pts: &[Point], corners: &[usize]) -> Vec<[f64; 2]> {
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
                mid = [w[0].x + (w[1].x - w[0].x) * t, w[0].y + (w[1].y - w[0].y) * t];
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
fn fit_circle(pts: &[Point]) -> Option<([f64; 2], f64)> {
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
    let mut best = ([pts.first().map_or(0.0, |p| p.x), pts.first().map_or(0.0, |p| p.y)], f64::MAX);
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
const MAX_SIM_CELLS_ACROSS: f64 = 1200.0;

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
fn narrowest_cut_width(program: &Program, tools: &[Tool], surface_z: f64) -> f64 {
    use cam_cldata::{MoveKind, Step};
    use std::collections::BTreeMap;

    // The deepest cutting Z reached under each tool number, walking the tool changes.
    let mut current: Option<u32> = None;
    let mut deepest: BTreeMap<u32, f64> = BTreeMap::new();
    for step in program.steps() {
        match step {
            Step::ToolChange { tool } => current = Some(*tool),
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
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
fn sim_profile(tool: &Tool) -> ToolProfile {
    let radius = tool.radius();
    let shape = match tool.kind {
        ToolKind::EndMill | ToolKind::FaceMill => ProfileShape::Flat,
        ToolKind::BallMill => ProfileShape::Ball,
        ToolKind::BullNose { corner_radius } => ProfileShape::BullNose {
            corner_radius: corner_radius.clamp(0.0, radius),
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
        // A thread mill's material removal is approximated by its footprint.
        ToolKind::ThreadMill { .. } => ProfileShape::Flat,
    };
    ToolProfile { radius, shape }
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
        Operation::Thread(_) => OpKind::Thread,
        Operation::Engrave(_) => OpKind::Engrave,
        Operation::Carve(_) => OpKind::Carve,
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

/// An empty starting document: a zero-size box stock, one default end mill, no
/// operations.
fn empty_document(p: &JobParams) -> Document {
    Document::new(Setup {
        name: "Untitled".to_string(),
        heights: Heights::new(p.clearance, p.retract, p.top_of_stock),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: p.top_of_stock,
            thickness: p.top_of_stock + p.depth,
        },
        tools: vec![Tool {
            number: 1,
            diameter: p.tool_diameter,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }],
        operations: Vec::new(),
        origin: [0.0, 0.0, 0.0],
        extra_origins: vec![],
        origin_index: 1,
        tool_change_height: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_model::{Envelope, Point3};

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
            Operation::Thread(o) => o.z_bottom,
            Operation::Chamfer(o) => o.top,
            Operation::Engrave(o) => o.depth,
            Operation::Carve(o) => o.depth,
        }
    }

    #[test]
    fn open_generates_ops_and_selects_the_first() {
        let mut app = AppController::new(machine());
        assert_eq!(app.open_dxf(PART_DXF, "part.dxf").unwrap(), 1);
        // Outside profile for the boundary + a hole op (drill/pocket, never an
        // inner profile) = 2 operations.
        assert_eq!(app.document().setup.operations.len(), 2);
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
            .setup
            .operations
            .iter()
            .map(|o| o.id())
            .collect()
    }

    #[test]
    fn stock_auto_fits_the_part_and_offsets_grow_it() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let (bmin, bmax) = app.bounds_xy();

        // Default stock snugly fits the part in XY (no offset) and hangs from the
        // top down to the deepest cut.
        let (min, max) = app.stock_box();
        assert_eq!([min[0], min[1]], bmin, "no XY offset by default");
        assert_eq!([max[0], max[1]], bmax);
        assert!(max[2] > min[2], "non-degenerate Z");

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
        assert_eq!([min[0], min[1]], bmin, "undo restores the snug fit");
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
        assert_eq!(
            app.duplicate_operation_ids(),
            BTreeSet::from([0, 2])
        );

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
        assert!(app.duplicate_operation_groups().is_empty(), "different work");
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
        let (bmin, bmax) = app.bounds_xy();

        // The convenience builder targets the first region's outer loop with no islands.
        app.new_operation(OpKind::Pocket);
        match app.selected_operation() {
            Some(Operation::Pocket(o)) => {
                assert!(o.islands.is_empty(), "no islands by default");
                assert!((0.0..1.0).contains(&o.overlap), "a sane default overlap");
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
        // The guard that stops a mis-picked stroke turning into a nonsense profile
        // or pocket — those strategies need a closed area to offset and clear.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_WITH_STROKE_DXF, "part.dxf").unwrap();
        for kind in [
            OpKind::Profile,
            OpKind::Pocket,
            OpKind::Face,
            OpKind::Drill,
            OpKind::Thread,
            OpKind::Chamfer,
        ] {
            assert!(
                app.build_op(kind, LoopRef::open(0), &[], 1, None, None).is_none(),
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
        assert!(!app.open_paths_pickable());
        let hit = app.nearest_loop_point(on_stroke, 2.0, false);
        assert!(
            hit.is_none_or(|(l, _)| !l.is_open()),
            "profile must not pick the stroke"
        );

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
            schema_version: cam_model::SCHEMA_VERSION,
            document: app.document().clone(),
            regions: app.regions().to_vec(),
            open_paths: app.open_paths().to_vec(),
            defaults: *app.defaults(),
            source_name: app.source_name().to_string(),
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
            schema_version: cam_model::SCHEMA_VERSION,
            document: app.document().clone(),
            regions: app.regions().to_vec(),
            open_paths: Vec::new(),
            defaults: *app.defaults(),
            source_name: app.source_name().to_string(),
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
        let op = app.build_op(OpKind::Profile, picked, &[], 1, None, None).unwrap();
        app.replace_operation(target, op);

        let ids_after = op_ids(&app);
        assert_eq!(ids_after.len(), ids_before.len(), "no operation added or lost");
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
        // Force a guard error: profile with a V-bit (no cylindrical cutting flank).
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
        let out = app.run(&CancelToken::new());
        let errors: Vec<&Diagnostic> = out
            .diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Error)
            .collect();
        assert!(!errors.is_empty(), "the V-bit profile must error");
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
                assert_eq!(o.side, Side::Outside, "chamfers default to the outside edge");
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
        assert_eq!(app.document().setup.heights.clearance, 12.0);
        assert!(app.undo());
        assert_eq!(app.document().setup.heights.clearance, 5.0);
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
    fn export_before_run_fails() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert_eq!(app.export_nc(), Err(ExportError::NothingToExport));
    }

    #[test]
    fn unsafe_heights_surface_a_rapid_collision() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        // A clean run has no collisions.
        assert!(app.run(&CancelToken::new()).collisions.is_empty());
        // Clearance/retract below the stock top force lateral rapids through the
        // stock — the simulator must catch what a green backplot would hide.
        app.edit_heights(|h| {
            h.clearance = -1.0;
            h.retract = -1.0;
        });
        let outcome = app.run(&CancelToken::new());
        assert!(
            !outcome.collisions.is_empty(),
            "a rapid through stock must be flagged"
        );
        // ...and it blocks export (a machine-crash hazard).
        assert!(
            matches!(app.export_nc(), Err(ExportError::RapidThroughStock(_))),
            "a rapid through stock must block export"
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
        assert_eq!(app.export_nc(), Err(ExportError::HasErrors));
    }

    #[test]
    fn add_and_delete_tool_are_undoable() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        assert_eq!(app.document().setup.tools.len(), 1);

        app.add_tool();
        let tools = &app.document().setup.tools;
        assert_eq!(tools.len(), 2);
        // Numbered one past the highest existing tool, and selected.
        assert_eq!(tools[1].number, 2);
        assert_eq!(app.selection(), Selection::Tool(1));

        app.delete_tool(1);
        assert_eq!(app.document().setup.tools.len(), 1);

        // Both mutations are single undo steps.
        assert!(app.undo()); // undo delete
        assert_eq!(app.document().setup.tools.len(), 2);
        assert!(app.undo()); // undo add
        assert_eq!(app.document().setup.tools.len(), 1);
    }

    #[test]
    fn tool_length_round_trips_through_edit() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.edit_tool(0, |t| t.length = 42.0);
        assert_eq!(app.document().setup.tools[0].length, 42.0);
    }

    #[test]
    fn deleting_the_last_tool_selects_the_setup() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.delete_tool(0);
        assert!(app.document().setup.tools.is_empty());
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
    fn legacy_untagged_project_still_opens() {
        // A `.ocam` written before the `OcamFile` tag existed is a bare Project object.
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let project = Project {
            schema_version: cam_model::SCHEMA_VERSION,
            document: app.document().clone(),
            regions: app.regions().to_vec(),
            open_paths: app.open_paths().to_vec(),
            defaults: *app.defaults(),
            source_name: app.source_name().to_string(),
        };
        let path = temp_path("legacy.ocam");
        std::fs::write(&path, project.to_json().unwrap()).unwrap(); // untagged, on purpose

        app.new_project();
        app.open_project(&path).expect("legacy untagged project must load");
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
            schema_version: cam_model::SCHEMA_VERSION,
            document: app.document().clone(),
            regions: app.regions().to_vec(),
            open_paths: app.open_paths().to_vec(),
            defaults: *app.defaults(),
            source_name: app.source_name().to_string(),
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
            app.document().setup.operations.is_empty(),
            "import must not auto-create operations"
        );
        assert_eq!(app.selection(), Selection::Setup);
        // Real imports also start with no embedded tools — tools arrive when ops
        // are set up (picked from the library).
        assert!(
            app.document().setup.tools.is_empty(),
            "import must not seed tools"
        );
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn use_tool_embeds_dedupes_and_reports_used() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap(); // sample: seeds tool T1 + 2 ops
        let base = app.document().setup.tools.len();

        // Embedding a brand-new tool adds it with a fresh number.
        let bit = Tool {
            number: 99, // library numbering is ignored on embed
            diameter: 4.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        };
        let n1 = app.use_tool(bit);
        assert_eq!(app.document().setup.tools.len(), base + 1);
        assert_ne!(
            n1, 99,
            "embedded number is project-local, not the library's"
        );

        // Embedding the same geometry again reuses the number (no duplicate).
        let n2 = app.use_tool(bit);
        assert_eq!(n1, n2);
        assert_eq!(app.document().setup.tools.len(), base + 1);

        // used_tools reflects only tools referenced by operations. The freshly
        // embedded tool isn't used by any op yet, so it must not appear.
        assert!(app.used_tools().iter().all(|t| t.number != n1));
        // The sample's ops reference T1, which must appear.
        assert!(app.used_tools().iter().any(|t| t.number == 1));

        // Pruning drops the unreferenced embedded tool.
        app.prune_unused_tools();
        assert_eq!(app.document().setup.tools.len(), base);
        assert!(app.document().setup.tools.iter().all(|t| t.number != n1));
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
            app.document().setup.tools.iter().any(|t| t.number == number),
            "pruning must not drop a tool an operation still references"
        );
    }

    #[test]
    fn snap_at_resolves_end_mid_and_nearest_by_priority() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let close = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).abs() < 0.05 && (a[1] - b[1]).abs() < 0.05;

        // End near the (70,50) corner.
        let h = app.snap_at([69.4, 49.4], 1.5, &[SnapKind::End]).unwrap();
        assert_eq!(h.kind, SnapKind::End);
        assert!(close(h.point, [70.0, 50.0]), "end {:?}", h.point);

        // Mid of the right edge (70,10)–(70,50) → (70,30).
        let h = app.snap_at([69.5, 30.0], 1.5, &[SnapKind::Mid]).unwrap();
        assert_eq!(h.kind, SnapKind::Mid);
        assert!(close(h.point, [70.0, 30.0]), "mid {:?}", h.point);

        // Nearest on the right edge, away from corner and mid.
        let h = app.snap_at([69.6, 20.0], 1.5, &[SnapKind::Nearest]).unwrap();
        assert_eq!(h.kind, SnapKind::Nearest);
        assert!(close(h.point, [70.0, 20.0]), "nearest {:?}", h.point);

        // Priority: at a corner with all on, End beats Mid/Nearest.
        let h = app
            .snap_at([69.6, 49.6], 1.5, &[SnapKind::End, SnapKind::Mid, SnapKind::Nearest])
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
        let h = app.snap_at([44.6, 30.0], 1.5, &[SnapKind::Quadrant]).unwrap();
        assert_eq!(h.kind, SnapKind::Quadrant);
        assert!(
            (h.point[0] - 45.0).abs() < 0.1 && (h.point[1] - 30.0).abs() < 0.1,
            "quadrant {:?}",
            h.point
        );
        // The rectangle has corners ⇒ it is not treated as a circle: no quadrant.
        assert!(app.snap_at([69.5, 30.0], 1.5, &[SnapKind::Quadrant]).is_none());
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
                assert!((c[0] - 40.0).abs() < 0.1 && (c[1] - 30.0).abs() < 0.1, "centre {c:?}");
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
        assert_eq!(app.origin_indices(), vec![1, 3], "delete removes just that origin");
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
                assert!((s[0] - 70.0).abs() < 0.05 && (s[1] - 30.0).abs() < 0.05, "start {s:?}");
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
        assert!(app.document().setup.tools.is_empty());
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
        assert_eq!(app.datum_label(6).as_deref(), Some("G59"), "the last ISO offset");
        assert_eq!(app.datum_label(7), None, "and nothing past it");
        app.set_post_kind(PostKind::Okuma);
        assert_eq!(app.datum_label(7).as_deref(), Some("H7"));
    }
}
