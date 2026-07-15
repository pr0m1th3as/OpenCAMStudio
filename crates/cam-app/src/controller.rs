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

use cam_geo::{Contour, Point, Polygon};
use cam_import::{read_cad_file, read_dxf_str, ImportError, ImportOptions};

use crate::project::Project;
use cam_model::{
    ChamferOp, Comp, Document, DrillOp, FaceOp, Hand, Heights, History, Lead, Machine, Operation,
    Plunge, PocketOp, ProfileOp, Setup, Side, Stock, ThreadOp, Tool, ToolKind,
};
use cam_post::{GrblPost, Post, PostError, PostOptions};
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
            depth: -4.0,
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
}

/// Which node of the document is currently selected — what the tree highlights
/// and the inspector edits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Selection {
    /// The setup itself (its heights).
    #[default]
    Setup,
    /// The raw stock.
    Stock,
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
    regions: Vec<Polygon>,
    document: History<Document>,
    defaults: JobParams,
    selection: Selection,
    /// Operation ids excluded from toolpath generation (kept in the tree).
    excluded: BTreeSet<u32>,
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
}

/// A reference to one closed loop (outer or a hole) of one region.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoopRef {
    pub region: usize,
    pub part: LoopPart,
}

/// An operation being created via the pick wizard: the kind, the tool, the picked
/// boundary/path loop (once chosen), and — for a pocket — the loops toggled as
/// excluded islands.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PendingOp {
    pub kind: OpKind,
    pub tool: u32,
    /// The boundary/path loop, set on the first pick. `None` while awaiting it.
    pub boundary: Option<LoopRef>,
    /// Loops toggled as excluded islands (pocket island mode).
    pub islands: Vec<LoopRef>,
}

/// The result of a viewport pick during the wizard.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PickResult {
    /// An operation was created (finalised).
    Created,
    /// A loop was selected/toggled; the wizard is still active (pocket island mode).
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
}

impl std::fmt::Display for ProjectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProjectError::Io(e) => write!(f, "file error: {e}"),
            ProjectError::Json(e) => write!(f, "project format error: {e}"),
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
            regions: Vec::new(),
            document: History::new(empty_document(&defaults)),
            defaults,
            selection: Selection::default(),
            excluded: BTreeSet::new(),
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

    /// The current document.
    pub fn document(&self) -> &Document {
        self.document.current()
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
            Selection::Setup | Selection::Stock => selection,
            _ => Selection::Setup,
        };
    }

    /// The imported regions.
    pub fn regions(&self) -> &[Polygon] {
        &self.regions
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
        self.install_import(import.regions, name.into(), true);
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
        self.install_import(import.regions, name, false);
        Ok(self.regions.len())
    }

    /// Install freshly imported regions: generate a document, select the first
    /// operation (or the Setup when none is seeded), and reset derived state.
    /// Shared by every import path. `seed_ops` seeds a profile per boundary/hole
    /// (the bundled sample demo) versus bringing in bare geometry (real imports).
    fn install_import(&mut self, regions: Vec<Polygon>, name: String, seed_ops: bool) {
        self.regions = regions;
        self.source_name = name;
        let document = self.generate_document(seed_ops);
        self.document = History::new(document);
        self.excluded.clear();
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
        self.document = History::new(empty_document(&self.defaults));
        self.excluded.clear();
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
            defaults: self.defaults,
            source_name: self.source_name.clone(),
        };
        let json = project
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
        let project = Project::from_json(&text).map_err(|e| ProjectError::Json(e.to_string()))?;
        self.regions = project.regions;
        self.defaults = project.defaults;
        self.source_name = project.source_name;
        self.document = History::new(project.document);
        self.excluded.clear();
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
        let used: BTreeSet<u32> = setup.operations.iter().map(|o| o.tool()).collect();
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
        let used: BTreeSet<u32> = self
            .document
            .current()
            .setup
            .operations
            .iter()
            .map(|o| o.tool())
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
        self.document.edit(move |doc| doc.setup.operations.push(op));
        self.selection = Selection::Operation(id);
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
        if self.regions.is_empty() {
            return;
        }
        let tool = self.first_tool_number();
        let boundary = LoopRef {
            region: 0,
            part: LoopPart::Outer,
        };
        if let Some(op) = self.build_op(kind, boundary, &[], tool, None) {
            self.add_operation(op);
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
    ) -> Option<Operation> {
        let chain = self.loop_contour(boundary)?.clone();
        let p = self.defaults;
        // id 0 is a placeholder — add_operation renumbers with a fresh id.
        let op = match kind {
            OpKind::Profile => Operation::Profile(ProfileOp {
                id: 0,
                tool,
                chain,
                side: Side::Outside,
                comp: Comp::Computed,
                depth: p.depth,
                stepdown: p.stepdown,
                feed: p.feed,
                plunge_feed: p.plunge_feed,
                start,
                lead_in: Lead::None,
                lead_out: Lead::None,
                plunge: Plunge::Straight,
            }),
            OpKind::Pocket => {
                let island_contours = islands
                    .iter()
                    .filter_map(|l| self.loop_contour(*l).cloned())
                    .collect();
                Operation::Pocket(PocketOp {
                    id: 0,
                    tool,
                    boundary: chain,
                    islands: island_contours,
                    depth: p.depth,
                    stepdown: p.stepdown,
                    stepover: p.stepover,
                    feed: p.feed,
                    plunge_feed: p.plunge_feed,
                    plunge: Plunge::Straight,
                })
            }
            OpKind::Drill => Operation::Drill(DrillOp {
                id: 0,
                tool,
                points: vec![centroid(&chain)],
                depth: p.depth,
                peck: None,
                dwell: None,
                feed: p.plunge_feed,
            }),
            OpKind::Face => Operation::Face(FaceOp {
                id: 0,
                tool,
                boundary: chain,
                depth: p.depth,
                stepdown: p.stepdown,
                stepover: p.stepover,
                feed: p.feed,
                plunge_feed: p.plunge_feed,
            }),
            OpKind::Chamfer => Operation::Chamfer(ChamferOp {
                id: 0,
                tool,
                chain,
                side: Side::Outside,
                width: 1.0,
                top: p.top_of_stock,
                feed: p.feed,
                plunge_feed: p.plunge_feed,
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
                    z_top: p.top_of_stock,
                    z_bottom: p.depth,
                    climb: true,
                    feed: p.feed,
                    plunge_feed: p.plunge_feed,
                })
            }
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
    pub fn loop_contour(&self, l: LoopRef) -> Option<&Contour> {
        let region = self.regions.get(l.region)?;
        match l.part {
            LoopPart::Outer => Some(region.outer()),
            LoopPart::Hole(i) => region.holes().get(i),
        }
    }

    /// Begin creating an operation of `kind`: enter geometry-pick mode. A no-op if no
    /// geometry is loaded. The tool need not be embedded yet — it is picked from the
    /// library during setup (the GUI seeds a default and calls [`Self::use_tool`]);
    /// `tool` starts at the first embedded tool's number, or 1 as a placeholder.
    pub fn begin_operation(&mut self, kind: OpKind) {
        if self.regions.is_empty() {
            return;
        }
        self.pending_op = Some(PendingOp {
            kind,
            tool: self.first_tool_number(),
            boundary: None,
            islands: Vec::new(),
        });
    }

    /// Change the tool of the pending operation. A no-op unless the wizard is active.
    pub fn set_pending_tool(&mut self, number: u32) {
        if let Some(pending) = self.pending_op.as_mut() {
            pending.tool = number;
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

    /// Complete or advance the pending operation from a viewport pick at `world`
    /// with a pickbox of `aperture` world-mm.
    /// - The **first** pick selects the boundary/path loop under the box. For every
    ///   kind except Pocket the operation is created immediately.
    /// - For a **Pocket**, the first pick sets the boundary and the wizard stays in
    ///   island mode; each further pick toggles that loop as an excluded island
    ///   (the boundary loop itself is ignored). [`confirm_operation`] finalises it.
    pub fn pick_operation_geometry(&mut self, world: [f64; 2], aperture: f64) -> PickResult {
        let Some(pending) = self.pending_op.clone() else {
            return PickResult::Missed;
        };
        let Some((picked, start)) = self.nearest_loop(world, aperture) else {
            return PickResult::Missed;
        };
        match pending.boundary {
            None => {
                if pending.kind == OpKind::Pocket {
                    self.pending_op.as_mut().unwrap().boundary = Some(picked);
                    PickResult::Selecting
                } else if let Some(op) =
                    self.build_op(pending.kind, picked, &[], pending.tool, Some(start))
                {
                    self.add_operation(op);
                    self.pending_op = None;
                    PickResult::Created
                } else {
                    PickResult::Missed
                }
            }
            Some(boundary) => {
                // Pocket island mode: toggle the clicked loop (not the boundary).
                if picked == boundary {
                    return PickResult::Selecting;
                }
                let islands = &mut self.pending_op.as_mut().unwrap().islands;
                if let Some(pos) = islands.iter().position(|l| *l == picked) {
                    islands.remove(pos);
                } else {
                    islands.push(picked);
                }
                PickResult::Selecting
            }
        }
    }

    /// Finalise a pending operation from its picked boundary + islands (used by the
    /// Pocket wizard's Confirm). Returns `true` if an operation was created.
    pub fn confirm_operation(&mut self) -> bool {
        let Some(pending) = self.pending_op.clone() else {
            return false;
        };
        let Some(boundary) = pending.boundary else {
            return false;
        };
        if let Some(op) =
            self.build_op(pending.kind, boundary, &pending.islands, pending.tool, None)
        {
            self.add_operation(op);
            self.pending_op = None;
            return true;
        }
        false
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
                let j = if up {
                    i.checked_sub(1)
                } else if i + 1 < ops.len() {
                    Some(i + 1)
                } else {
                    None
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
        // Excluded operations are dropped from the job (kept in the tree, just
        // not machined) by running a filtered copy of the document.
        let base = self.document.current();
        let document = if self.excluded.is_empty() {
            Cow::Borrowed(base)
        } else {
            let mut d = base.clone();
            d.setup
                .operations
                .retain(|o| !self.excluded.contains(&o.id()));
            Cow::Owned(d)
        };
        let (program, diagnostics) =
            build_job(&document, self.defaults.spindle_rpm, SpindleDir::Cw, cancel);

        let mut scene = Scene::from_program(&program);
        for region in &self.regions {
            scene.add_region(region, PART);
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
        let options = PostOptions {
            program_name: Some(self.program_name()),
            ..Default::default()
        };
        let nc = GrblPost.post(&outcome.program, &self.machine, &options)?;
        self.nc = Some(nc);
        Ok(self.nc.as_deref().unwrap())
    }

    /// Simulate material removal for `program`: triangulate the remaining stock
    /// into render-ready vertices + indices, and return any collisions found.
    /// Empty when there is no stock (no geometry loaded).
    fn simulate_stock(&self, program: &Program) -> (Vec<MeshVertex>, Vec<u32>, Vec<Collision>) {
        let Stock::Box { min, max } = self.document.current().setup.stock;
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
        // Aim for ~200 cells across the larger side, bounded so a big part stays
        // cheap and a small one stays crisp.
        let span = (max[0] - min[0]).max(max[1] - min[1]);
        let resolution = (span / 200.0).clamp(0.25, 2.0);
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
        let surface = sim.field.to_mesh();
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
            }]
        } else {
            Vec::new()
        };

        let mut operations = Vec::new();
        if seed_ops {
            let mut id = 0u32;
            let mut push = |chain, side, operations: &mut Vec<Operation>| {
                operations.push(Operation::Profile(ProfileOp {
                    id,
                    tool: 1,
                    chain,
                    side,
                    comp: Comp::Computed,
                    depth: p.depth,
                    stepdown: p.stepdown,
                    feed: p.feed,
                    plunge_feed: p.plunge_feed,
                    start: None,
                    lead_in: Lead::None,
                    lead_out: Lead::None,
                    plunge: Plunge::Straight,
                }));
                id += 1;
            };
            for region in &self.regions {
                push(region.outer().clone(), Side::Outside, &mut operations);
                for hole in region.holes() {
                    push(hole.clone(), Side::Inside, &mut operations);
                }
            }
        }

        Document::new(Setup {
            name: self.program_name(),
            heights: Heights::new(p.clearance, p.retract, p.top_of_stock),
            stock: self.stock(p.top_of_stock, p.depth),
            tools,
            operations,
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
        if min[0] > max[0] {
            ([0.0, 0.0], [0.0, 0.0])
        } else {
            (min, max)
        }
    }

    /// A bounding-box stock around the imported geometry.
    fn stock(&self, top: f64, depth: f64) -> Stock {
        let (min, max) = self.bounds_xy();
        Stock::Box {
            min: [min[0], min[1], depth],
            max: [max[0], max[1], top],
        }
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

/// Whether two tools have the same cutting geometry, ignoring their numbers — used
/// to dedupe when embedding a library tool into a project's setup.
fn same_tool_geometry(a: &Tool, b: &Tool) -> bool {
    a.diameter == b.diameter && a.length == b.length && a.flutes == b.flutes && a.kind == b.kind
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
        // A thread mill's material removal is approximated by its footprint.
        ToolKind::ThreadMill { .. } => ProfileShape::Flat,
    };
    ToolProfile { radius, shape }
}

/// Set an operation's id, whatever its kind.
fn set_op_id(op: &mut Operation, id: u32) {
    match op {
        Operation::Profile(o) => o.id = id,
        Operation::Drill(o) => o.id = id,
        Operation::Pocket(o) => o.id = id,
        Operation::Face(o) => o.id = id,
        Operation::Chamfer(o) => o.id = id,
        Operation::Thread(o) => o.id = id,
    }
}

/// An empty starting document: a zero-size box stock, one default end mill, no
/// operations.
fn empty_document(p: &JobParams) -> Document {
    Document::new(Setup {
        name: "Untitled".to_string(),
        heights: Heights::new(p.clearance, p.retract, p.top_of_stock),
        stock: Stock::Box {
            min: [0.0, 0.0, p.depth],
            max: [0.0, 0.0, p.top_of_stock],
        },
        tools: vec![Tool {
            number: 1,
            diameter: p.tool_diameter,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
        }],
        operations: Vec::new(),
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

    fn depth_of(op: &Operation) -> f64 {
        match op {
            Operation::Profile(o) => o.depth,
            Operation::Pocket(o) => o.depth,
            Operation::Face(o) => o.depth,
            Operation::Drill(o) => o.depth,
            Operation::Thread(o) => o.z_bottom,
            Operation::Chamfer(o) => o.top,
        }
    }

    #[test]
    fn open_generates_ops_and_selects_the_first() {
        let mut app = AppController::new(machine());
        assert_eq!(app.open_dxf(PART_DXF, "part.dxf").unwrap(), 1);
        // Rectangle boundary (outside) + circular hole (inside) = 2 profiles.
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
                o.depth = -8.0;
            }
        });
        assert_eq!(depth_of(app.operation(0).unwrap()), -8.0);
        assert!(app.outcome().is_none(), "editing invalidates the stale run");
        assert!(app.can_undo());

        assert!(app.undo());
        assert_eq!(depth_of(app.operation(0).unwrap()), -4.0, "undo restores");
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
                assert!(o.stepover > 0.0);
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
    fn project_json_is_stable() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        let project = Project {
            schema_version: cam_model::SCHEMA_VERSION,
            document: app.document().clone(),
            regions: app.regions().to_vec(),
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
            app.pick_operation_geometry([30.0, 25.0], 2.0),
            PickResult::Missed
        );
        assert!(app.pending_op().is_some());

        // Near the (70,50) corner → a profile on the outer loop, tool 2, start there.
        assert_eq!(
            app.pick_operation_geometry([69.6, 49.6], 2.0),
            PickResult::Created
        );
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
        // Click on the circle edge (centre 40,30, r5) → the profile follows the hole.
        assert_eq!(
            app.pick_operation_geometry([45.4, 30.0], 2.0),
            PickResult::Created
        );
        let hole = app.regions()[0].holes()[0].clone();
        match app.selected_operation() {
            Some(Operation::Profile(o)) => assert_eq!(o.chain, hole, "chain is the circle"),
            other => panic!("expected a profile on the circle, got {other:?}"),
        }
    }

    #[test]
    fn pocket_wizard_picks_a_boundary_then_toggles_islands() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.begin_operation(OpKind::Pocket);

        // First pick = the boundary (outer). Stays in island mode.
        assert_eq!(
            app.pick_operation_geometry([10.4, 25.0], 2.0),
            PickResult::Selecting
        );
        let pending = app.pending_op().unwrap();
        assert_eq!(pending.boundary.unwrap().part, LoopPart::Outer);
        assert!(pending.islands.is_empty());

        // Click the circle → adds it as an island; clicking again removes it.
        app.pick_operation_geometry([45.4, 30.0], 2.0);
        assert_eq!(app.pending_op().unwrap().islands.len(), 1);
        app.pick_operation_geometry([45.4, 30.0], 2.0);
        assert!(app.pending_op().unwrap().islands.is_empty());

        // Re-add it and confirm → a pocket bounded by the rectangle with one island.
        app.pick_operation_geometry([45.4, 30.0], 2.0);
        assert!(app.confirm_operation());
        assert!(app.pending_op().is_none());
        match app.selected_operation() {
            Some(Operation::Pocket(o)) => assert_eq!(o.islands.len(), 1),
            other => panic!("expected a pocket with one island, got {other:?}"),
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
}
