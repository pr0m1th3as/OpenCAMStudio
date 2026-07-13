//! The headless application controller.
//!
//! All of the app's behaviour lives here, with no GUI dependency: open a DXF,
//! hold an editable [`Document`] with a [`Selection`], adjust the selected node
//! (undoably), run the strategies, build the viewport scene, simulate the stock,
//! and export G-code. The iced shell is a thin view over this — so the app's
//! logic is unit-tested exactly like the rest of the pipeline.

use cam_geo::Polygon;
use cam_import::{read_dxf_str, ImportError, ImportOptions};
use cam_model::{
    Comp, Document, Heights, History, Machine, Operation, ProfileOp, Setup, Side, Stock, Tool,
    ToolKind,
};
use cam_post::{GrblPost, Post, PostError, PostOptions};
use cam_render::{mesh_vertices, MeshVertex, Scene, PART};
use cam_sim::{simulate, SimOptions};

use cam_cldata::{Program, SpindleDir};
use cam_toolpath::{build_job, CancelToken, Diagnostic, Severity};

/// Seed defaults for a freshly-imported document: the values every generated
/// operation and the setup's heights start from. Once a document exists, editing
/// happens on the document, not here.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JobParams {
    pub tool_diameter: f64,
    pub depth: f64,
    pub stepdown: f64,
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
            feed: 300.0,
            plunge_feed: 100.0,
            spindle_rpm: 1000.0,
            clearance: 5.0,
            retract: 2.0,
            top_of_stock: 0.0,
        }
    }
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
    source_name: String,
    outcome: Option<RunOutcome>,
    nc: Option<String>,
}

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
            source_name: String::new(),
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

    /// Load geometry from DXF text, replacing any current drawing. Generates a
    /// fresh editable document (an outside profile per boundary, an inside
    /// profile per hole) from the seed defaults. Returns the number of regions.
    pub fn open_dxf(&mut self, text: &str, name: impl Into<String>) -> Result<usize, ImportError> {
        let import = read_dxf_str(text, &ImportOptions::default())?;
        self.regions = import.regions;
        self.source_name = name.into();
        let document = self.generate_document();
        self.document = History::new(document);
        self.selection = self
            .document
            .current()
            .setup
            .operations
            .first()
            .map(|op| Selection::Operation(op.id()))
            .unwrap_or(Selection::Setup);
        self.invalidate();
        Ok(self.regions.len())
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
        let document = self.document.current();
        let (program, diagnostics) =
            build_job(document, self.defaults.spindle_rpm, SpindleDir::Cw, cancel);

        let mut scene = Scene::from_program(&program);
        for region in &self.regions {
            scene.add_region(region, PART);
        }

        let (stock_vertices, stock_indices) = self.simulate_stock(&program);

        self.nc = None;
        self.outcome = Some(RunOutcome {
            program,
            diagnostics,
            scene,
            stock_vertices,
            stock_indices,
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
        let options = PostOptions {
            program_name: Some(self.program_name()),
            ..Default::default()
        };
        let nc = GrblPost.post(&outcome.program, &self.machine, &options)?;
        self.nc = Some(nc);
        Ok(self.nc.as_deref().unwrap())
    }

    /// Simulate material removal for `program` and triangulate the remaining
    /// stock into render-ready vertices + indices. Returns empty buffers when
    /// there is no stock (no geometry loaded).
    fn simulate_stock(&self, program: &Program) -> (Vec<MeshVertex>, Vec<u32>) {
        let Stock::Box { min, max } = self.document.current().setup.stock;
        if min[0] >= max[0] || min[1] >= max[1] {
            return (Vec::new(), Vec::new());
        }
        let tool_diameter = self
            .document
            .current()
            .setup
            .tools
            .first()
            .map(|t| t.diameter)
            .unwrap_or(self.defaults.tool_diameter);
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
        );
        let surface = sim.field.to_mesh();
        let vertices = mesh_vertices(&surface.positions, &surface.normals);
        (vertices, surface.indices)
    }

    /// Build a document from the current geometry and seed defaults: an outside
    /// profile for each region's boundary and an inside profile for each hole.
    fn generate_document(&self) -> Document {
        let p = self.defaults;
        let tool = Tool {
            number: 1,
            diameter: p.tool_diameter,
            flutes: 2,
            kind: ToolKind::EndMill,
        };

        let mut operations = Vec::new();
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
            }));
            id += 1;
        };
        for region in &self.regions {
            push(region.outer().clone(), Side::Outside, &mut operations);
            for hole in region.holes() {
                push(hole.clone(), Side::Inside, &mut operations);
            }
        }

        Document::new(Setup {
            name: self.program_name(),
            heights: Heights::new(p.clearance, p.retract, p.top_of_stock),
            stock: self.stock(p.top_of_stock, p.depth),
            tools: vec![tool],
            operations,
        })
    }

    /// A bounding-box stock around the imported geometry.
    fn stock(&self, top: f64, depth: f64) -> Stock {
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
            min = [0.0, 0.0];
            max = [0.0, 0.0];
        }
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
    fn oversized_tool_reports_errors_and_blocks_export() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        // A ⌀12 tool cannot open the 10 mm hole.
        app.edit_tool(0, |t| t.diameter = 12.0);
        let outcome = app.run(&CancelToken::new());
        assert!(outcome.has_errors());
        assert_eq!(app.export_nc(), Err(ExportError::HasErrors));
    }
}
