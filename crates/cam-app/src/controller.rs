//! The headless application controller.
//!
//! All of the app's behaviour lives here, with no GUI dependency: open a DXF,
//! adjust job parameters (undoably), run the strategies, build the viewport
//! scene, and export G-code. The iced shell is a thin view over this — so the
//! app's logic is unit-tested exactly like the rest of the pipeline.

use cam_geo::Polygon;
use cam_import::{read_dxf_str, ImportError, ImportOptions};
use cam_model::{
    Comp, Document, Heights, History, Machine, Operation, ProfileOp, Setup, Side, Stock, Tool,
    ToolKind,
};
use cam_post::{GrblPost, Post, PostError, PostOptions};
use cam_render::{Scene, PART};

use cam_cldata::{Program, SpindleDir};
use cam_toolpath::{build_job, CancelToken, Diagnostic, Severity};

/// The user-tunable parameters of the job. A single global set is applied to
/// every profiled contour — enough for the first interactive vertical.
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

/// The result of a run: the CL-data program, its diagnostics, and the viewport
/// scene (part outlines + backplot).
#[derive(Clone, Debug, Default)]
pub struct RunOutcome {
    pub program: Program,
    pub diagnostics: Vec<Diagnostic>,
    pub scene: Scene,
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
    params: History<JobParams>,
    source_name: String,
    outcome: Option<RunOutcome>,
    nc: Option<String>,
}

impl AppController {
    /// A fresh controller for `machine`, with default parameters and no geometry.
    pub fn new(machine: Machine) -> Self {
        Self {
            machine,
            regions: Vec::new(),
            params: History::new(JobParams::default()),
            source_name: String::new(),
            outcome: None,
            nc: None,
        }
    }

    /// The machine being driven.
    pub fn machine(&self) -> &Machine {
        &self.machine
    }

    /// The current job parameters.
    pub fn params(&self) -> &JobParams {
        self.params.current()
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

    /// Load geometry from DXF text, replacing any current drawing. Returns the
    /// number of regions imported. Clears the run/undo state for a clean start.
    pub fn open_dxf(&mut self, text: &str, name: impl Into<String>) -> Result<usize, ImportError> {
        let import = read_dxf_str(text, &ImportOptions::default())?;
        self.regions = import.regions;
        self.source_name = name.into();
        self.params = History::new(*self.params.current());
        self.invalidate();
        Ok(self.regions.len())
    }

    /// Edit the job parameters as one undoable change. Any stale run is dropped.
    pub fn edit_params(&mut self, f: impl FnOnce(&mut JobParams)) {
        self.params.edit(f);
        self.invalidate();
    }

    /// Undo the last parameter edit.
    pub fn undo(&mut self) -> bool {
        let changed = self.params.undo();
        if changed {
            self.invalidate();
        }
        changed
    }

    /// Redo an undone parameter edit.
    pub fn redo(&mut self) -> bool {
        let changed = self.params.redo();
        if changed {
            self.invalidate();
        }
        changed
    }

    /// Whether an undo / redo is available.
    pub fn can_undo(&self) -> bool {
        self.params.can_undo()
    }
    pub fn can_redo(&self) -> bool {
        self.params.can_redo()
    }

    /// Run the strategies for the current geometry and parameters, producing the
    /// program, diagnostics, and viewport scene. Returns a reference to the
    /// outcome. `cancel` allows a long run to be aborted.
    pub fn run(&mut self, cancel: &CancelToken) -> &RunOutcome {
        let document = self.build_document();
        let (program, diagnostics) = build_job(
            &document,
            self.params.current().spindle_rpm,
            SpindleDir::Cw,
            cancel,
        );

        let mut scene = Scene::from_program(&program);
        for region in &self.regions {
            scene.add_region(region, PART);
        }

        self.nc = None;
        self.outcome = Some(RunOutcome {
            program,
            diagnostics,
            scene,
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

    /// Turn the current geometry + parameters into a document: an outside profile
    /// for each region's boundary and an inside profile for each of its holes.
    fn build_document(&self) -> Document {
        let p = *self.params.current();
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

    /// Drop any run/export that no longer reflects the inputs.
    fn invalidate(&mut self) {
        self.outcome = None;
        self.nc = None;
    }
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

    #[test]
    fn open_run_export_round_trip() {
        let mut app = AppController::new(machine());
        assert_eq!(app.open_dxf(PART_DXF, "part.dxf").unwrap(), 1);
        assert_eq!(app.regions().len(), 1);

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
    fn param_edits_undo_and_invalidate_the_run() {
        let mut app = AppController::new(machine());
        app.open_dxf(PART_DXF, "part.dxf").unwrap();
        app.run(&CancelToken::new());
        assert!(app.outcome().is_some());

        app.edit_params(|p| p.depth = -8.0);
        assert_eq!(app.params().depth, -8.0);
        assert!(app.outcome().is_none(), "editing invalidates the stale run");
        assert!(app.can_undo());

        assert!(app.undo());
        assert_eq!(app.params().depth, -4.0, "undo restores the depth");
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
        app.edit_params(|p| p.tool_diameter = 12.0);
        let outcome = app.run(&CancelToken::new());
        assert!(outcome.has_errors());
        assert_eq!(app.export_nc(), Err(ExportError::HasErrors));
    }
}
