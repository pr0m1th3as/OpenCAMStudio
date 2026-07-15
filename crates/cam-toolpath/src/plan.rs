//! Job planning: run every operation's strategy and assemble the results into a
//! single, machine-ready CL-data program.
//!
//! This is the thin layer that wraps the pure per-operation strategies with the
//! job-level machine control — tool changes, spindle, coolant — and splices the
//! operation fragments together in order.

use cam_cldata::{Coolant, MoveKind, Point3, Program, SpindleDir, Step, Tag};
use cam_model::{Document, Operation};

use crate::{
    CancelToken, ChamferStrategy, Diagnostic, DrillStrategy, FaceStrategy, JobEnv, PocketStrategy,
    ProfileStrategy, Strategy, StrategyResult, ThreadStrategy,
};

/// Assemble a whole-job [`Program`] from a [`Document`].
///
/// Each operation is turned into its strategy, computed against the setup's
/// heights and tools, and its motions are spliced between a job preamble (tool
/// change, spindle on, coolant on) and postamble (coolant off, spindle off). A
/// tool change is inserted whenever the tool number changes between operations.
///
/// Diagnostics from every operation are collected; if any operation errors, its
/// motions are omitted but planning continues so the caller sees all problems at
/// once. Returns the program and the combined diagnostics.
pub fn build_job(
    doc: &Document,
    spindle_rpm: f64,
    dir: SpindleDir,
    cancel: &CancelToken,
) -> (Program, Vec<Diagnostic>) {
    let setup = &doc.setup;
    let env = JobEnv {
        heights: setup.heights,
        tools: &setup.tools,
    };

    let mut program = Program::new();
    let mut diagnostics = Vec::new();

    program.push(Step::Comment(setup.name.clone()));

    // Optional program start point: begin with a rapid to it, so the toolpath's
    // first motion originates at a known safe spot. Resolved from its base +
    // offset; tagged to the first op (Link) so it colours as a rapid.
    if let Some(sp) = setup.start_point {
        let [x, y, z] = sp.resolve(setup.origin);
        let op_id = setup.operations.first().map_or(0, Operation::id);
        program.push(Step::Rapid {
            to: Point3::new(x, y, z),
            tag: Tag::new(op_id, MoveKind::Link),
        });
    }

    let mut spindle_started = false;
    let mut current_tool: Option<u32> = None;

    for operation in &setup.operations {
        if cancel.is_cancelled() {
            break;
        }

        let result = compute(operation, &env, cancel);
        let fragment = result.program;
        diagnostics.extend(result.diagnostics);
        if fragment.is_empty() {
            continue;
        }

        // Tool change when the tool differs from the one in the spindle.
        let tool = operation_tool(operation);
        if current_tool != Some(tool) {
            program.push(Step::ToolChange { tool });
            current_tool = Some(tool);
        }

        // Spindle + coolant on, once, before the first cutting.
        if !spindle_started {
            program.push(Step::Spindle {
                rpm: spindle_rpm,
                dir,
            });
            program.push(Step::Coolant(Coolant::Flood));
            spindle_started = true;
        }

        program.extend(fragment);
    }

    if spindle_started {
        program.push(Step::Coolant(Coolant::Off));
        program.push(Step::SpindleOff);
    }

    (program, diagnostics)
}

/// Dispatch an operation to its strategy and compute it.
fn compute(operation: &Operation, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
    match operation {
        Operation::Profile(op) => ProfileStrategy::new(op.clone()).compute(env, cancel),
        Operation::Drill(op) => DrillStrategy::new(op.clone()).compute(env, cancel),
        Operation::Pocket(op) => PocketStrategy::new(op.clone()).compute(env, cancel),
        Operation::Face(op) => FaceStrategy::new(op.clone()).compute(env, cancel),
        Operation::Chamfer(op) => ChamferStrategy::new(op.clone()).compute(env, cancel),
        Operation::Thread(op) => ThreadStrategy::new(op.clone()).compute(env, cancel),
    }
}

fn operation_tool(operation: &Operation) -> u32 {
    match operation {
        Operation::Profile(op) => op.tool,
        Operation::Drill(op) => op.tool,
        Operation::Pocket(op) => op.tool,
        Operation::Face(op) => op.tool,
        Operation::Chamfer(op) => op.tool,
        Operation::Thread(op) => op.tool,
    }
}
