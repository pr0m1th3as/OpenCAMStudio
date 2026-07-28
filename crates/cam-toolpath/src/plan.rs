//! Job planning: run every operation's strategy and assemble the results into a
//! single, machine-ready CL-data program.
//!
//! This is the thin layer that wraps the pure per-operation strategies with the
//! job-level machine control — tool changes, spindle, coolant — and splices the
//! operation fragments together in order.

use cam_cldata::{Coolant, MoveKind, Point3, Program, SpindleDir, Step, Tag};
use cam_model::{Document, Operation};

use crate::{
    CancelToken, CarveStrategy, ChamferStrategy, Diagnostic, DrillStrategy, EngraveStrategy,
    FaceStrategy, JobEnv, PocketStrategy, ProfileStrategy, Strategy, StrategyResult,
    ThreadStrategy,
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
    stock: Option<([f64; 2], [f64; 2])>,
    cancel: &CancelToken,
) -> (Program, Vec<Diagnostic>) {
    let setup = &doc.setup;
    let env = JobEnv {
        heights: setup.heights,
        tools: &setup.tools,
        stock,
    };

    let mut program = Program::new();
    let mut diagnostics = Vec::new();

    program.push(Step::Comment(setup.name.clone()));

    // Optional program start point: begin with a rapid to origin + offset, so the
    // toolpath's first motion originates at a known safe spot. Tagged to the first
    // op (Link) so it colours as a rapid.
    if let Some(off) = setup.start_offset {
        let o = setup.origin;
        let op_id = setup.operations.first().map_or(0, Operation::id);
        program.push(Step::Rapid {
            to: Point3::new(o[0] + off[0], o[1] + off[1], o[2] + off[2]),
            tag: Tag::new(op_id, MoveKind::Link),
        });
    }

    let mut spindle_started = false;
    let mut current_rpm: Option<f64> = None;
    let mut current_tool: Option<u32> = None;

    for operation in &setup.operations {
        if cancel.is_cancelled() {
            break;
        }

        let result = compute(operation, &env, cancel);
        let fragment = result.program;
        // Stamp ownership here: the planner knows which operation produced these,
        // the strategy does not need to.
        let op_id = operation.id();
        diagnostics.extend(result.diagnostics.into_iter().map(|d| d.for_op(op_id)));
        if fragment.is_empty() {
            continue;
        }

        // Tool change when the operation's *first* tool differs from the one in the
        // spindle. A multi-tool operation orders its own subsequent changes — see the
        // resync after the fragment is appended.
        let tool = operation.tools()[0];
        if current_tool != Some(tool) {
            program.push(Step::ToolChange { tool });
            current_tool = Some(tool);
        }

        // Spindle speed is per-operation: the op's own value if set, else the job
        // default (which keeps existing documents — every op at rpm 0 — unchanged).
        // Re-command M3 S whenever the effective speed changes between operations, so
        // a slow drill after a fast profile spins at its own rpm rather than inheriting.
        let rpm = if operation.spindle_rpm() > 0.0 {
            operation.spindle_rpm()
        } else {
            spindle_rpm
        };
        if current_rpm.is_none_or(|c| (c - rpm).abs() > f64::EPSILON) {
            program.push(Step::Spindle { rpm, dir });
            current_rpm = Some(rpm);
        }
        // Coolant on once, after the spindle first starts.
        if !spindle_started {
            program.push(Step::Coolant(Coolant::Flood));
            spindle_started = true;
        }

        // A multi-tool operation (Carve) emits its own tool change mid-fragment, because
        // only it knows the order its tools must run in. Whatever it left in the spindle
        // is what the *next* operation compares against — read it back rather than
        // assuming, or the next operation would be handed a stale tool number and its
        // change silently omitted.
        let ends_with = last_tool_change(&fragment);

        program.extend(fragment);

        if let Some(t) = ends_with {
            current_tool = Some(t);
        }
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
        Operation::Engrave(op) => EngraveStrategy::new(op.clone()).compute(env, cancel),
        Operation::Carve(op) => CarveStrategy::new(op.clone()).compute(env, cancel),
    }
}

/// The tool left in the spindle by `fragment` — the last [`Step::ToolChange`] it emits,
/// or `None` if it emits none (the single-tool case, which is every operation but Carve).
fn last_tool_change(fragment: &Program) -> Option<u32> {
    fragment.steps().iter().rev().find_map(|s| match s {
        Step::ToolChange { tool } => Some(*tool),
        _ => None,
    })
}
