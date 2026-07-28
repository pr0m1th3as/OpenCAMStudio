//! Job planning: run every operation's strategy and assemble the results into a
//! single, machine-ready CL-data program.
//!
//! This is the thin layer that wraps the pure per-operation strategies with the
//! job-level machine control — tool changes, spindle, coolant — and splices the
//! operation fragments together in order.

use cam_cldata::{Coolant, MoveKind, Point3, Program, SpindleDir, Step, Tag};
use cam_model::{DatumKind, Document, Operation, ReplicationOrder, Setup};

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

    // Compute every operation's fragment once. In replication mode the same fragment
    // is re-emitted under each datum — the coordinates are identical (part frame) and
    // the post applies each work offset — so geometry is never recomputed per fixture.
    let mut planned: Vec<Planned> = Vec::new();
    for operation in &setup.operations {
        if cancel.is_cancelled() {
            break;
        }
        let result = compute(operation, &env, cancel);
        // Stamp ownership here: the planner knows which operation produced these,
        // the strategy does not need to.
        diagnostics.extend(
            result
                .diagnostics
                .into_iter()
                .map(|d| d.for_op(operation.id())),
        );
        if result.program.is_empty() {
            continue;
        }
        // Per-operation spindle speed: the op's own value if set, else the job default
        // (which keeps existing documents — every op at rpm 0 — unchanged).
        let rpm = if operation.spindle_rpm() > 0.0 {
            operation.spindle_rpm()
        } else {
            spindle_rpm
        };
        planned.push(Planned {
            tool: operation.tools()[0],
            rpm,
            // A multi-tool operation (Carve) emits its own tool change mid-fragment;
            // whatever it leaves in the spindle is what the next instance compares
            // against, so read it back rather than assume.
            ends_with: last_tool_change(&result.program),
            datum: operation.work_offset(),
            fragment: result.program,
        });
    }

    // Emit the planned operations. Without replication each runs once, under its own
    // `work_offset`. With replication (Workflow A) the whole list is run across the
    // setup's simultaneous datums, in the chosen order.
    let mut st = EmitState::new();
    match setup.replication {
        None => {
            for pl in &planned {
                emit_instance(&mut program, pl, pl.datum, dir, &mut st);
            }
        }
        // ByTool: fixtures *inside* each operation — op order preserved, and no tool
        // change is added over a single part.
        Some(ReplicationOrder::ByTool) => {
            let datums = simultaneous_datums(setup);
            for pl in &planned {
                for &d in &datums {
                    emit_instance(&mut program, pl, d, dir, &mut st);
                }
            }
        }
        // ByFixture: the whole operation list per fixture — tool loads multiply by the
        // fixture count (the `PL-0-3T.MIN` house style).
        Some(ReplicationOrder::ByFixture) => {
            let datums = simultaneous_datums(setup);
            for &d in &datums {
                for pl in &planned {
                    emit_instance(&mut program, pl, d, dir, &mut st);
                }
            }
        }
    }

    if st.spindle_started {
        program.push(Step::Coolant(Coolant::Off));
        program.push(Step::SpindleOff);
    }

    (program, diagnostics)
}

/// One operation, computed and ready to be emitted — possibly more than once, under
/// different datums, in replication mode. The fragment's coordinates are in the part
/// frame and identical for every datum; only the emitted work offset differs.
struct Planned {
    fragment: Program,
    tool: u32,
    rpm: f64,
    /// The tool a multi-tool fragment (Carve) leaves in the spindle, if any.
    ends_with: Option<u32>,
    /// The operation's own work datum — used only when not replicating.
    datum: u32,
}

/// Running machine state threaded across emitted operation instances, so a tool,
/// spindle speed, datum or coolant is re-commanded only when it actually changes.
struct EmitState {
    spindle_started: bool,
    current_rpm: Option<f64>,
    current_tool: Option<u32>,
    current_datum: u32,
}

impl EmitState {
    fn new() -> Self {
        // Datum 1 is the base WCS and the initial state, so an op on datum 1 emits no
        // `Step::Datum`; a single-datum job's output is unchanged from before multi-WCS.
        Self {
            spindle_started: false,
            current_rpm: None,
            current_tool: None,
            current_datum: 1,
        }
    }
}

/// Emit one operation instance under work datum `d`: the datum select (*before* the
/// tool change, so a section reads datum-then-tool and a post can state `G15 H<n>` in
/// the tool-section head), the tool change, spindle and coolant — each only when it
/// changes — then the operation's motions.
fn emit_instance(program: &mut Program, pl: &Planned, d: u32, dir: SpindleDir, st: &mut EmitState) {
    if st.current_datum != d {
        program.push(Step::Datum(d));
        st.current_datum = d;
    }
    if st.current_tool != Some(pl.tool) {
        program.push(Step::ToolChange { tool: pl.tool });
        st.current_tool = Some(pl.tool);
    }
    if st.current_rpm.is_none_or(|c| (c - pl.rpm).abs() > f64::EPSILON) {
        program.push(Step::Spindle { rpm: pl.rpm, dir });
        st.current_rpm = Some(pl.rpm);
    }
    // Coolant on once, after the spindle first starts.
    if !st.spindle_started {
        program.push(Step::Coolant(Coolant::Flood));
        st.spindle_started = true;
    }
    program.extend(pl.fragment.clone());
    if let Some(t) = pl.ends_with {
        st.current_tool = Some(t);
    }
}

/// The indices of the setup's simultaneous datums — the fixtures replication visits,
/// in registry order. Reorient datums are excluded (they belong to Workflow B, not
/// replication). Falls back to the base datum 1 if none are simultaneous, so a
/// replicated job never silently emits nothing.
fn simultaneous_datums(setup: &Setup) -> Vec<u32> {
    let datums: Vec<u32> = setup
        .work_offsets
        .iter()
        .filter(|d| d.kind == DatumKind::Simultaneous)
        .map(|d| d.index)
        .collect();
    if datums.is_empty() {
        vec![1]
    } else {
        datums
    }
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
