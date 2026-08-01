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
    tool_change_height: f64,
    cancel: &CancelToken,
) -> (Program, Vec<Diagnostic>) {
    let setup = &doc.setup;
    let env = JobEnv {
        heights: setup.heights,
        tools: &setup.tools,
        stock,
        // The same direction the job's `Step::Spindle` will command, so a strategy's
        // climb/conventional choice and the spindle it is cut with cannot disagree.
        spindle: dir,
    };

    let mut program = Program::new();
    let mut diagnostics = Vec::new();

    program.push(Step::Comment(setup.name.clone()));

    // Compute every operation's fragment once. Coordinates are in the part frame; the
    // post re-references each group to its origin.
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
            op_id: operation.id(),
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

    // Group the operations by origin, in the setup's origin order, so each datum's
    // block is contiguous however the operations happen to be ordered in the list — a
    // reorientation is done once, never ping-ponged. A stable sort keeps each group's
    // internal order; an operation whose `work_offset` names no known origin falls into
    // the base group (rank 0), never silently dropped.
    let rank: std::collections::HashMap<u32, usize> = setup
        .origin_indices()
        .into_iter()
        .enumerate()
        .map(|(i, idx)| (idx, i))
        .collect();
    planned.sort_by_key(|pl| rank.get(&pl.datum).copied().unwrap_or(0));

    // Emit each operation once, under the origin its `work_offset` names. Every new
    // context — the first operation, a tool change, or a reorientation — opens by lifting
    // to the tool-change height and descending to clearance (a reorientation adds the
    // `M00` operator stop for re-fixturing); the emitter handles that.
    let mut st = EmitState::new();
    for pl in &planned {
        emit_instance(
            &mut program,
            pl,
            dir,
            &mut st,
            tool_change_height,
            setup.heights.clearance,
        );
    }

    if st.spindle_started {
        program.push(Step::Coolant(Coolant::Off));
        program.push(Step::SpindleOff);
    }

    (program, diagnostics)
}

/// One operation, computed and ready to be emitted. The fragment's coordinates are in
/// the part frame; the post re-references them to the operation's origin.
struct Planned {
    /// The emitting operation's id — for tagging the group's start-point rapid.
    op_id: u32,
    fragment: Program,
    tool: u32,
    rpm: f64,
    /// The tool a multi-tool fragment (Carve) leaves in the spindle, if any.
    ends_with: Option<u32>,
    /// The operation's origin index (its `work_offset`) — the group it belongs to.
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

/// Emit one operation instance, opening every new context from the tool-change height.
///
/// The datum select precedes the tool change so a post can state `G15 H<n>` in the
/// tool-section head; tool/spindle/coolant are re-commanded only on change. Then, for
/// the first operation, a tool change, or a reorientation, the planner crosses to the
/// operation's start at the tool-change height and descends to clearance before handing
/// off — so the tool always reaches a known safe height before descending, whatever the
/// machine's prior state. A reorientation additionally halts (`M00`) for the manual
/// re-fixture, and its cross is left visually broken in the backplot (the pen-lift at the
/// `Datum`). Same-tool, same-datum consecutive operations get no transition — they hop at
/// clearance via the operation's own approach.
fn emit_instance(
    program: &mut Program,
    pl: &Planned,
    dir: SpindleDir,
    st: &mut EmitState,
    tool_change_height: f64,
    clearance: f64,
) {
    let datum_changing = st.current_datum != pl.datum;
    let tool_changing = st.current_tool != Some(pl.tool);

    if datum_changing {
        // A new datum is a physical reorientation of the part. Before the operator
        // handles it, stop the spindle and coolant (`M05`/`M09`) so nothing is live
        // during the flip, lift clear to the tool-change height so the part is free to
        // handle, then halt (`M00`) — but not before the *first* group (nothing to
        // re-fixture from). Force spindle and coolant to be re-commanded for the new
        // group by clearing their running state.
        if st.spindle_started {
            program.push(Step::SpindleOff);
            program.push(Step::Coolant(Coolant::Off));
            lift_to_tool_change(program, tool_change_height, pl.op_id);
            program.push(Step::Stop);
            st.current_rpm = None;
            st.spindle_started = false;
        }
        program.push(Step::Datum(pl.datum));
        st.current_datum = pl.datum;
    } else if tool_changing && st.spindle_started {
        // A same-datum tool change: lift clear to the tool-change height in place before
        // the `M6`, so the ATC swings with the tool out of the work. The first tool load
        // (spindle not yet started) has nothing to lift away from — the cross below still
        // takes it up to the tool-change height first.
        lift_to_tool_change(program, tool_change_height, pl.op_id);
    }
    if tool_changing {
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
    // Open every new context from the tool-change height: cross to this operation's start
    // XY at that height, then descend to clearance and hand off (the operation owns
    // clearance and below). Tagged `Traverse` so the backplot draws it apart. For the
    // first op and same-datum tool changes it draws as one continuous up-over-down; for a
    // reorientation the pen-lift at the `Datum` breaks the cross, leaving it distinct.
    // Same-tool, same-datum consecutive operations skip this and hop at clearance.
    if tool_changing || datum_changing {
        if let Some(start) = first_position(&pl.fragment) {
            program.push(Step::Rapid {
                to: Point3::new(start.x, start.y, tool_change_height),
                tag: Tag::new(pl.op_id, MoveKind::Traverse),
            });
            program.push(Step::Rapid {
                to: Point3::new(start.x, start.y, clearance),
                tag: Tag::new(pl.op_id, MoveKind::Traverse),
            });
        }
    }
    program.extend(pl.fragment.clone());
    if let Some(t) = pl.ends_with {
        st.current_tool = Some(t);
    }
}

/// Rapid straight up to the tool-change height, in place (X/Y unchanged), so a tool
/// change or manual re-fixture happens with the tool clear of the work. Tagged
/// [`MoveKind::Traverse`] so the backplot renders it apart from an operation's own
/// rapids. A no-op if nothing has moved yet — there is no prior position to lift from.
fn lift_to_tool_change(program: &mut Program, z: f64, op_id: u32) {
    if let Some(p) = last_position(program) {
        program.push(Step::Rapid {
            to: Point3::new(p.x, p.y, z),
            tag: Tag::new(op_id, MoveKind::Traverse),
        });
    }
}

/// The tool's current position: the endpoint of the last positional step emitted so
/// far. A drill cycle ends above its last hole, at that cycle's retract plane.
fn last_position(program: &Program) -> Option<Point3> {
    program.steps().iter().rev().find_map(|s| match s {
        Step::Rapid { to, .. } | Step::Linear { to, .. } => Some(*to),
        Step::Arc { end, .. } => Some(*end),
        Step::Drill(c) => c.points.last().map(|[x, y]| Point3::new(*x, *y, c.retract)),
        _ => None,
    })
}

/// The first point an operation's fragment moves to — its approach start. Used to aim
/// the tool-change cross-and-descend at the next operation before handing off.
fn first_position(program: &Program) -> Option<Point3> {
    program.steps().iter().find_map(|s| match s {
        Step::Rapid { to, .. } | Step::Linear { to, .. } => Some(*to),
        Step::Arc { end, .. } => Some(*end),
        Step::Drill(c) => c.points.first().map(|[x, y]| Point3::new(*x, *y, c.retract)),
        _ => None,
    })
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
