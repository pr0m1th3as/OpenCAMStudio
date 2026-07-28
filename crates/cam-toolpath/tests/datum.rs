//! Per-operation work datum (O2): the planner emits `Step::Datum` for an
//! operation whose `work_offset` differs from the one in force, placed *before*
//! that operation's tool change so a post can state `G15 H<n>` in the tool-section
//! head. Datum 1 is the default, so a single-datum job emits none — every post's
//! output is then byte-identical to before multi-WCS existed.

use cam_cldata::{SpindleDir, Step};
use cam_geo::{Contour, Point};
use cam_model::{
    Comp, Document, Heights, Lead, Operation, Plunge, ProfileOp, Setup, Side, Stock, Tool, ToolKind,
};
use cam_toolpath::{build_job, CancelToken};

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Contour {
    Contour::new(vec![
        Point::new(x0, y0),
        Point::new(x1, y0),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ])
}

fn profile(id: u32, chain: Contour, side: Side, tool: u32, work_offset: u32) -> Operation {
    Operation::Profile(ProfileOp {
        spindle_rpm: 0.0,
        work_offset,
        clearing: cam_model::Clearing::default(),
        id,
        tool,
        chain,
        side,
        comp: Comp::Computed,
        offset: 0.0,
        depth: 4.0,
        stepdown: 2.0,
        stepover: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: 0.0,
        plunge: Plunge::Straight,
    })
}

fn tool(number: u32) -> Tool {
    Tool {
        number,
        diameter: 6.0,
        length: 30.0,
        flutes: 2,
        kind: ToolKind::EndMill,
        ..Default::default()
    }
}

fn doc(ops: Vec<Operation>) -> Document {
    Document::new(Setup {
        name: "datum".into(),
        heights: Heights::new(5.0, 2.0, 0.0),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: 0.0,
            thickness: 10.0,
        },
        tools: vec![tool(1), tool(2)],
        operations: ops,
        origin: [0.0, 0.0, 0.0],
        start_offset: None,
        work_offsets: vec![cam_model::Datum::base()],
        replication: None,
    })
}

fn steps(doc: &Document) -> Vec<Step> {
    let (program, _) = build_job(doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    program.steps().to_vec()
}

/// The datum index of every `Step::Datum` the job emits, in order.
fn datums(doc: &Document) -> Vec<u32> {
    steps(doc)
        .iter()
        .filter_map(|s| match s {
            Step::Datum(n) => Some(*n),
            _ => None,
        })
        .collect()
}

#[test]
fn a_single_datum_job_emits_no_datum_step() {
    // Every op on datum 1 (the default) -> no Step::Datum at all, so the CL-data and
    // therefore every post's output is unchanged from before multi-WCS.
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 1, 1),
    ]);
    assert!(datums(&d).is_empty(), "no datum steps for a single-datum job");
}

#[test]
fn a_datum_change_between_ops_emits_one_step_datum() {
    // Two ops on datums 1 then 2 -> a single Step::Datum(2). Datum 1 is implicit
    // (the initial state), so only the *change* is recorded.
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 1, 2),
    ]);
    assert_eq!(datums(&d), vec![2]);
}

#[test]
fn the_first_op_on_a_non_default_datum_still_emits_it() {
    // If the very first operation is on datum 2, the change from the implicit datum 1
    // is real and must be emitted — otherwise it would silently run on the wrong WCS.
    let d = doc(vec![profile(
        0,
        rect(10.0, 10.0, 70.0, 50.0),
        Side::Outside,
        1,
        2,
    )]);
    assert_eq!(datums(&d), vec![2]);
}

#[test]
fn the_datum_step_precedes_the_tool_change_for_that_operation() {
    // op0: tool 1, datum 1. op1: tool 2, datum 2. The datum select must come before
    // op1's tool change so the section reads datum-then-tool.
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 2, 2),
    ]);
    let s = steps(&d);
    let datum_at = s.iter().position(|x| matches!(x, Step::Datum(2))).expect("a datum step");
    let tc2_at = s
        .iter()
        .position(|x| matches!(x, Step::ToolChange { tool: 2 }))
        .expect("op1's tool change");
    assert!(datum_at < tc2_at, "datum select precedes the tool change:\n{s:#?}");
    // Nothing is emitted between them — they form the section head together.
    assert_eq!(datum_at + 1, tc2_at, "datum sits immediately before the tool change");
}
