//! Replication (Workflow A): one operation list run across several *simultaneous*
//! work datums, authored once and expanded by the planner. Two orders — `ByTool`
//! (fixtures inside each op; op order preserved, no extra tool changes) and
//! `ByFixture` (the whole list per fixture; tool loads multiply).

use cam_cldata::{SpindleDir, Step};
use cam_geo::{Contour, Point};
use cam_model::{
    Comp, Datum, DatumKind, Document, Heights, Lead, Operation, Plunge, ProfileOp, ReplicationOrder,
    Setup, Side, Stock, Tool, ToolKind,
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

fn profile(id: u32, chain: Contour, side: Side, tool: u32) -> Operation {
    Operation::Profile(ProfileOp {
        spindle_rpm: 0.0,
        work_offset: 1,
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

fn sim(index: u32) -> Datum {
    Datum {
        index,
        label: String::new(),
        kind: DatumKind::Simultaneous,
    }
}

/// Two profiles on two different tools — enough that the tool-change count separates
/// the two replication orders.
fn two_op_doc(work_offsets: Vec<Datum>, replication: Option<ReplicationOrder>) -> Document {
    Document::new(Setup {
        name: "rep".into(),
        heights: Heights::new(5.0, 2.0, 0.0),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: 0.0,
            thickness: 10.0,
        },
        tools: vec![tool(1), tool(2)],
        operations: vec![
            profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1),
            profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 2),
        ],
        origin: [0.0, 0.0, 0.0],
        start_offset: None,
        work_offsets,
        replication,
    })
}

fn program(doc: &Document) -> Vec<Step> {
    build_job(doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new())
        .0
        .steps()
        .to_vec()
}

/// The datum-select and tool-change markers, in order — the skeleton that tells the
/// two orders apart. `H<n>` = `Step::Datum(n)`, `T<n>` = `Step::ToolChange`.
fn markers(steps: &[Step]) -> Vec<String> {
    steps
        .iter()
        .filter_map(|s| match s {
            Step::Datum(n) => Some(format!("H{n}")),
            Step::ToolChange { tool } => Some(format!("T{tool}")),
            _ => None,
        })
        .collect()
}

fn tool_changes(steps: &[Step]) -> usize {
    steps
        .iter()
        .filter(|s| matches!(s, Step::ToolChange { .. }))
        .count()
}

#[test]
fn by_tool_runs_each_op_across_all_fixtures_with_no_extra_tool_changes() {
    // O1(T1), O2(T2) across H1,H2,H3. ByTool keeps op order and loads each tool once:
    // O1 on all three fixtures, then O2 on all three. Datum 1 is the initial state, so
    // O1's first instance emits no H marker.
    let doc = two_op_doc(vec![sim(1), sim(2), sim(3)], Some(ReplicationOrder::ByTool));
    let m = markers(&program(&doc));
    assert_eq!(m, ["T1", "H2", "H3", "H1", "T2", "H2", "H3"], "fixtures inside each op");
    // The headline property: two tools, two loads — same as a single un-replicated part.
    assert_eq!(tool_changes(&program(&doc)), 2, "no tool change added by replication");
}

#[test]
fn by_fixture_runs_the_whole_list_per_fixture_and_multiplies_tool_loads() {
    // The same job ByFixture: the whole [O1,O2] list under each datum. Tool loads
    // multiply by the fixture count (2 tools x 3 fixtures = 6).
    let doc = two_op_doc(vec![sim(1), sim(2), sim(3)], Some(ReplicationOrder::ByFixture));
    let m = markers(&program(&doc));
    assert_eq!(m, ["T1", "T2", "H2", "T1", "T2", "H3", "T1", "T2"], "whole list per fixture");
    assert_eq!(tool_changes(&program(&doc)), 6, "tool loads multiply by fixture count");
}

#[test]
fn replicating_across_a_single_datum_matches_no_replication() {
    // A registry with only the base datum is a no-op: replication produces exactly the
    // same program as a plain single-datum job, whichever order is asked for.
    let plain = program(&two_op_doc(vec![sim(1)], None));
    let by_tool = program(&two_op_doc(vec![sim(1)], Some(ReplicationOrder::ByTool)));
    let by_fixture = program(&two_op_doc(vec![sim(1)], Some(ReplicationOrder::ByFixture)));
    assert_eq!(by_tool, plain, "ByTool across one datum == no replication");
    assert_eq!(by_fixture, plain, "ByFixture across one datum == no replication");
}

#[test]
fn reorient_datums_are_excluded_from_replication() {
    // Replication visits only *simultaneous* datums; a Reorient datum in the registry
    // (Workflow B territory) is not a fixture to repeat across.
    let mut offsets = vec![sim(1), sim(2)];
    offsets.push(Datum {
        index: 3,
        label: "flip".into(),
        kind: DatumKind::Reorient,
    });
    let doc = two_op_doc(offsets, Some(ReplicationOrder::ByTool));
    let m = markers(&program(&doc));
    assert!(!m.contains(&"H3".to_string()), "the Reorient datum H3 is not replicated: {m:?}");
    // It still replicates across the two simultaneous datums (H2 appears; H1 is implicit).
    assert!(m.contains(&"H2".to_string()), "simultaneous datums are still replicated: {m:?}");
}
