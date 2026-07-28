//! Per-operation spindle speed: the planner emits `Step::Spindle` for each
//! operation's own RPM, re-commanding it whenever the effective speed changes, and
//! falling back to the job default when an operation leaves its RPM unset (0).

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

fn profile(id: u32, chain: Contour, side: Side, spindle_rpm: f64) -> Operation {
    Operation::Profile(ProfileOp {
        spindle_rpm,
        clearing: cam_model::Clearing::default(),
        id,
        tool: 1,
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

fn doc(ops: Vec<Operation>) -> Document {
    Document::new(Setup {
        name: "spindle".into(),
        heights: Heights::new(5.0, 2.0, 0.0),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: 0.0,
            thickness: 10.0,
        },
        tools: vec![Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }],
        operations: ops,
        origin: [0.0, 0.0, 0.0],
        start_offset: None,
    })
}

/// The RPMs of every `Step::Spindle` the job emits, in order.
fn spindle_rpms(doc: &Document) -> Vec<f64> {
    let (program, _) = build_job(doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    program
        .steps()
        .iter()
        .filter_map(|s| match s {
            Step::Spindle { rpm, .. } => Some(*rpm),
            _ => None,
        })
        .collect()
}

#[test]
fn each_operations_own_rpm_is_commanded_and_re_commanded_on_change() {
    // Two ops at different speeds -> two M3 S at those speeds.
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 3000.0),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 6000.0),
    ]);
    assert_eq!(spindle_rpms(&d), vec![3000.0, 6000.0]);
}

#[test]
fn an_unchanged_rpm_is_not_re_commanded() {
    // Same speed across two ops -> a single M3 S, not one per op.
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 4000.0),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 4000.0),
    ]);
    assert_eq!(spindle_rpms(&d), vec![4000.0]);
}

#[test]
fn an_unset_rpm_falls_back_to_the_job_default() {
    // rpm 0 (the "unset" sentinel from a tool with no nominal) uses the job default
    // passed to build_job (1000), so existing documents keep their old single speed.
    let d = doc(vec![profile(
        0,
        rect(10.0, 10.0, 70.0, 50.0),
        Side::Outside,
        0.0,
    )]);
    assert_eq!(spindle_rpms(&d), vec![1000.0]);
}
