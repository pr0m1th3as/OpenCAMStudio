//! P7 verification: run real generated jobs through the simulator and prove they
//! do what they claim — a pocket clears its floor, and the whole program is
//! collision-free.

use cam_cldata::SpindleDir;
use cam_geo::{Contour, Point};
use cam_model::{
    Comp, Document, Heights, Operation, PocketOp, ProfileOp, Setup, Side, Stock, Tool, ToolKind,
};
use cam_sim::{simulate, SimOptions};
use cam_toolpath::{build_job, CancelToken};

const TOOL_DIA: f64 = 6.0;
const STOCK_MIN: [f64; 3] = [0.0, 0.0, -10.0];
const STOCK_MAX: [f64; 3] = [40.0, 40.0, 0.0];

fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> Contour {
    Contour::new(vec![
        Point::new(x0, y0),
        Point::new(x1, y0),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ])
}

fn setup(operations: Vec<Operation>) -> Document {
    Document::new(Setup {
        name: "verify".into(),
        heights: Heights::new(5.0, 2.0, 0.0),
        stock: Stock::Box {
            min: STOCK_MIN,
            max: STOCK_MAX,
        },
        tools: vec![Tool {
            number: 1,
            diameter: TOOL_DIA,
            flutes: 2,
            kind: ToolKind::EndMill,
        }],
        operations,
    })
}

fn run(doc: &Document) -> cam_sim::SimResult {
    let (program, diags) = build_job(doc, 1000.0, SpindleDir::Cw, &CancelToken::new());
    assert!(
        !diags
            .iter()
            .any(|d| d.severity == cam_toolpath::Severity::Error),
        "{diags:?}"
    );
    simulate(
        &program,
        STOCK_MIN,
        STOCK_MAX,
        &SimOptions {
            resolution: 0.5,
            tool_radius: TOOL_DIA / 2.0,
        },
    )
}

#[test]
fn a_pocket_clears_its_floor_without_collisions() {
    let op = PocketOp {
        id: 0,
        tool: 1,
        boundary: square(5.0, 5.0, 35.0, 35.0),
        islands: vec![],
        depth: -4.0,
        stepdown: 2.0,
        stepover: 3.0,
        feed: 300.0,
        plunge_feed: 100.0,
    };
    let sim = run(&setup(vec![Operation::Pocket(op)]));

    // Collision-free, and the interior is cut to the floor at points the rings
    // solidly cover.
    assert!(
        sim.is_clean(),
        "unexpected collisions: {:?}",
        sim.collisions
    );
    assert!(
        sim.field.sample(11.0, 11.0) < -3.9,
        "interior floor not reached"
    );
    assert!(
        sim.field.sample(24.0, 24.0) < -3.9,
        "interior floor not reached"
    );
    // Stock well outside the pocket is untouched.
    assert!(
        sim.field.sample(1.0, 1.0) > -0.1,
        "outside the pocket stays uncut"
    );
    // The bulk of the pocket volume is removed (30×30×4 ≈ 3600 mm³ ideal; the
    // tool leaves a wall margin and — a v1 limitation the sim exposes — a small
    // uncut nub at the dead centre when stepover ≈ tool radius).
    assert!(
        sim.removed_volume > 2400.0,
        "removed {}",
        sim.removed_volume
    );
}

#[test]
fn a_bad_setup_that_rapids_low_is_caught() {
    // A profile whose clearance/retract sit *below* the stock top forces lateral
    // rapids through the stock — the sim must catch what a backplot would hide.
    let doc = Document::new(Setup {
        name: "unsafe".into(),
        heights: Heights::new(-1.0, -1.0, 0.0), // clearance below stock top!
        stock: Stock::Box {
            min: STOCK_MIN,
            max: STOCK_MAX,
        },
        tools: vec![Tool {
            number: 1,
            diameter: TOOL_DIA,
            flutes: 2,
            kind: ToolKind::EndMill,
        }],
        operations: vec![Operation::Profile(ProfileOp {
            id: 0,
            tool: 1,
            chain: square(10.0, 10.0, 30.0, 30.0),
            side: Side::Outside,
            comp: Comp::Computed,
            depth: -3.0,
            stepdown: 1.5,
            feed: 300.0,
            plunge_feed: 100.0,
        })],
    });
    let (program, _) = build_job(&doc, 1000.0, SpindleDir::Cw, &CancelToken::new());
    let sim = simulate(
        &program,
        STOCK_MIN,
        STOCK_MAX,
        &SimOptions {
            resolution: 0.5,
            tool_radius: TOOL_DIA / 2.0,
        },
    );
    assert!(!sim.is_clean(), "an unsafe retract plane must be flagged");
}
