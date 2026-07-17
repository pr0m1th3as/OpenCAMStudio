//! P7 verification: run real generated jobs through the simulator and prove they
//! do what they claim — a pocket clears its floor, and the whole program is
//! collision-free.

use cam_cldata::SpindleDir;
use cam_geo::{Contour, Point};
use cam_model::{
    Comp, Document, Heights, Lead, Operation, Plunge, PocketOp, ProfileOp, Setup, Side, Stock,
    Tool, ToolKind,
};
use cam_sim::{check_gouge, simulate, Heightfield, SimOptions};
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

fn profile_square(id: u32, chain: Contour) -> ProfileOp {
    ProfileOp {
        clearing: cam_model::Clearing::default(),
        id,
        tool: 1,
        chain,
        side: Side::Outside,
        comp: Comp::Computed,
        offset: 0.0,
        depth: 3.0,
        stepdown: 1.5,
        stepover: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: 0.0,
        plunge: Plunge::Straight,
    }
}

fn setup(operations: Vec<Operation>) -> Document {
    Document::new(Setup {
        name: "verify".into(),
        heights: Heights::new(5.0, 2.0, 0.0),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: STOCK_MAX[2],
            thickness: STOCK_MAX[2] - STOCK_MIN[2],
        },
        tools: vec![Tool {
            number: 1,
            diameter: TOOL_DIA,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
        }],
        operations,
        origin: [0.0, 0.0, 0.0],
        start_offset: None,
    })
}

fn run(doc: &Document) -> cam_sim::SimResult {
    let (program, diags) = build_job(doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
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
        &[],
    )
}

#[test]
fn a_pocket_clears_its_floor_without_collisions() {
    let op = PocketOp {
        clearing: cam_model::Clearing::default(),
        id: 0,
        tool: 1,
        boundary: square(5.0, 5.0, 35.0, 35.0),
        islands: vec![],
        depth: 4.0,
        stepdown: 2.0,
        overlap: 0.5,
        offset: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        plunge: Plunge::Straight,
        start: None,
        lead_overlap: 0.0,
        lead_in: Lead::None,
        lead_out: Lead::None,
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

/// The intended finished part for the 30×30, 4 mm-deep pocket: a flat -4 floor
/// inside the boundary, the original top (0) everywhere else. `intended_depth`
/// lets a test lie about the wanted depth to provoke a gouge.
fn pocket_target(intended_depth: f64) -> Heightfield {
    let mut t = Heightfield::new(
        [STOCK_MIN[0], STOCK_MIN[1]],
        [STOCK_MAX[0], STOCK_MAX[1]],
        0.5,
        STOCK_MAX[2],
    );
    t.lower_rect([5.0, 5.0], [35.0, 35.0], intended_depth);
    t
}

fn pocket_op() -> PocketOp {
    PocketOp {
        clearing: cam_model::Clearing::default(),
        id: 0,
        tool: 1,
        boundary: square(5.0, 5.0, 35.0, 35.0),
        islands: vec![],
        depth: 4.0,
        stepdown: 2.0,
        overlap: 0.5,
        offset: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        plunge: Plunge::Straight,
        start: None,
        lead_overlap: 0.0,
        lead_in: Lead::None,
        lead_out: Lead::None,
    }
}

#[test]
fn a_pocket_cut_to_depth_does_not_gouge_its_target() {
    // The real pocket program cuts to -4; the target's floor is -4. The tool
    // leaves wall material (never crosses the boundary), so nothing goes below
    // target: no gouge.
    let sim = run(&setup(vec![Operation::Pocket(pocket_op())]));
    assert!(
        check_gouge(&sim.field, &pocket_target(-4.0), 0.05).is_none(),
        "an on-depth pocket must not gouge its target"
    );
}

#[test]
fn a_pocket_deeper_than_intended_is_caught_as_a_gouge() {
    // Same real program (cuts to -4), but the part only wanted a 2 mm-deep
    // pocket. The extra 2 mm is a gouge the backplot would never reveal.
    let sim = run(&setup(vec![Operation::Pocket(pocket_op())]));
    let gouge =
        check_gouge(&sim.field, &pocket_target(-2.0), 0.05).expect("the over-cut must be flagged");
    assert_eq!(gouge.kind, cam_sim::CollisionKind::Gouge);
    assert!(
        (gouge.at[2] + 4.0).abs() < 0.3,
        "gouge floor near -4: {:?}",
        gouge.at
    );
}

#[test]
fn a_bad_setup_that_rapids_low_is_caught() {
    // A clearance/retract plane set *below* the stock top is the classic unsafe setup.
    // Two separate profiles force a **lateral** rapid between them at that low plane (from
    // the first op's retract to the second's approach) — the tool plows sideways through
    // the uncut stock between them, which the sim must catch even though a backplot would
    // hide it. (A single-loop op only *lifts* vertically at the low plane, which is safe:
    // the tool retreats up through the column it cut — so the hazard is the lateral move.)
    let doc = Document::new(Setup {
        name: "unsafe".into(),
        heights: Heights::new(-1.0, -1.0, 0.0), // clearance below stock top!
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: STOCK_MAX[2],
            thickness: STOCK_MAX[2] - STOCK_MIN[2],
        },
        tools: vec![Tool {
            number: 1,
            diameter: TOOL_DIA,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
        }],
        // Two separate profiles: linking from the first's retract to the second's
        // approach forces a *lateral* rapid across the (too-low) clearance plane, over
        // the uncut stock between them.
        operations: vec![
            Operation::Profile(profile_square(0, square(5.0, 5.0, 15.0, 15.0))),
            Operation::Profile(profile_square(1, square(25.0, 25.0, 35.0, 35.0))),
        ],
        origin: [0.0, 0.0, 0.0],
        start_offset: None,
    });
    let (program, _) = build_job(&doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    let sim = simulate(
        &program,
        STOCK_MIN,
        STOCK_MAX,
        &SimOptions {
            resolution: 0.5,
            tool_radius: TOOL_DIA / 2.0,
        },
        &[],
    );
    assert!(!sim.is_clean(), "an unsafe retract plane must be flagged");
}
