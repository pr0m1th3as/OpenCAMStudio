//! Regression cover for the **area-clearing** strategies — pocket, profile
//! outside-roughing, and a carve's clearing pass.
//!
//! These share one engine (`clearing.rs` / `rings.rs`) and, until this file existed, had
//! **no byte-level cover at all**: the two goldens in the tree are a pair of profile ops
//! and the import pipeline. That gap is how a rapid onto the stock surface survived in
//! the shared engine long after the rule forbidding it was applied to every strategy
//! that emits its own approach.
//!
//! So there are two kinds of test here:
//!
//! - **Goldens** — a posted `.nc` per clearing kind, byte-stable, so any change to the
//!   engine has to be looked at and consented to.
//! - **A safety property** — [`assert_no_rapid_onto_uncut_stock`], which is what a
//!   golden cannot give: goldens pin *what the output is*, this pins *what it must never
//!   be*, for any input, including ones nobody wrote a golden for.

use cam_cldata::{MoveKind, Program, SpindleDir, Step};
use cam_geo::{Contour, Point};
use cam_model::{
    Axis, CarveOp, Clearing, Comp, Document, FaceOp, Heights, Lead, Machine, Operation, Plunge,
    PocketOp, ProfileOp, Setup, Side, Stock, Tool, ToolKind,
};
use cam_post::{GrblPost, Post, PostOptions};
use cam_toolpath::{build_job, CancelToken};

const CLEARANCE: f64 = 5.0;
const RETRACT: f64 = 2.0;
const TOP: f64 = 0.0;

fn machine() -> Machine {
    Machine {
        name: "OCS-3018".into(),
        rapid_rate: 2000.0,
        max_spindle_rpm: 10_000.0,
        max_feed: 800.0,
        envelope: cam_model::Envelope::new(
            cam_model::Point3::new(-50.0, -50.0, -50.0),
            cam_model::Point3::new(300.0, 180.0, 50.0),
        ),
        safe_z: 5.0,
        tool_change_pos: None,
    }
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Contour {
    Contour::new(vec![
        Point::new(x0, y0),
        Point::new(x1, y0),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ])
}

fn end_mill(number: u32, diameter: f64) -> Tool {
    Tool {
        number,
        diameter,
        length: 30.0,
        flute_length: 20.0,
        flutes: 2,
        kind: ToolKind::EndMill,
        ..Default::default()
    }
}

fn setup(operations: Vec<Operation>, tools: Vec<Tool>) -> Document {
    Document::new(Setup {
        name: "clearing".into(),
        heights: Heights::new(CLEARANCE, RETRACT, TOP),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: TOP,
            thickness: 10.0,
        },
        tools,
        operations,
        origin: [0.0, 0.0, 0.0],
        start_offset: None,
    })
}

fn post(doc: &Document, name: &str) -> (Program, String) {
    let (program, diags) = build_job(doc, 1000.0, SpindleDir::Cw, None, &CancelToken::new());
    assert!(
        !diags
            .iter()
            .any(|d| d.severity == cam_toolpath::Severity::Error),
        "{diags:?}"
    );
    let opts = PostOptions {
        program_name: Some(name.into()),
        ..Default::default()
    };
    let nc = GrblPost.post(&program, &machine(), &opts).expect("post ok");
    (program, nc)
}

// --- the safety property ---

/// Assert that **no rapid ever descends onto material the tool has not already cut**.
///
/// A rapid may finish below the retract plane only where the tool has already been:
/// dropping to the previous level's Z is a descent through air the same operation cut on
/// its way past. Anywhere else, ending a G0 at or under the stock surface means that a
/// slightly proud blank, or a Z-zero touched off a hair low, drives the cutter into
/// metal at **rapid rate** — no feed control, no chip clearance.
///
/// Levels already cut are tracked per operation, since each carries its own top.
fn assert_no_rapid_onto_uncut_stock(program: &Program, floor: f64) {
    use std::collections::{HashMap, HashSet};
    // Z values (as integer micrometres, so they compare exactly) each op has cut at.
    let key = |z: f64| (z * 1000.0).round() as i64;
    let mut cut: HashMap<u32, HashSet<i64>> = HashMap::new();

    for (i, step) in program.steps().iter().enumerate() {
        match step {
            Step::Linear { to, tag, .. }
                if matches!(tag.kind, MoveKind::Cutting | MoveKind::Plunge) =>
            {
                cut.entry(tag.op_id).or_default().insert(key(to.z));
            }
            Step::Arc { end, tag, .. }
                if matches!(tag.kind, MoveKind::Cutting | MoveKind::Plunge) =>
            {
                cut.entry(tag.op_id).or_default().insert(key(end.z));
            }
            Step::Rapid { to, tag } => {
                if to.z >= floor - 1e-9 {
                    continue; // at or above the retract plane: always safe
                }
                let seen = cut.get(&tag.op_id).is_some_and(|s| s.contains(&key(to.z)));
                assert!(
                    seen,
                    "step {i}: operation {} rapids to Z{:.3}, below the {floor:.3} retract \
                     plane, at a depth it has not yet cut — a G0 into solid stock",
                    tag.op_id, to.z
                );
            }
            _ => {}
        }
    }
}

// --- the documents ---

/// A 60×40 pocket with a 10×10 island, cleared in two levels by a ⌀6 end mill.
fn pocket_doc() -> Document {
    let op = PocketOp {
        clearing: Clearing::default(),
        id: 0,
        tool: 1,
        boundary: rect(0.0, 0.0, 60.0, 40.0),
        islands: vec![rect(25.0, 15.0, 35.0, 25.0)],
        depth: 3.0,
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
    setup(vec![Operation::Pocket(op)], vec![end_mill(1, 6.0)])
}

/// An outside profile with radial roughing — the *other* caller of the clearing engine.
fn roughing_doc() -> Document {
    let op = ProfileOp {
        clearing: Clearing::default(),
        id: 0,
        tool: 1,
        chain: rect(10.0, 10.0, 50.0, 40.0),
        side: Side::Outside,
        comp: Comp::Computed,
        offset: 0.0,
        depth: 3.0,
        stepdown: 2.0,
        stepover: 4.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: 0.0,
        plunge: Plunge::Straight,
    };
    setup(vec![Operation::Profile(op)], vec![end_mill(1, 6.0)])
}

/// A facing pass. Facing has its **own** approach path rather than the clearing
/// engine's, and enters past the stock edge by `overshoot` — so it is covered here
/// separately, and the property below is what says whether that argument holds.
fn face_doc(overshoot: f64) -> Document {
    let op = FaceOp {
        id: 0,
        tool: 1,
        boundary: rect(0.0, 0.0, 60.0, 40.0),
        direction: Axis::X,
        start_offset: 0.0,
        depth: 1.0,
        stepdown: 1.0,
        overlap: 0.5,
        overshoot,
        feed: 300.0,
        plunge_feed: 100.0,
    };
    setup(vec![Operation::Face(op)], vec![end_mill(1, 12.0)])
}

/// A carve whose depth cap leaves a flat land, cleared by a ⌀6 end mill before the
/// V-bit runs — the third caller of the shared clearing engine.
fn carve_doc() -> Document {
    let vbit = Tool {
        number: 1,
        diameter: 6.0,
        length: 30.0,
        flutes: 1,
        kind: ToolKind::VBit {
            included_angle_deg: 90.0,
            tip_radius: 0.1,
        },
        ..Default::default()
    };
    let op = CarveOp {
        id: 0,
        tool: 1,
        clear: Some(cam_model::CarveClearing { tool: 2, params: cam_model::ClearParams::default() }),
        boundary: rect(0.0, 0.0, 40.0, 40.0),
        islands: Vec::new(),
        top: TOP,
        depth: 1.0,
        offset: 0.0,
        ring_step: 0.5,
        feed: 300.0,
        plunge_feed: 100.0,
        stay_down: true,
        start: None,
    };
    setup(vec![Operation::Carve(op)], vec![vbit, end_mill(2, 6.0)])
}

// --- goldens ---

#[test]
fn pocket_nc_golden_is_stable() {
    let (_, nc) = post(&pocket_doc(), "pocket");
    assert_eq!(
        nc,
        include_str!("golden/pocket.nc"),
        "pocket .nc drifted from golden; diff it before regenerating"
    );
}

#[test]
fn roughing_nc_golden_is_stable() {
    let (_, nc) = post(&roughing_doc(), "roughing");
    assert_eq!(
        nc,
        include_str!("golden/roughing.nc"),
        "profile-roughing .nc drifted from golden; diff it before regenerating"
    );
}

#[test]
fn face_nc_golden_is_stable() {
    let (_, nc) = post(&face_doc(2.0), "face");
    assert_eq!(
        nc,
        include_str!("golden/face.nc"),
        "face .nc drifted from golden; diff it before regenerating"
    );
}

#[test]
fn carve_nc_golden_is_stable() {
    let (_, nc) = post(&carve_doc(), "carve");
    assert_eq!(
        nc,
        include_str!("golden/carve.nc"),
        "carve .nc drifted from golden; diff it before regenerating"
    );
}

// --- the property ---

#[test]
fn a_carves_clearing_pass_never_rapids_onto_uncut_stock() {
    // The clearing pass enters solid stock -- the V-bit has not run yet -- so this is
    // the case where a rapid onto the surface has the least margin of all.
    let (program, _) = post(&carve_doc(), "carve");
    assert_no_rapid_onto_uncut_stock(&program, RETRACT.max(TOP));
}

#[test]
fn a_pocket_never_rapids_onto_uncut_stock() {
    let (program, _) = post(&pocket_doc(), "pocket");
    assert_no_rapid_onto_uncut_stock(&program, RETRACT.max(TOP));
}

#[test]
fn profile_roughing_never_rapids_onto_uncut_stock() {
    let (program, _) = post(&roughing_doc(), "roughing");
    assert_no_rapid_onto_uncut_stock(&program, RETRACT.max(TOP));
}

#[test]
fn a_deep_pocket_stays_safe_at_every_level() {
    // Many levels, so the property is exercised against the *re-entries* too, not just
    // the single first approach.
    let mut doc = pocket_doc();
    if let Operation::Pocket(p) = &mut doc.setup.operations[0] {
        p.depth = 6.0;
        p.stepdown = 0.5;
    }
    let (program, _) = post(&doc, "deep");
    assert_no_rapid_onto_uncut_stock(&program, RETRACT.max(TOP));
}

#[test]
fn facing_enters_beyond_the_stock_edge() {
    // Facing descends to its top cutting plane, which at `start_offset = 0` is the stock
    // surface — so the thing that keeps it safe is *where* it descends, not how high it
    // stops. Pin that: the entry must lie outside the stock in the travel axis.
    let (program, _) = post(&face_doc(2.0), "face");
    let first_descent = program
        .steps()
        .iter()
        .find_map(|s| match s {
            Step::Rapid { to, .. } if to.z < CLEARANCE - 1e-9 => Some(*to),
            _ => None,
        })
        .expect("a descent");
    assert!(
        first_descent.x < 0.0 || first_descent.x > 60.0,
        "facing descended to Z{:.3} at X{:.3}, which is over the stock, not past its edge",
        first_descent.z,
        first_descent.x
    );
}
