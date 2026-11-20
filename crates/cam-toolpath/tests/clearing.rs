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
    Axis, CarveOp, ClearParams, Clearing, Comp, Document, FaceOp, Heights, Lead, Machine,
    Operation, Plunge, PocketOp, ProfileOp, Setup, Side, Stock, Tool, ToolKind,
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
        tool_change_seconds: 0.0,
        min_spindle_rpm: 0.0,
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
        id: 0,
        frame: Default::default(),
        material: String::new(),
        material_id: None,
        model_instances: Vec::new(),
        fixture_instances: Vec::new(),
        fixture_clearance: Default::default(),
        program_name: String::new(),
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
        patterns: Vec::new(),
        origin: [0.0, 0.0, 0.0],
        extra_origins: vec![],
        origin_index: 1,
        operation_heights: Default::default(),
        height_settings: Default::default(),
        stock_to_leave: Default::default(),
        tool_change_height: None,
    })
}

fn post(doc: &Document, name: &str) -> (Program, String) {
    // A roughing fixture must provide real stock rather than exercising the old
    // silent single-pass fallback. Other strategies do not consume these bounds.
    let stock = doc
        .setup()
        .operations
        .iter()
        .any(|op| matches!(op,Operation::Profile(p) if p.stepover>0.))
        .then_some(([0., 0.], [60., 50.]));
    let cam_toolpath::JobResult {
        program,
        diagnostics: diags,
        ..
    } = build_job(
        doc,
        1000.0,
        SpindleDir::Cw,
        stock,
        machine().envelope.max.z,
        &CancelToken::new(),
    );
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
                // The op must already have cut *at least* this deep: a rapid ending
                // anywhere above its own deepest cut is in the column it has opened.
                //
                // This used to demand the rapid land on a depth exactly cut, which read
                // as "in cleared air" but actually required the tool to come to rest on
                // a cut floor with no margin — the very thing `emit::rapid_floor` now
                // stops it doing, so the proxy had to go. What it still catches is the
                // real fault: a `G0` to a depth deeper than anything cut, into solid.
                let deepest = cut
                    .get(&tag.op_id)
                    .and_then(|s| s.iter().min().copied())
                    .map(|z| z as f64 / 1000.0);
                assert!(
                    deepest.is_some_and(|d| to.z >= d - 1e-9),
                    "step {i}: operation {} rapids to Z{:.3}, below the {floor:.3} retract \
                     plane and deeper than anything it has cut ({:?}) — a G0 into solid stock",
                    tag.op_id,
                    to.z,
                    deepest
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
        maximum_stepover: None,
        parameter_expressions: Default::default(),
        wrap: None,
        additional_regions: Vec::new(),
        stock_contours: Default::default(),
        finishing: Default::default(),
        spindle_rpm: 0.0,
        tool_preset: None,
        cooling: Default::default(),
        entry_feeds: Default::default(),
        work_offset: 1,
        id: 0,
        tool: 1,
        boundary: rect(0.0, 0.0, 60.0, 40.0),
        islands: vec![rect(25.0, 15.0, 35.0, 25.0)],
        depth: 3.0,
        end: None,
        start: None,
        clear: ClearParams {
            multiple_depths: Default::default(),
            adaptive_enabled: None,
            pocket_overlap: None,
            keep_tool_down: false,
            lift_height: 0.0,
            clearing: Clearing::default(),
            stepdown: 2.0,
            overlap: 0.5,
            offset: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            plunge: Plunge::Straight,
            lead_overlap: 0.0,
            lead_in: Lead::None,
            lead_out: Lead::None,
        },
    };
    setup(vec![Operation::Pocket(op)], vec![end_mill(1, 6.0)])
}

/// An outside profile with radial roughing — the *other* caller of the clearing engine.
fn roughing_doc() -> Document {
    let op = ProfileOp {
        parameter_expressions: Default::default(),
        wrap: None,
        passes: Default::default(),
        selections: None,
        closed: true,
        additional_chains: Vec::new(),
        tabs: Default::default(),
        spindle_rpm: 0.0,
        tool_preset: None,
        cooling: Default::default(),
        entry_feeds: Default::default(),
        work_offset: 1,
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
        end: None,
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
        absolute_overlap: None,
        multiple_depths: Default::default(),
        parameter_expressions: Default::default(),
        spindle_rpm: 0.0,
        tool_preset: None,
        cooling: Default::default(),
        entry_feeds: Default::default(),
        work_offset: 1,
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
        parameter_expressions: Default::default(),
        spindle_rpm: 0.0,
        tool_preset: None,
        cooling: Default::default(),
        entry_feeds: Default::default(),
        work_offset: 1,
        id: 0,
        tool: 1,
        clear: Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams {
                lead_in: cam_model::Lead::None,
                lead_out: cam_model::Lead::None,
                ..Default::default()
            },
        }),
        boundary: rect(0.0, 0.0, 40.0, 40.0),
        islands: Vec::new(),
        top: TOP,
        depth: 1.0,
        offset: 0.0,
        ring_step: 0.5,
        scallop: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        plunge: cam_model::Plunge::Straight,
        stay_down: true,
        end: None,
        start: None,
    };
    setup(vec![Operation::Carve(op)], vec![vbit, end_mill(2, 6.0)])
}

/// A carve/pocket wall ring ramps the same way a profile pass does — via the shared
/// `profile::contour_ramp` — so the two cannot drift apart. What is ring-specific is the
/// *wiring*: the cut has to begin where the ramp ended and go a full perimeter from
/// there, or the stretch the ramp sloped is left standing.
#[test]
fn a_ramped_clearing_entry_never_drops_vertically_and_re_machines_its_ramp() {
    use cam_cldata::{MoveKind, Point3, Step};
    use cam_model::Plunge;

    let mut d = pocket_doc();
    if let cam_model::Operation::Pocket(p) = &mut d.setups[0].operations[0] {
        p.clear.plunge = Plunge::Ramp { angle_deg: 5.0 };
    }
    let cam_toolpath::JobResult {
        program,
        diagnostics: diags,
        ..
    } = build_job(&d, 1000.0, SpindleDir::Cw, None, 50.0, &CancelToken::new());
    assert!(
        diags
            .iter()
            .all(|x| x.severity != cam_toolpath::Severity::Error),
        "{diags:?}"
    );

    // No `Plunge` move may drop straight down while it is descending: that is the entry
    // a ramp exists to avoid, and it is what a silent fallback would produce.
    let mut pos = Point3::new(0.0, 0.0, 0.0);
    let (mut vertical, mut ramped) = (0, 0);
    for st in program.steps() {
        let (to, kind) = match st {
            Step::Linear { to, tag, .. } => (*to, Some(tag.kind)),
            Step::Rapid { to, .. } => (*to, None),
            Step::Arc { end, .. } => (*end, None),
            _ => continue,
        };
        if kind == Some(MoveKind::Plunge) && to.z < pos.z - 1e-9 {
            if (to.x - pos.x).hypot(to.y - pos.y) <= 1e-9 {
                vertical += 1;
            } else {
                ramped += 1;
            }
        }
        pos = to;
    }
    assert_eq!(
        vertical, 0,
        "a ramped clearing entry dropped vertically {vertical} times"
    );
    assert!(ramped > 0, "no ramp was emitted at all");

    // And the cutting that follows each ramp must be at least a full ring — the ramp
    // consumed part of one, and only a full perimeter from there re-machines it.
    let cutting: f64 = {
        let mut pos = Point3::new(0.0, 0.0, 0.0);
        let mut total = 0.0;
        for st in program.steps() {
            let to = match st {
                Step::Linear { to, tag, .. } => {
                    if tag.kind == MoveKind::Cutting {
                        total += (to.x - pos.x).hypot(to.y - pos.y);
                    }
                    *to
                }
                Step::Rapid { to, .. } => *to,
                Step::Arc { end, .. } => *end,
                _ => continue,
            };
            pos = to;
        }
        total
    };
    // The boundary ring alone is ~200 mm per level, three levels, plus island rings.
    assert!(
        cutting > 400.0,
        "only {cutting:.0} mm of cutting — the rings are not being closed"
    );
}

// --- goldens ---

#[test]
fn roughing_end_mill_generates_pocket_using_nominal_envelope() {
    let finishing = pocket_doc();
    let mut roughing = pocket_doc();
    roughing.setup_mut().tools[0].kind = ToolKind::RoughingMill;
    let (program, nc) = post(&roughing, "roughing-cutter");
    let (_, expected) = post(&finishing, "roughing-cutter");
    assert_eq!(nc, expected);
    assert_no_rapid_onto_uncut_stock(&program, RETRACT);
}

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
    if let Operation::Pocket(p) = &mut doc.setups[0].operations[0] {
        p.depth = 6.0;
        p.clear.stepdown = 0.5;
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

/// **The guard that should have caught the collision Andreas hit.** Run the same
/// simulator the GUI runs over an island pocket cleared adaptively, and require it clean.
///
/// This exists because a real defect reached a real machine file with the whole headless
/// suite green: an optimisation had the steered clearer *rapid* down to just above the floor
/// on re-entry instead of feeding, which `cam-sim` rightly flags — it checks descending
/// rapids, and a seed stands by construction where uncut material sits just outside the
/// tool's disc. Every island pocket reported
/// `RapidThroughStock: rapid at Z -1.500 passes through stock standing at Z 0.000`
/// and would not export. Nothing in this crate's tests simulated anything, so nothing knew.
#[test]
fn lifted_pocket_links_preserve_floor_coverage_and_simulate_clean() {
    use cam_sim::{simulate, SimOptions, SimTool, ToolProfile};

    for (engagement, plunge, leads) in [
        (0., Plunge::Straight, false),
        (0., Plunge::Ramp { angle_deg: 5. }, true),
        (
            0.,
            Plunge::Helix {
                radius: 1.,
                pitch: 0.5,
            },
            true,
        ),
        (2., Plunge::Straight, false),
    ] {
        let mut baseline = None;
        for lift in [0., 0.2, 2., 100.] {
            let mut doc = pocket_doc();
            let Operation::Pocket(op) = &mut doc.setups[0].operations[0] else {
                unreachable!()
            };
            op.clear.lift_height = lift;
            op.clear.clearing.engagement = engagement;
            op.clear.plunge = plunge;
            if leads {
                op.clear.lead_in = Lead::Arc { radius: 1. };
                op.clear.lead_out = Lead::Arc { radius: 1. };
            }
            let (program, nc) = post(&doc, "lifted-pocket");
            assert!(!nc.is_empty());
            assert_no_rapid_onto_uncut_stock(&program, TOP);
            if lift > 0. && engagement == 0. {
                // Both depth levels have local horizontal links at their own
                // Z + lift. Even an excessive lift stops at clearance.
                for z in [-2., -3.] {
                    let raised = (z + lift).min(CLEARANCE);
                    assert!(program.steps().windows(2).any(|pair| matches!(pair,
                        [Step::Linear { to: a, tag, .. }, Step::Linear { to: b, .. }]
                            if tag.kind == MoveKind::Link && a.z == raised && b.z == raised
                                && (a.x != b.x || a.y != b.y))));
                }
            }
            let sim = simulate(
                &program,
                [-5., -5., -10.],
                [65., 45., TOP],
                &SimOptions {
                    resolution: 0.5,
                    tool_radius: 3.,
                },
                &[SimTool {
                    number: 1,
                    profile: ToolProfile::flat(3.),
                    noncutting_tip_radius: None,
                }],
            );
            assert!(
                sim.is_clean(),
                "lift {lift}, engagement {engagement}, {plunge:?}: {:?}",
                sim.collisions
            );
            if lift == 0. {
                baseline = Some(sim.field);
            } else {
                let difference = sim.field.compare(baseline.as_ref().unwrap(), 0.01);
                assert!(
                    difference.residual_volume <= 1.,
                    "lift left extra stock: {difference:?}, {plunge:?}"
                );
            }
        }
    }
}

#[test]
fn concentric_pocket_clears_lobes_that_disappear_before_the_next_ring() {
    let mut doc = pocket_doc();
    let setup = &mut doc.setups[0];
    setup.tools = vec![end_mill(1, 3.)];
    let Operation::Pocket(op) = &mut setup.operations[0] else {
        unreachable!()
    };
    let center = Point::new(11., 10.5);
    let island = Contour::new(
        (0..96)
            .map(|i| {
                let angle = f64::from(i) * std::f64::consts::TAU / 96.;
                Point::new(center.x + 2.5 * angle.cos(), center.y + 2.5 * angle.sin())
            })
            .collect(),
    );
    op.boundary = rect(0., 0., 29., 19.5);
    op.islands = vec![island];
    op.maximum_stepover = Some(2.);
    op.clear.lift_height = 0.2;
    let (program, _) = post(&doc, "concentric-collapse");
    let sim = cam_sim::simulate(
        &program,
        [-2., -2., -5.],
        [31., 21.5, TOP],
        &cam_sim::SimOptions {
            resolution: 0.1,
            tool_radius: 1.5,
        },
        &[],
    );
    assert!(sim.is_clean(), "{:?}", sim.collisions);
    for i in 8..138 {
        for j in 8..90 {
            let point = Point::new(f64::from(i) * 0.2, f64::from(j) * 0.2);
            let z = sim.field.sample(point.x, point.y);
            if point.distance(center) > 2.7 {
                assert!(z <= -2.99, "stock left at {point:?}: Z{z}");
            } else if point.distance(center) < 2.3 {
                assert_eq!(z, TOP as f32, "island cut at {point:?}");
            }
        }
    }
}

#[test]
fn an_adaptively_cleared_island_pocket_simulates_clean() {
    use cam_sim::{simulate, SimOptions, SimTool, ToolProfile};

    let mut op = match pocket_doc().setups[0].operations[0].clone() {
        Operation::Pocket(p) => p,
        other => panic!("expected a pocket, got {other:?}"),
    };
    // An engagement cap + climb is what routes a holed region to the steered clearer.
    op.clear.clearing = Clearing {
        engagement: 2.0,
        climb: true,
    };
    let doc = setup(vec![Operation::Pocket(op)], vec![end_mill(1, 6.0)]);

    let cam_toolpath::JobResult {
        program,
        diagnostics: diags,
        ..
    } = build_job(
        &doc,
        1000.0,
        SpindleDir::Cw,
        None,
        machine().envelope.max.z,
        &CancelToken::new(),
    );
    assert!(
        !diags
            .iter()
            .any(|d| d.severity == cam_toolpath::Severity::Error),
        "{diags:?}"
    );

    let sim = simulate(
        &program,
        [-5.0, -5.0, TOP - 10.0],
        [65.0, 45.0, TOP],
        &SimOptions {
            resolution: 0.5,
            tool_radius: 3.0,
        },
        &[SimTool {
            number: 1,
            profile: ToolProfile::flat(3.0),
            noncutting_tip_radius: None,
        }],
    );
    assert!(
        sim.is_clean(),
        "the adaptive island clear must simulate without collisions, got: {:?}",
        sim.collisions
    );
}

#[test]
fn adaptive_reference_pocket_uses_lifted_rapid_links_and_clears_each_depth() {
    use cam_sim::{simulate, SimOptions, SimTool, ToolProfile};
    // Recreate cnc_test.ocam and the NC reference's pocket/cutter dimensions.
    // No external projects, CAD kernel, GUI, or skipped fixture checks.
    for (engagement, stepdown, plunge, allowance) in [
        (1.5, 2., Plunge::Straight, 0.),
        (
            2.4,
            6.,
            Plunge::Helix {
                radius: 2.,
                pitch: 1.,
            },
            0.,
        ),
        (2.4, 6., Plunge::Straight, 0.4),
    ] {
        let mut doc = pocket_doc();
        let Operation::Pocket(op) = &mut doc.setups[0].operations[0] else {
            unreachable!()
        };
        op.boundary = rect(10., -10., 40., 10.);
        op.islands.clear();
        op.depth = 6.;
        op.clear.stepdown = stepdown;
        op.clear.plunge = plunge;
        op.clear.offset = allowance;
        op.clear.clearing.engagement = engagement;
        let (program, nc) = post(&doc, "adaptive-reference-pocket");
        program.validate().unwrap();
        assert!(nc.contains("Z-6.000"));
        let mut previous = None;
        let mut cutting_travel = 0.;
        let mut lifts = 0;
        let mut short_lifts = 0;
        let mut rapid_travel = 0.;
        let mut middle_wall_cut = 0.;
        let mut depths = std::collections::BTreeSet::new();
        for step in program.steps() {
            let end = match step {
                Step::Rapid { to, .. } => {
                    if let Some(p) = previous {
                        let p: cam_cldata::Point3 = p;
                        if p.z < 0. && to.z > p.z + 1e-6 {
                            if to.z >= CLEARANCE {
                                lifts += 1;
                            } else {
                                short_lifts += 1;
                                assert!((to.z - p.z - 0.2).abs() < 1e-6);
                            }
                        }
                        if p.z < 0. && (p.z - to.z).abs() < 1e-6 {
                            rapid_travel += (p.x - to.x).hypot(p.y - to.y);
                            assert!((13.6 + allowance - 0.001..=36.4 - allowance + 0.001)
                                .contains(&to.x));
                            assert!((-6.4 + allowance - 0.001..=6.4 - allowance + 0.001)
                                .contains(&to.y));
                        }
                    }
                    *to
                }
                Step::Linear { to, tag, .. } => {
                    if tag.kind == MoveKind::Cutting {
                        let p = previous.unwrap();
                        cutting_travel += (p.x - to.x).hypot(p.y - to.y);
                        depths.insert((-to.z * 1000.).round() as i32);
                        if (p.y - to.y).abs() < 0.02 && (to.y.abs() - (7. - allowance)).abs() < 0.02
                        {
                            middle_wall_cut +=
                                (p.x.max(to.x).min(25.5) - p.x.min(to.x).max(24.5)).max(0.);
                        }
                        assert!((13. + allowance - 0.02..=37. - allowance + 0.02).contains(&to.x));
                        assert!((-7. + allowance - 0.02..=7. - allowance + 0.02).contains(&to.y));
                    }
                    *to
                }
                Step::Arc { end, .. } => *end,
                _ => continue,
            };
            previous = Some(end);
        }
        let levels = (6. / stepdown) as usize;
        assert_eq!(lifts, levels, "only one retract per cutting depth");
        eprintln!("stepover {engagement}: short lifts {short_lifts}, rapid {rapid_travel}, middle wall {middle_wall_cut}");
        assert!(short_lifts >= 4 * levels);
        assert!(rapid_travel > 30. * levels as f64);
        assert!(
            middle_wall_cut <= 2.01 * levels as f64,
            "middle of each long wall should cut once: {middle_wall_cut}"
        );
        assert_eq!(depths.len(), levels);
        // Old frontier loops travelled 782 / 555 mm per level respectively.
        let limit = if engagement == 1.5 { 680. } else { 500. };
        assert!(cutting_travel < limit * levels as f64, "{cutting_travel}");
        for resolution in [0.25, 0.5] {
            let sim = simulate(
                &program,
                [9., -11., -8.],
                [41., 11., 0.],
                &SimOptions {
                    resolution,
                    tool_radius: 3.,
                },
                &[SimTool {
                    number: 1,
                    profile: ToolProfile::flat(3.),
                    noncutting_tip_radius: None,
                }],
            );
            assert!(sim.is_clean(), "{:?}", sim.collisions);
            for x in 14..=36 {
                for y in -6..=6 {
                    assert!(
                        sim.field.sample(x as f64, y as f64) <= -5.99,
                        "uncut floor at {x},{y}"
                    );
                }
            }
            assert_eq!(
                sim.field.sample(9.5, 0.),
                0.,
                "outside wall must remain intact"
            );
            assert_eq!(sim.field.sample(40.5, 0.), 0.);
            if allowance > 0. {
                assert_eq!(sim.field.sample(10.125, 0.), 0., "reserved wall stock");
            }
        }
    }
}

#[test]
fn configured_adaptive_leads_are_posted_as_arcs_and_simulate_cleanly() {
    use cam_sim::{simulate, SimOptions, SimTool, ToolProfile};
    for (overlap, finishing) in [(0., false), (0.75, false), (0., true), (0.75, true)] {
        let mut doc = pocket_doc();
        let Operation::Pocket(op) = &mut doc.setups[0].operations[0] else {
            unreachable!()
        };
        op.boundary = rect(10., -10., 40., 10.);
        op.islands.clear();
        op.depth = 6.;
        op.clear.stepdown = 6.;
        op.clear.clearing.engagement = 2.4;
        op.clear.lead_in = Lead::Arc { radius: 0.6 };
        op.clear.lead_out = Lead::Arc { radius: 0.6 };
        op.clear.lead_overlap = overlap;
        op.finishing.enabled = finishing;
        op.finishing.count = 1;
        op.finishing.stepover = 0.4;
        op.entry_feeds.lead_in = Some(123.);
        op.entry_feeds.lead_out = Some(234.);
        let (program, nc) = post(&doc, "adaptive-configured-leads");
        program.validate().unwrap();
        let mut incoming = 0;
        let mut outgoing = 0;
        for step in program.steps() {
            if let Step::Arc {
                end,
                center,
                feed,
                tag,
                ..
            } = step
            {
                if matches!(tag.kind, MoveKind::LeadIn | MoveKind::LeadOut) {
                    assert!(((end.x - center.x).hypot(end.y - center.y) - 0.6).abs() < 1e-10);
                    if tag.kind == MoveKind::LeadIn {
                        incoming += 1;
                        assert_eq!(*feed, 123.);
                        assert_eq!(end.z, -6.);
                    } else {
                        outgoing += 1;
                        assert_eq!(*feed, 234.);
                    }
                }
            }
        }
        assert!(
            incoming >= 5 && outgoing >= 5,
            "configured leads must not force concentric clearing: {incoming}/{outgoing}"
        );
        assert!(nc.contains("G2 ") || nc.contains("G3 "));
        for resolution in [0.25, 0.5] {
            let sim = simulate(
                &program,
                [9., -11., -8.],
                [41., 11., 0.],
                &SimOptions {
                    resolution,
                    tool_radius: 3.,
                },
                &[SimTool {
                    number: 1,
                    profile: ToolProfile::flat(3.),
                    noncutting_tip_radius: None,
                }],
            );
            assert!(sim.is_clean(), "{:?}", sim.collisions);
            for x in 14..=36 {
                for y in -6..=6 {
                    assert!(
                        sim.field.sample(x as f64, y as f64) <= -5.99,
                        "uncut floor at {x},{y}"
                    );
                }
            }
            assert_eq!(sim.field.sample(9.5, 0.), 0.);
            assert_eq!(sim.field.sample(40.5, 0.), 0.);
        }
    }
}

#[test]
fn saved_adaptive_stepover_paths_are_recomputed_after_travel_trimming() {
    use cam_toolpath::{JobEnv, OperationCache, StrategyResult};
    for mut doc in [pocket_doc(), roughing_doc(), carve_doc()] {
        for enabled in [false, true] {
            let op = &mut doc.setups[0].operations[0];
            let engagement = if enabled { 2.4 } else { 0. };
            match op {
                Operation::Pocket(p) => p.clear.clearing.engagement = engagement,
                Operation::Profile(p) => p.clearing.engagement = engagement,
                Operation::Carve(p) => {
                    p.clear.as_mut().unwrap().params.clearing.engagement = engagement
                }
                _ => unreachable!(),
            }
            let env = JobEnv {
                heights: Heights::new(5., 2., 0.),
                tools: &[],
                stock: None,
                spindle: SpindleDir::Cw,
            };
            let token = CancelToken::new();
            let mut cache = OperationCache::default();
            cache.compute(op, &env, &token, StrategyResult::default);
            let revisions = if enabled {
                vec![None, Some(0), Some(1), Some(2)]
            } else {
                vec![None, Some(0)]
            };
            for revision in revisions {
                let mut saved = serde_json::to_value(&cache).unwrap();
                let key = saved["entries"][0]["key"].as_object_mut().unwrap();
                if let Some(revision) = revision {
                    key.insert(
                        "adaptive_clearing_revision".into(),
                        serde_json::json!(revision),
                    );
                } else {
                    key.remove("adaptive_clearing_revision");
                }
                let mut reopened: OperationCache = serde_json::from_value(saved).unwrap();
                reopened.compute(op, &env, &token, StrategyResult::default);
                assert_eq!(reopened.stats.computed, usize::from(enabled));
                assert_eq!(reopened.stats.reused, usize::from(!enabled));
                reopened.compute(op, &env, &token, || panic!("current path must be reused"));
                assert_eq!(reopened.stats.reused, 1 + usize::from(!enabled));
            }
        }
    }
}

/// A steered clear must actually **traverse** the stock it has already removed.
///
/// A third of a steered path is the tool crossing cleared ground to reach the next front, and
/// feeding it at cutting rate cost 0.76 min of a 10.42 min pocket here. The saving is only
/// real if the flags survive: they are read by index against the path, and a single
/// `path.push` without its matching flag once misattributed every later move — silently, and
/// straight past the simulator on the shapes that happened not to collide. So assert both
/// halves: that traverses are emitted at all, and (in the simulation test above) that they are
/// safe.
#[test]
fn a_steered_clear_traverses_the_stock_it_has_already_removed() {
    let mut op = match pocket_doc().setups[0].operations[0].clone() {
        Operation::Pocket(p) => p,
        other => panic!("expected a pocket, got {other:?}"),
    };
    op.clear.clearing = Clearing {
        engagement: 2.0,
        climb: true,
    };
    let doc = setup(vec![Operation::Pocket(op)], vec![end_mill(1, 6.0)]);
    let cam_toolpath::JobResult { program, .. } = build_job(
        &doc,
        1000.0,
        SpindleDir::Cw,
        None,
        machine().envelope.max.z,
        &CancelToken::new(),
    );

    let mut prev: Option<[f64; 3]> = None;
    let mut traversed = 0.0_f64;
    for step in &program.steps {
        let to = match step {
            Step::Linear { to, .. } => [to.x, to.y, to.z],
            Step::Rapid { to, .. } => {
                if let Some(a) = prev {
                    // Lateral, at cutting depth: a link across ground already cleared.
                    if a[2] < -0.5 && (a[2] - to.z).abs() < 1e-9 {
                        traversed += (a[0] - to.x).hypot(a[1] - to.y);
                    }
                }
                [to.x, to.y, to.z]
            }
            _ => continue,
        };
        prev = Some(to);
    }
    assert!(
        traversed > 100.0,
        "a steered island clear should traverse its cleared ground, got {traversed:.0} mm"
    );
}

/// Rewrite this file's goldens after an intentional change.
///
/// `cargo test -p cam-toolpath --test clearing -- --ignored regen_goldens`, then **read the
/// diff** before committing — that is the whole value of a golden. Ignored by default
/// so a normal run never rewrites the thing it is meant to be checking against.
#[test]
#[ignore = "regeneration helper"]
fn regen_goldens() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/golden/");
    for (name, doc) in [
        ("pocket", pocket_doc()),
        ("roughing", roughing_doc()),
        ("face", face_doc(2.0)),
        ("carve", carve_doc()),
    ] {
        let (_, nc) = post(&doc, name);
        std::fs::write(format!("{dir}{name}.nc"), nc).unwrap();
    }
}

/// **An arc lead-in stays an arc when the ramp descends down it.**
///
/// Andreas, on a pocket export: *"I selected arc lead-in together with ramp-along-the-
/// path … it's doing ramp-along with no arcs."* Two faults met here. The clearing level
/// entry ignored the lead entirely — it was passed an empty one, on the reasoning that a
/// clearing lead is the wall-finish lead. And the ramp emitted raw chords, because it
/// skipped the `fit_arcs` pass the cutting emission has always run; even once the lead
/// was wired in it would have come out as a fan of straight moves.
#[test]
fn a_ramped_entry_flies_its_arc_lead_in_as_one_arc() {
    use cam_cldata::{MoveKind, Step};
    use cam_model::{Lead, Plunge};

    let mut d = pocket_doc();
    if let cam_model::Operation::Pocket(p) = &mut d.setups[0].operations[0] {
        p.clear.plunge = Plunge::Ramp { angle_deg: 5.0 };
        p.clear.lead_in = Lead::Arc { radius: 3.0 };
        p.clear.lead_out = Lead::Arc { radius: 3.0 };
    }
    let cam_toolpath::JobResult { program, .. } =
        build_job(&d, 1000.0, SpindleDir::Cw, None, 50.0, &CancelToken::new());

    // The first thing the entry does is a *descending arc* — the lead, flown in air.
    let first_plunge = program
        .steps()
        .iter()
        .find(|s| {
            matches!(s, Step::Arc { tag, .. } | Step::Linear { tag, .. }
                           if tag.kind == MoveKind::Plunge)
        })
        .expect("a ramped entry");
    match first_plunge {
        Step::Arc { end, center, .. } => {
            // Descends to the material surface, not below it: the lead cuts nothing.
            // Within a micron of the stock top — it prints as `Z0.000`.
            //
            // Not exactly zero, and the reason is worth knowing: the lead's *rise* is
            // computed from the sampled polyline's chord length, while the emission now
            // spreads the descent over true **arc** length, which is fractionally
            // longer. So the lead ends a hair below the surface. Arc length is the right
            // measure for the emission — it is what keeps the ramp at the requested
            // angle *through* a curve rather than steepening — and 0.2 µm of dip is far
            // below anything a machine or a 3-decimal G-code word can express. Trading a
            // correct angle for an exact touch would be the wrong way round.
            assert!(
                end.z.abs() < 1e-3,
                "the lead must land on the stock top, got {}",
                end.z
            );
            assert!(
                (center.z - end.z).abs() < 1e-9,
                "a helical arc carries the arriving Z on its centre"
            );
        }
        other => panic!("the ramp should open with the lead *arc*, got {other:?}"),
    }

    // And no run of chords where an arc belongs: with a 3 mm lead sampled for re-fitting,
    // the whole lead is one move.
    let plunge_arcs = program
        .steps()
        .iter()
        .filter(|s| matches!(s, Step::Arc { tag, .. } if tag.kind == MoveKind::Plunge))
        .count();
    assert!(plunge_arcs >= 1, "the ramp emitted no arcs at all");
}

#[test]
fn keep_tool_down_with_finishing_preserves_islands_and_simulates_clean() {
    use cam_sim::{simulate, SimOptions, SimTool, ToolProfile};
    let mut baseline = None;
    for down in [false, true] {
        let mut doc = pocket_doc();
        let Operation::Pocket(op) = &mut doc.setups[0].operations[0] else {
            unreachable!()
        };
        op.clear.keep_tool_down = down;
        op.clear.lead_in = Lead::Arc { radius: 1. };
        op.clear.lead_out = Lead::Arc { radius: 1. };
        op.finishing.enabled = true;
        op.finishing.count = 2;
        op.finishing.repeat_pass = true;
        let (program, _) = post(&doc, "keep-tool-down");
        assert_no_rapid_onto_uncut_stock(&program, TOP);
        let sim = simulate(
            &program,
            [-5., -5., -10.],
            [65., 45., TOP],
            &SimOptions {
                resolution: 0.5,
                tool_radius: 3.,
            },
            &[SimTool {
                number: 1,
                profile: ToolProfile::flat(3.),
                noncutting_tip_radius: None,
            }],
        );
        assert!(sim.is_clean(), "{down}: {:?}", sim.collisions);
        if let Some(expected) = baseline.as_ref() {
            let difference = sim.field.compare(expected, 0.01);
            assert!(difference.residual_volume <= 1., "{difference:?}");
            assert!(difference.gouge_volume <= 1., "{difference:?}");
        } else {
            baseline = Some(sim.field);
        }
    }
}

#[test]
fn saved_keep_down_paths_with_noncutting_bridges_are_recomputed() {
    use cam_toolpath::{JobEnv, OperationCache, StrategyResult};
    for mut doc in [pocket_doc(), roughing_doc()] {
        for enabled in [false, true] {
            let op = &mut doc.setups[0].operations[0];
            match op {
                Operation::Pocket(p) => p.clear.keep_tool_down = enabled,
                Operation::Profile(p) => p.passes.keep_tool_down = enabled,
                _ => unreachable!(),
            }
            let env = JobEnv {
                heights: Heights::new(30., 25., 21.),
                tools: &[],
                stock: None,
                spindle: SpindleDir::Cw,
            };
            let token = CancelToken::new();
            let mut cache = OperationCache::default();
            cache.compute(op, &env, &token, StrategyResult::default);
            for revision in [None, Some(0)] {
                let mut saved = serde_json::to_value(&cache).unwrap();
                let key = saved["entries"][0]["key"].as_object_mut().unwrap();
                if let Some(revision) = revision {
                    key.insert("keep_down_revision".into(), serde_json::json!(revision));
                } else {
                    key.remove("keep_down_revision");
                }
                let mut reopened: OperationCache = serde_json::from_value(saved).unwrap();
                reopened.compute(op, &env, &token, StrategyResult::default);
                assert_eq!(reopened.stats.computed, usize::from(enabled));
                assert_eq!(reopened.stats.reused, usize::from(!enabled));
                reopened.compute(op, &env, &token, || panic!("Current path should be reused"));
                assert_eq!(reopened.stats.reused, 1 + usize::from(!enabled));
            }
        }
    }
}

#[test]
fn keep_down_depth_and_finishing_passes_verify_on_raised_synthetic_stock() {
    use cam_sim::{simulate, SimOptions, SimTool, ToolProfile};
    for finishing in [false, true] {
        for mut doc in [pocket_doc(), roughing_doc()] {
            doc.setups[0].heights = Heights::new(30., 25., 21.);
            let mut lifted_retracts = 0;
            for down in [false, true] {
                let op = &mut doc.setups[0].operations[0];
                match op {
                    Operation::Pocket(p) => {
                        p.boundary = rect(0., 0., 24., 24.);
                        p.islands.clear();
                        p.depth = 0.;
                        p.clear.stepdown = 6.;
                        p.clear.keep_tool_down = down;
                        p.clear.lead_in = Lead::Arc { radius: 2. };
                        p.clear.lead_out = Lead::Arc { radius: 2. };
                        p.finishing.enabled = finishing;
                        p.finishing.count = 2;
                        p.finishing.repeat_pass = true;
                    }
                    Operation::Profile(p) => {
                        p.depth = 1.;
                        p.passes.floor_allowance = 1.;
                        p.stepdown = 6.;
                        p.stepover = 0.;
                        p.start = Some([30., 10.]);
                        p.lead_in = Lead::Arc { radius: 2. };
                        p.lead_out = Lead::Arc { radius: 2. };
                        p.passes.keep_tool_down = down;
                        p.passes.finishing = finishing;
                        p.passes.finish_count = 2;
                        p.passes.spring_passes = 1;
                    }
                    _ => unreachable!(),
                }
                let (program, _) = post(&doc, "raised-stock-keep-down");
                let sim = simulate(
                    &program,
                    [-5., -5., -5.],
                    [65., 55., 21.],
                    &SimOptions {
                        resolution: 0.25,
                        tool_radius: 3.,
                    },
                    &[SimTool {
                        number: 1,
                        profile: ToolProfile::flat(3.),
                        noncutting_tip_radius: None,
                    }],
                );
                assert!(
                    sim.is_clean(),
                    "finishing {finishing}, down {down}: {:?}",
                    sim.collisions
                );
                // Both fixtures protect this point: the pocket exterior and
                // the profile interior must remain at the original stock top.
                assert_eq!(sim.field.sample(30., 20.), 21.);
                let retracts = program
                    .steps()
                    .iter()
                    .filter(
                        |s| matches!(s, Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract),
                    )
                    .count();
                if down {
                    assert!(retracts < lifted_retracts,
                        "finishing {finishing}: {retracts} retracts with keep-down, {lifted_retracts} without");
                } else {
                    lifted_retracts = retracts;
                }
            }
        }
    }
}

#[test]
fn profile_arc_leads_survive_keep_down_finishing_and_posting() {
    for angle_deg in [45_f64, 90., 135.] {
        for side in [Side::Inside, Side::Outside] {
            for finishing in [false, true] {
                let mut expected = None;
                for down in [false, true] {
                    let mut doc = roughing_doc();
                    doc.setups[0].heights = Heights::new(30., 25., 21.);
                    let Operation::Profile(p) = &mut doc.setups[0].operations[0] else {
                        unreachable!()
                    };
                    p.side = side;
                    p.stepover = 0.;
                    p.stepdown = 6.;
                    p.lead_in = Lead::ArcSweep {
                        radius: 3.,
                        angle_deg,
                    };
                    p.lead_out = Lead::ArcSweep {
                        radius: 3.,
                        angle_deg,
                    };
                    p.passes.finishing = finishing;
                    p.passes.keep_tool_down = down;
                    let (program, nc) = post(&doc, "profile-arc-leads");
                    program.validate().unwrap();
                    let leads: Vec<_> = program.steps().iter().filter(|step| matches!(step,
                    Step::Arc { tag, .. } if matches!(tag.kind, MoveKind::LeadIn | MoveKind::LeadOut)))
                    .cloned().collect();
                    assert_eq!(leads.len(), if finishing { 16 } else { 8 });
                    let mut position = None;
                    for step in program.steps() {
                        match step {
                            Step::Rapid { to, .. } | Step::Linear { to, .. } => {
                                position = Some(*to)
                            }
                            Step::Arc {
                                end,
                                center,
                                dir,
                                tag,
                                ..
                            } => {
                                if matches!(tag.kind, MoveKind::LeadIn | MoveKind::LeadOut) {
                                    let from = position.unwrap();
                                    assert!(
                                        ((from.x - center.x).hypot(from.y - center.y) - 3.).abs()
                                            < 1e-8
                                    );
                                    assert!(
                                        (cam_cldata::arc_sweep(from, *end, *center, *dir).abs()
                                            - angle_deg.to_radians())
                                        .abs()
                                            < 1e-8
                                    );
                                }
                                position = Some(*end);
                            }
                            _ => {}
                        }
                    }
                    let posted_arcs = nc
                        .lines()
                        .filter(|line| line.starts_with("G2 ") || line.starts_with("G3 "))
                        .count();
                    assert!(posted_arcs >= leads.len(), "{nc}");
                    if let Some(original) = &expected {
                        assert_eq!(&leads, original);
                    } else {
                        expected = Some(leads);
                    }
                }
            }
        }
    }
}
