//! **A rapid never ends on a cut floor.**
//!
//! Every strategy that cuts in stepdown passes has to get the tool back down to the
//! previous level before taking the next bite, and each of them used to do it by
//! rapiding to exactly the previous floor — legal against the geometry model, since
//! that material really is gone in the model, and therefore clean to `cam-sim`, whose
//! test is material-based. The machine is not the model: an uncut cusp, stock that
//! sprang back, a pass that ran fractionally shallow, or a small Z-zero error all put
//! metal where the model has air, and a rapid is the one move that cannot cut through
//! the difference.
//!
//! So the rule, shared by `emit::descend_to`: a rapid stops `FLOOR_CLEARANCE` above the
//! floor it is returning to and the last fraction is fed. Found by auditing a real
//! Fanuc export (`samples/fanuc_multiple origins.nc`, line 45: `G0 Z-2.000`).

use cam_cldata::{MoveKind, Step};
use cam_geo::{Contour, Point};
use cam_model::{
    ChamferOp, ClearParams, Clearing, Comp, Document, Envelope, Heights, Lead, Machine, Operation,
    Plunge, Point3, PocketOp, ProfileOp, Setup, Side, Stock, Tool, ToolKind,
};
use cam_toolpath::{build_job, CancelToken};

/// Clearance 5, retract 2, stock top 0 — so the stock top is the interesting boundary.
fn heights() -> Heights {
    Heights::new(5.0, 2.0, 0.0)
}

fn machine() -> Machine {
    Machine {
        name: "test".into(),
        rapid_rate: 2000.0,
        max_spindle_rpm: 10_000.0,
        max_feed: 800.0,
        envelope: Envelope::new(Point3::new(0.0, 0.0, -50.0), Point3::new(300.0, 180.0, 50.0)),
        safe_z: 5.0,
        tool_change_pos: None,
    }
}

/// The invariant, stated as the rule states it: **once cutting has begun, every rapid
/// ends at least `FLOOR_CLEARANCE` above the deepest Z cut so far.**
///
/// Tracking the running deepest level, rather than testing against every level in the
/// program, is what makes this work on a chamfer — whose passes are 0.5 mm apart, so
/// stopping 0.5 mm above the current floor lands exactly on the *previous* one, which
/// is a coincidence of numbers and not a hazard. What matters is that the rapid does
/// not come to rest on the floor it is returning to.
///
/// Single-operation fixtures only: with two operations the deepest level of the first
/// would constrain rapids belonging to the second, which cuts somewhere else entirely.
fn assert_no_rapid_lands_on_a_cut_floor(steps: &[Step], what: &str) {
    const FLOOR_CLEARANCE: f64 = 0.5; // mirrors `emit::FLOOR_CLEARANCE` (crate-private)
    let mut deepest: Option<f64> = None;
    let mut checked = 0;
    for (i, s) in steps.iter().enumerate() {
        match s {
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                deepest = Some(deepest.map_or(to.z, |d: f64| d.min(to.z)));
            }
            Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => {
                deepest = Some(deepest.map_or(end.z, |d: f64| d.min(end.z)));
            }
            Step::Rapid { to, .. } => {
                let Some(floor) = deepest else { continue };
                checked += 1;
                assert!(
                    to.z >= floor + FLOOR_CLEARANCE - 1e-9,
                    "{what}: rapid #{i} ends at Z{:.4}, only {:.4} above the cut floor \
                     Z{floor:.4} — the move that meets a cusp must be a feed",
                    to.z,
                    to.z - floor
                );
            }
            _ => {}
        }
    }
    assert!(checked > 0, "{what}: no rapid followed a cut, so this proves nothing");
}

fn steps_of(d: &Document) -> Vec<Step> {
    let (program, diags) = build_job(
        d,
        1000.0,
        cam_cldata::SpindleDir::Cw,
        None,
        machine().envelope.max.z,
        &CancelToken::new(),
    );
    assert!(
        !diags.iter().any(|x| x.severity == cam_toolpath::Severity::Error),
        "the fixture must actually cut: {diags:?}"
    );
    program.steps().to_vec()
}

#[test]
fn a_leaded_profile_never_rapids_onto_its_previous_floor() {
    // The case the export showed: leads put the pass's exit somewhere other than its
    // entry, so the tool lifts between passes and has to come back down.
    let d = doc(vec![profile_with(Lead::Arc { radius: 3.0 }, 2.0, 6.0)]);
    assert_no_rapid_lands_on_a_cut_floor(&steps_of(&d), "leaded profile");
}

#[test]
fn an_unleaded_profile_does_not_lift_between_passes_at_all() {
    // With no lead the pass ends where the next one starts, so the reposition the lift
    // existed for does not exist. Staying down is both quicker and — the point here —
    // a descent that never happens cannot end in metal. Below the stock top the only
    // rapids left are the approach and the final retract, neither of which descends
    // onto a floor.
    let d = doc(vec![profile_with(Lead::None, 2.0, 6.0)]);
    let steps = steps_of(&d);
    assert_no_rapid_lands_on_a_cut_floor(&steps, "unleaded profile");
    let descending_below_stock = steps
        .iter()
        .filter(|s| matches!(s, Step::Rapid { to, .. } if to.z < 0.0))
        .count();
    assert_eq!(
        descending_below_stock, 0,
        "an unleaded profile has no business rapiding below the stock top:\n{steps:#?}"
    );
}

#[test]
fn an_unleaded_chamfer_does_not_lift_between_passes_either() {
    // Every chamfer pass runs the same XY loop at a deeper Z, so without a lead or a
    // closure overlap the lift between passes returns the tool to exactly where it
    // already was. Same reasoning as the profile above, and the same benefit twice
    // over: fewer moves, and no descent to get wrong.
    let steps = steps_of(&doc(vec![chamfer()]));
    let below_stock = steps
        .iter()
        .filter(|s| matches!(s, Step::Rapid { to, .. } if to.z < 0.0))
        .count();
    assert_eq!(
        below_stock, 0,
        "an unleaded chamfer has no business rapiding below the stock top:\n{steps:#?}"
    );
    // The operation's own rapids to clearance are now just its approach and its final
    // retract — not one pair per pass. (The planner's Traverse descent from the
    // tool-change height also lands at clearance and is not the operation's doing.)
    let to_clearance = steps
        .iter()
        .filter(|s| {
            matches!(s, Step::Rapid { to, tag }
                if (to.z - 5.0).abs() < 1e-9 && tag.kind != MoveKind::Traverse)
        })
        .count();
    assert_eq!(
        to_clearance, 2,
        "three passes, but only one approach and one retract:\n{steps:#?}"
    );
}

#[test]
fn a_leaded_chamfer_still_lifts_and_still_descends_safely() {
    // With a lead the exit is elsewhere, so the lift is real work — and then the
    // descent rule has to hold, which is what this checks.
    let mut op = chamfer();
    if let cam_model::Operation::Chamfer(c) = &mut op {
        c.lead_in = Lead::Arc { radius: 2.0 };
        c.lead_out = Lead::Arc { radius: 2.0 };
    }
    assert_no_rapid_lands_on_a_cut_floor(&steps_of(&doc(vec![op])), "leaded chamfer");
}

#[test]
fn a_pocket_never_rapids_onto_its_previous_floor() {
    // Area clearing re-enters constantly — it is where the pattern was most expensive.
    let d = doc(vec![pocket()]);
    assert_no_rapid_lands_on_a_cut_floor(&steps_of(&d), "pocket");
}

#[test]
fn a_chamfer_never_rapids_onto_its_previous_width() {
    let d = doc(vec![chamfer()]);
    assert_no_rapid_lands_on_a_cut_floor(&steps_of(&d), "chamfer");
}

#[test]
fn the_first_descent_is_unchanged() {
    // The rule must not disturb the approach: above the stock top there is nothing to
    // hit, so the first rapid still goes all the way to the retract plane (2.0) and
    // nothing is fed through open air.
    let steps = steps_of(&doc(vec![profile_with(Lead::None, 2.0, 6.0)]));
    let first = steps
        .iter()
        .filter_map(|s| match s {
            Step::Rapid { to, .. } if to.z < 5.0 => Some(to.z),
            _ => None,
        })
        .next()
        .expect("an approach");
    assert_eq!(first, 2.0, "the approach still rapids to the retract plane");
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Contour {
    Contour::new(vec![
        Point::new(x0, y0),
        Point::new(x1, y1 - (y1 - y0)),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ])
}

fn profile_with(lead: Lead, stepdown: f64, depth: f64) -> Operation {
    Operation::Profile(ProfileOp {
        spindle_rpm: 0.0,
        work_offset: 1,
        clearing: Clearing::default(),
        id: 0,
        tool: 1,
        chain: rect(10.0, 10.0, 70.0, 50.0),
        side: Side::Outside,
        comp: Comp::Computed,
        offset: 0.0,
        depth,
        stepdown,
        stepover: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: lead,
        lead_out: lead,
        lead_overlap: 0.0,
        plunge: Plunge::Straight,
    })
}

fn pocket() -> Operation {
    Operation::Pocket(PocketOp {
        spindle_rpm: 0.0,
        work_offset: 1,
        id: 1,
        tool: 1,
        boundary: rect(10.0, 10.0, 70.0, 50.0),
        islands: vec![],
        depth: 6.0,
        start: None,
        clear: ClearParams {
            stepdown: 2.0,
            overlap: 0.4,
            offset: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            clearing: Clearing::default(),
            plunge: Plunge::Straight,
            lead_overlap: 0.0,
            lead_in: Lead::None,
            lead_out: Lead::None,
        },
    })
}

fn chamfer() -> Operation {
    Operation::Chamfer(ChamferOp {
        spindle_rpm: 0.0,
        work_offset: 1,
        id: 2,
        tool: 2,
        chain: rect(10.0, 10.0, 70.0, 50.0),
        side: Side::Outside,
        width: 1.5,
        step: 0.5, // three passes of increasing width
        top: 0.0,
        depth: 0.0,
        feed: 300.0,
        plunge_feed: 100.0,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: 0.0,
        gradual: false,
        start: None,
    })
}

fn tool(number: u32, kind: ToolKind) -> Tool {
    Tool {
        number,
        diameter: 6.0,
        length: 30.0,
        flutes: 2,
        kind,
        ..Default::default()
    }
}

fn doc(ops: Vec<Operation>) -> Document {
    Document::new(Setup {
        name: "descent".into(),
        heights: heights(),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: 0.0,
            thickness: 10.0,
        },
        tools: vec![tool(1, ToolKind::EndMill), tool(2, ToolKind::ChamferMill { included_angle_deg: 90.0, tip_diameter: 0.5 })],
        operations: ops,
        origin: [0.0, 0.0, 0.0],
        extra_origins: vec![],
        origin_index: 1,
        tool_change_height: None,
    })
}
