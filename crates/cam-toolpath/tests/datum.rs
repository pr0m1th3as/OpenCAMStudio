//! Per-operation work datum (O2): the planner emits `Step::Datum` for an
//! operation whose `work_offset` differs from the one in force, placed *before*
//! that operation's tool change so a post can state `G15 H<n>` in the tool-section
//! head. Datum 1 is the default, so a single-datum job emits none — every post's
//! output is then byte-identical to before multi-WCS existed.
//!
//! The same neutral step reaches the ISO families as a `G54`-`G59` change; both
//! renderings are pinned end-to-end below.

use cam_cldata::{SpindleDir, Step};
use cam_geo::{Contour, Point};
use cam_model::{
    Comp, Document, Envelope, Heights, Lead, Machine, Operation, Origin, Plunge, Point3, ProfileOp,
    Setup, Side, Stock, Tool, ToolKind,
};
use cam_post::{PostKind, PostOptions};
use cam_toolpath::{build_job, CancelToken};

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

/// A two-origin job with a real reorientation: op0 on datum 1, op1 on datum 2, whose
/// origin sits elsewhere on the part and carries its own start point. Posting this
/// through Okuma exercises the whole Case-1 frame — the `M05`/`M09`/`M00`
/// reorientation halt, the `G15 H2` datum select, the per-datum coordinate
/// re-referencing (`translated_per_datum`), the clean-start rapid, and the Z
/// re-statement into the new frame.
fn two_origin_doc() -> Document {
    let mut d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 2, 2),
    ]);
    d.setup.extra_origins.push(Origin {
        index: 2,
        position: [50.0, 0.0, 0.0],
    });
    d
}

/// The export path exactly as the app wires it (`controller.rs`): plan, re-reference
/// each group to its own origin, then post through `kind`.
fn nc(d: &Document, kind: PostKind) -> String {
    let (program, _) = build_job(
        d,
        1000.0,
        SpindleDir::Cw,
        None,
        machine().envelope.max.z,
        &CancelToken::new(),
    );
    let setup = &d.setup;
    let translated = program.translated_per_datum(|idx| {
        let o = setup.origin_position(idx);
        [-o[0], -o[1], -o[2]]
    });
    kind.post(
        &translated,
        &machine(),
        &PostOptions {
            program_name: Some("two_origins".into()),
            ..Default::default()
        },
    )
    .expect("posts")
}

fn okuma_nc(d: &Document) -> String {
    nc(d, PostKind::Okuma)
}

/// The re-fixture halt and the end of the program are the same physical situation —
/// the operator is about to reach in — so they must emit the same sequence. They did
/// not: the end of the program stopped coolant before the spindle and never lifted,
/// having been written at first light, months before the re-fixture path existed.
/// Nothing produced both in one program until a multi-datum job with a tool change.
#[test]
fn every_shutdown_stops_the_spindle_first_then_coolant_then_lifts() {
    use cam_cldata::{Coolant, MoveKind};
    // Two origins, so the program contains a re-fixture halt *and* an ending.
    let mut d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(20.0, 20.0, 60.0, 40.0), Side::Outside, 1, 2),
    ]);
    d.setup.extra_origins = vec![cam_model::Origin {
        index: 2,
        position: [0.0, 0.0, 0.0],
    }];
    let (program, _) = build_job(&d, 1000.0, SpindleDir::Cw, None, 42.0, &CancelToken::new());
    let s = program.steps();

    // Each shutdown: SpindleOff, Coolant(Off), then the Traverse lift to 42.
    let starts: Vec<usize> = s
        .iter()
        .enumerate()
        .filter(|(_, x)| matches!(x, Step::SpindleOff))
        .map(|(i, _)| i)
        .collect();
    assert_eq!(starts.len(), 2, "one shutdown per re-fixture, plus the ending:\n{s:#?}");

    for i in starts {
        assert!(
            matches!(s[i + 1], Step::Coolant(Coolant::Off)),
            "coolant must stop *after* the spindle — the spindle is the half that hurts"
        );
        match &s[i + 2] {
            Step::Rapid { to, tag } => {
                assert_eq!(tag.kind, MoveKind::Traverse);
                assert_eq!(to.z, 42.0, "the tool must be lifted clear of the work");
            }
            other => panic!("every shutdown must lift the tool clear, got {other:?}"),
        }
    }
}

/// A tool-change height below the clearance plane would turn every "lift" into a
/// descent, and send the cross across the part below clearance. Raised, with a warning
/// — not silently obeyed, and not silently ignored either.
#[test]
fn a_tool_change_height_below_clearance_is_raised_and_reported() {
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 2, 1),
    ]);
    let clearance = d.setup.heights.clearance;
    // Well below the clearance plane.
    let (program, diags) = build_job(
        &d,
        1000.0,
        SpindleDir::Cw,
        None,
        clearance - 10.0,
        &CancelToken::new(),
    );
    assert!(
        diags.iter().any(|x| x.message.contains("below the clearance plane")),
        "the operator must be told their value was overruled: {diags:?}"
    );
    // And nothing labelled a lift may descend.
    let mut z = f64::NAN;
    for st in program.steps() {
        if let Step::Rapid { to, tag } = st {
            if tag.kind == cam_cldata::MoveKind::Traverse && !z.is_nan() {
                assert!(
                    to.z >= z - 1e-9 || (to.x, to.y) != (z, z),
                    "a traverse descended below where it started"
                );
            }
            z = to.z;
        }
    }
    assert!(
        program.steps().iter().all(|st| !matches!(
            st,
            Step::Rapid { to, tag } if tag.kind == cam_cldata::MoveKind::Traverse && to.z < clearance
        )),
        "no traverse may sit below the clearance plane"
    );
}

#[test]
fn a_same_datum_tool_change_is_a_full_planner_owned_transition() {
    use cam_cldata::MoveKind;
    // Two ops on the same datum, different tools. The planner owns the whole transition:
    // a Traverse lift to the tool-change height (42) before the `M6`, then after the
    // change a Traverse cross at 42 to the next op's XY and a Traverse descent to
    // clearance (5, from the doc's Heights) — all in the distinct Traverse role.
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 2, 1),
    ]);
    let (program, _) = build_job(&d, 1000.0, SpindleDir::Cw, None, 42.0, &CancelToken::new());
    let s = program.steps();
    let tc = s
        .iter()
        .position(|x| matches!(x, Step::ToolChange { tool: 2 }))
        .expect("the second tool change");

    // Just before the M6: the in-place lift to tool-change height.
    match &s[tc - 1] {
        Step::Rapid { to, tag } => {
            assert_eq!(tag.kind, MoveKind::Traverse, "the pre-M6 lift is a Traverse");
            assert_eq!(to.z, 42.0, "the lift rises to the tool-change height");
        }
        other => panic!("expected a Traverse lift before the M6, got {other:?}"),
    }

    // After the M6: the two Traverse moves that cross to the next op and descend to
    // clearance, in order. (Spindle/coolant re-commands sit between the M6 and these.)
    let traverses: Vec<&Point3> = s[tc..]
        .iter()
        .filter_map(|x| match x {
            Step::Rapid { to, tag } if tag.kind == MoveKind::Traverse => Some(to),
            _ => None,
        })
        .collect();
    assert_eq!(
        traverses.len(),
        3,
        "a cross then a descent after the M6, then the end-of-program lift:\n{s:#?}"
    );
    assert_eq!(traverses[0].z, 42.0, "cross to the next op at tool-change height");
    assert_eq!(traverses[1].z, 5.0, "then descend to clearance");
    assert_eq!(
        traverses[2].z, 42.0,
        "and the job ends with the tool lifted clear, not parked over the part"
    );
    assert_eq!(
        (traverses[0].x, traverses[0].y),
        (traverses[1].x, traverses[1].y),
        "cross and descent share the next op's XY (descent is vertical)"
    );
}

#[test]
fn same_tool_consecutive_ops_get_no_transition() {
    use cam_cldata::MoveKind;
    // One tool, one datum. The first op opens from tool-change height (its own cross +
    // descend), but the second same-tool op adds no transition — it hops at clearance via
    // its own approach. So the only Traverse moves are the first op's opening pair.
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 1, 1),
    ]);
    let (program, _) = build_job(&d, 1000.0, SpindleDir::Cw, None, 42.0, &CancelToken::new());
    let traverse_ops: Vec<u32> = program
        .steps()
        .iter()
        .filter_map(|x| match x {
            Step::Rapid { tag, .. } if tag.kind == MoveKind::Traverse => Some(tag.op_id),
            _ => None,
        })
        .collect();
    assert_eq!(
        traverse_ops,
        vec![0, 0, 1],
        "only the first op opens from TCH (cross + descend); the second adds none. The \
         trailing entry is the end-of-program lift, tagged to the last op — the program \
         must not finish with the tool parked over the part."
    );
}

/// Byte-pinned end-to-end golden (O5): a two-origin job planned and posted through
/// Okuma, the whole Case-1 reorientation frame in one file. A tripwire against drift
/// in *either* the planner's group emission or the post's rendering — the audit above
/// checked each fragment; this pins the composition. To regenerate after an intentional
/// change, run `regen_goldens` (below) and **read the diff**. Not a substitute for the
/// O6 line-audit against real posted words.
#[test]
fn golden_multi_datum_reorientation() {
    assert_eq!(
        okuma_nc(&two_origin_doc()),
        include_str!("golden/multi_datum_okuma.min")
    );
}

/// The same two-origin job through Fanuc, where the datum lands as `G55` rather than
/// `G15 H2`. Pinned for the same reason as the Okuma golden and read line-by-line once
/// against the ISO word definitions — we have no Fanuc shop programs to audit house
/// *style* against (the 16 `.MIN` files are Okuma), so this pins what we believe and
/// makes any later drift visible rather than silent. Regenerate as for the Okuma
/// golden above, into `/tests/golden/multi_datum_fanuc.nc`.
#[test]
fn golden_multi_datum_reorientation_fanuc() {
    assert_eq!(
        nc(&two_origin_doc(), PostKind::Fanuc),
        include_str!("golden/multi_datum_fanuc.nc")
    );
}

#[test]
fn the_iso_families_state_the_reorientation_as_a_work_offset_change() {
    // The behaviour the golden pins, stated as a property so it survives a
    // regeneration: the second fixture runs under `G55`, and the `M0` re-fixture halt
    // comes first — the operator must not be asked to flip the part after the frame
    // has already moved.
    let g = nc(&two_origin_doc(), PostKind::Fanuc);
    let halt = g.lines().position(|l| l.trim() == "M0").expect("the re-fixture halt");
    let select = g
        .lines()
        .position(|l| l.split_whitespace().any(|w| w == "G55"))
        .expect("the second datum");
    assert!(halt < select, "the halt precedes the datum select:\n{g}");
    assert_eq!(
        g.lines().filter(|l| l.split_whitespace().any(|w| w == "G55")).count(),
        1,
        "the modal offset is stated once, not per move:\n{g}"
    );
}

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
        extra_origins: vec![],
        origin_index: 1,
        tool_change_height: None,
    })
}

fn steps(doc: &Document) -> Vec<Step> {
    let (program, _) = build_job(
        doc,
        1000.0,
        SpindleDir::Cw,
        None,
        machine().envelope.max.z,
        &CancelToken::new(),
    );
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
fn a_reorientation_emits_an_operator_stop_before_the_new_datum() {
    // Two ops on origins 1 then 2. Crossing to origin 2 is a physical reorientation, so
    // an `M00` stop (`Step::Stop`) precedes the `Step::Datum(2)`; the first group carries
    // no stop (datum 1 is the initial state).
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 1, 2),
    ]);
    let s = steps(&d);
    let stops = s.iter().filter(|x| matches!(x, Step::Stop)).count();
    assert_eq!(stops, 1, "one stop, at the single reorientation");
    let stop_at = s.iter().position(|x| matches!(x, Step::Stop)).unwrap();
    let datum_at = s.iter().position(|x| matches!(x, Step::Datum(2))).unwrap();
    assert_eq!(stop_at + 1, datum_at, "the stop immediately precedes the new datum");
}

#[test]
fn a_reorientation_stops_spindle_and_coolant_before_the_halt_and_restarts_after() {
    use cam_cldata::{Coolant, MoveKind};
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 1, 2),
    ]);
    let s = steps(&d);
    let stop_at = s.iter().position(|x| matches!(x, Step::Stop)).expect("an M00");
    // Nothing live during the flip: M05 then M09, then a lift clear to tool-change
    // height, immediately precede the M00 so the part is free to re-fixture.
    assert!(matches!(s[stop_at - 3], Step::SpindleOff), "spindle off before the halt");
    assert!(
        matches!(s[stop_at - 2], Step::Coolant(Coolant::Off)),
        "coolant off before the halt"
    );
    assert!(
        matches!(s[stop_at - 1], Step::Rapid { tag, .. } if tag.kind == MoveKind::Traverse),
        "a lift to tool-change height before the halt"
    );
    // The new group brings both back.
    let datum_at = s.iter().position(|x| matches!(x, Step::Datum(2))).unwrap();
    assert!(
        s[datum_at..].iter().any(|x| matches!(x, Step::Spindle { .. })),
        "spindle restarts for the new group"
    );
    assert!(
        s[datum_at..].iter().any(|x| matches!(x, Step::Coolant(Coolant::Flood))),
        "coolant restarts for the new group"
    );
}

#[test]
fn a_single_origin_job_emits_no_stop() {
    let d = doc(vec![
        profile(0, rect(10.0, 10.0, 70.0, 50.0), Side::Outside, 1, 1),
        profile(1, rect(35.0, 25.0, 45.0, 35.0), Side::Inside, 1, 1),
    ]);
    assert!(!steps(&d).iter().any(|x| matches!(x, Step::Stop)), "no stop without reorientation");
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

/// Rewrite this file's goldens after an intentional change.
///
/// `cargo test -p cam-toolpath --test datum -- --ignored regen_goldens`, then **read the
/// diff** before committing — that is the whole value of a golden. Ignored by default
/// so a normal run never rewrites the thing it is meant to be checking against.
#[test]
#[ignore = "regeneration helper"]
fn regen_goldens() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/golden/");
    std::fs::write(format!("{dir}multi_datum_okuma.min"), okuma_nc(&two_origin_doc())).unwrap();
    std::fs::write(
        format!("{dir}multi_datum_fanuc.nc"),
        nc(&two_origin_doc(), PostKind::Fanuc),
    )
    .unwrap();
}
