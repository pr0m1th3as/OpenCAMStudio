//! P2 acceptance tests for the grbl post: a hand-built toolpath lowered to
//! G-code, proven two ways — a **golden file** (byte-for-byte determinism) and a
//! **semantic check** that parses the emitted G-code and asserts real safety /
//! correctness invariants (spindle-before-cut, no lateral rapid through stock,
//! everything inside the envelope, clean program end).

use cam_cldata::{
    ArcDir, Coolant, DrillCycle, MoveKind, Point3, Program, ProgramBuilder, SpindleDir, Tag,
};
use cam_model::{Envelope, Machine};
use cam_post::{GrblPost, Post, PostError, PostOptions};

/// A small hobby-class 3-axis machine. Z runs from −50 (deep) to +50 (above the
/// table); WCS Z0 is the top of stock by convention.
fn machine() -> Machine {
    Machine {
        name: "OCS-3018".into(),
        rapid_rate: 2000.0,
        max_spindle_rpm: 10_000.0,
        max_feed: 800.0,
        envelope: Envelope::new(
            Point3::new(0.0, 0.0, -50.0),
            Point3::new(300.0, 180.0, 50.0),
        ),
        safe_z: 5.0,
        tool_change_pos: None,
    }
}

/// A representative job: tool change, spindle + coolant on, a rectangular profile
/// with one quarter-arc corner at Z −1, then a two-hole peck-drill cycle, then
/// spindle + coolant off.
fn sample_program() -> Program {
    ProgramBuilder::new()
        .comment("OpenCAMStudio sample")
        .tool_change(1)
        .spindle_on(1000.0, SpindleDir::Cw)
        .coolant(Coolant::Flood)
        // op 0 — profile
        .op(0)
        .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
        .feed(100.0)
        .linear(Point3::new(10.0, 10.0, -1.0), MoveKind::Plunge)
        .feed(300.0)
        .linear(Point3::new(50.0, 10.0, -1.0), MoveKind::Cutting)
        .arc(
            Point3::new(60.0, 20.0, -1.0),
            Point3::new(50.0, 20.0, -1.0),
            ArcDir::Ccw,
            MoveKind::Cutting,
        )
        .linear(Point3::new(60.0, 50.0, -1.0), MoveKind::Cutting)
        .linear(Point3::new(10.0, 50.0, -1.0), MoveKind::Cutting)
        .linear(Point3::new(10.0, 10.0, -1.0), MoveKind::Cutting)
        .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Retract)
        // op 1 — peck drilling
        .drill(DrillCycle {
            points: vec![[20.0, 30.0], [40.0, 30.0]],
            z_top: 0.0,
            depth: -8.0,
            retract: 2.0,
            peck: Some(3.0),
            dwell: Some(0.5),
            feed: 120.0,
            tag: Tag::new(1, MoveKind::Plunge),
        })
        .spindle_off()
        .coolant(Coolant::Off)
        .build()
}

fn post_sample() -> String {
    let opts = PostOptions {
        program_name: Some("sample".into()),
        ..Default::default()
    };
    GrblPost
        .post(&sample_program(), &machine(), &opts)
        .expect("post should succeed")
}

#[test]
fn dump() {
    // Inspection helper: `cargo test -p cam-post dump -- --nocapture`.
    println!("\n{}", post_sample());
}

// ---------------------------------------------------------------------------
// Golden file — byte-for-byte determinism
// ---------------------------------------------------------------------------

#[test]
fn golden_output_is_stable() {
    let got = post_sample();
    let want = include_str!("golden/sample.nc");
    assert_eq!(
        got, want,
        "grbl output drifted from tests/golden/sample.nc; if the change is \
         intended, regenerate the golden file"
    );
}

// ---------------------------------------------------------------------------
// Semantic check — parse the emitted G-code and assert real invariants
// ---------------------------------------------------------------------------

/// Parse grbl output (modal, absolute/G90) and assert safety + correctness
/// invariants. `stock_top` is the WCS Z above which lateral rapids are safe.
fn assert_gcode_is_sound(gcode: &str, m: &Machine, stock_top: f64) {
    let mut g: Option<u8> = None;
    let (mut x, mut y, mut z) = (f64::NAN, f64::NAN, f64::NAN);
    let mut spindle_on = false;
    let mut spindle_ok_before_cut: Option<bool> = None;
    let mut saw_units = false;
    let mut saw_absolute = false;
    let mut saw_work_offset = false;
    let mut ended = false;
    let mut last_line = "";

    for raw in gcode.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('(') {
            continue;
        }
        last_line = line;

        let (mut had_xy, mut had_coord, mut motion) = (false, false, false);
        for tok in line.split_whitespace() {
            let (letter, rest) = tok.split_at(1);
            match letter {
                "G" => match rest {
                    "0" | "1" | "2" | "3" => {
                        g = Some(rest.parse().unwrap());
                        motion = true;
                    }
                    "21" => saw_units = true,
                    "90" => saw_absolute = true,
                    "54" | "55" | "56" | "57" | "58" | "59" => saw_work_offset = true,
                    _ => {}
                },
                "M" => match rest {
                    "3" | "4" => spindle_on = true,
                    "5" => spindle_on = false,
                    "30" => ended = true,
                    _ => {}
                },
                "X" => {
                    x = rest.parse().unwrap();
                    had_xy = true;
                    had_coord = true;
                }
                "Y" => {
                    y = rest.parse().unwrap();
                    had_xy = true;
                    had_coord = true;
                }
                "Z" => {
                    z = rest.parse().unwrap();
                    had_coord = true;
                }
                _ => {} // F, S, I, J, P, T
            }
        }
        let _ = motion;

        // Spindle must be running by the first cutting motion.
        let is_cut = matches!(g, Some(1) | Some(2) | Some(3)) && had_coord;
        if is_cut && spindle_ok_before_cut.is_none() {
            spindle_ok_before_cut = Some(spindle_on);
        }

        // Every reached coordinate must be inside the work envelope.
        if had_coord && x.is_finite() && y.is_finite() && z.is_finite() {
            assert!(
                m.envelope.contains(x, y, z),
                "coordinate outside envelope: {line}"
            );
        }

        // A rapid that moves in XY must stay at or above the stock top — never
        // drag the tool through material at G0.
        if g == Some(0) && had_xy {
            assert!(
                z >= stock_top - 1e-9,
                "lateral rapid below stock top ({stock_top}): {line} (z={z})"
            );
        }
    }

    assert!(
        saw_units && saw_absolute && saw_work_offset,
        "preamble must set mm (G21), absolute (G90), and a work offset (G54–G59)"
    );
    assert_eq!(
        spindle_ok_before_cut,
        Some(true),
        "spindle must be on before the first cutting move"
    );
    assert!(!spindle_on, "spindle must be off at program end");
    assert!(ended, "program must contain M30");
    assert_eq!(last_line, "M30", "M30 must be the final line");
}

#[test]
fn sample_gcode_is_semantically_sound() {
    assert_gcode_is_sound(&post_sample(), &machine(), 0.0);
}

// ---------------------------------------------------------------------------
// Capabilities + machine-limit queries + cycle lowering
// ---------------------------------------------------------------------------

#[test]
fn grbl_has_no_canned_cycles_but_does_arcs() {
    let caps = GrblPost.capabilities();
    assert!(caps.arcs, "grbl interpolates arcs natively");
    assert!(
        !caps.canned_drill,
        "grbl has no canned cycles — must expand pecks"
    );
}

#[test]
fn peck_cycle_expands_to_one_g1_per_peck() {
    // Single hole, 0 → −8 in 3 mm pecks ⇒ pecks at −3, −6, −8: three G1 plunges.
    let prog = ProgramBuilder::new()
        .spindle_on(1000.0, SpindleDir::Cw)
        .drill(DrillCycle {
            points: vec![[20.0, 30.0]],
            z_top: 0.0,
            depth: -8.0,
            retract: 2.0,
            peck: Some(3.0),
            dwell: None,
            feed: 120.0,
            tag: Tag::new(0, MoveKind::Plunge),
        })
        .build();
    let gcode = GrblPost
        .post(&prog, &machine(), &PostOptions::default())
        .unwrap();
    let g1_moves = gcode.lines().filter(|l| l.starts_with("G1 ")).count();
    assert_eq!(g1_moves, 3, "expected three peck plunges\n{gcode}");
    assert!(gcode.contains("Z-8.000"), "must reach final depth");
}

#[test]
fn spindle_over_machine_max_is_rejected() {
    let prog = ProgramBuilder::new()
        .spindle_on(20_000.0, SpindleDir::Cw)
        .build();
    assert_eq!(
        GrblPost.post(&prog, &machine(), &PostOptions::default()),
        Err(PostError::SpindleOutOfRange(20_000.0))
    );
}

#[test]
fn feed_over_machine_max_is_rejected() {
    let prog = ProgramBuilder::new()
        .op(0)
        .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
        .feed(900.0)
        .linear(Point3::new(10.0, 0.0, -1.0), MoveKind::Cutting)
        .build();
    assert_eq!(
        GrblPost.post(&prog, &machine(), &PostOptions::default()),
        Err(PostError::FeedOutOfRange(900.0))
    );
}

#[test]
fn a_toolpath_wider_than_the_travel_is_rejected() {
    // The machine travels 300 mm in X; a path spanning 0..400 can't fit no matter
    // where the operator zeroes it.
    let prog = ProgramBuilder::new()
        .op(0)
        .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
        .rapid(Point3::new(400.0, 0.0, 5.0), MoveKind::Link)
        .build();
    assert!(matches!(
        GrblPost.post(&prog, &machine(), &PostOptions::default()),
        Err(PostError::TravelExceeded { axis: 'X', .. })
    ));
}

#[test]
fn negative_work_coordinates_that_fit_the_travel_post_fine() {
    // Outside profiling with the datum on the part corner puts moves at negative
    // work coordinates (x = -radius). As long as the *span* fits the travel that is
    // valid — the operator's G54 offset places it within the machine. This is the
    // case the old absolute-position check wrongly rejected.
    let prog = ProgramBuilder::new()
        .op(0)
        .rapid(Point3::new(-3.0, 40.0, 5.0), MoveKind::Link)
        .feed(300.0)
        .linear(Point3::new(60.0, 40.0, -2.0), MoveKind::Cutting)
        .build();
    assert!(
        GrblPost
            .post(&prog, &machine(), &PostOptions::default())
            .is_ok(),
        "a small path at negative work coords must post"
    );
}

#[test]
fn arc_without_a_prior_position_is_rejected() {
    let prog = ProgramBuilder::new()
        .op(0)
        .feed(300.0)
        .arc(
            Point3::new(10.0, 0.0, -1.0),
            Point3::new(5.0, 0.0, -1.0),
            ArcDir::Ccw,
            MoveKind::Cutting,
        )
        .build();
    assert_eq!(
        GrblPost.post(&prog, &machine(), &PostOptions::default()),
        Err(PostError::ArcWithoutStart)
    );
}
