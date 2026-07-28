//! The Okuma OSP post — a fourth output family, verified where it diverges from the
//! Fanuc-shaped families. O1 scope: the frame skeleton (defensive safe-start, no
//! wrapper, per-tool-section `G15`/`G56`, `M02` end, `G04 F` dwell, native arcs) and
//! the deliberate refusal of drilling until the `G71`/`M53` frame lands (O3).

use cam_cldata::{
    ArcDir, DrillCycle, MoveKind, Point3, Program, ProgramBuilder, SpindleDir, Tag,
};
use cam_model::{Envelope, Machine};
use cam_post::{PostKind, PostOptions};

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

fn opts() -> PostOptions {
    PostOptions {
        program_name: Some("part".into()),
        ..Default::default()
    }
}

/// A milling program that exercises the Okuma frame: a tool change, spindle, a
/// standalone dwell, a linear cut and a native arc.
fn mill_program() -> Program {
    ProgramBuilder::new()
        .comment("part")
        .tool_change(88)
        .spindle_on(2779.0, SpindleDir::Cw)
        .dwell(0.5)
        .feed(600.0)
        .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
        .linear(Point3::new(10.0, 20.0, -3.0), MoveKind::Cutting)
        .arc(
            Point3::new(20.0, 30.0, -3.0),
            Point3::new(20.0, 20.0, -3.0),
            ArcDir::Ccw,
            MoveKind::Cutting,
        )
        .spindle_off()
        .build()
}

fn okuma(program: &Program) -> String {
    PostKind::Okuma
        .post(program, &machine(), &opts())
        .expect("posts")
}

#[test]
fn okuma_appears_in_the_picker_as_a_seventh_post() {
    assert_eq!(PostKind::ALL.len(), 7);
    assert!(PostKind::ALL.contains(&PostKind::Okuma));
    assert_eq!(PostKind::Okuma.to_string(), "Okuma");
}

#[test]
fn okuma_exports_default_to_the_min_extension() {
    // OSP programs are .MIN files; every other dialect stays .nc.
    assert_eq!(PostKind::Okuma.file_extensions(), &["min"]);
    assert_eq!(PostKind::Okuma.default_file_name(), "program.min");
    assert_eq!(PostKind::Fanuc.file_extensions(), &["nc"]);
    assert_eq!(PostKind::Grbl.default_file_name(), "program.nc");
}

#[test]
fn frame_is_unwrapped_with_a_defensive_safe_start_and_m02_end() {
    let g = okuma(&mill_program());
    // No `%`/O-number wrapper — the file name is the program name on OSP.
    assert!(!g.contains('%'), "Okuma output is not %-wrapped:\n{g}");
    assert!(!g.contains("O1000"), "no O-number:\n{g}");
    // Defensive safe-start (OKUMA_PLAN §6b), and none of the Fanuc opener's
    // G40/G49/G80 (G49 is not a tool-length cancel on Okuma).
    assert!(g.contains("G21 G17 G90 G94"), "defensive safe-start:\n{g}");
    assert!(!g.contains("G49"), "no G49 on Okuma:\n{g}");
    // Program end is M02, never M30.
    assert!(g.contains("M02"), "ends with M02:\n{g}");
    assert!(!g.contains("M30"), "never M30:\n{g}");
}

#[test]
fn tool_section_carries_g15_wcs_and_g56_tool_length() {
    let g = okuma(&mill_program());
    assert!(g.contains("T88 M6"), "tool change:\n{g}");
    assert!(g.contains("G15 H1"), "work-coordinate select:\n{g}");
    // Tool-length offset is keyed to the tool number, and lives in a different H
    // number space than G15 (OKUMA_PLAN §3).
    assert!(g.contains("G56 H88"), "tool-length offset keyed to the tool:\n{g}");
}

#[test]
fn standalone_dwell_is_g04_f_seconds() {
    let g = okuma(&mill_program());
    let dwell = g
        .lines()
        .find(|l| l.starts_with("G4 "))
        .expect("a standalone dwell line");
    assert_eq!(dwell, "G4 F0.5", "OSP dwell is G04 F<sec>:\n{g}");
}

#[test]
fn arcs_are_native_with_incremental_ij() {
    let g = okuma(&mill_program());
    // CCW arc from start (10,20) about centre (20,20): incremental I/J = centre -
    // start = (10, 0), the convention verified numerically in OKUMA_PLAN §3.
    let arc = g
        .lines()
        .find(|l| l.starts_with("G3"))
        .expect("a native G3 arc line");
    assert!(arc.contains("I10.000") && arc.contains("J0.000"), "incremental I/J:\n{arc}");
}

/// A human-readable dump of the Okuma frame. Not an assertion — run it to see the
/// output for the O6 audit loop:
///   `cargo test -p cam-post --test okuma reference_dump -- --nocapture`
#[test]
fn reference_dump() {
    println!("\n===== Okuma OSP: milling =====\n{}", okuma(&mill_program()));
    println!(
        "\n===== Okuma OSP: peck drilling (G71/M53) =====\n{}",
        okuma(&drill_program(DrillCycle {
            peck: Some(3.0),
            ..base_cycle()
        }))
    );
}

#[test]
fn a_new_tools_first_move_restates_its_motion_word() {
    // Finding 1 (O6 audit): the shop always re-emits G00 after a tool change rather
    // than rely on a modal G0 carrying across M6, and OSP may reset the interpolation
    // group on a change. So the first move of a second tool must carry its own G-word,
    // not appear as bare coordinates.
    let program = ProgramBuilder::new()
        .tool_change(1)
        .spindle_on(1000.0, SpindleDir::Cw)
        .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
        .tool_change(2)
        .rapid(Point3::new(40.0, 30.0, 5.0), MoveKind::Link)
        .build();
    let g = okuma(&program);
    let first_after_t2 = g
        .lines()
        .skip_while(|l| *l != "G56 H2")
        .skip(1)
        .find(|l| l.starts_with('G') || l.starts_with('X'))
        .expect("a move after the second tool change");
    assert!(
        first_after_t2.starts_with("G0 "),
        "the second tool's first move must re-state G0, not rely on modal carry:\n{g}"
    );
}

/// Build a single-op drilling program with the given cycle, so a test can assert the
/// exact G71/M53 frame it lowers to.
fn drill_program(cycle: DrillCycle) -> Program {
    ProgramBuilder::new()
        .tool_change(3)
        .spindle_on(1200.0, SpindleDir::Cw)
        .drill(cycle)
        .spindle_off()
        .build()
}

fn base_cycle() -> DrillCycle {
    DrillCycle {
        points: vec![[20.0, 30.0], [40.0, 30.0]],
        z_top: 0.0,
        depth: -8.0,
        retract: 2.0,
        peck: None,
        dwell: None,
        feed: 120.0,
        tag: Tag::new(0, MoveKind::Plunge),
    }
}

/// The lines of the drill cycle proper: from the `G71` return-level declaration
/// through the `G80` cancel, inclusive.
fn cycle_lines(g: &str) -> Vec<&str> {
    let from = g.lines().position(|l| l.starts_with("G71 ")).expect("a G71");
    let to = g.lines().position(|l| l == "G80").expect("a G80");
    g.lines().skip(from).take(to - from + 1).collect()
}

#[test]
fn peck_cycle_is_g83_with_g71_return_level_and_m53() {
    let g = okuma(&drill_program(DrillCycle {
        peck: Some(3.0),
        ..base_cycle()
    }));
    assert_eq!(
        cycle_lines(&g),
        [
            "G71 Z2.000",
            "G83 X20.000 Y30.000 Z-8.000 R2.000 Q3 F120 M53",
            "X40.000 Y30.000",
            "G80",
        ],
        "peck -> G83 with G71/M53 frame, M53 on the cycle line only:\n{g}"
    );
}

#[test]
fn dwell_without_peck_is_g82_with_p() {
    let g = okuma(&drill_program(DrillCycle {
        dwell: Some(0.4),
        ..base_cycle()
    }));
    assert!(
        cycle_lines(&g).contains(&"G82 X20.000 Y30.000 Z-8.000 R2.000 P0.4 F120 M53"),
        "dwell-without-peck -> G82 P:\n{g}"
    );
}

#[test]
fn plain_hole_is_g81() {
    let g = okuma(&drill_program(base_cycle()));
    assert!(
        cycle_lines(&g).contains(&"G81 X20.000 Y30.000 Z-8.000 R2.000 F120 M53"),
        "plain drill -> G81:\n{g}"
    );
}

#[test]
fn g71_return_level_is_the_retract_plane() {
    // The M53 level is the cycle's own clearance plane, so a raised retract raises it.
    let g = okuma(&drill_program(DrillCycle {
        retract: 15.0,
        ..base_cycle()
    }));
    assert!(cycle_lines(&g).contains(&"G71 Z15.000"), "G71 tracks retract:\n{g}");
}

#[test]
fn g80_auto_m05_is_left_alone_when_the_section_ends() {
    // The common case: drill then spindle-off. G80's auto-M05 is harmless, so no
    // spindle is re-asserted between the cycle and the M5.
    let g = okuma(&drill_program(base_cycle()));
    let after_g80: Vec<&str> = g
        .lines()
        .skip_while(|l| *l != "G80")
        .skip(1)
        .take_while(|l| *l != "M02")
        .collect();
    assert!(
        !after_g80.iter().any(|l| l.starts_with("M3") || l.starts_with("M4")),
        "no spurious spindle restart before the section ends:\n{g}"
    );
}

#[test]
fn g80_auto_m05_is_compensated_when_cutting_continues() {
    // Drill, then keep milling in the same section under the same spindle. OSP's G80
    // auto-M05 would kill the spindle mid-section, so the post re-asserts it.
    let program = ProgramBuilder::new()
        .tool_change(3)
        .spindle_on(1200.0, SpindleDir::Cw)
        .drill(base_cycle())
        .feed(300.0)
        .linear(Point3::new(50.0, 50.0, -2.0), MoveKind::Cutting)
        .spindle_off()
        .build();
    let g = okuma(&program);
    let restart = g
        .lines()
        .skip_while(|l| *l != "G80")
        .skip(1)
        .take_while(|l| !l.starts_with('G'))
        .find(|l| l.starts_with("M3"));
    assert_eq!(
        restart,
        Some("M3 S1200"),
        "spindle re-asserted after G80 before the continued cut:\n{g}"
    );
}
