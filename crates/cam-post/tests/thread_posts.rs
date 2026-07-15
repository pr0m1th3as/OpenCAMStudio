//! Acceptance test for **helical** arc output — the motion a thread-milling
//! operation is built from. A helix is a `Step::Arc` whose end Z differs from the
//! start Z; a correct post must put the `Z` advance **and** the `I`/`J` centre
//! offset on the same `G2`/`G3` block. This proves both posts do, so thread
//! milling lowers to valid G-code on real controllers.

use cam_cldata::{ArcDir, MoveKind, Point3, Program, ProgramBuilder, SpindleDir};
use cam_model::{Envelope, Machine};
use cam_post::{FanucPost, GrblPost, Post, PostOptions};

fn machine() -> Machine {
    Machine {
        name: "OCS-3018".into(),
        rapid_rate: 2000.0,
        max_spindle_rpm: 10_000.0,
        max_feed: 800.0,
        envelope: Envelope::new(Point3::new(0.0, 0.0, -50.0), Point3::new(300.0, 180.0, 50.0)),
        safe_z: 5.0,
        tool_change_pos: None,
    }
}

/// One half-turn of a helix about (10, 0): from the +X side of the orbit down to
/// the −X side, descending 0.75 mm (half of a 1.5 mm pitch).
fn helix_program() -> Program {
    ProgramBuilder::new()
        .spindle_on(1000.0, SpindleDir::Cw)
        .op(0)
        .rapid(Point3::new(12.5, 0.0, 5.0), MoveKind::Link)
        .feed(200.0)
        .linear(Point3::new(12.5, 0.0, -6.0), MoveKind::Plunge)
        .arc(
            Point3::new(7.5, 0.0, -5.25),
            Point3::new(10.0, 0.0, -5.25),
            ArcDir::Ccw,
            MoveKind::Cutting,
        )
        .build()
}

/// Assert the posted G-code has a `G2`/`G3` block that carries a `Z` word and both
/// `I` and `J` — i.e. a genuine helical arc, not a flat one.
fn assert_has_helical_arc(gcode: &str) {
    let mut modal_arc = false;
    for raw in gcode.lines() {
        let line = raw.trim();
        // Track the modal motion mode; an arc block may omit G2/G3 if unchanged.
        for tok in line.split_whitespace() {
            match tok {
                "G2" | "G3" | "G02" | "G03" => modal_arc = true,
                "G0" | "G1" | "G00" | "G01" => modal_arc = false,
                _ => {}
            }
        }
        if !modal_arc {
            continue;
        }
        let has = |p: char| line.split_whitespace().any(|t| t.starts_with(p));
        if has('Z') && has('I') && has('J') {
            return;
        }
    }
    panic!("no helical arc (G2/G3 with Z and I/J on one block) in:\n{gcode}");
}

#[test]
fn grbl_emits_helical_arc() {
    let gcode = GrblPost
        .post(&helix_program(), &machine(), &PostOptions::default())
        .expect("post should succeed");
    assert_has_helical_arc(&gcode);
}

#[test]
fn fanuc_emits_helical_arc() {
    let gcode = FanucPost
        .post(&helix_program(), &machine(), &PostOptions::default())
        .expect("post should succeed");
    assert_has_helical_arc(&gcode);
}
