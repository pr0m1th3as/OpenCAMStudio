//! P6 marquee: one neutral `Drill` cycle intent, posted two ways. grbl expands
//! it into explicit peck moves; Fanuc emits a canned `G83`/`G80`. Same CL-data,
//! different capabilities — proven by golden files and a shared semantic check.

use cam_cldata::{DrillCycle, MoveKind, Point3, Program, ProgramBuilder, SpindleDir, Tag};
use cam_model::{Envelope, Machine};
use cam_post::{FanucPost, GrblPost, Post, PostOptions};

fn machine() -> Machine {
    Machine {
        name: "test".into(),
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

/// A two-hole peck-drilling job with a dwell at the bottom.
fn drill_program() -> Program {
    ProgramBuilder::new()
        .comment("drill demo")
        .tool_change(2)
        .spindle_on(1200.0, SpindleDir::Cw)
        .drill(DrillCycle {
            points: vec![[20.0, 30.0], [40.0, 30.0]],
            z_top: 0.0,
            depth: -8.0,
            retract: 2.0,
            peck: Some(3.0),
            dwell: Some(0.5),
            feed: 120.0,
            tag: Tag::new(0, MoveKind::Plunge),
        })
        .spindle_off()
        .build()
}

fn opts() -> PostOptions {
    PostOptions {
        program_name: Some("drill".into()),
        ..Default::default()
    }
}

fn grbl_nc() -> String {
    GrblPost
        .post(&drill_program(), &machine(), &opts())
        .unwrap()
}

fn fanuc_nc() -> String {
    FanucPost
        .post(&drill_program(), &machine(), &opts())
        .unwrap()
}

#[test]
fn dump() {
    println!("=== GRBL ===\n{}\n=== FANUC ===\n{}", grbl_nc(), fanuc_nc());
}

#[test]
fn capabilities_differ_on_canned_cycles() {
    assert!(!GrblPost.capabilities().canned_drill);
    assert!(FanucPost.capabilities().canned_drill);
}

#[test]
fn fanuc_uses_a_canned_cycle_grbl_expands_it() {
    let grbl = grbl_nc();
    let fanuc = fanuc_nc();

    // Fanuc: one G83 + a G80, no explicit peck G1s.
    assert_eq!(fanuc.matches("G83").count(), 1, "one canned cycle\n{fanuc}");
    assert!(fanuc.contains("G80"), "cycle cancelled\n{fanuc}");
    assert!(
        !fanuc.contains("G1 "),
        "no explicit peck moves in Fanuc\n{fanuc}"
    );
    assert!(fanuc.contains('%'), "Fanuc program is %-bracketed");

    // grbl: no canned cycle, many explicit feed moves (3 pecks × 2 holes).
    assert!(!grbl.contains("G83"), "grbl has no canned cycle");
    assert_eq!(grbl.lines().filter(|l| l.starts_with("G1 ")).count(), 6);

    // The canned cycle is far more compact.
    assert!(
        fanuc.lines().count() < grbl.lines().count(),
        "Fanuc ({}) should be shorter than grbl ({})",
        fanuc.lines().count(),
        grbl.lines().count()
    );
}

#[test]
fn golden_grbl() {
    assert_eq!(grbl_nc(), include_str!("golden/drill_grbl.nc"));
}

#[test]
fn golden_fanuc() {
    assert_eq!(fanuc_nc(), include_str!("golden/drill_fanuc.nc"));
}

#[test]
fn both_reach_full_depth_within_the_envelope() {
    // Independent of dialect, every emitted coordinate must be in the envelope
    // and the deepest Z must reach the hole bottom (-8).
    for nc in [grbl_nc(), fanuc_nc()] {
        let m = machine();
        let mut deepest = f64::MAX;
        for line in nc.lines() {
            for tok in line.split_whitespace() {
                if let Some(z) = tok.strip_prefix('Z').and_then(|v| v.parse::<f64>().ok()) {
                    deepest = deepest.min(z);
                    assert!(
                        z >= m.envelope.min.z && z <= m.envelope.max.z,
                        "Z {z} out of envelope in: {line}"
                    );
                }
            }
        }
        assert!(
            (deepest - (-8.0)).abs() < 1e-9,
            "must reach depth -8, got {deepest}"
        );
    }
}
