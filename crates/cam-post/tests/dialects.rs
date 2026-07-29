//! The post picker: the seven dialects fall into four output families for basic
//! milling — grbl-family (expanded, unwrapped, `P` dwell), LinuxCNC (canned,
//! unwrapped, `P` dwell), Fanuc-family (canned, %-wrapped, `X` dwell), and Okuma
//! (canned, unwrapped, `F` dwell, `G71`/`M53` drilling frame, `M02` end — its own
//! emitter, not a Fanuc parameterisation). Proven on a peck-drilling job where the
//! canned-vs-expanded split is visible, and — for Okuma vs LinuxCNC, which share the
//! (unwrapped, canned) pair — on the standalone-dwell word.

use cam_cldata::{DrillCycle, MoveKind, Point3, Program, ProgramBuilder, SpindleDir, Tag};
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

/// Post `program` through grbl with an explicit program name (what puts the header
/// comment in play).
fn post_with_name(program: &Program, name: &str) -> String {
    let options = PostOptions {
        program_name: Some(name.to_string()),
        ..Default::default()
    };
    PostKind::Grbl
        .post(program, &machine(), &options)
        .expect("posts")
}

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

fn nc(kind: PostKind) -> String {
    kind.post(
        &drill_program(),
        &machine(),
        &PostOptions {
            program_name: Some("drill".into()),
            ..Default::default()
        },
    )
    .unwrap()
}

#[test]
fn grbl_family_shares_expanded_unwrapped_output() {
    let g = nc(PostKind::Grbl);
    assert_eq!(nc(PostKind::FluidNc), g, "FluidNC matches the grbl core");
    assert_eq!(nc(PostKind::GrblHal), g, "grblHAL matches the grbl core");
    // No canned cycle (drilling is expanded into explicit moves) and no wrapping.
    assert!(
        !g.contains("G83") && !g.contains("G81") && !g.contains("G98"),
        "grbl expands drilling, no canned cycle:\n{g}"
    );
    assert!(!g.contains('%'), "grbl output is not %-wrapped");
}

#[test]
fn linuxcnc_is_canned_but_unwrapped() {
    let l = nc(PostKind::LinuxCnc);
    assert!(l.contains("G83"), "LinuxCNC uses a canned peck cycle:\n{l}");
    assert!(l.contains("G80"), "and cancels it");
    assert!(!l.contains('%'), "LinuxCNC is not %-wrapped");
    assert!(l.contains("G40 G49"), "RS-274NGC safe-start cancels comp/length");
}

#[test]
fn fanuc_family_wraps_and_uses_canned_cycles() {
    let f = nc(PostKind::Fanuc);
    let h = nc(PostKind::Haas);
    assert!(f.starts_with('%'), "Fanuc is %-wrapped:\n{f}");
    assert!(f.contains("O1000"), "with an O-number");
    assert!(f.contains("G83") && f.contains("G80"), "canned peck cycle");
    assert_eq!(f, h, "Haas shares the Fanuc-family output today");
}

#[test]
fn okuma_is_its_own_family() {
    let o = nc(PostKind::Okuma);
    // Canned, but through the OSP return-level frame, not the Fanuc G98/G99 one.
    assert!(o.contains("G83") && o.contains("G80"), "canned peck cycle:\n{o}");
    assert!(o.contains("G71 Z"), "OSP return-level frame declares the retract:\n{o}");
    assert!(o.contains("M53"), "and the drill cycle carries the M53 return:\n{o}");
    // Not a Fanuc parameterisation: unwrapped, M02 end (not M30), no O-number.
    assert!(!o.contains('%'), "Okuma is not %-wrapped:\n{o}");
    assert!(!o.contains("M30"), "Okuma ends with M02, not M30:\n{o}");
    assert!(o.trim_end().ends_with("M02"), "M02 program end:\n{o}");
}

/// A reference program that exercises **every** axis on which the dialects diverge
/// in one job: a standalone `G4` dwell (the `P`-vs-`X`-vs-`F` word), and a peck drill
/// (canned-vs-expanded, and the `%`/`O`-number wrap). Posting it through all seven
/// controllers gives a signature — (wrapped?, canned?, dwell word) — that uniquely
/// identifies each family, so the operator can eyeball an exported `.nc`/`.MIN`
/// against it.
fn reference_program() -> Program {
    ProgramBuilder::new()
        .comment("dialect reference")
        .tool_change(2)
        .spindle_on(1200.0, SpindleDir::Cw)
        .dwell(0.5) // standalone G4 -> the P/X divergence
        .drill(DrillCycle {
            points: vec![[20.0, 30.0], [40.0, 30.0]],
            z_top: 0.0,
            depth: -8.0,
            retract: 2.0,
            peck: Some(3.0),
            dwell: Some(0.4),
            feed: 120.0,
            tag: Tag::new(0, MoveKind::Plunge),
        })
        .spindle_off()
        .build()
}

const ALL_POSTS: [PostKind; 7] = [
    PostKind::Grbl,
    PostKind::FluidNc,
    PostKind::GrblHal,
    PostKind::LinuxCnc,
    PostKind::Fanuc,
    PostKind::Haas,
    PostKind::Okuma,
];

/// The word carrying the *standalone* `G4` dwell — the first `G4 ` line (the drill
/// cycle's own dwell comes later). `P` on grbl/LinuxCNC, `X` on Fanuc/Haas, `F` on Okuma.
fn standalone_dwell_word(nc: &str) -> Option<char> {
    nc.lines()
        .find(|l| l.starts_with("G4 "))
        .and_then(|l| l.chars().nth(3))
}

#[test]
fn each_dialect_has_the_signature_of_its_family() {
    // (dialect, %-wrapped, canned drilling, standalone dwell word)
    let expected = [
        (PostKind::Grbl, false, false, 'P'),
        (PostKind::FluidNc, false, false, 'P'),
        (PostKind::GrblHal, false, false, 'P'),
        (PostKind::LinuxCnc, false, true, 'P'),
        (PostKind::Fanuc, true, true, 'X'),
        (PostKind::Haas, true, true, 'X'),
        (PostKind::Okuma, false, true, 'F'),
    ];
    for (kind, wrapped, canned, dwell) in expected {
        let g = kind
            .post(
                &reference_program(),
                &machine(),
                &PostOptions {
                    program_name: Some("reference".into()),
                    ..Default::default()
                },
            )
            .unwrap();
        assert_eq!(g.starts_with('%'), wrapped, "{kind}: wrap\n{g}");
        assert_eq!(g.contains("G83"), canned, "{kind}: canned drilling\n{g}");
        assert_eq!(
            standalone_dwell_word(&g),
            Some(dwell),
            "{kind}: standalone dwell word\n{g}"
        );
    }
}

/// A human-readable reference dump. Not an assertion — run it to print every
/// dialect's output side by side:
///   `cargo test -p cam-post reference_dump -- --nocapture`
#[test]
fn reference_dump() {
    println!("\n=== post-dialect reference (one program, six controllers) ===");
    for kind in ALL_POSTS {
        let g = kind
            .post(
                &reference_program(),
                &machine(),
                &PostOptions {
                    program_name: Some("reference".into()),
                    ..Default::default()
                },
            )
            .unwrap();
        let sig = format!(
            "wrapped={:<5} canned={:<5} dwell-word={}",
            g.starts_with('%'),
            g.contains("G83"),
            standalone_dwell_word(&g).unwrap_or('-'),
        );
        println!("\n----- {kind}  [{sig}] -----\n{g}");
    }
}

/// The setup name usually derives from the same source file as the program name, so
/// it would repeat the header verbatim and say nothing. It is emitted only when it
/// differs — and only the *first* comment is eligible, so a later match still prints.
#[test]
fn a_setup_comment_matching_the_program_name_is_not_repeated() {
    use cam_cldata::{Program, Step};
    let mut program = Program::new();
    program.push(Step::Comment("part.dxf".into()));
    program.push(Step::Comment("second op".into()));
    program.push(Step::Comment("part.dxf".into()));

    let nc = post_with_name(&program, "part.dxf");
    let header_hits = nc.lines().filter(|l| l.trim() == "(part.dxf)").count();
    assert_eq!(
        header_hits, 2,
        "one header + the later (non-first) match, not three:\n{nc}"
    );
    assert!(nc.contains("(second op)"));
}

#[test]
fn a_setup_comment_that_differs_is_kept() {
    use cam_cldata::{Program, Step};
    let mut program = Program::new();
    program.push(Step::Comment("Setup 1".into()));
    let nc = post_with_name(&program, "part.dxf");
    assert!(nc.contains("(part.dxf)"), "{nc}");
    assert!(nc.contains("(Setup 1)"), "{nc}");
}
