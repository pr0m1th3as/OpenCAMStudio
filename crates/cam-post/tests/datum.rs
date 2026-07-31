//! Per-operation work datums on the ISO families — the six posts that share
//! `dialect::emit` (grbl/FluidNC/grblHAL, LinuxCNC, Fanuc/Haas). Datum `n` selects the
//! (n-1)-th work coordinate system after the program's base, so datum 1 is the base
//! the preamble already stated and a single-datum program is unchanged.
//!
//! Okuma has its own emitter and its own `G15 H<n>` number space; it is covered in
//! `okuma.rs`. What is pinned here is the ISO word (`G54`-`G59`), the arithmetic from
//! a raised base, the frame re-statement after a change, and the refusals — both the
//! ceiling at `G59` and a datum index of 0. The refusal path in particular went
//! untested for the whole life of the earlier Okuma-only guard.

use cam_cldata::{MoveKind, Point3, Program, ProgramBuilder, SpindleDir};
use cam_model::{Envelope, Machine};
use cam_post::{PostError, PostKind, PostOptions, WorkOffset};

/// Every post that goes through the shared dialect walker.
const ISO_POSTS: [PostKind; 6] = [
    PostKind::Grbl,
    PostKind::FluidNc,
    PostKind::GrblHal,
    PostKind::LinuxCnc,
    PostKind::Fanuc,
    PostKind::Haas,
];

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

/// A two-fixture job: cut on the base datum, change datum, cut again. The second cut
/// repeats the *same* coordinates — under `translated_per_datum` each group is
/// re-referenced to its own origin, so identical part features post identically and
/// only the work offset distinguishes them.
fn two_datum_program(second: u32) -> Program {
    ProgramBuilder::new()
        .tool_change(1)
        .spindle_on(1200.0, SpindleDir::Cw)
        .feed(300.0)
        .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
        .linear(Point3::new(10.0, 20.0, -3.0), MoveKind::Cutting)
        .datum(second)
        .tool_change(1)
        .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
        .linear(Point3::new(10.0, 20.0, -3.0), MoveKind::Cutting)
        .spindle_off()
        .build()
}

fn post_base(kind: PostKind, program: &Program, base: WorkOffset) -> Result<String, PostError> {
    kind.post(
        program,
        &machine(),
        &PostOptions {
            work_offset: base,
            ..Default::default()
        },
    )
}

fn post(kind: PostKind, program: &Program) -> String {
    post_base(kind, program, WorkOffset::G54).expect("posts")
}

/// The work-offset words the program states, in order.
fn offsets(nc: &str) -> Vec<&str> {
    nc.lines()
        .flat_map(|l| l.split_whitespace())
        .filter(|w| WorkOffset::ALL.iter().any(|o| o.code() == *w))
        .collect()
}

#[test]
fn every_iso_post_states_the_second_datum_as_g55() {
    // The whole point of the change: six posts that used to refuse a multi-fixture job
    // now emit it. Checked on all of them, since the walker is shared but the picker
    // is what an operator actually chooses.
    for kind in ISO_POSTS {
        let nc = post(kind, &two_datum_program(2));
        assert_eq!(
            offsets(&nc),
            vec!["G54", "G55"],
            "{kind:?} states the base then the second datum:\n{nc}"
        );
    }
}

#[test]
fn datum_one_is_the_base_and_is_not_restated() {
    // An explicit `Datum(1)` is the offset already in force. Restating it would be
    // harmless on the control but would move every existing golden, and it says
    // nothing — so nothing is emitted.
    for kind in ISO_POSTS {
        let nc = post(kind, &two_datum_program(1));
        assert_eq!(
            offsets(&nc),
            vec!["G54"],
            "{kind:?} states the base once and does not restate it:\n{nc}"
        );
    }
}

#[test]
fn datums_count_up_from_the_programs_base() {
    // Datum indices are relative to whatever base the operator chose, so a program
    // based at G56 puts its second fixture on G57 — not on G55, which would contradict
    // the preamble the very first time the datum changed.
    let nc = post_base(PostKind::Fanuc, &two_datum_program(2), WorkOffset::G56).expect("posts");
    assert_eq!(offsets(&nc), vec!["G56", "G57"], "{nc}");
}

#[test]
fn all_six_offsets_are_reachable_from_the_default_base() {
    // Datum 6 is G59 — the last word the ISO families carry.
    let nc = post(PostKind::LinuxCnc, &two_datum_program(6));
    assert_eq!(offsets(&nc), vec!["G54", "G59"], "{nc}");
}

#[test]
fn a_datum_past_g59_is_refused_rather_than_wrapped() {
    // There is no seventh work offset in this word space. Silently clamping to G59
    // would run two fixtures on one datum and miscut the part, and `G54.1 P` is a
    // Fanuc option that need not be fitted — so refuse, and say why.
    let err = post_base(PostKind::Fanuc, &two_datum_program(7), WorkOffset::G54)
        .expect_err("a seventh datum has no word");
    let PostError::Unsupported(msg) = err else {
        panic!("expected Unsupported, got {err:?}");
    };
    assert!(msg.contains("G54-G59"), "the message names the word space: {msg}");
    assert!(msg.contains("Fanuc"), "and the control it applies to: {msg}");
}

#[test]
fn a_raised_base_lowers_the_ceiling() {
    // Six offsets exist in total, not six *beyond* the base: from G58 only datums 1
    // and 2 are reachable. This is the failure a user meets by changing the post's
    // work offset after building a multi-fixture job, so it must not go unnoticed.
    assert!(
        post_base(PostKind::Haas, &two_datum_program(2), WorkOffset::G58).is_ok(),
        "G58 + 1 = G59 is fine"
    );
    let err = post_base(PostKind::Haas, &two_datum_program(3), WorkOffset::G58)
        .expect_err("G58 + 2 runs past G59");
    let PostError::Unsupported(msg) = err else {
        panic!("expected Unsupported, got {err:?}");
    };
    assert!(msg.contains("G58"), "the message names the base in force: {msg}");
}

#[test]
fn datum_zero_is_refused() {
    // Datum indices are 1-based (origin 1 is the base). A 0 is an upstream bug, and
    // folding it onto the base would hide it inside a file that still cuts.
    let err = post_base(PostKind::Grbl, &two_datum_program(0), WorkOffset::G54)
        .expect_err("datum 0 does not exist");
    let PostError::Unsupported(msg) = err else {
        panic!("expected Unsupported, got {err:?}");
    };
    assert!(msg.contains("datum 0"), "the message names the bad index: {msg}");
}

#[test]
fn the_first_move_after_a_datum_change_restates_x_y_and_z() {
    // A work-offset change shifts the whole coordinate frame, so a move that would
    // otherwise be modally abbreviated ("same Z, omit it") must re-state every axis —
    // the old Z is a different height in the new datum. Both cuts here are at the same
    // numbers precisely so that a stale modal state would show up as a missing word.
    for kind in ISO_POSTS {
        let nc = post(kind, &two_datum_program(2));
        let after = nc
            .lines()
            .skip_while(|l| !l.split_whitespace().any(|w| w == "G55"))
            .find(|l| l.contains('X') && l.contains('Y'))
            .unwrap_or_else(|| panic!("a positioning move after the datum change:\n{nc}"));
        assert!(
            after.contains('X') && after.contains('Y') && after.contains('Z'),
            "{kind:?} re-states X, Y and Z in the new frame, got `{after}`:\n{nc}"
        );
        assert!(
            after.starts_with("G0") || after.starts_with("G1"),
            "{kind:?} re-states the motion word too, got `{after}`"
        );
    }
}

#[test]
fn the_label_is_the_word_the_post_actually_emits() {
    // `datum_label` exists so the GUI can name an origin's datum without re-deriving
    // it. That is only worth anything if the two agree, so: for every post and every
    // reachable datum, the label must be exactly the word that turns up in the file.
    // A second `match` in the UI would pass a test written against itself; this one is
    // written against the emitter.
    for kind in ISO_POSTS {
        for base in WorkOffset::ALL {
            for datum in 1..=WorkOffset::ALL.len() as u32 {
                let label = kind.datum_label(datum, base);
                let posted = post_base(kind, &two_datum_program(datum), base);
                match (label, posted) {
                    (Some(l), Ok(nc)) => assert_eq!(
                        *offsets(&nc).last().expect("an offset word"),
                        l,
                        "{kind:?} datum {datum} from {}: label vs file:\n{nc}",
                        base.code()
                    ),
                    // Unreachable datum: the label says so *and* the post refuses. If
                    // these ever part company the UI promises a file that will not post.
                    (None, Err(_)) => {}
                    (l, p) => panic!(
                        "{kind:?} datum {datum} from {}: label {l:?} disagrees with the \
                         post ({})",
                        base.code(),
                        if p.is_ok() { "which posted" } else { "which refused" }
                    ),
                }
            }
        }
    }
}

#[test]
fn okuma_labels_its_own_number_space_and_ignores_the_base() {
    // The reason the mapping cannot be one rule: OSP takes the datum index literally,
    // so the base a program is written against does not enter into it.
    for base in WorkOffset::ALL {
        assert_eq!(PostKind::Okuma.datum_label(2, base).as_deref(), Some("H2"));
        // And it has no six-offset ceiling — a seventh fixture is `H7`, which the
        // Okuma emitter does state.
        assert_eq!(PostKind::Okuma.datum_label(7, base).as_deref(), Some("H7"));
    }
    let nc = PostKind::Okuma
        .post(&two_datum_program(7), &machine(), &PostOptions::default())
        .expect("OSP states any index");
    assert!(nc.contains("G15 H7"), "the label's word is in the file:\n{nc}");
}

#[test]
fn datum_zero_has_no_label_on_any_post() {
    for kind in PostKind::ALL {
        assert_eq!(kind.datum_label(0, WorkOffset::G54), None, "{kind:?}");
    }
}

#[test]
fn a_single_datum_program_states_its_offset_exactly_once() {
    // The regression that matters most: nothing about the single-fixture output moved.
    let single = ProgramBuilder::new()
        .tool_change(1)
        .spindle_on(1200.0, SpindleDir::Cw)
        .feed(300.0)
        .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
        .linear(Point3::new(10.0, 20.0, -3.0), MoveKind::Cutting)
        .spindle_off()
        .build();
    for kind in ISO_POSTS {
        assert_eq!(offsets(&post(kind, &single)), vec!["G54"], "{kind:?}");
    }
}
