//! Modal state across a tool change, on **all seven posts**.
//!
//! A tool change is not a plain line in the middle of a program: `M6` hands control to
//! the machine's own change logic, which may move the axes and — on some controls —
//! reset the interpolation group. A move emitted after it that omits its motion word is
//! therefore relying on a property of the *control*, not of the program. Where the
//! assumption fails the bare `X Y` is read under whatever mode survived, and a rapid
//! becomes a feed move into the part (or the reverse).
//!
//! Okuma has emitted `G00` after every change since `OKUMA_PLAN` Finding 1, because its
//! 20 shop programs all do. The six ISO posts share `dialect::emit` and did not; the
//! rule is now the walker's, and this file pins it for every post so neither emitter can
//! drift back.
//!
//! What is *not* asserted: that the position is re-stated. A tool change does not move
//! the work coordinate frame, so the tracked coordinates remain true and re-stating them
//! would be noise — the opposite of a datum change (see `datum.rs`).

use cam_cldata::{MoveKind, Point3, Program, ProgramBuilder, SpindleDir};
use cam_model::{Envelope, Machine};
use cam_post::{PostKind, PostOptions};

/// Every post, both emitters — the six that share the dialect walker and Okuma, which
/// does not.
const ALL_POSTS: [PostKind; 7] = [
    PostKind::Grbl,
    PostKind::FluidNc,
    PostKind::GrblHal,
    PostKind::LinuxCnc,
    PostKind::Fanuc,
    PostKind::Haas,
    PostKind::Okuma,
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

/// Two tools, shaped like the real thing: cut, **retract**, change tool, rapid to the
/// next feature, cut again.
///
/// The retract is what makes this a test rather than a formality. It leaves the writer
/// in `G0` and at Z5, so the second tool's opening rapid is `G0` at Z5 *again* — every
/// word is a candidate for modal suppression, and anything the tool change fails to
/// reset shows up as a missing word. An earlier version of this file cut straight into
/// the change, so the mode flipped `G1`→`G0` on its own and the motion word was emitted
/// no matter what the walker did: the test passed against the unfixed post. This is the
/// shape `carve.nc` actually posts, and the shape that caught the bug.
fn two_tool_program() -> Program {
    ProgramBuilder::new()
        .tool_change(1)
        .spindle_on(1200.0, SpindleDir::Cw)
        .feed(300.0)
        .rapid(Point3::new(10.0, 10.0, 5.0), MoveKind::Link)
        .linear(Point3::new(10.0, 20.0, -3.0), MoveKind::Cutting)
        .rapid(Point3::new(10.0, 20.0, 5.0), MoveKind::Link)
        .tool_change(2)
        // Same Z as the retract — so a Z word here would prove the position was reset.
        .rapid(Point3::new(40.0, 60.0, 5.0), MoveKind::Link)
        .linear(Point3::new(40.0, 60.0, -2.0), MoveKind::Cutting)
        .spindle_off()
        .build()
}

fn post(kind: PostKind, program: &Program) -> String {
    kind.post(program, &machine(), &PostOptions::default())
        .expect("posts")
}

/// The lines after the *last* `M6`, in order, with blank lines and comments dropped.
fn after_last_tool_change(nc: &str) -> Vec<&str> {
    let lines: Vec<&str> = nc.lines().collect();
    let at = lines
        .iter()
        .rposition(|l| l.split_whitespace().any(|w| w == "M6"))
        .expect("the program has a tool change");
    lines[at + 1..]
        .iter()
        .copied()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('('))
        .collect()
}

/// Whether a line is axis motion — carries an axis word and is not a canned cycle or a
/// setup word. Deliberately narrow: it is the first *move* we care about.
fn is_motion(line: &str) -> bool {
    line.split_whitespace()
        .any(|w| matches!(w.as_bytes().first(), Some(b'X' | b'Y' | b'Z')))
}

/// The motion word (`G0`/`G00`/`G1`/…) a line states, if any.
fn motion_word(line: &str) -> Option<&str> {
    line.split_whitespace().find(|w| {
        w.strip_prefix('G')
            .is_some_and(|rest| !rest.is_empty() && rest.bytes().all(|b| b.is_ascii_digit()))
    })
}

#[test]
fn the_first_move_after_a_tool_change_states_its_motion_word() {
    // The invariant, on every post. Without it the second tool's opening move is a bare
    // `X10.000 Y10.000` inheriting whatever mode survived `M6`.
    for kind in ALL_POSTS {
        let nc = post(kind, &two_tool_program());
        let first_move = after_last_tool_change(&nc)
            .into_iter()
            .find(|l| is_motion(l))
            .unwrap_or_else(|| panic!("{kind:?}: nothing moves after the tool change:\n{nc}"));
        let g = motion_word(first_move).unwrap_or_else(|| {
            panic!("{kind:?}: the first move after `M6` has no motion word: {first_move:?}\n{nc}")
        });
        assert!(
            g == "G0" || g == "G00",
            "{kind:?}: the move after `M6` is a planner rapid but posts as {g}: \
             {first_move:?}\n{nc}"
        );
    }
}

#[test]
fn the_first_feed_move_after_a_tool_change_restates_its_feed() {
    // The other half of the modal reset, and the one with teeth: the feed *rate* in force
    // is the new tool's business. A second tool inheriting the first tool's F is the
    // classic way a small cutter gets fed at a big cutter's rate.
    for kind in ALL_POSTS {
        let nc = post(kind, &two_tool_program());
        let feed_move = after_last_tool_change(&nc)
            .into_iter()
            .find(|l| is_motion(l) && matches!(motion_word(l), Some("G1" | "G01")))
            .unwrap_or_else(|| panic!("{kind:?}: no feed move after the tool change:\n{nc}"));
        assert!(
            feed_move.split_whitespace().any(|w| w.starts_with('F')),
            "{kind:?}: the first feed move after `M6` inherits a stale feed: \
             {feed_move:?}\n{nc}"
        );
    }
}

#[test]
fn a_tool_change_does_not_re_state_the_position() {
    // The deliberate non-reset, stated directly. The program retracts to Z5, changes
    // tool, and rapids to a new XY at *the same Z5*. If the change reset the tracked
    // position the walker would re-state Z; because it does not, Z is suppressed and the
    // line carries X and Y alone.
    //
    // Worth pinning rather than leaving implicit: `reset_modal` and `reset_position` sit
    // next to each other on the writer and a datum change calls both, so "reset
    // everything at a tool change" is the easy wrong edit — and it would be invisible in
    // the output except as noise.
    for kind in ALL_POSTS {
        let nc = post(kind, &two_tool_program());
        let first_move = after_last_tool_change(&nc)
            .into_iter()
            .find(|l| is_motion(l))
            .unwrap_or_else(|| panic!("{kind:?}: nothing moves after the tool change:\n{nc}"));
        assert!(
            !first_move.split_whitespace().any(|w| w.starts_with('Z')),
            "{kind:?}: Z is re-stated though the retract already left the tool there — \
             the tool change reset the tracked position, which only a datum change should \
             do: {first_move:?}\n{nc}"
        );
    }
}
