//! The Okuma OSP post — a fourth output family.
//!
//! Okuma OSP is **not** a Fanuc parameterisation, and treating it as one is a
//! machine-safety hazard: `G54`–`G59` are per-axis *tool-length* offsets (not work
//! offsets), work coordinates are selected with `G15 H<n>`, tool length with
//! `G56 H<n>`, the program ends with `M02` (not `M30`), and drilling returns to a
//! level declared by the `G71`/`M53` pair. Emitting those through the shared,
//! Fanuc-shaped [`crate::dialect::emit`] walker would produce codes that are
//! silently valid on an Okuma and mean something else entirely — so this family has
//! its own emitter. It reuses the common machinery ([`Writer`], number/word
//! formatting, travel checking); only the frame diverges. See `OKUMA_PLAN.md`.
//!
//! **Status (O1 + O2 + O3).** The frame skeleton (defensive safe-start, per-tool-
//! section `G15`/`G56`, milling motion, standalone dwell), the `G71`/`M53` drilling
//! cycles (`G81`/`G82`/`G83`, `G80`, with the `G80` auto-`M05` compensated), and
//! per-operation multi-WCS (`G15 H<n>` driven by a work-datum index in CL-data —
//! [`Step::Datum`]) are in. Datum 1 is the default, so a single-datum job emits
//! `G15 H1` per section exactly as before.

use cam_cldata::{Coolant, CutterComp, DrillCycle, Point3, Program, SpindleDir, Step};
use cam_model::Machine;

use crate::words::{compact, num, sanitize};
use crate::writer::Writer;
use crate::{Capabilities, Post, PostError, PostOptions};

/// The Okuma safe-start block — deliberately defensive (OKUMA_PLAN §6b, decided
/// 2026-07-28). OSP takes units / plane / absolute / feed-mode from power-on
/// parameters, so none of the shop sample files state them; we assert them anyway so
/// the program cannot be wrecked by how the machine was left. `G21` is a unit-system
/// *check* — it alarms rather than silently cutting a metric program in inch — and
/// `G90` guards a machine left in `G91`. `G40`/`G49`/`G80` are deliberately absent:
/// `G49` is not a tool-length cancel on Okuma, and comp/cycle state is handled at its
/// own sites.
const SAFE_START: &str = "G21 G17 G90 G94";

/// The Okuma post. Stateless; construct with [`OkumaPost`].
#[derive(Clone, Copy, Debug, Default)]
pub struct OkumaPost;

impl Post for OkumaPost {
    fn name(&self) -> &str {
        "okuma"
    }

    fn capabilities(&self) -> Capabilities {
        // The *control's* capabilities. OSP has canned drilling (G81/G82/G83/G84 with
        // the G71/M53 return frame), emitted natively (see `canned_drill`).
        Capabilities {
            arcs: true,
            canned_drill: true,
            cutter_comp: true,
            work_offsets: true,
            coolant: true,
            tool_change: true,
        }
    }

    fn post(
        &self,
        program: &Program,
        machine: &Machine,
        options: &PostOptions,
    ) -> Result<String, PostError> {
        emit(program, machine, options)
    }
}

/// Emit `program` as Okuma OSP G-code.
pub(crate) fn emit(
    program: &Program,
    machine: &Machine,
    options: &PostOptions,
) -> Result<String, PostError> {
    crate::check_travel(program, machine)?;
    let mut w = Writer::new(machine, options.precision);

    // No `%`/O-number wrapper: on OSP the file name *is* the program name. Emit the
    // job name as a plain comment when we have one, then the defensive safe-start.
    if let Some(name) = &options.program_name {
        w.line(format!("({})", sanitize(name)));
    }
    w.line(SAFE_START.to_string());

    // First-comment dedupe, mirroring the shared walker: the setup name usually
    // derives from the same source as the program name and would repeat the header
    // verbatim. Only the *first* comment is eligible, so a later match still prints.
    let header_name = options.program_name.as_deref().map(sanitize);
    let mut header_dedupe = header_name.is_some();

    // The spindle running at the current point, tracked so a drilling cycle can undo
    // OSP's `G80` auto-`M05` when cutting continues (see the `Drill` arm).
    let mut spindle: Option<(f64, SpindleDir)> = None;

    // The work datum in force, restated as `G15 H<n>` in every tool-section head.
    // Set by `Step::Datum`; defaults to 1, so a program with no datum steps emits
    // `G15 H1` per section exactly as before multi-WCS existed.
    let mut current_datum: u32 = 1;

    let steps = program.steps();
    for (i, step) in steps.iter().enumerate() {
        match step {
            Step::Comment(text) => {
                let text = sanitize(text);
                if header_dedupe {
                    header_dedupe = false;
                    if header_name.as_deref() == Some(text.as_str()) {
                        continue;
                    }
                }
                w.line(format!("({text})"));
            }
            Step::Datum(n) => {
                // Work coordinate select. In the ordinary job a datum change coincides
                // with a tool change — each fixture restarts the tool sequence (see the
                // shop `PL-0-3T.MIN`: five tools per datum, H1→H2→H3) — so we defer to
                // the tool-section head, where `G15 H<n>` sits between `T M6` and
                // `G56 H` in shop order. A datum change with *no* following tool change
                // (same tool, a different fixture) has no section head to ride, so it
                // states `G15 H<n>` on its own line.
                current_datum = *n;
                if !next_section_is_tool_change(&steps[i + 1..]) {
                    w.line(format!("G15 H{n}"));
                }
            }
            Step::ToolChange { tool } => {
                // Tool-section head: change tool, select the work coordinate system,
                // then the tool-length offset keyed to the tool number. `G15 H` and
                // `G56 H` are unrelated number spaces (OKUMA_PLAN §3): `G15 H<n>` is the
                // work datum in force (restated every section as a modal, matching the
                // shop files), `G56 H<tool>` the tool-length offset keyed to the tool.
                w.line(format!("T{tool} M6"));
                w.line(format!("G15 H{current_datum}"));
                w.line(format!("G56 H{tool}"));
                // Force the next move to re-state its motion word. The shop files
                // always re-emit `G00` after a tool change rather than rely on a modal
                // `G0` carrying across `M6`, and OSP may reset the interpolation group
                // on a tool change — a bare `X Y Z` would then be an unintended feed.
                w.reset_modal();
            }
            Step::Spindle { rpm, dir } => {
                w.check_spindle(*rpm)?;
                let m = match dir {
                    SpindleDir::Cw => "M3",
                    SpindleDir::Ccw => "M4",
                };
                w.line(format!("{m} S{}", compact(*rpm, 0)));
                spindle = Some((*rpm, *dir));
            }
            Step::SpindleOff => {
                w.line("M5".to_string());
                spindle = None;
            }
            Step::Coolant(c) => w.line(
                match c {
                    Coolant::Flood => "M8",
                    Coolant::Mist => "M7",
                    Coolant::Off => "M9",
                }
                .to_string(),
            ),
            Step::CutterComp(c) => w.line(match c {
                CutterComp::Off => "G40".to_string(),
                CutterComp::Left(d) => format!("G41 D{d}"),
                CutterComp::Right(d) => format!("G42 D{d}"),
            }),
            Step::Dwell { seconds } => {
                // Standalone dwell is `G04 F<sec>` on OSP (OKUMA_PLAN §6b). Emit an
                // explicit decimal so the value reads as seconds regardless of the
                // machine's dwell-unit parameter (same argument as §4 formatting).
                w.line(format!("G4 F{}", compact(*seconds, 3)));
            }
            Step::Rapid { to, .. } => w.rapid(*to)?,
            Step::Linear { to, feed, .. } => w.feed_move(*to, *feed)?,
            Step::Arc {
                end,
                center,
                dir,
                feed,
                ..
            } => w.arc(*end, *center, *dir, *feed)?,
            Step::Drill(cycle) => {
                canned_drill(&mut w, cycle)?;
                // OSP's `G80` auto-generates `M05` (OKUMA_PLAN §6b). When the CL-data
                // keeps cutting in this section before restating the spindle, that
                // auto-stop would silently kill it — so re-assert the running spindle.
                // In the common case (drill → retract → spindle-off / tool change) the
                // auto-stop is harmless and no re-assertion is emitted.
                if let Some((rpm, dir)) = spindle {
                    if cuts_before_spindle_change(&steps[i + 1..]) {
                        let m = match dir {
                            SpindleDir::Cw => "M3",
                            SpindleDir::Ccw => "M4",
                        };
                        w.line(format!("{m} S{}", compact(rpm, 0)));
                    }
                }
            }
        }
    }

    // End of program is `M02` on Okuma, never `M30`, and there is no `%` to close.
    w.line("M02".to_string());
    Ok(w.finish())
}

/// Emit a drilling cycle in the Okuma OSP frame: a `G71 Z<level>` return-level
/// declaration, one cycle-defining line carrying the trailing `M53` (which selects
/// that level), one bare-coordinate line per further hole, then `G80`.
///
/// The cycle code follows the family: peck → `G83` (`Q` peck, optional `P` dwell),
/// dwell-without-peck → `G82` (`P`), plain → `G81`. `Z` is the hole bottom, `R` the
/// point-R level; both come straight from the neutral [`DrillCycle`]. The return
/// level is the cycle's own `retract` — the "clearance plane between holes" by its
/// definition — which also matches what the Fanuc post returns to via `G98`. `G71`
/// is re-emitted for every cycle because its level is undefined after an NC reset and
/// must never be assumed to carry (OKUMA_PLAN §6b).
fn canned_drill(w: &mut Writer, c: &DrillCycle) -> Result<(), PostError> {
    w.check_feed(c.feed)?;
    if c.points.is_empty() {
        return Ok(());
    }

    let p = w.precision();
    let (x0, y0) = (c.points[0][0], c.points[0][1]);

    // Position over the first hole at the clearance plane, then declare the M53 level.
    w.rapid(Point3::new(x0, y0, c.retract))?;
    w.line(format!("G71 Z{}", num(c.retract, p)));

    let z = num(c.depth, p);
    let r = num(c.retract, p);
    let f = compact(c.feed, p);
    // `M53` rides the cycle-defining line only, never the bare-coordinate repeats.
    let cycle = match (c.peck, c.dwell) {
        (Some(q), Some(d)) => format!(
            "G83 X{} Y{} Z{z} R{r} Q{} P{} F{f} M53",
            num(x0, p),
            num(y0, p),
            compact(q, p),
            compact(d, 3),
        ),
        (Some(q), None) => format!(
            "G83 X{} Y{} Z{z} R{r} Q{} F{f} M53",
            num(x0, p),
            num(y0, p),
            compact(q, p),
        ),
        (None, Some(d)) => format!(
            "G82 X{} Y{} Z{z} R{r} P{} F{f} M53",
            num(x0, p),
            num(y0, p),
            compact(d, 3),
        ),
        (None, None) => format!("G81 X{} Y{} Z{z} R{r} F{f} M53", num(x0, p), num(y0, p)),
    };
    w.line(cycle);

    for &[x, y] in &c.points[1..] {
        w.line(format!("X{} Y{}", num(x, p), num(y, p)));
    }
    w.line("G80".to_string());

    let last = *c.points.last().unwrap();
    w.set_position(Point3::new(last[0], last[1], c.retract));
    w.reset_modal();
    Ok(())
}

/// Whether the next tool-section head follows immediately — i.e. the next
/// significant step is a [`Step::ToolChange`], with only comments in between. Used
/// by the `Datum` arm to decide whether to defer its `G15 H<n>` to that head (shop
/// order `T M6` / `G15` / `G56`) or state it standalone.
fn next_section_is_tool_change(rest: &[Step]) -> bool {
    for s in rest {
        match s {
            Step::Comment(_) => continue,
            Step::ToolChange { .. } => return true,
            _ => return false,
        }
    }
    false
}

/// Whether a cutting move — one that needs the spindle running — occurs before the
/// spindle is next restated. Comments, coolant, rapids, dwell and comp are
/// transparent; a `Spindle`, `SpindleOff` or `ToolChange` redefines the section's
/// spindle intent, so a preceding `G80`'s auto-`M05` is no longer ours to undo.
fn cuts_before_spindle_change(rest: &[Step]) -> bool {
    for s in rest {
        match s {
            Step::Linear { .. } | Step::Arc { .. } | Step::Drill(_) => return true,
            Step::Spindle { .. } | Step::SpindleOff | Step::ToolChange { .. } => return false,
            _ => {}
        }
    }
    false
}
