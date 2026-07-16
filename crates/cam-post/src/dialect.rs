//! Dialect-driven post emission. The controllers we target differ in only a
//! handful of ways — canned vs expanded drilling, program wrapping, the safe-start
//! block, the dwell word, and whether cutter compensation exists — so a single
//! [`emit`] walks the neutral CL-data and a [`Dialect`] supplies the differences.
//! This is also the seam a future data-driven post format slots into.

use cam_cldata::{Coolant, CutterComp, DrillCycle, Point3, Program, SpindleDir, Step};
use cam_model::Machine;

use crate::words::{compact, num, sanitize};
use crate::writer::Writer;
use crate::{PostError, PostOptions};

/// Rapid clearance left above the previous peck depth before feeding again, mm.
const PECK_CLEARANCE: f64 = 0.5;
/// Program number emitted in the `O` word (Fanuc/Haas require one).
const PROGRAM_NUMBER: u32 = 1000;

/// How a drilling cycle is expressed.
#[derive(Clone, Copy)]
pub(crate) enum Drilling {
    /// A single `G81`/`G82`/`G83` canned cycle (Fanuc-family, LinuxCNC).
    Canned,
    /// Explicit peck moves (grbl-family — no canned cycles).
    Expanded,
}

/// The knobs that distinguish one controller dialect from another.
pub(crate) struct Dialect {
    /// Display name (shown in the post picker).
    pub name: &'static str,
    /// Safe-start modal block emitted after the header.
    pub preamble: &'static str,
    /// Wrap the program in `%` and carry an `O`-number (Fanuc/Haas).
    pub wrap: bool,
    /// Canned vs expanded drilling.
    pub drilling: Drilling,
    /// The word carrying dwell *seconds* on a standalone `G4` — `P` on grbl and
    /// LinuxCNC, `X` on Fanuc/Haas.
    pub dwell_word: char,
    /// Whether the control has cutter compensation; if not, `G41`/`G42` is refused
    /// rather than emitted (we default to computed comp, so this is rarely hit).
    pub cutter_comp: bool,
}

// grbl and its descendants: no canned cycles, no cutter comp. For basic milling
// grbl / FluidNC / grblHAL accept the same program; the distinct names declare the
// operator's target and let the dialects diverge as features grow.
pub(crate) static GRBL: Dialect = grbl_like("grbl");
pub(crate) static FLUIDNC: Dialect = grbl_like("FluidNC");
pub(crate) static GRBLHAL: Dialect = grbl_like("grblHAL");

/// LinuxCNC (RS-274NGC): canned cycles and cutter comp, no program wrapping.
pub(crate) static LINUXCNC: Dialect = Dialect {
    name: "LinuxCNC",
    preamble: "G17 G21 G90 G94 G40 G49",
    wrap: false,
    drilling: Drilling::Canned,
    dwell_word: 'P',
    cutter_comp: true,
};

/// Fanuc: `%`-wrapped with an O-number, canned cycles, `G4 X` dwell.
pub(crate) static FANUC: Dialect = fanuc_like("Fanuc");
/// Haas: a Fanuc-family control; the same basic milling program.
pub(crate) static HAAS: Dialect = fanuc_like("Haas");

/// A grbl-family dialect with a given name.
const fn grbl_like(name: &'static str) -> Dialect {
    Dialect {
        name,
        preamble: "G17 G21 G90 G94",
        wrap: false,
        drilling: Drilling::Expanded,
        dwell_word: 'P',
        cutter_comp: false,
    }
}

/// A Fanuc-family dialect with a given name.
const fn fanuc_like(name: &'static str) -> Dialect {
    Dialect {
        name,
        preamble: "G17 G21 G90 G94 G40 G49 G80",
        wrap: true,
        drilling: Drilling::Canned,
        dwell_word: 'X',
        cutter_comp: true,
    }
}

/// Emit the whole program in `dialect`'s flavour.
pub(crate) fn emit(
    program: &Program,
    machine: &Machine,
    options: &PostOptions,
    dialect: &Dialect,
) -> Result<String, PostError> {
    crate::check_travel(program, machine)?;
    let mut w = Writer::new(machine, options.precision);

    // Header: `%` + O-number for Fanuc/Haas, else a plain name comment.
    if dialect.wrap {
        w.line("%".to_string());
        match &options.program_name {
            Some(name) => w.line(format!("O{PROGRAM_NUMBER} ({})", sanitize(name))),
            None => w.line(format!("O{PROGRAM_NUMBER}")),
        }
    } else if let Some(name) = &options.program_name {
        w.line(format!("({})", sanitize(name)));
    }
    w.line(dialect.preamble.to_string());
    w.line(options.work_offset.code().to_string());

    for step in program.steps() {
        match step {
            Step::Comment(text) => w.line(format!("({})", sanitize(text))),
            Step::Spindle { rpm, dir } => {
                w.check_spindle(*rpm)?;
                let m = match dir {
                    SpindleDir::Cw => "M3",
                    SpindleDir::Ccw => "M4",
                };
                w.line(format!("{m} S{}", compact(*rpm, 0)));
            }
            Step::SpindleOff => w.line("M5".to_string()),
            Step::Coolant(c) => w.line(
                match c {
                    Coolant::Flood => "M8",
                    Coolant::Mist => "M7",
                    Coolant::Off => "M9",
                }
                .to_string(),
            ),
            Step::ToolChange { tool } => w.line(format!("T{tool} M6")),
            Step::CutterComp(c) => {
                if dialect.cutter_comp {
                    w.line(match c {
                        CutterComp::Off => "G40".to_string(),
                        CutterComp::Left(d) => format!("G41 D{d}"),
                        CutterComp::Right(d) => format!("G42 D{d}"),
                    });
                } else {
                    // No cutter comp on this control — refuse control comp rather
                    // than silently drop it.
                    match c {
                        CutterComp::Left(_) | CutterComp::Right(_) => {
                            return Err(PostError::Unsupported(
                                "cutter radius compensation (G41/G42)".to_string(),
                            ));
                        }
                        CutterComp::Off => {}
                    }
                }
            }
            Step::Dwell { seconds } => {
                w.line(format!("G4 {}{}", dialect.dwell_word, compact(*seconds, 3)))
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
            Step::Drill(cycle) => match dialect.drilling {
                Drilling::Canned => canned_drill(&mut w, cycle)?,
                Drilling::Expanded => expanded_drill(&mut w, cycle)?,
            },
        }
    }

    w.line("M30".to_string());
    if dialect.wrap {
        w.line("%".to_string());
    }
    Ok(w.finish())
}

/// Expand a drilling cycle into explicit moves (no canned cycle). Peck drilling
/// follows `G83` semantics: full retract after each peck to clear chips, then a
/// rapid back down to just above the last depth.
fn expanded_drill(w: &mut Writer, c: &DrillCycle) -> Result<(), PostError> {
    w.check_feed(c.feed)?;
    for &[x, y] in &c.points {
        w.rapid(Point3::new(x, y, c.retract))?;
        w.rapid(Point3::new(x, y, c.z_top))?;

        match c.peck {
            None => {
                w.feed_move(Point3::new(x, y, c.depth), c.feed)?;
                if let Some(d) = c.dwell {
                    w.line(format!("G4 P{}", compact(d, 3)));
                }
                w.rapid(Point3::new(x, y, c.retract))?;
            }
            Some(peck) => {
                let mut top = c.z_top;
                let mut first = true;
                loop {
                    let target = (top - peck).max(c.depth);
                    if !first {
                        let resume = (top + PECK_CLEARANCE).min(c.z_top);
                        w.rapid(Point3::new(x, y, resume))?;
                    }
                    w.feed_move(Point3::new(x, y, target), c.feed)?;
                    let last = target <= c.depth;
                    if last {
                        if let Some(d) = c.dwell {
                            w.line(format!("G4 P{}", compact(d, 3)));
                        }
                    }
                    w.rapid(Point3::new(x, y, c.retract))?;
                    top = target;
                    first = false;
                    if last {
                        break;
                    }
                }
            }
        }
    }
    Ok(())
}

/// Emit a drilling cycle as a canned cycle: `G83` (peck) / `G82` (dwell) / `G81`
/// (plain), one cycle line plus one line per further hole, then `G80`.
fn canned_drill(w: &mut Writer, c: &DrillCycle) -> Result<(), PostError> {
    w.check_feed(c.feed)?;
    if c.points.is_empty() {
        return Ok(());
    }

    let p = w.precision();
    let (x0, y0) = (c.points[0][0], c.points[0][1]);
    w.rapid(Point3::new(x0, y0, c.retract))?;

    let z = num(c.depth, p);
    let r = num(c.retract, p);
    let f = compact(c.feed, p);
    let cycle = match (c.peck, c.dwell) {
        (Some(q), Some(d)) => format!(
            "G98 G83 X{} Y{} Z{z} R{r} Q{} P{} F{f}",
            num(x0, p),
            num(y0, p),
            compact(q, p),
            compact(d, 3),
        ),
        (Some(q), None) => format!(
            "G98 G83 X{} Y{} Z{z} R{r} Q{} F{f}",
            num(x0, p),
            num(y0, p),
            compact(q, p),
        ),
        (None, Some(d)) => format!(
            "G98 G82 X{} Y{} Z{z} R{r} P{} F{f}",
            num(x0, p),
            num(y0, p),
            compact(d, 3),
        ),
        (None, None) => format!("G98 G81 X{} Y{} Z{z} R{r} F{f}", num(x0, p), num(y0, p)),
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
