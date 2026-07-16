//! A Fanuc-dialect post (also a reasonable base for Haas).
//!
//! Fanuc has the machinery grbl lacks — most importantly **canned drilling
//! cycles**. So where the grbl post expands a [`Drill`](cam_cldata::Step::Drill)
//! intent into dozens of explicit peck moves, this post emits a single
//! `G83`/`G82`/`G81` cycle and a `G80` to cancel it. Same neutral CL-data, very
//! different G-code — the capabilities model doing real work.

use cam_cldata::{Coolant, CutterComp, DrillCycle, Point3, Program, SpindleDir, Step};
use cam_model::Machine;

use crate::words::{compact, num, sanitize};
use crate::writer::Writer;
use crate::{Capabilities, Post, PostError, PostOptions};

/// Program number emitted in the `O` word (Fanuc requires one).
const PROGRAM_NUMBER: u32 = 1000;

/// The Fanuc post. Stateless; construct with [`FanucPost`].
#[derive(Clone, Copy, Debug, Default)]
pub struct FanucPost;

impl Post for FanucPost {
    fn name(&self) -> &str {
        "fanuc"
    }

    fn capabilities(&self) -> Capabilities {
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
        crate::check_travel(program, machine)?;
        let mut w = Writer::new(machine, options.precision);

        // Fanuc programs are bracketed by `%` and carry an O-number.
        w.line("%".to_string());
        match &options.program_name {
            Some(name) => w.line(format!("O{PROGRAM_NUMBER} ({})", sanitize(name))),
            None => w.line(format!("O{PROGRAM_NUMBER}")),
        }
        // Safe-start block: plane, units, absolute, feed/min, cancel comp / length
        // offset / canned cycle.
        w.line("G17 G21 G90 G94 G40 G49 G80".to_string());
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
                Step::CutterComp(c) => w.line(match c {
                    CutterComp::Off => "G40".to_string(),
                    CutterComp::Left(d) => format!("G41 D{d}"),
                    CutterComp::Right(d) => format!("G42 D{d}"),
                }),
                // Fanuc dwell is in seconds via X.
                Step::Dwell { seconds } => w.line(format!("G4 X{}", compact(*seconds, 3))),
                Step::Rapid { to, .. } => w.rapid(*to)?,
                Step::Linear { to, feed, .. } => w.feed_move(*to, *feed)?,
                Step::Arc {
                    end,
                    center,
                    dir,
                    feed,
                    ..
                } => w.arc(*end, *center, *dir, *feed)?,
                Step::Drill(cycle) => drill(&mut w, cycle)?,
            }
        }

        w.line("M30".to_string());
        w.line("%".to_string());
        Ok(w.finish())
    }
}

/// Emit a drilling cycle as a Fanuc canned cycle: `G83` (peck) / `G82` (dwell) /
/// `G81` (plain), one cycle line plus one line per further hole, then `G80`.
fn drill(w: &mut Writer, c: &DrillCycle) -> Result<(), PostError> {
    w.check_feed(c.feed)?;
    if c.points.is_empty() {
        return Ok(());
    }

    let p = w.precision();
    let (x0, y0) = (c.points[0][0], c.points[0][1]);

    // Establish a safe initial Z (returned to by G98) and position over the
    // first hole; the cycle's R word handles the rest.
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

    // The cycle left the tool over the last hole at the initial (retract) plane;
    // forget the modal motion word so the next move re-emits it.
    let last = *c.points.last().unwrap();
    w.set_position(Point3::new(last[0], last[1], c.retract));
    w.reset_modal();
    Ok(())
}
