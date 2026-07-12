//! The grbl / FluidNC post.
//!
//! grbl speaks `G0/G1/G2/G3` and the common `M` words, but has **no canned
//! cycles** and no cutter compensation. So this post emits arcs natively but
//! **expands drilling into explicit peck moves**, and it queries the
//! [`Machine`] for spindle/feed/envelope limits as it formats — refusing output
//! that would exceed them rather than silently clamping.
//!
//! Output is **modal** (see [`crate::writer::Writer`]): idiomatic and
//! deterministic (golden-file friendly).

use cam_cldata::{Coolant, DrillCycle, Point3, Program, SpindleDir, Step};
use cam_model::Machine;

use crate::words::{compact, sanitize};
use crate::writer::Writer;
use crate::{Capabilities, Post, PostError, PostOptions};

/// Rapid clearance left above the previous peck depth before feeding again, mm.
const PECK_CLEARANCE: f64 = 0.5;

/// The grbl post. Stateless; construct with [`GrblPost`].
#[derive(Clone, Copy, Debug, Default)]
pub struct GrblPost;

impl Post for GrblPost {
    fn name(&self) -> &str {
        "grbl"
    }

    fn capabilities(&self) -> Capabilities {
        Capabilities {
            arcs: true,
            canned_drill: false,
            cutter_comp: false,
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
        let mut w = Writer::new(machine, options.precision);

        // Preamble: header comment, then modal defaults and the work offset.
        if let Some(name) = &options.program_name {
            w.line(format!("({})", sanitize(name)));
        }
        w.line("G17 G21 G90 G94".to_string());
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
                Step::Dwell { seconds } => w.line(format!("G4 P{}", compact(*seconds, 3))),
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
        Ok(w.finish())
    }
}

/// Expand a drilling cycle into explicit moves (grbl has no canned cycle). Peck
/// drilling follows `G83` semantics: full retract to the clearance plane after
/// each peck to clear chips, then a rapid back down to just above the last depth.
fn drill(w: &mut Writer, c: &DrillCycle) -> Result<(), PostError> {
    w.check_feed(c.feed)?;
    for &[x, y] in &c.points {
        // Reposition to the hole at the clearance plane, then plunge to the
        // surface, before any cutting.
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
                        // Rapid back down to just above where the last peck ended
                        // (never above the surface).
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
