//! The grbl / FluidNC post.
//!
//! grbl speaks `G0/G1/G2/G3` and the common `M` words, but has **no canned
//! cycles** and no cutter compensation. So this post emits arcs natively but
//! **expands drilling into explicit peck moves**, and it queries the
//! [`Machine`] for spindle/feed/envelope limits as it formats — refusing output
//! that would exceed them rather than silently clamping.
//!
//! Output is **modal**: a motion word or an axis word is emitted only when it
//! changes, which is both idiomatic and deterministic (golden-file friendly).

use cam_cldata::{ArcDir, Coolant, DrillCycle, Point3, Program, SpindleDir, Step};
use cam_model::Machine;

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
        let mut w = Writer::new(machine, options);

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
                    if *rpm > machine.max_spindle_rpm {
                        return Err(PostError::SpindleOutOfRange(*rpm));
                    }
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
                Step::Drill(cycle) => w.drill(cycle)?,
            }
        }

        w.line("M30".to_string());
        Ok(w.finish())
    }
}

/// Accumulates output lines while tracking modal state (motion word, position,
/// feed) so words are emitted only when they change.
struct Writer<'a> {
    out: Vec<String>,
    machine: &'a Machine,
    precision: usize,
    cur: Option<Point3>,
    mode: Option<u8>,
    feed: Option<f64>,
}

impl<'a> Writer<'a> {
    fn new(machine: &'a Machine, options: &PostOptions) -> Self {
        Self {
            out: Vec::new(),
            machine,
            precision: options.precision,
            cur: None,
            mode: None,
            feed: None,
        }
    }

    fn line(&mut self, s: String) {
        self.out.push(s);
    }

    fn finish(self) -> String {
        let mut s = self.out.join("\n");
        s.push('\n');
        s
    }

    fn check_envelope(&self, p: Point3) -> Result<(), PostError> {
        if self.machine.envelope.contains(p.x, p.y, p.z) {
            Ok(())
        } else {
            Err(PostError::OutOfEnvelope {
                x: p.x,
                y: p.y,
                z: p.z,
            })
        }
    }

    /// Emit a motion to `to` with motion word `g`, optional feed, and optional
    /// arc offsets, suppressing words that have not changed.
    fn motion(
        &mut self,
        g: u8,
        to: Point3,
        feed: Option<f64>,
        ij: Option<(f64, f64)>,
    ) -> Result<(), PostError> {
        let p = self.precision;
        let mut words: Vec<String> = Vec::new();

        if self.mode != Some(g) {
            words.push(format!("G{g}"));
            self.mode = Some(g);
        }

        let changed = |old: Option<f64>, new: f64| old.is_none_or(|o| num(o, p) != num(new, p));
        if changed(self.cur.map(|c| c.x), to.x) {
            words.push(format!("X{}", num(to.x, p)));
        }
        if changed(self.cur.map(|c| c.y), to.y) {
            words.push(format!("Y{}", num(to.y, p)));
        }
        if changed(self.cur.map(|c| c.z), to.z) {
            words.push(format!("Z{}", num(to.z, p)));
        }

        if let Some((i, j)) = ij {
            words.push(format!("I{}", num(i, p)));
            words.push(format!("J{}", num(j, p)));
        }

        if let Some(f) = feed {
            if self.feed != Some(f) {
                words.push(format!("F{}", compact(f, p)));
                self.feed = Some(f);
            }
        }

        self.cur = Some(to);
        if !words.is_empty() {
            self.out.push(words.join(" "));
        }
        Ok(())
    }

    fn rapid(&mut self, to: Point3) -> Result<(), PostError> {
        self.check_envelope(to)?;
        self.motion(0, to, None, None)
    }

    fn feed_move(&mut self, to: Point3, feed: f64) -> Result<(), PostError> {
        if feed > self.machine.max_feed {
            return Err(PostError::FeedOutOfRange(feed));
        }
        self.check_envelope(to)?;
        self.motion(1, to, Some(feed), None)
    }

    fn arc(
        &mut self,
        end: Point3,
        center: Point3,
        dir: ArcDir,
        feed: f64,
    ) -> Result<(), PostError> {
        if feed > self.machine.max_feed {
            return Err(PostError::FeedOutOfRange(feed));
        }
        let start = self.cur.ok_or(PostError::ArcWithoutStart)?;
        self.check_envelope(end)?;
        // grbl uses I/J as the center offset *from the start point* (incremental).
        let ij = (center.x - start.x, center.y - start.y);
        let g = match dir {
            ArcDir::Cw => 2,
            ArcDir::Ccw => 3,
        };
        self.motion(g, end, Some(feed), Some(ij))
    }

    /// Expand a drilling cycle into explicit moves (grbl has no canned cycle).
    /// Peck drilling follows `G83` semantics: full retract to the clearance plane
    /// after each peck to clear chips, then a rapid back down to just above the
    /// last depth.
    fn drill(&mut self, c: &DrillCycle) -> Result<(), PostError> {
        if c.feed > self.machine.max_feed {
            return Err(PostError::FeedOutOfRange(c.feed));
        }
        for &[x, y] in &c.points {
            // Reposition to the hole at the clearance plane, then plunge to the
            // surface, before any cutting.
            self.rapid(Point3::new(x, y, c.retract))?;
            self.rapid(Point3::new(x, y, c.z_top))?;

            match c.peck {
                None => {
                    self.feed_move(Point3::new(x, y, c.depth), c.feed)?;
                    if let Some(d) = c.dwell {
                        self.line(format!("G4 P{}", compact(d, 3)));
                    }
                    self.rapid(Point3::new(x, y, c.retract))?;
                }
                Some(peck) => {
                    let mut top = c.z_top;
                    let mut first = true;
                    loop {
                        let target = (top - peck).max(c.depth);
                        if !first {
                            // Rapid back down to just above where the last peck
                            // ended (never above the surface).
                            let resume = (top + PECK_CLEARANCE).min(c.z_top);
                            self.rapid(Point3::new(x, y, resume))?;
                        }
                        self.feed_move(Point3::new(x, y, target), c.feed)?;
                        let last = target <= c.depth;
                        if last {
                            if let Some(d) = c.dwell {
                                self.line(format!("G4 P{}", compact(d, 3)));
                            }
                        }
                        self.rapid(Point3::new(x, y, c.retract))?;
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
}

/// Format a coordinate with a fixed number of decimals, normalising `-0.000` to
/// `0.000` so output is sign-stable.
fn num(v: f64, precision: usize) -> String {
    let s = format!("{v:.precision$}");
    if s.starts_with('-') && s[1..].bytes().all(|b| b == b'0' || b == b'.') {
        s[1..].to_string()
    } else {
        s
    }
}

/// Format a value with up to `maxdec` decimals but no trailing zeros — for feeds
/// and speeds, where `F300` reads better than `F300.000`. Only the fractional
/// part is trimmed, so integers keep their zeros (`S1000`, not `S1`).
fn compact(v: f64, maxdec: usize) -> String {
    let s = format!("{v:.maxdec$}");
    let s = if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.')
    } else {
        &s
    };
    if s.is_empty() || s == "-0" {
        "0".to_string()
    } else {
        s.to_string()
    }
}

/// Make a string safe inside a `(…)` grbl comment by stripping parentheses.
fn sanitize(text: &str) -> String {
    text.replace(['(', ')'], "")
}
