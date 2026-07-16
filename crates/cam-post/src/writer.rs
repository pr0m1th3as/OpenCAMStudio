//! A shared modal G-code writer used by the posts.
//!
//! Tracks the current position, motion word, and feed, emitting a word only when
//! it changes — idiomatic and deterministic. Machine limits (feed, envelope) are
//! checked as moves are written. Post-specific bits (preamble, drilling cycles,
//! dwell dialect) live in each post; this is the common machinery.

use cam_cldata::{ArcDir, Point3};
use cam_model::Machine;

use crate::words::num;
use crate::PostError;

pub(crate) struct Writer<'a> {
    out: Vec<String>,
    machine: &'a Machine,
    precision: usize,
    cur: Option<Point3>,
    mode: Option<u8>,
    feed: Option<f64>,
}

impl<'a> Writer<'a> {
    pub(crate) fn new(machine: &'a Machine, precision: usize) -> Self {
        Self {
            out: Vec::new(),
            machine,
            precision,
            cur: None,
            mode: None,
            feed: None,
        }
    }

    /// Coordinate precision (decimals).
    pub(crate) fn precision(&self) -> usize {
        self.precision
    }

    /// Append a raw line.
    pub(crate) fn line(&mut self, s: String) {
        self.out.push(s);
    }

    /// Forget the modal motion state — call after emitting raw motion lines (e.g.
    /// a canned cycle) so the next move re-emits its words.
    pub(crate) fn reset_modal(&mut self) {
        self.mode = None;
        self.feed = None;
    }

    /// Record a position without emitting anything (e.g. after a canned cycle
    /// that left the tool at a known place).
    pub(crate) fn set_position(&mut self, p: Point3) {
        self.cur = Some(p);
    }

    /// Join the accumulated lines, ending with a newline.
    pub(crate) fn finish(self) -> String {
        let mut s = self.out.join("\n");
        s.push('\n');
        s
    }

    pub(crate) fn check_feed(&self, feed: f64) -> Result<(), PostError> {
        if feed > self.machine.max_feed {
            Err(PostError::FeedOutOfRange(feed))
        } else {
            Ok(())
        }
    }

    pub(crate) fn check_spindle(&self, rpm: f64) -> Result<(), PostError> {
        if rpm > self.machine.max_spindle_rpm {
            Err(PostError::SpindleOutOfRange(rpm))
        } else {
            Ok(())
        }
    }

    /// Emit a motion to `to` with motion word `g`, optional feed, and optional arc
    /// offsets, suppressing words that have not changed.
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
                words.push(format!("F{}", crate::words::compact(f, p)));
                self.feed = Some(f);
            }
        }

        self.cur = Some(to);
        if !words.is_empty() {
            self.out.push(words.join(" "));
        }
        Ok(())
    }

    /// Rapid traverse (`G0`).
    pub(crate) fn rapid(&mut self, to: Point3) -> Result<(), PostError> {
        self.motion(0, to, None, None)
    }

    /// Linear feed move (`G1`).
    pub(crate) fn feed_move(&mut self, to: Point3, feed: f64) -> Result<(), PostError> {
        self.check_feed(feed)?;
        self.motion(1, to, Some(feed), None)
    }

    /// Circular move (`G2`/`G3`) about an absolute `center`; `I`/`J` are emitted
    /// as the incremental offset from the start point.
    pub(crate) fn arc(
        &mut self,
        end: Point3,
        center: Point3,
        dir: ArcDir,
        feed: f64,
    ) -> Result<(), PostError> {
        self.check_feed(feed)?;
        let start = self.cur.ok_or(PostError::ArcWithoutStart)?;
        let ij = (center.x - start.x, center.y - start.y);
        let g = match dir {
            ArcDir::Cw => 2,
            ArcDir::Ccw => 3,
        };
        self.motion(g, end, Some(feed), Some(ij))
    }
}
