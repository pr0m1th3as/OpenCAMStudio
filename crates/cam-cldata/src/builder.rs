//! A small fluent builder for authoring [`Program`]s by hand.
//!
//! It threads the *current operation id* and *current feed* through a chain of
//! calls, so a hand-written program reads close to the G-code it will become —
//! invaluable for tests and for prototyping before `cam-toolpath` exists.

use crate::{ArcDir, Coolant, DrillCycle, MoveKind, Point3, Program, SpindleDir, Step, Tag};

/// Fluent builder for a [`Program`]. See the [module docs](self).
#[derive(Clone, Debug, Default)]
pub struct ProgramBuilder {
    steps: Vec<Step>,
    op_id: u32,
    feed: f64,
}

impl ProgramBuilder {
    /// A fresh builder with operation id 0 and zero feed.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the current operation id, stamped onto the tag of subsequent motions.
    pub fn op(mut self, op_id: u32) -> Self {
        self.op_id = op_id;
        self
    }

    /// Set the current cutting feed (mm/min) used by [`linear`](Self::linear) and
    /// [`arc`](Self::arc).
    pub fn feed(mut self, feed: f64) -> Self {
        self.feed = feed;
        self
    }

    /// Emit a free-text comment.
    pub fn comment(mut self, text: impl Into<String>) -> Self {
        self.steps.push(Step::Comment(text.into()));
        self
    }

    /// Start the spindle.
    pub fn spindle_on(mut self, rpm: f64, dir: SpindleDir) -> Self {
        self.steps.push(Step::Spindle { rpm, dir });
        self
    }

    /// Stop the spindle.
    pub fn spindle_off(mut self) -> Self {
        self.steps.push(Step::SpindleOff);
        self
    }

    /// Set the coolant state.
    pub fn coolant(mut self, coolant: Coolant) -> Self {
        self.steps.push(Step::Coolant(coolant));
        self
    }

    /// Change tool.
    pub fn tool_change(mut self, tool: u32) -> Self {
        self.steps.push(Step::ToolChange { tool });
        self
    }

    /// Select work coordinate datum `index` (1-based) for subsequent steps.
    pub fn datum(mut self, index: u32) -> Self {
        self.steps.push(Step::Datum(index));
        self
    }

    /// A mandatory program stop (`M00`) — the machine waits for the operator.
    pub fn stop(mut self) -> Self {
        self.steps.push(Step::Stop);
        self
    }

    /// Dwell in place for `seconds`.
    pub fn dwell(mut self, seconds: f64) -> Self {
        self.steps.push(Step::Dwell { seconds });
        self
    }

    /// Rapid traverse to `to`, tagged with the current op and the given `kind`.
    pub fn rapid(mut self, to: Point3, kind: MoveKind) -> Self {
        let tag = Tag::new(self.op_id, kind);
        self.steps.push(Step::Rapid { to, tag });
        self
    }

    /// Linear cutting move to `to` at the current feed, tagged with `kind`.
    pub fn linear(mut self, to: Point3, kind: MoveKind) -> Self {
        let tag = Tag::new(self.op_id, kind);
        self.steps.push(Step::Linear {
            to,
            feed: self.feed,
            tag,
        });
        self
    }

    /// Circular/helical move to `end` about the absolute `center`, at the current
    /// feed, tagged with `kind`.
    pub fn arc(mut self, end: Point3, center: Point3, dir: ArcDir, kind: MoveKind) -> Self {
        let tag = Tag::new(self.op_id, kind);
        self.steps.push(Step::Arc {
            end,
            center,
            dir,
            feed: self.feed,
            tag,
        });
        self
    }

    /// Emit a Tier-2 drilling cycle.
    pub fn drill(mut self, cycle: DrillCycle) -> Self {
        self.steps.push(Step::Drill(cycle));
        self
    }

    /// Finish, yielding the assembled program.
    pub fn build(self) -> Program {
        Program { steps: self.steps }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn threads_op_id_and_feed_onto_motions() {
        let prog = ProgramBuilder::new()
            .op(2)
            .feed(300.0)
            .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
            .linear(Point3::new(10.0, 0.0, -1.0), MoveKind::Cutting)
            .build();

        assert_eq!(prog.len(), 2);
        match &prog.steps()[0] {
            Step::Rapid { tag, .. } => {
                assert_eq!(*tag, Tag::new(2, MoveKind::Link));
            }
            other => panic!("expected rapid, got {other:?}"),
        }
        match &prog.steps()[1] {
            Step::Linear { feed, tag, .. } => {
                assert_eq!(*feed, 300.0);
                assert_eq!(*tag, Tag::new(2, MoveKind::Cutting));
            }
            other => panic!("expected linear, got {other:?}"),
        }
    }

    #[test]
    fn preserves_step_order() {
        let prog = ProgramBuilder::new()
            .comment("start")
            .spindle_on(1000.0, SpindleDir::Cw)
            .op(0)
            .rapid(Point3::new(0.0, 0.0, 5.0), MoveKind::Link)
            .spindle_off()
            .build();

        assert!(matches!(prog.steps()[0], Step::Comment(_)));
        assert!(matches!(prog.steps()[1], Step::Spindle { .. }));
        assert!(matches!(prog.steps()[2], Step::Rapid { .. }));
        assert!(matches!(prog.steps()[3], Step::SpindleOff));
    }
}
