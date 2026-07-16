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

use cam_cldata::Program;
use cam_model::Machine;

use crate::{Capabilities, Post, PostError, PostOptions};

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
        crate::dialect::emit(program, machine, options, &crate::dialect::GRBL)
    }
}
