//! A Fanuc-dialect post (also a reasonable base for Haas).
//!
//! Fanuc has the machinery grbl lacks — most importantly **canned drilling
//! cycles**. So where the grbl post expands a [`Drill`](cam_cldata::Step::Drill)
//! intent into dozens of explicit peck moves, this post emits a single
//! `G83`/`G82`/`G81` cycle and a `G80` to cancel it. Same neutral CL-data, very
//! different G-code — the capabilities model doing real work.

use cam_cldata::Program;
use cam_model::Machine;

use crate::{Capabilities, Post, PostError, PostOptions};

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
        crate::dialect::emit(program, machine, options, &crate::dialect::FANUC)
    }
}
