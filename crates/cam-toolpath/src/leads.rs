//! Lead-in / lead-out geometry shared by the strategies that run a tool onto and
//! off a closed contour (profile, chamfer). A lead is a short approach/departure
//! move — linear (tangent) or a 90° tangent arc — placed on the air side of the
//! contour so the cutter eases onto the edge instead of diving straight in.
//!
//! The caller supplies the on-contour point, the travel tangent there, and the
//! outward normal (see `profile::outward_normal_at`); these helpers turn that into
//! the off-contour entry/exit points and the emitted move.

use cam_cldata::{ArcDir, Point3, Program, Step, Tag};
use cam_geo::Point;
use cam_model::Lead;

/// The point the tool plunges at for a lead-in (off the contour), given the start,
/// its tangent, the outward normal, and the lead. `None` plunges on the contour.
pub(crate) fn lead_start_point(start: Point, tan: (f64, f64), out: (f64, f64), lead: Lead) -> Point {
    match lead {
        Lead::None => start,
        Lead::Linear { length } => Point::new(start.x - tan.0 * length, start.y - tan.1 * length),
        // The far end of a 90° tangent arc: centre − tangent·r, centre = start + out·r.
        Lead::Arc { radius } => Point::new(
            start.x + (out.0 - tan.0) * radius,
            start.y + (out.1 - tan.1) * radius,
        ),
    }
}

/// The point a lead-out departs to, mirroring [`lead_start_point`] with the arrival
/// tangent.
pub(crate) fn lead_end_point(start: Point, tan: (f64, f64), out: (f64, f64), lead: Lead) -> Point {
    match lead {
        Lead::None => start,
        Lead::Linear { length } => Point::new(start.x + tan.0 * length, start.y + tan.1 * length),
        Lead::Arc { radius } => Point::new(
            start.x + (out.0 + tan.0) * radius,
            start.y + (out.1 + tan.1) * radius,
        ),
    }
}

/// CW/CCW of the short arc from `from` to `to` about `centre`.
fn short_arc_dir(centre: Point, from: Point, to: Point) -> ArcDir {
    let a = (from.x - centre.x, from.y - centre.y);
    let b = (to.x - centre.x, to.y - centre.y);
    if a.0 * b.1 - a.1 * b.0 > 0.0 {
        ArcDir::Ccw
    } else {
        ArcDir::Cw
    }
}

/// Emit a lead move `from → to` at height `z` (linear, or a tangent arc centred at
/// `on + out·radius`, where `on` is the on-contour endpoint). `None` emits nothing.
#[allow(clippy::too_many_arguments)]
pub(crate) fn emit_lead(
    prog: &mut Program,
    from: Point,
    to: Point,
    on: Point,
    out: (f64, f64),
    lead: Lead,
    z: f64,
    feed: f64,
    tag: Tag,
) {
    match lead {
        Lead::None => {}
        Lead::Linear { .. } => prog.push(Step::Linear {
            to: Point3::new(to.x, to.y, z),
            feed,
            tag,
        }),
        Lead::Arc { radius } => {
            let centre = Point::new(on.x + out.0 * radius, on.y + out.1 * radius);
            let dir = short_arc_dir(centre, from, to);
            prog.push(Step::Arc {
                end: Point3::new(to.x, to.y, z),
                center: Point3::new(centre.x, centre.y, z),
                dir,
                feed,
                tag,
            });
        }
    }
}
