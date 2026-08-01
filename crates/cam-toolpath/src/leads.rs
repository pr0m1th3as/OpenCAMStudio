//! Lead-in / lead-out geometry shared by the strategies that run a tool onto and
//! off a closed contour (profile, chamfer). A lead is a short approach/departure
//! move — linear (tangent) or a 90° tangent arc — placed on the air side of the
//! contour so the cutter eases onto the edge instead of diving straight in.
//!
//! The caller supplies the on-contour point, the travel tangent there, and the
//! outward normal (see `profile::outward_normal_at`); these helpers turn that into
//! the off-contour entry/exit points and the emitted move.

use cam_cldata::{ArcDir, Point3, Program, Step, Tag};
use cam_geo::{Point, Polygon};
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

/// Sample points along a lead move, for containment testing against the cleared
/// region. `on` is the on-contour endpoint (start of a lead-in, end of a lead-out),
/// `off` the off-contour end, `out` the cleared-side normal — the same geometry
/// [`emit_lead`] draws. A linear lead is a straight segment, so its far end is the
/// extreme; an arc is a 90° tangent arc about `on + out·radius`, sampled along its
/// sweep so a bulge that pokes past the far wall is caught, not just the endpoints.
pub(crate) fn lead_samples(on: Point, off: Point, out: (f64, f64), lead: Lead) -> Vec<Point> {
    lead_samples_n(on, off, out, lead, 8)
}

/// [`lead_samples`] at a chosen density.
///
/// Eight points is plenty to *test* a lead against the cleared region, which is all the
/// guard needs. It is not enough to **re-fit**: `fit_arcs` rejects a run whose chords
/// depart from the circle by more than its tolerance, and at eight points a 3 mm lead
/// sags 14 µm against a 10 µm tolerance — correctly refused as "a polygon whose corners
/// merely happen to be concyclic". A ramp descends *along* the lead, so its samples do
/// get re-fitted, and they have to be fine enough to survive it. See
/// [`arc_lead_density`].
pub(crate) fn lead_samples_n(
    on: Point,
    off: Point,
    out: (f64, f64),
    lead: Lead,
    n: usize,
) -> Vec<Point> {
    match lead {
        Lead::None => Vec::new(),
        Lead::Linear { .. } => vec![off],
        Lead::Arc { radius } => {
            let centre = Point::new(on.x + out.0 * radius, on.y + out.1 * radius);
            arc_samples(centre, on, off, n.max(2))
        }
    }
}

/// How many segments an arc lead of `radius` needs for its chords to stay within `tol`
/// of the true circle, so a re-fit recognises it as the arc it is.
///
/// The sagitta of a chord subtending `θ` on radius `r` is `r(1 − cos(θ/2))`; solving for
/// `θ` and dividing into the sweep gives the count. Halving the tolerance leaves margin,
/// because the fitter checks the *worst* chord and the sweep is not always exactly 90°.
pub(crate) fn arc_lead_density(radius: f64, sweep: f64, tol: f64) -> usize {
    if radius <= 0.0 || tol <= 0.0 {
        return 8;
    }
    let c = 1.0 - tol / (2.0 * radius);
    if !(-1.0..=1.0).contains(&c) {
        return 8;
    }
    let max_step = 2.0 * c.acos();
    if max_step <= 1e-9 {
        return 64;
    }
    ((sweep / max_step).ceil() as usize).clamp(8, 256)
}

/// Points evenly spaced along the short arc from `a` to `b` about `centre`
/// (inclusive of both ends).
fn arc_samples(centre: Point, a: Point, b: Point, n: usize) -> Vec<Point> {
    let r = (a.x - centre.x).hypot(a.y - centre.y);
    let a0 = (a.y - centre.y).atan2(a.x - centre.x);
    let b0 = (b.y - centre.y).atan2(b.x - centre.x);
    let mut sweep = b0 - a0;
    while sweep > std::f64::consts::PI {
        sweep -= std::f64::consts::TAU;
    }
    while sweep < -std::f64::consts::PI {
        sweep += std::f64::consts::TAU;
    }
    (0..=n)
        .map(|i| {
            let ang = a0 + sweep * (i as f64) / (n as f64);
            Point::new(centre.x + r * ang.cos(), centre.y + r * ang.sin())
        })
        .collect()
}

/// Whether every sampled point of a lead stays inside the cleared/air region (the
/// union of `guard` polygons — the area bounded by the walls the lead must not cross).
/// An empty guard carries no information, so it never blocks a lead.
pub(crate) fn lead_fits(guard: &[Polygon], samples: &[Point]) -> bool {
    guard.is_empty() || samples.iter().all(|p| guard.iter().any(|g| g.contains(*p)))
}

/// A lead resolved through the overshoot guard: the lead unchanged when its whole
/// swept move (arc bulge included) stays inside `guard` — the cleared/air region the
/// cutter approaches from — otherwise [`Lead::None`], so a lead too big for the
/// feature falls back to a plain pass instead of easing on across a finished wall.
/// `on` is the on-contour endpoint, `tan` the travel tangent there, `out` the
/// cleared-side normal. `is_in` selects lead-in vs lead-out geometry. An empty
/// `guard` leaves the lead untouched (used where the air side is unbounded).
pub(crate) fn guard_lead(
    guard: &[Polygon],
    on: Point,
    tan: (f64, f64),
    out: (f64, f64),
    lead: Lead,
    is_in: bool,
) -> Lead {
    let off = if is_in {
        lead_start_point(on, tan, out, lead)
    } else {
        lead_end_point(on, tan, out, lead)
    };
    if lead_fits(guard, &lead_samples(on, off, out, lead)) {
        lead
    } else {
        Lead::None
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
