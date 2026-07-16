//! Concentric-ring area clearing, shared by the pocket strategy and profile
//! roughing (stepover). Rings are offset loops of a region marching inward from
//! its wall; each is a plain closed cutting loop (approach, plunge per the plunge
//! strategy, cut with any closure overlap, retract).

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, JoinStyle, Point, Polygon};
use cam_model::{Heights, Plunge};

use crate::CancelToken;

/// Safety cap on the number of concentric rings (guards the offset loop).
const MAX_RINGS: usize = 100_000;

/// Why ring generation stopped without producing rings.
pub(crate) enum RingsError {
    /// The job was cancelled mid-generation.
    Cancelled,
    /// A geometry offset failed.
    Offset(String),
}

/// Concentric offset rings clearing a region: offset it inward from the wall by
/// `first` (usually the tool radius), then by `stepover` repeatedly until it
/// closes off. Returns every resulting loop (outer boundaries and island/hole
/// loops), ordered by increasing offset (wall-most first). An empty result means
/// the tool cannot even enter.
pub(crate) fn concentric_rings(
    region: &Polygon,
    first: f64,
    stepover: f64,
    cancel: &CancelToken,
) -> Result<Vec<Vec<Point>>, RingsError> {
    let mut rings: Vec<Vec<Point>> = Vec::new();
    let mut d = first;
    loop {
        if cancel.is_cancelled() {
            return Err(RingsError::Cancelled);
        }
        let offsets = offset(std::slice::from_ref(region), -d, JoinStyle::Round)
            .map_err(|e| RingsError::Offset(e.to_string()))?;
        if offsets.is_empty() {
            break;
        }
        for poly in &offsets {
            rings.push(poly.outer().points().to_vec());
            for hole in poly.holes() {
                rings.push(hole.points().to_vec());
            }
        }
        d += stepover;
        // A non-positive stepover would never advance; take the wall ring and stop.
        if stepover <= 0.0 || rings.len() > MAX_RINGS {
            break;
        }
    }
    Ok(rings)
}

/// Emit approach, plunge, one closed cutting loop (plus any closure overlap), and
/// retract for a ring at height `z`. The entry uses the given plunge strategy: a
/// helix/ramp is placed on the *inward* side of the ring so it stays within the
/// cleared area, not the wall.
#[allow(clippy::too_many_arguments)]
pub(crate) fn emit_ring(
    prog: &mut Program,
    pts: &[Point],
    id: u32,
    feed: f64,
    plunge_feed: f64,
    plunge: Plunge,
    lead_overlap: f64,
    h: &Heights,
    z: f64,
) {
    if pts.len() < 3 {
        return;
    }
    let start = pts[0];
    let link = Tag::new(id, MoveKind::Link);
    let plunge_tag = Tag::new(id, MoveKind::Plunge);
    let cut = Tag::new(id, MoveKind::Cutting);
    let retract = Tag::new(id, MoveKind::Retract);

    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.clearance),
        tag: link,
    });
    prog.push(Step::Rapid {
        to: Point3::new(start.x, start.y, h.top_of_stock),
        tag: link,
    });
    let tan = crate::profile::start_tangent(pts);
    let out = crate::profile::outward_normal(pts);
    crate::profile::emit_plunge(
        prog,
        start,
        tan,
        (-out.0, -out.1),
        h.top_of_stock,
        z,
        plunge,
        plunge_feed,
        feed,
        plunge_tag,
    );
    let (loop_pts, exit_pt, _tan) = crate::emit::loop_with_overlap(pts, lead_overlap);
    crate::emit::cut_polyline(prog, &loop_pts, feed, cut, z);
    prog.push(Step::Rapid {
        to: Point3::new(exit_pt.x, exit_pt.y, h.clearance),
        tag: retract,
    });
}

/// Cut one closed ring (plus closure overlap) at `z`, returning the exit point.
fn cut_ring(prog: &mut Program, pts: &[Point], feed: f64, tag: Tag, lead_overlap: f64, z: f64) -> Point {
    let (loop_pts, exit, _tan) = crate::emit::loop_with_overlap(pts, lead_overlap);
    crate::emit::cut_polyline(prog, &loop_pts, feed, tag, z);
    exit
}

/// Rapid over `p`, down through cleared air to `from_z`, then straight-plunge to
/// `z`. Used for level re-entries and lift-reposition re-entries, which only ever
/// drop one stepdown into a layer already cleared above.
#[allow(clippy::too_many_arguments)]
fn enter_straight(prog: &mut Program, p: Point, from_z: f64, z: f64, h: &Heights, plunge_feed: f64, id: u32) {
    prog.push(Step::Rapid {
        to: Point3::new(p.x, p.y, h.clearance),
        tag: Tag::new(id, MoveKind::Link),
    });
    prog.push(Step::Rapid {
        to: Point3::new(p.x, p.y, from_z),
        tag: Tag::new(id, MoveKind::Link),
    });
    prog.push(Step::Linear {
        to: Point3::new(p.x, p.y, z),
        feed: plunge_feed,
        tag: Tag::new(id, MoveKind::Plunge),
    });
}

/// Emit a **stay-down, inside-out** pocket path. Per depth level: enter once at the
/// innermost ring, then cut every ring, **linking at depth** to the next ring when
/// the hop is short (adjacent rings), and only **lifting** when the hop is too long
/// to cut across (an island loop or a pinched-off lobe). Between levels the tool
/// retracts to the interior start and replunges, so every level is carved the same
/// way (wall last). Only the very first entry uses the op's plunge strategy (into
/// virgin material); later entries drop one stepdown into cleared stock, so they
/// plunge straight.
///
/// `rings` must be ordered **inner-first** (innermost at index 0, wall last).
/// `link_threshold` is the hop above which a stay-down link would cut across uncut
/// material, so we lift instead.
#[allow(clippy::too_many_arguments)]
pub(crate) fn emit_stay_down(
    prog: &mut Program,
    rings: &[Vec<Point>],
    start: Option<[f64; 2]>,
    id: u32,
    feed: f64,
    plunge_feed: f64,
    plunge: Plunge,
    lead_overlap: f64,
    h: &Heights,
    levels: &[f64],
    link_threshold: f64,
) {
    if rings.is_empty() {
        return;
    }
    let link = Tag::new(id, MoveKind::Link);
    let cut = Tag::new(id, MoveKind::Cutting);
    let retract = Tag::new(id, MoveKind::Retract);

    let mut from_z = h.top_of_stock;
    for &z in levels {
        // Enter at the innermost ring (a boundary loop, so the plunge strategy's
        // helix/ramp is placed on the inward, cleared side), per the operator's
        // start preference. Every level enters this way so a helix-plunge op helixes
        // each level; the descent is only one stepdown into stock cleared above.
        let r0 = crate::profile::rotate_to_start(&rings[0], start);
        let tan = crate::profile::start_tangent(&r0);
        let out = crate::profile::outward_normal(&r0);
        prog.push(Step::Rapid {
            to: Point3::new(r0[0].x, r0[0].y, h.clearance),
            tag: link,
        });
        prog.push(Step::Rapid {
            to: Point3::new(r0[0].x, r0[0].y, from_z),
            tag: link,
        });
        crate::profile::emit_plunge(
            prog,
            r0[0],
            tan,
            (-out.0, -out.1),
            from_z,
            z,
            plunge,
            plunge_feed,
            feed,
            Tag::new(id, MoveKind::Plunge),
        );
        let mut prev_end = cut_ring(prog, &r0, feed, cut, lead_overlap, z);

        for ring in &rings[1..] {
            // Begin this ring at the point nearest where the last one ended, so the
            // hop is the shortest possible — a clean radial step for adjacent rings.
            let ri = crate::profile::rotate_to_start(ring, Some([prev_end.x, prev_end.y]));
            let start_i = ri[0];
            let hop = (start_i.x - prev_end.x).hypot(start_i.y - prev_end.y);
            if hop <= link_threshold {
                // Stay down: cut across the short gap to the next ring.
                prog.push(Step::Linear {
                    to: Point3::new(start_i.x, start_i.y, z),
                    feed,
                    tag: cut,
                });
            } else {
                // Too far to cut across (island / lobe) — lift, reposition, replunge.
                prog.push(Step::Rapid {
                    to: Point3::new(prev_end.x, prev_end.y, h.clearance),
                    tag: link,
                });
                enter_straight(prog, start_i, from_z, z, h, plunge_feed, id);
            }
            prev_end = cut_ring(prog, &ri, feed, cut, lead_overlap, z);
        }
        // Retract; the next level re-enters at the interior start.
        prog.push(Step::Rapid {
            to: Point3::new(prev_end.x, prev_end.y, h.clearance),
            tag: retract,
        });
        from_z = z;
    }
}
