//! Concentric-ring area clearing, shared by the pocket strategy and profile
//! roughing (stepover). Rings are offset loops of a region marching inward from
//! its wall; each is a plain closed cutting loop (approach, plunge per the plunge
//! strategy, cut with any closure overlap, retract).

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, JoinStyle, Point, Polygon};
use cam_model::{Heights, Lead, Plunge};

use crate::CancelToken;

/// Safety cap on the number of concentric rings (guards the offset loop).
const MAX_RINGS: usize = 100_000;

/// One concentric clearing ring.
pub(crate) struct Ring {
    /// The loop to cut.
    pub pts: Vec<Point>,
    /// Whether this loop is a **finished wall** — a loop from the first (smallest)
    /// offset, hugging the boundary or an island. These get the wall-finish leads.
    pub is_wall: bool,
}

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
/// loops), ordered by increasing offset (wall-most first); loops from the first
/// offset are tagged `is_wall`. An empty result means the tool cannot even enter.
pub(crate) fn concentric_rings(
    region: &Polygon,
    first: f64,
    stepover: f64,
    cancel: &CancelToken,
) -> Result<Vec<Ring>, RingsError> {
    let mut rings: Vec<Ring> = Vec::new();
    let mut d = first;
    let mut is_wall = true; // the first offset hugs the walls
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
            rings.push(Ring {
                pts: poly.outer().points().to_vec(),
                is_wall,
            });
            for hole in poly.holes() {
                rings.push(Ring {
                    pts: hole.points().to_vec(),
                    is_wall,
                });
            }
        }
        d += stepover;
        is_wall = false;
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

/// The normal toward the **cleared** side of a wall loop — where the tool has been,
/// and where a wall lead must sit (into the pocket, away from the finished wall).
///
/// The clearing rings are wound so the cleared/pocket side is on the **left** of
/// travel (a boundary loop CCW, an island loop CW both satisfy this). Conventional
/// milling reverses every loop's travel, moving the cleared side to the **right**;
/// `reversed` (`climb == false`) selects that.
fn cleared_normal(t: (f64, f64), reversed: bool) -> (f64, f64) {
    let left = (-t.1, t.0);
    if reversed {
        (-left.0, -left.1)
    } else {
        left
    }
}

/// Move from `prev_end` to `p` at depth: a straight cut if the hop is short, else a
/// lift-reposition-replunge (an island loop or pinched lobe too far to cut across).
#[allow(clippy::too_many_arguments)]
fn approach(
    prog: &mut Program,
    prev_end: Point,
    p: Point,
    from_z: f64,
    z: f64,
    h: &Heights,
    feed: f64,
    plunge_feed: f64,
    id: u32,
    link_threshold: f64,
) {
    let hop = (p.x - prev_end.x).hypot(p.y - prev_end.y);
    if hop <= link_threshold {
        prog.push(Step::Linear {
            to: Point3::new(p.x, p.y, z),
            feed,
            tag: Tag::new(id, MoveKind::Cutting),
        });
    } else {
        prog.push(Step::Rapid {
            to: Point3::new(prev_end.x, prev_end.y, h.clearance),
            tag: Tag::new(id, MoveKind::Link),
        });
        enter_straight(prog, p, from_z, z, h, plunge_feed, id);
    }
}

/// Cut a finished-wall ring with a lead-in/out eased from the cleared side, and
/// return the exit point. Handles boundary (lead inward) and island (lead outward)
/// walls uniformly via the loop winding.
///
/// A lead that would overshoot the cleared area — an arc or line whose swept
/// geometry pokes past the far wall, which happens when the pocket is narrower than
/// the lead radius — is dropped back to a plain (no-lead) approach on that side, so
/// the cutter never eases on across the finished wall (`guard` bounds the cleared
/// region; see [`lead_fits`]). Lead-in and lead-out are judged independently.
#[allow(clippy::too_many_arguments)]
fn emit_wall_ring(
    prog: &mut Program,
    pts: &[Point],
    prev_end: Point,
    from_z: f64,
    z: f64,
    id: u32,
    feed: f64,
    plunge_feed: f64,
    lead_overlap: f64,
    lead_in: Lead,
    lead_out: Lead,
    guard: &[Polygon],
    reversed: bool,
    h: &Heights,
    link_threshold: f64,
) -> Point {
    let ri = crate::profile::rotate_to_start(pts, Some([prev_end.x, prev_end.y]));
    let start = ri[0];
    let tan_in = crate::profile::start_tangent(&ri);
    let cin = cleared_normal(tan_in, reversed);
    let (loop_pts, exit_on, tan_out) = crate::emit::loop_with_overlap(&ri, lead_overlap);
    let cout = cleared_normal(tan_out, reversed);

    // Drop any lead that would swing past the far wall down to a plain pass.
    let eff_in = crate::leads::guard_lead(guard, start, tan_in, cin, lead_in, true);
    let eff_out = crate::leads::guard_lead(guard, exit_on, tan_out, cout, lead_out, false);
    let entry = crate::leads::lead_start_point(start, tan_in, cin, eff_in);
    let exit = crate::leads::lead_end_point(exit_on, tan_out, cout, eff_out);

    let lead = Tag::new(id, MoveKind::LeadIn);
    let cut = Tag::new(id, MoveKind::Cutting);
    approach(prog, prev_end, entry, from_z, z, h, feed, plunge_feed, id, link_threshold);
    crate::leads::emit_lead(prog, entry, start, start, cin, eff_in, z, feed, lead);
    crate::emit::cut_polyline(prog, &loop_pts, feed, cut, z);
    crate::leads::emit_lead(prog, exit_on, exit, exit_on, cout, eff_out, z, feed, lead);
    exit
}

/// Emit a **stay-down, inside-out** pocket path. Per depth level: enter once at the
/// innermost ring, then cut every ring, **linking at depth** to the next ring when
/// the hop is short (adjacent rings), and only **lifting** when the hop is too long
/// to cut across (an island loop or a pinched-off lobe). The finished-wall rings
/// (`is_wall`) are eased on/off with the leads. Between levels the tool retracts to
/// the interior start and replunges, so every level is carved the same way (wall
/// last). Every level's entry uses the op's plunge strategy (into cleared stock for
/// levels after the first).
///
/// `rings` must be ordered **inner-first** (innermost at index 0, walls last).
/// `link_threshold` is the hop above which a stay-down link would cut across uncut
/// material, so we lift instead. `guard` is the cleared region (the area bounded by
/// the wall rings); a wall lead that would overshoot it is dropped to a plain pass.
/// `reversed` is `true` when the caller has flipped the loops for conventional
/// milling, so the wall leads sit on the (now right-of-travel) cleared side.
#[allow(clippy::too_many_arguments)]
pub(crate) fn emit_stay_down(
    prog: &mut Program,
    rings: &[Ring],
    start: Option<[f64; 2]>,
    id: u32,
    feed: f64,
    plunge_feed: f64,
    plunge: Plunge,
    lead_overlap: f64,
    lead_in: Lead,
    lead_out: Lead,
    guard: &[Polygon],
    reversed: bool,
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
    let leaded = lead_in != Lead::None || lead_out != Lead::None;

    let mut from_z = h.top_of_stock;
    for &z in levels {
        // Enter at the innermost ring (a boundary loop, so the plunge strategy's
        // helix/ramp is placed on the inward, cleared side), per the operator's
        // start preference. The descent after the first level is one stepdown into
        // stock cleared above.
        let r0 = crate::profile::rotate_to_start(&rings[0].pts, start);
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
            if ring.is_wall && leaded {
                prev_end = emit_wall_ring(
                    prog,
                    &ring.pts,
                    prev_end,
                    from_z,
                    z,
                    id,
                    feed,
                    plunge_feed,
                    lead_overlap,
                    lead_in,
                    lead_out,
                    guard,
                    reversed,
                    h,
                    link_threshold,
                );
                continue;
            }
            // Begin this ring at the point nearest where the last one ended, so the
            // hop is the shortest possible — a clean radial step for adjacent rings.
            let ri = crate::profile::rotate_to_start(&ring.pts, Some([prev_end.x, prev_end.y]));
            approach(
                prog,
                prev_end,
                ri[0],
                from_z,
                z,
                h,
                feed,
                plunge_feed,
                id,
                link_threshold,
            );
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
