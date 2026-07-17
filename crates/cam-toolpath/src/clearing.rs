//! Shared 2.5-D area clearing.
//!
//! Both the pocket strategy and profile outside-roughing clear a region with the
//! same engine: concentric offset rings emitted as a **stay-down, inside-out** path
//! (one plunge per level, ring-to-ring links at depth, walls last). Routing them
//! through here gives both the same cut order, linking, and finishing-lead handling,
//! plus the two shared controls:
//!
//! - **Engagement cap** — the ring spacing is capped at the engagement width, so the
//!   radial width of cut on a straight wall never exceeds it. (Corner engagement is
//!   still bounded only by the spacing; flattening the corner spikes with trochoidal
//!   loops is the next step, and slots in behind this same entry point.)
//! - **Climb / conventional** — the rings are wound so climb milling keeps the
//!   cleared side on the left of travel; conventional reverses every loop.

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{Point, Polygon};
use cam_model::{Clearing, Heights, Lead, Plunge};

use crate::rings::{concentric_rings, emit_stay_down, RingsError};
use crate::CancelToken;

/// Everything the clearing pass needs beyond the region geometry itself.
pub(crate) struct ClearJob<'a> {
    /// Operation id, stamped onto emitted tags.
    pub id: u32,
    /// Tool radius.
    pub radius: f64,
    /// Finishing allowance left on the walls.
    pub finish: f64,
    /// Inward offset from the wall to the first (wall-hugging) ring — the tool
    /// radius plus any finishing allowance left on the walls (`radius + finish`).
    pub first: f64,
    /// Nominal ring spacing (radial stepover) before the engagement cap applies.
    pub spacing: f64,
    /// Engagement cap + climb/conventional.
    pub clearing: Clearing,
    /// How the tool enters Z at each level.
    pub plunge: Plunge,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed, mm/min.
    pub plunge_feed: f64,
    /// Re-machine distance past a loop's start before leading/closing off.
    pub lead_overlap: f64,
    /// Finishing lead onto the walls (pocket only; roughing passes `None`).
    pub lead_in: Lead,
    /// Finishing lead off the walls.
    pub lead_out: Lead,
    /// Preferred entry point (part XY) for the innermost ring.
    pub start: Option<[f64; 2]>,
    /// Cleared region the wall leads must stay inside (empty ⇒ no lead guard).
    pub guard: &'a [Polygon],
}

impl ClearJob<'_> {
    /// The ring spacing actually used: the engagement cap holds the radial width of
    /// cut at or below itself, so it can only *tighten* the nominal spacing.
    /// `engagement <= 0` leaves the nominal spacing untouched (plain concentric).
    fn effective_spacing(&self) -> f64 {
        if self.clearing.engagement > 0.0 {
            self.clearing.engagement.min(self.spacing)
        } else {
            self.spacing
        }
    }
}

/// Clear `region`: generate the rings, orient them for climb/conventional, and emit
/// the stay-down inside-out path over `levels`. Returns how many rings were produced
/// (`0` ⇒ the tool cannot even enter), or a generation error.
pub(crate) fn clear(
    prog: &mut Program,
    region: &Polygon,
    job: &ClearJob,
    heights: &Heights,
    levels: &[f64],
    cancel: &CancelToken,
) -> Result<usize, RingsError> {
    // Try constant-engagement adaptive clearing first. It self-certifies against the
    // oracle and returns None whenever it cannot — so we simply fall through to the
    // proven concentric path below, and every emitted toolpath is verified correct.
    if job.clearing.engagement > 0.0 {
        if region.holes().is_empty() {
            if let Some(tc) = crate::adaptive::adaptive_path(
                region,
                job.radius,
                job.finish,
                job.clearing.engagement,
                job.start,
            ) {
                emit_adaptive(prog, &tc, job, heights, levels);
                return Ok(1);
            }
        } else if let Some(moves) =
            crate::adaptive::adaptive_frame(region, job.radius, job.finish, job.clearing.engagement)
        {
            // A frame/annulus around an island: certified constant-engagement moves
            // with a lift between the outer spiral and the island loops.
            emit_adaptive_moves(prog, &moves, job, heights, levels);
            return Ok(1);
        }
    }

    let spacing = job.effective_spacing();
    let mut rings = concentric_rings(region, job.first, spacing, cancel)?;
    if rings.is_empty() {
        return Ok(0);
    }
    let n = rings.len();

    // Conventional milling reverses every loop's travel; the wall-lead geometry is
    // told via `reversed` so the leads still sit on the cleared side.
    let reversed = !job.clearing.climb;
    if reversed {
        for r in &mut rings {
            r.pts.reverse();
        }
    }
    // `concentric_rings` yields wall-most first; carve inside-out (walls last).
    rings.reverse();

    emit_stay_down(
        prog,
        &rings,
        job.start,
        job.id,
        job.feed,
        job.plunge_feed,
        job.plunge,
        job.lead_overlap,
        job.lead_in,
        job.lead_out,
        job.guard,
        reversed,
        heights,
        levels,
        1.5 * spacing,
    );
    Ok(n)
}

/// Emit a certified adaptive tool-centre path over the depth levels: per level,
/// approach and enter at the path start (per the plunge strategy), cut the whole
/// path at depth, and retract. Between levels the entry re-plunges into stock
/// cleared above.
fn emit_adaptive(prog: &mut Program, tc: &[Point], job: &ClearJob, h: &Heights, levels: &[f64]) {
    if tc.len() < 2 {
        return;
    }
    let link = Tag::new(job.id, MoveKind::Link);
    let cut = Tag::new(job.id, MoveKind::Cutting);
    let retract = Tag::new(job.id, MoveKind::Retract);
    let plunge_tag = Tag::new(job.id, MoveKind::Plunge);
    let start = tc[0];
    let tan = crate::profile::unit(tc[1].x - start.x, tc[1].y - start.y);
    let out = (-tan.1, tan.0);

    let mut from_z = h.top_of_stock;
    for &z in levels {
        prog.push(Step::Rapid {
            to: Point3::new(start.x, start.y, h.clearance),
            tag: link,
        });
        prog.push(Step::Rapid {
            to: Point3::new(start.x, start.y, from_z),
            tag: link,
        });
        crate::profile::emit_plunge(
            prog,
            start,
            tan,
            out,
            from_z,
            z,
            job.plunge,
            job.plunge_feed,
            job.feed,
            plunge_tag,
        );
        for p in &tc[1..] {
            prog.push(Step::Linear {
                to: Point3::new(p.x, p.y, z),
                feed: job.feed,
                tag: cut,
            });
        }
        let last = tc[tc.len() - 1];
        prog.push(Step::Rapid {
            to: Point3::new(last.x, last.y, h.clearance),
            tag: retract,
        });
        from_z = z;
    }
}

/// Emit a certified adaptive **move** path — `(point, is_cut)` pairs, as a frame needs
/// where the tool lifts between the outer spiral and the island loops. Like
/// [`emit_adaptive`] but honouring the cut flag: a `false` (rapid) move retracts to
/// clearance, repositions, and re-plunges, so a lift becomes a real lift-and-replunge.
/// The first move is the entry plunge. Between levels the entry re-plunges into stock
/// cleared above.
fn emit_adaptive_moves(
    prog: &mut Program,
    moves: &[(Point, bool)],
    job: &ClearJob,
    h: &Heights,
    levels: &[f64],
) {
    if moves.len() < 2 {
        return;
    }
    let link = Tag::new(job.id, MoveKind::Link);
    let cut = Tag::new(job.id, MoveKind::Cutting);
    let retract = Tag::new(job.id, MoveKind::Retract);
    let plunge_tag = Tag::new(job.id, MoveKind::Plunge);

    // Plunge tangent at move `i`: direction toward the next point that actually moves.
    let tan_at = |i: usize| -> (f64, f64) {
        let a = moves[i].0;
        for &(b, _) in &moves[i + 1..] {
            let d = crate::profile::unit(b.x - a.x, b.y - a.y);
            if d != (0.0, 0.0) {
                return d;
            }
        }
        (1.0, 0.0)
    };

    let mut from_z = h.top_of_stock;
    for &z in levels {
        let start = moves[0].0;
        let tan = tan_at(0);
        let out = (-tan.1, tan.0);
        prog.push(Step::Rapid {
            to: Point3::new(start.x, start.y, h.clearance),
            tag: link,
        });
        prog.push(Step::Rapid {
            to: Point3::new(start.x, start.y, from_z),
            tag: link,
        });
        crate::profile::emit_plunge(
            prog, start, tan, out, from_z, z, job.plunge, job.plunge_feed, job.feed, plunge_tag,
        );

        for i in 1..moves.len() {
            let (p, is_cut) = moves[i];
            if is_cut {
                prog.push(Step::Linear {
                    to: Point3::new(p.x, p.y, z),
                    feed: job.feed,
                    tag: cut,
                });
            } else {
                // A lift between families: retract where we stopped, rapid over, and
                // re-plunge into fresh material at the next family's start.
                let from = moves[i - 1].0;
                prog.push(Step::Rapid {
                    to: Point3::new(from.x, from.y, h.clearance),
                    tag: retract,
                });
                prog.push(Step::Rapid {
                    to: Point3::new(p.x, p.y, h.clearance),
                    tag: link,
                });
                prog.push(Step::Rapid {
                    to: Point3::new(p.x, p.y, from_z),
                    tag: link,
                });
                let t2 = tan_at(i);
                let o2 = (-t2.1, t2.0);
                crate::profile::emit_plunge(
                    prog, p, t2, o2, from_z, z, job.plunge, job.plunge_feed, job.feed, plunge_tag,
                );
            }
        }
        let last = moves[moves.len() - 1].0;
        prog.push(Step::Rapid {
            to: Point3::new(last.x, last.y, h.clearance),
            tag: retract,
        });
        from_z = z;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_geo::Contour;

    fn frame_job() -> Clearing {
        Clearing { engagement: 2.0, climb: true }
    }

    fn count(prog: &Program, pred: impl Fn(&Step) -> bool) -> usize {
        prog.steps().iter().filter(|s| pred(s)).count()
    }

    #[test]
    fn a_holed_region_routes_through_the_certified_frame_clearer() {
        // A pocket with an island (60×60 with a 20×20 island) + an engagement cap must
        // route through the adaptive frame path — cutting motion, and a lift/replunge
        // between the outer spiral and the island loops — replicated per depth level.
        let outer = Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(60.0, 0.0),
            Point::new(60.0, 60.0),
            Point::new(0.0, 60.0),
        ]);
        let island = Contour::new(vec![
            Point::new(20.0, 20.0),
            Point::new(40.0, 20.0),
            Point::new(40.0, 40.0),
            Point::new(20.0, 40.0),
        ]);
        let region = Polygon::with_holes(outer, vec![island]).unwrap();
        let job = ClearJob {
            id: 7,
            radius: 3.0,
            finish: 0.0,
            first: 3.0,
            spacing: 4.0,
            clearing: frame_job(),
            plunge: Plunge::Straight,
            feed: 300.0,
            plunge_feed: 100.0,
            lead_overlap: 0.0,
            lead_in: Lead::None,
            lead_out: Lead::None,
            start: None,
            guard: &[],
        };
        let heights = Heights::new(5.0, 2.0, 0.0);
        let levels = [-1.0, -2.0]; // two depth passes

        let mut prog = Program::new();
        let Ok(n) = clear(&mut prog, &region, &job, &heights, &levels, &CancelToken::new()) else {
            panic!("clearing a frame should succeed");
        };
        assert_eq!(n, 1, "the adaptive frame path is a single certified path");

        let cuts = count(&prog, |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Cutting));
        let plunges = count(&prog, |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge));
        assert!(cuts > 50, "the frame has substantial cutting motion, got {cuts}");
        // Per level: one entry plunge + at least one inter-family replunge ⇒ ≥ 2 each,
        // over two levels ⇒ ≥ 4 plunges total.
        assert!(plunges >= 4, "entry + inter-family replunge per level, got {plunges}");
        // Every cutting move stays at a level depth (never above the stock top).
        for s in prog.steps() {
            if let Step::Linear { to, tag, .. } = s {
                if tag.kind == MoveKind::Cutting {
                    assert!(to.z <= heights.top_of_stock + 1e-9, "a cut rose above stock: z={}", to.z);
                }
            }
        }
    }
}
