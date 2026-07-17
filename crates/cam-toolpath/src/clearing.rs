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
