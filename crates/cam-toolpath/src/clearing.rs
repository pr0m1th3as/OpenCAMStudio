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
    // **Adaptive dispatch: the front-advance clearer, behind the exact oracle.**
    //
    // When an engagement cap is set and the cut is climb, try [`crate::frontadvance`]: it
    // advances the cleared frontier one stepover per pass, so the tool-centre path holds
    // engagement at the geometric floor (2.69–2.90 on r=3/e=2 — no slots) where the
    // concentric path slots at the entry, the pass-to-pass links, and the concave corner.
    // The path self-certifies against the **exact** oracle ([`crate::clearsim`]) and
    // returns `None` — meaning fall through to concentric — whenever it cannot.
    //
    // Three gates before we even try, all by construction:
    //
    // - **Climb only.** Adaptive clearing inherits the offset winding = climb; there is no
    //   radial-order-preserving rotation flip, and conventional defeats constant-engagement
    //   HSM anyway. Conventional falls through to concentric, which honours it.
    // - **Simply connected only.** A region with islands has no single seam; the frontier
    //   flowing around holes is a later increment. `front_advance_certified` returns `None`
    //   for a holed region regardless, but gating here keeps the intent legible.
    // - **No finishing leads.** Front-advance is a *roughing* engine: it emits its frontier
    //   spiral with no wall lead-on, because a lead onto a spiral loop has no clean
    //   wall-hugging ring to ease onto. If the operator asked for a finishing lead
    //   (`lead_in`/`lead_out`), the concentric path lays it on the wall ring and we must not
    //   silently drop it — so a leaded pocket keeps the proven concentric clear. (Profile
    //   roughing always passes `Lead::None`, so it is never held back by this.) A proper
    //   front-advance-rough + separate leaded finish-wall pass is a later increment.
    //
    // **The gate is the exact oracle, never the raster.** Until 2026-07-17 an earlier
    // adaptive dispatch (the spiral-morph) gated on `raster.rs`, which is blind to slots —
    // measured, raster verdict vs the exact oracle on the same *emitted* path, r=3,
    // engagement 2.0:
    //
    // ```text
    //   square 40 : raster 2.20 → shipped   exact 6.00   ← the full diameter
    //   square 24 : raster 0.80 → shipped   exact 6.00   ← the full diameter
    // ```
    //
    // It shipped full-diameter cuts at whatever axial depth the level set. The raster read
    // 0.80 against a true 6.00 — a 7.5× under-read in the unsafe direction. It stays out of
    // this path until it is re-anchored against the exact oracle.
    //
    // The engagement value is the **straight-wall stepover**, not a hard ceiling: the
    // geometric floor `a_e(ρ) = e(ρ+r)/ρ − e²/(2ρ)` exceeds `e` at every finite radius and
    // reaches ~1.4·e on the tight loops near a pocket centre, unreachable by any spiral
    // clearer. Front-advance kills the *slots* (the machine-and-tool hazard); the benign
    // floor is a per-loop feedrate matter. See [`crate::frontadvance::CERT_ENGAGEMENT_SLACK`].
    if job.clearing.engagement > 0.0
        && job.clearing.climb
        && region.holes().is_empty()
        && job.lead_in == Lead::None
        && job.lead_out == Lead::None
    {
        if let Some(tc) = crate::frontadvance::front_advance_certified(
            region,
            job.radius,
            job.finish,
            job.clearing.engagement,
            job.start,
        ) {
            emit_adaptive(prog, &tc, job, heights, levels);
            // One continuous certified path (not a ring count); `Ok(_)` ⇒ emitted.
            return Ok(1);
        }
        // Uncertified — fall through to the proven concentric path below.
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

    let mut from_z = h.retract.max(h.top_of_stock);
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
#[allow(dead_code)] // the emitter the front-advance will use once frames/islands land
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

    let mut from_z = h.retract.max(h.top_of_stock);
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

    /// A 60×60 pocket with a 20×20 island — the frame the adaptive clearer certifies.
    fn frame_region() -> Polygon {
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
        Polygon::with_holes(outer, vec![island]).unwrap()
    }

    fn frame_clearjob(clearing: Clearing) -> ClearJob<'static> {
        ClearJob {
            id: 7,
            radius: 3.0,
            finish: 0.0,
            first: 3.0,
            spacing: 4.0,
            clearing,
            plunge: Plunge::Straight,
            feed: 300.0,
            plunge_feed: 100.0,
            lead_overlap: 0.0,
            lead_in: Lead::None,
            lead_out: Lead::None,
            start: None,
            guard: &[],
        }
    }

    #[test]
    fn a_holed_region_routes_through_the_certified_frame_clearer() {
        // A pocket with an island + an engagement cap clears **concentrically**, per
        // depth level. It used to route through the adaptive frame path; that path was
        // raster-gated and the raster is blind to slots (see `clear`), so the adaptive
        // dispatch is gone and everything takes the proven concentric path until
        // `frontadvance` is wired in behind the exact oracle.
        let region = frame_region();
        let job = frame_clearjob(frame_job());
        let heights = Heights::new(5.0, 2.0, 0.0);
        let levels = [-1.0, -2.0]; // two depth passes

        let mut prog = Program::new();
        let Ok(n) = clear(&mut prog, &region, &job, &heights, &levels, &CancelToken::new()) else {
            panic!("clearing a frame should succeed");
        };
        assert!(n > 1, "a frame clears as concentric rings, got {n}");

        let cuts = count(&prog, |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Cutting));
        let plunges = count(&prog, |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge));
        assert!(cuts > 50, "the frame has substantial cutting motion, got {cuts}");
        assert!(plunges >= 2, "at least one entry plunge per level, got {plunges}");
        // Every cutting move stays at a level depth (never above the stock top).
        for s in prog.steps() {
            if let Step::Linear { to, tag, .. } = s {
                if tag.kind == MoveKind::Cutting {
                    assert!(to.z <= heights.top_of_stock + 1e-9, "a cut rose above stock: z={}", to.z);
                }
            }
        }
    }

    #[test]
    fn the_engagement_cap_tightens_the_ring_spacing_and_climb_flips_the_winding() {
        // The two controls the clearing engine still honours, now that the adaptive
        // dispatch is gone (see `clear`):
        //
        // 1. **Engagement caps the ring spacing** (`effective_spacing`), which genuinely
        //    bounds the radial width of cut on a straight wall — a tighter cap means more
        //    rings. This is what the engagement parameter honestly buys today; what it
        //    does *not* buy is a bound around corners or at entry.
        // 2. **Conventional reverses every ring's winding.**
        let region = frame_region();
        let heights = Heights::new(5.0, 2.0, 0.0);
        let levels = [-1.0];
        let run = |c: Clearing| {
            let mut prog = Program::new();
            let Ok(n) = clear(&mut prog, &region, &frame_clearjob(c), &heights, &levels,
                &CancelToken::new()) else {
                panic!("clear should succeed");
            };
            (n, prog)
        };

        // A tight cap must produce strictly more rings than a loose one.
        let (tight, _) = run(Clearing { engagement: 1.0, climb: true });
        let (loose, _) = run(Clearing { engagement: 4.0, climb: true });
        assert!(
            tight > loose,
            "a tighter engagement cap must mean more rings: {tight} vs {loose}"
        );

        // Climb and conventional both clear concentrically; they differ in winding.
        let (climb_n, climb_prog) = run(Clearing { engagement: 2.0, climb: true });
        let (conv_n, conv_prog) = run(Clearing { engagement: 2.0, climb: false });
        assert_eq!(climb_n, conv_n, "same rings either way; only the winding differs");

        // Signed area of the cutting moves: opposite senses.
        let signed = |prog: &Program| -> f64 {
            let pts: Vec<Point3> = prog
                .steps()
                .iter()
                .filter_map(|s| match s {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(*to),
                    _ => None,
                })
                .collect();
            pts.windows(2).map(|w| w[0].x * w[1].y - w[1].x * w[0].y).sum()
        };
        let (sc, sv) = (signed(&climb_prog), signed(&conv_prog));
        assert!(
            sc * sv < 0.0,
            "climb and conventional must wind opposite ways, got {sc:.1} and {sv:.1}"
        );
    }
}
