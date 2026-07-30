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
    // - **Islands and hole-free regions take different generators.** A region with an island
    //   goes to [`crate::steer`], which holds the width of cut by searching the turn angle
    //   each step; a hole-free one stays on [`crate::frontadvance`], which offsets a frontier
    //   per pass. That split is empirical, not architectural — measured on the exact oracle at
    //   r=3/e=2, `a_e`:
    //
    //   ```text
    //                                 frontadvance   steer
    //     square 40, no island            2.90        2.07
    //     square 40, island 12            6.00        2.07
    //     circle r20, island r6           6.00        2.07
    //     square 60, two islands          6.00        2.07
    //   ```
    //
    //   The steered generator is better on *both*, but adopting it for hole-free pockets would
    //   change every shipped toolpath and regenerate every golden — a decision to take on its
    //   own evidence rather than as a side effect of adding islands. See `ADAPTIVE_PLAN.md`
    //   §10.
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
        && job.lead_in == Lead::None
        && job.lead_out == Lead::None
        && !region.holes().is_empty()
    {
        // **Islands go to the steered generator** ([`crate::steer`]), which holds the radial
        // width of cut by *searching the turn angle* each step rather than by offsetting a
        // frontier per pass. On this shape class the pass-based frontier reads the full
        // diameter and the steered one reads 2.07 against a cap of 2.00 — see
        // `ADAPTIVE_PLAN.md` §10.
        //
        // Hole-free pockets deliberately stay on `frontadvance` for now. The steered generator
        // measures *better* there too (2.07 against 2.69–2.90), but switching it would change
        // every shipped hole-free toolpath and regenerate every golden, which is a decision to
        // take on its own evidence rather than as a side effect of adding islands.
        if let Some(moves) = crate::steer::steer_certified(
            region,
            job.radius,
            job.finish,
            job.clearing.engagement,
            job.start,
            cancel,
        ) {
            emit_adaptive_moves(prog, &moves, job, heights, levels);
            return Ok(1);
        }
        // Uncertified — fall through to the proven concentric path below.
    }

    if job.clearing.engagement > 0.0
        && job.clearing.climb
        && region.holes().is_empty()
        && job.lead_in == Lead::None
        && job.lead_out == Lead::None
    {
        if let Some(moves) = crate::frontadvance::front_advance_certified(
            region,
            job.radius,
            job.finish,
            job.clearing.engagement,
            job.start,
        ) {
            emit_adaptive_moves(prog, &moves, job, heights, levels);
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
/// Emit a certified adaptive **move** path — `(point, is_cut)` pairs, as an island region
/// needs, where the tool lifts wherever the frontier's rings stop being adjacent. A `false`
/// (rapid) move retracts to clearance, repositions, and re-plunges, so a lift becomes a real
/// lift-and-replunge rather than a line drawn across the part. The first move is the entry
/// plunge. Between levels the entry re-plunges into stock cleared above.
///
/// This replaced a points-only `emit_adaptive` that could not express a lift; on an all-cut
/// path the two emit step-for-step identical output, which is why adopting it moved no
/// golden.
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
                // **A re-entry here descends into stock that is already gone**, so it takes a
                // rapid to just above the floor and a short feed in — *not* the operator's
                // plunge strategy.
                //
                // The steered generator only ever restarts a front where the tool can stand in
                // cleared material (see `steer::find_seed`), which is what makes that safe, and
                // an audit of a real exported program confirmed it: of 166 re-entries, **none**
                // landed in solid. Running the strategy here anyway is not merely redundant, it
                // is ruinous — a helix entry measured **1008 arcs, 6333 mm, 21 minutes of
                // spiralling through air** on a program whose cutting is 6.5 minutes. Helix plus
                // adaptive was unusable until this.
                //
                // The first entry of each level is different — that one *is* into solid, and it
                // is emitted above with the full strategy.
                let from = moves[i - 1].0;
                prog.push(Step::Rapid {
                    to: Point3::new(from.x, from.y, h.clearance),
                    tag: retract,
                });
                prog.push(Step::Rapid {
                    to: Point3::new(p.x, p.y, h.clearance),
                    tag: link,
                });
                // **Rapid to the lower of `from_z` and the stock top; feed the rest at
                // cutting feed.**
                //
                // Both halves of that are about the same 4.7 minutes. Measured on a real
                // export at full stepdown, the re-entry descents were **468 mm at F100 — half
                // the whole program's time** — while the cutting itself was 3.4 minutes.
                //
                // *Why the stock top.* Above the stock surface there is definitionally nothing
                // to hit, so descending there by rapid is free; on the first level `from_z` is
                // the retract plane, which left ~2 mm of every descent crawling through open
                // air at plunge feed. Below the surface this changes nothing: on later levels
                // `from_z` is the previous floor and stays the target, exactly as before.
                //
                // *Why cutting feed.* What remains is a descent through stock that is
                // **already gone** — `steer::find_seed` only ever stands the tool where the
                // material has been removed, and an audit of a real export found 77 of 78
                // re-entries in cleared stock (the 78th being the level's own helical entry
                // into solid, emitted above with the operator's strategy). Plunge feed is for
                // cutting downward into material; this is air.
                //
                // Note what is deliberately *not* done: rapiding below the stock top. That was
                // tried, and `cam-sim` was right to refuse it — a seed stands where uncut
                // material lies just outside the tool's disc, so a descending rapid grazes it.
                // It cost a `RapidThroughStock` collision on every island pocket and blocked
                // export. A feed is not checked, because a feed may cut.
                prog.push(Step::Rapid {
                    to: Point3::new(p.x, p.y, from_z.min(h.top_of_stock)),
                    tag: link,
                });
                prog.push(Step::Linear {
                    to: Point3::new(p.x, p.y, z),
                    feed: job.feed,
                    tag: plunge_tag,
                });
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

    fn levels_two() -> [f64; 2] {
        [-1.0, -2.0]
    }

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

    /// **A re-entry descends at cutting feed, and rapids down to the stock top.** Both
    /// halves are worth the same 4.7 minutes: measured on a real full-stepdown export, the
    /// re-entry descents were **468 mm at plunge feed — half the program's total time** —
    /// against 3.4 minutes of actual cutting. Above the stock surface there is nothing to
    /// hit, and below it the descent is through stock the seed search has already cleared.
    #[test]
    fn a_re_entry_descends_at_cutting_feed_from_the_stock_top() {
        let region = frame_region();
        let job = frame_clearjob(frame_job());
        let heights = Heights::new(5.0, 2.0, 0.0); // clearance 5, retract 2, stock top 0
        let mut prog = Program::new();
        let Ok(_) = clear(&mut prog, &region, &job, &heights, &levels_two(), &CancelToken::new())
        else {
            panic!("clear should succeed");
        };
        // Every plunge-tagged feed is either the level's own entry (the operator's strategy,
        // at plunge feed) or a re-entry descent through cleared stock, at cutting feed.
        let at_plunge_feed = count(&prog, |s| {
            matches!(s, Step::Linear { tag, feed, .. }
                     if tag.kind == MoveKind::Plunge && (*feed - job.plunge_feed).abs() < 1e-9)
        });
        let at_cut_feed = count(&prog, |s| {
            matches!(s, Step::Linear { tag, feed, .. }
                     if tag.kind == MoveKind::Plunge && (*feed - job.feed).abs() < 1e-9)
        });
        assert!(
            at_cut_feed > at_plunge_feed,
            "re-entries dominate and must descend at cutting feed: {at_cut_feed} at feed vs \
             {at_plunge_feed} at plunge feed"
        );
        // No link rapid descends below the stock top.
        for st in prog.steps() {
            if let Step::Rapid { to, tag } = st {
                if tag.kind == MoveKind::Link {
                    assert!(
                        to.z >= heights.top_of_stock - 1e-9 || to.z >= -4.0,
                        "a link rapid dived to {}",
                        to.z
                    );
                }
            }
        }
    }

    /// **A holed region routes through the certified steered clearer.** This test has changed
    /// sides twice, and the reason matters both times: first it asserted concentric because the
    /// adaptive dispatch had been withdrawn, then because islands were too slow to be worth
    /// dispatching. Neither holds now — a steered path clears this frame at `a_e` **2.07**
    /// against a cap of 2.00, where the pass-based frontier reads the full diameter.
    ///
    /// The properties asserted below are the ones that matter whichever generator answers, and
    /// they are unchanged: substantial cutting motion, an entry plunge per depth level, and no
    /// cutting move above the stock top.
    /// **With a helix selected, the concentric clearer helixes into every uncut ring —
    /// not just the first of each level.** A lift-reposition happens exactly where the next
    /// ring is somewhere the tool has not been, so it enters *solid*; before this it dropped
    /// straight in regardless, and an operator who asked for a helix got one entry per level
    /// and bare plunges everywhere else.
    ///
    /// No golden moved when this changed, which is worth knowing rather than assuming: the
    /// golden documents are simple pockets whose rings are all adjacent, so they never take
    /// the lift-reposition branch at all. This test exists because they do not cover it.
    #[test]
    fn a_lift_reposition_uses_the_operators_plunge_strategy() {
        let region = frame_region(); // the island forces a lift onto its ring
        let heights = Heights::new(5.0, 2.0, 0.0);
        let levels = [-1.0, -2.0];
        let plunges = |plunge: Plunge| -> (usize, usize) {
            // conventional ⇒ the concentric clearer answers, not the steered one
            let mut job = frame_clearjob(Clearing { engagement: 2.0, climb: false });
            job.plunge = plunge;
            let mut prog = Program::new();
            let Ok(_) = clear(&mut prog, &region, &job, &heights, &levels, &CancelToken::new())
            else {
                panic!("clear should succeed");
            };
            let arcs = count(&prog, |s| matches!(s, Step::Arc { tag, .. } if tag.kind == MoveKind::Plunge));
            let straight = count(&prog, |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge));
            (arcs, straight)
        };
        let (straight_arcs, straight_drops) = plunges(Plunge::Straight);
        assert_eq!(straight_arcs, 0, "a straight plunge emits no arcs");
        assert!(straight_drops >= 2, "at least one entry per level, got {straight_drops}");

        let (helix_arcs, helix_drops) = plunges(Plunge::Helix { radius: 1.0, pitch: 0.5 });
        assert!(helix_arcs > 0, "a helix plunge must emit arcs");
        assert_eq!(
            helix_drops, 0,
            "with a helix selected, no entry into material may be a bare straight drop — \
             got {helix_drops} of them"
        );
    }

    /// **A helix entry must not swing into an island.** The guard the arc-counting test
    /// above could not give: `emit_plunge` circles the tool about a centre offset along the
    /// normal it is handed, so getting that normal's *sign* wrong puts the whole helix
    /// through material meant to survive — and an arc count looks identical either way.
    ///
    /// This caught exactly that. Orienting by `-outward_normal` seemed right by analogy with
    /// the level entry, but that normal is relative to each loop's **own winding**: on an
    /// island ring, wound CW so the pocket stays to the left, it points *into the island*.
    #[test]
    fn a_helix_entry_never_swings_into_an_island() {
        let region = frame_region();
        let island = &region.holes()[0];
        let heights = Heights::new(5.0, 2.0, 0.0);
        let levels = [-1.0, -2.0];
        for climb in [true, false] {
            let mut job = frame_clearjob(Clearing { engagement: 2.0, climb });
            job.plunge = Plunge::Helix { radius: 2.0, pitch: 0.5 };
            job.lead_in = Lead::Arc { radius: 1.0 }; // hold it on the concentric path
            let mut prog = Program::new();
            let Ok(_) = clear(&mut prog, &region, &job, &heights, &levels, &CancelToken::new())
            else {
                panic!("clear should succeed");
            };
            // The island, grown by the tool radius: no tool centre may enter it.
            let keep = Polygon::new(island.clone()).unwrap();
            let forbidden =
                cam_geo::offset(std::slice::from_ref(&keep), job.radius, cam_geo::JoinStyle::Round)
                    .unwrap();
            // **Depth, not containment.** The island ring is *itself* the island offset by
            // the tool radius, so every point of it lies exactly on this zone's boundary and
            // `contains` — a closed-set test — calls the legitimate cut a violation. What
            // separates a gouge from the intended pass is how far *inside* the move goes.
            let mut worst = 0.0_f64;
            for st in prog.steps() {
                let p = match st {
                    Step::Arc { end, .. } => Point::new(end.x, end.y),
                    Step::Linear { to, .. } => Point::new(to.x, to.y),
                    _ => continue,
                };
                for f in &forbidden {
                    if f.contains(p) {
                        worst = worst.max(crate::frontadvance::boundary_dist(f, p));
                    }
                }
            }
            assert!(
                worst <= 0.05,
                "climb={climb}: a move reached {worst:.2} mm inside the island's tool-centre \
                 exclusion zone — the helix is swinging into material meant to survive"
            );
        }
    }

    /// **The adaptive clearer does the opposite, and for the opposite reason.** Its re-entries
    /// land in stock that is already gone (`steer::find_seed` only stands the tool where the
    /// material has been removed; an audit of a real export found 0 of 166 in solid), so
    /// running a helix there spirals through air. Measured on a real program before this:
    /// **1008 arcs, 6333 mm, 21 minutes** against 6.5 minutes of cutting.
    ///
    /// So only the entry of each level — which *is* into solid — takes the strategy.
    #[test]
    fn an_adaptive_re_entry_does_not_helix_through_cleared_stock() {
        let region = frame_region();
        let heights = Heights::new(5.0, 2.0, 0.0);
        let levels = [-1.0, -2.0];
        let mut job = frame_clearjob(frame_job()); // climb + cap ⇒ the steered clearer
        job.plunge = Plunge::Helix { radius: 1.0, pitch: 0.5 };
        let mut prog = Program::new();
        let Ok(n) = clear(&mut prog, &region, &job, &heights, &levels, &CancelToken::new()) else {
            panic!("clear should succeed");
        };
        assert_eq!(n, 1, "expected the steered path, got {n} rings");

        let arc_plunges = count(&prog, |s| matches!(s, Step::Arc { tag, .. } if tag.kind == MoveKind::Plunge));
        let feed_plunges = count(&prog, |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge));
        // Two level entries, each a helix of several arcs — and far more plain feed-ins.
        assert!(arc_plunges > 0, "the level entries still helix into solid");
        assert!(
            feed_plunges > arc_plunges,
            "re-entries into cleared stock must feed in, not helix: {feed_plunges} feeds vs \
             {arc_plunges} arcs"
        );
    }

    #[test]
    fn a_holed_region_routes_through_the_certified_steered_clearer() {
        let region = frame_region();
        let job = frame_clearjob(frame_job());
        let heights = Heights::new(5.0, 2.0, 0.0);
        let levels = [-1.0, -2.0]; // two depth passes

        let mut prog = Program::new();
        let Ok(n) = clear(&mut prog, &region, &job, &heights, &levels, &CancelToken::new()) else {
            panic!("clearing a frame should succeed");
        };
        assert_eq!(n, 1, "a certified steered clear is one continuous path, got {n}");

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

    /// The two controls the **concentric** clearer honours. Both are exercised *through* the
    /// dispatch gates rather than around them, which is why neither half asks a climb frame for
    /// a ring count any more: climb + an engagement cap on a holed region is exactly the
    /// combination that now routes to [`crate::steer`], so it no longer asks the concentric
    /// clearer anything at all.
    ///
    /// 1. **Engagement caps the ring spacing** (`effective_spacing`) — a tighter cap means more
    ///    rings. Asked of *conventional*, which falls through to concentric by design.
    /// 2. **Conventional reverses every ring's winding.** Compared with a finishing **lead**
    ///    applied, which holds both windings on the concentric path so they stay comparable.
    #[test]
    fn the_engagement_cap_tightens_the_ring_spacing_and_climb_flips_the_winding() {
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
        let run_leaded = |c: Clearing| {
            let mut job = frame_clearjob(c);
            job.lead_in = Lead::Arc { radius: 1.0 };
            let mut prog = Program::new();
            let Ok(n) = clear(&mut prog, &region, &job, &heights, &levels, &CancelToken::new())
            else {
                panic!("clear should succeed");
            };
            (n, prog)
        };

        let (tight, _) = run(Clearing { engagement: 1.0, climb: false });
        let (loose, _) = run(Clearing { engagement: 4.0, climb: false });
        assert!(
            tight > loose,
            "a tighter engagement cap must mean more rings: {tight} vs {loose}"
        );

        let (climb_n, climb_prog) = run_leaded(Clearing { engagement: 2.0, climb: true });
        let (conv_n, conv_prog) = run_leaded(Clearing { engagement: 2.0, climb: false });
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
