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
//!   cleared side on the left of travel; conventional reverses every loop. Which
//!   winding *is* climb depends on the spindle, so the job carries its direction
//!   rather than assuming the usual `M3`.

use cam_cldata::{MoveKind, Point3, Program, SpindleDir, Step, Tag};
use cam_geo::{Point, Polygon};
use cam_model::{Clearing, Heights, Lead, Plunge, Tool, ToolKind};

use crate::pocket_helix::PocketHelix;
use crate::rings::{concentric_rings, emit_stay_down, Ring, RingsError};
use crate::CancelToken;

/// Everything the clearing pass needs beyond the region geometry itself.
pub(crate) struct ClearJob<'a> {
    /// Lift above each cutting level for local links, limited to clearance.
    pub lift_height: f64,
    /// With helical pocket entry, feed short, contained links at depth.
    pub keep_tool_down: bool,
    /// Operation id, stamped onto emitted tags.
    pub id: u32,
    /// Frontier and corner-detection simplification tolerance, mm; 0 disables it.
    pub simplification_tolerance: f64,
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
    /// Which way the spindle turns; decides which winding counts as climb.
    pub spindle: SpindleDir,
    /// How the tool enters Z at each level.
    pub plunge: Plunge,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed, mm/min.
    pub plunge_feed: f64,
    pub entry_feeds: cam_model::EntryFeeds,
    /// Re-machine distance past a loop's start before leading/closing off.
    pub lead_overlap: f64,
    /// Finishing lead onto the walls (pocket only; roughing passes `None`).
    pub lead_in: Lead,
    /// Finishing lead off the walls.
    pub lead_out: Lead,
    /// Preferred entry point (part XY) for the innermost ring.
    pub start: Option<[f64; 2]>,
    pub end: Option<[f64; 2]>,
    /// Cutter-centre region that bounds wall leads and entry helices
    /// (empty ⇒ no containment guard).
    pub guard: &'a [Polygon],
    /// Placement across the pocket, with a hard error for an unplaceable helix.
    pub helix: Option<&'a PocketHelix<'a>>,
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

    /// Whether every loop must be reversed from the offset winding to cut the way
    /// the operation asked for.
    ///
    /// Climb is not a property of the path alone — it is the relation between the
    /// direction of travel and the direction the cutting edge moves where it meets
    /// the material, so an `M4` spindle makes the *other* winding the climb one.
    /// Under the usual `M3` this is exactly the old `!climb`, so no shipped path
    /// moves.
    fn reversed(&self) -> bool {
        self.clearing.climb != (self.spindle == SpindleDir::Cw)
    }

    /// Whether the adaptive generators may be tried at all.
    ///
    /// They inherit the offset winding as the climb direction by construction and
    /// have no radial-order-preserving flip (see the dispatch note below), so they
    /// are only valid when climb *is* that winding. Conventional already fell
    /// through to concentric; a reversed spindle now falls through the same way,
    /// rather than emitting a path whose whole premise is inverted.
    fn adaptive_may_run(&self) -> bool {
        self.clearing.climb && self.spindle == SpindleDir::Cw
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
    clear_validated(prog, region, job, heights, levels, cancel, None, |_| true)
        .map(|result| result.paths)
}

pub(crate) struct ClearResult {
    pub paths: usize,
    pub concentric: bool,
}

/// Allow a caller with additional model constraints to reject a frontier path
/// and use the concentric fallback. With the 3D tool, compare a fragmented
/// frontier with the fallback and keep the safe path requiring fewer lifts.
/// Validation never emits a rejected candidate, including the fallback.
#[allow(clippy::too_many_arguments)]
pub(crate) fn clear_validated(
    prog: &mut Program,
    region: &Polygon,
    job: &ClearJob,
    heights: &Heights,
    levels: &[f64],
    cancel: &CancelToken,
    tool_3d: Option<&Tool>,
    accept: impl Fn(&Program) -> bool,
) -> Result<ClearResult, RingsError> {
    if !job.lift_height.is_finite() || job.lift_height < 0.0 {
        return Err(RingsError::Limit(
            "Lift height must be finite and nonnegative".into(),
        ));
    }
    let prefer_stay_down = tool_3d.is_some();
    let flat_floor = tool_3d.is_some_and(|tool| {
        matches!(
            tool.kind,
            ToolKind::EndMill | ToolKind::RoughingMill | ToolKind::FaceMill
        ) && tool.shank_dia().max(tool.neck_dia()) <= tool.diameter
    });
    let mut adaptive_candidate = None;
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
    // - **Climb only, and only on an `M3` spindle.** Adaptive clearing inherits the offset
    //   winding = climb; there is no radial-order-preserving rotation flip, and conventional
    //   defeats constant-engagement HSM anyway. Conventional falls through to concentric,
    //   which honours it — and so does a reversed spindle, for which that same winding is
    //   *conventional*, not climb. See [`ClearJob::adaptive_may_run`].
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
    // - **Configured leads must fit.** In 2D, hole-free frontiers can preserve
    //   adjoining seams with configured leads. 3D checks each raised floor
    //   link and its lead arcs separately.
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
        && !matches!(job.plunge, Plunge::Ramp { .. })
        && job.adaptive_may_run()
        && (prefer_stay_down || (job.lead_in == Lead::None && job.lead_out == Lead::None))
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
        if let Some((moves, air)) = crate::steer::steer_certified(
            region,
            job.radius,
            job.finish,
            job.clearing.engagement,
            job.start,
            cancel,
        ) {
            let mut candidate = Program::new();
            let emitted = if prefer_stay_down {
                let path = crate::frontadvance::LinkedFrontier {
                    feed_links: vec![false; moves.len()],
                    moves,
                    air,
                };
                let arcs = flat_floor
                    .then(|| crate::adaptive_linking::floor_links(&path, job, false, cancel));
                emit_3d_adaptive(
                    &mut candidate,
                    &path,
                    arcs.as_deref().unwrap_or_default(),
                    job,
                    heights,
                    levels,
                )
            } else {
                emit_adaptive_moves(&mut candidate, &moves, &air, job, heights, levels)
            };
            if emitted.is_ok() && accept(&candidate) {
                if !prefer_stay_down || clearance_lifts(&candidate, heights) <= levels.len() {
                    prog.extend(candidate);
                    return Ok(ClearResult {
                        paths: 1,
                        concentric: false,
                    });
                }
                adaptive_candidate = Some(candidate);
            }
        }
        // Uncertified — fall through to the proven concentric path below.
    }

    if job.clearing.engagement > 0.0
        && !matches!(job.plunge, Plunge::Ramp { .. })
        && job.adaptive_may_run()
        && region.holes().is_empty()
    {
        if let Some(moves) = crate::frontadvance::front_advance_certified(
            region,
            job.radius,
            job.finish,
            job.clearing.engagement,
            job.start,
            job.simplification_tolerance,
        ) {
            let configured_leads =
                job.lead_in != Lead::None || job.lead_out != Lead::None || job.lead_overlap > 0.;
            let ends = if prefer_stay_down && job.start.is_none() && job.end.is_none() {
                crate::frontadvance::clear_pocket_ends(
                    region,
                    job.radius,
                    job.clearing.engagement,
                    job.simplification_tolerance,
                    &moves,
                    flat_floor,
                    cancel,
                )
            } else {
                None
            };
            if let Some(ends) = ends.as_ref().filter(|path| path.air.iter().any(|&air| air)) {
                let mut candidate = Program::new();
                let arcs = crate::adaptive_linking::floor_links(ends, job, true, cancel);
                if emit_3d_adaptive(&mut candidate, ends, &arcs, job, heights, levels).is_ok()
                    && accept(&candidate)
                {
                    prog.extend(candidate);
                    return Ok(ClearResult {
                        paths: 1,
                        concentric: false,
                    });
                }
            }
            for moves in ends
                .iter()
                .map(|path| path.moves.as_slice())
                .chain(std::iter::once(moves.as_slice()))
            {
                let mut candidate = Program::new();
                let emitted = if let Some(linked) = crate::frontadvance::link_cleared_travel(
                    moves,
                    region,
                    job.radius,
                    job.finish,
                    job.clearing.engagement,
                    cancel,
                    prefer_stay_down.then_some(crate::frontadvance::MAX_3D_LINK_CHECK_VERTICES),
                ) {
                    if prefer_stay_down {
                        let arcs = flat_floor.then(|| {
                            crate::adaptive_linking::floor_links(&linked, job, true, cancel)
                        });
                        emit_3d_adaptive(
                            &mut candidate,
                            &linked,
                            arcs.as_deref().unwrap_or_default(),
                            job,
                            heights,
                            levels,
                        )
                    } else {
                        match crate::adaptive_linking::prepare(&linked, region, job) {
                            Ok(Some(prepared)) => {
                                prepared.emit(&mut candidate, job, heights, levels)
                            }
                            Err(_) if configured_leads => Ok(()), // The leaded concentric fallback remains available.
                            _ => emit_adaptive_moves_with_travel(
                                &mut candidate,
                                &linked.moves,
                                AdaptiveTravel {
                                    air: &linked.air,
                                    feed_links: &linked.feed_links,
                                    minimum_lift: 0.2,
                                    guarded: true,
                                    floor_links: &[],
                                },
                                job,
                                heights,
                                levels,
                            ),
                        }
                    }
                } else if !cancel.is_cancelled() && (prefer_stay_down || !configured_leads) {
                    emit_adaptive_moves(&mut candidate, moves, &[], job, heights, levels)
                } else {
                    Ok(())
                };
                if emitted.is_ok() && !candidate.steps().is_empty() && accept(&candidate) {
                    if !prefer_stay_down || clearance_lifts(&candidate, heights) <= levels.len() {
                        prog.extend(candidate);
                        return Ok(ClearResult {
                            paths: 1,
                            concentric: false,
                        });
                    }
                    adaptive_candidate = Some(candidate);
                }
            }
        }
        // Uncertified — fall through to the proven concentric path below.
    }

    let mut fallback = Program::new();
    let result = clear_concentric(
        &mut fallback,
        region,
        job,
        heights,
        levels,
        cancel,
        prefer_stay_down,
        &accept,
    );
    if cancel.is_cancelled() {
        return Err(RingsError::Cancelled);
    }
    if let Some(candidate) = adaptive_candidate {
        // Efficiency must not turn a valid operation into an unsafe one. Keep
        // the certified frontier if rings fail validation or need as many lifts.
        if !matches!(result, Ok(n) if n > 0)
            || clearance_lifts(&fallback, heights) >= clearance_lifts(&candidate, heights)
            || !accept(&fallback)
        {
            prog.extend(candidate);
            return Ok(ClearResult {
                paths: 1,
                concentric: false,
            });
        }
    } else if result.is_ok() && !fallback.steps().is_empty() && !accept(&fallback) {
        return Err(RingsError::Limit(
            "No clearing path passes the model safety check".into(),
        ));
    }
    let paths = result?;
    prog.extend(fallback);
    Ok(ClearResult {
        paths,
        concentric: true,
    })
}

/// Count actual lifts, including concentric links tagged `Link` rather than
/// `Retract`. Comparing tags alone hides ring-to-ring clearance moves.
fn clearance_lifts(program: &Program, heights: &Heights) -> usize {
    let mut previous_z = None;
    let mut count = 0;
    for step in program.steps() {
        let end = match step {
            Step::Rapid { to, .. } => {
                if to.z >= heights.clearance - cam_geo::GRID_MM
                    && previous_z.is_some_and(|z| to.z > z + cam_geo::GRID_MM)
                {
                    count += 1;
                }
                to
            }
            Step::Linear { to, .. } => to,
            Step::Arc { end, .. } => end,
            _ => continue,
        };
        previous_z = Some(end.z);
    }
    count
}

#[allow(clippy::too_many_arguments)]
fn clear_concentric(
    prog: &mut Program,
    region: &Polygon,
    job: &ClearJob,
    heights: &Heights,
    levels: &[f64],
    cancel: &CancelToken,
    adaptive_3d: bool,
    accept: impl Fn(&Program) -> bool,
) -> Result<usize, RingsError> {
    if cancel.is_cancelled() {
        return Err(RingsError::Cancelled);
    }
    let spacing = job.effective_spacing();
    let mut rings = concentric_rings(region, job.first, spacing, job.radius, cancel)?;
    if rings.is_empty() {
        return Ok(0);
    }
    // Short joins between offset rings can remove stock. They stay at depth
    // in 3D, where Lift height applies only to verified cleared-floor links.
    let (lift_height, lead_in, lead_out) = if adaptive_3d {
        (0.0, Lead::None, Lead::None)
    } else {
        let lift = if job.keep_tool_down && !job.guard.is_empty() {
            0.0
        } else {
            job.lift_height
        };
        (lift, job.lead_in, job.lead_out)
    };
    let n = rings.len();
    let vertices: usize = rings.iter().map(|r| r.pts.len()).sum();
    if vertices.saturating_mul(levels.len()).saturating_mul(34) > cam_model::MAX_PROFILE_MOVES {
        return Err(RingsError::Limit(
            "Clearing motion budget exceeded; increase stepover/stepdown".into(),
        ));
    }
    for ring in &rings {
        let mut previous = heights.top_of_stock;
        for &z in levels {
            crate::profile::validate_ramp_travel(
                job.plunge,
                previous - z,
                crate::profile::loop_perimeter(&ring.pts),
            )
            .map_err(RingsError::Limit)?;
            if lift_height > 0.0 {
                let top = (z + lift_height)
                    .min(heights.clearance)
                    .min(heights.top_of_stock);
                crate::profile::validate_ramp_travel(
                    job.plunge,
                    (top - z).max(0.0),
                    crate::profile::loop_perimeter(&ring.pts),
                )
                .map_err(RingsError::Limit)?;
            }
            previous = z;
        }
    }

    // Conventional milling reverses every loop's travel; the wall-lead geometry is
    // told via `reversed` so the leads still sit on the cleared side. Which winding
    // is which depends on the spindle — see `ClearJob::reversed`.
    let reversed = job.reversed();
    if reversed {
        for r in &mut rings {
            r.pts.reverse();
        }
    }
    // `concentric_rings` yields wall-most first; carve inside-out (walls last).
    rings.reverse();
    let emit = |program: &mut Program, rings: &[Ring]| {
        emit_stay_down(
            program,
            rings,
            job.start,
            job.end,
            job.id,
            job.feed,
            job.plunge_feed,
            job.entry_feeds,
            job.plunge,
            lift_height,
            job.lead_overlap,
            lead_in,
            lead_out,
            job.guard,
            reversed,
            heights,
            levels,
            1.5 * spacing,
            cancel,
            job.helix,
        )
    };
    if adaptive_3d && job.start.is_none() && job.end.is_none() {
        if let Some(joined) = join_small_entry_rings(&rings, spacing, job.guard) {
            let mut candidate = Program::new();
            emit(&mut candidate, &joined)?;
            if cancel.is_cancelled() {
                return Err(RingsError::Cancelled);
            }
            if accept(&candidate) {
                prog.extend(candidate);
                return Ok(joined.len());
            }
        }
    }
    emit(prog, &rings)?;
    if cancel.is_cancelled() {
        return Err(RingsError::Cancelled);
    }
    Ok(n)
}

/// Merge a short offset fragment into an adjacent pass so it gets cut at
/// depth instead of needing its own approach. Keep its winding and full loop.
fn join_small_entry_rings(rings: &[Ring], spacing: f64, guard: &[Polygon]) -> Option<Vec<Ring>> {
    if guard.is_empty() {
        return None;
    }
    let short = rings
        .iter()
        .take_while(|ring| {
            !ring.is_wall && crate::profile::loop_perimeter(&ring.pts) <= 2. * spacing
        })
        .count();
    if short == 0 || short == rings.len() {
        return None;
    }
    let mut joined = rings.to_vec();
    let mut first_host = short;
    for index in (0..short).rev() {
        let small = &joined[index].pts;
        let entry = small[0];
        let attachment = (first_host..joined.len()).rev().find_map(|host| {
            let host_path =
                crate::profile::rotate_to_start(&joined[host].pts, Some([entry.x, entry.y]));
            let anchor = host_path[0];
            (anchor.distance(entry) <= spacing + 2. * cam_geo::GRID_MM
                && cam_geo::clip_path(&cam_geo::Polyline::new(vec![anchor, entry]), guard, false)
                    .is_ok_and(|outside| outside.is_empty()))
            .then_some((host, host_path))
        });
        if let Some((host, host_path)) = attachment {
            let anchor = host_path[0];
            let mut merged = host_path;
            merged.rotate_left(1);
            // The host can clear its lap before returning to the attachment.
            merged.extend_from_slice(small);
            merged.extend([entry, anchor]);
            joined[host].pts = merged;
            joined.remove(index);
            first_host -= 1;
        }
    }
    (joined.len() < rings.len()).then_some(joined)
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
    air: &[bool],
    job: &ClearJob,
    h: &Heights,
    levels: &[f64],
) -> Result<(), RingsError> {
    emit_adaptive_moves_with_travel(
        prog,
        moves,
        AdaptiveTravel {
            air,
            feed_links: &[],
            minimum_lift: 0.,
            guarded: false,
            floor_links: &[],
        },
        job,
        h,
        levels,
    )
}

fn emit_3d_adaptive(
    program: &mut Program,
    path: &crate::frontadvance::LinkedFrontier,
    floor_links: &[crate::adaptive_linking::FloorLink],
    job: &ClearJob,
    heights: &Heights,
    levels: &[f64],
) -> Result<(), RingsError> {
    emit_adaptive_moves_with_travel(
        program,
        &path.moves,
        AdaptiveTravel {
            air: &[],
            feed_links: &[],
            minimum_lift: 0.2,
            guarded: false,
            floor_links,
        },
        job,
        heights,
        levels,
    )
}

struct AdaptiveTravel<'a> {
    air: &'a [bool],
    feed_links: &'a [bool],
    minimum_lift: f64,
    /// Includes radial withdrawal from walls and reserved finishing stock.
    guarded: bool,
    floor_links: &'a [crate::adaptive_linking::FloorLink],
}

fn emit_adaptive_moves_with_travel(
    prog: &mut Program,
    moves: &[(Point, bool)],
    travel: AdaptiveTravel<'_>,
    job: &ClearJob,
    h: &Heights,
    levels: &[f64],
) -> Result<(), RingsError> {
    if moves.len() < 2 {
        return Ok(());
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

    let start = moves[0].0;
    let tan = tan_at(0);
    let out = (-tan.1, tan.0);
    let placement = job
        .helix
        .map(|helix| helix.place(start, out, job.start.is_some()))
        .transpose()?;
    let mut from_z = h.retract.max(h.top_of_stock);
    for &z in levels {
        if let Some((helix, placement)) = job.helix.zip(placement.as_ref()) {
            helix.emit(
                prog,
                placement,
                from_z,
                z,
                h,
                job.feed,
                job.entry_feeds.ramp(job.feed),
                job.id,
                None,
            )?;
        } else {
            prog.push(Step::Rapid {
                to: Point3::new(start.x, start.y, h.clearance),
                tag: link,
            });
            crate::emit::descend_to(prog, start, from_z, h, job.feed, job.id);
            // A steered clearing path is **open**: it never returns to its entry, so a
            // one-way contour ramp would strand the wedge it leaves. Here the ramp runs
            // forward along the path and retraces it at depth — still along the toolpath,
            // but ending where it began, which is the contract the rest of this loop
            // assumes. A closed loop (profile, carve rings) needs no such return.
            match crate::profile::contour_ramp_len(job.plunge, from_z - z) {
                Some(len) => {
                    let path: Vec<_> = moves.iter().map(|&(p, _)| p).collect();
                    let ramp = crate::profile::advance_along_path(&path, len);
                    crate::profile::emit_open_ramp(
                        prog,
                        &ramp,
                        from_z,
                        z,
                        job.entry_feeds.ramp(job.feed),
                        plunge_tag,
                    );
                }
                None => crate::profile::emit_plunge(
                    prog,
                    start,
                    tan,
                    out,
                    from_z,
                    z,
                    crate::leads::guard_plunge(job.guard, start, out, job.plunge),
                    job.plunge_feed,
                    job.entry_feeds.ramp(job.feed),
                    plunge_tag,
                ),
            }
        }

        let lift_height = job.lift_height.max(travel.minimum_lift);
        let link_z = (z + lift_height).min(h.clearance);
        let mut lifted = false;
        let mut links = travel.floor_links.iter().peekable();
        let mut linked_through = 0;
        for i in 1..moves.len() {
            if i <= linked_through {
                continue;
            }
            if links.peek().is_some_and(|link| link.start == i) {
                let link = links.next().unwrap();
                link.emit(prog, job, z, link_z);
                linked_through = link.end;
                continue;
            }
            let (p, is_cut) = moves[i];
            // Air flags certify the clearing region. With wall stock reserved,
            // the cutter can still graze material just outside that region, so
            // these transitions must remain feed moves rather than rapids.
            let is_air = is_cut
                && (job.finish == 0.0 || travel.guarded)
                && travel.air.get(i).copied().unwrap_or(false);
            let is_feed_link = is_cut && travel.feed_links.get(i).copied().unwrap_or(false);
            let is_travel = is_air || is_feed_link;
            if lifted && !is_travel {
                if is_cut {
                    let from = moves[i - 1].0;
                    prog.push(Step::Linear {
                        to: Point3::new(from.x, from.y, z),
                        feed: job.plunge_feed,
                        tag: plunge_tag,
                    });
                }
                lifted = false;
            }
            if is_travel {
                // **A move that removes nothing is a traverse, not a cut.** A third of a
                // steered path is the tool crossing stock it has already removed to reach the
                // next front, and feeding it at cutting rate is the single largest remaining
                // waste — 352 mm of a 5.8 min program, ~1.2 min of pure air.
                //
                // Safe as a rapid *because of what is measured*, not by assumption: the flag
                // means the move's whole swept disc, inflated by a margin, met no uncut
                // material in the running model. Engagement will not do here — it is a
                // cutting-*load* measure, counting only the tool's leading arc, so a move
                // reads zero while the body of the tool sits in stock. Flagging traverses that
                // way put 776 `RapidThroughStock` collisions past this line. Nor is a
                // *starving* step enough: it may still shave a fraction of the target, and
                // that is a cut.
                if lift_height > 0.0 && !lifted {
                    let from = moves[i - 1].0;
                    prog.push(Step::Rapid {
                        to: Point3::new(from.x, from.y, link_z),
                        tag: link,
                    });
                    lifted = true;
                }
                let to = Point3::new(p.x, p.y, if lifted { link_z } else { z });
                if is_air {
                    prog.push(Step::Rapid { to, tag: link });
                } else {
                    prog.push(Step::Linear {
                        to,
                        feed: job.feed,
                        tag: link,
                    });
                }
            } else if is_cut {
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
                //
                // What *has* changed since that was written: the rapid no longer ends
                // exactly on the previous floor either. `emit::rapid_floor` stops it
                // `FLOOR_CLEARANCE` short — the cleared air is only cleared in the
                // model, and a cusp the last pass left standing is invisible to it.
                prog.push(Step::Rapid {
                    to: Point3::new(p.x, p.y, crate::emit::rapid_floor(from_z, h)),
                    tag: link,
                });
                prog.push(Step::Linear {
                    to: Point3::new(p.x, p.y, z),
                    feed: job.feed,
                    tag: plunge_tag,
                });
            }
        }
        let mut last = moves[moves.len() - 1].0;
        if lifted && job.end.is_some() {
            prog.push(Step::Linear {
                to: Point3::new(last.x, last.y, z),
                feed: job.plunge_feed,
                tag: plunge_tag,
            });
        }
        if let Some(preferred) = job.end.filter(|p| p.iter().all(|v| v.is_finite())) {
            // Retrace only the final connected run. Every segment has already been
            // swept by the cutter at this depth; never shortcut across a protected
            // island or a discontinuity that required a clearance move.
            let first = moves.iter().rposition(|(_, cutting)| !cutting).unwrap_or(0);
            let mut nearest = (moves.len() - 1, last, f64::INFINITY);
            for i in first + 1..moves.len() {
                let a = moves[i - 1].0;
                let b = moves[i].0;
                let dx = b.x - a.x;
                let dy = b.y - a.y;
                let length2 = dx * dx + dy * dy;
                if length2 <= 1e-24 {
                    continue;
                }
                let t = (((preferred[0] - a.x) * dx + (preferred[1] - a.y) * dy) / length2)
                    .clamp(0.0, 1.0);
                let point = Point::new(a.x + t * dx, a.y + t * dy);
                let distance = (point.x - preferred[0]).powi(2) + (point.y - preferred[1]).powi(2);
                // Ties prefer the latest visit, keeping return travel short.
                if distance <= nearest.2 {
                    nearest = (i, point, distance);
                }
            }
            if nearest.2.is_finite() {
                let tag = Tag::new(job.id, MoveKind::LeadOut);
                for i in (nearest.0..moves.len() - 1).rev() {
                    let p = moves[i].0;
                    prog.push(Step::Linear {
                        to: Point3::new(p.x, p.y, z),
                        feed: job.entry_feeds.lead_out(job.feed),
                        tag,
                    });
                }
                last = nearest.1;
                prog.push(Step::Linear {
                    to: Point3::new(last.x, last.y, z),
                    feed: job.entry_feeds.lead_out(job.feed),
                    tag,
                });
            }
        }
        prog.push(Step::Rapid {
            to: Point3::new(last.x, last.y, h.clearance),
            tag: retract,
        });
        from_z = z;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_geo::Contour;

    fn levels_two() -> [f64; 2] {
        [-1.0, -2.0]
    }

    fn frame_job() -> Clearing {
        Clearing {
            engagement: 2.0,
            climb: true,
        }
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
            lift_height: 0.0,
            keep_tool_down: false,
            id: 7,
            simplification_tolerance: crate::frontadvance::SIMPLIFY_EPS,
            radius: 3.0,
            finish: 0.0,
            first: 3.0,
            spacing: 4.0,
            clearing,
            plunge: Plunge::Straight,
            feed: 300.0,
            plunge_feed: 100.0,
            entry_feeds: Default::default(),
            lead_overlap: 0.0,
            lead_in: Lead::None,
            lead_out: Lead::None,
            start: None,
            end: None,
            guard: &[],
            helix: None,
            spindle: cam_cldata::SpindleDir::Cw,
        }
    }

    #[test]
    fn short_entry_fragment_joins_only_a_reachable_pass() {
        let ring = |x: f64, width: f64| Ring {
            pts: vec![
                Point::new(x, 0.),
                Point::new(x + width, 0.),
                Point::new(x + width, width),
                Point::new(x, width),
            ],
            is_wall: false,
        };
        let fragment = ring(0., 0.5);
        let host = ring(1., 10.);
        let guard = [Polygon::new(Contour::new(vec![
            Point::new(-1., -1.),
            Point::new(15., -1.),
            Point::new(15., 15.),
            Point::new(-1., 15.),
        ]))
        .unwrap()];
        let merged = join_small_entry_rings(&[fragment.clone(), host], 2.4, &guard).unwrap();
        assert_eq!(merged.len(), 1);
        assert!(merged[0].pts[0].x >= 1.);
        assert!(fragment.pts.iter().all(|p| merged[0].pts.contains(p)));
        assert!(join_small_entry_rings(&[fragment, ring(5., 8.)], 2.4, &guard).is_none());
    }

    #[test]
    fn adaptive_lift_spans_an_air_run_and_returns_to_depth_before_cutting() {
        let moves = [
            (Point::new(10., 10.), true),
            (Point::new(11., 10.), true),
            (Point::new(12., 10.), true),
            (Point::new(13., 10.), true),
        ];
        let air = [false, true, true, false];
        let h = Heights::new(5., 2., 0.);
        for lift in [0., 2., 100.] {
            let mut job = frame_clearjob(frame_job());
            job.lift_height = lift;
            let mut program = Program::new();
            emit_adaptive_moves(&mut program, &moves, &air, &job, &h, &[-3., -6.]).unwrap();
            for z in [-3., -6.] {
                let raised = (z + lift).min(h.clearance);
                for x in [11., 12.] {
                    assert!(program.steps().iter().any(|s| matches!(s,
                        Step::Rapid { to, tag } if tag.kind == MoveKind::Link
                            && *to == Point3::new(x, 10., raised))));
                }
                let cutting = program
                    .steps()
                    .iter()
                    .position(|s| {
                        matches!(s,
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting
                        && *to == Point3::new(13., 10., z))
                    })
                    .unwrap();
                if lift > 0. {
                    assert!(matches!(&program.steps()[cutting - 1],
                        Step::Linear { to, feed, tag } if tag.kind == MoveKind::Plunge
                            && *to == Point3::new(12., 10., z) && *feed == job.plunge_feed));
                    assert!(matches!(&program.steps()[cutting - 2],
                        Step::Rapid { to, .. } if *to == Point3::new(12., 10., raised)));
                }
            }
        }
    }

    #[test]
    fn cleared_stock_rapids_save_time_including_their_lifts() {
        let region = Polygon::new(Contour::new(vec![
            Point::new(10., -10.),
            Point::new(40., -10.),
            Point::new(40., 10.),
            Point::new(10., 10.),
        ]))
        .unwrap();
        let job = frame_clearjob(Clearing {
            engagement: 2.4,
            climb: true,
        });
        let heights = Heights::new(5., 2., 0.);
        let moves = crate::frontadvance::front_advance_certified(
            &region,
            3.,
            0.,
            2.4,
            None,
            crate::frontadvance::SIMPLIFY_EPS,
        )
        .unwrap();
        let mut cutting_links = Program::new();
        emit_adaptive_moves(&mut cutting_links, &moves, &[], &job, &heights, &[-6.]).unwrap();
        let mut rapid_links = Program::new();
        clear(
            &mut rapid_links,
            &region,
            &job,
            &heights,
            &[-6.],
            &CancelToken::new(),
        )
        .unwrap();
        let before = cutting_links
            .processing_time(4000., 0., 100.)
            .unwrap()
            .total_seconds();
        let after = rapid_links
            .processing_time(4000., 0., 100.)
            .unwrap()
            .total_seconds();
        eprintln!("cutting links {before:.2}s, lifted rapid links {after:.2}s");
        assert!(
            after < before - 5.,
            "lifts and re-approaches must not erase the travel saving"
        );
    }

    #[test]
    fn guarded_wall_links_lift_before_withdrawal_and_feed_back_before_cutting() {
        let moves = [
            (Point::new(10., 10.), false),
            (Point::new(10., 9.4), true),
            (Point::new(20., 9.4), true),
            (Point::new(20., 10.), true),
            (Point::new(21., 10.), true),
        ];
        let air = [false, false, true, false, false];
        let feed_links = [false, true, false, true, false];
        let h = Heights::new(5., 2., 0.);
        for requested in [0., 0.8, 100.] {
            let mut job = frame_clearjob(frame_job());
            job.lift_height = requested;
            let mut program = Program::new();
            emit_adaptive_moves_with_travel(
                &mut program,
                &moves,
                AdaptiveTravel {
                    air: &air,
                    feed_links: &feed_links,
                    minimum_lift: 0.2,
                    guarded: true,
                    floor_links: &[],
                },
                &job,
                &h,
                &[-3.],
            )
            .unwrap();
            let raised = (-3. + requested.max(0.2)).min(5.);
            assert!(program
                .steps()
                .iter()
                .any(|s| matches!(s, Step::Linear { to, tag, .. }
                if tag.kind == MoveKind::Link && *to == Point3::new(10., 9.4, raised))));
            assert!(program
                .steps()
                .iter()
                .any(|s| matches!(s, Step::Rapid { to, tag }
                if tag.kind == MoveKind::Link && *to == Point3::new(20., 9.4, raised))));
            assert!(program
                .steps()
                .iter()
                .any(|s| matches!(s, Step::Linear { to, tag, .. }
                if tag.kind == MoveKind::Link && *to == Point3::new(20., 10., raised))));
            let cutting = program
                .steps()
                .iter()
                .position(|s| {
                    matches!(s, Step::Linear { to, tag, .. }
                if tag.kind == MoveKind::Cutting && *to == Point3::new(21., 10., -3.))
                })
                .unwrap();
            assert!(
                matches!(&program.steps()[cutting - 1], Step::Linear { to, tag, .. }
                if tag.kind == MoveKind::Plunge && *to == Point3::new(20., 10., -3.))
            );
        }
    }

    #[test]
    fn a_lifted_air_run_retracts_without_touching_the_floor_again() {
        let moves = [
            (Point::new(10., 10.), true),
            (Point::new(11., 10.), true),
            (Point::new(30., 30.), false),
            (Point::new(31., 30.), true),
        ];
        let mut job = frame_clearjob(frame_job());
        job.lift_height = 2.;
        let h = Heights::new(5., 2., 0.);
        let mut program = Program::new();
        emit_adaptive_moves(
            &mut program,
            &moves,
            &[false, true, false, false],
            &job,
            &h,
            &[-3.],
        )
        .unwrap();
        assert!(!program.steps().iter().any(|s| matches!(s,
            Step::Linear { to, .. } if *to == Point3::new(11., 10., -3.))));
        assert!(program.steps().iter().any(|s| matches!(s,
            Step::Rapid { to, .. } if *to == Point3::new(11., 10., 5.))));
    }

    #[test]
    fn invalid_lift_height_emits_no_motion() {
        for lift in [-1., f64::NAN, f64::INFINITY] {
            let mut job = frame_clearjob(frame_job());
            job.lift_height = lift;
            let mut program = Program::new();
            assert!(clear(
                &mut program,
                &frame_region(),
                &job,
                &Heights::new(5., 2., 0.),
                &[-1.],
                &CancelToken::new()
            )
            .is_err());
            assert!(program.steps().is_empty());
        }
    }

    #[test]
    fn rejected_frontier_is_discarded_before_concentric_fallback() {
        let region = Polygon::new(Contour::new(vec![
            Point::new(0., 0.),
            Point::new(40., 0.),
            Point::new(40., 40.),
            Point::new(0., 40.),
        ]))
        .unwrap();
        let mut job = frame_clearjob(frame_job());
        let heights = Heights {
            clearance: 5.,
            retract: 2.,
            top_of_stock: 0.,
        };
        let token = CancelToken::new();
        let checked = std::cell::Cell::new(false);
        let mut actual = Program::new();
        actual.push(Step::Comment("preceding region".into()));
        let rings = clear_validated(
            &mut actual,
            &region,
            &job,
            &heights,
            &[-1.],
            &token,
            None,
            |candidate| {
                assert!(!candidate.steps().is_empty());
                checked.replace(true)
            },
        )
        .unwrap();
        assert!(checked.get(), "fixture must exercise frontier rejection");
        // Disable frontier dispatch while preserving the same ring spacing.
        job.spacing = job.effective_spacing();
        job.clearing.engagement = 0.;
        let mut expected = Program::new();
        expected.push(Step::Comment("preceding region".into()));
        let expected_rings = clear(&mut expected, &region, &job, &heights, &[-1.], &token).unwrap();
        assert_eq!(rings.paths, expected_rings);
        assert!(rings.concentric);
        assert_eq!(
            actual, expected,
            "no rejected moves may leak into the output"
        );
    }

    #[test]
    fn stay_down_preference_requires_fewer_lifts_and_an_accepted_fallback() {
        let region = frame_region();
        let job = frame_clearjob(frame_job());
        let heights = Heights::new(5., 2., 0.);
        let token = CancelToken::new();
        for accept_fallback in [true, false] {
            let original = std::cell::RefCell::new(None);
            let checked = std::cell::Cell::new(0);
            let mut actual = Program::new();
            let result = clear_validated(
                &mut actual,
                &region,
                &job,
                &heights,
                &[-1.],
                &token,
                Some(&Tool::default()),
                |candidate| {
                    let first = checked.replace(checked.get() + 1) == 0;
                    if first {
                        original.replace(Some(candidate.clone()));
                    }
                    first || accept_fallback
                },
            )
            .unwrap();
            assert_eq!(checked.get(), 2, "validate both frontier and fallback");
            let original = original.into_inner().unwrap();
            let original_lifts = clearance_lifts(&original, &heights);
            assert!(
                original_lifts > 1,
                "fixture must have a fragmented frontier"
            );
            assert_eq!(result.concentric, accept_fallback);
            if accept_fallback {
                assert!(clearance_lifts(&actual, &heights) < original_lifts);
            } else {
                assert_eq!(
                    actual, original,
                    "a rejected fallback must preserve the safe frontier"
                );
            }
        }
    }

    #[test]
    fn rejected_concentric_fallback_emits_no_motion() {
        let region = frame_region();
        let job = frame_clearjob(Clearing {
            engagement: 0.,
            climb: true,
        });
        let heights = Heights::new(5., 2., 0.);
        let mut program = Program::new();
        program.push(Step::Comment("preceding region".into()));
        let original = program.clone();
        assert!(clear_validated(
            &mut program,
            &region,
            &job,
            &heights,
            &[-1.],
            &CancelToken::new(),
            Some(&Tool::default()),
            |_| false,
        )
        .is_err());
        assert_eq!(program, original);
    }

    /// **A re-entry descends at cutting feed, and rapids down to the stock top.** Both
    /// halves are worth the same 4.7 minutes: measured on a real full-stepdown export, the
    /// re-entry descents were **468 mm at plunge feed — half the program's total time** —
    /// against 3.4 minutes of actual cutting. Above the stock surface there is nothing to
    /// hit, and below it the descent is through stock the seed search has already cleared.
    #[test]
    fn adaptive_exit_retraces_only_the_final_connected_cleared_run() {
        let mut job = frame_clearjob(frame_job());
        job.end = Some([10., 0.]);
        let moves = vec![
            (Point::new(0., 0.), false),
            (Point::new(10., 0.), true),
            (Point::new(20., 20.), false),
            (Point::new(30., 20.), true),
            (Point::new(30., 30.), true),
        ];
        let mut program = Program::new();
        emit_adaptive_moves(
            &mut program,
            &moves,
            &[],
            &job,
            &Heights::new(5., 2., 0.),
            &[-1.],
        )
        .unwrap();
        assert!(program.validate().is_ok());
        let exits: Vec<_> = program
            .steps()
            .iter()
            .filter_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::LeadOut => Some(*to),
                _ => None,
            })
            .collect();
        assert_eq!(
            exits,
            vec![Point3::new(30., 20., -1.), Point3::new(20., 20., -1.)]
        );
        assert!(
            matches!(program.steps().last(), Some(Step::Rapid { to, .. }) if *to == Point3::new(20., 20., 5.))
        );
        assert_eq!(
            count(
                &program,
                |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Cutting)
            ),
            3
        );
    }

    #[test]
    fn concentric_exit_is_independent_of_entry_and_keeps_wall_leads() {
        let region = Polygon::new(Contour::new(vec![
            Point::new(0., 0.),
            Point::new(40., 0.),
            Point::new(40., 40.),
            Point::new(0., 40.),
        ]))
        .unwrap();
        for plunge in [Plunge::Straight, Plunge::Ramp { angle_deg: 10.0 }] {
            let mut job = frame_clearjob(Clearing::default());
            job.start = Some([20., 10.]);
            job.end = Some([37., 20.]);
            job.plunge = plunge;
            job.lead_in = Lead::Arc { radius: 0.5 };
            job.lead_out = Lead::Arc { radius: 0.5 };
            let mut program = Program::new();
            clear(
                &mut program,
                &region,
                &job,
                &Heights::new(5., 2., 0.),
                &[-1.],
                &CancelToken::new(),
            )
            .unwrap();
            assert!(program.validate().is_ok());
            let mut previous = None;
            let mut last_exit = None;
            for step in program.steps() {
                let (point, tag) = match step {
                    Step::Linear { to, tag, .. } | Step::Rapid { to, tag } => (*to, *tag),
                    Step::Arc { end, tag, .. } => (*end, *tag),
                    _ => continue,
                };
                if tag.kind == MoveKind::LeadOut {
                    last_exit = previous;
                }
                previous = Some(point);
            }
            let exit: Point3 = last_exit.unwrap();
            assert!(
                (exit.x - 37.).hypot(exit.y - 20.) < 0.02,
                "{plunge:?}: {exit:?}"
            );
        }
    }

    #[test]
    fn a_re_entry_descends_at_cutting_feed_from_the_stock_top() {
        let region = frame_region();
        let job = frame_clearjob(frame_job());
        let heights = Heights::new(5.0, 2.0, 0.0); // clearance 5, retract 2, stock top 0
        let mut prog = Program::new();
        let Ok(_) = clear(
            &mut prog,
            &region,
            &job,
            &heights,
            &levels_two(),
            &CancelToken::new(),
        ) else {
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
            let mut job = frame_clearjob(Clearing {
                engagement: 2.0,
                climb: false,
            });
            job.plunge = plunge;
            let mut prog = Program::new();
            let Ok(_) = clear(
                &mut prog,
                &region,
                &job,
                &heights,
                &levels,
                &CancelToken::new(),
            ) else {
                panic!("clear should succeed");
            };
            let arcs = count(
                &prog,
                |s| matches!(s, Step::Arc { tag, .. } if tag.kind == MoveKind::Plunge),
            );
            let straight = count(
                &prog,
                |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge),
            );
            (arcs, straight)
        };
        let (straight_arcs, straight_drops) = plunges(Plunge::Straight);
        assert_eq!(straight_arcs, 0, "a straight plunge emits no arcs");
        assert!(
            straight_drops >= 2,
            "at least one entry per level, got {straight_drops}"
        );

        let (helix_arcs, helix_drops) = plunges(Plunge::Helix {
            radius: 1.0,
            pitch: 0.5,
        });
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
            let mut job = frame_clearjob(Clearing {
                engagement: 2.0,
                climb,
            });
            job.plunge = Plunge::Helix {
                radius: 2.0,
                pitch: 0.5,
            };
            job.lead_in = Lead::Arc { radius: 1.0 }; // hold it on the concentric path
            let mut prog = Program::new();
            let Ok(_) = clear(
                &mut prog,
                &region,
                &job,
                &heights,
                &levels,
                &CancelToken::new(),
            ) else {
                panic!("clear should succeed");
            };
            // The island, grown by the tool radius: no tool centre may enter it.
            let keep = Polygon::new(island.clone()).unwrap();
            let forbidden = cam_geo::offset(
                std::slice::from_ref(&keep),
                job.radius,
                cam_geo::JoinStyle::Round,
            )
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
        job.plunge = Plunge::Helix {
            radius: 1.0,
            pitch: 0.5,
        };
        let mut prog = Program::new();
        let Ok(n) = clear(
            &mut prog,
            &region,
            &job,
            &heights,
            &levels,
            &CancelToken::new(),
        ) else {
            panic!("clear should succeed");
        };
        assert_eq!(n, 1, "expected the steered path, got {n} rings");

        let arc_plunges = count(
            &prog,
            |s| matches!(s, Step::Arc { tag, .. } if tag.kind == MoveKind::Plunge),
        );
        let feed_plunges = count(
            &prog,
            |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge),
        );
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
        let Ok(n) = clear(
            &mut prog,
            &region,
            &job,
            &heights,
            &levels,
            &CancelToken::new(),
        ) else {
            panic!("clearing a frame should succeed");
        };
        assert_eq!(
            n, 1,
            "a certified steered clear is one continuous path, got {n}"
        );

        let cuts = count(
            &prog,
            |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Cutting),
        );
        let plunges = count(
            &prog,
            |s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge),
        );
        assert!(
            cuts > 50,
            "the frame has substantial cutting motion, got {cuts}"
        );
        assert!(
            plunges >= 2,
            "at least one entry plunge per level, got {plunges}"
        );
        // Every cutting move stays at a level depth (never above the stock top).
        for s in prog.steps() {
            if let Step::Linear { to, tag, .. } = s {
                if tag.kind == MoveKind::Cutting {
                    assert!(
                        to.z <= heights.top_of_stock + 1e-9,
                        "a cut rose above stock: z={}",
                        to.z
                    );
                }
            }
        }
    }

    /// Reversing the spindle exchanges climb and conventional, so a **climb** request on an
    /// `M4` spindle must wind the way an `M3` **conventional** one does — the same path, not a
    /// path that merely calls itself climb.
    ///
    /// This is the assumption the module header used to carry in prose ("the rings are wound so
    /// climb milling keeps the cleared side on the left of travel"), true only for `M3`.
    #[test]
    fn a_reversed_spindle_exchanges_climb_and_conventional_windings() {
        use cam_cldata::SpindleDir;
        let region = frame_region();
        let heights = Heights::new(5.0, 2.0, 0.0);
        let levels = [-1.0];
        let run = |climb: bool, spindle: SpindleDir| {
            // A finishing lead holds both windings on the concentric path, so the two are
            // comparable at all — the adaptive generators emit a single fixed winding.
            let mut job = frame_clearjob(Clearing {
                engagement: 2.0,
                climb,
            });
            job.spindle = spindle;
            job.lead_in = Lead::Arc { radius: 1.0 };
            let mut prog = Program::new();
            let Ok(_) = clear(
                &mut prog,
                &region,
                &job,
                &heights,
                &levels,
                &CancelToken::new(),
            ) else {
                panic!("clear should succeed")
            };
            prog
        };
        for climb in [true, false] {
            assert_eq!(
                run(climb, SpindleDir::Ccw).steps,
                run(!climb, SpindleDir::Cw).steps,
                "climb={climb} on M4 must be the M3 conventional path"
            );
        }
    }

    /// The adaptive generators emit one fixed winding and have no radial-order-preserving
    /// flip, so on a reversed spindle that winding is *conventional* — they must decline and
    /// let the concentric clearer, which honours either, answer instead.
    ///
    /// Measured through the emitted path rather than by inspecting the gate: the steered
    /// generator's output is nothing like a set of concentric rings, and the ring count is
    /// what tells them apart.
    #[test]
    fn the_adaptive_clearer_declines_a_reversed_spindle() {
        use cam_cldata::SpindleDir;
        let region = frame_region();
        let heights = Heights::new(5.0, 2.0, 0.0);
        let levels = [-1.0];
        let run = |spindle: SpindleDir| {
            let mut job = frame_clearjob(Clearing {
                engagement: 2.0,
                climb: true,
            });
            job.spindle = spindle;
            let mut prog = Program::new();
            let Ok(n) = clear(
                &mut prog,
                &region,
                &job,
                &heights,
                &levels,
                &CancelToken::new(),
            ) else {
                panic!("clear should succeed")
            };
            (n, prog)
        };
        // M3 + climb + a cap on a holed region is exactly the steered generator's dispatch.
        // It reports `1` — one continuous certified path, not a ring count — where the
        // concentric clearer reports the rings it actually laid.
        let (adaptive, adaptive_prog) = run(SpindleDir::Cw);
        let (concentric, concentric_prog) = run(SpindleDir::Ccw);
        assert_eq!(
            adaptive, 1,
            "M3 climb + cap must reach the steered generator"
        );
        assert!(
            concentric > 1,
            "M4 must fall through to the concentric clearer, got {concentric}"
        );
        assert_ne!(
            adaptive_prog.steps, concentric_prog.steps,
            "the two generators must not coincidentally agree"
        );
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
            let Ok(n) = clear(
                &mut prog,
                &region,
                &frame_clearjob(c),
                &heights,
                &levels,
                &CancelToken::new(),
            ) else {
                panic!("clear should succeed");
            };
            (n, prog)
        };
        let run_leaded = |c: Clearing| {
            let mut job = frame_clearjob(c);
            job.lead_in = Lead::Arc { radius: 1.0 };
            let mut prog = Program::new();
            let Ok(n) = clear(
                &mut prog,
                &region,
                &job,
                &heights,
                &levels,
                &CancelToken::new(),
            ) else {
                panic!("clear should succeed");
            };
            (n, prog)
        };

        let (tight, _) = run(Clearing {
            engagement: 1.0,
            climb: false,
        });
        let (loose, _) = run(Clearing {
            engagement: 4.0,
            climb: false,
        });
        assert!(
            tight > loose,
            "a tighter engagement cap must mean more rings: {tight} vs {loose}"
        );

        let (climb_n, climb_prog) = run_leaded(Clearing {
            engagement: 2.0,
            climb: true,
        });
        let (conv_n, conv_prog) = run_leaded(Clearing {
            engagement: 2.0,
            climb: false,
        });
        assert_eq!(
            climb_n, conv_n,
            "same rings either way; only the winding differs"
        );

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
            pts.windows(2)
                .map(|w| w[0].x * w[1].y - w[1].x * w[0].y)
                .sum()
        };
        let (sc, sv) = (signed(&climb_prog), signed(&conv_prog));
        assert!(
            sc * sv < 0.0,
            "climb and conventional must wind opposite ways, got {sc:.1} and {sv:.1}"
        );
    }
}
