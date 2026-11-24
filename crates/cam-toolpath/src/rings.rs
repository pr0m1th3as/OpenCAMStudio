//! Concentric-ring area clearing, shared by the pocket strategy and profile
//! roughing (stepover). Rings are offset loops of a region marching inward from
//! its wall; each is a plain closed cutting loop (approach, plunge per the plunge
//! strategy, cut with any closure overlap, retract).

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, JoinStyle, Point, Polygon};
use cam_model::{Heights, Lead, Plunge};

use crate::pocket_helix::{PocketHelix, PocketHelixFit};
use crate::CancelToken;

/// Safety cap on the number of concentric rings (guards the offset loop).
const MAX_RINGS: usize = 100_000;

/// One concentric clearing ring.
#[derive(Clone)]
pub(crate) struct Ring {
    /// The loop to cut.
    pub pts: Vec<Point>,
    /// Whether this loop is a **finished wall** — a loop from the first (smallest)
    /// offset, hugging the boundary or an island. These get the wall-finish leads.
    pub is_wall: bool,
}

/// Why ring generation stopped without producing rings.
#[derive(Debug)]
pub(crate) enum RingsError {
    /// The job was cancelled mid-generation.
    Cancelled,
    /// A geometry offset failed.
    Offset(String),
    Limit(String),
    PocketHelixFit(PocketHelixFit),
}

impl std::fmt::Display for RingsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Cancelled => f.write_str("Clearing was cancelled"),
            Self::Offset(message) | Self::Limit(message) => f.write_str(message),
            Self::PocketHelixFit(fit) => fit.fmt(f),
        }
    }
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
    radius: f64,
    cancel: &CancelToken,
) -> Result<Vec<Ring>, RingsError> {
    if cancel.is_cancelled() {
        return Err(RingsError::Cancelled);
    }
    let at_offset = |distance: f64| {
        offset(std::slice::from_ref(region), -distance, JoinStyle::Round)
            .map_err(|e| RingsError::Offset(e.to_string()))
    };
    let mut rings: Vec<Ring> = Vec::new();
    let mut d = first;
    let mut offsets = at_offset(first)?;
    while !offsets.is_empty() {
        if cancel.is_cancelled() {
            return Err(RingsError::Cancelled);
        }
        let is_wall = d == first;
        for poly in offsets {
            rings.extend(boundary_rings(&poly, is_wall));
        }
        // A non-positive stepover takes only the requested wall pass.
        if stepover <= 0.0 {
            return if rings.len() > MAX_RINGS {
                Err(RingsError::Limit(
                    "Too many roughing rings; increase stepover".into(),
                ))
            } else {
                Ok(rings)
            };
        }

        let next = d + stepover;
        if next == d {
            return Err(RingsError::Limit(
                "Radial stepover is too small to advance the offset".into(),
            ));
        }
        offsets = at_offset(next)?;
        if stepover > radius && stepover <= 2.0 * radius {
            // The current rings reach one cutter radius into their offset.
            // The next rings reach one radius back out. When an offset lobe
            // shrinks or disappears, some stock may be beyond both sweeps.
            let inner = at_offset(d + radius)?;
            let covered = offset(&offsets, radius, JoinStyle::Round)
                .map_err(|e| RingsError::Offset(e.to_string()))?;
            let gaps = cam_geo::difference(&inner, &covered)
                .map_err(|e| RingsError::Offset(e.to_string()))?;
            if !gaps.is_empty() {
                let reach = offset(&gaps, radius, JoinStyle::Round)
                    .map_err(|e| RingsError::Offset(e.to_string()))?;
                for poly in inner {
                    for ring in boundary_rings(&poly, false) {
                        if cancel.is_cancelled() {
                            return Err(RingsError::Cancelled);
                        }
                        let mut closed = ring.pts.clone();
                        closed.push(ring.pts[0]);
                        let hits = cam_geo::clip_path(&cam_geo::Polyline::new(closed), &reach, true)
                            .map_err(|e| RingsError::Offset(e.to_string()))?;
                        if !hits.is_empty() {
                            rings.push(ring);
                        }
                    }
                }
            }
        }
        d = next;
        if rings.len() > MAX_RINGS {
            return Err(RingsError::Limit(
                "Too many roughing rings; increase stepover".into(),
            ));
        }
    }
    Ok(rings)
}

fn boundary_rings(region: &Polygon, is_wall: bool) -> impl Iterator<Item = Ring> + '_ {
    std::iter::once(region.outer())
        .chain(region.holes())
        .map(move |contour| Ring {
            pts: contour.points().to_vec(),
            is_wall,
        })
}

/// Cut one closed ring (plus closure overlap) at `z`, returning the exit point.
fn cut_ring(
    prog: &mut Program,
    pts: &[Point],
    feed: f64,
    tag: Tag,
    lead_overlap: f64,
    end: Option<[f64; 2]>,
    z: f64,
) -> Point {
    let (loop_pts, exit, _tan) = crate::emit::loop_with_exit(pts, lead_overlap, end);
    crate::emit::cut_polyline(prog, &loop_pts, feed, tag, z);
    exit
}

/// Rapid over `p`, down through cleared air to `from_z`, then enter `z` **with the
/// operator's plunge strategy**.
///
/// This used to plunge straight here regardless, justified as "only ever one stepdown
/// into a layer already cleared above". That reasoning does not hold: it is one
/// stepdown of *depth*, but the material at this XY is **uncut** — a lift-reposition
/// exists precisely because the next ring is somewhere the tool has not been — and at
/// the first level there is no cleared layer above at all. So an operator who selected
/// a helix got a straight drop into solid at every island loop and pinched lobe. What
/// is cleared above only bounds how *far* it drops, not what it drops into.
///
/// The rule everywhere else in CAM is that the entry strategy is chosen by what is
/// under the tool, not by whether it is the first entry of the level; that is what this
/// now does. `tan`/`out` orient the helix or ramp, as at a level entry.
///
/// `ring` is the closed loop about to be cut, beginning where the cut begins;
/// `lead_path` is the lead-in from its own start point to that ring point (empty when
/// there is no lead).
///
/// A contour ramp descends along the lead **through air** and then along the ring, in
/// one unbroken slope — the same shape a profile pass uses, via the shared
/// [`crate::profile::contour_ramp`]. Returns the arc length of ring the ramp consumed:
/// the caller must begin its cut there and travel a full perimeter, so the stretch the
/// ramp sloped is re-machined at depth. `None` means it did not ramp, and the caller's
/// lead-in still has its usual job.
#[allow(clippy::too_many_arguments)]
fn enter_with_plunge(
    prog: &mut Program,
    p: Point,
    tan: (f64, f64),
    out: (f64, f64),
    from_z: f64,
    z: f64,
    h: &Heights,
    plunge: Plunge,
    plunge_feed: f64,
    entry_feeds: cam_model::EntryFeeds,
    feed: f64,
    id: u32,
    ring: &[Point],
    lead_path: Vec<Point>,
    guard: &[Polygon],
    local_from: Option<Point>,
    helix: Option<&PocketHelix<'_>>,
    fixed_entry: bool,
) -> Result<Option<f64>, RingsError> {
    if let Some(helix) = helix {
        let placement = helix.place(p, out, fixed_entry)?;
        helix.emit(
            prog,
            &placement,
            from_z,
            z,
            h,
            feed,
            entry_feeds.ramp(feed),
            id,
            local_from,
        )?;
        return Ok(None);
    }
    // The ramp never travels through air to reach the material: its height is measured
    // from the top of stock, or from the floor the previous level left.
    let ramp_top = from_z.min(h.top_of_stock);
    let ramp = crate::profile::contour_ramp(ring, lead_path, plunge, ramp_top, z);
    // Where the tool comes down: the ramp's own start (above the material when there is
    // a lead to fly in), or the entry footprint.
    let (down_at, arrive_at) = match &ramp {
        Some((path, _, top)) => (path[0], top.max(z)),
        None => (p, from_z),
    };
    if let Some(from) = local_from {
        // A short ring-to-ring link may still touch stock. Feed the lift and
        // traverse, then use the requested entry strategy at the next ring.
        let link = Tag::new(id, MoveKind::Link);
        prog.push(Step::Linear {
            to: Point3::new(from.x, from.y, from_z),
            feed,
            tag: link,
        });
        prog.push(Step::Linear {
            to: Point3::new(down_at.x, down_at.y, from_z),
            feed,
            tag: if from_z < h.clearance {
                Tag::new(id, MoveKind::Cutting)
            } else {
                link
            },
        });
        if arrive_at < from_z {
            prog.push(Step::Linear {
                to: Point3::new(down_at.x, down_at.y, arrive_at),
                feed,
                tag: link,
            });
        }
    } else {
        prog.push(Step::Rapid {
            to: Point3::new(down_at.x, down_at.y, h.clearance),
            tag: Tag::new(id, MoveKind::Link),
        });
        // Stop the rapid short of a cut floor; feed the rest of the approach.
        crate::emit::descend_to(prog, down_at, arrive_at, h, feed, id);
    }
    if let Some((path, on_ring, top)) = &ramp {
        crate::profile::emit_descending_path(
            prog,
            path,
            *top,
            z,
            entry_feeds.ramp(feed),
            Tag::new(id, MoveKind::Plunge),
        );
        return Ok(Some(*on_ring));
    }
    crate::profile::emit_plunge(
        prog,
        p,
        tan,
        out,
        from_z,
        z,
        crate::leads::guard_plunge(guard, p, out, plunge),
        plunge_feed,
        entry_feeds.ramp(feed),
        Tag::new(id, MoveKind::Plunge),
    );
    Ok(None)
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
/// A local link with nonzero lift feeds above the level and re-enters with the
/// requested plunge strategy; a zero lift preserves the original at-depth cut.
#[allow(clippy::too_many_arguments)]
fn approach(
    prog: &mut Program,
    prev_end: Point,
    p: Point,
    tan: (f64, f64),
    out: (f64, f64),
    from_z: f64,
    z: f64,
    h: &Heights,
    plunge: Plunge,
    feed: f64,
    plunge_feed: f64,
    entry_feeds: cam_model::EntryFeeds,
    id: u32,
    link_threshold: f64,
    lift_height: f64,
    ring: &[Point],
    lead_path: Vec<Point>,
    guard: &[Polygon],
    helix: Option<&PocketHelix<'_>>,
    fixed_entry: bool,
) -> Result<Option<f64>, RingsError> {
    let hop = (p.x - prev_end.x).hypot(p.y - prev_end.y);
    let local = hop <= link_threshold
        && (guard.is_empty() || hop <= 1e-9 || crate::linking::contained(guard, prev_end, p));
    if local && (lift_height == 0.0 || hop <= 1e-9) {
        prog.push(Step::Linear {
            to: Point3::new(p.x, p.y, z),
            feed,
            tag: Tag::new(id, MoveKind::Cutting),
        });
        return Ok(None);
    }
    // A lead can start beyond the cleared area even when the ring-to-ring hop
    // is short. Retract before moving to that entry; a local lifted traverse
    // cannot establish clearance for the whole lead-in path.
    // Retain the short lifted link when the helix keeps its usual footprint.
    // A relocated entry gets approached from clearance.
    if local
        && lead_path.is_empty()
        && !guard.is_empty()
        && (helix.is_none() || crate::leads::guard_plunge(guard, p, out, plunge) == plunge)
    {
        return enter_with_plunge(
            prog,
            p,
            tan,
            out,
            (z + lift_height).min(h.clearance),
            z,
            h,
            plunge,
            plunge_feed,
            entry_feeds,
            feed,
            id,
            ring,
            lead_path,
            guard,
            Some(prev_end),
            helix,
            fixed_entry,
        );
    }
    prog.push(Step::Rapid {
        to: Point3::new(prev_end.x, prev_end.y, h.clearance),
        tag: Tag::new(id, MoveKind::Link),
    });
    enter_with_plunge(
        prog,
        p,
        tan,
        out,
        from_z,
        z,
        h,
        plunge,
        plunge_feed,
        entry_feeds,
        feed,
        id,
        ring,
        lead_path,
        guard,
        None,
        helix,
        fixed_entry,
    )
}

/// An automatic wall seam needs space on both sides for entry and exit leads.
/// Match the finishing strategy by starting at the longest edge's midpoint.
pub(crate) fn automatic_wall_start(pts: &[Point]) -> Point {
    let (a, b) = pts
        .iter()
        .zip(pts.iter().cycle().skip(1))
        .max_by(|(a, b), (c, d)| a.distance_sq(**b).total_cmp(&c.distance_sq(**d)))
        .expect("a wall ring has at least three points");
    Point::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5)
}

pub(crate) enum WallApproach {
    Clearance,
    FromRing { max_distance: f64, lift_height: f64 },
    AfterRetract { max_distance: f64, move_seam: bool },
}

/// The point immediately before a vertical retract from this pass's depth.
fn retracted_at_depth(program: &Program, id: u32, z: f64) -> Option<Point> {
    let [.., previous, Step::Rapid { to, tag }] = program.steps() else {
        return None;
    };
    if tag.kind != MoveKind::Retract || tag.op_id != id || to.z <= z {
        return None;
    }
    let (from, previous_tag) = match previous {
        Step::Linear { to, tag, .. } | Step::Arc { end: to, tag, .. } => (to, tag),
        _ => return None,
    };
    (previous_tag.op_id == id
        && (from.z - z).abs() <= 1e-9
        && (from.x - to.x).hypot(from.y - to.y) <= 1e-9)
        .then(|| Point::new(from.x, from.y))
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
pub(crate) fn emit_wall_ring(
    prog: &mut Program,
    pts: &[Point],
    seed: Point,
    from_z: f64,
    z: f64,
    id: u32,
    feed: f64,
    plunge_feed: f64,
    entry_feeds: cam_model::EntryFeeds,
    lead_overlap: f64,
    lead_in: Lead,
    lead_out: Lead,
    guard: &[Polygon],
    reversed: bool,
    h: &Heights,
    plunge: Plunge,
    approach_mode: WallApproach,
    end: Option<[f64; 2]>,
    helix: Option<&PocketHelix<'_>>,
    fixed_entry: bool,
) -> Result<Point, RingsError> {
    let rotate = |p: Point| crate::profile::rotate_to_start(pts, Some([p.x, p.y]));
    let entry_at = |ring: &[Point]| {
        let tan = crate::profile::start_tangent(ring);
        let normal = cleared_normal(tan, reversed);
        let lead = crate::leads::guard_lead(guard, ring[0], tan, normal, lead_in, true);
        let point = crate::leads::lead_start_point(ring[0], tan, normal, lead);
        (lead, point)
    };
    let mut ri = rotate(seed);
    let (mut eff_in, mut entry) = entry_at(&ri);
    let mut at_depth = None;
    if let WallApproach::AfterRetract {
        max_distance,
        move_seam,
    } = approach_mode
    {
        if let Some(from) = retracted_at_depth(prog, id, z) {
            let reachable = |point: Point| {
                point.distance_sq(from) <= max_distance.powi(2)
                    && crate::linking::contained(guard, from, point)
            };
            if reachable(entry) {
                at_depth = Some(from);
            } else if move_seam {
                let nearby = rotate(from);
                let (nearby_lead, nearby_entry) = entry_at(&nearby);
                if (eff_in == Lead::None || nearby_lead != Lead::None) && reachable(nearby_entry) {
                    ri = nearby;
                    eff_in = nearby_lead;
                    entry = nearby_entry;
                    at_depth = Some(from);
                }
            }
        }
    }
    let start = ri[0];
    let tan_in = crate::profile::start_tangent(&ri);
    let cin = cleared_normal(tan_in, reversed);
    let (loop_pts, exit_on, tan_out) = crate::emit::loop_with_exit(&ri, lead_overlap, end);
    let cout = cleared_normal(tan_out, reversed);

    // Drop any lead that would swing past the far wall down to a plain pass.
    let eff_out = crate::leads::guard_lead(guard, exit_on, tan_out, cout, lead_out, false);
    let exit = crate::leads::lead_end_point(exit_on, tan_out, cout, eff_out);

    let lead = Tag::new(id, MoveKind::LeadIn);
    let cut = Tag::new(id, MoveKind::Cutting);
    // The ramp descends *along the lead* — through air, meeting the wall at `start` —
    // and then along the ring. So the lead is handed to the entry rather than emitted
    // after it.
    let ramped = if let Some(from) = at_depth {
        prog.steps.pop();
        if from.distance_sq(entry) > 1e-18 {
            // The link can cut the reserved wall stock, so feed it as a cut.
            prog.push(Step::Linear {
                to: Point3::new(entry.x, entry.y, z),
                feed,
                tag: cut,
            });
        }
        None
    } else {
        let (link_threshold, lift_height) = match approach_mode {
            WallApproach::FromRing {
                max_distance,
                lift_height,
            } => (max_distance, lift_height),
            _ => (-1., 0.),
        };
        approach(
            prog,
            seed,
            entry,
            tan_in,
            cin,
            from_z,
            z,
            h,
            plunge,
            feed,
            plunge_feed,
            entry_feeds,
            id,
            link_threshold,
            lift_height,
            &ri,
            crate::profile::lead_in_path(start, entry, cin, eff_in),
            guard,
            helix,
            fixed_entry,
        )?
    };
    Ok(match ramped {
        // It ramped: the lead was flown on the way down, and the cut must begin where
        // the ramp ended and travel a full perimeter, so the sloped stretch is
        // re-machined at depth.
        Some(on_ring) => {
            let perim = crate::profile::loop_perimeter(&ri);
            let walked = crate::profile::walk_loop(
                &ri,
                on_ring,
                perim + crate::profile::exit_overlap(&ri, on_ring, lead_overlap, end),
            );
            let last = walked[walked.len() - 1];
            let prev = walked[walked.len().saturating_sub(2)];
            let tan_out = crate::profile::unit(last.x - prev.x, last.y - prev.y);
            let cout = cleared_normal(tan_out, reversed);
            let eff_out = crate::leads::guard_lead(guard, last, tan_out, cout, lead_out, false);
            let exit = crate::leads::lead_end_point(last, tan_out, cout, eff_out);
            crate::emit::cut_polyline(prog, &walked, feed, cut, z);
            // The lead-**out** stays flat at depth; only the lead-in descends.
            crate::leads::emit_lead(
                prog,
                last,
                exit,
                last,
                cout,
                eff_out,
                z,
                entry_feeds.lead_out(feed),
                Tag::new(id, MoveKind::LeadOut),
            );
            exit
        }
        None => {
            crate::leads::emit_lead(
                prog,
                entry,
                start,
                start,
                cin,
                eff_in,
                z,
                entry_feeds.lead_in(feed),
                lead,
            );
            crate::emit::cut_polyline(prog, &loop_pts, feed, cut, z);
            crate::leads::emit_lead(
                prog,
                exit_on,
                exit,
                exit_on,
                cout,
                eff_out,
                z,
                entry_feeds.lead_out(feed),
                Tag::new(id, MoveKind::LeadOut),
            );
            exit
        }
    })
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
    end: Option<[f64; 2]>,
    id: u32,
    feed: f64,
    plunge_feed: f64,
    entry_feeds: cam_model::EntryFeeds,
    plunge: Plunge,
    lift_height: f64,
    lead_overlap: f64,
    lead_in: Lead,
    lead_out: Lead,
    guard: &[Polygon],
    reversed: bool,
    h: &Heights,
    levels: &[f64],
    link_threshold: f64,
    cancel: &CancelToken,
    helix: Option<&PocketHelix<'_>>,
) -> Result<(), RingsError> {
    if rings.is_empty() {
        return Ok(());
    }
    // The level entry's own Link rapid now belongs to `enter_with_plunge`, which is
    // the only thing that knows where the tool must come down.
    let cut = Tag::new(id, MoveKind::Cutting);
    let retract = Tag::new(id, MoveKind::Retract);
    let leaded = lead_in != Lead::None || lead_out != Lead::None;

    let mut from_z = h.retract.max(h.top_of_stock);
    for &z in levels {
        if cancel.is_cancelled() {
            return Err(RingsError::Cancelled);
        }
        // Enter at the innermost ring (a boundary loop, so the plunge strategy's
        // helix/ramp is placed on the inward, cleared side), per the operator's
        // start preference. The descent after the first level is one stepdown into
        // stock cleared above.
        let entry_start = start.or_else(|| {
            (rings[0].is_wall && leaded).then(|| {
                let seed = automatic_wall_start(&rings[0].pts);
                [seed.x, seed.y]
            })
        });
        let r0 = crate::profile::rotate_to_start(&rings[0].pts, entry_start);
        let tan = crate::profile::start_tangent(&r0);
        let out = crate::profile::outward_normal(&r0);
        // The side the lead eases in from — into the pocket, as for a wall ring.
        let cin = cleared_normal(tan, reversed);
        let mut prev_end = if rings[0].is_wall && leaded {
            // A pocket with no interior rings still needs the wall emitter: the
            // ordinary first-ring path only uses leads as part of a ramp, and
            // never emits a flat lead-in or lead-out.
            emit_wall_ring(
                prog,
                &r0,
                r0[0],
                from_z,
                z,
                id,
                feed,
                plunge_feed,
                entry_feeds,
                lead_overlap,
                lead_in,
                lead_out,
                guard,
                reversed,
                h,
                plunge,
                WallApproach::Clearance,
                end.filter(|_| rings.len() == 1),
                helix,
                start.is_some(),
            )?
        } else {
            let on_ring = enter_with_plunge(
                prog,
                r0[0],
                tan,
                (-out.0, -out.1),
                from_z,
                z,
                h,
                plunge,
                plunge_feed,
                entry_feeds,
                feed,
                id,
                &r0,
                // **The lead-in applies here too.** It used to be omitted, on the reasoning
                // that a clearing lead is the *wall-finish* lead and this is the interior
                // entry. But the operator selected an arc lead-in *and* a ramp, and got
                // neither an arc nor a lead — the ramp simply started on the ring. The rule
                // is the one Andreas gave: if there is a lead-in, the ramp comes down it.
                //
                // Safe here despite there being nothing cleared at this level yet: the ramp
                // flies its lead *above* the material, meeting the surface where the ring
                // starts, so the lead cuts nothing on the way in.
                {
                    let eff = crate::leads::guard_lead(guard, r0[0], tan, cin, lead_in, true);
                    let entry = crate::leads::lead_start_point(r0[0], tan, cin, eff);
                    crate::profile::lead_in_path(r0[0], entry, cin, eff)
                },
                guard,
                None,
                helix,
                start.is_some(),
            )?;
            // A ramped entry begins its cut where the ramp ended, and goes a full perimeter
            // from there so the sloped stretch is re-machined at depth.
            match on_ring {
                Some(from) => {
                    let perim = crate::profile::loop_perimeter(&r0);
                    let overlap = crate::profile::exit_overlap(
                        &r0,
                        from,
                        lead_overlap,
                        end.filter(|_| rings.len() == 1),
                    );
                    let walked = crate::profile::walk_loop(&r0, from, perim + overlap);
                    crate::emit::cut_polyline(prog, &walked, feed, cut, z);
                    walked[walked.len() - 1]
                }
                None => cut_ring(
                    prog,
                    &r0,
                    feed,
                    cut,
                    lead_overlap,
                    end.filter(|_| rings.len() == 1),
                    z,
                ),
            }
        };

        for (index, ring) in rings[1..].iter().enumerate() {
            if cancel.is_cancelled() {
                return Err(RingsError::Cancelled);
            }
            let exit_preference = end.filter(|_| index + 2 == rings.len());
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
                    entry_feeds,
                    lead_overlap,
                    lead_in,
                    lead_out,
                    guard,
                    reversed,
                    h,
                    plunge,
                    WallApproach::FromRing {
                        max_distance: link_threshold,
                        lift_height,
                    },
                    exit_preference,
                    helix,
                    false,
                )?;
                continue;
            }
            // Begin this ring at the point nearest where the last one ended, so the
            // hop is the shortest possible — a clean radial step for adjacent rings.
            let ri = crate::profile::rotate_to_start(&ring.pts, Some([prev_end.x, prev_end.y]));
            // Orient a helix/ramp toward the **cleared** side. Not `-outward_normal`:
            // that is relative to each loop's own winding, so on an island ring — which
            // is wound CW precisely so the pocket stays on the left — it points *into
            // the island*, and the helix would swing through material meant to survive.
            // [`cleared_normal`] is the function that already knows this, and it handles
            // conventional milling's flipped windings too.
            let rtan = crate::profile::start_tangent(&ri);
            let on_ring = approach(
                prog,
                prev_end,
                ri[0],
                rtan,
                cleared_normal(rtan, reversed),
                from_z,
                z,
                h,
                plunge,
                feed,
                plunge_feed,
                entry_feeds,
                id,
                link_threshold,
                lift_height,
                &ri,
                // An interior clearing ring has no lead of its own.
                Vec::new(),
                guard,
                helix,
                false,
            )?;
            // As at the level entry: a ramped approach starts its cut where the ramp
            // ended and travels a full perimeter, so it re-machines what it sloped. A
            // short hop cuts across at depth and needs none of that.
            prev_end = match on_ring {
                Some(from) => {
                    let perim = crate::profile::loop_perimeter(&ri);
                    let walked = crate::profile::walk_loop(
                        &ri,
                        from,
                        perim
                            + crate::profile::exit_overlap(
                                &ri,
                                from,
                                lead_overlap,
                                exit_preference,
                            ),
                    );
                    crate::emit::cut_polyline(prog, &walked, feed, cut, z);
                    walked[walked.len() - 1]
                }
                None => cut_ring(prog, &ri, feed, cut, lead_overlap, exit_preference, z),
            };
        }
        // Retract; the next level re-enters at the interior start.
        prog.push(Step::Rapid {
            to: Point3::new(prev_end.x, prev_end.y, h.clearance),
            tag: retract,
        });
        from_z = z;
    }
    Ok(())
}
