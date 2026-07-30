//! Engagement-**steered** clearing — a probe of the other adaptive architecture.
//!
//! [`crate::frontadvance`] works a **pass** at a time: offset the cleared region outward,
//! erode by the tool radius, take the contours, connect them. Engagement is a *consequence*
//! of that construction, checked afterwards by the oracle. The assumption underneath is
//! *advance a fixed distance ⇒ bounded engagement*, which holds in open material and fails
//! exactly where we are failing — in the narrow band between an island and a wall, and
//! wherever the frontier changes shape. `ADAPTIVE_PLAN.md` §8.6 has the measurements.
//!
//! This module works a **step** at a time, the way FreeCAD's `Adaptive2d` (the Tusek
//! algorithm, LGPL-2.1-or-later — read for the algorithm, implemented here from scratch
//! against our own oracle) and the constant-engagement literature do: from the current
//! position, *search the turn angle* for the one that removes the target area, take a short
//! step, repeat. Engagement stops being something the geometry hands you and becomes the
//! control variable. A narrow band needs no special case — the tool simply steers to hold
//! the target.
//!
//! **Status.** It works, and **nothing dispatches it yet.** Measured against the exact oracle
//! on the shapes `frontadvance` was measured on, at r=3 / e=2 (harness
//! `steered_over_every_shape`), **6 of 7 certify** — every one terminating by finishing the
//! region, none gouging:
//!
//! ```text
//!   square 40, no island        a_e 2.07   uncut  7/33   CERTIFIED
//!   circle r30, no island       a_e 2.07   uncut  7/58   CERTIFIED
//!   square 40, island 12        a_e 2.07   uncut  7/30   CERTIFIED   ← frontier reads 6.00
//!   circle r20, island r6       a_e 2.07   uncut  3/24   CERTIFIED   ← frontier reads 6.00
//!   square 60, island 20        a_e 2.07   uncut  4/65   CERTIFIED
//!   square 60, two islands      a_e 2.58   uncut 11/67   CERTIFIED   ← frontier reads 6.00
//!   square 40, island offset    a_e 5.65   uncut 26/29   no
//! ```
//!
//! Note the hole-free figures: **2.07 against `frontadvance`'s own 2.69–2.90**, so this is not
//! merely an island strategy. The single failure is **geometry, not the generator** — that shape
//! leaves a strip between island and wall exactly 6 mm wide, the full diameter of the tool, and
//! a tool cannot clear a slot its own width at anything less than full width. Give it a Ø5 and
//! the same shape certifies at 2.50 (`the_offset_island_is_geometry_not_generator`).
//!
//! Costs 1.2–7.8 s per region in release. Still to do before dispatch: cheaper travel (a large
//! share of the moves are hunting through cleared stock), a real entry search rather than a
//! fixed opening spiral, and the dispatch wiring itself.

#![allow(dead_code)]

use cam_geo::{offset, JoinStyle, Point, Polygon};

use crate::clearsim::ClearedModel;
use crate::CancelToken;

/// Step length, as a fraction of the tool radius. Short enough that a 45° turn per step
/// bends at ~1 mm radius, which is tighter than any feature a Ø6 tool can enter.
const STEP_RADII: f64 = 0.25;

/// **Smallest radius the path may turn on, in tool radii.**
///
/// The turn limit is expressed as a *radius*, not an angle, because an angle bound says
/// nothing about smoothness on its own: a 72° turn is gentle over 20 mm and a corner over
/// 0.75 mm. Measured on the previous fixed-angle bound, **21% of corners exceeded 20°**, one
/// step in ten sat exactly on the 72° limit, and the implied corner radius was **0.64 mm —
/// 0.21·r for a ⌀6 cutter**. A machine cannot run that at feed: it decelerates into every
/// corner, which spikes the chip load and gives away the very thing constant-engagement
/// milling is for.
const MIN_TURN_RADII: f64 = 0.25;

/// The per-step turn that [`MIN_TURN_RADII`] allows, for a step of length `h`.
fn max_turn(h: f64, r: f64) -> f64 {
    let r_min = (MIN_TURN_RADII * r).max(1e-6);
    (h / (2.0 * r_min)).clamp(-1.0, 1.0).asin() * 2.0
}

/// Iterations of the turn-angle search. Eight halves the bracket 256-fold, far below the
/// grid's own resolution, so more would be measuring quantisation noise.
const SEARCH_ITERS: usize = 8;

/// Fraction of the target bite below which the front counts as exhausted — there is no
/// material left within reach of a turn, so this front is finished.
const STALL_FRAC: f64 = 0.15;

/// How far a starving front will hunt for material, in tool radii, before it is declared
/// dead and resumed elsewhere.
const HUNT_RADII: f64 = 8.0;

/// Consecutive starved steps allowed before the front counts as dead — and therefore how far
/// the tool will hunt through cleared stock at **feed rate** before it lifts and re-seeds.
///
/// This is a travel knob, and it is worth real money. Hunting is air, and air at feed rate is
/// the one way a constant-engagement path can end up *slower* than the concentric clear it
/// replaces. Measured across the shape set as the limit falls (60 → 20 → 8 → 4), the share of
/// path length spent in air on an island pocket goes **69% → 59% → 48% → 32%**, and total
/// travel on a 60 mm pocket **4966 mm → 2248 mm**. Everything still certifies throughout.
///
/// 8 rather than 4 because the last step of that trade is a poor one: it buys ~20% more travel
/// while **doubling** the uncut remainder (18 → 35 mm² of a 65 mm² tolerance on the 60 mm
/// case). Margin against the coverage tolerance is worth more than the last of the air.
///
/// **Re-derived against estimated cycle time rather than path length**, because length is not
/// what the operator pays: hunting less means re-seeding more, and every re-seed costs a
/// retract, a cross and a plunge at *plunge* feed. Scoring cut/air/plunge/rapid at their own
/// rates, 8 still wins and not narrowly — on the 60 mm island pocket **11.9 min at 8, 14.4 at
/// 20, 18.8 at 60**. The suspicion that the length metric had bought hidden plunge overhead was
/// worth checking and turned out to be wrong.
const STARVE_LIMIT: usize = 8;

/// Guard on total steps.
const MAX_STEPS: usize = 100_000;

/// Steps between progress checks.
const PROGRESS_WINDOW: usize = 200;

/// Cells that must be cleared within a [`PROGRESS_WINDOW`] for the front to count as working.
/// At 0.1 mm cells, 400 cells is 4 mm² — a couple of steps' worth of honest cutting, so this
/// only fires on a front that has genuinely stopped.
const PROGRESS_MIN_CELLS: usize = 400;

/// Guard on total steps spent hunting rather than cutting, across the whole run. Air time is
/// not free and a generator that spends more of it than this is not converging.
const MAX_HUNT_TOTAL: usize = 40_000;

/// Directions tried when placing the tool to restart a front beside uncut material.
const PLACE_SAMPLES: usize = 32;

/// Lattice spacing (mm) for uncut-material candidates when seeding a new front.
const SEED_STRIDE: f64 = 1.0;

/// How many candidates a seed search will consider before giving the region up.
const SEED_CANDIDATES: usize = 400;

/// Fraction of the target bite a seed's first step must achieve to count as a live front.
const SEED_BITE: f64 = 0.4;

/// A seed must also have a turn this light available — an escape from being buried.
const SEED_ESCAPE: f64 = 0.35;

/// How many steps of the real control rule a candidate seed must survive before it is
/// accepted.
///
/// **This is the number that closed the last gap**, and only once the simulation was faithful.
/// While the lookahead differed from the front in any way — a different turn rule, no hunt aim,
/// stopping at the first starving step — deepening it did *nothing at all*: 12, 30 and 60 steps
/// gave a peak of 3.83 to the last decimal, three times over. With the rule shared and the hunt
/// simulated, depth suddenly bites: 12 → 3.42, and **25 → 2.07 with nothing buried**, which
/// certifies. That progression is the whole lesson of this module.
const SEED_LOOKAHEAD: usize = 25;

/// How far over the target a step may read and still count as "holding".
const OK_SLACK: f64 = 1.05;

/// Which side the front keeps its material on: `+1` turns toward increasing angle (the
/// offset winding's sense, i.e. climb), `-1` the other way. Not a preference — reversing it
/// reverses the cut direction, and adaptive clearing inherits climb the same way
/// [`crate::frontadvance`] does.
const SIDE: f64 = 1.0;

/// Turn angles sampled across the range before refining. Enough to find the feasible
/// sub-range when a wall or an island cuts the range in two, which a plain bracket cannot.
const SAMPLES: usize = 24;

/// Rotate the unit vector `d` by `t` radians.
fn rotate(d: (f64, f64), t: f64) -> (f64, f64) {
    let (c, s) = (t.cos(), t.sin());
    (d.0 * c - d.1 * s, d.0 * s + d.1 * c)
}

/// How a steered run is seeded. Bundled because every field of it is something a finished
/// generator would **find** — the entry point, the opening helix sized to fit, the direction
/// to set off in — and which the Stage 1 probe is handed instead.
pub(crate) struct Seed {
    /// Where the tool plunges.
    pub(crate) start: Point,
    /// The direction it sets off in.
    pub(crate) dir: (f64, f64),
    /// Radius of the pocket opened about `start` before steering begins.
    pub(crate) open_r: f64,
    /// How many times a dead front may be resumed elsewhere.
    pub(crate) resume_budget: usize,
}

/// The outcome of one steered run, for the probe's reporting.
pub(crate) struct SteerRun {
    /// The tool-centre moves — `(point, is_cut)`, so a jump to a new front is a rapid
    /// rather than a line drawn across the part.
    pub(crate) path: Vec<(Point, bool)>,
    /// How many times a dead front was resumed elsewhere.
    pub(crate) resumes: usize,
    /// How many steps ended up over the target because no turn could avoid it.
    pub(crate) buried_steps: usize,
    /// Steps spent hunting rather than cutting at target.
    pub(crate) starved_steps: usize,
    /// Path length (mm) of moves that actually removed material.
    pub(crate) cut_len: f64,
    /// Path length (mm) of moves that cut nothing — air, at feed rate.
    pub(crate) air_len: f64,
    /// Per move of `path`: was this step hunting (air) rather than cutting at target?
    pub(crate) hunting: Vec<bool>,
    /// Per move of `path`: does this move remove **nothing at all**?
    ///
    /// Not the same as `hunting`. A starving step may still shave up to `STALL_FRAC` of the
    /// target — real material, at a real feed. Only a move measured to remove *zero* is safe
    /// to emit as a traverse, and that is the distinction that lets the emitter turn air into
    /// a rapid without turning a light cut into one.
    pub(crate) air: Vec<bool>,
    /// Re-entries: each costs a retract, a reposition and a plunge.
    pub(crate) reentries: usize,
    /// Why the run ended.
    pub(crate) stopped: &'static str,
}

/// The points of an Archimedean spiral opening a pocket of radius `open_r` about `c`, clipped
/// to the tool-centre region.
///
/// **This is the one entry primitive whose bite is fixed by construction.** A spiral of pitch
/// `p` exposes an annulus `p` wide per revolution, so its radial width of cut *is* `p`, chosen
/// here at `e/2`. Contrast the trochoidal entry loop tried four times and rejected: for a
/// trochoid in steady state the instantaneous bite is `a·cos θ` — the advance per revolution —
/// but an *entry* loop has no previous revolution to subtract from, so its first pass takes the
/// whole swept band `R + r − c` at once, which nothing in the loop's parameters bounds. That is
/// why shrinking the advance made engagement *worse* rather than better: each extra loop was
/// another entry bite.
fn open_pocket(tc: &Polygon, c: Point, open_r: f64, e: f64) -> Vec<Point> {
    let turns = ((open_r / (0.5 * e)).ceil() as usize).max(1);
    let steps = turns * 48;
    let mut out = Vec::with_capacity(steps);
    for k in 1..=steps {
        let t = k as f64 / steps as f64;
        let ang = std::f64::consts::TAU * turns as f64 * t;
        let rad = open_r * t;
        let q = Point::new(c.x + rad * ang.cos(), c.y + rad * ang.sin());
        // The entry is bound by `tc` like every other move. It was not, and at small tool
        // radii the opening spiral walked outside the part — the only gouge left in the
        // measurements (9–16 mm² at Ø3–Ø4, none at Ø5–Ø6, which is exactly the pattern a
        // fixed-radius opening in a shrinking tool-centre region produces).
        if tc.contains(q) {
            out.push(q);
        }
    }
    out
}

/// How big an opening fits about `c`, by the rule the initial entry has always used.
///
/// Size the opening to what actually fits, rather than assuming. A fixed radius is fine in the
/// middle of a wide pocket and wrong in a narrow band, where it would be clipped away against
/// `tc` and leave the front to start from a hole too small to steer out of.
fn opening_radius(tc: &Polygon, c: Point, e: f64) -> f64 {
    let room = crate::frontadvance::boundary_dist(tc, c);
    (1.5 * e).min((room - 0.5 * e).max(0.25 * e))
}

/// How far clear of uncut stock a move must sweep before it may be emitted as a traverse, in mm.
///
/// Two discretisations have to be paid for: the generator's own occupancy grid (0.1 mm cells,
/// which can mark a cell cleared when the tool only clipped it) and `cam-sim`'s heightfield
/// (0.5 mm cells — half-diagonal 0.35 mm — where a cell whose centre escaped every cut still
/// stands at full stock height). 0.6 mm covers both with room to spare.
const AIR_MARGIN: f64 = 0.6;

/// Steer a path that clears `region` (less a `finish` skin) at radial width of cut `e`,
/// starting at `start` heading `dir0`, having already opened a pocket of radius `open_r`
/// about the start.
///
/// The entry pocket is a **given**, not something this solves: leaving a bare plunge hole
/// the tool is surrounded by material and *every* direction is a full-width cut, which is
/// why real implementations open with a helix sized to fit. The probe hands it one.
pub(crate) fn steer_path(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    seed: Seed,
    cancel: &CancelToken,
) -> Option<SteerRun> {
    let Seed { start, dir: dir0, open_r, resume_budget } = seed;
    if !(e > 0.0 && e < 2.0 * r) {
        return None;
    }
    let to_clear = crate::steer::largest(
        offset(std::slice::from_ref(region), -finish, JoinStyle::Round).ok()?,
    )?;
    // Tool centres may not leave this: inside `tc` ⇔ the tool is inside the part.
    let tc = largest(offset(std::slice::from_ref(region), -(r + finish), JoinStyle::Round).ok()?)?;
    let mut model = ClearedModel::bounded(r, to_clear.clone());

    let h = STEP_RADII * r;
    let target = e; // radial width of cut the step should hold — the gate's own measure

    // Open the entry pocket: an Archimedean spiral out to `open_r`, committed so the model
    // knows the material is gone. Cheap stand-in for the helical entry a real generator
    // would size to the region.
    let mut path: Vec<(Point, bool)> = vec![(start, false)];
    let mut hunting: Vec<bool> = vec![false];
    let mut air_flags: Vec<bool> = vec![false];
    model.seed_disc(start);
    {
        let mut prev = start;
        for q in open_pocket(&tc, start, open_r, e) {
            model.commit(prev, q);
            path.push((q, true));
            hunting.push(false);
            air_flags.push(false);
            prev = q;
        }
    }

    let mut p = path.last()?.0;
    let mut d = {
        let l = dir0.0.hypot(dir0.1).max(1e-12);
        (dir0.0 / l, dir0.1 / l)
    };
    let mut buried_steps = 0usize;
    let mut resumes = 0usize;
    let mut starved = 0usize;
    let mut starved_steps = 0usize;
    let mut tried: Vec<Point> = Vec::new();
    let mut cut_len = 0.0_f64;
    let mut air_len = 0.0_f64;
    let mut force_reseed = false;
    let mut cells_at_check = model.cleared_cells();
    let mut next_check = path.len() + PROGRESS_WINDOW;
    let mut stopped = "max steps";

    for step_no in 0..MAX_STEPS {
        // **Polled, unlike the generators before it.** This one can run for seconds on a real
        // pocket, and an operator who asks a long job to stop is entitled to have it stop.
        if step_no % 64 == 0 && cancel.is_cancelled() {
            stopped = "cancelled";
            break;
        }
        if starved_steps > MAX_HUNT_TOTAL {
            stopped = "hunt budget spent";
            break;
        }
        // **Progress, not starvation, is what decides a front is finished.** Hunting was
        // meant to rescue a front that had merely turned the wrong way, but it also lets one
        // wander for ever: any crumb it cuts works the starve counter back down, so the
        // counter never reaches its limit and the front never yields to a re-seed. Measured,
        // raising the hunt budget from 6 000 steps to 40 000 grew a run from 13 000 moves to
        // **98 000 and left the uncut area identical to the digit** — 289 mm² either way. The
        // leftovers needed a *seed*, and hunting was standing in the way of asking for one.
        //
        // So watch cells cleared instead. A window that clears almost nothing is a finished
        // front whatever its counter says.
        if path.len() >= next_check {
            let now = model.cleared_cells();
            // An explicit flag, not a nudge to the starve counter: a non-starving step
            // decays that counter on the very next line, so forcing it there was silently
            // undone unless the step happened to be starving too — which is why two shapes
            // still ran to 98 000 moves with the guard apparently in place.
            force_reseed = now.saturating_sub(cells_at_check) < PROGRESS_MIN_CELLS;
            cells_at_check = now;
            next_check = path.len() + PROGRESS_WINDOW;
        }
        let Some(step) = decide_step(&model, &tc, p, d, h, r, target) else {
            // No feasible turn at all — boxed in against a wall.
            let Some((c, dd)) = find_seed(&mut model, &tc, r, h, target, p, &tried) else {
                stopped = "region clear";
                break;
            };
            resumes += 1;
            if resumes > resume_budget {
                stopped = "resume budget spent";
                break;
            }
            tried.push(c);
            (d, p, starved) = (dd, c, 0);
            model.seed_disc(p);
            path.push((p, false));
            hunting.push(false);
            air_flags.push(false);
            continue;
        };

        if step.starving {
            starved += 1;
            starved_steps += 1;
        } else {
            starved = starved.saturating_sub(2);
        }
        {
            if force_reseed || (step.starving && starved >= STARVE_LIMIT) {
                let Some((c, dd)) = find_seed(&mut model, &tc, r, h, target, p, &tried) else {
                    stopped = "region clear";
                    break;
                };
                resumes += 1;
                if resumes > resume_budget {
                    stopped = "resume budget spent";
                    break;
                }
                tried.push(c);
                (d, p, starved) = (dd, c, 0);
                force_reseed = false;
                cells_at_check = model.cleared_cells();
                next_check = path.len() + PROGRESS_WINDOW;
                model.seed_disc(p);
                path.push((p, false));
                hunting.push(false);
                air_flags.push(false);
                continue;
            }
        }
        if step.buried {
            buried_steps += 1;
        }

        let best_t = step.turn;

        let dd = rotate(d, best_t);
        let q = Point::new(p.x + h * dd.0, p.y + h * dd.1);
        if step.starving {
            air_len += h;
        } else {
            cut_len += h;
        }
        // **Before** the commit: afterwards this move's own swath reads as cleared and every
        // move would be flagged as air.
        air_flags.push(model.sweeps_only_cleared(p, q, AIR_MARGIN));
        model.commit(p, q);
        path.push((q, true));
        hunting.push(step.starving);
        p = q;
        d = dd;
    }

    // The flags are read by index against `path`; a `path.push` without its pair silently
    // misattributes every later move. That is not hypothetical — the re-seed branch was
    // missing both pushes, and the resulting drift made a third of the emitted traverses
    // point at the wrong move, standing stock included.
    debug_assert_eq!(path.len(), air_flags.len());
    debug_assert_eq!(path.len(), hunting.len());
    Some(SteerRun { path, buried_steps, resumes, starved_steps, cut_len, air_len, hunting, air: air_flags, reentries: resumes, stopped })
}

/// One step's decision: which way to turn, and what kind of step it is.
#[derive(Clone, Copy)]
struct Step {
    turn: f64,
    /// The engagement the chosen turn actually achieves.
    got: f64,
    /// Nothing light enough was available — the tool is buried.
    buried: bool,
    /// Nothing worth cutting was available — the front is starving and should hunt.
    starving: bool,
}

/// **The whole step decision, in one place** — sample the feasible turns, choose one, refine
/// it. Used both to drive the front and to decide whether a candidate seed's front would
/// survive, and it exists as one function because those two must be *identical*.
///
/// They were not, twice, and each time the symptom was a lookahead that validated a
/// trajectory the generator would never take: deepening it from 3 steps to 12 moved the peak
/// by exactly nothing. Nor can the divergence simply be removed by dropping the refinement —
/// tried, and the front then spins in place, clearing a 6 × 5 mm patch in 100 000 moves. The
/// refinement is load-bearing, so both callers have to run it.
fn decide_step(
    model: &ClearedModel,
    tc: &Polygon,
    p: Point,
    d: (f64, f64),
    h: f64,
    r: f64,
    target: f64,
) -> Option<Step> {
    let step_to = |dir: (f64, f64), t: f64| -> Point {
        let dd = rotate(dir, t);
        Point::new(p.x + h * dd.0, p.y + h * dd.1)
    };
    // **Feasibility first, then engagement.** The controller's measure counts only *material*,
    // so an island and the air past the wall both read as "nothing to cut" — indistinguishable
    // from cleared stock. Steering on that alone drove the probe across an island and gouged
    // 648 mm². Tool-centre containment is a hard constraint, not a term to trade against the
    // target bite.
    let samples: Vec<(f64, f64)> = (0..=SAMPLES)
        .map(|k| { let m = max_turn(h, r); -m + 2.0 * m * (k as f64 / SAMPLES as f64) })
        .filter(|&t| tc.contains(step_to(d, t)))
        .map(|t| (t, model.engagement_grid(p, step_to(d, t))))
        .collect();
    if samples.is_empty() {
        return None;
    }
    if samples.iter().all(|&(_, a)| a < STALL_FRAC * target) {
        // **Starving is not dying — hunt, but hunt legally.** Turn toward whatever material
        // there is and keep stepping through cleared stock. The turn must still come from the
        // *feasible* set: an earlier version returned a bare zero here, and a starving tool
        // then drove straight ahead out of the region — 75 mm² of gouge from one missing
        // filter.
        // The aim is computed **here**, not passed in, so the seed lookahead cannot possibly
        // hunt differently from the front it is predicting. Passing it in was the last place
        // the two rules diverged: the lookahead had no aim and simply *stopped* at the first
        // starving step, while the real front hunted onward and could be buried later — which
        // is why deepening the lookahead from 12 steps to 60 changed the peak by nothing at
        // all, three times in a row.
        let want = model
            .nearest_uncut(p, HUNT_RADII * r)
            .map(|t| {
                let w = (t.x - p.x, t.y - p.y);
                let wl = w.0.hypot(w.1).max(1e-12);
                let (wx, wy) = (w.0 / wl, w.1 / wl);
                (d.0 * wy - d.1 * wx).atan2(d.0 * wx + d.1 * wy)
            })
            .unwrap_or(0.0);
        let turn = samples
            .iter()
            .fold((samples[0].0, f64::MAX), |acc, &(t, _)| {
                let err = (t - want).abs();
                if err < acc.1 {
                    (t, err)
                } else {
                    acc
                }
            })
            .0;
        return Some(Step { turn, got: 0.0, buried: false, starving: true });
    }
    let ok_max = target * OK_SLACK;
    let (mut turn, buried) = choose_turn(&samples, ok_max);
    // Refine between the chosen turn and the first neighbour that does not hold.
    if let Some(&(t2, _)) = samples.iter().find(|&&(t, a)| t != turn && a > ok_max) {
        let (mut lo, mut hi) = (turn.min(t2), turn.max(t2));
        for _ in 0..SEARCH_ITERS {
            let mid = 0.5 * (lo + hi);
            let q = step_to(d, mid);
            if !tc.contains(q) {
                break;
            }
            if model.engagement_grid(p, q) <= ok_max {
                if turn < t2 { lo = mid } else { hi = mid }
            } else if turn < t2 {
                hi = mid
            } else {
                lo = mid
            }
        }
        turn = if turn < t2 { lo } else { hi };
    }
    let got = model.engagement_grid(p, step_to(d, turn));
    Some(Step { turn, got, buried, starving: false })
}

/// **The control rule**, in one place.
///
/// Given the feasible `(turn, engagement)` samples, pick the turn. Extracted because it is
/// used twice — once to drive the front, once to decide whether a candidate seed's front
/// would survive — and those two *must* be the same rule. They were not, and the seed
/// lookahead spent its effort validating a trajectory the generator would never take:
/// deepening it from 3 steps to 12 changed the peak by exactly nothing (3.93 either way),
/// which is the signature of simulating the wrong thing.
///
/// Returns the turn and whether the front was buried (nothing light enough available).
fn choose_turn(samples: &[(f64, f64)], ok_max: f64) -> (f64, bool) {
    let holding: Vec<(f64, f64)> = samples.iter().copied().filter(|&(_, a)| a <= ok_max).collect();
    if holding.is_empty() {
        let t = samples
            .iter()
            .fold((samples[0].0, f64::MAX), |acc, &(t, a)| if a < acc.1 { (t, a) } else { acc })
            .0;
        return (t, true);
    }
    // Among the turns that hold, the one turning hardest toward the material side, so the
    // front orbits the cleared region rather than wandering across its own track.
    let t = holding
        .iter()
        .fold((holding[0].0, f64::MIN), |acc, &(t, _)| {
            if t * SIDE > acc.1 * SIDE {
                (t, t)
            } else {
                acc
            }
        })
        .0;
    (t, false)
}

#[cfg(test)]
thread_local! {
    /// `(candidate placements examined, rejected by the lookahead, accepted)` — for
    /// `seed_acceptance`, which asks whether a tighter turn floor starves the generator of
    /// *places to start* rather than of the ability to cut.
    static SEED_TRACE: std::cell::RefCell<(usize, usize, usize)> =
        const { std::cell::RefCell::new((0, 0, 0)) };
}

/// Find somewhere to restart a dead front: a tool-centre position and a heading.
///
/// Walks uncut-material candidates outward from `near` and, for each, tries standing points
/// on a ring around it. A seed is accepted only when **all** of these hold, each of which was
/// learned by watching one of them fail:
///
/// - `tc.contains(c)` — outside the tool-centre region is not a place the tool can be.
/// - `model.is_cleared_at(c)` — otherwise the re-entry plunges into solid stock.
/// - the first step stays inside `tc` **and** bites at least a fraction of the target —
///   otherwise the new front is stillborn and the search comes straight back to it.
///
/// Places already tried are skipped, so a seed that turns out badly is not chosen again.
fn find_seed(
    model: &mut ClearedModel,
    tc: &Polygon,
    r: f64,
    h: f64,
    target: f64,
    near: Point,
    tried: &[Point],
) -> Option<(Point, (f64, f64))> {
    for m in model.uncut_candidates(near, SEED_STRIDE, SEED_CANDIDATES) {
        for ring in [1.0, 1.5, 2.0] {
            let rho = r + ring * h;
            for k in 0..PLACE_SAMPLES {
                let a = std::f64::consts::TAU * (k as f64) / (PLACE_SAMPLES as f64);
                let c = Point::new(m.x + rho * a.cos(), m.y + rho * a.sin());
                if tried.iter().any(|t| t.distance(c) < 2.0 * h) {
                    continue;
                }
                if !tc.contains(c) || !model.is_cleared_at(c) {
                    continue;
                }
                let dir = (m.x - c.x, m.y - c.y);
                let l = dir.0.hypot(dir.1).max(1e-12);
                let face = (dir.0 / l, dir.1 / l);

                // **A seed must start a front that can be *continued*, not merely one that
                // cuts.** Bounding only the first step's bite was measured putting the peak at
                // 5.95 — and the peak sat **two steps after the rapid**, not on it: the tool
                // started fine and was then buried, because the material wrapped it and no
                // turn was light. So survey the whole turn range from here and require both
                // that there is material to take *and* a light direction to escape into.
                let mut bites: Vec<(f64, f64)> = Vec::new();
                for k in 0..=SAMPLES {
                    let m = max_turn(h, r);
                    let t = -m + 2.0 * m * (k as f64 / SAMPLES as f64);
                    let dd = rotate(face, t);
                    let q = Point::new(c.x + h * dd.0, c.y + h * dd.1);
                    if tc.contains(q) {
                        bites.push((t, model.engagement_grid(c, q)));
                    }
                }
                if bites.is_empty() {
                    continue;
                }
                let hi = bites.iter().map(|&(_, a)| a).fold(0.0_f64, f64::max);
                let lo = bites.iter().map(|&(_, a)| a).fold(f64::MAX, f64::min);
                if hi < SEED_BITE * target || lo > SEED_ESCAPE * target {
                    continue; // nothing to cut, or nowhere light to go from here
                }
                // Set off already steering: the turn whose bite is nearest the target.
                let (best_t, _) = bites.iter().fold((0.0, f64::MAX), |acc, &(t, a)| {
                    let err = (a - target).abs();
                    if err < acc.1 {
                        (t, err)
                    } else {
                        acc
                    }
                });
                let head = rotate(face, best_t);

                // **Simulate it.** Every heuristic tried here failed in the same way: the
                // seed's *first* step was fine and the front was buried on the second or
                // third, so the peak sat one or two steps after a rapid however the first
                // step was bounded (3.62, then 5.82). The only thing that answers "will this
                // front survive?" is running it — so run the real control rule for a few
                // steps against the real model, recording the cells so it can be undone.
                let mut undo: Vec<usize> = Vec::new();
                let (mut sp, mut sd) = (c, head);
                let mut survives = true;
                model.seed_disc_recording(sp, &mut undo);
                for _ in 0..SEED_LOOKAHEAD {
                    let Some(st) = decide_step(model, tc, sp, sd, h, r, target) else {
                        survives = false;
                        break;
                    };
                    if st.buried {
                        survives = false;
                        break;
                    }
                    // Starving steps are simulated too, not treated as a happy ending: the
                    // real front hunts through them and may be buried on the far side.
                    sd = rotate(sd, st.turn);
                    let nq = Point::new(sp.x + h * sd.0, sp.y + h * sd.1);
                    model.commit_recording(sp, nq, &mut undo);
                    sp = nq;
                }
                model.rollback(&undo);
                #[cfg(test)]
                SEED_TRACE.with(|t| {
                    let mut t = t.borrow_mut();
                    t.0 += 1;
                    if survives { t.2 += 1 } else { t.1 += 1 }
                });
                if !survives {
                    continue;
                }
                return Some((c, head));
            }
        }
    }
    None
}

/// Steer a whole region: pick the entry the way [`crate::frontadvance`] does, open a pocket,
/// and clear. The generator-shaped entry point, as opposed to the hand-seeded probe.
pub(crate) fn steer_region(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
    cancel: &CancelToken,
) -> Option<SteerRun> {
    let tc = largest(offset(std::slice::from_ref(region), -(r + finish), JoinStyle::Round).ok()?)?;
    let entry = start
        .map(|s| Point::new(s[0], s[1]))
        .filter(|p| tc.contains(*p))
        .or_else(|| crate::frontadvance::entry_point(&tc))?;
    let open_r = opening_radius(&tc, entry, e);
    steer_path(
        region,
        r,
        finish,
        e,
        Seed { start: entry, dir: (1.0, 0.0), open_r, resume_budget: 400 },
        cancel,
    )
}

/// [`steer_region`], certified against the exact oracle — the entry [`crate::clearing::clear`]
/// dispatches.
///
/// Same contract as [`crate::frontadvance::front_advance_certified`], and deliberately the same
/// tolerances: the path is returned only if it holds the radial width of cut at the
/// geometric-floor bound, covers the reachable target, and never gouges. `None` means *fall
/// back*, so a shape this cannot clear costs a wasted generation and nothing else.
///
/// That contract is what makes the one shape it declines a non-event. A pocket whose island
/// leaves a strip exactly the tool's diameter cannot be cleared at a bounded width of cut by
/// anything; this generator now leaves that strip **uncut** rather than slotting it, the
/// coverage check sees it, and the operator gets the proven concentric clear.
/// A certified steered path: the moves (with their cut/reposition flag) and, per move, whether
/// it removes nothing and may be traversed.
pub(crate) type CertifiedPath = (Vec<(Point, bool)>, Vec<bool>);

pub(crate) fn steer_certified(
    region: &Polygon,
    r: f64,
    finish: f64,
    e: f64,
    start: Option<[f64; 2]>,
    cancel: &CancelToken,
) -> Option<CertifiedPath> {
    let run = steer_region(region, r, finish, e, start, cancel)?;
    if run.stopped == "cancelled" {
        return None; // a half-cleared region must never be emitted as if it were finished
    }
    let to_clear = largest(offset(std::slice::from_ref(region), -finish, JoinStyle::Round).ok()?)?;
    let reach = crate::clearsim::reachable(&to_clear, r);
    if reach.is_empty() {
        return None;
    }
    let cover_tol = 0.02 * reach.iter().map(Polygon::area).sum::<f64>() + 1.0;
    let v = crate::clearsim::certify_moves(&run.path, r, &to_clear);
    let ok = v.max_engagement <= e * crate::frontadvance::CERT_ENGAGEMENT_SLACK
        && v.uncut_area <= cover_tol
        && v.gouge_area <= cover_tol;
    ok.then_some((run.path, run.air))
}

/// The largest polygon by area.
fn largest(polys: Vec<Polygon>) -> Option<Polygon> {
    polys
        .into_iter()
        .max_by(|a, b| a.area().partial_cmp(&b.area()).unwrap_or(std::cmp::Ordering::Equal))
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_geo::Contour;

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Contour {
        Contour::new(vec![
            Point::new(x0, y0),
            Point::new(x1, y0),
            Point::new(x1, y1),
            Point::new(x0, y1),
        ])
    }

    /// **Stage 3: does it hold across shapes?** Every island case the pass-based frontier
    /// was measured on, plus hole-free controls, through the same certification predicate the
    /// dispatcher would use.
    #[test]
    #[ignore = "measurement harness for ADAPTIVE_PLAN.md §10"]
    fn steered_over_every_shape() {
        let (r, e) = (3.0, 2.0);
        let cases: Vec<(&str, Polygon)> = vec![
            ("CONTROL square 40", Polygon::new(rect(0.0, 0.0, 40.0, 40.0)).unwrap()),
            ("CONTROL circle r30", {
                let pts = (0..96)
                    .map(|i| {
                        let a = std::f64::consts::TAU * (i as f64) / 96.0;
                        Point::new(30.0 * a.cos(), 30.0 * a.sin())
                    })
                    .collect();
                Polygon::new(Contour::new(pts)).unwrap()
            }),
            (
                "square 40, island 12 centred",
                Polygon::with_holes(rect(0.0, 0.0, 40.0, 40.0), vec![rect(14.0, 14.0, 26.0, 26.0)])
                    .unwrap(),
            ),
            (
                "square 40, island 12 offset",
                Polygon::with_holes(rect(0.0, 0.0, 40.0, 40.0), vec![rect(22.0, 14.0, 34.0, 26.0)])
                    .unwrap(),
            ),
            (
                "circle r20, island r6",
                {
                    let ring = |rad: f64, n: usize| {
                        Contour::new(
                            (0..n)
                                .map(|i| {
                                    let a = std::f64::consts::TAU * (i as f64) / (n as f64);
                                    Point::new(rad * a.cos(), rad * a.sin())
                                })
                                .collect(),
                        )
                    };
                    Polygon::with_holes(ring(20.0, 64), vec![ring(6.0, 32)]).unwrap()
                },
            ),
            (
                "square 60, island 20 centred",
                Polygon::with_holes(rect(0.0, 0.0, 60.0, 60.0), vec![rect(20.0, 20.0, 40.0, 40.0)])
                    .unwrap(),
            ),
            (
                "square 60, two islands",
                Polygon::with_holes(
                    rect(0.0, 0.0, 60.0, 60.0),
                    vec![rect(12.0, 12.0, 24.0, 24.0), rect(36.0, 36.0, 48.0, 48.0)],
                )
                .unwrap(),
            ),
        ];
        println!(
            "\n| case | moves | a_e | uncut/tol | cut mm | air mm | air % | re-entries | est time | verdict |"
        );
        println!("|---|---|---|---|---|---|---|---|---|---|");
        for (name, region) in cases {
            let t0 = std::time::Instant::now();
            let Some(run) = steer_region(&region, r, 0.0, e, None, &CancelToken::new()) else {
                println!("| {name} | — | no path | — | — | — | — | — | no |");
                continue;
            };
            let secs = t0.elapsed().as_secs_f64();
            let v = crate::clearsim::certify_moves(&run.path, r, &region);
            let reach: f64 =
                crate::clearsim::reachable(&region, r).iter().map(|p| p.area()).sum();
            let tol = 0.02 * reach + 1.0;
            let ok = v.max_engagement <= 1.5 * e && v.uncut_area <= tol && v.gouge_area <= tol;
            // **Estimated cycle time, not path length.** Length was the wrong quantity: a
            // re-entry costs a retract, a cross and a plunge at *plunge* feed, and tuning to
            // minimise air travel quietly bought that overhead instead. On a real exported
            // part this showed up as 168 plunges totalling 504 mm at F100 — **5 minutes of
            // plunging** against 6.5 minutes of actual cutting.
            let (feed, plunge_feed, rapid) = (300.0, 100.0, 5000.0);
            let plunge_depth = 4.0; // retract-to-floor per re-entry, mm
            let mins = run.cut_len / feed
                + run.air_len / feed
                + (run.reentries as f64) * plunge_depth / plunge_feed
                + (run.reentries as f64) * 20.0 / rapid;
            let total = run.cut_len + run.air_len;
            println!(
                "| {name} | {} | **{:.2}** | {:.0}/{:.0} | {:.0} | {:.0} | {:.0}% | {} | \
                 **{:.1} min** | {} |",
                run.path.len(),
                v.max_engagement,
                v.uncut_area,
                tol,
                run.cut_len,
                run.air_len,
                100.0 * run.air_len / total.max(1e-9),
                run.reentries,
                mins,
                if ok { "**CERTIFIED**" } else { "no" },
            );
            let _ = (v.gouge_area, secs);
        }
        println!();
    }

    /// Diagnostic: is the seed *search* what a tighter turn floor starves? The shape table
    /// already hints it — at 0.5·r the same shapes take 8–49 re-entries where 0.25·r takes
    /// 55–122, while abandoning 800–1400 mm² — but a hint read off a summary column is not a
    /// measurement. `find_seed` validates every candidate with the same bounded rule the front
    /// uses, so a tighter floor should show up here as a rejection rate.
    #[test]
    #[ignore = "diagnostic"]
    fn seed_acceptance() {
        let (r, e) = (3.0, 2.0);
        let region =
            Polygon::with_holes(rect(0.0, 0.0, 60.0, 40.0), vec![rect(25.0, 15.0, 35.0, 25.0)])
                .unwrap();
        SEED_TRACE.with(|t| *t.borrow_mut() = (0, 0, 0));
        let run = steer_region(&region, r, 0.0, e, None, &CancelToken::new()).expect("a path");
        let (tried, rejected, accepted) = SEED_TRACE.with(|t| *t.borrow());
        println!(
            "\nMIN_TURN_RADII = {MIN_TURN_RADII}·r\n  \
             seed placements validated {tried}: accepted {accepted}, rejected {rejected} \
             ({:.0}%)\n  path {} moves, stopped: {}\n",
            100.0 * rejected as f64 / tried.max(1) as f64,
            run.path.len(),
            run.stopped,
        );
    }

    /// Diagnostic: **why** does a machine-friendly turn floor break the generator? Two
    /// trigger designs for the trochoidal loop were built on guesses about that and neither
    /// fired at all. This asks the generator instead: at each step, what bite did it actually
    /// get, and was there material on the tool to orient a loop by?
    #[test]
    #[ignore = "diagnostic"]
    fn why_the_radius_floor_breaks_it() {
        let (r, e) = (3.0, 2.0);
        let region =
            Polygon::with_holes(rect(0.0, 0.0, 60.0, 40.0), vec![rect(25.0, 15.0, 35.0, 25.0)])
                .unwrap();
        let tc = largest(offset(std::slice::from_ref(&region), -r, JoinStyle::Round).unwrap())
            .unwrap();
        let to_clear = region.clone();
        let mut model = ClearedModel::bounded(r, to_clear);
        let h = STEP_RADII * r;
        let entry = crate::frontadvance::entry_point(&tc).unwrap();
        model.seed_disc(entry);
        // Walk one front by the real rule and record what it sees.
        let (mut p, mut d) = (entry, (1.0, 0.0));
        let (mut bands, mut bearing_when_low, mut steps) = ([0usize; 5], 0usize, 0usize);
        for _ in 0..4000 {
            let Some(st) = decide_step(&model, &tc, p, d, h, r, e) else { break };
            if st.starving {
                bands[0] += 1;
                if model.material_bearing(p).is_some() {
                    bearing_when_low += 1;
                }
                break; // one front only — this is about how it dies
            }
            let f = st.got / e;
            let b = if f < 0.3 { 0 } else if f < 0.6 { 1 } else if f < 0.9 { 2 }
                    else if f < 1.1 { 3 } else { 4 };
            bands[b] += 1;
            if f < 0.6 && model.material_bearing(p).is_some() {
                bearing_when_low += 1;
            }
            d = rotate(d, st.turn);
            let q = Point::new(p.x + h * d.0, p.y + h * d.1);
            model.commit(p, q);
            p = q;
            steps += 1;
        }
        println!("\nMIN_TURN_RADII = {MIN_TURN_RADII}·r, one front, {steps} steps before it died");
        for (i, l) in ["<0.3", "0.3-0.6", "0.6-0.9", "0.9-1.1", ">1.1"].iter().enumerate() {
            println!("  bite {l:>8} × target: {:5} steps", bands[i]);
        }
        println!("  steps with a low bite *and* material still on the tool: {bearing_when_low}");
        println!("  (that count is what a loop trigger would have to fire on)\n");
    }

    /// Diagnostic: how sharp is the path? A constant-engagement path is only worth having
    /// if the machine can actually run it at feed, and a corner forces the controller to
    /// decelerate — so the turn per step, and the radius it implies, are as much a part of
    /// the quality of this path as the engagement is.
    #[test]
    #[ignore = "diagnostic"]
    fn turn_sharpness() {
        let (r, e) = (3.0, 2.0);
        let region =
            Polygon::with_holes(rect(0.0, 0.0, 60.0, 40.0), vec![rect(25.0, 15.0, 35.0, 25.0)])
                .unwrap();
        let run = steer_region(&region, r, 0.0, e, None, &CancelToken::new()).expect("a path");
        // **Measure the radius each corner actually has, from the chords either side of it.**
        // This used to divide a constant `STEP_RADII·r` by the turn angle, which is right only
        // while every move is a front step. It is not: an opening spiral's chords are ~0.4 mm,
        // so a constant 0.75 mm numerator reported roughly twice the radius those corners have.
        // And a bare percentage is diluted by however many moves a variant happens to emit, so
        // the rate per metre of path is reported alongside it — that is the number a machine
        // feels, since every sharp corner is one deceleration.
        struct Corner {
            radius: f64,
            cutting: bool,
        }
        let mut corners: Vec<Corner> = Vec::new();
        let mut path_mm = 0.0_f64;
        let idx: Vec<usize> = (0..run.path.len()).filter(|&i| run.path[i].1).collect();
        let pts: Vec<Point> = idx.iter().map(|&i| run.path[i].0).collect();
        let hunt: Vec<bool> =
            idx.iter().map(|&i| run.hunting.get(i).copied().unwrap_or(false)).collect();
        for (k, w) in pts.windows(3).enumerate() {
            let (a, b, c) = (w[0], w[1], w[2]);
            let (ux, uy) = (b.x - a.x, b.y - a.y);
            let (vx, vy) = (c.x - b.x, c.y - b.y);
            let (lu, lv) = (ux.hypot(uy), vx.hypot(vy));
            if lu < 1e-9 || lv < 1e-9 {
                continue;
            }
            path_mm += lv;
            let ang = ((ux * vy - uy * vx) / (lu * lv)).clamp(-1.0, 1.0).asin().abs();
            // The circle through the three points: chord over twice the sine of the half-turn.
            let radius = if ang > 1e-6 {
                0.5 * (lu + lv) / (2.0 * (ang / 2.0).sin())
            } else {
                f64::INFINITY
            };
            corners.push(Corner { radius, cutting: !hunt.get(k + 1).copied().unwrap_or(false) });
        }
        let mut radii: Vec<f64> = corners.iter().map(|c| c.radius).collect();
        radii.sort_by(|a, b| a.partial_cmp(b).unwrap());
        println!("\n{} corners over {path_mm:.0} mm of path", corners.len());
        for q in [0.5, 0.1, 0.01, 0.0] {
            let v = radii[((radii.len() - 1) as f64 * q) as usize];
            println!(
                "  p{:>3.0} sharpest: corner radius {:6.2} mm ({:.2}·r)",
                (1.0 - q) * 100.0,
                v,
                v / r
            );
        }
        // A corner the tool must decelerate into: tighter than half the tool radius.
        let tight = 0.5 * r;
        for (label, want_cut) in [("cutting", true), ("hunting (air)", false)] {
            let v: Vec<&Corner> = corners.iter().filter(|c| c.cutting == want_cut).collect();
            if v.is_empty() {
                continue;
            }
            let n = v.iter().filter(|c| c.radius < tight).count();
            let mm: f64 = path_mm * v.len() as f64 / corners.len() as f64;
            println!(
                "    {label}: {} corners, {n} tighter than {tight:.2} mm ({:.1}%, {:.0} per metre)",
                v.len(),
                100.0 * n as f64 / v.len() as f64,
                1000.0 * n as f64 / mm.max(1e-9),
            );
        }
        println!();
    }

    /// **The property this module exists for**, guarded rather than merely measured: an
    /// island pocket comes out fully cleared, without gouging, holding the radial width of cut
    /// under the same bound the dispatcher certifies against. The pass-based frontier reads the
    /// **full diameter** on this shape class.
    ///
    /// Kept small deliberately — the harnesses in this module cover the larger shapes, and this
    /// one has to be affordable in a debug test run.
    #[test]
    fn a_steered_island_pocket_clears_and_holds_the_cap() {
        let (r, e) = (3.0, 2.0);
        let region =
            Polygon::with_holes(rect(0.0, 0.0, 30.0, 30.0), vec![rect(11.0, 11.0, 19.0, 19.0)])
                .unwrap();
        let run = steer_region(&region, r, 0.0, e, None, &CancelToken::new()).expect("a steered path");
        let v = crate::clearsim::certify_moves(&run.path, r, &region);
        let reach: f64 = crate::clearsim::reachable(&region, r).iter().map(|p| p.area()).sum();
        let tol = 0.02 * reach + 1.0;
        assert!(
            v.max_engagement <= 1.5 * e,
            "engagement must hold the geometric-floor bound, got {:.2}",
            v.max_engagement
        );
        assert!(v.gouge_area <= tol, "must not gouge, got {:.1}", v.gouge_area);
        assert!(
            v.uncut_area <= tol,
            "must clear the reachable target, {:.1} left of {tol:.1}",
            v.uncut_area
        );
        assert_eq!(run.stopped, "region clear", "must terminate by finishing the region");
    }

    /// **Is the offset-island failure geometry or generator?** That shape leaves a strip
    /// between the island (x ≤ 34) and the wall (x = 40) exactly **6 mm** wide — the full
    /// diameter of the Ø6 tool. A tool can only clear a slot its own width at full width, so
    /// if the explanation is right, the same shape must certify the moment the tool is smaller
    /// than the strip, with nothing else changed.
    #[test]
    #[ignore = "measurement harness for ADAPTIVE_PLAN.md §10"]
    fn the_offset_island_is_geometry_not_generator() {
        let region =
            Polygon::with_holes(rect(0.0, 0.0, 40.0, 40.0), vec![rect(22.0, 14.0, 34.0, 26.0)])
                .unwrap();
        println!("\n| tool ⌀ | strip / ⌀ | a_e | uncut/tol | gouge | verdict |");
        println!("|---|---|---|---|---|---|");
        // `e` held fixed, so the only variable is the tool against the strip.
        let e = 2.0;
        for r in [3.0_f64, 2.5, 2.0, 1.5] {
            let Some(run) = steer_region(&region, r, 0.0, e, None, &CancelToken::new()) else {
                println!("| {:.0} | — | — | — | — | no path |", 2.0 * r);
                continue;
            };
            let v = crate::clearsim::certify_moves(&run.path, r, &region);
            let reach: f64 =
                crate::clearsim::reachable(&region, r).iter().map(|p| p.area()).sum();
            let tol = 0.02 * reach + 1.0;
            let ok = v.max_engagement <= 1.5 * e && v.uncut_area <= tol && v.gouge_area <= tol;
            println!(
                "| {:.1} | {:.2} | **{:.2}** | {:.0}/{:.0} | {:.1} | {} |",
                2.0 * r,
                6.0 / (2.0 * r),
                v.max_engagement,
                v.uncut_area,
                tol,
                v.gouge_area,
                if ok { "**CERTIFIED**" } else { "no" },
            );
        }
        println!();
    }

    /// **Stage 1 probe.** The question, with a number: can an engagement-steered path hold
    /// the cap in the narrow band between an island and a wall, where the pass-based
    /// frontier reads the full diameter (6.00 against a cap of 2.00)?
    #[test]
    #[ignore = "Stage 1 probe for ADAPTIVE_PLAN.md §10"]
    fn steered_path_in_a_narrow_band() {
        let (r, e) = (3.0, 2.0);
        // The case front-advance slots on: 40 mm square, 12 mm island, an 8 mm-wide
        // tool-centre band around it.
        let region =
            Polygon::with_holes(rect(0.0, 0.0, 40.0, 40.0), vec![rect(14.0, 14.0, 26.0, 26.0)])
                .unwrap();
        // Seed in the middle of the band on the left, heading "up" along it.
        let start = Point::new(7.0, 20.0);
        let reach: f64 = crate::clearsim::reachable(&region, r).iter().map(|p| p.area()).sum();
        println!("\n| regime | moves | stopped | buried | starved | resumes | a_e | uncut/tol | gouge | verdict |");
        println!("|---|---|---|---|---|---|---|---|---|---|");
        for (label, budget) in [("one front", 0usize), ("with resumption", 400)] {
            let seed = Seed {
                start,
                dir: (0.0, 1.0),
                open_r: 1.5 * e,
                resume_budget: budget,
            };
            let run = steer_path(&region, r, 0.0, e, seed, &CancelToken::new())
                .expect("the probe produces a path");
            let v = crate::clearsim::certify_moves(&run.path, r, &region);
            let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
            for (q, _) in &run.path {
                lo[0] = lo[0].min(q.x);
                lo[1] = lo[1].min(q.y);
                hi[0] = hi[0].max(q.x);
                hi[1] = hi[1].max(q.y);
            }
            // Where does the peak actually sit, and is it a seed's doing? Walk the moves
            // against a fresh model and record the worst reading with its context.
            {
                let mut m = crate::clearsim::ClearedModel::bounded(r, region.clone());
                let (mut prev, mut prev_cut) = (None, false);
                let mut worst = (0.0_f64, Point::new(0.0, 0.0), 0usize, 0usize);
                let mut since_rapid = 0usize;
                for i in 0..run.path.len() {
                    let (q, cut) = run.path[i];
                    if !cut {
                        prev = Some(q);
                        prev_cut = false;
                        since_rapid = 0;
                        continue;
                    }
                    if let Some(pp) = prev {
                        if !prev_cut {
                            m.seed_disc(pp);
                        }
                        let a = m.engagement(pp, q);
                        if a > worst.0 {
                            worst = (a, pp, i, since_rapid);
                        }
                        m.commit(pp, q);
                    }
                    prev = Some(q);
                    prev_cut = true;
                    since_rapid += 1;
                }
                println!(
                    "    peak {:.2} at ({:.1},{:.1}), move {} of {}, {} steps after the last rapid",
                    worst.0, worst.1.x, worst.1.y, worst.2, run.path.len(), worst.3
                );
            }
            // The caller's own certification predicate, verbatim — engagement at the
            // geometric-floor bound, the reachable target covered, no gouge.
            let cover_tol = 0.02 * reach + 1.0;
            let certified = v.max_engagement <= 1.5 * e
                && v.uncut_area <= cover_tol
                && v.gouge_area <= cover_tol;
            println!(
                "| {label} | {} | {} | {} | {} | {} | **{:.2}** | {:.0}/{:.0} | {:.1} | {} |",
                run.path.len(),
                run.stopped,
                run.buried_steps,
                run.starved_steps,
                run.resumes,
                v.max_engagement,
                v.uncut_area,
                cover_tol,
                v.gouge_area,
                if certified { "**CERTIFIED**" } else { "no" },
            );
            let _ = (lo, hi);
        }
        println!("\n(cap {e:.2}, certification gate {:.2})\n", 1.5 * e);
    }
}
