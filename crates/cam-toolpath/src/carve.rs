//! The **V-carving** strategy: consume a region with a V-bit whose flanks land on the
//! region's boundary, so the carved depth is *derived from the shape*.
//!
//! ## The relation to engraving
//!
//! [`engrave`](crate::engrave) runs the tool's tip **along** a path — the path *is* the
//! cut, at a depth the operator chose. Carving instead treats the path as the **outline
//! of an area**: the tool never touches the boundary; it stands back from it and lets its
//! cone's flank meet the boundary at the surface. The same relation as profile → pocket.
//!
//! ## Why offset rings and not a medial axis
//!
//! The textbook description of V-carving is "run the medial axis, take depth from the
//! distance to the boundary". Computing an exact medial axis of a polygon with holes is
//! genuinely nasty — near-degenerate bisectors, spurious branches off boundary noise,
//! junction merges — and would be the single largest risk in this operation.
//!
//! It is also unnecessary. A point at inward distance `w` from the boundary must sit at
//! depth `vtip_depth_for_half_width(α, rt, w)`, and the **inward offset ring at distance
//! `w` is exactly the locus of points at distance `w`**. So:
//!
//! ```text
//! for w in step, 2·step, 3·step, …:
//!     ring = inward_offset(region, w)          ← cam_geo::offset, already robust
//!     cut that ring at z = top − depth_for(w)
//!     stop when the ring comes back empty      ← that emptiness IS the medial axis
//! ```
//!
//! The medial axis is never computed; it is where the offsets vanish. This is
//! contour-parallel V-carving, and it reuses the integer-grid-robust offsetter rather
//! than introducing a new algorithm.
//!
//! **The honest trade-off:** the carved surface is a staircase of rings rather than a
//! continuous ruled surface, so `ring_step` is a *finish* control, exactly like a scallop
//! tolerance. A finer step costs path length, not correctness.
//!
//! ## Linking the rings: why staying down is safe
//!
//! A carve runs to hundreds of rings, and lifting to clearance for each costs far more
//! time than the cutting does. [`CarveOp::stay_down`] links them instead — and that is
//! not a gamble, it is a theorem.
//!
//! Write `f(w)` for `vtip_depth_for_half_width` — the tool's own profile height at
//! radial offset `w`. Put the tool at inward distance `d` with its tip at the depth of
//! the ring at `w`, i.e. `f(w)`. The material it removes at inward distance `x` is
//!
//! ```text
//! cut(x) = f(w) − f(|x − d|)
//! ```
//!
//! and the surface the carve *intends* at `x` is `f(x)`. So the link gouges only if
//! `f(w) − f(|x − d|) > f(x)` for some `x`. Take the worst case `d = w` (the tool right
//! on the ring); the condition for **no** gouge becomes
//!
//! ```text
//! f(x) + f(w − x) ≥ f(w)      for all 0 ≤ x ≤ w
//! ```
//!
//! which is exactly **superadditivity**, and `f` is convex with `f(0) = 0` — the ball
//! branch `rt − √(rt² − w²)` is convex, the flank branch is linear, and they join
//! smoothly — so it holds. Any `d > w` only moves the tool further from the wall, which
//! is more slack, not less.
//!
//! The condition to check per link is therefore just **`d ≥ w` all the way along it**:
//! the traverse must stay inside the inward-offset region the ring itself bounds. That
//! region is already in hand (it *is* the ring), so each link is verified against it and
//! the ones that fail — a link that would cut a corner across a concave notch, or hop
//! between two disjoint components — fall back to a lift.
//!
//! ## Depth is capped, not commanded
//!
//! `depth` is a **maximum**. The rings stop at the inward distance `w_max` whose groove
//! is `2·w_max` wide at that depth; anything further in is a **flat land** at full depth,
//! which the optional clearing tool handles (see [`crate::carve`] phase notes and
//! [`cam_model::CarveOp::clear_tool`]).

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, vtip_depth_for_half_width, vtip_half_width, vtip_max_depth, JoinStyle, Polygon};
use cam_model::{CarveOp, Clearing, Lead, Plunge, ToolKind};

use crate::{CancelToken, Diagnostic, JobEnv, Strategy, StrategyResult};

/// Default radial ring spacing when `ring_step` is `0`, in mm.
///
/// A fixed, deliberately fine value: the rings are a *finish* control, and 0.2 mm is
/// under what a carved surface shows at arm's length while keeping ring counts sane for
/// signage-sized work. Capped below by a quarter of the carve's own width so a tiny
/// carve still gets several rings rather than one.
const DEFAULT_RING_STEP_MM: f64 = 0.2;

/// Absolute floor on the ring spacing, mm — below this the ring count explodes for no
/// visible gain, and the offsetter's integer grid stops resolving the difference.
const MIN_RING_STEP_MM: f64 = 0.01;

/// Carves a region with a V-bit. Construct from a [`CarveOp`].
#[derive(Clone, Debug)]
pub struct CarveStrategy {
    op: CarveOp,
}

impl CarveStrategy {
    /// Build a carving strategy for `op`.
    pub fn new(op: CarveOp) -> Self {
        Self { op }
    }
}

impl Strategy for CarveStrategy {
    fn name(&self) -> &str {
        "carve"
    }

    fn compute(&self, env: &JobEnv, cancel: &CancelToken) -> StrategyResult {
        let op = &self.op;
        let mut diagnostics = Vec::new();

        macro_rules! bail {
            () => {
                return StrategyResult {
                    diagnostics,
                    ..Default::default()
                }
            };
        }
        macro_rules! fail {
            ($($arg:tt)*) => {{
                diagnostics.push(Diagnostic::error(format!($($arg)*)));
                bail!();
            }};
        }

        let Some(tool) = env.tool(op.tool) else {
            fail!(
                "operation {}: references tool {} which is not in the setup",
                op.id,
                op.tool
            );
        };

        // GATE 1 — the tool must be a V-bit. As with engraving, the chamfer mill is the
        // near miss worth naming: it is conical, but its tip is a flat that does not cut.
        let (included_angle_deg, tip_radius) = match tool.kind {
            ToolKind::VBit {
                included_angle_deg,
                tip_radius,
            } => (included_angle_deg, tip_radius),
            ToolKind::ChamferMill { .. } => fail!(
                "operation {}: tool {} is a chamfer mill, whose tip is a flat \
                 non-cutting face — it would rub, not cut. Carving needs a V-bit.",
                op.id,
                op.tool
            ),
            _ => fail!(
                "operation {}: tool {} is a {}; carving needs a V-bit (the carved \
                 V-section comes from the tool's point)",
                op.id,
                op.tool,
                tool.kind
            ),
        };

        if !op.boundary.is_valid() {
            fail!(
                "operation {}: a carve boundary needs at least 3 points — it outlines an \
                 area, so there is nothing to carve without an interior",
                op.id
            );
        }
        if op.depth <= 0.0 {
            fail!("operation {}: carve depth must be positive", op.id);
        }

        let alpha = 0.5 * included_angle_deg.to_radians();
        if !(alpha > 0.0 && alpha < std::f64::consts::FRAC_PI_2) {
            fail!(
                "operation {}: tool included angle {} is not a valid V (0-180 deg)",
                op.id,
                included_angle_deg
            );
        }

        // GATE 2 — as for engraving, past the depth where the cone reaches the full
        // cutting radius it is the shank, not an edge, against the wall.
        let max_depth = vtip_max_depth(alpha, tip_radius, tool.radius());
        if op.depth > max_depth + 1e-9 {
            fail!(
                "operation {}: depth {:.3} mm exceeds the {:.3} mm at which tool {}'s \
                 cone reaches its full cutting dia {:.3} - deeper, the shank rubs \
                 instead of cutting. Use a larger-dia or narrower-angle V-bit.",
                op.id,
                op.depth,
                max_depth,
                op.tool,
                tool.diameter
            );
        }

        let region = match Polygon::with_holes(op.boundary.clone(), op.islands.clone()) {
            Ok(p) => p,
            Err(e) => fail!("operation {}: invalid carve region: {e}", op.id),
        };

        // The widest the carve gets: the half-width of the groove at the depth cap.
        // Rings run from the boundary inward to exactly here, and no further — beyond it
        // the V would be deeper than allowed.
        let w_max = vtip_half_width(alpha, tip_radius, op.depth);
        let step = ring_step(op.ring_step, w_max);
        let widths = ring_widths(w_max, step);

        let link = Tag::new(op.id, MoveKind::Link);
        let plunge = Tag::new(op.id, MoveKind::Plunge);
        let cut = Tag::new(op.id, MoveKind::Cutting);
        let retract = Tag::new(op.id, MoveKind::Retract);

        // The rings go into their own program first: the header comment reports the ring
        // count and the depth actually reached, and neither is known until the offsets
        // have been walked (they may vanish well before `w_max`).
        let mut body = Program::new();
        let mut rings_cut = 0usize;
        let mut lifts = 0usize;
        let mut deepest = 0.0f64;
        // Where the tool is, when it is still down: its XY and its Z.
        let mut at: Option<(cam_geo::Point, f64)> = None;
        // The region the tool may traverse at that Z without gouging — the offset region
        // bounded by the ring it has just finished (see the module docs). Held one level
        // behind, so the first ring of a new width is judged against the width the tool is
        // coming *from*, and later rings of the same width against their own.
        let mut guard: Vec<Polygon> = Vec::new();

        for &w in &widths {
            if cancel.is_cancelled() {
                return StrategyResult {
                    program: body,
                    diagnostics,
                    cancelled: true,
                };
            }
            // Inward by the hold-off plus this ring's distance. Positive `offset` leaves
            // material, so it shrinks the carved region — the same sign convention as a
            // profile's finishing allowance.
            let rings = match offset(std::slice::from_ref(&region), -(op.offset + w), JoinStyle::Round) {
                Ok(r) => r,
                Err(e) => fail!("operation {}: offset failed: {e}", op.id),
            };
            if rings.is_empty() {
                // The offsets have vanished: this is where the medial axis lies, and the
                // region is fully carved. Everything further in does not exist.
                break;
            }

            let depth = vtip_depth_for_half_width(alpha, tip_radius, w).min(op.depth);
            let z = op.top - depth;
            deepest = deepest.max(depth);

            let mut first_of_this_width = true;
            for poly in &rings {
                // A hole in the offset result is a ring around an island (or a counter of
                // a letter): it is carved at the same depth, being the same distance from
                // a boundary.
                for contour in std::iter::once(poly.outer()).chain(poly.holes()) {
                    if !contour.is_valid() {
                        continue;
                    }
                    // When staying down, begin this ring at the point nearest where the
                    // tool already is, so the link is as short as it can be — the
                    // operator's `start` then applies only to the first ring, which is
                    // where the entry witness actually lands.
                    let nearest = at.map(|(q, _)| {
                        crate::profile::rotate_to_start(contour.points(), Some([q.x, q.y]))
                    });
                    let linked = op.stay_down
                        && match (&at, &nearest) {
                            (Some((q, _)), Some(cand)) => link_is_safe(&guard, *q, cand[0]),
                            _ => false,
                        };
                    let pts = match (linked, nearest) {
                        (true, Some(cand)) => cand,
                        _ => crate::profile::rotate_to_start(contour.points(), op.start),
                    };
                    let start = pts[0];

                    match (linked, at) {
                        (true, Some((_, zq))) => {
                            // Traverse at the *previous* ring's depth, where the region
                            // just verified guarantees no gouge, and only then sink to
                            // this ring's depth. Both moves cut real material — the shelf
                            // between the two rings — so they are fed, not rapid.
                            body.push(Step::Linear {
                                to: Point3::new(start.x, start.y, zq),
                                feed: op.feed,
                                tag: cut,
                            });
                            body.push(Step::Linear {
                                to: Point3::new(start.x, start.y, z),
                                feed: op.plunge_feed,
                                tag: plunge,
                            });
                        }
                        _ => {
                            // Lift and come back down: either the operator asked for it,
                            // or this particular link failed its safety check.
                            if let Some((q, _)) = at {
                                body.push(Step::Rapid {
                                    to: Point3::new(q.x, q.y, env.heights.clearance),
                                    tag: retract,
                                });
                            }
                            body.push(Step::Rapid {
                                to: Point3::new(start.x, start.y, env.heights.clearance),
                                tag: link,
                            });
                            body.push(Step::Rapid {
                                to: Point3::new(start.x, start.y, env.heights.retract.max(op.top)),
                                tag: link,
                            });
                            body.push(Step::Linear {
                                to: Point3::new(start.x, start.y, z),
                                feed: op.plunge_feed,
                                tag: plunge,
                            });
                            lifts += 1;
                        }
                    }

                    crate::emit::cut_loop(&mut body, &pts, op.feed, cut, z);
                    rings_cut += 1;
                    // `cut_loop` closes back to the start, so that is where the tool is.
                    at = Some((start, z));
                    if first_of_this_width {
                        // From here on, links are judged against *this* width's region.
                        guard.clone_from(&rings);
                        first_of_this_width = false;
                    }
                }
            }
        }

        // Whatever happened, come home to clearance.
        if let Some((q, _)) = at {
            body.push(Step::Rapid {
                to: Point3::new(q.x, q.y, env.heights.clearance),
                tag: retract,
            });
        }

        if rings_cut == 0 {
            fail!(
                "operation {}: nothing to carve - the region vanishes {:.3} mm in, before \
                 the first ring. The hold-off may exceed the region's own width.",
                op.id,
                op.offset + widths.first().copied().unwrap_or(0.0)
            );
        }

        // --- what the shape itself allows, and what the cap left behind ---
        //
        // One bisection answers both questions. The inward distance at which the region's
        // offsets vanish is the widest half-width the shape can hold, and so — through
        // the tool's own profile — its *natural full depth*: the depth at which the V
        // exactly reaches the widest inscribed point and no flat floor remains. If that
        // distance lies beyond `w_max`, the depth cap has stopped the V short and left a
        // flat land behind.
        let w_full = vanishing_width(&region, op.offset);
        let full_depth = vtip_depth_for_half_width(alpha, tip_radius, w_full);
        let reach = if full_depth > max_depth + 1e-9 {
            format!(", though tool {} reaches only {max_depth:.3} mm", op.tool)
        } else {
            String::new()
        };

        // The flat land itself: the region beyond where the capped V reaches. Its
        // boundary is the innermost carve ring, so the two meet by construction — the
        // ring at `w_max` sits at `depth`, which is exactly this floor's Z.
        let flat = if w_full > w_max + FLAT_LAND_TOL_MM {
            offset(
                std::slice::from_ref(&region),
                -(op.offset + w_max),
                JoinStyle::Round,
            )
            .unwrap_or_default()
        } else {
            Vec::new()
        };

        if !flat.is_empty() {
            diagnostics.push(Diagnostic::info(format!(
                "operation {}: full depth for this shape is {full_depth:.3} mm{reach}; \
                 the {:.3} mm cap leaves {}",
                op.id,
                op.depth,
                areas_phrase(flat.len())
            )));
            if op.clear_tool.is_none() {
                // Not an error: it does cut. But a cone cannot leave a flat floor —
                // adjacent passes leave uncut material between them — so say so.
                diagnostics.push(Diagnostic::warning(format!(
                    "operation {}: {} left at full depth with no clearing tool. The \
                     V-bit will cut them, but a cone cannot leave a flat floor, so they \
                     will be ridged. Set a clearing end mill, or deepen to \
                     {full_depth:.3} mm{reach}.",
                    op.id,
                    areas_phrase(flat.len()),
                )));
            }
        } else {
            diagnostics.push(Diagnostic::info(format!(
                "operation {}: the shape carves out at {full_depth:.3} mm{reach}, within \
                 the {:.3} mm cap — no flat areas remain",
                op.id, op.depth
            )));
        }

        // --- the clearing pass: strong tool first, on the flat land only ---
        //
        // It runs *before* the carve for two reasons. Bulk removal belongs to the end
        // mill, and a fine carving tip taking full-depth material is how one is snapped.
        // The V-bit then tapers down to meet the floor this leaves, exactly at the flat
        // land's boundary.
        let mut clear_body = Program::new();
        let mut clear_comment = None;
        if let Some(ct) = op.clear_tool {
            let Some(ctool) = env.tool(ct) else {
                fail!(
                    "operation {}: clearing tool {ct} is not in the setup",
                    op.id
                );
            };
            if ct == op.tool {
                fail!(
                    "operation {}: the clearing tool and the carving tool are both {ct}. \
                     A V-bit cannot leave the flat floor the clearing pass exists for.",
                    op.id
                );
            }
            if flat.is_empty() {
                diagnostics.push(Diagnostic::warning(format!(
                    "operation {}: tool {ct} is set to clear flat areas, but at {:.3} mm \
                     this shape leaves none - the tool change buys nothing. Clear the \
                     clearing tool, or cap the depth shallower.",
                    op.id, op.depth
                )));
            } else {
                if !crate::guards::check_flat_floor(op.id, "carve clearing", ctool, &mut diagnostics)
                    || !crate::guards::check_plunge(op.id, "carve clearing", ctool, &mut diagnostics)
                    || !crate::guards::check_axial_reach(
                        op.id,
                        "carve clearing",
                        ctool,
                        op.depth,
                        &mut diagnostics,
                    )
                {
                    bail!();
                }

                let r = ctool.radius();
                let spacing = if op.clear_stepover > 0.0 {
                    op.clear_stepover
                } else {
                    0.5 * ctool.diameter
                };
                let stepdown = if op.clear_stepdown > 0.0 {
                    op.clear_stepdown
                } else {
                    op.depth
                };
                let feed = if op.clear_feed > 0.0 { op.clear_feed } else { op.feed };
                let plunge_feed = if op.clear_plunge_feed > 0.0 {
                    op.clear_plunge_feed
                } else {
                    op.plunge_feed
                };
                let levels = crate::profile::depth_levels(op.top, op.top - op.depth, stepdown);

                let mut cleared = 0usize;
                let mut too_small = 0usize;
                for area in &flat {
                    // `first = r` puts the outermost ring's *edge* on the flat land's
                    // boundary, so the floor is cleared right out to where the carved
                    // wall meets it. No finishing allowance: the V-bit is the finish.
                    let job = crate::clearing::ClearJob {
                        id: op.id,
                        radius: r,
                        finish: 0.0,
                        first: r,
                        spacing,
                        clearing: Clearing::default(),
                        plunge: Plunge::Straight,
                        feed,
                        plunge_feed,
                        lead_overlap: 0.0,
                        lead_in: Lead::None,
                        lead_out: Lead::None,
                        start: op.start,
                        guard: &[],
                    };
                    match crate::clearing::clear(
                        &mut clear_body,
                        area,
                        &job,
                        &env.heights,
                        &levels,
                        cancel,
                    ) {
                        Ok(0) => too_small += 1,
                        Ok(_) => cleared += 1,
                        Err(crate::rings::RingsError::Cancelled) => {
                            return StrategyResult {
                                diagnostics,
                                cancelled: true,
                                ..Default::default()
                            };
                        }
                        Err(crate::rings::RingsError::Offset(e)) => {
                            fail!("operation {}: clearing offset failed: {e}", op.id)
                        }
                    }
                }

                if cleared == 0 {
                    fail!(
                        "operation {}: tool {ct} (dia {:.3}) does not fit any of the {} \
                         it was set to clear. Use a smaller clearing tool, or none.",
                        op.id,
                        ctool.diameter,
                        areas_phrase(flat.len())
                    );
                }
                if too_small > 0 {
                    diagnostics.push(Diagnostic::warning(format!(
                        "operation {}: tool {ct} (dia {:.3}) does not fit {} of the {}; \
                         those keep the ridged floor the V-bit leaves.",
                        op.id,
                        ctool.diameter,
                        too_small,
                        areas_phrase(flat.len())
                    )));
                }
                clear_comment = Some(format!(
                    "Carve clearing: {} at {:.3} mm deep with tool {ct} dia {:.3}, \
                     {:.3} mm stepover",
                    areas_phrase(cleared),
                    op.depth,
                    ctool.diameter,
                    spacing
                ));
            }
        }

        let mut program = Program::new();
        if let Some(c) = clear_comment {
            program.push(Step::Comment(c));
        }
        program.extend(clear_body);
        if op.clear_tool.is_some() {
            // The planner put the clearing tool in the spindle for us (it is `tools()[0]`),
            // so the change back to the V-bit is ours to emit — and it is emitted even
            // when there was nothing to clear, or the fragment would leave the wrong tool
            // loaded for whatever the operation does next.
            program.push(Step::ToolChange { tool: op.tool });
        }
        program.push(Step::Comment(format!(
            "Carve: {rings_cut} rings at {step:.3} mm, to {deepest:.3} mm deep with a \
             {included_angle_deg} deg V-bit, {lifts} lifts"
        )));
        program.extend(body);

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

/// How far past `w_max` the offsets must survive before a flat land is called real, mm.
///
/// Where the V exactly reaches the shape's widest inscribed point, the offsets vanish
/// *at* `w_max` and what remains is the medial axis — a curve of zero area, not a floor.
/// This keeps that case, and the bisection's own resolution, from being reported as a
/// flat land the operator should do something about.
const FLAT_LAND_TOL_MM: f64 = 2.0 * VANISH_TOL_MM;

/// Resolution of the vanishing-width bisection, mm. The reported full depth is this
/// precise divided by `tan α`, so ~0.002 mm on a 90° bit — far under what is machined.
const VANISH_TOL_MM: f64 = 1e-3;

/// The inward distance at which `region`'s offsets vanish, measured from `base`.
///
/// This is the widest half-width the shape can hold — the radius of its largest
/// inscribed circle — and so, read through the tool's profile, the depth at which a
/// V-carve exactly consumes it. It is found by bisection on emptiness rather than by
/// constructing a medial axis: emptiness is **monotone** in the offset distance, which
/// is all a bisection needs.
fn vanishing_width(region: &Polygon, base: f64) -> f64 {
    let empty_at = |w: f64| {
        offset(std::slice::from_ref(region), -(base + w), JoinStyle::Round)
            .map(|r| r.is_empty())
            .unwrap_or(true)
    };
    // An upper bound that must be empty: no inscribed circle can be wider than the
    // region's own bounding box.
    let pts = region.outer().points();
    let (mut lo_x, mut hi_x, mut lo_y, mut hi_y) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in pts {
        lo_x = lo_x.min(p.x);
        hi_x = hi_x.max(p.x);
        lo_y = lo_y.min(p.y);
        hi_y = hi_y.max(p.y);
    }
    let mut hi = ((hi_x - lo_x).max(hi_y - lo_y)).max(VANISH_TOL_MM);
    if !empty_at(hi) {
        // Should not happen for a sane region; do not loop forever if it does.
        return hi;
    }
    let mut lo = 0.0;
    while hi - lo > VANISH_TOL_MM {
        let mid = 0.5 * (lo + hi);
        if empty_at(mid) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    hi
}

/// `"1 flat area"` / `"3 flat areas"` — written out because the post strips parentheses
/// from comments, and the same phrasing reads correctly in a diagnostic.
fn areas_phrase(n: usize) -> String {
    if n == 1 {
        "1 flat area".to_string()
    } else {
        format!("{n} flat areas")
    }
}

/// Spacing at which a candidate link is sampled for containment, mm.
///
/// The link must stay inside `guard` (see the module docs); an excursion outside it is
/// caught if it is longer than this. A missed excursion shorter than 0.05 mm cuts a
/// notch narrower than the offsetter's own round-join facets, which is why sampling —
/// rather than exact segment/edge predicates and their degenerate touching cases — is
/// the right instrument here.
const LINK_SAMPLE_MM: f64 = 0.05;

/// Cap on the number of samples per link, so a pathologically long link cannot make the
/// check quadratic. Links are normally about one ring step long, so this is slack.
const LINK_SAMPLE_CAP: usize = 256;

/// Whether the tool may travel straight from `a` to `b` at the depth `region` was
/// computed for, without cutting below the surface the carve intends.
///
/// The whole condition is that the traverse stays at or beyond the ring's own inward
/// distance from the boundary — that is, inside `region` — which the module docs derive
/// from the convexity of the tool's profile. Both endpoints sit *on* `region`'s boundary
/// by construction, so it is the interior of the segment that is interrogated.
///
/// A link that crosses between two disjoint components of `region` fails, as it must:
/// the gap between them is exactly the material the tool must not plough through.
fn link_is_safe(region: &[Polygon], a: cam_geo::Point, b: cam_geo::Point) -> bool {
    if region.is_empty() {
        return false;
    }
    let len = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
    if len <= LINK_SAMPLE_MM {
        // Shorter than one sample: the endpoints are both on the region, and nothing can
        // deviate meaningfully in between.
        return true;
    }
    let n = ((len / LINK_SAMPLE_MM).ceil() as usize).min(LINK_SAMPLE_CAP);
    // One component must contain the whole link — hopping between components would mean
    // crossing the material that separates them.
    region.iter().any(|poly| {
        (1..n).all(|i| {
            let t = i as f64 / n as f64;
            poly.contains(cam_geo::Point::new(
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
            ))
        })
    })
}

/// The radial ring spacing actually used: the operator's `ring_step` when set, else a
/// default fine enough for the carve's own size.
fn ring_step(requested: f64, w_max: f64) -> f64 {
    let s = if requested > 0.0 {
        requested
    } else {
        DEFAULT_RING_STEP_MM.min(w_max / 4.0)
    };
    s.max(MIN_RING_STEP_MM)
}

/// Inward distances of the carve rings: `step, 2·step, …`, always ending **exactly** on
/// `w_max`.
///
/// Landing exactly on `w_max` is what makes the carved wall meet the flat land at full
/// depth: the ring at `w_max` sits at `depth`, which is precisely the flat floor's Z. A
/// ring short of it would leave a step at the junction.
fn ring_widths(w_max: f64, step: f64) -> Vec<f64> {
    if w_max <= 0.0 || step <= 0.0 {
        return Vec::new();
    }
    let mut ws = Vec::new();
    let mut w = step;
    while w < w_max {
        ws.push(w);
        w += step;
    }
    if ws.last().is_some_and(|&l| (w_max - l).abs() < 1e-9) {
        ws.pop();
    }
    ws.push(w_max);
    ws
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Severity;
    use cam_geo::{Contour, Point};
    use cam_model::{Heights, Tool};

    /// Positional tolerance for anything read back out of the offsetter, mm.
    ///
    /// Two effects set the floor: the offsetter's 0.1 µm integer grid, and the
    /// round-join arcs, whose chord deviation is `ROUND_RATIO²·r/8` — about 1.6e-4 mm
    /// at the sub-millimetre radii a carve ring uses. An order of magnitude over that
    /// is tight enough to catch a real placement error and loose enough not to pin
    /// the flattening resolution.
    const OFFSET_TOL: f64 = 2e-3;

    /// Compare cutting depths, allowing for accumulated floating-point step sums.
    fn assert_zs(got: &[f64], want: &[f64]) {
        assert_eq!(got.len(), want.len(), "got {got:?} want {want:?}");
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-9, "got {got:?} want {want:?}");
        }
    }

    fn vbit(included_angle_deg: f64, tip_radius: f64) -> Tool {
        Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 1,
            kind: ToolKind::VBit {
                included_angle_deg,
                tip_radius,
            },
            ..Default::default()
        }
    }

    fn tool_of(kind: ToolKind) -> Tool {
        Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind,
            ..Default::default()
        }
    }

    fn square(side: f64) -> Contour {
        Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(side, 0.0),
            Point::new(side, side),
            Point::new(0.0, side),
        ])
    }

    fn op(depth: f64) -> CarveOp {
        CarveOp {
            id: 0,
            tool: 1,
            clear_tool: None,
            boundary: square(20.0),
            islands: Vec::new(),
            top: 0.0,
            depth,
            offset: 0.0,
            ring_step: 0.5,
            feed: 200.0,
            plunge_feed: 100.0,
            stay_down: false,
            clear_stepover: 0.0,
            clear_stepdown: 0.0,
            clear_feed: 0.0,
            clear_plunge_feed: 0.0,
            start: None,
        }
    }

    fn run(op: CarveOp, tool: Tool) -> StrategyResult {
        run_with(op, &[tool])
    }

    fn run_with(op: CarveOp, tools: &[Tool]) -> StrategyResult {
        let env = JobEnv {
            heights: Heights::new(5.0, 2.0, 0.0),
            tools,
            stock: None,
        };
        CarveStrategy::new(op).compute(&env, &CancelToken::new())
    }

    /// An end mill, numbered, for the clearing pass.
    fn endmill(number: u32, diameter: f64) -> Tool {
        Tool {
            number,
            diameter,
            length: 30.0,
            flute_length: 20.0,
            flutes: 2,
            kind: ToolKind::EndMill,
            ..Default::default()
        }
    }

    fn errors(r: &StrategyResult) -> Vec<String> {
        r.diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Error)
            .map(|d| d.message.clone())
            .collect()
    }

    /// Every cutting point, with its Z.
    fn cuts(r: &StrategyResult) -> Vec<(f64, f64, f64)> {
        r.program
            .steps()
            .iter()
            .filter_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                    Some((to.x, to.y, to.z))
                }
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => {
                    Some((end.x, end.y, end.z))
                }
                _ => None,
            })
            .collect()
    }

    /// The distinct cutting depths, in the order first reached.
    fn cut_zs(r: &StrategyResult) -> Vec<f64> {
        let mut zs: Vec<f64> = Vec::new();
        for (_, _, z) in cuts(r) {
            if zs.last().is_none_or(|&l: &f64| (l - z).abs() > 1e-9) {
                zs.push(z);
            }
        }
        zs
    }

    // --- the gates ---

    #[test]
    fn a_chamfer_mill_is_rejected_because_its_tip_does_not_cut() {
        let r = run(
            op(1.0),
            tool_of(ToolKind::ChamferMill {
                included_angle_deg: 90.0,
                tip_diameter: 0.2,
            }),
        );
        let e = errors(&r);
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("chamfer mill") && e[0].contains("non-cutting"), "{e:?}");
        assert!(r.program.steps().is_empty(), "no path may be emitted");
    }

    #[test]
    fn an_end_mill_is_rejected_as_the_carving_tool() {
        let r = run(op(1.0), tool_of(ToolKind::EndMill));
        let e = errors(&r);
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("V-bit"), "{e:?}");
    }

    #[test]
    fn depth_past_the_cone_flare_is_rejected() {
        // 90° V-bit, ⌀6 (r=3), sharp tip → full ⌀ at depth 3.0.
        let t = vbit(90.0, 0.0);
        assert!(errors(&run(op(2.9), t)).is_empty());
        let e = errors(&run(op(3.1), t));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("shank rubs"), "{e:?}");
    }

    #[test]
    fn an_open_or_degenerate_boundary_is_rejected() {
        let mut o = op(1.0);
        o.boundary = Contour::new(vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0)]);
        let e = errors(&run(o, vbit(90.0, 0.0)));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("outlines an area"), "{e:?}");
    }

    #[test]
    fn a_hold_off_wider_than_the_region_carves_nothing() {
        let mut o = op(1.0);
        o.offset = 15.0; // the 20 mm square vanishes 10 mm in
        let e = errors(&run(o, vbit(90.0, 0.0)));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("nothing to carve"), "{e:?}");
    }

    // --- the carve ---

    #[test]
    fn rings_step_inward_and_deepen_together() {
        // The defining property: a ring further from the boundary is cut deeper, by
        // exactly the V-bit's own width-to-depth relation. A 90° sharp bit gives
        // depth == inward distance.
        let mut o = op(1.0);
        o.ring_step = 0.25;
        let r = run(o, vbit(90.0, 0.0));
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        assert_zs(&cut_zs(&r), &[-0.25, -0.5, -0.75, -1.0]);
    }

    #[test]
    fn each_ring_sits_exactly_its_own_depth_in_from_the_boundary() {
        // Not "a ring arrived" but "the ring is in the right place": for a 90° sharp
        // bit, the ring cut at depth d must be inset d from the 20 mm square's walls.
        let mut o = op(1.5);
        o.ring_step = 0.5;
        let r = run(o, vbit(90.0, 0.0));
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        for (x, y, z) in cuts(&r) {
            let d = -z; // depth below top 0.0
            // distance from the nearest wall of the [0,20]² square
            let inset = x.min(y).min(20.0 - x).min(20.0 - y);
            assert!(
                (inset - d).abs() < OFFSET_TOL,
                "point ({x:.4},{y:.4}) at depth {d:.4} is {inset:.4} from the wall"
            );
        }
    }

    #[test]
    fn a_tipped_bit_uses_the_ball_relation_not_the_naive_cone() {
        // Shallow rings on a tipped bit sit in the *ball*, where depth grows far more
        // slowly than w/tanα. Getting this wrong would carve a visibly wrong section.
        let mut o = op(1.0);
        o.ring_step = 0.1;
        let rt = 0.3;
        let r = run(o, vbit(60.0, rt));
        let a = (30.0_f64).to_radians();
        let first = cut_zs(&r)[0];
        let naive = -0.1 / a.tan();
        let ball = -(rt - (rt * rt - 0.1 * 0.1_f64).sqrt());
        assert!((first - ball).abs() < 1e-9, "first={first} ball={ball}");
        assert!(first > naive, "the ball must be shallower than the naive cone");
    }

    #[test]
    fn the_innermost_ring_lands_exactly_on_the_depth_cap() {
        // The junction with the flat land depends on this: a ring short of w_max would
        // leave a step where the carved wall meets the floor.
        let mut o = op(1.0);
        o.ring_step = 0.3; // does not divide 1.0
        let r = run(o, vbit(90.0, 0.0));
        let zs = cut_zs(&r);
        assert!((zs.last().unwrap() + 1.0).abs() < 1e-9, "{zs:?}");
        assert_zs(&zs, &[-0.3, -0.6, -0.9, -1.0]);
    }

    #[test]
    fn the_carve_stops_where_the_offsets_vanish() {
        // A 20 mm square's offsets die at 10 mm in, well before this 12 mm-wide V —
        // that vanishing point *is* the medial axis, and the carve must simply end.
        let mut o = op(2.9); // w_max = 2.9 for a 90° bit… still inside
        o.boundary = square(3.0); // …but this square dies 1.5 mm in
        o.ring_step = 0.5;
        let r = run(o, vbit(90.0, 0.0));
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        let deepest = cut_zs(&r).into_iter().fold(0.0_f64, f64::min);
        assert!(deepest > -1.6 && deepest < -1.0, "deepest={deepest}");
    }

    #[test]
    fn an_island_is_carved_around_at_the_same_depth_as_the_boundary() {
        // A letter's counter must carve like any other wall: the rings around the
        // island are at the same depth as the rings the same distance from the outside.
        let mut o = op(1.0);
        o.ring_step = 0.5;
        o.islands = vec![Contour::new(vec![
            Point::new(8.0, 8.0),
            Point::new(12.0, 8.0),
            Point::new(12.0, 12.0),
            Point::new(8.0, 12.0),
        ])];
        let r = run(o, vbit(90.0, 0.0));
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        // Points near the island must obey the same inset == depth rule, measuring to
        // the island's wall.
        let near_island = cuts(&r)
            .into_iter()
            .filter(|&(x, y, _)| (7.0..13.0).contains(&x) && (7.0..13.0).contains(&y))
            .count();
        assert!(near_island > 0, "no rings were cut around the island");
        for (x, y, z) in cuts(&r) {
            let outer = x.min(y).min(20.0 - x).min(20.0 - y);
            // Distance outside the [8,12]² island (0 inside, which cannot happen).
            let dx = (8.0 - x).max(x - 12.0).max(0.0);
            let dy = (8.0 - y).max(y - 12.0).max(0.0);
            let island = (dx * dx + dy * dy).sqrt();
            let inset = outer.min(island);
            assert!(
                (inset + z).abs() < OFFSET_TOL,
                "({x:.4},{y:.4},{z:.4}) is {inset:.4} from a wall"
            );
        }
    }

    #[test]
    fn the_hold_off_shifts_the_whole_carve_inward() {
        let mut o = op(1.0);
        o.ring_step = 0.5;
        o.offset = 1.0;
        let r = run(o, vbit(90.0, 0.0));
        for (x, y, z) in cuts(&r) {
            let inset = x.min(y).min(20.0 - x).min(20.0 - y);
            assert!((inset - 1.0 + z).abs() < OFFSET_TOL, "({x:.4},{y:.4},{z:.4})");
        }
    }

    #[test]
    fn depth_is_measured_down_from_the_top_plane() {
        let mut o = op(1.0);
        o.top = -2.0;
        o.ring_step = 0.5;
        let r = run(o, vbit(90.0, 0.0));
        assert_zs(&cut_zs(&r), &[-2.5, -3.0]);
    }

    fn plunges(r: &StrategyResult) -> usize {
        r.program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Linear { tag, .. } if tag.kind == MoveKind::Plunge))
            .count()
    }

    fn retracts(r: &StrategyResult) -> usize {
        r.program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract))
            .count()
    }

    #[test]
    fn lifting_mode_gives_every_ring_its_own_plunge_and_retract() {
        let mut o = op(1.0);
        o.ring_step = 0.5;
        let r = run(o, vbit(90.0, 0.0));
        assert_eq!(plunges(&r), 2);
        assert_eq!(retracts(&r), 2);
        // And no rapid ever ends on the stock surface itself.
        for s in r.program.steps() {
            if let Step::Rapid { to, tag } = s {
                if tag.kind == MoveKind::Link {
                    assert!(to.z >= 2.0 - 1e-9, "rapid to z={} is below retract", to.z);
                }
            }
        }
    }

    // --- staying down between rings ---

    #[test]
    fn staying_down_links_the_rings_and_lifts_only_at_the_end() {
        // A convex region's rings all nest, so every link is safe: one entry plunge,
        // one final retract, however many rings there are.
        let mut o = op(1.0);
        o.ring_step = 0.25;
        o.stay_down = true;
        let r = run(o, vbit(90.0, 0.0));
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        assert_eq!(cut_zs(&r).len(), 4, "still four rings");
        assert_eq!(retracts(&r), 1, "one retract, at the very end");
        // One entry plunge from clearance + one short sink per subsequent ring.
        assert_eq!(plunges(&r), 4);
        assert_eq!(
            r.program
                .steps()
                .iter()
                .filter(|s| matches!(s, Step::Rapid { tag, .. } if tag.kind == MoveKind::Link))
                .count(),
            2,
            "only the single approach from clearance"
        );
    }

    #[test]
    fn staying_down_cuts_the_same_rings_in_the_same_places() {
        // The toggle is a linking choice, not a geometry one: the carved rings
        // themselves must be identical either way.
        let mut lift = op(1.0);
        lift.ring_step = 0.25;
        let mut down = lift.clone();
        down.stay_down = true;
        let a = run(lift, vbit(90.0, 0.0));
        let b = run(down, vbit(90.0, 0.0));
        // Compare only the closed-loop cutting, ignoring the inward link moves that
        // staying down adds.
        let loops = |r: &StrategyResult| -> Vec<(f64, f64, f64)> {
            let mut v = cuts(r);
            v.dedup();
            v
        };
        let (la, lb) = (loops(&a), loops(&b));
        for p in &la {
            assert!(
                lb.iter().any(|q| (q.0 - p.0).abs() < 1e-9
                    && (q.1 - p.1).abs() < 1e-9
                    && (q.2 - p.2).abs() < 1e-9),
                "{p:?} is cut when lifting but not when staying down"
            );
        }
    }

    #[test]
    fn a_stay_down_link_never_dips_below_the_intended_surface() {
        // The property the whole scheme rests on. Every point of every link move is
        // checked against the surface the carve intends there: depth == distance from
        // the wall for a 90° sharp bit.
        let mut o = op(1.0);
        o.ring_step = 0.25;
        o.stay_down = true;
        let r = run(o, vbit(90.0, 0.0));
        for (x, y, z) in cuts(&r) {
            let inset = x.min(y).min(20.0 - x).min(20.0 - y);
            // The tool's tip may be at most as deep as the V allows at this distance,
            // and never deeper than the cap.
            let allowed = inset.min(1.0);
            assert!(
                -z <= allowed + OFFSET_TOL,
                "({x:.4},{y:.4}) at depth {:.4} exceeds the {allowed:.4} the shape allows",
                -z
            );
        }
    }

    #[test]
    fn a_link_across_a_disjoint_component_is_refused() {
        // Two lobes joined by a neck 0.8 mm thick — thinner than the rings get. Once the
        // offsets pinch that neck off, the lobes are separate islands of safe travel and
        // the tool must lift between them rather than plough through the neck.
        let dumbbell = Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(8.0, 0.0),
            Point::new(8.0, 7.2),
            Point::new(12.0, 7.2),
            Point::new(12.0, 0.0),
            Point::new(20.0, 0.0),
            Point::new(20.0, 8.0),
            Point::new(0.0, 8.0),
        ]);
        let mut o = op(1.0);
        o.boundary = dumbbell;
        o.ring_step = 0.25;
        o.stay_down = true;
        let r = run(o, vbit(90.0, 0.0));
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        assert!(
            retracts(&r) > 1,
            "the tool must lift to reach a pinched-off lobe, got {} retracts",
            retracts(&r)
        );
    }

    #[test]
    fn link_safety_rejects_a_shortcut_outside_the_region() {
        // A direct check on the predicate: an L-shaped region, and a chord between two
        // points on its boundary that cuts across the missing quadrant.
        let l = Polygon::new(Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 4.0),
            Point::new(4.0, 4.0),
            Point::new(4.0, 10.0),
            Point::new(0.0, 10.0),
        ]))
        .unwrap();
        let region = [l];
        // Along the inside of the L: safe.
        assert!(link_is_safe(&region, Point::new(0.0, 0.0), Point::new(10.0, 0.0)));
        // Across the notch: not.
        assert!(!link_is_safe(&region, Point::new(10.0, 4.0), Point::new(4.0, 10.0)));
        // An empty region can never be traversed.
        assert!(!link_is_safe(&[], Point::new(0.0, 0.0), Point::new(1.0, 1.0)));
    }

    // --- flat lands and the shape's own full depth ---

    fn warnings(r: &StrategyResult) -> Vec<String> {
        r.diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Warning)
            .map(|d| d.message.clone())
            .collect()
    }

    fn infos(r: &StrategyResult) -> Vec<String> {
        r.diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Info)
            .map(|d| d.message.clone())
            .collect()
    }

    #[test]
    fn the_vanishing_width_is_the_largest_inscribed_circle() {
        // Not "it returns something": the number has a closed form. A 20 mm square's
        // largest inscribed circle has radius 10; a 20x6 rectangle's has radius 3.
        let sq = Polygon::new(square(20.0)).unwrap();
        assert!((vanishing_width(&sq, 0.0) - 10.0).abs() < 2e-3);
        // A hold-off shrinks the shape, and so its inscribed circle, one for one.
        assert!((vanishing_width(&sq, 2.0) - 8.0).abs() < 2e-3);
        let rect = Polygon::new(Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(20.0, 0.0),
            Point::new(20.0, 6.0),
            Point::new(0.0, 6.0),
        ]))
        .unwrap();
        assert!((vanishing_width(&rect, 0.0) - 3.0).abs() < 2e-3);
        // An island eats into it — and the answer is *not* half the frame width, which
        // is the trap. A 20 mm square with a 10 mm island centred in it leaves a 5 mm
        // frame, but the biggest circle sits in a corner, touching two outer walls and
        // the island's corner: centred at (c, c) with radius c, needing
        // sqrt(2)*(5 - c) >= c, so c = 5*sqrt(2)/(1 + sqrt(2)) ~ 2.929 — appreciably
        // more than the 2.5 the straight frame would suggest.
        let framed = Polygon::with_holes(
            square(20.0),
            vec![Contour::new(vec![
                Point::new(5.0, 5.0),
                Point::new(15.0, 5.0),
                Point::new(15.0, 15.0),
                Point::new(5.0, 15.0),
            ])],
        )
        .unwrap();
        let corner = 5.0 * std::f64::consts::SQRT_2 / (1.0 + std::f64::consts::SQRT_2);
        assert!((vanishing_width(&framed, 0.0) - corner).abs() < 2e-3);
    }

    #[test]
    fn a_cap_short_of_the_shape_reports_the_flat_land_it_leaves() {
        // A 20 mm square wants 10 mm of depth to carve out; a 1 mm cap cannot, so a
        // flat land is left and the operator is told the number that would clear it.
        let r = run(op(1.0), vbit(90.0, 0.0));
        let i = infos(&r);
        assert_eq!(i.len(), 1, "{i:?}");
        assert!(i[0].contains("full depth for this shape is 10.0"), "{i:?}");
        assert!(i[0].contains("1 flat area"), "{i:?}");
        // …and that the bit itself could not reach that depth even if asked.
        assert!(i[0].contains("reaches only 3.000 mm"), "{i:?}");
    }

    #[test]
    fn a_flat_land_with_no_clearing_tool_warns_about_a_ridged_floor() {
        // A warning, not an error: it does cut, just worse. Refusing would be wrong.
        let r = run(op(1.0), vbit(90.0, 0.0));
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        let w = warnings(&r);
        assert_eq!(w.len(), 1, "{w:?}");
        assert!(w[0].contains("ridged"), "{w:?}");
        assert!(!r.program.steps().is_empty(), "the carve must still be emitted");
    }

    #[test]
    fn setting_a_clearing_tool_silences_the_ridged_floor_warning() {
        let mut o = op(1.0);
        o.clear_tool = Some(2);
        let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        assert!(warnings(&r).is_empty(), "{:?}", warnings(&r));
    }

    // --- the clearing pass and the tool change ---

    /// The program's tool changes and the index of the first cutting move after each.
    fn tool_changes(r: &StrategyResult) -> Vec<u32> {
        r.program
            .steps()
            .iter()
            .filter_map(|s| match s {
                Step::ToolChange { tool } => Some(*tool),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn the_clearing_pass_runs_first_and_hands_back_to_the_v_bit() {
        // The ordering is the whole point: bulk removal with the strong tool, then a
        // change, then the carve. A fine carving tip taking full-depth material is how
        // one is snapped.
        let mut o = op(1.0);
        o.clear_tool = Some(2);
        let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        assert_eq!(tool_changes(&r), vec![1], "one change, back to the V-bit");

        // Everything before that change belongs to the clearing pass, everything after
        // to the carve — and both must actually contain cutting.
        let steps = r.program.steps();
        let change = steps
            .iter()
            .position(|s| matches!(s, Step::ToolChange { .. }))
            .expect("a tool change");
        let cutting = |r: &[Step]| {
            r.iter().any(
                |s| matches!(s, Step::Linear { tag, .. } | Step::Arc { tag, .. } if tag.kind == MoveKind::Cutting),
            )
        };
        assert!(cutting(&steps[..change]), "the clearing pass cut nothing");
        assert!(cutting(&steps[change + 1..]), "the carve cut nothing");
    }

    #[test]
    fn the_clearing_pass_reaches_full_depth_and_stays_inside_the_flat_land() {
        // The flat land of a 20 mm square capped at 1 mm is the square inset by
        // w_max = 1 mm. The end mill must clear it at exactly the cap depth and never
        // stray outside it, or it would cut through the carved V wall.
        let mut o = op(1.0);
        o.clear_tool = Some(2);
        let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
        let steps = r.program.steps();
        let change = steps
            .iter()
            .position(|s| matches!(s, Step::ToolChange { .. }))
            .unwrap();
        let mut saw = false;
        for s in &steps[..change] {
            let (x, y, z) = match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => (to.x, to.y, to.z),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => {
                    (end.x, end.y, end.z)
                }
                _ => continue,
            };
            saw = true;
            assert!((z + 1.0).abs() < 1e-9, "clearing at z={z}, not the 1 mm cap");
            // Inside the flat land (inset 1 mm) by at least the tool radius (2 mm).
            let inset = x.min(y).min(20.0 - x).min(20.0 - y);
            assert!(
                inset >= 1.0 + 2.0 - OFFSET_TOL,
                "({x:.4},{y:.4}) is {inset:.4} in - the cutter would breach the V wall"
            );
        }
        assert!(saw, "the clearing pass emitted no cutting");
    }

    #[test]
    fn a_clearing_tool_missing_from_the_setup_is_an_error() {
        let mut o = op(1.0);
        o.clear_tool = Some(9);
        let e = errors(&run(o, vbit(90.0, 0.0)));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("clearing tool 9 is not in the setup"), "{e:?}");
    }

    #[test]
    fn clearing_with_the_carving_tool_itself_is_an_error() {
        // It would be a no-op tool change and a floor the cone cannot flatten.
        let mut o = op(1.0);
        o.clear_tool = Some(1);
        let e = errors(&run(o, vbit(90.0, 0.0)));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("both"), "{e:?}");
    }

    #[test]
    fn a_clearing_tool_too_large_for_the_flat_land_is_an_error() {
        // The flat land here is an 18 mm square; a 30 mm cutter cannot enter it, so
        // there is nothing to clear with and the operator must be told, not left with
        // a silent tool change.
        let mut o = op(1.0);
        o.clear_tool = Some(2);
        let e = errors(&run_with(o, &[vbit(90.0, 0.0), endmill(2, 30.0)]));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("does not fit"), "{e:?}");
    }

    #[test]
    fn a_clearing_tool_that_cannot_reach_the_depth_is_an_error() {
        // 2 mm of flute cannot cut a 3 mm-deep floor: past that it is the shank in the
        // pocket, which is a hard error, not a preference.
        let mut o = op(2.5);
        o.clear_tool = Some(2);
        let mut short = endmill(2, 4.0);
        short.flute_length = 2.0;
        let e = errors(&run_with(o, &[vbit(90.0, 0.0), short]));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("cutting edge"), "{e:?}");
    }

    #[test]
    fn a_clearing_tool_with_nothing_to_clear_warns_but_still_hands_over() {
        // The shape carves out entirely, so the clearing tool has no flat land. The
        // change back to the V-bit must still be emitted: the planner loaded the
        // clearing tool for us, and leaving it in the spindle would carve with it.
        let mut o = op(2.9);
        o.boundary = square(3.0);
        o.ring_step = 0.5;
        o.clear_tool = Some(2);
        let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        let w = warnings(&r);
        assert_eq!(w.len(), 1, "{w:?}");
        assert!(w[0].contains("leaves none"), "{w:?}");
        assert_eq!(tool_changes(&r), vec![1], "the hand-back is not optional");
    }

    #[test]
    fn the_clearing_stepdown_splits_the_depth() {
        let mut o = op(2.0);
        o.clear_tool = Some(2);
        o.clear_stepdown = 0.8;
        let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        let steps = r.program.steps();
        let change = steps
            .iter()
            .position(|s| matches!(s, Step::ToolChange { .. }))
            .unwrap();
        let mut zs: Vec<f64> = steps[..change]
            .iter()
            .filter_map(|s| match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(to.z),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => Some(end.z),
                _ => None,
            })
            .collect();
        zs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        assert_zs(&zs, &[-0.8, -1.6, -2.0]);
    }

    #[test]
    fn a_shape_the_carve_consumes_leaves_no_flat_land_and_no_warning() {
        // A 3 mm square carves out at 1.5 mm, well inside a 2.9 mm cap.
        let mut o = op(2.9);
        o.boundary = square(3.0);
        o.ring_step = 0.5;
        let r = run(o, vbit(90.0, 0.0));
        assert!(warnings(&r).is_empty(), "{:?}", warnings(&r));
        let i = infos(&r);
        assert_eq!(i.len(), 1, "{i:?}");
        assert!(i[0].contains("carves out at 1.5"), "{i:?}");
        assert!(i[0].contains("no flat areas remain"), "{i:?}");
    }

    #[test]
    fn a_cap_exactly_at_the_shapes_full_depth_leaves_no_flat_land() {
        // The boundary case the tolerance exists for: the V reaching the widest
        // inscribed point exactly. What remains is the medial axis — a curve of zero
        // area — and calling that a flat land would send the operator hunting for a
        // clearing tool that has nothing to clear.
        let mut o = op(1.5); // 90° sharp bit: w_max = 1.5, and a 3 mm square vanishes at 1.5
        o.boundary = square(3.0);
        o.ring_step = 0.25;
        let r = run(o, vbit(90.0, 0.0));
        assert!(warnings(&r).is_empty(), "{:?}", warnings(&r));
        assert!(infos(&r)[0].contains("no flat areas remain"), "{:?}", infos(&r));
    }

    #[test]
    fn separate_flat_lands_are_counted_separately() {
        // Two wide lobes joined by a narrow bar: the bar carves out entirely, so the
        // flat land is two disjoint areas, and the operator should be told so.
        let mut o = op(0.5);
        o.ring_step = 0.25;
        o.boundary = Contour::new(vec![
            Point::new(0.0, 0.0),
            Point::new(6.0, 0.0),
            Point::new(6.0, 2.6),
            Point::new(14.0, 2.6),
            Point::new(14.0, 0.0),
            Point::new(20.0, 0.0),
            Point::new(20.0, 6.0),
            Point::new(14.0, 6.0),
            Point::new(14.0, 3.4),
            Point::new(6.0, 3.4),
            Point::new(6.0, 6.0),
            Point::new(0.0, 6.0),
        ]);
        let r = run(o, vbit(90.0, 0.0));
        let i = infos(&r);
        assert!(i[0].contains("2 flat areas"), "{i:?}");
    }

    // --- the ring schedule ---

    #[test]
    fn ring_widths_are_monotonic_and_end_on_w_max() {
        for &(w_max, step) in &[(1.0, 0.3), (2.0, 0.7), (0.5, 0.5), (1.2, 0.4), (0.1, 5.0)] {
            let ws = ring_widths(w_max, step);
            assert!((ws.last().unwrap() - w_max).abs() < 1e-12, "w={w_max} s={step}");
            assert!(ws.windows(2).all(|p| p[1] > p[0]), "w={w_max} s={step} {ws:?}");
            assert!(ws.iter().all(|&x| x > 0.0 && x <= w_max + 1e-12));
        }
    }

    #[test]
    fn a_step_dividing_evenly_leaves_no_sliver_ring() {
        assert_eq!(ring_widths(1.2, 0.4), vec![0.4, 0.8, 1.2]);
    }

    #[test]
    fn the_default_ring_step_scales_down_for_a_small_carve() {
        // A shallow carve on a fine bit is only fractions of a mm wide; the fixed
        // default would give it a single ring.
        assert!((ring_step(0.0, 10.0) - DEFAULT_RING_STEP_MM).abs() < 1e-12);
        assert!((ring_step(0.0, 0.4) - 0.1).abs() < 1e-12);
        assert!((ring_step(0.0, 1e-6) - MIN_RING_STEP_MM).abs() < 1e-12);
        assert!((ring_step(0.75, 10.0) - 0.75).abs() < 1e-12);
    }
}
