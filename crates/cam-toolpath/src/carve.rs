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
//! ## What the ring spacing actually controls
//!
//! Not the wall's finish, which is the trap. A ring at inward distance `w` puts the tool
//! tip at depth `f(w)`, and *by construction* the tool's half-width at the surface is
//! then exactly `w` — its flank lands on the boundary. So the **deepest ring alone cuts
//! the whole wall**, from the boundary down to its tip, and every shallower ring cuts a
//! narrower V entirely inside it. On a straight wall the intermediate rings change the
//! finished surface not at all; they are a **roughing** schedule, limiting how much one
//! pass takes. (They do earn their keep at a convex corner, where the deepest ring's
//! cone cannot reach the apex — the corner is the one place all of them contribute.)
//!
//! Where spacing *does* set the finish is the **floor**: there the surface is flat but
//! the tool is a cone, so adjacent passes leave a ridge of `f(spacing/2)` between them.
//! That is why the floor pass is spaced from a **scallop height** rather than from
//! `ring_step` — and why a rounded tip may take a far wider step than a sharp one for the
//! same ridge.
//!
//! ## Linking the rings: why staying down is safe
//!
//! A carve runs to hundreds of rings, and lifting to clearance for each costs far more
//! time than the cutting does. [`CarveOp::stay_down`] links them instead, and the licence
//! for that is one identity plus one monotonicity.
//!
//! Write `f(w)` for [`vtip_depth_for_half_width`], the tool's own profile height at radial
//! offset `w`. A ring at inward distance `w` sinks the tip to `f(w)`, so **at the stock
//! top the tool is exactly `w` wide on each side** — that is what `f` and
//! [`vtip_half_width`] being inverses *means*, and it is the defining property of the
//! whole schedule: the flank lands on the boundary. Hence a tool carried at that depth,
//! centred at inward distance `d`, occupies exactly
//!
//! ```text
//! [d − w,  d + w]
//! ```
//!
//! in distance-from-the-boundary. So it stays off the boundary **iff `d ≥ w`** — nothing
//! subtler than that. There is no material above the stock top for the wider part of the
//! cone to reach.
//!
//! It also cannot cut below the finished surface. That surface is the envelope of the
//! *deepest* ring, and for a fixed `x` the cut a ring at `w` makes,
//! `f(w) − f(w − x)`, is **increasing in `w`** — its derivative is
//! `f′(w) − f′(w − x) ≥ 0` because `f` is convex (the ball branch `rt − √(rt² − w²)` is
//! convex, the flank is linear, and they join smoothly). So no shallower pass ever
//! reaches deeper than the deepest one, and a link at a shallower ring's depth is
//! bounded by the surface that ring's own pass already cut.
//!
//! The condition to check per link is therefore just **`d ≥ w` all the way along it**:
//! the traverse must stay inside the inward-offset region the ring itself bounds. That
//! region is already in hand (it *is* the ring), so each link is verified against it and
//! the ones that fail — a link that would cut a corner across a concave notch, or hop
//! between two disjoint components — fall back to a lift.
//!
//! **A correction worth keeping**, because the wrong version was written here first: the
//! no-gouge condition is *not* `f(x) + f(w − x) ≥ f(w)`. That inequality is false — `f`
//! is convex with `f(0) = 0`, hence **super**additive, so it runs the other way. A single
//! pass genuinely does cut deeper than the nominal `depth(x) = f(x)` curve near the
//! boundary, and that is correct rather than a gouge: the nominal curve is the tool's
//! *profile*, only ever realised at the tip, while the wall a V-bit actually leaves is its
//! straight flank.
//!
//! ## Depth is capped, not commanded
//!
//! `depth` is a **maximum**. The rings stop at the inward distance `w_max` whose groove
//! is `2·w_max` wide at that depth; anything further in is a **flat land** at full depth,
//! which the optional clearing tool handles (see [`crate::carve`] phase notes and
//! [`cam_model::CarveOp::clear_tool`]).

use cam_cldata::{MoveKind, Point3, Program, Step, Tag};
use cam_geo::{offset, vtip_depth_for_half_width, vtip_half_width, vtip_max_depth, JoinStyle, Polygon};
use cam_model::{CarveOp, Plunge, Tool, ToolKind};

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

/// Default target ridge height on the flat floor, mm, when `scallop` is `0`.
///
/// Fine enough not to be felt with a thumbnail, and — because the spacing it implies is
/// read back through the tool's own profile — automatically generous on a rounded tip
/// and tight on a sharp one.
const DEFAULT_SCALLOP_MM: f64 = 0.05;

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
        // The floor's spacing comes from the ridge the operator will accept, not from the
        // wall's roughing step: a ridge of `h` sits midway between two passes, so the
        // spacing that leaves it is twice the tool's half-width at `h`.
        let scallop = if op.scallop > 0.0 {
            op.scallop
        } else {
            DEFAULT_SCALLOP_MM
        };
        let floor_step =
            (2.0 * vtip_half_width(alpha, tip_radius, scallop)).max(MIN_RING_STEP_MM);
        let widths = ring_widths(w_max, step);

        let link = Tag::new(op.id, MoveKind::Link);
        let plunge = Tag::new(op.id, MoveKind::Plunge);
        let cut = Tag::new(op.id, MoveKind::Cutting);
        let retract = Tag::new(op.id, MoveKind::Retract);

        // The rings go into their own program first: the header comment reports the ring
        // count and the depth actually reached, and neither is known until the offsets
        // have been walked (they may vanish well before `w_max`).
        let mut body = Program::new();
        let mut deepest = 0.0f64;
        let mut st = RingState::default();
        let tags = RingTags {
            link,
            plunge,
            cut,
            retract,
        };

        // --- pass 1: the V walls, from the boundary inward to the depth cap ---
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
            deepest = deepest.max(depth);
            emit_rings(
                &mut body,
                &mut st,
                &rings,
                &rings,
                op.top - depth,
                op,
                &env.heights,
                tags,
                0.0,
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
        let (w_full, flat) = shape_facts(&region, op.offset, w_max);
        let full_depth = vtip_depth_for_half_width(alpha, tip_radius, w_full);
        let reach = if full_depth > max_depth + 1e-9 {
            format!(", though tool {} reaches only {max_depth:.3} mm", op.tool)
        } else {
            String::new()
        };

        if !flat.is_empty() {
            diagnostics.push(Diagnostic::info(format!(
                "operation {}: full depth for this shape is {full_depth:.3} mm{reach}; \
                 the {:.3} mm cap leaves {}",
                op.id,
                op.depth,
                areas_phrase(flat.len())
            )));
            if op.clear.is_none() {
                // Not an error: it does cut. But a cone cannot leave a flat floor —
                // adjacent passes leave uncut material between them — so say so.
                diagnostics.push(Diagnostic::warning(format!(
                    "operation {}: {} at full depth with no clearing tool. The V-bit \
                     cuts them in {floor_step:.3} mm passes, but a cone cannot leave a \
                     flat floor, so they come out ridged by about {:.3} mm. Set a \
                     clearing end mill, or deepen to {full_depth:.3} mm{reach}.",
                    op.id,
                    areas_phrase(flat.len()),
                    vtip_depth_for_half_width(alpha, tip_radius, 0.5 * floor_step),
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
        // What the clearing tool actually swept, so the V-bit can be sent only where it
        // did not reach. Empty when there is no clearing pass.
        let mut swept: Vec<Polygon> = Vec::new();
        if let Some(clear) = op.clear {
            let ct = clear.tool;
            let cp = clear.params;
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
                // The clearing tool exists for exactly one reason: to leave a flat
                // floor where the cone cannot. A tool that cannot leave one has no
                // purpose here — it is not "worse", it is the wrong tool for the job,
                // and the V-bit alone would do as well. So this is an error, unlike the
                // general flat-floor guard, which warns for a merely scalloped floor.
                if !ctool.profile().cuts_flat_bottom() {
                    if ctool.profile().has_cutting_tip() {
                        fail!(
                            "operation {}: tool {ct} ({}) is not flat-bottomed, so it                              would leave a scalloped floor, not the flat one the                              clearing pass exists for. Use an end mill or bull nose, or                              no clearing tool at all.",
                            op.id,
                            ctool.kind
                        );
                    }
                    fail!(
                        "operation {}: tool {ct} ({}) has a non-cutting tip - it would                          leave an uncut ridge under its axis instead of a floor.",
                        op.id,
                        ctool.kind
                    );
                }
                // A straight drop into solid stock needs the axis to cut. Any other
                // entry eases in along the path, so the rule does not apply.
                if matches!(cp.plunge, Plunge::Straight)
                    && !crate::guards::check_plunge(op.id, "carve clearing", ctool, &mut diagnostics)
                {
                    bail!();
                }
                if !crate::guards::check_axial_reach(
                    op.id,
                    "carve clearing",
                    ctool,
                    op.depth,
                    &mut diagnostics,
                ) {
                    bail!();
                }

                let r = ctool.radius();
                // The same spacing rule a pocket uses, from the same field — and the
                // same validation: rejected rather than clamped, so a hand-edited file
                // says so instead of quietly cutting something else.
                if !(0.0..1.0).contains(&cp.overlap) {
                    fail!(
                        "operation {}: clearing overlap must be a fraction in [0, 1)",
                        op.id
                    );
                }
                let spacing = ctool.diameter * (1.0 - cp.overlap);
                if spacing <= 1e-9 {
                    fail!("operation {}: clearing overlap leaves no stepover", op.id);
                }
                let stepdown = if cp.stepdown > 0.0 { cp.stepdown } else { op.depth };
                let feed = if cp.feed > 0.0 { cp.feed } else { op.feed };
                let plunge_feed = if cp.plunge_feed > 0.0 {
                    cp.plunge_feed
                } else {
                    op.plunge_feed
                };
                let levels = crate::profile::depth_levels(op.top, op.top - op.depth, stepdown);
                // `offset` is a real finishing allowance here: how far the end mill stays
                // off the carved surface, leaving that skin for the V-bit — which cuts it
                // better, with the flank of its cone rather than a corner. Nothing is left
                // behind by it, because the V-bit's own passes are computed against what
                // the end mill actually swept.
                let finish = cp.offset.max(0.0);
                let first = r + finish;

                // **Each level clears to its own depth's V-width, not the bottom's.**
                // At depth d the carved surface stands `vtip_half_width(d)` in from the
                // boundary, so everything beyond that is waste *at that level*, and the
                // higher the level the more of it there is. Clearing every level to the
                // bottom's width would leave the whole taper for the V-bit.
                //
                // It cannot gouge: the tool's edge reaches only `w(d) + finish`, where the
                // intended depth is `f(w(d) + finish) >= f(w(d)) = d`. So a cut to depth
                // `d` there is always at or above the surface the carve wants.
                let mut cleared = 0usize;
                let mut too_small = 0usize;
                let mut bottom_start = 0usize;
                for (li, &z) in levels.iter().enumerate() {
                    let d = op.top - z;
                    let w = vtip_half_width(alpha, tip_radius, d);
                    let at_level = match offset(
                        std::slice::from_ref(&region),
                        -(op.offset + w + finish),
                        JoinStyle::Round,
                    ) {
                        Ok(a) => a,
                        Err(e) => fail!("operation {}: clearing offset failed: {e}", op.id),
                    };
                    // The floor's rest region is judged against what the *bottom* level
                    // swept; the shallower levels cover more ground and would overstate it.
                    if li + 1 == levels.len() {
                        bottom_start = clear_body.len();
                    }
                    for area in &at_level {
                        // Region the wall leads must stay inside, as a pocket computes it.
                        let guards =
                            offset(std::slice::from_ref(area), -first, JoinStyle::Round)
                                .unwrap_or_default();
                        let job = crate::clearing::ClearJob {
                            id: op.id,
                            radius: r,
                            finish,
                            first,
                            spacing,
                            clearing: cp.clearing,
                            plunge: cp.plunge,
                            feed,
                            plunge_feed,
                            lead_overlap: cp.lead_overlap,
                            lead_in: cp.lead_in,
                            lead_out: cp.lead_out,
                            start: op.start,
                            guard: &guards,
                        };
                        match crate::clearing::clear(
                            &mut clear_body,
                            area,
                            &job,
                            &env.heights,
                            &[z],
                            cancel,
                        ) {
                            Ok(0) => {
                                if li + 1 == levels.len() {
                                    too_small += 1;
                                }
                            }
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
                swept = swept_by(&clear_body.steps()[bottom_start..], r);
                clear_comment = Some(format!(
                    "Carve clearing: {} at {:.3} mm deep with tool {ct} dia {:.3}, \
                     {:.3} mm stepover, {}",
                    areas_phrase(cleared),
                    op.depth,
                    ctool.diameter,
                    spacing,
                    crate::passes_phrase(levels.len())
                ));
            }
        }

        // --- pass 2: the floor ---
        //
        // Pass 1 stops at `w_max`, which is only the *edge* of the flat land. Everything
        // inside it still has to be cut by something, and two cases arise:
        //
        // - **No clearing tool.** Nothing has touched the flat land, so the V-bit must
        //   cut all of it. (Without this it was left as solid stock at full height, while
        //   the diagnostic cheerfully described a merely *ridged* floor.)
        // - **A clearing tool.** The end mill took the bulk, but a round cutter cannot
        //   reach into a sharp corner: it leaves a lens of stock at every concave corner
        //   of the flat land, reaching `r·(√2−1)` in from a right-angled one. Pass 1 never
        //   goes there, so those would stand as raised nubs on the finished floor.
        //
        // Either way the V-bit runs concentric rings at **constant** full depth over
        // whatever is left, and it is safe for the same two reasons the stay-down links
        // are. Its tip sits at `f(w_max)`, so at the stock top it is `w_max` wide on each
        // side and stays off the boundary anywhere at or beyond `w_max` from it — which
        // is the whole flat land, by definition. And where its cone reaches back out over
        // the wall, at distance `x` it cuts to `f(w_max) − f(d − x) <= f(w_max) −
        // f(w_max − x)`, exactly the surface the deepest wall ring already left.
        let floor_region = if swept.is_empty() {
            flat.clone()
        } else {
            match cam_geo::difference(&flat, &swept) {
                Ok(rest) => rest,
                Err(e) => fail!("operation {}: rest-region failed: {e}", op.id),
            }
        };
        let floor_z = op.top - op.depth;
        // With no clearing tool the flat land's own boundary was already cut as pass 1's
        // innermost ring, so start one step in. A rest region has its own boundary, which
        // nothing has cut, so start on it.
        let first_d = if swept.is_empty() { floor_step } else { 0.0 };

        // Gather the floor loops before emitting any of them, component by component.
        // Two things fall out of doing it this way, and both were real defects:
        //
        // - **A rest island smaller than the ridge we already accept is dropped.** Around
        //   a curved island the clearing tool leaves a rash of slivers; cutting one costs
        //   a full lift/plunge/retract cycle to remove almost nothing. If a component's
        //   largest inscribed circle is no wider than half a floor step, the material
        //   standing in it is at most `f(inradius) <= f(floor_step/2)` — the very scallop
        //   the operator asked for. So it is already within tolerance, by construction.
        //
        // - **The loops are then ordered for locality.** The floor is all one depth and
        //   travel across the flat land is safe anywhere (above), so the order is free —
        //   and the natural order is the wrong one: it walks every component at one
        //   radius, then walks them all again at the next, criss-crossing the part.
        //
        // Each component keeps its loops together, and each is guarded by **itself**
        // rather than by the whole flat land. That is what makes the tool lift between
        // rest regions instead of skimming its tip across the finished floor to reach
        // the next one: `link_is_safe` refuses a link that leaves the region it is
        // judged against, and falls back to a retract. Within a region — and across the
        // whole flat land when no clearing tool ran, which is one region — linking is
        // still free, so a plain carve stays a single stay-down chain.
        let mut components: Vec<(Vec<Polygon>, Vec<Vec<cam_geo::Point>>)> = Vec::new();
        for comp in &floor_region {
            if cancel.is_cancelled() {
                return StrategyResult {
                    program: body,
                    diagnostics,
                    cancelled: true,
                };
            }
            if vanishing_width(comp, 0.0) <= 0.5 * floor_step {
                continue;
            }
            let mut loops: Vec<Vec<cam_geo::Point>> = Vec::new();
            let mut d = first_d;
            loop {
                let rings = if d <= 0.0 {
                    vec![comp.clone()]
                } else {
                    match offset(std::slice::from_ref(comp), -d, JoinStyle::Round) {
                        Ok(r) => r,
                        Err(e) => fail!("operation {}: floor offset failed: {e}", op.id),
                    }
                };
                if rings.is_empty() {
                    break;
                }
                for poly in &rings {
                    for contour in std::iter::once(poly.outer()).chain(poly.holes()) {
                        if contour.is_valid() && perimeter(contour) >= floor_step {
                            loops.push(contour.points().to_vec());
                        }
                    }
                }
                d += floor_step;
            }
            if !loops.is_empty() {
                components.push((vec![comp.clone()], loops));
            }
        }

        if !components.is_empty() {
            deepest = deepest.max(op.depth);
            // Regions in nearest-first order, and each region's own loops likewise, so
            // one place is finished before the tool goes anywhere else.
            let mut at = st.at.map(|(q, _)| q);
            while !components.is_empty() {
                let pick = match at {
                    None => 0,
                    Some(p) => nearest_component(&components, p),
                };
                let (guard, loops) = components.swap_remove(pick);
                for pts in order_for_locality(loops, at) {
                    emit_loop(
                        &mut body,
                        &mut st,
                        &pts,
                        &guard,
                        floor_z,
                        op,
                        &env.heights,
                        tags,
                    );
                    at = st.at.map(|(q, _)| q);
                }
            }
        }

        // Whatever happened, come home to clearance.
        if let Some((q, _)) = st.at {
            body.push(Step::Rapid {
                to: Point3::new(q.x, q.y, env.heights.clearance),
                tag: retract,
            });
        }

        if st.rings_cut == 0 {
            fail!(
                "operation {}: nothing to carve - the region vanishes {:.3} mm in, before \
                 the first ring. The hold-off may exceed the region's own width.",
                op.id,
                op.offset + widths.first().copied().unwrap_or(0.0)
            );
        }

        let mut program = Program::new();
        if let Some(c) = clear_comment {
            program.push(Step::Comment(c));
        }
        program.extend(clear_body);
        if op.clear.is_some() {
            // The planner put the clearing tool in the spindle for us (it is `tools()[0]`),
            // so the change back to the V-bit is ours to emit — and it is emitted even
            // when there was nothing to clear, or the fragment would leave the wrong tool
            // loaded for whatever the operation does next.
            program.push(Step::ToolChange { tool: op.tool });
        }
        program.push(Step::Comment(format!(
            "Carve: {} rings at {step:.3} mm wall / {floor_step:.3} mm floor, to \
             {deepest:.3} mm deep with a {included_angle_deg} deg V-bit, {} lifts",
            st.rings_cut,
            st.lifts
        )));
        program.extend(body);

        StrategyResult {
            program,
            diagnostics,
            cancelled: false,
        }
    }
}

/// Tags for the four move roles a ring pass emits.
#[derive(Clone, Copy)]
struct RingTags {
    link: Tag,
    plunge: Tag,
    cut: Tag,
    retract: Tag,
}

/// State carried across the ring passes, so pass 2 can link on from where pass 1 left
/// the tool rather than lifting between them.
#[derive(Default)]
struct RingState {
    /// Where the tool is while still down: its XY and its Z.
    at: Option<(cam_geo::Point, f64)>,
    /// The region it may traverse at that Z without gouging (see the module docs). Held
    /// one pass behind, so a new ring is judged against the region it is coming *from*.
    guard: Vec<Polygon>,
    rings_cut: usize,
    lifts: usize,
}

/// Cut every contour of `rings` at height `z`, linking without lifting where that is
/// safe, and leave `guard_next` as the region the following ring will be judged against.
#[allow(clippy::too_many_arguments)]
fn emit_rings(
    body: &mut Program,
    st: &mut RingState,
    rings: &[Polygon],
    guard_next: &[Polygon],
    z: f64,
    op: &CarveOp,
    heights: &cam_model::Heights,
    tags: RingTags,
    min_perimeter: f64,
) {
    for poly in rings {
        // A hole in the offset result is a ring around an island (or the counter of a
        // letter): it is cut at the same depth, being the same distance from a boundary.
        for contour in std::iter::once(poly.outer()).chain(poly.holes()) {
            if !contour.is_valid() {
                continue;
            }
            // A ring shorter than one step is the sliver left as an offset closes out.
            // Cutting it costs a whole plunge/retract cycle to remove almost nothing:
            // what it leaves is under a quarter of a step, well inside the ridge the
            // neighbouring pass leaves anyway.
            if min_perimeter > 0.0 && perimeter(contour) < min_perimeter {
                continue;
            }
            emit_loop(body, st, contour.points(), guard_next, z, op, heights, tags);
        }
    }
}

/// Cut one closed loop at height `z`, linking to it without lifting where that is safe,
/// and leave `guard_next` as the region the following loop will be judged against.
#[allow(clippy::too_many_arguments)]
fn emit_loop(
    body: &mut Program,
    st: &mut RingState,
    contour: &[cam_geo::Point],
    guard_next: &[Polygon],
    z: f64,
    op: &CarveOp,
    heights: &cam_model::Heights,
    tags: RingTags,
) {
    if contour.len() < 3 {
        return;
    }
    // When staying down, begin this loop at the point nearest where the tool already
    // is, so the link is as short as it can be — the operator's `start` then applies
    // only to the first loop, which is where the entry witness lands.
    let nearest = st
        .at
        .map(|(q, _)| crate::profile::rotate_to_start(contour, Some([q.x, q.y])));
    let linked = op.stay_down
        && match (&st.at, &nearest) {
            (Some((q, _)), Some(cand)) => link_is_safe(&st.guard, *q, cand[0]),
            _ => false,
        };
    let pts = match (linked, nearest) {
        (true, Some(cand)) => cand,
        _ => crate::profile::rotate_to_start(contour, op.start),
    };
    let start = pts[0];

    match (linked, st.at) {
        (true, Some((_, zq))) => {
            // Traverse at the *previous* loop's depth, where the region just verified
            // guarantees no gouge, and only then sink to this one's. Both moves cut real
            // material, so they are fed, not rapid.
            body.push(Step::Linear {
                to: Point3::new(start.x, start.y, zq),
                feed: op.feed,
                tag: tags.cut,
            });
            if (z - zq).abs() > 1e-12 {
                body.push(Step::Linear {
                    to: Point3::new(start.x, start.y, z),
                    feed: op.plunge_feed,
                    tag: tags.plunge,
                });
            }
        }
        _ => {
            // Lift and come back down: either the operator asked for it, or this
            // particular link failed its safety check.
            if let Some((q, _)) = st.at {
                body.push(Step::Rapid {
                    to: Point3::new(q.x, q.y, heights.clearance),
                    tag: tags.retract,
                });
            }
            body.push(Step::Rapid {
                to: Point3::new(start.x, start.y, heights.clearance),
                tag: tags.link,
            });
            body.push(Step::Rapid {
                to: Point3::new(start.x, start.y, heights.retract.max(op.top)),
                tag: tags.link,
            });
            body.push(Step::Linear {
                to: Point3::new(start.x, start.y, z),
                feed: op.plunge_feed,
                tag: tags.plunge,
            });
            st.lifts += 1;
        }
    }

    crate::emit::cut_loop(body, &pts, op.feed, tags.cut, z);
    st.rings_cut += 1;
    // `cut_loop` closes back to the start, so that is where the tool is.
    st.at = Some((start, z));
    st.guard = guard_next.to_vec();
}

/// Index of the rest region whose loops start nearest `p`.
fn nearest_component(
    components: &[(Vec<Polygon>, Vec<Vec<cam_geo::Point>>)],
    p: cam_geo::Point,
) -> usize {
    let mut best = (f64::MAX, 0usize);
    for (i, (_, loops)) in components.iter().enumerate() {
        for l in loops {
            let stride = (l.len() / 16).max(1);
            for q in l.iter().step_by(stride) {
                let d = (q.x - p.x).hypot(q.y - p.y);
                if d < best.0 {
                    best = (d, i);
                }
            }
        }
    }
    best.1
}

/// Order same-depth loops so each is followed by whichever is nearest — a greedy
/// nearest-neighbour walk from wherever the tool already is.
///
/// Emitting them in the order the offsets produce them means walking every component at
/// one radius, then walking them all again at the next: on a rectangle with an island
/// that is a 36 mm hop between consecutive loops, repeated for every ring. Nothing forces
/// that order — the floor is one depth and travel across it is safe anywhere — so the
/// cheapest order is simply available for the taking.
///
/// Candidate distance is measured against a bounded sample of each loop (concentric rings
/// share a centroid, so a centroid test would be blind here), then the winner is rotated
/// to its true nearest point when it is cut.
fn order_for_locality(
    mut loops: Vec<Vec<cam_geo::Point>>,
    from: Option<cam_geo::Point>,
) -> Vec<Vec<cam_geo::Point>> {
    /// Most points of one loop consulted when ranking it, so the walk stays near-linear
    /// in the number of loops rather than in their total length.
    const SAMPLES: usize = 32;

    let mut out: Vec<Vec<cam_geo::Point>> = Vec::with_capacity(loops.len());
    let mut at = from;
    while !loops.is_empty() {
        let (pick, entry) = match at {
            None => (0, None),
            Some(p) => {
                let mut best = (f64::MAX, 0usize, p);
                for (i, l) in loops.iter().enumerate() {
                    let stride = (l.len() / SAMPLES).max(1);
                    for q in l.iter().step_by(stride) {
                        let d = (q.x - p.x).hypot(q.y - p.y);
                        if d < best.0 {
                            best = (d, i, *q);
                        }
                    }
                }
                (best.1, Some(best.2))
            }
        };
        let chosen = loops.swap_remove(pick);
        // The tool will start and finish this loop at whichever of its points is nearest,
        // so that — not the loop's own first vertex — is where the next hop starts from.
        at = Some(entry.unwrap_or(chosen[0]));
        out.push(chosen);
    }
    out
}

/// The closed length of a contour, mm.
fn perimeter(c: &cam_geo::Contour) -> f64 {
    let pts = c.points();
    let mut total = 0.0;
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        total += (b.x - a.x).hypot(b.y - a.y);
    }
    total
}

/// The area a clearing pass's cutter actually swept, as polygons — its cutting moves
/// stroked by the tool radius.
///
/// This is what makes the V-bit's floor pass a *rest* pass: it is sent only where the
/// end mill could not reach, which for a flat-bottomed round cutter is every concave
/// corner of the flat land.
fn swept_by(steps: &[Step], radius: f64) -> Vec<Polygon> {
    let mut out = Vec::new();
    let mut run: Vec<cam_geo::Point> = Vec::new();
    let mut flush = |run: &mut Vec<cam_geo::Point>| {
        if run.len() >= 2 {
            let path = cam_geo::Polyline::new(std::mem::take(run));
            if let Ok(mut polys) = cam_geo::stroke_path(&path, radius, cam_geo::CapStyle::Round, JoinStyle::Round)
            {
                out.append(&mut polys);
            }
        } else {
            run.clear();
        }
    };
    for step in steps {
        match step {
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => {
                run.push(cam_geo::Point::new(to.x, to.y));
            }
            Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => {
                run.push(cam_geo::Point::new(end.x, end.y));
            }
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => {
                // A plunge starts a run at that XY: the tool is down from here.
                flush(&mut run);
                run.push(cam_geo::Point::new(to.x, to.y));
            }
            Step::Rapid { .. } => flush(&mut run),
            _ => {}
        }
    }
    flush(&mut run);
    // One region, so the difference against it is a clean rest set.
    cam_geo::union(&out, &[]).unwrap_or(out)
}

/// What a carve's shape allows, independent of running the strategy.
///
/// The inspector needs these three numbers *live*, as soon as region, tool and depth are
/// set, so it can tell the operator whether to deepen, accept, or add a clearing tool —
/// without waiting for a run. They come from the same code the strategy uses, so the
/// inspector and the diagnostics can never disagree.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarveShape {
    /// The depth at which the V exactly consumes the shape, leaving no flat floor —
    /// the shape's *natural full depth*.
    pub full_depth: f64,
    /// The deepest this V-bit can cut before its cone reaches full diameter and the
    /// shank would rub. `full_depth` beyond this cannot be reached with this tool.
    pub tool_max_depth: f64,
    /// How many separate flat areas the current depth cap leaves. `0` = none.
    pub flat_areas: usize,
}

/// Work out what `op`'s shape allows with `tool`, without generating a toolpath.
///
/// `None` when the question is not yet meaningful: the tool is not a V-bit, its angle is
/// degenerate, the depth is unset, or the boundary does not bound a region.
pub fn carve_shape(op: &CarveOp, tool: &Tool) -> Option<CarveShape> {
    let ToolKind::VBit {
        included_angle_deg,
        tip_radius,
    } = tool.kind
    else {
        return None;
    };
    let alpha = 0.5 * included_angle_deg.to_radians();
    if !(alpha > 0.0 && alpha < std::f64::consts::FRAC_PI_2) || op.depth <= 0.0 {
        return None;
    }
    let region = Polygon::with_holes(op.boundary.clone(), op.islands.clone()).ok()?;
    let w_max = vtip_half_width(alpha, tip_radius, op.depth);
    let (w_full, flat) = shape_facts(&region, op.offset, w_max);
    Some(CarveShape {
        full_depth: vtip_depth_for_half_width(alpha, tip_radius, w_full),
        tool_max_depth: vtip_max_depth(alpha, tip_radius, tool.radius()),
        flat_areas: flat.len(),
    })
}

/// The two facts the shape yields: the inward distance at which its offsets vanish, and
/// the flat land a cap at `w_max` leaves behind (empty when the V consumes the shape).
fn shape_facts(region: &Polygon, hold_off: f64, w_max: f64) -> (f64, Vec<Polygon>) {
    let w_full = vanishing_width(region, hold_off);
    let flat = if w_full > w_max + FLAT_LAND_TOL_MM {
        // Its boundary is the innermost carve ring, so the carved wall and this floor
        // meet by construction.
        offset(
            std::slice::from_ref(region),
            -(hold_off + w_max),
            JoinStyle::Round,
        )
        .unwrap_or_default()
    } else {
        Vec::new()
    };
    (w_full, flat)
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
/// distance from the boundary — that is, inside `region`. At the ring's depth the tool is
/// exactly that distance wide at the stock top (see the module docs), so anywhere inside
/// `region` it cannot reach the boundary. Both endpoints sit *on* `region`'s boundary by
/// construction, so it is the interior of the segment that is interrogated.
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

    /// Bits spanning the useful range, for the geometric safety properties.
    const VBITS_FOR_SAFETY: &[(f64, f64)] =
        &[(90.0, 0.0), (90.0, 0.2), (60.0, 0.3), (30.0, 0.05), (120.0, 0.4), (150.0, 0.1)];

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
            clear: None,
            boundary: square(20.0),
            islands: Vec::new(),
            top: 0.0,
            depth,
            offset: 0.0,
            ring_step: 0.5,
            scallop: 0.0,
            feed: 200.0,
            plunge_feed: 100.0,
            stay_down: false,
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

    /// A clearing pass with default parameters on tool `n`.
    fn clearing(n: u32) -> cam_model::CarveClearing {
        cam_model::CarveClearing {
            tool: n,
            params: cam_model::ClearParams::default(),
        }
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
            // Until the cap bites; past it the floor is flat at the cap.
            let want = inset.min(1.5);
            assert!(
                (want - d).abs() < OFFSET_TOL,
                "point ({x:.4},{y:.4}) at depth {d:.4} wants {want:.4} ({inset:.4} in)"
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
            let want = outer.min(island).min(1.0);
            assert!(
                (want + z).abs() < OFFSET_TOL,
                "({x:.4},{y:.4},{z:.4}) wants {want:.4}"
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
            let want = (inset - 1.0).min(1.0);
            assert!((want + z).abs() < OFFSET_TOL, "({x:.4},{y:.4},{z:.4}) wants {want:.4}");
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
        // Two wall rings, then the floor rings that consume the flat land.
        assert!(plunges(&r) > 2, "{}", plunges(&r));
        assert_eq!(retracts(&r), plunges(&r));
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
        o.clear = Some(clearing(2));
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
        o.clear = Some(clearing(2));
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
        o.clear = Some(clearing(2));
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
    fn each_clearing_level_follows_the_taper_rather_than_the_bottom() {
        // The point of stepping down at all: at depth d the carved surface stands
        // vtip_half_width(d) in from the boundary, so a shallow level has far more waste
        // to take than the bottom does. Clearing every level to the bottom's width would
        // hand the whole taper to the V-bit for no reason.
        let mut o = op(2.0);
        o.clear = Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams {
                stepdown: 0.5,
                ..Default::default()
            },
        });
        let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        let steps = r.program.steps();
        let change = steps
            .iter()
            .position(|s| matches!(s, Step::ToolChange { .. }))
            .unwrap();
        // For each clearing level, how far out from the 20 mm square's wall the cutter
        // reached: the tool's edge is its radius beyond the ring it rides.
        let mut reach: std::collections::BTreeMap<i64, f64> = std::collections::BTreeMap::new();
        for s in &steps[..change] {
            let (x, y, z) = match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => (to.x, to.y, to.z),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => {
                    (end.x, end.y, end.z)
                }
                _ => continue,
            };
            let inset = x.min(y).min(20.0 - x).min(20.0 - y);
            let e = reach.entry((z * 1000.0).round() as i64).or_insert(f64::MAX);
            *e = e.min(inset - 2.0); // tool radius 2
        }
        assert!(reach.len() >= 4, "expected several levels, got {reach:?}");
        // A 90 deg sharp bit: the surface at depth d is d in from the wall, so each
        // level's cutter edge must land on its own depth, not the bottom's.
        for (&zk, &edge) in &reach {
            let d = -(zk as f64) / 1000.0;
            assert!(
                (edge - d).abs() < 0.05,
                "level at depth {d:.3} reached {edge:.3} from the wall, not {d:.3}"
            );
        }
        // And they really do differ: the shallowest level reaches much further out.
        let shallow = *reach.values().next_back().unwrap();
        let deep = *reach.values().next().unwrap();
        assert!(shallow < deep - 1.0, "shallow={shallow:.3} deep={deep:.3}");
    }

    #[test]
    fn the_clearing_offset_holds_the_end_mill_off_the_carved_surface() {
        // A finishing allowance for the V-bit: the end mill must stop that much short of
        // the surface at *every* level, not just the floor.
        let mut o = op(2.0);
        o.clear = Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams {
                stepdown: 0.5,
                offset: 0.4,
                ..Default::default()
            },
        });
        let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        let steps = r.program.steps();
        let change = steps
            .iter()
            .position(|s| matches!(s, Step::ToolChange { .. }))
            .unwrap();
        for s in &steps[..change] {
            let (x, y, z) = match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => (to.x, to.y, to.z),
                Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => {
                    (end.x, end.y, end.z)
                }
                _ => continue,
            };
            let inset = x.min(y).min(20.0 - x).min(20.0 - y);
            let d = -z;
            // Edge of the cutter = inset - radius; it must stay at d + offset or beyond.
            assert!(
                inset - 2.0 >= d + 0.4 - 0.05,
                "at depth {d:.3} the cutter edge reached {:.3}, inside the {:.3} allowance",
                inset - 2.0,
                d + 0.4
            );
        }
    }

    #[test]
    fn what_the_clearing_offset_leaves_is_handed_back_to_the_v_bit() {
        // The allowance is not abandoned material: the floor pass is computed from what
        // the end mill actually swept, so a bigger allowance simply gives the V-bit more
        // to do. Measured as the length of V-bit cutting at full depth.
        let floor_len = |offset: f64| {
            let mut o = op(1.0);
            o.ring_step = 0.25;
            o.clear = Some(cam_model::CarveClearing {
                tool: 2,
                params: cam_model::ClearParams {
                    offset,
                    ..Default::default()
                },
            });
            let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
            assert!(errors(&r).is_empty(), "{:?}", errors(&r));
            let mut total = 0.0;
            let mut prev: Option<(f64, f64)> = None;
            for s in r.program.steps() {
                match s {
                    Step::Rapid { to, .. } => prev = Some((to.x, to.y)),
                    Step::Linear { to, tag, .. } | Step::Arc { end: to, tag, .. } => {
                        if tag.kind == MoveKind::Cutting && (to.z + 1.0).abs() < 1e-9 {
                            if let Some(q) = prev {
                                total += ((to.x - q.0).powi(2) + (to.y - q.1).powi(2)).sqrt();
                            }
                        }
                        prev = Some((to.x, to.y));
                    }
                    _ => {}
                }
            }
            total
        };
        let tight = floor_len(0.0);
        let generous = floor_len(0.5);
        assert!(
            generous > tight * 1.2,
            "a 0.5 mm allowance should hand the V-bit visibly more floor: {generous:.1} vs {tight:.1} mm"
        );
    }

    #[test]
    fn a_clearing_tool_missing_from_the_setup_is_an_error() {
        let mut o = op(1.0);
        o.clear = Some(clearing(9));
        let e = errors(&run(o, vbit(90.0, 0.0)));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("clearing tool 9 is not in the setup"), "{e:?}");
    }

    #[test]
    fn clearing_with_the_carving_tool_itself_is_an_error() {
        // It would be a no-op tool change and a floor the cone cannot flatten.
        let mut o = op(1.0);
        o.clear = Some(clearing(1));
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
        o.clear = Some(clearing(2));
        let e = errors(&run_with(o, &[vbit(90.0, 0.0), endmill(2, 30.0)]));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("does not fit"), "{e:?}");
    }

    #[test]
    fn a_ball_nose_clearing_tool_is_an_error_not_a_warning() {
        // The clearing tool exists to leave a flat floor. A ball nose cuts, but leaves
        // scallops -- which is what the V-bit already does, so it buys a tool change
        // and nothing else. Wrong tool for the job, not merely a worse one.
        let mut o = op(1.0);
        o.clear = Some(clearing(2));
        let mut ball = endmill(2, 4.0);
        ball.kind = ToolKind::BallMill;
        let e = errors(&run_with(o, &[vbit(90.0, 0.0), ball]));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("not flat-bottomed"), "{e:?}");
        assert!(e[0].contains("scalloped"), "{e:?}");
    }

    #[test]
    fn a_non_cutting_tipped_clearing_tool_says_so_specifically() {
        let mut o = op(1.0);
        o.clear = Some(clearing(2));
        let mut cham = endmill(2, 4.0);
        cham.kind = ToolKind::ChamferMill {
            included_angle_deg: 90.0,
            tip_diameter: 0.2,
        };
        let e = errors(&run_with(o, &[vbit(90.0, 0.0), cham]));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("non-cutting tip"), "{e:?}");
    }

    #[test]
    fn a_bull_nose_clears_fine_since_its_centre_is_flat() {
        let mut o = op(1.0);
        o.clear = Some(clearing(2));
        let mut bull = endmill(2, 4.0);
        bull.kind = ToolKind::BullNose { corner_radius: 0.5 };
        let r = run_with(o, &[vbit(90.0, 0.0), bull]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        assert!(warnings(&r).is_empty(), "{:?}", warnings(&r));
    }

    #[test]
    fn the_clearing_plunge_style_is_the_operators_choice() {
        // A straight drop into solid stock is not always wanted; a helix eases in.
        let mut o = op(1.0);
        o.clear = Some(clearing(2));
        o.clear = Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams {
                plunge: Plunge::Helix {
                    radius: 1.0,
                    pitch: 0.5,
                },
                ..Default::default()
            },
        });
        let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        // A helical entry emits arcs before the floor is reached; a straight one does not.
        let steps = r.program.steps();
        let change = steps
            .iter()
            .position(|s| matches!(s, Step::ToolChange { .. }))
            .unwrap();
        assert!(
            steps[..change]
                .iter()
                .any(|s| matches!(s, Step::Arc { tag, .. } if tag.kind == MoveKind::Plunge)),
            "no helical plunge was emitted"
        );
    }

    #[test]
    fn the_clearing_overlap_really_sets_the_ring_spacing() {
        // Not "the field is stored" but "the field moves the metal": a tighter overlap
        // must put the rings closer together, and the reported stepover must match the
        // pocket rule `diameter * (1 - overlap)`.
        let spacing_for = |overlap: f64| {
            let mut o = op(1.0);
            o.clear = Some(cam_model::CarveClearing {
                tool: 2,
                params: cam_model::ClearParams {
                    overlap,
                    ..Default::default()
                },
            });
            let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
            assert!(errors(&r).is_empty(), "{:?}", errors(&r));
            let comment = r
                .program
                .steps()
                .iter()
                .find_map(|s| match s {
                    Step::Comment(c) if c.starts_with("Carve clearing") => Some(c.clone()),
                    _ => None,
                })
                .expect("a clearing comment");
            let cuts = r
                .program
                .steps()
                .iter()
                .take_while(|s| !matches!(s, Step::ToolChange { .. }))
                .filter(|s| matches!(s, Step::Linear { tag, .. } | Step::Arc { tag, .. } if tag.kind == MoveKind::Cutting))
                .count();
            (comment, cuts)
        };
        let (half, coarse_cuts) = spacing_for(0.5);
        assert!(half.contains("2.000 mm stepover"), "{half}");
        let (tight, fine_cuts) = spacing_for(0.8);
        assert!(tight.contains("0.800 mm stepover"), "{tight}");
        assert!(
            fine_cuts > coarse_cuts,
            "a tighter overlap must cut more, got {fine_cuts} vs {coarse_cuts}"
        );
    }

    #[test]
    fn the_clearing_feeds_fall_back_to_the_carves_own() {
        // 0 means "same as the carve", which is what makes a one-click clearing pass
        // runnable. A non-zero value must win.
        let feeds_of = |feed: f64, plunge_feed: f64| {
            let mut o = op(1.0);
            o.clear = Some(cam_model::CarveClearing {
                tool: 2,
                params: cam_model::ClearParams {
                    feed,
                    plunge_feed,
                    ..Default::default()
                },
            });
            let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
            let steps = r.program.steps();
            let change = steps
                .iter()
                .position(|s| matches!(s, Step::ToolChange { .. }))
                .unwrap();
            let cut = steps[..change]
                .iter()
                .find_map(|s| match s {
                    Step::Linear { feed, tag, .. } if tag.kind == MoveKind::Cutting => Some(*feed),
                    _ => None,
                })
                .unwrap();
            let plunge = steps[..change]
                .iter()
                .find_map(|s| match s {
                    Step::Linear { feed, tag, .. } if tag.kind == MoveKind::Plunge => Some(*feed),
                    _ => None,
                })
                .unwrap();
            (cut, plunge)
        };
        // op() carves at 200 / 100.
        assert_eq!(feeds_of(0.0, 0.0), (200.0, 100.0), "0 inherits the carve's");
        assert_eq!(feeds_of(450.0, 250.0), (450.0, 250.0), "set values win");
    }

    #[test]
    fn an_out_of_range_clearing_overlap_is_rejected_not_clamped() {
        let mut o = op(1.0);
        o.clear = Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams {
                overlap: 1.0,
                ..Default::default()
            },
        });
        let e = errors(&run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]));
        assert_eq!(e.len(), 1, "{e:?}");
        assert!(e[0].contains("fraction in [0, 1)"), "{e:?}");
    }

    #[test]
    fn a_clearing_tool_that_cannot_reach_the_depth_is_an_error() {
        // 2 mm of flute cannot cut a 3 mm-deep floor: past that it is the shank in the
        // pocket, which is a hard error, not a preference.
        let mut o = op(2.5);
        o.clear = Some(clearing(2));
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
        o.clear = Some(clearing(2));
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
        o.clear = Some(clearing(2));
        o.clear = Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams {
                stepdown: 0.8,
                ..Default::default()
            },
        });
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

    #[test]
    fn carve_shape_agrees_with_what_the_strategy_reports() {
        // The inspector and the diagnostics must never disagree: same code, so pin it.
        let o = op(1.0);
        let t = vbit(90.0, 0.0);
        let shape = carve_shape(&o, &t).expect("a V-bit and a valid region");
        assert!((shape.full_depth - 10.0).abs() < 2e-3, "{shape:?}");
        assert!((shape.tool_max_depth - 3.0).abs() < 1e-9, "{shape:?}");
        assert_eq!(shape.flat_areas, 1);
        let i = infos(&run(o, t));
        assert!(i[0].contains(&format!("{:.3}", shape.full_depth)), "{i:?}");

        // A shape the carve consumes reports no flat areas, live.
        let mut consumed = op(2.9);
        consumed.boundary = square(3.0);
        let shape = carve_shape(&consumed, &t).unwrap();
        assert_eq!(shape.flat_areas, 0);
        assert!((shape.full_depth - 1.5).abs() < 2e-3, "{shape:?}");
    }

    #[test]
    fn carve_shape_declines_the_questions_it_cannot_answer() {
        let t = vbit(90.0, 0.0);
        assert!(carve_shape(&op(0.0), &t).is_none(), "an unset depth");
        assert!(
            carve_shape(&op(1.0), &tool_of(ToolKind::EndMill)).is_none(),
            "not a V-bit"
        );
        let mut open = op(1.0);
        open.boundary = Contour::new(vec![Point::new(0.0, 0.0), Point::new(1.0, 0.0)]);
        assert!(carve_shape(&open, &t).is_none(), "not a region");
    }


    #[test]
    fn nothing_stands_in_the_corners_the_clearing_tool_cannot_reach() {
        // 20 mm square, 1 mm cap, 90 deg sharp V-bit (f(x) == x), dia 4 end mill (r=2).
        let mut o = op(1.0);
        o.ring_step = 0.25;
        o.clear = Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams::default(),
        });
        let r = run_with(o, &[vbit(90.0, 0.0), endmill(2, 4.0)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        let steps = r.program.steps();
        let change = steps.iter().position(|s| matches!(s, Step::ToolChange { .. })).unwrap();

        let collect = |range: &[Step], floor_only: bool| {
            let mut segs: Vec<((f64, f64), (f64, f64))> = Vec::new();
            let mut last: Option<(f64, f64)> = None;
            for s in range {
                match s {
                    Step::Rapid { to, .. } => last = Some((to.x, to.y)),
                    Step::Linear { to, tag, .. } => {
                        let at_floor = !floor_only || (to.z + 1.0).abs() < 1e-9;
                        if tag.kind == MoveKind::Cutting && at_floor {
                            if let Some(q) = last { segs.push((q, (to.x, to.y))); }
                        }
                        last = Some((to.x, to.y));
                    }
                    Step::Arc { end, center, dir, tag, .. } => {
                        let at_floor = !floor_only || (end.z + 1.0).abs() < 1e-9;
                        if tag.kind == MoveKind::Cutting && at_floor {
                            // Densify: a chord across a 90 deg arc misses it by r(1-cos45),
                            // which for these lens boundaries is most of the answer.
                            if let Some(q) = last {
                                let c = (center.x, center.y);
                                let r0 = ((q.0 - c.0).powi(2) + (q.1 - c.1).powi(2)).sqrt();
                                let a0 = (q.1 - c.1).atan2(q.0 - c.0);
                                let a1 = (end.y - c.1).atan2(end.x - c.0);
                                let ccw = matches!(dir, cam_cldata::ArcDir::Ccw);
                                let mut sweep = a1 - a0;
                                if ccw && sweep <= 0.0 { sweep += std::f64::consts::TAU; }
                                if !ccw && sweep >= 0.0 { sweep -= std::f64::consts::TAU; }
                                let n = 64;
                                let mut prev = q;
                                for k in 1..=n {
                                    let a = a0 + sweep * (k as f64) / (n as f64);
                                    let pt = (c.0 + r0 * a.cos(), c.1 + r0 * a.sin());
                                    segs.push((prev, pt));
                                    prev = pt;
                                }
                            }
                        }
                        last = Some((end.x, end.y));
                    }
                    _ => {}
                }
            }
            segs
        };
        let mill = collect(&steps[..change], false);
        let vbit_floor = collect(&steps[change..], true);

        let dist_seg = |p: (f64, f64), a: (f64, f64), b: (f64, f64)| {
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let l2 = dx * dx + dy * dy;
            let t = if l2 <= 0.0 { 0.0 } else { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0) };
            let (cx, cy) = (a.0 + t * dx, a.1 + t * dy);
            ((p.0 - cx).powi(2) + (p.1 - cy).powi(2)).sqrt()
        };
        let (lo, hi) = (1.0_f64, 19.0_f64);   // the flat land
        let step = 0.02_f64;
        let mut worst = 0.0_f64;
        let mut worst_at = (0.0, 0.0);
        let n = ((hi - lo) / step) as usize;
        for i in 0..=n {
            for j in 0..=n {
                let p = (lo + i as f64 * step, lo + j as f64 * step);
                let dm = mill.iter().map(|(a, b)| dist_seg(p, *a, *b)).fold(f64::MAX, f64::min);
                if dm <= 2.0 + 1e-9 { continue; }             // the end mill floored it
                // Otherwise only the V-bit's cone has been here: material stands from
                // the floor up to f(distance to the nearest V-bit floor pass).
                let dv = vbit_floor.iter().map(|(a, b)| dist_seg(p, *a, *b)).fold(f64::MAX, f64::min);
                let h = dv.min(1.0);                           // 90 deg sharp: f(x) == x
                if h > worst { worst = h; worst_at = p; }
            }
        }
        // A round cutter cannot enter a sharp corner: it leaves a lens of stock at each
        // corner of the flat land, reaching r*(sqrt(2)-1) = 0.83 mm in from a right angle
        // with this dia 4 mill. Before the V-bit's floor pass existed, that lens stood
        // 0.580 mm proud of a 1.000 mm floor -- a nub over half the carve's depth.
        //
        // What may remain is the V-bit's own ridging between adjacent passes, f(step/2)
        // = 0.125 mm here. Allow a whole ring step, which is loose enough for the corner
        // geometry not to tile exactly and far tighter than the defect it guards.
        assert!(
            worst <= 0.25 + 1e-9,
            "material stands {worst:.4} mm proud at {worst_at:?}; the ring ridge is only {:.4}",
            0.25 / 2.0
        );
    }

    #[test]
    fn a_carve_with_no_clearing_tool_still_cuts_the_flat_land() {
        // The bug the floor pass also fixes, and the worse of the two: the rings used to
        // stop at w_max, which is only the EDGE of the flat land. Everything inside it
        // was left as solid stock at full height -- while the diagnostic described a
        // merely *ridged* floor. It has to be cut, ridges and all.
        let mut o = op(1.0);
        o.ring_step = 0.5;
        let r = run(o, vbit(90.0, 0.0));
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        // The flat land of a 20 mm square capped at 1 mm is [1,19]^2. Cutting must
        // happen well inside it, at full depth -- not merely on its boundary.
        let inside = cuts(&r)
            .into_iter()
            .filter(|&(x, y, z)| {
                (z + 1.0).abs() < 1e-9
                    && x > 2.0
                    && x < 18.0
                    && y > 2.0
                    && y < 18.0
            })
            .count();
        assert!(inside > 0, "the flat land was never entered");
        // And the deepest cut is still the cap, not deeper.
        let deepest = cuts(&r).into_iter().map(|(_, _, z)| z).fold(0.0_f64, f64::min);
        assert!((deepest + 1.0).abs() < 1e-9, "deepest={deepest}");
    }

    #[test]
    fn the_floor_pass_is_a_rest_pass_when_a_clearing_tool_ran() {
        // With an end mill on the flat land, the V-bit must go only where it could not
        // reach -- the corners -- not re-scrub the whole floor, which would drag its tip
        // across finished material for no gain.
        let mut with = op(1.0);
        with.ring_step = 0.25;
        with.clear = Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams::default(),
        });
        let mut without = with.clone();
        without.clear = None;
        // Length, not move count: the rest lenses are curved, so they carry many short
        // segments while covering very little ground.
        let floor_len = |r: &StrategyResult| {
            let mut total = 0.0;
            let mut prev: Option<(f64, f64)> = None;
            for s in r.program.steps() {
                match s {
                    // A rapid breaks the chain: what follows is a fresh entry, not a cut
                    // continuing from where the last one stopped.
                    Step::Rapid { to, .. } => prev = Some((to.x, to.y)),
                    Step::Linear { to, tag, .. } | Step::Arc { end: to, tag, .. } => {
                        let floor = (to.z + 1.0).abs() < 1e-9;
                        if tag.kind == MoveKind::Cutting && floor {
                            if let Some(q) = prev {
                                total += ((to.x - q.0).powi(2) + (to.y - q.1).powi(2)).sqrt();
                            }
                        }
                        prev = Some((to.x, to.y));
                    }
                    _ => {}
                }
            }
            total
        };
        let a = floor_len(&run_with(with, &[vbit(90.0, 0.0), endmill(2, 4.0)]));
        let b = floor_len(&run(without, vbit(90.0, 0.0)));
        assert!(
            a * 4.0 < b,
            "the rest pass should be a small fraction of the full floor: {a:.1} vs {b:.1} mm"
        );
    }

    #[test]
    fn the_floor_spacing_comes_from_the_scallop_not_the_wall_step() {
        // The finish control the operator actually has. A tighter ridge must give a
        // tighter spacing, and the reported numbers must be the two different things.
        let comment_of = |scallop: f64, ring_step: f64, tip: f64| {
            let mut o = op(1.0);
            o.ring_step = ring_step;
            o.scallop = scallop;
            let r = run(o, vbit(90.0, tip));
            assert!(errors(&r).is_empty(), "{:?}", errors(&r));
            r.program
                .steps()
                .iter()
                .find_map(|s| match s {
                    Step::Comment(c) if c.starts_with("Carve:") => Some(c.clone()),
                    _ => None,
                })
                .unwrap()
        };
        // 90 deg sharp bit: f(u) == u, so a 0.05 mm ridge wants a 0.100 mm spacing.
        let c = comment_of(0.05, 0.5, 0.0);
        assert!(c.contains("0.500 mm wall"), "{c}");
        assert!(c.contains("0.100 mm floor"), "{c}");
        // Halve the ridge, halve the spacing — and the wall step is untouched.
        let c = comment_of(0.025, 0.5, 0.0);
        assert!(c.contains("0.500 mm wall") && c.contains("0.050 mm floor"), "{c}");
    }

    #[test]
    fn a_rounded_tip_earns_a_wider_floor_step_at_the_same_ridge() {
        // The whole point of asking for a ridge height instead of a spacing: the tool's
        // own geometry decides how far it may step. Inside the tip ball the profile
        // widens as sqrt(depth), so a rounded tip clears a far wider band per pass than
        // a sharp one for the same ridge -- free speed, at equal finish.
        let floor_step_of = |tip: f64| {
            let mut o = op(1.0);
            o.scallop = 0.05;
            let r = run(o, vbit(90.0, tip));
            let c = r
                .program
                .steps()
                .iter()
                .find_map(|s| match s {
                    Step::Comment(c) if c.starts_with("Carve:") => Some(c.clone()),
                    _ => None,
                })
                .unwrap();
            let at = c.find("mm wall / ").unwrap() + "mm wall / ".len();
            c[at..at + 5].parse::<f64>().unwrap()
        };
        let sharp = floor_step_of(0.0);
        let rounded = floor_step_of(0.3);
        assert!(
            rounded > sharp * 2.0,
            "a 0.3 mm tip should step far wider than a sharp one: {rounded:.3} vs {sharp:.3}"
        );
        // And it is exactly 2*sqrt(2*rt*h - h^2), the tip circle's own half-width.
        let want = 2.0 * (2.0 * 0.3 * 0.05 - 0.05 * 0.05_f64).sqrt();
        assert!((rounded - want).abs() < 1e-3, "{rounded} vs {want}");
    }

    #[test]
    fn the_deepest_wall_ring_alone_cuts_the_whole_wall() {
        // Why the wall step is a *roughing* control: a ring at w puts the tip at f(w),
        // and the tool's half-width at the surface is then exactly w -- so its flank
        // runs from the boundary right down to the tip. Every shallower ring cuts a
        // narrower V wholly inside it. Pinned because the docs used to claim the
        // opposite, and it is the reason a coarser wall step costs load, not finish.
        for &(deg, rt) in &[(90.0, 0.0), (60.0, 0.0), (120.0, 0.0)] {
            let a = (deg * 0.5_f64).to_radians();
            let w_max = 1.0;
            let tip = vtip_depth_for_half_width(a, rt, w_max);
            for i in 1..20 {
                let x = w_max * i as f64 / 20.0;
                let cut = tip - vtip_depth_for_half_width(a, rt, w_max - x);
                let nominal = vtip_depth_for_half_width(a, rt, x);
                assert!(
                    (cut - nominal).abs() < 1e-9,
                    "deg={deg} x={x}: one pass gives {cut}, nominal {nominal}"
                );
            }
        }
    }

    #[test]
    fn a_shallower_ring_never_cuts_deeper_than_the_deepest_one() {
        // The second leg of the safety argument, and the one that is easy to get
        // backwards: for a fixed point x, the depth a ring at w reaches there,
        // f(w) - f(w - x), must be INCREASING in w. That is what makes the finished
        // surface the envelope of the deepest ring, and what bounds every stay-down
        // link and every floor pass. It follows from f being convex -- but a comment
        // saying so is exactly what was wrong here before, so measure it.
        for &(deg, rt) in VBITS_FOR_SAFETY {
            let a = (deg * 0.5_f64).to_radians();
            for i in 1..40 {
                let x = i as f64 * 0.05;
                let mut prev = f64::MIN;
                let mut w = x;
                while w <= 6.0 {
                    let cut = vtip_depth_for_half_width(a, rt, w)
                        - vtip_depth_for_half_width(a, rt, w - x);
                    assert!(
                        cut >= prev - 1e-12,
                        "deg={deg} rt={rt} x={x}: ring at w={w} cuts {cut}, shallower cut {prev}"
                    );
                    prev = cut;
                    w += 0.05;
                }
            }
        }
    }

    #[test]
    fn a_ring_is_exactly_its_own_width_at_the_stock_top() {
        // The first leg: a ring at inward distance w sinks the tip to f(w), and the tool
        // is then exactly w wide on each side at the surface -- so at inward distance
        // d >= w it spans [d-w, d+w] and cannot reach the boundary. Everything the
        // stay-down guard does rests on this one identity.
        for &(deg, rt) in VBITS_FOR_SAFETY {
            let a = (deg * 0.5_f64).to_radians();
            for i in 1..40 {
                let w = i as f64 * 0.1;
                let tip = vtip_depth_for_half_width(a, rt, w);
                let half_width_at_surface = vtip_half_width(a, rt, tip);
                assert!(
                    (half_width_at_surface - w).abs() < 1e-9,
                    "deg={deg} rt={rt}: ring at {w} is {half_width_at_surface} wide at the top"
                );
            }
        }
    }

    /// Andreas's sample: a 60x40 rectangle with a circular island, which is where the
    /// floor pass's ordering and its sliver rash both showed up.
    fn ringed_rect() -> (Contour, Vec<Contour>) {
        let outer = Contour::new(vec![
            Point::new(10.0, 10.0),
            Point::new(70.0, 10.0),
            Point::new(70.0, 50.0),
            Point::new(10.0, 50.0),
        ]);
        let island = Contour::new(
            (0..64)
                .map(|i| {
                    let a = std::f64::consts::TAU * i as f64 / 64.0;
                    Point::new(40.0 + 5.0 * a.cos(), 30.0 + 5.0 * a.sin())
                })
                .collect::<Vec<_>>(),
        );
        (outer, vec![island])
    }

    /// Every floor loop, as (entry XY, cut length). The entry is the plunge point, which
    /// is where the tool arrives and, since a closed loop returns to its start, also where
    /// it leaves — so consecutive entries bound the travel between loops.
    ///
    /// Measuring the *first cutting move* instead is wrong and was tried first:
    /// `cut_loop` emits its first move to the loop's **second** vertex, which on a
    /// rectangular ring is a whole side away, and turns a well-ordered path into what
    /// looks like a 35 mm hop.
    fn floor_loops_of(r: &StrategyResult, floor_z: f64) -> Vec<((f64, f64), f64)> {
        let mut out = Vec::new();
        let mut entry: Option<(f64, f64)> = None;
        let mut prev: Option<(f64, f64)> = None;
        let mut len = 0.0;
        for s in r.program.steps() {
            match s {
                Step::Linear { to, tag, .. } if tag.kind == MoveKind::Plunge => {
                    if let Some(e) = entry.take() {
                        out.push((e, len));
                    }
                    len = 0.0;
                    if (to.z - floor_z).abs() < 1e-9 {
                        entry = Some((to.x, to.y));
                    }
                    prev = Some((to.x, to.y));
                }
                Step::Linear { to, tag, .. } | Step::Arc { end: to, tag, .. } => {
                    if tag.kind == MoveKind::Cutting && (to.z - floor_z).abs() < 1e-9 {
                        if let Some(q) = prev {
                            len += (to.x - q.0).hypot(to.y - q.1);
                        }
                    }
                    prev = Some((to.x, to.y));
                }
                Step::Rapid { to, .. } => prev = Some((to.x, to.y)),
                _ => {}
            }
        }
        if let Some(e) = entry {
            out.push((e, len));
        }
        out
    }

    #[test]
    fn the_clearing_tool_leaves_no_rash_of_slivers_round_an_island() {
        // A round cutter working round a curved island leaves a scatter of thin rest
        // slivers. Cutting one costs a full lift/plunge/retract cycle to remove almost
        // nothing, and a couple of dozen of them is what showed in the viewport as
        // chaos. A component no wider than half a floor step is already inside the
        // accepted ridge, by construction, so it is dropped.
        let (outer, islands) = ringed_rect();
        let mut o = op(1.0);
        o.boundary = outer;
        o.islands = islands;
        o.clear = Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams::default(),
        });
        let r = run_with(o, &[vbit(60.0, 0.1), endmill(2, 6.8)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        // A loop worth a plunge cycle cuts at least one floor step of material.
        let loops = floor_loops_of(&r, -1.0);
        let runts: Vec<f64> = loops
            .iter()
            .map(|&(_, len)| len)
            .filter(|&len| len < 0.173)
            .collect();
        assert!(
            runts.len() <= 2,
            "{} floor loops cut less than one step: {runts:?}",
            runts.len()
        );
    }

    #[test]
    fn the_tool_lifts_between_rest_regions_but_not_within_one() {
        // Skimming the tip across a finished floor to reach the next rest region marks
        // it. Each region is guarded by itself, so a link that would leave it is refused
        // and becomes a retract -- while linking *inside* a region, and across the whole
        // flat land when no clearing tool ran, stays free.
        let (outer, islands) = ringed_rect();
        let mut with = op(1.0);
        with.boundary = outer.clone();
        with.islands = islands.clone();
        with.stay_down = true;
        with.clear = Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams::default(),
        });
        let r = run_with(with, &[vbit(60.0, 0.1), endmill(2, 6.8)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));

        // The observable is the lift count: with several disjoint rest regions the tool
        // must come up between them rather than skim across. (Measuring the longest
        // floor-depth cut was tried and is wrong -- the innermost WALL ring is also at
        // full depth, and its edges are legitimately the width of the part.)
        let lifts = r
            .program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract))
            .count();
        assert!(
            lifts > 1,
            "only {lifts} lift: the tip skimmed the finished floor between rest regions"
        );

        // …but a plain carve, whose flat land is one region, still links throughout.
        let mut plain = op(1.0);
        plain.boundary = outer;
        plain.islands = islands;
        plain.stay_down = true;
        let r = run(plain, vbit(60.0, 0.1));
        let lifts = r
            .program
            .steps()
            .iter()
            .filter(|s| matches!(s, Step::Rapid { tag, .. } if tag.kind == MoveKind::Retract))
            .count();
        assert_eq!(lifts, 1, "a single-region carve should stay down throughout");
    }

    #[test]
    fn no_emitted_arc_is_wildly_larger_than_the_part() {
        // The worst defect this operation has produced. Round an island the clearing
        // tool leaves degenerate slivers; closing one into a loop gave the arc fitter a
        // run whose start and end coincide, its near-straight guard had no chord to
        // measure, and it accepted a circumcircle METRES across. Posted, that is a
        // `G3` with equal endpoints -- a full 360 circle, at feed, right across the
        // machine. Fixed in the fitter (cam-geo) and by not emitting the slivers at all;
        // pinned here on the geometry that produced it.
        let (outer, islands) = ringed_rect();
        let mut o = op(1.0);
        o.boundary = outer;
        o.islands = islands;
        o.clear = Some(cam_model::CarveClearing {
            tool: 2,
            params: cam_model::ClearParams::default(),
        });
        let r = run_with(o, &[vbit(60.0, 0.1), endmill(2, 6.8)]);
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        // The part spans 60x40; nothing on it justifies an arc bigger than the stock.
        let mut prev: Option<(f64, f64)> = None;
        for s in r.program.steps() {
            match s {
                Step::Rapid { to, .. } | Step::Linear { to, .. } => prev = Some((to.x, to.y)),
                Step::Arc { end, center, .. } => {
                    let radius = (center.x - end.x).hypot(center.y - end.y);
                    assert!(
                        radius <= 60.0,
                        "an arc of radius {radius:.1} mm on a 60x40 part"
                    );
                    if let Some(p) = prev {
                        assert!(
                            (end.x - p.0).hypot(end.y - p.1) > 1e-9,
                            "an arc closed on itself: a full circle at ({:.3},{:.3})",
                            end.x,
                            end.y
                        );
                    }
                    prev = Some((end.x, end.y));
                }
                _ => {}
            }
        }
    }

    #[test]
    fn floor_loops_are_ordered_so_the_tool_does_not_criss_cross() {
        // The floor is all one depth and travel across it is safe anywhere, so the order
        // is free -- and the order the offsets come out in is the wrong one: every
        // component at one radius, then all of them again at the next, criss-crossing
        // the part between every pair.
        let (outer, islands) = ringed_rect();
        let mut o = op(1.0);
        o.boundary = outer;
        o.islands = islands;
        let r = run(o, vbit(60.0, 0.1));
        assert!(errors(&r).is_empty(), "{:?}", errors(&r));
        let loops = floor_loops_of(&r, -1.0);
        assert!(loops.len() > 20, "expected many floor loops, got {}", loops.len());
        let total: f64 = loops
            .windows(2)
            .map(|w| (w[1].0 .0 - w[0].0 .0).hypot(w[1].0 .1 - w[0].0 .1))
            .sum();
        let mean = total / (loops.len() - 1) as f64;
        // Well ordered, consecutive loops sit about one floor step apart; the
        // pathological order costs tens of millimetres on this sample every time.
        assert!(
            mean < 5.0,
            "mean hop between floor loops is {mean:.1} mm over {} loops - not ordered",
            loops.len()
        );
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
