//! Supported numeric operation field inventory.
use super::*;
/// The numeric parameters the inspector shows for an operation, in display order.
///
/// A free function taking only the operation, so the list can be checked without
/// standing up a GUI — `visible_fields` is a method on the iced `App` and nothing
/// headless can reach it. The same reason `origin_move_targets` was extracted: a
/// rule the tests cannot see is a rule that drifts, and this one silently governs
/// whether an operation's parameter is reachable by the operator at all.
pub(super) fn operation_fields(op: &Operation) -> Vec<Field> {
    let mut fields = match op {
        Operation::Profile(p) => {
            let mut fields = vec![Field::Depth, Field::Stepdown];
            fields.extend(profile_parameters::numeric_fields(p));
            // Radial roughing (stepover) is outside-only; an inner profile is
            // a single-pass wall finish (rough the pocket first).
            if (p.capabilities(false, true).roughing || p.stepover != 0.)
                && p.closed
                && p.additional_chains.is_empty()
                && p.side == Side::Outside
                && p.comp == cam_model::Comp::Computed
            {
                fields.push(Field::Stepover);
                fields.push(Field::Engagement);
            }
            fields.extend([Field::SpindleRpm, Field::Feed, Field::PlungeFeed]);
            if p.tabs.enabled {
                fields.extend([Field::TabWidth, Field::TabHeight]);
                if !p.tabs.manual && p.tabs.count.is_none() {
                    fields.push(Field::TabSpacing);
                }
            }
            // Lead/plunge sizes appear only when the kind uses them.
            if p.lead_in != Lead::None && p.passes.lead_in_enabled != Some(false) {
                fields.push(Field::LeadInSize);
            }
            if p.lead_out != Lead::None
                && (!p.passes.same_lead_out || p.passes.lead_in_enabled == Some(false))
                && p.passes.lead_out_enabled != Some(false)
            {
                fields.push(Field::LeadOutSize);
            }
            fields.push(Field::LeadOverlap);
            match p.plunge {
                Plunge::Straight => {}
                Plunge::Ramp { .. } => fields.push(Field::PlungeA),
                Plunge::Helix { .. } | Plunge::ZigZag { .. } => {
                    fields.push(Field::PlungeA);
                    fields.push(Field::PlungeB);
                }
            }
            fields
        }
        Operation::Pocket(p) => {
            let mut fields = vec![
                Field::LiftHeight,
                Field::Depth,
                Field::Stepdown,
                Field::PocketStepover,
                Field::Engagement,
                Field::SpindleRpm,
                Field::Feed,
                Field::PlungeFeed,
                Field::LeadOverlap,
            ];
            if p.finishing.enabled {
                fields.extend(pocket_parameters::Number::ALL.map(Field::PocketFinish));
            }
            if p.clear.lead_in != Lead::None {
                fields.push(Field::LeadInSize);
            }
            if p.clear.lead_out != Lead::None {
                fields.push(Field::LeadOutSize);
            }
            match p.clear.plunge {
                Plunge::Straight => {}
                Plunge::Ramp { .. } => fields.push(Field::PlungeA),
                Plunge::Helix { .. } | Plunge::ZigZag { .. } => {
                    fields.push(Field::PlungeA);
                    fields.push(Field::PlungeB);
                }
            }
            fields
        }
        Operation::Face(_) => vec![
            Field::FaceStartOffset,
            Field::Depth,
            Field::Stepdown,
            Field::FaceOverlap,
            Field::FaceOvershoot,
            Field::SpindleRpm,
            Field::Feed,
            Field::PlungeFeed,
        ],
        Operation::Drill(_) => vec![
            Field::Depth,
            Field::DrillStartOffset,
            Field::Peck,
            Field::Dwell,
            Field::SpindleRpm,
            Field::Feed,
        ],
        Operation::Chamfer(c) => {
            let mut fields = vec![
                // Heights first and top-down, as the Setup inspector reads.
                Field::ChamferTop,
                Field::ChamferWidth,
                Field::ChamferDepth,
                Field::ChamferTipOffset,
                Field::ChamferStep,
                Field::Stepdown,
                Field::SpindleRpm,
                Field::Feed,
                Field::PlungeFeed,
            ];
            if c.lead_in != Lead::None && c.lead_in_enabled != Some(false) {
                fields.push(Field::LeadInSize);
            }
            if c.lead_in_enabled != Some(false)
                && matches!(c.lead_in, Lead::Arc { .. } | Lead::ArcSweep { .. })
            {
                fields.push(Field::Profile(profile_parameters::Number::LeadInSweep));
            }
            if c.lead_out != Lead::None
                && (!c.same_lead_out || c.lead_in_enabled == Some(false))
                && c.lead_out_enabled != Some(false)
            {
                fields.push(Field::LeadOutSize);
            }
            if (!c.same_lead_out || c.lead_in_enabled == Some(false))
                && c.lead_out_enabled != Some(false)
                && matches!(c.lead_out, Lead::Arc { .. } | Lead::ArcSweep { .. })
            {
                fields.push(Field::Profile(profile_parameters::Number::LeadOutSweep));
            }
            if c.closed {
                fields.push(Field::LeadOverlap);
            }
            fields
        }
        Operation::Engrave(_) => {
            vec![
                Field::OperationTop,
                Field::Depth,
                Field::Stepdown,
                Field::SpindleRpm,
                Field::Feed,
                Field::PlungeFeed,
            ]
        }
        // Carve has no stepdown (single-pass, deferred) and no stepover: its
        // ring spacing plays that role, and `Depth` is a *cap* the shape may
        // not reach.
        Operation::Carve(c) => {
            let mut fields = vec![
                Field::OperationTop,
                Field::Depth,
                Field::ProfileOffset,
                Field::RingStep,
                Field::Scallop,
                Field::SpindleRpm,
                Field::Feed,
                Field::PlungeFeed,
            ];
            // The V-bit's own entry parameters, exactly as a profile's.
            match c.plunge {
                Plunge::Straight => {}
                Plunge::Ramp { .. } => fields.push(Field::PlungeA),
                Plunge::Helix { .. } | Plunge::ZigZag { .. } => {
                    fields.push(Field::PlungeA);
                    fields.push(Field::PlungeB);
                }
            }
            // The clearing pass is a pocket over a derived region, so it gets a
            // pocket's controls -- all but two. `Depth` is not its own (the
            // carve's cap sets it), and a wall finishing allowance would leave a
            // full-depth ridge at the wall/floor junction, since the V-bit's
            // innermost ring runs along that boundary rather than beside it.
            if let Some(cl) = &c.clear {
                fields.extend([
                    Field::ClearStepdown,
                    Field::ClearOverlap,
                    Field::ClearOffset,
                    Field::ClearEngagement,
                    Field::ClearFeed,
                    Field::ClearPlungeFeed,
                    Field::ClearLeadOverlap,
                ]);
                if cl.params.lead_in != Lead::None {
                    fields.push(Field::ClearLeadInSize);
                }
                if cl.params.lead_out != Lead::None {
                    fields.push(Field::ClearLeadOutSize);
                }
                match cl.params.plunge {
                    Plunge::Straight => {}
                    Plunge::Ramp { .. } => fields.push(Field::ClearPlungeA),
                    Plunge::Helix { .. } | Plunge::ZigZag { .. } => {
                        fields.push(Field::ClearPlungeA);
                        fields.push(Field::ClearPlungeB);
                    }
                }
            }
            fields
        }
        Operation::Spiral(o) => {
            let mut fields = vec![
                Field::AdaptiveOffset,
                Field::SpiralFromSlope,
                Field::SpiralToSlope,
                Field::AdaptiveTolerance,
                Field::SpiralStepover,
                Field::SpiralInnerLimit,
                Field::SpiralOuterLimit,
                Field::SpindleRpm,
                Field::Feed,
                Field::PlungeFeed,
            ];
            if o.params.retraction != cam_model::ScallopRetraction::Full {
                fields.extend([Field::ScallopSafeDistance, Field::SpiralStayDownDistance]);
            }
            if o.params.multiple_depths {
                fields.extend([
                    Field::SpiralAxialOffset,
                    Field::Stepdown,
                    Field::SpiralStepdowns,
                ]);
            }
            if o.params.smoothing {
                fields.push(Field::ScallopSmoothingTolerance);
            }
            fields
        }
        Operation::Parallel(o) => {
            let mut fields = vec![Field::ParallelStepover];
            if !o.params.scallop {
                fields.push(Field::ParallelAngle);
            }
            if o.params.scallop && o.params.scallop_smoothing {
                fields.push(Field::ScallopSmoothingTolerance);
            }
            if o.params.retraction_policy() != cam_model::ScallopRetraction::Full {
                fields.push(Field::ScallopSafeDistance);
            }
            fields.extend([
                Field::AdaptiveTolerance,
                Field::AdaptiveOffset,
                Field::SpindleRpm,
                Field::Feed,
                Field::PlungeFeed,
            ]);
            fields
        }
        Operation::Adaptive(_) => vec![
            Field::LiftHeight,
            Field::LeadInSize,
            Field::LeadOutSize,
            Field::AdaptiveTolerance,
            Field::AdaptiveOptimalLoad,
            Field::Stepdown,
            Field::AdaptiveFineStepdown,
            Field::AdaptiveOffset,
            Field::SpindleRpm,
            Field::Feed,
            Field::PlungeFeed,
        ],
        Operation::Bore(b) => {
            let mut fields = vec![
                Field::BoreDiameter,
                Field::BoreTaper,
                Field::BoreTolerance,
                Field::BoreStepovers,
                Field::BoreStepover,
                Field::BoreFinishing,
                Field::BoreRadialStock,
                Field::BoreAxialStock,
                Field::BoreSafeDistance,
                Field::SpindleRpm,
                Field::Feed,
                Field::PlungeFeed,
            ];
            fields.push(if b.ramp_angle.is_some() {
                Field::BoreAngle
            } else {
                Field::BorePitch
            });
            if !b.internal || !b.lead_to_center {
                fields.push(Field::BoreLeadRadius);
            }
            fields
        }
        Operation::Thread(t) => {
            let mut fields = vec![
                Field::MajorDia,
                Field::Pitch,
                Field::ThreadTop,
                Field::ThreadBottom,
                Field::ThreadPasses,
                Field::ThreadSpringPasses,
            ];
            // Blind-hole fields only for internal threads (a boss has no bore).
            if t.internal {
                fields.push(Field::ThreadDrillClearance);
                if t.drill_clearance > 0.0 {
                    fields.push(Field::ThreadBlindAllowance);
                }
            }
            fields.push(Field::SpindleRpm);
            fields.push(Field::Feed);
            fields.push(Field::PlungeFeed);
            fields
        }
    };
    if matches!(op, Operation::Profile(p) if p.closed)
        || matches!(
            op,
            Operation::Pocket(_)
                | Operation::Carve(_)
                | Operation::Adaptive(_)
                | Operation::Bore(_)
        )
    {
        fields.push(Field::RampFeed);
    }
    if matches!(op, Operation::Profile(p) if p.closed)
        || matches!(
            op,
            Operation::Pocket(_)
                | Operation::Adaptive(_)
                | Operation::Chamfer(_)
                | Operation::Thread(_)
                | Operation::Bore(_)
        )
        || matches!(op, Operation::Carve(c) if c.clear.is_some())
    {
        fields.extend([Field::LeadInFeed, Field::LeadOutFeed]);
    }
    if super::operation_tabs::operation_start(op).is_some() {
        fields.extend([Field::StartX, Field::StartY]);
    }
    if op.rotary_wrap().is_some() {
        fields.extend(
            rotary_view::Number::ALL
                .into_iter()
                .filter(|n| *n == rotary_view::Number::Offset)
                .map(Field::Wrap),
        );
    }
    if matches!(op, Operation::Profile(p) if p.passes.contour_roughing.enabled)
        || matches!(op, Operation::Chamfer(c) if c.contour_roughing.enabled)
    {
        fields.extend([
            Field::ContourRoughStepover,
            Field::ContourRoughSmoothing,
            Field::ContourRoughCount,
        ]);
    }

    fields
}
