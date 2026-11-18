//! Named numeric operation parameters shared by the editor and machining workers.
//! Expressions retain their original source; numeric fields are full-precision snapshots.
use crate::{Lead, Operation, Plunge, Tool, ToolKind};
use std::collections::BTreeMap;
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub enum OperationParameter {
    #[serde(rename = "cuttingSpeed")]
    CuttingSpeed,
    #[serde(rename = "feedPerTooth")]
    FeedPerTooth,
    #[serde(rename = "liftHeight")]
    LiftHeight,
    #[serde(rename = "operationTop")]
    OperationTop,
    #[serde(rename = "spindleRpm")]
    SpindleRpm,
    #[serde(rename = "rampFeed")]
    RampFeed,
    #[serde(rename = "leadInFeed")]
    LeadInFeed,
    #[serde(rename = "leadOutFeed")]
    LeadOutFeed,
    #[serde(rename = "maximumStepover")]
    MaximumStepover,
    #[serde(rename = "contourRoughSmoothing")]
    ContourRoughSmoothing,
    #[serde(rename = "contourRoughCount")]
    ContourRoughCount,
    #[serde(rename = "depth")]
    Depth,
    #[serde(rename = "stepover")]
    Stepover,
    #[serde(rename = "tabWidth")]
    TabWidth,
    #[serde(rename = "tabHeight")]
    TabHeight,
    #[serde(rename = "tabSpacing")]
    TabSpacing,
    #[serde(rename = "profileOffset")]
    ProfileOffset,
    #[serde(rename = "stepdown")]
    Stepdown,
    #[serde(rename = "feed")]
    Feed,
    #[serde(rename = "plungeFeed")]
    PlungeFeed,
    #[serde(rename = "leadInSize")]
    LeadInSize,
    #[serde(rename = "leadOutSize")]
    LeadOutSize,
    #[serde(rename = "leadOverlap")]
    LeadOverlap,
    #[serde(rename = "ringStep")]
    RingStep,
    #[serde(rename = "scallop")]
    Scallop,
    #[serde(rename = "plungeA")]
    PlungeA,
    #[serde(rename = "plungeB")]
    PlungeB,
    #[serde(rename = "clearStepdown")]
    ClearStepdown,
    #[serde(rename = "clearOverlap")]
    ClearOverlap,
    #[serde(rename = "clearOffset")]
    ClearOffset,
    #[serde(rename = "clearEngagement")]
    ClearEngagement,
    #[serde(rename = "clearFeed")]
    ClearFeed,
    #[serde(rename = "clearPlungeFeed")]
    ClearPlungeFeed,
    #[serde(rename = "clearLeadOverlap")]
    ClearLeadOverlap,
    #[serde(rename = "clearLeadInSize")]
    ClearLeadInSize,
    #[serde(rename = "clearLeadOutSize")]
    ClearLeadOutSize,
    #[serde(rename = "clearPlungeA")]
    ClearPlungeA,
    #[serde(rename = "clearPlungeB")]
    ClearPlungeB,
    #[serde(rename = "engagement")]
    Engagement,
    #[serde(rename = "overlap")]
    Overlap,
    #[serde(rename = "pocketOverlap")]
    PocketOverlap,
    #[serde(rename = "faceOverlap")]
    FaceOverlap,
    #[serde(rename = "faceStartOffset")]
    FaceStartOffset,
    #[serde(rename = "faceOvershoot")]
    FaceOvershoot,
    #[serde(rename = "drillStartOffset")]
    DrillStartOffset,
    #[serde(rename = "peck")]
    Peck,
    #[serde(rename = "dwell")]
    Dwell,
    #[serde(rename = "chamferTop")]
    ChamferTop,
    #[serde(rename = "chamferWidth")]
    ChamferWidth,
    #[serde(rename = "chamferDepth")]
    ChamferDepth,
    #[serde(rename = "chamferTipOffset")]
    ChamferTipOffset,
    #[serde(rename = "chamferStep")]
    ChamferStep,
    #[serde(rename = "boreDiameter")]
    BoreDiameter,
    #[serde(rename = "boreTaper")]
    BoreTaper,
    #[serde(rename = "borePitch")]
    BorePitch,
    #[serde(rename = "boreAngle")]
    BoreAngle,
    #[serde(rename = "boreStepovers")]
    BoreStepovers,
    #[serde(rename = "boreStepover")]
    BoreStepover,
    #[serde(rename = "boreFinishing")]
    BoreFinishing,
    #[serde(rename = "boreRadialStock")]
    BoreRadialStock,
    #[serde(rename = "boreAxialStock")]
    BoreAxialStock,
    #[serde(rename = "boreTolerance")]
    BoreTolerance,
    #[serde(rename = "boreSafeDistance")]
    BoreSafeDistance,
    #[serde(rename = "boreLeadRadius")]
    BoreLeadRadius,
    #[serde(rename = "majorDia")]
    MajorDia,
    #[serde(rename = "pitch")]
    Pitch,
    #[serde(rename = "threadTop")]
    ThreadTop,
    #[serde(rename = "threadBottom")]
    ThreadBottom,
    #[serde(rename = "threadPasses")]
    ThreadPasses,
    #[serde(rename = "threadSpringPasses")]
    ThreadSpringPasses,
    #[serde(rename = "threadDrillClearance")]
    ThreadDrillClearance,
    #[serde(rename = "threadBlindAllowance")]
    ThreadBlindAllowance,
    #[serde(rename = "parallelStepover")]
    ParallelStepover,
    #[serde(rename = "parallelAngle")]
    ParallelAngle,
    #[serde(rename = "scallopSafeDistance")]
    ScallopSafeDistance,
    #[serde(rename = "scallopSmoothingTolerance")]
    ScallopSmoothingTolerance,
    #[serde(rename = "adaptiveTolerance")]
    AdaptiveTolerance,
    #[serde(rename = "adaptiveOffset")]
    AdaptiveOffset,
    #[serde(rename = "adaptiveOptimalLoad")]
    AdaptiveOptimalLoad,
    #[serde(rename = "adaptiveFineStepdown")]
    AdaptiveFineStepdown,
    #[serde(rename = "finishingPasses")]
    ProfileFinishCount,
    #[serde(rename = "finishingStepover")]
    FinishingStepover,
    #[serde(rename = "finishFeed")]
    ProfileFinishFeed,
    #[serde(rename = "profileSpringPasses")]
    ProfileSpringPasses,
    #[serde(rename = "floorAllowance")]
    ProfileFloorLeave,
    #[serde(rename = "profileFinishDepths")]
    ProfileFinishDepths,
    #[serde(rename = "profileFinishDepthStep")]
    ProfileFinishDepthStep,
    #[serde(rename = "profileTangentialExtensionDistance")]
    ProfileTangentialExtensionDistance,
    #[serde(rename = "profileStartExtension")]
    ProfileStartExtension,
    #[serde(rename = "profileEndExtension")]
    ProfileEndExtension,
    #[serde(rename = "arcTolerance")]
    ProfileTolerance,
    #[serde(rename = "taperAngle")]
    ProfileTaper,
    #[serde(rename = "profileLeadEndDistance")]
    ProfileLeadEndDistance,
    #[serde(rename = "profileRampRpm")]
    ProfileRampRpm,
    #[serde(rename = "profileTabCount")]
    ProfileTabCount,
    #[serde(rename = "pocketFinishCount")]
    PocketFinishCount,
    #[serde(rename = "pocketFinishStepover")]
    PocketFinishStepover,
    #[serde(rename = "pocketFinishFeed")]
    PocketFinishFeed,
    #[serde(rename = "pocketFinishOverlap")]
    PocketFinishOverlap,
    #[serde(rename = "profileLeadInSweep")]
    ProfileLeadInSweep,
    #[serde(rename = "profileLeadOutSweep")]
    ProfileLeadOutSweep,
    #[serde(rename = "startX")]
    StartX,
    #[serde(rename = "startY")]
    StartY,
}
impl OperationParameter {
    pub const ALL: [Self; 103] = [
        Self::CuttingSpeed,
        Self::FeedPerTooth,
        Self::LiftHeight,
        Self::OperationTop,
        Self::SpindleRpm,
        Self::RampFeed,
        Self::LeadInFeed,
        Self::LeadOutFeed,
        Self::MaximumStepover,
        Self::ContourRoughSmoothing,
        Self::ContourRoughCount,
        Self::Depth,
        Self::Stepover,
        Self::TabWidth,
        Self::TabHeight,
        Self::TabSpacing,
        Self::ProfileOffset,
        Self::Stepdown,
        Self::Feed,
        Self::PlungeFeed,
        Self::LeadInSize,
        Self::LeadOutSize,
        Self::LeadOverlap,
        Self::RingStep,
        Self::Scallop,
        Self::PlungeA,
        Self::PlungeB,
        Self::ClearStepdown,
        Self::ClearOverlap,
        Self::ClearOffset,
        Self::ClearEngagement,
        Self::ClearFeed,
        Self::ClearPlungeFeed,
        Self::ClearLeadOverlap,
        Self::ClearLeadInSize,
        Self::ClearLeadOutSize,
        Self::ClearPlungeA,
        Self::ClearPlungeB,
        Self::Engagement,
        Self::Overlap,
        Self::PocketOverlap,
        Self::FaceOverlap,
        Self::FaceStartOffset,
        Self::FaceOvershoot,
        Self::DrillStartOffset,
        Self::Peck,
        Self::Dwell,
        Self::ChamferTop,
        Self::ChamferWidth,
        Self::ChamferDepth,
        Self::ChamferTipOffset,
        Self::ChamferStep,
        Self::BoreDiameter,
        Self::BoreTaper,
        Self::BorePitch,
        Self::BoreAngle,
        Self::BoreStepovers,
        Self::BoreStepover,
        Self::BoreFinishing,
        Self::BoreRadialStock,
        Self::BoreAxialStock,
        Self::BoreTolerance,
        Self::BoreSafeDistance,
        Self::BoreLeadRadius,
        Self::MajorDia,
        Self::Pitch,
        Self::ThreadTop,
        Self::ThreadBottom,
        Self::ThreadPasses,
        Self::ThreadSpringPasses,
        Self::ThreadDrillClearance,
        Self::ThreadBlindAllowance,
        Self::ParallelStepover,
        Self::ParallelAngle,
        Self::ScallopSafeDistance,
        Self::ScallopSmoothingTolerance,
        Self::AdaptiveTolerance,
        Self::AdaptiveOffset,
        Self::AdaptiveOptimalLoad,
        Self::AdaptiveFineStepdown,
        Self::ProfileFinishCount,
        Self::FinishingStepover,
        Self::ProfileFinishFeed,
        Self::ProfileSpringPasses,
        Self::ProfileFloorLeave,
        Self::ProfileFinishDepths,
        Self::ProfileFinishDepthStep,
        Self::ProfileTangentialExtensionDistance,
        Self::ProfileStartExtension,
        Self::ProfileEndExtension,
        Self::ProfileTolerance,
        Self::ProfileTaper,
        Self::ProfileLeadEndDistance,
        Self::ProfileRampRpm,
        Self::ProfileTabCount,
        Self::PocketFinishCount,
        Self::PocketFinishStepover,
        Self::PocketFinishFeed,
        Self::PocketFinishOverlap,
        Self::ProfileLeadInSweep,
        Self::ProfileLeadOutSweep,
        Self::StartX,
        Self::StartY,
    ];
    pub fn name(self) -> &'static str {
        match self {
            Self::CuttingSpeed => "cuttingSpeed",
            Self::FeedPerTooth => "feedPerTooth",
            Self::LiftHeight => "liftHeight",
            Self::OperationTop => "operationTop",
            Self::SpindleRpm => "spindleRpm",
            Self::RampFeed => "rampFeed",
            Self::LeadInFeed => "leadInFeed",
            Self::LeadOutFeed => "leadOutFeed",
            Self::MaximumStepover => "maximumStepover",
            Self::ContourRoughSmoothing => "contourRoughSmoothing",
            Self::ContourRoughCount => "contourRoughCount",
            Self::Depth => "depth",
            Self::Stepover => "stepover",
            Self::TabWidth => "tabWidth",
            Self::TabHeight => "tabHeight",
            Self::TabSpacing => "tabSpacing",
            Self::ProfileOffset => "profileOffset",
            Self::Stepdown => "stepdown",
            Self::Feed => "feed",
            Self::PlungeFeed => "plungeFeed",
            Self::LeadInSize => "leadInSize",
            Self::LeadOutSize => "leadOutSize",
            Self::LeadOverlap => "leadOverlap",
            Self::RingStep => "ringStep",
            Self::Scallop => "scallop",
            Self::PlungeA => "plungeA",
            Self::PlungeB => "plungeB",
            Self::ClearStepdown => "clearStepdown",
            Self::ClearOverlap => "clearOverlap",
            Self::ClearOffset => "clearOffset",
            Self::ClearEngagement => "clearEngagement",
            Self::ClearFeed => "clearFeed",
            Self::ClearPlungeFeed => "clearPlungeFeed",
            Self::ClearLeadOverlap => "clearLeadOverlap",
            Self::ClearLeadInSize => "clearLeadInSize",
            Self::ClearLeadOutSize => "clearLeadOutSize",
            Self::ClearPlungeA => "clearPlungeA",
            Self::ClearPlungeB => "clearPlungeB",
            Self::Engagement => "engagement",
            Self::Overlap => "overlap",
            Self::PocketOverlap => "pocketOverlap",
            Self::FaceOverlap => "faceOverlap",
            Self::FaceStartOffset => "faceStartOffset",
            Self::FaceOvershoot => "faceOvershoot",
            Self::DrillStartOffset => "drillStartOffset",
            Self::Peck => "peck",
            Self::Dwell => "dwell",
            Self::ChamferTop => "chamferTop",
            Self::ChamferWidth => "chamferWidth",
            Self::ChamferDepth => "chamferDepth",
            Self::ChamferTipOffset => "chamferTipOffset",
            Self::ChamferStep => "chamferStep",
            Self::BoreDiameter => "boreDiameter",
            Self::BoreTaper => "boreTaper",
            Self::BorePitch => "borePitch",
            Self::BoreAngle => "boreAngle",
            Self::BoreStepovers => "boreStepovers",
            Self::BoreStepover => "boreStepover",
            Self::BoreFinishing => "boreFinishing",
            Self::BoreRadialStock => "boreRadialStock",
            Self::BoreAxialStock => "boreAxialStock",
            Self::BoreTolerance => "boreTolerance",
            Self::BoreSafeDistance => "boreSafeDistance",
            Self::BoreLeadRadius => "boreLeadRadius",
            Self::MajorDia => "majorDia",
            Self::Pitch => "pitch",
            Self::ThreadTop => "threadTop",
            Self::ThreadBottom => "threadBottom",
            Self::ThreadPasses => "threadPasses",
            Self::ThreadSpringPasses => "threadSpringPasses",
            Self::ThreadDrillClearance => "threadDrillClearance",
            Self::ThreadBlindAllowance => "threadBlindAllowance",
            Self::ParallelStepover => "parallelStepover",
            Self::ParallelAngle => "parallelAngle",
            Self::ScallopSafeDistance => "scallopSafeDistance",
            Self::ScallopSmoothingTolerance => "scallopSmoothingTolerance",
            Self::AdaptiveTolerance => "adaptiveTolerance",
            Self::AdaptiveOffset => "adaptiveOffset",
            Self::AdaptiveOptimalLoad => "adaptiveOptimalLoad",
            Self::AdaptiveFineStepdown => "adaptiveFineStepdown",
            Self::ProfileFinishCount => "finishingPasses",
            Self::FinishingStepover => "finishingStepover",
            Self::ProfileFinishFeed => "finishFeed",
            Self::ProfileSpringPasses => "profileSpringPasses",
            Self::ProfileFloorLeave => "floorAllowance",
            Self::ProfileFinishDepths => "profileFinishDepths",
            Self::ProfileFinishDepthStep => "profileFinishDepthStep",
            Self::ProfileTangentialExtensionDistance => "profileTangentialExtensionDistance",
            Self::ProfileStartExtension => "profileStartExtension",
            Self::ProfileEndExtension => "profileEndExtension",
            Self::ProfileTolerance => "arcTolerance",
            Self::ProfileTaper => "taperAngle",
            Self::ProfileLeadEndDistance => "profileLeadEndDistance",
            Self::ProfileRampRpm => "profileRampRpm",
            Self::ProfileTabCount => "profileTabCount",
            Self::PocketFinishCount => "pocketFinishCount",
            Self::PocketFinishStepover => "pocketFinishStepover",
            Self::PocketFinishFeed => "pocketFinishFeed",
            Self::PocketFinishOverlap => "pocketFinishOverlap",
            Self::ProfileLeadInSweep => "profileLeadInSweep",
            Self::ProfileLeadOutSweep => "profileLeadOutSweep",
            Self::StartX => "startX",
            Self::StartY => "startY",
        }
    }
    pub fn default_expression(self) -> &'static str {
        match self {
            Self::FinishingStepover => "Math.min(tool_diameter * 0.1; maximumStepover)",
            Self::MaximumStepover => {
                "Math.max((tool_diameter - (2 * tool_cornerRadius)) * 0.95; tool_diameter * 0.20)"
            }
            _ => "0",
        }
    }
    pub fn value(self, op: &Operation) -> Option<f64> {
        match (op, self) {
            (_, Self::CuttingSpeed | Self::FeedPerTooth) => Some(0.),
            (Operation::Pocket(o), Self::LiftHeight) => Some(o.clear.lift_height),
            (Operation::Adaptive(o), Self::LiftHeight) => Some(o.params.lift_height),
            (Operation::Parallel(o), Self::LiftHeight) => Some(o.params.lift_height),
            (Operation::Engrave(op), Self::OperationTop) => Some(op.top),
            (Operation::Carve(op), Self::OperationTop) => Some(op.top),
            (op, Self::SpindleRpm) => Some(op.spindle_rpm()),
            (op, Self::RampFeed) => Some(op.entry_feeds().ramp.unwrap_or(0.)),
            (op, Self::LeadInFeed) => Some(op.entry_feeds().lead_in.unwrap_or(0.)),
            (op, Self::LeadOutFeed) => Some(op.entry_feeds().lead_out.unwrap_or(0.)),
            (Operation::Profile(o), Self::MaximumStepover) => {
                Some(o.passes.contour_roughing.stepover)
            }
            (Operation::Profile(o), Self::ContourRoughSmoothing) => {
                Some(o.passes.contour_roughing.smoothing_deviation)
            }
            (Operation::Chamfer(o), Self::ContourRoughSmoothing) => {
                Some(o.contour_roughing.smoothing_deviation)
            }
            (Operation::Profile(o), Self::ContourRoughCount) => {
                Some(o.passes.contour_roughing.stepovers as f64)
            }
            (Operation::Chamfer(o), Self::MaximumStepover) => Some(o.contour_roughing.stepover),
            (Operation::Chamfer(o), Self::ContourRoughCount) => {
                Some(o.contour_roughing.stepovers as f64)
            }
            (Operation::Profile(o), Self::Depth) => Some(o.depth),
            (Operation::Profile(o), Self::Stepover) => Some(o.stepover),
            (Operation::Profile(o), Self::TabWidth) => Some(o.tabs.width),
            (Operation::Profile(o), Self::TabHeight) => Some(o.tabs.height),
            (Operation::Profile(o), Self::TabSpacing) => Some(o.tabs.spacing),
            (Operation::Profile(o), Self::ProfileOffset) => Some(o.offset),
            (Operation::Profile(o), Self::Stepdown) => Some(o.stepdown),
            (Operation::Profile(o), Self::Feed) => Some(o.feed),
            (Operation::Profile(o), Self::PlungeFeed) => Some(o.plunge_feed),
            (Operation::Profile(o), Self::LeadInSize) => Some(lead_size(o.lead_in)),
            (Operation::Profile(o), Self::LeadOutSize) => Some(lead_size(o.lead_out)),
            (Operation::Profile(o), Self::LeadOverlap) => Some(o.lead_overlap),
            (Operation::Carve(o), Self::Depth) => Some(o.depth),
            (Operation::Carve(o), Self::ProfileOffset) => Some(o.offset),
            (Operation::Carve(o), Self::RingStep) => Some(o.ring_step),
            (Operation::Carve(o), Self::Scallop) => Some(o.scallop),
            (Operation::Carve(o), Self::Feed) => Some(o.feed),
            (Operation::Carve(o), Self::PlungeFeed) => Some(o.plunge_feed),
            (Operation::Carve(o), Self::PlungeA) => Some(plunge_params(o.plunge).0),
            (Operation::Carve(o), Self::PlungeB) => Some(plunge_params(o.plunge).1),
            (Operation::Carve(o), Self::ClearStepdown) => Some(o.clear?.params.stepdown),
            (Operation::Carve(o), Self::ClearOverlap) => Some(o.clear?.params.overlap * 100.0),
            (Operation::Carve(o), Self::ClearOffset) => Some(o.clear?.params.offset),
            (Operation::Carve(o), Self::ClearEngagement) => {
                Some(o.clear?.params.clearing.engagement)
            }
            (Operation::Carve(o), Self::ClearFeed) => Some(o.clear?.params.feed),
            (Operation::Carve(o), Self::ClearPlungeFeed) => Some(o.clear?.params.plunge_feed),
            (Operation::Carve(o), Self::ClearLeadOverlap) => Some(o.clear?.params.lead_overlap),
            (Operation::Carve(o), Self::ClearLeadInSize) => {
                Some(lead_size(o.clear?.params.lead_in))
            }
            (Operation::Carve(o), Self::ClearLeadOutSize) => {
                Some(lead_size(o.clear?.params.lead_out))
            }
            (Operation::Carve(o), Self::ClearPlungeA) => {
                Some(plunge_params(o.clear?.params.plunge).0)
            }
            (Operation::Carve(o), Self::ClearPlungeB) => {
                Some(plunge_params(o.clear?.params.plunge).1)
            }
            (Operation::Profile(o), Self::Engagement) => Some(o.clearing.engagement),
            (Operation::Profile(o), Self::PlungeA) => Some(plunge_params(o.plunge).0),
            (Operation::Profile(o), Self::PlungeB) => Some(plunge_params(o.plunge).1),
            (Operation::Pocket(o), Self::Depth) => Some(o.depth),
            (Operation::Pocket(o), Self::Stepdown) => Some(o.clear.stepdown),
            (Operation::Pocket(o), Self::Overlap) => Some(o.clear.overlap * 100.0),
            (Operation::Pocket(o), Self::PocketOverlap) => {
                Some(o.clear.pocket_overlap.unwrap_or(0.0))
            }
            (Operation::Pocket(o), Self::ProfileOffset) => Some(o.clear.offset),
            (Operation::Pocket(o), Self::LeadInSize) => Some(lead_size(o.clear.lead_in)),
            (Operation::Pocket(o), Self::LeadOutSize) => Some(lead_size(o.clear.lead_out)),
            (Operation::Pocket(o), Self::Feed) => Some(o.clear.feed),
            (Operation::Pocket(o), Self::PlungeFeed) => Some(o.clear.plunge_feed),
            (Operation::Pocket(o), Self::LeadOverlap) => Some(o.clear.lead_overlap),
            (Operation::Pocket(o), Self::Engagement) => Some(o.clear.clearing.engagement),
            (Operation::Pocket(o), Self::PlungeA) => Some(plunge_params(o.clear.plunge).0),
            (Operation::Pocket(o), Self::PlungeB) => Some(plunge_params(o.clear.plunge).1),
            (Operation::Face(o), Self::FaceStartOffset) => Some(o.start_offset),
            (Operation::Face(o), Self::Depth) => Some(o.depth),
            (Operation::Face(o), Self::Stepdown) => Some(o.stepdown),
            (Operation::Face(o), Self::Overlap) => Some(o.overlap * 100.0),
            (Operation::Face(o), Self::FaceOverlap) => Some(o.absolute_overlap.unwrap_or(0.0)),
            (Operation::Face(o), Self::FaceOvershoot) => Some(o.overshoot),
            (Operation::Face(o), Self::Feed) => Some(o.feed),
            (Operation::Face(o), Self::PlungeFeed) => Some(o.plunge_feed),
            (Operation::Drill(o), Self::Depth) => Some(o.depth),
            (Operation::Drill(o), Self::DrillStartOffset) => Some(o.start_offset),
            (Operation::Drill(o), Self::Peck) => Some(o.peck.unwrap_or(0.0)),
            (Operation::Drill(o), Self::Dwell) => Some(o.dwell.unwrap_or(0.0)),
            (Operation::Drill(o), Self::Feed) => Some(o.feed),
            (Operation::Engrave(o), Self::Depth) => Some(o.depth),
            (Operation::Engrave(o), Self::Stepdown) => Some(o.stepdown),
            (Operation::Engrave(o), Self::Feed) => Some(o.feed),
            (Operation::Engrave(o), Self::PlungeFeed) => Some(o.plunge_feed),
            (Operation::Chamfer(o), Self::ChamferTop) => Some(o.top),
            (Operation::Chamfer(o), Self::ChamferWidth) => Some(o.width),
            (Operation::Chamfer(o), Self::ChamferDepth) => Some(o.depth),
            (Operation::Chamfer(o), Self::ChamferTipOffset) => Some(o.tip_offset),
            (Operation::Chamfer(o), Self::ChamferStep) => Some(o.step),
            (Operation::Chamfer(o), Self::Stepdown) => Some(o.stepdown),
            (Operation::Chamfer(o), Self::Feed) => Some(o.feed),
            (Operation::Chamfer(o), Self::PlungeFeed) => Some(o.plunge_feed),
            (Operation::Chamfer(o), Self::LeadInSize) => Some(lead_size(o.lead_in)),
            (Operation::Chamfer(o), Self::LeadOutSize) => Some(lead_size(o.lead_out)),
            (Operation::Chamfer(o), Self::LeadOverlap) => Some(o.lead_overlap),
            (Operation::Bore(o), Self::BoreDiameter) => Some(o.diameter),
            (Operation::Bore(o), Self::BoreTaper) => Some(o.taper_angle),
            (Operation::Bore(o), Self::BorePitch) => Some(o.pitch),
            (Operation::Bore(o), Self::BoreAngle) => Some(o.ramp_angle.unwrap_or(3.)),
            (Operation::Bore(o), Self::BoreStepovers) => Some(o.stepovers as f64),
            (Operation::Bore(o), Self::BoreStepover) => Some(o.stepover),
            (Operation::Bore(o), Self::BoreFinishing) => Some(o.finishing_stepover),
            (Operation::Bore(o), Self::BoreRadialStock) => Some(o.radial_stock),
            (Operation::Bore(o), Self::BoreAxialStock) => Some(o.axial_stock),
            (Operation::Bore(o), Self::BoreTolerance) => Some(o.tolerance),
            (Operation::Bore(o), Self::BoreSafeDistance) => Some(o.safe_distance),
            (Operation::Bore(o), Self::BoreLeadRadius) => Some(o.lead_radius),
            (Operation::Bore(o), Self::Feed) => Some(o.feed),
            (Operation::Bore(o), Self::PlungeFeed) => Some(o.plunge_feed),
            (Operation::Thread(o), Self::MajorDia) => Some(o.major_dia),
            (Operation::Thread(o), Self::Pitch) => Some(o.pitch),
            (Operation::Thread(o), Self::ThreadTop) => Some(o.z_top),
            (Operation::Thread(o), Self::ThreadBottom) => Some(o.z_bottom),
            (Operation::Thread(o), Self::ThreadPasses) => Some(o.passes as f64),
            (Operation::Thread(o), Self::ThreadSpringPasses) => Some(o.spring_passes as f64),
            (Operation::Thread(o), Self::ThreadDrillClearance) => Some(o.drill_clearance),
            (Operation::Thread(o), Self::ThreadBlindAllowance) => Some(o.blind_allowance),
            (Operation::Thread(o), Self::Feed) => Some(o.feed),
            (Operation::Thread(o), Self::PlungeFeed) => Some(o.plunge_feed),
            (Operation::Parallel(o), Self::ParallelStepover) => Some(o.params.stepover),
            (Operation::Parallel(o), Self::ParallelAngle) => Some(o.params.angle),
            (Operation::Parallel(o), Self::ScallopSafeDistance) => {
                Some(o.params.scallop_safe_distance)
            }
            (Operation::Parallel(o), Self::ScallopSmoothingTolerance) => {
                Some(o.params.scallop_smoothing_tolerance)
            }
            (Operation::Parallel(o), Self::AdaptiveTolerance) => Some(o.params.tolerance),
            (Operation::Parallel(o), Self::AdaptiveOffset) => Some(o.params.additional_offset),
            (Operation::Parallel(o), Self::Feed) => Some(o.params.feed),
            (Operation::Parallel(o), Self::PlungeFeed) => Some(o.params.plunge_feed),
            (Operation::Adaptive(o), Self::AdaptiveTolerance) => Some(o.params.tolerance),
            (Operation::Adaptive(o), Self::AdaptiveOptimalLoad) => Some(o.params.optimal_load),
            (Operation::Adaptive(o), Self::Stepdown) => Some(o.params.stepdown),
            (Operation::Adaptive(o), Self::AdaptiveFineStepdown) => Some(o.params.fine_stepdown),
            (Operation::Adaptive(o), Self::AdaptiveOffset) => Some(o.params.additional_offset),
            (Operation::Adaptive(o), Self::Feed) => Some(o.params.feed),
            (Operation::Adaptive(o), Self::PlungeFeed) => Some(o.params.plunge_feed),
            (Operation::Profile(o), Self::ProfileFinishCount) => Some(o.passes.finish_count as f64),
            (Operation::Profile(o), Self::FinishingStepover) => Some(o.passes.finish_stepover),
            (Operation::Profile(o), Self::ProfileFinishFeed) => Some(o.passes.finish_feed),
            (Operation::Profile(o), Self::ProfileSpringPasses) => {
                Some(o.passes.spring_passes as f64)
            }
            (Operation::Profile(o), Self::ProfileFloorLeave) => Some(o.passes.floor_allowance),
            (Operation::Profile(o), Self::ProfileFinishDepths) => {
                Some(o.passes.finishing_depths as f64)
            }
            (Operation::Profile(o), Self::ProfileFinishDepthStep) => {
                Some(o.passes.finishing_stepdown)
            }
            (Operation::Profile(o), Self::ProfileTangentialExtensionDistance) => {
                Some(o.passes.tangential_extension_distance)
            }
            (Operation::Profile(o), Self::ProfileStartExtension) => Some(o.passes.start_extension),
            (Operation::Profile(o), Self::ProfileEndExtension) => Some(o.passes.end_extension),
            (Operation::Profile(o), Self::ProfileTolerance) => Some(o.passes.tolerance),
            (Operation::Profile(o), Self::ProfileTaper) => Some(o.passes.taper_angle),
            (Operation::Profile(o), Self::ProfileLeadEndDistance) => {
                Some(o.passes.lead_end_distance)
            }
            (Operation::Profile(o), Self::ProfileRampRpm) => Some(o.passes.ramp_rpm),
            (Operation::Profile(o), Self::ProfileTabCount) => {
                Some(o.tabs.count.unwrap_or(4) as f64)
            }
            (Operation::Pocket(o), Self::PocketFinishCount) => Some(o.finishing.count as f64),
            (Operation::Pocket(o), Self::PocketFinishStepover) => Some(o.finishing.stepover),
            (Operation::Pocket(o), Self::PocketFinishFeed) => Some(o.finishing.feed),
            (Operation::Pocket(o), Self::PocketFinishOverlap) => Some(o.finishing.overlap),
            (Operation::Profile(o), Self::ProfileLeadInSweep) => Some(match o.lead_in {
                Lead::ArcSweep { angle_deg, .. } => angle_deg,
                _ => 90.,
            }),
            (Operation::Chamfer(o), Self::ProfileLeadInSweep) => Some(match o.lead_in {
                Lead::ArcSweep { angle_deg, .. } => angle_deg,
                _ => 90.,
            }),
            (Operation::Profile(o), Self::ProfileLeadOutSweep) => Some(match o.lead_out {
                Lead::ArcSweep { angle_deg, .. } => angle_deg,
                _ => 90.,
            }),
            (Operation::Chamfer(o), Self::ProfileLeadOutSweep) => Some(match o.lead_out {
                Lead::ArcSweep { angle_deg, .. } => angle_deg,
                _ => 90.,
            }),
            (Operation::Profile(o), Self::StartX) => Some(o.start?[0]),
            (Operation::Profile(o), Self::StartY) => Some(o.start?[1]),
            (Operation::Pocket(o), Self::StartX) => Some(o.start?[0]),
            (Operation::Pocket(o), Self::StartY) => Some(o.start?[1]),
            (Operation::Chamfer(o), Self::StartX) => Some(o.start?[0]),
            (Operation::Chamfer(o), Self::StartY) => Some(o.start?[1]),
            (Operation::Engrave(o), Self::StartX) => Some(o.start?[0]),
            (Operation::Engrave(o), Self::StartY) => Some(o.start?[1]),
            (Operation::Carve(o), Self::StartX) => Some(o.start?[0]),
            (Operation::Carve(o), Self::StartY) => Some(o.start?[1]),
            (Operation::Adaptive(o), Self::StartX) => Some(o.params.start?[0]),
            (Operation::Adaptive(o), Self::StartY) => Some(o.params.start?[1]),
            (Operation::Parallel(o), Self::StartX) => Some(o.params.scallop_start?[0]),
            (Operation::Parallel(o), Self::StartY) => Some(o.params.scallop_start?[1]),
            _ => None,
        }
    }
    pub fn expression(self, op: &Operation) -> Option<&str> {
        if let Some(source) = op.parameter_expressions().get(&self) {
            return Some(source);
        }
        // Read bindings saved by the initial proof of concept.
        match (self, op) {
            (Self::FinishingStepover, Operation::Profile(p)) => {
                p.passes.finish_stepover_expression.as_deref()
            }
            (Self::MaximumStepover, Operation::Profile(p)) => {
                p.passes.contour_roughing.stepover_expression.as_deref()
            }
            (Self::MaximumStepover, Operation::Chamfer(p)) => {
                p.contour_roughing.stepover_expression.as_deref()
            }
            _ => None,
        }
    }
    pub fn set_source(self, op: &mut Operation, source: &str) {
        if self.value(op).is_none() {
            return;
        }
        let source = source.trim();
        let formula = source.strip_prefix('=');
        let literal = if formula.is_some() {
            None
        } else {
            source.parse::<f64>().ok()
        };
        if let Some(value) = literal {
            self.set_value(op, value);
        }
        // Clear the legacy slot too, so a literal always overrides a formula.
        match (self, &mut *op) {
            (Self::FinishingStepover, Operation::Profile(p)) => {
                p.passes.finish_stepover_expression = None
            }
            (Self::MaximumStepover, Operation::Profile(p)) => {
                p.passes.contour_roughing.stepover_expression = None
            }
            (Self::MaximumStepover, Operation::Chamfer(p)) => {
                p.contour_roughing.stepover_expression = None
            }
            _ => {}
        }
        if literal.is_none() {
            op.parameter_expressions_mut()
                .insert(self, formula.unwrap_or(source).trim().into());
        } else {
            op.parameter_expressions_mut().remove(&self);
        }
    }
    fn set_value(self, op: &mut Operation, value: f64) {
        match (op, self) {
            (Operation::Pocket(o), Self::LiftHeight) => {
                o.clear.lift_height = value;
            }
            (Operation::Adaptive(o), Self::LiftHeight) => {
                o.params.lift_height = value;
            }
            (Operation::Parallel(o), Self::LiftHeight) => {
                o.params.lift_height = value;
            }
            (Operation::Engrave(op), Self::OperationTop) => {
                op.top = value;
            }
            (Operation::Carve(op), Self::OperationTop) => {
                op.top = value;
            }
            (op, Self::SpindleRpm) => {
                op.set_spindle_rpm(value);
            }
            (op, Self::RampFeed) => {
                op.entry_feeds_mut().ramp = (value > 0.).then_some(value);
            }
            (op, Self::LeadInFeed) => {
                op.entry_feeds_mut().lead_in = (value > 0.).then_some(value);
            }
            (op, Self::LeadOutFeed) => {
                op.entry_feeds_mut().lead_out = (value > 0.).then_some(value);
            }
            (Operation::Profile(o), Self::MaximumStepover) => {
                o.passes.contour_roughing.stepover = value;
            }
            (Operation::Profile(o), Self::ContourRoughSmoothing) => {
                o.passes.contour_roughing.smoothing_deviation = value;
            }
            (Operation::Chamfer(o), Self::ContourRoughSmoothing) => {
                o.contour_roughing.smoothing_deviation = value;
            }
            (Operation::Profile(o), Self::ContourRoughCount) => {
                o.passes.contour_roughing.stepovers = value as u32;
            }
            (Operation::Chamfer(o), Self::MaximumStepover) => {
                o.contour_roughing.stepover = value;
            }
            (Operation::Chamfer(o), Self::ContourRoughCount) => {
                o.contour_roughing.stepovers = value as u32;
            }
            (Operation::Profile(o), Self::Depth) => {
                o.depth = value;
            }
            (Operation::Profile(o), Self::Stepover) => {
                o.stepover = value;
            }
            (Operation::Profile(o), Self::TabWidth) => {
                o.tabs.width = value;
            }
            (Operation::Profile(o), Self::TabHeight) => {
                o.tabs.height = value;
            }
            (Operation::Profile(o), Self::TabSpacing) => {
                o.tabs.spacing = value;
            }
            (Operation::Profile(o), Self::ProfileOffset) => {
                o.offset = value;
            }
            (Operation::Profile(o), Self::Stepdown) => {
                o.stepdown = value;
            }
            (Operation::Profile(o), Self::Feed) => {
                o.feed = value;
            }
            (Operation::Profile(o), Self::PlungeFeed) => {
                o.plunge_feed = value;
            }
            (Operation::Profile(o), Self::LeadInSize) => {
                o.lead_in = set_lead_size(o.lead_in, value);
            }
            (Operation::Profile(o), Self::LeadOutSize) => {
                o.lead_out = set_lead_size(o.lead_out, value);
            }
            (Operation::Profile(o), Self::LeadOverlap) => {
                o.lead_overlap = value;
            }
            (Operation::Carve(o), Self::Depth) => {
                o.depth = value;
            }
            (Operation::Carve(o), Self::ProfileOffset) => {
                o.offset = value;
            }
            (Operation::Carve(o), Self::RingStep) => {
                o.ring_step = value;
            }
            (Operation::Carve(o), Self::Scallop) => {
                o.scallop = value;
            }
            (Operation::Carve(o), Self::Feed) => {
                o.feed = value;
            }
            (Operation::Carve(o), Self::PlungeFeed) => {
                o.plunge_feed = value;
            }
            (Operation::Carve(o), Self::PlungeA) => {
                let (_, b) = plunge_params(o.plunge);
                o.plunge = set_plunge_params(o.plunge, value, b);
            }
            (Operation::Carve(o), Self::PlungeB) => {
                let (a, _) = plunge_params(o.plunge);
                o.plunge = set_plunge_params(o.plunge, a, value);
            }
            (Operation::Carve(o), Self::ClearStepdown) => {
                if let Some(clear) = &mut o.clear {
                    clear.params.stepdown = value;
                }
            }
            (Operation::Carve(o), Self::ClearOverlap) => {
                if let Some(clear) = &mut o.clear {
                    clear.params.overlap = value / 100.;
                }
            }
            (Operation::Carve(o), Self::ClearOffset) => {
                if let Some(clear) = &mut o.clear {
                    clear.params.offset = value;
                }
            }
            (Operation::Carve(o), Self::ClearEngagement) => {
                if let Some(clear) = &mut o.clear {
                    clear.params.clearing.engagement = value;
                }
            }
            (Operation::Carve(o), Self::ClearFeed) => {
                if let Some(clear) = &mut o.clear {
                    clear.params.feed = value;
                }
            }
            (Operation::Carve(o), Self::ClearPlungeFeed) => {
                if let Some(clear) = &mut o.clear {
                    clear.params.plunge_feed = value;
                }
            }
            (Operation::Carve(o), Self::ClearLeadOverlap) => {
                if let Some(clear) = &mut o.clear {
                    clear.params.lead_overlap = value;
                }
            }
            (Operation::Carve(o), Self::ClearLeadInSize) => {
                if let Some(clear) = &mut o.clear {
                    clear.params.lead_in = set_lead_size(clear.params.lead_in, value);
                }
            }
            (Operation::Carve(o), Self::ClearLeadOutSize) => {
                if let Some(clear) = &mut o.clear {
                    clear.params.lead_out = set_lead_size(clear.params.lead_out, value);
                }
            }
            (Operation::Carve(o), Self::ClearPlungeA) => {
                if let Some(clear) = &mut o.clear {
                    let (_, b) = plunge_params(clear.params.plunge);
                    clear.params.plunge = set_plunge_params(clear.params.plunge, value, b);
                }
            }
            (Operation::Carve(o), Self::ClearPlungeB) => {
                if let Some(clear) = &mut o.clear {
                    let (a, _) = plunge_params(clear.params.plunge);
                    clear.params.plunge = set_plunge_params(clear.params.plunge, a, value);
                }
            }
            (Operation::Profile(o), Self::Engagement) => {
                o.clearing.engagement = value;
            }
            (Operation::Profile(o), Self::PlungeA) => {
                let (_, b) = plunge_params(o.plunge);
                o.plunge = set_plunge_params(o.plunge, value, b);
            }
            (Operation::Profile(o), Self::PlungeB) => {
                let (a, _) = plunge_params(o.plunge);
                o.plunge = set_plunge_params(o.plunge, a, value);
            }
            (Operation::Pocket(o), Self::Depth) => {
                o.depth = value;
            }
            (Operation::Pocket(o), Self::Stepdown) => {
                o.clear.stepdown = value;
            }
            (Operation::Pocket(o), Self::Overlap) => {
                o.clear.overlap = value / 100.;
            }
            (Operation::Pocket(o), Self::PocketOverlap) => {
                o.clear.pocket_overlap = Some(value);
            }
            (Operation::Pocket(o), Self::ProfileOffset) => {
                o.clear.offset = value;
            }
            (Operation::Pocket(o), Self::LeadInSize) => {
                o.clear.lead_in = set_lead_size(o.clear.lead_in, value);
            }
            (Operation::Pocket(o), Self::LeadOutSize) => {
                o.clear.lead_out = set_lead_size(o.clear.lead_out, value);
            }
            (Operation::Pocket(o), Self::Feed) => {
                o.clear.feed = value;
            }
            (Operation::Pocket(o), Self::PlungeFeed) => {
                o.clear.plunge_feed = value;
            }
            (Operation::Pocket(o), Self::LeadOverlap) => {
                o.clear.lead_overlap = value;
            }
            (Operation::Pocket(o), Self::Engagement) => {
                o.clear.clearing.engagement = value;
            }
            (Operation::Pocket(o), Self::PlungeA) => {
                let (_, b) = plunge_params(o.clear.plunge);
                o.clear.plunge = set_plunge_params(o.clear.plunge, value, b);
            }
            (Operation::Pocket(o), Self::PlungeB) => {
                let (a, _) = plunge_params(o.clear.plunge);
                o.clear.plunge = set_plunge_params(o.clear.plunge, a, value);
            }
            (Operation::Face(o), Self::FaceStartOffset) => {
                o.start_offset = value;
            }
            (Operation::Face(o), Self::Depth) => {
                o.depth = value;
            }
            (Operation::Face(o), Self::Stepdown) => {
                o.stepdown = value;
            }
            (Operation::Face(o), Self::Overlap) => {
                o.overlap = value / 100.;
            }
            (Operation::Face(o), Self::FaceOverlap) => {
                o.absolute_overlap = Some(value);
            }
            (Operation::Face(o), Self::FaceOvershoot) => {
                o.overshoot = value;
            }
            (Operation::Face(o), Self::Feed) => {
                o.feed = value;
            }
            (Operation::Face(o), Self::PlungeFeed) => {
                o.plunge_feed = value;
            }
            (Operation::Drill(o), Self::Depth) => {
                o.depth = value;
            }
            (Operation::Drill(o), Self::DrillStartOffset) => {
                o.start_offset = value;
            }
            (Operation::Drill(o), Self::Peck) => {
                o.peck = (value > 0.).then_some(value);
            }
            (Operation::Drill(o), Self::Dwell) => {
                o.dwell = (value > 0.).then_some(value);
            }
            (Operation::Drill(o), Self::Feed) => {
                o.feed = value;
            }
            (Operation::Engrave(o), Self::Depth) => {
                o.depth = value;
            }
            (Operation::Engrave(o), Self::Stepdown) => {
                o.stepdown = value;
            }
            (Operation::Engrave(o), Self::Feed) => {
                o.feed = value;
            }
            (Operation::Engrave(o), Self::PlungeFeed) => {
                o.plunge_feed = value;
            }
            (Operation::Chamfer(o), Self::ChamferTop) => {
                o.top = value;
            }
            (Operation::Chamfer(o), Self::ChamferWidth) => {
                o.width = value;
            }
            (Operation::Chamfer(o), Self::ChamferDepth) => {
                o.depth = value;
            }
            (Operation::Chamfer(o), Self::ChamferTipOffset) => {
                o.tip_offset = value;
            }
            (Operation::Chamfer(o), Self::ChamferStep) => {
                o.step = value;
            }
            (Operation::Chamfer(o), Self::Stepdown) => {
                o.stepdown = value;
            }
            (Operation::Chamfer(o), Self::Feed) => {
                o.feed = value;
            }
            (Operation::Chamfer(o), Self::PlungeFeed) => {
                o.plunge_feed = value;
            }
            (Operation::Chamfer(o), Self::LeadInSize) => {
                o.lead_in = set_lead_size(o.lead_in, value);
            }
            (Operation::Chamfer(o), Self::LeadOutSize) => {
                o.lead_out = set_lead_size(o.lead_out, value);
            }
            (Operation::Chamfer(o), Self::LeadOverlap) => {
                o.lead_overlap = value;
            }
            (Operation::Bore(o), Self::BoreDiameter) => {
                o.diameter = value;
            }
            (Operation::Bore(o), Self::BoreTaper) => {
                o.taper_angle = value;
            }
            (Operation::Bore(o), Self::BorePitch) => {
                o.pitch = value;
            }
            (Operation::Bore(o), Self::BoreAngle) => {
                if o.ramp_angle.is_some() {
                    o.ramp_angle = Some(value);
                }
            }
            (Operation::Bore(o), Self::BoreStepovers) => {
                o.stepovers = value as u32;
            }
            (Operation::Bore(o), Self::BoreStepover) => {
                o.stepover = value;
            }
            (Operation::Bore(o), Self::BoreFinishing) => {
                o.finishing_stepover = value;
            }
            (Operation::Bore(o), Self::BoreRadialStock) => {
                o.radial_stock = value;
            }
            (Operation::Bore(o), Self::BoreAxialStock) => {
                o.axial_stock = value;
            }
            (Operation::Bore(o), Self::BoreTolerance) => {
                o.tolerance = value;
            }
            (Operation::Bore(o), Self::BoreSafeDistance) => {
                o.safe_distance = value;
            }
            (Operation::Bore(o), Self::BoreLeadRadius) => {
                o.lead_radius = value;
            }
            (Operation::Bore(o), Self::Feed) => {
                o.feed = value;
            }
            (Operation::Bore(o), Self::PlungeFeed) => {
                o.plunge_feed = value;
            }
            (Operation::Thread(o), Self::MajorDia) => {
                o.major_dia = value;
            }
            (Operation::Thread(o), Self::Pitch) => {
                o.pitch = value;
            }
            (Operation::Thread(o), Self::ThreadTop) => {
                o.z_top = value;
            }
            (Operation::Thread(o), Self::ThreadBottom) => {
                o.z_bottom = value;
            }
            (Operation::Thread(o), Self::ThreadPasses) => {
                o.passes = value as u32;
            }
            (Operation::Thread(o), Self::ThreadSpringPasses) => {
                o.spring_passes = value as u32;
            }
            (Operation::Thread(o), Self::ThreadDrillClearance) => {
                o.drill_clearance = value;
            }
            (Operation::Thread(o), Self::ThreadBlindAllowance) => {
                o.blind_allowance = value;
            }
            (Operation::Thread(o), Self::Feed) => {
                o.feed = value;
            }
            (Operation::Thread(o), Self::PlungeFeed) => {
                o.plunge_feed = value;
            }
            (Operation::Parallel(o), Self::ParallelStepover) => {
                o.params.stepover = value;
            }
            (Operation::Parallel(o), Self::ParallelAngle) => {
                o.params.angle = value;
            }
            (Operation::Parallel(o), Self::ScallopSafeDistance) => {
                o.params.scallop_safe_distance = value;
            }
            (Operation::Parallel(o), Self::ScallopSmoothingTolerance) => {
                o.params.scallop_smoothing_tolerance = value;
            }
            (Operation::Parallel(o), Self::AdaptiveTolerance) => {
                o.params.tolerance = value;
            }
            (Operation::Parallel(o), Self::AdaptiveOffset) => {
                o.params.additional_offset = value;
            }
            (Operation::Parallel(o), Self::Feed) => {
                o.params.feed = value;
            }
            (Operation::Parallel(o), Self::PlungeFeed) => {
                o.params.plunge_feed = value;
            }
            (Operation::Adaptive(o), Self::AdaptiveTolerance) => {
                o.params.tolerance = value;
            }
            (Operation::Adaptive(o), Self::AdaptiveOptimalLoad) => {
                o.params.optimal_load = value;
            }
            (Operation::Adaptive(o), Self::Stepdown) => {
                o.params.stepdown = value;
            }
            (Operation::Adaptive(o), Self::AdaptiveFineStepdown) => {
                o.params.fine_stepdown = value;
            }
            (Operation::Adaptive(o), Self::AdaptiveOffset) => {
                o.params.additional_offset = value;
            }
            (Operation::Adaptive(o), Self::Feed) => {
                o.params.feed = value;
            }
            (Operation::Adaptive(o), Self::PlungeFeed) => {
                o.params.plunge_feed = value;
            }
            (Operation::Profile(o), Self::ProfileFinishCount) => {
                o.passes.finish_count = value as u32;
            }
            (Operation::Profile(o), Self::FinishingStepover) => {
                o.passes.finish_stepover = value;
            }
            (Operation::Profile(o), Self::ProfileFinishFeed) => {
                o.passes.finish_feed = value;
            }
            (Operation::Profile(o), Self::ProfileSpringPasses) => {
                o.passes.spring_passes = value as u32;
            }
            (Operation::Profile(o), Self::ProfileFloorLeave) => {
                o.passes.floor_allowance = value;
            }
            (Operation::Profile(o), Self::ProfileFinishDepths) => {
                o.passes.finishing_depths = value as u32;
            }
            (Operation::Profile(o), Self::ProfileFinishDepthStep) => {
                o.passes.finishing_stepdown = value;
            }
            (Operation::Profile(o), Self::ProfileTangentialExtensionDistance) => {
                o.passes.tangential_extension_distance = value;
            }
            (Operation::Profile(o), Self::ProfileStartExtension) => {
                o.passes.start_extension = value;
            }
            (Operation::Profile(o), Self::ProfileEndExtension) => {
                o.passes.end_extension = value;
            }
            (Operation::Profile(o), Self::ProfileTolerance) => {
                o.passes.tolerance = value;
            }
            (Operation::Profile(o), Self::ProfileTaper) => {
                o.passes.taper_angle = value;
            }
            (Operation::Profile(o), Self::ProfileLeadEndDistance) => {
                o.passes.lead_end_distance = value;
            }
            (Operation::Profile(o), Self::ProfileRampRpm) => {
                o.passes.ramp_rpm = value;
            }
            (Operation::Profile(o), Self::ProfileTabCount) => {
                o.tabs.count = Some(value as u32);
            }
            (Operation::Pocket(o), Self::PocketFinishCount) => {
                o.finishing.count = value as u32;
            }
            (Operation::Pocket(o), Self::PocketFinishStepover) => {
                o.finishing.stepover = value;
            }
            (Operation::Pocket(o), Self::PocketFinishFeed) => {
                o.finishing.feed = value;
            }
            (Operation::Pocket(o), Self::PocketFinishOverlap) => {
                o.finishing.overlap = value;
            }
            (Operation::Profile(o), Self::ProfileLeadInSweep) => {
                if let Lead::Arc { radius } | Lead::ArcSweep { radius, .. } = o.lead_in {
                    o.lead_in = Lead::ArcSweep {
                        radius,
                        angle_deg: value,
                    };
                }
            }
            (Operation::Chamfer(o), Self::ProfileLeadInSweep) => {
                if let Lead::Arc { radius } | Lead::ArcSweep { radius, .. } = o.lead_in {
                    o.lead_in = Lead::ArcSweep {
                        radius,
                        angle_deg: value,
                    };
                }
            }
            (Operation::Profile(o), Self::ProfileLeadOutSweep) => {
                if let Lead::Arc { radius } | Lead::ArcSweep { radius, .. } = o.lead_out {
                    o.lead_out = Lead::ArcSweep {
                        radius,
                        angle_deg: value,
                    };
                }
            }
            (Operation::Chamfer(o), Self::ProfileLeadOutSweep) => {
                if let Lead::Arc { radius } | Lead::ArcSweep { radius, .. } = o.lead_out {
                    o.lead_out = Lead::ArcSweep {
                        radius,
                        angle_deg: value,
                    };
                }
            }
            (Operation::Profile(o), Self::StartX) => {
                if let Some(point) = &mut o.start {
                    point[0] = value;
                }
            }
            (Operation::Profile(o), Self::StartY) => {
                if let Some(point) = &mut o.start {
                    point[1] = value;
                }
            }
            (Operation::Pocket(o), Self::StartX) => {
                if let Some(point) = &mut o.start {
                    point[0] = value;
                }
            }
            (Operation::Pocket(o), Self::StartY) => {
                if let Some(point) = &mut o.start {
                    point[1] = value;
                }
            }
            (Operation::Chamfer(o), Self::StartX) => {
                if let Some(point) = &mut o.start {
                    point[0] = value;
                }
            }
            (Operation::Chamfer(o), Self::StartY) => {
                if let Some(point) = &mut o.start {
                    point[1] = value;
                }
            }
            (Operation::Engrave(o), Self::StartX) => {
                if let Some(point) = &mut o.start {
                    point[0] = value;
                }
            }
            (Operation::Engrave(o), Self::StartY) => {
                if let Some(point) = &mut o.start {
                    point[1] = value;
                }
            }
            (Operation::Carve(o), Self::StartX) => {
                if let Some(point) = &mut o.start {
                    point[0] = value;
                }
            }
            (Operation::Carve(o), Self::StartY) => {
                if let Some(point) = &mut o.start {
                    point[1] = value;
                }
            }
            (Operation::Adaptive(o), Self::StartX) => {
                if let Some(point) = &mut o.params.start {
                    point[0] = value;
                }
            }
            (Operation::Adaptive(o), Self::StartY) => {
                if let Some(point) = &mut o.params.start {
                    point[1] = value;
                }
            }
            (Operation::Parallel(o), Self::StartX) => {
                if let Some(point) = &mut o.params.scallop_start {
                    point[0] = value;
                }
            }
            (Operation::Parallel(o), Self::StartY) => {
                if let Some(point) = &mut o.params.scallop_start {
                    point[1] = value;
                }
            }
            _ => {}
        }
    }
    pub fn integer(self) -> bool {
        matches!(
            self,
            Self::BoreStepovers
                | Self::ContourRoughCount
                | Self::PocketFinishCount
                | Self::ProfileFinishCount
                | Self::ProfileFinishDepths
                | Self::ProfileSpringPasses
                | Self::ProfileTabCount
                | Self::ThreadPasses
                | Self::ThreadSpringPasses
        )
    }
    pub fn validate_value(self, op: &Operation, value: f64) -> Result<(), String> {
        if !value.is_finite() {
            return Err(format!("{} must be finite", self.name()));
        }
        if self.integer() && (value.fract() != 0. || value > u32::MAX as f64) {
            return Err(format!("{} must be a whole number in range", self.name()));
        }
        let invalid = match self {
            Self::CuttingSpeed | Self::FeedPerTooth => value <= 0.,
            Self::MaximumStepover
            | Self::ContourRoughSmoothing
            | Self::Depth
            | Self::ChamferWidth
            | Self::Feed
            | Self::PlungeFeed
            | Self::ParallelStepover
            | Self::AdaptiveTolerance
            | Self::AdaptiveOptimalLoad
            | Self::AdaptiveFineStepdown
            | Self::ScallopSmoothingTolerance
            | Self::MajorDia
            | Self::Pitch
            | Self::BoreDiameter
            | Self::BorePitch
            | Self::BoreTolerance
            | Self::BoreSafeDistance
            | Self::TabWidth
            | Self::TabHeight
            | Self::TabSpacing
            | Self::FinishingStepover
            | Self::ProfileFinishDepthStep
            | Self::ProfileTolerance
            | Self::PocketFinishStepover => value <= 0.,
            Self::ContourRoughCount | Self::ProfileFinishCount | Self::PocketFinishCount => {
                value < 1. || value > 1000.
            }
            Self::ProfileTabCount => value < 1. || value > 10000.,
            Self::ProfileSpringPasses | Self::ProfileFinishDepths => value < 0. || value > 1000.,
            Self::BoreStepovers => !(0. ..=10000.).contains(&value),
            Self::ThreadPasses => value < 1.,
            Self::BoreAngle => value <= 0. || value >= 90.,
            Self::ProfileLeadInSweep | Self::ProfileLeadOutSweep => value <= 0. || value > 170.,
            Self::ProfileTaper => value <= -80. || value > 0.,
            Self::Overlap | Self::ClearOverlap => !(0. ..=99.).contains(&value),
            Self::Engagement if matches!(op, Operation::Pocket(p) if p.clear.pocket_adaptive_enabled()) => {
                value <= 0.
            }
            Self::Stepdown
                if matches!(op, Operation::Face(f) if !f.multiple_depths.enabled)
                    || matches!(op, Operation::Profile(p) if !p.passes.multiple_depths) =>
            {
                false
            }
            Self::Stepdown
                if matches!(
                    op,
                    Operation::Profile(_)
                        | Operation::Face(_)
                        | Operation::Adaptive(_)
                        | Operation::Parallel(_)
                ) =>
            {
                value <= 0.
            }
            Self::OperationTop
            | Self::StartX
            | Self::StartY
            | Self::DrillStartOffset
            | Self::ThreadTop
            | Self::ThreadBottom
            | Self::ChamferTop
            | Self::FaceOvershoot
            | Self::ParallelAngle
            | Self::AdaptiveOffset
            | Self::ProfileFloorLeave
            | Self::BoreTaper
            | Self::BoreRadialStock
            | Self::BoreAxialStock => false,
            Self::ProfileOffset if matches!(op, Operation::Profile(_)) => false,
            Self::PlungeA | Self::ClearPlungeA => {
                let plunge = match op {
                    Operation::Profile(p) => Some(p.plunge),
                    Operation::Pocket(p) => Some(p.clear.plunge),
                    Operation::Carve(p) if self == Self::ClearPlungeA => {
                        p.clear.map(|c| c.params.plunge)
                    }
                    Operation::Carve(p) => Some(p.plunge),
                    _ => None,
                };
                match plunge {
                    Some(Plunge::Ramp { .. } | Plunge::ZigZag { .. }) => {
                        value <= 0. || value >= 90.
                    }
                    Some(Plunge::Helix { .. }) => value <= 0.,
                    _ => false,
                }
            }
            Self::PlungeB | Self::ClearPlungeB => value <= 0.,
            _ => value < 0.,
        };
        if invalid {
            Err(format!("{} is outside its allowed range", self.name()))
        } else {
            Ok(())
        }
    }
}
impl Operation {
    pub fn parameter_expressions(&self) -> &BTreeMap<OperationParameter, String> {
        match self {
            Self::Profile(o) => &o.parameter_expressions,
            Self::Drill(o) => &o.parameter_expressions,
            Self::Pocket(o) => &o.parameter_expressions,
            Self::Adaptive(o) => &o.parameter_expressions,
            Self::Parallel(o) => &o.parameter_expressions,
            Self::Face(o) => &o.parameter_expressions,
            Self::Chamfer(o) => &o.parameter_expressions,
            Self::Thread(o) => &o.parameter_expressions,
            Self::Bore(o) => &o.parameter_expressions,
            Self::Engrave(o) => &o.parameter_expressions,
            Self::Carve(o) => &o.parameter_expressions,
        }
    }
    pub fn parameter_expressions_mut(&mut self) -> &mut BTreeMap<OperationParameter, String> {
        match self {
            Self::Profile(o) => &mut o.parameter_expressions,
            Self::Drill(o) => &mut o.parameter_expressions,
            Self::Pocket(o) => &mut o.parameter_expressions,
            Self::Adaptive(o) => &mut o.parameter_expressions,
            Self::Parallel(o) => &mut o.parameter_expressions,
            Self::Face(o) => &mut o.parameter_expressions,
            Self::Chamfer(o) => &mut o.parameter_expressions,
            Self::Thread(o) => &mut o.parameter_expressions,
            Self::Bore(o) => &mut o.parameter_expressions,
            Self::Engrave(o) => &mut o.parameter_expressions,
            Self::Carve(o) => &mut o.parameter_expressions,
        }
    }
    pub fn has_parameter_expressions(&self) -> bool {
        !self.parameter_expressions().is_empty()
            || match self {
                Operation::Profile(p) => {
                    p.passes.finish_stepover_expression.is_some()
                        || p.passes.contour_roughing.stepover_expression.is_some()
                }
                Operation::Chamfer(p) => p.contour_roughing.stepover_expression.is_some(),
                _ => false,
            }
    }
    pub fn evaluate_parameter_sources(
        &self,
        tool: Option<&Tool>,
        overrides: &BTreeMap<OperationParameter, String>,
    ) -> Result<BTreeMap<OperationParameter, f64>, String> {
        self.evaluate_parameter_source_results(tool, overrides)
            .into_iter()
            .map(|(parameter, value)| value.map(|value| (parameter, value)))
            .collect()
    }
    /// Editor diagnostics are isolated to a parameter and its dependency chain.
    pub fn evaluate_parameter_source_results(
        &self,
        tool: Option<&Tool>,
        overrides: &BTreeMap<OperationParameter, String>,
    ) -> BTreeMap<OperationParameter, Result<f64, String>> {
        let mut source_errors: BTreeMap<String, String> = BTreeMap::new();
        for (parameter, source) in overrides {
            if !source.trim_start().starts_with('=') && source.trim().parse::<f64>().is_err() {
                source_errors.insert(
                    parameter.name().into(),
                    "Start a formula with =, or enter a number".into(),
                );
            }
        }
        let mut sources: BTreeMap<String, String> = BTreeMap::new();
        for parameter in OperationParameter::ALL {
            if let Some(value) = parameter.value(self) {
                let source = overrides
                    .get(&parameter)
                    .cloned()
                    .or_else(|| parameter.expression(self).map(str::to_owned))
                    .unwrap_or_else(|| value.to_string());
                sources.insert(parameter.name().into(), source);
            }
        }
        let mut variables = BTreeMap::new();
        if let Some(tool) = tool {
            let corner = match tool.kind {
                ToolKind::BullNose { corner_radius }
                | ToolKind::RadiusMill { corner_radius, .. } => corner_radius,
                ToolKind::BallMill => tool.radius(),
                _ => 0.,
            };
            variables.insert("tool_diameter".into(), tool.diameter);
            variables.insert("tool_cornerRadius".into(), corner);
            variables.insert("tool_length".into(), tool.length);
            variables.insert("tool_fluteLength".into(), tool.flute_len());
            variables.insert("tool_flutes".into(), tool.flutes as f64);
        }
        let bound = |parameter: OperationParameter| {
            overrides.get(&parameter).map_or_else(
                || parameter.expression(self).is_some(),
                |source| source.trim_start().starts_with('='),
            )
        };
        let speed_bound = bound(OperationParameter::CuttingSpeed);
        let tooth_bound = bound(OperationParameter::FeedPerTooth);
        if speed_bound && bound(OperationParameter::SpindleRpm) {
            for name in ["spindleRpm", "cuttingSpeed"] {
                source_errors.insert(
                    name.into(),
                    "Use a formula for spindle RPM or cutting speed, not both".into(),
                );
            }
        }
        if tooth_bound && bound(OperationParameter::Feed) {
            for name in ["feed", "feedPerTooth"] {
                source_errors.insert(
                    name.into(),
                    "Use a formula for feed or feed per tooth, not both".into(),
                );
            }
        }
        if let Some(tool) = tool {
            if !speed_bound {
                sources.insert(
                    "cuttingSpeed".into(),
                    format!(
                        "spindleRpm * tool_diameter * {} / 1000",
                        std::f64::consts::PI
                    ),
                );
            } else {
                sources.insert(
                    "spindleRpm".into(),
                    format!(
                        "cuttingSpeed * 1000 / ({} * tool_diameter)",
                        std::f64::consts::PI
                    ),
                );
            }
            if !tooth_bound {
                let has_rpm = self.spindle_rpm() > 0.
                    || speed_bound
                    || bound(OperationParameter::SpindleRpm)
                    || overrides
                        .get(&OperationParameter::SpindleRpm)
                        .is_some_and(|s| s.parse::<f64>().is_ok_and(|v| v > 0.));
                sources.insert(
                    "feedPerTooth".into(),
                    if has_rpm && tool.flutes > 0 {
                        "feed / (tool_flutes * spindleRpm)".into()
                    } else {
                        "0".into()
                    },
                );
            } else {
                if tool.flutes == 0 {
                    source_errors.insert(
                        "feedPerTooth".into(),
                        "Feed-per-tooth formulas require a cutter flute count".into(),
                    );
                }
                sources.insert(
                    "feed".into(),
                    "feedPerTooth * tool_flutes * spindleRpm".into(),
                );
            }
        } else if speed_bound || tooth_bound {
            for (parameter, bound) in [
                (OperationParameter::CuttingSpeed, speed_bound),
                (OperationParameter::FeedPerTooth, tooth_bound),
            ] {
                if bound {
                    source_errors.insert(
                        parameter.name().into(),
                        "Choose a cutter before entering cutting-data formulas".into(),
                    );
                }
            }
        }
        let explicit_rpm = self.spindle_rpm() > 0.
            || speed_bound
            || bound(OperationParameter::SpindleRpm)
            || overrides
                .get(&OperationParameter::SpindleRpm)
                .is_some_and(|s| s.parse::<f64>().is_ok_and(|v| v > 0.));
        if !explicit_rpm {
            for (name, source) in &sources {
                if name != "cuttingSpeed"
                    && name != "feedPerTooth"
                    && (source.contains("cuttingSpeed") || source.contains("feedPerTooth"))
                {
                    source_errors.insert(
                        name.clone(),
                        "Set spindle RPM before using cuttingSpeed or feedPerTooth in a formula"
                            .into(),
                    );
                }
            }
        }
        let tooth_referenced = sources
            .iter()
            .any(|(name, source)| name != "feedPerTooth" && source.contains("feedPerTooth"));
        if !tooth_bound && !tooth_referenced {
            sources.remove("feedPerTooth");
        }
        let mut values = crate::parameter_expression::evaluate_parameter_results_with(
            &sources,
            &variables,
            &source_errors,
            |name, value| {
                if let Some(parameter) = OperationParameter::ALL
                    .into_iter()
                    .find(|p| p.name() == name)
                {
                    if bound(parameter) {
                        parameter.validate_value(self, value)?;
                    }
                }
                Ok(())
            },
        );
        if !values.contains_key("feedPerTooth") {
            let tooth = (|| {
                let rpm = values.get("spindleRpm").cloned().unwrap_or(Ok(0.))?;
                let feed = values.get("feed").cloned().unwrap_or(Ok(0.))?;
                let flutes = tool.map_or(0, |t| t.flutes);
                Ok(if rpm > 0. && flutes > 0 {
                    feed / (flutes as f64 * rpm)
                } else {
                    0.
                })
            })();
            values.insert("feedPerTooth".into(), tooth);
        }
        OperationParameter::ALL
            .into_iter()
            .filter_map(|parameter| {
                values
                    .get(parameter.name())
                    .cloned()
                    .map(|value| (parameter, value))
            })
            .collect()
    }
    /// Worker-only: freeze resolved numbers before height/stock adapters modify
    /// them. Saved operations retain their bindings; derived machining passes do not.
    pub fn freeze_parameter_expressions(&mut self) {
        self.parameter_expressions_mut().clear();
        match self {
            Operation::Profile(p) => {
                p.passes.finish_stepover_expression = None;
                p.passes.contour_roughing.stepover_expression = None;
            }
            Operation::Chamfer(p) => p.contour_roughing.stepover_expression = None,
            _ => {}
        }
    }
    pub fn resolve_parameter_expressions(&mut self, tools: &[Tool]) -> Result<(), String> {
        if !self.has_parameter_expressions() {
            return Ok(());
        }
        let values = self.evaluate_parameter_sources(
            tools.iter().find(|t| t.number == self.tool()),
            &BTreeMap::new(),
        )?;
        if OperationParameter::CuttingSpeed.expression(self).is_some() {
            self.set_spindle_rpm(values[&OperationParameter::SpindleRpm]);
        }
        if OperationParameter::FeedPerTooth.expression(self).is_some() {
            OperationParameter::Feed.set_value(self, values[&OperationParameter::Feed]);
        }
        // Apply only bindings, so defaults, absent leads, and sentinel modes keep their meaning.
        for (parameter, value) in values {
            if parameter.expression(self).is_some() {
                parameter.set_value(self, value);
            }
        }
        Ok(())
    }
}
fn lead_size(lead: Lead) -> f64 {
    match lead {
        Lead::None => 0.0,
        Lead::Linear { length } => length,
        Lead::Arc { radius } | Lead::ArcSweep { radius, .. } => radius,
    }
}

/// Plunge parameters as `(a, b)` for the inspector: `(angle, —)` for ramp,
/// `(radius, pitch)` for helix, `(angle, length)` for zig-zag, `(0, 0)` straight.
fn plunge_params(plunge: Plunge) -> (f64, f64) {
    match plunge {
        Plunge::Straight => (0.0, 0.0),
        Plunge::Ramp { angle_deg } => (angle_deg, 0.0),
        Plunge::Helix { radius, pitch } => (radius, pitch),
        Plunge::ZigZag { length, angle_deg } => (angle_deg, length),
    }
}

/// Set a lead's size (length/radius), keeping its kind.
fn set_lead_size(lead: Lead, size: f64) -> Lead {
    match lead {
        Lead::None => Lead::None,
        Lead::Linear { .. } => Lead::Linear { length: size },
        Lead::Arc { .. } => Lead::Arc { radius: size },
        Lead::ArcSweep { angle_deg, .. } => Lead::ArcSweep {
            radius: size,
            angle_deg,
        },
    }
}

/// Set a plunge's parameters from the inspector `(a, b)`, keeping its kind.
fn set_plunge_params(plunge: Plunge, a: f64, b: f64) -> Plunge {
    match plunge {
        Plunge::Straight => Plunge::Straight,
        Plunge::Ramp { .. } => Plunge::Ramp { angle_deg: a },
        Plunge::Helix { .. } => Plunge::Helix {
            radius: a,
            pitch: b,
        },
        Plunge::ZigZag { .. } => Plunge::ZigZag {
            angle_deg: a,
            length: b,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn profile() -> Operation {
        Operation::Profile(crate::ProfileOp {
            parameter_expressions: Default::default(),
            wrap: None,
            passes: Default::default(),
            selections: None,
            closed: true,
            additional_chains: Vec::new(),
            tabs: Default::default(),
            spindle_rpm: 0.0,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            clearing: crate::Clearing::default(),
            id: 1,
            tool: 1,
            chain: cam_geo::Contour::new(vec![
                cam_geo::Point::new(0.0, 0.0),
                cam_geo::Point::new(10.0, 0.0),
                cam_geo::Point::new(10.0, 10.0),
                cam_geo::Point::new(0.0, 10.0),
            ]),
            side: crate::Side::Outside,
            comp: crate::Comp::Computed,
            offset: 0.0,
            depth: 4.0,
            stepdown: 2.0,
            stepover: 0.0,
            feed: 300.0,
            plunge_feed: 100.0,
            end: None,
            start: None,
            lead_in: crate::Lead::None,
            lead_out: crate::Lead::None,
            lead_overlap: 0.0,
            plunge: crate::Plunge::Straight,
        })
    }
    #[test]
    fn screenshot_formulas_follow_tool_and_numeric_overrides() {
        let mut op = profile();
        let mut tool = Tool {
            number: op.tool(),
            diameter: 6.,
            ..Default::default()
        };
        for p in [
            OperationParameter::FinishingStepover,
            OperationParameter::MaximumStepover,
        ] {
            p.set_source(&mut op, p.default_expression());
        }
        op.resolve_parameter_expressions(&[tool.clone()]).unwrap();
        assert!((OperationParameter::MaximumStepover.value(&op).unwrap() - 5.7).abs() < 1e-12);
        assert!((OperationParameter::FinishingStepover.value(&op).unwrap() - 0.6).abs() < 1e-12);
        // A numeric override is an independent source in the dependency graph.
        OperationParameter::MaximumStepover.set_source(&mut op, "0.3");
        if let Operation::Profile(p) = &mut op {
            p.passes.contour_roughing.stepover = 0.3;
        }
        tool.diameter = 12.;
        op.resolve_parameter_expressions(&[tool]).unwrap();
        assert_eq!(OperationParameter::FinishingStepover.value(&op), Some(0.3));
        assert_eq!(OperationParameter::MaximumStepover.expression(&op), None);
        let saved = serde_json::to_string(&op).unwrap();
        assert_eq!(serde_json::from_str::<Operation>(&saved).unwrap(), op);
    }
    #[test]
    fn invalid_resolution_is_atomic_and_old_files_keep_literals() {
        let mut op = profile();
        let before = op.clone();
        OperationParameter::MaximumStepover.set_source(&mut op, "finishingStepover");
        OperationParameter::FinishingStepover.set_source(&mut op, "maximumStepover");
        let invalid = op.clone();
        assert!(op.resolve_parameter_expressions(&[]).is_err());
        assert_eq!(op, invalid);
        let json = serde_json::to_value(&before).unwrap();
        assert!(json["Profile"]["passes"]
            .get("finish_stepover_expression")
            .is_none());
        assert_eq!(serde_json::from_value::<Operation>(json).unwrap(), before);
    }
}
