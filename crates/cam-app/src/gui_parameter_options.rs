//! Checkboxes and dropdowns that store an operation parameter.
use super::*;
use cam_model::{
    AdaptiveContainment, BoreOrder, Comp, Cooling, HeightPlaneKind as Plane,
    HeightReference as Reference, OperationHeightSettings, ParallelTransition, ProfileCorner,
    RotaryAxis, RotaryIndex, RotaryWrap, ScallopOrder, ScallopRetraction, SpiralBoundary,
    SpiralCutDirection, SpiralMilling, SpiralMode, SpiralOrder, TabShape,
};
use operation_tabs::Compensation;
use profile_tabs::Positioning;
use rotary_view::{Action as RotaryAction, IndexMode, MachiningType};
use serde::{de::DeserializeOwned, Serialize};
use thread_parameters::{ThreadFamily, ThreadMatchMode, ThreadMessage, ThreadParameters};

#[derive(Debug)]
pub(super) struct OptionSpec {
    pub name: &'static str,
    pub read: fn(&Operation) -> Option<String>,
    pub change: fn(&str) -> Option<Message>,
}

#[derive(Clone, Debug)]
pub(super) struct OptionMenu {
    pub spec: &'static OptionSpec,
    pub source: String,
    pub display: String,
    pub available: Vec<String>,
}

impl OptionMenu {
    pub(super) fn allows(&self, source: &str) -> bool {
        self.available.iter().any(|value| value == source)
    }

    pub(super) fn seed_system_context(&self, operation: &Operation, template: &mut Operation) {
        match (operation, template) {
            (Operation::Parallel(current), Operation::Parallel(system))
                if self.spec.name == RETRACTION.spec.name =>
            {
                // Parallel and Scallop ship with different retraction policies.
                system.params.scallop = current.params.scallop;
            }
            (Operation::Face(current), Operation::Face(system))
                if self.spec.name == FACE_DIRECTION.spec.name =>
            {
                system.direction = Axis::along_longest_edge(current.boundary.points());
            }
            _ => {}
        }
    }
}

#[derive(Clone, Copy)]
pub(super) struct Setting<T> {
    spec: &'static OptionSpec,
    change: fn(T) -> Message,
}

fn encode<T: Serialize>(value: &T) -> String {
    // These settings are booleans and fieldless enums.
    serde_json::to_string(value).expect("serialize operation option")
}

impl<T: Serialize> Setting<T> {
    pub(super) fn on_change(&self) -> fn(T) -> Message {
        self.change
    }

    pub(super) fn menu(
        self,
        selected: &T,
        display: String,
        options: &[T],
        widget: impl Into<Element<'static, Message>>,
    ) -> Element<'static, Message> {
        let open = Message::OpenParameterOptionMenu(self.selection(selected, display, options));
        row![
            mouse_area(widget).on_right_press(open.clone()),
            parameter_defaults::menu_button(open)
        ]
        .spacing(4)
        .align_y(Alignment::Center)
        .into()
    }

    fn selection(&self, selected: &T, display: String, options: &[T]) -> OptionMenu {
        OptionMenu {
            spec: self.spec,
            source: encode(selected),
            display,
            available: options.iter().map(encode).collect(),
        }
    }
}

impl Setting<bool> {
    pub(super) fn checkbox(self, checked: bool, size: u32) -> Element<'static, Message> {
        self.menu(
            &checked,
            if checked { "On" } else { "Off" }.into(),
            &[false, true],
            checkbox(checked).size(size).on_toggle(self.change),
        )
    }
}

impl<T> Setting<T>
where
    T: Serialize + ToString + Clone + PartialEq + 'static,
{
    pub(super) fn picker(
        self,
        selected: T,
        options: &[T],
        width: Length,
    ) -> Element<'static, Message> {
        let widget = pick_list(options.to_vec(), Some(selected.clone()), self.change)
            .text_size(12)
            .padding([7, 9])
            .style(inspector_picker)
            .width(width);
        self.menu(&selected, selected.to_string(), options, widget)
    }
}

pub(super) fn operation_picker<T>(
    label: &str,
    help: &'static str,
    show: bool,
    selected: T,
    options: &[T],
    setting: Setting<T>,
) -> Element<'static, Message>
where
    T: Serialize + ToString + Clone + PartialEq + 'static,
{
    profile_picker_row(
        label,
        help,
        show,
        setting.picker(selected, options, Length::Fixed(INSPECTOR_INPUT_W)),
    )
}

fn decode<T: DeserializeOwned>(source: &str, change: fn(T) -> Message) -> Option<Message> {
    serde_json::from_str(source).ok().map(change)
}

macro_rules! get {
    ($variant:ident($value:ident) => $result:expr) => {
        |operation| match operation {
            Operation::$variant($value) => Some($result),
            _ => None,
        }
    };
}

macro_rules! settings {
    ($($setting:ident: $type:ty = $name:literal, $read:expr, $change:expr;)*) => {
        $(
            pub(super) const $setting: Setting<$type> = Setting {
                spec: &OptionSpec {
                    name: $name,
                    read: |op| {
                        let read: fn(&Operation) -> Option<$type> = $read;
                        read(op).as_ref().map(encode::<$type>)
                    },
                    change: |source| decode::<$type>(source, $change),
                },
                change: $change,
            };
        )*

        const OPTIONS: &[&OptionSpec] = &[$($setting.spec),*];
    };
}

use pocket_parameters::{DepthToggle, Toggle as PocketToggle};
use profile_parameters::Toggle as ProfileToggle;

settings! {
    MACHINING_TYPE: MachiningType = "machiningType", |_| Some(MachiningType::ThreeAxis), |value| Message::Rotary(match value {
        MachiningType::ThreeAxis => RotaryAction::Enable(false),
        MachiningType::Indexed => RotaryAction::Indexed(true),
        MachiningType::Wrap => RotaryAction::Enable(true),
    });
    ROTARY_AXIS: RotaryAxis = "indexAxis", |_| Some(RotaryIndex::default().axis), |value| Message::Rotary(RotaryAction::IndexAxis(value));
    ROTARY_ANGLE_MODE: IndexMode = "indexAngleMode", |_| Some(IndexMode::Manual), |value| Message::Rotary(RotaryAction::IndexMode(value));
    ROTARY_FLIP: bool = "flipToolEngagement", |_| Some(RotaryIndex::default().flip), |value| Message::Rotary(RotaryAction::IndexFlip(value));
    ROTARY_UNWRAP: bool = "unwrapGeometry", |op| op.supports_rotary_wrap().then_some(RotaryWrap::default().unwrap_geometry), |value| Message::Rotary(RotaryAction::Unwrap(value));
    COMPENSATION: Compensation = "compensation", get!(Profile(p) => Compensation::of(p.comp)), |value: Compensation| Message::ProfileCompChanged(value.value());
    STOCK_CONTOURS: bool = "useStockContours", get!(Pocket(p) => p.stock_contours.enabled), |v| Message::OperationGeometry(GeometryMessage::StockContours(v));
    CONTAINMENT: AdaptiveContainment = "toolContainment", |op| match op {
        Operation::Adaptive(a) => Some(a.params.containment),
        Operation::Parallel(p) => Some(p.params.containment),
        Operation::Spiral(s) => Some(s.params.containment),
        _ => None,
    }, Message::AdaptiveContainmentChanged;
    STOCK_LEAVE: bool = "stockToLeave", |op| App::stock_to_leave_supported(op).then_some(false), Message::StockToLeaveToggled;
    CONTOUR_ROUGHING: bool = "contourRoughing", |op| match op {
        Operation::Profile(p) => Some(p.passes.contour_roughing.enabled),
        Operation::Chamfer(c) => Some(c.contour_roughing.enabled),
        _ => None,
    }, Message::ContourRoughingToggled;
    HOLDING_TABS: bool = "holdingTabs", get!(Profile(p) => p.tabs.enabled), Message::ProfileTabsEnabled;
    TAB_SHAPE: TabShape = "tabShape", get!(Profile(p) => p.tabs.shape), Message::ProfileTabShape;
    TAB_POSITIONING: Positioning = "tabPositioning", get!(Profile(p) => if p.tabs.manual {
        Positioning::Manual
    } else if p.tabs.count.is_some() {
        Positioning::ByCount
    } else {
        Positioning::ByDistance
    }), Message::ProfileTabPositioning;
    CORNER: ProfileCorner = "outerCorners", get!(Profile(p) => p.passes.outer_corner), Message::ProfileCorner;
    ADAPTIVE_POCKET: bool = "adaptiveClearing", get!(Pocket(p) => p.clear.pocket_adaptive_enabled()), Message::PocketAdaptiveToggled;
    CLEARING_CLIMB: bool = "climbClearing", |op| match op {
        Operation::Profile(p) => Some(p.clearing.climb),
        Operation::Pocket(p) => Some(p.clear.clearing.climb),
        _ => None,
    }, Message::ClearingClimbToggled;
    POCKET_STAY_DOWN: bool = "keepToolDown", get!(Pocket(p) => p.clear.keep_tool_down), Message::PocketKeepToolDownToggled;
    POCKET_FINISHING: bool = "finishing", get!(Pocket(p) => p.finishing.enabled), |v| Message::PocketFinishToggle(PocketToggle::Enabled, v);
    POCKET_FINISH_LEADS: bool = "finishingLeads", get!(Pocket(p) => p.finishing.lead_in_out_all), |v| Message::PocketFinishToggle(PocketToggle::AllLeads, v);
    POCKET_FINISH_REPEAT: bool = "repeatFinishingPass", get!(Pocket(p) => p.finishing.repeat_pass), |v| Message::PocketFinishToggle(PocketToggle::Repeat, v);
    POCKET_FINISH_FINAL: bool = "finishOnlyAtFinalDepth", get!(Pocket(p) => p.finishing.finish_only_final), |v| Message::PocketFinishToggle(PocketToggle::FinishOnlyFinal, v);
    MULTIPLE_DEPTHS: bool = "multipleDepths", |op| multiple_depths(op).map(|d| d.enabled), |v| Message::PocketDepthToggle(DepthToggle::Enabled, v);
    EVEN_DEPTHS: bool = "evenStepdowns", |op| multiple_depths(op).map(|d| d.even_stepdowns), |v| Message::PocketDepthToggle(DepthToggle::Even, v);
    DEPTH_ORDER: bool = "orderByDepth", get!(Pocket(p) => p.clear.multiple_depths.order_by_depth), |v| Message::PocketDepthToggle(DepthToggle::ByDepth, v);
    AREA_ORDER: bool = "orderByArea", get!(Pocket(p) => p.clear.multiple_depths.order_by_area), |v| Message::PocketDepthToggle(DepthToggle::ByArea, v);
    FACE_DIRECTION: Axis = "passDirection", get!(Face(f) => f.direction), Message::FaceDirectionChanged;
    CHAMFER_SIDE: Side = "side", get!(Chamfer(c) => c.side), Message::SideChanged;
    CHAMFER_GRADUAL: bool = "gradual", get!(Chamfer(c) => c.gradual), Message::ChamferGradualToggled;
    CHAMFER_ENTRY: bool = "leadInEnabled", get!(Chamfer(c) => c.lead_in != Lead::None && c.lead_in_enabled.unwrap_or(true)), Message::ChamferEntryEnabled;
    CHAMFER_EXIT: bool = "leadOutEnabled", get!(Chamfer(c) => c.lead_out_enabled.unwrap_or(true) && (c.lead_out != Lead::None || (c.same_lead_out && c.lead_in != Lead::None && c.lead_in_enabled.unwrap_or(true)))), Message::ChamferExitEnabled;
    CHAMFER_SAME_LEAD: bool = "sameLeadOut", get!(Chamfer(c) => c.same_lead_out), Message::ChamferSameLeadOut;
    LEAD_IN: LeadKind = "leadInType", |op| match op {
        Operation::Profile(p) => Some(LeadKind::of(p.lead_in)),
        Operation::Pocket(p) => Some(LeadKind::of(p.clear.lead_in)),
        Operation::Chamfer(c) => Some(LeadKind::of(c.lead_in)),
        _ => None,
    }, Message::LeadInKindChanged;
    LEAD_OUT: LeadKind = "leadOutType", |op| match op {
        Operation::Profile(p) => Some(LeadKind::of(p.lead_out)),
        Operation::Pocket(p) => Some(LeadKind::of(p.clear.lead_out)),
        Operation::Chamfer(c) => Some(LeadKind::of(c.lead_out)),
        _ => None,
    }, Message::LeadOutKindChanged;
    PLUNGE: PlungeKind = "plungeType", |op| App::op_plunge_styles(op).0.map(PlungeKind::of), Message::PlungeKindChanged;
    BORE_INTERNAL: bool = "internalHole", get!(Bore(b) => b.internal), |v| Message::BoreOption(BoreOption::Internal(v));
    BORE_CLIMB: bool = "climb", get!(Bore(b) => b.climb), |v| Message::BoreOption(BoreOption::Climb(v));
    BORE_ANGLE: bool = "useRampAngle", get!(Bore(b) => b.ramp_angle.is_some()), |v| Message::BoreOption(BoreOption::Angle(v));
    BORE_REPEAT: bool = "repeatFinalPass", get!(Bore(b) => b.repeat_pass), |v| Message::BoreOption(BoreOption::Repeat(v));
    BORE_CENTER: bool = "leadToCenter", get!(Bore(b) => b.lead_to_center), |v| Message::BoreOption(BoreOption::Center(v));
    BORE_REVERSE: bool = "reverseOrder", get!(Bore(b) => b.reverse_order), |v| Message::BoreOption(BoreOption::Reverse(v));
    BORE_ORDER: BoreOrder = "holeOrder", get!(Bore(b) => b.order), |v| Message::BoreOption(BoreOption::Order(v));
    THREAD_BORE: Bore = "threadBore", get!(Thread(t) => Bore::of(t.internal)), |v| Message::ThreadInternalChanged(v == Bore::Internal);
    THREAD_SIZING: ThreadMatchMode = "threadSizing", get!(Thread(_thread) => ThreadParameters::for_creation(OpKind::Thread).mode), |v| Message::ThreadParameters(ThreadMessage::Mode(v));
    THREAD_SERIES: ThreadFamily = "threadSeries", get!(Thread(_thread) => ThreadFamily::default()), |v| Message::ThreadParameters(ThreadMessage::Family(v));
    THREAD_HAND: Hand = "threadHand", get!(Thread(t) => t.hand), Message::ThreadHandChanged;
    THREAD_CUT: CutStyle = "cutDirection", get!(Thread(t) => CutStyle::of(t.climb)), |v| Message::ThreadClimbChanged(v == CutStyle::Climb);
    THREAD_GRADUAL: bool = "gradual", get!(Thread(t) => t.gradual), Message::ThreadGradualToggled;
    SPIRAL_MODE: SpiralMode = "spiralMode", get!(Spiral(s) => s.params.mode), Message::SpiralModeChanged;
    SPIRAL_ORDER: SpiralOrder = "spiralOrder", get!(Spiral(s) => s.params.order), Message::SpiralOrderChanged;
    SPIRAL_CUT: SpiralCutDirection = "cutDirection", get!(Spiral(s) => s.params.cut_direction), Message::SpiralCutDirectionChanged;
    SPIRAL_MILLING: SpiralMilling = "upDownMilling", get!(Spiral(s) => s.params.milling), Message::SpiralMillingChanged;
    SPIRAL_LINK_ORDER: bool = "linkInsideOut", get!(Spiral(s) => s.params.link_inside_out), Message::SpiralLinkInsideOutToggled;
    SPIRAL_CLOCKWISE: bool = "clockwise", get!(Spiral(s) => s.params.clockwise), Message::SpiralClockwiseToggled;
    SPIRAL_DEPTHS: bool = "multipleDepths", get!(Spiral(s) => s.params.multiple_depths), Message::SpiralDepthsToggled;
    SPIRAL_DEPTH_ORDER: bool = "orderByDepth", get!(Spiral(s) => s.params.order_by_depth), Message::SpiralDepthOrderToggled;
    SPIRAL_BOUNDARY: SpiralBoundary = "machiningBoundary", get!(Spiral(s) => s.params.boundary), Message::SpiralBoundaryChanged;
    SPIRAL_CONTACT: bool = "contactOnly", get!(Spiral(s) => s.params.contact_only), Message::SpiralContactOnlyToggled;
    PARALLEL_SCALLOP: bool = "scallop", get!(Parallel(p) => p.params.scallop), Message::ParallelScallopToggled;
    PARALLEL_BOTH_WAYS: bool = "bothWays", get!(Parallel(p) => p.params.both_ways), Message::ParallelBothWaysToggled;
    PARALLEL_TRANSITION: ParallelTransition = "transitionType", get!(Parallel(p) => p.params.transition), Message::ParallelTransitionChanged;
    SCALLOP_ORDER: ScallopOrder = "scallopOrder", get!(Parallel(p) => p.params.scallop_order), Message::ScallopOrderChanged;
    SMOOTHING: bool = "smoothing", |op| match op {
        Operation::Parallel(p) => Some(p.params.scallop_smoothing),
        Operation::Spiral(p) => Some(p.params.smoothing),
        _ => None,
    }, Message::ScallopSmoothingToggled;
    RETRACTION: ScallopRetraction = "retractionPolicy", |op| match op {
        Operation::Parallel(p) => Some(p.params.retraction_policy()),
        Operation::Spiral(p) => Some(p.params.retraction),
        _ => None,
    }, Message::ScallopRetractionChanged;
    ADAPTIVE_CLIMB: bool = "climb", get!(Adaptive(a) => a.params.climb), Message::AdaptiveClimbToggled;
    ENGRAVE_CLOSED: bool = "closedChain", get!(Engrave(e) => e.closed), Message::EngraveClosedChanged;
    CARVE_STAY_DOWN: bool = "stayDown", get!(Carve(c) => c.stay_down), Message::CarveStayDownToggled;
    CARVE_CLEAR: bool = "clearingPass", get!(Carve(c) => c.clear.is_some()), Message::CarveClearToggled;
    CARVE_CLEAR_CLIMB: bool = "clearingClimb", get!(Carve(c) => clear_params(c).clearing.climb), Message::CarveClearClimbToggled;
    CARVE_CLEAR_PLUNGE: PlungeKind = "clearingPlunge", get!(Carve(c) => PlungeKind::of(clear_params(c).plunge)), Message::CarveClearPlungeChanged;
    CARVE_CLEAR_LEAD_IN: LeadKind = "clearingLeadIn", get!(Carve(c) => LeadKind::of(clear_params(c).lead_in)), Message::CarveClearLeadInChanged;
    CARVE_CLEAR_LEAD_OUT: LeadKind = "clearingLeadOut", get!(Carve(c) => LeadKind::of(clear_params(c).lead_out)), Message::CarveClearLeadOutChanged;
    COOLING: Cooling = "cooling", |op: &Operation| Some(op.cooling()), Message::OperationCoolingChanged;
    PROFILE_FINISH: bool = "finishing", get!(Profile(p) => ProfileToggle::Finishing.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::Finishing, v);
    PROFILE_FINISH_FINAL: bool = "finishOnlyAtFinalDepth", get!(Profile(p) => ProfileToggle::FinishOnlyFinal.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::FinishOnlyFinal, v);
    PROFILE_MULTIPLE: bool = "multipleDepths", get!(Profile(p) => ProfileToggle::MultipleDepths.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::MultipleDepths, v);
    PROFILE_EVEN: bool = "evenStepdowns", get!(Profile(p) => ProfileToggle::Even.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::Even, v);
    PROFILE_DEPTH_ORDER: bool = "orderByDepth", get!(Profile(p) => ProfileToggle::OrderDepth.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::OrderDepth, v);
    PROFILE_AREA_ORDER: bool = "orderByArea", get!(Profile(p) => ProfileToggle::OrderArea.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::OrderArea, v);
    PROFILE_RADIAL_ORDER: bool = "orderByRadialPass", get!(Profile(p) => ProfileToggle::OrderRadial.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::OrderRadial, v);
    PROFILE_PRESERVE: bool = "preserveContourOrder", get!(Profile(p) => ProfileToggle::Preserve.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::Preserve, v);
    PROFILE_BOTH_WAYS: bool = "bothWays", get!(Profile(p) => ProfileToggle::BothWays.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::BothWays, v);
    PROFILE_KEEP_DOWN: bool = "keepToolDown", get!(Profile(p) => ProfileToggle::KeepToolDown.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::KeepToolDown, v);
    PROFILE_FEED_RETRACT: bool = "feedRetract", get!(Profile(p) => ProfileToggle::FeedRetract.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::FeedRetract, v);
    PROFILE_SAME_OUT: bool = "sameLeadOut", get!(Profile(p) => ProfileToggle::SameLeadOut.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::SameLeadOut, v);
    PROFILE_ENTRY: bool = "leadInEnabled", get!(Profile(p) => ProfileToggle::LeadInEnabled.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::LeadInEnabled, v);
    PROFILE_EXIT: bool = "leadOutEnabled", get!(Profile(p) => ProfileToggle::LeadOutEnabled.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::LeadOutEnabled, v);
    PROFILE_LEAD_END: bool = "useLeadEndDistance", get!(Profile(p) => ProfileToggle::LeadEndDistance.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::LeadEndDistance, v);
    PROFILE_ROUGH_FINAL: bool = "roughFinalDepths", get!(Profile(p) => ProfileToggle::RoughFinal.get(p)), |v| Message::ProfilePassToggle(ProfileToggle::RoughFinal, v);
    HEIGHT_CLEARANCE: Reference = "clearanceReference", |_| height_reference(Plane::Clearance), |value| Message::HeightReferenceChanged(Plane::Clearance, value);
    HEIGHT_RETRACT: Reference = "retractReference", |_| height_reference(Plane::Retract), |value| Message::HeightReferenceChanged(Plane::Retract, value);
    HEIGHT_FEED: Reference = "feedReference", |_| height_reference(Plane::Feed), |value| Message::HeightReferenceChanged(Plane::Feed, value);
    HEIGHT_TOP: Reference = "topReference", |_| height_reference(Plane::Top), |value| Message::HeightReferenceChanged(Plane::Top, value);
    HEIGHT_BOTTOM: Reference = "bottomReference", |_| height_reference(Plane::Bottom), |value| Message::HeightReferenceChanged(Plane::Bottom, value);
    HEIGHT_FEED_ENABLED: bool = "feedHeightEnabled", |_| Some(height_reference(Plane::Feed) != Some(Reference::Disabled)), Message::HeightFeedEnabled;
}

fn height_reference(plane: Plane) -> Option<Reference> {
    Some(
        OperationHeightSettings::default()
            .definition(plane)
            .reference,
    )
}

pub(super) fn height_reference_setting(plane: Plane) -> Setting<Reference> {
    match plane {
        Plane::Clearance => HEIGHT_CLEARANCE,
        Plane::Retract => HEIGHT_RETRACT,
        Plane::Feed => HEIGHT_FEED,
        Plane::Top => HEIGHT_TOP,
        Plane::Bottom => HEIGHT_BOTTOM,
    }
}

fn multiple_depths(op: &Operation) -> Option<&cam_model::MultipleDepths> {
    match op {
        Operation::Pocket(p) => Some(&p.clear.multiple_depths),
        Operation::Face(f) => Some(&f.multiple_depths),
        _ => None,
    }
}

fn clear_params(carve: &cam_model::CarveOp) -> ClearParams {
    carve
        .clear
        .as_ref()
        .map(|clear| clear.params)
        .unwrap_or_default()
}

pub(super) fn profile_toggle(toggle: ProfileToggle) -> Setting<bool> {
    match toggle {
        ProfileToggle::Finishing => PROFILE_FINISH,
        ProfileToggle::FinishOnlyFinal => PROFILE_FINISH_FINAL,
        ProfileToggle::MultipleDepths => PROFILE_MULTIPLE,
        ProfileToggle::Even => PROFILE_EVEN,
        ProfileToggle::OrderDepth => PROFILE_DEPTH_ORDER,
        ProfileToggle::OrderArea => PROFILE_AREA_ORDER,
        ProfileToggle::OrderRadial => PROFILE_RADIAL_ORDER,
        ProfileToggle::Preserve => PROFILE_PRESERVE,
        ProfileToggle::BothWays => PROFILE_BOTH_WAYS,
        ProfileToggle::KeepToolDown => PROFILE_KEEP_DOWN,
        ProfileToggle::FeedRetract => PROFILE_FEED_RETRACT,
        ProfileToggle::SameLeadOut => PROFILE_SAME_OUT,
        ProfileToggle::LeadInEnabled => PROFILE_ENTRY,
        ProfileToggle::LeadOutEnabled => PROFILE_EXIT,
        ProfileToggle::LeadEndDistance => PROFILE_LEAD_END,
        ProfileToggle::RoughFinal => PROFILE_ROUGH_FINAL,
    }
}

pub(super) fn pocket_toggle(toggle: PocketToggle) -> Setting<bool> {
    match toggle {
        PocketToggle::Enabled => POCKET_FINISHING,
        PocketToggle::AllLeads => POCKET_FINISH_LEADS,
        PocketToggle::Repeat => POCKET_FINISH_REPEAT,
        PocketToggle::FinishOnlyFinal => POCKET_FINISH_FINAL,
    }
}

pub(super) fn depth_toggle(toggle: DepthToggle) -> Setting<bool> {
    match toggle {
        DepthToggle::Enabled => MULTIPLE_DEPTHS,
        DepthToggle::Even => EVEN_DEPTHS,
        DepthToggle::ByDepth => DEPTH_ORDER,
        DepthToggle::ByArea => AREA_ORDER,
    }
}

impl App {
    pub(super) fn creation_cooling_override(&self) -> Option<Cooling> {
        let op = self.operation_draft.as_ref()?;
        let source = self.user_option_default_source(op, COOLING.spec.name)?;
        (COOLING.spec.change)(source)?;
        Some(op.cooling())
    }

    pub(super) fn seed_user_option_defaults(&mut self) {
        self.seed_user_options(OPTIONS);
    }

    pub(super) fn seed_user_clearing_options(&mut self) {
        self.seed_user_options(&[
            CARVE_CLEAR_CLIMB.spec,
            CARVE_CLEAR_PLUNGE.spec,
            CARVE_CLEAR_LEAD_IN.spec,
            CARVE_CLEAR_LEAD_OUT.spec,
        ]);
    }

    pub(super) fn seed_user_rotary_index_options(&mut self) {
        self.seed_user_options(&[ROTARY_AXIS.spec, ROTARY_ANGLE_MODE.spec, ROTARY_FLIP.spec]);
    }

    pub(super) fn seed_user_rotary_wrap_options(&mut self) {
        self.seed_user_options(&[ROTARY_UNWRAP.spec]);
    }

    fn seed_user_options(&mut self, options: &[&OptionSpec]) {
        if !self.creating_operation() || self.settings.parameter_defaults.is_empty() {
            return;
        }
        for spec in options {
            let message = self.parameter_operation().and_then(|op| {
                let source = self.user_option_default_source(op, spec.name)?;
                let current = (spec.read)(op)?;
                let message = (spec.change)(source)?;
                let changed = match &message {
                    Message::StockToLeaveToggled(enabled) => {
                        *enabled != self.stock_to_leave_enabled()
                    }
                    Message::HeightFeedEnabled(enabled) => {
                        self.height_draft.as_ref().is_some_and(|heights| {
                            *enabled
                                != (heights.definition(Plane::Feed).reference
                                    != Reference::Disabled)
                        })
                    }
                    Message::HeightReferenceChanged(plane, reference) => self
                        .height_draft
                        .as_ref()
                        .is_some_and(|heights| *reference != heights.definition(*plane).reference),
                    Message::Rotary(RotaryAction::IndexAxis(axis)) => self
                        .rotary_index_draft
                        .as_ref()
                        .is_some_and(|index| index.axis != *axis),
                    Message::Rotary(RotaryAction::IndexMode(mode)) => {
                        self.rotary_index_mode != *mode
                    }
                    Message::Rotary(RotaryAction::IndexFlip(flip)) => self
                        .rotary_index_draft
                        .as_ref()
                        .is_some_and(|index| index.flip != *flip),
                    Message::Rotary(RotaryAction::Unwrap(unwrap)) => op
                        .rotary_wrap()
                        .is_some_and(|wrap| wrap.unwrap_geometry != *unwrap),
                    _ => source != current,
                };
                changed.then_some(message)
            });
            if let Some(message) = message {
                if self.option_default_allowed(&message) {
                    let _ = self.update_message(message);
                }
            }
        }
    }

    pub(super) fn option_default_allowed(&self, message: &Message) -> bool {
        let Some(op) = self.parameter_operation() else {
            return false;
        };
        match (message, op) {
            (Message::Rotary(RotaryAction::Enable(true)), op) => op.supports_rotary_wrap(),
            (
                Message::Rotary(
                    RotaryAction::IndexAxis(_)
                    | RotaryAction::IndexMode(_)
                    | RotaryAction::IndexFlip(_),
                ),
                _,
            ) => self.rotary_index_draft.is_some(),
            (Message::Rotary(RotaryAction::Unwrap(_)), op) => op.rotary_wrap().is_some(),
            (
                Message::CarveClearClimbToggled(_)
                | Message::CarveClearPlungeChanged(_)
                | Message::CarveClearLeadInChanged(_)
                | Message::CarveClearLeadOutChanged(_),
                Operation::Carve(c),
            ) => c.clear.is_some(),
            (Message::HeightReferenceChanged(plane, reference), _) => {
                Reference::options(*plane).contains(reference)
            }
            (Message::ProfileCompChanged(Comp::Control), _) => {
                self.controller.post_kind().supports_cutter_comp()
            }
            (Message::ProfileTabsEnabled(true), Operation::Profile(p)) => {
                p.capabilities(
                    self.operation_uses_cad_geometry(),
                    self.controller.post_kind().supports_cutter_comp(),
                )
                .tabs
            }
            (
                Message::ProfilePassToggle(ProfileToggle::KeepToolDown, true),
                Operation::Profile(p),
            ) => p.comp == Comp::Computed && !p.tabs.enabled && p.additional_chains.is_empty(),
            (Message::PlungeKindChanged(kind), Operation::Profile(p))
                if *kind != PlungeKind::Straight =>
            {
                p.capabilities(
                    self.operation_uses_cad_geometry(),
                    self.controller.post_kind().supports_cutter_comp(),
                )
                .ramp
            }
            (
                Message::LeadInKindChanged(kind) | Message::LeadOutKindChanged(kind),
                Operation::Profile(p),
            ) if p.comp == Comp::Control => *kind == LeadKind::Linear,
            _ => true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::tests::Scratch;
    use parameter_defaults::Action;

    #[test]
    fn checkbox_and_dropdown_defaults_round_trip_and_seed_new_operations() {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "options.dxf").unwrap();
        let _ = app.update(Message::BeginOp(OpKind::Pocket));
        let scratch = Scratch::new("operation-option-defaults");
        let path = scratch.file("settings.json");

        let _ = app.update(Message::PocketAdaptiveToggled(true));
        let _ = app.update(Message::PlungeKindChanged(PlungeKind::Helix));
        let adaptive = ADAPTIVE_POCKET.selection(&true, "On".into(), &[false, true]);
        let plunge = PLUNGE.selection(&PlungeKind::Helix, "Helix".into(), &PlungeKind::ALL);
        for (option, system) in [
            (&adaptive, encode(&false)),
            (&plunge, encode(&PlungeKind::Straight)),
        ] {
            app.save_option_default(option, |app, settings| {
                app.save_preferences_to(settings, &path)
            });
            let _ = app.update(Message::OpenParameterOptionMenu(option.clone()));
            assert!(app.parameter_menu_overlay().is_some());
            let _ = app.update(Message::ParameterMenuAction(Action::ResetSystem));
            let op = app.parameter_operation().unwrap();
            assert_eq!((option.spec.read)(op), Some(system));
            let _ = app.update(Message::OpenParameterOptionMenu(option.clone()));
            let _ = app.update(Message::ParameterMenuAction(Action::ResetUser));
            assert_eq!(
                (option.spec.read)(app.parameter_operation().unwrap()).as_deref(),
                Some(option.source.as_str()),
            );
        }

        let _ = app.update(Message::CancelOp);
        let (settings, outcome) = crate::settings::load_from(&path);
        assert_eq!(outcome, crate::settings::LoadOutcome::Loaded);
        app.settings = settings;
        let _ = app.update(Message::BeginOp(OpKind::Pocket));
        let op = app.parameter_operation().unwrap();
        assert_eq!((adaptive.spec.read)(op), Some(adaptive.source));
        assert_eq!((plunge.spec.read)(op), Some(plunge.source));
    }

    #[test]
    #[ignore = "requires a graphics adapter to dispatch widget events"]
    fn right_click_opens_parameter_menus_but_not_section_heading_menus() {
        use iced::advanced::{layout, mouse, renderer::Headless, widget::Tree, Layout, Shell};
        use iced::{Event, Point, Rectangle, Size};

        let renderer = iced::futures::executor::block_on(<iced::Renderer as Headless>::new(
            iced::Font::DEFAULT,
            iced::Pixels(12.),
            None,
        ))
        .expect("graphics adapter");
        let (app, _) = App::new();
        let heading = operation_sections::Form::new(&app, OperationTab::Passes, None)
            .optional_section("Stock to Leave", false, Some(STOCK_LEAVE))
            .finish();
        for (name, position, mut element) in [
            (
                Some(ADAPTIVE_POCKET.spec.name),
                Point::new(8., 10.),
                ADAPTIVE_POCKET.checkbox(false, 15),
            ),
            (
                Some(PLUNGE.spec.name),
                Point::new(8., 10.),
                PLUNGE.picker(PlungeKind::Straight, &PlungeKind::ALL, Length::Fixed(100.)),
            ),
            (None, Point::new(26., 33.), heading),
        ] {
            let mut tree = Tree::new(&element);
            let size = Size::new(300., 100.);
            let node = element.as_widget_mut().layout(
                &mut tree,
                &renderer,
                &layout::Limits::new(Size::ZERO, size),
            );
            let mut click = |button| {
                let mut messages = Vec::new();
                element.as_widget_mut().update(
                    &mut tree,
                    &Event::Mouse(mouse::Event::ButtonPressed(button)),
                    Layout::new(&node),
                    mouse::Cursor::Available(position),
                    &renderer,
                    &mut iced::advanced::clipboard::Null,
                    &mut Shell::new(&mut messages),
                    &Rectangle::with_size(size),
                );
                messages
            };
            let messages = click(mouse::Button::Right);
            if let Some(name) = name {
                assert!(matches!(
                    messages.as_slice(),
                    [Message::OpenParameterOptionMenu(option)] if option.spec.name == name
                ));
            } else {
                assert!(messages.is_empty());
                assert!(matches!(
                    click(mouse::Button::Left).as_slice(),
                    [Message::StockToLeaveToggled(true)]
                ));
            }
        }
    }
}
