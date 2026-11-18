//! Inspector adapters for the shared profile pass model.
use super::*;
pub(super) const CUT_ORDER_HELP: &str = "By contour completes each contour before the next. By depth visits contours at each depth before going deeper. By radial pass completes each radial stage across contours and depths before the next. Outside stock clearing runs first when enabled.";
pub(super) const CORNER_HELP: &str = "Roll around corner uses a rounded offset at outside corners. Keep sharp corner extends the offset edges to their intersection; very acute corners are bevelled to limit the extension.";
pub(super) const WALL_ALLOWANCE_HELP: &str = "Signed offset of the final profile wall from the selected contour. Positive leaves material on the part; negative cuts past the nominal wall. This allowance remains after the finishing passes.";
pub(super) const PROFILE_FINISHING_HELP: &str = "Enable radial finishing cuts after roughing. Set the number and spacing of finishing cuts, the finishing feed, and any spring passes. The final wall retains the wall allowance.";
use cam_model::{ProfileOp, ProfileOrder};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum StockChoice {
    Setup,
    Drawing(usize),
    Saved,
}
impl std::fmt::Display for StockChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Setup => f.write_str("Setup stock"),
            Self::Drawing(i) => write!(f, "Drawing outline {}", i + 1),
            Self::Saved => f.write_str("Saved boundary snapshot"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn editor() -> (App, u32) {
        let (mut app, _) = App::new();
        app.controller = AppController::new(crate::default_machine());
        app.controller.open_dxf(SAMPLE_DXF, "profile.dxf").unwrap();
        app.controller.new_operation(OpKind::Profile);
        let id = app
            .controller
            .inspection_setup()
            .operations
            .last()
            .unwrap()
            .id();
        app.begin_operation_draft(id);
        (app, id)
    }
    #[test]
    fn multiple_depths_toggles_keep_settings_and_map_ordering() {
        let (mut app, id) = editor();
        let _ = app.update(Message::ProfilePassToggle(Toggle::Even, true));
        let _ = app.update(Message::ProfilePassToggle(Toggle::OrderDepth, true));
        let _ = app.update(Message::ProfilePassToggle(Toggle::OrderArea, true));
        let _ = app.update(Message::ProfilePassToggle(Toggle::MultipleDepths, false));
        let _ = app.update(Message::ProfilePassToggle(Toggle::MultipleDepths, true));
        assert!(
            matches!(app.current_operation(id), Some(Operation::Profile(p)) if p.passes.even_stepdowns && p.passes.order_by_area && p.passes.order == ProfileOrder::Depth)
        );
        let _ = app.update(Message::ProfilePassToggle(Toggle::OrderRadial, true));
        assert!(
            matches!(app.current_operation(id), Some(Operation::Profile(p)) if p.passes.order == ProfileOrder::Radial)
        );
        let _ = app.update(Message::ProfilePassToggle(Toggle::Preserve, true));
        assert!(
            matches!(app.current_operation(id), Some(Operation::Profile(p)) if !p.passes.order_by_area)
        );
    }

    #[test]
    fn keep_tool_down_draft_applies_cancels_and_undoes() {
        let (mut app, id) = editor();
        let checked =
            |op: &Operation| matches!(op, Operation::Profile(p) if p.passes.keep_tool_down);
        let _ = app.update(Message::ProfilePassToggle(Toggle::KeepToolDown, true));
        assert!(checked(app.current_operation(id).unwrap()));
        assert!(!checked(app.controller.operation(id).unwrap()));
        let _ = app.update(Message::CancelOperationDraft);
        assert!(!checked(app.controller.operation(id).unwrap()));
        app.begin_operation_draft(id);
        let _ = app.update(Message::ProfilePassToggle(Toggle::KeepToolDown, true));
        let _ = app.update(Message::OperationTabChanged(OperationTab::Linking));
        let _ = app.update(Message::Apply);
        assert!(
            checked(app.controller.operation(id).unwrap()),
            "{}",
            app.status
        );
        assert!(app.controller.undo());
        assert!(!checked(app.controller.operation(id).unwrap()));
        assert!(app.controller.redo());
        assert!(checked(app.controller.operation(id).unwrap()));
    }

    #[test]
    fn contour_roughing_draft_retains_values_and_undoes() {
        let (mut app, id) = editor();
        let before = app.controller.operation(id).unwrap().clone();
        let _ = app.update(Message::ContourRoughingToggled(true));
        let _ = app.update(Message::FieldChanged(
            Field::ContourRoughStepover,
            "2".into(),
        ));
        let _ = app.update(Message::FieldChanged(
            Field::ContourRoughSmoothing,
            "0.2".into(),
        ));
        let _ = app.update(Message::FieldChanged(Field::ContourRoughCount, "3".into()));
        let _ = app.update(Message::ContourRoughingToggled(false));
        let _ = app.update(Message::ContourRoughingToggled(true));
        assert_eq!(app.fields[&Field::ContourRoughCount], "3");
        let _ = app.update(Message::Apply);
        assert!(
            matches!(app.controller.operation(id), Some(Operation::Profile(p)) if p.passes.contour_roughing.enabled && p.passes.contour_roughing.stepover == 2. && p.passes.contour_roughing.smoothing_deviation == 0.2 && p.passes.contour_roughing.stepovers == 3),
            "{}",
            app.status
        );
        assert!(app.controller.undo());
        assert_eq!(app.controller.operation(id), Some(&before));
    }

    #[test]
    fn finishing_draft_survives_collapse_commits_atomically_and_undoes() {
        let (mut app, id) = editor();
        let before = app.controller.operation(id).unwrap().clone();
        let _ = app.update(Message::ProfilePassToggle(Toggle::Finishing, true));
        let _ = app.update(Message::FieldChanged(
            Field::Profile(Number::FinishCount),
            "3".into(),
        ));
        let _ = app.update(Message::FieldChanged(
            Field::Profile(Number::FinishFeed),
            "123".into(),
        ));
        let _ = app.update(Message::ProfilePassToggle(Toggle::Finishing, false));
        let _ = app.update(Message::ProfilePassToggle(Toggle::Finishing, true));
        assert_eq!(app.fields[&Field::Profile(Number::FinishCount)], "3");
        let _ = app.update(Message::OperationTabChanged(OperationTab::Linking));
        let _ = app.update(Message::Apply);
        assert!(
            matches!(app.controller.operation(id),Some(Operation::Profile(p)) if p.passes.finishing&&p.passes.finish_count==3&&p.passes.finish_feed==123.)
        );
        assert!(app.controller.undo());
        assert_eq!(app.controller.operation(id), Some(&before));
    }
    #[test]
    fn cross_field_errors_block_apply_and_cancel_preserves_committed_settings() {
        let (mut app, id) = editor();
        let before = app.controller.operation(id).unwrap().clone();
        let _ = app.update(Message::FieldChanged(
            Field::Profile(Number::FinishDepths),
            "1000".into(),
        ));
        assert!(app.any_field_invalid());
        let _ = app.update(Message::Apply);
        assert_eq!(app.controller.operation(id), Some(&before));
        let _ = app.update(Message::CancelOperationDraft);
        assert_eq!(app.controller.operation(id), Some(&before));
    }
    #[test]
    fn profile_arc_leads_from_editor_reach_generated_program() {
        use cam_toolpath::{ProfileStrategy, Strategy};
        let (mut app, id) = editor();
        let _ = app.update(Message::OperationTabChanged(OperationTab::Linking));
        let _ = app.update(Message::ProfilePassToggle(Toggle::SameLeadOut, false));
        let _ = app.update(Message::LeadInKindChanged(LeadKind::Arc));
        let _ = app.update(Message::LeadOutKindChanged(LeadKind::Arc));
        let _ = app.update(Message::FieldChanged(Field::LeadInSize, "2.5".into()));
        let _ = app.update(Message::FieldChanged(Field::LeadOutSize, "1.5".into()));
        let _ = app.update(Message::ProfilePassToggle(Toggle::Finishing, true));
        let _ = app.update(Message::ProfilePassToggle(Toggle::KeepToolDown, true));
        let _ = app.update(Message::Apply);
        let Some(Operation::Profile(op)) = app.controller.operation(id) else {
            panic!("profile expected")
        };
        assert_eq!(
            op.lead_in,
            Lead::ArcSweep {
                radius: 2.5,
                angle_deg: 90.
            }
        );
        assert_eq!(
            op.lead_out,
            Lead::ArcSweep {
                radius: 1.5,
                angle_deg: 90.
            }
        );
        let tools = [cam_model::Tool {
            number: op.tool,
            diameter: 6.,
            length: 30.,
            ..Default::default()
        }];
        let result = ProfileStrategy::new(op.clone()).compute(
            &cam_toolpath::JobEnv {
                heights: cam_model::Heights::new(5., 2., 0.),
                tools: &tools,
                stock: None,
                spindle: cam_cldata::SpindleDir::Cw,
            },
            &cam_toolpath::CancelToken::new(),
        );
        assert!(!result.has_errors(), "{:?}", result.diagnostics);
        result.program.validate().unwrap();
        for kind in [cam_cldata::MoveKind::LeadIn, cam_cldata::MoveKind::LeadOut] {
            assert!(result
                .program
                .steps()
                .iter()
                .any(|step| matches!(step, cam_cldata::Step::Arc { tag, .. } if tag.kind == kind)));
        }
    }

    #[test]
    fn one_arc_editor_preserves_legacy_radius_and_edits_sweep_without_duplicate_modes() {
        let (mut app, _) = editor();
        let _ = app.update(Message::LeadInKindChanged(LeadKind::Arc));
        let radius = match &app.operation_draft {
            Some(Operation::Profile(p)) => lead_size(p.lead_in),
            _ => unreachable!(),
        };
        assert_eq!(app.fields[&Field::Profile(Number::LeadInSweep)], "90");
        let _ = app.update(Message::FieldChanged(
            Field::Profile(Number::LeadInSweep),
            "45".into(),
        ));
        let _ = app.update(Message::ProfilePassToggle(Toggle::SameLeadOut, true));
        assert!(
            matches!(&app.operation_draft, Some(Operation::Profile(p)) if p.lead_in == Lead::ArcSweep { radius, angle_deg: 45. })
        );
        assert_eq!(
            LeadKind::of(Lead::Arc { radius }),
            LeadKind::of(Lead::ArcSweep {
                radius,
                angle_deg: 45.
            })
        );
    }
    #[test]
    fn compensation_method_does_not_change_milling_direction() {
        let (mut app, _) = editor();
        let _ = app.update(Message::ClearingClimbToggled(false));
        let _ = app.update(Message::ProfileCompChanged(cam_model::Comp::Control));
        assert!(matches!(&app.operation_draft,Some(Operation::Profile(p)) if !p.clearing.climb));
    }

    #[test]
    fn lead_end_switch_preserves_distance_and_starts_disabled() {
        let (mut app, _) = editor();
        let Some(Operation::Profile(p)) = &app.operation_draft else {
            panic!("profile draft")
        };
        assert_eq!(p.passes.lead_end_distance, 0.6);
        assert_eq!(p.passes.effective_lead_end_distance(), 0.);
        let _ = app.update(Message::ProfilePassToggle(Toggle::LeadEndDistance, true));
        assert_eq!(app.fields[&Field::Profile(Number::LeadEndDistance)], "0.6");
        let _ = app.update(Message::FieldChanged(
            Field::Profile(Number::LeadEndDistance),
            "1.2".into(),
        ));
        let _ = app.update(Message::ProfilePassToggle(Toggle::LeadEndDistance, false));
        let _ = app.update(Message::ProfilePassToggle(Toggle::LeadEndDistance, true));
        assert_eq!(app.fields[&Field::Profile(Number::LeadEndDistance)], "1.2");
    }

    #[test]
    fn entry_and_exit_remain_independent_when_entry_is_disabled() {
        let (mut app, _) = editor();
        let _ = app.update(Message::ProfilePassToggle(Toggle::SameLeadOut, true));
        let _ = app.update(Message::ProfilePassToggle(Toggle::LeadInEnabled, false));
        let Some(Operation::Profile(p)) = &app.operation_draft else {
            panic!("profile draft")
        };
        assert_eq!(p.passes.lead_in_enabled, Some(false));
        assert!(p.passes.same_lead_out);
        assert!(p.lead_out != Lead::None);
        assert!(app.fields.contains_key(&Field::LeadOutSize));
        assert!(!app.fields.contains_key(&Field::LeadInSize));
        let _ = app.update(Message::ProfilePassToggle(Toggle::LeadInEnabled, true));
        assert!(!app.fields.contains_key(&Field::LeadOutSize));
        assert!(app.fields.contains_key(&Field::LeadInSize));
    }

    #[test]
    fn disabling_same_as_entry_keeps_implicit_exit_enabled() {
        let (mut app, _) = editor();
        let Some(Operation::Profile(profile)) = &mut app.operation_draft else {
            panic!("profile draft")
        };
        profile.lead_in = Lead::ArcSweep {
            radius: 1.2,
            angle_deg: 60.,
        };
        profile.lead_out = Lead::None;
        profile.passes.same_lead_out = true;
        profile.passes.lead_out_enabled = None;
        app.refresh_fields();
        let _ = app.update(Message::ProfilePassToggle(Toggle::SameLeadOut, false));
        let Some(Operation::Profile(profile)) = &app.operation_draft else {
            panic!("profile draft")
        };
        assert!(!profile.passes.same_lead_out);
        assert!(Toggle::LeadOutEnabled.get(profile));
        assert_eq!(
            profile.lead_out,
            Lead::ArcSweep {
                radius: 1.2,
                angle_deg: 60.
            }
        );
        assert_eq!(app.fields[&Field::LeadOutSize], "1.2");
        assert_eq!(app.fields[&Field::Profile(Number::LeadOutSweep)], "60");
    }

    #[test]
    fn disabling_same_as_entry_preserves_disabled_exit() {
        let (mut app, _) = editor();
        let Some(Operation::Profile(profile)) = &mut app.operation_draft else {
            panic!("profile draft")
        };
        profile.lead_out = Lead::None;
        profile.passes.same_lead_out = true;
        profile.passes.lead_out_enabled = Some(false);
        app.refresh_fields();
        let _ = app.update(Message::ProfilePassToggle(Toggle::SameLeadOut, false));
        let Some(Operation::Profile(profile)) = &app.operation_draft else {
            panic!("profile draft")
        };
        assert!(!Toggle::LeadOutEnabled.get(profile));
        assert_eq!(profile.lead_out, Lead::None);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum Number {
    FinishCount,
    FinishStep,
    FinishFeed,
    SpringPasses,
    FloorLeave,
    FinishDepths,
    FinishDepthStep,
    TangentialExtensionDistance,
    StartExtension,
    EndExtension,
    Tolerance,
    Taper,
    TabCount,
    LeadInSweep,
    LeadOutSweep,
    LeadEndDistance,
    RampRpm,
}
impl Number {
    pub(super) fn label(self) -> &'static str {
        match self {
            Self::FinishCount => "Finishing passes",
            Self::FinishStep => "Finish spacing (mm)",
            Self::FinishFeed => "Finish feed (mm/min)",
            Self::SpringPasses => "Spring passes",
            Self::FloorLeave => "Floor allowance (mm)",
            Self::FinishDepths => "Finishing depths",
            Self::FinishDepthStep => "Finishing step (mm)",
            Self::TangentialExtensionDistance => "Tangential extension distance (mm)",
            Self::StartExtension => "Start extension (mm)",
            Self::EndExtension => "End extension (mm)",
            Self::Tolerance => "Arc-fit tolerance (mm)",
            Self::Taper => "Wall taper (deg)",
            Self::TabCount => "Tabs per contour",
            Self::LeadInSweep => "Lead-In Sweep Angle (deg)",
            Self::LeadOutSweep => "Lead-Out Sweep Angle (deg)",
            Self::LeadEndDistance => "Lead end distance (mm)",
            Self::RampRpm => "Ramp spindle (rpm)",
        }
    }
    pub(super) fn help(self) -> &'static str {
        match self {
        Self::FinishCount => "Number of radial finishing cuts, including the final allowanced wall. Each earlier cut is one finish spacing farther from that wall.",
        Self::FinishStep => "Radial spacing between finishing cuts and between the roughing contour and the first finishing cut. The final finishing cut still retains the wall allowance. Separate from roughing stepover.",
        Self::FinishFeed => "Feed for the finishing stage. Zero inherits the cutting feed.",
        Self::SpringPasses => "Additional repeats of the final wall path to remove material left by tool deflection.",
        Self::FloorLeave => "Signed axial allowance above the selected bottom. Positive leaves material; negative cuts below the nominal bottom. Linked CAD requires a nonnegative value.",
        Self::FinishDepths => "Number of fixed-size depth increments reserved at the bottom of the cut. Zero uses the regular stepdown throughout. This is independent of radial finishing passes.",
        Self::FinishDepthStep => "Height of each reserved bottom depth increment. Their combined height must be smaller than the total cut. This is independent of radial finish spacing.",
        Self::TangentialExtensionDistance => "Extend both ends of each open toolpath in a straight line along its endpoint tangents. Overlapping extensions are not trimmed. Zero disables the extension.",
        Self::StartExtension => "Extend the selected open chain backward along its first tangent before offsetting. This does not add a closing edge.",
        Self::EndExtension => "Extend the selected open chain forward along its last tangent before offsetting. Independent of the start extension.",
        Self::Tolerance => "Maximum error used when fitting arcs to the cutting polyline. This does not improve the accuracy of a coarse imported curve.",
        Self::Taper => "Stepped wall slope from the contour at the nominal bottom. Use a negative angle to remove more material at higher levels; stepdown controls the staircase height. Positive angles form an unreachable undercut and are rejected. Unavailable for linked CAD and tabs.",
        Self::LeadInSweep | Self::LeadOutSweep => "Sweep of the tangent lead arc, greater than zero and at most 170 degrees. A smaller angle shortens the arc while retaining tangency at the wall.",
        Self::LeadEndDistance => "Start using the lead-out feed this far before the end of the contour. The path is split before arc fitting so the feed boundary is retained. Unavailable with tabs.",
        Self::RampRpm => "Optional spindle speed during ramp, helix or zigzag entry. Zero keeps the cutting speed. Set an explicit cutting RPM; it is restored before leads or normal cutting, and before leaving each entry. Machine spindle limits still apply.",
        Self::TabCount => "Exact number of equally spaced holding tabs on the compensated contour. Tabs must leave room for the cutter and entry.",
    }
    }
    pub(super) fn get(self, op: &ProfileOp) -> f64 {
        let p = &op.passes;
        match self {
            Self::FinishCount => p.finish_count as f64,
            Self::FinishStep => p.finish_stepover,
            Self::FinishFeed => p.finish_feed,
            Self::SpringPasses => p.spring_passes as f64,
            Self::FloorLeave => p.floor_allowance,
            Self::FinishDepths => p.finishing_depths as f64,
            Self::FinishDepthStep => p.finishing_stepdown,
            Self::TangentialExtensionDistance => p.tangential_extension_distance,
            Self::StartExtension => p.start_extension,
            Self::EndExtension => p.end_extension,
            Self::Tolerance => p.tolerance,
            Self::Taper => p.taper_angle,
            Self::LeadEndDistance => p.lead_end_distance,
            Self::RampRpm => p.ramp_rpm,
            Self::TabCount => op.tabs.count.unwrap_or(4) as f64,
            Self::LeadInSweep => {
                if let Lead::ArcSweep { angle_deg, .. } = op.lead_in {
                    angle_deg
                } else {
                    90.
                }
            }
            Self::LeadOutSweep => {
                if let Lead::ArcSweep { angle_deg, .. } = op.lead_out {
                    angle_deg
                } else {
                    90.
                }
            }
        }
    }
    pub(super) fn set(self, op: &mut ProfileOp, value: f64) {
        let p = &mut op.passes;
        match self {
            Self::FinishCount => p.finish_count = value as u32,
            Self::FinishStep => p.finish_stepover = value,
            Self::FinishFeed => p.finish_feed = value,
            Self::SpringPasses => p.spring_passes = value as u32,
            Self::FloorLeave => p.floor_allowance = value,
            Self::FinishDepths => p.finishing_depths = value as u32,
            Self::FinishDepthStep => p.finishing_stepdown = value,
            Self::TangentialExtensionDistance => p.tangential_extension_distance = value,
            Self::StartExtension => p.start_extension = value,
            Self::EndExtension => p.end_extension = value,
            Self::Tolerance => p.tolerance = value,
            Self::Taper => p.taper_angle = value,
            Self::LeadEndDistance => p.lead_end_distance = value,
            Self::RampRpm => p.ramp_rpm = value,
            Self::TabCount => op.tabs.count = Some(value as u32),
            Self::LeadInSweep => {
                if let Lead::Arc { radius } | Lead::ArcSweep { radius, .. } = op.lead_in {
                    op.lead_in = Lead::ArcSweep {
                        radius,
                        angle_deg: value,
                    };
                }
            }
            Self::LeadOutSweep => {
                if let Lead::Arc { radius } | Lead::ArcSweep { radius, .. } = op.lead_out {
                    op.lead_out = Lead::ArcSweep {
                        radius,
                        angle_deg: value,
                    };
                }
            }
        }
    }
    pub(super) fn invalid(self, v: f64) -> bool {
        !v.is_finite()
            || match self {
                Self::FinishCount => v < 1. || v > 1000. || v.fract() != 0.,
                Self::TabCount => v < 1. || v > 10_000. || v.fract() != 0.,
                Self::SpringPasses | Self::FinishDepths => v < 0. || v > 1000. || v.fract() != 0.,
                Self::FinishStep | Self::FinishDepthStep | Self::Tolerance => v <= 0.,
                Self::LeadInSweep | Self::LeadOutSweep => v <= 0. || v > 170.,
                Self::FloorLeave => false,
                Self::Taper => v <= -80. || v > 0.,
                _ => v < 0.,
            }
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) enum Toggle {
    Finishing,
    FinishOnlyFinal,
    MultipleDepths,
    Even,
    OrderDepth,
    OrderArea,
    OrderRadial,
    Preserve,
    BothWays,
    KeepToolDown,
    FeedRetract,
    SameLeadOut,
    LeadInEnabled,
    LeadOutEnabled,
    LeadEndDistance,
    RoughFinal,
}
impl Toggle {
    fn label(self) -> &'static str {
        match self {
            Self::Finishing => "Finishing passes",
            Self::FinishOnlyFinal => "Finish only at final depth",
            Self::MultipleDepths => "Multiple Depths",
            Self::Even => "Use Even Stepdowns",
            Self::OrderDepth => "Order by Depth",
            Self::OrderArea => "Order By Area",
            Self::OrderRadial => "Order by radial pass",
            Self::Preserve => "Preserve contour order",
            Self::BothWays => "Both ways",
            Self::KeepToolDown => "Keep tool down",
            Self::FeedRetract => "Retract at lead-out feed",
            Self::SameLeadOut => "Exit same as entry",
            Self::LeadInEnabled => "Lead-In (Entry)",
            Self::LeadOutEnabled => "Exit",
            Self::LeadEndDistance => "Lead end distance",
            Self::RoughFinal => "Small bottom steps on rough passes",
        }
    }
    pub(super) fn help(self) -> &'static str {
        match self {
            Self::Finishing => "Enable radial finishing cuts after roughing. Finish spacing, finishing feed and spring passes control how the final wall is cut.",
            Self::FinishOnlyFinal => "Run radial finishing cuts only at the final depth instead of after every depth level.",
            Self::MultipleDepths => "Cut in multiple axial levels. Disable to cut the full depth in one level.",
            Self::OrderDepth => "Visit each contour at the current depth before moving deeper. Selecting this clears radial-pass ordering.",
            Self::OrderArea => "Cut larger closed contours first; open contours follow in their selected order.",
            Self::OrderRadial => "Complete each radial stage across contours and depths before the next. Selecting this clears depth ordering.",
            Self::Even => "Divide the regular depth range into equal increments no larger than the maximum stepdown. Reserved bottom increments retain their specified finishing step.",
            Self::Preserve => "Keep contours in their selected order instead of reordering them to reduce travel.",
            Self::BothWays => "Alternate travel direction on successive passes of an open contour. The contour remains open.",
            Self::KeepToolDown => operation_linking::KEEP_TOOL_DOWN_HELP,
            Self::FeedRetract => "Use the lead-out feed for the retract after leaving the contour.",
            Self::SameLeadOut => "Use the entry lead style, size and arc sweep for the exit lead as well.",
            Self::LeadInEnabled => "Enable the entry lead.",
            Self::LeadOutEnabled => "Enable a lead-out move after the contour.",
            Self::LeadEndDistance => "Start the lead-out feed before the end of the contour.",
            Self::RoughFinal => "Use smaller bottom increments on every radial/roughing stage. When disabled, intermediate stages keep the regular maximum stepdown; the final wall and its spring passes retain the smaller increments.",
        }
    }
    fn get(self, p: &ProfileOp) -> bool {
        if matches!(self, Self::LeadInEnabled) {
            return p.passes.lead_in_enabled.unwrap_or(p.lead_in != Lead::None);
        }
        if matches!(self, Self::LeadOutEnabled) {
            return p.passes.lead_out_enabled.unwrap_or(
                p.lead_out != Lead::None || (p.passes.same_lead_out && p.lead_in != Lead::None),
            );
        }
        let p = &p.passes;
        match self {
            Self::Finishing => p.finishing,
            Self::FinishOnlyFinal => p.finish_only_final,
            Self::MultipleDepths => p.multiple_depths,
            Self::Even => p.even_stepdowns,
            Self::OrderDepth => p.order == ProfileOrder::Depth,
            Self::OrderArea => p.order_by_area,
            Self::OrderRadial => p.order == ProfileOrder::Radial,
            Self::Preserve => p.preserve_order,
            Self::BothWays => p.both_ways,
            Self::KeepToolDown => p.keep_tool_down,
            Self::FeedRetract => p.feed_retract,
            Self::SameLeadOut => p.same_lead_out,
            Self::LeadInEnabled => unreachable!(),
            Self::LeadOutEnabled => unreachable!(),
            Self::LeadEndDistance => p
                .lead_end_distance_enabled
                .unwrap_or(p.lead_end_distance > 0.),
            Self::RoughFinal => p.rough_final,
        }
    }
    pub(super) fn set(self, p: &mut ProfileOp, v: bool) {
        if matches!(self, Self::SameLeadOut) {
            let exit_enabled = Self::LeadOutEnabled.get(p);
            if !v && p.passes.same_lead_out && exit_enabled && p.lead_out == Lead::None {
                p.lead_out = if p.lead_in == Lead::None {
                    Lead::Arc { radius: 0.6 }
                } else {
                    p.lead_in
                };
            }
            p.passes.same_lead_out = v;
            return;
        }
        if matches!(self, Self::LeadInEnabled) {
            p.passes.lead_in_enabled = Some(v);
            if !v && p.passes.same_lead_out && p.lead_out == Lead::None {
                p.lead_out = if p.lead_in == Lead::None {
                    Lead::Arc { radius: 0.6 }
                } else {
                    p.lead_in
                };
            }
            if v && p.lead_in == Lead::None {
                p.lead_in = Lead::Arc { radius: 0.6 };
            }
            return;
        }
        if matches!(self, Self::LeadOutEnabled) {
            p.passes.lead_out_enabled = Some(v);
            if v && p.lead_out == Lead::None {
                p.lead_out = Lead::Arc { radius: 0.6 };
            }
            return;
        }
        let p = &mut p.passes;
        match self {
            Self::Finishing => p.finishing = v,
            Self::FinishOnlyFinal => p.finish_only_final = v,
            Self::MultipleDepths => p.multiple_depths = v,
            Self::Even => p.even_stepdowns = v,
            Self::OrderDepth => {
                if v {
                    p.order = ProfileOrder::Depth;
                } else if p.order == ProfileOrder::Depth {
                    p.order = ProfileOrder::Contour;
                }
            }
            Self::OrderArea => {
                p.order_by_area = v;
                if v {
                    p.preserve_order = false;
                }
            }
            Self::OrderRadial => {
                if v {
                    p.order = ProfileOrder::Radial;
                } else if p.order == ProfileOrder::Radial {
                    p.order = ProfileOrder::Contour;
                }
            }
            Self::Preserve => {
                p.preserve_order = v;
                if v {
                    p.order_by_area = false;
                }
            }
            Self::BothWays => p.both_ways = v,
            Self::KeepToolDown => p.keep_tool_down = v,
            Self::FeedRetract => p.feed_retract = v,
            Self::SameLeadOut => unreachable!(),
            Self::LeadInEnabled => unreachable!(),
            Self::LeadOutEnabled => unreachable!(),
            Self::LeadEndDistance => {
                if v && p.lead_end_distance == 0. {
                    p.lead_end_distance = 0.6;
                }
                p.lead_end_distance_enabled = Some(v);
            }
            Self::RoughFinal => p.rough_final = v,
        }
    }
}

pub(super) fn numeric_fields(p: &ProfileOp) -> Vec<Field> {
    let mut values = vec![
        Number::FinishDepths,
        Number::FinishDepthStep,
        Number::Tolerance,
        Number::Taper,
    ];
    if p.passes
        .lead_end_distance_enabled
        .unwrap_or(p.passes.lead_end_distance > 0.)
    {
        values.push(Number::LeadEndDistance);
    }
    if p.plunge != Plunge::Straight || p.passes.ramp_rpm != 0. {
        values.push(Number::RampRpm);
    }
    if p.passes.finishing {
        values.extend([
            Number::FinishCount,
            Number::FinishStep,
            Number::FinishFeed,
            Number::SpringPasses,
        ]);
    }
    if p.contours().any(|c| !c.closed) {
        values.extend([
            Number::TangentialExtensionDistance,
            Number::StartExtension,
            Number::EndExtension,
        ]);
    }
    if p.tabs.enabled && !p.tabs.manual && p.tabs.count.is_some() {
        values.push(Number::TabCount);
    }
    if p.passes.lead_in_enabled != Some(false)
        && matches!(p.lead_in, Lead::Arc { .. } | Lead::ArcSweep { .. })
    {
        values.push(Number::LeadInSweep);
    }
    if (!p.passes.same_lead_out || p.passes.lead_in_enabled == Some(false))
        && p.passes.lead_out_enabled != Some(false)
        && matches!(p.lead_out, Lead::Arc { .. } | Lead::ArcSweep { .. })
    {
        values.push(Number::LeadOutSweep);
    }
    values.into_iter().map(Field::Profile).collect()
}

impl App {
    pub(super) fn profile_parameter_error(&self) -> Option<String> {
        if self.library_mode() {
            return None;
        }
        let Operation::Profile(mut profile) = self.operation_draft.clone()? else {
            return None;
        };
        let parsed = self.parsed_operation_fields();
        let mut operation = Operation::Profile(profile);
        apply_op_fields(&mut operation, &parsed);
        if let Err(error) = self.apply_parameter_sources(&mut operation) {
            return Some(error);
        }
        let Operation::Profile(p) = operation else {
            unreachable!()
        };
        profile = p;
        let id = if self.creating_operation() {
            0
        } else {
            profile.id
        };
        if let Ok(heights) = self.resolve_height_editor(id) {
            profile.depth = heights.top - heights.bottom;
        }
        let linked = self.operation_binding_preview.is_some()
            || self.cad_pending_kind == Some(OpKind::Profile)
            || (!self.creating_operation()
                && self
                    .controller
                    .geometry_state()
                    .bindings
                    .contains_key(&profile.id));
        if profile.comp == cam_model::Comp::Control
            && !self.controller.post_kind().supports_cutter_comp()
        {
            return Some(
                "The selected post does not support controller compensation; choose Computed."
                    .into(),
            );
        }
        if let Some(tool) = self
            .operation_tools
            .iter()
            .chain(self.controller.inspection_setup().tools.iter())
            .find(|tool| tool.number == profile.tool)
        {
            if let Err(message) = profile.validate_control_leads(tool.radius()) {
                return Some(message);
            }
        }
        profile
            .validate_profile_source(linked)
            .and_then(|_| profile.validate_profile(0.))
            .err()
    }

    pub(super) fn profile_extra_controls(
        &self,
        p: &ProfileOp,
        tab: OperationTab,
        linked: bool,
    ) -> operation_sections::Form<'_> {
        let mut list = operation_sections::Form::new(
            self,
            tab,
            match tab {
                OperationTab::Tool => Some("Cutting data"),
                OperationTab::Passes => Some("Passes"),
                _ => None,
            },
        );
        let number = |n: Number| {
            let field = Field::Profile(n);
            let value = self
                .fields
                .get(&field)
                .cloned()
                .unwrap_or_else(|| fmt_num(n.get(p)));
            self.parameter_field_row(field, n.label(), n.help(), &value)
        };
        let toggle = |t: Toggle| -> Element<'_, Message> {
            row![
                label_help(t.label(), t.help(), self.tooltips),
                checkbox(t.get(p))
                    .size(15)
                    .on_toggle(move |v| Message::ProfilePassToggle(t, v))
            ]
            .spacing(8)
            .align_y(Alignment::Center)
            .into()
        };
        if tab == OperationTab::Tool && (p.plunge != Plunge::Straight || p.passes.ramp_rpm != 0.) {
            list = list.push(number(Number::RampRpm));
        }
        if tab == OperationTab::Passes {
            if !linked && p.capabilities(false, true).roughing {
                let mut choices = vec![StockChoice::Setup];
                choices.extend((0..self.controller.regions().len()).map(StockChoice::Drawing));
                let selected =
                    p.passes
                        .stock_boundary
                        .as_ref()
                        .map_or(StockChoice::Setup, |boundary| {
                            self.controller
                                .regions()
                                .iter()
                                .position(|r| r.outer() == boundary)
                                .map_or(StockChoice::Saved, StockChoice::Drawing)
                        });
                if selected == StockChoice::Saved {
                    choices.push(StockChoice::Saved);
                }
                list=list.push(row![label_help("Roughing boundary","Setup stock or a saved snapshot of a drawing outline. The boundary must contain the allowanced part. Reselect a snapshot after editing its source.",self.tooltips),pick_list(choices,Some(selected),Message::ProfileStockBoundary).text_size(12).padding([7,9]).style(inspector_picker).width(Length::Fill)].spacing(8).align_y(Alignment::Center));
            }
            list = list.optional_section(
                "Multiple Depths",
                p.passes.multiple_depths,
                Some(|v| Message::ProfilePassToggle(Toggle::MultipleDepths, v)),
            );
            if p.passes.multiple_depths {
                let field = Field::Stepdown;
                let value = self
                    .fields
                    .get(&field)
                    .cloned()
                    .unwrap_or_else(|| fmt_num(p.stepdown));
                list = list.push(self.parameter_field_row(
                    field,
                    "Maximum Roughing Stepdown (mm)",
                    "Maximum axial depth removed per pass.",
                    &value,
                ));
                list = list.push(toggle(Toggle::Even));
                list = list.push(toggle(Toggle::OrderDepth));
                list = list.push(toggle(Toggle::OrderArea));
                list = list.push(toggle(Toggle::OrderRadial));
                list = list.push(number(Number::FinishDepths));
                list = list.push(number(Number::FinishDepthStep));
                if p.passes.finishing_depths > 0 {
                    list = list.push(toggle(Toggle::RoughFinal));
                }
            }
            list = list.section("Passes");
            list = list.push(toggle(Toggle::Preserve));
            if p.contours().any(|c| !c.closed) {
                list = list.push(toggle(Toggle::BothWays));
                list = list.push(number(Number::TangentialExtensionDistance));
            }
            list = list.optional_section(
                "Finishing passes",
                p.passes.finishing,
                Some(|v| Message::ProfilePassToggle(Toggle::Finishing, v)),
            );
            if p.passes.finishing {
                for n in [
                    Number::FinishCount,
                    Number::FinishStep,
                    Number::FinishFeed,
                    Number::SpringPasses,
                ] {
                    list = list.push(number(n));
                }
                list = list.push(toggle(Toggle::FinishOnlyFinal));
            }
            list = list.section("Passes").push(number(Number::Tolerance));
            list = list.push(profile_picker(
                "Outer corners",
                CORNER_HELP,
                self.tooltips,
                p.passes.outer_corner,
                &cam_model::ProfileCorner::ALL[..],
                Message::ProfileCorner,
            ));
            if (!linked && !p.tabs.enabled) || p.passes.taper_angle != 0. {
                list = list.push(number(Number::Taper));
            }
        }
        if tab == OperationTab::Linking {
            list = list.push(toggle(Toggle::LeadEndDistance));
            if p.passes
                .lead_end_distance_enabled
                .unwrap_or(p.passes.lead_end_distance > 0.)
            {
                list = list.push(number(Number::LeadEndDistance));
            }
            if p.contours().any(|c| !c.closed) {
                list = list
                    .section("Open contour extensions")
                    .push(number(Number::StartExtension))
                    .push(number(Number::EndExtension));
            }
        }
        list = list.plain();
        if self.parameter_editor.focused.is_none() {
            if let Some(error) = self.profile_parameter_error() {
                list = list.push(text(error).size(12).color(palette::ERROR));
            }
        }
        list
    }
}
