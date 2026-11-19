//! Surface finishing controls, in setup coordinates and millimetres.
/// How surface finishing repositions between cutting passes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScallopRetraction {
    #[default]
    Full,
    Minimum,
    Shortest,
}
impl std::fmt::Display for ScallopRetraction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Full => "Full retraction",
            Self::Minimum => "Minimum retraction",
            Self::Shortest => "Shortest path",
        })
    }
}

/// Shape of the connection between neighboring Parallel rows.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParallelTransition {
    NoContact,
    #[default]
    StraightLine,
    ShortestPath,
    Smooth,
}

impl ParallelTransition {
    pub const ALL: [Self; 4] = [
        Self::NoContact,
        Self::StraightLine,
        Self::ShortestPath,
        Self::Smooth,
    ];
}

impl std::fmt::Display for ParallelTransition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::NoContact => "No contact",
            Self::StraightLine => "Straight line",
            Self::ShortestPath => "Shortest path",
            Self::Smooth => "Smooth",
        })
    }
}

/// Order of Scallop passes by inward distance from the machining boundaries.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScallopOrder {
    #[default]
    OutsideIn,
    InsideOut,
    Shortest,
}
impl std::fmt::Display for ScallopOrder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::OutsideIn => "Outside → in",
            Self::InsideOut => "Inside → out",
            Self::Shortest => "Shortest machining distance",
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ParallelParams {
    /// Generate inward surface-distance contours instead of parallel rows.
    pub scallop: bool,
    /// Pass order measured from the sampled machining boundaries.
    pub scallop_order: ScallopOrder,
    /// Preferred lead-in location in setup XY; each contour starts nearby when possible.
    pub scallop_start: Option<[f64; 2]>,
    pub scallop_retraction: ScallopRetraction,
    pub retraction: ScallopRetraction,
    pub transition: ParallelTransition,
    /// Additional vertical clearance for minimum retraction, in millimetres.
    #[serde(alias = "scallop_safe_distance")]
    pub safe_distance: f64,
    /// Optional post-filtering of the certified Scallop path.
    pub scallop_smoothing: bool,
    /// Maximum additional filtering deviation in millimetres.
    pub scallop_smoothing_tolerance: f64,
    pub tolerance: f64,
    pub stepover: f64,
    pub angle: f64,
    pub feed: f64,
    pub plunge_feed: f64,
    pub both_ways: bool,
    pub containment: crate::AdaptiveContainment,
    pub additional_offset: f64,
}
impl ParallelParams {
    pub fn retraction_policy(self) -> ScallopRetraction {
        if self.scallop {
            self.scallop_retraction
        } else {
            self.retraction
        }
    }
}

impl Default for ParallelParams {
    fn default() -> Self {
        Self {
            scallop: false,
            scallop_order: ScallopOrder::OutsideIn,
            scallop_start: None,
            scallop_retraction: ScallopRetraction::Full,
            retraction: ScallopRetraction::Shortest,
            transition: ParallelTransition::StraightLine,
            safe_distance: 1.,
            scallop_smoothing: false,
            scallop_smoothing_tolerance: 0.01,
            tolerance: 0.02,
            stepover: 0.5,
            angle: 0.,
            feed: 500.,
            plunge_feed: 100.,
            both_ways: true,
            containment: crate::AdaptiveContainment::Inside,
            additional_offset: 0.,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn parallel_defaults_and_project_operation_roundtrip() {
        let params: ParallelParams = serde_json::from_str("{}").unwrap();
        assert_eq!(params, ParallelParams::default());
        assert!(!params.scallop, "older operations retain parallel rows");
        assert_eq!(params.scallop_order, ScallopOrder::OutsideIn);
        for order in [
            ScallopOrder::OutsideIn,
            ScallopOrder::InsideOut,
            ScallopOrder::Shortest,
        ] {
            let configured = ParallelParams {
                scallop_order: order,
                ..params
            };
            let restored: ParallelParams =
                serde_json::from_str(&serde_json::to_string(&configured).unwrap()).unwrap();
            assert_eq!(restored.scallop_order, order);
        }
        assert_eq!(params.retraction_policy(), ScallopRetraction::Shortest);
        assert_eq!(params.transition, ParallelTransition::StraightLine);
        for transition in ParallelTransition::ALL {
            let configured = ParallelParams {
                transition,
                ..params
            };
            let serialized = serde_json::to_string(&configured).unwrap();
            let restored: ParallelParams = serde_json::from_str(&serialized).unwrap();
            assert_eq!(restored.transition, transition);
        }
        assert!(!params.scallop_smoothing);
        let legacy: ParallelParams = serde_json::from_str(
            r#"{"scallop":true,"scallop_retraction":"minimum","scallop_safe_distance":0.8,"stay_down":true,"lift_height":2}"#,
        ).unwrap();
        assert_eq!(legacy.retraction_policy(), ScallopRetraction::Minimum);
        assert_eq!(legacy.safe_distance, 0.8);
        let mut operation = crate::Operation::Parallel(crate::ParallelOp {
            parameter_expressions: Default::default(),
            id: 3,
            tool: 1,
            spindle_rpm: 4000.,
            work_offset: 2,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            params: ParallelParams {
                angle: 37.,
                scallop: true,
                scallop_order: ScallopOrder::Shortest,
                scallop_start: Some([12., -4.]),
                scallop_retraction: ScallopRetraction::Shortest,
                safe_distance: 0.6,
                scallop_smoothing: true,
                scallop_smoothing_tolerance: 0.05,
                stepover: 0.25,
                both_ways: false,
                ..params
            },
        });
        operation.set_id(5);
        operation.set_tool(4);
        operation.set_work_offset(3);
        let json = serde_json::to_string(&operation).unwrap();
        assert_eq!(
            serde_json::from_str::<crate::Operation>(&json).unwrap(),
            operation
        );
        assert_eq!(operation.tools(), vec![4]);
        assert_eq!(operation.work_offset(), 3);
    }
}
