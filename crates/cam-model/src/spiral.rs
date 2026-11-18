//! Controls for projected spiral finishing in setup coordinates and millimetres.
use crate::{AdaptiveContainment, ScallopRetraction};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpiralMode {
    #[default]
    Spiral,
    SpiralWithCircles,
    ConcentricCircles,
}

impl std::fmt::Display for SpiralMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Spiral => "Spiral",
            Self::SpiralWithCircles => "Spiral with circles",
            Self::ConcentricCircles => "Concentric circles",
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpiralCutDirection {
    Climb,
    Conventional,
    #[default]
    BothWays,
}

impl std::fmt::Display for SpiralCutDirection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Climb => "Climb",
            Self::Conventional => "Conventional",
            Self::BothWays => "Both ways",
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpiralOrder {
    #[default]
    InsideOut,
    OutsideIn,
    Shortest,
}

impl std::fmt::Display for SpiralOrder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::InsideOut => "Inside → out",
            Self::OutsideIn => "Outside → in",
            Self::Shortest => "Shortest machining distance",
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpiralMilling {
    #[default]
    Both,
    Up,
    Down,
}

impl std::fmt::Display for SpiralMilling {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Both => "Both",
            Self::Up => "Up milling",
            Self::Down => "Down milling",
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpiralBoundary {
    #[default]
    BoundingBox,
    Silhouette,
    StockBox,
}

impl std::fmt::Display for SpiralBoundary {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::BoundingBox => "Bounding box",
            Self::Silhouette => "Selected silhouette",
            Self::StockBox => "Stock bounding box",
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SpiralParams {
    /// A picked setup-XY point, or the center of the selected geometry bounds.
    pub center: Option<[f64; 2]>,
    pub mode: SpiralMode,
    pub order: SpiralOrder,
    /// Retain the radial order when shortest-distance linking is selected.
    pub link_inside_out: bool,
    pub clockwise: bool,
    pub cut_direction: SpiralCutDirection,
    pub inner_limit: f64,
    /// Zero lets the machining boundary determine the outer limit.
    pub outer_limit: f64,
    pub stepover: f64,
    pub tolerance: f64,
    pub feed: f64,
    pub plunge_feed: f64,
    pub boundary: SpiralBoundary,
    pub containment: AdaptiveContainment,
    pub contact_only: bool,
    pub additional_offset: f64,
    pub from_slope: f64,
    pub to_slope: f64,
    pub milling: SpiralMilling,
    pub multiple_depths: bool,
    /// Thickness to remove incrementally above the final surface.
    pub axial_offset: f64,
    pub max_stepdown: f64,
    /// Zero calculates the number of finishing layers from the maximum stepdown.
    pub stepdowns: u32,
    pub order_by_depth: bool,
    pub retraction: ScallopRetraction,
    pub safe_distance: f64,
    pub max_stay_down_distance: f64,
    pub smoothing: bool,
    pub smoothing_tolerance: f64,
}

impl Default for SpiralParams {
    fn default() -> Self {
        Self {
            center: None,
            mode: SpiralMode::Spiral,
            order: SpiralOrder::InsideOut,
            link_inside_out: false,
            clockwise: false,
            cut_direction: SpiralCutDirection::BothWays,
            inner_limit: 0.,
            outer_limit: 0.,
            stepover: 0.5,
            tolerance: 0.02,
            feed: 500.,
            plunge_feed: 100.,
            boundary: SpiralBoundary::BoundingBox,
            containment: AdaptiveContainment::Inside,
            contact_only: true,
            additional_offset: 0.,
            from_slope: 0.,
            to_slope: 90.,
            milling: SpiralMilling::Both,
            multiple_depths: false,
            axial_offset: 1.,
            max_stepdown: 0.5,
            stepdowns: 0,
            order_by_depth: true,
            retraction: ScallopRetraction::Full,
            safe_distance: 1.,
            max_stay_down_distance: 25.,
            smoothing: false,
            smoothing_tolerance: 0.01,
        }
    }
}
