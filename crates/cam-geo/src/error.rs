//! Error type for geometry operations.

use core::fmt;

use i_overlay::float::scale::FixedScaleOverlayError;

/// Anything that can go wrong constructing or operating on CAM geometry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GeoError {
    /// A contour had fewer than three distinct vertices, so it bounds no area.
    DegenerateContour,
    /// An operation received no input geometry to work on.
    EmptyInput,
    /// The requested geometry did not fit the fixed integer grid at
    /// [`crate::GRID_MM`] — typically because it spans more than the working
    /// envelope. Carries a human-readable reason from the geometry engine.
    OutOfGrid(&'static str),
}

impl fmt::Display for GeoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GeoError::DegenerateContour => {
                write!(f, "contour has fewer than three distinct vertices")
            }
            GeoError::EmptyInput => write!(f, "operation received no input geometry"),
            GeoError::OutOfGrid(why) => {
                write!(f, "geometry does not fit the fixed integer grid: {why}")
            }
        }
    }
}

impl std::error::Error for GeoError {}

impl From<FixedScaleOverlayError> for GeoError {
    fn from(e: FixedScaleOverlayError) -> Self {
        // The engine's error variants collapse to one meaning for us: the
        // geometry can't be represented on our fixed grid.
        let why = match e {
            FixedScaleOverlayError::ScaleTooLarge => "grid too fine for the geometry's extent",
            FixedScaleOverlayError::ScaleNonPositive => "non-positive grid scale",
            FixedScaleOverlayError::ScaleNotFinite => "non-finite grid scale",
        };
        GeoError::OutOfGrid(why)
    }
}
