//! Boolean set operations on filled regions.
//!
//! All three run on the fixed integer grid with the non-zero fill rule, so a
//! region's orientation (CCW outer, CW holes) is honoured — holes correctly
//! subtract from their enclosing boundary.

use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::scale::FixedScaleFloatOverlay;

use crate::{polygon, GeoError, Polygon, GRID_SCALE};

fn boolean(
    subject: &[Polygon],
    clip: &[Polygon],
    rule: OverlayRule,
) -> Result<Vec<Polygon>, GeoError> {
    let subj = polygon::to_shapes(subject);
    let clp = polygon::to_shapes(clip);
    let result = subj.overlay_with_fixed_scale(&clp, rule, FillRule::NonZero, GRID_SCALE)?;
    Ok(polygon::from_shapes(result))
}

/// Union: the area covered by `subject` **or** `clip`. Overlapping regions merge
/// into one; disjoint regions stay separate in the result.
pub fn union(subject: &[Polygon], clip: &[Polygon]) -> Result<Vec<Polygon>, GeoError> {
    boolean(subject, clip, OverlayRule::Union)
}

/// Intersection: the area covered by `subject` **and** `clip`.
pub fn intersection(subject: &[Polygon], clip: &[Polygon]) -> Result<Vec<Polygon>, GeoError> {
    boolean(subject, clip, OverlayRule::Intersect)
}

/// Difference: the area of `subject` with `clip` removed (`subject − clip`).
pub fn difference(subject: &[Polygon], clip: &[Polygon]) -> Result<Vec<Polygon>, GeoError> {
    boolean(subject, clip, OverlayRule::Difference)
}
