//! Path clipping — trimming an open polyline against a filled region.
//!
//! The planar analogue of "sectioning": keep the parts of a toolpath that lie
//! inside (or outside) a boundary — a stock outline, a containment region, a
//! keep-out zone. This is `cam-geo`'s 2-D slicing primitive; true 3-D Z-slicing
//! of a solid belongs to the kernel/mesh layer, not here.

use i_overlay::core::fill_rule::FillRule;
use i_overlay::float::clip::FloatClip;
use i_overlay::string::clip::ClipRule;

use crate::{polygon, GeoError, Polygon, Polyline, GRID_SCALE};

/// Clip a polyline against a set of filled regions, keeping the portions that
/// fall inside them (`keep_inside = true`) or outside them (`keep_inside =
/// false`).
///
/// The polyline is cut wherever it crosses a region boundary, so the result is a
/// set of sub-polylines. Segments lying exactly on a boundary are excluded from
/// the result. An invalid polyline (fewer than two points) yields an empty
/// result.
pub fn clip_path(
    path: &Polyline,
    regions: &[Polygon],
    keep_inside: bool,
) -> Result<Vec<Polyline>, GeoError> {
    if !path.is_valid() || regions.is_empty() {
        return Ok(Vec::new());
    }
    let clip_region = polygon::to_shapes(regions);
    // `invert = false` keeps the portion inside the clip region; inverting keeps
    // the outside. Boundary segments are left out either way.
    let clip_rule = ClipRule {
        invert: !keep_inside,
        boundary_included: false,
    };
    let result = path.points().clip_by_fixed_scale(
        &clip_region,
        FillRule::NonZero,
        clip_rule,
        GRID_SCALE,
    )?;
    Ok(result.into_iter().map(Polyline::new).collect())
}
