//! Path stroking — inflating an open polyline into the closed region it sweeps.
//!
//! This is the standard "open-path offset": a tool of radius `r` moving along a
//! centreline removes exactly the polyline **stroked** by `r` — a filled ribbon
//! of width `2r` with rounded (or flat/square) ends. It is the material-removal
//! footprint of a move, and the geometry behind slots and engraving.
//!
//! A true *single-sided, trimmed* offset (open polyline → open polyline offset
//! to one side) is a different, more specialised operation; it is deferred until
//! `cam-toolpath` (P3) pins down its exact requirements.

use i_overlay::mesh::stroke::offset::StrokeOffset;
use i_overlay::mesh::style::{LineCap, StrokeStyle};

use crate::offset::ROUND_RATIO;
use crate::{polygon, GeoError, JoinStyle, Point, Polygon, Polyline, GRID_SCALE};

/// How the ends of a stroked open path are capped.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CapStyle {
    /// Cut flat exactly at the end point — the ribbon stops at the path ends.
    Butt,
    /// A semicircular cap of the tool radius, centred on the end point. This is
    /// the physically correct cap for a round tool that plunges and retracts.
    Round,
    /// Cut flat, but extended past the end point by the tool radius.
    Square,
}

impl CapStyle {
    fn to_line_cap(self) -> LineCap<Point> {
        match self {
            CapStyle::Butt => LineCap::Butt,
            CapStyle::Round => LineCap::Round(ROUND_RATIO),
            CapStyle::Square => LineCap::Square,
        }
    }
}

/// Stroke an open polyline by a tool of radius `radius`, producing the closed
/// region it sweeps.
///
/// `radius` is the tool radius (the ribbon's half-width); the stroked region is
/// `2 · radius` wide. `cap` controls the ends, `join` the corners. The result
/// may be several polygons (a path that loops back can enclose a void, yielding
/// a hole). Stroking an invalid path (fewer than two points) or a non-positive
/// radius yields an empty result.
pub fn stroke_path(
    path: &Polyline,
    radius: f64,
    cap: CapStyle,
    join: JoinStyle,
) -> Result<Vec<Polygon>, GeoError> {
    if !path.is_valid() || radius <= 0.0 {
        return Ok(Vec::new());
    }
    let style = StrokeStyle {
        width: 2.0 * radius,
        start_cap: cap.to_line_cap(),
        end_cap: cap.to_line_cap(),
        join: join.to_line_join(),
    };
    let pts = path.points();
    let result = pts.stroke_fixed_scale(style, false, GRID_SCALE)?;
    Ok(polygon::from_shapes(result))
}
