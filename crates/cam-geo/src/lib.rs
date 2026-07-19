//! # cam-geo — robust 2D toolpath geometry
//!
//! The kernel-independent heart of OpenCAMStudio's CAM engine. Everything here
//! is pure planar geometry in **millimetres**: closed contours, filled regions
//! with holes, robust polygon **offsetting** (tool-radius compensation), boolean
//! set operations, point containment, and arc flattening. No dependency on the
//! solid kernel — toolpath geometry must never reach through to `truck`/OCCT, so
//! that the kernel stays swappable (see `ARCHITECTURE.md`).
//!
//! ## Robustness model
//!
//! Boolean and offset operations run on a fixed **integer grid** via `i_overlay`.
//! We pin the float→integer scale explicitly ([`GRID_MM`]) rather than letting the
//! library derive it from each input's bounding box, so a given shape snaps and
//! offsets **identically regardless of where it sits in space** — a determinism
//! invariant the rest of the pipeline relies on.
//!
//! ## The seam
//!
//! Downstream crates speak only in [`Point`], [`Contour`], and [`Polygon`]; the
//! `i_overlay` types never cross this boundary. Our [`Point`] implements
//! `i_overlay`'s point trait, so conversions in and out are zero-copy.
//!
//! ```
//! use cam_geo::{offset, Contour, JoinStyle, Point, Polygon};
//!
//! // A 10 mm square.
//! let square = Polygon::new(Contour::new(vec![
//!     Point::new(0.0, 0.0),
//!     Point::new(10.0, 0.0),
//!     Point::new(10.0, 10.0),
//!     Point::new(0.0, 10.0),
//! ]))
//! .unwrap();
//!
//! // Grow it outward by a 2 mm tool radius with rounded corners.
//! let grown = offset(&[square], 2.0, JoinStyle::Round).unwrap();
//! assert_eq!(grown.len(), 1);
//! ```

mod arc;
mod arcfit;
mod boolean;
mod clip;
mod contour;
mod error;
mod offset;
mod point;
mod polygon;
mod polyline;
mod stroke;
mod toolprofile;

pub use arc::Arc;
pub use arcfit::{fit_arcs, PathSeg};
pub use boolean::{difference, intersection, union};
pub use clip::clip_path;
pub use contour::Contour;
pub use error::GeoError;
pub use offset::{offset, JoinStyle};
pub use point::Point;
pub use polygon::{Containment, Polygon};
pub use polyline::Polyline;
pub use toolprofile::{
    generatrix, vtip_depth_for_half_width, vtip_half_width, vtip_max_depth, BottomShape,
    GeneratrixSpec, Profile2D, ProfileSeg, SegShape,
};
pub use stroke::{stroke_path, CapStyle};

/// The internal integer grid resolution, in millimetres.
///
/// All boolean/offset math snaps coordinates to a lattice of this spacing
/// (0.1 µm). Finer than any milling tolerance, while leaving the `i32`
/// engine a working envelope of roughly ±200 m — ample for any machine bed.
pub const GRID_MM: f64 = 1.0e-4;

/// Float→integer scale handed to `i_overlay` (`1 / GRID_MM`). Fixed, so results
/// are translation-invariant.
pub(crate) const GRID_SCALE: f64 = 1.0 / GRID_MM;
