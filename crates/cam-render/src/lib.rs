//! # cam-render — viewport scene + wgpu renderer
//!
//! Two layers, split so the logic stays testable without a GPU:
//!
//! - The **[`Scene`]** model and its mapping to GPU [`Vertex`]es — colored
//!   polylines for the part outline and the toolpath backplot. Pure data,
//!   unit-tested; builds with **no default features**.
//! - The **`gpu` feature**'s renderer, which uploads those vertices and draws
//!   them with `wgpu`. It needs a graphics stack, so it is compiled by CI and on
//!   the desktop, not exercised in headless unit tests.

mod camera;
mod scene;

pub use camera::top_view;
pub use scene::{Color, LineStrip, Scene, Vertex, CUT, PART, PLUNGE, RAPID};

#[cfg(feature = "gpu")]
mod gpu;
#[cfg(feature = "gpu")]
pub use gpu::LineRenderer;
