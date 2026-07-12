//! # cam-import — CAD geometry into CAM contours
//!
//! Reads a CAD drawing and hands the pipeline what it needs: closed
//! [`Polygon`](cam_geo::Polygon) regions (outer boundaries with nested holes),
//! ready to profile, pocket, or drill. The only format at first is **ASCII
//! DXF**; the reader is deliberately small and dependency-free (see
//! [`dxf`](crate::dxf)).
//!
//! The interesting work is not the parsing — DXF is a flat stream of group
//! pairs — but turning a *soup of disconnected entities* into machinable
//! geometry: flattening arcs and polyline bulges, **chaining** endpoint-adjacent
//! segments into closed loops within a weld tolerance, and **nesting** holes
//! inside their enclosing boundary by containment.
//!
//! ```no_run
//! let import = cam_import::read_dxf_file("part.dxf", &Default::default()).unwrap();
//! for region in &import.regions {
//!     // feed region.outer() / region.holes() to a strategy
//! }
//! ```

mod build;
pub mod dxf;

use std::fmt;
use std::path::Path;

use cam_geo::{Polygon, Polyline};

/// Tolerances that govern how geometry is reconstructed, in millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImportOptions {
    /// Endpoints closer than this are treated as coincident when chaining
    /// fragments into closed contours.
    pub weld_tolerance: f64,
    /// Maximum chord deviation when flattening arcs and bulges to line segments.
    pub chord_tolerance: f64,
}

impl Default for ImportOptions {
    fn default() -> Self {
        Self {
            weld_tolerance: 1.0e-3,  // 1 µm
            chord_tolerance: 1.0e-2, // 10 µm
        }
    }
}

/// The result of importing a drawing.
#[derive(Clone, Debug, Default)]
pub struct Import {
    /// Closed filled regions (outer boundary + holes), ready for CAM.
    pub regions: Vec<Polygon>,
    /// Open chains that could not be closed — kept for inspection, not machined
    /// as regions.
    pub open_chains: Vec<Polyline>,
    /// Non-fatal notes: unsupported entities skipped, chains dropped, etc.
    pub warnings: Vec<String>,
}

/// Why an import failed outright.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ImportError {
    /// The file could not be read.
    Io(String),
    /// No supported entities were found in the drawing.
    NoEntities,
}

impl fmt::Display for ImportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ImportError::Io(e) => write!(f, "could not read DXF file: {e}"),
            ImportError::NoEntities => {
                write!(
                    f,
                    "no supported geometry (LINE/ARC/CIRCLE/LWPOLYLINE) found"
                )
            }
        }
    }
}

impl std::error::Error for ImportError {}

/// Import geometry from DXF text.
pub fn read_dxf_str(text: &str, options: &ImportOptions) -> Result<Import, ImportError> {
    let (entities, skipped) = dxf::read_entities(text);
    if entities.is_empty() {
        return Err(ImportError::NoEntities);
    }

    let (regions, open_chains, mut warnings) =
        build::assemble(&entities, options.weld_tolerance, options.chord_tolerance);

    // One warning per distinct unsupported entity type.
    let mut kinds = skipped;
    kinds.sort();
    kinds.dedup();
    for kind in kinds {
        warnings.push(format!("ignored unsupported entity: {kind}"));
    }

    Ok(Import {
        regions,
        open_chains,
        warnings,
    })
}

/// Import geometry from a DXF file on disk.
pub fn read_dxf_file(
    path: impl AsRef<Path>,
    options: &ImportOptions,
) -> Result<Import, ImportError> {
    let text = std::fs::read_to_string(path).map_err(|e| ImportError::Io(e.to_string()))?;
    read_dxf_str(&text, options)
}
