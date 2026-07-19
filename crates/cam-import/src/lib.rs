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

/// Import geometry from DXF text (the in-crate ASCII reader — used for the bundled
/// sample and tests; real file imports go through [`read_cad_file`]).
pub fn read_dxf_str(text: &str, options: &ImportOptions) -> Result<Import, ImportError> {
    let (entities, skipped) = dxf::read_entities(text);
    assemble_import(entities, skipped, options)
}

/// Import geometry from a DXF file on disk (in-crate ASCII reader).
pub fn read_dxf_file(
    path: impl AsRef<Path>,
    options: &ImportOptions,
) -> Result<Import, ImportError> {
    let text = std::fs::read_to_string(path).map_err(|e| ImportError::Io(e.to_string()))?;
    read_dxf_str(&text, options)
}

/// Read a CAD file's **raw entity inventory** — the typed entities plus the names of any
/// unsupported/skipped ones — *without* assembling regions. This is the tool-import
/// **vet** path (`TOOLING_PLAN.md` §5): the vet must reject a drawing containing anything
/// other than LINE/ARC, which needs to see every entity, not the chained geometry that
/// [`read_cad_file`] returns. DXF (ASCII or binary) or DWG, chosen by extension.
pub fn read_cad_entities(
    path: impl AsRef<Path>,
) -> Result<(Vec<dxf::Entity>, Vec<String>), ImportError> {
    let doc = read_cad_document(path.as_ref())?;
    Ok(map_acad_entities(&doc))
}

/// The entity inventory of an in-memory ASCII-DXF string (the in-crate reader) — the
/// string-level companion to [`read_cad_entities`], for the bundled sample and tests.
pub fn read_dxf_entities(text: &str) -> (Vec<dxf::Entity>, Vec<String>) {
    dxf::read_entities(text)
}

/// Import geometry from a CAD file (`.dxf` — ASCII or binary — or `.dwg`) via
/// **acadrust**. This is the user-facing import path; the supported entities
/// (LINE / CIRCLE / ARC / LWPOLYLINE) are mapped into the same chaining +
/// hole-nesting pipeline as the ASCII reader. The format is chosen by extension.
pub fn read_cad_file(
    path: impl AsRef<Path>,
    options: &ImportOptions,
) -> Result<Import, ImportError> {
    let path = path.as_ref();
    let doc = read_cad_document(path)?;
    let (entities, skipped) = map_acad_entities(&doc);
    assemble_import(entities, skipped, options)
}

/// Read a `.dxf`/`.dwg` into an acadrust document, picking the reader by extension.
fn read_cad_document(path: &Path) -> Result<acadrust::CadDocument, ImportError> {
    let io = |e: acadrust::DxfError| ImportError::Io(e.to_string());
    let is_dwg = path
        .extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("dwg"));
    if is_dwg {
        let mut reader = acadrust::io::dwg::DwgReader::from_file(path).map_err(io)?;
        reader.read().map_err(io)
    } else {
        acadrust::DxfReader::from_file(path)
            .map_err(io)?
            .read()
            .map_err(io)
    }
}

/// Map the acadrust entities we support onto our [`dxf::Entity`] model; the rest
/// are recorded (by variant name) as skipped, exactly like the ASCII reader.
fn map_acad_entities(doc: &acadrust::CadDocument) -> (Vec<dxf::Entity>, Vec<String>) {
    use acadrust::entities::EntityType;
    let mut entities = Vec::new();
    let mut skipped = Vec::new();
    for entity in doc.entities() {
        match entity {
            EntityType::Line(l) => entities.push(dxf::Entity::Line {
                a: (l.start.x, l.start.y),
                b: (l.end.x, l.end.y),
            }),
            EntityType::Circle(c) => entities.push(dxf::Entity::Circle {
                center: (c.center.x, c.center.y),
                radius: c.radius,
            }),
            // **acadrust reports arc angles in radians**, while [`dxf::Entity::Arc`]
            // carries degrees (as the DXF file itself stores them, group codes 50/51)
            // and every downstream consumer converts from degrees. Without this
            // conversion a 30°→330° arc arrives as 0.52°→5.76° — a 5° sliver instead
            // of a 300° sweep — silently mangling every arc that comes through the
            // real file-open path.
            EntityType::Arc(a) => entities.push(dxf::Entity::Arc {
                center: (a.center.x, a.center.y),
                radius: a.radius,
                start_deg: a.start_angle.to_degrees(),
                end_deg: a.end_angle.to_degrees(),
            }),
            EntityType::LwPolyline(p) => entities.push(dxf::Entity::LwPolyline {
                closed: p.is_closed,
                verts: p
                    .vertices
                    .iter()
                    .map(|v| (v.location.x, v.location.y, v.bulge))
                    .collect(),
            }),
            other => skipped.push(acad_variant_name(other)),
        }
    }
    (entities, skipped)
}

/// The bare variant name of an acadrust entity (for skipped-entity warnings),
/// derived from its `Debug` form so we needn't enumerate all 41 types.
fn acad_variant_name(entity: &acadrust::entities::EntityType) -> String {
    let dbg = format!("{entity:?}");
    dbg.split(['(', ' ', '{'])
        .next()
        .unwrap_or("entity")
        .to_string()
}

/// Run the chaining + hole-nesting pipeline on parsed entities, attaching one
/// warning per distinct skipped entity type. Shared by every reader.
fn assemble_import(
    entities: Vec<dxf::Entity>,
    skipped: Vec<String>,
    options: &ImportOptions,
) -> Result<Import, ImportError> {
    if entities.is_empty() {
        return Err(ImportError::NoEntities);
    }

    let (regions, open_chains, mut warnings) =
        build::assemble(&entities, options.weld_tolerance, options.chord_tolerance);

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

#[cfg(test)]
mod entity_inventory_tests {
    use super::*;

    // A tiny ASCII-DXF ENTITIES section with a LINE, an ARC, and an LWPOLYLINE.
    const MIXED: &str = "0\nSECTION\n2\nENTITIES\n\
        0\nLINE\n10\n0.0\n20\n0.0\n11\n10.0\n21\n0.0\n\
        0\nARC\n10\n5.0\n20\n0.0\n40\n5.0\n50\n0.0\n51\n90.0\n\
        0\nLWPOLYLINE\n90\n2\n10\n0.0\n20\n0.0\n10\n1.0\n20\n1.0\n\
        0\nENDSEC\n0\nEOF\n";

    #[test]
    fn read_dxf_entities_surfaces_every_entity_type_for_the_vet() {
        let (entities, _skipped) = read_dxf_entities(MIXED);
        // The tool-import vet needs to *see* the LWPOLYLINE (to reject it), not have it
        // silently chained away.
        assert!(entities.iter().any(|e| matches!(e, dxf::Entity::Line { .. })));
        assert!(entities.iter().any(|e| matches!(e, dxf::Entity::Arc { .. })));
        assert!(
            entities.iter().any(|e| matches!(e, dxf::Entity::LwPolyline { .. })),
            "the polyline is visible to the vet, which will reject the whole drawing"
        );
    }

    #[test]
    fn a_line_and_arc_only_drawing_has_nothing_the_vet_rejects() {
        let clean = "0\nSECTION\n2\nENTITIES\n\
            0\nLINE\n10\n0.0\n20\n0.0\n11\n6.0\n21\n0.0\n\
            0\nARC\n10\n6.0\n20\n0.0\n40\n2.0\n50\n0.0\n51\n90.0\n\
            0\nENDSEC\n0\nEOF\n";
        let (entities, skipped) = read_dxf_entities(clean);
        assert_eq!(entities.len(), 2);
        assert!(entities
            .iter()
            .all(|e| matches!(e, dxf::Entity::Line { .. } | dxf::Entity::Arc { .. })));
        assert!(skipped.is_empty(), "no unsupported entities skipped");
    }
}
