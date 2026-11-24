//! Persistent CAD identity and exact topology. Geometry is millimetres in asset-local
//! coordinates; display meshes are never the source of machining boundaries.
use crate::{Heights, Operation, Side};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

pub type AssetId = u64;
pub type BodyId = u64;
pub type InstanceId = u64;
pub type EntityId = u64;
const FRAME_REFERENCE_TOLERANCE: f64 = 1e-9;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CadFormat {
    Step,
    Iges,
    Dxf,
    Dwg,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SourceUnits {
    Millimetres,
    Inches,
    Metres,
    Unknown,
}
impl SourceUnits {
    pub fn millimetre_scale(self) -> Option<f64> {
        match self {
            Self::Millimetres => Some(1.0),
            Self::Inches => Some(25.4),
            Self::Metres => Some(1000.0),
            Self::Unknown => None,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidTransform {
    pub rotation: [[f64; 3]; 3],
    pub translation: [f64; 3],
}
impl Default for RigidTransform {
    fn default() -> Self {
        Self {
            rotation: [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            translation: [0.; 3],
        }
    }
}
impl RigidTransform {
    pub fn transform_vector(&self, p: [f64; 3]) -> [f64; 3] {
        self.rotation
            .map(|row| row.into_iter().zip(p).map(|(a, b)| a * b).sum())
    }
    pub fn transform_point(&self, p: [f64; 3]) -> [f64; 3] {
        let p = self.transform_vector(p);
        std::array::from_fn(|i| p[i] + self.translation[i])
    }
    pub fn inverse_point(&self, p: [f64; 3]) -> [f64; 3] {
        let p = std::array::from_fn::<_, 3, _>(|i| p[i] - self.translation[i]);
        std::array::from_fn(|i| (0..3).map(|j| self.rotation[j][i] * p[j]).sum())
    }
    pub fn is_rigid(&self) -> bool {
        if !self
            .translation
            .iter()
            .chain(self.rotation.iter().flatten())
            .all(|x| x.is_finite())
        {
            return false;
        }
        for i in 0..3 {
            for j in 0..3 {
                let dot: f64 = (0..3)
                    .map(|k| self.rotation[i][k] * self.rotation[j][k])
                    .sum();
                if (dot - if i == j { 1. } else { 0. }).abs() > 1e-8 {
                    return false;
                }
            }
        }
        let m = self.rotation;
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        (det - 1.).abs() < 1e-8
    }
}

/// Persisted work coordinate frame. `world_from_setup` carries rotation only;
/// the setup's `origin` remains the datum point and is subtracted by posting.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SetupFrame {
    #[serde(default)]
    pub world_from_setup: RigidTransform,
    #[serde(default)]
    pub references: FrameReferences,
}
impl SetupFrame {
    pub fn to_setup_vector(&self, world: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|i| {
            (0..3)
                .map(|j| self.world_from_setup.rotation[j][i] * world[j])
                .sum()
        })
    }
    pub fn to_world_vector(&self, local: [f64; 3]) -> [f64; 3] {
        self.world_from_setup.transform_vector(local)
    }
    pub fn validate(&self) -> Result<(), String> {
        if self
            .world_from_setup
            .translation
            .iter()
            .any(|x| x.abs() > 1e-9)
        {
            return Err("setup frame must not contain a translation; use the setup origin".into());
        }
        if !self.world_from_setup.is_rigid() {
            return Err("setup axes must form a right-handed orthonormal frame".into());
        }
        Ok(())
    }
    /// Build the frame from X/Y directions. Z is derived as X cross Y.
    pub fn from_xy(x: [f64; 3], y: [f64; 3]) -> Result<Self, String> {
        Self::from_axes(Some(x), Some(y), None)
    }
    /// Build a right-handed orthonormal frame from supplied directions. One missing
    /// axis may be derived; supplied directions are never silently adjusted.
    pub fn from_axes(
        x: Option<[f64; 3]>,
        y: Option<[f64; 3]>,
        z: Option<[f64; 3]>,
    ) -> Result<Self, String> {
        fn unit(v: [f64; 3], name: &str) -> Result<[f64; 3], String> {
            let n = v.iter().map(|x| x * x).sum::<f64>().sqrt();
            if !n.is_finite() || n < 1e-10 {
                return Err(format!("{name} reference is degenerate"));
            }
            Ok(v.map(|x| x / n))
        }
        fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
            (0..3).map(|i| a[i] * b[i]).sum()
        }
        fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        }
        let (x, y, z) = match (x, y, z) {
            (Some(x), Some(y), None) => {
                let x = unit(x, "X")?;
                let y = unit(y, "Y")?;
                if dot(x, y).abs() > FRAME_REFERENCE_TOLERANCE {
                    return Err("X and Y references must be perpendicular".into());
                }
                (x, y, unit(cross(x, y), "Z")?)
            }
            (Some(x), None, Some(z)) => {
                let x = unit(x, "X")?;
                let z = unit(z, "Z")?;
                if dot(x, z).abs() > FRAME_REFERENCE_TOLERANCE {
                    return Err("X and Z references must be perpendicular".into());
                }
                (x, unit(cross(z, x), "Y")?, z)
            }
            (None, Some(y), Some(z)) => {
                let y = unit(y, "Y")?;
                let z = unit(z, "Z")?;
                if dot(y, z).abs() > FRAME_REFERENCE_TOLERANCE {
                    return Err("Y and Z references must be perpendicular".into());
                }
                (unit(cross(y, z), "X")?, y, z)
            }
            (Some(x), Some(y), Some(z)) => {
                let (x, y, z) = (unit(x, "X")?, unit(y, "Y")?, unit(z, "Z")?);
                if dot(x, y).abs() > FRAME_REFERENCE_TOLERANCE {
                    return Err("X and Y references must be perpendicular".into());
                }
                if dot(x, z).abs() > FRAME_REFERENCE_TOLERANCE {
                    return Err("X and Z references must be perpendicular".into());
                }
                if dot(y, z).abs() > FRAME_REFERENCE_TOLERANCE {
                    return Err("Y and Z references must be perpendicular".into());
                }
                if dot(cross(x, y), z) < 1. - FRAME_REFERENCE_TOLERANCE {
                    return Err("selected axes do not form a right-handed frame".into());
                }
                (x, y, z)
            }
            _ => return Err("select at least two perpendicular axis references".into()),
        };
        let frame = Self {
            world_from_setup: RigidTransform {
                rotation: [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]],
                translation: [0.; 3],
            },
            references: FrameReferences::default(),
        };
        frame.validate()?;
        Ok(frame)
    }
    pub fn from_model_instance(state: &GeometryState, instance_id: u64) -> Result<Self, String> {
        let instance = state
            .instance(instance_id)
            .ok_or("model instance missing")?;
        let mut f = Self::default();
        f.world_from_setup.rotation = instance.placement.rotation;
        f.references.model_instance = Some(instance_id);
        f.validate()?;
        Ok(f)
    }
    pub fn from_model_coordinate_system(
        state: &GeometryState,
        instance_id: u64,
    ) -> Result<Self, String> {
        Self::from_model_instance(state, instance_id)
    }
    /// Resolve selected straight edges to world directions, apply explicit flips,
    /// and validate/derive a right-handed frame. References are retained for editing.
    pub fn from_edge_references(
        state: &GeometryState,
        x: Option<FrameAxisReference>,
        y: Option<FrameAxisReference>,
        z: Option<FrameAxisReference>,
    ) -> Result<Self, String> {
        Self::from_references(
            state,
            FrameReferences {
                x,
                y,
                z,
                ..Default::default()
            },
        )
    }
    /// Stock edge directions are captured in world coordinates, so resizing or
    /// reorienting the stock cannot make a selected reference follow itself.
    pub fn from_references(
        state: &GeometryState,
        references: FrameReferences,
    ) -> Result<Self, String> {
        Self::default().resolve_references(state, references)
    }
    /// Resolve axis picks using this frame's orientation for unassigned axes.
    /// A single pick fixes its axis and projects an existing transverse axis
    /// onto its perpendicular plane; two or three picks retain strict validation.
    pub fn resolve_references(
        &self,
        state: &GeometryState,
        references: FrameReferences,
    ) -> Result<Self, String> {
        self.validate()?;
        fn direction(
            state: &GeometryState,
            axis: Option<&FrameAxisReference>,
            label: &str,
        ) -> Result<Option<[f64; 3]>, String> {
            let Some(reference) = axis else {
                return Ok(None);
            };
            if reference.entity.kind != EntityKind::Edge {
                return Err(format!("{label} reference must be a straight edge"));
            }
            if !state.entity_exists(&reference.entity) {
                return Err(format!("{label} reference is stale or missing"));
            }
            let asset = state
                .asset(reference.entity.asset_id)
                .ok_or_else(|| format!("{label} reference asset is missing"))?;
            let edge = asset
                .topology
                .edges
                .iter()
                .find(|e| e.id == reference.entity.id && e.body_id == reference.entity.body_id)
                .ok_or_else(|| format!("{label} reference edge is missing"))?;
            let CadCurve::Line { start, end } = &edge.curve else {
                return Err(format!("{label} reference must be straight"));
            };
            let instance = state
                .instance(reference.entity.instance_id)
                .ok_or_else(|| format!("{label} reference instance is missing"))?;
            let mut v = instance
                .placement
                .transform_vector(std::array::from_fn(|i| end[i] - start[i]));
            if reference.flipped {
                v = v.map(|c| -c)
            }
            Ok(Some(v))
        }
        let mut axes = [None; 3];
        for (i, (edge, label)) in [
            (&references.x, "X"),
            (&references.y, "Y"),
            (&references.z, "Z"),
        ]
        .into_iter()
        .enumerate()
        {
            if edge.is_some() && references.stock_axes[i].is_some() {
                return Err(format!("{label} has both a model and stock reference"));
            }
            axes[i] = direction(state, edge.as_ref(), label)?.or_else(|| {
                references.stock_axes[i].map(|v| {
                    if references.stock_flipped[i] {
                        v.map(|c| -c)
                    } else {
                        v
                    }
                })
            });
        }
        let count = axes.iter().filter(|axis| axis.is_some()).count();
        if count == 1 {
            let primary = axes.iter().position(Option::is_some).unwrap();
            let direction = axes[primary].unwrap();
            let length = direction.iter().map(|v| v * v).sum::<f64>().sqrt();
            if !length.is_finite() || length < 1e-10 {
                return Err(format!(
                    "{} reference is degenerate",
                    ["X", "Y", "Z"][primary]
                ));
            }
            let direction = direction.map(|v| v / length);
            axes[primary] = Some(direction);
            for secondary in [(primary + 1) % 3, (primary + 2) % 3] {
                let current =
                    self.to_world_vector(std::array::from_fn(
                        |i| {
                            if i == secondary {
                                1.
                            } else {
                                0.
                            }
                        },
                    ));
                let dot: f64 = (0..3).map(|i| current[i] * direction[i]).sum();
                let projected: [f64; 3] = std::array::from_fn(|i| current[i] - dot * direction[i]);
                let length = projected.iter().map(|v| v * v).sum::<f64>().sqrt();
                if length > 1e-10 {
                    axes[secondary] = Some(projected.map(|v| v / length));
                    break;
                }
            }
        }
        let mut frame = if count == 0 {
            self.clone()
        } else {
            Self::from_axes(axes[0], axes[1], axes[2])?
        };
        frame.references = references;
        Ok(frame)
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FrameAxisReference {
    pub entity: EntityRef,
    #[serde(default)]
    pub flipped: bool,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum FrameOrigin {
    #[default]
    ModelOrigin,
    Explicit([f64; 3]),
    SelectedPoint([f64; 3]),
    StockBoxPoint([f64; 3]),
    ModelBoxPoint([f64; 3]),
    TwoPoints([f64; 3]),
}
impl FrameOrigin {
    /// Update coordinates after a frame change while retaining their source.
    pub fn with_coordinates(&self, coordinates: [f64; 3]) -> Self {
        match self {
            Self::ModelOrigin => Self::ModelOrigin,
            Self::Explicit(_) => Self::Explicit(coordinates),
            Self::SelectedPoint(_) => Self::SelectedPoint(coordinates),
            Self::StockBoxPoint(_) => Self::StockBoxPoint(coordinates),
            Self::ModelBoxPoint(_) => Self::ModelBoxPoint(coordinates),
            Self::TwoPoints(_) => Self::TwoPoints(coordinates),
        }
    }
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FrameReferences {
    /// Explicit world directions captured from stock edges; no CAD entity is needed.
    #[serde(default)]
    pub stock_axes: [Option<[f64; 3]>; 3],
    #[serde(default)]
    pub stock_flipped: [bool; 3],
    #[serde(default)]
    pub x: Option<FrameAxisReference>,
    #[serde(default)]
    pub y: Option<FrameAxisReference>,
    #[serde(default)]
    pub z: Option<FrameAxisReference>,
    #[serde(default)]
    pub origin: FrameOrigin,
    #[serde(default)]
    pub model_instance: Option<u64>,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometryImportOptions {
    pub import_tolerance: f64,
    pub display_tolerance: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeometryAsset {
    #[serde(default)]
    pub import_options: Option<GeometryImportOptions>,
    pub id: AssetId,
    pub revision: u64,
    pub name: String,
    pub source_format: CadFormat,
    /// Original file location for reload. Embedded geometry remains self-contained.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_path: Option<String>,
    /// Shared immutable bytes; history edits never duplicate imported blobs.
    pub source_data: Arc<Vec<u8>>,
    pub source_units: SourceUnits,
    pub backend: String,
    #[serde(default)]
    pub import_diagnostics: Vec<String>,
    /// Source content provenance; a deterministic non-cryptographic fingerprint.
    #[serde(default)]
    pub source_fingerprint: String,
    /// Shared immutable topology; replacement creates a new snapshot/revision.
    pub topology: Arc<TopologySnapshot>,
}
impl PartialEq for GeometryAsset {
    fn eq(&self, other: &Self) -> bool {
        self.import_options == other.import_options
            && self.id == other.id
            && self.revision == other.revision
            && self.name == other.name
            && self.source_format == other.source_format
            && self.source_path == other.source_path
            && self.source_units == other.source_units
            && self.backend == other.backend
            && self.import_diagnostics == other.import_diagnostics
            && self.source_fingerprint == other.source_fingerprint
            && (Arc::ptr_eq(&self.source_data, &other.source_data)
                || self.source_data == other.source_data)
            // Arc's PartialEq does not skip shared allocations for types containing
            // floats. Dirty-state checks otherwise walk every projection triangle
            // on each GUI update, even when both snapshots share the same topology.
            && (Arc::ptr_eq(&self.topology, &other.topology)
                || self.topology == other.topology)
    }
}
impl GeometryAsset {
    pub fn fingerprint_source(bytes: &[u8]) -> String {
        let hash = bytes.iter().fold(0xcbf29ce484222325_u64, |hash, byte| {
            (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
        });
        format!("{hash:016x}")
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometryInstance {
    pub id: InstanceId,
    pub asset_id: AssetId,
    pub body_id: BodyId,
    pub name: String,
    pub placement: RigidTransform,
    pub visible: bool,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EntityKind {
    Body,
    Face,
    Edge,
    Loop,
    Vertex,
    /// Body outline projected into the active setup XY plane.
    Silhouette,
}
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct EntityRef {
    pub asset_id: AssetId,
    pub revision: u64,
    pub instance_id: InstanceId,
    pub body_id: BodyId,
    pub kind: EntityKind,
    pub id: EntityId,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadBody {
    pub id: BodyId,
    pub name: String,
    pub valid_solid: bool,
    pub bounds: [[f64; 3]; 2],
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum CadSurface {
    Plane {
        origin: [f64; 3],
        normal: [f64; 3],
    },
    Cylinder {
        origin: [f64; 3],
        axis: [f64; 3],
        radius: f64,
        inward: bool,
    },
    /// Exact cone metadata; tapered machining requires an operation adapter.
    Cone {
        origin: [f64; 3],
        axis: [f64; 3],
        radius: f64,
        semi_angle: f64,
    },
    Sphere {
        center: [f64; 3],
        radius: f64,
    },
    /// Analytic fillet surface; inspectable, with no 3D surfacing adapter.
    Torus {
        origin: [f64; 3],
        axis: [f64; 3],
        major_radius: f64,
        minor_radius: f64,
    },
    /// Native extrusion, including spline surfaces whose complete pole rows
    /// are axial translations with separable positive rational weights.
    LinearExtrusion {
        axis: [f64; 3],
        /// Local outward surface orientation at each boundary edge's midpoint.
        /// Keeps open chains independent of unrelated face trims.
        edge_sides: Vec<CadWallEdgeSide>,
    },
    Unsupported {
        description: String,
    },
}
impl CadSurface {
    pub fn kind_name(&self) -> &str {
        match self {
            Self::Plane { .. } => "plane",
            Self::Cylinder { .. } => "cylinder",
            Self::Cone { .. } => "cone",
            Self::Sphere { .. } => "sphere",
            Self::Torus { .. } => "torus",
            Self::LinearExtrusion { .. } => "linear extrusion",
            Self::Unsupported { description } => description,
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadWallEdgeSide {
    pub edge_id: EntityId,
    pub tangent: [f64; 3],
    pub outward: [f64; 3],
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadFace {
    pub id: EntityId,
    pub body_id: BodyId,
    pub surface: CadSurface,
    pub loop_ids: Vec<EntityId>,
    pub bounds: [[f64; 3]; 2],
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum CadCurve {
    Line {
        start: [f64; 3],
        end: [f64; 3],
    },
    CircleArc {
        center: [f64; 3],
        normal: [f64; 3],
        x_axis: [f64; 3],
        radius: f64,
        start_angle: f64,
        end_angle: f64,
    },
    /// Exact trimmed spline, decomposed into rational Bezier spans by OCCT.
    BezierSpans {
        spans: Vec<crate::CadBezierSpan>,
    },
    Unsupported {
        description: String,
    },
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadEdge {
    pub id: EntityId,
    pub body_id: BodyId,
    pub vertices: [EntityId; 2],
    pub face_ids: Vec<EntityId>,
    pub curve: CadCurve,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EdgeUse {
    pub edge_id: EntityId,
    pub reversed: bool,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadLoop {
    pub id: EntityId,
    pub body_id: BodyId,
    pub face_id: EntityId,
    pub edge_uses: Vec<EdgeUse>,
    pub is_outer: bool,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadVertex {
    pub id: EntityId,
    pub body_id: BodyId,
    pub point: [f64; 3],
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CadMesh {
    pub vertices: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
    pub triangle_faces: Vec<EntityId>,
}
/// Independent CAD tessellation for projected machining outlines. Never replace
/// this with the viewport mesh: display quality must not change a toolpath.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadProjectionMesh {
    pub mesh: CadMesh,
    pub chord_tolerance: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TopologySnapshot {
    pub version: u32,
    pub bodies: Vec<CadBody>,
    pub faces: Vec<CadFace>,
    pub edges: Vec<CadEdge>,
    pub loops: Vec<CadLoop>,
    pub vertices: Vec<CadVertex>,
    pub mesh: CadMesh,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub projection: Option<CadProjectionMesh>,
}
impl Default for TopologySnapshot {
    fn default() -> Self {
        Self {
            version: 1,
            bodies: vec![],
            faces: vec![],
            edges: vec![],
            loops: vec![],
            vertices: vec![],
            mesh: CadMesh::default(),
            projection: None,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CuttingHeights {
    pub top: f64,
    pub bottom: f64,
}
impl CuttingHeights {
    pub fn validate(self) -> Result<(), String> {
        if self.top.is_finite() && self.bottom.is_finite() && self.top > self.bottom {
            Ok(())
        } else {
            Err("cutting heights must be finite with top above bottom".into())
        }
    }
    pub fn depth(self) -> f64 {
        self.top - self.bottom
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ExtractionRecipe {
    /// Conservative 3D adaptive roughing over selected bodies and faces, with
    /// every included setup body retained as an obstacle.
    AdaptiveClearing,
    ParallelFinishing,
    SpiralFinishing,
    PocketFloor,
    /// One closed planar loop or connected closed edge chain, at an operator-defined depth.
    PocketBoundary,
    DrillHoles,
    /// One closed planar rim; the operation defines the bevel dimensions.
    ChamferLoop,
    /// Modeled inclined planar faces bounded by two horizontal edges.
    ChamferFaces,
    /// One planar loop or connected open/closed chain, at an operator-defined depth.
    EngravePath,
    /// A planar face (including islands) or closed loop to V-carve.
    CarveRegion,
    /// Constant-diameter cylindrical bores or bosses aligned with setup Z.
    BoreCylinders {
        internal: bool,
    },
    ThreadCylinders {
        internal: bool,
    },
    ProfileLoop {
        side: Side,
    },
    /// Face boundaries or explicitly selected connected edges/loops.
    ProfileContours,
    /// Complete setup-Z projection, including through-opening contours.
    ProfileSilhouette,
    ProfileEntries {
        entries: Vec<crate::ProfileGeometryEntry>,
    },
    FaceRegion,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct GeometryOverrides {
    pub top: Option<f64>,
    pub bottom: Option<f64>,
}
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResolutionStatus {
    Resolved,
    NeedsReselection { reason: String },
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometryBinding {
    pub selected: Vec<EntityRef>,
    pub recipe: ExtractionRecipe,
    pub heights: CuttingHeights,
    #[serde(default)]
    pub overrides: GeometryOverrides,
    pub status: ResolutionStatus,
    /// Identifies the exact dependencies used to materialize operation geometry.
    /// Absence deliberately blocks generation, including older saved bindings.
    #[serde(default)]
    pub resolved_fingerprint: Option<String>,
}
impl GeometryBinding {
    pub fn effective_heights(&self) -> CuttingHeights {
        CuttingHeights {
            top: self.overrides.top.unwrap_or(self.heights.top),
            bottom: self.overrides.bottom.unwrap_or(self.heights.bottom),
        }
    }
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct GeometryState {
    pub assets: Vec<GeometryAsset>,
    pub instances: Vec<GeometryInstance>,
    #[serde(default, deserialize_with = "deserialize_bindings")]
    pub bindings: BTreeMap<u32, GeometryBinding>,
    /// Existing planar drawing geometry participates in project undo/history.
    #[serde(default)]
    pub drawing_regions: Vec<cam_geo::Polygon>,
    #[serde(default)]
    pub drawing_paths: Vec<cam_geo::Polyline>,
}
// The internally tagged .ocam union buffers map keys as strings. Convert them
// explicitly instead of relying on serde_json's numeric-key deserializer.
fn deserialize_bindings<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> Result<BTreeMap<u32, GeometryBinding>, D::Error> {
    let encoded = BTreeMap::<String, GeometryBinding>::deserialize(deserializer)?;
    let mut bindings = BTreeMap::new();
    for (key, value) in encoded {
        let id = key.parse::<u32>().map_err(serde::de::Error::custom)?;
        if bindings.insert(id, value).is_some() {
            return Err(serde::de::Error::custom("duplicate operation binding ID"));
        }
    }
    Ok(bindings)
}
impl GeometryState {
    /// Return a view of the shared geometry expressed in setup axes. Source assets,
    /// entity identities and bindings remain untouched; only instance placements in
    /// the returned value are transformed.
    pub fn geometry_in_setup(&self, frame: &SetupFrame) -> Result<Self, String> {
        frame.validate()?;
        let mut out = self.clone();
        let r = frame.world_from_setup.rotation;
        for instance in &mut out.instances {
            if !instance.placement.is_rigid() {
                return Err("invalid model instance placement".into());
            }
            let old = instance.placement;
            let rotation = std::array::from_fn(|i| {
                std::array::from_fn(|j| (0..3).map(|k| r[k][i] * old.rotation[k][j]).sum())
            });
            let translation =
                std::array::from_fn(|i| (0..3).map(|k| r[k][i] * old.translation[k]).sum());
            instance.placement = RigidTransform {
                rotation,
                translation,
            };
        }
        Ok(out)
    }

    /// Deterministic dependency key for an immutable asset revision. Display
    /// tessellation does not participate; placement and machining overrides do.
    pub fn binding_fingerprint(&self, binding: &GeometryBinding) -> String {
        if matches!(
            binding.recipe,
            ExtractionRecipe::AdaptiveClearing
                | ExtractionRecipe::ParallelFinishing
                | ExtractionRecipe::SpiralFinishing
        ) {
            let mut dependencies: Vec<_> = self
                .instances
                .iter()
                .map(|instance| {
                    let asset = self
                        .asset(instance.asset_id)
                        .map(|asset| (asset.id, asset.revision, &asset.backend));
                    (
                        instance.id,
                        instance.asset_id,
                        instance.body_id,
                        instance.placement,
                        asset,
                    )
                })
                .collect();
            dependencies.sort_by_key(|dependency| dependency.0);
            return serde_json::to_string(&(
                &binding.recipe,
                &binding.selected,
                binding.effective_heights(),
                &binding.overrides,
                dependencies,
            ))
            .expect("adaptive geometry dependency records serialize");
        }
        let dependencies: Vec<_> = binding
            .selected
            .iter()
            .map(|r| {
                let asset = self
                    .asset(r.asset_id)
                    .map(|a| (a.id, a.revision, &a.backend));
                let instance = self
                    .instance(r.instance_id)
                    .map(|i| (i.id, i.asset_id, i.body_id, i.placement));
                (asset, instance)
            })
            .collect();
        serde_json::to_string(&(
            &binding.recipe,
            &binding.selected,
            binding.effective_heights(),
            &binding.overrides,
            dependencies,
        ))
        .expect("geometry dependency records serialize")
    }
    pub fn asset(&self, id: AssetId) -> Option<&GeometryAsset> {
        self.assets.iter().find(|a| a.id == id)
    }
    pub fn instance(&self, id: InstanceId) -> Option<&GeometryInstance> {
        self.instances.iter().find(|i| i.id == id)
    }
    pub fn entity_exists(&self, r: &EntityRef) -> bool {
        let Some(a) = self.asset(r.asset_id) else {
            return false;
        };
        let Some(i) = self.instance(r.instance_id) else {
            return false;
        };
        if a.revision != r.revision || i.asset_id != r.asset_id || i.body_id != r.body_id {
            return false;
        }
        let t = &a.topology;
        match r.kind {
            EntityKind::Body | EntityKind::Silhouette => {
                t.bodies.iter().any(|e| e.id == r.id && e.id == r.body_id)
            }
            EntityKind::Face => t
                .faces
                .iter()
                .any(|e| e.id == r.id && e.body_id == r.body_id),
            EntityKind::Edge => t
                .edges
                .iter()
                .any(|e| e.id == r.id && e.body_id == r.body_id),
            EntityKind::Loop => t
                .loops
                .iter()
                .any(|e| e.id == r.id && e.body_id == r.body_id),
            EntityKind::Vertex => t
                .vertices
                .iter()
                .any(|e| e.id == r.id && e.body_id == r.body_id),
        }
    }
    /// Check persistent structure. Unresolved references are intentionally retained
    /// for review; callers must resolve included bindings before generation.
    pub fn validate(&self) -> Result<(), String> {
        let mut ids = BTreeSet::new();
        for a in &self.assets {
            if !a.source_fingerprint.is_empty()
                && a.source_fingerprint != GeometryAsset::fingerprint_source(&a.source_data)
            {
                return Err("embedded CAD source fingerprint does not match its bytes".into());
            }
            if let Some(options) = a.import_options {
                if !options.import_tolerance.is_finite()
                    || options.import_tolerance <= 0.0
                    || !options.display_tolerance.is_finite()
                    || options.display_tolerance <= 0.0
                {
                    return Err("stored CAD import tolerances must be finite and positive".into());
                }
            }
            if !ids.insert(a.id) {
                return Err("duplicate geometry asset ID".into());
            }
            a.topology.validate_cached()?;
        }
        ids.clear();
        for i in &self.instances {
            if !ids.insert(i.id) {
                return Err("duplicate geometry instance ID".into());
            }
            let a = self
                .asset(i.asset_id)
                .ok_or("geometry instance refers to a missing asset")?;
            if !a.topology.bodies.iter().any(|b| b.id == i.body_id) {
                return Err("geometry instance refers to a missing body".into());
            }
            if !i.placement.is_rigid() {
                return Err("geometry placement must be a finite proper rigid transform".into());
            }
        }
        for b in self.bindings.values() {
            if b.selected.is_empty() {
                return Err("geometry binding has no selection".into());
            }
            b.effective_heights().validate()?;
            if matches!(b.status, ResolutionStatus::Resolved)
                && !b.selected.iter().all(|r| self.entity_exists(r))
            {
                return Err("resolved geometry binding has stale or missing entities".into());
            }
        }
        Ok(())
    }
}
impl Operation {
    /// Legacy embedded contours have explicit geometry, with their historical
    /// world-space cutting planes preserved. No CAD association is guessed.
    pub fn legacy_cutting_heights(&self, heights: &Heights) -> Option<CuttingHeights> {
        let (top, bottom) = match self {
            Self::Pocket(p) => (heights.top_of_stock, -p.depth),
            Self::Profile(p) => (heights.top_of_stock, -p.depth),
            Self::Drill(p) => {
                let top = heights.top_of_stock + p.start_offset;
                (top, top - p.depth)
            }
            Self::Face(p) => (p.start_offset, p.start_offset - p.depth),
            _ => return None,
        };
        Some(CuttingHeights { top, bottom })
    }
}

thread_local! {
    // Weak ownership lets deleted imports go and makes Arc::make_mut detach a
    // validated snapshot even when it has only one remaining strong owner.
    static VALID_TOPOLOGIES: std::cell::RefCell<Vec<std::sync::Weak<TopologySnapshot>>> = const { std::cell::RefCell::new(Vec::new()) };
}

impl TopologySnapshot {
    /// Validate an immutable shared snapshot once per thread. Reference revisions,
    /// placements and selections still need validation on every request.
    pub fn validate_cached(self: &Arc<Self>) -> Result<(), String> {
        VALID_TOPOLOGIES.with(|cache| {
            let mut cache = cache.borrow_mut();
            cache.retain(|entry| entry.strong_count() > 0);
            if cache
                .iter()
                .any(|entry| entry.upgrade().is_some_and(|old| Arc::ptr_eq(&old, self)))
            {
                return Ok(());
            }
            self.validate()?;
            if cache.len() >= 32 {
                cache.remove(0);
            }
            cache.push(Arc::downgrade(self));
            Ok(())
        })
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.version != 1 {
            return Err(format!(
                "unsupported CAD topology snapshot version {}",
                self.version
            ));
        }
        let bodies: BTreeSet<_> = self.bodies.iter().map(|b| b.id).collect();
        if bodies.len() != self.bodies.len() {
            return Err("duplicate CAD body ID".into());
        }
        let unique = |ids: Vec<EntityId>| {
            let count = ids.len();
            ids.into_iter().collect::<BTreeSet<_>>().len() == count
        };
        if !unique(self.faces.iter().map(|e| e.id).collect())
            || !unique(self.edges.iter().map(|e| e.id).collect())
            || !unique(self.loops.iter().map(|e| e.id).collect())
            || !unique(self.vertices.iter().map(|e| e.id).collect())
        {
            return Err("duplicate CAD entity ID".into());
        }
        let finite = |p: [f64; 3]| p.into_iter().all(f64::is_finite);
        let direction = |p: [f64; 3]| {
            finite(p) && (p.into_iter().map(|x| x * x).sum::<f64>() - 1.).abs() < 1e-6
        };
        let bounds =
            |b: [[f64; 3]; 2]| finite(b[0]) && finite(b[1]) && (0..3).all(|i| b[0][i] <= b[1][i]);
        for b in &self.bodies {
            if !bounds(b.bounds) {
                return Err("invalid CAD body bounds".into());
            }
        }
        for v in &self.vertices {
            if !bodies.contains(&v.body_id) || !finite(v.point) {
                return Err("invalid CAD vertex".into());
            }
        }
        for f in &self.faces {
            if !bodies.contains(&f.body_id) || !bounds(f.bounds) {
                return Err("invalid CAD face body or bounds".into());
            }
            let ok = match &f.surface {
                CadSurface::Plane { origin, normal } => finite(*origin) && direction(*normal),
                CadSurface::Cylinder {
                    origin,
                    axis,
                    radius,
                    ..
                } => finite(*origin) && direction(*axis) && radius.is_finite() && *radius > 0.,
                CadSurface::Cone {
                    origin,
                    axis,
                    radius,
                    semi_angle,
                } => {
                    finite(*origin)
                        && direction(*axis)
                        && radius.is_finite()
                        && *radius >= 0.
                        && semi_angle.is_finite()
                        && semi_angle.abs() > 0.
                        && semi_angle.abs() < std::f64::consts::FRAC_PI_2
                }
                CadSurface::Sphere { center, radius } => {
                    finite(*center) && radius.is_finite() && *radius > 0.
                }
                CadSurface::Torus {
                    origin,
                    axis,
                    major_radius,
                    minor_radius,
                } => {
                    finite(*origin)
                        && direction(*axis)
                        && major_radius.is_finite()
                        && *major_radius > 0.
                        && minor_radius.is_finite()
                        && *minor_radius > 0.
                }
                CadSurface::LinearExtrusion { axis, edge_sides } => {
                    direction(*axis)
                        && edge_sides.iter().all(|side| {
                            direction(side.tangent)
                                && direction(side.outward)
                                && (0..3)
                                    .map(|i| side.tangent[i] * side.outward[i])
                                    .sum::<f64>()
                                    .abs()
                                    < 1e-6
                                && self
                                    .edges
                                    .iter()
                                    .any(|e| e.id == side.edge_id && e.face_ids.contains(&f.id))
                        })
                }
                CadSurface::Unsupported { .. } => true,
            };
            if !ok {
                return Err("invalid analytic CAD surface".into());
            }
            if !f.loop_ids.iter().all(|id| {
                self.loops
                    .iter()
                    .any(|l| l.id == *id && l.face_id == f.id && l.body_id == f.body_id)
            }) {
                return Err("CAD face refers to a missing loop".into());
            }
        }
        for e in &self.edges {
            if !bodies.contains(&e.body_id)
                || !e.vertices.iter().all(|id| {
                    self.vertices
                        .iter()
                        .any(|v| v.id == *id && v.body_id == e.body_id)
                })
                || !e.face_ids.iter().all(|id| {
                    self.faces
                        .iter()
                        .any(|f| f.id == *id && f.body_id == e.body_id)
                })
            {
                return Err("invalid CAD edge adjacency".into());
            }
            let ok = match &e.curve {
                CadCurve::Line { start, end } => finite(*start) && finite(*end),
                CadCurve::CircleArc {
                    center,
                    normal,
                    x_axis,
                    radius,
                    start_angle,
                    end_angle,
                } => {
                    finite(*center)
                        && direction(*normal)
                        && direction(*x_axis)
                        && (0..3).map(|i| normal[i] * x_axis[i]).sum::<f64>().abs() < 1e-6
                        && radius.is_finite()
                        && *radius > 0.
                        && start_angle.is_finite()
                        && end_angle.is_finite()
                }
                CadCurve::BezierSpans { spans } => {
                    !spans.is_empty()
                        && spans.iter().map(|s| s.poles.len()).sum::<usize>() <= 65536
                        && spans.iter().all(|s| s.is_valid())
                }
                CadCurve::Unsupported { .. } => true,
            };
            if !ok {
                return Err("invalid analytic CAD curve".into());
            }
        }
        for l in &self.loops {
            if !self
                .faces
                .iter()
                .any(|f| f.id == l.face_id && f.body_id == l.body_id)
                || !l.edge_uses.iter().all(|u| {
                    self.edges.iter().any(|e| {
                        e.id == u.edge_id
                            && e.body_id == l.body_id
                            && e.face_ids.contains(&l.face_id)
                    })
                })
            {
                return Err("invalid CAD loop adjacency".into());
            }
        }
        for mesh in std::iter::once(&self.mesh).chain(self.projection.iter().map(|p| &p.mesh)) {
            if mesh.triangles.len() != mesh.triangle_faces.len()
                || !mesh.vertices.iter().all(|p| finite(*p))
                || !mesh
                    .triangles
                    .iter()
                    .flatten()
                    .all(|i| (*i as usize) < mesh.vertices.len())
                || !mesh
                    .triangle_faces
                    .iter()
                    .all(|id| self.faces.iter().any(|f| f.id == *id))
            {
                return Err("invalid CAD mesh face mapping".into());
            }
        }
        if self
            .projection
            .as_ref()
            .is_some_and(|p| !p.chord_tolerance.is_finite() || p.chord_tolerance <= 0.)
        {
            return Err("invalid CAD projection tolerance".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn setup_frame_derives_missing_axis_and_rejects_bad_handedness() {
        let f = SetupFrame::from_axes(Some([0., 1., 0.]), Some([-1., 0., 0.]), None).unwrap();
        assert_eq!(f.to_setup_vector([0., 1., 0.]), [1., 0., 0.]);
        assert!(
            SetupFrame::from_axes(Some([1., 0., 0.]), Some([0., 1., 0.]), Some([0., 0., -1.]))
                .is_err()
        );
        assert!(SetupFrame::from_axes(Some([1., 0., 0.]), Some([1., 1., 0.]), None).is_err());
        assert!(
            SetupFrame::from_axes(Some([1., 0., 0.]), Some([1e-10, 1., 0.]), None).is_ok(),
            "roundoff sized deviation is accepted and produces a rigid frame"
        );
        assert!(
            SetupFrame::from_axes(Some([1., 0., 0.]), Some([1e-7, 1., 0.]), None).is_err(),
            "a deviation outside frame rigidity tolerance is rejected"
        );
    }
    #[test]
    fn partial_references_preserve_orientation_and_complete_a_rigid_frame() {
        let state = GeometryState::default();
        let current = SetupFrame::from_xy([0., 1., 0.], [0., 0., 1.]).unwrap();
        assert_eq!(
            current
                .resolve_references(&state, FrameReferences::default())
                .unwrap(),
            current
        );
        for axis in 0..3 {
            for direction in [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [-1., -2., -3.]] {
                let mut refs = FrameReferences::default();
                refs.stock_axes[axis] = Some(direction);
                let frame = current.resolve_references(&state, refs.clone()).unwrap();
                frame.validate().unwrap();
                let selected =
                    frame.to_world_vector(std::array::from_fn(|i| if i == axis { 1. } else { 0. }));
                let length = direction.iter().map(|v| v * v).sum::<f64>().sqrt();
                assert!((0..3).all(|i| (selected[i] - direction[i] / length).abs() < 1e-9));
                let repeated = frame.resolve_references(&state, refs.clone()).unwrap();
                assert!(
                    (0..3).all(|i| (0..3).all(|j| (frame.world_from_setup.rotation[i][j]
                        - repeated.world_from_setup.rotation[i][j])
                        .abs()
                        < 1e-9))
                );
                refs.stock_flipped[axis] = true;
                let flipped = current.resolve_references(&state, refs).unwrap();
                flipped.validate().unwrap();
                let selected =
                    flipped
                        .to_world_vector(std::array::from_fn(|i| if i == axis { 1. } else { 0. }));
                assert!((0..3).all(|i| (selected[i] + direction[i] / length).abs() < 1e-9));
            }
        }
        let mut refs = FrameReferences::default();
        refs.stock_axes[2] = Some([1., 0., 0.]);
        // Z is aligned with current X: use current Y in its own axis slot.
        let frame = SetupFrame::default()
            .resolve_references(&state, refs)
            .unwrap();
        assert_eq!(frame.to_world_vector([1., 0., 0.]), [0., 0., -1.]);
        assert_eq!(frame.to_world_vector([0., 1., 0.]), [0., 1., 0.]);
    }
    #[test]
    fn partial_references_still_reject_invalid_picks() {
        let state = GeometryState::default();
        for direction in [[0.; 3], [f64::NAN, 0., 0.], [f64::INFINITY, 0., 0.]] {
            let mut refs = FrameReferences::default();
            refs.stock_axes[2] = Some(direction);
            assert!(SetupFrame::default()
                .resolve_references(&state, refs)
                .is_err());
        }
        let mut refs = FrameReferences {
            stock_axes: [Some([1., 0., 0.]), Some([1., 0., 0.]), None],
            ..Default::default()
        };
        assert!(SetupFrame::default()
            .resolve_references(&state, refs.clone())
            .is_err());
        refs.stock_axes = [Some([1., 0., 0.]), Some([0., 1., 0.]), Some([0., 0., -1.])];
        assert!(SetupFrame::default()
            .resolve_references(&state, refs)
            .is_err());
    }
    fn state() -> GeometryState {
        GeometryState {
            assets: vec![GeometryAsset {
                import_options: None,
                id: 1,
                revision: 2,
                name: "plate".into(),
                source_format: CadFormat::Step,
                source_path: None,
                source_data: b"ISO-10303-21;".to_vec().into(),
                source_units: SourceUnits::Inches,
                backend: "test-1".into(),
                import_diagnostics: vec![],
                source_fingerprint: GeometryAsset::fingerprint_source(b"ISO-10303-21;"),
                topology: TopologySnapshot {
                    bodies: vec![CadBody {
                        id: 1,
                        name: "body".into(),
                        valid_solid: true,
                        bounds: [[0., 0., 0.], [10., 10., 10.]],
                    }],
                    faces: vec![CadFace {
                        id: 5,
                        body_id: 1,
                        surface: CadSurface::Plane {
                            origin: [0., 0., 5.],
                            normal: [0., 0., 1.],
                        },
                        loop_ids: vec![],
                        bounds: [[0., 0., 5.], [10., 10., 5.]],
                    }],
                    mesh: CadMesh {
                        vertices: vec![[0., 0., 5.], [10., 0., 5.], [0., 10., 5.]],
                        triangles: vec![[0, 1, 2]],
                        triangle_faces: vec![5],
                    },
                    ..Default::default()
                }
                .into(),
            }],
            instances: vec![GeometryInstance {
                id: 10,
                asset_id: 1,
                body_id: 1,
                name: "part".into(),
                placement: RigidTransform::default(),
                visible: true,
            }],
            bindings: BTreeMap::from([(
                7,
                GeometryBinding {
                    selected: vec![EntityRef {
                        asset_id: 1,
                        revision: 2,
                        instance_id: 10,
                        body_id: 1,
                        kind: EntityKind::Face,
                        id: 5,
                    }],
                    recipe: ExtractionRecipe::PocketFloor,
                    heights: CuttingHeights {
                        top: 10.,
                        bottom: 5.,
                    },
                    overrides: GeometryOverrides {
                        top: Some(11.),
                        bottom: None,
                    },
                    status: ResolutionStatus::Resolved,
                    resolved_fingerprint: None,
                },
            )]),
            drawing_regions: vec![],
            drawing_paths: vec![],
        }
    }
    #[test]
    fn cone_and_sphere_metadata_round_trip_and_reject_invalid_dimensions() {
        for (surface, valid) in [
            (
                CadSurface::Sphere {
                    center: [1., 2., 3.],
                    radius: 5.,
                },
                true,
            ),
            (
                CadSurface::Sphere {
                    center: [1., 2., 3.],
                    radius: -5.,
                },
                false,
            ),
            (
                CadSurface::Cone {
                    origin: [0.; 3],
                    axis: [0., 0., 1.],
                    radius: 0.,
                    semi_angle: 0.3,
                },
                true,
            ),
            (
                CadSurface::Cone {
                    origin: [0.; 3],
                    axis: [0., 0., 1.],
                    radius: 5.,
                    semi_angle: -0.3,
                },
                true,
            ),
            (
                CadSurface::Cone {
                    origin: [0.; 3],
                    axis: [0., 0., 1.],
                    radius: 5.,
                    semi_angle: 0.,
                },
                false,
            ),
            (
                CadSurface::Cone {
                    origin: [0.; 3],
                    axis: [0., 0., 1.],
                    radius: 5.,
                    semi_angle: std::f64::consts::FRAC_PI_2,
                },
                false,
            ),
        ] {
            let mut topology = (*state().assets[0].topology).clone();
            topology.faces[0].surface = surface;
            let encoded = serde_json::to_vec(&topology).unwrap();
            let restored: TopologySnapshot = serde_json::from_slice(&encoded).unwrap();
            assert_eq!(restored, topology);
            assert_eq!(
                restored.validate().is_ok(),
                valid,
                "{:?}",
                restored.faces[0].surface
            );
        }
    }
    #[test]
    fn mismatched_embedded_source_and_invalid_import_budgets_are_rejected() {
        let mut s = state();
        std::sync::Arc::make_mut(&mut s.assets[0].source_data).push(0);
        assert!(s.validate().unwrap_err().contains("fingerprint"));
        let mut s = state();
        s.assets[0].import_options = Some(GeometryImportOptions {
            import_tolerance: f64::NAN,
            display_tolerance: 0.1,
        });
        assert!(s.validate().unwrap_err().contains("tolerances"));
    }

    #[test]
    fn shared_topology_validation_reuses_only_immutable_snapshots() {
        let mut s = state();
        s.validate().unwrap();
        let original = s.assets[0].topology.clone();
        assert_eq!(Arc::weak_count(&original), 1);
        s.validate().unwrap();
        assert_eq!(
            Arc::weak_count(&original),
            1,
            "reuse the existing cache entry"
        );

        Arc::make_mut(&mut s.assets[0].topology).version = 2;
        assert!(s.validate().unwrap_err().contains("snapshot version"));
        original.validate_cached().unwrap();
        let weak = Arc::downgrade(&original);
        drop(original);
        assert!(
            weak.upgrade().is_none(),
            "validation must not retain deleted CAD data"
        );

        let mut sole_owner = state().assets.remove(0).topology;
        sole_owner.validate_cached().unwrap();
        let weak = Arc::downgrade(&sole_owner);
        Arc::make_mut(&mut sole_owner).mesh.triangles.push([999; 3]);
        assert!(weak.upgrade().is_none());
        assert!(
            sole_owner.validate_cached().is_err(),
            "sole-owner edits must be revalidated"
        );
    }
    #[test]
    fn self_contained_geometry_roundtrip_keeps_identity_mesh_and_overrides() {
        let s = state();
        s.validate().unwrap();
        let loaded: GeometryState =
            serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        assert_eq!(s, loaded);
        loaded.validate().unwrap();
        assert_eq!(
            loaded.bindings[&7].effective_heights(),
            CuttingHeights {
                top: 11.,
                bottom: 5.
            }
        );
        assert_eq!(loaded.assets[0].source_data.as_slice(), b"ISO-10303-21;");
        assert_eq!(SourceUnits::Inches.millimetre_scale(), Some(25.4));
        assert_eq!(SourceUnits::Unknown.millimetre_scale(), None);
    }
    #[test]
    fn undo_snapshots_share_blobs_and_restore_bindings_together() {
        let original = state();
        let bytes = original.assets[0].source_data.clone();
        let topology = original.assets[0].topology.clone();
        let mut history = crate::History::new(original);
        history.edit(|geometry| geometry.instances[0].placement.translation[2] = 30.0);
        assert!(Arc::ptr_eq(
            &history.current().assets[0].source_data,
            &bytes
        ));
        assert!(Arc::ptr_eq(
            &history.current().assets[0].topology,
            &topology
        ));
        history.edit(|geometry| {
            geometry.bindings.clear();
            geometry.instances.clear();
            geometry.assets.clear();
        });
        assert!(history.undo());
        assert_eq!(history.current().bindings.len(), 1);
        assert_eq!(
            history.current().instances[0].placement.translation[2],
            30.0
        );
        assert!(history.undo());
        assert_eq!(history.current().instances[0].placement.translation[2], 0.0);
        assert!(Arc::ptr_eq(
            &history.current().assets[0].topology,
            &topology
        ));
    }
    #[test]
    fn shared_asset_equality_still_compares_detached_geometry_and_metadata() {
        let original = state().assets.remove(0);
        assert_eq!(original, original.clone());
        let detached: GeometryAsset =
            serde_json::from_str(&serde_json::to_string(&original).unwrap()).unwrap();
        assert!(!Arc::ptr_eq(&original.topology, &detached.topology));
        assert_eq!(original, detached);

        let mut edited = original.clone();
        edited.name.push_str(" renamed");
        assert_ne!(original, edited);
        let mut edited = original.clone();
        Arc::make_mut(&mut edited.topology)
            .mesh
            .vertices
            .push([1., 2., 3.]);
        assert_ne!(original, edited);
        let mut edited = original.clone();
        Arc::make_mut(&mut edited.source_data).push(0);
        assert_ne!(original, edited);
    }
    #[test]
    fn stale_revision_requires_reselection_and_never_reattaches_by_entity_number() {
        let mut s = state();
        s.assets[0].revision += 1;
        assert!(!s.entity_exists(&s.bindings[&7].selected[0]));
        assert!(s.validate().unwrap_err().contains("stale"));
        s.bindings.get_mut(&7).unwrap().status = ResolutionStatus::NeedsReselection {
            reason: "model replaced".into(),
        };
        s.validate().unwrap();
        let loaded: GeometryState =
            serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        assert_eq!(loaded.bindings[&7].overrides.top, Some(11.));
    }
    #[test]
    fn resolution_key_tracks_placement_revision_and_overrides_but_not_display() {
        let mut s = state();
        let original = s.binding_fingerprint(&s.bindings[&7]);
        s.bindings.get_mut(&7).unwrap().resolved_fingerprint = Some(original.clone());
        assert_eq!(s.binding_fingerprint(&s.bindings[&7]), original);
        s.instances[0].visible = false;
        Arc::make_mut(&mut s.assets[0].topology).mesh.vertices[0][0] += 0.01;
        assert_eq!(s.binding_fingerprint(&s.bindings[&7]), original);
        s.instances[0].placement.translation[2] = 10.0;
        assert_ne!(s.binding_fingerprint(&s.bindings[&7]), original);
        s.instances[0].placement.translation[2] = 0.0;
        s.assets[0].revision += 1;
        assert_ne!(s.binding_fingerprint(&s.bindings[&7]), original);
        s.assets[0].revision -= 1;
        s.bindings.get_mut(&7).unwrap().overrides.bottom = Some(4.0);
        assert_ne!(s.binding_fingerprint(&s.bindings[&7]), original);
    }
    #[test]
    fn mesh_mapping_and_duplicate_ids_are_checked() {
        let mut s = state();
        Arc::make_mut(&mut s.assets[0].topology)
            .mesh
            .triangle_faces
            .clear();
        assert!(s.validate().unwrap_err().contains("mesh"));
        let mut s = state();
        s.instances.push(s.instances[0].clone());
        assert!(s.validate().unwrap_err().contains("duplicate"));
    }
    #[test]
    fn placement_preserves_double_precision_and_rejects_scaling_or_reflection() {
        let t = RigidTransform {
            rotation: [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]],
            translation: [1e9 + 0.125, 20., 30.],
        };
        assert!(t.is_rigid());
        let p = [3., 4., 5.];
        assert_eq!(t.inverse_point(t.transform_point(p)), p);
        let mut scale = RigidTransform::default();
        scale.rotation[0][0] = 2.;
        assert!(!scale.is_rigid());
        scale.rotation[0][0] = -1.;
        assert!(!scale.is_rigid());
    }
    #[test]
    fn legacy_face_and_drill_keep_their_different_reference_planes() {
        let heights = Heights::new(30., 25., 20.);
        let face:Operation=serde_json::from_value(serde_json::json!({"Face":{"id":1,"tool":1,"boundary":{"points":[[0.,0.],[1.,0.],[0.,1.]]},"start_offset":7.,"depth":3.,"stepdown":1.,"feed":100.,"plunge_feed":50.}})).unwrap();
        let drill:Operation=serde_json::from_value(serde_json::json!({"Drill":{"id":2,"tool":1,"points":[[0.,0.]],"start_offset":7.,"depth":3.,"peck":null,"dwell":null,"feed":100.}})).unwrap();
        assert_eq!(
            face.legacy_cutting_heights(&heights),
            Some(CuttingHeights {
                top: 7.,
                bottom: 4.
            })
        );
        assert_eq!(
            drill.legacy_cutting_heights(&heights),
            Some(CuttingHeights {
                top: 27.,
                bottom: 24.
            })
        );
    }
}
