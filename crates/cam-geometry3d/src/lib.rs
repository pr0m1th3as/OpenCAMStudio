//! Fixed-axis feature extraction. Analytic operations use exact topology;
//! silhouettes use a separate CAD projection tessellation with an explicit
//! allowance. Display meshes never enter machining derivation.
mod parallel;
mod rotary;
pub use parallel::{resolve_parallel, ParallelGeometry};
pub use rotary::{
    periodic_cylinder_selection, unwrap_boundaries, wrap_cylinder, wrapped_boundary_height,
    wrapped_regions,
};
mod adaptive;
mod advisory;
mod drill_holes;
pub use drill_holes::{drill_holes, rebind_drill_face, DrillHole};
mod pocket_regions;
mod pocket_stock;
pub use advisory::{
    resolve_binding_advisory, resolve_selection_advisory, verify_profile_entry, verify_selection,
};
pub use pocket_stock::pocket_stock_domains;
mod bore_entry;
mod chamfer_faces;
mod profile_contours;
mod profile_entries;
mod surface_geometry;
pub use adaptive::{
    adaptive_heights, pocket_stock_shadow, projected_footprint, resolve_adaptive, validate_adaptive_selection,
    AdaptiveGeometry, AdaptiveLayer, AdaptiveResolveConfig,
};
use cam_geo::{Contour, Point, Polygon};
use cam_model::*;
pub use profile_contours::coplanar_faces as coplanar_profile_faces;
pub use profile_entries::{profile_entry_contours, split_profile_chain_entry};
use std::{collections::BTreeSet, f64::consts::TAU};

/// Recognition and machining budgets are independent of display tessellation.
pub const RECOGNITION_TOLERANCE: f64 = 1e-5;
pub const MACHINING_CHORD_TOLERANCE: f64 = 0.01;
/// Version of the CAD pocket floor classification used by saved toolpaths.
pub const POCKET_FLOOR_REVISION: u32 = 3;
const T: f64 = RECOGNITION_TOLERANCE;

/// Borrow only machining dependencies. Exact planar curves do not depend on
/// either tessellation; silhouette operations also depend on projection data.
#[derive(serde::Serialize)]
struct MachiningTopology<'a> {
    version: u32,
    bodies: &'a [CadBody],
    faces: &'a [CadFace],
    edges: &'a [CadEdge],
    loops: &'a [CadLoop],
    vertices: &'a [CadVertex],
    projection: Option<&'a CadProjectionMesh>,
}
impl<'a> MachiningTopology<'a> {
    fn new(topology: &'a TopologySnapshot, projection: bool) -> Self {
        Self {
            version: topology.version,
            bodies: &topology.bodies,
            faces: &topology.faces,
            edges: &topology.edges,
            loops: &topology.loops,
            vertices: &topology.vertices,
            projection: if projection {
                topology.projection.as_ref()
            } else {
                None
            },
        }
    }
}

struct CachedTopologyFingerprint {
    topology: std::sync::Weak<TopologySnapshot>,
    exact: Option<u64>,
    projection: Option<u64>,
}

/// Geometry snapshots never change in place. Hash large projection arrays once,
/// then combine that digest with the current placement and operation settings.
/// Weak keys neither retain deleted imports nor survive Arc::make_mut edits.
fn topology_fingerprint(
    topology: &std::sync::Arc<TopologySnapshot>,
    projection: bool,
) -> Result<u64, String> {
    TOPOLOGY_FINGERPRINTS.with(|cache| {
        let mut cache = cache.borrow_mut();
        cache.retain(|entry| entry.topology.strong_count() > 0);
        let index = cache.iter().position(|entry| {
            entry
                .topology
                .upgrade()
                .is_some_and(|old| std::sync::Arc::ptr_eq(&old, topology))
        });
        if let Some(index) = index {
            let entry = &cache[index];
            if let Some(hash) = if projection {
                entry.projection
            } else {
                entry.exact
            } {
                return Ok(hash);
            }
        }
        let encoded = serde_json::to_vec(&MachiningTopology::new(topology, projection))
            .map_err(|error| error.to_string())?;
        let hash = encoded
            .into_iter()
            .fold(0xcbf29ce484222325_u64, |hash, byte| {
                (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
            });
        let index = if let Some(index) = index {
            index
        } else {
            if cache.len() >= 32 {
                cache.remove(0);
            }
            cache.push(CachedTopologyFingerprint {
                topology: std::sync::Arc::downgrade(topology),
                exact: None,
                projection: None,
            });
            cache.len() - 1
        };
        if projection {
            cache[index].projection = Some(hash);
        } else {
            cache[index].exact = Some(hash);
        }
        Ok(hash)
    })
}
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedHole {
    pub center: Point,
    pub diameter: f64,
}
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedInput {
    pub chamfers: Vec<ChamferFace>,
    pub profiles: Vec<ProfileContour>,
    pub regions: Vec<Polygon>,
    pub holes: Vec<ResolvedHole>,
    pub heights: CuttingHeights,
    /// Includes topology/projection data, placement, recipe and overrides; excludes the display mesh.
    pub fingerprint: String,
    pub diagnostics: Vec<String>,
}

/// A bore boundary must be cut on its inside. Strategy extraction still checks
/// the complete selected feature; this only chooses the initial profile side.
pub fn profile_side(state: &GeometryState, selected: &[EntityRef]) -> Result<Side, String> {
    let view = View::new(state, selected)?;
    if selected.len() == 1 && selected[0].kind == EntityKind::Silhouette {
        return Ok(Side::Outside);
    }
    if selected.len() != 1 || selected[0].kind != EntityKind::Loop {
        return Err(
            "Profile requires one closed coplanar loop; select one boundary per operation".into(),
        );
    }
    let wire = view.wire(selected[0].id)?;
    // A concave cylindrical segment can also be part of an exterior scallop.
    // Only an entirely cylindrical boundary suggests a bore; extraction still
    // certifies the complete feature before accepting an inside cut.
    let mut bore_boundary = !wire.edge_uses.is_empty();
    for usage in &wire.edge_uses {
        let mut inward_wall = false;
        for id in &view.edge(usage.edge_id)?.face_ids {
            inward_wall |= matches!(
                view.face(*id)?.surface,
                CadSurface::Cylinder { inward: true, .. }
            );
        }
        bore_boundary &= inward_wall;
    }
    if bore_boundary {
        return Ok(Side::Inside);
    }
    Ok(Side::Outside)
}

pub fn resolve_selection(
    state: &GeometryState,
    selected: &[EntityRef],
    recipe: ExtractionRecipe,
    stock_top: f64,
) -> Result<GeometryBinding, String> {
    if !stock_top.is_finite() {
        return Err("stock top must be finite".into());
    }
    let mut binding = GeometryBinding {
        selected: selected.to_vec(),
        recipe,
        heights: CuttingHeights {
            top: stock_top,
            bottom: stock_top - 1.0,
        },
        overrides: Default::default(),
        resolved_fingerprint: None,
        status: ResolutionStatus::Resolved,
    };
    let (_, _, heights) = derive(state, &binding, stock_top)?;
    binding.heights = heights;
    Ok(binding)
}

pub fn resolve_binding(
    state: &GeometryState,
    binding: &GeometryBinding,
) -> Result<ResolvedInput, String> {
    if let Some(input) = pocket_regions::resolve(state, binding, false) {
        return input;
    }
    if let ExtractionRecipe::ProfileEntries { entries } = &binding.recipe {
        return profile_entries::resolve(state, binding, entries);
    }
    // Cached contour or status is never sufficient: validate every entity revision.
    let (regions, holes, base) = derive(state, binding, binding.heights.top)?;
    let heights = CuttingHeights {
        top: binding.overrides.top.unwrap_or(base.top),
        bottom: binding.overrides.bottom.unwrap_or(base.bottom),
    };
    heights.validate()?;
    if !surface_geometry::is_surface_recipe(&binding.recipe) && heights.bottom < base.bottom - T {
        return Err("bottom override extends below the geometry-certified cutting interval".into());
    }
    if heights.top < base.top - T {
        return Err("top override skips material above the geometry-certified entry plane".into());
    }
    if matches!(
        binding.recipe,
        ExtractionRecipe::AdaptiveClearing
            | ExtractionRecipe::ParallelFinishing
            | ExtractionRecipe::SpiralFinishing
    ) {
        return Ok(ResolvedInput {
            chamfers: Vec::new(),
            profiles: Vec::new(),
            regions,
            holes,
            heights,
            fingerprint: state.binding_fingerprint(binding),
            diagnostics: vec![match binding.recipe {
                ExtractionRecipe::ParallelFinishing =>
                    "3D Parallel follows selected CAD surfaces and protects all setup bodies",
                ExtractionRecipe::SpiralFinishing =>
                    "3D Spiral projects radial passes onto selected CAD surfaces and protects all setup bodies",
                _ => "3D Adaptive uses selected machining domains and complete setup-model obstacle shadows",
            }.into()],
        });
    }
    let view = View::new(state, &binding.selected)?;
    let exact = topology_fingerprint(
        &view.asset.topology,
        binding
            .selected
            .iter()
            .any(|r| r.kind == EntityKind::Silhouette),
    )?;
    let encoded = serde_json::to_vec(&(
        view.asset.id,
        view.asset.revision,
        &exact,
        view.instance.placement,
        &binding.selected,
        &binding.recipe,
        &binding.overrides,
    ))
    .map_err(|e| e.to_string())?;
    // Deterministic FNV-1a dependency fingerprint (not a security/content hash).
    let hash = encoded.into_iter().fold(0xcbf29ce484222325_u64, |h, b| {
        (h ^ b as u64).wrapping_mul(0x100000001b3)
    });
    let diagnostics = if binding
        .selected
        .iter()
        .any(|r| r.kind == EntityKind::Silhouette)
    {
        let tolerance = view
            .asset
            .topology
            .projection
            .as_ref()
            .unwrap()
            .chord_tolerance;
        vec![format!("XY silhouette from independent CAD projection tessellation ({tolerance} mm deflection); outward allowance {} mm. This profiles the whole body envelope, including rounded transitions.", silhouette_allowance(tolerance))]
    } else {
        vec![format!("Exact curves approximated within {MACHINING_CHORD_TOLERANCE} mm; planar grid error ≤ {} mm",cam_geo::GRID_MM/2f64.sqrt())]
    };
    let profiles = if matches!(
        binding.recipe,
        ExtractionRecipe::ChamferLoop | ExtractionRecipe::EngravePath
    ) {
        surface_geometry::path(
            state,
            &binding.selected,
            binding.recipe == ExtractionRecipe::ChamferLoop,
        )?
        .0
    } else if binding.recipe == ExtractionRecipe::ProfileContours {
        profile_contours::extract(state, &binding.selected)?.0
    } else if binding.recipe == ExtractionRecipe::ProfileSilhouette {
        regions
            .iter()
            .flat_map(|region| {
                std::iter::once(ProfileContour {
                    chain: region.outer().clone(),
                    closed: true,
                    side: Side::Outside,
                })
                .chain(region.holes().iter().map(|hole| ProfileContour {
                    chain: hole.clone(),
                    closed: true,
                    side: Side::Inside,
                }))
            })
            .collect()
    } else {
        Vec::new()
    };
    Ok(ResolvedInput {
        chamfers: if binding.recipe == ExtractionRecipe::ChamferFaces {
            chamfer_faces::extract(state, &binding.selected)?.0
        } else {
            Vec::new()
        },
        profiles,
        regions,
        holes,
        heights,
        fingerprint: format!("{hash:016x}"),
        diagnostics,
    })
}
/// Project one placed body onto setup XY. Only the external outline is used;
/// holes are filled because this selection represents an outside profile.
/// Old projects need reimporting to obtain the independent projection data.
pub fn silhouette(state: &GeometryState, reference: &EntityRef) -> Result<Polygon, String> {
    let view = View::new(state, std::slice::from_ref(reference))?;
    view.silhouette()
}
fn silhouette_allowance(tolerance: f64) -> f64 {
    // Cover tessellation deviation and planar-grid rounding. Leave this allowance
    // on the part rather than offsetting a faceted approximation into its surface.
    tolerance + cam_geo::GRID_MM * 2.
}
struct CachedSilhouette {
    topology: std::sync::Weak<TopologySnapshot>,
    body: BodyId,
    placement: RigidTransform,
    include_holes: bool,
    region: Polygon,
}
thread_local! {
    // Hover reuses immutable CAD data. Weak ownership does not keep deleted
    // imports alive; placement and snapshot identity prevent stale outlines.
    static SILHOUETTES: std::cell::RefCell<Vec<CachedSilhouette>> = const { std::cell::RefCell::new(Vec::new()) };
    static TOPOLOGY_FINGERPRINTS: std::cell::RefCell<Vec<CachedTopologyFingerprint>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// Direction used to index a planar face, closed planar CAD loop, or straight edge.
/// The input state is expressed in setup coordinates.
pub fn index_selection_direction(
    state: &GeometryState,
    reference: &EntityRef,
) -> Result<[f64; 3], String> {
    let view = View::for_path(state, std::slice::from_ref(reference))?;
    let dot = |a: [f64; 3], b: [f64; 3]| (0..3).map(|i| a[i] * b[i]).sum::<f64>();
    match reference.kind {
        EntityKind::Face => {
            let face = view
                .asset
                .topology
                .faces
                .iter()
                .find(|f| f.id == reference.id)
                .ok_or("Face is missing")?;
            match face.surface {
                CadSurface::Plane { normal, .. } => {
                    Ok(view.instance.placement.transform_vector(normal))
                }
                _ => Err("Select a planar face for indexing".into()),
            }
        }
        EntityKind::Edge => {
            let edge = view.edge(reference.id)?;
            match edge.curve {
                CadCurve::Line { start, end } => Ok(view
                    .instance
                    .placement
                    .transform_vector(std::array::from_fn(|i| end[i] - start[i]))),
                _ => Err("Select a straight edge to define the Z axis".into()),
            }
        }
        EntityKind::Loop => {
            let wire = view
                .asset
                .topology
                .loops
                .iter()
                .find(|w| w.id == reference.id)
                .ok_or("Chain is missing")?;
            if wire.edge_uses.is_empty() {
                return Err("Select a closed planar chain".into());
            }
            let mut points: Vec<[f64; 3]> = Vec::new();
            let mut support = Vec::new();
            for usage in &wire.edge_uses {
                let edge = view.edge(usage.edge_id)?;
                let segment = view.curve_points(edge, usage.reversed, MACHINING_CHORD_TOLERANCE)?;
                if points.last().is_some_and(|p| distance(*p, segment[0]) > T) {
                    return Err("Chain is not connected".into());
                }
                support.extend(view.curve_support_points(edge)?);
                points.extend(segment);
            }
            if distance(points[0], *points.last().unwrap()) > T {
                return Err("Select a closed chain".into());
            }
            let origin = points[0];
            let a = points
                .iter()
                .map(|p| std::array::from_fn::<_, 3, _>(|i| p[i] - origin[i]))
                .max_by(|a, b| dot(*a, *a).total_cmp(&dot(*b, *b)))
                .unwrap();
            let n = points
                .iter()
                .map(|p| cross(a, std::array::from_fn(|i| p[i] - origin[i])))
                .max_by(|a, b| dot(*a, *a).total_cmp(&dot(*b, *b)))
                .unwrap();
            let length = dot(n, n).sqrt();
            if length < 1e-10 {
                return Err("Chain does not define a plane".into());
            }
            let n = n.map(|v| v / length);
            if support
                .iter()
                .any(|p| dot(std::array::from_fn(|i| p[i] - origin[i]), n).abs() > T)
            {
                return Err("Select a planar chain".into());
            }
            Ok(n)
        }
        _ => Err("Select a planar face, closed chain, or straight edge".into()),
    }
}

struct View<'a> {
    asset: &'a GeometryAsset,
    instance: &'a GeometryInstance,
    body: &'a CadBody,
}
impl<'a> View<'a> {
    fn new(state: &'a GeometryState, refs: &[EntityRef]) -> Result<Self, String> {
        Self::new_impl(state, refs, true)
    }
    fn for_path(state: &'a GeometryState, refs: &[EntityRef]) -> Result<Self, String> {
        Self::new_impl(state, refs, false)
    }
    fn new_impl(
        state: &'a GeometryState,
        refs: &[EntityRef],
        require_solid: bool,
    ) -> Result<Self, String> {
        let first = refs.first().ok_or("select geometry first")?;
        if !refs.iter().all(|r| state.entity_exists(r)) {
            return Err("Needs reselection: missing entity or stale asset revision".into());
        }
        if !refs.iter().all(|r| {
            r.instance_id == first.instance_id
                && r.body_id == first.body_id
                && r.asset_id == first.asset_id
        }) {
            return Err("machining requires one explicitly selected body instance".into());
        }
        let asset = state.asset(first.asset_id).unwrap();
        let instance = state.instance(first.instance_id).unwrap();
        if !instance.placement.is_rigid() {
            return Err("invalid machining placement".into());
        }
        asset.topology.validate_cached()?;
        let body = asset
            .topology
            .bodies
            .iter()
            .find(|b| b.id == first.body_id)
            .ok_or("body missing")?;
        if require_solid && !body.valid_solid {
            return Err("invalid/open solid is inspectable but cannot drive machining".into());
        }
        Ok(Self {
            asset,
            instance,
            body,
        })
    }
    fn silhouette(&self) -> Result<Polygon, String> {
        self.projected_outline(false)
    }
    fn projected_outline(&self, include_holes: bool) -> Result<Polygon, String> {
        if let Some(region) = SILHOUETTES.with(|cache| {
            cache
                .borrow()
                .iter()
                .find(|entry| {
                    entry.body == self.body.id
                        // The outside-only overlay can fill an already resolved
                        // profile silhouette instead of unioning its mesh again.
                        && (entry.include_holes == include_holes || !include_holes)
                        && entry.placement == self.instance.placement
                        && entry
                            .topology
                            .upgrade()
                            .is_some_and(|t| std::sync::Arc::ptr_eq(&t, &self.asset.topology))
                })
                .and_then(|entry| {
                    if entry.include_holes && !include_holes {
                        Polygon::new(entry.region.outer().clone()).ok()
                    } else {
                        Some(entry.region.clone())
                    }
                })
        }) {
            return Ok(region);
        }
        let projection = self.asset.topology.projection.as_ref().ok_or(
            "Reimport this STEP file to select its XY silhouette; this project has no independent CAD projection data"
        )?;
        if projection.chord_tolerance > MACHINING_CHORD_TOLERANCE / 2. {
            return Err(
                "CAD projection exceeds the machining tolerance; reimport the STEP file".into(),
            );
        }
        let face_ids: BTreeSet<_> = self
            .asset
            .topology
            .faces
            .iter()
            .filter(|f| f.body_id == self.body.id)
            .map(|f| f.id)
            .collect();
        let covered: BTreeSet<_> = projection
            .mesh
            .triangle_faces
            .iter()
            .copied()
            .filter(|id| face_ids.contains(id))
            .collect();
        if covered != face_ids || covered.is_empty() {
            return Err("Incomplete CAD projection data; reimport the STEP file".into());
        }
        let mut triangles = Vec::new();
        for (indices, face) in projection
            .mesh
            .triangles
            .iter()
            .zip(&projection.mesh.triangle_faces)
        {
            if !face_ids.contains(face) {
                continue;
            }
            let points: Vec<_> = indices
                .iter()
                .map(|i| {
                    let p = self.point(projection.mesh.vertices[*i as usize]);
                    Point::new(p[0], p[1])
                })
                .collect();
            let contour = Contour::new(points);
            // Vertical triangles project to a segment. The planar boolean handles
            // rounding and tiny slivers consistently with every other strategy.
            if contour.area() > 0. {
                triangles.push(Polygon::new(contour).map_err(|e| e.to_string())?);
            }
        }
        let regions = cam_geo::union(&triangles, &[]).map_err(|e| e.to_string())?;
        // Preserve every projected component. Tiny seams introduced by curve
        // discretization/grid rounding close under the declared allowance;
        // genuinely separate outlines still fail below, never become a hull.
        let envelopes: Vec<_> = if include_holes {
            regions
        } else {
            regions
                .iter()
                .map(|region| Polygon::new(region.outer().clone()))
                .collect::<Result<_, _>>()
                .map_err(|e| e.to_string())?
        };
        let expanded = cam_geo::offset(
            &envelopes,
            silhouette_allowance(projection.chord_tolerance),
            cam_geo::JoinStyle::Round,
        )
        .map_err(|e| e.to_string())?;
        let mut expanded = cam_geo::union(&expanded, &[]).map_err(|e| e.to_string())?;
        if expanded.len() != 1 {
            return Err("The projected outline cannot form one profile boundary".into());
        }
        let region = expanded.remove(0);
        SILHOUETTES.with(|cache| {
            let mut cache = cache.borrow_mut();
            cache.retain(|entry| entry.topology.strong_count() > 0);
            if cache.len() >= 8 {
                cache.remove(0);
            }
            cache.push(CachedSilhouette {
                topology: std::sync::Arc::downgrade(&self.asset.topology),
                body: self.body.id,
                placement: self.instance.placement,
                include_holes,
                region: region.clone(),
            });
        });
        Ok(region)
    }
    fn face(&self, id: u64) -> Result<&CadFace, String> {
        self.asset
            .topology
            .faces
            .iter()
            .find(|f| f.id == id && f.body_id == self.body.id)
            .ok_or_else(|| "face missing".into())
    }
    fn edge(&self, id: u64) -> Result<&CadEdge, String> {
        self.asset
            .topology
            .edges
            .iter()
            .find(|f| f.id == id && f.body_id == self.body.id)
            .ok_or_else(|| "edge missing".into())
    }
    fn wire(&self, id: u64) -> Result<&CadLoop, String> {
        self.asset
            .topology
            .loops
            .iter()
            .find(|f| f.id == id && f.body_id == self.body.id)
            .ok_or_else(|| "loop missing".into())
    }
    fn point(&self, p: [f64; 3]) -> [f64; 3] {
        self.instance.placement.transform_point(p)
    }
    fn vector(&self, p: [f64; 3]) -> [f64; 3] {
        self.instance.placement.transform_vector(p)
    }
    fn plane(&self, f: &CadFace) -> Result<(f64, [f64; 3]), String> {
        match f.surface {
            CadSurface::Plane { origin, normal } => {
                let n = self.vector(normal);
                if n[0].abs() > T || n[1].abs() > T {
                    return Err("selected plane is not aligned with setup Z; align setup Z with the target face normal".into());
                }
                Ok((self.point(origin)[2], n))
            }
            _ => Err("select a planar face".into()),
        }
    }
    /// Conservative B-rep bounds in the machining frame. All eight corners are
    /// needed when the instance or setup is rotated; display meshes are unused.
    fn bounds(&self, bounds: [[f64; 3]; 2]) -> [[f64; 3]; 2] {
        let mut result = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        for x in [bounds[0][0], bounds[1][0]] {
            for y in [bounds[0][1], bounds[1][1]] {
                for z in [bounds[0][2], bounds[1][2]] {
                    let p = self.point([x, y, z]);
                    for axis in 0..3 {
                        result[0][axis] = result[0][axis].min(p[axis]);
                        result[1][axis] = result[1][axis].max(p[axis]);
                    }
                }
            }
        }
        result
    }
    fn footprint(&self, bounds: [[f64; 3]; 2]) -> Result<Polygon, String> {
        // Preserve a conservative enclosure after planar-grid quantization,
        // including for zero-width projections.
        let margin = T.max(cam_geo::GRID_MM);
        Polygon::new(Contour::new(vec![
            Point::new(bounds[0][0] - margin, bounds[0][1] - margin),
            Point::new(bounds[1][0] + margin, bounds[0][1] - margin),
            Point::new(bounds[1][0] + margin, bounds[1][1] + margin),
            Point::new(bounds[0][0] - margin, bounds[1][1] + margin),
        ]))
        .map_err(|e| e.to_string())
    }
    fn vertical(&self, f: &CadFace) -> bool {
        match f.surface {
            CadSurface::Plane { normal, .. } => self.vector(normal)[2].abs() < T,
            CadSurface::Cylinder { axis, .. } | CadSurface::LinearExtrusion { axis, .. } => {
                let n = self.vector(axis);
                n[0].abs() < T && n[1].abs() < T
            }
            _ => false,
        }
    }
    fn curve_points(
        &self,
        e: &CadEdge,
        reversed: bool,
        tolerance: f64,
    ) -> Result<Vec<[f64; 3]>, String> {
        let mut p = match e.curve {
            CadCurve::Line { start, end } => vec![self.point(start), self.point(end)],
            CadCurve::CircleArc {
                center,
                normal,
                x_axis,
                radius,
                start_angle,
                end_angle,
            } => {
                let sweep = end_angle - start_angle;
                if sweep.abs() <= T || sweep.abs() > TAU + T {
                    return Err("invalid circular arc parameter interval".into());
                }
                let step = 2.0 * (1.0 - (tolerance / radius).min(1.0)).acos();
                let segments = (sweep.abs() / step).ceil().max(2.0) as usize;
                if segments > 65536 {
                    return Err("machining curve exceeds approximation resource budget".into());
                }
                let y = cross(normal, x_axis);
                (0..=segments)
                    .map(|i| {
                        let a = start_angle + sweep * i as f64 / segments as f64;
                        self.point(std::array::from_fn(|k| {
                            center[k] + radius * (x_axis[k] * a.cos() + y[k] * a.sin())
                        }))
                    })
                    .collect()
            }
            CadCurve::BezierSpans { ref spans } => approximate_bezier_spans(spans, tolerance)?
                .into_iter()
                .map(|p| self.point(p))
                .collect(),
            _ => {
                return Err(
                    "unsupported exact boundary curve; display tessellation cannot be machined"
                        .into(),
                )
            }
        };
        if reversed {
            p.reverse();
        }
        Ok(p)
    }
    /// Positive-weight Bezier curves lie inside their control hull. Use it for
    /// conservative height/planarity certificates, never just sampled vertices.
    fn curve_support_points(&self, edge: &CadEdge) -> Result<Vec<[f64; 3]>, String> {
        if let CadCurve::BezierSpans { spans } = &edge.curve {
            Ok(spans
                .iter()
                .flat_map(|s| &s.poles)
                .map(|p| self.point(*p))
                .collect())
        } else {
            self.curve_points(edge, false, MACHINING_CHORD_TOLERANCE)
        }
    }
    fn contour(&self, w: &CadLoop) -> Result<(Contour, f64), String> {
        if w.edge_uses.is_empty() {
            return Err("empty boundary loop".into());
        }
        let mut points: Vec<[f64; 3]> = vec![];
        for u in &w.edge_uses {
            let segment =
                self.curve_points(self.edge(u.edge_id)?, u.reversed, MACHINING_CHORD_TOLERANCE)?;
            if let Some(last) = points.last() {
                if distance(*last, segment[0]) > T {
                    return Err("boundary edges do not form an ordered closed loop".into());
                }
            }
            if !points.is_empty() {
                points.pop();
            }
            points.extend(segment);
        }
        if distance(points[0], *points.last().unwrap()) > T {
            return Err("open boundary cannot drive an operation".into());
        }
        points.pop();
        let z = points[0][2];
        if points.iter().any(|p| (p[2] - z).abs() > T)
            || w.edge_uses.iter().any(|u| {
                self.edge(u.edge_id)
                    .and_then(|e| self.curve_support_points(e))
                    .is_ok_and(|p| p.iter().any(|p| (p[2] - z).abs() > T))
            })
        {
            return Err("boundary loop is not coplanar in setup XY".into());
        }
        let c = Contour::new(points.into_iter().map(|p| Point::new(p[0], p[1])).collect());
        if !c.is_valid() || c.area() <= T * T {
            return Err("degenerate machining boundary".into());
        }
        Ok((c, z))
    }
    fn outer_wire(&self, face: &CadFace) -> Result<&CadLoop, String> {
        face.loop_ids
            .iter()
            .find_map(|&id| self.wire(id).ok().filter(|wire| wire.is_outer))
            .ok_or("Face has no outer boundary".into())
    }
    fn region(&self, f: &CadFace) -> Result<Polygon, String> {
        self.region_from_loops(f.loop_ids.iter().copied())
    }
    fn region_from_loops(
        &self,
        loop_ids: impl IntoIterator<Item = EntityId>,
    ) -> Result<Polygon, String> {
        let mut outer = None;
        let mut holes = vec![];
        for id in loop_ids {
            let w = self.wire(id)?;
            let (c, _) = self.contour(w)?;
            if w.is_outer {
                if outer.replace(c).is_some() {
                    return Err("ambiguous multiple outer boundaries".into());
                }
            } else {
                holes.push(c);
            }
        }
        Polygon::with_holes(outer.ok_or("face has no outer boundary")?, holes)
            .map_err(|e| e.to_string())
    }
    fn extent(&self, f: &CadFace) -> Result<(f64, f64), String> {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for id in &f.loop_ids {
            for u in &self.wire(*id)?.edge_uses {
                let e = self.edge(u.edge_id)?;
                for p in self.curve_support_points(e)? {
                    min = min.min(p[2]);
                    max = max.max(p[2]);
                }
                // Analytic extrema rather than display/curve sample extrema.
                if let CadCurve::CircleArc {
                    center,
                    normal,
                    x_axis,
                    radius,
                    start_angle,
                    end_angle,
                } = e.curve
                {
                    let x = self.vector(x_axis)[2];
                    let y = self.vector(cross(normal, x_axis))[2];
                    let z = self.point(center)[2];
                    for a in [y.atan2(x), y.atan2(x) + std::f64::consts::PI] {
                        for k in -2..=2 {
                            let a = a + k as f64 * TAU;
                            if a >= start_angle.min(end_angle) - T
                                && a <= start_angle.max(end_angle) + T
                            {
                                let v = z + radius * (x * a.cos() + y * a.sin());
                                min = min.min(v);
                                max = max.max(v);
                            }
                        }
                    }
                }
            }
        }
        if !min.is_finite() {
            return Err("face has no exact boundary extent".into());
        }
        Ok((min, max))
    }
    fn walls(&self, w: &CadLoop) -> Result<Vec<&CadFace>, String> {
        let mut ids = BTreeSet::new();
        for u in &w.edge_uses {
            let e = self.edge(u.edge_id)?;
            let other: BTreeSet<_> = e
                .face_ids
                .iter()
                .copied()
                .filter(|id| *id != w.face_id)
                .collect();
            if other.len() != 1 {
                return Err("boundary does not have unambiguous solid wall adjacency".into());
            }
            ids.extend(other);
        }
        ids.into_iter()
            .map(|id| {
                let f = self.face(id)?;
                if !self.vertical(f) {
                    return Err(
                        "sloped or unsupported boundary needs vertical constant-section walls"
                            .into(),
                    );
                }
                Ok(f)
            })
            .collect()
    }
    /// Certify the open approach above a locally qualified pocket/bore. Curved
    /// faces elsewhere on the body do not matter, but overlapping unknown
    /// surfaces must never be silently treated as empty space.
    fn roofs_over(&self, region: &Polygon, entry: f64) -> Result<(), String> {
        self.roofs_over_except(region, entry, &BTreeSet::new())
    }

    /// Skip only entry surfaces independently certified to widen away from the
    /// bore. Other overlapping faces still have to pass the roof checks.
    fn roofs_over_except(
        &self,
        region: &Polygon,
        entry: f64,
        open_entries: &BTreeSet<EntityId>,
    ) -> Result<(), String> {
        for f in self
            .asset
            .topology
            .faces
            .iter()
            .filter(|f| f.body_id == self.body.id)
        {
            if open_entries.contains(&f.id) {
                continue;
            }
            let bounds = self.bounds(f.bounds);
            if bounds[1][2] <= entry + T {
                continue;
            }
            let overlap =
                cam_geo::intersection(std::slice::from_ref(region), &[self.footprint(bounds)?])
                    .map_err(|e| e.to_string())?;
            if overlap.is_empty() {
                continue;
            }
            if self.vertical(f) {
                continue;
            }
            if let Ok((z, n)) = self.plane(f) {
                if z <= entry + T || n[2] > 0.0 {
                    continue;
                }
                // A planar roof may have a hole giving access to this feature.
                // Unknown trim curves remain blocked by their conservative bounds.
                if let Ok(protected) = self.region(f) {
                    let overlap = cam_geo::intersection(std::slice::from_ref(region), &[protected])
                        .map_err(|e| e.to_string())?;
                    if overlap.is_empty() {
                        continue;
                    }
                }
            }
            return Err(format!(
                "overhang/undercut or unsupported surface on face {} blocks vertical access above the selected feature", f.id
            ));
        }
        Ok(())
    }

    /// Surface type alone is insufficient: a vertical wall with intermediate
    /// trims can hide a side opening or a roof. Only full-height walls bounded
    /// by horizontal rims and axial seams establish a constant-section void.
    fn certify_wall(&self, wall: &CadFace, bottom: f64, top: f64) -> Result<(), String> {
        if wall.loop_ids.len() != 1
            && !(matches!(wall.surface, CadSurface::Cylinder { .. }) && wall.loop_ids.len() == 2)
        {
            return Err("wall has intermediate openings or ambiguous trims".into());
        }
        for id in &wall.loop_ids {
            for usage in &self.wire(*id)?.edge_uses {
                let edge = self.edge(usage.edge_id)?;
                let points = self.curve_support_points(edge)?;
                let horizontal = points.iter().all(|p| (p[2] - bottom).abs() < T)
                    || points.iter().all(|p| (p[2] - top).abs() < T);
                let axial = matches!(
                    edge.curve,
                    CadCurve::Line { .. } | CadCurve::BezierSpans { .. }
                ) && points
                    .iter()
                    .all(|p| (p[0] - points[0][0]).abs() < T && (p[1] - points[0][1]).abs() < T);
                if !horizontal && !axial {
                    return Err(format!(
                        "wall has intermediate openings or nonconstant-section trims (face {}, edge {})", wall.id, edge.id
                    ));
                }
            }
        }
        Ok(())
    }

    /// Open profiles follow a selected planar curve through the end transitions.
    /// An end may trim a vertical wall obliquely; it need not remain an axial
    /// seam. Keep the closed-feature certificate for pockets and closed loops.
    fn certify_open_profile_wall(
        &self,
        wall: &CadFace,
        bottom: f64,
        top: f64,
    ) -> Result<(), String> {
        if self.certify_wall(wall, bottom, top).is_ok() {
            return Ok(());
        }
        if wall.loop_ids.len() != 1 {
            return Err("profile wall has intermediate openings or ambiguous trims".into());
        }
        let wire = self.wire(wall.loop_ids[0])?;
        let mut uses = Vec::new();
        for usage in &wire.edge_uses {
            let edge = self.edge(usage.edge_id)?;
            let mut points = self.curve_support_points(edge)?;
            if usage.reversed {
                points.reverse();
            }
            let rim = if points.iter().all(|p| (p[2] - top).abs() < T) {
                Some(true)
            } else if points.iter().all(|p| (p[2] - bottom).abs() < T) {
                Some(false)
            } else {
                None
            };
            if rim.is_none()
                && !matches!(
                    edge.curve,
                    CadCurve::Line { .. } | CadCurve::BezierSpans { .. }
                )
            {
                return Err(format!(
                    "profile wall end Edge {} needs an exact end trim",
                    edge.id
                ));
            }
            uses.push((rim, points));
        }
        let first_rim = uses
            .iter()
            .position(|(rim, _)| rim.is_some())
            .ok_or("profile wall has no horizontal rims")?;
        uses.rotate_left(first_rim);
        if !uses.iter().any(|(rim, _)| *rim == Some(true))
            || !uses.iter().any(|(rim, _)| *rim == Some(false))
        {
            return Err("profile wall needs both an upper and a lower horizontal rim".into());
        }
        let mut caps = 0;
        let mut index = 0;
        while index < uses.len() {
            if uses[index].0.is_some() {
                index += 1;
                continue;
            }
            let before = uses[index - 1].0.unwrap();
            let mut heights = vec![if before { top } else { bottom }];
            while index < uses.len() && uses[index].0.is_none() {
                heights.extend(uses[index].1.iter().map(|p| p[2]));
                index += 1;
            }
            let after = uses[index % uses.len()].0.unwrap();
            if before == after {
                return Err("profile wall has an intermediate opening in a rim".into());
            }
            heights.push(if after { top } else { bottom });
            if (heights[1] - heights[0]).abs() > T
                || (heights[heights.len() - 2] - heights[heights.len() - 1]).abs() > T
                || heights.iter().any(|z| *z < bottom - T || *z > top + T)
            {
                return Err("profile wall end trims must connect its two horizontal rims within the cutting interval".into());
            }
            caps += 1;
        }
        if caps != 2 {
            return Err("profile wall needs exactly two end transitions".into());
        }
        Ok(())
    }

    fn certify_pocket_rim(
        &self,
        wall: &CadFace,
        floor_wire: &CadLoop,
        top: f64,
    ) -> Result<(), String> {
        let mut found = false;
        for id in &wall.loop_ids {
            for usage in &self.wire(*id)?.edge_uses {
                let edge = self.edge(usage.edge_id)?;
                let points = self.curve_support_points(edge)?;
                if !points.iter().all(|p| (p[2] - top).abs() < T) {
                    continue;
                }
                found = true;
                let others: Vec<_> = edge.face_ids.iter().filter(|id| **id != wall.id).collect();
                if others.len() != 1 {
                    return Err("pocket opening has ambiguous rim adjacency".into());
                }
                let entry = self.face(*others[0])?;
                let (z, normal) = self.plane(entry)?;
                if (z - top).abs() > T || normal[2] <= 0.0 {
                    return Err("pocket opening needs an upward planar entry face".into());
                }
                let rim = entry
                    .loop_ids
                    .iter()
                    .find_map(|id| {
                        self.wire(*id)
                            .ok()
                            .filter(|w| w.edge_uses.iter().any(|u| u.edge_id == edge.id))
                    })
                    .ok_or("pocket entry is missing its rim loop")?;
                if rim.is_outer == floor_wire.is_outer {
                    return Err(
                        "pocket entry covers the selected void or has ambiguous island adjacency"
                            .into(),
                    );
                }
            }
        }
        if !found {
            return Err("pocket wall has no certified opening rim".into());
        }
        Ok(())
    }

    /// A complete inward cylindrical wall and a disk floor certify the local
    /// void in a valid solid. Requiring its entry at/above the entire body also
    /// excludes overhangs, including surfaces outside the analytic subset.
    fn blind_bore_profile(
        &self,
        refs: &[EntityRef],
    ) -> Result<(Vec<Polygon>, Vec<ResolvedHole>, CuttingHeights), String> {
        if refs.len() != 1 || refs[0].kind != EntityKind::Loop {
            return Err(
                "Profile requires one closed coplanar loop; select one boundary per operation"
                    .into(),
            );
        }
        let wire = self.wire(refs[0].id)?;
        let floor = self.face(wire.face_id)?;
        if matches!(floor.surface, CadSurface::Cylinder { .. }) {
            return Err("This loop wraps a cylindrical wall and includes its seam. Select the circular loop on the planar recess floor for an inside Profile".into());
        }
        let (floor_z, normal) = self.plane(floor)?;
        if !wire.is_outer || floor.loop_ids.len() != 1 || normal[2] <= 0.0 {
            return Err("Inside Profile requires the circular floor boundary of one blind bore facing setup Z".into());
        }
        let mut cylinder = None;
        for usage in &wire.edge_uses {
            let edge = self.edge(usage.edge_id)?;
            if !matches!(edge.curve, CadCurve::CircleArc { .. }) {
                return Err(
                    "Inside CAD Profile currently requires a circular blind-bore floor".into(),
                );
            }
            for id in &edge.face_ids {
                if *id == floor.id {
                    continue;
                }
                let face = self.face(*id)?;
                if !matches!(face.surface, CadSurface::Cylinder { inward: true, .. }) {
                    return Err(
                        "The floor boundary must adjoin an inward cylindrical bore wall".into(),
                    );
                }
                cylinder.get_or_insert(face.id);
            }
        }
        let cylinder = cylinder.ok_or("No cylindrical wall adjoining the selected floor")?;
        let (_, entry_z) = self.extent(self.face(cylinder)?)?;
        for x in [self.body.bounds[0][0], self.body.bounds[1][0]] {
            for y in [self.body.bounds[0][1], self.body.bounds[1][1]] {
                for z in [self.body.bounds[0][2], self.body.bounds[1][2]] {
                    if self.point([x, y, z])[2] > entry_z + T {
                        return Err("Material above the bore entry prevents local profile certification; choose a setup with unobstructed access from above the body".into());
                    }
                }
            }
        }
        let mut bore = refs[0].clone();
        bore.kind = EntityKind::Face;
        bore.id = cylinder;
        let (_, holes, heights) = derive_holes(self, &[bore], true)?;
        if holes.len() != 1 || (heights.bottom - floor_z).abs() > T {
            return Err("The selected floor does not terminate one constant-diameter bore".into());
        }
        let (contour, _) = self.contour(wire)?;
        let hole = &holes[0];
        if contour
            .points()
            .iter()
            .any(|p| (p.distance(hole.center) - hole.diameter / 2.0).abs() > T)
        {
            return Err("The selected floor boundary does not match its certified bore".into());
        }
        Ok((
            vec![Polygon::new(contour).map_err(|e| e.to_string())?],
            vec![],
            heights,
        ))
    }
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt()
}

fn derive(
    state: &GeometryState,
    binding: &GeometryBinding,
    stock_top: f64,
) -> Result<(Vec<Polygon>, Vec<ResolvedHole>, CuttingHeights), String> {
    if let Some(input) = pocket_regions::resolve(state, binding, false) {
        let input = input?;
        return Ok((input.regions, input.holes, input.heights));
    }
    if matches!(
        binding.recipe,
        ExtractionRecipe::AdaptiveClearing
            | ExtractionRecipe::ParallelFinishing
            | ExtractionRecipe::SpiralFinishing
    ) {
        return Ok((
            Vec::new(),
            Vec::new(),
            adaptive_heights(state, &binding.selected, stock_top)?,
        ));
    }
    if let ExtractionRecipe::ProfileEntries { entries } = &binding.recipe {
        let (profiles, heights) = profile_entries::extract(state, entries, stock_top)?;
        let regions = profiles
            .into_iter()
            .filter(|p| p.closed)
            .map(|p| Polygon::new(p.chain).map_err(|e| e.to_string()))
            .collect::<Result<Vec<_>, _>>()?;
        return Ok((regions, vec![], heights));
    }
    if binding.recipe == ExtractionRecipe::ChamferFaces {
        let (faces, heights) = chamfer_faces::extract(state, &binding.selected)?;
        let regions = faces
            .iter()
            .map(|face| {
                Polygon::new(Contour::new(
                    [face.lower[0], face.lower[1], face.upper[1], face.upper[0]]
                        .into_iter()
                        .map(|p| Point::new(p[0], p[1]))
                        .collect(),
                ))
                .map_err(|e| e.to_string())
            })
            .collect::<Result<_, _>>()?;
        return Ok((regions, Vec::new(), heights));
    }
    if surface_geometry::is_surface_recipe(&binding.recipe) {
        return surface_geometry::derive(state, binding);
    }
    if binding.recipe == ExtractionRecipe::ProfileContours {
        let (profiles, heights) = profile_contours::extract(state, &binding.selected)?;
        let regions = profiles
            .into_iter()
            .filter(|p| p.closed)
            .map(|p| Polygon::new(p.chain).map_err(|e| e.to_string()))
            .collect::<Result<Vec<_>, _>>()?;
        return Ok((regions, vec![], heights));
    }
    let view = View::new(state, &binding.selected)?;
    if matches!(
        binding.recipe,
        ExtractionRecipe::ProfileLoop { side: Side::Inside }
    ) {
        return view.blind_bore_profile(&binding.selected);
    }
    let refs = &binding.selected;
    if refs.len() == 1 && refs[0].kind == EntityKind::Silhouette {
        let bounds = view.bounds(view.body.bounds);
        let heights = match binding.recipe {
            ExtractionRecipe::ProfileLoop { side: Side::Outside } | ExtractionRecipe::ProfileSilhouette => CuttingHeights {
                top: bounds[1][2], bottom: bounds[0][2],
            },
            ExtractionRecipe::FaceRegion => CuttingHeights {
                top: stock_top, bottom: bounds[1][2],
            },
            _ => return Err("An XY silhouette supports outside Profile or Face; select a face for Pocket or Drill".into()),
        };
        heights.validate()?;
        let region = if binding.recipe == ExtractionRecipe::ProfileSilhouette {
            view.projected_outline(true)?
        } else {
            view.silhouette()?
        };
        return Ok((vec![region], vec![], heights));
    }
    match binding.recipe {
        ExtractionRecipe::PocketFloor => {
            if refs.len() != 1 || refs[0].kind != EntityKind::Face {
                return Err("Pocket requires one planar floor face".into());
            }
            let f = view.face(refs[0].id)?;
            let (floor, normal) = view.plane(f)?;
            if normal[2] < 0.0 {
                return Err("floor normal must face the machining direction".into());
            }
            let region = view.region_from_loops(pocket_regions::floor_loop_ids(&view, f, floor)?)?;
            for face in view
                .asset
                .topology
                .faces
                .iter()
                .filter(|face| face.body_id == view.body.id)
            {
                if let Ok((z, n)) = view.plane(face) {
                    if z > floor + T && n[2] < 0.0 {
                        if cam_geo::intersection(
                            std::slice::from_ref(&region),
                            &[view.footprint(view.bounds(face.bounds))?],
                        )
                        .map_err(|e| e.to_string())?
                        .is_empty()
                        {
                            continue;
                        }
                        let protected = view.region(face).map_err(|_| format!(
                            "unsupported roof boundary on face {} prevents vertical access certification", face.id
                        ))?;
                        if !cam_geo::intersection(std::slice::from_ref(&region), &[protected])
                            .map_err(|e| e.to_string())?
                            .is_empty()
                        {
                            return Err("overhang/undercut blocks vertical access above the selected region".into());
                        }
                    }
                }
            }
            let mut top: Option<f64> = None;
            for &id in &f.loop_ids {
                let wire = view.wire(id)?;
                if pocket_regions::descends_below_floor(&view, wire, floor) {
                    continue;
                }
                for wall in view.walls(wire)? {
                    let (lo, hi) = view.extent(wall)?;
                    if (lo - floor).abs() > T || hi <= floor + T {
                        return Err("floor boundary is open, has a bore below it, or is not a rising pocket wall".into());
                    }
                    view.certify_wall(wall, lo, hi)?;
                    view.certify_pocket_rim(wall, wire, hi)?;
                    if let Some(z) = top {
                        if (z - hi).abs() > T {
                            return Err("pocket walls have ambiguous or different openings".into());
                        }
                    } else {
                        top = Some(hi);
                    }
                }
            }
            let heights = CuttingHeights {
                top: top.ok_or("no pocket walls")?,
                bottom: floor,
            };
            view.roofs_over(&region, heights.top)?;
            heights.validate()?;
            Ok((vec![region], vec![], heights))
        }
        ExtractionRecipe::FaceRegion => {
            if refs.len() != 1 || refs[0].kind != EntityKind::Face {
                return Err("Face requires one planar target face".into());
            }
            let f = view.face(refs[0].id)?;
            let (z, n) = view.plane(f)?;
            if n[2] < 0.0 {
                return Err("facing target must face setup Z".into());
            }
            // Bounds come from the native B-rep, never the display mesh or an
            // approximation of a spline. Transform all eight corners to retain a
            // conservative enclosure in the active setup, even for curved bodies.
            let mut min = [f64::INFINITY; 3];
            let mut max = [f64::NEG_INFINITY; 3];
            let bounds = std::iter::once(view.body.bounds).chain(
                view.asset
                    .topology
                    .faces
                    .iter()
                    .filter(|face| face.body_id == view.body.id)
                    .map(|face| face.bounds),
            );
            for bounds in bounds {
                for x in [bounds[0][0], bounds[1][0]] {
                    for y in [bounds[0][1], bounds[1][1]] {
                        for z in [bounds[0][2], bounds[1][2]] {
                            let point = view.point([x, y, z]);
                            for axis in 0..3 {
                                min[axis] = min[axis].min(point[axis]);
                                max[axis] = max[axis].max(point[axis]);
                            }
                        }
                    }
                }
            }
            for vertex in view
                .asset
                .topology
                .vertices
                .iter()
                .filter(|v| v.body_id == view.body.id)
            {
                let point = view.point(vertex.point);
                for axis in 0..3 {
                    min[axis] = min[axis].min(point[axis]);
                    max[axis] = max[axis].max(point[axis]);
                }
            }
            if max[2] > z + T {
                return Err(
                    "adjacent bosses/protected material above the target prevent whole-body facing"
                        .into(),
                );
            }
            let region = Polygon::new(Contour::new(vec![
                Point::new(min[0], min[1]),
                Point::new(max[0], min[1]),
                Point::new(max[0], max[1]),
                Point::new(min[0], max[1]),
            ]))
            .map_err(|error| error.to_string())?;
            if stock_top <= z {
                return Err("stock top must be above the facing target; increase the positive Z stock allowance".into());
            }
            let heights = CuttingHeights {
                top: stock_top,
                bottom: z,
            };
            heights.validate()?;
            Ok((vec![region], vec![], heights))
        }
        ExtractionRecipe::ProfileLoop { side } => {
            if side != Side::Outside {
                return Err("initial profile certification supports only the outside of a whole-body exterior loop".into());
            }
            if refs.len() != 1 || refs[0].kind != EntityKind::Loop {
                return Err("Profile requires one closed coplanar loop".into());
            }
            let w = view.wire(refs[0].id)?;
            if !w.is_outer {
                return Err("outside profile requires the body's exterior loop".into());
            }
            let (c, z) = view.contour(w)?;
            let region = Polygon::new(c).map_err(|e| e.to_string())?;
            let mut bottom: Option<f64> = None;
            let mut top: Option<f64> = None;
            let walls = view.walls(w)?;
            for wall in &walls {
                let (lo, hi) = view.extent(wall)?;
                if ((hi - z).abs() > T && (lo - z).abs() > T) || lo >= hi - T {
                    return Err(
                        "profile loop must be an upper or lower rim of a vertical exterior wall"
                            .into(),
                    );
                }
                view.certify_wall(wall, lo, hi)?;
                if top.is_some_and(|v| (hi - v).abs() > T) {
                    return Err("profile walls have different entry heights".into());
                }
                top = Some(hi);
                if let Some(v) = bottom {
                    if (lo - v).abs() > T {
                        return Err("profile walls have different depths".into());
                    }
                } else {
                    bottom = Some(lo);
                }
            }
            // Faces enclosed by the silhouette cannot be hit by an outside
            // profile, regardless of their surface/curve type. Remaining planar
            // faces can be checked by their exact boundary; unknown exterior
            // material stays unqualified.
            for face in view
                .asset
                .topology
                .faces
                .iter()
                .filter(|f| f.body_id == view.body.id)
            {
                if face.id == w.face_id || walls.iter().any(|wall| wall.id == face.id) {
                    continue;
                }
                let bounds = view.bounds(face.bounds);
                let footprint = Polygon::new(Contour::new(vec![
                    Point::new(bounds[0][0], bounds[0][1]),
                    Point::new(bounds[1][0], bounds[0][1]),
                    Point::new(bounds[1][0], bounds[1][1]),
                    Point::new(bounds[0][0], bounds[1][1]),
                ]));
                if let Ok(footprint) = footprint {
                    if cam_geo::difference(&[footprint], std::slice::from_ref(&region))
                        .map_err(|e| e.to_string())?
                        .is_empty()
                    {
                        continue;
                    }
                }
                if view.plane(face).is_ok() {
                    if let Ok(protected) = view.region(face) {
                        if cam_geo::difference(&[protected], std::slice::from_ref(&region))
                            .map_err(|e| e.to_string())?
                            .is_empty()
                        {
                            continue;
                        }
                    }
                }
                return Err(format!("material on face {} outside the selected silhouette prevents safe side certification", face.id));
            }
            let heights = CuttingHeights {
                top: top.ok_or("no profile walls")?,
                bottom: bottom.ok_or("no profile walls")?,
            };
            heights.validate()?;
            Ok((vec![region], vec![], heights))
        }
        ExtractionRecipe::DrillHoles => derive_holes(&view, refs, true),
        ExtractionRecipe::BoreCylinders { internal } => derive_holes(&view, refs, internal),
        ExtractionRecipe::ThreadCylinders { internal } => derive_holes(&view, refs, internal),
        ExtractionRecipe::ChamferFaces
        | ExtractionRecipe::ChamferLoop
        | ExtractionRecipe::EngravePath
        | ExtractionRecipe::CarveRegion
        | ExtractionRecipe::PocketBoundary => unreachable!(),
        ExtractionRecipe::ProfileContours | ExtractionRecipe::ProfileEntries { .. } => {
            unreachable!()
        }
        ExtractionRecipe::ProfileSilhouette => Err("Select one body for its silhouette".into()),
        ExtractionRecipe::AdaptiveClearing
        | ExtractionRecipe::ParallelFinishing
        | ExtractionRecipe::SpiralFinishing => unreachable!(),
    }
}
fn derive_holes(
    view: &View<'_>,
    refs: &[EntityRef],
    internal: bool,
) -> Result<(Vec<Polygon>, Vec<ResolvedHole>, CuttingHeights), String> {
    let mut holes: Vec<ResolvedHole> = vec![];
    let mut heights: Option<CuttingHeights> = None;
    for r in refs {
        if r.kind != EntityKind::Face {
            return Err(
                "Select cylindrical bore or boss faces; circular edges alone do not establish a hole"
                    .into(),
            );
        }
        let f = view.face(r.id)?;
        let CadSurface::Cylinder {
            origin,
            axis,
            radius,
            inward,
        } = f.surface
        else {
            return Err("Select cylindrical bore or boss faces".into());
        };
        let center = view.point(origin);
        let a = view.vector(axis);
        if inward != internal || a[0].abs() > T || a[1].abs() > T {
            return Err(
                "Select cylinders of the same internal/external type with axes parallel to setup Z"
                    .into(),
            );
        }
        let (lo, hi) = view.extent(f)?;
        // Merge split cylindrical seam faces into one analytic bore before validating
        // its complete top/bottom circular rims and termination adjacency.
        let group: Vec<_> = view
            .asset
            .topology
            .faces
            .iter()
            .filter(|candidate| candidate.body_id == view.body.id)
            .filter(|candidate| match candidate.surface {
                CadSurface::Cylinder {
                    origin: o,
                    axis: ax,
                    radius: rad,
                    inward: inside,
                } if inside == internal => {
                    let c = view.point(o);
                    let ax = view.vector(ax);
                    (rad - radius).abs() < T
                        && (c[0] - center[0]).abs() < T
                        && (c[1] - center[1]).abs() < T
                        && ax[0].abs() < T
                        && ax[1].abs() < T
                        && view
                            .extent(candidate)
                            .is_ok_and(|(l, h)| (l - lo).abs() < T && (h - hi).abs() < T)
                }
                _ => false,
            })
            .collect();
        let mut rim_edges = BTreeSet::new();
        let mut open_entries = BTreeSet::new();
        let mut top_sweep = 0.0;
        let mut bottom_sweep = 0.0;
        for face in group {
            for id in &face.loop_ids {
                for u in &view.wire(*id)?.edge_uses {
                    let edge = view.edge(u.edge_id)?;
                    if !rim_edges.insert(edge.id) {
                        continue;
                    }
                    match edge.curve {
                        CadCurve::CircleArc {
                            center: c,
                            radius: rad,
                            start_angle,
                            end_angle,
                            ..
                        } => {
                            let c = view.point(c);
                            if (rad - radius).abs() > T
                                || (c[0] - center[0]).abs() > T
                                || (c[1] - center[1]).abs() > T
                            {
                                return Err(
                                    "bore radius or centre varies over the cutting interval".into(),
                                );
                            }
                            if (c[2] - hi).abs() < T {
                                top_sweep += (end_angle - start_angle).abs();
                            } else if (c[2] - lo).abs() < T {
                                bottom_sweep += (end_angle - start_angle).abs();
                            } else {
                                return Err(
                                    "bore has intermediate rims or ambiguous termination".into()
                                );
                            }
                            let mut terminations = 0;
                            for fid in &edge.face_ids {
                                let adjacent = view.face(*fid)?;
                                if matches!(adjacent.surface, CadSurface::Cylinder { .. }) {
                                    continue;
                                }
                                if internal
                                    && (c[2] - hi).abs() < T
                                    && matches!(adjacent.surface, CadSurface::Torus { .. })
                                {
                                    bore_entry::certify(view, adjacent, face, edge)?;
                                    open_entries.insert(adjacent.id);
                                    terminations += 1;
                                    continue;
                                }
                                let (z,n)=view.plane(adjacent).map_err(|_|"blind drill tips, blends or ambiguous hole terminations need explicit qualification".to_string())?;
                                if (z - c[2]).abs() > T || ((c[2] - hi).abs() < T && n[2] < 0.0) {
                                    return Err(
                                        "hole opening is covered or its termination is ambiguous"
                                            .into(),
                                    );
                                }
                                if (c[2] - hi).abs() < T {
                                    let open_rim = adjacent.loop_ids.iter().any(|id| {
                                        view.wire(*id).is_ok_and(|w| {
                                            w.is_outer != internal
                                                && w.edge_uses.iter().any(|u| u.edge_id == edge.id)
                                        })
                                    });
                                    if !open_rim {
                                        return Err("bore entry is capped or lacks a certified inner opening rim".into());
                                    }
                                }
                                terminations += 1;
                            }
                            if terminations != 1 {
                                return Err(
                                    "bore rims require one certified planar entry/termination face"
                                        .into(),
                                );
                            }
                        }
                        CadCurve::Line { start, end } => {
                            let start = view.point(start);
                            let end = view.point(end);
                            if (start[0] - end[0]).abs() > T || (start[1] - end[1]).abs() > T {
                                return Err("cylindrical trim is not an axial seam".into());
                            }
                        }
                        _ => return Err("unsupported cylindrical bore trim".into()),
                    }
                }
            }
        }
        if (top_sweep - TAU).abs() > T || (bottom_sweep - TAU).abs() > T {
            return Err(
                "split cylindrical faces do not establish a complete constant-diameter bore".into(),
            );
        }
        let h = CuttingHeights {
            top: hi,
            bottom: lo,
        };
        h.validate()?;
        let circle = Contour::new(
            (0..128)
                .map(|i| {
                    let angle = TAU * i as f64 / 128.;
                    Point::new(
                        center[0] + radius / (std::f64::consts::PI / 128.).cos() * angle.cos(),
                        center[1] + radius / (std::f64::consts::PI / 128.).cos() * angle.sin(),
                    )
                })
                .collect(),
        );
        view.roofs_over_except(
            &Polygon::new(circle).map_err(|e| e.to_string())?,
            hi,
            &open_entries,
        )?;
        if let Some(existing) = heights {
            if (h.top - existing.top).abs() > T
                || (h.bottom - existing.bottom).abs() > T
                || holes
                    .first()
                    .is_some_and(|p| (p.diameter - 2.0 * radius).abs() > T)
            {
                return Err(
                    "holes with different entry, depth or diameter require separate operations"
                        .into(),
                );
            }
        } else {
            heights = Some(h);
        }
        let p = Point::new(center[0], center[1]);
        if !holes.iter().any(|h| h.center.distance(p) < T) {
            holes.push(ResolvedHole {
                center: p,
                diameter: 2.0 * radius,
            });
        }
    }
    Ok((vec![], holes, heights.ok_or("select cylindrical holes")?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    struct Fixture {
        t: TopologySnapshot,
        next: u64,
    }
    impl Fixture {
        fn new() -> Self {
            Self {
                t: TopologySnapshot {
                    bodies: vec![CadBody {
                        id: 1,
                        name: "Analytic plate".into(),
                        valid_solid: true,
                        bounds: [[0., 0., -5.], [20., 20., 0.]],
                    }],
                    ..Default::default()
                },
                next: 2,
            }
        }
        fn id(&mut self) -> u64 {
            let id = self.next;
            self.next += 1;
            id
        }
        fn vertex(&mut self, p: [f64; 3]) -> u64 {
            if let Some(v) = self.t.vertices.iter().find(|v| v.point == p) {
                return v.id;
            }
            let id = self.id();
            self.t.vertices.push(CadVertex {
                id,
                body_id: 1,
                point: p,
            });
            id
        }
        fn face(&mut self, rings: Vec<Vec<[f64; 3]>>, normal: [f64; 3]) -> u64 {
            let id = self.id();
            let mut loop_ids = vec![];
            let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
            for (i, ring) in rings.iter().enumerate() {
                let wid = self.id();
                loop_ids.push(wid);
                let mut edge_uses = vec![];
                for j in 0..ring.len() {
                    let a = ring[j];
                    let b = ring[(j + 1) % ring.len()];
                    for k in 0..3 {
                        bounds[0][k] = bounds[0][k].min(a[k]);
                        bounds[1][k] = bounds[1][k].max(a[k]);
                    }
                    let va = self.vertex(a);
                    let vb = self.vertex(b);
                    let ei = self
                        .t
                        .edges
                        .iter()
                        .position(|e| e.vertices == [va, vb] || e.vertices == [vb, va]);
                    let (edge_id, reversed) = if let Some(ei) = ei {
                        let e = &mut self.t.edges[ei];
                        e.face_ids.push(id);
                        (e.id, e.vertices != [va, vb])
                    } else {
                        let edge_id = self.id();
                        self.t.edges.push(CadEdge {
                            id: edge_id,
                            body_id: 1,
                            vertices: [va, vb],
                            face_ids: vec![id],
                            curve: CadCurve::Line { start: a, end: b },
                        });
                        (edge_id, false)
                    };
                    edge_uses.push(EdgeUse { edge_id, reversed });
                }
                self.t.loops.push(CadLoop {
                    id: wid,
                    body_id: 1,
                    face_id: id,
                    edge_uses,
                    is_outer: i == 0,
                });
            }
            self.t.faces.push(CadFace {
                id,
                body_id: 1,
                surface: CadSurface::Plane {
                    origin: rings[0][0],
                    normal,
                },
                loop_ids,
                bounds,
            });
            id
        }
        fn walls(&mut self, x0: f64, x1: f64, y0: f64, y1: f64, lo: f64, hi: f64) {
            self.face(
                vec![vec![[x0, y0, lo], [x1, y0, lo], [x1, y0, hi], [x0, y0, hi]]],
                [0., -1., 0.],
            );
            self.face(
                vec![vec![[x1, y0, lo], [x1, y1, lo], [x1, y1, hi], [x1, y0, hi]]],
                [1., 0., 0.],
            );
            self.face(
                vec![vec![[x1, y1, lo], [x0, y1, lo], [x0, y1, hi], [x1, y1, hi]]],
                [0., 1., 0.],
            );
            self.face(
                vec![vec![[x0, y1, lo], [x0, y0, lo], [x0, y0, hi], [x0, y1, hi]]],
                [-1., 0., 0.],
            );
        }
        fn state(self) -> GeometryState {
            GeometryState {
                assets: vec![GeometryAsset {
                    import_options: None,
                    import_diagnostics: vec![],
                    source_fingerprint: String::new(),
                    id: 1,
                    revision: 1,
                    name: "analytic".into(),
                    source_format: CadFormat::Step,
                    source_path: None,
                    source_data: vec![].into(),
                    source_units: SourceUnits::Millimetres,
                    backend: "analytic test fixture (not imported STEP)".into(),
                    topology: self.t.into(),
                }],
                instances: vec![GeometryInstance {
                    id: 1,
                    asset_id: 1,
                    body_id: 1,
                    name: "part".into(),
                    placement: Default::default(),
                    visible: true,
                }],
                ..Default::default()
            }
        }
    }
    fn square(x0: f64, x1: f64, y0: f64, y1: f64, z: f64) -> Vec<[f64; 3]> {
        vec![[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]]
    }
    fn reference(id: u64, kind: EntityKind) -> EntityRef {
        EntityRef {
            asset_id: 1,
            revision: 1,
            instance_id: 1,
            body_id: 1,
            kind,
            id,
        }
    }
    fn pocket() -> (GeometryState, EntityRef) {
        let mut f = Fixture::new();
        let floor = f.face(
            vec![square(2., 18., 2., 18., -3.), square(8., 12., 8., 12., -3.)],
            [0., 0., 1.],
        );
        f.walls(2., 18., 2., 18., -3., 0.);
        f.walls(8., 12., 8., 12., -3., 0.);
        f.face(
            vec![square(0., 20., 0., 20., 0.), square(2., 18., 2., 18., 0.)],
            [0., 0., 1.],
        );
        f.face(vec![square(8., 12., 8., 12., 0.)], [0., 0., 1.]);
        f.walls(0., 20., 0., 20., -5., 0.);
        f.face(vec![square(0., 20., 0., 20., -5.)], [0., 0., -1.]);
        (f.state(), reference(floor, EntityKind::Face))
    }
    #[test]
    fn pocket_stock_domains_extend_open_edges_but_continue_rising_walls() {
        let mut f = Fixture::new();
        let floor = f.face(
            vec![square(2., 18., 2., 18., -3.), square(8., 12., 8., 12., -3.)],
            [0., 0., 1.],
        );
        f.face(
            vec![vec![
                [2., 2., -3.],
                [2., 18., -3.],
                [2., 18., 0.],
                [2., 2., 0.],
            ]],
            [-1., 0., 0.],
        );
        f.face(
            vec![vec![
                [18., 2., -3.],
                [18., 18., -3.],
                [18., 18., -5.],
                [18., 2., -5.],
            ]],
            [1., 0., 0.],
        );
        // An inner raised island is protected by the shadow, never used to
        // restrict which side of the outer walls contains the floor.
        f.walls(8., 12., 8., 12., -3., 0.);
        let mut state = f.state();
        let reference = reference(floor, EntityKind::Face);
        let domains = pocket_stock_domains(
            &state,
            std::slice::from_ref(&reference),
            -3.,
            [[-10., -10.], [30., 30.]],
        )
        .unwrap();
        assert_eq!(domains.len(), 1);
        assert!((domains[0].area() - 28. * 40.).abs() < T);
        assert!(
            !domains[0].contains(Point::new(1., 25.)),
            "no wraparound behind the rising wall"
        );
        assert!(
            domains[0].contains(Point::new(25., 25.)),
            "descending and open edges extend to stock"
        );
        assert!(
            domains[0].contains(Point::new(10., 10.)),
            "inner loops do not clip the domain"
        );

        state.instances[0].placement = RigidTransform {
            rotation: [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]],
            translation: [100., 50., 10.],
        };
        let domains =
            pocket_stock_domains(&state, &[reference], 7., [[70., 40.], [110., 80.]]).unwrap();
        assert!((domains[0].area() - 28. * 40.).abs() < T);
        assert!(!domains[0].contains(Point::new(75., 51.)));
        assert!(domains[0].contains(Point::new(75., 75.)));
    }

    #[test]
    fn pocket_stock_domains_do_not_clip_across_concave_floors() {
        let mut f = Fixture::new();
        let floor = f.face(
            vec![vec![
                [2., 2., -3.],
                [18., 2., -3.],
                [18., 8., -3.],
                [8., 8., -3.],
                [8., 18., -3.],
                [2., 18., -3.],
            ]],
            [0., 0., 1.],
        );
        f.face(
            vec![vec![
                [8., 8., -3.],
                [8., 18., -3.],
                [8., 18., 0.],
                [8., 8., 0.],
            ]],
            [1., 0., 0.],
        );
        let state = f.state();
        let domains = pocket_stock_domains(
            &state,
            &[reference(floor, EntityKind::Face)],
            -3.,
            [[-10., -10.], [30., 30.]],
        )
        .unwrap();
        assert_eq!(domains[0].area(), 1600.);
        assert!(
            domains[0].contains(Point::new(15., 5.)),
            "the concave floor extends to both sides of this wall"
        );
    }

    #[test]
    fn pocket_island_boundary_and_translated_heights_are_exact() {
        let (mut state, r) = pocket();
        state.validate().unwrap();
        state.instances[0].placement.translation = [100., -50., 40.];
        let b = resolve_selection(&state, &[r], ExtractionRecipe::PocketFloor, 40.).unwrap();
        let result = resolve_binding(&state, &b).unwrap();
        assert_eq!(
            result.heights,
            CuttingHeights {
                top: 40.,
                bottom: 37.
            }
        );
        assert_eq!(result.regions[0].holes().len(), 1);
        assert_eq!(result.regions[0].area(), 240.);
        assert_eq!(
            result.regions[0].outer().points()[0],
            Point::new(102., -48.)
        );
    }
    #[test]
    fn stale_revisions_and_deeper_overrides_never_use_cached_contours() {
        let (mut state, r) = pocket();
        let mut b = resolve_selection(&state, &[r], ExtractionRecipe::PocketFloor, 0.).unwrap();
        b.overrides.bottom = Some(-4.);
        assert!(resolve_binding(&state, &b)
            .unwrap_err()
            .contains("certified"));
        b.overrides.bottom = None;
        state.assets[0].revision += 1;
        assert!(resolve_binding(&state, &b)
            .unwrap_err()
            .contains("Needs reselection"));
    }
    #[test]
    fn display_mesh_changes_do_not_change_machining_or_fingerprint() {
        let (mut state, r) = pocket();
        let b = resolve_selection(&state, &[r], ExtractionRecipe::PocketFloor, 0.).unwrap();
        let a = resolve_binding(&state, &b).unwrap();
        let t = std::sync::Arc::make_mut(&mut state.assets[0].topology);
        t.mesh.vertices = vec![[0., 0., -3.], [20., 0., -3.], [0., 20., -3.]];
        t.mesh.triangles = vec![[0, 1, 2]];
        t.mesh.triangle_faces = vec![b.selected[0].id];
        assert_eq!(resolve_binding(&state, &b).unwrap(), a);
    }
    #[test]
    fn slope_and_overhang_are_rejected() {
        let (mut state, r) = pocket();
        let id = r.id;
        let t = std::sync::Arc::make_mut(&mut state.assets[0].topology);
        let wall = t
            .faces
            .iter_mut()
            .find(|f| {
                f.id != id && matches!(f.surface,CadSurface::Plane{normal,..}if normal[2]==0.)
            })
            .unwrap();
        wall.surface = CadSurface::Plane {
            origin: [2., 2., -3.],
            normal: [0., 0.6, 0.8],
        };
        assert!(
            resolve_selection(&state, &[r], ExtractionRecipe::PocketFloor, 0.)
                .unwrap_err()
                .contains("sloped")
        );
        let mut f = Fixture::new();
        let floor = f.face(vec![square(2., 18., 2., 18., -3.)], [0., 0., 1.]);
        f.walls(2., 18., 2., 18., -3., 0.);
        f.face(vec![square(2., 6., 2., 18., -1.)], [0., 0., -1.]);
        assert!(resolve_selection(
            &f.state(),
            &[reference(floor, EntityKind::Face)],
            ExtractionRecipe::PocketFloor,
            0.
        )
        .unwrap_err()
        .contains("overhang"));
    }
    fn plate_with_hole() -> (GeometryState, EntityRef) {
        let mut f = Fixture::new();
        let rings = |z| vec![square(0., 20., 0., 20., z), square(8., 12., 8., 12., z)];
        let top = f.face(rings(0.), [0., 0., 1.]);
        f.walls(0., 20., 0., 20., -5., 0.);
        f.walls(8., 12., 8., 12., -5., 0.);
        f.face(rings(-5.), [0., 0., -1.]);
        (f.state(), reference(top, EntityKind::Face))
    }

    #[test]
    fn advisory_pocket_face_bridges_certified_descending_cutouts() {
        let (mut state, face) = plate_with_hole();
        let selection = std::slice::from_ref(&face);
        let binding =
            resolve_selection_advisory(&state, selection, ExtractionRecipe::PocketFloor, 1.).unwrap();
        let resolved = resolve_binding_advisory(&state, &binding).unwrap();
        assert_eq!(
            resolved.heights,
            CuttingHeights { top: 1., bottom: 0. }
        );
        assert_eq!(resolved.regions[0].area(), 400.);
        assert!(resolved.regions[0].holes().is_empty());
        assert!(resolved
            .diagnostics
            .iter()
            .any(|message| message.contains("verification (info)")));

        Arc::make_mut(&mut state.assets[0].topology).bodies[0].valid_solid = false;
        let resolved = resolve_binding_advisory(&state, &binding).unwrap();
        assert_eq!(resolved.regions[0].holes().len(), 1);
    }

    #[test]
    fn advisory_recognition_keeps_exact_geometry_without_wall_or_face_certificates() {
        let (mut state, face) = plate_with_hole();
        let topology = Arc::make_mut(&mut state.assets[0].topology);
        topology.bodies[0].valid_solid = false;
        for face in &mut topology.faces {
            face.surface = CadSurface::Unsupported {
                description: "unclassified spline".into(),
            };
        }
        let selection = std::slice::from_ref(&face);
        for recipe in [
            ExtractionRecipe::ProfileContours,
            ExtractionRecipe::PocketFloor,
            ExtractionRecipe::FaceRegion,
            ExtractionRecipe::CarveRegion,
        ] {
            assert!(verify_selection(&state, selection, recipe.clone(), 1.).is_some());
            let binding = resolve_selection_advisory(&state, selection, recipe, 1.).unwrap();
            let resolved = resolve_binding_advisory(&state, &binding).unwrap();
            assert!(!resolved.regions.is_empty());
            assert!(resolved
                .diagnostics
                .iter()
                .any(|message| message.contains("verification (info)")));
        }
        let mut stale = face;
        stale.revision += 1;
        assert!(resolve_selection_advisory(
            &state,
            &[stale],
            ExtractionRecipe::ProfileContours,
            1.
        )
        .unwrap_err()
        .contains("stale"));
    }

    #[test]
    fn lower_exterior_loop_uses_the_full_certified_wall_interval() {
        let (state, _) = plate_with_hole();
        let wire = state.assets[0]
            .topology
            .loops
            .iter()
            .find(|wire| {
                wire.is_outer
                    && state.assets[0].topology.faces.iter().any(|face| {
                        face.id == wire.face_id
                            && matches!(
                                face.surface,
                                CadSurface::Plane {
                                    normal: [0., 0., -1.],
                                    ..
                                }
                            )
                    })
            })
            .unwrap();
        let mut entry = ProfileGeometryEntry::new(ProfileGeometryKind::Chain, "Lower rim".into());
        entry.source = ProfileGeometrySource::Cad(vec![reference(wire.id, EntityKind::Loop)]);
        let (contours, heights) = profile_entry_contours(&state, &entry).unwrap();
        assert_eq!(contours.len(), 1);
        assert!(contours[0].closed);
        assert_eq!(contours[0].side, Side::Outside);
        assert_eq!(
            heights,
            Some(CuttingHeights {
                top: 0.,
                bottom: -5.
            })
        );

        // Lower rims retain the exterior material check.
        let mut blocked = state.clone();
        std::sync::Arc::make_mut(&mut blocked.assets[0].topology)
            .faces
            .push(CadFace {
                id: 999,
                body_id: 1,
                surface: CadSurface::Unsupported {
                    description: "exterior overhang".into(),
                },
                loop_ids: vec![],
                bounds: [[19., 0., -2.], [22., 20., -1.]],
            });
        assert!(verify_profile_entry(&blocked, &entry).is_some());
        assert!(profile_entry_contours(&blocked, &entry).unwrap().0[0].closed);
    }

    #[test]
    fn explicit_closed_loop_uses_exact_curves_and_operation_heights_on_spline_faces() {
        let (mut state, face) = plate_with_hole();
        let wire = state.assets[0]
            .topology
            .loops
            .iter()
            .find(|wire| wire.face_id == face.id && wire.is_outer)
            .unwrap()
            .clone();
        // Legacy IGES snapshots can contain a planar boundary on an unrecognized
        // spline face. No surface or wall certificate is inferred from the mesh.
        for face in &mut Arc::make_mut(&mut state.assets[0].topology).faces {
            face.surface = CadSurface::Unsupported {
                description: "spline surface".into(),
            };
        }
        let selected = vec![reference(wire.id, EntityKind::Loop)];
        let mut entry = ProfileGeometryEntry::new(ProfileGeometryKind::Chain, "Closed rim".into());
        entry.source = ProfileGeometrySource::Cad(selected.clone());
        entry.chain_mode = ProfileChainMode::Closed;
        let (contours, interval) = profile_entry_contours(&state, &entry).unwrap();
        assert!(interval.is_none());
        assert_eq!(contours.len(), 1);
        assert!(contours[0].closed);
        assert_eq!(contours[0].chain.area(), 400.);
        let mut binding = resolve_selection(
            &state,
            &selected,
            ExtractionRecipe::ProfileEntries {
                entries: vec![entry],
            },
            1.,
        )
        .unwrap();
        binding.overrides = GeometryOverrides {
            top: Some(1.),
            bottom: Some(-12.),
        };
        let resolved = resolve_binding(&state, &binding).unwrap();
        assert_eq!(
            resolved.heights,
            CuttingHeights {
                top: 1.,
                bottom: -12.
            }
        );
        let topology = Arc::make_mut(&mut state.assets[0].topology);
        topology.mesh = CadMesh::default();
        topology.projection = Some(CadProjectionMesh {
            chord_tolerance: 0.0025,
            mesh: CadMesh::default(),
        });
        assert_eq!(
            resolve_binding(&state, &binding).unwrap(),
            resolved,
            "exact closed paths must not depend on tessellation"
        );

        // A small interior Z excursion can disappear at chord tolerance. The
        // exact spline control hull must still reject that nonplanar boundary.
        let edge = Arc::make_mut(&mut state.assets[0].topology)
            .edges
            .iter_mut()
            .find(|edge| edge.id == wire.edge_uses[0].edge_id)
            .unwrap();
        let CadCurve::Line { start, end } = edge.curve else {
            panic!("line")
        };
        let middle = [(start[0] + end[0]) / 2., (start[1] + end[1]) / 2., 0.005];
        edge.curve = CadCurve::BezierSpans {
            spans: vec![CadBezierSpan {
                poles: vec![start, middle, end],
                weights: vec![1.; 3],
            }],
        };
        assert!(resolve_binding(&state, &binding)
            .unwrap_err()
            .contains("not coplanar"));
    }

    #[test]
    fn cached_topology_validation_rechecks_changed_snapshots() {
        let (mut state, reference) = plate_with_hole();
        View::new(&state, std::slice::from_ref(&reference)).unwrap();
        View::new(&state, std::slice::from_ref(&reference)).unwrap();
        Arc::make_mut(&mut state.assets[0].topology)
            .mesh
            .triangles
            .push([999, 999, 999]);
        assert!(View::new(&state, &[reference]).is_err());
    }

    #[test]
    fn one_concave_wall_does_not_classify_an_exterior_loop_as_a_bore() {
        let (mut state, face) = plate_with_hole();
        let topology = std::sync::Arc::make_mut(&mut state.assets[0].topology);
        let wire = topology
            .loops
            .iter()
            .find(|w| w.face_id == face.id && w.is_outer)
            .unwrap();
        let selected = reference(wire.id, EntityKind::Loop);
        let edge = topology
            .edges
            .iter()
            .find(|e| e.id == wire.edge_uses[0].edge_id)
            .unwrap();
        let wall = edge
            .face_ids
            .iter()
            .copied()
            .find(|id| *id != face.id)
            .unwrap();
        topology
            .faces
            .iter_mut()
            .find(|f| f.id == wall)
            .unwrap()
            .surface = CadSurface::Cylinder {
            origin: [10., -10., 0.],
            axis: [0., 0., 1.],
            radius: 10.,
            inward: true,
        };
        assert_eq!(profile_side(&state, &[selected]).unwrap(), Side::Outside);
    }

    #[test]
    fn face_contours_extract_outer_and_inner_walls_with_independent_sides() {
        let (mut state, face) = plate_with_hole();
        state.instances[0].placement.translation = [100., -50., 40.];
        let b = resolve_selection(&state, &[face], ExtractionRecipe::ProfileContours, 40.).unwrap();
        let input = resolve_binding(&state, &b).unwrap();
        assert_eq!(input.profiles.len(), 2);
        assert_eq!(input.profiles[0].side, Side::Outside);
        assert_eq!(input.profiles[1].side, Side::Inside);
        assert!(input.profiles.iter().all(|p| p.closed));
        assert_eq!(input.profiles[0].chain.area(), 400.);
        assert_eq!(input.profiles[1].chain.area(), 16.);
        assert_eq!(
            input.heights,
            CuttingHeights {
                top: 40.,
                bottom: 35.
            }
        );
        let topology = std::sync::Arc::make_mut(&mut state.assets[0].topology);
        topology.mesh = CadMesh::default();
        assert_eq!(resolve_binding(&state, &b).unwrap(), input);
        state.assets[0].revision += 1;
        assert!(resolve_binding(&state, &b).is_err());
    }

    #[test]
    fn rectangular_floor_resolves_as_inside_profile() {
        let mut fixture = Fixture::new();
        let floor = fixture.face(vec![square(0., 30., 0., 20., 15.)], [0., 0., 1.]);
        fixture.walls(0., 30., 0., 20., 15., 20.);
        fixture.face(
            vec![
                square(-2., 32., -2., 22., 20.),
                square(0., 30., 0., 20., 20.),
            ],
            [0., 0., 1.],
        );
        let state = fixture.state();
        let mut entry =
            ProfileGeometryEntry::new(ProfileGeometryKind::FaceContours, "Face Contours 1".into());
        entry.source = ProfileGeometrySource::Cad(vec![reference(floor, EntityKind::Face)]);
        let (contours, heights) = profile_entry_contours(&state, &entry).unwrap();
        assert_eq!(
            heights,
            Some(CuttingHeights {
                top: 20.,
                bottom: 15.
            })
        );
        assert_eq!(contours.len(), 1);
        assert_eq!(contours[0].side, Side::Inside);
        assert!((contours[0].chain.area() - 600.).abs() < T);
    }

    #[test]
    fn face_contours_accept_pocket_floor_and_preserve_island_sides() {
        let (state, face) = pocket();
        let binding =
            resolve_selection(&state, &[face], ExtractionRecipe::ProfileContours, 0.).unwrap();
        let input = resolve_binding(&state, &binding).unwrap();
        assert_eq!(
            input.heights,
            CuttingHeights {
                top: 0.,
                bottom: -3.
            }
        );
        assert_eq!(input.profiles.len(), 2);
        assert_eq!(input.profiles[0].side, Side::Inside);
        assert_eq!(input.profiles[1].side, Side::Outside);
        assert_eq!(input.profiles[0].chain.area(), 256.);
        assert_eq!(input.profiles[1].chain.area(), 16.);
    }

    #[test]
    fn face_contours_reject_obstructed_or_sloped_pocket_floors() {
        let (mut state, face) = pocket();
        let topology = std::sync::Arc::make_mut(&mut state.assets[0].topology);
        let wall = topology
            .faces
            .iter_mut()
            .find(|f| matches!(f.surface, CadSurface::Plane { normal, .. } if normal[2] == 0.))
            .unwrap();
        wall.surface = CadSurface::Plane {
            origin: [2., 2., -3.],
            normal: [0., 0.6, 0.8],
        };
        assert!(resolve_selection(&state, &[face], ExtractionRecipe::ProfileContours, 0.).is_err());

        let mut fixture = Fixture::new();
        let floor = fixture.face(vec![square(2., 18., 2., 18., -3.)], [0., 0., 1.]);
        fixture.walls(2., 18., 2., 18., -3., 0.);
        fixture.face(
            vec![square(0., 20., 0., 20., 0.), square(2., 18., 2., 18., 0.)],
            [0., 0., 1.],
        );
        fixture.face(vec![square(2., 6., 2., 18., -1.)], [0., 0., -1.]);
        assert!(resolve_selection(
            &fixture.state(),
            &[reference(floor, EntityKind::Face)],
            ExtractionRecipe::ProfileContours,
            0.,
        )
        .unwrap_err()
        .contains("overhang"));
    }
    #[test]
    fn selected_inner_loop_resolves_through_profile_contours_with_inside_side() {
        let (state, face) = plate_with_hole();
        let loop_id = state.assets[0]
            .topology
            .loops
            .iter()
            .find(|wire| wire.face_id == face.id && !wire.is_outer)
            .unwrap()
            .id;
        let selected = reference(loop_id, EntityKind::Loop);

        let mut entry =
            ProfileGeometryEntry::new(ProfileGeometryKind::Chain, "Selected inner loop".into());
        entry.source = ProfileGeometrySource::Cad(vec![selected.clone()]);
        let (entry_contours, heights) = profile_entry_contours(&state, &entry).unwrap();
        assert_eq!(entry_contours.len(), 1);
        assert_eq!(entry_contours[0].side, Side::Inside);
        assert_eq!(entry_contours[0].chain.area(), 16.);
        assert_eq!(
            heights,
            Some(CuttingHeights {
                top: 0.,
                bottom: -5.
            })
        );

        let binding =
            resolve_selection(&state, &[selected], ExtractionRecipe::ProfileContours, 0.).unwrap();
        let direct = resolve_binding(&state, &binding).unwrap();
        assert_eq!(direct.profiles.len(), 1);
        assert_eq!(direct.profiles[0].side, Side::Inside);
        assert_eq!(direct.profiles[0].chain.area(), 16.);
    }

    #[test]
    fn connected_edges_preserve_open_endpoints_and_detect_complete_loops_and_gaps() {
        let (state, face) = plate_with_hole();
        let asset = &state.assets[0];
        let wire = asset
            .topology
            .loops
            .iter()
            .find(|w| w.face_id == face.id && w.is_outer)
            .unwrap();
        let edges: Vec<_> = wire
            .edge_uses
            .iter()
            .map(|u| reference(u.edge_id, EntityKind::Edge))
            .collect();
        let b =
            resolve_selection(&state, &edges[..2], ExtractionRecipe::ProfileContours, 0.).unwrap();
        let input = resolve_binding(&state, &b).unwrap();
        assert_eq!(input.profiles.len(), 1);
        assert!(!input.profiles[0].closed);
        assert_eq!(input.profiles[0].chain.len(), 3);
        assert_eq!(
            input.profiles[0].chain.points().first(),
            Some(&Point::new(0., 0.))
        );
        assert_eq!(
            input.profiles[0].chain.points().last(),
            Some(&Point::new(20., 20.))
        );
        let shuffled = vec![
            edges[2].clone(),
            edges[0].clone(),
            edges[3].clone(),
            edges[1].clone(),
        ];
        let b =
            resolve_selection(&state, &shuffled, ExtractionRecipe::ProfileContours, 0.).unwrap();
        assert!(resolve_binding(&state, &b).unwrap().profiles[0].closed);
        assert!(resolve_selection(
            &state,
            &[edges[0].clone(), edges[2].clone()],
            ExtractionRecipe::ProfileContours,
            0.
        )
        .is_err());
    }

    #[test]
    fn profile_chain_resolves_edges_from_multiple_body_instances() {
        let (mut state, face) = plate_with_hole();
        let mut second = state.instances[0].clone();
        second.id = 2;
        second.placement.translation = [30., 0., 0.];
        state.instances.push(second);
        let wire = state.assets[0]
            .topology
            .loops
            .iter()
            .find(|wire| wire.face_id == face.id && wire.is_outer)
            .unwrap();
        let edges: Vec<_> = wire
            .edge_uses
            .iter()
            .map(|usage| reference(usage.edge_id, EntityKind::Edge))
            .collect();
        for mode in [
            ProfileChainMode::Auto,
            ProfileChainMode::Closed,
            ProfileChainMode::Open,
        ] {
            let selected: Vec<_> = edges
                .iter()
                .cloned()
                .chain(edges.iter().cloned().map(|mut edge| {
                    edge.instance_id = 2;
                    edge
                }))
                .collect();
            let mut entry = ProfileGeometryEntry::new(ProfileGeometryKind::Chain, "Chains".into());
            entry.source = ProfileGeometrySource::Cad(selected.clone());
            entry.chain_mode = mode;
            entry.side_overrides.insert(1, Side::On);
            let recipe = ExtractionRecipe::ProfileEntries {
                entries: vec![entry.clone()],
            };
            let binding = resolve_selection_advisory(&state, &selected, recipe, 0.).unwrap();
            let input = resolve_binding_advisory(&state, &binding).unwrap();
            assert_eq!(input.profiles.len(), 2);
            assert_eq!(input.profiles[1].side, Side::On);
            assert!(input
                .profiles
                .iter()
                .all(|p| p.closed == (mode != ProfileChainMode::Open)));
            assert!(input.profiles[1].chain.points().iter().all(|p| p.x >= 30.));
            assert!(verify_profile_entry(&state, &entry).is_none());
        }
    }

    #[test]
    fn open_chain_uses_local_wall_normals_despite_unsupported_face_trims() {
        let (mut state, face) = plate_with_hole();
        let wire = state.assets[0]
            .topology
            .loops
            .iter()
            .find(|w| w.face_id == face.id && w.is_outer)
            .unwrap()
            .clone();
        let edges: Vec<_> = wire
            .edge_uses
            .iter()
            .map(|u| reference(u.edge_id, EntityKind::Edge))
            .collect();
        let topology = std::sync::Arc::make_mut(&mut state.assets[0].topology);
        topology
            .edges
            .iter_mut()
            .find(|e| e.id == edges[2].id)
            .unwrap()
            .curve = CadCurve::Unsupported {
            description: "unselected spline on the same face".into(),
        };
        for reversed in [false, true] {
            let selected = if reversed {
                vec![edges[1].clone(), edges[0].clone()]
            } else {
                edges[..2].to_vec()
            };
            let binding =
                resolve_selection(&state, &selected, ExtractionRecipe::ProfileContours, 0.)
                    .unwrap();
            let input = resolve_binding(&state, &binding).unwrap();
            let profile = &input.profiles[0];
            assert!(!profile.closed);
            assert_eq!(profile.chain.len(), 3);
            assert_eq!(
                profile.side,
                if reversed {
                    Side::Outside
                } else {
                    Side::Inside
                }
            );
            assert_eq!(
                input.heights,
                CuttingHeights {
                    top: 0.,
                    bottom: -5.
                }
            );
            // Transforming both the edge and its wall must preserve the free side.
            let mut rotated = state.clone();
            rotated.instances[0].placement = RigidTransform {
                rotation: [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]],
                translation: [100., -50., 40.],
            };
            let rotated_binding =
                resolve_selection(&rotated, &selected, ExtractionRecipe::ProfileContours, 40.)
                    .unwrap();
            assert_eq!(
                resolve_binding(&rotated, &rotated_binding)
                    .unwrap()
                    .profiles[0]
                    .side,
                profile.side
            );
        }
        // Unsupported selected geometry still cannot be machined.
        assert!(
            resolve_selection(&state, &edges[2..3], ExtractionRecipe::ProfileContours, 0.)
                .unwrap_err()
                .contains("unsupported exact boundary curve")
        );
        assert!(resolve_selection(
            &state,
            &[reference(wire.id, EntityKind::Loop)],
            ExtractionRecipe::ProfileLoop {
                side: Side::Outside
            },
            0.
        )
        .is_err());
    }

    #[test]
    fn spline_control_hulls_reject_nonplanar_curves_hidden_by_chord_tolerance() {
        let (mut state, face) = plate_with_hole();
        let edge_id = state.assets[0]
            .topology
            .loops
            .iter()
            .find(|w| w.face_id == face.id && w.is_outer)
            .unwrap()
            .edge_uses[0]
            .edge_id;
        let topology = std::sync::Arc::make_mut(&mut state.assets[0].topology);
        let edge = topology.edges.iter_mut().find(|e| e.id == edge_id).unwrap();
        let CadCurve::Line { start, end } = edge.curve else {
            panic!("line")
        };
        let middle = std::array::from_fn(|i| {
            if i == 2 {
                0.005
            } else {
                (start[i] + end[i]) * 0.5
            }
        });
        let spans = vec![CadBezierSpan {
            poles: vec![start, middle, end],
            weights: vec![1.; 3],
        }];
        assert_eq!(
            approximate_bezier_spans(&spans, MACHINING_CHORD_TOLERANCE)
                .unwrap()
                .len(),
            2
        );
        edge.curve = CadCurve::BezierSpans { spans };
        let error = resolve_selection(
            &state,
            &[reference(edge_id, EntityKind::Edge)],
            ExtractionRecipe::ProfileContours,
            0.,
        )
        .unwrap_err();
        assert!(error.contains("one accessible plane"), "{error}");
    }

    #[test]
    fn explicit_open_chain_uses_operation_depth_without_a_vertical_wall() {
        let (mut state, face) = plate_with_hole();
        let wire = state.assets[0]
            .topology
            .loops
            .iter()
            .find(|w| w.face_id == face.id && w.is_outer)
            .unwrap()
            .clone();
        let edges: Vec<_> = wire
            .edge_uses
            .iter()
            .map(|u| reference(u.edge_id, EntityKind::Edge))
            .collect();
        let topology = std::sync::Arc::make_mut(&mut state.assets[0].topology);
        let wall_id = topology
            .edges
            .iter()
            .find(|e| e.id == edges[0].id)
            .unwrap()
            .face_ids
            .iter()
            .copied()
            .find(|id| *id != face.id)
            .unwrap();
        topology
            .faces
            .iter_mut()
            .find(|f| f.id == wall_id)
            .unwrap()
            .surface = CadSurface::Unsupported {
            description: "spline blend".into(),
        };
        let mut entry =
            ProfileGeometryEntry::new(ProfileGeometryKind::Chain, "Open blend rim".into());
        entry.source = ProfileGeometrySource::Cad(edges[..2].to_vec());
        entry.chain_mode = ProfileChainMode::Open;
        let (contours, heights) = profile_entry_contours(&state, &entry).unwrap();
        assert!(
            heights.is_none(),
            "a planar path does not certify wall depth"
        );
        assert!(!contours[0].closed);
        assert_eq!(contours[0].side, Side::Inside);
        let mut backwards = entry.clone();
        backwards.source = ProfileGeometrySource::Cad(vec![edges[1].clone(), edges[0].clone()]);
        assert_eq!(
            profile_entry_contours(&state, &backwards).unwrap().0[0].side,
            Side::Outside
        );
        let mut binding = resolve_selection(
            &state,
            &edges[..2],
            ExtractionRecipe::ProfileEntries {
                entries: vec![entry.clone()],
            },
            1.,
        )
        .unwrap();
        binding.overrides = GeometryOverrides {
            top: Some(1.),
            bottom: Some(-12.),
        };
        let resolved = resolve_binding(&state, &binding).unwrap();
        assert_eq!(
            resolved.heights,
            CuttingHeights {
                top: 1.,
                bottom: -12.
            }
        );
        std::sync::Arc::make_mut(&mut state.assets[0].topology).mesh = CadMesh::default();
        assert_eq!(resolve_binding(&state, &binding).unwrap(), resolved);
        entry.chain_mode = ProfileChainMode::Auto;
        assert!(verify_profile_entry(&state, &entry)
            .unwrap()
            .contains("no adjoining vertical wall"));
        assert!(!profile_entry_contours(&state, &entry).unwrap().0[0].closed);
        entry.chain_mode = ProfileChainMode::Closed;
        assert!(profile_entry_contours(&state, &entry)
            .unwrap_err()
            .contains("Complete the chain"));
        entry.chain_mode = ProfileChainMode::Open;
        state.instances[0].placement.rotation = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]];
        entry.source = ProfileGeometrySource::Cad(vec![edges[1].clone()]);
        assert!(profile_entry_contours(&state, &entry)
            .unwrap_err()
            .contains("one accessible plane"));
    }

    #[test]
    fn interleaved_profile_edges_split_into_independent_open_chains() {
        let (state, face) = plate_with_hole();
        let topology = &state.assets[0].topology;
        let edges = |outer| {
            topology
                .loops
                .iter()
                .find(|wire| wire.face_id == face.id && wire.is_outer == outer)
                .unwrap()
                .edge_uses
                .iter()
                .map(|usage| reference(usage.edge_id, EntityKind::Edge))
                .collect::<Vec<_>>()
        };
        let outer = edges(true);
        let inner = edges(false);
        let groups = [
            vec![outer[1].clone(), outer[0].clone()],
            vec![inner[0].clone()],
            vec![inner[2].clone()],
        ];
        let picks = vec![
            groups[0][0].clone(),
            groups[1][0].clone(),
            groups[2][0].clone(),
            groups[0][1].clone(),
        ];
        let mut entry = ProfileGeometryEntry::new(ProfileGeometryKind::Chain, "Open rims".into());
        entry.chain_mode = ProfileChainMode::Open;
        entry.source = ProfileGeometrySource::Cad(picks);
        entry.reversed = true;
        entry.side_overrides.insert(1, Side::On);

        let split = split_profile_chain_entry(&state, &entry).unwrap().unwrap();
        assert_eq!(split.len(), 3);
        for (entry, refs) in split.iter().zip(groups) {
            assert_eq!(entry.source, ProfileGeometrySource::Cad(refs));
            assert!(entry.reversed);
            assert_eq!(entry.contour_count, 1);
        }
        assert_eq!(split[1].side_overrides.get(&0), Some(&Side::On));
        let (contours, heights) = profile_entry_contours(&state, &entry).unwrap();
        assert!(heights.is_none());
        assert_eq!(contours.len(), 3);
        assert!(contours.iter().all(|contour| !contour.closed));
        assert_eq!(contours[0].chain.len(), 3);
        assert_eq!(contours[1].side, Side::On);

        entry.chain_mode = ProfileChainMode::Closed;
        assert!(profile_entry_contours(&state, &entry).is_err());
    }

    #[test]
    fn trimmed_wall_ends_are_usable_only_for_open_profiles() {
        let mut fixture = Fixture::new();
        let wall_id = fixture.face(
            vec![vec![
                [0., 0., 0.],
                [20., 0., 0.],
                [18., 0., -5.],
                [2., 0., -5.],
            ]],
            [0., -1., 0.],
        );
        let state = fixture.state();
        let selected = [reference(wall_id, EntityKind::Face)];
        let view = View::new(&state, &selected).unwrap();
        let wall = view.face(wall_id).unwrap();
        assert!(view.certify_open_profile_wall(wall, -5., 0.).is_ok());
        assert!(
            view.certify_wall(wall, -5., 0.).is_err(),
            "closed pockets retain their full-height trim guard"
        );
        let mut with_opening = state.clone();
        let topology = std::sync::Arc::make_mut(&mut with_opening.assets[0].topology);
        let wall = topology.faces.iter_mut().find(|f| f.id == wall_id).unwrap();
        wall.loop_ids.push(wall.loop_ids[0]);
        let view = View::new(&with_opening, &selected).unwrap();
        assert!(view
            .certify_open_profile_wall(view.face(wall_id).unwrap(), -5., 0.)
            .is_err());
    }

    #[test]
    fn coplanar_faces_stay_in_the_selected_body_and_use_the_shallowest_wall() {
        let (state, _) = pocket();
        let top = state.assets[0].topology.faces.iter().find(|f| matches!(f.surface, CadSurface::Plane { origin, normal } if origin[2] == 0. && normal[2] > 0.)).unwrap();
        let refs = coplanar_profile_faces(&state, &reference(top.id, EntityKind::Face)).unwrap();
        assert_eq!(refs.len(), 2);
        assert!(refs
            .iter()
            .all(|r| r.kind == EntityKind::Face && r.body_id == 1));
        let mut binding =
            resolve_selection(&state, &refs, ExtractionRecipe::ProfileContours, 0.).unwrap();
        assert_eq!(
            binding.heights,
            CuttingHeights {
                top: 0.,
                bottom: -3.
            }
        );
        let resolved = resolve_binding(&state, &binding).unwrap();
        assert_eq!(resolved.profiles.len(), 3);
        assert_eq!(resolved.heights, binding.heights);
        binding.overrides.bottom = Some(-5.);
        assert!(
            resolve_binding(&state, &binding).is_err(),
            "the deeper exterior wall must not allow cutting below the pocket floor"
        );
    }

    #[test]
    fn rectangle_facing_and_exterior_profile_support_only_certified_sides() {
        let mut f = Fixture::new();
        let top = f.face(vec![square(0., 20., 0., 20., 0.)], [0., 0., 1.]);
        f.walls(0., 20., 0., 20., -5., 0.);
        f.face(vec![square(0., 20., 0., 20., -5.)], [0., 0., -1.]);
        let wire = f.t.faces.iter().find(|f| f.id == top).unwrap().loop_ids[0];
        let state = f.state();
        let b = resolve_selection(
            &state,
            &[reference(top, EntityKind::Face)],
            ExtractionRecipe::FaceRegion,
            2.,
        )
        .unwrap();
        assert_eq!(
            resolve_binding(&state, &b).unwrap().heights,
            CuttingHeights {
                top: 2.,
                bottom: 0.
            }
        );
        let r = reference(wire, EntityKind::Loop);
        let b = resolve_selection(
            &state,
            std::slice::from_ref(&r),
            ExtractionRecipe::ProfileLoop {
                side: Side::Outside,
            },
            0.,
        )
        .unwrap();
        assert_eq!(resolve_binding(&state, &b).unwrap().heights.bottom, -5.);
        assert!(resolve_selection(
            &state,
            &[r],
            ExtractionRecipe::ProfileLoop { side: Side::Inside },
            0.
        )
        .is_err());
    }
    #[test]
    fn exterior_profile_allows_enclosed_curves_but_rejects_unknown_material_outside() {
        let mut fixture = Fixture::new();
        let top = fixture.face(vec![square(0., 20., 0., 20., 0.)], [0., 0., 1.]);
        fixture.walls(0., 20., 0., 20., -5., 0.);
        fixture.face(vec![square(0., 20., 0., 20., -5.)], [0., 0., -1.]);
        let curved = fixture.face(vec![square(5., 7., 5., 7., -1.)], [0., 0., 1.]);
        fixture
            .t
            .faces
            .iter_mut()
            .find(|f| f.id == curved)
            .unwrap()
            .surface = CadSurface::Unsupported {
            description: "enclosed blend".into(),
        };
        let edge = fixture
            .t
            .loops
            .iter()
            .find(|w| w.face_id == curved)
            .unwrap()
            .edge_uses[0]
            .edge_id;
        fixture
            .t
            .edges
            .iter_mut()
            .find(|e| e.id == edge)
            .unwrap()
            .curve = CadCurve::Unsupported {
            description: "enclosed spline".into(),
        };
        let wire = fixture
            .t
            .faces
            .iter()
            .find(|f| f.id == top)
            .unwrap()
            .loop_ids[0];
        let mut state = fixture.state();
        let selected = [reference(wire, EntityKind::Loop)];
        let recipe = ExtractionRecipe::ProfileLoop {
            side: Side::Outside,
        };
        let binding = resolve_selection(&state, &selected, recipe.clone(), 0.).unwrap();
        assert_eq!(
            resolve_binding(&state, &binding).unwrap().heights.bottom,
            -5.
        );
        // Check the enclosure in setup coordinates rather than asset coordinates.
        state.instances[0].placement.translation = [100., -50., 40.];
        let binding = resolve_selection(&state, &selected, recipe.clone(), 40.).unwrap();
        assert_eq!(
            resolve_binding(&state, &binding).unwrap().heights.bottom,
            35.
        );
        std::sync::Arc::make_mut(&mut state.assets[0].topology)
            .faces
            .iter_mut()
            .find(|f| f.id == curved)
            .unwrap()
            .bounds[1][0] = 21.;
        assert!(resolve_selection(&state, &selected, recipe, 40.)
            .unwrap_err()
            .contains("outside the selected silhouette"));
    }

    #[test]
    fn pocket_walls_with_intermediate_trims_and_capped_entries_are_rejected() {
        for defect in ["intermediate trim", "capped entry"] {
            let (mut state, selected) = pocket();
            let topology = std::sync::Arc::make_mut(&mut state.assets[0].topology);
            let floor = topology.faces.iter().find(|f| f.id == selected.id).unwrap();
            let wire = topology
                .loops
                .iter()
                .find(|w| w.id == floor.loop_ids[0])
                .unwrap();
            let wall_id = topology
                .edges
                .iter()
                .find(|e| e.id == wire.edge_uses[0].edge_id)
                .unwrap()
                .face_ids
                .iter()
                .copied()
                .find(|id| *id != selected.id)
                .unwrap();
            let wall = topology.faces.iter().find(|f| f.id == wall_id).unwrap();
            let wall_wire = topology
                .loops
                .iter()
                .find(|w| w.id == wall.loop_ids[0])
                .unwrap();
            let top_edge = wall_wire
                .edge_uses
                .iter()
                .map(|u| u.edge_id)
                .find(|id| {
                    matches!(topology.edges.iter().find(|e| e.id == *id).unwrap().curve,
                    CadCurve::Line { start, end } if start[2] == 0. && end[2] == 0.)
                })
                .unwrap();
            if defect == "intermediate trim" {
                let edge = topology
                    .edges
                    .iter_mut()
                    .find(|e| e.id == top_edge)
                    .unwrap();
                if let CadCurve::Line { end, .. } = &mut edge.curve {
                    end[2] = -1.;
                }
            } else {
                let entry_id = topology
                    .edges
                    .iter()
                    .find(|e| e.id == top_edge)
                    .unwrap()
                    .face_ids
                    .iter()
                    .copied()
                    .find(|id| *id != wall_id)
                    .unwrap();
                topology
                    .loops
                    .iter_mut()
                    .find(|w| {
                        w.face_id == entry_id && w.edge_uses.iter().any(|u| u.edge_id == top_edge)
                    })
                    .unwrap()
                    .is_outer = true;
            }
            let error = resolve_selection(&state, &[selected], ExtractionRecipe::PocketFloor, 0.)
                .unwrap_err();
            assert!(
                error.contains(if defect == "intermediate trim" {
                    "trims"
                } else {
                    "covers the selected void"
                }),
                "{error}"
            );
        }
    }

    #[test]
    fn facing_uses_exact_body_bounds_despite_splines_holes_and_blends_below_the_target() {
        let mut fixture = Fixture::new();
        let target = fixture.face(
            vec![
                vec![[0., 0., 0.], [20., 0., 0.], [18., 20., 0.], [0., 16., 0.]],
                square(5., 7., 5., 7., 0.),
            ],
            [0., 0., 1.],
        );
        fixture.t.edges[0].curve = CadCurve::Unsupported {
            description: "exact spline retained in B-rep".into(),
        };
        let side = fixture.face(vec![square(0., 20., 0., 20., -2.)], [0., 0., -1.]);
        fixture
            .t
            .faces
            .iter_mut()
            .find(|face| face.id == side)
            .unwrap()
            .surface = CadSurface::Unsupported {
            description: "blend below target".into(),
        };
        let state = fixture.state();
        let selected = [reference(target, EntityKind::Face)];
        let binding =
            resolve_selection(&state, &selected, ExtractionRecipe::FaceRegion, 1.).unwrap();
        let input = resolve_binding(&state, &binding).unwrap();
        assert_eq!(
            input.heights,
            CuttingHeights {
                top: 1.,
                bottom: 0.
            }
        );
        assert_eq!(input.regions[0].area(), 400.);
        assert!(
            input.regions[0].holes().is_empty(),
            "facing clears stock above the body, including above holes"
        );
        assert!(
            resolve_selection(&state, &selected, ExtractionRecipe::PocketFloor, 1.).is_err(),
            "pocket certification is unchanged"
        );
        let mut boss = state;
        std::sync::Arc::make_mut(&mut boss.assets[0].topology)
            .faces
            .iter_mut()
            .find(|face| face.id == side)
            .unwrap()
            .bounds[1][2] = 2.;
        assert!(
            resolve_selection(&boss, &selected, ExtractionRecipe::FaceRegion, 3.)
                .unwrap_err()
                .contains("protected material")
        );
    }

    #[test]
    fn circular_edges_alone_do_not_establish_holes() {
        let (state, r) = pocket();
        let mut edge = r;
        edge.kind = EntityKind::Edge;
        edge.id = state.assets[0].topology.edges[0].id;
        assert!(
            resolve_selection(&state, &[edge], ExtractionRecipe::DrillHoles, 0.)
                .unwrap_err()
                .contains("circular edges")
        );
    }
    fn projected_l_prism() -> (GeometryState, EntityRef) {
        let mut fixture = Fixture::new();
        let face = fixture.id();
        fixture.t.bodies[0].bounds = [[0., 0., -5.], [4., 4., 0.]];
        fixture.t.faces.push(CadFace {
            id: face,
            body_id: 1,
            surface: CadSurface::Unsupported {
                description: "rounded exterior".into(),
            },
            loop_ids: vec![],
            bounds: fixture.t.bodies[0].bounds,
        });
        fixture.t.projection = Some(CadProjectionMesh {
            chord_tolerance: 0.0025,
            mesh: CadMesh {
                vertices: vec![
                    [0., 0., 0.],
                    [4., 0., 0.],
                    [4., 1., 0.],
                    [1., 1., 0.],
                    [1., 4., 0.],
                    [0., 4., 0.],
                ],
                triangles: vec![[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5]],
                triangle_faces: vec![face; 4],
            },
        });
        let geometry = GeometryState {
            assets: vec![GeometryAsset {
                id: 1,
                revision: 1,
                name: "L".into(),
                source_format: CadFormat::Step,
                source_path: None,
                source_data: vec![].into(),
                source_units: SourceUnits::Millimetres,
                backend: "test".into(),
                import_options: None,
                import_diagnostics: vec![],
                source_fingerprint: String::new(),
                topology: fixture.t.into(),
            }],
            instances: vec![GeometryInstance {
                id: 1,
                asset_id: 1,
                body_id: 1,
                name: "L".into(),
                placement: RigidTransform::default(),
                visible: true,
            }],
            ..Default::default()
        };
        (
            geometry,
            EntityRef {
                asset_id: 1,
                revision: 1,
                instance_id: 1,
                body_id: 1,
                kind: EntityKind::Silhouette,
                id: 1,
            },
        )
    }

    #[test]
    fn xy_silhouette_preserves_concavities_and_is_independent_of_display_geometry() {
        let (mut geometry, selected) = projected_l_prism();
        let binding = resolve_selection(
            &geometry,
            &[selected],
            ExtractionRecipe::ProfileLoop {
                side: Side::Outside,
            },
            0.,
        )
        .unwrap();
        let before = resolve_binding(&geometry, &binding).unwrap();
        assert_eq!(
            before.heights,
            CuttingHeights {
                top: 0.,
                bottom: -5.
            }
        );
        let region = &before.regions[0];
        assert!(region.contains(Point::new(3., 0.5)));
        assert!(region.contains(Point::new(0.5, 3.)));
        assert!(
            !region.contains(Point::new(2., 2.)),
            "projection must not become a convex hull or bounding box"
        );
        assert!(region.area() > 7. && region.area() < 7.1);
        std::sync::Arc::make_mut(&mut geometry.assets[0].topology).mesh = CadMesh {
            vertices: vec![[100., 100., 100.]],
            ..Default::default()
        };
        assert_eq!(before, resolve_binding(&geometry, &binding).unwrap());
        geometry.instances[0].placement.translation = [10., 20., 30.];
        let moved = resolve_binding(&geometry, &binding).unwrap();
        assert!(moved.regions[0].contains(Point::new(13., 20.5)));
        assert!(!moved.regions[0].contains(Point::new(3., 0.5)));
        assert_eq!(
            moved.heights,
            CuttingHeights {
                top: 30.,
                bottom: 25.
            }
        );
        assert_ne!(before.fingerprint, moved.fingerprint);
        geometry.instances[0].placement.rotation = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]];
        let frame = SetupFrame::from_axes(Some([1., 0., 0.]), Some([0., 0., 1.]), None).unwrap();
        let in_setup = geometry.geometry_in_setup(&frame).unwrap();
        let rotated = resolve_binding(&in_setup, &binding).unwrap();
        assert!(rotated.regions[0].contains(Point::new(13., 30.5)));
        assert!(!rotated.regions[0].contains(Point::new(12., 32.)));
        assert_eq!(
            rotated.heights,
            CuttingHeights {
                top: -20.,
                bottom: -25.
            }
        );
    }

    #[test]
    fn cached_topology_fingerprints_follow_content_and_release_old_snapshots() {
        let (geometry, _) = projected_l_prism();
        let mut topology = Arc::new((*geometry.assets[0].topology).clone());
        let exact = topology_fingerprint(&topology, false).unwrap();
        let projected = topology_fingerprint(&topology, true).unwrap();
        assert_eq!(topology_fingerprint(&topology, false).unwrap(), exact);
        assert_eq!(topology_fingerprint(&topology, true).unwrap(), projected);

        // Detached/reopened data hashes identically, independent of allocation.
        let detached = Arc::new((*topology).clone());
        assert_eq!(topology_fingerprint(&detached, false).unwrap(), exact);
        assert_eq!(topology_fingerprint(&detached, true).unwrap(), projected);
        let weak = Arc::downgrade(&detached);
        drop(detached);
        assert!(
            weak.upgrade().is_none(),
            "fingerprint caching must not retain geometry"
        );

        // A sole owner's make_mut must detach the weak cache key too.
        let original = Arc::downgrade(&topology);
        Arc::make_mut(&mut topology)
            .projection
            .as_mut()
            .unwrap()
            .chord_tolerance *= 2.;
        assert!(original.upgrade().is_none());
        assert_eq!(topology_fingerprint(&topology, false).unwrap(), exact);
        let changed_projection = topology_fingerprint(&topology, true).unwrap();
        assert_ne!(changed_projection, projected);

        Arc::make_mut(&mut topology)
            .mesh
            .vertices
            .push([100., 200., 300.]);
        assert_eq!(topology_fingerprint(&topology, false).unwrap(), exact);
        assert_eq!(
            topology_fingerprint(&topology, true).unwrap(),
            changed_projection
        );
        Arc::make_mut(&mut topology).bodies[0].bounds[0][0] -= 1.;
        assert_ne!(topology_fingerprint(&topology, false).unwrap(), exact);
        assert_ne!(
            topology_fingerprint(&topology, true).unwrap(),
            changed_projection
        );
    }

    #[test]
    fn profile_silhouettes_include_through_openings_and_fill_blind_recesses() {
        let (mut geometry, selected) = projected_l_prism();
        let topology = std::sync::Arc::make_mut(&mut geometry.assets[0].topology);
        let face = topology.faces[0].id;
        let projection = topology.projection.as_mut().unwrap();
        projection.mesh.vertices = vec![
            [0., 0., 0.],
            [4., 0., 0.],
            [4., 4., 0.],
            [0., 4., 0.],
            [1., 1., 0.],
            [3., 1., 0.],
            [3., 3., 0.],
            [1., 3., 0.],
        ];
        projection.mesh.triangles = vec![
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ];
        projection.mesh.triangle_faces = vec![face; 8];
        let binding = resolve_selection(
            &geometry,
            std::slice::from_ref(&selected),
            ExtractionRecipe::ProfileSilhouette,
            0.,
        )
        .unwrap();
        let input = resolve_binding(&geometry, &binding).unwrap();
        assert_eq!(input.profiles.len(), 2);
        assert_eq!(input.profiles[0].side, Side::Outside);
        assert_eq!(input.profiles[1].side, Side::Inside);
        assert!(!input.regions[0].contains(Point::new(2., 2.)));
        // The older envelope-only recipe and hover cache retain their original behavior.
        let cached_outline = silhouette(&geometry, &selected).unwrap();
        assert!(cached_outline.holes().is_empty());
        SILHOUETTES.with(|cache| cache.borrow_mut().clear());
        assert_eq!(
            cached_outline,
            silhouette(&geometry, &selected).unwrap(),
            "filling a cached profile silhouette must match an independent outside-only projection"
        );
        let projection = std::sync::Arc::make_mut(&mut geometry.assets[0].topology)
            .projection
            .as_mut()
            .unwrap();
        projection.mesh.triangles.extend([[4, 5, 6], [4, 6, 7]]);
        projection.mesh.triangle_faces.extend([face; 2]);
        let input = resolve_binding(&geometry, &binding).unwrap();
        assert_eq!(
            input.profiles.len(),
            1,
            "a projected recess floor fills the opening"
        );
        assert!(input.regions[0].contains(Point::new(2., 2.)));
    }

    #[test]
    fn xy_silhouette_rejects_missing_incomplete_stale_and_coarse_projection_data() {
        let (mut geometry, selected) = projected_l_prism();
        let topology = std::sync::Arc::make_mut(&mut geometry.assets[0].topology);
        let saved = topology.projection.take().unwrap();
        assert!(silhouette(&geometry, &selected)
            .unwrap_err()
            .contains("Reimport"));
        std::sync::Arc::make_mut(&mut geometry.assets[0].topology).projection = Some(saved);
        let projection = std::sync::Arc::make_mut(&mut geometry.assets[0].topology)
            .projection
            .as_mut()
            .unwrap();
        projection.chord_tolerance = 0.5;
        assert!(silhouette(&geometry, &selected)
            .unwrap_err()
            .contains("tolerance"));
        let projection = std::sync::Arc::make_mut(&mut geometry.assets[0].topology)
            .projection
            .as_mut()
            .unwrap();
        projection.chord_tolerance = 0.0025;
        projection.mesh.triangles.clear();
        projection.mesh.triangle_faces.clear();
        assert!(silhouette(&geometry, &selected)
            .unwrap_err()
            .contains("Incomplete"));
        geometry.assets[0].revision += 1;
        assert!(silhouette(&geometry, &selected)
            .unwrap_err()
            .contains("stale"));
    }
    mod rotary_tests {
        include!("rotary_tests.rs");
    }
}
