//! Semantic CAD display/picking. All geometry tests use f64 world coordinates;
//! conversion to f32 happens only while preparing the display buffers.
use crate::{Color, MeshVertex, Ray, Scene};
use cam_model::{CadCurve, CadSurface, EntityKind, EntityRef, GeometryState, InstanceId};
use std::collections::{BTreeMap, BTreeSet};

type Point = [f64; 3];
#[path = "comparison.rs"]
mod comparison;
pub use comparison::SurfaceClass;
#[derive(Clone, Debug)]
struct Triangle {
    points: [Point; 3],
    face: EntityRef,
}
#[derive(Clone, Debug)]
struct Path {
    points: Vec<Point>,
    entity: EntityRef,
}
#[derive(Clone, Debug)]
struct Vertex {
    point: Point,
    entity: EntityRef,
}
#[derive(Clone, Debug)]
struct Node {
    bounds: [Point; 2],
    children: Option<(usize, usize)>,
    triangles: Vec<usize>,
}

/// A pick candidate retains semantic identity independent of triangle numbering.
#[derive(Clone, Debug, PartialEq)]
pub struct PickCandidate {
    pub entity: EntityRef,
    /// Forward distance in millimetres from the near-plane ray origin.
    pub depth: f64,
    /// Candidates behind visible solid surfaces need explicit select-through.
    pub occluded: bool,
}

#[derive(Clone, Debug, Default)]
pub struct CadScene {
    bounds: Option<([f32; 3], [f32; 3])>,
    triangles: Vec<Triangle>,
    paths: Vec<Path>,
    vertices: Vec<Vertex>,
    nodes: Vec<Node>,
    face_outer_loops: BTreeMap<EntityRef, EntityRef>,
    face_edges: BTreeMap<EntityRef, BTreeSet<EntityRef>>,
    cylindrical_faces: BTreeSet<EntityRef>,
    bore_rims: BTreeMap<EntityRef, Vec<EntityRef>>,
    planar_loops: BTreeSet<EntityRef>,
    surface_loops: BTreeSet<EntityRef>,
    profile_targets: BTreeMap<EntityRef, EntityRef>,
}

impl CadScene {
    /// Compose component placement without changing the topology identities.
    /// Hidden instances contribute neither pixels nor pick candidates.
    pub fn from_geometry(geometry: &GeometryState, hidden: &BTreeSet<InstanceId>) -> Self {
        Self::from_geometry_in_setup(geometry, hidden, &cam_model::SetupFrame::default())
    }

    /// Display remains in world coordinates; profile candidates are classified
    /// relative to the machining frame, independently of camera orientation.
    pub fn from_geometry_in_setup(
        geometry: &GeometryState,
        hidden: &BTreeSet<InstanceId>,
        frame: &cam_model::SetupFrame,
    ) -> Self {
        let mut scene = Self::default();
        for instance in &geometry.instances {
            if !instance.visible || hidden.contains(&instance.id) {
                continue;
            }
            let Some(asset) = geometry.asset(instance.asset_id) else {
                continue;
            };
            let topology = &asset.topology;
            let entity = |kind, id| EntityRef {
                asset_id: asset.id,
                revision: asset.revision,
                instance_id: instance.id,
                body_id: instance.body_id,
                kind,
                id,
            };
            for face in topology
                .faces
                .iter()
                .filter(|face| face.body_id == instance.body_id)
            {
                let face_ref = entity(EntityKind::Face, face.id);
                if let Some(wire) = topology
                    .loops
                    .iter()
                    .find(|wire| wire.face_id == face.id && wire.is_outer)
                {
                    scene
                        .face_outer_loops
                        .insert(face_ref.clone(), entity(EntityKind::Loop, wire.id));
                }
                if matches!(face.surface, CadSurface::Cylinder { .. }) {
                    scene.cylindrical_faces.insert(face_ref);
                }
            }
            for wire in topology
                .loops
                .iter()
                .filter(|w| w.body_id == instance.body_id)
            {
                let points: Vec<_> = wire
                    .edge_uses
                    .iter()
                    .filter_map(|u| topology.edges.iter().find(|e| e.id == u.edge_id))
                    .flat_map(|e| curve_points(&e.curve))
                    .map(|p| frame.to_setup_vector(instance.placement.transform_point(p)))
                    .collect();
                let planar_face = topology
                    .faces
                    .iter()
                    .find(|f| f.id == wire.face_id)
                    .is_some_and(|f| match f.surface {
                        CadSurface::Plane { normal, .. } => {
                            let n =
                                frame.to_setup_vector(instance.placement.transform_vector(normal));
                            n[0].abs() < 1e-5 && n[1].abs() < 1e-5
                        }
                        _ => false,
                    });
                let planar = planar_face
                    || points
                        .first()
                        .is_some_and(|p| points.iter().all(|q| (p[2] - q[2]).abs() < 1e-5));
                let reference = entity(EntityKind::Loop, wire.id);
                if planar {
                    scene.planar_loops.insert(reference.clone());
                }
                if topology
                    .faces
                    .iter()
                    .find(|f| f.id == wire.face_id)
                    .is_some_and(|f| match f.surface {
                        CadSurface::Plane { normal, .. } => {
                            let n =
                                frame.to_setup_vector(instance.placement.transform_vector(normal));
                            n[0].abs() < 1e-5 && n[1].abs() < 1e-5 && n[2] > 0.
                        }
                        _ => false,
                    })
                {
                    scene.surface_loops.insert(reference.clone());
                }
                // A rounded transition makes the top face boundary an inset.
                // Select the full body envelope explicitly in that case.
                let blended = wire.is_outer
                    && wire.edge_uses.iter().any(|u| {
                        topology
                            .edges
                            .iter()
                            .find(|e| e.id == u.edge_id)
                            .is_some_and(|e| {
                                matches!(e.curve, CadCurve::Unsupported { .. })
                                    || e.face_ids.iter().any(|id| {
                                        if *id == wire.face_id {
                                            return false;
                                        }
                                        topology.faces.iter().find(|f| f.id == *id).is_some_and(
                                            |f| match f.surface {
                                                CadSurface::Unsupported { .. }
                                                | CadSurface::Torus { .. }
                                                | CadSurface::Cone { .. }
                                                | CadSurface::Sphere { .. } => true,
                                                CadSurface::Cylinder {
                                                    axis,
                                                    inward: false,
                                                    ..
                                                } => {
                                                    let axis = frame.to_setup_vector(
                                                        instance.placement.transform_vector(axis),
                                                    );
                                                    axis[0].abs() > 1e-5 || axis[1].abs() > 1e-5
                                                }
                                                _ => false,
                                            },
                                        )
                                    })
                            })
                    });
                if blended && topology.projection.is_some() {
                    scene
                        .profile_targets
                        .insert(reference, entity(EntityKind::Silhouette, instance.body_id));
                }
            }
            for edge in topology.edges.iter().filter(|edge| {
                edge.body_id == instance.body_id && matches!(edge.curve, CadCurve::CircleArc { .. })
            }) {
                let cylinders: Vec<_> = edge
                    .face_ids
                    .iter()
                    .filter_map(|id| {
                        let candidate = entity(EntityKind::Face, *id);
                        scene
                            .cylindrical_faces
                            .contains(&candidate)
                            .then_some(candidate)
                    })
                    .collect();
                if !cylinders.is_empty() {
                    scene
                        .bore_rims
                        .insert(entity(EntityKind::Edge, edge.id), cylinders);
                }
            }
            for (indices, face_id) in topology
                .mesh
                .triangles
                .iter()
                .zip(&topology.mesh.triangle_faces)
            {
                if !topology
                    .faces
                    .iter()
                    .any(|f| f.id == *face_id && f.body_id == instance.body_id)
                {
                    continue;
                }
                let points: Option<Vec<Point>> = indices
                    .iter()
                    .map(|&i| {
                        topology
                            .mesh
                            .vertices
                            .get(i as usize)
                            .map(|p| instance.placement.transform_point(*p))
                    })
                    .collect();
                if let Some(p) = points {
                    scene.triangles.push(Triangle {
                        points: [p[0], p[1], p[2]],
                        face: entity(EntityKind::Face, *face_id),
                    });
                }
            }
            for edge in topology
                .edges
                .iter()
                .filter(|e| e.body_id == instance.body_id)
            {
                for face_id in &edge.face_ids {
                    scene
                        .face_edges
                        .entry(entity(EntityKind::Face, *face_id))
                        .or_default()
                        .insert(entity(EntityKind::Edge, edge.id));
                }
                let points: Vec<_> = curve_points(&edge.curve)
                    .into_iter()
                    .map(|p| instance.placement.transform_point(p))
                    .collect();
                if points.len() >= 2 {
                    scene.paths.push(Path {
                        points,
                        entity: entity(EntityKind::Edge, edge.id),
                    });
                }
            }
            for wire in topology
                .loops
                .iter()
                .filter(|e| e.body_id == instance.body_id)
            {
                // Each edge is kept as a separate strip, avoiding artificial segments
                // if a malformed loop or unsupported curve leaves a gap.
                for edge_use in &wire.edge_uses {
                    let Some(edge) = topology.edges.iter().find(|e| e.id == edge_use.edge_id)
                    else {
                        continue;
                    };
                    let points = curve_points(&edge.curve)
                        .into_iter()
                        .map(|p| instance.placement.transform_point(p))
                        .collect::<Vec<_>>();
                    if points.len() >= 2 {
                        scene.paths.push(Path {
                            points,
                            entity: entity(EntityKind::Loop, wire.id),
                        });
                    }
                }
            }
            for vertex in topology
                .vertices
                .iter()
                .filter(|e| e.body_id == instance.body_id)
            {
                scene.vertices.push(Vertex {
                    point: instance.placement.transform_point(vertex.point),
                    entity: entity(EntityKind::Vertex, vertex.id),
                });
            }
        }
        if !scene.triangles.is_empty() {
            scene.build_node((0..scene.triangles.len()).collect());
        }
        scene.bounds = scene.compute_bounds();
        scene
    }

    fn build_node(&mut self, mut ids: Vec<usize>) -> usize {
        let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        for &id in &ids {
            for p in self.triangles[id].points {
                include(&mut bounds, p);
            }
        }
        let id = self.nodes.len();
        self.nodes.push(Node {
            bounds,
            children: None,
            triangles: vec![],
        });
        if ids.len() <= 12 {
            self.nodes[id].triangles = ids;
        } else {
            let axis = (0..3)
                .max_by(|&a, &b| {
                    (bounds[1][a] - bounds[0][a]).total_cmp(&(bounds[1][b] - bounds[0][b]))
                })
                .unwrap();
            // The BVH needs a median partition, not a full ordering at every level.
            let middle = ids.len() / 2;
            ids.select_nth_unstable_by(middle, |&a, &b| {
                let centre = |i: usize| {
                    self.triangles[i]
                        .points
                        .iter()
                        .map(|p| p[axis])
                        .sum::<f64>()
                };
                centre(a).total_cmp(&centre(b))
            });
            let right = ids.split_off(ids.len() / 2);
            let l = self.build_node(ids);
            let r = self.build_node(right);
            self.nodes[id].children = Some((l, r));
        }
        id
    }

    pub fn bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        self.bounds
    }

    fn compute_bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        for t in &self.triangles {
            for p in t.points {
                include(&mut bounds, p);
            }
        }
        for p in &self.paths {
            for &point in &p.points {
                include(&mut bounds, point);
            }
        }
        if !bounds[0][0].is_finite() {
            return None;
        }
        Some((bounds[0].map(|v| v as f32), bounds[1].map(|v| v as f32)))
    }

    fn ray_hits(&self, ray: Ray, id: usize, hits: &mut Vec<(usize, f64)>) {
        let node = &self.nodes[id];
        if !ray_box(ray, node.bounds) {
            return;
        }
        if let Some((l, r)) = node.children {
            self.ray_hits(ray, l, hits);
            self.ray_hits(ray, r, hits);
        } else {
            for &i in &node.triangles {
                if let Some(t) = ray_triangle(ray, self.triangles[i].points) {
                    hits.push((i, t));
                }
            }
        }
    }

    fn nearest_depth_at(&self, ray: Ray, point: Point, depth: f64) -> f64 {
        // The aperture can catch a feature beside the cursor ray. Test visibility
        // on its own parallel ray: the solid under the cursor is not its occluder.
        let ray = Ray {
            origin: sub(point, scale(ray.direction, depth)),
            direction: ray.direction,
        };
        let mut hits = vec![];
        if !self.nodes.is_empty() {
            self.ray_hits(ray, 0, &mut hits);
        }
        hits.into_iter()
            .map(|(_, depth)| depth)
            .min_by(f64::total_cmp)
            .unwrap_or(f64::INFINITY)
    }

    /// Whether a world feature is in front of the model on its own camera ray.
    pub fn point_visible(&self, ray: Ray, point: Point) -> bool {
        let depth = dot(sub(point, ray.origin), ray.direction);
        depth >= 0. && depth <= self.nearest_depth_at(ray, point, depth) + 1e-6
    }

    /// Candidates ordered front to back, deduplicated by semantic entity. The
    /// closest solid remains the occluder even under edge/loop/vertex filters.
    /// `aperture` is a screen-derived world distance for narrow geometry.
    pub fn pick_candidates(
        &self,
        ray: Ray,
        filter: EntityKind,
        aperture: f64,
        select_through: bool,
    ) -> Vec<PickCandidate> {
        if !ray
            .origin
            .iter()
            .chain(&ray.direction)
            .all(|v| v.is_finite())
            || aperture < 0.0
            || !aperture.is_finite()
        {
            return vec![];
        }
        let mut hits = vec![];
        if !self.nodes.is_empty() {
            self.ray_hits(ray, 0, &mut hits);
        }
        let nearest = hits
            .iter()
            .map(|(_, d)| *d)
            .min_by(f64::total_cmp)
            .unwrap_or(f64::INFINITY);
        // Only a small depth epsilon is allowed: lateral catch aperture must not
        // expose back edges through a thin solid.
        let epsilon = (self
            .bounds()
            .map(|(lo, hi)| {
                (0..3)
                    .map(|i| (hi[i] - lo[i]).abs() as f64)
                    .fold(0., f64::max)
            })
            .unwrap_or(1.)
            * 1e-7)
            .max(1e-6);
        let mut candidates = vec![];
        let mut push = |entity: EntityRef, depth: f64, nearest: f64| {
            let occluded = depth > nearest + epsilon;
            if depth >= 0. && (select_through || !occluded) {
                candidates.push(PickCandidate {
                    entity,
                    depth,
                    occluded,
                });
            }
        };
        match filter {
            EntityKind::Face | EntityKind::Body | EntityKind::Silhouette => {
                for (i, depth) in hits {
                    let mut entity = self.triangles[i].face.clone();
                    if matches!(filter, EntityKind::Body | EntityKind::Silhouette) {
                        entity.kind = filter;
                        entity.id = entity.body_id;
                    }
                    push(entity, depth, nearest);
                }
            }
            EntityKind::Edge | EntityKind::Loop => {
                for path in self.paths.iter().filter(|p| p.entity.kind == filter) {
                    for segment in path.points.windows(2) {
                        let (distance, depth, point) = ray_segment(ray, segment[0], segment[1]);
                        if distance <= aperture {
                            push(
                                path.entity.clone(),
                                depth,
                                self.nearest_depth_at(ray, point, depth),
                            );
                        }
                    }
                }
            }
            EntityKind::Vertex => {
                for vertex in &self.vertices {
                    let delta = sub(vertex.point, ray.origin);
                    let depth = dot(delta, ray.direction);
                    let distance = length(sub(delta, scale(ray.direction, depth)));
                    if distance <= aperture {
                        push(
                            vertex.entity.clone(),
                            depth,
                            self.nearest_depth_at(ray, vertex.point, depth),
                        );
                    }
                }
            }
        }
        candidates.sort_by(|a, b| {
            a.depth
                .total_cmp(&b.depth)
                .then_with(|| a.entity.cmp(&b.entity))
        });
        let mut seen = BTreeSet::new();
        candidates.retain(|c| seen.insert(c.entity.clone()));
        candidates
    }

    /// Pick across several semantic feature kinds, preserving their screen-depth
    /// ordering so the caller can offer overlapping features as alternatives.
    pub fn pick_any_candidates(
        &self,
        ray: Ray,
        filters: &[EntityKind],
        aperture: f64,
        select_through: bool,
    ) -> Vec<PickCandidate> {
        let mut candidates = Vec::new();
        for filter in filters {
            candidates.extend(self.pick_candidates(ray, *filter, aperture, select_through));
        }
        candidates.sort_by(|a, b| {
            a.depth
                .total_cmp(&b.depth)
                .then_with(|| {
                    filters
                        .iter()
                        .position(|kind| *kind == a.entity.kind)
                        .cmp(&filters.iter().position(|kind| *kind == b.entity.kind))
                })
                .then_with(|| a.entity.cmp(&b.entity))
        });
        let mut seen = BTreeSet::new();
        candidates.retain(|candidate| seen.insert(candidate.entity.clone()));
        candidates
    }

    /// Profile geometry uses an outer loop. A face-interior click identifies
    /// that loop when the perimeter is outside the screen catch aperture.
    pub fn pick_profile_candidates(
        &self,
        ray: Ray,
        aperture: f64,
        select_through: bool,
    ) -> Vec<PickCandidate> {
        let mut loops = self.pick_candidates(ray, EntityKind::Loop, aperture, select_through);
        loops.retain(|c| self.planar_loops.contains(&c.entity));
        for candidate in &mut loops {
            if let Some(target) = self.profile_targets.get(&candidate.entity) {
                candidate.entity = target.clone();
            }
        }
        if !loops.is_empty() {
            deduplicate_candidates(&mut loops);
            return loops;
        }
        let mut candidates: Vec<_> = self
            .pick_candidates(ray, EntityKind::Face, aperture, select_through)
            .into_iter()
            .filter_map(|mut candidate| {
                let wire = self.face_outer_loops.get(&candidate.entity)?;
                candidate.entity = if let Some(target) = self.profile_targets.get(wire) {
                    target.clone()
                } else if self.planar_loops.contains(wire) {
                    wire.clone()
                } else {
                    // Vertical wall/fillet loops include seams or span several
                    // elevations. They cannot be used as a planar profile.
                    let mut body = candidate.entity;
                    body.kind = EntityKind::Silhouette;
                    body.id = body.body_id;
                    body
                };
                Some(candidate)
            })
            .collect();
        deduplicate_candidates(&mut candidates);
        candidates
    }

    /// Pocket boundaries may use either side of a planar rim, independently of
    /// the supporting face normal. Keep the actual loop beside rounded walls.
    pub fn pick_pocket_loop_candidates(
        &self,
        ray: Ray,
        aperture: f64,
        select_through: bool,
    ) -> Vec<PickCandidate> {
        let mut loops = self.pick_candidates(ray, EntityKind::Loop, aperture, select_through);
        loops.retain(|c| self.planar_loops.contains(&c.entity));
        if loops.is_empty() {
            loops = self
                .pick_candidates(ray, EntityKind::Face, aperture, select_through)
                .into_iter()
                .filter_map(|mut candidate| {
                    let wire = self.face_outer_loops.get(&candidate.entity)?;
                    if !self.planar_loops.contains(wire) {
                        return None;
                    }
                    candidate.entity = wire.clone();
                    Some(candidate)
                })
                .collect();
        }
        deduplicate_candidates(&mut loops);
        loops
    }

    /// Chamfer/engrave/carve select the actual planar rim. Rounded transitions
    /// do not redirect these surface operations to a whole-body silhouette.
    pub fn pick_surface_loop_candidates(
        &self,
        ray: Ray,
        aperture: f64,
        select_through: bool,
    ) -> Vec<PickCandidate> {
        let mut loops = self.pick_candidates(ray, EntityKind::Loop, aperture, select_through);
        loops.retain(|c| self.surface_loops.contains(&c.entity));
        if loops.is_empty() {
            loops = self
                .pick_candidates(ray, EntityKind::Face, aperture, select_through)
                .into_iter()
                .filter_map(|mut c| {
                    let wire = self.face_outer_loops.get(&c.entity)?;
                    if !self.surface_loops.contains(wire) {
                        return None;
                    }
                    c.entity = wire.clone();
                    Some(c)
                })
                .collect();
        }
        deduplicate_candidates(&mut loops);
        loops
    }

    /// Ctrl-click selects the envelope of the visible body instance, regardless
    /// of the active face/edge/loop filter. Catch narrow features near their edges.
    pub fn pick_silhouette_candidates(
        &self,
        ray: Ray,
        aperture: f64,
        select_through: bool,
    ) -> Vec<PickCandidate> {
        let mut candidates =
            self.pick_candidates(ray, EntityKind::Silhouette, aperture, select_through);
        if candidates.is_empty() {
            candidates = self.pick_candidates(ray, EntityKind::Edge, aperture, select_through);
            for candidate in &mut candidates {
                candidate.entity.kind = EntityKind::Silhouette;
                candidate.entity.id = candidate.entity.body_id;
            }
            deduplicate_candidates(&mut candidates);
        }
        candidates
    }

    /// Drill geometry uses cylindrical faces, including when a visible circular
    /// rim is easier to click than the bore wall. Planar entry faces are not holes.
    pub fn pick_drill_candidates(
        &self,
        ray: Ray,
        aperture: f64,
        select_through: bool,
    ) -> Vec<PickCandidate> {
        let mut candidates: Vec<_> = self
            .pick_candidates(ray, EntityKind::Face, aperture, select_through)
            .into_iter()
            .filter(|candidate| self.cylindrical_faces.contains(&candidate.entity))
            .collect();
        for rim in self.pick_candidates(ray, EntityKind::Edge, aperture, select_through) {
            if let Some(faces) = self.bore_rims.get(&rim.entity) {
                candidates.extend(faces.iter().map(|face| PickCandidate {
                    entity: face.clone(),
                    depth: rim.depth,
                    occluded: rim.occluded,
                }));
            }
        }
        deduplicate_candidates(&mut candidates);
        candidates
    }

    /// Solid triangles carry their face/instance colour so complete trimmed
    /// faces highlight, including holes, independently of display triangulation.
    pub fn mesh(
        &self,
        selected: &[EntityRef],
        hover: Option<&EntityRef>,
    ) -> (Vec<MeshVertex>, Vec<u32>) {
        let mut vertices = vec![];
        let mut indices = vec![];
        for t in &self.triangles {
            let normal = cross(sub(t.points[1], t.points[0]), sub(t.points[2], t.points[0]));
            let len = length(normal);
            if len <= 1e-12 {
                continue;
            }
            let normal = scale(normal, 1. / len).map(|v| v as f32);
            let color = if selected.iter().any(|r| matches_face(r, &t.face)) {
                [0.20, 0.60, 0.98, 1.]
            } else if hover.is_some_and(|r| matches_face(r, &t.face)) {
                [0.95, 0.72, 0.25, 1.]
            } else {
                [0.55, 0.58, 0.63, 1.]
            };
            for p in t.points {
                indices.push(vertices.len() as u32);
                vertices.push(MeshVertex {
                    position: p.map(|v| v as f32),
                    normal,
                    color,
                });
            }
        }
        (vertices, indices)
    }

    pub fn edges(&self, selected: &[EntityRef], hover: Option<&EntityRef>) -> Scene {
        let mut scene = Scene::new();
        for p in &self.paths {
            let matches = |r: &EntityRef| {
                *r == p.entity
                    || matches_body(r, &p.entity)
                    || self
                        .face_edges
                        .get(r)
                        .is_some_and(|edges| edges.contains(&p.entity))
            };
            let selected_path = selected.iter().any(matches);
            let hovered = hover.is_some_and(matches);
            // Loops draw only as semantic highlights, avoiding duplicate ordinary edges.
            if p.entity.kind == EntityKind::Loop && !selected_path && !hovered {
                continue;
            }
            let color: Color = if selected_path {
                [0.2, 0.6, 0.98, 1.]
            } else if hovered {
                [0.95, 0.72, 0.25, 1.]
            } else {
                [0.20, 0.22, 0.25, 1.]
            };
            let points = p.points.iter().map(|p| p.map(|v| v as f32)).collect();
            if selected_path || hovered {
                scene.add_highlight(points, color);
            } else {
                scene.add_strip(points, color);
            }
        }
        for v in &self.vertices {
            let picked = selected.contains(&v.entity);
            let hovered = hover == Some(&v.entity);
            if picked || hovered {
                let size = self
                    .bounds()
                    .map(|(a, b)| (0..3).map(|i| (b[i] - a[i]).abs()).fold(0., f32::max) * 0.006)
                    .unwrap_or(0.3)
                    .max(0.03);
                let color = if picked {
                    [0.2, 0.6, 0.98, 1.]
                } else {
                    [0.95, 0.72, 0.25, 1.]
                };
                for axis in 0..3 {
                    let mut a = v.point.map(|v| v as f32);
                    let mut b = a;
                    a[axis] -= size;
                    b[axis] += size;
                    scene.add_highlight(vec![a, b], color);
                }
            }
        }
        scene
    }
}

fn deduplicate_candidates(candidates: &mut Vec<PickCandidate>) {
    candidates.sort_by(|a, b| {
        a.depth
            .total_cmp(&b.depth)
            .then_with(|| a.entity.cmp(&b.entity))
    });
    let mut seen = BTreeSet::new();
    candidates.retain(|candidate| seen.insert(candidate.entity.clone()));
}

fn matches_body(a: &EntityRef, b: &EntityRef) -> bool {
    a.kind == EntityKind::Body
        && a.asset_id == b.asset_id
        && a.revision == b.revision
        && a.instance_id == b.instance_id
        && a.body_id == b.body_id
}
fn matches_face(a: &EntityRef, b: &EntityRef) -> bool {
    a == b || matches_body(a, b)
}
fn include(bounds: &mut [Point; 2], p: Point) {
    for (i, value) in p.into_iter().enumerate() {
        bounds[0][i] = bounds[0][i].min(value);
        bounds[1][i] = bounds[1][i].max(value);
    }
}
fn sub(a: Point, b: Point) -> Point {
    std::array::from_fn(|i| a[i] - b[i])
}
fn scale(a: Point, s: f64) -> Point {
    a.map(|v| v * s)
}
fn dot(a: Point, b: Point) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
fn length(a: Point) -> f64 {
    dot(a, a).sqrt()
}
fn cross(a: Point, b: Point) -> Point {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn ray_box(ray: Ray, bounds: [Point; 2]) -> bool {
    let mut near: f64 = 0.;
    let mut far = f64::INFINITY;
    for (i, _) in ray.direction.iter().enumerate() {
        if ray.direction[i].abs() < 1e-14 {
            if ray.origin[i] < bounds[0][i] || ray.origin[i] > bounds[1][i] {
                return false;
            }
        } else {
            let a = (bounds[0][i] - ray.origin[i]) / ray.direction[i];
            let b = (bounds[1][i] - ray.origin[i]) / ray.direction[i];
            near = near.max(a.min(b));
            far = far.min(a.max(b));
            if near > far {
                return false;
            }
        }
    }
    true
}
fn ray_triangle(ray: Ray, p: [Point; 3]) -> Option<f64> {
    let e1 = sub(p[1], p[0]);
    let e2 = sub(p[2], p[0]);
    let h = cross(ray.direction, e2);
    let det = dot(e1, h);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv = 1. / det;
    let s = sub(ray.origin, p[0]);
    let u = dot(s, h) * inv;
    if !(-1e-9..=1. + 1e-9).contains(&u) {
        return None;
    }
    let q = cross(s, e1);
    let v = dot(ray.direction, q) * inv;
    if v < -1e-9 || u + v > 1. + 1e-9 {
        return None;
    }
    let t = dot(e2, q) * inv;
    (t >= 0.).then_some(t)
}
fn ray_segment(ray: Ray, a: Point, b: Point) -> (f64, f64, Point) {
    let edge = sub(b, a);
    let w = sub(a, ray.origin);
    let projected = sub(edge, scale(ray.direction, dot(edge, ray.direction)));
    let wp = sub(w, scale(ray.direction, dot(w, ray.direction)));
    let denominator = dot(projected, projected);
    let fraction = if denominator > 1e-20 {
        (-dot(wp, projected) / denominator).clamp(0., 1.)
    } else {
        0.
    };
    let delta = std::array::from_fn(|i| w[i] + fraction * edge[i]);
    let depth = dot(delta, ray.direction);
    (
        length(sub(delta, scale(ray.direction, depth))),
        depth,
        std::array::from_fn(|i| a[i] + fraction * edge[i]),
    )
}
fn curve_points(curve: &CadCurve) -> Vec<Point> {
    match curve {
        CadCurve::Line { start, end } => vec![*start, *end],
        CadCurve::CircleArc {
            center,
            normal,
            x_axis,
            radius,
            start_angle,
            end_angle,
        } => {
            let y = cross(*normal, *x_axis);
            let span = end_angle - start_angle;
            // Display sampling only; machining resolvers use the exact curve.
            let steps = ((span.abs() * radius.abs() / 0.05).sqrt().ceil() as usize).clamp(8, 2048);
            (0..=steps)
                .map(|i| {
                    let angle = start_angle + span * i as f64 / steps as f64;
                    std::array::from_fn(|j| {
                        center[j] + radius * (x_axis[j] * angle.cos() + y[j] * angle.sin())
                    })
                })
                .collect()
        }
        CadCurve::BezierSpans { spans } => {
            cam_model::approximate_bezier_spans(spans, 0.05).unwrap_or_default()
        }
        CadCurve::Unsupported { .. } => vec![],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{orientation, OrbitCamera};
    use cam_model::{
        CadBody, CadEdge, CadFace, CadFormat, CadLoop, CadSurface, CadVertex, EdgeUse,
        GeometryAsset, GeometryInstance, RigidTransform, SourceUnits, TopologySnapshot,
    };

    fn geometry() -> GeometryState {
        let mut topology = TopologySnapshot::default();
        topology.bodies.push(CadBody {
            id: 1,
            name: "part".into(),
            valid_solid: true,
            bounds: [[-2., -2., 0.], [2., 2., 2.]],
        });
        for (face, z) in [(10, 2.), (20, 0.)] {
            topology.faces.push(CadFace {
                id: face,
                body_id: 1,
                surface: CadSurface::Plane {
                    origin: [0., 0., z],
                    normal: [0., 0., 1.],
                },
                loop_ids: vec![],
                bounds: [[-2., -2., z], [2., 2., z]],
            });
            let base = topology.mesh.vertices.len() as u32;
            topology
                .mesh
                .vertices
                .extend([[-2., -2., z], [2., -2., z], [0., 2., z]]);
            topology.mesh.triangles.push([base, base + 1, base + 2]);
            topology.mesh.triangle_faces.push(face);
        }
        topology.edges.push(CadEdge {
            id: 30,
            body_id: 1,
            vertices: [40, 41],
            face_ids: vec![20],
            curve: CadCurve::Line {
                start: [-0.1, 0., 0.],
                end: [0.1, 0., 0.],
            },
        });
        topology.vertices.push(CadVertex {
            id: 40,
            body_id: 1,
            point: [0., 0., 0.],
        });
        topology.loops.push(CadLoop {
            id: 50,
            body_id: 1,
            face_id: 20,
            edge_uses: vec![EdgeUse {
                edge_id: 30,
                reversed: false,
            }],
            is_outer: true,
        });
        GeometryState {
            assets: vec![GeometryAsset {
                import_options: None,
                import_diagnostics: vec![],
                source_fingerprint: String::new(),
                id: 1,
                revision: 1,
                name: "part".into(),
                source_format: CadFormat::Step,
                source_path: None,
                source_data: vec![].into(),
                source_units: SourceUnits::Millimetres,
                backend: "fixture".into(),
                topology: topology.into(),
            }],
            instances: vec![GeometryInstance {
                id: 1,
                asset_id: 1,
                body_id: 1,
                name: "first".into(),
                placement: RigidTransform::default(),
                visible: true,
            }],
            ..Default::default()
        }
    }
    fn down() -> Ray {
        Ray {
            origin: [0., 0., 5.],
            direction: [0., 0., -1.],
        }
    }

    #[test]
    fn nearest_visible_face_and_semantic_body() {
        let scene = CadScene::from_geometry(&geometry(), &BTreeSet::new());
        let hits = scene.pick_candidates(down(), EntityKind::Face, 0.1, false);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].entity.id, 10);
        let hits = scene.pick_candidates(down(), EntityKind::Body, 0.1, false);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].entity.id, 1);
        let hits = scene.pick_candidates(down(), EntityKind::Face, 0.1, true);
        assert_eq!(hits.len(), 2);
        assert!(hits[1].occluded);
    }
    #[test]
    fn mixed_pick_candidates_expose_each_supported_semantic_kind() {
        let scene = CadScene::from_geometry(&geometry(), &BTreeSet::new());
        let candidates = scene.pick_any_candidates(
            down(),
            &[
                EntityKind::Face,
                EntityKind::Edge,
                EntityKind::Loop,
                EntityKind::Vertex,
                EntityKind::Body,
            ],
            0.1,
            true,
        );
        let kinds: BTreeSet<_> = candidates.iter().map(|pick| pick.entity.kind).collect();
        assert!(
            [
                EntityKind::Face,
                EntityKind::Edge,
                EntityKind::Loop,
                EntityKind::Vertex,
                EntityKind::Body,
            ]
            .into_iter()
            .all(|kind| kinds.contains(&kind)),
            "available kinds: {kinds:?}"
        );
        assert!(candidates
            .windows(2)
            .all(|pair| pair[0].depth <= pair[1].depth));
    }
    #[test]
    fn profile_face_interior_selects_its_outer_loop_with_instance_identity() {
        let mut geometry = geometry();
        let topology = std::sync::Arc::make_mut(&mut geometry.assets[0].topology);
        topology
            .faces
            .iter_mut()
            .find(|face| face.id == 10)
            .unwrap()
            .loop_ids
            .push(60);
        topology.loops.push(CadLoop {
            id: 60,
            body_id: 1,
            face_id: 10,
            edge_uses: vec![],
            is_outer: true,
        });
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        assert!(scene
            .pick_candidates(down(), EntityKind::Loop, 0.1, false)
            .is_empty());
        let hits = scene.pick_profile_candidates(down(), 0.1, false);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].entity.kind, EntityKind::Loop);
        assert_eq!(hits[0].entity.id, 60);
        assert_eq!(hits[0].entity.instance_id, 1);
        assert_eq!(hits[0].entity.revision, 1);
        assert!(!hits[0].occluded);
        let mut scene = scene;
        let rim = hits[0].entity.clone();
        let mut silhouette = rim.clone();
        silhouette.kind = EntityKind::Silhouette;
        silhouette.id = silhouette.body_id;
        scene.profile_targets.insert(rim.clone(), silhouette);
        let surface_hits = scene.pick_surface_loop_candidates(down(), 0.1, false);
        assert_eq!(surface_hits.len(), 1);
        assert_eq!(
            surface_hits[0].entity, rim,
            "Surface operations retain the actual rim even beside a blend"
        );
        let hidden = CadScene::from_geometry(&geometry, &BTreeSet::from([1]));
        assert!(hidden
            .pick_surface_loop_candidates(down(), 0.1, false)
            .is_empty());
        assert!(hidden
            .pick_profile_candidates(down(), 0.1, false)
            .is_empty());
    }
    #[test]
    fn pocket_can_pick_a_downward_facing_planar_boundary() {
        let mut geometry = geometry();
        let topology = std::sync::Arc::make_mut(&mut geometry.assets[0].topology);
        let face = topology
            .faces
            .iter_mut()
            .find(|face| face.id == 10)
            .unwrap();
        face.surface = CadSurface::Plane {
            origin: [0., 0., 0.],
            normal: [0., 0., -1.],
        };
        face.loop_ids.push(60);
        topology.loops.push(CadLoop {
            id: 60,
            body_id: 1,
            face_id: 10,
            edge_uses: vec![],
            is_outer: true,
        });
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        assert!(scene
            .pick_surface_loop_candidates(down(), 0.1, false)
            .is_empty());
        let hits = scene.pick_pocket_loop_candidates(down(), 0.1, false);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].entity.kind, EntityKind::Loop);
        assert_eq!(hits[0].entity.id, 60);
    }

    #[test]
    fn drill_picks_cylinders_from_visible_rims_and_excludes_planar_entry_faces() {
        let mut geometry = geometry();
        let topology = std::sync::Arc::make_mut(&mut geometry.assets[0].topology);
        topology.faces.push(CadFace {
            id: 70,
            body_id: 1,
            surface: CadSurface::Cylinder {
                origin: [0., 0., 0.],
                axis: [0., 0., 1.],
                radius: 0.5,
                inward: true,
            },
            loop_ids: vec![],
            bounds: [[-0.5, -0.5, 0.], [0.5, 0.5, 2.]],
        });
        topology.vertices.push(CadVertex {
            id: 90,
            body_id: 1,
            point: [0.5, 0., 2.],
        });
        topology.edges.push(CadEdge {
            id: 80,
            body_id: 1,
            vertices: [90, 90],
            face_ids: vec![10, 70],
            curve: CadCurve::CircleArc {
                center: [0., 0., 2.],
                normal: [0., 0., 1.],
                x_axis: [1., 0., 0.],
                radius: 0.5,
                start_angle: 0.,
                end_angle: std::f64::consts::TAU,
            },
        });
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        assert!(
            scene.pick_drill_candidates(down(), 0.1, false).is_empty(),
            "planar face interiors are not drill sources"
        );
        let ray = Ray {
            origin: [0.55, 0., 5.],
            direction: [0., 0., -1.],
        };
        let hits = scene.pick_drill_candidates(ray, 0.1, false);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].entity.kind, EntityKind::Face);
        assert_eq!(hits[0].entity.id, 70);
        assert!(!hits[0].occluded);
        // An occluded rim cannot turn its hidden cylindrical face into a visible pick.
        let topology = std::sync::Arc::make_mut(&mut geometry.assets[0].topology);
        let edge = topology
            .edges
            .iter_mut()
            .find(|edge| edge.id == 80)
            .unwrap();
        if let CadCurve::CircleArc { center, .. } = &mut edge.curve {
            center[2] = 0.;
        }
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        assert!(scene.pick_drill_candidates(ray, 0.1, false).is_empty());
        let hits = scene.pick_drill_candidates(ray, 0.1, true);
        assert_eq!(hits.len(), 1);
        assert!(hits[0].occluded);
        // A wall's own mesh also remains selectable without needing a rim hit.
        let topology = std::sync::Arc::make_mut(&mut geometry.assets[0].topology);
        topology.mesh.triangle_faces[0] = 70;
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        assert_eq!(
            scene.pick_drill_candidates(down(), 0.1, false)[0].entity.id,
            70
        );
    }
    #[test]
    fn filters_do_not_expose_edges_loops_or_vertices_through_solid() {
        let scene = CadScene::from_geometry(&geometry(), &BTreeSet::new());
        for filter in [EntityKind::Edge, EntityKind::Loop, EntityKind::Vertex] {
            assert!(scene.pick_candidates(down(), filter, 0.2, false).is_empty());
            let through = scene.pick_candidates(down(), filter, 0.2, true);
            assert_eq!(through.len(), 1);
            assert!(through[0].occluded);
        }
    }
    #[test]
    fn aperture_selects_visible_features_on_a_sloped_face_without_exposing_back_edges() {
        let face = EntityRef {
            asset_id: 1,
            revision: 1,
            instance_id: 1,
            body_id: 1,
            kind: EntityKind::Face,
            id: 10,
        };
        let mut scene = CadScene {
            triangles: vec![
                Triangle {
                    points: [[0., 0., 0.], [10., 0., 10.], [10., 10., 10.]],
                    face: face.clone(),
                },
                Triangle {
                    points: [[0., 0., 0.], [10., 10., 10.], [0., 10., 0.]],
                    face: face.clone(),
                },
            ],
            ..Default::default()
        };
        for kind in [EntityKind::Edge, EntityKind::Loop] {
            scene.paths.push(Path {
                points: vec![[0., 0., 0.], [0., 10., 0.]],
                entity: EntityRef {
                    kind,
                    id: 20,
                    ..face.clone()
                },
            });
            scene.paths.push(Path {
                points: vec![[0., 0., -1.], [0., 10., -1.]],
                entity: EntityRef {
                    kind,
                    id: 30,
                    ..face.clone()
                },
            });
        }
        scene.vertices.push(Vertex {
            point: [0., 5., 0.],
            entity: EntityRef {
                kind: EntityKind::Vertex,
                id: 40,
                ..face
            },
        });
        scene.build_node(vec![0, 1]);
        // The cursor is slightly inside the face. Its surface is higher than the
        // visible edge caught by the aperture, but does not cover that edge.
        let ray = Ray {
            origin: [0.1, 5., 20.],
            direction: [0., 0., -1.],
        };
        for kind in [EntityKind::Edge, EntityKind::Loop, EntityKind::Vertex] {
            let picks = scene.pick_candidates(ray, kind, 0.2, false);
            assert_eq!(picks.len(), 1, "visible {kind:?} must remain selectable");
            assert!(!picks[0].occluded);
        }
        let through = scene.pick_candidates(ray, EntityKind::Edge, 0.2, true);
        assert_eq!(through.len(), 2);
        assert!(!through[0].occluded);
        assert!(through[1].occluded);
    }
    #[test]
    fn repeated_instances_have_distinct_ids_and_hidden_instances_cannot_pick() {
        let mut geometry = geometry();
        let mut second = geometry.instances[0].clone();
        second.id = 2;
        second.placement.translation = [0., 0., 4.];
        geometry.instances.push(second);
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        let ray = Ray {
            origin: [0., 0., 10.],
            direction: [0., 0., -1.],
        };
        assert_eq!(
            scene.pick_candidates(ray, EntityKind::Face, 0.1, false)[0]
                .entity
                .instance_id,
            2
        );
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::from([2]));
        assert_eq!(
            scene.pick_candidates(ray, EntityKind::Face, 0.1, false)[0]
                .entity
                .instance_id,
            1
        );
    }
    #[test]
    fn rotated_instance_and_camera_pick_the_correct_face() {
        let mut geometry = geometry();
        geometry.instances[0].placement = RigidTransform {
            rotation: [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]],
            translation: [8., 9., 10.],
        };
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        let (min, max) = scene.bounds().unwrap();
        let mut camera = OrbitCamera::framed(min, max);
        camera.orient = orientation(0., -std::f32::consts::FRAC_PI_2);
        let hits =
            scene.pick_candidates(camera.pick_ray(0., 0., 0.6), EntityKind::Face, 0.1, false);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].entity.id, 10);
    }
    #[test]
    fn triangle_numbering_changes_do_not_change_identity() {
        let mut geometry = geometry();
        let before = CadScene::from_geometry(&geometry, &BTreeSet::new()).pick_candidates(
            down(),
            EntityKind::Face,
            0.1,
            false,
        );
        let mesh = &mut std::sync::Arc::make_mut(&mut geometry.assets[0].topology).mesh;
        mesh.triangles.reverse();
        mesh.triangle_faces.reverse();
        let after = CadScene::from_geometry(&geometry, &BTreeSet::new()).pick_candidates(
            down(),
            EntityKind::Face,
            0.1,
            false,
        );
        assert_eq!(before, after);
    }
    #[test]
    fn hover_uses_selection_weight_only_on_the_target_instance_and_clears() {
        let mut geometry = geometry();
        let mut second = geometry.instances[0].clone();
        second.id = 2;
        second.placement.translation = [8., 0., 0.];
        geometry.instances.push(second);
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        let edge = scene
            .paths
            .iter()
            .find(|p| p.entity.kind == EntityKind::Edge)
            .unwrap()
            .entity
            .clone();
        for kind in [
            EntityKind::Edge,
            EntityKind::Loop,
            EntityKind::Body,
            EntityKind::Face,
            EntityKind::Vertex,
        ] {
            let mut target = edge.clone();
            target.kind = kind;
            target.id = match kind {
                EntityKind::Loop => 50,
                EntityKind::Body => target.body_id,
                EntityKind::Vertex => 40,
                EntityKind::Face => 20,
                _ => target.id,
            };
            let hovered = scene.edges(&[], Some(&target)).line_vertices();
            let selected = scene.edges(&[target.clone()], None).line_vertices();
            let hovered_points: Vec<_> = hovered
                .iter()
                .filter(|v| v.width > 1.)
                .map(|v| (v.position, v.width))
                .collect();
            let selected_points: Vec<_> = selected
                .iter()
                .filter(|v| v.width > 1.)
                .map(|v| (v.position, v.width))
                .collect();
            assert!(!hovered_points.is_empty(), "{kind:?}");
            assert_eq!(hovered_points, selected_points, "{kind:?}");
            assert!(hovered_points
                .iter()
                .all(|(p, width)| p[0] < 4. && *width == 3.));
            assert!(hovered
                .iter()
                .filter(|v| v.position[0] > 4.)
                .all(|v| v.width == 1.));
        }
        assert!(scene
            .edges(&[], None)
            .line_vertices()
            .iter()
            .all(|v| v.width == 1.));
    }

    #[test]
    fn complete_face_highlights_only_the_selected_instance() {
        let mut geometry = geometry();
        let mut second = geometry.instances[0].clone();
        second.id = 2;
        second.placement.translation = [8., 0., 0.];
        geometry.instances.push(second);
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        let hit = scene
            .pick_candidates(down(), EntityKind::Face, 0.1, false)
            .remove(0);
        let (vertices, _) = scene.mesh(&[hit.entity], None);
        assert_eq!(
            vertices
                .iter()
                .filter(|v| v.color == [0.2, 0.6, 0.98, 1.])
                .count(),
            3
        );
    }
    #[test]
    fn silhouette_picks_nearest_visible_instance_and_respects_hidden_instances() {
        let mut geometry = geometry();
        let mut other = geometry.instances[0].clone();
        other.id = 2;
        other.placement.translation[2] = 3.;
        geometry.instances.push(other);
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        let ray = Ray {
            origin: [0., 0., 10.],
            direction: [0., 0., -1.],
        };
        let picks = scene.pick_silhouette_candidates(ray, 0.1, false);
        assert_eq!(picks.len(), 1);
        assert_eq!(picks[0].entity.kind, EntityKind::Silhouette);
        assert_eq!(picks[0].entity.id, 1);
        assert_eq!(picks[0].entity.instance_id, 2);
        let through = scene.pick_silhouette_candidates(ray, 0.1, true);
        assert_eq!(through.len(), 2);
        assert!(through[1].occluded);
        let hidden = CadScene::from_geometry(&geometry, &BTreeSet::from([2]));
        assert_eq!(
            hidden.pick_silhouette_candidates(ray, 0.1, false)[0]
                .entity
                .instance_id,
            1
        );
    }

    #[test]
    fn profile_prefers_planar_rim_over_a_wall_loop_with_a_vertical_seam() {
        let mut geometry = geometry();
        let topology = std::sync::Arc::make_mut(&mut geometry.assets[0].topology);
        for (edge, start, end) in [
            (61, [-0.1, 0., 2.], [0.1, 0., 2.]),
            (62, [0., 0., 2.], [0., 0., 0.]),
        ] {
            topology.edges.push(CadEdge {
                id: edge,
                body_id: 1,
                vertices: [40, 41],
                face_ids: vec![10, 70],
                curve: CadCurve::Line { start, end },
            });
        }
        for (wire, face, edge) in [(60, 10, 61), (63, 70, 62)] {
            topology.loops.push(CadLoop {
                id: wire,
                body_id: 1,
                face_id: face,
                is_outer: true,
                edge_uses: vec![EdgeUse {
                    edge_id: edge,
                    reversed: false,
                }],
            });
        }
        topology.faces.push(CadFace {
            id: 70,
            body_id: 1,
            surface: CadSurface::Cylinder {
                origin: [0., 0., 0.],
                axis: [0., 0., 1.],
                radius: 1.,
                inward: false,
            },
            loop_ids: vec![63],
            bounds: [[-1., -1., 0.], [1., 1., 2.]],
        });
        let scene = CadScene::from_geometry(&geometry, &BTreeSet::new());
        let picks = scene.pick_profile_candidates(down(), 0.1, false);
        assert_eq!(picks.len(), 1);
        assert_eq!(picks[0].entity.id, 60);
        assert_eq!(picks[0].entity.kind, EntityKind::Loop);
        geometry.instances[0].placement.rotation = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]];
        let frame =
            cam_model::SetupFrame::from_axes(Some([1., 0., 0.]), Some([0., 0., 1.]), None).unwrap();
        let rotated = CadScene::from_geometry_in_setup(&geometry, &BTreeSet::new(), &frame);
        let ray = Ray {
            origin: [0., -5., 0.],
            direction: [0., 1., 0.],
        };
        let picks = rotated.pick_profile_candidates(ray, 0.1, false);
        assert_eq!(
            picks[0].entity.id, 60,
            "profile loop planarity follows the setup axes"
        );
    }
}
