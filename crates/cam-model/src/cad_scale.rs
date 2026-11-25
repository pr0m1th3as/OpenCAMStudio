//! Scale CAD snapshots in world axes while keeping instance placements rigid.
use std::collections::BTreeMap;
use std::sync::Arc;

use crate::{
    BodyId, CadBezierSpan, CadCurve, CadMesh, CadSurface, EntityId, GeometryInstance,
    RigidTransform, TopologySnapshot,
};

type Vector = [f64; 3];

fn dot(a: Vector, b: Vector) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}

fn cross(a: Vector, b: Vector) -> Vector {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn length([x, y, z]: Vector) -> f64 {
    x.hypot(y).hypot(z)
}

fn unit(v: Vector) -> Vector {
    let norm = length(v);
    v.map(|component| component / norm)
}

#[derive(Clone, Copy, PartialEq)]
struct Scale {
    /// World scaling expressed in the body's local coordinate system.
    linear: [Vector; 3],
    normal: [Vector; 3],
    uniform: Option<f64>,
}

impl Scale {
    fn new(factors: Vector, placement: RigidTransform) -> Self {
        let uniform = (factors[0] == factors[1] && factors[0] == factors[2]).then_some(factors[0]);
        let matrix = |values: Vector| {
            std::array::from_fn(|i| {
                std::array::from_fn(|j| {
                    if values[0] == values[1] && values[0] == values[2] {
                        return if i == j { values[0] } else { 0. };
                    }
                    (0..3)
                        .map(|k| placement.rotation[k][i] * values[k] * placement.rotation[k][j])
                        .sum()
                })
            })
        };
        let min = factors.into_iter().fold(f64::INFINITY, f64::min);
        Self {
            linear: matrix(factors),
            // A common multiplier on the inverse transpose cancels on normalization.
            normal: matrix(factors.map(|f| min / f)),
            uniform,
        }
    }

    fn point(self, v: Vector) -> Vector {
        self.linear.map(|row| dot(row, v))
    }

    fn normal(self, v: Vector) -> Vector {
        unit(self.normal.map(|row| dot(row, v)))
    }

    fn bounds(self, [min, max]: [Vector; 2]) -> [Vector; 2] {
        let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        for mask in 0..8 {
            let p = self.point(std::array::from_fn(|i| {
                if mask & (1 << i) == 0 {
                    min[i]
                } else {
                    max[i]
                }
            }));
            if !p.into_iter().all(f64::is_finite) {
                return [p; 2];
            }
            for (i, value) in p.into_iter().enumerate() {
                bounds[0][i] = bounds[0][i].min(value);
                bounds[1][i] = bounds[1][i].max(value);
            }
        }
        bounds
    }

    fn circle_scale(x: Vector, y: Vector) -> Option<f64> {
        let (a, b) = (length(x), length(y));
        (a > 0. && b > 0. && (a / b - 1.).abs() < 1e-9 && dot(unit(x), unit(y)).abs() < 1e-9)
            .then_some(a)
    }

    fn radial_and_axial(self, axis: Vector) -> Option<(f64, f64)> {
        if let Some(factor) = self.uniform {
            return Some((factor, factor));
        }
        let basis = if axis[0].abs() < 0.8 {
            [1., 0., 0.]
        } else {
            [0., 1., 0.]
        };
        let x = unit(cross(axis, basis));
        let (x, y, z) = (self.point(x), self.point(cross(axis, x)), self.point(axis));
        let radial = Self::circle_scale(x, y)?;
        let axial = length(z);
        (axial > 0. && dot(unit(z), unit(cross(unit(x), unit(y)))) > 1. - 1e-12)
            .then_some((radial, axial))
    }

    fn surface(self, surface: &CadSurface) -> CadSurface {
        use CadSurface::*;
        let unsupported = || Unsupported {
            description: format!("{} after independent-axis scaling", surface.kind_name()),
        };
        match *surface {
            Plane { origin, normal } => Plane {
                origin: self.point(origin),
                normal: self.normal(normal),
            },
            Cylinder {
                origin,
                axis,
                radius,
                inward,
            } => self
                .radial_and_axial(axis)
                .map_or_else(unsupported, |(radial, _)| Cylinder {
                    origin: self.point(origin),
                    axis: unit(self.point(axis)),
                    radius: radius * radial,
                    inward,
                }),
            Cone {
                origin,
                axis,
                radius,
                semi_angle,
            } => self
                .radial_and_axial(axis)
                .map_or_else(unsupported, |(radial, axial)| Cone {
                    origin: self.point(origin),
                    axis: unit(self.point(axis)),
                    radius: radius * radial,
                    semi_angle: (semi_angle.tan() * (radial / axial)).atan(),
                }),
            Sphere { center, radius } => self.uniform.map_or_else(unsupported, |factor| Sphere {
                center: self.point(center),
                radius: radius * factor,
            }),
            Torus {
                origin,
                axis,
                major_radius,
                minor_radius,
            } => self.uniform.map_or_else(unsupported, |factor| Torus {
                origin: self.point(origin),
                axis,
                major_radius: major_radius * factor,
                minor_radius: minor_radius * factor,
            }),
            LinearExtrusion {
                axis,
                ref edge_sides,
            } => LinearExtrusion {
                axis: unit(self.point(axis)),
                edge_sides: edge_sides
                    .iter()
                    .cloned()
                    .map(|mut side| {
                        side.tangent = unit(self.point(side.tangent));
                        side.outward = self.normal(side.outward);
                        side
                    })
                    .collect(),
            },
            Unsupported { .. } => surface.clone(),
        }
    }

    fn curve(self, curve: &CadCurve) -> Result<CadCurve, String> {
        use CadCurve::*;
        Ok(match *curve {
            Line { start, end } => Line {
                start: self.point(start),
                end: self.point(end),
            },
            CircleArc {
                center,
                normal,
                x_axis,
                radius,
                start_angle,
                end_angle,
            } => {
                let x = self.point(x_axis);
                let y = self.point(cross(normal, x_axis));
                let center = self.point(center);
                if let Some(factor) = self.uniform.or_else(|| Self::circle_scale(x, y)) {
                    CircleArc {
                        center,
                        normal: self.normal(normal),
                        x_axis: unit(x),
                        radius: radius * factor,
                        start_angle,
                        end_angle,
                    }
                } else {
                    // A stretched circular arc is an exact rational quadratic per
                    // quarter turn, with the original sweep and edge direction.
                    let sweep = end_angle - start_angle;
                    let count = (sweep.abs() / std::f64::consts::FRAC_PI_2).ceil().max(1.) as usize;
                    if count > 32 {
                        return Err("Cannot scale a CAD arc with an invalid angular sweep".into());
                    }
                    let pole = |angle: f64, weight: f64| {
                        let (sin, cos) = angle.sin_cos();
                        std::array::from_fn(|i| {
                            center[i] + radius * (cos * x[i] + sin * y[i]) / weight
                        })
                    };
                    BezierSpans {
                        spans: (0..count)
                            .map(|i| {
                                let start = start_angle + sweep * (i as f64 / count as f64);
                                let end = start_angle + sweep * ((i + 1) as f64 / count as f64);
                                let half = (end - start) / 2.;
                                let weight = half.cos();
                                CadBezierSpan {
                                    poles: vec![
                                        pole(start, 1.),
                                        pole(start + half, weight),
                                        pole(end, 1.),
                                    ],
                                    weights: vec![1., weight, 1.],
                                }
                            })
                            .collect(),
                    }
                }
            }
            BezierSpans { ref spans } => BezierSpans {
                spans: spans
                    .iter()
                    .cloned()
                    .map(|mut span| {
                        for p in &mut span.poles {
                            *p = self.point(*p);
                        }
                        span
                    })
                    .collect(),
            },
            Unsupported { .. } => curve.clone(),
        })
    }
}

#[derive(Clone, Copy)]
struct BodyScale {
    scale: Scale,
    offset: EntityId,
}

/// Scale all instances of an asset. An instanced body can still share its
/// topology whenever its local transform is the same in each placement.
pub fn scale_cad_topology(
    source: &Arc<TopologySnapshot>,
    instances: &mut [GeometryInstance],
    factors: Vector,
) -> Result<(Arc<TopologySnapshot>, BTreeMap<u64, EntityId>), String> {
    if !factors.into_iter().all(|f| f.is_finite() && f > 0.) {
        return Err("Scale factors must be finite and greater than zero".into());
    }
    source.validate_cached()?;
    let initial_max = source
        .bodies
        .iter()
        .map(|b| b.id)
        .chain(source.faces.iter().map(|f| f.id))
        .chain(source.edges.iter().map(|e| e.id))
        .chain(source.loops.iter().map(|l| l.id))
        .chain(source.vertices.iter().map(|v| v.id))
        .max()
        .unwrap_or(0)
        .max(1);
    let mut max_id = initial_max;
    let mut body_scales: BTreeMap<BodyId, Vec<BodyScale>> = source
        .bodies
        .iter()
        .map(|body| (body.id, Vec::new()))
        .collect();
    let mut offsets = BTreeMap::new();
    for instance in instances.iter() {
        let variants = body_scales
            .get_mut(&instance.body_id)
            .ok_or("Missing CAD body")?;
        let scale = Scale::new(factors, instance.placement);
        let offset = if let Some(entry) = variants.iter().find(|entry| entry.scale == scale) {
            entry.offset
        } else {
            let offset = if variants.is_empty() {
                0
            } else {
                let offset = max_id.checked_add(1).ok_or("CAD entity IDs exhausted")?;
                max_id = offset
                    .checked_add(initial_max)
                    .ok_or("CAD entity IDs exhausted")?;
                offset
            };
            variants.push(BodyScale { scale, offset });
            offset
        };
        if offsets.insert(instance.id, offset).is_some() {
            return Err("Duplicate CAD instance ID".into());
        }
        if !instance.placement.is_rigid()
            || instance
                .placement
                .translation
                .into_iter()
                .zip(factors)
                .any(|(v, factor)| !(v * factor).is_finite())
        {
            return Err("Scaled CAD placement is out of range".into());
        }
    }
    for variants in body_scales
        .values_mut()
        .filter(|variants| variants.is_empty())
    {
        variants.push(BodyScale {
            scale: Scale::new(factors, RigidTransform::default()),
            offset: 0,
        });
    }

    let mut scaled = TopologySnapshot {
        version: source.version,
        ..Default::default()
    };
    for body in &source.bodies {
        for &BodyScale { scale, offset } in &body_scales[&body.id] {
            let mut body = body.clone();
            let original = source.source_body_id(body.id);
            body.id += offset;
            if body.id != original {
                scaled.body_sources.push((body.id, original));
            }
            body.bounds = scale.bounds(body.bounds);
            scaled.bodies.push(body);
        }
    }
    let mut face_scales = BTreeMap::new();
    for face in &source.faces {
        let variants = &body_scales[&face.body_id];
        face_scales.insert(face.id, (face.body_id, variants.as_slice()));
        for &BodyScale { scale, offset } in variants {
            let mut face = face.clone();
            face.id += offset;
            face.body_id += offset;
            face.bounds = scale.bounds(face.bounds);
            face.surface = scale.surface(&face.surface);
            if let CadSurface::LinearExtrusion { edge_sides, .. } = &mut face.surface {
                for side in edge_sides {
                    side.edge_id += offset;
                }
            }
            face.loop_ids.iter_mut().for_each(|id| *id += offset);
            scaled.faces.push(face);
        }
    }
    for edge in &source.edges {
        for &BodyScale { scale, offset } in &body_scales[&edge.body_id] {
            let mut edge = edge.clone();
            edge.id += offset;
            edge.body_id += offset;
            edge.curve = scale.curve(&edge.curve)?;
            edge.vertices.iter_mut().for_each(|id| *id += offset);
            edge.face_ids.iter_mut().for_each(|id| *id += offset);
            scaled.edges.push(edge);
        }
    }
    for contour in &source.loops {
        for &BodyScale { offset, .. } in &body_scales[&contour.body_id] {
            let mut contour = contour.clone();
            contour.id += offset;
            contour.body_id += offset;
            contour.face_id += offset;
            contour
                .edge_uses
                .iter_mut()
                .for_each(|edge| edge.edge_id += offset);
            scaled.loops.push(contour);
        }
    }
    for vertex in &source.vertices {
        for &BodyScale { scale, offset } in &body_scales[&vertex.body_id] {
            let mut vertex = vertex.clone();
            vertex.id += offset;
            vertex.body_id += offset;
            vertex.point = scale.point(vertex.point);
            scaled.vertices.push(vertex);
        }
    }
    scaled.mesh = scale_mesh(&source.mesh, &face_scales)?;
    if let Some(projection) = &source.projection {
        let factor = factors.into_iter().fold(0., f64::max);
        scaled.projection = Some(crate::CadProjectionMesh {
            mesh: scale_mesh(&projection.mesh, &face_scales)?,
            chord_tolerance: projection.chord_tolerance * factor,
        });
    }
    let scaled = Arc::new(scaled);
    scaled.validate_cached()?;
    for instance in instances {
        instance.body_id += offsets[&instance.id];
        for (value, factor) in instance.placement.translation.iter_mut().zip(factors) {
            *value *= factor;
        }
    }
    Ok((scaled, offsets))
}

fn scale_mesh(
    mesh: &CadMesh,
    faces: &BTreeMap<EntityId, (BodyId, &[BodyScale])>,
) -> Result<CadMesh, String> {
    let mut scaled = CadMesh::default();
    let mut indices = std::collections::HashMap::new();
    for (triangle, &face) in mesh.triangles.iter().zip(&mesh.triangle_faces) {
        let (body, scales) = faces[&face];
        for &BodyScale { scale, offset } in scales {
            let mut triangle = *triangle;
            for index in &mut triangle {
                *index = match indices.entry((body, offset, *index)) {
                    std::collections::hash_map::Entry::Occupied(entry) => *entry.get(),
                    std::collections::hash_map::Entry::Vacant(entry) => {
                        let new_index = u32::try_from(scaled.vertices.len())
                            .map_err(|_| "Scaled CAD mesh exceeds the vertex limit")?;
                        scaled
                            .vertices
                            .push(scale.point(mesh.vertices[*index as usize]));
                        *entry.insert(new_index)
                    }
                };
            }
            scaled.triangles.push(triangle);
            scaled.triangle_faces.push(face + offset);
        }
    }
    Ok(scaled)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn independent_scale_converts_circle_to_exact_ellipse() {
        let circle = CadCurve::CircleArc {
            center: [1., 2., 3.],
            normal: [0., 0., 1.],
            x_axis: [1., 0., 0.],
            radius: 2.,
            start_angle: 0.,
            end_angle: std::f64::consts::TAU,
        };
        let scale = Scale::new([2., 3., 4.], RigidTransform::default());
        let CadCurve::BezierSpans { spans } = scale.curve(&circle).unwrap() else {
            panic!("the stretched circle should be an exact spline");
        };
        let points = crate::approximate_bezier_spans(&spans, 0.001).unwrap();
        assert_eq!(points[0], [6., 6., 12.]);
        for point in points {
            let value = ((point[0] - 2.) / 4.).powi(2) + ((point[1] - 6.) / 6.).powi(2);
            assert!((value - 1.).abs() < 1e-12);
            assert!((point[2] - 12.).abs() < 1e-12);
        }
    }
}
