use super::*;

const DIAMETER_TOLERANCE: f64 = 1e-3;

#[derive(Clone, Copy, Debug)]
pub struct DrillHole {
    pub center: Point,
    pub diameter: f64,
    pub heights: CuttingHeights,
}

struct Segment {
    hole: DrillHole,
    faces: Vec<u64>,
    top_rim: BTreeSet<u64>,
    bottom_rim: BTreeSet<u64>,
}

impl Segment {
    fn same_cylinder(&self, hole: DrillHole) -> bool {
        self.hole.center.distance(hole.center) < T
            && (self.hole.diameter - hole.diameter).abs() < T
            && (self.hole.heights.top - hole.heights.top).abs() < T
            && (self.hole.heights.bottom - hole.heights.bottom).abs() < T
    }

    fn complete(&self, view: &View<'_>) -> bool {
        [&self.top_rim, &self.bottom_rim].into_iter().all(|rim| {
            let sweep: f64 = rim
                .iter()
                .filter_map(|id| match view.edge(*id).ok()?.curve {
                    CadCurve::CircleArc {
                        start_angle,
                        end_angle,
                        ..
                    } => Some((end_angle - start_angle).abs()),
                    _ => None,
                })
                .sum();
            (sweep - TAU).abs() < T
        })
    }

    fn has_entry(&self, view: &View<'_>) -> bool {
        self.top_rim.iter().all(|&id| {
            view.edge(id).is_ok_and(|edge| {
                edge.face_ids.iter().any(|&face_id| {
                    if self.faces.contains(&face_id) {
                        return false;
                    }
                    view.face(face_id).is_ok_and(|face| match face.surface {
                        CadSurface::Plane { .. } => {
                            view.plane(face).is_ok_and(|(_, normal)| normal[2] > 0.)
                                && face.loop_ids.iter().any(|wire| {
                                    view.wire(*wire).is_ok_and(|wire| {
                                        !wire.is_outer
                                            && wire.edge_uses.iter().any(|u| u.edge_id == id)
                                    })
                                })
                        }
                        CadSurface::Cylinder { inward: true, .. }
                        | CadSurface::Cone { .. }
                        | CadSurface::Torus { .. } => view
                            .extent(face)
                            .is_ok_and(|(_, top)| top > self.hole.heights.top + T),
                        _ => false,
                    })
                })
            })
        })
    }
}

/// Resolve the selected cylindrical faces plus any matches on the selected body.
/// The returned sites are disposable. The binding retains only explicit seeds.
pub fn drill_holes(
    state: &GeometryState,
    binding: &GeometryBinding,
    options: DrillGeometry,
) -> Result<Vec<DrillHole>, String> {
    let view = View::for_path(state, &binding.selected)?;
    let mut segments: Vec<Segment> = Vec::new();
    for face in view
        .asset
        .topology
        .faces
        .iter()
        .filter(|f| f.body_id == view.body.id)
    {
        let CadSurface::Cylinder {
            origin,
            axis,
            radius,
            inward: true,
        } = face.surface
        else {
            continue;
        };
        let axis = view.vector(axis);
        if axis[0].abs() > T || axis[1].abs() > T {
            continue;
        }
        let Ok((bottom, top)) = view.extent(face) else {
            continue;
        };
        let heights = CuttingHeights { top, bottom };
        if heights.validate().is_err() {
            continue;
        }
        let center = view.point(origin);
        let hole = DrillHole {
            center: Point::new(center[0], center[1]),
            diameter: 2. * radius,
            heights,
        };
        let index = segments
            .iter()
            .position(|segment| segment.same_cylinder(hole))
            .unwrap_or_else(|| {
                segments.push(Segment {
                    hole,
                    faces: Vec::new(),
                    top_rim: BTreeSet::new(),
                    bottom_rim: BTreeSet::new(),
                });
                segments.len() - 1
            });
        let segment = &mut segments[index];
        segment.faces.push(face.id);
        for id in &face.loop_ids {
            for usage in &view.wire(*id)?.edge_uses {
                let edge = view.edge(usage.edge_id)?;
                let CadCurve::CircleArc {
                    center,
                    normal,
                    radius: rim_radius,
                    ..
                } = edge.curve
                else {
                    continue;
                };
                let point = view.point(center);
                if (radius - rim_radius).abs() > T
                    || hole.center.distance(Point::new(point[0], point[1])) > T
                    || (view.vector(normal)[2].abs() - 1.).abs() > T
                {
                    continue;
                }
                if (point[2] - top).abs() < T {
                    segment.top_rim.insert(edge.id);
                } else if (point[2] - bottom).abs() < T {
                    segment.bottom_rim.insert(edge.id);
                }
            }
        }
    }
    let mut picked = Vec::new();
    for reference in &binding.selected {
        let index = segments
            .iter()
            .position(|segment| {
                reference.kind == EntityKind::Face && segment.faces.contains(&reference.id)
            })
            .ok_or("Select inward cylindrical hole faces aligned with setup Z")?;
        if !picked.contains(&index) {
            picked.push(index);
        }
    }
    let complete: Vec<_> = segments
        .iter()
        .map(|segment| segment.complete(&view))
        .collect();
    let mut selected = picked.clone();
    if options.select_same_diameter {
        selected.extend((0..segments.len()).filter(|&i| {
            complete[i]
                && !picked.contains(&i)
                && picked.iter().any(|&j| {
                    (segments[i].hole.diameter - segments[j].hole.diameter).abs()
                        <= DIAMETER_TOLERANCE
                })
                && segments[i].has_entry(&view)
        }));
    }
    let mut holes: Vec<DrillHole> = Vec::new();
    for index in selected {
        let mut hole = segments[index].hole;
        if options.auto_merge_hole_segments {
            let mut current = index;
            while let Some(upper) = (0..segments.len()).find(|&i| {
                let next = &segments[i];
                let lower = &segments[current];
                complete[i]
                    && next.hole.center.distance(hole.center) < T
                    && next.hole.diameter >= lower.hole.diameter - T
                    && next.hole.heights.top > lower.hole.heights.top + T
                    && next.hole.heights.bottom >= lower.hole.heights.top - T
                    && connected(&view, lower, next)
            }) {
                hole.heights.top = segments[upper].hole.heights.top;
                current = upper;
            }
        }
        hole.heights.top = binding.overrides.top.unwrap_or(hole.heights.top);
        hole.heights.bottom = binding.overrides.bottom.unwrap_or(hole.heights.bottom);
        hole.heights.validate()?;
        if let Some(previous) = holes.iter_mut().find(|previous| {
            previous.center.distance(hole.center) < T
                && (previous.diameter - hole.diameter).abs() < T
                && (previous.heights.top - hole.heights.top).abs() < T
                && (options.auto_merge_hole_segments
                    || (previous.heights.bottom - hole.heights.bottom).abs() < T)
        }) {
            previous.heights.bottom = previous.heights.bottom.min(hole.heights.bottom);
        } else {
            holes.push(hole);
        }
    }
    Ok(holes)
}

fn connected(view: &View<'_>, lower: &Segment, upper: &Segment) -> bool {
    let neighbors = |rim: &BTreeSet<u64>| {
        rim.iter()
            .filter_map(|id| view.edge(*id).ok())
            .flat_map(|edge| edge.face_ids.iter().copied())
            .collect::<BTreeSet<_>>()
    };
    !lower.top_rim.is_disjoint(&upper.bottom_rim)
        || neighbors(&lower.top_rim)
            .intersection(&neighbors(&upper.bottom_rim))
            .any(|&id| {
                view.face(id).is_ok_and(|face| {
                    matches!(
                        face.surface,
                        CadSurface::Plane { .. } | CadSurface::Cone { .. }
                    )
                })
            })
}

/// Preserve a drill seed when a reload leaves exactly one equivalent cylinder
/// face on the same body. A recycled topology ID alone cannot establish identity.
pub fn rebind_drill_face(
    old: &GeometryAsset,
    replacement: &GeometryAsset,
    reference: &EntityRef,
) -> Option<EntityRef> {
    if reference.kind != EntityKind::Face || reference.revision != old.revision {
        return None;
    }
    let face = old
        .topology
        .faces
        .iter()
        .find(|face| face.id == reference.id && face.body_id == reference.body_id)?;
    let CadSurface::Cylinder {
        origin,
        axis,
        radius,
        inward: true,
    } = face.surface
    else {
        return None;
    };
    let mut matches = replacement.topology.faces.iter().filter(|candidate| {
        let CadSurface::Cylinder {
            origin: next_origin,
            axis: next_axis,
            radius: next_radius,
            inward: true,
        } = candidate.surface
        else {
            return false;
        };
        candidate.body_id == face.body_id
            && (radius - next_radius).abs() < T
            && (axis
                .into_iter()
                .zip(next_axis)
                .map(|(a, b)| a * b)
                .sum::<f64>()
                .abs()
                - 1.)
                .abs()
                < T
            && distance(
                cross(axis, std::array::from_fn(|i| origin[i] - next_origin[i])),
                [0.; 3],
            ) < T
            && candidate
                .bounds
                .iter()
                .flatten()
                .zip(face.bounds.iter().flatten())
                .all(|(a, b)| (a - b).abs() < T)
    });
    let id = matches.next()?.id;
    matches.next().is_none().then(|| EntityRef {
        revision: replacement.revision,
        id,
        ..reference.clone()
    })
}
