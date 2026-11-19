//! Resolve independent pocket features without merging their islands or linking
//! across uncut material. A 2.5D operation has one common cutting interval.
use super::*;

/// Keep the selected floor's machining outline and its raised islands.
/// Descending cutouts within that outline can be cleared across.
pub(super) fn floor_loop_ids(
    view: &View,
    face: &CadFace,
    floor: f64,
) -> Result<Vec<EntityId>, String> {
    let outer_center = circular_wire_center(view, view.outer_wire(face)?);
    let mut kept = Vec::new();
    for &id in &face.loop_ids {
        let wire = view.wire(id)?;
        if !descends_below_floor(view, wire, floor) || is_annular_inner_rim(view, wire, outer_center) {
            kept.push(id);
        }
    }
    Ok(kept)
}

fn is_annular_inner_rim(
    view: &View,
    wire: &CadLoop,
    outer_center: Option<[f64; 3]>,
) -> bool {
    // Concentric closed wires define the two rims of an annular floor.
    // The inner rim bounds the selected face at every diameter.
    outer_center.is_some_and(|outer| {
        circular_wire_center(view, wire).is_some_and(|inner| distance(outer, inner) <= T)
    })
}

fn circular_wire_center(view: &View, wire: &CadLoop) -> Option<[f64; 3]> {
    let mut common_center = None;
    let mut sweep = 0.;
    for usage in &wire.edge_uses {
        let CadCurve::CircleArc {
            center,
            start_angle,
            end_angle,
            ..
        } = view.edge(usage.edge_id).ok()?.curve
        else {
            return None;
        };
        if distance(center, *common_center.get_or_insert(center)) > T {
            return None;
        }
        sweep += (end_angle - start_angle).abs();
    }
    common_center.filter(|_| (sweep - TAU).abs() <= T)
}

pub(super) fn descends_below_floor(view: &View, wire: &CadLoop, floor: f64) -> bool {
    !wire.is_outer
        && view.body.valid_solid
        && !wire.edge_uses.is_empty()
        && wire.edge_uses.iter().all(|usage| {
            let Ok(edge) = view.edge(usage.edge_id) else {
                return false;
            };
            let mut adjacent = edge.face_ids.iter().filter(|&&id| id != wire.face_id);
            let (Some(&id), None) = (adjacent.next(), adjacent.next()) else {
                return false;
            };
            let Ok(wall) = view.face(id) else {
                return false;
            };
            let [[_, _, lo], [_, _, hi]] = view.bounds(wall.bounds);
            if hi <= floor + T {
                return lo < floor - T;
            }
            // Transforming a native bounding box can overestimate Z after a
            // setup rotation. For planes and Z-aligned extrusions, the exact
            // boundary controls the full height range.
            (matches!(wall.surface, CadSurface::Plane { .. }) || view.vertical(wall))
                && view
                    .extent(wall)
                    .is_ok_and(|(lo, hi)| lo < floor - T && hi <= floor + T)
        })
}

pub(super) fn resolve(
    state: &GeometryState,
    binding: &GeometryBinding,
    advisory: bool,
) -> Option<Result<ResolvedInput, String>> {
    if binding.selected.len() < 2
        || !matches!(
            binding.recipe,
            ExtractionRecipe::PocketFloor | ExtractionRecipe::PocketBoundary
        )
    {
        return None;
    }
    let groups = match groups(state, binding) {
        Ok(groups) if groups.len() > 1 => groups,
        Ok(_) => return None,
        Err(reason) => return Some(Err(reason)),
    };
    Some((|| {
        let mut combined: Option<ResolvedInput> = None;
        let mut fingerprints = Vec::new();
        for selected in groups {
            let child = GeometryBinding {
                selected,
                overrides: Default::default(),
                ..binding.clone()
            };
            let input = if advisory {
                resolve_binding_advisory(state, &child)?
            } else {
                resolve_binding(state, &child)?
            };
            fingerprints.push(input.fingerprint.clone());
            if let Some(all) = &mut combined {
                if (all.heights.bottom - input.heights.bottom).abs() > T {
                    return Err("Selected pockets have different floor heights; use separate operations with their own cutting heights".into());
                }
                all.heights.top = all.heights.top.max(input.heights.top);
                all.regions.extend(input.regions);
                all.diagnostics.extend(input.diagnostics);
            } else {
                combined = Some(input);
            }
        }
        let mut input = combined.unwrap();
        let base = input.heights;
        input.heights.top = binding.overrides.top.unwrap_or(base.top);
        input.heights.bottom = binding.overrides.bottom.unwrap_or(base.bottom);
        input.heights.validate()?;
        if !advisory {
            if binding.recipe == ExtractionRecipe::PocketFloor
                && input.heights.bottom < base.bottom - T
            {
                return Err(
                    "bottom override extends below the geometry-certified cutting interval".into(),
                );
            }
            if input.heights.top < base.top - T {
                return Err(
                    "top override skips material above the geometry-certified entry plane".into(),
                );
            }
        }
        let encoded = serde_json::to_vec(&(fingerprints, &binding.overrides, input.heights))
            .map_err(|e| e.to_string())?;
        let hash = encoded.into_iter().fold(0xcbf29ce484222325_u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x100000001b3)
        });
        input.fingerprint = format!("{hash:016x}");
        input.diagnostics.dedup();
        Ok(input)
    })())
}

fn groups(state: &GeometryState, binding: &GeometryBinding) -> Result<Vec<Vec<EntityRef>>, String> {
    let refs = &binding.selected;
    for (index, reference) in refs.iter().enumerate() {
        if !state.entity_exists(reference) {
            return Err("Needs reselection: missing entity or stale asset revision".into());
        }
        if refs[..index].contains(reference) {
            return Err("The same geometry feature was selected more than once".into());
        }
    }
    if binding.recipe == ExtractionRecipe::PocketFloor {
        if refs.iter().any(|r| r.kind != EntityKind::Face) {
            return Err("Select planar pocket floor faces".into());
        }
        return Ok(refs.iter().cloned().map(|r| vec![r]).collect());
    }
    if refs.iter().all(|r| r.kind == EntityKind::Loop) {
        return Ok(refs.iter().cloned().map(|r| vec![r]).collect());
    }
    if refs.iter().any(|r| r.kind != EntityKind::Edge) {
        return Err("Select closed boundary loops or closed edge chains".into());
    }
    // Connected components keep each closed edge chain together. Endpoint
    // comparison happens in setup coordinates, including instance placements.
    let endpoints = refs
        .iter()
        .map(|r| {
            let view = View::for_path(state, std::slice::from_ref(r))?;
            let points = view.curve_points(view.edge(r.id)?, false, MACHINING_CHORD_TOLERANCE)?;
            Ok([points[0], *points.last().unwrap()])
        })
        .collect::<Result<Vec<_>, String>>()?;
    let mut remaining: Vec<usize> = (0..refs.len()).collect();
    let mut groups = Vec::new();
    while !remaining.is_empty() {
        let mut component = vec![remaining.remove(0)];
        let mut cursor = 0;
        while cursor < component.len() {
            let current = component[cursor];
            let mut index = 0;
            while index < remaining.len() {
                let candidate = remaining[index];
                let same_instance = refs[current].instance_id == refs[candidate].instance_id
                    && refs[current].asset_id == refs[candidate].asset_id
                    && refs[current].body_id == refs[candidate].body_id;
                if same_instance
                    && endpoints[current]
                        .iter()
                        .any(|a| endpoints[candidate].iter().any(|b| distance(*a, *b) < T))
                {
                    component.push(remaining.remove(index));
                } else {
                    index += 1;
                }
            }
            cursor += 1;
        }
        groups.push(component.into_iter().map(|i| refs[i].clone()).collect());
    }
    Ok(groups)
}
