//! Independent CAD projection for surface finishing, including unselected obstacles.
use super::*;
#[derive(Clone, Debug)]
pub struct ParallelGeometry {
    pub triangles: Vec<[[f64; 3]; 3]>,
    pub selected: Vec<[[f64; 3]; 3]>,
    pub error_bound: f64,
    pub heights: CuttingHeights,
}
pub fn resolve_parallel(
    state: &GeometryState,
    binding: &GeometryBinding,
) -> Result<ParallelGeometry, String> {
    let strategy = match binding.recipe {
        ExtractionRecipe::ParallelFinishing => "Parallel",
        ExtractionRecipe::SpiralFinishing => "Spiral",
        _ => return Err("Surface finishing requires a finishing geometry binding".into()),
    };
    validate_adaptive_selection(state, &binding.selected)?;
    let mut out = ParallelGeometry {
        triangles: vec![],
        selected: vec![],
        error_bound: 0.,
        heights: binding.effective_heights(),
    };
    for instance in &state.instances {
        if !instance.placement.is_rigid() {
            return Err("Invalid setup model placement".into());
        }
        let asset = state
            .asset(instance.asset_id)
            .ok_or("Missing setup asset")?;
        asset.topology.validate_cached()?;
        let body = asset
            .topology
            .bodies
            .iter()
            .find(|b| b.id == instance.body_id)
            .ok_or("Missing setup body")?;
        if !body.valid_solid {
            return Err(format!("{strategy} requires valid solid setup bodies"));
        }
        let projection = adaptive::checked_projection(&asset.topology)?;
        let faces: Vec<_> = asset
            .topology
            .faces
            .iter()
            .filter(|f| f.body_id == body.id)
            .map(|f| f.id)
            .collect();
        if faces.is_empty()
            || projection.mesh.triangles.len() != projection.mesh.triangle_faces.len()
            || faces
                .iter()
                .any(|f| !projection.mesh.triangle_faces.contains(f))
        {
            return Err("Incomplete CAD machining projection; reimport the model".into());
        }
        out.triangles.extend(adaptive::triangles_for_faces(
            &projection.mesh,
            &faces,
            instance.placement,
        )?);
        let selected_faces: Vec<_> = faces
            .into_iter()
            .filter(|face| {
                binding.selected.iter().any(|r| {
                    r.asset_id == instance.asset_id
                        && r.instance_id == instance.id
                        && r.body_id == body.id
                        && (r.kind == EntityKind::Body || r.id == *face)
                })
            })
            .collect();
        out.selected.extend(adaptive::triangles_for_faces(
            &projection.mesh,
            &selected_faces,
            instance.placement,
        )?);
        out.error_bound = out.error_bound.max(projection.chord_tolerance);
    }
    if out.selected.is_empty() {
        return Err(format!(
            "{strategy} selection contains no machining triangles"
        ));
    }
    Ok(out)
}
