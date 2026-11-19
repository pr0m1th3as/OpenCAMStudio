//! Feature recognition is advisory. Exact geometry extraction and numerical
//! validity remain independent of the automatic wall/access/side classifier.
//! The original resolvers remain available for verification and future policy.
use super::*;

pub fn verify_profile_entry(state: &GeometryState, entry: &ProfileGeometryEntry) -> Option<String> {
    let ProfileGeometrySource::Cad(selected) = &entry.source else {
        return None;
    };
    if selected.is_empty() {
        return None;
    }
    let result = if entry.kind == ProfileGeometryKind::Silhouette {
        selected.iter().try_for_each(|reference| {
            let body = EntityRef {
                kind: EntityKind::Silhouette,
                id: reference.body_id,
                ..reference.clone()
            };
            resolve_selection(state, &[body], ExtractionRecipe::ProfileSilhouette, 0.).map(|_| ())
        })
    } else {
        profile_entries::body_groups(selected)
            .iter()
            .try_for_each(|group| profile_contours::extract(state, group).map(|_| ()))
    };
    result.err()
}

pub fn verify_selection(
    state: &GeometryState,
    selected: &[EntityRef],
    recipe: ExtractionRecipe,
    stock_top: f64,
) -> Option<String> {
    if let ExtractionRecipe::ProfileEntries { entries } = &recipe {
        let mut findings = Vec::new();
        let mut seen = Vec::new();
        let mut interval: Option<CuttingHeights> = None;
        for entry in entries {
            if let Some(reason) = verify_profile_entry(state, entry) {
                findings.push(reason);
            }
            if let Ok((contours, heights)) = profile_entry_contours(state, entry) {
                for contour in contours {
                    if seen
                        .iter()
                        .any(|previous| equivalent_profile_contours(previous, &contour))
                    {
                        findings.push(format!("Entry '{}' repeats an earlier selected contour; all selections are retained", entry.name));
                    }
                    seen.push(contour);
                }
                if let Some(heights) = heights {
                    if interval.is_some_and(|previous| (previous.top - heights.top).abs() > T) {
                        findings.push(
                            "Selected contours lie on different planes; operation heights are used"
                                .into(),
                        );
                    }
                    interval = Some(heights);
                }
            }
        }
        return (!findings.is_empty()).then(|| findings.join("; "));
    }
    resolve_selection(state, selected, recipe, stock_top)
        .and_then(|binding| resolve_binding(state, &binding))
        .err()
}

pub fn resolve_selection_advisory(
    state: &GeometryState,
    selected: &[EntityRef],
    recipe: ExtractionRecipe,
    stock_top: f64,
) -> Result<GeometryBinding, String> {
    match resolve_selection(state, selected, recipe.clone(), stock_top) {
        Ok(binding) => Ok(binding),
        Err(reason) => {
            if !stock_top.is_finite() {
                return Err("stock top must be finite".into());
            }
            let mut binding = GeometryBinding {
                selected: selected.to_vec(),
                recipe,
                heights: CuttingHeights {
                    top: stock_top,
                    bottom: stock_top - 1.,
                },
                overrides: Default::default(),
                status: ResolutionStatus::Resolved,
                resolved_fingerprint: None,
            };
            binding.heights = fallback(state, &binding, &reason)?.heights;
            Ok(binding)
        }
    }
}

pub fn resolve_binding_advisory(
    state: &GeometryState,
    binding: &GeometryBinding,
) -> Result<ResolvedInput, String> {
    if let Some(input) = pocket_regions::resolve(state, binding, true) {
        return input;
    }
    match resolve_binding(state, binding) {
        Ok(input) => Ok(input),
        Err(reason) => fallback(state, binding, &reason),
    }
}

fn fallback(
    state: &GeometryState,
    binding: &GeometryBinding,
    reason: &str,
) -> Result<ResolvedInput, String> {
    if let Some(input) = pocket_regions::resolve(state, binding, true) {
        return input;
    }
    let refs = &binding.selected;
    let view = View::for_path(state, refs)?;
    let bounds = view.bounds(view.body.bounds);
    let mut heights = CuttingHeights {
        top: bounds[1][2],
        bottom: bounds[0][2],
    };
    let mut profiles = Vec::new();
    let mut holes = Vec::new();
    let regions = match binding.recipe {
        ExtractionRecipe::ProfileContours | ExtractionRecipe::ProfileLoop { .. } => {
            profiles = profile_contours::unverified_contours(state, refs)?;
            if let ExtractionRecipe::ProfileLoop { side } = binding.recipe {
                for profile in &mut profiles {
                    profile.side = side;
                }
            }
            profiles
                .iter()
                .filter(|profile| profile.closed)
                .map(|profile| {
                    Polygon::new(profile.chain.clone()).map_err(|error| error.to_string())
                })
                .collect::<Result<Vec<_>, _>>()?
        }
        ExtractionRecipe::PocketFloor
        | ExtractionRecipe::FaceRegion
        | ExtractionRecipe::CarveRegion => {
            if refs.len() != 1 {
                return Err("Select one region per operation geometry field".into());
            }
            let (region, z) = match refs[0].kind {
                EntityKind::Face => {
                    let face = view.face(refs[0].id)?;
                    let (_, z) = view.contour(view.outer_wire(face)?)?;
                    let region = if binding.recipe == ExtractionRecipe::PocketFloor {
                        view.region_from_loops(pocket_regions::floor_loop_ids(&view, face, z)?)?
                    } else {
                        view.region(face)?
                    };
                    (region, z)
                }
                EntityKind::Loop => {
                    let (contour, z) = view.contour(view.wire(refs[0].id)?)?;
                    (Polygon::new(contour).map_err(|error| error.to_string())?, z)
                }
                _ => return Err("Select a face or closed boundary region".into()),
            };
            heights = if binding.recipe == ExtractionRecipe::CarveRegion {
                CuttingHeights {
                    top: z,
                    bottom: z - binding.heights.depth(),
                }
            } else {
                CuttingHeights {
                    top: binding.heights.top.max(bounds[1][2]),
                    bottom: z,
                }
            };
            vec![region]
        }
        ExtractionRecipe::PocketBoundary => {
            let (contours, z) = surface_geometry::unverified_path(state, refs, false)?;
            if contours.len() != 1 || !contours[0].closed {
                return Err("Pocket requires one closed boundary chain".into());
            }
            heights = CuttingHeights {
                top: z,
                bottom: z - binding.heights.depth(),
            };
            vec![Polygon::new(contours[0].chain.clone()).map_err(|e| e.to_string())?]
        }
        ExtractionRecipe::ChamferLoop | ExtractionRecipe::EngravePath => {
            let (contours, z) = surface_geometry::unverified_path(
                state,
                refs,
                binding.recipe == ExtractionRecipe::ChamferLoop,
            )?;
            profiles = contours;
            heights = CuttingHeights {
                top: z,
                bottom: z - binding.heights.depth(),
            };
            profiles
                .iter()
                .filter(|profile| profile.closed)
                .map(|profile| {
                    Polygon::new(profile.chain.clone()).map_err(|error| error.to_string())
                })
                .collect::<Result<Vec<_>, _>>()?
        }
        ExtractionRecipe::DrillHoles
        | ExtractionRecipe::BoreCylinders { .. }
        | ExtractionRecipe::ThreadCylinders { .. } => {
            for reference in refs {
                if reference.kind != EntityKind::Face {
                    return Err("A circular operation needs cylindrical face geometry".into());
                }
                let face = view.face(reference.id)?;
                let CadSurface::Cylinder { origin, radius, .. } = face.surface else {
                    return Err("A circular operation needs cylindrical face geometry".into());
                };
                let point = view.point(origin);
                holes.push(ResolvedHole {
                    center: Point::new(point[0], point[1]),
                    diameter: radius * 2.,
                });
            }
            Vec::new()
        }
        // A domain/curve that cannot be represented is a geometry extraction
        // issue, rather than a negative feature-classification result.
        _ => return Err(reason.into()),
    };
    heights.top = binding.overrides.top.unwrap_or(heights.top);
    heights.bottom = binding.overrides.bottom.unwrap_or(heights.bottom);
    heights.validate()?;
    let topology = topology_fingerprint(&view.asset.topology, false)?;
    let encoded = serde_json::to_vec(&(
        topology,
        view.asset.id,
        view.asset.revision,
        view.instance.placement,
        refs,
        &binding.recipe,
        &binding.overrides,
        heights,
    ))
    .map_err(|error| error.to_string())?;
    let hash = encoded
        .into_iter()
        .fold(0xcbf29ce484222325_u64, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
        });
    Ok(ResolvedInput {
        chamfers: Vec::new(), profiles, regions, holes, heights,
        fingerprint: format!("{hash:016x}"),
        diagnostics: vec![format!("Automatic geometry verification (info): {reason}. The selected geometry and operation settings are used.")],
    })
}
