use super::*;

pub(super) fn body_groups(selected: &[EntityRef]) -> Vec<Vec<EntityRef>> {
    let mut groups: Vec<Vec<EntityRef>> = Vec::new();
    for reference in selected {
        if let Some(group) = groups.iter_mut().find(|group| {
            let first = &group[0];
            (first.asset_id, first.instance_id, first.body_id)
                == (reference.asset_id, reference.instance_id, reference.body_id)
        }) {
            group.push(reference.clone());
        } else {
            groups.push(vec![reference.clone()]);
        }
    }
    groups
}

/// Resolve named entries independently, allowing faces, chains and silhouettes
/// in one operation with advisory feature verification.
pub fn profile_entry_contours(
    state: &GeometryState,
    entry: &ProfileGeometryEntry,
) -> Result<(Vec<ProfileContour>, Option<CuttingHeights>), String> {
    let mut heights = None;
    let mut contours = match &entry.source {
        ProfileGeometrySource::Contours(contours) => contours.clone(),
        ProfileGeometrySource::Cad(selected)
            if entry.kind == ProfileGeometryKind::Chain && body_groups(selected).len() > 1 =>
        {
            let mut all = Vec::new();
            for group in body_groups(selected) {
                let mut body_entry = entry.clone();
                body_entry.source = ProfileGeometrySource::Cad(group);
                // Apply presentation settings once, after combining the bodies,
                // so per-contour overrides keep their global indices.
                body_entry.reversed = false;
                body_entry.side_rule = ProfileSideRule::StartOutside;
                body_entry.side_overrides.clear();
                let (contours, interval) = profile_entry_contours(state, &body_entry)?;
                if let Some(interval) = interval {
                    merge_heights(&mut heights, interval)?;
                }
                all.extend(contours);
            }
            all
        }
        ProfileGeometrySource::Cad(selected)
            if entry.kind == ProfileGeometryKind::Chain
                && entry.chain_mode == ProfileChainMode::Closed =>
        {
            // Keep automatic wall heights where a complete feature certificate
            // exists. Otherwise a selected closed path uses operation heights,
            // just as an explicit open path or a drawing contour does.
            match profile_contours::extract(state, selected) {
                Ok((contours, interval)) => {
                    heights = Some(interval);
                    contours
                }
                Err(_) => profile_contours::closed_chain(state, selected)?,
            }
        }
        ProfileGeometrySource::Cad(selected)
            if entry.kind == ProfileGeometryKind::Chain
                && entry.chain_mode == ProfileChainMode::Open
                && selected.iter().all(|r| r.kind == EntityKind::Edge) =>
        {
            // Exact planar paths are valid even along blends or unsupported
            // surfaces. Their cutting depth comes from operation settings.
            profile_contours::open_chain(state, selected)?
        }
        ProfileGeometrySource::Cad(selected) => {
            if selected.is_empty() {
                return Err(format!("{}: select geometry", entry.name));
            }
            let mut groups = Vec::new();
            if entry.kind == ProfileGeometryKind::Silhouette {
                if !entry.tolerance.is_finite() || entry.tolerance <= 0. {
                    return Err("Silhouette tolerance must be positive and finite".into());
                }
                for reference in selected {
                    let mut body = reference.clone();
                    body.kind = EntityKind::Silhouette;
                    body.id = body.body_id;
                    let asset = state
                        .asset(body.asset_id)
                        .ok_or("Source asset is missing")?;
                    if asset
                        .topology
                        .projection
                        .as_ref()
                        .is_some_and(|p| p.chord_tolerance > entry.tolerance)
                    {
                        return Err("Silhouette tolerance is finer than the available CAD projection; reimport with a finer projection tolerance".into());
                    }
                    groups.push((vec![body], ExtractionRecipe::ProfileSilhouette));
                }
            } else if entry.kind == ProfileGeometryKind::FaceContours {
                let mut faces = Vec::new();
                for reference in selected {
                    if reference.kind != EntityKind::Face {
                        return Err("Face Contours requires planar faces".into());
                    }
                    let candidates = if entry.coplanar {
                        coplanar_profile_faces(state, reference)
                            .unwrap_or_else(|_| vec![reference.clone()])
                    } else {
                        vec![reference.clone()]
                    };
                    for face in candidates {
                        if !faces.contains(&face) {
                            faces.push(face);
                        }
                    }
                }
                // Different bodies are separately qualified, then combined.
                let mut bodies =
                    std::collections::BTreeMap::<(u64, u64, u64), Vec<EntityRef>>::new();
                for face in faces {
                    bodies
                        .entry((face.asset_id, face.instance_id, face.body_id))
                        .or_default()
                        .push(face);
                }
                groups.extend(
                    bodies
                        .into_values()
                        .map(|faces| (faces, ExtractionRecipe::ProfileContours)),
                );
            } else {
                // Resolve selected loops through the contour extractor. It can
                // certify exterior walls, blind bores, and accessible inner
                // walls, while ProfileLoop deliberately accepts only a whole
                // body's exterior loop for outside cutting.
                groups.push((selected.clone(), ExtractionRecipe::ProfileContours));
            }
            let mut all = Vec::new();
            for (references, recipe) in groups {
                let binding = resolve_selection_advisory(state, &references, recipe, 0.)?;
                let input = resolve_binding_advisory(state, &binding)?;
                merge_heights(&mut heights, input.heights)?;
                if input.profiles.is_empty() {
                    let side = match binding.recipe {
                        ExtractionRecipe::ProfileLoop { side } => side,
                        _ => Side::Outside,
                    };
                    all.extend(input.regions.into_iter().map(|r| ProfileContour {
                        chain: r.outer().clone(),
                        closed: true,
                        side,
                    }));
                } else {
                    all.extend(input.profiles);
                }
            }
            all
        }
    };
    if entry.kind != ProfileGeometryKind::Chain {
        contours.retain(|c| match entry.curves {
            ProfileCurves::All => true,
            ProfileCurves::Outer => c.side != Side::Inside,
            ProfileCurves::Inner => c.side == Side::Inside,
        });
    }
    for (index, contour) in contours.iter_mut().enumerate() {
        if entry.kind == ProfileGeometryKind::Chain {
            match entry.chain_mode {
                ProfileChainMode::Closed if !contour.closed => {
                    return Err("Complete the chain before selecting Closed mode".into())
                }
                ProfileChainMode::Open => contour.closed = false,
                _ => {}
            }
        }
        if entry.reversed {
            contour.chain.reverse();
        }
        contour.side = match entry.side_rule {
            ProfileSideRule::StartOutside => contour.side,
            ProfileSideRule::StartInside => match contour.side {
                Side::Outside => Side::Inside,
                Side::Inside => Side::Outside,
                Side::On => Side::On,
            },
            ProfileSideRule::AlwaysOutside => Side::Outside,
            ProfileSideRule::AlwaysInside => Side::Inside,
            ProfileSideRule::On => Side::On,
        };
        if let Some(side) = entry.side_overrides.get(&index) {
            contour.side = *side;
        }
        if contour.chain.len() < if contour.closed { 3 } else { 2 } {
            return Err("Select a valid open or closed contour".into());
        }
    }
    if contours.is_empty() {
        return Err(format!(
            "{}: no contours match the selected curve filter",
            entry.name
        ));
    }
    Ok((contours, heights))
}
fn merge_heights(result: &mut Option<CuttingHeights>, next: CuttingHeights) -> Result<(), String> {
    if let Some(previous) = result {
        previous.top = previous.top.max(next.top);
        previous.bottom = previous.bottom.min(next.bottom);
    } else {
        *result = Some(next);
    }
    Ok(())
}
pub(super) fn extract(
    state: &GeometryState,
    entries: &[ProfileGeometryEntry],
    stock_top: f64,
) -> Result<(Vec<ProfileContour>, CuttingHeights), String> {
    let (profiles, heights) = collect(state, entries)?;
    Ok((
        profiles,
        heights.unwrap_or(CuttingHeights {
            top: stock_top,
            bottom: stock_top - 1.,
        }),
    ))
}

fn collect(
    state: &GeometryState,
    entries: &[ProfileGeometryEntry],
) -> Result<(Vec<ProfileContour>, Option<CuttingHeights>), String> {
    if entries.is_empty() {
        return Err("Add at least one profile geometry entry".into());
    }
    let mut profiles = Vec::new();
    let mut heights = None;
    for entry in entries {
        let (contours, interval) = profile_entry_contours(state, entry)
            .map_err(|error| format!("Profile geometry entry '{}': {error}", entry.name))?;
        if let Some(interval) = interval {
            merge_heights(&mut heights, interval)?;
        }
        for contour in contours {
            profiles.push(contour);
        }
    }
    Ok((profiles, heights))
}
pub(super) fn resolve(
    state: &GeometryState,
    binding: &GeometryBinding,
    entries: &[ProfileGeometryEntry],
) -> Result<ResolvedInput, String> {
    let (profiles, certified) = collect(state, entries)?;
    let base = certified.unwrap_or(binding.heights);
    let heights = CuttingHeights {
        top: binding.overrides.top.unwrap_or(base.top),
        bottom: binding.overrides.bottom.unwrap_or(base.bottom),
    };
    heights.validate()?;
    let mut findings: Vec<_> = verify_selection(
        state,
        &binding.selected,
        binding.recipe.clone(),
        binding.heights.top,
    )
    .into_iter()
    .map(|reason| format!("Automatic geometry verification (info): {reason}"))
    .collect();
    if certified.is_some() && (heights.bottom < base.bottom - T || heights.top < base.top - T) {
        findings.push("Automatic geometry verification (info): operation heights extend outside the inferred feature interval; the operation's heights are used.".into());
    }
    let uses_projection = entries.iter().any(|entry| {
        entry.kind == ProfileGeometryKind::Silhouette
            && matches!(entry.source, ProfileGeometrySource::Cad(_))
    });
    let exact: Vec<_> = state
        .assets
        .iter()
        .map(|asset| {
            Ok((
                asset.id,
                asset.revision,
                &asset.backend,
                topology_fingerprint(&asset.topology, uses_projection)?,
            ))
        })
        .collect::<Result<_, String>>()?;
    let encoded = serde_json::to_vec(&(
        exact,
        &state.instances,
        entries,
        &binding.overrides,
        heights,
    ))
    .map_err(|e| e.to_string())?;
    let hash = encoded.into_iter().fold(0xcbf29ce484222325_u64, |h, b| {
        (h ^ b as u64).wrapping_mul(0x100000001b3)
    });
    let regions = profiles
        .iter()
        .filter(|p| p.closed)
        .map(|p| Polygon::new(p.chain.clone()).map_err(|e| e.to_string()))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(ResolvedInput {
        chamfers: Vec::new(),
        profiles,
        regions,
        holes: Vec::new(),
        heights,
        fingerprint: format!("{hash:016x}"),
        diagnostics: std::iter::once(if certified.is_none() {
            "Profile paths resolved from exact planar curves; operation heights define the cut depth without an adjoining wall certificate.".into()
        } else {
            "Profile geometry entries resolved from their individual sources and settings.".into()
        }).chain(findings).collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn entry() -> ProfileGeometryEntry {
        let square = |lo, hi| {
            Contour::new(vec![
                Point::new(lo, lo),
                Point::new(hi, lo),
                Point::new(hi, hi),
                Point::new(lo, hi),
            ])
        };
        let mut entry =
            ProfileGeometryEntry::new(ProfileGeometryKind::FaceContours, "Face boundaries".into());
        entry.source = ProfileGeometrySource::Contours(vec![
            ProfileContour {
                chain: square(0., 20.),
                closed: true,
                side: Side::Outside,
            },
            ProfileContour {
                chain: square(8., 12.),
                closed: true,
                side: Side::Inside,
            },
        ]);
        entry
    }
    #[test]
    fn curve_filters_and_side_rules_are_independent_and_overrides_are_per_contour() {
        let state = GeometryState::default();
        let mut e = entry();
        let (p, _) = profile_entry_contours(&state, &e).unwrap();
        assert_eq!(
            p.iter().map(|p| p.side).collect::<Vec<_>>(),
            vec![Side::Outside, Side::Inside]
        );
        e.side_rule = ProfileSideRule::StartInside;
        let (p, _) = profile_entry_contours(&state, &e).unwrap();
        assert_eq!(
            p.iter().map(|p| p.side).collect::<Vec<_>>(),
            vec![Side::Inside, Side::Outside]
        );
        e.side_rule = ProfileSideRule::AlwaysOutside;
        e.side_overrides.insert(1, Side::On);
        let (p, _) = profile_entry_contours(&state, &e).unwrap();
        assert_eq!(p[0].side, Side::Outside);
        assert_eq!(p[1].side, Side::On);
        e.side_overrides.clear();
        e.curves = ProfileCurves::Inner;
        let (p, _) = profile_entry_contours(&state, &e).unwrap();
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].chain.area(), 16.);
        assert_eq!(p[0].side, Side::Outside);
        e.curves = ProfileCurves::Outer;
        let (p, _) = profile_entry_contours(&state, &e).unwrap();
        assert_eq!(p[0].chain.area(), 400.);
    }
    #[test]
    fn chain_mode_and_reverse_do_not_add_an_implicit_closing_cut() {
        let state = GeometryState::default();
        let mut e = entry();
        e.kind = ProfileGeometryKind::Chain;
        e.chain_mode = ProfileChainMode::Open;
        e.reversed = true;
        let (p, _) = profile_entry_contours(&state, &e).unwrap();
        assert!(p.iter().all(|p| !p.closed));
        assert_eq!(p[0].chain.points()[0], Point::new(0., 20.));
        e.source = ProfileGeometrySource::Contours(vec![ProfileContour {
            chain: Contour::new(vec![Point::new(0., 0.), Point::new(10., 0.)]),
            closed: false,
            side: Side::Outside,
        }]);
        e.chain_mode = ProfileChainMode::Closed;
        assert!(profile_entry_contours(&state, &e)
            .unwrap_err()
            .contains("Complete"));
    }
}
