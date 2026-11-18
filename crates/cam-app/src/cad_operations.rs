//! CAD import preparation and the boundary between topology resolution and the
//! existing planar strategies. No source tessellation is used to create a cut.
use crate::{JobParams, OpKind};
use cam_model::*;
use std::path::Path;

#[derive(Clone, Debug)]
pub struct CadImportPreview {
    pub name: String,
    pub items: usize,
    pub source_units: SourceUnits,
    pub dimensions_mm: [f64; 3],
    pub diagnostics: Vec<String>,
}
pub(crate) enum PreparedImport {
    Drawing {
        import: cam_import::Import,
        name: String,
    },
    Solid {
        asset: GeometryAsset,
        instances: Vec<GeometryInstance>,
    },
}
impl PreparedImport {
    pub fn preview(&self) -> CadImportPreview {
        match self {
            Self::Drawing { import, name } => {
                let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
                for p in import
                    .regions
                    .iter()
                    .flat_map(|r| r.outer().points())
                    .chain(import.open_chains.iter().flat_map(|p| p.points()))
                {
                    for (axis, value) in [p.x, p.y, 0.0].into_iter().enumerate() {
                        bounds[0][axis] = bounds[0][axis].min(value);
                        bounds[1][axis] = bounds[1][axis].max(value);
                    }
                }
                CadImportPreview {
                    name: name.clone(),
                    items: import.regions.len() + import.open_chains.len(),
                    source_units: SourceUnits::Millimetres,
                    dimensions_mm: std::array::from_fn(|i| (bounds[1][i] - bounds[0][i]).max(0.0)),
                    diagnostics: import.warnings.clone(),
                }
            }
            Self::Solid { asset, instances } => {
                let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
                for instance in instances {
                    if let Some(body) = asset
                        .topology
                        .bodies
                        .iter()
                        .find(|b| b.id == instance.body_id)
                    {
                        for x in [body.bounds[0][0], body.bounds[1][0]] {
                            for y in [body.bounds[0][1], body.bounds[1][1]] {
                                for z in [body.bounds[0][2], body.bounds[1][2]] {
                                    let p = instance.placement.transform_point([x, y, z]);
                                    for i in 0..3 {
                                        bounds[0][i] = bounds[0][i].min(p[i]);
                                        bounds[1][i] = bounds[1][i].max(p[i]);
                                    }
                                }
                            }
                        }
                    }
                }
                CadImportPreview {
                    name: asset.name.clone(),
                    items: instances.len(),
                    source_units: asset.source_units,
                    dimensions_mm: std::array::from_fn(|i| (bounds[1][i] - bounds[0][i]).max(0.0)),
                    diagnostics: asset.import_diagnostics.clone(),
                }
            }
        }
    }
}
pub(crate) fn prepare_import(path: &Path) -> Result<PreparedImport, String> {
    let name = path
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("Imported model")
        .to_owned();
    // Snapshot source bytes once so provenance cannot race a CAD re-export.
    let source_data = std::fs::read(path).map_err(|e| e.to_string())?;
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|e| e.to_string())?
        .as_nanos();
    let extension = path.extension().and_then(|e| e.to_str()).unwrap_or("");
    static NEXT_SNAPSHOT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let serial = NEXT_SNAPSHOT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let temporary = std::env::temp_dir().join(format!(
        "ocam-import-{}-{stamp}-{serial}.{extension}",
        std::process::id()
    ));
    struct SourceSnapshot(std::path::PathBuf);
    impl Drop for SourceSnapshot {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }
    use std::io::Write;
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&temporary)
        .map_err(|e| e.to_string())?;
    let snapshot = SourceSnapshot(temporary);
    file.write_all(&source_data).map_err(|e| e.to_string())?;
    drop(file);
    match cam_import::read_geometry_file(&snapshot.0, &Default::default(), &Default::default())
        .map_err(|e| e.to_string())?
    {
        cam_import::GeometryImport::Drawing(import) => Ok(PreparedImport::Drawing { import, name }),
        cam_import::GeometryImport::Solid(import) => {
            let import = *import;
            let format = match path
                .extension()
                .and_then(|e| e.to_str())
                .unwrap_or("")
                .to_ascii_lowercase()
                .as_str()
            {
                "iges" | "igs" => CadFormat::Iges,
                _ => CadFormat::Step,
            };
            let instances = import
                .placements
                .into_iter()
                .enumerate()
                .map(|(index, p)| GeometryInstance {
                    id: index as u64 + 1,
                    asset_id: 1,
                    body_id: p.body_id,
                    name: format!("Body {}", index + 1),
                    placement: p.placement,
                    visible: true,
                })
                .collect();
            let options = cam_kernel::SolidImportOptions::default();
            let asset = GeometryAsset {
                import_options: Some(GeometryImportOptions {
                    import_tolerance: options.import_tolerance,
                    display_tolerance: options.display_tolerance,
                }),
                import_diagnostics: import.warnings,
                source_fingerprint: GeometryAsset::fingerprint_source(&source_data),
                id: 1,
                revision: 1,
                name,
                source_format: format,
                source_data: source_data.into(),
                source_units: import.source_units,
                backend: import.backend,
                topology: import.topology.into(),
            };
            Ok(PreparedImport::Solid { asset, instances })
        }
    }
}
pub(crate) fn recipe(
    kind: OpKind,
    geometry: &GeometryState,
    selected: &[EntityRef],
) -> Result<ExtractionRecipe, String> {
    Ok(match kind {
        OpKind::Pocket
            if selected
                .first()
                .is_some_and(|r| matches!(r.kind, EntityKind::Loop | EntityKind::Edge)) =>
        {
            ExtractionRecipe::PocketBoundary
        }
        OpKind::Pocket => ExtractionRecipe::PocketFloor,
        OpKind::Drill => ExtractionRecipe::DrillHoles,
        OpKind::Chamfer if selected.first().is_some_and(|r| r.kind == EntityKind::Face) => {
            ExtractionRecipe::ChamferFaces
        }
        OpKind::Chamfer => ExtractionRecipe::ChamferLoop,
        OpKind::Engrave => ExtractionRecipe::EngravePath,
        OpKind::Carve => ExtractionRecipe::CarveRegion,
        OpKind::Thread | OpKind::Bore => {
            let reference = selected
                .first()
                .ok_or("Select cylindrical bore or boss faces")?;
            let face = geometry
                .asset(reference.asset_id)
                .and_then(|a| a.topology.faces.iter().find(|f| f.id == reference.id))
                .filter(|_| reference.kind == EntityKind::Face)
                .ok_or("Thread/Bore requires cylindrical bore or boss faces")?;
            let CadSurface::Cylinder { inward, .. } = face.surface else {
                return Err(format!(
                    "Thread/Bore requires cylindrical bore or boss faces; selected face: {}",
                    face.surface.kind_name()
                ));
            };
            if kind == OpKind::Bore {
                ExtractionRecipe::BoreCylinders { internal: inward }
            } else {
                ExtractionRecipe::ThreadCylinders { internal: inward }
            }
        }
        OpKind::Profile if selected.len() == 1 && selected[0].kind == EntityKind::Silhouette => {
            ExtractionRecipe::ProfileSilhouette
        }
        OpKind::Profile
            if selected.first().is_some_and(|first| {
                selected.iter().any(|reference| {
                    reference.kind != first.kind
                        || reference.instance_id != first.instance_id
                        || reference.asset_id != first.asset_id
                        || reference.body_id != first.body_id
                })
            }) =>
        {
            ExtractionRecipe::ProfileEntries {
                entries: profile_entries_from_selection(selected),
            }
        }
        OpKind::Profile => ExtractionRecipe::ProfileContours,
        OpKind::Face => ExtractionRecipe::FaceRegion,
        OpKind::Parallel => ExtractionRecipe::ParallelFinishing,
        OpKind::Spiral => ExtractionRecipe::SpiralFinishing,
        OpKind::Adaptive => ExtractionRecipe::AdaptiveClearing,
    })
}

pub(crate) fn profile_entries_from_selection(selected: &[EntityRef]) -> Vec<ProfileGeometryEntry> {
    let mut entries: Vec<ProfileGeometryEntry> = Vec::new();
    for reference in selected {
        if matches!(reference.kind, EntityKind::Edge | EntityKind::Face) {
            if let Some(entry) = entries.iter_mut().find(|entry|
                matches!(&entry.source, ProfileGeometrySource::Cad(refs) if refs.first().is_some_and(|first|
                    first.kind == reference.kind && first.asset_id == reference.asset_id
                        && first.instance_id == reference.instance_id && first.body_id == reference.body_id))) {
                let ProfileGeometrySource::Cad(refs) = &mut entry.source else { unreachable!() };
                refs.push(reference.clone());
                continue;
            }
        }
        let (kind, name) = match reference.kind {
            EntityKind::Face => (ProfileGeometryKind::FaceContours, "Face Contours"),
            EntityKind::Body | EntityKind::Silhouette => {
                (ProfileGeometryKind::Silhouette, "Silhouette")
            }
            _ => (ProfileGeometryKind::Chain, "Chain"),
        };
        let mut entry = ProfileGeometryEntry::new(kind, format!("{name} {}", entries.len() + 1));
        if reference.kind == EntityKind::Loop {
            entry.chain_mode = ProfileChainMode::Closed;
        }
        entry.use_setup_models = false;
        entry.source = ProfileGeometrySource::Cad(vec![reference.clone()]);
        entries.push(entry);
    }
    entries
}

/// Source topology stays immutable. Only instance placements are rotated into
/// the setup axes, preserving semantic references and approximation budgets.
pub(crate) fn geometry_for_setup(document: &Document) -> Result<GeometryState, String> {
    let setup = document
        .active_setup()
        .ok_or("Create a setup before evaluating machining geometry.")?;
    let mut geometry = document.geometry.geometry_in_setup(&setup.frame)?;
    let ids: std::collections::BTreeSet<_> = setup.operations.iter().map(Operation::id).collect();
    geometry.bindings.retain(|id, _| ids.contains(id));
    geometry
        .instances
        .retain(|instance| setup.model_instances.contains(&instance.id));
    Ok(geometry)
}

/// Conservative model bounds from exact source body bounds; display mesh quality
/// cannot alter stock dimensions or NC. Evaluate each body in setup axes.
pub(crate) fn model_bounds(geometry: &GeometryState, setup: &Setup) -> ([f64; 3], [f64; 3]) {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    let mut include = |p: [f64; 3]| {
        for axis in 0..3 {
            min[axis] = min[axis].min(p[axis]);
            max[axis] = max[axis].max(p[axis]);
        }
    };
    for instance in &geometry.instances {
        if !setup.model_instances.contains(&instance.id) {
            continue;
        }
        let Some(asset) = geometry.asset(instance.asset_id) else {
            continue;
        };
        let Some(body) = asset
            .topology
            .bodies
            .iter()
            .find(|b| b.id == instance.body_id)
        else {
            continue;
        };
        for x in [body.bounds[0][0], body.bounds[1][0]] {
            for y in [body.bounds[0][1], body.bounds[1][1]] {
                for z in [body.bounds[0][2], body.bounds[1][2]] {
                    include(
                        setup
                            .frame
                            .to_setup_vector(instance.placement.transform_point([x, y, z])),
                    );
                }
            }
        }
    }
    for p in geometry
        .drawing_regions
        .iter()
        .flat_map(|r| r.outer().points())
        .chain(geometry.drawing_paths.iter().flat_map(|p| p.points()))
    {
        include(setup.frame.to_setup_vector([p.x, p.y, 0.]));
    }
    if min[0].is_infinite() {
        ([0.; 3], [0.; 3])
    } else {
        (min, max)
    }
}

fn stock_top(document: &Document) -> f64 {
    document.stock_bounds_for(document.setup()).1[2]
}
fn operation_stock_top(document: &Document, id: u32) -> Result<f64, String> {
    Ok(document
        .indexed_view(id)?
        .as_ref()
        .map_or_else(|| stock_top(document), stock_top))
}
/// Validate the complete selection before strategy extraction. A mixed geometry
/// field cannot become a valid operation by dropping its incompatible members.
pub(crate) fn validate_selection(
    geometry: &GeometryState,
    selected: &[EntityRef],
) -> Result<(), String> {
    if selected.iter().any(|r| !geometry.entity_exists(r)) {
        return Err("Needs reselection: missing entity or stale asset revision".into());
    }
    if selected
        .iter()
        .enumerate()
        .any(|(i, r)| selected[..i].contains(r))
    {
        return Err("The same geometry feature was selected more than once".into());
    }
    Ok(())
}

pub(crate) fn operation_type(operation: &Operation) -> &'static str {
    match operation {
        Operation::Profile(_) => "Profile",
        Operation::Pocket(_) => "Pocket",
        Operation::Drill(_) => "Drill",
        Operation::Face(_) => "Face",
        Operation::Chamfer(_) => "Chamfer",
        Operation::Bore(_) => "Bore",
        Operation::Thread(_) => "Thread",
        Operation::Engrave(_) => "Engrave",
        Operation::Carve(_) => "Carve",
        Operation::Adaptive(_) => "3D Adaptive Clearing",
        Operation::Parallel(o) if o.params.scallop => "3D Scallop",
        Operation::Parallel(_) => "3D Parallel",
        Operation::Spiral(_) => "3D Spiral",
    }
}

/// Stable feature labels used in selection errors and run diagnostics.
pub(crate) fn describe_entities(geometry: &GeometryState, selected: &[EntityRef]) -> String {
    selected
        .iter()
        .map(|r| {
            let asset = geometry
                .asset(r.asset_id)
                .map_or("missing asset", |a| a.name.as_str());
            let instance = geometry
                .instance(r.instance_id)
                .map_or("missing instance", |i| i.name.as_str());
            format!(
                "'{}' / '{}' (instance {}, body {}, revision {}): {:?} {}",
                asset, instance, r.instance_id, r.body_id, r.revision, r.kind, r.id
            )
        })
        .collect::<Vec<_>>()
        .join("; ")
}

/// Rebuild the machining cache from independently configured profile entries.
pub(crate) fn resolve_profile_entries(
    document: &Document,
    profile: &mut ProfileOp,
) -> Result<Option<GeometryBinding>, String> {
    let geometry = geometry_for_operation(document, profile.id)?;
    let entries = profile
        .selections
        .as_mut()
        .ok_or("Profile entries are unavailable")?;
    let mut certified_heights = false;
    for entry in entries.iter_mut() {
        let (contours, heights) = cam_geometry3d::profile_entry_contours(&geometry, entry)
            .map_err(|error| {
                format!(
                    "Profile operation #{} / geometry entry '{}': {error}",
                    profile.id, entry.name
                )
            })?;
        entry.contour_count = contours.len();
        certified_heights |= heights.is_some();
    }
    let mut selected = Vec::new();
    for entry in entries.iter() {
        if let ProfileGeometrySource::Cad(refs) = &entry.source {
            for reference in refs {
                if !selected.contains(reference) {
                    selected.push(reference.clone());
                }
            }
        }
    }
    let mut binding = cam_geometry3d::resolve_selection_advisory(
        &geometry,
        &selected,
        ExtractionRecipe::ProfileEntries {
            entries: entries.clone(),
        },
        operation_stock_top(document, profile.id)?,
    )?;
    if !certified_heights {
        // Open paths use the operator's depth, rather than a default wall
        // interval. Reference height settings remain authoritative at planning.
        binding.heights.bottom = binding.heights.top - profile.depth;
        binding.heights.validate()?;
    }
    let input = cam_geometry3d::resolve_binding_advisory(&geometry, &binding)?;
    let mut contours = input.profiles;
    let first = contours.remove(0);
    profile.chain = first.chain;
    profile.closed = first.closed;
    profile.side = first.side;
    profile.additional_chains = contours;
    if selected.is_empty() {
        Ok(None)
    } else {
        profile.depth = input.heights.depth();
        Ok(Some(binding))
    }
}

pub(crate) fn create_operation(
    document: &Document,
    kind: OpKind,
    selected: Vec<EntityRef>,
    id: u32,
    tool: u32,
    work_offset: u32,
    defaults: JobParams,
) -> Result<(Operation, GeometryBinding), String> {
    let geometry = geometry_for_operation(document, id)?;
    if matches!(kind, OpKind::Adaptive | OpKind::Parallel | OpKind::Spiral) {
        cam_geometry3d::validate_adaptive_selection(&geometry, &selected)?;
    } else {
        validate_selection(&geometry, &selected)?;
    }
    let mut binding = cam_geometry3d::resolve_selection_advisory(
        &geometry,
        &selected,
        recipe(kind, &geometry, &selected)?,
        operation_stock_top(document, id)?,
    )?;
    let resolved = cam_geometry3d::resolve_binding_advisory(&geometry, &binding)?;
    let depth = if binding.recipe == ExtractionRecipe::PocketBoundary {
        defaults.depth
    } else {
        resolved.heights.depth()
    };
    let boundary = || {
        resolved
            .regions
            .first()
            .map(|r| r.outer().clone())
            .ok_or_else(|| "No resolved boundary".to_string())
    };
    let op = match kind {
        OpKind::Pocket => Operation::Pocket(PocketOp {
            parameter_expressions: Default::default(),
            wrap: None,
            additional_regions: resolved.regions.iter().skip(1).cloned().collect(),
            stock_contours: Default::default(),
            finishing: Default::default(),
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            boundary: boundary()?,
            islands: resolved.regions[0].holes().to_vec(),
            depth,
            end: None,
            start: None,
            clear: ClearParams {
                multiple_depths: Default::default(),
                adaptive_enabled: None,
                pocket_overlap: None,
                keep_tool_down: false,
                lift_height: 0.2,
                stepdown: defaults.stepdown,
                overlap: 0.5,
                offset: 0.0,
                feed: defaults.feed,
                plunge_feed: defaults.plunge_feed,
                plunge: Plunge::Straight,
                lead_overlap: 0.0,
                lead_in: Lead::Arc { radius: 0.6 },
                lead_out: Lead::Arc { radius: 0.6 },
                clearing: Clearing::default(),
            },
        }),
        OpKind::Profile => Operation::Profile(ProfileOp {
            parameter_expressions: Default::default(),
            wrap: None,
            passes: cam_model::ProfilePasses {
                lead_end_distance: 0.6,
                lead_end_distance_enabled: Some(false),
                ..Default::default()
            },
            selections: match &binding.recipe {
                ExtractionRecipe::ProfileEntries { entries } => Some(entries.clone()),
                _ => None,
            },
            closed: resolved.profiles.first().is_none_or(|p| p.closed),
            additional_chains: resolved.profiles.iter().skip(1).cloned().collect(),
            tabs: Default::default(),
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            chain: resolved
                .profiles
                .first()
                .map(|p| p.chain.clone())
                .map(Ok)
                .unwrap_or_else(boundary)?,
            side: match binding.recipe {
                ExtractionRecipe::ProfileLoop { side } => side,
                ExtractionRecipe::ProfileContours
                | ExtractionRecipe::ProfileSilhouette
                | ExtractionRecipe::ProfileEntries { .. } => resolved.profiles[0].side,
                _ => unreachable!("Profile uses a profile extraction recipe"),
            },
            comp: Comp::Computed,
            offset: 0.0,
            depth,
            stepdown: defaults.stepdown,
            stepover: 0.0,
            feed: defaults.feed,
            plunge_feed: defaults.plunge_feed,
            end: None,
            start: None,
            lead_in: Lead::Arc { radius: 0.6 },
            lead_out: Lead::Arc { radius: 0.6 },
            lead_overlap: if resolved.profiles.first().is_none_or(|p| p.closed) {
                0.6
            } else {
                0.0
            },
            plunge: Plunge::Straight,
            clearing: Clearing::default(),
        }),
        OpKind::Drill => Operation::Drill(DrillOp {
            parameter_expressions: Default::default(),
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            points: resolved
                .holes
                .iter()
                .map(|h| [h.center.x, h.center.y])
                .collect(),
            depth,
            start_offset: resolved.heights.top - document.setup().heights.top_of_stock,
            peck: None,
            dwell: None,
            feed: defaults.plunge_feed,
        }),
        OpKind::Face => Operation::Face(FaceOp {
            absolute_overlap: None,
            multiple_depths: Default::default(),
            parameter_expressions: Default::default(),
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            boundary: boundary()?,
            start_offset: resolved.heights.top,
            depth,
            stepdown: defaults.stepdown,
            overlap: 0.5,
            overshoot: 0.0,
            direction: Axis::X,
            feed: defaults.feed,
            plunge_feed: defaults.plunge_feed,
        }),
        OpKind::Chamfer => Operation::Chamfer(ChamferOp {
            parameter_expressions: Default::default(),
            contour_roughing: Default::default(),
            tip_offset: 1.0,
            lead_in_enabled: Some(true),
            same_lead_out: false,
            lead_out_enabled: Some(true),
            closed: resolved.profiles.first().is_none_or(|p| p.closed),
            faces: resolved.chamfers.clone(),
            stepdown: 0.,
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            chain: if resolved.chamfers.is_empty() {
                resolved.profiles[0].chain.clone()
            } else {
                boundary()?
            },
            side: resolved.profiles.first().map_or(Side::Outside, |p| p.side),
            width: resolved.chamfers.first().map_or(1., ChamferFace::width),
            top: resolved.heights.top,
            depth: if resolved.chamfers.is_empty() {
                0.
            } else {
                depth
            },
            step: 0.,
            gradual: false,
            feed: defaults.feed,
            plunge_feed: defaults.plunge_feed,
            end: None,
            start: None,
            lead_in: Lead::Arc { radius: 0.6 },
            lead_out: Lead::Arc { radius: 0.6 },
            lead_overlap: if resolved.chamfers.is_empty()
                && resolved.profiles.first().is_none_or(|p| p.closed)
            {
                0.6
            } else {
                0.0
            },
        }),
        OpKind::Engrave => Operation::Engrave(EngraveOp {
            parameter_expressions: Default::default(),
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            chain: resolved.profiles[0].chain.clone(),
            closed: resolved.profiles[0].closed,
            top: resolved.heights.top,
            depth: 0.3,
            stepdown: 0.,
            feed: defaults.feed,
            plunge_feed: defaults.plunge_feed,
            end: None,
            start: None,
        }),
        OpKind::Carve => Operation::Carve(CarveOp {
            parameter_expressions: Default::default(),
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            boundary: boundary()?,
            islands: resolved.regions[0].holes().to_vec(),
            top: resolved.heights.top,
            depth: 1.,
            clear: None,
            offset: 0.,
            ring_step: 0.,
            scallop: 0.,
            feed: defaults.feed,
            plunge_feed: defaults.plunge_feed,
            end: None,
            start: None,
            plunge: Plunge::Straight,
            stay_down: true,
        }),
        OpKind::Bore => Operation::Bore(BoreOp {
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            points: resolved
                .holes
                .iter()
                .map(|h| [h.center.x, h.center.y])
                .collect(),
            internal: matches!(
                binding.recipe,
                ExtractionRecipe::BoreCylinders { internal: true }
            ),
            diameter: resolved.holes[0].diameter,
            z_top: resolved.heights.top,
            z_bottom: resolved.heights.bottom,
            feed: defaults.feed,
            plunge_feed: defaults.plunge_feed,
            ..Default::default()
        }),
        OpKind::Thread => Operation::Thread(ThreadOp {
            parameter_expressions: Default::default(),
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            points: resolved
                .holes
                .iter()
                .map(|h| [h.center.x, h.center.y])
                .collect(),
            internal: matches!(
                binding.recipe,
                ExtractionRecipe::ThreadCylinders { internal: true }
            ),
            major_dia: resolved.holes[0].diameter,
            pitch: 1.,
            hand: Hand::Right,
            z_top: resolved.heights.top,
            z_bottom: resolved.heights.bottom,
            climb: true,
            passes: 1,
            gradual: false,
            spring_passes: 0,
            drill_clearance: 0.,
            blind_allowance: 0.,
            feed: defaults.feed,
            plunge_feed: defaults.plunge_feed,
        }),
        OpKind::Adaptive => Operation::Adaptive(AdaptiveOp {
            parameter_expressions: Default::default(),
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            params: AdaptiveParams {
                feed: defaults.feed,
                plunge_feed: defaults.plunge_feed,
                ..Default::default()
            },
        }),
        OpKind::Parallel => Operation::Parallel(ParallelOp {
            parameter_expressions: Default::default(),
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            params: ParallelParams {
                feed: defaults.feed,
                plunge_feed: defaults.plunge_feed,
                ..Default::default()
            },
        }),
        OpKind::Spiral => Operation::Spiral(cam_model::SpiralOp {
            parameter_expressions: Default::default(),
            id,
            tool,
            work_offset,
            spindle_rpm: defaults.spindle_rpm,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            params: cam_model::SpiralParams {
                feed: defaults.feed,
                plunge_feed: defaults.plunge_feed,
                ..Default::default()
            },
        }),
    };
    if binding.recipe == ExtractionRecipe::PocketBoundary {
        binding.heights.bottom = binding.heights.top - depth;
    } else if surface_recipe(&binding.recipe) {
        binding.heights = document.operation_base_cutting_heights(&op);
    }
    Ok((op, binding))
}

/// A fresh topology resolution replaces every linked strategy input. Stale
/// embedded contours are never a fallback after asset replacement or removal.
pub(crate) fn resolve_document(document: &Document) -> Result<Document, String> {
    resolve_document_checked(document, &|| false).map_err(|diagnostic| diagnostic.message)
}

pub(crate) fn geometry_for_operation(
    document: &Document,
    id: u32,
) -> Result<GeometryState, String> {
    let geometry = geometry_for_setup(document)?;
    if let Some(index) = document.rotary_indexes.get(&id) {
        index.validate()?;
        Ok(index.geometry(&geometry))
    } else {
        Ok(geometry)
    }
}

pub(crate) fn resolve_wrapped_operation(
    geometry: &GeometryState,
    operation: &mut Operation,
) -> Result<(), String> {
    let Some(mut wrap) = operation.rotary_wrap().cloned() else {
        return Ok(());
    };
    wrap.validate()?;
    if let Some(cylinder) = wrap.cylinder.clone() {
        wrap = cam_geometry3d::wrap_cylinder(geometry, &cylinder, wrap)?;
    }
    wrap.boundary_z = if wrap.unwrap_geometry {
        cam_geometry3d::wrapped_boundary_height(geometry, &wrap)?
    } else {
        None
    };
    wrap.periodic =
        wrap.unwrap_geometry && cam_geometry3d::periodic_cylinder_selection(geometry, &wrap)?;
    if !wrap.geometry.is_empty() {
        let contours = if wrap.unwrap_geometry {
            cam_geometry3d::unwrap_boundaries(geometry, &wrap)?
        } else {
            let recipe = if matches!(operation, Operation::Pocket(_)) {
                ExtractionRecipe::PocketBoundary
            } else {
                ExtractionRecipe::ProfileContours
            };
            let binding = cam_geometry3d::resolve_selection_advisory(
                geometry,
                &wrap.geometry,
                recipe,
                wrap.surface_z,
            )?;
            let input = cam_geometry3d::resolve_binding_advisory(geometry, &binding)?;
            if !input.profiles.is_empty() {
                input.profiles
            } else {
                input
                    .regions
                    .iter()
                    .flat_map(|r| std::iter::once(r.outer()).chain(r.holes().iter()))
                    .map(|chain| ProfileContour {
                        chain: chain.clone(),
                        closed: true,
                        side: Side::On,
                    })
                    .collect()
            }
        };
        match operation {
            Operation::Profile(profile) => {
                let mut contours = contours.into_iter();
                let first = contours.next().ok_or("Empty wrapped contour")?;
                profile.chain = first.chain;
                profile.closed = first.closed;
                profile.additional_chains = contours.collect();
                profile.selections = None;
            }
            Operation::Pocket(pocket) => {
                let mut regions = cam_geometry3d::wrapped_regions(&contours)?.into_iter();
                let first = regions.next().ok_or("Empty wrapped pocket")?;
                pocket.boundary = first.outer().clone();
                pocket.islands = first.holes().to_vec();
                pocket.additional_regions = regions.collect();
            }
            _ => return Err("Rotary wrapping supports Profile and Pocket operations".into()),
        }
    }
    if matches!(operation,Operation::Profile(profile) if profile.comp!=Comp::Computed) {
        return Err("Wrapped contours require computed cutter compensation".into());
    }
    if matches!(operation,Operation::Pocket(pocket) if pocket.stock_contours.enabled) {
        return Err("Disable Stock Contours for a wrapped pocket; define its boundary in the unwrapped plane".into());
    }
    operation.set_rotary_wrap(Some(wrap));
    Ok(())
}

/// Typed source ownership is preserved when resolution runs in a worker.
pub(crate) fn resolve_document_checked(
    document: &Document,
    cancelled: &dyn Fn() -> bool,
) -> Result<Document, cam_toolpath::Diagnostic> {
    let error = |message: String| {
        cam_toolpath::Diagnostic::error(message)
            .with_code(cam_toolpath::DiagnosticCode::GeometryUnresolved)
            .for_setup(document.setup().id)
    };
    if cancelled() {
        return Err(error("Geometry resolution cancelled".into())
            .with_code(cam_toolpath::DiagnosticCode::Cancelled));
    }
    document.geometry.validate().map_err(error)?;
    let mut resolved = document.clone();
    let tools = resolved.setup().tools.clone();
    for operation in &mut resolved.setup_mut().operations {
        if document.operation_is_included(document.setup().id, operation.id()) {
            operation
                .resolve_parameter_expressions(&tools)
                .map_err(|reason| error(reason).for_op(operation.id()))?;
        }
    }
    let prepared_operations = resolved.setup().operations.clone();
    // Resolve all reference geometry in its source placement, before rotating
    // instances for the strategy adapters. Keep a worker-only absolute snapshot.
    for operation in &prepared_operations {
        if !document.operation_is_included(document.setup().id, operation.id()) {
            continue;
        }
        if operation.rotary_wrap().is_some()
            || document.rotary_indexes.contains_key(&operation.id())
        {
            continue;
        }
        if let Some(settings) = document.setup().height_settings.get(&operation.id()) {
            let heights = settings.resolve(document, operation).map_err(|reason| {
                error(format!("Operation {}: {reason}", operation.id())).for_op(operation.id())
            })?;
            resolved
                .setup_mut()
                .height_settings
                .get_mut(&operation.id())
                .unwrap()
                .resolved = Some(heights);
        }
    }
    resolved.geometry = geometry_for_setup(document).map_err(error)?;
    let mut operations = std::mem::take(&mut resolved.setup_mut().operations);
    for operation in &mut operations {
        if !document.operation_is_included(document.setup().id, operation.id()) {
            continue;
        }
        if cancelled() {
            return Err(error("Geometry resolution cancelled".into())
                .with_code(cam_toolpath::DiagnosticCode::Cancelled));
        }
        let id = operation.id();
        let base_instances = resolved.geometry.instances.clone();
        if let Some(index) = document.rotary_indexes.get(&id) {
            index.validate().map_err(error)?;
            resolved.geometry = index.geometry(&resolved.geometry);
        }
        let resolved_operation: Result<(), String> = (|| {
            operation.resolve_parameter_expressions(&resolved.setup().tools)?;
            if operation.rotary_wrap().is_some() {
                resolve_wrapped_operation(&resolved.geometry, operation)?;
                resolved.geometry.bindings.remove(&id);
                resolved.cad_linked_operations.remove(&id);
                if let Some(settings) = resolved.setup().height_settings.get(&id).cloned() {
                    let mut settings = settings;
                    settings.resolved = None;
                    // The wrap now carries its developed CAD floor, but stock
                    // references still belong to the original world geometry.
                    // Resolving against the rotated worker geometry would apply
                    // the setup frame twice when measuring the round blank.
                    let heights = settings.resolve(document, operation)?;
                    resolved
                        .setup_mut()
                        .height_settings
                        .get_mut(&id)
                        .unwrap()
                        .resolved = Some(heights);
                }
                return Ok(());
            }
            let Some(binding) = document.geometry.bindings.get(&id) else {
                if document.cad_linked_operations.contains(&id) {
                    return Err(format!(
                        "Operation {id}: linked CAD geometry is missing; reselect its geometry"
                    ));
                }
                return Ok(());
            };
            if let ResolutionStatus::NeedsReselection { reason } = &binding.status {
                if !reason.starts_with("Geometry changed:") {
                    return Err(format!("Operation {id}: Needs reselection — {reason}"));
                }
            }
            if matches!(
                binding.recipe,
                ExtractionRecipe::AdaptiveClearing
                    | ExtractionRecipe::ParallelFinishing
                    | ExtractionRecipe::SpiralFinishing
            ) {
                cam_geometry3d::validate_adaptive_selection(&resolved.geometry, &binding.selected)
                    .map_err(|e| format!("Operation {id}: {e}"))?;
            } else if !matches!(binding.recipe, ExtractionRecipe::ProfileEntries { .. }) {
                validate_selection(&resolved.geometry, &binding.selected)
                    .map_err(|e| format!("Operation {id}: {e}"))?;
            }
            let mut current_binding = binding.clone();
            if surface_recipe(&binding.recipe) {
                let cutting = document.operation_base_cutting_heights(operation);
                current_binding.heights.bottom = current_binding.heights.top - cutting.depth();
            }
            if matches!(current_binding.recipe, ExtractionRecipe::FaceRegion) {
                current_binding.heights.top = operation_stock_top(document, id)?;
            }
            let input =
                cam_geometry3d::resolve_binding_advisory(&resolved.geometry, &current_binding)
                    .map_err(|e| {
                        format!(
                            "{} operation #{id} / setup '{}' / {}: {e}",
                            operation_type(operation),
                            document.setup().name,
                            describe_entities(&resolved.geometry, &current_binding.selected)
                        )
                    })?;
            let depth = input.heights.depth();
            let region = || input.regions.first().ok_or("No resolved machining region");
            match (&mut *operation, &binding.recipe) {
                (
                    Operation::Pocket(p),
                    ExtractionRecipe::PocketFloor | ExtractionRecipe::PocketBoundary,
                ) => {
                    p.boundary = region()?.outer().clone();
                    p.islands = region()?.holes().to_vec();
                    p.additional_regions = input.regions.iter().skip(1).cloned().collect();
                    p.depth = depth;
                }
                (Operation::Profile(p), ExtractionRecipe::ProfileEntries { entries }) => {
                    p.validate_profile_source(true)?;
                    let mut contours = input.profiles.clone();
                    let first = contours.remove(0);
                    p.chain = first.chain;
                    p.closed = first.closed;
                    p.side = first.side;
                    p.additional_chains = contours;
                    p.selections = Some(entries.clone());
                    p.depth = depth;
                }
                (
                    Operation::Profile(p),
                    ExtractionRecipe::ProfileContours | ExtractionRecipe::ProfileSilhouette,
                ) => {
                    let first = input.profiles.first().ok_or("No resolved profile chains")?;
                    p.validate_profile_source(true)?;
                    p.chain = first.chain.clone();
                    p.closed = first.closed;
                    let sides: Vec<_> = p
                        .additional_chains
                        .iter()
                        .map(|contour| contour.side)
                        .collect();
                    p.additional_chains = input
                        .profiles
                        .iter()
                        .skip(1)
                        .cloned()
                        .enumerate()
                        .map(|(index, mut contour)| {
                            if let Some(side) = sides.get(index) {
                                contour.side = *side;
                            }
                            contour
                        })
                        .collect();
                    p.depth = depth;
                }
                (Operation::Profile(p), ExtractionRecipe::ProfileLoop { .. }) => {
                    // The recipe certifies the source geometry and cutting interval.
                    // Cutter side is an independent, per-contour operation setting.
                    p.validate_profile_source(true)?;
                    p.chain = region()?.outer().clone();
                    p.closed = true;
                    p.additional_chains.clear();
                    p.depth = depth;
                }
                (Operation::Chamfer(p), ExtractionRecipe::ChamferFaces) => {
                    p.faces = input.chamfers.clone();
                    p.chain = region()?.outer().clone();
                    p.width = p.faces.first().ok_or("No modeled chamfer faces")?.width();
                    p.top = input.heights.top;
                    p.depth = depth;
                }
                (Operation::Chamfer(p), ExtractionRecipe::ChamferLoop) => {
                    p.faces.clear();
                    let path = input.profiles.first().ok_or("No chamfer rim")?;
                    if p.chain != path.chain {
                        p.side = path.side;
                    }
                    p.chain = path.chain.clone();
                    p.closed = path.closed;
                    if !p.closed {
                        p.start = None;
                        p.end = None;
                        p.lead_overlap = 0.;
                    }
                    p.top = input.heights.top;
                }
                (Operation::Engrave(p), ExtractionRecipe::EngravePath) => {
                    let path = input.profiles.first().ok_or("No engraving path")?;
                    p.chain = path.chain.clone();
                    p.closed = path.closed;
                    p.top = input.heights.top;
                    p.depth = depth;
                    if !p.closed {
                        p.start = None;
                        p.end = None;
                    }
                }
                (Operation::Carve(p), ExtractionRecipe::CarveRegion) => {
                    p.boundary = region()?.outer().clone();
                    p.islands = region()?.holes().to_vec();
                    p.top = input.heights.top;
                    p.depth = depth;
                }
                (Operation::Bore(p), ExtractionRecipe::BoreCylinders { internal }) => {
                    if p.internal != *internal || p.taper_angle != 0. {
                        return Err(
                            "Bore type and straight wall must match the selected cylinder".into(),
                        );
                    }
                    p.points = input
                        .holes
                        .iter()
                        .map(|h| [h.center.x, h.center.y])
                        .collect();
                    p.diameter = input.holes[0].diameter;
                    p.z_top = input.heights.top;
                    p.z_bottom = input.heights.bottom;
                    if p.axial_stock < 0. {
                        return Err("CAD Bore axial stock cannot extend below the certified face; use a qualified bottom selection".into());
                    }
                }
                (Operation::Thread(p), ExtractionRecipe::ThreadCylinders { internal }) => {
                    if p.internal != *internal {
                        return Err("Thread type must match the selected bore or boss".into());
                    }
                    // The nominal thread diameter is independently editable.
                    // Fitting its theoretical minor diameter does not prove that
                    // the cutter can enter the actual selected/prepared bore.
                    if p.internal {
                        let tool = document
                            .setup()
                            .tools
                            .iter()
                            .find(|t| t.number == p.tool)
                            .ok_or("Missing thread tool")?;
                        if input
                            .holes
                            .iter()
                            .any(|hole| tool.diameter >= hole.diameter)
                        {
                            return Err(format!(
                                "Thread tool diameter {:.3} cannot enter the selected bore; choose a smaller thread mill or prepare a larger bore",
                                tool.diameter
                            ));
                        }
                    }
                    p.points = input
                        .holes
                        .iter()
                        .map(|h| [h.center.x, h.center.y])
                        .collect();
                    p.z_top = input.heights.top;
                    p.z_bottom = input.heights.bottom;
                }
                (Operation::Drill(p), ExtractionRecipe::DrillHoles) => {
                    let tool = document
                        .setup()
                        .tools
                        .iter()
                        .find(|t| t.number == p.tool)
                        .ok_or("Missing drill tool")?;
                    if input
                        .holes
                        .iter()
                        .any(|h| (h.diameter - tool.diameter).abs() > 1e-3)
                    {
                        return Err(format!(
                            "Operation {id}: drill diameter must match the selected bore"
                        ));
                    }
                    p.points = input
                        .holes
                        .iter()
                        .map(|h| [h.center.x, h.center.y])
                        .collect();
                    p.depth = depth;
                    p.start_offset =
                        input.heights.top - document.setup().heights_for(id).top_of_stock;
                }
                (Operation::Face(p), ExtractionRecipe::FaceRegion) => {
                    if p.overshoot < 0.0 {
                        return Err(format!(
                            "Operation {id}: negative overshoot is not certified for CAD facing"
                        ));
                    }
                    p.boundary = region()?.outer().clone();
                    p.depth = depth;
                    p.start_offset = input.heights.top;
                }
                (Operation::Parallel(_), ExtractionRecipe::ParallelFinishing)
                | (Operation::Spiral(_), ExtractionRecipe::SpiralFinishing)
                | (Operation::Adaptive(_), ExtractionRecipe::AdaptiveClearing) => {
                    // Body heights and projected silhouette belong to the binding;
                    // the adaptive planner resolves conservative CAD-derived layers.
                }
                _ => {
                    return Err(format!(
                        "Operation {id}: geometry recipe does not match the operation"
                    ))
                }
            }
            // Planner consumes absolute heights. The extraction service already applied
            // and certified manual overrides; do not apply them a second time.
            let cache = resolved.geometry.bindings.get_mut(&id).unwrap();
            cache.heights = input.heights;
            cache.overrides = GeometryOverrides::default();
            let fingerprint = resolved
                .geometry
                .binding_fingerprint(&resolved.geometry.bindings[&id]);
            resolved
                .geometry
                .bindings
                .get_mut(&id)
                .unwrap()
                .resolved_fingerprint = Some(fingerprint);
            Ok(())
        })();
        resolved.geometry.instances = base_instances;
        resolved_operation.map_err(|message| error(message).for_op(id))?;
        if document.rotary_indexes.contains_key(&id) {
            if let Some(settings) = document.setup().height_settings.get(&id) {
                // References such as SelectedContours depend on the binding's
                // cutting interval. Resolve it after extracting indexed CAD,
                // while keeping original placements for the indexed view.
                let mut source = document.clone();
                if let Some(binding) = resolved.geometry.bindings.get(&id) {
                    source.geometry.bindings.insert(id, binding.clone());
                }
                let mut settings = settings.clone();
                settings.resolved = None;
                let heights = settings
                    .resolve(&source, operation)
                    .map_err(|reason| error(format!("Operation {id}: {reason}")).for_op(id))?;
                resolved
                    .setup_mut()
                    .height_settings
                    .get_mut(&id)
                    .unwrap()
                    .resolved = Some(heights);
            }
        }
    }
    resolved.setup_mut().operations = operations;
    // Keep fixture placements for the planner's independent safety checks, after
    // resolving machining references against model bodies only.
    document
        .setup()
        .validate_fixtures(&document.geometry)
        .map_err(error)?;
    if !document.setup().fixture_instances.is_empty() {
        let setup_geometry = document
            .geometry
            .geometry_in_setup(&document.setup().frame)
            .map_err(error)?;
        resolved.geometry.instances.extend(
            setup_geometry
                .instances
                .into_iter()
                .filter(|instance| document.setup().fixture_instances.contains(&instance.id)),
        );
    }
    Ok(resolved)
}

pub(crate) fn mark_missing_bindings(state: &mut GeometryState) {
    let stale: Vec<_> = state
        .bindings
        .iter()
        .filter(|(_, b)| !b.selected.iter().all(|r| state.entity_exists(r)))
        .map(|(id, _)| *id)
        .collect();
    for id in stale {
        state.bindings.get_mut(&id).unwrap().status = ResolutionStatus::NeedsReselection {
            reason: "Source geometry revision or entity is missing; reselect geometry".into(),
        };
    }
}

pub(crate) fn selection_height_defaults(
    document: &Document,
    operation: &Operation,
) -> OperationHeightSettings {
    let mut settings = OperationHeightSettings::default();
    // Pocket floors, loops and closed edge chains all describe the target
    // bottom plane. Start at stock top, including after geometry replacement.
    if matches!(operation, Operation::Pocket(_)) {
        return settings;
    }
    if matches!(
        operation,
        Operation::Adaptive(_) | Operation::Parallel(_) | Operation::Spiral(_)
    ) {
        *settings.definition_mut(HeightPlaneKind::Top) =
            HeightDefinition::new(HeightReference::StockTop, 0.);
        *settings.definition_mut(HeightPlaneKind::Bottom) =
            HeightDefinition::new(HeightReference::SelectedContours, 0.);
        return settings;
    }
    let cutting = document.operation_base_cutting_heights(operation);
    *settings.definition_mut(HeightPlaneKind::Top) =
        HeightDefinition::new(HeightReference::SelectedContours, 0.);
    *settings.definition_mut(HeightPlaneKind::Bottom) = HeightDefinition::new(
        HeightReference::Plane(HeightPlaneKind::Top),
        -cutting.depth(),
    );
    settings
}

pub(crate) fn surface_recipe(recipe: &ExtractionRecipe) -> bool {
    matches!(
        recipe,
        ExtractionRecipe::ChamferLoop
            | ExtractionRecipe::EngravePath
            | ExtractionRecipe::CarveRegion
            | ExtractionRecipe::PocketBoundary
    )
}

/// Inspector depth edits are explicit overrides while tool/feed/allowance edits
/// preserve the geometry link. Depth values remain physical magnitudes.
pub(crate) fn capture_height_override(
    binding: Option<&mut GeometryBinding>,
    before: Option<CuttingHeights>,
    after: &Operation,
    heights: &Heights,
) {
    // Adaptive operations have no legacy cutting-height fields. Their interval
    // belongs to the binding and reference-height settings, so comparing it with
    // legacy_cutting would turn an unrelated edit into a spurious height shift.
    if matches!(
        after,
        Operation::Adaptive(_) | Operation::Parallel(_) | Operation::Spiral(_)
    ) {
        return;
    }
    let Some(binding) = binding else {
        return;
    };
    let Some(before) = before else {
        return;
    };
    let operation = after;
    let after = cam_model::legacy_cutting(operation, heights);
    let effective = binding.effective_heights();
    match after.top - before.top {
        delta if delta != 0.0 => {
            binding.overrides.top = Some(effective.top + delta);
            binding.overrides.bottom = Some(effective.bottom + delta);
        }
        _ => {}
    }
    let (before_depth, after_depth) = match operation {
        Operation::Pocket(p) if binding.recipe == ExtractionRecipe::PocketBoundary => {
            (before.depth(), p.depth)
        }
        Operation::Pocket(p) => (-before.bottom, p.depth),
        Operation::Profile(p) => (-before.bottom, p.depth),
        Operation::Drill(p) => (before.depth(), p.depth),
        Operation::Face(p) => (before.depth(), p.depth),
        Operation::Engrave(p) => (before.depth(), p.depth),
        Operation::Carve(p) => (before.depth(), p.depth),
        Operation::Bore(p) => (before.depth(), p.z_top - p.z_bottom),
        Operation::Thread(p) => (before.depth(), p.z_top - p.z_bottom),
        Operation::Chamfer(_)
        | Operation::Adaptive(_)
        | Operation::Parallel(_)
        | Operation::Spiral(_) => return,
    };
    if after_depth != before_depth {
        binding.overrides.bottom = Some(binding.effective_heights().top - after_depth);
    }
}

/// Validate an inspector edit without discarding its explicit overrides or
/// requiring unrelated unresolved operations to be repaired first.
pub(crate) fn validate_binding_edit(document: &mut Document, id: u32) -> Result<(), String> {
    let mut preview = document.clone();
    preview.setup_mut().operations.retain(|op| op.id() == id);
    // Geometry repair is independent of the height editor's pending references.
    preview.setup_mut().height_settings.clear();
    let resolved = resolve_document(&preview)?;
    let operation = resolved
        .setup()
        .operations
        .first()
        .cloned()
        .ok_or("Missing operation")?;
    let slot = document
        .setup_mut()
        .operations
        .iter_mut()
        .find(|op| op.id() == id)
        .ok_or("Missing operation")?;
    *slot = operation.clone();
    let mut base = document.geometry.bindings[&id].clone();
    base.overrides = GeometryOverrides::default();
    if surface_recipe(&base.recipe) {
        let cutting = document.operation_base_cutting_heights(&operation);
        base.heights.bottom = base.heights.top - cutting.depth();
    }
    if matches!(base.recipe, ExtractionRecipe::FaceRegion) {
        base.heights.top = operation_stock_top(document, id)?;
    }
    let input =
        cam_geometry3d::resolve_binding_advisory(&geometry_for_operation(document, id)?, &base)?;
    let binding = document.geometry.bindings.get_mut(&id).unwrap();
    binding.heights = input.heights;
    binding.resolved_fingerprint = None;
    Ok(())
}

/// Keep displayed linked heights synchronized with placement and stock edits.
/// Replacement/removal requests remain unresolved until explicit reselection.
pub(crate) fn refresh_binding_heights(document: &mut Document) {
    let updates: Vec<_> = document.geometry.bindings.iter().filter_map(|(&id,binding)| {
        if matches!(&binding.status,ResolutionStatus::NeedsReselection{reason} if !reason.starts_with("Geometry changed:")) {return None;}
        let mut base=binding.clone();base.overrides=GeometryOverrides::default();
        let mut context=document.clone();
        let owner=document.setups.iter().find(|s|s.operations.iter().any(|o|o.id()==id))?;
        context.active_setup_id=owner.id;
        if matches!(base.recipe,ExtractionRecipe::FaceRegion) {base.heights.top=operation_stock_top(&context,id).ok()?;}
        Some((id,geometry_for_operation(&context,id).and_then(|geometry|cam_geometry3d::resolve_binding_advisory(&geometry,&base)).map(|input|input.heights)))
    }).collect();
    for (id, result) in updates {
        let binding = document.geometry.bindings.get_mut(&id).unwrap();
        binding.resolved_fingerprint = None;
        match result {
            Ok(heights) => {
                binding.heights = heights;
                binding.status = ResolutionStatus::Resolved;
            }
            Err(reason) => {
                binding.status = ResolutionStatus::NeedsReselection {
                    reason: format!("Geometry changed: {reason}"),
                }
            }
        }
    }
}

#[cfg(test)]
mod pocket_stock_tests {
    use super::*;
    use crate::{default_machine, AppController};
    use cam_cldata::{MoveKind, SpindleDir, Step};
    use cam_toolpath::{CancelToken, JobCompletion};

    #[test]
    #[ignore = "supplied testfile2.ocam regression; set OCAM_STOCK_PROJECT"]
    fn supplied_floor_holes_are_cleared_to_the_stock_contour() {
        let path = std::env::var("OCAM_STOCK_PROJECT").unwrap();
        let mut app = AppController::new(default_machine());
        app.open_project(Path::new(&path)).unwrap();
        let (min, max) = app.stock_box();
        // The saved project selects the right floor; the screenshot selects
        // the corresponding left floor in the same flipped setup.
        for (face, center) in [(34, 75.), (32, -75.)] {
            let mut selected = app.geometry_state().bindings[&0].selected.clone();
            selected[0].id = face;
            app.edit_cad_geometry(0, selected).unwrap();
            let document = resolve_document(app.document()).unwrap();
            assert!(
                matches!(&document.setup().operations[0], Operation::Pocket(p)
                if p.stock_contours.enabled && p.islands.len() == 1)
            );
            let started = std::time::Instant::now();
            let result = cam_toolpath::build_job(
                &document,
                1000.,
                SpindleDir::Cw,
                Some(([min[0], min[1]], [max[0], max[1]])),
                200.,
                &CancelToken::new(),
            );
            eprintln!(
                "Face {face}: {} moves in {:?}",
                result.program.len(),
                started.elapsed()
            );
            assert_eq!(
                result.completion,
                JobCompletion::Complete,
                "{:?}",
                result.diagnostics
            );
            result.program.validate().unwrap();
            let mut previous: Option<cam_cldata::Point3> = None;
            let mut crosses_hole = false;
            for step in result.program.steps() {
                match step {
                    Step::Linear { to, tag, .. } => {
                        if let Some(a) = previous.filter(|_| tag.kind == MoveKind::Cutting) {
                            let dx = to.x - a.x;
                            let dy = to.y - a.y;
                            let length_sq = dx * dx + dy * dy;
                            if length_sq > 0. {
                                let t =
                                    (((center - a.x) * dx - a.y * dy) / length_sq).clamp(0., 1.);
                                crosses_hole |= (a.x + t * dx - center).hypot(a.y + t * dy) < 3.;
                            }
                        }
                        previous = Some(*to);
                    }
                    Step::Rapid { to, .. } => previous = Some(*to),
                    Step::Arc { end, .. } => previous = Some(*end),
                    _ => {}
                }
            }
            let cuts: Vec<_> = result
                .program
                .steps()
                .iter()
                .filter_map(|s| match s {
                    Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(*to),
                    Step::Arc { end, tag, .. } if tag.kind == MoveKind::Cutting => Some(*end),
                    _ => None,
                })
                .collect();
            assert!(
                crosses_hole,
                "stock above the 20 mm through-hole must be machined"
            );
            assert!(cuts.iter().any(|p| p.y > max[1]));
            assert!(cuts.iter().any(|p| p.y < min[1]));
            assert!(
                cuts.iter().any(|p| if center > 0. {
                    p.x > max[0]
                } else {
                    p.x < min[0]
                }),
                "the open floor must extend to the stock edges with runout"
            );
            assert!(
                cuts.iter().all(|p| if center > 0. {
                    p.x >= 53. - 0.01
                } else {
                    p.x <= -53. + 0.01
                }),
                "the rising wall at X +/-50 must remain protected"
            );
        }
    }
}
