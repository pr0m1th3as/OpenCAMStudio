use std::ops::Range;

use cam_model::{
    scale_cad_topology, Drawing, EntityRef, FrameOrigin, Operation, ProfileGeometrySource,
    ResolutionStatus, Stock, StockAxis, StockAxisReference,
};

use super::*;

pub(crate) struct DrawingSource<'a> {
    pub name: &'a str,
    pub regions: Range<usize>,
    pub paths: Range<usize>,
}

impl AppController {
    pub(crate) fn drawing_sources(&self) -> Vec<DrawingSource<'_>> {
        let geometry = self.geometry_state();
        let ends = (geometry.drawing_regions.len(), geometry.drawing_paths.len());
        if ends == (0, 0) {
            return Vec::new();
        }
        let mut ranges = Vec::new();
        let (mut regions, mut paths) = (0usize, 0usize);
        for drawing in &self.document().drawings {
            let Some(end_regions) = regions
                .checked_add(drawing.regions)
                .filter(|&n| n <= ends.0)
            else {
                break;
            };
            let Some(end_paths) = paths.checked_add(drawing.paths).filter(|&n| n <= ends.1) else {
                break;
            };
            ranges.push(DrawingSource {
                name: &drawing.name,
                regions: regions..end_regions,
                paths: paths..end_paths,
            });
            (regions, paths) = (end_regions, end_paths);
        }
        if (regions, paths) == ends && ranges.len() == self.document().drawings.len() {
            return ranges;
        }
        // Older projects stored drawing geometry as a single undifferentiated collection.
        vec![DrawingSource {
            name: if self.source_name.is_empty() || !geometry.assets.is_empty() {
                "Drawing"
            } else {
                &self.source_name
            },
            regions: 0..ends.0,
            paths: 0..ends.1,
        }]
    }

    pub(crate) fn drawing_records(&self) -> Vec<Drawing> {
        self.drawing_sources()
            .into_iter()
            .map(|source| Drawing {
                name: source.name.into(),
                regions: source.regions.len(),
                paths: source.paths.len(),
            })
            .collect()
    }

    pub(crate) fn scaled_drawing_geometry(
        &self,
        source: &DrawingSource<'_>,
        factors: [f64; 2],
    ) -> Result<(Vec<Polygon>, Vec<Polyline>), String> {
        let point = |p: &Point| {
            let result = Point::new(p.x * factors[0], p.y * factors[1]);
            if result.x.is_finite() && result.y.is_finite() {
                Ok(result)
            } else {
                Err("Scaled drawing coordinates are out of range".to_string())
            }
        };
        let points = |values: &[Point]| values.iter().map(point).collect::<Result<Vec<_>, _>>();
        let contour = |value: &Contour| {
            let scaled = Contour::new(points(value.points())?);
            let area = scaled.area();
            if !area.is_finite() || (area == 0. && value.area() != 0.) {
                return Err(
                    "Scale factor makes a drawing contour degenerate or out of range".into(),
                );
            }
            Ok::<_, String>(scaled)
        };
        let new_regions = self.regions[source.regions.clone()]
            .iter()
            .map(|region| {
                let holes = region
                    .holes()
                    .iter()
                    .map(contour)
                    .collect::<Result<Vec<_>, _>>()?;
                Polygon::with_holes(contour(region.outer())?, holes).map_err(|e| e.to_string())
            })
            .collect::<Result<Vec<_>, _>>()?;
        let new_paths = self.open_paths[source.paths.clone()]
            .iter()
            .map(|path| {
                let scaled = Polyline::new(points(path.points())?);
                if scaled.points().windows(2).all(|pair| pair[0] == pair[1]) {
                    return Err("Scale factor makes a drawing path degenerate".into());
                }
                Ok(scaled)
            })
            .collect::<Result<Vec<_>, String>>()?;
        Ok((new_regions, new_paths))
    }

    pub fn scale_drawing(&mut self, index: usize, factors: [f64; 2]) -> Result<(), String> {
        if !factors
            .into_iter()
            .all(|factor| factor.is_finite() && factor > 0.)
        {
            return Err("Scale factors must be finite and greater than zero".into());
        }
        let sources = self.drawing_sources();
        let source = sources
            .get(index)
            .ok_or("Drawing is no longer in the project")?;
        if factors == [1.; 2] {
            return Ok(());
        }
        let regions = source.regions.clone();
        let paths = source.paths.clone();
        let old_regions = &self.regions[regions.clone()];
        let old_paths = &self.open_paths[paths.clone()];
        let (new_regions, new_paths) = self.scaled_drawing_geometry(source, factors)?;

        let mut document = self.document().clone();
        for plane in document
            .setups
            .iter_mut()
            .flat_map(|setup| setup.height_settings.values_mut())
            .flat_map(|settings| &mut settings.planes)
        {
            for drawing in &mut plane.drawings {
                let matches = |points: &[Point]| {
                    points
                        .iter()
                        .map(|p| [p.x, p.y])
                        .eq(drawing.points.iter().copied())
                };
                let replacement = if drawing.closed {
                    old_regions.iter().zip(&new_regions).find_map(|(old, new)| {
                        std::iter::once(old.outer())
                            .chain(old.holes())
                            .zip(std::iter::once(new.outer()).chain(new.holes()))
                            .find_map(|(old, new)| matches(old.points()).then_some(new.points()))
                    })
                } else {
                    old_paths
                        .iter()
                        .zip(&new_paths)
                        .find_map(|(old, new)| matches(old.points()).then_some(new.points()))
                };
                if let Some(points) = replacement {
                    drawing.points = points.iter().map(|p| [p.x, p.y]).collect();
                }
            }
        }
        document
            .geometry
            .drawing_regions
            .splice(regions, new_regions);
        document.geometry.drawing_paths.splice(paths, new_paths);
        self.edit_document(|current| *current = document);
        self.sync_drawing_geometry();
        self.invalidate();
        Ok(())
    }

    pub fn scale_cad_asset(&mut self, id: u64, factors: [f64; 3]) -> Result<(), String> {
        let mut document = self.document().clone();
        let asset = document
            .geometry
            .assets
            .iter_mut()
            .find(|asset| asset.id == id)
            .ok_or("Model is no longer in the project")?;
        if factors == [1.; 3] {
            return Ok(());
        }
        let mut instances: Vec<_> = document
            .geometry
            .instances
            .iter()
            .filter(|instance| instance.asset_id == id)
            .cloned()
            .collect();
        let (topology, offsets) = scale_cad_topology(&asset.topology, &mut instances, factors)?;
        let previous_revision = asset.revision;
        let revision = self.next_cad_revision(previous_revision)?;
        asset.revision = revision;
        asset.topology = topology;
        for (instance, scaled) in document
            .geometry
            .instances
            .iter_mut()
            .filter(|instance| instance.asset_id == id)
            .zip(instances)
        {
            *instance = scaled;
        }

        let remap = |reference: &mut EntityRef| {
            if reference.asset_id != id
                || reference.revision != previous_revision
                || !self.geometry_state().entity_exists(reference)
            {
                return false;
            }
            reference.revision = revision;
            reference.body_id += offsets[&reference.instance_id];
            reference.id += offsets[&reference.instance_id];
            true
        };
        for binding in document.geometry.bindings.values_mut() {
            for reference in &mut binding.selected {
                if remap(reference) {
                    binding.resolved_fingerprint = None;
                }
            }
        }
        for index in document.rotary_indexes.values_mut() {
            if let Some(face) = &mut index.face {
                remap(face);
            }
        }
        for setup in &mut document.setups {
            let refs = &mut setup.frame.references;
            if let (FrameOrigin::ModelOrigin, Some(instance)) = (&refs.origin, refs.model_instance)
            {
                if offsets.contains_key(&instance) {
                    let translation = document
                        .geometry
                        .instance(instance)
                        .unwrap()
                        .placement
                        .translation;
                    setup.origin = setup.frame.to_setup_vector(translation);
                }
            }
            let mut refs = setup.frame.references.clone();
            let mut axes_changed = false;
            for axis in [&mut refs.x, &mut refs.y, &mut refs.z]
                .into_iter()
                .flatten()
            {
                axes_changed |= remap(&mut axis.entity);
            }
            if axes_changed {
                // An anisotropic scale can make the old edge picks nonorthogonal.
                // Keep the existing rigid setup until its axes can be resolved.
                if let Ok(frame) = setup
                    .frame
                    .resolve_references(&document.geometry, refs.clone())
                {
                    let world_origin = setup.frame.to_world_vector(setup.origin);
                    setup.origin = frame.to_setup_vector(world_origin);
                    for origin in &mut setup.extra_origins {
                        origin.position =
                            frame.to_setup_vector(setup.frame.to_world_vector(origin.position));
                    }
                    setup.frame = frame;
                } else {
                    setup.frame.references = refs;
                }
            }
            if let Stock::FixedRound { axis, .. } | Stock::RelativeRound { axis, .. } =
                &mut setup.stock
            {
                if let Some(StockAxis::Edge(edge)) = axis {
                    let mut reference = edge.entity();
                    if remap(&mut reference) {
                        *edge = StockAxisReference::from(&reference);
                    }
                }
            }
            for plane in setup
                .height_settings
                .values_mut()
                .flat_map(|settings| &mut settings.planes)
            {
                for reference in &mut plane.entities {
                    remap(reference);
                }
            }
            for pattern in &mut setup.patterns {
                if let Some(axis) = &mut pattern.options.axis {
                    remap(axis);
                }
                for direction in &mut pattern.options.directions {
                    if let Some(reference) = &mut direction.reference {
                        remap(reference);
                    }
                }
            }
            for operation in &mut setup.operations {
                let wrap = match operation {
                    Operation::Profile(profile) => {
                        for entry in profile.selections.iter_mut().flatten() {
                            if let ProfileGeometrySource::Cad(references) = &mut entry.source {
                                for reference in references {
                                    remap(reference);
                                }
                            }
                        }
                        &mut profile.wrap
                    }
                    Operation::Pocket(pocket) => {
                        if let Some(binding) = &mut pocket.stock_contours.source {
                            for reference in &mut binding.selected {
                                if remap(reference) {
                                    binding.resolved_fingerprint = None;
                                }
                            }
                        }
                        &mut pocket.wrap
                    }
                    _ => continue,
                };
                if let Some(wrap) = wrap {
                    if let Some(cylinder) = &mut wrap.cylinder {
                        remap(cylinder);
                    }
                    for reference in &mut wrap.geometry {
                        remap(reference);
                    }
                }
            }
        }

        crate::cad_operations::refresh_binding_heights(&mut document);
        for (id, binding) in &mut document.geometry.bindings {
            if let Err(reason) = binding.effective_heights().validate() {
                binding.heights = self.geometry_state().bindings[id].heights;
                binding.status = ResolutionStatus::NeedsReselection {
                    reason: format!("Geometry changed: {reason}"),
                };
            }
        }
        document.geometry.validate()?;
        let mut selection = self.cad_selection.clone();
        for reference in &mut selection {
            remap(reference);
        }
        self.edit_document(|current| *current = document);
        self.cad_selection = selection;
        self.cad_revision_floor = revision;
        self.invalidate();
        Ok(())
    }
}
