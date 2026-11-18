//! Persistent reference-plus-offset operation heights, resolved in setup axes.
use crate::{CuttingHeights, Document, EntityKind, EntityRef, Heights, Operation};
use serde::{Deserialize, Serialize};

/// Round blank frame and bounds expressed in that frame.
pub type RoundStockGeometry = (crate::SetupFrame, [f64; 3], [f64; 3]);

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum HeightPlaneKind {
    Clearance,
    Retract,
    Feed,
    Top,
    Bottom,
}
impl HeightPlaneKind {
    pub const ALL: [Self; 5] = [
        Self::Clearance,
        Self::Retract,
        Self::Feed,
        Self::Top,
        Self::Bottom,
    ];
    pub fn index(self) -> usize {
        Self::ALL.iter().position(|p| *p == self).unwrap()
    }
}
impl std::fmt::Display for HeightPlaneKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Clearance => "Clearance height",
            Self::Retract => "Retract height",
            Self::Feed => "Feed height",
            Self::Top => "Top height",
            Self::Bottom => "Bottom height",
        })
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HeightReference {
    Plane(HeightPlaneKind),
    Disabled,
    ModelTop,
    ModelBottom,
    StockTop,
    StockBottom,
    FixtureTop,
    FixtureBottom,
    SelectedContours,
    Selection,
    Origin,
    Highest,
    Lowest,
}
impl HeightReference {
    pub fn options(plane: HeightPlaneKind) -> Vec<Self> {
        let mut choices: Vec<_> = HeightPlaneKind::ALL
            .into_iter()
            .filter(|p| *p != plane)
            .map(Self::Plane)
            .collect();
        if plane == HeightPlaneKind::Feed {
            choices.insert(2, Self::Disabled);
        }
        choices.extend([
            Self::ModelTop,
            Self::ModelBottom,
            Self::StockTop,
            Self::StockBottom,
            Self::FixtureTop,
            Self::FixtureBottom,
            Self::SelectedContours,
            Self::Selection,
            Self::Origin,
            Self::Highest,
            Self::Lowest,
        ]);
        choices
    }
}
impl std::fmt::Display for HeightReference {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Plane(p) => p.fmt(f),
            _ => f.write_str(match self {
                Self::Disabled => "Disabled",
                Self::ModelTop => "Model top",
                Self::ModelBottom => "Model bottom",
                Self::StockTop => "Stock top",
                Self::StockBottom => "Stock bottom",
                Self::FixtureTop => "Fixture top",
                Self::FixtureBottom => "Fixture bottom",
                Self::SelectedContours => "Selected contour(s)",
                Self::Selection => "Selection",
                Self::Origin => "Origin (absolute)",
                Self::Highest => "Highest of…",
                Self::Lowest => "Lowest of…",
                _ => unreachable!(),
            }),
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DrawingHeightSelection {
    pub points: Vec<[f64; 2]>,
    pub closed: bool,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HeightDefinition {
    pub reference: HeightReference,
    pub offset: f64,
    /// Stable CAD geometry for selection/fixture references; never a screen mesh.
    #[serde(default)]
    pub entities: Vec<EntityRef>,
    #[serde(default)]
    pub drawings: Vec<DrawingHeightSelection>,
    /// Reference set for Highest/Lowest. Nested aggregates are rejected.
    #[serde(default)]
    pub sources: Vec<HeightReference>,
}
impl HeightDefinition {
    pub fn new(reference: HeightReference, offset: f64) -> Self {
        Self {
            reference,
            offset,
            entities: vec![],
            drawings: vec![],
            sources: vec![],
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OperationHeightSettings {
    pub planes: [HeightDefinition; 5],
    /// A worker resolves source geometry before transforming it into setup axes.
    /// This transient snapshot prevents applying that transformation twice.
    #[serde(skip)]
    pub resolved: Option<ResolvedOperationHeights>,
}
impl Default for OperationHeightSettings {
    fn default() -> Self {
        use HeightPlaneKind::*;
        use HeightReference::*;
        Self {
            planes: [
                HeightDefinition::new(Plane(Retract), 10.),
                HeightDefinition::new(StockTop, 5.),
                HeightDefinition::new(Plane(Top), 5.),
                HeightDefinition::new(StockTop, 0.),
                HeightDefinition::new(SelectedContours, 0.),
            ],
            resolved: None,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResolvedOperationHeights {
    pub stock_top: f64,
    pub clearance: f64,
    pub retract: f64,
    pub feed: Option<f64>,
    pub top: f64,
    pub bottom: f64,
}
impl ResolvedOperationHeights {
    pub fn plane(self, kind: HeightPlaneKind) -> Option<f64> {
        match kind {
            HeightPlaneKind::Clearance => Some(self.clearance),
            HeightPlaneKind::Retract => Some(self.retract),
            HeightPlaneKind::Feed => self.feed,
            HeightPlaneKind::Top => Some(self.top),
            HeightPlaneKind::Bottom => Some(self.bottom),
        }
    }
    pub fn safety(self) -> Heights {
        Heights::new(self.clearance, self.retract, self.stock_top)
    }
    pub fn cutting(self) -> CuttingHeights {
        CuttingHeights {
            top: self.top,
            bottom: self.bottom,
        }
    }
    pub fn validate(self) -> Result<(), String> {
        if ![
            self.stock_top,
            self.clearance,
            self.retract,
            self.top,
            self.bottom,
        ]
        .into_iter()
        .chain(self.feed)
        .all(f64::is_finite)
        {
            return Err("Height references must resolve to finite coordinates".into());
        }
        if self.top <= self.bottom {
            return Err("Top height must be above bottom height".into());
        }
        if self.clearance < self.retract || self.retract < self.top {
            return Err(
                "Clearance must be at or above retract, and retract at or above top height".into(),
            );
        }
        if self
            .feed
            .is_some_and(|z| z < self.top || z > self.clearance)
        {
            return Err("Feed height must be between top and clearance heights".into());
        }
        Ok(())
    }
}
impl OperationHeightSettings {
    pub fn definition(&self, plane: HeightPlaneKind) -> &HeightDefinition {
        &self.planes[plane.index()]
    }
    pub fn definition_mut(&mut self, plane: HeightPlaneKind) -> &mut HeightDefinition {
        self.resolved = None;
        &mut self.planes[plane.index()]
    }
    pub fn validate_structure(&self) -> Result<(), String> {
        for plane in HeightPlaneKind::ALL {
            let d = self.definition(plane);
            if !d.offset.is_finite() {
                return Err(format!("{plane}: offset must be a finite number"));
            }
            if d.reference == HeightReference::Disabled && plane != HeightPlaneKind::Feed {
                return Err("Only feed height may be disabled".into());
            }
            if d.reference == HeightReference::Plane(plane) {
                return Err(format!("{plane} cannot reference itself"));
            }
            if matches!(
                d.reference,
                HeightReference::Highest | HeightReference::Lowest
            ) && (d.sources.is_empty()
                || d.sources.iter().any(|s| {
                    matches!(
                        s,
                        HeightReference::Highest
                            | HeightReference::Lowest
                            | HeightReference::Disabled
                    )
                }))
            {
                return Err(format!(
                    "{plane}: choose at least one non-aggregate reference"
                ));
            }
        }
        Ok(())
    }
    pub fn resolve(
        &self,
        document: &Document,
        operation: &Operation,
    ) -> Result<ResolvedOperationHeights, String> {
        if let Some(resolved) = self.resolved {
            return Ok(resolved);
        }
        if let Some(view) = document.indexed_view(operation.id())? {
            return self.resolve(&view, operation);
        }
        self.validate_structure()?;
        let mut values = [None; 5];
        let mut visiting = [false; 5];
        for p in HeightPlaneKind::ALL {
            self.resolve_plane(p, document, operation, &mut values, &mut visiting)?;
        }
        let stock_top = if let Some(wrap) = operation.rotary_wrap() {
            wrap.stock_top(document)?
        } else {
            let (model_min, model_max) = document.height_model_bounds();
            document
                .setup()
                .stock
                .resolve_bounds(model_min, model_max)
                .1[2]
        };
        let resolved = ResolvedOperationHeights {
            stock_top,
            clearance: values[0].unwrap(),
            retract: values[1].unwrap(),
            feed: values[2],
            top: values[3].unwrap(),
            bottom: values[4].unwrap(),
        };
        resolved.validate()?;
        if let Operation::Chamfer(op) = operation {
            if op.faces.is_empty() {
                let minimum = document.chamfer_minimum_depth(op);
                if resolved.top - resolved.bottom < minimum - 1e-9 {
                    return Err(format!("Bottom height must allow at least {minimum:.3} mm below top for the selected chamfer and cutter geometry"));
                }
            }
        }
        Ok(resolved)
    }
    fn resolve_plane(
        &self,
        p: HeightPlaneKind,
        doc: &Document,
        op: &Operation,
        values: &mut [Option<f64>; 5],
        visiting: &mut [bool; 5],
    ) -> Result<Option<f64>, String> {
        let i = p.index();
        if let Some(z) = values[i] {
            return Ok(Some(z));
        }
        if visiting[i] {
            return Err(format!("Circular height reference involving {p}"));
        }
        let d = self.definition(p);
        if d.reference == HeightReference::Disabled {
            return Ok(None);
        }
        visiting[i] = true;
        let base = self.resolve_reference(d.reference, d, doc, op, values, visiting)?;
        let z = base + d.offset;
        if !z.is_finite() {
            return Err(format!("{p}: resolved height is not finite"));
        }
        visiting[i] = false;
        values[i] = Some(z);
        Ok(Some(z))
    }
    fn resolve_reference(
        &self,
        reference: HeightReference,
        d: &HeightDefinition,
        doc: &Document,
        op: &Operation,
        values: &mut [Option<f64>; 5],
        visiting: &mut [bool; 5],
    ) -> Result<f64, String> {
        use HeightReference::*;
        let setup = doc.setup();
        if let Some(wrap) = op.rotary_wrap() {
            match reference {
                StockTop => return wrap.stock_top(doc),
                ModelTop => return Ok(wrap.surface_z),
                Origin => return Ok(0.),
                SelectedContours => return match wrap.boundary_z {
                    Some(height) => Ok(height),
                    None if wrap.unwrap_geometry && !wrap.geometry.is_empty() => Err(
                        "Selected contours do not have one radial height; split the wrapped selections or choose an explicit radial plane".into()
                    ),
                    None => Ok(doc.operation_base_cutting_heights(op).bottom),
                },
                ModelBottom|StockBottom|Selection|FixtureTop|FixtureBottom=>return Err("Wrapped heights use the developed plane; choose a height plane, Origin, or Stock/Model top with a radial offset".into()),
                _=>{}
            }
        }
        let (model_min, model_max) = doc.height_model_bounds();
        let (stock_min, stock_max) = doc.stock_bounds_for(setup);
        Ok(match reference {
            Plane(p) => self
                .resolve_plane(p, doc, op, values, visiting)?
                .ok_or("A height cannot reference disabled feed height")?,
            Disabled => return Err("Disabled height cannot be a reference".into()),
            ModelTop => model_max[2],
            ModelBottom => model_min[2],
            StockTop => stock_max[2],
            StockBottom => stock_min[2],
            Origin => setup.origin_position(op.work_offset())[2],
            SelectedContours => doc.selected_contour_height(op)?,
            Selection | FixtureTop | FixtureBottom => {
                let (min, max) = doc.height_definition_bounds(d)?;
                if reference == Selection && (max[2] - min[2]).abs() > 1e-5 {
                    return Err("Selection must lie on one plane perpendicular to setup Z; use Highest/Lowest or a fixture limit for non-planar geometry".into());
                }
                if reference == FixtureBottom {
                    min[2]
                } else {
                    max[2]
                }
            }
            Highest | Lowest => {
                let zs = d
                    .sources
                    .iter()
                    .map(|source| {
                        if *source == Selection {
                            let (min, max) = doc.height_definition_bounds(d)?;
                            Ok(if reference == Highest { max[2] } else { min[2] })
                        } else {
                            self.resolve_reference(*source, d, doc, op, values, visiting)
                        }
                    })
                    .collect::<Result<Vec<_>, String>>()?;
                zs.into_iter()
                    .reduce(if reference == Highest {
                        f64::max
                    } else {
                        f64::min
                    })
                    .ok_or("Choose references for Highest/Lowest")?
            }
        })
    }
    /// Preserve a pre-reference operation without silently changing its toolpath.
    pub fn legacy(document: &Document, operation: &Operation) -> Self {
        let safety = document.setup().heights_for(operation.id());
        let cutting = document
            .geometry
            .bindings
            .get(&operation.id())
            .map(|b| b.effective_heights())
            .unwrap_or_else(|| document.operation_base_cutting_heights(operation));
        let origin = document.setup().origin_position(operation.work_offset())[2];
        let mut settings = Self::default();
        for (p, z) in [
            (HeightPlaneKind::Clearance, safety.clearance),
            (HeightPlaneKind::Retract, safety.retract),
            (HeightPlaneKind::Top, cutting.top),
            (HeightPlaneKind::Bottom, cutting.bottom),
        ] {
            settings.planes[p.index()] = HeightDefinition::new(HeightReference::Origin, z - origin);
        }
        settings.planes[2] = HeightDefinition::new(HeightReference::Disabled, 0.);
        settings
    }
}
pub fn legacy_cutting(op: &Operation, h: &Heights) -> CuttingHeights {
    op.legacy_cutting_heights(h).unwrap_or_else(|| match op {
        Operation::Bore(o) => CuttingHeights {
            top: o.z_top,
            bottom: o.z_bottom,
        },
        Operation::Thread(o) => CuttingHeights {
            top: o.z_top,
            bottom: o.z_bottom,
        },
        Operation::Chamfer(o) => CuttingHeights {
            top: o.top,
            bottom: o.top - o.depth,
        },
        Operation::Engrave(o) => CuttingHeights {
            top: o.top,
            bottom: o.top - o.depth,
        },
        Operation::Carve(o) => CuttingHeights {
            top: o.top,
            bottom: o.top - o.depth,
        },
        Operation::Parallel(_) | Operation::Spiral(_) => CuttingHeights {
            top: h.top_of_stock,
            bottom: h.top_of_stock - 1.0,
        },
        Operation::Adaptive(o) => {
            let top = h.top_of_stock;
            let stepdown = if o.params.stepdown.is_finite() && o.params.stepdown > 0.0 {
                o.params.stepdown
            } else {
                1.0
            };
            CuttingHeights {
                top,
                bottom: top - stepdown,
            }
        }
        _ => unreachable!(),
    })
}
impl Document {
    fn selected_contour_height(&self, op: &Operation) -> Result<f64, String> {
        let Some(binding) = self.geometry.bindings.get(&op.id()) else {
            return Ok(self.operation_base_cutting_heights(op).bottom);
        };
        // Profile face contours and selected edge chains have their own Z plane.
        // A binding's wall interval can extend well below that source plane.
        if matches!(
            binding.recipe,
            crate::ExtractionRecipe::ChamferLoop
                | crate::ExtractionRecipe::ChamferFaces
                | crate::ExtractionRecipe::EngravePath
                | crate::ExtractionRecipe::CarveRegion
                | crate::ExtractionRecipe::BoreCylinders { .. }
                | crate::ExtractionRecipe::ThreadCylinders { .. }
        ) {
            return Ok(binding.heights.top);
        }
        let references: Vec<_> = match &binding.recipe {
            crate::ExtractionRecipe::ProfileEntries { entries }
                if entries.iter().all(|entry| {
                    matches!(
                        entry.kind,
                        crate::ProfileGeometryKind::Chain
                            | crate::ProfileGeometryKind::FaceContours
                    ) && matches!(&entry.source, crate::ProfileGeometrySource::Cad(_))
                }) =>
            {
                entries
                    .iter()
                    .flat_map(|entry| match &entry.source {
                        crate::ProfileGeometrySource::Cad(refs) => refs.clone(),
                        _ => unreachable!(),
                    })
                    .collect()
            }
            crate::ExtractionRecipe::ProfileContours | crate::ExtractionRecipe::PocketBoundary => {
                binding.selected.clone()
            }
            _ => vec![],
        };
        let profile_faces = matches!(op, Operation::Profile(_))
            && matches!(
                binding.recipe,
                crate::ExtractionRecipe::ProfileEntries { .. }
                    | crate::ExtractionRecipe::ProfileContours
            );
        if references.is_empty()
            || references.iter().any(|r| {
                !matches!(r.kind, EntityKind::Edge | EntityKind::Loop)
                    && !(profile_faces && r.kind == EntityKind::Face)
            })
        {
            return Ok(binding.heights.bottom);
        }
        let (min, max) = self.height_selection_bounds(&references)?;
        if (max[2] - min[2]).abs() > 1e-5 {
            return Err(
                "Selected contour(s) must lie on one plane perpendicular to setup Z".into(),
            );
        }
        Ok(max[2])
    }
    fn chamfer_minimum_depth(&self, op: &crate::ChamferOp) -> f64 {
        if let Some(crate::ToolKind::RadiusMill {
            corner_radius,
            tip_length,
        }) = self
            .setup()
            .tools
            .iter()
            .find(|tool| tool.number == op.tool)
            .map(|tool| tool.kind)
        {
            return corner_radius + tip_length;
        }
        let angle = self
            .setup()
            .tools
            .iter()
            .find(|tool| tool.number == op.tool)
            .and_then(|tool| match tool.kind {
                crate::ToolKind::ChamferMill {
                    included_angle_deg, ..
                }
                | crate::ToolKind::VBit {
                    included_angle_deg, ..
                } => Some(included_angle_deg),
                _ => None,
            })
            .unwrap_or(90.);
        op.width / (angle.to_radians() * 0.5).tan()
    }
    /// A chamfer's zero depth means its natural cutter depth, rather than a zero
    /// cutting interval. All other strategies store their target interval directly.
    pub fn operation_base_cutting_heights(&self, op: &Operation) -> CuttingHeights {
        if let Some(wrap) = op.rotary_wrap() {
            let depth = match op {
                Operation::Profile(p) => p.depth,
                Operation::Pocket(p) => p.depth,
                _ => unreachable!(),
            };
            return CuttingHeights {
                top: wrap.surface_z,
                bottom: wrap.surface_z - depth,
            };
        }
        if matches!(
            op,
            Operation::Adaptive(_) | Operation::Parallel(_) | Operation::Spiral(_)
        ) {
            if let Some(binding) = self.geometry.bindings.get(&op.id()) {
                return binding.effective_heights();
            }
        }
        let mut cutting = legacy_cutting(op, &self.setup().heights_for(op.id()));
        if let Operation::Chamfer(chamfer) = op {
            if chamfer.faces.is_empty() {
                cutting.bottom =
                    chamfer.top - chamfer.depth.max(self.chamfer_minimum_depth(chamfer));
            } else {
                cutting.top = chamfer.faces[0].top;
                cutting.bottom = chamfer.faces[0].bottom;
            }
        }
        cutting
    }
    /// Same conservative source bounds used for stock, independent of display mesh.
    pub fn height_model_bounds(&self) -> ([f64; 3], [f64; 3]) {
        self.height_model_bounds_for(self.setup())
    }

    pub fn height_model_bounds_for(&self, setup: &crate::Setup) -> ([f64; 3], [f64; 3]) {
        let mut points = vec![];
        for instance in &self.geometry.instances {
            if !setup.model_instances.contains(&instance.id) {
                continue;
            }
            if let Some(body) = self
                .geometry
                .asset(instance.asset_id)
                .and_then(|a| a.topology.bodies.iter().find(|b| b.id == instance.body_id))
            {
                points.extend(box_corners(body.bounds).map(|p| {
                    setup
                        .frame
                        .to_setup_vector(instance.placement.transform_point(p))
                }));
            }
        }
        for p in self
            .geometry
            .drawing_regions
            .iter()
            .flat_map(|r| r.outer().points())
            .chain(self.geometry.drawing_paths.iter().flat_map(|p| p.points()))
        {
            points.push(setup.frame.to_setup_vector([p.x, p.y, 0.]));
        }
        bounds(&points).unwrap_or(([0.; 3], [0.; 3]))
    }
    /// Conservative radial extent of selected geometry around a line in `frame`.
    /// Analytic cylinders and planar faces with line/circle boundaries avoid the
    /// exaggerated diagonal of a cylindrical model's rectangular bounds.
    fn stock_radial_extent(
        &self,
        setup: &crate::Setup,
        frame: &crate::SetupFrame,
        center: [f64; 2],
    ) -> f64 {
        let radius = |point: [f64; 3]| (point[0] - center[0]).hypot(point[1] - center[1]);
        let mut largest = 0.0_f64;
        for instance in &self.geometry.instances {
            if !setup.model_instances.contains(&instance.id) {
                continue;
            }
            let Some(asset) = self.geometry.asset(instance.asset_id) else {
                continue;
            };
            let topology = &asset.topology;
            let Some(body) = topology
                .bodies
                .iter()
                .find(|body| body.id == instance.body_id)
            else {
                continue;
            };
            let transform =
                |point| frame.to_setup_vector(instance.placement.transform_point(point));
            let bounds_radius = |bounds| {
                box_corners(bounds)
                    .into_iter()
                    .map(|point| radius(transform(point)))
                    .fold(0.0, f64::max)
            };
            let faces: Vec<_> = topology
                .faces
                .iter()
                .filter(|face| face.body_id == instance.body_id)
                .collect();
            if faces.is_empty() {
                largest = largest.max(bounds_radius(body.bounds));
            }
            for face in faces {
                let estimate = match &face.surface {
                    crate::CadSurface::Cylinder {
                        origin,
                        axis,
                        radius: r,
                        ..
                    } => {
                        let direction =
                            frame.to_setup_vector(instance.placement.transform_vector(*axis));
                        // Bound axial displacement even for a slightly tilted cylinder:
                        // a tiny angular error can matter on a very long part.
                        let axial_extent = box_corners(face.bounds)
                            .into_iter()
                            .map(|point| {
                                (0..3)
                                    .map(|i| (point[i] - origin[i]) * axis[i])
                                    .sum::<f64>()
                                    .abs()
                            })
                            .fold(0.0, f64::max);
                        let cylinder_bound = radius(transform(*origin))
                            + r
                            + direction[0].hypot(direction[1]) * axial_extent;
                        cylinder_bound.min(bounds_radius(face.bounds))
                    }
                    crate::CadSurface::Plane { .. } => {
                        let edges: Vec<_> = topology
                            .edges
                            .iter()
                            .filter(|edge| {
                                edge.body_id == instance.body_id && edge.face_ids.contains(&face.id)
                            })
                            .collect();
                        if edges.is_empty() {
                            bounds_radius(face.bounds)
                        } else {
                            let mut r = 0.0_f64;
                            let mut analytic = true;
                            for edge in edges {
                                match &edge.curve {
                                    crate::CadCurve::Line { start, end } => {
                                        r = r
                                            .max(radius(transform(*start)))
                                            .max(radius(transform(*end)))
                                    }
                                    crate::CadCurve::CircleArc {
                                        center: c,
                                        radius: arc_radius,
                                        ..
                                    } => r = r.max(radius(transform(*c)) + arc_radius),
                                    _ => {
                                        analytic = false;
                                        break;
                                    }
                                }
                            }
                            if analytic {
                                r
                            } else {
                                bounds_radius(face.bounds)
                            }
                        }
                    }
                    _ => bounds_radius(face.bounds),
                };
                largest = largest.max(estimate);
            }
        }
        for point in self
            .geometry
            .drawing_regions
            .iter()
            .flat_map(|r| r.outer().points())
            .chain(self.geometry.drawing_paths.iter().flat_map(|p| p.points()))
        {
            largest = largest.max(radius(frame.to_setup_vector([point.x, point.y, 0.])));
        }
        largest
    }

    /// Resolve a round blank in its own frame. Edge references select a direction;
    /// origin-axis references additionally locate the center at the setup origin.
    pub fn round_stock_geometry(
        &self,
        setup: &crate::Setup,
    ) -> Result<Option<RoundStockGeometry>, String> {
        let axis = match setup.stock {
            crate::Stock::FixedRound { axis, .. } | crate::Stock::RelativeRound { axis, .. } => {
                axis
            }
            _ => return Ok(None),
        };
        let references = match axis {
            Some(crate::StockAxis::Edge(edge)) => Some(crate::FrameReferences {
                z: Some(crate::FrameAxisReference {
                    entity: edge.entity(),
                    flipped: false,
                }),
                ..Default::default()
            }),
            Some(crate::StockAxis::Origin { origin_axis }) => {
                let direction = setup.frame.to_world_vector(std::array::from_fn(|i| {
                    if i == origin_axis.index() {
                        1.
                    } else {
                        0.
                    }
                }));
                let mut refs = crate::FrameReferences::default();
                refs.stock_axes[2] = Some(direction);
                Some(refs)
            }
            None => None,
        };
        let frame = match references {
            Some(references) => setup
                .frame
                .resolve_references(&self.geometry, references)
                .map_err(|e| format!("Stock axis: {e}"))?,
            None => setup.frame.clone(),
        };
        let mut oriented = setup.clone();
        oriented.frame = frame.clone();
        let (model_min, model_max) = self.height_model_bounds_for(&oriented);
        let (mut min, mut max) = setup.stock.resolve_bounds(model_min, model_max);
        let center = if matches!(axis, Some(crate::StockAxis::Origin { .. })) {
            let point = frame.to_setup_vector(
                setup
                    .frame
                    .to_world_vector(setup.origin_position(setup.origin_index)),
            );
            [point[0], point[1]]
        } else {
            [
                (model_min[0] + model_max[0]) * 0.5,
                (model_min[1] + model_max[1]) * 0.5,
            ]
        };
        let radius = match setup.stock {
            crate::Stock::FixedRound { diameter, .. } => diameter * 0.5,
            crate::Stock::RelativeRound { radial, .. } => {
                self.stock_radial_extent(setup, &frame, center) + radial
            }
            _ => unreachable!(),
        };
        for i in 0..2 {
            min[i] = center[i] - radius;
            max[i] = center[i] + radius;
        }
        Ok(Some((frame, min, max)))
    }

    /// Enclosing stock bounds in the requested setup's machining axes. Inherited
    /// stock keeps the original blank's enclosure; its removed regions are supplied
    /// by simulation. Resolve directly from the source axes to avoid accumulating
    /// oversized bounding boxes through a chain of rotated setups.
    pub fn stock_bounds_for(&self, setup: &crate::Setup) -> ([f64; 3], [f64; 3]) {
        if let crate::Stock::FromSolid { instance } = setup.stock {
            if let Some((instance, body)) = instance
                .and_then(|id| self.geometry.instance(id))
                .and_then(|instance| {
                    self.geometry
                        .asset(instance.asset_id)
                        .and_then(|a| a.topology.bodies.iter().find(|b| b.id == instance.body_id))
                        .map(|body| (instance, body))
                })
            {
                let points = box_corners(body.bounds).map(|p| {
                    setup
                        .frame
                        .to_setup_vector(instance.placement.transform_point(p))
                });
                return bounds(&points).unwrap_or(([0.; 3], [0.; 3]));
            }
            return ([0.; 3], [0.; 3]);
        }
        match self.round_stock_geometry(setup) {
            Ok(Some((frame, min, max))) => {
                let points = box_corners([min, max])
                    .map(|p| setup.frame.to_setup_vector(frame.to_world_vector(p)));
                return bounds(&points).unwrap_or(([0.; 3], [0.; 3]));
            }
            Err(_) => return ([0.; 3], [0.; 3]),
            Ok(None) => {}
        }
        if setup.stock != crate::Stock::PreviousSetup {
            let (min, max) = self.height_model_bounds_for(setup);
            return setup.stock.resolve_bounds(min, max);
        }
        let Some(at) = self.setups.iter().position(|s| s.id == setup.id) else {
            return self.height_model_bounds_for(setup);
        };
        let Some(source) = self.setups[..at]
            .iter()
            .rev()
            .find(|s| s.stock != crate::Stock::PreviousSetup)
        else {
            return self.height_model_bounds_for(setup);
        };
        let (min, max) = self.stock_bounds_for(source);
        let points = box_corners([min, max])
            .map(|p| setup.frame.to_setup_vector(source.frame.to_world_vector(p)));
        bounds(&points).unwrap_or(([0.; 3], [0.; 3]))
    }

    pub fn height_definition_bounds(
        &self,
        d: &HeightDefinition,
    ) -> Result<([f64; 3], [f64; 3]), String> {
        let mut points = vec![];
        if !d.entities.is_empty() {
            let (min, max) = self.height_selection_bounds(&d.entities)?;
            points.extend([min, max]);
        }
        for drawing in &d.drawings {
            let matches = |source: &[cam_geo::Point]| {
                source
                    .iter()
                    .map(|p| [p.x, p.y])
                    .eq(drawing.points.iter().copied())
            };
            let exists = if drawing.closed {
                self.geometry.drawing_regions.iter().any(|r| {
                    std::iter::once(r.outer())
                        .chain(r.holes())
                        .any(|c| matches(c.points()))
                })
            } else {
                self.geometry
                    .drawing_paths
                    .iter()
                    .any(|p| matches(p.points()))
            };
            if !exists {
                return Err("Drawing height reference changed; pick it again".into());
            }
            points.extend(
                drawing
                    .points
                    .iter()
                    .map(|[x, y]| self.setup().frame.to_setup_vector([*x, *y, 0.])),
            );
        }
        bounds(&points).ok_or_else(|| "Pick reference geometry in the viewport".into())
    }
    pub fn height_selection_bounds(
        &self,
        refs: &[EntityRef],
    ) -> Result<([f64; 3], [f64; 3]), String> {
        let mut points = vec![];
        for r in refs {
            if !self.geometry.entity_exists(r) {
                return Err(
                    "Height reference geometry is missing or changed; pick it again".into(),
                );
            }
            let instance = self.geometry.instance(r.instance_id).unwrap();
            let t = &self.geometry.asset(r.asset_id).unwrap().topology;
            let transform = |p| {
                self.setup()
                    .frame
                    .to_setup_vector(instance.placement.transform_point(p))
            };
            let face_id = if r.kind == EntityKind::Loop {
                t.loops.iter().find(|l| l.id == r.id).map(|l| l.face_id)
            } else {
                None
            };
            match r.kind {
                EntityKind::Body | EntityKind::Silhouette => points.extend(
                    box_corners(t.bodies.iter().find(|b| b.id == r.body_id).unwrap().bounds)
                        .map(transform),
                ),
                EntityKind::Face | EntityKind::Loop => {
                    let face = t
                        .faces
                        .iter()
                        .find(|f| f.id == face_id.unwrap_or(r.id))
                        .unwrap();
                    if let crate::CadSurface::Plane { origin, normal } = face.surface {
                        let n = self
                            .setup()
                            .frame
                            .to_setup_vector(instance.placement.transform_vector(normal));
                        if n[0].abs() < 1e-7 && n[1].abs() < 1e-7 {
                            points.push(transform(origin));
                            continue;
                        }
                    }
                    points.extend(box_corners(face.bounds).map(transform));
                }
                EntityKind::Vertex => points.push(transform(
                    t.vertices.iter().find(|v| v.id == r.id).unwrap().point,
                )),
                EntityKind::Edge => match &t.edges.iter().find(|e| e.id == r.id).unwrap().curve {
                    crate::CadCurve::Line { start, end } => {
                        points.extend([transform(*start), transform(*end)])
                    }
                    crate::CadCurve::CircleArc {
                        center,
                        normal,
                        x_axis,
                        radius,
                        start_angle,
                        end_angle,
                    } => {
                        let y = [
                            normal[1] * x_axis[2] - normal[2] * x_axis[1],
                            normal[2] * x_axis[0] - normal[0] * x_axis[2],
                            normal[0] * x_axis[1] - normal[1] * x_axis[0],
                        ];
                        let xz = self
                            .setup()
                            .frame
                            .to_setup_vector(instance.placement.transform_vector(*x_axis))[2];
                        let yz = self
                            .setup()
                            .frame
                            .to_setup_vector(instance.placement.transform_vector(y))[2];
                        let mut angles = vec![*start_angle, *end_angle];
                        let peak = yz.atan2(xz);
                        let lo = start_angle.min(*end_angle);
                        let hi = start_angle.max(*end_angle);
                        for base in [peak, peak + std::f64::consts::PI] {
                            let turn = ((lo - base) / std::f64::consts::TAU).ceil();
                            let angle = base + turn * std::f64::consts::TAU;
                            if angle <= hi {
                                angles.push(angle);
                            }
                        }
                        points.extend(angles.into_iter().map(|a| {
                            transform(std::array::from_fn(|i| {
                                center[i] + radius * (x_axis[i] * a.cos() + y[i] * a.sin())
                            }))
                        }));
                    }
                    crate::CadCurve::BezierSpans { spans } => {
                        // Control hull bounds conservatively enclose every point
                        // of a positive-weight rational spline in any setup.
                        points.extend(spans.iter().flat_map(|s| &s.poles).map(|p| transform(*p)));
                    }
                    _ => return Err("This edge cannot define an exact height reference".into()),
                },
            }
        }
        bounds(&points).ok_or_else(|| "Pick reference geometry in the viewport".into())
    }
}
fn box_corners(b: [[f64; 3]; 2]) -> [[f64; 3]; 8] {
    std::array::from_fn(|i| std::array::from_fn(|a| b[usize::from(i & (1 << a) != 0)][a]))
}
fn bounds(points: &[[f64; 3]]) -> Option<([f64; 3], [f64; 3])> {
    if points.is_empty() {
        None
    } else {
        Some((
            std::array::from_fn(|a| points.iter().map(|p| p[a]).fold(f64::INFINITY, f64::min)),
            std::array::from_fn(|a| {
                points
                    .iter()
                    .map(|p| p[a])
                    .fold(f64::NEG_INFINITY, f64::max)
            }),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::*;
    fn doc() -> Document {
        let mut doc:Document=serde_json::from_value(serde_json::json!({"schema_version":18,"setups":[{"id":1,"name":"reference test","heights":{"clearance":5.,"retract":2.,"top_of_stock":0.},"stock":{"BoundingBox":{"x_offset":0.,"y_offset":0.,"top":10.,"thickness":15.}},"tools":[],"operations":[]}]})).unwrap();
        doc.setup_mut().operations.push(Operation::Drill(DrillOp {
            parameter_expressions: Default::default(),
            id: 1,
            tool: 1,
            points: vec![[0., 0.]],
            depth: 4.,
            start_offset: 0.,
            peck: None,
            dwell: None,
            spindle_rpm: 1000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            feed: 100.,
        }));
        doc
    }
    #[test]
    fn relative_round_stock_fits_cylindrical_faces_without_bbox_corner_inflation() {
        let mut document = doc();
        let cylinder = CadFace {
            id: 2,
            body_id: 1,
            surface: CadSurface::Cylinder {
                origin: [0., 0., 0.],
                axis: [0., 0., 1.],
                radius: 15.,
                inward: false,
            },
            loop_ids: vec![],
            bounds: [[-15., -15., 0.], [15., 15., 40.]],
        };
        let mut faces = vec![cylinder];
        let mut edges = vec![CadEdge {
            id: 9,
            body_id: 1,
            vertices: [1, 2],
            face_ids: vec![2],
            curve: CadCurve::Line {
                start: [15., 0., 0.],
                end: [15., 0., 40.],
            },
        }];
        for (id, z) in [(3, 0.), (4, 40.)] {
            faces.push(CadFace {
                id,
                body_id: 1,
                surface: CadSurface::Plane {
                    origin: [0., 0., z],
                    normal: [0., 0., 1.],
                },
                loop_ids: vec![],
                bounds: [[-15., -15., z], [15., 15., z]],
            });
            edges.push(CadEdge {
                id: 10 + id,
                body_id: 1,
                vertices: [1, 2],
                face_ids: vec![id],
                curve: CadCurve::CircleArc {
                    center: [0., 0., z],
                    normal: [0., 0., 1.],
                    x_axis: [1., 0., 0.],
                    radius: 15.,
                    start_angle: 0.,
                    end_angle: std::f64::consts::PI,
                },
            });
        }
        document.geometry.assets.push(GeometryAsset {
            id: 1,
            revision: 1,
            name: "Round part".into(),
            source_format: CadFormat::Step,
            source_data: vec![].into(),
            source_units: SourceUnits::Millimetres,
            backend: "test".into(),
            import_options: None,
            import_diagnostics: vec![],
            source_fingerprint: String::new(),
            topology: TopologySnapshot {
                bodies: vec![CadBody {
                    id: 1,
                    name: "Body".into(),
                    valid_solid: true,
                    bounds: [[-15., -15., 0.], [15., 15., 40.]],
                }],
                faces,
                edges,
                ..Default::default()
            }
            .into(),
        });
        document.geometry.instances.push(GeometryInstance {
            id: 1,
            asset_id: 1,
            body_id: 1,
            name: "Body".into(),
            placement: Default::default(),
            visible: true,
        });
        let setup = document.setup_mut();
        setup.model_instances = vec![1];
        setup.stock = Stock::RelativeRound {
            radial: 0.,
            inner_diameter: 0.,
            bottom: 0.,
            top: 0.,
            tube: false,
            axis: Some(StockAxis::Edge(StockAxisReference {
                asset_id: 1,
                revision: 1,
                instance_id: 1,
                body_id: 1,
                edge_id: 9,
            })),
        };
        let (_, lo, hi) = document
            .round_stock_geometry(document.setup())
            .unwrap()
            .unwrap();
        assert_eq!((lo, hi), ([-15., -15., 0.], [15., 15., 40.]));
        if let Stock::RelativeRound { radial, .. } = &mut document.setup_mut().stock {
            *radial = 2.;
        }
        let (_, lo, hi) = document
            .round_stock_geometry(document.setup())
            .unwrap()
            .unwrap();
        assert_eq!((lo, hi), ([-17., -17., 0.], [17., 17., 40.]));
        document.setup_mut().origin = [3., 0., 0.];
        if let Stock::RelativeRound { axis, .. } = &mut document.setup_mut().stock {
            *axis = Some(StockAxis::Origin {
                origin_axis: StockOriginAxis::Z,
            });
        }
        let (_, lo, hi) = document
            .round_stock_geometry(document.setup())
            .unwrap()
            .unwrap();
        assert_eq!((lo, hi), ([-17., -20., 0.], [23., 20., 40.]));
    }

    #[test]
    fn adaptive_height_helpers_are_safe_before_and_after_binding() {
        let mut doc = doc();
        let mut op = AdaptiveOp {
            parameter_expressions: Default::default(),
            id: 2,
            tool: 1,
            spindle_rpm: 1000.,
            tool_preset: None,
            cooling: Default::default(),
            entry_feeds: Default::default(),
            work_offset: 1,
            params: Default::default(),
        };
        op.params.stepdown = 2.0;
        let unbound = Operation::Adaptive(op);
        let heights = doc.operation_base_cutting_heights(&unbound);
        assert!(heights.validate().is_ok());
        assert_eq!(heights.bottom, heights.top - 2.0);

        doc.geometry.bindings.insert(
            unbound.id(),
            GeometryBinding {
                selected: vec![EntityRef {
                    asset_id: 1,
                    revision: 1,
                    instance_id: 1,
                    body_id: 1,
                    kind: EntityKind::Body,
                    id: 1,
                }],
                recipe: ExtractionRecipe::AdaptiveClearing,
                heights: CuttingHeights {
                    top: 12.,
                    bottom: 3.,
                },
                overrides: Default::default(),
                status: ResolutionStatus::Resolved,
                resolved_fingerprint: None,
            },
        );
        assert_eq!(doc.operation_base_cutting_heights(&unbound).bottom, 3.);
        assert_eq!(legacy_cutting(&unbound, &doc.setup().heights), heights);
    }
    #[test]
    fn screenshot_defaults_resolve_dependencies_and_follow_stock_changes() {
        let mut doc = doc();
        let settings = OperationHeightSettings::default();
        let resolved = settings.resolve(&doc, &doc.setup().operations[0]).unwrap();
        assert_eq!(
            (
                resolved.clearance,
                resolved.retract,
                resolved.feed,
                resolved.top,
                resolved.bottom
            ),
            (25., 15., Some(15.), 10., -4.)
        );
        doc.setup_mut().stock = Stock::BoundingBox {
            x_offset: 0.,
            y_offset: 0.,
            top: 12.,
            thickness: 15.,
        };
        let next = settings.resolve(&doc, &doc.setup().operations[0]).unwrap();
        assert_eq!(
            (
                next.clearance,
                next.retract,
                next.feed,
                next.top,
                next.bottom
            ),
            (27., 17., Some(17.), 12., -4.)
        );
    }
    #[test]
    fn rejects_indirect_cycles_disabled_dependencies_and_overflow() {
        let doc = doc();
        let op = &doc.setup().operations[0];
        let mut settings = OperationHeightSettings::default();
        settings.definition_mut(HeightPlaneKind::Retract).reference =
            HeightReference::Plane(HeightPlaneKind::Clearance);
        assert!(settings.resolve(&doc, op).unwrap_err().contains("Circular"));
        settings = OperationHeightSettings::default();
        settings.definition_mut(HeightPlaneKind::Feed).reference = HeightReference::Disabled;
        settings
            .definition_mut(HeightPlaneKind::Clearance)
            .reference = HeightReference::Plane(HeightPlaneKind::Feed);
        assert!(settings.resolve(&doc, op).unwrap_err().contains("disabled"));
        settings = OperationHeightSettings::default();
        settings.definition_mut(HeightPlaneKind::Retract).offset = f64::MAX;
        settings.definition_mut(HeightPlaneKind::Clearance).offset = f64::MAX;
        assert!(settings.resolve(&doc, op).unwrap_err().contains("finite"));
    }
    #[test]
    fn aggregate_references_are_evaluated_before_offset_and_origin_is_operation_datum() {
        let mut doc = doc();
        doc.setup_mut().extra_origins.push(Origin {
            index: 2,
            position: [0., 0., 3.],
        });
        doc.setup_mut().operations[0].set_work_offset(2);
        let op = &doc.setup().operations[0];
        let mut settings = OperationHeightSettings::default();
        let top = settings.definition_mut(HeightPlaneKind::Top);
        top.reference = HeightReference::Highest;
        top.sources = vec![HeightReference::ModelTop, HeightReference::Origin];
        top.offset = 2.;
        assert_eq!(settings.resolve(&doc, op).unwrap().top, 5.);
        let bottom = settings.definition_mut(HeightPlaneKind::Bottom);
        bottom.reference = HeightReference::Lowest;
        bottom.sources = vec![
            HeightReference::StockBottom,
            HeightReference::SelectedContours,
        ];
        bottom.offset = -1.;
        assert_eq!(settings.resolve(&doc, op).unwrap().bottom, -6.);
    }
    fn add_body(doc: &mut Document) -> EntityRef {
        let topology = TopologySnapshot {
            bodies: vec![CadBody {
                id: 1,
                name: "fixture".into(),
                valid_solid: true,
                bounds: [[0., 0., -2.], [20., 10., 7.]],
            }],
            vertices: vec![CadVertex {
                id: 2,
                body_id: 1,
                point: [0., 0., 6.],
            }],
            ..Default::default()
        };
        doc.geometry.assets.push(GeometryAsset {
            import_options: None,
            id: 1,
            revision: 1,
            name: "fixture".into(),
            source_format: CadFormat::Step,
            source_data: vec![].into(),
            source_units: SourceUnits::Millimetres,
            backend: "height test".into(),
            import_diagnostics: vec![],
            source_fingerprint: String::new(),
            topology: topology.into(),
        });
        doc.geometry.instances.push(GeometryInstance {
            id: 1,
            asset_id: 1,
            body_id: 1,
            name: "fixture".into(),
            placement: RigidTransform::default(),
            visible: true,
        });
        EntityRef {
            asset_id: 1,
            revision: 1,
            instance_id: 1,
            body_id: 1,
            kind: EntityKind::Body,
            id: 1,
        }
    }
    #[test]
    fn pocket_contour_height_follows_placement_and_setup_axes_independently_of_wcs() {
        let mut doc = doc();
        let mut reference = add_body(&mut doc);
        reference.kind = EntityKind::Loop;
        reference.id = 4;
        let topology = std::sync::Arc::make_mut(&mut doc.geometry.assets[0].topology);
        topology.faces.push(CadFace {
            id: 3,
            body_id: 1,
            surface: CadSurface::Plane {
                origin: [0.; 3],
                normal: [0., 0., -1.],
            },
            loop_ids: vec![4],
            bounds: [[0.; 3], [20., 10., 0.]],
        });
        topology.loops.push(CadLoop {
            id: 4,
            body_id: 1,
            face_id: 3,
            edge_uses: vec![],
            is_outer: false,
        });
        doc.geometry.bindings.insert(
            1,
            GeometryBinding {
                selected: vec![reference],
                recipe: ExtractionRecipe::PocketBoundary,
                heights: CuttingHeights {
                    top: 0.,
                    bottom: -4.,
                },
                overrides: GeometryOverrides::default(),
                status: ResolutionStatus::Resolved,
                resolved_fingerprint: None,
            },
        );
        let mut settings = OperationHeightSettings::default();
        *settings.definition_mut(HeightPlaneKind::Top) =
            HeightDefinition::new(HeightReference::SelectedContours, 0.);
        *settings.definition_mut(HeightPlaneKind::Bottom) =
            HeightDefinition::new(HeightReference::Plane(HeightPlaneKind::Top), -4.);
        doc.setup_mut().stock = Stock::BoundingBox {
            x_offset: 0.,
            y_offset: 0.,
            top: 40.,
            thickness: 45.,
        };
        let resolve = |doc: &Document| settings.resolve(doc, &doc.setup().operations[0]).unwrap();
        doc.geometry.instances[0].placement.translation = [0., 0., 20.];
        assert_eq!(resolve(&doc).top, 20.);
        assert_eq!(resolve(&doc).bottom, 16.);
        doc.setup_mut().origin[2] = 20.;
        assert_eq!(resolve(&doc).top - doc.setup().origin[2], 0.);
        assert_eq!(resolve(&doc).bottom - doc.setup().origin[2], -4.);
        doc.geometry.instances[0].placement.translation = [0., 30., 20.];
        doc.setup_mut().frame.world_from_setup.rotation =
            [[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]];
        // This loop is perpendicular to world Z, so rotating setup Z into world Y
        // must reject it rather than reuse the cached, apparently planar height.
        assert!(settings
            .resolve(&doc, &doc.setup().operations[0])
            .unwrap_err()
            .contains("one plane"));
        doc.setup_mut().frame.world_from_setup.rotation = RigidTransform::default().rotation;
        doc.geometry.bindings.get_mut(&1).unwrap().selected[0].revision = 2;
        assert!(settings
            .resolve(&doc, &doc.setup().operations[0])
            .unwrap_err()
            .contains("missing or changed"));
    }
    #[test]
    fn selected_chain_height_uses_source_plane_in_setup_axes_instead_of_cached_wall_bottom() {
        let mut doc = doc();
        let mut edge = add_body(&mut doc);
        edge.kind = EntityKind::Edge;
        edge.id = 3;
        std::sync::Arc::make_mut(&mut doc.geometry.assets[0].topology)
            .edges
            .push(CadEdge {
                id: 3,
                body_id: 1,
                vertices: [2, 2],
                face_ids: vec![],
                curve: CadCurve::BezierSpans {
                    spans: vec![CadBezierSpan {
                        poles: vec![[0., 30., 4.], [2., 30., 3.], [4., 30., 0.]],
                        weights: vec![1.; 3],
                    }],
                },
            });
        doc.setup_mut().frame.world_from_setup.rotation =
            [[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]];
        doc.setup_mut().stock = Stock::BoundingBox {
            x_offset: 0.,
            y_offset: 0.,
            top: 40.,
            thickness: 45.,
        };
        let profile: Operation = serde_json::from_value(serde_json::json!({"Profile": {
            "id": 1, "tool": 1, "chain": {"points": [{"x":0.,"y":-4.},{"x":4.,"y":0.}]},
            "closed": false, "side": "Outside", "comp": "Computed", "depth": 2.,
            "stepdown": 2., "feed": 100., "plunge_feed": 100., "start": null,
            "lead_in": "None", "lead_out": "None", "plunge": "Straight"
        }}))
        .unwrap();
        doc.setup_mut().operations = vec![profile];
        let mut entry = ProfileGeometryEntry::new(ProfileGeometryKind::Chain, "Chain 1".into());
        entry.source = ProfileGeometrySource::Cad(vec![edge.clone()]);
        doc.geometry.bindings.insert(
            1,
            GeometryBinding {
                selected: vec![edge],
                recipe: ExtractionRecipe::ProfileEntries {
                    entries: vec![entry],
                },
                heights: CuttingHeights {
                    top: 30.,
                    bottom: 28.,
                },
                overrides: GeometryOverrides {
                    top: None,
                    bottom: Some(27.),
                },
                status: ResolutionStatus::Resolved,
                resolved_fingerprint: None,
            },
        );
        let mut settings = OperationHeightSettings::default();
        let resolve = |doc: &Document, settings: &OperationHeightSettings| {
            settings
                .resolve(doc, &doc.setup().operations[0])
                .unwrap()
                .bottom
        };
        assert_eq!(resolve(&doc, &settings), 30.);
        settings.definition_mut(HeightPlaneKind::Bottom).offset = -0.75;
        assert_eq!(resolve(&doc, &settings), 29.25);
        doc.geometry.instances[0].placement.translation[1] = 2.;
        assert_eq!(resolve(&doc, &settings), 31.25);
        // The legacy connected-edge recipe uses the same source plane.
        doc.geometry.bindings.get_mut(&1).unwrap().recipe = ExtractionRecipe::ProfileContours;
        assert_eq!(resolve(&doc, &settings), 31.25);
        let reopened: Document =
            serde_json::from_str(&serde_json::to_string(&doc).unwrap()).unwrap();
        assert_eq!(resolve(&reopened, &settings), 31.25);
        std::sync::Arc::make_mut(&mut doc.geometry.assets[0].topology).edges[0].curve =
            CadCurve::Line {
                start: [0., 30., 0.],
                end: [4., 31., 0.],
            };
        assert!(settings
            .resolve(&doc, &doc.setup().operations[0])
            .unwrap_err()
            .contains("one plane"));
        doc.geometry.bindings.get_mut(&1).unwrap().selected[0].revision = 2;
        assert!(settings
            .resolve(&doc, &doc.setup().operations[0])
            .unwrap_err()
            .contains("missing or changed"));
    }
    #[test]
    fn fixture_and_geometry_references_follow_placement_and_reject_stale_identity() {
        let mut doc = doc();
        let body = add_body(&mut doc);
        let mut settings = OperationHeightSettings::default();
        let top = settings.definition_mut(HeightPlaneKind::Top);
        top.reference = HeightReference::FixtureTop;
        top.entities = vec![body.clone()];
        top.offset = 1.;
        assert_eq!(
            settings
                .resolve(&doc, &doc.setup().operations[0])
                .unwrap()
                .top,
            8.
        );
        doc.geometry.instances[0].placement.translation[2] = 1.;
        assert_eq!(
            settings
                .resolve(&doc, &doc.setup().operations[0])
                .unwrap()
                .top,
            9.
        );
        let top = settings.definition_mut(HeightPlaneKind::Top);
        top.reference = HeightReference::Selection;
        top.entities[0].kind = EntityKind::Vertex;
        top.entities[0].id = 2;
        top.offset = 0.;
        assert_eq!(
            settings
                .resolve(&doc, &doc.setup().operations[0])
                .unwrap()
                .top,
            7.
        );
        doc.geometry.instances[0].placement.translation[2] = 0.;
        doc.setup_mut().frame.world_from_setup.rotation =
            [[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]];
        assert_eq!(
            settings
                .resolve(&doc, &doc.setup().operations[0])
                .unwrap()
                .top,
            0.
        );
        settings.definition_mut(HeightPlaneKind::Top).entities[0].revision = 2;
        assert!(settings
            .resolve(&doc, &doc.setup().operations[0])
            .unwrap_err()
            .contains("missing or changed"));
    }
    #[test]
    fn circular_edge_height_limits_are_exact_even_for_unwrapped_curve_parameters() {
        let mut doc = doc();
        let mut edge = add_body(&mut doc);
        edge.kind = EntityKind::Edge;
        edge.id = 3;
        let start = 100.0 * std::f64::consts::TAU + std::f64::consts::PI + 0.1;
        let end = 101.0 * std::f64::consts::TAU + 0.2;
        std::sync::Arc::make_mut(&mut doc.geometry.assets[0].topology)
            .edges
            .push(CadEdge {
                id: 3,
                body_id: 1,
                vertices: [2, 2],
                face_ids: vec![],
                curve: CadCurve::CircleArc {
                    center: [0.; 3],
                    normal: [0., 1., 0.],
                    x_axis: [1., 0., 0.],
                    radius: 4.,
                    start_angle: start,
                    end_angle: end,
                },
            });
        let (_, max) = doc.height_selection_bounds(&[edge]).unwrap();
        assert!((max[2] - 4.).abs() < 1e-10);
    }
}
