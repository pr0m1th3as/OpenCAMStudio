//! Geometry for setup stock defined independently of the machining model.
use cam_model::Stock;
use cam_sim::SurfaceMesh;

/// A Z-axis round blank in setup coordinates. The annular caps are triangulated
/// explicitly, so rasterization retains the central void in tube stock.
pub(crate) fn round_mesh(stock: Stock, min: [f64; 3], max: [f64; 3]) -> Option<SurfaceMesh> {
    let inner = match stock {
        Stock::FixedRound {
            inner_diameter,
            tube,
            ..
        }
        | Stock::RelativeRound {
            inner_diameter,
            tube,
            ..
        } => {
            if tube {
                inner_diameter / 2.
            } else {
                0.
            }
        }
        _ => return None,
    };
    let r = (max[0] - min[0]) / 2.;
    let center = [(max[0] + min[0]) / 2., (max[1] + min[1]) / 2.];
    if !r.is_finite()
        || r <= 0.
        || !inner.is_finite()
        || inner < 0.
        || inner >= r
        || max[2] <= min[2]
    {
        return None;
    }
    let mut mesh = SurfaceMesh::default();
    let mut tri = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| {
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let len = n[0].hypot(n[1]).hypot(n[2]);
        if len < 1e-12 {
            return;
        }
        let base = mesh.positions.len() as u32;
        mesh.positions
            .extend([a, b, c].map(|p| p.map(|v| v as f32)));
        mesh.normals.extend([n.map(|v| (v / len) as f32); 3]);
        mesh.machined.extend([false; 3]);
        mesh.operations.extend([None; 3]);
        mesh.indices.extend([base, base + 1, base + 2]);
    };
    let p = |radius: f64, angle: f64, z: f64| {
        [
            center[0] + radius * angle.cos(),
            center[1] + radius * angle.sin(),
            z,
        ]
    };
    const SEGMENTS: usize = 128;
    for i in 0..SEGMENTS {
        let a = std::f64::consts::TAU * i as f64 / SEGMENTS as f64;
        let b = std::f64::consts::TAU * (i + 1) as f64 / SEGMENTS as f64;
        let (oa, ob, oc, od) = (
            p(r, a, min[2]),
            p(r, b, min[2]),
            p(r, a, max[2]),
            p(r, b, max[2]),
        );
        let (ia, ib, ic, id) = (
            p(inner, a, min[2]),
            p(inner, b, min[2]),
            p(inner, a, max[2]),
            p(inner, b, max[2]),
        );
        tri(oa, ob, od);
        tri(oa, od, oc);
        if inner > 0. {
            tri(ib, ia, ic);
            tri(ib, ic, id);
        }
        tri(oc, od, id);
        tri(oc, id, ic);
        tri(ob, oa, ia);
        tri(ob, ia, ib);
    }
    Some(mesh)
}

/// Build the same oriented blank for the viewport and material simulation.
pub(crate) fn oriented_round_mesh(
    document: &cam_model::Document,
    setup: &cam_model::Setup,
) -> Result<Option<SurfaceMesh>, String> {
    let Some((frame, min, max)) = document.round_stock_geometry(setup)? else {
        return Ok(None);
    };
    let mut mesh = round_mesh(setup.stock, min, max).ok_or("Invalid round stock dimensions")?;
    for point in &mut mesh.positions {
        *point = setup
            .frame
            .to_setup_vector(frame.to_world_vector(point.map(f64::from)))
            .map(|v| v as f32);
    }
    for normal in &mut mesh.normals {
        *normal = setup
            .frame
            .to_setup_vector(frame.to_world_vector(normal.map(f64::from)))
            .map(|v| v as f32);
    }
    Ok(Some(mesh))
}

/// Triangulated selected solid in setup coordinates, preserving face winding.
pub(crate) fn solid_mesh(
    document: &cam_model::Document,
    setup: &cam_model::Setup,
) -> Option<SurfaceMesh> {
    let Stock::FromSolid { instance: Some(id) } = setup.stock else {
        return None;
    };
    let instance = document.geometry.instance(id)?;
    let asset = document.geometry.asset(instance.asset_id)?;
    if !asset
        .topology
        .bodies
        .iter()
        .any(|b| b.id == instance.body_id && b.valid_solid)
    {
        return None;
    }
    let source = &asset.topology.mesh;
    let mut mesh = SurfaceMesh::default();
    for (face, indices) in source.triangle_faces.iter().zip(&source.triangles) {
        if !asset
            .topology
            .faces
            .iter()
            .any(|f| f.id == *face && f.body_id == instance.body_id)
        {
            continue;
        }
        let points = indices.map(|i| {
            setup.frame.to_setup_vector(
                instance
                    .placement
                    .transform_point(source.vertices[i as usize]),
            )
        });
        let u = std::array::from_fn::<_, 3, _>(|i| points[1][i] - points[0][i]);
        let v = std::array::from_fn::<_, 3, _>(|i| points[2][i] - points[0][i]);
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let length = n[0].hypot(n[1]).hypot(n[2]);
        if length < 1e-12 {
            continue;
        }
        let base = mesh.positions.len() as u32;
        mesh.positions.extend(points.map(|p| p.map(|v| v as f32)));
        mesh.normals.extend([n.map(|v| (v / length) as f32); 3]);
        mesh.machined.extend([false; 3]);
        mesh.operations.extend([None; 3]);
        mesh.indices.extend([base, base + 1, base + 2]);
    }
    (!mesh.indices.is_empty()).then_some(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_sim::Heightfield;

    #[test]
    fn selected_solid_uses_its_body_and_placement() {
        use cam_model::*;
        let app = crate::AppController::new(crate::default_machine());
        let mut document = Document::new(app.inspection_setup().clone());
        let blank = Stock::FixedRound {
            diameter: 10.,
            inner_diameter: 4.,
            length: 4.,
            position: 0.5,
            tube: true,
            axis: None,
        };
        let source = round_mesh(blank, [-5., -5., -4.], [5., 5., 0.]).unwrap();
        let triangles: Vec<[u32; 3]> = source
            .indices
            .as_chunks::<3>()
            .0
            .iter()
            .map(|c| [c[0], c[1], c[2]])
            .collect();
        document.geometry.assets.push(GeometryAsset {
            id: 1,
            revision: 1,
            name: "Blank".into(),
            source_format: CadFormat::Step,
            source_path: None,
            source_data: Vec::new().into(),
            source_units: SourceUnits::Millimetres,
            backend: "test".into(),
            import_options: None,
            import_diagnostics: vec![],
            source_fingerprint: String::new(),
            topology: TopologySnapshot {
                bodies: vec![CadBody {
                    id: 1,
                    name: "Blank".into(),
                    valid_solid: true,
                    bounds: [[-5., -5., -4.], [5., 5., 0.]],
                }],
                faces: vec![CadFace {
                    id: 1,
                    body_id: 1,
                    surface: CadSurface::Plane {
                        origin: [0.; 3],
                        normal: [0., 0., 1.],
                    },
                    loop_ids: vec![],
                    bounds: [[-5., -5., -4.], [5., 5., 0.]],
                }],
                mesh: CadMesh {
                    vertices: source.positions.iter().map(|p| p.map(f64::from)).collect(),
                    triangle_faces: vec![1; triangles.len()],
                    triangles,
                },
                ..Default::default()
            }
            .into(),
        });
        document.geometry.instances.push(GeometryInstance {
            id: 1,
            asset_id: 1,
            body_id: 1,
            name: "Blank".into(),
            placement: RigidTransform {
                translation: [10., 0., 0.],
                ..Default::default()
            },
            visible: true,
        });
        document.setup_mut().stock = Stock::FromSolid { instance: Some(1) };
        let bounds = document.stock_bounds_for(document.setup());
        assert_eq!(bounds, ([5., -5., -4.], [15., 5., 0.]));
        let mesh = solid_mesh(&document, document.setup()).unwrap();
        let field =
            Heightfield::from_stock_mesh_controlled(&mesh, bounds.0, bounds.1, 0.1, &|| false)
                .unwrap();
        assert_eq!(field.sample(10., 0.), -4.);
        assert_eq!(field.sample(13., 0.), 0.);

        // The selected edge drives both the enclosure and the simulated blank.
        std::sync::Arc::make_mut(&mut document.geometry.assets[0].topology)
            .edges
            .push(CadEdge {
                id: 7,
                body_id: 1,
                vertices: [1, 2],
                face_ids: vec![],
                curve: CadCurve::Line {
                    start: [-5., 0., -2.],
                    end: [5., 0., -2.],
                },
            });
        document.setup_mut().model_instances = vec![1];
        document.setup_mut().stock = Stock::FixedRound {
            diameter: 6.,
            inner_diameter: 0.,
            length: 10.,
            position: 0.5,
            tube: false,
            axis: Some(StockAxis::Edge(StockAxisReference {
                asset_id: 1,
                revision: 1,
                instance_id: 1,
                body_id: 1,
                edge_id: 7,
            })),
        };
        let bounds = document.stock_bounds_for(document.setup());
        assert_eq!(bounds, ([5., -3., -5.], [15., 3., 1.]));
        let mesh = oriented_round_mesh(&document, document.setup())
            .unwrap()
            .unwrap();
        let field =
            Heightfield::from_stock_mesh_controlled(&mesh, bounds.0, bounds.1, 0.1, &|| false)
                .unwrap();
        assert!(field.sample(10., 0.) > 0.9);
        assert!(field.sample(10., 2.95) < -1.);
        if let Stock::FixedRound {
            inner_diameter,
            tube,
            ..
        } = &mut document.setup_mut().stock
        {
            *inner_diameter = 4.;
            *tube = true;
        }
        let mesh = oriented_round_mesh(&document, document.setup())
            .unwrap()
            .unwrap();
        let mut tube =
            Heightfield::from_stock_mesh_controlled(&mesh, bounds.0, bounds.1, 0.1, &|| false)
                .unwrap();
        tube.lower_rect([9., -0.5], [11., 0.5], -2.);
        assert!(
            tube.sample(10., 0.) < -3.9,
            "a horizontal tube must retain its hollow center"
        );
        let encoded = serde_json::to_string(&document.setup().stock).unwrap();
        assert_eq!(
            serde_json::from_str::<Stock>(&encoded).unwrap(),
            document.setup().stock
        );
        document.geometry.instances[0].placement.rotation =
            [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]];
        let bounds = document.stock_bounds_for(document.setup());
        assert_eq!(bounds, ([7., -5., -5.], [13., 5., 1.]));
        std::sync::Arc::make_mut(&mut document.geometry.assets[0].topology).edges[0].curve =
            CadCurve::Line {
                start: [-5., 0., 1.],
                end: [5., 0., 1.],
            };
        let bounds = document.stock_bounds_for(document.setup());
        assert_eq!(
            bounds,
            ([7., -5., -5.], [13., 5., 1.]),
            "edge selection controls direction while stock remains centered on the model"
        );
        document.setup_mut().origin = [0., 0., 1.];
        if let Stock::FixedRound { axis, .. } = &mut document.setup_mut().stock {
            *axis = Some(StockAxis::Origin {
                origin_axis: StockOriginAxis::X,
            });
        }
        let bounds = document.stock_bounds_for(document.setup());
        assert_eq!(
            bounds,
            ([5., -3., -2.], [15., 3., 4.]),
            "origin-axis stock is centered on the setup origin"
        );
        let saved = serde_json::to_string(&document.setup().stock).unwrap();
        assert_eq!(
            serde_json::from_str::<Stock>(&saved).unwrap(),
            document.setup().stock
        );
        if let Stock::FixedRound { axis, .. } = &mut document.setup_mut().stock {
            *axis = Some(StockAxis::Edge(StockAxisReference {
                asset_id: 1,
                revision: 1,
                instance_id: 1,
                body_id: 1,
                edge_id: 7,
            }));
        }
        document.geometry.assets[0].revision += 1;
        assert!(document.round_stock_geometry(document.setup()).is_err());
    }

    #[test]
    fn round_blank_rasterization_preserves_bore_and_outer_boundary() {
        let bounds = ([-5., -5., -4.], [5., 5., 0.]);
        let tube = Stock::FixedRound {
            diameter: 10.,
            inner_diameter: 4.,
            length: 4.,
            position: 0.5,
            tube: true,
            axis: None,
        };
        let mesh = round_mesh(tube, bounds.0, bounds.1).unwrap();
        let field =
            Heightfield::from_stock_mesh_controlled(&mesh, bounds.0, bounds.1, 0.1, &|| false)
                .unwrap();
        assert_eq!(field.sample(0., 0.), -4.);
        assert_eq!(field.sample(3., 0.), 0.);
        assert_eq!(field.sample(4.8, 4.8), -4.);
        let cylinder = Stock::FixedRound {
            diameter: 10.,
            inner_diameter: 0.,
            length: 4.,
            position: 0.5,
            tube: false,
            axis: None,
        };
        let mesh = round_mesh(cylinder, bounds.0, bounds.1).unwrap();
        let field =
            Heightfield::from_stock_mesh_controlled(&mesh, bounds.0, bounds.1, 0.1, &|| false)
                .unwrap();
        assert_eq!(field.sample(0., 0.), 0.);
    }
}
