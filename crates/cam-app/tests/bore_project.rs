//! Regression coverage for the supplied CAD project, including its rounded bore entrance.

#[path = "../src/test_fixture.rs"]
mod test_fixture;
use cam_app::{default_machine, AppController, OpKind};
use cam_cldata::{MoveKind, Step};
use cam_model::{CadSurface, EntityKind, EntityRef, Operation};
use cam_toolpath::{CancelToken, JobCompletion};

fn project() -> (AppController, EntityRef) {
    let mut app = AppController::new(default_machine());
    app.open_project(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testfiles/3d_toolpath_calculation.ocam"),
    )
    .unwrap();
    let geometry = app.geometry_state();
    let asset = &geometry.assets[0];
    let face = asset
        .topology
        .faces
        .iter()
        .find(|face| {
            matches!(face.surface,
                CadSurface::Cylinder { radius, inward: true, .. } if (radius - 15.).abs() < 1e-8
            )
        })
        .unwrap();
    let reference = EntityRef {
        asset_id: asset.id,
        revision: asset.revision,
        instance_id: geometry.instances[0].id,
        body_id: face.body_id,
        kind: EntityKind::Face,
        id: face.id,
    };
    (app, reference)
}

/// Explicit stock preparation: a flat, centre-cutting cutter clears the whole
/// hole. CAD display and a wall-finishing Bore do not remove the solid core.
fn prepare_thread_bore(app: &mut AppController) {
    let tool = app.use_tool(cam_model::Tool {
        number: 44,
        diameter: 30.,
        length: 62.,
        flute_length: 32.,
        kind: cam_model::ToolKind::EndMill,
        ..Default::default()
    });
    app.add_operation(Operation::Drill(cam_model::DrillOp {
        cycle: Default::default(),
        tip_through_bottom: false,
        geometry: Default::default(),
        hole_heights: Vec::new(),
        parameter_expressions: Default::default(),
        id: 0,
        tool,
        points: vec![[-30., 10.]],
        depth: 22.,
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
    app.edit_setup(|setup| {
        let id = setup.operations.last().unwrap().id();
        let settings = setup.height_settings.get_mut(&id).unwrap();
        *settings.definition_mut(cam_model::HeightPlaneKind::Bottom) =
            cam_model::HeightDefinition::new(cam_model::HeightReference::Origin, -1.);
    });
}

#[test]
fn supplied_thread_project_detects_the_noncutting_tip_hitting_the_leftover_core() {
    if !test_fixture::available("thread.ocam") {
        return;
    }
    let mut app = AppController::new(default_machine());
    app.open_project(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testfiles/thread.ocam"),
    )
    .unwrap();
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    let collision = result
        .collisions
        .iter()
        .find(|c| c.op_id == Some(2) && c.kind == cam_sim::CollisionKind::NonCuttingThroughStock)
        .expect("the thread mill cannot plunge through the core left by Bore");
    assert!((collision.at[0] + 30.).abs() < 1e-8);
    assert!((collision.at[1] - 10.).abs() < 1e-8);
    // Contact uses the lower tooth flank, above the non-cutting tip; the
    // 8 mm pilot hole clears the neck but leaves stock beneath the tooth.
    assert!((19.8..21.).contains(&collision.at[2]), "{collision:?}");
    assert!(matches!(
        app.export_nc(),
        Err(cam_app::ExportError::NonCuttingThroughStock(_))
    ));
}

#[test]
fn supplied_project_bore_with_rounded_entrance_generates_and_posts() {
    if !test_fixture::available("3d_toolpath_calculation.ocam") {
        return;
    }
    let (mut app, face) = project();
    let original = app.document().clone();
    app.set_operation_excluded(original.setup().operations[0].id(), true);
    let id = app
        .create_cad_operation_with_tool(OpKind::Bore, vec![face], 42)
        .unwrap();
    assert!(matches!(app.operation(id), Some(Operation::Bore(b))
        if b.points == vec![[-30., 10.]] && b.diameter == 30. && b.z_top == 16. && b.z_bottom == 0.));
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    let mut cut_count = 0;
    for step in result.program.steps() {
        if let Step::Arc { end, tag, .. } = step {
            if tag.op_id == id && tag.kind == MoveKind::Cutting {
                assert!(((end.x + 30.).hypot(end.y - 10.) - 11.).abs() < 1e-8);
                assert!((0. ..=16.).contains(&end.z));
                cut_count += 1;
            }
        }
    }
    assert!(cut_count >= 34, "full 16 mm helix and bottom circle");
    assert!(app.export_nc().unwrap().contains("Bore"));
}

#[test]
fn intentional_cutter_shape_mismatch_warns_but_simulates_and_exports() {
    if !test_fixture::available("3d_toolpath_calculation.ocam") {
        return;
    }
    use cam_model::{Tool, ToolKind};
    for kind in [
        ToolKind::VBit {
            included_angle_deg: 60.,
            tip_radius: 0.2,
        },
        ToolKind::ChamferMill {
            included_angle_deg: 90.,
            tip_diameter: 1.,
        },
        ToolKind::BallMill,
    ] {
        let (mut app, face) = project();
        let previous = app.document().setup().operations[0].id();
        app.set_operation_excluded(previous, true);
        prepare_thread_bore(&mut app);
        let tool = app.use_tool(Tool {
            number: 43,
            diameter: 8.,
            length: 62.,
            flute_length: 32.,
            kind,
            ..Default::default()
        });
        let id = app
            .create_cad_operation_with_tool(OpKind::Bore, vec![face], tool)
            .unwrap();
        let result = app.run(&CancelToken::new());
        assert_eq!(
            result.completion,
            JobCompletion::Complete,
            "{kind:?}: {:?}",
            result.diagnostics
        );
        assert!(!result.has_errors(), "{kind:?}: {:?}", result.diagnostics);
        assert!(
            result
                .diagnostics
                .iter()
                .any(|d| d.op == Some(id) && d.severity == cam_toolpath::Severity::Warning),
            "{kind:?}"
        );
        assert!(
            !result.stock_vertices.is_empty(),
            "{kind:?}: simulation must still run"
        );
        assert!(result.program.steps().iter().any(|step| matches!(step,
            Step::Arc { tag, .. } if tag.op_id == id && tag.kind == MoveKind::Cutting)));
        assert!(app
            .export_nc()
            .unwrap_or_else(|error| panic!("{kind:?}: {error}"))
            .contains("Bore"));
    }
}

/// The same cylinder/torus with a deliberately prepared bore supports threading.
#[test]
fn rounded_bore_threads_regenerate_reopen_and_export_with_a_thread_mill() {
    if !test_fixture::available("3d_toolpath_calculation.ocam") {
        return;
    }
    use cam_model::{Tool, ToolKind};
    let (mut app, face) = project();
    let old_id = app.document().setup().operations[0].id();
    app.set_operation_excluded(old_id, true);
    prepare_thread_bore(&mut app);
    let tool = app.use_tool(Tool {
        number: 43,
        diameter: 8.,
        neck_diameter: 4.,
        flute_length: 32.,
        length: 62.,
        kind: ToolKind::ThreadMill { pitch: None },
        ..Default::default()
    });
    let id = app
        .create_cad_operation_with_tool(OpKind::Thread, vec![face], tool)
        .unwrap();
    let mut operation = app.operation(id).unwrap().clone();
    let Operation::Thread(thread) = &mut operation else {
        unreachable!()
    };
    assert_eq!(thread.points, vec![[-30., 10.]]);
    assert_eq!((thread.z_top, thread.z_bottom), (16., 0.));
    // A bore is not a nominal thread specification. Choose an illustrative
    // 60-degree form whose basic minor diameter is the existing 30 mm hole.
    thread.pitch = 1.25;
    thread.major_dia = 30. + 1.082_53 * thread.pitch;
    thread.passes = 3;
    thread.spring_passes = 1;
    app.apply_operation_settings(operation, None).unwrap();
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    let thread_faces: Vec<_> = result
        .stock_vertices
        .iter()
        .zip(&result.stock_operations)
        .zip(&result.stock_machined)
        .filter(|((_, owner), cut)| **owner == Some(id) && **cut)
        .map(|((vertex, _), _)| vertex)
        .collect();
    assert!(
        thread_faces.iter().any(|v| v.normal[2] > 0.5),
        "upper flank cut faces"
    );
    assert!(
        thread_faces.iter().any(|v| v.normal[2] < -0.5),
        "lower flank cut faces"
    );
    let cuts: Vec<_> = result
        .program
        .steps()
        .iter()
        .filter_map(|step| match step {
            Step::Arc {
                end, center, tag, ..
            } if tag.op_id == id && tag.kind == MoveKind::Cutting => Some((*end, *center)),
            _ => None,
        })
        .collect();
    // Fractional 12.8-turn helices retain exactly 1.25 mm/revolution.
    assert_eq!(cuts.len(), 26 * 4);
    for pass in cuts.as_chunks::<26>().0 {
        let mut previous_angle = 0.;
        let mut previous_z = 0.;
        for (end, center) in pass {
            assert!((center.x + 30.).abs() < 1e-8 && (center.y - 10.).abs() < 1e-8);
            let angle = (end.y - center.y).atan2(end.x - center.x);
            let sweep = (angle - previous_angle).rem_euclid(std::f64::consts::TAU);
            assert!(((end.z - previous_z) / sweep * std::f64::consts::TAU - 1.25).abs() < 1e-8);
            assert!((0. ..=16.).contains(&end.z));
            previous_angle = angle;
            previous_z = end.z;
        }
        assert!((previous_z - 16.).abs() < 1e-8);
    }
    let path = std::env::temp_dir().join(format!("rounded-thread-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = AppController::new(default_machine());
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(reopened.operation(id), app.operation(id));
    for post in cam_post::PostKind::ALL {
        reopened.set_post_kind(post);
        let result = reopened.run(&CancelToken::new());
        assert_eq!(
            result.completion,
            JobCompletion::Complete,
            "{post:?}: {:?}",
            result.diagnostics
        );
        assert!(
            reopened.export_nc().unwrap().contains("Thread mill"),
            "{post:?}"
        );
    }
}

#[test]
fn supplied_part_has_compatible_selections_and_tools_for_every_operation() {
    if !test_fixture::available("3d_toolpath_calculation.ocam") {
        return;
    }
    use cam_model::{Tool, ToolKind};
    let (_, cylinder) = project();
    let reference = |kind, id| EntityRef {
        kind,
        id,
        ..cylinder.clone()
    };
    // Each operation gets its manufacturing feature, rather than the same
    // arbitrary face. The rounded entrance itself is not a planar chamfer.
    for (kind, feature) in [
        (OpKind::Profile, reference(EntityKind::Silhouette, 1)),
        (OpKind::Pocket, reference(EntityKind::Face, 9)),
        (OpKind::Drill, cylinder.clone()),
        (OpKind::Face, reference(EntityKind::Face, 4)),
        (OpKind::Chamfer, reference(EntityKind::Face, 10)),
        (OpKind::Bore, cylinder.clone()),
        (OpKind::Thread, cylinder.clone()),
        (OpKind::Engrave, reference(EntityKind::Edge, 18)),
        (OpKind::Carve, reference(EntityKind::Face, 9)),
        (OpKind::Adaptive, reference(EntityKind::Body, 1)),
    ] {
        let (mut app, _) = project();
        app.preview_cad_operation(kind, std::slice::from_ref(&feature))
            .unwrap_or_else(|e| panic!("{kind:?}: {e}"));
        let old_id = app.document().setup().operations[0].id();
        app.set_operation_excluded(old_id, true);
        let mut cutter = Tool {
            number: 43,
            diameter: 8.,
            flute_length: 32.,
            length: 62.,
            kind: ToolKind::EndMill,
            ..Default::default()
        };
        match kind {
            OpKind::Drill => {
                cutter.diameter = 30.;
                cutter.kind = ToolKind::Drill {
                    point_angle_deg: 118.,
                };
            }
            OpKind::Thread => {
                cutter.neck_diameter = 4.;
                cutter.kind = ToolKind::ThreadMill { pitch: None };
            }
            OpKind::Chamfer => {
                cutter.kind = ToolKind::ChamferMill {
                    included_angle_deg: 90.,
                    tip_diameter: 1.,
                }
            }
            OpKind::Engrave | OpKind::Carve => {
                cutter.kind = ToolKind::VBit {
                    included_angle_deg: 60.,
                    tip_radius: 0.,
                }
            }
            _ => {}
        }
        let tool = app.use_tool(cutter);
        if kind == OpKind::Thread {
            prepare_thread_bore(&mut app);
        }
        if kind == OpKind::Carve {
            // The stock starts as a solid block. Expose the recessed carving
            // surface first, including stock over its sharp corners that a
            // round pocket cutter cannot reach. CAD display does not remove stock.
            app.create_cad_operation_with_tool(
                OpKind::Face,
                vec![reference(EntityKind::Face, 4)],
                42,
            )
            .unwrap();
            app.create_cad_operation_with_tool(OpKind::Pocket, vec![feature.clone()], 42)
                .unwrap();
        }
        let id = app
            .create_cad_operation_with_tool(kind, vec![feature], tool)
            .unwrap_or_else(|e| panic!("{kind:?}: {e}"));
        if kind == OpKind::Thread {
            let mut op = app.operation(id).unwrap().clone();
            if let Operation::Thread(t) = &mut op {
                t.major_dia = 31.082_53;
            }
            app.apply_operation_settings(op, None).unwrap();
        }
        let result = app.run(&CancelToken::new());
        assert!(
            result.collisions.is_empty(),
            "{kind:?}: {:?}",
            result.collisions
        );
        assert_eq!(
            result.completion,
            JobCompletion::Complete,
            "{kind:?}: {:?}",
            result.diagnostics
        );
        assert!(result.program.steps().iter().any(|step| matches!(step,
            Step::Linear { tag, .. } | Step::Arc { tag, .. } if tag.op_id == id && tag.kind == MoveKind::Cutting
        )) || kind == OpKind::Drill, "{kind:?}: no cutting motion");
        app.export_nc().unwrap_or_else(|e| panic!("{kind:?}: {e}"));
    }
}

#[test]
fn rounded_bore_access_is_analytic_and_rejects_caps_incomplete_rims_and_obstructions() {
    if !test_fixture::available("3d_toolpath_calculation.ocam") {
        return;
    }
    use cam_model::{CadCurve, CadFace, CadMesh, ExtractionRecipe};
    let (app, reference) = project();
    let original = app.geometry_state().clone();
    let resolve = |geometry: &cam_model::GeometryState, recipe| {
        cam_geometry3d::resolve_selection(geometry, std::slice::from_ref(&reference), recipe, 20.)
    };
    let recipes = [
        ExtractionRecipe::BoreCylinders { internal: true },
        ExtractionRecipe::DrillHoles,
        ExtractionRecipe::ThreadCylinders { internal: true },
    ];
    let mut without_mesh = original.clone();
    std::sync::Arc::make_mut(&mut without_mesh.assets[0].topology).mesh = CadMesh::default();
    for recipe in &recipes {
        let expected = resolve(&original, recipe.clone()).unwrap();
        assert_eq!(
            expected.heights,
            cam_model::CuttingHeights {
                top: 16.,
                bottom: 0.
            }
        );
        assert_eq!(resolve(&without_mesh, recipe.clone()).unwrap(), expected);
    }
    for case in [
        "noncoaxial",
        "narrowing",
        "partial",
        "capped",
        "roof",
        "rounded_bottom",
    ] {
        let mut geometry = original.clone();
        let topology = std::sync::Arc::make_mut(&mut geometry.assets[0].topology);
        match case {
            "noncoaxial" => {
                let CadSurface::Torus { origin, .. } = &mut topology
                    .faces
                    .iter_mut()
                    .find(|f| f.id == 2)
                    .unwrap()
                    .surface
                else {
                    unreachable!()
                };
                origin[0] += 1.;
            }
            "narrowing" => {
                let CadSurface::Torus { major_radius, .. } = &mut topology
                    .faces
                    .iter_mut()
                    .find(|f| f.id == 2)
                    .unwrap()
                    .surface
                else {
                    unreachable!()
                };
                *major_radius -= 1.;
            }
            "partial" => {
                let CadCurve::CircleArc {
                    start_angle,
                    end_angle,
                    ..
                } = &mut topology
                    .edges
                    .iter_mut()
                    .find(|e| e.id == 16)
                    .unwrap()
                    .curve
                else {
                    unreachable!()
                };
                *end_angle = *start_angle + std::f64::consts::PI;
            }
            "capped" => {
                for wire in &mut topology.loops {
                    if wire.face_id == 4 && wire.edge_uses.iter().any(|u| u.edge_id == 16) {
                        wire.is_outer = true;
                    }
                }
            }
            "roof" => topology.faces.push(CadFace {
                id: 1000,
                body_id: reference.body_id,
                loop_ids: Vec::new(),
                surface: CadSurface::Unsupported {
                    description: "obstruction over bore".into(),
                },
                bounds: [[-31., 9., 18.], [-29., 11., 19.]],
            }),
            "rounded_bottom" => {
                topology
                    .faces
                    .iter_mut()
                    .find(|f| f.id == 15)
                    .unwrap()
                    .surface = CadSurface::Torus {
                    origin: [-30., 10., 0.],
                    axis: [0., 0., 1.],
                    major_radius: 19.,
                    minor_radius: 4.,
                };
            }
            _ => unreachable!(),
        }
        for recipe in &recipes {
            let error = resolve(&geometry, recipe.clone()).expect_err(case);
            assert!(
                error.contains("rounded entrance")
                    || error.contains("blocks vertical access")
                    || error.contains("terminations need explicit qualification"),
                "{recipe:?} / {case}: {error}"
            );
        }
    }
}
