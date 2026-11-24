//! Run explicitly with `--features occt --test native_step -- --ignored` and a
//! real OCCT SDK. These never count a missing native backend as a successful import.
#![cfg(feature = "occt")]
use cam_app::{default_machine, AppController, OpKind};
use cam_model::*;
use cam_toolpath::{CancelToken, JobCompletion};
fn fixture(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../cam-kernel/fixtures")
        .join(name)
}
fn reference(state: &GeometryState, kind: EntityKind, id: u64) -> EntityRef {
    let asset = &state.assets[0];
    let instance = &state.instances[0];
    EntityRef {
        asset_id: asset.id,
        revision: asset.revision,
        instance_id: instance.id,
        body_id: instance.body_id,
        kind,
        id,
    }
}
fn instance_reference(
    state: &GeometryState,
    instance_id: u64,
    kind: EntityKind,
    id: u64,
) -> EntityRef {
    let instance = state.instance(instance_id).unwrap();
    let asset = state.asset(instance.asset_id).unwrap();
    EntityRef {
        asset_id: asset.id,
        revision: asset.revision,
        instance_id,
        body_id: instance.body_id,
        kind,
        id,
    }
}
fn cutting_z_range(program: &cam_cldata::Program) -> (f64, f64) {
    let mut z: Vec<f64> = Vec::new();
    for step in program.steps() {
        match step {
            cam_cldata::Step::Linear { to, tag, .. }
                if matches!(
                    tag.kind,
                    cam_cldata::MoveKind::Cutting | cam_cldata::MoveKind::Plunge
                ) =>
            {
                z.push(to.z)
            }
            cam_cldata::Step::Arc { end, tag, .. }
                if matches!(
                    tag.kind,
                    cam_cldata::MoveKind::Cutting | cam_cldata::MoveKind::Plunge
                ) =>
            {
                z.push(end.z)
            }
            _ => {}
        }
    }
    (
        *z.iter()
            .min_by(|a, b| a.total_cmp(b))
            .expect("cutting moves"),
        *z.iter()
            .max_by(|a, b| a.total_cmp(b))
            .expect("cutting moves"),
    )
}
fn posted_cutting_z_range(nc: &str) -> (f64, f64) {
    let mut mode = 0;
    let mut z: Vec<f64> = Vec::new();
    for line in nc.lines() {
        let words: Vec<_> = line.split_whitespace().collect();
        if let Some(g) = words
            .iter()
            .find_map(|word| word.strip_prefix('G').and_then(|n| n.parse::<u8>().ok()))
        {
            mode = g;
        }
        if (1..=3).contains(&mode) {
            for value in words.iter().filter_map(|word| word.strip_prefix('Z')) {
                let number: String = value
                    .chars()
                    .take_while(|c| c.is_ascii_digit() || matches!(c, '.' | '-' | '+'))
                    .collect();
                if let Ok(value) = number.parse() {
                    z.push(value);
                }
            }
        }
    }
    (
        *z.iter()
            .min_by(|a, b| a.total_cmp(b))
            .expect("posted Z words"),
        *z.iter()
            .max_by(|a, b| a.total_cmp(b))
            .expect("posted Z words"),
    )
}
fn select_all_models(app: &mut AppController) {
    let ids = app
        .geometry_state()
        .instances
        .iter()
        .map(|i| i.id)
        .collect();
    app.edit_setup(|setup| setup.model_instances = ids);
}
fn new_app() -> AppController {
    assert!(
        cam_kernel::backend_available(),
        "This qualification test requires a real OCCT SDK"
    );
    let mut app = AppController::new(default_machine());
    app.edit_tool(0, |t| {
        t.diameter = 3.;
        t.flute_length = 12.;
    });
    app.edit_stock(|s| {
        *s = Stock::BoundingBox {
            x_offset: 0.,
            y_offset: 0.,
            top: 0.,
            thickness: 10.,
        }
    });
    app
}
#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn step_source_tolerances_preserve_validity_and_geometry_selection() {
    let mut app = new_app();
    app.import_cad(fixture("toleranced-spline-prism-mm.step"))
        .unwrap();
    select_all_models(&mut app);
    let state = app.geometry_state();
    state.validate().unwrap();
    let topology = &state.assets[0].topology;
    assert_eq!(topology.bodies.len(), 1);
    assert!(
        topology.bodies[0].valid_solid,
        "source tolerances must not be clamped to the import fallback precision"
    );
    let vertical = topology
        .faces
        .iter()
        .find(|face| {
            matches!(face.surface,
                CadSurface::LinearExtrusion { axis, .. } if axis[2].abs() > 0.99
            )
        })
        .expect("vertical boundary face");
    let face = reference(state, EntityKind::Face, vertical.id);
    app.toggle_cad_selection(face.clone(), false).unwrap();
    assert_eq!(app.cad_selection(), &[face]);
    // Validity does not make this vertical boundary machinable as an XY pocket.
    let error = app.cad_operation_eligibility(OpKind::Pocket).unwrap_err();
    assert!(error.contains("not coplanar in setup XY"), "{error}");
}
#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn blind_bore_inside_profile_ignores_unrelated_blends_and_round_trips() {
    let mut app = new_app();
    app.import_cad(fixture("blended-box-blind-bore-mm.step"))
        .unwrap();
    select_all_models(&mut app);
    let state = app.geometry_state();
    let topology = &state.assets[0].topology;
    assert!(topology.bodies[0].valid_solid);
    assert!(topology.faces.iter().any(|f| matches!(
        f.surface,
        CadSurface::Unsupported { .. } | CadSurface::Torus { .. } | CadSurface::Sphere { .. }
    )));
    let floor = topology.faces.iter().find(|f| matches!(f.surface,
        CadSurface::Plane { origin, normal } if (origin[2] + 2.0).abs() < 1e-6 && normal[2] > 0.99
    )).expect("blind-bore floor");
    let selected = reference(state, EntityKind::Loop, floor.loop_ids[0]);
    let cylinder = topology
        .faces
        .iter()
        .find(|f| matches!(f.surface, CadSurface::Cylinder { inward: true, .. }))
        .unwrap();
    let trim = reference(state, EntityKind::Loop, cylinder.loop_ids[0]);
    let error = app
        .preview_cad_operation(OpKind::Profile, std::slice::from_ref(&trim))
        .unwrap_err();
    assert!(error.contains("not coplanar in setup XY"), "{error}");
    assert!(
        app.preview_cad_operation(OpKind::Profile, &[trim, selected.clone()])
            .is_err(),
        "a cylindrical seam cannot become a planar chain by adding another loop"
    );
    let preview = app
        .preview_cad_operation(OpKind::Profile, std::slice::from_ref(&selected))
        .unwrap();
    assert_eq!(
        preview.heights,
        CuttingHeights {
            top: 0.0,
            bottom: -2.0
        }
    );
    let id = app
        .create_cad_operation(OpKind::Profile, vec![selected.clone()])
        .unwrap();
    assert!(matches!(app.operation(id), Some(Operation::Profile(p)) if p.side == Side::Inside));
    assert_eq!(app.geometry_state().bindings[&id].selected, vec![selected]);
    assert_eq!(
        app.geometry_state().bindings[&id].recipe,
        ExtractionRecipe::ProfileContours
    );
    let run = app.run(&CancelToken::new());
    assert_eq!(
        run.completion,
        JobCompletion::Complete,
        "{:?}",
        run.diagnostics
    );
    let program = run.program.clone();
    let (lo, hi) = cutting_z_range(&program);
    assert!((lo + 2.0).abs() < 1e-6 && hi <= 0.0);
    let tool_radius = 1.5;
    for step in program.steps() {
        let point = match step {
            cam_cldata::Step::Linear { to, tag, .. }
                if tag.kind == cam_cldata::MoveKind::Cutting =>
            {
                Some(to)
            }
            cam_cldata::Step::Arc { end, tag, .. } if tag.kind == cam_cldata::MoveKind::Cutting => {
                Some(end)
            }
            _ => None,
        };
        if let Some(point) = point {
            assert!(
                (point.x - 30.0).hypot(point.y - 20.0) + tool_radius <= 4.001,
                "inside compensation must keep the cutter within the bore: {point:?}"
            );
        }
    }
    let nc = app.export_nc().unwrap().to_owned();
    let path =
        std::env::temp_dir().join(format!("native-bore-profile-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = new_app();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(reopened.run(&CancelToken::new()).program, program);
    assert_eq!(reopened.export_nc().unwrap(), nc);
}
#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn step_plate_floor_island_four_holes_profile_generates_and_reopens_without_source() {
    let mut app = new_app();
    let source = std::env::temp_dir().join(format!("native-source-{}.step", std::process::id()));
    std::fs::copy(fixture("pocket-island-holes-mm.step"), &source).unwrap();
    app.import_cad(&source).unwrap();
    select_all_models(&mut app);
    std::fs::remove_file(&source).unwrap();
    app.geometry_state().validate().unwrap();
    let topology = &app.geometry_state().assets[0].topology;
    assert_eq!(topology.bodies.len(), 1);
    assert_eq!(topology.bodies[0].bounds, [[0., 0., -10.], [60., 40., 0.]]);
    let floor=topology.faces.iter().find(|f|matches!(f.surface,CadSurface::Plane{origin,normal} if (origin[2]+6.).abs()<1e-6&&normal[2]>0.9)).expect("pocket floor");
    let floor_ref = reference(app.geometry_state(), EntityKind::Face, floor.id);
    let holes: Vec<_> = topology
        .faces
        .iter()
        .filter(|f| matches!(f.surface, CadSurface::Cylinder { inward: true, .. }))
        .map(|f| reference(app.geometry_state(), EntityKind::Face, f.id))
        .collect();
    assert_eq!(holes.len(), 4);
    let top=topology.faces.iter().find(|f|matches!(f.surface,CadSurface::Plane{origin,normal} if origin[2].abs()<1e-6&&normal[2]>0.9)&&f.loop_ids.len()>2).unwrap();
    let outer = topology
        .loops
        .iter()
        .find(|w| w.face_id == top.id && w.is_outer)
        .unwrap();
    let profile_ref = reference(app.geometry_state(), EntityKind::Loop, outer.id);
    let pocket = app
        .create_cad_operation(OpKind::Pocket, vec![floor_ref])
        .unwrap();
    app.edit_operation(pocket, |op| {
        let Operation::Pocket(p) = op else { panic!() };
        // The 2 mm stepdown requires lifting above the uncut material between rings.
        p.clear.lift_height = 2.0;
    });
    let Operation::Pocket(p) = app.operation(pocket).unwrap() else {
        panic!()
    };
    assert_eq!(p.islands.len(), 1);
    assert_eq!(p.depth, 6.);
    let drill = app.use_tool(Tool {
        number: 7,
        diameter: 5.,
        length: 30.,
        flute_length: 12.,
        kind: ToolKind::Drill {
            point_angle_deg: 118.,
        },
        ..Default::default()
    });
    app.create_cad_operation_with_tool(OpKind::Drill, holes, drill)
        .unwrap();
    let profile = app
        .create_cad_operation_with_tool(OpKind::Profile, vec![profile_ref], 1)
        .unwrap();
    // The selected outer loop is at stock top; profile through the stock.
    app.edit_setup(|setup| {
        setup.height_settings.get_mut(&profile).unwrap().planes[4] =
            HeightDefinition::new(HeightReference::StockBottom, 0.);
    });
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    let program = result.program.clone();
    let nc = app.export_nc().unwrap().to_string();
    assert!(nc.contains("G"));
    let path = std::env::temp_dir().join(format!("native-step-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = new_app();
    reopened.open_project(&path).unwrap();
    assert_eq!(reopened.run(&CancelToken::new()).program, program);
    assert_eq!(reopened.export_nc().unwrap(), nc);
    std::fs::remove_file(path).unwrap();
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn adaptive_clearing_creates_runs_posts_and_round_trips() {
    let mut app = new_app();
    app.import_cad(fixture("pocket-island-holes-mm.step"))
        .unwrap();
    select_all_models(&mut app);
    let topology = &app.geometry_state().assets[0].topology;
    assert_eq!(topology.bodies.len(), 1);
    let body = reference(
        app.geometry_state(),
        EntityKind::Body,
        topology.bodies[0].id,
    );
    let id = app
        .create_cad_operation(OpKind::Adaptive, vec![body.clone()])
        .unwrap();
    app.edit_operation(id, |operation| {
        if let Operation::Adaptive(adaptive) = operation {
            adaptive.params.optimal_load = 2.9;
            adaptive.params.stepdown = 10.0;
            adaptive.params.fine_stepdown = 10.0;
        }
    });
    assert!(matches!(app.operation(id), Some(Operation::Adaptive(_))));
    assert_eq!(app.geometry_state().bindings[&id].selected, vec![body]);

    let run = app.run(&CancelToken::new());
    assert_eq!(
        run.completion,
        JobCompletion::Complete,
        "{:?}",
        run.diagnostics
    );
    let program = run.program.clone();
    assert!(program.steps().iter().any(|step| matches!(
        step,
        cam_cldata::Step::Linear { tag, .. } | cam_cldata::Step::Arc { tag, .. }
            if tag.kind == cam_cldata::MoveKind::Cutting
    )));
    let nc = app.export_nc().unwrap().to_owned();
    assert!(nc.contains("G"));

    let path = std::env::temp_dir().join(format!("adaptive-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = new_app();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(reopened.run(&CancelToken::new()).program, program);
    assert_eq!(reopened.export_nc().unwrap(), nc);
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn adaptive_body_subset_can_target_a_nonfirst_setup_instance_and_persist() {
    let mut app = new_app();
    // Solid boxes have no pockets: provide surrounding stock to rough away.
    app.edit_stock(|stock| {
        if let Stock::BoundingBox {
            x_offset, y_offset, ..
        } = stock
        {
            *x_offset = 5.;
            *y_offset = 5.;
        }
    });
    app.import_cad(fixture("repeated-boxes.step")).unwrap();
    select_all_models(&mut app);
    let geometry = app.geometry_state();
    assert_eq!(geometry.instances.len(), 2);
    let setup_instances = app.document().setup().model_instances.clone();
    assert_eq!(setup_instances.len(), 2);
    let second = geometry.instances[1].id;
    let body_id = geometry.instances[1].body_id;
    let body = instance_reference(geometry, second, EntityKind::Body, body_id);
    let preview = app
        .preview_cad_operation(OpKind::Adaptive, std::slice::from_ref(&body))
        .unwrap();

    let id = app
        .create_cad_operation(OpKind::Adaptive, vec![body.clone()])
        .unwrap();
    assert_eq!(
        app.geometry_state().bindings[&id].selected,
        vec![body.clone()]
    );
    assert_eq!(app.document().setup().model_instances, setup_instances);
    app.edit_operation(id, |operation| {
        if let Operation::Adaptive(adaptive) = operation {
            adaptive.params.containment = AdaptiveContainment::Outside;
            adaptive.params.additional_offset = 5.;
            adaptive.params.optimal_load = 2.9;
            adaptive.params.stepdown = 10.;
            adaptive.params.fine_stepdown = 10.;
        }
    });
    let run = app.run(&CancelToken::new());
    assert_eq!(
        run.completion,
        JobCompletion::Complete,
        "{:?}",
        run.diagnostics
    );
    let (cut_bottom, _) = cutting_z_range(&run.program);
    assert!(
        cut_bottom < preview.heights.top - 1.0,
        "selected body must produce cuts below stock top"
    );
    assert!(run.program.steps().iter().any(|step| matches!(
        step,
        cam_cldata::Step::Linear { tag, .. } | cam_cldata::Step::Arc { tag, .. }
            if tag.kind == cam_cldata::MoveKind::Cutting
    )));
    assert!(app.export_nc().unwrap().contains("G"));

    let path = std::env::temp_dir().join(format!("adaptive-subset-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = new_app();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(reopened.document().setup().model_instances, setup_instances);
    assert_eq!(reopened.geometry_state().bindings[&id].selected, vec![body]);
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn adaptive_selection_across_two_component_instances_creates_runs_and_round_trips() {
    let mut app = new_app();
    // Solid boxes have no pockets: provide surrounding stock to rough away.
    app.edit_stock(|stock| {
        if let Stock::BoundingBox {
            x_offset, y_offset, ..
        } = stock
        {
            *x_offset = 5.;
            *y_offset = 5.;
        }
    });
    app.import_cad(fixture("repeated-boxes.step")).unwrap();
    select_all_models(&mut app);
    let geometry = app.geometry_state();
    assert_eq!(geometry.instances.len(), 2);
    let selected: Vec<_> = geometry
        .instances
        .iter()
        .map(|instance| {
            instance_reference(geometry, instance.id, EntityKind::Body, instance.body_id)
        })
        .collect();
    assert_ne!(selected[0].instance_id, selected[1].instance_id);
    let setup_instances = app.document().setup().model_instances.clone();
    assert_eq!(setup_instances.len(), 2);

    let preview = app
        .preview_cad_operation(OpKind::Adaptive, &selected)
        .unwrap();
    assert!(preview.heights.depth() > 1.0);
    let id = app
        .create_cad_operation(OpKind::Adaptive, selected.clone())
        .unwrap();
    app.edit_operation(id, |operation| {
        if let Operation::Adaptive(adaptive) = operation {
            adaptive.params.containment = AdaptiveContainment::Outside;
            adaptive.params.additional_offset = 5.;
            adaptive.params.optimal_load = 2.9;
            adaptive.params.stepdown = 10.;
            adaptive.params.fine_stepdown = 10.;
        }
    });
    assert_eq!(app.geometry_state().bindings[&id].selected, selected);
    assert_eq!(app.document().setup().model_instances, setup_instances);

    let run = app.run(&CancelToken::new());
    assert_eq!(
        run.completion,
        JobCompletion::Complete,
        "{:?}",
        run.diagnostics
    );
    assert!(run.program.steps().iter().any(|step| matches!(
        step,
        cam_cldata::Step::Linear { tag, .. } | cam_cldata::Step::Arc { tag, .. }
            if tag.kind == cam_cldata::MoveKind::Cutting
    )));
    let program = run.program.clone();
    let nc = app.export_nc().unwrap().to_owned();
    assert!(nc.contains("G"));

    let path = std::env::temp_dir().join(format!(
        "adaptive-multi-instance-{}.ocam",
        std::process::id()
    ));
    app.save_project(&path).unwrap();
    let mut reopened = new_app();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(reopened.document().setup().model_instances, setup_instances);
    assert_eq!(reopened.geometry_state().bindings[&id].selected, selected);
    assert_eq!(reopened.run(&CancelToken::new()).program, program);
    assert_eq!(reopened.export_nc().unwrap(), nc);
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn adaptive_selected_pocket_floor_face_scope_and_reference_survive_project_round_trip() {
    let mut app = new_app();
    app.import_cad(fixture("pocket-island-holes-mm.step"))
        .unwrap();
    select_all_models(&mut app);
    let geometry = app.geometry_state();
    let instance = &geometry.instances[0];
    let asset = geometry.asset(instance.asset_id).unwrap();
    let body = asset
        .topology
        .bodies
        .iter()
        .find(|body| body.id == instance.body_id)
        .unwrap();
    let body_bounds = body.bounds;
    let face = asset
        .topology
        .faces
        .iter()
        .find(|face| {
            matches!(&face.surface, CadSurface::Plane { normal, .. } if normal[2] > 0.99)
                && face.bounds[0][2] > body.bounds[0][2] + 0.1
                && face.bounds[1][2] < body.bounds[1][2] - 0.1
                && face.bounds[1][0] > face.bounds[0][0]
                && face.bounds[1][1] > face.bounds[0][1]
        })
        .expect("upward-facing pocket floor below the stock top")
        .id;
    let selected = instance_reference(geometry, instance.id, EntityKind::Face, face);
    let preview = app
        .preview_cad_operation(OpKind::Adaptive, std::slice::from_ref(&selected))
        .unwrap();
    assert!(preview.heights.bottom.is_finite());
    let id = app
        .create_cad_operation(OpKind::Adaptive, vec![selected.clone()])
        .unwrap();
    assert_eq!(
        app.geometry_state().bindings[&id].selected,
        vec![selected.clone()]
    );
    app.edit_operation(id, |operation| {
        if let Operation::Adaptive(adaptive) = operation {
            adaptive.params.optimal_load = 2.9;
            adaptive.params.stepdown = 10.;
            adaptive.params.fine_stepdown = 10.;
        }
    });
    let setup_geometry = app.setup_geometry_state().unwrap();
    let scoped = cam_geometry3d::resolve_adaptive(
        &setup_geometry,
        &app.geometry_state().bindings[&id],
        cam_geometry3d::AdaptiveResolveConfig {
            heights: preview.heights,
            stock_to_leave: StockToLeave::default(),
            stepdown: 2.,
            fine_stepdown: 1.,
            tolerance: 0.1,
        },
        || false,
    )
    .unwrap();
    assert!(!scoped.layers.is_empty());
    assert!(scoped
        .layers
        .iter()
        .all(|layer| layer.body_domains.is_empty()));
    assert!(scoped
        .layers
        .iter()
        .all(|layer| !layer.face_domains.is_empty()));
    let selected_area: f64 = scoped.layers[0]
        .face_domains
        .iter()
        .map(cam_geo::Polygon::area)
        .sum();
    let body_area =
        (body_bounds[1][0] - body_bounds[0][0]) * (body_bounds[1][1] - body_bounds[0][1]);
    assert!(
        selected_area < body_area,
        "face domain must remain narrower than the body XY bounds"
    );
    let run = app.run(&CancelToken::new());
    assert_eq!(
        run.completion,
        JobCompletion::Complete,
        "{:?}",
        run.diagnostics
    );
    let (cut_bottom, _) = cutting_z_range(&run.program);
    assert!(
        cut_bottom < preview.heights.top - 1.0,
        "selected pocket floor must produce real cuts below stock top"
    );
    assert!(run.program.steps().iter().any(|step| matches!(
        step,
        cam_cldata::Step::Linear { tag, .. } | cam_cldata::Step::Arc { tag, .. }
            if tag.kind == cam_cldata::MoveKind::Cutting
    )));
    assert!(app.export_nc().unwrap().contains("G"));

    let path = std::env::temp_dir().join(format!("adaptive-face-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = new_app();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(
        reopened.geometry_state().bindings[&id].selected,
        vec![selected]
    );
    let resolved = reopened
        .preview_cad_operation(
            OpKind::Adaptive,
            &reopened.geometry_state().bindings[&id].selected,
        )
        .unwrap();
    assert_eq!(resolved.heights, preview.heights);
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn box_step_machines_top_and_bottom_faces_in_independent_frames_and_round_trips() {
    let mut app = new_app();
    app.import_cad(fixture("box-mm.step")).unwrap();
    select_all_models(&mut app);
    app.geometry_state().validate().unwrap();
    let asset = &app.geometry_state().assets[0];
    let instance_id = app.geometry_state().instances[0].id;
    let original_placement = app.geometry_state().instances[0].placement;
    let topology = &asset.topology;
    assert_eq!(
        topology.bodies[0].bounds,
        [[0.0, 0.0, -10.0], [60.0, 40.0, 0.0]]
    );
    let top = topology
        .faces
        .iter()
        .find(|face| {
            matches!(face.surface,
                CadSurface::Plane { origin, normal } if origin[2].abs() < 1e-7 && normal[2] > 0.99
            )
        })
        .expect("top planar face at maximum Z");
    let bottom = topology.faces.iter().find(|face| matches!(face.surface,
        CadSurface::Plane { origin, normal } if (origin[2] + 10.0).abs() < 1e-7 && normal[2] < -0.99
    )).expect("bottom planar face at minimum Z");
    let top_ref = reference(app.geometry_state(), EntityKind::Face, top.id);
    let bottom_ref = reference(app.geometry_state(), EntityKind::Face, bottom.id);

    let mut top_setup = app.document().setup().clone();
    top_setup.name = "Top".into();
    top_setup.program_name = "TOP".into();
    top_setup.stock = Stock::FixedBox {
        size: [62.0, 42.0, 12.0],
        model_position: [0.0; 3],
    };
    top_setup.heights = Heights::new(15.0, 5.0, 2.0);
    app.apply_setup_settings(top_setup).unwrap();
    assert_eq!(app.stock_box(), ([0.0, 0.0, -10.0], [62.0, 42.0, 2.0]));
    let top_operation = app
        .create_cad_operation(OpKind::Face, vec![top_ref])
        .unwrap();
    let top_heights = app.geometry_state().bindings[&top_operation].heights;
    assert!((top_heights.top - 2.0).abs() < 1e-6 && (top_heights.bottom - 0.0).abs() < 1e-6);
    let top_run = app.run(&CancelToken::new());
    assert_eq!(
        top_run.completion,
        JobCompletion::Complete,
        "{:?}",
        top_run.diagnostics
    );
    let top_program = top_run.program.clone();
    let (top_cut_min, top_cut_max) = cutting_z_range(&top_program);
    assert!(
        (top_cut_min - 0.0).abs() < 1e-6 && (top_cut_max - 0.0).abs() < 1e-6,
        "top face must cut at the model's maximum Z0: {top_cut_min}..{top_cut_max}"
    );
    let top_nc = app.export_nc().unwrap().to_owned();
    assert_eq!(posted_cutting_z_range(&top_nc), (0.0, 0.0));

    let top_setup_id = app.active_setup_id();
    let bottom_setup_id = app.add_setup();
    let mut bottom_setup = app.document().setup().clone();
    bottom_setup.name = "Bottom".into();
    bottom_setup.model_instances = vec![instance_id];
    bottom_setup.program_name = "BOTTOM".into();
    bottom_setup.frame =
        SetupFrame::from_axes(Some([1.0, 0.0, 0.0]), Some([0.0, -1.0, 0.0]), None).unwrap();
    bottom_setup.stock = Stock::FixedBox {
        size: [62.0, 42.0, 12.0],
        model_position: [0.0; 3],
    };
    bottom_setup.heights = Heights::new(20.0, 15.0, 12.0);
    // In the flipped frame the bottom face is local Z10; the operator touches off
    // there, so posted cutting coordinates should be 0..2 after one datum shift.
    bottom_setup.origin = [0.0, 0.0, 10.0];
    app.apply_setup_settings(bottom_setup).unwrap();
    assert_eq!(app.stock_box(), ([0.0, -40.0, 0.0], [62.0, 2.0, 12.0]));
    assert_eq!(
        app.geometry_state()
            .instance(instance_id)
            .unwrap()
            .placement,
        original_placement,
        "editing a setup frame must leave shared source placement unchanged"
    );
    let bottom_operation = app
        .create_cad_operation(OpKind::Face, vec![bottom_ref])
        .unwrap();
    let bottom_heights = app.geometry_state().bindings[&bottom_operation].heights;
    assert!(
        (bottom_heights.top - 12.0).abs() < 1e-6 && (bottom_heights.bottom - 10.0).abs() < 1e-6
    );
    let bottom_run = app.run(&CancelToken::new());
    assert_eq!(
        bottom_run.completion,
        JobCompletion::Complete,
        "{:?}",
        bottom_run.diagnostics
    );
    let bottom_program = bottom_run.program.clone();
    let (bottom_cut_min, bottom_cut_max) = cutting_z_range(&bottom_program);
    assert!(
        (bottom_cut_min - 10.0).abs() < 1e-6 && (bottom_cut_max - 10.0).abs() < 1e-6,
        "world minimum-Z face must resolve to local Z10: {bottom_cut_min}..{bottom_cut_max}"
    );
    let bottom_nc = app.export_nc().unwrap().to_owned();
    assert_eq!(
        posted_cutting_z_range(&bottom_nc),
        (0.0, 0.0),
        "the local Z10 datum must be subtracted exactly once"
    );
    assert_ne!(
        top_program, bottom_program,
        "setup-local cut coordinates must reflect the bottom fixture"
    );
    assert_ne!(
        top_nc, bottom_nc,
        "each setup must export its own named NC program"
    );
    assert!(top_nc.contains("TOP"));
    assert!(bottom_nc.contains("BOTTOM"));
    assert_eq!(
        app.geometry_state()
            .instance(instance_id)
            .unwrap()
            .placement,
        original_placement
    );

    let path = std::env::temp_dir().join(format!("native-two-side-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = new_app();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(reopened.setups().len(), 2);
    assert_eq!(
        reopened
            .geometry_state()
            .instance(instance_id)
            .unwrap()
            .placement,
        original_placement
    );
    reopened.activate_setup(top_setup_id).unwrap();
    assert_eq!(reopened.run(&CancelToken::new()).program, top_program);
    assert_eq!(reopened.export_nc().unwrap(), top_nc);
    reopened.activate_setup(bottom_setup_id).unwrap();
    assert_eq!(reopened.run(&CancelToken::new()).program, bottom_program);
    assert_eq!(reopened.export_nc().unwrap(), bottom_nc);
}
#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn inch_fixture_normalizes_to_the_same_millimetre_geometry() {
    let mm =
        cam_kernel::read_step_file(fixture("pocket-island-holes-mm.step"), &Default::default())
            .unwrap();
    let inch = cam_kernel::read_step_file(
        fixture("pocket-island-holes-inch.step"),
        &Default::default(),
    )
    .unwrap();
    assert_eq!(mm.source_units, SourceUnits::Millimetres);
    assert_eq!(inch.source_units, SourceUnits::Inches);
    for i in 0..2 {
        for j in 0..3 {
            assert!(
                (mm.topology.bodies[0].bounds[i][j] - inch.topology.bodies[0].bounds[i][j]).abs()
                    < 1e-5
            );
        }
    }
    assert_eq!(mm.topology.faces.len(), inch.topology.faces.len());
    assert_eq!(mm.topology.edges.len(), inch.topology.edges.len());
}
#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn display_tessellation_quality_does_not_change_pocket_nc() {
    let mut snapshots = vec![];
    for tolerance in [0.5, 0.02] {
        let import = cam_kernel::read_step_file(
            fixture("pocket-island-holes-mm.step"),
            &cam_kernel::SolidImportOptions {
                display_tolerance: tolerance,
                ..Default::default()
            },
        )
        .unwrap();
        let asset = GeometryAsset {
            import_options: None,
            id: 1,
            revision: 1,
            name: "plate".into(),
            source_format: CadFormat::Step,
            source_path: None,
            source_data: vec![].into(),
            source_units: import.source_units,
            backend: import.backend,
            source_fingerprint: String::new(),
            import_diagnostics: import.warnings,
            topology: import.topology.into(),
        };
        let instances = import
            .placements
            .into_iter()
            .map(|p| GeometryInstance {
                id: 1,
                asset_id: 1,
                body_id: p.body_id,
                name: "plate".into(),
                placement: p.placement,
                visible: true,
            })
            .collect();
        let mut app = new_app();
        app.add_cad_asset(asset, instances).unwrap();
        select_all_models(&mut app);
        let f=app.geometry_state().assets[0].topology.faces.iter().find(|f|matches!(f.surface,CadSurface::Plane{origin,normal} if (origin[2]+6.).abs()<1e-6&&normal[2]>0.9)).unwrap();
        let r = reference(app.geometry_state(), EntityKind::Face, f.id);
        let pocket = app.create_cad_operation(OpKind::Pocket, vec![r]).unwrap();
        app.edit_operation(pocket, |op| {
            let Operation::Pocket(p) = op else { panic!() };
            p.clear.lift_height = 2.0;
        });
        let result = app.run(&CancelToken::new());
        assert_eq!(result.completion, JobCompletion::Complete);
        snapshots.push(app.export_nc().unwrap().to_string());
    }
    assert_eq!(snapshots[0], snapshots[1]);
}
#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn rotated_step_can_be_aligned_to_one_fixed_machining_frame() {
    let mut app = new_app();
    app.import_cad(fixture("pocket-island-holes-rotated.step"))
        .unwrap();
    select_all_models(&mut app);
    let topology = &app.geometry_state().assets[0].topology;
    let top = topology
        .faces
        .iter()
        .find(|f| {
            f.loop_ids.len() > 2 && matches!(f.surface,CadSurface::Plane{normal,..}if normal[2]>0.5)
        })
        .unwrap();
    let floor = topology
        .faces
        .iter()
        .find(|f| {
            f.loop_ids.len() == 2
                && matches!(f.surface,CadSurface::Plane{normal,..}if normal[2]>0.5)
        })
        .unwrap();
    let top_ref = reference(app.geometry_state(), EntityKind::Face, top.id);
    let floor_ref = reference(app.geometry_state(), EntityKind::Face, floor.id);
    assert!(app
        .create_cad_operation(OpKind::Pocket, vec![floor_ref.clone()])
        .is_err());
    app.align_cad_top_face(top_ref).unwrap();
    let preview = app
        .preview_cad_operation(OpKind::Pocket, std::slice::from_ref(&floor_ref))
        .unwrap();
    assert!(preview.heights.top.abs() < 1e-6);
    assert!((preview.heights.bottom + 6.).abs() < 1e-6);
    app.create_cad_operation(OpKind::Pocket, vec![floor_ref])
        .unwrap();
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
}
#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn repeated_step_components_keep_separate_placements_and_selection_identity() {
    let mut app = new_app();
    app.import_cad(fixture("repeated-boxes.step")).unwrap();
    select_all_models(&mut app);
    assert_eq!(app.geometry_state().instances.len(), 2);
    assert_eq!(app.geometry_state().assets[0].topology.bodies.len(), 1);
    let first = &app.geometry_state().instances[0];
    let second = &app.geometry_state().instances[1];
    assert_ne!(first.id, second.id);
    assert_eq!(first.body_id, second.body_id);
    assert_ne!(first.placement, second.placement);
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn background_replacement_preview_requires_commit_and_marks_operations_for_reselection() {
    let mut app = new_app();
    app.import_cad(fixture("pocket-island-holes-mm.step"))
        .unwrap();
    select_all_models(&mut app);
    let f=app.geometry_state().assets[0].topology.faces.iter().find(|f|matches!(f.surface,CadSurface::Plane{origin,normal} if (origin[2]+6.).abs()<1e-6&&normal[2]>0.9)).unwrap();
    let r = reference(app.geometry_state(), EntityKind::Face, f.id);
    let id = app.create_cad_operation(OpKind::Pocket, vec![r]).unwrap();
    app.start_cad_replacement(1, fixture("pocket-island-holes-mm.step"))
        .unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    loop {
        if let Some(result) = app.poll_cad_import() {
            result.unwrap();
            break;
        }
        assert!(std::time::Instant::now() < deadline);
        std::thread::yield_now();
    }
    assert_eq!(app.geometry_state().assets[0].revision, 1);
    assert!(app.cad_import_preview().is_some());
    app.commit_cad_preview().unwrap();
    assert_eq!(app.geometry_state().assets[0].revision, 2);
    assert!(matches!(
        app.geometry_state().bindings[&id].status,
        ResolutionStatus::NeedsReselection { .. }
    ));
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Failed
    );
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn facing_uses_actual_stock_top_and_rederives_it_after_stock_edits() {
    let mut app = new_app();
    app.import_cad(fixture("box-mm.step")).unwrap();
    select_all_models(&mut app);
    let f=app.geometry_state().assets[0].topology.faces.iter().find(|f|matches!(f.surface,CadSurface::Plane{origin,normal}if origin[2].abs()<1e-6&&normal[2]>0.9)).unwrap();
    let r = reference(app.geometry_state(), EntityKind::Face, f.id);
    app.edit_stock(|s| {
        *s = Stock::BoundingBox {
            x_offset: 0.,
            y_offset: 0.,
            top: 1.,
            thickness: 11.,
        }
    });
    let id = app.create_cad_operation(OpKind::Face, vec![r]).unwrap();
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
    app.edit_stock(|s| {
        *s = Stock::BoundingBox {
            x_offset: 0.,
            y_offset: 0.,
            top: 1.5,
            thickness: 11.5,
        }
    });
    assert_eq!(app.geometry_state().bindings[&id].heights.top, 1.5);
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
    assert!(app.run(&CancelToken::new()).program.steps().iter().any(|s|matches!(s,cam_cldata::Step::Linear{to,tag,..}if tag.kind==cam_cldata::MoveKind::Cutting&&to.z.abs()<1e-9)));
}

#[test]
#[ignore = "requires OCCT SDK and local testfiles/Tigerente.step"]
fn tigerente_face_32_can_face_stock_above_spline_boundaries_and_blends() {
    assert!(cam_kernel::backend_available());
    let mut machine = default_machine();
    machine.envelope = Envelope::new(
        cam_cldata::Point3::new(-500., -500., -100.),
        cam_cldata::Point3::new(500., 500., 200.),
    );
    let mut app = AppController::new(machine);
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testfiles/Tigerente.step");
    app.import_cad(&path).unwrap();
    select_all_models(&mut app);
    let target = reference(app.geometry_state(), EntityKind::Face, 32);
    assert!(app
        .preview_cad_operation(OpKind::Face, std::slice::from_ref(&target))
        .unwrap_err()
        .contains("align setup Z"));
    let mut setup = app.document().setup().clone();
    setup.frame = SetupFrame::from_axes(Some([1., 0., 0.]), Some([0., 0., 1.]), None).unwrap();
    app.apply_setup_settings(setup).unwrap();
    let preview = app
        .preview_cad_operation(OpKind::Face, std::slice::from_ref(&target))
        .unwrap();
    assert!((preview.heights.bottom - 0.).abs() < 1e-6);
    assert!(preview.heights.top > 0.99);
    let id = app
        .create_cad_operation(OpKind::Face, vec![target])
        .unwrap();
    let outcome = app.run(&CancelToken::new());
    assert_eq!(
        outcome.completion,
        JobCompletion::Complete,
        "{:?}",
        outcome.diagnostics
    );
    assert!(!outcome.has_errors(), "{:?}", outcome.diagnostics);
    assert!(!outcome.program.is_empty());
    let nc = app.export_nc().unwrap();
    assert!(nc.contains("M30"));
    assert!(app.geometry_state().bindings.contains_key(&id));
}

#[test]
#[ignore = "requires OCCT SDK and local testfiles/Tigerente.step"]
fn tigerente_blind_pockets_and_drills_use_local_certificates_in_both_setups() {
    for (floor_id, bore_id, setup_y, bottom, top) in [
        (14, 13, [0., 0., -1.], 28., 30.),
        (27, 26, [0., 0., 1.], -2., 0.),
    ] {
        let mut app = new_app();
        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testfiles/Tigerente.step");
        app.import_cad(&path).unwrap();
        select_all_models(&mut app);
        let floor = reference(app.geometry_state(), EntityKind::Face, floor_id);
        let bore = reference(app.geometry_state(), EntityKind::Face, bore_id);
        // A wrong setup still fails for the selected feature, regardless of
        // which unrelated curved face happens to appear first in the B-rep.
        assert!(app
            .preview_cad_operation(OpKind::Pocket, std::slice::from_ref(&floor))
            .unwrap_err()
            .contains("align setup Z"));
        let mut setup = app.document().setup().clone();
        setup.frame = SetupFrame::from_xy([1., 0., 0.], setup_y).unwrap();
        setup.stock = Stock::RelativeBox {
            offsets: [[1.; 3]; 2],
        };
        setup.heights = Heights::new(top + 6., top + 3., top + 1.);
        app.apply_setup_settings(setup).unwrap();
        let preview = app
            .preview_cad_operation(OpKind::Pocket, std::slice::from_ref(&floor))
            .unwrap();
        assert!((preview.heights.bottom - bottom).abs() < 1e-6);
        assert!((preview.heights.top - top).abs() < 1e-6);
        app.create_cad_operation(OpKind::Pocket, vec![floor])
            .unwrap();
        let drill_preview = app
            .preview_cad_operation(OpKind::Drill, std::slice::from_ref(&bore))
            .unwrap();
        assert_eq!(preview.heights, drill_preview.heights);
        let drill = app.use_tool(Tool {
            number: 7,
            diameter: 8.,
            length: 30.,
            flute_length: 12.,
            kind: ToolKind::Drill {
                point_angle_deg: 118.,
            },
            ..Default::default()
        });
        app.create_cad_operation_with_tool(OpKind::Drill, vec![bore], drill)
            .unwrap();
        let outcome = app.run(&CancelToken::new());
        assert_eq!(
            outcome.completion,
            JobCompletion::Complete,
            "{:?}",
            outcome.diagnostics
        );
        let (min_z, _) = cutting_z_range(&outcome.program);
        assert!((min_z - bottom).abs() < 1e-6, "{min_z}");
        assert!(!outcome.program.is_empty());
        assert!(app.export_nc().unwrap().contains("M30"));
    }
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn rounded_exterior_xy_silhouette_profiles_and_reopens_without_source() {
    let mut app = new_app();
    app.import_cad(fixture("blended-box-blind-bore-mm.step"))
        .unwrap();
    select_all_models(&mut app);
    let state = app.geometry_state();
    let selected = reference(state, EntityKind::Silhouette, state.instances[0].body_id);
    let scene = cam_render::CadScene::from_geometry(state, &std::collections::BTreeSet::new());
    let ray = cam_render::Ray {
        origin: [10., 10., 50.],
        direction: [0., 0., -1.],
    };
    let hits = scene.pick_profile_candidates(ray, 0.1, false);
    assert_eq!(
        hits[0].entity, selected,
        "the inset top-face boundary must resolve to the full exterior across its fillet"
    );
    assert!(
        state.assets[0]
            .topology
            .faces
            .iter()
            .any(|face| matches!(face.surface,
                CadSurface::Cylinder { axis, radius, .. } if axis[2].abs() < 0.1 && radius > 0.
            )),
        "rounded top transitions retain their inspectable radius"
    );
    let preview = app
        .preview_cad_operation(OpKind::Profile, std::slice::from_ref(&selected))
        .unwrap();
    assert!(preview
        .diagnostics
        .iter()
        .any(|d| d.contains("outward allowance")));
    assert_eq!(preview.regions.len(), 1);
    assert!(preview.regions[0].holes().is_empty());
    let id = app
        .create_cad_operation(OpKind::Profile, vec![selected])
        .unwrap();
    assert!(matches!(app.operation(id), Some(Operation::Profile(p)) if p.side == Side::Outside));
    let run = app.run(&CancelToken::new());
    assert_eq!(
        run.completion,
        JobCompletion::Complete,
        "{:?}",
        run.diagnostics
    );
    let bounds = run.program.validate().unwrap().unwrap();
    assert!(
        bounds.max.x - bounds.min.x < 65. && bounds.max.y - bounds.min.y < 45.,
        "profile motions must stay beside the 60 × 40 mm body: {bounds:?}"
    );
    let nc = app.export_nc().unwrap().to_owned();
    let path =
        std::env::temp_dir().join(format!("opencam-xy-silhouette-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut loaded = new_app();
    loaded.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    let run = loaded.run(&CancelToken::new());
    assert_eq!(
        run.completion,
        JobCompletion::Complete,
        "{:?}",
        run.diagnostics
    );
    assert_eq!(loaded.export_nc().unwrap(), nc);
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn silhouette_projection_is_independent_of_display_quality_on_curved_step_models() {
    for path in [
        fixture("blended-box-blind-bore-mm.step"),
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testfiles/Tigerente.step"),
    ] {
        let coarse = cam_kernel::read_step_file(
            &path,
            &cam_kernel::SolidImportOptions {
                display_tolerance: 0.5,
                ..Default::default()
            },
        )
        .unwrap();
        let fine = cam_kernel::read_step_file(
            &path,
            &cam_kernel::SolidImportOptions {
                display_tolerance: 0.01,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(
            coarse.topology.projection.is_some(),
            "{} has no usable projection",
            path.display()
        );
        assert_eq!(
            coarse.topology.projection,
            fine.topology.projection,
            "{}",
            path.display()
        );
        let mut app = new_app();
        app.import_cad(&path).unwrap();
        select_all_models(&mut app);
        let state = app.geometry_state();
        let selected = reference(state, EntityKind::Silhouette, state.instances[0].body_id);
        let preview = app
            .preview_cad_operation(OpKind::Profile, &[selected])
            .unwrap();
        assert_eq!(preview.regions.len(), 1);
        assert!(preview.regions[0].area() > 1.);
    }
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn profile_face_contours_and_silhouette_resolve_real_step_boundaries() {
    let mut app = new_app();
    app.import_cad(fixture("pocket-island-holes-mm.step"))
        .unwrap();
    select_all_models(&mut app);
    let state = app.geometry_state();
    let topology = &state.assets[0].topology;
    let top = topology
        .faces
        .iter()
        .filter(|face| {
            matches!(face.surface,
        CadSurface::Plane { origin, normal } if origin[2].abs() < 1e-6 && normal[2] > 0.99)
        })
        .max_by_key(|face| face.loop_ids.len())
        .unwrap();
    let face = reference(state, EntityKind::Face, top.id);
    let silhouette = reference(state, EntityKind::Silhouette, state.instances[0].body_id);
    let contours = app.preview_cad_operation(OpKind::Profile, &[face]).unwrap();
    assert_eq!(
        contours.profiles.len(),
        6,
        "outside, pocket opening and four bores"
    );
    assert_eq!(
        contours
            .profiles
            .iter()
            .filter(|p| p.side == Side::Inside)
            .count(),
        5
    );
    assert!(
        (contours.heights.bottom + 6.).abs() < 1e-6,
        "stop all contours at the shallowest wall"
    );
    let projected = app
        .preview_cad_operation(OpKind::Profile, std::slice::from_ref(&silhouette))
        .unwrap();
    assert_eq!(
        projected.profiles.len(),
        5,
        "outside plus four through holes; blind pocket is filled by its floor"
    );
    let id = app
        .create_cad_operation(OpKind::Profile, vec![silhouette])
        .unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Profile(p)) if p.additional_chains.len() == 4)
    );
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn named_profile_entries_mix_sources_preserve_settings_and_reopen() {
    let mut app = new_app();
    app.import_cad(fixture("pocket-island-holes-mm.step"))
        .unwrap();
    select_all_models(&mut app);
    let state = app.geometry_state();
    let topology = &state.assets[0].topology;
    let top = topology
        .faces
        .iter()
        .filter(|face| {
            matches!(face.surface,
        CadSurface::Plane { origin, normal } if origin[2].abs() < 1e-6 && normal[2] > 0.99)
        })
        .max_by_key(|face| face.loop_ids.len())
        .unwrap();
    let face = reference(state, EntityKind::Face, top.id);
    let outer = topology
        .loops
        .iter()
        .find(|wire| wire.face_id == top.id && wire.is_outer)
        .unwrap();
    let chain = reference(state, EntityKind::Loop, outer.id);
    let silhouette = reference(state, EntityKind::Silhouette, state.instances[0].body_id);
    let mut faces =
        ProfileGeometryEntry::new(ProfileGeometryKind::FaceContours, "Face openings".into());
    faces.source = ProfileGeometrySource::Cad(vec![face]);
    faces.curves = ProfileCurves::Inner;
    faces.side_rule = ProfileSideRule::On;
    let mut edge = ProfileGeometryEntry::new(ProfileGeometryKind::Chain, "Reversed rim".into());
    edge.source = ProfileGeometrySource::Cad(vec![chain]);
    edge.chain_mode = ProfileChainMode::Closed;
    edge.reversed = true;
    edge.side_rule = ProfileSideRule::AlwaysInside;
    let mut body =
        ProfileGeometryEntry::new(ProfileGeometryKind::Silhouette, "Body silhouette".into());
    body.source = ProfileGeometrySource::Cad(vec![silhouette.clone()]);
    body.curves = ProfileCurves::Outer;
    body.side_overrides.insert(0, Side::On);
    let id = app
        .create_cad_operation(OpKind::Profile, vec![silhouette])
        .unwrap();
    let mut operation = app.operation(id).unwrap().clone();
    let Operation::Profile(profile) = &mut operation else {
        panic!("profile")
    };
    profile.selections = Some(vec![faces, edge, body]);
    app.apply_operation_settings(operation, None).unwrap();
    let Some(Operation::Profile(profile)) = app.operation(id) else {
        panic!("profile")
    };
    assert_eq!(profile.contours().count(), 7);
    assert_eq!(
        profile.contours().map(|p| p.side).collect::<Vec<_>>(),
        vec![
            Side::On,
            Side::On,
            Side::On,
            Side::On,
            Side::On,
            Side::Inside,
            Side::On
        ]
    );
    assert!(profile.contour_reversed(5));
    assert!(!profile.contour_reversed(6));
    assert_eq!(
        profile
            .selections
            .as_ref()
            .unwrap()
            .iter()
            .map(|e| e.contour_count)
            .collect::<Vec<_>>(),
        vec![5, 1, 1]
    );
    assert!(matches!(
        app.geometry_state().bindings[&id].recipe,
        ExtractionRecipe::ProfileEntries { .. }
    ));
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    let path =
        std::env::temp_dir().join(format!("native-named-profile-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = new_app();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(reopened.operation(id), app.operation(id));
    let result = reopened.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    let before = app.document().clone();
    let mut invalid = app.operation(id).unwrap().clone();
    let Operation::Profile(profile) = &mut invalid else {
        panic!("profile")
    };
    profile.selections.as_mut().unwrap()[2].tolerance = 1e-10;
    assert!(app
        .apply_operation_settings(invalid, None)
        .unwrap_err()
        .contains("finer"));
    assert_eq!(app.document(), &before);
}

#[test]
#[ignore = "requires OCCT SDK and local testfiles/Tigerente.step"]
fn tigerente_mouth_open_edges_ignore_unrelated_spline_trims_and_reopen() {
    let mut app = new_app();
    app.import_cad(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testfiles/Tigerente.step"),
    )
    .unwrap();
    app.edit_tool(0, |tool| tool.diameter = 0.5);
    select_all_models(&mut app);
    let mut setup = app.document().setup().clone();
    setup.stock = Stock::RelativeBox {
        offsets: [[1.; 3]; 2],
    };
    setup.frame = SetupFrame::from_xy([1., 0., 0.], [0., 0., 1.]).unwrap();
    app.apply_setup_settings(setup).unwrap();
    let topology = &app.geometry_state().assets[0].topology;
    assert!(matches!(
        topology.faces.iter().find(|f| f.id == 19).unwrap().surface,
        CadSurface::LinearExtrusion { .. }
    ));
    assert!(matches!(
        topology.faces.iter().find(|f| f.id == 25).unwrap().surface,
        CadSurface::Unsupported { .. }
    ));
    let mut operations = Vec::new();
    for edge_id in [88, 92, 96, 91, 95, 99] {
        let state = app.geometry_state();
        let topology = &state.assets[0].topology;
        let edge = topology.edges.iter().find(|e| e.id == edge_id).unwrap();
        assert!(matches!(
            edge.curve,
            CadCurve::Line { .. } | CadCurve::BezierSpans { .. }
        ));
        let top = edge
            .face_ids
            .iter()
            .filter_map(|id| topology.faces.iter().find(|f| f.id == *id))
            .find(|f| matches!(f.surface, CadSurface::Plane { normal, .. } if normal[1] < -0.99))
            .unwrap();
        assert!(top
            .loop_ids
            .iter()
            .filter_map(|id| topology.loops.iter().find(|w| w.id == *id))
            .flat_map(|w| &w.edge_uses)
            .any(|u| topology
                .edges
                .iter()
                .any(|e| e.id == u.edge_id && matches!(e.curve, CadCurve::BezierSpans { .. }))));
        let selected = reference(state, EntityKind::Edge, edge_id);
        let preview = app
            .preview_cad_operation(OpKind::Profile, std::slice::from_ref(&selected))
            .unwrap_or_else(|error| panic!("mouth edge {edge_id}: {error}"));
        assert_eq!(preview.profiles.len(), 1);
        assert!(!preview.profiles[0].closed);
        assert!(preview.profiles[0].chain.len() >= 2);
        assert!((preview.heights.top - 0.).abs() < 1e-6);
        assert!((preview.heights.bottom + 2.).abs() < 1e-6);
        let id = app
            .create_cad_operation(OpKind::Profile, vec![selected])
            .unwrap();
        let Some(Operation::Profile(profile)) = app.operation(id) else {
            panic!("profile")
        };
        assert!(!profile.closed);
        operations.push(id);
    }
    for edge_id in [86, 100] {
        let selected = reference(app.geometry_state(), EntityKind::Edge, edge_id);
        let preview = app
            .preview_cad_operation(OpKind::Profile, &[selected])
            .unwrap();
        assert!(!preview.profiles[0].closed);
        assert!((preview.heights.bottom + 2.).abs() < 1e-6);
    }
    let floor_chain: Vec<_> = [74, 75, 76, 77, 78, 79, 80, 81]
        .into_iter()
        .map(|id| reference(app.geometry_state(), EntityKind::Edge, id))
        .collect();
    let floor = app
        .preview_cad_operation(OpKind::Profile, &floor_chain)
        .unwrap();
    assert!(!floor.profiles[0].closed);
    assert!((floor.heights.top - 0.).abs() < 1e-6);
    assert!((floor.heights.bottom + 2.).abs() < 1e-6);
    let chain: Vec<_> = [86, 88, 91, 92, 95, 96, 99, 100]
        .into_iter()
        .map(|id| reference(app.geometry_state(), EntityKind::Edge, id))
        .collect();
    let preview = app.preview_cad_operation(OpKind::Profile, &chain).unwrap();
    assert!(!preview.profiles[0].closed);
    assert!(preview.profiles[0].chain.len() > 7);
    operations.push(
        app.create_cad_operation(OpKind::Profile, chain.clone())
            .unwrap(),
    );
    // Display data is deliberately irrelevant to machining and fingerprints.
    let mut geometry = app
        .document()
        .geometry
        .geometry_in_setup(&app.document().setup().frame)
        .unwrap();
    let binding =
        cam_geometry3d::resolve_selection(&geometry, &chain, ExtractionRecipe::ProfileContours, 0.)
            .unwrap();
    let expected = cam_geometry3d::resolve_binding(&geometry, &binding).unwrap();
    std::sync::Arc::make_mut(&mut geometry.assets[0].topology).mesh = CadMesh::default();
    assert_eq!(
        cam_geometry3d::resolve_binding(&geometry, &binding).unwrap(),
        expected
    );
    let outcome = app.run(&CancelToken::new());
    assert_eq!(
        outcome.completion,
        JobCompletion::Complete,
        "{:?}",
        outcome.diagnostics
    );
    assert!(!outcome.has_errors(), "{:?}", outcome.diagnostics);
    assert!(!outcome.program.is_empty());
    let path = std::env::temp_dir().join(format!(
        "tigerente-mouth-profile-{}.ocam",
        std::process::id()
    ));
    app.save_project(&path).unwrap();
    let mut reopened = new_app();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    for id in operations {
        assert_eq!(reopened.operation(id), app.operation(id));
    }
    let outcome = reopened.run(&CancelToken::new());
    assert_eq!(
        outcome.completion,
        JobCompletion::Complete,
        "{:?}",
        outcome.diagnostics
    );
    assert!(!outcome.has_errors(), "{:?}", outcome.diagnostics);
}

#[test]
#[ignore = "requires OCCT SDK; run explicitly for native qualification"]
fn spline_prism_open_rims_use_exact_curves_despite_toleranced_face_joins() {
    let mut app = new_app();
    app.import_cad(fixture("toleranced-spline-prism-mm.step"))
        .unwrap();
    select_all_models(&mut app);
    let mut setup = app.document().setup().clone();
    setup.stock = Stock::RelativeBox {
        offsets: [[1.; 3]; 2],
    };
    app.apply_setup_settings(setup).unwrap();
    let topology = &app.geometry_state().assets[0].topology;
    let top = topology
        .faces
        .iter()
        .find(|f| {
            matches!(f.surface,
        CadSurface::Plane { origin, normal } if (origin[2] - 5.).abs() < 1e-6 && normal[2] > 0.99)
        })
        .unwrap();
    let edges: Vec<_> = topology
        .loops
        .iter()
        .filter(|w| w.face_id == top.id)
        .flat_map(|w| &w.edge_uses)
        .map(|u| reference(app.geometry_state(), EntityKind::Edge, u.edge_id))
        .collect();
    assert_eq!(edges.len(), 4);
    let mut curved = 0;
    for selected in edges {
        let input = app
            .preview_cad_operation(OpKind::Profile, &[selected])
            .unwrap();
        assert!(!input.profiles[0].closed);
        if input.profiles[0].chain.len() > 2 {
            curved += 1;
        }
        assert!((input.heights.top - 5.).abs() < 1e-6);
        assert!(input.heights.bottom.abs() < 1e-6);
    }
    assert_eq!(curved, 2);
}
