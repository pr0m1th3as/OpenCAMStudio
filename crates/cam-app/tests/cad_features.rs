use cam_app::{default_machine, AppController, ExportError, OpKind};
use cam_geo::{Contour, Point};
use cam_model::*;
use cam_toolpath::{CancelToken, JobCompletion};
struct Fixture {
    t: TopologySnapshot,
    next: u64,
}
impl Fixture {
    fn new() -> Self {
        Self {
            t: TopologySnapshot {
                bodies: vec![CadBody {
                    id: 1,
                    name: "Analytic plate".into(),
                    valid_solid: true,
                    bounds: [[0., 0., -5.], [20., 20., 0.]],
                }],
                ..Default::default()
            },
            next: 2,
        }
    }
    fn id(&mut self) -> u64 {
        let id = self.next;
        self.next += 1;
        id
    }
    fn vertex(&mut self, p: [f64; 3]) -> u64 {
        if let Some(v) = self.t.vertices.iter().find(|v| v.point == p) {
            return v.id;
        }
        let id = self.id();
        self.t.vertices.push(CadVertex {
            id,
            body_id: 1,
            point: p,
        });
        id
    }
    fn face(&mut self, rings: Vec<Vec<[f64; 3]>>, normal: [f64; 3]) -> u64 {
        let id = self.id();
        let mut loop_ids = vec![];
        let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        for (i, ring) in rings.iter().enumerate() {
            let wid = self.id();
            loop_ids.push(wid);
            let mut edge_uses = vec![];
            for j in 0..ring.len() {
                let a = ring[j];
                let b = ring[(j + 1) % ring.len()];
                for k in 0..3 {
                    bounds[0][k] = bounds[0][k].min(a[k]);
                    bounds[1][k] = bounds[1][k].max(a[k]);
                }
                let va = self.vertex(a);
                let vb = self.vertex(b);
                let ei = self
                    .t
                    .edges
                    .iter()
                    .position(|e| e.vertices == [va, vb] || e.vertices == [vb, va]);
                let (edge_id, reversed) = if let Some(ei) = ei {
                    let e = &mut self.t.edges[ei];
                    e.face_ids.push(id);
                    (e.id, e.vertices != [va, vb])
                } else {
                    let edge_id = self.id();
                    self.t.edges.push(CadEdge {
                        id: edge_id,
                        body_id: 1,
                        vertices: [va, vb],
                        face_ids: vec![id],
                        curve: CadCurve::Line { start: a, end: b },
                    });
                    (edge_id, false)
                };
                edge_uses.push(EdgeUse { edge_id, reversed });
            }
            self.t.loops.push(CadLoop {
                id: wid,
                body_id: 1,
                face_id: id,
                edge_uses,
                is_outer: i == 0,
            });
        }
        self.t.faces.push(CadFace {
            id,
            body_id: 1,
            surface: CadSurface::Plane {
                origin: rings[0][0],
                normal,
            },
            loop_ids,
            bounds,
        });
        id
    }
    fn walls(&mut self, x0: f64, x1: f64, y0: f64, y1: f64, lo: f64, hi: f64) {
        self.face(
            vec![vec![[x0, y0, lo], [x1, y0, lo], [x1, y0, hi], [x0, y0, hi]]],
            [0., -1., 0.],
        );
        self.face(
            vec![vec![[x1, y0, lo], [x1, y1, lo], [x1, y1, hi], [x1, y0, hi]]],
            [1., 0., 0.],
        );
        self.face(
            vec![vec![[x1, y1, lo], [x0, y1, lo], [x0, y1, hi], [x1, y1, hi]]],
            [0., 1., 0.],
        );
        self.face(
            vec![vec![[x0, y1, lo], [x0, y0, lo], [x0, y0, hi], [x0, y1, hi]]],
            [-1., 0., 0.],
        );
    }
    fn bore(&mut self, center: [f64; 2], radius: f64, top: u64, bottom: u64) -> u64 {
        let face = self.id();
        let mut loop_ids = vec![];
        let height = |id| self.t.faces.iter().find(|f| f.id == id).unwrap().bounds[0][2];
        let (top_z, bottom_z) = (height(top), height(bottom));
        for (z, adjacent) in [(top_z, top), (bottom_z, bottom)] {
            let vertex = self.vertex([center[0] + radius, center[1], z]);
            let edge = self.id();
            self.t.edges.push(CadEdge {
                id: edge,
                body_id: 1,
                vertices: [vertex, vertex],
                face_ids: vec![face, adjacent],
                curve: CadCurve::CircleArc {
                    center: [center[0], center[1], z],
                    normal: [0., 0., 1.],
                    x_axis: [1., 0., 0.],
                    radius,
                    start_angle: 0.,
                    end_angle: std::f64::consts::TAU,
                },
            });
            for owner in [face, adjacent] {
                let wire = self.id();
                self.t.loops.push(CadLoop {
                    id: wire,
                    body_id: 1,
                    face_id: owner,
                    edge_uses: vec![EdgeUse {
                        edge_id: edge,
                        reversed: owner == adjacent,
                    }],
                    is_outer: owner == face,
                });
                if owner == face {
                    loop_ids.push(wire);
                } else {
                    self.t
                        .faces
                        .iter_mut()
                        .find(|f| f.id == owner)
                        .unwrap()
                        .loop_ids
                        .push(wire);
                }
            }
        }
        self.t.faces.push(CadFace {
            id: face,
            body_id: 1,
            surface: CadSurface::Cylinder {
                origin: [center[0], center[1], top_z],
                axis: [0., 0., 1.],
                radius,
                inward: true,
            },
            loop_ids,
            bounds: [
                [center[0] - radius, center[1] - radius, bottom_z],
                [center[0] + radius, center[1] + radius, top_z],
            ],
        });
        face
    }
    fn state(self) -> GeometryState {
        GeometryState {
            assets: vec![GeometryAsset {
                import_options: None,
                import_diagnostics: vec![],
                source_fingerprint: String::new(),
                id: 1,
                revision: 1,
                name: "analytic".into(),
                source_format: CadFormat::Step,
                source_data: vec![].into(),
                source_units: SourceUnits::Millimetres,
                backend: "analytic test fixture (not imported STEP)".into(),
                topology: self.t.into(),
            }],
            instances: vec![GeometryInstance {
                id: 1,
                asset_id: 1,
                body_id: 1,
                name: "part".into(),
                placement: Default::default(),
                visible: true,
            }],
            ..Default::default()
        }
    }
}
fn square(x0: f64, x1: f64, y0: f64, y1: f64, z: f64) -> Vec<[f64; 3]> {
    vec![[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]]
}
fn reference(id: u64, kind: EntityKind) -> EntityRef {
    EntityRef {
        asset_id: 1,
        revision: 1,
        instance_id: 1,
        body_id: 1,
        kind,
        id,
    }
}
fn pocket() -> (GeometryState, EntityRef) {
    pocket_with_bores(&[])
}
fn pocket_with_bores(bores: &[([f64; 2], f64)]) -> (GeometryState, EntityRef) {
    let mut f = Fixture::new();
    let floor = f.face(
        vec![square(2., 18., 2., 18., -3.), square(8., 12., 8., 12., -3.)],
        [0., 0., 1.],
    );
    f.walls(2., 18., 2., 18., -3., 0.);
    f.walls(8., 12., 8., 12., -3., 0.);
    f.face(
        vec![square(0., 20., 0., 20., 0.), square(2., 18., 2., 18., 0.)],
        [0., 0., 1.],
    );
    f.face(vec![square(8., 12., 8., 12., 0.)], [0., 0., 1.]);
    f.walls(0., 20., 0., 20., -5., 0.);
    let bottom = f.face(vec![square(0., 20., 0., 20., -5.)], [0., 0., -1.]);
    for &(center, radius) in bores {
        f.bore(center, radius, floor, bottom);
    }
    (f.state(), reference(floor, EntityKind::Face))
}
fn controller() -> AppController {
    let mut app = AppController::new(default_machine());
    app.edit_tool(0, |t| {
        t.diameter = 3.;
        t.flute_length = 4.;
    });
    app
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

#[test]
fn cad_pocket_toolpaths_bridge_bores_on_both_sides_of_the_cutter_diameter() {
    let (state, floor) = pocket_with_bores(&[([5., 10.], 2.5), ([15., 15.], 1.)]);
    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let id = app
        .create_cad_operation(OpKind::Pocket, vec![floor])
        .unwrap();
    let Some(Operation::Pocket(operation)) = app.operation(id) else {
        panic!("expected a pocket operation");
    };
    assert!(!operation.stock_contours.enabled);
    assert_eq!(operation.islands.len(), 1);
    assert_eq!(operation.islands[0].area(), 16.);

    for adaptive in [false, true] {
        app.edit_operation(id, |op| {
            let Operation::Pocket(pocket) = op else {
                unreachable!()
            };
            pocket.clear.adaptive_enabled = Some(adaptive);
            pocket.clear.clearing.engagement = 1.;
            pocket.clear.lift_height = 0.2;
            pocket.clear.keep_tool_down = true;
            pocket.maximum_stepover = Some(2.85);
        });
        let outcome = app.run(&CancelToken::new());
        assert_eq!(
            outcome.completion,
            JobCompletion::Complete,
            "{:?}",
            outcome.diagnostics
        );
        assert!(!outcome.has_errors(), "{:?}", outcome.diagnostics);
        let simulation = cam_sim::simulate(
            &outcome.program,
            [0., 0., -5.],
            [20., 20., 0.],
            &cam_sim::SimOptions {
                resolution: 0.1,
                tool_radius: 1.5,
            },
            &[],
        );
        assert!(simulation.is_clean(), "{:?}", simulation.collisions);
        let stock = simulation.field;
        assert_eq!(stock.sample(15., 15.), -3., "stock above the small bore");
        assert_eq!(stock.sample(5., 10.), -3., "stock above the wider bore");
        for x in [8.2, 10., 11.8] {
            assert_eq!(stock.sample(x, 10.), 0., "raised island at X{x}");
        }
    }
}

fn circular_face_with_bores(inner_radius: f64) -> (GeometryState, EntityRef) {
    let mut fixture = Fixture::new();
    let mut plane = |z, normal| {
        let id = fixture.id();
        fixture.t.faces.push(CadFace {
            id,
            body_id: 1,
            surface: CadSurface::Plane {
                origin: [10., 10., z],
                normal: [0., 0., normal],
            },
            loop_ids: vec![],
            bounds: [[0., 0., z], [20., 20., z]],
        });
        id
    };
    let top = plane(0., 1.);
    let bottom = plane(-5., -1.);
    let outside = fixture.bore([10., 10.], 10., top, bottom);
    let wall = fixture.t.faces.iter_mut().find(|f| f.id == outside).unwrap();
    let CadSurface::Cylinder { inward, .. } = &mut wall.surface else {
        unreachable!()
    };
    *inward = false;
    for wire in &mut fixture.t.loops {
        if wire.face_id == top || wire.face_id == bottom {
            wire.is_outer = true;
        }
    }
    fixture.bore([10., 10.], inner_radius, top, bottom);
    fixture.bore([18., 10.], 1., top, bottom);
    (fixture.state(), reference(top, EntityKind::Face))
}

#[test]
fn cad_pocket_keeps_the_inner_rim_of_an_annular_face_at_different_diameters() {
    for inner_radius in [1., 6.] {
        let (state, selected) = circular_face_with_bores(inner_radius);
        let binding = cam_geometry3d::resolve_selection_advisory(
            &state,
            &[selected],
            ExtractionRecipe::PocketFloor,
            1.,
        )
        .unwrap();
        let resolved = cam_geometry3d::resolve_binding_advisory(&state, &binding).unwrap();
        let region = &resolved.regions[0];
        assert_eq!(region.holes().len(), 1);
        assert!(!region.contains(Point::new(10., 10.)));
        assert!(region.contains(Point::new(18., 10.)));
    }
}

#[test]
fn indexed_cad_resolves_side_faces_in_the_machine_frame_and_invalidates_paths() {
    let (state, face) = pocket();
    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let id = app
        .create_cad_operation(OpKind::Pocket, vec![face])
        .unwrap();
    app.place_cad_instance(
        1,
        RigidTransform {
            rotation: [[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]],
            translation: [0.; 3],
        },
    )
    .unwrap();
    app.set_operation_rotary_index(
        id,
        Some(RotaryIndex {
            angle: 90.,
            ..Default::default()
        }),
    )
    .unwrap();
    app.edit_stock(|s| {
        *s = Stock::FixedBox {
            size: [22., 7., 22.],
            model_position: [0.5; 3],
        }
    });
    app.edit_tool(0, |t| {
        t.flute_length = 30.;
        t.length = 30.;
    });
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    assert!(result.program.steps().iter().any(
        |s| matches!(s,cam_cldata::Step::Rotary{feed:Some(_),angle,..} if (*angle-90.).abs()<1e-9)
    ));
    assert!(app.toolpath_is_up_to_date(id));
    app.set_operation_rotary_index(
        id,
        Some(RotaryIndex {
            angle: 180.,
            ..Default::default()
        }),
    )
    .unwrap();
    assert!(!app.toolpath_is_up_to_date(id));
    app.undo();
    assert!(app.toolpath_is_up_to_date(id));
}

#[test]
fn closed_spline_boundary_profiles_keep_explicit_depth_after_save_and_reopen() {
    let (mut state, face) = pocket();
    let wire = state.assets[0]
        .topology
        .loops
        .iter()
        .find(|wire| wire.face_id == face.id && wire.is_outer)
        .unwrap()
        .id;
    for face in &mut std::sync::Arc::make_mut(&mut state.assets[0].topology).faces {
        face.surface = CadSurface::Unsupported {
            description: "IGES spline".into(),
        };
    }
    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let selected = vec![reference(wire, EntityKind::Loop)];
    let mut entry =
        ProfileGeometryEntry::new(ProfileGeometryKind::Chain, "Closed spline rim".into());
    entry.chain_mode = ProfileChainMode::Closed;
    entry.source = ProfileGeometrySource::Cad(selected.clone());
    let binding = cam_geometry3d::resolve_selection(
        &app.setup_geometry_state().unwrap(),
        &selected,
        ExtractionRecipe::ProfileEntries {
            entries: vec![entry.clone()],
        },
        0.,
    )
    .unwrap();
    let mut operation = app.operation_template(OpKind::Profile, 1);
    let Operation::Profile(profile) = &mut operation else {
        panic!("profile")
    };
    profile.selections = Some(vec![entry]);
    let id = app
        .commit_operation_creation(
            operation,
            Some(binding),
            OperationHeightSettings::default(),
            vec![],
            StockToLeave::default(),
        )
        .unwrap();
    let mut operation = app.operation(id).unwrap().clone();
    let Operation::Profile(profile) = &mut operation else {
        panic!("profile")
    };
    profile.depth = 3.;
    app.apply_operation_settings(
        operation,
        Some(GeometryOverrides {
            top: Some(0.),
            bottom: Some(-3.),
        }),
    )
    .unwrap();
    let before = app.operation(id).unwrap().clone();
    assert!(
        matches!(&before, Operation::Profile(p) if p.closed && p.depth == 3.),
        "{before:?}"
    );
    let path =
        std::env::temp_dir().join(format!("closed-spline-profile-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = controller();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(reopened.operation(id), Some(&before));
    assert_eq!(reopened.geometry_state().bindings[&id].selected, selected);
    assert!(!reopened.has_unsaved_changes());
}
fn install_pocket(app: &mut AppController) -> EntityRef {
    let (state, r) = pocket();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(app);
    r
}

fn blind_bore_with_unrelated_surface() -> (GeometryState, EntityRef) {
    let mut fixture = Fixture::new();
    let top = fixture.face(vec![square(0., 20., 0., 20., 0.)], [0., 0., 1.]);
    let floor = fixture.face(vec![square(6., 14., 6., 14., -5.)], [0., 0., 1.]);
    fixture.bore([10., 10.], 4., top, floor);
    let floor_wire = fixture
        .t
        .faces
        .iter()
        .find(|f| f.id == floor)
        .unwrap()
        .loop_ids[1];
    fixture
        .t
        .faces
        .iter_mut()
        .find(|f| f.id == floor)
        .unwrap()
        .loop_ids = vec![floor_wire];
    fixture
        .t
        .loops
        .iter_mut()
        .find(|w| w.id == floor_wire)
        .unwrap()
        .is_outer = true;
    let blend = fixture.face(vec![square(18., 20., 18., 20., -1.)], [0., 0., 1.]);
    fixture
        .t
        .faces
        .iter_mut()
        .find(|f| f.id == blend)
        .unwrap()
        .surface = CadSurface::Unsupported {
        description: "unrelated blend away from the bore".into(),
    };
    (fixture.state(), reference(floor_wire, EntityKind::Loop))
}

#[test]
fn face_profile_and_open_chain_replace_each_other_transactionally_and_survive_reopen() {
    let mut fixture = Fixture::new();
    let top = fixture.face(vec![square(0., 20., 0., 20., 0.)], [0., 0., 1.]);
    fixture.walls(0., 20., 0., 20., -5., 0.);
    fixture.face(vec![square(0., 20., 0., 20., -5.)], [0., 0., -1.]);
    let state = fixture.state();
    let top_face = state.assets[0]
        .topology
        .faces
        .iter()
        .find(|face| face.id == top)
        .unwrap();
    let wire = state.assets[0]
        .topology
        .loops
        .iter()
        .find(|wire| wire.id == top_face.loop_ids[0])
        .unwrap();
    let edges: Vec<_> = wire
        .edge_uses
        .iter()
        .take(2)
        .map(|usage| reference(usage.edge_id, EntityKind::Edge))
        .collect();
    let mut app = controller();
    app.edit_tool(0, |tool| tool.flute_length = 10.);
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let face = reference(top, EntityKind::Face);
    let id = app
        .create_cad_operation(OpKind::Profile, vec![face.clone()])
        .unwrap();
    assert!(matches!(app.operation(id), Some(Operation::Profile(profile)) if profile.closed));
    let before = app.document().clone();
    let binding = app.preview_operation_binding(id, edges.clone()).unwrap();
    assert_eq!(binding.recipe, ExtractionRecipe::ProfileContours);
    assert_eq!(app.document(), &before);
    let mut operation = app.operation(id).unwrap().clone();
    if let Operation::Profile(profile) = &mut operation {
        profile.lead_overlap = 0.;
    }
    app.apply_operation_settings_with_geometry(operation, None, vec![], Some(edges))
        .unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Profile(profile)) if !profile.closed && profile.chain.len() == 3 && profile.side != Side::On)
    );
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    let path = std::env::temp_dir().join(format!("profile-contours-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = controller();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(matches!(reopened.operation(id), Some(Operation::Profile(profile)) if !profile.closed));
    assert_eq!(
        reopened.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
    let operation = app.operation(id).unwrap().clone();
    app.apply_operation_settings_with_geometry(operation, None, vec![], Some(vec![face]))
        .unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Profile(profile)) if profile.closed && profile.side == Side::Outside)
    );
    assert!(app.undo());
    assert!(matches!(app.operation(id), Some(Operation::Profile(profile)) if !profile.closed));
}

#[test]
fn profile_can_be_created_from_a_certified_inner_loop() {
    let mut fixture = Fixture::new();
    let rings = |z| vec![square(0., 20., 0., 20., z), square(8., 12., 8., 12., z)];
    let top = fixture.face(rings(0.), [0., 0., 1.]);
    fixture.walls(0., 20., 0., 20., -5., 0.);
    fixture.walls(8., 12., 8., 12., -5., 0.);
    fixture.face(rings(-5.), [0., 0., -1.]);
    let state = fixture.state();
    let inner_loop = state.assets[0]
        .topology
        .loops
        .iter()
        .find(|wire| wire.face_id == top && !wire.is_outer)
        .unwrap()
        .id;

    let mut app = controller();
    app.edit_tool(0, |tool| tool.flute_length = 10.);
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let id = app
        .create_cad_operation(
            OpKind::Profile,
            vec![reference(inner_loop, EntityKind::Loop)],
        )
        .unwrap();
    assert!(matches!(
        app.operation(id),
        Some(Operation::Profile(profile)) if profile.closed && profile.side == Side::Inside
    ));
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
}

#[test]
fn cad_face_contours_preserve_independent_sides_through_resolution_save_and_undo() {
    let mut fixture = Fixture::new();
    let rings = |z| vec![square(0., 20., 0., 20., z), square(8., 12., 8., 12., z)];
    let top = fixture.face(rings(0.), [0., 0., 1.]);
    fixture.walls(0., 20., 0., 20., -5., 0.);
    fixture.walls(8., 12., 8., 12., -5., 0.);
    fixture.face(rings(-5.), [0., 0., -1.]);
    let state = fixture.state();
    let mut app = controller();
    app.edit_tool(0, |tool| tool.flute_length = 10.);
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let id = app
        .create_cad_operation(OpKind::Profile, vec![reference(top, EntityKind::Face)])
        .unwrap();
    let before = app.document().clone();
    let mut operation = app.operation(id).unwrap().clone();
    let Operation::Profile(profile) = &mut operation else {
        unreachable!()
    };
    assert_eq!(profile.additional_chains.len(), 1);
    profile.side = Side::Inside;
    profile.additional_chains[0].side = Side::On;
    app.apply_operation_settings(operation, None).unwrap();
    assert!(
        matches!(app.operation(id),Some(Operation::Profile(p)) if p.side==Side::Inside && p.additional_chains[0].side==Side::On)
    );
    let outcome = app.run(&CancelToken::new());
    assert_eq!(
        outcome.completion,
        JobCompletion::Complete,
        "{:?}",
        outcome.diagnostics
    );
    let path = std::env::temp_dir().join(format!("cad-contour-sides-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = controller();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(reopened.operation(id), app.operation(id));
    assert_eq!(
        reopened.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
    assert!(app.undo());
    assert_eq!(app.document(), &before);
}

#[test]
fn inside_profile_uses_local_geometry_and_keeps_depth_verification_advisory() {
    let (state, selected) = blind_bore_with_unrelated_surface();
    let mut app = controller();
    app.edit_tool(0, |t| t.flute_length = 10.);
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    app.toggle_cad_selection(selected.clone(), false).unwrap();
    let preview = app.cad_operation_eligibility(OpKind::Profile).unwrap();
    assert_eq!(
        preview.heights,
        CuttingHeights {
            top: 0.,
            bottom: -5.
        }
    );
    let id = app
        .create_cad_operation(OpKind::Profile, vec![selected])
        .unwrap();
    assert!(matches!(app.operation(id), Some(Operation::Profile(p)) if p.side == Side::Inside));
    let before = app.document().clone();
    app.override_cad_heights(id, None, Some(-6.)).unwrap();
    let binding = &app.geometry_state().bindings[&id];
    assert_eq!(binding.effective_heights().bottom, -6.);
    let geometry = app.setup_geometry_state().unwrap();
    assert!(cam_geometry3d::resolve_binding(&geometry, binding)
        .unwrap_err()
        .contains("certified"));
    assert!(cam_geometry3d::resolve_binding_advisory(&geometry, binding)
        .unwrap()
        .diagnostics
        .iter()
        .any(|message| message.contains("verification (info)")));
    assert!(app.undo());
    assert_eq!(app.document(), &before);
}

#[test]
fn inside_profile_accepts_a_certified_top_entry_rim() {
    let mut fixture = Fixture::new();
    let top = fixture.face(vec![square(0., 20., 0., 20., 0.)], [0., 0., 1.]);
    let floor = fixture.face(vec![square(6., 14., 6., 14., -5.)], [0., 0., 1.]);
    fixture.bore([10., 10.], 4., top, floor);
    let state = fixture.state();
    let entry_loop = state.assets[0]
        .topology
        .loops
        .iter()
        .find(|wire| wire.face_id == top && !wire.is_outer)
        .unwrap()
        .id;

    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let id = app
        .create_cad_operation(
            OpKind::Profile,
            vec![reference(entry_loop, EntityKind::Loop)],
        )
        .unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Profile(profile)) if profile.side == Side::Inside)
    );
}

#[test]
fn inside_profile_reports_recognition_findings_and_rejects_unusable_curve_data() {
    for defect in [
        "blocked entry",
        "invalid solid",
        "partial rim",
        "ambiguous floor",
        "wrong setup",
    ] {
        let (mut state, selected) = blind_bore_with_unrelated_surface();
        let topology = std::sync::Arc::make_mut(&mut state.assets[0].topology);
        let reason = match defect {
            "blocked entry" => {
                topology.bodies[0].bounds[1][2] = 1.;
                // Face Contours checks local access. Enlarging only the body
                // bounds does not put material above the selected opening.
                topology.faces.push(CadFace {
                    id: 1000,
                    body_id: 1,
                    loop_ids: Vec::new(),
                    surface: CadSurface::Unsupported {
                        description: "roof above the bore".into(),
                    },
                    bounds: [[9., 9., 0.5], [11., 11., 1.]],
                });
                "blocks vertical access"
            }
            "invalid solid" => {
                topology.bodies[0].valid_solid = false;
                "invalid/open solid"
            }
            "partial rim" => {
                let wire = topology.loops.iter().find(|w| w.id == selected.id).unwrap();
                let edge = topology
                    .edges
                    .iter_mut()
                    .find(|e| e.id == wire.edge_uses[0].edge_id)
                    .unwrap();
                if let CadCurve::CircleArc { end_angle, .. } = &mut edge.curve {
                    *end_angle = std::f64::consts::PI;
                }
                "open boundary"
            }
            "ambiguous floor" => {
                let floor_id = topology
                    .loops
                    .iter()
                    .find(|w| w.id == selected.id)
                    .unwrap()
                    .face_id;
                let floor = topology
                    .faces
                    .iter_mut()
                    .find(|f| f.id == floor_id)
                    .unwrap();
                let other_loop = topology
                    .loops
                    .iter()
                    .find(|w| w.face_id == floor_id && w.id != selected.id)
                    .unwrap();
                floor.loop_ids.push(other_loop.id);
                "multiple outer boundaries"
            }
            _ => {
                state.instances[0].placement.rotation = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]];
                "not aligned with setup Z"
            }
        };
        let mut app = controller();
        app.add_cad_asset(state.assets[0].clone(), state.instances)
            .unwrap();
        select_all_models(&mut app);
        let before = app.document().clone();
        let geometry = app.setup_geometry_state().unwrap();
        let finding = cam_geometry3d::verify_selection(
            &geometry,
            std::slice::from_ref(&selected),
            ExtractionRecipe::ProfileContours,
            0.,
        )
        .unwrap();
        assert!(finding.contains(reason), "{defect}: {finding}");
        if ["blocked entry", "invalid solid", "ambiguous floor"].contains(&defect) {
            let preview = app
                .preview_cad_operation(OpKind::Profile, std::slice::from_ref(&selected))
                .unwrap();
            assert!(preview
                .diagnostics
                .iter()
                .any(|message| message.contains("verification (info)")));
            app.create_cad_operation(OpKind::Profile, vec![selected])
                .unwrap();
        } else {
            assert!(
                app.create_cad_operation(OpKind::Profile, vec![selected])
                    .is_err(),
                "{defect}"
            );
            assert_eq!(app.document(), &before);
        }
    }
}

fn cuts(program: &cam_cldata::Program) -> Vec<cam_cldata::Step> {
    program
        .steps()
        .iter()
        .filter(|s| {
            matches!(s,
                cam_cldata::Step::Linear{tag,..} | cam_cldata::Step::Arc{tag,..}
                if matches!(tag.kind,cam_cldata::MoveKind::Cutting | cam_cldata::MoveKind::Plunge)
            )
        })
        .cloned()
        .collect()
}
#[test]
fn positive_world_floor_uses_physical_depth_and_matches_an_independent_planar_cut() {
    let mut app = controller();
    let r = install_pocket(&mut app);
    app.place_cad_instance(
        1,
        RigidTransform {
            translation: [0., 0., 40.],
            ..Default::default()
        },
    )
    .unwrap();
    app.edit_heights(|h| {
        h.top_of_stock = 40.;
        h.retract = 42.;
        h.clearance = 45.;
    });
    app.edit_stock(|s| {
        *s = Stock::BoundingBox {
            x_offset: 0.,
            y_offset: 0.,
            top: 40.,
            thickness: 5.,
        }
    });
    let id = app.create_cad_operation(OpKind::Pocket, vec![r]).unwrap();
    let outcome = app.run(&CancelToken::new());
    assert_eq!(
        outcome.completion,
        JobCompletion::Complete,
        "{:?}",
        outcome.diagnostics
    );
    let actual = cuts(&outcome.program);
    assert!(!actual.is_empty());
    assert!(actual
        .iter()
        .any(|s| matches!(s,cam_cldata::Step::Linear{to,..} if (to.z-37.).abs()<1e-8)));
    // Build independent explicit 2D geometry; do not call the CAD resolver.
    let mut planar = controller();
    // Match the CAD stock top after translating this reference program by 40 mm.
    planar.edit_stock(|s| {
        *s = Stock::BoundingBox {
            x_offset: 0.,
            y_offset: 0.,
            top: 0.,
            thickness: 5.,
        };
    });
    let contour = |x0, x1, y0, y1| {
        Contour::new(vec![
            Point::new(x0, y0),
            Point::new(x1, y0),
            Point::new(x1, y1),
            Point::new(x0, y1),
        ])
    };
    planar.add_operation(Operation::Pocket(PocketOp {
        maximum_stepover: None,
        parameter_expressions: Default::default(),
        wrap: None,
        additional_regions: Vec::new(),
        stock_contours: Default::default(),
        finishing: Default::default(),
        id,
        tool: 1,
        work_offset: 1,
        spindle_rpm: 1000.,
        tool_preset: None,
        cooling: Default::default(),
        entry_feeds: Default::default(),
        boundary: contour(2., 18., 2., 18.),
        islands: vec![contour(8., 12., 8., 12.)],
        depth: 3.,
        end: None,
        start: None,
        clear: ClearParams {
            multiple_depths: Default::default(),
            adaptive_enabled: None,
            pocket_overlap: None,
            keep_tool_down: false,
            lift_height: 0.2,
            stepdown: 2.,
            overlap: 0.5,
            offset: 0.,
            feed: 300.,
            plunge_feed: 100.,
            plunge: Plunge::Straight,
            lead_overlap: 0.,
            lead_in: Lead::Arc { radius: 0.6 },
            lead_out: Lead::Arc { radius: 0.6 },
            clearing: Clearing::default(),
        },
    }));
    let expected = cuts(
        &planar
            .run(&CancelToken::new())
            .program
            .translated([0., 0., 40.]),
    );
    assert_eq!(actual, expected);
}
#[test]
fn embedded_assets_bindings_and_overrides_reopen_with_identical_output() {
    let mut app = controller();
    let r = install_pocket(&mut app);
    let id = app.create_cad_operation(OpKind::Pocket, vec![r]).unwrap();
    app.override_cad_heights(id, None, Some(-2.)).unwrap();
    let before = app.run(&CancelToken::new()).program.clone();
    let path = std::env::temp_dir().join(format!("cad-roundtrip-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = controller();
    reopened.open_project(&path).unwrap();
    assert_eq!(app.document(), reopened.document());
    assert_eq!(
        reopened.document().geometry.bindings[&id].overrides.bottom,
        Some(-2.)
    );
    assert_eq!(before, reopened.run(&CancelToken::new()).program);
    std::fs::remove_file(path).unwrap();
}

#[test]
fn a_cad_linked_operation_cannot_reopen_with_its_binding_removed() {
    let mut app = controller();
    let selected = install_pocket(&mut app);
    let operation = app
        .create_cad_operation(OpKind::Pocket, vec![selected])
        .unwrap();
    let path =
        std::env::temp_dir().join(format!("cad-missing-binding-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut project: serde_json::Value =
        serde_json::from_str(&cam_app::OcamFile::read(&path).unwrap().to_json().unwrap()).unwrap();
    project["document"]["geometry"]["bindings"]
        .as_object_mut()
        .unwrap()
        .remove(&operation.to_string());
    std::fs::write(&path, project.to_string()).unwrap();

    let mut reopened = controller();
    let error = reopened.open_project(&path).unwrap_err().to_string();
    std::fs::remove_file(path).unwrap();
    assert!(error.contains("missing its geometry binding"), "{error}");
}

#[test]
fn a_persisted_binding_cannot_reopen_after_its_linked_marker_is_removed() {
    let mut app = controller();
    let selected = install_pocket(&mut app);
    let operation = app
        .create_cad_operation(OpKind::Pocket, vec![selected])
        .unwrap();
    let path = std::env::temp_dir().join(format!("cad-missing-marker-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut project: serde_json::Value =
        serde_json::from_str(&cam_app::OcamFile::read(&path).unwrap().to_json().unwrap()).unwrap();
    assert!(project["document"]["geometry"]["bindings"][operation.to_string()].is_object());
    project["document"]["cad_linked_operations"] = serde_json::json!([]);
    std::fs::write(&path, project.to_string()).unwrap();

    let mut reopened = controller();
    let error = reopened.open_project(&path).unwrap_err().to_string();
    std::fs::remove_file(path).unwrap();
    assert!(error.contains("not marked as CAD-linked"), "{error}");
}

#[test]
fn a_rotated_setup_resolves_linked_cad_geometry_in_its_own_cutting_frame() {
    let mut app = controller();
    let floor = install_pocket(&mut app);
    let first_id = app
        .create_cad_operation(OpKind::Pocket, vec![floor])
        .unwrap();
    let first = app.run(&CancelToken::new());
    assert_eq!(
        first.completion,
        JobCompletion::Complete,
        "{:?}",
        first.diagnostics
    );
    let first_cut_x: Vec<_> = cuts(&first.program)
        .iter()
        .filter_map(|step| match step {
            cam_cldata::Step::Linear { to, .. } => Some(to.x),
            cam_cldata::Step::Arc { end, .. } => Some(end.x),
            _ => None,
        })
        .collect();
    assert!(!first_cut_x.is_empty());
    assert!(first_cut_x.iter().any(|x| *x > 0.0));

    let first_setup = app.active_setup_id();
    let rotated_id = app.duplicate_setup(first_setup).unwrap();
    let mut rotated = app
        .setups()
        .iter()
        .find(|s| s.id == rotated_id)
        .unwrap()
        .clone();
    rotated.frame =
        SetupFrame::from_axes(Some([-1.0, 0.0, 0.0]), Some([0.0, -1.0, 0.0]), None).unwrap();
    rotated.stock = Stock::BoundingBox {
        x_offset: 0.0,
        y_offset: 0.0,
        top: 0.0,
        thickness: 6.0,
    };
    app.apply_setup_settings(rotated).unwrap();
    let duplicated_op = app.document().setup().operations[0].id();
    assert_ne!(first_id, duplicated_op);
    assert!(app
        .document()
        .geometry
        .bindings
        .contains_key(&duplicated_op));

    let second = app.run(&CancelToken::new());
    assert_eq!(
        second.completion,
        JobCompletion::Complete,
        "{:?}",
        second.diagnostics
    );
    let rotated_cut_xy: Vec<_> = cuts(&second.program)
        .iter()
        .filter_map(|step| match step {
            cam_cldata::Step::Linear { to, .. } => Some([to.x, to.y]),
            cam_cldata::Step::Arc { end, .. } => Some([end.x, end.y]),
            _ => None,
        })
        .collect();
    assert!(!rotated_cut_xy.is_empty());
    assert!(
        rotated_cut_xy.iter().any(|p| p[0] < 0.0 && p[1] < 0.0),
        "the linked pocket should be resolved in the rotated setup frame"
    );
}

#[test]
fn model_coordinate_system_tracks_instance_placement_and_undo_restores_both() {
    let mut app = controller();
    let _ = install_pocket(&mut app);
    let instance_id = app.geometry_state().instances[0].id;

    let (frame, origin) = app.pending_frame_from_model(instance_id).unwrap();
    let mut settings = app.document().setup().clone();
    settings.name = "Model WCS".into();
    settings.frame = frame;
    settings.origin = origin;
    settings.stock = Stock::RelativeBox {
        offsets: [[1.0, 1.0, 2.0], [1.0, 1.0, 2.0]],
    };
    settings.heights = Heights::new(12.0, 5.0, 2.0);
    app.apply_setup_settings(settings).unwrap();
    assert_eq!(
        app.document().setup().frame.references.model_instance,
        Some(instance_id)
    );
    assert!(matches!(
        app.document().setup().frame.references.origin,
        FrameOrigin::ModelOrigin
    ));

    let rotated_x = RigidTransform {
        rotation: [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        translation: [10.0, 20.0, 30.0],
    };
    app.place_cad_instance(instance_id, rotated_x).unwrap();
    assert_eq!(
        app.geometry_state()
            .instance(instance_id)
            .unwrap()
            .placement,
        rotated_x
    );
    assert_eq!(
        app.document().setup().frame.world_from_setup.rotation,
        rotated_x.rotation
    );
    assert_eq!(app.document().setup().origin, [10.0, 30.0, -20.0]);

    let restored_instance = app
        .geometry_state()
        .instance(instance_id)
        .unwrap()
        .placement;
    let restored_frame = app.document().setup().frame.clone();
    let restored_origin = app.document().setup().origin;
    let rotated_z = RigidTransform {
        rotation: [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
        translation: [15.0, 25.0, 35.0],
    };
    app.place_cad_instance(instance_id, rotated_z).unwrap();
    assert_eq!(
        app.document().setup().frame.world_from_setup.rotation,
        rotated_z.rotation
    );
    assert_eq!(app.document().setup().origin, [-15.0, -25.0, 35.0]);
    assert!(
        app.undo(),
        "instance placement and associated model WCS share one history edit"
    );
    assert_eq!(
        app.geometry_state()
            .instance(instance_id)
            .unwrap()
            .placement,
        restored_instance
    );
    assert_eq!(app.document().setup().frame, restored_frame);
    assert_eq!(app.document().setup().origin, restored_origin);

    let mut explicit = app.document().setup().clone();
    explicit.frame.references.origin = FrameOrigin::Explicit([4.0, 5.0, 6.0]);
    explicit.origin = [4.0, 5.0, 6.0];
    app.apply_setup_settings(explicit).unwrap();
    let explicit_before_move = app.document().setup().origin;
    let rotated_y = RigidTransform {
        rotation: [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
        translation: [11.0, 12.0, 13.0],
    };
    app.place_cad_instance(instance_id, rotated_y).unwrap();
    assert_eq!(
        app.geometry_state()
            .instance(instance_id)
            .unwrap()
            .placement,
        rotated_y
    );
    assert_eq!(
        app.document().setup().frame.world_from_setup.rotation,
        rotated_y.rotation
    );
    assert_eq!(
        app.document().setup().origin,
        explicit_before_move,
        "an explicit datum must remain fixed when the shared model moves"
    );
    assert!(matches!(
        app.document().setup().frame.references.origin,
        FrameOrigin::Explicit([4.0, 5.0, 6.0])
    ));
    assert!(app.undo());
    assert_eq!(
        app.geometry_state()
            .instance(instance_id)
            .unwrap()
            .placement,
        restored_instance
    );
}

#[test]
fn replacement_requires_review_and_undo_restores_links_and_export_eligibility() {
    let mut app = controller();
    let r = install_pocket(&mut app);
    let id = app
        .create_cad_operation(OpKind::Pocket, vec![r.clone()])
        .unwrap();
    app.override_cad_heights(id, None, Some(-2.)).unwrap();
    let replacement = app.geometry_state().assets[0].clone();
    app.replace_cad_asset(1, replacement).unwrap();
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Failed
    );
    assert!(matches!(
        app.export_nc(),
        Err(ExportError::HasErrors) | Err(ExportError::Incomplete)
    ));
    app.set_operation_excluded(id, true);
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
    app.set_operation_excluded(id, false);
    let mut new_ref = r.clone();
    new_ref.revision = 2;
    app.edit_cad_geometry(id, vec![new_ref]).unwrap();
    assert_eq!(
        app.document().geometry.bindings[&id].overrides.bottom,
        Some(-2.)
    );
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
    assert!(app.undo()); // undo reselection
    assert!(matches!(
        app.document().geometry.bindings[&id].status,
        ResolutionStatus::NeedsReselection { .. }
    ));
    assert!(app.undo()); // undo re-inclusion
    assert!(app.is_operation_excluded(id));
    assert!(app.undo()); // undo suppression
    assert!(!app.is_operation_excluded(id));
    assert!(app.undo()); // undo replacement
    assert_eq!(app.document().geometry.assets[0].revision, 1);
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
}
#[test]
fn asset_and_binding_duplicate_delete_undo_together() {
    let mut app = controller();
    let r = install_pocket(&mut app);
    assert!(app.undo()); // setup body selection
    assert!(app.document().setup().model_instances.is_empty());
    assert!(app.undo()); // import
    assert!(app.geometry_state().assets.is_empty());
    assert!(app.redo());
    assert!(app.redo());
    let id = app.create_cad_operation(OpKind::Pocket, vec![r]).unwrap();
    app.duplicate_selected_operation();
    assert_eq!(app.geometry_state().bindings.len(), 2);
    app.delete_selected_operation();
    assert_eq!(app.geometry_state().bindings.len(), 1);
    assert!(app.geometry_state().bindings.contains_key(&id));
    assert!(app.undo());
    assert_eq!(app.geometry_state().bindings.len(), 2);
    assert!(app.remove_cad_asset(1));
    assert!(app.document().setup().model_instances.is_empty());
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Failed
    );
    assert!(app.undo());
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
}
#[test]
fn direct_planner_rejects_linked_geometry_without_a_matching_resolution() {
    let mut app = controller();
    let r = install_pocket(&mut app);
    app.create_cad_operation(OpKind::Pocket, vec![r]).unwrap();
    let job = cam_toolpath::build_job(
        app.document(),
        1000.,
        cam_cldata::SpindleDir::Cw,
        None,
        50.,
        &CancelToken::new(),
    );
    assert_eq!(job.completion, JobCompletion::Failed);
    assert!(job
        .diagnostics
        .iter()
        .any(|d| d.message.contains("resolved snapshot")));
}
#[test]
fn editing_translated_linked_depth_keeps_a_physical_magnitude() {
    let mut app = controller();
    let r = install_pocket(&mut app);
    app.place_cad_instance(
        1,
        RigidTransform {
            translation: [0., 0., 40.],
            ..Default::default()
        },
    )
    .unwrap();
    app.edit_heights(|h| {
        h.top_of_stock = 40.;
        h.retract = 42.;
        h.clearance = 45.;
    });
    let id = app.create_cad_operation(OpKind::Pocket, vec![r]).unwrap();
    app.edit_operation(id, |op| {
        if let Operation::Pocket(p) = op {
            p.depth = 2.;
        }
    });
    assert_eq!(
        app.document().geometry.bindings[&id].overrides.bottom,
        Some(38.)
    );
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
}

#[test]
fn committing_operation_depth_with_unchanged_link_fields_retains_the_override() {
    let mut app = controller();
    let reference = install_pocket(&mut app);
    let id = app
        .create_cad_operation(OpKind::Pocket, vec![reference])
        .unwrap();
    let mut draft = app.operation(id).unwrap().clone();
    let unchanged_links = app.document().geometry.bindings[&id].overrides.clone();
    if let Operation::Pocket(operation) = &mut draft {
        operation.depth = 2.;
    }
    app.apply_operation_settings(draft, Some(unchanged_links))
        .unwrap();
    assert_eq!(
        app.document().geometry.bindings[&id].overrides.bottom,
        Some(-2.)
    );
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
    let mut draft = app.operation(id).unwrap().clone();
    if let Operation::Pocket(operation) = &mut draft {
        operation.clear.feed = 250.;
    }
    app.apply_operation_settings(draft, Some(GeometryOverrides::default()))
        .unwrap();
    assert_eq!(app.document().geometry.bindings[&id].overrides.bottom, None);
}
#[test]
fn background_import_previews_cancel_and_reject_project_edits_without_losing_work() {
    let path = std::env::temp_dir().join(format!("cad-job-{}.dxf", std::process::id()));
    let drawing =
        "0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n0\n20\n0\n11\n20\n21\n0\n0\nENDSEC\n0\nEOF\n";
    std::fs::write(&path, drawing).unwrap();
    let mut app = controller();
    let original = app.document().clone();
    app.start_cad_import(path.clone());
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    loop {
        if let Some(result) = app.poll_cad_import() {
            result.unwrap();
            break;
        }
        assert!(std::time::Instant::now() < deadline);
        std::thread::yield_now();
    }
    assert_eq!(app.document(), &original);
    assert!(app.cad_import_preview().is_some());
    app.cancel_cad_import();
    assert!(app.cad_import_preview().is_none());
    assert_eq!(app.document(), &original);
    app.start_cad_import(path.clone());
    app.edit_heights(|h| h.clearance = 10.);
    assert!(app.poll_cad_import().unwrap().is_err());
    assert!(app.open_paths().is_empty());
    app.start_cad_import(path.clone());
    loop {
        if let Some(result) = app.poll_cad_import() {
            result.unwrap();
            break;
        }
        assert!(std::time::Instant::now() < deadline);
        std::thread::yield_now();
    }
    app.edit_heights(|h| h.retract = 3.);
    assert!(app.commit_cad_preview().is_err());
    assert!(app.open_paths().is_empty());
    std::fs::remove_file(path).unwrap();
}

#[test]
fn deleted_asset_id_is_never_reused_for_an_unrelated_import() {
    let mut app = controller();
    let reference = install_pocket(&mut app);
    let asset = app.geometry_state().assets[0].clone();
    let instances = app.geometry_state().instances.clone();
    assert!(app.remove_cad_asset(asset.id));
    app.add_cad_asset(asset, instances).unwrap();
    select_all_models(&mut app);
    assert_eq!(app.geometry_state().assets[0].id, 2);
    assert!(!app.geometry_state().entity_exists(&reference));
    assert_eq!(app.geometry_state().instances[0].id, 2);
}

#[test]
fn cad_feature_and_operation_first_resolve_and_create_equivalent_inputs() {
    let mut feature_first = controller();
    let feature = install_pocket(&mut feature_first);
    feature_first
        .toggle_cad_selection(feature.clone(), false)
        .unwrap();
    let kind = OpKind::Pocket;
    let before = feature_first.cad_operation_eligibility(kind).unwrap();
    let first_id = feature_first
        .create_cad_operation_with_tool(kind, feature_first.cad_selection().to_vec(), 1)
        .unwrap();

    let mut operation_first = controller();
    install_pocket(&mut operation_first);
    assert!(operation_first
        .cad_operation_eligibility(kind)
        .unwrap_err()
        .contains("select geometry"));
    operation_first
        .toggle_cad_selection(feature, false)
        .unwrap();
    assert_eq!(
        before,
        operation_first.cad_operation_eligibility(kind).unwrap()
    );
    let second_id = operation_first
        .create_cad_operation_with_tool(kind, operation_first.cad_selection().to_vec(), 1)
        .unwrap();
    assert_eq!(
        feature_first.operation(first_id),
        operation_first.operation(second_id)
    );
    assert_eq!(
        feature_first.geometry_state().bindings[&first_id],
        operation_first.geometry_state().bindings[&second_id]
    );
    assert_eq!(
        cuts(&feature_first.run(&CancelToken::new()).program),
        cuts(&operation_first.run(&CancelToken::new()).program)
    );
}

#[test]
fn cad_selection_keeps_mixed_features_and_only_rejects_stale_references() {
    let mut app = controller();
    let floor = install_pocket(&mut app);
    let topology = &app.geometry_state().assets[0].topology;
    let other = reference(
        topology.faces.iter().find(|f| f.id != floor.id).unwrap().id,
        EntityKind::Face,
    );
    let edge = reference(topology.edges[0].id, EntityKind::Edge);
    app.toggle_cad_selection(floor.clone(), false).unwrap();
    app.toggle_cad_selection(edge.clone(), true).unwrap();
    assert_eq!(app.cad_selection(), &[floor.clone(), edge.clone()]);
    app.toggle_cad_selection(edge, true).unwrap();
    app.toggle_cad_selection(other.clone(), true).unwrap();
    assert_eq!(app.cad_selection(), &[floor.clone(), other]);
    let before = app.document().clone();
    assert!(app
        .cad_operation_eligibility(OpKind::Pocket)
        .unwrap_err()
        .contains("planar"));
    assert!(app
        .create_cad_operation(OpKind::Pocket, app.cad_selection().to_vec())
        .is_err());
    assert_eq!(app.document(), &before);
    let mut stale = floor.clone();
    stale.revision += 1;
    assert!(app
        .toggle_cad_selection(stale.clone(), true)
        .unwrap_err()
        .contains("stale"));
    assert_eq!(app.cad_selection().len(), 2);
    app.set_cad_selection(vec![floor, stale]);
    assert_eq!(
        app.cad_selection().len(),
        2,
        "invalid members are never silently dropped"
    );
    assert!(app
        .cad_operation_eligibility(OpKind::Pocket)
        .unwrap_err()
        .contains("stale"));
    assert!(app
        .preview_cad_operation(OpKind::Carve, &app.cad_selection()[..1])
        .is_ok());
    for kind in [OpKind::Chamfer, OpKind::Thread, OpKind::Engrave] {
        assert!(app
            .preview_cad_operation(kind, &app.cad_selection()[..1])
            .is_err());
    }
}

#[test]
fn selections_can_include_features_from_different_body_instances() {
    let mut app = controller();
    let floor = install_pocket(&mut app);
    let mut second = app.geometry_state().instances[0].clone();
    second.id = 2;
    let mut asset = app.geometry_state().assets[0].clone();
    asset.id = 2;
    second.asset_id = 2;
    app.add_cad_asset(asset, vec![second]).unwrap();
    select_all_models(&mut app);
    let mut second_floor = floor.clone();
    second_floor.asset_id = 2;
    second_floor.instance_id = 2;
    app.toggle_cad_selection(floor.clone(), false).unwrap();
    app.toggle_cad_selection(second_floor.clone(), true)
        .unwrap();
    assert_eq!(app.cad_selection(), &[floor, second_floor]);
    let profile = app
        .preview_cad_operation(OpKind::Profile, app.cad_selection())
        .unwrap();
    assert_eq!(profile.profiles.len(), 4);
}

#[test]
fn persisted_mixed_geometry_cannot_generate_or_export_using_cached_contours() {
    let mut app = controller();
    let floor = install_pocket(&mut app);
    let edge = reference(
        app.geometry_state().assets[0].topology.edges[0].id,
        EntityKind::Edge,
    );
    let id = app
        .create_cad_operation(OpKind::Pocket, vec![floor])
        .unwrap();
    assert_eq!(
        app.run(&CancelToken::new()).completion,
        JobCompletion::Complete
    );
    let path =
        std::env::temp_dir().join(format!("cad-mixed-selection-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut project: serde_json::Value =
        serde_json::from_str(&cam_app::OcamFile::read(&path).unwrap().to_json().unwrap()).unwrap();
    project["document"]["geometry"]["bindings"][id.to_string()]["selected"]
        .as_array_mut()
        .unwrap()
        .push(serde_json::to_value(edge).unwrap());
    std::fs::write(&path, project.to_string()).unwrap();
    let mut reopened = controller();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    let outcome = reopened.run(&CancelToken::new());
    assert_eq!(outcome.completion, JobCompletion::Failed);
    assert!(outcome
        .diagnostics
        .iter()
        .any(|d| d.message.contains("planar")));
    assert!(cuts(&outcome.program).is_empty());
    assert!(matches!(reopened.export_nc(), Err(ExportError::HasErrors)));
}

#[test]
fn linked_drill_previews_staged_cutter_and_bore_then_commits_in_one_undo() {
    let mut fixture = Fixture::new();
    let top = fixture.face(vec![square(0., 20., 0., 20., 0.)], [0., 0., 1.]);
    let bottom = fixture.face(vec![square(0., 20., 0., 20., -5.)], [0., 0., -1.]);
    fixture.walls(0., 20., 0., 20., -5., 0.);
    let old_bore = reference(fixture.bore([5., 10.], 3., top, bottom), EntityKind::Face);
    let new_bore = reference(fixture.bore([15., 10.], 4., top, bottom), EntityKind::Face);
    let state = fixture.state();
    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let old_tool = app.use_tool(Tool {
        number: 10,
        diameter: 6.,
        length: 30.,
        flute_length: 20.,
        kind: ToolKind::Drill {
            point_angle_deg: 118.,
        },
        ..Default::default()
    });
    let id = app
        .create_cad_operation_with_tool(OpKind::Drill, vec![old_bore], old_tool)
        .unwrap();
    let before = app.document().clone();
    let revision = app.revision();
    let mut staged_tool = app
        .document()
        .setup()
        .tools
        .iter()
        .find(|t| t.number == old_tool)
        .unwrap()
        .clone();
    staged_tool.number = app
        .document()
        .setup()
        .tools
        .iter()
        .map(|t| t.number)
        .max()
        .unwrap()
        + 1;
    staged_tool.diameter = 8.;
    let mut draft = app.operation(id).unwrap().clone();
    if let Operation::Drill(operation) = &mut draft {
        operation.tool = staged_tool.number;
        operation.feed = 173.;
    } else {
        panic!("expected linked drill");
    }
    let overrides = Some(GeometryOverrides {
        top: None,
        bottom: Some(-4.),
    });
    // The committed cutter cannot qualify the replacement bore by itself.
    assert!(app
        .preview_operation_geometry(id, vec![new_bore.clone()])
        .unwrap_err()
        .contains("diameter"));
    let binding = app
        .preview_operation_settings_with_geometry(
            draft.clone(),
            overrides.clone(),
            vec![staged_tool.clone()],
            vec![new_bore.clone()],
        )
        .unwrap();
    assert_eq!(binding.selected, vec![new_bore.clone()]);
    assert_eq!(binding.effective_heights().bottom, -4.);
    assert_eq!(
        app.document(),
        &before,
        "preview never publishes draft state"
    );
    assert_eq!(app.revision(), revision);
    assert!(app
        .apply_operation_settings_with_geometry(
            draft.clone(),
            overrides.clone(),
            vec![],
            Some(vec![new_bore.clone()]),
        )
        .unwrap_err()
        .contains("missing tool"));
    assert_eq!(app.document(), &before, "rejected transaction is atomic");
    app.apply_operation_settings_with_geometry(
        draft,
        overrides,
        vec![staged_tool.clone()],
        Some(vec![new_bore.clone()]),
    )
    .unwrap();
    let Operation::Drill(committed) = app.operation(id).unwrap() else {
        panic!("expected linked drill");
    };
    assert_eq!(committed.tool, staged_tool.number);
    assert_eq!(committed.points, vec![[15., 10.]]);
    assert_eq!(committed.depth, 4.);
    assert_eq!(committed.feed, 173.);
    assert_eq!(app.geometry_state().bindings[&id].selected, vec![new_bore]);
    assert!(app.document().setup().tools.contains(&staged_tool));
    assert!(app.undo());
    assert_eq!(
        app.document(),
        &before,
        "one undo restores cutter, source and all tabs"
    );
}

#[test]
fn single_model_edge_uses_current_orientation_and_applies_without_other_axis_picks() {
    for axis in 0..3 {
        let mut app = controller();
        install_pocket(&mut app);
        let mut settings = app.document().setup().clone();
        settings.frame = SetupFrame::from_xy([0., 1., 0.], [0., 0., 1.]).unwrap();
        app.apply_setup_settings(settings).unwrap();
        let previous = app.document().setup().clone();
        let geometry = app.geometry_state().clone();
        let edge = geometry.assets[0].topology.edges[0].clone();
        let selected = reference(edge.id, EntityKind::Edge);
        let mut picks = [None, None, None];
        picks[axis] = Some(selected.clone());
        let frame = app.pending_frame_from_edges(picks, [false; 3]).unwrap();
        frame.validate().unwrap();
        let CadCurve::Line { start, end } = edge.curve else {
            panic!("fixture edge must be straight")
        };
        let direction: [f64; 3] = std::array::from_fn(|i| end[i] - start[i]);
        let length = direction.iter().map(|v| v * v).sum::<f64>().sqrt();
        let actual =
            frame.to_world_vector(std::array::from_fn(|i| if i == axis { 1. } else { 0. }));
        assert!((0..3).all(|i| (actual[i] - direction[i] / length).abs() < 1e-9));
        let mut setup = previous.clone();
        setup.frame = frame;
        app.apply_setup_settings(setup.clone()).unwrap();
        assert_eq!(app.document().setup(), &setup);
        assert_eq!(app.geometry_state(), &geometry);
        let loaded: Document =
            serde_json::from_str(&serde_json::to_string(app.document()).unwrap()).unwrap();
        assert_eq!(loaded.setup().frame, setup.frame);
        assert!(app.undo());
        assert_eq!(app.document().setup(), &previous);
        assert!(app.redo());
        assert_eq!(app.document().setup(), &setup);
    }
}

#[test]
fn pocket_and_drill_report_obstructed_approaches_as_advisory_findings() {
    for kind in [OpKind::Pocket, OpKind::Drill] {
        let (state, floor_loop) = blind_bore_with_unrelated_surface();
        let wire = state.assets[0]
            .topology
            .loops
            .iter()
            .find(|w| w.id == floor_loop.id)
            .unwrap();
        let id = if kind == OpKind::Pocket {
            wire.face_id
        } else {
            state.assets[0]
                .topology
                .faces
                .iter()
                .find(|f| matches!(f.surface, CadSurface::Cylinder { inward: true, .. }))
                .unwrap()
                .id
        };
        let selected = reference(id, EntityKind::Face);
        let mut app = controller();
        app.edit_tool(0, |t| t.flute_length = 10.);
        app.add_cad_asset(state.assets[0].clone(), state.instances.clone())
            .unwrap();
        select_all_models(&mut app);
        let preview = app
            .preview_cad_operation(kind, std::slice::from_ref(&selected))
            .unwrap();
        assert_eq!(
            preview.heights,
            CuttingHeights {
                top: 0.,
                bottom: -5.
            }
        );
        let tool = if kind == OpKind::Drill {
            app.use_tool(Tool {
                number: 7,
                diameter: 8.,
                length: 30.,
                flute_length: 10.,
                kind: ToolKind::Drill {
                    point_angle_deg: 118.,
                },
                ..Default::default()
            })
        } else {
            1
        };
        app.create_cad_operation_with_tool(kind, vec![selected.clone()], tool)
            .unwrap();
        let outcome = app.run(&CancelToken::new());
        assert_eq!(outcome.completion, JobCompletion::Complete);
        let result = app.export_nc();
        assert!(result.is_ok(), "{result:?}");

        // An unknown curved roof is ignored only when its exact conservative
        // bounds prove it cannot block this feature, including after rotation.
        for blocked in [false, true] {
            let mut state = state.clone();
            let topology = std::sync::Arc::make_mut(&mut state.assets[0].topology);
            let blend = topology
                .faces
                .iter_mut()
                .find(|f| matches!(f.surface, CadSurface::Unsupported { .. }))
                .unwrap();
            blend.bounds = if blocked {
                [[9., 9., 1.], [11., 11., 2.]]
            } else {
                [[18., 18., 1.], [20., 20., 2.]]
            };
            state.instances[0].placement = RigidTransform {
                rotation: [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]],
                translation: [100., -50., 40.],
            };
            let mut app = controller();
            app.add_cad_asset(state.assets[0].clone(), state.instances)
                .unwrap();
            select_all_models(&mut app);
            let result = app.preview_cad_operation(kind, std::slice::from_ref(&selected));
            if blocked {
                let preview = result.unwrap();
                assert!(preview
                    .diagnostics
                    .iter()
                    .any(|message| message.contains("blocks vertical access")));
            } else {
                assert!(result.is_ok(), "{result:?}");
            }
        }
    }
}

fn vbit(app: &mut AppController) -> u32 {
    app.use_tool(Tool {
        number: 20,
        diameter: 6.,
        length: 30.,
        flute_length: 10.,
        kind: ToolKind::VBit {
            included_angle_deg: 90.,
            tip_radius: 0.,
        },
        ..Default::default()
    })
}

#[test]
fn surface_operations_select_exact_geometry_at_the_placed_surface_and_reopen() {
    for kind in [OpKind::Chamfer, OpKind::Engrave, OpKind::Carve] {
        let mut app = controller();
        let floor = install_pocket(&mut app);
        let wire = app.geometry_state().assets[0]
            .topology
            .faces
            .iter()
            .find(|f| f.id == floor.id)
            .unwrap()
            .loop_ids[0];
        let geometry = if kind == OpKind::Carve {
            floor
        } else {
            reference(wire, EntityKind::Loop)
        };
        app.place_cad_instance(
            1,
            RigidTransform {
                translation: [5., 7., 10.],
                ..Default::default()
            },
        )
        .unwrap();
        let tool = vbit(&mut app);
        let id = app
            .create_cad_operation_with_tool(kind, vec![geometry.clone()], tool)
            .unwrap();
        match app.operation(id).unwrap() {
            Operation::Chamfer(op) => {
                assert_eq!(op.top, 7.);
                assert_eq!(op.side, Side::Outside);
            }
            Operation::Engrave(op) => {
                assert_eq!(op.top, 7.);
                assert!(op.closed);
                assert!((op.depth - 0.3).abs() < 1e-8);
            }
            Operation::Carve(op) => {
                assert_eq!(op.top, 7.);
                assert_eq!(op.islands.len(), 1);
            }
            _ => unreachable!(),
        }
        let heights = app.document().setup().height_settings[&id]
            .resolve(app.document(), app.operation(id).unwrap())
            .unwrap();
        assert_eq!(heights.top, 7.);
        assert!(heights.bottom < heights.top);
        let outcome = app.run(&CancelToken::new());
        assert_eq!(
            outcome.completion,
            JobCompletion::Complete,
            "{kind:?}: {:?}",
            outcome.diagnostics
        );
        assert!(!outcome.has_errors(), "{kind:?}: {:?}", outcome.diagnostics);
        assert!(!outcome.program.is_empty());
        let program = outcome.program.clone();
        let path =
            std::env::temp_dir().join(format!("surface-{kind:?}-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = controller();
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert_eq!(
            reopened.geometry_state().bindings[&id].selected,
            vec![geometry]
        );
        assert_eq!(reopened.run(&CancelToken::new()).program, program);
    }
}

#[test]
fn engraving_open_chain_replacement_preserves_parameters_and_chamfer_accepts_it() {
    let mut app = controller();
    let floor = install_pocket(&mut app);
    let topology = &app.geometry_state().assets[0].topology;
    let wire = topology
        .faces
        .iter()
        .find(|f| f.id == floor.id)
        .unwrap()
        .loop_ids[0];
    let edges: Vec<_> = topology
        .loops
        .iter()
        .find(|w| w.id == wire)
        .unwrap()
        .edge_uses
        .iter()
        .take(2)
        .map(|u| reference(u.edge_id, EntityKind::Edge))
        .collect();
    let tool = vbit(&mut app);
    let id = app
        .create_cad_operation_with_tool(
            OpKind::Engrave,
            vec![reference(wire, EntityKind::Loop)],
            tool,
        )
        .unwrap();
    let before = app.document().clone();
    let mut operation = app.operation(id).unwrap().clone();
    if let Operation::Engrave(op) = &mut operation {
        op.feed = 245.;
        op.depth = 0.6;
    }
    app.apply_operation_settings_with_geometry(operation, None, vec![], Some(edges.clone()))
        .unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Engrave(op)) if !op.closed && op.chain.len() == 3 && op.feed == 245. && (op.depth - 0.6).abs() < 1e-8)
    );
    let chamfer = app.preview_cad_operation(OpKind::Chamfer, &edges).unwrap();
    assert!(!chamfer.profiles[0].closed);
    assert_eq!(chamfer.profiles[0].chain.len(), 3);
    assert!(app.undo());
    assert_eq!(app.document(), &before);
}

#[test]
fn thread_selection_resolves_multiple_centres_and_keeps_the_nominal_diameter_editable() {
    let mut fixture = Fixture::new();
    let top = fixture.face(vec![square(0., 20., 0., 20., 0.)], [0., 0., 1.]);
    let bottom = fixture.face(vec![square(0., 20., 0., 20., -5.)], [0., 0., -1.]);
    fixture.walls(0., 20., 0., 20., -5., 0.);
    let bores = vec![
        reference(fixture.bore([5., 10.], 3., top, bottom), EntityKind::Face),
        reference(fixture.bore([15., 10.], 3., top, bottom), EntityKind::Face),
    ];
    let state = fixture.state();
    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let tool = app.use_tool(Tool {
        number: 30,
        diameter: 3.,
        neck_diameter: 1.5,
        flute_length: 12.,
        length: 30.,
        kind: ToolKind::ThreadMill { pitch: None },
        ..Default::default()
    });
    let id = app
        .create_cad_operation_with_tool(OpKind::Thread, bores.clone(), tool)
        .unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Thread(op)) if op.points == vec![[5., 10.], [15., 10.]] && op.internal && op.z_top == 0. && op.z_bottom == -5. && op.major_dia == 6.)
    );
    let mut operation = app.operation(id).unwrap().clone();
    if let Operation::Thread(op) = &mut operation {
        op.major_dia = 6.8;
        op.pitch = 0.8;
    }
    app.apply_operation_settings_with_geometry(
        operation,
        None,
        vec![],
        Some(vec![bores[1].clone()]),
    )
    .unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Thread(op)) if op.points == vec![[15., 10.]] && op.major_dia == 6.8 && op.pitch == 0.8)
    );
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    assert!(!result.has_errors(), "{:?}", result.diagnostics);

    // Nominal size can be enlarged independently of the prepared hole. A mill
    // fitting that nominal thread still has to enter the selected 6 mm bore.
    let oversized = app.use_tool(Tool {
        number: 31,
        diameter: 7.,
        neck_diameter: 3.,
        flute_length: 12.,
        length: 30.,
        kind: ToolKind::ThreadMill { pitch: None },
        ..Default::default()
    });
    let before = app.document().clone();
    let mut invalid = app.operation(id).unwrap().clone();
    if let Operation::Thread(op) = &mut invalid {
        op.tool = oversized;
        op.major_dia = 12.;
    }
    assert!(app
        .apply_operation_settings(invalid, None)
        .unwrap_err()
        .contains("cannot enter the selected bore"));
    assert_eq!(app.document(), &before);
}

#[test]
fn surface_selection_rejects_disconnected_nonplanar_and_stale_paths() {
    let mut app = controller();
    let floor = install_pocket(&mut app);
    let topology = &app.geometry_state().assets[0].topology;
    let wire = topology
        .faces
        .iter()
        .find(|f| f.id == floor.id)
        .unwrap()
        .loop_ids[0];
    let edges: Vec<_> = topology
        .loops
        .iter()
        .find(|w| w.id == wire)
        .unwrap()
        .edge_uses
        .iter()
        .map(|u| reference(u.edge_id, EntityKind::Edge))
        .collect();
    assert!(app
        .preview_cad_operation(OpKind::Engrave, &[edges[0].clone(), edges[2].clone()])
        .unwrap_err()
        .contains("connect"));
    let vertical = topology
        .edges
        .iter()
        .find(|e| matches!(e.curve, CadCurve::Line { start, end } if start[2] != end[2]))
        .unwrap();
    assert!(app
        .preview_cad_operation(OpKind::Engrave, &[reference(vertical.id, EntityKind::Edge)])
        .unwrap_err()
        .contains("one plane"));
    let mut stale = edges[0].clone();
    stale.revision += 1;
    assert!(app
        .preview_cad_operation(OpKind::Engrave, &[stale])
        .unwrap_err()
        .contains("stale"));
}

#[test]
fn external_thread_selects_a_boss_and_rejects_a_mismatched_operation_type() {
    let mut fixture = Fixture::new();
    let top = fixture.face(vec![square(0., 20., 0., 20., 0.)], [0., 0., 1.]);
    let bottom = fixture.face(vec![square(0., 20., 0., 20., -5.)], [0., 0., -1.]);
    let cylinder = fixture.bore([10., 10.], 3., top, bottom);
    for face in &mut fixture.t.faces {
        if face.id == cylinder {
            if let CadSurface::Cylinder { inward, .. } = &mut face.surface {
                *inward = false;
            }
        } else {
            // The circular rims bound the end caps of this solid cylinder.
            face.loop_ids.remove(0);
            for wire in &mut fixture.t.loops {
                if face.loop_ids.contains(&wire.id) {
                    wire.is_outer = true;
                }
            }
        }
    }
    let state = fixture.state();
    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let cylinder = reference(cylinder, EntityKind::Face);
    let resolved = app
        .preview_cad_operation(OpKind::Thread, std::slice::from_ref(&cylinder))
        .unwrap();
    assert_eq!(resolved.holes.len(), 1);
    assert_eq!(resolved.holes[0].diameter, 6.);
    let tool = app.use_tool(Tool {
        number: 30,
        diameter: 3.,
        neck_diameter: 1.5,
        flute_length: 12.,
        length: 30.,
        kind: ToolKind::ThreadMill { pitch: None },
        ..Default::default()
    });
    let id = app
        .create_cad_operation_with_tool(OpKind::Thread, vec![cylinder.clone()], tool)
        .unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Thread(op)) if !op.internal && op.major_dia == 6.)
    );
    let before = app.document().clone();
    let mut wrong_type = app.operation(id).unwrap().clone();
    if let Operation::Thread(op) = &mut wrong_type {
        op.internal = true;
    }
    assert!(app
        .apply_operation_settings(wrong_type, None)
        .unwrap_err()
        .contains("must match"));
    assert_eq!(app.document(), &before);
    let outcome = app.run(&CancelToken::new());
    assert_eq!(
        outcome.completion,
        JobCompletion::Complete,
        "{:?}",
        outcome.diagnostics
    );
}

#[test]
fn bore_cad_selection_regeneration_save_reopen_and_all_posts() {
    let mut fixture = Fixture::new();
    let top = fixture.face(vec![square(0., 20., 0., 20., 0.)], [0., 0., 1.]);
    let bottom = fixture.face(vec![square(0., 20., 0., 20., -5.)], [0., 0., -1.]);
    fixture.walls(0., 20., 0., 20., -5., 0.);
    let bores = vec![
        reference(fixture.bore([5., 10.], 3., top, bottom), EntityKind::Face),
        reference(fixture.bore([15., 10.], 3., top, bottom), EntityKind::Face),
    ];
    let state = fixture.state();
    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let tool = app.use_tool(Tool {
        number: 30,
        diameter: 3.,
        flute_length: 12.,
        length: 30.,
        kind: ToolKind::EndMill,
        ..Default::default()
    });
    let id = app
        .create_cad_operation_with_tool(OpKind::Bore, bores.clone(), tool)
        .unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Bore(op)) if op.points == vec![[5., 10.], [15., 10.]] && op.internal && op.diameter == 6. && op.z_top == 0. && op.z_bottom == -5.)
    );
    assert_eq!(
        app.geometry_state().bindings[&id].recipe,
        ExtractionRecipe::BoreCylinders { internal: true }
    );
    let mut operation = app.operation(id).unwrap().clone();
    if let Operation::Bore(op) = &mut operation {
        op.pitch = 0.8;
        op.repeat_pass = true;
        op.radial_stock = 0.1;
    }
    app.apply_operation_settings_with_geometry(
        operation,
        None,
        vec![],
        Some(vec![bores[1].clone()]),
    )
    .unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Bore(op)) if op.points == vec![[15., 10.]] && op.pitch == 0.8 && op.repeat_pass)
    );
    assert!(app.undo());
    assert!(matches!(app.operation(id), Some(Operation::Bore(op)) if op.points.len() == 2));
    assert!(app.redo());
    let path = std::env::temp_dir().join(format!("bore-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = controller();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(reopened.operation(id), app.operation(id));
    for post in [
        cam_post::PostKind::Grbl,
        cam_post::PostKind::Fanuc,
        cam_post::PostKind::Okuma,
        cam_post::PostKind::Estlcam,
    ] {
        reopened.set_post_kind(post);
        let result = reopened.run(&CancelToken::new());
        assert_eq!(
            result.completion,
            JobCompletion::Complete,
            "{:?}",
            result.diagnostics
        );
        let nc = reopened.export_nc().unwrap();
        assert!(nc.contains("Bore"), "{post:?}: {nc}");
        assert!(
            nc.lines()
                .any(|line| line.contains('Z') && line.contains('I')),
            "missing helical interpolation: {nc}"
        );
    }
    let mut invalid = reopened.operation(id).unwrap().clone();
    if let Operation::Bore(op) = &mut invalid {
        op.taper_angle = 2.;
    }
    assert!(reopened
        .apply_operation_settings_with_geometry(invalid, None, vec![], None)
        .is_err());
}

#[test]
fn pocket_boundary_chains_cut_from_stock_to_selection_and_survive_reopen() {
    for edges in [false, true] {
        let mut app = controller();
        let floor = install_pocket(&mut app);
        let wire = app.geometry_state().assets[0]
            .topology
            .loops
            .iter()
            .find(|wire| wire.face_id == floor.id && wire.is_outer)
            .unwrap()
            .clone();
        let selected = if edges {
            wire.edge_uses
                .iter()
                .rev()
                .map(|edge| reference(edge.edge_id, EntityKind::Edge))
                .collect()
        } else {
            vec![reference(wire.id, EntityKind::Loop)]
        };
        app.place_cad_instance(
            1,
            RigidTransform {
                translation: [5., 7., 10.],
                ..Default::default()
            },
        )
        .unwrap();
        let id = app
            .create_cad_operation(OpKind::Pocket, selected.clone())
            .unwrap();
        assert_eq!(
            app.geometry_state().bindings[&id].recipe,
            ExtractionRecipe::PocketBoundary
        );
        assert_eq!(app.geometry_state().bindings[&id].heights.top, 7.);
        let defaults = app.document().setup().height_settings[&id]
            .resolve(app.document(), app.operation(id).unwrap())
            .unwrap();
        assert_eq!((defaults.top, defaults.bottom), (11., 7.));
        assert!(matches!(app.operation(id), Some(Operation::Pocket(op)) if op.islands.is_empty()));
        app.edit_operation(id, |op| {
            if let Operation::Pocket(p) = op {
                p.depth = 2.;
            }
        });
        app.edit_setup(|setup| {
            setup
                .height_settings
                .get_mut(&id)
                .unwrap()
                .definition_mut(HeightPlaneKind::Bottom)
                .offset = -2.;
        });
        let heights = app.document().setup().height_settings[&id]
            .resolve(app.document(), app.operation(id).unwrap())
            .unwrap();
        assert_eq!(heights.top, 11.);
        assert_eq!(heights.bottom, 5.);
        let outcome = app.run(&CancelToken::new());
        assert_eq!(
            outcome.completion,
            JobCompletion::Complete,
            "{:?}",
            outcome.diagnostics
        );
        assert!(!outcome.has_errors(), "{:?}", outcome.diagnostics);
        assert!(!outcome.program.is_empty());
        let program = outcome.program.clone();
        let path =
            std::env::temp_dir().join(format!("pocket-chain-{edges}-{}.ocam", std::process::id()));
        app.save_project(&path).unwrap();
        let mut reopened = controller();
        reopened.open_project(&path).unwrap();
        std::fs::remove_file(path).unwrap();
        assert_eq!(reopened.geometry_state().bindings[&id].selected, selected);
        assert_eq!(reopened.run(&CancelToken::new()).program, program);
    }
}

#[test]
fn pocket_boundary_rejects_open_disconnected_and_nonplanar_edge_chains() {
    let mut app = controller();
    let floor = install_pocket(&mut app);
    let topology = &app.geometry_state().assets[0].topology;
    let wire = topology
        .loops
        .iter()
        .find(|wire| wire.face_id == floor.id && wire.is_outer)
        .unwrap();
    let edges: Vec<_> = wire
        .edge_uses
        .iter()
        .map(|edge| reference(edge.edge_id, EntityKind::Edge))
        .collect();
    let nonplanar = topology
        .edges
        .iter()
        .find(|edge| match edge.curve {
            CadCurve::Line { start, end } => (start[2] - end[2]).abs() > 1.,
            _ => false,
        })
        .unwrap()
        .id;
    let before = app.document().clone();
    for selected in [
        vec![edges[0].clone()],
        vec![edges[0].clone(), edges[2].clone()],
        vec![reference(nonplanar, EntityKind::Edge)],
    ] {
        assert!(app.create_cad_operation(OpKind::Pocket, selected).is_err());
        assert_eq!(app.document(), &before);
    }
}

#[test]
fn parallel_creation_uses_setup_cad_and_survives_reopening() {
    let mut app = controller();
    let selected = vec![install_pocket(&mut app)];
    let tool = app.use_tool(cam_model::Tool {
        number: 901,
        diameter: 4.,
        length: 40.,
        kind: ToolKind::BallMill,
        ..Default::default()
    });
    let id = app
        .create_cad_operation_with_tool(OpKind::Parallel, selected, tool)
        .unwrap();
    app.edit_operation(id, |op| {
        let Operation::Parallel(p) = op else { panic!() };
        p.params.angle = 37.;
        p.params.stepover = 0.7;
        p.params.retraction = cam_model::ScallopRetraction::Minimum;
        p.params.safe_distance = 0.8;
        p.params.both_ways = false;
    });
    assert_eq!(
        app.geometry_state().bindings[&id].recipe,
        cam_model::ExtractionRecipe::ParallelFinishing
    );
    let path = std::env::temp_dir().join(format!("parallel-roundtrip-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = AppController::new(default_machine());
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        matches!(reopened.operation(id),Some(Operation::Parallel(p))if p.params.angle==37. && p.params.stepover==0.7 && p.params.retraction == cam_model::ScallopRetraction::Minimum && p.params.safe_distance == 0.8 && !p.params.both_ways)
    );
    assert_eq!(app.document(), reopened.document());
}

#[test]
fn spiral_cad_operation_generates_verifies_posts_and_reopens() {
    let mut fixture = Fixture::new();
    let top = fixture.face(vec![square(0., 20., 0., 20., 0.)], [0., 0., 1.]);
    fixture.face(vec![square(0., 20., 0., 20., -5.)], [0., 0., -1.]);
    fixture.walls(0., 20., 0., 20., -5., 0.);
    let mut mesh = CadMesh::default();
    for wire in &fixture.t.loops {
        let base = mesh.vertices.len() as u32;
        for use_ in &wire.edge_uses {
            let edge = fixture
                .t
                .edges
                .iter()
                .find(|e| e.id == use_.edge_id)
                .unwrap();
            let id = edge.vertices[usize::from(use_.reversed)];
            mesh.vertices.push(
                fixture
                    .t
                    .vertices
                    .iter()
                    .find(|v| v.id == id)
                    .unwrap()
                    .point,
            );
        }
        mesh.triangles
            .extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
        mesh.triangle_faces.extend([wire.face_id; 2]);
    }
    fixture.t.mesh = mesh.clone();
    fixture.t.projection = Some(CadProjectionMesh {
        mesh,
        chord_tolerance: 0.0025,
    });
    let state = fixture.state();
    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let tool = app.use_tool(Tool {
        number: 5,
        kind: ToolKind::BallMill,
        diameter: 3.,
        flute_length: 12.,
        length: 24.,
        ..Default::default()
    });
    let id = app
        .create_cad_operation_with_tool(
            OpKind::Spiral,
            vec![reference(top, EntityKind::Face)],
            tool,
        )
        .unwrap();
    app.edit_operation(id, |op| {
        let Operation::Spiral(spiral) = op else {
            panic!("Spiral operation")
        };
        spiral.params.center = Some([9., 11.]);
        spiral.params.inner_limit = 1.;
        spiral.params.outer_limit = 6.;
        spiral.params.mode = SpiralMode::SpiralWithCircles;
        spiral.params.clockwise = true;
        spiral.params.smoothing = true;
        OperationParameter::SpiralStepover.set_source(op, "=1.2");
    });
    assert_eq!(
        app.geometry_state().bindings[&id].recipe,
        ExtractionRecipe::SpiralFinishing
    );
    let path = std::env::temp_dir().join(format!("spiral-roundtrip-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = AppController::new(default_machine());
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(app.document(), reopened.document());
    let result = reopened.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    assert!(result.collisions.is_empty(), "{:?}", result.collisions);
    assert!(result.program.validate().is_ok());
    let nc = reopened.export_nc().unwrap();
    assert!(nc.lines().any(|line| line.starts_with("G1 ")), "{nc}");
}

/// Built entirely from analytic topology: no imported CAD or saved project fixture.
fn two_pockets() -> (GeometryState, Vec<EntityRef>) {
    let mut f = Fixture::new();
    f.t.bodies[0].bounds[1][0] = 40.;
    let mut floors = Vec::new();
    for x in [0., 20.] {
        floors.push(reference(
            f.face(
                vec![
                    square(x + 2., x + 18., 2., 18., -3.),
                    square(x + 8., x + 12., 8., 12., -3.),
                ],
                [0., 0., 1.],
            ),
            EntityKind::Face,
        ));
        f.walls(x + 2., x + 18., 2., 18., -3., 0.);
        f.walls(x + 8., x + 12., 8., 12., -3., 0.);
        f.face(vec![square(x + 8., x + 12., 8., 12., 0.)], [0., 0., 1.]);
    }
    f.face(
        vec![
            square(0., 40., 0., 20., 0.),
            square(2., 18., 2., 18., 0.),
            square(22., 38., 2., 18., 0.),
        ],
        [0., 0., 1.],
    );
    f.walls(0., 40., 0., 20., -5., 0.);
    f.face(vec![square(0., 40., 0., 20., -5.)], [0., 0., -1.]);
    (f.state(), floors)
}

#[test]
fn multiple_pocket_floors_keep_islands_and_output_after_save_and_reselection() {
    let (state, floors) = two_pockets();
    state.validate().unwrap();
    let binding =
        cam_geometry3d::resolve_selection(&state, &floors, ExtractionRecipe::PocketFloor, 0.)
            .unwrap();
    assert!(
        cam_geometry3d::verify_selection(&state, &floors, ExtractionRecipe::PocketFloor, 0.)
            .is_none()
    );
    let input = cam_geometry3d::resolve_binding(&state, &binding).unwrap();
    assert_eq!(input.regions.len(), 2);
    assert!(input
        .regions
        .iter()
        .all(|r| r.holes().len() == 1 && (r.area() - 240.).abs() < 1e-6));
    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    select_all_models(&mut app);
    let id = app
        .create_cad_operation(OpKind::Pocket, floors.clone())
        .unwrap();
    let Some(Operation::Pocket(op)) = app.operation(id) else {
        panic!("pocket")
    };
    assert_eq!(op.additional_regions, input.regions[1..]);
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    let before = result.program.clone();
    let cut_points: Vec<_> = cuts(&before)
        .iter()
        .filter_map(|s| match s {
            cam_cldata::Step::Linear { to, .. } | cam_cldata::Step::Arc { end: to, .. } => {
                Some(*to)
            }
            _ => None,
        })
        .collect();
    for region in &input.regions {
        assert!(cut_points
            .iter()
            .any(|p| region.contains(Point::new(p.x, p.y))));
        assert!(cut_points
            .iter()
            .all(|p| !cam_geo::Polygon::new(region.holes()[0].clone())
                .unwrap()
                .contains(Point::new(p.x, p.y))));
    }
    let path = std::env::temp_dir().join(format!("multi-pocket-{}.ocam", std::process::id()));
    app.save_project(&path).unwrap();
    let mut reopened = controller();
    reopened.open_project(&path).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(app.document(), reopened.document());
    assert_eq!(before, reopened.run(&CancelToken::new()).program);
    app.edit_cad_geometry(id, vec![floors[0].clone()]).unwrap();
    assert!(
        matches!(app.operation(id), Some(Operation::Pocket(op)) if op.additional_regions.is_empty())
    );
    assert!(app.undo());
    assert_eq!(before, app.run(&CancelToken::new()).program);
}

#[test]
fn multiple_pocket_boundary_loops_and_disconnected_edge_chains_resolve_all_regions() {
    let (state, floors) = two_pockets();
    let topology = &state.assets[0].topology;
    let loops: Vec<_> = floors
        .iter()
        .map(|r| {
            let face = topology.faces.iter().find(|f| f.id == r.id).unwrap();
            reference(face.loop_ids[0], EntityKind::Loop)
        })
        .collect();
    let mut edges: Vec<_> = loops
        .iter()
        .flat_map(|r| {
            topology
                .loops
                .iter()
                .find(|l| l.id == r.id)
                .unwrap()
                .edge_uses
                .iter()
                .map(|e| reference(e.edge_id, EntityKind::Edge))
        })
        .collect();
    edges.reverse();
    for selected in [loops, edges] {
        let binding = cam_geometry3d::resolve_selection_advisory(
            &state,
            &selected,
            ExtractionRecipe::PocketBoundary,
            0.,
        )
        .unwrap();
        let input = cam_geometry3d::resolve_binding_advisory(&state, &binding).unwrap();
        assert_eq!(input.regions.len(), 2);
        assert_eq!(
            input.heights,
            CuttingHeights {
                top: -3.,
                bottom: -4.
            }
        );
        let mut app = controller();
        app.add_cad_asset(state.assets[0].clone(), state.instances.clone())
            .unwrap();
        select_all_models(&mut app);
        let id = app.create_cad_operation(OpKind::Pocket, selected).unwrap();
        assert!(
            matches!(app.operation(id), Some(Operation::Pocket(op)) if op.additional_regions.len() == 1)
        );
        let result = app.run(&CancelToken::new());
        assert_eq!(
            result.completion,
            JobCompletion::Complete,
            "{:?}",
            result.diagnostics
        );
    }
}

#[test]
fn multiple_pockets_do_not_drop_invalid_duplicate_or_different_height_features() {
    let (mut state, floors) = two_pockets();
    let mut selected = floors.clone();
    selected[1].revision += 1;
    assert!(cam_geometry3d::resolve_selection_advisory(
        &state,
        &selected,
        ExtractionRecipe::PocketFloor,
        0.
    )
    .unwrap_err()
    .contains("stale"));
    selected[1] = selected[0].clone();
    assert!(cam_geometry3d::resolve_selection_advisory(
        &state,
        &selected,
        ExtractionRecipe::PocketFloor,
        0.
    )
    .unwrap_err()
    .contains("more than once"));
    // An unusable second face must fail the complete selection.
    let topology = std::sync::Arc::make_mut(&mut state.assets[0].topology);
    let face = topology
        .faces
        .iter()
        .find(|face| face.id == floors[1].id)
        .unwrap();
    let wire = topology
        .loops
        .iter()
        .find(|wire| wire.id == face.loop_ids[0])
        .unwrap();
    let edge_id = wire.edge_uses[0].edge_id;
    topology
        .edges
        .iter_mut()
        .find(|edge| edge.id == edge_id)
        .unwrap()
        .curve = CadCurve::Unsupported {
        description: "unusable selected boundary".into(),
    };
    assert!(cam_geometry3d::resolve_selection_advisory(
        &state,
        &floors,
        ExtractionRecipe::PocketFloor,
        0.
    )
    .is_err());
    let mut f = Fixture::new();
    let a = f.face(vec![square(1., 8., 1., 8., -2.)], [0., 0., 1.]);
    let b = f.face(vec![square(11., 18., 1., 8., -3.)], [0., 0., 1.]);
    assert!(cam_geometry3d::resolve_selection_advisory(
        &f.state(),
        &[
            reference(a, EntityKind::Face),
            reference(b, EntityKind::Face)
        ],
        ExtractionRecipe::PocketFloor,
        0.
    )
    .unwrap_err()
    .contains("different floor heights"));
}

#[test]
fn multiple_pockets_track_each_instances_placement_and_reject_incomplete_chains() {
    let (mut state, floors) = two_pockets();
    let mut instance = state.instances[0].clone();
    instance.id = 2;
    instance.placement.translation = [60., 0., 0.];
    state.instances.push(instance);
    let refs = vec![
        floors[0].clone(),
        EntityRef {
            instance_id: 2,
            ..floors[0].clone()
        },
    ];
    let mut binding = cam_geometry3d::resolve_selection_advisory(
        &state,
        &refs,
        ExtractionRecipe::PocketFloor,
        0.,
    )
    .unwrap();
    binding.overrides.bottom = Some(-2.);
    let before = cam_geometry3d::resolve_binding_advisory(&state, &binding).unwrap();
    assert_eq!(before.regions.len(), 2);
    assert_eq!(before.heights.bottom, -2.);
    assert!(before.regions[1].contains(Point::new(63., 3.)));
    state.instances[1].placement.translation[0] += 10.;
    let moved = cam_geometry3d::resolve_binding_advisory(&state, &binding).unwrap();
    assert_ne!(before.fingerprint, moved.fingerprint);
    assert!(moved.regions[1].contains(Point::new(73., 3.)));
    state.instances[1].placement.translation[2] = 1.;
    assert!(cam_geometry3d::resolve_binding_advisory(&state, &binding)
        .unwrap_err()
        .contains("different floor heights"));
    let topology = &state.assets[0].topology;
    let mut edges: Vec<_> = floors
        .iter()
        .flat_map(|r| {
            let face = topology.faces.iter().find(|f| f.id == r.id).unwrap();
            topology
                .loops
                .iter()
                .find(|l| l.id == face.loop_ids[0])
                .unwrap()
                .edge_uses
                .iter()
                .map(|e| reference(e.edge_id, EntityKind::Edge))
        })
        .collect();
    edges.pop();
    assert!(cam_geometry3d::resolve_selection_advisory(
        &state,
        &edges,
        ExtractionRecipe::PocketBoundary,
        0.
    )
    .is_err());
}

#[test]
fn multi_body_placement_is_atomic_and_undoable() {
    let (mut state, _) = pocket();
    let mut second = state.instances[0].clone();
    second.id = 2;
    second.placement.translation = [10., 0., 0.];
    state.instances.push(second);
    let mut app = controller();
    app.add_cad_asset(state.assets[0].clone(), state.instances)
        .unwrap();
    let first = RigidTransform {
        translation: [3., 4., 0.],
        ..Default::default()
    };
    let second = RigidTransform {
        translation: [13., 4., 0.],
        ..Default::default()
    };
    assert!(app
        .place_cad_instances(vec![(1, first), (99, second)])
        .is_err());
    assert_eq!(
        app.geometry_state()
            .instance(1)
            .unwrap()
            .placement
            .translation,
        [0.; 3]
    );
    app.place_cad_instances(vec![(1, first), (2, second)])
        .unwrap();
    assert_eq!(app.geometry_state().instance(2).unwrap().placement, second);
    assert!(app.undo());
    assert_eq!(
        app.geometry_state()
            .instance(1)
            .unwrap()
            .placement
            .translation,
        [0.; 3]
    );
    assert_eq!(
        app.geometry_state()
            .instance(2)
            .unwrap()
            .placement
            .translation,
        [10., 0., 0.]
    );
}
