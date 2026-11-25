//! Fixture routing regressions built entirely from analytic bodies and contours.
//! No external project, CAD kernel, or local testfile is needed.
use cam_app::{default_machine, AppController, OpKind};
use cam_cldata::{MoveKind, Point3, Program, Step};
use cam_geo::{Contour, Point};
use cam_model::*;
use cam_toolpath::{CancelToken, JobCompletion};

fn clamped_profile() -> AppController {
    let mut app = AppController::new(default_machine());
    app.edit_tool(0, |t| {
        t.diameter = 6.;
        t.length = 42.;
        t.flute_length = 12.;
    });
    app.add_cad_asset(
        GeometryAsset {
            id: 1,
            revision: 1,
            name: "Analytic workpiece and clamps".into(),
            source_format: CadFormat::Step,
            source_path: None,
            source_data: Vec::new().into(),
            source_units: SourceUnits::Millimetres,
            backend: "analytic test".into(),
            import_options: None,
            import_diagnostics: vec![],
            source_fingerprint: String::new(),
            topology: TopologySnapshot {
                bodies: vec![
                    CadBody {
                        id: 1,
                        name: "100 x 60 block".into(),
                        valid_solid: true,
                        bounds: [[-50., -30., 0.], [50., 30., 20.]],
                    },
                    CadBody {
                        id: 2,
                        name: "Clamp bar".into(),
                        valid_solid: true,
                        bounds: [[-50., -10., 0.], [50., 10., 12.]],
                    },
                ],
                ..Default::default()
            }
            .into(),
        },
        vec![
            GeometryInstance {
                id: 1,
                asset_id: 1,
                body_id: 1,
                name: "Workpiece".into(),
                placement: Default::default(),
                visible: true,
            },
            GeometryInstance {
                id: 2,
                asset_id: 1,
                body_id: 2,
                name: "Right clamp".into(),
                placement: RigidTransform {
                    translation: [97., 5., 18.],
                    ..Default::default()
                },
                visible: true,
            },
            GeometryInstance {
                id: 3,
                asset_id: 1,
                body_id: 2,
                name: "Left clamp".into(),
                placement: RigidTransform {
                    rotation: [[-1., 0., 0.], [0., -1., 0.], [0., 0., 1.]],
                    translation: [-98.5, -11., 17.],
                },
                visible: true,
            },
        ],
    )
    .unwrap();
    app.edit_setup(|s| {
        s.model_instances = vec![1];
        s.fixture_instances = vec![2, 3];
        s.fixture_clearance = FixtureClearance {
            radial: 5.,
            axial: 5.,
        };
        s.stock = Stock::RelativeBox {
            offsets: [[1., 1., 4.], [1., 1., 1.]],
        };
    });
    let Operation::Profile(mut p) = app.operation_template(OpKind::Profile, 1) else {
        unreachable!()
    };
    p.chain = Contour::new(vec![
        Point::new(-50., 30.),
        Point::new(50., 30.),
        Point::new(50., -30.),
        Point::new(-50., -30.),
    ]);
    p.side = Side::Outside;
    p.comp = Comp::Computed;
    p.depth = 21.;
    p.stepdown = 2.;
    p.stepover = 0.;
    p.start = None;
    p.end = None;
    p.lead_in = Lead::ArcSweep {
        radius: 0.6,
        angle_deg: 90.,
    };
    p.lead_out = p.lead_in;
    p.lead_overlap = 0.6;
    p.plunge = Plunge::Straight;
    p.feed = 300.;
    p.plunge_feed = 100.;
    p.tabs.enabled = false;
    app.add_operation(Operation::Profile(p));
    let id = app.inspection_setup().operations[0].id();
    app.edit_setup(|s| {
        s.height_settings.insert(
            id,
            OperationHeightSettings {
                planes: [36., 26., 26., 21., 0.]
                    .map(|z| HeightDefinition::new(HeightReference::Origin, z)),
                resolved: None,
            },
        );
    });
    app
}
fn cutting_length(program: &Program) -> f64 {
    let mut previous = None::<Point3>;
    let mut length = 0.;
    for s in program.steps() {
        let end = match s {
            Step::Rapid { to, .. } | Step::Linear { to, .. } => Some(*to),
            Step::Arc { end, .. } => Some(*end),
            _ => None,
        };
        if s.tag().is_some_and(|t| t.kind == MoveKind::Cutting) {
            if let (Some(a), Some(b)) = (previous, end) {
                length += match s {
                    Step::Arc { center, dir, .. } => {
                        let r = (a.x - center.x).hypot(a.y - center.y);
                        (r * cam_cldata::arc_sweep(a, b, *center, *dir)).hypot(b.z - a.z)
                    }
                    _ => (b.x - a.x).hypot(b.y - a.y).hypot(b.z - a.z),
                };
            }
        }
        if let Some(end) = end {
            previous = Some(end);
        }
    }
    length
}
fn add_box(app: &mut AppController, name: &str, bounds: [[f64; 3]; 2]) -> u64 {
    app.add_cad_asset(
        GeometryAsset {
            id: 0,
            revision: 1,
            name: name.into(),
            source_format: CadFormat::Step,
            source_path: None,
            source_data: Vec::new().into(),
            source_units: SourceUnits::Millimetres,
            backend: "analytic test".into(),
            import_options: None,
            import_diagnostics: vec![],
            source_fingerprint: String::new(),
            topology: TopologySnapshot {
                bodies: vec![CadBody {
                    id: 1,
                    name: name.into(),
                    valid_solid: true,
                    bounds,
                }],
                ..Default::default()
            }
            .into(),
        },
        vec![GeometryInstance {
            id: 0,
            asset_id: 0,
            body_id: 1,
            name: name.into(),
            placement: Default::default(),
            visible: true,
        }],
    )
    .unwrap();
    app.geometry_state().instances.last().unwrap().id
}

#[test]
fn two_clamps_split_an_outside_profile_into_verified_retracts_and_reentries() {
    let mut app = clamped_profile();
    let result = app.run(&CancelToken::new());
    assert_eq!(
        result.completion,
        JobCompletion::Complete,
        "{:?}",
        result.diagnostics
    );
    assert!(result.collisions.is_empty(), "{:?}", result.collisions);
    assert!(result.program.validate().is_ok());
    assert!(result
        .diagnostics
        .iter()
        .any(|d| d.message.contains("Fixture clearance excluded")));
    let plunges = result
        .program
        .steps()
        .iter()
        .filter(|s| matches!(s,Step::Linear { tag,.. } if tag.kind==MoveKind::Plunge))
        .count();
    assert!(
        plunges >= 33,
        "Every level needs entries after both clamps: {plunges}"
    );
    let geometry = app
        .geometry_state()
        .geometry_in_setup(&app.inspection_setup().frame)
        .unwrap();
    let fixtures = app.inspection_setup().fixture_envelopes(&geometry).unwrap();
    assert!(cam_toolpath::verify_fixture_clearance(
        &app.outcome().unwrap().program,
        &app.inspection_setup().tools,
        &fixtures,
        &CancelToken::new()
    )
    .unwrap()
    .is_empty());
    assert!(app.export_nc().is_ok());
}
#[test]
fn fixture_clearance_edits_recalculate_the_profile_and_removal_restores_the_original_path() {
    let mut app = clamped_profile();
    app.edit_setup(|s| s.fixture_instances.clear());
    let unblocked = app.run(&CancelToken::new()).program.clone();
    app.edit_setup(|s| {
        s.fixture_instances = vec![2, 3];
        s.fixture_clearance.radial = 0.;
    });
    let tight = app.run(&CancelToken::new());
    assert_eq!(
        tight.completion,
        JobCompletion::Complete,
        "{:?}",
        tight.diagnostics
    );
    assert!(tight.collisions.is_empty());
    let tight_length = cutting_length(&tight.program);
    app.edit_setup(|s| s.fixture_clearance.radial = 5.);
    let wide = app.run(&CancelToken::new());
    assert_eq!(
        wide.completion,
        JobCompletion::Complete,
        "{:?}",
        wide.diagnostics
    );
    assert!(wide.collisions.is_empty());
    assert!(cutting_length(&wide.program) < tight_length);
    assert!(tight_length < cutting_length(&unblocked));
    app.edit_setup(|s| s.fixture_instances.clear());
    assert_eq!(app.run(&CancelToken::new()).program, unblocked);
}
#[test]
fn a_fixture_covering_every_cut_fails_instead_of_posting_an_empty_profile() {
    let mut app = clamped_profile();
    let plate = add_box(
        &mut app,
        "Cover plate",
        [[-100., -100., 0.], [100., 100., 40.]],
    );
    app.edit_setup(|s| s.fixture_instances = vec![plate]);
    let result = app.run(&CancelToken::new());
    assert_eq!(result.completion, JobCompletion::Failed);
    assert!(
        result
            .diagnostics
            .iter()
            .any(|d| d.message.contains("block all cutting passes")),
        "{:?}",
        result.diagnostics
    );
    assert!(app.export_nc().is_err());
}

#[test]
fn an_escape_through_an_overhead_fixture_is_rejected_by_final_verification() {
    let mut app = AppController::new(default_machine());
    app.edit_tool(0, |t| {
        t.diameter = 2.;
        t.length = 2.;
        t.flute_length = 2.;
    });
    let clamp = add_box(&mut app, "Side clamp", [[-0.1, -2., -2.], [0.1, 2., 0.]]);
    let overhead = add_box(
        &mut app,
        "Overhead fixture",
        [[-2.2, -2., 5.], [-1.8, 2., 6.]],
    );
    app.edit_setup(|s| {
        s.fixture_instances = vec![clamp, overhead];
        s.fixture_clearance = FixtureClearance {
            radial: 0.,
            axial: 0.,
        };
        s.stock = Stock::FixedBox {
            size: [40., 10., 5.],
            model_position: [0.5, 0.5, 1.],
        };
        s.heights = Heights::new(10., 8., 0.);
    });
    let Operation::Profile(mut p) = app.operation_template(OpKind::Profile, 1) else {
        unreachable!()
    };
    p.closed = false;
    p.chain = Contour::new(vec![Point::new(-10., 0.), Point::new(10., 0.)]);
    p.side = Side::On;
    p.comp = Comp::Computed;
    p.depth = 1.;
    p.stepdown = 1.;
    p.stepover = 0.;
    p.start = None;
    p.end = None;
    p.lead_in = Lead::None;
    p.lead_out = Lead::None;
    p.lead_overlap = 0.;
    p.plunge = Plunge::Straight;
    app.add_operation(Operation::Profile(p));
    let result = app.run(&CancelToken::new());
    assert_eq!(result.completion, JobCompletion::Failed);
    assert!(
        result
            .diagnostics
            .iter()
            .any(|d| d.message.contains("Overhead fixture")),
        "{:?}",
        result.diagnostics
    );
    assert!(app.export_nc().is_err());
}
