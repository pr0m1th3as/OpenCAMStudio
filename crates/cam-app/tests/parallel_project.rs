use cam_app::{default_machine, AppController, OpKind};
use cam_cldata::{MoveKind, Step};
use cam_model::{EntityKind, EntityRef, Operation, ToolKind};
use cam_toolpath::{CancelToken, JobCompletion};

#[test]
#[ignore = "uses the supplied local tray-parallel.ocam project"]
fn tray_parallel_stay_down_verifies_against_remaining_stock() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut app = AppController::new(default_machine());
    app.open_project(root.join("testfiles/tray-parallel.ocam"))
        .unwrap();
    let run = app.run(&CancelToken::new());
    assert_eq!(
        run.completion,
        JobCompletion::Complete,
        "{:?}",
        run.diagnostics
    );
    let retracts: Vec<_> = run
        .program
        .steps()
        .iter()
        .filter_map(|s| match s {
            Step::Rapid { to, tag } if tag.op_id == 0 && tag.kind == MoveKind::Retract => Some(*to),
            _ => None,
        })
        .collect();
    eprintln!(
        "Parallel clearance retracts: {} {:?}",
        retracts.len(),
        retracts
    );
    let ys: Vec<_> = run
        .program
        .steps()
        .iter()
        .filter_map(|s| match s {
            Step::Linear { to, tag, .. } if tag.op_id == 0 && tag.kind == MoveKind::Cutting => {
                Some(to.y)
            }
            _ => None,
        })
        .collect();
    let min_y = ys.iter().copied().fold(f64::INFINITY, f64::min);
    let max_y = ys.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let margin = (max_y - min_y) * 0.1;
    assert!(
        !retracts
            .iter()
            .any(|p| p.y > min_y + margin && p.y < max_y - margin),
        "continuous interior finishing rows must stay down, not only pass verification"
    );
    for collision in run.collisions.iter().take(3) {
        if let Some(index) = collision.step_index {
            eprintln!(
                "{collision:?}; nearby steps: {:?}",
                &run.program.steps()
                    [index.saturating_sub(2)..(index + 3).min(run.program.steps().len())]
            );
        }
    }
    assert!(
        run.collisions.is_empty(),
        "{} collisions: {:?}",
        run.collisions.len(),
        run.collisions.first()
    );
    // This fixture exceeds its configured machine's Y travel. Its independent
    // travel check must remain active, while stock collisions no longer block export.
    assert!(!matches!(
        app.export_nc(),
        Err(cam_app::ExportError::NonCuttingThroughStock(_)
            | cam_app::ExportError::RapidThroughStock(_))
    ));
}

/// The supplied local CAD fixture is deliberately optional in ordinary CI.
/// Run with: cargo test -p cam-app --test parallel_project -- --ignored --nocapture
#[test]
#[ignore = "uses the supplied local testfiles/tray.ocam CAD project"]
fn tray_parallel_finishing() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut app = AppController::new(default_machine());
    app.open_project(root.join("testfiles/tray.ocam")).unwrap();
    app.edit_tool(0, |t| {
        t.kind = ToolKind::BallMill;
        t.description = "6 mm ball nose - Parallel finishing".into();
    });
    let state = app.geometry_state();
    let instance = &state.instances[0];
    let asset = state.asset(instance.asset_id).unwrap();
    let reference = EntityRef {
        asset_id: asset.id,
        revision: asset.revision,
        instance_id: instance.id,
        body_id: instance.body_id,
        kind: EntityKind::Body,
        id: instance.body_id,
    };
    let id = app
        .create_cad_operation_with_tool(OpKind::Parallel, vec![reference], 1)
        .unwrap();
    app.edit_operation(id, |op| {
        let Operation::Parallel(p) = op else { panic!() };
        p.params.stepover = 1.;
        p.params.tolerance = 0.02;
        p.params.angle = 0.;
    });
    let (_, hi) = app.stock_box();
    app.edit_heights(|h| {
        h.top_of_stock = hi[2];
        h.retract = hi[2] + 3.;
        h.clearance = hi[2] + 8.;
    });
    let started = std::time::Instant::now();
    let run = app.run(&CancelToken::new());
    assert_eq!(
        run.completion,
        JobCompletion::Complete,
        "{:?}",
        run.diagnostics
    );
    assert!(run.program.validate().is_ok());
    assert!(run.collisions.is_empty(), "{:?}", run.collisions);
    let cuts: Vec<_> = run
        .program
        .steps()
        .iter()
        .filter_map(|s| match s {
            Step::Linear { to, tag, .. } if tag.kind == MoveKind::Cutting => Some(*to),
            _ => None,
        })
        .collect();
    assert!(cuts.len() > 100);
    let min = cuts.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    let max = cuts.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
    assert!(max - min > 5., "surface following Z range: {min}..{max}");
    eprintln!(
        "tray: {} cutting moves, Z {min:.3}..{max:.3}, {:?}, diagnostics {:?}",
        cuts.len(),
        started.elapsed(),
        run.diagnostics
    );
    // A separate review project preserves the user's source fixture.
    app.save_project(root.join("testfiles/tray-parallel.ocam"))
        .unwrap();
    let mut reopened = AppController::new(default_machine());
    reopened
        .open_project(root.join("testfiles/tray-parallel.ocam"))
        .unwrap();
    assert!(matches!(reopened.operation(id),Some(Operation::Parallel(p))if p.params.stepover==1.));
}
