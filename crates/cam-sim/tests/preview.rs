use cam_cldata::{ArcDir, MoveKind, Point3, Program, ProgramBuilder, Step};
use cam_sim::{
    simulate, Heightfield, ProfileShape, SimOptions, SimTool, SimulationCache, StockMeshCache,
    ToolProfile,
};
use std::cell::Cell;

#[test]
fn inherited_stock_is_used_for_collisions_rewind_and_changed_source() {
    let min = [0., 0., -5.];
    let max = [10., 10., 0.];
    let options = SimOptions {
        resolution: 0.2,
        tool_radius: 0.5,
    };
    let mut stock = Heightfield::new([0., 0.], [10., 10.], options.resolution, 0.);
    stock.lower_rect([1., 1.], [9., 9.], -3.);
    let program = ProgramBuilder::new()
        .op(2)
        .feed(100.)
        .rapid(Point3::new(3., 5., 2.), MoveKind::Link)
        .rapid(Point3::new(3., 5., -2.), MoveKind::Link)
        .linear(Point3::new(7., 5., -4.), MoveKind::Cutting)
        .build();
    let mut cache = SimulationCache::default();
    let simulate = |cache: &mut SimulationCache, program: &Program, key: &str| {
        cache
            .simulate_from_stock_controlled(
                program,
                min,
                max,
                &options,
                &[],
                &stock,
                key,
                &|| false,
                |_, _| {},
            )
            .unwrap()
    };
    let full = simulate(&mut cache, &program, "first source");
    assert!(full.collisions.is_empty());
    assert!(full.field.sample(7., 5.) < -3.);
    let prefix = Program {
        steps: program.steps[..2].to_vec(),
    };
    let rewind = simulate(&mut cache, &prefix, "first source");
    assert_eq!(rewind.field.sample(7., 5.), -3.);
    assert_eq!(
        simulate(&mut cache, &program, "first source")
            .field
            .to_solid_mesh(-5.),
        full.field.to_solid_mesh(-5.)
    );

    let virgin = Heightfield::new([0., 0.], [10., 10.], options.resolution, 0.);
    let changed = cache
        .simulate_from_stock_controlled(
            &program,
            min,
            max,
            &options,
            &[],
            &virgin,
            "edited source",
            &|| false,
            |_, _| {},
        )
        .unwrap();
    assert!(
        !changed.collisions.is_empty(),
        "a changed upstream blank must invalidate checkpoints"
    );
}

#[test]
fn circular_wall_vertices_follow_the_cutter_rather_than_the_grid() {
    for resolution in [0.5, 0.25] {
        let mut field = Heightfield::new([0., 0.], [20., 20.], resolution, 0.);
        field.cut_segment([10., 10., -6.], [10., 10., -6.], 4.);
        let mesh = field.to_solid_mesh(-5.);
        let mut worst = 0f32;
        let mut count = 0;
        for (p, n) in mesh.positions.iter().zip(&mesh.normals) {
            if n[2].abs() < 0.01 && p[0] > 1. && p[0] < 19. && p[1] > 1. && p[1] < 19. {
                worst = worst.max(((p[0] - 10.).hypot(p[1] - 10.) - 4.).abs());
                count += 1;
            }
        }
        assert!(count > 30);
        assert!(
            worst < resolution as f32 * 0.06,
            "{resolution}: contour error {worst}"
        );
    }
}

#[test]
fn shallow_helix_playback_recovers_the_cutter_wall_and_keeps_the_ramp() {
    let resolution = 0.5;
    let options = SimOptions {
        resolution,
        tool_radius: 1.9,
    };
    let center = Point3::new(16., 16., 0.);
    let start = Point3::new(19., 16., 0.08);
    let min = [0., 0., -5.];
    let max = [32., 32., 0.];
    let mut cache = SimulationCache::default();
    let mut mesh_cache = StockMeshCache::default();
    for angle in [60_f64, 135., 85., 170.] {
        let (sin, cos) = angle.to_radians().sin_cos();
        let tip = Point3::new(
            center.x + 3. * cos,
            center.y + 3. * sin,
            start.z - angle / 200.,
        );
        let program = ProgramBuilder::new()
            .op(7)
            .feed(100.)
            .rapid(start, MoveKind::Link)
            .arc(tip, center, ArcDir::Ccw, MoveKind::Plunge)
            .build();
        let sim = cache
            .simulate_controlled(&program, min, max, &options, &[], &|| false, |_, _| {})
            .unwrap();
        let mesh = mesh_cache
            .mesh_controlled(&sim.field, min[2], &|| false)
            .unwrap();
        let fresh = simulate(&program, min, max, &options, &[]);
        assert_eq!(
            mesh,
            StockMeshCache::default()
                .mesh_controlled(&fresh.field, min[2], &|| false)
                .unwrap()
        );

        // The forward cap of the sweep is a cylinder centered at the current
        // tip, even when the cut is shallower than the usual wall threshold.
        let mut cap = 0;
        let mut ramp = 0;
        for (i, (p, n)) in mesh.positions.iter().zip(&mesh.normals).enumerate() {
            if !mesh.machined[i] {
                continue;
            }
            assert_eq!(mesh.operations[i], Some(7));
            let dx = f64::from(p[0]) - tip.x;
            let dy = f64::from(p[1]) - tip.y;
            if n[2].abs() < 0.01 && -sin * dx + cos * dy > options.tool_radius * 0.75 {
                assert!((dx.hypot(dy) - options.tool_radius).abs() < resolution * 0.15);
                cap += 1;
            }
            if n[2] > 0.95 && n[2] < 0.99999 && p[2] < -0.05 && f64::from(p[2]) > tip.z {
                ramp += 1;
            }
        }
        assert!(cap > 4, "{angle}°: only {cap} vertices on the cutter wall");
        assert!(
            ramp > 4,
            "{angle}°: only {ramp} vertices on the smooth ramp"
        );
    }
}

#[test]
fn diagonal_cut_keeps_parallel_straight_walls_and_exact_width() {
    let mut field = Heightfield::new([0., 0.], [20., 20.], 0.5, 0.);
    field.cut_segment([2., 2., -3.], [18., 18., -3.], 1.73);
    let mesh = field.to_solid_mesh(-5.);
    let mut count = 0;
    for (p, n) in mesh.positions.iter().zip(&mesh.normals) {
        if n[2].abs() < 0.01 && p[0] > 5. && p[0] < 15. && p[1] > 5. && p[1] < 15. {
            let distance = (p[1] - p[0]).abs() / 2f32.sqrt();
            assert!((distance - 1.73).abs() < 0.002, "{p:?}: {distance}");
            count += 1;
        }
    }
    assert!(count > 20);
}

#[test]
fn flat_stock_mesh_size_scales_with_boundary_not_surface_area() {
    let mesh = Heightfield::new([0., 0.], [100., 100.], 0.1, 0.).to_solid_mesh(-10.);
    assert!(mesh.indices.len() / 3 < 10_000);
    assert!(mesh.positions.len() < 20_000);
}

#[test]
fn fractional_stock_extent_is_preserved() {
    let field = Heightfield::new([-0.3, 0.2], [3.13, 2.72], 0.5, 1.);
    let mesh = field.to_solid_mesh(-2.);
    for axis in 0..2 {
        let hi = mesh
            .positions
            .iter()
            .map(|p| p[axis])
            .fold(f32::NEG_INFINITY, f32::max);
        assert!((hi - [3.13, 2.72][axis]).abs() < 1e-5);
    }
}

fn source() -> Program {
    let mut builder = ProgramBuilder::new()
        .tool_change(1)
        .feed(100.)
        .rapid(Point3::new(2., 2., 2.), MoveKind::Link);
    for i in 0..800 {
        let t = f64::from(i);
        builder = builder.linear(
            Point3::new(
                10. + 6. * (t * 0.03).cos(),
                10. + 6. * (t * 0.03).sin(),
                -1. - t / 800.,
            ),
            MoveKind::Cutting,
        );
        if i == 390 {
            builder = builder.tool_change(2);
        }
    }
    builder.build()
}
fn options() -> SimOptions {
    SimOptions {
        resolution: 0.25,
        tool_radius: 1.,
    }
}
fn tools() -> [SimTool; 2] {
    [
        SimTool {
            noncutting_tip_radius: None,
            number: 1,
            profile: ToolProfile::flat(1.),
        },
        SimTool {
            noncutting_tip_radius: None,
            number: 2,
            profile: ToolProfile {
                radius: 1.5,
                shape: ProfileShape::Ball,
            },
        },
    ]
}
fn check(cache: &mut SimulationCache, program: &Program, options: &SimOptions, tools: &[SimTool]) {
    let actual = cache
        .simulate_controlled(
            program,
            [0., 0., -5.],
            [20., 20., 0.],
            options,
            tools,
            &|| false,
            |_, _| {},
        )
        .unwrap();
    let expected = simulate(program, [0., 0., -5.], [20., 20., 0.], options, tools);
    assert_eq!(actual.collisions, expected.collisions);
    assert_eq!(actual.removed_volume, expected.removed_volume);
    assert_eq!(
        actual.field.to_solid_mesh(-5.),
        expected.field.to_solid_mesh(-5.)
    );
}

#[test]
fn incremental_playback_rewind_and_edited_prefix_match_fresh_simulation() {
    let source = source();
    let mut cache = SimulationCache::default();
    for count in [0, 30, 32, 300, 650, 750, 500, 100, 803] {
        check(
            &mut cache,
            &Program {
                steps: source.steps[..count.min(source.steps.len())].to_vec(),
            },
            &options(),
            &tools(),
        );
    }
    let mut edited = source.clone();
    if let Step::Linear { to, .. } = &mut edited.steps[350] {
        to.z -= 1.;
    }
    check(&mut cache, &edited, &options(), &tools());
    check(
        &mut cache,
        &source,
        &SimOptions {
            resolution: 0.4,
            ..options()
        },
        &tools(),
    );
    check(
        &mut cache,
        &source,
        &options(),
        &[SimTool {
            noncutting_tip_radius: None,
            number: 1,
            profile: ToolProfile::flat(2.),
        }],
    );
}

#[test]
fn interpolated_last_move_is_not_committed_to_the_next_frame() {
    let mut cache = SimulationCache::default();
    let a = ProgramBuilder::new()
        .feed(100.)
        .rapid(Point3::new(2., 10., 2.), MoveKind::Link)
        .linear(Point3::new(2., 10., -2.), MoveKind::Plunge)
        .build();
    let mut prefix = a.clone();
    for x in [4., 8., 15., 6., 18.] {
        prefix.steps.truncate(a.steps.len());
        prefix.steps.push(Step::Linear {
            to: Point3::new(x, 10., -2.),
            feed: 100.,
            tag: cam_cldata::Tag::new(0, MoveKind::Cutting),
        });
        check(&mut cache, &prefix, &options(), &[]);
    }
}

#[test]
fn cancellation_cannot_poison_reusable_checkpoints() {
    let source = source();
    let mut cache = SimulationCache::default();
    let calls = Cell::new(0);
    let result = cache.simulate_controlled(
        &source,
        [0., 0., -5.],
        [20., 20., 0.],
        &options(),
        &tools(),
        &|| {
            calls.set(calls.get() + 1);
            calls.get() > 6000
        },
        |_, _| {},
    );
    assert!(result.is_none());
    check(&mut cache, &source, &options(), &tools());
    check(
        &mut cache,
        &Program {
            steps: source.steps[..400].to_vec(),
        },
        &options(),
        &tools(),
    );
}

#[test]
fn arc_tessellation_refines_with_the_simulation_grid() {
    let program = ProgramBuilder::new()
        .feed(100.)
        .rapid(Point3::new(15., 10., 2.), MoveKind::Link)
        .linear(Point3::new(15., 10., -2.), MoveKind::Plunge)
        .arc(
            Point3::new(15., 10., -2.),
            Point3::new(10., 10., -2.),
            ArcDir::Ccw,
            MoveKind::Cutting,
        )
        .build();
    let sim = simulate(
        &program,
        [0., 0., -5.],
        [20., 20., 0.],
        &SimOptions {
            resolution: 0.05,
            tool_radius: 0.2,
        },
        &[],
    );
    for i in 0..360 {
        let a = f64::from(i).to_radians();
        assert!(
            sim.field.sample(10. + 5. * a.cos(), 10. + 5. * a.sin()) < -1.99,
            "arc gap at {i}°"
        );
    }
}

#[test]
fn scanline_rasterisation_matches_a_brute_force_swept_tool() {
    let shapes = [
        ProfileShape::Flat,
        ProfileShape::Ball,
        ProfileShape::BullNose { corner_radius: 0.4 },
        ProfileShape::Cone {
            half_angle_rad: 0.7,
            flat_radius: 0.1,
        },
        ProfileShape::VTip {
            half_angle_rad: 0.5,
            tip_radius: 0.2,
        },
    ];
    let moves = [
        ([-5., -4., -2.], [8., 7., -2.]),
        ([8., -4., -1.], [-5., 7., -3.]),
        ([1., 2., -3.], [1., 2., 2.]),
        ([-9., 1., -2.], [12., 1., -2.]),
        ([1., -9., -3.], [1., 12., -1.]),
        ([9., 9., -2.], [11., 11., -2.]),
    ];
    for shape in shapes {
        let tool = ToolProfile { radius: 1.3, shape };
        for (a, b) in moves {
            let mut field = Heightfield::new([-3., -2.], [7., 6.], 0.25, 0.);
            field.cut_segment_profile(a, b, &tool);
            for iy in 0..32 {
                for ix in 0..40 {
                    let x = -3. + (ix as f64 + 0.5) * 0.25;
                    let y = -2. + (iy as f64 + 0.5) * 0.25;
                    let dx = b[0] - a[0];
                    let dy = b[1] - a[1];
                    let len2 = dx * dx + dy * dy;
                    let t = if len2 < 1e-12 {
                        0.
                    } else {
                        (((x - a[0]) * dx + (y - a[1]) * dy) / len2).clamp(0., 1.)
                    };
                    let d = (x - a[0] - dx * t).hypot(y - a[1] - dy * t);
                    let z = if len2 < 1e-12 {
                        a[2].min(b[2])
                    } else {
                        a[2] + (b[2] - a[2]) * t
                    };
                    let expected = if d <= tool.radius {
                        (z + tool.offset(d)).min(0.) as f32
                    } else {
                        0.
                    };
                    assert!(
                        (field.sample(x, y) - expected).abs() < 1e-6,
                        "{shape:?}, {a:?} → {b:?} at {x},{y}"
                    );
                }
            }
        }
    }
}

#[test]
fn forward_frames_resume_after_the_previous_complete_steps() {
    let source = source();
    let mut cache = SimulationCache::default();
    let mut run = |count: usize| {
        let mut progress = Vec::new();
        cache
            .simulate_controlled(
                &Program {
                    steps: source.steps[..count].to_vec(),
                },
                [0., 0., -5.],
                [20., 20., 0.],
                &options(),
                &tools(),
                &|| false,
                |done, _| progress.push(done),
            )
            .unwrap();
        progress
    };
    run(600);
    let progress = run(700);
    assert!(
        progress.iter().all(|&done| done >= 599),
        "replayed old steps: {progress:?}"
    );
}
