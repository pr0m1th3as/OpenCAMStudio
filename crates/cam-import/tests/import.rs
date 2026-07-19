
#[test]
fn acadrust_arc_angles_arrive_in_degrees_not_radians() {
    // Regression: acadrust reports arc angles in **radians**, while `dxf::Entity::Arc`
    // carries degrees (as the DXF file stores them) and every downstream consumer
    // converts from degrees. Passing them through unconverted turned a 0°→90° quarter
    // arc into a 0°→1.57° sliver — silently wrong on every arc opened from a real
    // file, while the in-crate ASCII reader (used by most tests) stayed correct.
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/quarter_arc.dxf");
    let (entities, _) = cam_import::read_cad_entities(path).expect("fixture reads");
    match entities.as_slice() {
        [cam_import::dxf::Entity::Arc {
            center,
            radius,
            start_deg,
            end_deg,
        }] => {
            assert_eq!(*center, (0.0, 0.0));
            assert!((*radius - 10.0).abs() < 1e-9);
            assert!(start_deg.abs() < 1e-6, "start {start_deg}");
            assert!((*end_deg - 90.0).abs() < 1e-6, "end {end_deg} (1.571 = radians)");
        }
        other => panic!("expected exactly one arc, got {other:?}"),
    }
}

#[test]
fn a_real_file_arc_sweeps_its_full_angle() {
    // The consequence at the geometry level: the flattened chain must span the whole
    // quarter, from (10,0) round to (0,10) — not a stub near the start.
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/quarter_arc.dxf");
    let im = cam_import::read_cad_file(path, &Default::default()).expect("fixture imports");
    let chain = im.open_chains.first().expect("an open arc chain");
    let pts = chain.points();
    let first = pts.first().unwrap();
    let last = pts.last().unwrap();
    assert!((first.x - 10.0).abs() < 1e-6 && first.y.abs() < 1e-6, "{first:?}");
    assert!(last.x.abs() < 1e-6 && (last.y - 10.0).abs() < 1e-6, "{last:?}");
}
