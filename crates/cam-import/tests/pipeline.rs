//! P4 acceptance: an external `part.dxf` runs the *same* pipeline as first light
//! — import → profile → grbl — producing sound, stable G-code.

use cam_cldata::SpindleDir;
use cam_import::{read_dxf_file, ImportOptions};
use cam_model::{
    Comp, Document, Heights, Lead, Machine, Operation, Plunge, ProfileOp, Setup, Side, Stock, Tool,
    ToolKind,
};
use cam_post::{GrblPost, Post, PostOptions};
use cam_toolpath::{build_job, CancelToken};

fn machine() -> Machine {
    Machine {
        name: "OCS-3018".into(),
        rapid_rate: 2000.0,
        max_spindle_rpm: 10_000.0,
        max_feed: 800.0,
        envelope: cam_model::Envelope::new(
            cam_model::Point3::new(0.0, 0.0, -50.0),
            cam_model::Point3::new(300.0, 180.0, 50.0),
        ),
        safe_z: 5.0,
        tool_change_pos: None,
    }
}

fn fixture_path() -> String {
    format!("{}/tests/fixtures/part.dxf", env!("CARGO_MANIFEST_DIR"))
}

fn profile_op(id: u32, chain: cam_geo::Contour, side: Side) -> Operation {
    Operation::Profile(ProfileOp {
        id,
        tool: 1,
        chain,
        side,
        comp: Comp::Computed,
        depth: 4.0,
        stepdown: 2.0,
        feed: 300.0,
        plunge_feed: 100.0,
        start: None,
        lead_in: Lead::None,
        lead_out: Lead::None,
        lead_overlap: 0.0,
        plunge: Plunge::Straight,
    })
}

/// Import the fixture and build a job: profile the outer boundary (outside) and
/// each hole (inside).
fn import_and_plan() -> (String, Vec<cam_toolpath::Diagnostic>, usize, usize) {
    let import = read_dxf_file(fixture_path(), &ImportOptions::default()).expect("import ok");
    assert_eq!(import.regions.len(), 1, "one region: {:?}", import.warnings);
    let region = &import.regions[0];

    let mut ops = vec![profile_op(0, region.outer().clone(), Side::Outside)];
    for (i, hole) in region.holes().iter().enumerate() {
        ops.push(profile_op(1 + i as u32, hole.clone(), Side::Inside));
    }

    let doc = Document::new(Setup {
        name: "part.dxf".into(),
        heights: Heights::new(5.0, 2.0, 0.0),
        stock: Stock::BoundingBox {
            x_offset: 0.0,
            y_offset: 0.0,
            top: 0.0,
            thickness: 10.0,
        },
        tools: vec![Tool {
            number: 1,
            diameter: 6.0,
            length: 30.0,
            flutes: 2,
            kind: ToolKind::EndMill,
        }],
        operations: ops,
        origin: [0.0, 0.0, 0.0],
        start_offset: None,
    });

    let (program, diags) = build_job(&doc, 1000.0, SpindleDir::Cw, &CancelToken::new());
    let opts = PostOptions {
        program_name: Some("part".into()),
        ..Default::default()
    };
    let nc = GrblPost.post(&program, &machine(), &opts).expect("post ok");
    (nc, diags, region.outer().len(), region.holes().len())
}

#[test]
fn dump() {
    let (nc, diags, outer_pts, holes) = import_and_plan();
    eprintln!("outer_pts={outer_pts} holes={holes} diags={diags:?}");
    println!("{nc}");
}

#[test]
fn import_yields_a_rectangle_with_one_hole() {
    let import = read_dxf_file(fixture_path(), &ImportOptions::default()).unwrap();
    assert_eq!(import.regions.len(), 1);
    let region = &import.regions[0];
    // Outer is the chained 4-line rectangle (area 60×40 = 2400) with a ⌀10 hole.
    assert!((region.outer().area() - 2400.0).abs() < 1.0);
    assert_eq!(region.holes().len(), 1);
    assert!(import.open_chains.is_empty(), "no dangling chains");
}

#[test]
fn pipeline_reports_no_errors() {
    let (_, diags, _, _) = import_and_plan();
    assert!(
        !diags
            .iter()
            .any(|d| d.severity == cam_toolpath::Severity::Error),
        "unexpected errors: {diags:?}"
    );
}

#[test]
fn nc_golden_is_stable() {
    let (nc, _, _, _) = import_and_plan();
    assert_eq!(
        nc,
        include_str!("golden/part.nc"),
        "grbl .nc drifted from golden; regenerate if intended"
    );
}

#[test]
fn gcode_is_semantically_sound() {
    let (nc, _, _, _) = import_and_plan();
    assert_gcode_is_sound(&nc, &machine(), 0.0);
}

/// Parse grbl output and assert the same safety invariants used elsewhere:
/// preamble present, spindle on before the first cut, no lateral rapid below the
/// stock top, every coordinate in the envelope, clean M30 end.
fn assert_gcode_is_sound(gcode: &str, m: &Machine, stock_top: f64) {
    let mut g: Option<u8> = None;
    let (mut x, mut y, mut z) = (f64::NAN, f64::NAN, f64::NAN);
    let mut spindle_on = false;
    let mut spindle_before_cut: Option<bool> = None;
    let (mut units, mut abs, mut wcs, mut ended) = (false, false, false, false);
    let mut last = "";

    for raw in gcode.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('(') {
            continue;
        }
        last = line;
        let (mut had_xy, mut had_coord) = (false, false);
        for tok in line.split_whitespace() {
            let (letter, rest) = tok.split_at(1);
            match letter {
                "G" => match rest {
                    "0" | "1" | "2" | "3" => g = Some(rest.parse().unwrap()),
                    "21" => units = true,
                    "90" => abs = true,
                    "54" | "55" | "56" | "57" | "58" | "59" => wcs = true,
                    _ => {}
                },
                "M" => match rest {
                    "3" | "4" => spindle_on = true,
                    "5" => spindle_on = false,
                    "30" => ended = true,
                    _ => {}
                },
                "X" => {
                    x = rest.parse().unwrap();
                    had_xy = true;
                    had_coord = true;
                }
                "Y" => {
                    y = rest.parse().unwrap();
                    had_xy = true;
                    had_coord = true;
                }
                "Z" => {
                    z = rest.parse().unwrap();
                    had_coord = true;
                }
                _ => {}
            }
        }
        if matches!(g, Some(1) | Some(2) | Some(3)) && had_coord && spindle_before_cut.is_none() {
            spindle_before_cut = Some(spindle_on);
        }
        if had_coord && x.is_finite() && y.is_finite() && z.is_finite() {
            assert!(m.envelope.contains(x, y, z), "outside envelope: {line}");
        }
        if g == Some(0) && had_xy {
            assert!(
                z >= stock_top - 1e-9,
                "lateral rapid below stock top: {line}"
            );
        }
    }

    assert!(units && abs && wcs, "preamble must set G21/G90/work offset");
    assert_eq!(spindle_before_cut, Some(true), "spindle on before cut");
    assert!(!spindle_on, "spindle off at end");
    assert!(ended && last == "M30", "clean M30 end");
}
