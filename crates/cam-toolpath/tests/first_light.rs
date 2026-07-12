//! P3 "first light": an in-code rectangle-with-hole flows all the way through —
//! `cam-geo` offset → `cam-cldata` → `cam-post` grbl — into a real `.nc`.
//!
//! Proven three ways: no error diagnostics, a **golden `.nc`** (byte-stable), a
//! **semantic check** on the emitted G-code, and an **ASCII backplot** of the
//! cutting motions (a visual golden).

use cam_cldata::{MoveKind, Program, SpindleDir, Step};
use cam_geo::{Contour, Point};
use cam_model::{
    Comp, Document, Heights, Machine, Operation, ProfileOp, Setup, Side, Stock, Tool, ToolKind,
};
use cam_post::{GrblPost, Post, PostOptions};
use cam_toolpath::{build_job, CancelToken, JobEnv, ProfileStrategy, Strategy};

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Contour {
    Contour::new(vec![
        Point::new(x0, y0),
        Point::new(x1, y0),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ])
}

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

/// A 60×40 rectangular part (at 10,10) with a 10×10 hole. Two profile ops with a
/// ⌀6 end mill: cut the outside free, then open the hole to size.
fn document() -> Document {
    let tool = Tool {
        number: 1,
        diameter: 6.0,
        flutes: 2,
        kind: ToolKind::EndMill,
    };
    let outer = ProfileOp {
        id: 0,
        tool: 1,
        chain: rect(10.0, 10.0, 70.0, 50.0),
        side: Side::Outside,
        comp: Comp::Computed,
        depth: -4.0,
        stepdown: 2.0,
        feed: 300.0,
        plunge_feed: 100.0,
    };
    let hole = ProfileOp {
        id: 1,
        tool: 1,
        chain: rect(35.0, 25.0, 45.0, 35.0),
        side: Side::Inside,
        comp: Comp::Computed,
        depth: -4.0,
        stepdown: 2.0,
        feed: 300.0,
        plunge_feed: 100.0,
    };
    Document::new(Setup {
        name: "first light".into(),
        heights: Heights::new(5.0, 2.0, 0.0),
        stock: Stock::Box {
            min: [0.0, 0.0, -10.0],
            max: [80.0, 60.0, 0.0],
        },
        tools: vec![tool],
        operations: vec![Operation::Profile(outer), Operation::Profile(hole)],
    })
}

fn plan_and_post() -> (Program, String, Vec<cam_toolpath::Diagnostic>) {
    let doc = document();
    let (program, diags) = build_job(&doc, 1000.0, SpindleDir::Cw, &CancelToken::new());
    let opts = PostOptions {
        program_name: Some("first_light".into()),
        ..Default::default()
    };
    let nc = GrblPost.post(&program, &machine(), &opts).expect("post ok");
    (program, nc, diags)
}

#[test]
fn dump() {
    let (program, nc, diags) = plan_and_post();
    println!("\n--- diagnostics: {diags:?}\n");
    println!("--- gcode ---\n{nc}");
    println!("--- backplot ---\n{}", ascii_backplot(&program, 54, 22));
}

// ---------------------------------------------------------------------------
// First light: the full pipeline produces sound, stable G-code
// ---------------------------------------------------------------------------

#[test]
fn pipeline_reports_no_errors() {
    let (_, _, diags) = plan_and_post();
    assert!(
        !diags
            .iter()
            .any(|d| d.severity == cam_toolpath::Severity::Error),
        "unexpected error diagnostics: {diags:?}"
    );
}

#[test]
fn nc_golden_is_stable() {
    let (_, nc, _) = plan_and_post();
    assert_eq!(
        nc,
        include_str!("golden/first_light.nc"),
        "grbl .nc drifted from golden; regenerate if intended"
    );
}

#[test]
fn backplot_golden_is_stable() {
    let (program, _, _) = plan_and_post();
    let got = ascii_backplot(&program, 54, 22);
    let want = include_str!("golden/first_light.txt").trim_end_matches('\n');
    assert_eq!(got, want, "backplot drifted from golden\n{got}");
}

#[test]
fn gcode_is_semantically_sound() {
    let (_, nc, _) = plan_and_post();
    assert_gcode_is_sound(&nc, &machine(), 0.0);
}

// ---------------------------------------------------------------------------
// Strategy behaviour: diagnostics and cancellation
// ---------------------------------------------------------------------------

#[test]
fn tool_too_large_for_hole_reports_error() {
    // A ⌀12 tool cannot open a 10 mm hole — the inside offset consumes it.
    let tool = Tool {
        number: 1,
        diameter: 12.0,
        flutes: 2,
        kind: ToolKind::EndMill,
    };
    let op = ProfileOp {
        id: 0,
        tool: 1,
        chain: rect(35.0, 25.0, 45.0, 35.0),
        side: Side::Inside,
        comp: Comp::Computed,
        depth: -4.0,
        stepdown: 2.0,
        feed: 300.0,
        plunge_feed: 100.0,
    };
    let tools = [tool];
    let env = JobEnv {
        heights: Heights::new(5.0, 2.0, 0.0),
        tools: &tools,
    };
    let result = ProfileStrategy::new(op).compute(&env, &CancelToken::new());
    assert!(result.has_errors(), "expected a tool-too-large error");
    assert!(result.program.is_empty(), "no motions on error");
}

#[test]
fn cancellation_stops_before_emitting() {
    let tool = Tool {
        number: 1,
        diameter: 6.0,
        flutes: 2,
        kind: ToolKind::EndMill,
    };
    let op = ProfileOp {
        id: 0,
        tool: 1,
        chain: rect(10.0, 10.0, 70.0, 50.0),
        side: Side::Outside,
        comp: Comp::Computed,
        depth: -4.0,
        stepdown: 2.0,
        feed: 300.0,
        plunge_feed: 100.0,
    };
    let tools = [tool];
    let env = JobEnv {
        heights: Heights::new(5.0, 2.0, 0.0),
        tools: &tools,
    };
    let cancel = CancelToken::new();
    cancel.cancel();
    let result = ProfileStrategy::new(op).compute(&env, &cancel);
    assert!(result.cancelled, "should report cancellation");
    assert!(result.program.is_empty(), "no motions when cancelled");
}

/// Parse grbl output (modal, absolute) and assert safety/correctness invariants:
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
    assert_eq!(
        spindle_before_cut,
        Some(true),
        "spindle must be on before cutting"
    );
    assert!(!spindle_on, "spindle must be off at end");
    assert!(ended, "must contain M30");
    assert_eq!(last, "M30", "M30 must be the final line");
}

/// Render the XY cutting motions to an ASCII grid (Y up). Deterministic, so it
/// doubles as a visual golden.
fn ascii_backplot(prog: &Program, w: usize, h: usize) -> String {
    let mut segs: Vec<((f64, f64), (f64, f64))> = Vec::new();
    let mut cur: Option<(f64, f64)> = None;
    for step in prog.steps() {
        let (to, cutting) = match step {
            Step::Rapid { to, .. } => (Some((to.x, to.y)), false),
            Step::Linear { to, tag, .. } => (Some((to.x, to.y)), tag.kind == MoveKind::Cutting),
            _ => (None, false),
        };
        if let Some(p) = to {
            if cutting {
                if let Some(prev) = cur {
                    segs.push((prev, p));
                }
            }
            cur = Some(p);
        }
    }

    let pts = segs.iter().flat_map(|(a, b)| [*a, *b]);
    let (mut minx, mut miny, mut maxx, mut maxy) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for (x, y) in pts {
        minx = minx.min(x);
        miny = miny.min(y);
        maxx = maxx.max(x);
        maxy = maxy.max(y);
    }
    let (sx, sy) = (maxx - minx, maxy - miny);

    let mut grid = vec![vec![b' '; w]; h];
    let map = |x: f64, y: f64| -> (usize, usize) {
        let col = if sx > 0.0 {
            ((x - minx) / sx * (w - 1) as f64).round() as usize
        } else {
            0
        };
        let row = if sy > 0.0 {
            ((maxy - y) / sy * (h - 1) as f64).round() as usize
        } else {
            0
        };
        (col.min(w - 1), row.min(h - 1))
    };
    for (a, b) in &segs {
        let (c0, r0) = map(a.0, a.1);
        let (c1, r1) = map(b.0, b.1);
        let steps = (c0.abs_diff(c1)).max(r0.abs_diff(r1)).max(1);
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            let x = a.0 + (b.0 - a.0) * t;
            let y = a.1 + (b.1 - a.1) * t;
            let (c, r) = map(x, y);
            grid[r][c] = b'#';
        }
    }
    grid.into_iter()
        .map(|row| String::from_utf8(row).unwrap())
        .collect::<Vec<_>>()
        .join("\n")
}
